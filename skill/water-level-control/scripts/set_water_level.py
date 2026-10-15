#!/usr/bin/env python3
"""一键设定目标水位(抗超调 / 抗振荡优化版)。

相比旧版的改进:
1. [增益] 弃用过于激进的 Kp=3.0 + 高 Ki 线性公式(超调/振荡主因之一),
   改用 SKILL.md 中现场实测验证的分段稳定参数(生产验证稳态误差 <±2mm):
       50-70mm    Kp=1.8  Ki=0.30  Kd=0
       70-100mm   Kp=1.5  Ki=0.28  Kd=0
       100-150mm  Kp=1.4  Ki=0.26  Kd=0
2. [斜坡] 设定值不再一次性阶跃写入 PLC(纯滞后 τ 内误差持续累积 → 积分饱和
   → 超调的另一主因)。改为:
   - 起始目标 = 当前实测水位(误差≈0, 无扰切换, 积分项保持);
   - 以 ≤5mm/步 逐步逼近最终目标;
   - 目标相对实测水位的超前量限制在 ±10mm 内, 误差有界 → PID 输出不饱和。
   上升方向靠重力自流(被动), 超前限幅尤其重要: 防止目标跑得太远导致阀
   全关、水位猛涨、纯滞后内刹不住车而冲过目标。
3. [监视] 写入后轮询实测水位, 连续 3 次进入 ±3mm 稳定带才报成功;
   传感器故障立即中止, 报警(如低液位保护)打印警告但不中止。
4. [健壮性] 直接 import 同目录 water_control_cli 复用其 Modbus 读写/标定/
   状态字函数, 替代原 subprocess + 文本解析(旧方式脆弱且无法读状态位)。

用法:
    python3 set_water_level.py <target_mm> [-v]
    -v/--verbose: 打印斜坡与整定过程细节(默认静默, 只报最终结果)
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import water_control_cli as wc  # noqa: E402  同目录复用, 目标设备无 pip

# 支持的控制范围: 50-150mm。超出此区间拒绝, 不执行任何 PLC 写入。
# PID 增益仅在此窗口内现场整定过; 区间外控制不稳定且可能无意义。
SUPPORTED_MIN, SUPPORTED_MAX = 50.0, 150.0

# 分段 PID 参数(现场实测整定, SKILL.md "Segment-optimized gains")。
# 目标越高增益越小: 高水位段对象响应更快、更易振荡。
# 表项: (区间上界 mm, Kp, Ki); Kd 全程为 0(慢积分对象, 微分收益低且放大噪声)。
SEGMENTS = [
    (70.0, 1.8, 0.30),          # 低段 50-70mm
    (100.0, 1.5, 0.28),         # 中段 70-100mm
    (float("inf"), 1.4, 0.26),  # 高段 100-150mm
]

RAMP_STEP_MM = 5.0         # 每步目标最多推进 5mm
RAMP_MAX_LEAD_MM = 10.0    # 目标相对实测水位的最大超前量(误差限幅, 防积分饱和)
RAMP_INTERVAL_S = 3.0      # 每步间隔, 给水位留出响应时间
RAMP_TIMEOUT_S = 20 * 60   # 斜坡总超时(上升为被动自流, 可能很慢)
SETTLE_TOL_MM = 3.0        # 稳定带: |实测-目标| ≤ 3mm
SETTLE_HITS = 3            # 需连续 3 次采样在带内才算稳定
SETTLE_TIMEOUT_S = 5 * 60  # 稳定判定超时
POLL_INTERVAL_S = 2.0      # 稳定监视轮询间隔


def gains_for(mm: float):
    """按目标水位返回分段整定的 (Kp, Ki, Kd)。"""
    for upper, kp, ki in SEGMENTS:
        if mm <= upper:
            return kp, ki, 0.0
    return SEGMENTS[-1][1], SEGMENTS[-1][2], 0.0  # 不可达, 兜底


def read_level_mm(client, config) -> float:
    """读实测水位(FC04 rLevel)并换算为 mm。"""
    pct = wc.read_input_real(client, wc.IREG_LEVEL, config["slave_id"])
    return wc.pct_to_mm(pct, config)


class HealthMonitor:
    """轮询状态字: 传感器故障立即中止; 报警只警告一次(低液位保护属预期行为)。"""

    def __init__(self, client, config, verbose=False):
        self.client = client
        self.config = config
        self.verbose = verbose
        self._alarm_warned = False

    def check(self):
        sw = wc.read_status_word(self.client, self.config["slave_id"])
        if sw & wc.ST_SENSOR_FAULT:
            raise RuntimeError("传感器故障 (bSensorFault), 已中止; 请检查 4-20mA 接线")
        if (sw & wc.ST_ALARM) and not self._alarm_warned:
            self._alarm_warned = True
            print("警告: PLC 报警位 (bAlarm) 置位 — 若为低液位停泵保护属预期行为",
                  file=sys.stderr)


def apply_pid(client, config, kp, ki, kd):
    """写 Kp/Ki/Kd 并置 bPidTune 使能位(与 CLI set-pid 相同的寄存器序列)。"""
    sid = config["slave_id"]
    wc.write_real(client, wc.REG_KP, kp, sid)
    wc.write_real(client, wc.REG_KI, ki, sid)
    wc.write_real(client, wc.REG_KD, kd, sid)
    wc.set_control_bit(client, wc.CTL_PID_TUNE, True, sid)


def write_target_mm(client, config, mm):
    wc.write_real(client, wc.REG_TARGET_LEVEL, wc.mm_to_pct(mm, config),
                  config["slave_id"])


def ramp_to(client, config, goal, health, verbose):
    """斜坡逼近: 从实测水位出发, 小步推进目标, 限制超前量, 直至写入最终 goal。"""
    level = read_level_mm(client, config)
    target = level                     # 无扰起点: 误差≈0, 积分项保持原值
    write_target_mm(client, config, target)
    direction = 1.0 if goal >= target else -1.0
    t0 = time.time()

    while abs(goal - target) > 0.5:
        if time.time() - t0 > RAMP_TIMEOUT_S:
            raise RuntimeError(
                f"斜坡超时 ({RAMP_TIMEOUT_S // 60} min): 目标 {target:.1f}mm 未到 {goal:g}mm")
        health.check()
        level = read_level_mm(client, config)
        step = min(RAMP_STEP_MM, abs(goal - target))
        target += direction * step
        # 超前限幅: 目标与实测水位的差(=PID 误差)不超过 MAX_LEAD, 防输出饱和
        if direction > 0:
            target = min(target, level + RAMP_MAX_LEAD_MM, goal)
        else:
            target = max(target, level - RAMP_MAX_LEAD_MM, goal)
        write_target_mm(client, config, target)
        if verbose:
            print(f"  ramp: level={level:6.1f}mm  target->{target:6.1f}mm  "
                  f"(goal {goal:g}mm)", file=sys.stderr)
        time.sleep(RAMP_INTERVAL_S)

    write_target_mm(client, config, goal)  # 确保最终目标精确等于 goal
    if verbose:
        print(f"  ramp done: target={goal:g}mm  ({time.time() - t0:.0f}s)",
              file=sys.stderr)


def wait_settled(client, config, goal, health, verbose):
    """轮询实测水位, 连续 SETTLE_HITS 次进入 ±SETTLE_TOL_MM 判稳。

    返回 (是否稳定, 耗时 s, 最后实测 mm)。
    """
    hits = 0
    level = read_level_mm(client, config)
    t0 = time.time()
    while time.time() - t0 < SETTLE_TIMEOUT_S:
        health.check()
        level = read_level_mm(client, config)
        if abs(level - goal) <= SETTLE_TOL_MM:
            hits += 1
            if hits >= SETTLE_HITS:
                return True, time.time() - t0, level
        else:
            hits = 0
        if verbose:
            print(f"  settle: level={level:6.1f}mm  err={level - goal:+.1f}mm  "
                  f"hits={hits}/{SETTLE_HITS}", file=sys.stderr)
        time.sleep(POLL_INTERVAL_S)
    return False, time.time() - t0, level


def main():
    ap = argparse.ArgumentParser(
        prog="set_water_level",
        description="设定目标水位 (mm), 自动匹配分段 PID 并以斜坡方式逼近, 抑制超调")
    ap.add_argument("target_mm", type=float,
                    help=f"目标水位 mm ({SUPPORTED_MIN:g}-{SUPPORTED_MAX:g})")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="打印斜坡/整定过程细节 (stderr)")
    args = ap.parse_args()

    if not (SUPPORTED_MIN <= args.target_mm <= SUPPORTED_MAX):
        print(f"Error: target {args.target_mm:g} mm out of supported range "
              f"({SUPPORTED_MIN:g}-{SUPPORTED_MAX:g} mm). "
              f"water-level-control only supports {SUPPORTED_MIN:g}-{SUPPORTED_MAX:g} mm.")
        sys.exit(1)

    config = wc.load_config()
    client = wc.connect_plc(config)
    try:
        health = HealthMonitor(client, config, args.verbose)
        health.check()

        # 目标水位只在自动模式下有意义, 确保 bAuto=1
        wc.set_control_bit(client, wc.CTL_AUTO, True, config["slave_id"])

        kp, ki, kd = gains_for(args.target_mm)
        apply_pid(client, config, kp, ki, kd)
        if args.verbose:
            print(f"  pid: Kp={kp} Ki={ki} Kd={kd} (segment for {args.target_mm:g}mm)",
                  file=sys.stderr)

        ramp_to(client, config, args.target_mm, health, args.verbose)
        ok, elapsed, level = wait_settled(client, config, args.target_mm,
                                          health, args.verbose)
        if ok:
            print(f"✅ 已成功将水位目标设置为 {args.target_mm:g} mm "
                  f"(当前 {level:.1f} mm, 稳定耗时 {elapsed:.0f}s)")
        else:
            print(f"⚠️ 目标 {args.target_mm:g} mm 已写入, 当前 {level:.1f} mm, "
                  f"{SETTLE_TIMEOUT_S // 60} min 内未进入 ±{SETTLE_TOL_MM:g}mm 稳定带; "
                  f"请用 status 继续观察")
            sys.exit(2)
    except RuntimeError as e:
        print(f"❌ 设置失败: {e}")
        sys.exit(1)
    except Exception:
        print("❌ 设置失败，请检查PLC连接")
        sys.exit(1)
    finally:
        client.close()


if __name__ == "__main__":
    main()
