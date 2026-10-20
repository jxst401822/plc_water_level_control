#!/usr/bin/env python3
"""阶跃试验工具 — 实测对象增益 K 与纯滞后 τ, 计算推荐 PID 增益。

背景: 文档中两套"现场验证"增益(Kp=3.0/Ki=0.40-0.49 与 Kp=1.8/Ki=0.30)
互相矛盾、出处不可考。本工具用经典阶跃响应法(反应曲线法)实测:
  - K  [%/s per %]: 阀开度每变化 1%, 水位变化速率改变多少 (对象增益)
  - τ  [s]:         阀动作后水位开始响应的延迟 (纯滞后, 切线法求取)
  - d0 [%/s]:       阀全关时的自然入流速率 (重力自流扰动)
再按 SIMC/IMC 整定公式 (积分+纯滞后对象) 给出推荐增益:
  Kp = 1/(K·(λ+τ)),  Ti = 4·(λ+τ),  Ki = Kp/Ti,  Kd = 0
  λ 为期望闭环时间常数, 输出 λ=τ (标准) 与 λ=2τ (更鲁棒) 两档。

试验流程 (自动执行, 全程约 3-5 分钟):
  1. 检查液位在安全窗口 (55-140mm), 保存当前模式/阀开度
  2. 切手动模式, 开泵, 阀置 u0
     u0 默认取 PLC 自学习的平衡开度 u_eq (get-learn 同源), 退化为当前阀指令
  3. **调零**: 短探针测基线斜率, 若液位还在漂(没停在平衡点)就微调 u0 直到接近静止
  4. 基线记录 60s: 固定阀开度下拟合基线斜率 s0
  5. 阀阶跃 +Δu (默认 +10%, 多抽水方向), 继续记录 (时长按液位离守卫的余量自动截断)
  6. 恢复现场 (阀回 u0, 按原模式还原 auto/manual 与泵位)
  7. 切线法求 τ, 斜率差求 K, 输出推荐增益 + CSV 数据文件
     中止/中断时也会用已采到的数据尽力输出分析

为什么必须调零 (2026-10-19 现场教训): u0 直接取自动模式的瞬时阀指令(33.5%),
比真实平衡开度(≈23%)高了 10%, 整个基线液位以 -0.46mm/s 一路排到泵物理下限,
阶跃后的斜率被"低液位区排水加速"污染, 算出的 K=0.18 比工作带真实 K≈0.02~0.03
大 8 倍 —— 用这样的 K 定增益会把闭环整定得过激。

⚠️ 安全: 手动模式下 PLC 的 HH/LL 联锁被手动阀开度覆盖 (ST 第 5 段),
本工具自带液位守卫: 试验中液位越出 [45, 150]mm 或传感器故障立即中止并恢复。
Ctrl+C 同样会执行恢复。

用法:
    python3 step_test.py                    # 默认: u0=PLC 学到的 u_eq, +10% 阶跃
    python3 step_test.py --step -10         # 关小阀方向 (液位会上升)
    python3 step_test.py --u0 40 --step 15 --baseline 90 --record 240
    python3 step_test.py --no-trim          # 跳过调零 (不推荐)
    python3 step_test.py --no-restore-auto  # 结束后保持手动 (默认还原原模式)

仅依赖标准库 + pymodbus (经 water_control_cli), 目标设备无 pip 也可直接运行。
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import water_control_cli as wc  # noqa: E402  同目录复用, 目标设备无 pip

# ---- 安全守卫 ----
START_MIN_MM, START_MAX_MM = 55.0, 140.0   # 试验允许的起始液位窗口
GUARD_LOW_MM, GUARD_HIGH_MM = 45.0, 150.0  # 试验中越界立即中止 (手动模式无 PLC 联锁!)
PUMP_FLOOR_MM = 33.0                       # 泵物理下限, 仅供提示

# ---- 采样 ----
DEFAULT_INTERVAL_S = 1.0

# ---- 基线调零 (u0 微调) ----
K_PRIOR = 0.025        # 工作带对象增益先验 %/s per %阀 (仅用于换算调零步长, 非实测 K)
TRIM_TOL = 0.06        # 基线斜率容差 %/s (约 0.1mm/s); 超过则继续调零
TRIM_MAX_ITERS = 3
TRIM_MAX_DU = 15.0     # 单次调零最大阀开度修正 %
DEFAULT_PROBE_S = 20.0 # 单次探针时长 s


class AbortTest(Exception):
    pass


def read_level(client, config):
    """返回 (液位%, 液位mm)。"""
    pct = wc.read_input_real(client, wc.IREG_LEVEL, config["slave_id"])
    return pct, wc.pct_to_mm(pct, config)


def read_valve(client, config):
    return wc.read_input_real(client, wc.IREG_VALVE_CMD, config["slave_id"])


def set_manual_valve(client, config, pct):
    wc.write_real(client, wc.REG_MANUAL_VALVE, pct, config["slave_id"])


def read_learned(client, config):
    """读 PLC [G12] 自学习值 (d0 %/s, u_eq %); 读不到返回 (None, None)。"""
    sid = config["slave_id"]
    try:
        d0 = wc.read_input_real(client, wc.IREG_INFLOW, sid)
        ueq = wc.read_input_real(client, wc.IREG_UEQ, sid)
    except Exception:
        return None, None
    if not (-5.0 < d0 < 5.0 and -0.01 < ueq < 100.1):
        return None, None          # 寄存器未映射/全 0/垃圾值
    return d0, ueq


def linfit(xs, ys):
    """最小二乘直线拟合 y = a + s·x, 返回 (a, s)。纯标准库实现。"""
    n = len(xs)
    if n < 2:
        raise AbortTest("数据点不足, 无法拟合")
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx == 0:
        raise AbortTest("时间序列无变化, 无法拟合")
    s = sxy / sxx
    return my - s * mx, s


def record_phase(client, config, duration, interval, phase, samples,
                 t0, guards=True, alarm_warned=[False]):
    """按 interval 采样 level/valve/status 至 samples, 持续 duration 秒。

    guards=True 时执行液位/故障守卫, 越界抛 AbortTest。
    返回该阶段结束的相对时刻。
    """
    t_end = duration
    next_poll = 0.0
    while True:
        t_rel = time.time() - t0
        if t_rel >= t_end:
            return t_rel
        if t_rel >= next_poll:
            pct, mm = read_level(client, config)
            valve = read_valve(client, config)
            sw = wc.read_status_word(client, config["slave_id"])
            samples.append({
                "t": t_rel,
                "iso": datetime.now().isoformat(timespec="seconds"),
                "phase": phase,
                "mm": mm,
                "pct": pct,
                "valve": valve,
                "status": f"0x{sw:04X}",
            })
            if guards:
                if sw & wc.ST_SENSOR_FAULT:
                    raise AbortTest("传感器故障 (bSensorFault), 中止试验")
                if mm <= GUARD_LOW_MM:
                    raise AbortTest(
                        f"液位 {mm:.1f}mm 低于守卫 {GUARD_LOW_MM:g}mm, 中止 "
                        f"(泵物理下限约 {PUMP_FLOOR_MM:g}mm)")
                if mm >= GUARD_HIGH_MM:
                    raise AbortTest(
                        f"液位 {mm:.1f}mm 高于守卫 {GUARD_HIGH_MM:g}mm, 中止")
                if (sw & wc.ST_ALARM) and not alarm_warned[0]:
                    alarm_warned[0] = True
                    print("  警告: PLC 报警位置位 (继续试验)", file=sys.stderr)
            next_poll = t_rel + interval
            elapsed_int = int(t_rel)
            if elapsed_int % 15 == 0 and elapsed_int > 0 and phase != "prep":
                print(f"  [{phase}] {elapsed_int}s/{int(t_end)}s  "
                      f"level={mm:6.1f}mm  valve={valve:5.1f}%")
        time.sleep(min(0.2, max(0.0, next_poll - (time.time() - t0))))


def analyze(samples, t_step, u0, u1, learned_d0=None):
    """切线法: 基线斜率 s0, 阶跃后斜率 s1, 交点求 τ, 斜率差求 K。"""
    base = [s for s in samples if s["phase"] == "baseline"]
    post = [s for s in samples if s["phase"] == "poststep"]
    if len(base) < 5 or len(post) < 5:
        raise AbortTest("采样点过少, 无法分析")

    a0, s0 = linfit([s["t"] for s in base], [s["pct"] for s in base])
    # 阶跃后取后 50% 窗口拟合稳定斜率 (前段仍受纯滞后/过渡影响)
    half = len(post) // 2
    tail = post[half:] if len(post) - half >= 5 else post
    a1, s1 = linfit([s["t"] for s in tail], [s["pct"] for s in tail])

    du = u1 - u0
    ds = s0 - s1                     # 抽水侧: 阀开大 -> 斜率更负 -> ds > 0
    if abs(ds) < 0.005:
        raise AbortTest(
            f"阶跃前后斜率差过小 ({ds:+.4f} %/s), K 不可辨; "
            f"请加大 --step 或延长 --record 后重试")
    K = ds / du                      # %/s per %阀
    if K <= 0:
        raise AbortTest(
            f"计算得 K={K:.4f} ≤ 0, 与抽水侧对象方向矛盾; "
            f"请检查阶跃方向/数据 (可用 --step 反向重试)")

    # 切线法: 基线直线与阶跃后渐近直线的交点 = 响应"表观起点"
    t_star = (a1 - a0) / (s0 - s1)
    tau = t_star - t_step
    interval_hint = DEFAULT_INTERVAL_S
    if tau < 0:
        print(f"  警告: 切线交点早于阶跃时刻 (τ={tau:.1f}s), "
              f"按一个采样周期取 τ={interval_hint:g}s", file=sys.stderr)
        tau = interval_hint
    d0 = s0 + K * u0                 # 阀全关时的自然入流速率 %/s

    recs = []
    for name, lam in (("λ=τ (标准)", tau), ("λ=2τ (鲁棒)", 2 * tau)):
        kp = 1.0 / (K * (lam + tau))
        ti = 4.0 * (lam + tau)
        recs.append((name, kp, kp / ti, ti))

    warn = []
    exc_base = max(s["mm"] for s in base) - min(s["mm"] for s in base)
    exc_post = max(s["mm"] for s in post) - min(s["mm"] for s in post)
    if exc_base > 15.0:
        warn.append(
            f"基线液位漂移 {exc_base:.1f}mm: u0 偏离平衡开度, 整个基线在排水, "
            f"K 会被液位相关非线性带偏 (别拿本次结果定全域增益)")
    if exc_post > 25.0:
        warn.append(
            f"阶跃后液位漂移 {exc_post:.1f}mm: 记录窗口内液位跨越过大, "
            f"斜率取自非线性区, 建议缩短 --record 或减小 --step 重测")
    if learned_d0 and learned_d0 > 0.02 and d0 > 2.0 * learned_d0:
        warn.append(
            f"外推 d0={d0:+.3f} %/s 远大于 PLC 自学习的 {learned_d0:+.3f} %/s: "
            f"同一阀开度区间内近似成立, 但外推到阀全关已失真 -> "
            f"本次 K 只能用于 u0={u0:.0f}% 附近的液位区间")
    return {"s0": s0, "s1": s1, "K": K, "tau": tau, "d0": d0,
            "recs": recs, "warn": warn}


def print_report(r, u0, u1):
    print("\n========== 阶跃试验结果 ==========")
    print(f"阀阶跃:        {u0:.1f}% -> {u1:.1f}%  (Δu = {u1 - u0:+.1f}%)")
    print(f"基线斜率 s0:   {r['s0']:+.4f} %/s")
    print(f"阶跃后斜率 s1: {r['s1']:+.4f} %/s")
    print(f"对象增益 K:    {r['K']:.5f} %/s per %阀")
    print(f"纯滞后 τ:      {r['tau']:.1f} s   (切线法)")
    print(f"自然入流 d0:   {r['d0']:+.4f} %/s (阀全关时液位上升速率)")
    print("\n推荐增益 (SIMC/IMC, 积分+纯滞后对象, Kd=0):")
    for name, kp, ki, ti in r["recs"]:
        print(f"  {name:<14} Kp={kp:6.2f}  Ki={ki:7.4f}  (Ti={ti:6.1f}s)")
    kp, ki = r["recs"][0][1], r["recs"][0][2]
    print(f"\n现场应用 (先用 λ=2τ 鲁棒档验证, 响应过慢再换 λ=τ 档):")
    kp2, ki2 = r["recs"][1][1], r["recs"][1][2]
    print(f"  python3 water_control_cli.py set-pid {kp2:.2f} {ki2:.4f} 0")
    for w in r.get("warn", []):
        print(f"  ⚠️ {w}", file=sys.stderr)
    print("观察无振荡后, 可将实测值回填: st/PLC_PRG_improved.st 默认增益、"
          "set_water_level.py 分段表、SKILL.md 文档。")
    # 与两套文档值的对照
    print(f"\n对照文档争议值: Kp=3.0/Ki≈0.45 vs Kp=1.8/Ki=0.30 -> "
          f"实测推荐 Kp={kp:.2f} (λ=τ) / {kp2:.2f} (λ=2τ)")


def write_csv(samples, path):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["t", "iso", "phase",
                                          "mm", "pct", "valve", "status"])
        w.writeheader()
        for s in samples:
            w.writerow({**s, "mm": f"{s['mm']:.2f}", "pct": f"{s['pct']:.2f}",
                        "valve": f"{s['valve']:.2f}"})


def try_analyze(samples, t_step, u0, u1, learned_d0=None):
    """中止/中断时也用已采到的数据尽力分析 (样本可能不完整)。"""
    if t_step is None or u0 is None or u1 is None:
        print("  中止发生在阶跃之前, 无 K/τ 可算", file=sys.stderr)
        return
    print("\n--- 用已采到的数据尽力分析 (样本不完整, 结论仅供参考) ---")
    try:
        print_report(analyze(samples, t_step, u0, u1, learned_d0), u0, u1)
    except Exception as ae:
        print(f"  数据不足以分析: {ae}", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(
        prog="step_test",
        description="阀阶跃试验: 实测 K/τ/d0 并输出 SIMC 推荐 PID 增益")
    ap.add_argument("--u0", type=float, default=None,
                    help="基线阀开度 %% (默认=PLC 自学习的 u_eq, 读不到则用当前阀指令)")
    ap.add_argument("--step", type=float, default=10.0,
                    help="阶跃幅度 %% (默认 +10=开大阀多抽水; 可为负=关小阀)")
    ap.add_argument("--baseline", type=float, default=60.0,
                    help="基线记录时长 s (默认 60)")
    ap.add_argument("--record", type=float, default=120.0,
                    help="阶跃后记录时长 s (默认 120, 会按离液位守卫的余量自动截短)")
    ap.add_argument("--probe", type=float, default=DEFAULT_PROBE_S,
                    help="调零探针时长 s (默认 %g)" % DEFAULT_PROBE_S)
    ap.add_argument("--no-trim", dest="trim", action="store_false",
                    help="跳过 u0 调零 (不推荐: 基线在漂时测出的 K 不可信)")
    ap.add_argument("--interval", type=float, default=DEFAULT_INTERVAL_S,
                    help="采样间隔 s (默认 1.0)")
    ap.add_argument("--out", type=str, default=None,
                    help="CSV 输出路径 (默认 ./step_test_<时间戳>.csv)")
    ap.add_argument("--no-restore-auto", action="store_true",
                    help="结束后保持手动模式 (默认还原试验前的 auto/manual)")
    args = ap.parse_args()

    if abs(args.step) < 3.0:
        print("Error: |step| 至少 3%, 否则斜率差可能被噪声淹没")
        sys.exit(1)

    config = wc.load_config()
    client = wc.connect_plc(config)
    sid = config["slave_id"]
    samples = []
    # 预初始化, 保证早期 Ctrl+C 时恢复逻辑不引用未绑定变量
    u0 = None
    u1 = None
    t_step = None
    was_auto = True
    was_manual_pump = False
    learned_d0 = None

    def restore():
        """尽力恢复现场: 阀回 u0, 按原模式还原。u0 未知则跳过。"""
        if u0 is None:
            return
        try:
            set_manual_valve(client, config, u0)
            if was_auto and not args.no_restore_auto:
                wc.set_control_bit(client, wc.CTL_AUTO, True, sid)
                wc.set_control_bit(client, wc.CTL_MANUAL_PUMP,
                                   was_manual_pump, sid)
            print(f"现场已恢复 (阀回 {u0:.1f}%, 模式还原)", file=sys.stderr)
        except Exception as re:
            print(f"⚠️ 恢复失败, 请手动检查 PLC 状态: {re}", file=sys.stderr)

    try:
        # ---- 前置检查与状态保存 ----
        sw = wc.read_status_word(client, sid)
        if sw & wc.ST_SENSOR_FAULT:
            print("Error: 传感器故障中, 拒绝试验"); sys.exit(1)
        was_auto = bool(sw & wc.ST_AUTO)
        # 手动泵指令位在控制字 (保持寄存器 Device_10 bit1), 不在状态字里
        cw = wc.read_control_word(client, sid)
        was_manual_pump = bool(cw & wc.CTL_MANUAL_PUMP)
        target_pct = wc.read_real(client, wc.REG_TARGET_LEVEL, sid)

        pct0, mm0 = read_level(client, config)
        if not (START_MIN_MM <= mm0 <= START_MAX_MM):
            print(f"Error: 当前液位 {mm0:.1f}mm 不在试验窗口 "
                  f"[{START_MIN_MM:g}, {START_MAX_MM:g}]mm 内; "
                  f"请先调到窗口中部 (约 90-100mm) 再试验")
            sys.exit(1)

        learned_d0, learned_ueq = read_learned(client, config)
        if learned_d0 is not None:
            print(f"PLC 自学习值: d0={learned_d0:+.3f} %/s  u_eq={learned_ueq:.1f}%")

        if args.u0 is not None:
            u0 = min(90.0, max(0.0, args.u0))
            u0_src = "命令行指定"
        elif learned_ueq is not None and 5.0 <= learned_ueq <= 60.0:
            u0 = learned_ueq
            u0_src = "PLC 自学习 u_eq"
        else:
            u0 = min(60.0, max(10.0, read_valve(client, config)))
            u0_src = "当前阀指令(自学习值不可用)"
        print(f"基线阀开度 u0 = {u0:.1f}%  <- {u0_src}")

        u1 = min(100.0, max(0.0, u0 + args.step))
        if abs(u1 - u0) < 3.0:
            print(f"Error: u0={u0:.1f}% 时加 step={args.step:+.1f}% 会顶到阀开度边界, "
                  f"实际阶跃量只有 {u1 - u0:+.1f}% (<3%); 请改 --u0 或反号 --step")
            sys.exit(1)
        du_step = u1 - u0

        # 预估方向提示
        if args.step > 0:
            print(f"起始液位 {mm0:.1f}mm, 阶跃 {u0:.1f}%->{u1:.1f}% (开大阀, 液位将下降)")
            if mm0 < 75.0:
                print("警告: 液位偏低且将下降, 注意 45mm 守卫; 必要时 --step 取负")
        else:
            print(f"起始液位 {mm0:.1f}mm, 阶跃 {u0:.1f}%->{u1:.1f}% (关小阀, 液位将上升)")
            if mm0 > 120.0:
                print("警告: 液位偏高且将上升, 注意 150mm 守卫")

        # 阶跃后记录时长: 按"离守卫的液位余量 / 预估速率"自动截短, 防排到泵下限
        if args.step > 0:
            margin_pct = pct0 - wc.mm_to_pct(GUARD_LOW_MM, config)
        else:
            margin_pct = wc.mm_to_pct(GUARD_HIGH_MM, config) - pct0
        est_rate = max(0.02, K_PRIOR * abs(u1 - u0))
        record_len = min(args.record, max(30.0, 0.8 * margin_pct / est_rate))
        if record_len < args.record - 1.0:
            print(f"记录时长自动截短 {args.record:.0f}s -> {record_len:.0f}s "
                  f"(离守卫余量约 {margin_pct:.0f}%, 预估速率 {est_rate:.2f}%/s)")

        print(f"记录: 基线 {args.baseline:.0f}s + 阶跃后 {record_len:.0f}s, "
              f"采样 {args.interval:g}s; Ctrl+C 可中止 (自动恢复现场)")

        t0 = time.time()
        # ---- 切手动 + 开泵 + 阀置 u0 ----
        wc.set_control_bit(client, wc.CTL_MANUAL_PUMP, True, sid)
        wc.set_control_bit(client, wc.CTL_AUTO, False, sid)
        set_manual_valve(client, config, u0)
        print(f"已切手动模式, 泵 ON, 阀 -> u0={u0:.1f}%")

        # 等阀指令生效并稳定若干秒
        record_phase(client, config, 5.0, args.interval, "prep", samples, t0,
                     guards=False)

        # ---- 调零: 把 u0 微调到液位基本静止, 保证基线是"平衡点附近"的样本 ----
        if args.trim:
            for it in range(TRIM_MAX_ITERS):
                t_probe = time.time() - t0
                record_phase(client, config, t_probe + args.probe,
                             args.interval, "trim", samples, t0)
                rows = [s for s in samples
                        if s["phase"] == "trim" and s["t"] >= t_probe - 1e-9]
                _, slope = linfit([s["t"] for s in rows], [s["pct"] for s in rows])
                if abs(slope) <= TRIM_TOL:
                    print(f"  调零到位: 基线斜率 {slope:+.4f} %/s (≤{TRIM_TOL:g})")
                    break
                du_fix = max(-TRIM_MAX_DU, min(TRIM_MAX_DU, slope / K_PRIOR))
                u0 = min(80.0, max(3.0, u0 + du_fix))
                u1 = min(100.0, max(0.0, u0 + du_step))
                set_manual_valve(client, config, u0)
                print(f"  调零 {it + 1}/{TRIM_MAX_ITERS}: 斜率 {slope:+.4f} %/s "
                      f"-> u0={u0:.1f}%, 阶跃后 {u1:.1f}%")

        # ---- 基线 ----
        print("基线记录中...")
        record_phase(client, config, time.time() - t0 + args.baseline,
                     args.interval, "baseline", samples, t0)
        t_step = time.time() - t0

        # ---- 阶跃 ----
        set_manual_valve(client, config, u1)
        print(f"阶跃: 阀 {u0:.1f}% -> {u1:.1f}%, 记录中...")
        record_phase(client, config, t_step + record_len, args.interval,
                     "poststep", samples, t0)

        # ---- 恢复现场 ----
        set_manual_valve(client, config, u0)
        if was_auto and not args.no_restore_auto:
            wc.set_control_bit(client, wc.CTL_AUTO, True, sid)
            wc.set_control_bit(client, wc.CTL_MANUAL_PUMP, was_manual_pump, sid)
            print(f"已恢复: 阀={u0:.1f}%, 自动模式 ON, 目标={target_pct:.1f}%")
        else:
            print(f"已恢复阀={u0:.1f}%, 保持手动模式 (--no-restore-auto 或试验前即手动)")

        # ---- 分析 ----
        r = analyze(samples, t_step, u0, u1, learned_d0)
        print_report(r, u0, u1)

        out = args.out or os.path.join(
            os.getcwd(),
            f"step_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        write_csv(samples, out)
        print(f"\n原始数据: {out}")

    except AbortTest as e:
        print(f"\n❌ 试验中止: {e}", file=sys.stderr)
        restore()
        if samples:
            out = args.out or os.path.join(
                os.getcwd(),
                f"step_test_ABORTED_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
            write_csv(samples, out)
            print(f"已中止数据仍写入: {out}", file=sys.stderr)
        try_analyze(samples, t_step, u0, u1, learned_d0)
        sys.exit(2)
    except KeyboardInterrupt:
        print("\n❌ 用户中断", file=sys.stderr)
        restore()
        if samples:
            write_csv(samples, os.path.join(
                os.getcwd(),
                f"step_test_INTERRUPTED_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"))
        try_analyze(samples, t_step, u0, u1, learned_d0)
        sys.exit(130)
    finally:
        client.close()


if __name__ == "__main__":
    main()
