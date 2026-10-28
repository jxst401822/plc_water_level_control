#!/usr/bin/env python3
"""阶跃试验工具 — 辨识对象模型, 计算推荐 PID 增益。

背景: 文档中两套"现场验证"增益(Kp=3.0/Ki=0.40-0.49 与 Kp=1.8/Ki=0.30)
互相矛盾、出处不可考。本工具用经典阶跃响应法(反应曲线法)实测, 并**自动判别
对象是"积分型"还是"自衡型"**(2026-10-20 现场实测: 本装置是自衡的 —— 阀开度
固定时液位会趋于一个新平台, 而不是等速漂移):

  自衡 (FOPDT): 静态增益 K [%液位 per %阀]、时间常数 τp [s]、纯滞后 θ [s]
                SIMC: Kp = τp/(|K|(λ+θ)), Ti = min(τp, 4(λ+θ)), Ki = Kp/Ti
  积分 (IPDT): 斜率增益 K [%/s per %阀]、纯滞后 τ [s]、自然入流 d0 [%/s]
                SIMC: Kp = 1/(K(λ+τ)), Ti = 4(λ+τ), Ki = Kp/Ti

判别: 阶跃后末段斜率收敛到 ~0 且液位确实走过 ≥2% -> 有平台 -> FOPDT。
把自衡对象按积分对象辨识会得到荒谬增益(分母趋零 -> 现场吐过 Kp=107), 报告里
对超出可信范围的推荐值会直接拒绝给出 set-pid 命令。

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
# [G13] 自衡对象静态增益 |K| (%液位 per %阀), 2026-10-20 实测。
# 用 getattr 兜底: 设备上的 water_control_cli.py 可能是旧版(没有 STAT_K),
# 两文件不同步时不能让 step_test 直接 AttributeError
STAT_K = getattr(wc, "STAT_K", 2.13)
TRIM_TOL = 0.15        # 基线斜率容差 %/s (约 0.25mm/s); 超过则继续调零
TRIM_MAX_ITERS = 3
TRIM_MAX_DU = 8.0      # 单次调零最大阀开度修正 %
DEFAULT_SETTLE_S = 30.0  # 切手动+开泵后静置时间 s —— 必须覆盖泵启动/阀阶跃的瞬态
DEFAULT_PROBE_S = 60.0   # 单次探针时长 s —— 本对象 τp≈19s, 20s 的窗口只看到瞬态
TRIM_TAU_S = 20.0      # 由斜率反推"还要漂多久"用的一阶时间常数(s), 仅用于换算调零步长


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
    """读 PLC 前馈参数 (平衡线截距 C, 当前目标的平衡开度 u_eq); 读不到返回 (None, None)。"""
    sid = config["slave_id"]
    try:
        c = wc.read_input_real(client, wc.IREG_UEQC, sid)
        ueq = wc.read_input_real(client, wc.IREG_UEQ, sid)
    except Exception:
        return None, None
    if not (-0.01 < c < 200.0 and -0.01 < ueq < 100.1):
        return None, None          # 寄存器未映射/全 0/垃圾值
    return c, ueq


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


def _crossing(post, t_step, tgt):
    """阶跃后曲线第一次到达 tgt(%) 的时刻 (线性插值)。找不到返回 None。"""
    for a, b in zip(post[:-1], post[1:]):
        if b["t"] <= t_step:
            continue
        if (b["pct"] - tgt) * (a["pct"] - tgt) <= 0 and a["pct"] != b["pct"]:
            return a["t"] + (tgt - a["pct"]) / (b["pct"] - a["pct"]) * (b["t"] - a["t"])
    return None


def _median(xs):
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def analyze(samples, t_step, u0, u1, learned_d0=None):
    """辨识对象模型并给出 SIMC 推荐增益。

    自动判别两种对象特性 (2026-10-20 现场实测确认本装置是**自衡**的):
      自衡 (FOPDT): 阀开度固定 -> 液位趋于一个新的平台 h*(u)。静态增益
                    K = Δy平台/Δu, 一阶时间常数 τp, 纯滞后 θ。
                    SIMC: Kp = τp/(|K|(λ+θ)), Ti = min(τp, 4(λ+θ)), Ki = Kp/Ti
      积分 (IPDT): 阀开度固定 -> 液位等速漂移, 没有平台。
                    SIMC: Kp = 1/(K(λ+τ)), Ti = 4(λ+τ), Ki = Kp/Ti
    判别依据: 阶跃后末段斜率是否收敛到 ~0 (平台)。把自衡对象当积分对象辨识,
    分母斜率差趋零 -> K 趋零 -> 推荐增益爆炸 (本次实测吐过 Kp=107)。
    """
    base = [s for s in samples if s["phase"] == "baseline"]
    post = [s for s in samples if s["phase"] == "poststep"]
    if len(base) < 5 or len(post) < 8:
        raise AbortTest("采样点过少, 无法分析")

    du = u1 - u0
    a0, s0 = linfit([s["t"] for s in base], [s["pct"] for s in base])
    early = [s for s in post if s["t"] - t_step <= 0.35 * (post[-1]["t"] - t_step)]
    late = [s for s in post if s["t"] - t_step >= 0.70 * (post[-1]["t"] - t_step)]
    if len(early) < 3 or len(late) < 3:
        raise AbortTest("阶跃后数据太短 (early/late 窗口不足), 请延长 --record")
    _, s_early = linfit([s["t"] for s in early], [s["pct"] for s in early])
    _, s_late = linfit([s["t"] for s in late], [s["pct"] for s in late])

    y0 = base[-1]["pct"]
    y_f = _median([s["pct"] for s in late])
    dy = y_f - y0
    warn = []

    # ---- 模型判别: 末段几乎不动 且 确实走了一段距离 -> 自衡 ----
    plateau = abs(s_late) <= 0.25 * max(abs(s_early), abs(s0), 1e-6) and abs(dy) >= 2.0
    r = {"plateau": plateau, "s0": s0, "s_early": s_early, "s_late": s_late,
         "y0": y0, "yf": y_f, "warn": warn}

    if plateau:
        K = dy / du                            # %液位 per %阀 (带符号)
        if abs(K) < 0.05:
            raise AbortTest(f"静态增益过小 ({K:.3f}), 平台判断或 Δu 有问题")
        t28t = _crossing(post, t_step, y0 + 0.283 * dy)
        t63t = _crossing(post, t_step, y0 + 0.632 * dy)
        if t28t is None or t63t is None:
            raise AbortTest("响应没走到 28.3%/63.2%, 请延长 --record 后重试")
        tau_p = (t63t - t28t) / 0.67
        theta = t28t - t_step - 0.33 * tau_p
        if tau_p <= 0:
            raise AbortTest(f"拟合得一阶时间常数 τp={tau_p:.1f}s ≤ 0, 数据不可信")
        if theta < DEFAULT_INTERVAL_S:
            theta = DEFAULT_INTERVAL_S
        r.update({"K": K, "tau": theta, "tau_p": tau_p, "d0": None})
        recs = []
        for name, lam in (("λ=θ (标准)", theta), ("λ=2θ (鲁棒)", 2 * theta),
                          ("λ=3θ (很稳)", 3 * theta)):
            kp = tau_p / abs(K) / (lam + theta)
            ti = min(tau_p, 4.0 * (lam + theta))
            recs.append((name, kp, kp / ti, ti))
        r["recs"] = recs
        if (t63t - t_step) > 0.85 * (post[-1]["t"] - t_step):
            warn.append("63.2% 到达点已接近记录窗口末端: τp 可能被低估, 建议 --record 加倍重测")
        exc = max(s["mm"] for s in post) - min(s["mm"] for s in post)
        if exc > 25.0:
            warn.append(f"阶跃后液位跨越 {exc:.1f}mm, 静态增益取的是全程平均, "
                        f"如需精细整定请缩小 --step 分段重测")
        return r

    # ---- 积分型 (IPDT): 沿用切线法 ----
    _, s1 = linfit([s["t"] for s in late], [s["pct"] for s in late])
    ds = s0 - s1
    if abs(ds) < 0.005:
        raise AbortTest(
            f"阶跃前后斜率差过小 ({ds:+.4f} %/s), K 不可辨; 请加大 --step 或延长 --record")
    K = ds / du
    if K <= 0:
        raise AbortTest(f"计算得 K={K:.4f} ≤ 0, 与抽水侧对象方向矛盾; 检查阶跃方向/数据")
    a1, _ = linfit([s["t"] for s in late], [s["pct"] for s in late])
    tau = (a1 - a0) / (s0 - s1) - t_step
    tau_fallback = tau < DEFAULT_INTERVAL_S
    if tau_fallback:
        warn.append(f"切线交点早于阶跃时刻, 纯滞后按一个采样周期取 τ={DEFAULT_INTERVAL_S:g}s "
                    f"(说明本段响应几乎无滞后, 更接近自衡对象)")
        tau = DEFAULT_INTERVAL_S
    d0 = s0 + K * u0
    recs = []
    for name, lam in (("λ=τ (标准)", tau), ("λ=2τ (鲁棒)", 2 * tau)):
        kp = 1.0 / (K * (lam + tau))
        ti = 4.0 * (lam + tau)
        recs.append((name, kp, kp / ti, ti))
    r.update({"K": K, "tau": tau, "tau_p": None, "d0": d0, "recs": recs,
              "s1": s1, "tau_fallback": tau_fallback})

    exc_base = max(s["mm"] for s in base) - min(s["mm"] for s in base)
    exc_post = max(s["mm"] for s in post) - min(s["mm"] for s in post)
    if exc_base > 15.0:
        warn.append(f"基线液位漂移 {exc_base:.1f}mm: u0 偏离平衡开度, K 会被液位相关非线性带偏")
    if exc_post > 25.0:
        warn.append(f"阶跃后液位漂移 {exc_post:.1f}mm: 斜率取自非线性区, 缩短 --record 或减小 --step 重测")
    if learned_d0 and learned_d0 > 0.02 and d0 > 2.0 * learned_d0:
        warn.append(f"外推 d0={d0:+.3f} %/s 远大于 PLC 自学习的 {learned_d0:+.3f} %/s: "
                    f"本次 K 只能用于 u0={u0:.0f}% 附近的液位区间")
    return r


def print_report(r, u0, u1):
    du = u1 - u0
    print("\n========== 阶跃试验结果 ==========")
    print(f"阀阶跃:        {u0:.1f}% -> {u1:.1f}%  (Δu = {du:+.1f}%)")
    print(f"基线斜率 s0:   {r['s0']:+.4f} %/s")
    if r["plateau"]:
        print(f"阶跃后: 前段 {r['s_early']:+.4f} -> 末段 {r['s_late']:+.4f} %/s  (趋于 0 = 有平台)")
        print(f"液位平台:    {r['y0']:.2f}% -> {r['yf']:.2f}%")
        print(f"对象模型:    自衡一阶+纯滞后 (FOPDT)  ← 不是积分对象")
        print(f"静态增益 K:  {r['K']:+.3f} %液位 per %阀")
        print(f"时间常数 τp: {r['tau_p']:.1f} s")
        print(f"纯滞后 θ:    {r['tau']:.1f} s   (两点法 28.3%/63.2%)")
        # 平衡线 u_eq(h) = C - h/|K|: 两个平台点各解一次 C, 相差小 = 线性关系成立
        c0 = u0 + r["y0"] / abs(r["K"])
        c1 = u1 + r["yf"] / abs(r["K"])
        print(f"平衡线截距 C: {0.5 * (c0 + c1):.2f}%  "
              f"(阶跃前 {c0:.2f} / 阶跃后 {c1:.2f}, 相差 {abs(c1 - c0):.2f}%"
              f"{' → 线性成立' if abs(c1 - c0) < 2.0 else ' → 非线性明显, C 只在此区间可用'})")
        print(f"             = ST 里 rUeqBase 的初值; get-learn 读到的 C 应与此接近")
        print("\n推荐增益 (SIMC FOPDT, Kd=0):")
    else:
        print(f"阶跃后斜率 s1: {r['s1']:+.4f} %/s  (仍在等速漂移 = 积分对象)")
        print(f"对象模型:    积分+纯滞后 (IPDT)")
        print(f"对象增益 K:    {r['K']:.5f} %/s per %阀")
        print(f"纯滞后 τ:      {r['tau']:.1f} s   (切线法)")
        print(f"自然入流 d0:   {r['d0']:+.4f} %/s (阀全关时液位上升速率)")
        print("\n推荐增益 (SIMC IPDT, Kd=0):")
    for name, kp, ki, ti in r["recs"]:
        print(f"  {name:<14} Kp={kp:6.2f}  Ki={ki:7.4f}  (Ti={ti:6.1f}s)")

    kp2, ki2 = r["recs"][1][1], r["recs"][1][2]      # 鲁棒档
    warns = r.get("warn", [])
    for w in warns:
        print(f"  ⚠️ {w}", file=sys.stderr)
    if kp2 > 10.0 or ki2 > 1.0:
        print(f"\n⚠️ 鲁棒档 Kp={kp2:.2f}/Ki={ki2:.4f} 超出可信范围: 本次辨识不可用, "
              f"不要写进 PLC。常见原因: 基线没调零、记录窗口太短、液位跌进非线性区。",
              file=sys.stderr)
    elif warns:
        print(f"\n⚠️ 本次有 {len(warns)} 条有效性警告: 上面的增益只作量级参考, "
              f"先把警告清掉再重测一次, 不要直接写进 PLC。", file=sys.stderr)
    else:
        print(f"\n现场应用 (先用鲁棒档验证, 响应过慢再换标准档):")
        print(f"  python3 water_control_cli.py set-pid {kp2:.2f} {ki2:.4f} 0")
    print("观察无振荡后, 可将实测值回填: st/PLC_PRG_improved.st 默认增益、SKILL.md 文档。")


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
    ap.add_argument("--settle", type=float, default=DEFAULT_SETTLE_S,
                    help="切手动+开泵后静置 s (默认 %g)" % DEFAULT_SETTLE_S)
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
    learned_c = None

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

        learned_c, learned_ueq = read_learned(client, config)
        if learned_c is not None:
            print(f"PLC 前馈参数: C={learned_c:.2f}%  "
                  f"u_eq(按目标 {target_pct:.1f}%)={learned_ueq:.1f}%")

        if args.u0 is not None:
            u0 = min(90.0, max(0.0, args.u0))
            u0_src = "命令行指定"
        elif learned_ueq is not None and 5.0 <= learned_ueq <= 60.0:
            # [G13] 本对象自衡: 平衡开度随水位线性变化。PLC 回读的 u_eq 是按
            # 目标派生的, 而试验要的是"**当前液位**的平衡开度" → 用静态增益换算
            u0 = learned_ueq + (target_pct - pct0) / STAT_K
            u0 = min(80.0, max(3.0, u0))
            u0_src = f"PLC u_eq 按当前液位换算 (K={STAT_K:g})"
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

        # 静置: 覆盖泵启动/阀阶跃的瞬态。本对象 τp≈19s, 5s 远远不够 ——
        # 之前 15s 窗口把泵启动的水位冲击当成"基线在漂", 把 u0 猛推了 15%
        record_phase(client, config, args.settle, args.interval, "prep",
                     samples, t0, guards=False)

        # ---- 调零: 把 u0 微调到液位基本静止, 保证基线是"平衡点附近"的样本 ----
        if args.trim:
            best = None                       # (|slope|, u0) —— 取斜率最小的那次
            for it in range(TRIM_MAX_ITERS):
                t_probe = time.time() - t0
                record_phase(client, config, t_probe + args.probe,
                             args.interval, "trim", samples, t0)
                rows = [s for s in samples
                        if s["phase"] == "trim" and s["t"] >= t_probe - 1e-9]
                _, slope = linfit([s["t"] for s in rows], [s["pct"] for s in rows])
                if best is None or abs(slope) < best[0]:
                    best = (abs(slope), u0)
                if abs(slope) <= TRIM_TOL:
                    print(f"  调零到位: 基线斜率 {slope:+.4f} %/s (≤{TRIM_TOL:g})")
                    break
                # 自衡对象: 斜率 s 表示液位还在朝平台 h* 收口, 剩余行程 ≈ s·τp,
                # 换算成阀开度要除静态增益 —— 用 s/K_PRIOR(积分对象的算法)会放大 10 倍
                du_fix = slope * TRIM_TAU_S / STAT_K
                du_fix = max(-TRIM_MAX_DU, min(TRIM_MAX_DU, du_fix))
                u0 = min(80.0, max(3.0, u0 + du_fix))
                u1 = min(100.0, max(0.0, u0 + du_step))
                set_manual_valve(client, config, u0)
                print(f"  调零 {it + 1}/{TRIM_MAX_ITERS}: 斜率 {slope:+.4f} %/s "
                      f"-> u0={u0:.1f}%, 阶跃后 {u1:.1f}%")
            if best is not None and abs(best[1] - u0) > 0.05:
                u0 = best[1]
                u1 = min(100.0, max(0.0, u0 + du_step))
                set_manual_valve(client, config, u0)
                print(f"  末次修正反而变差, 回退到 |斜率| 最小的 u0={u0:.1f}% "
                      f"({best[0]:.4f} %/s)")

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
        r = analyze(samples, t_step, u0, u1)
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
        try_analyze(samples, t_step, u0, u1)
        sys.exit(2)
    except KeyboardInterrupt:
        print("\n❌ 用户中断", file=sys.stderr)
        restore()
        if samples:
            write_csv(samples, os.path.join(
                os.getcwd(),
                f"step_test_INTERRUPTED_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"))
        try_analyze(samples, t_step, u0, u1)
        sys.exit(130)
    finally:
        client.close()


if __name__ == "__main__":
    main()
