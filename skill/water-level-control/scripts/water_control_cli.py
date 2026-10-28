"""
water_control_cli.py - 施耐德 PLC 水位控制命令行工具

通过 Modbus TCP 读写水位目标、实测液位、PID 增益、手动阀/泵, 并做液位标定。
PLC 内部一律用百分比 (0-100% = 4-20mA), mm 换算在客户端完成。

**完整用法、SOP、安全须知见同目录上级的 SKILL.md**(中文版, 随 skill 打包)。
快速上手:
    python3 water_control_cli.py status                 # 看当前状态
    python3 water_control_cli.py set-mm 100 --wait      # 调到 100mm 并阻塞到稳定
    python3 water_control_cli.py stop --delay 5         # 静止化(留 5s 给操作员关上方阀门)
    python3 water_control_cli.py resume                 # 恢复自动运行
    python3 water_control_cli.py calibrate --show       # 看当前 mm 标定

配置文件: ~/.water-control/config.json (plc_ip / plc_port / slave_id / level_model)
设备限制: 支持的目标水位 50-150mm; 泵物理下限约 45.3mm。
"""

import argparse
import csv
import struct
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

from pymodbus.client import ModbusTcpClient

# Windows GBK 控制台兼容: 输出里的 OK/WARN 标记与 emoji 无法编码时替换而不是抛
# UnicodeEncodeError 把整条命令崩掉。Linux/UTF-8 下无影响。
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        try:
            _stream.reconfigure(errors="replace")
        except Exception:
            pass

# Config file path
CONFIG_DIR = Path.home() / ".water-control"
CONFIG_FILE = CONFIG_DIR / "config.json"

# Default configuration
DEFAULT_CONFIG = {
    "plc_ip": "192.168.0.122",
    "plc_port": 502,
    "slave_id": 1,
    # 分段线性标定(多点插值): 传感器有非线性, 一条直线拟合全量程会有数 mm 残差,
    # 改用实测点表分段插值, 每段独立斜率, 段内残差<0.1mm。
    # 点对按 iLevelRaw 升序: (iLevelRaw, 物理 mm)。
    # 现场核对: 原标定整体偏高约 10mm(raw14128实测110, raw16500实测130),
    # 故在原拟合点基础上物理 mm 统一 -10, 与实测吻合。
    #   5748->33, 10181->75, 12560->97, 16587->131, 20000->158(20mA满量程)
    # 段外(低于首点/高于末点)按端点斜率线性外推; 末点即20mA实测, 无需外推。
    "level_cal_points": [
        [5748, 33.0],
        [10181, 75.0],
        [12560, 97.0],
        [16587, 131.0],
        [20000, 158.0],
    ],
    # 线性模型 (优先于点表): mm = k*(iLevelRaw-4000) + b
    # k 由上述 5 点最小二乘拟合 (残差 <2.2mm) —— 传感器/桶形的固有斜率, 不随漂移变
    # b 是"4mA 对应多少 mm", 每次 calibrate 只更新它
    "level_model": {"k": 0.0087683, "b": 19.8},
}

# ---- 两套地址空间 ----
# 保持寄存器 (Holding, FC03读/FC16写): 可写设定量。数组 iw...(保持)下标。
REG_TARGET_LEVEL = 0     # rTargetLevel  写入PLC (Device_0/1)
REG_KP = 2               # rKp           写入PLC (Device_2/3)
REG_KI = 4               # rKi           写入PLC (Device_4/5)
REG_MANUAL_VALVE = 6     # rManualValve  写入PLC (Device_6/7)
REG_CONTROL = 10         # 控制字 (Device_10), 可写位: bit0=bAuto, bit1=bManualPump, bit2=bPidTune
REG_KD = 12              # rKd           写入PLC (Device_12/13)
# 控制字位定义 (与 PLC 读入段一致)
CTL_AUTO = 0x01          # bit0 bAuto (1=自动 0=手动)
CTL_MANUAL_PUMP = 0x02   # bit1 bManualPump (手动模式泵开关)
CTL_PID_TUNE = 0x04      # bit2 bPidTune (1=采用 Modbus 传入的 Kp/Ki/Kd)
# 输入寄存器 (Input, FC04只读): 只读状态量。数组 qwModbusTCP_Server_Device_N(输入)下标。
# PLC 映射: Device_2/3<-rLevel, Device_4/5<-rValveCmd, Device_8<-状态位打包字
IREG_LEVEL = 2           # rLevel     只读
IREG_VALVE_CMD = 4       # rValveCmd  只读
IREG_KP = 10             # rKp 反馈 (PLC 实际在用值) 只读
IREG_KI = 12             # rKi 反馈 只读
IREG_KD = 14             # rKd 反馈 只读
IREG_STATUS = 8          # 状态位打包字 (FC04), PLC: bit0=bAuto,1=bPumpCmd,2=bSensorFault,3=bAlarm,4=bPump
IREG_LEVEL_RAW = 9       # iLevelRaw 原始计数 (4000..20000), 校准量程用; 旧 PLC 程序未映射时读到 0
# [v2] PLC 前馈参数回读 (ST 把它们映射到输入寄存器空闲槽 0/1 与 6/7)
IREG_UEQC = 0            # 平衡线截距 C (%阀开度), PLC 在线学习; u_eq(SP) = C - SP/STAT_K
IREG_UEQ = 6             # 当前目标的平衡开度 u_eq (%) —— PLC 每拍派生
# 自衡对象静态增益 |K| (%液位 per %阀), 2026-10-20 阶跃试验实测
STAT_K = 2.13
IREG_INFLOW = IREG_UEQC  # 兼容旧名(旧版这一槽是 d0, v2 起改为 C)

# 状态字位定义 (与 PLC qwModbusTCP_Server_Device_8 打包一致)
ST_AUTO = 0x01           # bit0 bAuto
ST_PUMP_CMD = 0x02       # bit1 bPumpCmd
ST_SENSOR_FAULT = 0x04   # bit2 bSensorFault
ST_ALARM = 0x08          # bit3 bAlarm
ST_PUMP = 0x10           # bit4 bPump (实际输出 %QX4.0)

# 注: %M 位未单独映射到线圈, 已改为通过控制字(写)/状态字(读)访问,
# 见 REG_CONTROL / IREG_STATUS。旧的 BIT_*/write_coil/read_bit 已移除。


def load_config() -> dict:
    """Load config from file or return defaults (merge missing keys from defaults)."""
    if CONFIG_FILE.exists():
        with open(CONFIG_FILE, "r") as f:
            saved = json.load(f)
        # 合并: 缺失的键用默认值补上(避免旧 config.json 缺 level_cm_full 报 KeyError)
        merged = DEFAULT_CONFIG.copy()
        merged.update(saved)
        return merged
    return DEFAULT_CONFIG.copy()


def save_config(config: dict):
    """Save config to file."""
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    with open(CONFIG_FILE, "w") as f:
        json.dump(config, f, indent=2)


# ---------- 毫米 <-> 百分比 换算 (分段线性插值标定) ----------
# PLC 百分比 rLevel = (iLevelRaw-4000)/16000*100, 即 iLevelRaw = pct/100*16000+4000。
# 传感器有非线性, 用实测点表 (iLevelRaw, mm) 分段线性插值, 段内残差<0.1mm。
# 段外按最近段斜率线性外推。点表在 config["level_cal_points"], 按 iLevelRaw 升序。

def _model(config: dict):
    """线性模型 {"k","b"} 或 None; mm = k*(raw-4000) + b。"""
    m = config.get("level_model")
    return m if m and "k" in m and "b" in m else None


def _model_k(config: dict) -> float:
    """当前使用的斜率 (mm/计数), 线性模型优先, 否则由点表首段估。"""
    m = _model(config)
    if m:
        return m["k"]
    pts = _cal_points(config)
    return (pts[1][1] - pts[0][1]) / (pts[1][0] - pts[0][0])


def _cal_points(config: dict) -> list:
    """返回 [(iLevelRaw, mm), ...], 按 iLevelRaw 升序。"""
    pts = config.get("level_cal_points", DEFAULT_CONFIG["level_cal_points"])
    return sorted(pts, key=lambda p: p[0])


def _cal_range_mm(config: dict):
    """返回 (min_mm, max_mm) 标定范围, 用于命令参数范围校验。"""
    m = _model(config)
    if m:
        return m["k"] * 0.0 + m["b"], m["b"] + m["k"] * 16000.0
    pts = _cal_points(config)
    return pts[0][1], pts[-1][1]


def _interp_mm(raw: float, pts: list) -> float:
    """iLevelRaw -> mm, 分段线性插值, 段外按端点斜率外推。"""
    if raw <= pts[0][0]:
        # 低于首点: 用首段斜率外推
        if len(pts) >= 2:
            x0, y0 = pts[0]; x1, y1 = pts[1]
            s = (y1 - y0) / (x1 - x0)
            return y0 + (raw - x0) * s
        return pts[0][1]
    if raw >= pts[-1][0]:
        # 高于末点: 用末段斜率外推
        if len(pts) >= 2:
            x0, y0 = pts[-2]; x1, y1 = pts[-1]
            s = (y1 - y0) / (x1 - x0)
            return y1 + (raw - x1) * s
        return pts[-1][1]
    # 段内插值
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]; x1, y1 = pts[i + 1]
        if x0 <= raw <= x1:
            t = (raw - x0) / (x1 - x0)
            return y0 + t * (y1 - y0)
    return pts[-1][1]


def _interp_raw(mm: float, pts: list) -> float:
    """mm -> iLevelRaw, 分段线性反插值, 段外按端点斜率外推。"""
    if mm <= pts[0][1]:
        if len(pts) >= 2:
            x0, y0 = pts[0]; x1, y1 = pts[1]
            s = (x1 - x0) / (y1 - y0)
            return x0 + (mm - y0) * s
        return pts[0][0]
    if mm >= pts[-1][1]:
        if len(pts) >= 2:
            x0, y0 = pts[-2]; x1, y1 = pts[-1]
            s = (x1 - x0) / (y1 - y0)
            return x1 + (mm - y1) * s
        return pts[-1][0]
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]; x1, y1 = pts[i + 1]
        if y0 <= mm <= y1:
            t = (mm - y0) / (y1 - y0)
            return x0 + t * (x1 - x0)
    return pts[-1][0]


def mm_to_pct(mm: float, config: dict) -> float:
    """毫米 -> PLC 百分比 (线性模型优先, 否则分段插值反解)。"""
    m = _model(config)
    if m:
        return ((mm - m["b"]) / m["k"]) / 160.0
    pts = _cal_points(config)
    raw = _interp_raw(mm, pts)
    return (raw - 4000) / 16000.0 * 100.0


def pct_to_mm(pct: float, config: dict) -> float:
    """PLC 百分比 -> 毫米 (线性模型优先, 否则分段插值)。"""
    m = _model(config)
    if m:
        return m["k"] * (pct * 160.0) + m["b"]
    pts = _cal_points(config)
    raw = pct / 100.0 * 16000.0 + 4000
    return _interp_mm(raw, pts)


def float_to_registers(value: float) -> list:
    """Convert float to 2 Modbus registers (Schneider: low word first).

    %MW0 = low 16 bits, %MW1 = high 16 bits.
    现场实测确认: 低字在前才对(写 10.0 -> %MW0=0, %MW1=16672, PLC 读回 10.0)。
    若改后读回乱码, 说明本台 PLC 改成了 high-word-first, 再换回 [high, low]。
    """
    raw = struct.pack('>f', value)
    high = struct.unpack('>H', raw[0:2])[0]
    low = struct.unpack('>H', raw[2:4])[0]
    return [low, high]


def registers_to_float(registers: list) -> float:
    """Convert 2 Modbus registers to float (Schneider: low word first)."""
    low = struct.pack('>H', registers[0])
    high = struct.pack('>H', registers[1])
    return struct.unpack('>f', high + low)[0]


def read_real(client: ModbusTcpClient, address: int, slave_id: int) -> float:
    """Read a REAL from 2 HOLDING registers (FC03). 用于可读写的设定量 (rTargetLevel)。"""
    result = client.read_holding_registers(address=address, count=2, slave=slave_id)
    if result.isError():
        raise Exception(f"Read error (FC03) at address {address}: {result}")
    return registers_to_float(result.registers)


def read_input_real(client: ModbusTcpClient, address: int, slave_id: int) -> float:
    """Read a REAL from 2 INPUT registers (FC04). 用于只读状态量 (rLevel, rValveCmd)。

    PLC 把状态量映射到输入寄存器数组(FC04 只读), 必须用 read_input_registers,
    用 FC03(保持寄存器)读不到 —— 这是之前 rLevel 读出 0 的原因。
    """
    result = client.read_input_registers(address=address, count=2, slave=slave_id)
    if result.isError():
        raise Exception(f"Read error (FC04) at address {address}: {result}")
    return registers_to_float(result.registers)


def write_real(client: ModbusTcpClient, address: int, value: float, slave_id: int):
    """Write a REAL to 2 HOLDING registers (FC16). 用于可写的设定量 (rTargetLevel)。"""
    regs = float_to_registers(value)
    result = client.write_registers(address=address, values=regs, slave=slave_id)
    if result.isError():
        raise Exception(f"Write error at address {address}: {result}")


def read_status_word(client: ModbusTcpClient, slave_id: int) -> int:
    """Read the packed status word from INPUT register (FC04).

    PLC 把状态位打包成一个字: qwModbusTCP_Server_Device_8 :=
        BOOL_TO_INT(bAuto) + bPumpCmd*2 + bSensorFault*4 + bAlarm*8 + bPump*16
    比读线圈可靠(位没单独映射到线圈)。返回原始 INT, 用 ST_* 掩码取各位。
    """
    result = client.read_input_registers(address=IREG_STATUS, count=1, slave=slave_id)
    if result.isError():
        raise Exception(f"Read status word error (FC04) at {IREG_STATUS}: {result}")
    return result.registers[0]


def read_control_word(client: ModbusTcpClient, slave_id: int) -> int:
    """Read the control word from HOLDING register (FC03). 用于读改写保留其它位。"""
    result = client.read_holding_registers(address=REG_CONTROL, count=1, slave=slave_id)
    if result.isError():
        raise Exception(f"Read control word error (FC03) at {REG_CONTROL}: {result}")
    return result.registers[0]


def set_control_bit(client: ModbusTcpClient, mask: int, value: bool, slave_id: int) -> int:
    """Read-modify-write one bit of the control word (HOLDING reg, FC16).

    控制字在保持寄存器 Device_10, 直接写整字会覆盖同字其它位, 故读回->改目标位->写回,
    保留 bAuto / bManualPump 互不干扰。返回写入后的整字值。
    """
    word = read_control_word(client, slave_id)
    if value:
        word |= mask
    else:
        word &= ~mask
    word &= 0xFFFF
    result = client.write_registers(address=REG_CONTROL, values=[word], slave=slave_id)
    if result.isError():
        raise Exception(f"Write control word error at {REG_CONTROL}: {result}")
    return word


def connect_plc(config: dict) -> ModbusTcpClient:
    """Connect to PLC and return client."""
    client = ModbusTcpClient(config["plc_ip"], port=config["plc_port"])
    if not client.connect():
        print(f"Error: Cannot connect to PLC at {config['plc_ip']}:{config['plc_port']}")
        sys.exit(1)
    return client


def cmd_read(args, config):
    """Read current target level."""
    client = connect_plc(config)
    try:
        target = read_real(client, REG_TARGET_LEVEL, config["slave_id"])
        print(f"{target:.1f}")
    finally:
        client.close()


def cmd_read_mm(args, config):
    """Read current actual level in mm (reads rLevel via FC04, 换算成毫米)."""
    client = connect_plc(config)
    try:
        level = read_input_real(client, IREG_LEVEL, config["slave_id"])
        mm = pct_to_mm(level, config)
        print(f"{mm:.1f} mm  ({level:.1f}%)")
    finally:
        client.close()


SUPPORTED_MM = (50.0, 150.0)   # 支持设定目标的区间(平衡曲线实测覆盖范围)


def _wait_settled(client, config, target_mm, tolerance=3.0, timeout=100.0,
                  interval=2.0, need=3, stall_grace=15.0):
    """写目标之后就地轮询, 直到液位进入 target±tolerance 并连续 need 次保持。

    存在的意义: 让 agent **一次工具调用**就完成"设定 + 等待 + 结论", 不需要自己写
    轮询循环。agent 每多一次工具调用就多一次向用户自言自语的机会
    ("还在等""再查一次"), 这是本函数要避免的主要故障模式。

    三种结束方式:
      settled  —— 连续 need 次落在容差内                     (reason='settled')
      stalled  —— 液位已停住 stall_grace 秒但仍差 tolerance 以上 (reason='stalled')
                  提前退出, 不白等到 timeout。到不了位通常就是这个形态。
      timeout  —— 到 timeout 仍在动且没进容差                  (reason='timeout')

    复用同一个 Modbus 连接。返回 (ok, reason, elapsed_s, level_mm, valve_pct,
    drift_mm_per_s)。不抛异常 —— 调用方决定怎么报告。
    """
    sid = config["slave_id"]
    t0 = time.time()
    hist = []            # (t, mm) 用于算漂移
    streak = 0
    still_since = None
    next_report = 10.0
    while True:
        el = time.time() - t0
        lvl_pct = read_input_real(client, IREG_LEVEL, sid)
        lvl_mm = pct_to_mm(lvl_pct, config)
        try:
            valve = read_input_real(client, IREG_VALVE_CMD, sid)
        except Exception:
            valve = float("nan")
        hist.append((el, lvl_mm))
        if len(hist) > 30:
            hist.pop(0)

        dev = abs(lvl_mm - target_mm)
        if dev <= tolerance:
            streak += 1
            if streak >= need:
                return True, "settled", el, lvl_mm, valve, _drift(hist[-6:])
        else:
            streak = 0

        # 停住判定: 用最近 6 点(约 12s)的斜率, 静差大于容差且已不动 -> 提前退出
        d6 = _drift(hist[-6:]) if len(hist) >= 6 else None
        if d6 is not None and abs(d6) < 0.02:
            if still_since is None:
                still_since = el
            elif dev > tolerance and (el - still_since) >= stall_grace:
                return False, "stalled", el, lvl_mm, valve, d6
        else:
            still_since = None

        if el >= timeout:
            return False, "timeout", el, lvl_mm, valve, _drift(hist[-6:])
        if el >= next_report:
            print(f"  t={el:5.0f}s  level={lvl_mm:6.1f}mm  (距目标 "
                  f"{lvl_mm - target_mm:+5.1f}mm)  valve={valve:5.1f}%")
            next_report += 10.0
        time.sleep(max(0.1, interval))


def _drift(hist):
    """最近若干点的最小二乘斜率, mm/s。点太少返回 0。"""
    if len(hist) < 3:
        return 0.0
    n = len(hist)
    sx = sum(t for t, _ in hist)
    sy = sum(y for _, y in hist)
    sxx = sum(t * t for t, _ in hist)
    sxy = sum(t * y for t, y in hist)
    den = n * sxx - sx * sx
    return (n * sxy - sx * sy) / den if den else 0.0


def _report_wait(ok, reason, el, lvl_mm, target_mm, valve, drift,
                 tolerance, timeout):
    """打印 --wait 的结论行, 返回进程退出码 (0=已稳定, 3=未到位)。"""
    dev = lvl_mm - target_mm
    if ok:
        print(f"OK 已稳定: {lvl_mm:.1f} mm  (目标 {target_mm:g}, 偏差 {dev:+.1f}mm)  "
              f"用时 {el:.0f}s  阀 {valve:.1f}%")
        return 0
    head = ("WARN 到不了位" if reason == "stalled" else "WARN 超时未稳定")
    why = (f"液位已停住不再变化 (等了 {el:.0f}s)" if reason == "stalled"
           else f"超时 {timeout:g}s, 漂移 {drift:+.3f} mm/s")
    print(f"{head}: 当前 {lvl_mm:.1f} mm, 距目标 {dev:+.1f}mm "
          f"(容差 ±{tolerance:g}mm), 阀 {valve:.1f}%  [{why}]", file=sys.stderr)
    if reason == "stalled":
        print("  这是稳态静差, 不是还没走到: 该目标处的前馈偏差超出了积分的修正权限。"
              " 把(目标, 实测液位, 阀开度)记录下来上报, **不要重试、不要继续轮询、"
              "不要自行改 PID**。", file=sys.stderr)
    elif abs(dev) <= tolerance:
        print("  最后一次读数其实已在容差内, 但没能连续多次保持 —— 说明液位"
              " 正在目标附近来回穿越(振荡), 不是没走到。记下波动带宽上报,"
              " 不要重试。", file=sys.stderr)
    else:
        print("  上升本来就慢(靠重力自流)。可以再用 --wait 等一次; 若两次都超时"
              " 就按上面那条上报。", file=sys.stderr)
    return 3


def _check_supported(mm):
    lo, hi = SUPPORTED_MM
    if not lo <= mm <= hi:
        print(f"WARN 目标 {mm:g}mm 超出支持区间 {lo:g}-{hi:g}mm "
              f"(平衡曲线只在这段有实测点)。仍可执行, 但可能到不了位。",
              file=sys.stderr)


def cmd_set_mm(args, config):
    """Set target level in mm (converts to % via affine cal and writes rTargetLevel)."""
    mm = args.value
    lo, hi = _cal_range_mm(config)
    if not lo <= mm <= hi:
        print(f"Error: Value must be {lo:g}-{hi:g} mm")
        sys.exit(1)
    pct = mm_to_pct(mm, config)
    client = connect_plc(config)
    try:
        write_real(client, REG_TARGET_LEVEL, pct, config["slave_id"])
        print(f"OK: rTargetLevel = {mm:g} mm ({pct:.1f}%)")
        if not args.wait:
            _check_supported(mm)
            return
        _check_supported(mm)
        print(f"等待稳定 (容差 ±{args.tolerance:g}mm, 连续 {args.confirm} 次, "
              f"超时 {args.timeout:g}s, 每 {args.interval:g}s 轮询)...")
        ok, reason, el, lvl, valve, drift = _wait_settled(
            client, config, mm, args.tolerance, args.timeout,
            args.interval, args.confirm)
        sys.exit(_report_wait(ok, reason, el, lvl, mm, valve, drift,
                              args.tolerance, args.timeout))
    finally:
        client.close()


def cmd_raise_lower(args, config):
    """Raise/lower target by delta mm: 读当前目标, 加/减增量, 写回。

    增量控制本质 = 绝对目标: target_new = target_old ± delta。
    PLC 无需支持增量, CLI 读旧目标、算新目标、写绝对值即可。
    """
    delta = args.value
    sign = 1.0 if args.command == "raise" else -1.0
    lo, hi = _cal_range_mm(config)
    client = connect_plc(config)
    try:
        old_pct = read_real(client, REG_TARGET_LEVEL, config["slave_id"])
        old_mm = pct_to_mm(old_pct, config)
        new_mm = max(lo, min(hi, old_mm + sign * delta))
        new_pct = mm_to_pct(new_mm, config)
        write_real(client, REG_TARGET_LEVEL, new_pct, config["slave_id"])
        print(f"OK: target {old_mm:.1f} -> {new_mm:.1f} mm  ({old_pct:.1f}% -> {new_pct:.1f}%)")
        _check_supported(new_mm)
        if args.wait:
            print(f"等待稳定 (容差 ±{args.tolerance:g}mm, 连续 {args.confirm} 次, "
                  f"超时 {args.timeout:g}s, 每 {args.interval:g}s 轮询)...")
            ok, reason, el, lvl, valve, drift = _wait_settled(
                client, config, new_mm, args.tolerance, args.timeout,
                args.interval, args.confirm)
            sys.exit(_report_wait(ok, reason, el, lvl, new_mm, valve, drift,
                                  args.tolerance, args.timeout))
    finally:
        client.close()


def cmd_write(args, config):
    """Set target level — only writes %MW0, nothing else."""
    value = args.value
    if not 0.0 <= value <= 100.0:
        print("Error: Value must be 0-100%")
        sys.exit(1)

    client = connect_plc(config)
    try:
        write_real(client, REG_TARGET_LEVEL, value, config["slave_id"])
        print(f"OK: rTargetLevel = {value:.1f}%")
    finally:
        client.close()


def cmd_read_raw(args, config):
    """Dump raw registers for byte-order / address debugging."""
    client = connect_plc(config)
    try:
        addr = args.address if args.address is not None else REG_TARGET_LEVEL
        result = client.read_holding_registers(address=addr, count=8, slave=config["slave_id"])
        if result.isError():
            raise Exception(f"Read error at address {addr}: {result}")
        regs = result.registers
        print(f"Raw registers from %{addr} (8 words):")
        for i, r in enumerate(regs):
            print(f"  %MW{addr+i} = {r}  (0x{r:04X})")
        # 按高字在前解释前两个寄存器为 REAL
        if len(regs) >= 2:
            hi = f"{regs[0]:04X}{regs[1]:04X}"
            lo = f"{regs[1]:04X}{regs[0]:04X}"
            import struct as _s
            v_hi = _s.unpack('>f', bytes.fromhex(hi))[0]
            v_lo = _s.unpack('>f', bytes.fromhex(lo))[0]
            print(f"  as REAL [high-word-first]: {v_hi}")
            print(f"  as REAL [low-word-first] : {v_lo}   <- 本台 PLC 实际采用")
    finally:
        client.close()


def cmd_auto(args, config):
    """Switch to auto mode (control word bit0 = 1)."""
    client = connect_plc(config)
    try:
        word = set_control_bit(client, CTL_AUTO, True, config["slave_id"])
        print(f"Auto mode: ON  (control word=0x{word:04X})")
    finally:
        client.close()


def cmd_manual(args, config):
    """Switch to manual mode (control word bit0 = 0)."""
    client = connect_plc(config)
    try:
        word = set_control_bit(client, CTL_AUTO, False, config["slave_id"])
        print(f"Auto mode: OFF -> manual  (control word=0x{word:04X})")
    finally:
        client.close()


def cmd_pump(args, config):
    """Manual pump on/off (control word bit1 = bManualPump). 仅在手动模式生效。"""
    on = args.state == "on"
    client = connect_plc(config)
    try:
        word = set_control_bit(client, CTL_MANUAL_PUMP, on, config["slave_id"])
        print(f"Manual pump: {'ON' if on else 'OFF'}  (control word=0x{word:04X})")
        print("  note: 仅在手动模式(manual)下生效; 自动模式泵由 PID 控制")
    finally:
        client.close()


def cmd_valve(args, config):
    """Manual valve opening 0-100% (writes rManualValve, Device_6). 仅手动模式生效。"""
    pct = args.value
    if not 0.0 <= pct <= 100.0:
        print("Error: Value must be 0-100%")
        sys.exit(1)
    client = connect_plc(config)
    try:
        write_real(client, REG_MANUAL_VALVE, pct, config["slave_id"])
        print(f"OK: rManualValve = {pct:.1f}%")
        print("  note: 仅在手动模式(manual)下生效; 自动模式阀由 PID 控制")
    finally:
        client.close()


def _quiescent_report(client, config, settle=5.0, samples=5, last=True):
    """静水确认: 回读阀/泵状态, 再看液位是否真的不动了。

    返回 (ok, drift_mm)。ok=False 有两种原因, 分开报:
      - PLC 侧没执行(阀仍开/泵仍转) —— 多半是 HH 联锁在强制抽水
      - PLC 侧已全关但液位仍在变 —— 上方左右桶连通没阻断, 或在漏水

    last=False 表示后面还会重试(操作员可能正在关上方阀门), 此时把"没阻断"
    的告警降级成提示, 避免第一次尝试就吓一跳。
    """
    sid = config["slave_id"]
    sw = read_status_word(client, sid)
    valve = read_input_real(client, IREG_VALVE_CMD, sid)
    pump_on = bool(sw & ST_PUMP) or bool(sw & ST_PUMP_CMD)
    print(f"回读: 阀指令 {valve:.1f}%   泵 {'ON' if pump_on else 'OFF'}   "
          f"自动 {'ON' if (sw & ST_AUTO) else 'OFF(手动)'}")
    if pump_on or valve > 1.0:
        if sw & ST_SENSOR_FAULT:
            print("⚠️ 传感器故障联锁生效中(阀应为 0), 先查探头接线", file=sys.stderr)
        elif sw & ST_ALARM:
            print("⚠️ PLC 安全联锁在覆盖手动指令 —— 液位高于 rLvlHH(当前 98.5%) 时"
                  "会强制阀 100% + 泵 ON。这是设计行为, 手动/stop 都盖不住它。"
                  "要真正静止, 先把液位降到 HH 以下(或临时抬高 rLvlHH)。",
                  file=sys.stderr)
        else:
            print("⚠️ 指令已写但 PLC 未执行(阀仍开或泵仍转)", file=sys.stderr)
        return False, None

    # 液位漂移检查: 全关之后液位只应随温度/毛细缓慢变化
    if settle <= 0 or samples < 2:
        return True, 0.0
    raws = []
    for i in range(samples):
        raws.append(read_level_raw(client, sid))
        if i < samples - 1:
            time.sleep(settle / (samples - 1))
    if raws[0] == 0:
        print("  (iLevelRaw 未映射, 跳过漂移检查)")
        return True, None
    drift = (raws[-1] - raws[0]) * _model_k(config)
    span = (max(raws) - min(raws)) * _model_k(config)
    print(f"漂移检查({settle:g}s, {samples} 次): raw {raws[0]} -> {raws[-1]}   "
          f"净漂移 {drift:+.2f}mm   峰峰 {span:.2f}mm")
    if abs(drift) > 1.0 or span > 3.0:
        if last:
            print("⚠️ 液位仍在移动 —— 泵和阀都已关闭, 说明**上方左右桶连通没阻断**"
                  "(或桶体/管路在漏水)。此时做校准或阶跃测量都会失真。",
                  file=sys.stderr)
        else:
            print("  液位仍在移动 —— 如果操作员刚刚才关掉上方阀门, 属正常, 会再查一次")
        return False, drift
    print("✅ 系统已静止: 上下都封闭, 液位不动。可以校准/采样。")
    return True, drift


def cmd_stop(args, config):
    """把装置置于全静止: 泵 OFF + 阀 0% + 切手动。上方连通由现场人工阻断。

    写入顺序有讲究: 先写 rManualValve=0 与 bManualPump=0, **最后**才清 bAuto。
    手动段的阀指令没有速率限制(rValveRate 只作用于自动路径), 若先切手动而
    rManualValve 还是旧值, 阀会一步跳到那个值。
    """
    client = connect_plc(config)
    try:
        sid = config["slave_id"]
        before_valve = read_input_real(client, IREG_VALVE_CMD, sid)
        before_lvl = read_input_real(client, IREG_LEVEL, sid)
        ueq = read_input_real(client, IREG_UEQ, sid)
        print(f"stop 前: 液位 {pct_to_mm(before_lvl, config):.1f}mm ({before_lvl:.1f}%)  "
              f"阀 {before_valve:.1f}%")
        print(f"  记下这个平衡开度: u_eq(当前目标) = {ueq:.1f}%  ← resume 会用它预置积分")

        # ---- 倒计时: 给操作员留出去关上方进水开关的时间 ----
        # 必须在写入之前倒数, 因为一旦切手动+阀0, 液位就只受上方自流支配了。
        d = max(0.0, args.delay)
        if d > 0:
            print(f"⏳ {d:.0f} 秒后关闭水泵与阀门。"
                  f"**请现在关闭设备上方左右桶连通的开关。**", flush=True)
            n = int(round(d))
            for k in range(n, 0, -1):
                print(f"   {k} ...", flush=True)
                time.sleep(1.0)
            frac = d - n
            if frac > 0.05:
                time.sleep(frac)

        write_real(client, REG_MANUAL_VALVE, 0.0, sid)      # 1. 手动阀归零
        set_control_bit(client, CTL_MANUAL_PUMP, False, sid)  # 2. 手动泵 OFF
        word = set_control_bit(client, CTL_AUTO, False, sid)  # 3. 最后切手动
        print(f"已写: rManualValve=0.0%  bManualPump=OFF  bAuto=OFF "
              f"(control word=0x{word:04X})")
        print("  现场动作: 阻断设备上方左右桶的连通(关阀/夹管/抬管)")

        time.sleep(0.4)                                     # 等 >=1 个 PLC 周期

        # ---- 漂移检查, 允许重试 ----
        # 操作员可能是在倒计时里才动手, 关阀动作会落在第一次检查窗口内, 所以
        # 第一次失败不一定是漏水/没关, 再查 recheck 次才下结论。
        tries = max(1, args.recheck + 1)
        ok = False
        for i in range(tries):
            ok, _ = _quiescent_report(client, config, settle=args.settle,
                                      samples=args.samples, last=(i == tries - 1))
            if ok or args.force:
                break
            if i < tries - 1:
                print(f"  {args.retry_wait:g}s 后复查 ({i + 2}/{tries}) ...", flush=True)
                time.sleep(max(0.5, args.retry_wait))
        if not ok and not args.force:
            sys.exit(2)
        print("下一步: 校准 → calibrate --sample --label low|mid|high ;  "
              "恢复运行 → resume")
    finally:
        client.close()


def cmd_resume(args, config):
    """从 stop 恢复到自动运行, 并预置积分避免冷启动超调。

    直接 `auto` 会让 rITerm 从 0 开始爬。自衡对象的平衡开度约 27%, 而积分在
    小误差下只能以 Ki*|e| ≈ 0.1pp/s 的速度累加, 爬满要 4~5 分钟 —— 这期间阀
    接近全关, 液位一路涨过目标。
    所以先把 rManualValve 写成 PLC 报出的 u_eq, 等一个周期让手动段把 rITerm
    跟踪到该值(无扰切换), 再置 bAuto。
    """
    client = connect_plc(config)
    try:
        sid = config["slave_id"]
        preload = args.valve
        if preload is None:
            preload = read_input_real(client, IREG_UEQ, sid)
            if preload <= 0.0:
                print("⚠️ PLC 报出的 u_eq = 0(前馈/观测器未映射或未学习), "
                      "无法预置积分。用 resume --valve <开度> 手动指定", file=sys.stderr)
        preload = max(0.0, min(100.0, preload))
        target = read_real(client, REG_TARGET_LEVEL, sid)
        lvl = read_input_real(client, IREG_LEVEL, sid)
        print(f"当前: 液位 {pct_to_mm(lvl, config):.1f}mm ({lvl:.1f}%)  "
              f"目标 {pct_to_mm(target, config):.1f}mm ({target:.1f}%)")
        print(f"预置: rManualValve = {preload:.1f}% (平衡开度 u_eq), "
              f"等 PLC 把 rITerm 跟踪到该值后再切自动")

        write_real(client, REG_MANUAL_VALVE, preload, sid)
        set_control_bit(client, CTL_MANUAL_PUMP, False, sid)
        sw0 = read_status_word(client, sid)
        if sw0 & ST_AUTO:
            # 已经是自动: 必须先落到手动, 手动段才会把 rITerm 跟踪到 preload
            set_control_bit(client, CTL_AUTO, False, sid)
            print("  (当前是自动模式, 先切手动以便预置积分)")
        time.sleep(max(0.5, args.hold))          # 手动段每拍跟踪 rITerm, 0.5s = 5 拍
        word = set_control_bit(client, CTL_AUTO, True, sid)
        print(f"已切自动 (control word=0x{word:04X})")
        print("  现场动作: 恢复设备上方左右桶的连通")

        time.sleep(1.0)
        sw = read_status_word(client, sid)
        valve = read_input_real(client, IREG_VALVE_CMD, sid)
        print(f"回读: 自动 {'ON' if (sw & ST_AUTO) else 'OFF'}  阀 {valve:.1f}%  "
              f"泵 {'ON' if (sw & ST_PUMP or sw & ST_PUMP_CMD) else 'OFF'}")
        if not (sw & ST_AUTO):
            print("⚠️ bAuto 没置上", file=sys.stderr)
            sys.exit(2)
    finally:
        client.close()


def cmd_set_pid(args, config):
    """Write PID gains and enable Modbus tuning (control word bit2 = bPidTune).

    写 Kp/Ki/Kd 到保持寄存器 2/4/12, 并置使能位, PLC 才采用这些值(否则用默认)。
    """
    kp, ki, kd = args.kp, args.ki, args.kd
    for name, v in (("kp", kp), ("ki", ki), ("kd", kd)):
        if v < 0:
            print(f"Error: {name} must be >= 0")
            sys.exit(1)
    client = connect_plc(config)
    try:
        write_real(client, REG_KP, kp, config["slave_id"])
        write_real(client, REG_KI, ki, config["slave_id"])
        write_real(client, REG_KD, kd, config["slave_id"])
        # 置使能位, PLC 才采用 Modbus 的 PID 参数
        word = set_control_bit(client, CTL_PID_TUNE, True, config["slave_id"])
        print(f"OK: Kp={kp:g} Ki={ki:g} Kd={kd:g}  (bPidTune ON, control word=0x{word:04X})")
        # 读回确认
        rkp = read_input_real(client, IREG_KP, config["slave_id"])
        rki = read_input_real(client, IREG_KI, config["slave_id"])
        rkd = read_input_real(client, IREG_KD, config["slave_id"])
        print(f"   PLC now using: Kp={rkp:.3f} Ki={rki:.3f} Kd={rkd:.3f}")
    finally:
        client.close()


def cmd_get_pid(args, config):
    """Read PID gains the PLC is actually using (input regs 10/12/14, feedback)."""
    client = connect_plc(config)
    try:
        rkp = read_input_real(client, IREG_KP, config["slave_id"])
        rki = read_input_real(client, IREG_KI, config["slave_id"])
        rkd = read_input_real(client, IREG_KD, config["slave_id"])
        cw = read_control_word(client, config["slave_id"])   # 控制字在保持寄存器 Device_10
        tune = "ON" if (cw & CTL_PID_TUNE) else "OFF"
        print(f"Kp={rkp:.3f}  Ki={rki:.3f}  Kd={rkd:.3f}   (Modbus tune: {tune})")
    finally:
        client.close()


def cmd_get_learn(args, config):
    """读 PLC 的前馈参数: 平衡线截距 C 与当前目标的平衡开度 u_eq。

    自衡对象: u_eq(SP) = C - SP/STAT_K。C 随两桶装水量整体平移, 由 PLC 在线学习;
    实测参考值 49(10-19) / 58(10-20) / 52.4(10-21)。
    """
    client = connect_plc(config)
    try:
        c_plc = read_input_real(client, IREG_UEQC, config["slave_id"])
        ueq = read_input_real(client, IREG_UEQ, config["slave_id"])
        level = read_input_real(client, IREG_LEVEL, config["slave_id"])
        valve = read_input_real(client, IREG_VALVE_CMD, config["slave_id"])
        target = read_real(client, REG_TARGET_LEVEL, config["slave_id"])
        print(f"C    = {c_plc:.4f} %    (平衡线截距, PLC 在线学习)")
        print(f"u_eq = {ueq:.4f} %    (目标 {target:.1f}% 对应的平衡开度)")
        print(f"当前: 液位 {level:.2f}% ({pct_to_mm(level, config):.1f}mm)  "
              f"阀指令 {valve:.2f}%")
        c_chk = ueq + target / STAT_K
        print(f"一致性校验: u_eq + SP/{STAT_K} = {c_chk:.2f}  "
              f"(应与 C 相符, 差 {c_chk - c_plc:+.2f})")
        if ueq > 1.0:
            print(f"  阀 {valve:.2f}% vs 平衡 {ueq:.2f}% → 液位将 "
                  f"{'下降' if valve > ueq + 0.5 else ('上升' if valve < ueq - 0.5 else '保持')}")
        print("  note: C 随两桶装水量平移, PLC 自动跟随(EMA 300s); "
              "要固化就抄进 ST 的 rUeqC 默认值")
    finally:
        client.close()


def read_level_raw(client: ModbusTcpClient, slave_id: int) -> int:
    """Read iLevelRaw (raw 4-20mA counts) from INPUT register slot 9.

    PLC 侧需有 qwModbusTCP_Server_Device_9 := INT_TO_WORD(iLevelRaw)。
    旧程序没映射时读回 0 —— 调用方据此判断"不可用"而不是当成真实水位。
    """
    result = client.read_input_registers(address=IREG_LEVEL_RAW, count=1, slave=slave_id)
    if result.isError():
        raise Exception(f"Read iLevelRaw error (FC04) at {IREG_LEVEL_RAW}: {result}")
    return result.registers[0]


def cmd_read_level_raw(args, config):
    """打印 iLevelRaw 原始计数 + 由标定表换算出的 % 与 mm, 供现场校准量程。

    校准流程: 量出实际水位 → 若与这里报的 mm 不符 → set-scale add <raw> <实测mm>。
    """
    client = connect_plc(config)
    try:
        raw = read_level_raw(client, config["slave_id"])
        if raw == 0:
            print("Error: PLC 回读的 iLevelRaw = 0, 说明当前程序未映射 "
                  "qwModbusTCP_Server_Device_9。请在 PLC_PRG 末尾加一行:\n"
                  "  qwModbusTCP_Server_Device_9 := INT_TO_WORD(iLevelRaw);\n"
                  "然后重新编译下载。")
            sys.exit(1)
        pct = (raw - 4000) / 16000.0 * 100.0
        mm = pct_to_mm(pct, config)
        print(f"iLevelRaw     = {raw}  (0x{raw:04X})")
        print(f"rLevel        = {pct:.2f} %")
        print(f"换算水位      = {mm:.1f} mm")
        print(f"模型          = {_describe_model(config)}")
        if not (4000 <= raw <= 20000):
            print(f"⚠️ 超出 4-20mA 范围 (4000..20000): 传感器断路/短路或量程配置不对",
                  file=sys.stderr)
        print("校准: 量一下实际水位, 若与上面不符 →")
        print(f"  python3 water_control_cli.py calibrate --actual <实测mm>")
    finally:
        client.close()


def _read_raw_settled(client, config, n=3, gap=1.0, force=False):
    """连读 n 次 iLevelRaw, 返回 (中位数, 抖动 mm)。抖动过大 = 水位没稳。"""
    sid = config["slave_id"]
    vals = []
    for i in range(n):
        vals.append(read_level_raw(client, sid))
        if i < n - 1:
            time.sleep(gap)
    vals.sort()
    med = vals[len(vals) // 2]
    spread = (vals[-1] - vals[0]) * _model_k(config)
    if spread > 3.0 and not force:
        raise SystemExit(
            f"Error: {n} 次读数抖动 {spread:.1f}mm (>3mm), 水位未稳, 拒绝校准。"
            "等 level/trend 平稳后重试, 或确认无误后加 --force")
    return med, spread


def _describe_model(config):
    m = _model(config)
    if not m:
        return "分段点表 (level_cal_points), 未启用线性模型"
    return f"线性 mm = {m['k']:.7f}*(raw-4000) + {m['b']:.2f}"


def _fit_line(pts):
    """最小二乘拟合 mm = k*(raw-4000) + b。pts = [(raw, mm), ...]，至少 2 点。

    返回 (k, b, residuals)。2 点时残差恒为 0(2 点解 2 未知数, 零冗余 —— 残差
    不构成任何检验); **3 点起才有 1 个自由度**, 残差才真正能查出量错的点或非线性。
    """
    xs = [float(r) - 4000.0 for r, _ in pts]
    ys = [float(m) for _, m in pts]
    n = len(pts)
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    den = n * sxx - sx * sx
    if abs(den) < 1e-9:
        return None, None, None
    k = (n * sxy - sx * sy) / den
    b = (sy - k * sx) / n
    res = [y - (k * x + b) for x, y in zip(xs, ys)]
    return k, b, res


def _confirm(prompt):
    try:
        return input(f"{prompt} [y/N] ").strip().lower() in ("y", "yes")
    except EOFError:
        return False


def _save_model(config, k, b):
    config["level_model"] = {"k": round(k, 9), "b": round(b, 4)}
    hist = config.get("level_cal_history", [])
    hist.append({"time": time.strftime("%Y-%m-%dT%H:%M:%S"),
                 "k": round(k, 9), "b": round(b, 4)})
    config["level_cal_history"] = hist[-20:]
    save_config(config)


def cmd_calibrate(args, config):
    """量程校准: 斜率 K 固定, 只用实测水位修正截距 b。

    模型 mm = K*(iLevelRaw-4000) + b。压力探头打湿/温度/零点漂移只会整体平移
    这条直线(改 b), 不会改斜率(改 K) —— 所以日常校准只动 b。
    K 只在换探头/改管路/出现"两端误差反号"时用 --two-point 重拟合。
    """
    cur = _model(config) or dict(DEFAULT_CONFIG["level_model"])

    if args.show:
        print(f"当前模型: {_describe_model(config)}")
        print(f"  K = {cur['k']:.7f} mm/计数  (1% = {cur['k'] * 160:.3f}mm, "
              f"16mA = {cur['k'] * 16000:.1f}mm)")
        print(f"  b = {cur['b']:.2f} mm   (4mA 对应的高度)")
        print("  4mA  -> raw  4000  -> %6.1f mm" % (cur["b"]))
        for mm in (50.0, 100.0, 150.0):
            raw = int(round(4000 + (mm - cur["b"]) / cur["k"]))
            print("  %-4gmm -> raw %5d  -> %6.2f%%" % (mm, raw, (raw - 4000) / 160.0))
        print("  20mA -> raw 20000  -> %6.1f mm" % (cur["b"] + cur["k"] * 16000))
        return

    if args.sample:
        client = connect_plc(config)
        try:
            sw = read_status_word(client, config["slave_id"])
            valve = read_input_real(client, IREG_VALVE_CMD, config["slave_id"])
            pump_on = bool(sw & ST_PUMP) or bool(sw & ST_PUMP_CMD)
            still = (not pump_on) and valve <= 1.0
            print(f"泵: {'ON' if pump_on else 'OFF'}   阀指令: {valve:.1f}%   "
                  f"{'静水状态 OK' if still else '⚠️ 不是静水状态'}")
            if not still and not args.force:
                sys.exit("Error: 静水两点校准要求泵 OFF 且阀 ≈0 (左侧也不能有自流)。\n"
                         "  先执行: python3 water_control_cli.py stop\n"
                         "  (它会关泵+关阀+切手动, 并回读确认液位真的不动了),\n"
                         "  再人工阻断上方左右桶连通。确认现场确实已无流动后加 --force")
            raw, spread = _read_raw_settled(client, config, n=5, gap=0.5,
                                           force=args.force)
            if raw == 0:
                sys.exit("Error: PLC 回读的 iLevelRaw = 0, 说明当前程序未映射 "
                         "qwModbusTCP_Server_Device_9。请在 PLC_PRG 末尾加一行:\n"
                         "  qwModbusTCP_Server_Device_9 := INT_TO_WORD(iLevelRaw);\n"
                         "然后重新编译下载。")
            label = (args.label or "POINT").upper()
            print(f"raw 中位数 = {raw}   (5 次抖动 {spread:.2f}mm)")
            print(f"记录: {label} raw={raw}  mm=<尺子读数>")
            print("三点都记好后(推荐 ~55 / ~100 / ~145mm):")
            print("  python3 water_control_cli.py calibrate --three-point "
                  "<raw1> <mm1> <raw2> <mm2> <raw3> <mm3>")
            print("  三点会给出逐点残差 —— 两点解两未知数残差恒为 0, 检不出量错的点")
        finally:
            client.close()
        return

    if args.two_point:
        raw1, mm1, raw2, mm2 = args.two_point
        if raw2 <= raw1 + 1000:
            sys.exit("Error: 两个点的 raw 相差太小 (<1000 计数), 无法拟合斜率; "
                     "把两个水位拉开到 60mm 以上")
        if abs(mm2 - mm1) < 20.0:
            sys.exit("Error: 两个实测水位相差 <20mm, 无法拟合斜率; "
                     "建议低点 ~60mm、高点 ~140mm, 让两点把工作区间夹住")
        if not (4000 <= raw1 <= 20000 and 4000 <= raw2 <= 20000):
            sys.exit("Error: raw 超出 4-20mA 范围 (4000..20000)")
        for mm in (mm1, mm2):
            if not 20.0 <= mm <= 200.0:
                sys.exit(f"Error: 实测水位 {mm:.1f}mm 不合理 (应在 20-200mm)")
        k_new = (mm2 - mm1) / (raw2 - raw1)
        b_new = mm1 - k_new * (raw1 - 4000)
        ratio = k_new / cur["k"]
        print(f"现模型: {_describe_model(config)}")
        print(f"新拟合: K={k_new:.7f} mm/计数 (原 {cur['k']:.7f}, 比值 {ratio:.3f}), "
              f"b={b_new:.2f} mm")
        print(f"  跨度检查: raw {int(raw1)}->{int(raw2)} ({int(raw2 - raw1)} 计数), "
              f"{mm1:.1f}->{mm2:.1f}mm; 端点 4mA={b_new:.1f}mm  "
              f"20mA={b_new + k_new * 16000:.1f}mm")
        print("  [!] 两点解两未知数, 残差恒为 0, 拟合优度无法检验。"
              "有条件请用 --three-point(多一个点才有一个自由度)")
        if not 0.7 <= ratio <= 1.3 and not args.force:
            sys.exit(f"Error: 新斜率与原值差 {abs(ratio - 1) * 100:.0f}%, 超过 30% 的"
                     "自动接受范围。若是首次按静水两点法重建(旧点表本就不可信), "
                     "加 --force 接受; 否则先核对测量基准/是否量错桶")
        if not args.yes and not _confirm("写入新的 K 和 b?"):
            print("已取消, 配置未改动")
            return
        _save_model(config, k_new, b_new)
        print("已写入配置")
        return

    if args.three_point:
        v = args.three_point
        pts = sorted([(v[0], v[1]), (v[2], v[3]), (v[4], v[5])])
        raws = [r for r, _ in pts]
        mms = [m for _, m in pts]
        if len(set(raws)) < 3:
            sys.exit("Error: 三个 raw 必须互不相同")
        for r in raws:
            if not 4000 <= r <= 20000:
                sys.exit(f"Error: raw {int(r)} 超出 4-20mA 范围 (4000..20000)")
        for m in mms:
            if not 20.0 <= m <= 200.0:
                sys.exit(f"Error: 实测水位 {m:.1f}mm 不合理 (应在 20-200mm)")
        if raws[-1] - raws[0] < 4000:
            sys.exit(f"Error: 三点跨度只有 {int(raws[-1] - raws[0])} 计数 (<4000, "
                     f"约 51mm), 斜率分辨不出来。建议 ~55 / ~100 / ~145mm")
        if mms[-1] - mms[0] < 50.0:
            sys.exit(f"Error: 三点实测跨度只有 {mms[-1] - mms[0]:.1f}mm (<50mm)。"
                     "建议 ~55 / ~100 / ~145mm, 把 50-150mm 工作区间夹住")
        if mms != sorted(mms):
            sys.exit("Error: raw 递增而实测 mm 不递增 —— 有点量错了或记串了")
        # 相邻两点间距不能太悬殊, 否则中间那点几乎不提供约束
        g1, g2 = raws[1] - raws[0], raws[2] - raws[1]
        if min(g1, g2) < 0.25 * max(g1, g2):
            print(f"[!] 三点分布不均 (间距 {g1:.0f} / {g2:.0f} 计数), "
                  "靠得近的那两点几乎决定不了斜率", file=sys.stderr)

        k_new, b_new, res = _fit_line(pts)
        if k_new is None:
            sys.exit("Error: 三点共线于同一 raw, 无法拟合")
        ratio = k_new / cur["k"]
        print(f"现模型: {_describe_model(config)}")
        print(f"三点最小二乘: K={k_new:.7f} mm/计数 (原 {cur['k']:.7f}, 比值 {ratio:.3f}), "
              f"b={b_new:.2f} mm")
        print("  逐点残差 (这才是三点的意义 —— 两点时恒为 0, 检不出任何东西):")
        worst = 0
        for i, ((r, m), e) in enumerate(zip(pts, res), 1):
            flag = ""
            if abs(e) > 1.5:
                flag = "  <-- 超差"
            print(f"    P{i}: raw={int(r):5d}  实测 {m:6.2f}mm  拟合 "
                  f"{k_new * (r - 4000) + b_new:6.2f}mm  残差 {e:+6.2f}mm{flag}")
            if abs(e) > abs(res[worst]):
                worst = i - 1
        rmax = max(abs(e) for e in res)
        print(f"    最大残差 {rmax:.2f}mm   端点 4mA={b_new:.1f}mm  "
              f"20mA={b_new + k_new * 16000:.1f}mm")
        if rmax > 1.5 and not args.force:
            (wr, wm) = pts[worst]
            sys.exit(
                f"Error: 最大残差 {rmax:.2f}mm 超过 1.5mm。三点应当落在一条直线上, "
                f"超差说明 P{worst + 1}(raw={int(wr)}, 实测 {wm:.2f}mm) 量错了、"
                "水面没静、或探头在这一段确实非线性。\n"
                "  静水抖动底是 0.4~0.5mm, 1.5mm 已经是它的 3 倍。"
                "重量那一点, 或确认无误后加 --force 强行写入")
        if not 0.7 <= ratio <= 1.3 and not args.force:
            sys.exit(f"Error: 新斜率与原值差 {abs(ratio - 1) * 100:.0f}%, 超过 30% 的"
                     "自动接受范围。加 --force 接受, 或先核对测量基准")
        db = b_new - cur["b"]
        print(f"  Δb = {db:+.2f}mm   ΔK = {(ratio - 1) * 100:+.1f}%")
        if abs(db) > 40.0:
            sys.exit(f"Error: Δb={db:+.2f}mm 超过 40mm 上限。这个量级通常意味着"
                     "探头还湿着/通气管积水、测量基准变了、或量错了桶")
        if abs(db) > 25.0 and not args.force:
            print(f"[!] Δb 达 {db:+.2f}mm, 变化很大; 确认基准没变后加 --force 重试",
                  file=sys.stderr)
            sys.exit(2)
        if not args.yes and not _confirm("写入新的 K 和 b?"):
            print("已取消, 配置未改动")
            return
        _save_model(config, k_new, b_new)
        print("已写入配置")
        return

    if args.actual is None:
        sys.exit("Error: 需要 --actual <实测mm>; 查看当前模型用 --show, "
                 "只校验不写入用 --check <实测mm>")

    mm_true = args.actual
    client = connect_plc(config)
    try:
        raw, spread = _read_raw_settled(client, config, force=args.force)
        if not 4000 <= raw <= 20000:
            sys.exit(f"Error: iLevelRaw={raw} 超出 4-20mA 范围, 先查传感器接线")
        mm_now = cur["k"] * (raw - 4000) + cur["b"]
        if not 50.0 <= mm_now <= 150.0:
            sys.exit(f"Error: 当前显示 {mm_now:.1f}mm 不在 50-150mm 窗口内; "
                     "请先把水位调到窗口中部 (建议 100mm) 再校准")
        if not 20.0 <= mm_true <= 200.0:
            sys.exit(f"Error: 实测值 {mm_true:.1f}mm 不合理 (应在 20-200mm)")

        b_new = mm_true - cur["k"] * (raw - 4000)
        delta = b_new - cur["b"]
        print(f"现模型: {_describe_model(config)}")
        print(f"读数:   iLevelRaw={raw}  (3 次采样抖动 {spread:.2f}mm)")
        print(f"当前显示 {mm_now:.1f}mm   实测 {mm_true:.1f}mm   "
              f"→ 误差 {mm_true - mm_now:+.1f}mm")
        print(f"修正:   Δb = {delta:+.1f}mm   →  b={b_new:.2f}mm (K 不变)")
        print(f"  端点: 4mA {cur['b']:.1f} → {b_new:.1f}mm,  "
              f"20mA {cur['b'] + cur['k'] * 16000:.1f} → "
              f"{b_new + cur['k'] * 16000:.1f}mm")
        if abs(delta) > 40.0:
            sys.exit(f"Error: Δb={delta:+.1f}mm 超过 40mm 上限。这个量级通常意味着: "
                     "探头还湿着/通气管积水、测量基准变了、或量错了桶。请先排查")
        if args.check:
            print("(--check: 只报告, 不写入配置)")
            return
        if abs(delta) > 25.0 and not args.force:
            print(f"⚠️ Δb 达 {delta:+.1f}mm, 变化很大; 确认基准没变后加 --force 重试",
                  file=sys.stderr)
            sys.exit(2)
        if not args.yes and not _confirm("写入配置?"):
            print("已取消, 配置未改动")
            return
        _save_model(config, cur["k"], b_new)
        print(f"已写入: b={b_new:.2f}mm  ({CONFIG_FILE})")
    finally:
        client.close()


def cmd_status(args, config):
    """Show all PLC status."""
    client = connect_plc(config)
    try:
        target = read_real(client, REG_TARGET_LEVEL, config["slave_id"])
        level = read_input_real(client, IREG_LEVEL, config["slave_id"])
        valve = read_input_real(client, IREG_VALVE_CMD, config["slave_id"])
        # 状态位从打包字读取(FC04), 替代读线圈: 位没单独映射到线圈, 线圈值不可信
        sw = read_status_word(client, config["slave_id"])
        b_auto = bool(sw & ST_AUTO)
        b_pumpcmd = bool(sw & ST_PUMP_CMD)
        b_fault = bool(sw & ST_SENSOR_FAULT)
        b_alarm = bool(sw & ST_ALARM)
        b_pump = bool(sw & ST_PUMP)

        lo, hi = _cal_range_mm(config)
        rkp = read_input_real(client, IREG_KP, config["slave_id"])
        rki = read_input_real(client, IREG_KI, config["slave_id"])
        rkd = read_input_real(client, IREG_KD, config["slave_id"])
        print(f"Target Level:  {target:.1f}%  ({pct_to_mm(target, config):.1f} mm)")
        print(f"Current Level: {level:.1f}%  ({pct_to_mm(level, config):.1f} mm)")
        print(f"Valve Command: {valve:.1f}%")
        print(f"Auto Mode:     {'ON' if b_auto else 'OFF'}")
        print(f"Pump Cmd:      {'ON' if b_pumpcmd else 'OFF'}")
        print(f"Pump Output:   {'ON' if b_pump else 'OFF'}  (%QX4.0)")
        print(f"Sensor Fault:  {'YES' if b_fault else 'NO'}")
        print(f"Alarm:         {'YES' if b_alarm else 'NO'}")
        print(f"PID:           Kp={rkp:.3f} Ki={rki:.3f} Kd={rkd:.3f}")
        try:
            iraw = read_level_raw(client, config["slave_id"])
        except Exception:
            iraw = 0
        if iraw:
            print(f"iLevelRaw:     {iraw}  (calibrate with: calibrate --actual <measured mm>)")
        else:
            print("iLevelRaw:     n/a  (PLC 程序未映射 Device_9)")
        print(f"(cal: {_describe_model(config)}, range {lo:g}-{hi:g} mm, "
              f"status word=0x{sw:02X})")
    finally:
        client.close()


def cmd_trend(args, config):
    """趋势记录: 周期采样液位/阀指令/泵状态, 写 CSV 并逐行打印。

    用于观察漂移与振荡: 波动带、周期、阀-液位相位关系。
    Ctrl+C 提前结束仍会写出已采数据与统计。
    """
    duration = args.duration
    interval = max(0.5, args.interval)
    out_path = args.out or os.path.join(
        os.getcwd(), f"trend_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    sid = config["slave_id"]
    client = connect_plc(config)
    rows = []
    print(f"记录 {duration:.0f}s, 间隔 {interval:g}s -> {out_path}  (Ctrl+C 提前结束)")
    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            if t >= duration:
                break
            target = read_real(client, REG_TARGET_LEVEL, sid)
            level = read_input_real(client, IREG_LEVEL, sid)
            valve = read_input_real(client, IREG_VALVE_CMD, sid)
            sw = read_status_word(client, sid)
            mm = pct_to_mm(level, config)
            rows.append((f"{t:.1f}", f"{mm:.2f}", f"{level:.2f}",
                         f"{valve:.2f}", f"{target:.2f}", f"0x{sw:04X}"))
            print(f"  t={t:6.1f}s  level={mm:6.1f}mm ({level:5.1f}%)  "
                  f"valve={valve:5.1f}%  pump={'ON ' if sw & ST_PUMP else 'OFF'}")
            # 扣除本次读取耗时, 保持采样间隔稳定
            time.sleep(max(0.0, t + interval - (time.time() - t0)))
    except KeyboardInterrupt:
        print("\n中断, 统计已采集数据")
    finally:
        client.close()

    if rows:
        with open(out_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "level_mm", "level_pct", "valve_pct",
                        "target_pct", "status"])
            w.writerows(rows)
        ts = [float(r[0]) for r in rows]
        mms = [float(r[1]) for r in rows]
        pcts = [float(r[2]) for r in rows]
        valves = [float(r[3]) for r in rows]
        n = len(rows)

        def _slope(ys):
            """最小二乘斜率 dy/dt (纯标准库)。"""
            mt, my = sum(ts) / n, sum(ys) / n
            sxx = sum((t - mt) ** 2 for t in ts)
            sxy = sum((t - mt) * (y - my) for t, y in zip(ts, ys))
            return sxy / sxx if sxx else 0.0

        print(f"\n采样 {n} 点, 时长 {ts[-1]:.0f}s -> {out_path}")
        print(f"液位: min={min(mms):.1f}  max={max(mms):.1f}  "
              f"mean={sum(mms)/n:.1f} mm  波动带={max(mms)-min(mms):.1f} mm")
        print(f"液位速率(拟合): {_slope(mms):+.3f} mm/s  ({_slope(pcts):+.4f} %/s)"
              f"   <- 阀全关时即自然上涨速率 d0")
        print(f"阀指令: min={min(valves):.1f}  max={max(valves):.1f}  "
              f"mean={sum(valves)/n:.1f}%  摆幅={max(valves)-min(valves):.1f} pp"
              f"   <- 稳态时 mean 即平衡开度 u_eq")
    else:
        print("未采集到数据")


def cmd_set_ip(args, config):
    """Set PLC IP address."""
    config["plc_ip"] = args.ip
    save_config(config)
    print(f"PLC IP set to {args.ip}")


def cmd_set_scale(args, config):
    """Manage piecewise calibration points: (iLevelRaw, mm).

    set-scale                # 显示当前标定点表
    set-scale add <raw> <mm> # 加一个实测点 (iLevelRaw, 物理 mm), 自动去重排序
    set-scale del <raw>      # 删除某 raw 的点
    set-scale reset          # 恢复默认点表

    标定方法: 注水到已知高度 -> 读 PLC iLevelRaw(可在 Machine Expert 看或由
    rLevel 反算 raw=rLevel/100*16000+4000) -> set-scale add <raw> <实测mm>。
    至少 2 个点生效, 点越多段越细越准。改后所有 mm 命令自动跟进, 不动 PLC 程序。
    """
    sub = args.sub
    pts = config.get("level_cal_points", DEFAULT_CONFIG["level_cal_points"])

    if sub is None:
        # 显示
        pts_sorted = sorted(pts, key=lambda p: p[0])
        print("Calibration points (iLevelRaw, mm):")
        for raw, mm in pts_sorted:
            rlevel = (raw - 4000) / 16000.0 * 100.0
            print(f"  raw={raw:<6} (rLevel={rlevel:5.1f}%)  ->  {mm:g} mm")
        if len(pts_sorted) < 2:
            print("  WARN: <2 points, need at least 2 for interpolation")
        return

    if sub == "add":
        raw = args.raw; mm = args.mm
        if raw < 0 or raw > 65535:
            print("Error: iLevelRaw must be 0..65535"); sys.exit(1)
        # 去重: 同 raw 替换
        pts = [[r, m] for r, m in pts if r != raw]
        pts.append([raw, mm])
        config["level_cal_points"] = sorted(pts, key=lambda p: p[0])
        save_config(config)
        print(f"Added point: raw={raw} -> {mm:g} mm  (now {len(config['level_cal_points'])} points)")

    elif sub == "del":
        raw = args.raw
        before = len(pts)
        pts = [[r, m] for r, m in pts if r != raw]
        config["level_cal_points"] = pts
        save_config(config)
        print(f"Deleted raw={raw}  ({before} -> {len(pts)} points)")

    elif sub == "reset":
        config["level_cal_points"] = DEFAULT_CONFIG["level_cal_points"]
        save_config(config)
        print(f"Reset to default {len(DEFAULT_CONFIG['level_cal_points'])} points")

    else:
        print(f"Error: unknown subcommand '{sub}'"); sys.exit(1)


def _add_wait_opts(p):
    """给"设定目标"类子命令加上 --wait 及其参数。

    默认 timeout=100s 是刻意的: 要留在 Bash 工具默认 120s 超时之内, 这样 agent
    一次调用就能拿到结论, 不必自己写轮询循环(那会导致它每轮都向用户发一句
    "还在等待")。需要更久就显式传 --timeout, 并同步放大 Bash 工具的超时。
    """
    p.add_argument("--wait", action="store_true",
                   help="block until the level settles within --tolerance of the "
                        "target (exit 0) or --timeout elapses (exit 3); polls "
                        "internally so the caller needs only ONE invocation")
    p.add_argument("--tolerance", type=float, default=3.0,
                   help="settled band around the target in mm (default 3.0; the "
                        "PLC has a +-1.4mm steady-state deadband by design)")
    p.add_argument("--timeout", type=float, default=100.0,
                   help="max seconds to wait (default 100, keep it under the "
                        "caller's own tool timeout)")
    p.add_argument("--interval", type=float, default=2.0,
                   help="poll interval in seconds (default 2)")
    p.add_argument("--confirm", type=int, default=3,
                   help="consecutive in-band reads required to call it settled "
                        "(default 3 = 6s, guards against a ripple crossing)")


def main():
    parser = argparse.ArgumentParser(
        prog="water-control",
        description="Water Level Control CLI for Schneider PLC"
    )
    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # read command
    subparsers.add_parser("read", help="Read current target level (percent)")

    # read-mm command
    subparsers.add_parser("read-mm", help="Read current ACTUAL level in mm")

    # write command (percent)
    write_parser = subparsers.add_parser("write", help="Set target level 0-100 (percent)")
    write_parser.add_argument("value", type=float, help="Target level 0-100")

    # set-mm command (absolute mm)
    setmm_parser = subparsers.add_parser("set-mm", help="Set target level in mm (absolute)")
    setmm_parser.add_argument("value", type=float, help="Target level in mm (0..full-scale)")
    _add_wait_opts(setmm_parser)

    # raise / lower commands (delta mm)
    raise_parser = subparsers.add_parser("raise", help="Raise target by delta mm")
    raise_parser.add_argument("value", type=float, help="Delta in mm (e.g. 20 for +20mm)")
    _add_wait_opts(raise_parser)
    lower_parser = subparsers.add_parser("lower", help="Lower target by delta mm")
    lower_parser.add_argument("value", type=float, help="Delta in mm (e.g. 15 for -15mm)")
    _add_wait_opts(lower_parser)

    # status command
    subparsers.add_parser("status", help="Show all PLC status (with mm)")

    # trend command (level/valve logging for drift & oscillation diagnosis)
    trend_parser = subparsers.add_parser("trend", help="Record level/valve trend to CSV")
    trend_parser.add_argument("--duration", type=float, default=300.0,
                              help="record duration in seconds (default 300)")
    trend_parser.add_argument("--interval", type=float, default=2.0,
                              help="sample interval in seconds (default 2)")
    trend_parser.add_argument("--out", type=str, default=None,
                              help="CSV output path (default ./trend_<ts>.csv)")

    # calibrate command (level range calibration: fix slope K, adjust offset b)
    cal_p = subparsers.add_parser(
        "calibrate",
        help="Calibrate level range: fix K, adjust offset b from a measured level")
    cal_p.add_argument("--actual", type=float, default=None,
                       help="operator-measured true level in mm (single-point calibration)")
    cal_p.add_argument("--check", action="store_true",
                       help="report the error only, do not write config")
    cal_p.add_argument("--show", action="store_true",
                       help="show current K/b model and table")
    cal_p.add_argument("--two-point", type=float, nargs=4, default=None,
                       metavar=("RAW1", "MM1", "RAW2", "MM2"),
                       help="refit K and b from two (raw, measured mm) pairs "
                            "(zero redundancy: residuals are always 0)")
    cal_p.add_argument("--three-point", type=float, nargs=6, default=None,
                       metavar=("RAW1", "MM1", "RAW2", "MM2", "RAW3", "MM3"),
                       help="PREFERRED: refit K and b from three pairs by least "
                            "squares and report per-point residuals (refuses if "
                            "any residual exceeds 1.5mm)")
    cal_p.add_argument("--sample", action="store_true",
                       help="quiescent sampling for two/three-point calibration "
                            "(refuses if the pump runs or the valve is open)")
    cal_p.add_argument("--label", default=None,
                       help="label printed by --sample (e.g. low / high)")
    cal_p.add_argument("--yes", action="store_true", help="write without confirmation")
    cal_p.add_argument("--force", action="store_true",
                       help="bypass the >3mm jitter, >1.5mm fit-residual, "
                            ">30%% slope-change and >25mm shift guards")

    # read-level-raw command (calibration: raw 4-20mA counts)
    subparsers.add_parser("read-level-raw",
                          help="Read iLevelRaw counts + derived percent/mm (for calibration)")

    # read-raw command (debug byte order / address)
    raw_parser = subparsers.add_parser("read-raw", help="Dump raw registers for debugging")
    raw_parser.add_argument("address", type=int, nargs="?", help="Start address (default %%MW0)")

    # auto / manual commands
    subparsers.add_parser("auto", help="Switch to auto mode (control word bit0)")
    subparsers.add_parser("manual", help="Switch to manual mode (control word bit0)")

    # stop / resume: full quiescent state (pump OFF + valve 0% + manual)
    stop_p = subparsers.add_parser(
        "stop",
        help="Quiesce the rig: pump OFF + valve 0%% + manual (block the upper "
             "link by hand), then verify the level really stops moving")
    stop_p.add_argument("--delay", type=float, default=5.0,
                        help="seconds of countdown before closing the pump/valve, "
                             "so the operator can shut the upper link between the "
                             "two tanks (default 5, 0 = act immediately). The agent "
                             "must announce this BEFORE calling stop")
    stop_p.add_argument("--settle", type=float, default=8.0,
                        help="seconds of drift checking after stopping (default 8, 0=skip)")
    stop_p.add_argument("--samples", type=int, default=5,
                        help="level samples taken during each drift check (default 5)")
    stop_p.add_argument("--recheck", type=int, default=2,
                        help="extra drift checks if the first one fails (default 2) — "
                             "the operator may still be closing the upper valve "
                             "during the first window")
    stop_p.add_argument("--retry-wait", type=float, default=10.0,
                        help="seconds to wait between drift rechecks (default 10)")
    stop_p.add_argument("--force", action="store_true",
                        help="exit 0 even if the rig is not quiescent")
    resume_p = subparsers.add_parser(
        "resume",
        help="Leave the stopped state: preload the integral with u_eq, then switch "
             "to auto (plain 'auto' would cold-start the integral from 0)")
    resume_p.add_argument("--valve", type=float, default=None,
                          help="override the preload opening in percent "
                               "(default: u_eq reported by the PLC)")
    resume_p.add_argument("--hold", type=float, default=0.5,
                          help="seconds to hold manual before switching to auto "
                               "(default 0.5 = 5 PLC cycles)")

    # pump command (manual pump on/off)
    pump_parser = subparsers.add_parser("pump", help="Manual pump on/off (manual mode only)")
    pump_parser.add_argument("state", choices=["on", "off"], help="on or off")

    # valve command (manual valve opening)
    valve_parser = subparsers.add_parser("valve", help="Manual valve opening 0-100 (manual mode only)")
    valve_parser.add_argument("value", type=float, help="Valve opening 0-100")

    # set-pid / get-pid commands
    setpid_parser = subparsers.add_parser("set-pid", help="Set PID gains Kp Ki Kd (enables Modbus tuning)")
    setpid_parser.add_argument("kp", type=float, help="Proportional gain")
    setpid_parser.add_argument("ki", type=float, help="Integral gain")
    setpid_parser.add_argument("kd", type=float, help="Derivative gain")
    subparsers.add_parser("get-pid", help="Read PID gains the PLC is using")
    subparsers.add_parser("get-learn",
                          help="Read whatever the PLC maps into input regs 0/1 and 6/7 "
                               "(labelled C / u_eq; only correct for PLC_PRG_simple and _v3)")

    # set-ip command
    ip_parser = subparsers.add_parser("set-ip", help="Set PLC IP address")
    ip_parser.add_argument("ip", type=str, help="PLC IP address")

    # set-scale command (piecewise calibration point management)
    scale_parser = subparsers.add_parser("set-scale", help="Manage calibration points (piecewise interp)")
    scale_sub = scale_parser.add_subparsers(dest="sub")
    scale_sub.add_parser("reset", help="Reset to default calibration points")
    add_p = scale_sub.add_parser("add", help="Add a point: set-scale add <iLevelRaw> <mm>")
    add_p.add_argument("raw", type=int, help="PLC iLevelRaw (4000..20000)")
    add_p.add_argument("mm", type=float, help="measured water level in mm")
    del_p = scale_sub.add_parser("del", help="Delete a point by iLevelRaw")
    del_p.add_argument("raw", type=int, help="iLevelRaw to remove")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        sys.exit(1)

    config = load_config()

    commands = {
        "read": cmd_read,
        "read-mm": cmd_read_mm,
        "write": cmd_write,
        "set-mm": cmd_set_mm,
        "raise": cmd_raise_lower,
        "lower": cmd_raise_lower,
        "status": cmd_status,
        "trend": cmd_trend,
        "read-raw": cmd_read_raw,
        "read-level-raw": cmd_read_level_raw,
        "calibrate": cmd_calibrate,
        "auto": cmd_auto,
        "manual": cmd_manual,
        "stop": cmd_stop,
        "resume": cmd_resume,
        "pump": cmd_pump,
        "valve": cmd_valve,
        "set-pid": cmd_set_pid,
        "get-pid": cmd_get_pid,
        "get-learn": cmd_get_learn,
        "set-ip": cmd_set_ip,
        "set-scale": cmd_set_scale,
    }

    commands[args.command](args, config)


if __name__ == "__main__":
    main()
