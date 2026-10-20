"""
water-control-cli - Water Level Control CLI for Schneider PLC

A command-line tool to control water level via Modbus TCP.
Communicates with Schneider TM310MM35ECT16U PLC.

Usage:
    water-control read                  Read current target level (percent)
    water-control read-mm               Read current ACTUAL level (mm)
    water-control write <value>         Set target level (0-100 percent)
    water-control set-mm <mm>           Set target level in mm (absolute)
    water-control raise <mm>            Raise target by mm (e.g. raise 20 for +20mm)
    water-control lower <mm>            Lower target by mm (e.g. lower 15 for -15mm)
    water-control status                Show all PLC status (with mm)
    water-control set-pid <kp> <ki> <kd>  Set PID gains (enables Modbus tuning)
    water-control get-pid               Read PID gains the PLC is using
    water-control set-ip <ip>           Set PLC IP address
    water-control set-scale             Show calibration points (run with no subcmd)
    water-control set-scale add <raw> <mm>  Add a calibration point (iLevelRaw, mm)
    water-control set-scale del <raw>   Delete a calibration point
    water-control set-scale reset       Reset to default calibration points

Calibration (piecewise linear, multi-point) is in ~/.water-control/config.json as
"level_cal_points": list of [iLevelRaw, mm] measured pairs. PLC stays in percent;
CLI converts mm<->% via interpolation so the PLC program never changes.
Device limit: pump can only draw water down to ~43mm (PLC ~10.9%); below that
the pump cannot evacuate further even though the sensor reads lower.
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
# [G12] PLC 自学习值回读 (ST 把学习值映射到输入寄存器空闲槽 0/1 与 6/7)
IREG_INFLOW = 0          # 学习到的自然上涨速率 d0 (%/s) 只读
IREG_UEQ = 6             # 学习到的平衡阀开度 u_eq (%) 只读

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

def _cal_points(config: dict) -> list:
    """返回 [(iLevelRaw, mm), ...], 按 iLevelRaw 升序。"""
    pts = config.get("level_cal_points", DEFAULT_CONFIG["level_cal_points"])
    return sorted(pts, key=lambda p: p[0])


def _cal_range_mm(config: dict):
    """返回 (min_mm, max_mm) 标定范围(首末点), 用于命令参数范围校验。"""
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
    """毫米 -> PLC 百分比 (分段插值反解)。"""
    pts = _cal_points(config)
    raw = _interp_raw(mm, pts)
    return (raw - 4000) / 16000.0 * 100.0


def pct_to_mm(pct: float, config: dict) -> float:
    """PLC 百分比 -> 毫米 (分段插值)。"""
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
    """读 PLC 自学习到的 d0 / u_eq (输入寄存器空闲槽 0/1 与 6/7, [G12])。

    用于观察自学习收敛情况, 以及把学到的值抄进 ST 默认值长期固化。
    """
    client = connect_plc(config)
    try:
        d0 = read_input_real(client, IREG_INFLOW, config["slave_id"])
        ueq = read_input_real(client, IREG_UEQ, config["slave_id"])
        print(f"学习到的 d0   = {d0:.4f} %/s   (自然上涨速率)")
        print(f"学习到的 u_eq = {ueq:.4f} %     (平衡阀开度)")
        if ueq > 1.0 and d0 > 0.005:
            print(f"  由此推得工作带对象增益 K = d0/u_eq = {d0 / ueq:.5f} "
                  f"%/s per %阀 (step_test 实测 K 应与此同量级)")
        print("  note: 未收敛/未启用自学习时显示默认值; 满意后可把此值写入")
        print("        st/PLC_PRG_improved.st 的 rInflowRate / rUeqEst 默认值")
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
        npts = len(_cal_points(config))
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
        print(f"(cal: {npts}-pt interp, range {lo:g}-{hi:g} mm, status word=0x{sw:02X})")
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

    # raise / lower commands (delta mm)
    raise_parser = subparsers.add_parser("raise", help="Raise target by delta mm")
    raise_parser.add_argument("value", type=float, help="Delta in mm (e.g. 20 for +20mm)")
    lower_parser = subparsers.add_parser("lower", help="Lower target by delta mm")
    lower_parser.add_argument("value", type=float, help="Delta in mm (e.g. 15 for -15mm)")

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

    # read-raw command (debug byte order / address)
    raw_parser = subparsers.add_parser("read-raw", help="Dump raw registers for debugging")
    raw_parser.add_argument("address", type=int, nargs="?", help="Start address (default %%MW0)")

    # auto / manual commands
    subparsers.add_parser("auto", help="Switch to auto mode (control word bit0)")
    subparsers.add_parser("manual", help="Switch to manual mode (control word bit0)")

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
    subparsers.add_parser("get-learn", help="Read PLC self-learned d0 / u_eq")

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
        "auto": cmd_auto,
        "manual": cmd_manual,
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
