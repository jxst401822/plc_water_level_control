# PLC 水位控制系统 (plc_water_level_control)

基于施耐德 TM310MM35ECT16U PLC 的双桶闭环水位控制系统。PLC 侧运行 ST(结构化文本)PID 控制程序,上位机侧提供 Python CLI,通过 Modbus TCP 设定目标水位、在线整定 PID、监控运行状态。

## 系统概述

```
  左桶(高位)                右桶(低位,被控对象)
  ┌─────────┐  上管道(重力自流→)  ┌─────────┐
  │  水+泵  │───────────────────→│  水      │
  │         │                    │ 水位传感器│
  └─────────┘←───────────────────└─────────┘
            下管道(泵抽水←) + [阀门]
```

- **被控量**:右桶水位(4–20mA 水位传感器,`%IW3`)。
- **执行器**:抽水阀开度(`%QW3`,4–20mA)+ 抽水泵启停(`%QX4.0`,继电器)。
- **控制策略**:PID(位置式,误差 = PV − SP,阀在抽水侧故取正增益)+ HH/LL 联锁保护 + 泵启停迟滞。
- 上管道重力自流是持续扰动,稳态需积分项平衡;系统只能主动降水位,升水位靠自流(被动)。

详细控制原理、IO 映射、标定与整定步骤见 [docs/控制方案.md](docs/控制方案.md)。

## 目录结构

```
├── docs/                        # 文档
│   ├── product.md               # 产品设备信息(PLC/IO 模块/传感器/执行器)
│   ├── 控制方案.md               # 控制原理、IO 映射、Machine Expert 配置、整定步骤
│   └── ST侧改进说明.md           # ST 程序抗超调改进(斜坡/滤波/限速/去抖)与验证清单
├── st/                          # Machine Expert 工程文件(.project 及恢复文件)
│   └── PLC_PRG_improved.st      # 改进版 PLC_PRG 完整源码(整段替换用)
├── skill/water-level-control/   # 上位机 CLI(Agent Skill 形式)
│   ├── SKILL.md                 # Skill 说明:命令、Modbus 寄存器映射、整定经验
│   └── scripts/
│       ├── water_control_cli.py # 主 CLI:读写水位/PID/标定/手动模式
│       └── step_test.py         # 阀阶跃试验:实测 K/τ/d0,输出 SIMC 推荐 PID 增益
└── output/                      # 运行输出目录(不入库)
```

## 硬件与 IO 映射

| 信号 | 模块 | 地址 | 量程 | 方向 |
|---|---|---|---|---|
| 水位测量 | TM3AI4 ch0 | `%IW3` | 4–20mA → INT 4000..20000 | 输入 |
| 抽水阀开度 | TM3AQ2 ch0 | `%QW3` | 4–20mA → INT 4000..20000 | 输出 |
| 抽水泵启停 | TM3DQ16R Q0 | `%QX4.0` | BOOL(继电器) | 输出 |
| 数字输入备用 | TM3DI16 | — | — | 输入 |

HMI/Modbus 侧使用 `%MW` 区(目标水位 `%MW100-101`、实测水位 `%MW110-111` 等),与 Modbus TCP 服务器寄存器数组的映射在 ST 程序中显式完成,详见 [SKILL.md](skill/water-level-control/SKILL.md) 的 "Modbus Architecture" 一节。

## PLC 侧(Machine Expert)

1. 设备树挂接 TM3DI16 / TM3DQ16R / TM3AI4 / TM3AQ2,核对地址偏移。
2. TM3AI4/TM3AQ2 通道配置为电流型 4–20mA。
3. 导入 `st/` 下的 Machine Expert 工程(或按 docs 新建 POU `PRG_LevelControl`,ST 语言)。
4. 新建周期任务 `T#100ms` 调用控制程序(勿放 MAST)。
5. 启用 Modbus TCP 从站(slave ID = 1),配置保持/输入寄存器数组。

完整步骤见 [docs/控制方案.md](docs/控制方案.md) 第 3 节。

## 上位机 CLI

依赖:Python 3 + `pymodbus`(目标设备为 3.8.6,3.9+ 需将 `slave=` 改为 `device_id=`)。目标设备无 pip,直接以脚本方式运行:

```bash
cd skill/water-level-control/scripts

python3 water_control_cli.py status          # 查看全部状态(百分比 + mm + PID)
python3 water_control_cli.py set-mm 85       # 设定目标水位 85 mm
python3 water_control_cli.py read-mm         # 读取实测水位(mm)
python3 water_control_cli.py set-pid 3 0.5 0 # 在线写入 Kp/Ki/Kd(自动使能 bPidTune)
python3 water_control_cli.py auto|manual     # 自动/手动模式切换
python3 water_control_cli.py set-mm 60       # 设定 60mm (PLC 内部斜坡限速, 写完即返回)
```

常用命令一览(完整说明见 [SKILL.md](skill/water-level-control/SKILL.md)):

| 命令 | 功能 |
|---|---|
| `set-mm <mm>` / `raise <mm>` / `lower <mm>` | 设定/增量调节目标水位(mm) |
| `read` / `read-mm` / `write <pct>` | 读取目标(%)、实测(mm)、写目标(%) |
| `set-pid` / `get-pid` | PID 在线整定与读回 |
| `status` | 全状态监控 |
| `auto` / `manual` / `pump on\|off` / `valve <pct>` | 模式与手动操作 |
| `set-scale add/del/reset` | 多点分段线性标定(mm ↔ 原始值) |
| `set-ip <ip>` | 修改 PLC IP |

配置存于 `~/.water-control/config.json`(PLC IP、端口、slave ID、标定点表)。

## 运行范围与整定要点

- **传感器量程**:约 16.4–158 mm 对应 0–100%(4mA 点 ≈ 16.4mm,非标定错误)。
- **支持的控制目标窗口**:50–150 mm,区间外拒绝设定(PID 仅在此窗口现场整定)。
- **泵物理下限**:约 33 mm,低于此无法再抽降。
- **PID 增益(临时值,待实测定稿)**:当前统一 Kp=2.0/Ki=0.10/Kd=0(P 主导 + 小积分)。文档历史上两套"现场验证"增益(3.0/0.45 与 1.8/0.30 分段表)均已被现场证伪——2026-10-16 在 1.8/0.30 下观察到 ±10mm 积分振荡(升到 110 才开始抽、抽到 95 才停)。权威增益以 `step_test.py` 实测 K/τ → SIMC 推荐为准,见 [docs/ST侧改进说明.md](docs/ST侧改进说明.md)。
- **设定值斜坡**:目标水位以 ≤5mm/步 逐步逼近、超前量限幅 ±10mm,避免阶跃设定在纯滞后内造成积分饱和与超调;写入后监视至 ±3mm 稳定带内才报成功。
- **联锁**:HH=95% 全开抽水;LL=5% 停泵防干抽;传感器原始值 <3800 或 >20200 判断线故障。

调试与整定完整流程(标定 → 开环阶跃 → 闭环整定 → 联锁验证)见 [docs/控制方案.md](docs/控制方案.md) 第 5 节。

## 安全注意事项

- 阀为气开型(fail-closed):失气时阀关死、抽水停止,右桶靠自流上涨——应配独立高液位硬报警或溢流口作为最后防线。
- 修改 PID 前先确认 `bPidTune` 机制(使能位防止上电零增益),PLC 侧对增益有钳制(Kp≤20, Ki≤5, Kd≤10)。
- 现场首次投运务必按文档执行标定与联锁验证(拔传感器、强制 HH/LL)。
