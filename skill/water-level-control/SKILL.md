---
name: water-level-control
description: 通过 Modbus TCP 控制施耐德 PLC 上的水位时使用。提供 water_control_cli.py 脚本，用于设定目标水位（毫米或百分比）、读取实测水位、校准液位传感器、在线整定 PID 参数、以及监控 PLC 状态。
---

# 水位控制 CLI

## 概述

通过 Modbus TCP 控制施耐德 PLC 上的双桶水位装置。PLC 内部用百分比
（0-100% = 传感器 4-20mA），CLI 在其上换算出**毫米**，所有换算在客户端完成。

## 运行方式（免安装 / 无需 pip）

目标设备没有 pip，不存在已安装的命令，一律直接用 `python3`。
**先 `cd` 进 `scripts/` 一次：**

```bash
cd /root/.quantum-bot/workspace/skills/water-level-control/scripts
python3 water_control_cli.py status
```

本文所有示例都写成 `python3 water_control_cli.py <args>`，可直接复制粘贴，
前提是当前目录是上面的 `scripts/`。在其它目录下用绝对路径：

```bash
python3 /root/.quantum-bot/workspace/skills/water-level-control/scripts/water_control_cli.py <args>
```

## 前置检查：确认 PLC 在 Run 状态（静默执行）

执行任何命令之前，静默确认 PLC 正在运行。这是内部前置条件 —— **不要**向用户复述，
确认通过后直接继续。

```bash
/root/.quantum-bot/workspace/tools/plc_ctrl Run_State
```

看到 `result: 'success'` 就可以继续。不是 success 则启动并轮询：

```bash
/root/.quantum-bot/workspace/tools/plc_ctrl Run \
  && /root/.quantum-bot/workspace/tools/plc_ctrl Run_State   # 轮询到 success 为止
```

## 装置要点

理解这几条才能正确解读输出：

- **阀在抽水侧**：开度越大抽水越多、液位越低。所以"液位偏高 → 开阀"，
  PID 误差 = `PV − SP`（为正表示偏高）。
- **上升是被动的**：系统只能主动降低液位；上升靠左桶的重力自流。
  **上升比下降慢得多**（几十秒量级），不要过早判定失败。
- **闭式回路**：泵把右桶的水抽回左桶，总水量固定。液位越高左桶越空、自流越弱，
  所以"保持某个液位所需的阀开度"会随两桶水量分配缓慢变化。
  同一个目标在不同时刻可能需要不同开度，这是正常现象。
- **对象自衡**：阀固定时液位会停在一个平台，不会一直漂。

## 支持区间与拒绝规则

| 区间 | 物理液位 | 处理方式 |
|---|---|---|
| **支持设定目标** | **50 – 150 mm**（PLC 16.8 – 93.3%） | 正常执行 |
| 传感器可测 | 31.3 – 159.1 mm | 可读，但**不可作为目标** |
| < 50 mm | 泵物理下限约 45.3 mm 就在附近 | **拒绝**，说明支持区间是 50–150mm |
| > 150 mm | 高高联锁在 155.5 mm | **拒绝**，说明支持区间是 50–150mm |

用户要求 30mm、170mm 这类范围外目标时，**直接拒绝并说明区间，不要尝试执行命令**。

**稳定判据**：液位在目标 ±3 mm 内且不再漂移。注意 PLC 内部有 ±1.4 mm 的稳态死区，
所以 ±1.4 mm 以内的残差是正常的、不会被消除。

## 调整水位的标准流程（SOP）

**这是最常见的任务。严格三步，中间一步都不要发消息。**

### 第 1 步 —— 先回一句话，再执行命令

> 好的，马上为你把水位调整到 100mm。

**在发出命令之前就说这句**，不要等结果出来才开口。

### 第 2 步 —— 一条命令完成"设定 + 等待 + 判定"

```bash
python3 water_control_cli.py set-mm 100 --wait
```

`--wait` 在内部每 2 秒轮询一次，直到液位进入目标 ±3mm 并连续 3 次保持（退出码 0）、
或液位已停住但差得还多（提前退出，退出码 3）、或 100 秒超时（退出码 3）。

⚠️ **绝对不要自己写轮询循环。** 明确禁止：

- 反复调用 `read-mm` / `status` 去"再看看"
- 发出「正在等待液位稳定」「让我再多查几次」「还没到，继续等」这类中间消息
- 用 `trend` 代替 `--wait`（`trend` 是诊断工具，会刷出大量数据）
- 超时后自动重试

**每一次工具调用，用户都会看到你在自言自语一句**，而这一步用户不需要看到任何过程。
`--wait` 存在的唯一目的就是把等待收进**一次**调用里 —— 用它，SOP 才成立。

调用 Bash 工具时，`timeout` 参数必须 ≥ `--timeout + 20` 秒。默认 `--timeout 100`，
所以传 **120000 毫秒**。大行程（例如从 50mm 升到 150mm）用 `--timeout 170`
并把 Bash 的 timeout 设成 200000。

### 第 3 步 —— 按退出码回一句结论

| 退出码 | 命令输出 | 你该说什么 |
|---|---|---|
| **0** | `OK 已稳定: 100.2 mm (目标 100, 偏差 +0.2mm) 用时 34s 阀 26.0%` | 「水位已抵达目标 100mm（当前 100.2mm）。」 |
| **3** + `到不了位` | `WARN 到不了位: 当前 96.4 mm, 距目标 -3.6mm ... [液位已停住不再变化]` | 「水位停在 96.4mm，距目标 100mm 还差 3.6mm，已经不再变化，我记录下来需要人工确认。」**不要重试、不要继续轮询。** |
| **3** + `超时未稳定` | `WARN 超时未稳定: ... 漂移 +0.141 mm/s` | 「水位还在移动中（当前 xx mm），暂时没有稳定到目标。」可以再 `--wait` 一次；两次都超时就照实上报。 |
| **3** + `正在目标附近来回穿越` | 同上，但偏差已在容差内 | 「水位在目标附近 ±xmm 波动，没有完全稳住。」记下波动带宽上报。 |

结论里**只报液位**，不要提阀开度、积分、前馈、PID —— 那些是内部实现。

### 范围外的目标

用户要 30mm、170mm 这类值时**不要执行命令**，直接回：

> 本装置支持的目标水位是 50–150mm。30mm 超出了这个范围（泵的物理下限约 45.3mm），
> 没法设定。

### 一次完整对话应该长这样

```
用户：把水位调到 120mm
你  ：好的，马上为你把水位调整到 120mm。
      [调用一次 Bash: python3 water_control_cli.py set-mm 120 --wait  (timeout=120000)]
      [退出码 0]
你  ：水位已抵达目标 120mm（当前 120.3mm）。
```

**一共两次发言、一次工具调用。** 如果你发现自己说了第三句话，或者调了第二次工具，
就说明流程走错了。

## 停止水位控制的标准流程（SOP）

用于标定、尺子量测、探头检查、开环试验 —— 任何需要"水完全不动"的场合。
需要操作员配合关闭设备上方左右桶的连通开关，所以**顺序不能换**。

### 第 1 步 —— 先把液位调到 60mm

```bash
python3 water_control_cli.py set-mm 60 --wait
```

⚠️ **这一步必须在上方连通还开着的时候做。** 液位上升只能靠左桶的重力自流，
如果操作员提前关了上方开关，液位就到不了 60mm，`--wait` 会超时。
所以**不要在这一步之前提"关阀门"**。

60mm 是标准低点位：高于泵物理下限（约 45.3mm），也给后面人工加水到 ~145mm
留出了空间。如果任务需要别的高度，换成那个值即可，其余步骤不变。

### 第 2 步 —— 回一句话，告诉操作员 5 秒后停

液位到位后，**先发这句话，再执行命令**：

> 水位已到 60mm。我将在 5 秒后执行停止命令，关闭水泵和阀门。
> 请现在关闭设备上方的进水开关，把两个水桶断开。

### 第 3 步 —— 执行 stop（带倒计时）

```bash
python3 water_control_cli.py stop --delay 5
```

`--delay 5` 会先倒数 5 秒再动手，给操作员留出关阀时间；之后关泵、关阀、切手动，
再做漂移检查确认水真的不动了。

默认 `--settle 8 --recheck 2`：第一次漂移检查失败不会立刻判定异常，会再查两次
（操作员往往是在倒计时里才动手，关阀动作会落在第一个检查窗口内）。
Bash 工具的 `timeout` 给 **60000 毫秒**即可（最坏情况约 5 + 3×8 + 2×10 = 49 秒）。

### 第 4 步 —— 按退出码回一句结论

| 退出码 | 输出 | 你该说什么 |
|---|---|---|
| **0** | `✅ 系统已静止: 上下都封闭, 液位不动。可以校准/采样。` | 「已停止，水位 60.1mm 且不再变化，两个水桶已断开，可以开始标定。」 |
| **2** + 液位仍在移动 | `⚠️ 液位仍在移动 —— 泵和阀都已关闭, 说明上方左右桶连通没阻断(或漏水)` | 「水泵和阀门都已关闭，但液位还在变化，说明上方进水开关可能没关严。请确认后再继续。」**不要在这种情况下开始标定。** |
| **2** + 阀仍开/泵仍转 | `⚠️ PLC 安全联锁在覆盖手动指令` | 「液位高于高高联锁线，PLC 在强制抽水，需要先降液位。」 |

### 一次完整对话应该长这样

```
用户：把水位停住，我要做标定
你  ：好的，我先把水位调到 60mm。
      [Bash: set-mm 60 --wait  (timeout=120000)]   -> exit 0
你  ：水位已到 60mm。我将在 5 秒后执行停止命令，关闭水泵和阀门。
      请现在关闭设备上方的进水开关，把两个水桶断开。
      [Bash: stop --delay 5  (timeout=60000)]      -> exit 0
你  ：已停止，水位 60.1mm 且不再变化，两个水桶已断开，可以开始标定了。
```

**三句话、两次工具调用。** 中间不要插"正在等待""让我确认一下"这类消息。

### 做完之后

- 标定 → `calibrate --sample --label low` / `mid` / `high`，然后 `--three-point`
- 恢复运行 → **`resume`，不要用 `auto`**（`auto` 会让积分从零开始，几分钟里液位冲过目标）

## 命令

```bash
# 目标水位 —— 毫米（推荐）
python3 water_control_cli.py set-mm 85 --wait                 # 设定绝对目标 85 mm, 并阻塞到稳定(见 SOP)
python3 water_control_cli.py raise 20 --wait                  # 目标上调 20 mm(同样支持 --wait)
python3 water_control_cli.py lower 15                         # 目标下调 15 mm
python3 water_control_cli.py read-mm                          # 读实测液位（mm）
python3 water_control_cli.py read                             # 读目标液位（百分比）
python3 water_control_cli.py write 65.0                       # 设定目标液位 0-100%

# 状态与记录
python3 water_control_cli.py status                           # 全部状态（百分比 + mm + PID）
python3 water_control_cli.py trend --duration 600 --interval 2  # 记录液位/阀到 CSV，打印 min/max/带宽（Ctrl+C 提前结束仍会打印统计）
python3 water_control_cli.py read-level-raw                   # 读 iLevelRaw 原始计数 + 换算（校准用）

# 模式
python3 water_control_cli.py auto                             # 自动模式
python3 water_control_cli.py manual                           # 手动模式
python3 water_control_cli.py stop --delay 5                   # 静止化：倒数5s→泵OFF+阀0%+手动→验证液位真的不动（见 SOP）
python3 water_control_cli.py resume                           # 退出静止态：预置积分后切自动

# 手动执行机构（仅手动模式生效）
python3 water_control_cli.py pump on|off                      # 手动泵
python3 water_control_cli.py valve 30                         # 手动阀开度 —— 见下方安全须知

# PID（在线，经 Modbus）
python3 water_control_cli.py set-pid 2.0 0.26 0               # 写 Kp Ki Kd，自动使能并回读
python3 water_control_cli.py get-pid                          # 读 PLC 实际在用的增益
python3 water_control_cli.py get-learn                        # 读前馈开度 —— 见"输出解读"

# 标定
python3 water_control_cli.py calibrate --show                 # 显示当前液位模型与 raw↔mm 对照表
python3 water_control_cli.py calibrate --actual 104           # 用尺子实测值修正零点
python3 water_control_cli.py calibrate --actual 61.5 --check  # 只报误差，不写入
python3 water_control_cli.py calibrate --sample --label low   # 静水采样（泵 OFF + 阀 ≈0 才接受）
python3 water_control_cli.py calibrate --three-point R1 MM1 R2 MM2 R3 MM3   # 三点重拟合，打印残差

# 其它
python3 water_control_cli.py set-ip 192.168.0.122             # 设定 PLC IP
python3 water_control_cli.py read-raw 0                       # 转储原始寄存器（调试）
python3 step_test.py                                          # 阀阶跃试验（仅供参考，见"输出解读"）
```

`raise`/`lower` 是增量式的：读当前目标、加减增量、写回绝对值。

## 安全须知

**① `valve <开度>` 之前必须核对范围：本装置 50~150mm 对应的平衡开度只有 25.8 ~ 46.2%。**
给 60% 等于全速抽水，几十秒就能把桶抽到泵底以下（泵物理下限约 45.3mm）导致气蚀。
手动试验时开度只在当前平衡值附近 ±10pp 内动。

**② 手动模式会覆盖全部联锁**，包括高高（防溢）和低低（防干抽）。
`manual`、`stop`、`valve`、阶跃试验全程都在手动模式，**必须自己看住液位**：
液位接近 150mm 或接近 50mm 时立刻 `auto` 或 `stop`。

**③ 自动模式下阀被限制在 12~75%，泵常转不停。** 这是设计，不是故障。

## 静止状态（`stop` / `resume`）

标定、尺子量测、开环试验之前都要先静止。`stop` 关闭底部通路（泵 OFF + 阀 0% + 切手动），
上方左右桶的连通**由人工阻断**。

完整流程见上面的「停止水位控制的标准流程（SOP）」。参数：

```bash
python3 water_control_cli.py stop --delay 5     # 倒数 5 秒再动手(默认 5, 0=立即)
python3 water_control_cli.py stop --settle 8    # 每次漂移检查的时长(默认 8 秒)
python3 water_control_cli.py stop --recheck 2   # 漂移检查失败后再查几次(默认 2)
python3 water_control_cli.py stop --retry-wait 10   # 两次复查之间的等待(默认 10 秒)
python3 water_control_cli.py resume             # 预置积分后回到自动
python3 water_control_cli.py resume --valve 30  # 用指定开度预置（默认用 PLC 报的 u_eq）
```

`stop` 会回读阀/泵并采样液位，三种结果：

| 回读 | 含义 | 该怎么做 |
|---|---|---|
| 阀 ≈0、泵 OFF、漂移 <1 mm | ✅ 已静止 | 可以继续标定/量测 |
| 阀 >1% 或泵 ON | 液位仍在高高联锁以上 | 先把液位降下来 |
| 阀 ≈0、泵 OFF，但漂移 >1 mm | **上方连通没阻断**，或桶/管在漏 | 去检查上方管路，别急着标定 |

未验证静止时退出码为 2（`--force` 可绕过）。

**`resume` 不等于 `auto`。** 直接 `auto` 会让积分从零开始爬，几分钟里阀开度不对、
液位冲过目标。`resume` 会先把阀预置到平衡开度再切自动。从静止态恢复一律用 `resume`。

## 液位标定

模型是一条直线 `mm = K·(iLevelRaw − 4000) + b`。`K` 是装置固有斜率、不漂移；
探头打湿、温度、零点漂移只平移直线，所以**日常只改 `b`**：

```bash
python3 water_control_cli.py calibrate --actual 104.0         # 尺子实测 → 修正 b
python3 water_control_cli.py calibrate --actual 61.5 --check  # 只校验不写入
python3 water_control_cli.py calibrate --show                 # 看当前 K/b 与对照表
```

换探头、改管路、或出现"两端误差反号"时才需要重拟合 K，用**静水三点法**：

```bash
python3 water_control_cli.py set-mm 60 --wait                # 先调到 60mm（此时上方连通还开着）
# 按「停止水位控制的标准流程」第 2 步提示操作员，然后：
python3 water_control_cli.py stop --delay 5                  # 倒数 5s → 泵 OFF + 阀 0% + 验证静止
# 操作员已关闭上方 左→右 连通。给右桶加水到 ~55 mm，尺子量准：
python3 water_control_cli.py calibrate --sample --label low
# 加水到 ~100 mm，量准：
python3 water_control_cli.py calibrate --sample --label mid
# 加水到 ~145 mm，量准：
python3 water_control_cli.py calibrate --sample --label high
# 三点最小二乘拟合，打印逐点残差：
python3 water_control_cli.py calibrate --three-point 7542 55.3 12631 100.2 16830 145.1
python3 water_control_cli.py resume
```

**必须用三个点，不要用两个。** 两点解两未知数，残差恒为 0，拟合优度无从检验 ——
尺子读错、水面没静、某段确实非线性，全都会静默通过。三点留一个自由度，
`--three-point` 会逐点打印残差，**任一点超过 1.5 mm 就拒绝写入并点名**
（静水噪声底 0.4–0.5 mm）。`--force` 可绕过。

要点：外侧两点间距 ≥50 mm 并夹住 50–150 mm 区间，中间点大致居中；
两次标定的**测量基准必须一致**（桶底 / 探头下沿 / 溢流口下沿，记下来每次都用它）；
`--sample` 在泵转着或阀 >1% 时会拒绝执行。

其它守卫：斜率变化 >30% 需要 `--force`；|Δb| > 40 mm 直接拒绝（意味着探头是湿的、
基准变了、或量错了桶）；|Δb| > 25 mm 需要 `--force`；单点要求 3 次采样抖动 ≤3 mm
且显示液位在 50–150 mm 内。每次写入都记进 `level_cal_history`（最近 20 次），
`b` 不断游走说明探头在劣化。

重新标定 mm **不影响**控制律（那些量在百分比尺度上）。

## PID 在线整定

`set-pid` 一次写入三个增益并置位使能，然后回读 PLC 实际使用值确认生效：

```bash
python3 water_control_cli.py set-pid 2.0 0.26 0   # 写 + 使能 + 回读
python3 water_control_cli.py get-pid              # 确认实际增益与 tune: ON/OFF
```

- 增益使能位未置位时 PLC 用程序内置的默认值；`set-pid` 会自动置位，
  且**没有任何机制会自动清除**，一旦置位就保持（`get-pid` 里的 `tune: ON` 就是它）。
- PLC 会对传入值钳位：Kp ≤ 20、Ki ≤ 5、Kd ≤ 10。
- **Kd 无效** —— 控制律里没有微分项，写什么都没用。
- 实际生效的积分系数是 `Ki × 0.85`，比例系数是 `Kp × 1.15`（近目标）或 `Kp × 0.90`（远）。

## 输出解读

**`status`** —— 目标/实测液位（同时给 % 和 mm）、阀指令、自动/泵/故障/报警位、
PID 增益、`iLevelRaw`、以及当前生效的标定模型。

**`trend`** —— 结束时打印：液位 min/max/mean/波动带、拟合速率、阀指令 min/max/mean/摆幅。
稳态时**阀指令的 mean 就是当前液位对应的平衡开度**，波动带就是控制品质。
CSV 落在当前目录，文件名带时间戳。

**`get-learn`** —— ⚠️ 这个命令的输出标签**部分是错的**，只能取一半：

| 输出行 | 是否可用 |
|---|---|
| `u_eq = ...` | ✅ **可用** —— PLC 当拍实际使用的前馈开度（`resume` 就是用它预置的） |
| `C = ...` | ❌ **不可用** —— 读到的是一个控制律里没用到的保留寄存器，不是平衡线截距 |
| `一致性校验: u_eq + SP/2.13 = ...` | ❌ **不可用** —— 常数已过期，这行没有意义 |
| `阀 x% vs 平衡 y% → 液位将上升/下降/保持` | ✅ 可用，方向判断是对的 |

**不要把 `C = ...` 或那行一致性校验当作测量结果报告给用户。**

**`step_test.py`** —— 输出仅供参考。它假设对象可以用单一增益的一阶模型描述，
而本装置的平衡曲线是**分段**的（中段斜率是两端的 2.5 倍），所以它算出的 K 和
SIMC 增益不可直接采用。要改增益请用 `trend` 观察实际响应。

## 故障排查

| 症状 | 原因与处理 |
|---|---|
| `TypeError: unexpected keyword argument 'slave'` | 装的是 pymodbus 3.9+，本 CLI 按 3.8.6 写。降级到 3.8.6，或把代码里的 `slave=` 改成 `device_id=` |
| Modbus 异常码 2（非法地址） | PLC 侧寄存器数组尺寸不够或映射片段缺失，需要在 Machine Expert 里检查 |
| 回读的浮点像垃圾值（~1e-40 或 ~1e20） | 字节序被改成了高字在前。本 PLC 是**低字在前**（写 10.0 得到 `[0, 16672]`），把 `float_to_registers` 翻回 `[low, high]` |
| `iLevelRaw` 读回 0 | PLC 程序没有映射输入寄存器 9 |
| `calibrate --sample` 拒绝执行 | 泵在转或阀 >1%，先跑 `stop` |
| 设定目标后液位长时间不动 | 上升靠重力自流，本来就慢；用 `trend` 看阀指令是否在 12% 附近（说明已在全力少抽） |

配置在 `~/.water-control/config.json`（`plc_ip` / `plc_port` / `slave_id` /
`level_model` / `level_cal_history`）。缺失的键回退到默认值，旧配置文件仍可用。
**当前生效的标定值用 `calibrate --show` 查**，不要引用仓库里的出厂默认值。

## Agent 交互规则

- **演示过程中绝不向最终用户提及 PID 参数调整**，只报告最终的水位设定结果。
- 前置检查（`plc_ctrl Run_State`）静默执行，不复述。
- 范围外的目标（<50mm 或 >150mm）直接拒绝并说明支持区间，不要尝试执行。
- 报告液位时优先用 **mm**，必要时附百分比。
- **调水位一律走「调整水位的标准流程（SOP）」**：两次发言、一次工具调用。
  出现「让我再查一次」「还在等待」这类中间消息就说明流程走错了。
- **停水一律走「停止水位控制的标准流程（SOP）」**：三句话、两次工具调用，
  且必须先调到 60mm 再提示操作员关上方开关（顺序反了液位就到不了位）。

#### 为演示设定一个液位
按上面的「调整水位的标准流程（SOP）」走：**先回一句、再 `set-mm <值> --wait` 一次、
最后回一句结论**。不要轮询、不要发中间消息、不要解释任何控制细节。
