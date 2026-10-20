(* ============================================================================
   PLC_PRG — 水位控制改进版 (optimize/level-control 分支)
   适用: TM310MM35ECT16U + Machine Expert, 任务周期 T#100ms
   用法: 整段替换现有 PLC_PRG 的声明区与实现区 (GVL 不变)

   相对现行版本的改进 (针对控制超调 / 液位不稳):
   [G1] 默认增益改为保守值 Kp=2.0 Ki=0.10 (临时值, 待 step_test.py 实测定稿)
   [G2] SP 内部斜坡: 上位机阶跃写入的目标在 PLC 内限速逼近 (2%/s),
        纯滞后 τ 内误差不再持续拉大 -> 消除积分饱和型超调;
        对 set-mm/write 直写路径同样生效 (CLI 斜坡之外的第二道防线)
   [G3] PV 一阶低通滤波 (Tc=1s): 抑制 4-20mA 采样噪声进 PID/微分
   [G4] 微分改为对测量微分 (derivative-on-measurement): SP 阶跃无微分踢
   [G5] 抗饱和一致性修复: 饱和判别与最终输出使用同一个更新后的积分
   [G9] 积分状态改存"I 项输出贡献量(%阀开度)", 限幅与 Ki 解耦:
        旧 rInt 钳位 ±100 → I 项范围 = Ki×100, 改 Ki 即改限幅, 整定不安全
        (2026-10-16 现场: Ki=0.30 时 ±10mm 积分振荡; Ki 调小又怕顶不住入流)
        (原代码 rOut 用旧积分判饱和, 输出用新积分, 二者不一致)
   [G6] 传感器故障去抖: 连续 1s 超限才判故障, 单次干扰脉冲不再误停泵
   [G7] 阀输出速率限制 (10%/s, 仅 PID 路径): 防液压冲击/阀频繁大幅动作;
        联锁与手动路径不限速 (安全动作必须立即执行)
   [G8] 目标水位钳位 0..100%, 防 Modbus 写入越界值
   [G10] 微分符号修复: 本系统阀在抽水侧 = 反作用对象, 对测量微分应取
        正号 +(dPV/dt)。原负号是正作用对象(加热/注水)的写法, 启用 Kd 后
        会加剧而非抑制超调(此前 Kd=0 未暴露)。同时把微分/速率估计移到
        公共段, 联锁/手动路径也能得到当拍新鲜值。
   [G11] 上升预开阀 (predictive pre-open): 上升方向无刹车(阀已全关, 只能
        等自流), 唯一手段是提前抽水。用实测上涨速率算出到达目标剩余时间,
        当剩余时间 < T_lead 时按比例把阀预开到平衡开度 u_eq, 交接给 PID 时
        积分跟踪预开值保证输出连续; 抽水一旦超过入流(水位转降)立即退出。
        这是上升方向防超调的主手段, 对 set-mm/HMI 直写路径同样生效。
        需标定 3 个参数: rInflowRate(上涨速率) / rUeqEst(平衡开度) /
        rPreOpenLead(提前量), 标定方法见 docs/ST侧改进说明.md。
   [G12] 在线自学习 d0 / u_eq: 两个量都是直接可观测量(阀全关时上涨速率=d0;
        稳态时阀开度=u_eq), 用一阶低通(指数滑动平均)缓慢逼近, 永不跳变。
        学到的值实时映射到输入寄存器空闲槽(Device_0/1=d0, Device_6/7=u_eq)
        供上位机 get-learn 读取; 满意后可把 bAutoLearn 置 FALSE 冻结, 或把
        读到的值写成 rInflowRate/rUeqEst 的默认值(掉电不丢)。
        水量变化(左桶加水)后自动跟随, 不必重测 —— 这是 rInflowRate/rUeqEst
        可以手测写死之外的另一种用法, 二者共用同一组变量。
   联锁逻辑 (HH/LL/传感器故障) 与 Modbus 寄存器映射保持不变,
   上位机 water_control_cli.py 无需任何修改即可兼容。
   ============================================================================ *)

PROGRAM PLC_PRG
VAR
    (* ===== 周期 ===== *)
    tCycle : TIME := T#100MS;
    rDt    : REAL;                       (* 采样周期, 秒 *)

    (* ===== 物理IO (地址须与 Device IO Mapping 核对) ===== *)
    iLevelRaw AT %IW3    : INT;          (* 水位 4-20mA, 4000..20000 = 0..100% *)
    iValveOut  AT %QW3   : INT;          (* 抽水阀开度 4-20mA, 4000..20000 = 0..100% *)
    bPump      AT %QX4.0 : BOOL;         (* 抽水泵继电器, ON=抽水 *)

    (* ===== HMI / Modbus 设定 ===== *)
    bAuto        AT %MX102.0 : BOOL := TRUE;    (* 1=自动 0=手动 *)
    bManualPump  AT %MX106.0 : BOOL := FALSE;   (* 手动泵 ON *)

    (* ===== 状态/报警 (只读供 HMI) ===== *)
    bPumpCmd     AT %MX114.0 : BOOL;     (* 泵指令 *)
    bSensorFault AT %MX114.1 : BOOL;     (* 传感器故障 *)
    bAlarm       AT %MX114.2 : BOOL;     (* 总报警 *)

    (* ===== 量程 (INT) ===== *)
    iRawMin : INT := 4000;
    iRawMax : INT := 20000;

    (* ===== PID 内部 ===== *)
    (* [G9] 积分状态改存"I 项输出贡献量(%阀开度)", 不再是原始 ∫e·dt:
       旧写法 rInt 钳位 ±100, I 项范围 = Ki×100, 与 Ki 耦合 ——
       Ki=0.3 时 I 项可摆 ±30% (过大 → 积分振荡, 2026-10-16 现场 ±10mm 循环实锤);
       Ki=0.05 时 I 项最大仅 5% (过小 → 顶不住入流平衡点, 静差消不掉)。
       改存贡献量后限幅单位就是 %阀开度, 与 Ki 无关, 在线改 Ki 安全。 *)
    rError, rITerm, rDeriv, rPD, rOut : REAL;

    (* ===== PID 参数 ===== *)
    (* [G1] 临时保守值, 待 step_test.py 实测 K/τ 后回填:
       2026-10-16 现场观察 Ki=0.30 仍 ±10mm 积分振荡, 故 Ki 降至 0.10;
       bPidTune=1 时由 Modbus 写入值覆盖 *)
    rKp AT %MW20 : REAL := 2.0;      (* 占 MW20-21, 默认值(上电/未使能时用) *)
    rKi AT %MW22 : REAL := 0.10;     (* 占 MW22-23, 1/s, 抵消自然入流静差 *)
    rKd AT %MW24 : REAL := 0.0;      (* 占 MW24-25 *)
    bPidTune : BOOL := FALSE;        (* 1=采用 Modbus 传入的 Kp/Ki/Kd, 0=用上面默认 *)

    (* [G9] I 项输出贡献限幅 (%阀开度, 与 Ki 无关):
       上限须 ≥ 平衡入流所需阀开度 (预估 20-40%, 留裕量到 60);
       下限允许小负值, 用于水位偏低时快速关阀 *)
    rILimHi : REAL := 60.0;
    rILimLo : REAL := -20.0;

    (* ===== 联锁阈值 (%, 工程量) ===== *)
    rLvlHH      : REAL := 95.0;   (* 高高: 最大抽水降压 *)
    rLvlLL      : REAL := 5.0;    (* 低低: 停泵防干抽, 靠自流回涨 *)
    rPumpOnThr  : REAL := 3.0;    (* 阀开度>3% 开泵 *)
    rPumpOffThr : REAL := 1.0;    (* 阀开度<1% 关泵(防憋压) *)

    (* ===== [G2][G3][G6][G7] 新增: 斜坡 / 滤波 / 去抖 / 限速 ===== *)
    rSpRamp      : REAL;               (* [G2] 内部斜坡后的设定值 %, PID 实际使用 *)
    rSpRate      : REAL := 2.0;        (* [G2] SP 斜坡速率 %/s (100%行程≥50s, 可按现场加快) *)
    rPvFilt      : REAL;               (* [G3] 滤波后的水位 %, 仅 PID 使用 *)
    rPvTc        : REAL := 1.0;        (* [G3] PV 低通滤波时间常数 s (联锁仍用未滤波值, 不影响安全响应) *)
    rPrevLevel   : REAL;               (* [G4] 上周期滤波 PV, 对测量微分用 *)
    iFaultCnt    : INT;                (* [G6] 传感器超限连续周期计数 *)
    iFaultDebounce : INT := 10;        (* [G6] 去抖阈值: 10 周期 = 1s 连续超限才判故障 *)
    rValveRate   : REAL := 10.0;       (* [G7] 阀输出速率限制 %/s (仅 PID 路径) *)
    rPrevValve   : REAL;               (* [G7] 上周期阀指令, 限速基准 *)
    bFirstScan   : BOOL := TRUE;       (* 首周期初始化标志 *)

    (* ===== [G10][G11] 微分/速率估计 + 上升预开阀 ===== *)
    (* 微分: 公共段计算, 联锁/手动路径也拿到当拍新鲜值 *)
    rRateFilt    : REAL;               (* [G10] 上涨速率低通估计 %/s (由 dPV/dt 滤波而来) *)
    rRateTc      : REAL := 10.0;       (* [G10] 速率估计时间常数 s (兼作水位转降判据的平滑) *)
    rRateAlpha   : REAL;               (* [G10] 速率滤波系数 *)

    (* [G11] 上升预开阀: 标定这 3 个参数后把 bPreOpenEn 置 TRUE *)
    bPreOpenEn   : BOOL := FALSE;      (* 使能开关: 三参数未标定前保持 FALSE *)
    (* [G12] 占用 %MW26-29 供 Modbus 回读学习值(当前映射未用这些地址;
       若与其它 POU 的 %MW 冲突, 改到空闲地址并同步改下方写出段) *)
    rInflowRate  AT %MW26 : REAL := 0.30;  (* 自然上涨速率 d0, %/s  —— 手测填入或自学习 *)
    rUeqEst      AT %MW28 : REAL := 20.0;  (* 平衡阀开度 u_eq, % —— 手测填入或自学习 *)
    rPreOpenLead : REAL := 20.0;       (* 提前量 T_lead, s —— 取 2τ, τ 由 step_test 实测 *)
    rTArrive     : REAL;               (* 预计到达目标剩余秒数 *)
    rPreOpen     : REAL;               (* 预开阀开度 %, 0=不预开 *)

    (* ===== [G12] 在线自学习参数 ===== *)
    bAutoLearn     : BOOL := TRUE;     (* 自学习总开关: 学到的值满意后可置 FALSE 冻结 *)
    rLearnTcD0     : REAL := 30.0;     (* d0 学习时间常数 s (越小跟得越快, 噪声越大) *)
    rLearnTcUeq    : REAL := 60.0;     (* u_eq 学习时间常数 s (稳态学习, 取更慢更稳) *)
    rLearnAlphaD0  : REAL;             (* 本拍 d0 学习系数 *)
    rLearnAlphaUeq : REAL;             (* 本拍 u_eq 学习系数 *)
    rD0LearnMin    : REAL := 0.02;     (* d0 学习门槛: 观测速率须 > 此值 %/s (避开噪声与近平衡点) *)
    rUeqSettleE    : REAL := 1.0;      (* u_eq 学习门槛: |误差| < 此值 % (约 1.3mm) 才算稳态 *)
    rUeqSettleRate : REAL := 0.05;     (* u_eq 学习门槛: |速率| < 此值 %/s 才算稳态 *)

    (* ===== 临时量 ===== *)
    rSpRaw    : REAL;                  (* [G8] 钳位后的原始目标 *)
    rAlpha    : REAL;                  (* [G3] 滤波系数 *)
    rValveDelta : REAL;                (* [G7] 每周期最大阀变化量 *)
END_VAR

rDt := TIME_TO_REAL(tCycle) / 1000;   (* 0.1 s *)

(* ===== Modbus 读入: 上位机 -> PLC 设定值 ===== *)
(* REAL 占 2 字, 低字在前(Device_0=低字, 与现有 rLevel 映射顺序一致) *)
%MW0 := iwModbusTCP_Server_Device_0;   (* rTargetLevel 低字 *)
%MW1 := iwModbusTCP_Server_Device_1;   (* rTargetLevel 高字 *)
%MW2 := iwModbusTCP_Server_Device_6;   (* rManualValve 低字 (可选) *)
%MW3 := iwModbusTCP_Server_Device_7;   (* rManualValve 高字 (可选) *)
bAuto       := iwModbusTCP_Server_Device_10.0;   (* 提取 bit0 *)
bManualPump := iwModbusTCP_Server_Device_10.1;   (* 提取 bit1 *)
bPidTune    := iwModbusTCP_Server_Device_10.2;   (* 提取 bit2 *)

(* [G8] 目标钳位: 防上位机写入越界/异常值 *)
GVL.rTargetLevel := LIMIT(0.0, GVL.rTargetLevel, 100.0);

(* 仅当使能位置 1 时, 才用 Modbus 传入的 PID 参数覆盖默认值 *)
IF bPidTune THEN
    %MW20 := iwModbusTCP_Server_Device_2;    (* rKp 低字 *)
    %MW21 := iwModbusTCP_Server_Device_3;    (* rKp 高字 *)
    %MW22 := iwModbusTCP_Server_Device_4;    (* rKi 低字 *)
    %MW23 := iwModbusTCP_Server_Device_5;    (* rKi 高字 *)
    %MW24 := iwModbusTCP_Server_Device_12;   (* rKd 低字 *)
    %MW25 := iwModbusTCP_Server_Device_13;   (* rKd 高字 *)

    (* 安全钳制: 防止写入过大值导致剧烈振荡 *)
    rKp := LIMIT(0.0, rKp, 20.0);
    rKi := LIMIT(0.0, rKi, 5.0);
    rKd := LIMIT(0.0, rKd, 10.0);
END_IF
(* bPidTune=0 时不覆盖, rKp/rKi/rKd 保持声明里的默认(或上一次使能写入的值) *)

(* ---------- 1. 传感器自检 + 水位标定 ---------- *)
(* [G6] 去抖: 连续 iFaultDebounce 个周期超限才置故障, 单次干扰不误停泵 *)
IF (iLevelRaw < iRawMin - 200) OR (iLevelRaw > iRawMax + 200) THEN
    IF iFaultCnt < 32000 THEN iFaultCnt := iFaultCnt + 1; END_IF
ELSE
    iFaultCnt := 0;
END_IF
bSensorFault := (iFaultCnt >= iFaultDebounce);

GVL.rLevel := LIMIT(0.0,
    (INT_TO_REAL(iLevelRaw - iRawMin) / 16000.0) * 100.0, 100.0);

(* ---------- 1a. [G3] PV 一阶低通滤波 (仅 PID 使用; 联锁/HMI 仍用 rLevel) ---------- *)
IF bFirstScan THEN
    (* 首周期用实测值初始化, 避免从 0 拉起造成假误差 *)
    rPvFilt    := GVL.rLevel;
    rPrevLevel := GVL.rLevel;
    rSpRamp    := GVL.rTargetLevel;
    rPrevValve := GVL.rValveCmd;
    bFirstScan := FALSE;
END_IF
rAlpha  := rDt / (rPvTc + rDt);
rPvFilt := rPvFilt + rAlpha * (GVL.rLevel - rPvFilt);

(* ---------- 1a2. [G10] 微分与上涨速率估计 (公共段, 各路径共用当拍新鲜值) ----------
   [G10] 微分符号: 本系统阀在抽水侧 = 反作用对象, 水位在涨时应开大阀刹车,
   故对测量微分取正号 +dPV/dt。教科书里的 -Kd·dPV/dt 是正作用对象(加热/注水)
   的写法, 用在这里符号相反, 启用 Kd 会加剧超调。 *)
rDeriv := (rPvFilt - rPrevLevel) / rDt;
rPrevLevel := rPvFilt;

(* [G11] 上涨速率低通估计 (供预开阀判断"水位是否仍在涨") *)
rRateAlpha := rDt / (rRateTc + rDt);
rRateFilt  := rRateFilt + rRateAlpha * (rDeriv - rRateFilt);

(* ---------- 1b. [G2] SP 内部斜坡: 阶跃目标限速逼近 ---------- *)
rSpRaw := GVL.rTargetLevel;   (* 已钳位 *)
IF rSpRamp < rSpRaw THEN
    rSpRamp := MIN(rSpRamp + rSpRate * rDt, rSpRaw);
ELSIF rSpRamp > rSpRaw THEN
    rSpRamp := MAX(rSpRamp - rSpRate * rDt, rSpRaw);
END_IF

(* ---------- 2. 联锁 (最高优先级; 用未滤波 rLevel, 保证安全响应速度) ---------- *)
IF bSensorFault THEN
    (* 传感器故障: 停泵关阀+报警 (自然入流会缓慢上升, 需人工干预) *)
    bPumpCmd  := FALSE;
    GVL.rValveCmd := 0.0;
    rITerm    := 0.0;
ELSIF (GVL.rLevel >= rLvlHH) THEN
    (* 高高液位: 最大抽水快速降压 *)
    bPumpCmd  := TRUE;
    GVL.rValveCmd := 100.0;
    rITerm    := 0.0;
ELSIF (GVL.rLevel <= rLvlLL) THEN
    (* 低低液位: 停抽防干抽, 靠自然入流回涨 *)
    bPumpCmd  := FALSE;
    GVL.rValveCmd := 0.0;
    rITerm    := 0.0;
ELSE
    (* ---------- 3. PID (阀在抽水侧: 偏高->开阀抽水) ---------- *)
    (* [G2] 误差用斜坡后的 SP; [G3] PV 用滤波值; [G10] rDeriv 已在公共段算好 *)
    rError := rPvFilt - rSpRamp;                   (* PV - SP, 正=偏高需抽水 *)
    rPD    := rKp * rError + rKd * rDeriv;

    (* [G5][G9] 抗积分饱和: 先算候选输出判饱和, 未饱和才累积积分贡献量,
       然后用同一个更新后的 rITerm 重算输出 —— 判别与输出严格一致。
       rITerm 单位为 %阀开度, 限幅 [rILimLo, rILimHi] 与 Ki 无关 *)
    rOut := rPD + rITerm;
    IF NOT ((rOut >= 100.0 AND rError > 0.0) OR (rOut <= 0.0 AND rError < 0.0)) THEN
        rITerm := rITerm + rKi * rError * rDt;
        rITerm := LIMIT(rILimLo, rITerm, rILimHi);
    END_IF
    rOut := rPD + rITerm;                          (* 用更新后的 rITerm *)
    GVL.rValveCmd := LIMIT(0.0, rOut, 100.0);

    (* ---------- 3a. [G11] 上升预开阀: 提前抽水, 消除上升方向超调 ----------
       上升时阀已全关(误差为负 → 输出饱和在 0), 系统只能靠自流被动上涨,
       没有减速手段; 唯一办法是在到达目标之前就开阀抽水。
       剩余时间 = (SP − PV) / d0 (d0 为实测上涨速率);
       剩余时间 < T_lead 时按比例把阀预开到平衡开度 u_eq。
       退出条件: 水位转降(rRateFilt ≤ 0, 说明抽水已超过入流) 或 已到/超过目标。
       预开生效时把积分跟踪到预开值, 保证交接给 PID 时输出连续不跳变。 *)
    rPreOpen := 0.0;
    IF bPreOpenEn AND (rInflowRate > 0.01)
       AND (rError < 0.0) AND (rRateFilt > 0.0) THEN
        rTArrive := (rSpRamp - rPvFilt) / rInflowRate;
        IF rTArrive < rPreOpenLead THEN
            rPreOpen := rUeqEst * (1.0 - MAX(0.0, rTArrive) / rPreOpenLead);
            IF rPreOpen > GVL.rValveCmd THEN
                GVL.rValveCmd := rPreOpen;
                rITerm := LIMIT(rILimLo, rPreOpen - rPD, rILimHi);
            END_IF
        END_IF
    END_IF

    (* [G7] 阀输出速率限制 (仅 PID 路径, 联锁/手动不限速):
       每周期最多变化 rValveRate*rDt (默认 10%/s = 1%/周期) *)
    rValveDelta := rValveRate * rDt;
    IF GVL.rValveCmd > rPrevValve + rValveDelta THEN
        GVL.rValveCmd := rPrevValve + rValveDelta;
    ELSIF GVL.rValveCmd < rPrevValve - rValveDelta THEN
        GVL.rValveCmd := rPrevValve - rValveDelta;
    END_IF
    rPrevValve   := GVL.rValveCmd;

    (* ---------- 4. 泵启停迟滞 (保护继电器+防憋压) ---------- *)
    IF bPumpCmd THEN
        IF GVL.rValveCmd < rPumpOffThr THEN bPumpCmd := FALSE; END_IF
    ELSE
        IF GVL.rValveCmd > rPumpOnThr  THEN bPumpCmd := TRUE;  END_IF
    END_IF
END_IF

(* ---------- 5. 手动模式 (含无扰切换: 积分贡献量直接跟踪手动阀开度) ---------- *)
IF NOT bAuto THEN
    GVL.rValveCmd := LIMIT(0.0, GVL.rManualValve, 100.0);
    bPumpCmd  := bManualPump;
    (* [G9] rITerm 即输出贡献量, 无扰切换 = 手动阀开度 − 当前 P(D) 项,
       不再除以 Ki (旧写法在 Ki=0 时失效, 且量纲随 Ki 漂移) *)
    rITerm := GVL.rValveCmd - (rKp * (rPvFilt - rSpRamp) + rKd * rDeriv);
    rITerm := LIMIT(rILimLo, rITerm, rILimHi);
    (* 手动期间同步限速基准, 切回自动无扰 (微分基准 rPrevLevel 已在公共段更新) *)
    rPrevValve := GVL.rValveCmd;
END_IF

(* ---------- 5a. [G12] 在线自学习: d0 与 u_eq ----------
   原理: 两个量都是直接可观测量 ——
     d0  : 阀全关(抽水=0)时, 水位上涨速率就是 d0;
     u_eq: 稳态(|误差|小 且 不再漂移)时, 阀开度就是平衡开度 u_eq。
   学习方法 = 一阶低通(指数滑动平均): 每拍把估计值往观测量挪 α 比例,
   α = 周期/(时间常数+周期) ≈ 0.003 —— 永不跳变, 单个噪声样本只影响 α 比例;
   指数收敛(30s 时间常数: 60s 后误差 ~14%, 90s 后 ~5%); 工况变化自动跟随。
   门控: 只在"抽水=0"时学 d0, 只在"真稳"时学 u_eq, 避免用被自己扰动过的数据学习。
   注意: 预开阀一动作阀开度就 >1%, d0 学习自动停止 —— 自洽。 *)
IF bAutoLearn AND bAuto AND NOT bSensorFault
   AND (GVL.rLevel < rLvlHH) AND (GVL.rLevel > rLvlLL) THEN

    (* d0: 阀基本全关 且 水位确实在涨 → 此刻净速率 = d0 *)
    rLearnAlphaD0 := rDt / (rLearnTcD0 + rDt);
    IF (GVL.rValveCmd < 1.0) AND (rRateFilt > rD0LearnMin) THEN
        rInflowRate := LIMIT(0.01,
            rInflowRate + rLearnAlphaD0 * (rRateFilt - rInflowRate), 5.0);
    END_IF

    (* u_eq: 已到目标附近 且 已稳 → 此刻阀开度 = 平衡开度 *)
    rLearnAlphaUeq := rDt / (rLearnTcUeq + rDt);
    IF (ABS(rError) < rUeqSettleE) AND (ABS(rRateFilt) < rUeqSettleRate) THEN
        rUeqEst := LIMIT(0.0,
            rUeqEst + rLearnAlphaUeq * (GVL.rValveCmd - rUeqEst), rILimHi);
    END_IF
END_IF

(* ---------- 6. 输出标定与写入 ---------- *)
iValveOut := LIMIT(iRawMin,
    iRawMin + REAL_TO_INT(GVL.rValveCmd / 100.0 * 16000.0), iRawMax);
bPump := bPumpCmd AND NOT bSensorFault;

(* ---------- 7. 报警汇总 ---------- *)
bAlarm := bSensorFault OR (GVL.rLevel >= rLvlHH) OR (GVL.rLevel <= rLvlLL);

(* ===== Modbus 写出: PLC -> 上位机 只读状态 ===== *)
qwModbusTCP_Server_Device_2 := %MW4;   (* rLevel 低字 *)
qwModbusTCP_Server_Device_3 := %MW5;   (* rLevel 高字 *)
qwModbusTCP_Server_Device_4 := %MW6;   (* rValveCmd 低字 *)
qwModbusTCP_Server_Device_5 := %MW7;   (* rValveCmd 高字 *)
(* bit0=bAuto, bit1=bPumpCmd, bit2=bSensorFault, bit3=bAlarm, bit4=bPump *)
(* 直接给 WORD 的指定位赋值，0 警告，执行效率极高 *)
qwModbusTCP_Server_Device_8.0 := bAuto;         (* 第 0 位赋值 *)
qwModbusTCP_Server_Device_8.1 := bPumpCmd;      (* 第 1 位赋值 *)
qwModbusTCP_Server_Device_8.2 := bSensorFault;  (* 第 2 位赋值 *)
qwModbusTCP_Server_Device_8.3 := bAlarm;        (* 第 3 位赋值 *)
qwModbusTCP_Server_Device_8.4 := bPump;         (* 第 4 位赋值 *)
qwModbusTCP_Server_Device_10 := %MW20;   (* rKp 低字 *)
qwModbusTCP_Server_Device_11 := %MW21;   (* rKp 高字 *)
qwModbusTCP_Server_Device_12 := %MW22;   (* rKi 低字 *)
qwModbusTCP_Server_Device_13 := %MW23;   (* rKi 高字 *)
qwModbusTCP_Server_Device_14 := %MW24;   (* rKd 低字 *)
qwModbusTCP_Server_Device_15 := %MW25;   (* rKd 高字 *)
(* [G12] 自学习值回读 (占用输入寄存器空闲槽 0/1 与 6/7, 无需改数组大小) *)
qwModbusTCP_Server_Device_0 := %MW26;    (* 学习到的 d0 低字 *)
qwModbusTCP_Server_Device_1 := %MW27;    (* 学习到的 d0 高字 *)
qwModbusTCP_Server_Device_6 := %MW28;    (* 学习到的 u_eq 低字 *)
qwModbusTCP_Server_Device_7 := %MW29;    (* 学习到的 u_eq 高字 *)
