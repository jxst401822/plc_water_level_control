PROGRAM PLC_PRG
VAR
    (* ===== 1. 周期 (严格匹配 CODESYS MAST 20ms) ===== *)
    tCycle : TIME := T#20MS;
    rDt    : REAL;                       (* 采样周期, 秒 (0.02s) *)

    (* ===== 2. 物理IO (地址须与 Device IO Mapping 核对) ===== *)
    iLevelRaw AT %IW3    : INT;          (* 水位 4-20mA, 4000..20000 = 0..100% *)
    iValveOut  AT %QW3   : INT;          (* 抽水阀开度 4-20mA, 4000..20000 = 0..100% *)
    bPump      AT %QX4.0 : BOOL;         (* 抽水泵继电器, ON=抽水 *)

    (* ===== 3. HMI / Modbus 设定 ===== *)
    bAuto        AT %MX102.0 : BOOL := TRUE;    (* 1=自动 0=手动 *)
    bManualPump  AT %MX106.0 : BOOL := FALSE;   (* 手动泵 ON *)

    (* ===== 4. 状态/报警 (只读供 HMI) ===== *)
    bPumpCmd     AT %MX114.0 : BOOL;     (* 泵指令 *)
    bSensorFault AT %MX114.1 : BOOL;     (* 传感器故障 *)
    bAlarm       AT %MX114.2 : BOOL;     (* 总报警 *)

    (* ===== 5. 量程 (INT) ===== *)
    iRawMin : INT := 4000;
    iRawMax : INT := 20000;

    (* ===== 6. PID 与 20ms 状态变量 ===== *)
    rError, rPrevError, rInt, rPD, rOut : REAL;
    bFilterInit  : BOOL := FALSE;        (* 上电首次采样无扰初始化标志 *)
    rLevelFilt   : REAL := 0.0;          (* 快速滤波液位 (τ = 0.16s) *)
    rLevelSlow   : REAL := 0.0;          (* 慢速跟踪液位 (τ = 0.65s) *)
    rLevelRate   : REAL := 0.0;          (* 零毛刺真实水位速度 %/s *)
    rPrevTarget  : REAL := -999.0;       (* 用于检测目标设定变更 *)
    bRisingTask  : BOOL := FALSE;        (* 方向记忆：TRUE=上升任务，FALSE=下降任务 *)

    (* ===== 7. PID 参数 (对齐上位机 2.0 / 0.26 / 0.0) ===== *)
    rKp AT %MW20 : REAL := 2.0;          
    rKi AT %MW22 : REAL := 0.26;         
    rKd AT %MW24 : REAL := 0.0;          
    bPidTune : BOOL := FALSE;        

    (* ===== 8. 联锁阈值 (%, 工程量) ===== *)
    rLvlHH      : REAL := 98.5;          (* 155.5mm 高高报，保障 150mm 正常调节 *)
    rLvlLL      : REAL := 5.0;    
    rPumpOnThr  : REAL := 3.0;    
    rPumpOffThr : REAL := 1.0;    

    (* ===== 9. 动态平衡与阻尼参数 (保留 %MW26/%MW28 供 CLI 读取) ===== *)
    rRiseSpeedOffset AT %MW26 : REAL := 8.0;     (* 保留寄存器映射供 CLI 监控 *)
    rUeqDyn          AT %MW28 : REAL;            (* 实时计算的精准目标平衡开度 % *)
    rUeqBase         : REAL;                     (* 基础平衡开度 % *)
    rVelDampGain     : REAL := 3.6;              (* 速度阻尼增益 *)
    rEffKi           : REAL;                     (* 实际生效的积分系数 *)
    rTargetMM        : REAL;                     (* 目标水位 mm *)
    rUnsaturatedOut  : REAL;                    
    rMinOut, rMaxOut : REAL;                     (* 动态上下限 *)
END_VAR

(* 精确计算采样周期：20ms = 0.02s *)
rDt := TIME_TO_REAL(tCycle) / 1000.0;

(* ===== Modbus 读入: 上位机 -> PLC 设定值 ===== *)
%MW0 := iwModbusTCP_Server_Device_0;   
%MW1 := iwModbusTCP_Server_Device_1;   
%MW2 := iwModbusTCP_Server_Device_6;   
%MW3 := iwModbusTCP_Server_Device_7;   
bAuto       := iwModbusTCP_Server_Device_10.0;   
bManualPump := iwModbusTCP_Server_Device_10.1;   
bPidTune    := iwModbusTCP_Server_Device_10.2;   

IF bPidTune THEN
    %MW20 := iwModbusTCP_Server_Device_2;    
    %MW21 := iwModbusTCP_Server_Device_3;    
    %MW22 := iwModbusTCP_Server_Device_4;    
    %MW23 := iwModbusTCP_Server_Device_5;    
    %MW24 := iwModbusTCP_Server_Device_12;   
    %MW25 := iwModbusTCP_Server_Device_13;   

    rKp := LIMIT(0.0, rKp, 20.0);
    rKi := LIMIT(0.0, rKi, 5.0);
    rKd := LIMIT(0.0, rKd, 10.0);
END_IF

(* ---------- 1. 传感器自检 + 20ms 快慢双滤波器提取零噪声速度 ---------- *)
IF (iLevelRaw < iRawMin - 200) OR (iLevelRaw > iRawMax + 200) THEN
    bSensorFault := TRUE;
ELSE
    bSensorFault := FALSE;
END_IF
GVL.rLevel := LIMIT(0.0,
    (INT_TO_REAL(iLevelRaw - iRawMin) / 16000.0) * 100.0, 100.0);

IF NOT bFilterInit THEN
    rLevelFilt  := GVL.rLevel;
    rLevelSlow  := GVL.rLevel;
    bFilterInit := TRUE;
ELSE
    rLevelFilt  := 0.89 * rLevelFilt + 0.11 * GVL.rLevel;     (* 快滤波 τ = 0.16s *)
    rLevelSlow  := 0.97 * rLevelSlow + 0.03 * rLevelFilt;     (* 慢滤波 τ = 0.65s *)
END_IF
rLevelRate := (rLevelFilt - rLevelSlow) / 0.49;

(* ---------- 2. 联锁 (最高优先级) ---------- *)
IF bSensorFault THEN
    bPumpCmd      := FALSE;
    GVL.rValveCmd := 0.0;
    rInt          := 0.0;
ELSIF (GVL.rLevel >= rLvlHH) THEN
    bPumpCmd      := TRUE;
    GVL.rValveCmd := 100.0;
    rInt          := 0.0;
ELSIF (GVL.rLevel <= rLvlLL) THEN
    bPumpCmd      := FALSE;
    GVL.rValveCmd := 0.0;
    rInt          := 0.0;
ELSE
    (* ---------- 3. 目标变更检测与精准分段前馈 ---------- *)
    rError := rLevelFilt - GVL.rTargetLevel;

    IF ABS(GVL.rTargetLevel - rPrevTarget) > 1.0 THEN
        rInt        := 0.0;
        rPrevTarget := GVL.rTargetLevel;
        IF rError < 0.0 THEN
            bRisingTask := TRUE;
        ELSE
            bRisingTask := FALSE;
        END_IF
    END_IF

    (* 四点实测基准曲线：50mm->46.2%, 60mm->45.0%, 100mm->32.5%, 150mm->25.8% *)
    rTargetMM := 28.31 + (GVL.rTargetLevel / 100.0) * (157.45 - 28.31);
    IF rTargetMM <= 65.0 THEN
        rUeqBase := 46.2 - (rTargetMM - 50.0) * 0.120;
    ELSIF rTargetMM <= 105.0 THEN
        rUeqBase := 44.4 - (rTargetMM - 65.0) * 0.320;
    ELSE
        rUeqBase := 31.6 - (rTargetMM - 105.0) * 0.129;
    END_IF

    (* 中高水位区 (75mm ~ 135mm) 上升任务机械回差补偿（完美复刻 60->100mm 的零超调表现，且不干扰 60mm 与 150mm） *)
    IF bRisingTask AND rTargetMM >= 75.0 AND rTargetMM <= 135.0 THEN
        rUeqDyn := rUeqBase + 8.0 * (1.0 - ABS(rTargetMM - 100.0) / 40.0);
    ELSE
        rUeqDyn := rUeqBase;
    END_IF
    rUeqDyn := LIMIT(0.0, rUeqDyn, 100.0);

    (* ---------- 4. 核心控制（拔除上升 rMinOut 卡死陷阱） ---------- *)
    IF ABS(rError) <= 4.0 THEN
        rPD := (rKp * 1.15) * rError + (rVelDampGain * 1.15) * rLevelRate;
    ELSE
        rPD := (rKp * 0.90) * rError + rVelDampGain * rLevelRate;
    END_IF

    (* 【核心修复 1】：彻底移除上升段 (bRisingTask) 的 rMinOut 托底！
       - 仅在下降段远距离 (rError > 8.0%) 保留已完美验证的 (rUeqDyn + 2.0%) 托底；
       - 上升段与近距离稳态区 rMinOut 永远为 12.0%，彻底解决 50->60mm 卡在 44.8% 以及 150mm 卡在 22.9% 的问题！ *)
    rMinOut := 12.0;
    rMaxOut := 75.0;
    IF rError > 8.0 THEN
        rMinOut := LIMIT(12.0, rUeqDyn + 2.0, 75.0);
    END_IF

    (* 【核心修复 2】：取消激进的 2.2 倍破冰积分，将稳态锁定靶心设为 ±1.1% (约 ±1.4mm)
       - 在 50->60mm 等小步长任务中，允许全量程 (|rError| <= 8.0%) 连续积分；
       - 当水位进入目标 ±1.1% (如 150mm 的 148.6~151.4mm) 且速度平稳时，积分器休眠保平，
         彻底消除 150mm 处 t=64~80s 因为积分过度累加引发的慢速台阶跳变！ *)
    rEffKi := rKi * 0.85;
    rUnsaturatedOut := rUeqDyn + rPD + rEffKi * rInt;

    IF ABS(rError) <= 8.0 THEN
        IF ABS(rError) <= 1.1 AND ABS(rLevelRate) < 0.25 THEN
            (* 已进入 ±1.4mm 稳态靶心：冻结积分，保持水平直线 *)
        ELSIF (rError > 2.0 AND rLevelRate < -0.5) OR (rError < -2.0 AND rLevelRate > 0.5) THEN
            (* 正在高速冲向目标：暂停同向积分防超调 *)
        ELSIF NOT ((rUnsaturatedOut >= 75.0 AND rError > 0.0) OR (rUnsaturatedOut <= 12.0 AND rError < 0.0)) THEN
            (* 注意：抗饱和只对比物理上下限 12.0 和 75.0，绝不对比动态 rMinOut，确保积分器永不卡死！ *)
            rInt := rInt + rError * rDt;
        END_IF
    ELSE
        rInt := 0.0;
    END_IF

    rInt := LIMIT(-80.0, rInt, 80.0);

    (* 最终输出 *)
    rOut := rUeqDyn + rPD + rEffKi * rInt;
    GVL.rValveCmd := LIMIT(rMinOut, rOut, rMaxOut);

    rPrevError := rError;

    (* ---------- 5. 泵启停迟滞 ---------- *)
    IF bPumpCmd THEN
        IF GVL.rValveCmd < rPumpOffThr THEN bPumpCmd := FALSE; END_IF
    ELSE
        IF GVL.rValveCmd > rPumpOnThr  THEN bPumpCmd := TRUE;  END_IF
    END_IF
END_IF

(* ---------- 6. 手动模式 ---------- *)
IF NOT bAuto THEN
    GVL.rValveCmd := LIMIT(0.0, GVL.rManualValve, 100.0);
    bPumpCmd      := bManualPump;
    IF rKi <> 0.0 THEN
        rInt := (GVL.rValveCmd - rUeqDyn - rKp * (GVL.rLevel - GVL.rTargetLevel)) / rKi;
        rInt := LIMIT(-80.0, rInt, 80.0);
    END_IF
    rPrevError := GVL.rLevel - GVL.rTargetLevel;
    rLevelFilt := GVL.rLevel;
    rLevelSlow := GVL.rLevel;
END_IF

(* ---------- 7. 输出标定与写入 ---------- *)
iValveOut := LIMIT(iRawMin,
    iRawMin + REAL_TO_INT(GVL.rValveCmd / 100.0 * 16000.0), iRawMax);
bPump := bPumpCmd AND NOT bSensorFault;

(* ---------- 8. 报警汇总 ---------- *)
bAlarm := bSensorFault OR (GVL.rLevel >= rLvlHH) OR (GVL.rLevel <= rLvlLL);

(* ===== 9. Modbus 写出: PLC -> 上位机 ===== *)
qwModbusTCP_Server_Device_2 := %MW4;    
qwModbusTCP_Server_Device_3 := %MW5;
qwModbusTCP_Server_Device_4 := %MW6;    
qwModbusTCP_Server_Device_5 := %MW7;
qwModbusTCP_Server_Device_8.0 := bAuto;         
qwModbusTCP_Server_Device_8.1 := bPumpCmd;      
qwModbusTCP_Server_Device_8.2 := bSensorFault;  
qwModbusTCP_Server_Device_8.3 := bAlarm;        
qwModbusTCP_Server_Device_8.4 := bPump;         
qwModbusTCP_Server_Device_9  := INT_TO_WORD(iLevelRaw);   
qwModbusTCP_Server_Device_10 := %MW20;   
qwModbusTCP_Server_Device_11 := %MW21;   
qwModbusTCP_Server_Device_12 := %MW22;   
qwModbusTCP_Server_Device_13 := %MW23;   
qwModbusTCP_Server_Device_14 := %MW24;   
qwModbusTCP_Server_Device_15 := %MW25;   
qwModbusTCP_Server_Device_0 := %MW26;    
qwModbusTCP_Server_Device_1 := %MW27;
qwModbusTCP_Server_Device_6 := %MW28;    
qwModbusTCP_Server_Device_7 := %MW29;