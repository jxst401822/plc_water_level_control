PROGRAM PLC_PRG
VAR
	    (* ===== 周期 ===== *)
    tCycle : TIME := T#100MS;
    rDt    : REAL;                       (* 采样周期, 秒 *)

    (* ===== 物理IO (地址须与 Device IO Mapping 核对) ===== *)
    iLevelRaw AT %IW3    : INT;          (* 水位 4-20mA, 4000..20000 = 0..100% *)
    iValveOut  AT %QW3   : INT;          (* 抽水阀开度 4-20mA, 4000..20000 = 0..100% *)
    bPump      AT %QX4.0 : BOOL;         (* 抽水泵继电器, ON=抽水 *)

    (* ===== HMI / Modbus 设定 (地址可按需调整) ===== *)
    //rTargetLevel AT %MW0   : REAL := 40.0;    (* 目标水位 %, 占 MW0-1 *)
    bAuto        AT %MX102.0 : BOOL := TRUE;    (* 1=自动 0=手动 *)
   // rManualValve AT %MW2   : REAL := 0.0;     (* 手动阀开度 %, 占 MW2-3 *)
    bManualPump  AT %MX106.0 : BOOL := FALSE;   (* 手动泵 ON *)

    (* ===== 状态/报警 (只读供 HMI) ===== *)
	//rLevel       AT %MW4   : REAL;     (* 实测水位 %, 占 MW4-5 *)
	//rValveCmd    AT %MW6   : REAL;     (* 阀指令 %, 占 MW6-7 *)
    bPumpCmd     AT %MX114.0 : BOOL;     (* 泵指令 *)
    bSensorFault AT %MX114.1 : BOOL;     (* 传感器故障 *)
    bAlarm       AT %MX114.2 : BOOL;     (* 总报警 *)

    (* ===== 量程 (INT) ===== *)
    iRawMin : INT := 4000;
    iRawMax : INT := 20000;

    (* ===== PID 内部 ===== *)
    rError, rPrevError, rInt, rDeriv, rPD, rOut : REAL;

    (* ===== PID 参数 (待整定, 见文档) ===== *)
    //rKp : REAL := 3.0;
   // rKi : REAL := 0.1;     (* 1/s, 抵消自然入流静差; 积分对象 Ki 勿过大 *)
   // rKd : REAL := 0.0;
	rKp AT %MW20 : REAL := 3.0;      (* 占 MW20-21, 默认值(上电/未使能时用) *)
    rKi AT %MW22 : REAL := 0.5;      (* 占 MW22-23 *)
    rKd AT %MW24 : REAL := 0.0;      (* 占 MW24-25 *)
    bPidTune : BOOL := FALSE;        (* 1=采用 Modbus 传入的 Kp/Ki/Kd, 0=用上面默认 *)

    (* ===== 联锁阈值 (%, 工程量) ===== *)
    rLvlHH      : REAL := 95.0;   (* 高高: 最大抽水降压 *)
    rLvlLL      : REAL := 5.0;    (* 低低: 停泵防干抽, 靠自流回涨 *)
    rPumpOnThr  : REAL := 3.0;    (* 阀开度>3% 开泵 *)
    rPumpOffThr : REAL := 1.0;    (* 阀开度<1% 关泵(防憋压) *)
	
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
IF (iLevelRaw < iRawMin - 200) OR (iLevelRaw > iRawMax + 200) THEN
    bSensorFault := TRUE;
ELSE
    bSensorFault := FALSE;
END_IF
GVL.rLevel := LIMIT(0.0,
    (INT_TO_REAL(iLevelRaw - iRawMin) / 16000.0) * 100.0, 100.0);

(* ---------- 2. 联锁 (最高优先级) ---------- *)
IF bSensorFault THEN
    (* 传感器故障: 停泵关阀+报警 (自然入流会缓慢上升, 需人工干预) *)
    bPumpCmd  := FALSE;
    GVL.rValveCmd := 0.0;
    rInt      := 0.0;
ELSIF (GVL.rLevel >= rLvlHH) THEN
    (* 高高液位: 最大抽水快速降压 *) 
    bPumpCmd  := TRUE;
    GVL.rValveCmd := 100.0;
    rInt      := 0.0;
ELSIF (GVL.rLevel <= rLvlLL) THEN
    (* 低低液位: 停抽防干抽, 靠自然入流回涨 *)
    bPumpCmd  := FALSE;
    GVL.rValveCmd := 0.0;
    rInt      := 0.0;
ELSE
    (* ---------- 3. PID (阀在抽水侧: 偏高->开阀抽水) ---------- *)
    rError := GVL.rLevel - GVL.rTargetLevel;            (* PV - SP, 正=偏高需抽水 *)
    rDeriv := (rError - rPrevError) / rDt;
    rPD    := rKp * rError + rKd * rDeriv;
    rOut   := rPD + rKi * rInt;

    (* 抗积分饱和: 输出饱和且误差同向时冻结积分 *)
    IF NOT ((rOut > 100.0 AND rError > 0.0) OR (rOut < 0.0 AND rError < 0.0)) THEN
        rInt := rInt + rError * rDt;
        rInt := LIMIT(-100.0, rInt, 100.0);
    END_IF
    GVL.rValveCmd := LIMIT(0.0, rPD + rKi * rInt, 100.0);
    rPrevError := rError;

    (* ---------- 4. 泵启停迟滞 (保护继电器+防憋压) ---------- *)
    IF bPumpCmd THEN
        IF GVL.rValveCmd < rPumpOffThr THEN bPumpCmd := FALSE; END_IF
    ELSE
        IF GVL.rValveCmd > rPumpOnThr  THEN bPumpCmd := TRUE;  END_IF
    END_IF
END_IF

(* ---------- 5. 手动模式 (含无扰切换: 积分跟踪手动阀开度) ---------- *)
IF NOT bAuto THEN
    GVL.rValveCmd := LIMIT(0.0, GVL.rManualValve, 100.0);
    bPumpCmd  := bManualPump;
    IF rKi <> 0.0 THEN
        rInt := (GVL.rValveCmd - rKp * (GVL.rLevel - GVL.rTargetLevel)) / rKi;
        rInt := LIMIT(-100.0, rInt, 100.0);
    END_IF
    rPrevError := GVL.rLevel - GVL.rTargetLevel;
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