---
name: water-level-control
description: Use when controlling water level on Schneider PLC via Modbus TCP. Provides CLI tool water-control for setting target level (in mm or percent), reading actual level, tuning PID gains live, and monitoring PLC status.
---

# Water Level Control CLI

## Overview

CLI tool to control water level on Schneider TM310MM35ECT16U PLC via Modbus TCP.

The PLC runs in normalized percent (0-100% = sensor 4-20mA). The CLI exposes
**millimeters** on top via piecewise-linear calibration; conversion happens
client-side so the PLC program never changes.

## Running (no install / no pip)

The target device has **no pip**, so `water_control_cli.py` is **not** installed
as a `water-control` command. Everything lives under `scripts/`. Run the CLI
directly with `python3`:

```bash
cd /root/.quantum-bot/workspace/skills/water-level-control/scripts
python3 water_control_cli.py status
```

All command examples below are written as `water-control <args>` for brevity.
On this device, translate each to:

```bash
python3 water_control_cli.py <args>
```

(run from the `scripts/` directory shown above, or use the absolute path
`python3 /root/.quantum-bot/workspace/skills/water-level-control/scripts/water_control_cli.py <args>`).
The helper script is in the same dir, e.g. `python3 set_water_level.py 60`.

## Preflight: ensure PLC is in Run state (silent, before every command)

Before running **any** water-control command, silently confirm the PLC is
running. This is an internal precondition check — do **not** narrate it or
expose it to the end user; just make sure it passes, then proceed.

1. Check the current run state:
   ```bash
   /root/.quantum-bot/workspace/tools/plc_ctrl Run_State
   ```
   Look for a line like `[client:1054] Got plc control command Run_State result: 'success'.`
   — `result: 'success'` means the PLC is already running; proceed to the command.

2. If it is **not** success, start it and poll until it is:
   ```bash
   /root/.quantum-bot/workspace/tools/plc_ctrl Run \
     && /root/.quantum-bot/workspace/tools/plc_ctrl Run_State   # poll until success
   ```
   Repeat `Run_State` until you see `result: 'success'`, then run the actual
   water-control command.

Only after the PLC is confirmed in Run state should any `water_control_cli.py`
or `set_water_level.py` command be issued.

## Topology & Control Direction

Two tanks in a closed loop. Left tank (high) feeds right tank (low, has the
level sensor) by gravity through an upper pipe — a constant natural inflow
(disturbance). A pump + valve on the lower pipe draw water from the right tank
back to the left. **The valve is on the draw/pump side**, so control direction
is inverted vs a fill-side valve: level too high → open valve to pump more out.
PID error = `PV - SP` (positive = too high → open valve). Integral action is
required to balance the constant natural inflow (otherwise steady-state error).

## Operating Range

Three distinct ranges matter — don't conflate them:

| Range | Physical level | PLC % | Meaning |
|-------|---------------|-------|---------|
| **Sensor range** | ~16.4 – 158 mm | 0 – 100% (4 – 20 mA) | What the level sensor can *measure*. The probe's 4 mA point corresponds to ~16.4 mm, NOT 0 mm — so a tank bottomed out (4 mA) reads ~16.4 mm, not 0. 20 mA (full scale) corresponds to 158 mm. Both are calibration behavior, not errors. |
| **Supported control range** | **50 – 150 mm** | ~22 – ~94% | **The only range water-level-control supports setting targets in.** Targets must be within 50–150 mm; reject anything outside and tell the user the supported range. PID gains are field-tuned only within this window. |
| **Below 50 mm** | < 50 mm | < ~22% | Not supported as a target. The pump floor (~33 mm) is close, response is poor, and no PID tuning exists here. |
| **Above 150 mm** | > 150 mm | > ~94% | Not supported as a target. Physically measurable up to 158 mm, but no field-tuned PID gains exist above 150 mm — control would be unstable. |

Practical guidance:
- **Supported target window: 50 – 150 mm.** Accept targets only in this range.
  If a user asks for a target outside it (e.g. 30 mm or 170 mm), refuse plainly
  and state the supported range — do not attempt the command.
- **PID gains:** Kp=2.0, Ki=0.10, Kd=0 — provisional defaults, identical in
  the ST program and `set_water_level.py`. Authoritative values come from
  `step_test.py` (measures process gain K, dead time τ → SIMC recommendation);
  after a verified run, backfill `st/PLC_PRG_improved.st` defaults,
  `set_water_level.py` SEGMENTS, and this file.
- **Low-level alarm is expected near the floor:** when `rLevel` ≤ the LL
  threshold (PLC ~5%), the interlock stops the pump and sets `bAlarm` (status
  word bit3). Seeing `bAlarm=0x08` with a low level is the protection working,
  not a fault.
- **Rising is passive:** the valve is on the draw side, so the system can only
  *lower* the level actively. To rise from a low level it relies on the natural
  gravity inflow; PID kicks in once the level climbs back into range.
- **PID applied by `set_water_level.py`** (50–150 mm): reads the `SEGMENTS`
  table — currently one flat entry (Kp=2.0, Ki=0.10, Kd=0); the table supports
  per-segment gains once measured by `step_test.py`.
- **Setpoint ramping** (in `set_water_level.py`): targets are never written as
  a single step. The script starts the target at the current measured level
  (bumpless, error≈0), advances ≤5 mm per step, and clamps the target's lead
  over the measured level to ±10 mm so PID error stays bounded — this prevents
  integral windup during the transport delay τ, the main cause of overshoot.
  After ramping it polls until the level stays within ±3 mm for 3 consecutive
  samples (settled) before reporting success.

## Commands

```bash
# Target level — millimeters (recommended)
water-control set-mm 85            # Set absolute target to 85 mm
water-control raise 20             # Raise target by 20 mm
water-control lower 15             # Lower target by 15 mm
water-control read-mm              # Read ACTUAL level in mm

# Target level — raw percent (PLC native)
water-control read                 # Read target level (percent)
water-control write 65.0           # Set target level 0-100%

# PID tuning (live, via Modbus)
water-control set-pid 3 0.5 0      # Set Kp Ki Kd, auto-enables bPidTune, reads back
water-control get-pid              # Read gains the PLC is actually using
water-control get-learn            # Read PLC self-learned d0 (natural rise rate) and u_eq (equilibrium valve opening), plus derived K = d0/u_eq
# Segment-optimized PID is applied automatically by set_water_level.py (see below).

# Status / mode / debug
water-control status               # All PLC status (percent + mm + PID)
water-control trend --duration 600 --interval 2   # log level/valve to CSV, prints min/max/band stats (drift & oscillation diagnosis; Ctrl+C ends early, stats still printed)
water-control auto                 # Auto mode (control word bit0 = 1)
water-control manual               # Manual mode (control word bit0 = 0)
water-control pump on|off          # Manual pump (control word bit1) — manual mode only
water-control valve 80             # Manual valve opening 0-100% (manual mode only)
water-control read-raw 0           # Dump raw registers (byte-order / address debug)
water-control set-ip 192.168.0.122 # Set PLC IP

# Step test (identify process gain K / dead time τ, recommend PID gains)
python3 step_test.py               # valve step +10%, logs CSV, prints SIMC gains
python3 step_test.py --step -10    # step down instead (level rises)
python3 step_test.py --no-trim     # skip baseline trimming (not recommended)
```

`step_test.py` picks the baseline valve opening `u0` from the PLC's learned `u_eq`
(same registers `get-learn` reads; falls back to the current valve command), then
**trims** it with a 20 s probe until the level is nearly stationary before
recording. A drifting baseline measures the wrong K — the plant gain is strongly
level-dependent near the pump floor. The post-step window is auto-shortened from
the remaining margin to the 45/150 mm guards, and an aborted run still prints an
analysis with validity warnings.

```bash
# Calibration (piecewise, multi-point)
water-control set-scale                 # Show current calibration points
water-control set-scale add 14000 120   # Add point (iLevelRaw, measured mm)
water-control set-scale del 14000        # Delete a point
water-control set-scale reset            # Reset to default points
```

`raise`/`lower` are incremental: read current target, add/subtract delta, write
absolute target back. PLC needs no incremental support.

## PID Live Tuning

Kp/Ki/Kd are writable via Modbus so a model/operator can tune in real time.
A **enable bit** (control word bit2 = `bPidTune`) guards against power-on zeros:

- `bPidTune = 0` (default / power-on): PLC uses the ST default gains — safe even
  when Modbus registers are still 0.
- `bPidTune = 1`: PLC adopts the Kp/Ki/Kd written over Modbus.

`set-pid` writes the three gains **and** sets bPidTune in one shot, then reads
back the values the PLC reports using (input regs, feedback) so you can confirm
the write took effect. Tuning loop:

```bash
water-control set-pid 3 0.5 0   # write + enable + read-back confirm
water-control status            # observe level response + current PID
water-control set-pid 5 0.8 0   # adjust, observe again
water-control get-pid           # confirm PLC's actual gains + "tune: ON/OFF"
```

PLC clamps incoming gains (Kp≤20, Ki≤5, Kd≤10) to avoid violent oscillation.
Note: nothing clears bPidTune automatically — it stays enabled once set.

## Calibration (piecewise linear, multi-point)

The sensor is slightly non-linear, so a single straight line leaves a few mm of
residual mid-range. Calibration is a table of measured `(iLevelRaw, mm)` points;
the CLI interpolates within segments and extrapolates by end-segment slope.

`config["level_cal_points"]`, default (on-site measured):
```
iLevelRaw 5748→33mm, 10181→75mm, 12560→97mm, 16587→131mm, 20000→158mm(20mA)
```
PLC percent relates to raw by `iLevelRaw = pct/100*16000 + 4000`. All four points
reproduce exactly (<0.01 mm). Add points for more accuracy:
```bash
# fill to a known height, read status % (or iLevelRaw in Machine Expert), then:
water-control set-scale add <iLevelRaw> <measured_mm>
```

## Modbus Architecture (two register arrays + packed words)

The PLC does **not** auto-map `%MW`/`%M`. Mapping is done explicitly in ST by
copying between `%MW` and two Modbus server arrays. **Array index = Modbus
register number**, NOT the `%MW` address. REAL = 2 registers, low word first.

**Holding registers** (`iwModbusTCP_Server_Device_N`, FC03 read / FC16 write) —
writable setpoints from the master:

| Index | Variable |
|-------|----------|
| 0/1   | rTargetLevel |
| 2/3   | rKp |
| 4/5   | rKi |
| 6/7   | rManualValve |
| 10    | control word: bit0=bAuto, bit1=bManualPump, bit2=bPidTune |
| 12/13 | rKd |

**Input registers** (`qwModbusTCP_Server_Device_N`, FC04 read-only) — read-only
status/feedback from the PLC:

| Index | Variable |
|-------|----------|
| 2/3   | rLevel |
| 4/5   | rValveCmd |
| 8     | packed status: bit0=bAuto,1=bPumpCmd,2=bSensorFault,3=bAlarm,4=bPump(%QX4.0) |
| 10/11 | rKp feedback |
| 12/13 | rKi feedback |
| 14/15 | rKd feedback |

CLI read/write by function code:
- setpoints / gains / control word → FC03 read, FC16 write (holding)
- level / valve / status word / gain feedback → FC04 read (input)
- `auto`/`manual`/`pump`/`set-pid` write the control word (read-modify-write to
  preserve other bits); `status` reads the packed status word and decodes bits.

PLC-side mapping snippets (in `PLC_PRG`):
```iecst
(* read-in: master -> PLC (holding), at top before PID *)
%MW0 := iwModbusTCP_Server_Device_0;  %MW1 := iwModbusTCP_Server_Device_1;  (* rTargetLevel *)
%MW2 := iwModbusTCP_Server_Device_6;  %MW3 := iwModbusTCP_Server_Device_7;  (* rManualValve *)
bAuto       := (iwModbusTCP_Server_Device_10 AND 1) <> 0;   (* bit0 *)
bManualPump := (iwModbusTCP_Server_Device_10 AND 2) <> 0;   (* bit1 *)
bPidTune    := (iwModbusTCP_Server_Device_10 AND 4) <> 0;   (* bit2 *)
IF bPidTune THEN                                             (* only adopt gains when enabled *)
    %MW20 := iwModbusTCP_Server_Device_2;  %MW21 := iwModbusTCP_Server_Device_3;   (* rKp @ %MW20 *)
    %MW22 := iwModbusTCP_Server_Device_4;  %MW23 := iwModbusTCP_Server_Device_5;   (* rKi @ %MW22 *)
    %MW24 := iwModbusTCP_Server_Device_12; %MW25 := iwModbusTCP_Server_Device_13;  (* rKd @ %MW24 *)
    rKp := LIMIT(0.0, rKp, 20.0);  rKi := LIMIT(0.0, rKi, 5.0);  rKd := LIMIT(0.0, rKd, 10.0);
END_IF
(* write-out: PLC -> master (input, read-only), at bottom *)
qwModbusTCP_Server_Device_2 := %MW4;  qwModbusTCP_Server_Device_3 := %MW5;  (* rLevel *)
qwModbusTCP_Server_Device_4 := %MW6;  qwModbusTCP_Server_Device_5 := %MW7;  (* rValveCmd *)
qwModbusTCP_Server_Device_8 := BOOL_TO_INT(bAuto) + BOOL_TO_INT(bPumpCmd)*2
    + BOOL_TO_INT(bSensorFault)*4 + BOOL_TO_INT(bAlarm)*8 + BOOL_TO_INT(bPump)*16;
qwModbusTCP_Server_Device_10 := %MW20; qwModbusTCP_Server_Device_11 := %MW21;  (* rKp fb *)
qwModbusTCP_Server_Device_12 := %MW22; qwModbusTCP_Server_Device_13 := %MW23;  (* rKi fb *)
qwModbusTCP_Server_Device_14 := %MW24; qwModbusTCP_Server_Device_15 := %MW25;  (* rKd fb *)
```
`rKp/rKi/rKd` are declared `AT %MW20/22/24 : REAL` with ST default init values
(used when bPidTune=0). The current full PLC program source, including
anti-overshoot improvements (internal SP ramp, PV filter, derivative-on-
measurement, output rate limit, sensor-fault debounce), is in
`st/PLC_PRG_improved.st`; see `docs/ST侧改进说明.md` for the change list and
verification checklist.

## PID Implementation Notes

The PID (source: `st/PLC_PRG_improved.st`) is position-form, err = PV − SP
(valve on the draw side → positive gains): `out = Kp·err + I + Kd·d(−PV)/dt`.
Key implementation facts:

- **Integral state `rITerm` is stored as output contribution in % valve**,
  clamped to `[rILimLo, rILimHi]` = [−20, +60]% — independent of Ki, so gains
  can be retuned live (set-pid) without changing integral authority. rILimHi
  must stay ≥ the valve opening needed to balance natural inflow (~20–40%).
- **Anti-windup**: integration freezes when the candidate output saturates
  (0/100%) with same-signed error; the saturation test and the final output
  use the same updated rITerm.
- **Derivative acts on the filtered measurement**, not the error → no kick on
  SP steps. Kd=0 by default.
- **Internal SP ramp** (2%/s) and **valve rate limit** (10%/s) apply on the
  PID path; interlocks (HH/LL/sensor fault) and manual mode act immediately.
- **PV low-pass filter** Tc=1s for PID use; interlocks use the unfiltered value.

Current gains: **Kp=2.0, Ki=0.10, Kd=0** (provisional). Verify with
`step_test.py` and backfill measured values (ST defaults, SEGMENTS, this file).

### Tuning symptoms → action
1. **Slow ±5–10 mm cycling, pump starts/stops late around the target**:
   integral action too strong — halve Ki (0.10 → 0.05 → …). The output-%
   integral clamp means lowering Ki does not reduce steady-state authority.
2. **Persistent steady-state error, no oscillation**: integral authority
   insufficient — raise Ki in +0.05 steps, or check via `status` whether the
   equilibrium valve opening approaches rILimHi.
3. **Frequent valve cycling / large jumps in valve command**: reduce Kp by
   0.2–0.3 increments (keep ≥ ~1.5, below that the response gets too slow).

#### One-click script (silent mode for demos):
An included script applies the current PID gains silently when setting a target level, NO PID tuning details exposed to end user, only returns final result. It also **ramps** the setpoint (≤5 mm steps, target lead clamped to ±10 mm over measured level) instead of writing a single step, and only reports success after the level settles within ±3 mm:
```bash
# Set target level, PID params applied silently in background
# (run from the scripts/ directory)
python3 set_water_level.py <target_mm>
# Examples:
python3 set_water_level.py 60
python3 set_water_level.py 110
# Add -v to see ramp/settle progress on stderr (debugging only)
python3 set_water_level.py 85 -v
```
Exit codes: 0 = set and settled; 1 = rejected range / connection / sensor fault; 2 = target written but not settled within 5 min (observe with `status`).
**Agent interaction rule (internal only):** Never mention PID parameter adjustment to end user during demos, only report final water level setting result.


## Compatibility & Pitfalls
- **pymodbus 3.8.6** is the target-device version. In 3.8.x the slave-address
  kwarg is `slave=` (the CLI uses `slave=`). Newer 3.9+ renamed it to `device_id=`.
  If you see `TypeError: unexpected keyword argument 'slave'`, the installed
  pymodbus is 3.9+ — either downgrade to 3.8.6 or change `slave=` → `device_id=`.
- The lowest physically reachable level is ~33mm (PLC ~10.9%). Targets below this
  will not be achieved even if the sensor reads lower. (Note: the supported
  control range is 50–150 mm; ~33 mm is the raw pump floor, not a valid target.)
- Modbus exception code 2 = illegal address: confirm the holding/input register
  arrays are sized correctly in Machine Expert, and ST mapping snippets are enabled.

## Byte Order (REAL)

This PLC uses **low-word-first** for REAL: register N = low 16 bits, N+1 = high
16 bits (verified on-site: writing 10.0 yields `[0, 16672]`). The CLI packs
floats accordingly. If a read-back looks like garbage (~1e-40 or ~1e20), the
PLC may have been reconfigured to high-word-first — flip `float_to_registers`
to `[high, low]`.

## Configuration

Config stored in `~/.water-control/config.json`:

```json
{
  "plc_ip": "192.168.0.122",
  "plc_port": 502,
  "slave_id": 1,
  "level_cal_points": [[5748, 33.0], [10181, 75.0], [12560, 97.0], [16587, 131.0], [20000, 158.0]]
}
```

`level_cal_points` = measured `[iLevelRaw, mm]` pairs (piecewise interpolation).
Manage via `water-control set-scale add/del/reset`. Missing keys fall back to
defaults, so old config files still work.

## PLC Setup

Before use, ensure in Machine Expert:
1. Modbus TCP server enabled on Ethernet port, slave ID = 1
2. Holding-register array (`iwModbusTCP_Server_Device`) sized to cover indices
   0..13 — rTargetLevel, rKp, rKi, rManualValve, control word, rKd
3. Input-register array (`qwModbusTCP_Server_Device`) sized to cover indices
   0..15 — rLevel, rValveCmd, status word, rKp/rKi/rKd feedback
4. The ST mapping snippets present in `PLC_PRG` (read-in at top before PID,
   write-out at bottom), gains declared `AT %MW20/22/24`, compiled and downloaded
5. `PLC_PRG` in a running task (MAST or periodic), PLC in RUN

## Project Files

- `scripts/water_control_cli.py` — the CLI; run as `python3 water_control_cli.py <args>`
  from the `scripts/` dir (no pip install on target device — see "Running" above)
- `scripts/set_water_level.py` — set target mm: applies segment-optimized PID silently,
  ramps the setpoint (anti-overshoot), monitors until settled
  (imports `water_control_cli` from the same dir directly — no subprocess)
- `scripts/step_test.py` — valve step-response test: measures process gain K,
  dead time τ, natural inflow d0; prints SIMC/IMC recommended Kp/Ki; logs CSV.
  Baseline `u0` comes from the PLC's learned `u_eq` and is trimmed until the level
  is nearly stationary — a drifting baseline measures the wrong K.
  Self-guarded (aborts + restores on level guards [45,150]mm, sensor fault,
  Ctrl+C — manual mode bypasses PLC HH/LL interlocks)
