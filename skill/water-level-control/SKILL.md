---
name: water-level-control
description: Use when controlling water level on Schneider PLC via Modbus TCP. Provides the water_control_cli.py script for setting target level (in mm or percent), reading actual level, calibrating the level sensor, tuning PID gains live, and monitoring PLC status.
---

# Water Level Control CLI

## Overview

CLI tool to control water level on Schneider TM310MM35ECT16U PLC via Modbus TCP.

The PLC runs in normalized percent (0-100% = sensor 4-20mA). The CLI exposes
**millimeters** on top via piecewise-linear calibration; conversion happens
client-side so the PLC program never changes.

## Running (no install / no pip)

The target device has **no pip**, so there is no installed console command —
everything is run directly with `python3`. **`cd` into `scripts/` once first:**

```bash
cd /root/.quantum-bot/workspace/skills/water-level-control/scripts
python3 water_control_cli.py status
```

Every example in this file is written literally as
`python3 water_control_cli.py <args>` and can be copy-pasted as-is, assuming the
working directory is the `scripts/` directory above. From anywhere else, use the
absolute path:

```bash
python3 /root/.quantum-bot/workspace/skills/water-level-control/scripts/water_control_cli.py <args>
```

## Preflight: ensure PLC is in Run state (silent, before every command)

Before running **any** CLI command, silently confirm the PLC is
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
   CLI command.

Only after the PLC is confirmed in Run state should any `water_control_cli.py`
command be issued.

## Topology & Control Direction

Two tanks in a closed loop. Left tank (high) feeds right tank (low, has the
level sensor) by gravity through an upper pipe — a natural inflow (disturbance).
A pump + valve on the lower pipe draw water from the right tank back to the
left. **The valve is on the draw/pump side**, so control direction is inverted
vs a fill-side valve: level too high → open valve to pump more out. PID error =
`PV - SP` (positive = too high → open valve). Integral action is required to
hold any level (otherwise steady-state error).

**The plant is self-regulating, not integrating**: with the valve held at a fixed
opening the level settles to a *new plateau* rather than drifting at constant
rate (confirmed 2026-10-26 by three 300 s plateaus at valve 20 / 23 / 26 %).
Each opening therefore has an equilibrium level, and the opening that holds a
given target is a decreasing line `u_eq(h)`.

⚠️ **The numbers for that line are not settled — do not quote them as fact.**

| Source | Result | Status |
|---|---|---|
| 2026-10-20 step test | `K = −2.13 %level/%valve`, `τp = 18.6 s`, `θ = 5.0 s` | **Superseded.** Two plateaus solving two unknowns — zero redundancy, no way to check the fit |
| 2026-10-26 plateaus (121.9–147.1 mm) | `u_eq = 53.56 − 0.2295·mm`, i.e. \|K\| ≈ 3.4 %level/%valve; max residual 0.62 pp | Valid **only over 122–147 mm**, and only for the loop's water distribution at that moment |
| 2026-10-26, 35 min later | valve 24.46 % balanced at ~96 mm, not the ≥127 mm the line predicts | The line moved **≥10 pp in under an hour** |

Two consequences for anyone tuning against this model:

- **Do not extrapolate the line below ~120 mm.** The local slope is not constant
  (2.85 mm/pp between 26→23 %, 5.57 mm/pp between 23→20 %), so a straight-line
  extrapolation under-predicts the opening needed at low level.
- **Because the loop is closed, `u_eq` depends on how the fixed total water volume
  is split between the two tanks.** Any plateau measurement is only valid for the
  split it was taken at — refill the left tank to a marked level before each point,
  or the "line" you fit is a mixture of different states.

Opening the valve to `u_eq(SP)` while below the target makes the level approach SP
monotonically and self-brake — the plant provides the brake, no lead time needed.
That argument holds only for as long as `u_eq` is accurate.

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
- **PID gains:** Kp=2.0, Ki=0.10, Kd=0 — ST default gains, known
  to be **2–3× too aggressive** (matches the residual ±3.5 mm slow wander).
  `step_test.py` measured FOPDT K=2.13 / τp=18.6 s / θ=5.0 s on 2026-10-20,
  giving SIMC **Kp 0.58 / Ki 0.031 (robust)** or **Kp 0.88 / Ki 0.047 (standard)**;
  apply with `set-pid` and confirm no oscillation before backfilling the defaults.
- **Low-level alarm is expected near the floor:** when `rLevel` ≤ the LL
  threshold (PLC ~5%), the interlock stops the pump and sets `bAlarm` (status
  word bit3). Seeing `bAlarm=0x08` with a low level is the protection working,
  not a fault.
- **Rising is passive:** the valve is on the draw side, so the system can only
  *lower* the level actively. To rise from a low level it relies on the natural
  gravity inflow; PID kicks in once the level climbs back into range.
- **Ramp / anti-overshoot lives in the PLC**, not in the CLI: `set-mm` writes the
  target once and returns. `PLC_PRG` then limits the setpoint to `rSpRate=2%/s`
  internally ([G2]), freezes integration when the output saturates ([G5]), and —
  once `bPreOpenEn := TRUE` — pre-opens the valve to the target's equilibrium
  opening so the level self-brakes at the target ([G11]/[G13]).
- **Verifying a move**: `set-mm 60` then poll `read-mm` / `status`, or record with
  `trend`. Settled ≈ level within ±3 mm of target and no longer drifting.

## Commands

```bash
# Target level — millimeters (recommended)
python3 water_control_cli.py set-mm 85                        # Set absolute target to 85 mm
python3 water_control_cli.py raise 20                         # Raise target by 20 mm
python3 water_control_cli.py lower 15                         # Lower target by 15 mm
python3 water_control_cli.py read-mm                          # Read ACTUAL level in mm
python3 water_control_cli.py read-level-raw                   # Read iLevelRaw counts + derived %/mm (calibration)
python3 water_control_cli.py calibrate --show                 # Show the level model (K, b) and the raw<->mm table
python3 water_control_cli.py calibrate --actual 104           # Fix the offset b from a ruler-measured level
python3 water_control_cli.py calibrate --actual 61.5 --check  # Verify only, do not write
python3 water_control_cli.py calibrate --three-point 7542 55.3 12631 100.2 16830 145.1  # refit K and b, prints residuals

# Target level — raw percent (PLC native)
python3 water_control_cli.py read                             # Read target level (percent)
python3 water_control_cli.py write 65.0                       # Set target level 0-100%

# PID tuning (live, via Modbus)
python3 water_control_cli.py set-pid 3 0.5 0                  # Set Kp Ki Kd, auto-enables bPidTune, reads back
python3 water_control_cli.py get-pid                          # Read gains the PLC is actually using
python3 water_control_cli.py get-learn                        # Read what the PLC maps into input regs 0/1 and 6/7 (see warning below)
# Gains are set explicitly with set-pid; set-mm only writes the target (see PID Live Tuning).

# Status / mode / debug
python3 water_control_cli.py status                           # All PLC status (percent + mm + PID)
python3 water_control_cli.py trend --duration 600 --interval 2  # log level/valve to CSV, prints min/max/band stats (drift & oscillation diagnosis; Ctrl+C ends early, stats still printed)
python3 water_control_cli.py auto                             # Auto mode (control word bit0 = 1)
python3 water_control_cli.py manual                           # Manual mode (control word bit0 = 0)
python3 water_control_cli.py stop                             # Quiesce: pump OFF + valve 0% + manual, then verify the level really stops
python3 water_control_cli.py resume                           # Leave the stopped state: preload the integral with u_eq, then go auto
python3 water_control_cli.py pump on|off                      # Manual pump (control word bit1) — manual mode only
python3 water_control_cli.py valve 80                         # Manual valve opening 0-100% (manual mode only)
python3 water_control_cli.py read-raw 0                       # Dump raw registers (byte-order / address debug)
python3 water_control_cli.py set-ip 192.168.0.122             # Set PLC IP

# Step test (identify process gain K / dead time τ, recommend PID gains)
python3 step_test.py                                          # valve step +10%, logs CSV, prints SIMC gains
python3 step_test.py --step -10                               # step down instead (level rises)
python3 step_test.py --no-trim                                # skip baseline trimming (not recommended)
```

`step_test.py` auto-detects the plant type: if the level settles to a new plateau
after the step it fits **FOPDT** (K, τp, θ by the two-point method, plus the
equilibrium-line intercept C) and prints SIMC gains for λ=θ/2θ/3θ; if the level
keeps drifting it falls back to **IPDT** (tangent method, K, τ, d0). Baseline `u0`
comes from the PLC's learned u_eq converted to the *current* level's equilibrium
(`+ (SP − level)/2.13`) and is trimmed with a 20 s probe until nearly stationary.
Runs are refused as unusable when the robust tier lands outside Kp≤10 / Ki≤1,
and any validity warning suppresses the "apply this" command line. The post-step
window is auto-shortened from the margin to the 45/150 mm guards; an aborted run
still prints an analysis with warnings.

```bash
# Calibration (piecewise, multi-point) — LEGACY, only used if level_model is absent
python3 water_control_cli.py set-scale                # Show current calibration points
python3 water_control_cli.py set-scale add 14000 120  # Add point (iLevelRaw, measured mm)
python3 water_control_cli.py set-scale del 14000      # Delete a point
python3 water_control_cli.py set-scale reset          # Reset to default points
```

`raise`/`lower` are incremental: read current target, add/subtract delta, write
absolute target back. PLC needs no incremental support.

### ⚠️ `get-learn` reads whatever the PLC maps into input registers 0/1 and 6/7

The CLI labels register 0/1 as **C** (equilibrium-line intercept) and 6/7 as
**u_eq**, and cross-checks `C = u_eq + SP/STAT_K`. That mapping only means
something for the `PLC_PRG_simple` / `_v3` programs. Other programs put different
variables in those slots:

| Program | reg 0/1 (`%MW26`) | reg 6/7 (`%MW28`) | `get-learn` output |
|---|---|---|---|
| `PLC_PRG_simple`, `_v3` | learned intercept C | derived `u_eq(SP)` | meaningful |
| `PLC_PRG_v4` | **`rRiseSpeedOffset`** (= 10.0) | `rUeqDyn` | "C = 10.0" is **not** an intercept — it is the rise offset; the consistency check is meaningless |
| `PLC_PRG_Current`, `_improved` | learned inflow rate `d0` | learned `u_eq` | C/u_eq labels are wrong |

Check which program the PLC is running before trusting the numbers. `STAT_K`
(2.13) in the CLI is also stale — see Topology.

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
python3 water_control_cli.py set-pid 3 0.5 0  # write + enable + read-back confirm
python3 water_control_cli.py status           # observe level response + current PID
python3 water_control_cli.py set-pid 5 0.8 0  # adjust, observe again
python3 water_control_cli.py get-pid          # confirm PLC's actual gains + "tune: ON/OFF"
```

PLC clamps incoming gains (Kp≤20, Ki≤5, Kd≤10) to avoid violent oscillation.
Note: nothing clears bPidTune automatically — it stays enabled once set.

## Quiescent State (`stop` / `resume`)

The right tank has two connections: gravity inflow from the upper left tank, and
the pump+valve draw at the bottom. `stop` closes the bottom one and verifies the
water column is genuinely static; the upper link is blocked by hand.

```bash
python3 water_control_cli.py stop               # pump OFF + valve 0% + manual, then drift-check
python3 water_control_cli.py stop --settle 10   # longer drift check (default 5 s, 5 samples)
python3 water_control_cli.py resume             # preload the integral, then back to auto
python3 water_control_cli.py resume --valve 27  # preload with an explicit opening instead of u_eq
```

`stop` writes in this order: `rManualValve = 0` → `bManualPump = 0` → `bAuto = 0`.
The order matters — the manual valve path has **no** rate limit (`rValveRate` only
applies to the auto path), so clearing `bAuto` first would step the valve straight
to whatever stale value `rManualValve` held.

It then reads back the valve/pump and samples the level over `--settle` seconds:

| Reading | Meaning |
|---|---|
| valve ≈0, pump OFF, drift <1 mm | ✅ static — safe to calibrate or measure |
| valve >1 % or pump ON with `bAlarm` | the **HH interlock** is overriding manual (by design). Lower the level below `rLvlHH` first |
| valve ≈0, pump OFF, but drift >1 mm | the upper link is **not** blocked, or the tank is leaking |

Exit code is 2 unless the rig is verified static (`--force` overrides).

Use it before: `calibrate --sample` / `--three-point`, any ruler measurement,
open-loop disturbance tests, and probe inspection.

`resume` is not the same as `auto`. A self-regulating plant needs ~27 % valve
opening just to hold the level, and the integral can only build at `Ki·|e|` ≈
0.1 pp/s under a small error — starting it from zero means 4–5 minutes with the
valve nearly shut and the level climbing past the target. So `resume` first writes
`rManualValve = u_eq` (read back from the PLC's input register 6/7), lets the
manual branch track `rITerm` to that value, and only then sets `bAuto`. Pass
`--valve` if you would rather preload an opening you found by eye.

## Level Calibration (linear model, offset-only)

The hydrostatic probe + tank is modelled as one straight line:

```
mm = K * (iLevelRaw - 4000) + b        K = 0.0087683 mm/count (fixed)
```

`K` is the installation's inherent slope (probe span × tank geometry) — it does
not drift. `b` is "what 4 mA corresponds to"; wetting, temperature and zero drift
only translate the line, so **routine calibration changes `b` only**.

`b = 19.8 mm` was fitted by least squares over five historical on-site points
(5748→33, 10181→75, 12560→97, 16587→131, 20000→158 mm); max residual ±2.2 mm, so
a single line is a fair description. The old point table is still in the config
(`level_cal_points`) but is ignored while `level_model` exists.

Calibration routine — **quiescent three-point** (pump OFF, valve ≈0, no gravity
inflow into the right tank; fill it by hand). No flow means no surface agitation,
so the probe reads a clean static head:

```bash
python3 water_control_cli.py stop                            # pump OFF + valve 0% + verifies static
# block the upper left→right link by hand, then fill the right tank to ~55 mm:
python3 water_control_cli.py calibrate --sample --label low  # refuses if pump runs / valve open
# add water to ~100 mm, ruler-measured
python3 water_control_cli.py calibrate --sample --label mid
# add water to ~145 mm, ruler-measured
python3 water_control_cli.py calibrate --sample --label high
# least-squares fit of all three, prints per-point residuals:
python3 water_control_cli.py calibrate --three-point 7542 55.3 12631 100.2 16830 145.1
python3 water_control_cli.py resume                          # back to auto
```

**Use three points, not two.** Two points solve two unknowns exactly, so the
residuals are identically zero and the fit cannot be checked — a mis-read ruler,
an unsettled surface or a genuinely non-linear segment all pass silently. Three
points leave one degree of freedom, so `--three-point` prints a residual per point
and **refuses to write if any exceeds 1.5 mm** (the quiescent noise floor is
0.4–0.5 mm), naming the offending point. `--force` overrides.

Keep the outer points ≥50 mm apart and straddling the 50–150 mm working band, and
the middle point roughly centred — a point clustered with one end adds almost no
slope information (the command warns about that). `--sample` takes 5 readings and
prints the median plus a paste-ready line; it refuses unless the pump is off and
the valve ≤1 %.

`--two-point` still works for a quick check but prints the zero-redundancy warning.

Everyday zero drift (probe re-wetted, temperature) only moves `b`, so a single
point is enough:

```bash
python3 water_control_cli.py calibrate --actual 104.0         # ruler-measured level -> fixes b
python3 water_control_cli.py calibrate --actual 61.5 --check  # verify only, do not write
```

Guards, `--three-point`: outer raw span ≥4000 counts, outer level span ≥50 mm,
mm must increase with raw, **max fit residual ≤1.5 mm** (names the offending
point), and a warning if the three points are unevenly spaced.
Guards, `--two-point`: raw pair ≥1000 counts apart, levels ≥20 mm apart.
Guards, both: >30 % slope change needs `--force`; |Δb| > 40 mm is refused outright
(that magnitude means a wet probe, a changed measurement datum, or the wrong
tank), |Δb| > 25 mm needs `--force`.
Guards, `--actual` (single point): 3-sample jitter ≤3 mm, displayed level inside
50–150 mm. `--force` bypasses the jitter, residual and slope guards.
Every write is appended to `level_cal_history` (last 20) — a `b` that keeps
walking is a probe that is degrading.

Measurement datum must stay the same between calibrations — record it (tank
bottom / probe tip / overflow edge) and always measure from there.
Re-calibrating mm does **not** change the plant model (the equilibrium line, τp,
θ) — those live on the percent scale and come from step/plateau tests, not from
the ruler.

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
| 9     | iLevelRaw (raw 4-20mA counts 4000-20000, for range calibration) |
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

#### Setting a level for a demo (silent path)
`set-mm` is the whole operation — the PLC does the ramping and anti-windup
internally, so nothing PID-related needs to be said to the operator:
```bash
python3 water_control_cli.py set-mm 60  # writes target, returns immediately
python3 water_control_cli.py read-mm    # poll until within ~3 mm of target
```
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
Manage via `python3 water_control_cli.py set-scale add/del/reset`. Missing keys fall back to
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
- `scripts/step_test.py` — valve step-response test: auto-detects the plant type
  (self-regulating FOPDT → K/τp/θ + intercept C; integrating IPDT → K/τ/d0),
  prints SIMC recommended Kp/Ki, refuses unusable fits, logs CSV. Baseline `u0`
  comes from the PLC's learned `u_eq` (converted to the current level's
  equilibrium) and is trimmed until the level is nearly stationary — a drifting
  baseline measures the wrong K. Self-guarded (aborts + restores on level guards
  [45,150]mm, sensor fault, Ctrl+C — manual mode bypasses PLC HH/LL interlocks)
