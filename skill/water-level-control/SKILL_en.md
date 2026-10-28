---
name: water-level-control
description: Use when controlling water level on Schneider PLC via Modbus TCP. Provides the water_control_cli.py script for setting target level (in mm or percent), reading actual level, calibrating the level sensor, tuning PID gains live, and monitoring PLC status.
---

# Water Level Control CLI

> English counterpart of `SKILL.md` (Chinese, the one that gets packaged).
> Keep the two in sync. This file is **not** included in the zip.

## Overview

Controls a two-tank water level rig on a Schneider PLC over Modbus TCP. The PLC
works in percent (0-100% = sensor 4-20mA); the CLI converts to **millimeters**
on top, entirely client-side.

## Running (no install / no pip)

The target device has no pip, so there is no installed command — everything runs
directly through `python3`. **`cd` into `scripts/` once first:**

```bash
cd /root/.quantum-bot/workspace/skills/water-level-control/scripts
python3 water_control_cli.py status
```

Every example here is written literally as `python3 water_control_cli.py <args>`
and can be copy-pasted as-is, assuming the working directory is the `scripts/`
directory above. From anywhere else use the absolute path:

```bash
python3 /root/.quantum-bot/workspace/skills/water-level-control/scripts/water_control_cli.py <args>
```

## Preflight: ensure the PLC is in Run state (silent)

Before running any command, silently confirm the PLC is running. This is an
internal precondition — do **not** narrate it to the user; once it passes, proceed.

```bash
/root/.quantum-bot/workspace/tools/plc_ctrl Run_State
```

`result: 'success'` means it is running. Otherwise start it and poll:

```bash
/root/.quantum-bot/workspace/tools/plc_ctrl Run \
  && /root/.quantum-bot/workspace/tools/plc_ctrl Run_State   # poll until success
```

## How the plant behaves

You need these to interpret the output correctly:

- **The valve is on the draw side**: more opening = more pumping = level falls.
  So "level too high → open the valve", and PID error = `PV − SP` (positive = too high).
- **Rising is passive**: the system can only actively *lower* the level; rising
  relies on gravity inflow from the left tank. **A rise is much slower than a
  fall** (tens of seconds) — do not declare failure prematurely.
- **Closed loop**: the pump returns right-tank water to the left tank, so total
  volume is fixed. The higher the right tank, the emptier the left tank and the
  weaker the inflow — so the opening needed to hold a given level drifts slowly
  with how the water is split. The same target may need a different opening at a
  different time. That is normal.
- **Self-regulating**: with the valve fixed, the level settles to a plateau
  rather than drifting forever.

## Supported range and refusal rule

| Range | Physical level | What to do |
|---|---|---|
| **Supported target** | **50 – 150 mm** (PLC 16.8 – 93.3%) | proceed normally |
| Sensor can measure | 31.3 – 159.1 mm | readable, but **not valid as a target** |
| < 50 mm | the pump floor (~45.3 mm) is close by | **refuse**, state the 50–150 mm range |
| > 150 mm | the high-high interlock is at 155.5 mm | **refuse**, state the 50–150 mm range |

If the user asks for something outside the range (e.g. 30 mm or 170 mm),
**refuse plainly and state the supported range — do not attempt the command.**

**Settled criterion**: level within ±3 mm of target and no longer drifting.
Note the PLC has a ±1.4 mm steady-state deadband by design, so a residual inside
±1.4 mm is normal and will not be removed.

## Standard procedure for a level change (SOP)

**This is the most common task. Three steps, and not one message in between.**

### Step 1 — reply once, *before* running anything

> Sure, setting the level to 100 mm now.

Say this **before** issuing the command. Do not wait for the result before speaking.

### Step 2 — one command does "set + wait + verdict"

```bash
python3 water_control_cli.py set-mm 100 --wait
```

`--wait` polls internally every 2 s until the level is within ±3 mm of the target
for 3 consecutive reads (exit 0), or the level has stopped moving but is still
off target (early exit, code 3), or 100 s elapse (exit 3).

⚠️ **Never write your own polling loop.** Explicitly forbidden:

- calling `read-mm` / `status` over and over to "check again"
- emitting interim messages such as "waiting for the level to settle",
  "let me poll a few more times", "not there yet, continuing to wait"
- using `trend` instead of `--wait` (`trend` is a diagnostic tool and dumps a lot
  of data)
- retrying automatically after a timeout

**Every tool call is a chance for the user to see you narrating to yourself**, and
this step needs no narration at all. The whole point of `--wait` is to fold the
waiting into **one** invocation — the SOP only works if you use it.

When calling the Bash tool, its `timeout` must be at least `--timeout + 20` s.
The default `--timeout` is 100, so pass **120000 ms**. For a large move (say
50 mm up to 150 mm) use `--timeout 170` and a Bash timeout of 200000.

### Step 3 — reply once, based on the exit code

| Exit | Command output | What to say |
|---|---|---|
| **0** | `OK 已稳定: 100.2 mm (目标 100, 偏差 +0.2mm) 用时 34s 阀 26.0%` | "The level has reached the 100 mm target (currently 100.2 mm)." |
| **3** + `到不了位` | `WARN 到不了位: 当前 96.4 mm, 距目标 -3.6mm ... [液位已停住不再变化]` | "The level stopped at 96.4 mm, 3.6 mm short of the 100 mm target, and is no longer moving — logging this for a human to check." **Do not retry, do not keep polling.** |
| **3** + `超时未稳定` | `WARN 超时未稳定: ... 漂移 +0.141 mm/s` | "The level is still moving (currently xx mm) and has not settled on the target yet." You may run `--wait` once more; if it times out twice, report as-is. |
| **3** + `正在目标附近来回穿越` | as above, but the deviation is already inside tolerance | "The level is oscillating ±x mm around the target and is not fully steady." Report the band. |

Report **the level only** — never the valve opening, the integral, the
feedforward or PID. Those are implementation details.

### Out-of-range targets

For values like 30 mm or 170 mm, **do not run the command**. Reply:

> This rig supports targets from 50 to 150 mm. 30 mm is outside that range (the
> pump floor is around 45.3 mm), so it cannot be set.

### What a correct exchange looks like

```
user : set the level to 120 mm
you  : Sure, setting the level to 120 mm now.
       [ONE Bash call: python3 water_control_cli.py set-mm 120 --wait  (timeout=120000)]
       [exit 0]
you  : The level has reached the 120 mm target (currently 120.3 mm).
```

**Two messages, one tool call.** If you find yourself writing a third message or
making a second call, the procedure went wrong.

## Standard procedure for stopping level control (SOP)

Use this for calibration, ruler measurements, probe inspection and open-loop tests
— anything that needs the water completely still. An operator has to shut the
upper link between the two tanks, so **the order cannot be changed**.

### Step 1 — bring the level to 60 mm first

```bash
python3 water_control_cli.py set-mm 60 --wait
```

⚠️ **This must happen while the upper link is still open.** The level can only
rise by gravity inflow from the left tank, so if the operator shuts the upper
valve first the level will never reach 60 mm and `--wait` will time out.
**Do not mention "close the valve" before this step.**

60 mm is the standard low point: above the pump floor (~45.3 mm) and leaving room
to fill by hand up to ~145 mm afterwards. If the task needs a different height,
substitute it — the rest of the procedure is unchanged.

### Step 2 — reply once, telling the operator you will stop in 5 s

After the level arrives, **send this message before running the command**:

> The level is at 60 mm. In 5 seconds I will run the stop command, which turns off
> the pump and closes the valve. Please close the upper inflow valve now to
> disconnect the two tanks.

### Step 3 — run stop with a countdown

```bash
python3 water_control_cli.py stop --delay 5
```

`--delay 5` counts down 5 seconds before acting, giving the operator time to shut
the valve; then it stops the pump, closes the valve, switches to manual, and runs
a drift check to confirm the water really is still.

The defaults `--settle 8 --recheck 2` mean a failed first drift check is not
immediately treated as a fault — it re-checks twice, because the operator often
acts during the countdown and the shut-off lands inside the first check window.
A Bash `timeout` of **60000 ms** is enough (worst case about 5 + 3x8 + 2x10 = 49 s).

### Step 4 — reply once, based on the exit code

| Exit | Command output | What to say |
|---|---|---|
| **0** | `✅ 系统已静止: 上下都封闭, 液位不动。可以校准/采样。` | "Stopped. The level is 60.1 mm and no longer moving, and the two tanks are disconnected — ready to calibrate." |
| **2** + level still moving | `⚠️ 液位仍在移动 —— 泵和阀都已关闭, 说明上方左右桶连通没阻断(或漏水)` | "The pump and valve are both off but the level is still changing, so the upper inflow valve may not be fully closed. Please confirm before we continue." **Do not start calibrating in this state.** |
| **2** + valve open / pump on | `⚠️ PLC 安全联锁在覆盖手动指令` | "The level is above the high-high interlock, so the PLC is forcing full pumping. It needs to come down first." |

### What a correct exchange looks like

```
user : stop the level control, I want to calibrate
you  : Sure, I will bring the level to 60 mm first.
       [Bash: set-mm 60 --wait  (timeout=120000)]   -> exit 0
you  : The level is at 60 mm. In 5 seconds I will run the stop command, which turns
       off the pump and closes the valve. Please close the upper inflow valve now
       to disconnect the two tanks.
       [Bash: stop --delay 5  (timeout=60000)]      -> exit 0
you  : Stopped. The level is 60.1 mm and no longer moving, and the two tanks are
       disconnected — ready to calibrate.
```

**Three messages, two tool calls.** Do not insert "waiting..." or "let me confirm"
messages in between.

### Afterwards

- To calibrate: `calibrate --sample --label low` / `mid` / `high`, then `--three-point`
- To resume: **`resume`, not `auto`** (`auto` restarts the integral from zero and
  the level will sail past the target for several minutes)

## Commands

```bash
# Target level — millimeters (recommended)
python3 water_control_cli.py set-mm 85 --wait                 # set absolute target 85 mm and block until settled (see SOP)
python3 water_control_cli.py raise 20 --wait                  # raise target by 20 mm (also supports --wait)
python3 water_control_cli.py lower 15                         # lower target by 15 mm
python3 water_control_cli.py read-mm                          # read ACTUAL level (mm)
python3 water_control_cli.py read                             # read target level (percent)
python3 water_control_cli.py write 65.0                       # set target level 0-100%

# Status and logging
python3 water_control_cli.py status                           # all status (percent + mm + PID)
python3 water_control_cli.py trend --duration 600 --interval 2  # log level/valve to CSV, prints min/max/band stats (Ctrl+C ends early, stats still printed)
python3 water_control_cli.py read-level-raw                   # read iLevelRaw counts + conversion (calibration)

# Mode
python3 water_control_cli.py auto                             # auto mode
python3 water_control_cli.py manual                           # manual mode
python3 water_control_cli.py stop --delay 5                   # quiesce: 5s countdown, pump OFF + valve 0% + manual, then verify the level really stops (see SOP)
python3 water_control_cli.py resume                           # leave the stopped state: preload the integral, then go auto

# Manual actuators (manual mode only)
python3 water_control_cli.py pump on|off                      # manual pump
python3 water_control_cli.py valve 30                         # manual valve opening — see Safety below

# PID (live, via Modbus)
python3 water_control_cli.py set-pid 2.0 0.26 0               # write Kp Ki Kd, auto-enables, reads back
python3 water_control_cli.py get-pid                          # read the gains the PLC is actually using
python3 water_control_cli.py get-learn                        # read the feedforward opening — see "Reading the output"

# Calibration
python3 water_control_cli.py calibrate --show                 # show the live level model and raw↔mm table
python3 water_control_cli.py calibrate --actual 104           # fix the offset from a ruler-measured level
python3 water_control_cli.py calibrate --actual 61.5 --check  # report the error only, do not write
python3 water_control_cli.py calibrate --sample --label low   # quiescent sampling (only accepted with pump OFF + valve ≈0)
python3 water_control_cli.py calibrate --three-point R1 MM1 R2 MM2 R3 MM3   # three-point refit, prints residuals

# Other
python3 water_control_cli.py set-ip 192.168.0.122             # set PLC IP
python3 water_control_cli.py read-raw 0                       # dump raw registers (debugging)
python3 step_test.py                                          # valve step test (indicative only — see "Reading the output")
```

`raise`/`lower` are incremental: read the current target, add/subtract the delta,
write the absolute value back.

## Safety

**① Check the range before `valve <opening>`: over 50–150 mm the equilibrium
opening is only 25.8 – 46.2 %.** Commanding 60 % is full-speed pumping and will
drag the tank below the pump floor (~45.3 mm) in tens of seconds, risking
cavitation. During manual experiments stay within ±10 pp of the current
equilibrium opening.

**② Manual mode overrides every interlock**, including high-high (overflow) and
low-low (dry run). `manual`, `stop`, `valve` and step tests all run in manual
mode, so **watch the level yourself**: as soon as it approaches 150 mm or 50 mm,
switch back to `auto` or run `stop`.

**③ In auto mode the valve is clamped to 12–75 % and the pump runs continuously.**
That is by design, not a fault.

## Quiescent state (`stop` / `resume`)

Quiesce before calibrating, before any ruler measurement, and before open-loop
tests. `stop` closes the bottom path (pump OFF + valve 0 % + manual); the upper
link between the tanks is **blocked by hand**.

See the **Standard procedure for stopping level control (SOP)** above for the full
flow. Options:

```bash
python3 water_control_cli.py stop --delay 5         # count down 5 s before acting (default 5, 0 = immediately)
python3 water_control_cli.py stop --settle 8        # length of each drift check (default 8 s)
python3 water_control_cli.py stop --recheck 2       # extra drift checks if one fails (default 2)
python3 water_control_cli.py stop --retry-wait 10   # wait between rechecks (default 10 s)
python3 water_control_cli.py resume                 # preload the integral, then back to auto
python3 water_control_cli.py resume --valve 30      # preload with an explicit opening (default: the PLC's u_eq)
```

`stop` reads back the valve/pump and samples the level. Three outcomes:

| Reading | Meaning | What to do |
|---|---|---|
| valve ≈0, pump OFF, drift <1 mm | ✅ static | proceed with calibration / measurement |
| valve >1 % or pump ON | the level is still above the high-high interlock | bring the level down first |
| valve ≈0, pump OFF, but drift >1 mm | the **upper link is not blocked**, or the tank/pipe leaks | go check the upper pipe; do not calibrate yet |

Exit code is 2 unless the rig verifies as static (`--force` overrides).

**`resume` is not the same as `auto`.** Plain `auto` starts the integral from
zero, leaving the valve wrong for minutes while the level sails past the target.
`resume` preloads the valve to the equilibrium opening first. Always use `resume`
to leave the stopped state.

## Level calibration

The model is one straight line `mm = K·(iLevelRaw − 4000) + b`. `K` is the
installation's inherent slope and does not drift; wetting, temperature and zero
drift only translate the line, so **routine calibration changes `b` only**:

```bash
python3 water_control_cli.py calibrate --actual 104.0         # ruler-measured → fixes b
python3 water_control_cli.py calibrate --actual 61.5 --check  # verify only, do not write
python3 water_control_cli.py calibrate --show                 # show live K/b and the table
```

Refitting `K` is only needed after replacing the probe, changing the plumbing, or
seeing the error change sign between the two ends — use the **quiescent
three-point** method:

```bash
python3 water_control_cli.py set-mm 60 --wait                # bring it to 60 mm first (upper link still open)
# follow Step 2 of the "Standard procedure for stopping level control" to warn the operator, then:
python3 water_control_cli.py stop --delay 5                  # 5 s countdown, pump OFF + valve 0%, verifies static
# the operator has now closed the upper left→right link. Fill the right tank to ~55 mm, measure with a ruler:
python3 water_control_cli.py calibrate --sample --label low
# add water to ~100 mm, measure:
python3 water_control_cli.py calibrate --sample --label mid
# add water to ~145 mm, measure:
python3 water_control_cli.py calibrate --sample --label high
# least-squares fit of all three, prints per-point residuals:
python3 water_control_cli.py calibrate --three-point 7542 55.3 12631 100.2 16830 145.1
python3 water_control_cli.py resume
```

**Use three points, never two.** Two points solve two unknowns exactly, so the
residuals are identically zero and the fit cannot be checked — a mis-read ruler,
an unsettled surface or a genuinely non-linear segment all pass silently. Three
points leave one degree of freedom, so `--three-point` prints a residual per point
and **refuses to write if any exceeds 1.5 mm** (the quiescent noise floor is
0.4–0.5 mm), naming the offending point. `--force` overrides.

Key points: outer points ≥50 mm apart and straddling the 50–150 mm band, middle
point roughly centred; the **measurement datum must stay the same** between
calibrations (tank bottom / probe tip / overflow edge — write it down and always
measure from there); `--sample` refuses unless the pump is off and the valve ≤1 %.

Other guards: a >30 % slope change needs `--force`; a |Δb| larger than 40 mm is
refused outright (that magnitude means a wet probe, a changed datum, or the wrong
tank) and larger than 25 mm needs `--force`; single-point requires 3-sample
jitter ≤3 mm and a displayed level inside 50–150 mm. Every write is appended to
`level_cal_history` (last 20) — a `b` that keeps walking means the probe is
degrading.

Recalibrating mm does **not** affect the control law (those quantities live on the
percent scale).

## PID live tuning

`set-pid` writes all three gains, sets the enable bit, then reads back what the
PLC actually reports:

```bash
python3 water_control_cli.py set-pid 2.0 0.26 0   # write + enable + read back
python3 water_control_cli.py get-pid              # confirm the gains and "tune: ON/OFF"
```

- While the gain-enable bit is clear the PLC uses its built-in defaults;
  `set-pid` sets it, and **nothing ever clears it automatically** — once set it
  stays enabled (that is the `tune: ON` shown by `get-pid`).
- The PLC clamps incoming values: Kp ≤ 20, Ki ≤ 5, Kd ≤ 10.
- **Kd has no effect** — the control law contains no derivative term.
- The effective integral gain is `Ki × 0.85`; the effective proportional gain is
  `Kp × 1.15` near the target and `Kp × 0.90` far from it.

## Reading the output

**`status`** — target and actual level (both % and mm), valve command,
auto/pump/fault/alarm bits, PID gains, `iLevelRaw`, and the calibration model
currently in effect.

**`trend`** — on exit prints level min/max/mean/band, the fitted rate, and valve
min/max/mean/swing. In steady state **the valve mean is the equilibrium opening
for the current level**, and the band is the control quality. The CSV is written
to the current directory with a timestamp in the name.

**`get-learn`** — ⚠️ some of this command's labels are **wrong**; only take half
of it:

| Output line | Usable? |
|---|---|
| `u_eq = ...` | ✅ **usable** — the feedforward opening the PLC actually used this scan (this is what `resume` preloads) |
| `C = ...` | ❌ **not usable** — it reads a leftover register the control law never uses; it is not an equilibrium intercept |
| `consistency check: u_eq + SP/2.13 = ...` | ❌ **not usable** — the constant is stale, the line means nothing |
| `valve x% vs equilibrium y% → level will rise/fall/hold` | ✅ usable, the direction is correct |

**Never report `C = ...` or that consistency-check line to the user as a measurement.**

**`step_test.py`** — treat the output as indicative only. It assumes a
single-gain first-order plant, but this rig's equilibrium curve is **piecewise**
(the middle segment is 2.5× steeper than the ends), so the K and the SIMC gains it
prints must not be applied directly. To change gains, use `trend` and observe the
actual response.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `TypeError: unexpected keyword argument 'slave'` | pymodbus 3.9+ is installed; this CLI targets 3.8.6. Downgrade, or change `slave=` to `device_id=` in the code |
| Modbus exception code 2 (illegal address) | the PLC's register arrays are too small or the mapping is missing — check in Machine Expert |
| Read-back floats look like garbage (~1e-40 or ~1e20) | byte order was flipped to high-word-first. This PLC is **low-word-first** (writing 10.0 yields `[0, 16672]`); flip `float_to_registers` back to `[low, high]` |
| `iLevelRaw` reads 0 | the PLC program does not map input register 9 |
| `calibrate --sample` refuses | the pump is running or the valve is >1 % — run `stop` first |
| Level does not move after setting a target | rising relies on gravity inflow and is genuinely slow; use `trend` to check whether the valve is near 12 % (it is already pumping as little as it can) |

Config lives in `~/.water-control/config.json` (`plc_ip` / `plc_port` /
`slave_id` / `level_model` / `level_cal_history`). Missing keys fall back to
defaults, so old config files still work. **Use `calibrate --show` for the values
actually in effect** — do not quote the repo's shipped defaults.

## Agent interaction rules

- **Never mention PID parameter adjustment to the end user during demos** — only
  report the final water level result.
- Run the preflight (`plc_ctrl Run_State`) silently; do not narrate it.
- Refuse targets outside 50–150 mm plainly and state the supported range; do not
  attempt the command.
- Report levels in **mm** first, with percent in parentheses when useful.
- **Always use the SOP for a level change**: two messages, one tool call. If you
  catch yourself writing "let me check again" or "still waiting", the procedure
  went wrong.
- **Always use the stop SOP to quiesce the rig**: three messages, two tool calls.
  The level must reach 60 mm *before* the operator is asked to close the upper
  valve — the other way round and the level can never get there.

#### Setting a level for a demo
Follow the **Standard procedure for a level change (SOP)** above: one reply, one
`set-mm <value> --wait` call, one concluding reply. Do not poll, do not emit
interim messages, do not explain any control detail.
