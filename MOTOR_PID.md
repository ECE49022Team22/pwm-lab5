# Closed-Loop Motor Speed Control — Status & Design

STM32F091 (Nucleo-F091RC) closed-loop PID speed control of an AndyMark PG27
gearmotor through a Cytron MDD20A driver, commanded in RPM from the Raspberry
Pi over UART. Status as of 2026-10-19, branch `mdd20a-pg27`.

Pin/timer tables for the whole project are in [PWM_PINOUT.md](PWM_PINOUT.md).
The firmware lives in `src/main.c` (section "Closed-loop PID motor speed
control"); the host console is `motor_pid_uart.py`.

---

## 0a. Calibration with the wheels on (2026-11-04)

Recalibrated after the motors were re-plugged and fitted with wheels on the chassis (wheels off the
ground, battery power). Both motors and encoders checked first: each motor moves only its own
encoder, and + duty gives + measured speed on both. On the chassis, **motor 1 = right wheel and its +
direction rolls the robot backward; motor 2 = left wheel, + rolls it forward** (the demo's
`--left-motor 2 --left-sign 1 --right-sign -1` defaults; left/right confirmed driving on the floor, where
speeding up motor 1 turned the robot left).

**Floor driving on tile (same day).** The chassis is skid-steer: two driven 8 in wheels plus two unpowered
ones that slide sideways in a turn. What we learned (`testing/motor_cal/` (also `~/motor_cal` on the Pi), scripts `robot_drive.py` and `turnmap.py`):

- **Effective track ~0.95 m** (0.88-1.03 m across inner/outer ratios 0.5 to -0.25), against 50 cm between the
  wheel centres. A kinematic model with that track predicts the turn well; the gyro trims the rest.
- **Spinning in place stalls** (both motors at 100% duty, stall fault); inner/outer ratio -0.5 stalls too.
  Tightest clean turn: ratio 0 to -0.25, ~11-12 deg/s, 0.5-0.7 m radius.
- **Never command a wheel exactly 0 while turning.** The firmware's "target 0 = off" rule then switches the
  motor off and the inner wheel freewheels (ratio 0 turned 1.3 deg/s that way, 11.7 deg/s with 0.05 RPM).
  This is also why the first gyro-corrected circle collapsed.
- **On the floor the wheels need 2-3 s to force a turn**: the outer wheel needs 60-90% duty to drag the
  chassis round and the inner one gets pulled along while braking. Feedforward (`kf`/`ks`, new, 0 at boot)
  made little difference: it's torque, not controller lag.
- **Wheel lift:** the rigid 4-wheel chassis can rock onto a diagonal; a lifted driven wheel ends the turn.
  Weight over the driven axle or casters would help; `robot_drive.py circle` detects it (wheel well above
  its command) and recovers with a short slow straight roll.
- **2 m circle achieved:** `robot_drive.py circle --diameter 2 --side right --speed 0.2` (track 0.95 m
  feedforward + gyro yaw-rate PI, stops at 360 deg by the gyro): 360 deg in 36 s, radius 1.04 m from wheels
  and gyro (target 1.00), LiDAR guard at 0.5 m. Looked right to the team.
- **Left turns are harder than right turns.** In a left circle the right driven wheel is the outer one and
  it lifts (the chassis rocks off it, even though the robot is heavier on the right). With the plain
  controller the left circle stalled at 128 deg (6 lift/stuck events). With traction control (`--recover-mode
  ease`, the default: on a lift, halve the speed difference and ramp it back over 3 s; outer wheel capped at
  ~10% over the model) it completed 360 deg with one recovery, but slowly: 63 s, ~6 deg/s, radius ~1.5 m by
  wheel odometry (wheel slip inflates that). Weight over the right driven wheel, or casters/suspension on
  the unpowered pair, should fix the asymmetry.

Open-loop speed (RPM) vs duty, both wheels:

| Duty | 10% | 20% | 30% | 40% | 50% | 60% | 70% | 80% | 90% | 100% |
|---|---|---|---|---|---|---|---|---|---|---|
| Motor 1 fwd | 9.5 | 31.5 | 53.7 | 74.3 | 96.7 | 117.5 | 137.8 | 158.1 | 177.6 | 198.1 |
| Motor 1 rev | −12.7 | −34.4 | −56.3 | −77.6 | −98.6 | −118.9 | −138.8 | −158.9 | −178.6 | −199.3 |
| Motor 2 fwd | 10.5 | 31.3 | 53.5 | 73.6 | 95.0 | 114.6 | 136.3 | 155.5 | 174.3 | 194.5 |
| Motor 2 rev | −11.6 | −32.7 | −54.5 | −75.8 | −96.9 | −117.3 | −137.5 | −157.2 | −176.6 | −196.7 |

~2.1 RPM per % duty, top speed ~195-199 RPM (was 214 bare on the bench supply). `motor_autotune.py`
(now with `--motor 2`) per motor: motor 1 2.24 RPM/%, 18 ms delay, 50 ms time constant -> **Kp 0.104,
Ki 2.055**; motor 2 2.18 RPM/%, 22 ms, 43 ms -> **Kp 0.089, Ki 2.062**. These are the boot gains
(`MOTOR1_BOOT_KP` etc. in `src/main.c`). Verified as robot moves (forward 60/120/160, back 60, spin
left/right 40): overshoot <=3% (8.5% of a 20 RPM step once), steady error <=0.2 RPM, jitter <=1.1 RPM.
Keep targets <= ~175 RPM so the PID has duty headroom. Raw logs: `testing/motor_cal/` (also `~/motor_cal` on the Pi).

## 0. Two motors (2026-10-27)

The firmware now runs a second PG27 on MDD20A channel 2 (PWM2 = PA7 / TIM3_CH2, DIR2 = PB2,
encoder PA8/PA9 on TIM1; see [PWM_PINOUT.md](PWM_PINOUT.md)). Both motors share the 100 Hz PID tick;
each has its own gains, cap, target, open-loop test and stall fault.

**Protocol.** Unprefixed commands act on motor 1 as before, except `stop` and `reset`, which now act
on both. `m1 <cmd>` / `m2 <cmd>` send any command to one motor (`m2 40`, `m2 open 20`, `m2 kp 0.07`;
replies are the same text as motor 1's). `d <rpm1> <rpm2>` sets both targets on the same tick
(reply `OK target <rpm1> <rpm2> RPM`). Telemetry keeps motor 1's fields first and unchanged, then
adds ` tgt2= meas2= duty2=% cap2=% cnt2=`; motor 2's flags are lowercase (` open2`, ` stall2`) so a
search for `OPEN` / `FAULT:STALL` still means motor 1. `motor_live.py` and `motor_autotune.py` still
work on motor 1 unchanged.

**Motor 2 calibration.** Encoder direction matches with `MOTOR2_ENC_INVERT 1` (same as motor 1), and
the open-loop curve matches motor 1's within ~2-3%, so it boots with the same gains (Kp 0.066 /
Ki 1.97):

| Duty | Motor 2 forward | Motor 2 reverse | Motor 1 forward (below) |
|---|---|---|---|
| 10% | 13.2 | −15.0 | 12.7 |
| 20% | 35.9 | −37.4 | 34.9 |
| 30% | 58.6 | −59.8 | 56.8 |
| 50% | 104.0 | −104.7 | 101.2 |
| 70% | 148.6 | −149.3 | 145.6 |
| 90% | 194.4 | −194.1 | 189.8 |

Closed-loop steps on motor 2 with those gains (0→30→60→120→−60→0 RPM): overshoot ≤2.7%, settle
≤1.3 s (after the 200 RPM/s ramp), steady error ≤0.1 RPM, jitter ≤1 RPM. Both motors driven together
with `d` (60/60, 60/−60, −40/80): overshoot ≤1.5%, steady error ≤0.4 RPM on each.

A copy of the single-motor firmware's flash, as it was before this change, is in
`~/stm32_backups/stm32_flash_backup_before_2motor.bin` (restore: `openocd -f openocd.cfg -c
"program <file> 0x08000000 verify reset exit"`).


**Running and calibrated across the full speed range (unloaded, 12 V).**
Boot gains Kp 0.066 / Ki 1.97 came from `motor_autotune.py` and were
re-confirmed by a full-range run (a model fitted at 30%/70% duty gave
0.068 / 1.99).

### Open-loop speed vs duty

| Duty | Forward RPM | Reverse RPM |
|---|---|---|
| 10% | 12.7 | −14.2 |
| 20% | 34.9 | −36.2 |
| 30% | 56.8 | −58.3 |
| 40% | 78.9 | −80.5 |
| 50% | 101.2 | −102.6 |
| 60% | 123.4 | −124.6 |
| 70% | 145.6 | −146.6 |
| 80% | 168.1 | −168.7 |
| 90% | 189.8 | −191.0 |
| **100%** | **214.0** | **−215.1** |

- **Max speed 214 RPM forward, 215 reverse** (AndyMark spec: 198).
- Linear at **~2.21 RPM per % duty** from 10% to 90%, above a ~3–4% duty
  deadband. Forward and reverse within ~1.5 RPM of each other.
- The ~5 A supply limit didn't visibly flatten the curve.
- Response: ~25–30 ms delay (includes the 50 ms speed window), ~34 ms time
  constant, the same at 15%, 30% and 70% duty.

### Closed-loop verification (Kp 0.066, Ki 1.97, cap 100%)

| Step | Overshoot | Settle (after the 200 RPM/s ramp) | Steady error | Jitter (sd) | Duty |
|---|---|---|---|---|---|
| 0 → 10 | 1.2 RPM (< one count) | 0.35 s | 0 | 0.7 RPM | 7.9% |
| 10 → 60 | 1.4% | 0.39 s | 0 | 0.8 RPM | 30.4% |
| 60 → 120 | 2.2% | 0.30 s | −0.1 | 0.6 RPM | 57.3% |
| 120 → 190 | 2.1% | 0.23 s | +0.1 | 0.6 RPM | 88.5% |
| 190 → −190 | 0.4% | 0.28 s | +0.1 | 0.4 RPM | −89.0% |
| −190 → −60 | 0.7% | 0.51 s | 0 | 0.8 RPM | −30.4% |
| −60 → 0 | 0 | 0.06 s | 0 | 0 | 0 |

So the loop regulates from 10 RPM to 190 RPM in both directions. 190 RPM
needs ~89% duty; above ~200 RPM there's no duty headroom left to correct
with, so treat **~190–200 RPM as the closed-loop maximum**.

**Encoder scale verified (2026-10-19).** The motor was driven at 10 RPM and
stopped by count after 5 × 751.8 = 3,759 counts (it stopped at 3,761). A tape
flag on the output shaft made exactly 5 turns and ended lined up with its
reference mark. A 0.3% error in counts per revolution would have left it
~5° off, so the RPM figures are confirmed against the shaft itself.

Not yet checked:
- **Behavior under load**: rerun `motor_autotune.py` once a load is attached.
- The duty cap boots at 100% (full range). Lower it with `cap` for gentler bench work.

---

## 2. Hardware

| Part | Details |
|---|---|
| Controller | ST Nucleo-F091RC, STM32F091RC at 48 MHz |
| Driver | Cytron MDD20A, dual 20 A, 6–30 V, **channel 1 used**. Sign-magnitude (PWM + DIR), 3.3 V logic, PWM up to 20 kHz. Has its own current limiting. **No reverse-polarity protection.** |
| Motor | AndyMark PG27 (am-3651), RS775-125, 12 V. Gear ratio 3249/121 = **26.85:1**. No-load **198 RPM** at the output. Stall torque 6.3 ft·lbf. |
| Encoder | Hall, rear-mounted, 7 pulses per motor rev per channel, 5 V. ×4 decoding → **751.8 counts per output rev**. Rear mount counts opposite to a front-mounted encoder. |
| Supply | 12 V bench supply, current limit ~5 A |
| Host | Raspberry Pi 5, FTDI USB-serial adapter on `/dev/ttyUSB0` |

### Driver truth table (per channel)

| PWM | DIR | M1A | M1B | Motor |
|---|---|---|---|---|
| low | x | low | low | brake |
| high | low | high | low | forward |
| high | high | low | high | reverse |

So during the off part of each PWM cycle the motor is braked (shorted), not
coasting. Speed tracks duty more linearly than on the old L298N.

### Wiring

| From | To | Nucleo header |
|---|---|---|
| MDD20A PWM1 | PA6 (TIM3_CH1, AF1) | D12 |
| MDD20A DIR1 | PB0 (GPIO out, low = forward) | A3 |
| MDD20A signal GND | Nucleo GND | |
| Encoder red (pin 1, +5 V) | Nucleo 5V | CN6 |
| Encoder black (pin 2, GND) | Nucleo GND | |
| Encoder yellow (pin 3, ch A) | PA15 (TIM2_CH1, AF2, 5 V tolerant) | CN7-17 |
| Encoder green (pin 4, ch B) | PB3 (TIM2_CH2, AF2, 5 V tolerant) | D3 |
| Motor power red (+) | MDD20A M1A | |
| Motor power black (−) | MDD20A M1B | |
| 12 V supply + / − | MDD20A VB+ / VB− | |

Host link: USART5, PC12 TX / PD2 RX, 115200 8N1. All grounds (Nucleo, MDD20A,
supply) are common.

---

## 3. Firmware changes for the new hardware

| Setting | Old (Pololu + DRI0002) | New | Why |
|---|---|---|---|
| `MOTOR_COUNTS_PER_REV` | 9600 | **751.8** | 7 × 4 × 26.85 |
| Speed measurement | count change over 1 tick (10 ms) | **over the last 5 ticks (50 ms), updated every tick** | At 751.8 counts/rev one count per 10 ms is ~8 RPM. Over 50 ms it's ~1.6 RPM. |
| PWM | 1 kHz (`MOTOR_PWM_PERIOD` 48000) | **20 kHz** (2400) | The MDD20A handles 20 kHz; inaudible. The 1 kHz was for the slow L298N. |
| Duty cap boot default | 30% | **100%** | Full speed range. Was 40% for first bring-up; stall current isn't a driver risk (§5). Adjustable with `cap`. |
| `MOTOR_DUTY_HARD_MAX` | 35% | **100%** | Old value came from the DRI0002's 2 A rating. The MDD20A limits its own current. |
| Gains Kp / Ki | 0.6 / 3.0 | **0.066 / 1.97** | Measured and tuned by `motor_autotune.py` (§1). Started from 0.2 / 1.0. |
| Ramp | 80 RPM/s | **200 RPM/s** | Matches the larger speed range |
| Encoder pull-ups | always on | **off** (`MOTOR_ENC_PULLUPS` 0) | PA15/PB3 tolerate 5 V only with pull-ups off. Set 1 only for an open-collector encoder. |
| Encoder direction | — | **`MOTOR_ENC_INVERT` 1** | The rear-mounted encoder counted backwards (`open 20` read −34 RPM); inverts TI1 (TIM2 CC1P) |
| Target 0 | integral kept | **output 0, integral cleared once the ramp reaches 0** | The integral built up while slowing held a few % duty into the stopped motor (seen: −4.6% after −30 → 0) |
| New commands | — | **`kp`/`ki`/`kd <v>`, `rate <hz>`** | Live gains (Ki change keeps the integral term continuous); 1–50 Hz telemetry with ` t=<ms>` off the default 10 Hz |

Unchanged: existing commands, telemetry format at the default rate, stall watchdog, reversal dead tick,
open-loop test, `motor_pid_uart.py`, and the DMA UART from `latency-benchmark`.

---

## 4. How the code works

### 4.1 Overview

```
 Raspberry Pi                          STM32F091
┌──────────────────┐   "30\n"  ┌──────────────────────────────────────────┐
│ motor_pid_uart.py│ ────────► │ motor_uart_loop()   parses commands,      │
│  type target RPM │  USART5   │                     queues telemetry      │
│                  │ ◄──────── │        │ mc_target_rpm   (RX/TX on DMA)   │
│  shows telemetry │ tgt=..    │        ▼                                  │
└──────────────────┘           │ TIM14_IRQHandler()  every 10 ms:          │
                               │   encoder → RPM over last 50 ms           │
                               │   ramp setpoint → PID → clamp to cap      │
                               │   set PWM duty + direction                │
                               │   stall check                             │
                               │        │                  ▲               │
                               └────────┼──────────────────┼───────────────┘
                               PA6 PWM1 │ PB0 DIR1  PA15/PB3│ encoder A/B
                                        ▼                  │
                                    MDD20A ──► PG27 ──► encoder
```

Two things run on the STM32:
- **The main loop** (`motor_uart_loop()`) talks to the Pi. USART5 runs on DMA,
  so a command arriving while telemetry is printing isn't lost.
- **A timer interrupt** (`TIM14_IRQHandler()`) runs the control loop at
  exactly 100 Hz, whatever the main loop is doing.

The Pi only sets the target; all control decisions happen on the STM32, so
Linux or USB timing on the Pi cannot disturb the loop.

### 4.2 Measuring speed

TIM2 runs in hardware **quadrature encoder mode**. The encoder's A and B lines
are two square waves 90° apart; the timer counts up on every edge of either
line in one direction and down in the other, with no CPU involvement.
`TIM2->CNT` is the shaft position in counts.

Every 10 ms the interrupt stores `CNT` in a 5-slot ring buffer and compares it
with the value from 5 ticks ago:

```
RPM = Δcounts(last 5 ticks) / 751.8 counts-per-rev / 0.05 s × 60 s/min
    = Δcounts × 1.596
```

So speed still updates every 10 ms but is measured in steps of ~1.6 RPM. The
cost is lag: the reading reflects the average over the last 50 ms (~25 ms
behind on average). That's small next to the motor's own response. At
198 RPM the encoder produces ~2,500 counts/s, ~124 per window.

### 4.3 Setpoint ramp

Each tick, the working setpoint (`mc_ramped_rpm`) moves toward the Pi's
target by at most 2 RPM (200 RPM/s). No sudden full-duty jolt at start;
reversals pass smoothly through zero.

### 4.4 The PID controller

```
error  = setpoint_rpm − measured_rpm
output = Kp·error + Ki·∫error dt + Kd·d(error)/dt      (in % duty)
```

| Term | What it does | Now |
|---|---|---|
| **P** | Pushes in proportion to the current error | `Kp = 0.066` % duty per RPM |
| **I** | Accumulates error over time until it's gone | `Ki = 1.97` % duty per RPM·s |
| **D** | Reacts to how fast the error changes | `Kd = 0` (off) |

**Why the integral term matters.** Holding a speed takes a steady duty:
17.7% for 30 RPM on this motor (measured). P alone could only produce that by
keeping a 17.7 / 0.066 ≈ 270 RPM error. The integral builds the steady duty
up so the error can go to zero. With this fast motor (~34 ms time constant)
the tuned controller is mostly integral: Ki / Kp ≈ 30 per second.

**Anti-windup.** The integral term is clamped to ±cap, so when the output is
limited it can't store up a correction the motor can't use and then overshoot
once the limit lifts.

### 4.5 Applying the output

- Clamped to ±`cap` (100% at boot).
- **Sign** → DIR1 (PB0). **Magnitude** → PWM1 duty (`TIM3->CCR1`, 0–2400).
- **Reversals** get one 10 ms tick at zero duty before DIR1 flips. On the
  MDD20A zero duty means brake.

---

## 5. Safety design

**What changed.** On the old setup the motor's resistance was high (~2.2 Ω)
and the driver was rated 2 A, so the duty cap doubled as a stall-current limit.
Estimated from AndyMark's numbers (198 RPM no-load, 6.3 ft·lbf stall, 26.85:1,
gearbox assumed lossless), the RS775-125 has about **0.7 Ω** and a **stall
current of roughly 15–20 A at 12 V**. These are estimates, not datasheet values.
A full stall at 12 V (~15–20 A) is within the MDD20A's 20 A rating, and the
bench supply's ~5 A limit caps it first. So stall current isn't a risk
to the driver; the supply limits it first. There is still no current sensor.

| Layer | Role |
|---|---|
| **Bench supply limit (~5 A)** | Hard limit on what the system can draw. Goes away with a bigger supply. |
| **MDD20A current limiting** | The driver limits its own output current. |
| **Stall watchdog** | Output pinned at the cap **and** speed under 3 RPM for 500 ms → latch `FAULT:STALL`, cut PWM, zero the target. Cleared with `reset`. In simulation a jam at 30 RPM tripped it ~1.2 s after the jam. |
| **Duty cap, 100% at boot** | Lower it with `cap` to limit speed and torque (~2.2 RPM per % duty). Not a current limit. |
| **Setpoint ramp** | No instant full-duty starts. |
| **Reversal dead tick** | Never flips direction with drive applied. |

Watch items:
- **Reverse polarity.** The MDD20A has no protection on VB+/VB−. Check supply
  polarity before every power-up after rewiring.
- **Energy back into the supply.** When the motor slows faster than friction
  alone would stop it, sign-magnitude drive returns some energy to the supply,
  and bench supplies can't absorb much. The 200 RPM/s ramp keeps this small
  on an unloaded gearmotor; watch the supply voltage on fast stops if you
  raise the ramp or add inertia.

---

## 6. Using it

### Build and flash

```
./stm.sh upload          # build + flash only (no serial monitor)
```

**12 V off while flashing**: the PWM pin floats while the chip resets.

### Run

```
python3 motor_pid_uart.py
```

Only one program may hold `/dev/ttyUSB0`: not `./stm.sh` without arguments,
not the GPS logger service (`sudo systemctl stop gps-logger`).

### Live graph (best for demos)

```
python3 motor_live.py        # then open http://127.0.0.1:8090 in the Pi's browser
```

Holds the serial port itself (close `motor_pid_uart.py` first), switches
telemetry to 50 Hz, and serves a page with live charts of target vs.
measured speed and of duty, a large measured-speed readout, buttons for
common targets, Stop, a 15 s open-loop 30% test, and a box for any command.
Hover a chart (or focus it and use the arrow keys) for a readout at that
moment; **Pause** freezes the charts; **Table** shows recent samples.
`?theme=dark` / `?theme=light` forces a theme. Ctrl-C sends `stop` and
puts telemetry back to 10 Hz.

### Commands

| Command | Effect |
|---|---|
| `30`, `-30`, … | Target RPM, signed (+ = forward) |
| `0` or `stop` | Stop; also ends an open-loop test |
| `open 20`, `open -20` | Open-loop test: fixed duty %, PID and stall watchdog off, auto-stops after 15 s (send it again to extend) |
| `cap 60` | Set the duty cap in % (0–100) |
| `kp 0.07`, `ki 2`, `kd 0` | Set a gain live; reply lists all three. Lost on reset. |
| `rate 50` | Telemetry 1–50 Hz; adds ` t=<ms>` when not 10 Hz |
| `reset` | Clear a latched stall fault |
| `q` | Quit the console |

### Telemetry

```
tgt=30.0 meas=30.3 duty=15.4% cap=40.0% cnt=8123     (values illustrative)
```

| Field | Meaning |
|---|---|
| `tgt` | Target RPM |
| `meas` | Measured RPM (50 ms window) |
| `duty` | Applied duty, signed |
| `cap` | Duty cap |
| `cnt` | Raw encoder count (position) |
| `OPEN` | Open-loop test running |
| `FAULT:STALL` | Stall fault latched; send `reset` |
| `ORE=n` | UART receive overruns counted (should never appear) |
| `t` | Device time in ms (only when `rate` isn't 10 Hz) |

### Emergency stop

Turn off the 12 V supply. `stop` only works when the loop is healthy. If the
encoder and the drive disagree on direction, the controller drives harder the
wrong way and `stop` won't halt it. That's what the `open 20` sign check in
bring-up is for.

---

## 7. Bring-up checklist (new hardware)

1. **12 V off.** Flash: `./stm.sh upload`. Start `motor_pid_uart.py`; telemetry
   should stream with `meas=0.0`.
2. **Supply polarity**: VB+ to supply +, VB− to supply −. Then 12 V on.
3. **`open 20`**: motor turns (up to 15 s) and `meas` should be **positive**
   (expect very roughly 30–40 RPM).
   - **Negative**: set `MOTOR_ENC_INVERT 1`, 12 V off, reflash, repeat.
   - **Stays 0 while turning**: encoder wiring/power (red 5 V, black GND,
     yellow PA15, green PB3).
   - **Doesn't turn**: PWM1/DIR1/signal GND, VB+/VB−, M1A/M1B.
   - Also try `open -20`: `meas` should be negative.
4. **Encoder scale**, 12 V off: turn the output shaft exactly one revolution
   by hand. `cnt` should change by about **752**.
5. **Closed loop**: `30`, `60`, `-30`, `0`. `meas` should settle on each
   target with `duty` below the cap.
6. **Tune** (§8).

---

## 8. Tuning

**Automatic:** with 12 V on, motor clamped and nothing else holding the port:

```
python3 motor_autotune.py
```

1. Direction check: `open ±20` must read ± speed. Aborts otherwise (e.g. set
   `MOTOR_ENC_INVERT`).
2. Open-loop duty steps (15%, 30%) fitted to a first-order-plus-delay model.
3. SIMC PI gains from the model (`--tau-c`, default 0.2 s, sets how fast).
4. Closed-loop steps 30 → 60 → −30 → 0 RPM, scored; gains scaled ×0.7 if a
   step overshoots >10% (ignoring overshoot under ~one count), ×1.3 if it
   settles more than 1.0 s after the setpoint ramp ends, up to 3 rounds.

It aborts on a stall fault, speed far above target, or the wrong direction,
by sending `cap 0` then `stop` (leaving the cap at 0). Gains it finds are
live only; it prints the lines to put in `src/main.c`. Every telemetry line
is logged to a CSV (`--log`).

**Manual:** set gains live with `kp`/`ki`/`kd`. Start with `ki 0`, raise `kp`
until the speed oscillates, back it off to 50–60%, then raise `ki` until the
steady-state error goes away without bringing back oscillation.

---

## 9. History and bring-up problems

The first version ran a Pololu #2828 (150:1, 9600 counts/rev, 67 RPM no-load)
through a DFRobot DRI0002 (L298N, 2 A/channel), with a 30% duty cap chosen to
keep stall current under the driver's rating. Verified there: ±5 RPM closed
loop, duty cap holding (14.4 RPM ceiling at 30%), encoder/drive direction
agreement, stall fault tripping. Problems found during that bring-up, still
worth knowing:

1. **Two programs on one serial port** (the GPS logger service and the motor
   console) garbled everything. Only one program may hold `/dev/ttyUSB0`.
2. **PWM and direction wires swapped**: the motor buzzed in place and didn't
   turn. Found by setting the driver inputs by hand with the Nucleo
   disconnected.
3. **Driver lost power**: the DRI0002's power LED went out, and the motor and
   the encoder stopped working together. The hardware was replaced afterwards.

On the new hardware:

4. **Encoder counted backwards**: fixed with `MOTOR_ENC_INVERT 1`.
5. **Reverse worked only sometimes**: the MDD20A's GND wasn't connected to the
   Nucleo's GND, so DIR1's high level had no reliable reference. Forward
   (DIR1 low) happened to work. Fixed by adding the ground wire.
