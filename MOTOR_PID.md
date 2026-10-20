# Closed-Loop Motor Speed Control — Status & Design

STM32F091 (Nucleo-F091RC) closed-loop PID speed control of an AndyMark PG27
gearmotor through a Cytron MDD20A driver, commanded in RPM from the Raspberry
Pi over UART. Status as of 2026-10-19, branch `mdd20a-pg27`.

Pin/timer tables for the whole project are in [PWM_PINOUT.md](PWM_PINOUT.md).
The firmware lives in `src/main.c` (section "Closed-loop PID motor speed
control"); the host console is `motor_pid_uart.py`.

---

## 1. Current status

**Running and tuned on the new hardware (unloaded).** `motor_autotune.py`
measured the motor and picked the gains, which are now the boot defaults.

Measured (open-loop duty steps, no load, 12 V):

| Quantity | Value |
|---|---|
| Speed vs duty | 2.21 RPM per % duty above ~4.1% duty |
| Delay | ~30 ms (includes the 50 ms speed window) |
| Time constant | ~34 ms |
| ±20% duty | +34.4 / −35.1 RPM (symmetric) |

Closed-loop verification with Kp 0.066, Ki 1.97:

| Step | Overshoot | Settle (±5% + 2 RPM) | Steady error | Jitter (sd) | Duty |
|---|---|---|---|---|---|
| 0 → 30 RPM | 1.0% | 0.68 s | −0.1 RPM | 0.7 RPM | 17.7% |
| 30 → 60 RPM | 2.3% | 0.48 s | −0.1 RPM | 0.8 RPM | 31.4% |
| 60 → −30 RPM | 0.3% | 1.10 s | −0.1 RPM | 0.5 RPM | −17.5% |
| −30 → 0 RPM | 0% | 0.66 s | 0 | 0 | — |

Not yet checked:
- **Encoder scale by hand**: turn the output shaft exactly one revolution
  with the 12 V off; `cnt` should change by about 752. All RPM figures above
  rely on the 751.8 counts/rev from the spec.
- **Behavior under load**: rerun `motor_autotune.py` once a load is attached.
- With the 40% cap, top speed is about (40 − 4.1) × 2.21 ≈ 79 RPM.

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
| Duty cap boot default | 30% | **40%** | Bench-safety limit, adjustable with `cap` |
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

- Clamped to ±`cap` (40% at boot).
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
A stall at the 40% boot cap would draw ~7 A, more than the bench supply's ~5 A
limit and well within the MDD20A's 20 A rating. So stall current isn't a risk
to the driver; the supply limits it first. There is still no current sensor.

| Layer | Role |
|---|---|
| **Bench supply limit (~5 A)** | Hard limit on what the system can draw. Goes away with a bigger supply. |
| **MDD20A current limiting** | The driver limits its own output current. |
| **Stall watchdog** | Output pinned at the cap **and** speed under 3 RPM for 500 ms → latch `FAULT:STALL`, cut PWM, zero the target. Cleared with `reset`. In simulation a jam at 30 RPM tripped it ~1.2 s after the jam. |
| **Duty cap, 40% at boot** | Limits speed and torque for bench work (~80 RPM no-load at 40%). Not a current limit. |
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

### Commands

| Command | Effect |
|---|---|
| `30`, `-30`, … | Target RPM, signed (+ = forward) |
| `0` or `stop` | Stop; also ends an open-loop test |
| `open 20`, `open -20` | Open-loop test: fixed duty %, PID and stall watchdog off, auto-stops after 5 s |
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
3. **`open 20`**: motor turns for 5 s and `meas` should be **positive**
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
   step overshoots >10%, ×1.3 if settling is slower than 1.2 s, up to 3 rounds.

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
