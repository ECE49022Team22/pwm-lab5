# Closed-Loop Motor Speed Control — Status & Design

STM32F091 (Nucleo-F091RC) closed-loop PID speed control of a Pololu #2828
gearmotor through a DFRobot DRI0002 H-bridge, commanded in RPM from the
Raspberry Pi over UART. Status as of 2026-09-23.

Pin/timer tables for the whole project are in [PWM_PINOUT.md](PWM_PINOUT.md).
The firmware lives in `src/main.c` (section "Closed-loop PID motor speed
control"); the host console is `motor_pid_uart.py`.

---

## 1. Current status

**In one sentence:** closed-loop speed control runs on the STM32, is commanded
in RPM from the Pi, is verified at ±5 RPM with no load, and is limited to about
±14 RPM by a 30% duty cap that keeps worst-case stall current under the
driver's 2 A rating. That current figure is an estimate; nothing measures
current.

### Verified on hardware

| What | How it was checked | Result |
|---|---|---|
| Closed-loop speed hold, forward | sent `5` | `meas` held steady at 5 RPM |
| Closed-loop speed hold, reverse | sent `-5` | `meas` held steady at −5 RPM |
| Encoder and drive agree on direction | `open 25` / `open -25` | `meas` positive / negative respectively |
| Duty cap holds | sent `15` (above what 30% can reach) | duty stopped at 30.0%, speed settled at 14.4 RPM |
| Stall fault trips | happened during a wiring fault (motor not turning) | duty climbed to 30%, `FAULT:STALL` latched ~0.5 s later, PWM cut to 0 |
| Open-loop speed at fixed duty (no load, 12 V) | `open 25`, target `15` | 25% → ~8.8–9.4 RPM; 30% → 14.4 RPM |
| Telemetry to the Pi | `motor_pid_uart.py` | ~10 lines/s: target, measured, duty, cap, raw count, flags |

The 14.4 RPM figure shows that the cap holds and how fast this motor runs
unloaded at 30% duty. It does **not** show closed-loop control at 14.4 RPM: at
that point the controller was at its limit and could not correct for anything.

### Built but not yet verified on hardware

- **Closed-loop control near the top of the range.** Send `12` and `13`. If
  `meas` holds the target with `duty` below 30%, the loop regulates up to
  there.
- **Encoder scale.** All RPM numbers are the encoder's own reading, using
  Pololu's 9600 counts per output revolution. Check it independently: at `5`
  the shaft should make 5 full turns in 60 s, and one full hand turn should
  change `cnt` by about 9600.
- **Direct reversal** (+5 → −5 without stopping at 0 first).
- **Stall fault under a real jam** (e.g. grip the shaft with a rag at `5`).
- **Any mechanical load.** All testing so far was unloaded; gains are untuned
  for load.
- **Very low speeds.** Speed is measured in steps of 0.625 RPM (see §4.2), so
  targets below ~2 RPM will be coarse.

### Known limits

- Speed range about ±14 RPM (unloaded, 12 V), set by the 30% duty cap.
- No current sensor. Current limits are estimates from the motor datasheet.
- The motor needs about 16% duty before the shaft moves at all (friction plus
  L298N losses; extrapolated from the two open-loop points above).
- 1 kHz PWM is audible as a faint whine. That is expected.

---

## 2. Hardware

| Part | Details |
|---|---|
| Controller | ST Nucleo-F091RC, STM32F091RC at 48 MHz |
| Driver | DFRobot DRI0002 ("MD1.3"), L298N, **2 A per channel**, channel 2 used |
| Motor | Pololu #2828: 12 V, 150:1 gearbox, 64 CPR encoder (**9600 counts per output rev**), no-load 67 RPM / 0.2 A, stall 5.5 A (datasheet extrapolation) |
| Motor supply | 12 V, 1 A |
| Host | Raspberry Pi 5, FTDI USB-serial adapter on `/dev/ttyUSB0` |

### Wiring

| Signal | STM32 pin | Function |
|---|---|---|
| DRI0002 **E2** (speed / enable) | **PA6** | TIM3_CH1 (AF1), 1 kHz PWM |
| DRI0002 **M2** (direction) | **PB0** | GPIO output: low = forward, high = reverse |
| Encoder A (yellow) | PA15 | TIM2_CH1 (AF2) |
| Encoder B (white) | PB3 | TIM2_CH2 (AF2) |
| UART TX to Pi | PC12 | USART5_TX (AF2), 115200 8N1 |
| UART RX from Pi | PD2 | USART5_RX (AF2) |

Also required:
- **Common ground:** Nucleo GND, DRI0002 GND, 12 V supply negative and encoder GND
  must all be connected.
- **Encoder power:** the encoder needs its own Vcc and GND, in addition to the A/B wires.
- **Logic jumper:** DRI0002 jumper shorting **VS–VD**, so the board's logic runs from
  the 12 V supply. DFRobot warns against this above 12 V.

---

## 3. Changes made

| Change | Why |
|---|---|
| New motor-control section in `src/main.c` | PWM on PA6, direction on PB0, TIM2 hardware encoder on PA15/PB3, TIM14 100 Hz PID loop, UART command loop |
| `main()` now runs `MOTOR_PID_DEMO` | Replaces the PC9 PWM demo as the default. The old demo is untouched and selectable with `TEST_UART_PWM` |
| New `motor_pid_uart.py` | Host console: sends commands, prints telemetry while you type |
| PWM lowered from 20 kHz to **1 kHz** | The L298N switches slowly, and DFRobot drives this board at Arduino's ~1 kHz |
| RPM conversion bug fixed | A unit error made measured RPM 10,000× too small; caught before first run |
| `open <pct>` test command added | Drives a fixed duty with the PID off, to tell a drive fault from an encoder fault |
| `cnt=` added to telemetry | Raw encoder count, so tiny movements are visible |
| Command parser accepts capitals and leading spaces | `Open 25` and `open 25` now both work |
| Console shows telemetry only when it changes, and keeps typed text visible | The 10 Hz stream was wiping the prompt while typing |

### Bring-up problems found and fixed

1. **Two programs on one serial port.** The GPS logger service (from earlier
   GPS work) uses the same FTDI adapter. With both running, each got half the
   bytes: garbled text and reconnect loops. Stop it before using the motor:
   `sudo systemctl stop gps-logger`. Use `sudo systemctl disable gps-logger`
   to keep it from starting at boot. Likewise, don't run `./stm.sh` with no
   arguments (which opens a serial monitor) while `motor_pid_uart.py` is open.
   Use `./stm.sh upload`.
2. **E2 and M2 wires swapped.** The PWM was going to the direction input, so
   the motor was flipped forward/back hundreds of times a second. It buzzed in
   place and averaged ~0.3 V. Found by setting E2 and M2 by hand with the
   Nucleo disconnected, then swapping one wire at a time.

---

## 4. How the code works

### 4.1 Overview

```
 Raspberry Pi                          STM32F091
┌──────────────────┐   "5\n"   ┌──────────────────────────────────────────┐
│ motor_pid_uart.py│ ────────► │ motor_uart_loop()   parses commands,      │
│  type target RPM │  USART5   │                     prints telemetry      │
│                  │ ◄──────── │        │ mc_target_rpm                    │
│  shows telemetry │ tgt=..    │        ▼                                  │
└──────────────────┘           │ TIM14_IRQHandler()  every 10 ms:          │
                               │   read encoder → RPM                      │
                               │   ramp setpoint → PID → clamp to cap      │
                               │   set PWM duty + direction                │
                               │   stall check                             │
                               │        │                  ▲               │
                               └────────┼──────────────────┼───────────────┘
                                 PA6 PWM│ PB0 dir   PA15/PB3│ encoder A/B
                                        ▼                  │
                                   DRI0002 ──► motor ──► encoder
```

Two things run on the STM32:
- **The main loop** (`motor_uart_loop()`) talks to the Pi. It reads command
  lines and prints telemetry.
- **A timer interrupt** (`TIM14_IRQHandler()`) runs the control loop at
  exactly 100 Hz, whatever the main loop is doing.

They share state through `volatile` variables (`mc_target_rpm`,
`mc_measured_rpm`, `mc_output_pct`, …). The Pi only sets the target; all
control decisions happen on the STM32, so Linux or USB timing on the Pi cannot
disturb the loop.

### 4.2 Measuring speed

TIM2 runs in hardware **quadrature encoder mode**. The encoder's A and B lines
are two square waves 90° apart. The timer counts up on every edge of either
line when the shaft turns one way, and down when it turns the other, with no
CPU involvement. `TIM2->CNT` is therefore the shaft's position in counts.

Every 10 ms the interrupt reads `CNT`, subtracts last time's value, and
converts the difference to RPM:

```
RPM = Δcounts / 9600 counts-per-rev / 0.01 s × 60 s/min
    = Δcounts × 0.625
```

So speed is measured in steps of 0.625 RPM. 5 RPM is 8 counts per tick.

### 4.3 Setpoint ramp

The Pi's target is not applied directly. Each tick, the working setpoint
(`mc_ramped_rpm`) moves toward the target by at most 0.8 RPM (80 RPM/s). This
avoids a sudden full-duty jolt at start, and it makes reversals pass smoothly
through zero.

### 4.4 The PID controller

The controller compares the speed we want (setpoint) with the speed we have
(measured) and adjusts duty to close the gap. The gap is the **error**:

```
error  = setpoint_rpm − measured_rpm
output = Kp·error + Ki·∫error dt + Kd·d(error)/dt      (in % duty)
```

| Term | What it does | Here |
|---|---|---|
| **P** (proportional) | Pushes in proportion to the current error. Big gap, big push. | `Kp = 0.6` % duty per RPM of error |
| **I** (integral) | Adds up error over time. Keeps growing while any error remains, until the error is gone. | `Ki = 3.0` % duty per RPM·s |
| **D** (derivative) | Reacts to how fast the error is changing; damps overshoot. | `Kd = 0` (off) |

**Why the integral term is essential for speed control.** Holding a speed
takes a steady, nonzero duty. The motor needs ~16% duty just to overcome
friction, and more to turn at any speed (roughly 21% for 5 RPM, estimated from
the open-loop points). With P alone, the only way to produce 21% duty is to
keep an error of 21 / 0.6 ≈ 35 RPM, so the motor could never reach 5 RPM. The
integral term builds up that steady duty over time, letting the error go to
zero while the output stays at 21%. That's why the measured speed settles
exactly on the target.

**Anti-windup.** When the output is limited, the integral would otherwise keep
growing while the motor can't respond (e.g. target 15 with a 14.4 RPM
ceiling). When the target then drops, that stored-up integral would cause a
big overshoot. The code clamps the integral term to ±cap, so it can never
exceed what the output is allowed to deliver.

### 4.5 Applying the output

- The output is clamped to ±`cap` (30% by default).
- Its **sign** sets direction (PB0), and its **magnitude** sets PWM duty
  (`TIM3->CCR1`).
- **Reversals** get one tick at zero duty before PB0 flips, so the H-bridge
  never reverses a live current.

---

## 5. Safety design (staying under stall current)

**The concern:** the motor's stall current is 5.5 A, but the DRI0002 is rated
for 2 A. There is no current sensor, so duty cycle is the only lever.

| Layer | How it works |
|---|---|
| **12 V / 1 A supply** | Physically can't deliver more than 1 A. The hard guarantee *for now*. It goes away with a stronger supply. |
| **Duty cap, 30% default** | A stalled motor is just a resistor (12 V / 5.5 A ≈ 2.2 Ω), so average current scales with duty: 30% × 5.5 A ≈ **1.65 A**, under 2 A. The real figure is lower, because the L298N drops 2–3 V. |
| **Hard maximum, 35%** | `cap` commands above 35% are clamped in firmware (≈1.9 A by the same estimate). |
| **Stall watchdog** | Output pinned at the cap **and** measured speed under 3 RPM for 500 ms → latch `FAULT:STALL`, cut PWM, zero the target. Cleared with `reset`. |
| **Setpoint ramp** | No instant full-duty starts. |
| **Reversal dead-tick** | Never reverses a live current. |

**Once you move to a stronger supply:** everything except the first row is
firmware estimation. Add a fuse or current limit on that supply rather than
relying on firmware alone.

---

## 6. Using it

### Build and flash

```
./stm.sh upload          # build + flash only (no serial monitor)
```

Turn off the 12 V while flashing: the PWM pin floats during reset.

### Run

```
sudo systemctl stop gps-logger    # if the GPS logger service is running
python3 motor_pid_uart.py
```

### Commands

| Command | Effect |
|---|---|
| `5`, `-5`, … | Target RPM, signed (+ = forward) |
| `0` or `stop` | Stop; also ends an open-loop test |
| `open 25`, `open -25` | Open-loop test: fixed duty %, PID and stall watchdog off, auto-stops after 5 s |
| `cap 25` | Set the duty cap in % (clamped to 35%) |
| `reset` | Clear a latched stall fault |
| `q` | Quit the console |

### Telemetry

Example line (values illustrative):

```
tgt=5.0 meas=5.0 duty=21.3% cap=30.0% cnt=41234
```

| Field | Meaning |
|---|---|
| `tgt` | Target RPM you sent |
| `meas` | Measured RPM from the encoder |
| `duty` | Duty the controller applied (signed) |
| `cap` | Current duty cap |
| `cnt` | Raw encoder count (position) |
| `OPEN` | Open-loop test running |
| `FAULT:STALL` | Stall fault latched; send `reset` |

### Emergency stop

Turn off the 12 V supply. `stop` only works when the loop is healthy. If the
encoder and drive ever disagree on direction, the controller drives harder
the wrong way and `stop` will not halt it.

---

## 7. Tuning procedure (for when load is added)

Gains are compile-time constants (`mc_Kp`, `mc_Ki`, `mc_Kd` in `src/main.c`).
Changing them means editing and reflashing.

1. Set `Ki = Kd = 0`. Raise `Kp` until the speed oscillates steadily around
   the target; note that value.
2. Set `Kp` to 50–60% of it.
3. Raise `Ki` until steady-state error disappears in reasonable time, without
   bringing back oscillation.
4. Add a little `Kd` only if step changes overshoot too much.
5. Retune (usually lower `Kp`) whenever the load changes.

---

## 8. Next steps

| Step | Why |
|---|---|
| Send `12`, `13` | Verify closed-loop control near the top of the range |
| Encoder scale check (timed turns, hand turn) | Verify the RPM numbers independently |
| Jam test at `5` | Verify the stall fault under a real mechanical stall |
| Speed-based duty limit | Raise the range past ~14 RPM while keeping the same ~1.65 A current ceiling. More duty is allowed as the motor speeds up, because a spinning motor's back-voltage cuts its current (§5). Falls back to 30% if the encoder fails. |
| Live `kp`/`ki`/`kd` commands | Tune under load without reflashing |
| Telemetry chart on the Pi | See target vs. measured speed over time |
