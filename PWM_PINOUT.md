# PWM Demo — Pin & Timer Mapping

Board: **ST Nucleo-F091RC** (STM32F091RCTx), PlatformIO env `nucleo_f091rc` (see `platformio.ini`).

## Clock

`internal_clock()` (`src/clock.c`) configures SYSCLK = HCLK = PCLK = **48 MHz**
(HSI/2 × PLL×12, AHB and APB prescalers both = /1).

Because the APB prescaler is 1 (not >1), the APB1 and APB2 **timer kernel clocks
are also 48 MHz** — there's no ×2 timer-clock multiplier to account for.

## Active demo: closed-loop PID motor speed control

`main()` currently builds with `MOTOR_PID_DEMO` defined (this superseded the
`TEST_UART_PWM` demo described below as the active path — that demo's code
is untouched and still selectable). Closed-loop speed control of an
AndyMark PG27 gearmotor (26.85:1, 751.8 encoder counts per output rev,
198 RPM no-load) through a Cytron MDD20A driver, channel 1. Status, PID
theory and usage are in [MOTOR_PID.md](MOTOR_PID.md); design notes are in the
comment block above `setup_motor_pwm()` in `src/main.c`. Host interface:
`motor_pid_uart.py` (target RPM, `open <pct>`, `cap <pct>`, `reset`, `stop`),
same USART5 link as the PC9 demo below.

| Signal | Pin | AF | Role |
|---|---|---|---|
| MDD20A PWM1 | PA6 | AF1 (TIM3_CH1) | 20 kHz PWM, duty = speed magnitude |
| MDD20A DIR1 | PB0 | — (plain GPIO out) | 0 = forward, 1 = reverse |
| Encoder A (yellow) | PA15 | AF2 (TIM2_CH1) | hardware quadrature decode, 5 V tolerant, no pull-up |
| Encoder B (green) | PB3 | AF2 (TIM2_CH2) | hardware quadrature decode, 5 V tolerant, no pull-up |

TIM2 runs in encoder mode (`SMS=011`), counting both edges of both channels
in hardware — no ISR needed just to count pulses. TIM14 (100 Hz) drives the
PID loop tick; speed is measured over the last 5 ticks (50 ms). There's no
current sensing: current is bounded by the MDD20A's own limiting and the
bench supply. The firmware adds a duty cap (40% at boot, up to 100%) and a
stall watchdog that force-stops the motor if it's pinned at the cap with
near-zero measured speed for 500 ms.

## Earlier demo: UART-controlled PWM on PC9

`src/main.c:539` area, now reached only if `TEST_UART_PWM` is defined
(`src/main.c:547`), which calls, in order: `uart5_init()` → `setup_tim3()` →
`pwm_uart_loop()` (loops forever). The other demo path, `TEST_TIMER3`
(`src/main.c:556`), is commented out — it just calls `setup_tim3()` and idles,
for probing CH1–CH4 on a scope with no UART involved.

Host-side control: `pwm_uart.py` opens `/dev/ttyUSB0` at 115200 baud, prompts
for a duty cycle (0–100 %), sends it as a text line, and prints the firmware's
reply (`OK PC9 duty XX.X%`).

### USART5 — host UART link

| Signal | Pin | AF | Direction |
|---|---|---|---|
| USART5_TX | PC12 | AF2 | STM32 → host |
| USART5_RX | PD2 | AF2 | host → STM32 |

- Baud: 115200 8N1. `USART5->BRR = 0x1A1` → 48 MHz / 417 ≈ 115.1 kbaud.
- Set up in `uart5_init()` (`src/main.c:95`). RCC enable: `RCC_APB1ENR_USART5EN`.

### TIM3 — 4-channel general-purpose PWM

Configured in `setup_tim3()` (`src/main.c:40`). RCC enable: `RCC_APB1ENR_TIM3EN`
(APB1 timer). GPIO alternate function: **AF0** on GPIOC.

| Channel | Pin | AF | Role |
|---|---|---|---|
| TIM3_CH1 | PC6 | AF0 | fixed-duty demo output |
| TIM3_CH2 | PC7 | AF0 | fixed-duty demo output |
| TIM3_CH3 | PC8 | AF0 | fixed-duty demo output |
| **TIM3_CH4** | **PC9** | **AF0** | **duty set live from the host, via `pc9_set_duty()` (`src/main.c:132`)** |

Timer config:
- `PSC = 48000 − 1` → 48 MHz / 48000 = 1 kHz counter tick
- `ARR = 1000 − 1` → period = 1000 ticks = **1 Hz PWM frequency** (a deliberately
  slow, human-visible rate for this demo, not a practical PWM speed)
- Mode: PWM Mode 1 (`OCxM = 110`) on all 4 channels; counts up (`DIR = 0`);
  all four channel outputs enabled (`CCxE`)
- Startup duty (of `ARR+1` = 1000 counts): CH1 = 800 (80%), CH2 = 600 (60%),
  CH3 = 400 (40%), CH4 = 200 (20%, immediately overwritten by the first UART
  command). Only CH4/PC9 changes after startup — CH1–CH3 stay fixed.
- `pc9_set_duty(pct)` sets `TIM3->CCR4 = round((ARR+1) * pct / 100)` and returns
  the duty actually produced after that rounding.

## Also in this file, not part of the UART-PWM demo path

These are only reached if both `TEST_UART_PWM` and `TEST_TIMER3` are undefined
(they fall through to the Lab 4 keypad/DMA/dialer code and the `TEST_TIM1`
RGB-breathing demo). Included here for completeness since they're PWM too.

### TIM1 — RGB LED PWM + PWM-as-DAC audio channel

Configured in `setup_tim1()` (`src/main.c:181`). RCC enable:
`RCC_APB2ENR_TIM1EN` (APB2 timer). GPIO alternate function: **AF2** on GPIOA.

| Channel | Pin | AF | Role |
|---|---|---|---|
| TIM1_CH1 | PA8 | AF2 | RGB LED — red (`setrgb()`, `src/main.c:247`) |
| TIM1_CH2 | PA9 | AF2 | RGB LED — green |
| TIM1_CH3 | PA10 | AF2 | RGB LED — blue |
| TIM1_CH4 | PA11 | AF2 | crude PWM "DAC": `TIM6_DAC_IRQHandler` (`src/main.c:460`) writes synthesized audio samples directly into `TIM1->CCR4` at the 20 kHz `TIM6` ISR rate |

Timer config:
- `PSC = 0` (no division), `ARR = 2400 − 1` → **48 MHz / 2400 = 20 kHz PWM
  frequency** (high enough for the RGB LED not to visibly flicker and to work
  as a crude audio PWM-DAC)
- `TIM1->BDTR |= TIM_BDTR_MOE` is required — TIM1 is an *advanced-control*
  timer, so its channel outputs stay disabled until Main Output Enable is set,
  in addition to the per-channel `CCxE` bits. Must be set only **after** the
  RCC clock to TIM1 is enabled.
- `setrgb()` inverts the percentages before writing `CCRx` (`(100 − pct) × 24`)
  because the RGB LED on this board is active-low: 0% duty = fully on.
- Mode: PWM Mode 1 on all 4 channels; all channel outputs enabled.

### Other timers used elsewhere in `main.c` (not PWM outputs)

| Timer | Set up in | Purpose |
|---|---|---|
| TIM15 | `init_tim15()` (`src/main.c:332`) | drives the 7-segment display refresh via DMA (`DMA1_Channel5` → `GPIOB->ODR`) |
| TIM7 | `init_tim7()` (`src/main.c:372`) | 1 kHz ISR for debounced keypad column scanning |
| TIM2 | `init_tim2()` (`src/main.c:429`) | periodic ADC sampling / boxcar averaging for the "volume" knob |
| TIM6 | `init_tim6()` (`src/main.c:484`) | 20 kHz audio-sample ISR; writes samples into `TIM1->CCR4` (see above) |

## Quick reference

```
PC9  → TIM3_CH4 (AF0)  — the PWM output this demo controls live over UART
PC6  → TIM3_CH1 (AF0)  — fixed 80% duty
PC7  → TIM3_CH2 (AF0)  — fixed 60% duty
PC8  → TIM3_CH3 (AF0)  — fixed 40% duty
PC12 → USART5_TX (AF2) — to host
PD2  → USART5_RX (AF2) — from host

PA8  → TIM1_CH1 (AF2) — RGB LED red   (RGB demo, not the active main() path)
PA9  → TIM1_CH2 (AF2) — RGB LED green
PA10 → TIM1_CH3 (AF2) — RGB LED blue
PA11 → TIM1_CH4 (AF2) — audio PWM-DAC
```
