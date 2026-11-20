/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Jan 31 2024
  * @brief   ECE 362 Lab 5 Student template
  ******************************************************************************
*/

/*******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "jyasuumi";

/*******************************************************************************/ 

#include "stm32f0xx.h"
#include <math.h>   // for M_PI
#include <stdio.h>  // snprintf
#include <stdlib.h> // strtod
#include <string.h> // strncmp

void nano_wait(int);

// 16-bits per digit.
// The most significant 8 bits are the digit number.
// The least significant 8 bits are the segments to illuminate.
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];
// Print an 8-character string on the 8 digits
void print(const char str[]);
// Print a floating-point value.
void printfloat(float f);

void autotest(void);

//============================================================================
// PWM Lab Functions
//============================================================================
void setup_tim3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable TIM3 clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // enable GPIOC clock

    GPIOC->MODER &= ~0xFF000;
    GPIOC->MODER |=  0xAA000; // configure PC6-9 as alternate function (10) 

    // configure GPIO to select AF0 (0x0)
    GPIOC->AFR[0] &= ~GPIO_AFRL_AFRL6;
    GPIOC->AFR[0] &= ~GPIO_AFRL_AFRL7;
    GPIOC->AFR[1] &= ~GPIO_AFRH_AFRH0;
    GPIOC->AFR[1] &= ~GPIO_AFRH_AFRH1;


    TIM3->PSC = 48000 - 1; // 48 MHZ to 1KHz
    TIM3->ARR = 1000 - 1; // 1KHz to 1Hz

    // counting direction: 0=up, 1=down
    TIM3->CR1 &= ~TIM_CR1_DIR;

    // configure TIM3 for PWM mode 1 (110) for all 4 channels
    TIM3->CCMR1 &=  ~TIM_CCMR1_OC1M_0; 
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

    TIM3->CCMR1 &=  ~TIM_CCMR1_OC2M_0; 
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

    TIM3->CCMR2 &= ~TIM_CCMR2_OC3M_0;
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;

    TIM3->CCMR2 &= ~TIM_CCMR2_OC4M_0;
    TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

    // CCR Value from 0 to 1000
    TIM3->CCR1 = 800;
    TIM3->CCR2 = 600;
    TIM3->CCR3 = 400;
    TIM3->CCR4 = 200;

    // enable the four channel outputs
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    // enable tim3
    TIM3->CR1 |= TIM_CR1_CEN;

}

//============================================================================
// UART-controlled PWM duty cycle
// The host sends a duty cycle in percent as a text line (e.g. "40\n") over
// USART5 (PC12 = TX, PD2 = RX, 115200 8N1) and the STM32 sets the duty cycle
// of PC9 (TIM3 CH4). Names are prefixed to avoid clashing with autotest.o.
//============================================================================
#define CMD_MAX 24

// Host link. The team PCB (delivery-robot-pcb) wires USART2 to the Pi connector J13:
// PA2 = USART2_TX -> R12 1k -> J13 pin 2 (to the Pi's RX), PA3 = USART2_RX <- R13 1k <- J13 pin 1 (the Pi's TX).
// The Nucleo build used USART5 on PC12/PD2 (through an FTDI adapter); define HOST_UART_NUCLEO_USART5 for that.
#ifdef HOST_UART_NUCLEO_USART5
#define HOST_UART            USART5
#define HOST_UART_RCC_EN     RCC_APB1ENR_USART5EN
#define HOST_DMA_CSELR       (DMA1_CSELR_CH1_USART5_RX | DMA1_CSELR_CH2_USART5_TX)
#else
#define HOST_UART            USART2
#define HOST_UART_RCC_EN     RCC_APB1ENR_USART2EN
#define HOST_DMA_CSELR       (DMA1_CSELR_CH1_USART2_RX | DMA1_CSELR_CH2_USART2_TX)
#endif

// The host UART runs on DMA instead of polling RDR/TDR. Not on interrupts: autotest.o already defines
// USART3_8_IRQHandler (shared by USART3-8). DMA1 ch1 = RX (circular), ch2 = TX (one shot per buffer).
#define U5_RX_SIZE 128
#define U5_TX_SIZE 256
static volatile char u5_rx[U5_RX_SIZE];
static uint32_t u5_rx_tail;
static char u5_tx[2][U5_TX_SIZE];        // double buffer: one being sent, one being filled
static int u5_tx_cur;                    // index of the buffer being filled
static int u5_tx_fill;                   // bytes queued in it
volatile uint32_t u5_overruns;           // ORE events, reported in telemetry if nonzero

static void uart5_dma_init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1->CSELR = (DMA1->CSELR & ~(DMA_CSELR_C1S | DMA_CSELR_C2S))
                  | HOST_DMA_CSELR;

    DMA1_Channel1->CCR = 0;
    DMA1_Channel1->CPAR = (uint32_t)&HOST_UART->RDR;
    DMA1_Channel1->CMAR = (uint32_t)u5_rx;
    DMA1_Channel1->CNDTR = U5_RX_SIZE;
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL_1;   // 8-bit, periph -> mem
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    DMA1_Channel2->CCR = 0;
    DMA1_Channel2->CPAR = (uint32_t)&HOST_UART->TDR;
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_0;   // 8-bit, mem -> periph
}

void uart5_init(void) {
#ifdef HOST_UART_NUCLEO_USART5
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;

    // PC12 = USART5_TX, PD2 = USART5_RX, both alternate function 2
    GPIOC->MODER &= ~GPIO_MODER_MODER12;
    GPIOC->MODER |= GPIO_MODER_MODER12_1;
    GPIOD->MODER &= ~GPIO_MODER_MODER2;
    GPIOD->MODER |= GPIO_MODER_MODER2_1;
    GPIOC->AFR[1] &= ~GPIO_AFRH_AFRH4;
    GPIOC->AFR[1] |= 0x2 << 16;
    GPIOD->AFR[0] &= ~GPIO_AFRL_AFRL2;
    GPIOD->AFR[0] |= 0x2 << 8;
#else
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // PA2 = USART2_TX, PA3 = USART2_RX, both alternate function 1. PA3 gets a pull-up so the RX line
    // idles high (no stray start bits) while the Pi end is unwired or the Pi is booting.
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
    GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPDR3) | GPIO_PUPDR_PUPDR3_0;
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
    GPIOA->AFR[0] |= (0x1 << 8) | (0x1 << 12);
#endif

    RCC->APB1ENR |= HOST_UART_RCC_EN;
    HOST_UART->CR1 &= ~USART_CR1_UE;
    HOST_UART->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1 | USART_CR1_PCE | USART_CR1_OVER8);
    HOST_UART->CR2 &= ~USART_CR2_STOP;
    HOST_UART->BRR = 0x1A1; // 48 MHz / 115200 = 417
    HOST_UART->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
    HOST_UART->CR1 |= USART_CR1_TE | USART_CR1_RE;
    HOST_UART->CR1 |= USART_CR1_UE;
    while ((HOST_UART->ISR & (USART_ISR_TEACK | USART_ISR_REACK)) != (USART_ISR_TEACK | USART_ISR_REACK))
        ;
    uart5_dma_init();
}

// Bytes received but not yet read. RX runs in circular DMA mode, so the hardware stores every byte
// even while the main loop is busy (e.g. formatting telemetry); before, a byte that arrived during
// a blocking print overran RDR and was lost, dropping that command.
// Returns 1 and stores the next byte in *c, or 0 if nothing is waiting.
static int uart5_getc(char *c) {
    if (HOST_UART->ISR & USART_ISR_ORE) {   // can't happen with DMA keeping up; counted in case it does
        HOST_UART->ICR = USART_ICR_ORECF;
        u5_overruns++;
    }
    uint32_t head = U5_RX_SIZE - DMA1_Channel1->CNDTR;
    if (head >= U5_RX_SIZE)
        head = 0;
    if (u5_rx_tail == head)
        return 0;
    *c = u5_rx[u5_rx_tail];
    u5_rx_tail = (u5_rx_tail + 1) % U5_RX_SIZE;
    return 1;
}

// Start sending the buffer being filled if the TX DMA channel is idle. Call often (the main loops
// do); uart5_puts() calls it too.
static void uart5_tx_pump(void) {
    if (u5_tx_fill == 0)
        return;
    if ((DMA1_Channel2->CCR & DMA_CCR_EN) && DMA1_Channel2->CNDTR != 0)
        return;                          // previous buffer still going out
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CMAR = (uint32_t)u5_tx[u5_tx_cur];
    DMA1_Channel2->CNDTR = u5_tx_fill;
    DMA1_Channel2->CCR |= DMA_CCR_EN;
    u5_tx_cur ^= 1;                      // fill the other buffer while this one is sent
    u5_tx_fill = 0;
}

// Queue one byte; only waits if a whole buffer is queued behind the one being sent.
static void uart5_putc(char c) {
    while (u5_tx_fill >= U5_TX_SIZE)
        uart5_tx_pump();
    u5_tx[u5_tx_cur][u5_tx_fill++] = c;
}

// Queue a string and return without waiting for it to be sent.
static void uart5_puts(const char *s) {
    while (*s)
        uart5_putc(*s++);
    uart5_tx_pump();
}

// Sets the duty cycle of PC9 (TIM3 CH4); the other channels are untouched.
// Returns the duty (%) actually produced after rounding to a whole timer count.
double pc9_set_duty(double pct) {
    uint32_t period = TIM3->ARR + 1;
    TIM3->CCR4 = (uint32_t)(period * pct / 100.0 + 0.5);
    return 100.0 * TIM3->CCR4 / period;
}

void pwm_uart_loop(void) {
    char line[CMD_MAX];
    int n = 0;

    for (;;) {
        uart5_tx_pump();
        char c;
        if (!uart5_getc(&c))
            continue;
        if (c != '\r' && c != '\n') {
            if (n < CMD_MAX - 1)
                line[n] = c;
            n++; // keep counting so an over-long line is rejected, not truncated
            continue;
        }

        if (n == 0)
            continue; // blank line, or the \n of a \r\n pair
        if (n > CMD_MAX - 1) {
            n = 0;
            uart5_puts("ERR line too long\r\n");
            continue;
        }
        line[n] = '\0';
        n = 0;

        char *end;
        double duty = strtod(line, &end);
        while (*end == ' ' || *end == '\t')
            end++;
        if (end == line || *end != '\0' || !(duty >= 0.0 && duty <= 100.0)) {
            uart5_puts("ERR need a number from 0 to 100 (%)\r\n");
            continue;
        }

        char reply[48];
        snprintf(reply, sizeof reply, "OK PC9 duty %.1f%%\r\n", pc9_set_duty(duty));
        uart5_puts(reply);
    }
}

//============================================================================
// Closed-loop PID motor speed control, two motors
//
// Hardware: Cytron MDD20A dual 20 A driver (6-30 V), both channels, driven
// sign-magnitude: PWMn = speed, DIRn = direction. PWM low brakes (MnA = MnB
// = low). Accepts 3.3 V logic, PWM up to 20 kHz. No reverse-polarity
// protection on VB+/VB-.
// Motors: AndyMark PG27 (am-3651), RS775-125, 12 V, gear ratio 3249/121 =
// 26.85:1, no-load 198 RPM at the output. Rear-mounted hall encoder, 7 pulses
// per motor rev per channel, 5 V supply: 7 x 4 edges x 26.85 = 751.8 counts
// per OUTPUT shaft rev. AndyMark notes the rear mount reverses its count
// direction relative to a front-mounted encoder (see MOTORn_ENC_INVERT).
//
//   Motor 1 (MDD20A channel 1)
//   PWM1 -> PA6  -> TIM3_CH1 (AF1), 20 kHz PWM
//   DIR1 -> PB0  -> plain GPIO output (0 = forward, 1 = reverse)
//   Encoder A (yellow) -> PA15 -> TIM2_CH1 (AF2), 5 V tolerant
//   Encoder B (green)  -> PB3  -> TIM2_CH2 (AF2), 5 V tolerant
//   Motor 2 (MDD20A channel 2)
//   PWM2 -> PA7  -> TIM3_CH2 (AF1), 20 kHz PWM (same timer as PWM1)
//   DIR2 -> PB2  -> plain GPIO output (0 = forward, 1 = reverse)
//   Encoder A -> PA8 -> TIM1_CH1 (AF2), 5 V tolerant
//   Encoder B -> PA9 -> TIM1_CH2 (AF2), 5 V tolerant
//   TIM2 (32-bit) and TIM1 (16-bit, extended to 32 bits in software) run in
//   hardware quadrature encoder mode (SMS = 011, counts both edges of both
//   channels), so no ISR is needed just to count pulses.
//
// Both motors share one PID tick (TIM14, 100 Hz) and one set of constants;
// each has its own gains, duty cap, target, open-loop test and stall fault.
//
// Safety: there is no current sensing here. Estimated from the specs (198 RPM
// no-load, 6.3 ft-lbf stall, 26.85:1, lossless gearbox) the RS775-125 has
// ~0.7 ohm and ~15-20 A stall current at 12 V, within the MDD20A's 20 A
// rating; the bench supply's ~5 A limit caps it first. Current is bounded by
// the supply limit and the MDD20A's own current limiting, not by the duty cap.
// The duty cap boots at 100% (full speed range, ~214 RPM); lower it with
// `cap` to limit speed/torque. A software stall watchdog force-stops a
// motor if it asks for max output while showing ~zero speed for
// MOTOR_STALL_TIMEOUT_MS, and every direction reversal passes through one
// tick of zero duty (= brake on the MDD20A) before flipping DIR.
//============================================================================
#include <math.h> // fabsf

#define MOTOR_COUNTS_PER_REV   751.8f   // encoder counts per OUTPUT shaft rev
#define MOTOR_PID_HZ           100      // PID loop rate (TIM14)
#define MOTOR_PID_DT           (1.0f / MOTOR_PID_HZ)
// Speed is the count change over the last MOTOR_SPEED_WINDOW ticks, updated
// every tick. One count per 10 ms tick would be ~8 RPM; over 5 ticks (50 ms)
// it is ~1.6 RPM.
#define MOTOR_SPEED_WINDOW     5
#define MOTOR_RPM_PER_COUNT    (60.0f / (MOTOR_COUNTS_PER_REV * MOTOR_SPEED_WINDOW * MOTOR_PID_DT))
#define MOTOR_PWM_PERIOD       2400     // TIM3 ARR+1 -> 48 MHz / 2400 = 20 kHz
#define MOTOR_DUTY_HARD_MAX    100.0f   // % duty; the MDD20A limits its own current
#define MOTOR_RAMP_RPM_PER_S   200.0f   // soft-start slew rate
#define MOTOR_STALL_RPM        3.0f     // "basically not moving" threshold
#define MOTOR_STALL_TIMEOUT_MS 500

// Encoder build switches.
// MOTOR_ENC_PULLUPS: the encoder pins only tolerate 5 V with the internal
// pull-ups off, so they're off by default. Set 1 only if the encoders turn
// out to be open-collector (then the lines idle at 3.3 V via the pull-ups).
// MOTORn_ENC_INVERT: set 1 if `open 20` (`m2 open 20`) shows negative meas
// (a rear-mounted encoder counts backwards); inverts TI1 polarity, which
// reverses the count.
#define MOTOR_ENC_PULLUPS      0
// On the team PCB the encoder A/B order is the other way round from the Nucleo wiring: with TI1 inverted a
// positive command counted negative on both motors (19 Nov 2026 bring-up), so the inversion is off here.
#ifdef HOST_UART_NUCLEO_USART5
#define MOTOR1_ENC_INVERT      1
#define MOTOR2_ENC_INVERT      1
#else
#define MOTOR1_ENC_INVERT      0
#define MOTOR2_ENC_INVERT      0
#endif

// Tuned by motor_autotune.py on the unloaded PG27 (2026-10-19): measured
// 2.21 RPM per % duty above ~4% duty, ~30 ms delay (incl. the 50 ms speed
// window), ~34 ms time constant; SIMC PI for a ~0.2 s closed-loop time
// constant. Verified 0->30->60->-30->0 RPM: <=2.3% overshoot, settle <=1.1 s,
// steady error ~0.1 RPM. Retune (rerun the script) after adding load.
// Retuned 2026-11-04 per motor, with the wheels fitted (chassis on a stand, battery
// power): the wheels slow the response (~45-50 ms time constant, ~20 ms delay)
// and the speed curve is ~2.1-2.2 RPM per % duty, 195-198 RPM at 100%.
// Verified 0->30->60->-30->0 RPM on each: overshoot <=6.3% (motor 1, 0->30),
// settle <=0.61 s after the ramp, steady error <=0.7 RPM.
// Bring-up procedure: start with Ki = Kd = 0, raise Kp until you see
// sustained oscillation at your target RPM, then back Kp off to ~50-60% of
// that value; raise Ki just enough to kill steady-state RPM error without
// re-introducing oscillation; add a little Kd only if overshoot on step
// changes is a problem. Retune (lower Kp especially) if you add real load.
#define MOTOR1_BOOT_KP         0.104f
#define MOTOR1_BOOT_KI         2.055f
#define MOTOR2_BOOT_KP         0.089f
#define MOTOR2_BOOT_KI         2.062f
#define MOTOR_BOOT_KD          0.0f

// Open-loop test mode ("open <pct>" over UART): drives a fixed duty with the
// PID bypassed, for telling a dead drive path apart from a dead encoder. The
// stall watchdog can't help here (a dead encoder looks like a stall), so the
// run is time-limited instead and the duty is still clamped to the cap.
#define MOTOR_OPEN_LOOP_MS     15000

typedef struct {
    // hardware
    volatile uint32_t *ccr;              // PWM compare register (TIM3 CCR1 / CCR2)
    GPIO_TypeDef *dir_port;
    uint32_t dir_pin;
    TIM_TypeDef *enc;                    // encoder timer
    int enc_16bit;                       // TIM1 is 16 bits wide, TIM2 32
    // shared with the UART loop
    volatile float Kp, Ki, Kd;
    volatile float Kf, Ks;               // feedforward: Kf %/RPM x setpoint + Ks % in its direction (0 at boot)
    volatile float cap_pct;              // boot default: full range; lower with `cap`
    volatile float target_rpm;           // set over UART, signed (+ = forward)
    volatile float ramped_rpm;           // soft-start setpoint, updated in the ISR
    volatile float measured_rpm;
    volatile float output_pct;           // last PID output actually applied
    volatile int   fault_stall;          // latched; cleared by a UART command
    volatile float open_pct;
    volatile int   open_ms_left;
    volatile int32_t pos;                // encoder count since boot, 32 bits for both
    // ISR only
    float integral;
    float prev_error;
    uint16_t last_raw;                   // previous TIM1 count, for the 16-bit extension
    int32_t count_hist[MOTOR_SPEED_WINDOW]; // pos at each of the last WINDOW ticks
    int hist_i;                          // slot written MOTOR_SPEED_WINDOW ticks ago
    int forward;                         // direction currently applied to DIR
    uint32_t stall_ms;
} motor_t;

#define MOTOR_COUNT 2
static motor_t mot[MOTOR_COUNT];

volatile int   mc_telemetry_due = 0;     // set every few ticks, printed in main loop
static int mc_tick = 0;
// Telemetry rate, set with `rate <hz>` (1-50). At the default 10 Hz the line
// format is unchanged; at any other rate a " t=<ms>" timestamp is appended so
// a host script can time step responses without relying on USB latency.
#define MOTOR_TEL_MAX_HZ       50
volatile int mc_tel_div = MOTOR_PID_HZ / 10;  // PID ticks per telemetry line
volatile uint32_t mc_ms = 0;                  // ms since the PID timer started

// Command watchdog (requirement 4-3). Once a `d` drive command has been received, the motors stop being driven
// (target 0, output 0 at once: they coast) if no further `d` arrives within cmd_timeout_ms; the next `d` re-arms it.
// The Pi's control loop sends `d` every 50 ms, so this only fires if the Pi program hangs, dies or loses the link.
// `timeout <ms>` changes it (0 = off). Bench tools that only send bare targets (motor_live.py) never arm it.
#define CMD_TIMEOUT_MS_DEFAULT 400
volatile uint32_t cmd_timeout_ms = CMD_TIMEOUT_MS_DEFAULT;
volatile uint32_t cmd_age_ms = 0;
volatile int cmd_armed = 0;                   // a `d` command has been seen since boot
volatile int cmd_timed_out = 0;               // the watchdog stopped the motors (" WD" in telemetry) until the next `d`

void setup_motor_pwm(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;   // AF mode
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7);
    GPIOA->AFR[0] |= (0x1 << (6 * 4)) | (0x1 << (7 * 4));       // AF1 = TIM3_CH1 / TIM3_CH2

    TIM3->PSC = 0;
    TIM3->ARR = MOTOR_PWM_PERIOD - 1;        // 48 MHz / 2400 = 20 kHz
    TIM3->CR1 &= ~TIM_CR1_DIR;

    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC2M_0);
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2       // PWM mode 1 on CH1 and CH2
                 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

    TIM3->CCR1 = 0;                          // start at 0% -- motors off
    TIM3->CCR2 = 0;
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void setup_motor_dir(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER2);
    GPIOB->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER2_0;   // general-purpose outputs
    GPIOB->BSRR = (1 << (0 + 16)) | (1 << (2 + 16));            // start LOW = forward, off via CCR=0
}

// Quadrature encoder mode on CH1/CH2 of `tim`; the pins are set up by the caller.
static void encoder_timer_init(TIM_TypeDef *tim, int invert, uint32_t arr) {
    tim->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    tim->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;  // map CH1/CH2 to TI1/TI2
    tim->CCMR1 |= (0x3 << 4) | (0x3 << 12);             // light input filter on both
    tim->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);      // non-inverted
    if (invert)
        tim->CCER |= TIM_CCER_CC1P;                     // invert TI1 -> count runs backwards
    tim->SMCR &= ~TIM_SMCR_SMS;
    tim->SMCR |= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;       // encoder mode 3 (both edges, both ch)
    tim->ARR = arr;
    tim->CNT = 0;
    tim->CR1 |= TIM_CR1_CEN;
}

void setup_encoders(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    // Motor 1: PA15 = TIM2_CH1 (AF2), PB3 = TIM2_CH2 (AF2).
    // Motor 2: PA8 = TIM1_CH1 (AF2), PA9 = TIM1_CH2 (AF2).
    // Pull-ups off unless MOTOR_ENC_PULLUPS: the 5 V encoders drive these pins directly.
    GPIOA->MODER &= ~(GPIO_MODER_MODER15 | GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOA->MODER |= GPIO_MODER_MODER15_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR15 | GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
#if MOTOR_ENC_PULLUPS
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR15_0 | GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0;
#endif
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH7 | GPIO_AFRH_AFRH0 | GPIO_AFRH_AFRH1);
    GPIOA->AFR[1] |= (0x2 << 28) | (0x2 << 0) | (0x2 << 4);  // AF2: pin15 nibble 7, pin8 0, pin9 1

    GPIOB->MODER &= ~GPIO_MODER_MODER3;
    GPIOB->MODER |= GPIO_MODER_MODER3_1;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR3;
#if MOTOR_ENC_PULLUPS
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR3_0;
#endif
    GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL3;
    GPIOB->AFR[0] |= 0x2 << 12;              // AF2, pin3 = AFR[0] nibble 3

    encoder_timer_init(TIM2, MOTOR1_ENC_INVERT, 0xFFFFFFFF);
    encoder_timer_init(TIM1, MOTOR2_ENC_INVERT, 0xFFFF);
}

static void motor_init(motor_t *m, volatile uint32_t *ccr, GPIO_TypeDef *dir_port, uint32_t dir_pin,
                       TIM_TypeDef *enc, int enc_16bit, float kp, float ki) {
    memset(m, 0, sizeof *m);
    m->ccr = ccr;
    m->dir_port = dir_port;
    m->dir_pin = dir_pin;
    m->enc = enc;
    m->enc_16bit = enc_16bit;
    m->Kp = kp;
    m->Ki = ki;
    m->Kd = MOTOR_BOOT_KD;
    m->cap_pct = 100.0f;
    m->forward = 1;
}

void init_motor_pid_timer(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = 48000 - 1;                  // 48 MHz -> 1 kHz tick
    TIM14->ARR = (1000 / MOTOR_PID_HZ) - 1;  // 1 kHz -> MOTOR_PID_HZ
    TIM14->DIER |= TIM_DIER_UIE;
    TIM14->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] |= 1 << TIM14_IRQn;
}

void setup_motor_control(void) {
    motor_init(&mot[0], &TIM3->CCR1, GPIOB, 0, TIM2, 0, MOTOR1_BOOT_KP, MOTOR1_BOOT_KI);
    motor_init(&mot[1], &TIM3->CCR2, GPIOB, 2, TIM1, 1, MOTOR2_BOOT_KP, MOTOR2_BOOT_KI);
    setup_motor_pwm();
    setup_motor_dir();
    setup_encoders();
    init_motor_pid_timer();
}

// One PID tick for one motor.
static void motor_tick(motor_t *m) {
    // --- measure speed from the encoder, over the last MOTOR_SPEED_WINDOW ticks ---
    int32_t count;
    if (m->enc_16bit) {
        uint16_t raw = (uint16_t)m->enc->CNT;
        count = m->pos + (int16_t)(uint16_t)(raw - m->last_raw);   // < 32768 counts per tick
        m->last_raw = raw;
    } else {
        count = (int32_t)m->enc->CNT;
    }
    m->pos = count;
    int32_t delta = count - m->count_hist[m->hist_i];
    m->count_hist[m->hist_i] = count;
    if (++m->hist_i >= MOTOR_SPEED_WINDOW)
        m->hist_i = 0;
    m->measured_rpm = delta * MOTOR_RPM_PER_COUNT;

    // --- soft-start: slew the working setpoint toward the commanded one ---
    float max_step = MOTOR_RAMP_RPM_PER_S * MOTOR_PID_DT;
    float diff = m->target_rpm - m->ramped_rpm;
    if (diff > max_step) diff = max_step;
    if (diff < -max_step) diff = -max_step;
    m->ramped_rpm += diff;

    // --- PID ---
    float cap = m->cap_pct;
    if (cap > MOTOR_DUTY_HARD_MAX) cap = MOTOR_DUTY_HARD_MAX;
    if (cap < 0.0f) cap = 0.0f;

    float Ki = m->Ki;
    float error = m->ramped_rpm - m->measured_rpm;
    m->integral += error * MOTOR_PID_DT;
    float i_term = Ki * m->integral;
    if (i_term > cap)  { i_term = cap;  m->integral = (Ki != 0.0f) ? cap / Ki : 0.0f; }
    if (i_term < -cap) { i_term = -cap; m->integral = (Ki != 0.0f) ? -cap / Ki : 0.0f; }
    float derivative = (error - m->prev_error) / MOTOR_PID_DT;
    m->prev_error = error;

    // Feedforward (off at boot): the duty the setpoint needs on its own, so the PID only has to
    // correct the remainder. On the floor the load makes the integral alone take ~2-3 s to build up.
    float ff = m->Kf * m->ramped_rpm;
    if (m->ramped_rpm > 0.0f) ff += m->Ks;
    if (m->ramped_rpm < 0.0f) ff -= m->Ks;

    float output = m->Kp * error + i_term + m->Kd * derivative + ff;

    int open_loop = m->open_ms_left > 0;
    if (open_loop) {
        output = m->open_pct;
        m->integral = 0.0f;                  // don't carry windup back into closed loop
        m->open_ms_left -= (1000 / MOTOR_PID_HZ);
        if (m->open_ms_left <= 0)
            m->open_pct = 0.0f;
    }

    // Target 0 means off once the ramp has reached it. Otherwise the integral
    // keeps whatever it built up while the motor slowed and, with no error left
    // to unwind it, holds a few % of duty into a stopped motor indefinitely --
    // too little to turn it, but a steady current through the stalled motor
    // (~0.8 A at the 4.6% seen during tuning, at an estimated ~0.7 ohm).
    if (!open_loop && m->target_rpm == 0.0f && m->ramped_rpm == 0.0f) {
        output = 0.0f;
        m->integral = 0.0f;
    }

    if (output > cap)  output = cap;
    if (output < -cap) output = -cap;
    m->output_pct = output;

    // --- apply to hardware, with a dead tick at 0 duty on every reversal ---
    if (m->fault_stall) {
        *m->ccr = 0;
    } else {
        int want_forward = (output >= 0.0f);
        if (want_forward != m->forward) {
            *m->ccr = 0;
            m->dir_port->BSRR = want_forward ? (1 << (m->dir_pin + 16)) : (1 << m->dir_pin);
            m->forward = want_forward;
        } else {
            float mag = fabsf(output);
            *m->ccr = (uint32_t)(mag / 100.0f * MOTOR_PWM_PERIOD + 0.5f);
        }
    }

    // --- stall watchdog: pinned at the cap but not actually turning ---
    // (skipped in open-loop test mode, which is time-limited instead)
    if (!m->fault_stall && !open_loop) {
        int pinned = fabsf(output) >= (cap - 1.0f) && cap > 0.0f;
        int not_moving = fabsf(m->measured_rpm) < MOTOR_STALL_RPM;
        if (pinned && not_moving) {
            m->stall_ms += (1000 / MOTOR_PID_HZ);
            if (m->stall_ms >= MOTOR_STALL_TIMEOUT_MS) {
                m->fault_stall = 1;
                *m->ccr = 0;
                m->target_rpm = 0.0f;
                m->ramped_rpm = 0.0f;
                m->integral = 0.0f;
            }
        } else {
            m->stall_ms = 0;
        }
    }
}

void TIM14_IRQHandler(void) {
    TIM14->SR &= ~TIM_SR_UIF;
    if (cmd_armed && cmd_timeout_ms) {           // command watchdog, see cmd_timeout_ms
        if (cmd_age_ms < cmd_timeout_ms) {
            cmd_age_ms += 1000 / MOTOR_PID_HZ;
        } else if (!cmd_timed_out) {
            cmd_timed_out = 1;
            for (int i = 0; i < MOTOR_COUNT; i++) {
                if (mot[i].open_ms_left > 0)     // an open-loop test has its own time limit
                    continue;
                mot[i].target_rpm = 0.0f;
                mot[i].ramped_rpm = 0.0f;        // no ramp-down: motor_tick sets the output to 0 on this tick
            }
        }
    }
    for (int i = 0; i < MOTOR_COUNT; i++)
        motor_tick(&mot[i]);

    mc_ms += 1000 / MOTOR_PID_HZ;
    if (++mc_tick >= mc_tel_div) {           // 10 Hz by default, see `rate`
        mc_tick = 0;
        mc_telemetry_due = 1;
    }
}

// The per-motor commands (see motor_uart_loop). `cmd` is lowercased and trimmed.
// Replies are the same for either motor, so host tools written for motor 1
// (motor_autotune.py, motor_live.py) work on motor 2 by prefixing "m2 ".
static void motor_command(motor_t *m, const char *line) {
    char reply[64];
    if (strncmp(line, "reset", 5) == 0) {
        m->fault_stall = 0;
        m->stall_ms = 0;
        uart5_puts("OK fault cleared\r\n");
        return;
    }
    if (strncmp(line, "stop", 4) == 0) {
        m->open_ms_left = 0;
        m->open_pct = 0.0f;
        m->target_rpm = 0.0f;
        uart5_puts("OK target 0 RPM\r\n");
        return;
    }
    if (strncmp(line, "open", 4) == 0) {
        char *end;
        double v = strtod(line + 4, &end);
        if (end == line + 4 || !(v >= -100.0 && v <= 100.0)) {
            uart5_puts("ERR need 'open <-100..100>'\r\n");
            return;
        }
        if (m->fault_stall) {
            uart5_puts("ERR stall fault latched, send 'reset' first\r\n");
            return;
        }
        m->target_rpm = 0.0f;
        m->ramped_rpm = 0.0f;
        m->open_pct = (float)v;
        m->open_ms_left = MOTOR_OPEN_LOOP_MS;
        snprintf(reply, sizeof reply, "OK open-loop %.1f%% for %d ms (clamped to cap)\r\n",
                  v, MOTOR_OPEN_LOOP_MS);
        uart5_puts(reply);
        return;
    }
    if (line[0] == 'k' && (line[1] == 'p' || line[1] == 'i' || line[1] == 'd' || line[1] == 'f' || line[1] == 's')) {
        char *end;
        double v = strtod(line + 2, &end);
        while (*end == ' ' || *end == '\t')
            end++;
        if (end == line + 2 || *end != '\0' || !(v >= 0.0 && v <= 100.0)) {
            uart5_puts("ERR need 'kp|ki|kd|kf|ks <0-100>'\r\n");
            return;
        }
        __disable_irq();                     // don't let the PID tick see a half-applied change
        if (line[1] == 'p') {
            m->Kp = (float)v;
        } else if (line[1] == 'd') {
            m->Kd = (float)v;
        } else if (line[1] == 'f') {
            m->Kf = (float)v;
        } else if (line[1] == 's') {
            m->Ks = (float)v;
        } else {
            // keep Ki*integral (the integral's share of the output) continuous,
            // so changing Ki mid-run doesn't kick the output
            m->integral = (v > 0.0 && m->Ki > 0.0f) ? m->integral * m->Ki / (float)v : 0.0f;
            m->Ki = (float)v;
        }
        __enable_irq();
        snprintf(reply, sizeof reply, "OK kp=%.4f ki=%.4f kd=%.4f kf=%.4f ks=%.2f\r\n",
                  (double)m->Kp, (double)m->Ki, (double)m->Kd, (double)m->Kf, (double)m->Ks);
        uart5_puts(reply);
        return;
    }
    if (strncmp(line, "cap", 3) == 0) {
        char *end;
        double v = strtod(line + 3, &end);
        if (end == line + 3 || !(v >= 0.0 && v <= 100.0)) {
            uart5_puts("ERR need 'cap <0-100>'\r\n");
            return;
        }
        m->cap_pct = (float)(v > MOTOR_DUTY_HARD_MAX ? MOTOR_DUTY_HARD_MAX : v);
        snprintf(reply, sizeof reply, "OK cap %.1f%% (hard max %.0f%%)\r\n",
                  m->cap_pct, (double)MOTOR_DUTY_HARD_MAX);
        uart5_puts(reply);
        return;
    }

    char *end;
    double rpm = strtod(line, &end);
    while (*end == ' ' || *end == '\t')
        end++;
    if (end == line || *end != '\0') {
        uart5_puts("ERR unknown command; send a target RPM, 'open <pct>', 'cap <pct>', 'reset', or 'stop'\r\n");
        return;
    }
    m->open_ms_left = 0;                     // a speed command ends any open-loop test
    m->open_pct = 0.0f;
    m->target_rpm = (float)rpm;
    snprintf(reply, sizeof reply, "OK target %.1f RPM\r\n", m->target_rpm);
    uart5_puts(reply);
}

// Host UART command loop, reusing uart5_init()/uart5_putc()/uart5_puts()
// from the PC9 PWM demo above (host UART: USART2 on PA2/PA3 on the PCB, 115200 8N1).
//
// ---------------------------------------------------------------------------
// Latch servo: standard hobby (PWM) servo on the team PCB's J17.
// PB10 -> U8 SN74LVC1T45 (A side; DIR = PB11 held high: MCU drives) -> R42 100R -> J17 pin 3, 5 V signal.
// J17 pin 2 is +12 V (VBAT_SERVO): this 5 V servo is powered separately, not from J17 pin 2.
// PB10's only timer channel is TIM2_CH3, and TIM2 is motor 1's encoder, so the pulse is made in software
// from TIM15: the update interrupt (every 20 ms) raises PB10, the CC1 interrupt drops it after the pulse width.
// TIM15 runs at a higher interrupt priority than the PID tick (TIM14), so the pulse edges don't wait for it.
// Moves ramp at servo_rate us/s (default ~33 us/s = ~3.3 deg/s): fast moves made the servo jerk (supply sag).
// No pulses at boot. Positions set 19 Nov 2026 after re-fitting the horn: closed 1700 us (0 deg), open 2000 us
// (2000 us = 30 deg first, widened to 2050 us; longer pulse = clockwise, ~0.1 deg/us).
#define SERVO_CLOSED_US    1700
#define SERVO_OPEN_US      2050     // was 2000; opened ~5 deg more on 19 Nov
#define SERVO_MIN_US       1000
#define SERVO_MAX_US       2100
#define SERVO_RATE_DEFAULT 33       // us per second
static volatile uint32_t servo_target_us;   // 0 = no pulses
static volatile uint32_t servo_cur_x16;     // current pulse width, 1/16 us (0 = no pulses)
static volatile uint32_t servo_last_x16;    // last pulse width sent, so a restart begins where it stopped
static volatile uint32_t servo_rate = SERVO_RATE_DEFAULT;

void setup_servo(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    GPIOB->BSRR = GPIO_BSRR_BS_11 | GPIO_BSRR_BR_10;                 // U8: MCU drives; signal low
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11))
                   | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0;
    TIM15->PSC = 48 - 1;                     // 1 us ticks
    TIM15->ARR = 20000 - 1;                  // 20 ms frame, 50 Hz
    TIM15->CCR1 = 0;
    TIM15->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
    TIM15->CR1 = TIM_CR1_CEN;
    NVIC_SetPriority(TIM15_IRQn, 0);
    NVIC_SetPriority(TIM14_IRQn, 2);
    NVIC_EnableIRQ(TIM15_IRQn);
}

void TIM15_IRQHandler(void) {
    uint32_t sr = TIM15->SR;
    if (sr & TIM_SR_CC1IF) {                 // end of the pulse
        TIM15->SR = ~TIM_SR_CC1IF;
        GPIOB->BSRR = GPIO_BSRR_BR_10;
    }
    if (sr & TIM_SR_UIF) {                   // start of a 20 ms frame
        TIM15->SR = ~TIM_SR_UIF;
        uint32_t tgt = servo_target_us << 4, cur = servo_cur_x16;
        if (tgt == 0) {
            cur = 0;
        } else {
            if (cur == 0)
                cur = servo_last_x16 ? servo_last_x16 : tgt;
            uint32_t step = (servo_rate * 16u + 25u) / 50u;          // per 20 ms frame
            if (step == 0) step = 1;
            if (cur < tgt) cur = (tgt - cur > step) ? cur + step : tgt;
            else if (cur > tgt) cur = (cur - tgt > step) ? cur - step : tgt;
        }
        servo_cur_x16 = cur;
        if (cur) {
            servo_last_x16 = cur;
            TIM15->CCR1 = cur >> 4;
            GPIOB->BSRR = GPIO_BSRR_BS_10;
        }
    }
}

// "servo" commands, handled by motor_uart_loop:
//   servo                status: current and target pulse width, rate
//   servo open | close   ramp to SERVO_OPEN_US / SERVO_CLOSED_US
//   servo <us>           ramp to a pulse width (SERVO_MIN_US..SERVO_MAX_US); "servo 0" or "servo off" stops pulses
//   servo rate <us/s>    ramp speed, 1..2000 (default SERVO_RATE_DEFAULT)
static void servo_command(const char *arg) {
    char reply[64];
    while (*arg == ' ' || *arg == '\t')
        arg++;
    if (*arg == '\0') {
        // nothing to change
    } else if (strncmp(arg, "open", 4) == 0) {
        servo_target_us = SERVO_OPEN_US;
    } else if (strncmp(arg, "close", 5) == 0) {
        servo_target_us = SERVO_CLOSED_US;
    } else if (strncmp(arg, "off", 3) == 0) {
        servo_target_us = 0;
    } else if (strncmp(arg, "rate", 4) == 0) {
        char *end;
        long r = strtol(arg + 4, &end, 10);
        if (end == arg + 4 || r < 1 || r > 2000) {
            uart5_puts("ERR need 'servo rate <1-2000>' (us/s)\r\n");
            return;
        }
        servo_rate = (uint32_t)r;
    } else {
        char *end;
        long us = strtol(arg, &end, 10);
        while (*end == ' ' || *end == '\t')
            end++;
        if (end == arg || *end != '\0' || (us != 0 && (us < SERVO_MIN_US || us > SERVO_MAX_US))) {
            snprintf(reply, sizeof reply, "ERR need 'servo open|close|off|<%d-%d>|rate <n>'\r\n", SERVO_MIN_US, SERVO_MAX_US);
            uart5_puts(reply);
            return;
        }
        servo_target_us = (uint32_t)us;
    }
    snprintf(reply, sizeof reply, "OK servo %lu us target %lu rate %lu\r\n",
             (unsigned long)(servo_cur_x16 >> 4), (unsigned long)servo_target_us, (unsigned long)servo_rate);
    uart5_puts(reply);
}

// Commands (one per line). Without a prefix they act on motor 1, as before
// motor 2 existed, except `stop` and `reset`, which act on both:
//   <number>      set target RPM, signed, e.g. "40" or "-40"
//   cap <number>  set the runtime duty-cycle cap in %, clamped to
//                 MOTOR_DUTY_HARD_MAX regardless of what's asked for
//   open <pct>    open-loop test: fixed duty (signed), PID and stall watchdog
//                 bypassed, auto-stops after MOTOR_OPEN_LOOP_MS
//   kp|ki|kd <v>  set a PID gain live (0-100); replies with all of them
//   kf|ks <v>     feedforward: kf % duty per RPM of setpoint, ks % duty in the
//                 setpoint's direction (dead band); both 0 at boot
//   reset         clear a latched stall fault (both motors)
//   stop          target 0 on both motors, also ends open-loop tests
// m1 <cmd> / m2 <cmd>  any command above, for that motor only ("m2 40", "m2 open 20")
// d <rpm1> <rpm2>      both targets at once (drive command), replies "OK target <rpm1> <rpm2> RPM"
// rate <hz>            telemetry rate, 1-50 Hz (adds " t=<ms>" when not 10 Hz)
// servo [open|close|off|<us>|rate <us/s>]  latch servo on J17, see setup_servo(); not affected by stop
// timeout <ms>         command watchdog: after the first `d`, no `d` for this long -> both targets 0 (default 400, 0 = off)
//
// Telemetry: motor 1's fields first, unchanged, so motor-1 tools still parse it, then motor 2's:
//   tgt= meas= duty=% cap=% cnt= tgt2= meas2= duty2=% cap2=% cnt2= [OPEN] [FAULT:STALL]
//   [open2] [stall2] [WD] [ORE=] [t=]      (WD: the command watchdog stopped the motors)
// (motor 2's flags are lowercase so a search for "OPEN" / "FAULT:STALL" still means motor 1)
void motor_uart_loop(void) {
    char line[CMD_MAX];
    int n = 0;
    uart5_puts("Motor PID ready (2 motors). Send a target RPM, 'd <rpm1> <rpm2>', 'm2 <cmd>', 'open <pct>', 'cap <pct>', "
               "'kp|ki|kd <v>', 'rate <hz>', 'reset', or 'stop'.\r\n");

    for (;;) {
        if (mc_telemetry_due) {
            mc_telemetry_due = 0;
            motor_t *a = &mot[0], *b = &mot[1];
            char t[128];
            snprintf(t, sizeof t, "tgt=%.1f meas=%.1f duty=%.1f%% cap=%.1f%% cnt=%ld",
                      a->target_rpm, a->measured_rpm, a->output_pct, a->cap_pct, (long)a->pos);
            uart5_puts(t);
            snprintf(t, sizeof t, " tgt2=%.1f meas2=%.1f duty2=%.1f%% cap2=%.1f%% cnt2=%ld%s%s%s%s%s",
                      b->target_rpm, b->measured_rpm, b->output_pct, b->cap_pct, (long)b->pos,
                      a->open_ms_left > 0 ? " OPEN" : "",
                      a->fault_stall ? " FAULT:STALL" : "",
                      b->open_ms_left > 0 ? " open2" : "",
                      b->fault_stall ? " stall2" : "",
                      cmd_timed_out ? " WD" : "");
            uart5_puts(t);
            if (u5_overruns) {
                snprintf(t, sizeof t, " ORE=%lu", (unsigned long)u5_overruns);
                uart5_puts(t);
            }
            if (mc_tel_div != MOTOR_PID_HZ / 10) {
                snprintf(t, sizeof t, " t=%lu", (unsigned long)mc_ms);
                uart5_puts(t);
            }
            uart5_puts("\r\n");
        }

        uart5_tx_pump();
        char c;
        if (!uart5_getc(&c))
            continue;
        if (c != '\r' && c != '\n') {
            if (n < CMD_MAX - 1)
                line[n] = c;
            n++;
            continue;
        }
        if (n == 0)
            continue;
        if (n > CMD_MAX - 1) {
            n = 0;
            uart5_puts("ERR line too long\r\n");
            continue;
        }
        line[n] = '\0';
        n = 0;

        // accept "Open 25", " reset", etc.: drop leading blanks, lowercase
        char *p = line;
        while (*p == ' ' || *p == '\t')
            p++;
        memmove(line, p, strlen(p) + 1);
        for (char *q = line; *q; q++)
            if (*q >= 'A' && *q <= 'Z')
                *q += 'a' - 'A';

        if (line[0] == 'm' && (line[1] == '1' || line[1] == '2') && (line[2] == ' ' || line[2] == '\t')) {
            char *cmd = line + 3;
            while (*cmd == ' ' || *cmd == '\t')
                cmd++;
            motor_command(&mot[line[1] - '1'], cmd);
            continue;
        }
        if (line[0] == 'd' && (line[1] == ' ' || line[1] == '\t')) {
            char *end1, *end2;
            double r1 = strtod(line + 2, &end1);
            double r2 = strtod(end1, &end2);
            while (*end2 == ' ' || *end2 == '\t')
                end2++;
            if (end1 == line + 2 || end2 == end1 || *end2 != '\0' || r1 != r1 || r2 != r2) {
                uart5_puts("ERR need 'd <rpm1> <rpm2>'\r\n");
                continue;
            }
            __disable_irq();                 // both targets change on the same PID tick
            for (int i = 0; i < MOTOR_COUNT; i++) {
                mot[i].open_ms_left = 0;
                mot[i].open_pct = 0.0f;
            }
            mot[0].target_rpm = (float)r1;
            mot[1].target_rpm = (float)r2;
            cmd_age_ms = 0;                  // feed the command watchdog
            cmd_armed = 1;
            cmd_timed_out = 0;
            __enable_irq();
            char reply[48];
            snprintf(reply, sizeof reply, "OK target %.1f %.1f RPM\r\n", mot[0].target_rpm, mot[1].target_rpm);
            uart5_puts(reply);
            continue;
        }
        if (strncmp(line, "timeout", 7) == 0 && (line[7] == '\0' || line[7] == ' ' || line[7] == '\t')) {
            char *end;
            long v = strtol(line + 7, &end, 10);
            if (end != line + 7 && v >= 0 && v <= 60000) {
                cmd_timeout_ms = (uint32_t)v;
                cmd_age_ms = 0;
            }
            char reply[48];
            snprintf(reply, sizeof reply, "OK timeout %lu ms\r\n", (unsigned long)cmd_timeout_ms);
            uart5_puts(reply);
            continue;
        }
        if (strncmp(line, "servo", 5) == 0 && (line[5] == '\0' || line[5] == ' ' || line[5] == '\t')) {
            servo_command(line + 5);
            continue;
        }
        if (strncmp(line, "stop", 4) == 0) {
            __disable_irq();
            for (int i = 0; i < MOTOR_COUNT; i++) {
                mot[i].open_ms_left = 0;
                mot[i].open_pct = 0.0f;
                mot[i].target_rpm = 0.0f;
            }
            __enable_irq();
            uart5_puts("OK target 0 RPM\r\n");
            continue;
        }
        if (strncmp(line, "reset", 5) == 0) {
            for (int i = 0; i < MOTOR_COUNT; i++) {
                mot[i].fault_stall = 0;
                mot[i].stall_ms = 0;
            }
            uart5_puts("OK fault cleared\r\n");
            continue;
        }
        if (strncmp(line, "rate", 4) == 0) {
            char *end;
            long hz = strtol(line + 4, &end, 10);
            if (end == line + 4 || hz < 1 || hz > MOTOR_TEL_MAX_HZ) {
                uart5_puts("ERR need 'rate <1-50>'\r\n");
                continue;
            }
            mc_tel_div = (int)((MOTOR_PID_HZ + hz / 2) / hz);
            char reply[48];
            snprintf(reply, sizeof reply, "OK rate %d Hz\r\n", MOTOR_PID_HZ / mc_tel_div);
            uart5_puts(reply);
            continue;
        }
        motor_command(&mot[0], line);
    }
}

void setup_tim1(void) {
    // Generally the steps are similar to those in setup_tim3
    // except we will need to set the MOE bit in BDTR.
    // Be sure to do so ONLY after enabling the RCC clock to TIM1.
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;      
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    
    // configure PA8-11 for alternate function mode;
    GPIOA->MODER &= ~0xFF0000;
    GPIOA->MODER |= 0xAA0000;

    // select AF2 for PA8-11
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFRH0;
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFRH1;
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFRH2;
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFRH3;

    GPIOA->AFR[1] |= 0x2;
    GPIOA->AFR[1] |= 0x2 << 4;
    GPIOA->AFR[1] |= 0x2 << 8;
    GPIOA->AFR[1] |= 0x2 << 12;

    // enable MOE bit 
    TIM1->BDTR |= TIM_BDTR_MOE;  
    TIM1->PSC = 1 - 1;
    TIM1->ARR = 2400 - 1;

    // counting direction: 0=up, 1=down
    TIM1->CR1 &= ~TIM_CR1_DIR; // default to 0 "up" so this can be ommitted

    // configure TIM3 for PWM mode 1 (110) for all 4 channels
    TIM1->CCMR1 &=  ~TIM_CCMR1_OC1M_0; 
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

    TIM1->CCMR1 &=  ~TIM_CCMR1_OC2M_0; 
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

    TIM1->CCMR2 &= ~TIM_CCMR2_OC3M_0;
    TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;

    TIM1->CCMR2 &= ~TIM_CCMR2_OC4M_0;
    TIM1->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
    
    // channel 4 of tim1, enable output compare preload enable
    TIM1->CCMR2 |= TIM_CCMR2_OC4PE;

    // enable the four channel outputs
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    // enable tim1
    TIM1->CR1 |= TIM_CR1_CEN;
}

int getrgb(void);

// Helper function for you
// Accept a byte in BCD format and convert it to decimal
uint8_t bcd2dec(uint8_t bcd) {
    // Lower digit
    uint8_t dec = bcd & 0xF;

    // Higher digit
    dec += 10 * (bcd >> 4);
    return dec;
}

void setrgb(int rgb) {
    uint8_t b = bcd2dec(rgb & 0xFF);
    uint8_t g = bcd2dec((rgb >> 8) & 0xFF);
    uint8_t r = bcd2dec((rgb >> 16) & 0xFF);

    // TODO: Assign values to TIM1->CCRx registers
    // Remember these are all percentages
    // Also, LEDs are on when the corresponding PWM output is low
    // so you might want to invert the numbers.
    TIM1->CCR1 = (100 - r) * 24;
    TIM1->CCR2 = (100 - g) * 24;
    TIM1->CCR3 = (100 - b) * 24;
}



//============================================================================
// Lab 4 code
// Add in your functions from previous lab
//============================================================================

// Part 3: Analog-to-digital conversion for a volume level.
// uint32_t volume = 2400;
int volume = 2400;

// Variables for boxcar averaging.
#define BCSIZE 32
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;

void dialer(void);

// Parameters for the wavetable size and expected synthesis rate.
#define N 1000
#define RATE 20000
short int wavetable[N];
int step0 = 0;
int offset0 = 0;
int step1 = 0;
int offset1 = 0;

//============================================================================
// enable_ports()
//============================================================================
void enable_ports(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    GPIOB->MODER |= 0x155555;
    GPIOC->MODER |= 0x5500;
    // Open drain config
    GPIOC->OTYPER |= 0xF0;
    // Pull down high
    GPIOC->PUPDR |= 0x55;
}

//============================================================================
// setup_dma() + enable_dma()
//============================================================================
void setup_dma(void)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;                // enable rcc clock to the DMA controller
    DMA1_Channel5->CCR &= !DMA_CCR_EN;               // clear enable bit
    DMA1_Channel5->CMAR = (uint32_t)msg;             // src address
    DMA1_Channel5->CPAR = (uint32_t)(&(GPIOB->ODR)); // dest address
    DMA1_Channel5->CNDTR = 0x8;                      // number of data
    DMA1_Channel5->CCR |= DMA_CCR_DIR;               // Direction for copying from memory to peripheral
    DMA1_Channel5->CCR |= DMA_CCR_MINC;              // Memory incrememnt enable
    DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0;           // Memory size 16 bit
    DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;           // Periperal size 16 bit
    DMA1_Channel5->CCR |= DMA_CCR_CIRC;              // tells the transfer to keep going after completion
}

void enable_dma(void)
{
    // NVIC->ISER[0] = 1 << DMA1_Channel4_5_IRQn;
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}


//============================================================================
// init_tim15()
//============================================================================
void init_tim15(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; // enable clock for tim15
    TIM15->DIER |= TIM_DIER_UDE;         // UDE triggers DMA requests on interrrupts
    TIM15->PSC = 48 - 1;                 // prescales clock to 1MHz
    TIM15->ARR = 1000 - 1;               // scales clock to 1 KHz
    TIM15->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] |= 1 << TIM15_IRQn;
}


//=============================================================================
// Part 2: Debounced keypad scanning.
//=============================================================================

uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()

//============================================================================
// The Timer 7 ISR
//============================================================================
void TIM7_IRQHandler(void)
{
    TIM7->SR &= ~TIM_SR_UIF; // acknowledge interrupt
    int rows = read_rows();
    update_history(col, rows);
    col = (col + 1) & 3;
    drive_column(col);
}

//============================================================================
// init_tim7()
//============================================================================
void init_tim7(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 48 - 1;
    TIM7->ARR = 1000 - 1;
    TIM7->CR1 |= TIM_CR1_CEN;
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] |= 1 << TIM7_IRQn;
}


//============================================================================
// setup_adc()
//============================================================================

void setup_adc(void)
{
    // GPIO config
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= 0xC; // 11 for analog

    RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // enable clock to the ADC peripheral
    RCC->CR2 |= RCC_CR2_HSI14ON;       // turn on 14MHz clock
    while (!(RCC->CR2 & RCC_CR2_HSI14RDY))
        ;                    // wait for 14MHz clock to be ready
    ADC1->CR |= ADC_CR_ADEN; // enable ADC
    while (!(ADC1->ISR & ADC_ISR_ADRDY))
        ;             // wait for ADC to be ready
    // ADC1->CHSELR = 0; // deactivate the other channels
    ADC1->CHSELR |= ADC_CHSELR_CHSEL1;
    while (!(ADC1->ISR & ADC_ISR_ADRDY))
        ;
}

//============================================================================
// Timer 2 ISR
//============================================================================
// Write the Timer 2 ISR here.  Be sure to give it the right name.
void TIM2_IRQHandler(void)
{
    TIM2->SR &= ~TIM_SR_UIF; // acknowledge interrupt
    ADC1->CR |= ADC_CR_ADSTART;
    while(!(ADC1->ISR & ADC_ISR_EOC));
    bcsum -= boxcar[bcn];
    // boxcar[bcn] = ADC1->DR;
    bcsum += boxcar[bcn] = ADC1->DR;
    bcn += 1;
    if (bcn >= BCSIZE)
    {
        bcn = 0;
    }
    volume = bcsum / BCSIZE;
}

//============================================================================
// init_tim2()
//============================================================================
void init_tim2(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 48 - 1;
    TIM2->ARR = 100000 - 1;
    TIM2->CR1 |= TIM_CR1_CEN;        // enable counter
    TIM2->DIER |= TIM_DIER_UIE;      // enable interuptEN
    NVIC->ISER[0] |= 1 << TIM2_IRQn; // enable interrupt in NVIC
    NVIC_SetPriority(TIM2_IRQn, 3);
}



//============================================================================
// setup_dac()
//============================================================================
void setup_dac(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= 0x300;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    DAC1->CR &= ~0x38;
    DAC1->CR |= DAC_CR_TEN1;
    DAC1->CR |= DAC_CR_EN1;
}

//============================================================================
// Timer 6 ISR
//============================================================================
// Write the Timer 6 ISR here.  Be sure to give it the right name.

void TIM6_DAC_IRQHandler(void)
{
    TIM6->SR &= ~TIM_SR_UIF;
    offset0 += step0;
    offset1 += step1;
    if (offset0 >= (N << 16))
    {
        offset0 -= (N << 16);
    }
    if (offset1 >= (N << 16))
    {
        offset1 -= (N << 16);
    }
    int samp = wavetable[offset0 >> 16] + wavetable[offset1 >> 16];
    // samp *= volume;
    // samp = samp >> 17;
    // samp += 2048;
    samp = ((samp * volume)>>18) + 1200; 
    // DAC->DHR12R1 = samp;
    TIM1->CCR4 = samp;
}
//============================================================================
// init_tim6()
//============================================================================
void init_tim6(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 48 - 1;
    TIM6->ARR = (1000000 / RATE) - 1;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
    // TIM6->CR2 |= TIM_CR2_MMS_1; 
    NVIC->ISER[0] |= 1 << TIM6_DAC_IRQn;
}

//===========================================================================
// init_wavetable()
// Write the pattern for a complete cycle of a sine wave into the
// wavetable[] array.
//===========================================================================
void init_wavetable(void)
{
    for (int i = 0; i < N; i++)
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
}



//============================================================================
// set_freq()
//============================================================================

void set_freq(int chan, float f)
{
    if (chan == 0)
    {
        if (f == 0.0)
        {
            step0 = 0;
            offset0 = 0;
        }
        else
            step0 = (f * N / RATE) * (1 << 16);
    }
    if (chan == 1)
    {
        if (f == 0.0)
        {
            step1 = 0;
            offset1 = 0;
        }
        else
            step1 = (f * N / RATE) * (1 << 16);
    }
}

//============================================================================
// All the things you need to test your subroutines.
//============================================================================
int main(void) {
    internal_clock();

    // Uncomment autotest to get the confirmation code.
    // autotest();

    // Uncomment only one of MOTOR_PID_DEMO / TEST_UART_PWM / TEST_TIMER3
    // (each loops forever).

    // Closed-loop PID speed control of two AndyMark PG27 gearmotors through
    // the Cytron MDD20A: channel 1 (PWM1=PA6, DIR1=PB0, encoder A=PA15, B=PB3)
    // and channel 2 (PWM2=PA7, DIR2=PB2, encoder A=PA8, B=PA9). Host interface
    // over the host UART (USART2 on the PCB), see motor_pid_uart.py.
#define MOTOR_PID_DEMO
#ifdef MOTOR_PID_DEMO
    uart5_init();
    setup_motor_control();
    setup_servo();
    motor_uart_loop();
#endif

    // PC9 duty cycle set from the host over USART5 (see pwm_uart.py).
// #define TEST_UART_PWM
#ifdef TEST_UART_PWM
    uart5_init();
    setup_tim3();
    pwm_uart_loop();
#endif

    // Demonstrate part 1
// #define TEST_TIMER3
#ifdef TEST_TIMER3
    setup_tim3();
    for(;;) { }
#endif

    // Initialize the display to something interesting to get started.
    msg[0] |= font['E'];
    msg[1] |= font['C'];
    msg[2] |= font['E'];
    msg[3] |= font[' '];
    msg[4] |= font['3'];
    msg[5] |= font['6'];
    msg[6] |= font['2'];
    msg[7] |= font[' '];

    enable_ports();
    setup_dma();
    enable_dma();
    init_tim15();
    init_tim7();
    setup_adc();
    init_tim2();
    init_wavetable();
    init_tim6();

    setup_tim1();
    setrgb(0x001500);

    // demonstrate part 2
#define TEST_TIM1
#ifdef TEST_TIM1
    for(;;) {
        // Breathe in...
        for(float x=1; x<2400; x *= 1.1) {
            TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 2400-x;
            nano_wait(100000000);
        }
        // ...and out...
        for(float x=2400; x>=1; x /= 1.1) {
            TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 2400-x;
            nano_wait(100000000);
        }
        // ...and start over.
    }
#endif

    // demonstrate part 3
// #define MIX_TONES
// #ifdef MIX_TONES
//     set_freq(0, 1000);
//     for(;;) {
//         char key = get_keypress();
//         if (key == 'A')
//             set_freq(0,getfloat());
//         if (key == 'B')
//             set_freq(1,getfloat());
//     }
// #endif

    // demonstrate part 4
// #define TEST_SETRGB
// #ifdef TEST_SETRGB
//     for(;;) {
//         char key = get_keypress();
//         if (key == 'A')
//             set_freq(0,getfloat());
//         if (key == 'B')
//             set_freq(1,getfloat());
//         if (key == 'D')
//             setrgb(getrgb());
//     }
// #endif

    // Have fun.
    dialer();
}
