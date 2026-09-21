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

void uart5_init(void) {
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

    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
    USART5->CR1 &= ~USART_CR1_UE;
    USART5->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1 | USART_CR1_PCE | USART_CR1_OVER8);
    USART5->CR2 &= ~USART_CR2_STOP;
    USART5->BRR = 0x1A1; // 48 MHz / 115200 = 417
    USART5->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART5->CR1 |= USART_CR1_UE;
    while ((USART5->ISR & (USART_ISR_TEACK | USART_ISR_REACK)) != (USART_ISR_TEACK | USART_ISR_REACK))
        ;
}

static void uart5_putc(char c) {
    while (!(USART5->ISR & USART_ISR_TXE))
        ;
    USART5->TDR = c;
}

static void uart5_puts(const char *s) {
    while (*s)
        uart5_putc(*s++);
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
        if (USART5->ISR & USART_ISR_ORE)
            USART5->ICR = USART_ICR_ORECF;
        if (!(USART5->ISR & USART_ISR_RXNE))
            continue;

        char c = USART5->RDR;
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

    // Uncomment only one of TEST_UART_PWM / TEST_TIMER3 (each loops forever).
    // PC9 duty cycle set from the host over USART5 (see pwm_uart.py).
#define TEST_UART_PWM
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
