#include <printk.h>
#include <pwm.h>
#include <rcc.h>
#include <unistd.h>
#include <nvic.h>
#include "timer.h"

/** @brief TIM1 register map */
struct tim1 {
    volatile uint32_t cr1;     /**< 00 Control Register 1 */
    volatile uint32_t cr2;     /**< 04 Control Register 2 */
    volatile uint32_t smcr;    /**< 08 Slave Mode Control */
    volatile uint32_t dier;    /**<*< 0C DMA/Interrupt Enable */
    volatile uint32_t sr;      /**< 10 Status Register */
    volatile uint32_t egr;     /**< 14 Event Generation */
    volatile uint32_t ccmr[2]; /**< 18-1C Capture/Compare Mode */
    volatile uint32_t ccer;    /**< 20 Capture/Compare Enable */
    volatile uint32_t cnt;     /**< 24 Counter Register */
    volatile uint32_t psc;     /**< 28 Prescaler Register */
    volatile uint32_t arr;     /**< 2C Auto-Reload Register */
    volatile uint32_t rcr;     /**< 30 Repetition Counter Register */
    volatile uint32_t ccr[4];  /**< 34-40 Capture/Compare */
    volatile uint32_t bdtr;    /**< 44 Break and Dead-Time Register */
    volatile uint32_t dcr;     /**< 48 DMA Control Register */
    volatile uint32_t dmar;    /**< 4C DMA address for full transfer Register */
};

/** @brief TIM2-5 register map */
struct tim2_5 {
    volatile uint32_t cr1;        /**< 00 Control Register 1 */
    volatile uint32_t cr2;        /**< 04 Control Register 2 */
    volatile uint32_t smcr;       /**< 08 Slave Mode Control */
    volatile uint32_t dier;       /**< 0C DMA/Interrupt Enable */
    volatile uint32_t sr;         /**< 10 Status Register */
    volatile uint32_t egr;        /**< 14 Event Generation */
    volatile uint32_t ccmr[2];    /**< 18-1C Capture/Compare Mode */
    volatile uint32_t ccer;       /**< 20 Capture/Compare Enable */
    volatile uint32_t cnt;        /**< 24 Counter Register */
    volatile uint32_t psc;        /**< 28 Prescaler Register */
    volatile uint32_t arr;        /**< 2C Auto-Reload Register */
    volatile uint32_t reserved_1; /**< 30 */
    volatile uint32_t ccr[4];     /**< 34-40 Capture/Compare */
    volatile uint32_t reserved_2; /**< 44 */
    volatile uint32_t dcr;        /**< 48 DMA Control Register */
    volatile uint32_t dmar; /**< 4C DMA address for full transfer Register */
    volatile uint32_t or ;  /**< 50 Option Register */
};

uint8_t IS_COMP = 0;

#define TIM1_BASE (struct tim1 *) 0x40010000 // TIMER 1
#define TIM1EN (1)
#define TIM_ARPE (1 << 7) // Auto-reload preload enable

// 0b110 -- PWM Mode 1: active until duty cycle ends, then inactive for rest of period
#define TIM_OC1M (0b110 << 4)   // Output compare 1 mode
#define TIM_OC2M (0b110 << 12)  // Output compare 2 mode
#define TIM_OC3M (0b110 << 4)   // Output compare 3 mode
#define TIM_OC4M (0b110 << 12)  // Output compare 4 mode

#define TIM_OC1PE (1 << 3)      // Output compare 1 preload enable
#define TIM_OC2PE (1 << 11)     // Output compare 2 preload enable
#define TIM_OC3PE (1 << 3)      // Output compare 3 preload enable
#define TIM_OC4PE (1 << 11)     // Output compare 4 preload enable

#define TIM_CC2E (1 << 4)       // Capture/Compare 2 output enable
#define TIM_CC3E (1 << 8)       // Capture/Compare 3 output enable
#define TIM_CC4E (1 << 12)       // Capture/Compare 4 output enable

struct tim2_5* const timer_base_pwm[] = {(void *)0x0,   // N/A - Don't fill out
                                     (void *)0x0,       // N/A - Don't fill out
                                     (void *)0x40000000,    // TIMER 2
                                     (void *)0x40000400,    // TIMER 3
                                     (void *)0x40000800,    // TIMER 4
                                     (void *)0x40000C00};   // TIMER 5

void timer_start_pwm(uint32_t period, uint32_t duty_cycle, uint32_t timer, uint32_t channel) {
    struct tim2_5 *tim = timer_base_pwm[timer];
    
    // stolen from timer_init (timer.c)
    struct rcc_reg_map *rcc = RCC_BASE;
    // enable timer in RCC
    switch (timer) {
        case 2:
            rcc->apb1_enr |= TIM2EN;
            // nvic_irq(TIM2_INT_NUM, IRQ_ENABLE);
            break;
        case 3:
            rcc->apb1_enr |= TIM3EN;
            // nvic_irq(TIM3_INT_NUM, IRQ_ENABLE);
            break;
        case 4:
            rcc->apb1_enr |= TIM4EN;
            // nvic_irq(TIM4_INT_NUM, IRQ_ENABLE);
            break;
        case 5:
            rcc->apb1_enr |= TIM5EN;
            // nvic_irq(TIM5_INT_NUM, IRQ_ENABLE);
            break;
    }

    // put period into the auto-reload register
    tim->arr = period - 1;
    // enable update events by setting UG bit in the TIMx_EGR register
    tim->egr |= TIMUG;
    // set counter enable bit in TIMx_CR1 register
    tim->cr1 |= TIMCEN;
    // update interrupt enable
    tim->dier |= TIMUIE;

    // enable auto-reload preload -- not required
    tim->cr1 |= TIM_ARPE;
    switch (channel) {
        case 1:
            // set output compare 1 mode to PWM mode 1: upcounting
            tim->ccmr[0] |= TIM_OC1M;
            // enable preload register on TIMx_CCR1. I think this allows dynamically changing duty cycle.
            tim->ccmr[0] |= TIM_OC1PE;
            // set duty cycle for channel 1... lower 16 bits is low capture/compare 1 value
            tim->ccr[0] = duty_cycle;
            break;
        case 2:
            tim->ccmr[0] |= TIM_OC2M;
            tim->ccmr[0] |= TIM_OC2PE;
            tim->ccer |= TIM_CC2E;
            tim->ccr[1] = duty_cycle;
            break;
        case 3:
            tim->ccmr[1] |= TIM_OC3M;
            tim->ccmr[1] |= TIM_OC3PE;
            tim->ccer |= TIM_CC3E;
            tim->ccr[2] = duty_cycle;
            break;
        case 4:
            tim->ccmr[1] |= TIM_OC4M;
            tim->ccmr[1] |= TIM_OC4PE;
            tim->ccer |= TIM_CC4E;
            tim->ccr[3] = duty_cycle;
            break;
    }
    /*
    Registers
    TIMx_ARR -- determines frequency
    TIMx_CR1 -- ARPE bit: auto-reload preload register
    TIMx_CCRx -- determines duty cycle
    TIMx_CCMRx -- OCxM bits (select PWM MODE, WANT MODE 1)
               -- OCxPE bit: enables corresponding preload register

    initialize GPIO pin/channel corresponding to the timer
    see page 45 of stm32f401re.pdf for table of alt function mappings for GPIO pins

    switch (timer) {
        case 2:
            if (channel == 1) gpio_init(GPIO_A, 0, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT1);
            else if (channel == 2) gpio_init(GPIO_A, 1, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT1);
            else if (channel == 3) gpio_init(GPIO_A, 2, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT1);
            else if (channel == 4) gpio_init(GPIO_A, 3, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT1);
            break;
        case 3:
            if (channel == 1) gpio_init(GPIO_B, 6, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT2);
            else if (channel == 2) gpio_init(GPIO_B, 5, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT2);
            else if (channel == 3) gpio_init(GPIO_B, 0, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT2);
            else if (channel == 4) gpio_init(GPIO_B, 1, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT2);
            break;
        case 4:
            if (channel == 1) gpio_init(GPIO_B, 6, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT2);
            else if (channel == 2) gpio_init(GPIO_B, 7, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT2);
            else if (channel == 3) gpio_init(GPIO_B, 8, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT2);
            else if (channel == 4) gpio_init(GPIO_B, 9, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT2);
            break;
        case 5:
            if (channel == 1) gpio_init(GPIO_A, 0, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT2);
            else if (channel == 2) gpio_init(GPIO_A, 1, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT2);
            else if (channel == 3) gpio_init(GPIO_A, 2, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT2);
            else if (channel == 4) gpio_init(GPIO_A, 3, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT2);
            break;
    }
    */

    return;
}

void timer_disable_pwm(uint32_t timer, uint32_t channel) {
    (void)channel;
    timer_disable(timer);
    /*
    struct tim2_5 *tim = timer_base_pwm[timer];
    switch (channel) {
        case 1:
            tim->ccmr[0] &= ~TIM_OC1M; // set output mode to Frozen (0b000)
            break;
        case 2:
            tim->ccmr[0] &= ~TIM_OC2M;
            break;
        case 3:
            tim->ccmr[1] &= ~TIM_OC3M;
            break;
        case 4:
            tim->ccmr[1] &= ~TIM_OC4M;
            break;
    }
    */
    return;
}

void timer_set_duty_cycle(uint32_t timer, uint32_t channel, uint32_t duty_cycle) {
    struct tim2_5 *tim = timer_base_pwm[timer];
    switch (channel) {
        case 1:
            tim->ccr[0] = duty_cycle;
            break;
        case 2:
            tim->ccr[1] = duty_cycle;
            break;
        case 3:
            tim->ccr[2] = duty_cycle;
            break;
        case 4:
            tim->ccr[3] = duty_cycle;
            break;
    }
    return;
}

