/**
 * @file
 *
 * @brief
 *
 * @date
 *
 * @author
 */

#include <unistd.h>
#include <timer.h>
#include <rcc.h>
#include <nvic.h>

#define UNUSED __attribute__((unused))

#define TIMUG (1)  // update generation
#define TIMCEN (1) // counter enable
#define TIMUIE (1) // update interrupt enable
#define TIMUIF (1) // update interrupt flag

#define TIM5EN (1 << 3)
#define TIM4EN (1 << 2)
#define TIM3EN (1 << 1)
#define TIM2EN (1)

#define BASEFREQ (16000000)

/** @brief tim2_5 */
struct tim2_5 {
  volatile uint32_t cr1; /**< 00 Control Register 1 */
  volatile uint32_t cr2; /**< 04 Control Register 2 */
  volatile uint32_t smcr; /**< 08 Slave Mode Control */
  volatile uint32_t dier; /**< 0C DMA/Interrupt Enable */
  volatile uint32_t sr; /**< 10 Status Register */
  volatile uint32_t egr; /**< 14 Event Generation */
  volatile uint32_t ccmr[2]; /**< 18-1C Capture/Compare Mode */
  volatile uint32_t ccer; /**< 20 Capture/Compare Enable */
  volatile uint32_t cnt; /**< 24 Counter Register */
  volatile uint32_t psc; /**< 28 Prescaler Register */
  volatile uint32_t arr; /**< 2C Auto-Reload Register */
  volatile uint32_t reserved_1; /**< 30 */
  volatile uint32_t ccr[4]; /**< 34-40 Capture/Compare */
  volatile uint32_t reserved_2; /**< 44 */
  volatile uint32_t dcr; /**< 48 DMA Control Register */
  volatile uint32_t dmar; /**< 4C DMA address for full transfer Register */
  volatile uint32_t or; /**< 50 Option Register */
};

struct tim2_5* const timer_base[] = {(void *)0x0,   // N/A - Don't fill out
                                     (void *)0x0,   // N/A - Don't fill out
                                     (void *)0x40000000,
                                     (void *)0x40000400,
                                     (void *)0x40000800,
                                     (void *)0x40000C00};

// timer -- the timer
// prescaler -- prescalar for clock
// period -- period of the timer interrupt
void timer_init(int timer, uint32_t prescalar, uint32_t period) {
  struct tim2_5 *tim = timer_base[timer];
  struct rcc_reg_map *rcc = RCC_BASE;

  // enable timer in RCC
  switch (timer) {
    case 2:
      rcc->apb1_enr |= TIM2EN;
      nvic_irq(28, IRQ_ENABLE);
      break;
    case 3:
      rcc->apb1_enr |= TIM3EN;
      nvic_irq(29, IRQ_ENABLE);
      break;
    case 4:
      rcc->apb1_enr |= TIM4EN;
      nvic_irq(30, IRQ_ENABLE);
      break;
    case 5:
      rcc->apb1_enr |= TIM5EN;
      nvic_irq(50, IRQ_ENABLE);
      break;
  }

  // take sysclock and divide it by prescalar input value
  // put input into the prescalar register
  tim->psc = prescalar;
  // put period into the auto-reload register
  tim->arr = period - 1;
  // enable update events by setting UG bit in the TIMx_EGR register
  tim->egr |= TIMUG;
  // set counter enable bit in TIMx_CR1 register
  tim->cr1 |= TIMCEN;
  // update interrupt enable
  tim->dier |= TIMUIE;

}

void timer_disable(int timer) {
  // struct tim2_5 *tim = timer_base[timer];
  struct rcc_reg_map *rcc = RCC_BASE;

  // unset the bit in the RCC enable register
  switch (timer) {
    case 2:
      rcc->apb1_enr &= ~TIM2EN;
      break;
    case 3:
      rcc->apb1_enr &= ~TIM3EN;
      break;
    case 4:
      rcc->apb1_enr &= ~TIM4EN;
      break;
    case 5:
      rcc->apb1_enr &= ~TIM5EN;
      break;
  }
}

void timer_clear_interrupt_bit(int timer) {
  struct tim2_5 *tim = timer_base[timer];
  tim->sr &= ~TIMUIF;
}