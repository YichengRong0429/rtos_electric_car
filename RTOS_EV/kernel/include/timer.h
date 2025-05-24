#ifndef _TIMER_H_
#define _TIMER_H_

#define TIM2_INT_NUM 28
#define TIM3_INT_NUM 29
#define TIM4_INT_NUM 30
#define TIM5_INT_NUM 50

#define TIMUG (1)  // update generation
#define TIMCEN (1) // counter enable
#define TIMUIE (1) // update interrupt enable
#define TIMUIF (1) // update interrupt flag

#define TIM5EN (1 << 3)
#define TIM4EN (1 << 2)
#define TIM3EN (1 << 1)
#define TIM2EN (1)

#include <unistd.h>

/*
 * Starts the timer
 *
 * @param timer      - The timer
 * @param prescaler  - Prescalar for clock
 * @param Period     - Period of the timer interrupt
 */
void timer_init(int timer, uint32_t prescalar, uint32_t period);

/*
 * Stops the timer
 *
 * @param timer      - The timer
 */
void timer_disable(int timer);

/*
 * Clears the timer interrupt bit
 *
 * @param timer      - The timer
 */
void timer_clear_interrupt_bit(int timer);

#endif /* _TIMER_H_ */