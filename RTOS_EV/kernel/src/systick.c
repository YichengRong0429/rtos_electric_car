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
#include <systick.h>
#include <stdbool.h>
#include "arm.h"
#include "syscall.h"

#define UNUSED __attribute__((unused))

/** @brief The STK (SysTick timer) register map. */
struct stk_reg_map {
    volatile uint32_t CTRL;     /**< Control Register */
    volatile uint32_t LOAD;     /**< Reload Value Register */
    volatile uint32_t VAL;  
    volatile uint32_t CALIB;
};

/** @brief Base address for STK (SysTick timer) */
#define STK_BASE (struct stk_reg_map *) 0xE000E010

/** @brief Clock source selection (CTRL) */
#define STK_CLKSOURCE (1 << 2)
/** @brief Systick exception request enable (CTRL) */
#define STK_TICKINT (1 << 1)
/** @brief Counter enable (CTRL) */
#define STK_ENABLE (1 << 0)

/** @brief RELOAD value 
* TODO: check if this is CORRECT. NOT SURE.
* N - 1
* 0.001s = x * (1/16MHz) --> x = 16000 clock cycles for 1ms interrupts
*/
#define STK_RELOAD (16000 - 1)

/** @brief Total elapsed ticks global variable */
uint32_t TOTAL_TICKS = 0;

bool DELAY = false;

void systick_init(uint32_t frequency) {
    struct stk_reg_map *stk = STK_BASE;
    stk->LOAD = (BASE_FREQ / frequency) - 1;     // Sets RELOAD Value

    // configure STK control registers
    stk->CTRL |= STK_CLKSOURCE; // select processor clock as source clock
    stk->CTRL |= STK_TICKINT;   // Countdown to 0 asserts SysTick exception req 
    stk->CTRL |= STK_ENABLE;    // Enables counter

    // Reset total elapsed ticks
    TOTAL_TICKS = 0; 
    return;
}

// When current tick reaches end tick, return
void systick_delay(uint32_t ticks) {
    uint32_t end_ticks = TOTAL_TICKS + ticks;
    while (TOTAL_TICKS < end_ticks) ;
    return;
}

uint32_t systick_get_ticks() {
    return TOTAL_TICKS;
}
