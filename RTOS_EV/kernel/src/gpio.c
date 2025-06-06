#include <gpio.h>
#include <rcc.h>
#include <stdint.h>

#define BITS_PER_ALT      4
#define BITS_PER_MODE     2
#define BITS_PER_SPEED    2
#define BITS_PER_PUPD     2
#define BITS_PER_TYPE     1
#define GPIOS_PER_ALT_REG 8

/** @brief GPIO Registers - A through G */
typedef struct {
    volatile uint32_t mode;    /**< 0 - Mode */
    volatile uint32_t o_type;  /**< 4 - Output Type */
    volatile uint32_t o_speed; /**< 8 - Output Speed */
    volatile uint32_t pu_pd;   /**< C - Pull-Up/Pull-Down */
    volatile uint32_t idr;     /**< 10 - Input Data */
    volatile uint32_t odr;     /**< 14 - Output Data*/
    volatile uint32_t bsrr;    /**< 18 - Set/Reset */
    volatile uint32_t lckr;    /**< 1C - Configuration Lock */
    volatile uint32_t afr[2];  /**< 20 - Alternate Function Control*/
} gpio_reg;

/* Bitmask to enable IO Port (A - E) */
const int gpio_en[] = {0x01, 0x02, 0x04, 0x08, 0x10};

/* Base addresses of GPIO regs A, B, and C */
gpio_reg *const gpio_regs[] = {(void *)0x40020000, (void *)0x40020400,
                               (void *)0x40020800};

/*
 * gpio_init: GPIO initialization function
 *
 * port  - GPIO_A, GPIO_B, or GPIO_C
 * num - 0 to 15
 * mode - GPIO Port Mode
 * cnf  - GPIO Port Configuration
 */
void gpio_init(gpio_port port, uint32_t num, uint32_t mode, uint32_t otype,
               uint32_t speed, uint32_t pupd, uint32_t alt) {

    struct rcc_reg_map *rcc =
        (struct rcc_reg_map *)RCC_BASE; /* Base address of RCC */
    rcc->ahb1_enr |= gpio_en[port]; /* Enable clock for GPIO[port] (A - G) */

    gpio_reg *gp = gpio_regs[port]; /* Base address of GPIO[port] (A - G) */

    gp->mode &= ~(0x3 << (num * BITS_PER_MODE));
    gp->mode |= (mode << (num * BITS_PER_MODE));

    gp->o_type &= ~(0x1 << (num * BITS_PER_TYPE));
    gp->o_type |= (otype << (num * BITS_PER_TYPE));

    gp->o_speed &= ~(0x3 << (num * BITS_PER_SPEED));
    gp->o_speed |= (speed << (num * BITS_PER_SPEED));

    gp->pu_pd &= ~(0x3 << (num * BITS_PER_PUPD));
    gp->pu_pd |= (pupd << (num * BITS_PER_PUPD));

    int high = num >= GPIOS_PER_ALT_REG; /* Ports 0-7 are in Config Low Reg.
                                     Ports 8-15 are in Config High Reg */

    int shift_num = num % 8;
    gp->afr[high] &= ~(0xF << (shift_num * BITS_PER_ALT));
    gp->afr[high] |= (alt << (shift_num * BITS_PER_ALT));
}

/*
 * gpio_set: Set specified GPIO pin to high.
 */
void gpio_set(gpio_port port, uint32_t num) {
    gpio_regs[port]->bsrr =
        UINT32_C(1) << num; /* Writing to bits 0-15 of BSRR sets the GPIO pin */
}

/*
 * gpio_set: Clear specified GPIO pin to low.
 */
void gpio_clr(gpio_port port, uint32_t num) {
    gpio_regs[port]->bsrr =
        UINT32_C(1)
        << (num + 16); /* Writing to bits 16-31 of BSRR clears the GPIO pin */
}
/*
 * gpio_read: Read all the gpio pins from selected port.
 * Each bit corresponds to its GPIO pin.
 */
uint32_t gpio_read(gpio_port port, uint32_t channel) {
    return ((gpio_regs[port]->idr) >> channel) & 0x1;
}
