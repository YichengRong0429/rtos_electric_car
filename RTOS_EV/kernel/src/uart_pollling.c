#include <unistd.h>

#include <gpio.h>
#include <rcc.h>
#include <uart_polling.h>

/** @brief The UART register map. */
struct uart_reg_map {
    volatile uint32_t SR;   /**< Status Register */
    volatile uint32_t DR;   /**<  Data Register */
    volatile uint32_t BRR;  /**<  Baud Rate Register */
    volatile uint32_t CR1;  /**<  Control Register 1 */
    volatile uint32_t CR2;  /**<  Control Register 2 */
    volatile uint32_t CR3;  /**<  Control Register 3 */
    volatile uint32_t GTPR; /**<  Guard Time and Prescaler Register */
};

/** @brief Base address for UART2 */
#define UART2_BASE  (struct uart_reg_map *) 0x40004400

/** @brief Enable  Bit for UART Config register */
#define UART_EN (1 << 13)

#define USARTDIV 0x008B // =8.6875, perfect would be 8.6806

/** @brief Enable Bit for UART clock enable in RCC_APB1ENR register*/
#define UART_CLK_EN (1 << 17)

/** @brief Enable Bits for UART transmitter/receiver */
#define UART_TE (1 << 3)
#define UART_RE (1 << 2)

/** @brief Read-only bit for Transmit data register empty */
#define UART_TXE (1 << 7)
/** @brief Read-only bit for read data register not empty */
#define UART_RXNE (1 << 5)

/**
 * @brief initializes UART to given baud rate with 8-bit word length, 1 stop bit, 0 parity bits
 *
 * @param baud Baud rate
 */
void uart_polling_init (int baud){
    (void) baud; /* This line is simply here to suppress the Unused Variable Error. */
                 /* You should remove this line in your final implementation */

    struct uart_reg_map *uart = UART2_BASE;

    // TODO: Check if ALT7 is correct.
    // Initialize GPIO pins for UART
    // UART2_TX
    gpio_init(GPIO_A, 2, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT7);
    // UART2_RX
    gpio_init(GPIO_A, 3, MODE_ALT, OUTPUT_OPEN_DRAIN, OUTPUT_SPEED_LOW, PUPD_NONE, ALT7);

    // enable peripheral clock for UART
    struct rcc_reg_map *rcc = RCC_BASE;
    rcc->apb1_enr |= UART_CLK_EN;

    // set the baud rate
    uart->BRR = USARTDIV;

    // configure UART control registers
    uart->CR1 |= UART_TE; // transmitter enable
    uart->CR1 |= UART_RE; // receiver enable
    uart->CR1 |= UART_EN;

    return;
}

/**
 * @brief transmits a byte over UART
 *
 * @param c character to be sent
 */
void uart_polling_put_byte (char c){
    struct uart_reg_map *uart = UART2_BASE;
    //TX
    // wait until transmit data register is empty
    while (!(uart->SR & UART_TXE)) {}
    // write byte into data register
    uart->DR = c;

    return;
}

/**
 * @brief receives a byte over UART
 */
char uart_polling_get_byte () {
    struct uart_reg_map *uart = UART2_BASE;
    //RX
    // wait until read data register is not empty
    while (!(uart->SR & UART_RXNE)) {}
    // read byte into c variable
    char c = uart->DR;

    return c;
}
