/**
 * @file 
 *
 * @brief      
 *
 * @date       
 *
 * @author     
 */

#include <gpio.h>
#include <unistd.h>
#include <rcc.h>
#include <uart.h>
#include <uart_polling.h>
#include <nvic.h>
#include "assert.h"
#include <arm.h>

/*
Implementing a circular buffer for RX and TX
Use global variables for RX and TX
start should always be size greater than end
  - aka, start = (end + size) % len
  - buffer is empty when size = 0 (and start == end)
  - buffer is full when size = len (and start == end)
*/
struct buffer {
    volatile char buf[16]; // 16 byte long buffer
    volatile uint32_t start; // index where we put
    volatile uint32_t end; // index where we get
    volatile uint32_t len; // max bytes that can fit
    volatile uint32_t size; // current number of bytes stored in buf
};
struct buffer rxbuf;
struct buffer txbuf;

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

/** @brief TXE interrupt enable */
#define UART_TXEIE (1 << 7)
/** @brief TC interrupt enable... is this needed when we have TXE? */
// #define UART_TCIE (1 << 6)
/** @brief RXNE interrupt enable */
#define UART_RXNEIE (1 << 5)

#define IRQ_NUM (38)
#define BUFLEN (16)

#define UNUSED __attribute__((unused))

void uart_init(UNUSED int baud){
    struct uart_reg_map *uart = UART2_BASE;
    // Initialize GPIO pins for UART
    // UART2_TX
    gpio_init(GPIO_A, 2, MODE_ALT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW, PUPD_NONE, ALT7);
    // UART2_RX
    gpio_init(GPIO_A, 3, MODE_ALT, OUTPUT_OPEN_DRAIN, OUTPUT_SPEED_LOW, PUPD_NONE, ALT7);

    // Initialize RX and TX buffers
    rxbuf.start = 0;
    rxbuf.end = 0;
    rxbuf.len = BUFLEN;
    rxbuf.size = 0;

    txbuf.start = 0;
    txbuf.end = 0;
    txbuf.len = BUFLEN;
    txbuf.size = 0;

    // enable peripheral clock for UART
    struct rcc_reg_map *rcc = RCC_BASE;
    rcc->apb1_enr |= UART_CLK_EN;

    // set the baud rate
    uart->BRR = USARTDIV;

    // Enable corresponding interrupt in Nested Vector Interrupt Controller (NVIC)
    // IRQ38 from vector table matches with USART2
    nvic_irq(IRQ_NUM, IRQ_ENABLE);

    // Enable interrupts within UART control registers
    // Tell UART peripheral to generate interrupts
    uart->CR1 |= UART_RXNEIE; // RXNE=1 interrupt

    // configure UART control registers
    uart->CR1 |= UART_TE; // transmitter enable
    uart->CR1 |= UART_RE; // receiver enable
    uart->CR1 |= UART_EN;
    return;
}

// Returns 0 on success, -1 on failure (buffer is full)
int uart_put_byte(char c){
  int state = save_interrupt_state_and_disable();
  struct uart_reg_map *uart = UART2_BASE;
  // enable TXEIE when numElems(txbuf): 0 --> 1
  if (txbuf.size < txbuf.len) {

    txbuf.buf[txbuf.start] = c;
    txbuf.size++;
    txbuf.start = (txbuf.start + 1) % txbuf.len;

    if (txbuf.size == 1) {
      uart->CR1 |= UART_TXEIE; 
    }
    // uart->CR1 |= UART_TXEIE; // TXE=1 interrupt
    restore_interrupt_state(state);
    return 0;
  }
  restore_interrupt_state(state);
  return -1; // buffer is full
}

// Returns 0 on success, -1 on failure (no bytes in buffer)
int uart_get_byte(char *c){
  int state = save_interrupt_state_and_disable();
  struct uart_reg_map *uart = UART2_BASE;
  if (rxbuf.size > 0) {
    *c = rxbuf.buf[rxbuf.end];
    rxbuf.size--;
    rxbuf.end = (rxbuf.end + 1) % rxbuf.len;

    // enable RXNEIE when numElems(rxbuf): full -> not full
    if (rxbuf.size == rxbuf.len - 1) {
      uart->CR1 |= UART_RXNEIE;
    }
    restore_interrupt_state(state);
    return 0;
  }
  restore_interrupt_state(state);
  return -1; // no bytes in buffer
}

char buf_deq(struct buffer *b) {
  char byte = b->buf[b->end];
  b->end = (b->end + 1) % b->len;
  b->size--;
  return byte;
}

void buf_enq(struct buffer *b, char byte) {
  b->buf[b->start] = byte;
  b->start = (b->start + 1) % b->len;
  b->size++;
}

void uart_irq_handler(){
  int state = save_interrupt_state_and_disable();
  struct uart_reg_map *uart = UART2_BASE;
  // check status register to determine whether I can transmit or receive

  // read the status register ONCE
  uint32_t status = uart->SR & (UART_TXE | UART_RXNE);

  if (status & UART_TXE) { // DR empty & transmit buffer nonempty -> can transmit
    assert(txbuf.size > 0);
    // Take out byte at txbuf.end, increment end, decrease size
    // Place byte into uart dataregister
    char byte = buf_deq(&txbuf);
    uart->DR = byte;
    // disable TXEIE when numElems(txbuf): 1 --> 0
    if (txbuf.size == 0) {
      uart->CR1 &= ~UART_TXEIE;
    }
  }

  // DR not empty & rx buffer not full --> can read from DR and put into receive buf
  if (status & UART_RXNE) { 
    assert(rxbuf.size < rxbuf.len);
    // take out byte from DR
    // put byte into rxbuf at rxbuf.start, increment start + size
    char byte = uart->DR & 0xFF;
    buf_enq(&rxbuf, byte);
    // disable RXNEIE when numElems(rxbuf) not full --> full
    if (rxbuf.size == rxbuf.len) {
      uart->CR1 &= ~UART_RXNEIE; 
    }
  }

  restore_interrupt_state(state);
  nvic_clear_pending(IRQ_NUM);
}

// Not required
void uart_flush()
{
  struct uart_reg_map *uart = UART2_BASE;
  if (txbuf.size > 0) {
    uart->CR1 &= ~UART_TXEIE;
  }
  while (txbuf.size != 0) 
  { 
    // DR empty & transmit buffer nonempty -> can transmit
    // Take out byte at txbuf.end, increment end, decrease size
    // Place byte into uart dataregister
    while(!(uart->SR & UART_TXE));
    char byte = buf_deq(&txbuf);
    uart->DR = byte;
  }
}