#include <unistd.h>

#include <gpio.h>
#include <i2c.h>
#include <rcc.h>

/** @brief The I2C register map. */
struct i2c_reg_map {
    // volatile uint32_t SR;   /**< Status Register */
    volatile uint32_t CR1;      /**< Control Register 1 */
    volatile uint32_t CR2;      /**< Control Register 2 */
    volatile uint32_t OAR1;     /**< Own Address Register 1 */
    volatile uint32_t OAR2;     /**< Own Address Register 2 */
    volatile uint32_t DR;       /**< Data Register */
    volatile uint32_t SR1;      /**< Status Register 1 */
    volatile uint32_t SR2;      /**< Status Register 1 */
    volatile uint32_t CCR;      /**< Clock Control Register */
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
};

/** @brief Base address for I2C */
#define I2C_BASE (struct i2c_reg_map *) 0x40005400

/** @brief Enable Bit for I2C1 clock enable in RCC_APB1ENR register */
#define I2C1_EN (1 << 21)

/** @brief Peripheral Enable bit */
#define I2C_PE (1 << 0)

/** @brief Peripheral clock frequency = APB clk freq in MHz */
#define I2C_FREQ (16)

/** @brief Configure the I2C clock speed to 100kHz 
 * This is based on peripheral clock frequency (16 MHz)
*/
#define I2C_CCR (160)

/** @brief Start Generation -- CR1 */
#define I2C_START (1 << 8)

/** @brief Stop Generation -- CR1 */
#define I2C_STOP (1 << 9)

/** @brief Acknowledge enable -- CR1 */
#define I2C_ACK (1 << 10)

/** @brief Byte transfer finished -- SR1 */
#define I2C_BTF (1 << 2)
/** @brief Address Sent -- SR1 */
#define I2C_ADDR (1 << 1)
/** @brief Start condition generated bit -- SR1 */
#define I2C_SB (1 << 0)

/** @brief Data register empty (transmitters) -- SR1 */
#define I2C_TXE (1 << 7)

/** @brief Data bus busy bit -- SR2 */
#define I2C_BUSY (1 << 1)

/** @brief Master/Slave mode bit -- SR2 */
#define I2C_MSL (1 << 0)


void i2c_master_init(uint16_t clk){
    (void) clk; /* This line is simply here to suppress the Unused Variable Error. */
                /* You should remove this line in your final implementation */

    /*
    D15 = I2C1_SCL = PB_8, ALT4
    D14 = I2C1_SDA = PB_9, ALT4
    Initialize GPIO pins for I2C

    No need for internal pull-up resistors b/c external is common practice
    Thus, use PUPD_NONE
    Init SCL before SDA
    */
    gpio_init(GPIO_B, 8, MODE_ALT, OUTPUT_OPEN_DRAIN, OUTPUT_SPEED_LOW, PUPD_NONE, ALT4); // SCL
    gpio_init(GPIO_B, 9, MODE_ALT, OUTPUT_OPEN_DRAIN, OUTPUT_SPEED_LOW, PUPD_NONE, ALT4); // SDA

    // enable peripheral clock for I2C
    struct rcc_reg_map *rcc = RCC_BASE;
    rcc->apb1_enr |= I2C1_EN;

    // Enter Peripheral Clock Frequency into CR2, bits FREQ[5:0]
    // "Must be configured with the APB clock frequency value"
    // APB defaults to 16MHz
    struct i2c_reg_map *i2c = I2C_BASE;
    i2c->CR2 = I2C_FREQ;

    // Configure the I2C clock speed to 100kHz
    // Performed similarly to UART baud rate
    // FREQ = 16 --> T_PCLK1 = 1 / (16 * 10^6) = 0.0000000625 = 62.5ns
    // NOTE: "CCR register must be configured only when I2C is disabled (PE=0)"
    while ((i2c->CR1 & I2C_PE) != 0) ;
    i2c->CCR = I2C_CCR;

    // enable peripheral clock for I2C?
    // programmed in the I2C_CR2 register to generate correct timings
    // TODO: not sure if this is necessary.
    i2c->CR1 |= I2C_PE;

    // Use 7-bit addressing
    // I think this is already configured with default reset values -- see page 497
    // Configured at i2c->OAR1 bit 0
    // enable ACK in CR1
    i2c->CR1 |= I2C_ACK;
    return;
}

void i2c_master_start() {
    // Send start condition
    struct i2c_reg_map *i2c = I2C_BASE;

    // then send I2C_START bit
    // wait in a while before set start SR2 regsiter for the busy bit
    

    // wait for busy bit in SR to go low
    while ((i2c->SR2 & I2C_BUSY) != 0) ;
    i2c->CR1 |= I2C_START; 

    // wait until SB=1 (EV5)
    while ((i2c->SR1 & I2C_SB) == 0) ;
    return;
}

void i2c_master_stop() {
    // STOP bit is set by software to generate a Stop condition
    // TODO: also reset master mode bit for data bus
    struct i2c_reg_map *i2c = I2C_BASE;
    i2c->CR1 |= I2C_STOP; 
    return;
}

int i2c_master_write(uint8_t *buf, uint16_t len, uint8_t slave_addr)
{
    struct i2c_reg_map *i2c = I2C_BASE;

    // initialize data line to be master mode
    // Bottom bit determines whether writing to or reading from device
    slave_addr = slave_addr << 1;
    slave_addr &= 0xFE; // Write --> LSB = 0

    // write addr to SDA
    // TODO: |= or =? top 8 bits must be kept at 0.
    i2c->DR = slave_addr;

    // A=Acknowledge event.
    // if ((i2c->CR1 & I2C_ACK) == 0) return -1; 
    
    // EV6: 
    // wait for ACK
    // TODO: poll until ready bit is good for both SR1 and SR2
    while ((i2c->SR1 & I2C_ADDR) == 0) ; // poll until I2C_ADDR = 1 
    while ((i2c->SR2 & I2C_MSL) == 0) ; // poll until MSL in SR2 = 1 

    // EV8_1: wait for TxE = 1
    while ((i2c->SR1 & I2C_TXE) == 0) ;
    for (int i = 0; i < len; i++) 
    {
        // write Data1 in DR
        i2c->DR = buf[i];

        // EV8: wait for TxE
        while ((i2c->SR1 & I2C_TXE) == 0) ; // STUCK HERE
        // if ACK != true, exit code
        if ((i2c->CR1 & I2C_ACK) == 0) return -1;
    }

    // EV8_2:
    // wait for TxE = 1 and BTF = 1
    while ( ((i2c->SR1 & I2C_BTF) == 0) && ((i2c->SR1 & I2C_TXE) == 0) ) ;
    return 0;
}

int i2c_master_read(uint8_t *buf, uint16_t len, uint8_t slave_addr)
{
    (void) buf;
    (void) len;
    (void) slave_addr;

    return 0;
}
