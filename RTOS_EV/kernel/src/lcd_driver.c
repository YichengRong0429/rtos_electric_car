#include <unistd.h>
#include <i2c.h>
#include <lcd_driver.h>
#include <systick.h>

#define LCD_Address 0x20 //LCD slave address
#define LCD_RS (1<<0) //LCD_RS Bit
#define LCD_RW (1<<1) //LCD_RW Bit
#define LCD_E (1<<2) //LCD_E Bit
#define LCD_BackLight (1<<3) //LCD_BackLight bit
#define Cursor_bit (1<<7) //Cursor_bit instruction header bit


void lcd_driver_init() 
{
    uint8_t buf[4];
    uint8_t instruction = LCD_BackLight;
    // 1. wait for more than 15ms
    // for(int i=0;i<180000;i++);
    systick_delay(50); // Delay 50ms
    // 2. send b0011 0000 (send three times)
    for (int i=0;i<3;i++)
    {
        buf[0]=(uint8_t)((0b0011<<4) | instruction | LCD_E); //1100
        buf[1]=(uint8_t)(0b0011<<4) | instruction;//1000
        buf[2]=(0b0000<<4) | instruction | LCD_E;
        buf[3]=(0b0000<<4) | instruction;
        i2c_master_start();
        i2c_master_write(buf, 4, LCD_Address);
        i2c_master_stop();
        // for(int j=0;j<50000;j++); //5ms
        systick_delay(5); // Delay 5ms
    }
    // 3. send 0b0010 1000 
    buf[0]=(0b0010<<4) | instruction | LCD_E;
    buf[1]=(0b0010<<4) | instruction;
    buf[2]=(0b0000<<4) | instruction | LCD_E; //data_1100
    buf[3]=(0b0000<<4) | instruction;
    i2c_master_start();
    i2c_master_write(buf, 4, LCD_Address);
    i2c_master_stop();
    // for(int j=0;j<50000;j++);
    systick_delay(5); // Delay 5ms

    //4. send 0b0000 1000
    buf[0]=(0b0010<<4) | instruction | LCD_E;
    buf[1]=(0b0010<<4) | instruction;
    buf[2]=(0b1000<<4) | instruction | LCD_E;//find this out 
    buf[3]=(0b1000<<4) | instruction;
    i2c_master_start();
    i2c_master_write(buf, 4, LCD_Address);
    i2c_master_stop();
    // for(int j=0;j<50000;j++);
    systick_delay(5); // Delay 5ms

    //5. send 0b0000 0001
    buf[0]=(0b0000<<4) | instruction | LCD_E;
    buf[1]=(0b0000<<4) | instruction;
    buf[2]=(0b1000<<4) | instruction | LCD_E;
    buf[3]=(0b1000<<4) | instruction;
    i2c_master_start();
    i2c_master_write(buf, 4, LCD_Address);
    i2c_master_stop();
    // for(int j=0;j<50000;j++);
    systick_delay(5); // Delay 5ms

    //6. send 0b0000 0100
    buf[0]=(0b0000<<4) | instruction | LCD_E;
    buf[1]=(0b0000<<4) | instruction;
    buf[2]=(0b0001<<4) | instruction | LCD_E;
    buf[3]=(0b0001<<4) | instruction;
    i2c_master_start();
    i2c_master_write(buf, 4, LCD_Address);
    i2c_master_stop();
    // for(int j=0;j<50000;j++);
    systick_delay(5); // Delay 5ms

    //7. send 0b0000 0100
    buf[0]=(0b0000<<4) | instruction | LCD_E;
    buf[1]=(0b0000<<4) | instruction;
    buf[2]=(0b0110<<4) | instruction | LCD_E;
    buf[3]=(0b0110<<4) | instruction;
    i2c_master_start();
    i2c_master_write(buf, 4, LCD_Address);
    i2c_master_stop();
    // for(int j=0;j<50000;j++);
    systick_delay(5); // Delay 5ms

    //8. turn on 0b0000 1111
    buf[0]=(0b0000<<4) | instruction | LCD_E;
    buf[1]=(0b0000<<4) | instruction;
    buf[2]=(0b1111<<4) | instruction | LCD_E;
    buf[3]=(0b1111<<4) | instruction;
    i2c_master_start();
    i2c_master_write(buf, 4, LCD_Address);
    i2c_master_stop();
    // for(int j=0;j<50000;j++){}
    systick_delay(5); // Delay 5ms
    
	return;
}

void lcd_print(char *input)
{    
    uint8_t buf[4];
    uint8_t instruction = LCD_BackLight;
    // for(int j=0;j<50000;j++){}
    systick_delay(5); // Delay 5ms

    
    while(*input)
    {
        buf[0] = (*input & 0xF0) | instruction | LCD_E | LCD_RS; //1_E_RW_RS
        buf[1] = (*input & 0xF0) | instruction| LCD_RS;
        buf[2] = (*input & 0x0F)<<4 | instruction | LCD_E | LCD_RS;
        buf[3] = (*input & 0x0F)<<4 | instruction| LCD_RS;
        i2c_master_start();
        i2c_master_write(buf, 4, LCD_Address);
        i2c_master_stop();
        // for(int j=0;j<50000;j++){}
        systick_delay(5); // Delay 5ms

        input = &input[1];
    }
    // for(int i=0;i<180000;i++);
    systick_delay(50); // Delay 50ms
    
    return;
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t buf[4];
    uint8_t instruction = LCD_BackLight;
    uint8_t address;

    // Set the correct address based on row and column
    if (row == 0) {

        address = 0x00 + col;
        address = Cursor_bit|address;
    } else if (row == 1) {
        address = 0x40 + col;
        address = Cursor_bit|address;
    }

    // Send the command to set DDRAM address
    buf[0] = (address & 0xF0) | instruction | LCD_E;  
    buf[1] = (address & 0xF0) | instruction;
    buf[2] = (address & 0x0F)<<4 | instruction | LCD_E;  
    buf[3] = (address & 0x0F)<<4 | instruction;
    i2c_master_start();
    i2c_master_write(buf, 4, LCD_Address);
    i2c_master_stop();
    // for(int i=0;i<180000;i++);
    systick_delay(50); // Delay 50ms
}

void lcd_clear() {
    uint8_t buf[4];
    uint8_t control = LCD_BackLight;
    buf[0]=(0b0000<<4) | control | LCD_E;
    buf[1]=(0b0000<<4) | control;
    buf[2]=(0b0001<<4) | control | LCD_E;
    buf[3]=(0b0001<<4) | control;
    // Send the command to set DDRAM address to clear the screen
    i2c_master_start();
    i2c_master_write(buf, 4, LCD_Address);
    i2c_master_stop();
    // for(int i=0;i<100000;i++){};
    systick_delay(2000); // Delay 2s = 2000ms
	return;
}
