/**
 * @file     kernel.c
 *
 * @brief    Kernel entry point
 *
 */

#include <stdlib.h>
#include <string.h>
#include <gpio.h>
#include <systick.h>
#include <lcd_driver.h>
#include <keypad_driver.h>


#include <timer.h>
#include <nvic.h>

#include "arm.h"
#include "kernel.h"
#include "timer.h"
#include "i2c.h"
#include "printk.h"
#include "uart_polling.h"
#include "uart.h"
#include "stdint.h"
#include "syscall.h"
#include "stdio.h"
#include "unistd.h"
#include "string.h"
#include "arm.h"
#include "motor_driver.h"
#include "pwm.h"

#define BAUD_RATE_115200 0x8B
#define PRESCALAR_10KHZ (0)
#define SERVO_PERIOD (160)
#define SERVO_TIMER (2)

/* ----- GLOBAL VARIABLES ----- */
int SERVO_CUTOFF = 60; // is set by sys_servo_set, 60 --> 0.6ms
int SERVO_ENABLE = 0; // 0 if servo disabled, 1 if servo enabled
int PERIOD_20MS = 2000;
int SERVO_CTR = 0; // Used with the servo handler: when SERVOTICK reaches SERVO_CUTOFF --> set output low

void servo_handler(void) {
  if (SERVO_ENABLE) { // Only set servo pin output if enabled
    if (SERVO_CTR < SERVO_CUTOFF) // output HIGH to GPIO
      gpio_set(GPIO_A, 0);
    else
      gpio_clr(GPIO_A, 0);

    // increment servo tick counter, check if end of period reached
    SERVO_CTR++;
    if (SERVO_CTR >= PERIOD_20MS) {
      SERVO_CTR = 0;
    }
  }

  // upon exiting an interrupt, clear the update interrupt flag
  timer_clear_interrupt_bit(SERVO_TIMER);
}

int kernel_main(void) {
  init_349(); // DO NOT REMOVE THIS LINE
  uart_init(0);
  /*
  (Josiah's board)
  D9 -- ENCA1 -- PC_7 -- EXTI7
  D6 -- ENCB1 -- PB_10 -- EXTI10
  A0 -- ENCA2 -- PA_0 -- EXTI0
  A1 -- ENCB2 -- PA_1 -- EXTI1

  D2 -- MOTOR_IN1 -- PA_10
  D7 -- MOTOR_IN2 -- PA_8
  D8 -- MOTOR_IN3 -- PA_9
  D11 -- MOTOR_IN4 -- PA_7

  D3 -- MOTOR_EN_A -- PB_3 -- timer 2 / channel 2
  D5 -- MOTOR_EN_B -- PB_4 -- timer 3 / channel 1
  */
  struct pin ENCA1 = { GPIO_C, 7, 23};
  struct pin ENCB1 = { GPIO_B, 10, 40};
  struct encoder_pin_attr ENC1 = { ENCA1, ENCB1 }; // encoder 1
  struct motor_timer motor_timer1 = { 2, 2, ALT1, true };
  struct pin motor_in1 = { GPIO_A, 10, -1}; // Don't need IRQ number for motor pins
  struct pin motor_in2 = { GPIO_A, 8, -1};
  struct pin motor_en1 = { GPIO_B, 3, -1};
  struct motor_attr MOTOR1 = { motor_in1, motor_in2, motor_en1, motor_timer1 };

  // struct pin ENCA2 = { GPIO_A, 0, 6};
  // struct pin ENCB2 = { GPIO_A, 1, 7};
  // struct encoder_pin_attr ENC2 = { ENCA2, ENCB2 }; // encoder 2
  // struct motor_timer motor_timer2 = { 3, 1, ALT2, true };  
  // struct pin motor_in3 = { GPIO_A, 9, -1};
  // struct pin motor_in4 = { GPIO_A, 7, -1};
  // struct pin motor_en2 = { GPIO_B, 4, -1};
  // struct motor_attr MOTOR2 = { motor_in3, motor_in4, motor_en2, motor_timer2 };

  // initialize motors
  motor_init(RIGHT_MOTOR, &MOTOR1, &ENC1);
  // motor_init(LEFT_MOTOR, &MOTOR2, &ENC2);
  
  // systick_init(16000 - 1); // 1ms systick ticks
  enter_user_mode();
  while (1) {
  }
  return 0;
}
