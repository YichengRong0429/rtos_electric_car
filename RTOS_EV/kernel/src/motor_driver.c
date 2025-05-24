#include <encoder.h>
#include <gpio.h>
#include <motor_driver.h>
#include <pwm.h>
#include <stdint.h>
#include <unistd.h>

#define PWM_PERIOD     4000

#define MAX_DUTY_CYCLE 100

#define NUM_MOTORS     UINT32_C(2)

#define GPIO_MOTOR_IN_ATTR                                                     \
    MODE_GP_OUTPUT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_HIGH, PUPD_NONE, ALT0

static struct motor_attr motors[NUM_MOTORS];

void motor_init(enum motor_mapping motor, struct motor_attr *attr,
                struct encoder_pin_attr *enc_attr) {
    motors[motor] = *attr;

    gpio_init(attr->motor_in1.port, attr->motor_in1.num, GPIO_MOTOR_IN_ATTR);
    gpio_init(attr->motor_in2.port, attr->motor_in2.num, GPIO_MOTOR_IN_ATTR);
    gpio_init(attr->motor_en.port, attr->motor_en.num, MODE_ALT,
              OUTPUT_PUSH_PULL, OUTPUT_SPEED_VERY_HIGH, PUPD_PULL_DOWN,
              attr->timer.gpio_alt);

    encoder_init((enum encoder_mapping)motor, enc_attr);

    IS_COMP = (attr->timer.is_comp) ? 1 : 0;
    timer_start_pwm(PWM_PERIOD, 0, attr->timer.timer, attr->timer.channel);
}

uint8_t motor_position(enum motor_mapping motor) {
    return encoder_read((enum encoder_mapping)motor);
}

int sys_motor_set(enum motor_mapping motor, uint32_t duty_cycle,
                  direction_t direction) {
    // NOTE: This function should return -1 if an invalid motor is given and 0
    // otherwise
    // Hint: Take a look at the datasheet for the L298N motor driver
    struct motor_attr mt = motors[motor];
    struct pin in1 = mt.motor_in1; ///< GPIO Pin for MOTOR_IN1
    struct pin in2 = mt.motor_in2; ///< GPIO Pin for MOTOR_IN2

    switch (direction) {
        case FREE: // set both to low
            gpio_clr(in1.port, in1.num);
            gpio_clr(in2.port, in2.num);
            break;
        case FORWARD: // set one high, one low
            gpio_set(in1.port, in1.num);
            gpio_clr(in2.port, in2.num);
            break;
        case BACKWARD: // set one low, one high
            gpio_clr(in1.port, in1.num);
            gpio_set(in2.port, in2.num);
            break;
        case STOP: // set both high
            gpio_set(in1.port, in1.num);
            gpio_set(in2.port, in2.num);
            break;
    }

    timer_set_duty_cycle(motors[motor].timer.timer, motors[motor].timer.channel, duty_cycle);
    return 0;
}
