/** @file main.c
 *
 *  @brief  Lab 5 Fall 2024
 */
#include <349_lib.h>
#include <349_peripheral.h>
#include <349_threads.h>
#include <stdio.h>
#include <stdlib.h>

#define UNUSED __attribute__((unused))
#define RIGHT_MOTOR (1)
#define HIGH_DUTY_CYCLE (4000)
#define LOW_DUTY_CYCLE (1500)
double target_speed=3.0;
uint32_t current_time;
uint32_t speed;

/* --- GLOBAL VARIABLES --- */
typedef struct {
    // POSITION
    int32_t prevTicks;
    int32_t ticks;

    uint32_t prev; // previous state
    direction_t dir;
    
    uint32_t record_time;
    // TIME
    uint32_t prevTime; // for use with derivative
} motor_state;
motor_state globalState = { 0, 0, 0b00, MOTOR_FORWARD, 0,0,0 };

typedef struct {
    double dState;  // Last position input (for derivative term)
    double iState;  // Integrator state

    double iMax, iMin; // Max/min allowable integrator state

    double iGain,   // integral gain (k_i)
           pGain,   // proportional gain (k_p)
           dGain;   // derivative gain (k_d)
} SPid;
// TODO: determine values for these constants
const double k_i = 1.0f; // integral gain (k_i)
const double k_p = 1.0f; // proportional gain (k_p)
const double k_d = 1.0f; // derivative gain (k_d)
SPid plantPID = { 0.0f, 0.0f, 0.0f, 0.0f, k_i, k_p, k_d };

/* --- END GLOBAL VARIABLES --- */

int count=0;
// TODO?: add argument that differentiates left and right motor so that you can increment corresponding motor
// void encoder_callback(uint32_t motor, uint32_t pin_a, uint32_t pin_b) {
void encoder_callback(uint32_t pin_a, uint32_t pin_b) {
    // compare state with prevState, increment or decrement globalState.ticks correspondingly
    uint32_t state = (pin_b << 1) | pin_a;
    uint32_t prevState = globalState.prev;
    switch (prevState) {
        case 0b00:
            if (state == 0b10) { 
                globalState.ticks--; // counter-clockwise
                globalState.dir = MOTOR_BACKWARD;
            }
            else if (state == 0b01) {
                globalState.ticks++; // clockwise
                globalState.dir = MOTOR_FORWARD;
            }
            break;
        case 0b01:
            if (state == 0b00) {
                globalState.ticks--;
                globalState.dir = MOTOR_BACKWARD;
            }
            else if (state == 0b11) {
                globalState.ticks++;
                globalState.dir = MOTOR_FORWARD;
            }
            break;
        case 0b11:
            if (state == 0b01) {
                globalState.ticks--;
                globalState.dir = MOTOR_BACKWARD;
            }
            else if (state == 0b10) {
                globalState.ticks++;
                globalState.dir = MOTOR_FORWARD;
            }
            break;
        case 0b10:
            if (state == 0b11) {
                globalState.ticks--;
                globalState.dir = MOTOR_BACKWARD;
            }
            else if (state == 0b00) {
                globalState.ticks++;
                globalState.dir = MOTOR_FORWARD;
            }
            break;
    }
    count++;
    
    globalState.prev = state;
    return;
}

// passed in command rather than error for better accuracy... ticks might change command==target speed
double updatePID(SPid *pid, double target, UNUSED double position) {
    int32_t curTicks = globalState.ticks;
    uint32_t curTime = get_time();
    int32_t prevTicks = globalState.prevTicks;
    uint32_t prevTime = globalState.prevTime;
    double pTerm, iTerm, dTerm;
    double error = target - curTicks;

    pTerm = pid->pGain * error; // calculate proportional term

    pid->iState += error; // calculate integral state
    pid->iState = (pid->iState < pid->iMax) ? pid->iState : pid->iMax; // upper limit
    pid->iState = (pid->iMin < pid->iState) ? pid->iState : pid->iMin; // lower limit
    iTerm = pid->iGain * pid->iState; // calculate integral term

    // calculate the derivative term
    // dTerm = pid->dGain * (position - pid->dState);
    double derivative = (double)(curTicks - prevTicks) / (double)(curTime - prevTime);
    dTerm = pid->dGain * derivative; // calculate the derivative term

    // update dState for next time
    // pid->dState = position;
    globalState.prevTicks = curTicks;
    globalState.prevTime = curTime;
    
    return pTerm + iTerm + dTerm; // After return, send correction to DrivePlantADC(drive);
}

void drivePlantADC(double drive) {
    (void)drive;
    // how to determine duty cycle?? apparently just pass drive in??
    motor_set(RIGHT_MOTOR, drive, globalState.dir);
}

// TODO:
void uart_thread_fn(UNUSED void *vargp ) {
    
    while(1)
    {
        printf("%ld\n", speed);
        wait_until_next_period();
    }
    
}

// TODO:
void pid_thread_fn(UNUSED void *vargp ) {
    
    motor_set(RIGHT_MOTOR, 3000, MOTOR_FORWARD); // forward
    // motor_set(0, 3000, 1); // free rotate
    globalState.prevTicks = globalState.ticks;
    globalState.prevTime = get_time();
    while (1) {
        //printf("Position: %ld | Time: %ld\n", globalState.ticks, get_time());
        //high duty cycle 6000
        //low duty cycle 1500
        motor_set(RIGHT_MOTOR, 2000, globalState.dir);

        // double plantCommand = 0;
        // PID LOOP
        // read sensed value of output
        // double position = readPlantADC();
        // double position = globalState.ticks;

        // call the PID function to update the drive value
        // send the existing state:     &plantPID
        // send the actual error:       plantCommand - position
        // send the actual position:    position
        // double drive = updatePID(&plantPID, plantCommand, -1);
        
        // send the new drive signal to the plant
        // drivePlantADC(drive);
        if(count>=2000)
        {
            count=0;
            current_time=get_time();
            speed=current_time-globalState.record_time;
            globalState.record_time=get_time();
            globalState.prevTicks=globalState.ticks;
            globalState.ticks=2000/speed;
        }
        wait_until_next_period();
    }
    
}

int main(UNUSED int argc, UNUSED char const *argv[]) {
   /** @brief thread user space stack size - 1KB */
    #define USR_STACK_WORDS 256
    #define NUM_THREADS 2
    #define NUM_MUTEXES 0
    #define CLOCK_FREQUENCY 2000
    register_encoder_callback(RIGHT_MOTOR, &encoder_callback); // right motor
    ABORT_ON_ERROR( thread_init( NUM_THREADS, USR_STACK_WORDS, NULL, NUM_MUTEXES ) );
    // TODO: harmonics??
    ABORT_ON_ERROR( thread_create( &pid_thread_fn, 1, 20, 100, ( void * )0 ), "Thread %d\n", 1); // PID thread

    ABORT_ON_ERROR( thread_create( &uart_thread_fn, 0, 100,300, ( void * )0 ), "Thread %d\n", 0); // UART thread

    printf( "Starting scheduler...\n" );
    ABORT_ON_ERROR( scheduler_start( CLOCK_FREQUENCY ) );

    return RET_0349;
}
