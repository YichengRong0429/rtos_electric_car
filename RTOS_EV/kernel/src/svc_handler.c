/**
 * @file 
 *
 * @brief      
 *
 * @date       
 *
 * @author     
 */

#include <stdint.h>
#include <debug.h>
#include <svc_num.h>
#include <syscall.h>
#include "syscall_thread.h"
#include <syscall_mutex.h>
#include "motor_driver.h"
#define UNUSED __attribute__((unused))

struct stm_reg_map {
  volatile uint32_t r0; // begin exception frame
  volatile uint32_t r1;
  volatile uint32_t r2;
  volatile uint32_t r3;
  volatile uint32_t r12;
  volatile uint32_t lr;
  volatile uint32_t pc; // end exception frame
  volatile uint32_t xpsr; // end exception frame
  volatile uint32_t r4;
};
void svc_c_handler(void* psp) 
{
  struct stm_reg_map *current_psp = (struct stm_reg_map*)psp;
  uint32_t *previous_pc=(uint32_t *)(current_psp->pc-2);
  uint16_t svc_number = ((uint16_t )*previous_pc)&0x00FF;
  char *ptr;
  int incr=0;
  int len=0;
  int status=0;
  int returnVal=0;
  char* returnValChar;
  kmutex_t* returnVal1=NULL;
  // uint8_t channel;
  // uint8_t angle;
  // uint8_t enabled;
  uint32_t max_threads;
  uint32_t stack_size;
  void* fn;
  uint32_t prio;
  int32_t frequency;
  uint32_t time;
  uint32_t priority;
  uint32_t max_Prio;
  kmutex_t *current_mutex;
  kmutex_t *current_mutex1;
  uint32_t encoder;
  void *callback;
  uint32_t motor;
  uint32_t duty_cycle;
  direction_t direction;
  switch(svc_number)
  {
    case SVC_SBRK:
      incr=(int)current_psp->r0;
      returnValChar=sys_sbrk(incr);
      current_psp->r0=(uint32_t)returnValChar;
      break;
    
    case SVC_WRITE:
      ptr=(char*)current_psp->r1;
      len=(int)current_psp->r2;
      returnVal=sys_write(1,ptr,len);
      current_psp->r0=returnVal;
      break;

    case SVC_CLOSE:
      break;

    case SVC_FSTAT:
      break;

    case SVC_ISATTY:
      break;
    
    case SVC_LSEEK:
      break;
    
    case SVC_READ:
      ptr=(char*)current_psp->r1;
      len=(int)current_psp->r2;
      returnVal=sys_read(0,ptr,len);
      current_psp->r0=returnVal;
      break;
    
    case SVC_EXIT:
      DEBUG_PRINT( "svc_exit invoked %d\n", svc_number);
      status=(int)current_psp->r0;
      sys_exit(status);
      break;
    
    case SVC_THR_INIT:
      max_threads = current_psp->r0;
      stack_size = current_psp->r1;
      void *idle_fn = (void *)current_psp->r2;
      uint32_t max_mutexes = current_psp->r3;
      returnVal = sys_thread_init(max_threads, stack_size, idle_fn, max_mutexes);
      current_psp->r0 = returnVal;
      break;
    
    case SVC_THR_CREATE:
      fn = (void *)current_psp->r0;
      prio = (uint32_t)current_psp->r1;
      uint32_t C = (uint32_t)current_psp->r2;
      uint32_t T = (uint32_t)current_psp->r3;
      void* vargp = (void *)current_psp->r4;
      returnVal = sys_thread_create(fn, prio, C, T, vargp);
      current_psp->r0 = returnVal;
      break;
    
    case SVC_THR_KILL:
      sys_thread_kill();
      break;

    case SVC_SCHD_START:
      frequency = (uint32_t)current_psp->r0;
      returnVal = sys_scheduler_start(frequency);
      current_psp->r0 = returnVal;
      break;
    
    case SVC_MUT_INIT:
      max_Prio = (uint32_t)current_psp->r0;
      returnVal1 = sys_mutex_init(max_Prio);
      current_psp->r0 = (uint32_t)returnVal1;
      break;

    case SVC_MUT_LOK:
      current_mutex = (kmutex_t *)current_psp->r0;
      sys_mutex_lock(current_mutex);
      break;
    
    case SVC_MUT_ULK:
      current_mutex1 = (kmutex_t *)current_psp->r0;
      sys_mutex_unlock(current_mutex1);
      break;
    
    case SVC_WAIT:
      sys_wait_until_next_period();
      break;
    
    case SVC_TIME:
      time = sys_get_time();
      current_psp->r0 = time;
      break;
    
    case SVC_PRIORITY:
      priority = sys_get_priority();
      current_psp->r0 = priority;
      break;

    case SVC_THR_TIME:
      time = sys_thread_time();
      current_psp->r0 = time;
      break;

    case SVC_SERVO_ENABLE:
      // channel = (uint8_t)current_psp->r0;
      // enabled = (uint8_t)current_psp->r1;
      // returnVal = sys_servo_enable(channel, enabled);
      // current_psp->r0 = returnVal;
      break;

    case SVC_SERVO_SET:
      // channel = (uint8_t)current_psp->r0;
      // angle = (uint8_t)current_psp->r1;
      // returnVal = sys_servo_set(channel, angle);
      // current_psp->r0 = returnVal;
      break;

    case SVC_REG_ENC_CALLBACK:
      encoder = (uint32_t)current_psp->r0;
      callback = (void *)current_psp->r1;
      returnVal = sys_register_encoder_callback(encoder, callback);
      current_psp->r0 = returnVal;
      break;

    case SVC_SET_MOTOR:
      motor = (uint32_t)current_psp->r0;
      duty_cycle = (uint32_t)current_psp->r1;
      direction = (direction_t)current_psp->r2;

      returnVal = sys_motor_set(motor, duty_cycle, direction);
      current_psp->r0 = returnVal;
      break;
      
    default:
      ASSERT(0);
      break;

  }
}