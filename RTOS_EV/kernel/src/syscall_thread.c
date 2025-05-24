/** @file   syscall_thread.c
 *
 *  @brief  
 *
 *  @date   
 *
 *  @author 
 */

#include <stdint.h>
#include "syscall_thread.h"
#include "syscall_mutex.h"
#include <stdbool.h>
#include "mpu.h"
#include <assert.h>
#include "systick.h"
#include "arm.h"
#include "printk.h"
#include "uart.h"
#include "syscall.h"

/** @brief      Initial XPSR value, all 0s except thumb bit. */
#define XPSR_INIT 0x1000000

/** @brief Interrupt return code to user mode using PSP.*/
#define LR_RETURN_TO_USER_PSP 0xFFFFFFFD
/** @brief Interrupt return code to kernel mode using MSP.*/
#define LR_RETURN_TO_KERNEL_MSP 0xFFFFFFF1

#define RUNNABLE  (0)
#define RUNNING   (1)
#define WAITING   (2)
#define INACTIVE  (3)
#define IDLE_PRIO (14)
#define ACTIVE_THREADS (14)
#define DEFAULT (15)
#define TOTAL_THREADS (16)
#define MAX_MUTEX (32)
#define NOT_LOCKED (64) // Any number that isn't a valid thread index.
#define NOT_DYNAMIC (64) // dynamic_prio = NOT_DYNAMIC if thread holds NO mutexes
#define LOWEST_PRIO  (10000)

/**
 * @brief      Heap high and low pointers.
 */
//@{
extern char
  __thread_u_stacks_low,
  __thread_u_stacks_top,
  __thread_k_stacks_low,
  __thread_k_stacks_top;
//@}

/**
 * @brief      Precalculated values for UB test.
 */
float ub_table[] = {
  0.000, 1.000, .8284, .7798, .7568,
  .7435, .7348, .7286, .7241, .7205,
  .7177, .7155, .7136, .7119, .7106,
  .7094, .7083, .7075, .7066, .7059,
  .7052, .7047, .7042, .7037, .7033,
  .7028, .7025, .7021, .7018, .7015,
  .7012, .7009
};

/**
 * @struct user_stack_frame
 *
 * @brief  Stack frame upon exception.
 */
typedef struct {
  uint32_t r0;   /** @brief Register value for r0 */
  uint32_t r1;   /** @brief Register value for r1 */
  uint32_t r2;   /** @brief Register value for r2 */
  uint32_t r3;   /** @brief Register value for r3 */
  uint32_t r12;  /** @brief Register value for r12 */
  uint32_t lr;   /** @brief Register value for lr*/
  uint32_t pc;   /** @brief Register value for pc */
  uint32_t xPSR; /** @brief Register value for xPSR */
} interrupt_stack_frame; // USER STACK FRAME

typedef struct {
  interrupt_stack_frame *psp; // r13
  uint32_t r4;
  uint32_t r5;
  uint32_t r6;
  uint32_t r7;
  uint32_t r8;
  uint32_t r9;
  uint32_t r10;
  uint32_t r11;
  uint32_t lr; // r14
} thread_context; // KERNEL STACK FRAME

typedef struct {
  thread_context *context; // context
  uint32_t T; // period
  uint32_t C; // allocated computation time
  uint32_t runtime; // time while RUNNING
  uint32_t periodtime; // time while RUNNING, RUNNABLE, and WAITING
  uint32_t allctime; // computation time OVER ALL PERIODS
  uint32_t static_priority; // static priority, ranges from [0 - 15]
  uint32_t dynamic_priority; // NOT_DYNAMIC if thread doesn't hold ANY mutex
  uint32_t status; // RUNNABLE = 0, RUNNING = 1, WAITING = 2, INACTIVE = 3
  uint32_t svc_status; // 0 = inactive, 1 = active
  float thread_schedulability;
  int hold_mutex[MAX_MUTEX];
} tcb;

kmutex_t global_mutexes[MAX_MUTEX];
int mutexCount=0;

/* GLOBAL KERNEL DATA STRUCTURES */
uint32_t global_time = 0;
uint32_t global_max_threads;
uint32_t global_stack_size;
void *global_idle_fn;
uint32_t global_max_mutexes;
uint32_t cur_thread = 15; // which thread is currently running
static uint32_t msps[TOTAL_THREADS];
static uint32_t psps[TOTAL_THREADS];
extern void *thread_kill;
extern void *default_idle;
tcb tcbs[TOTAL_THREADS];
bool runnable_pool[TOTAL_THREADS];
bool waiting_pool[TOTAL_THREADS];
float schedulability;
void update_status(uint32_t prio, uint32_t status) {
  tcbs[prio].status = status;
  runnable_pool[prio] = (tcbs[prio].status == RUNNABLE) ? true : false;
  waiting_pool[prio] = (tcbs[prio].status == WAITING) ? true : false;
  return;
}

void rms_scheduler() {
  // If no ACTIVE threads (not including IDLE, then next_thread = DEFAULT)
  bool any_active = false;
  uint32_t next_thread = IDLE_PRIO;
  /*
  Sweep through all threads
  if there is a RUNNABLE active thread with a shorter T period, it has higher prio
  find the thread with the SHORTEST T: it will be the next thread.
  */
  for (int i = 0; i < ACTIVE_THREADS; i++) {
    if (tcbs[i].status != INACTIVE) {
      any_active = true;
    }

    if(tcbs[i].status == RUNNABLE || tcbs[i].status == RUNNING) {
      next_thread = i;
      break;
    }
  }

  if (!any_active) {
    cur_thread = DEFAULT;
    return;
  }

  // if next_thread == IDLE_PRIO, then no active candidates were found.
  // if cur_thread is running, continue running.
  if (next_thread == IDLE_PRIO && tcbs[cur_thread].status == RUNNING) {
    return;
  }
  else {
    if (tcbs[cur_thread].status == RUNNING) {
      update_status(cur_thread, RUNNABLE);
      update_status(next_thread, RUNNING);
    }
    else {
      update_status(next_thread, RUNNING);
    }
  }
  cur_thread = next_thread;
}

void HLP_scheduler() {
  // If no ACTIVE threads then n = DEFAULT
  bool any_active = false;
  uint32_t n = IDLE_PRIO;
  /* Sweep through ALL threads. Do NOT early exit */
  /* CANNOT assume that earlier threads are higher prio. MANUALLY CHECK using static/dynamic*/
  for (int i = 0; i < ACTIVE_THREADS; i++) {
    if (tcbs[i].status != INACTIVE) any_active = true;
    if (tcbs[i].status == RUNNABLE || tcbs[i].status == RUNNING) {
      // Four cases: differentiate between n and i using dynamic vs static
      if (tcbs[i].dynamic_priority == NOT_DYNAMIC && tcbs[n].dynamic_priority == NOT_DYNAMIC) { // both static
        if (tcbs[i].static_priority < tcbs[n].static_priority) n = i;
      }
      else if (tcbs[i].dynamic_priority != NOT_DYNAMIC && tcbs[n].dynamic_priority == NOT_DYNAMIC) { // i dynamic, n static!!! use <=
        if (tcbs[i].dynamic_priority <= tcbs[n].static_priority) n = i;
      }
      else if ((tcbs[i].dynamic_priority == NOT_DYNAMIC && tcbs[n].dynamic_priority != NOT_DYNAMIC)) { // i static, n dynamic
        if (tcbs[i].static_priority < tcbs[n].dynamic_priority) n = i;
      }
      else { // both dynamic
        if (tcbs[i].dynamic_priority < tcbs[n].dynamic_priority) n = i;
      }
    }
  }

  if (!any_active) {
    cur_thread = DEFAULT;
    return;
  }

  // if n == IDLE_PRIO, then no active candidates were found.
  // if cur_thread is running, continue running.
  if (n == IDLE_PRIO && tcbs[cur_thread].status == RUNNING) {
    return;
  }
  else {
    if (tcbs[cur_thread].status == RUNNING) {
      update_status(cur_thread, RUNNABLE);
      update_status(n, RUNNING);
    }
    else {
      update_status(n, RUNNING);
    }
  }
  cur_thread = n;
}

// NOTE: This isn't a true round robin scheduler. Is that okay?
void rr_scheduler() {
  uint32_t next_thread = IDLE_PRIO;
  /*
  Assume all thread states are properly determined prior in systick_c_handler  
  1. if cur_thread is the IDLE thread:
    - if there are active threads that are RUNNABLE, switch to first RUNNABLE active thread
    - if no other active threads are RUNNABLE, stay on idle thread
  2. if cur_thread is WAITING and non-IDLE:
    - if there are active threads that are RUNNABLE, switch to first RUNNABLE active thread
    - if no other active threads are RUNNABLE, switch to idle thread
  3. if cur_thread is RUNNING and non-IDLE:
    - if there are active threads that are RUNNABLE, switch to first RUNNABLE active thread
    - if no other active_threads are RUNNABLE, stay on cur_thread
  */

  for (int i = 0; i < ACTIVE_THREADS; i++) {
    if (tcbs[i].status == RUNNABLE) { // NEXT THREAD
      next_thread = i;
      break;
    }
  }

  if (next_thread == IDLE_PRIO && tcbs[cur_thread].status == RUNNING) {
    return;
  }
  else {
    if (tcbs[cur_thread].status == RUNNING) {
      update_status(cur_thread, RUNNABLE);
      update_status(next_thread, RUNNING);
    }
    else {
      if (tcbs[cur_thread].status != WAITING) {
        breakpoint();
      }
      assert(tcbs[cur_thread].status == WAITING);
      update_status(next_thread, RUNNING);
    }
  }
  cur_thread = next_thread;
}

// Runs once every 1ms
void systick_c_handler() {
  global_time++;
  tcbs[cur_thread].runtime++;
  tcbs[cur_thread].allctime++;

  // if cur_thread has reached C, set cur_thread to WAITING
  if (cur_thread != IDLE_PRIO && tcbs[cur_thread].runtime == tcbs[cur_thread].C) {
    update_status(cur_thread, WAITING);
  }

  // have any active threads entered a new period?
  // -- does not apply to idle or default threads
  for (uint32_t i = 0; i < ACTIVE_THREADS; i++) {
    if (tcbs[i].status != INACTIVE) {
      tcbs[i].periodtime++;
      if (tcbs[i].periodtime == tcbs[i].T) { // NEW PERIOD
        update_status(i, RUNNABLE);
        tcbs[i].periodtime = 0;
        tcbs[i].runtime = 0; 
      }
    }
  }

  pend_pendsv();
}

// e.g. in the case of first starting your kernel and no threads are running yet
void *pendsv_c_handler(void *context_ptr){
  // get svc status NOT from TCB but from get_svc_status
  // set TCB svc_status for current thread
  tcbs[cur_thread].context = (thread_context *)context_ptr;
  tcbs[cur_thread].svc_status = get_svc_status();
  // rr_scheduler(); // ROUND ROBIN SCHEDULER
  HLP_scheduler(); // RMS SCHEDULER
  set_svc_status(tcbs[cur_thread].svc_status);
  return (void *)tcbs[cur_thread].context;
}

void clear_registers(uint32_t prio) {
  tcbs[prio].context->r4 = 0;
  tcbs[prio].context->r5 = 0;
  tcbs[prio].context->r6 = 0;
  tcbs[prio].context->r7 = 0;
  tcbs[prio].context->r8 = 0;
  tcbs[prio].context->r9 = 0;
  tcbs[prio].context->r10 = 0;
  tcbs[prio].context->r11 = 0;

  tcbs[prio].context->psp->r1 = 0;
  tcbs[prio].context->psp->r2 = 0;
  tcbs[prio].context->psp->r3 = 0;
  tcbs[prio].context->psp->r12 = 0;
}

int sys_thread_init(
  uint32_t max_threads,
  uint32_t stack_size,
  void *idle_fn,
  uint32_t max_mutexes
){
  // initialize global kernel data structures required by scheduler
  // global_time = 0;
  global_max_threads = max_threads;
  global_stack_size = 1 << (mm_log2ceil_size(stack_size * 4)); // round to closest power of 2
  global_idle_fn = idle_fn;
  global_max_mutexes = max_mutexes;
  schedulability = 0.0;

  // check that stack size fits in user-space and kernel-space stacks
  // each has 32kB of space
  if (global_stack_size * global_max_threads > 32768) {
    return -1;
  } 

  // initialize array of msps and psps
  // point tcb contexts to context array
  // DO NOT allocate for default thread (tcbs[15])
  for (int i = 0; i < TOTAL_THREADS - 1; i++) {
    msps[i] = (uint32_t)(&__thread_k_stacks_top - i * global_stack_size);
    psps[i] = (uint32_t)(&__thread_u_stacks_top - i * global_stack_size);
    update_status(i, INACTIVE); // default
    tcbs[i].svc_status = 0;
  }
  update_status(DEFAULT, INACTIVE); // default
  tcbs[DEFAULT].svc_status = 0;
  tcbs[DEFAULT].dynamic_priority = NOT_DYNAMIC;

  // default idle function if none provided
  if (idle_fn == NULL) {
    idle_fn = &default_idle;
  }

  // create IDLE and DEFAULT threads
  sys_thread_create(idle_fn, IDLE_PRIO, 0, 0, 0);
  update_status(DEFAULT, RUNNING);
  return 0;
}

// sums up thread_schedulabilities for all active threads
void update_schedulability() {
  float new_schedulability = 0.0;
  for (int i = 0; i < ACTIVE_THREADS; i++) {
    if (tcbs[i].status != INACTIVE) {
      new_schedulability = new_schedulability + tcbs[i].thread_schedulability;
    }
  }
  schedulability = new_schedulability;
}

int sys_thread_create(void *fn,uint32_t prio,uint32_t C,uint32_t T,void *vargp){
  // check that thread hasn't already been created:
  if (tcbs[prio].status != INACTIVE) {
    return -1;
  }

  // check that we don't exceed global_max_threads
  uint32_t thread_count = 1; // = 1 representing the thread we're about to create
  for (int i = 0; i < ACTIVE_THREADS; i++) {
    if (tcbs[i].status != INACTIVE) {
      thread_count++;
    }
  }
  if (thread_count > global_max_threads) {
    return -1;
  }

  // must update status prior to updating schedulability
  update_status(prio, RUNNABLE);
  // check for schedulability
  if (prio != IDLE_PRIO && prio != DEFAULT) {
    float thread_schedulability = (float)C / (float)T;
    if (schedulability + thread_schedulability > ub_table[thread_count]) {
      update_status(prio, INACTIVE);
      return -1;
    }
    tcbs[prio].thread_schedulability = thread_schedulability;
    update_schedulability(); // GLOBAL
  }

  tcbs[prio].C = C;
  tcbs[prio].T = T;
  tcbs[prio].static_priority = prio;
  tcbs[prio].dynamic_priority = NOT_DYNAMIC;
  tcbs[prio].runtime = 0;
  tcbs[prio].periodtime = 0;
  tcbs[prio].allctime = 0;
  tcbs[prio].svc_status = 0; // initialize inactive SVC
  if (prio != DEFAULT) {
    tcbs[prio].context = (thread_context *) (msps[prio] - sizeof(thread_context));
    tcbs[prio].context->psp = (interrupt_stack_frame *)(psps[prio] - sizeof(interrupt_stack_frame));
  }

  tcbs[prio].context->lr = LR_RETURN_TO_USER_PSP;
  tcbs[prio].context->psp->r0 = (uint32_t)vargp;
  tcbs[prio].context->psp->pc = (uint32_t)fn;
  tcbs[prio].context->psp->lr = (uint32_t)&thread_kill;
  clear_registers(prio);
  
  // set xPSR to some initialization
  tcbs[prio].context->psp->xPSR = XPSR_INIT;

  // No mutexes initially held
  for (int i = 0; i < MAX_MUTEX; i++){
    tcbs[prio].hold_mutex[i]=0;
  }

  return 0;
}

int sys_scheduler_start( uint32_t frequency ){
  systick_init(frequency);
  pend_pendsv();
  return 0;
}

uint32_t sys_get_priority(){
  if (tcbs[cur_thread].dynamic_priority == NOT_DYNAMIC) return tcbs[cur_thread].static_priority;
  else return tcbs[cur_thread].dynamic_priority;
}

uint32_t sys_get_time(){
  return global_time;
}

uint32_t sys_thread_time(){
  // "the number of ticks the current thread has used for computation since the scheduler started"
  return tcbs[cur_thread].allctime;
}

void sys_thread_kill(){
  /*
  if cur_thread == IDLE, set PC of tcbs[IDLE].context.psp.pc to &global_idle_fn, then call pend_pendsv
  if cur_thread == (default = 15) call sys_exit
  else, deschedule current thread (global_schedulability), set inactive, call pend_pendsv
  */

  if (cur_thread == IDLE_PRIO) {
    breakpoint();
    tcbs[IDLE_PRIO].context->psp->pc = (uint32_t)&global_idle_fn;
    pend_pendsv();
  }
  else if (cur_thread == DEFAULT) {
    sys_exit(1);
  }
  else {
    update_status(cur_thread, INACTIVE); // default
    update_schedulability();
    pend_pendsv();
  }
}

void sys_wait_until_next_period(){
  update_status(cur_thread, WAITING); // default
  pend_pendsv();
}

kmutex_t *sys_mutex_init( uint32_t max_prio ) 
{
  if(mutexCount>=MAX_MUTEX)
  {
    return NULL;
  }
  global_mutexes[mutexCount].index=mutexCount;
  global_mutexes[mutexCount].locked_by=NOT_LOCKED;
  global_mutexes[mutexCount].prio_ceil=max_prio;
  mutexCount++;
  return (kmutex_t*)&global_mutexes[mutexCount-1];
}

void sys_mutex_lock( kmutex_t *mutex ) 
{
  if(cur_thread==IDLE_PRIO) { // idle thread can't use mutex
    return;
  }
  uint32_t static_prio=tcbs[cur_thread].static_priority;
  uint32_t state=save_interrupt_state_and_disable();
  disable_interrupts();

  // thread must have lower priority than the mutex's priority
  // if thread is higher priority (smaller number than mutex), return
  if (static_prio < mutex->prio_ceil) {
    restore_interrupt_state(state);
    sys_thread_kill();
    return;
  }
  
  // Place the lock on the mutex... update mutex fields
  mutex->locked_by = cur_thread;
  tcbs[cur_thread].hold_mutex[mutex->index] = 1;
  if (tcbs[cur_thread].dynamic_priority == NOT_DYNAMIC) {
    tcbs[cur_thread].dynamic_priority = mutex->prio_ceil;
  }
  else if (mutex->prio_ceil < tcbs[cur_thread].dynamic_priority) {
    tcbs[cur_thread].dynamic_priority = mutex->prio_ceil;
  }
  restore_interrupt_state(state);
  pend_pendsv();
  return;
}

void sys_mutex_unlock( kmutex_t *mutex ) 
{
  uint32_t state = save_interrupt_state_and_disable();
  disable_interrupts();

  if (mutex->locked_by==NOT_LOCKED) { // if mutex already unlocked
    restore_interrupt_state(state);
    return;
  }

  // if attempting to unlock a mutex not locked by cur_thread
  if (mutex->locked_by != cur_thread) { 
    restore_interrupt_state(state);
    return;
  }

  // unlock the mutex
  mutex->locked_by = NOT_LOCKED;
  tcbs[cur_thread].hold_mutex[mutex->index] = 0;

  // Find next highest dynamic priority from cur_thread's locked mutexes
  // If no mutexes are found, dynamic prio set to NOT_DYNAMIC
  uint32_t highest_dynamic = NOT_DYNAMIC;
  for (uint32_t i = 0; i < MAX_MUTEX; i++) {
    if ((tcbs[cur_thread].hold_mutex[i]) && (global_mutexes[i].prio_ceil < highest_dynamic)) {
      highest_dynamic = global_mutexes[i].prio_ceil;
    }
  }
  tcbs[cur_thread].dynamic_priority = highest_dynamic;
  restore_interrupt_state(state);
  pend_pendsv();
}
