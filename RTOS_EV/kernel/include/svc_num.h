/**
 * @file   svc_num.h
 *
 * @brief  Definitions for SVC numbers used by kernel and newlib syscalls.
 *
 * @date   August 19, 2019
 * @author Ronit Banerjee <ronitb@andrew.cmu.edu>
 */

#ifndef _SVC_NUM_H_
#define _SVC_NUM_H_

/** @brief SVC number for sbrk() */
#define SVC_SBRK 0
/** @brief SVC number for write() */
#define SVC_WRITE 1
/** @brief SVC number for close() */
#define SVC_CLOSE 2
/** @brief SVC number for fstat() */
#define SVC_FSTAT 3
/** @brief SVC number for isatty() */
#define SVC_ISATTY 4
/** @brief SVC number for lseek() */
#define SVC_LSEEK 5
/** @brief SVC number for read() */
#define SVC_READ 6
/** @brief SVC number for exit() */
#define SVC_EXIT 7
/** @brief SVC number for thread_init() */
#define SVC_THR_INIT 9
/** @brief SVC number for thread_create() */
#define SVC_THR_CREATE 10
/** @brief SVC number for thread_kill() */
#define SVC_THR_KILL 11
/** @brief SVC number for scheduler_start() */
#define SVC_SCHD_START 12
/** @brief SVC number for mutex_init() */
#define SVC_MUT_INIT 13
/** @brief SVC number for mutex_lock() */
#define SVC_MUT_LOK 14
/** @brief SVC number for mutex_unlock() */
#define SVC_MUT_ULK 15
/** @brief SVC number for wait_until_next_period() */
#define SVC_WAIT 16
/** @brief SVC number for get_time() */
#define SVC_TIME 17
/** @brief SVC number for get_priority() */
#define SVC_PRIORITY 19
/** @brief SVC number for thread_time() */
#define SVC_THR_TIME 20

/** @brief SVC number for servo_enable() */
#define SVC_SERVO_ENABLE 22
/** @brief SVC number for servo_set() */
#define SVC_SERVO_SET 23

#define SVC_REG_ENC_CALLBACK 24
#define SVC_SET_MOTOR 25

#endif /* _SVC_NUM_H_ */
