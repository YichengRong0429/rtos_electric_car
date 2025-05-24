.cpu cortex-m4
.syntax unified
.thumb

.section .text

.thumb_func
.global enter_user_mode
enter_user_mode:
  ldr r0, = __psp_stack_top        // load the process stack pointer (used by user mode) into r0
  msr psp, r0                      // move this value to the Process stack pointer

  // jump to the crt0 function (user mode)
  b _crt0                          // load the address of the crt function
  bkpt