/* boot.s
 * Assembler startup file for the STM32F103
 * Ford Seidel  2019-01-31
 * Ronit Banerjee
 *
 * Adapted from Tom Trebisky
 */

# The Cortex M4 is a thumb only processor
.cpu cortex-m4
.syntax unified
.section .ivt
.thumb

.global ivt
ivt:
.word   __msp_stack_top     /* stack top address */
.word   _reset_             /* 1 Reset */
.word   _nmi_               /* 2 NMI */
.word   _hard_fault_        /* 3 Hard Fault */
.word   spin                /* 4 MM Fault */
.word   _bus_fault_         /* 5 Bus Fault */
.word   _usage_fault_       /* 6 Usage Fault */
.word   spin                /* 7 RESERVED */
.word   spin                /* 8 RESERVED */
.word   spin                /* 9 RESERVED*/
.word   spin                /* 10 RESERVED */
.word   _svc_asm_handler_   /* 11 SV call */
.word   spin                /* 12 Debug reserved */
.word   spin                /* 13 RESERVED */
.word   psv_asm_handler     /* 14 PendSV */
.word   systick_c_handler   /* 15 SysTick */
.word   spin                /* 16 IRQ0 Window Watchdog Interrupt */
.word   spin                /* 17 IRQ1 PVD */
.word   spin                /* 18 IRQ2 TAMPER */
.word   spin                /* 19 IRQ3 RTC   */
.word   spin                /* 20 IRQ4 FLASH   */
.word   spin                /* 21 IRQ5 RCC */
.word   encoder_irq_handler_left                /* 22 IRQ6 EXTI0 */
.word   encoder_irq_handler_left                /* 23 IRQ7 EXTI1  */
.word   spin                /* 24 IRQ8 EXTI2   */
.word   spin                /* 25 IRQ9 EXTI3 */
.word   spin                /* 26 IRQ10 EXTI4 */
.word   spin                /* 27 IRQ11 DMA1_Stream0 */
.word   spin                /* 28 IRQ12 DMA1_Stream1 */
.word   spin                /* 29 IRQ13 DMA1_Stream2 */
.word   spin                /* 30 IRQ14 DMA1_Stream3 */
.word   spin                /* 31 IRQ15 DMA1_Stream4 */
.word   spin                /* 32 IRQ16 DMA1_Stream5 */
.word   spin                /* 33 IRQ17 DMA1_Stream6 */
.word   spin                /* 34 IRQ18 ADC1_2 */
.word   spin                /* 35 IRQ19 CAN1_TX   */
.word   spin                /* 36 IRQ20 CAN1_TX0   */
.word   spin                /* 37 IRQ21 CAN1_RX1 */
.word   spin                /* 38 IRQ22 CAN1_SCE */
.word   encoder_irq_handler_right              /* 39 IRQ23 EXTI9_5   */
.word   spin                /* 40 IRQ24 TIM1_BRK   */
.word   spin                /* 41 IRQ25 TIM1_UP */
.word   spin                /* 42 IRQ26 TIM1_TRG_COM */
.word   spin                /* 43 IRQ27 TIM1_CC   */
.word   spin                /* 44 IRQ28 TIM2   */
.word   spin                /* 45 IRQ29 TIM3 */
.word   spin                /* 46 IRQ30 TIM4 */
.word   spin                /* 47 IRQ31 I2C1_EV   */
.word   spin                /* 48 IRQ32 I2C1_ER   */
.word   spin                /* 49 IRQ33 I2C2_EV */
.word   spin                /* 50 IRQ34 I2C2_ER */
.word   spin                /* 51 IRQ35 SPI1   */
.word   spin                /* 52 IRQ36 SPI2   */
.word   spin                /* 53 IRQ37 USART1 */
.word   uart_irq_handler    /* 54 IRQ38 USART2 */
.word   spin                /* 55 IRQ39 USART3   */
.word   encoder_irq_handler_right                /* 56 IRQ40 EXTI15_10   */
.word   spin                /* 57 IRQ41 RTCAlarm */
.word   spin                /* 58 IRQ42 OTG_FS_WKUP */
.word   spin                /* 59 IRQ43 RESERVED   */
.word   spin                /* 60 IRQ44 RESERVED   */
.word   spin                /* 61 IRQ45 RESERVED */
.word   spin                /* 62 IRQ46 RESERVED */
.word   spin                /* 63 IRQ47 RESERVED   */
.word   spin                /* 64 IRQ48 RESERVED   */
.word   spin                /* 65 IRQ49 RESERVED */
.word   spin                /* 66 IRQ50 TIM5 */
.word   spin                /* 67 IRQ51 SPI3   */
.word   spin                /* 68 IRQ52 UART4   */
.word   spin                /* 69 IRQ53 UART5 */
.word   spin                /* 70 IRQ54 TIM6 */
.word   spin                /* 71 IRQ55 TIM7   */
.word   spin                /* 72 IRQ56 DMA2_Channel1   */
.word   spin                /* 73 IRQ57 DMA2_Channel2 */
.word   spin                /* 74 IRQ58 DMA2_Channel3 */
.word   spin                /* 75 IRQ59 DMA2_Channel4   */
.word   spin                /* 76 IRQ60 DMA2_Channel5   */
.word   spin                /* 77 IRQ61 ETH */
.word   spin                /* 78 IRQ62 ETH_WKUP */
.word   spin                /* 79 IRQ63 CAN2_TX   */
.word   spin                /* 80 IRQ64 CAN2_RX0  */
.word   spin                /* 81 IRQ65 CAN2_RX1 */
.word   spin                /* 82 IRQ66 CAN2_SCE */
.word   spin                /* 83 IRQ67 OTG_FS   */

.section .text

.thumb_func
spin: bkpt

.thumb_func
_nmi_ : bkpt

.thumb_func
_hard_fault_ : bkpt

.thumb_func
_mm_fault_:
  bkpt

.thumb_func
_bus_fault_ : bkpt

.thumb_func
_usage_fault_ : bkpt

.thumb_func
_pend_sv_ :
  bkpt

.thumb_func
_spi1_handler:
  bkpt

.thumb_func
.global _reset_
_reset_:
@ Write your boot loader here!
@ The bkpt instruction puts a breakpoint that fires an exception that
@ can be caught if a debugger is attached.
  ldr r2, =_erodata @ ;beginning of FLASH data
  ldr r3, =.data @ ;beginning of SRAM data
  ldr r10, =_data_size
  add r11, r2, r10 @ ; bottom
  cmp r10, #0 @ ;if data size is 0, do not copy flash into SRAM, branch to BSS copying
  beq bss
L1 :
  ldr r4, [r2], #4 @ ;each mem address is length 4
  str r4, [r3], #4
  cmp r2, r11 @ ;checks bottom
  beq bss @ ;if at bottom, branch to beginning of bss loop
  b L1 @ ;else, branch to loop
bss :
  ldr r2, =_k_bss @; beginning of SRAM data
  ldr r3, =_bss_size
  add r4, r2, r3
  cmp r3, #0 @ ;if bss size is 0, skip
  beq end
  mov r0, #0
L2 :
  str r0, [r2], #4
  cmp r2, r4
  beq end
  b L2
end :
  bl kernel_main
  bkpt

.thumb_func
.global _sys_tick_asm_
_sys_tick_asm_:
  bkpt

.thumb_func
.global psv_asm_handler
psv_asm_handler:
  mrs r0, psp                       // move the Process stack pointer to r0
  push {r0, r4-r11, r14} // push thread context onto kernel stack
  mrs r0, msp

  bl pendsv_c_handler
  msr msp, r0
  pop {r0, r4-r11, r14}
  msr psp, r0
  bx lr

.thumb_func
.global _svc_asm_handler_
_svc_asm_handler_: //branch to C handler
  mrs r0, psp                       // move the Process stack pointer to r0

  // jump to the svc_c_handler function (user mode)
  b svc_c_handler                             // Branch to svc_c_handler
  