/*
 * FreeRTOS Kernel V11.1.0
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *----------------------------------------------------------*/

/* Ensure stdint is only used by the compiler, and not the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
#include <stdint.h>
extern uint32_t SystemCoreClock;
#endif

// CRITICAL FIXES:
#define configUSE_PREEMPTION                        1
#define configUSE_IDLE_HOOK                         1 // CHANGED: Enable for debugging
#define configUSE_TICK_HOOK                         1 // CHANGED: Enable for debugging
#define configUSE_PORT_OPTIMISED_TASK_SELECTION     0
#define configUSE_TICKLESS_IDLE                     0
#define configCPU_CLOCK_HZ                          (SystemCoreClock)
#define configTICK_RATE_HZ                          ((TickType_t)1000)
#define configMAX_PRIORITIES                        (7)
#define configMINIMAL_STACK_SIZE                    ((uint16_t)256) // CHANGED: Reduced from 512
#define configTOTAL_HEAP_SIZE                       ((size_t)(64 * 1024))
#define configMAX_TASK_NAME_LEN                     (16)
#define configUSE_TRACE_FACILITY                    1
#define configUSE_16_BIT_TICKS                      0
#define configIDLE_SHOULD_YIELD                     1
#define configUSE_MUTEXES                           1
#define configQUEUE_REGISTRY_SIZE                   8
#define configCHECK_FOR_STACK_OVERFLOW              1 // CHANGED: Enable stack overflow detection
#define configUSE_RECURSIVE_MUTEXES                 1
#define configUSE_MALLOC_FAILED_HOOK                1 // CHANGED: Enable for debugging
#define configUSE_APPLICATION_TASK_TAG              0
#define configUSE_COUNTING_SEMAPHORES               1
#define configGENERATE_RUN_TIME_STATS               0

// CRITICAL FIX: Enable dynamic allocation
#define configSUPPORT_STATIC_ALLOCATION 0  // CHANGED: Disable static
#define configSUPPORT_DYNAMIC_ALLOCATION 1 // ADDED: Enable dynamic allocation

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 0
#define configMAX_CO_ROUTINE_PRIORITIES (2)

/* Software timer definitions. */
#define configUSE_TIMERS 0
#define configTIMER_TASK_PRIORITY (2)
#define configTIMER_QUEUE_LENGTH 10
#define configTIMER_TASK_STACK_DEPTH (configMINIMAL_STACK_SIZE * 2)

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskCleanUpResources 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xQueueGetMutexHolder 1
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_eTaskGetState 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1 // ADDED: For stack debugging

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
/* __NVIC_PRIO_BITS will be specified when CMSIS is being used. */
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 4 /* 15 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 0xf

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY                                        \
  (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY                                   \
  (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
#define configASSERT(x)                                                        \
  if ((x) == 0) {                                                              \
    taskDISABLE_INTERRUPTS();                                                  \
    for (;;)                                                                   \
      ;                                                                        \
  }

/* CRITICAL FIX: These MUST be commented out when using custom tick source
   (TIM7) Uncomment ONLY if using SysTick for FreeRTOS tick */
#define vPortSVCHandler                             SVC_Handler
#define xPortPendSVHandler                          PendSV_Handler
// #define xPortSysTickHandler                         SysTick_Handler

/* ADDED: Memory management scheme - use heap_4.c */
#define configUSE_NEWLIB_REENTRANT 0

#endif /* FREERTOS_CONFIG_H */