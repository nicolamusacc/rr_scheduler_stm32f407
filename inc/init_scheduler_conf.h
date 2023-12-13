#ifndef INIT_SCHEDULER_CONF_H_
#define INIT_SCHEDULER_CONF_H_

#include <stdint.h>
#include "user_tasks.h"
#include "STM32F407VGT6.h"

#define DUMMY_XPSR 			0x01000000

#define SIZE_TASK_STACK 	1024U
#define SIZE_SCHED_TASK 	1024U

#define SRAM_START 			0x20000000U
#define SIZE_SRAM 			((128) * (1024))
#define SRAM_END 			((SRAM_START) + (SIZE_SRAM))

#define T1_STACK_START 		SRAM_END
#define T2_STACK_START 		((SRAM_END) - (1 * SIZE_TASK_STACK))
#define T3_STACK_START 		((SRAM_END) - (2 * SIZE_TASK_STACK))
#define T4_STACK_START 		((SRAM_END) - (3 * SIZE_TASK_STACK))
#define IDLE_STACK_START	((SRAM_END) - (4 * SIZE_TASK_STACK))
#define SCHED_STACK_START 	((SRAM_END) - (5 * SIZE_TASK_STACK))

#define TASK_READY_STATE 0x00
#define TASK_BLOCKED_STATE 0xFF


/** @brief The function enables all fault exceptions
 * 
 *  @return void
 */
void enable_processor_faults(void);


/** @brief The function sets the systick timer
 *
 *  @param[in] timer_hz
 *  @return void
 */
void init_systick_timer(uint32_t timer_hz);


/** @brief The function initializes the stack for the scheduler 
 *
 *  @param[in] sched_top_of_stack  
 *  @return void
 */
__attribute__((naked)) void init_scheduler_stack(uint32_t sched_top_of_stack);


/** @brief The function initializes the state, the stack and the task handler of each user task.
 *
 *  @return void
 */
void init_tasks_stack(void);



#endif