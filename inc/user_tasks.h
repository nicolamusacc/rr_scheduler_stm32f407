#ifndef USER_TASKS_H_
#define USER_TASKS_H_

#include<stdint.h>
#include<stdio.h>
#include "scheduler.h"
#include "STM32F407VGT6.h"
#include "led.h"

#define MAX_TASKS  5

extern uint8_t current_task;		// start with task 1 and not with idle

typedef struct{
	uint32_t psp_value;
	uint32_t block_counter;
	uint8_t current_state;
	void (*task_handler)(void);
}TCB_t;

extern TCB_t user_task[MAX_TASKS];

/** @brief The function is the idle task handler. It waits in loop.
 *	
 *  @return void
 */
void idle_task(void);

/** @brief The function is the task 1 handler
 *  
 * It blinks the <led>, then it put itself in a blocking state and leaves the control to the scheduler 
 *
 *  @return void
 */
void task1_handler(void);


/** @brief The function is the task 2 handler
 *  
 * It blinks the <led>, then it put itself in a blocking state and leaves the control to the scheduler 
 *
 *  @return void
 */
void task2_handler(void);


/** @brief The function is the task 3 handler
 *  
 * It blinks the <led>, then it put itself in a blocking state and leaves the control to the scheduler 
 *
 *  @return void
 */
void task3_handler(void);


/** @brief The function is the task 4 handler
 *  
 * It blinks the <led>, then it put itself in a blocking state and leaves the control to the scheduler 
 *
 *  @return void
 */
void task4_handler(void);

#endif