#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include <stdint.h>
#include "user_tasks.h"
#include "init_scheduler_conf.h"
#include "STM32F407VGT6.h"


/** @brief The function returns the current psp value 
 *
 *   
 *  @return uint32: 
 */
uint32_t get_psp_value(void);


/** @brief The functions saves the current psp value
 *
 *  @param[in] current_psp_value 
 *  @return void
 */
void save_psp_value(uint32_t current_psp_value);


/** @brief The function switches the msp value with the psp value
 * 
 *  @return void
 */
__attribute__((naked)) void switch_msp_with_psp(void);


/** @brief The funciton puts the current task in a blocked state for
 *         nb tick_count and schedule the next task to be executed
 *
 *  @param[in] tick_count  
 *  @return void
 */
void task_delay(uint32_t tick_count);


/** @brief The function checks the tasks in blocking state and unblocks
 *         the tasks to be scheduled (if any)
 *
 *  @return void
 */
void unblock_tasks(void);


/** @brief The function updates the tick count at each Systick Handler call
 *
 *  @return void
 */
void update_global_tick_count(void);


#endif