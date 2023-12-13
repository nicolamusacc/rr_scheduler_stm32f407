#include "init_scheduler_conf.h"

TCB_t user_task[MAX_TASKS];

void enable_processor_faults(void)
{
	// enable all the fault exceptions
		uint32_t * pSCB_SHCSR = (uint32_t*) 0xE000ED24;
		*pSCB_SHCSR |= (1<<16);		// mem fault
		*pSCB_SHCSR |= (1<<17);		// bus fault
		*pSCB_SHCSR |= (1<<18);		// user fault
}


__attribute__((naked)) void init_scheduler_stack(uint32_t sched_top_of_stack)
{
	__asm volatile("MSR MSP, r0");
	__asm volatile("BX LR");

}


void init_systick_timer(uint32_t timer_hz)
{
	uint32_t *pSRVR = (uint32_t*) 0xE000E014;
	uint32_t *pSCSR = (uint32_t*) 0xE000E010;
	uint32_t count_value = (SYSTICK_TIM_CLK/timer_hz)-1;

	// clear the first 24 bits
	*pSRVR &= ~(0x00FFFFFF);

	//load counter value
	*pSRVR |= count_value;

	// settings
	*pSCSR |= (1 << 1); 	// enable systick exception request
	*pSCSR |= (1 << 2);		// clock source: processors clk

	// enable counter
	*pSCSR |= (1 << 0);
}


void init_tasks_stack(void)
{
	user_task[0].current_state = TASK_READY_STATE;
	user_task[1].current_state = TASK_READY_STATE;
	user_task[2].current_state = TASK_READY_STATE;
	user_task[3].current_state = TASK_READY_STATE;
	user_task[4].current_state = TASK_READY_STATE;

	user_task[0].psp_value = IDLE_STACK_START;
	user_task[1].psp_value = T1_STACK_START;
	user_task[2].psp_value = T2_STACK_START;
	user_task[3].psp_value = T3_STACK_START;
	user_task[4].psp_value = T4_STACK_START;

	user_task[0].task_handler = idle_task;
	user_task[1].task_handler = task1_handler;
	user_task[2].task_handler = task2_handler;
	user_task[3].task_handler = task3_handler;
	user_task[4].task_handler = task4_handler;

	uint32_t *pPSP;
	// create dummy state of stack for all tasks
	for(int i=0; i<MAX_TASKS; i++){
		pPSP = (uint32_t*) user_task[i].psp_value;

		pPSP--;		// xPSR
		*pPSP = DUMMY_XPSR;

		pPSP--;		// PC
		*pPSP = (uint32_t) user_task[i].task_handler;

		pPSP--;		// LR
		*pPSP = 0xFFFFFFFD;

		// push the rest of the stack frame with empty values
		for(int j=0; j<13; j++){
			pPSP--;
			*pPSP = 0;
		}
		user_task[i].psp_value = (uint32_t) pPSP;
	}
}