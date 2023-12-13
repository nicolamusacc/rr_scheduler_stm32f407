#include "scheduler.h"

uint32_t static g_tick_count = 0;

uint8_t current_task = 1;

/* psp and msp operations*/
uint32_t get_psp_value(void)
{
	return(user_task[current_task].psp_value);
}


__attribute__((naked)) void switch_msp_with_psp(void)
{
	// set psp value of the current task
	__asm volatile("PUSH {LR}");
	__asm volatile("BL get_psp_value");		// return result store in R0
	__asm volatile("MSR PSP, R0");
	__asm volatile("POP {LR}");

	// modify control register to change SP to PSP
	__asm volatile("MOV R0, #0x02");
	__asm volatile("MSR CONTROL,R0");
	__asm volatile("BX LR");
}


void save_psp_value(uint32_t current_psp_value)
{
	user_task[current_task].psp_value = current_psp_value;
}


void schedule(void)
{
	uint32_t *pICSR = (uint32_t*)0xE000ED04;
	// pend the pendsv exception
	*pICSR |= (1<<28);
}


void task_delay(uint32_t tick_count)
{

	// disable interrupts
	INTERRUPT_DISABLE();
	if(current_task){

	user_task[current_task].block_counter = g_tick_count + tick_count;
	user_task[current_task].current_state = TASK_BLOCKED_STATE;

	schedule();
	}

	// enable interrupts
	INTERRUPT_ENABLE();
}


void update_next_task(void)
{
	uint8_t state = TASK_BLOCKED_STATE;

	// look for a read task to schedule
	for(int i = 0; i < MAX_TASKS; i++){
		current_task++;
		current_task %= MAX_TASKS;
		state = user_task[current_task].current_state;
		if( (state == TASK_READY_STATE) && (current_task != 0) ){
			break;
		}
	}
	// when all user tasks are blocked, schedule idle task
	if(state != TASK_READY_STATE){
		current_task = 0;
	}
}


void update_global_tick_count(void)
{
	g_tick_count++;
}


void unblock_tasks(void)
{
	for(int i = 1 ; i < MAX_TASKS ; i++)
	{
		if(user_task[i].current_state != TASK_READY_STATE)
		{
			if(user_task[i].block_counter == g_tick_count)
			{
				user_task[i].current_state = TASK_READY_STATE;
			}
		}
	}
}


void SysTick_Handler(void)
{

	uint32_t *pICSR = (uint32_t*)0xE000ED04;

	update_global_tick_count();

	unblock_tasks();

	// pend the pendsv exception
	*pICSR |= (1<<28);

}


__attribute__((naked)) void PendSV_Handler(void)
{
	/* save the context of current task*/

	// get psp of the current running task
	__asm volatile("MRS R0, PSP");
	// push the second part of the stack frame to task stack
	__asm volatile("STMDB R0!, {R4-R11}");
	// save LR which gets modified due to the BL instructions
	__asm volatile("PUSH {LR}");
	// save current value of psp
	__asm volatile("BL save_psp_value");

	/* retrieve the context of next task*/

	// decide next task to be scheduled
	__asm volatile("BL update_next_task");
	// get past psp value
	__asm volatile("BL get_psp_value");
	// using psp value, retrieve sf2 registers (R4-R11)
	__asm volatile("LDMIA R0!, {R4-R11}");
	// update the psp register and exit
	__asm volatile("MSR PSP, R0");

	__asm volatile("POP {LR}");

	__asm volatile("BX LR");
	// sf1 is then automatically fetched into processor
}