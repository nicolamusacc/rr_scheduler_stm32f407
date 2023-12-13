#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "main.h"

extern void initialise_monitor_handles(void);

int main(void)
{
	enable_processor_faults();

	initialise_monitor_handles();

	init_scheduler_stack(SCHED_STACK_START);

	AllLedConfInit();

	init_tasks_stack();

	init_systick_timer(TICK_HZ);

	switch_msp_with_psp();

	task1_handler();

    /* Loop forever */
	for(;;);
}



void MemManage_Handler(void){
	printf("mem handler");
	while(1);
}

void BusFault_Handler(void){
	printf("bus handler");
	while(1);
}


void HardFault_Handler(void){
	printf("hard handler");
	while(1);
}
