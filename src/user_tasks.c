#include "user_tasks.h"


void idle_task(void){

	while(1);
}

void task1_handler(void){
	while(1){
		printf("task1 is executing\n");
		LedToggle(GPIO_PIN_NB_12);
		task_delay(1000);
		LedToggle(GPIO_PIN_NB_12);
		task_delay(1000);
	}
}

void task2_handler(void){
	while(1){
		printf("task2 is executing\n");
		LedToggle(GPIO_PIN_NB_13);
		task_delay(500);
		LedToggle(GPIO_PIN_NB_13);
		task_delay(500);
	}
}

void task3_handler(void){
	while(1){
		printf("task3 is executing\n");
		LedToggle(GPIO_PIN_NB_14);
		task_delay(250);
		LedToggle(GPIO_PIN_NB_15);
		task_delay(250);
	}
}

void task4_handler(void){
	while(1){
		printf("task4 is executing\n");
		LedToggle(GPIO_PIN_NB_15);
		task_delay(125);
		LedToggle(GPIO_PIN_NB_15);
		task_delay(125);
	}
}

