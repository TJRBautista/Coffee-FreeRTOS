//******************************************************************************
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"
//******************************************************************************

//******************************************************************************
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include "queue.h"
#include <stdbool.h>
//******************************************************************************

void vSenderTask1(void *pvParameters);
void vSenderTask2(void *pvParameters);
void vReceiverTask(void *pvParameters);

//void Delay(uint32_t val);

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS (4 bytes).*/

// define timing for events
const uint16_t LONG_PRESS_TIME = 2; // 3 seconds holding for long press.
const float MIN_PRESS_TIME = 0.05; // the min single press should need 0.05 second.
const float DOUBLE_CLICK_TIME = 0.5; // double press should be with in 0.5 second.
const uint16_t IDLE_TIME = 5;
const int SOUND_OUTPUT = 1; // output the sound for 1 second
xQueueHandle xQueue;
//******************************************************************************
int main(void)
{
	xQueue = xQueueCreate(100, sizeof(long));
	
	if (xQueue != NULL) {
		NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
		STM_EVAL_LEDInit(LED_BLUE);
		STM_EVAL_LEDInit(LED_GREEN);
		STM_EVAL_LEDInit(LED_ORANGE);
		STM_EVAL_LEDInit(LED_RED);
		
		xTaskCreate(vSenderTask1, (const char*) "Sender 1",
			STACK_SIZE_MIN, (void *)88, 1, NULL);
		xTaskCreate(vSenderTask2, (const char*) "Sender 2",
			STACK_SIZE_MIN, (void *)99, 1, NULL);
		xTaskCreate(vReceiverTask, (const char*) "Receiver 1",
			STACK_SIZE_MIN, NULL, 1, NULL);
		
		vTaskStartScheduler();
	}
}
//******************************************************************************

void vSenderTask1(void *pvParameters) {
	long lValueToSend;
	portBASE_TYPE xStatus;
	const portTickType xTicksToWait = 0;
	lValueToSend = (long) pvParameters;
	while(1) {
		// the queue shouldn't be full, xTicksToWait = 0
		xStatus = xQueueSendToBack (xQueue, &lValueToSend, xTicksToWait);
		if(xStatus!= pdPASS){
			STM_EVAL_LEDToggle(LED_RED);
		} else
			STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay(100/portTICK_RATE_MS);
		// Yield task before the end of the current time slice
		taskYIELD();
	}
}

void vSenderTask2(void *pvParameters) {
	long lValueToSend;
	portBASE_TYPE xStatus;
	const portTickType xTicksToWait = 0;
	lValueToSend = (long) pvParameters;
	while(1) {
		// the queue shouldn't be full, xTicksToWait = 0
		xStatus = xQueueSendToBack (xQueue, &lValueToSend, xTicksToWait);
		if(xStatus!= pdPASS){
			STM_EVAL_LEDToggle(LED_RED);
		} else
			STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay(50/portTICK_RATE_MS);
		// Yield task before the end of the current time slice
		taskYIELD();
	}
}

void vReceiverTask(void *pvParameters) {
	long lReceiveValue;
	portBASE_TYPE xStatus;
	const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
	
	while(1) {
		xStatus = xQueueReceive (xQueue, &lReceiveValue, xTicksToWait);
		if(xStatus == pdPASS){
			if(lReceiveValue == 88) {
				STM_EVAL_LEDToggle(LED_BLUE);
			} else if (lReceiveValue == 99)
				STM_EVAL_LEDToggle(LED_GREEN);
			
			
			//vTaskDelay(5000/portTICK_RATE_MS);
		}
	}
}
