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
#include "main.h"
#include <stdbool.h>
//******************************************************************************

// size option for coffee
const uint16_t SMALL = LED_ORANGE; // orange
const uint16_t MEDIUM = LED_RED; // red
const uint16_t EXTRA_LARGE = LED_BLUE; // blue

// define few timing for events
const uint16_t LONG_PRESS_TIME = 2; // 3 seconds holding for long press.
const float MIN_PRESS_TIME = 0.05; // the min single press should need 0.05 second.
const float DOUBLE_CLICK_TIME = 0.5; // double press should be with in 0.5 second.
const uint16_t IDLE_TIME = 5;
const int SOUND_OUTPUT = 1; // output the sound for 1 second

//coffee machine variables
int num_blink = 0;
int coffee_size = 0;
const int MAX_SIZE_OPTION = 3;

// predefine timing for coffee machine
uint16_t time_small = 2;
uint16_t time_medium = 4;
uint16_t time_ex_large = 6;
uint16_t new_num_click = 0;

// variables for sound
fir_8 filt;
bool output_sound = true;
int timer_for_sound = 0;
bool start_sound_timer = false;

// variables for servo
const uint32_t VALVE_OFF = 001;
const uint32_t VALVE_ESPRESSO = 1000;
const uint32_t VALVE_MILK = 1500;
const uint32_t VALVE_CHOCO = 2100;
uint32_t curValvePos = VALVE_OFF;

void initSound(void);
float updateFilter(fir_8* filt, float val);
void initFilter(fir_8* theFilter);

void initServo(void);
void InitPWMTimer4(void);
void SetupPWM(void);

void vLedBlinkBlue(void *pvParameters);
void vLedBlinkRed(void *pvParameters);
void vLedBlinkGreen(void *pvParameters);
void vLedBlinkOrange(void *pvParameters);
void vButtonTask(void *pvParameters);
void vSoundTask(void *pvParameters);
void vServoTask(void *pvParameters);

//volatile uint32_t msTicks; //counts 1ms timeTicks
//void SysTick_Handler(void) {
//	msTicks++;
//}

////Delays number of Systicks (happens every 1 ms)
//static void Delay(__IO uint32_t dlyTicks){
//	uint32_t curTicks = msTicks;
//	while ((msTicks - curTicks) < dlyTicks);
//}
//void setSysTick(void){
//	if (SysTick_Config(SystemCoreClock / 1000)) {
//		while (1){};
//	}
//}

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS (4 bytes).*/

TaskHandle_t xHandleBlue = NULL;

//******************************************************************************
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32f4xx.s) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32f4xx.c file
	 */
	
	/*!< Most systems default to the wanted configuration, with the noticeable 
		exception of the STM32 driver library. If you are using an STM32 with 
		the STM32 driver library then ensure all the priority bits are assigned 
		to be preempt priority bits by calling 
		NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); before the RTOS is started.
	*/
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);
	initSound();
	initFilter(&filt);
	
//	setSysTick();
	initServo();
	InitPWMTimer4();
	SetupPWM();
	
	xTaskCreate( vLedBlinkBlue, (const char*)"Led Blink Task Blue", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, &xHandleBlue );
//	xTaskCreate( vLedBlinkRed, (const char*)"Led Blink Task Red", 
//		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
//	xTaskCreate( vLedBlinkGreen, (const char*)"Led Blink Task Green", 
//		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
//	xTaskCreate( vLedBlinkOrange, (const char*)"Led Blink Task Orange", 
//		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vButtonTask, (const char*)"Button Task",
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL);
//	xTaskCreate( vSoundTask, (const char*)"Sound Task",
//		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL);
//	xTaskCreate( vServoTask, (const char*)"Servo Task",
//		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL);
	
	vTaskStartScheduler();
}

void vLedBlinkBlue(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_BLUE);
		vTaskDelay( 100 / portTICK_RATE_MS );
	}
}

void vLedBlinkRed(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_RED);
		vTaskDelay( 500 / portTICK_RATE_MS );
	}
}

void vLedBlinkGreen(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_GREEN);
		vTaskDelay( 200 / portTICK_RATE_MS );
	}
}

void vLedBlinkOrange(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay( 300 / portTICK_RATE_MS );
	}
}

void vButtonTask(void *pvParameters)
{
//	bool cancelled = false;
//	for(;;)
//	{
//		if (STM_EVAL_PBGetState(BUTTON_USER)) 
//		{
//			if(!cancelled) {
//				cancelled = true;
//				// suspend the task that point to its task handle pointer
//				vTaskSuspend(xHandleBlue);
//			} else {
//				cancelled = false;
//				vTaskResume(xHandleBlue);
//			}
//		}
//	}
	while(1) {
		if (STM_EVAL_PBGetState(BUTTON_USER)) {
			output_sound = true;
			curValvePos = VALVE_ESPRESSO;
		} else {
			output_sound = false;
			curValvePos = VALVE_OFF;
		}
	}
}

void vSoundTask(void *pvParameters) {
	while(1) {
		if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE)) {
			SPI_I2S_SendData(CODEC_I2S, sample);

			if (output_sound) {
				//only update on every second sample to insure that L & R ch. have the same sample value
				if (sampleCounter & 0x00000001)
				{
					sawWave += NOTEFREQUENCY;
					if (sawWave > 1.0)
						sawWave -= 2.0;

					filteredSaw = updateFilter(&filt, sawWave);
					sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
				}
				sampleCounter++;
			} else {
				sample = 0;
				sawWave = 0;
			}
		} 
	}
}

void vServoTask(void *pvParameters) {
	while(1) {
		TIM4->CCR1 = curValvePos;
	}
}

//******************************************************************************

void initSound() {
	SystemInit();

	//enables GPIO clock for PortD
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	codec_init();
	codec_ctrl_init();

	I2S_Cmd(CODEC_I2S, ENABLE);

	initFilter(&filt);
}

// the following code is for sound
// a very crude FIR lowpass filter
float updateFilter(fir_8* filt, float val) {
	uint16_t valIndex;
	uint16_t paramIndex;
	float outval = 0.0;

	valIndex = filt->currIndex;
	filt->tabs[valIndex] = val;

	for (paramIndex=0; paramIndex<8; paramIndex++)
	{
		outval += (filt->params[paramIndex]) * (filt->tabs[(valIndex+paramIndex)&0x07]);
	}

	valIndex++;
	valIndex &= 0x07;

	filt->currIndex = valIndex;

	return outval;
}

void initFilter(fir_8* theFilter) {
	uint8_t i;

	theFilter->currIndex = 0;

	for (i=0; i<8; i++)
		theFilter->tabs[i] = 0.0;

	theFilter->params[0] = 3;
	theFilter->params[1] = 3;
	theFilter->params[2] = 3;
	theFilter->params[3] = 3;
	theFilter->params[4] = 3;
	theFilter->params[5] = 3;
	theFilter->params[6] = 3;
	theFilter->params[7] = 3;
}

void initServo(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	//Initialize PB6 (TIM4 Ch1) and PB7 (TIM4 Ch2)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // GPIO_High_Speed
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// Assign Alternate Functions to pins
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
}
	
void InitPWMTimer4(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//TIM4 Clock Enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	// Time Base Configuration for 50Hz
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 84 -1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM4, ENABLE);
}
	
void SetupPWM(void) {
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //Set output capture as PWM mode
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //Enable output compare
	TIM_OCInitStructure.TIM_Pulse = 0; // Initial duty cycle at 0%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // HIGH output compare active
	// Set the output capture channel 1 and 2 (upto 4)
	TIM_OC1Init(TIM4, &TIM_OCInitStructure); // Channel 1
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure); // Channel 2
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
}
