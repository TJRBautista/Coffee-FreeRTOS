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

// use for setup the clock speed for TIM2
// this timer is used for general purpose timing
const uint16_t TIMER_2_PRESCALER = 232;
const uint16_t TIMER_2_PERIOD = 2999;
const uint16_t TIMER_2_FREQUENCY = 120;

TIM_TimeBaseInitTypeDef timer_InitStructure;
void InitTimer_2() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  timer_InitStructure.TIM_Prescaler = TIMER_2_PRESCALER;
  timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timer_InitStructure.TIM_Period = TIMER_2_PERIOD;
  timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timer_InitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, & timer_InitStructure);
  TIM_Cmd(TIM2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void EnableTimer2Interrupt() {
  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( & nvicStructure);
}

// size option for coffee
const uint16_t SMALL = LED_ORANGE; // orange
const uint16_t MEDIUM = LED_RED; // red
const uint16_t EXTRA_LARGE = LED_BLUE; // blue

// define few timing for events
const uint16_t LONG_PRESS_TIME = 2; // 3 seconds holding for long press.
const float MIN_PRESS_TIME = 0.05; // the min single press should need 0.05 second.
const float DOUBLE_CLICK_TIME = 0.5; // double press should be with in 0.5 second.
const uint16_t IDLE_TIME = 5;
const float SOUND_OUTPUT = 0.5; // output the sound for 1 second

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
bool sound_init = true;
//bool start_sound_timer = false;

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

void vButtonTask(void *pvParameters);
void vSoundTask(void *pvParameters);
void vServoTask(void *pvParameters);
void vMainTask(void *pvParameters);


void TIM2_IRQHandler() {
  //Checks whether the TIM2 interrupt has occurred or not
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
			
		if(output_sound)
			{
			timer_for_sound++;
		}
		if(timer_for_sound > SOUND_OUTPUT * TIMER_2_FREQUENCY)
		{
			timer_for_sound = 0;
			output_sound = false;
		}
		
  }
}

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
	
	InitTimer_2();
	EnableTimer2Interrupt();
	
	initSound();
	initFilter(&filt);
	
	initServo();
	InitPWMTimer4();
	SetupPWM();
	
	xTaskCreate( vButtonTask, (const char*)"Button Task",
		STACK_SIZE_MIN, NULL, 0, NULL);
	xTaskCreate( vSoundTask, (const char*)"Sound Task",
		STACK_SIZE_MIN, NULL, 1, NULL);
	xTaskCreate( vServoTask, (const char*)"Servo Task",
		STACK_SIZE_MIN, NULL, 0, NULL);
	xTaskCreate( vMainTask, (const char*)"main project Task",
		STACK_SIZE_MIN, NULL, 0, NULL);
	
	
	vTaskStartScheduler();
}


void vButtonTask(void *pvParameters)
{
	while(1) {
		if (STM_EVAL_PBGetState(BUTTON_USER)) {
			STM_EVAL_LEDOn(LED_BLUE);
			output_sound = true;

			curValvePos = VALVE_ESPRESSO;
		} else {
			STM_EVAL_LEDOff(LED_BLUE);
			//output_sound = false;

			curValvePos = VALVE_OFF;
		}
	}
}

void vSoundTask(void *pvParameters) {
	while(1) {
		if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE)) {
			SPI_I2S_SendData(CODEC_I2S, sample);
			if (sampleCounter & 0x00000001) {
				sawWave += NOTEFREQUENCY;
				if (sawWave > 1.0)
					sawWave -= 2.0;

				filteredSaw = updateFilter(&filt, sawWave);
				sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
			}
			sampleCounter++;
		}
		if (output_sound) {
			if(!sound_init) {
				sound_init = true;
				initSound();
			}
		} else {
			GPIO_ResetBits(GPIOD, GPIO_Pin_4);
			sound_init = false;
			vTaskDelay(10/portTICK_RATE_MS);
		}
	}
}

void vMainTask(void *pvParameters) {
	while(1) {
		
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
