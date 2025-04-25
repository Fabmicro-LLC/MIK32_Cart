
#define ADC_CONFIG_SAH_TIME_S          8
#define ADC_CONFIG_SAH_TIME_M          (0x3F << ADC_CONFIG_SAH_TIME_S)

#include "mik32_hal_adc.h"
#include "mik32_hal_pcc.h"
#include "mik32_hal_scr1_timer.h"
#include "mik32_hal_gpio.h"
#include "mik32_hal_irq.h"
#include "mik32_hal_timer32.h"

#include "uart_lib.h"
#include "xprintf.h"
#include "ir_decoder.h"

#define	DEBUG	1

#define	MIK32V2

//#define	ADC_OFFSET	172
#define	ADC_OFFSET	0	
#define	ADC_THRESHOLD	1500	// Stop if ADC is greater than this value	
#define	TIMER_FREQ	100	// Motor PWM and Servo PDM frequency, Hz
#define	STEERING_MAX	1800	// Steering servo max PDM value (60 degree)
#define	STEERING_MID	1500	// Steering servo middle PDM value (0 degree)
#define	STEERING_MIN	1200	// Steering servo max PDM value (-60 degree)
#define	STEERING_OFFSET	30	// Error correction

ADC_HandleTypeDef hadc;
TIMER32_HandleTypeDef htimer32_1;
TIMER32_CHANNEL_HandleTypeDef htimer32_channel1; // Motor Forward
TIMER32_CHANNEL_HandleTypeDef htimer32_channel3; // Motor Backward
TIMER32_CHANNEL_HandleTypeDef htimer32_channel2; // Steering Servo


void SystemClock_Config(void);
static void GPIO_Init(void);
static void ADC_Init(void);
static void Scr1_Timer_Init(void);
static void Timer32_1_Init(void);

int32_t adc_avg = 0;
uint32_t count = 0;
int32_t motor_state = 0;	// 0 - stop, 1 - forward, -1 - backward
uint32_t timer_top = OSC_SYSTEM_VALUE / 8 / TIMER_FREQ;
int32_t motor_pwm = OSC_SYSTEM_VALUE / 8 / TIMER_FREQ / 2;
uint32_t steering_pdm_us = STEERING_MID - STEERING_OFFSET; // 1500us - center, 1100us - -90 degree, 1900us - +90 degree

void DelayMs(uint32_t ms)
{
	uint32_t t0 = SCR1_TIMER->MTIME;

	while((SCR1_TIMER->MTIME - t0) / 32000 < ms);
}


int main()
{

	SystemClock_Config();
	Scr1_Timer_Init();

	UART_Init(UART_0, OSC_SYSTEM_VALUE/115200, UART_CONTROL1_TE_M | UART_CONTROL1_M_8BIT_M, 0, 0);

	GPIO_Init();
	ADC_Init();
	Timer32_1_Init();

	HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, (OSC_SYSTEM_VALUE / 8 / 1000000) * steering_pdm_us);

	#if(DEBUG)
	xprintf("IR-Cart Control Software. Copyright (C) 2025, Fabmicro, LLC., Tyumen, Russia.\r\n");
	#endif

	/* Разрешить прерывания по уровню для линии EPIC GPIO_IRQ */
	HAL_EPIC_MaskLevelSet(HAL_EPIC_GPIO_IRQ_MASK);

	/* Разрешить глобальные прерывания */
	HAL_IRQ_EnableInterrupts();

	int16_t adc_corrected_value, adc_raw_value;

	while (1) {

	 	HAL_ADC_SINGLE_AND_SET_CH(hadc.Instance, 0);

		/* Ожидание и чтение актуальных данных (режим одиночного преобразования) */
		adc_raw_value = HAL_ADC_WaitAndGetValue(&hadc);

		adc_corrected_value = adc_raw_value - ADC_OFFSET;
		adc_avg = (adc_avg + adc_corrected_value) / 2; /* Скользящее среднее */
		//adc_avg = (adc_avg * 14 + adc_corrected_value * 2) / 16; /* Сглаживание по альфа-бета */

		if(count % 2000 == 0) {
			/* Печатать усредненное значение после 1000 преобразований */
			#if(DEBUG)
			xprintf("Time: %08X:%08X,\tADC: %4d %4d %4d (%d.%03d V)\t",
				SCR1_TIMER->MTIMEH, SCR1_TIMER->MTIME,
				adc_raw_value, adc_corrected_value, adc_avg,
				((adc_avg * 1200) / 4095) / 1000,
				((adc_avg * 1200) / 4095) % 1000);

			xprintf("\r\n");
			#endif
		}

		/* Мигать зеленым светодиодом */
		if(count % 4000 == 0) {
			GPIO_0->OUTPUT ^= GPIO_PIN_9; // Инвертируем GPIO светодиода
		}

		if(count % 100 == 0) {
			if(adc_avg >= ADC_THRESHOLD && motor_state == 1) {
				motor_state = -1;

				#if(DEBUG)
				xprintf("OBSTACLE!!\r\n");
				#endif

				// Reverse
				HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, 0);
				HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, timer_top);

				#if(1)
				DelayMs(1000);

				// Stop
				HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, 0);
				HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, 0);
				#endif


				// Turn wheels left
				steering_pdm_us = (STEERING_MAX - STEERING_OFFSET);
				HAL_Timer32_Channel_OCR_Set(&htimer32_channel2,
					(OSC_SYSTEM_VALUE / 8 / 1000000) * steering_pdm_us);

				// Reverse 
				HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, 0);
				HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, motor_pwm);

			}
		}

		if(ir_decoder_command_ready) {
			ir_decoder_command_ready = 0;
	
			switch(ir_decoder_command) {
				case 0x19E60707: { // Power - Stop 
						motor_state = 0;
						#if(DEBUG)
						xprintf("MOTOR STOP\r\n");
						#endif
						break;
					}
					
				case 0x9F600707: { // UP - Move Forward
						motor_state = 1;
						#if(DEBUG)
						xprintf("MOTOR FORWARD: %d\r\n", motor_pwm);
						#endif
						break;
					}
					
				case 0x9E610707: { // Down - Move Backward
						motor_state = -1;
						#if(DEBUG)
						xprintf("MOTOR BACKWARD: %d\r\n", motor_pwm);
						#endif
						break;
					}
					
				case 0xF8070707: { // Volume+ - Increase speed
						
						motor_pwm += timer_top / 10;

						if(motor_pwm > timer_top)
							motor_pwm = timer_top;

						#if(DEBUG)
						xprintf("MOTOR SPEED UP: %d\r\n", motor_pwm);
						#endif
						break;
					}
					
				case 0xF40B0707: { // Volume- - Decrease speed 
						motor_pwm -= timer_top / 10;

						if(motor_pwm < 0)
							motor_pwm = 0;

						#if(DEBUG)
						xprintf("MOTOR SPEED DOWN: %d\r\n", motor_pwm);
						#endif
						break;
					}
					
				case 0x9A650707: { // Left - Rotate left 5%
						steering_pdm_us += (STEERING_MAX - STEERING_MIN) / 40;

						if(steering_pdm_us > (STEERING_MAX - STEERING_OFFSET))
							steering_pdm_us = STEERING_MAX - STEERING_OFFSET;
						
						HAL_Timer32_Channel_OCR_Set(&htimer32_channel2,
							(OSC_SYSTEM_VALUE / 8 / 1000000) * steering_pdm_us);

						
						#if(DEBUG)
						xprintf("LEFT: %d us\r\n", steering_pdm_us);
						#endif
						break;
					}

				case 0x9D620707: { // Right - Rotate right 5%
						steering_pdm_us -= (STEERING_MAX - STEERING_MIN) / 40;

						if(steering_pdm_us < (STEERING_MIN - STEERING_OFFSET))
							steering_pdm_us = STEERING_MIN - STEERING_OFFSET;
						
						HAL_Timer32_Channel_OCR_Set(&htimer32_channel2,
							(OSC_SYSTEM_VALUE / 8 / 1000000) * steering_pdm_us);

						#if(DEBUG)
						xprintf("RIGHT: %d us\r\n", steering_pdm_us);
						#endif
						break;
					}

				case 0x86790707: { // Home - Center wheels, keep moving
						steering_pdm_us = STEERING_MID - STEERING_OFFSET;

						HAL_Timer32_Channel_OCR_Set(&htimer32_channel2,
							(OSC_SYSTEM_VALUE / 8 / 1000000) * steering_pdm_us);

						#if(DEBUG)
						xprintf("CENTERED: %d us\r\n", steering_pdm_us);
						#endif

						motor_state = 0;
						HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, 0);
						HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, 0);

						#if(DEBUG)
						xprintf("MOTOR STOP\r\n");
						#endif

						break;
					}

				case 0x97680707: { // Enter - Center wheels and stop
						steering_pdm_us = STEERING_MID - STEERING_OFFSET;

						HAL_Timer32_Channel_OCR_Set(&htimer32_channel2,
							(OSC_SYSTEM_VALUE / 8 / 1000000) * steering_pdm_us);

						#if(DEBUG)
						xprintf("CENTERED: %d us\r\n", steering_pdm_us);
						#endif

						break;
					}


				default: {}
			}


			if(motor_state == 0) {
				HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, 0);
				HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, 0);
			}

			if(motor_state == 1) {
				HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, motor_pwm);
				HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, 0);
			}

			if(motor_state == -1) {
				HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, 0);
				HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, motor_pwm);
			}

			//HAL_Timer32_Value_Clear(&htimer32_1);

		}


		count++;
	}
}

void SystemClock_Config(void)
{
	PCC_InitTypeDef PCC_OscInit = {0};

	PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
	PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
	PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
	PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
	PCC_OscInit.AHBDivider = 0;
	PCC_OscInit.APBMDivider = 0;
	PCC_OscInit.APBPDivider = 0;
	PCC_OscInit.HSI32MCalibrationValue = 128;
	PCC_OscInit.LSI32KCalibrationValue = 128;
	PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
	PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
	HAL_PCC_Config(&PCC_OscInit);
}


static void Scr1_Timer_Init(void)
{   
	/* Источник тактирования */
	/* Делитель частоты 10-битное число */

	HAL_SCR1_Timer_Init(HAL_SCR1_TIMER_CLKSRC_INTERNAL, 0); // 0 - 32 MHz
}


static void ADC_Init(void)
{
	hadc.Instance = ANALOG_REG;

	/* Выбор канала АЦП для которого будет выполнена инициализация */
	hadc.Init.Sel = ADC_CHANNEL0; // P1.5

	/* Выбор источника опорного напряжения: «1» - внешний; «0» - встроенный */
	hadc.Init.EXTRef = ADC_EXTREF_OFF;	
	//hadc.Init.EXTRef = ADC_EXTREF_ON;

	/* Выбор внешнего опорного напряжения: «1» - внешний вывод; «0» - настраиваемый ОИН */
	hadc.Init.EXTClb = ADC_EXTCLB_ADCREF;

	/* Функция HAL_ADC_Init() сама переведет соответствующий вывод GPIO в режим ANALOG */
	HAL_ADC_Init(&hadc);
}

void GPIO_Init()
{
	/* Включить тактирование портов GPIO */
	__HAL_PCC_GPIO_0_CLK_ENABLE();
	__HAL_PCC_GPIO_1_CLK_ENABLE();
	__HAL_PCC_GPIO_2_CLK_ENABLE();
	__HAL_PCC_GPIO_IRQ_CLK_ENABLE();

 	__HAL_PCC_PAD_CONFIG_CLK_ENABLE();   
	__HAL_PCC_EPIC_CLK_ENABLE();

	/* Настроить сигнал P1.15 (User Button) как входной сигнал GPIO */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_INPUT;
	GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
	HAL_GPIO_Init(GPIO_1, &GPIO_InitStruct);

	/* Настроить сигнал P2.1 (IR Receiver) как входной сигнал GPIO */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	HAL_GPIO_Init(GPIO_2, &GPIO_InitStruct);

	/* Настроить сигнал P0.9 (LED Green) как выходной сигнал GPIO */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
	GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
	HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);

	/* Включить прерывания от P1.15 (User Button) как IRQ Line 3 по спаду */
	HAL_GPIO_InitInterruptLine(GPIO_MUX_PORT1_15_LINE_3, GPIO_INT_MODE_FALLING);

	/* Включить прерывания от P2.1 (IR Receiver) как IRQ Line 1 по спаду */
	HAL_GPIO_InitInterruptLine(GPIO_MUX_PORT2_1_LINE_1, GPIO_INT_MODE_CHANGE);
}

static void Timer32_1_Init(void)
{
	htimer32_1.Instance = TIMER32_1;
	htimer32_1.Top = 0;
	htimer32_1.State = TIMER32_STATE_DISABLE;
	htimer32_1.Clock.Source = TIMER32_SOURCE_PRESCALER;
	htimer32_1.Clock.Prescaler = 7; // Делить на 8
	htimer32_1.InterruptMask = 0;
	htimer32_1.CountMode = TIMER32_COUNTMODE_FORWARD;
	HAL_Timer32_Init(&htimer32_1);
	
	htimer32_channel1.TimerInstance = htimer32_1.Instance;
	htimer32_channel1.ChannelIndex = TIMER32_CHANNEL_1;
	htimer32_channel1.PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
	htimer32_channel1.Mode = TIMER32_CHANNEL_MODE_PWM;
	htimer32_channel1.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
	htimer32_channel1.OCR = 0;
	htimer32_channel1.Noise = TIMER32_CHANNEL_FILTER_OFF;
	HAL_Timer32_Channel_Init(&htimer32_channel1);
	HAL_Timer32_Channel_Enable(&htimer32_channel1);
	
	htimer32_channel2.TimerInstance = htimer32_1.Instance;
	htimer32_channel2.ChannelIndex = TIMER32_CHANNEL_2;
	htimer32_channel2.PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
	htimer32_channel2.Mode = TIMER32_CHANNEL_MODE_PWM;
	htimer32_channel2.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
	htimer32_channel2.OCR = 0;
	htimer32_channel2.Noise = TIMER32_CHANNEL_FILTER_OFF;
	HAL_Timer32_Channel_Init(&htimer32_channel2);
	HAL_Timer32_Channel_Enable(&htimer32_channel2);
	
	htimer32_channel3.TimerInstance = htimer32_1.Instance;
	htimer32_channel3.ChannelIndex = TIMER32_CHANNEL_3;
	htimer32_channel3.PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
	htimer32_channel3.Mode = TIMER32_CHANNEL_MODE_PWM;
	htimer32_channel3.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
	htimer32_channel3.OCR = 0;
	htimer32_channel3.Noise = TIMER32_CHANNEL_FILTER_OFF;
	HAL_Timer32_Channel_Init(&htimer32_channel3);
	HAL_Timer32_Channel_Enable(&htimer32_channel3);
	
	HAL_Timer32_Top_Set(&htimer32_1, timer_top);
	HAL_Timer32_Value_Clear(&htimer32_1);
	HAL_Timer32_Start(&htimer32_1);
}



void trap_handler()
{
    if (EPIC_CHECK_GPIO_IRQ()) {
        if (HAL_GPIO_LineInterruptState(GPIO_LINE_3)) {
		#if(DEBUG)
		xprintf("\r\nIRQ_LINE3_BUTTON\r\n");
		#endif
        }
        if (HAL_GPIO_LineInterruptState(GPIO_LINE_1)) {
		ir_decoder_irq(HAL_GPIO_ReadPin(GPIO_2, GPIO_PIN_1));
        }
        HAL_GPIO_ClearInterrupts();
    }

    /* Сброс прерываний */
    HAL_EPIC_Clear(0xFFFFFFFF);
}
