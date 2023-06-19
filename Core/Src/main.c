/*
 * Sets up display + transitions microcontroller state
 *
 * Author:	John Kesler	<keslejoh@students.zhaw.ch>
 * 			Linus Leuch	<leuchlin@students.zhaw.ch>
 * 			Simon Meli	<melisim1@students.zhaw.ch>
 * Derived from code by:
 * 			Hanspeter Hochreutener <hhrt@zhaw.ch>
 */

#include <stdbool.h>

#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"

#include "main.h"
#include "pushbutton.h"
#include "render.h"
#include "measuring.h"
#include "calculation.h"
#include "display.h"

bool buzzer = true; ///< Toggle for Buzzer on or off

static void SystemClock_Config(void); ///< System Clock Configuration
static void gyro_disable(void);		  ///< Disable the onboard gyroscope
static void shutdown(void);			  ///< Shutdown the board
void buzz(void);					  ///< Play a short beep on the buzzer

int main(void)
{
	HAL_Init(); // Initialize the system

	SystemClock_Config(); // Configure system clocks

#ifdef FLIPPED_LCD
	BSP_LCD_Init_Flipped(); // Initialize the LCD for flipped orientation
#else
	BSP_LCD_Init(); // Initialize the LCD display
#endif
	BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
	BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
	BSP_LCD_DisplayOn();
	BSP_LCD_Clear(LCD_COLOR_WHITE);

	BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize()); // Touchscreen

	PB_init();		// Initialize the user pushbutton
	PB_enableIRQ(); // Enable interrupt on user pushbutton

	DAC_reset();
	DAC_init();

	BSP_LED_Init(LED3); // Toggles in while loop
	BSP_LED_Init(LED4); // Is toggled by user button

	gyro_disable(); // Disable gyro, use those analog inputs

	MEAS_GPIO_analog_init(); // Configure GPIOs in analog mode
	MEAS_timer_init();		 // Configure the timer

	for (int i = 0; i < FMCW_ADC_SAMPLE_COUNT / 2; i++)
	{
		fft_avg_vec_fmcw[i] = 0;
	}

	while (1)
	{						  // Infinitely loop in main function
		BSP_LED_Toggle(LED3); // Visual feedback when running

		/* Comment next line if touchscreen interrupt is enabled */
		// MENU_check_transition();

		// FMCW_ADC_scan_init();
		// DAC_sweep_start();
		// FMCW_ADC_scan_start();

		ADC_DOPP_scan_init();
		ADC_DOPP_scan_start();

		HAL_Delay(REFRESH_RATE / 2);

		DISPLAY_FFT_diagnosis();
		HAL_Delay(REFRESH_RATE / 2); // Wait or sleep
	}
}

/** ***************************************************************************
 * @brief System Clock Configuration
 *
 *****************************************************************************/
static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
	/* Configure the main internal regulator output voltage */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/* Initialize High Speed External Oscillator and PLL circuits */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
	/* Initialize gates and clock dividers for CPU, AHB and APB busses */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
	/* Initialize PLL and clock divider for the LCD */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
	/* Set clock prescaler for ADCs */
	ADC->CCR |= ADC_CCR_ADCPRE_0;
}

/** ***************************************************************************
 * @brief Disable the GYRO on the microcontroller board.
 *
 * @note MISO of the GYRO is connected to PF8 and CS to PC1.
 * @n Some times the GYRO goes into an undefined mode at startup
 * and pulls the MISO low or high thus blocking the analog input on PF8.
 * @n The simplest solution is to pull the CS of the GYRO low for a short while
 * which is done with the code below.
 * @n PF8 is also reconfigured.
 * @n An other solution would be to remove the GYRO
 * from the microcontroller board by unsoldering it.
 *****************************************************************************/
static void gyro_disable(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable Clock for GPIO port C
	/* Disable PC1 and PF8 first */
	GPIOC->MODER &= ~GPIO_MODER_MODER1;		 // Reset mode for PC1
	GPIOC->MODER |= GPIO_MODER_MODER1_0;	 // Set PC1 as output
	GPIOC->BSRR |= GPIO_BSRR_BR1;			 // Set GYRO (CS) to 0 for a short time
	HAL_Delay(10);							 // Wait some time
	GPIOC->MODER |= GPIO_MODER_MODER1_Msk;	 // Analog mode PC1 = ADC123_IN11
	__HAL_RCC_GPIOF_CLK_ENABLE();			 // Enable Clock for GPIO port F
	GPIOF->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED8; // Reset speed of PF8
	GPIOF->AFR[1] &= ~GPIO_AFRH_AFSEL8;		 // Reset alternate func. of PF8
	GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD8;		 // Reset pulup/down of PF8
	HAL_Delay(10);							 // Wait some time
	GPIOF->MODER |= GPIO_MODER_MODER8_Msk;	 // Analog mode for PF6 = ADC3_IN4
}

/** ***************************************************************************
 * @brief shuts the cable monitor down
 *
 *****************************************************************************/
static void shutdown(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();		 // Enable Clock for GPIO port C
	GPIOC->MODER &= ~GPIO_MODER_MODER8;	 // Reset mode for PC8
	GPIOC->MODER |= GPIO_MODER_MODER8_0; // Set PC8 as output
	GPIOC->BSRR = GPIO_BSRR_BR8;		 // Bit reset to turn off device
}

/** ***************************************************************************
 * @brief Plays short beep on buzzer
 *
 *****************************************************************************/
void buzz(void)
{
	if (buzzer)
	{
		__HAL_RCC_GPIOG_CLK_ENABLE();		 // Enable Clock for GPIO port G
		GPIOG->MODER &= ~GPIO_MODER_MODER3;	 // Reset mode for PG3
		GPIOG->MODER |= GPIO_MODER_MODER3_0; // Set PG3 as output
		GPIOG->BSRR = GPIO_BSRR_BS3;		 // Bit set to turn on buzzer
		HAL_Delay(10);
		GPIOG->BSRR = GPIO_BSRR_BR3; // Bit reset to turn off buzzer
	}
}
