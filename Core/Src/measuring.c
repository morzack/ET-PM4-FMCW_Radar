/*
 * Obtain data from ACDs + write to DAC to generate sweep
 *
 * Author:	John Kesler	<keslejoh@students.zhaw.ch>
 * 			Linus Leuch	<leuchlin@students.zhaw.ch>
 * 			Simon Meli	<melisim1@students.zhaw.ch>
 *
 * Timers:
 * 			TIM8	used for FMCW measurement
 * 			TIM2	used for DOPP measurement
 * 			TIM5	used for DAC FMCW sweep
 * ADCs/pins:
 * 			PC1 -> ADC1 -> DMA str4 -> DOPP input
 * 			PC3 -> ADC2 -> DMA str4 -> DOPP quadrature input
 * 			PC5 -> ADC1 -> DMA str4 -> FMCW input
 * 			(fmcw is actually read in dual mode w/ PC3 as the other)
 * 			(pc3 is just ignored during fmcw signal processing)
 * DAC/pin:
 * 			DAC->DHR12R2 -> DAC_OUT2 -> PA5
 *
 * DMA2 Stream 4 is used for all acquisitions.
 */

#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"

#include "measuring.h"

static uint32_t DAC_sample = 0; // value for DAC FMCW sweep

bool FMCW_MEAS_ready = false;
uint32_t FMCW_ADC_samples[FMCW_ADC_SAMPLE_COUNT * 2];

bool MEAS_data1_ready = false; ///< New data from ADC 1 is ready
bool MEAS_data3_ready = false; ///< New data from ADC 3 is ready
bool ACC_data_rdy = false;	   ///< Accumulator data is ready

bool MEAS_DOPP_ready = false;
uint32_t ADC_DOPP_samples[DOPP_ADC_SAMPLES * 2]; // needs zero padding later

/** ***************************************************************************
 * @brief Configure GPIOs in analog mode.
 *
 * @note The input number for the ADCs is not equal to the GPIO pin number!
 * - ADC123_IN11 = GPIO PC1
 * - ADC123_IN13 = GPIO PC3
 * - ADC12_IN15 = GPIO PC5
 * - DAC_OUT2 = GPIO PA5 (= same GPIO as ADC12_IN5)
 *****************************************************************************/
void MEAS_GPIO_analog_init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();			 // Enable Clock for GPIO port A
	GPIOA->MODER |= (GPIO_MODER_MODER5_Msk); // Analog mode for PA5 ADC12_IN5

	__HAL_RCC_GPIOC_CLK_ENABLE();			 // Enable Clock for GPIO port C
	GPIOC->MODER |= (GPIO_MODER_MODER1_Msk); // Analog mode for PC1 = ADC123_IN11
	GPIOC->MODER |= (GPIO_MODER_MODER3_Msk); // Analog mode for PC3 = ADC123_IN13
	GPIOC->MODER |= (GPIO_MODER_MODER5_Msk); // Analog mode for PC5 = ADC12_IN15
}

/** ***************************************************************************
 * @brief Configure the timer to trigger the ADC(s)
 *
 * @note For debugging the timer interrupt might be useful
 *****************************************************************************/
void MEAS_timer_init(void)
{

	// DOPP timer
	__HAL_RCC_TIM2_CLK_ENABLE();   // Enable Clock for TIM2
	TIM2->PSC = DOPP_TIM_PRESCALE; // Prescaler for clock freq.
	TIM2->ARR = DOPP_TIM_TOP;	   // Auto reload = counter top value
	TIM2->CR2 |= TIM_CR2_MMS_1;	   // TRGO on update

	// DAC timer
	__HAL_RCC_TIM5_CLK_ENABLE();
	TIM5->PSC = DAC_TIM_PRESCALE;
	TIM5->ARR = DAC_TIM_TOP;
	TIM5->DIER |= TIM_DIER_UIE;
	NVIC_ClearPendingIRQ(TIM5_IRQn);
	NVIC_EnableIRQ(TIM5_IRQn);

	// FMCW timer (TIM8 -- 16bit advanced, which is different from TIM2. still works :) )
	__HAL_RCC_TIM8_CLK_ENABLE();
	TIM8->PSC = FMCW_TIM_PRESCALER;
	TIM8->ARR = FMCW_TIM_TOP;
	TIM8->CR2 |= TIM_CR2_MMS_1;
}

void DAC_reset(void)
{
	RCC->APB1RSTR |= RCC_APB1RSTR_DACRST;  // Reset the DAC
	RCC->APB1RSTR &= ~RCC_APB1RSTR_DACRST; // Release reset of the DAC
}

/*
 * initialize DAC (shouuld be called once)
 */
void DAC_init(void)
{
	__HAL_RCC_DAC_CLK_ENABLE(); // Enable Clock for DAC
	DAC->CR |= DAC_CR_EN2;		// Enable DAC output 2
}

void DAC_increment(void)
{
	DAC_sample += DAC_INCREMENT; // Increment DAC output
	if (DAC_sample >= (1UL << DAC_RESOLUTION))
	{
		DAC_sample = 0;
		TIM5->CR1 &= ~TIM_CR1_CEN; // Disable timer
	}							   // Go to 0
	DAC->DHR12R2 = DAC_sample;	   // Write new DAC output value
}

void TIM5_IRQHandler(void)
{
	TIM5->SR &= ~TIM_SR_UIF; // Clear pending interrupt flag
	DAC_increment();		 // timer will be toggled off when bounds hit
}

/*
 * Resets the ADCs and the ADC-related timers
 */
void ADC_reset(void)
{
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;  // Reset ADCs
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST; // Release reset of ADCs
	TIM2->CR1 &= ~TIM_CR1_CEN;			   // Disable timers
	TIM8->CR1 &= ~TIM_CR1_CEN;
}

/** ***************************************************************************
 * @brief reads value from GPIO PG2 (status pin charging IC)
 *
 * @note The pin is LOW when battery is charging
 *****************************************************************************/
bool batteryStatus(void)
{
	__HAL_RCC_GPIOG_CLK_ENABLE();				   // Enable Clock for GPIO port G
	GPIOG->MODER &= ~GPIO_MODER_MODER2;			   // Reset mode for PG2
	GPIOG->MODER |= (0u << GPIO_MODER_MODER2_Pos); // Set PG2 as input
	uint32_t value = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_2);

	if (value)
	{
		return false;
	}
	else
	{
		return true;
	}
}

void FMCW_ADC_scan_init(void)
{
	__HAL_RCC_ADC1_CLK_ENABLE();				   // Enable Clock for ADC1
	__HAL_RCC_ADC2_CLK_ENABLE();				   // Enable Clock for ADC2
	ADC->CCR |= ADC_CCR_DMA_1;					   // Enable DMA mode 2 = dual DMA
	ADC->CCR |= ADC_CCR_MULTI_1 | ADC_CCR_MULTI_2; // ADC1 and ADC2 simultan.
	ADC1->CR2 |= (1UL << ADC_CR2_EXTEN_Pos);	   // En. ext. trigger on rising e.
	ADC1->CR2 |= (0b1110 << ADC_CR2_EXTSEL_Pos);   // Timer 8 TRGO event -- p398 STM32 ref man
	ADC1->SQR3 |= (15UL << ADC_SQR3_SQ1_Pos);	   // Input 11 = first conversion
	ADC2->SQR3 |= (13UL << ADC_SQR3_SQ1_Pos);	   // Input 13 = first conversion
	__HAL_RCC_DMA2_CLK_ENABLE();				   // Enable Clock for DMA2

	DMA2_Stream4->CR &= ~DMA_SxCR_EN; // Disable the DMA stream 4
	while (DMA2_Stream4->CR & DMA_SxCR_EN)
	{
		;
	}								 // Wait for DMA to finish
	DMA2->HIFCR |= DMA_HIFCR_CTCIF4; // Clear transfer complete interrupt fl.

	DMA2_Stream4->CR |= (0UL << DMA_SxCR_CHSEL_Pos); // Select channel 0
	DMA2_Stream4->CR |= DMA_SxCR_PL_1;				 // Priority high
	DMA2_Stream4->CR |= DMA_SxCR_MSIZE_1;			 // Memory data size = 32 bit
	DMA2_Stream4->CR |= DMA_SxCR_PSIZE_1;			 // Peripheral data size = 32 bit
	DMA2_Stream4->CR |= DMA_SxCR_MINC;				 // Increment memory address pointer
	DMA2_Stream4->CR |= DMA_SxCR_TCIE;				 // Transfer complete interrupt enable
	DMA2_Stream4->NDTR = FMCW_ADC_SAMPLE_COUNT;		 // Number of data items to transfer
	DMA2_Stream4->PAR = (uint32_t)&ADC->CDR;		 // Peripheral register address
	DMA2_Stream4->M0AR = (uint32_t)FMCW_ADC_samples; // Buffer memory loc. address
}

void FMCW_ADC_scan_start(void)
{
	DMA2_Stream4->CR |= DMA_SxCR_EN;		 // Enable DMA
	NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn); // Clear pending DMA interrupt
	NVIC_EnableIRQ(DMA2_Stream4_IRQn);		 // Enable DMA interrupt in the NVIC
	ADC1->CR2 |= ADC_CR2_ADON;				 // Enable ADC1
	ADC2->CR2 |= ADC_CR2_ADON;				 // Enable ADC2
	TIM8->CR1 |= TIM_CR1_CEN;				 // Enable timer
}

void ADC_DOPP_scan_init(void)
{
	// scan inputs from PC1/PC3 into memory
	// PC1 -> ADC1 -> DMA ch0str4
	// PC3 -> ADC2 -> DMA ch1str3
	__HAL_RCC_ADC1_CLK_ENABLE();				   // Enable Clock for ADC1
	__HAL_RCC_ADC2_CLK_ENABLE();				   // Enable Clock for ADC2
	ADC->CCR |= ADC_CCR_DMA_1;					   // Enable DMA mode 2 = dual DMA
	ADC->CCR |= ADC_CCR_MULTI_1 | ADC_CCR_MULTI_2; // ADC1 and ADC2 simultan.
	ADC1->CR2 |= (1UL << ADC_CR2_EXTEN_Pos);	   // En. ext. trigger on rising e.
	ADC1->CR2 |= (6UL << ADC_CR2_EXTSEL_Pos);	   // Timer 2 TRGO event
	ADC1->SQR3 |= (11UL << ADC_SQR3_SQ1_Pos);	   // Input 11 = first conversion
	ADC2->SQR3 |= (13UL << ADC_SQR3_SQ1_Pos);	   // Input 13 = first conversion
	__HAL_RCC_DMA2_CLK_ENABLE();				   // Enable Clock for DMA2

	DMA2_Stream4->CR &= ~DMA_SxCR_EN; // Disable the DMA stream 4
	while (DMA2_Stream4->CR & DMA_SxCR_EN)
	{
		;
	}								 // Wait for DMA to finish
	DMA2->HIFCR |= DMA_HIFCR_CTCIF4; // Clear transfer complete interrupt fl.

	DMA2_Stream4->CR |= (0UL << DMA_SxCR_CHSEL_Pos); // Select channel 0
	DMA2_Stream4->CR |= DMA_SxCR_PL_1;				 // Priority high
	DMA2_Stream4->CR |= DMA_SxCR_MSIZE_1;			 // Memory data size = 32 bit
	DMA2_Stream4->CR |= DMA_SxCR_PSIZE_1;			 // Peripheral data size = 32 bit
	DMA2_Stream4->CR |= DMA_SxCR_MINC;				 // Increment memory address pointer
	DMA2_Stream4->CR |= DMA_SxCR_TCIE;				 // Transfer complete interrupt enable
	DMA2_Stream4->NDTR = DOPP_ADC_SAMPLES;			 // Number of data items to transfer
	DMA2_Stream4->PAR = (uint32_t)&ADC->CDR;		 // Peripheral register address
	DMA2_Stream4->M0AR = (uint32_t)ADC_DOPP_samples; // Buffer memory loc. address
}

void ADC_DOPP_scan_start(void)
{
	DMA2_Stream4->CR |= DMA_SxCR_EN;		 // Enable DMA
	NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn); // Clear pending DMA interrupt
	NVIC_EnableIRQ(DMA2_Stream4_IRQn);		 // Enable DMA interrupt in the NVIC
	ADC1->CR2 |= ADC_CR2_ADON;				 // Enable ADC1
	ADC2->CR2 |= ADC_CR2_ADON;				 // Enable ADC2
	TIM2->CR1 |= TIM_CR1_CEN;				 // Enable timer
}

/** ***************************************************************************
 * @brief Interrupt handler for DMA2 Stream4
 *
 * Here the interrupt handler is used together with ADC1 and ADC2
 * in dual mode where they sample simultaneously.
 * @n The samples from both ADCs packed in a 32 bit word have been transfered
 * to memory by the DMA2 and are ready for unpacking.
 * @note In dual ADC mode two values are combined (packed) in a single uint32_t
 * ADC_CDR[31:0] = ADC2_DR[15:0] | ADC1_DR[15:0]
 * and are therefore extracted before further processing.
 *****************************************************************************/
void DMA2_Stream4_IRQHandler(void)
{
	if (DMA2->HISR & DMA_HISR_TCIF4)
	{											 // Stream4 transfer compl. interrupt f.
		NVIC_DisableIRQ(DMA2_Stream4_IRQn);		 // Disable DMA interrupt in the NVIC
		NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn); // Clear pending DMA interrupt
		DMA2_Stream4->CR &= ~DMA_SxCR_EN;		 // Disable the DMA
		while (DMA2_Stream4->CR & DMA_SxCR_EN)
		{
			;
		}								 // Wait for DMA to finish
		DMA2->HIFCR |= DMA_HIFCR_CTCIF4; // Clear transfer complete interrupt fl.
		TIM2->CR1 &= ~TIM_CR1_CEN;		 // Disable timer
		TIM8->CR1 &= ~TIM_CR1_CEN;		 // Disable timer
		ADC1->CR2 &= ~ADC_CR2_ADON;		 // Disable ADC1
		ADC2->CR2 &= ~ADC_CR2_ADON;		 // Disable ADC2
		ADC->CCR &= ~ADC_CCR_DMA_1;		 // Disable DMA mode

		// TODO use different interleaving for different modes
		/* Extract combined samples for current mode */
		for (int32_t i = FMCW_ADC_SAMPLE_COUNT - 1; i >= 0; i--)
		{
			FMCW_ADC_samples[2 * i + 1] = (FMCW_ADC_samples[i] >> 16);
			FMCW_ADC_samples[2 * i] = (FMCW_ADC_samples[i] & 0xffff);
		}
		FMCW_MEAS_ready = true;

		for (int32_t i = DOPP_ADC_SAMPLES - 1; i >= 0; i--)
		{
			ADC_DOPP_samples[2 * i + 1] = (ADC_DOPP_samples[i] >> 16);
			ADC_DOPP_samples[2 * i] = (ADC_DOPP_samples[i] & 0xffff);
		}
		MEAS_DOPP_ready = true;

		ADC_reset();
	}
}

void DAC_sweep_start(void)
{
	TIM5->CR1 |= TIM_CR1_CEN; // Enable timer
}