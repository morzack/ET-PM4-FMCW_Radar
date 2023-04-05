/** ***************************************************************************
 * @file
 * @brief Measuring voltages with the ADC(s) in different configurations
 *
 *
 * Configures different ADC (Analog to Digital Converter) modes
 * ==============================================================
 *
 * - ADC in single conversion mode
 * - ADC combined with DMA (Direct Memory Access) to fill a buffer
 * - Scan mode = sequential sampling of two or three inputs by one ADC
 *
 * Peripherals @ref HowTo
 *
 * @image html demo_screenshot_board.jpg
 * 
 * 
 * @anchor HowTo
 * How to Configure the Peripherals: ADC, TIMER and DMA
 * ====================================================
 *
 * All the peripherals are accessed by writing to or reading from registers.
 * From the programmer’s point of view this is done exactly as
 * writing or reading the value of a variable.
 * @n Writing to a register configures the HW of the associated peripheral
 * to do what is required.
 * @n Reading from a registers gets status and data from the HW peripheral.
 *
 * The information on which bits have to be set to get a specific behavior
 * is documented in the <b>reference manual</b> of the microcontroller.
 *
 *
 * ----------------------------------------------------------------------------
 * @author Hanspeter Hochreutener, hhrt@zhaw.ch,
 * @n Linus Leuch, leuchlin@students.zhaw.ch,
 * @n Simon Meli, melisim1@students.zhaw.ch
 * @date 22.12.2021
 *****************************************************************************/


/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"

#include "measuring.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define ADC_FS			600	                               ///< Sampling freq. => 12 samples for a 50Hz period
#define ADC_CLOCK		84000000	                       ///< APB2 peripheral clock frequency
#define ADC_CLOCKS_PS	15			                       ///< Clocks/sample: 3 hold + 12 conversion
#define TIM_CLOCK		84000000	                       ///< APB1 timer clock frequency
#define TIM_TOP			9			                       ///< Timer top value
#define TIM_PRESCALE	(TIM_CLOCK/ADC_FS/(TIM_TOP+1)-1)   ///< Clock prescaler


/******************************************************************************
 * Variables
 *****************************************************************************/
bool MEAS_data1_ready = false;                      ///< New data from ADC 1 is ready
bool MEAS_data3_ready = false;                      ///< New data from ADC 3 is ready
bool ACC_data_rdy = false;                          ///< Accumulator data is ready

uint32_t MEAS_input_count = NUM_CHANNEL;            ///< Number of input channels?
bool DAC_active = false;				            ///< DAC output active?

uint32_t ADC_samples[NUM_CHANNEL*ADC_NUMS];         ///< Global Array
uint16_t batt_sample;                               ///< Global variable

static uint32_t ADC1_sample_count = 0;              ///< Index for ADC1 buffer
static uint32_t ADC3_sample_count = 0;              ///< Index for ADC3 buffer
static uint32_t ADC1_samples[2 * ADC_NUMS];         ///< ADC1 values of max. 2 input channels
static uint32_t ADC3_samples[3 * ADC_NUMS];         ///< ADC3 values of max. 3 input channels
static uint32_t DAC_sample = 0;                     ///< DAC output value

bool MEAS_DOPP_ready = false;
uint32_t MEAS_DOPP_input_count = 0;
uint32_t ADC_DOPP_samples[_DOPP_ADC_SAMPLES * 2]; // needs zero padding later


/******************************************************************************
 * Functions
 *****************************************************************************/


/** ***************************************************************************
 * @brief Configure GPIOs in analog mode.
 *
 * @note The input number for the ADCs is not equal to the GPIO pin number!
 * - ADC12_IN5 = GPIO PA5
 * - ADC12_IN7 = GPIO PA7
 * - ADC123_IN11 = GPIO PC1
 * - ADC123_IN13 = GPIO PC3
 * - ADC12_IN15 = GPIO PC5
 * - ADC3_IN4 = GPIO PF6
 * - DAC_OUT2 = GPIO PA5 (= same GPIO as ADC12_IN5)
 *****************************************************************************/
void MEAS_GPIO_analog_init(void)
{
     __HAL_RCC_GPIOA_CLK_ENABLE();              // Enable Clock for GPIO port A
    GPIOA->MODER |= (GPIO_MODER_MODER5_Msk);    // Analog mode for PA5 ADC12_IN5
    GPIOA->MODER |= (GPIO_MODER_MODER7_Msk);    // Analog mode for PA7 ADC12_IN7

    __HAL_RCC_GPIOC_CLK_ENABLE();               // Enable Clock for GPIO port C
    GPIOC->MODER |= (GPIO_MODER_MODER1_Msk);    // Analog mode for PC1 = ADC123_IN11
    GPIOC->MODER |= (GPIO_MODER_MODER3_Msk);    // Analog mode for PC3 = ADC123_IN13
    GPIOC->MODER |= (GPIO_MODER_MODER5_Msk);    // Analog mode for PC5 = ADC12_IN15

	__HAL_RCC_GPIOF_CLK_ENABLE();		        // Enable Clock for GPIO port F
	GPIOF->MODER |= (GPIO_MODER_MODER6_Msk);    // Analog mode for PF6 = ADC3_IN4
}


/** ***************************************************************************
 * @brief Configure the timer to trigger the ADC(s)
 *
 * @note For debugging the timer interrupt might be useful
 *****************************************************************************/
void MEAS_timer_init(void)
{
//    __HAL_RCC_TIM2_CLK_ENABLE();        // Enable Clock for TIM2
//    TIM2->PSC = TIM_PRESCALE;           // Prescaler for clock freq. = 1MHz
//    TIM2->ARR = TIM_TOP;                // Auto reload = counter top value
//    TIM2->CR2 |= TIM_CR2_MMS_1;         // TRGO on update
//    /* If timer interrupt is not needed, comment the following lines */
//    TIM2->DIER |= TIM_DIER_UIE;         // Enable update interrupt
//    NVIC_ClearPendingIRQ(TIM2_IRQn);    // Clear pending interrupt on line 0
//    NVIC_EnableIRQ(TIM2_IRQn);          // Enable interrupt line 0 in the NVIC


    __HAL_RCC_TIM2_CLK_ENABLE();        // Enable Clock for TIM2
    TIM2->PSC = _DOPP_TIM_PRESCALE;           // Prescaler for clock freq. = 1MHz
    TIM2->ARR = _DOPP_TIM_TOP;                // Auto reload = counter top value
    TIM2->CR2 |= TIM_CR2_MMS_1;         // TRGO on update
    /* If timer interrupt is not needed, comment the following lines */
    TIM2->DIER |= TIM_DIER_UIE;         // Enable update interrupt
    NVIC_ClearPendingIRQ(TIM2_IRQn);    // Clear pending interrupt on line 0
    NVIC_EnableIRQ(TIM2_IRQn);          // Enable interrupt line 0 in the NVIC
}


/** ***************************************************************************
 * @brief Resets the DAC
 *
 * when it is no longer used.
 *****************************************************************************/
void DAC_reset(void) {
	RCC->APB1RSTR |= RCC_APB1RSTR_DACRST;	// Reset the DAC
	RCC->APB1RSTR &= ~RCC_APB1RSTR_DACRST;	// Release reset of the DAC
}


/** ***************************************************************************
 * @brief Initialize the DAC
 *
 * The output used is DAC_OUT2 = GPIO PA5
 * @n As DAC_OUT2 = GPIO PA5 (= same GPIO as ADC12_IN5)
 * it is possible to monitor the output voltage DAC_OUT2 by the input ADC12_IN5.
 *****************************************************************************/
void DAC_init(void)
{
	__HAL_RCC_DAC_CLK_ENABLE();			// Enable Clock for DAC
	DAC->CR |= DAC_CR_EN2;				// Enable DAC output 2
}


/** ***************************************************************************
 * @brief Increment the DAC value and write it to the output
 *
 *****************************************************************************/
void DAC_increment(void) {
	DAC_sample += 20;				// Increment DAC output
	if (DAC_sample >= (1UL << ADC_DAC_RES)) { DAC_sample = 0; }	// Go to 0
	DAC->DHR12R2 = DAC_sample;		// Write new DAC output value
}


/** ***************************************************************************
 * @brief Resets the ADCs and the timer
 *
 * to make sure the different demos do not interfere.
 *****************************************************************************/
void ADC_reset(void) {
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;	// Reset ADCs
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;	// Release reset of ADCs
	TIM2->CR1 &= ~TIM_CR1_CEN;				// Disable timer
}


/** ***************************************************************************
 * @brief Initialize the ADC in single conversion mode
 *
 * The input is ADC2_IN15 = GPIO PC5 (Accumulator)
 *****************************************************************************/
void ADC2_IN15_single_init(void)
{
	MEAS_input_count = 1;				        // Only 1 input is converted
	__HAL_RCC_ADC2_CLK_ENABLE();		        // Enable Clock for ADC2
	ADC2->SQR3 |= (15UL << ADC_SQR3_SQ1_Pos);	// Input 15 = first conversion
}


/** ***************************************************************************
 * @brief Read one single value of the ADC in single conversion mode
 *
 * Start the conversion, wait in a while loop for end of conversion, read data.
 *****************************************************************************/
void ADC2_IN15_single_read(void)
{
	ADC2->CR2 |= ADC_CR2_ADON;			      // Enable ADC2
	HAL_Delay(1);						      // ADC needs some time to stabilize
	ADC2->CR2 |= ADC_CR2_SWSTART;
	
    while (!(ADC2->SR & ADC_SR_EOC)) { ; }    // Wait for end of conversion
	
    batt_sample = ADC2->DR;			      	  // Read the converted value
	ADC2->CR2 &= ~ADC_CR2_ADON;			      // Disable ADC2
	
	ADC_reset();
	ACC_data_rdy = true;
}

/** ***************************************************************************
 * @brief Initialize ADC, timer and DMA for sequential acquisition = scan mode
 *
 * Uses ADC1 and DMA2_Stream4 channel0
 * @n The ADC1 trigger is set to TIM2 TRGO event
 * @n At each trigger both inputs are converted sequentially
 * and transfered to memory by the DMA.
 * @n As each conversion triggers the DMA, the number of transfers is doubled.
 * @n The DMA triggers the transfer complete interrupt when all data is ready.
 * @n The inputs used are ADC1_IN5 = GPIO PA5 (Pad 2) and ADC1_IN7 = GPIO PA7 (Coil)
 *****************************************************************************/
void ADC1_IN5_IN13_scan_init(void)
{   
    __HAL_RCC_ADC1_CLK_ENABLE();                        // Enable Clock for ADC1
    ADC1->SQR1 |= ADC_SQR1_L_0;                         // Convert 2 inputs
    ADC1->SQR3 |= (5UL << ADC_SQR3_SQ1_Pos);            // Input 5 = first conversion
    ADC1->SQR3 |= (7UL << ADC_SQR3_SQ2_Pos);            // Input 7 = second conversion
    ADC1->CR1 |= ADC_CR1_SCAN;                          // Enable scan mode
    ADC1->CR2 |= (1UL << ADC_CR2_EXTEN_Pos);            // En. ext. trigger on rising e.
    ADC1->CR2 |= (6UL << ADC_CR2_EXTSEL_Pos);           // Timer 2 TRGO event
    ADC1->CR2 |= ADC_CR2_DMA;                           // Enable DMA mode
    
    __HAL_RCC_DMA2_CLK_ENABLE();                        // Enable Clock for DMA2
    DMA2_Stream4->CR &= ~DMA_SxCR_EN;                   // Disable the DMA stream 4
    while (DMA2_Stream4->CR & DMA_SxCR_EN) { ; }        // Wait for DMA to finish
    
    DMA2->HIFCR |= DMA_HIFCR_CTCIF4;                    // Clear transfer complete interrupt fl.
    DMA2_Stream4->CR |= (0UL << DMA_SxCR_CHSEL_Pos);    // Select channel 0
    DMA2_Stream4->CR |= DMA_SxCR_PL_1;                  // Priority high
    DMA2_Stream4->CR |= DMA_SxCR_MSIZE_1;               // Memory data size = 32 bit
    DMA2_Stream4->CR |= DMA_SxCR_PSIZE_1;               // Peripheral data size = 32 bit
    DMA2_Stream4->CR |= DMA_SxCR_MINC;                  // Increment memory address pointer
    DMA2_Stream4->CR |= DMA_SxCR_TCIE;                  // Transfer complete interrupt enable
    DMA2_Stream4->NDTR = 2*ADC_NUMS;                    // Number of data items to transfer
    DMA2_Stream4->PAR = (uint32_t)&ADC1->DR;            // Peripheral register address
    DMA2_Stream4->M0AR = (uint32_t)ADC1_samples;        // Buffer memory loc. address
}


/** ***************************************************************************
 * @brief Start DMA, ADC and timer
 *
 *****************************************************************************/
void ADC1_IN5_IN13_scan_start(void)
{
    DMA2_Stream4->CR |= DMA_SxCR_EN;            // Enable DMA
    NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn);    // Clear pending DMA interrupt
    NVIC_EnableIRQ(DMA2_Stream4_IRQn);          // Enable DMA interrupt in the NVIC
    ADC1->CR2 |= ADC_CR2_ADON;                  // Enable ADC1
    TIM2->CR1 |= TIM_CR1_CEN;                   // Enable timer
}


/** ***************************************************************************
 * @brief Initialize ADC, timer and DMA for sequential acquisition = scan mode
 *
 * Uses ADC3 and DMA2_Stream1 channel2
 * @n The ADC3 trigger is set to TIM2 TRGO event
 * @n At each trigger both inputs are converted sequentially
 * and transfered to memory by the DMA.
 * @n As each conversion triggers the DMA, the number of transfers is doubled.
 * @n The DMA triggers the transfer complete interrupt when all data is ready.
 * @n The inputs used are ADC3_IN4 = GPIO PF6 (PAD 3), ADC3_IN11 = GPIO PC1 (Pad 4)
 * and ADC3_IN13 = GPIO PC3 (Pad 1)
 *****************************************************************************/
void ADC3_IN4_IN11_IN13_scan_init(void)
{
    __HAL_RCC_ADC3_CLK_ENABLE();                       // Enable Clock for ADC3
	ADC3->SQR1 |= ADC_SQR1_L_1;                        // Convert 3 inputs
	ADC3->SQR3 |= (4UL << ADC_SQR3_SQ1_Pos);           // Input 4 = first conversion
	ADC3->SQR3 |= (11UL << ADC_SQR3_SQ2_Pos);          // Input 11 = second conversion
    ADC3->SQR3 |= (13UL << ADC_SQR3_SQ3_Pos);          // Input 13 = third conversion
	ADC3->CR1 |= ADC_CR1_SCAN;                         // Enable scan mode
	ADC3->CR2 |= (1UL << ADC_CR2_EXTEN_Pos);           // En. ext. trigger on rising e.
	ADC3->CR2 |= (6UL << ADC_CR2_EXTSEL_Pos);          // Timer 2 TRGO event
	ADC3->CR2 |= ADC_CR2_DMA;                          // Enable DMA mode
	
    __HAL_RCC_DMA2_CLK_ENABLE();                       // Enable Clock for DMA2
	DMA2_Stream1->CR &= ~DMA_SxCR_EN;                  // Disable the DMA stream 1
	while (DMA2_Stream1->CR & DMA_SxCR_EN) { ; }       // Wait for DMA to finish
	
    DMA2->LIFCR |= DMA_LIFCR_CTCIF1;	               // Clear transfer complete interrupt fl.
	DMA2_Stream1->CR |= (2UL << DMA_SxCR_CHSEL_Pos);   // Select channel 2
	DMA2_Stream1->CR |= DMA_SxCR_PL_1;                 // Priority high
	DMA2_Stream1->CR |= DMA_SxCR_MSIZE_1;              // Memory data size = 32 bit
	DMA2_Stream1->CR |= DMA_SxCR_PSIZE_1;              // Peripheral data size = 32 bit
	DMA2_Stream1->CR |= DMA_SxCR_MINC;                 // Increment memory address pointer
	DMA2_Stream1->CR |= DMA_SxCR_TCIE;                 // Transfer complete interrupt enable
	DMA2_Stream1->NDTR = 3*ADC_NUMS;                   // Number of data items to transfer
	DMA2_Stream1->PAR = (uint32_t)&ADC3->DR;           // Peripheral register address
	DMA2_Stream1->M0AR = (uint32_t)ADC3_samples;       // Buffer memory loc. address
}


/** ***************************************************************************
 * @brief Start DMA, ADC and timer
 *
 *****************************************************************************/
void ADC3_IN4_IN11_IN13_scan_start(void)
{
	DMA2_Stream1->CR |= DMA_SxCR_EN;           // Enable DMA
	NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);   // Clear pending DMA interrupt
	NVIC_EnableIRQ(DMA2_Stream1_IRQn);         // Enable DMA interrupt in the NVIC
	ADC3->CR2 |= ADC_CR2_ADON;                 // Enable ADC3
	TIM2->CR1 |= TIM_CR1_CEN;                  // Enable timer
}


/** ***************************************************************************
 * @brief Interrupt handler for the timer 2
 *
 * @note This interrupt handler was only used for debugging purposes
 * and to increment the DAC value.
 *****************************************************************************/
void TIM2_IRQHandler(void)
{
	TIM2->SR &= ~TIM_SR_UIF;			// Clear pending interrupt flag
	if (DAC_active) {
		DAC_increment();
	}
}


/** ***************************************************************************
 * @brief Interrupt handler for the ADCs
 *
 * Reads one sample from the ADC1 and ADC3 DataRegister and transfers it to a buffer.
 * @n Stops when 2 * ADC_NUMS or 3 * ADC_NUMS samples have been read.
 *****************************************************************************/
void ADC_IRQHandler(void)
{
	if (ADC1->SR & ADC_SR_EOC) {                      // Check if ADC1 end of conversion
		ADC_samples[ADC1_sample_count++] = ADC1->DR;  // Read input channel 1 only
		
        if (ADC1_sample_count >= 2 * ADC_NUMS) {      // Buffer full
			TIM2->CR1 &= ~TIM_CR1_CEN;	              // Disable timer
			ADC1->CR2 &= ~ADC_CR2_ADON;               // Disable ADC1
			ADC_reset();
			MEAS_data1_ready = true;
		}
	}

    if (ADC3->SR & ADC_SR_EOC) {                      // Check if ADC3 end of conversion
        ADC_samples[ADC3_sample_count++] = ADC3->DR;  // Read input channel 1 only
        
        if (ADC3_sample_count >= 3 * ADC_NUMS) {      // Buffer full
            TIM2->CR1 &= ~TIM_CR1_CEN;                // Disable timer
            ADC3->CR2 &= ~ADC_CR2_ADON;               // Disable ADC3
            ADC_reset();
            MEAS_data3_ready = true;
        }
    }
}


/** ***************************************************************************
 * @brief Interrupt handler for DMA2 Stream1
 *
 * The samples from the ADC3 have been transfered to memory by the DMA2 Stream1
 * and are ready for processing.
 *****************************************************************************/
void DMA2_Stream1_IRQHandler(void)
{
	if (DMA2->LISR & DMA_LISR_TCIF1) {                // Stream1 transfer compl. interrupt f.
		NVIC_DisableIRQ(DMA2_Stream1_IRQn);           // Disable DMA interrupt in the NVIC
		NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);      // Clear pending DMA interrupt
		DMA2_Stream1->CR &= ~DMA_SxCR_EN;             // Disable the DMA
		while (DMA2_Stream1->CR & DMA_SxCR_EN) { ; }  // Wait for DMA to finish
		DMA2->LIFCR |= DMA_LIFCR_CTCIF1;              // Clear transfer complete interrupt fl.
		//TIM2->CR1 &= ~TIM_CR1_CEN;                  // Disable timer
		ADC3->CR2 &= ~ADC_CR2_ADON;                   // Disable ADC3
		ADC3->CR2 &= ~ADC_CR2_DMA;                    // Disable DMA mode
		//ADC_reset();
		MEAS_data3_ready = true;
	}
}


///** ***************************************************************************
// * @brief Interrupt handler for DMA2 Stream4
// *
// * The samples from the ADC1 have been transfered to memory by the DMA2 Stream4
// * and are ready for processing.
// *****************************************************************************/
//void DMA2_Stream4_IRQHandler(void)
//{
//	if (DMA2->HISR & DMA_HISR_TCIF4) {                // Stream4 transfer compl. interrupt f.
//		NVIC_DisableIRQ(DMA2_Stream4_IRQn);           // Disable DMA interrupt in the NVIC
//		NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn);      // Clear pending DMA interrupt
//		DMA2_Stream4->CR &= ~DMA_SxCR_EN;             // Disable the DMA
//		while (DMA2_Stream4->CR & DMA_SxCR_EN) { ; }  // Wait for DMA to finish
//		DMA2->HIFCR |= DMA_HIFCR_CTCIF4;              // Clear transfer complete interrupt fl.
//		//TIM2->CR1 &= ~TIM_CR1_CEN;                  // Disable timer
//		ADC1->CR2 &= ~ADC_CR2_ADON;                   // Disable ADC1
//		ADC1->CR2 &= ~ADC_CR2_DMA;                    // Disable DMA mode
//		//ADC_reset();
//		MEAS_data1_ready = true;
//	}
//}


/** ***************************************************************************
 * @brief Resets sample arrays
 *
 * Array elements are all set to zero.
 *****************************************************************************/
void reset_data(void)
{
    for (uint32_t i = 0; i < ADC_NUMS; i++) {
        ADC_samples[NUM_CHANNEL * i] = 0;
        ADC_samples[NUM_CHANNEL * i + 1] = 0;
        ADC_samples[NUM_CHANNEL * i + 2] = 0;
        ADC_samples[NUM_CHANNEL * i + 3] = 0;
        ADC_samples[NUM_CHANNEL * i + 4] = 0;

        ADC1_samples[2 * i] = 0;
        ADC1_samples[2 * i + 1] = 0;

        ADC3_samples[3 * i] = 0;
        ADC3_samples[3 * i + 1] = 0;
        ADC3_samples[3 * i + 2] = 0;
    }

    ADC1_sample_count = 0;
    ADC3_sample_count = 0;
}

/** ***************************************************************************
 * @brief Copies into global array
 *
 * The acquired samples are copied into a global array, for being accessed by additional functions
 *****************************************************************************/
void copy_data(void)
{
    for (uint32_t i = 0; i < ADC_NUMS; i++) {
        ADC_samples[i * NUM_CHANNEL] = ADC3_samples[i * 3 + 2];
        ADC_samples[i * NUM_CHANNEL + 1] = ADC1_samples[i * 2];
        ADC_samples[i * NUM_CHANNEL + 2] = ADC3_samples[i * 3];
        ADC_samples[i * NUM_CHANNEL + 3] = ADC3_samples[i * 3 + 1];
        ADC_samples[i * NUM_CHANNEL + 4] = ADC1_samples[i * 2 + 1];
    }
}

/** ***************************************************************************
 * @brief reads value from GPIO PG2 (status pin charging IC)
 *
 * @note The pin is LOW when battery is charging
 *****************************************************************************/
bool batteryStatus(void)
{
    __HAL_RCC_GPIOG_CLK_ENABLE();                   // Enable Clock for GPIO port G
    GPIOG->MODER &= ~GPIO_MODER_MODER2;             // Reset mode for PG2
    GPIOG->MODER |= (0u << GPIO_MODER_MODER2_Pos);  // Set PG2 as input
    uint32_t value = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_2);

    if(value){
    	return false;
    } else {
    	return true;
    }
}


void ADC_DOPP_scan_init(void) {
	// scan inputs from PC1/PC3 into memory
	// PC1 -> ADC1 -> DMA ch0str4
	// PC3 -> ADC2 -> DMA ch1str3
	MEAS_input_count = 2; // 2 inputs are converted
	__HAL_RCC_ADC1_CLK_ENABLE();		// Enable Clock for ADC1
	__HAL_RCC_ADC2_CLK_ENABLE();		// Enable Clock for ADC2
	ADC->CCR |= ADC_CCR_DMA_1;			// Enable DMA mode 2 = dual DMA
	ADC->CCR |= ADC_CCR_MULTI_1 | ADC_CCR_MULTI_2; // ADC1 and ADC2 simultan.
	ADC1->CR2 |= (1UL << ADC_CR2_EXTEN_Pos);	// En. ext. trigger on rising e.
	ADC1->CR2 |= (6UL << ADC_CR2_EXTSEL_Pos);	// Timer 2 TRGO event
	ADC1->SQR3 |= (11UL << ADC_SQR3_SQ1_Pos);	// Input 11 = first conversion
	ADC2->SQR3 |= (13UL << ADC_SQR3_SQ1_Pos);	// Input 13 = first conversion
	__HAL_RCC_DMA2_CLK_ENABLE();		// Enable Clock for DMA2

	DMA2_Stream4->CR &= ~DMA_SxCR_EN;	// Disable the DMA stream 4
	while (DMA2_Stream4->CR & DMA_SxCR_EN) { ; }	// Wait for DMA to finish
	DMA2->HIFCR |= DMA_HIFCR_CTCIF4;	// Clear transfer complete interrupt fl.

	DMA2_Stream4->CR |= (0UL << DMA_SxCR_CHSEL_Pos);	// Select channel 0
	DMA2_Stream4->CR |= DMA_SxCR_PL_1;		// Priority high
	DMA2_Stream4->CR |= DMA_SxCR_MSIZE_1;	// Memory data size = 32 bit
	DMA2_Stream4->CR |= DMA_SxCR_PSIZE_1;	// Peripheral data size = 32 bit
	DMA2_Stream4->CR |= DMA_SxCR_MINC;	// Increment memory address pointer
	DMA2_Stream4->CR |= DMA_SxCR_TCIE;	// Transfer complete interrupt enable
	DMA2_Stream4->NDTR = _DOPP_ADC_SAMPLES;		// Number of data items to transfer
	DMA2_Stream4->PAR = (uint32_t)&ADC->CDR;	// Peripheral register address
	DMA2_Stream4->M0AR = (uint32_t)ADC_DOPP_samples;	// Buffer memory loc. address
}

void ADC_DOPP_scan_start(void) {
	DMA2_Stream4->CR |= DMA_SxCR_EN;	// Enable DMA
	NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn);	// Clear pending DMA interrupt
	NVIC_EnableIRQ(DMA2_Stream4_IRQn);	// Enable DMA interrupt in the NVIC
	ADC1->CR2 |= ADC_CR2_ADON;			// Enable ADC1
	ADC2->CR2 |= ADC_CR2_ADON;			// Enable ADC2
	TIM2->CR1 |= TIM_CR1_CEN;			// Enable timer
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
	if (DMA2->HISR & DMA_HISR_TCIF4) {	// Stream4 transfer compl. interrupt f.
		NVIC_DisableIRQ(DMA2_Stream4_IRQn);	// Disable DMA interrupt in the NVIC
		NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn);// Clear pending DMA interrupt
		DMA2_Stream4->CR &= ~DMA_SxCR_EN;	// Disable the DMA
		while (DMA2_Stream4->CR & DMA_SxCR_EN) { ; }	// Wait for DMA to finish
		DMA2->HIFCR |= DMA_HIFCR_CTCIF4;// Clear transfer complete interrupt fl.
		TIM2->CR1 &= ~TIM_CR1_CEN;		// Disable timer
		ADC1->CR2 &= ~ADC_CR2_ADON;		// Disable ADC1
		ADC2->CR2 &= ~ADC_CR2_ADON;		// Disable ADC2
		ADC->CCR &= ~ADC_CCR_DMA_1;		// Disable DMA mode
		/* Extract combined samples */
		for (int32_t i = _DOPP_ADC_SAMPLES-1; i >= 0; i--){
			ADC_DOPP_samples[2*i+1] = (ADC_DOPP_samples[i] >> 16);
			ADC_DOPP_samples[2*i]   = (ADC_DOPP_samples[i] & 0xffff);
		}
		ADC_reset();
		MEAS_DOPP_ready = true;
	}
}


void DOPP_copy_data(void) {
	// no need, really
}


void DOPP_reset_data(void) {
	for (uint32_t i=0; i<_DOPP_ADC_SAMPLES; i++) {
		ADC_DOPP_samples[i] = 0;
	}
}

