/**
  *
  *
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
void _Error_Handler(char * file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

//Initialize all four LEDs
void init_leds(void)
{
  //Enable GPIOC on RCC
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  //Initialize red LED, PC6
  GPIOC->MODER |= GPIO_MODER_MODER6_0; //General purpose output
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_6; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR6_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR6_1); //No pull up or down

  //Initialize blue LED, PC7
  GPIOC->MODER |= GPIO_MODER_MODER7_0; //General purpose output
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_7; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR7_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7_0 | GPIO_PUPDR_PUPDR7_1); //No pull up or down

  //Initialize orange LED, PC8
  GPIOC->MODER |= GPIO_MODER_MODER8_0; //General purpose output
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_8; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR8_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1); //No pull up or down

  //Initialize green LED, PC9
  GPIOC->MODER |= GPIO_MODER_MODER9_0; //General purpose output
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_9; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR9_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1); //No pull up or down

  //Set all LEDs off
  GPIOC->BSRR |= GPIO_BSRR_BR_6;
  GPIOC->BSRR |= GPIO_BSRR_BR_7;
  GPIOC->BSRR |= GPIO_BSRR_BR_8;
  GPIOC->BSRR |= GPIO_BSRR_BR_9;
}

int main(void)
{
  HAL_Init(); // Reset of all peripherals, init the Flash and Systick
  SystemClock_Config(); //Configure the system clock

  init_leds();

  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  //Select PA1 as ADC channel 1 input
  //Configure pin to analog, no pull up/down
  GPIOA->MODER |= (GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1);
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1_0 | GPIO_PUPDR_PUPDR1_1);
  //Connect output (center pin) of potentiometer to input pin, other two to 3v and gnd
  //Enable ADC1 in RCC
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
  //Configure ADC to 8-bit resolution, continuous conversion, hardware triggers disabled
  ADC1->CFGR1 |= ADC_CFGR1_RES_1;
  ADC1->CFGR1 |= ADC_CFGR1_CONT;
  //Select/enable input pin's channel for ADC conversion
  ADC1->CHSELR |= ADC_CHSELR_CHSEL1;
  //Perform self calibration, enable, and start ADC
  //Set ADCAL to 1, ADEN must be 0
  ADC1->CR &= ~ADC_CR_ADEN;
  ADC1->CR |= ADC_CR_ADCAL;
  //Wait until ADCAL is reset
  while (ADC1->CR & ADC_CR_ADCAL) {}
  //Clear ADRDY
  ADC1->ISR &= ~ADC_ISR_ADRDY;
  //Set ADEN to 1 to enable
  ADC1->CR |= ADC_CR_ADEN;
  //Wait until ADRDY is set
  while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}
  //Set ADSTART to start
  ADC1->CR |= ADC_CR_ADSTART;

  //Select PA4 DAC_OUT_1
  //Configure pin to analog, no pull up/down
  GPIOA->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER4_1);
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR4_1);
  //Connect oscilloscope/logic analyzer to pin
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  //Set DAC to software trigger mode
  DAC1->CR &= ~DAC_CR_TEN1;
  DAC1->CR |= (DAC_CR_TSEL1_0 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_2);
  //Enable DAC channel
  DAC1->CR |= DAC_CR_EN1;
  //Copy one wave table, not square wave
  //In main loop, index array to write value into appropriate DAC data register
  //Use a 1ms delay between updating the DAC to new values
  //Submit capture to postlab

  // Sine Wave: 8-bit, 32 samples/cycle
  const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
  232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};

  uint8_t index = 0;

  uint16_t pot_val;

  while (1)
  {
    //Read ADC data reg and assign each LED to a threshold
    pot_val = ADC1->DR;

    if (pot_val > 50)
      GPIOC->ODR |= GPIO_ODR_6;
    else
      GPIOC->ODR &= ~GPIO_ODR_6;

    if (pot_val > 100)
      GPIOC->ODR |= GPIO_ODR_9;
    else
      GPIOC->ODR &= ~GPIO_ODR_9;

    if (pot_val > 150)
      GPIOC->ODR |= GPIO_ODR_7;
    else
      GPIOC->ODR &= ~GPIO_ODR_7;

    if (pot_val > 200)
      GPIOC->ODR |= GPIO_ODR_8;
    else
      GPIOC->ODR &= ~GPIO_ODR_8;

    //Output sine wave on DAC
    HAL_Delay(1);
    if (index == 31)
      index = 0;
    else
      index++;

    DAC1->DHR8R1 = sine_table[index];

  }

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
