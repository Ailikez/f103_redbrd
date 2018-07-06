/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "Adafruit_GFX.h"
#include "stdbool.h"
#include "math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
bool is_k1_pressed = false; 
bool is_k2_pressed = false; 
bool is_k3_pressed = false; 
bool is_dot_grid_on = false;
bool is_axis_on = false;
bool is_cos_on = false;
bool is_pwm_on = false; 
extern __IO uint32_t uwTick;
__IO uint32_t last_uwTick;

unsigned int counter = 0;
float duty = 0.0f;
int x1=0, y1=31, r1=0;
signed char x2[128], y2[128];
extern int16_t _width, _height, cursor_x, cursor_y;
extern uint16_t textcolor, textbgcolor;
extern uint8_t textsize, rotation;
extern bool wrap;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
//  MX_SPI1_Init();
//  MX_USART3_UART_Init();
//  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Adafruit_GFX();
//  Adafruit_GFX_setCursor(0, 0);
//  Adafruit_GFX_write_string("FUCK");
//  Adafruit_GFX_setCursor(80, 0);
//  Adafruit_GFX_write_string(" YOU");

//      for(int i = 0; i < 127; i++)
//  {
//    x2[i] = i - 63;
//    y2[i] = (signed char)(10*tan((i - 63.0f)/10.0f)) + 31;
//    ssd1306_DrawPixel(i, y2[i], 1);
//  }
//  ssd1306_WriteString("D: ", Font_11x18, White);
//  ssd1306_WriteFloat(0.0f, 0, Font_11x18, White);
//  ssd1306_WriteChar('%', Font_11x18, White);
//  ssd1306_SetCursor(2, 40);
//  ssd1306_WriteString("PWM off", Font_11x18, White);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
//  MX_FREERTOS_Init();

  /* Start scheduler */
//  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  uint32_t temp = uwTick;

  if(is_k1_pressed)
  {
    is_k1_pressed = false;
    is_cos_on = !is_cos_on;
    if(!is_cos_on)
    {
      for(int i = 0; i < 127; i++)
      {
        x2[i] = i - 63;
        y2[i] = (signed char)(30.0f * cos((i*3 - 63.0f)/10.0f + temp / 500.0f)) + 31;
        ssd1306_DrawPixel(i, y2[i], 0);
      }
    }
//    if(x1<127)
//    {Adafruit_GFX_drawCircle(x1, y1, r1, Black);x1++;Adafruit_GFX_drawCircle(x1, y1, r1, White);}
  }
  else if(is_k2_pressed)
  {
    is_k2_pressed = false;
    is_axis_on = !is_axis_on;
    if(!is_axis_on)
    {
      Adafruit_GFX_drawFastHLine(0, 31, 128, Black);
      Adafruit_GFX_drawFastVLine(63, 0, 64, Black);
      Adafruit_GFX_fillTriangle(127, 31, 123, 28, 123, 34, Black);
      Adafruit_GFX_fillTriangle(63, 0, 60, 4, 66, 4, Black);
    }
//    if(x1>0)               ;
//    {Adafruit_GFX_drawCircle(x1, y1, r1, Black);x1--;Adafruit_GFX_drawCircle(x1, y1, r1, White);}
  }
  if(is_k3_pressed)
  {
    is_k3_pressed = false;
    is_dot_grid_on = !is_dot_grid_on;
    if(!is_dot_grid_on)
    {
      for(int i = 0; i < 127; i++)
      {
        for(int j = 0; j < 63; j++)
        {
          if(!(i%8) && !(j%8))
          {
            ssd1306_DrawPixel(i, j, 0);
          }
        }
      }
    }
//    {Adafruit_GFX_drawCircle(x1, y1, r1, Black);r1++;Adafruit_GFX_drawCircle(x1, y1, r1, White);}
  }
  if(is_axis_on)
  {
    Adafruit_GFX_drawFastHLine(0, 31, 128, White);
    Adafruit_GFX_drawFastVLine(63, 0, 64, White);
    Adafruit_GFX_fillTriangle(127, 31, 123, 28, 123, 34, White);
    Adafruit_GFX_fillTriangle(63, 0, 60, 4, 66, 4, White);
  }

  if(is_dot_grid_on)
  {
    for(int i = 0; i < 127; i++)
    {
      for(int j = 0; j < 63; j++)
      {
        if(!(i%8) && !(j%8))
        {
          ssd1306_DrawPixel(i, j, 1);
        }
      }
    }
  }
  

  for(int i = 0; i < 127; i++)
  {
    x2[i] = i - 63;
    y2[i] = (signed char)(30.0f * sin((2*i - 63.0f)/10.0f + temp / 1000.0f) * cos((i - 63.0f)/10.0f + temp / 500.0f)) + 31;
    ssd1306_DrawPixel(i, y2[i], 1);
    last_uwTick = temp;
  }
  if(is_cos_on)
  {
    for(int i = 0; i < 127; i++)
    {
      x2[i] = i - 63;
      y2[i] = (signed char)(30.0f * cos((i*3 - 63.0f)/10.0f + temp / 500.0f)) + 31;
      ssd1306_DrawPixel(i, y2[i], 1);
    }
  }

  ssd1306_UpdateScreen();    
  for(int i = 0; i < 127; i++)
  {
    x2[i] = i - 63;
    y2[i] = (signed char)(30.0f * sin((2*i - 63.0f)/10.0f + last_uwTick / 1000.0f) * cos((i - 63.0f)/10.0f + last_uwTick / 500.0f)) + 31;
    ssd1306_DrawPixel(i, y2[i], 0);
  }
  if(is_cos_on)
  {
    for(int i = 0; i < 127; i++)
    {
      x2[i] = i - 63;
      y2[i] = (signed char)(30.0f * cos((3*i - 63.0f)/10.0f + last_uwTick / 500.0f)) + 31;
      ssd1306_DrawPixel(i, y2[i], 0);
    }
  }
#if 0
    if(is_k1_pressed)
    {
      is_k1_pressed = false;
      if(counter < 25)
      {
        counter += 1;
      }
      duty = (float)counter / 9999.0f;
      ssd1306_SetCursor(0, 15);
      ssd1306_WriteString("D: ", Font_11x18, White);
      ssd1306_WriteFloat(duty, 0, Font_11x18, White);
      ssd1306_WriteChar('%', Font_11x18, White);
    }
    else if(is_k2_pressed)
    {
      is_k2_pressed = false;
      if(counter > 0)
      {
        counter -= 1;
      }
      duty = (float)counter / 9999.0f;
      ssd1306_SetCursor(0, 15);
      ssd1306_WriteString("D: ", Font_11x18, White);
//      ssd1306_SetCursor(2, 35);
      ssd1306_WriteFloat(duty, 0, Font_11x18, White);
      ssd1306_WriteChar('%', Font_11x18, White);
    }

    if(is_k3_pressed)
    {
      is_k3_pressed = false;
      is_pwm_on = !is_pwm_on;
      if(is_pwm_on)
      {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        ssd1306_SetCursor(2, 40);
        ssd1306_WriteString("PWM on ", Font_11x18, White);
      }
      else
      {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        ssd1306_SetCursor(2, 40);
        ssd1306_WriteString("PWM off", Font_11x18, White);
      }
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, counter);
    BSP_LED4_GPIO_Port->ODR ^= BSP_LED4_Pin | BSP_LED3_Pin;
    ssd1306_UpdateScreen();
#endif
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
  {
    case BSP_K1_Pin:
    {
      is_k1_pressed = true;
      break;
    }
    case BSP_K2_Pin:
    {
      is_k2_pressed = true;
      break;
    }    
    case BSP_K3_Pin:
    {
      is_k3_pressed = true;
      break;
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
//  if (htim->Instance == TIM4) {
//    ssd1306_UpdateScreen();
//  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
