/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "lcd.h"
#include "delay.h"
#include "stdio.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int mode=0,i1=0,i2=0,i3=0,wrong=0;
char text[30],secret[]={'@','0','1','2','3','4','5','6','7','8','9'};
typedef struct{
  char b1,b2,b3;
}secret_put;
secret_put out,ans;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
bool check(void){
  if((USART1_RECEIVE_ARRAY[0]>='0'&&USART1_RECEIVE_ARRAY[0]<='9')&&
     (USART1_RECEIVE_ARRAY[1]>='0'&&USART1_RECEIVE_ARRAY[1]<='9')&&
     (USART1_RECEIVE_ARRAY[2]>='0'&&USART1_RECEIVE_ARRAY[2]<='9')&&
     (USART1_RECEIVE_ARRAY[3]=='-')&&
     (USART1_RECEIVE_ARRAY[4]>='0'&&USART1_RECEIVE_ARRAY[4]<='9')&&
     (USART1_RECEIVE_ARRAY[5]>='0'&&USART1_RECEIVE_ARRAY[5]<='9')&&
     (USART1_RECEIVE_ARRAY[6]>='0'&&USART1_RECEIVE_ARRAY[6]<='9'))
      return true;
  return false;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if ((USART1_RECEIVE_STATE & 0x8000) == 0)
        {
            if (USART1_RECEIVE_STATE & 0x4000)
                if (USART1_RECEIVE_DATA[0] != 0x0a)
                    USART1_RECEIVE_STATE = 0;
                else
                    USART1_RECEIVE_STATE |= 0x8000;
            else
            {
                if (USART1_RECEIVE_DATA[0] == 0x0d)
                    USART1_RECEIVE_STATE |= 0x4000;
                else
                {
                    USART1_RECEIVE_ARRAY[USART1_RECEIVE_STATE & 0X3FFF] = USART1_RECEIVE_DATA[0];
                    USART1_RECEIVE_STATE++;
                    if (USART1_RECEIVE_STATE > (USART1_RECEIVE_LENGTH - 1))
                        USART1_RECEIVE_STATE = 0;
                }
            }
        }
    }
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  out.b1=secret[i1];
  out.b2=secret[i2];
  out.b3=secret[i3];
  
  ans.b1=secret[2];
  ans.b2=secret[3];
  ans.b3=secret[4];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  delay_init(80);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  led_init();
  LCD_Init();
  LCD_Clear(Black);
  LCD_SetBackColor(Black);
  LCD_SetTextColor(White);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_UART_Receive_IT(&huart1,&USART1_RECEIVE_DATA,USART1_RECEIVE_BUUFER_SIZE);
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_2);
  while (1)
  {
    if(USART1_RECEIVE_STATE&0x8000){//接收上位机发来的字符串
      if(check()){
        ans.b1=USART1_RECEIVE_ARRAY[4];
        ans.b2=USART1_RECEIVE_ARRAY[5];
        ans.b3=USART1_RECEIVE_ARRAY[6];
        printf("%d%d%d\r\n",ans.b1,ans.b2,ans.b3);
      }else{
        printf("\r\nError\r\n");
      }
      USART1_RECEIVE_STATE=0;
    }
    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0){
      while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0);
      if(mode==0){
        i1++;
        if(i1==11)i1=0;
        out.b1=secret[i1];
      }
    }else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0){
      while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0);
      if(mode==0){
        i2++;
        if(i2==11)i2=0;
        out.b2=secret[i2];
      }
    }else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0){
      while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0);
      if(mode==0){
        i3++;
        if(i3==11)i3=0;
        out.b3=secret[i3];
      }
    }else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0){
      while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0);
      if(out.b1==ans.b1&&out.b2==ans.b2&&out.b3==ans.b3){
        mode=1;
        wrong=0;
        led_ON(1);
        __HAL_TIM_SET_AUTORELOAD(&htim2,50-1);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,5);
      }else{
        wrong++;
        i1=0;
        i2=0;
        i3=0;
        out.b1=secret[i1];
        out.b2=secret[i2];
        out.b3=secret[i3];
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int counter=0,wrong_counter=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance==TIM6){
    if(mode==0){
      LCD_DisplayStringLine(Line2,(u8*)"       PSD    ");
      sprintf(text,"    B1:%c     ",out.b1);
      LCD_DisplayStringLine(Line4,(u8*)text);
      sprintf(text,"    B2:%c   ",out.b2);
      LCD_DisplayStringLine(Line5,(u8*)text);
      sprintf(text,"    B3:%c   ",out.b3);
      LCD_DisplayStringLine(Line6,(u8*)text);
    }else{
      LCD_DisplayStringLine(Line2,(u8*)"       STA    ");
      LCD_DisplayStringLine(Line4,(u8*)"    F:2000Hz   ");
      LCD_DisplayStringLine(Line5,(u8*)"    D:10%    ");
      LCD_DisplayStringLine(Line6,(u8*)"              ");
      counter++;
      if(counter==50){
        counter=0;
        led_OFF(1);
        __HAL_TIM_SET_AUTORELOAD(&htim2,100-1);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,5);
        mode=0;
        i1=0;
        i2=0;
        i3=0;
        out.b1=secret[i1];
        out.b2=secret[i2];
        out.b3=secret[i3];
      }
    }
    if(wrong>=3){
      led_Toggle(2);
      wrong_counter++;
      if(wrong_counter==50){
        wrong_counter=0;
        wrong=0;
      }
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
