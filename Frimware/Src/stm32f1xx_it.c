/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file    stm32f1xx_it.c
* @brief   Interrupt Service Routines.
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float oct2_1  =       0;
float oct1_1  =       0;
typedef enum {FALSE, TRUE}flag;
flag state = FALSE;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void pwmOC(float div);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */
  
  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */
  
  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */
  
  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */
  
  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
  
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */
  
  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  
  /* USER CODE END SysTick_IRQn 0 */
  
  /* USER CODE BEGIN SysTick_IRQn 1 */
  
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
  TIM4->SR &= ~(TIM_SR_CC4IF);
  
  // state = state == TRUE ? FALSE : TRUE;
  if(state) state = FALSE;
  else direct(STOP);
  
 // LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_8); 
  
  
  
  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */
  
  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  state = TRUE;
  if (USART1->SR & USART_SR_RXNE){
    switch(USART1->DR){
    case 'F':
      TIM2->CCR1=(uint8_t)oct2_1;
      TIM1->CCR1=(uint8_t)oct1_1;
      direct(FORWARD);
      break;
    case 'B': 
      TIM2->CCR1=(uint8_t)oct2_1;
      TIM1->CCR1=(uint8_t)oct1_1;
      direct(BACK);
      break;
    case 'L': 
      TIM2->CCR1=(uint8_t)oct2_1;
      TIM1->CCR1=(uint8_t)oct1_1;
      direct(LEFT);
      break;
    case 'R': 
      TIM2->CCR1=(uint8_t)oct2_1;
      TIM1->CCR1=(uint8_t)oct1_1;
      direct(RIGHT);
      break;
    case 'S': direct(STOP);
    break;
    case 'G': direct(FORWARD);
    TIM2->CCR1=(uint8_t) (0.5*oct2_1);
    break;
    case 'I': direct(FORWARD);
    TIM1->CCR1=(uint8_t)(0.5*oct1_1);
    break;
    case 'H': direct(BACK);
    TIM2->CCR1=(uint8_t) (oct2_1*0.5);
    break;
    case 'J': direct(BACK);
    TIM1->CCR1=(uint8_t) (oct1_1*0.5);
    break;  
    case 'W': 
      GPIOB->BSRR |= GPIO_BSRR_BS4;
      GPIOB->BSRR |= GPIO_BSRR_BS6;
      break;
    case 'w': 
      GPIOB->BSRR |= GPIO_BSRR_BR4;
      GPIOB->BSRR |= GPIO_BSRR_BR6;    
      break;
      
    case '1': 
      pwmOC(0.1);
      break;
      
    case '2': 
      pwmOC(0.2);
      break;
      
    case '3': 
      pwmOC(0.3);
      
      break;
      
    case '4': 
      pwmOC(0.4);
      
      break;
      
    case '5': 
      pwmOC(0.5);
      break;
      
    case '6': 
      pwmOC(0.6);
      break;
      
    case '7': 
      pwmOC(0.7);
      break;
      
    case '8': 
      pwmOC(0.8);
      break;
      
    case '9': 
      pwmOC(0.9);
      break;
      
    case 'q': 
      pwmOC(1);
      break;
    }
  }
  
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */
  
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void pwmOC(float div){
  oct1_1 = div*ARR;
  oct2_1 = div*ARR;
  TIM1->CCR1=(uint8_t)oct1_1;
  TIM2->CCR1=(uint8_t)oct2_1;
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
