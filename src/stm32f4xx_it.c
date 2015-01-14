/**
 ******************************************************************************
 * @file    FreeRTOS/FreeRTOS_ThreadCreation/Src/stm32f4xx_it.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    26-June-2014
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_gpio.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "clib.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern void xPortSysTickHandler(void);

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (DRDY),for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

extern bool accel_avail, gyro_avail, compass_avail;
extern xTaskHandle recvTaskHandle;
/**
 * @brief  Sensor DRDY interrupt, PG10, PG11, PG12
 * @param  None
 * @retval None
 */
void EXTI15_10_IRQHandler(void)
{
    kputs("EXTI15_10_IRQHandler\r\n");
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET)  {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
        accel_avail = true;
        xTaskResumeFromISR(recvTaskHandle);
    }
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET)  {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
        gyro_avail = true;
        xTaskResumeFromISR(recvTaskHandle);
    }
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET)  {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
        compass_avail = true;
        xTaskResumeFromISR(recvTaskHandle);
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
