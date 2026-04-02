/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Gimbal_Yaw_Small.h"
#include "Gimbal_Yaw_Big.h"
#include "Gimbal_Trigger.h"
#include "Gimbal_Shoot.h"
#include "Gimbal_Pitch.h"
#include "Gimbal_CtoC.h"
#include "iwdg.h"
#include "Remote.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

#define NEW_FRAME_HEADER1    0xBB    // 自定义帧头1，可根据实际需求修改
#define NEW_FRAME_HEADER2    0x77    // 自定义帧头2，可根据实际需求修改
#define NEW_FRAME_TAIL1			 0xCC
#define NEW_FRAME_TAIL2      0xEE    // 自定义帧尾，可根据实际需求修改
#define NEW_FRAME_LENGTH     15      // 帧总长度15字节
#define NEW_DATA_BODY_LENGTH 11      // 帧头后到CRC8前的长度（15-3=12）

uint8_t NewRxBuffer[NEW_FRAME_LENGTH] = {0};

NewRxDataStruct NewRxData = {0};     // 解析后的数据

uint8_t MiniPC_Flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim11;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

extern PID_PositionInitTypedef BigYaw_SpeedPID;
extern PID_PositionInitTypedef SmallYaw_SpeedPID;
extern RC_ctrl_t *local_rc_ctrl;		
extern uint8_t Remote_Status;     //遥控器连接状态,默认未连接(0)


/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Pre-fetch fault, memory access fault.
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
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(IST8310_DRDY_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(INT1_ACCEL_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(INT1_GRYO_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */
	if (__HAL_TIM_GET_FLAG(&htim11, TIM_FLAG_UPDATE) != RESET) {//如果进入这个timer11中断，说明没连接小电脑
		MiniPC_Flag = 0;
	}
  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  HAL_TIM_IRQHandler(&htim11);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	//中断接收遥控器数据
	__HAL_TIM_SET_COUNTER(&htim7, 0);//timer7刷新
	HAL_TIM_Base_Stop(&htim7); 
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)){
      Remote_UART_IDLE_Callback();//处理接收完成的遥控器数据
		if(Remote_Status == 0){//首次连接
//			if(	local_rc_ctrl->rc.ch[0]!=1024 || local_rc_ctrl->rc.ch[1]!=1024 || 
//					local_rc_ctrl->rc.ch[2]!=1024 || local_rc_ctrl->rc.ch[3]!=1024 ){
				Remote_Status = 1;
//			}
		}
		else if(Remote_Status == 1){
			
		}
  }
//	HAL_IWDG_Refresh(&hiwdg);
	HAL_TIM_Base_Start(&htim7); 
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	Gimbal_Trigger_Control();
	Gimbal_YawSmall_Control();
	Gimbal_YawBig_Control();
	Gimbal_Shoot_Control();
	Gimbal_Pitch_Control();
	Gimbal_CtoC_Remote();
	if(Remote_Status == 1)
		Motor_6020_Voltage1((int16_t)BigYaw_SpeedPID.OUT, (int16_t)SmallYaw_SpeedPID.OUT, 0, 0, &hcan2);
	else if(Remote_Status == 0)
		Motor_6020_Voltage1(0, 0, 0, 0, &hcan2);
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	Remote_Status = 0;
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX1 interrupt.
  */
void CAN2_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX1_IRQn 0 */

  /* USER CODE END CAN2_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX1_IRQn 1 */

  /* USER CODE END CAN2_RX1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
//	
  uint8_t RxByte = 0; // 临时存储当前接收字节
	static uint8_t RxState = 0;          // 状态机：0-等待帧头1，1-等待帧头2，2-接收数据，3-校验CRC8，4-校验帧尾
	static uint8_t RxCount = 0;          // 数据接收计数器
	
	
	__HAL_TIM_SET_COUNTER(&htim11, 0);//timer7刷新
	HAL_TIM_Base_Stop(&htim11); 
	
 if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET)
 {
     RxByte = (uint8_t)(huart6.Instance->DR & 0xFF);

     // 接收状态机处理
     switch(RxState)
     {
         // 状态0：等待帧头1
         case 0:
             if(RxByte == NEW_FRAME_HEADER1)
             {
                 NewRxBuffer[0] = RxByte; // 存储帧头1
                 RxState = 1;             // 切换到等待帧头2
             }
             break;

         // 状态1：等待帧头2
         case 1:
             if(RxByte == NEW_FRAME_HEADER2)
             {
                 NewRxBuffer[1] = RxByte; // 存储帧头2
                 RxCount = 0;             // 重置数据计数器
                 RxState = 2;             // 切换到接收数据
             }
             else
             {
                 RxState = 0; // 帧头2错误，回到初始状态
             }
             break;

         // 状态2：接收数据（字节3~13，共11字节）
         case 2:
             NewRxBuffer[2 + RxCount] = RxByte; // 存储数据（从字节3开始）
             RxCount++;

             // 已接收完字节3~13（共11字节）
             if(RxCount >= NEW_DATA_BODY_LENGTH)
             {
                 RxState = 3; // 切换到校验CRC8
             }
             break;

         // 状态3：校验CRC8（字节14）
         case 3:
             NewRxBuffer[13] = RxByte; // 存储CRC8字节
             if (RxByte == NEW_FRAME_TAIL1)
             {
                 RxState = 4; // 校验成功，切换到校验帧尾2
             }
             else
             {
                 RxState = 0; // 帧尾1错误，重置状态机
             }
             break;

         // 状态4：校验帧尾（字节15）
         case 4:
             NewRxBuffer[14] = RxByte; // 存储帧尾字节
             if(RxByte == NEW_FRAME_TAIL2)
             {
                 NewRxData.data1 = (int16_t)NewRxBuffer[3] << 8 | NewRxBuffer[2];
                 NewRxData.data2 = (int16_t)NewRxBuffer[5] << 8 | NewRxBuffer[4];
                 NewRxData.data3 = (int16_t)NewRxBuffer[7] << 8 | NewRxBuffer[6];
                 NewRxData.data4 = (int16_t)NewRxBuffer[9] << 8 | NewRxBuffer[8];
                 NewRxData.data5 = (int16_t)NewRxBuffer[11] << 8 | NewRxBuffer[10];
                 
                 NewRxData.numA = (NewRxBuffer[12] >> 4) & 0x0F;
                 NewRxData.numB = NewRxBuffer[12] & 0x0F;

                 if((NewRxData.numA < 1 || NewRxData.numA > 3) || 
                    (NewRxData.numB < 1 || NewRxData.numB > 3))
                 {
                     memset(&NewRxData, 0, sizeof(NewRxDataStruct));
                 }
             }
             RxState = 0;
             RxCount = 0;
						 MiniPC_Flag = 1;
             break;

         default:
             RxState = 0;
             RxCount = 0;
             break;
     }
	}
 
	
	HAL_TIM_Base_Start(&htim11); 
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
