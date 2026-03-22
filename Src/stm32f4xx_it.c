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
#define NEW_FRAME_TAIL       0xEE    // 自定义帧尾，可根据实际需求修改
#define NEW_FRAME_LENGTH     15      // 帧总长度15字节
#define NEW_DATA_BODY_LENGTH 12      // 帧头后到CRC8前的长度（15-3=12）

uint8_t NewRxBuffer[NEW_FRAME_LENGTH] = {0};

const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0x3B, 0x3C, 0x35, 0x32, 0x27, 0x20, 0x29, 0x2E, 0x03, 0x04, 0x0D, 0x0A, 0x1F, 0x18, 0x11, 0x16,
    0x4B, 0x4C, 0x45, 0x42, 0x57, 0x50, 0x59, 0x5E, 0x73, 0x74, 0x7D, 0x7A, 0x6F, 0x68, 0x61, 0x66,
    0xDB, 0xDC, 0xD5, 0xD2, 0xC7, 0xC0, 0xC9, 0xCE, 0xE3, 0xE4, 0xED, 0xEA, 0xFF, 0xF8, 0xF1, 0xF6,
    0xAB, 0xAC, 0xA5, 0xA2, 0xB7, 0xB0, 0xB9, 0xBE, 0x93, 0x94, 0x9D, 0x9A, 0x8F, 0x88, 0x81, 0x86,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63, 0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13, 0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83, 0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3, 0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0x4D, 0x4A, 0x43, 0x44, 0x51, 0x56, 0x5F, 0x58, 0x75, 0x72, 0x7B, 0x7C, 0x69, 0x6E, 0x67, 0x60,
    0x3D, 0x3A, 0x33, 0x34, 0x21, 0x26, 0x2F, 0x28, 0x05, 0x02, 0x0B, 0x0C, 0x19, 0x1E, 0x17, 0x10,
    0xAD, 0xAA, 0xA3, 0xA4, 0xB1, 0xB6, 0xBF, 0xB8, 0x95, 0x92, 0x9B, 0x9C, 0x89, 0x8E, 0x87, 0x80,
    0xDD, 0xDA, 0xD3, 0xD4, 0xC1, 0xC6, 0xCF, 0xC8, 0xE5, 0xE2, 0xEB, 0xEC, 0xF9, 0xFE, 0xF7, 0xF0
};

typedef struct {
    uint16_t data1;   // 字节3-4
    uint16_t data2;   // 字节5-6
    uint16_t data3;   // 字节7-8
    uint16_t data4;   // 字节9-10
    uint16_t data5;   // 字节11-12
    uint8_t  numA;    // 字节13高4位（1/2/3）
    uint8_t  numB;    // 字节13低4位（1/2/3）
} NewRxDataStruct;
NewRxDataStruct NewRxData = {0};     // 解析后的数据


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//CRC校验
uint8_t RefereeSystem_VerifyCRC8CheckSum(const uint8_t* msg, uint32_t length) {
    uint8_t crc = 0x00; // 初始值
    for (uint32_t i = 0; i < length; i++) {
        crc = crc8_table[crc ^ msg[i]];
    }
    return crc;
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

extern PID_PositionInitTypedef BigYaw_SpeedPID;
extern PID_PositionInitTypedef SmallYaw_SpeedPID;
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
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
  {
      Remote_UART_IDLE_Callback();
  }
	HAL_IWDG_Refresh(&hiwdg);
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
//	Gimbal_Trigger_Control();
//	Gimbal_YawSmall_Control();
//	Gimbal_YawBig_Control();
//	Gimbal_Shoot_Control();
//	Gimbal_Pitch_Control();
//	Gimbal_CtoC_Remote();
//  Motor_6020_Voltage1((int16_t)BigYaw_SpeedPID.OUT, (int16_t)SmallYaw_SpeedPID.OUT, 0, 0, &hcan2);
	CToC_MasterSendData(	0,0,0,0,
												&hcan1, 0x149);
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
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
	
  uint8_t RxByte = 0; // 临时存储当前接收字节
	static uint8_t RxState = 0;          // 状态机：0-等待帧头1，1-等待帧头2，2-接收数据，3-校验CRC8，4-校验帧尾
	static uint8_t RxCount = 0;          // 数据接收计数器
	
    if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET)
    {
        // 读取接收字节（使用正确的HAL方法，或直接操作寄存器）
        // 方法1: 直接从DR寄存器读取 (推荐，因为这是在中断里)
        RxByte = (uint8_t)(huart6.Instance->DR & 0xFF);
        // 方法2: 使用HAL库宏 (效果相同，但更明确)
        // RxByte = (uint8_t)__HAL_UART_GET_DATA(&huart6);

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
                // 校验CRC8（校验范围：字节1~13，共13字节）
                if(RefereeSystem_VerifyCRC8CheckSum(NewRxBuffer, 13) == 1)
                {
                    RxState = 4; // CRC8校验成功，切换到校验帧尾
                }
                else
                {
                    RxState = 0; // CRC8校验失败，重置状态机
                }
                break;

            // 状态4：校验帧尾（字节15）
            case 4:
                NewRxBuffer[14] = RxByte; // 存储帧尾字节
                if(RxByte == NEW_FRAME_TAIL)
                {
                    // 帧尾正确，解析数据
                    // 解析两字节数据（低字节在前，若实际是高字节在前则交换顺序）
                    NewRxData.data1 = (uint16_t)NewRxBuffer[3] << 8 | NewRxBuffer[2];
                    NewRxData.data2 = (uint16_t)NewRxBuffer[5] << 8 | NewRxBuffer[4];
                    NewRxData.data3 = (uint16_t)NewRxBuffer[7] << 8 | NewRxBuffer[6];
                    NewRxData.data4 = (uint16_t)NewRxBuffer[9] << 8 | NewRxBuffer[8];
                    NewRxData.data5 = (uint16_t)NewRxBuffer[11] << 8 | NewRxBuffer[10];
                    
                    // 解析字节13：高4位=numA，低4位=numB
                    NewRxData.numA = (NewRxBuffer[12] >> 4) & 0x0F;
                    NewRxData.numB = NewRxBuffer[12] & 0x0F;

                    // 可选：验证numA/numB是否为1/2/3
                    if((NewRxData.numA < 1 || NewRxData.numA > 3) || 
                       (NewRxData.numB < 1 || NewRxData.numB > 3))
                    {
                        // 数值非法，清空数据（可选逻辑）
                        memset(&NewRxData, 0, sizeof(NewRxDataStruct));
                    }
                }
                // 无论帧尾是否正确，都重置状态机准备下一次接收
                RxState = 0;
                RxCount = 0;
                break;

            // 默认状态：重置
            default:
                RxState = 0;
                RxCount = 0;
                break;
        }
	}
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
