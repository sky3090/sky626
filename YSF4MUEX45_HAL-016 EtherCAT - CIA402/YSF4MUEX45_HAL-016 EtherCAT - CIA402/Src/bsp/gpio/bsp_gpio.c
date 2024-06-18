/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-3-30
  * 功    能: GPIO-输入检测
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F4Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "gpio/bsp_gpio.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

void RST_Configuration(void)
{
   /* 定义IO硬件初始化结构体变量 */
  GPIO_InitTypeDef GPIO_InitStruct;
	
  __HAL_RCC_GPIOD_CLK_ENABLE();	
    /**  GPIO Configuration    
    PD2     ------> RST
    */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); 
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);  

}

void EXTI4_Configuration(void)
{
   /* 定义IO硬件初始化结构体变量 */
  GPIO_InitTypeDef GPIO_InitStruct;
	
  __HAL_RCC_GPIOD_CLK_ENABLE();	
    /**  GPIO Configuration    
    PD4     ------> IRQ
    */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);   
	
  HAL_NVIC_SetPriority(EXTI4_IRQn,0, 0);
  //HAL_NVIC_EnableIRQ(EXTI4_IRQn);	
}

void EXTI14_Configuration(void)
{
   /* 定义IO硬件初始化结构体变量 */
  GPIO_InitTypeDef GPIO_InitStruct;	
	
  __HAL_RCC_GPIOE_CLK_ENABLE();	
    /**  GPIO Configuration    
    PE14     ------> SYNC0
    */	
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);   
		
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 1);
 // HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	
}

void EXTI13_Configuration(void)
{
   /* 定义IO硬件初始化结构体变量 */
  GPIO_InitTypeDef GPIO_InitStruct;	
	
  __HAL_RCC_GPIOE_CLK_ENABLE();	
    /**  GPIO Configuration    
    PE13     ------> SYNC1
    */	
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);   
		
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 1);
 // HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
