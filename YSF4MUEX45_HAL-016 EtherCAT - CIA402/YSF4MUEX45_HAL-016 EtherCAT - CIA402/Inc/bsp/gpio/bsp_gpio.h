#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 --------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/
#define RST_ESC     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
#define RST_ESCEND  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);


/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void EXTI4_Configuration(void);
void EXTI14_Configuration(void);
void EXTI13_Configuration(void);
void RST_Configuration(void);

#endif  // __BSP_GPIO_H__

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
