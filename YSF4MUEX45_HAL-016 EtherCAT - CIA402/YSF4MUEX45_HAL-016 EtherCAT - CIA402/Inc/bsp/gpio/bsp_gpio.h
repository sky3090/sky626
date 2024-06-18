#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� --------------------------------------------------------------*/

/* �궨�� --------------------------------------------------------------------*/
#define RST_ESC     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
#define RST_ESCEND  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);


/* ��չ���� ------------------------------------------------------------------*/
/* �������� ------------------------------------------------------------------*/
void EXTI4_Configuration(void);
void EXTI14_Configuration(void);
void EXTI13_Configuration(void);
void RST_Configuration(void);

#endif  // __BSP_GPIO_H__

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
