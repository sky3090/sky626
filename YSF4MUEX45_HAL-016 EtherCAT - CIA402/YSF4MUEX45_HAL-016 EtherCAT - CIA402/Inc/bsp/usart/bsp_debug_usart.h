/**
  ******************************************************************************
  * �ļ�����: bsp_debug_usart.h
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2022-11-20
  * ��    ��: bsp_debug_usartͷ�ļ�
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4STDʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
#ifndef __BSP_DEBUG_USART_H__
#define __BSP_DEBUG_USART_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* ���Ͷ��� ------------------------------------------------------------------*/

/* �궨�� --------------------------------------------------------------------*/
#define DEBUG_USARTx                                 USART2
#define DEBUG_USARTx_BAUDRATE                        115200
#define DEBUG_USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART2_CLK_ENABLE()
#define DEBUG_USART_RCC_CLK_DISABLE()                __HAL_RCC_USART2_CLK_DISABLE()

#define DEBUG_USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()
#define DEBUG_USARTx_Tx_GPIO_PIN                     GPIO_PIN_5
#define DEBUG_USARTx_Tx_GPIO                         GPIOD
#define DEBUG_USARTx_Rx_GPIO_PIN                     GPIO_PIN_6
#define DEBUG_USARTx_Rx_GPIO                         GPIOD

#define DEBUG_USARTx_AFx                             GPIO_AF7_USART2

#define DEBUG_USART_IRQn                             USART2_IRQn

/* ��չ���� ------------------------------------------------------------------*/
extern UART_HandleTypeDef husart_debug;

/* �������� ------------------------------------------------------------------*/
void MX_DEBUG_USART_Init(void);


#endif  /* __BSP_DEBUG_USART_H__ */

/******************* (C) COPYRIGHT 2020-2030 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
