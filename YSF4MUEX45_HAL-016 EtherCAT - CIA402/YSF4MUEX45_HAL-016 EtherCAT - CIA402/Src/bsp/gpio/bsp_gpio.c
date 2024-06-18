/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-3-30
  * ��    ��: GPIO-������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "gpio/bsp_gpio.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

void RST_Configuration(void)
{
   /* ����IOӲ����ʼ���ṹ����� */
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
   /* ����IOӲ����ʼ���ṹ����� */
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
   /* ����IOӲ����ʼ���ṹ����� */
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
   /* ����IOӲ����ʼ���ṹ����� */
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

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
