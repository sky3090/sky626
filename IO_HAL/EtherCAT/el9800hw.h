/*
* This source file is part of the EtherCAT Slave Stack Code licensed by Beckhoff Automation GmbH & Co KG, 33415 Verl, Germany.
* The corresponding license agreement applies. This hint shall not be removed.
*/

/**
 * \addtogroup EL9800_HW EL9800 Platform (Serial ESC Access)
 * @{
 */

/**
\file el9800hw.h
\author EthercatSSC@beckhoff.com
\brief Defines to access the EL9800 specific periphery and ESC (via SPI)

\version 5.11

<br>Changes to version V5.01:<br>
V5.11 ECAT10: change PROTO handling to prevent compiler errors<br>
<br>Changes to version - :<br>
V5.01 : Start file change log
 */

#ifndef _EL9800HW_H_
#define _EL9800HW_H_


/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include  "esc.h"

#include "stm32f4xx_hal.h"
//#include "adc.h"
//#include "dac.h"
//#include "dma.h"
#include "spi.h"
#include "tim.h"
//#include "usart.h"
#include "gpio.h"
//#include "fsmc.h"





#define SPI_Connect 1//宏定义ON--->接口模式选择 SPI
//											 //宏定义OFF-->接口模式选择 16bit_HBI
											 
/*-----------------------------------------------------------------------------------------
------
------    Defines and Types
------
-----------------------------------------------------------------------------------------*/
#define ESC_RD                    0x02 /**< \brief Indicates a read access to ESC or EEPROM*/
#define ESC_WR                    0x04 /**< \brief Indicates a write access to ESC or EEPROM.*/


/*---------------------------------------------
-    端口定义
-----------------------------------------------*/

#define SWITCH_1         HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_12)	 /**< \brief Access to switch 1 input*/
#define SWITCH_2         HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11)	 /**< \brief Access to switch 2 input*/
#define SWITCH_3         HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10)	 /**< \brief Access to switch 3 input*/
#define SWITCH_4         HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9) /**< \brief Access to switch 4 input*/
#define SWITCH_5         HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8) /**< \brief Access to switch 5 input*/
#define SWITCH_6         HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7) /**< \brief Access to switch 6 input*/
#define SWITCH_7         HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6) /**< \brief Access to switch 7 input*/
#define SWITCH_8         HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15) /**< \brief Access to switch 8 input*/

#define LED_1(i)         HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, i) /**< \brief Access to led 1 output*/
#define LED_2(i)         HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, i) /**< \brief Access to led 2 output*/
#define LED_3(i)         HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, i) /**< \brief Access to led 3 output*/
#define LED_4(i)         HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11, i) /**< \brief Access to led 4 output*/
#define LED_5(i)         HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, i) /**< \brief Access to led 5 output*/
#define LED_6(i)         HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0, i) /**< \brief Access to led 6 output*/
#define LED_7(i)         HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5, i) /**< \brief Access to led 7 output*/
#define LED_8(i)         HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,i) /**< \brief Access to led 8 output*/




/*---------------------------------------------
-    hardware timer settings
-----------------------------------------------*/

#define ECAT_TIMER_INC_P_MS                1000 /**< \brief 625 ticks per ms*/



/*---------------------------------------------
-    Interrupt and Timer defines
-----------------------------------------------*/

#ifndef DISABLE_ESC_INT
#define    DISABLE_ESC_INT()            HAL_NVIC_DisableIRQ(EXTI9_5_IRQn) /**< \brief Disable interrupt source INT1*/
#endif
#ifndef ENABLE_ESC_INT
#define    ENABLE_ESC_INT()            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn) /**< \brief Enable interrupt source INT1*/
#endif	

#ifndef HW_GetTimer
#define HW_GetTimer()        TIM2->CNT /**< \brief Access to the hardware timer*/
#endif

#ifndef HW_ClearTimer
#define HW_ClearTimer()        ((TIM2->CNT)=0);/**< \brief Clear the hardware timer*/
#endif




#ifdef SPI_Connect //SPI模式
extern UINT16 HW_GetALEventRegister(void);
extern UINT16 HW_GetALEventRegister_Isr(void);
extern void HW_EscRead( MEM_ADDR * pData, UINT16 Address, UINT16 Len );
extern void HW_EscReadIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len );
extern void HW_EscWrite( MEM_ADDR *pData, UINT16 Address, UINT16 Len );
extern void HW_EscWriteIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len );


#define HW_EscReadWord(WordValue, Address) HW_EscRead(((MEM_ADDR *)&(WordValue)),((UINT16)(Address)),2) /**< \brief 16Bit ESC read access*/
#define HW_EscReadDWord(DWordValue, Address) HW_EscRead(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4) /**< \brief 32Bit ESC read access*/
#define HW_EscReadMbxMem(pData,Address,Len) HW_EscRead(((MEM_ADDR *)(pData)),((UINT16)(Address)),(Len)) /**< \brief The mailbox data is stored in the local uC memory therefore the default read function is used.*/

#define HW_EscReadWordIsr(WordValue, Address) HW_EscReadIsr(((MEM_ADDR *)&(WordValue)),((UINT16)(Address)),2) /**< \brief Interrupt specific 16Bit ESC read access*/
#define HW_EscReadDWordIsr(DWordValue, Address) HW_EscReadIsr(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4) /**< \brief Interrupt specific 32Bit ESC read access*/


#define HW_EscWriteWord(WordValue, Address) HW_EscWrite(((MEM_ADDR *)&(WordValue)),((UINT16)(Address)),2) /**< \brief 16Bit ESC write access*/
#define HW_EscWriteDWord(DWordValue, Address) HW_EscWrite(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4) /**< \brief 32Bit ESC write access*/
#define HW_EscWriteMbxMem(pData,Address,Len) HW_EscWrite(((MEM_ADDR *)(pData)),((UINT16)(Address)),(Len)) /**< \brief The mailbox data is stored in the local uC memory therefore the default write function is used.*/

#define HW_EscWriteWordIsr(WordValue, Address) HW_EscWriteIsr(((MEM_ADDR *)&(WordValue)),((UINT16)(Address)),2) /**< \brief Interrupt specific 16Bit ESC write access*/
#define HW_EscWriteDWordIsr(DWordValue, Address) HW_EscWriteIsr(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4) /**< \brief Interrupt specific 32Bit ESC write access*/

#else  //FSMC并口模式  

#define     HW_GetALEventRegister()             (*(__IO UINT16 ESCMEM *)(NOR_MEMORY_ADRESS3+(ESC_AL_EVENT_OFFSET)))
																								 /**< \brief Returns the first 16Bit of the AL Event register (0x220)*/
#define     HW_GetALEventRegister_Isr           HW_GetALEventRegister /**< \brief Returns the first 16Bit of the AL Event register (0x220).<br>Called from ISRs.*/
																								
//#define     HW_GetALEventRegister_Isr()           (*(__IO UINT16 ESCMEM *)(NOR_MEMORY_ADRESS3+(ESC_AL_EVENT_OFFSET))) /**< \brief Returns the first 16Bit of the AL Event register (0x220)*/

#define     HW_EscRead(pData,Address,Len)  			ESCMEMCPY((MEM_ADDR *)(pData), (MEM_ADDR ESCMEM *)(NOR_MEMORY_ADRESS3+((Address))), (Len)) /**< \brief Generic ESC (register and DPRAM) read access.*/
#define     HW_EscReadIsr                       HW_EscRead /**< \brief Generic ESC (register and DPRAM) read access.<br>Called for ISRs.*/
//#define     HW_EscReadIsr(pData,Address,Len)  			ESCMEMCPY((MEM_ADDR *)(pData), (MEM_ADDR ESCMEM *)(NOR_MEMORY_ADRESS3+((Address))), (Len)) /**< \brief Generic ESC (register and DPRAM) read access.*/

#define     HW_EscWrite(pData,Address,Len)      ESCMEMCPY((MEM_ADDR ESCMEM *)(NOR_MEMORY_ADRESS3+(Address)), (MEM_ADDR *)(pData), (Len)) /**< \brief Generic ESC (register and DPRAM) write access.*/
#define     HW_EscWriteIsr                      HW_EscWrite /**< \brief Generic ESC (register and DPRAM) write access.<br>Called for ISRs.*/
//#define     HW_EscWriteIsr(pData,Address,Len)      ESCMEMCPY((MEM_ADDR ESCMEM *)(NOR_MEMORY_ADRESS3+(Address)), (MEM_ADDR *)(pData), (Len)) /**< \brief Generic ESC (register and DPRAM) write access.*/


#define HW_EscReadWord(WordValue, Address)   (WordValue=(uint16_t)(*(__IO uint16_t*) (NOR_MEMORY_ADRESS3 + Address)))  /**< \brief 16Bit ESC read access*/
#define HW_EscReadDWord(DWordValue, Address) (DWordValue=(uint32_t)(*(__IO uint32_t*) (NOR_MEMORY_ADRESS3 + Address))) /**< \brief 32Bit ESC read access*/
#define HW_EscReadMbxMem(pData,Address,Len)  ESCMBXMEMCPY((MEM_ADDR *)(pData), (MEM_ADDR ESCMEM *)(NOR_MEMORY_ADRESS3+((Address))), (Len)) /**< \brief The mailbox data is stored in the local uC memory therefore the default read function is used.*/

#define HW_EscReadWordIsr(WordValue, Address)   HW_EscReadWord(WordValue, Address) /**< \brief Interrupt specific 16Bit ESC read access*/
#define HW_EscReadDWordIsr(DWordValue, Address) HW_EscReadDWord(DWordValue, Address) /**< \brief Interrupt specific 32Bit ESC read access*/


#define HW_EscWriteWord(WordValue, Address) 	((*(__IO uint16_t*) (NOR_MEMORY_ADRESS3 + Address))=WordValue) /**< \brief 16Bit ESC write access*/
#define HW_EscWriteDWord(DWordValue, Address) ((*(__IO uint32_t*) (NOR_MEMORY_ADRESS3 + Address))=DWordValue) /**< \brief 32Bit ESC write access*/
#define HW_EscWriteMbxMem(pData,Address,Len) 	ESCMBXMEMCPY(((MEM_ADDR ESCMEM *)(NOR_MEMORY_ADRESS3+((Address)))),(MEM_ADDR *)(pData), (Len)) /**< \brief The mailbox data is stored in the local uC memory therefore the default write function is used.*/

#define HW_EscWriteWordIsr(WordValue, Address) 		HW_EscWriteWord(WordValue, Address) /**< \brief Interrupt specific 16Bit ESC write access*/
#define HW_EscWriteDWordIsr(DWordValue, Address) 	HW_EscWriteDWord(DWordValue, Address) /**< \brief Interrupt specific 32Bit ESC write access*/

#endif //SPI_Connect

#endif //_EL9800HW_H_

#if defined(_EL9800HW_) && (_EL9800HW_ == 1)
    #define PROTO
#else
    #define PROTO extern
#endif


/*-----------------------------------------------------------------------------------------
------
------    Global variables
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    Global functions
------
-----------------------------------------------------------------------------------------*/
PROTO UINT8 HW_Init(void);
PROTO void HW_Release(void);
PROTO void HW_SetLed(UINT8 RunLed,UINT8 ErrLed);




#undef    PROTO
/** @}*/

