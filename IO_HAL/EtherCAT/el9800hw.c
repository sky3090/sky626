/*
* This source file is part of the EtherCAT Slave Stack Code licensed by Beckhoff Automation GmbH & Co KG, 33415 Verl, Germany.
* The corresponding license agreement applies. This hint shall not be removed.
*/

/**
\addtogroup EL9800_HW EL9800 Platform (Serial ESC Access)
@{
*/

/**
\file    el9800hw.c
\author EthercatSSC@beckhoff.com
\brief Implementation
Hardware access implementation for EL9800 onboard PIC18/PIC24 connected via SPI to ESC

\version 5.12

<br>Changes to version V5.11:<br>
V5.12 EL9800 1: improve the SPI access<br>
<br>Changes to version V5.10:<br>
V5.11 ECAT10: change PROTO handling to prevent compiler errors<br>
V5.11 EL9800 2: change PDI access test to 32Bit ESC access and reset AL Event mask after test even if AL Event is not enabled<br>
<br>Changes to version V5.01:<br>
V5.10 ESC5: Add missing swapping<br>
V5.10 HW3: Sync1 Isr added<br>
V5.10 HW4: Add volatile directive for direct ESC DWORD/WORD/BYTE access<br>
           Add missing swapping in mcihw.c<br>
           Add "volatile" directive vor dummy variables in enable and disable SyncManger functions<br>
           Add missing swapping in EL9800hw files<br>
<br>Changes to version V5.0:<br>
V5.01 HW1: Invalid ESC access function was used<br>
<br>Changes to version V4.40:<br>
V5.0 ESC4: Save SM disable/Enable. Operation may be pending due to frame handling.<br>
<br>Changes to version V4.30:<br>
V4.40 : File renamed from spihw.c to el9800hw.c<br>
<br>Changes to version V4.20:<br>
V4.30 ESM: if mailbox Syncmanger is disabled and bMbxRunning is true the SyncManger settings need to be revalidate<br>
V4.30 EL9800: EL9800_x hardware initialization is moved to el9800.c<br>
V4.30 SYNC: change synchronisation control function. Add usage of 0x1C32:12 [SM missed counter].<br>
Calculate bus cycle time (0x1C32:02 ; 0x1C33:02) CalcSMCycleTime()<br>
V4.30 PDO: rename PDO specific functions (COE_xxMapping -> PDO_xxMapping and COE_Application -> ECAT_Application)<br>
V4.30 ESC: change requested address in GetInterruptRegister() to prevent acknowledge events.<br>
(e.g. reading an SM config register acknowledge SM change event)<br>
GENERIC: renamed several variables to identify used SPI if multiple interfaces are available<br>
V4.20 MBX 1: Add Mailbox queue support<br>
V4.20 SPI 1: include SPI RxBuffer dummy read<br>
V4.20 DC 1: Add Sync0 Handling<br>
V4.20 PIC24: Add EL9800_4 (PIC24) required source code<br>
V4.08 ECAT 3: The AlStatusCode is changed as parameter of the function AL_ControlInd<br>
<br>Changes to version V4.02:<br>
V4.03 SPI 1: In ISR_GetInterruptRegister the NOP-command should be used.<br>
<br>Changes to version V4.01:<br>
V4.02 SPI 1: In HW_OutputMapping the variable u16OldTimer shall not be set,<br>
             otherwise the watchdog might exceed too early.<br>
<br>Changes to version V4.00:<br>
V4.01 SPI 1: DI and DO were changed (DI is now an input for the uC, DO is now an output for the uC)<br>
V4.01 SPI 2: The SPI has to operate with Late-Sample = FALSE on the Eva-Board<br>
<br>Changes to version V3.20:<br>
V4.00 ECAT 1: The handling of the Sync Manager Parameter was included according to<br>
              the EtherCAT Guidelines and Protocol Enhancements Specification<br>
V4.00 APPL 1: The watchdog checking should be done by a microcontroller<br>
                 timer because the watchdog trigger of the ESC will be reset too<br>
                 if only a part of the sync manager data is written<br>
V4.00 APPL 4: The EEPROM access through the ESC is added

*/


/*--------------------------------------------------------------------------------------
------
------    Includes
------
--------------------------------------------------------------------------------------*/
#include "ecat_def.h"

#include "ecatslv.h"


#define    _EL9800HW_ 1
#include "el9800hw.h"
#undef    _EL9800HW_
/*remove definition of _EL9800HW_ (#ifdef is used in el9800hw.h)*/

#include "ecatappl.h"



/*--------------------------------------------------------------------------------------
------
------    internal Types and Defines
------
--------------------------------------------------------------------------------------*/

typedef union
{
    unsigned short    Word;
    unsigned char    Byte[2];
} UBYTETOWORD;

typedef union 
{
    UINT8           Byte[2];
    UINT16          Word;
}
UALEVENT;

/*-----------------------------------------------------------------------------------------
------
------    SPI defines/macros
------
-----------------------------------------------------------------------------------------*/

#define    SELECT_SPI                      		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
#define    DESELECT_SPI                    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

/*-----------------------------------------------------------------------------------------
------
------    Global Interrupt setting
------
-----------------------------------------------------------------------------------------*/

#define DISABLE_GLOBAL_INT           			  __set_PRIMASK(1) //__disable_irq()					
#define ENABLE_GLOBAL_INT           		    __set_PRIMASK(0) //__enable_irq()				
#define    DISABLE_AL_EVENT_INT          DISABLE_GLOBAL_INT
#define    ENABLE_AL_EVENT_INT           ENABLE_GLOBAL_INT


/*-----------------------------------------------------------------------------------------
------
------    ESC Interrupt
------
-----------------------------------------------------------------------------------------*/


#define    INIT_ESC_INT               MX_GPIO_Init();//EXTI0_Configuration(); //_INT1EP = 1:  negative edge ; _INT1IP = 1; //highest priority
//#define    ESC_INT_REQ                (_INT1IF) //ESC Interrupt (INT1) state
//#define    INT_EL                    (_RD8) //ESC Interrupt input port
#define    EscIsr                    EXTI9_5_IRQHandler // primary interrupt vector name
#define    ACK_ESC_INT               __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);




/*-----------------------------------------------------------------------------------------
------
------    SYNC0 Interrupt
------
-----------------------------------------------------------------------------------------*/

#define    INIT_SYNC0_INT                   MX_GPIO_Init();//EXTI1_Configuration();//_INT3EP = 1:  negative edge ; _INT3IP = 1; //highest priority
//#define    SYNC0_INT_REQ                    (_INT3IF) //Sync0 Interrupt (INT3) state
//#define    INT_SYNC0                        (_RD10) //Sync1 Interrupt input port
#define    Sync0Isr                        EXTI0_IRQHandler// primary interrupt vector name
#define    DISABLE_SYNC0_INT               HAL_NVIC_DisableIRQ(EXTI0_IRQn);//disable interrupt source INT3
#define    ENABLE_SYNC0_INT                HAL_NVIC_EnableIRQ(EXTI0_IRQn);	 //enable interrupt source INT3
#define    ACK_SYNC0_INT                   __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
//#define    SET_SYNC0_INT                    {(SYNC0_INT_REQ) = 1;}
//#define    SYNC0_INT_PORT_IS_ACTIVE        {(INT_EL) == 0;}


#define    INIT_SYNC1_INT                   MX_GPIO_Init();//EXTI2_Configuration();//_INT4EP = 1:  negative edge ; _INT4IP = 1; //highest priority
//#define    SYNC1_INT_REQ                    (_INT4IF) //Sync1 Interrupt (INT4) state
//#define    INT_SYNC1                        (_RD11) //Sync1 Interrupt input port
#define    Sync1Isr                        EXTI1_IRQHandler // primary interrupt vector name
#define    DISABLE_SYNC1_INT               HAL_NVIC_DisableIRQ(EXTI1_IRQn);//disable interrupt source INT4
#define    ENABLE_SYNC1_INT                HAL_NVIC_EnableIRQ(EXTI1_IRQn);  //enable interrupt source INT4
#define    ACK_SYNC1_INT                   __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
//#define    SET_SYNC1_INT                    {(SYNC1_INT_REQ) = 1;}
//#define    SYNC1_INT_PORT_IS_ACTIVE        {(INT_EL) == 0;}


/*-----------------------------------------------------------------------------------------
------
------    Hardware timer
------
-----------------------------------------------------------------------------------------*/



#define ECAT_TIMER_INT_STATE        
#define ECAT_TIMER_ACK_INT             __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE)	
#define    TimerIsr                    TIM2_IRQHandler	
#define    ENABLE_ECAT_TIMER_INT       {HAL_NVIC_EnableIRQ(TIM2_IRQn) ;HAL_TIM_Base_Start_IT(&htim2);}
#define    DISABLE_ECAT_TIMER_INT      {HAL_NVIC_DisableIRQ(TIM2_IRQn);HAL_TIM_Base_Stop_IT(&htim2);}
//desired period 1ms (625 counts at 40 MHz and prescale 1:64)
#define INIT_ECAT_TIMER                MX_TIM2_Init();
#define STOP_ECAT_TIMER                {DISABLE_ECAT_TIMER_INT;/*disable timer interrupt*/  /*disable timer*/}

#define START_ECAT_TIMER            {ENABLE_ECAT_TIMER_INT; /*enable timer interrupt*/  /*enable timer*/}




/*-----------------------------------------------------------------------------------------
------
------    Configuration Bits
------
-----------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------
------
------    LED defines
------
-----------------------------------------------------------------------------------------*/
// EtherCAT Status LEDs -> StateMachine
#define LED_ECATGREEN                 
#define LED_ECATRED                    

/*--------------------------------------------------------------------------------------
------
------    internal Variables
------
--------------------------------------------------------------------------------------*/
UALEVENT         EscALEvent;            //contains the content of the ALEvent register (0x220), this variable is updated on each Access to the Esc
/*--------------------------------------------------------------------------------------
------
------    internal functions
------
--------------------------------------------------------------------------------------*/

/*ECATCHANGE_START(V5.12) EL9800 1*/
static UINT8 RxTxSpiData(UINT8 MosiByte)
{
     UINT8 MisoByte = 0;   
//		HAL_SPI_Transmit(&hspi3, &MosiByte, 1,1000);
//		HAL_SPI_Receive(&hspi3,&MisoByte,1,1000);	
		HAL_SPI_TransmitReceive(&hspi2,&MosiByte,&MisoByte,1,1000);
    return MisoByte;

//	SPI3->DR = MosiByte;        //发送数据
//	while ((SPI3->SR & SPI_FLAG_RXNE) == 0x00) ;
//	return SPI3->DR;	
}


///////////////////////////////////////////////////////////////////////////////////////
/**
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Command    ESC_WR performs a write access; ESC_RD performs a read access.

 \brief The function addresses the EtherCAT ASIC via SPI for a following SPI access.
*////////////////////////////////////////////////////////////////////////////////////////
static void AddressingEsc( UINT16 Address, UINT8 Command )
{
    VARVOLATILE UBYTETOWORD tmp;
    tmp.Word = ( Address << 3 ) | Command;



    /* select the SPI */
    SELECT_SPI;

    /* send the first address/command byte to the ESC 
       receive the first AL Event Byte*/
    EscALEvent.Byte[0] = RxTxSpiData(tmp.Byte[1]);

    EscALEvent.Byte[1] = RxTxSpiData(tmp.Byte[0]);

}

///////////////////////////////////////////////////////////////////////////////////////
/**
 \brief  The function operates a SPI access without addressing.

        The first two bytes of an access to the EtherCAT ASIC always deliver the AL_Event register (0x220).
        It will be saved in the global "EscALEvent"
*////////////////////////////////////////////////////////////////////////////////////////

static void GetInterruptRegister(void)
{
    DISABLE_AL_EVENT_INT;//关中断
    SELECT_SPI;
		HW_EscReadIsr((MEM_ADDR*)&EscALEvent.Word, ESC_AL_EVENT_OFFSET, 2);
		DESELECT_SPI;	
    ENABLE_AL_EVENT_INT;//开中断
}

///////////////////////////////////////////////////////////////////////////////////////
/**
 \brief  The function operates a SPI access without addressing.
        Shall be implemented if interrupts are supported else this function is equal to "GetInterruptRegsiter()"

        The first two bytes of an access to the EtherCAT ASIC always deliver the AL_Event register (0x220).
        It will be saved in the global "EscALEvent"
*////////////////////////////////////////////////////////////////////////////////////////

static void ISR_GetInterruptRegister(void)
{
    DESELECT_SPI;

    /* select the SPI */
    SELECT_SPI;	
		HW_EscReadIsr((MEM_ADDR*)&EscALEvent.Word, ESC_AL_EVENT_OFFSET, 2);
    DESELECT_SPI;
}


/*ECATCHANGE_END(V5.12) EL9800 1*/

/*--------------------------------------------------------------------------------------
------
------    exported hardware access functions
------
--------------------------------------------------------------------------------------*/


/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     0 if initialization was successful

 \brief    This function intialize the Process Data Interface (PDI) and the host controller.
*////////////////////////////////////////////////////////////////////////////////////////
UINT8 HW_Init(void)
{
    UINT32 intMask;
//#ifdef SPI_Connect	
//    MX_SPI3_Init(); //SPI模式
//#else
//		MX_FSMC_Init(); //并口模式
//#endif
//		MX_TIM2_Init();	
//		HAL_TIM_Base_Start_IT(&htim2);
		HW_EscReadWord(intMask, 0x00);//驱动测试：读取ESC Information->Type返回值0xc8
    do
    {
        intMask = 0x2493;
        HW_EscWriteWord(intMask, ESC_AL_EVENTMASK_OFFSET);
        intMask = 0;
        HW_EscReadWord(intMask, ESC_AL_EVENTMASK_OFFSET);
    } while (intMask != 0x2493);

    intMask = 0x00;
    HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);

/******************************
******************************/
//    INIT_ESC_INT;//配置IRQ中断
		ENABLE_ESC_INT(); 

//    INIT_SYNC0_INT;//配置SYNC0中断
//    INIT_SYNC1_INT;

    ENABLE_SYNC0_INT;//配置SYNC1中断
    ENABLE_SYNC1_INT;

//    INIT_ECAT_TIMER;//配置定时器2
    START_ECAT_TIMER;

    /* enable all interrupts */
    ENABLE_GLOBAL_INT;

    return 0;
}


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    This function shall be implemented if hardware resources need to be release
        when the sample application stops
*////////////////////////////////////////////////////////////////////////////////////////
void HW_Release(void)
{
}
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param RunLed            desired EtherCAT Run led state
 \param ErrLed            desired EtherCAT Error led state

  \brief    This function updates the EtherCAT run and error led
*////////////////////////////////////////////////////////////////////////////////////////
void HW_SetLed(UINT8 RunLed,UINT8 ErrLed)
{
//      LED_ECATGREEN = RunLed;
//      LED_ECATRED   = ErrLed;
}

#ifdef SPI_Connect //SPI模式
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    first two Bytes of ALEvent register (0x220)

 \brief  This function gets the current content of ALEvent register
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 HW_GetALEventRegister(void)
{
    GetInterruptRegister();
    return EscALEvent.Word;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    first two Bytes of ALEvent register (0x220)

 \brief  The SPI PDI requires an extra ESC read access functions from interrupts service routines.
        The behaviour is equal to "HW_GetALEventRegister()"
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 HW_GetALEventRegister_Isr(void)
{
     ISR_GetInterruptRegister();
    return EscALEvent.Word;
}


///////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves read data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

 \brief  This function operates the SPI read access to the EtherCAT ASIC.
*////////////////////////////////////////////////////////////////////////////////////////
void HW_EscRead( MEM_ADDR *pData, UINT16 Address, UINT16 Len )
{
    /* HBu 24.01.06: if the SPI will be read by an interrupt routine too the
                     mailbox reading may be interrupted but an interrupted
                     reading will remain in a SPI transmission fault that will
                     reset the internal Sync Manager status. Therefore the reading
                     will be divided in 1-byte reads with disabled interrupt */
    UINT16 i = Len;
    UINT8 *pTmpData = (UINT8 *)pData;
		UINT8 ADDC=0;

    /* loop for all bytes to be read */
    while ( i-- > 0 )
    {
			#if AL_EVENT_ENABLED
        /* the reading of data from the ESC can be interrupted by the
           AL Event ISR, in that case the address has to be reinitialized,
           in that case the status flag will indicate an error because
           the reading operation was interrupted without setting the last
           sent byte to 0xFF */
        DISABLE_AL_EVENT_INT;
#endif
         AddressingEsc( Address, ESC_RD );
			
//			HAL_SPI_Transmit(&hspi3, &ADDC, 1,1000);
//			HAL_SPI_Receive(&hspi3,pTmpData,1,1000);
        /* when reading the last byte the DI pin shall be 1 */
			*pTmpData++ = RxTxSpiData(0xFF);
        /* enable the ESC interrupt to get the AL Event ISR the chance to interrupt,
           if the next byte is the last the transmission shall not be interrupted,
           otherwise a sync manager could unlock the buffer, because the last was
           read internally */
   #if AL_EVENT_ENABLED
        ENABLE_AL_EVENT_INT;
#endif
        /* there has to be at least 15 ns + CLK/2 after the transmission is finished
           before the SPI1_SEL signal shall be 1 */
        DESELECT_SPI;
        /* next address */
        Address++;
//        /* reset transmission flag */
//        SPI1_IF = 0;
    }
}

/*ECATCHANGE_START(V5.12) EL9800 1*/
///////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves read data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

 \brief  This function operates the SPI read access to the EtherCAT ASIC.
*////////////////////////////////////////////////////////////////////////////////////////


void HW_EscReadIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len )
{
    UINT16 i = Len;
    UINT8 data = 0;

   UINT8 *pTmpData = (UINT8 *)pData;

    /* send the address and command to the ESC */
     AddressingEsc( Address, ESC_RD );
    /* loop for all bytes to be read */
    while ( i-- > 0 )
    {
        if ( i == 0 )
        {
            /* when reading the last byte the DI pin shall be 1 */
            data = 0xFF;
        }
		*pTmpData++= RxTxSpiData(data);
    }

    /* there has to be at least 15 ns + CLK/2 after the transmission is finished
       before the SPI1_SEL signal shall be 1 */
    DESELECT_SPI;
}
///////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves write data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

  \brief  This function operates the SPI write access to the EtherCAT ASIC.
*////////////////////////////////////////////////////////////////////////////////////////
void HW_EscWrite( MEM_ADDR *pData, UINT16 Address, UINT16 Len )
{
    UINT16 i = Len;
    VARVOLATILE UINT8 dummy;

    UINT8 *pTmpData = (UINT8 *)pData;

    /* loop for all bytes to be written */
    while ( i-- > 0 )
    {
   #if AL_EVENT_ENABLED
        /* the reading of data from the ESC can be interrupted by the
           AL Event ISR, so every byte will be written separate */
        DISABLE_AL_EVENT_INT;
#endif
        /* HBu 24.01.06: wrong parameter ESC_RD */
         AddressingEsc( Address, ESC_WR );

        /* enable the ESC interrupt to get the AL Event ISR the chance to interrupt */
        /* SPI1_BUF must be read, otherwise the module will not transfer the next received data from SPIxSR to SPIxRXB.*/
			dummy= RxTxSpiData(*pTmpData++);
			
    #if AL_EVENT_ENABLED
        ENABLE_AL_EVENT_INT;
#endif

        DESELECT_SPI;
        /* next address */
        Address++;
    }
}

///////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves write data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

 \brief  The SPI PDI requires an extra ESC write access functions from interrupts service routines.
        The behaviour is equal to "HW_EscWrite()"
*////////////////////////////////////////////////////////////////////////////////////////
/*ECATCHANGE_END(V5.12) EL9800 1*/
void HW_EscWriteIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len )
{
    UINT16 i = Len;
    VARVOLATILE UINT16 dummy;
    UINT8 *pTmpData = (UINT8 *)pData;

    /* send the address and command to the ESC */
     AddressingEsc( Address, ESC_WR );
    /* loop for all bytes to be written */
    while ( i-- > 0 )
    {
        /* start transmission */
			dummy= RxTxSpiData(*pTmpData);
      /* increment data pointer */
        pTmpData++;
    }

    /* there has to be at least 15 ns + CLK/2 after the transmission is finished
       before the SPI1_SEL signal shall be 1 */
    DESELECT_SPI;
}
#endif


///////////////////////////////////////////////////////
//UINT16 HW_GetALEventRegister(void)
//{
//	UINT16 ALeventr=0;
//	DISABLE_AL_EVENT_INT;
//	ALeventr=(*(__IO UINT16 ESCMEM *)(NOR_MEMORY_ADRESS3+(ESC_AL_EVENT_OFFSET)));
//	ENABLE_AL_EVENT_INT;
//	return ALeventr;
//}



/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    Interrupt service routine for the PDI interrupt from the EtherCAT Slave Controller
*////////////////////////////////////////////////////////////////////////////////////////

void  EscIsr(void)//IQR中断函数
{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);			
		PDI_Isr();
    /* reset the interrupt flag */
    ACK_ESC_INT;
}



/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    Interrupt service routine for the interrupts from SYNC0
*////////////////////////////////////////////////////////////////////////////////////////
void Sync0Isr(void)//同步0中断函数
{		
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    Sync0_Isr();
    /* reset the interrupt flag */
    ACK_SYNC0_INT;
}
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    Interrupt service routine for the interrupts from SYNC1
*////////////////////////////////////////////////////////////////////////////////////////
void Sync1Isr(void)//同步1中断函数
{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1); 
		Sync1_Isr();
    /* reset the interrupt flag */
    ACK_SYNC1_INT;
}



UINT16 SYSruncon;//运行计数
//BOOL LEDtus=0;
// Timer 2 ISR (1ms)
void  TimerIsr(void)//定时器2中断函数
{
	HAL_TIM_IRQHandler(&htim2);
	ECAT_CheckTimer();
  ECAT_TIMER_ACK_INT;
	SYSruncon++;
	if(SYSruncon>=500)//500ms--系统呼吸指示灯
	{
//		LEDtus=!LEDtus;
		SYSruncon=0;
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14, LEDtus);
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);
	}

}

/** @} */
