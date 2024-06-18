/**
  ******************************************************************************
  * �ļ�����: bsp_spiflash.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2022-11-20
  * ��    ��: ���ش���Flash�ײ�����ʵ��
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4STDʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "spiflash/bsp_spiflash.h"
#include "usart/bsp_debug_usart.h"
#include "ecat_def.h"
#include "el9800hw.h"
#include "core_cm4.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#define Dummy_Byte                      0xFF

/* ˽�б��� ------------------------------------------------------------------*/
SPI_HandleTypeDef hspix;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ����FLASH��ʼ��
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
*/
void MX_SPIFlash_Init(void)
{
  hspix.Instance = SPI3;
  hspix.Init.Mode = SPI_MODE_MASTER;//��ģʽ
  hspix.Init.Direction = SPI_DIRECTION_2LINES;//˫��
  hspix.Init.DataSize = SPI_DATASIZE_8BIT;//8λ����֡����
  hspix.Init.CLKPolarity = SPI_POLARITY_LOW;//ʱ�Ӽ��Ե�
  hspix.Init.CLKPhase = SPI_PHASE_1EDGE;//����λ
  hspix.Init.NSS = SPI_NSS_SOFT;//���
  hspix.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;//Ԥ��Ƶϵ��Ϊ4
  hspix.Init.FirstBit = SPI_FIRSTBIT_MSB;//MSB����
  hspix.Init.TIMode = SPI_TIMODE_DISABLE;
  hspix.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;//CRCУ��ʧ��
  hspix.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspix);
	
}

/**
  * ��������: SPI����ϵͳ����ʼ��
  * �������: hspi��SPI�������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hspi->Instance==SPI3)
  {
    /* SPI����ʱ��ʹ�� */
    FLASH_SPIx_RCC_CLK_ENABLE();
    /* GPIO����ʱ��ʹ�� */
    FLASH_SPI_SCK_ClK_ENABLE();
    FLASH_SPI_MISO_ClK_ENABLE();    
		FLASH_SPI_MOSI_ClK_ENABLE();    
		FLASH_SPI_CS_ClK_ENABLE();    
    /**SPI3 GPIO Configuration    
    PD11    ------> SPI3_CS
    PC10    ------> SPI3_SCK
    PC11    ------> SPI3_MISO
    PC12    ------> SPI3_MOSI 
    */
    GPIO_InitStruct.Pin = FLASH_SPI_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AFx_SPIx;
    HAL_GPIO_Init(FLASH_SPI_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FLASH_SPI_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = GPIO_AFx_SPIx;
    HAL_GPIO_Init(FLASH_SPI_MISO_PORT, &GPIO_InitStruct);
		
    GPIO_InitStruct.Pin = FLASH_SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AFx_SPIx;
    HAL_GPIO_Init(FLASH_SPI_MOSI_PORT, &GPIO_InitStruct);
				
    HAL_GPIO_WritePin(FLASH_SPI_CS_PORT, FLASH_SPI_CS_PIN, GPIO_PIN_SET);		
    GPIO_InitStruct.Pin = FLASH_SPI_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;		
    HAL_GPIO_Init(FLASH_SPI_CS_PORT, &GPIO_InitStruct);
  }
}

/*******************************************************************************
* Function Name  : WR_CMD
* Description    : Read and Wire data to ET1100 
* Input          : - cmd: the data send to ET1100
* Output         : none
* Return         : temp: the data read from ET1100
* Attention		 : None
*******************************************************************************/
uint8_t WR_CMD (uint8_t cmd)  
{ 

  uint8_t temp,d_send=cmd;
	
  if(HAL_SPI_TransmitReceive(&hspix,&d_send,&temp,1,0xFFFFFF)!=HAL_OK)
    temp=Dummy_Byte;
 
  return temp; 

} 

/*******************************************************************************
* Function Name  : SPIWrite
* Description    : Wire byte data to lan9252
* Input          : - data: the data send to LAN9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIWrite(UINT8 data)
{
   //WR_CMD(data);
	HAL_SPI_Transmit(&hspix, &data, 1, 2000);
}

/*******************************************************************************
* Function Name  : SPIRead
* Description    : read byte data from lan9252
* Input          : none
* Output         : none
* Return         : data: read from lan9252
* Attention		 : None
*******************************************************************************/
UINT8 SPIRead(void)
{
//    UINT8 data;			
//    data = WR_CMD(0);
//    return (data);
	
   UINT8 data;   
    HAL_SPI_Receive(&hspix, &data, 1, 2000);
    return (data);	
}


/*******************************************************************************
* Function Name  : SPIReadDWord
* Description    : read word data from lan9252
* Input          : Address��the addreoss to be read from lan9252
* Output         : none
* Return         : data: word data from lan9252
* Attention		 : None
*******************************************************************************/
uint32_t SPIReadDWord (uint16_t Address)
{
    UINT32_VAL dwResult;
    UINT16_VAL wAddr;
		
    wAddr.Val  = Address;
    //Assert CS line
    CSLOW();
    //Write Command
     SPIWriteByte(CMD_FAST_READ);
    //Write Address
    SPIWriteByte(wAddr.byte.HB);
    SPIWriteByte(wAddr.byte.LB);
    
    //Dummy Byte
    SPIWriteByte(CMD_FAST_READ_DUMMY);
	  
    //Read Bytes
    dwResult.byte.LB = SPIReadByte();
    dwResult.byte.HB = SPIReadByte();
    dwResult.byte.UB = SPIReadByte();
    dwResult.byte.MB = SPIReadByte();
    //De-Assert CS line
    CSHIGH();
   
    return dwResult.Val;
}
/*******************************************************************************
* Function Name  : SPISendAddr
* Description    : write address to lan9252
* Input          : Address��the address write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPISendAddr (uint16_t Address)
{
    UINT16_VAL wAddr;

    wAddr.Val  = Address;
    //Write Address
    SPIWriteByte(wAddr.byte.HB);
    SPIWriteByte(wAddr.byte.LB);
}

/*******************************************************************************
* Function Name  : SPIReadBurstMode
* Description    : Read word from lan9252 in burst mode
* Input          : none
* Output         : none
* Return         : word data from lan9252
* Attention		 : None
*******************************************************************************/
UINT32 SPIReadBurstMode ()
{
    UINT32_VAL dwResult;
    //Read Bytes
    dwResult.byte.LB = SPIReadByte();
    dwResult.byte.HB = SPIReadByte();
    dwResult.byte.UB = SPIReadByte();
    dwResult.byte.MB = SPIReadByte();
    
    return dwResult.Val;
}

/*******************************************************************************
* Function Name  : SPIWriteBurstMode
* Description    : write data to lan9252 in burst mode
* Input          : val��the data write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIWriteBurstMode (uint32_t Val)
{
    UINT32_VAL dwData;
    dwData.Val = Val;
    
    //Write Bytes
    SPIWriteByte(dwData.byte.LB);
    SPIWriteByte(dwData.byte.HB);
    SPIWriteByte(dwData.byte.UB);
    SPIWriteByte(dwData.byte.MB);
}
/*******************************************************************************
* Function Name  : SPIWriteDWord
* Description    : write Word lan9252 in 
* Input          : Address the address write to lan9252
										val��the data write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIWriteDWord (uint16_t Address, uint32_t Val)
{
    UINT32_VAL dwData;
    UINT16_VAL wAddr;

    wAddr.Val  = Address;
    dwData.Val = Val;
    //Assert CS line
    CSLOW();
    //Write Command
    SPIWriteByte(CMD_SERIAL_WRITE);
    //Write Address
    SPIWriteByte(wAddr.byte.HB);
    SPIWriteByte(wAddr.byte.LB);
    //Write Bytes
    SPIWriteByte(dwData.byte.LB);
    SPIWriteByte(dwData.byte.HB);
    SPIWriteByte(dwData.byte.UB);
    SPIWriteByte(dwData.byte.MB);

    //De-Assert CS line
    CSHIGH();
}

/*******************************************************************************
* Function Name  : SPIReadRegUsingCSR
* Description    : Read data from lan9252 use CSR
* Input          : ReadBuffer:data buf 
									 Address��the reg address write to lan9252
										Count:the number write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIReadRegUsingCSR(uint8_t *ReadBuffer, uint16_t Address, uint8_t Count)
{
    UINT32_VAL param32_1 = {0};
    UINT8 i = 0;
    UINT16_VAL wAddr;
    wAddr.Val = Address;

    param32_1.v[0] = wAddr.byte.LB;
    param32_1.v[1] = wAddr.byte.HB;
    param32_1.v[2] = Count;
    param32_1.v[3] = ESC_READ_BYTE;

    SPIWriteDWord (ESC_CSR_CMD_REG, param32_1.Val);

    do
    {
        param32_1.Val = SPIReadDWord (ESC_CSR_CMD_REG);
		
    }while(param32_1.v[3] & ESC_CSR_BUSY);

    param32_1.Val = SPIReadDWord (ESC_CSR_DATA_REG);

    
    for(i=0;i<Count;i++)
         ReadBuffer[i] = param32_1.v[i];
   
    return;
}

/*******************************************************************************
* Function Name  : SPIWriteRegUsingCSR
* Description    : write data to lan9252 use CSR
* Input          : ReadBuffer:data buf 
									 Address��the reg address write to lan9252
										Count:the number write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIWriteRegUsingCSR( uint8_t *WriteBuffer, uint16_t Address, uint8_t Count)
{
    UINT32_VAL param32_1 = {0};
    UINT8 i = 0;
    UINT16_VAL wAddr;

    for(i=0;i<Count;i++)
         param32_1.v[i] = WriteBuffer[i];

    SPIWriteDWord (ESC_CSR_DATA_REG, param32_1.Val);


    wAddr.Val = Address;

    param32_1.v[0] = wAddr.byte.LB;
    param32_1.v[1] = wAddr.byte.HB;
    param32_1.v[2] = Count;
    param32_1.v[3] = ESC_WRITE_BYTE;

    SPIWriteDWord (0x304, param32_1.Val);
    do
    {
        param32_1.Val = SPIReadDWord (0x304);

    }while(param32_1.v[3] & ESC_CSR_BUSY);

    return;
}


/*******************************************************************************
* Function Name  : SPIReadPDRamRegister
* Description    : read data from lan9252 pd ram
* Input          : ReadBuffer:data buf 
									 Address��the reg address write to lan9252
										Count:the number write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIReadPDRamRegister(uint8_t *ReadBuffer, uint16_t Address, uint16_t Count)
{
    UINT32_VAL param32_1 = {0};
    UINT8 i = 0,nlength, nBytePosition;
    UINT8 nReadSpaceAvblCount;

//    /*Reset/Abort any previous commands.*/
    param32_1.Val = (unsigned long int)PRAM_RW_ABORT_MASK;                                                 

    SPIWriteDWord (PRAM_READ_CMD_REG, param32_1.Val);

    /*The host should not modify this field unless the PRAM Read Busy
    (PRAM_READ_BUSY) bit is a 0.*/
    do
    {

			param32_1.Val = SPIReadDWord (PRAM_READ_CMD_REG);

    }while((param32_1.v[3] & PRAM_RW_BUSY_8B));

    /*Write address and length in the EtherCAT Process RAM Read Address and
     * Length Register (ECAT_PRAM_RD_ADDR_LEN)*/
    param32_1.w[0] = Address;
    param32_1.w[1] = Count;

    SPIWriteDWord (PRAM_READ_ADDR_LEN_REG, param32_1.Val);


    /*Set PRAM Read Busy (PRAM_READ_BUSY) bit(-EtherCAT Process RAM Read Command Register)
     *  to start read operatrion*/

    param32_1.Val = PRAM_RW_BUSY_32B; /*TODO:replace with #defines*/

    SPIWriteDWord (PRAM_READ_CMD_REG, param32_1.Val);

		

    /*Read PRAM Read Data Available (PRAM_READ_AVAIL) bit is set*/
    do
    {			
      //  param32_1.Val
			param32_1.Val = SPIReadDWord (PRAM_READ_CMD_REG);
		

    }while(!(param32_1.v[0] & IS_PRAM_SPACE_AVBL_MASK));

    nReadSpaceAvblCount = param32_1.v[1] & PRAM_SPACE_AVBL_COUNT_MASK;

    /*Fifo registers are aliased address. In indexed it will read indexed data reg 0x04, but it will point to reg 0
     In other modes read 0x04 FIFO register since all registers are aliased*/

    /*get the UINT8 lenth for first read*/
    //Auto increment is supported in SPIO
    param32_1.Val = SPIReadDWord (PRAM_READ_FIFO_REG);
    nReadSpaceAvblCount--;
    nBytePosition = (Address & 0x03);
    nlength = (4-nBytePosition) > Count ? Count:(4-nBytePosition);
    memcpy(ReadBuffer+i ,&param32_1.v[nBytePosition],nlength);
    Count-=nlength;
    i+=nlength;

    //Lets do it in auto increment mode
    CSLOW();

    //Write Command
    SPIWriteByte(CMD_FAST_READ);

    SPISendAddr(PRAM_READ_FIFO_REG);
    
    //Dummy Byte
    SPIWriteByte(CMD_FAST_READ_DUMMY);

    while(Count)
    {
        param32_1.Val = SPIReadBurstMode();

        nlength = Count > 4 ? 4: Count;
        memcpy((ReadBuffer+i) ,&param32_1,nlength);

        i+=nlength;
        Count-=nlength;
        nReadSpaceAvblCount --;
    }

    CSHIGH();

    return;
}
/*******************************************************************************
* Function Name  : SPIWritePDRamRegister
* Description    : write data from lan9252 pd ram
* Input          : ReadBuffer:data buf 
									 Address��the reg address write to lan9252
										Count:the number write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIWritePDRamRegister(uint8_t *WriteBuffer, uint16_t Address, uint16_t Count)
{
    UINT32_VAL param32_1 = {0};
    UINT8 i = 0,nlength, nBytePosition,nWrtSpcAvlCount;

//    /*Reset or Abort any previous commands.*/
    param32_1.Val = PRAM_RW_ABORT_MASK;                                                

    SPIWriteDWord (PRAM_WRITE_CMD_REG, param32_1.Val);

    /*Make sure there is no previous write is pending
    (PRAM Write Busy) bit is a 0 */
    do
    {			
        param32_1.Val = SPIReadDWord (PRAM_WRITE_CMD_REG);

    }while((param32_1.v[3] & PRAM_RW_BUSY_8B));

    /*Write Address and Length Register (ECAT_PRAM_WR_ADDR_LEN) with the
    starting UINT8 address and length)*/
    param32_1.w[0] = Address;
    param32_1.w[1] = Count;

    SPIWriteDWord (PRAM_WRITE_ADDR_LEN_REG, param32_1.Val);

    /*write to the EtherCAT Process RAM Write Command Register (ECAT_PRAM_WR_CMD) with the  PRAM Write Busy
    (PRAM_WRITE_BUSY) bit set*/

    param32_1.Val = PRAM_RW_BUSY_32B; /*TODO:replace with #defines*/

    SPIWriteDWord (PRAM_WRITE_CMD_REG, param32_1.Val);
		

    /*Read PRAM write Data Available (PRAM_READ_AVAIL) bit is set*/
    do
    {	
       param32_1.Val = SPIReadDWord (PRAM_WRITE_CMD_REG);

    }while(!(param32_1.v[0] & IS_PRAM_SPACE_AVBL_MASK));

    /*Check write data available count*/
    nWrtSpcAvlCount = param32_1.v[1] & PRAM_SPACE_AVBL_COUNT_MASK;

    /*Write data to Write FIFO) */ 
    /*get the byte lenth for first read*/
    nBytePosition = (Address & 0x03);

    nlength = (4-nBytePosition) > Count ? Count:(4-nBytePosition);

    param32_1.Val = 0;
    memcpy(&param32_1.v[nBytePosition],WriteBuffer+i, nlength);

    SPIWriteDWord (PRAM_WRITE_FIFO_REG,param32_1.Val);

    nWrtSpcAvlCount--;
    Count-=nlength;
    i+=nlength;

    //Auto increment mode
    CSLOW();

    //Write Command
    SPIWriteByte(CMD_SERIAL_WRITE);

    SPISendAddr(PRAM_WRITE_FIFO_REG);

    while(Count)
    {
        nlength = Count > 4 ? 4: Count;
        param32_1.Val = 0;
        memcpy(&param32_1, (WriteBuffer+i), nlength);

        SPIWriteBurstMode (param32_1.Val);
        i+=nlength;
        Count-=nlength;
        nWrtSpcAvlCount--;
    }

    CSHIGH();
    return;
}


/*******************************************************************************
* Function Name  : SPIReadDRegister
* Description    : read reg from lan9252 pd ram
* Input          : ReadBuffer:data buf 
									 Address��the reg address write to lan9252
										Count:the number write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIReadDRegister(uint8_t *ReadBuffer, uint16_t Address, uint16_t Count)
{
    if (Address >= 0x1000)
    {
         SPIReadPDRamRegister(ReadBuffer, Address,Count);
    }
    else
    {
         SPIReadRegUsingCSR(ReadBuffer, Address,Count);
    }
}

/*******************************************************************************
* Function Name  : SPIWriteRegister
* Description    : write reg from lan9252 pd ram
* Input          : ReadBuffer:data buf 
									 Address��the reg address write to lan9252
										Count:the number write to lan9252
* Output         : none
* Return         : none: 
* Attention		 : None
*******************************************************************************/
void SPIWriteRegister( uint8_t *WriteBuffer, uint16_t Address, uint16_t Count)
{
   
   if (Address >= 0x1000)
   {
		SPIWritePDRamRegister(WriteBuffer, Address,Count);
   }
   else
   {
		SPIWriteRegUsingCSR(WriteBuffer, Address,Count);
   }
    
}



/*
 * ��    ����SPIReadDWord_test
 * ��    �ܣ���һ��32����
 * ��ڲ�����Address---��������ַ
 * ���ڲ���������������
 */ 
unsigned long SPIReadDWord_test(unsigned short Address)
{
	UINT32_VAL dwResult;
	UINT16_VAL wAddr;

	wAddr.Val  = Address;
	//Assert CS line
	CSLOW();
	//Write Command
	SPIWriteByte(CMD_FAST_READ);
    //Write Address
	SPIWriteByte(wAddr.byte.HB);
	SPIWriteByte(wAddr.byte.LB);
			
	//Dummy Byte
	SPIWriteByte(CMD_FAST_READ_DUMMY);
    //Read Bytes
    dwResult.byte.LB = SPIReadByte();
    dwResult.byte.HB = SPIReadByte();
    dwResult.byte.UB = SPIReadByte();
    dwResult.byte.MB = SPIReadByte();
	
    //De-Assert CS line
		CSHIGH();
		//printf("temp =  %x %x %x %x\n", dwResult.byte.LB,dwResult.byte.HB,dwResult.byte.UB, dwResult.byte.MB);
		return dwResult.Val;
}
/*
 * ��    ����LAN9252_ReadID
 * ��    �ܣ�CSR��������LAN9252��оƬID
 * ��ڲ�������
 * ���ڲ�����������оƬID
 */
unsigned long LAN9252_ReadID(void)
{
	UINT8 Temp[10] = {0,0,0,0,0,0,0,0,0,0};	  
	SPIReadRegUsingCSR(Temp, 0x0e02, 2);	    
	return (Temp[0] | ((u32)Temp[1] << 8) |
		((u32)Temp[2] << 16) | ((u32)Temp[3] << 24));	
	
}   
/*
 * ��    ����mem_test
 * ��    �ܣ�����PDI�ӿ�
 * ��ڲ�������
 * ���ڲ�������
 */
void mem_test(void)
{
	unsigned long i;	
	unsigned long temp = 0;

	//------------------------------------�������ַ64H=87654321
	temp = SPIReadDWord_test(0x64);
	//printf("temp =  %x ", (int)temp);
	temp = SPIReadDWord_test(0x64);
	
	printf("temp =  %x ", (int)temp);
    
	if(temp != 0x87654321)
	{
		printf("stop();");
		while(1);
	}
	
	//------------------------------------�������ַ0e02H=9252
	temp = LAN9252_ReadID();
	printf(temp == 0x9252 ? "   ID = %x\n\r":"\n\r  test:FAILED! ID = %x\n\r", (int)temp);
	
	if(temp != 0x9252)
	{
//		printf("stop();");
		while(1);
	}
}


   
/******************* (C) COPYRIGHT 2020-2030 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/

