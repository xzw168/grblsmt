
#include "stm32f10x.h"

#define USART_MAIN USART2
#define USART_CHILD USART1

#define USART_MAIN_RX_BUF 256
u8 uart_mainRxBuf[USART_MAIN_RX_BUF];

#define USART_CHILD_RX_BUF 256
u8 uart_childRxBuf[USART_CHILD_RX_BUF];

void uartX_conf(USART_TypeDef *USARTx, u32 BaudRate)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = BaudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USARTx->CR1 |= (USART_CR1_RE | USART_CR1_TE);
    USART_Init(USARTx, &USART_InitStructure);
    //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    USART_Cmd(USARTx, ENABLE);
	
	USART_DMACmd(USARTx, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
}

static void _DmaConfig(void)
{

    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
	
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    //rx

    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址寄存器是否增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;          //存储器地址寄存器是否被递增
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;      //外设作为源端
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //优先级

    DMA_InitStructure.DMA_BufferSize = USART_MAIN_RX_BUF;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart_mainRxBuf;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART_MAIN->DR;
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);
	//DMA_RemapConfig(DMA1, DMA1_CH5_USART2_RX);
    DMA_Cmd(DMA1_Channel6, ENABLE);

    DMA_InitStructure.DMA_BufferSize = USART_CHILD_RX_BUF;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart_childRxBuf;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART_CHILD->DR;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel5, ENABLE);

    //tx
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; //外设作为目的端
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART_MAIN->DR;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART_CHILD->DR;
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);

    //DMA_ITConfig(uart_Tx_Channel, DMA_IT_TC, ENABLE);
	
	

	
}

void uartX_Init(void)
{
    uartX_conf(USART_MAIN, 1700000);//0.84%  1.713   2300000
    uartX_conf(USART_CHILD, 500000);
    _DmaConfig();
	USART_SendData(USART2, 0x41);
}
