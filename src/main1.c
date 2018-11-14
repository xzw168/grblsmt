

#include "stm32f10x.h"
#include "uartX.h"

int main(void)
{
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  uartX_Init();
  while (1)
  {
  }
}
