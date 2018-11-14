/*
  coolant_control.c - 冷却液控制方法 coolant control methods
  冷却液控制功能
  IO初始化和控制
*/

#include "grbl.h"


void coolant_init()
{
#ifdef STM32F103C8
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_COOLANT_FLOOD_PORT, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = 1 << COOLANT_FLOOD_BIT;
	GPIO_Init(COOLANT_FLOOD_PORT, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_COOLANT_MIST_PORT, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = 1 << COOLANT_MIST_BIT;
	GPIO_Init(COOLANT_MIST_PORT, &GPIO_InitStructure);
#endif
  coolant_stop();
}


// 返回当前的冷却液输出状态。覆盖可能会改变它从编程状态。Returns current coolant output state. Overrides may alter it from programmed state.
uint8_t coolant_get_state()
{
  uint8_t cl_state = COOLANT_STATE_DISABLE;
#if defined(STM32F103C8)
  #ifdef INVERT_COOLANT_FLOOD_PIN
    if (bit_isfalse(
		GPIO_ReadOutputData(COOLANT_FLOOD_PORT)

		,(1 << COOLANT_FLOOD_BIT))) {
  #else
    if (bit_istrue(
		GPIO_ReadOutputData(COOLANT_FLOOD_PORT)
		,(1 << COOLANT_FLOOD_BIT))) {
  #endif
    cl_state |= COOLANT_STATE_FLOOD;
  }
  #ifdef ENABLE_M7
    #ifdef INVERT_COOLANT_MIST_PIN
      if (bit_isfalse(
		  GPIO_ReadOutputData(COOLANT_MIST_PORT)
		  ,(1 << COOLANT_MIST_BIT))) {
    #else
      if (bit_istrue(
		  GPIO_ReadOutputData(COOLANT_MIST_PORT)
		  ,(1 << COOLANT_MIST_BIT))) {
    #endif
      cl_state |= COOLANT_STATE_MIST;
    }
  #endif
#endif
  return(cl_state);
}


//由coolant_init（），coolant_set_state（）和mc_reset（）直接调用，可以在. Directly called by coolant_init(), coolant_set_state(), and mc_reset(), which can be at
//中断级别 没有设置报告标志，但只能由不需要它的例程调用。 an interrupt-level. No report flag set, but only called by routines that don't need it.
void coolant_stop()
{
#if defined(STM32F103C8)
  #ifdef INVERT_COOLANT_FLOOD_PIN
	GPIO_SetBits(COOLANT_FLOOD_PORT,1 << COOLANT_FLOOD_BIT);
  #else
	GPIO_ResetBits(COOLANT_FLOOD_PORT,1 << COOLANT_FLOOD_BIT);
  #endif
  #ifdef ENABLE_M7
    #ifdef INVERT_COOLANT_MIST_PIN
	GPIO_SetBits(COOLANT_MIST_PORT, 1 << COOLANT_MIST_BIT);
    #else
	GPIO_ResetBits(COOLANT_MIST_PORT, 1 << COOLANT_MIST_BIT);
    #endif
  #endif
#endif
}


//仅限主程序。立即设置淹没冷却液运行状态，同时雾化冷却液，
//如果启用。还设置一个标志来报告冷却剂状态的更新。
//通过冷却液切换超驰，停车恢复，停车收回，睡眠模式，g代码调用
//解析器程序结束，g-code parser cooling_sync（）。
void coolant_set_state(uint8_t mode)
{
  if (sys.abort) { return; } // Block during abort.  
  
  if (mode == COOLANT_DISABLE) {
  
    coolant_stop(); 
  
  } else {
  
#if defined(STM32F103C8)
	  if (mode & COOLANT_FLOOD_ENABLE) {
      #ifdef INVERT_COOLANT_FLOOD_PIN
		GPIO_ResetBits(COOLANT_FLOOD_PORT,1 << COOLANT_FLOOD_BIT);
      #else
		GPIO_SetBits(COOLANT_FLOOD_PORT,1 << COOLANT_FLOOD_BIT);
      #endif
    }
  
    #ifdef ENABLE_M7
      if (mode & COOLANT_MIST_ENABLE) {
        #ifdef INVERT_COOLANT_MIST_PIN
			GPIO_ResetBits(COOLANT_MIST_PORT, 1 << COOLANT_MIST_BIT);
        #else
		  GPIO_SetBits(COOLANT_MIST_PORT, 1 << COOLANT_MIST_BIT);
        #endif
      }
    #endif
#endif  
  }
  sys.report_ovr_counter = 0; //设置立即报告更改. Set to report change immediately
}


//用于设置冷却液状态的G代码解析器入口点。强制规划器缓冲区同步和保留
//如果中止或检查模式处于活动状态。
void coolant_sync(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); //在程序中指定时，确保冷却液打开。 Ensure coolant turns on when specified in program.
  coolant_set_state(mode);
}
