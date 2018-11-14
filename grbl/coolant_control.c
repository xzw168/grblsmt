/*
  coolant_control.c - ��ȴҺ���Ʒ��� coolant control methods
  ��ȴҺ���ƹ���
  IO��ʼ���Ϳ���
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


// ���ص�ǰ����ȴҺ���״̬�����ǿ��ܻ�ı����ӱ��״̬��Returns current coolant output state. Overrides may alter it from programmed state.
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


//��coolant_init������coolant_set_state������mc_reset����ֱ�ӵ��ã�������. Directly called by coolant_init(), coolant_set_state(), and mc_reset(), which can be at
//�жϼ��� û�����ñ����־����ֻ���ɲ���Ҫ�������̵��á� an interrupt-level. No report flag set, but only called by routines that don't need it.
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


//��������������������û��ȴҺ����״̬��ͬʱ����ȴҺ��
//������á�������һ����־��������ȴ��״̬�ĸ��¡�
//ͨ����ȴҺ�л����ۣ�ͣ���ָ���ͣ���ջأ�˯��ģʽ��g�������
//���������������g-code parser cooling_sync������
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
  sys.report_ovr_counter = 0; //���������������. Set to report change immediately
}


//����������ȴҺ״̬��G�����������ڵ㡣ǿ�ƹ滮��������ͬ���ͱ���
//�����ֹ����ģʽ���ڻ״̬��
void coolant_sync(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); //�ڳ�����ָ��ʱ��ȷ����ȴҺ�򿪡� Ensure coolant turns on when specified in program.
  coolant_set_state(mode);
}
