
/*
  probe.c - �й�̽�ⷽ���Ĵ���
*/

#include "grbl.h"


// ȡ�����û����ú�̽��ѭ��ģʽ����ת̽������״̬��
uint8_t probe_invert_mask;


// ̽�����ų�ʼ������
void probe_init()
{
#ifdef STM32F103C8
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_PROBE_PORT, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef DISABLE_PROBE_PIN_PULL_UP
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
#else
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
#endif
	GPIO_InitStructure.GPIO_Pin = PROBE_MASK;
	GPIO_Init(PROBE_PORT, &GPIO_InitStructure);
#endif
  probe_configure_invert_mask(false); // Initialize invert mask.
}


//��probe_init������mc_probe�������̵��á���̽�����ŷ�ת��������Ϊ
//����������/�����͵Ĳ��������ʵ��������߼�
//�ͳ��򹤼�/�뿪������̽��ѭ��ģʽ��
void probe_configure_invert_mask(uint8_t is_probe_away)
{
  probe_invert_mask = 0; // Initialize as zero.
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { probe_invert_mask ^= PROBE_MASK; }
  if (is_probe_away) { probe_invert_mask ^= PROBE_MASK; }
}


//����̽������״̬������=�档��gcode��������̽��״̬���������á�
uint8_t probe_get_state() 
{ 
#ifdef STM32F103C8
	return ((GPIO_ReadInputData(PROBE_PORT) & PROBE_MASK) ^ probe_invert_mask) != 0;
#endif
}


//���̽��״̬���ڼ��ʱ��¼ϵͳλ�á��е�
//ÿ��ISR tick�Ĳ���ISR��
//ע�⣺������ܱ����Ƿǳ���Ч�ģ��Է�ֹ����ISR��
void probe_state_monitor()
{
  if (probe_get_state()) {
    sys_probe_state = PROBE_OFF;
    memcpy(sys_probe_position, sys_position, sizeof(sys_position));
    bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
  }
}
