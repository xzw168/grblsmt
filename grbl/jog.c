/*
jog.h - ���ܷ���
�ֶ�ָ��ִ��
ͨ��G��������������ֶ�ָ��
*/

#include "grbl.h"


//���ô�g������������յ�����Ч�㶯�˶�����������ƣ���ִ�е㶯�� Sets up valid jog motion received from g-code parser, checks for soft-limits, and executes the jog.
uint8_t jog_execute(plan_line_data_t *pl_data, parser_block_t *gc_block)
{
	  //��ʼ�����������˶��ļƻ������ݽṹ��
	  //ע�⣺�������ȴҺ�ڵ㶯����������Խ���ܡ�
  pl_data->feed_rate = gc_block->values.f;
  pl_data->condition |= PL_COND_FLAG_NO_FEED_OVERRIDE;
#ifdef USE_LINE_NUMBERS
  pl_data->line_number = gc_block->values.n;
#endif

  if (bit_istrue(settings.flags, BITFLAG_SOFT_LIMIT_ENABLE)) {
    if (system_check_travel_limits(gc_block->values.xyz)) { return(STATUS_TRAVEL_EXCEEDED); }
  }

  // ��Ч�ĵ㶯���� �ƻ�������״̬��ִ�С�
  mc_line(gc_block->values.xyz, pl_data);
  if (sys.state == STATE_IDLE) {
    if (plan_get_current_block() != NULL) { // ����Ƿ��п�Ҫִ��.
      sys.state = STATE_JOG;
      st_prep_buffer();
      st_wake_up();  // ע�⣺�ֶ�����������Ҫ״̬��.
    }
  }

  return(STATUS_OK);
}
