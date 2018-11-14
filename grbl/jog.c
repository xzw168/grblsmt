/*
jog.h - 慢跑方法
手动指令执行
通过G代码解析器接收手动指令
*/

#include "grbl.h"


//设置从g代码解析器接收到的有效点动运动，检查软限制，并执行点动。 Sets up valid jog motion received from g-code parser, checks for soft-limits, and executes the jog.
uint8_t jog_execute(plan_line_data_t *pl_data, parser_block_t *gc_block)
{
	  //初始化用于慢跑运动的计划器数据结构。
	  //注意：主轴和冷却液在点动过程中允许超越功能。
  pl_data->feed_rate = gc_block->values.f;
  pl_data->condition |= PL_COND_FLAG_NO_FEED_OVERRIDE;
#ifdef USE_LINE_NUMBERS
  pl_data->line_number = gc_block->values.n;
#endif

  if (bit_istrue(settings.flags, BITFLAG_SOFT_LIMIT_ENABLE)) {
    if (system_check_travel_limits(gc_block->values.xyz)) { return(STATUS_TRAVEL_EXCEEDED); }
  }

  // 有效的点动命令 计划，设置状态和执行。
  mc_line(gc_block->values.xyz, pl_data);
  if (sys.state == STATE_IDLE) {
    if (plan_get_current_block() != NULL) { // 检查是否有块要执行.
      sys.state = STATE_JOG;
      st_prep_buffer();
      st_wake_up();  // 注意：手动启动。不需要状态机.
    }
  }

  return(STATUS_OK);
}
