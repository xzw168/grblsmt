
/*
  probe.c - 有关探测方法的代码
*/

#include "grbl.h"


// 取决于用户设置和探测循环模式，反转探针引脚状态。
uint8_t probe_invert_mask;


// 探针引脚初始化程序。
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


//由probe_init（）和mc_probe（）例程调用。将探针引脚反转遮罩设置为
//根据正常高/正常低的操作设置适当的引脚逻辑
//和朝向工件/离开工件的探测循环模式。
void probe_configure_invert_mask(uint8_t is_probe_away)
{
  probe_invert_mask = 0; // Initialize as zero.
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { probe_invert_mask ^= PROBE_MASK; }
  if (is_probe_away) { probe_invert_mask ^= PROBE_MASK; }
}


//返回探针引脚状态。触发=真。由gcode解析器和探针状态监视器调用。
uint8_t probe_get_state() 
{ 
#ifdef STM32F103C8
	return ((GPIO_ReadInputData(PROBE_PORT) & PROBE_MASK) ^ probe_invert_mask) != 0;
#endif
}


//监测探针状态并在检测时记录系统位置。叫的
//每个ISR tick的步进ISR。
//注意：这个功能必须是非常有效的，以防止步进ISR。
void probe_state_monitor()
{
  if (probe_get_state()) {
    sys_probe_state = PROBE_OFF;
    memcpy(sys_probe_position, sys_position, sizeof(sys_position));
    bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
  }
}
