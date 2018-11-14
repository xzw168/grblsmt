/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"
// 声明系统全局变量结构
system_t sys;
int32_t sys_position[N_AXIS];      //实时机器（aka home）位置矢量步骤 .Real-time machine (aka home) position vector in steps.
int32_t sys_probe_position[N_AXIS]; //机床坐标和步骤中的最后探头位置 . Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;   //探测状态值 用于协调探测循环与步进ISR . Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;   //全局实时执行程序位标志变量用于状态管理。请参阅EXEC位掩码。 Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   //全局实时执行器位标志变量，用于设置各种报警。 Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override; //全局实时执行器位标志变量，用于基于运动的覆盖。 Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; //主轴/冷却液覆盖的全局实时执行程序位标志变量。 Global realtime executor bitflag variable for spindle/coolant overrides.
#ifdef DEBUG
volatile uint8_t sys_rt_exec_debug;
#endif

#if defined (STM32F103C8)
//#include "usb_lib.h"
#ifdef USEUSB
#include "usb_desc.h"
#endif
//#include "hw_config.h"
#ifdef USEUSB
#include "usb_pwr.h"
#endif
#include "stm32eeprom.h"
#ifndef USEUSB
#include "stm32f10x_usart.h"
void USART1_Configuration(u32 BaudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);                 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = BaudRate;	  
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	 
	USART_InitStructure.USART_Parity = USART_Parity_No;	 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART1->CR1 |= (USART_CR1_RE | USART_CR1_TE);
	USART_Init(USART1, &USART_InitStructure);
	//	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
}
#endif

#endif



int main1(void)
{
	while(1);
#if defined (STM32F103C8)
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
#ifdef LEDBLINK
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif
	//Set_System();
#ifndef USEUSB
	USART1_Configuration(115200);
#else
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
#endif

#ifndef NOEEPROMSUPPORT
	FLASH_Unlock();
	eeprom_init();
#endif
	SysTick->CTRL &= 0xfffffffb;
#endif
  // 启动时初始化系统。
  serial_init();   // 设置串口波特率和中断

  settings_init(); // 从EEPROM加载Grbl设置
  stepper_init();  // 配置步进引脚和中断计时器
  system_init();   // 配置引脚引脚和引脚改变中断

  memset(sys_position,0,sizeof(sys_position)); //清除机器位置。
  // Initialize system state.
  #ifdef FORCE_INITIALIZATION_ALARM
    // Force Grbl into an ALARM state upon a power-cycle or hard reset.
    sys.state = STATE_ALARM;
  #else
    sys.state = STATE_IDLE;
  #endif
  
    //如果启用了回原点，则检查通电并设置系统警报强制回原点周期
    //通过设置Grbl的警报状态。警报锁定所有的g代码命令，包括
    //启动脚本，但允许访问设置和内部命令。只有一个归巢
    //循环'$ H'或关闭警报锁'$ X'将禁用警报。
    //注意：启动脚本将在成功完成归位循环后运行，但是
    //禁用警报锁定后 防止运动启动块崩溃
    //不可控制的东西 很坏。
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif

    //上电或系统中止时Grbl初始化循环。对于后者，所有的过程
    //将返回到这个循环来干净地重新初始化。
  for(;;) {

	  //重置系统变量
    uint8_t prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t)); // 清除系统结构变量.
    sys.state = prior_state;
    sys.f_override = DEFAULT_FEED_OVERRIDE;  // 设置为100%
    sys.r_override = DEFAULT_RAPID_OVERRIDE; // 设置为100%
    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // 设置为100%
		memset(sys_probe_position,0,sizeof(sys_probe_position)); //清除探针位置. Clear probe position.
    sys_probe_state = 0;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys_rt_exec_motion_override = 0;
    sys_rt_exec_accessory_override = 0;

    // 重置Grbl主系统。
    serial_reset_read_buffer(); // 清除串行读取缓冲区
    gc_init(); // 将g代码解析器设置为默认状态.Set g-code parser to default state
    spindle_init();
    coolant_init();
    limits_init();
    probe_init();
    plan_reset(); // 清除块缓冲区和规划器变量.Clear block buffer and planner variables
    st_reset(); // 清除步进子系统变量。Clear stepper subsystem variables.

    // 同步清除当前系统位置的gcode和planner位置。Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();

    // 打印欢迎信息 表示在上电或复位时发生初始化。
    report_init_message();

    // 启动Grbl主循环 处理程序输入并执行它们。
    protocol_main_loop();

  }
  //return 0;   /* 从未达到 */
}
#if defined (STM32F103C8)
void _delay_ms(uint32_t x)
{
	u32 temp;
	SysTick->LOAD = (u32)72000000 / 8000;                     //加载时间 Loading time
	SysTick->VAL = 0x00;                                            //清空计数器 Empty the counter
	SysTick->CTRL = 0x01;                                           //从底部开始 Start from bottom
	do
	{
		temp = SysTick->CTRL;
	} while (temp & 0x01 && !(temp&(1 << 16)));                             //等待时间到来  Wait time arrive
	SysTick->CTRL = 0x00;                                            // 关闭计数器
	SysTick->VAL = 0X00;                                            // 清空计数器
}
void LedBlink(void)
{
	static BitAction nOnFlag = Bit_SET;
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, nOnFlag);
	nOnFlag = (nOnFlag == Bit_SET) ? Bit_RESET : Bit_SET;
}
#endif
