
/*
  system.c - 处理系统级命令和实时进程
  "$"系统命令的解析
*/

#include "grbl.h"


void system_init()
{

#ifdef STM32F103C8
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_CONTROL_PORT | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef DISABLE_CONTROL_PIN_PULL_UP
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
#else
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
#endif
  GPIO_InitStructure.GPIO_Pin = CONTROL_MASK;
  GPIO_Init(CONTROL_PORT, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_CONTROL_PORT, CONTROL_RESET_BIT);
  GPIO_EXTILineConfig(GPIO_CONTROL_PORT, CONTROL_FEED_HOLD_BIT);
  GPIO_EXTILineConfig(GPIO_CONTROL_PORT, CONTROL_CYCLE_START_BIT);
  GPIO_EXTILineConfig(GPIO_CONTROL_PORT, CONTROL_SAFETY_DOOR_BIT);

  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = CONTROL_MASK;    //
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //Interrupt mode, optional values for the interrupt EXTI_Mode_Interrupt and event EXTI_Mode_Event.
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //Trigger mode, can be a falling edge trigger EXTI_Trigger_Falling, the rising edge triggered EXTI_Trigger_Rising, or any level (rising edge and falling edge trigger EXTI_Trigger_Rising_Falling)
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; //Enable keypad external interrupt channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; //Priority 2,
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02; //Sub priority 2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //Enable external interrupt channel
  NVIC_Init(&NVIC_InitStructure);
#endif
}


//将控制引脚状态作为uint8位域返回。每一位表示输入引脚状态，其中
//触发为1，未触发为0.应用反转掩码。比特菲尔德组织是
//由头文件中的CONTROL_PIN_INDEX定义。
uint8_t system_control_get_state()
{
  uint8_t control_state = 0;

#ifdef STM32F103C8
  uint16_t pin= GPIO_ReadInputData(CONTROL_PIN_PORT);
#endif
  #ifdef INVERT_CONTROL_PIN_MASK
    pin ^= INVERT_CONTROL_PIN_MASK;
  #endif
  if (pin) {
    #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
      if (bit_isfalse(pin,(1<<CONTROL_SAFETY_DOOR_BIT))) { control_state |= CONTROL_PIN_INDEX_SAFETY_DOOR; }
    #endif
    if (bit_isfalse(pin,(1<<CONTROL_RESET_BIT))) { control_state |= CONTROL_PIN_INDEX_RESET; }
    if (bit_isfalse(pin,(1<<CONTROL_FEED_HOLD_BIT))) { control_state |= CONTROL_PIN_INDEX_FEED_HOLD; }
    if (bit_isfalse(pin,(1<<CONTROL_CYCLE_START_BIT))) { control_state |= CONTROL_PIN_INDEX_CYCLE_START; }
  }
  return(control_state);
}


//引脚变化中断引脚命令，即循环开始，进给保持和复位。集
//只有realtime命令执行变量才能让主程序执行这些
//准备好了 这与拾取时基于字符的实时命令完全相同
//直接来自串行数据流。

#if defined (STM32F103C8)
void EXTI9_5_IRQHandler(void)
{
    EXTI_ClearITPendingBit((1 << CONTROL_RESET_BIT) | (1 << CONTROL_FEED_HOLD_BIT) | (1 << CONTROL_CYCLE_START_BIT) | (1 << CONTROL_SAFETY_DOOR_BIT));
	uint8_t pin = system_control_get_state();
	if (pin) 
	{ 
		if (bit_istrue(pin,CONTROL_PIN_INDEX_RESET)) 
		{
			mc_reset();
		}
		else if (bit_istrue(pin, CONTROL_PIN_INDEX_CYCLE_START))
		{
			bit_true(sys_rt_exec_state, EXEC_CYCLE_START);
		}
#ifndef ENABLE_SAFETY_DOOR_INPUT_PIN
		else if (bit_istrue(pin, CONTROL_PIN_INDEX_FEED_HOLD))
		{
			bit_true(sys_rt_exec_state, EXEC_FEED_HOLD);
		}
#else
		else if (bit_istrue(pin, CONTROL_PIN_INDEX_SAFETY_DOOR))
		{
			bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
		}
#endif
		NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}
}
#endif

//根据引脚状态，安全门关闭（T）或关闭（F）返回。
uint8_t system_check_safety_door_ajar()
{
  #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    return(system_control_get_state() & CONTROL_PIN_INDEX_SAFETY_DOOR);
  #else
    return(false); // 输入引脚没有启用，所以只是返回它关闭。
  #endif
}


// 执行用户启动脚本（如果存储）。
void system_execute_startup(char *line)
{
  uint8_t n;
  for (n=0; n < N_STARTUP_LINE; n++) {
    if (!(settings_read_startup_line(n, line))) {
      line[0] = 0;
      report_execute_startup_message(line,STATUS_SETTING_READ_FAIL);
    } else {
      if (line[0] != 0) {
        uint8_t status_code = gc_execute_line(line);
        report_execute_startup_message(line,status_code);
      }
    }
  }
}


//指定并执行protocol_process的一行格式化输入。主要是
//传入的流g代码块，这也执行Grbl内部命令，如
//设置，启动归位循环以及切换开关状态。这不同于
//当Grbl准备执行时，易受到实时命令模块的影响
//一个循环中的下一行，所以对于像块删除这样的开关，开关只起作用
//之后处理的行，在循环中不一定是实时的，
//因为已经存储在缓冲区中的运动。但是，这个“滞后”不应该
//是一个问题，因为这些命令在一个周期中通常不被使用。
uint8_t system_execute_line(char *line)
{
  uint8_t char_counter = 1;
  uint8_t helper_var = 0; // 助手变量
  float parameter, value;
  switch( line[char_counter] ) {
    case 0 : report_grbl_help(); break;
    case 'J' : // 慢跑
      // 仅在IDLE或JOG状态下执行。
      if (sys.state != STATE_IDLE && sys.state != STATE_JOG) { return(STATUS_IDLE_ERROR); }
      if(line[2] != '=') { return(STATUS_INVALID_STATEMENT); }
      return(gc_execute_line(line)); // NOTE: $J= 在g代码解析器中被忽略，并用于检测点动运动。
      //break;
    case '$': case 'G': case 'C': case 'X':
      if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      switch( line[1] ) {
        case '$' : // 打印Grbl设置
          if ( sys.state & (STATE_CYCLE | STATE_HOLD) ) { return(STATUS_IDLE_ERROR); } // 在循环中阻塞 打印时间太长。
          else { report_grbl_settings(); }
          break;
        case 'G' : // 打印G-码解析器的状态
          // TODO: 将其移至GUI的实时命令以在挂起状态期间请求此数据。
          report_gcode_modes();
          break;
        case 'C' : // 设置检查g代码模式 [空闲/检查]
          // 切换时执行重置。检查g代码模式应该只适用于Grbl
          // 无论警报是否锁定，都处于闲置状态。这主要是为了保持东西
          // 简单一致。
          if ( sys.state == STATE_CHECK_MODE ) {
            mc_reset();
            report_feedback_message(MESSAGE_DISABLED);
          } else {
            if (sys.state) { return(STATUS_IDLE_ERROR); } // 不需要报警模式。
            sys.state = STATE_CHECK_MODE;
            report_feedback_message(MESSAGE_ENABLED);
          }
          break;
        case 'X' : // 禁用警报锁定[ALARM]
          if (sys.state == STATE_ALARM) {
            // 如果安全门是半开的
            if (system_check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); }
            report_feedback_message(MESSAGE_ALARM_UNLOCK);
            sys.state = STATE_IDLE;
            // 不要运行启动脚本。 防止启动时存储的移动造成事故。
          } // 否则，没有效果。
          break;
      }
      break;
    default :
      // 阻止任何要求状态为空闲/报警的系统命令。（即EEPROM，归位）
      if ( !(sys.state == STATE_IDLE || sys.state == STATE_ALARM) ) { return(STATUS_IDLE_ERROR); }
      switch( line[1] ) {
        case '#' : // 打印Grbl NGC参数
          if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); }
          else { report_ngc_parameters(); }
          break;
        case 'H' : // 执行归位循环[IDLE / ALARM]
          if (bit_isfalse(settings.flags,BITFLAG_HOMING_ENABLE)) {return(STATUS_SETTING_DISABLED); }
          if (system_check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); } // 块，如果安全门开启的情况。
          sys.state = STATE_HOMING; // 设置系统状态变量
          if (line[2] == 0) {
            mc_homing_cycle(HOMING_CYCLE_ALL);
          #ifdef HOMING_SINGLE_AXIS_COMMANDS
            } else if (line[3] == 0) {
              switch (line[2]) {
                case 'X': mc_homing_cycle(HOMING_CYCLE_X); break;
                case 'Y': mc_homing_cycle(HOMING_CYCLE_Y); break;
                case 'Z': mc_homing_cycle(HOMING_CYCLE_Z); break;
                default: return(STATUS_INVALID_STATEMENT);
              }
          #endif
          } else { return(STATUS_INVALID_STATEMENT); }
          if (!sys.abort) {  // 成功归位后执行启动脚本。
            sys.state = STATE_IDLE; // 完成时设置为空闲。
            st_go_idle(); // 在返回之前将步进器设置为空闲状态。
            if (line[2] == 0) { system_execute_startup(line); }
          }
          break;
        case 'S' : // 把Grbl睡觉 [IDLE/ALARM]
          if ((line[2] != 'L') || (line[3] != 'P') || (line[4] != 0)) { return(STATUS_INVALID_STATEMENT); }
          system_set_exec_state_flag(EXEC_SLEEP); // 设置立即执行睡眠模式
          break;
        case 'I' : // 打印或存储构建信息。[IDLE / ALARM]
          if ( line[++char_counter] == 0 ) {
            settings_read_build_info(line);
            report_build_info(line);
          #ifdef ENABLE_BUILD_INFO_WRITE_COMMAND
            } else { // 存储启动行[IDLE / ALARM]
              if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
              helper_var = char_counter; // 将辅助变量设置为计数器，以启动用户信息行。
              do {
                line[char_counter-helper_var] = line[char_counter];
              } while (line[char_counter++] != 0);
              settings_store_build_info(line);
          #endif
          }
          break;
        case 'R' : // 恢复默认[IDLE / ALARM]
          if ((line[2] != 'S') || (line[3] != 'T') || (line[4] != '=') || (line[6] != 0)) { return(STATUS_INVALID_STATEMENT); }
          switch (line[5]) {
            #ifdef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS
              case '$': settings_restore(SETTINGS_RESTORE_DEFAULTS); break;
            #endif
            #ifdef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS
              case '#': settings_restore(SETTINGS_RESTORE_PARAMETERS); break;
            #endif
            #ifdef ENABLE_RESTORE_EEPROM_WIPE_ALL
              case '*': settings_restore(SETTINGS_RESTORE_ALL); break;
            #endif
            default: return(STATUS_INVALID_STATEMENT);
          }
          report_feedback_message(MESSAGE_RESTORE_DEFAULTS);
          mc_reset(); // 强制重置以确保设置正确初始化。
          break;
        case 'N' : // 启动线。[IDLE / ALARM]
          if ( line[++char_counter] == 0 ) { // 打印启动行
            for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++) {
              if (!(settings_read_startup_line(helper_var, line))) {
                report_status_message(STATUS_SETTING_READ_FAIL);
              } else {
                report_startup_line(helper_var,line);
              }
            }
            break;
          } else { // 存储启动行[仅IDLE]防止在ALARM期间移动。
            if (sys.state != STATE_IDLE) { return(STATUS_IDLE_ERROR); } // 仅在空闲时存储。
            helper_var = true;  // 设置helper_var标记存储方法。
            // 没有休息 继续默认：读取剩余的命令字符。
          }
        default :  // 存储设置方法[IDLE / ALARM]
          if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
          if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
          if (helper_var) { // 存储启动线
            // 通过移动所有字符准备发送gcode块到gcode解析器
            helper_var = char_counter; // 设置辅助变量作为计数器开始gcode块
            do {
              line[char_counter-helper_var] = line[char_counter];
            } while (line[char_counter++] != 0);
            // 执行gcode块以确保块有效。
            helper_var = gc_execute_line(line); // 将helper_var设置为返回的状态码。
            if (helper_var) { return(helper_var); }
            else {
              helper_var = truncf(parameter); // 将helper_var设置为参数的int值
              settings_store_startup_line(helper_var,line);
            }
          } else { // 存储全局设置。
            if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
            if((line[char_counter] != 0) || (parameter > 255)) { return(STATUS_INVALID_STATEMENT); }
            return(settings_store_global_setting((uint8_t)parameter, value));
          }
      }
  }
  return(STATUS_OK); // 如果'$'命令到达这里，那么一切正常。
}



void system_flag_wco_change()
{
  #ifdef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE
    protocol_buffer_synchronize();
  #endif
  sys.report_wco_counter = 0;
}


//返回轴“idx”的机床位置。必须发送一个“步骤”数组。
//注意：如果电机步进和机床位置不在同一坐标系中，此功能
//    作为计算转换的中心位置。
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx)
{
  float pos;
  #ifdef COREXY
    if (idx==X_AXIS) {
      pos = (float)system_convert_corexy_to_x_axis_steps(steps) / settings.steps_per_mm[idx];
    } else if (idx==Y_AXIS) {
      pos = (float)system_convert_corexy_to_y_axis_steps(steps) / settings.steps_per_mm[idx];
    } else {
      pos = steps[idx]/settings.steps_per_mm[idx];
    }
  #else
    pos = steps[idx]/settings.steps_per_mm[idx];
  #endif
  return(pos);
}


void system_convert_array_steps_to_mpos(float *position, int32_t *steps)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    position[idx] = system_convert_axis_steps_to_mpos(steps, idx);
  }
  return;
}


//仅CoreXY计算。根据CoreXY电机步进返回x或y轴“步”。
#ifdef COREXY
  int32_t system_convert_corexy_to_x_axis_steps(int32_t *steps)
  {
    return( (steps[A_MOTOR] + steps[B_MOTOR])/2 );
  }
  int32_t system_convert_corexy_to_y_axis_steps(int32_t *steps)
  {
    return( (steps[A_MOTOR] - steps[B_MOTOR])/2 );
  }
#endif


  //检查并报告目标数组是否超出机器行程限制。
uint8_t system_check_travel_limits(float *target)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef HOMING_FORCE_SET_ORIGIN
      //当启用归位强制设置原点时，软限制检查需要考虑方向性。
      //注意：max_travel存储为负数
      if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
        if (target[idx] < 0 || target[idx] > -settings.max_travel[idx]) { return(true); }
      } else {
        if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { return(true); }
      }
    #else
      // 注意：max_travel存储为负数
      if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { return(true); }
    #endif
  }
  return(false);
}



// 用于设置和清除Grbl的实时执行标志的特殊处理程序。
void system_set_exec_state_flag(uint8_t mask) {

  __disable_irq();
  sys_rt_exec_state |= (mask);
  __enable_irq();
}

void system_clear_exec_state_flag(uint8_t mask) {

  __disable_irq();
  sys_rt_exec_state &= ~(mask);
  __enable_irq();

}

void system_set_exec_alarm(uint8_t code) {


  __disable_irq();
  sys_rt_exec_alarm |= (code);
  __enable_irq();

}

void system_clear_exec_alarm() {

  __disable_irq();
  sys_rt_exec_alarm = 0;
  __enable_irq();

}

void system_set_exec_motion_override_flag(uint8_t mask) {

  __disable_irq();
  sys_rt_exec_motion_override |= (mask);
  __enable_irq();

}

void system_set_exec_accessory_override_flag(uint8_t mask) {

  __disable_irq();
  sys_rt_exec_accessory_override |= (mask);
  __enable_irq();

}

void system_clear_exec_motion_overrides() {

  __disable_irq();
  sys_rt_exec_motion_override = 0;
  __enable_irq();

}

void system_clear_exec_accessory_overrides() {

  __disable_irq();
  sys_rt_exec_accessory_override = 0;
  __enable_irq();

}
