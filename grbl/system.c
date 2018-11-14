
/*
  system.c - ����ϵͳ�������ʵʱ����
  "$"ϵͳ����Ľ���
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


//����������״̬��Ϊuint8λ�򷵻ء�ÿһλ��ʾ��������״̬������
//����Ϊ1��δ����Ϊ0.Ӧ�÷�ת���롣���طƶ�����֯��
//��ͷ�ļ��е�CONTROL_PIN_INDEX���塣
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


//���ű仯�ж����������ѭ����ʼ���������ֺ͸�λ����
//ֻ��realtime����ִ�б���������������ִ����Щ
//׼������ ����ʰȡʱ�����ַ���ʵʱ������ȫ��ͬ
//ֱ�����Դ�����������

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

//��������״̬����ȫ�Źرգ�T����رգ�F�����ء�
uint8_t system_check_safety_door_ajar()
{
  #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    return(system_control_get_state() & CONTROL_PIN_INDEX_SAFETY_DOOR);
  #else
    return(false); // ��������û�����ã�����ֻ�Ƿ������رա�
  #endif
}


// ִ���û������ű�������洢����
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


//ָ����ִ��protocol_process��һ�и�ʽ�����롣��Ҫ��
//�������g����飬��Ҳִ��Grbl�ڲ������
//���ã�������λѭ���Լ��л�����״̬���ⲻͬ��
//��Grbl׼��ִ��ʱ�����ܵ�ʵʱ����ģ���Ӱ��
//һ��ѭ���е���һ�У����Զ������ɾ�������Ŀ��أ�����ֻ������
//֮������У���ѭ���в�һ����ʵʱ�ģ�
//��Ϊ�Ѿ��洢�ڻ������е��˶������ǣ�������ͺ󡱲�Ӧ��
//��һ�����⣬��Ϊ��Щ������һ��������ͨ������ʹ�á�
uint8_t system_execute_line(char *line)
{
  uint8_t char_counter = 1;
  uint8_t helper_var = 0; // ���ֱ���
  float parameter, value;
  switch( line[char_counter] ) {
    case 0 : report_grbl_help(); break;
    case 'J' : // ����
      // ����IDLE��JOG״̬��ִ�С�
      if (sys.state != STATE_IDLE && sys.state != STATE_JOG) { return(STATUS_IDLE_ERROR); }
      if(line[2] != '=') { return(STATUS_INVALID_STATEMENT); }
      return(gc_execute_line(line)); // NOTE: $J= ��g����������б����ԣ������ڼ��㶯�˶���
      //break;
    case '$': case 'G': case 'C': case 'X':
      if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      switch( line[1] ) {
        case '$' : // ��ӡGrbl����
          if ( sys.state & (STATE_CYCLE | STATE_HOLD) ) { return(STATUS_IDLE_ERROR); } // ��ѭ�������� ��ӡʱ��̫����
          else { report_grbl_settings(); }
          break;
        case 'G' : // ��ӡG-���������״̬
          // TODO: ��������GUI��ʵʱ�������ڹ���״̬�ڼ���������ݡ�
          report_gcode_modes();
          break;
        case 'C' : // ���ü��g����ģʽ [����/���]
          // �л�ʱִ�����á����g����ģʽӦ��ֻ������Grbl
          // ���۾����Ƿ�����������������״̬������Ҫ��Ϊ�˱��ֶ���
          // ��һ�¡�
          if ( sys.state == STATE_CHECK_MODE ) {
            mc_reset();
            report_feedback_message(MESSAGE_DISABLED);
          } else {
            if (sys.state) { return(STATUS_IDLE_ERROR); } // ����Ҫ����ģʽ��
            sys.state = STATE_CHECK_MODE;
            report_feedback_message(MESSAGE_ENABLED);
          }
          break;
        case 'X' : // ���þ�������[ALARM]
          if (sys.state == STATE_ALARM) {
            // �����ȫ���ǰ뿪��
            if (system_check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); }
            report_feedback_message(MESSAGE_ALARM_UNLOCK);
            sys.state = STATE_IDLE;
            // ��Ҫ���������ű��� ��ֹ����ʱ�洢���ƶ�����¹ʡ�
          } // ����û��Ч����
          break;
      }
      break;
    default :
      // ��ֹ�κ�Ҫ��״̬Ϊ����/������ϵͳ�������EEPROM����λ��
      if ( !(sys.state == STATE_IDLE || sys.state == STATE_ALARM) ) { return(STATUS_IDLE_ERROR); }
      switch( line[1] ) {
        case '#' : // ��ӡGrbl NGC����
          if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); }
          else { report_ngc_parameters(); }
          break;
        case 'H' : // ִ�й�λѭ��[IDLE / ALARM]
          if (bit_isfalse(settings.flags,BITFLAG_HOMING_ENABLE)) {return(STATUS_SETTING_DISABLED); }
          if (system_check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); } // �飬�����ȫ�ſ����������
          sys.state = STATE_HOMING; // ����ϵͳ״̬����
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
          if (!sys.abort) {  // �ɹ���λ��ִ�������ű���
            sys.state = STATE_IDLE; // ���ʱ����Ϊ���С�
            st_go_idle(); // �ڷ���֮ǰ������������Ϊ����״̬��
            if (line[2] == 0) { system_execute_startup(line); }
          }
          break;
        case 'S' : // ��Grbl˯�� [IDLE/ALARM]
          if ((line[2] != 'L') || (line[3] != 'P') || (line[4] != 0)) { return(STATUS_INVALID_STATEMENT); }
          system_set_exec_state_flag(EXEC_SLEEP); // ��������ִ��˯��ģʽ
          break;
        case 'I' : // ��ӡ��洢������Ϣ��[IDLE / ALARM]
          if ( line[++char_counter] == 0 ) {
            settings_read_build_info(line);
            report_build_info(line);
          #ifdef ENABLE_BUILD_INFO_WRITE_COMMAND
            } else { // �洢������[IDLE / ALARM]
              if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
              helper_var = char_counter; // ��������������Ϊ���������������û���Ϣ�С�
              do {
                line[char_counter-helper_var] = line[char_counter];
              } while (line[char_counter++] != 0);
              settings_store_build_info(line);
          #endif
          }
          break;
        case 'R' : // �ָ�Ĭ��[IDLE / ALARM]
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
          mc_reset(); // ǿ��������ȷ��������ȷ��ʼ����
          break;
        case 'N' : // �����ߡ�[IDLE / ALARM]
          if ( line[++char_counter] == 0 ) { // ��ӡ������
            for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++) {
              if (!(settings_read_startup_line(helper_var, line))) {
                report_status_message(STATUS_SETTING_READ_FAIL);
              } else {
                report_startup_line(helper_var,line);
              }
            }
            break;
          } else { // �洢������[��IDLE]��ֹ��ALARM�ڼ��ƶ���
            if (sys.state != STATE_IDLE) { return(STATUS_IDLE_ERROR); } // ���ڿ���ʱ�洢��
            helper_var = true;  // ����helper_var��Ǵ洢������
            // û����Ϣ ����Ĭ�ϣ���ȡʣ��������ַ���
          }
        default :  // �洢���÷���[IDLE / ALARM]
          if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
          if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
          if (helper_var) { // �洢������
            // ͨ���ƶ������ַ�׼������gcode�鵽gcode������
            helper_var = char_counter; // ���ø���������Ϊ��������ʼgcode��
            do {
              line[char_counter-helper_var] = line[char_counter];
            } while (line[char_counter++] != 0);
            // ִ��gcode����ȷ������Ч��
            helper_var = gc_execute_line(line); // ��helper_var����Ϊ���ص�״̬�롣
            if (helper_var) { return(helper_var); }
            else {
              helper_var = truncf(parameter); // ��helper_var����Ϊ������intֵ
              settings_store_startup_line(helper_var,line);
            }
          } else { // �洢ȫ�����á�
            if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
            if((line[char_counter] != 0) || (parameter > 255)) { return(STATUS_INVALID_STATEMENT); }
            return(settings_store_global_setting((uint8_t)parameter, value));
          }
      }
  }
  return(STATUS_OK); // ���'$'����������ôһ��������
}



void system_flag_wco_change()
{
  #ifdef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE
    protocol_buffer_synchronize();
  #endif
  sys.report_wco_counter = 0;
}


//�����ᡰidx���Ļ���λ�á����뷢��һ�������衱���顣
//ע�⣺�����������ͻ���λ�ò���ͬһ����ϵ�У��˹���
//    ��Ϊ����ת��������λ�á�
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


//��CoreXY���㡣����CoreXY�����������x��y�ᡰ������
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


  //��鲢����Ŀ�������Ƿ񳬳������г����ơ�
uint8_t system_check_travel_limits(float *target)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef HOMING_FORCE_SET_ORIGIN
      //�����ù�λǿ������ԭ��ʱ�������Ƽ����Ҫ���Ƿ����ԡ�
      //ע�⣺max_travel�洢Ϊ����
      if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
        if (target[idx] < 0 || target[idx] > -settings.max_travel[idx]) { return(true); }
      } else {
        if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { return(true); }
      }
    #else
      // ע�⣺max_travel�洢Ϊ����
      if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { return(true); }
    #endif
  }
  return(false);
}



// �������ú����Grbl��ʵʱִ�б�־�����⴦�����
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
