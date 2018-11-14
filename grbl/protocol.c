
/*
  protocol.c - ����Grblִ��Э��ͳ���
  ��λ��ͨ��Э�����
*/



#include "grbl.h"

// ΪԤ�������岻ͬ��ע�����͡�
#define LINE_FLAG_OVERFLOW bit(0)
#define LINE_FLAG_COMMENT_PARENTHESES bit(1)//����
#define LINE_FLAG_COMMENT_SEMICOLON bit(2)//�ֺ�

// ��Ҫִ�еĴ����������е��л�������С��
// ע�⣺���˼������֮�⣬û�����⣬�����л������Ĵ�С����̫С�ˡ�
// ��G�������Խضϡ���ʽ�Ĵ����׼��֧�ָߴ�256
// �ַ������Ժ�İ汾�У��⽫���ӣ�������֪���ж��ٶ����
// �ڴ�ռ����Ͷ�ʵ���������±�дG���������û�����
// ������
static char line[LINE_BUFFER_SIZE]; // Ҫִ�е��� ����ֹ��
#ifdef LEDBLINK
void LedBlink(void);
#endif

static void protocol_exec_rt_suspend();


// ��ʼ��ѭ���ķ��������������Դ��ж˿ڵ����д����ַ���ִ�С�
// ���������ʱ������������ɳ�ʼ�����̡�
void protocol_main_loop()
{
  // ִ��һЩ���������ȷ��һ��˳����
  #ifdef CHECK_LIMITS_AT_INIT
    if (bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE)) {
      if (limits_get_state()) {
        sys.state = STATE_ALARM; // ȷ������״̬���ڻ״̬��
        report_feedback_message(MESSAGE_CHECK_LIMITS);
      }
    }
  #endif
    //��λ��������ʼ�ӵ���鲢���汨��״̬��
    //ע�⣺˯��ģʽ���ò�����������λ�ò��ܵõ���֤��
    //��˯��״̬���³�ʼ��Ϊ����ģʽ����ȷ���û��ļ��л�ȷ�ϡ�
  if (sys.state & (STATE_ALARM | STATE_SLEEP)) {
    report_feedback_message(MESSAGE_ALARM_LOCK);
    sys.state = STATE_ALARM; // ȷ�������˾���״̬��
  } else {
    // ��鰲ȫ���Ƿ�򿪡�
    sys.state = STATE_IDLE;
    if (system_check_safety_door_ajar()) {
      bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
      protocol_execute_realtime(); // ���밲ȫ��ģʽ��Ӧ�÷���Ϊ����״̬��
    }
    // ����ϵͳ��ȥ
    system_execute_startup(line); // ִ�������ű���
  }

  // ---------------------------------------------------------------------------------
  // ��ѭ����һ��ϵͳ��ֹ�����˳�������������ϵͳ��
  // ��Ҳ��Grbl�ڵȴ�ĳ������ʱ���õĵط���
  // ---------------------------------------------------------------------------------

  uint8_t line_flags = 0;
  uint8_t char_counter = 0;
  uint8_t c;
  for (;;) {

    // ����һ�д���Ĵ������ݣ���Ϊ���ݿ��á�ִ��һ��
    // ͨ��ɾ���ո��ע�ͳ�ʼ���˲���д������ĸ��
    while((c = serial_read()) != SERIAL_NO_DATA) {
      if ((c == '\n') || (c == '\r')) { // ������β
        protocol_execute_realtime(); // ����ʱ������㡣
        if (sys.abort) { return; } // ϵͳ��ֹ��־�� ǿ���˳�����ѭ���Խ������á�

        line[char_counter] = 0; // �����ַ�����ֹ�ַ���



#ifdef LEDBLINK
				LedBlink();
#endif
		#ifdef REPORT_ECHO_LINE_RECEIVED
          report_echo_line_received(line);
        #endif

        // ָ����ִ��һ�и�ʽ�������룬������ִ��״̬��
        if (line_flags & LINE_FLAG_OVERFLOW) {
          // �������������
          report_status_message(STATUS_OVERFLOW);
        } else if (line[0] == 0) {
          // ���л�ע���� Ϊ��ͬ��Ŀ�ġ�
          report_status_message(STATUS_OK);
        } else if (line[0] == '$') {
          // Grbl'$'ϵͳ����
          report_status_message(system_execute_line(line));
        } else if (sys.state & (STATE_ALARM | STATE_JOG)) {
          // һ�ж���gcode��������ڱ�����㶯ģʽ������ֹ.
          report_status_message(STATUS_SYSTEM_GC_LOCK);
        } else {
          // ������ִ��g����顣
          report_status_message(gc_execute_line(line));
        }

        // ������һ�еĸ������ݡ�
        line_flags = 0;
        char_counter = 0;

      } else {

        if (line_flags) {
          // �������У�����EOL��ע���ַ�������ַ���
          if (c == ')') {
            // '����'ע�ͽ���������������
            if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) { line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES); }
          }
        } else {
          if (c <= ' ') {
            // �ӵ��հ׺Ϳ����ַ�
          } else if (c == '/') {
            // ��ֹɾ����֧�֡������ַ���
            // ע�⣺���֧�֣�ֻ��Ҫ���ϵͳ�Ƿ������˿�ɾ����
          } else if (c == '(') {
              //����ע�ͱ�־�����������ַ���ֱ��'��'��EOL��
              //ע�⣺�Ⲣ����ȫ��ѭNIST�Ķ��壬�������Ѿ��㹻�ˡ�
              //���������ǿ��Լ򵥵�ɾ�������е���Ŀ��������
              //ע�Ϳ����ַ����Ա�g-code���������Զ�����д����顣
            line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
          } else if (c == ';') {
            // ע�⣺';' ��EOL��������һ��LinuxCNC���塣����NIST��
            line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
          // TODO: ��װ'��'����
          // } else if (c == '%') {
            //����ʼ�����ٷֺŲ�֧�֡�
            //ע�⣺���԰�װ�������������Grbl����������ʱ���ֶ����룬
            //�ڳ����У�ϵͳ�Զ�ѭ������������ִ��
            //ֱ����һ��'��'���š��⽫�����ڽ��ĳЩ����Ļָ�����
            //��չ滮���������Լ�ʱִ��������ĺ�����
          } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
        	  //����л�������������ñ�־��
            line_flags |= LINE_FLAG_OVERFLOW;
          } else if (c >= 'a' && c <= 'z') { // ��дСд
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        }

      }
    }

    //������ж�ȡ��������û�и����ַ���Ҫ�����ִ�У�
    //�����g�������Ѿ�����˹滮���������Ѿ�
    //��� ���κ�һ������£��Զ�ѭ��������������ã����κ��Ŷӵ��ƶ���
    protocol_auto_cycle_start();

    protocol_execute_realtime();  // ����ʱ������㡣
    if (sys.abort) { return; } // ���浽main��������ѭ��������ϵͳ��
  }

  //return; /*��δ�ﵽ*/
}


//����ֱ�����л���Ĳ��豻ִ�л���ѭ��״̬�������ϱ���һ����
//��ͬ�������ڼ䣬��������������⣬�ȴ�������ڽ�����
void protocol_buffer_synchronize()
{
	//���ϵͳ�����Ŷӣ���ô����Զ�������־���ڣ�ȷ��ѭ��������
  protocol_auto_cycle_start();
  do {
    protocol_execute_realtime();   //��鲢ִ������ʱ����
    if (sys.abort) { return; } //���ϵͳ��ֹ
  } while (plan_get_current_block() || (sys.state == STATE_CYCLE));
}


//���˶�׼����ִ�в���������û��ʱ���Զ�ѭ����ʼ����
//������������
//ע�⣺������ѭ����������ͬ����mc_line�����е��ô˺�����ִ��
//����Щ����֮һ�ֱ����ʱ��û�и���Ŀ鱻���ͣ�����ʽ����
//��ɣ������������Ҫ�ȴ��������е��˶�������
//ִ�е��û�����ͬ�������߼ƻ�������������׼��������
void protocol_auto_cycle_start()
{
  if (plan_get_current_block() != NULL) { // ��黺�������Ƿ����κο顣
    system_set_exec_state_flag(EXEC_CYCLE_START); // ����ǵĻ���ִ�����ǣ�
  }
}


//���������Grbl��ʵʱ����ִ��ϵͳ��ͨ�ýӿڡ�������Ϊ
//�������еĸ������㣬��Ҫ�������������һ��whileѭ���ȴ�
//������տռ�Ļ�����������һ�������ִ��ʱ����ܵ��κε�
//��ֹһ���ӡ�����һ���첽ִ��ʵʱ����ķ���
//�����������񣩣���grbl��g-code�����͹滮���ܡ��������Ҳ������
//��Ϊ�жϵĽӿ�������ϵͳ��ʵʱ��־������ֻ��������
//�������ǣ�����Ҫ�������ļ��㰺���volatile���������
//���ṩ��һ���ܿصķ�ʽ��ִ��ĳЩ���񣬶�û������������ʵ��
//��ͬ�����񣬱���滮�����¼���һ��feedhold��overrides�ϵĻ�������
//ע�⣺sys_rt_exec_state������־���κν��̣���������жϣ����ŷ��䣬
//���ƿ��ػ�������
//Э��ִ��ʵʱ
void protocol_execute_realtime()
{
  protocol_exec_rt_system();
  //ϵͳ����λ��־���������ڹ����֣�ȡ���Ͱ�ȫ�š�
  if (sys.suspend) { protocol_exec_rt_suspend(); }
}


//��Ҫʱִ������ʱ������������Ҫ����Grbl��״̬
//�����Ϳ���Grbl�����ṩ�ĸ���ʵʱ���ܡ�
//ע�⣺��Ҫ�ı������������ȷ��֪��������ʲô��
void protocol_exec_rt_system()
{
  uint8_t rt_exec; // ��ʱ�������Ա�����ûӷ��Զ�Ρ�
  rt_exec = sys_rt_exec_alarm; //ȫ��ʵʱִ����λ��־�������������ø��ֱ���
  if (rt_exec) { // ֻ�����κ�λ��־Ϊ��ʱ�Ž���
	    //ϵͳ���� һ�ж��Ѿ��ر���һЩ���ش���Ķ���������
	    //�������Դ���û������������Ҫ��Grbl��ͨ����������������
	    //ѭ��ֱ��ϵͳ����/��ֹ��
    sys.state = STATE_ALARM; // ����ϵͳ����״̬
    report_alarm_message(rt_exec);
    //��ͣ�ؼ��¼���־�ϵ��������� ĿǰӲ���ƺ������Ʊ�־����һ�㡣
    if ((rt_exec == EXEC_ALARM_HARD_LIMIT) || (rt_exec == EXEC_ALARM_SOFT_LIMIT)) {
      report_feedback_message(MESSAGE_CRITICAL_EVENT);
      system_clear_exec_state_flag(EXEC_RESET); // �����κ����е�����
      do {
          //�������ú�״̬����֮�⣬��ֹ���е����飬ֱ���û��������û��߹���
          //ѭ�� Ӳ������ͨ���������չܻ�ע�������·�������
          //�û���һ��ͼ���û�����ʱ����������֮ǰ��Ҫ�������飬�����ɱ
          //������ ������Ҳ����ˡ���λ�ò���
          //��ʧ��������ʽ������ܻᵼ�����صı����������������ִ�С�
      } while (bit_isfalse(sys_rt_exec_state,EXEC_RESET));
    }
    system_clear_exec_alarm(); // �������
  }

  rt_exec = sys_rt_exec_state; //ȫ��ʵʱִ�г���λ��־��������״̬���������EXECλ����
  if (rt_exec) {

    // ִ��ϵͳ��ֹ��
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // ֻ�н��������Ϊtrue��
      return; // û�б�İ취�����˳���
    }

    // ִ�к�������ӡ״̬
    if (rt_exec & EXEC_STATUS_REPORT) {
      report_realtime_status();//����ʵʱ״̬
      system_clear_exec_state_flag(EXEC_STATUS_REPORT);
    }

    //ע�⣺һ���������֣�ϵͳ����������ͣ״̬����ֹ���е�״̬
    //��������̣�ֱ�����û�ָ�����ȷ���˱��ְ�ȫ��ɡ�
    if (rt_exec & (EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP)) {

    	//״̬��鱣�ַ���������״̬��
      if (!(sys.state & (STATE_ALARM | STATE_CHECK_MODE))) {
      
    	  //�������CYCLE��JOG״̬����������һ���˶�HOLD��
        if (sys.state & (STATE_CYCLE | STATE_JOG)) {
          if (!(sys.suspend & (SUSPEND_MOTION_CANCEL | SUSPEND_JOG_CANCEL))) { //�飬����Ѿ����С�
            st_update_plan_block_parameters(); //֪ͨ����ģ�����¼��㱣�ּ��١�
            sys.step_control = STEP_CONTROL_EXECUTE_HOLD; //������Ч��־����ͣ״̬��
            if (sys.state == STATE_JOG) { //�㶯ȡ�����κΰ����¼�������˯����
              if (!(rt_exec & EXEC_SLEEP)) { sys.suspend |= SUSPEND_JOG_CANCEL; } 
            }
          }
        }
        //���IDLE��Grbl�����˶��� ֻ��ָʾ����״̬��������ɼ��ɡ�
        if (sys.state == STATE_IDLE) { sys.suspend = SUSPEND_HOLD_COMPLETE; }

        //ִ�в��ü��ٱ�־һ������ȡ�������ص�����״̬����Ҫ����̽������
        //ֹͣ��ȡ��ʣ��Ķ�����
        if (rt_exec & EXEC_MOTION_CANCEL) {
            // MOTION_CANCEL����ѭ���з�����������Ԥ��������HOLD��SAFETY_DOOR
            //����ѭ����ȡ�������������ڵ����ƻ�������˶�����ȡ���㶯
            //�������������ƻ�������˶���
          if (!(sys.state & STATE_JOG)) { sys.suspend |= SUSPEND_MOTION_CANCEL; } // NOTE: State is STATE_CYCLE.
        }

        //������Ҫ����ִ�н������֡�Ȼ����ͣϵͳ��
        if (rt_exec & EXEC_FEED_HOLD) {
        	//��ֹSAFETY_DOOR��JOG��SLEEP״̬�Ӹ���ΪHOLD״̬��
          if (!(sys.state & (STATE_SAFETY_DOOR | STATE_JOG | STATE_SLEEP))) { sys.state = STATE_HOLD; }
        }

        //ͨ����������ִ�а�ȫ��ֹͣ����������/��ȴҺ��
        //ע�⣺��ȫ�Ų�ͬ�����ϱ���ͨ��ֹͣһ������״̬�����ù���
        //�豸������/��ȴҺ��������������ģ�飬ֱ���������½�ͨ��
        if (rt_exec & EXEC_SAFETY_DOOR) {
          report_feedback_message(MESSAGE_SAFETY_DOOR_AJAR);
          //������ܣ�����ֹ��ȫ�ŷ�����ֱ���㶯ȡ����ɡ�ֻҪ����������ˡ�
          if (!(sys.suspend & SUSPEND_JOG_CANCEL)) {
              //����Ƿ���ڻָ�ͣ�������ڼ����´򿪰�ȫװ�á��������
              //�Ѿ��ջأ�ͣ�Ż���˯��״̬��
            if (sys.state == STATE_SAFETY_DOOR) {
              if (sys.suspend & SUSPEND_INITIATE_RESTORE) { //�����ָ�
                #ifdef PARKING_ENABLE
            	  //���ñ��ֲ������ʵ��Ŀ��Ʊ�־����������ͣ��˳��
                  if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
                    st_update_plan_block_parameters(); //֪ͨ����ģ�����¼��㱣�ּ��١�
                    sys.step_control = (STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION);
                    sys.suspend &= ~(SUSPEND_HOLD_COMPLETE);
                  } //����NO_MOTION���ڻ״̬��
                #endif
                sys.suspend &= ~(SUSPEND_RETRACT_COMPLETE | SUSPEND_INITIATE_RESTORE | SUSPEND_RESTORE_COMPLETE);
                sys.suspend |= SUSPEND_RESTART_RETRACT;
              }
            }
            if (sys.state != STATE_SLEEP) { sys.state = STATE_SAFETY_DOOR; }
          }
          //ע�⣺��sys.state��ͬ���˱�־���Źر�ʱ����ı䡣ȷ���κ�ͣ���˶�
          //����ſ��عرղ���״̬���ص�HOLD����ִ�иò�����
          sys.suspend |= SUSPEND_SAFETY_DOOR_AJAR;
        }
        
      }

      if (rt_exec & EXEC_SLEEP) {
        if (sys.state == STATE_ALARM) { sys.suspend |= (SUSPEND_RETRACT_COMPLETE|SUSPEND_HOLD_COMPLETE); }
        sys.state = STATE_SLEEP; 
      }

      system_clear_exec_state_flag((EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP));
    }

    //ͨ�������������ж���ִ��ѭ���������Կ�ʼִ�ж����еĿ顣
    if (rt_exec & EXEC_CYCLE_START) {
        //����뱣������ͬʱ�����У����������������֣��˶�ȡ���Ͱ�ȫ�š�
        //ȷ���Զ�ѭ������������û����ȷ���û����������¼������֡�
      if (!(rt_exec & (EXEC_FEED_HOLD | EXEC_MOTION_CANCEL | EXEC_SAFETY_DOOR))) {
    	  //��ͣ���˶������ҳ����ѹر�ʱ���ָ�����״̬��
        if ((sys.state == STATE_SAFETY_DOOR) && !(sys.suspend & SUSPEND_SAFETY_DOOR_AJAR)) {
          if (sys.suspend & SUSPEND_RESTORE_COMPLETE) {
            sys.state = STATE_IDLE; //����Ϊ���������ָ�ѭ����
          } else if (sys.suspend & SUSPEND_RETRACT_COMPLETE) {
              //���SAFETY_DOOR���ã����������¼����������ָ�ԭʼλ�á�
              //ע�⣺Ҫʹ��ȫ�Żָ�������رտ��أ���HOLD״̬��ʾ
              //�˵�ִ����ɣ�����ζ�ų�ʼ�������ֲ������
              //�ָ������������ָ����̱��������±�־������һ����
              //��ɺ������Զ�����CYCLE_START���ָ����˳�����
            sys.suspend |= SUSPEND_INITIATE_RESTORE;
          }
        }
        //����IDLE�򱣳���ɲ�׼���ָ�ʱѭ��������
        if ((sys.state == STATE_IDLE) || ((sys.state & STATE_HOLD) && (sys.suspend & SUSPEND_HOLD_COMPLETE))) {
          if (sys.state == STATE_HOLD && sys.spindle_stop_ovr) {
            sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE_CYCLE; //����Ϊ����ͣ�����лָ���֮��ѭ��������
          } else {
        	  //ֻ�мƻ����������д����Ŷ��˶����˶�δȡ��ʱ���Ż�����ѭ����
            sys.step_control = STEP_CONTROL_NORMAL_OP; //��������ƻָ�����������
            if (plan_get_current_block() && bit_isfalse(sys.suspend,SUSPEND_MOTION_CANCEL)) {
              sys.suspend = SUSPEND_DISABLE; //��ͣ״̬
              sys.state = STATE_CYCLE;
              st_prep_buffer(); //�ڿ�ʼѭ��֮ǰ��ʼ�����λ�������
              st_wake_up();
            } else { //����ʲô�����������ò��ָ�����״̬��
              sys.suspend = SUSPEND_DISABLE; //��ͣ״̬
              sys.state = STATE_IDLE;
            }
          }
        }
      }
      system_clear_exec_state_flag(EXEC_CYCLE_START);
    }

    if (rt_exec & EXEC_CYCLE_STOP) {
        //���³�ʼ��ѭ���ƻ��Ͳ���ϵͳ�����ϱ���һ����������
        //����������ִ��ʵʱ���ȷ���滮��Ա��ȫ�����¼ƻ���
        //ע�⣺Bresenham�㷨������Ȼͨ���滮���Ͳ�������ά��
        //ѭ�����³�ʼ�� ����·��Ӧ����û���κ�����һ��������
        //ע�⣺��ѭ��������������ʱ���ɲ�����ϵͳ����EXEC_CYCLE_STOP��
      if ((sys.state & (STATE_HOLD|STATE_SAFETY_DOOR|STATE_SLEEP)) && !(sys.soft_limit) && !(sys.suspend & SUSPEND_JOG_CANCEL)) {
          //������� ����Ϊ��ʾ׼���ָ�������HOLD��DOOR״ֱ̬���û�
          //�ѷ����ָ���������á�
        plan_cycle_reinitialize();
        if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { sys.suspend |= SUSPEND_HOLD_COMPLETE; }
        bit_false(sys.step_control,(STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION));
      } else {
          //�˶���� ����CYCLE / JOG / HOMING״̬�͵㶯ȡ��/����ȡ��/�������¼���
          //ע�⣺�˶��͵㶯ȡ�������ڱ�����ɺ��������ص�����״̬��
        if (sys.suspend & SUSPEND_JOG_CANCEL) {   //��������ȡ����ˢ�»�������ͬ��λ�á�
          sys.step_control = STEP_CONTROL_NORMAL_OP;
          plan_reset();
          st_reset();
          gc_sync_position();
          plan_sync_position();
        }
        if (sys.suspend & SUSPEND_SAFETY_DOOR_AJAR) { //ֻ��������ʱ��ȫ�Ŵ򿪲Żᷢ����
          sys.suspend &= ~(SUSPEND_JOG_CANCEL);
          sys.suspend |= SUSPEND_HOLD_COMPLETE;
          sys.state = STATE_SAFETY_DOOR;
        } else {
          sys.suspend = SUSPEND_DISABLE;
          sys.state = STATE_IDLE;
        }
      }
      system_clear_exec_state_flag(EXEC_CYCLE_STOP);
    }
  }

  //ִ�и��ǡ�
  rt_exec = sys_rt_exec_motion_override; //ȫ��ʵʱִ����λ��־���������ڻ����˶��ĸ���
  if (rt_exec) {
    system_clear_exec_motion_overrides(); //��������˶����Ǳ�־��

    uint8_t new_f_override =  sys.f_override;
    if (rt_exec & EXEC_FEED_OVR_RESET) { new_f_override = DEFAULT_FEED_OVERRIDE; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_PLUS) { new_f_override += FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_MINUS) { new_f_override -= FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_PLUS) { new_f_override += FEED_OVERRIDE_FINE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_MINUS) { new_f_override -= FEED_OVERRIDE_FINE_INCREMENT; }
    new_f_override = min(new_f_override,MAX_FEED_RATE_OVERRIDE);
    new_f_override = max(new_f_override,MIN_FEED_RATE_OVERRIDE);

    uint8_t new_r_override = sys.r_override;
    if (rt_exec & EXEC_RAPID_OVR_RESET) { new_r_override = DEFAULT_RAPID_OVERRIDE; }
    if (rt_exec & EXEC_RAPID_OVR_MEDIUM) { new_r_override = RAPID_OVERRIDE_MEDIUM; }
    if (rt_exec & EXEC_RAPID_OVR_LOW) { new_r_override = RAPID_OVERRIDE_LOW; }

    if ((new_f_override != sys.f_override) || (new_r_override != sys.r_override)) {
      sys.f_override = new_f_override;
      sys.r_override = new_r_override;
      sys.report_ovr_counter = 0; //���������������
      plan_update_velocity_profile_parameters();
      plan_cycle_reinitialize();
    }
  }

  rt_exec = sys_rt_exec_accessory_override;//����/��ȴҺ���ǵ�ȫ��ʵʱִ�г���λ��־����
  if (rt_exec) {
    system_clear_exec_accessory_overrides(); //�������������Ǳ�־��

    //ע�⣺���˶����ǲ�ͬ�����Ḳ�ǲ���Ҫ�滮�����³�ʼ����
    uint8_t last_s_override =  sys.spindle_speed_ovr;
    if (rt_exec & EXEC_SPINDLE_OVR_RESET) { last_s_override = DEFAULT_SPINDLE_SPEED_OVERRIDE; }
    if (rt_exec & EXEC_SPINDLE_OVR_COARSE_PLUS) { last_s_override += SPINDLE_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_COARSE_MINUS) { last_s_override -= SPINDLE_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_FINE_PLUS) { last_s_override += SPINDLE_OVERRIDE_FINE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_FINE_MINUS) { last_s_override -= SPINDLE_OVERRIDE_FINE_INCREMENT; }
    last_s_override = min(last_s_override,MAX_SPINDLE_SPEED_OVERRIDE);
    last_s_override = max(last_s_override,MIN_SPINDLE_SPEED_OVERRIDE);

    if (last_s_override != sys.spindle_speed_ovr) {
      bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
      sys.spindle_speed_ovr = last_s_override;
      sys.report_ovr_counter = 0; //���������������
    }

    if (rt_exec & EXEC_SPINDLE_OVR_STOP) {
        //ֻ���ڱ���״̬�²���������ֹͣ���ۡ�
        //ע�⣺ִ������ֹͣʱ�������������spindle_set_state���������á�
      if (sys.state == STATE_HOLD) {
        if (!(sys.spindle_stop_ovr)) { sys.spindle_stop_ovr = SPINDLE_STOP_OVR_INITIATE; }
        else if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_ENABLED) { sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE; }
      }
    }

    //ע�⣺������ȴ��״̬�����ڼƻ���ͬ��ʱִ�У���ǰ
    //����״̬����ͨ����������״̬��ȷ����
    if (rt_exec & (EXEC_COOLANT_FLOOD_OVR_TOGGLE | EXEC_COOLANT_MIST_OVR_TOGGLE)) {
      if ((sys.state == STATE_IDLE) || (sys.state & (STATE_CYCLE | STATE_HOLD))) {
        uint8_t coolant_state = gc_state.modal.coolant;
        #ifdef ENABLE_M7
          if (rt_exec & EXEC_COOLANT_MIST_OVR_TOGGLE) {
            if (coolant_state & COOLANT_MIST_ENABLE) { bit_false(coolant_state,COOLANT_MIST_ENABLE); }
            else { coolant_state |= COOLANT_MIST_ENABLE; }
          }
          if (rt_exec & EXEC_COOLANT_FLOOD_OVR_TOGGLE) {
            if (coolant_state & COOLANT_FLOOD_ENABLE) { bit_false(coolant_state,COOLANT_FLOOD_ENABLE); }
            else { coolant_state |= COOLANT_FLOOD_ENABLE; }
          }
        #else
          if (coolant_state & COOLANT_FLOOD_ENABLE) { bit_false(coolant_state,COOLANT_FLOOD_ENABLE); }
          else { coolant_state |= COOLANT_FLOOD_ENABLE; }
        #endif
        coolant_set_state(coolant_state); //��coolant_set_state���������ñ����������
        gc_state.modal.coolant = coolant_state;
      }
    }
  }

  #ifdef DEBUG
    if (sys_rt_exec_debug) {
      report_realtime_debug();
      sys_rt_exec_debug = 0;
    }
  #endif

    //���¼��ضλ�����
  if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_SAFETY_DOOR | STATE_HOMING | STATE_SLEEP| STATE_JOG)) {
    st_prep_buffer();
  }

}


//����Grblϵͳ��ͣ������������֣���ȫ�ź�ͣ��������
//ϵͳ�������ѭ����Ϊ��ͣ���񴴽��ֲ�������Ȼ�󷵻�
//������ͣ���κκ������Ա�Grbl�ָ�����������
//���������д����Ϊ�˴ٽ��Զ����ͣ���������򵥵�ʹ�������Ϊһ��
//ģ��
static void protocol_exec_rt_suspend()
{
  #ifdef PARKING_ENABLE
	//��������ʼ��ͣ�žֲ�����
    float restore_target[N_AXIS];
    float parking_target[N_AXIS];
    float retract_waypoint = PARKING_PULLOUT_INCREMENT;
    plan_line_data_t plan_data;
    plan_line_data_t *pl_data = &plan_data;
    memset(pl_data,0,sizeof(plan_line_data_t));
    pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
    #ifdef USE_LINE_NUMBERS
      pl_data->line_number = PARKING_MOTION_LINE_NUMBER;
    #endif
  #endif

  plan_block_t *block = plan_get_current_block();
  uint8_t restore_condition;
  #ifdef VARIABLE_SPINDLE
    float restore_spindle_speed;
    if (block == NULL) {
      restore_condition = (gc_state.modal.spindle | gc_state.modal.coolant);
      restore_spindle_speed = gc_state.spindle_speed;
    } else {
      restore_condition = block->condition;
      restore_spindle_speed = block->spindle_speed;
    }
    #ifdef DISABLE_LASER_DURING_HOLD
      if (bit_istrue(settings.flags, BITFLAG_LASER_MODE)) {
        system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP);
      }
    #endif
  #else
    if (block == NULL) { restore_condition = (gc_state.modal.spindle | gc_state.modal.coolant); }
    else { restore_condition = block->condition; }
  #endif

  while (sys.suspend) {

    if (sys.abort) { return; }

    //��ֹ��ֱ����ʼ������ɣ�����ֹͣ�˶���
    if (sys.suspend & SUSPEND_HOLD_COMPLETE) {

        //ͣ������������de / re-energizing������״̬����ͣ���˶�
        //��ȫ�ź�˯��״̬��
      if (sys.state & (STATE_SAFETY_DOOR | STATE_SLEEP)) {
      
          //��������˶��Ͷϵ硣
        if (bit_isfalse(sys.suspend,SUSPEND_RETRACT_COMPLETE)) {

            //ȷ���ڿ�ʼ��ȫ�ų���ʱ�����κ���ǰ������ֹͣ���ǡ�
          sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED;

          #ifndef PARKING_ENABLE

            spindle_set_state(SPINDLE_DISABLE,0.0f); // �ϵ�
            coolant_set_state(COOLANT_DISABLE);     // �ϵ�

          #else
					
            //��ȡ��ǰλ�ò��洢�ָ�λ�ú������ջغ��㡣
            system_convert_array_steps_to_mpos(parking_target,sys_position);
            if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
              memcpy(restore_target,parking_target,sizeof(parking_target));
              retract_waypoint += restore_target[PARKING_AXIS];
              retract_waypoint = min(retract_waypoint,PARKING_TARGET);
            }

            //ִ������ͣ���շŶ�����ͣ����Ҫ���õ�����
            //��ǰλ�ò�����ͣ��Ŀ��λ�ã����ü���ģʽ��
            //ע�⣺״̬������DOOR״̬��ֱ����ȫ�ϵ���ջء�
						#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
						if ((bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE)) &&
														(parking_target[PARKING_AXIS] < PARKING_TARGET) &&
														bit_isfalse(settings.flags, BITFLAG_LASER_MODE) &&
														(sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
						#else
						if ((bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE)) &&
														(parking_target[PARKING_AXIS] < PARKING_TARGET) &&
														bit_isfalse(settings.flags, BITFLAG_LASER_MODE)) {
						#endif
							//ͨ�����������ջ����ᡣȷ�������˶��뿪
							//�����ͺ�·���˶�������ͣ��Ŀ��λ�á�
              if (parking_target[PARKING_AXIS] < retract_waypoint) {
                parking_target[PARKING_AXIS] = retract_waypoint;
                pl_data->feed_rate = PARKING_PULLOUT_RATE;
                pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); //�������״̬
                pl_data->spindle_speed = restore_spindle_speed;
                mc_parking_motion(parking_target, pl_data);
              }

              //ע�⣺���ջغ���ֹ�ָ�����֮���������״̬��
              pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
              pl_data->spindle_speed = 0.0f;
              spindle_set_state(SPINDLE_DISABLE,0.0f); // �ϵ�
              coolant_set_state(COOLANT_DISABLE); // �ϵ�

              // ִ�п���ͣ�������˶���ͣ��Ŀ��λ�á�
              if (parking_target[PARKING_AXIS] < PARKING_TARGET) {
                parking_target[PARKING_AXIS] = PARKING_TARGET;
                pl_data->feed_rate = PARKING_RATE;
                mc_parking_motion(parking_target, pl_data);
              }

            } else {

              // ͣ�������� ֻ������������ȴҺ��
              // ע�⣺����ģʽ������ͣ�Ŷ�����ȷ����������ֹͣ��
              spindle_set_state(SPINDLE_DISABLE,0.0f); // �ϵ�
              coolant_set_state(COOLANT_DISABLE);     // �ϵ�

            }

          #endif

          sys.suspend &= ~(SUSPEND_RESTART_RETRACT);
          sys.suspend |= SUSPEND_RETRACT_COMPLETE;

        } else {

          
          if (sys.state == STATE_SLEEP) {
            report_feedback_message(MESSAGE_SLEEP_MODE);
            //�������ȴҺӦ���Ѿ�ֹͣ�ˣ�����Ҫȷ��һ�¡�
            spindle_set_state(SPINDLE_DISABLE,0.0f); // �ϵ�
            coolant_set_state(COOLANT_DISABLE); // �ϵ�
            st_go_idle(); // ���ò�����
            while (!(sys.abort)) { protocol_exec_rt_system(); } // ��Ҫ���κ��£�ֱ�����á�
            return; // ��ֹ�յ����������³�ʼ����
          }    
          
          //�����ͣ��/��ȫ�Żָ���������鰲ȫ���Ƿ�رղ�׼���ָ���
          if (sys.state == STATE_SAFETY_DOOR) {
            if (!(system_check_safety_door_ajar())) {
              sys.suspend &= ~(SUSPEND_SAFETY_DOOR_AJAR); //������С�����ʾ׼���ָ���
            }
          }

          //����ͣ���ָ��Ͱ�ȫ�ż�����
          if (sys.suspend & SUSPEND_INITIATE_RESTORE) {

            #ifdef PARKING_ENABLE
              //ִ�п��ٻָ��˶�������λ�á�ͣ����Ҫ���û�ԭ�㡣
              //ע�⣺״̬������DOOR״̬��ֱ����ȫ�ϵ���ջء�
							#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
							if (((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) &&
									 (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
							#else
							if ((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
							#endif
				//�����ȷ���˶�����������λ��֮���ƶ���
                if (parking_target[PARKING_AXIS] <= PARKING_TARGET) {
                  parking_target[PARKING_AXIS] = retract_waypoint;
                  pl_data->feed_rate = PARKING_RATE;
                  mc_parking_motion(parking_target, pl_data);
                }
              }
            #endif

			//�ӳ��������������������ȴҺ����ʱ�ϵ磬Ȼ�����¿�ʼѭ����
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              // ����ǰ�Ļָ������У������ȫ�����´򿪣�����ֹ��
              if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
                if (bit_istrue(settings.flags,BITFLAG_LASER_MODE)) {
                  // �ڼ���ģʽ�£�����������ת�ӳ١��趨��ѭ����ʼʱ�򿪼��⡣
                  bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
                } else {
                  spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
                  delay_sec(SAFETY_DOOR_SPINDLE_DELAY, DELAY_MODE_SYS_SUSPEND);
                }
              }
            }
            if (gc_state.modal.coolant != COOLANT_DISABLE) {
              // ����ǰ�Ļָ������У������ȫ�����´򿪣�����ֹ��
              if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
                // ע�⣺����ģʽ����������ӳ١�����ϵͳͨ���ɴ����ſ��ơ�
                coolant_set_state((restore_condition & (PL_COND_FLAG_COOLANT_FLOOD | PL_COND_FLAG_COOLANT_FLOOD)));
                delay_sec(SAFETY_DOOR_COOLANT_DELAY, DELAY_MODE_SYS_SUSPEND);
              }
            }

            #ifdef PARKING_ENABLE
              // ������λ��ִ�л����Ĳ��붯���Իָ�λ�á�
						#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
						if (((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) &&
									(sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
							#else
							if ((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
							#endif
                // ����ǰ�Ļָ������У������ȫ�����´򿪣�����ֹ��
                if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
                    //�����ջ�ͣ���Ƿ�����Ч/��ȫ�Ķ�����
                    //�ָ�ͣ���鰸Ӧ�߼�����Ч��Ҫôͨ������
                    //ͨ����Ч�Ļ����ռ���߸������ƶ�ԭʼλ�á�
                  pl_data->feed_rate = PARKING_PULLOUT_RATE;
									pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); // Restore accessory state
									pl_data->spindle_speed = restore_spindle_speed;
                  mc_parking_motion(restore_target, pl_data);
                }
              }
            #endif

            if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
              sys.suspend |= SUSPEND_RESTORE_COMPLETE;
              system_set_exec_state_flag(EXEC_CYCLE_START); //����Ϊ�ָ�����
            }
          }

        }


      } else {

          // Feed���ֹ���������������ֹͣ����״̬��
          //ע�⣺����ͣ����ʼʱͨ�����������ɱ��֡�
        if (sys.spindle_stop_ovr) {
        	//��������ֹͣ�Ŀ�ʼ
          if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_INITIATE) {
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              spindle_set_state(SPINDLE_DISABLE,0.0f); //�ϵ�
              sys.spindle_stop_ovr = SPINDLE_STOP_OVR_ENABLED; //��ֹͣ����״̬����Ϊ���ã�����ϵ磩��
            } else {
              sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED; //���ֹͣ����״̬
            }
            //��������״̬�Ļָ�
          } else if (sys.spindle_stop_ovr & (SPINDLE_STOP_OVR_RESTORE | SPINDLE_STOP_OVR_RESTORE_CYCLE)) {
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              report_feedback_message(MESSAGE_SPINDLE_RESTORE);
              if (bit_istrue(settings.flags,BITFLAG_LASER_MODE)) {
            	  //�ڼ���ģʽ�£�����������ת�ӳ١��趨��ѭ����ʼʱ�򿪼��⡣
                bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
              } else {
                spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
              }
            }
            if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_RESTORE_CYCLE) {
              system_set_exec_state_flag(EXEC_CYCLE_START);  //����Ϊ�ָ�����
            }
            sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED; //���ֹͣ����״̬
          }
        } else {
            //�ڱ����ڼ䴦������״̬��ע�⣺�ڱ���״̬�£�����ת�ٸ�д���ܻ�ı䡣
            //ע�⣺STEP_CONTROL_UPDATE_SPINDLE_PWM�ڲ����������лָ����Զ���λ��
          if (bit_istrue(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM)) {
            spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
            bit_false(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
          }
        }

      }
    }

    protocol_exec_rt_system();

  }
}
