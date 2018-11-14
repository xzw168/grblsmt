
/*
  protocol.c - 控制Grbl执行协议和程序
  上位机通信协议解析
*/



#include "grbl.h"

// 为预解析定义不同的注释类型。
#define LINE_FLAG_OVERFLOW bit(0)
#define LINE_FLAG_COMMENT_PARENTHESES bit(1)//括号
#define LINE_FLAG_COMMENT_SEMICOLON bit(2)//分号

// 从要执行的串行输入流中的行缓冲区大小。
// 注意：除了极端情况之外，没有问题，但是行缓冲区的大小可能太小了。
// 和G代码块可以截断。正式的代码标准，支持高达256
// 字符。在以后的版本中，这将增加，当我们知道有多少额外的
// 内存空间可以投资到这里或重新编写G代码解释器没有这个
// 缓冲区
static char line[LINE_BUFFER_SIZE]; // 要执行的行 零终止。
#ifdef LEDBLINK
void LedBlink(void);
#endif

static void protocol_exec_rt_suspend();


// 开始主循环的方法。它处理来自串行端口的所有传入字符并执行。
// 当它们完成时。它还负责完成初始化过程。
void protocol_main_loop()
{
  // 执行一些机器检查以确保一切顺利。
  #ifdef CHECK_LIMITS_AT_INIT
    if (bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE)) {
      if (limits_get_state()) {
        sys.state = STATE_ALARM; // 确保警报状态处于活动状态。
        report_feedback_message(MESSAGE_CHECK_LIMITS);
      }
    }
  #endif
    //复位，错误或初始加电后检查并报告报警状态。
    //注意：睡眠模式禁用步进驱动器，位置不能得到保证。
    //将睡眠状态重新初始化为警报模式，以确保用户的家中或确认。
  if (sys.state & (STATE_ALARM | STATE_SLEEP)) {
    report_feedback_message(MESSAGE_ALARM_LOCK);
    sys.state = STATE_ALARM; // 确保设置了警报状态。
  } else {
    // 检查安全门是否打开。
    sys.state = STATE_IDLE;
    if (system_check_safety_door_ajar()) {
      bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
      protocol_execute_realtime(); // 进入安全门模式。应该返回为空闲状态。
    }
    // 所有系统都去
    system_execute_startup(line); // 执行启动脚本。
  }

  // ---------------------------------------------------------------------------------
  // 主循环！一旦系统中止，这退出到主（）重置系统。
  // 这也是Grbl在等待某件事情时闲置的地方。
  // ---------------------------------------------------------------------------------

  uint8_t line_flags = 0;
  uint8_t char_counter = 0;
  uint8_t c;
  for (;;) {

    // 处理一行传入的串行数据，因为数据可用。执行一个
    // 通过删除空格和注释初始过滤并大写所有字母。
    while((c = serial_read()) != SERIAL_NO_DATA) {
      if ((c == '\n') || (c == '\r')) { // 到达行尾
        protocol_execute_realtime(); // 运行时命令检查点。
        if (sys.abort) { return; } // 系统中止标志。 强制退出到主循环以进行重置。

        line[char_counter] = 0; // 设置字符串终止字符。



#ifdef LEDBLINK
				LedBlink();
#endif
		#ifdef REPORT_ECHO_LINE_RECEIVED
          report_echo_line_received(line);
        #endif

        // 指定并执行一行格式化的输入，并报告执行状态。
        if (line_flags & LINE_FLAG_OVERFLOW) {
          // 报告行溢出错误。
          report_status_message(STATUS_OVERFLOW);
        } else if (line[0] == 0) {
          // 空行或注释行 为了同步目的。
          report_status_message(STATUS_OK);
        } else if (line[0] == '$') {
          // Grbl'$'系统命令
          report_status_message(system_execute_line(line));
        } else if (sys.state & (STATE_ALARM | STATE_JOG)) {
          // 一切都是gcode。如果处于报警或点动模式，则阻止.
          report_status_message(STATUS_SYSTEM_GC_LOCK);
        } else {
          // 解析并执行g代码块。
          report_status_message(gc_execute_line(line));
        }

        // 重置下一行的跟踪数据。
        line_flags = 0;
        char_counter = 0;

      } else {

        if (line_flags) {
          // 丢弃所有（除了EOL）注释字符和溢出字符。
          if (c == ')') {
            // '（）'注释结束。简历行允许。
            if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) { line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES); }
          }
        } else {
          if (c <= ' ') {
            // 扔掉空白和控制字符
          } else if (c == '/') {
            // 阻止删除不支持。忽略字符。
            // 注意：如果支持，只需要检查系统是否启用了块删除。
          } else if (c == '(') {
              //启用注释标志并忽略所有字符，直到'）'或EOL。
              //注意：这并不完全遵循NIST的定义，但现在已经足够了。
              //将来，我们可以简单地删除评论中的项目，但保留
              //注释控制字符，以便g-code解析器可以对其进行错误检查。
            line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
          } else if (c == ';') {
            // 注意：';' 对EOL的评论是一个LinuxCNC定义。不是NIST。
            line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
          // TODO: 安装'％'功能
          // } else if (c == '%') {
            //程序开始结束百分号不支持。
            //注意：可以安装这个程序来告诉Grbl程序在运行时与手动输入，
            //在程序中，系统自动循环启动将继续执行
            //直到下一个'％'符号。这将有助于解决某些问题的恢复问题
            //清空规划器缓冲区以及时执行其任务的函数。
          } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
        	  //检测行缓冲区溢出并设置标志。
            line_flags |= LINE_FLAG_OVERFLOW;
          } else if (c >= 'a' && c <= 'z') { // 大写小写
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        }

      }
    }

    //如果串行读取缓冲区中没有更多字符需要处理和执行，
    //这表明g代码流已经填充了规划缓冲区或已经
    //完成 在任何一种情况下，自动循环启动（如果启用），任何排队的移动。
    protocol_auto_cycle_start();

    protocol_execute_realtime();  // 运行时命令检查点。
    if (sys.abort) { return; } // 保存到main（）程序循环来重置系统。
  }

  //return; /*从未达到*/
}


//阻塞直到所有缓冲的步骤被执行或处于循环状态。与饲料保持一起工作
//在同步调用期间，如果它发生。另外，等待清洁周期结束。
void protocol_buffer_synchronize()
{
	//如果系统正在排队，那么如果自动启动标志存在，确保循环继续。
  protocol_auto_cycle_start();
  do {
    protocol_execute_realtime();   //检查并执行运行时命令
    if (sys.abort) { return; } //检查系统中止
  } while (plan_get_current_block() || (sys.state == STATE_CYCLE));
}


//当运动准备好执行并且主程序没有时，自动循环开始触发
//主动解析命令
//注意：仅在主循环，缓冲区同步和mc_line（）中调用此函数并执行
//当这些条件之一分别存在时：没有更多的块被发送（即流式传输
//完成，单个命令），需要等待缓冲区中的运动的命令
//执行调用缓冲区同步，或者计划缓冲区已满并准备就绪。
void protocol_auto_cycle_start()
{
  if (plan_get_current_block() != NULL) { // 检查缓冲区中是否有任何块。
    system_set_exec_state_flag(EXEC_CYCLE_START); // 如果是的话，执行它们！
  }
}


//这个函数是Grbl的实时命令执行系统的通用接口。它被称为
//主程序中的各个检查点，主要是在那里可能有一个while循环等待
//用于清空空间的缓冲区或者上一个检查点的执行时间可能的任何点
//不止一秒钟。这是一种异步执行实时命令的方法
//（又名多任务），用grbl的g-code解析和规划功能。这个功能也起作用
//作为中断的接口来设置系统的实时标志，其中只有主程序
//处理它们，不需要定义更多的计算昂贵的volatile变量。这个
//还提供了一个受控的方式来执行某些任务，而没有两个或更多的实例
//相同的任务，比如规划者重新计算一个feedhold或overrides上的缓冲区。
//注意：sys_rt_exec_state变量标志由任何进程，步骤或串行中断，引脚分配，
//限制开关或主程序。
//协议执行实时
void protocol_execute_realtime()
{
  protocol_exec_rt_system();
  //系统挂起位标志变量，用于管理保持，取消和安全门。
  if (sys.suspend) { protocol_exec_rt_suspend(); }
}


//需要时执行运行时命令。这个函数主要用作Grbl的状态
//机器和控制Grbl必须提供的各种实时功能。
//注意：不要改变这个，除非你确切知道你在做什么！
void protocol_exec_rt_system()
{
  uint8_t rt_exec; // 临时变量，以避免调用挥发性多次。
  rt_exec = sys_rt_exec_alarm; //全局实时执行器位标志变量，用于设置各种报警
  if (rt_exec) { // 只有在任何位标志为真时才进入
	    //系统报警 一切都已经关闭了一些严重错误的东西。报告
	    //错误的来源给用户。如果至关重要，Grbl将通过输入无限来禁用
	    //循环直到系统重置/中止。
    sys.state = STATE_ALARM; // 设置系统报警状态
    report_alarm_message(rt_exec);
    //暂停关键事件标志上的所有内容 目前硬限制和软限制标志着这一点。
    if ((rt_exec == EXEC_ALARM_HARD_LIMIT) || (rt_exec == EXEC_ALARM_SOFT_LIMIT)) {
      report_feedback_message(MESSAGE_CRITICAL_EVENT);
      system_clear_exec_state_flag(EXEC_RESET); // 禁用任何现有的重置
      do {
          //除了重置和状态报告之外，阻止所有的事情，直到用户问题重置或者供电
          //循环 硬性限制通常在无人照管或不注意的情况下发生。给
          //用户和一个图形用户界面时间来做重置之前需要做的事情，比如查杀
          //传入流 软限制也是如此。而位置不是
          //丢失，继续流式传输可能会导致严重的崩溃，如果碰巧它被执行。
      } while (bit_isfalse(sys_rt_exec_state,EXEC_RESET));
    }
    system_clear_exec_alarm(); // 清除警报
  }

  rt_exec = sys_rt_exec_state; //全局实时执行程序位标志变量用于状态管理。请参阅EXEC位掩码
  if (rt_exec) {

    // 执行系统中止。
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // 只有将这个设置为true。
      return; // 没有别的办法，但退出。
    }

    // 执行和连续打印状态
    if (rt_exec & EXEC_STATUS_REPORT) {
      report_realtime_status();//报告实时状态
      system_clear_exec_state_flag(EXEC_STATUS_REPORT);
    }

    //注意：一旦启动保持，系统立即进入暂停状态以阻止所有的状态
    //主程序进程，直到重置或恢复。这确保了保持安全完成。
    if (rt_exec & (EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP)) {

    	//状态检查保持方法的允许状态。
      if (!(sys.state & (STATE_ALARM | STATE_CHECK_MODE))) {
      
    	  //如果处于CYCLE或JOG状态，立即启动一个运动HOLD。
        if (sys.state & (STATE_CYCLE | STATE_JOG)) {
          if (!(sys.suspend & (SUSPEND_MOTION_CANCEL | SUSPEND_JOG_CANCEL))) { //块，如果已经持有。
            st_update_plan_block_parameters(); //通知步进模块重新计算保持减速。
            sys.step_control = STEP_CONTROL_EXECUTE_HOLD; //启动有效标志的暂停状态。
            if (sys.state == STATE_JOG) { //点动取消在任何把握事件，除了睡觉。
              if (!(rt_exec & EXEC_SLEEP)) { sys.suspend |= SUSPEND_JOG_CANCEL; } 
            }
          }
        }
        //如果IDLE，Grbl不在运动中 只需指示挂起状态并保持完成即可。
        if (sys.state == STATE_IDLE) { sys.suspend = SUSPEND_HOLD_COMPLETE; }

        //执行并用减速标志一个动作取消并返回到空闲状态。主要用于探测周期
        //停止并取消剩余的动作。
        if (rt_exec & EXEC_MOTION_CANCEL) {
            // MOTION_CANCEL仅在循环中发生，但可能预先启动了HOLD和SAFETY_DOOR
            //保存循环。取消动作仅适用于单个计划程序块运动，而取消点动
            //将处理并清除多个计划程序块运动。
          if (!(sys.state & STATE_JOG)) { sys.suspend |= SUSPEND_MOTION_CANCEL; } // NOTE: State is STATE_CYCLE.
        }

        //根据需要减速执行进给保持。然后，暂停系统。
        if (rt_exec & EXEC_FEED_HOLD) {
        	//阻止SAFETY_DOOR，JOG和SLEEP状态从更改为HOLD状态。
          if (!(sys.state & (STATE_SAFETY_DOOR | STATE_JOG | STATE_SLEEP))) { sys.state = STATE_HOLD; }
        }

        //通过进给保持执行安全门停止并禁用主轴/冷却液。
        //注意：安全门不同于饲料保持通过停止一切无论状态，禁用供电
        //设备（主轴/冷却液），并重新启动模块，直到开关重新接通。
        if (rt_exec & EXEC_SAFETY_DOOR) {
          report_feedback_message(MESSAGE_SAFETY_DOOR_AJAR);
          //如果慢跑，则阻止安全门方法，直到点动取消完成。只要标记它发生了。
          if (!(sys.suspend & SUSPEND_JOG_CANCEL)) {
              //检查是否仅在恢复停车动作期间重新打开安全装置。忽略如果
              //已经收回，停放或处于睡眠状态。
            if (sys.state == STATE_SAFETY_DOOR) {
              if (sys.suspend & SUSPEND_INITIATE_RESTORE) { //积极恢复
                #ifdef PARKING_ENABLE
            	  //设置保持并重置适当的控制标志以重新启动停车顺序。
                  if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
                    st_update_plan_block_parameters(); //通知步进模块重新计算保持减速。
                    sys.step_control = (STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION);
                    sys.suspend &= ~(SUSPEND_HOLD_COMPLETE);
                  } //否则NO_MOTION处于活动状态。
                #endif
                sys.suspend &= ~(SUSPEND_RETRACT_COMPLETE | SUSPEND_INITIATE_RESTORE | SUSPEND_RESTORE_COMPLETE);
                sys.suspend |= SUSPEND_RESTART_RETRACT;
              }
            }
            if (sys.state != STATE_SLEEP) { sys.state = STATE_SAFETY_DOOR; }
          }
          //注意：与sys.state不同，此标志在门关闭时不会改变。确保任何停车运动
          //如果门开关关闭并且状态返回到HOLD，则执行该操作。
          sys.suspend |= SUSPEND_SAFETY_DOOR_AJAR;
        }
        
      }

      if (rt_exec & EXEC_SLEEP) {
        if (sys.state == STATE_ALARM) { sys.suspend |= (SUSPEND_RETRACT_COMPLETE|SUSPEND_HOLD_COMPLETE); }
        sys.state = STATE_SLEEP; 
      }

      system_clear_exec_state_flag((EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP));
    }

    //通过启动步进器中断来执行循环启动，以开始执行队列中的块。
    if (rt_exec & EXEC_CYCLE_START) {
        //如果与保持命令同时被呼叫，则阻塞：进给保持，运动取消和安全门。
        //确保自动循环启动不会在没有明确的用户输入的情况下继续保持。
      if (!(rt_exec & (EXEC_FEED_HOLD | EXEC_MOTION_CANCEL | EXEC_SAFETY_DOOR))) {
    	  //当停车运动缩回且车门已关闭时，恢复车门状态。
        if ((sys.state == STATE_SAFETY_DOOR) && !(sys.suspend & SUSPEND_SAFETY_DOOR_AJAR)) {
          if (sys.suspend & SUSPEND_RESTORE_COMPLETE) {
            sys.state = STATE_IDLE; //设置为空闲立即恢复循环。
          } else if (sys.suspend & SUSPEND_RETRACT_COMPLETE) {
              //如果SAFETY_DOOR禁用，则标记以重新激活供电组件并恢复原始位置。
              //注意：要使安全门恢复，必须关闭开关，如HOLD状态所示
              //退刀执行完成，这意味着初始进给保持不活动。至
              //恢复正常操作，恢复过程必须由以下标志启动。一旦，
              //完成后，它会自动调用CYCLE_START来恢复并退出挂起。
            sys.suspend |= SUSPEND_INITIATE_RESTORE;
          }
        }
        //仅在IDLE或保持完成并准备恢复时循环启动。
        if ((sys.state == STATE_IDLE) || ((sys.state & STATE_HOLD) && (sys.suspend & SUSPEND_HOLD_COMPLETE))) {
          if (sys.state == STATE_HOLD && sys.spindle_stop_ovr) {
            sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE_CYCLE; //设置为在暂停程序中恢复，之后循环启动。
          } else {
        	  //只有计划器缓冲区中存在排队运动且运动未取消时，才会启动循环。
            sys.step_control = STEP_CONTROL_NORMAL_OP; //将步骤控制恢复到正常操作
            if (plan_get_current_block() && bit_isfalse(sys.suspend,SUSPEND_MOTION_CANCEL)) {
              sys.suspend = SUSPEND_DISABLE; //暂停状态
              sys.state = STATE_CYCLE;
              st_prep_buffer(); //在开始循环之前初始化步段缓冲区。
              st_wake_up();
            } else { //否则，什么都不做。设置并恢复空闲状态。
              sys.suspend = SUSPEND_DISABLE; //暂停状态
              sys.state = STATE_IDLE;
            }
          }
        }
      }
      system_clear_exec_state_flag(EXEC_CYCLE_START);
    }

    if (rt_exec & EXEC_CYCLE_STOP) {
        //重新初始化循环计划和步进系统后，饲料保持一个简历。叫
        //在主程序中执行实时命令，确保规划人员安全地重新计划。
        //注意：Bresenham算法变量仍然通过规划器和步进器来维护
        //循环重新初始化 步进路径应该像没有任何事情一样继续。
        //注意：当循环或进给保持完成时，由步进子系统设置EXEC_CYCLE_STOP。
      if ((sys.state & (STATE_HOLD|STATE_SAFETY_DOOR|STATE_SLEEP)) && !(sys.soft_limit) && !(sys.suspend & SUSPEND_JOG_CANCEL)) {
          //保持完成 设置为表示准备恢复。保持HOLD或DOOR状态直到用户
          //已发出恢复命令或重置。
        plan_cycle_reinitialize();
        if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { sys.suspend |= SUSPEND_HOLD_COMPLETE; }
        bit_false(sys.step_control,(STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION));
      } else {
          //运动完成 包括CYCLE / JOG / HOMING状态和点动取消/动作取消/软限制事件。
          //注意：运动和点动取消都会在保持完成后立即返回到空闲状态。
        if (sys.suspend & SUSPEND_JOG_CANCEL) {   //用于慢跑取消，刷新缓冲区和同步位置。
          sys.step_control = STEP_CONTROL_NORMAL_OP;
          plan_reset();
          st_reset();
          gc_sync_position();
          plan_sync_position();
        }
        if (sys.suspend & SUSPEND_SAFETY_DOOR_AJAR) { //只有在慢跑时安全门打开才会发生。
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

  //执行覆盖。
  rt_exec = sys_rt_exec_motion_override; //全局实时执行器位标志变量，用于基于运动的覆盖
  if (rt_exec) {
    system_clear_exec_motion_overrides(); //清除所有运动覆盖标志。

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
      sys.report_ovr_counter = 0; //设置立即报告更改
      plan_update_velocity_profile_parameters();
      plan_cycle_reinitialize();
    }
  }

  rt_exec = sys_rt_exec_accessory_override;//主轴/冷却液覆盖的全局实时执行程序位标志变量
  if (rt_exec) {
    system_clear_exec_accessory_overrides(); //清除所有配件覆盖标志。

    //注意：与运动覆盖不同，主轴覆盖不需要规划器重新初始化。
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
      sys.report_ovr_counter = 0; //设置立即报告更改
    }

    if (rt_exec & EXEC_SPINDLE_OVR_STOP) {
        //只有在保持状态下才允许主轴停止超驰。
        //注意：执行主轴停止时，报告计数器在spindle_set_state（）中设置。
      if (sys.state == STATE_HOLD) {
        if (!(sys.spindle_stop_ovr)) { sys.spindle_stop_ovr = SPINDLE_STOP_OVR_INITIATE; }
        else if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_ENABLED) { sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE; }
      }
    }

    //注意：由于冷却剂状态总是在计划器同步时执行，当前
    //运行状态可以通过检查解析器状态来确定。
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
        coolant_set_state(coolant_state); //在coolant_set_state（）中设置报告计数器。
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

    //重新加载段缓冲区
  if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_SAFETY_DOOR | STATE_HOMING | STATE_SLEEP| STATE_JOG)) {
    st_prep_buffer();
  }

}


//处理Grbl系统暂停程序，如进给保持，安全门和停车动作。
//系统将进入该循环，为暂停任务创建局部变量，然后返回
//调用暂停的任何函数，以便Grbl恢复正常操作。
//这个函数的写法是为了促进自定义的停车动作。简单地使用这个作为一个
//模板
static void protocol_exec_rt_suspend()
{
  #ifdef PARKING_ENABLE
	//声明并初始化停放局部变量
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

    //阻止，直到初始保持完成，机器停止运动。
    if (sys.suspend & SUSPEND_HOLD_COMPLETE) {

        //停车场经理。处理de / re-energizing，开关状态检查和停车运动
        //安全门和睡眠状态。
      if (sys.state & (STATE_SAFETY_DOOR | STATE_SLEEP)) {
      
          //处理回缩运动和断电。
        if (bit_isfalse(sys.suspend,SUSPEND_RETRACT_COMPLETE)) {

            //确保在开始安全门程序时禁用任何先前的主轴停止覆盖。
          sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED;

          #ifndef PARKING_ENABLE

            spindle_set_state(SPINDLE_DISABLE,0.0f); // 断电
            coolant_set_state(COOLANT_DISABLE);     // 断电

          #else
					
            //获取当前位置并存储恢复位置和主轴收回航点。
            system_convert_array_steps_to_mpos(parking_target,sys_position);
            if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
              memcpy(restore_target,parking_target,sizeof(parking_target));
              retract_waypoint += restore_target[PARKING_AXIS];
              retract_waypoint = min(retract_waypoint,PARKING_TARGET);
            }

            //执行慢速停车收放动作。停车需要启用导航，
            //当前位置不超过停车目标位置，禁用激光模式。
            //注意：状态将保持DOOR状态，直到完全断电和收回。
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
							//通过拉出距离收回主轴。确保回缩运动离开
							//工件和航路点运动不超过停车目标位置。
              if (parking_target[PARKING_AXIS] < retract_waypoint) {
                parking_target[PARKING_AXIS] = retract_waypoint;
                pl_data->feed_rate = PARKING_PULLOUT_RATE;
                pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); //保留配件状态
                pl_data->spindle_speed = restore_spindle_speed;
                mc_parking_motion(parking_target, pl_data);
              }

              //注意：在收回和中止恢复动作之后清除附件状态。
              pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
              pl_data->spindle_speed = 0.0f;
              spindle_set_state(SPINDLE_DISABLE,0.0f); // 断电
              coolant_set_state(COOLANT_DISABLE); // 断电

              // 执行快速停车撤回运动到停车目标位置。
              if (parking_target[PARKING_AXIS] < PARKING_TARGET) {
                parking_target[PARKING_AXIS] = PARKING_TARGET;
                pl_data->feed_rate = PARKING_RATE;
                mc_parking_motion(parking_target, pl_data);
              }

            } else {

              // 停车不可行 只需禁用主轴和冷却液。
              // 注意：激光模式不启动停放动作以确保激光立即停止。
              spindle_set_state(SPINDLE_DISABLE,0.0f); // 断电
              coolant_set_state(COOLANT_DISABLE);     // 断电

            }

          #endif

          sys.suspend &= ~(SUSPEND_RESTART_RETRACT);
          sys.suspend |= SUSPEND_RETRACT_COMPLETE;

        } else {

          
          if (sys.state == STATE_SLEEP) {
            report_feedback_message(MESSAGE_SLEEP_MODE);
            //主轴和冷却液应该已经停止了，但是要确认一下。
            spindle_set_state(SPINDLE_DISABLE,0.0f); // 断电
            coolant_set_state(COOLANT_DISABLE); // 断电
            st_go_idle(); // 禁用步进器
            while (!(sys.abort)) { protocol_exec_rt_system(); } // 不要做任何事，直到重置。
            return; // 中止收到。返回重新初始化。
          }    
          
          //允许从停车/安全门恢复。积极检查安全门是否关闭并准备恢复。
          if (sys.state == STATE_SAFETY_DOOR) {
            if (!(system_check_safety_door_ajar())) {
              sys.suspend &= ~(SUSPEND_SAFETY_DOOR_AJAR); //重置门小面旗表示准备恢复。
            }
          }

          //处理停车恢复和安全门继续。
          if (sys.suspend & SUSPEND_INITIATE_RESTORE) {

            #ifdef PARKING_ENABLE
              //执行快速恢复运动到拉出位置。停车需要启用回原点。
              //注意：状态将保持DOOR状态，直到完全断电和收回。
							#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
							if (((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) &&
									 (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
							#else
							if ((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
							#endif
				//检查以确保运动不会在拉出位置之下移动。
                if (parking_target[PARKING_AXIS] <= PARKING_TARGET) {
                  parking_target[PARKING_AXIS] = retract_waypoint;
                  pl_data->feed_rate = PARKING_RATE;
                  mc_parking_motion(parking_target, pl_data);
                }
              }
            #endif

			//延迟任务：重新启动主轴和冷却液，延时上电，然后重新开始循环。
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              // 在先前的恢复操作中，如果安全门重新打开，则阻止。
              if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
                if (bit_istrue(settings.flags,BITFLAG_LASER_MODE)) {
                  // 在激光模式下，忽略主轴旋转延迟。设定在循环开始时打开激光。
                  bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
                } else {
                  spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
                  delay_sec(SAFETY_DOOR_SPINDLE_DELAY, DELAY_MODE_SYS_SUSPEND);
                }
              }
            }
            if (gc_state.modal.coolant != COOLANT_DISABLE) {
              // 在先前的恢复操作中，如果安全门重新打开，则阻止。
              if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
                // 注意：激光模式将遵守这个延迟。排气系统通常由此引脚控制。
                coolant_set_state((restore_condition & (PL_COND_FLAG_COOLANT_FLOOD | PL_COND_FLAG_COOLANT_FLOOD)));
                delay_sec(SAFETY_DOOR_COOLANT_DELAY, DELAY_MODE_SYS_SUSPEND);
              }
            }

            #ifdef PARKING_ENABLE
              // 从拉出位置执行缓慢的插入动作以恢复位置。
						#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
						if (((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) &&
									(sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
							#else
							if ((settings.flags & (BITFLAG_HOMING_ENABLE | BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
							#endif
                // 在先前的恢复操作中，如果安全门重新打开，则阻止。
                if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
                    //无论收回停车是否是有效/安全的动作，
                    //恢复停车议案应逻辑上有效，要么通过返回
                    //通过有效的机器空间或者根本不移动原始位置。
                  pl_data->feed_rate = PARKING_PULLOUT_RATE;
									pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); // Restore accessory state
									pl_data->spindle_speed = restore_spindle_speed;
                  mc_parking_motion(restore_target, pl_data);
                }
              }
            #endif

            if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
              sys.suspend |= SUSPEND_RESTORE_COMPLETE;
              system_set_exec_state_flag(EXEC_CYCLE_START); //设置为恢复程序。
            }
          }

        }


      } else {

          // Feed保持管理器。控制主轴停止超驰状态。
          //注意：在暂停程序开始时通过条件检查完成保持。
        if (sys.spindle_stop_ovr) {
        	//处理主轴停止的开始
          if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_INITIATE) {
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              spindle_set_state(SPINDLE_DISABLE,0.0f); //断电
              sys.spindle_stop_ovr = SPINDLE_STOP_OVR_ENABLED; //将停止覆盖状态设置为启用（如果断电）。
            } else {
              sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED; //清除停止覆盖状态
            }
            //处理主轴状态的恢复
          } else if (sys.spindle_stop_ovr & (SPINDLE_STOP_OVR_RESTORE | SPINDLE_STOP_OVR_RESTORE_CYCLE)) {
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              report_feedback_message(MESSAGE_SPINDLE_RESTORE);
              if (bit_istrue(settings.flags,BITFLAG_LASER_MODE)) {
            	  //在激光模式下，忽略主轴旋转延迟。设定在循环开始时打开激光。
                bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
              } else {
                spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
              }
            }
            if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_RESTORE_CYCLE) {
              system_set_exec_state_flag(EXEC_CYCLE_START);  //设置为恢复程序。
            }
            sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED; //清除停止覆盖状态
          }
        } else {
            //在保持期间处理主轴状态。注意：在保持状态下，主轴转速改写可能会改变。
            //注意：STEP_CONTROL_UPDATE_SPINDLE_PWM在步骤生成器中恢复后自动复位。
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
