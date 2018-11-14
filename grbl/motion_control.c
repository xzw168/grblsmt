/*
  motion_control.c - 用于发出运动命令的高级接口
  插补和运动控制流程
  包括：

  1.直线插补

  2.圆弧插补

  3.运动延时

  4.回原运动

  5.探针运动

  6.特殊的主轴退刀停机运动

  7.复位运动控制状态
*/

#include "grbl.h"


//以绝对毫米坐标执行线性运动。 进料速率以毫米/秒为单位
//除非invert_feed_rate为true。 然后feed_rate意味着应该完成动作
//（1分钟）/ feed_rate时间。
//注意：这是grbl规划器的主要网关。 所有直线运动，包括弧线
//段，必须在传递给规划器之前通过此例程。 分开
// mc_line和plan_buffer_line主要用于放置非计划类型的函数
//在计划器中，让间隙补偿或固定循环集成简单直接。
void mc_line(float *target, plan_line_data_t *pl_data)
{
	//如果启用，请检查软限制违规。 放在这里所有的线条动作都被拾起
	//来自Grbl的所有地方。
  if (bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE)) {
    // 注意：阻止慢跑状态。 慢跑是一种特殊情况，软限制是独立处理的。
    if (sys.state != STATE_JOG) { limits_soft_check(target); }
  }

  // 如果处于检查gcode模式，请阻止计划程序阻止运动。 软限制仍然有效。
  if (sys.state == STATE_CHECK_MODE) { return; }

  //如果缓冲区已满：好！这意味着我们远远领先于机器人。
  //保持在此循环中，直到缓冲区中有空间。
  do {
    protocol_execute_realtime(); // 检查任何运行时命令
    if (sys.abort) { return; } // 保释，如果系统中止。
    if ( plan_check_full_buffer() ) { protocol_auto_cycle_start(); } // 缓冲区已满时自动循环启动。
    else { break; }
  } while (1);

  // 计划并将运动排入计划缓冲区
	if (plan_buffer_line(target, pl_data) == PLAN_EMPTY_BLOCK) {
		if (bit_istrue(settings.flags, BITFLAG_LASER_MODE)) {
			// 如果有重合位置，则正确设置主轴状态。 强制缓冲
			// 仅在M3激光模式下同步。
			if (pl_data->condition & PL_COND_FLAG_SPINDLE_CW) {
				spindle_sync(PL_COND_FLAG_SPINDLE_CW, pl_data->spindle_speed);
			}
		}
	}
}


//以偏移模式格式执行弧。 position == current xyz，target == target xyz，
// offset ==偏离当前xyz，axis_X定义工具空间中的圆平面，axis_linear是
//螺旋行程的方向，半径==圆半径，isclockwise布尔值。 用过的
//用于矢量转换方向。
//通过生成大量细小的线性段来近似弧。 弦宽容
//每个段的//在settings.arc_tolerance中配置，定义为最大法线
//当终点都位于圆上时，从段到圆的距离。
void mc_arc(float *target, plan_line_data_t *pl_data, float *position, float *offset, float radius,
  uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc)
{
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float r_axis0 = -offset[axis_0];  // Radius vector from center to current location
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;

  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2f(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (is_clockwise_arc) { // Correct atan2 output per direction
    if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel -= 2*M_PI; }
  } else {
    if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel += 2*M_PI; }
  }

  // NOTE: Segment end points are on the arc, which can lead to the arc diameter being smaller by up to
  // (2x) settings.arc_tolerance. For 99% of users, this is just fine. If a different arc segment fit
  // is desired, i.e. least-squares, midpoint on arc, just change the mm_per_arc_segment calculation.
  // For the intended uses of Grbl, this value shouldn't exceed 2000 for the strictest of cases.
  uint16_t segments = (uint16_t)floorf(fabsf(0.5f*angular_travel*radius) /
                          sqrtf(settings.arc_tolerance*(2*radius - settings.arc_tolerance)) );

  if (segments) {
    // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
    // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
    // all segments.
    if (pl_data->condition & PL_COND_FLAG_INVERSE_TIME) { 
      pl_data->feed_rate *= segments; 
      bit_false(pl_data->condition,PL_COND_FLAG_INVERSE_TIME); // Force as feed absolute mode over arc segments.
    }
    
    float theta_per_segment = angular_travel/segments;
    float linear_per_segment = (target[axis_linear] - position[axis_linear])/segments;

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
       and phi is the angle of rotation. Solution approach by Jens Geisler.
           r_T = [cos(phi) -sin(phi);
                  sin(phi)  cos(phi] * r ;

       For arc generation, the center of the circle is the axis of rotation and the radius vector is
       defined from the circle center to the initial position. Each line segment is formed by successive
       vector rotations. Single precision values can accumulate error greater than tool precision in rare
       cases. So, exact arc path correction is implemented. This approach avoids the problem of too many very
       expensive trig operations [sin(),cos(),tan()] which can take 100-200 usec each to compute.

       Small angle approximation may be used to reduce computation overhead further. A third-order approximation
       (second order sin() has too much error) holds for most, if not, all CNC applications. Note that this
       approximation will begin to accumulate a numerical drift error when theta_per_segment is greater than
       ~0.25 rad(14 deg) AND the approximation is successively used without correction several dozen times. This
       scenario is extremely unlikely, since segment lengths and theta_per_segment are automatically generated
       and scaled by the arc tolerance setting. Only a very large arc tolerance setting, unrealistic for CNC
       applications, would cause this numerical drift error. However, it is best to set N_ARC_CORRECTION from a
       low of ~4 to a high of ~20 or so to avoid trig operations while keeping arc generation accurate.

       This approximation also allows mc_arc to immediately insert a line segment into the planner
       without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
       This is important when there are successive arc motions.
    */
    // Computes: cos_T = 1 - theta_per_segment^2/2, sin_T = theta_per_segment - theta_per_segment^3/6) in ~52usec
    float cos_T = 2.0f - theta_per_segment*theta_per_segment;
    float sin_T = theta_per_segment*0.16666667f*(cos_T + 4.0f);
    cos_T *= 0.5;

    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    uint8_t count = 0;

    for (i = 1; i<segments; i++) { // Increment (segments-1).

      if (count < N_ARC_CORRECTION) {
        // Apply vector rotation matrix. ~40 usec
        r_axisi = r_axis0*sin_T + r_axis1*cos_T;
        r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
        r_axis1 = r_axisi;
        count++;
      } else {
        // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments. ~375 usec
        // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
        cos_Ti = cosf(i*theta_per_segment);
        sin_Ti = sinf(i*theta_per_segment);
        r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
        r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
        count = 0;
      }

      // Update arc_target location
      position[axis_0] = center_axis0 + r_axis0;
      position[axis_1] = center_axis1 + r_axis1;
      position[axis_linear] += linear_per_segment;

      mc_line(position, pl_data);

      // Bail mid-circle on system abort. Runtime command check already performed by mc_line.
      if (sys.abort) { return; }
    }
  }
  // Ensure last segment arrives at target location.
  mc_line(target, pl_data);
}


//在几秒钟内执行停留。
void mc_dwell(float seconds)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize();
  delay_sec(seconds, DELAY_MODE_DWELL);
}


//执行归位循环以定位和设置机器零点。 只有'$ H'执行此命令。
//注意：缓冲区中应该没有运动，Grbl之前必须处于空闲状态
//执行归位循环 这可以防止归位后不正确的缓冲计划。
void mc_homing_cycle(uint8_t cycle_mask)
{
  // Check and abort homing cycle, if hard limits are already enabled. Helps prevent problems
  // with machines with limits wired on both ends of travel to one limit pin.
  // TODO: Move the pin-specific LIMIT_PIN call to limits.c as a function.
  #ifdef LIMITS_TWO_SWITCHES_ON_AXES
    if (limits_get_state()) {
      mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
      system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT);
      return;
    }
  #endif

  limits_disable(); // Disable hard limits pin change register for cycle duration

  // -------------------------------------------------------------------------------------
  // Perform homing routine. NOTE: Special motion case. Only system reset works.
  
  #ifdef HOMING_SINGLE_AXIS_COMMANDS
    if (cycle_mask) { limits_go_home(cycle_mask); } // Perform homing cycle based on mask.
    else
  #endif
  {
    // Search to engage all axes limit switches at faster homing seek rate.
    limits_go_home(HOMING_CYCLE_0);  // Homing cycle 0
    #ifdef HOMING_CYCLE_1
      limits_go_home(HOMING_CYCLE_1);  // Homing cycle 1
    #endif
    #ifdef HOMING_CYCLE_2
      limits_go_home(HOMING_CYCLE_2);  // Homing cycle 2
    #endif
  }

  protocol_execute_realtime(); // Check for reset and set system abort.
  if (sys.abort) { return; } // Did not complete. Alarm state set by mc_alarm.

  // Homing cycle complete! Setup system for normal operation.
  // -------------------------------------------------------------------------------------

  // Sync gcode parser and planner positions to homed position.
  gc_sync_position();
  plan_sync_position();

  // If hard limits feature enabled, re-enable hard limits pin change register after homing cycle.
#ifdef STM32F103C8
	EXTI_ClearITPendingBit((1 << X_LIMIT_BIT) | (1 << Y_LIMIT_BIT) | (1 << Z_LIMIT_BIT));
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
#else
	limits_init();
#endif
}


//执行刀具长度探针循环。 需要探头开关。
//注意：探测失败时，程序将停止并进入ALARM状态。
uint8_t mc_probe_cycle(float *target, plan_line_data_t *pl_data, uint8_t parser_flags)
{
  // TODO: Need to update this cycle so it obeys a non-auto cycle start.
  if (sys.state == STATE_CHECK_MODE) { return(GC_PROBE_CHECK_MODE); }

  // Finish all queued commands and empty planner buffer before starting probe cycle.
  protocol_buffer_synchronize();
  if (sys.abort) { return(GC_PROBE_ABORT); } // Return if system reset has been issued.

  // Initialize probing control variables
  uint8_t is_probe_away = bit_istrue(parser_flags, GC_PARSER_PROBE_IS_AWAY);
  uint8_t is_no_error = bit_istrue(parser_flags, GC_PARSER_PROBE_IS_NO_ERROR);
  sys.probe_succeeded = false; // Re-initialize probe history before beginning cycle.
  probe_configure_invert_mask(is_probe_away);

  // After syncing, check if probe is already triggered. If so, halt and issue alarm.
  // NOTE: This probe initialization error applies to all probing cycles.
  if ( probe_get_state() ) { // Check probe pin state.
    system_set_exec_alarm(EXEC_ALARM_PROBE_FAIL_INITIAL);
    protocol_execute_realtime();
    probe_configure_invert_mask(false); // Re-initialize invert mask before returning.
    return(GC_PROBE_FAIL_INIT); // Nothing else to do but bail.
  }

  // Setup and queue probing motion. Auto cycle-start should not start the cycle.
  mc_line(target, pl_data);

  // Activate the probing state monitor in the stepper module.
  sys_probe_state = PROBE_ACTIVE;

  // Perform probing cycle. Wait here until probe is triggered or motion completes.
  system_set_exec_state_flag(EXEC_CYCLE_START);
  do {
    protocol_execute_realtime();
    if (sys.abort) { return(GC_PROBE_ABORT); } // Check for system abort
  } while (sys.state != STATE_IDLE);

  // Probing cycle complete!

  // Set state variables and error out, if the probe failed and cycle with error is enabled.
  if (sys_probe_state == PROBE_ACTIVE) {
    if (is_no_error) { memcpy(sys_probe_position, sys_position, sizeof(sys_position)); }
    else { system_set_exec_alarm(EXEC_ALARM_PROBE_FAIL_CONTACT); }
  } else {
    sys.probe_succeeded = true; // Indicate to system the probing cycle completed successfully.
  }
  sys_probe_state = PROBE_OFF; // Ensure probe state monitor is disabled.
  probe_configure_invert_mask(false); // Re-initialize invert mask.
  protocol_execute_realtime();   // Check and execute run-time commands

  // Reset the stepper and planner buffers to remove the remainder of the probe motion.
  st_reset(); // Reset step segment buffer.
  plan_reset(); // Reset planner buffer. Zero planner positions. Ensure probing motion is cleared.
  plan_sync_position(); // Sync planner position to current machine position.

  #ifdef MESSAGE_PROBE_COORDINATES
    // All done! Output the probe position as message.
    report_probe_parameters();
  #endif

  if (sys.probe_succeeded) { return(GC_PROBE_FOUND); } // Successful probe cycle.
  else { return(GC_PROBE_FAIL_END); } // Failed to trigger probe within travel. With or without error.
}

#ifdef PARKING_ENABLE
	void mc_parking_motion(float *parking_target, plan_line_data_t *pl_data)
	{
		if (sys.abort) { return; } // Block during abort.

		uint8_t plan_status = plan_buffer_line(parking_target, pl_data);

		if (plan_status) {
			bit_true(sys.step_control, STEP_CONTROL_EXECUTE_SYS_MOTION);
			bit_false(sys.step_control, STEP_CONTROL_END_MOTION); // Allow parking motion to execute, if feed hold is active.
			st_parking_setup_buffer(); // Setup step segment buffer for special parking motion case
			st_prep_buffer();
			st_wake_up();
			do {
				protocol_exec_rt_system();
				if (sys.abort) { return; }
			} while (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION);
			st_parking_restore_buffer(); // Restore step segment buffer to normal run state.
		}
		else {
			bit_false(sys.step_control, STEP_CONTROL_EXECUTE_SYS_MOTION);
			protocol_exec_rt_system();
		}

	}
#endif


#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
void mc_override_ctrl_update(uint8_t override_state)
{
	// Finish all queued commands before altering override control state
	protocol_buffer_synchronize();
	if (sys.abort) { return; }
	sys.override_ctrl = override_state;
}
#endif
//通过设置realtime reset命令并杀死任何命令来使系统准备好重置的方法
//系统中的活动进程 这还检查在Grbl时是否发出系统复位
//处于运动状态。 如果是，则杀死步进器并将系统警报设置为标记位置
//输了，因为有一个突然失控的减速。 被称为中断级别
//实时中止命令和硬限制。 所以，保持最低限度。
void mc_reset()
{
  // Only this function can set the system reset. Helps prevent multiple kill calls.
  if (bit_isfalse(sys_rt_exec_state, EXEC_RESET)) {
    system_set_exec_state_flag(EXEC_RESET);

    // Kill spindle and coolant.
    spindle_stop();
    coolant_stop();

    // Kill steppers only if in any motion state, i.e. cycle, actively holding, or homing.
    // NOTE: If steppers are kept enabled via the step idle delay setting, this also keeps
    // the steppers enabled by avoiding the go_idle call altogether, unless the motion state is
    // violated, by which, all bets are off.
    if ((sys.state & (STATE_CYCLE | STATE_HOMING | STATE_JOG)) ||
    		(sys.step_control & (STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION))) {
      if (sys.state == STATE_HOMING) {
        if (!sys_rt_exec_alarm) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_RESET); }
      }
      else { system_set_exec_alarm(EXEC_ALARM_ABORT_CYCLE); }
      st_go_idle(); // Force kill steppers. Position has likely been lost.
    }
  }
}
