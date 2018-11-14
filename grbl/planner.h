/*
  planner.h - buffers movement commands and manages the acceleration profile plan
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

#ifndef planner_h
#define planner_h


// The number of linear motions that can be in the plan at any give time
#ifndef BLOCK_BUFFER_SIZE
	#define BLOCK_BUFFER_SIZE 36
#endif

// Returned status message from planner.
#define PLAN_OK true
#define PLAN_EMPTY_BLOCK false  //不需要移动

// 定义计划程序数据条件标志。 用于表示块的运行条件。
#define PL_COND_FLAG_RAPID_MOTION      bit(0)
#define PL_COND_FLAG_SYSTEM_MOTION     bit(1) // Single motion. Circumvents planner state. Used by home/park.
#define PL_COND_FLAG_NO_FEED_OVERRIDE  bit(2) // Motion does not honor feed override.
#define PL_COND_FLAG_INVERSE_TIME      bit(3) // Interprets feed rate value as inverse time when set.
#define PL_COND_FLAG_SPINDLE_CW        bit(4)
#define PL_COND_FLAG_SPINDLE_CCW       bit(5)
#define PL_COND_FLAG_COOLANT_FLOOD     bit(6)
#define PL_COND_FLAG_COOLANT_MIST      bit(7)
#define PL_COND_MOTION_MASK    (PL_COND_FLAG_RAPID_MOTION|PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE)
#define PL_COND_ACCESSORY_MASK (PL_COND_FLAG_SPINDLE_CW|PL_COND_FLAG_SPINDLE_CCW|PL_COND_FLAG_COOLANT_FLOOD|PL_COND_FLAG_COOLANT_MIST)


// 该结构存储g代码块运动的线性运动及其关键的“标称”值
// 如源代码g-code中所指定。
typedef struct {
  // bresenham算法用于跟踪线的字段
  // 注意：由步进算法用于正确执行块。 不要改变这些值。
  uint32_t steps[N_AXIS];    // 每个坐标轴所需走的脉冲数
  uint32_t step_event_count; // 所有轴中走最多脉冲数的
  uint8_t direction_bits;    // 每一个位代表一个轴的方向 

  // 阻止条件数据以确保根据状态和覆盖正确执行。
  uint8_t condition;      // 块位标志变量定义块运行条件。 从pl_line_data复制。
  #ifdef USE_LINE_NUMBERS
    int32_t line_number;  // Block line number for real-time reporting. Copied from pl_line_data.
  #endif

  // 运动规划器用于管理加速度的字段。 其中一些值可能会更新
  // 在执行特殊运动情况期间由步进模块进行重新计划。
  float entry_speed_sqr;     // 进入速度，即从上一个block进入到这个block的速度
  float max_entry_speed_sqr; // 最大进入速度，进入速度不能超过这个值
                           
  float acceleration;        // 加速度，单位mm/sec^2
  float millimeters;         // 该块的剩余距离以（mm）为单位执行。
                             // 注意：执行期间，步进算法可能会更改此值。

  // 计划程序在发生更改时使用的存储速率限制数据。
  float max_junction_speed_sqr; // 基于（mm / min）^ 2的方向矢量的结点进入速度限制
  float rapid_rate;             // 该块方向的轴限制调整最大速率（mm / min）
  float programmed_rate;        // 该程序段的编程速率（mm / min）。

  #ifdef VARIABLE_SPINDLE
    // 主轴覆盖和恢复方法使用的存储主轴速度数据。
    float spindle_speed;    // 块的主轴转速。 从pl_line_data复制。
  #endif
} plan_block_t;


// 规划师数据原型。 将新动作传递给计划者时必须使用。
typedef struct {
  float feed_rate;          // 线运动所需的进给速率。 如果快速运动，则忽略值。
  float spindle_speed;      // 通过线运动获得所需的主轴速度。
  uint8_t condition;        // 位标志变量表示计划员条件。 见上文定义。
  #ifdef USE_LINE_NUMBERS
    int32_t line_number;    // Desired line number to report when executing.
  #endif
} plan_line_data_t;


// Initialize and reset the motion plan subsystem
void plan_reset(); // Reset all
void plan_reset_buffer(); // Reset buffer only.

// Add a new linear movement to the buffer. target[N_AXIS] is the signed, absolute target position
// in millimeters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
uint8_t plan_buffer_line(float *target, plan_line_data_t *pl_data);

// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
void plan_discard_current_block();

// Gets the planner block for the special system motion cases. (Parking/Homing)
plan_block_t *plan_get_system_motion_block();

// Gets the current block. Returns NULL if buffer empty
plan_block_t *plan_get_current_block();

// Called periodically by step segment buffer. Mostly used internally by planner.
uint8_t plan_next_block_index(uint8_t block_index);

// Called by step segment buffer when computing executing block velocity profile.
float plan_get_exec_block_exit_speed_sqr();

// Called by main program during planner calculations and step segment buffer during initialization.
float plan_compute_profile_nominal_speed(plan_block_t *block);

// Re-calculates buffered motions profile parameters upon a motion-based override change.
void plan_update_velocity_profile_parameters();

// Reset the planner position vector (in steps)
void plan_sync_position();

// Reinitialize plan with a partially completed block
void plan_cycle_reinitialize();

// Returns the number of available blocks are in the planner buffer.
uint8_t plan_get_block_buffer_available();

// Returns the number of active blocks are in the planner buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h
uint8_t plan_get_block_buffer_count();

// Returns the status of the block ring buffer. True, if buffer is full.
uint8_t plan_check_full_buffer();

void plan_get_planner_mpos(float *target);


#endif
