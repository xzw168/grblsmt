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
#define PLAN_EMPTY_BLOCK false  //����Ҫ�ƶ�

// ����ƻ���������������־�� ���ڱ�ʾ�������������
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


// �ýṹ�洢g������˶��������˶�����ؼ��ġ���ơ�ֵ
// ��Դ����g-code����ָ����
typedef struct {
  // bresenham�㷨���ڸ����ߵ��ֶ�
  // ע�⣺�ɲ����㷨������ȷִ�п顣 ��Ҫ�ı���Щֵ��
  uint32_t steps[N_AXIS];    // ÿ�������������ߵ�������
  uint32_t step_event_count; // ���������������������
  uint8_t direction_bits;    // ÿһ��λ����һ����ķ��� 

  // ��ֹ����������ȷ������״̬�͸�����ȷִ�С�
  uint8_t condition;      // ��λ��־������������������� ��pl_line_data���ơ�
  #ifdef USE_LINE_NUMBERS
    int32_t line_number;  // Block line number for real-time reporting. Copied from pl_line_data.
  #endif

  // �˶��滮�����ڹ�����ٶȵ��ֶΡ� ����һЩֵ���ܻ����
  // ��ִ�������˶�����ڼ��ɲ���ģ��������¼ƻ���
  float entry_speed_sqr;     // �����ٶȣ�������һ��block���뵽���block���ٶ�
  float max_entry_speed_sqr; // �������ٶȣ������ٶȲ��ܳ������ֵ
                           
  float acceleration;        // ���ٶȣ���λmm/sec^2
  float millimeters;         // �ÿ��ʣ������ԣ�mm��Ϊ��λִ�С�
                             // ע�⣺ִ���ڼ䣬�����㷨���ܻ���Ĵ�ֵ��

  // �ƻ������ڷ�������ʱʹ�õĴ洢�����������ݡ�
  float max_junction_speed_sqr; // ���ڣ�mm / min��^ 2�ķ���ʸ���Ľ������ٶ�����
  float rapid_rate;             // �ÿ鷽��������Ƶ���������ʣ�mm / min��
  float programmed_rate;        // �ó���εı�����ʣ�mm / min����

  #ifdef VARIABLE_SPINDLE
    // ���Ḳ�Ǻͻָ�����ʹ�õĴ洢�����ٶ����ݡ�
    float spindle_speed;    // �������ת�١� ��pl_line_data���ơ�
  #endif
} plan_block_t;


// �滮ʦ����ԭ�͡� ���¶������ݸ��ƻ���ʱ����ʹ�á�
typedef struct {
  float feed_rate;          // ���˶�����Ľ������ʡ� ��������˶��������ֵ��
  float spindle_speed;      // ͨ�����˶��������������ٶȡ�
  uint8_t condition;        // λ��־������ʾ�ƻ�Ա������ �����Ķ��塣
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
