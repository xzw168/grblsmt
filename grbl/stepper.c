

/*
  stepper.c - 步进电机驱动器：使用步进电机执行运动计划
*/

#include "grbl.h"

#ifdef STM32F103C8
typedef int bool;
#include "stm32f10x_rcc.h"
//#include "stm32f10x_tim.h"
#include "misc.h"
void TIM_Configuration(TIM_TypeDef* TIMER, u16 Period, u16 Prescaler, u8 PP);
#endif


// 一些有用的常量.
#define DT_SEGMENT (1.0f/(ACCELERATION_TICKS_PER_SECOND*60.0f)) // 分钟/段
#define REQ_MM_INCREMENT_SCALAR 1.25f
#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2
#define RAMP_DECEL_OVERRIDE 3

#define PREP_FLAG_RECALCULATE bit(0)
#define PREP_FLAG_HOLD_PARTIAL_BLOCK bit(1)
#define PREP_FLAG_PARKING bit(2)
#define PREP_FLAG_DECEL_OVERRIDE bit(3)
const PORTPINDEF step_pin_mask[N_AXIS] =
{
	1 << X_STEP_BIT,
	1 << Y_STEP_BIT,
	1 << Z_STEP_BIT,

};
const PORTPINDEF direction_pin_mask[N_AXIS] =
{
	1 << X_DIRECTION_BIT,
	1 << Y_DIRECTION_BIT,
	1 << Z_DIRECTION_BIT,
};
const PORTPINDEF limit_pin_mask[N_AXIS] =
{
	1 << X_LIMIT_BIT,
	1 << Y_LIMIT_BIT,
	1 << Z_LIMIT_BIT,
};

//定义自适应多轴步进平滑（AMASS）电平和截止频率。最高的水平
//频率开始于0Hz，并以截止频率结束。下一个较低级别的频率仓
//从下一个更高的截止频率开始，依此类推。每个级别的截止频率必须
//要仔细考虑它过度驱动步进器ISR的精度是多少
//定时器和CPU开销。等级0（无AMASS，正常操作）频率仓开始于
//级别1的截止频率，最高可达CPU允许的频率（在有限的测试中超过30kHz）。
//注意：AMASS截止频率乘以ISR过驱动因子不得超过最大步进频率。
//注意：当前设置被设置为将ISR过驱动到不超过16kHz，从而平衡CPU开销
//和计时器的准确性。除非你知道你在做什么，否则不要改变这些设置。
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
	#define MAX_AMASS_LEVEL 3
	// AMASS_LEVEL0: 正常运行。没有AMASS。没有上限截止频率。从LEVEL1截止频率开始。
	#define AMASS_LEVEL1 (F_CPU/8000) // 超载驱动ISR（x2）。定义为F_CPU /（以Hz为单位的截止频率）
	#define AMASS_LEVEL2 (F_CPU/4000) // Over-drives ISR (x4)
	#define AMASS_LEVEL3 (F_CPU/2000) // Over-drives ISR (x8)

  #if MAX_AMASS_LEVEL <= 0
    error "AMASS must have 1 or more levels to operate correctly."
  #endif
#endif



//存储段中段的规划块Bresenham算法执行数据
//缓冲区。通常情况下，这个缓冲区部分在使用中，但是，对于最坏的情况，它会
//永远不会超过可访问的步进缓冲区段的数量（SEGMENT_BUFFER_SIZE-1）。
//注意：这些数据是从prepped planner块中复制的，这样规划块就可以了
//被段缓冲区完全消耗并完成时丢弃。另外，AMASS改变了这一点
//数据供自己使用。
typedef struct {
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;//步骤事件计数
  uint8_t direction_bits;//方向位
  #ifdef VARIABLE_SPINDLE
    uint8_t is_pwm_rate_adjusted; // 追踪需要恒定激光功率/速率的运动
  #endif
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];

//主步进器段环形缓冲区 包含步进器的小的短线段
//执行的算法，从第一个块开始逐渐“检出”
//计划器缓冲区 一旦“检出”，段缓冲区中的步骤不能被修改
//策划者，剩下的策划者块步骤仍然可以。
typedef struct {
  uint16_t n_step;           // 这个段要执行的步骤事件的数量
  uint16_t cycles_per_tick;  // 每个ISR嘀嗒的步距，也就是步幅。
  uint8_t  st_block_index;   // 步进块数据索引。使用此信息来执行此段。
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;    // 表示ISR执行该段的AMASS级别
  #else
    uint8_t prescaler;      // 没有AMASS，需要预分频器来调整慢时序。
  #endif
  #ifdef VARIABLE_SPINDLE
    uint8_t spindle_pwm;
  #endif
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// 步进ISR数据结构。包含主步进器ISR的运行数据。
typedef struct {
  // 由bresenham线算法使用
  uint32_t counter_x,        // 用于bresenham线跟踪器的计数器变量
           counter_y,
           counter_z;
  #ifdef STEP_PULSE_DELAY
    uint8_t step_bits;  // 存储out_bits输出以完成阶跃脉冲延迟
  #endif

  uint8_t execute_step;     // 标志为每个中断执行一次。

  uint8_t step_pulse_time;  // 步进上升后的步进脉冲复位时间

  PORTPINDEF step_outbits;         // 输出下一个步进位
  PORTPINDEF dir_outbits;
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
  #endif

  uint16_t step_count;       // 线段运动中剩余的步骤
  uint8_t exec_block_index; // 跟踪当前的st_block索引。更改表示新块。
  st_block_t *exec_block;   // 指向正在执行的段的块数据的指针
  segment_t *exec_segment;  // 指向正在执行的段的指针
} stepper_t;
static stepper_t st;

//步进段环形缓冲区索引. Step segment ring buffer indices
static volatile uint8_t segment_buffer_tail; //段缓冲尾
static uint8_t segment_buffer_head;//段缓冲头
static uint8_t segment_next_head;

//步进和方向端口反转掩码  Step and direction port invert masks.
static PORTPINDEF step_port_invert_mask;
static PORTPINDEF dir_port_invert_mask;

// 用于避免ISR嵌套“步进驱动器中断”。 应该永远不会发生。
static volatile uint8_t busy;

// 指定步骤片段的指针从规划器缓冲区准备好。只有通过访问
// 主程序 指针可能正在计划执行的部分或计划块.
static plan_block_t *pl_block;     // 指向正在准备的计划程序块的指针
static st_block_t *st_prep_block;  // 指向正在准备的步进块数据的指针

// 段准备数据结构。包含计算新细分的所有必要信息
// 基于当前正在执行的计划程序块。
typedef struct {
  uint8_t st_block_index;  // 准备好步进通用数据块的索引
  uint8_t recalculate_flag;

  float dt_remainder;
  float steps_remaining;
  float step_per_mm;
  float req_mm_increment;

  #ifdef PARKING_ENABLE //停车使能
    uint8_t last_st_block_index;
    float last_steps_remaining;
    float last_step_per_mm;
    float last_dt_remainder;
  #endif

  uint8_t ramp_type;      // 当前的段斜坡状态
  float mm_complete;      // 以当前计划程序块结束的速度曲线结束（mm）。
                          // 注意：这个值在转换时必须与一个步骤（无尾数）重合。
  float current_speed;    // 段缓冲区末尾的当前速度（mm / min）
  float maximum_speed;    // 执行块的最大速度。并不总是标称速度。（毫米/分钟）
  float exit_speed;       // 执行程序段的退出速度（mm / min）
  float accelerate_until; // 从程序段末端开始测量的加速斜坡终点（mm）
  float decelerate_after; // 从程序段结束起测量的减速斜坡开始（mm）

  #ifdef VARIABLE_SPINDLE
    float inv_rate;    // 由PWM激光模式使用来加速分段计算。
    uint8_t current_spindle_pwm;
  #endif
} st_prep_t;
static st_prep_t prep;


/*    块速度轮廓定义
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            ^      BLOCK 2          |    d
                                       |
                  时间 ----->      例如：方块2进入速度是最大结点速度

  计划程序块缓冲区计划假设恒定加速度速度配置文件和
  如上所示连续加入区块路口。但是，策划者只能积极计算
  块入口速度为最佳速度计划，但不计算块内部
  速度曲线。这些速度曲线是由它们执行的专门计算的
  步进算法，并且只包含7种可能的类型：巡航，巡航 -
  减速，加速巡航，加速度，仅减速，全梯形和
  三角形（没有巡航）。

                                                                                                                                                                        最大速度 (< 额定速度) ->  +
                    +--------+ <- 最大速度 (= 额定速度)                          /|\
                   /          \                                           / | \
                         当前速度    -> +            \                                         /  |  + <- exit_speed
                  |             + <- exit_speed                         /   |  |
                  +-------------+                             当前速度  -> +----+--+
                   time -->  ^  ^                                           ^  ^
                             |  |                                           |  |
                                                                                减速后(毫米为单位)                                  减速后(毫米为单位)
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                                                              加速直到(毫米为单位)                                        加速直到(毫米为单位)

  步段缓冲器计算执行块速度曲线并跟踪关键点
  步进算法的参数来准确地跟踪轮廓。这些关键参数
  在上图中显示和定义。
*/


//步进状态初始化 周期应该只在st.cycle_start标志是时才开始
//启用。启动init和限制调用这个函数，但是不应该启动这个循环。
void st_wake_up()
{
  // 启用步进驱动程序。
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) 
  { 
	  SetStepperDisableBit();
  }
  else 
  { 
	  ResetStepperDisableBit(); 
  }

  // 初始化步进输出位，以确保第一个ISR调用不会执行。
  st.step_outbits = step_port_invert_mask;

  // 从设置初始化步进脉冲计时。这里确保重写后更新。
  #ifdef STEP_PULSE_DELAY
    // 设置方向引脚后设置总步进脉冲时间。示波器的特殊计算。
    st.step_pulse_time = -(((settings.pulse_microseconds+STEP_PULSE_DELAY-2)*TICKS_PER_MICROSECOND) >> 3);
    // 设置方向引脚写入和步进命令之间的延迟。
    OCR0A = -(((settings.pulse_microseconds)*TICKS_PER_MICROSECOND) >> 3);
  #else // 正常运行
    // 设置步脉冲时间 示波器的特殊计算。使用二补。

#if defined(STM32F103C8)
  st.step_pulse_time = (settings.pulse_microseconds)*TICKS_PER_MICROSECOND;
#endif
  #endif

  // 启用步进驱动器中断

#if defined (STM32F103C8)
  TIM3->ARR = st.step_pulse_time - 1;
  TIM3->EGR = TIM_PSCReloadMode_Immediate;
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

  TIM2->ARR = st.exec_segment->cycles_per_tick - 1;
  /* 设置自动重载值 */
#ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING        
  TIM2->PSC = st.exec_segment->prescaler;
#endif
  TIM2->EGR = TIM_PSCReloadMode_Immediate;
  NVIC_EnableIRQ(TIM2_IRQn);
#endif
}


// 步进关机
void st_go_idle()
{
  // 禁用步进驱动器中断。如果激活，允许步进器端口复位中断完成。

#ifdef STM32F103C8
  NVIC_DisableIRQ(TIM2_IRQn);
#endif

  busy = false;

  // 根据设置和情况设置步进驱动器空闲状态，禁用或启用。
  bool pin_state = false; // 保持启用状态
  if (((settings.stepper_idle_lock_time != 0xff) || sys_rt_exec_alarm || sys.state == STATE_SLEEP) && sys.state != STATE_HOMING) {
    // 强制步进器暂停锁定轴一定的时间，以确保轴完整
    // 在最后一次移动结束时停止并且不会从剩余惯性力漂移。
    delay_ms(settings.stepper_idle_lock_time);
    pin_state = true; // 覆盖 禁用步进器。
  }
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { pin_state = !pin_state; } //应用引脚反转
  if (pin_state) 
  { 
	  SetStepperDisableBit();
  }
  else 
  { 
	  ResetStepperDisableBit();
  }
}


/*
 “步进驱动器中断” - 这个定时器中断是Grbl的主力。Grbl雇佣
   历史悠久的Bresenham线算法来管理和精确同步多轴移动。
   与流行的DDA算法不同，Bresenham算法不易受数值影响
   四舍五入错误，只需要快速整数计数器，这意味着低计算开销
   并最大化Arduino的功能。然而，Bresenham算法的缺点
   对于某些多轴运动而言，非主轴可能遭受不平滑的步进
   脉冲火车或锯齿，这可能会导致奇怪的声音或震动。这是
   特别明显的或可能导致低步频率（0-5kHz）的运动??问题，但是
   通常不是高频的物理问题，虽然可以听到。
     为了提高Bresenham的多轴性能，Grbl使用我们所说的自适应多轴
   步骤平滑（AMASS）算法，它的名字意味着什么。在较低的步频下，
   AMASS人为地增加了Bresenham分辨率而不影响算法
   天生的正确性。AMASS根据步骤自动调整其分辨率级别
   要执行的频率，意味着对于甚至更低的步进频率，步进平滑
   水平提高。从算法上讲，AMASS是通过Bresenham的简单位移来实现的
   每个AMASS级别的步数。例如，对于一级平滑，我们位移
   Bresenham步骤事件计数，有效地乘以2，而轴步数
   保持不变，然后加倍步进ISR频率。实际上，我们正在允许的
   非显性Bresenham坐标轴在中间ISR刻度线上，而主导坐标轴是
   每隔两个ISR报价，而不是传统意义上的每个ISR报价。在AMASS
   等级2，我们只是再次移位，所以非主导的Bresenham轴可以在任何一个步骤内进行
   在四个ISR报价中，主导轴每四个ISR报价一步，四倍
   步进ISR频率。等等。实际上，这实际上消除了多轴混叠
   问题与Bresenham算法并没有显着改变Grbl的表现，但是
   实际上，在所有配置中更高效地利用未使用的CPU周期。
     AMASS通过要求它始终执行完整性来保留Bresenham算法的正确性
   Bresenham一步，不管AMASS的水平。这意味着对于一个AMASS二级，全部四个
   必须完成中间步骤，使基线Bresenham（0级）数始终保持不变
   保留。同样，AMASS Level 3意味着所有八个中间步骤必须被执行。
   尽管AMASS水平实际上是任意的，但Bresenham的基线数据可以
   乘以任何整数值，乘以二的幂只是简单地用来缓解
   带有bitshift整数操作的CPU开销。
     这个中断是简单而愚蠢的设计。所有的计算举重，如在
   确定加速度，在其他地方执行。这个中断弹出预先计算的段，
   定义为在步数缓冲区之后的n个步骤中的恒定速度
   通过Bresenham算法适当的脉冲步进引脚来执行它们。这个
   ISR由Stepper端口复位中断支持，它用来复位步进器端口
   每个脉冲之后。bresenham线示踪算法控制所有步进输出
   与这两个中断同时进行。
   注意：此中断必须尽可能高效，并在下一次ISR打勾之前完成，
   Grbl必须小于33.3usec（@ 30kHz ISR速率）。示波器测量时间
   ISR是典型的5usec和最大25usec，远低于要求。
   注意：这个ISR期望每个段至少执行一个步骤。
*/
// TODO: 以某种方式替换ISR中的int32位置计数器的直接更新。也许使用更小
// int8变量和更新位置计数器只有当一个段完成。这可能变得复杂
// 需要真正的实时位置的探测和归位周期。

void TIM2_IRQHandler(void)
{
#ifdef STM32F103C8
	if ((TIM2->SR & 0x0001) != 0)                  // 查中断源
	{
		TIM2->SR &= ~(1 << 0);                          // 清除UIF标志
		TIM2->CNT = 0;
	}
	else
	{
		return;
	}
#endif

  if (busy) { return; } // 忙标志用来避免重新进入这个中断

#ifdef STM32F103C8
  GPIO_Write(DIRECTION_PORT, (GPIO_ReadOutputData(DIRECTION_PORT) & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK));
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
#endif

  // 然后脉冲步进引脚
  #ifdef STEP_PULSE_DELAY
    st.step_bits = (STEP_PORT & ~STEP_MASK) | st.step_outbits; // 存储out_bits以防止覆盖。
  #else  // 正常工作
#ifdef STM32F103C8
	GPIO_Write(STEP_PORT, (GPIO_ReadOutputData(STEP_PORT) & ~STEP_MASK) | st.step_outbits);
#endif
  #endif

  // 启用步进脉冲复位定时器，使得步进端口复位中断可以在复位后复位信号
  // 精确地设置为.pulse_microseconds微秒，独立于主Timer1预分频器。
#ifdef STM32F103C8
  NVIC_EnableIRQ(TIM3_IRQn);
#endif

  busy = true;

  // 如果没有步骤段，尝试从步进缓冲区中弹出一个
  if (st.exec_segment == NULL) {
    // 缓冲区中的任何东西？如果是这样，加载并初始化下一步段。
    if (segment_buffer_head != segment_buffer_tail) {
      // 初始化新的步骤段并加载要执行的步骤数
      st.exec_segment = &segment_buffer[segment_buffer_tail];

      // 初始化每步的步骤段时间并加载要执行的步骤数。
#ifdef STM32F103C8
	  TIM2->ARR = st.exec_segment->cycles_per_tick - 1;
	  /* 设置自动重载值 */
#ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING        
	  TIM2->PSC = st.exec_segment->prescaler;
#endif
#endif
      st.step_count = st.exec_segment->n_step; // 注意：移动速度有时可能为零。
      // 如果新的段开始一个新的规划块，初始化步进变量和计数器。
      // 注意：当分段数据索引改变时，这表示新的计划程序块。
      if ( st.exec_block_index != st.exec_segment->st_block_index ) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];

        // 初始化Bresenham线和距离计数器
        st.counter_x = st.counter_y = st.counter_z = (st.exec_block->step_event_count >> 1);
      }
      st.dir_outbits = st.exec_block->direction_bits ^ dir_port_invert_mask;

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // 启用AMASS后，根据AMASS等级调整Bresenham轴增量计数器。
        st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.exec_segment->amass_level;
        st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.exec_segment->amass_level;
        st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.exec_segment->amass_level;
      #endif

      #ifdef VARIABLE_SPINDLE
        // 在加载段时设置实时主轴输出，就在第一步之前。
        spindle_set_speed(st.exec_segment->spindle_pwm);
      #endif

    } else {
      // 段缓冲区为空 关掉。
      st_go_idle();
      // 完成速率控制运动后，确保pwm设置正确。
      #ifdef VARIABLE_SPINDLE
      if (st.exec_block->is_pwm_rate_adjusted) { spindle_set_speed(SPINDLE_PWM_OFF_VALUE); }
      #endif
      system_set_exec_state_flag(EXEC_CYCLE_STOP); // 标记周期结束的主程序
      return; // 没有什么要做，但退出。
    }
  }


  // 检查探测状态
  if (sys_probe_state == PROBE_ACTIVE) { probe_state_monitor(); }

  //重置失步位。 Reset step out bits.
  st.step_outbits = 0;

  // 通过Bresenham线算法执行步进位移曲线
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_x += st.steps[X_AXIS];
  #else
    st.counter_x += st.exec_block->steps[X_AXIS];
  #endif
  if (st.counter_x > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<X_STEP_BIT);
    st.counter_x -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<X_DIRECTION_BIT)) { sys_position[X_AXIS]--; }
    else { sys_position[X_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_y += st.steps[Y_AXIS];
  #else
    st.counter_y += st.exec_block->steps[Y_AXIS];
  #endif
  if (st.counter_y > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Y_STEP_BIT);
    st.counter_y -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Y_DIRECTION_BIT)) { sys_position[Y_AXIS]--; }
    else { sys_position[Y_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_z += st.steps[Z_AXIS];
  #else
    st.counter_z += st.exec_block->steps[Z_AXIS];
  #endif
  if (st.counter_z > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Z_STEP_BIT);
    st.counter_z -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Z_DIRECTION_BIT)) { sys_position[Z_AXIS]--; }
    else { sys_position[Z_AXIS]++; }
  }

  // 在归位循环过程中，锁定并防止所需的轴移动。
  if (sys.state == STATE_HOMING) { st.step_outbits &= sys.homing_axis_lock; }

  st.step_count--; // 减少步骤事件计数
  if (st.step_count == 0) {
    // 细分完成 丢弃当前分段和提前分段索引。
    st.exec_segment = NULL;

	uint8_t segment_tail_next = segment_buffer_tail + 1;
	if (segment_tail_next == SEGMENT_BUFFER_SIZE)
		segment_tail_next = 0;
	segment_buffer_tail = segment_tail_next;

  }

  st.step_outbits ^= step_port_invert_mask;  // 应用步骤端口翻转掩码
  busy = false;
}


/*
  步进器端口复位中断：Timer0 OVF中断处理该步骤的下降沿
   脉冲。这应该始终在下一个Timer1 COMPA中断之前触发并独立进行
   完成后，如果完成移动后Timer1被禁用。
   注意：串行和步进中断之间的中断冲突可能导致延迟
   几微秒，如果他们执行一个在另一个之前。不是什么大事，但可以
   如果另一个高频异步中断导致高步进速率问题
   添加到Grbl。
*/
// 当设置电机端口位执行时，该中断由ISR_TIMER1_COMPAREA使能
// 一个步骤 此ISR在短时间后重置电机端口（settings.pulse_microseconds）
// 完成一个循环

void TIM3_IRQHandler(void)
{
#ifdef STM32F103C8
	if ((TIM3->SR & 0x0001) != 0)                  // 检查中断源
	{
		TIM3->SR &= ~(1<<0);                          // 清除UIF标志
		TIM3->CNT = 0;
		NVIC_DisableIRQ(TIM3_IRQn);
		GPIO_Write(STEP_PORT, (GPIO_ReadOutputData(STEP_PORT) & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK));
	}
#endif

}
#ifdef STEP_PULSE_DELAY
//这个中断仅在STEP_PULSE_DELAY被使能时使用。这里，阶跃脉冲是
//在STEP_PULSE_DELAY时间段结束后启动。ISR TIMER2_OVF中断
//然后会在正常操作之后触发相应的settings.pulse_microseconds。
//在方向，步进脉冲和步骤完成事件之间的新时间设置在
// st_wake_up（）例程。
  ISR(TIMER0_COMPA_vect)
  {
    STEP_PORT = st.step_bits; // 开始步进脉冲
  }
#endif


// 生成步进中断驱动程序中使用的步进和方向端口反转掩码。
void st_generate_step_dir_invert_masks()
{
  uint8_t idx;
  step_port_invert_mask = 0;
  dir_port_invert_mask = 0;
  for (idx=0; idx<N_AXIS; idx++) {
    if (bit_istrue(settings.step_invert_mask,bit(idx))) { step_port_invert_mask |= step_pin_mask[idx]; }
    if (bit_istrue(settings.dir_invert_mask,bit(idx))) { dir_port_invert_mask |= direction_pin_mask[idx]; }
  }
}


// 重置并清除步进子系统变量
void st_reset()
{
  // 初始化步进驱动器空闲状态。
  st_go_idle();

  // 初始化步进算法变量。
  memset(&prep, 0, sizeof(st_prep_t));
  memset(&st, 0, sizeof(stepper_t));
  st.exec_segment = NULL;
  pl_block = NULL;  // 段缓冲区使用的规划块指针
  segment_buffer_tail = 0;
  segment_buffer_head = 0; // empty = tail
  segment_next_head = 1;
  busy = false;

  st_generate_step_dir_invert_masks();
  st.dir_outbits = dir_port_invert_mask; // 将方向位初始化为默认值

  // 初始化步进和方向端口引脚。
#ifdef STM32F103C8
  GPIO_Write(STEP_PORT, (GPIO_ReadOutputData(STEP_PORT) & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK));
  GPIO_Write(DIRECTION_PORT, (GPIO_ReadOutputData(DIRECTION_PORT) & ~DIRECTION_MASK) | (dir_port_invert_mask & DIRECTION_MASK));
#endif
}

// 初始化并启动步进电机子系统
void stepper_init()
{
  // 配置步进和方向接口引脚
#ifdef STM32F103C8
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_STEPPERS_DISABLE_PORT, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = STEPPERS_DISABLE_MASK;
	GPIO_Init(STEPPERS_DISABLE_PORT, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_STEP_PORT, ENABLE);
	GPIO_InitStructure.GPIO_Pin = STEP_MASK;
	GPIO_Init(STEP_PORT, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_DIRECTION_PORT, ENABLE);
	GPIO_InitStructure.GPIO_Pin = DIRECTION_MASK;
	GPIO_Init(DIRECTION_PORT, &GPIO_InitStructure);

	RCC->APB1ENR |= RCC_APB1Periph_TIM2;
	TIM_Configuration(TIM2, 1, 1, 1);
	RCC->APB1ENR |= RCC_APB1Periph_TIM3;
	TIM_Configuration(TIM3, 1, 1, 1);
	NVIC_DisableIRQ(TIM3_IRQn);
	NVIC_DisableIRQ(TIM2_IRQn);
#endif

}


// 当执行块被新的计划更新时，由planner_recalculate（）调用。
void st_update_plan_block_parameters()
{
  if (pl_block != NULL) { // 如果在新块的开始处忽略。
    prep.recalculate_flag |= PREP_FLAG_RECALCULATE;
    pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed; // 更新输入速度
    pl_block = NULL; // 标记st_prep_segment（）以加载并检查活动速度配置文件。
  }
}


// 增加步段缓冲块数据环形缓冲区。
static uint8_t st_next_block_index(uint8_t block_index)
{
  block_index++;
  if ( block_index == (SEGMENT_BUFFER_SIZE-1) ) { return(0); }
  return(block_index);
}


#ifdef PARKING_ENABLE
  // 改变步段缓冲区的运行状态，执行特殊的停车动作。
  void st_parking_setup_buffer()
  {
    // 必要时，存储部分完成的块的步骤执行数据。
    if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK) {
      prep.last_st_block_index = prep.st_block_index;
      prep.last_steps_remaining = prep.steps_remaining;
      prep.last_dt_remainder = prep.dt_remainder;
      prep.last_step_per_mm = prep.step_per_mm;
    }
    // 设置标志来执行停车动作
    prep.recalculate_flag |= PREP_FLAG_PARKING;
    prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE);
    pl_block = NULL; // 总是重置停车动作重新加载新的区块。
  }


  // 停车后，将步进缓冲区恢复到正常运行状态。
  void st_parking_restore_buffer()
  {
    // 根据需要恢复部分完成块的步骤执行数据和标志。
    if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK) {
      st_prep_block = &st_block_buffer[prep.last_st_block_index];
      prep.st_block_index = prep.last_st_block_index;
      prep.steps_remaining = prep.last_steps_remaining;
      prep.dt_remainder = prep.last_dt_remainder;
      prep.step_per_mm = prep.last_step_per_mm;
      prep.recalculate_flag = (PREP_FLAG_HOLD_PARTIAL_BLOCK | PREP_FLAG_RECALCULATE);
      prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm; // Recompute this value.
    } else {
      prep.recalculate_flag = false;
    }
    pl_block = NULL; // 设置为重新加载下一个块。
  }
#endif


/* 准备步骤段缓冲区。从主程序连续调用。

   段缓冲区是执行步骤之间的中间缓冲区接口
   由步进算法和规划器生成的速度曲线。步进
   算法只执行段缓冲区内的步骤，由主程序填充
   当从计划缓冲区中的第一个块“检出”这些步骤时。这保持了
   一步执行和计划优化处理原子和相互保护。
   从计划者缓冲区中“检出”的步数以及中的分段数
   段缓冲区的大小和计算方式使得主程序中不需要进行任何操作
   比重新填充之前步进器算法清空它的时间要长。
   目前，分段缓冲器保守地保持大致40-50毫秒的步长。
   注：计算单位是步进，毫米和分钟。
*/
void st_prep_buffer()
{
  // 阻止步骤预备缓冲区，处于暂停状态，并且没有执行暂停动作。
  if (bit_istrue(sys.step_control,STEP_CONTROL_END_MOTION)) { return; }

  while (segment_buffer_tail != segment_next_head) { // 检查是否需要填充缓冲区。

    // 确定是否需要加载新的规划器块或者是否需要重新计算块。
    if (pl_block == NULL) {

      // 排队的块的查询计划器
      if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) { pl_block = plan_get_system_motion_block(); }
      else { pl_block = plan_get_current_block(); }
      if (pl_block == NULL) { return; } // 没有计划者块。出口。

      // 检查是否只需要重新计算速度曲线或加载一个新的程序段。
      if (prep.recalculate_flag & PREP_FLAG_RECALCULATE) {

        #ifdef PARKING_ENABLE
          if (prep.recalculate_flag & PREP_FLAG_PARKING) { prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE); }
          else { prep.recalculate_flag = false; }
        #else
          prep.recalculate_flag = false;
        #endif

      } else {

        // 加载块的Bresenham步进数据。
        prep.st_block_index = st_next_block_index(prep.st_block_index);

        // 从新的计划程序块准备并复制Bresenham算法分段数据，这样
        // 当段缓冲区完成规划块的时候，可能会被丢弃
        // 段缓冲区完成准备块，但步进器ISR仍在执行它。
        st_prep_block = &st_block_buffer[prep.st_block_index];
        st_prep_block->direction_bits = pl_block->direction_bits;//方向位
        uint8_t idx;
        #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
          for (idx=0; idx<N_AXIS; idx++) { st_prep_block->steps[idx] = (pl_block->steps[idx] << 1); }
          st_prep_block->step_event_count = (pl_block->step_event_count << 1);
        #else
          //启用AMASS后，简单的位移将所有Bresenham数据乘以最大AMASS
          //级别，这样我们就不会在算法中的任何地方超出原始数据。
          //如果原始数据被分割，我们可以从整数舍入中丢失一步。
          for (idx=0; idx<N_AXIS; idx++) { st_prep_block->steps[idx] = pl_block->steps[idx] << MAX_AMASS_LEVEL; }
          st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
        #endif

        // 初始化段缓冲区数据以生成段。
        prep.steps_remaining = (float)pl_block->step_event_count;
        prep.step_per_mm = prep.steps_remaining/pl_block->millimeters;
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm;
        prep.dt_remainder = 0.0f; // 重置为新的分块

        if ((sys.step_control & STEP_CONTROL_EXECUTE_HOLD) || (prep.recalculate_flag & PREP_FLAG_DECEL_OVERRIDE)) {
          // 新的块加载中旬举行。重写计划程序块的输入速度以强制减速。
          prep.current_speed = prep.exit_speed;
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed;
          prep.recalculate_flag &= ~(PREP_FLAG_DECEL_OVERRIDE);
        } else {
          prep.current_speed = sqrtf(pl_block->entry_speed_sqr);
        }
#ifdef VARIABLE_SPINDLE
        // 设置激光模式变量。PWM速率调整后的动作总是会随着动作完成
        // 主轴关闭
        st_prep_block->is_pwm_rate_adjusted = false;
        if (settings.flags & BITFLAG_LASER_MODE) {
          if (pl_block->condition & PL_COND_FLAG_SPINDLE_CCW) {
            // 预先计算反编程速率以加快每个段的PWM更新。
            prep.inv_rate = 1.0f / pl_block->programmed_rate;
            st_prep_block->is_pwm_rate_adjusted = true;
          }
        }
#endif
      }

			/* ---------------------------------------------------------------------------------
			 根据进入和退出计算新计划程序块的速度曲线
			 速度，或重新计算一个部分完成的规划师块的个人资料，如果
			 策划者已经更新了它。对于指令强制减速，如来自饲料
			 保持，重写规划器的速度并减速到目标出口速度。
			*/
			prep.mm_complete = 0.0f; // 在距离程序段末尾0.0mm处完成默认的速度曲线。
			float inv_2_accel = 0.5f/pl_block->acceleration;
			if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { // [强制减速到零速度]
				// 计算进给保持进行中的速度曲线参数。此配置文件覆盖
				// 计划程序块配置文件，强制减速到零速度。
				prep.ramp_type = RAMP_DECEL;
				// 计算相对于程序段结束的减速距离。
				float decel_dist = pl_block->millimeters - inv_2_accel*pl_block->entry_speed_sqr;
				if (decel_dist < 0.0f) {
					// 通过整个计划程序块减速。进纸保持结束不在该块中。
					prep.exit_speed = sqrtf(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->millimeters);
				} else {
					prep.mm_complete = decel_dist; // 结束保留。
					prep.exit_speed = 0.0f;
				}
			} else { // [正常操作]
				// 计算或重新计算预备计划程序块的速度曲线参数。
				prep.ramp_type = RAMP_ACCEL; // 初始化为加速斜坡
				prep.accelerate_until = pl_block->millimeters;

				float exit_speed_sqr;
				float nominal_speed;
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          prep.exit_speed = exit_speed_sqr = 0.0f; // 在系统运动结束时强制停止。
        } else {
          exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
          prep.exit_speed = sqrtf(exit_speed_sqr);
        }

        nominal_speed = plan_compute_profile_nominal_speed(pl_block);
				float nominal_speed_sqr = nominal_speed*nominal_speed;
				float intersect_distance =
								0.5f*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));

        if (pl_block->entry_speed_sqr > nominal_speed_sqr) { // 只在超驰减少期间发生。
          prep.accelerate_until = pl_block->millimeters - inv_2_accel*(pl_block->entry_speed_sqr-nominal_speed_sqr);
          if (prep.accelerate_until <= 0.0f) { // 减速而已
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;

            // 计算覆盖块的退出速度，因为它不符合计划器的退出速度。
            prep.exit_speed = sqrtf(pl_block->entry_speed_sqr - 2*pl_block->acceleration*pl_block->millimeters);
            prep.recalculate_flag |= PREP_FLAG_DECEL_OVERRIDE; // 标记为减速覆盖加载下一个程序段。

            // TODO: 确定仅在减速时正确处理参数。
            // 可能会比较棘手，因为进入速度将会是当前的速度，就像在进给保持时一样。
            // 另外，看看这个接近零的速度处理问题。

          } else {
            // 减速巡航或巡航减速类型。保证相交更新的计划。
            prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr); // 由于规划器重新启动，应始终> = 0.0。
						prep.maximum_speed = nominal_speed;
            prep.ramp_type = RAMP_DECEL_OVERRIDE;
          }
				} else if (intersect_distance > 0.0f) {
					if (intersect_distance < pl_block->millimeters) { // 梯形或三角形类型
						// 注意：对于加速巡航和巡航类型，以下计算将为0.0。
						prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr);
						if (prep.decelerate_after < intersect_distance) { // 梯形型
							prep.maximum_speed = nominal_speed;
							if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
								// 巡航减速或巡航型。
								prep.ramp_type = RAMP_CRUISE;
							} else {
								// 全梯形或加速巡航类型
								prep.accelerate_until -= inv_2_accel*(nominal_speed_sqr-pl_block->entry_speed_sqr);
							}
						} else { // 三角型
							prep.accelerate_until = intersect_distance;
							prep.decelerate_after = intersect_distance;
							prep.maximum_speed = sqrtf(2.0f*pl_block->acceleration*intersect_distance+exit_speed_sqr);
						}
					} else { // 只有减速型
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;
					}
				} else { // 仅加速类型
					prep.accelerate_until = 0.0f;
					// prep.decelerate_after = 0.0f;
					prep.maximum_speed = prep.exit_speed;
				}
			}
      
      #ifdef VARIABLE_SPINDLE
        bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM); // 更新块时强制更新.
      #endif
    }
    
    // 初始化新的细分
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    // 设置新的段指向当前的段数据块.
    prep_segment->st_block_index = prep.st_block_index;

    /*------------------------------------------------------------------------------------
        通过确定总距离来计算这个新段的平均速度
      遍历段时间DT_SEGMENT。下面的代码首先尝试创建
      基于当前斜坡条件的完整分段。如果段时间不完整
      当终止在一个斜坡状态改变，代码将继续循环通过
      进行斜坡状态以填充剩余的段执行时间。但是，如果
      一个不完整的段在速度曲线的末端终止，该段是
      尽管截断的执行时间少于DT_SEGMENT，但仍被视为已完成。
        总是假定速度曲线通过斜坡序列：
      加速坡道，巡航状态和减速坡道。每个坡道的行驶距离
      可能范围从零到块的长度。速度配置文件可以在
      在强制减速结束时规划区块（典型）结束或中间区块，
      如来自饲料保持。
    */
    float dt_max = DT_SEGMENT; // 最大段时间
    float dt = 0.0f; // 初始化段时间
    float time_var = dt_max; // 时间工人变量
    float mm_var; // 毫米距离工作变量
    float speed_var; // 速度工人变量
    float mm_remaining = pl_block->millimeters; // 从块结束的新段距离。
    float minimum_mm = mm_remaining-prep.req_mm_increment; // 保证至少一个步骤.
    if (minimum_mm < 0.0f) { minimum_mm = 0.0f; }

    do {
      switch (prep.ramp_type) {
        case RAMP_DECEL_OVERRIDE:
          speed_var = pl_block->acceleration*time_var;
					if (prep.current_speed-prep.maximum_speed <= speed_var) {
            // 巡航或巡航减速类型仅用于减速倍率.
						mm_remaining = prep.accelerate_until;
            time_var = 2.0f*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            prep.ramp_type = RAMP_CRUISE;
            prep.current_speed = prep.maximum_speed;
          } else { // 中等减速倍率斜坡。
						mm_remaining -= time_var*(prep.current_speed - 0.5f*speed_var);
            prep.current_speed -= speed_var;
          }
          break;
        case RAMP_ACCEL:
          // 注意：加速斜坡只在第一个do-while循环期间计算。
          speed_var = pl_block->acceleration*time_var;
          mm_remaining -= time_var*(prep.current_speed + 0.5f*speed_var);
          if (mm_remaining < prep.accelerate_until) { // 加速斜坡结束。
            // 加速巡航，加速 - 减速匝道连接，或块的结束。
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            time_var = 2.0f*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { // 仅加速。
            prep.current_speed += speed_var;
          }
          break;
        case RAMP_CRUISE:
          // 注意：mm_var用于保留不完整段time_var计算的最后一个mm_remaining。
          // 注意：如果maximum_speed * time_var值太低，舍入可能导致mm_var不能改变。至
          // 防止这种情况，只需在规划器中强制执行最低速度阈值。
          mm_var = mm_remaining - prep.maximum_speed*time_var;
          if (mm_var < prep.decelerate_after) { // 巡航结束。
            // 巡航减速连接点或程序段结束
            time_var = (mm_remaining - prep.decelerate_after)/prep.maximum_speed;
            mm_remaining = prep.decelerate_after; // 注意：在EOB为0.0
            prep.ramp_type = RAMP_DECEL;
          } else { // 只巡航。
            mm_remaining = mm_var;
          }
          break;
        default: // case RAMP_DECEL:
          // 注意：mm_var用作misc工作变量以防止接近零速时的错误。
          speed_var = pl_block->acceleration*time_var; // 用作增量速度（毫米/分钟）
          if (prep.current_speed > speed_var) { // 检查零速或低于零速。
            // 计算从段尾到块尾的距离。
            mm_var = mm_remaining - time_var*(prep.current_speed - 0.5f*speed_var); // (mm)
            if (mm_var > prep.mm_complete) { // 典型情况。在减速斜坡。
              mm_remaining = mm_var;
              prep.current_speed -= speed_var;
              break; // 段完成。退出switch-case语句。继续执行循环。
            }
          }
          // 否则，在程序段结束或强制减速结束时。
          time_var = 2.0f*(mm_remaining-prep.mm_complete)/(prep.current_speed+prep.exit_speed);
          mm_remaining = prep.mm_complete;
          prep.current_speed = prep.exit_speed;
      }
      dt += time_var; // 将计算的斜坡时间添加到总段时间。
      if (dt < dt_max) { time_var = dt_max - dt; } // **不完整**在匝道连接处。
      else {
        if (mm_remaining > minimum_mm) { // 检查零级非常慢的段。
          // 增加细分时间以确保细分中至少有一个步骤。覆盖和循环
          // 通过距离计算直到minimum_mm或者mm_complete。
          dt_max += DT_SEGMENT;
          time_var = dt_max - dt;
        } else {
          break; // **完成**退出循环。分段执行时间最长。
        }
      }
    } while (mm_remaining > prep.mm_complete); // **完成**退出循环。档案完整。

    #ifdef VARIABLE_SPINDLE
      /* -----------------------------------------------------------------------------------
        	计算步进段的主轴转速PWM输出
      */

      if (st_prep_block->is_pwm_rate_adjusted || (sys.step_control & STEP_CONTROL_UPDATE_SPINDLE_PWM)) {
        if (pl_block->condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)) {
          float rpm = pl_block->spindle_speed;
          // 注意：进给和快速覆盖与PWM值无关，不会改变激光功率/速率。
          if (st_prep_block->is_pwm_rate_adjusted) { rpm *= (prep.current_speed * prep.inv_rate); }
          // 如果current_speed为零，则可能需要rpm_min *（100 / MAX_SPINDLE_SPEED_OVERRIDE）
          // 但这只是瞬间的动作。可能无关紧要。
          prep.current_spindle_pwm = spindle_compute_pwm_value(rpm);
        }
        else {
          sys.spindle_speed = 0.0;
          prep.current_spindle_pwm = SPINDLE_PWM_OFF_VALUE;
        }
        bit_false(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
      }
      prep_segment->spindle_pwm = prep.current_spindle_pwm; // 重新加载段的PWM值

    #endif
    
    /* -----------------------------------------------------------------------------------
		   计算分段步进速率，执行步骤以及应用必要的速率更正。
		   注：步骤是通过毫米距离的直接标量转换来计算的
		   保留在块中，而不是递增地计算每个执行的步骤
		   分割。这有助于消除几个附加的浮点四舍五入问题。
		   然而，由于花车只有7.2位有效数字，长时间的移动非常
		   高步数可能会超过浮子的精度，这可能会导致步骤丢失。
		   幸运的是，这种情况在CNC机器中是不太可能和不切实际的
		   由Grbl支持（即以200步/ mm超过10米的轴线行程）。
    */
    float step_dist_remaining = prep.step_per_mm*mm_remaining; // 将mm_remaining转换为步骤
    float n_steps_remaining = ceilf(step_dist_remaining); // 剩余的当前步骤
    float last_n_steps_remaining = ceilf(prep.steps_remaining); // 总结最后的步骤
	prep_segment->n_step = (uint16_t)(last_n_steps_remaining - n_steps_remaining); // 计算要执行的步骤数。

    // 如果我们在一个feed的结尾，并且没有一个步骤来执行，那么保释。
    if (prep_segment->n_step == 0) {
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) {
        // 不到一个步骤减速到零速，但已经非常接近。AMASS
        // 需要执行完整的步骤。所以，只要保释。
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // 段没有生成，但当前的步骤数据仍然保留。
      }
    }

    //计算段的步进速率。由于步骤是整数，并且行程的距离不是，
    //每一段的结尾都可以有一个不同量值的部分步骤
    //执行，因为步进器ISR由于AMASS算法需要整个步骤。至
    //补偿，我们跟踪执行前一个分段的部分步骤的时间
    //以当前段的部分步距应用它，这样它就可以了
    //调整整个分段速率以保持步骤输出的准确性。这些利率调整是
    //通常非常小，不会对性能产生负面影响，但确保Grbl
    //输出计划者计算的确切的加速度和速度曲线。
    dt += prep.dt_remainder; // 应用上一个段的部分步骤执行时间
    float inv_rate = dt/(last_n_steps_remaining - step_dist_remaining); // 计算调整的步进速率倒数

    // 计算准备好的段的每个步骤的CPU周期。
	uint32_t cycles = (uint32_t)ceilf((TICKS_PER_MICROSECOND * 1000000) *inv_rate * 60); // (cycles/step)

    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      // 计算步进时间和多轴平滑水平。
      // 注意：AMASS每个级别都会超时，所以只需要一个预分频器。
      if (cycles < AMASS_LEVEL1) { prep_segment->amass_level = 0; }
      else {
        if (cycles < AMASS_LEVEL2) { prep_segment->amass_level = 1; }
        else if (cycles < AMASS_LEVEL3) { prep_segment->amass_level = 2; }
        else { prep_segment->amass_level = 3; }
        cycles >>= prep_segment->amass_level;
        prep_segment->n_step <<= prep_segment->amass_level;
      }
      if (cycles < (1UL << 16)) { prep_segment->cycles_per_tick = cycles; } // < 65536 (4.1ms @ 16MHz)
      else { prep_segment->cycles_per_tick = 0xffff; } // 只需设置最慢的速度。
    #else
      // 计算正常步骤生成的步进时序和定时器预分频器。
      if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
        prep_segment->prescaler = 1; // 预分频器: 0
        prep_segment->cycles_per_tick = cycles;
      } else if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        prep_segment->prescaler = 2; // 预分频器: 8
        prep_segment->cycles_per_tick = cycles >> 3;
      } else {
        prep_segment->prescaler = 3; // 预分频器: 64
        if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
          prep_segment->cycles_per_tick =  cycles >> 6;
        } else { // 只需设置最慢的速度。（约4步/秒）
          prep_segment->cycles_per_tick = 0xffff;
        }
      }
    #endif

    // 段完成！递增段缓冲区索引，所以步进ISR可以立即执行它。
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }

    // 更新适当的计划者和细分数据。
    pl_block->millimeters = mm_remaining;
    prep.steps_remaining = n_steps_remaining;
    prep.dt_remainder = (n_steps_remaining - step_dist_remaining)*inv_rate;

    // 检查退出条件和标志加载下一个计划模块。
    if (mm_remaining == prep.mm_complete) {
      // 计划程序块结束或强制终止。没有更多的距离被执行。
      if (mm_remaining > 0.0f) { // 在强制终止结束。
          // 重置准备参数恢复，然后保释。允许步进ISR完成
          //段队列，实时协议将在接收到的情况下设置新的状态
          //从ISR循环停止标志。在此之前，Prep_segment被阻止。
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // Bail!
      } else { // 计划器块结束
        // 规划师块是完整的。所有步骤都设置为在段缓冲区中执行.
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
          return;
        }
        pl_block = NULL; // 设置指针指示检查并加载下一个计划程序块。
        plan_discard_current_block();
      }
    }

  }
}


//通过实时状态报告调用来获取正在执行的当前速度。这个值
//然而不是现在的速度，而是在最后一个步骤中计算的速度
//在段缓冲区中。它将总是落后于块的数量（-1）
// 除以每秒加速度秒数.
float st_get_realtime_rate()
{
  if (sys.state & (STATE_CYCLE | STATE_HOMING | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR)){
    return prep.current_speed;
  }
  return 0.0f;
}
#ifdef STM32F103C8
void TIM_Configuration(TIM_TypeDef* TIMER, u16 Period, u16 Prescaler, u8 PP)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	TIM_TimeBaseStructure.TIM_Period = Period - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMER, &TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIMER, TIM_IT_Update);
	TIM_ITConfig(TIMER, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIMER, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	if (TIMER == TIM2) { NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; }
	else if (TIMER == TIM3) { NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; }
	else if (TIMER == TIM4) { NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; }

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
#endif
