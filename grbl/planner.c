/*
  planner.c - 缓冲移动命令并管理加速度曲线图

加减速规划器
包括：
1.梯型加减速
2.前瞻
3.混合段拐角限制
*/

#include "grbl.h"


static plan_block_t block_buffer[BLOCK_BUFFER_SIZE];  // 运动指令的环形缓冲区
static uint8_t block_buffer_tail;     // 现在处理的块的索引
static uint8_t block_buffer_head;     // 下一个要推的块的索引
static uint8_t next_buffer_head;      // 下一个缓冲区的索引
static uint8_t block_buffer_planned;  // 最佳计划区块的索引

// 定义规划变量
typedef struct {
  int32_t position[N_AXIS];          // 绝对步骤中的工具的计划器位置。保持分开
                                     // 从g代码位置进行需要多行移动的移动，
                                     // 即圆弧，固定循环和间隙补偿。
  float previous_unit_vec[N_AXIS];   // 以前路径线段的单位向量
  float previous_nominal_speed;  // 先前路径线段的标称速度
} planner_t;
static planner_t pl;


// 返回环形缓冲区中下一个块的索引。也称为步进段缓冲区。
uint8_t plan_next_block_index(uint8_t block_index)
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}


// 返回环形缓冲区中前一个块的索引
static uint8_t plan_prev_block_index(uint8_t block_index)
{
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}


/*                                    计划速度定义
                                     +--------+   <- 当前->额定速度
                                    /          \
                                                           当前->进入速度 ->   +            \
                                   |             + <- 下一个->进入速度 (又名退出速度)
                                   +-------------+
                                       time -->

  根据以下基本指导重新计算运动计划：

    1. 按相反顺序依次检查每个可行的块，并计算结点速度
        （即current-> entry_speed），使得：
      a. 没有结速度超过预先计算的最大结速限制或相邻块的标称速度。
      b. 一个程序段的进入速度不能超过一个从其出口速度（next-> entry_speed）计算出来的反向运算，并且在程序段行程距离内允许的最大减速度。
      c. 最后一个（或最新追加的）程序段是从一个完全停止（零的退出速度）计划的。
    2. 按照时间顺序（向前）顺序遍历每一个块，并且如果是，则调低交点速度值
      a. 出口速度超过从入口速度向前计算的出口速度，并且在行程距离上允许的最大加速度。

  当这些阶段完成后，规划人员将最大化整个速度分布
  的规划区块，每个区块在最大允许加速限制下运行。在
  换句话说，对于规划者中的所有区块来说，这个规划是最佳的，并且不会进一步提高速度
  是可能的。如果新的块被添加到缓冲区中，则根据所述重新计算该计划
  一个新的最佳计划的准则。

  为了提高这些指南的计算效率，一组规划指针已经被使用
  当计划者准则无法在逻辑上做出任何进一步的时候被创建以指示停止计算点
  正常运行时计划的改变或改进，新的块被流式传输并添加到计划中
  计划者缓冲区。例如，如果计划者中的连续块的一个子集已经计划好了，
  以最大交叉点速度（或由第一个规划区块）包围，没有新的区块
  添加到规划缓冲区将会改变其中的速度分布。所以我们不再需要计算了
  他们。或者，如果规划器中第一个块的一组连续块（或最优停止计算
  点）都加速，他们都是最佳的，不能被添加到一个新的块更改
  规划缓冲区，因为这将只会进一步增加计划速度，按时间顺序排列，直到达到最大值
  结速度达到。但是，如果计划的操作条件从不经常变化
  使用的feed保留或进给覆盖，停止计算指针将被重置，整个计划
  如一般准则所述重新计算。

  规划器缓冲区索引映射：
  -  block_buffer_tail：指向规划缓冲区的开始。首先被执行或被执行。
  -  block_buffer_head：指向缓冲区中最后一个块之后的缓冲块。用于表示是否
      缓冲区已满或为空。如标准环形缓冲区所述，此块始终为空。
  next_buffer_head：指向缓冲区块后面的下一个规划缓冲区块。等于的时候
      缓冲尾巴，这表示缓冲区已满。
  -  block_buffer_planned：指向正常的最后一个优化计划块之后的第一个缓冲块
      流媒体操作条件。用于规划优化，避免重新计算部分
      如上所述，规划器缓冲区不会随着新块的添加而改变。此外，
      这个块不能小于block_buffer_tail，并且总是被推送和维护
      在一个周期中遇到plan_discard_current_block（）例程时的这个需求。

  注意：由于规划者只计算规划缓冲区中的内容，所以有些动作很短
  线段，如G2 / 3弧线或复杂曲线，似乎移动缓慢。这是因为根本没有
  在整个缓冲区中行进的足够的组合距离加速到标称速度，然后
  如指南所述，在缓冲器的末端减速到完全停止。如果发生这种情况，
  成为一个烦恼，有几个简单的解决方案：（1）最大化机器加速度。计划者
  将能够在相同的组合距离内计算更高的速度分布。（2）最大化线路
  将每个块的运动距离调整到期望的容差。规划者使用的距离越远，
  它可以走得更快。（3）最大化计划缓冲区大小。这也会增加组合距离
  计划者计算结束。这也增加了规划者必须执行的计算的数量
  计算一个最佳的计划，所以仔细选择。Arduino的328p内存已经超出了，但未来
  ARM版本应该具有足够的内存和速度，以便编译数百或更多的预读块。

*/
static void planner_recalculate()
{
  // 将块索引初始化到规划器缓冲区中的最后一个块。
  uint8_t block_index = plan_prev_block_index(block_buffer_head);

  // 保释 只有一个计划块可以做任何事情。
  if (block_index == block_buffer_planned) { return; }

  //反向传递：粗略地最大化所有可能的减速曲线
  //在缓冲区中阻塞 停止计划，当最后的最佳计划或尾指针已达到。
  //注意：正向传球稍后将改进并纠正反向传球以创建一个最佳方案。
  float entry_speed_sqr;
  plan_block_t *next;
  plan_block_t *current = &block_buffer[block_index];

  //计算缓冲区中最后一个块的最大入口速度，其中出口速度总是为零。
  current->entry_speed_sqr = min( current->max_entry_speed_sqr, 2*current->acceleration*current->millimeters);

  block_index = plan_prev_block_index(block_index);
  if (block_index == block_buffer_planned) { // 只有两个在缓冲液可计划块。反向传递完成。
    // 检查第一个块是否是尾部 如果是，请通知步进器更新其当前参数。
    if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }
  } else { // 三个或更多的计划，能块
    while (block_index != block_buffer_planned) {
      next = current;
      current = &block_buffer[block_index];
      block_index = plan_prev_block_index(block_index);

      // 检查下一个块是否是尾部块（=计划的块）。如果是，请更新当前的步进参数。
      if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }

      // 计算最大进入速度从当前程序段的出口速度减速。
      if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
        entry_speed_sqr = next->entry_speed_sqr + 2*current->acceleration*current->millimeters;
        if (entry_speed_sqr < current->max_entry_speed_sqr) {
          current->entry_speed_sqr = entry_speed_sqr;
        } else {
          current->entry_speed_sqr = current->max_entry_speed_sqr;
        }
      }
    }
  }

  // 正向通过：从计划指针向前计划加速曲线。
  // 也扫描最佳的计划断点并适当地更新计划的指针。
  next = &block_buffer[block_buffer_planned]; // 从缓冲区计划的指针开始
  block_index = plan_next_block_index(block_buffer_planned);
  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];

    //在正向通行中检测到的任何加速都会自动移动最佳计划
    //指针前进，因为之前的所有内容都是最优的。换句话说，没什么
    //可以通过逻辑改善从缓冲尾到计划指针的计划。
    if (current->entry_speed_sqr < next->entry_speed_sqr) {
      entry_speed_sqr = current->entry_speed_sqr + 2*current->acceleration*current->millimeters;
      // 如果为true，则当前块为全加速，我们可以将计划的指针向前移动。
      if (entry_speed_sqr < next->entry_speed_sqr) {
        next->entry_speed_sqr = entry_speed_sqr; // 总是<= max_entry_speed_sqr。落后的传球设定了这一点。
        block_buffer_planned = block_index; // 设置最佳的计划指针。
      }
    }

    //以最大输入速度设置的任何块也创建一个最佳的计划
    //指向缓冲区。当计划被开头的方括号时
    //缓冲区和最大输入速度或两个最大输入速度，每个块之间
    //不能从逻辑上进一步改进。因此，我们不必重新计算它们了。
    if (next->entry_speed_sqr == next->max_entry_speed_sqr) { block_buffer_planned = block_index; }
    block_index = plan_next_block_index( block_index );
  }
}


void plan_reset()
{
  memset(&pl, 0, sizeof(planner_t)); // 清除规划器结构
  plan_reset_buffer();
}


void plan_reset_buffer()
{
  block_buffer_tail = 0;
  block_buffer_head = 0; // Empty = tail
  next_buffer_head = 1; // plan_next_block_index(block_buffer_head)
  block_buffer_planned = 0; // = block_buffer_tail;
}


void plan_discard_current_block()
{
  if (block_buffer_head != block_buffer_tail) { // 丢弃非空缓冲区。
    uint8_t block_index = plan_next_block_index( block_buffer_tail );
    // 推block_buffer_planned指针，如果遇到。
    if (block_buffer_tail == block_buffer_planned) { block_buffer_planned = block_index; }
    block_buffer_tail = block_index;
  }
}


// 返回系统运动使用的规划器缓冲块的地址。由段生成器调用。
plan_block_t *plan_get_system_motion_block()
{
  return(&block_buffer[block_buffer_head]);
}


// 返回第一个计划程序块的地址（如果可用）。由各种主要程序功能调用。
plan_block_t *plan_get_current_block()
{
  if (block_buffer_head == block_buffer_tail) { return(NULL); } // 缓冲区为空
  return(&block_buffer[block_buffer_tail]);
}


float plan_get_exec_block_exit_speed_sqr()
{
  uint8_t block_index = plan_next_block_index(block_buffer_tail);
  if (block_index == block_buffer_head) { return( 0.0 ); }
  return( block_buffer[block_index].entry_speed_sqr );
}


// 返回块环形缓冲区的可用性状态。真的，如果满了。
uint8_t plan_check_full_buffer()
{
  if (block_buffer_tail == next_buffer_head) { return(true); }
  return(false);
}


// 根据运行条件和覆盖值计算并返回块的额定速度。
// 注意：所有系统运动命令（例如归位/停车）不会被覆盖。
float plan_compute_profile_nominal_speed(plan_block_t *block)
{
  float nominal_speed = block->programmed_rate;
  if (block->condition & PL_COND_FLAG_RAPID_MOTION) { nominal_speed *= (0.01f*sys.r_override); }
  else {
    if (!(block->condition & PL_COND_FLAG_NO_FEED_OVERRIDE)) { nominal_speed *= (0.01f*sys.f_override); }
    if (nominal_speed > block->rapid_rate) { nominal_speed = block->rapid_rate; }
  }
  if (nominal_speed > MINIMUM_FEED_RATE) { return(nominal_speed); }
  return(MINIMUM_FEED_RATE);
}


// 根据结点的最小值计算并更新块的最大进入速度（sqr）
// 以前和当前的标称速度和最大结速。
static void plan_compute_profile_parameters(plan_block_t *block, float nominal_speed, float prev_nominal_speed)
{
  // 根据结点速度和相邻标称速度的最小值计算结点最大入口。
  if (nominal_speed > prev_nominal_speed) { block->max_entry_speed_sqr = prev_nominal_speed*prev_nominal_speed; }
  else { block->max_entry_speed_sqr = nominal_speed*nominal_speed; }
  if (block->max_entry_speed_sqr > block->max_junction_speed_sqr) { block->max_entry_speed_sqr = block->max_junction_speed_sqr; }
}


// 在基于运动的覆盖更改时重新计算缓冲的运动轮廓参数。
void plan_update_velocity_profile_parameters()
{
  uint8_t block_index = block_buffer_tail;
  plan_block_t *block;
  float nominal_speed;
  float prev_nominal_speed = SOME_LARGE_VALUE; // 将第一个模块的额定速度计算设置为高。
  while (block_index != block_buffer_head) {
    block = &block_buffer[block_index];
    nominal_speed = plan_compute_profile_nominal_speed(block);
    plan_compute_profile_parameters(block, nominal_speed, prev_nominal_speed);
    prev_nominal_speed = nominal_speed;
    block_index = plan_next_block_index(block_index);
  }
  pl.previous_nominal_speed = prev_nominal_speed; // 更新下一个传入块的prev标称速度。
}


/* 
   添加一个新的线性运动到缓冲区。目标[N_AXIS]是有符号的绝对目标位置
   以毫米为单位。进给率指定运动的速度。如果进给速率倒置，则进给
   速率是指“频率”，并将在1 / feed_rate分钟内完成操作。
   传递给计划者的所有位置数据必须以机器位置来保持计划者
   独立于由g代码分析器处理的任何坐标系变化和偏移量。
   注意：假设缓冲区可用。缓冲区检查由motion_control在更高级别处理。
   换句话说，缓冲区头永远不会等于缓冲区尾部。也是进给率输入值
   以三种方式使用：如果invert_feed_rate为假，则作为正常进给速率，如果是，则作为反时限
   如果feed_rate值为负数（和。），则reverse_feed_rate为true，否则为seek / rapids rate
   invert_feed_rate总是为false）。
   系统运动状态通知计划者在总是未使用的块缓冲区中计划运动
   头。它避免了改变规划器的状态，并保留缓冲区，以确保后续的gcode
   运动仍然正确计划，而步进模块只指向块缓冲区头
   执行特殊的系统动作。 
*/
// 结果保存在block_buffer[block_buffer_head]
// 返回ok 或不需要移动空快
uint8_t plan_buffer_line(float *target, plan_line_data_t *pl_data)
{
  // 准备并初始化新块。复制相关的pl_data以执行块。
  plan_block_t *block = &block_buffer[block_buffer_head];
  memset(block,0,sizeof(plan_block_t)); // 清零所有的块值。
  block->condition = pl_data->condition;//位标志变量表示计划员条件
  #ifdef VARIABLE_SPINDLE
    block->spindle_speed = pl_data->spindle_speed;
  #endif
  #ifdef USE_LINE_NUMBERS
    block->line_number = pl_data->line_number;
  #endif

  // 计算并存储初始移动距离数据。
  int32_t target_steps[N_AXIS], position_steps[N_AXIS];
  float unit_vec[N_AXIS], delta_mm;
  uint8_t idx;

  // 根据计划的运动类型复制位置数据。
  if (block->condition & PL_COND_FLAG_SYSTEM_MOTION) {
#ifdef COREXY
    position_steps[X_AXIS] = system_convert_corexy_to_x_axis_steps(sys_position);
    position_steps[Y_AXIS] = system_convert_corexy_to_y_axis_steps(sys_position);
    position_steps[Z_AXIS] = sys_position[Z_AXIS];
#else
    memcpy(position_steps, sys_position, sizeof(sys_position));
#endif
  }
  else { memcpy(position_steps, pl.position, sizeof(pl.position)); }

  #ifdef COREXY
    target_steps[A_MOTOR] = lround(target[A_MOTOR]*settings.steps_per_mm[A_MOTOR]);
    target_steps[B_MOTOR] = lround(target[B_MOTOR]*settings.steps_per_mm[B_MOTOR]);
    block->steps[A_MOTOR] = labs((target_steps[X_AXIS]-position_steps[X_AXIS]) + (target_steps[Y_AXIS]-position_steps[Y_AXIS]));
    block->steps[B_MOTOR] = labs((target_steps[X_AXIS]-position_steps[X_AXIS]) - (target_steps[Y_AXIS]-position_steps[Y_AXIS]));
  #endif
  //计算各轴脉冲数，走多少毫米，最多的脉冲数
  for (idx=0; idx<N_AXIS; idx++) {
    // 以绝对步数计算目标位置，为每个轴计算步数，并确定最大步进事件。
    // 另外，计算移动和准备单位矢量计算的各个轴的距离。
    // 注意：从转换后的步骤值计算真实距离。
    #ifdef COREXY
      if ( !(idx == A_MOTOR) && !(idx == B_MOTOR) ) {
        target_steps[idx] = lroundf(target[idx]*settings.steps_per_mm[idx]);
        block->steps[idx] = fabsf(target_steps[idx]-position_steps[idx]);
      }
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
      if (idx == A_MOTOR) {
        delta_mm = (target_steps[X_AXIS]-position_steps[X_AXIS] + target_steps[Y_AXIS]-position_steps[Y_AXIS])/settings.steps_per_mm[idx];
      } else if (idx == B_MOTOR) {
        delta_mm = (target_steps[X_AXIS]-position_steps[X_AXIS] - target_steps[Y_AXIS]+position_steps[Y_AXIS])/settings.steps_per_mm[idx];
      } else {
        delta_mm = (target_steps[idx] - position_steps[idx])/settings.steps_per_mm[idx];
      }
    #else
      //算出需要的脉冲数
	  target_steps[idx] = lroundf(target[idx]*settings.steps_per_mm[idx]);//函数将返回最接近x的整数 去掉小数
	  //本轴共要走多个脉冲  （目标-当前）
      block->steps[idx] = abs(target_steps[idx]-position_steps[idx]);//求整数的绝对值,就是去掉负号-100=100 0=0 100=100
	  //所有轴中走最多脉冲数的保存下来（当前和上次比较）
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
	  //本轴共要走多少毫米+-mm
      delta_mm = (target_steps[idx] - position_steps[idx])/settings.steps_per_mm[idx];
	#endif
    unit_vec[idx] = delta_mm; // 存储单位矢量分子

    // 设置本轴方向掩码位 每个位代表一个轴
    if (delta_mm < 0.0f ) { block->direction_bits |= direction_pin_mask[idx]; }
  }

  // 如果不需要移动就退出返回空快
  if (block->step_event_count == 0) { return(PLAN_EMPTY_BLOCK); }

  //计算线移动的单位矢量和块的最大进给速率和加速度
  //向下，使得相对于线方向没有超出单个轴的最大值。
  //注意：这个计算假定所有的轴都是正交的（笛卡尔）并且与ABC轴一起工作，
  //如果它们也是正交的/独立的。在单位矢量的绝对值上运行。

  //开平方求出线段空间里移动的距离
  block->millimeters = convert_delta_vector_to_unit_vector(unit_vec);//三角洲矢量转换为单位矢量 会改变unit_vec的值
  block->acceleration = limit_value_by_axis_maximum(settings.acceleration, unit_vec);
  block->rapid_rate = limit_value_by_axis_maximum(settings.max_rate, unit_vec);

  // 存储编程的速率。
  if (block->condition & PL_COND_FLAG_RAPID_MOTION) { block->programmed_rate = block->rapid_rate; }
  else { 
    block->programmed_rate = pl_data->feed_rate;
    if (block->condition & PL_COND_FLAG_INVERSE_TIME) { block->programmed_rate *= block->millimeters; }
  }

  // TODO: 从休息开始时，需要检查这种处理零结速度的方法。
  if ((block_buffer_head == block_buffer_tail) || (block->condition & PL_COND_FLAG_SYSTEM_MOTION)) {

	    //将块的入口速度初始化为零。假设它将从休息开始。策划者稍后会解决这个问题。
	    //如果系统运动，系统运动块始终假定从休息开始并在完全停止时结束。
    block->entry_speed_sqr = 0.0f;
    block->max_junction_speed_sqr = 0.0f; // 从休息开始 强制从零速度开始。

  } else {
	    //通过向心加速度近似计算交叉点的最大允许进入速度。
	    //让一个圆与先前和当前路径线段（交界处）相切
	    //偏差被定义为从结到最近边的距离，
	    //与圆心共线。连接两条路径的圆形部分代表
	    //向心加速度的路径 根据最大加速度求解最大速度
	    //圆的半径，由结点偏差间接定义。这可能也被视为
	    //之前的Grbl版本中的路径宽度或max_jerk。这种方法实际上并没有偏离
	    //来自路径，但用作计算转弯速度的可靠方法，因为它考虑到了
	    //结角和结速的非线性。
	    //
	    //注意：如果结点偏差值是有限的，则Grbl以确切的路径执行运动
	    //模式（G61）。如果结偏差值为零，则Grbl将精确地执行运动
	    //停止模式（G61.1）的方式。在未来，如果需要连续模式（G64），那么数学就在这里
	    //完全一样 机器将不会一直移动到联结点
	    //只需按照此处定义的弧形圆 Arduino没有CPU周期执行
	    //一个连续的模式路径，但基于ARM的微控制器肯定是这样做的。
	    //
	    //注意：最大结合速度是一个固定值，因为机器加速度限制不能
	    //在操作期间动态更改，也不能移动几何图形。这必须保存在
	    //进给速度倍率改变块的标称速度时的内存
	    //改变所有块的整体最大进入速度条件。

    float junction_unit_vec[N_AXIS];
    float junction_cos_theta = 0.0f;
    for (idx=0; idx<N_AXIS; idx++) {
      junction_cos_theta -= pl.previous_unit_vec[idx]*unit_vec[idx];
      junction_unit_vec[idx] = unit_vec[idx]-pl.previous_unit_vec[idx];
    }

    // 注意：计算没有任何昂贵的三角函数sin（）或acos（），通过三角函数cos（theta）的半角标识。
    if (junction_cos_theta > 0.999999f) {
      //  对于0度急转弯，只需设置最小结速。
      block->max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED;
    } else {
      if (junction_cos_theta < -0.999999f) {
        // 结点是一条直线或180度。连接速度是无限的。
        block->max_junction_speed_sqr = SOME_LARGE_VALUE;
      } else {
        convert_delta_vector_to_unit_vector(junction_unit_vec);
        float junction_acceleration = limit_value_by_axis_maximum(settings.acceleration, junction_unit_vec);
        float sin_theta_d2 = sqrtf(0.5f*(1.0f-junction_cos_theta)); // 触发半角身份。 总是积极的。
        block->max_junction_speed_sqr = max( MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED,
                       (junction_acceleration * settings.junction_deviation * sin_theta_d2)/(1.0f-sin_theta_d2) );
      }
    }
  }

  // 通过更新这个数据来阻止系统运动，以确保下一个g代码运动正确计算。
  if (!(block->condition & PL_COND_FLAG_SYSTEM_MOTION)) {
    float nominal_speed = plan_compute_profile_nominal_speed(block);//返回块的额定速度
    plan_compute_profile_parameters(block, nominal_speed, pl.previous_nominal_speed);//更新块的最大进入速度（sqr）
    pl.previous_nominal_speed = nominal_speed;

    //更新之前的路径unit_vector和规划器位置。
    memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec)); // pl.previous_unit_vec[] = unit_vec[]
    memcpy(pl.position, target_steps, sizeof(target_steps)); // pl.position[] = target_steps[]

    // 新块是全部设置。更新缓冲区头和下一个缓冲区头索引。
    block_buffer_head = next_buffer_head;
    next_buffer_head = plan_next_block_index(block_buffer_head);

    // 通过重新计算新块的计划完成。
    planner_recalculate();
  }
  return(PLAN_OK);
}


// 重置规划器位置向量。由系统中止/初始化例程调用。
void plan_sync_position()
{
  // TODO: 对于与机器位置不在同一坐标系中的电机配置，
  // 这个功能需要更新以适应差异。
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef COREXY
      if (idx==X_AXIS) {
        pl.position[X_AXIS] = system_convert_corexy_to_x_axis_steps(sys_position);
      } else if (idx==Y_AXIS) {
        pl.position[Y_AXIS] = system_convert_corexy_to_y_axis_steps(sys_position);
      } else {
        pl.position[idx] = sys_position[idx];
      }
    #else
      pl.position[idx] = sys_position[idx];
    #endif
  }
}


// 返回规划器缓冲区中可用块的数量。
uint8_t plan_get_block_buffer_available()
{
  if (block_buffer_head >= block_buffer_tail) { return((BLOCK_BUFFER_SIZE-1)-(block_buffer_head-block_buffer_tail)); }
  return((block_buffer_tail-block_buffer_head-1));
}


// 返回活动块的数量在规划器缓冲区中。
// 注意：弃用。除非在config.h中启用经典的状态报告，否则不要使用
uint8_t plan_get_block_buffer_count()
{
  if (block_buffer_head >= block_buffer_tail) { return(block_buffer_head-block_buffer_tail); }
  return(BLOCK_BUFFER_SIZE - (block_buffer_tail-block_buffer_head));
}


//用部分完成的块重新初始化缓冲区计划，假设存在于缓冲区尾部。
//在步进器完全停止进给保持并且循环停止后调用。
void plan_cycle_reinitialize()
{
  //完全停止重新计划 重置计划者入口速度和缓冲计划的指针。
  st_update_plan_block_parameters();
  block_buffer_planned = block_buffer_tail;
  planner_recalculate();
}
