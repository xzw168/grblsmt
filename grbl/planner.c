/*
  planner.c - �����ƶ����������ٶ�����ͼ

�Ӽ��ٹ滮��
������
1.���ͼӼ���
2.ǰհ
3.��϶ιս�����
*/

#include "grbl.h"


static plan_block_t block_buffer[BLOCK_BUFFER_SIZE];  // �˶�ָ��Ļ��λ�����
static uint8_t block_buffer_tail;     // ���ڴ���Ŀ������
static uint8_t block_buffer_head;     // ��һ��Ҫ�ƵĿ������
static uint8_t next_buffer_head;      // ��һ��������������
static uint8_t block_buffer_planned;  // ��Ѽƻ����������

// ����滮����
typedef struct {
  int32_t position[N_AXIS];          // ���Բ����еĹ��ߵļƻ���λ�á����ַֿ�
                                     // ��g����λ�ý�����Ҫ�����ƶ����ƶ���
                                     // ��Բ�����̶�ѭ���ͼ�϶������
  float previous_unit_vec[N_AXIS];   // ��ǰ·���߶εĵ�λ����
  float previous_nominal_speed;  // ��ǰ·���߶εı���ٶ�
} planner_t;
static planner_t pl;


// ���ػ��λ���������һ�����������Ҳ��Ϊ�����λ�������
uint8_t plan_next_block_index(uint8_t block_index)
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}


// ���ػ��λ�������ǰһ���������
static uint8_t plan_prev_block_index(uint8_t block_index)
{
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}


/*                                    �ƻ��ٶȶ���
                                     +--------+   <- ��ǰ->��ٶ�
                                    /          \
                                                           ��ǰ->�����ٶ� ->   +            \
                                   |             + <- ��һ��->�����ٶ� (�����˳��ٶ�)
                                   +-------------+
                                       time -->

  �������»���ָ�����¼����˶��ƻ���

    1. ���෴˳�����μ��ÿ�����еĿ飬���������ٶ�
        ����current-> entry_speed����ʹ�ã�
      a. û�н��ٶȳ���Ԥ�ȼ�������������ƻ����ڿ�ı���ٶȡ�
      b. һ������εĽ����ٶȲ��ܳ���һ����������ٶȣ�next-> entry_speed����������ķ������㣬�����ڳ�����г̾���������������ٶȡ�
      c. ���һ����������׷�ӵģ�������Ǵ�һ����ȫֹͣ������˳��ٶȣ��ƻ��ġ�
    2. ����ʱ��˳����ǰ��˳�����ÿһ���飬��������ǣ�����ͽ����ٶ�ֵ
      a. �����ٶȳ���������ٶ���ǰ����ĳ����ٶȣ��������г̾���������������ٶȡ�

  ����Щ�׶���ɺ󣬹滮��Ա����������ٶȷֲ�
  �Ĺ滮���飬ÿ���������������������������С���
  ���仰˵�����ڹ滮���е�����������˵������滮����ѵģ����Ҳ����һ������ٶ�
  �ǿ��ܵġ�����µĿ鱻��ӵ��������У�������������¼���üƻ�
  һ���µ���Ѽƻ���׼��

  Ϊ�������Щָ�ϵļ���Ч�ʣ�һ��滮ָ���Ѿ���ʹ��
  ���ƻ���׼���޷����߼��������κν�һ����ʱ�򱻴�����ָʾֹͣ�����
  ��������ʱ�ƻ��ĸı��Ľ����µĿ鱻��ʽ���䲢��ӵ��ƻ���
  �ƻ��߻����������磬����ƻ����е��������һ���Ӽ��Ѿ��ƻ����ˣ�
  ����󽻲���ٶȣ����ɵ�һ���滮���飩��Χ��û���µ�����
  ��ӵ��滮����������ı����е��ٶȷֲ����������ǲ�����Ҫ������
  ���ǡ����ߣ�����滮���е�һ�����һ�������飨������ֹͣ����
  �㣩�����٣����Ƕ�����ѵģ����ܱ���ӵ�һ���µĿ����
  �滮����������Ϊ�⽫ֻ���һ�����Ӽƻ��ٶȣ���ʱ��˳�����У�ֱ���ﵽ���ֵ
  ���ٶȴﵽ�����ǣ�����ƻ��Ĳ��������Ӳ������仯
  ʹ�õ�feed������������ǣ�ֹͣ����ָ�뽫�����ã������ƻ�
  ��һ��׼���������¼��㡣

  �滮������������ӳ�䣺
  -  block_buffer_tail��ָ��滮�������Ŀ�ʼ�����ȱ�ִ�л�ִ�С�
  -  block_buffer_head��ָ�򻺳��������һ����֮��Ļ���顣���ڱ�ʾ�Ƿ�
      ������������Ϊ�ա����׼���λ������������˿�ʼ��Ϊ�ա�
  next_buffer_head��ָ�򻺳�����������һ���滮�������顣���ڵ�ʱ��
      ����β�ͣ����ʾ������������
  -  block_buffer_planned��ָ�����������һ���Ż��ƻ���֮��ĵ�һ�������
      ��ý��������������ڹ滮�Ż����������¼��㲿��
      �����������滮�����������������¿����Ӷ��ı䡣���⣬
      ����鲻��С��block_buffer_tail���������Ǳ����ͺ�ά��
      ��һ������������plan_discard_current_block��������ʱ���������

  ע�⣺���ڹ滮��ֻ����滮�������е����ݣ�������Щ�����ܶ�
  �߶Σ���G2 / 3���߻������ߣ��ƺ��ƶ�������������Ϊ����û��
  ���������������н����㹻����Ͼ�����ٵ�����ٶȣ�Ȼ��
  ��ָ���������ڻ�������ĩ�˼��ٵ���ȫֹͣ������������������
  ��Ϊһ�����գ��м����򵥵Ľ����������1����󻯻������ٶȡ��ƻ���
  ���ܹ�����ͬ����Ͼ����ڼ�����ߵ��ٶȷֲ�����2�������·
  ��ÿ������˶�����������������ݲ�滮��ʹ�õľ���ԽԶ��
  �������ߵø��졣��3����󻯼ƻ���������С����Ҳ��������Ͼ���
  �ƻ��߼����������Ҳ�����˹滮�߱���ִ�еļ��������
  ����һ����ѵļƻ���������ϸѡ��Arduino��328p�ڴ��Ѿ������ˣ���δ��
  ARM�汾Ӧ�þ����㹻���ڴ���ٶȣ��Ա�������ٻ�����Ԥ���顣

*/
static void planner_recalculate()
{
  // ����������ʼ�����滮���������е����һ���顣
  uint8_t block_index = plan_prev_block_index(block_buffer_head);

  // ���� ֻ��һ���ƻ���������κ����顣
  if (block_index == block_buffer_planned) { return; }

  //���򴫵ݣ����Ե�������п��ܵļ�������
  //�ڻ����������� ֹͣ�ƻ�����������Ѽƻ���βָ���Ѵﵽ��
  //ע�⣺�������Ժ󽫸Ľ��������������Դ���һ����ѷ�����
  float entry_speed_sqr;
  plan_block_t *next;
  plan_block_t *current = &block_buffer[block_index];

  //���㻺���������һ������������ٶȣ����г����ٶ�����Ϊ�㡣
  current->entry_speed_sqr = min( current->max_entry_speed_sqr, 2*current->acceleration*current->millimeters);

  block_index = plan_prev_block_index(block_index);
  if (block_index == block_buffer_planned) { // ֻ�������ڻ���Һ�ɼƻ��顣���򴫵���ɡ�
    // ����һ�����Ƿ���β�� ����ǣ���֪ͨ�����������䵱ǰ������
    if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }
  } else { // ���������ļƻ����ܿ�
    while (block_index != block_buffer_planned) {
      next = current;
      current = &block_buffer[block_index];
      block_index = plan_prev_block_index(block_index);

      // �����һ�����Ƿ���β���飨=�ƻ��Ŀ飩������ǣ�����µ�ǰ�Ĳ���������
      if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }

      // �����������ٶȴӵ�ǰ����εĳ����ٶȼ��١�
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

  // ����ͨ�����Ӽƻ�ָ����ǰ�ƻ��������ߡ�
  // Ҳɨ����ѵļƻ��ϵ㲢�ʵ��ظ��¼ƻ���ָ�롣
  next = &block_buffer[block_buffer_planned]; // �ӻ������ƻ���ָ�뿪ʼ
  block_index = plan_next_block_index(block_buffer_planned);
  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];

    //������ͨ���м�⵽���κμ��ٶ����Զ��ƶ���Ѽƻ�
    //ָ��ǰ������Ϊ֮ǰ���������ݶ������ŵġ����仰˵��ûʲô
    //����ͨ���߼����ƴӻ���β���ƻ�ָ��ļƻ���
    if (current->entry_speed_sqr < next->entry_speed_sqr) {
      entry_speed_sqr = current->entry_speed_sqr + 2*current->acceleration*current->millimeters;
      // ���Ϊtrue����ǰ��Ϊȫ���٣����ǿ��Խ��ƻ���ָ����ǰ�ƶ���
      if (entry_speed_sqr < next->entry_speed_sqr) {
        next->entry_speed_sqr = entry_speed_sqr; // ����<= max_entry_speed_sqr�����Ĵ����趨����һ�㡣
        block_buffer_planned = block_index; // ������ѵļƻ�ָ�롣
      }
    }

    //����������ٶ����õ��κο�Ҳ����һ����ѵļƻ�
    //ָ�򻺳��������ƻ�����ͷ�ķ�����ʱ
    //����������������ٶȻ�������������ٶȣ�ÿ����֮��
    //���ܴ��߼��Ͻ�һ���Ľ�����ˣ����ǲ������¼��������ˡ�
    if (next->entry_speed_sqr == next->max_entry_speed_sqr) { block_buffer_planned = block_index; }
    block_index = plan_next_block_index( block_index );
  }
}


void plan_reset()
{
  memset(&pl, 0, sizeof(planner_t)); // ����滮���ṹ
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
  if (block_buffer_head != block_buffer_tail) { // �����ǿջ�������
    uint8_t block_index = plan_next_block_index( block_buffer_tail );
    // ��block_buffer_plannedָ�룬���������
    if (block_buffer_tail == block_buffer_planned) { block_buffer_planned = block_index; }
    block_buffer_tail = block_index;
  }
}


// ����ϵͳ�˶�ʹ�õĹ滮�������ĵ�ַ���ɶ����������á�
plan_block_t *plan_get_system_motion_block()
{
  return(&block_buffer[block_buffer_head]);
}


// ���ص�һ���ƻ������ĵ�ַ��������ã����ɸ�����Ҫ�����ܵ��á�
plan_block_t *plan_get_current_block()
{
  if (block_buffer_head == block_buffer_tail) { return(NULL); } // ������Ϊ��
  return(&block_buffer[block_buffer_tail]);
}


float plan_get_exec_block_exit_speed_sqr()
{
  uint8_t block_index = plan_next_block_index(block_buffer_tail);
  if (block_index == block_buffer_head) { return( 0.0 ); }
  return( block_buffer[block_index].entry_speed_sqr );
}


// ���ؿ黷�λ������Ŀ�����״̬����ģ�������ˡ�
uint8_t plan_check_full_buffer()
{
  if (block_buffer_tail == next_buffer_head) { return(true); }
  return(false);
}


// �������������͸���ֵ���㲢���ؿ�Ķ�ٶȡ�
// ע�⣺����ϵͳ�˶���������λ/ͣ�������ᱻ���ǡ�
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


// ���ݽ�����Сֵ���㲢���¿���������ٶȣ�sqr��
// ��ǰ�͵�ǰ�ı���ٶȺ������١�
static void plan_compute_profile_parameters(plan_block_t *block, float nominal_speed, float prev_nominal_speed)
{
  // ���ݽ���ٶȺ����ڱ���ٶȵ���Сֵ�����������ڡ�
  if (nominal_speed > prev_nominal_speed) { block->max_entry_speed_sqr = prev_nominal_speed*prev_nominal_speed; }
  else { block->max_entry_speed_sqr = nominal_speed*nominal_speed; }
  if (block->max_entry_speed_sqr > block->max_junction_speed_sqr) { block->max_entry_speed_sqr = block->max_junction_speed_sqr; }
}


// �ڻ����˶��ĸ��Ǹ���ʱ���¼��㻺����˶�����������
void plan_update_velocity_profile_parameters()
{
  uint8_t block_index = block_buffer_tail;
  plan_block_t *block;
  float nominal_speed;
  float prev_nominal_speed = SOME_LARGE_VALUE; // ����һ��ģ��Ķ�ٶȼ�������Ϊ�ߡ�
  while (block_index != block_buffer_head) {
    block = &block_buffer[block_index];
    nominal_speed = plan_compute_profile_nominal_speed(block);
    plan_compute_profile_parameters(block, nominal_speed, prev_nominal_speed);
    prev_nominal_speed = nominal_speed;
    block_index = plan_next_block_index(block_index);
  }
  pl.previous_nominal_speed = prev_nominal_speed; // ������һ��������prev����ٶȡ�
}


/* 
   ���һ���µ������˶�����������Ŀ��[N_AXIS]���з��ŵľ���Ŀ��λ��
   �Ժ���Ϊ��λ��������ָ���˶����ٶȡ�����������ʵ��ã������
   ������ָ��Ƶ�ʡ���������1 / feed_rate��������ɲ�����
   ���ݸ��ƻ��ߵ�����λ�����ݱ����Ի���λ�������ּƻ���
   ��������g���������������κ�����ϵ�仯��ƫ������
   ע�⣺���軺�������á������������motion_control�ڸ��߼�����
   ���仰˵��������ͷ��Զ������ڻ�����β����Ҳ�ǽ���������ֵ
   �����ַ�ʽʹ�ã����invert_feed_rateΪ�٣�����Ϊ�����������ʣ�����ǣ�����Ϊ��ʱ��
   ���feed_rateֵΪ�������͡�������reverse_feed_rateΪtrue������Ϊseek / rapids rate
   invert_feed_rate����Ϊfalse����
   ϵͳ�˶�״̬֪ͨ�ƻ���������δʹ�õĿ黺�����мƻ��˶�
   ͷ���������˸ı�滮����״̬������������������ȷ��������gcode
   �˶���Ȼ��ȷ�ƻ���������ģ��ָֻ��黺����ͷ
   ִ�������ϵͳ������ 
*/
// ���������block_buffer[block_buffer_head]
// ����ok ����Ҫ�ƶ��տ�
uint8_t plan_buffer_line(float *target, plan_line_data_t *pl_data)
{
  // ׼������ʼ���¿顣������ص�pl_data��ִ�п顣
  plan_block_t *block = &block_buffer[block_buffer_head];
  memset(block,0,sizeof(plan_block_t)); // �������еĿ�ֵ��
  block->condition = pl_data->condition;//λ��־������ʾ�ƻ�Ա����
  #ifdef VARIABLE_SPINDLE
    block->spindle_speed = pl_data->spindle_speed;
  #endif
  #ifdef USE_LINE_NUMBERS
    block->line_number = pl_data->line_number;
  #endif

  // ���㲢�洢��ʼ�ƶ��������ݡ�
  int32_t target_steps[N_AXIS], position_steps[N_AXIS];
  float unit_vec[N_AXIS], delta_mm;
  uint8_t idx;

  // ���ݼƻ����˶����͸���λ�����ݡ�
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
  //����������������߶��ٺ��ף�����������
  for (idx=0; idx<N_AXIS; idx++) {
    // �Ծ��Բ�������Ŀ��λ�ã�Ϊÿ������㲽������ȷ����󲽽��¼���
    // ���⣬�����ƶ���׼����λʸ������ĸ�����ľ��롣
    // ע�⣺��ת����Ĳ���ֵ������ʵ���롣
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
      //�����Ҫ��������
	  target_steps[idx] = lroundf(target[idx]*settings.steps_per_mm[idx]);//������������ӽ�x������ ȥ��С��
	  //���ṲҪ�߶������  ��Ŀ��-��ǰ��
      block->steps[idx] = abs(target_steps[idx]-position_steps[idx]);//�������ľ���ֵ,����ȥ������-100=100 0=0 100=100
	  //��������������������ı�����������ǰ���ϴαȽϣ�
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
	  //���ṲҪ�߶��ٺ���+-mm
      delta_mm = (target_steps[idx] - position_steps[idx])/settings.steps_per_mm[idx];
	#endif
    unit_vec[idx] = delta_mm; // �洢��λʸ������

    // ���ñ��᷽������λ ÿ��λ����һ����
    if (delta_mm < 0.0f ) { block->direction_bits |= direction_pin_mask[idx]; }
  }

  // �������Ҫ�ƶ����˳����ؿտ�
  if (block->step_event_count == 0) { return(PLAN_EMPTY_BLOCK); }

  //�������ƶ��ĵ�λʸ���Ϳ�����������ʺͼ��ٶ�
  //���£�ʹ��������߷���û�г�������������ֵ��
  //ע�⣺�������ٶ����е��ᶼ�������ģ��ѿ�����������ABC��һ������
  //�������Ҳ��������/�����ġ��ڵ�λʸ���ľ���ֵ�����С�

  //��ƽ������߶οռ����ƶ��ľ���
  block->millimeters = convert_delta_vector_to_unit_vector(unit_vec);//������ʸ��ת��Ϊ��λʸ�� ��ı�unit_vec��ֵ
  block->acceleration = limit_value_by_axis_maximum(settings.acceleration, unit_vec);
  block->rapid_rate = limit_value_by_axis_maximum(settings.max_rate, unit_vec);

  // �洢��̵����ʡ�
  if (block->condition & PL_COND_FLAG_RAPID_MOTION) { block->programmed_rate = block->rapid_rate; }
  else { 
    block->programmed_rate = pl_data->feed_rate;
    if (block->condition & PL_COND_FLAG_INVERSE_TIME) { block->programmed_rate *= block->millimeters; }
  }

  // TODO: ����Ϣ��ʼʱ����Ҫ������ִ�������ٶȵķ�����
  if ((block_buffer_head == block_buffer_tail) || (block->condition & PL_COND_FLAG_SYSTEM_MOTION)) {

	    //���������ٶȳ�ʼ��Ϊ�㡣������������Ϣ��ʼ���߻����Ժ����������⡣
	    //���ϵͳ�˶���ϵͳ�˶���ʼ�ռٶ�����Ϣ��ʼ������ȫֹͣʱ������
    block->entry_speed_sqr = 0.0f;
    block->max_junction_speed_sqr = 0.0f; // ����Ϣ��ʼ ǿ�ƴ����ٶȿ�ʼ��

  } else {
	    //ͨ�����ļ��ٶȽ��Ƽ��㽻���������������ٶȡ�
	    //��һ��Բ����ǰ�͵�ǰ·���߶Σ����紦������
	    //ƫ�����Ϊ�ӽᵽ����ߵľ��룬
	    //��Բ�Ĺ��ߡ���������·����Բ�β��ִ���
	    //���ļ��ٶȵ�·�� ���������ٶ��������ٶ�
	    //Բ�İ뾶���ɽ��ƫ���Ӷ��塣�����Ҳ����Ϊ
	    //֮ǰ��Grbl�汾�е�·����Ȼ�max_jerk�����ַ���ʵ���ϲ�û��ƫ��
	    //����·��������������ת���ٶȵĿɿ���������Ϊ�����ǵ���
	    //��Ǻͽ��ٵķ����ԡ�
	    //
	    //ע�⣺������ƫ��ֵ�����޵ģ���Grbl��ȷ�е�·��ִ���˶�
	    //ģʽ��G61���������ƫ��ֵΪ�㣬��Grbl����ȷ��ִ���˶�
	    //ֹͣģʽ��G61.1���ķ�ʽ����δ���������Ҫ����ģʽ��G64������ô��ѧ��������
	    //��ȫһ�� ����������һֱ�ƶ��������
	    //ֻ�谴�մ˴�����Ļ���Բ Arduinoû��CPU����ִ��
	    //һ��������ģʽ·����������ARM��΢�������϶����������ġ�
	    //
	    //ע�⣺������ٶ���һ���̶�ֵ����Ϊ�������ٶ����Ʋ���
	    //�ڲ����ڼ䶯̬���ģ�Ҳ�����ƶ�����ͼ�Ρ�����뱣����
	    //�����ٶȱ��ʸı��ı���ٶ�ʱ���ڴ�
	    //�ı����п�������������ٶ�������

    float junction_unit_vec[N_AXIS];
    float junction_cos_theta = 0.0f;
    for (idx=0; idx<N_AXIS; idx++) {
      junction_cos_theta -= pl.previous_unit_vec[idx]*unit_vec[idx];
      junction_unit_vec[idx] = unit_vec[idx]-pl.previous_unit_vec[idx];
    }

    // ע�⣺����û���κΰ�������Ǻ���sin������acos������ͨ�����Ǻ���cos��theta���İ�Ǳ�ʶ��
    if (junction_cos_theta > 0.999999f) {
      //  ����0�ȼ�ת�䣬ֻ��������С���١�
      block->max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED;
    } else {
      if (junction_cos_theta < -0.999999f) {
        // �����һ��ֱ�߻�180�ȡ������ٶ������޵ġ�
        block->max_junction_speed_sqr = SOME_LARGE_VALUE;
      } else {
        convert_delta_vector_to_unit_vector(junction_unit_vec);
        float junction_acceleration = limit_value_by_axis_maximum(settings.acceleration, junction_unit_vec);
        float sin_theta_d2 = sqrtf(0.5f*(1.0f-junction_cos_theta)); // ���������ݡ� ���ǻ����ġ�
        block->max_junction_speed_sqr = max( MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED,
                       (junction_acceleration * settings.junction_deviation * sin_theta_d2)/(1.0f-sin_theta_d2) );
      }
    }
  }

  // ͨ�����������������ֹϵͳ�˶�����ȷ����һ��g�����˶���ȷ���㡣
  if (!(block->condition & PL_COND_FLAG_SYSTEM_MOTION)) {
    float nominal_speed = plan_compute_profile_nominal_speed(block);//���ؿ�Ķ�ٶ�
    plan_compute_profile_parameters(block, nominal_speed, pl.previous_nominal_speed);//���¿���������ٶȣ�sqr��
    pl.previous_nominal_speed = nominal_speed;

    //����֮ǰ��·��unit_vector�͹滮��λ�á�
    memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec)); // pl.previous_unit_vec[] = unit_vec[]
    memcpy(pl.position, target_steps, sizeof(target_steps)); // pl.position[] = target_steps[]

    // �¿���ȫ�����á����»�����ͷ����һ��������ͷ������
    block_buffer_head = next_buffer_head;
    next_buffer_head = plan_next_block_index(block_buffer_head);

    // ͨ�����¼����¿�ļƻ���ɡ�
    planner_recalculate();
  }
  return(PLAN_OK);
}


// ���ù滮��λ����������ϵͳ��ֹ/��ʼ�����̵��á�
void plan_sync_position()
{
  // TODO: ���������λ�ò���ͬһ����ϵ�еĵ�����ã�
  // ���������Ҫ��������Ӧ���졣
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


// ���ع滮���������п��ÿ��������
uint8_t plan_get_block_buffer_available()
{
  if (block_buffer_head >= block_buffer_tail) { return((BLOCK_BUFFER_SIZE-1)-(block_buffer_head-block_buffer_tail)); }
  return((block_buffer_tail-block_buffer_head-1));
}


// ���ػ��������ڹ滮���������С�
// ע�⣺���á�������config.h�����þ����״̬���棬����Ҫʹ��
uint8_t plan_get_block_buffer_count()
{
  if (block_buffer_head >= block_buffer_tail) { return(block_buffer_head-block_buffer_tail); }
  return(BLOCK_BUFFER_SIZE - (block_buffer_tail-block_buffer_head));
}


//�ò�����ɵĿ����³�ʼ���������ƻ�����������ڻ�����β����
//�ڲ�������ȫֹͣ�������ֲ���ѭ��ֹͣ����á�
void plan_cycle_reinitialize()
{
  //��ȫֹͣ���¼ƻ� ���üƻ�������ٶȺͻ���ƻ���ָ�롣
  st_update_plan_block_parameters();
  block_buffer_planned = block_buffer_tail;
  planner_recalculate();
}
