

/*
  stepper.c - ���������������ʹ�ò������ִ���˶��ƻ�
*/

#include "grbl.h"

#ifdef STM32F103C8
typedef int bool;
#include "stm32f10x_rcc.h"
//#include "stm32f10x_tim.h"
#include "misc.h"
void TIM_Configuration(TIM_TypeDef* TIMER, u16 Period, u16 Prescaler, u8 PP);
#endif


// һЩ���õĳ���.
#define DT_SEGMENT (1.0f/(ACCELERATION_TICKS_PER_SECOND*60.0f)) // ����/��
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

//��������Ӧ���Ჽ��ƽ����AMASS����ƽ�ͽ�ֹƵ�ʡ���ߵ�ˮƽ
//Ƶ�ʿ�ʼ��0Hz�����Խ�ֹƵ�ʽ�������һ���ϵͼ����Ƶ�ʲ�
//����һ�����ߵĽ�ֹƵ�ʿ�ʼ���������ơ�ÿ������Ľ�ֹƵ�ʱ���
//Ҫ��ϸ��������������������ISR�ľ����Ƕ���
//��ʱ����CPU�������ȼ�0����AMASS������������Ƶ�ʲֿ�ʼ��
//����1�Ľ�ֹƵ�ʣ���߿ɴ�CPU�����Ƶ�ʣ������޵Ĳ����г���30kHz����
//ע�⣺AMASS��ֹƵ�ʳ���ISR���������Ӳ��ó�����󲽽�Ƶ�ʡ�
//ע�⣺��ǰ���ñ�����Ϊ��ISR��������������16kHz���Ӷ�ƽ��CPU����
//�ͼ�ʱ����׼ȷ�ԡ�������֪��������ʲô������Ҫ�ı���Щ���á�
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
	#define MAX_AMASS_LEVEL 3
	// AMASS_LEVEL0: �������С�û��AMASS��û�����޽�ֹƵ�ʡ���LEVEL1��ֹƵ�ʿ�ʼ��
	#define AMASS_LEVEL1 (F_CPU/8000) // ��������ISR��x2��������ΪF_CPU /����HzΪ��λ�Ľ�ֹƵ�ʣ�
	#define AMASS_LEVEL2 (F_CPU/4000) // Over-drives ISR (x4)
	#define AMASS_LEVEL3 (F_CPU/2000) // Over-drives ISR (x8)

  #if MAX_AMASS_LEVEL <= 0
    error "AMASS must have 1 or more levels to operate correctly."
  #endif
#endif



//�洢���жεĹ滮��Bresenham�㷨ִ������
//��������ͨ������£����������������ʹ���У����ǣ�����������������
//��Զ���ᳬ���ɷ��ʵĲ����������ε�������SEGMENT_BUFFER_SIZE-1����
//ע�⣺��Щ�����Ǵ�prepped planner���и��Ƶģ������滮��Ϳ�����
//���λ�������ȫ���Ĳ����ʱ���������⣬AMASS�ı�����һ��
//���ݹ��Լ�ʹ�á�
typedef struct {
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;//�����¼�����
  uint8_t direction_bits;//����λ
  #ifdef VARIABLE_SPINDLE
    uint8_t is_pwm_rate_adjusted; // ׷����Ҫ�㶨���⹦��/���ʵ��˶�
  #endif
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];

//���������λ��λ����� ������������С�Ķ��߶�
//ִ�е��㷨���ӵ�һ���鿪ʼ�𽥡������
//�ƻ��������� һ������������λ������еĲ��費�ܱ��޸�
//�߻��ߣ�ʣ�µĲ߻��߿鲽����Ȼ���ԡ�
typedef struct {
  uint16_t n_step;           // �����Ҫִ�еĲ����¼�������
  uint16_t cycles_per_tick;  // ÿ��ISR��વĲ��࣬Ҳ���ǲ�����
  uint8_t  st_block_index;   // ����������������ʹ�ô���Ϣ��ִ�д˶Ρ�
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;    // ��ʾISRִ�иöε�AMASS����
  #else
    uint8_t prescaler;      // û��AMASS����ҪԤ��Ƶ����������ʱ��
  #endif
  #ifdef VARIABLE_SPINDLE
    uint8_t spindle_pwm;
  #endif
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// ����ISR���ݽṹ��������������ISR���������ݡ�
typedef struct {
  // ��bresenham���㷨ʹ��
  uint32_t counter_x,        // ����bresenham�߸������ļ���������
           counter_y,
           counter_z;
  #ifdef STEP_PULSE_DELAY
    uint8_t step_bits;  // �洢out_bits�������ɽ�Ծ�����ӳ�
  #endif

  uint8_t execute_step;     // ��־Ϊÿ���ж�ִ��һ�Ρ�

  uint8_t step_pulse_time;  // ����������Ĳ������帴λʱ��

  PORTPINDEF step_outbits;         // �����һ������λ
  PORTPINDEF dir_outbits;
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
  #endif

  uint16_t step_count;       // �߶��˶���ʣ��Ĳ���
  uint8_t exec_block_index; // ���ٵ�ǰ��st_block���������ı�ʾ�¿顣
  st_block_t *exec_block;   // ָ������ִ�еĶεĿ����ݵ�ָ��
  segment_t *exec_segment;  // ָ������ִ�еĶε�ָ��
} stepper_t;
static stepper_t st;

//�����λ��λ���������. Step segment ring buffer indices
static volatile uint8_t segment_buffer_tail; //�λ���β
static uint8_t segment_buffer_head;//�λ���ͷ
static uint8_t segment_next_head;

//�����ͷ���˿ڷ�ת����  Step and direction port invert masks.
static PORTPINDEF step_port_invert_mask;
static PORTPINDEF dir_port_invert_mask;

// ���ڱ���ISRǶ�ס������������жϡ��� Ӧ����Զ���ᷢ����
static volatile uint8_t busy;

// ָ������Ƭ�ε�ָ��ӹ滮��������׼���á�ֻ��ͨ������
// ������ ָ��������ڼƻ�ִ�еĲ��ֻ�ƻ���.
static plan_block_t *pl_block;     // ָ������׼���ļƻ�������ָ��
static st_block_t *st_prep_block;  // ָ������׼���Ĳ��������ݵ�ָ��

// ��׼�����ݽṹ������������ϸ�ֵ����б�Ҫ��Ϣ
// ���ڵ�ǰ����ִ�еļƻ�����顣
typedef struct {
  uint8_t st_block_index;  // ׼���ò���ͨ�����ݿ������
  uint8_t recalculate_flag;

  float dt_remainder;
  float steps_remaining;
  float step_per_mm;
  float req_mm_increment;

  #ifdef PARKING_ENABLE //ͣ��ʹ��
    uint8_t last_st_block_index;
    float last_steps_remaining;
    float last_step_per_mm;
    float last_dt_remainder;
  #endif

  uint8_t ramp_type;      // ��ǰ�Ķ�б��״̬
  float mm_complete;      // �Ե�ǰ�ƻ������������ٶ����߽�����mm����
                          // ע�⣺���ֵ��ת��ʱ������һ�����裨��β�����غϡ�
  float current_speed;    // �λ�����ĩβ�ĵ�ǰ�ٶȣ�mm / min��
  float maximum_speed;    // ִ�п������ٶȡ��������Ǳ���ٶȡ�������/���ӣ�
  float exit_speed;       // ִ�г���ε��˳��ٶȣ�mm / min��
  float accelerate_until; // �ӳ����ĩ�˿�ʼ�����ļ���б���յ㣨mm��
  float decelerate_after; // �ӳ���ν���������ļ���б�¿�ʼ��mm��

  #ifdef VARIABLE_SPINDLE
    float inv_rate;    // ��PWM����ģʽʹ�������ٷֶμ��㡣
    uint8_t current_spindle_pwm;
  #endif
} st_prep_t;
static st_prep_t prep;


/*    ���ٶ���������
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            ^      BLOCK 2          |    d
                                       |
                  ʱ�� ----->      ���磺����2�����ٶ���������ٶ�

  �ƻ�����黺�����ƻ�����㶨���ٶ��ٶ������ļ���
  ������ʾ������������·�ڡ����ǣ��߻���ֻ�ܻ�������
  ������ٶ�Ϊ����ٶȼƻ�������������ڲ�
  �ٶ����ߡ���Щ�ٶ�������������ִ�е�ר�ż����
  �����㷨������ֻ����7�ֿ��ܵ����ͣ�Ѳ����Ѳ�� -
  ���٣�����Ѳ�������ٶȣ������٣�ȫ���κ�
  �����Σ�û��Ѳ������

                                                                                                                                                                        ����ٶ� (< ��ٶ�) ->  +
                    +--------+ <- ����ٶ� (= ��ٶ�)                          /|\
                   /          \                                           / | \
                         ��ǰ�ٶ�    -> +            \                                         /  |  + <- exit_speed
                  |             + <- exit_speed                         /   |  |
                  +-------------+                             ��ǰ�ٶ�  -> +----+--+
                   time -->  ^  ^                                           ^  ^
                             |  |                                           |  |
                                                                                ���ٺ�(����Ϊ��λ)                                  ���ٺ�(����Ϊ��λ)
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                                                              ����ֱ��(����Ϊ��λ)                                        ����ֱ��(����Ϊ��λ)

  ���λ���������ִ�п��ٶ����߲����ٹؼ���
  �����㷨�Ĳ�����׼ȷ�ظ�����������Щ�ؼ�����
  ����ͼ����ʾ�Ͷ��塣
*/


//����״̬��ʼ�� ����Ӧ��ֻ��st.cycle_start��־��ʱ�ſ�ʼ
//���á�����init�����Ƶ���������������ǲ�Ӧ���������ѭ����
void st_wake_up()
{
  // ���ò�����������
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) 
  { 
	  SetStepperDisableBit();
  }
  else 
  { 
	  ResetStepperDisableBit(); 
  }

  // ��ʼ���������λ����ȷ����һ��ISR���ò���ִ�С�
  st.step_outbits = step_port_invert_mask;

  // �����ó�ʼ�����������ʱ������ȷ����д����¡�
  #ifdef STEP_PULSE_DELAY
    // ���÷������ź������ܲ�������ʱ�䡣ʾ������������㡣
    st.step_pulse_time = -(((settings.pulse_microseconds+STEP_PULSE_DELAY-2)*TICKS_PER_MICROSECOND) >> 3);
    // ���÷�������д��Ͳ�������֮����ӳ١�
    OCR0A = -(((settings.pulse_microseconds)*TICKS_PER_MICROSECOND) >> 3);
  #else // ��������
    // ���ò�����ʱ�� ʾ������������㡣ʹ�ö�����

#if defined(STM32F103C8)
  st.step_pulse_time = (settings.pulse_microseconds)*TICKS_PER_MICROSECOND;
#endif
  #endif

  // ���ò����������ж�

#if defined (STM32F103C8)
  TIM3->ARR = st.step_pulse_time - 1;
  TIM3->EGR = TIM_PSCReloadMode_Immediate;
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

  TIM2->ARR = st.exec_segment->cycles_per_tick - 1;
  /* �����Զ�����ֵ */
#ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING        
  TIM2->PSC = st.exec_segment->prescaler;
#endif
  TIM2->EGR = TIM_PSCReloadMode_Immediate;
  NVIC_EnableIRQ(TIM2_IRQn);
#endif
}


// �����ػ�
void st_go_idle()
{
  // ���ò����������жϡ����������������˿ڸ�λ�ж���ɡ�

#ifdef STM32F103C8
  NVIC_DisableIRQ(TIM2_IRQn);
#endif

  busy = false;

  // �������ú�������ò�������������״̬�����û����á�
  bool pin_state = false; // ��������״̬
  if (((settings.stepper_idle_lock_time != 0xff) || sys_rt_exec_alarm || sys.state == STATE_SLEEP) && sys.state != STATE_HOMING) {
    // ǿ�Ʋ�������ͣ������һ����ʱ�䣬��ȷ��������
    // �����һ���ƶ�����ʱֹͣ���Ҳ����ʣ�������Ư�ơ�
    delay_ms(settings.stepper_idle_lock_time);
    pin_state = true; // ���� ���ò�������
  }
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { pin_state = !pin_state; } //Ӧ�����ŷ�ת
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
 �������������жϡ� - �����ʱ���ж���Grbl��������Grbl��Ӷ
   ��ʷ�ƾõ�Bresenham���㷨������;�ȷͬ�������ƶ���
   �����е�DDA�㷨��ͬ��Bresenham�㷨��������ֵӰ��
   �����������ֻ��Ҫ��������������������ζ�ŵͼ��㿪��
   �����Arduino�Ĺ��ܡ�Ȼ����Bresenham�㷨��ȱ��
   ����ĳЩ�����˶����ԣ�������������ܲ�ƽ���Ĳ���
   ����𳵻��ݣ�����ܻᵼ����ֵ��������𶯡�����
   �ر����ԵĻ���ܵ��µͲ�Ƶ�ʣ�0-5kHz�����˶�??���⣬����
   ͨ�����Ǹ�Ƶ���������⣬��Ȼ����������
     Ϊ�����Bresenham�Ķ������ܣ�Grblʹ��������˵������Ӧ����
   ����ƽ����AMASS���㷨������������ζ��ʲô���ڽϵ͵Ĳ�Ƶ�£�
   AMASS��Ϊ��������Bresenham�ֱ��ʶ���Ӱ���㷨
   ��������ȷ�ԡ�AMASS���ݲ����Զ�������ֱ��ʼ���
   Ҫִ�е�Ƶ�ʣ���ζ�Ŷ����������͵Ĳ���Ƶ�ʣ�����ƽ��
   ˮƽ��ߡ����㷨�Ͻ���AMASS��ͨ��Bresenham�ļ�λ����ʵ�ֵ�
   ÿ��AMASS����Ĳ��������磬����һ��ƽ��������λ��
   Bresenham�����¼���������Ч�س���2�����Ჽ��
   ���ֲ��䣬Ȼ��ӱ�����ISRƵ�ʡ�ʵ���ϣ��������������
   ������Bresenham���������м�ISR�̶����ϣ���������������
   ÿ������ISR���ۣ������Ǵ�ͳ�����ϵ�ÿ��ISR���ۡ���AMASS
   �ȼ�2������ֻ���ٴ���λ�����Է�������Bresenham��������κ�һ�������ڽ���
   ���ĸ�ISR�����У�������ÿ�ĸ�ISR����һ�����ı�
   ����ISRƵ�ʡ��ȵȡ�ʵ���ϣ���ʵ���������˶�����
   ������Bresenham�㷨��û�����Ÿı�Grbl�ı��֣�����
   ʵ���ϣ������������и���Ч������δʹ�õ�CPU���ڡ�
     AMASSͨ��Ҫ����ʼ��ִ��������������Bresenham�㷨����ȷ��
   Bresenhamһ��������AMASS��ˮƽ������ζ�Ŷ���һ��AMASS������ȫ���ĸ�
   ��������м䲽�裬ʹ����Bresenham��0������ʼ�ձ��ֲ���
   ������ͬ����AMASS Level 3��ζ�����а˸��м䲽����뱻ִ�С�
   ����AMASSˮƽʵ����������ģ���Bresenham�Ļ������ݿ���
   �����κ�����ֵ�����Զ�����ֻ�Ǽ򵥵���������
   ����bitshift����������CPU������
     ����ж��Ǽ򵥶��޴�����ơ����еļ�����أ�����
   ȷ�����ٶȣ��������ط�ִ�С�����жϵ���Ԥ�ȼ���ĶΣ�
   ����Ϊ�ڲ���������֮���n�������еĺ㶨�ٶ�
   ͨ��Bresenham�㷨�ʵ������岽��������ִ�����ǡ����
   ISR��Stepper�˿ڸ�λ�ж�֧�֣���������λ�������˿�
   ÿ������֮��bresenham��ʾ���㷨�������в������
   ���������ж�ͬʱ���С�
   ע�⣺���жϱ��뾡���ܸ�Ч��������һ��ISR��֮ǰ��ɣ�
   Grbl����С��33.3usec��@ 30kHz ISR���ʣ���ʾ��������ʱ��
   ISR�ǵ��͵�5usec�����25usec��Զ����Ҫ��
   ע�⣺���ISR����ÿ��������ִ��һ�����衣
*/
// TODO: ��ĳ�ַ�ʽ�滻ISR�е�int32λ�ü�������ֱ�Ӹ��¡�Ҳ��ʹ�ø�С
// int8�����͸���λ�ü�����ֻ�е�һ������ɡ�����ܱ�ø���
// ��Ҫ������ʵʱλ�õ�̽��͹�λ���ڡ�

void TIM2_IRQHandler(void)
{
#ifdef STM32F103C8
	if ((TIM2->SR & 0x0001) != 0)                  // ���ж�Դ
	{
		TIM2->SR &= ~(1 << 0);                          // ���UIF��־
		TIM2->CNT = 0;
	}
	else
	{
		return;
	}
#endif

  if (busy) { return; } // æ��־�����������½�������ж�

#ifdef STM32F103C8
  GPIO_Write(DIRECTION_PORT, (GPIO_ReadOutputData(DIRECTION_PORT) & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK));
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
#endif

  // Ȼ�����岽������
  #ifdef STEP_PULSE_DELAY
    st.step_bits = (STEP_PORT & ~STEP_MASK) | st.step_outbits; // �洢out_bits�Է�ֹ���ǡ�
  #else  // ��������
#ifdef STM32F103C8
	GPIO_Write(STEP_PORT, (GPIO_ReadOutputData(STEP_PORT) & ~STEP_MASK) | st.step_outbits);
#endif
  #endif

  // ���ò������帴λ��ʱ����ʹ�ò����˿ڸ�λ�жϿ����ڸ�λ��λ�ź�
  // ��ȷ������Ϊ.pulse_microseconds΢�룬��������Timer1Ԥ��Ƶ����
#ifdef STM32F103C8
  NVIC_EnableIRQ(TIM3_IRQn);
#endif

  busy = true;

  // ���û�в���Σ����ԴӲ����������е���һ��
  if (st.exec_segment == NULL) {
    // �������е��κζ�������������������ز���ʼ����һ���Ρ�
    if (segment_buffer_head != segment_buffer_tail) {
      // ��ʼ���µĲ���β�����Ҫִ�еĲ�����
      st.exec_segment = &segment_buffer[segment_buffer_tail];

      // ��ʼ��ÿ���Ĳ����ʱ�䲢����Ҫִ�еĲ�������
#ifdef STM32F103C8
	  TIM2->ARR = st.exec_segment->cycles_per_tick - 1;
	  /* �����Զ�����ֵ */
#ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING        
	  TIM2->PSC = st.exec_segment->prescaler;
#endif
#endif
      st.step_count = st.exec_segment->n_step; // ע�⣺�ƶ��ٶ���ʱ����Ϊ�㡣
      // ����µĶο�ʼһ���µĹ滮�飬��ʼ�����������ͼ�������
      // ע�⣺���ֶ����������ı�ʱ�����ʾ�µļƻ�����顣
      if ( st.exec_block_index != st.exec_segment->st_block_index ) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];

        // ��ʼ��Bresenham�ߺ;��������
        st.counter_x = st.counter_y = st.counter_z = (st.exec_block->step_event_count >> 1);
      }
      st.dir_outbits = st.exec_block->direction_bits ^ dir_port_invert_mask;

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // ����AMASS�󣬸���AMASS�ȼ�����Bresenham��������������
        st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.exec_segment->amass_level;
        st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.exec_segment->amass_level;
        st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.exec_segment->amass_level;
      #endif

      #ifdef VARIABLE_SPINDLE
        // �ڼ��ض�ʱ����ʵʱ������������ڵ�һ��֮ǰ��
        spindle_set_speed(st.exec_segment->spindle_pwm);
      #endif

    } else {
      // �λ�����Ϊ�� �ص���
      st_go_idle();
      // ������ʿ����˶���ȷ��pwm������ȷ��
      #ifdef VARIABLE_SPINDLE
      if (st.exec_block->is_pwm_rate_adjusted) { spindle_set_speed(SPINDLE_PWM_OFF_VALUE); }
      #endif
      system_set_exec_state_flag(EXEC_CYCLE_STOP); // ������ڽ�����������
      return; // û��ʲôҪ�������˳���
    }
  }


  // ���̽��״̬
  if (sys_probe_state == PROBE_ACTIVE) { probe_state_monitor(); }

  //����ʧ��λ�� Reset step out bits.
  st.step_outbits = 0;

  // ͨ��Bresenham���㷨ִ�в���λ������
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

  // �ڹ�λѭ�������У���������ֹ��������ƶ���
  if (sys.state == STATE_HOMING) { st.step_outbits &= sys.homing_axis_lock; }

  st.step_count--; // ���ٲ����¼�����
  if (st.step_count == 0) {
    // ϸ����� ������ǰ�ֶκ���ǰ�ֶ�������
    st.exec_segment = NULL;

	uint8_t segment_tail_next = segment_buffer_tail + 1;
	if (segment_tail_next == SEGMENT_BUFFER_SIZE)
		segment_tail_next = 0;
	segment_buffer_tail = segment_tail_next;

  }

  st.step_outbits ^= step_port_invert_mask;  // Ӧ�ò���˿ڷ�ת����
  busy = false;
}


/*
  �������˿ڸ�λ�жϣ�Timer0 OVF�жϴ���ò�����½���
   ���塣��Ӧ��ʼ������һ��Timer1 COMPA�ж�֮ǰ��������������
   ��ɺ��������ƶ���Timer1�����á�
   ע�⣺���кͲ����ж�֮����жϳ�ͻ���ܵ����ӳ�
   ��΢�룬�������ִ��һ������һ��֮ǰ������ʲô���£�������
   �����һ����Ƶ�첽�жϵ��¸߲�����������
   ��ӵ�Grbl��
*/
// �����õ���˿�λִ��ʱ�����ж���ISR_TIMER1_COMPAREAʹ��
// һ������ ��ISR�ڶ�ʱ������õ���˿ڣ�settings.pulse_microseconds��
// ���һ��ѭ��

void TIM3_IRQHandler(void)
{
#ifdef STM32F103C8
	if ((TIM3->SR & 0x0001) != 0)                  // ����ж�Դ
	{
		TIM3->SR &= ~(1<<0);                          // ���UIF��־
		TIM3->CNT = 0;
		NVIC_DisableIRQ(TIM3_IRQn);
		GPIO_Write(STEP_PORT, (GPIO_ReadOutputData(STEP_PORT) & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK));
	}
#endif

}
#ifdef STEP_PULSE_DELAY
//����жϽ���STEP_PULSE_DELAY��ʹ��ʱʹ�á������Ծ������
//��STEP_PULSE_DELAYʱ��ν�����������ISR TIMER2_OVF�ж�
//Ȼ�������������֮�󴥷���Ӧ��settings.pulse_microseconds��
//�ڷ��򣬲�������Ͳ�������¼�֮�����ʱ��������
// st_wake_up�������̡�
  ISR(TIMER0_COMPA_vect)
  {
    STEP_PORT = st.step_bits; // ��ʼ��������
  }
#endif


// ���ɲ����ж�����������ʹ�õĲ����ͷ���˿ڷ�ת���롣
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


// ���ò����������ϵͳ����
void st_reset()
{
  // ��ʼ����������������״̬��
  st_go_idle();

  // ��ʼ�������㷨������
  memset(&prep, 0, sizeof(st_prep_t));
  memset(&st, 0, sizeof(stepper_t));
  st.exec_segment = NULL;
  pl_block = NULL;  // �λ�����ʹ�õĹ滮��ָ��
  segment_buffer_tail = 0;
  segment_buffer_head = 0; // empty = tail
  segment_next_head = 1;
  busy = false;

  st_generate_step_dir_invert_masks();
  st.dir_outbits = dir_port_invert_mask; // ������λ��ʼ��ΪĬ��ֵ

  // ��ʼ�������ͷ���˿����š�
#ifdef STM32F103C8
  GPIO_Write(STEP_PORT, (GPIO_ReadOutputData(STEP_PORT) & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK));
  GPIO_Write(DIRECTION_PORT, (GPIO_ReadOutputData(DIRECTION_PORT) & ~DIRECTION_MASK) | (dir_port_invert_mask & DIRECTION_MASK));
#endif
}

// ��ʼ�����������������ϵͳ
void stepper_init()
{
  // ���ò����ͷ���ӿ�����
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


// ��ִ�п鱻�µļƻ�����ʱ����planner_recalculate�������á�
void st_update_plan_block_parameters()
{
  if (pl_block != NULL) { // ������¿�Ŀ�ʼ�����ԡ�
    prep.recalculate_flag |= PREP_FLAG_RECALCULATE;
    pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed; // ���������ٶ�
    pl_block = NULL; // ���st_prep_segment�����Լ��ز�����ٶ������ļ���
  }
}


// ���Ӳ��λ�������ݻ��λ�������
static uint8_t st_next_block_index(uint8_t block_index)
{
  block_index++;
  if ( block_index == (SEGMENT_BUFFER_SIZE-1) ) { return(0); }
  return(block_index);
}


#ifdef PARKING_ENABLE
  // �ı䲽�λ�����������״̬��ִ�������ͣ��������
  void st_parking_setup_buffer()
  {
    // ��Ҫʱ���洢������ɵĿ�Ĳ���ִ�����ݡ�
    if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK) {
      prep.last_st_block_index = prep.st_block_index;
      prep.last_steps_remaining = prep.steps_remaining;
      prep.last_dt_remainder = prep.dt_remainder;
      prep.last_step_per_mm = prep.step_per_mm;
    }
    // ���ñ�־��ִ��ͣ������
    prep.recalculate_flag |= PREP_FLAG_PARKING;
    prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE);
    pl_block = NULL; // ��������ͣ���������¼����µ����顣
  }


  // ͣ���󣬽������������ָ�����������״̬��
  void st_parking_restore_buffer()
  {
    // ������Ҫ�ָ�������ɿ�Ĳ���ִ�����ݺͱ�־��
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
    pl_block = NULL; // ����Ϊ���¼�����һ���顣
  }
#endif


/* ׼������λ����������������������á�

   �λ�������ִ�в���֮����м仺�����ӿ�
   �ɲ����㷨�͹滮�����ɵ��ٶ����ߡ�����
   �㷨ִֻ�жλ������ڵĲ��裬�����������
   ���Ӽƻ��������еĵ�һ���顰�������Щ����ʱ���Ᵽ����
   һ��ִ�кͼƻ��Ż�����ԭ�Ӻ��໥������
   �Ӽƻ��߻������С�������Ĳ����Լ��еķֶ���
   �λ������Ĵ�С�ͼ��㷽ʽʹ���������в���Ҫ�����κβ���
   ���������֮ǰ�������㷨�������ʱ��Ҫ����
   Ŀǰ���ֶλ��������صر��ִ���40-50����Ĳ�����
   ע�����㵥λ�ǲ��������׺ͷ��ӡ�
*/
void st_prep_buffer()
{
  // ��ֹ����Ԥ����������������ͣ״̬������û��ִ����ͣ������
  if (bit_istrue(sys.step_control,STEP_CONTROL_END_MOTION)) { return; }

  while (segment_buffer_tail != segment_next_head) { // ����Ƿ���Ҫ��仺������

    // ȷ���Ƿ���Ҫ�����µĹ滮��������Ƿ���Ҫ���¼���顣
    if (pl_block == NULL) {

      // �ŶӵĿ�Ĳ�ѯ�ƻ���
      if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) { pl_block = plan_get_system_motion_block(); }
      else { pl_block = plan_get_current_block(); }
      if (pl_block == NULL) { return; } // û�мƻ��߿顣���ڡ�

      // ����Ƿ�ֻ��Ҫ���¼����ٶ����߻����һ���µĳ���Ρ�
      if (prep.recalculate_flag & PREP_FLAG_RECALCULATE) {

        #ifdef PARKING_ENABLE
          if (prep.recalculate_flag & PREP_FLAG_PARKING) { prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE); }
          else { prep.recalculate_flag = false; }
        #else
          prep.recalculate_flag = false;
        #endif

      } else {

        // ���ؿ��Bresenham�������ݡ�
        prep.st_block_index = st_next_block_index(prep.st_block_index);

        // ���µļƻ������׼��������Bresenham�㷨�ֶ����ݣ�����
        // ���λ�������ɹ滮���ʱ�򣬿��ܻᱻ����
        // �λ��������׼���飬��������ISR����ִ������
        st_prep_block = &st_block_buffer[prep.st_block_index];
        st_prep_block->direction_bits = pl_block->direction_bits;//����λ
        uint8_t idx;
        #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
          for (idx=0; idx<N_AXIS; idx++) { st_prep_block->steps[idx] = (pl_block->steps[idx] << 1); }
          st_prep_block->step_event_count = (pl_block->step_event_count << 1);
        #else
          //����AMASS�󣬼򵥵�λ�ƽ�����Bresenham���ݳ������AMASS
          //�����������ǾͲ������㷨�е��κεط�����ԭʼ���ݡ�
          //���ԭʼ���ݱ��ָ���ǿ��Դ����������ж�ʧһ����
          for (idx=0; idx<N_AXIS; idx++) { st_prep_block->steps[idx] = pl_block->steps[idx] << MAX_AMASS_LEVEL; }
          st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
        #endif

        // ��ʼ���λ��������������ɶΡ�
        prep.steps_remaining = (float)pl_block->step_event_count;
        prep.step_per_mm = prep.steps_remaining/pl_block->millimeters;
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm;
        prep.dt_remainder = 0.0f; // ����Ϊ�µķֿ�

        if ((sys.step_control & STEP_CONTROL_EXECUTE_HOLD) || (prep.recalculate_flag & PREP_FLAG_DECEL_OVERRIDE)) {
          // �µĿ������Ѯ���С���д�ƻ������������ٶ���ǿ�Ƽ��١�
          prep.current_speed = prep.exit_speed;
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed;
          prep.recalculate_flag &= ~(PREP_FLAG_DECEL_OVERRIDE);
        } else {
          prep.current_speed = sqrtf(pl_block->entry_speed_sqr);
        }
#ifdef VARIABLE_SPINDLE
        // ���ü���ģʽ������PWM���ʵ�����Ķ������ǻ����Ŷ������
        // ����ر�
        st_prep_block->is_pwm_rate_adjusted = false;
        if (settings.flags & BITFLAG_LASER_MODE) {
          if (pl_block->condition & PL_COND_FLAG_SPINDLE_CCW) {
            // Ԥ�ȼ��㷴��������Լӿ�ÿ���ε�PWM���¡�
            prep.inv_rate = 1.0f / pl_block->programmed_rate;
            st_prep_block->is_pwm_rate_adjusted = true;
          }
        }
#endif
      }

			/* ---------------------------------------------------------------------------------
			 ���ݽ�����˳������¼ƻ��������ٶ�����
			 �ٶȣ������¼���һ��������ɵĹ滮ʦ��ĸ������ϣ����
			 �߻����Ѿ���������������ָ��ǿ�Ƽ��٣�����������
			 ���֣���д�滮�����ٶȲ����ٵ�Ŀ������ٶȡ�
			*/
			prep.mm_complete = 0.0f; // �ھ�������ĩβ0.0mm�����Ĭ�ϵ��ٶ����ߡ�
			float inv_2_accel = 0.5f/pl_block->acceleration;
			if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { // [ǿ�Ƽ��ٵ����ٶ�]
				// ����������ֽ����е��ٶ����߲������������ļ�����
				// �ƻ�����������ļ���ǿ�Ƽ��ٵ����ٶȡ�
				prep.ramp_type = RAMP_DECEL;
				// ��������ڳ���ν����ļ��پ��롣
				float decel_dist = pl_block->millimeters - inv_2_accel*pl_block->entry_speed_sqr;
				if (decel_dist < 0.0f) {
					// ͨ�������ƻ��������١���ֽ���ֽ������ڸÿ��С�
					prep.exit_speed = sqrtf(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->millimeters);
				} else {
					prep.mm_complete = decel_dist; // ����������
					prep.exit_speed = 0.0f;
				}
			} else { // [��������]
				// ��������¼���Ԥ���ƻ��������ٶ����߲�����
				prep.ramp_type = RAMP_ACCEL; // ��ʼ��Ϊ����б��
				prep.accelerate_until = pl_block->millimeters;

				float exit_speed_sqr;
				float nominal_speed;
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          prep.exit_speed = exit_speed_sqr = 0.0f; // ��ϵͳ�˶�����ʱǿ��ֹͣ��
        } else {
          exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
          prep.exit_speed = sqrtf(exit_speed_sqr);
        }

        nominal_speed = plan_compute_profile_nominal_speed(pl_block);
				float nominal_speed_sqr = nominal_speed*nominal_speed;
				float intersect_distance =
								0.5f*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));

        if (pl_block->entry_speed_sqr > nominal_speed_sqr) { // ֻ�ڳ��ۼ����ڼ䷢����
          prep.accelerate_until = pl_block->millimeters - inv_2_accel*(pl_block->entry_speed_sqr-nominal_speed_sqr);
          if (prep.accelerate_until <= 0.0f) { // ���ٶ���
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;

            // ���㸲�ǿ���˳��ٶȣ���Ϊ�������ϼƻ������˳��ٶȡ�
            prep.exit_speed = sqrtf(pl_block->entry_speed_sqr - 2*pl_block->acceleration*pl_block->millimeters);
            prep.recalculate_flag |= PREP_FLAG_DECEL_OVERRIDE; // ���Ϊ���ٸ��Ǽ�����һ������Ρ�

            // TODO: ȷ�����ڼ���ʱ��ȷ���������
            // ���ܻ�Ƚϼ��֣���Ϊ�����ٶȽ����ǵ�ǰ���ٶȣ������ڽ�������ʱһ����
            // ���⣬��������ӽ�����ٶȴ������⡣

          } else {
            // ����Ѳ����Ѳ���������͡���֤�ཻ���µļƻ���
            prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr); // ���ڹ滮������������Ӧʼ��> = 0.0��
						prep.maximum_speed = nominal_speed;
            prep.ramp_type = RAMP_DECEL_OVERRIDE;
          }
				} else if (intersect_distance > 0.0f) {
					if (intersect_distance < pl_block->millimeters) { // ���λ�����������
						// ע�⣺���ڼ���Ѳ����Ѳ�����ͣ����¼��㽫Ϊ0.0��
						prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr);
						if (prep.decelerate_after < intersect_distance) { // ������
							prep.maximum_speed = nominal_speed;
							if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
								// Ѳ�����ٻ�Ѳ���͡�
								prep.ramp_type = RAMP_CRUISE;
							} else {
								// ȫ���λ����Ѳ������
								prep.accelerate_until -= inv_2_accel*(nominal_speed_sqr-pl_block->entry_speed_sqr);
							}
						} else { // ������
							prep.accelerate_until = intersect_distance;
							prep.decelerate_after = intersect_distance;
							prep.maximum_speed = sqrtf(2.0f*pl_block->acceleration*intersect_distance+exit_speed_sqr);
						}
					} else { // ֻ�м�����
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;
					}
				} else { // ����������
					prep.accelerate_until = 0.0f;
					// prep.decelerate_after = 0.0f;
					prep.maximum_speed = prep.exit_speed;
				}
			}
      
      #ifdef VARIABLE_SPINDLE
        bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM); // ���¿�ʱǿ�Ƹ���.
      #endif
    }
    
    // ��ʼ���µ�ϸ��
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    // �����µĶ�ָ��ǰ�Ķ����ݿ�.
    prep_segment->st_block_index = prep.st_block_index;

    /*------------------------------------------------------------------------------------
        ͨ��ȷ���ܾ�������������¶ε�ƽ���ٶ�
      ������ʱ��DT_SEGMENT������Ĵ������ȳ��Դ���
      ���ڵ�ǰб�������������ֶΡ������ʱ�䲻����
      ����ֹ��һ��б��״̬�ı䣬���뽫����ѭ��ͨ��
      ����б��״̬�����ʣ��Ķ�ִ��ʱ�䡣���ǣ����
      һ���������Ķ����ٶ����ߵ�ĩ����ֹ���ö���
      ���ܽضϵ�ִ��ʱ������DT_SEGMENT�����Ա���Ϊ����ɡ�
        ���Ǽٶ��ٶ�����ͨ��б�����У�
      �����µ���Ѳ��״̬�ͼ����µ���ÿ���µ�����ʻ����
      ���ܷ�Χ���㵽��ĳ��ȡ��ٶ������ļ�������
      ��ǿ�Ƽ��ٽ���ʱ�滮���飨���ͣ��������м����飬
      ���������ϱ��֡�
    */
    float dt_max = DT_SEGMENT; // ����ʱ��
    float dt = 0.0f; // ��ʼ����ʱ��
    float time_var = dt_max; // ʱ�乤�˱���
    float mm_var; // ���׾��빤������
    float speed_var; // �ٶȹ��˱���
    float mm_remaining = pl_block->millimeters; // �ӿ�������¶ξ��롣
    float minimum_mm = mm_remaining-prep.req_mm_increment; // ��֤����һ������.
    if (minimum_mm < 0.0f) { minimum_mm = 0.0f; }

    do {
      switch (prep.ramp_type) {
        case RAMP_DECEL_OVERRIDE:
          speed_var = pl_block->acceleration*time_var;
					if (prep.current_speed-prep.maximum_speed <= speed_var) {
            // Ѳ����Ѳ���������ͽ����ڼ��ٱ���.
						mm_remaining = prep.accelerate_until;
            time_var = 2.0f*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            prep.ramp_type = RAMP_CRUISE;
            prep.current_speed = prep.maximum_speed;
          } else { // �еȼ��ٱ���б�¡�
						mm_remaining -= time_var*(prep.current_speed - 0.5f*speed_var);
            prep.current_speed -= speed_var;
          }
          break;
        case RAMP_ACCEL:
          // ע�⣺����б��ֻ�ڵ�һ��do-whileѭ���ڼ���㡣
          speed_var = pl_block->acceleration*time_var;
          mm_remaining -= time_var*(prep.current_speed + 0.5f*speed_var);
          if (mm_remaining < prep.accelerate_until) { // ����б�½�����
            // ����Ѳ�������� - �����ѵ����ӣ����Ľ�����
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            time_var = 2.0f*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { // �����١�
            prep.current_speed += speed_var;
          }
          break;
        case RAMP_CRUISE:
          // ע�⣺mm_var���ڱ�����������time_var��������һ��mm_remaining��
          // ע�⣺���maximum_speed * time_varֵ̫�ͣ�������ܵ���mm_var���ܸı䡣��
          // ��ֹ���������ֻ���ڹ滮����ǿ��ִ������ٶ���ֵ��
          mm_var = mm_remaining - prep.maximum_speed*time_var;
          if (mm_var < prep.decelerate_after) { // Ѳ��������
            // Ѳ���������ӵ�����ν���
            time_var = (mm_remaining - prep.decelerate_after)/prep.maximum_speed;
            mm_remaining = prep.decelerate_after; // ע�⣺��EOBΪ0.0
            prep.ramp_type = RAMP_DECEL;
          } else { // ֻѲ����
            mm_remaining = mm_var;
          }
          break;
        default: // case RAMP_DECEL:
          // ע�⣺mm_var����misc���������Է�ֹ�ӽ�����ʱ�Ĵ���
          speed_var = pl_block->acceleration*time_var; // ���������ٶȣ�����/���ӣ�
          if (prep.current_speed > speed_var) { // ������ٻ�������١�
            // ����Ӷ�β����β�ľ��롣
            mm_var = mm_remaining - time_var*(prep.current_speed - 0.5f*speed_var); // (mm)
            if (mm_var > prep.mm_complete) { // ����������ڼ���б�¡�
              mm_remaining = mm_var;
              prep.current_speed -= speed_var;
              break; // ����ɡ��˳�switch-case��䡣����ִ��ѭ����
            }
          }
          // �����ڳ���ν�����ǿ�Ƽ��ٽ���ʱ��
          time_var = 2.0f*(mm_remaining-prep.mm_complete)/(prep.current_speed+prep.exit_speed);
          mm_remaining = prep.mm_complete;
          prep.current_speed = prep.exit_speed;
      }
      dt += time_var; // �������б��ʱ����ӵ��ܶ�ʱ�䡣
      if (dt < dt_max) { time_var = dt_max - dt; } // **������**���ѵ����Ӵ���
      else {
        if (mm_remaining > minimum_mm) { // ����㼶�ǳ����ĶΡ�
          // ����ϸ��ʱ����ȷ��ϸ����������һ�����衣���Ǻ�ѭ��
          // ͨ���������ֱ��minimum_mm����mm_complete��
          dt_max += DT_SEGMENT;
          time_var = dt_max - dt;
        } else {
          break; // **���**�˳�ѭ�����ֶ�ִ��ʱ�����
        }
      }
    } while (mm_remaining > prep.mm_complete); // **���**�˳�ѭ��������������

    #ifdef VARIABLE_SPINDLE
      /* -----------------------------------------------------------------------------------
        	���㲽���ε�����ת��PWM���
      */

      if (st_prep_block->is_pwm_rate_adjusted || (sys.step_control & STEP_CONTROL_UPDATE_SPINDLE_PWM)) {
        if (pl_block->condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)) {
          float rpm = pl_block->spindle_speed;
          // ע�⣺�����Ϳ��ٸ�����PWMֵ�޹أ�����ı伤�⹦��/���ʡ�
          if (st_prep_block->is_pwm_rate_adjusted) { rpm *= (prep.current_speed * prep.inv_rate); }
          // ���current_speedΪ�㣬�������Ҫrpm_min *��100 / MAX_SPINDLE_SPEED_OVERRIDE��
          // ����ֻ��˲��Ķ����������޹ؽ�Ҫ��
          prep.current_spindle_pwm = spindle_compute_pwm_value(rpm);
        }
        else {
          sys.spindle_speed = 0.0;
          prep.current_spindle_pwm = SPINDLE_PWM_OFF_VALUE;
        }
        bit_false(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
      }
      prep_segment->spindle_pwm = prep.current_spindle_pwm; // ���¼��ضε�PWMֵ

    #endif
    
    /* -----------------------------------------------------------------------------------
		   ����ֶβ������ʣ�ִ�в����Լ�Ӧ�ñ�Ҫ�����ʸ�����
		   ע��������ͨ�����׾����ֱ�ӱ���ת���������
		   �����ڿ��У������ǵ����ؼ���ÿ��ִ�еĲ���
		   �ָ�������������������ӵĸ��������������⡣
		   Ȼ�������ڻ���ֻ��7.2λ��Ч���֣���ʱ����ƶ��ǳ�
		   �߲������ܻᳬ�����ӵľ��ȣ�����ܻᵼ�²��趪ʧ��
		   ���˵��ǣ����������CNC�������ǲ�̫���ܺͲ���ʵ�ʵ�
		   ��Grbl֧�֣�����200��/ mm����10�׵������г̣���
    */
    float step_dist_remaining = prep.step_per_mm*mm_remaining; // ��mm_remainingת��Ϊ����
    float n_steps_remaining = ceilf(step_dist_remaining); // ʣ��ĵ�ǰ����
    float last_n_steps_remaining = ceilf(prep.steps_remaining); // �ܽ����Ĳ���
	prep_segment->n_step = (uint16_t)(last_n_steps_remaining - n_steps_remaining); // ����Ҫִ�еĲ�������

    // ���������һ��feed�Ľ�β������û��һ��������ִ�У���ô���͡�
    if (prep_segment->n_step == 0) {
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) {
        // ����һ��������ٵ����٣����Ѿ��ǳ��ӽ���AMASS
        // ��Ҫִ�������Ĳ��衣���ԣ�ֻҪ���͡�
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // ��û�����ɣ�����ǰ�Ĳ���������Ȼ������
      }
    }

    //����εĲ������ʡ����ڲ����������������г̵ľ��벻�ǣ�
    //ÿһ�εĽ�β��������һ����ͬ��ֵ�Ĳ��ֲ���
    //ִ�У���Ϊ������ISR����AMASS�㷨��Ҫ�������衣��
    //���������Ǹ���ִ��ǰһ���ֶεĲ��ֲ����ʱ��
    //�Ե�ǰ�εĲ��ֲ���Ӧ�������������Ϳ�����
    //���������ֶ������Ա��ֲ��������׼ȷ�ԡ���Щ���ʵ�����
    //ͨ���ǳ�С����������ܲ�������Ӱ�죬��ȷ��Grbl
    //����ƻ��߼����ȷ�еļ��ٶȺ��ٶ����ߡ�
    dt += prep.dt_remainder; // Ӧ����һ���εĲ��ֲ���ִ��ʱ��
    float inv_rate = dt/(last_n_steps_remaining - step_dist_remaining); // ��������Ĳ������ʵ���

    // ����׼���õĶε�ÿ�������CPU���ڡ�
	uint32_t cycles = (uint32_t)ceilf((TICKS_PER_MICROSECOND * 1000000) *inv_rate * 60); // (cycles/step)

    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      // ���㲽��ʱ��Ͷ���ƽ��ˮƽ��
      // ע�⣺AMASSÿ�����𶼻ᳬʱ������ֻ��Ҫһ��Ԥ��Ƶ����
      if (cycles < AMASS_LEVEL1) { prep_segment->amass_level = 0; }
      else {
        if (cycles < AMASS_LEVEL2) { prep_segment->amass_level = 1; }
        else if (cycles < AMASS_LEVEL3) { prep_segment->amass_level = 2; }
        else { prep_segment->amass_level = 3; }
        cycles >>= prep_segment->amass_level;
        prep_segment->n_step <<= prep_segment->amass_level;
      }
      if (cycles < (1UL << 16)) { prep_segment->cycles_per_tick = cycles; } // < 65536 (4.1ms @ 16MHz)
      else { prep_segment->cycles_per_tick = 0xffff; } // ֻ�������������ٶȡ�
    #else
      // ���������������ɵĲ���ʱ��Ͷ�ʱ��Ԥ��Ƶ����
      if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
        prep_segment->prescaler = 1; // Ԥ��Ƶ��: 0
        prep_segment->cycles_per_tick = cycles;
      } else if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        prep_segment->prescaler = 2; // Ԥ��Ƶ��: 8
        prep_segment->cycles_per_tick = cycles >> 3;
      } else {
        prep_segment->prescaler = 3; // Ԥ��Ƶ��: 64
        if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
          prep_segment->cycles_per_tick =  cycles >> 6;
        } else { // ֻ�������������ٶȡ���Լ4��/�룩
          prep_segment->cycles_per_tick = 0xffff;
        }
      }
    #endif

    // ����ɣ������λ��������������Բ���ISR��������ִ������
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }

    // �����ʵ��ļƻ��ߺ�ϸ�����ݡ�
    pl_block->millimeters = mm_remaining;
    prep.steps_remaining = n_steps_remaining;
    prep.dt_remainder = (n_steps_remaining - step_dist_remaining)*inv_rate;

    // ����˳������ͱ�־������һ���ƻ�ģ�顣
    if (mm_remaining == prep.mm_complete) {
      // �ƻ�����������ǿ����ֹ��û�и���ľ��뱻ִ�С�
      if (mm_remaining > 0.0f) { // ��ǿ����ֹ������
          // ����׼�������ָ���Ȼ���͡�������ISR���
          //�ζ��У�ʵʱЭ�齫�ڽ��յ�������������µ�״̬
          //��ISRѭ��ֹͣ��־���ڴ�֮ǰ��Prep_segment����ֹ��
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // Bail!
      } else { // �ƻ��������
        // �滮ʦ���������ġ����в��趼����Ϊ�ڶλ�������ִ��.
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
          return;
        }
        pl_block = NULL; // ����ָ��ָʾ��鲢������һ���ƻ�����顣
        plan_discard_current_block();
      }
    }

  }
}


//ͨ��ʵʱ״̬�����������ȡ����ִ�еĵ�ǰ�ٶȡ����ֵ
//Ȼ���������ڵ��ٶȣ����������һ�������м�����ٶ�
//�ڶλ������С�������������ڿ��������-1��
// ����ÿ����ٶ�����.
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
