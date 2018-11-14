/*
  gcode.c - rs274 / ngc��������
  G���������
  ���� rs274/ngc ��׼���н���
*/

#include "grbl.h"

//ע�����������g�����׼����Ϊ99999.���ƺ���һ��
//����ֵ����ЩGUI������Ҫ���ࡣ�������������˻������ȫ
//��float��7.2���־��ȣ�ת��Ϊ����ʱ��ֵ��
#define MAX_LINE_NUMBER 10000000
#define MAX_TOOL_NUMBER 255 // ������޷���8λֵ������. Limited by max unsigned 8-bit value

#define AXIS_COMMAND_NONE 0
#define AXIS_COMMAND_NON_MODAL 1
#define AXIS_COMMAND_MOTION_MODE 2
#define AXIS_COMMAND_TOOL_LENGTH_OFFSET 3 //*����������Ҫ    *Undefined but required

// ����gc extern�ṹ��
parser_state_t gc_state;
parser_block_t gc_block;

#define FAIL(status) return(status);


void gc_init()
{
  memset(&gc_state, 0, sizeof(parser_state_t));

  // ����Ĭ�ϵ�G54����ϵ
  if (!(settings_read_coord_data(gc_state.modal.coord_select,gc_state.coord_system))) {
    report_status_message(STATUS_SETTING_READ_FAIL);
  }
}


//�� gΪ��λ����g�����������λ�á����벽�衣��ϵͳ������ֹ��Ŭ��
//�����������̡�
void gc_sync_position()
{
  system_convert_array_steps_to_mpos(gc_state.position,sys_position);
}


//ִ��һ����0��β��G���롣���мٶ�ֻ������д��ĸ
//�ַ��ʹ����ŵĸ���ֵ���޿ո񣩡����ۺͿ�ɾ��
//�ַ��ѱ�ɾ��������������У����е�λ��ְλ����ת��
//�ԣ�mm��mm / min���;��Ի�������ʽ�����grbl���ڲ�����
//���꣬�ֱ�
uint8_t gc_execute_line(char *line)
{
  /* -------------------------------------------------------------------------------------
     ����1����ʼ����������ṹ�����Ƶ�ǰ��g����״̬ģʽ��������
     ������Щģʽ�������Ϊ�����ǽ�������ֻ�ᱻʹ��
     �ɹ��Ĵ������ִ�С���������ṹҲ����һ����
     ֵ�ṹ���ָ��ٱ�����һ����ģ̬�������������
     �顣����ṹ�������ִ������������ȫ����Ϣ�� */
  
  memset(&gc_block, 0, sizeof(parser_block_t)); //��ʼ����������ṹ�� Initialize the parser block struct.
  memcpy(&gc_block.modal,&gc_state.modal,sizeof(gc_modal_t)); //���Ƶ�ǰģʽ Copy current modes

  uint8_t axis_command = AXIS_COMMAND_NONE;
  uint8_t axis_0, axis_1, axis_linear;
  uint8_t coord_select = 0; //����G10 P����ѡ��ִ�� Tracks G10 P coordinate selection for execution

  //Ϊ���������ݲ�����ʼ��λ��־���ٱ����� Initialize bitflag tracking variables for axis indices compatible operations.
  uint8_t axis_words = 0; //XYZ����  ��־�ĸ�����Ҫ����
  uint8_t ijk_words = 0; //IJK���� IJK tracking

  //��ʼ�������ֵ�ֺͷ�������־������ Initialize command and value words and parser flags variables.
  uint16_t command_words = 0; //����G��M�������֡�Ҳ����ģ̬��Υ�档 Tracks G and M command words. Also used for modal group violations.
  uint16_t value_words = 0; //���ټ�ֵ�ʡ� Tracks value words.
  uint8_t gc_parser_flags = GC_PARSER_NONE;

  //ȷ�������������˶�����������g����顣 Determine if the line is a jogging motion or a normal g-code block.
  if (line[0] == '$') { // ע�⣺�����ݸ��������ʱ���Ѿ�������`$ J =`��NOTE: `$J=` already parsed when passed to this function.
						//����G1��G94ǿ��ģʽ��ȷ��׼ȷ�Ĵ����顣 Set G1 and G94 enforced modes to ensure accurate error checks.
    gc_parser_flags |= GC_PARSER_JOG_MOTION;
	gc_block.modal.motion = MOTION_MODE_LINEAR;
    gc_block.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
#ifdef USE_LINE_NUMBERS
     gc_block.values.n = JOG_LINE_NUMBER; // Initialize default line number reported during jog.
#endif
  }

  /* -------------------------------------------------------------------------------------
     ����2���ڿ����е������е�g�����֡�һ��g������һ����ĸ���
     һ�����֣��ȿ����ǡ�G��/��M�����Ҳ��������/��������ֵ��Ҳ��
     ���κ��ظ���������ģ̬��Υ��ִ�г�ʼ������
     ����ֵ��F��N��P��T��S�趨��ֵ�� */

  uint8_t word_bit; //���ڷ�����ٱ�����λֵ Bit-value for assigning tracking variables
  uint8_t char_counter;
  char letter;
  float value;
  uint8_t int_value = 0;
  uint16_t mantissa = 0;//Gxx.x�����β��
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) { char_counter = 3; } //��`$ J =`֮��ʼ����  Start parsing after `$J=`
  else { char_counter = 0; }

  while (line[char_counter] != 0) { //ѭ��ֱ��û�и����g�����ַ��ϡ� Loop until no more g-code words in line.

    //������һ��g�����֣�����һ����ĸ���һ��ֵ�����򣬳��� Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
    letter = line[char_counter];
    if((letter < 'A') || (letter > 'Z')) { FAIL(STATUS_EXPECTED_COMMAND_LETTER); } //[Ԥ����ĸ] [Expected word letter]
    char_counter++;
    if (!read_float(line, &char_counter, &value)) { FAIL(STATUS_BAD_NUMBER_FORMAT); } //[�����ĵ���ֵ] [Expected word value]

    //��ֵת��Ϊ��С��uint8��Ч����β��ֵ������������ʡ�
    //ע�⣺β������100�������������ֵ�����Ǹ���
    //�����������x10��NIST gcodeҪ��׼ȷ��������ȫ
    //������Ҫ������0.0001���ڵļ�ֵ�����㹻׼ȷ����Ӧ����
    //�㹻�õ�comprimise�Ͳ�����������������Ϊ��ʹ֮�Ϲ棬
    //����ֻ��Ҫ��β����Ϊint16������������ӱ��������ռ䡣
    //�Ժ���ܻ���¡�
    int_value = truncf(value);
	mantissa = (uint16_t)lroundf(100 * (value - int_value)); //����Gxx.x�����β���� Compute mantissa for Gxx.x commands.
    //ע�⣺���������������С�ĸ������ NOTE: Rounding must be used to catch small floating point errors.

    //���g�������Ƿ�֧�֣���������ģ̬��Υ����ߴ����´���
    //��g��������ظ���������ԣ�����������¼����ֵ��
    switch(letter) {

      /* 'G'��'M'�����֣�����������ģ̬��Υ�档
         ע��ģ̬������NIST RS274-NGC��3���4�ж��壬��20ҳ */

      case 'G':
        // ȷ��'G'�����ģ̬��
        switch(int_value) {
          case 10: case 28: case 30: case 92:
            //�����ͬһ�����������G0 / 1/2/3/38����G10 / 28/30/92�� Check for G10/28/30/92 being called with G0/1/2/3/38 on same block.
            // * G43.1Ҳ��һ���������û����ȷ�Ķ��塣 * G43.1 is also an axis command but is not explicitly defined this way.
            if (mantissa == 0) { // Ignore G28.1, G30.1, and G92.1
              if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } //[���/�����ͻ] [Axis word/command conflict]
              axis_command = AXIS_COMMAND_NON_MODAL;
            }
            //û����Ϣ ������һ�С� No break. Continues to next line.
          case 4: case 53:
            word_bit = MODAL_GROUP_G0;
            gc_block.non_modal_command = int_value;
            if ((int_value == 28) || (int_value == 30) || (int_value == 92)) {
              if (!((mantissa == 0) || (mantissa == 10))) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); }
              gc_block.non_modal_command += mantissa;
              mantissa = 0; //����Ϊ���ʾ��Ч�ķ�����G��� Set to zero to indicate valid non-integer G command.
            }                
            break;
          case 0: case 1: case 2: case 3: case 38:
            //�����ͬһ�����������G10 / 28/30/92���õ�G0 / 1/2/3/38�� Check for G0/1/2/3/38 being called with G10/28/30/92 on same block.
            //* G43.1Ҳ��һ���������û����ȷ�Ķ��塣 * G43.1 is also an axis command but is not explicitly defined this way.
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } //[���/�����ͻ] [Axis word/command conflict]
            axis_command = AXIS_COMMAND_MOTION_MODE;
            //û����Ϣ ������һ�С� No break. Continues to next line.
          case 80:
            word_bit = MODAL_GROUP_G1;
            gc_block.modal.motion = int_value;
            if (int_value == 38){
              if (!((mantissa == 20) || (mantissa == 30) || (mantissa == 40) || (mantissa == 50))) {
                FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); //[��֧�ֵ�G38.x����] [Unsupported G38.x command]
              }
              gc_block.modal.motion += (mantissa/10)+100;
              mantissa = 0; //����Ϊ���ʾ��Ч�ķ�����G��� Set to zero to indicate valid non-integer G command.
            }  
            break;
          case 17: case 18: case 19:
            word_bit = MODAL_GROUP_G2;
            gc_block.modal.plane_select = int_value - 17;
            break;
          case 90: case 91:
            if (mantissa == 0) {
              word_bit = MODAL_GROUP_G3;
              gc_block.modal.distance = int_value - 90;
            } else {
              word_bit = MODAL_GROUP_G4;
              if ((mantissa != 10) || (int_value == 90)) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } //[��֧��G90.1] [G90.1 not supported]
              mantissa = 0; //����Ϊ���ʾ��Ч�ķ�����G��� Set to zero to indicate valid non-integer G command.
              //���򣬻�IJK����ģʽ��Ĭ�ϵġ�G91.1ʲô�������� Otherwise, arc IJK incremental mode is default. G91.1 does nothing.
            }
            break;
          case 93: case 94:
            word_bit = MODAL_GROUP_G5;
            gc_block.modal.feed_rate = 94 - int_value;
            break;
          case 20: case 21:
            word_bit = MODAL_GROUP_G6;
            gc_block.modal.units = 21 - int_value;
            break;
          case 40:
            word_bit = MODAL_GROUP_G7;
            //ע�⣺���ڵ��߰뾶����ʼ�ձ����ã���˲���Ҫ��ֻ�������� NOTE: Not required since cutter radius compensation is always disabled. Only here
            //֧�־���������g�������ͷ�ļ��е�G40����������Ĭ��ֵ�� to support G40 commands that often appear in g-code program headers to setup defaults.
            // gc_block.modal.cutter_comp = CUTTER_COMP_DISABLE; // G40
            break;
          case 43: case 49:
            word_bit = MODAL_GROUP_G8;
            //ע�⣺NIST��g�����׼��Լָ���������߳��Ȳ����ı�ʱ��
            //�������κ����˶�������ƫ�Ƹ��¡�����G43��G43.1��G49
            //���еĶ�����ȷ����������������Ƿ���Ҫ������
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } //[�ᵥ��/�����ͻ]} [Axis word/command conflict] }
            axis_command = AXIS_COMMAND_TOOL_LENGTH_OFFSET;
            if (int_value == 49) { // G49
              gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_CANCEL;
            } else if (mantissa == 10) { // G43.1
              gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC;
            } else { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } //[��֧�ֵ�G43.x����] [Unsupported G43.x command]
            mantissa = 0; //��Ϊ���ʾ��Ч�ķ�����G��� Set to zero to indicate valid non-integer G command.
            break;
          case 54: case 55: case 56: case 57: case 58: case 59:
            ///ע�⣺��֧��G59.x���������ǵ�int_values����60,61��62.�� NOTE: G59.x are not supported. (But their int_values would be 60, 61, and 62.)
            word_bit = MODAL_GROUP_G12;
            gc_block.modal.coord_select = int_value - 54; //�л������������� Shift to array indexing.
            break;
          case 61:
            word_bit = MODAL_GROUP_G13;
            if (mantissa != 0) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } //[��֧��G61.1] [G61.1 not supported]
            // gc_block.modal.control = CONTROL_MODE_EXACT_PATH; // G61
            break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); //[��֧�ֵ�Gָ��] [Unsupported G command]
        }
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } //[��֧�ֻ���Ч��Gxx.x����] [Unsupported or invalid Gxx.x command]
        //�ڵ�ǰ���м��ÿ��ģ̬��Υ��Ķ������ Check for more than one command per modal group violations in the current block
        //ע�⣺���������Ч������'word_bit'���Ǳ���ֵ�� NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;

      case 'M':

        // ȷ��'M'�����ģ̬��
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } //[û��Mxx.x����] [No Mxx.x commands]
        switch(int_value) {
          case 0: case 1: case 2: case 30:
            word_bit = MODAL_GROUP_M4;
            switch(int_value) {
              case 0: gc_block.modal.program_flow = PROGRAM_FLOW_PAUSED; break; //������ͣ  Program pause
              case 1: break; //��ѡ��ֹͣ��֧�֡����ӡ� Optional stop not supported. Ignore.
              default: gc_block.modal.program_flow = int_value; //�������������  Program end and reset
            }
            break;
					case 3: case 4: case 5:
            word_bit = MODAL_GROUP_M7;
            switch(int_value) {
              case 3: gc_block.modal.spindle = SPINDLE_ENABLE_CW; break;
              case 4: gc_block.modal.spindle = SPINDLE_ENABLE_CCW; break;
              case 5: gc_block.modal.spindle = SPINDLE_DISABLE; break;
            }
            break;
          #ifdef ENABLE_M7
            case 7: case 8: case 9:
          #else
            case 8: case 9:
          #endif
            word_bit = MODAL_GROUP_M8;
            switch(int_value) {
              #ifdef ENABLE_M7
                case 7: gc_block.modal.coolant = COOLANT_MIST_ENABLE; break;
              #endif
              case 8: gc_block.modal.coolant = COOLANT_FLOOD_ENABLE; break;
              case 9: gc_block.modal.coolant = COOLANT_DISABLE; break;
            }
            break;
					#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
						case 56:
							word_bit = MODAL_GROUP_M9;
							gc_block.modal.override = OVERRIDE_PARKING_MOTION;
							break;
					#endif
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [��֧�ֵ�M����] [Unsupported M command]
        }

        //�ڵ�ǰ���м��ÿ��ģ̬��Υ��Ķ������ Check for more than one command per modal group violations in the current block
        //ע�⣺���������Ч������'word_bit'���Ǳ���ֵ�� NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;

      //ע�⣺����ʣ�����ĸ����ֵ�� NOTE: All remaining letters assign values.
      default:

        /* *�������֣��˳�ʼ�����׶ν����ʣ����ظ�
           	   �Ϸ���g���֣����洢���ǵļ�ֵ���������һЩ�Ժ�ִ��
           	   �֣�I��J��K��L��P��R�����ж���ں���/��ȡ��������������� */
        switch(letter){
          // case 'A': // ��֧��
          // case 'B': // ��֧��
          // case 'C': // ��֧��
          // case 'D': // ��֧��
          case 'F': word_bit = WORD_F; gc_block.values.f = value; break;
          // case 'H': // ��֧��
          case 'I': word_bit = WORD_I; gc_block.values.ijk[X_AXIS] = value; ijk_words |= (1<<X_AXIS); break;
          case 'J': word_bit = WORD_J; gc_block.values.ijk[Y_AXIS] = value; ijk_words |= (1<<Y_AXIS); break;
          case 'K': word_bit = WORD_K; gc_block.values.ijk[Z_AXIS] = value; ijk_words |= (1<<Z_AXIS); break;
          case 'L': word_bit = WORD_L; gc_block.values.l = int_value; break;
          case 'N': word_bit = WORD_N; gc_block.values.n = truncf(value); break;
          case 'P': word_bit = WORD_P; gc_block.values.p = value; break;
          //ע�⣺����ĳЩ���Pֵ����������������֧����Щ��� NOTE: For certain commands, P value must be an integer, but none of these commands are supported.
          // case 'Q': // ��֧��
          case 'R': word_bit = WORD_R; gc_block.values.r = value; break;
          case 'S': word_bit = WORD_S; gc_block.values.s = value; break;
		  case 'T': word_bit = WORD_T;
				if (value > MAX_TOOL_NUMBER) { FAIL(STATUS_GCODE_MAX_VALUE_EXCEEDED); }
					gc_block.values.t = int_value;
				break;
		  case 'X': word_bit = WORD_X; gc_block.values.xyz[X_AXIS] = value; axis_words |= (1<<X_AXIS); break;
          case 'Y': word_bit = WORD_Y; gc_block.values.xyz[Y_AXIS] = value; axis_words |= (1<<Y_AXIS); break;
          case 'Z': word_bit = WORD_Z; gc_block.values.xyz[Z_AXIS] = value; axis_words |= (1<<Z_AXIS); break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
        }

        //ע�⣺�����������ĸ��Ч����ʼ�շ��������word_bit���� NOTE: Variable 'word_bit' is always assigned, if the non-command letter is valid.
        if (bit_istrue(value_words,bit(word_bit))) { FAIL(STATUS_GCODE_WORD_REPEATED); } //[�ظ�����] [Word repeated]
        //��鵥��F��N��P��T��S����Ч��ֵ Check for invalid negative values for words F, N, P, T, and S.
        //ע�⣺Ϊ�˴���Ч�ʣ���������ɸ�ֵ��顣 NOTE: Negative value check is done here simply for code-efficiency.
        if ( bit(word_bit) & (bit(WORD_F)|bit(WORD_N)|bit(WORD_P)|bit(WORD_T)|bit(WORD_S)) ) {
          if (value < 0.0) { FAIL(STATUS_NEGATIVE_VALUE); } //[�ֵ�ֵ�����Ǹ���] [Word value cannot be negative]
        }
        value_words |= bit(word_bit); //�����ָʾ����Ĳ����� Flag to indicate parameter assigned.

    }
  }
  // ������ɣ�


  /* -------------------------------------------------------------------------------------
     ��3�����������ڸÿ��д��ݵ����������ֵ����һ��ȷ�����е�
     ��Щ�������ִ������Ч�ģ����Ҿ����ܵ���ѭNIST��׼��
     ������ִ�����ÿ��е����������ֵ������ת�������Ҳ������
     ����ϵͳg����ģʽ������ÿ�û�����⣬��ô���ϵͳg����ģʽ������
     ����������������и��£������źŸ���ִ�С�
     ���⣬���Ǳ���Ԥ��ת�����л��ڽ������õ�ģʽ���ݵ�ֵ
     �顣������������ҪĿ����Ϣ��ֻ����
     ������ǽ���Щֵ�������һ��ת��������Ծ�ȷ���������
     �⽫��һ��ִ�в�����Ϊֻ����ϵͳg����ģʽ��
     ��˳��ִ�б�̵Ķ�����ִ�в��費Ӧ��Ҫ���κ�
     ת�����㣬����ֻ��Ҫִ�����ٵļ�顣
  */

  /*
     ע�⣺��ʱ��g������ѱ����������пɱ��ͷš�
     ע�⣺��δ��ĳ��ʱ��Ҳ�п��ֽܷ��2����������ֶ�
     ��ÿ�����ʵĻ����Ͻ����飬�����������顣����ܻ�ɾ��
     ��ҪΪ������ά��һ������ַ����������ͷ�һЩ�ڴ档
     Ϊ�ˣ�ֻ��Ҫ����STEP 1�е��������ݣ������¿�
     ���ݽṹ��ģ̬���ֵλ��Ǹ��ٱ����Լ�����������
     ���ݵı�������Щ���ݰ����˴����������������Ϣ
     ���յ�EOL�ַ�ʱ�µ�g����顣���ǣ��⽫����Grbl�Ĵ�ҵ
     ��ι�����������ҪһЩ�ع���ʹ����ݡ�
  */

  //[0�����ض�/�������������������]�� [0. Non-specific/common error-checks and miscellaneous setup]:

  //ȷ����ʽ��������������Ļ��Ѿ�ͨ������û����ȷ����   Determine implicit axis command conditions. Axis words have been passed, but no explicit axis
  //�����Ѿ����͡�����ǣ�������������Ϊ��ǰ�˶�ģʽ�� command has been sent. If so, set axis command to current motion mode.
  if (axis_words) {
    if (!axis_command) { axis_command = AXIS_COMMAND_MOTION_MODE; } //������ʽ�˶�ģʽ  Assign implicit motion-mode
  }

  //�����Ч���к�Nֵ��  Check for valid line number N value.
  if (bit_istrue(value_words,bit(WORD_N))) {
    //�к�ֵ���ܴ��������к�С���㣨��ɣ������ Line number value cannot be less than zero (done) or greater than max line number.
    if (gc_block.values.n > MAX_LINE_NUMBER) { FAIL(STATUS_GCODE_INVALID_LINE_NUMBER); } // [Exceeds max line number]
  }
  // bit_false(value_words,bit(WORD_N)); // NOTE: Single-meaning value word. Set at end of error-checking.

  //�ڴ��������ʱ����δʹ�õĵ��ʡ�
  //ע�⣺�ڴ��������ʱ����ֵ������ֱ�һ��ɾ������Ϊ
  //���������ڳ�ʱʹ�á���������Ϊ�˽�ʡ�����ֽڵ����档Ϊ����������
  //ʹ�õ�ֵ����ĵ��ʿ��ܻᱻɾ�������⣬������ڴ���
  //ͬ���ķ�ʽ���������ȷ��/�����������XYZ�����Ǳ�ʹ�ò�����
  //�ڴ��������ʱ��ɾ����

  //[1������]��ζ����֧�֡�Э��ִ�е�ע�ʹ��� [1. Comments ]: MSG's NOT SUPPORTED. Comment handling performed by protocol.

  //[2�����ý�������ģʽ]��G1��G2 / 3����ʱ��G93 F��ȱʧ����ʽ����ʽ�������� [2. Set feed rate mode ]: G93 F word missing with G1,G2/3 active, implicitly or explicitly. Feed rate
  //��G93�л���G94��û�ж��塣   is not defined after switching to G94 from G93.
  //ע�⣺�������ܣ�����֮ǰ�Ľ�������ģʽ��ǿ��ִ��G94����������F�֡� NOTE: For jogging, ignore prior feed rate mode. Enforce G94 and check for required F word.
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) {
    if (bit_isfalse(value_words,bit(WORD_F))) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); }
    if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.f *= MM_PER_INCH; }
  } else {
    if (gc_block.modal.feed_rate == FEED_RATE_MODE_INVERSE_TIME) { // = G93
      //ע�⣺G38Ҳ���Է�ʱ�����У�����û�ж���Ϊ�������������ȱ��F�ּ�顣 NOTE: G38 can also operate in inverse time, but is undefined as an error. Missing F word check added here.
      if (axis_command == AXIS_COMMAND_MOTION_MODE) {
		if ((gc_block.modal.motion != MOTION_MODE_NONE) && (gc_block.modal.motion != MOTION_MODE_SEEK)) {
          if (bit_isfalse(value_words,bit(WORD_F))) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); } // [F word missing]
        }
      }
      //ע�⣺����G94�л���G93��Ҫ���ݵ�F���ƺ��Ƕ���ġ����ǻ�
      //�����������ֵ��������Ϊ�㣬������ÿ��֮��δ���壬�����ͬ��������
      //��ʱ��飬��Ϊʹ�����ֵ�������Ѿ�ִ����δ����ļ�顣�����
      //Ҳ��������������������֮��ִ�У������ǲ���Ҫ�س������������
      //���������������ģʽ�������趨�Ľ����ٶȴ����顣

      // [3���趨��������]��FΪ��ֵ����ɣ�
      // - ��ʱ��ģʽ�������ڿ����֮ǰ��֮����ʽ�ؽ���������ֵ���㡣
      //ע�⣺�����G93ģʽ�»��ߴ�G94ģʽ�л���G99��ֻ�轫Fֵ����Ϊ��ʼ����򴫵�F��
      //�ڿ��е�ֵ�����û��F��ͨ����Ҫ�����ٶȵ��˶�����ݣ��⽫�����
      //���˶�ģʽ�½��д����顣���ǣ����û��F����ͨ��NO����������Ҫ��
      //�����ٶȣ����Ǽ򵥵ؼ�����״̬��������ֵ����Ϊ�㣬������δ���塣
    } else { // = G94
      // - �Ժ��׵�λΪ��λ�����F��ͨ����ȷ��ֵ�Ժ���/����Ϊ��λ���������һ��״ֵ̬��
      if (gc_state.modal.feed_rate == FEED_RATE_MODE_UNITS_PER_MIN) { //���״̬ҲG94 Last state is also G94
        if (bit_istrue(value_words,bit(WORD_F))) {
          if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.f *= MM_PER_INCH; }
        } else {
          gc_block.values.f = gc_state.feed_rate; //�ƶ�����״̬�������� Push last state feed rate
        }
      } //���򣬴�G93�л���G94�����Բ�Ҫ������״̬�������ʡ�����δ����򴫵ݵ�F��ֵ�� Else, switching to G94 from G93, so don't push last state feed rate. Its undefined or the passed F word value.
    }
  }
  // bit_false(value_words,bit(WORD_F)); // NOTE: Single-meaning value word. Set at end of error-checking.

  //[4����������ת��]��SΪ��ֵ����ɣ� [4. Set spindle speed ]: S is negative (done.)
  if (bit_isfalse(value_words,bit(WORD_S))) { gc_block.values.s = gc_state.spindle_speed; }
  // bit_false(value_words,bit(WORD_S)); // NOTE: Single-meaning value word. Set at end of error-checking.

  //[5��ѡ�񹤾�]����֧�֡�ֻ���ټ�ֵ��T�Ƿ񶨵ģ���ɣ�����һ��������������󹤾�ֵ�� [5. Select tool ]: NOT SUPPORTED. Only tracks value. T is negative (done.) Not an integer. Greater than max tool value.
  // bit_false(value_words,bit(WORD_T)); // NOTE: Single-meaning value word. Set at end of error-checking.

  // [6. ���� ]: N/A
  // [7. ������� ]: N/A
  // [8. ��ȴҺ���� ]: N/A
	// [9. ���ǿ��� ]: ��Grblר�õ�ͣ���˶����ǿ����⣬��֧�֡�
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
	if (bit_istrue(command_words, bit(MODAL_GROUP_M9))) { // Already set as enabled in parser.
		if (bit_istrue(value_words, bit(WORD_P))) {
			if (gc_block.values.p == 0.0f) { gc_block.modal.override = OVERRIDE_DISABLED; }
			bit_false(value_words, bit(WORD_P));
		}
	}
#endif
	
  // [10. ��ͣ ]: ȱ��Pֵ��P�Ƿ񶨵ģ���ɣ�ע�������ġ�
  if (gc_block.non_modal_command == NON_MODAL_DWELL) {
    if (bit_isfalse(value_words,bit(WORD_P))) { FAIL(STATUS_GCODE_VALUE_WORD_MISSING); } // [P�ֶ�ʧ]
    bit_false(value_words,bit(WORD_P));
  }

  // [11�����ûƽ��]��N / A
  switch (gc_block.modal.plane_select) {
    case PLANE_SELECT_XY:
      axis_0 = X_AXIS;
      axis_1 = Y_AXIS;
      axis_linear = Z_AXIS;
      break;
    case PLANE_SELECT_ZX:
      axis_0 = Z_AXIS;
      axis_1 = X_AXIS;
      axis_linear = Y_AXIS;
      break;
    default: // case PLANE_SELECT_YZ:
      axis_0 = Y_AXIS;
      axis_1 = Z_AXIS;
      axis_linear = X_AXIS;
  }

  // [12. ���ó��ȵ�λ ]: N/A
  // ��XYZ����ֵԤ��ת��Ϊ���ף�������ã���
  uint8_t idx;
  if (gc_block.modal.units == UNITS_MODE_INCHES) {
    for (idx=0; idx<N_AXIS; idx++) { // ���������һ�µģ����Կ���ʹ��ѭ����
      if (bit_istrue(axis_words,bit(idx)) ) {
        gc_block.values.xyz[idx] *= MM_PER_INCH;
      }
    }
  }

  // [13�����߰뾶����]��G41 / 42��֧�֡����������G53��Чʱ���á�
  // [G40����]����G40֮����G2 / 3�������ú��ֱ���˶�С�ڵ���ֱ����
  //    ע�⣺���ڵ��߰뾶������δ���ã�������ЩG40�������á�Grbl֧��G40
  //    ������G40��g�������ͷһ����������Ĭ��ģʽʱ�������Ŀ�ġ�

  // [14���е����Ȳ���]��G43��֧�֣�����G43.1��G49�ǡ�
  // [G43.1����]��ͬһ���е��˶����
  //    ע�⣺����û����ȷ˵����G43.1Ӧ��ֻӦ����һ����Ч��
  //    ���õ��ᣨ��config.h�У���������õ���Ӧ���д���
  //    �����ڻ����Ƿ�����κ��������ᵥ�ʡ�
  if (axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { //��ʾ�ڿ��е��á� Indicates called in block.
    if (gc_block.modal.tool_length == TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC) {
      if (axis_words ^ (1<<TOOL_LENGTH_OFFSET_AXIS)) { FAIL(STATUS_GCODE_G43_DYNAMIC_AXIS_ERROR); }
    }
  }

  // [15������ϵѡ��]��* N / A������������߰뾶�������
  // TODO����ȡ�������ݵ�EEPROM��ѭ��ʱ������Ҫ������ͬ��
  //���ڻ״̬ ����ʱ��ͣ�����������ܻᵼ�º����ı���������
  //���㹻�ڴ�Ĵ�������δ���汾�����е���������Ӧ�ñ��洢
  //�洢���У�����ֻ�е�û�����ڼ���ʱ��д��EEPROM��
  float block_coord_system[N_AXIS];
  memcpy(block_coord_system,gc_state.coord_system,sizeof(gc_state.coord_system));
  if ( bit_istrue(command_words,bit(MODAL_GROUP_G12)) ) { // ����Ƿ��ڿ��е���
    if (gc_block.modal.coord_select > N_COORDINATE_SYSTEM) { FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS); } //[����N sys] [Greater than N sys]
    if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
      if (!(settings_read_coord_data(gc_block.modal.coord_select,block_coord_system))) { FAIL(STATUS_SETTING_READ_FAIL); }
    }
  }

  // [16������·������ģʽ]��N / A��ֻ��G61��G61.1��G64��֧�֡�
  // [17�����þ���ģʽ]��N / A��ֻ��G91.1��G90.1��֧�֡�
  // [18����������ģʽ]����֧�֡�

  // [19��ʣ���ģ̬����]����ѡת��Ԥ��λ�ã�����G10��������ƫ�á�
  //ע�⣺������Ҫ��ʹ�������ƣ�G10 / G28 / G30 / G92���ķ�ģ̬����ֿ���
  //����Բ�ͬ�ķ�ʽ�����ᵥ�ʡ�G10��Ϊ����ƫ�ƻ���㵱ǰλ��
  //��ֵ��G92��G10 L20��G28 / 30һ������Ϊ�۲���м�Ŀ��λ��
  //���е�ǰ����ϵ��G92ƫ������
  switch (gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:
        // [G10����]��L��ʧ������2��20. P�ֶ�ʧ������Pֵ����ɣ�
        // [G10 L2����]��R�ֲ�֧�֡�Pֵ����0��nCoordSys�����9�������ֲ�����
        // [G10 L20 Errors]��P������0��nCoordSys�����9�������ֲ�����
      if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS) }; // [No axis words]
      if (bit_isfalse(value_words,((1<<WORD_P)|(1<<WORD_L)))) { FAIL(STATUS_GCODE_VALUE_WORD_MISSING); } // [P/L word missing]
      coord_select = truncf(gc_block.values.p); // Convert p value to int.
      if (coord_select > N_COORDINATE_SYSTEM) { FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS); } // [Greater than N sys]
      if (gc_block.values.l != 20) {
        if (gc_block.values.l == 2) {
          if (bit_istrue(value_words,bit(WORD_R))) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G10 L2 R not supported]
        } else { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [Unsupported L]
      }
      bit_false(value_words,(bit(WORD_L)|bit(WORD_P)));

      //ȷ������ϵ�ı䲢���Դ�EEPROM���ء�
      if (coord_select > 0) { coord_select--; } // �� P1-P6��������ΪEEPROM��������������
      else { coord_select = gc_block.modal.coord_select; } // �� P0ָ��Ϊ�����ϵ
      
      // ע�⣺���������ݴ洢��IJKֵ�С��������������û�б�ʹ�á�
      if (!settings_read_coord_data(coord_select,gc_block.values.ijk)) { FAIL(STATUS_SETTING_READ_FAIL); } // [EEPROM read fail]

      //Ԥ�ȼ����������ݵı仯��
      for (idx=0; idx<N_AXIS; idx++) { // ���������һ�µģ����Կ���ʹ��ѭ����
        // ���½��ڿ��ж�����ᡣʼ���ڻ��������С����Ըı������ϵͳ��
        if (bit_istrue(axis_words,bit(idx)) ) {
          if (gc_block.values.l == 20) {
            // L20��ʹ�ñ��ֵ���µ�ǰλ�ã����޸�����������ϵ��
            // WPos = MPos - WCS - G92 - TLO  ->  WCS = MPos - G92 - TLO - WPos
            gc_block.values.ijk[idx] = gc_state.position[idx]-gc_state.coord_offset[idx]-gc_block.values.xyz[idx];
            if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.ijk[idx] -= gc_state.tool_length_offset; }
          } else {
            // L2��������ϵ�����Ϊ���ֵ��
            gc_block.values.ijk[idx] = gc_block.values.xyz[idx];
          }
        } // ���򣬱��ֵ�ǰ�Ĵ洢ֵ��
      }
      break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
      // [G92����]�������֡�
      if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [������]

      //���½��ڿ��ж�����ᡣ����ǰϵͳƫ�Ƶ������ֵ��������ʱ
      //ѡ�񼤻������ϵ�����ǳ���G92.1����������������Ȼ�ǻ�ġ�
      for (idx=0; idx<N_AXIS; idx++) { // ���������һ�µģ����Կ���ʹ��ѭ����
        if (bit_istrue(axis_words,bit(idx)) ) {
          // WPos = MPos - WCS - G92 - TLO  ->  G92 = MPos - WCS - TLO - WPos
          gc_block.values.xyz[idx] = gc_state.position[idx]-block_coord_system[idx]-gc_block.values.xyz[idx];
          if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.xyz[idx] -= gc_state.tool_length_offset; }
        } else {
          gc_block.values.xyz[idx] = gc_state.coord_offset[idx];
        }
      }
      break;

    default:

        //��ʱ����ʽ����������ಿ�ֽ���ֵ��Ϊ��ͳֵ
        //������ϵƫ�ƣ�G92ƫ�ƣ����Ը��Ǻ;���ΪĿ��λ��
        //Ӧ�õ�ģʽ��������˶�ģʽ����������ڿ���Ԥ�ȼ���Ŀ��λ�á�
        //ע�⣺�������˴˹��ܣ����������Щת���и��ӵ��߲�����
      if (axis_command != AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // TLO��ֹ�κ������
        if (axis_words) {
          for (idx=0; idx<N_AXIS; idx++) { // ��������һ�µģ�����ѭ������������������ռ䡣.
            if ( bit_isfalse(axis_words,bit(idx)) ) {
              gc_block.values.xyz[idx] = gc_state.position[idx]; //����û����ĵ��ʡ�������ͬ����λ�á� No axis word in block. Keep same axis position.
            } else {
                //���ݾ���ģʽ����ָ��ֵ�������ھ��Ը��Ǽ���ʱ���ԡ�
                //ע�⣺����G28 / 30��ͬһ��ģ̬���У�G53���ᴦ�ڼ���״̬��
              if (gc_block.non_modal_command != NON_MODAL_ABSOLUTE_OVERRIDE) {
                // ���ݾ���ģʽӦ������ƫ������
                if (gc_block.modal.distance == DISTANCE_MODE_ABSOLUTE) {
                  gc_block.values.xyz[idx] += block_coord_system[idx] + gc_state.coord_offset[idx];
                  if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.xyz[idx] += gc_state.tool_length_offset; }
                } else {  // ����ģʽ
                  gc_block.values.xyz[idx] += gc_state.position[idx];
                }
              }
            }
          }
        }
      }

      // ���ʣ��ķ�ģ̬�����Ƿ��д���
      switch (gc_block.non_modal_command) {
        case NON_MODAL_GO_HOME_0: // G28
        case NON_MODAL_GO_HOME_1: // G30
            // [G28 / 30����]�����߲��������á�
            //��EEPROM��ȡG28 / 30��ԭ��λ�����ݣ��ڻ��������У�
            //ע�⣺���������ݴ洢��IJKֵ�С��������������û�б�ʹ�á�
          if (gc_block.non_modal_command == NON_MODAL_GO_HOME_0) {
            if (!settings_read_coord_data(SETTING_INDEX_G28,gc_block.values.ijk)) { FAIL(STATUS_SETTING_READ_FAIL); }
          } else { // == NON_MODAL_GO_HOME_1
            if (!settings_read_coord_data(SETTING_INDEX_G30,gc_block.values.ijk)) { FAIL(STATUS_SETTING_READ_FAIL); }
          }
          if (axis_words) {
            //ֻ�ƶ��ڶ����ƶ���ָ�����ᡣ Move only the axes specified in secondary move.
            for (idx=0; idx<N_AXIS; idx++) {
              if (!(axis_words & (1<<idx))) { gc_block.values.ijk[idx] = gc_state.position[idx]; }
            }
          } else {
            axis_command = AXIS_COMMAND_NONE; //���û���м��˶�������Ϊ�ޡ� Set to none if no intermediate motion.
          }
          break;
        case NON_MODAL_SET_HOME_0: // G28.1
        case NON_MODAL_SET_HOME_1: // G30.1
            // [G28.1 / 30.1����]�����߲������á�
            //ע�⣺��������ﴫ���Ự�����Ǳ�����Ϊ��ʽ�˶�ģʽ��
          break;
        case NON_MODAL_RESET_COORDINATE_OFFSET:
          //ע�⣺��������ﴫ���Ự�����Ǳ�����Ϊ��ʽ�˶�ģʽ�� NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
          break;
        case NON_MODAL_ABSOLUTE_OVERRIDE:
            // [G53����]��G0��G1û�м�����߲��������á�
            //ע�⣺������ʽ��������������ģ̬���С����Բ���Ҫ��ʽ��顣
          if (!(gc_block.modal.motion == MOTION_MODE_SEEK || gc_block.modal.motion == MOTION_MODE_LINEAR)) {
            FAIL(STATUS_GCODE_G53_INVALID_MOTION_MODE); // [G53 G0/1 ��Ч]
          }
          break;
      }
  }

  // [20. �˶�ģʽ ]:
  if (gc_block.modal.motion == MOTION_MODE_NONE) {
	    // [G80����]����G80��Чʱ�����ֱ���̡�
	    //ע�⣺��ʹ��ģ̬�����ʹ��������TLOҲ���׳�����ϸ�Ĵ���
    if (axis_words) { FAIL(STATUS_GCODE_AXIS_WORDS_EXIST); } // [No axis words allowed]

    //���ʣ����˶�ģʽ����������������ģ�G10 / 28/30/92���Ѿ����ں�δʹ�ã�����
    //��g���������ȷ�����
  } else if ( axis_command == AXIS_COMMAND_MOTION_MODE ) {

    if (gc_block.modal.motion == MOTION_MODE_SEEK) {
        // [G0����]�����ź�û�����û�û��ʵ��ֵ����ɣ�
        //����ǿ�ѡ�ġ����ȱ�٣��������������־����ִ�С�
      if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

      //����ʣ����˶�ģʽ����G0��G80�⣩����Ҫ��Ч�Ľ�������ֵ����ÿ���׵�λģʽ�£�
      //ֵ��������ֵ���ڷ�ʱ��ģʽ�£�ÿ������봫��һ����ֵ��
    } else {
      //����Ƿ�Ϊ��Ҫ���˶�ģʽ�����˽������ʡ� Check if feed rate is defined for the motion modes that require it.
      if (gc_block.values.f == 0.0f) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); } //[������δ����] [Feed rate undefined]

      switch (gc_block.modal.motion) {
        case MOTION_MODE_LINEAR:
            // [G1����]��������δ���塣���ż�û�����û�û��ʵ�ʼ�ֵ��
            //����ǿ�ѡ�ġ����ȱ�٣��������������־����ִ�С�
          if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

          break;
        case MOTION_MODE_CW_ARC: 
          gc_parser_flags |= GC_PARSER_ARC_IS_CLOCKWISE; //û�д������ No break intentional.
        case MOTION_MODE_CCW_ARC:
            // [G2 / 3 Errors All-Modes]����������δ���塣
            // [G2 / 3�뾶ģʽ����]����ѡ��ƽ����û�������֡�Ŀ����뵱ǰ��ͬ��
            // [G2 / 3ƫ��ģʽ����]������ѡƽ����û������ֺ�/��ƫ�ơ�����ǰ�İ뾶
            //    ���Ŀ���İ뾶���쳬��0.002mm��EMC�߶�0.5mm��0.005mm��0.1���뾶����
            // [G2 / 3ȫ��ģʽ����]����֧�֡���Ļ����ڡ�û��ƫ�Ʊ�̡�P������һ��������
            //ע�⣺Բ����������İ뾶��ƫ�������Ǵ������Ԥ�ȼ��㡣

          if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
          if (!(axis_words & (bit(axis_0)|bit(axis_1)))) { FAIL(STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE); } //[ƽ����û����ĵ���] [No axis words in plane]

          //������ÿ��ѡ�����λ�ñ仯 Calculate the change in position along each selected axis
          float x,y;
          x = gc_block.values.xyz[axis_0]-gc_state.position[axis_0]; // ������ x ��ǰλ�ú�Ŀ��֮��
          y = gc_block.values.xyz[axis_1]-gc_state.position[axis_1]; // ������ y ��ǰλ�ú�Ŀ��֮��

          if (value_words & bit(WORD_R)) { // Բ���뾶ģʽ
            bit_false(value_words,bit(WORD_R));
            if (isequal_position_vector(gc_state.position, gc_block.values.xyz)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Ŀ����Ч]

            // ���뾶ֵת��Ϊ�ʵ��ĵ�λ��
            if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.r *= MM_PER_INCH; }
            /*  ������Ҫ�������ָ���뾶��ͨ����Բ������
                ͨ����ǰλ�ú�Ŀ��λ�á��÷���������������
                �����飬����[x��y]�Ǵӵ�ǰλ�õ�Ŀ��λ�õ�ʸ����d ==����
                �Ǹ�ʸ����h ==б�����γɵ������ε�Բ�ģ�����Ϊ
                �������������ġ���ʸ����ֱ��ʸ��[-y��x]�����ŵ�
                ����Ϊh [-y / d * h��x / d * h]����ӵ��г�����[x / 2��y / 2]�����ģ��γ��µĵ�
                [i��j]��[x / 2-y / d * h��y / 2 + x / d * h]���⽫�����ǻ������ġ�

                d^2 == x^2 + y^2
                h^2 == r^2 - (d/2)^2
                i == x/2 - y/d*h
                j == y/2 + x/d*h

                                                                     O <- [i,j]
                                                                  -  |
                                                        r      -     |
                                                            -        |
                                                         -           | h
                                                      -              |
                                        [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                                  | <------ d/2 ---->|

                C - ��ǰλ��
                T - Ŀ��λ��
                O - ͨ��C��T��Բ������
                d - ��C��T�ľ���
                r - ָ���İ뾶
                h - ��CT���ĵ�O�ľ���

                                                ��չ���̣�

                d -> sqrt(x^2 + y^2)
                h -> sqrt(4 * r^2 - x^2 - y^2)/2
                i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
                j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

                                                ����д��:

                i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
                j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

                                                 ���ǵĳߴ���ٶȵ�ԭ�����Ż�:

                h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
                i = (x - (y * h_x2_div_d))/2
                j = (y + (x * h_x2_div_d))/2
            */

            //���ȣ�ʹ��h_x2_div_d������4 * h ^ 2��������Ƿ�Ϊ������r��С��
            //��d �����������������sqrt�Ǹ��ӵģ�����ġ�
            float h_x2_div_d = 4.0f * gc_block.values.r*gc_block.values.r - x*x - y*y;

            if (h_x2_div_d < 0) { FAIL(STATUS_GCODE_ARC_RADIUS_ERROR); } // [���뾶���]

            // ��ɼ��� h_x2_div_d.
            h_x2_div_d = -sqrtf(h_x2_div_d)/hypot_f(x,y); // == -(h * 2 / d)
            // ��תh_x2_div_d�ķ��ţ�����ͼ��ʾ��
            if (gc_block.modal.motion == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d; }

            /* ��ʱ��ԲȦλ��Ŀ�귽�����ࡣ�������ǻ����ģ�
               �������ߵ�ԲȦ - �����Ǹ�ֵʱ�������ұߵ�ԲȦ��

                                                                   T  <-- Ŀ��λ��

                                                                   ^
                                                                                                               �Դ�����˳ʱ��ԲȦ ������ĵ�˳ʱ��Բ����         |          Clockwise circles with this center will have
                                                                                       ������> 180�ȵĽ��г� ���г�<180�ȣ�����һ�����£�      |          < 180 deg of angular travel, which is a good thing!
                                                         \         |          /
                                                                                           ��h_x2_div_d����ֵʱ������Բ�� ->  x <----- | -----> x <- ��h_x2_div_dΪ����ʱ��������
                                                                   |
                                                                   |

                                                                   C  <-- ��ǰλ��
            */
            // Negative R��g���� - ������Ҫһ������180�ȵ��г̵�Բ����ȥͼ������
            //��ʹ���鲻Ҫ��һ��g����������������ԲȦ��ͨ��
            //��תh_x2_div_d�ķ��ţ�Բ������λ��
            //���У�������ǵõ��˲���Ԥ���ĳ�����
            if (gc_block.values.r < 0) {
                h_x2_div_d = -h_x2_div_d;
                gc_block.values.r = -gc_block.values.r; //���r������Ϊmc_arcΪ��ֵ Finished with r. Set to positive for mc_arc
            }
            // ͨ������Բ����ʵ����������ɲ���
            gc_block.values.ijk[axis_0] = 0.5f*(x-(y*h_x2_div_d));
            gc_block.values.ijk[axis_1] = 0.5f*(y+(x*h_x2_div_d));

          } else { // �����ĸ�ʽƫ��ģʽ
            if (!(ijk_words & (bit(axis_0)|bit(axis_1)))) { FAIL(STATUS_GCODE_NO_OFFSETS_IN_PLANE); } // [No offsets in plane]
            bit_false(value_words,(bit(WORD_I)|bit(WORD_J)|bit(WORD_K)));

            // ��IJKֵת��Ϊ�ʵ��ĵ�λ.
            if (gc_block.modal.units == UNITS_MODE_INCHES) {
              for (idx=0; idx<N_AXIS; idx++) { //��������һ�µģ�����ѭ������������������ռ� Axes indices are consistent, so loop may be used to save flash space.
                if (ijk_words & bit(idx)) { gc_block.values.ijk[idx] *= MM_PER_INCH; }
              }
            }

            // �����ĵ�Ŀ���Բ���뾶
            x -= gc_block.values.ijk[axis_0]; // ������ x ��ǰλ�ú�Ŀ��֮��
            y -= gc_block.values.ijk[axis_1]; // ������ y ��ǰλ�ú�Ŀ��֮��
            float target_r = hypot_f(x,y);

            //����mc_arc�Ļ��뾶���ӵ�ǰλ�ö��嵽���ġ� Compute arc radius for mc_arc. Defined from current location to center.
            gc_block.values.r = hypot_f(gc_block.values.ijk[axis_0], gc_block.values.ijk[axis_1]);

            //���㵱ǰλ�ú�Ŀ��뾶֮��Ĳ����Խ������յĴ����顣 Compute difference between current location and target radii for final error-checks.
            float delta_r = fabsf(target_r-gc_block.values.r);
            if (delta_r > 0.005f) {
              if (delta_r > 0.5f) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [���߶������] > 0.5mm
              if (delta_r > (0.001f*gc_block.values.r)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Բ���������] > 0.005mm AND �뾶Ϊ0.1��
            }
          }
          break;
        case MOTION_MODE_PROBE_TOWARD_NO_ERROR: case MOTION_MODE_PROBE_AWAY_NO_ERROR:
            gc_parser_flags |= GC_PARSER_PROBE_IS_NO_ERROR; //û�д������ No break intentional.
        case MOTION_MODE_PROBE_TOWARD: case MOTION_MODE_PROBE_AWAY:
            if ((gc_block.modal.motion == MOTION_MODE_PROBE_AWAY) ||
                (gc_block.modal.motion == MOTION_MODE_PROBE_AWAY_NO_ERROR)) { gc_parser_flags |= GC_PARSER_PROBE_IS_AWAY; }
            // [G38����]��Ŀ������ͬ�ĵ�����û����Ļ������߲��������á�������
            //    ��δ����� ̽������������ע�⣺̽����������̽��ѭ���������Ƿ���
            //    �����������ᷢ�������Է�ֹ��һ���ƶ���̽�롣��Ҳ����������ɵ�
            //    ����һ��̽��ѭ��֮ǰ������滮����������ղ��Ƴ�̽������������
          if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
          if (isequal_position_vector(gc_state.position, gc_block.values.xyz)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Invalid target]
          break;
      }
    }
  }

  // [21����������]������Ҫ���д����顣

  // [0�����ض�������]�����δʹ�õ�ֵ�ּ�飬��IJK�ڻ���ʹ��
  // �뾶ģʽ�����ڿ���δʹ�õ��ᵥ�ʡ�
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) {
      // ����ֻʹ��F�������ʺ�XYZֵ�֡�N��Ч����S��T��Ч��
      bit_false(value_words, (bit(WORD_N) | bit(WORD_F)));
  } else {
      bit_false(value_words, (bit(WORD_N) | bit(WORD_F) | bit(WORD_S) | bit(WORD_T))); // Remove single-meaning value words.
  }
  if (axis_command) { bit_false(value_words,(bit(WORD_X)|bit(WORD_Y)|bit(WORD_Z))); } // Remove axis words.
  if (value_words) { FAIL(STATUS_GCODE_UNUSED_WORDS); } // [Unused words]

  /* -------------------------------------------------------------------------------------
     ��4����ִ�У�
     �������д������Ѿ���ɣ����Ҳ�����ʧ��ģʽ�����Ǹո�
     ��Ҫ����ִ��˳�����״̬��ִ�иÿ顣
  */

  // ��ʼ���˶���Ĺ滮�����ݽṹ��
  plan_line_data_t plan_data;
  plan_line_data_t *pl_data = &plan_data;
  memset(pl_data,0,sizeof(plan_line_data_t)); // ��pl_data�ṹ

  //��ȡ�㶯��������Ч�㶯����Ĵ����鲢ִ�С�
  //ע�⣺G���������״̬������£����˱�֤�����㶯��λ����
  //Ŀ�������ȷ���㶯�����������λ���ڸ�����
  //��������ɻ�ȡ��ʱ��protocol_execute_realtime������
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) {
      //ֻ����ʹ�þ��뵥λģָ̬���G53���Ը���ָ�
      //ע�⣺�ڲ���3���Ѿ�ִ���˽������ֺ����ֵļ�顣
      if (command_words & ~(bit(MODAL_GROUP_G3) | bit(MODAL_GROUP_G6 | bit(MODAL_GROUP_G0)))) { FAIL(STATUS_INVALID_JOG_COMMAND) };
      if (!(gc_block.non_modal_command == NON_MODAL_ABSOLUTE_OVERRIDE || gc_block.non_modal_command == NON_MODAL_NO_ACTION)) { FAIL(STATUS_INVALID_JOG_COMMAND); }

      // ���ƻ������ݳ�ʼ��Ϊ��ǰ���������ȴ��ģ̬��
      pl_data->spindle_speed = gc_state.spindle_speed;
      plan_data.condition = (gc_state.modal.spindle | gc_state.modal.coolant);

      uint8_t status = jog_execute(&plan_data, &gc_block);
      if (status == STATUS_OK) { memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz)); }
      return(status);
  }

  // ����ڼ���ģʽ�£����ݵ�ǰ�͹�ȥ�ķ������������ü��⹦�ʡ�
  if (bit_istrue(settings.flags, BITFLAG_LASER_MODE)) {
      if (!((gc_block.modal.motion == MOTION_MODE_LINEAR) || (gc_block.modal.motion == MOTION_MODE_CW_ARC)
          || (gc_block.modal.motion == MOTION_MODE_CCW_ARC))) {
          gc_parser_flags |= GC_PARSER_LASER_DISABLE;
      }

      //�κδ������ֵ��˶�ģʽ������������ٶȸ����д��ݡ�
      //ע�⣺G1��G0û����Ļ���axis_command����Ϊnone��G28 / 30����ʡ�ԡ�
      // TODO�����û�н���滮��������M3�Ķ�����ͬ�����������㳤�ȣ���
      if (axis_words && (axis_command == AXIS_COMMAND_MOTION_MODE)) {
        gc_parser_flags |= GC_PARSER_LASER_ISMOTION;
      }
      else {
        //  M3�㹦�ʼ�������Ҫ�ƻ���ͬ�������¼�����֮��ı仯
        // G1 / 2/3�˶�ģʽ״̬����֮��Ȼ��
        if (gc_state.modal.spindle == SPINDLE_ENABLE_CW) {
          if ((gc_state.modal.motion == MOTION_MODE_LINEAR) || (gc_state.modal.motion == MOTION_MODE_CW_ARC)
            || (gc_state.modal.motion == MOTION_MODE_CCW_ARC)) {
            if (bit_istrue(gc_parser_flags, GC_PARSER_LASER_DISABLE)) {
              gc_parser_flags |= GC_PARSER_LASER_FORCE_SYNC; //��G1/2/3�˶�ģʽ�л� Change from G1/2/3 motion mode.
            }
          }
          else {
            //�ӷ�G1 / 2/3�˶�ģʽ�л���G1�˶�ģʽʱ�����Ự� When changing to a G1 motion mode without axis words from a non-G1/2/3 motion mode.
            if (bit_isfalse(gc_parser_flags, GC_PARSER_LASER_DISABLE)) {
              gc_parser_flags |= GC_PARSER_LASER_FORCE_SYNC;
            }
          }
        }
      }
  }

  // [0�����ض�/�������������������]��
  //ע�⣺����������кţ����ֵΪ�㡣
  gc_state.line_number = gc_block.values.n;
  #ifdef USE_LINE_NUMBERS
    pl_data->line_number = gc_state.line_number; // ��¼�滮��Աʹ�õ����ݡ�
  #endif

  // [1���������]����֧��

  // [2�����ý�������ģʽ]��
  gc_state.modal.feed_rate = gc_block.modal.feed_rate;
  if (gc_state.modal.feed_rate) { pl_data->condition |= PL_COND_FLAG_INVERSE_TIME; } // ���ù滮��ʹ�õ�������־��

  // [3. �趨�������� ]:
  gc_state.feed_rate = gc_block.values.f; //���Ǹ������ֵ���鿴�����ʴ����� Always copy this value. See feed rate error-checking.
  pl_data->feed_rate = gc_state.feed_rate; //��¼�滮��Աʹ�õ����� Record data for planner use.

  // [4. ��������ת�� ]:
  if ((gc_state.spindle_speed != gc_block.values.s) || bit_istrue(gc_parser_flags, GC_PARSER_LASER_FORCE_SYNC)) {
      if (gc_state.modal.spindle != SPINDLE_DISABLE) {
#ifdef VARIABLE_SPINDLE
        if (bit_isfalse(gc_parser_flags, GC_PARSER_LASER_ISMOTION)) {
          if (bit_istrue(gc_parser_flags, GC_PARSER_LASER_DISABLE)) {
            spindle_sync(gc_state.modal.spindle, 0.0);
          }
          else { spindle_sync(gc_state.modal.spindle, gc_block.values.s); }
        }
#else
          spindle_sync(gc_state.modal.spindle, 0.0);
#endif
      }
      gc_state.spindle_speed = gc_block.values.s; // ��������ת��״̬
  }
  // ע�⣺�������޼����˶���ͨ��������ת��.
  if (bit_isfalse(gc_parser_flags, GC_PARSER_LASER_DISABLE)) {
      pl_data->spindle_speed = gc_state.spindle_speed; // ��¼�滮��Աʹ�õ�����.
  } // else { pl_data->spindle_speed = 0.0; } // �Ѿ���ʼ��Ϊ��.

  // [5. ѡ�񹤾� ]: ��֧�֡�ֻ���ٹ���ֵ.
  gc_state.tool = gc_block.values.t;

  // [6. ���� ]: ��֧��

  // [7. ������� ]:
  if (gc_state.modal.spindle != gc_block.modal.spindle) {
	    //����������Ʋ��ڴ�ģ��������ʱӦ������ת�١�
	    //ע�⣺��ʹ�ڼ���ģʽ�£����е�����״̬�ı�Ҳ��ͬ���ġ����⣬pl_data��
	    //������gc_state�����ڹ���Ǽ����˶��ļ���״̬��
    spindle_sync(gc_block.modal.spindle, pl_data->spindle_speed);
    gc_state.modal.spindle = gc_block.modal.spindle;
  }
  pl_data->condition |= gc_state.modal.spindle; // Set condition flag for planner use.

  // [8. ��ȴҺ���� ]:
  if (gc_state.modal.coolant != gc_block.modal.coolant) {
	    //ע�⣺��ȴ��M������ģ̬�ġ�ÿ��ֻ����һ��������ǣ��������
	    //����ͬʱ���ڣ�����ȴҺ�����������״̬��
    coolant_sync(gc_block.modal.coolant);
    if (gc_block.modal.coolant == COOLANT_DISABLE) { gc_state.modal.coolant = COOLANT_DISABLE; }
    else { gc_state.modal.coolant |= gc_block.modal.coolant; }
  }
  pl_data->condition |= gc_state.modal.coolant; //���ù滮��ʹ�õ�������־ Set condition flag for planner use.

	// [9�����ǿ���]����֧�֡�ʼ�����á�����Grblר��ͣ�����ơ�
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
	if (gc_state.modal.override != gc_block.modal.override) {
		gc_state.modal.override = gc_block.modal.override;
		mc_override_ctrl_update(gc_state.modal.override);
	}
#endif

  // [10. ��ͣ ]:
  if (gc_block.non_modal_command == NON_MODAL_DWELL) { mc_dwell(gc_block.values.p); }

  // [11. ���ûƽ�� ]:
  gc_state.modal.plane_select = gc_block.modal.plane_select;

  // [12. ���ó��ȵ�λ ]:
  gc_state.modal.units = gc_block.modal.units;

  // [13. ���߰뾶���� ]: G41/42 ��֧��
  // gc_state.modal.cutter_comp = gc_block.modal.cutter_comp; //ע�⣺����Ҫ����Ϊһֱ���� NOTE: Not needed since always disabled.

  // [14. �������� ]: ֧��G43.1��G49��G43��֧�֡�.
  //ע�⣺���֧��G43�������з�ʽ��G43.1û���κ�����
  //ִ�� �����鲽�轫�򵥵ؽ�ƫ��ֵ���ص���ȷ��λ��
  //��XYZֵ������ᡣ
  if (axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // ָʾ���ġ�
    gc_state.modal.tool_length = gc_block.modal.tool_length;
    if (gc_state.modal.tool_length == TOOL_LENGTH_OFFSET_CANCEL) { // G49
      gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS] = 0.0f;
    } // else G43.1
    if ( gc_state.tool_length_offset != gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS] ) {
      gc_state.tool_length_offset = gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS];
      system_flag_wco_change();
    }
  }

  // [15. ����ϵѡ�� ]:
  if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
    gc_state.modal.coord_select = gc_block.modal.coord_select;
    memcpy(gc_state.coord_system,block_coord_system,N_AXIS*sizeof(float));
    system_flag_wco_change();
  }

  // [16. ����·������ģʽ ]: G61.1/G64 ��֧��
  // gc_state.modal.control = gc_block.modal.control; // ע�⣺ʼ��Ĭ��.

  // [17. ���þ���ģʽ ]:
  gc_state.modal.distance = gc_block.modal.distance;

  // [18. ��������ģʽ ]: ��֧��

  // [19. ת��Ԥ�����λ�ã�����G10��������ƫ�� ]:
  switch(gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:
      settings_write_coord_data(coord_select,gc_block.values.ijk);
      // �����ǰ��Ч������ϵͳ����ϵ.
      if (gc_state.modal.coord_select == coord_select) {
        memcpy(gc_state.coord_system,gc_block.values.ijk,N_AXIS*sizeof(float));
        system_flag_wco_change();
      }
      break;
    case NON_MODAL_GO_HOME_0: case NON_MODAL_GO_HOME_1:
      //�ؼ�֮ǰ�Ƶ��м�λ�� ���ص�ǰ������ϵ��ƫ��   Move to intermediate position before going home. Obeys current coordinate system and offsets
      //�;��Ժ�����ģʽ�� and absolute and incremental modes.
      pl_data->condition |= PL_COND_FLAG_RAPID_MOTION; // ���ÿ����˶�������־��
      if (axis_command) { mc_line(gc_block.values.xyz, pl_data); }
      mc_line(gc_block.values.ijk, pl_data);
      memcpy(gc_state.position, gc_block.values.ijk, N_AXIS*sizeof(float));
      break;
    case NON_MODAL_SET_HOME_0:
      settings_write_coord_data(SETTING_INDEX_G28,gc_state.position);
      break;
    case NON_MODAL_SET_HOME_1:
      settings_write_coord_data(SETTING_INDEX_G30,gc_state.position);
      break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
      memcpy(gc_state.coord_offset,gc_block.values.xyz,sizeof(gc_block.values.xyz));
      system_flag_wco_change();
      break;
    case NON_MODAL_RESET_COORDINATE_OFFSET:
      clear_vector(gc_state.coord_offset); //ͨ������ƫ��ʸ��������G92ƫ�ơ� Disable G92 offsets by zeroing offset vector.
      system_flag_wco_change();
      break;
  }


  // [20. �˶�ģʽ ]:
  // ע�⣺����G10��G28��G30��G92��������ֹ�����˶�ģʽ��ʹ�á�
  // ֻ���ڳ�����������ֻ��˶�ģʽ�����ֵ�����²��ܽ����˶�ģʽ��
  gc_state.modal.motion = gc_block.modal.motion;
  if (gc_state.modal.motion != MOTION_MODE_NONE) {
    if (axis_command == AXIS_COMMAND_MOTION_MODE) {
      uint8_t gc_update_pos = GC_UPDATE_POS_TARGET;
      if (gc_state.modal.motion == MOTION_MODE_LINEAR) {
        mc_line(gc_block.values.xyz, pl_data);
      } else if (gc_state.modal.motion == MOTION_MODE_SEEK) {
        pl_data->condition |= PL_COND_FLAG_RAPID_MOTION; // ���ÿ����˶�������־��
        mc_line(gc_block.values.xyz, pl_data);
      } else if ((gc_state.modal.motion == MOTION_MODE_CW_ARC) || (gc_state.modal.motion == MOTION_MODE_CCW_ARC)) {
          mc_arc(gc_block.values.xyz, pl_data, gc_state.position, gc_block.values.ijk, gc_block.values.r,
              axis_0, axis_1, axis_linear, bit_istrue(gc_parser_flags, GC_PARSER_ARC_IS_CLOCKWISE));
      } else {
        // ע�⣺gc_block.values.xyz�Ǵ�mc_probe_cycle���ظ��º��λ��ֵ������
        // �ڳɹ���̽��ѭ���У�����λ�úͷ���ֵӦ������ͬ�ġ�
        #ifndef ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES
          pl_data->condition |= PL_COND_FLAG_NO_FEED_OVERRIDE;
        #endif
        gc_update_pos = mc_probe_cycle(gc_block.values.xyz, pl_data, gc_parser_flags);
    }  
     
      //�ͽ��������ԣ�λ��������==Ŀ�ꡣ����ʵ��
      //�˶�����ϵͳ������Ȼ�ڴ���������ʵ�Ĺ���λ��
      //���κ��м�λ��
      if (gc_update_pos == GC_UPDATE_POS_TARGET) {
        memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz)); // gc_state.position[] = gc_block.values.xyz[]
      } else if (gc_update_pos == GC_UPDATE_POS_SYSTEM) {
        gc_sync_position(); // gc_state.position[] = sys_position
      } // == GC_UPDATE_POS_NONE
    }     

  }

  // [21����������]��
  // M0��M1��M2��M30��ִ�з����еĳ������������ڳ�����ͣ�ڼ䣬����������
  //������䣬ֻ����cycle start run-time����ָ���
  gc_state.modal.program_flow = gc_block.modal.program_flow;
  if (gc_state.modal.program_flow) {
    protocol_buffer_synchronize(); // ����ǰ����ͬ�����������ʣ��Ļ����˶���
    if (gc_state.modal.program_flow == PROGRAM_FLOW_PAUSED) {
      if (sys.state != STATE_CHECK_MODE) {
        system_set_exec_state_flag(EXEC_FEED_HOLD); // ʹ��feed hold��ͣ����
        protocol_execute_realtime(); // ִ�й���
      }
    } else { // == PROGRAM_FLOW_COMPLETED
        //��ɳ���󣬸���gid��ֻ��һ����g��������ΪĳЩĬ��ֵ
        // LinuxCNC�ĳ�����������Ͳ��ԡ�ֻ��ģ̬��[G����1,2,3,5,7,12]
        // [M����7,8,9]��λΪ[G1��G17��G90��G94��G40��G54��M5��M9��M48]�������ģ̬��
        // [G����4,6,8,10,13,14,15]��[M����4,5,6]��ģ̬��[F��S��T��H]�������á�
      gc_state.modal.motion = MOTION_MODE_LINEAR;
      gc_state.modal.plane_select = PLANE_SELECT_XY;
      gc_state.modal.distance = DISTANCE_MODE_ABSOLUTE;
      gc_state.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
      // gc_state.modal.cutter_comp = CUTTER_COMP_DISABLE; // ��֧�֡�
      gc_state.modal.coord_select = 0; // G54
      gc_state.modal.spindle = SPINDLE_DISABLE;
      gc_state.modal.coolant = COOLANT_DISABLE;
			#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
				#ifdef DEACTIVATE_PARKING_UPON_INIT
					gc_state.modal.override = OVERRIDE_DISABLED;
				#else
					gc_state.modal.override = OVERRIDE_PARKING_MOTION;
				#endif
			#endif

      #ifdef RESTORE_OVERRIDES_AFTER_PROGRAM_END
        sys.f_override = DEFAULT_FEED_OVERRIDE;
        sys.r_override = DEFAULT_RAPID_OVERRIDE;
        sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE;
      #endif

      // ִ������任������/��ȴҺֹͣ��
      if (sys.state != STATE_CHECK_MODE) {
        if (!(settings_read_coord_data(gc_state.modal.coord_select,gc_state.coord_system))) { FAIL(STATUS_SETTING_READ_FAIL); }
        system_flag_wco_change(); // ����Ϊ����ˢ�£��Է���һ�����ı䡣
        spindle_set_state(SPINDLE_DISABLE,0.0f);
        coolant_set_state(COOLANT_DISABLE);
      }
      report_feedback_message(MESSAGE_PROGRAM_END);
    }
    gc_state.modal.program_flow = PROGRAM_FLOW_RUNNING; // ���ó�������
  }

  // TODO: % ��ʾ����Ŀ�ʼ.

  return(STATUS_OK);
}


/*
  ��֧�֣�
  - �̶�ѭ��
  - ���߰뾶����
  -  A��B��C��
  - ��������
  - ����
  - ���ǿ��ƣ�TBD��
  - ���߸���
  - ����
   ��*����ʾ��ѡ������ͨ��config.h���ò����±���
   ��0 = {G92.2��G92.3}����ģ̬��ȡ������������G92ƫ�ƣ�
   ��1 = {G81  -  G89}���˶�ģʽ���̶�ѭ����
   ��4 = {M1}����ѡֹͣ�����ԣ�
   ��6 = {M6}��������
   ��7 = {G41��G42}���߰뾶������֧��G40��
   ��8 =���߳��Ȳ���{G43}��֧��G43.1 / G49��
   ��8 = {M7 *}��������ȴ����*����ѡ�
	 ��9 = {M48��M49��M56 *}����/���ø��ǿ��أ�*����ѡ�
	 ��10 = {G98��G99}����ģʽ�̶�ѭ��
   ��13 = {G61.1��G64}·������ģʽ��֧��G61��
*/
