/*
  gcode.c - rs274 / ngc解析器。
  G代码解析器
  按照 rs274/ngc 标准进行解析
*/

#include "grbl.h"

//注：最大行数由g代码标准定义为99999.它似乎是一个
//任意值，有些GUI可能需要更多。所以我们增加了基于最大安全
//将float（7.2数字精度）转换为整数时的值。
#define MAX_LINE_NUMBER 10000000
#define MAX_TOOL_NUMBER 255 // 由最大无符号8位值的限制. Limited by max unsigned 8-bit value

#define AXIS_COMMAND_NONE 0
#define AXIS_COMMAND_NON_MODAL 1
#define AXIS_COMMAND_MOTION_MODE 2
#define AXIS_COMMAND_TOOL_LENGTH_OFFSET 3 //*不定，但需要    *Undefined but required

// 声明gc extern结构体
parser_state_t gc_state;
parser_block_t gc_block;

#define FAIL(status) return(status);


void gc_init()
{
  memset(&gc_state, 0, sizeof(parser_state_t));

  // 加载默认的G54坐标系
  if (!(settings_read_coord_data(gc_state.modal.coord_select,gc_state.coord_system))) {
    report_status_message(STATUS_SETTING_READ_FAIL);
  }
}


//以 g为单位设置g代码解析器的位置。输入步骤。由系统调用中止和努力
//限制拉开例程。
void gc_sync_position()
{
  system_convert_array_steps_to_mpos(gc_state.position,sys_position);
}


//执行一行以0结尾的G代码。该行假定只包含大写字母
//字符和带符号的浮点值（无空格）。评论和块删除
//字符已被删除。在这个功能中，所有单位和职位都被转换
//以（mm，mm / min）和绝对机器的形式输出到grbl的内部功能
//坐标，分别。
uint8_t gc_execute_line(char *line)
{
  /* -------------------------------------------------------------------------------------
     步骤1：初始化解析器块结构并复制当前的g代码状态模式。解析器
     更新这些模式和命令，因为块行是解析器，只会被使用
     成功的错误检查后执行。解析器块结构也包含一个块
     值结构，字跟踪变量和一个非模态命令跟踪器的新
     块。这个结构体包含了执行这个块所需的全部信息。 */
  
  memset(&gc_block, 0, sizeof(parser_block_t)); //初始化解析器块结构。 Initialize the parser block struct.
  memcpy(&gc_block.modal,&gc_state.modal,sizeof(gc_modal_t)); //复制当前模式 Copy current modes

  uint8_t axis_command = AXIS_COMMAND_NONE;
  uint8_t axis_0, axis_1, axis_linear;
  uint8_t coord_select = 0; //跟踪G10 P坐标选择执行 Tracks G10 P coordinate selection for execution

  //为轴索引兼容操作初始化位标志跟踪变量。 Initialize bitflag tracking variables for axis indices compatible operations.
  uint8_t axis_words = 0; //XYZ跟踪  标志哪个轴需要处理
  uint8_t ijk_words = 0; //IJK跟踪 IJK tracking

  //初始化命令和值字和分析器标志变量。 Initialize command and value words and parser flags variables.
  uint16_t command_words = 0; //跟踪G和M的命令字。也用于模态组违规。 Tracks G and M command words. Also used for modal group violations.
  uint16_t value_words = 0; //跟踪价值词。 Tracks value words.
  uint8_t gc_parser_flags = GC_PARSER_NONE;

  //确定该行是慢跑运动还是正常的g代码块。 Determine if the line is a jogging motion or a normal g-code block.
  if (line[0] == '$') { // 注意：当传递给这个函数时，已经解析了`$ J =`。NOTE: `$J=` already parsed when passed to this function.
						//设置G1和G94强制模式以确保准确的错误检查。 Set G1 and G94 enforced modes to ensure accurate error checks.
    gc_parser_flags |= GC_PARSER_JOG_MOTION;
	gc_block.modal.motion = MOTION_MODE_LINEAR;
    gc_block.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
#ifdef USE_LINE_NUMBERS
     gc_block.values.n = JOG_LINE_NUMBER; // Initialize default line number reported during jog.
#endif
  }

  /* -------------------------------------------------------------------------------------
     步骤2：在块行中导入所有的g代码字。一个g码字是一个字母后跟
     一个数字，既可以是“G”/“M”命令，也可以设置/分配命令值。也，
     对任何重复的命令字模态组违规执行初始错误检查
     对于值字F，N，P，T和S设定负值。 */

  uint8_t word_bit; //用于分配跟踪变量的位值 Bit-value for assigning tracking variables
  uint8_t char_counter;
  char letter;
  float value;
  uint8_t int_value = 0;
  uint16_t mantissa = 0;//Gxx.x命令的尾数
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) { char_counter = 3; } //在`$ J =`之后开始解析  Start parsing after `$J=`
  else { char_counter = 0; }

  while (line[char_counter] != 0) { //循环直到没有更多的g代码字符合。 Loop until no more g-code words in line.

    //导入下一个g代码字，期望一个字母后跟一个值。否则，出错。 Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
    letter = line[char_counter];
    if((letter < 'A') || (letter > 'Z')) { FAIL(STATUS_EXPECTED_COMMAND_LETTER); } //[预期字母] [Expected word letter]
    char_counter++;
    if (!read_float(line, &char_counter, &value)) { FAIL(STATUS_BAD_NUMBER_FORMAT); } //[期望的单词值] [Expected word value]

    //将值转换为更小的uint8有效数和尾数值来解析这个单词。
    //注意：尾数乘以100捕获非整数命令值。这是更多
    //比用于命令的x10的NIST gcode要求准确，但不完全
    //对于需要整数在0.0001以内的价值单词足够准确。这应该是
    //足够好的comprimise和捕获大多数非整数错误。为了使之合规，
    //我们只需要将尾数改为int16，但是这会增加编译的闪存空间。
    //稍后可能会更新。
    int_value = truncf(value);
	mantissa = (uint16_t)lroundf(100 * (value - int_value)); //计算Gxx.x命令的尾数。 Compute mantissa for Gxx.x commands.
    //注意：舍入必须用来捕获小的浮点错误。 NOTE: Rounding must be used to catch small floating point errors.

    //检查g代码字是否被支持，或者由于模态组违规或者错误导致错误
    //在g代码块中重复。如果可以，更新命令或记录它的值。
    switch(letter) {

      /* 'G'和'M'命令字：解析命令并检查模态组违规。
         注：模态组编号在NIST RS274-NGC第3版表4中定义，第20页 */

      case 'G':
        // 确定'G'命令及其模态组
        switch(int_value) {
          case 10: case 28: case 30: case 92:
            //检查在同一个程序段中用G0 / 1/2/3/38调用G10 / 28/30/92。 Check for G10/28/30/92 being called with G0/1/2/3/38 on same block.
            // * G43.1也是一个轴命令，但没有明确的定义。 * G43.1 is also an axis command but is not explicitly defined this way.
            if (mantissa == 0) { // Ignore G28.1, G30.1, and G92.1
              if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } //[轴词/命令冲突] [Axis word/command conflict]
              axis_command = AXIS_COMMAND_NON_MODAL;
            }
            //没有休息 继续下一行。 No break. Continues to next line.
          case 4: case 53:
            word_bit = MODAL_GROUP_G0;
            gc_block.non_modal_command = int_value;
            if ((int_value == 28) || (int_value == 30) || (int_value == 92)) {
              if (!((mantissa == 0) || (mantissa == 10))) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); }
              gc_block.non_modal_command += mantissa;
              mantissa = 0; //设置为零表示有效的非整数G命令。 Set to zero to indicate valid non-integer G command.
            }                
            break;
          case 0: case 1: case 2: case 3: case 38:
            //检查在同一个程序段中用G10 / 28/30/92调用的G0 / 1/2/3/38。 Check for G0/1/2/3/38 being called with G10/28/30/92 on same block.
            //* G43.1也是一个轴命令，但没有明确的定义。 * G43.1 is also an axis command but is not explicitly defined this way.
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } //[轴词/命令冲突] [Axis word/command conflict]
            axis_command = AXIS_COMMAND_MOTION_MODE;
            //没有休息 继续下一行。 No break. Continues to next line.
          case 80:
            word_bit = MODAL_GROUP_G1;
            gc_block.modal.motion = int_value;
            if (int_value == 38){
              if (!((mantissa == 20) || (mantissa == 30) || (mantissa == 40) || (mantissa == 50))) {
                FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); //[不支持的G38.x命令] [Unsupported G38.x command]
              }
              gc_block.modal.motion += (mantissa/10)+100;
              mantissa = 0; //设置为零表示有效的非整数G命令。 Set to zero to indicate valid non-integer G command.
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
              if ((mantissa != 10) || (int_value == 90)) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } //[不支持G90.1] [G90.1 not supported]
              mantissa = 0; //设置为零表示有效的非整数G命令。 Set to zero to indicate valid non-integer G command.
              //否则，弧IJK增量模式是默认的。G91.1什么都不做。 Otherwise, arc IJK incremental mode is default. G91.1 does nothing.
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
            //注意：由于刀具半径补偿始终被禁用，因此不需要。只有在这里 NOTE: Not required since cutter radius compensation is always disabled. Only here
            //支持经常出现在g代码程序头文件中的G40命令来设置默认值。 to support G40 commands that often appear in g-code program headers to setup defaults.
            // gc_block.modal.cutter_comp = CUTTER_COMP_DISABLE; // G40
            break;
          case 43: case 49:
            word_bit = MODAL_GROUP_G8;
            //注意：NIST的g代码标准隐约指出，当刀具长度补偿改变时，
            //不能有任何轴运动或坐标偏移更新。含义G43，G43.1和G49
            //所有的都是明确的轴命令，无论它们是否需要轴名。
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } //[轴单词/命令冲突]} [Axis word/command conflict] }
            axis_command = AXIS_COMMAND_TOOL_LENGTH_OFFSET;
            if (int_value == 49) { // G49
              gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_CANCEL;
            } else if (mantissa == 10) { // G43.1
              gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC;
            } else { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } //[不支持的G43.x命令] [Unsupported G43.x command]
            mantissa = 0; //设为零表示有效的非整数G命令。 Set to zero to indicate valid non-integer G command.
            break;
          case 54: case 55: case 56: case 57: case 58: case 59:
            ///注意：不支持G59.x。（但他们的int_values将是60,61和62.） NOTE: G59.x are not supported. (But their int_values would be 60, 61, and 62.)
            word_bit = MODAL_GROUP_G12;
            gc_block.modal.coord_select = int_value - 54; //切换到数组索引。 Shift to array indexing.
            break;
          case 61:
            word_bit = MODAL_GROUP_G13;
            if (mantissa != 0) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } //[不支持G61.1] [G61.1 not supported]
            // gc_block.modal.control = CONTROL_MODE_EXACT_PATH; // G61
            break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); //[不支持的G指令] [Unsupported G command]
        }
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } //[不支持或无效的Gxx.x命令] [Unsupported or invalid Gxx.x command]
        //在当前块中检查每个模态组违规的多个命令 Check for more than one command per modal group violations in the current block
        //注意：如果命令有效，变量'word_bit'总是被赋值。 NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;

      case 'M':

        // 确定'M'命令及其模态组
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } //[没有Mxx.x命令] [No Mxx.x commands]
        switch(int_value) {
          case 0: case 1: case 2: case 30:
            word_bit = MODAL_GROUP_M4;
            switch(int_value) {
              case 0: gc_block.modal.program_flow = PROGRAM_FLOW_PAUSED; break; //程序暂停  Program pause
              case 1: break; //可选的停止不支持。忽视。 Optional stop not supported. Ignore.
              default: gc_block.modal.program_flow = int_value; //程序结束并重置  Program end and reset
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
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [不支持的M命令] [Unsupported M command]
        }

        //在当前块中检查每个模态组违规的多个命令 Check for more than one command per modal group violations in the current block
        //注意：如果命令有效，变量'word_bit'总是被赋值。 NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;

      //注意：所有剩余的字母分配值。 NOTE: All remaining letters assign values.
      default:

        /* *非命令字：此初始解析阶段仅检查剩余的重复
           	   合法的g码字，并存储它们的价值。错误检查从一些以后执行
           	   字（I，J，K，L，P，R）具有多个内涵和/或取决于所发布的命令。 */
        switch(letter){
          // case 'A': // 不支持
          // case 'B': // 不支持
          // case 'C': // 不支持
          // case 'D': // 不支持
          case 'F': word_bit = WORD_F; gc_block.values.f = value; break;
          // case 'H': // 不支持
          case 'I': word_bit = WORD_I; gc_block.values.ijk[X_AXIS] = value; ijk_words |= (1<<X_AXIS); break;
          case 'J': word_bit = WORD_J; gc_block.values.ijk[Y_AXIS] = value; ijk_words |= (1<<Y_AXIS); break;
          case 'K': word_bit = WORD_K; gc_block.values.ijk[Z_AXIS] = value; ijk_words |= (1<<Z_AXIS); break;
          case 'L': word_bit = WORD_L; gc_block.values.l = int_value; break;
          case 'N': word_bit = WORD_N; gc_block.values.n = truncf(value); break;
          case 'P': word_bit = WORD_P; gc_block.values.p = value; break;
          //注意：对于某些命令，P值必须是整数，但不支持这些命令。 NOTE: For certain commands, P value must be an integer, but none of these commands are supported.
          // case 'Q': // 不支持
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

        //注意：如果非命令字母有效，则始终分配变量“word_bit”。 NOTE: Variable 'word_bit' is always assigned, if the non-command letter is valid.
        if (bit_istrue(value_words,bit(word_bit))) { FAIL(STATUS_GCODE_WORD_REPEATED); } //[重复单词] [Word repeated]
        //检查单词F，N，P，T和S的无效负值 Check for invalid negative values for words F, N, P, T, and S.
        //注意：为了代码效率，在这里完成负值检查。 NOTE: Negative value check is done here simply for code-efficiency.
        if ( bit(word_bit) & (bit(WORD_F)|bit(WORD_N)|bit(WORD_P)|bit(WORD_T)|bit(WORD_S)) ) {
          if (value < 0.0) { FAIL(STATUS_NEGATIVE_VALUE); } //[字的值不能是负数] [Word value cannot be negative]
        }
        value_words |= bit(word_bit); //标记以指示分配的参数。 Flag to indicate parameter assigned.

    }
  }
  // 解析完成！


  /* -------------------------------------------------------------------------------------
     第3步：错误检查在该块中传递的所有命令和值。这一步确保所有的
     这些命令对于执行是有效的，并且尽可能地遵循NIST标准。
     如果发现错误，则该块中的所有命令和值都将被转储，并且不会更新
     主动系统g代码模式。如果该块没有问题，那么活动的系统g代码模式将会是
     根据这个块的命令进行更新，并发信号给它执行。
     此外，我们必须预先转换所有基于解析设置的模式传递的值
     块。有许多错误检查需要目标信息，只能是
     如果我们将这些值与错误检查一起转换，则可以精确计算出来。
     这将下一个执行步骤作为只更新系统g代码模式和
     按顺序执行编程的动作。执行步骤不应该要求任何
     转换计算，并且只需要执行最少的检查。
  */

  /*
     注意：此时，g代码块已被解析，块行可被释放。
     注意：在未来某个时候，也有可能分解第2步，以允许分段
     在每个单词的基础上解析块，而不是整个块。这可能会删除
     需要为整个块维护一个大的字符串变量并释放一些内存。
     为此，只需要保留STEP 1中的所有数据，例如新块
     数据结构，模态组和值位标记跟踪变量以及轴数组索引
     兼容的变量。这些数据包含了错误检查所需的所有信息
     接收到EOL字符时新的g代码块。但是，这将打破Grbl的创业
     如何工作，并将需要一些重构，使其兼容。
  */

  //[0。非特定/常见错误检查和其他设置]： [0. Non-specific/common error-checks and miscellaneous setup]:

  //确定隐式轴命令条件。轴的话已经通过，但没有明确的轴   Determine implicit axis command conditions. Axis words have been passed, but no explicit axis
  //命令已经发送。如果是，则将轴命令设置为当前运动模式。 command has been sent. If so, set axis command to current motion mode.
  if (axis_words) {
    if (!axis_command) { axis_command = AXIS_COMMAND_MOTION_MODE; } //分配隐式运动模式  Assign implicit motion-mode
  }

  //检查有效的行号N值。  Check for valid line number N value.
  if (bit_istrue(value_words,bit(WORD_N))) {
    //行号值不能大于最大的行号小于零（完成）或更大。 Line number value cannot be less than zero (done) or greater than max line number.
    if (gc_block.values.n > MAX_LINE_NUMBER) { FAIL(STATUS_GCODE_INVALID_LINE_NUMBER); } // [Exceeds max line number]
  }
  // bit_false(value_words,bit(WORD_N)); // NOTE: Single-meaning value word. Set at end of error-checking.

  //在错误检查结束时跟踪未使用的单词。
  //注意：在错误检查结束时，单值含义的字被一次删除，因为
  //他们总是在场时使用。这样做是为了节省几个字节的闪存。为了清楚起见，
  //使用单值含义的单词可能会被删除。另外，轴词是在处理
  //同样的方式。如果有明确的/隐含的轴命令，XYZ字总是被使用并且是
  //在错误检查结束时被删除。

  //[1。评论]：味精不支持。协议执行的注释处理。 [1. Comments ]: MSG's NOT SUPPORTED. Comment handling performed by protocol.

  //[2。设置进给速率模式]：G1，G2 / 3激活时，G93 F字缺失，隐式或显式。进给率 [2. Set feed rate mode ]: G93 F word missing with G1,G2/3 active, implicitly or explicitly. Feed rate
  //从G93切换到G94后没有定义。   is not defined after switching to G94 from G93.
  //注意：对于慢跑，忽略之前的进给速率模式。强制执行G94并检查所需的F字。 NOTE: For jogging, ignore prior feed rate mode. Enforce G94 and check for required F word.
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) {
    if (bit_isfalse(value_words,bit(WORD_F))) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); }
    if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.f *= MM_PER_INCH; }
  } else {
    if (gc_block.modal.feed_rate == FEED_RATE_MODE_INVERSE_TIME) { // = G93
      //注意：G38也可以反时间运行，但是没有定义为错误。在这里添加缺少F字检查。 NOTE: G38 can also operate in inverse time, but is undefined as an error. Missing F word check added here.
      if (axis_command == AXIS_COMMAND_MOTION_MODE) {
		if ((gc_block.modal.motion != MOTION_MODE_NONE) && (gc_block.modal.motion != MOTION_MODE_SEEK)) {
          if (bit_isfalse(value_words,bit(WORD_F))) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); } // [F word missing]
        }
      }
      //注意：检查从G94切换到G93后要传递的F字似乎是多余的。我们会
      //如果进给速率值总是重置为零，并且在每个之后都未定义，则完成同样的事情
      //反时间块，因为使用这个值的命令已经执行了未定义的检查。这个会
      //也允许其他命令，在这个开关之后执行，而不是不必要地出错。这个代码是
      //结合上述进给速率模式和下面设定的进给速度错误检查。

      // [3。设定进给速率]：F为负值（完成）
      // - 反时限模式：总是在块完成之前和之后隐式地将进给速率值清零。
      //注意：如果在G93模式下或者从G94模式切换到G99，只需将F值保持为初始化零或传递F字
      //在块中的值。如果没有F字通过需要进给速度的运动命令传递，这将会出错
      //在运动模式下进行错误检查。但是，如果没有F字是通过NO动作命令需要的
      //进给速度，我们简单地继续，状态进给速率值更新为零，并保持未定义。
    } else { // = G94
      // - 以毫米单位为单位：如果F字通过，确保值以毫米/分钟为单位，否则按最后一个状态值。
      if (gc_state.modal.feed_rate == FEED_RATE_MODE_UNITS_PER_MIN) { //最后状态也G94 Last state is also G94
        if (bit_istrue(value_words,bit(WORD_F))) {
          if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.f *= MM_PER_INCH; }
        } else {
          gc_block.values.f = gc_state.feed_rate; //推动最后的状态进给速率 Push last state feed rate
        }
      } //否则，从G93切换到G94，所以不要推最后的状态进给速率。它的未定义或传递的F字值。 Else, switching to G94 from G93, so don't push last state feed rate. Its undefined or the passed F word value.
    }
  }
  // bit_false(value_words,bit(WORD_F)); // NOTE: Single-meaning value word. Set at end of error-checking.

  //[4。设置主轴转速]：S为负值（完成） [4. Set spindle speed ]: S is negative (done.)
  if (bit_isfalse(value_words,bit(WORD_S))) { gc_block.values.s = gc_state.spindle_speed; }
  // bit_false(value_words,bit(WORD_S)); // NOTE: Single-meaning value word. Set at end of error-checking.

  //[5。选择工具]：不支持。只跟踪价值。T是否定的（完成）不是一个整数。大于最大工具值。 [5. Select tool ]: NOT SUPPORTED. Only tracks value. T is negative (done.) Not an integer. Greater than max tool value.
  // bit_false(value_words,bit(WORD_T)); // NOTE: Single-meaning value word. Set at end of error-checking.

  // [6. 换刀 ]: N/A
  // [7. 主轴控制 ]: N/A
  // [8. 冷却液控制 ]: N/A
	// [9. 覆盖控制 ]: 除Grbl专用的停车运动覆盖控制外，不支持。
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
	if (bit_istrue(command_words, bit(MODAL_GROUP_M9))) { // Already set as enabled in parser.
		if (bit_istrue(value_words, bit(WORD_P))) {
			if (gc_block.values.p == 0.0f) { gc_block.modal.override = OVERRIDE_DISABLED; }
			bit_false(value_words, bit(WORD_P));
		}
	}
#endif
	
  // [10. 暂停 ]: 缺少P值。P是否定的（完成）注：见下文。
  if (gc_block.non_modal_command == NON_MODAL_DWELL) {
    if (bit_isfalse(value_words,bit(WORD_P))) { FAIL(STATUS_GCODE_VALUE_WORD_MISSING); } // [P字丢失]
    bit_false(value_words,bit(WORD_P));
  }

  // [11。设置活动平面]：N / A
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

  // [12. 设置长度单位 ]: N/A
  // 将XYZ坐标值预先转换为毫米（如果适用）。
  uint8_t idx;
  if (gc_block.modal.units == UNITS_MODE_INCHES) {
    for (idx=0; idx<N_AXIS; idx++) { // 轴的索引是一致的，所以可以使用循环。
      if (bit_istrue(axis_words,bit(idx)) ) {
        gc_block.values.xyz[idx] *= MM_PER_INCH;
      }
    }
  }

  // [13。刀具半径补偿]：G41 / 42不支持。错误，如果在G53有效时启用。
  // [G40错误]：在G40之后编程G2 / 3弧。禁用后的直线运动小于刀具直径。
  //    注意：由于刀具半径补偿从未启用，所以这些G40错误不适用。Grbl支持G40
  //    仅用于G40与g代码程序头一起发送以设置默认模式时不出错的目的。

  // [14。切刀长度补偿]：G43不支持，但是G43.1和G49是。
  // [G43.1错误]：同一行中的运动命令。
  //    注意：尽管没有明确说明，G43.1应该只应用于一个有效的
  //    配置的轴（在config.h中）。如果配置的轴应该有错误
  //    不存在或者是否存在任何其他的轴单词。
  if (axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { //表示在块中调用。 Indicates called in block.
    if (gc_block.modal.tool_length == TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC) {
      if (axis_words ^ (1<<TOOL_LENGTH_OFFSET_AXIS)) { FAIL(STATUS_GCODE_G43_DYNAMIC_AXIS_ERROR); }
    }
  }

  // [15。坐标系选择]：* N / A。错误，如果刀具半径补偿激活。
  // TODO：读取坐标数据的EEPROM在循环时可能需要缓冲区同步
  //处于活动状态 读暂时暂停处理器，可能会导致罕见的崩溃。对于
  //有足够内存的处理器的未来版本，所有的坐标数据应该被存储
  //存储器中，并且只有当没有周期激活时才写入EEPROM。
  float block_coord_system[N_AXIS];
  memcpy(block_coord_system,gc_state.coord_system,sizeof(gc_state.coord_system));
  if ( bit_istrue(command_words,bit(MODAL_GROUP_G12)) ) { // 检查是否在块中调用
    if (gc_block.modal.coord_select > N_COORDINATE_SYSTEM) { FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS); } //[大于N sys] [Greater than N sys]
    if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
      if (!(settings_read_coord_data(gc_block.modal.coord_select,block_coord_system))) { FAIL(STATUS_SETTING_READ_FAIL); }
    }
  }

  // [16。设置路径控制模式]：N / A。只有G61。G61.1和G64不支持。
  // [17。设置距离模式]：N / A。只有G91.1。G90.1不支持。
  // [18。设置缩进模式]：不支持。

  // [19。剩余非模态动作]：勾选转到预定位置，设置G10或设置轴偏置。
  //注意：我们需要将使用轴名称（G10 / G28 / G30 / G92）的非模态命令分开，
  //命令都以不同的方式处理轴单词。G10作为绝对偏移或计算当前位置
  //轴值，G92与G10 L20，G28 / 30一样，作为观察的中间目标位置
  //所有当前坐标系和G92偏移量。
  switch (gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:
        // [G10错误]：L丢失，不是2或20. P字丢失。（负P值已完成）
        // [G10 L2错误]：R字不支持。P值不是0到nCoordSys（最多9）。轴字不见了
        // [G10 L20 Errors]：P必须是0到nCoordSys（最大9）。轴字不见了
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

      //确定坐标系改变并尝试从EEPROM加载。
      if (coord_select > 0) { coord_select--; } // 将 P1-P6索引调整为EEPROM坐标数据索引。
      else { coord_select = gc_block.modal.coord_select; } // 将 P0指定为活动坐标系
      
      // 注意：将参数数据存储在IJK值中。按规则，这个命令没有被使用。
      if (!settings_read_coord_data(coord_select,gc_block.values.ijk)) { FAIL(STATUS_SETTING_READ_FAIL); } // [EEPROM read fail]

      //预先计算坐标数据的变化。
      for (idx=0; idx<N_AXIS; idx++) { // 轴的索引是一致的，所以可以使用循环。
        // 更新仅在块中定义的轴。始终在机器坐标中。可以改变非主动系统。
        if (bit_istrue(axis_words,bit(idx)) ) {
          if (gc_block.values.l == 20) {
            // L20：使用编程值更新当前位置（含修改器）的坐标系轴
            // WPos = MPos - WCS - G92 - TLO  ->  WCS = MPos - G92 - TLO - WPos
            gc_block.values.ijk[idx] = gc_state.position[idx]-gc_state.coord_offset[idx]-gc_block.values.xyz[idx];
            if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.ijk[idx] -= gc_state.tool_length_offset; }
          } else {
            // L2：将坐标系轴更新为编程值。
            gc_block.values.ijk[idx] = gc_block.values.xyz[idx];
          }
        } // 否则，保持当前的存储值。
      }
      break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
      // [G92错误]：无轴字。
      if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [无轴字]

      //更新仅在块中定义的轴。将当前系统偏移到定义的值。不更新时
      //选择激活的坐标系，但是除非G92.1禁用它，否则它仍然是活动的。
      for (idx=0; idx<N_AXIS; idx++) { // 轴的索引是一致的，所以可以使用循环。
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

        //此时，显式轴命令的其余部分将轴值视为传统值
        //以坐标系偏移，G92偏移，绝对覆盖和距离为目标位置
        //应用的模式。这包括运动模式命令。我们现在可以预先计算目标位置。
        //注意：如果添加了此功能，则可以在这些转换中附加刀具补偿。
      if (axis_command != AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // TLO阻止任何轴命令。
        if (axis_words) {
          for (idx=0; idx<N_AXIS; idx++) { // 轴索引是一致的，所以循环可以用来保存闪存空间。.
            if ( bit_isfalse(axis_words,bit(idx)) ) {
              gc_block.values.xyz[idx] = gc_state.position[idx]; //块中没有轴的单词。保持相同的轴位置。 No axis word in block. Keep same axis position.
            } else {
                //根据距离模式更新指定值，或者在绝对覆盖激活时忽略。
                //注意：由于G28 / 30在同一个模态组中，G53不会处于激活状态。
              if (gc_block.non_modal_command != NON_MODAL_ABSOLUTE_OVERRIDE) {
                // 根据距离模式应用坐标偏移量。
                if (gc_block.modal.distance == DISTANCE_MODE_ABSOLUTE) {
                  gc_block.values.xyz[idx] += block_coord_system[idx] + gc_state.coord_offset[idx];
                  if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.xyz[idx] += gc_state.tool_length_offset; }
                } else {  // 增量模式
                  gc_block.values.xyz[idx] += gc_state.position[idx];
                }
              }
            }
          }
        }
      }

      // 检查剩余的非模态命令是否有错误。
      switch (gc_block.non_modal_command) {
        case NON_MODAL_GO_HOME_0: // G28
        case NON_MODAL_GO_HOME_1: // G30
            // [G28 / 30错误]：刀具补偿已启用。
            //从EEPROM读取G28 / 30回原点位置数据（在机器坐标中）
            //注意：将参数数据存储在IJK值中。按规则，这个命令没有被使用。
          if (gc_block.non_modal_command == NON_MODAL_GO_HOME_0) {
            if (!settings_read_coord_data(SETTING_INDEX_G28,gc_block.values.ijk)) { FAIL(STATUS_SETTING_READ_FAIL); }
          } else { // == NON_MODAL_GO_HOME_1
            if (!settings_read_coord_data(SETTING_INDEX_G30,gc_block.values.ijk)) { FAIL(STATUS_SETTING_READ_FAIL); }
          }
          if (axis_words) {
            //只移动在二次移动中指定的轴。 Move only the axes specified in secondary move.
            for (idx=0; idx<N_AXIS; idx++) {
              if (!(axis_words & (1<<idx))) { gc_block.values.ijk[idx] = gc_state.position[idx]; }
            }
          } else {
            axis_command = AXIS_COMMAND_NONE; //如果没有中间运动，则设为无。 Set to none if no intermediate motion.
          }
          break;
        case NON_MODAL_SET_HOME_0: // G28.1
        case NON_MODAL_SET_HOME_1: // G30.1
            // [G28.1 / 30.1错误]：刀具补偿启用。
            //注意：如果在这里传递轴话，它们被解释为隐式运动模式。
          break;
        case NON_MODAL_RESET_COORDINATE_OFFSET:
          //注意：如果在这里传递轴话，它们被解释为隐式运动模式。 NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
          break;
        case NON_MODAL_ABSOLUTE_OVERRIDE:
            // [G53错误]：G0和G1没有激活。刀具补偿已启用。
            //注意：所有显式的轴字命令都在这个模态组中。所以不需要隐式检查。
          if (!(gc_block.modal.motion == MOTION_MODE_SEEK || gc_block.modal.motion == MOTION_MODE_LINEAR)) {
            FAIL(STATUS_GCODE_G53_INVALID_MOTION_MODE); // [G53 G0/1 无效]
          }
          break;
      }
  }

  // [20. 运动模式 ]:
  if (gc_block.modal.motion == MOTION_MODE_NONE) {
	    // [G80错误]：当G80有效时，轴字被编程。
	    //注意：即使非模态命令或使用轴词语的TLO也会抛出这个严格的错误。
    if (axis_words) { FAIL(STATUS_GCODE_AXIS_WORDS_EXIST); } // [No axis words allowed]

    //检查剩余的运动模式，如果轴字是隐含的（G10 / 28/30/92中已经存在和未使用）或者
    //在g代码块中明确地命令。
  } else if ( axis_command == AXIS_COMMAND_MOTION_MODE ) {

    if (gc_block.modal.motion == MOTION_MODE_SEEK) {
        // [G0错误]：轴信号没有配置或没有实际值（完成）
        //轴词是可选的。如果缺少，则设置轴命令标志忽略执行。
      if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

      //所有剩余的运动模式（除G0和G80外）都需要有效的进给速率值。在每毫米单位模式下，
      //值必须是正值。在反时限模式下，每个块必须传递一个正值。
    } else {
      //检查是否为需要的运动模式定义了进给速率。 Check if feed rate is defined for the motion modes that require it.
      if (gc_block.values.f == 0.0f) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); } //[进给率未定义] [Feed rate undefined]

      switch (gc_block.modal.motion) {
        case MOTION_MODE_LINEAR:
            // [G1错误]：进给率未定义。轴信件没有配置或没有实际价值。
            //轴词是可选的。如果缺少，则设置轴命令标志忽略执行。
          if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

          break;
        case MOTION_MODE_CW_ARC: 
          gc_parser_flags |= GC_PARSER_ARC_IS_CLOCKWISE; //没有打算故意 No break intentional.
        case MOTION_MODE_CCW_ARC:
            // [G2 / 3 Errors All-Modes]：进给速率未定义。
            // [G2 / 3半径模式错误]：在选定平面中没有轴线字。目标点与当前相同。
            // [G2 / 3偏移模式错误]：在所选平面中没有轴的字和/或偏移。到当前的半径
            //    点和目标点的半径差异超过0.002mm（EMC高度0.5mm或0.005mm和0.1％半径）。
            // [G2 / 3全环模式错误]：不支持。轴的话存在。没有偏移编程。P必须是一个整数。
            //注意：圆弧跟踪所需的半径和偏移量都是错误检查的预先计算。

          if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
          if (!(axis_words & (bit(axis_0)|bit(axis_1)))) { FAIL(STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE); } //[平面中没有轴的单词] [No axis words in plane]

          //计算沿每个选定轴的位置变化 Calculate the change in position along each selected axis
          float x,y;
          x = gc_block.values.xyz[axis_0]-gc_state.position[axis_0]; // 三角洲 x 当前位置和目标之间
          y = gc_block.values.xyz[axis_1]-gc_state.position[axis_1]; // 三角洲 y 当前位置和目标之间

          if (value_words & bit(WORD_R)) { // 圆弧半径模式
            bit_false(value_words,bit(WORD_R));
            if (isequal_position_vector(gc_state.position, gc_block.values.xyz)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [目标无效]

            // 将半径值转换为适当的单位。
            if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.r *= MM_PER_INCH; }
            /*  我们需要计算具有指定半径并通过的圆的中心
                通过当前位置和目标位置。该方法计算以下内容
                方程组，其中[x，y]是从当前位置到目标位置的矢量，d ==幅度
                那个矢量，h ==斜边所形成的三角形的圆心，距离为
                旅行向量的中心。与矢量垂直的矢量[-y，x]被缩放到
                长度为h [-y / d * h，x / d * h]并添加到行程向量[x / 2，y / 2]的中心，形成新的点
                [i，j]在[x / 2-y / d * h，y / 2 + x / d * h]，这将是我们弧的中心。

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

                C - 当前位置
                T - 目标位置
                O - 通过C和T的圆的中心
                d - 从C到T的距离
                r - 指定的半径
                h - 从CT中心到O的距离

                                                扩展方程：

                d -> sqrt(x^2 + y^2)
                h -> sqrt(4 * r^2 - x^2 - y^2)/2
                i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
                j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

                                                可以写成:

                i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
                j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

                                                 我们的尺寸和速度的原因，以优化:

                h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
                i = (x - (y * h_x2_div_d))/2
                j = (y + (x * h_x2_div_d))/2
            */

            //首先，使用h_x2_div_d来计算4 * h ^ 2来检查它是否为负数或r是小于
            //比d 如果是这样，负数的sqrt是复杂的，错误的。
            float h_x2_div_d = 4.0f * gc_block.values.r*gc_block.values.r - x*x - y*y;

            if (h_x2_div_d < 0) { FAIL(STATUS_GCODE_ARC_RADIUS_ERROR); } // [弧半径误差]

            // 完成计算 h_x2_div_d.
            h_x2_div_d = -sqrtf(h_x2_div_d)/hypot_f(x,y); // == -(h * 2 / d)
            // 反转h_x2_div_d的符号（如下图所示）
            if (gc_block.modal.motion == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d; }

            /* 逆时针圆圈位于目标方向的左侧。当抵消是积极的，
               会产生左边的圆圈 - 当它是负值时，产生右边的圆圈。

                                                                   T  <-- 目标位置

                                                                   ^
                                                                                                               以此中心顺时针圆圈 这个中心的顺时针圆将有         |          Clockwise circles with this center will have
                                                                                       将具有> 180度的角行程 角行程<180度，这是一件好事！      |          < 180 deg of angular travel, which is a good thing!
                                                         \         |          /
                                                                                           当h_x2_div_d是正值时，弧的圆心 ->  x <----- | -----> x <- 当h_x2_div_d为负数时弧的中心
                                                                   |
                                                                   |

                                                                   C  <-- 当前位置
            */
            // Negative R是g代码 - “我想要一个超过180度的行程的圆”（去图！），
            //即使建议不要在一行g代码中生成这样的圆圈。通过
            //反转h_x2_div_d的符号，圆的中心位于
            //旅行，因此我们得到了不可预见的长弧。
            if (gc_block.values.r < 0) {
                h_x2_div_d = -h_x2_div_d;
                gc_block.values.r = -gc_block.values.r; //完成r。设置为mc_arc为正值 Finished with r. Set to positive for mc_arc
            }
            // 通过计算圆弧的实际中心来完成操作
            gc_block.values.ijk[axis_0] = 0.5f*(x-(y*h_x2_div_d));
            gc_block.values.ijk[axis_1] = 0.5f*(y+(x*h_x2_div_d));

          } else { // 弧中心格式偏置模式
            if (!(ijk_words & (bit(axis_0)|bit(axis_1)))) { FAIL(STATUS_GCODE_NO_OFFSETS_IN_PLANE); } // [No offsets in plane]
            bit_false(value_words,(bit(WORD_I)|bit(WORD_J)|bit(WORD_K)));

            // 将IJK值转换为适当的单位.
            if (gc_block.modal.units == UNITS_MODE_INCHES) {
              for (idx=0; idx<N_AXIS; idx++) { //轴索引是一致的，所以循环可以用来保存闪存空间 Axes indices are consistent, so loop may be used to save flash space.
                if (ijk_words & bit(idx)) { gc_block.values.ijk[idx] *= MM_PER_INCH; }
              }
            }

            // 从中心到目标的圆弧半径
            x -= gc_block.values.ijk[axis_0]; // 三角洲 x 当前位置和目标之间
            y -= gc_block.values.ijk[axis_1]; // 三角洲 y 当前位置和目标之间
            float target_r = hypot_f(x,y);

            //计算mc_arc的弧半径。从当前位置定义到中心。 Compute arc radius for mc_arc. Defined from current location to center.
            gc_block.values.r = hypot_f(gc_block.values.ijk[axis_0], gc_block.values.ijk[axis_1]);

            //计算当前位置和目标半径之间的差异以进行最终的错误检查。 Compute difference between current location and target radii for final error-checks.
            float delta_r = fabsf(target_r-gc_block.values.r);
            if (delta_r > 0.005f) {
              if (delta_r > 0.5f) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [弧线定义误差] > 0.5mm
              if (delta_r > (0.001f*gc_block.values.r)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [圆弧定义误差] > 0.005mm AND 半径为0.1％
            }
          }
          break;
        case MOTION_MODE_PROBE_TOWARD_NO_ERROR: case MOTION_MODE_PROBE_AWAY_NO_ERROR:
            gc_parser_flags |= GC_PARSER_PROBE_IS_NO_ERROR; //没有打算故意 No break intentional.
        case MOTION_MODE_PROBE_TOWARD: case MOTION_MODE_PROBE_AWAY:
            if ((gc_block.modal.motion == MOTION_MODE_PROBE_AWAY) ||
                (gc_block.modal.motion == MOTION_MODE_PROBE_AWAY_NO_ERROR)) { gc_parser_flags |= GC_PARSER_PROBE_IS_AWAY; }
            // [G38错误]：目标是相同的电流。没有轴的话。刀具补偿已启用。进给率
            //    是未定义的 探测器被触发。注意：探针检查已移至探针循环。而不是返回
            //    发生错误，它会发出警报以防止进一步移动到探针。这也是在那里完成的
            //    在另一个探测循环之前，允许规划器缓冲区清空并移出探测器触发器。
          if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
          if (isequal_position_vector(gc_state.position, gc_block.values.xyz)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Invalid target]
          break;
      }
    }
  }

  // [21。程序流程]：不需要进行错误检查。

  // [0。非特定错误检查]：完成未使用的值字检查，即IJK在弧中使用
  // 半径模式，或在块中未使用的轴单词。
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) {
      // 慢跑只使用F进给速率和XYZ值字。N有效，但S和T无效。
      bit_false(value_words, (bit(WORD_N) | bit(WORD_F)));
  } else {
      bit_false(value_words, (bit(WORD_N) | bit(WORD_F) | bit(WORD_S) | bit(WORD_T))); // Remove single-meaning value words.
  }
  if (axis_command) { bit_false(value_words,(bit(WORD_X)|bit(WORD_Y)|bit(WORD_Z))); } // Remove axis words.
  if (value_words) { FAIL(STATUS_GCODE_UNUSED_WORDS); } // [Unused words]

  /* -------------------------------------------------------------------------------------
     第4步：执行！
     假设所有错误检查已经完成，并且不存在失败模式。我们刚刚
     需要根据执行顺序更新状态并执行该块。
  */

  // 初始化运动块的规划器数据结构。
  plan_line_data_t plan_data;
  plan_line_data_t *pl_data = &plan_data;
  memset(pl_data,0,sizeof(plan_line_data_t)); // 零pl_data结构

  //截取点动命令并完成有效点动命令的错误检查并执行。
  //注意：G代码解析器状态不会更新，除了保证连续点动的位置外
  //目标计算正确。点动后的最后解析器位置在更新中
  //当慢跑完成或被取消时，protocol_execute_realtime（）。
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) {
      //只允许使用距离单位模态指令和G53绝对覆盖指令。
      //注意：在步骤3中已经执行了进给率字和轴字的检查。
      if (command_words & ~(bit(MODAL_GROUP_G3) | bit(MODAL_GROUP_G6 | bit(MODAL_GROUP_G0)))) { FAIL(STATUS_INVALID_JOG_COMMAND) };
      if (!(gc_block.non_modal_command == NON_MODAL_ABSOLUTE_OVERRIDE || gc_block.non_modal_command == NON_MODAL_NO_ACTION)) { FAIL(STATUS_INVALID_JOG_COMMAND); }

      // 将计划器数据初始化为当前的主轴和冷却剂模态。
      pl_data->spindle_speed = gc_state.spindle_speed;
      plan_data.condition = (gc_state.modal.spindle | gc_state.modal.coolant);

      uint8_t status = jog_execute(&plan_data, &gc_block);
      if (status == STATUS_OK) { memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz)); }
      return(status);
  }

  // 如果在激光模式下，根据当前和过去的分析器条件设置激光功率。
  if (bit_istrue(settings.flags, BITFLAG_LASER_MODE)) {
      if (!((gc_block.modal.motion == MOTION_MODE_LINEAR) || (gc_block.modal.motion == MOTION_MODE_CW_ARC)
          || (gc_block.modal.motion == MOTION_MODE_CCW_ARC))) {
          gc_parser_flags |= GC_PARSER_LASER_DISABLE;
      }

      //任何带有轴字的运动模式都允许从主轴速度更新中传递。
      //注意：G1和G0没有轴的话将axis_command设置为none。G28 / 30故意省略。
      // TODO：检查没有进入规划器的启用M3的动作的同步条件。（零长度）。
      if (axis_words && (axis_command == AXIS_COMMAND_MOTION_MODE)) {
        gc_parser_flags |= GC_PARSER_LASER_ISMOTION;
      }
      else {
        //  M3恒功率激光器需要计划者同步来更新激光器之间的变化
        // G1 / 2/3运动模式状态，反之亦然。
        if (gc_state.modal.spindle == SPINDLE_ENABLE_CW) {
          if ((gc_state.modal.motion == MOTION_MODE_LINEAR) || (gc_state.modal.motion == MOTION_MODE_CW_ARC)
            || (gc_state.modal.motion == MOTION_MODE_CCW_ARC)) {
            if (bit_istrue(gc_parser_flags, GC_PARSER_LASER_DISABLE)) {
              gc_parser_flags |= GC_PARSER_LASER_FORCE_SYNC; //从G1/2/3运动模式切换 Change from G1/2/3 motion mode.
            }
          }
          else {
            //从非G1 / 2/3运动模式切换到G1运动模式时，无轴话语。 When changing to a G1 motion mode without axis words from a non-G1/2/3 motion mode.
            if (bit_isfalse(gc_parser_flags, GC_PARSER_LASER_DISABLE)) {
              gc_parser_flags |= GC_PARSER_LASER_FORCE_SYNC;
            }
          }
        }
      }
  }

  // [0。非特定/常见错误检查和其他设置]：
  //注意：如果不存在行号，则该值为零。
  gc_state.line_number = gc_block.values.n;
  #ifdef USE_LINE_NUMBERS
    pl_data->line_number = gc_state.line_number; // 记录规划人员使用的数据。
  #endif

  // [1。意见反馈]：不支持

  // [2。设置进给速率模式]：
  gc_state.modal.feed_rate = gc_block.modal.feed_rate;
  if (gc_state.modal.feed_rate) { pl_data->condition |= PL_COND_FLAG_INVERSE_TIME; } // 设置规划器使用的条件标志。

  // [3. 设定进给速率 ]:
  gc_state.feed_rate = gc_block.values.f; //总是复制这个值。查看进给率错误检查 Always copy this value. See feed rate error-checking.
  pl_data->feed_rate = gc_state.feed_rate; //记录规划人员使用的数据 Record data for planner use.

  // [4. 设置主轴转速 ]:
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
      gc_state.spindle_speed = gc_block.values.s; // 更新主轴转速状态
  }
  // 注意：所有受限激光运动都通过零主轴转速.
  if (bit_isfalse(gc_parser_flags, GC_PARSER_LASER_DISABLE)) {
      pl_data->spindle_speed = gc_state.spindle_speed; // 记录规划人员使用的数据.
  } // else { pl_data->spindle_speed = 0.0; } // 已经初始化为零.

  // [5. 选择工具 ]: 不支持。只跟踪工具值.
  gc_state.tool = gc_block.values.t;

  // [6. 换刀 ]: 不支持

  // [7. 主轴控制 ]:
  if (gc_state.modal.spindle != gc_block.modal.spindle) {
	    //更新主轴控制并在此模块中启用时应用主轴转速。
	    //注意：即使在激光模式下，所有的主轴状态改变也是同步的。另外，pl_data，
	    //而不是gc_state，用于管理非激光运动的激光状态。
    spindle_sync(gc_block.modal.spindle, pl_data->spindle_speed);
    gc_state.modal.spindle = gc_block.modal.spindle;
  }
  pl_data->condition |= gc_state.modal.spindle; // Set condition flag for planner use.

  // [8. 冷却液控制 ]:
  if (gc_state.modal.coolant != gc_block.modal.coolant) {
	    //注意：冷却剂M代码是模态的。每行只允许一个命令。但是，多个国家
	    //可以同时存在，而冷却液禁用清除所有状态。
    coolant_sync(gc_block.modal.coolant);
    if (gc_block.modal.coolant == COOLANT_DISABLE) { gc_state.modal.coolant = COOLANT_DISABLE; }
    else { gc_state.modal.coolant |= gc_block.modal.coolant; }
  }
  pl_data->condition |= gc_state.modal.coolant; //设置规划器使用的条件标志 Set condition flag for planner use.

	// [9。覆盖控制]：不支持。始终启用。除了Grbl专用停车控制。
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
	if (gc_state.modal.override != gc_block.modal.override) {
		gc_state.modal.override = gc_block.modal.override;
		mc_override_ctrl_update(gc_state.modal.override);
	}
#endif

  // [10. 暂停 ]:
  if (gc_block.non_modal_command == NON_MODAL_DWELL) { mc_dwell(gc_block.values.p); }

  // [11. 设置活动平面 ]:
  gc_state.modal.plane_select = gc_block.modal.plane_select;

  // [12. 设置长度单位 ]:
  gc_state.modal.units = gc_block.modal.units;

  // [13. 刀具半径补偿 ]: G41/42 不支持
  // gc_state.modal.cutter_comp = gc_block.modal.cutter_comp; //注意：不需要，因为一直禁用 NOTE: Not needed since always disabled.

  // [14. 刀长补偿 ]: 支持G43.1和G49。G43不支持。.
  //注意：如果支持G43，其运行方式与G43.1没有任何区别
  //执行 错误检查步骤将简单地将偏移值加载到正确的位置
  //块XYZ值数组的轴。
  if (axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // 指示更改。
    gc_state.modal.tool_length = gc_block.modal.tool_length;
    if (gc_state.modal.tool_length == TOOL_LENGTH_OFFSET_CANCEL) { // G49
      gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS] = 0.0f;
    } // else G43.1
    if ( gc_state.tool_length_offset != gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS] ) {
      gc_state.tool_length_offset = gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS];
      system_flag_wco_change();
    }
  }

  // [15. 坐标系选择 ]:
  if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
    gc_state.modal.coord_select = gc_block.modal.coord_select;
    memcpy(gc_state.coord_system,block_coord_system,N_AXIS*sizeof(float));
    system_flag_wco_change();
  }

  // [16. 设置路径控制模式 ]: G61.1/G64 不支持
  // gc_state.modal.control = gc_block.modal.control; // 注意：始终默认.

  // [17. 设置距离模式 ]:
  gc_state.modal.distance = gc_block.modal.distance;

  // [18. 设置缩进模式 ]: 不支持

  // [19. 转到预定义的位置，设置G10或设置轴偏置 ]:
  switch(gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:
      settings_write_coord_data(coord_select,gc_block.values.ijk);
      // 如果当前有效，更新系统坐标系.
      if (gc_state.modal.coord_select == coord_select) {
        memcpy(gc_state.coord_system,gc_block.values.ijk,N_AXIS*sizeof(float));
        system_flag_wco_change();
      }
      break;
    case NON_MODAL_GO_HOME_0: case NON_MODAL_GO_HOME_1:
      //回家之前移到中间位置 遵守当前的坐标系和偏移   Move to intermediate position before going home. Obeys current coordinate system and offsets
      //和绝对和增量模式。 and absolute and incremental modes.
      pl_data->condition |= PL_COND_FLAG_RAPID_MOTION; // 设置快速运动条件标志。
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
      clear_vector(gc_state.coord_offset); //通过归零偏移矢量来禁用G92偏移。 Disable G92 offsets by zeroing offset vector.
      system_flag_wco_change();
      break;
  }


  // [20. 运动模式 ]:
  // 注意：命令G10，G28，G30，G92锁定并防止轴在运动模式下使用。
  // 只有在程序段中有轴字或运动模式命令字的情况下才能进入运动模式。
  gc_state.modal.motion = gc_block.modal.motion;
  if (gc_state.modal.motion != MOTION_MODE_NONE) {
    if (axis_command == AXIS_COMMAND_MOTION_MODE) {
      uint8_t gc_update_pos = GC_UPDATE_POS_TARGET;
      if (gc_state.modal.motion == MOTION_MODE_LINEAR) {
        mc_line(gc_block.values.xyz, pl_data);
      } else if (gc_state.modal.motion == MOTION_MODE_SEEK) {
        pl_data->condition |= PL_COND_FLAG_RAPID_MOTION; // 设置快速运动条件标志。
        mc_line(gc_block.values.xyz, pl_data);
      } else if ((gc_state.modal.motion == MOTION_MODE_CW_ARC) || (gc_state.modal.motion == MOTION_MODE_CCW_ARC)) {
          mc_arc(gc_block.values.xyz, pl_data, gc_state.position, gc_block.values.ijk, gc_block.values.r,
              axis_0, axis_1, axis_linear, bit_istrue(gc_parser_flags, GC_PARSER_ARC_IS_CLOCKWISE));
      } else {
        // 注意：gc_block.values.xyz是从mc_probe_cycle返回更新后的位置值。所以
        // 在成功的探测循环中，机床位置和返回值应该是相同的。
        #ifndef ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES
          pl_data->condition |= PL_COND_FLAG_NO_FEED_OVERRIDE;
        #endif
        gc_update_pos = mc_probe_cycle(gc_block.values.xyz, pl_data, gc_parser_flags);
    }  
     
      //就解析器而言，位置现在是==目标。在现实中
      //运动控制系统可能仍然在处理动作和真实的工具位置
      //在任何中间位置
      if (gc_update_pos == GC_UPDATE_POS_TARGET) {
        memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz)); // gc_state.position[] = gc_block.values.xyz[]
      } else if (gc_update_pos == GC_UPDATE_POS_SYSTEM) {
        gc_sync_position(); // gc_state.position[] = sys_position
      } // == GC_UPDATE_POS_NONE
    }     

  }

  // [21。程序流程]：
  // M0，M1，M2，M30：执行非运行的程序流动作。在程序暂停期间，缓冲区可以
  //重新填充，只能由cycle start run-time命令恢复。
  gc_state.modal.program_flow = gc_block.modal.program_flow;
  if (gc_state.modal.program_flow) {
    protocol_buffer_synchronize(); // 继续前进行同步并完成所有剩余的缓冲运动。
    if (gc_state.modal.program_flow == PROGRAM_FLOW_PAUSED) {
      if (sys.state != STATE_CHECK_MODE) {
        system_set_exec_state_flag(EXEC_FEED_HOLD); // 使用feed hold暂停程序。
        protocol_execute_realtime(); // 执行挂起。
      }
    } else { // == PROGRAM_FLOW_COMPLETED
        //完成程序后，根据gid，只有一部分g代码重置为某些默认值
        // LinuxCNC的程序结束描述和测试。只有模态组[G代码1,2,3,5,7,12]
        // [M代码7,8,9]复位为[G1，G17，G90，G94，G40，G54，M5，M9，M48]。其余的模态组
        // [G代码4,6,8,10,13,14,15]和[M代码4,5,6]和模态字[F，S，T，H]不会重置。
      gc_state.modal.motion = MOTION_MODE_LINEAR;
      gc_state.modal.plane_select = PLANE_SELECT_XY;
      gc_state.modal.distance = DISTANCE_MODE_ABSOLUTE;
      gc_state.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
      // gc_state.modal.cutter_comp = CUTTER_COMP_DISABLE; // 不支持。
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

      // 执行坐标变换和主轴/冷却液停止。
      if (sys.state != STATE_CHECK_MODE) {
        if (!(settings_read_coord_data(gc_state.modal.coord_select,gc_state.coord_system))) { FAIL(STATUS_SETTING_READ_FAIL); }
        system_flag_wco_change(); // 设置为立即刷新，以防万一有所改变。
        spindle_set_state(SPINDLE_DISABLE,0.0f);
        coolant_set_state(COOLANT_DISABLE);
      }
      report_feedback_message(MESSAGE_PROGRAM_END);
    }
    gc_state.modal.program_flow = PROGRAM_FLOW_RUNNING; // 重置程序流程
  }

  // TODO: % 表示程序的开始.

  return(STATUS_OK);
}


/*
  不支持：
  - 固定循环
  - 刀具半径补偿
  -  A，B，C轴
  - 表达的评估
  - 变量
  - 覆盖控制（TBD）
  - 工具更改
  - 开关
   （*）表示可选参数，通过config.h启用并重新编译
   组0 = {G92.2，G92.3}（非模态：取消并重新启用G92偏移）
   组1 = {G81  -  G89}（运动模式：固定循环）
   组4 = {M1}（可选停止，忽略）
   组6 = {M6}（换刀）
   组7 = {G41，G42}刀具半径补偿（支持G40）
   组8 =刀具长度补偿{G43}（支持G43.1 / G49）
   组8 = {M7 *}启用雾化冷却剂（*编译选项）
	 组9 = {M48，M49，M56 *}启用/禁用覆盖开关（*编译选项）
	 组10 = {G98，G99}返回模式固定循环
   组13 = {G61.1，G64}路径控制模式（支持G61）
*/
