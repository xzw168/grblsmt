/*
  config.h - compile time configuration
  系统接收的命令定义

系统默认初始化参数定义

系统功能开关
*/

// This file contains compile-time configurations for Grbl's internal system. For the most part,
// users will not need to directly modify these, but they are here for specific needs, i.e.
// performance tuning or adjusting to non-typical machines.

// IMPORTANT: Any changes here requires a full re-compiling of the source code to propagate them.

#ifndef config_h
#define config_h
#include "grbl.h" // For Arduino IDE compatibility.


//定义CPU引脚映射和默认设置。
//注意：OEM可以避免维护/更新defaults.h和cpu_map.h文件并仅使用
//一个配置文件，通过在文件底部放置它们的特定默认值和引脚映射。
//如果这样做，只需注释掉这两个定义，并参阅下面的说明。
//#define DEFAULTS_GENERIC
#define DEFAULT_CNC3020

#ifdef STM32F103C8
#define CPU_MAP_STM32F103
#endif

//串口波特率
// #define BAUD_RATE 230400
#define BAUD_RATE 115200

//定义实时命令特殊字符。这些角色是直接从这个角色“拾起”的
//串行读取数据流，并不传递给grbl行执行分析器。选择字符
//不能也不能存在于流式g代码程序中。ASCII控制字符可能是
//使用，如果他们可用每个用户设置。此外，扩展的ASCII码（> 127），这是永远不会
// g代码程序，可能是为接口程序选择的。
//注意：如果更改，手动更新report.c中的帮助消息。

#define CMD_RESET 0x18 // ctrl-x.
#define CMD_STATUS_REPORT '?'
#define CMD_CYCLE_START '~'
#define CMD_FEED_HOLD '!'

//注意：所有覆盖实时命令必须在扩展ASCII字符集中，开始
//字符值128（0x80）和255（0xFF）。如果正常的一组实时命令，
//如状态报告，进给保持，复位和循环启动，将被移至扩展集
//空间，serial.c的RX ISR将需要修改以适应变化。
// #define CMD_RESET 0x80
// #define CMD_STATUS_REPORT 0x81
// #define CMD_CYCLE_START 0x82
// #define CMD_FEED_HOLD 0x83
#define CMD_SAFETY_DOOR 0x84
#define CMD_JOG_CANCEL  0x85
#define CMD_DEBUG_REPORT 0x86 // Only when DEBUG enabled, sends debug report in '{}' braces.
#define CMD_FEED_OVR_RESET 0x90         // Restores feed override value to 100%.
#define CMD_FEED_OVR_COARSE_PLUS 0x91
#define CMD_FEED_OVR_COARSE_MINUS 0x92
#define CMD_FEED_OVR_FINE_PLUS  0x93
#define CMD_FEED_OVR_FINE_MINUS  0x94
#define CMD_RAPID_OVR_RESET 0x95        // Restores rapid override value to 100%.
#define CMD_RAPID_OVR_MEDIUM 0x96
#define CMD_RAPID_OVR_LOW 0x97
// #define CMD_RAPID_OVR_EXTRA_LOW 0x98 // *NOT SUPPORTED*
#define CMD_SPINDLE_OVR_RESET 0x99      // Restores spindle override value to 100%.
#define CMD_SPINDLE_OVR_COARSE_PLUS 0x9A
#define CMD_SPINDLE_OVR_COARSE_MINUS 0x9B
#define CMD_SPINDLE_OVR_FINE_PLUS 0x9C
#define CMD_SPINDLE_OVR_FINE_MINUS 0x9D
#define CMD_SPINDLE_OVR_STOP 0x9E
#define CMD_COOLANT_FLOOD_OVR_TOGGLE 0xA0
#define CMD_COOLANT_MIST_OVR_TOGGLE 0xA1

//如果启用了归位，归位初始化锁定将Grbl设置为上电后的警报状态。这力量
//用户在做任何其他事情之前执行归位循环（或覆盖锁定）。这是
//主要是提醒用户回家的安全功能，因为Grbl的位置是未知的。
#define HOMING_INIT_LOCK // 评论禁用

//用位掩码定义归位循环模式。归零循环首先执行搜索模式
//快速启用限位开关，然后放慢定位模式，然后短暂完成
//拉动运动以释放限位开关。以下HOMING_CYCLE_x定义被执行
//以后缀0开始，只完成指定轴的引导程序。如果
//定义中省略了一个坐标轴，它不会回到原点，系统也不会更新它的位置。
//这意味着这允许用户使用非标准笛卡尔机器，如车床（x然后z，
//没有y），根据需要配置归位循环行为。
//注意：归位循环被设计为允许共享极限引脚，如果轴不相同
//循环，但是这需要在cpu_map.h文件中更改一些引脚设置。例如，默认的归位
//循环可以使用X或Y限制引脚共享Z限制引脚，因为它们处于不同的周期。
//通过共享一个引脚，这释放了一个宝贵的IO引脚用于其他目的。从理论上讲，所有的轴限制了引脚
//如果所有的轴都有独立的循环，反之亦然，所有的三个轴都可以减少到一个轴
//在单独的引脚上，但是在一个周期内回到原点。另外，应该注意的是硬限制的功能
//不会受到引脚共享的影响。
//注意：默认设置为传统的3轴数控机床。Z轴首先清除，然后是X＆Y.
#define HOMING_CYCLE_0 (1<<Z_AXIS)                // 要求：首先移动Z到清除工作区。
#define HOMING_CYCLE_1 ((1<<X_AXIS)|(1<<Y_AXIS))  // 可选: 然后移动X，Y同时。
// #define HOMING_CYCLE_2                         // 可选：取消注释并添加轴掩码以启用

// 注意：以下是两轴机器设置归位的两个示例。
// #define HOMING_CYCLE_0 ((1<<X_AXIS)|(1<<Y_AXIS))  // 不兼容COREXY：在一个周期内同时驻留XY。

// #define HOMING_CYCLE_0 (1<<X_AXIS)  // COREXY COMPATIBLE: First home X
// #define HOMING_CYCLE_1 (1<<Y_AXIS)  // COREXY COMPATIBLE: Then home Y

//机器初始点动以限制开关后执行的回原点周期数。
//这有助于防止过冲，并应提高重复性。这个值应该是一个或者
//更大。
#define N_HOMING_LOCATE_CYCLE 1 // Integer (1-128)

//启用单轴回原点命令。用于X，Y和Z轴归位的$ HX，$ HY和$ HZ。完整的归巢
//循环仍然由$ H命令调用。这是默认禁用的。这里只是为了解决
//需要在两轴和三轴机器之间切换的用户。这实际上是非常罕见的。
//如果你有一个双轴机器，不要使用这个。相反，只要改变两轴的回零循环即可。
// #define HOMING_SINGLE_AXIS_COMMANDS // Default disabled. Uncomment to enable.

//归位之后，Grbl将默认将整个机器空间设置为负空间，这是典型的情况
//用于专业CNC机床，无论限位开关位于何处。取消注释
//定义强制Grbl始终将机床原点设置在原点位置，而不管开关方向如何。
// #define HOMING_FORCE_SET_ORIGIN // Uncomment to enable.

// Grbl在启动时执行的块数。这些块存储在EEPROM中，其中的大小
//和地址在settings.h中定义。使用当前设置，最多可以有2个启动块
//按顺序存储和执行。这些启动块通常用于设置g代码
//解析器状态取决于用户的喜好。
#define N_STARTUP_LINE 2 // Integer (1-2)

//由Grbl为某些值类型打印的浮动小数点的数量。这些设置是
//通过数控机床中现实和常见的数值确定。例如，位置
//值不能小于0.001mm或0.0001in，因为机器不能在物理上更多
//确切地说 所以，有可能不需要改变这些，但如果你需要在这里，你可以。
//注意：必须是从0到?4的整数值。超过4个可能会出现舍入误差。
#define N_DECIMAL_COORDVALUE_INCH 4 // 坐标或以英寸的位置值
#define N_DECIMAL_COORDVALUE_MM   3 // 坐标或毫米的位置值
#define N_DECIMAL_RATEVALUE_INCH  1 // 速率或速度值在/分钟
#define N_DECIMAL_RATEVALUE_MM    0 // 在毫米率或速度值/分钟
#define N_DECIMAL_SETTINGVALUE    3 // 小数浮点设定值
#define N_DECIMAL_RPMVALUE        0 // 在每分钟转RPM值

//如果您的机器有两个并行连接到一个轴的限位开关，则需要启用
//这个功能 由于两个交换机共享一个引脚，因此Grbl无法分辨
//启用哪一个 这个选项只影响归位，如果有限制，Grbl会
//报警并强制用户手动脱开限位开关。否则，如果你有一个
//为每个轴限制开关，请不要启用此选项。通过保持禁用，您可以执行一个
//限位开关上的回零循环，而不必移动机器。
// #define LIMITS_TWO_SWITCHES_ON_AXES

//允许GRBL跟踪和报告gcode行号。启用这意味着计划缓冲区
//从16到15，为plan_block_t结构中的附加行号数据腾出空间
// #define USE_LINE_NUMBERS // 默认情况下已禁用。取消注释以启用。

//在探测周期成功后，此选项立即提供探头坐标的反馈
//通过自动生成的消息。如果禁用，用户仍然可以访问最后一个探针
//通过Grbl'$＃'打印参数进行坐标。
#define MESSAGE_PROBE_COORDINATES // 默认情况下启用。注释禁用。

//通过Arduino Uno上的喷雾冷却剂g-code命令M7启用第二个冷却液控制销
//模拟引脚4.如果需要第二个冷却液控制引脚，则只能使用该选项。
//注意：无论如何，模拟引脚3上的M8溢流冷却液控制引脚仍然工作。
// #define ENABLE_M7 // Disabled by default. Uncomment to enable.

//此选项使进给保持输入作为安全门开关。一个安全门被触发时，
//立即强制进料，然后安全地切断机器的电源。恢复被阻止，直到
//安全门重新接合。如果是这样，Grbl会重新启动机器，然后重新开机
//以前的刀具路径，就像什么都没发生一样。
// #define ENABLE_SAFETY_DOOR_INPUT_PIN // Default disabled. Uncomment to enable.

//安全门开关切换并恢复后，此设置将设置加电延迟
//在恢复主轴和冷却液之间恢复循环。
#define SAFETY_DOOR_SPINDLE_DELAY 4.0 // Float (seconds)
#define SAFETY_DOOR_COOLANT_DELAY 1.0 // Float (seconds)

//启用CoreXY运动。仅与CoreXY机器一起使用。
//重要提示：如果启用了回原点，则必须将上面的定义循环#define重新配置为
// #define HOMING_CYCLE_0（1 << X_AXIS）和#define HOMING_CYCLE_1（1 << Y_AXIS）
//注意：该配置选项将X轴和Y轴的运动更改为操作原理
//定义在（http://corexy.com/theory.html）。假定电动机的定位和连线完全相同
//描述，如果不是，动作可能会沿着奇怪的方向移动。Grbl需要CoreXY A和B电机
//内部每毫米具有相同的步骤
// #define COREXY // Default disabled. Uncomment to enable.

//根据掩码反转控制命令引脚的引脚逻辑。这基本上意味着你可以使用
//指定引脚上的常闭开关，而不是默认的常开开关。
//注意：最上面的选项将屏蔽和反转所有控制引脚。底部选项是一个例子
//只反转两个控制引脚，安全门和复位。有关其他位定义，请参见cpu_map.h。
// #define INVERT_CONTROL_PIN_MASK CONTROL_MASK // Default disabled. Uncomment to disable.
// #define INVERT_CONTROL_PIN_MASK ((1<<CONTROL_SAFETY_DOOR_BIT)|(CONTROL_RESET_BIT)) // Default disabled.

//根据以下掩码反转选择限制引脚状态。这会影响所有的限制引脚功能，
//如硬限制和归位。但是，这与整体反转限制设置不同。
//此构建选项将只反转此处定义的限制引脚，然后反转限制设置
//将被应用到所有的人。当用户有两个限制引脚混合使用时，这是非常有用的
//机器上安装了常开（NO）和常闭（NC）开关。
//注意：请不要使用这个，除非你有一个需要它的情况。
// #define INVERT_LIMIT_PIN_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)) // Default disabled. Uncomment to enable.

//将主轴使能引脚从低电平有效/高电平有效转换为低电平有效/高电平有效。有用
//用于一些预制电子板。
//注意：如果VARIABLE_SPINDLE被启用（默认），这个选项与PWM输出和
//主轴使能被合并到一个引脚。如果你需要这个选项和主轴转速PWM，
//取消注释下面的配置选项USE_SPINDLE_DIR_AS_ENABLE_PIN。
// #define INVERT_SPINDLE_ENABLE_PIN // Default disabled. Uncomment to enable.

//将选定的冷却液引脚从低电平有效/高电平有效转换为低电平有效/高电平有效。有用
//用于一些预制电子板。
// #define INVERT_COOLANT_FLOOD_PIN // Default disabled. Uncomment to enable.
// #define INVERT_COOLANT_MIST_PIN // Default disabled. Note: Enable M7 mist coolant in config.h

//当Grbl通电或者用Arduino复位按钮硬重启时，Grbl启动时不会报警
//默认情况下 这是为了让新用户尽可能简单地开始使用Grbl。归巢时
//启用并且用户已经安装限位开关，Grbl将在ALARM状态下启动以指示
// Grbl不知道它的位置并在继续之前强迫用户回家。这个选项强制
// Grbl总是初始化进入报警状态，无论归巢与否。这个选项更多
// OEM和LinuxCNC用户喜欢这个电源循环行为。
// #define FORCE_INITIALIZATION_ALARM // Default disabled. Uncomment to enable.

//在上电或复位时，Grbl将检查限位开关状态以确保它们不处于活动状态
//初始化之前 如果它检测到一个问题，硬限制设置被启用，Grbl会
//简单地通知用户检查限制并进入报警状态，而不是空闲。Grbl会的
//不发出警报信息
#define CHECK_LIMITS_AT_INIT

// ---------------------------------------------------------------------------------------
//高级配置选项：

//启用用于调试目的的代码。不适合一般使用，并始终保持不变。
// #define DEBUG //取消注释以启用。默认禁用。

//配置快速，进给和主轴覆盖设置。这些值定义了最大值和最小值
//允许的覆盖值和每个命令收到的粗调和细调。请
//注意每个定义后面的描述中的允许值。
#define DEFAULT_FEED_OVERRIDE           100 // 100％。不要改变这个值。
#define MAX_FEED_RATE_OVERRIDE          200 // 百分比编程进给速率（100-255）的。通常120％或200％
#define MIN_FEED_RATE_OVERRIDE           10 // 百分比编程进给速率（1-100）的。通常50％或1％
#define FEED_OVERRIDE_COARSE_INCREMENT   10 // (1-99). 通常 10%.
#define FEED_OVERRIDE_FINE_INCREMENT      1 // (1-99). 通常 1%.

#define DEFAULT_RAPID_OVERRIDE  100 // 100%. 不要改变这个值。
#define RAPID_OVERRIDE_MEDIUM    50 // Percent of rapid (1-99). Usually 50%.
#define RAPID_OVERRIDE_LOW       25 // Percent of rapid (1-99). Usually 25%.
// #define RAPID_OVERRIDE_EXTRA_LOW 5 // *NOT SUPPORTED* Percent of rapid (1-99). Usually 5%.

#define DEFAULT_SPINDLE_SPEED_OVERRIDE    100 // 100%. 不要改变这个值。
#define MAX_SPINDLE_SPEED_OVERRIDE        200 // 百分比编程的主轴速度（100-255）的。通常是200％。
#define MIN_SPINDLE_SPEED_OVERRIDE         10 // 百分比编程的主轴转速（1-100）的。通常是10％。
#define SPINDLE_OVERRIDE_COARSE_INCREMENT  10 // (1-99). Usually 10%.
#define SPINDLE_OVERRIDE_FINE_INCREMENT     1 // (1-99). Usually 1%.

//执行M2或M30程序结束命令时，大部分g代码状态都恢复为默认值。
//此编译时间选项包括恢复进给，快速和主轴速度覆盖值
//到程序结束时的默认值。
#define RESTORE_OVERRIDES_AFTER_PROGRAM_END // Default enabled. Comment to disable.

// Grbl v1.1的状态报告发生变化，同时也删除了禁用/启用大部分数据的功能
//报告中的字段 这给GUI开发人员带来了一些问题，他们必须管理好几个场景
//和配置。新报告样式的效率提高允许所有数据字段
//发送没有潜在的性能问题。
//注意：下面的选项只是提供了一种方法来禁用某些数据字段，如果唯一的
//情况需要它，但要注意GUI可能依赖于这些数据。如果禁用，则可能不兼容。
#define REPORT_FIELD_BUFFER_STATE // Default enabled. Comment to disable.
#define REPORT_FIELD_PIN_STATE // Default enabled. Comment to disable.
#define REPORT_FIELD_CURRENT_FEED_SPEED // Default enabled. Comment to disable.
#define REPORT_FIELD_WORK_COORD_OFFSET // Default enabled. Comment to disable.
#define REPORT_FIELD_OVERRIDES // Default enabled. Comment to disable.
#define REPORT_FIELD_LINE_NUMBERS // Default enabled. Comment to disable.

//一些状态报告数据不是实时需要的，只是间歇性的，因为这些值没有
//经常改变 以下宏配置之前需要调用状态报告的次数
//关联的数据被刷新并包含在状态报告中。但是，如果这些价值之一
//更改，Grbl会自动将这些数据包含在下一个状态报告中，而不管是什么
//计数在当时 这有助于减少与高频率报告有关的通信开销
//和积极的流媒体。还有一个繁忙和空闲的刷新计数，设置Grbl发送
//当它没有做任何重要的事情时更频繁地刷新。用好的GUI，这个数据不需要
//经常刷新，大约几秒钟。
//注意：WCO刷新必须是2或更大。OVR刷新必须是1或更大。
#define REPORT_OVR_REFRESH_BUSY_COUNT 20  // (1-255)
#define REPORT_OVR_REFRESH_IDLE_COUNT 10  // (1-255) Must be less than or equal to the busy count
#define REPORT_WCO_REFRESH_BUSY_COUNT 30  // (2-255)
#define REPORT_WCO_REFRESH_IDLE_COUNT 10  // (2-255) Must be less than or equal to the busy count

//加速管理子系统的时间分辨率 数字越高越平滑
//加速度，特别是在运行速度非常高的机器上，但可能会有负面影响
//影响性能 这个参数的正确值是机器相关的，所以建议
//根据需要设置这个高度。大致成功的值可以从50到200或更多。
//注意：更改此值也会更改步段缓冲区中段的执行时间。
//当增加这个值的时候，这个存储在缓冲区中的总时间会减少，反之亦然。使
//确定步长段缓冲区增加/减少以应对这些变化。
#define ACCELERATION_TICKS_PER_SECOND 100

//自适应多轴步平滑（AMASS）是一个先进的功能，它的名字意味着什么，
//平滑多轴运动的步进。此功能特别是在低台阶下平滑运动
//低于10kHz的频率，多轴运动轴之间的混叠可能导致发声
//噪音和震动你的机器。甚至更低的步频，AMASS适应并提供更好的
//步平滑。有关AMASS系统工作的更多细节，请参阅stepper.c。
#define ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING  // 默认启用。 注释禁用。。Default enabled. Comment to disable.

//设置允许写入Grbl设置的最大步进速率。该选项启用错误
//检查设置模块以防止超出此限制的设置值。最大值
//步进速率严格受CPU速度的限制，如果AVR运行以外的其他情况将会改变
//使用16MHz。
//注意：现在禁用，如果闪存空间允许，将启用。
// #define MAX_STEP_RATE_HZ 30000 // Hz

// By default, Grbl sets all input pins to normal-high operation with their internal pull-up resistors
// enabled. This simplifies the wiring for users by requiring only a switch connected to ground,
// although its recommended that users take the extra step of wiring in low-pass filter to reduce
// electrical noise detected by the pin. If the user inverts the pin in Grbl settings, this just flips
// which high or low reading indicates an active signal. In normal operation, this means the user
// needs to connect a normal-open switch, but if inverted, this means the user should connect a
// normal-closed switch.
// The following options disable the internal pull-up resistors, sets the pins to a normal-low
// operation, and switches must be now connect to Vcc instead of ground. This also flips the meaning
// of the invert pin Grbl setting, where an inverted setting now means the user should connect a
// normal-open switch and vice versa.
// NOTE: All pins associated with the feature are disabled, i.e. XYZ limit pins, not individual axes.
// WARNING: When the pull-ups are disabled, this requires additional wiring with pull-down resistors!
//#define DISABLE_LIMIT_PIN_PULL_UP
//#define DISABLE_PROBE_PIN_PULL_UP
//#define DISABLE_CONTROL_PIN_PULL_UP

//设置应用刀具长度补偿的轴。假设主轴始终与主轴平行
//刀具朝向负方向的选定轴。换句话说，一个积极的
//从当前位置减去刀具长度补偿值。
#define TOOL_LENGTH_OFFSET_AXIS Z_AXIS // 默认z轴。有效值是X_AXIS，Y_AXIS或Z_AXIS。

//为不同的RPM值启用可变主轴输出电压。在Arduino Uno上，主轴
//启用引脚将输出5V，最大转速为256，中间电平为0V，禁用时为0V。
//注意：对于Arduino Unos重要！当使能时，Z限制引脚D11和主轴使能引脚D12切换！
// D11引脚上的硬件PWM输出对于可变主轴输出电压是必需的。
#define VARIABLE_SPINDLE // /默认启用。注释禁用。

//仅由可变主轴输出使用。当使能时，这将强制PWM输出达到最小占空比。
//当主轴被禁止时，PWM引脚仍将读取0V。大多数用户不需要这个选项，但是
//在某些情况下它可能是有用的。此最小PWM设置与主轴转速最小值一致
//设置，如最大转速到最大PWM。如果您需要禁用0V之间的较大电压差，这是非常方便的
//最小PWM所设置的电压，最小转速。这个差值是每个PWM值0.02V。所以，什么时候
//最小PWM为1，只有0.02伏分开启用和禁用。在PWM 5时，这将是0.1V。保持
//记住，由于增加了最小PWM值，您将开始失去PWM分辨率
//在总共255个PWM电平范围内有较小的范围来表示不同的主轴速度。
//注意：通过以下等式计算最小PWM的占空比:(占空比％）=（SPINDLE_PWM_MIN_VALUE / 255）* 100
// #define SPINDLE_PWM_MIN_VALUE 5 // Default disabled. Uncomment to enable. Must be greater than zero. Integer (1-255).

//默认情况下，在一个328p（Uno）上，Grbl将可变主轴PWM和使能信号合并到一个引脚中以提供帮助
//保留I / O引脚。对于某些设置，这些可能需要单独的引脚。这个配置选项使用
//主轴方向引脚（D13）作为单独的主轴使能引脚，以及引脚D11上的主轴速度PWM。
//注意：此配置选项仅适用于启用VARIABLE_SPINDLE和328p处理器（Uno）。
//注意：如果没有方向引脚，M4将不会有引脚输出来指示与M3的区别。
//注意：请注意！上电时，Arduino引导装载程序切换D13引脚。如果你用Grbl闪光
//一个程序员（你可以使用一个备用的Arduino作为“Arduino as ISP”。在网上搜索如何连线）
//这个D13 LED切换应该消失。我们还没有测试过这个。请报告如何去！
// #define USE_SPINDLE_DIR_AS_ENABLE_PIN // Default disabled. Uncomment to enable.

//使用USE_SPINDLE_DIR_AS_ENABLE_PIN选项更改主轴使能引脚的行为。默认，
//如果主轴速度为零且M3 / 4有效，Grbl将不会禁用使能引脚，但仍会设置PWM
//输出为零 这允许用户知道主轴是否处于活动状态，并将其用作附加控制
//输入。但是，在某些使用情况下，用户可能希望使能引脚以零主轴速度禁用
//主轴转速大于零时重新启用。这个选项可以。
//注意：需要启用USE_SPINDLE_DIR_AS_ENABLE_PIN。
// #define SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED // Default disabled. Uncomment to enable.

//启用此功能后，Grbl将发送已收到的已经预解析的行的回显（空格）
//删除，大写字母，没有评论），并立即由Grbl执行。回声不会
//发送一行缓冲区溢出，但是应该发送给Grbl的所有普通线路。例如，如果一个用户
//发送'g1 x1.032 y2.45（test comment）'这一行，Grbl会以'[echo：G1X1.032Y2.45]'的形式回显。
//注意：仅用于调试目的！回声时，这占用宝贵的资源，可以起作用
//性能。如果正常工作所需要的，串行写入缓冲器应该大大增加
//帮助最大限度地减少串行写入协议内的传输等待。
// #define REPORT_ECHO_LINE_RECEIVED // Default disabled. Uncomment to enable.

//最小规划师路口速度 设置计划者计划在的默认最小交接点速度
//每个缓冲区块结点，除了始终从缓冲区的始端和结尾开始
//零。该值控制机器通过连接点移动的速度，而不考虑加速度
//相邻块行移动方向之间的限制或角度。这对于不能的机器很有用
//容忍一秒钟的工具，即3D打印机或激光切割机。如果使用，这个值
//不应大于零或达到机器工作所需的最小值。
#define MINIMUM_JUNCTION_SPEED 0.0f // (mm/min)

//设置规划器允许的最小进给速度。低于它的任何值都将被设置为最小值
//值。这也确保了一个计划的动作总是完成并计算任何浮点数
//舍入错误。尽管不推荐，但低于1.0毫米/分钟的值可能会更小
//机器，也许到0.1mm / min，但是您的成功可能因多种因素而有所不同。
#define MINIMUM_FEED_RATE 1.0 // (mm/min)

//在精确的弧线轨迹之前通过小角度逼近的弧生成迭代次数
//用昂贵的sin（）和cos（）计算校正。如果有这个参数可能会减少
//是否具有电弧生成的准确性问题，或者如果电弧执行正在获得则会增加
//由于太多的trig计算而陷入困境。
#define N_ARC_CORRECTION 12 // Integer (1-255)

// G2 / 3 g代码标准的定义是有问题的。基于半径的弧具有可怕的数值
//半圆（pi）或全圆（2 * pi）弧形时的错误。基于偏移的弧更准确
//当弧是全圆（2 * pi）时仍然有问题。这为浮动帐户定义
//当基于偏移量的圆弧被命令为完整的圆形时，指出问题，但被解释为非常
//由于数值舍入和精确度问题，具有约机器ε（1.2e-7rad）的小弧。
//这个定义值设置机器的epsilon cutoff来确定弧是否是一个全圆。
//注意：调整此值时要非常小心。应该总是大于1.2e-7，但不能太大
//远远大于这个 默认设置应该捕获大部分（如果不是全部的话）全弧误差情况。
#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7 // Float (radians)

//在暂停期间执行延时增量。默认值设置为50ms，这提供了
//大约55分钟的最大时间延迟，对于大多数应用程序来说绰绰有余。增加
//这个延迟会线性增加最大停留时间，但也会降低响应性
//运行时命令的执行，就像状态报告一样，因为这些是在每个停顿之间执行的
//时间步骤 另外，请记住，Arduino延时定时器对于长时间的延迟不是很准确。
#define DWELL_TIME_STEP 50 // Integer (1-255) (milliseconds)

//通过创建，在方向引脚设置和相应的步进脉冲之间创建一个延迟
//另一个中断（Timer2比较）来管理它。主要的Grbl中断（Timer1比较）
//设置方向引脚，并不立即设置步进引脚，因为它会
//正常运行。Timer2比较接下来触发，设置步进之后的步进引脚
//脉冲延迟时间，Timer2溢出将完成步进脉冲，除了现在延迟
//由步进脉冲时间加上步进脉冲延迟。（感谢langwadt的想法！）
//注意：取消注释以启用。推荐的延迟时间必须> 3us，并且在添加了
//用户提供的步脉冲时间，总时间不得超过127us。报告成功
//某些设置的值范围从5到20us。
// #define STEP_PULSE_DELAY 10 // Step pulse delay in microseconds. Default disabled.

//在任何给定时间计划的规划缓冲区中的线性运动的数量。茫茫
// Grbl使用的大部分内存都基于这个缓冲区大小。只有有额外的时候才增加
//可用的RAM，就像重新编译Mega2560一样。或者如果Arduino开始减少
//由于缺乏可用的内存或如果CPU无法跟上计划而崩溃
//执行时新的传入动作。
// #define BLOCK_BUFFER_SIZE 16 // Uncomment to override default in planner.h.

//控制步骤执行算法之间的中间步段缓冲区的大小
//和计划者块。每一段都是以一个恒定的速度执行的
//由ACCELERATION_TICKS_PER_SECOND定义的固定时间。他们是这样计算的
//块速度轮廓被精确地追踪。这个缓冲区的大小决定了多少步
//执行提前期还有其他的Grbl进程需要计算和执行它们的事情
//在返回并重新填充该缓冲区之前，当前在步进约50毫秒时移动。
// #define SEGMENT_BUFFER_SIZE 6 // Uncomment to override default in stepper.h.

//要执行的串行输入流中的行缓冲区大小。另外，管理的大小
//每个启动块，因为它们都是以这种大小的字符串存储的。确保
//在settings.h和for中为已定义的内存地址记录可用的EEPROM
//所需启动块的数量
//注意：除极端情况外，80个字符不是问题，但行缓冲区大小
//可能太小，g代码块可能会被截断。g代码标准正式
//最多支持256个字符 在将来的版本中，这个默认值会增加
//我们知道我们可以重新投入多少额外的内存空间。
// #define LINE_BUFFER_SIZE 80  // Uncomment to override default in protocol.h

//串行发送和接收缓冲区大小。接收缓冲区通常用作另一个流
//缓存来存储Grbl处理的传入块。最流的
//接口将字符计数并跟踪每个块发送给每个块的响应。所以，
//增加接收缓冲区，如果需要更深的接收缓冲区进行流媒体和可用
//记忆允许 发送缓冲区主要处理Grbl中的消息。只有增加，如果大
//发送消息，Grbl开始停滞，等待发送剩余的消息。
//注意：Grbl在0.5毫秒左右产生一个平均状态报告，但是串行TX流在
// 115200波特将需要5毫秒来传输一个典型的55个字符的报告。最糟的病例报告是
//大约90-100个字符 只要串行TX缓冲区不会持续最大，Grbl
//将继续高效运行。将TX缓冲区大小设置为最差情况报告的大小。
#if !defined (STM32F103C8)
// #define RX_BUFFER_SIZE 128 // (1-254) Uncomment to override defaults in serial.h
// #define TX_BUFFER_SIZE 100 // (1-254)
#endif

//硬限位开关的简单软件去抖功能。当启用时，中断
//监控硬限位开关引脚将使Arduino的看门狗定时器重新检查
//延迟约32毫秒后的限制引脚状态。这可以帮助数控机床
//有问题的错误触发他们的硬限位开关，但它不会解决问题
//来自外部信号电缆的电气干扰。建议首先
//使用带屏蔽层的屏蔽信号电缆（旧的USB /计算机电缆
//工作良好，价格便宜），并将低通电路连接到每个限位引脚。
// #define ENABLE_SOFTWARE_DEBOUNCE // Default disabled. Uncomment to enable.

//在Grbl的检查模式下，在探测循环之后配置位置。禁用集合
//探测目标的位置，启用时将位置设置为开始位置。
// #define SET_CHECK_MODE_PROBE_TO_START // Default disabled. Uncomment to enable.

//强制Grbl在处理器检测到引脚时检查硬限位开关的状态
//改变硬限制的ISR例程。默认情况下，Grbl会触发硬限制
//任何引脚改变时都会报警，因为弹跳开关可能导致这样的状态检查
//误读了别针 当硬限制被触发时，它们应该是100％可靠的，这就是
//原因是这个选项默认是禁用的。只有当你的系统/电子产品可以保证
//开关不反弹，我们建议启用此选项。这将有助于防止
//当机器脱离开关时触发硬限制。
//注意：如果启用SOFTWARE_DEBOUNCE，则此选项不起作用。
// #define HARD_LIMIT_FORCE_STATE_CHECK // Default disabled. Uncomment to enable.

//调整归位周期搜索并定位标量。这些是Grbl使用的乘数
//归位循环，确保限位开关在每个相位上都被占用和清除
//循环。搜索阶段使用轴最大行程设置时间SEARCH_SCALAR到
//确定距离以查找限位开关。一旦找到，定位阶段开始
//使用LOCATE_SCALAR的归位脱离距离设置来拉开和重新啮合
//限位开关
//注意：这两个值都必须大于1.0以确保正确的功能。
// #define HOMING_AXIS_SEARCH_SCALAR  1.5f // 取消注释以覆盖limits.c中的默认值。
// #define HOMING_AXIS_LOCATE_SCALAR  10.0f // 取消注释以覆盖limits.c中的默认值。

//启用'$ RST = *'，'$ RST = $'和'$ RST =＃'eeprom restore命令。有的情况下
//这些命令可能是不可取的。只需注释所需的宏以禁用它。
//注意：请参阅SETTINGS_RESTORE_ALL宏来定制`$ RST = *`命令。
#define ENABLE_RESTORE_EEPROM_WIPE_ALL         // '$RST=*' 默认启用。注释禁用.
#define ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS // '$RST=$' 默认启用。注释禁用.
#define ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS // '$RST=#' 默认启用。注释禁用.

//定义在设置版本更改和`$ RST = *`命令时恢复的EEPROM数据。每当
// Grbl版本之间的设置或其他EEPROM数据结构会发生变化，Grbl会自动生成
//擦除并恢复EEPROM。这个宏控制着什么数据被擦除和恢复。这很有用
//特别是需要保留某些数据的OEM。例如，BUILD_INFO字符串可以
//通过单独的.INO草图写入Arduino EEPROM以包含产品数据。改变这一点
//宏不能恢复编译信息EEPROM将确保在固件升级后保留这些数据。
//注意：取消注释以覆盖settings.h中的默认值
// #define SETTINGS_RESTORE_ALL (SETTINGS_RESTORE_DEFAULTS | SETTINGS_RESTORE_PARAMETERS | SETTINGS_RESTORE_STARTUP_LINES | SETTINGS_RESTORE_BUILD_INFO)

//启用'$ I =（string）'build info write命令。如果禁用，任何现有的构建信息数据都必须
//通过具有有效校验和值的外部装置将其置于EEPROM中。这个宏选项很有用
//用于防止用户覆盖OEM数据时这些数据被覆盖。
//注意：如果禁用并确保Grbl永远不能改变构建信息行，则还需要启用
//上面的SETTING_RESTORE_ALL宏，并从掩码中移除SETTINGS_RESTORE_BUILD_INFO。
//注意：请参阅附带的grblWrite_BuildInfo.ino示例文件来单独编写此字符串。
#define ENABLE_BUILD_INFO_WRITE_COMMAND // '$I=' Default enabled. Comment to disable.

// AVR处理器要求在EEPROM写入期间禁止所有中断。这包括两个
//步进器ISR和串行通信ISR。如果长时间写入EEPROM，这个ISR暂停可以
//导致主动步进失去位置，串口接收数据丢失。这个配置
//选项强制计划器缓冲区完全清空，只要写入EEPROM即可
//任何丢失步骤的机会
//然而，这并不能防止EEPROM写入期间丢失串行接收数据的问题，特别是
//如果一个GUI预先同时填充串行接收缓冲区。这是非常建议的
//这些g代码（G10，G28.1，G30.1）的图形用户界面在包含一个块后总是等待“ok”
//发送更多数据以消除此问题之前，这些命令之一。
//注意：大多数EEPROM写命令在作业期间被隐式阻塞（所有'$'命令）。然而，
//坐标集g代码命令（G10，G28 / 30.1）不是，因为它们是活动流的一部分
//工作 目前，这个选项只会强制规划缓存与这些g-code命令同步。
#define FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE // Default enabled. Comment to disable.

//在Grbl v0.9以及之前的版本中，有一个老工具仓库报告的bug
//可能与正在执行的内容不相关，因为`WPos：`是基于g-code解析器状态的
//可以在后面几个动作。此选项强制规划器缓冲区清空，同步和停止
//只要有改变工件坐标偏移量的命令G10，G43.1，G92，G54-59`就运动。
//这是确保“WPos：”总是正确的最简单的方法。幸运的是，这是非常罕见的
//使用这些命令中的任何一个都需要通过它们进行连续的运动。
#define FORCE_BUFFER_SYNC_DURING_WCO_CHANGE // Default enabled. Comment to disable.

//默认情况下，Grbl禁用所有G38.x探测循环命令的进给速率覆盖。虽然这样
//可能与某些亲类机器控制不同，可以说这应该是这样的。
//大多数探头传感器会产生不同程度的错误，这取决于速度。通过
//将探测循环保持在其编程的进给速率，探头传感器应该更多
//可重复 如果需要，可以通过取消注释下面的定义来禁用此行为。
// #define ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES // Default disabled. Uncomment to enable.

//在安全门状态下启用和配置停车运动方法。主要针对OEM
//希望这个功能适用于他们的集成机器。目前，Grbl认为
//停车动作只涉及一个轴，尽管停车实施已经写好
//通过改变停车位，可以很容易地为任何数量的不同轴上的运动重构
//源代码。此时，Grbl只支持停放一个轴（通常是Z轴）
//在回位时向正方向移动，在恢复位置时向负方向移动。
//运动以缓慢的拉出回退运动，关闭电源和快速停车来执行。
//恢复到恢复位置跟随这些设置运动相反：快速恢复到
//拉出位置，通电超时，然后回到原来的位置
//较慢的提取率
//注意：仍然在进行中。机器坐标必须在所有负面空间中
//不能和HOMING_FORCE_SET_ORIGIN一起使用。停车运动也只能进入
//正面的方向
// #define PARKING_ENABLE  // Default disabled. Uncomment to enable

//配置停车运动的选项（如果启用）。
#define PARKING_AXIS Z_AXIS // Define which axis that performs the parking motion
#define PARKING_TARGET -5.0f // Parking axis target. In mm, as machine coordinate [-max_travel,0].
#define PARKING_RATE 500.0f // Parking fast rate after pull-out in mm/min.
#define PARKING_PULLOUT_RATE 100.0f // Pull-out/plunge slow feed rate in mm/min.
#define PARKING_PULLOUT_INCREMENT 5.0f // Spindle pull-out and plunge distance in mm. Incremental distance.
                                      // Must be positive value or equal to zero.

//启用一组特殊的M代码命令来启用和禁用停车运动。
//这些由M56，M56 P1或M56 Px控制，M56 P0为禁止。
//该命令是模式化的，将在规划器同步后设置。由于它是g代码，它是
//与g-code命令同步执行。这不是一个实时的命令。
//注意：PARKING_ENABLE是必需的。默认情况下，M56在初始化时处于活动状态。使用
// DEACTIVATE_PARKING_UPON_INIT to set M56 P0 as the power-up default.
// #define ENABLE_PARKING_OVERRIDE_CONTROL   // Default disabled. Uncomment to enable
// #define DEACTIVATE_PARKING_UPON_INIT // Default disabled. Uncomment to enable.

//通过调用主轴停止，此选项将在进给保持期间自动禁用激光器
//停止后立即覆盖。但是，这也意味着激光仍然可以
//如果需要禁用主轴停止覆盖，可以重新启用。这完全是一个安全功能
//确保激光在停止时不会无意中保持通电状态并引发火灾。
#define DISABLE_LASER_DURING_HOLD // Default enabled. Comment to disable.

//启用主轴PWM /速度输出的分段线性模型。需要一个解决方案
// repo的/ doc / script文件夹中的'fit_nonlinear_spindle.py'脚本。请参阅文件评论
//关于如何收集主轴数据并运行脚本以生成解决方案。
// #define ENABLE_PIECEWISE_LINEAR_SPINDLE  // Default disabled. Uncomment to enable.

// N_PIECES，RPM_MAX，RPM_MIN，RPM_POINTxx和RPM_LINE_XX常量全部设置并由
//在“fit_nonlinear_spindle.py”脚本的解决方案。仅在ENABLE_PIECEWISE_LINEAR_SPINDLE时使用
//已启用。确保常量值与脚本解决方案完全相同。
//注意：当N_PIECES <4时，未使用的RPM_LINE和RPM_POINT定义不需要和省略。
#define N_PIECES 4  // 整数（1-4）。脚本解决方案中使用的分段线数。
#define RPM_MAX  11686.4  // 模型的最大RPM。$ 30> RPM_MAX将被限制为RPM_MAX。
#define RPM_MIN  202.5    // 模型的最小RPM。$ 31 <RPM_MIN将被限制为RPM_MIN。
#define RPM_POINT12  6145.4  // Used N_PIECES >=2. Junction point between lines 1 and 2.
#define RPM_POINT23  9627.8  // Used N_PIECES >=3. Junction point between lines 2 and 3.
#define RPM_POINT34  10813.9 // Used N_PIECES = 4. Junction point between lines 3 and 4.
#define RPM_LINE_A1  3.197101e-03  // Used N_PIECES >=1. A and B constants of line 1.
#define RPM_LINE_B1  -3.526076e-1
#define RPM_LINE_A2  1.722950e-2   // Used N_PIECES >=2. A and B constants of line 2.
#define RPM_LINE_B2  8.588176e+01
#define RPM_LINE_A3  5.901518e-02  // Used N_PIECES >=3. A and B constants of line 3.
#define RPM_LINE_B3  4.881851e+02
#define RPM_LINE_A4  1.203413e-01  // Used N_PIECES = 4. A and B constants of line 4.
#define RPM_LINE_B4  1.151360e+03

/* ---------------------------------------------------------------------------------------
   OEM Single File Configuration Option

   Instructions: Paste the cpu_map and default setting definitions below without an enclosing
   #ifdef. Comment out the CPU_MAP_xxx and DEFAULT_xxx defines at the top of this file, and
   the compiler will ignore the contents of defaults.h and cpu_map.h and use the definitions
   below.
*/

// Paste CPU_MAP definitions here.

// Paste default settings definitions here.


#endif
