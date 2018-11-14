/*
  config.h - compile time configuration
  ϵͳ���յ������

ϵͳĬ�ϳ�ʼ����������

ϵͳ���ܿ���
*/

// This file contains compile-time configurations for Grbl's internal system. For the most part,
// users will not need to directly modify these, but they are here for specific needs, i.e.
// performance tuning or adjusting to non-typical machines.

// IMPORTANT: Any changes here requires a full re-compiling of the source code to propagate them.

#ifndef config_h
#define config_h
#include "grbl.h" // For Arduino IDE compatibility.


//����CPU����ӳ���Ĭ�����á�
//ע�⣺OEM���Ա���ά��/����defaults.h��cpu_map.h�ļ�����ʹ��
//һ�������ļ���ͨ�����ļ��ײ��������ǵ��ض�Ĭ��ֵ������ӳ�䡣
//�����������ֻ��ע�͵����������壬�����������˵����
//#define DEFAULTS_GENERIC
#define DEFAULT_CNC3020

#ifdef STM32F103C8
#define CPU_MAP_STM32F103
#endif

//���ڲ�����
// #define BAUD_RATE 230400
#define BAUD_RATE 115200

//����ʵʱ���������ַ�����Щ��ɫ��ֱ�Ӵ������ɫ��ʰ�𡱵�
//���ж�ȡ���������������ݸ�grbl��ִ�з�������ѡ���ַ�
//����Ҳ���ܴ�������ʽg��������С�ASCII�����ַ�������
//ʹ�ã�������ǿ���ÿ���û����á����⣬��չ��ASCII�루> 127����������Զ����
// g������򣬿�����Ϊ�ӿڳ���ѡ��ġ�
//ע�⣺������ģ��ֶ�����report.c�еİ�����Ϣ��

#define CMD_RESET 0x18 // ctrl-x.
#define CMD_STATUS_REPORT '?'
#define CMD_CYCLE_START '~'
#define CMD_FEED_HOLD '!'

//ע�⣺���и���ʵʱ�����������չASCII�ַ����У���ʼ
//�ַ�ֵ128��0x80����255��0xFF�������������һ��ʵʱ���
//��״̬���棬�������֣���λ��ѭ������������������չ��
//�ռ䣬serial.c��RX ISR����Ҫ�޸�����Ӧ�仯��
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

//��������˹�λ����λ��ʼ��������Grbl����Ϊ�ϵ��ľ���״̬��������
//�û������κ���������֮ǰִ�й�λѭ�����򸲸�������������
//��Ҫ�������û��ؼҵİ�ȫ���ܣ���ΪGrbl��λ����δ֪�ġ�
#define HOMING_INIT_LOCK // ���۽���

//��λ���붨���λѭ��ģʽ������ѭ������ִ������ģʽ
//����������λ���أ�Ȼ�������λģʽ��Ȼ��������
//�����˶����ͷ���λ���ء�����HOMING_CYCLE_x���屻ִ��
//�Ժ�׺0��ʼ��ֻ���ָ����������������
//������ʡ����һ�������ᣬ������ص�ԭ�㣬ϵͳҲ�����������λ�á�
//����ζ���������û�ʹ�÷Ǳ�׼�ѿ����������糵����xȻ��z��
//û��y����������Ҫ���ù�λѭ����Ϊ��
//ע�⣺��λѭ�������Ϊ�����������ţ�����᲻��ͬ
//ѭ������������Ҫ��cpu_map.h�ļ��и���һЩ�������á����磬Ĭ�ϵĹ�λ
//ѭ������ʹ��X��Y�������Ź���Z�������ţ���Ϊ���Ǵ��ڲ�ͬ�����ڡ�
//ͨ������һ�����ţ����ͷ���һ�������IO������������Ŀ�ġ��������Ͻ������е�������������
//������е��ᶼ�ж�����ѭ������֮��Ȼ�����е������ᶼ���Լ��ٵ�һ����
//�ڵ����������ϣ�������һ�������ڻص�ԭ�㡣���⣬Ӧ��ע�����Ӳ���ƵĹ���
//�����ܵ����Ź����Ӱ�졣
//ע�⣺Ĭ������Ϊ��ͳ��3�����ػ�����Z�����������Ȼ����X��Y.
#define HOMING_CYCLE_0 (1<<Z_AXIS)                // Ҫ�������ƶ�Z�������������
#define HOMING_CYCLE_1 ((1<<X_AXIS)|(1<<Y_AXIS))  // ��ѡ: Ȼ���ƶ�X��Yͬʱ��
// #define HOMING_CYCLE_2                         // ��ѡ��ȡ��ע�Ͳ����������������

// ע�⣺����������������ù�λ������ʾ����
// #define HOMING_CYCLE_0 ((1<<X_AXIS)|(1<<Y_AXIS))  // ������COREXY����һ��������ͬʱפ��XY��

// #define HOMING_CYCLE_0 (1<<X_AXIS)  // COREXY COMPATIBLE: First home X
// #define HOMING_CYCLE_1 (1<<Y_AXIS)  // COREXY COMPATIBLE: Then home Y

//������ʼ�㶯�����ƿ��غ�ִ�еĻ�ԭ����������
//�������ڷ�ֹ���壬��Ӧ����ظ��ԡ����ֵӦ����һ������
//����
#define N_HOMING_LOCATE_CYCLE 1 // Integer (1-128)

//���õ����ԭ���������X��Y��Z���λ��$ HX��$ HY��$ HZ�������Ĺ鳲
//ѭ����Ȼ��$ H������á�����Ĭ�Ͻ��õġ�����ֻ��Ϊ�˽��
//��Ҫ��������������֮���л����û�����ʵ�����Ƿǳ������ġ�
//�������һ��˫���������Ҫʹ��������෴��ֻҪ�ı�����Ļ���ѭ�����ɡ�
// #define HOMING_SINGLE_AXIS_COMMANDS // Default disabled. Uncomment to enable.

//��λ֮��Grbl��Ĭ�Ͻ����������ռ�����Ϊ���ռ䣬���ǵ��͵����
//����רҵCNC������������λ����λ�ںδ���ȡ��ע��
//����ǿ��Grblʼ�ս�����ԭ��������ԭ��λ�ã������ܿ��ط�����Ρ�
// #define HOMING_FORCE_SET_ORIGIN // Uncomment to enable.

// Grbl������ʱִ�еĿ�������Щ��洢��EEPROM�У����еĴ�С
//�͵�ַ��settings.h�ж��塣ʹ�õ�ǰ���ã���������2��������
//��˳��洢��ִ�С���Щ������ͨ����������g����
//������״̬ȡ�����û���ϲ�á�
#define N_STARTUP_LINE 2 // Integer (1-2)

//��GrblΪĳЩֵ���ʹ�ӡ�ĸ���С�������������Щ������
//ͨ�����ػ�������ʵ�ͳ�������ֵȷ�������磬λ��
//ֵ����С��0.001mm��0.0001in����Ϊ���������������ϸ���
//ȷ�е�˵ ���ԣ��п��ܲ���Ҫ�ı���Щ�����������Ҫ���������ԡ�
//ע�⣺�����Ǵ�0��?4������ֵ������4�����ܻ����������
#define N_DECIMAL_COORDVALUE_INCH 4 // �������Ӣ���λ��ֵ
#define N_DECIMAL_COORDVALUE_MM   3 // �������׵�λ��ֵ
#define N_DECIMAL_RATEVALUE_INCH  1 // ���ʻ��ٶ�ֵ��/����
#define N_DECIMAL_RATEVALUE_MM    0 // �ں����ʻ��ٶ�ֵ/����
#define N_DECIMAL_SETTINGVALUE    3 // С�������趨ֵ
#define N_DECIMAL_RPMVALUE        0 // ��ÿ����תRPMֵ

//������Ļ����������������ӵ�һ�������λ���أ�����Ҫ����
//������� ������������������һ�����ţ����Grbl�޷��ֱ�
//������һ�� ���ѡ��ֻӰ���λ����������ƣ�Grbl��
//������ǿ���û��ֶ��ѿ���λ���ء������������һ��
//Ϊÿ�������ƿ��أ��벻Ҫ���ô�ѡ�ͨ�����ֽ��ã�������ִ��һ��
//��λ�����ϵĻ���ѭ�����������ƶ�������
// #define LIMITS_TWO_SWITCHES_ON_AXES

//����GRBL���ٺͱ���gcode�кš���������ζ�żƻ�������
//��16��15��Ϊplan_block_t�ṹ�еĸ����к������ڳ��ռ�
// #define USE_LINE_NUMBERS // Ĭ��������ѽ��á�ȡ��ע�������á�

//��̽�����ڳɹ��󣬴�ѡ�������ṩ̽ͷ����ķ���
//ͨ���Զ����ɵ���Ϣ��������ã��û���Ȼ���Է������һ��̽��
//ͨ��Grbl'$��'��ӡ�����������ꡣ
#define MESSAGE_PROBE_COORDINATES // Ĭ����������á�ע�ͽ��á�

//ͨ��Arduino Uno�ϵ�������ȴ��g-code����M7���õڶ�����ȴҺ������
//ģ������4.�����Ҫ�ڶ�����ȴҺ�������ţ���ֻ��ʹ�ø�ѡ�
//ע�⣺������Σ�ģ������3�ϵ�M8������ȴҺ����������Ȼ������
// #define ENABLE_M7 // Disabled by default. Uncomment to enable.

//��ѡ��ʹ��������������Ϊ��ȫ�ſ��ء�һ����ȫ�ű�����ʱ��
//����ǿ�ƽ��ϣ�Ȼ��ȫ���жϻ����ĵ�Դ���ָ�����ֹ��ֱ��
//��ȫ�����½Ӻϡ������������Grbl����������������Ȼ�����¿���
//��ǰ�ĵ���·��������ʲô��û����һ����
// #define ENABLE_SAFETY_DOOR_INPUT_PIN // Default disabled. Uncomment to enable.

//��ȫ�ſ����л����ָ��󣬴����ý����üӵ��ӳ�
//�ڻָ��������ȴҺ֮��ָ�ѭ����
#define SAFETY_DOOR_SPINDLE_DELAY 4.0 // Float (seconds)
#define SAFETY_DOOR_COOLANT_DELAY 1.0 // Float (seconds)

//����CoreXY�˶�������CoreXY����һ��ʹ�á�
//��Ҫ��ʾ����������˻�ԭ�㣬����뽫����Ķ���ѭ��#define��������Ϊ
// #define HOMING_CYCLE_0��1 << X_AXIS����#define HOMING_CYCLE_1��1 << Y_AXIS��
//ע�⣺������ѡ�X���Y����˶�����Ϊ����ԭ��
//�����ڣ�http://corexy.com/theory.html�����ٶ��綯���Ķ�λ��������ȫ��ͬ
//������������ǣ��������ܻ�������ֵķ����ƶ���Grbl��ҪCoreXY A��B���
//�ڲ�ÿ���׾�����ͬ�Ĳ���
// #define COREXY // Default disabled. Uncomment to enable.

//�������뷴ת�����������ŵ������߼������������ζ�������ʹ��
//ָ�������ϵĳ��տ��أ�������Ĭ�ϵĳ������ء�
//ע�⣺�������ѡ����κͷ�ת���п������š��ײ�ѡ����һ������
//ֻ��ת�����������ţ���ȫ�ź͸�λ���й�����λ���壬��μ�cpu_map.h��
// #define INVERT_CONTROL_PIN_MASK CONTROL_MASK // Default disabled. Uncomment to disable.
// #define INVERT_CONTROL_PIN_MASK ((1<<CONTROL_SAFETY_DOOR_BIT)|(CONTROL_RESET_BIT)) // Default disabled.

//�����������뷴תѡ����������״̬�����Ӱ�����е��������Ź��ܣ�
//��Ӳ���ƺ͹�λ�����ǣ��������巴ת�������ò�ͬ��
//�˹���ѡ�ֻ��ת�˴�������������ţ�Ȼ��ת��������
//����Ӧ�õ����е��ˡ����û��������������Ż��ʹ��ʱ�����Ƿǳ����õ�
//�����ϰ�װ�˳�����NO���ͳ��գ�NC�����ء�
//ע�⣺�벻Ҫʹ���������������һ����Ҫ���������
// #define INVERT_LIMIT_PIN_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)) // Default disabled. Uncomment to enable.

//������ʹ�����Ŵӵ͵�ƽ��Ч/�ߵ�ƽ��Чת��Ϊ�͵�ƽ��Ч/�ߵ�ƽ��Ч������
//����һЩԤ�Ƶ��Ӱ塣
//ע�⣺���VARIABLE_SPINDLE�����ã�Ĭ�ϣ������ѡ����PWM�����
//����ʹ�ܱ��ϲ���һ�����š��������Ҫ���ѡ�������ת��PWM��
//ȡ��ע�����������ѡ��USE_SPINDLE_DIR_AS_ENABLE_PIN��
// #define INVERT_SPINDLE_ENABLE_PIN // Default disabled. Uncomment to enable.

//��ѡ������ȴҺ���Ŵӵ͵�ƽ��Ч/�ߵ�ƽ��Чת��Ϊ�͵�ƽ��Ч/�ߵ�ƽ��Ч������
//����һЩԤ�Ƶ��Ӱ塣
// #define INVERT_COOLANT_FLOOD_PIN // Default disabled. Uncomment to enable.
// #define INVERT_COOLANT_MIST_PIN // Default disabled. Note: Enable M7 mist coolant in config.h

//��Grblͨ�������Arduino��λ��ťӲ����ʱ��Grbl����ʱ���ᱨ��
//Ĭ������� ����Ϊ�������û������ܼ򵥵ؿ�ʼʹ��Grbl���鳲ʱ
//���ò����û��Ѿ���װ��λ���أ�Grbl����ALARM״̬��������ָʾ
// Grbl��֪������λ�ò��ڼ���֮ǰǿ���û��ؼҡ����ѡ��ǿ��
// Grbl���ǳ�ʼ�����뱨��״̬�����۹鳲������ѡ�����
// OEM��LinuxCNC�û�ϲ�������Դѭ����Ϊ��
// #define FORCE_INITIALIZATION_ALARM // Default disabled. Uncomment to enable.

//���ϵ��λʱ��Grbl�������λ����״̬��ȷ�����ǲ����ڻ״̬
//��ʼ��֮ǰ �������⵽һ�����⣬Ӳ�������ñ����ã�Grbl��
//�򵥵�֪ͨ�û�������Ʋ����뱨��״̬�������ǿ��С�Grbl���
//������������Ϣ
#define CHECK_LIMITS_AT_INIT

// ---------------------------------------------------------------------------------------
//�߼�����ѡ�

//�������ڵ���Ŀ�ĵĴ��롣���ʺ�һ��ʹ�ã���ʼ�ձ��ֲ��䡣
// #define DEBUG //ȡ��ע�������á�Ĭ�Ͻ��á�

//���ÿ��٣����������Ḳ�����á���Щֵ���������ֵ����Сֵ
//����ĸ���ֵ��ÿ�������յ��Ĵֵ���ϸ������
//ע��ÿ���������������е�����ֵ��
#define DEFAULT_FEED_OVERRIDE           100 // 100������Ҫ�ı����ֵ��
#define MAX_FEED_RATE_OVERRIDE          200 // �ٷֱȱ�̽������ʣ�100-255���ġ�ͨ��120����200��
#define MIN_FEED_RATE_OVERRIDE           10 // �ٷֱȱ�̽������ʣ�1-100���ġ�ͨ��50����1��
#define FEED_OVERRIDE_COARSE_INCREMENT   10 // (1-99). ͨ�� 10%.
#define FEED_OVERRIDE_FINE_INCREMENT      1 // (1-99). ͨ�� 1%.

#define DEFAULT_RAPID_OVERRIDE  100 // 100%. ��Ҫ�ı����ֵ��
#define RAPID_OVERRIDE_MEDIUM    50 // Percent of rapid (1-99). Usually 50%.
#define RAPID_OVERRIDE_LOW       25 // Percent of rapid (1-99). Usually 25%.
// #define RAPID_OVERRIDE_EXTRA_LOW 5 // *NOT SUPPORTED* Percent of rapid (1-99). Usually 5%.

#define DEFAULT_SPINDLE_SPEED_OVERRIDE    100 // 100%. ��Ҫ�ı����ֵ��
#define MAX_SPINDLE_SPEED_OVERRIDE        200 // �ٷֱȱ�̵������ٶȣ�100-255���ġ�ͨ����200����
#define MIN_SPINDLE_SPEED_OVERRIDE         10 // �ٷֱȱ�̵�����ת�٣�1-100���ġ�ͨ����10����
#define SPINDLE_OVERRIDE_COARSE_INCREMENT  10 // (1-99). Usually 10%.
#define SPINDLE_OVERRIDE_FINE_INCREMENT     1 // (1-99). Usually 1%.

//ִ��M2��M30�����������ʱ���󲿷�g����״̬���ָ�ΪĬ��ֵ��
//�˱���ʱ��ѡ������ָ����������ٺ������ٶȸ���ֵ
//���������ʱ��Ĭ��ֵ��
#define RESTORE_OVERRIDES_AFTER_PROGRAM_END // Default enabled. Comment to disable.

// Grbl v1.1��״̬���淢���仯��ͬʱҲɾ���˽���/���ô󲿷����ݵĹ���
//�����е��ֶ� ���GUI������Ա������һЩ���⣬���Ǳ������ü�������
//�����á��±�����ʽ��Ч������������������ֶ�
//����û��Ǳ�ڵ��������⡣
//ע�⣺�����ѡ��ֻ���ṩ��һ�ַ���������ĳЩ�����ֶΣ����Ψһ��
//�����Ҫ������Ҫע��GUI������������Щ���ݡ�������ã�����ܲ����ݡ�
#define REPORT_FIELD_BUFFER_STATE // Default enabled. Comment to disable.
#define REPORT_FIELD_PIN_STATE // Default enabled. Comment to disable.
#define REPORT_FIELD_CURRENT_FEED_SPEED // Default enabled. Comment to disable.
#define REPORT_FIELD_WORK_COORD_OFFSET // Default enabled. Comment to disable.
#define REPORT_FIELD_OVERRIDES // Default enabled. Comment to disable.
#define REPORT_FIELD_LINE_NUMBERS // Default enabled. Comment to disable.

//һЩ״̬�������ݲ���ʵʱ��Ҫ�ģ�ֻ�Ǽ�Ъ�Եģ���Ϊ��Щֵû��
//�����ı� ���º�����֮ǰ��Ҫ����״̬����Ĵ���
//���������ݱ�ˢ�²�������״̬�����С����ǣ������Щ��ֵ֮һ
//���ģ�Grbl���Զ�����Щ���ݰ�������һ��״̬�����У���������ʲô
//�����ڵ�ʱ �������ڼ������Ƶ�ʱ����йص�ͨ�ſ���
//�ͻ�������ý�塣����һ����æ�Ϳ��е�ˢ�¼���������Grbl����
//����û�����κ���Ҫ������ʱ��Ƶ����ˢ�¡��úõ�GUI��������ݲ���Ҫ
//����ˢ�£���Լ�����ӡ�
//ע�⣺WCOˢ�±�����2�����OVRˢ�±�����1�����
#define REPORT_OVR_REFRESH_BUSY_COUNT 20  // (1-255)
#define REPORT_OVR_REFRESH_IDLE_COUNT 10  // (1-255) Must be less than or equal to the busy count
#define REPORT_WCO_REFRESH_BUSY_COUNT 30  // (2-255)
#define REPORT_WCO_REFRESH_IDLE_COUNT 10  // (2-255) Must be less than or equal to the busy count

//���ٹ�����ϵͳ��ʱ��ֱ��� ����Խ��Խƽ��
//���ٶȣ��ر����������ٶȷǳ��ߵĻ����ϣ������ܻ��и���Ӱ��
//Ӱ������ �����������ȷֵ�ǻ�����صģ����Խ���
//������Ҫ��������߶ȡ����³ɹ���ֵ���Դ�50��200����ࡣ
//ע�⣺���Ĵ�ֵҲ����Ĳ��λ������жε�ִ��ʱ�䡣
//���������ֵ��ʱ������洢�ڻ������е���ʱ�����٣���֮��Ȼ��ʹ
//ȷ�������λ���������/������Ӧ����Щ�仯��
#define ACCELERATION_TICKS_PER_SECOND 100

//����Ӧ���Ჽƽ����AMASS����һ���Ƚ��Ĺ��ܣ�����������ζ��ʲô��
//ƽ�������˶��Ĳ������˹����ر����ڵ�̨����ƽ���˶�
//����10kHz��Ƶ�ʣ������˶���֮��Ļ�����ܵ��·���
//����������Ļ������������͵Ĳ�Ƶ��AMASS��Ӧ���ṩ���õ�
//��ƽ�����й�AMASSϵͳ�����ĸ���ϸ�ڣ������stepper.c��
#define ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING  // Ĭ�����á� ע�ͽ��á���Default enabled. Comment to disable.

//��������д��Grbl���õ���󲽽����ʡ���ѡ�����ô���
//�������ģ���Է�ֹ���������Ƶ�����ֵ�����ֵ
//���������ϸ���CPU�ٶȵ����ƣ����AVR��������������������ı�
//ʹ��16MHz��
//ע�⣺���ڽ��ã��������ռ����������á�
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

//����Ӧ�õ��߳��Ȳ������ᡣ��������ʼ��������ƽ��
//���߳��򸺷����ѡ���ᡣ���仰˵��һ��������
//�ӵ�ǰλ�ü�ȥ���߳��Ȳ���ֵ��
#define TOOL_LENGTH_OFFSET_AXIS Z_AXIS // Ĭ��z�ᡣ��Чֵ��X_AXIS��Y_AXIS��Z_AXIS��

//Ϊ��ͬ��RPMֵ���ÿɱ����������ѹ����Arduino Uno�ϣ�����
//�������Ž����5V�����ת��Ϊ256���м��ƽΪ0V������ʱΪ0V��
//ע�⣺����Arduino Unos��Ҫ����ʹ��ʱ��Z��������D11������ʹ������D12�л���
// D11�����ϵ�Ӳ��PWM������ڿɱ����������ѹ�Ǳ���ġ�
#define VARIABLE_SPINDLE // /Ĭ�����á�ע�ͽ��á�

//���ɿɱ��������ʹ�á���ʹ��ʱ���⽫ǿ��PWM����ﵽ��Сռ�ձȡ�
//�����ᱻ��ֹʱ��PWM�����Խ���ȡ0V��������û�����Ҫ���ѡ�����
//��ĳЩ����������������õġ�����СPWM����������ת����Сֵһ��
//���ã������ת�ٵ����PWM���������Ҫ����0V֮��Ľϴ��ѹ����Ƿǳ������
//��СPWM�����õĵ�ѹ����Сת�١������ֵ��ÿ��PWMֵ0.02V�����ԣ�ʲôʱ��
//��СPWMΪ1��ֻ��0.02���ֿ����úͽ��á���PWM 5ʱ���⽫��0.1V������
//��ס��������������СPWMֵ��������ʼʧȥPWM�ֱ���
//���ܹ�255��PWM��ƽ��Χ���н�С�ķ�Χ����ʾ��ͬ�������ٶȡ�
//ע�⣺ͨ�����µ�ʽ������СPWM��ռ�ձ�:(ռ�ձȣ���=��SPINDLE_PWM_MIN_VALUE / 255��* 100
// #define SPINDLE_PWM_MIN_VALUE 5 // Default disabled. Uncomment to enable. Must be greater than zero. Integer (1-255).

//Ĭ������£���һ��328p��Uno���ϣ�Grbl���ɱ�����PWM��ʹ���źźϲ���һ�����������ṩ����
//����I / O���š�����ĳЩ���ã���Щ������Ҫ���������š��������ѡ��ʹ��
//���᷽�����ţ�D13����Ϊ����������ʹ�����ţ��Լ�����D11�ϵ������ٶ�PWM��
//ע�⣺������ѡ�������������VARIABLE_SPINDLE��328p��������Uno����
//ע�⣺���û�з������ţ�M4�����������������ָʾ��M3������
//ע�⣺��ע�⣡�ϵ�ʱ��Arduino����װ�س����л�D13���š��������Grbl����
//һ������Ա�������ʹ��һ�����õ�Arduino��Ϊ��Arduino as ISP��������������������ߣ�
//���D13 LED�л�Ӧ����ʧ�����ǻ�û�в��Թ�������뱨�����ȥ��
// #define USE_SPINDLE_DIR_AS_ENABLE_PIN // Default disabled. Uncomment to enable.

//ʹ��USE_SPINDLE_DIR_AS_ENABLE_PINѡ���������ʹ�����ŵ���Ϊ��Ĭ�ϣ�
//��������ٶ�Ϊ����M3 / 4��Ч��Grbl���������ʹ�����ţ����Ի�����PWM
//���Ϊ�� �������û�֪�������Ƿ��ڻ״̬���������������ӿ���
//���롣���ǣ���ĳЩʹ������£��û�����ϣ��ʹ���������������ٶȽ���
//����ת�ٴ�����ʱ�������á����ѡ����ԡ�
//ע�⣺��Ҫ����USE_SPINDLE_DIR_AS_ENABLE_PIN��
// #define SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED // Default disabled. Uncomment to enable.

//���ô˹��ܺ�Grbl���������յ����Ѿ�Ԥ�������еĻ��ԣ��ո�
//ɾ������д��ĸ��û�����ۣ�����������Grblִ�С���������
//����һ�л��������������Ӧ�÷��͸�Grbl��������ͨ��·�����磬���һ���û�
//����'g1 x1.032 y2.45��test comment��'��һ�У�Grbl����'[echo��G1X1.032Y2.45]'����ʽ���ԡ�
//ע�⣺�����ڵ���Ŀ�ģ�����ʱ����ռ�ñ������Դ������������
//���ܡ����������������Ҫ�ģ�����д�뻺����Ӧ�ô������
//��������޶ȵؼ��ٴ���д��Э���ڵĴ���ȴ���
// #define REPORT_ECHO_LINE_RECEIVED // Default disabled. Uncomment to enable.

//��С�滮ʦ·���ٶ� ���üƻ��߼ƻ��ڵ�Ĭ����С���ӵ��ٶ�
//ÿ�����������㣬����ʼ�մӻ�������ʼ�˺ͽ�β��ʼ
//�㡣��ֵ���ƻ���ͨ�����ӵ��ƶ����ٶȣ��������Ǽ��ٶ�
//���ڿ����ƶ�����֮������ƻ�Ƕȡ�����ڲ��ܵĻ���������
//����һ���ӵĹ��ߣ���3D��ӡ���򼤹��и�������ʹ�ã����ֵ
//��Ӧ�������ﵽ���������������Сֵ��
#define MINIMUM_JUNCTION_SPEED 0.0f // (mm/min)

//���ù滮���������С�����ٶȡ����������κ�ֵ����������Ϊ��Сֵ
//ֵ����Ҳȷ����һ���ƻ��Ķ���������ɲ������κθ�����
//������󡣾��ܲ��Ƽ���������1.0����/���ӵ�ֵ���ܻ��С
//������Ҳ��0.1mm / min���������ĳɹ�������������ض�������ͬ��
#define MINIMUM_FEED_RATE 1.0 // (mm/min)

//�ھ�ȷ�Ļ��߹켣֮ǰͨ��С�Ƕȱƽ��Ļ����ɵ�������
//�ð����sin������cos��������У�������������������ܻ����
//�Ƿ���е绡���ɵ�׼ȷ�����⣬��������绡ִ�����ڻ���������
//����̫���trig���������������
#define N_ARC_CORRECTION 12 // Integer (1-255)

// G2 / 3 g�����׼�Ķ�����������ġ����ڰ뾶�Ļ����п��µ���ֵ
//��Բ��pi����ȫԲ��2 * pi������ʱ�Ĵ��󡣻���ƫ�ƵĻ���׼ȷ
//������ȫԲ��2 * pi��ʱ��Ȼ�����⡣��Ϊ�����ʻ�����
//������ƫ������Բ��������Ϊ������Բ��ʱ��ָ�����⣬��������Ϊ�ǳ�
//������ֵ����;�ȷ�����⣬����Լ�����ţ�1.2e-7rad����С����
//�������ֵ���û�����epsilon cutoff��ȷ�����Ƿ���һ��ȫԲ��
//ע�⣺������ֵʱҪ�ǳ�С�ġ�Ӧ�����Ǵ���1.2e-7��������̫��
//ԶԶ������� Ĭ������Ӧ�ò���󲿷֣��������ȫ���Ļ���ȫ����������
#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7 // Float (radians)

//����ͣ�ڼ�ִ����ʱ������Ĭ��ֵ����Ϊ50ms�����ṩ��
//��Լ55���ӵ����ʱ���ӳ٣����ڴ����Ӧ�ó�����˵�´����ࡣ����
//����ӳٻ������������ͣ��ʱ�䣬��Ҳ�ή����Ӧ��
//����ʱ�����ִ�У�����״̬����һ������Ϊ��Щ����ÿ��ͣ��֮��ִ�е�
//ʱ�䲽�� ���⣬���ס��Arduino��ʱ��ʱ�����ڳ�ʱ����ӳٲ��Ǻ�׼ȷ��
#define DWELL_TIME_STEP 50 // Integer (1-255) (milliseconds)

//ͨ���������ڷ����������ú���Ӧ�Ĳ�������֮�䴴��һ���ӳ�
//��һ���жϣ�Timer2�Ƚϣ�������������Ҫ��Grbl�жϣ�Timer1�Ƚϣ�
//���÷������ţ������������ò������ţ���Ϊ����
//�������С�Timer2�ȽϽ��������������ò���֮��Ĳ�������
//�����ӳ�ʱ�䣬Timer2�������ɲ������壬���������ӳ�
//�ɲ�������ʱ����ϲ��������ӳ١�����лlangwadt���뷨����
//ע�⣺ȡ��ע�������á��Ƽ����ӳ�ʱ�����> 3us�������������
//�û��ṩ�Ĳ�����ʱ�䣬��ʱ�䲻�ó���127us������ɹ�
//ĳЩ���õ�ֵ��Χ��5��20us��
// #define STEP_PULSE_DELAY 10 // Step pulse delay in microseconds. Default disabled.

//���κθ���ʱ��ƻ��Ĺ滮�������е������˶���������ãã
// Grblʹ�õĴ󲿷��ڴ涼���������������С��ֻ���ж����ʱ�������
//���õ�RAM���������±���Mega2560һ�����������Arduino��ʼ����
//����ȱ�����õ��ڴ�����CPU�޷����ϼƻ�������
//ִ��ʱ�µĴ��붯����
// #define BLOCK_BUFFER_SIZE 16 // Uncomment to override default in planner.h.

//���Ʋ���ִ���㷨֮����м䲽�λ������Ĵ�С
//�ͼƻ��߿顣ÿһ�ζ�����һ���㶨���ٶ�ִ�е�
//��ACCELERATION_TICKS_PER_SECOND����Ĺ̶�ʱ�䡣���������������
//���ٶ���������ȷ��׷�١�����������Ĵ�С�����˶��ٲ�
//ִ����ǰ�ڻ���������Grbl������Ҫ�����ִ�����ǵ�����
//�ڷ��ز��������û�����֮ǰ����ǰ�ڲ���Լ50����ʱ�ƶ���
// #define SEGMENT_BUFFER_SIZE 6 // Uncomment to override default in stepper.h.

//Ҫִ�еĴ����������е��л�������С�����⣬����Ĵ�С
//ÿ�������飬��Ϊ���Ƕ��������ִ�С���ַ����洢�ġ�ȷ��
//��settings.h��for��Ϊ�Ѷ�����ڴ��ַ��¼���õ�EEPROM
//���������������
//ע�⣺����������⣬80���ַ��������⣬���л�������С
//����̫С��g�������ܻᱻ�ضϡ�g�����׼��ʽ
//���֧��256���ַ� �ڽ����İ汾�У����Ĭ��ֵ������
//����֪�����ǿ�������Ͷ����ٶ�����ڴ�ռ䡣
// #define LINE_BUFFER_SIZE 80  // Uncomment to override default in protocol.h

//���з��ͺͽ��ջ�������С�����ջ�����ͨ��������һ����
//�������洢Grbl����Ĵ���顣������
//�ӿڽ��ַ�����������ÿ���鷢�͸�ÿ�������Ӧ�����ԣ�
//���ӽ��ջ������������Ҫ����Ľ��ջ�����������ý��Ϳ���
//�������� ���ͻ�������Ҫ����Grbl�е���Ϣ��ֻ�����ӣ������
//������Ϣ��Grbl��ʼͣ�ͣ��ȴ�����ʣ�����Ϣ��
//ע�⣺Grbl��0.5�������Ҳ���һ��ƽ��״̬���棬���Ǵ���TX����
// 115200���ؽ���Ҫ5����������һ�����͵�55���ַ��ı��档����Ĳ���������
//��Լ90-100���ַ� ֻҪ����TX����������������Grbl
//��������Ч���С���TX��������С����Ϊ����������Ĵ�С��
#if !defined (STM32F103C8)
// #define RX_BUFFER_SIZE 128 // (1-254) Uncomment to override defaults in serial.h
// #define TX_BUFFER_SIZE 100 // (1-254)
#endif

//Ӳ��λ���صļ����ȥ�����ܡ�������ʱ���ж�
//���Ӳ��λ�������Ž�ʹArduino�Ŀ��Ź���ʱ�����¼��
//�ӳ�Լ32��������������״̬������԰������ػ���
//������Ĵ��󴥷����ǵ�Ӳ��λ���أ���������������
//�����ⲿ�źŵ��µĵ������š���������
//ʹ�ô����β�������źŵ��£��ɵ�USB /���������
//�������ã��۸���ˣ���������ͨ��·���ӵ�ÿ����λ���š�
// #define ENABLE_SOFTWARE_DEBOUNCE // Default disabled. Uncomment to enable.

//��Grbl�ļ��ģʽ�£���̽��ѭ��֮������λ�á����ü���
//̽��Ŀ���λ�ã�����ʱ��λ������Ϊ��ʼλ�á�
// #define SET_CHECK_MODE_PROBE_TO_START // Default disabled. Uncomment to enable.

//ǿ��Grbl�ڴ�������⵽����ʱ���Ӳ��λ���ص�״̬
//�ı�Ӳ���Ƶ�ISR���̡�Ĭ������£�Grbl�ᴥ��Ӳ����
//�κ����Ÿı�ʱ���ᱨ������Ϊ�������ؿ��ܵ���������״̬���
//����˱��� ��Ӳ���Ʊ�����ʱ������Ӧ����100���ɿ��ģ������
//ԭ�������ѡ��Ĭ���ǽ��õġ�ֻ�е����ϵͳ/���Ӳ�Ʒ���Ա�֤
//���ز����������ǽ������ô�ѡ��⽫�����ڷ�ֹ
//���������뿪��ʱ����Ӳ���ơ�
//ע�⣺�������SOFTWARE_DEBOUNCE�����ѡ������á�
// #define HARD_LIMIT_FORCE_STATE_CHECK // Default disabled. Uncomment to enable.

//������λ������������λ��������Щ��Grblʹ�õĳ���
//��λѭ����ȷ����λ������ÿ����λ�϶���ռ�ú����
//ѭ���������׶�ʹ��������г�����ʱ��SEARCH_SCALAR��
//ȷ�������Բ�����λ���ء�һ���ҵ�����λ�׶ο�ʼ
//ʹ��LOCATE_SCALAR�Ĺ�λ���������������������������
//��λ����
//ע�⣺������ֵ���������1.0��ȷ����ȷ�Ĺ��ܡ�
// #define HOMING_AXIS_SEARCH_SCALAR  1.5f // ȡ��ע���Ը���limits.c�е�Ĭ��ֵ��
// #define HOMING_AXIS_LOCATE_SCALAR  10.0f // ȡ��ע���Ը���limits.c�е�Ĭ��ֵ��

//����'$ RST = *'��'$ RST = $'��'$ RST =��'eeprom restore����е������
//��Щ��������ǲ���ȡ�ġ�ֻ��ע������ĺ��Խ�������
//ע�⣺�����SETTINGS_RESTORE_ALL��������`$ RST = *`���
#define ENABLE_RESTORE_EEPROM_WIPE_ALL         // '$RST=*' Ĭ�����á�ע�ͽ���.
#define ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS // '$RST=$' Ĭ�����á�ע�ͽ���.
#define ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS // '$RST=#' Ĭ�����á�ע�ͽ���.

//���������ð汾���ĺ�`$ RST = *`����ʱ�ָ���EEPROM���ݡ�ÿ��
// Grbl�汾֮������û�����EEPROM���ݽṹ�ᷢ���仯��Grbl���Զ�����
//�������ָ�EEPROM������������ʲô���ݱ������ͻָ����������
//�ر�����Ҫ����ĳЩ���ݵ�OEM�����磬BUILD_INFO�ַ�������
//ͨ��������.INO��ͼд��Arduino EEPROM�԰�����Ʒ���ݡ��ı���һ��
//�겻�ָܻ�������ϢEEPROM��ȷ���ڹ̼�����������Щ���ݡ�
//ע�⣺ȡ��ע���Ը���settings.h�е�Ĭ��ֵ
// #define SETTINGS_RESTORE_ALL (SETTINGS_RESTORE_DEFAULTS | SETTINGS_RESTORE_PARAMETERS | SETTINGS_RESTORE_STARTUP_LINES | SETTINGS_RESTORE_BUILD_INFO)

//����'$ I =��string��'build info write���������ã��κ����еĹ�����Ϣ���ݶ�����
//ͨ��������ЧУ���ֵ���ⲿװ�ý�������EEPROM�С������ѡ�������
//���ڷ�ֹ�û�����OEM����ʱ��Щ���ݱ����ǡ�
//ע�⣺������ò�ȷ��Grbl��Զ���ܸı乹����Ϣ�У�����Ҫ����
//�����SETTING_RESTORE_ALL�꣬�����������Ƴ�SETTINGS_RESTORE_BUILD_INFO��
//ע�⣺����ĸ�����grblWrite_BuildInfo.inoʾ���ļ���������д���ַ�����
#define ENABLE_BUILD_INFO_WRITE_COMMAND // '$I=' Default enabled. Comment to disable.

// AVR������Ҫ����EEPROMд���ڼ��ֹ�����жϡ����������
//������ISR�ʹ���ͨ��ISR�������ʱ��д��EEPROM�����ISR��ͣ����
//������������ʧȥλ�ã����ڽ������ݶ�ʧ���������
//ѡ��ǿ�Ƽƻ�����������ȫ��գ�ֻҪд��EEPROM����
//�κζ�ʧ����Ļ���
//Ȼ�����Ⲣ���ܷ�ֹEEPROMд���ڼ䶪ʧ���н������ݵ����⣬�ر���
//���һ��GUIԤ��ͬʱ��䴮�н��ջ����������Ƿǳ������
//��Щg���루G10��G28.1��G30.1����ͼ���û������ڰ���һ��������ǵȴ���ok��
//���͸�������������������֮ǰ����Щ����֮һ��
//ע�⣺�����EEPROMд��������ҵ�ڼ䱻��ʽ����������'$'�����Ȼ����
//���꼯g�������G10��G28 / 30.1�����ǣ���Ϊ�����ǻ����һ����
//���� Ŀǰ�����ѡ��ֻ��ǿ�ƹ滮��������Щg-code����ͬ����
#define FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE // Default enabled. Comment to disable.

//��Grbl v0.9�Լ�֮ǰ�İ汾�У���һ���Ϲ��ֿ߲ⱨ���bug
//����������ִ�е����ݲ���أ���Ϊ`WPos��`�ǻ���g-code������״̬��
//�����ں��漸����������ѡ��ǿ�ƹ滮����������գ�ͬ����ֹͣ
//ֻҪ�иı乤������ƫ����������G10��G43.1��G92��G54-59`���˶���
//����ȷ����WPos����������ȷ����򵥵ķ��������˵��ǣ����Ƿǳ�������
//ʹ����Щ�����е��κ�һ������Ҫͨ�����ǽ����������˶���
#define FORCE_BUFFER_SYNC_DURING_WCO_CHANGE // Default enabled. Comment to disable.

//Ĭ������£�Grbl��������G38.x̽��ѭ������Ľ������ʸ��ǡ���Ȼ����
//������ĳЩ����������Ʋ�ͬ������˵��Ӧ���������ġ�
//�����̽ͷ�������������ͬ�̶ȵĴ�����ȡ�����ٶȡ�ͨ��
//��̽��ѭ�����������̵Ľ������ʣ�̽ͷ������Ӧ�ø���
//���ظ� �����Ҫ������ͨ��ȡ��ע������Ķ��������ô���Ϊ��
// #define ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES // Default disabled. Uncomment to enable.

//�ڰ�ȫ��״̬�����ú�����ͣ���˶���������Ҫ���OEM
//ϣ������������������ǵļ��ɻ�����Ŀǰ��Grbl��Ϊ
//ͣ������ֻ�漰һ���ᣬ����ͣ��ʵʩ�Ѿ�д��
//ͨ���ı�ͣ��λ�����Ժ����׵�Ϊ�κ������Ĳ�ͬ���ϵ��˶��ع�
//Դ���롣��ʱ��Grblֻ֧��ͣ��һ���ᣨͨ����Z�ᣩ
//�ڻ�λʱ���������ƶ����ڻָ�λ��ʱ�򸺷����ƶ���
//�˶��Ի��������������˶����رյ�Դ�Ϳ���ͣ����ִ�С�
//�ָ����ָ�λ�ø�����Щ�����˶��෴�����ٻָ���
//����λ�ã�ͨ�糬ʱ��Ȼ��ص�ԭ����λ��
//��������ȡ��
//ע�⣺��Ȼ�ڽ����С�����������������и���ռ���
//���ܺ�HOMING_FORCE_SET_ORIGINһ��ʹ�á�ͣ���˶�Ҳֻ�ܽ���
//����ķ���
// #define PARKING_ENABLE  // Default disabled. Uncomment to enable

//����ͣ���˶���ѡ�������ã���
#define PARKING_AXIS Z_AXIS // Define which axis that performs the parking motion
#define PARKING_TARGET -5.0f // Parking axis target. In mm, as machine coordinate [-max_travel,0].
#define PARKING_RATE 500.0f // Parking fast rate after pull-out in mm/min.
#define PARKING_PULLOUT_RATE 100.0f // Pull-out/plunge slow feed rate in mm/min.
#define PARKING_PULLOUT_INCREMENT 5.0f // Spindle pull-out and plunge distance in mm. Incremental distance.
                                      // Must be positive value or equal to zero.

//����һ�������M�������������úͽ���ͣ���˶���
//��Щ��M56��M56 P1��M56 Px���ƣ�M56 P0Ϊ��ֹ��
//��������ģʽ���ģ����ڹ滮��ͬ�������á���������g���룬����
//��g-code����ͬ��ִ�С��ⲻ��һ��ʵʱ�����
//ע�⣺PARKING_ENABLE�Ǳ���ġ�Ĭ������£�M56�ڳ�ʼ��ʱ���ڻ״̬��ʹ��
// DEACTIVATE_PARKING_UPON_INIT to set M56 P0 as the power-up default.
// #define ENABLE_PARKING_OVERRIDE_CONTROL   // Default disabled. Uncomment to enable
// #define DEACTIVATE_PARKING_UPON_INIT // Default disabled. Uncomment to enable.

//ͨ����������ֹͣ����ѡ��ڽ��������ڼ��Զ����ü�����
//ֹͣ���������ǡ����ǣ���Ҳ��ζ�ż�����Ȼ����
//�����Ҫ��������ֹͣ���ǣ������������á�����ȫ��һ����ȫ����
//ȷ��������ֹͣʱ���������б���ͨ��״̬���������֡�
#define DISABLE_LASER_DURING_HOLD // Default enabled. Comment to disable.

//��������PWM /�ٶ�����ķֶ�����ģ�͡���Ҫһ���������
// repo��/ doc / script�ļ����е�'fit_nonlinear_spindle.py'�ű���������ļ�����
//��������ռ��������ݲ����нű������ɽ��������
// #define ENABLE_PIECEWISE_LINEAR_SPINDLE  // Default disabled. Uncomment to enable.

// N_PIECES��RPM_MAX��RPM_MIN��RPM_POINTxx��RPM_LINE_XX����ȫ�����ò���
//�ڡ�fit_nonlinear_spindle.py���ű��Ľ������������ENABLE_PIECEWISE_LINEAR_SPINDLEʱʹ��
//�����á�ȷ������ֵ��ű����������ȫ��ͬ��
//ע�⣺��N_PIECES <4ʱ��δʹ�õ�RPM_LINE��RPM_POINT���岻��Ҫ��ʡ�ԡ�
#define N_PIECES 4  // ������1-4�����ű����������ʹ�õķֶ�������
#define RPM_MAX  11686.4  // ģ�͵����RPM��$ 30> RPM_MAX��������ΪRPM_MAX��
#define RPM_MIN  202.5    // ģ�͵���СRPM��$ 31 <RPM_MIN��������ΪRPM_MIN��
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
