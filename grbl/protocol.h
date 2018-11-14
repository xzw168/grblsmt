/*
  protocol.h - controls Grbl execution protocol and procedures
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

#ifndef protocol_h
#define protocol_h

// ��Ҫִ�еĴ����������е��л�������С��
// ע�⣺���˼������֮�⣬û�����⣬�����л������Ĵ�С����̫С�ˡ�
// ��G�������Խضϡ���ʽ�Ĵ����׼��֧�ָߴ�256
// �ַ������Ժ�İ汾�У��⽫���ӣ�������֪���ж��ٶ����
// �ڴ�ռ����Ͷ�ʵ���������±�дG���������û�����
// ��������
#ifndef LINE_BUFFER_SIZE
  #define LINE_BUFFER_SIZE 80
#endif

// ��ʼ��ѭ���ķ��������������Դ��ж˿ڵ����д����ַ���ִ�С�
// ���������ʱ������������ɳ�ʼ�����̡�
void protocol_main_loop();

// ��������ĸ���ֹͣ���鲢ִ��ʵʱ���
void protocol_execute_realtime();
void protocol_exec_rt_system();

//������ã���ִ���Զ�ѭ�����ܡ�
void protocol_auto_cycle_start();

// ����ֱ��ִ�����л��岽�衣
void protocol_buffer_synchronize();

#endif
