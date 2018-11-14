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

// 从要执行的串行输入流中的行缓冲区大小。
// 注意：除了极端情况之外，没有问题，但是行缓冲区的大小可能太小了。
// 和G代码块可以截断。正式的代码标准，支持高达256
// 字符。在以后的版本中，这将增加，当我们知道有多少额外的
// 内存空间可以投资到这里或重新编写G代码解释器没有这个
// 缓冲区。
#ifndef LINE_BUFFER_SIZE
  #define LINE_BUFFER_SIZE 80
#endif

// 开始主循环的方法。它处理来自串行端口的所有传入字符并执行。
// 当它们完成时。它还负责完成初始化过程。
void protocol_main_loop();

// 在主程序的各个停止点检查并执行实时命令。
void protocol_execute_realtime();
void protocol_exec_rt_system();

//如果启用，则执行自动循环功能。
void protocol_auto_cycle_start();

// 阻塞直到执行所有缓冲步骤。
void protocol_buffer_synchronize();

#endif
