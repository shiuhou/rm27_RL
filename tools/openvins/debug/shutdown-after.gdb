set pagination off
set confirm off
set print elements 20
set print thread-events off
set debuginfod enabled off
set breakpoint pending on
handle SIGINT nostop noprint pass
start
break ov_msckf::ROS2Visualizer::~ROS2Visualizer
python
import os, signal, threading
inferior_pid = gdb.selected_inferior().pid
threading.Timer(5.0, lambda: os.kill(inferior_pid, signal.SIGINT)).start()
end
continue
bt 15
thread apply all bt 8
disable breakpoints
continue
