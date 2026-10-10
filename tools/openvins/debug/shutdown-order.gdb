set pagination off
set confirm off
set print elements 10
set print thread-events off
set debuginfod enabled off
set breakpoint pending on
handle SIGINT nostop noprint pass
break eprosima::fastdds::dds::DomainParticipantFactory::~DomainParticipantFactory()
commands
silent
printf "ORDER: DDS_FACTORY_DESTRUCTOR\n"
bt 8
continue
end
break ov_msckf::ROS2Visualizer::~ROS2Visualizer
commands
silent
printf "ORDER: VISUALIZER_DESTRUCTOR\n"
bt 8
continue
end
start
python
import os, signal, threading
inferior_pid = gdb.selected_inferior().pid
threading.Timer(5.0, lambda: os.kill(inferior_pid, signal.SIGINT)).start()
end
continue
python
if gdb.selected_inferior().pid:
    gdb.execute('info threads')
    gdb.execute('bt full')
    gdb.execute('thread apply all bt full')
end
