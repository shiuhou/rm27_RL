set pagination off
set confirm off
set print elements 20
set print thread-events off
set debuginfod enabled off
handle SIGINT nostop noprint pass
start
python
import os, signal, threading
inferior_pid = gdb.selected_inferior().pid
threading.Timer(5.0, lambda: os.kill(inferior_pid, signal.SIGINT)).start()
end
continue
info threads
bt full
thread apply all bt full
info sharedlibrary
