#
# gdb helper commands and functions for Linux kernel debugging
#
#  Kernel Target Awareness Layer
#
# Copyright (c) 2016 Linaro Ltd
#
# Authors:
#  Kieran Bingham <kieran.bingham@linaro.org>
#
# This work is licensed under the terms of the GNU GPL version 2.
#

import gdb
import linux
from linux import constants
from linux import utils
from linux import tasks
from linux import lists
from linux import cpus
from linux import radixtree


class LxAwarenessTarget(gdb.Target):
    """ Provide a Linux Awareness Target Layer

This will provide hooks for our Thread implementation, and later
extra architecture specific target layers to handle memory """

    def __init__(self):
        gdb.write("\nLxAwarenessTarget: __init__()\n\n")
        self.shortname = "LxThreads"
        self.longname = "Linux Thread Integration Layer"
        super(LxAwarenessTarget, self).__init__("Kernel Thread Awareness")
        gdb.write("self.name = {}\n".format(self.name))

    def to_thread_name(self, gdbthread):
        return None  # Our tasks are presented by pid_to_str()

    def to_pid_to_str(self, pid):
        # This would be better as a new pid_t object type rather than tuple
        task = tasks.get_task_by_pid(pid[2])
        if task is not None:
            if task['mm']:  # Userspace process
                name = task['comm'].string()
            else:           # Kernel Thread
                name = "[{}]".format(task['comm'].string())
            return name
        else:
            return "NotFound"

    def to_thread_extra_info(self, gdbthread):
        return "LinuxExtra"

    def to_update_thread_list(self):
        gdb.write("LX.to_update_thread_list\n")
        inferior = gdb.selected_inferior()
        threads = inferior.threads()

        for task in tasks.task_lists():
            # Build ptid_t ... class object better here still
            ptid = (inferior.pid, 0, task['pid'])  # (pid, lwp, tid)
            if ptid not in threads:
                gdb.write("- New Task [{} {}]\n"
                          .format(task['pid'], task['comm'].string()))
                inferior.add_thread(ptid)


def load():
    LxAwarenessTarget()
