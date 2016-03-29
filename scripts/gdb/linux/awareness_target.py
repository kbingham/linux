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


ulong_type = gdb.lookup_type('unsigned long')

def setup_thread_amd64(thread, task):
    #rip = gdb.lookup_minimal_symbol("thread_return").value()

    rsp = task['thread']['sp'].cast(ulong_type.pointer())
    rbp = rsp.dereference().cast(ulong_type.pointer())
    rip = 0xdeadbeef
    rbx = (rbp - 1).dereference()
    r12 = (rbp - 2).dereference()
    r13 = (rbp - 3).dereference()
    r14 = (rbp - 4).dereference()
    r15 = (rbp - 5).dereference()
    # The two pushes that don't have CFI info
#    rsp += 2
#    ex = in_exception_stack(rsp)
#    if ex:
#    print "EXCEPTION STACK: pid %d" % task['pid']
    thread.registers['rsp'].value = rsp
    thread.registers['rbp'].value = rbp
    thread.registers['rip'].value = rip
    thread.registers['rbx'].value = rbx
    thread.registers['r12'].value = r12
    thread.registers['r13'].value = r13
    thread.registers['r14'].value = r14
    thread.registers['r15'].value = r15
    thread.registers['cs'].value = 2*8
    thread.registers['ss'].value = 3*8


def setup_thread_armv7(thread, task):
    thread_info = tasks.get_thread_info(task)
    cpu_ctx = thread_info['cpu_context']

    thread.registers['r4'].value = cpu_ctx['r4']
    thread.registers['r5'].value = cpu_ctx['r5']
    thread.registers['r6'].value = cpu_ctx['r6']
    thread.registers['r7'].value = cpu_ctx['r7']
    thread.registers['r8'].value = cpu_ctx['r8']
    thread.registers['r9'].value = cpu_ctx['r9']

    thread.registers['r10'].value = cpu_ctx['sl']
    thread.registers['r11'].value = cpu_ctx['fp']

    thread.registers['sp'].value = cpu_ctx['sp']  # r13
    thread.registers['pc'].value = cpu_ctx['pc']  # r15


setup_threads = {
    'i386:x86-64': setup_thread_amd64,
    'arm': setup_thread_armv7,
}


class LxAwarenessTarget(gdb.Target):
    """ Provide a Linux Awareness Target Layer

This will provide hooks for our Thread implementation, and later
extra architecture specific target layers to handle memory """

    def __init__(self):
        self.arch = gdb.newest_frame().architecture().name()
        self.setup_threads = setup_threads.get(self.arch, None)
        if self.setup_threads is None:
            gdb.write("No architecture layer found. Disabling LxAwareness\n")
            return  # Don't Initialise without an arch layer

        gdb.write("\nLxAwarenessTarget: ({}):\n\n".format(self.arch))
        self.shortname = "LxThreads"
        self.longname = "Linux Thread Integration Layer"
        super(LxAwarenessTarget, self).__init__("Kernel Thread Awareness")
        # self.setup_tasks()

    def setup_tasks(self):
        self.pid_to_task_struct = {}

        for task in tasks.task_lists():
            inferior = gdb.selected_inferior()
            thread = inferior.new_thread((inferior.pid, task['pid'], 0), task)
            thread.name = task['comm'].string()
            # Setup Threads must have been set by the init
            self.setup_threads(thread, task)

        # gdb.selected_inferior().executing = False

    def to_thread_name(self, gdbthread):
        return "" # None  # Our tasks are presented by pid_to_str()

    def to_pid_to_str(self, pid):
        # This would be better as a new pid_t object type rather than tuple
        task = tasks.get_task_by_pid(pid[1])
        if task is not None:
            if task['mm']:  # Userspace process
                name = task['comm'].string()
            else:           # Kernel Thread
                name = "[{}]".format(task['comm'].string())
            return name
        else:
            return "NotFound"

    def to_extra_thread_info(self, gdbthread):
        return "LinuxExtra"

    def to_update_thread_list_simple(self):
        self.setup_tasks()

    def to_update_thread_list(self):
        gdb.write("LX.to_update_thread_list\n")
        inferior = gdb.selected_inferior()
        threads = inferior.threads()

        for task in tasks.task_lists():
            # Build ptid_t ... class object better here still
            ptid = [inferior.pid, int(task['pid']), 0]  # (pid, lwp, tid)
            print ("PTID = {}".format(ptid))
            for t in threads:
                print (str(t))

            #ptid = (1, 0, task['pid'])  # (pid, lwp, tid)
            if ptid not in threads:
                thread = inferior.new_thread(ptid, task)
                thread.name = task['comm'].string()
                # Setup Threads must have been set by the init
                self.setup_threads(thread, task)

    def to_update_thread_list_pass(self):
        pass

    def to_thread_alive(self, ptid):
        return 1

    def to_fetch_registers(self, register):
        thread = gdb.selected_thread()
        # setup_thread_amd64(thread, thread.info)
        # setup_thread_armv7(thread, thread.info)
        self.setup_threads(thread, thread.info)
        # return True

    def to_prepare_to_store(self, thread):
        pass

    # We don't need to store anything; The regcache is already written.
    def to_store_registers(self, thread):
        pass

# We need to postpone adding our target until after the inferior is added
# Perhaps some sort of a hook, or observer registration which calls into
# this python layer is needed!
# Then in the same way as the C version there should be some checks to see if
# we are happy to load this target layer in! (Compare vmlinux-banner somehow)
def load():
    LxAwarenessTarget()
