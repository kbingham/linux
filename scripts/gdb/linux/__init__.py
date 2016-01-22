# Initialisation of Kernel Awareness in GDB.

import gdb


class LxAutostart():
    def __init__(self):
        gdb.write("Linux Kernel Debugger Loading\n")
        gdb.write("Detected the following kernel vesion\n\n")
        gdb.write(gdb.parse_and_eval("linux_banner").string())
        gdb.write("\n\n")

LxAutostart()
