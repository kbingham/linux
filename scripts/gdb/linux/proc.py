#
# gdb helper commands and functions for Linux kernel debugging
#
#  Kernel proc information reader
#
# Copyright (c) 2016 Linaro Ltd
#
# Authors:
#  Kieran Bingham <kieran.bingham@linaro.org>
#
# This work is licensed under the terms of the GNU GPL version 2.
#

import gdb
from linux import constants
from linux import utils
from linux import tasks
from linux import lists
from linux import cpus
from linux import radixtree


class LxCmdLine(gdb.Command):
    """ Report the Linux Commandline used in the current kernel.
        Equivalent to cat /proc/cmdline on a running target"""

    def __init__(self):
        super(LxCmdLine, self).__init__("lx-cmdline", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        gdb.write(gdb.parse_and_eval("saved_command_line").string() + "\n")

LxCmdLine()


class LxVersion(gdb.Command):
    """ Report the Linux Version of the current kernel.
        Equivalent to cat /proc/version on a running target"""

    def __init__(self):
        super(LxVersion, self).__init__("lx-version", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        # linux_banner should contain a newline
        gdb.write(gdb.parse_and_eval("linux_banner").string())

LxVersion()


# Resource Structure Printers
#  /proc/iomem
#  /proc/ioports

def get_resources(resource, depth):
    while resource:
        yield resource, depth

        child = resource['child']
        if child:
            for res, deep in get_resources(child, depth + 1):
                yield res, deep

        resource = resource['sibling']


def show_lx_resources(resource_str):
        resource = gdb.parse_and_eval(resource_str)
        width = 4 if resource['end'] < 0x10000 else 8
        # Iterate straight to the first child
        for res, depth in get_resources(resource['child'], 0):
            start = int(res['start'])
            end = int(res['end'])
            gdb.write(" " * depth * 2 +
                      "{0:0{1}x}-".format(start, width) +
                      "{0:0{1}x} : ".format(end, width) +
                      res['name'].string() + "\n")


class LxIOMem(gdb.Command):
    """Identify the IO memory resource locations defined by the kernel

Equivalent to cat /proc/iomem on a running target"""

    def __init__(self):
        super(LxIOMem, self).__init__("lx-iomem", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        return show_lx_resources("iomem_resource")

LxIOMem()


class LxIOPorts(gdb.Command):
    """Identify the IO port resource locations defined by the kernel

Equivalent to cat /proc/ioports on a running target"""

    def __init__(self):
        super(LxIOPorts, self).__init__("lx-ioports", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        return show_lx_resources("ioport_resource")

LxIOPorts()


# Mount namespace viewer
#  /proc/mounts

def info_opts(lst, opt):
    opts = ""
    for key, string in lst.items():
        if opt & key:
            opts += string
    return opts


FS_INFO = {constants.LX_MS_SYNCHRONOUS: ",sync",
           constants.LX_MS_MANDLOCK: ",mand",
           constants.LX_MS_DIRSYNC: ",dirsync",
           constants.LX_MS_NOATIME: ",noatime",
           constants.LX_MS_NODIRATIME: ",nodiratime"}

MNT_INFO = {constants.LX_MNT_NOSUID: ",nosuid",
            constants.LX_MNT_NODEV: ",nodev",
            constants.LX_MNT_NOEXEC: ",noexec",
            constants.LX_MNT_NOATIME: ",noatime",
            constants.LX_MNT_NODIRATIME: ",nodiratime",
            constants.LX_MNT_RELATIME: ",relatime"}

mount_type = utils.CachedType("struct mount")
mount_ptr_type = mount_type.get_type().pointer()


class LxMounts(gdb.Command):
    """Report the VFS mounts of the current process namespace.

Equivalent to cat /proc/mounts on a running target
An integer value can be supplied to display the mount
values of that process namespace"""

    def __init__(self):
        super(LxMounts, self).__init__("lx-mounts", gdb.COMMAND_DATA)

    # Equivalent to proc_namespace.c:show_vfsmnt
    # However, that has the ability to call into s_op functions
    # whereas we cannot and must make do with the information we can obtain.
    def invoke(self, arg, from_tty):
        argv = gdb.string_to_argv(arg)
        if len(argv) >= 1:
            try:
                pid = int(argv[0])
            except:
                raise gdb.GdbError("Provide a PID as integer value")
        else:
            pid = 1

        task = tasks.get_task_by_pid(pid)
        if not task:
            raise gdb.GdbError("Couldn't find a process with PID {}"
                               .format(pid))

        namespace = task['nsproxy']['mnt_ns']
        if not namespace:
            raise gdb.GdbError("No namespace for current process")

        for vfs in lists.list_for_each_entry(namespace['list'],
                                             mount_ptr_type, "mnt_list"):
            devname = vfs['mnt_devname'].string()
            devname = devname if devname else "none"

            pathname = ""
            parent = vfs
            while True:
                mntpoint = parent['mnt_mountpoint']
                pathname = utils.dentry_name(mntpoint) + pathname
                if (parent == parent['mnt_parent']):
                    break
                parent = parent['mnt_parent']

            if (pathname == ""):
                pathname = "/"

            superblock = vfs['mnt']['mnt_sb']
            fstype = superblock['s_type']['name'].string()
            s_flags = int(superblock['s_flags'])
            m_flags = int(vfs['mnt']['mnt_flags'])
            rd = "ro" if (s_flags & constants.LX_MS_RDONLY) else "rw"

            gdb.write(
                "{} {} {} {}{}{} 0 0\n"
                .format(devname,
                        pathname,
                        fstype,
                        rd,
                        info_opts(FS_INFO, s_flags),
                        info_opts(MNT_INFO, m_flags)))

LxMounts()


bdev_type = utils.CachedType("struct block_device")
bdev_ptr_type = bdev_type.get_type().pointer()


class LxMeminfo(gdb.Command):
    """ Identify the memory usage, statistics, and availability

Equivalent to cat /proc/meminfo on a running target """

    def __init__(self):
        super(LxMeminfo, self).__init__("lx-meminfo", gdb.COMMAND_DATA)

    def K(self, val):
        # Convert from PAGES to KB
        return int(val << (constants.LX_PAGE_SHIFT - 10))

    def page_K(self, remote_value):
        # Obtain page value, and Convert from PAGES to KB
        val = int(gdb.parse_and_eval(remote_value))
        return self.K(val)

    def gps(self, enum_zone_stat_item):
        # Access the Global Page State structure
        # I would prefer to read this structure in one go and then index
        # from the enum. But we can't determine the enum values with out
        # a call to GDB anyway so we may as well take the easy route and
        # get the value.
        remote_value = "vm_stat[" + enum_zone_stat_item + "].counter"
        return int(gdb.parse_and_eval(remote_value))

    def gps_K(self, enum_zone_stat_item):
        return self.K(self.gps(enum_zone_stat_item))

    def nr_blockdev_pages(self):
        bdevs_head = gdb.parse_and_eval("all_bdevs")
        pages = 0
        for bdev in lists.list_for_each_entry(bdevs_head,
                                              bdev_ptr_type,
                                              "bd_list"):
            try:
                pages += bdev['bd_inode']['i_mapping']['nrpages']
            except:
                # Any memory read failures are simply not counted
                pass
        return pages

    def total_swapcache_pages(self):
        pages = 0
        if not constants.LX_CONFIG_SWAP:
            return 0

        for i in range(0, int(constants.LX_MAX_SWAPFILES)):
            swap_space = "swapper_spaces[" + str(i) + "].nrpages"
            pages += int(gdb.parse_and_eval(swap_space))
        return pages

    def for_each_hstate(self):
        hugetlb_max_hstate = gdb.parse_and_eval("hugetlb_max_hstate")
        for i in range(0, hugetlb_max_hstate):
            yield gdb.parse_and_eval("hstates[" + str(i) + "]")

    def hugetlb_total_pages(self):
        if not constants.LX_CONFIG_HUGETLB_PAGE:
            return 0
        pages = 0
        for h in self.for_each_hstate():
            pages += h['nr_huge_pages'] * (1 << h['order'])
        return pages

    def vm_commit_limit(self, totalram_pages):
        total_swap_pages = 0
        overcommit = int(gdb.parse_and_eval("sysctl_overcommit_kbytes"))
        overcommit_ratio = int(gdb.parse_and_eval("sysctl_overcommit_ratio"))

        if constants.LX_CONFIG_SWAP:
            total_swap_pages = int(gdb.parse_and_eval("total_swap_pages"))

        hugetlb_total_pages = self.hugetlb_total_pages()

        if overcommit:
            allowed = overcommit >> (constants.LX_PAGE_SHIFT - 10)
        else:
            allowed = ((totalram_pages - hugetlb_total_pages *
                       overcommit_ratio / 100))

        allowed += total_swap_pages
        return allowed

    def quicklist_total_size(self):
        count = 0
        quicklist = utils.gdb_eval_or_none("quicklist")
        if quicklist is None:
            return 0

        for cpu in cpus.each_online_cpu():
            ql = cpus.per_cpu(quicklist, cpu)
            for q in range(0, constants.LX_CONFIG_NR_QUICK):
                count += ql[q]['nr_pages']

        return count

    # Main lx-meminfo command execution
    # See fs/proc/meminfo.c:meminfo_proc_show()
    def invoke(self, arg, from_tty):
        totalram = int(gdb.parse_and_eval("totalram_pages"))
        freeram = self.gps("NR_FREE_PAGES")
        reclaimable = self.gps("NR_SLAB_RECLAIMABLE")
        unreclaimable = self.gps("NR_SLAB_UNRECLAIMABLE")
        slab = reclaimable + unreclaimable
        # for_each_zone(zone)
        #     wmark_low += zone->watermark[WMARK_LOW];
        wmark_low = 0   # Zone parsing is unimplemented

        available = freeram - wmark_low
        available += reclaimable - min(reclaimable / 2, wmark_low)

        bufferram = self.nr_blockdev_pages()
        swapcached = self.total_swapcache_pages()

        file_pages = self.gps("NR_FILE_PAGES")
        cached = file_pages - swapcached - bufferram

        # LRU Pages
        active_pages_anon = self.gps("NR_ACTIVE_ANON")
        inactive_pages_anon = self.gps("NR_INACTIVE_ANON")
        active_pages_file = self.gps("NR_ACTIVE_FILE")
        inactive_pages_file = self.gps("NR_INACTIVE_FILE")
        unevictable_pages = self.gps("NR_UNEVICTABLE")
        active_pages = active_pages_anon + active_pages_file
        inactive_pages = inactive_pages_anon + inactive_pages_file

        kernelstack = int(self.gps("NR_KERNEL_STACK") *
                          constants.LX_THREAD_SIZE / 1024)

        commitlimit = int(self.vm_commit_limit(totalram))
        committed_as = int(gdb.parse_and_eval("vm_committed_as.count"))

        vmalloc_total = int(constants.LX_VMALLOC_TOTAL >> 10)

        gdb.write(
            "MemTotal:       {:8d} kB\n".format(self.K(totalram)) +
            "MemFree:        {:8d} kB\n".format(self.K(freeram)) +
            "MemAvailable:*  {:8d} kB\n".format(self.K(available)) +
            "Buffers:        {:8d} kB\n".format(self.K(bufferram)) +
            "Cached:         {:8d} kB\n".format(self.K(cached)) +
            "SwapCached:     {:8d} kB\n".format(self.K(swapcached)) +
            "Active:         {:8d} kB\n".format(self.K(active_pages)) +
            "Inactive:       {:8d} kB\n".format(self.K(inactive_pages)) +
            "Active(anon):   {:8d} kB\n".format(self.K(active_pages_anon)) +
            "Inactive(anon): {:8d} kB\n".format(self.K(inactive_pages_anon)) +
            "Active(file):   {:8d} kB\n".format(self.K(active_pages_file)) +
            "Inactive(file): {:8d} kB\n".format(self.K(inactive_pages_file)) +
            "Unevictable:    {:8d} kB\n".format(self.K(unevictable_pages)) +
            "Mlocked:        {:8d} kB\n".format(self.gps_K("NR_MLOCK"))
            )

        if constants.LX_CONFIG_HIGHMEM:
            totalhigh = int(gdb.parse_and_eval("totalhigh_pages"))
            # This really gets in the way.
            # Implementing free_highpages locally needs to walk all mm_zones
            freehigh = 0 # int(gdb.parse_and_eval("nr_free_highpages()"))
            lowtotal = totalram - totalhigh
            lowfree = freeram - freehigh
            gdb.write(
                "HighTotal:      {:8d} kB\n".format(self.K(totalhigh)) +
                "HighFree:*      {:8d} kB\n".format(self.K(freehigh)) +
                "LowTotal:       {:8d} kB\n".format(self.K(lowtotal)) +
                "LowFree:*       {:8d} kB\n".format(self.K(lowfree))
                )

        if not constants.LX_CONFIG_MMU:
            mmap_pg_alloc = gdb.parse_and_eval("mmap_pages_allocated.counter")
            gdb.write(
                "MmapCopy:       {:8d} kB\n".format(self.K(mmap_pg_alloc))
                )

        gdb.write(
            "SwapTotal:      {:8d} kB\n".format(self.K(0)) +
            "SwapFree:       {:8d} kB\n".format(self.K(0)) +
            "Dirty:          {:8d} kB\n".format(self.gps_K("NR_FILE_DIRTY")) +
            "Writeback:      {:8d} kB\n".format(self.gps_K("NR_WRITEBACK")) +
            "AnonPages:      {:8d} kB\n".format(self.gps_K("NR_ANON_PAGES")) +
            "Mapped:         {:8d} kB\n".format(self.gps_K("NR_FILE_MAPPED")) +
            "Shmem:          {:8d} kB\n".format(self.gps_K("NR_SHMEM")) +
            "Slab:           {:8d} kB\n".format(self.K(slab)) +
            "SReclaimable:   {:8d} kB\n".format(self.K(reclaimable)) +
            "SUnreclaim:     {:8d} kB\n".format(self.K(unreclaimable)) +
            "KernelStack:    {:8d} kB\n".format(kernelstack) +
            "PageTables:     {:8d} kB\n".format(self.gps_K("NR_PAGETABLE"))
            )

        if constants.LX_CONFIG_QUICKLIST:
            quicklist = self.quicklist_total_size()
            gdb.write(
                "Quicklists:     {:8d} kB\n".format(self.K(quicklist))
                )

        nfsunstable = self.gps("NR_UNSTABLE_NFS")
        nr_bounce = self.gps("NR_BOUNCE")
        writebacktmp = self.gps("NR_WRITEBACK_TEMP")
        gdb.write(
            "NFS_Unstable:   {:8d} kB\n".format(self.K(nfsunstable)) +
            "Bounce:         {:8d} kB\n".format(self.K(nr_bounce)) +
            "WritebackTmp:   {:8d} kB\n".format(self.K(writebacktmp))
            )

        gdb.write(
            "CommitLimit:    {:8d} kB\n".format(self.K(commitlimit)) +
            "Committed_AS:   {:8d} kB\n".format(self.K(committed_as)) +
            "VmallocTotal:   {:8d} kB\n".format(vmalloc_total)
            )

        # These are always zero now
        gdb.write(
            "VmallocUsed:    {:8d} kB\n".format(0) +
            "VmallocChunk:   {:8d} kB\n".format(0)
            )

        if constants.LX_CONFIG_MEMORY_FAILURE:
            gdb.write(
                "HardwareCorrupted: {:8d} kB\n"
                )

        if constants.LX_CONFIG_TRANSPARENT_HUGEPAGE:
            huge = self.gps("NR_ANON_TRANSPARENT_HUGEPAGES")
            # HPAGE_PMD_NR can not be determined in constants.py
            gdb.write(
                "AnonHugePages:* {:8d} kB ( * HPAGE_PMD_NR )\n"
                .format(self.K(huge))
                )
        if constants.LX_CONFIG_CMA:
            totalcma_pages = int(gdb.parse_and_eval("totalcma_pages"))
            cmafree = self.gps("NR_FREE_CMA_PAGES")
            gdb.write(
                "CmaTotal:       {:8d} kB\n".format(self.K(totalcma_pages)) +
                "CmaFree:        {:8d} kB\n".format(self.K(cmafree))
                )

        if constants.LX_CONFIG_HUGETLB_PAGE:
            h = gdb.parse_and_eval("hstates[default_hstate_idx]")
            nr_huge_pages = int(h['nr_huge_pages'])
            free_huge_pages = int(h['free_huge_pages'])
            resv_huge_pages = int(h['resv_huge_pages'])
            surplus_huge_pages = int(h['surplus_huge_pages'])
            huge_page_shift = int(h['order'] + constants.LX_PAGE_SHIFT - 10)
            huge_page_size = int(1 << huge_page_shift)
            gdb.write(
                "HugePages_Total:   {:5d}\n".format(nr_huge_pages) +
                "HugePages_Free:    {:5d}\n".format(free_huge_pages) +
                "HugePages_Rsvd:    {:5d}\n".format(resv_huge_pages) +
                "HugePages_Surp:    {:5d}\n".format(surplus_huge_pages) +
                "Hugepagesize:   {:8d} kB\n".format(huge_page_size)
                )

        gdb.write("* items may not be based on full information\n")

LxMeminfo()

irq_desc = None
irq_desc_type = utils.CachedType("struct irq_desc")


class LxInterrupts(gdb.Command):
    """ Report the interrupt counters of each CPU.
        Equivalent to cat /proc/interrupts on a running target """

    def __init__(self):
        super(LxInterrupts, self).__init__("lx-interrupts", gdb.COMMAND_DATA)

    def irq_to_desc(self, irq):
        if constants.LX_CONFIG_SPARSE_IRQ:
            irq_desc_tree = gdb.parse_and_eval("irq_desc_tree")
            irq_desc_addr = radixtree.lookup(irq_desc_tree, irq)

            if irq_desc_addr is None:
                return None

            if irq_desc_addr.address:
                return irq_desc_addr.cast(irq_desc_type.get_type())
            return None
        else:
            irq_desc_array = "irq_desc[{}]".format(irq)
            return gdb.parse_and_eval(irq_desc_array)

    def kstat_irqs_cpu(self, irq_desc, cpu):
        return int(cpus.per_cpu(irq_desc['kstat_irqs'], cpu))

    def invoke(self, arg, from_tty):
        nr_irqs = int(gdb.parse_and_eval("nr_irqs"))

        # Calculate the width of the first column
        prec = 3
        j = nr_irqs
        while (j <= nr_irqs):
            j *= 10
            prec += 1

        title = "{:{}}".format(" ", prec+8)
        for cpu in cpus.each_online_cpu():
            title += "CPU{:<8d}".format(cpu)
        gdb.write(title + "\n")

        for irq in range(0, nr_irqs):
            desc = self.irq_to_desc(irq)
            if desc is None:
                continue

            anycount = 0
            for cpu in cpus.each_online_cpu():
                anycount += self.kstat_irqs_cpu(desc, cpu)
            if (anycount == 0):
                continue

            irq_line = "{:{}d}: ".format(irq, prec)
            for cpu in cpus.each_online_cpu():
                count = self.kstat_irqs_cpu(desc, cpu)
                irq_line += "{:10d} ".format(count)
            gdb.write(irq_line + "\n")

LxInterrupts()
