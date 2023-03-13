const std = @import("std");
const os = std.os;
const linux = os.linux;
const kvm = @import("kvm.zig");
const serial = @import("serial.zig");
const loader = @import("loader.zig");
const boot_protocol = @import("boot_protocol.zig");
const log = @import("log.zig");
const pci = @import("pci.zig");
const io = @import("io.zig");
const virtio = @import("virtio.zig");
const console = @import("console.zig");

pub const VM = struct {
    kvmfd: i32 = 0,
    vmfd: i32 = 0,
    vcpufd: i32 = 0,

    initrd: []const u8,
    bzImage: []const u8,
    disk_img: []const u8,

    mem: []u8 = undefined,
    mem_addr: u64 = 0,
    serial: serial.Serial = undefined,

    allocator: std.mem.Allocator = undefined,
    con: console.Console = console.Console.init(),
    net: *virtio.VirtioNet = undefined,
    pci_devices: pci.PCI = .{},
    io_manager: io.IoManager = .{},

    const bootParamAddr = 0x10000;
    const cmdlineAddr = 0x20000;
    const kernelAddr = 0x100000;
    const initrdAddr = 0xf000000;
    const Self = @This();
    const Error = error{
        MMapFailed,
        IoctlFailed,
        UnsupportedAPIVersion,
        UnavailableFeature,
    };

    pub fn init(bzImage: []const u8, initrd: []const u8, disk_img: []const u8, mem_size: usize, allocator: std.mem.Allocator) !Self {
        const mem_addr = linux.mmap(null, mem_size, linux.PROT.READ | linux.PROT.WRITE, linux.MAP.SHARED | linux.MAP.ANONYMOUS, -1, 0);
        if (mem_addr == 0) {
            return Error.MMapFailed;
        }
        var mem = @intToPtr([]u8, mem_addr);
        mem.len = mem_size;

        return .{
            .initrd = initrd,
            .bzImage = bzImage,
            .disk_img = disk_img,
            .mem = mem,
            .mem_addr = mem_addr,
            .allocator = allocator,
        };
    }

    pub fn create(self: *Self) !void {
        self.kvmfd = try os.open("/dev/kvm", os.O.RDWR | os.FD_CLOEXEC, 0);

        // Check needed features are available.
        try self.checkCapability();

        // Create VM.
        self.vmfd = @intCast(os.fd_t, linux.ioctl(self.kvmfd, kvm.KVM_CREATE_VM, 0));
        if (self.vmfd == -1) {
            log.errExt(@src(), "failed to create VM", .{});
            return Error.IoctlFailed;
        }
        self.serial = serial.Serial.init(self.vmfd, &self.con);

        const initrd_buf = try loader.loadFile(self.initrd, self.allocator);
        defer self.allocator.free(initrd_buf);
        std.mem.copy(u8, self.mem[initrdAddr..], initrd_buf);

        var bzImage_mem = try loader.loadFile(self.bzImage, self.allocator);
        defer self.allocator.free(bzImage_mem);

        // https://kernel.googlesource.com/pub/scm/linux/kernel/git/penberg/linux/+/kvmtool/next/tools/kvm/Documentation/virtio-console.txt
        // dyndbg=\"file drivers/char/virtio_console.c +plf ; file drivers/virtio/virtio_ring.c +plf\"
        //const cmdline = "earlyprintk=serial root=/dev/ram rw console=ttyS0 rdinit=/sbin/init notsc virtio_pci.force_legacy=1";
        const cmdline = "earlyprintk=serial console=hvc0 root=/dev/vda rw";
        std.mem.copy(u8, self.mem[cmdlineAddr..], cmdline);
        self.mem[cmdlineAddr + cmdline.len] = 0;

        std.mem.copy(u8, self.mem[bootParamAddr..], bzImage_mem[0..boot_protocol.KernelHeader.EndOffset]);
        var kernel_header = try boot_protocol.KernelHeader.fromBytes(self.mem[bootParamAddr..]);

        kernel_header.vid_mode = 0xFFFF;
        kernel_header.type_of_loader = 0xFF;
        kernel_header.ramdisk_image = initrdAddr;
        kernel_header.ramdisk_size = @intCast(u32, initrd_buf.len);
        kernel_header.loadflags |= boot_protocol.CanUseHeap | boot_protocol.LoadedHigh | boot_protocol.KeepSegments;
        kernel_header.heap_end_ptr = 0xfe00;
        kernel_header.ext_loader_ver = 0;
        kernel_header.cmd_line_ptr = cmdlineAddr;
        kernel_header.cmdline_size = cmdline.len + 1;

        std.mem.copy(u8, self.mem[kernelAddr..], bzImage_mem[(@intCast(u32, kernel_header.setup_sects) + 1) * 512 ..]);

        var e820_array = boot_protocol.E820Array.fromBytes(self.mem[bootParamAddr..]);
        e820_array.clear();
        e820_array.add(boot_protocol.RealModeIvtBegin, boot_protocol.EBDAStart - boot_protocol.RealModeIvtBegin, boot_protocol.E820Ram);
        e820_array.add(boot_protocol.EBDAStart, boot_protocol.VGARAMBegin - boot_protocol.EBDAStart, boot_protocol.E820Reserved);
        e820_array.add(boot_protocol.MBBIOSBegin, boot_protocol.MBBIOSEnd - boot_protocol.MBBIOSBegin, boot_protocol.E820Reserved);
        e820_array.add(kernelAddr, self.mem.len - kernelAddr, boot_protocol.E820Ram);

        try self.configureTSSAddr();
        try self.configureIdentityMapAddr();
        try self.createIRQChip();
        try self.createPIT2();
        try self.configureUserMemoryRegion();
        try self.createVCPU();
        try self.configureCPUID();
        try self.configureSRegs();
        try self.configureRegs();

        try self.io_manager.append(serial.COM1Addr, serial.COM1Addr + 7, .{ .s = &self.serial });
        try self.io_manager.append(pci.PCI.AddrPortAddr, pci.PCI.AddrPortAddr, .{ .p = &self.pci_devices });
        try self.io_manager.append(pci.PCI.DataPortAddr, pci.PCI.DataPortAddr + pci.PCI.DataPortNum - 1, .{ .p = &self.pci_devices });

        self.pci_devices.devices[1] = pci.Device{ .virtio_console = virtio.VirtioConsole.init(self.vmfd, self.mem_addr, &self.con) };
        try self.io_manager.append(virtio.VirtioConsole.IoPortStart, virtio.VirtioConsole.IoPortStart + virtio.VirtioConsole.IoPortSize - 1, .{ .virtio_console = &self.pci_devices.devices[1].virtio_console });

        self.pci_devices.devices[2] = pci.Device{ .virtio_net = virtio.VirtioNet.init(self.vmfd, self.mem_addr) };
        self.net = &self.pci_devices.devices[2].virtio_net;
        try self.io_manager.append(virtio.VirtioNet.IoPortStart, virtio.VirtioNet.IoPortStart + virtio.VirtioNet.IoPortSize - 1, .{ .virtio_net = self.net });
        try self.net.createTapDevice();
        _ = try std.ChildProcess.exec(.{
            .allocator = self.allocator,
            .argv = &[_][]const u8{ "ip", "a", "add", "192.168.200.1/24", "dev", "zig-vmm-test" },
            .max_output_bytes = 1000 * 1024,
        });
        _ = try std.ChildProcess.exec(.{
            .allocator = self.allocator,
            .argv = &[_][]const u8{ "ip", "link", "set", "up", "dev", "zig-vmm-test" },
            .max_output_bytes = 1000 * 1024,
        });

        if (self.disk_img.len != 0) {
            self.pci_devices.devices[3] = pci.Device{ .virtio_blk = try virtio.VirtioBlk.init(self.vmfd, self.mem_addr, self.disk_img) };
            try self.io_manager.append(virtio.VirtioBlk.IoPortStart, virtio.VirtioBlk.IoPortStart + virtio.VirtioBlk.IoPortSize - 1, .{ .virtio_blk = &self.pci_devices.devices[3].virtio_blk });
        }
    }

    fn handle_stdin(con: *console.Console, s: *serial.Serial, vc: *virtio.VirtioConsole) !void {
        // virtio-console is used for users' tty
        _ = s;
        while (true) {
            // TODO: some locks for con is required?
            const c = try con.readByte();
            _ = try vc.putc(c);
        }
    }

    pub fn run(self: *Self) !void {
        try self.con.startServer();

        const mmap_size = linux.ioctl(self.kvmfd, kvm.KVM_GET_VCPU_MMAP_SIZE, 0);
        if (mmap_size == -1) {
            log.errExt(@src(), "failed to ioctl KVM_GET_VCPU_MMAP_SIZE", .{});
            return;
        }

        const run_addr = linux.mmap(null, mmap_size, linux.PROT.READ | linux.PROT.WRITE, linux.MAP.SHARED, self.vcpufd, 0);
        if (run_addr == -1) {
            log.errExt(@src(), "failed to mmap VCPU", .{});
            return;
        }
        const r = @intToPtr(*kvm.kvm_run, run_addr);

        _ = try std.Thread.spawn(.{}, handle_stdin, .{ &self.con, &self.serial, &self.pci_devices.devices[1].virtio_console });

        while (true) {
            const ret = linux.ioctl(self.vcpufd, kvm.KVM_RUN, 0);
            if (ret == -1) {
                log.errExt(@src(), "failed to KVM_RUN", .{});
                return;
            }

            switch (r.exit_reason) {
                kvm.KVM_EXIT_HLT => {
                    log.infoExt(@src(), "KVM_EXIT_HLT", .{});
                    return;
                },
                kvm.KVM_EXIT_IO => {
                    //log.debug("KVM_EXIT_IO", .{});
                    const _io = r.vars.io;
                    var c = @intToPtr([]u8, (@ptrToInt(r) + _io.data_offset));
                    c.len = _io.size;
                    if (_io.port == 0x61 and _io.direction == kvm.KVM_EXIT_IO_IN) {
                        // https://blog.bobuhiro11.net/2021/03-03-gokvm2.html
                        c[0] = 0x20;
                    } else {
                        if (_io.direction == kvm.KVM_EXIT_IO_IN) {
                            try self.io_manager.in(_io.port, _io.size, c);
                        } else {
                            try self.io_manager.out(_io.port, _io.size, c);
                        }
                    }
                },
                kvm.KVM_EXIT_SHUTDOWN => {
                    log.warnExt(@src(), "KVM_EXIT_SHUTDOWN", .{});
                    return;
                },
                else => {
                    log.errExt(@src(), "unknown exit_reasion={d}", .{r.exit_reason});
                    return;
                },
            }
        }
    }

    fn checkCapability(self: Self) !void {
        var ret = linux.ioctl(self.kvmfd, kvm.KVM_GET_API_VERSION, 0);
        if (ret == -1) {
            log.errExt(@src(), "failed to ioctl KVM_GET_API_VERSION", .{});
            return Error.IoctlFailed;
        }
        if (ret != 12) {
            log.errExt(@src(), "unexpected KVM_API_VERSION expected={d} actual={d}", .{ 12, ret });
            return Error.UnsupportedAPIVersion;
        }
        log.info("KVM_GET_API_VERSION={d}", .{ret});

        ret = linux.ioctl(self.kvmfd, kvm.KVM_CHECK_EXTENSION, kvm.KVM_CAP_USER_MEMORY);
        if (ret == -1) {
            log.errExt(@src(), "failed to ioctl KVM_CHECK_EXTENSION", .{});
            return Error.UnavailableFeature;
        }
        if (ret == 0) {
            log.errExt(@src(), "extension KVM_CAP_USER_MEMORY is not available", .{});
            return Error.UnavailableFeature;
        }
        log.info("KVM_CAP_USER_MEMORY is available", .{});
    }

    fn configureTSSAddr(self: Self) !void {
        const ret = linux.ioctl(self.vmfd, kvm.KVM_SET_TSS_ADDR, 0xffffd000);
        if (ret != 0) {
            log.errExt(@src(), "failed to configure TSS address", .{});
            return Error.IoctlFailed;
        }
    }

    fn configureIdentityMapAddr(self: Self) !void {
        var mapAddr: u64 = 0xffffc000;
        const ret = linux.ioctl(self.vmfd, kvm.KVM_SET_IDENTITY_MAP_ADDR, @ptrToInt(&mapAddr));
        if (ret != 0) {
            log.errExt(@src(), "failed to configure Identity map address", .{});
            return Error.IoctlFailed;
        }
    }

    fn createIRQChip(self: Self) !void {
        const ret = linux.ioctl(self.vmfd, kvm.KVM_CREATE_IRQCHIP, 0);
        if (ret != 0) {
            log.errExt(@src(), "failed to create IRQCHIP", .{});
            return Error.IoctlFailed;
        }
    }

    fn createPIT2(self: Self) !void {
        const pit = kvm.kvm_pit_config{};
        const ret = linux.ioctl(self.vmfd, kvm.KVM_CREATE_PIT2, @ptrToInt(&pit));
        if (ret != 0) {
            log.errExt(@src(), "failed to create PIT2", .{});
            return;
        }
    }

    fn configureUserMemoryRegion(self: Self) !void {
        var region = kvm.kvm_userspace_memory_region{
            .slot = 0,
            .guest_phys_addr = 0x0,
            .memory_size = self.mem.len,
            .userspace_addr = self.mem_addr,
        };

        const ret = linux.ioctl(self.vmfd, kvm.KVM_SET_USER_MEMORY_REGION, @ptrToInt(&region));
        if (ret != 0) {
            log.errExt(@src(), "failed to ioctl KVM_SET_USER_MEMORY_REGION", .{});
            return Error.IoctlFailed;
        }
    }

    fn createVCPU(self: *Self) !void {
        self.vcpufd = @intCast(i32, linux.ioctl(self.vmfd, kvm.KVM_CREATE_VCPU, 0));
        if (self.vcpufd == -1) {
            log.errExt(@src(), "failed to ioctl KVM_CREATE_VCPU", .{});
            return Error.IoctlFailed;
        }
    }

    fn configureCPUID(self: Self) !void {
        var cpuid = kvm.kvm_cpuid2{};
        var ret = linux.ioctl(self.kvmfd, kvm.KVM_GET_SUPPORTED_CPUID, @ptrToInt(&cpuid));
        if (ret != 0) {
            log.errExt(@src(), "failed to ioctl KVM_GET_SUPPORTED_CPUID", .{});
            return Error.IoctlFailed;
        }
        var i: usize = 0;
        while (i < cpuid.nent) : (i += 1) {
            if (cpuid.entry[i].function != kvm.KVM_CPUID_SIGNATURE) {
                continue;
            }

            cpuid.entry[i].eax = kvm.KVM_CPUID_FEATURES;
            cpuid.entry[i].ebx = 0x4b4d564b; // KVMK
            cpuid.entry[i].ecx = 0x564b4d56; // VMKV
            cpuid.entry[i].edx = 0x4d; // M
        }

        ret = linux.ioctl(self.vcpufd, kvm.KVM_SET_CPUID2, @ptrToInt(&cpuid));
        if (ret != 0) {
            log.errExt(@src(), "failed to ioctl KVM_SET_CPUID2", .{});
            return Error.IoctlFailed;
        }
    }

    fn configureSRegs(self: Self) !void {
        var sregs: kvm.kvm_sregs = .{};

        // do not remove this.
        var c: u65 = 0;
        _ = c;

        var ret = linux.ioctl(self.vcpufd, kvm.KVM_GET_SREGS, @ptrToInt(&sregs));
        if (ret == -1) {
            log.errExt(@src(), "failed to KVM_GET_SREGS", .{});
            return Error.IoctlFailed;
        }
        sregs.cs.base = 0x0;
        sregs.cs.limit = 0xFFFFFFFF;
        sregs.cs.g = 1;
        sregs.ds.base = 0x0;
        sregs.ds.limit = 0xFFFFFFFF;
        sregs.ds.g = 1;
        sregs.fs.base = 0x0;
        sregs.fs.limit = 0xFFFFFFFF;
        sregs.fs.g = 1;
        sregs.gs.base = 0x0;
        sregs.gs.limit = 0xFFFFFFFF;
        sregs.gs.g = 1;
        sregs.es.base = 0x0;
        sregs.es.limit = 0xFFFFFFFF;
        sregs.es.g = 1;
        sregs.ss.base = 0x0;
        sregs.ss.limit = 0xFFFFFFFF;
        sregs.ss.g = 1;
        sregs.cs.db = 1;
        sregs.ss.db = 1;
        sregs.cr0 |= 1; // protected mode
        ret = linux.ioctl(self.vcpufd, kvm.KVM_SET_SREGS, @ptrToInt(&sregs));
        if (ret == -1) {
            log.errExt(@src(), "failed to KVM_SET_SREGS", .{});
            return Error.IoctlFailed;
        }
    }

    fn configureRegs(self: Self) !void {
        var regs: kvm.kvm_regs = .{};
        var ret = linux.ioctl(self.vcpufd, kvm.KVM_GET_REGS, @ptrToInt(&regs));
        regs.rip = 0x100000;
        regs.rsi = 0x10000;
        regs.rflags = 0x2;
        ret = linux.ioctl(self.vcpufd, kvm.KVM_SET_REGS, @ptrToInt(&regs));
        if (ret != 0) {
            log.errExt(@src(), "failed to KVM_SET_REGS", .{});
            return;
        }
    }
};
