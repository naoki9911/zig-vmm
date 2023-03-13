const std = @import("std");
const pci = @import("pci.zig");
const log = @import("log.zig");
const kvm = @import("kvm.zig");
const console = @import("console.zig");
const loader = @import("loader.zig");
const linux = std.os.linux;

// 4.1.4.8 Legacy Interfaces: A Note on PCI Device Layout
pub const VirtioHeader = packed struct {
    device_features: u32 = 0, // 0x00
    guest_features: u32 = 0, // 0x04
    queue_pfn: u32 = 0, // 0x08
    queue_size: u16 = 0, // 0x0C
    queue_select: u16 = 0, // 0x0E
    queue_notify: u16 = 0, // 0x10
    device_status: u8 = 0, // 0x12
    isr_status: u8 = 0, // 0x13

    pub const DEVICE_ACKNOWLEDGED = 0x1;
    pub const DRIVER_LOADED = 0x2;
    pub const DRIVER_READY = 0x4;
    pub const DEVICE_ERROR = 0x40;
    pub const DRIVER_FAILED = 0x80;
};

pub fn VirtioQueue(comptime size: usize, comptime Parent: type) type {
    return struct {
        queue: *Queue,
        last_avail_idx: u16 = 0,
        mem_addr: u64,

        pub const QueueSize = size;
        pub const Queue = VirtioQueueSplit(size);
        const Self = @This();

        pub fn init(addr: u64, mem_addr: u64) Self {
            return .{
                .queue = @intToPtr(*Queue, addr),
                .mem_addr = mem_addr,
            };
        }

        const Error = error{
            TooManyDescriptorsToHandle,
            InvalidRequest,
            InvalidDescriptor,
            NotEnoughDescriptors,
        };

        // @return: inject IRQ or not
        pub fn handle(self: *Self, parent: *Parent) !bool {
            const descs = &self.queue.descriptor_tables;
            const ar = &self.queue.available_ring;
            const ur = &self.queue.used_ring;
            var descs_handle: [64]*Queue.DescriptorTable = undefined;

            //self.queue.print();
            while (ar.index != self.last_avail_idx) {
                var desc_num: usize = 0;
                var descID = ar.ring[self.last_avail_idx % size];
                ur.ring[ur.index % ur.ring.len].length = 0;
                ur.ring[ur.index % ur.ring.len].index = descID;
                while (true) {
                    if (desc_num >= descs_handle.len) {
                        return Error.TooManyDescriptorsToHandle;
                    }
                    const desc = &descs[descID];

                    descs_handle[desc_num] = desc;
                    desc_num += 1;

                    ur.ring[ur.index % ur.ring.len].length += desc.length;

                    if ((desc.flags & 0x1) == 0) {
                        break;
                    }

                    descID = desc.next;
                }

                try parent.handleVirtioQueue(descs_handle[0..desc_num]);

                self.last_avail_idx = @addWithOverflow(self.last_avail_idx, 1)[0];
                ur.index = @addWithOverflow(ur.index, 1)[0];
            }

            return ur.flags & 0x1 != 0x1;
        }

        pub fn put(self: *Self, buf: []const u8) !bool {
            const descs = &self.queue.descriptor_tables;
            const ar = &self.queue.available_ring;
            const ur = &self.queue.used_ring;
            var rest_len = buf.len;
            const cur_last_avail_idx = self.last_avail_idx;
            var prev_descID: i32 = -1;
            while (rest_len > 0) {
                if (self.last_avail_idx == ar.index) {
                    log.errExt(@src(), "queue is full", .{});
                    self.last_avail_idx = cur_last_avail_idx;
                    return Error.NotEnoughDescriptors;
                }
                const descID = ar.ring[self.last_avail_idx % QueueSize];
                const desc = &descs[descID];

                if (rest_len == buf.len) {
                    ur.ring[ur.index % QueueSize].index = descID;
                    ur.ring[ur.index % QueueSize].length = 0;
                }

                var desc_buf = @intToPtr([*]u8, self.mem_addr + desc.address)[0..desc.length];
                const desc_len = @intCast(u32, @min(rest_len, desc.length));
                const buf_idx = buf.len - rest_len;
                std.mem.copy(u8, desc_buf, buf[buf_idx .. buf_idx + desc_len]);
                desc.length = desc_len;
                ur.ring[ur.index % QueueSize].length += desc_len;

                if (prev_descID != -1) {
                    const p = @intCast(usize, prev_descID);
                    descs[p].flags |= 0x1; // desc is chained
                    descs[p].next = descID;
                }

                prev_descID = descID;
                self.last_avail_idx = @addWithOverflow(self.last_avail_idx, 1)[0];
                rest_len -= desc_len;
            }

            ur.index = @addWithOverflow(ur.index, 1)[0];

            return ur.flags & 0x1 != 0x1;
        }
    };
}

fn VirtioQueueSplit(comptime size: usize) type {
    return extern struct {
        descriptor_tables: [QueueSize]DescriptorTable = [_]DescriptorTable{.{}} ** QueueSize,
        available_ring: AvailableRing = .{},
        // 4KiB paging alignment
        _padding: [4096 - (((@sizeOf(DescriptorTable) * QueueSize) + @sizeOf(AvailableRing)) % 4096)]u8 = undefined,
        used_ring: UsedRing = .{},

        const Self = @This();
        pub const QueueSize = size;

        pub const DescriptorTable = extern struct {
            address: u64 = 0,
            length: u32 = 0,
            flags: u16 = 0,
            next: u16 = 0,
        };

        pub const AvailableRing = extern struct {
            flags: u16 = 0,
            index: u16 = 0,
            ring: [QueueSize]u16 = [_]u16{0} ** QueueSize,
            event_idx: u16 = 0,
        };

        pub const UsedRing = extern struct {
            flags: u16 = 0,
            index: u16 = 0,
            ring: [QueueSize]Ring = [_]Ring{.{}} ** QueueSize,
            avail_event: u16 = 0,
            pub const Ring = extern struct {
                index: u32 = 0,
                length: u32 = 0,
            };
        };

        pub fn print(self: Self) void {
            const descs = self.descriptor_tables;
            const ar = self.available_ring;
            const ur = self.used_ring;
            log.debug("VirtioQueue", .{});
            var i: usize = 0;
            while (i < self.descriptor_tables.len) : (i += 1) {
                log.debug("- Desc[{d}] = (addr:0x{x} len:0x{x} flags:0x{x} next:0x{x})", .{ i, descs[i].address, descs[i].length, descs[i].flags, descs[i].next });
            }
            log.debug("- AvailRing = (flags:0x{x} index:0x{x} event_idx:0x{x})", .{ ar.flags, ar.index, ar.event_idx });
            i = 0;
            while (i < ar.ring.len) : (i += 1) {
                log.debug("  - Ring[{d}] = 0x{x}", .{ i, ar.ring[i] });
            }
            log.debug("- UsedRing = (flags:0x{x} index:0x{x} avail_event:0x{x})", .{ ur.flags, ur.index, ur.avail_event });
            i = 0;
            while (i < ur.ring.len) : (i += 1) {
                log.debug("  - Ring[{d}] = index:0x{x} length:0x{x}", .{ i, ur.ring[i].index, ur.ring[i].length });
            }
        }
    };
}

pub const VirtioConsole = struct {
    vmfd: i32 = 0,
    mem_addr: u64 = 0,
    header: pci.DeviceHeader = .{
        // https://docs.oasis-open.org/virtio/virtio/v1.1/csprd01/virtio-v1.1-csprd01.html#x1-1020002
        .vendor_id = 0x1AF4,
        .device_id = 0x1003,

        .command = 0x1, // access via I/O port
        .base_address_0 = IoPortStart | 0x1,

        // https://docs.oasis-open.org/virtio/virtio/v1.1/csprd01/virtio-v1.1-csprd01.html#x1-1930005
        .subsystem_id = 0x03, // VIRTIO_CONSOLE
        .interrupt_line = IRQLine,
        .interrupt_pin = 1,
    },
    queue: [2]VQ,

    isBAR0SizeProbe: bool = false,
    vHeader: VirtioHeader = .{
        .queue_size = VQ.QueueSize,
    },
    con: *console.Console,

    const Error = error{
        InvalidIRQLevel,
    };

    pub const IoPortStart = 0x6200;
    pub const IoPortSize = 0x100;
    const IRQLine = 10;
    const VQ = VirtioQueue(128, Self);

    const Self = @This();

    pub fn getDeviceHeader(self: Self) *const pci.DeviceHeader {
        return &self.header;
    }

    pub fn enableBAR0SizeProbe(self: *Self) void {
        self.isBAR0SizeProbe = true;
    }

    pub fn disableBAR0SizeProbe(self: *Self) void {
        self.isBAR0SizeProbe = false;
    }

    pub fn isBAR0SizeProbeEnabled(self: Self) bool {
        return self.isBAR0SizeProbe;
    }

    pub fn getBAR0Size(self: Self) u32 {
        _ = self;
        return IoPortSize;
    }

    pub fn init(vmfd: i32, mem_addr: u64, con: *console.Console) Self {
        return .{
            .vmfd = vmfd,
            .mem_addr = mem_addr,
            .queue = [_]VQ{VQ.init(mem_addr, mem_addr)} ** 2,
            .con = con,
        };
    }

    pub fn injectIRQ(self: Self) !void {
        var irq = kvm.kvm_irq_level{
            .irq = Self.IRQLine,
            .level = 0,
        };
        var ret = linux.ioctl(self.vmfd, kvm.KVM_IRQ_LINE_STATUS, @ptrToInt(&irq));
        if (ret != 0) {
            return Error.InvalidIRQLevel;
        }

        irq.irq = Self.IRQLine;
        irq.level = 1;
        ret = linux.ioctl(self.vmfd, kvm.KVM_IRQ_LINE_STATUS, @ptrToInt(&irq));
        if (ret != 0) {
            return Error.InvalidIRQLevel;
        }

        return;
    }

    pub fn ioIn(self: *Self, port: u64, size: usize, value: []u8) !void {
        const offset = port - Self.IoPortStart;
        var device_bytes = @ptrCast([*]u8, &self.vHeader)[0..@sizeOf(VirtioHeader)];
        std.mem.copy(u8, value, device_bytes[offset .. offset + size]);

        if (offset == 0x08) {
            var v: u32 = 0;
            const q_sel = self.vHeader.queue_select;
            if (q_sel < self.queue.len) {
                v = @intCast(u32, (@ptrToInt(self.queue[q_sel].queue) - self.mem_addr) / 4096);
            }
            if (value.len < 4) {
                log.errExt(@src(), "unexpected value len(expected={d} actual={d})", .{ 4, value.len });
                return;
            }
            std.mem.writeIntLittle(u32, value[0..4], v);
        }
    }

    pub fn putc(self: *Self, value: u8) !void {
        const q_sel: usize = 0;
        self.vHeader.isr_status = 0;
        defer self.vHeader.isr_status = 1;

        const inject_irq = try self.queue[q_sel].put(&[_]u8{value});
        if (inject_irq) {
            try self.injectIRQ();
        }
    }

    fn handleVirtioQueue(self: *Self, descs: []*VQ.Queue.DescriptorTable) !void {
        for (descs) |desc| {
            const buf = @intToPtr([*]u8, self.mem_addr + desc.address)[0..desc.length];
            _ = try std.io.getStdOut().write(buf);
            try self.con.write(buf);
        }
    }

    pub fn ioOut(self: *Self, port: u64, size: usize, value: []const u8) !void {
        var device_bytes = @ptrCast([*]u8, &self.vHeader)[0..@sizeOf(VirtioHeader)];
        const offset = port - Self.IoPortStart;
        if (offset != 0x0 and offset != 0xc and offset != 0x13) { // device features, QueueSize and ISR is not writable
            std.mem.copy(u8, device_bytes[offset..], value);
        }
        var v: u32 = @intCast(u32, value[0]);
        if (size >= 2) {
            v |= (@intCast(u32, value[1]) << 8);
        }
        if (size >= 4) {
            v |= (@intCast(u32, value[2]) << 16);
            v |= (@intCast(u32, value[3]) << 24);
        }

        //log.info("[virtio-console] ioOut port=0x{x} value=0x{x}", .{ port, v });
        if (offset == 0x08) {
            if (self.vHeader.queue_select < self.queue.len) {
                log.debugExt(@src(), "SEL={d} PFN={d}", .{ self.vHeader.queue_select, v });
                self.queue[self.vHeader.queue_select] = VQ.init(v * 4096 + self.mem_addr, self.mem_addr);
            }
        } else if (offset == 0x0e) {
            //log.info("queue_select", .{});
        } else if (offset == 0x10) {
            self.vHeader.isr_status = 0;
            const qs = self.vHeader.queue_select;
            defer self.vHeader.isr_status = 1;
            //log.debugExt(@src(), "SEL={d} kick!", .{qs});
            const inject_irq = try self.queue[qs].handle(self);
            if (inject_irq) {
                try self.injectIRQ();
            }
        } else if (offset == 0x13) {
            //log.infoExt(@src(), "isr", .{});
        }
    }
};

pub const VirtioNet = struct {
    vmfd: i32 = 0,
    mem_addr: u64 = 0,
    header: pci.DeviceHeader = .{
        .vendor_id = 0x1AF4,
        .device_id = 0x1000,

        .command = 0x1, // via I/O port
        .base_address_0 = IoPortStart | 0x1,

        .subsystem_id = 0x1,
        .interrupt_line = IRQLine,
        .interrupt_pin = 1,
    },
    queue: [2]VQ,
    tx1_buf: [10000]u8 = undefined,
    rx1_buf: [10000]u8 = undefined,
    tap_sock: i32 = 0,
    rx1_started: bool = false,

    isBAR0SizeProbe: bool = false,
    vHeader: VirtioHeader = .{
        .queue_size = VQ.QueueSize,
    },

    const Error = error{
        InvalidIRQLevel,
    };

    pub const IoPortStart = 0x6300;
    pub const IoPortSize = 0x100;
    const IRQLine = 11;

    const VQ = VirtioQueue(2048, Self);
    const Self = @This();

    pub fn getDeviceHeader(self: Self) *const pci.DeviceHeader {
        return &self.header;
    }

    pub fn enableBAR0SizeProbe(self: *Self) void {
        self.isBAR0SizeProbe = true;
    }

    pub fn disableBAR0SizeProbe(self: *Self) void {
        self.isBAR0SizeProbe = false;
    }

    pub fn isBAR0SizeProbeEnabled(self: Self) bool {
        return self.isBAR0SizeProbe;
    }

    pub fn getBAR0Size(self: Self) u32 {
        _ = self;
        return IoPortSize;
    }

    pub fn init(vmfd: i32, mem_addr: u64) Self {
        return .{
            .vmfd = vmfd,
            .mem_addr = mem_addr,
            .queue = [_]VQ{VQ.init(mem_addr, mem_addr)} ** 2,
        };
    }

    pub fn createTapDevice(self: *Self) !void {
        const IFF_TAP = 0x0002;
        const IFF_NO_PI = 0x1000;
        const TUNSETIFF = linux.IOCTL.IOW('T', 202, u32);
        self.tap_sock = try std.os.open("/dev/net/tun", std.os.O.RDWR, 0);
        var ifr = linux.ifreq{ .ifrn = .{
            .name = [_]u8{0} ** linux.IFNAMESIZE,
        }, .ifru = .{
            .flags = IFF_TAP | IFF_NO_PI,
        } };
        std.mem.copy(u8, &ifr.ifrn.name, "zig-vmm-test");

        const ret = linux.ioctl(self.tap_sock, TUNSETIFF, @ptrToInt(&ifr));
        if (ret != 0) {
            log.errExt(@src(), "failed to create tap device", .{});
        }
    }

    pub fn injectIRQ(self: Self) !void {
        var irq = kvm.kvm_irq_level{
            .irq = Self.IRQLine,
            .level = 0,
        };
        var ret = linux.ioctl(self.vmfd, kvm.KVM_IRQ_LINE_STATUS, @ptrToInt(&irq));
        if (ret != 0) {
            return Error.InvalidIRQLevel;
        }

        irq.irq = Self.IRQLine;
        irq.level = 1;
        ret = linux.ioctl(self.vmfd, kvm.KVM_IRQ_LINE_STATUS, @ptrToInt(&irq));
        if (ret != 0) {
            return Error.InvalidIRQLevel;
        }

        return;
    }

    pub fn ioIn(self: *Self, port: u64, size: usize, value: []u8) !void {
        const offset = port - Self.IoPortStart;
        var device_bytes = @ptrCast([*]u8, &self.vHeader)[0..@sizeOf(VirtioHeader)];
        std.mem.copy(u8, value, device_bytes[offset .. offset + size]);

        if (offset == 0x08) {
            var v: u32 = 0;
            const q_sel = self.vHeader.queue_select;
            if (q_sel < self.queue.len) {
                v = @intCast(u32, (@ptrToInt(self.queue[q_sel].queue) - self.mem_addr) / 4096);
            }
            if (value.len < 4) {
                log.errExt(@src(), "unexpected value len(expected={d} actual={d})", .{ 4, value.len });
                return;
            }
            std.mem.writeIntLittle(u32, value[0..4], v);
        }
    }

    pub fn handleReceivedPackets(self: *Self) !void {
        while (true) {
            std.mem.set(u8, self.rx1_buf[0..10], 0);
            const recv_size = try std.os.read(self.tap_sock, self.rx1_buf[10..]) + 10;
            //log.infoExt(@src(), "packet received len={}", .{recv_size});

            const q_sel: usize = 0;
            self.vHeader.isr_status = 0;
            defer self.vHeader.isr_status = 1;

            const inject_irq = self.queue[q_sel].put(self.rx1_buf[0..recv_size]) catch |err| blk: {
                log.errExt(@src(), "failed to put packet err={}", .{err});
                break :blk false;
            };
            if (inject_irq) {
                try self.injectIRQ();
            }
        }
    }

    fn handleVirtioQueue(self: *Self, descs: []*VQ.Queue.DescriptorTable) !void {
        var buf_idx: usize = 0;
        for (descs) |desc| {
            const buf = @intToPtr([*]u8, self.mem_addr + desc.address)[0..desc.length];
            std.mem.copy(u8, self.tx1_buf[buf_idx..], buf);
            buf_idx += buf.len;
        }
        _ = try std.os.write(self.tap_sock, self.tx1_buf[10..buf_idx]);
    }

    pub fn ioOut(self: *Self, port: u64, size: usize, value: []const u8) !void {
        var device_bytes = @ptrCast([*]u8, &self.vHeader)[0..@sizeOf(VirtioHeader)];
        const offset = port - Self.IoPortStart;
        if (offset != 0x0 and offset != 0xc and offset != 0x13) { // device features, QueueSize and ISR is not writable
            std.mem.copy(u8, device_bytes[offset..], value);
        }
        var v: u32 = @intCast(u32, value[0]);
        if (size >= 2) {
            v |= (@intCast(u32, value[1]) << 8);
        }
        if (size >= 4) {
            v |= (@intCast(u32, value[2]) << 16);
            v |= (@intCast(u32, value[3]) << 24);
        }
        //log.debugExt(@src(), "ioOut port=0x{x} value=0x{x}", .{ port, v });
        if (offset == 0x08) {
            if (self.vHeader.queue_select < self.queue.len) {
                log.debugExt(@src(), "SEL={d} PFN={d}", .{ self.vHeader.queue_select, v });
                self.queue[self.vHeader.queue_select] = VQ.init(v * 4096 + self.mem_addr, self.mem_addr);
            }
        } else if (offset == 0x0e) {
            //log.info("queue_select", .{});
        } else if (offset == 0x10) {
            self.vHeader.isr_status = 0;
            const qs = self.vHeader.queue_select;
            //log.debugExt(@src(), "SEL={d} kick!", .{qs});

            if (qs != 1) { // only handling transmitq1
                return;
            }
            const inject_irq = try self.queue[qs].handle(self);
            if (inject_irq) {
                try self.injectIRQ();
            }
        } else if (offset == 0x12) {
            if (v == 0x07 and !self.rx1_started) {
                log.infoExt(@src(), "driver is ready. start to receive packetes", .{});
                const thread_config = std.Thread.SpawnConfig{};
                var rx_thread = try std.Thread.spawn(thread_config, handleReceivedPackets, .{self});
                _ = rx_thread;
                self.rx1_started = true;
            }
        } else if (offset == 0x13) {
            log.infoExt(@src(), "isr", .{});
        }
    }
};

pub const VirtioBlk = struct {
    vmfd: i32 = 0,
    mem_addr: u64 = 0,
    header: pci.DeviceHeader = .{
        .vendor_id = 0x1AF4,
        .device_id = 0x1001,

        .command = 0x1, // via I/O port
        .base_address_0 = IoPortStart | 0x1,

        .subsystem_id = 0x2,
        .interrupt_line = IRQLine,
        .interrupt_pin = 1,
    },
    blk_header: BlkRegisters,
    image: std.fs.File,
    queue: [1]VQ,

    isBAR0SizeProbe: bool = false,
    vHeader: VirtioHeader = .{
        .queue_size = VQ.QueueSize,
    },

    const Error = error{
        InvalidIRQLevel,
    };

    const Request = extern struct {
        type: u32 = 0,
        _pad: u32 = 0,
        sector: u64 = 0,
    };
    const VIRTIO_BLK_T_IN = 0;
    const VIRTIO_BLK_T_OUT = 1;
    const VIRTIO_BLK_T_FLUSH = 4;
    const VIRTIO_BLK_T_DISCARD = 11;
    const VIRTIO_BLK_T_WRITE_ZEROES = 13;

    const VIRTIO_BLK_S_OK = 0;
    const VIRTIO_BLK_S_IOERR = 1;
    const VIRTIO_BLK_S_UNSUPP = 2;

    const BlkRegisters = extern struct {
        // https://docs.oasis-open.org/virtio/virtio/v1.1/csprd01/virtio-v1.1-csprd01.html#x1-2440004
        capacity: u64,
    };

    pub const IoPortStart = 0x6400;
    pub const IoPortSize = 0x100;
    const IRQLine = 9;

    const VQ = VirtioQueue(64, Self);
    const Self = @This();

    pub fn getDeviceHeader(self: Self) *const pci.DeviceHeader {
        return &self.header;
    }

    pub fn enableBAR0SizeProbe(self: *Self) void {
        self.isBAR0SizeProbe = true;
    }

    pub fn disableBAR0SizeProbe(self: *Self) void {
        self.isBAR0SizeProbe = false;
    }

    pub fn isBAR0SizeProbeEnabled(self: Self) bool {
        return self.isBAR0SizeProbe;
    }

    pub fn getBAR0Size(self: Self) u32 {
        _ = self;
        return IoPortSize;
    }

    pub fn init(vmfd: i32, mem_addr: u64, img_path: []const u8) !Self {
        var img = try loader.openFile(img_path, .{ .mode = .read_write });
        const img_meta = try img.metadata();
        const sector_num = img_meta.size() / 512;
        log.info("opened {s}(size={d} sector_num={d})", .{ img_path, img_meta.size(), sector_num });
        return .{ .vmfd = vmfd, .mem_addr = mem_addr, .image = img, .queue = [_]VQ{VQ.init(mem_addr, mem_addr)} ** 1, .blk_header = .{
            .capacity = sector_num,
        } };
    }

    pub fn injectIRQ(self: Self) !void {
        var irq = kvm.kvm_irq_level{
            .irq = Self.IRQLine,
            .level = 0,
        };
        var ret = linux.ioctl(self.vmfd, kvm.KVM_IRQ_LINE_STATUS, @ptrToInt(&irq));
        if (ret != 0) {
            return Error.InvalidIRQLevel;
        }

        irq.irq = Self.IRQLine;
        irq.level = 1;
        ret = linux.ioctl(self.vmfd, kvm.KVM_IRQ_LINE_STATUS, @ptrToInt(&irq));
        if (ret != 0) {
            return Error.InvalidIRQLevel;
        }

        return;
    }

    pub fn ioIn(self: *Self, port: u64, size: usize, value: []u8) !void {
        const offset = port - Self.IoPortStart;
        //log.debugExt(@src(), "ioIn offset=0x{x} len={d}", .{ offset, value.len });

        std.mem.set(u8, value, 0);
        if (offset == 0x08) {
            var v: u32 = 0;
            const q_sel = self.vHeader.queue_select;
            if (q_sel < self.queue.len) {
                v = @intCast(u32, (@ptrToInt(self.queue[q_sel].queue) - self.mem_addr) / 4096);
            }
            if (value.len < 4) {
                log.errExt(@src(), "unexpected value len(expected={d} actual={d})", .{ 4, value.len });
                return;
            }
            std.mem.writeIntLittle(u32, value[0..4], v);
        } else if (offset >= 0x14 and offset < 0x14 + @sizeOf(BlkRegisters)) { // sector count
            const blk_offset = offset - 0x14;
            var hdr_bytes = @ptrCast([*]u8, &self.blk_header)[0..@sizeOf(BlkRegisters)];
            std.mem.copy(u8, value, hdr_bytes[blk_offset .. blk_offset + size]);
            //log.infoExt(@src(), "value[0]={d}", .{value[0]});
        } else {
            var device_bytes = @ptrCast([*]u8, &self.vHeader)[0..@sizeOf(VirtioHeader)];
            std.mem.copy(u8, value, device_bytes[offset .. offset + size]);
        }
    }

    fn handleVirtioQueue(self: *Self, descs: []*VQ.Queue.DescriptorTable) !void {
        if (descs.len < 3) {
            log.errExt(@src(), "invalid block request", .{});
            return VQ.Error.InvalidRequest;
        }

        const req = @intToPtr(*Request, self.mem_addr + descs[0].address);
        var idx: usize = req.sector * 512;
        var desc_idx: usize = 1;
        while (desc_idx < descs.len - 1) : (desc_idx += 1) {
            const desc = descs[desc_idx];
            const buf = @intToPtr([*]u8, self.mem_addr + desc.address)[0..desc.length];
            try self.image.seekTo(idx);
            if (req.type == VIRTIO_BLK_T_IN) {
                _ = try self.image.read(buf);
            } else if (req.type == VIRTIO_BLK_T_OUT) {
                _ = try self.image.write(buf);
            } else {
                log.warnExt(@src(), "unsupported operation type={d}", .{req.type});
            }
            idx += desc.length;
        }
        //log.info("virtio-blk sector=0x{x} len=0x{x}", .{req.sector, idx});

        const desc = descs[desc_idx];
        const buf = @intToPtr([*]u8, self.mem_addr + desc.address)[0..desc.length];
        if (desc.length != 1 or (desc.flags & 0x1) != 0) {
            log.errExt(@src(), "invalid descriptor len={d} flags={d}", .{ desc.length, desc.flags });
            return VQ.Error.InvalidDescriptor;
        }
        buf[0] = VIRTIO_BLK_S_OK;
    }

    pub fn ioOut(self: *Self, port: u64, size: usize, value: []const u8) !void {
        var device_bytes = @ptrCast([*]u8, &self.vHeader)[0..@sizeOf(VirtioHeader)];
        const offset = port - Self.IoPortStart;
        if (offset != 0x0 and offset != 0xc and offset != 0x13) { // device features, QueueSize and ISR is not writable
            std.mem.copy(u8, device_bytes[offset..], value);
        }
        var v: u32 = @intCast(u32, value[0]);
        if (size >= 2) {
            v |= (@intCast(u32, value[1]) << 8);
        }
        if (size >= 4) {
            v |= (@intCast(u32, value[2]) << 16);
            v |= (@intCast(u32, value[3]) << 24);
        }
        //log.debugExt(@src(), "ioOut port=0x{x} value=0x{x}", .{ port, v });
        if (offset == 0x08) {
            if (self.vHeader.queue_select < self.queue.len) {
                //log.debugExt(@src(), "SEL={d} PFN={d}", .{ self.vHeader.queue_select, v });
                self.queue[self.vHeader.queue_select] = VQ.init(v * 4096 + self.mem_addr, self.mem_addr);
            }
        } else if (offset == 0x0e) {
            //log.info("queue_select", .{});
        } else if (offset == 0x10) {
            self.vHeader.isr_status = 0;
            defer self.vHeader.isr_status = 1;
            const qs = self.vHeader.queue_select;
            //log.debugExt(@src(), "SEL={d} kick!", .{qs});
            if (qs >= self.queue.len) {
                log.err("invalid queue selector = {d}", .{qs});
                return;
            }
            const inject_irq = try self.queue[qs].handle(self);
            if (inject_irq) {
                try self.injectIRQ();
            }
        } else if (offset == 0x12) {} else if (offset == 0x13) {
            log.infoExt(@src(), "isr", .{});
        }
    }
};
