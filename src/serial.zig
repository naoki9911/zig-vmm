const std = @import("std");
const linux = std.os.linux;
const kvm = @import("kvm.zig");
const log = @import("log.zig");
const console = @import("console.zig");

pub const COM1Addr = 0x3f8;
const StaticFifo = std.fifo.LinearFifo(u8, std.fifo.LinearFifoBufferType{ .Static = 128 });

pub const Serial = struct {
    vmfd: i32 = 0,
    fifo: StaticFifo = undefined,
    IER: u8 = 0,
    LCR: u8 = 0,
    con: *console.Console,
    const Self = @This();

    const Error = error{
        InvalidIRQLevel,
    };

    pub fn init(vmfd: i32, con: *console.Console) Self {
        return .{
            .vmfd = vmfd,
            .fifo = StaticFifo.init(),
            .con = con,
        };
    }

    fn dlab(self: Self) bool {
        return self.LCR & 0x80 != 0;
    }

    pub fn ioIn(self: *Self, port: u64, size: usize, value: []u8) !void {
        _ = size;
        const p = port - COM1Addr;
        switch (p) {
            0 => {
                if (self.dlab()) {
                    value[0] = 0xc;
                    log.debugExt(@src(), "[IN DLL] value: 0x{x}", .{value[0]});
                } else {
                    if (self.fifo.readItem()) |r| {
                        value[0] = r;
                    }
                }
            },
            1 => {
                if (self.dlab()) {
                    value[0] = 0x0;
                } else {
                    value[0] = self.IER;
                }
            },
            2 => {},
            3 => {},
            4 => {},
            5 => {
                value[0] = 0x60;
                if (self.fifo.readableLength() > 0) {
                    value[0] |= 0x1;
                }
            },
            6 => {},
            else => {
                //log.warn("unexpected out port={d}", .{p});
                return;
            },
        }
    }

    pub fn ioOut(self: *Self, port: u64, size: usize, value: []const u8) !void {
        _ = size;
        const p = port - COM1Addr;
        switch (p) {
            0 => {
                if (self.dlab()) {
                    log.debugExt(@src(), "[OUT DLL] value: 0x{x}", .{value[0]});
                } else {
                    _ = try std.io.getStdOut().write(&[_]u8{value[0]});
                    try self.con.write(&[_]u8{value[0]});
                }
            },
            1 => {
                if (self.dlab()) {
                    log.debugExt(@src(), "[OUT DLM] value: 0x{x}", .{value[0]});
                } else {
                    self.IER = value[0];
                    if (self.IER != 0) {
                        try self.injectIRQ();
                        log.debugExt(@src(), "injected IRQ", .{});
                    }
                }
            },
            2 => {
                log.debugExt(@src(), "[OUT FCR] value: 0x{x}", .{value[0]});
            },
            3 => {
                self.LCR = value[0];
                log.debugExt(@src(), "[OUT LCR] value: 0x{x}", .{value[0]});
            },
            4 => {
                log.debugExt(@src(), "[OUT MCR] value: 0x{x}", .{value[0]});
            },
            else => {
                //log.warn("unexpected out port={d}", .{p});
                return;
            },
        }
    }

    pub fn injectIRQ(self: Self) !void {
        var irq = kvm.kvm_irq_level{
            .irq = 4,
            .level = 0,
        };
        var ret = linux.ioctl(self.vmfd, kvm.KVM_IRQ_LINE, @ptrToInt(&irq));
        if (ret != 0) {
            return Error.InvalidIRQLevel;
        }

        irq.irq = 4;
        irq.level = 1;
        ret = linux.ioctl(self.vmfd, kvm.KVM_IRQ_LINE, @ptrToInt(&irq));
        if (ret != 0) {
            return Error.InvalidIRQLevel;
        }

        return;
    }
};
