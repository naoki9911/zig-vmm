const std = @import("std");
const pci = @import("pci.zig");
const serial = @import("serial.zig");
const virtio = @import("virtio.zig");
const log = @import("log.zig");

const HandlerNone = struct {
    const Self = @This();
    pub fn ioIn(self: Self, port: u64, size: usize, value: []u8) !void {
        _ = self;
        _ = port;
        _ = size;
        _ = value;
    }

    pub fn ioOut(self: Self, port: u64, size: usize, value: []const u8) !void {
        _ = self;
        _ = port;
        _ = size;
        _ = value;
    }
};

pub const HandlerInterface = union(enum) {
    n: HandlerNone,
    p: *pci.PCI,
    s: *serial.Serial,
    virtio_console: *virtio.VirtioConsole,
    virtio_net: *virtio.VirtioNet,
    virtio_blk: *virtio.VirtioBlk,

    const Self = @This();

    pub fn in(self: Self, port: u64, size: usize, value: []u8) !void {
        switch (self) {
            inline else => |case| try case.ioIn(port, size, value),
        }
    }

    pub fn out(self: Self, port: u64, size: usize, value: []const u8) !void {
        switch (self) {
            inline else => |case| try case.ioOut(port, size, value),
        }
    }
};

pub const IoManager = struct {
    handler: [20]IoHandler = [_]IoHandler{.{}} ** 20,
    handler_end: usize = 0,

    const Self = @This();

    const Error = error{TooManyHandlers};

    pub fn append(self: *Self, addr_begin: u64, addr_end: u64, handler: HandlerInterface) !void {
        if (self.handler_end >= self.handler.len) {
            return Error.TooManyHandlers;
        }

        self.handler[self.handler_end] = IoHandler{
            .addr_begin = addr_begin,
            .addr_end = addr_end,
            .handler = handler,
        };
        self.handler_end += 1;
    }

    pub fn in(self: *Self, port: u64, size: usize, value: []u8) !void {
        //log.debug("IO In port=0x{} size={}", .{port, size});
        for (&self.handler, 0..) |*h, idx| {
            if (idx >= self.handler_end) {
                //log.warnExt(@src(), "unhandled KVM_EXT_IO_IN size={d} port=0x{x}", .{ size, port });
                return;
            }
            if (h.addr_begin <= port and port <= h.addr_end) {
                return try h.*.handler.in(port, size, value);
            }
        }
    }

    pub fn out(self: *Self, port: u64, size: usize, value: []const u8) !void {
        //log.debug("IO Out port=0x{} size={}", .{port, size});
        for (&self.handler, 0..) |*h, idx| {
            if (idx >= self.handler_end) {
                //log.warnExt(@src(), "unhandled KVM_EXT_IO_OUT size={d} port=0x{x}", .{ size, port });
                return;
            }
            if (h.addr_begin <= port and port <= h.addr_end) {
                return try h.*.handler.out(port, size, value);
            }
        }
    }

    const IoHandler = struct {
        addr_begin: u64 = 0,
        addr_end: u64 = 0,
        handler: HandlerInterface = undefined,
    };
};
