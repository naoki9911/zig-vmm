const std = @import("std");
const log = @import("log.zig");
const virtio = @import("virtio.zig");

// https://wiki.osdev.org/PCI
pub const DeviceHeader = packed struct {
    vendor_id: u16 = 0,
    device_id: u16 = 0,
    command: u16 = 0,
    status: u16 = 0,
    revision_id: u8 = 0,
    prog_if: u8 = 0,
    class_id: u16 = 0,
    cache_line_size: u8 = 0,
    latency_timer: u8 = 0,
    header_type: u8 = 0,
    bist: u8 = 0,
    base_address_0: u32 = 0,
    base_address_1: u32 = 0,
    base_address_2: u32 = 0,
    base_address_3: u32 = 0,
    base_address_4: u32 = 0,
    base_address_5: u32 = 0,
    cardbus_cis_pointer: u32 = 0,
    subsystem_vendor_id: u16 = 0,
    subsystem_id: u16 = 0,
    expansion_rom_base_address: u32 = 0,
    capabilities_pointer: u8 = 0,
    _1: u24 = 0,
    _2: u32 = 0,
    interrupt_line: u8 = 0,
    interrupt_pin: u8 = 0,
    min_grant: u8 = 0,
    max_latency: u8 = 0,
};

const PCIBridge = struct {
    header: DeviceHeader = .{
        .device_id = 0x6000,
        .vendor_id = 0x8086,
        .header_type = 1,
    },

    const Self = @This();

    fn getDeviceHeader(self: Self) *const DeviceHeader {
        return &self.header;
    }
};

const DeviceNone = struct {
    header: DeviceHeader = .{},

    const Self = @This();

    fn getDeviceHeader(self: Self) *const DeviceHeader {
        return &self.header;
    }
};

pub const Device = union(enum) {
    none: DeviceNone,
    bridge: PCIBridge,
    virtio_net: virtio.VirtioNet,
    virtio_blk: virtio.VirtioBlk,
    virtio_console: virtio.VirtioConsole,

    const Self = @This();

    pub fn getDeviceHeader(self: Self) *const DeviceHeader {
        switch (self) {
            inline else => |case| return case.getDeviceHeader(),
        }
    }

    pub fn enableBAR0SizeProbe(self: *Self) void {
        switch (self.*) {
            .virtio_net => |*c| c.enableBAR0SizeProbe(),
            .virtio_blk => |*c| c.enableBAR0SizeProbe(),
            .virtio_console => |*c| c.enableBAR0SizeProbe(),
            else => {},
        }
    }

    pub fn disableBAR0SizeProbe(self: *Self) void {
        switch (self.*) {
            .virtio_net => |*c| c.disableBAR0SizeProbe(),
            .virtio_blk => |*c| c.disableBAR0SizeProbe(),
            .virtio_console => |*c| c.disableBAR0SizeProbe(),
            else => {},
        }
    }

    pub fn isBAR0SizeProbeEnabled(self: Self) bool {
        switch (self) {
            .virtio_net => |c| return c.isBAR0SizeProbeEnabled(),
            .virtio_blk => |c| return c.isBAR0SizeProbeEnabled(),
            .virtio_console => |c| return c.isBAR0SizeProbeEnabled(),
            else => return false,
        }
    }

    pub fn getBAR0Size(self: Self) u32 {
        switch (self) {
            .virtio_net => |c| return c.getBAR0Size(),
            .virtio_blk => |c| return c.getBAR0Size(),
            .virtio_console => |c| return c.getBAR0Size(),
            else => return 0,
        }
    }
};

pub const PCI = struct {
    addr: u32 = 0,

    // zig fmt: off
    devices: [4]Device = [_]Device{
        .{ .bridge = PCIBridge{} },
        .{ .none = DeviceNone{} },
        .{ .none = DeviceNone{} },
        .{ .none = DeviceNone{} },
    },
    // zig fmt: on

    const Self = @This();
    pub const AddrPortAddr = 0xCF8;
    pub const DataPortAddr = 0xCFC;
    pub const DataPortNum = 4;

    pub fn ioIn(self: *Self, port: u64, size: usize, value: []u8) !void {
        if (port == Self.AddrPortAddr) {
            self.confAddrIn(port, size, value);
        } else {
            self.confDataIn(port, size, value);
        }
    }

    pub fn ioOut(self: *Self, port: u64, size: usize, value: []const u8) !void {
        if (port == Self.AddrPortAddr) {
            self.confAddrOut(port, size, value);
        } else {
            self.confDataOut(port, size, value);
        }
    }

    fn confAddrIn(self: *Self, port: u64, size: usize, value: []u8) void {
        _ = port;
        value[0] = @intCast(u8, self.addr & 0xFF);
        if (size >= 2) {
            value[1] = @intCast(u8, (self.addr >> 8) & 0xFF);
        }
        if (size >= 4) {
            value[2] = @intCast(u8, (self.addr >> 16) & 0xFF);
            value[3] = @intCast(u8, (self.addr >> 24) & 0xFF);
        }

        return;
    }

    fn confAddrOut(self: *Self, port: u64, size: usize, value: []const u8) void {
        _ = port;
        self.addr = value[0];
        if (size >= 2) {
            self.addr |= (@intCast(u32, value[1]) << 8);
        }
        if (size >= 4) {
            self.addr |= (@intCast(u32, value[2]) << 16);
            self.addr |= (@intCast(u32, value[3]) << 24);
        }

        return;
    }

    fn confDataIn(self: *Self, port: u64, size: usize, value: []u8) void {
        const offset = getRegisterOffset(self.addr) + port - Self.DataPortAddr;

        if (getBusNumber(self.addr) != 0) {
            return;
        }

        if (getFunctionNumber(self.addr) != 0) {
            return;
        }

        const devNumber = getDeviceNumber(self.addr);
        if (devNumber >= self.devices.len) {
            return;
        }

        const device = &self.devices[devNumber];

        log.debug("PCI confDataIn offset=0x{x} dev={d}", .{ offset, devNumber });
        if (device.isBAR0SizeProbeEnabled() and offset == 0x10 and size >= 4) {
            const s = ~(device.getBAR0Size() - 1);
            value[0] = @intCast(u8, s & 0xFF);
            value[1] = @intCast(u8, (s >> 8) & 0xFF);
            value[2] = @intCast(u8, (s >> 16) & 0xFF);
            value[3] = @intCast(u8, (s >> 24) & 0xFF);
            device.disableBAR0SizeProbe();
            log.debug("dev={d} BAR0 size probed", .{devNumber});
        } else {
            const device_bytes = std.mem.sliceAsBytes(&[_]DeviceHeader{device.getDeviceHeader().*});
            if (offset + size >= device_bytes.len) {
                return;
            }

            //log.info("devNumber={d} offset=0x{x} sizeof={d}", .{ devNumber, offset, @sizeOf(DeviceHeader) });
            std.mem.copy(u8, value, device_bytes[offset .. offset + size]);
        }
    }

    fn confDataOut(self: *Self, port: u64, size: usize, value: []const u8) void {
        var v: u32 = @intCast(u32, value[0]);
        if (size >= 2) {
            v |= (@intCast(u32, value[1]) << 8);
        }
        if (size >= 4) {
            v |= (@intCast(u32, value[2]) << 16);
            v |= (@intCast(u32, value[3]) << 24);
        }

        if (getBusNumber(self.addr) != 0) {
            return;
        }

        if (getFunctionNumber(self.addr) != 0) {
            return;
        }

        const devNumber = getDeviceNumber(self.addr);
        if (devNumber >= self.devices.len) {
            return;
        }

        const device = &self.devices[devNumber];

        const offset = getRegisterOffset(self.addr) + port - Self.DataPortAddr;
        log.debug("PCI confDataOut offset=0x{x} value=0x{x}", .{ offset, v });

        if (offset == 0x10 and v == 0xffffffff) {
            log.debug("dev={d} enable BAR0 size probe", .{devNumber});
            device.enableBAR0SizeProbe();
        }
    }

    fn getRegisterOffset(value: u32) u8 {
        return @intCast(u8, value & 0xFC); // [7:0] and [1:0] must be zero.
    }

    fn getFunctionNumber(value: u32) u8 {
        return @intCast(u8, (value >> 8) & 0x7);
    }

    fn getDeviceNumber(value: u32) u8 {
        return @intCast(u8, (value >> 11) & 0x1F);
    }

    fn getBusNumber(value: u32) u8 {
        return @intCast(u8, (value >> 16) & 0xFF);
    }

    fn isEnable(value: u32) bool {
        return (value >> 31) == 0x1;
    }
};
