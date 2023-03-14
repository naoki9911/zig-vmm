const std = @import("std");
const log = @import("log.zig");
const vm = @import("vm.zig");
const console = @import("console.zig");
const allocator = std.heap.page_allocator;
const argsParser = @import("zig-args");

const memSize = 1 << 30;

pub fn printCmdline() void {
    std.debug.print("  --initramfs <path to initramfs>\n", .{});
    std.debug.print("  --vmlinuz <path to vmlinuz>\n", .{});
    std.debug.print("  --disk_img <path to disk image>\n", .{});
    std.debug.print("  --console_client : run console client\n", .{});
}

pub fn main() !u8 {
    const options = argsParser.parseForCurrentProcess(struct {
        // This declares long options for double hyphen
        initramfs: ?[]const u8 = null,
        vmlinuz: ?[]const u8 = null,
        disk_img: ?[]const u8 = null,
        console_client: bool = false,
        help: bool = false,

        // This declares short-hand options for single hyphen
        pub const shorthands = .{};
    }, allocator, .print) catch return 1;
    defer options.deinit();
    if (options.options.help) {
        printCmdline();
        return 0;
    }

    if (options.options.console_client) {
        log.info("starting console", .{});
        var c = console.Console.init();
        try c.clientStart();
    } else {
        if (options.options.initramfs == null) {
            _ = try std.io.getStdErr().writer().write("--initramfs is not specified\n");
            return 1;
        }
        if (options.options.vmlinuz == null) {
            _ = try std.io.getStdErr().writer().write("--vmlinuz is not specified\n");
            return 1;
        }
        var v: vm.VM = undefined;
        if (options.options.disk_img) |img| {
            v = try vm.VM.init(options.options.vmlinuz.?, options.options.initramfs.?, img, memSize, allocator);
        } else {
            v = try vm.VM.init(options.options.vmlinuz.?, options.options.initramfs.?, "", memSize, allocator);
        }
        try v.create();
        try v.run();
    }

    return 0;
}
