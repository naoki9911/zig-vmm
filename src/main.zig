const std = @import("std");
const log = @import("log.zig");
const vm = @import("vm.zig");
const console = @import("console.zig");
const allocator = std.heap.page_allocator;

const memSize = 1 << 30;

pub fn main() !void {
    if (std.os.argv.len >= 2 and std.mem.eql(u8, "console", std.mem.span(std.os.argv[1]))) {
        log.info("starting console", .{});
        var c = console.Console.init();
        try c.clientStart();
    } else {
        var v = try vm.VM.init("./vmlinuz-5.19.0-1018-kvm", "./initrd.img-5.19.0-1018-kvm", "./ubuntu2210-base.img", memSize, allocator);
        try v.create();
        try v.run();
    }
}
