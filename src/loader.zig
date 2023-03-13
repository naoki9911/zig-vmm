const std = @import("std");

pub fn loadFile(path: []const u8, allocator: std.mem.Allocator) ![]u8 {
    const file = try openFile(path, .{});
    defer file.close();
    const file_stat = try file.stat();

    var buf = try allocator.alloc(u8, file_stat.size);
    _ = try file.read(buf);

    return buf;
}

pub fn openFile(path: []const u8, flags: std.fs.File.OpenFlags) !std.fs.File {
    var path_buffer: [std.fs.MAX_PATH_BYTES]u8 = undefined;
    const path_abs = try std.fs.realpath(path, &path_buffer);
    const file = try std.fs.openFileAbsolute(path_abs, flags);
    return file;
}
