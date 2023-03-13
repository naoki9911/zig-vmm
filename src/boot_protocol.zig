const std = @import("std");

pub const LoadedHigh: u8 = 1 << 0;
pub const KASLRFlag: u8 = 1 << 1;
pub const QuietFlag: u8 = 1 << 5;
pub const KeepSegments: u8 = 1 << 6;
pub const CanUseHeap: u8 = 1 << 7;

pub const E820Ram = 1;
pub const E820Reserved = 2;

pub const RealModeIvtBegin = 0x00000000;
pub const EBDAStart = 0x0009fc00;
pub const VGARAMBegin = 0x000a0000;
pub const MBBIOSBegin = 0x000f0000;
pub const MBBIOSEnd = 0x000fffff;

pub const E820Array = struct {
    entry_num: *u8 = undefined,

    const Self = @This();
    const EntryNumOffset = 0x1E8;
    const EntryArrayOffset = 0x2D0;

    const E820Entry = extern struct {
        addr: u64 align(4) = 0,
        size: u64 align(4) = 0,
        type: u32 align(4) = 0,
    };

    pub fn fromBytes(mem: []u8) Self {
        return .{
            .entry_num = &mem[EntryNumOffset],
        };
    }

    pub fn clear(self: *Self) void {
        self.entry_num.* = 0;
    }

    pub fn add(self: *Self, addr: u64, size: u64, typ: u32) void {
        var entry = @intToPtr(*E820Entry, @ptrToInt(self.entry_num) + (EntryArrayOffset - EntryNumOffset) + 20 * self.entry_num.*);
        entry.addr = addr;
        entry.size = size;
        entry.type = typ;
        self.entry_num.* += 1;
    }
};

// https://www.kernel.org/doc/html/latest/x86/boot.html
pub const KernelHeader = packed struct {
    setup_sects: u8 = 0,
    root_flags: u16 = 0,
    syssize: u32 = 0,
    ram_size: u16 = 0,
    vid_mode: u16 = 0,
    root_dev: u16 = 0,
    boot_flag: u16 = 0,
    jump: u16 = 0,
    header: u32 = 0,
    version: u16 = 0,
    realmode_swtch: u32 = 0,
    start_sys_seg: u16 = 0,
    kernel_version: u16 = 0,
    type_of_loader: u8 = 0,
    loadflags: u8 = 0,
    setup_move_size: u16 = 0,
    code32_start: u32 = 0,
    ramdisk_image: u32 = 0,
    ramdisk_size: u32 = 0,
    bootsect_kludge: u32 = 0,
    heap_end_ptr: u16 = 0,
    ext_loader_ver: u8 = 0,
    ext_loader_type: u8 = 0,
    cmd_line_ptr: u32 = 0,
    initrd_addr_max: u32 = 0,
    kernel_alignment: u32 = 0,
    relocatable_kernel: u8 = 0,
    min_alignment: u8 = 0,
    xloadflags: u16 = 0,
    cmdline_size: u32 = 0,
    hardware_subarch: u32 = 0,
    hardware_subarch_data: u64 = 0,
    payload_offset: u32 = 0,
    payload_length: u32 = 0,
    setup_data: u64 = 0,
    perf_address: u64 = 0,
    init_size: u32 = 0,
    handover_offset: u32 = 0,
    kernel_info_offset: u32 = 0,

    const Self = @This();
    pub const Offset = 0x1F1;
    pub const EndOffset = Offset + @sizeOf(Self);
    const MagicSignature = 0x53726448;

    const Error = error{
        InvalidKernelHeader,
    };

    pub fn fromBytes(mem: []u8) !*align(1) Self {
        const res = std.mem.bytesAsValue(@This(), mem[Offset..EndOffset]);
        if (res.header != MagicSignature) {
            return Error.InvalidKernelHeader;
        }

        return res;
    }
};
