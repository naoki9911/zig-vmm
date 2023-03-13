const std = @import("std");
const linux = std.os.linux;

const KVMIO = 0xAE;

pub const KVM_GET_API_VERSION = linux.IOCTL.IO(KVMIO, 0x0);
pub const KVM_CREATE_VM = linux.IOCTL.IO(KVMIO, 0x1);
pub const KVM_CHECK_EXTENSION = linux.IOCTL.IO(KVMIO, 0x3);
pub const KVM_GET_VCPU_MMAP_SIZE = linux.IOCTL.IO(KVMIO, 0x04);
pub const KVM_GET_SUPPORTED_CPUID = linux.IOCTL.IOWR(KVMIO, 0x05, [2]u32);
pub const KVM_SET_USER_MEMORY_REGION = linux.IOCTL.IOW(KVMIO, 0x46, kvm_userspace_memory_region);
pub const KVM_CREATE_VCPU = linux.IOCTL.IO(KVMIO, 0x41);
pub const KVM_SET_TSS_ADDR = linux.IOCTL.IO(KVMIO, 0x47);
pub const KVM_SET_IDENTITY_MAP_ADDR = linux.IOCTL.IOW(KVMIO, 0x48, u64);
pub const KVM_CREATE_IRQCHIP = linux.IOCTL.IO(KVMIO, 0x60);
pub const KVM_IRQ_LINE = linux.IOCTL.IOW(KVMIO, 0x61, kvm_irq_level);
pub const KVM_IRQ_LINE_STATUS = linux.IOCTL.IOWR(KVMIO, 0x67, kvm_irq_level);
pub const KVM_CREATE_PIT2 = linux.IOCTL.IOW(KVMIO, 0x77, [16]u32);
pub const KVM_RUN = linux.IOCTL.IO(KVMIO, 0x80);
pub const KVM_GET_REGS = linux.IOCTL.IOR(KVMIO, 0x81, [144]u8);
pub const KVM_SET_REGS = linux.IOCTL.IOW(KVMIO, 0x82, [144]u8);
pub const KVM_GET_SREGS = linux.IOCTL.IOR(KVMIO, 0x83, [312]u8);
pub const KVM_SET_SREGS = linux.IOCTL.IOW(KVMIO, 0x84, [312]u8);
pub const KVM_SET_CPUID2 = linux.IOCTL.IOW(KVMIO, 0x90, [2]u32);

pub const KVM_CAP_USER_MEMORY = 3;

pub const KVM_EXIT_IO_IN = 0;
pub const KVM_EXIT_IO_OUT = 1;

pub const KVM_EXIT_UNKNOWN = 0;
pub const KVM_EXIT_EXCEPTION = 1;
pub const KVM_EXIT_IO = 2;
pub const KVM_EXIT_HYPERCALL = 3;
pub const KVM_EXIT_DEBUG = 4;
pub const KVM_EXIT_HLT = 5;
pub const KVM_EXIT_MMIO = 6;
pub const KVM_EXIT_IRQ_WINDOW_OPEN = 7;
pub const KVM_EXIT_SHUTDOWN = 8;

pub const KVM_CPUID_SIGNATURE = 0x40000000;
pub const KVM_CPUID_FEATURES = 0x40000001;

pub const kvm_cpuid_entry = extern struct { function: u32 = 0, index: u32 = 0, flags: u32 = 0, eax: u32 = 0, ebx: u32 = 0, ecx: u32 = 0, edx: u32 = 0, pad: [3]u32 = [_]u32{ 0, 0, 0 } };

pub const kvm_cpuid2 = extern struct {
    nent: u32 = 100,
    pad: u32 = 0,
    entry: [100]kvm_cpuid_entry = [_]kvm_cpuid_entry{.{}} ** 100,
};

pub const kvm_userspace_memory_region = extern struct {
    slot: u32 = 0,
    flags: u32 = 0,
    guest_phys_addr: u64 = 0,
    memory_size: u64 = 0,
    userspace_addr: u64 = 0,
};

pub const kvm_irq_level = extern struct {
    irq: u32 = 0,
    level: u32 = 0,
};

pub const kvm_pit_config = extern struct {
    flags: u32 = 0,
    pad: [15]u32 = [_]u32{0} ** 15,
};

pub const kvm_regs = extern struct {
    rax: u64 = 0,
    rbx: u64 = 0,
    rcx: u64 = 0,
    rdx: u64 = 0,
    rsi: u64 = 0,
    rdi: u64 = 0,
    rsp: u64 = 0,
    rbp: u64 = 0,
    r8: u64 = 0,
    r9: u64 = 0,
    r10: u64 = 0,
    r11: u64 = 0,
    r12: u64 = 0,
    r13: u64 = 0,
    r14: u64 = 0,
    r15: u64 = 0,
    rip: u64 = 0,
    rflags: u64 = 0,
};

pub const kvm_segment = extern struct {
    base: u64 = 0,
    limit: u32 = 0,
    selector: u16 = 0,
    type: u8 = 0,
    present: u8 = 0,
    dpl: u8 = 0,
    db: u8 = 0,
    s: u8 = 0,
    l: u8 = 0,
    g: u8 = 0,
    avl: u8 = 0,
    unusable: u8 = 0,
    padding: u8 = 0,
};

pub const kvm_dtable = extern struct {
    base: u64 = 0,
    limit: u16 = 0,
    padding: [6]u8 = [_]u8{0} ** 6,
};

pub const KVM_NR_INTERRUPTS = 256;

pub const kvm_sregs = extern struct {
    cs: kvm_segment = .{},
    ds: kvm_segment = .{},
    es: kvm_segment = .{},
    fs: kvm_segment = .{},
    gs: kvm_segment = .{},
    ss: kvm_segment = .{},
    tr: kvm_segment = .{},
    ldt: kvm_segment = .{},
    gdt: kvm_dtable = .{},
    idt: kvm_dtable = .{},
    cr0: u64 = 0,
    cr2: u64 = 0,
    cr3: u64 = 0,
    cr4: u64 = 0,
    cr8: u64 = 0,
    efer: u64 = 0,
    apic_base: u64 = 0,
    interrupt_bitmap: [(KVM_NR_INTERRUPTS + 63) / 64]u8 = [_]u8{0} ** ((KVM_NR_INTERRUPTS + 63) / 64),
};

// include/uapi/linux/kvm.h
// /* for KVM_RUN, returned by mmap(vcpu_fd, offset=0) */
// struct kvm_run {
// 	/* in */
// 	__u8 request_interrupt_window;
// 	__u8 immediate_exit;
// 	__u8 padding1[6];
//
// 	/* out */
// 	__u32 exit_reason;
// 	__u8 ready_for_interrupt_injection;
// 	__u8 if_flag;
// 	__u16 flags;
//
// 	/* in (pre_kvm_run), out (post_kvm_run) */
// 	__u64 cr8;
// 	__u64 apic_base;
//
// #ifdef __KVM_S390
// 	/* the processor status word for s390 */
// 	__u64 psw_mask; /* psw upper half */
// 	__u64 psw_addr; /* psw lower half */
// #endif
// 	union {
// 		/* KVM_EXIT_UNKNOWN */
// 		struct {
// 			__u64 hardware_exit_reason;
// 		} hw;
// 		/* KVM_EXIT_FAIL_ENTRY */
// 		struct {
// 			__u64 hardware_entry_failure_reason;
// 			__u32 cpu;
// 		} fail_entry;
// 		/* KVM_EXIT_EXCEPTION */
// 		struct {
// 			__u32 exception;
// 			__u32 error_code;
// 		} ex;
// 		/* KVM_EXIT_IO */
// 		struct {
// #define KVM_EXIT_IO_IN  0
// #define KVM_EXIT_IO_OUT 1
// 			__u8 direction;
// 			__u8 size; /* bytes */
// 			__u16 port;
// 			__u32 count;
// 			__u64 data_offset; /* relative to kvm_run start */
// 		} io;
// 		/* KVM_EXIT_DEBUG */
// 		struct {
// 			struct kvm_debug_exit_arch arch;
// 		} debug;
// 		/* KVM_EXIT_MMIO */
// 		struct {
// 			__u64 phys_addr;
// 			__u8  data[8];
// 			__u32 len;
// 			__u8  is_write;
// 		} mmio;
// 		/* KVM_EXIT_HYPERCALL */
// 		struct {
// 			__u64 nr;
// 			__u64 args[6];
// 			__u64 ret;
// 			__u32 longmode;
// 			__u32 pad;
// 		} hypercall;
// 		/* KVM_EXIT_TPR_ACCESS */
// 		struct {
// 			__u64 rip;
// 			__u32 is_write;
// 			__u32 pad;
// 		} tpr_access;
// 		/* KVM_EXIT_S390_SIEIC */
// 		struct {
// 			__u8 icptcode;
// 			__u16 ipa;
// 			__u32 ipb;
// 		} s390_sieic;
// 		/* KVM_EXIT_S390_RESET */
// #define KVM_S390_RESET_POR       1
// #define KVM_S390_RESET_CLEAR     2
// #define KVM_S390_RESET_SUBSYSTEM 4
// #define KVM_S390_RESET_CPU_INIT  8
// #define KVM_S390_RESET_IPL       16
// 		__u64 s390_reset_flags;
// 		/* KVM_EXIT_S390_UCONTROL */
// 		struct {
// 			__u64 trans_exc_code;
// 			__u32 pgm_code;
// 		} s390_ucontrol;
// 		/* KVM_EXIT_DCR (deprecated) */
// 		struct {
// 			__u32 dcrn;
// 			__u32 data;
// 			__u8  is_write;
// 		} dcr;
// 		/* KVM_EXIT_INTERNAL_ERROR */
// 		struct {
// 			__u32 suberror;
// 			/* Available with KVM_CAP_INTERNAL_ERROR_DATA: */
// 			__u32 ndata;
// 			__u64 data[16];
// 		} internal;
// 		/*
// 		 * KVM_INTERNAL_ERROR_EMULATION
// 		 *
// 		 * "struct emulation_failure" is an overlay of "struct internal"
// 		 * that is used for the KVM_INTERNAL_ERROR_EMULATION sub-type of
// 		 * KVM_EXIT_INTERNAL_ERROR.  Note, unlike other internal error
// 		 * sub-types, this struct is ABI!  It also needs to be backwards
// 		 * compatible with "struct internal".  Take special care that
// 		 * "ndata" is correct, that new fields are enumerated in "flags",
// 		 * and that each flag enumerates fields that are 64-bit aligned
// 		 * and sized (so that ndata+internal.data[] is valid/accurate).
// 		 *
// 		 * Space beyond the defined fields may be used to store arbitrary
// 		 * debug information relating to the emulation failure. It is
// 		 * accounted for in "ndata" but the format is unspecified and is
// 		 * not represented in "flags". Any such information is *not* ABI!
// 		 */
// 		struct {
// 			__u32 suberror;
// 			__u32 ndata;
// 			__u64 flags;
// 			union {
// 				struct {
// 					__u8  insn_size;
// 					__u8  insn_bytes[15];
// 				};
// 			};
// 			/* Arbitrary debug data may follow. */
// 		} emulation_failure;
// 		/* KVM_EXIT_OSI */
// 		struct {
// 			__u64 gprs[32];
// 		} osi;
// 		/* KVM_EXIT_PAPR_HCALL */
// 		struct {
// 			__u64 nr;
// 			__u64 ret;
// 			__u64 args[9];
// 		} papr_hcall;
// 		/* KVM_EXIT_S390_TSCH */
// 		struct {
// 			__u16 subchannel_id;
// 			__u16 subchannel_nr;
// 			__u32 io_int_parm;
// 			__u32 io_int_word;
// 			__u32 ipb;
// 			__u8 dequeued;
// 		} s390_tsch;
// 		/* KVM_EXIT_EPR */
// 		struct {
// 			__u32 epr;
// 		} epr;
// 		/* KVM_EXIT_SYSTEM_EVENT */
// 		struct {
// #define KVM_SYSTEM_EVENT_SHUTDOWN       1
// #define KVM_SYSTEM_EVENT_RESET          2
// #define KVM_SYSTEM_EVENT_CRASH          3
// #define KVM_SYSTEM_EVENT_WAKEUP         4
// #define KVM_SYSTEM_EVENT_SUSPEND        5
// #define KVM_SYSTEM_EVENT_SEV_TERM       6
// 			__u32 type;
// 			__u32 ndata;
// 			union {
// #ifndef __KERNEL__
// 				__u64 flags;
// #endif
// 				__u64 data[16];
// 			};
// 		} system_event;
// 		/* KVM_EXIT_S390_STSI */
// 		struct {
// 			__u64 addr;
// 			__u8 ar;
// 			__u8 reserved;
// 			__u8 fc;
// 			__u8 sel1;
// 			__u16 sel2;
// 		} s390_stsi;
// 		/* KVM_EXIT_IOAPIC_EOI */
// 		struct {
// 			__u8 vector;
// 		} eoi;
// 		/* KVM_EXIT_HYPERV */
// 		struct kvm_hyperv_exit hyperv;
// 		/* KVM_EXIT_ARM_NISV */
// 		struct {
// 			__u64 esr_iss;
// 			__u64 fault_ipa;
// 		} arm_nisv;
// 		/* KVM_EXIT_X86_RDMSR / KVM_EXIT_X86_WRMSR */
// 		struct {
// 			__u8 error; /* user -> kernel */
// 			__u8 pad[7];
// #define KVM_MSR_EXIT_REASON_INVAL	(1 << 0)
// #define KVM_MSR_EXIT_REASON_UNKNOWN	(1 << 1)
// #define KVM_MSR_EXIT_REASON_FILTER	(1 << 2)
// #define KVM_MSR_EXIT_REASON_VALID_MASK	(KVM_MSR_EXIT_REASON_INVAL   |	\
// 					 KVM_MSR_EXIT_REASON_UNKNOWN |	\
// 					 KVM_MSR_EXIT_REASON_FILTER)
// 			__u32 reason; /* kernel -> user */
// 			__u32 index; /* kernel -> user */
// 			__u64 data; /* kernel <-> user */
// 		} msr;
// 		/* KVM_EXIT_XEN */
// 		struct kvm_xen_exit xen;
// 		/* KVM_EXIT_RISCV_SBI */
// 		struct {
// 			unsigned long extension_id;
// 			unsigned long function_id;
// 			unsigned long args[6];
// 			unsigned long ret[2];
// 		} riscv_sbi;
// 		/* KVM_EXIT_RISCV_CSR */
// 		struct {
// 			unsigned long csr_num;
// 			unsigned long new_value;
// 			unsigned long write_mask;
// 			unsigned long ret_value;
// 		} riscv_csr;
// 		/* KVM_EXIT_NOTIFY */
// 		struct {
// #define KVM_NOTIFY_CONTEXT_INVALID	(1 << 0)
// 			__u32 flags;
// 		} notify;
// 		/* Fix the size of the union. */
// 		char padding[256];
// 	};
//
// 	/* 2048 is the size of the char array used to bound/pad the size
// 	 * of the union that holds sync regs.
// 	 */
// 	#define SYNC_REGS_SIZE_BYTES 2048
// 	/*
// 	 * shared registers between kvm and userspace.
// 	 * kvm_valid_regs specifies the register classes set by the host
// 	 * kvm_dirty_regs specified the register classes dirtied by userspace
// 	 * struct kvm_sync_regs is architecture specific, as well as the
// 	 * bits for kvm_valid_regs and kvm_dirty_regs
// 	 */
// 	__u64 kvm_valid_regs;
// 	__u64 kvm_dirty_regs;
// 	union {
// 		struct kvm_sync_regs regs;
// 		char padding[SYNC_REGS_SIZE_BYTES];
// 	} s;
// };

pub const kvm_run = extern struct {
    request_interrupt_window: u8 = 0,
    immediate_exit: u8 = 0,
    padding1: [6]u8 = [_]u8{0} ** 6,

    exit_reason: u32 = 0,
    ready_for_interrupt_injection: u8 = 0,
    if_flag: u8 = 0,
    flags: u16 = 0,

    cr8: u64 = 0,
    apic_base: u64 = 0,
    vars: extern union {
        io: extern struct {
            direction: u8 = 0,
            size: u8 = 0,
            port: u16 = 0,
            count: u32 = 0,
            data_offset: u64 = 0,
        },
    },
};
