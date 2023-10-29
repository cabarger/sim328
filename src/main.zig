//!
//! main.zig
//!
//! Caleb Barger
//! 10/27/23
//! Linux x86_64
//!
//! Simulator for the atmega328
//!

const std = @import("std");

const SReg = packed struct {
    /// Carry flag
    c: u1,
    /// Zero flag
    z: u1,
    /// Negative flag
    n: u1,
    /// Two's complement overflow flag
    v: u1,
    /// Sign bit
    s: u1,
    /// Half carry flag
    h: u1,
    /// Bit copy storage
    t: u1,
    /// Global interrupt enable
    i: u1,
};

const Instruction = enum {
    add,
    nop,
    ldi,
};

fn KB(n: usize) usize {
    return n * 1024;
}

const stack_end: u16 = 0x08ff;

pub fn main() !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    var ally = arena.allocator();

    ////////////////////

    //     Data memory
    // --------------------
    // |     32 regs      | 0x0000 - 0x001f
    // --------------------
    // |   64 I/O regs    | 0x0020 - 0x005f
    // --------------------
    // | 160 Ext I/O regs | 0x0060 - 0x00ff
    // --------------------
    // |  Internal sram   | 0x08ff
    // --------------------

    var dmem = try ally.alloc(u8, 256 + KB(2));
    for (dmem) |*byte| byte.* = 0;

    // General purpose registers
    var gp_regs: []u8 = dmem[0x0..0x20];

    // 16-bit address pointers
    const x: *u16 = @ptrCast(@alignCast(gp_regs[0x1a..0x1c]));
    const xl: *u8 = &gp_regs[0x1a];
    _ = xl;
    const xh: *u8 = &gp_regs[0x1b];
    _ = xh;

    const y: *u16 = @ptrCast(@alignCast(gp_regs[0x1c..0x1e]));
    const yl: *u8 = &gp_regs[0x1c];
    const yh: *u8 = &gp_regs[0x1d];
    _ = yh;
    _ = yl;

    const z: *u16 = @ptrCast(@alignCast(gp_regs[0x1e..0x20]));
    const zl: *u8 = &gp_regs[0x1e];
    _ = zl;
    const zh: *u8 = &gp_regs[0x1f];
    _ = zh;

    ////////////////////

    // I/O registers
    var io_regs: []u8 = dmem[0x20..0x60];

    // Stack pointer
    // FIXME(caleb): Panic incorrect alignment??
    // const sp: *u16 = @ptrCast(@alignCast(io_regs[0x3d..0x3f]));
    const sph: *u8 = &io_regs[0x3e];
    const spl: *u8 = &io_regs[0x3d];
    sph.* = stack_end >> 8;
    spl.* = stack_end & 0xff;

    // Status register
    var sreg: *SReg = @ptrCast(&dmem[0x3f]);

    ////////////////////

    // Program counter
    var pc: u16 = 0;

    //        Program memory
    // -----------------------------
    // | Application flash section | 0x0000
    // -----------------------------
    // |    Boot flash section     | 0x3FFF
    // -----------------------------
    var pmem = try ally.alloc(u16, 0x4000);
    for (pmem) |*byte| byte.* = 0;

    ////////////////////

    // Process instructions!!!

    pmem[0] = 0xe001; // ldi r16, 1
    pmem[1] = 0xe012; // ldi r16, 2

    while (pmem[pc] != 0) {
        // 1) Fetch instruction from memory
        const op1: u4 = @intCast(pmem[pc] >> 12);

        // 2) Decode instructions
        var instruction: Instruction = undefined;
        switch (op1) {
            0x0 => {
                // add, nop
                const op2: u2 = @truncate(pmem[pc] >> 10);
                switch (op2) {
                    0x3 => instruction = .add,
                    0x0 => instruction = .nop,
                    else => unreachable,
                }
            },
            0x1 => {
                // cp, adc
            },
            0x2 => {
                // and, eor, or, mov
            },
            0xe => instruction = .ldi,
            else => unreachable,
        }

        switch (instruction) {
            .add => unreachable,
            .ldi => {
                // 3) Calculate effective address of operand
                // NOTE(caleb): Nothing in this case since data is constant

                // 4) Fetch operands from memory

                // Constant data
                const cdh: u8 = @intCast((pmem[pc] >> 4) & 0x00f0);
                const cdl: u8 = @intCast(pmem[pc] & 0x000f);

                // Dest register
                const dest_reg_index: u16 = ((pmem[pc] >> 4) & 0x000f) + 0x10;

                // 5) Execute operation
                gp_regs[dest_reg_index] = cdh | cdl;

                // 6) Store the result
                // NOTE(caleb): Normally write back to sram here??
            },
            .nop => unreachable,
        }

        pc += 1; // Increment program counter TODO(caleb): On a per instruction basis
    }

    ////////////////////

    // Print registers

    std.debug.print("sph: {x}\n", .{sph.*});
    std.debug.print("spl: {x}\n", .{spl.*});

    std.debug.print("sreg:\n", .{});
    std.debug.print("  ithsvnzc\n", .{});
    std.debug.print("  {b:0>8}\n", .{@as(u8, @bitCast(sreg.*))});

    for (gp_regs, 0..) |val, reg_index|
        std.debug.print("r{d}: {x}\n", .{ reg_index, val });

    std.debug.print("x: {x}\n", .{x.*});
    // std.debug.print("xh: {x}\n", .{xh.*});
    // std.debug.print("xl: {x}\n", .{xl.*});

    std.debug.print("y: {x}\n", .{y.*});
    // std.debug.print("yh: {x}\n", .{yh.*});
    // std.debug.print("yl: {x}\n", .{yl.*});

    std.debug.print("z: {x}\n", .{z.*});
    // std.debug.print("zh: {x}\n", .{zh.*});
    // std.debug.print("zl: {x}\n", .{zl.*});
}
