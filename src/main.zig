//!
//! main.zig
//!
//! Caleb Barger
//! 10/27/23
//!
//! Simulator for the atmega328
//!

// NOTE(caleb): questions.
// =============================
// - General registers are "mapped" into dmem starting at addr 0?
// - When does the instruction processing cycle by convention terminate?
// - Cycles? time???
// - What is ment exactly by write back. Does that include SREG updates?
// - Do sreg fields i.e. xor that depend on other sreg fields compute before or after sreg writes.

// TODO(caleb):
// - Finish SREG operations
// - Conventions for decoding sections of instructions, naming, impl, org...

const std = @import("std");
const elf = std.elf;
const rl = @import("rl.zig");

const c = @cImport({
    @cInclude("ncurses.h");
});

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

const SysCall = enum(u14) {
    hello_world = 0x3fff,
};

const Instruction = enum {
    // ALU instructions
    add,
    nop,
    cp,
    adc,
    @"and",
    eor,
    @"or",
    mov,

    call,
    jmp,

    ret,
    reti,

    // Immediate
    ldi,
};

fn instrFromOpCode(op_code: u16) Instruction {
    return switch (@as(u4, @truncate(op_code >> 12))) {
        0x0 => switch (@as(u2, @truncate(op_code >> 10))) {
            0x3 => .add,
            0x0 => .nop,
            else => unreachable,
        },
        0x1 => switch (@as(u2, @truncate(op_code >> 10))) {
            0x1 => .cp,
            0x3 => .adc,
            else => unreachable,
        },
        0x2 => switch (@as(u2, @truncate(op_code >> 10))) {
            0x0 => .@"and",
            0x1 => .eor,
            0x2 => .@"or",
            0x3 => .mov,
        },

        0x9 => switch (@as(u4, @truncate(op_code >> 8))) {
            0x4 => switch (@as(u4, @truncate(op_code))) {
                0xe => .call,
                0xc => .jmp,
                else => unreachable,
            },
            0x5 => switch (@as(u1, @truncate(op_code >> 4))) {
                0x0 => .ret,
                0x1 => .reti,
            },
            else => unreachable,
        },

        0xe => .ldi,
        else => unreachable,
    };
}

fn KB(n: usize) usize {
    return n * 1024;
}

const stack_end: u16 = 0x08ff;

const with_curses = true;

pub fn main() !void {
    const screen_width: c_int = 640;
    const screen_height: c_int = 576;

    rl.InitWindow(screen_width, screen_height, "Sim328");
    rl.SetConfigFlags(rl.FLAG_MSAA_4X_HINT);
    rl.SetWindowState(rl.FLAG_WINDOW_RESIZABLE);
    rl.SetTargetFPS(60);

    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    var scratch_arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    var scratch_ally = scratch_arena.allocator();
    var ally = arena.allocator();

    // Curses init
    if (with_curses)
        _ = c.initscr();

    ////////////////////

    //     Data memory
    // --------------------
    // |     32 regs      | 0x0000 - 0x001f
    // --------------------
    // |   64 I/O regs    | 0x0020 - 0x005f
    // --------------------
    // | 160 Ext I/O regs | 0x0060 - 0x00ff
    // --------------------
    // |  Internal sram   | 0x0100 - 0x08ff
    // --------------------

    var dmem = try ally.alloc(u8, 256 + KB(2));
    for (dmem) |*byte| byte.* = 0;

    // General purpose registers
    var gp_regs: []u8 = dmem[0x0..0x20];

    // 16-bit address pointers
    // const x: *u16 = @ptrCast(@alignCast(gp_regs[0x1a..0x1c]));
    // _ = x;
    // const xl: *u8 = &gp_regs[0x1a];
    // _ = xl;
    // const xh: *u8 = &gp_regs[0x1b];
    // _ = xh;

    // const y: *u16 = @ptrCast(@alignCast(gp_regs[0x1c..0x1e]));
    // _ = y;
    // const yl: *u8 = &gp_regs[0x1c];
    // const yh: *u8 = &gp_regs[0x1d];
    // _ = yh;
    // _ = yl;

    // const z: *u16 = @ptrCast(@alignCast(gp_regs[0x1e..0x20]));
    // _ = z;
    // const zl: *u8 = &gp_regs[0x1e];
    // _ = zl;
    // const zh: *u8 = &gp_regs[0x1f];
    // _ = zh;

    ////////////////////

    // I/O registers
    var io_regs: []u8 = dmem[0x20..0x60];

    // Stack pointer

    const sp1: *u16 = @ptrCast(@alignCast(io_regs[0x3d..0x3f]));
    _ = sp1;

    const spl: *u8 = &io_regs[0x3d];
    const sph: *u8 = &io_regs[0x3e];
    spl.* = stack_end & 0xff;
    sph.* = stack_end >> 8;

    // Status register
    var sreg: *SReg = @ptrCast(&dmem[0x3f]);

    ////////////////////

    // Program counter
    var pc: u14 = 0;

    //        Program memory
    // -----------------------------
    // | Application flash section | 0x0000-0x37ff
    // -----------------------------
    // |    Boot flash section     | 0x3800-0x3fff
    // -----------------------------
    var pmem = try ally.alloc(u16, 0x4000);
    for (pmem) |*byte| byte.* = 0;

    ////////////////////

    // Load instructions into pmem
    {
        const restore_state = scratch_arena.state;
        defer scratch_arena.state = restore_state;

        var args_iter = std.process.args();
        _ = args_iter.skip();
        const elf_path = args_iter.next() orelse unreachable;

        var elf_file = try std.fs.cwd().openFileZ(elf_path, .{});
        defer elf_file.close();

        var elf_bytes = try elf_file.readToEndAlloc(scratch_arena.allocator(), 1024 * 10);

        const elf_header = try elf.Header.read(elf_file);

        // HACK(caleb): Write insturctions to pmem after finding the first
        // loadable program header with an executable flag.
        var program_header_iter = elf_header.program_header_iterator(elf_file);
        while (try program_header_iter.next()) |p_header| {
            if ((p_header.p_type == elf.PT_LOAD) and
                ((p_header.p_flags & elf.PF_X) != 0))
            {
                var pmem_index: usize = 0;
                var byte_index: usize = 0;
                while (byte_index < p_header.p_memsz) : (byte_index += 2) {
                    const first_byte_index = p_header.p_offset + byte_index;
                    pmem[pmem_index] = std.mem.readIntSlice(
                        u16,
                        elf_bytes[first_byte_index .. first_byte_index + 2],
                        .Little,
                    );
                    pmem_index += 1;
                }
                break; // Instructions loaded. Move on.
            }
        }
    }

    ////////////////////

    // Instruction processing loop

    while (pc < 0x4000) { //pmem[pc] != 0) { // FIXME(caleb): How is this normally done?
        if (pc > 0x3ffd) { // Handle sys call
            switch (@as(SysCall, @enumFromInt(pc))) {
                .hello_world => std.debug.print("Hello, world!\n", .{}),
            }

            const sp = ((@as(u16, @intCast(sph.*))) << 8) | spl.*;

            // Pop return address
            const pch: u8 = dmem[sp + 1]; // sph
            const pcl: u8 = dmem[sp + 2]; // spl

            // Increment sp
            sph.* = @truncate((sp + 2) >> 8);
            spl.* = @truncate(sp + 2);

            // Restore pc
            pc = (@as(u14, @intCast(pch)) << 8) | pcl;
        } else { // Handle avr instruction
            // 1) Fetch instruction from memory NOTE(caleb): the implicit pmem[pc]
            // 2) Decode instructions
            switch (instrFromOpCode(pmem[pc])) {
                .add => {
                    // Source/Dest registers
                    const source_regh: u8 = @truncate((pmem[pc] >> 5) & (1 << 4));
                    const source_regl: u8 = @truncate(pmem[pc] & 0x0f);
                    const dest_reg: u5 = @truncate(pmem[pc] >> 4);

                    // Do the add
                    const result = @addWithOverflow(gp_regs[dest_reg], gp_regs[source_regh | source_regl]);
                    gp_regs[dest_reg] = result[0];

                    // Update SREG
                    sreg.c = result[1];
                    sreg.n = @intCast(gp_regs[dest_reg] >> 7);
                    sreg.z = @intFromBool(gp_regs[dest_reg] == 0);

                    pc += 1;
                },
                .nop => pc += 1, // No operation..
                .cp => {
                    // Source/Dest registers
                    const source_regh: u8 = @truncate((pmem[pc] >> 5) & (1 << 4));
                    const source_regl: u8 = @truncate(pmem[pc] & 0x0f);
                    const dest_reg: u5 = @truncate(pmem[pc] >> 4);

                    // Do the add
                    const result = @subWithOverflow(gp_regs[dest_reg], gp_regs[source_regh | source_regl]);
                    gp_regs[dest_reg] = result[0];

                    // Update SREG
                    sreg.n = @intCast(gp_regs[dest_reg] >> 7);
                    sreg.z = @intFromBool(gp_regs[dest_reg] == 0);

                    pc += 1;
                },
                .adc => {
                    // Source/Dest registers
                    const source_regh: u8 = @truncate((pmem[pc] >> 5) & (1 << 4));
                    const source_regl: u8 = @truncate(pmem[pc] & 0x0f);
                    const dest_reg: u5 = @truncate(pmem[pc] >> 4);

                    // Do the add
                    const result = @addWithOverflow(gp_regs[dest_reg], @addWithOverflow(gp_regs[source_regh | source_regl], sreg.c)[0]);
                    gp_regs[dest_reg] = result[0];

                    // Update SREG
                    sreg.c = result[1];
                    sreg.n = @intCast(gp_regs[dest_reg] >> 7);
                    sreg.z = @intFromBool(gp_regs[dest_reg] == 0);

                    pc += 1;
                },
                .@"and" => {
                    // Source/Dest registers
                    const source_regh: u8 = @truncate((pmem[pc] >> 5) & (1 << 4));
                    const source_regl: u8 = @truncate(pmem[pc] & 0x0f);
                    const dest_reg: u5 = @truncate(pmem[pc] >> 4);

                    // Logical and
                    gp_regs[dest_reg] = gp_regs[dest_reg] & gp_regs[source_regh | source_regl];

                    sreg.v = 0;
                    sreg.n = @intCast(gp_regs[dest_reg] >> 7);
                    sreg.z = @intFromBool(gp_regs[dest_reg] == 0);
                    sreg.s = sreg.n ^ sreg.v;

                    pc += 1;
                },
                .call => {
                    const sp = ((@as(u16, @intCast(sph.*))) << 8) | spl.*;

                    // Push return address
                    dmem[sp - 1] = @truncate((pc + 2) >> 8); // sph
                    dmem[sp] = @truncate(pc + 2); // spl

                    // Decrement stack pointer
                    sph.* = @truncate((sp - 2) >> 8);
                    spl.* = @truncate(sp - 2);

                    // Update pc
                    pc = @truncate(pmem[pc + 1]);
                },
                .jmp => {
                    // Update pc
                    pc = @truncate(pmem[pc + 1]);
                },
                .ret => {
                    const sp = ((@as(u16, @intCast(sph.*))) << 8) | spl.*;

                    // Pop return address
                    const pch: u8 = dmem[sp + 1]; // sph
                    const pcl: u8 = dmem[sp + 2]; // spl

                    // Increment sp
                    sph.* = @truncate((sp + 2) >> 8);
                    spl.* = @truncate(sp + 2);

                    // Restore pc
                    pc = (@as(u14, @intCast(pch)) << 8) | pcl;
                },

                .ldi => {
                    // Constant data
                    const cdh: u8 = @intCast((pmem[pc] >> 4) & 0x00f0);
                    const cdl: u8 = @intCast(pmem[pc] & 0x000f);

                    // Dest register
                    const dest_reg_index: u16 = ((pmem[pc] >> 4) & 0x000f) + 0x10;

                    // Execute operation
                    gp_regs[dest_reg_index] = cdh | cdl;

                    // Store the result
                    // NOTE(caleb): Normally write back to sram here??
                    // Things like update SREG??

                    pc += 1;
                },
                else => break, // Not implemented or "bad" instruction
            }
        }

        // Print registers

        // std.debug.print("x: {x}\n", .{x.*});
        // std.debug.print("xh: {x}\n", .{xh.*});
        // std.debug.print("xl: {x}\n", .{xl.*});

        // std.debug.print("y: {x}\n", .{y.*});
        // std.debug.print("yh: {x}\n", .{yh.*});
        // std.debug.print("yl: {x}\n", .{yl.*});

        // std.debug.print("z: {x}\n", .{z.*});
        // std.debug.print("zh: {x}\n", .{zh.*});
        // std.debug.print("zl: {x}\n", .{zl.*});

        if (with_curses) {
            const restore_state = scratch_arena.state;
            defer scratch_arena.state = restore_state;
            var print_buf = try scratch_ally.alloc(u8, 256);

            const pcz = try std.fmt.bufPrintZ(print_buf, "pc: {x}", .{pc});
            _ = c.mvaddstr(0, 0, pcz);

            const sphz = try std.fmt.bufPrintZ(print_buf, "sph: {x}", .{sph.*});
            _ = c.mvaddstr(1, 0, sphz);
            const splz = try std.fmt.bufPrintZ(print_buf, "spl: {x}", .{spl.*});
            _ = c.mvaddstr(2, 0, splz);

            const sregz = try std.fmt.bufPrintZ(print_buf, "sreg - {b:0>8}", .{@as(u8, @bitCast(sreg.*))});
            _ = c.mvaddstr(3, 0, sregz);

            for (gp_regs, 0..) |val, reg_index| {
                const gp_regz = try std.fmt.bufPrintZ(print_buf, "r{d}: {x}", .{ reg_index, val });
                _ = c.mvaddstr(@intCast(4 + reg_index), 0, gp_regz);
            }

            _ = c.refresh();
            _ = c.getch();
        }
    }
    if (with_curses)
        _ = c.endwin();
}
