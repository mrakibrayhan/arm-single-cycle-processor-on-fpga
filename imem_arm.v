//====================================================================
// imem_arm.v   (ARMv4 single-cycle CPU, step 4)
//
//   Instruction ROM with TWO asynchronous (combinational) read ports.
//   Specification: arm_sc_spec.md sections 3 and D4, D8, D9.
//
//     port A : instruction fetch.   instrA = ROM[ addrA ]   (addrA = PC)
//     port B : data-side read.      dataB  = ROM[ addrB ]   (addrB = DataAddr)
//              Used by loads whose address is in the ROM region: literal pools
//              (LDR Rd,[PC,#off]) and constant arrays. dmem_arm.v does the byte /
//              halfword selection and the sign extension.
//
//   Both ports return the full 32-bit word at word index addr[IMEM_AW+1:2]
//   (the byte address is given; bits [1:0] are ignored, as in the spec: unaligned
//   access is outside the supported subset). An address beyond the ROM wraps (aliases).
//   Read-only: there is no write port.
//
//   There is NO clock: both reads are plain wires from the array, so the instruction
//   is available in the same cycle as the PC (the key difference from imem_riscv.v,
//   which is a synchronous-read M9K ROM). A ROM that can be read asynchronously cannot
//   use an M9K block, so it is built from logic cells (ramstyle / romstyle = logic).
//
//   Contents come from INIT_FILE ($readmemh, one 8-digit hex word per line) at
//   synthesis time (spec D8). The file must contain 2**IMEM_AW lines (assembler_arm.py pads it);
//   in simulation a missing word would be 'x'.
//====================================================================
module imem_arm #(
    parameter IMEM_AW   = 7,                      // 2**7 = 128 words (board build, spec D9)
    parameter INIT_FILE = "program.hex"
) (
    input  [31:0] addrA,                          // byte address, port A (PC)
    output [31:0] instrA,
    input  [31:0] addrB,                          // byte address, port B (DataAddr)
    output [31:0] dataB
);

    (* ramstyle = "logic", romstyle = "logic" *) reg [31:0] mem [0:(1 << IMEM_AW) - 1];

    initial $readmemh(INIT_FILE, mem);          // same style as imem_riscv.v; the file must have 2**IMEM_AW lines

    assign instrA = mem[addrA[IMEM_AW+1:2]];
    assign dataB  = mem[addrB[IMEM_AW+1:2]];

endmodule
