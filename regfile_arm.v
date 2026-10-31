//====================================================================
// regfile_arm.v   (ARMv4 single-cycle CPU, step 3)
//
//   Register file R0 - R14 with THREE asynchronous (combinational) read ports and
//   TWO write ports. Specification: arm_sc_spec.md sections 3, 5 and D2/D3.
//
//   R15 is the PC and is not stored here: a read of address 15 returns the input
//   'r15' (the datapath connects PCPlus8, so an instruction sees its own address + 8).
//   A write to address 15 is ignored (it is a PC update, done by the PC logic).
//
//   Read ports (the address muxes RA1sel / RA3sel are outside, in the datapath):
//       rd1 = R[ra1]   rd2 = R[ra2]   rd3 = R[ra3]
//   Reads are plain wires from the register outputs: no clock, and no write-through
//   bypass. A value written at a rising clock edge is visible after that edge; during
//   the cycle in which it is written, a read of the same register returns the OLD value
//   (exactly the single-cycle textbook behaviour).
//
//   Write ports (both act at the rising edge of clk):
//       port 1: we1, wa1, wd1   (Result1: ALU / load data / PCPlus4 / MulLo)
//       port 2: we2, wa2, wd2   (Result2: new base register, or RdHi)
//   The decoder never writes the same register through both ports in one cycle
//   (those encodings are UNPREDICTABLE and rejected). If it ever happened, port 1 wins.
//
//   rst (synchronous, active high) clears all 15 registers in ONE clock cycle (the
//   registers are flip-flops with a synchronous clear, not a RAM, so there is no
//   32-cycle clear sequence as in the synchronous-read RISC-V cores).
//
//   Parameter RD3_EN = 0 removes the third read port (rd3 = 0): an area ablation for the paper.
//
//   Cyclone IV: 15 x 32 = 480 flip-flops (one logic element each, the write data select
//   shares the LE's lookup table) plus three 15:1 read multiplexers of 32 bits each.
//====================================================================
module regfile_arm #(
    parameter RD3_EN = 1     // 1 = third read port present (spec D2). 0 = rd3 tied to 0 and its 15:1 muxes removed
) (                          // (area ablation; the datapath must then handle 3-operand instructions in two beats)
    input         clk,
    input         rst,
    // read ports
    input  [3:0]  ra1,
    input  [3:0]  ra2,
    input  [3:0]  ra3,
    output [31:0] rd1,
    output [31:0] rd2,
    output [31:0] rd3,
    input  [31:0] r15,       // value returned for address 15 (PCPlus8)
    // write port 1
    input         we1,
    input  [3:0]  wa1,
    input  [31:0] wd1,
    // write port 2
    input         we2,
    input  [3:0]  wa2,
    input  [31:0] wd2
);

    reg [31:0] r [0:14];

    integer i;
    initial for (i = 0; i < 15; i = i + 1) r[i] = 32'd0;     // simulation power-up (FPGA registers start at 0)

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 15; i = i + 1) r[i] <= 32'd0;
        end else begin
            if (we2 && wa2 != 4'd15) r[wa2] <= wd2;
            if (we1 && wa1 != 4'd15) r[wa1] <= wd1;          // written last: port 1 wins on a conflict
        end
    end

    assign rd1 = (ra1 == 4'd15) ? r15 : r[ra1];
    assign rd2 = (ra2 == 4'd15) ? r15 : r[ra2];
    assign rd3 = !RD3_EN ? 32'd0 : (ra3 == 4'd15) ? r15 : r[ra3];

endmodule
