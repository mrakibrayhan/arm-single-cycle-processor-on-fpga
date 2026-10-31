//====================================================================
// ext_arm.v   (ARMv4 single-cycle CPU, step 6)  -  immediate "Extend" unit
//
//   ImmSrc 0 : rot8   imm8 rotated right by 2 x Instr[11:8]   (data processing, op = 001)
//   ImmSrc 1 : imm12  Instr[11:0] zero-extended               (LDR / STR immediate offset)
//   ImmSrc 2 : imm24  signext(Instr[23:0]) << 2               (B / BL)
//   ImmSrc 3 : imm8   {Instr[11:8], Instr[3:0]} zero-extended (LDRH / STRH / LDRSB / LDRSH immediate)
//   ROT_EN = 0 (base core, FEATURES[1] = 0): the rotation is not built, rot8 = imm8 zero-extended
//   (the decoder rejects rotated immediates in that case).
//   'rotc' = the rotated value's bit 31 (the shifter carry of a rotated immediate, step 9).
//====================================================================
module ext_arm #(
    parameter ROT_EN = 0
) (
    input  [23:0] instr24,       // Instr[23:0]
    input  [1:0]  ImmSrc,
    output reg [31:0] ext
);
    wire [7:0]  imm8 = instr24[7:0];
    wire [3:0]  rot  = instr24[11:8];
    wire [31:0] rot8;
    generate
        if (ROT_EN) begin : g_rot
            wire [63:0] dbl = {24'd0, imm8, 24'd0, imm8};
            assign rot8 = dbl[{rot, 1'b0} +: 32];             // ror(imm8, 2 x rot)
        end else begin : g_norot
            assign rot8 = {24'd0, imm8};
        end
    endgenerate

    always @(*) begin
        case (ImmSrc)
            2'd0:    ext = rot8;
            2'd1:    ext = {20'd0, instr24[11:0]};
            2'd2:    ext = {{6{instr24[23]}}, instr24, 2'b00};
            default: ext = {24'd0, instr24[11:8], instr24[3:0]};
        endcase
    end
endmodule
