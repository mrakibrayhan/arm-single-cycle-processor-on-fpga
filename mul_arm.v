//====================================================================
// mul_arm.v   (ARMv4 single-cycle CPU, step 12)  -  multiplier and multiply-accumulate adders
//
//   p = a * b (a = Rm = RD2, b = Rs = RD3), 32 x 32 -> 64 bits, unsigned or signed (UMULL / SMULL).
//   lo = p[31:0] + (acc ? acc_lo : 0)                    MLA accumulate, and beat 0 of UMLAL/SMLAL (acc_lo = RdLo)
//   hi = p[63:32] + (acc & long ? acc_hi : 0) + cin       beat 1 of UMLAL/SMLAL (acc_hi = RdHi, cin = carry of beat 0)
//   cout = carry out of the lo addition (saved between the two beats by the datapath)
//
//   MUL_EN  = 0 : no multiplier at all (outputs 0)                               [FEATURES[8] = FEATURES[9] = 0]
//   LONG_EN = 0 : only the low 32 bits are built (MUL / MLA; the low half of a
//                 product is the same for signed and unsigned operands)          [FEATURES[9] = 0]
//   In an FPGA the product maps to the embedded 18x18 multipliers (a 33 x 33 signed product for the 64-bit case).
//====================================================================
module mul_arm #(
    parameter MUL_EN  = 1,
    parameter LONG_EN = 1
) (
    input  [31:0] a,
    input  [31:0] b,
    input         sgn,           // MulSigned (SMULL / SMLAL)
    input         acc,           // MulAcc
    input         long_op,       // MulLong
    input  [31:0] acc_lo,        // RD1 in beat 0 (Rn of MLA, RdLo of xMLAL)
    input  [31:0] acc_hi,        // RD1 in beat 1 (RdHi of xMLAL)
    input         cin,           // carry from beat 0 (xMLAL beat 1 only, else 0)
    output [31:0] lo,
    output [31:0] hi,
    output        cout
);
    generate
        if (!MUL_EN) begin : g_none
            assign lo = 32'd0; assign hi = 32'd0; assign cout = 1'b0;
        end else if (!LONG_EN) begin : g_short
            wire [31:0] p = a * b;
            wire [32:0] s = {1'b0, p} + {1'b0, acc ? acc_lo : 32'd0};
            assign lo = s[31:0]; assign cout = s[32]; assign hi = 32'd0;
        end else begin : g_long
            wire signed [32:0] sa = {sgn & a[31], a};
            wire signed [32:0] sb = {sgn & b[31], b};
            wire signed [65:0] sp = sa * sb;
            wire [63:0] p = sp[63:0];
            wire [32:0] s = {1'b0, p[31:0]} + {1'b0, acc ? acc_lo : 32'd0};
            assign lo   = s[31:0];
            assign cout = s[32];
            assign hi   = p[63:32] + ((acc & long_op) ? acc_hi : 32'd0) + {31'd0, cin};
        end
    endgenerate
endmodule
