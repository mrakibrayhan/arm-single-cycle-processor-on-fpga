//====================================================================
// shifter_arm.v   (ARMv4 single-cycle CPU, step 9)  -  barrel shifter of data-processing operand 2
//
//   dout = din shifted / rotated, cout = shifter carry out.
//
//     typ      00 LSL   01 LSR   10 ASR   11 ROR          (Instr[6:5])
//     amt_imm  Instr[11:7]                                   (ShAmtSel = 0, shift by immediate)
//     amt_reg  Rs[7:0] = RD3[7:0]                            (ShAmtSel = 1, shift by register)
//     dis      1 = pass-through (dout = din, cout = cin)      (ShDis: LDRH-class offsets, BX)
//     cin      the old C flag
//
//   ARM rules (DDI 0100 A5.1):
//     immediate:  LSL #0 = no shift, cout = C          LSR #0 = LSR #32       ASR #0 = ASR #32      ROR #0 = RRX
//     register :  amount 0 = no shift, cout = C (all four types);  LSL/LSR 32: result 0, cout = bit 0 / bit 31;
//                 LSL/LSR > 32: result 0, cout 0;  ASR >= 32: result and cout = sign;
//                 ROR uses amount mod 32 (a multiple of 32 leaves the value, cout = bit 31).
//
//   Structure: ONE right shifter. LSL = reverse, shift right, reverse. The shifter works on a 65-bit vector
//   {upper, x, 0}: upper = x (ROR), the sign (ASR) or 0 (LSL, LSR). After the right shift by n, bits [32:1] are
//   the result and bit 0 is x[n-1] = the carry out. Amounts >= 32 and the special cases are decoded on the side.
//
//   SH_EN  = 0 : no shifter at all (dout = din, cout = cin)                         [FEATURES[1] = 0]
//   REG_EN = 0 : the register amount is not built (the decoder rejects those instructions) [FEATURES[2] = 0]
//====================================================================
module shifter_arm #(
    parameter SH_EN  = 1,
    parameter REG_EN = 1
) (
    input  [31:0] din,
    input  [1:0]  typ,
    input  [4:0]  amt_imm,
    input  [7:0]  amt_reg,
    input         by_reg,        // ShAmtSel
    input         dis,           // ShDis
    input         cin,
    output [31:0] dout,
    output        cout
);
    generate
        if (!SH_EN) begin : g_none
            assign dout = din;
            assign cout = cin;
        end else begin : g_sh
            wire is_lsl = (typ == 2'd0), is_lsr = (typ == 2'd1), is_asr = (typ == 2'd2), is_ror = (typ == 2'd3);
            wire reg_mode = REG_EN ? by_reg : 1'b0;

            // ---------------- amount decode ----------------
            wire        reg_zero  = (amt_reg == 8'd0);
            wire        reg_big   = |amt_reg[7:5];                       // >= 32
            wire        reg_eq32  = (amt_reg == 8'd32);
            wire        imm_zero  = (amt_imm == 5'd0);
            wire [4:0]  n         = reg_mode ? amt_reg[4:0] : amt_imm;

            wire pass  = dis | (reg_mode ? reg_zero : (imm_zero & is_lsl));
            wire rrx   = ~dis & ~reg_mode & imm_zero & is_ror;
            // amount of 32 or more for LSL / LSR / ASR (register), or the immediate #0 forms of LSR / ASR
            wire big   = ~dis & ~is_ror & (reg_mode ? reg_big : (imm_zero & ~is_lsl));
            wire eq32  = reg_mode ? reg_eq32 : 1'b1;

            // ---------------- one right shifter ----------------
            wire [31:0] rev_in = {din[0], din[1], din[2], din[3], din[4], din[5], din[6], din[7], din[8], din[9], din[10],
                                  din[11], din[12], din[13], din[14], din[15], din[16], din[17], din[18], din[19], din[20],
                                  din[21], din[22], din[23], din[24], din[25], din[26], din[27], din[28], din[29], din[30], din[31]};
            wire [31:0] x     = is_lsl ? rev_in : din;
            wire        fill  = is_asr & din[31];
            wire [31:0] upper = is_ror ? x : {32{fill}};
            wire [64:0] ext   = {upper, x, 1'b0};
            wire [64:0] sh    = ext >> n;
            wire [31:0] y     = sh[32:1];
            wire [31:0] y_rev = {y[0], y[1], y[2], y[3], y[4], y[5], y[6], y[7], y[8], y[9], y[10], y[11], y[12], y[13], y[14],
                                 y[15], y[16], y[17], y[18], y[19], y[20], y[21], y[22], y[23], y[24], y[25], y[26], y[27],
                                 y[28], y[29], y[30], y[31]};
            wire [31:0] normal = is_lsl ? y_rev : y;
            // carry of the normal case: the last bit shifted out; for ROR it is bit 31 of the result
            wire        c_norm = is_ror ? y[31] : sh[0];

            // ---------------- result / carry select ----------------
            wire [31:0] big_val = {32{fill}};                            // 0 for LSL / LSR, sign for ASR
            wire        big_c   = is_asr ? din[31] : (eq32 & din[31] & ~is_lsl) | (eq32 & din[0] & is_lsl);

            assign dout = pass ? din : rrx ? {cin, din[31:1]} : big ? big_val : normal;
            assign cout = pass ? cin : rrx ? din[0]           : big ? big_c   : c_norm;
        end
    endgenerate
endmodule
