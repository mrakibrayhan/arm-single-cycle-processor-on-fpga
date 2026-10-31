//====================================================================
// decoder_arm.v   (ARMv4 single-cycle CPU, step 5)
//
//   Main decoder + ALU decoder: purely combinational, instr -> control signals.
//   It is the RTL form of  decode()  in arm_ctrl_model.py and of the tables A and B in
//   arm_sc_spec.md section 6.2 (the tables are generated from that code), including every
//   rejected (illegal) encoding. The condition field only matters for cond = 1111 (illegal);
//   the condition check and the write gating are in condlogic_arm.v.
//
//   Signal meanings (spec 6.2):
//     RA1sel   0 Instr[19:16]  1 R15  2 Instr[15:12]            RA3sel 0 Instr[11:8]  1 Instr[15:12]
//     WA1sel   0 Instr[15:12]  1 R14  2 Instr[19:16]            (write port 2 address is always Instr[19:16])
//     BSrc     0 shifter output, 1 extended immediate           ImmSrc 0 rot8, 1 imm12, 2 imm24<<2, 3 imm8 split
//     ShAmtSel 1 = shift amount from Rs      ShDis 1 = shifter pass-through
//     ALUControl = data-processing opcode (ADD/SUB for load/store by U, ADD for branch, MOV for BX)
//     MemAddrSel 1 = ALU result (pre-index), 0 = RD1 (post-index)   MemSize 0 word, 1 half, 2 byte
//     ResSrc   0 ALU, 1 memory data, 2 PC+4, 3 multiplier low     Res2Src 0 ALU result, 1 multiplier high
//     FlagW    {N and Z, C, V} write enables          CSrc 0 adder carry, 1 shifter carry
//     PCS      the instruction writes the PC          PCTSel 0 ALU result, 1 memory data
//     Seq      executed by the sequencer (LDM/STM, UMLAL/SMLAL)
//     Halt     SWI, or "B ." (branch to itself)       BX  branch-and-exchange (checks Rm[0])
//
//   illegal = 1 for an encoding outside the supported subset (spec section 4) OR a class that is switched
//   off in FEATURES. When illegal = 1 the side-effect outputs (RegW1 RegW2 MemW FlagW PCS Halt Seq) are 0, so
//   the instruction changes nothing; the datapath selectors (RA1sel, ALUControl, ...) are then don't-care.
//   The datapath/PC logic stops the CPU and shows 'fault' when illegal = 1 (condition not considered).
//
//   FEATURES (bit = 1: class is built; default all ones = the full core). A class that is off is decoded as
//   illegal. This builds the base core of step 6 and the area ablations of step 16:
//     [0] ALLDP    all 16 data-processing opcodes   (off: only AND SUB ADD ORR)
//     [1] SHIFT    shifted register operands and rotated immediates (off: LSL #0 / rot 0 only)
//     [2] SHREG    shift amount from a register
//     [3] MEMX     LDRB/STRB, register offset, post-index, write-back  (off: LDR/STR imm12, pre-index, no write-back)
//     [4] HALF     LDRH STRH LDRSB LDRSH          [5] BL      [6] BX      [7] SWI
//     [8] MUL      MUL, MLA                        [9] MULL    UMULL SMULL UMLAL SMLAL
//     [10] LDMSTM  LDM / STM
//   (B and B-self halt are always built.)
//====================================================================
module decoder_arm #(
    parameter [10:0] FEATURES = 11'h7FF
) (
    input      [31:0] instr,
    output            illegal,
    output reg [1:0]  RA1sel,
    output reg        RA3sel,
    output reg [1:0]  WA1sel,
    output reg        BSrc,
    output reg [1:0]  ImmSrc,
    output reg        ShAmtSel,
    output reg        ShDis,
    output reg [3:0]  ALUControl,
    output reg        MemAddrSel,
    output            MemR,
    output            MemW,
    output reg [1:0]  MemSize,
    output reg        MemSigned,
    output reg [1:0]  ResSrc,
    output reg        Res2Src,
    output            RegW1,
    output            RegW2,
    output     [2:0]  FlagW,
    output reg        CSrc,
    output            PCS,
    output reg        PCTSel,
    output reg        Mul,
    output reg        MulLong,
    output reg        MulSigned,
    output reg        MulAcc,
    output            Halt,
    output            Seq,
    output reg        BX
);
    localparam ADD = 4'b0100, SUB = 4'b0010, MOV = 4'b1101;

    wire F_ALLDP = FEATURES[0], F_SHIFT = FEATURES[1], F_SHREG = FEATURES[2], F_MEMX = FEATURES[3],
         F_HALF = FEATURES[4], F_BL = FEATURES[5], F_BX = FEATURES[6], F_SWI = FEATURES[7],
         F_MUL = FEATURES[8], F_MULL = FEATURES[9], F_LDM = FEATURES[10];

    // ---------------- instruction fields ----------------
    wire [3:0] cnd = instr[31:28];
    wire [2:0] op  = instr[27:25];
    wire       P = instr[24], U = instr[23], B = instr[22], W = instr[21], L = instr[20];
    wire [3:0] rn = instr[19:16], rd = instr[15:12], rs = instr[11:8], rm = instr[3:0];
    wire [15:0] rl = instr[15:0];
    wire [3:0] opc = instr[24:21];
    wire       wb = ~P | W;                                  // base-register write-back of a load/store

    wire rn15 = (rn == 4'd15), rd15 = (rd == 4'd15), rs15 = (rs == 4'd15), rm15 = (rm == 4'd15);

    // class recognition (same order as the model)
    wire is_mulspace = (op == 3'b000) & (instr[7:4] == 4'b1001);
    wire is_mul      = is_mulspace & (instr[27:22] == 6'b000000);
    wire is_mull     = is_mulspace & (instr[27:23] == 5'b00001);
    wire is_half     = (op == 3'b000) & instr[7] & instr[4] & ~is_mulspace;
    wire is_bx       = (op == 3'b000) & (instr[27:4] == 24'h12FFF1) & ~is_half & ~is_mulspace;
    wire misc_dp     = (instr[24:23] == 2'b10) & ~L;         // MRS/MSR/undefined: not data processing
    wire is_dp_reg   = (op == 3'b000) & ~is_mulspace & ~is_half & ~is_bx & ~misc_dp;
    wire is_dp_imm   = (op == 3'b001) & ~misc_dp;
    wire is_dp       = is_dp_reg | is_dp_imm;
    wire is_mem      = (op == 3'b010) | (op == 3'b011);
    wire is_ldm      = (op == 3'b100);
    wire is_br       = (op == 3'b101);
    wire is_swi      = (op == 3'b111) & P;                   // instr[24]

    // ---------------- internal control (before illegal gating) ----------------
    reg bad;                        // architecturally rejected encoding
    reg unsup;                      // class switched off in FEATURES
    reg regw1, regw2, memw, memr, pcs, halt, seq;
    reg [2:0] flagw;

    // rl helpers for LDM/STM
    wire [15:0] lowbit  = rl & (~rl + 16'd1);                // lowest set bit of the list
    wire        rn_in_list = rl[rn];
    wire        rn_lowest  = (lowbit == (16'd1 << rn));

    // data-processing details
    wire dp_compare = (opc >= 4'd8) & (opc <= 4'd11);
    wire dp_arith   = (opc == 4'd2) | (opc == 4'd3) | (opc == 4'd4) | (opc == 4'd5) | (opc == 4'd6) | (opc == 4'd7) |
                      (opc == 4'd10) | (opc == 4'd11);
    wire dp_regshift = is_dp_reg & instr[4];

    always @(*) begin
        // defaults (match the dict defaults of decode())
        bad = 1'b0; unsup = 1'b0;
        RA1sel = 2'd0; RA3sel = 1'b0; WA1sel = 2'd0; BSrc = 1'b0; ImmSrc = 2'd0; ShAmtSel = 1'b0; ShDis = 1'b0;
        ALUControl = ADD; MemAddrSel = 1'b1; ResSrc = 2'd0; Res2Src = 1'b0;
        regw1 = 1'b0; regw2 = 1'b0; memw = 1'b0; memr = 1'b0; MemSize = 2'd0; MemSigned = 1'b0;
        flagw = 3'b000; CSrc = 1'b0; pcs = 1'b0; PCTSel = 1'b0;
        Mul = 1'b0; MulLong = 1'b0; MulSigned = 1'b0; MulAcc = 1'b0; halt = 1'b0; seq = 1'b0; BX = 1'b0;

        if (cnd == 4'b1111) bad = 1'b1;

        if (is_mul) begin
            // MUL / MLA: Rd = [19:16], accumulate Rn = [15:12]
            if (rn15 | rm15 | rs15 | (instr[21] & rd15) | (rn == rm) | (~instr[21] & (rd != 4'd0))) bad = 1'b1;
            unsup = ~F_MUL;
            RA1sel = 2'd2; Mul = 1'b1; MulAcc = instr[21]; ResSrc = 2'd3; regw1 = 1'b1; WA1sel = 2'd2;
            flagw = L ? 3'b100 : 3'b000;
        end else if (is_mull) begin
            // UMULL UMLAL SMULL SMLAL: RdHi = [19:16], RdLo = [15:12]
            if (rn15 | rd15 | rs15 | rm15 | (rn == rd) | (rn == rm) | (rd == rm)) bad = 1'b1;
            unsup = ~F_MULL;
            RA1sel = 2'd2; Mul = 1'b1; MulLong = 1'b1; MulSigned = B; MulAcc = instr[21]; ResSrc = 2'd3;
            regw1 = 1'b1; regw2 = 1'b1; Res2Src = 1'b1; seq = instr[21];
            flagw = L ? 3'b100 : 3'b000;
        end else if (is_mulspace) begin
            bad = 1'b1;                                          // SWP and the undefined multiply space
        end else if (is_half) begin
            // LDRH STRH LDRSB LDRSH
            if ((~L & (instr[6:5] != 2'b01)) | rd15 | (~B & (rm15 | (rs != 4'd0)))) bad = 1'b1;
            if (wb & rn15) bad = 1'b1;
            if (wb & L & (rn == rd)) bad = 1'b1;
            if (~P & W) bad = 1'b1;
            if (wb & ~L & (rn == rd)) bad = 1'b1;
            unsup = ~F_HALF;
            BSrc = B; ImmSrc = 2'd3; ShDis = 1'b1; RA3sel = 1'b1; MemAddrSel = P;
            ALUControl = U ? ADD : SUB;
            MemSize = (instr[6:5] == 2'b10) ? 2'd2 : 2'd1;
            MemSigned = instr[6];
            regw2 = wb;
            if (L) begin memr = 1'b1; regw1 = 1'b1; ResSrc = 2'd1; end
            else memw = 1'b1;
        end else if (is_bx) begin
            if (rm15) bad = 1'b1;
            unsup = ~F_BX;
            ALUControl = MOV; ShDis = 1'b1; pcs = 1'b1; BX = 1'b1;
        end else if (is_dp) begin
            if (L & rd15) bad = 1'b1;
            if (dp_compare & (rd != 4'd0)) bad = 1'b1;
            if (((opc == 4'd13) | (opc == 4'd15)) & (rn != 4'd0)) bad = 1'b1;
            if (dp_regshift & (rd15 | rn15 | rm15 | rs15)) bad = 1'b1;
            if (~F_ALLDP & (opc != 4'd0) & (opc != 4'd2) & (opc != 4'd4) & (opc != 4'd12)) unsup = 1'b1;
            if (~F_SHIFT & (is_dp_reg ? (~instr[4] & (instr[11:4] != 8'd0)) : (instr[11:8] != 4'd0))) unsup = 1'b1;
            if (~F_SHREG & dp_regshift) unsup = 1'b1;
            ALUControl = opc; BSrc = is_dp_imm; ImmSrc = 2'd0; ShAmtSel = dp_regshift;
            regw1 = ~dp_compare; pcs = ~dp_compare & rd15;
            flagw = L ? {1'b1, 1'b1, dp_arith} : 3'b000;
            CSrc = ~dp_arith;
        end else if (is_mem) begin
            if ((op == 3'b011) & instr[4]) bad = 1'b1;                 // media space
            if (wb & rn15) bad = 1'b1;
            if (wb & L & (rn == rd)) bad = 1'b1;
            if (~P & W) bad = 1'b1;
            if (wb & ~L & (rn == rd)) bad = 1'b1;
            if ((op == 3'b011) & rm15) bad = 1'b1;
            if (B & rd15) bad = 1'b1;
            if (~F_MEMX & ((op == 3'b011) | B | ~P | W)) unsup = 1'b1;
            if (~F_SHIFT & (op == 3'b011) & (instr[11:4] != 8'd0)) unsup = 1'b1;
            ALUControl = U ? ADD : SUB; BSrc = (op == 3'b010); ImmSrc = 2'd1; RA3sel = 1'b1; MemAddrSel = P;
            MemSize = B ? 2'd2 : 2'd0;
            regw2 = wb;
            if (L) begin memr = 1'b1; regw1 = 1'b1; ResSrc = 2'd1; pcs = rd15; PCTSel = 1'b1; end
            else memw = 1'b1;
        end else if (is_ldm) begin
            if (B | (rl == 16'd0) | rn15) bad = 1'b1;                   // S bit, empty list, Rn = PC
            if (L & rl[15] & ~U) bad = 1'b1;                           // LDM loading PC with U = 0
            if (L & W & rn_in_list) bad = 1'b1;
            if (~L & rl[15]) bad = 1'b1;                               // STM of PC
            if (~L & W & rn_in_list & ~rn_lowest) bad = 1'b1;
            unsup = ~F_LDM;
            seq = 1'b1;
        end else if (is_br) begin
            RA1sel = 2'd1; BSrc = 1'b1; ImmSrc = 2'd2; pcs = 1'b1;
            if (P) begin
                unsup = ~F_BL; regw1 = 1'b1; ResSrc = 2'd2; WA1sel = 2'd1;
            end else if (instr[23:0] == 24'hFFFFFE) halt = 1'b1;       // B .  (branch to itself) = HALT
        end else if (is_swi) begin
            unsup = ~F_SWI; halt = 1'b1;
        end else begin
            bad = 1'b1;                                                // coprocessor, MRS/MSR, undefined
        end
    end

    assign illegal = bad | unsup;
    wire ok = ~illegal;
    assign RegW1 = regw1 & ok;
    assign RegW2 = regw2 & ok;
    assign MemW  = memw  & ok;
    assign MemR  = memr  & ok;
    assign FlagW = flagw & {3{ok}};
    assign PCS   = pcs   & ok;
    assign Halt  = halt  & ok;
    assign Seq   = seq   & ok;

endmodule
