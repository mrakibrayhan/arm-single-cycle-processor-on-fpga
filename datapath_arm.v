//====================================================================
// datapath_arm.v   (ARMv4 single-cycle CPU, step 6)
//
//   Datapath of the single-cycle core (spec section 5, figure fig_arm_sc_datapath): PC register and adders,
//   register file (3 read / 2 write, R15 = PC + 8), extend unit, ALU, data address mux, result muxes,
//   PC target mux. All control inputs come from decoder_arm / condlogic_arm (already gated by CondEx).
//   Instruction ROM and data RAM are outside (cpu_core_arm.v): this module gives the PC and the data
//   address / store data and takes the instruction and the load data (asynchronous memories: the same cycle).
//
//   Step 6 built the base core without the barrel shifter; step 9 adds shifter_arm.v (SH_EN = FEATURES[1],
//   SHREG_EN = FEATURES[2]) and the rotated immediate (ROT_EN = FEATURES[1]). With SH_EN = 0 the shifter output is RD2
//   (LSL #0) and the hardware is the step-6 base core. Step 12 adds mul_arm.v (MUL_EN, MULL_EN) and the two-beat
//   UMLAL/SMLAL registers lo_r / c_r; step 13 adds the LDM/STM sequencer (ldmstm_seq_arm.v, LDM_EN). The decoder is built with FEATURES that switch the missing classes off (BUILT in cpu_core_arm.v),
//   so they are never executed.
//====================================================================
module datapath_arm #(
    parameter RD3_EN   = 1,
    parameter ROT_EN   = 0,          // rotated immediates      (FEATURES[1])
    parameter SH_EN    = 0,          // barrel shifter          (FEATURES[1])
    parameter SHREG_EN = 0,          // shift amount from Rs    (FEATURES[2])
    parameter MUL_EN   = 0,          // multiplier MUL / MLA     (FEATURES[8] or [9])
    parameter MULL_EN  = 0,          // 64-bit products, UMLAL/SMLAL beat register (FEATURES[9])
    parameter LDM_EN   = 0           // LDM / STM sequencer        (FEATURES[10])
) (
    input         clk,
    input         rst,
    // memories
    input  [31:0] instr,          // Instr = ROM[pc]
    output [31:0] pc,
    output [31:0] daddr,          // DataAddr
    output [31:0] wdata,          // store data (RD3)
    input  [31:0] rdata,          // load data (aligned / extended by dmem_arm)
    // control (from the decoder; RegWrite1/2, PCSrc are gated by CondEx and run)
    input  [1:0]  RA1sel,
    input         RA3sel,
    input  [1:0]  WA1sel,
    input         BSrc,
    input  [1:0]  ImmSrc,
    input  [3:0]  ALUControl,
    input         MemAddrSel,
    input  [1:0]  ResSrc,
    input         Res2Src,
    input         RegWrite1,
    input         RegWrite2,
    input         CSrc,
    input         ShAmtSel,
    input         ShDis,
    input         PCSrc,
    input         PCTSel,
    input         pc_en,          // 0 = hold the PC (halted, fault, beat 0 of UMLAL/SMLAL)
    input         Mul, MulLong, MulSigned, MulAcc,
    input         mul_beat1,      // 1 = second beat of UMLAL / SMLAL
    // LDM / STM (step 13): an executed LDM/STM in the core; its controls come from the sequencer
    input         ldm_act,
    output        ldm_last,
    // flags
    input  [3:0]  flags,          // {N, Z, C, V} current
    output [3:0]  nzcv_in,        // flags this instruction would write
    // debug (commit trace)
    output [3:0]  dbg_wa1,
    output [31:0] dbg_res1,
    output [3:0]  dbg_wa2,
    output [31:0] dbg_res2,
    output [31:0] dbg_pcnext,
    input         view,             // board view (halted / faulted CPU): read register dbg_ra on port 1, RAM word at dbg_addr on the data port
    input         view_mem,
    input  [3:0]  dbg_ra,
    input  [31:0] dbg_addr,
    output [31:0] dbg_rd1,
    output        tgt0              // bit 0 of the PC target (BX to an odd address = Thumb request, not supported)
);
    // ---------------- PC ----------------
    reg  [31:0] pc_r;
    wire [31:0] pc_plus4 = pc_r + 32'd4;
    wire [31:0] pc_plus8 = pc_r + 32'd8;
    assign pc = pc_r;

    // ---------------- register file ----------------
    wire [3:0] ra1 = view ? dbg_ra : (RA1sel == 2'd0) ? instr[19:16] : (RA1sel == 2'd1) ? 4'd15 : instr[15:12];
    wire [3:0] ra2 = instr[3:0];
    // ---------------- LDM / STM sequencer ----------------
    wire [3:0]  seq_reg;
    wire [31:0] seq_addr, seq_next;
    wire [31:0] rd1, rd2, rd3, res1, res2;
    ldmstm_seq_arm #(.EN(LDM_EN)) seq (
        .clk(clk), .rst(rst), .active(ldm_act), .rlist(instr[15:0]), .P(instr[24]), .U(instr[23]), .rn_val(rd1),
        .reg_idx(seq_reg), .addr(seq_addr), .next(seq_next), .last(ldm_last));

    wire [3:0] ra3 = ldm_act ? seq_reg : RA3sel ? instr[15:12] : instr[11:8];               // STM: store data = the beat's register
    wire [3:0] wa1 = ldm_act ? seq_reg : (WA1sel == 2'd0) ? instr[15:12] : (WA1sel == 2'd1) ? 4'd14 : instr[19:16];   // LDM: load target
    wire [3:0] wa2 = instr[19:16];

    regfile_arm #(.RD3_EN(RD3_EN)) rf (
        .clk(clk), .rst(rst), .ra1(ra1), .ra2(ra2), .ra3(ra3), .rd1(rd1), .rd2(rd2), .rd3(rd3), .r15(pc_plus8),
        .we1(RegWrite1), .wa1(wa1), .wd1(res1), .we2(RegWrite2), .wa2(wa2), .wd2(res2));

    // ---------------- extend, source B, ALU ----------------
    wire [31:0] ext;
    ext_arm #(.ROT_EN(ROT_EN)) ex (.instr24(instr[23:0]), .ImmSrc(ImmSrc), .ext(ext));

    // barrel shifter on RD2: type Instr[6:5], amount Instr[11:7] or RD3[7:0] (Rs), carry out for the logical instructions
    wire [31:0] shifted;
    wire        sh_c;
    shifter_arm #(.SH_EN(SH_EN), .REG_EN(SHREG_EN)) shf (
        .din(rd2), .typ(instr[6:5]), .amt_imm(instr[11:7]), .amt_reg(rd3[7:0]), .by_reg(ShAmtSel), .dis(ShDis),
        .cin(flags[1]), .dout(shifted), .cout(sh_c));
    // carry of a rotated immediate: bit 31 of the rotated value when the rotation is not 0, else the old C
    wire        imm_c     = (ROT_EN != 0 && instr[11:8] != 4'd0) ? ext[31] : flags[1];
    wire        shifter_c = BSrc ? imm_c : sh_c;
    wire [31:0] srcb = BSrc ? ext : shifted;

    wire [31:0] alu_res;
    wire alu_n, alu_z, alu_c, alu_v;
    alu_arm alu (.A(rd1), .B(srcb), .ALUControl(ALUControl), .Cin(flags[1]), .Result(alu_res),
                 .N(alu_n), .Z(alu_z), .Cadd(alu_c), .Vadd(alu_v));

    // multiply: N, Z of the result (64-bit result for the long forms); C and V are never written (FlagW = NZ only)
    wire [31:0] mul_lo, mul_hi;
    wire mul_n = MulLong ? mul_hi[31] : mul_lo[31];
    wire mul_z = (mul_lo == 32'd0) & (~MulLong | (mul_hi == 32'd0));
    assign nzcv_in = {Mul ? mul_n : alu_n, Mul ? mul_z : alu_z, CSrc ? shifter_c : alu_c, alu_v};

    // ---------------- data memory interface ----------------
    assign daddr = view_mem ? dbg_addr : ldm_act ? seq_addr : MemAddrSel ? alu_res : rd1;
    assign dbg_rd1 = rd1;
    assign wdata = rd3;

    // ---------------- result muxes ----------------
    // ---------------- multiplier (step 12) ----------------
    // UMLAL / SMLAL take two beats (spec 8.2): beat 0 reads RdLo on RD1 and keeps lo = p[31:0] + RdLo and its carry in
    // lo_r / c_r, writing nothing; beat 1 reads RdHi on RD1, writes RdLo = lo_r (port 1) and RdHi = p[63:32] + RdHi + c_r (port 2).
    wire [31:0] m_lo, m_hi;
    wire        m_c, c_sav;
    mul_arm #(.MUL_EN(MUL_EN), .LONG_EN(MULL_EN)) mul (
        .a(rd2), .b(rd3), .sgn(MulSigned), .acc(MulAcc), .long_op(MulLong), .acc_lo(rd1), .acc_hi(rd1),
        .cin(mul_beat1 & c_sav), .lo(m_lo), .hi(m_hi), .cout(m_c));
    generate
        if (MULL_EN) begin : g_beat
            reg [31:0] lo_r;
            reg        c_r;
            always @(posedge clk) if (~mul_beat1) begin lo_r <= m_lo; c_r <= m_c; end
            assign mul_lo = mul_beat1 ? lo_r : m_lo;
            assign c_sav  = c_r;
        end else begin : g_nobeat
            assign mul_lo = m_lo;
            assign c_sav  = 1'b0;
        end
    endgenerate
    assign mul_hi = m_hi;
    assign res1 = ldm_act ? rdata : (ResSrc == 2'd0) ? alu_res : (ResSrc == 2'd1) ? rdata : (ResSrc == 2'd2) ? pc_plus4 : mul_lo;
    assign res2 = ldm_act ? seq_next : Res2Src ? mul_hi : alu_res;      // LDM/STM write-back: Rn <- next (last beat)

    // ---------------- PC logic ----------------
    wire [31:0] pc_target = (PCTSel | ldm_act) ? rdata : alu_res;       // LDM of PC: the loaded word
    wire [31:0] pc_next   = PCSrc ? pc_target : pc_plus4;
    assign tgt0 = pc_target[0];
    always @(posedge clk) begin
        if (rst) pc_r <= 32'd0;
        else if (pc_en) pc_r <= {pc_next[31:2], 2'b00};
    end
    initial pc_r = 32'd0;

    assign dbg_wa1 = wa1; assign dbg_res1 = res1; assign dbg_wa2 = wa2; assign dbg_res2 = res2; assign dbg_pcnext = {pc_next[31:2], 2'b00};
endmodule