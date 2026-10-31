//====================================================================
// cpu_core_arm.v   (ARMv4 single-cycle CPU, step 6)  -  processor core with its memories
//
//   decoder_arm + condlogic_arm + datapath_arm + imem_arm (ROM, 2 async ports) + dmem_arm (RAM, aligner).
//   One instruction per clock cycle, except UMLAL/SMLAL (2 cycles, step 12) and LDM/STM (N cycles for N registers, step 13). Asynchronous memories:
//   PC -> ROM -> decode -> register read -> ALU -> data address -> RAM/ROM -> aligner -> result -> register
//   write is one clock cycle.
//
//   BUILT : classes whose hardware exists in this step (a bit per FEATURES bit, see decoder_arm.v). FEATURES is
//           ANDed with BUILT, so asking for a class that is not built yet decodes it as illegal.
//           Step 6: only the base core (no bit).   Step 9: bits 0,1,2 (all 16 data-processing opcodes, barrel shifter and
//           rotated immediates, shift by register).   Step 11: bits 5,6,7 (BL, BX, SWI = halt).   Step 10: bits 3,4 (LDRB/STRB, register and shifted-register offsets,
//           post-index, write-back; LDRH STRH LDRSB LDRSH). Step 12: bits 8,9 (MUL MLA; UMULL SMULL UMLAL SMLAL, the last two in 2 cycles). Step 13: bit 10 (LDM/STM, N cycles for N registers, ldmstm_seq_arm.v).
//           Bit 2 needs bit 1 (the register amount goes into the shifter).
//   Run control: 'halted' is set by SWI or "B ." (condition passing); 'fault' is set by an illegal encoding or by a BX
//   (condition passing) whose target is odd (Thumb state is not supported).
//   Either one stops the PC and every write (the CPU keeps showing the last state). rst clears both.
//====================================================================
module cpu_core_arm #(
    parameter [10:0] FEATURES = 11'h7FF,
    parameter        RD3_EN   = 1,
    parameter        IMEM_AW  = 7,
    parameter        DMEM_AW  = 5,
    parameter        INIT_FILE = "program.hex",
    parameter [10:0] BUILT    = 11'h7FF,     // classes with hardware (see above). Test hook: a testbench may set bits whose hardware
                                             // needs nothing outside the decoder (e.g. bit 7, SWI). Leave at the default otherwise.
    parameter        DBG_EN   = 0,           // 1 = board build: after a halt / fault, dbg_idx selects a register / the PC / a RAM word (dbg_val), step 15. 0 = no logic
    parameter        ROM_EXT  = 0,           // 1 = area / timing measurement build: no ROM inside; the instruction and the ROM data port
    parameter        MEM_M9K  = 0            // Step G3 ablation: 1 = ROM/RAM in M9K (ROM on the rising edge addressed by the PC's next value, RAM on the falling edge; CPI stays 1). 0 = textbook asynchronous memories (default, all paper / board builds)
) (                                          //     come from the inputs ext_instr / ext_romB (see the note below)
    input         clk,
    input         rst,              // synchronous, active high
    output        halted,
    output        fault,
    // observation (commit trace, board display)
    output [31:0] pc,
    output [31:0] instr,
    output        retire,           // 1 = an instruction completes in this cycle (including a condition-failed one)
    output [3:0]  flags,            // {N, Z, C, V}
    output [3:0]  flags_next,
    output        dbg_we1, output [3:0] dbg_wa1, output [31:0] dbg_wd1,
    output        dbg_we2, output [3:0] dbg_wa2, output [31:0] dbg_wd2,
    output        dbg_mw,   output [31:0] dbg_maddr, output [31:0] dbg_mdata, output [1:0] dbg_msize,
    output        dbg_pcs,  output [31:0] dbg_pcnext,
    input  [31:0] ext_instr,        // ROM_EXT = 1 only (leave unconnected otherwise)
    input  [31:0] ext_romB,
    input         freeze,           // DBG_EN = 1: the CPU is frozen (snapshot unit): nothing is written, the view is active. Leave unconnected when DBG_EN = 0
    input  [5:0]  dbg_idx,          // DBG_EN = 1: 0..14 = r0..r14, 15 = PC, 16.. = RAM word (idx - 16)
    output [31:0] dbg_val           // the selected value while halted / faulted (reads the register file on port 1 and the RAM on the data port: no extra mux tree); 0 while running
);
    localparam [10:0] FEAT_RAW = FEATURES & BUILT;
    localparam [10:0] FEAT     = {FEAT_RAW[10:3], FEAT_RAW[2] & FEAT_RAW[1], FEAT_RAW[1:0]};   // SHREG needs SHIFT
    wire [31:0] daddr, wdata, rdata, rom_word;
    wire        illegal, MemR, MemW, RegW1, RegW2, PCS, Halt, Seq, BX;
    wire        tgt0;
    wire [1:0]  RA1sel, WA1sel, ImmSrc, MemSize, ResSrc;
    wire        RA3sel, BSrc, ShAmtSel, ShDis, MemAddrSel, MemSigned, Res2Src, CSrc, PCTSel, Mul, MulLong, MulSigned, MulAcc;
    wire [2:0]  FlagW;
    wire [3:0]  ALUControl, nzcv_in;
    wire        CondEx, PCSrc, RegWrite1, RegWrite2, MemWrite, HaltEx;

    reg halted_r = 1'b0, fault_r = 1'b0;               // (declared before first use: some tools do not accept forward references)
    assign halted = halted_r;
    assign fault  = fault_r;
    wire frz = DBG_EN & freeze;                        // (0 & z = 0: an unconnected freeze is harmless when DBG_EN = 0)
    wire run = ~halted_r & ~fault_r & ~frz;            // CPU running

    // BX (condition passing) to an odd address asks for Thumb state: the core stops like on an illegal instruction
    wire thumb_bx = BX & CondEx & tgt0;
    wire bad      = illegal | thumb_bx;

    // UMLAL / SMLAL: two beats (spec 8.2). 'hold' = beat 0 of an executed one: PC, register and flag writes wait one cycle.
    // The condition is checked in beat 0; the flags cannot change before beat 1, so CondEx is the same in both beats.
    reg  beat1 = 1'b0;
    wire mhold = Seq & Mul & CondEx & ~beat1 & run;
    always @(posedge clk) beat1 <= ~rst & (frz ? beat1 : mhold);        // a freeze (board snapshot) in beat 1 keeps beat 1 for the cycle after it

    // LDM / STM (step 13, spec 8.1): N beats, one register per beat; the PC and 'retire' wait for the last beat.
    // Every beat writes (LDM: the loaded register on port 1; STM: memory); write-back of Rn (port 2) and an LDM of PC are on the last.
    wire ldm_cls  = Seq & ~Mul;
    wire ldm_act  = ldm_cls & CondEx & run;
    wire ldm_last;
    wire L_bit    = instr[20];
    wire hold     = mhold | (ldm_act & ~ldm_last);
    // controls of an LDM/STM beat, in place of the decoder's (the decoder only marks the class)
    wire lRegW1 = ldm_cls & L_bit;                                   // a load into R15 is ignored by the register file
    wire lRegW2 = ldm_cls & instr[21] & ldm_last;
    wire lMemW  = ldm_cls & ~L_bit;
    wire lPCS   = ldm_cls & L_bit & instr[15] & ldm_last;
    wire [1:0] RA1eff = beat1 ? 2'd0 : RA1sel;          // beat 1 reads RdHi = Instr[19:16] on RD1


    // ---------------- board view (DBG_EN = 1) ----------------
    // A halted or faulted CPU does nothing (all writes are gated by 'run'), so its register-file port 1 and its data port are free:
    // the view borrows them. idx 0..14 -> r0..r14 on port 1, 15 -> PC, 16.. -> RAM word (idx-16) on the data address (word access).
    wire [31:0] dbg_rd1, dbg_addr;
    wire [5:0]  dbg_ridx = dbg_idx - 6'd16;
    wire view     = DBG_EN & ~run;
    wire view_mem = view & (dbg_idx >= 6'd16);
    assign dbg_addr = {4'h1, 20'd0, dbg_ridx, 2'b00};
    generate
        if (DBG_EN) begin : g_dbg
            assign dbg_val = ~view ? 32'd0 : (dbg_idx < 6'd15) ? dbg_rd1 : (dbg_idx == 6'd15) ? pc :
                             ({1'b0, dbg_ridx} < (7'd1 << DMEM_AW)) ? rdata : 32'd0;
        end else begin : g_nodbg
            assign dbg_val = 32'd0;
        end
    endgenerate

    wire [31:0] pcnext;                                // datapath's next PC (also the M9K ROM address when MEM_M9K = 1)
    // ---------------- memories ----------------
    // ROM_EXT = 1 (measurement build only, not a working CPU): the ROM is built from logic cells, so with a fixed program in it synthesis
    // propagates the ROM contents as constants and removes every piece of logic that program never uses (e.g. the whole LDM/STM sequencer
    // when the program has no LDM). Area and Fmax of the core must not depend on the program, so ARM_Core_Test takes the instruction and
    // the ROM data word from (virtual) input pins. They pass through a register so that the paths that start at the instruction are
    // register-to-register paths that the timing analyzer checks (a path that starts at a virtual input pin is not analysed).
    // What this build leaves out, and the paper must add: the ROM itself (area) and its read delay (PC -> ROM -> instruction, and
    // data address -> ROM port B). The 64 flip-flops of instr_q / romB_q are measurement scaffolding.
    generate
        if (ROM_EXT) begin : g_romext
            reg [31:0] instr_q, romB_q;
            always @(posedge clk) begin instr_q <= ext_instr; romB_q <= ext_romB; end
            assign instr = instr_q;
            assign rom_word = romB_q;
        end else if (MEM_M9K) begin : g_m9k_rom
            // PC register's next value (datapath_arm: rst -> 0, pc_en -> {pc_next[31:2], 00}): address of the M9K ROM port A
            wire [31:0] pc_d = rst ? 32'd0 : ((run & ~bad & ~hold) ? pcnext : pc);
            imem_m9k_arm_sc #(.IMEM_AW(IMEM_AW), .INIT_FILE(INIT_FILE)) rom (.clk(clk), .addrA_next(pc_d), .instrA(instr), .addrB(daddr), .dataB(rom_word));
        end else begin : g_rom
            imem_arm #(.IMEM_AW(IMEM_AW), .INIT_FILE(INIT_FILE)) rom (.addrA(pc), .instrA(instr), .addrB(daddr), .dataB(rom_word));
        end
    endgenerate
    generate if (MEM_M9K) begin : g_m9k_ram
        dmem_m9k_arm_sc #(.DMEM_AW(DMEM_AW)) ram (.clk(clk), .addr(daddr), .wdata(wdata), .we(MemWrite), .size(view_mem ? 2'd0 : MemSize), .sgn(view_mem ? 1'b0 : MemSigned),
                                                  .rom_word(rom_word), .rdata(rdata));
    end else begin : g_ram
    dmem_arm #(.DMEM_AW(DMEM_AW)) ram (.clk(clk), .addr(daddr), .wdata(wdata), .we(MemWrite), .size(view_mem ? 2'd0 : MemSize), .sgn(view_mem ? 1'b0 : MemSigned),
                                       .rom_word(rom_word), .rdata(rdata));
    end endgenerate

    // ---------------- control ----------------
    decoder_arm #(.FEATURES(FEAT)) dec (
        .instr(instr), .illegal(illegal), .RA1sel(RA1sel), .RA3sel(RA3sel), .WA1sel(WA1sel), .BSrc(BSrc), .ImmSrc(ImmSrc),
        .ShAmtSel(ShAmtSel), .ShDis(ShDis), .ALUControl(ALUControl), .MemAddrSel(MemAddrSel), .MemR(MemR), .MemW(MemW),
        .MemSize(MemSize), .MemSigned(MemSigned), .ResSrc(ResSrc), .Res2Src(Res2Src), .RegW1(RegW1), .RegW2(RegW2),
        .FlagW(FlagW), .CSrc(CSrc), .PCS(PCS), .PCTSel(PCTSel), .Mul(Mul), .MulLong(MulLong), .MulSigned(MulSigned),
        .MulAcc(MulAcc), .Halt(Halt), .Seq(Seq), .BX(BX));

    condlogic_arm cl (
        .clk(clk), .rst(rst), .cond(instr[31:28]), .nzcv_in(nzcv_in), .FlagW(FlagW & {3{run & ~mhold}}), .PCS((PCS | lPCS) & run),
        .RegW1((RegW1 & ~mhold | lRegW1) & run), .RegW2((RegW2 & ~mhold | lRegW2) & run), .MemW((MemW | lMemW) & run), .Halt(Halt & run),
        .CondEx(CondEx), .flags(flags), .flags_next(flags_next), .PCSrc(PCSrc), .RegWrite1(RegWrite1),
        .RegWrite2(RegWrite2), .MemWrite(MemWrite), .HaltEx(HaltEx));

    // ---------------- datapath ----------------
    wire [3:0]  wa1, wa2;
    wire [31:0] res1, res2;
    datapath_arm #(.RD3_EN(RD3_EN), .ROT_EN(FEAT[1]), .SH_EN(FEAT[1]), .SHREG_EN(FEAT[2]), .MUL_EN(FEAT[8] | FEAT[9]), .MULL_EN(FEAT[9]), .LDM_EN(FEAT[10])) dp (
        .clk(clk), .rst(rst), .instr(instr), .pc(pc), .daddr(daddr), .wdata(wdata), .rdata(rdata),
        .RA1sel(RA1eff), .RA3sel(RA3sel), .WA1sel(WA1sel), .BSrc(BSrc), .ImmSrc(ImmSrc), .ALUControl(ALUControl),
        .MemAddrSel(MemAddrSel), .ResSrc(ResSrc), .Res2Src(Res2Src), .RegWrite1(RegWrite1), .RegWrite2(RegWrite2),
        .CSrc(CSrc), .ShAmtSel(ShAmtSel), .ShDis(ShDis), .PCSrc(PCSrc), .PCTSel(PCTSel), .pc_en(run & ~bad & ~hold), .Mul(Mul), .MulLong(MulLong), .MulSigned(MulSigned), .MulAcc(MulAcc), .mul_beat1(beat1), .ldm_act(ldm_act), .ldm_last(ldm_last), .flags(flags), .nzcv_in(nzcv_in),
        .dbg_wa1(wa1), .dbg_res1(res1), .dbg_wa2(wa2), .dbg_res2(res2), .dbg_pcnext(pcnext), .tgt0(tgt0),
        .view(view), .view_mem(view_mem), .dbg_ra(dbg_idx[3:0]), .dbg_addr(dbg_addr), .dbg_rd1(dbg_rd1));

    // ---------------- halt / fault ----------------
    always @(posedge clk) begin
        if (rst) begin halted_r <= 1'b0; fault_r <= 1'b0; end
        else begin
            if (HaltEx) halted_r <= 1'b1;
            if (bad & run) fault_r <= 1'b1;
        end
    end

    // ---------------- observation ----------------
    assign retire    = run & ~bad & ~hold;
    assign dbg_we1 = RegWrite1 & (wa1 != 4'd15); assign dbg_wa1 = wa1; assign dbg_wd1 = res1;
    assign dbg_we2 = RegWrite2 & (wa2 != 4'd15); assign dbg_wa2 = wa2; assign dbg_wd2 = res2;
    assign dbg_mw = MemWrite; assign dbg_maddr = daddr; assign dbg_mdata = wdata; assign dbg_msize = MemSize;
    assign dbg_pcs = PCSrc;   assign dbg_pcnext = pcnext;
endmodule