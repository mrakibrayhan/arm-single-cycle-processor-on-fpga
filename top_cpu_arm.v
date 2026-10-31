//====================================================================
// top_cpu_arm.v  (board top-level - RZ-EasyFPGA A2.2 / Cyclone IV EP4CE6)
//
//   ARMv4 SINGLE-CYCLE CPU with ASYNCHRONOUS-read memories built from logic elements (no M9K block is used:
//   the instruction ROM, the data RAM and the register file are all in logic cells, and so is the debug view).
//
//   Buttons (same as the RISC-V boards):
//     KEY[3] = reset (the CPU is also held in reset until the first press of KEY[2]),
//     KEY[2] = run (first press starts the program; KEY[3] stops it and clears 'started').
//     The program runs until it halts (SWI, or "B ." with the condition true) or faults (illegal encoding, BX to an odd address).
//   LEDG (active low = lit): [0] fault   [1] started   [2] Z flag   [3] halted.
//
//   SNAPSHOT (like the RISC-V boards): when the program halts / faults, or when the ISSP source bit is set, the CPU is frozen and r0..r15 and
//   the 32 RAM words and some status words are copied into four RAMs that the In-System Memory Content Editor reads (snapshot_arm.v):
//        R00 = r0..r15 (r15 = PC)     D00 = RAM words 0..15     D16 = RAM words 16..31      (RAM word n = byte address 0x10000000 + 4 * n)
//        R16 = status: word 0 CPSR (flags, user mode)  1 PC  2 instruction  3 cycle counter  4 instruction counter  5 {fault, halted, started}
//   ONE In-System Sources and Probes instance (altsource_probe primitive, no IP file needed):
//     source (1 bit)  = manual snapshot request (1 = freeze the CPU and take a snapshot, 0 = let the CPU run again)
//     probe (120 bits, LSB first):
//         [0] started  [1] halted  [2] fault  [3] snapshot done (the memories hold the copy)
//         [7:4]     flags  {N, Z, C, V}
//         [23:8]    PC (low 16 bits)  = address of the instruction that is (or was last) executed
//         [55:24]   instruction word
//         [87:56]   CYCLE COUNTER       clock cycles in which the CPU was running (the halting instruction's cycle included)
//         [119:88]  INSTRUCTION COUNTER retired instructions (the halting instruction included)
//       CPI = cycle counter / instruction counter. For a program checked with arm_iss.py:
//           instruction counter = instructions of the model,   cycle counter = 'sc_cycles' of the model.
//       Both counters stop when the CPU halts, faults or is frozen.
//   The RAM is not cleared by the reset (a program that needs zeroed RAM must be run after configuring the FPGA, or clear it itself).
//====================================================================
module top_cpu_arm #(
    parameter [10:0] FEATURES  = 11'h7FF,
    parameter        RD3_EN    = 1,
    parameter        IMEM_AW   = 7,              // 128 instruction words
    parameter        DMEM_AW   = 5,              // 32 data words
    parameter        INIT_FILE = "program.hex",
    parameter        DEBOUNCE  = 250000          // button filter length in clock cycles (the testbench makes it short)
) (
    input        clk,      // PIN_23
    input  [4:0] KEY,      // PIN_88,89,90,91,25 (only KEY[3], KEY[2] used)
    output [7:0] SEG,      // PIN_128,121,125,129,132,126,124,127
    output [3:0] DIG,      // PIN_133,135,136,137
    output [3:0] LEDG      // PIN_87,86,85,84
);

    //----------------------------------------------------------------
    // Clock divider: 50 MHz / 4 = 12.5 MHz for the CPU core
    //----------------------------------------------------------------
    reg [1:0] clkdiv = 2'b00;
    always @(posedge clk) clkdiv <= clkdiv + 2'b01;
    wire cpu_clk = clkdiv[1];

    //----------------------------------------------------------------
    // Buttons: KEY[3] reset (level), KEY[2] run (one pulse)
    //----------------------------------------------------------------
    wire key3_raw = ~KEY[3];
    wire rst_debounced;
    debounce #(.DEBOUNCE_LIMIT(DEBOUNCE)) db_rst (.clk(clk), .btn_in(key3_raw), .btn_out(rst_debounced));

    wire key2_raw = ~KEY[2];
    wire key2_debounced, step_pulse;
    debounce #(.DEBOUNCE_LIMIT(DEBOUNCE)) db_step (.clk(cpu_clk), .btn_in(key2_raw), .btn_out(key2_debounced));
    edge_detector ed_step (.clk(cpu_clk), .signal_in(key2_debounced), .pulse_out(step_pulse));

    reg started = 1'b0;
    always @(posedge cpu_clk) begin
        if (rst_debounced)   started <= 1'b0;
        else if (step_pulse) started <= 1'b1;
    end

    //----------------------------------------------------------------
    // CPU (held in reset until 'started')
    //----------------------------------------------------------------
    wire        snap_req;                                  // ISSP source: manual snapshot request (JTAG clock domain)
    wire        frozen, snap_done;
    wire [5:0]  snap_idx;
    wire        halted, fault, retire;
    wire [31:0] pc, instr, view_val;
    wire [3:0]  flags;
    wire        cpu_rst = rst_debounced | ~started;

    cpu_core_arm #(.FEATURES(FEATURES), .RD3_EN(RD3_EN), .IMEM_AW(IMEM_AW), .DMEM_AW(DMEM_AW), .INIT_FILE(INIT_FILE),
                   .BUILT(11'h7FF), .DBG_EN(1), .ROM_EXT(0)) mycpu (
        .clk(cpu_clk), .rst(cpu_rst), .halted(halted), .fault(fault), .pc(pc), .instr(instr), .retire(retire), .flags(flags),
        .flags_next(), .dbg_we1(), .dbg_wa1(), .dbg_wd1(), .dbg_we2(), .dbg_wa2(), .dbg_wd2(), .dbg_mw(), .dbg_maddr(),
        .dbg_mdata(), .dbg_msize(), .dbg_pcs(), .dbg_pcnext(),
        .ext_instr(32'd0), .ext_romB(32'd0), .freeze(frozen), .dbg_idx(snap_idx), .dbg_val(view_val));

    //----------------------------------------------------------------
    // Counters (cpu_clk). They count while the CPU runs and stop at the halt / fault; the reset clears them.
    //----------------------------------------------------------------
    reg [31:0] cycle_count = 32'd0, instr_count = 32'd0;
    wire running = started & ~halted & ~fault & ~frozen;
    always @(posedge cpu_clk) begin
        if (cpu_rst) begin
            cycle_count <= 32'd0; instr_count <= 32'd0;
        end else if (running) begin
            cycle_count <= cycle_count + 32'd1;
            if (retire) instr_count <= instr_count + 32'd1;
        end
    end

    //----------------------------------------------------------------
    // Snapshot unit (4 In-System Memory Content Editor RAMs) - freezes the CPU, copies r0..r15, the RAM and the status words
    //----------------------------------------------------------------
    // R16 words (snapshot idx 48..63): the status of the CPU (the ARM has only 16 registers)
    reg [31:0] extra_val;
    always @(*) begin
        case (snap_idx[3:0])
            4'd0:    extra_val = {flags, 20'd0, 8'h10};        // CPSR: N Z C V, I = F = T = 0, user mode (10000)
            4'd1:    extra_val = pc;
            4'd2:    extra_val = instr;
            4'd3:    extra_val = cycle_count;
            4'd4:    extra_val = instr_count;
            4'd5:    extra_val = {29'd0, fault, halted, started};
            default: extra_val = 32'd0;
        endcase
    end
    snapshot_arm snap (.clk(cpu_clk), .rst(cpu_rst), .started(started), .trigger_auto(halted | fault), .trigger_src(snap_req),
                       .view_val(view_val), .extra_val(extra_val), .freeze(frozen), .done(snap_done), .idx(snap_idx));

    //----------------------------------------------------------------
    // 7-seg display unused
    //----------------------------------------------------------------
    assign SEG = 8'hFF;
    assign DIG = 4'hF;

    // LEDs (active low)
    assign LEDG[0] = ~fault;
    assign LEDG[1] = ~started;
    assign LEDG[2] = ~flags[2];
    assign LEDG[3] = ~halted;

    //----------------------------------------------------------------
    // ISSP: source = manual snapshot request, probe = status + counters
    //----------------------------------------------------------------
    wire [119:0] probe_status = {instr_count, cycle_count, instr, pc[15:0], flags, snap_done, fault, halted, started};
    // altsource_probe is the megafunction behind the "In-System Sources and Probes" IP (it is in Quartus' altera_mf library; no IP file needed)
    altsource_probe #(
        .lpm_type                ("altsource_probe"),
        .lpm_hint                ("UNUSED"),
        .sld_auto_instance_index ("YES"),
        .sld_instance_index      (0),
        .SLD_NODE_INFO           (4746752),
        .sld_ir_width            (4),
        .instance_id             ("ARM0"),
        .probe_width             (120),
        .source_width            (1),
        .source_initial_value    ("0"),
        .enable_metastability    ("NO")
    ) inst_issp (
        .probe      (probe_status),
        .source     (snap_req),
        .source_clk (cpu_clk),
        .source_ena (1'b1)
    );

endmodule