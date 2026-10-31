//====================================================================
// snapshot_arm.v   (ARMv4 single-cycle CPU, board view, step 15b)  -  register / RAM snapshot for the In-System Memory Content Editor
//
//   Same idea as the RISC-V snapshot_unit.v. The CPU is frozen and a copy of r0..r15, of the 32 RAM words and of some status words is written into FOUR
//   small RAMs (16 words x 32 bit, M9K) that are read over JTAG with Quartus' In-System Memory Content Editor:
//        R00 = r0 .. r15   (r15 shows the PC, i.e. the address of the instruction that is or was last executed)
//        D00 = RAM word  0 .. 15      (byte addresses 0x10000000 .. 0x1000003C)
//        D16 = RAM word 16 .. 31      (byte addresses 0x10000040 .. 0x1000007C)
//        R16 = status words (the ARM has only 16 registers; this RAM plays the role of the RISC-V x16..x31 view):
//              word 0 CPSR {N,Z,C,V, 24'b0, 8'h10 (user mode)}   1 PC   2 instruction word   3 cycle counter   4 instruction counter
//              word 5 {29'b0, fault, halted, started}   words 6..15 = 0
//   WHEN: the program halts or faults (trigger_auto), or the ISSP source bit is set (trigger_src, manual request: works at any time,
//   also when the program is stuck). The CPU is frozen (freeze = 1: no register, flag, memory or PC write, no sequencer step) in SNAP and DONE;
//   a frozen CPU does not move, so the core's debug view (dbg_val for dbg_idx) can be read in the same cycle: no read latency, no flush.
//   IDLE (CPU runs) -> SNAP (64 cycles, idx 0..63) -> DONE (frozen while the trigger stays true; a halted program keeps it true) -> IDLE.
//   The view index (idx) is the one of cpu_core_arm: 0..14 = r0..r14, 15 = PC, 16.. = RAM word (idx - 16).
//   CLEAR: while rst is high (KEY[3] held, and until KEY[2] starts the CPU) the four RAMs are written with 0 (all four in parallel, one word per
//   cycle, 16 cycles), so after KEY[3] the Memory Content Editor shows zeros and after KEY[2] the new run's snapshot.
//====================================================================
module snapshot_arm (
    input         clk,           // cpu_clk
    input         rst,           // debounced reset (level)
    input         started,       // the CPU has been started with KEY[2]
    input         trigger_auto,  // the program halted or faulted
    input         trigger_src,   // manual request: ISSP source bit (JTAG clock domain)
    input  [31:0] view_val,      // cpu_core_arm dbg_val for idx (valid in the same cycle while frozen)
    input  [31:0] extra_val,     // status word for idx 48..63 (word idx[3:0] of R16), from the top
    output        freeze,
    output        done,          // the snapshot RAMs hold a complete copy
    output [5:0]  idx
);
    // the manual trigger comes from the JTAG clock domain: two flip-flops
    reg s1 = 1'b0, s2 = 1'b0;
    always @(posedge clk) begin
        s1 <= trigger_src;
        s2 <= s1;
    end
    wire trigger = trigger_auto | s2;

    localparam IDLE = 2'd0, SNAP = 2'd1, DONE = 2'd2;
    reg [1:0] state = IDLE;
    reg [5:0] cnt   = 6'd0;
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            cnt   <= 6'd0;
        end else begin
            case (state)
                IDLE: if (started && trigger) begin state <= SNAP; cnt <= 6'd0; end
                SNAP: if (cnt == 6'd63) state <= DONE; else cnt <= cnt + 6'd1;
                DONE: if (!trigger) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end
    assign freeze = (state != IDLE);
    assign done   = (state == DONE);
    assign idx    = cnt;

    wire snap = (state == SNAP);
    wire [3:0] word_in = cnt[3:0];
    wire [31:0] q_R00_unused, q_D00_unused, q_D16_unused, q_R16_unused;
    // clear while in reset: a 4-bit address counter, data 0, all four RAMs written
    reg  [3:0]  clr = 4'd0;
    always @(posedge clk) clr <= rst ? clr + 4'd1 : 4'd0;
    wire [3:0]  ram_addr  = rst ? clr : word_in;
    wire [31:0] snap_data = rst ? 32'd0 : (cnt[5:4] == 2'd3) ? extra_val : view_val;

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=R00"), .lpm_type ("altsyncram")
    ) snap_ram_R00 (.address_a (ram_addr), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (snap && cnt[5:4] == 2'd0)), .q_a (q_R00_unused));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=R16"), .lpm_type ("altsyncram")
    ) snap_ram_R16 (.address_a (ram_addr), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (snap && cnt[5:4] == 2'd3)), .q_a (q_R16_unused));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=D00"), .lpm_type ("altsyncram")
    ) snap_ram_D00 (.address_a (ram_addr), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (snap && cnt[5:4] == 2'd1)), .q_a (q_D00_unused));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=D16"), .lpm_type ("altsyncram")
    ) snap_ram_D16 (.address_a (ram_addr), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (snap && cnt[5:4] == 2'd2)), .q_a (q_D16_unused));
endmodule
