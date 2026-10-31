//====================================================================
// ldmstm_seq_arm.v   (ARMv4 single-cycle CPU, step 13)  -  LDM / STM sequencer (spec section 8.1)
//
//   An LDM/STM with N registers takes N cycles, one register per cycle (the register file has one free write port for
//   loads and one read port, RD3, for store data). While it runs the PC is held, so the instruction (and its register
//   list) stays on the ROM output: no instruction register and no copy of the list are needed.
//
//   per beat (combinational):
//     cand  = first beat ? Instr[15:0] : Instr[15:0] bits beyond lastpos (above it for U = 1, below it for U = 0)
//     reg   = U ? lowest set bit of cand : highest     (ascending addresses for IA/IB, descending for DA/DB:
//                                                       the memory image is the ARM one, lowest register at the lowest address)
//     base  = first beat ? Rn (RD1) : ptr
//     next  = U ? base + 4 : base - 4                  addr = P ? next : base
//     last  = no other bit left in cand
//   on the clock edge of an active beat:  ptr <= next, lastpos <= reg, busy <= ~last   (5 + 32 flip-flops)
//   The core uses: reg (STM: RA3 = reg; LDM: WA1 = reg), addr (data address), last (write-back Rn <- next on port 2,
//   LDM of PC: PC <- load data; only on the last beat does the PC move).
//
//   EN = 0 : not built (FEATURES[10] = 0): outputs are constant, the decoder rejects LDM/STM.
//====================================================================
module ldmstm_seq_arm #(
    parameter EN = 1
) (
    input         clk,
    input         rst,
    input         active,        // an executed (condition true) LDM/STM is in the core this cycle
    input  [15:0] rlist,         // Instr[15:0]  (stable during all beats: the PC is held)
    input         P, U,
    input  [31:0] rn_val,        // RD1 = Rn
    output [3:0]  reg_idx,
    output [31:0] addr,
    output [31:0] next,
    output        last
);
    generate
        if (!EN) begin : g_none
            assign reg_idx = 4'd0; assign addr = 32'd0; assign next = 32'd0; assign last = 1'b1;
        end else begin : g_seq
            // registers: busy (a beat has been done and more are to come), lastpos (register of the previous beat), ptr (running address)
            reg        busy = 1'b0;
            reg [31:0] ptr;
            reg [3:0]  lastpos;
            wire [31:0] base = busy ? ptr : rn_val;

            wire [15:0] beyond;
            genvar g;
            for (g = 0; g < 16; g = g + 1) begin : g_bey
                assign beyond[g] = U ? (lastpos < g) : (lastpos > g);
            end
            wire [15:0] cand = busy ? (rlist & beyond) : rlist;

            // the chosen bit: lowest set bit (U = 1) or highest (U = 0), plain AND/OR chains
            reg [15:0] lo_bit, hi_bit;
            reg        seen_lo, seen_hi;
            integer    i;
            always @(*) begin
                seen_lo = 1'b0;
                for (i = 0; i < 16; i = i + 1) begin lo_bit[i] = cand[i] & ~seen_lo; seen_lo = seen_lo | cand[i]; end
                seen_hi = 1'b0;
                for (i = 15; i >= 0; i = i - 1) begin hi_bit[i] = cand[i] & ~seen_hi; seen_hi = seen_hi | cand[i]; end
            end
            wire [15:0] one = U ? lo_bit : hi_bit;
            reg  [3:0]  pos;
            integer     k;
            always @(*) begin
                pos = 4'd0;
                for (k = 0; k < 16; k = k + 1) if (one[k]) pos = k[3:0];
            end
            assign reg_idx = pos;
            assign last = ((cand & ~one) == 16'd0);

            assign next = U ? base + 32'd4 : base - 32'd4;
            assign addr = P ? next : base;

            always @(posedge clk) begin
                if (rst) busy <= 1'b0;
                else if (active) begin
                    busy    <= ~last;
                    ptr     <= next;
                    lastpos <= pos;
                end
            end
        end
    endgenerate
endmodule