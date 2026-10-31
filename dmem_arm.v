//====================================================================
// dmem_arm.v   (ARMv4 single-cycle CPU, step 4)
//
//   Data side of the memory system: the data RAM (asynchronous read, byte-enable
//   write), the region select between RAM and ROM, the store lane replication and
//   the load aligner (byte / halfword select, zero or sign extension).
//   Specification: arm_sc_spec.md sections 3, 5 and D4, D9.
//
//   Address map (byte address 'addr' = DataAddr):
//       addr[31:28] = 4'h1 : data RAM, word index addr[DMEM_AW+1:2]  (aliases beyond the RAM size)
//       addr[31:28] = 4'h0 : instruction ROM, read through imem_arm port B ('rom_word' input),
//                            stores are ignored
//       anything else      : reads 0, stores ignored
//   (The spec's 0x00000000 / 0x10000000 map; the golden model rejects addresses outside the
//    real memories, so programs never depend on the aliasing or the "reads 0" rule.)
//
//   Size / sign (spec section 5 control signals):
//       size = 0 word, 1 half, 2 byte      sgn = 1 for LDRSB / LDRSH (sign extend), else zero extend
//
//   Store (STR / STRH / STRB), 'we' is MemW already gated with CondEx by the datapath:
//       the store data 'wdata' is replicated over the byte lanes (STRB: 4 x byte, STRH: 2 x half)
//       and the byte enables pick the addressed lane(s), so no store shifter is needed.
//   Load: the word is read asynchronously from RAM or ROM and aligned in this module.
//       Word accesses ignore addr[1:0]; halfword accesses use addr[1] only (addr[0] ignored).
//
//   Writes happen at the rising edge of clk. A load in the same cycle as a store to the same
//   word returns the OLD data (single-cycle behaviour; LDR and STR are never in one instruction).
//====================================================================
module dmem_arm #(
    parameter DMEM_AW = 5                         // 2**5 = 32 words (board build, spec D9)
) (
    input         clk,
    input  [31:0] addr,                           // byte address (DataAddr)
    input  [31:0] wdata,                          // store data (RD3)
    input         we,                             // store enable (MemW & CondEx)
    input  [1:0]  size,                           // 0 word, 1 half, 2 byte
    input         sgn,                            // sign-extend the load (LDRSB, LDRSH)
    input  [31:0] rom_word,                       // word read from the ROM by imem_arm port B
    output [31:0] rdata                           // aligned and extended load data
);

    // ---------------- region select ----------------
    wire sel_ram = (addr[31:28] == 4'h1);
    wire sel_rom = (addr[31:28] == 4'h0);

    // ---------------- RAM array ----------------
    (* ramstyle = "logic" *) reg [31:0] ram [0:(1 << DMEM_AW) - 1];
    integer i;
    initial for (i = 0; i < (1 << DMEM_AW); i = i + 1) ram[i] = 32'd0;

    wire [DMEM_AW-1:0] widx = addr[DMEM_AW+1:2];

    // ---------------- store: lane replication and byte enables ----------------
    reg [31:0] sdata;
    reg [3:0]  be;
    always @(*) begin
        case (size)
            2'd2:    begin sdata = {4{wdata[7:0]}};  be = 4'b0001 << addr[1:0];            end   // byte
            2'd1:    begin sdata = {2{wdata[15:0]}}; be = addr[1] ? 4'b1100 : 4'b0011;     end   // half
            default: begin sdata = wdata;            be = 4'b1111;                          end   // word
        endcase
    end

    always @(posedge clk) begin
        if (we && sel_ram) begin
            if (be[0]) ram[widx][7:0]   <= sdata[7:0];
            if (be[1]) ram[widx][15:8]  <= sdata[15:8];
            if (be[2]) ram[widx][23:16] <= sdata[23:16];
            if (be[3]) ram[widx][31:24] <= sdata[31:24];
        end
    end

    // ---------------- load: region mux and aligner ----------------
    wire [31:0] word = sel_ram ? ram[widx] : (sel_rom ? rom_word : 32'd0);

    reg [7:0]  b;
    always @(*) begin
        case (addr[1:0])
            2'd0: b = word[7:0];
            2'd1: b = word[15:8];
            2'd2: b = word[23:16];
            default: b = word[31:24];
        endcase
    end
    wire [15:0] h = addr[1] ? word[31:16] : word[15:0];

    reg [31:0] r;
    always @(*) begin
        case (size)
            2'd2:    r = {{24{sgn & b[7]}},  b};
            2'd1:    r = {{16{sgn & h[15]}}, h};
            default: r = word;
        endcase
    end
    assign rdata = r;

endmodule
