//====================================================================
// mem_m9k_arm_sc.sv - Step G3 ABLATION ONLY (cpu_core_arm MEM_M9K = 1; the default MEM_M9K = 0 never uses this file).
//   ROM (dual port): port A on the rising edge, addressed by the PC register's next value (instrA = ROM[PC] for the whole cycle,
//   CPI unchanged); port B (literal-pool loads, data address) on the falling edge.
//   RAM: falling edge (half a cycle for the data path). Byte lanes, ROM/RAM select and load aligner = dmem_arm.v.
//====================================================================
module imem_m9k_arm_sc #(
    parameter IMEM_AW   = 7,
    parameter INIT_FILE = "program.hex"
) (
    input             clk,
    input      [31:0] addrA_next,    // = the PC register's D input
    output reg [31:0] instrA,
    input      [31:0] addrB,         // data address (read at the falling edge)
    output reg [31:0] dataB
);
    (* ramstyle = "M9K" *) reg [31:0] mem [0:(1 << IMEM_AW) - 1];
    initial $readmemh(INIT_FILE, mem);
    always @(posedge clk) instrA <= mem[addrA_next[IMEM_AW+1:2]];
    always @(negedge clk) dataB  <= mem[addrB[IMEM_AW+1:2]];
endmodule

module dmem_m9k_arm_sc #(
    parameter DMEM_AW = 5
) (
    input         clk,
    input  [31:0] addr,
    input  [31:0] wdata,
    input         we,
    input  [1:0]  size,              // 0 word, 1 half, 2 byte
    input         sgn,
    input  [31:0] rom_word,          // registered at the same falling edge by imem_m9k_arm_sc port B
    output [31:0] rdata
);
    wire sel_ram = (addr[31:28] == 4'h1);
    wire sel_rom = (addr[31:28] == 4'h0);
    (* ramstyle = "M9K" *) logic [3:0][7:0] ram [0:(1 << DMEM_AW) - 1];
    // Starting values from data.hex in the project folder (Step H: CoreMark's constants must be in the RAM, as on the
    // other 8 cores; the textbook dmem_arm.v starts from zeros and is not changed). data.hex must exist when MEM_M9K = 1.
    initial $readmemh("data.hex", ram);
    wire [DMEM_AW-1:0] widx = addr[DMEM_AW+1:2];
    reg [31:0] sdata;
    reg [3:0]  be;
    always @(*) begin
        case (size)
            2'd2:    begin sdata = {4{wdata[7:0]}};  be = 4'b0001 << addr[1:0];        end
            2'd1:    begin sdata = {2{wdata[15:0]}}; be = addr[1] ? 4'b1100 : 4'b0011; end
            default: begin sdata = wdata;            be = 4'b1111;                      end
        endcase
    end
    reg [31:0] ram_word;
    always @(negedge clk) begin
        if (we && sel_ram && be[0]) ram[widx][0] <= sdata[7:0];
        if (we && sel_ram && be[1]) ram[widx][1] <= sdata[15:8];
        if (we && sel_ram && be[2]) ram[widx][2] <= sdata[23:16];
        if (we && sel_ram && be[3]) ram[widx][3] <= sdata[31:24];
        ram_word <= ram[widx];
    end
    wire [31:0] word = sel_ram ? ram_word : (sel_rom ? rom_word : 32'd0);
    reg [7:0] b;
    always @(*) begin
        case (addr[1:0])
            2'd0: b = word[7:0];  2'd1: b = word[15:8];  2'd2: b = word[23:16];  default: b = word[31:24];
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
