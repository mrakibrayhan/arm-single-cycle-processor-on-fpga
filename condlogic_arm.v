//====================================================================
// condlogic_arm.v   (ARMv4 single-cycle CPU, step 5)
//
//   Condition check, NZCV flag register and write-enable gating (Harris & Harris "Conditional Logic",
//   extended with the second write port, the halt and the carry/overflow write enables). Spec section 6.1.
//
//   CondEx = condition 'cond' evaluated on the CURRENT flags (the flags before this instruction).
//   Flags are written at the rising clock edge when CondEx and the FlagW bit are both 1:
//       FlagW[2] : N and Z       FlagW[1] : C       FlagW[0] : V
//   (nzcv_in is computed by the datapath: N, Z from the result (long multiplies: both halves),
//    C from the adder or the shifter (CSrc), V from the adder; a bit whose FlagW is 0 is not used.)
//   A failed condition changes nothing: all enables below are 0.
//
//   cond = 1111 gives CondEx = 0 (it is an illegal encoding; the decoder reports it with 'illegal').
//====================================================================
module condlogic_arm (
    input        clk,
    input        rst,                 // synchronous, clears the flags
    input  [3:0] cond,                // Instr[31:28]
    input  [3:0] nzcv_in,             // {N, Z, C, V} produced by this instruction
    input  [2:0] FlagW,               // {N and Z, C, V} write enables from the decoder
    input        PCS,
    input        RegW1,
    input        RegW2,
    input        MemW,
    input        Halt,
    output       CondEx,
    output [3:0] flags,               // {N, Z, C, V} current (ALU carry-in, shifter carry-in)
    output [3:0] flags_next,          // value the flags will have after this clock edge (debug / commit trace only)
    output       PCSrc,
    output       RegWrite1,
    output       RegWrite2,
    output       MemWrite,
    output       HaltEx
);
    reg [3:0] nzcv;
    wire N = nzcv[3], Z = nzcv[2], C = nzcv[1], V = nzcv[0];
    assign flags = nzcv;

    reg ex;
    always @(*) begin
        case (cond)
            4'b0000: ex = Z;                    // EQ
            4'b0001: ex = ~Z;                   // NE
            4'b0010: ex = C;                    // CS / HS
            4'b0011: ex = ~C;                   // CC / LO
            4'b0100: ex = N;                    // MI
            4'b0101: ex = ~N;                   // PL
            4'b0110: ex = V;                    // VS
            4'b0111: ex = ~V;                   // VC
            4'b1000: ex = C & ~Z;               // HI
            4'b1001: ex = ~C | Z;               // LS
            4'b1010: ex = (N == V);             // GE
            4'b1011: ex = (N != V);             // LT
            4'b1100: ex = ~Z & (N == V);        // GT
            4'b1101: ex = Z | (N != V);         // LE
            4'b1110: ex = 1'b1;                 // AL
            default: ex = 1'b0;                 // 1111: illegal
        endcase
    end
    assign CondEx = ex;

    assign flags_next = rst ? 4'b0000 :
                        ex  ? {FlagW[2] ? nzcv_in[3:2] : nzcv[3:2], FlagW[1] ? nzcv_in[1] : nzcv[1], FlagW[0] ? nzcv_in[0] : nzcv[0]}
                            : nzcv;

    always @(posedge clk) begin
        if (rst) nzcv <= 4'b0000;
        else if (ex) begin
            if (FlagW[2]) nzcv[3:2] <= nzcv_in[3:2];
            if (FlagW[1]) nzcv[1]   <= nzcv_in[1];
            if (FlagW[0]) nzcv[0]   <= nzcv_in[0];
        end
    end
    initial nzcv = 4'b0000;

    assign PCSrc     = PCS   & ex;
    assign RegWrite1 = RegW1 & ex;
    assign RegWrite2 = RegW2 & ex;
    assign MemWrite  = MemW  & ex;
    assign HaltEx    = Halt  & ex;

endmodule
