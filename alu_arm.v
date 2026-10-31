//====================================================================
// alu_arm.v   (ARMv4 single-cycle CPU, step 2)
//
//   32-bit ALU for all 16 ARM data-processing operations, with the
//   NZCV outputs the ARM flag logic needs.  Specification: arm_sc_spec.md
//   section 6.3.  The ALU control is the data-processing opcode itself
//   (Instr[24:21]); the decoder drives ADD / SUB for loads, stores and
//   branches and MOV for BX.
//
//     ALUControl  op    result              adder operands (X + Y + cin)
//     ----------  ----  ------------------  ---------------------------
//     0000        AND   A & B
//     0001        EOR   A ^ B
//     0010        SUB   A - B               A  + ~B + 1
//     0011        RSB   B - A               ~A +  B + 1
//     0100        ADD   A + B               A  +  B + 0
//     0101        ADC   A + B + C           A  +  B + C
//     0110        SBC   A - B - ~C          A  + ~B + C
//     0111        RSC   B - A - ~C          ~A +  B + C
//     1000        TST   A & B   (no write)
//     1001        TEQ   A ^ B   (no write)
//     1010        CMP   A - B   (no write)  A  + ~B + 1
//     1011        CMN   A + B   (no write)  A  +  B + 0
//     1100        ORR   A | B
//     1101        MOV   B
//     1110        BIC   A & ~B
//     1111        MVN   ~B
//
//   Outputs
//     Result : the ALU result (the compare / test opcodes also produce it; the
//              decoder simply does not write it to a register)
//     N      : Result[31]
//     Z      : Result == 0
//     Cadd   : carry out of the adder. Meaningful only for the arithmetic opcodes
//              (SUB RSB ADD ADC SBC RSC CMP CMN). For the logical opcodes the
//              decoder takes C from the barrel shifter instead (CSrc = 1).
//     Vadd   : signed overflow of the adder (arithmetic opcodes only; logical
//              opcodes leave V unchanged: the decoder does not write V for them).
//
//   Structure (why it is small and fast on a Cyclone IV):
//     ONE adder  X + Y + cin  is shared by all eight arithmetic opcodes. The
//     operands are X = A or ~A and Y = B or ~B, the carry-in is 0, 1 or C.
//     It is written with the "+" operator, which Quartus maps to the dedicated
//     carry chain (a ripple of 32 chain cells, no logic-cell carry logic).
//     cin is folded into the sum as a 34-bit add {X,1} + {Y,cin}: the extra
//     low bit makes the carry-in part of the same chain, so no second adder.
//====================================================================
module alu_arm (
    input  [31:0] A,           // RD1 (Rn)
    input  [31:0] B,           // SrcB (shifter output or extended immediate)
    input  [3:0]  ALUControl,
    input         Cin,         // current C flag (ADC, SBC, RSC)
    output reg [31:0] Result,
    output        N,
    output        Z,
    output        Cadd,
    output        Vadd
);

    // ---------------- operand conditioning for the shared adder ----------------
    wire invA = (ALUControl == 4'b0011) | (ALUControl == 4'b0111);               // RSB, RSC
    wire invB = (ALUControl == 4'b0010) | (ALUControl == 4'b0110) |
                (ALUControl == 4'b1010);                                         // SUB, SBC, CMP

    reg c_in;                                                                    // adder carry-in
    always @(*) begin
        case (ALUControl)
            4'b0010, 4'b0011, 4'b1010: c_in = 1'b1;       // SUB RSB CMP : +1
            4'b0101, 4'b0110, 4'b0111: c_in = Cin;        // ADC SBC RSC : +C
            default:                   c_in = 1'b0;       // ADD CMN (and unused for logic)
        endcase
    end

    wire [31:0] X = invA ? ~A : A;
    wire [31:0] Y = invB ? ~B : B;

    // {X,1} + {Y,c_in}: bit 0 of the 34-bit sum is discarded, bit 33 is the carry out
    wire [33:0] sum34 = {1'b0, X, 1'b1} + {1'b0, Y, c_in};
    wire [31:0] sum   = sum34[32:1];
    assign Cadd = sum34[33];
    // signed overflow of X + Y + cin: operands (after inversion) of equal sign, result of the other sign
    assign Vadd = (X[31] == Y[31]) & (sum[31] != X[31]);

    // ---------------- result select ----------------
    always @(*) begin
        case (ALUControl)
            4'b0000, 4'b1000: Result = A & B;               // AND, TST
            4'b0001, 4'b1001: Result = A ^ B;               // EOR, TEQ
            4'b1100:          Result = A | B;               // ORR
            4'b1101:          Result = B;                   // MOV
            4'b1110:          Result = A & ~B;              // BIC
            4'b1111:          Result = ~B;                  // MVN
            default:          Result = sum;                 // SUB RSB ADD ADC SBC RSC CMP CMN
        endcase
    end

    assign N = Result[31];
    assign Z = (Result == 32'd0);

endmodule
