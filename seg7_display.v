//====================================================================
// seg7_display.v  (v6 - FINAL, corrected using precise hardware test)
//
// CONFIRMED (from the detailed "6 lit / 1 dark" + "3 digits / 1 dark"
// test reports):
//  - SEG (segments): ACTIVE-LOW (0 = segment on) -> invert
//  - AN (digit select): ACTIVE-LOW (0 = this digit selected) -> invert
//====================================================================
module seg7_display (
    input        clk,
    input [15:0] hex_value,
    input        cout,
    output [7:0] seg,     // seg[6:0] = a-g, seg[7] = decimal point
    output [3:0] an
);

    reg [18:0] refresh_counter = 0;
    always @(posedge clk)
        refresh_counter <= refresh_counter + 1'b1;

    wire [1:0] digit_sel = refresh_counter[18:17];

    reg [3:0] nibble;
    always @(*) begin
        case (digit_sel)
            2'd0: nibble = hex_value[3:0];
            2'd1: nibble = hex_value[7:4];
            2'd2: nibble = hex_value[11:8];
            2'd3: nibble = hex_value[15:12];
            default: nibble = 4'h0;
        endcase
    end

    wire [6:0] seg_bits;              // active-HIGH pattern from decoder
    seg7_decoder dec (.hex(nibble), .seg(seg_bits));

    wire dp_bit = (digit_sel == 2'd3) ? cout : 1'b0;
    wire [7:0] seg_active_high = {dp_bit, seg_bits};

    assign seg = ~seg_active_high;               // invert: active-LOW segments
    assign an  = ~(4'b0001 << digit_sel);         // invert: active-LOW digit select

endmodule