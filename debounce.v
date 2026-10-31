//====================================================================
// debounce.v
// Cleans up a noisy mechanical button signal, and produces a single
// one-cycle pulse each time it's pressed (rising edge), instead of a
// burst of noisy transitions.
//
// Use this ONLY for buttons that need to trigger a one-time action
// (like "advance to next address"). Buttons used as live level
// signals (like your AND/OR/ADD/SUB selection) don't need this.
//====================================================================

module debounce #(
    parameter DEBOUNCE_LIMIT = 250000  // ~5ms at 50MHz; raise if still bouncy
) (
    input  clk,
    input  btn_in,       // raw button signal, already leveled so 1 = pressed
    output reg btn_out   // clean, debounced level (1 = pressed)
);
    reg [17:0] counter = 0;
    reg sync0, sync1;

    // Synchronize the async button signal to our clock domain
    always @(posedge clk) begin
        sync0 <= btn_in;
        sync1 <= sync0;
    end

    // Only accept a change once it's been stable for DEBOUNCE_LIMIT cycles
    always @(posedge clk) begin
        if (sync1 == btn_out) begin
            counter <= 0;
        end else begin
            counter <= counter + 1;
            if (counter >= DEBOUNCE_LIMIT) begin
                btn_out <= sync1;
                counter <= 0;
            end
        end
    end
endmodule


//====================================================================
// edge_detector.v
// Produces a single 1-clock-cycle pulse on the rising edge of a
// (already debounced) signal. This turns "button held down for
// 200ms" into "one clean pulse the instant it was pressed."
//====================================================================
module edge_detector (
    input  clk,
    input  signal_in,
    output pulse_out
);
    reg prev = 0;
    always @(posedge clk)
        prev <= signal_in;

    assign pulse_out = signal_in & ~prev;
endmodule