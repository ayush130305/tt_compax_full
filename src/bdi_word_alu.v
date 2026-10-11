`default_nettype none
// Shared word arithmetic: word - base while encoding, word + base while decoding.
module bdi_word_alu (
    input wire [31:0] word_value,
    input wire [31:0] base_value,
    input wire subtract,
    output wire [31:0] result
);
    wire [31:0] operand = base_value ^ {32{subtract}};
    assign result = word_value + operand + {31'd0,subtract};
endmodule
`default_nettype wire
