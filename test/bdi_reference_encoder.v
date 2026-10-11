`default_nettype none
// Four little-endian 32-bit words. Payload byte zero is bits [7:0].
// tag: 0 raw, 1 zero, 2 repeated word, 3 base32/delta8, 4 base32/delta16.
// Immediate mask bit i selects zero instead of the stored base for word i.
module bdi_reference_encoder (
    input wire [127:0] block_in,
    output reg [127:0] payload,
    output reg [2:0] tag,
    output reg [3:0] immediate_mask
);
    integer i;
    reg [31:0] word_value, base8, base16, difference;
    reg found8, found16, fits8, fits16, repeated;
    reg [127:0] candidate8, candidate16;
    reg [3:0] mask8, mask16;
    always @* begin
        base8 = block_in[31:0]; base16 = block_in[31:0];
        found8 = 1'b0; found16 = 1'b0;
        // Choose the first value that cannot be represented as an immediate.
        for (i=0; i<4; i=i+1) begin
            word_value = block_in[i*32 +: 32];
            if (!found8 && word_value != {{24{word_value[7]}},word_value[7:0]}) begin
                base8=word_value; found8=1'b1;
            end
            if (!found16 && word_value != {{16{word_value[15]}},word_value[15:0]}) begin
                base16=word_value; found16=1'b1;
            end
        end
        candidate8=128'd0; candidate16=128'd0;
        candidate8[31:0]=base8; candidate16[31:0]=base16;
        mask8=4'd0; mask16=4'd0; fits8=1'b1; fits16=1'b1; repeated=1'b1;
        difference=32'd0;
        for (i=0; i<4; i=i+1) begin
            word_value=block_in[i*32 +: 32];
            if (word_value != block_in[31:0]) repeated=1'b0;
            if (word_value == {{24{word_value[7]}},word_value[7:0]}) begin
                mask8[i]=1'b1; difference=word_value;
            end else difference=word_value-base8;
            if (difference != {{24{difference[7]}},difference[7:0]}) fits8=1'b0;
            candidate8[32+i*8 +: 8]=difference[7:0];
            if (word_value == {{16{word_value[15]}},word_value[15:0]}) begin
                mask16[i]=1'b1; difference=word_value;
            end else difference=word_value-base16;
            if (difference != {{16{difference[15]}},difference[15:0]}) fits16=1'b0;
            candidate16[32+i*16 +: 16]=difference[15:0];
        end
        payload=block_in; tag=3'd0; immediate_mask=4'd0;
        if (block_in == 128'd0) begin payload=128'd0; tag=3'd1; end
        else if (repeated) begin payload={96'd0,block_in[31:0]}; tag=3'd2; end
        else if (fits8) begin payload=candidate8; tag=3'd3; immediate_mask=mask8; end
        else if (fits16) begin payload=candidate16; tag=3'd4; immediate_mask=mask16; end
    end
endmodule
`default_nettype wire
