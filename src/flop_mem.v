// 64 x 8 flip-flop storage; synchronous write, combinational read.

`default_nettype none

module flop_mem #(
    parameter integer DEPTH = 64, // number of rows
    parameter integer AW    = 6   // address width, clog2(DEPTH)
) (
    input  wire          clk,   // system clock
    input  wire          we,    // write enable, valid for one cycle
    input  wire [AW-1:0] waddr, // write address
    input  wire [7:0]    wdata, // write data
    input  wire [AW-1:0] raddr, // read address
    output wire [7:0]    rdata  // read data (combinational)
);
    // Storage rows.
    reg [7:0] mem [0:DEPTH-1];

    // This always block writes the addressed row on the rising clock edge.
    always @(posedge clk) begin
        // Only write when the controller requests it.
        if (we) mem[waddr] <= wdata;
    end

    // Combinational read port.
    assign rdata = mem[raddr];
endmodule

`default_nettype wire
