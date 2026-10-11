// COMPAX: UART-controlled 64-byte flop memory with a 16-byte BDI codec.
// ui_in[0] = UART RX; uo_out[0] = UART TX.

`default_nettype none

// Decorative metal artwork macro: no ports, black box for synthesis.
(* blackbox *) (* keep *)
module compax_art ();
endmodule

module tt_um_ayush130305_compax (
    input  wire [7:0] ui_in,   // dedicated inputs
    output wire [7:0] uo_out,  // dedicated outputs
    input  wire [7:0] uio_in,  // bidirectional pins, input side
    output wire [7:0] uio_out, // bidirectional pins, output side
    output wire [7:0] uio_oe,  // bidirectional pins, output enables
    input  wire       ena,     // goes high when the design is selected
    input  wire       clk,     // system clock
    input  wire       rst_n    // active-low reset
);
    // This contains the instantiation for the decorative artwork macro.
    (* keep *)
    compax_art art();

    // UART timing parameters, matched to clock_hz in info.yaml.
    localparam integer CLK_FREQ  = 25_000_000;
    localparam integer BAUD_RATE = 115_200;

    // Active-high reset for the UART blocks.
    wire rst = ~rst_n;

    // Wires between receiver, command FSM, transmitter and memory.
    wire [7:0] rx_data, tx_data, mem_wdata, mem_rdata;
    wire       rx_valid, tx_start, tx_busy, tx_line, mem_we;
    wire [5:0] mem_addr;
    wire unused_framing_error;

    // This contains the instantiation for the UART receiver.
    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) u_rx (
        .clk(clk), .rst(rst), .rx_line(ui_in[0]),
        .rx_data(rx_data), .rx_valid(rx_valid), .framing_error(unused_framing_error)
    );

    // This contains the instantiation for the command FSM.
    compax_controller u_fsm (
        .clk(clk), .rst(rst),
        .rx_data(rx_data), .rx_valid(rx_valid),
        .tx_data(tx_data), .tx_start(tx_start), .tx_busy(tx_busy),
        .mem_we(mem_we), .mem_addr(mem_addr), .mem_wdata(mem_wdata), .mem_rdata(mem_rdata)
    );

    // This contains the instantiation for the UART transmitter.
    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) u_tx (
        .clk(clk), .rst(rst), .tx_start(tx_start), .tx_data(tx_data),
        .tx_line(tx_line), .tx_busy(tx_busy)
    );

    // This contains the instantiation for the flop memory.
    flop_mem #(.DEPTH(64), .AW(6)) u_mem (
        .clk(clk), .we(mem_we), .waddr(mem_addr), .wdata(mem_wdata),
        .raddr(mem_addr), .rdata(mem_rdata)
    );

    // Outputs: only the UART TX line is used.
    assign uo_out  = {7'b0000000, tx_line};
    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;

    // Unused inputs.
    wire _unused = &{ena, ui_in[7:1], uio_in, unused_framing_error, 1'b0};
endmodule

`default_nettype wire
