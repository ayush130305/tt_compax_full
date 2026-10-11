`timescale 1ns/1ps
// Randomized self-checking test of the controller, memory, codec and allocator (no UART framing).
module allocator_tb;
    // Clock, reset and controller handshake signals.
    reg clk=0; always #5 clk=~clk;
    reg rst=1, rx_valid=0; reg [7:0] rx_data=0;
    wire [7:0] tx_data, wdata, rdata; wire tx_start, we; wire [5:0] addr;
    // This contains the instantiation for the controller and the memory.
    compax_controller c(clk,rst,rx_data,rx_valid,tx_data,tx_start,1'b0,we,addr,wdata,rdata);
    flop_mem #(.DEPTH(64),.AW(6)) m(clk,we,addr,wdata,addr,rdata);
    // Reference encoder for the block under test.
    reg [127:0] ref_in; wire [127:0] ref_payload; wire [2:0] ref_tag; wire [3:0] ref_mask;
    bdi_reference_encoder refenc(ref_in,ref_payload,ref_tag,ref_mask);

    // Reply queue filled by the transmit strobe.
    reg [7:0] q [0:255]; integer qh=0, qt=0;
    always @(posedge clk) if (tx_start) begin q[qt[7:0]]=tx_data; qt=qt+1; end

    // Send one byte to the controller.
    task send(input [7:0] v); begin
        repeat(4) @(negedge clk); rx_data=v; rx_valid=1; @(negedge clk); rx_valid=0; repeat(2) @(negedge clk);
    end endtask
    // Pop the next reply, failing on timeout or mismatch.
    task expect_byte(input [7:0] v); integer t; begin
        t=0; while (qh==qt && t<2000) begin @(negedge clk); t=t+1; end
        if (qh==qt) $fatal(1,"timeout waiting for %h state=%0d op=%h qt=%0d",v,c.state,c.operation,qt);
        if (q[qh[7:0]]!==v) $fatal(1,"expected %h got %h (reply %0d)",v,q[qh[7:0]],qh);
        qh=qh+1;
    end endtask
    // Write 16 bytes to the scratch window.
    task put_scratch(input [127:0] d); integer k; begin
        send(8'h50); send(0); send(16); for (k=0;k<16;k=k+1) send(d[k*8+:8]); expect_byte(8'h4B);
    end endtask
    // Read the scratch window and compare.
    task check_scratch(input [127:0] d); integer k; begin
        send(8'h47); send(0); send(16); for (k=0;k<16;k=k+1) expect_byte(d[k*8+:8]);
    end endtask

    // Model of the block table.
    reg        mv [0:7]; reg [127:0] md [0:7]; integer moff [0:7]; integer mlen [0:7];
    integer used; integer n, op, step, k, j, cls, fails; reg [127:0] blk, junk; reg [31:0] b, v;
    integer exp_len; reg [7:0] bitmap; integer n_store=0,n_full=0,n_dup=0,n_load=0,n_free=0,n_empty=0,n_cls[0:4];
    initial for(k=0;k<5;k=k+1) n_cls[k]=0;

    // Build a random block of the requested class.
    task make_block(input integer cl); integer w; begin
        b = $random; blk = 0;
        for (w=0; w<4; w=w+1) begin
            case (cl)
                0: v = 32'd0;
                1: v = b;
                2: v = b + ($random & 32'h7F) - 32'h40;
                3: v = b + ($random & 32'h7FFF) - 32'h4000;
                4: v = $random;
                default: v = ($random & 1) ? ($random & 32'hFF) : b + ($random & 32'h3F);
            endcase
            blk[w*32+:32] = v;
        end
    end endtask

    initial begin
        for (n=0;n<8;n=n+1) begin mv[n]=0; moff[n]=0; mlen[n]=0; md[n]=0; end
        used=0; repeat(6) @(negedge clk); rst=0; repeat(4) @(negedge clk);
        // Basic raw access over the whole 64-byte range.
        send(8'h57); send(8'd63); send(8'hA5); expect_byte(8'h4B);
        send(8'h52); send(8'd63); expect_byte(8'hA5);
        send(8'h41); expect_byte(8'h3F);
        send(8'h51); expect_byte(48); expect_byte(0);
        for (step=0; step<3000; step=step+1) begin
            n = $urandom % 8; op = $urandom % 10;
            if (op < 5) begin
                // Store a random block.
                cls = $urandom % 6; make_block(cls); ref_in = blk; #1;
                put_scratch(blk);
                send(8'h43); send(n);
                exp_len = (ref_tag==0) ? 16 : (ref_tag==1) ? 0 : (ref_tag==2) ? 4 : (ref_tag==3) ? 8 : 12;
                if (mv[n]) begin expect_byte(8'h45); check_scratch(blk); n_dup=n_dup+1; end
                else if (exp_len > 48-used) begin expect_byte(8'h45); check_scratch(blk); n_full=n_full+1; end
                else begin
                    expect_byte(8'h4B); n_store=n_store+1; n_cls[ref_tag]=n_cls[ref_tag]+1;
                    mv[n]=1; md[n]=blk; moff[n]=used; mlen[n]=exp_len; used=used+exp_len;
                    // The packed payload must sit at the expected heap address.
                    for (k=0;k<exp_len;k=k+1)
                        if (m.mem[16+moff[n]+k]!==ref_payload[k*8+:8])
                            $fatal(1,"step %0d: heap byte %0d of block %0d wrong",step,k,n);
                    send(8'h4D); send(n); expect_byte({1'b1,ref_mask,ref_tag}); expect_byte(exp_len);
                end
            end else if (op < 8) begin
                // Load a block after trashing the scratch window.
                junk = {$random,$random,$random,$random}; put_scratch(junk);
                send(8'h44); send(n);
                if (!mv[n]) expect_byte(8'h45);
                else begin expect_byte(8'h4B); check_scratch(md[n]); n_load=n_load+1; end
            end else begin
                // Free a block and update the model like the hardware compaction does.
                send(8'h46); send(n);
                if (!mv[n]) expect_byte(8'h45);
                else begin
                    expect_byte(8'h4B); n_free=n_free+1; mv[n]=0; used=used-mlen[n];
                    for (j=0;j<8;j=j+1) if (mv[j] && moff[j]>moff[n]) moff[j]=moff[j]-mlen[n];
                end
            end
            // Status must match the model after every operation.
            bitmap=0; for (j=0;j<8;j=j+1) bitmap[j]=mv[j];
            send(8'h51); expect_byte(48-used); expect_byte(bitmap);
        end
        // Final check: every surviving block still decodes correctly.
        for (n=0;n<8;n=n+1) if (mv[n]) begin
            send(8'h44); send(n); expect_byte(8'h4B); check_scratch(md[n]);
        end
        // Invalid indices.
        send(8'h43); send(8); expect_byte(8'h3F);
        send(8'h46); send(200); expect_byte(8'h3F);
        $display("stores %0d (tags 0-4: %0d %0d %0d %0d %0d) heap-full %0d dup %0d loads %0d frees %0d",n_store,n_cls[0],n_cls[1],n_cls[2],n_cls[3],n_cls[4],n_full,n_dup,n_load,n_free);
        $display("PASS: allocator random test, %0d operations, final heap use %0d bytes",step,used);
        $finish;
    end
    initial begin #2000000000; $fatal(1,"global timeout"); end
endmodule
