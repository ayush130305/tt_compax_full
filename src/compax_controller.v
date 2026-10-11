// compax_controller.v
// UART controller for a 64-byte flop memory with a BDI codec and a block allocator.
// Physical map: bytes 0..15 are the scratch window, bytes 16..63 are a packed 48-byte heap.
// The host fills the scratch window with raw data, then stores it as one of 8 logical blocks.
//   'W' addr data        write one byte (addr 0..63)     -> 'K'
//   'R' addr             read one byte                   -> data
//   'P' addr n d0..dn-1  page write, n bytes             -> 'K'   (address wraps at 64)
//   'G' addr n           page read, n bytes              -> n data bytes
//   'C' blk              compress scratch into block blk -> 'K', or 'E' (blk used / heap full)
//   'D' blk              decompress block blk to scratch -> 'K', or 'E' (blk empty)
//   'F' blk              free block blk, compact heap    -> 'K', or 'E' (blk empty)
//   'M' blk              block metadata                  -> 2 bytes {valid,mask,tag}, length
//   'Q'                  allocator status                -> 2 bytes free heap bytes, valid bitmap
//   anything else                                        -> '?'

`default_nettype none

module compax_controller (
    input  wire       clk,       // system clock
    input  wire       rst,       // active-high asynchronous reset
    input  wire [7:0] rx_data,   // received byte
    input  wire       rx_valid,  // received byte strobe
    output reg  [7:0] tx_data,   // byte to transmit
    output reg        tx_start,  // transmit strobe
    input  wire       tx_busy,   // transmitter busy
    output reg        mem_we,    // memory write strobe
    output reg  [5:0] mem_addr,  // memory address for both ports
    output reg  [7:0] mem_wdata, // memory write data
    input  wire [7:0] mem_rdata  // memory read data
);
    // State encodings.
    localparam [4:0] S_IDLE = 5'd0, S_W_ADDR = 5'd1, S_W_DATA = 5'd2, S_P_ADDR = 5'd3,
                     S_P_LEN = 5'd4, S_P_DATA = 5'd5, S_P_NEXT = 5'd6, S_R_ADDR = 5'd7,
                     S_G_ADDR = 5'd8, S_G_LEN = 5'd9, S_RD_SEND = 5'd10, S_RD_GAP = 5'd11,
                     S_REPLY = 5'd12, S_BLOCK_IDX = 5'd13, S_META_SEND = 5'd14,
                     S_META_GAP = 5'd15, S_RW = 5'd16, S_RW_DONE = 5'd17, S_DECIDE = 5'd18,
                     S_WRITE = 5'd19, S_WSTEP = 5'd20, S_AFTER = 5'd21, S_COMMIT = 5'd22,
                     S_DR = 5'd23, S_DR_END = 5'd24, S_DCOMP = 5'd25, S_CFIT = 5'd26,
                     S_DEC_START = 5'd27, S_CP_CHK = 5'd28, S_CP_A = 5'd29, S_CP_B = 5'd30,
                     S_CP_C = 5'd31;
    // What to do when a word read finishes (phase) and when a byte write run finishes (after).
    localparam [2:0] PH_BASE = 3'd0, PH_CHECK = 3'd1, PH_PACK = 3'd2, PH_DBASE = 3'd3;
    localparam [2:0] A_PACK = 3'd0, A_BASE = 3'd1, A_COMMIT = 3'd2, A_DEC = 3'd3, A_DEC2 = 3'd4;
    // What to do when a byte-copy run finishes.
    localparam [1:0] R_STORE = 2'd0, R_LOAD = 2'd1, R_FREE = 2'd2;
    // First heap address and heap size in bytes.
    localparam [5:0] HEAP_BASE = 6'd16;
    localparam [5:0] HEAP_SIZE = 6'd48;

    // Current state.
    reg [4:0] state;
    // Bytes left in a page transfer.
    reg [6:0] remaining;
    // Reply byte for single-byte replies.
    reg [7:0] reply;
    // Cycles waited after starting a transmission.
    reg [1:0] gap;

    // Format tag and immediate-selector mask of the data currently in the scratch window.
    reg [2:0] stag;
    reg [3:0] smask;
    // Block table: one entry per logical block.
    reg [7:0] e_valid;
    reg [2:0] e_tag  [0:7];
    reg [3:0] e_mask [0:7];
    reg [5:0] e_off  [0:7];
    reg [4:0] e_len  [0:7];
    // Number of heap bytes in use.
    reg [5:0] heap_end;
    // Logical block addressed by the current operation.
    reg [2:0] idx;
    // Command letter of the block operation in progress.
    reg [7:0] operation;
    // Second byte returned by M and Q.
    reg [7:0] meta_length;
    // Byte-copy engine registers.
    reg [5:0] cp_src, cp_dst, cp_cnt;
    reg [1:0] cp_ret;
    // Loop variable used by the reset block and the compaction update.
    integer i;

    // Payload length in bytes for a format tag.
    function [4:0] payload_length;
        input [2:0] mode;
        begin
            case(mode)
                3'd1: payload_length=5'd0;
                3'd2: payload_length=5'd4;
                3'd3: payload_length=5'd8;
                3'd4: payload_length=5'd12;
                default: payload_length=5'd16;
            endcase
        end
    endfunction

    // Working registers: the word being processed, the base word and two saved deltas.
    reg [31:0] cur, base;
    reg [15:0] d, d0;
    // Word index, byte index inside a word, and bytes left in a write run.
    reg [1:0] wi, bi;
    reg [3:0] nb;
    // Sequencing selectors.
    reg [2:0] phase, after;
    // Delta width flag, scan flags and the prefetched-delta flag.
    reg wide, found, all_fit, zero_ok, rep_ok, pf;
    // Result tag and immediate mask being built for the scratch window.
    reg [2:0] ctag;
    reg [3:0] cmask;

    // Payload length of the data in the scratch window, and the free heap space.
    wire [4:0] plen = payload_length(stag);
    wire [5:0] heap_free = HEAP_SIZE - heap_end;

    // One 32-bit adder/subtractor shared by encoding and decoding.
    wire decoding = (operation==8'h44);
    wire [15:0] dx = (wi==2'd0) ? d0 : d;
    wire [31:0] alu_word = decoding ? {{16{dx[15]}},dx} : cur;
    wire imm_fit = wide ? (cur=={{16{cur[15]}},cur[15:0]}) : (cur=={{24{cur[7]}},cur[7:0]});
    wire use_zero = decoding ? smask[wi] : imm_fit;
    wire [31:0] alu_base = use_zero ? 32'd0 : base;
    wire [31:0] alu_result;
    wire delta_fit = wide ? (alu_result=={{16{alu_result[15]}},alu_result[15:0]}) :
                            (alu_result=={{24{alu_result[7]}},alu_result[7:0]});
    // This contains the instantiation for the shared word ALU.
    bdi_word_alu u_codec_alu (.word_value(alu_word), .base_value(alu_base),
        .subtract(!decoding), .result(alu_result));

    // Start address of delta k inside the scratch window.
    function [3:0] delta_addr;
        input        w16;
        input [1:0]  k;
        begin
            delta_addr = w16 ? (4'd4 + {1'b0,k,1'b0}) : (4'd4 + {2'b00,k});
        end
    endfunction

    // This always block is the command state machine, the codec sequencer and the allocator.
    always @(posedge clk or posedge rst) begin
        // Asynchronous reset puts everything in its idle state.
        if (rst) begin
            // Datapath registers are loaded before use and need no reset.
            stag <= 3'd0; smask <= 4'd0; e_valid <= 8'd0; heap_end <= 6'd0;
            idx <= 3'd0; operation <= 8'd0; meta_length <= 8'd0;
            state     <= S_IDLE;
            remaining <= 7'd0;
            reply     <= 8'd0;
            gap       <= 2'd0;
            tx_data   <= 8'd0;
            tx_start  <= 1'b0;
            mem_we    <= 1'b0;
            mem_addr  <= 6'd0;
            mem_wdata <= 8'd0;
        end else begin
            // Strobes last one cycle by default.
            tx_start <= 1'b0;
            mem_we   <= 1'b0;

            // Dispatch on the current state.
            case (state)
                // Wait for a command byte.
                S_IDLE: begin
                    // Only act when a byte has arrived.
                    if (rx_valid) begin
                        // Pick the command by its ASCII code.
                        case (rx_data)
                            8'h43, 8'h44, 8'h46, 8'h4D: begin operation<=rx_data; state<=S_BLOCK_IDX; end
                            8'h51: begin                                       // 'Q'
                                tx_data<={2'b00,heap_free}; meta_length<=e_valid; state<=S_META_SEND;
                            end
                            8'h57:   state <= S_W_ADDR;                       // 'W'
                            8'h52:   state <= S_R_ADDR;                       // 'R'
                            8'h50:   state <= S_P_ADDR;                       // 'P'
                            8'h47:   state <= S_G_ADDR;                       // 'G'
                            default: begin reply <= 8'h3F; state <= S_REPLY; end // '?'
                        endcase
                    end
                end
                // Single write: address byte.
                S_W_ADDR: if (rx_valid) begin
                    mem_addr <= rx_data[5:0];
                    state    <= S_W_DATA;
                end
                // Single write: data byte, then strobe the memory.
                S_W_DATA: if (rx_valid) begin
                    mem_wdata<=rx_data; mem_we<=1'b1; reply<=8'h4B;
                    state<=S_REPLY;
                end
                // Page write: start address.
                S_P_ADDR: if (rx_valid) begin
                    mem_addr <= rx_data[5:0];
                    state    <= S_P_LEN;
                end
                // Page write: byte count (0 means nothing to write).
                S_P_LEN: if (rx_valid) begin
                    remaining <= rx_data[6:0];
                    reply     <= 8'h4B;
                    state     <= (rx_data[6:0] == 7'd0) ? S_REPLY : S_P_DATA;
                end
                // Page write: one data byte, strobe the memory.
                S_P_DATA: if (rx_valid) begin
                    mem_wdata <= rx_data;
                    mem_we    <= 1'b1;
                    remaining <= remaining - 7'd1;
                    state     <= S_P_NEXT;
                end
                // Page write: advance the address after the write strobe cycle.
                S_P_NEXT: begin
                    mem_addr <= mem_addr + 6'd1;
                    state    <= (remaining == 7'd0) ? S_REPLY : S_P_DATA;
                end
                // Single read: address byte, then send one byte.
                S_R_ADDR: if (rx_valid) begin
                    mem_addr  <= rx_data[5:0];
                    remaining <= 7'd1;
                    state     <= S_RD_SEND;
                end
                // Page read: start address.
                S_G_ADDR: if (rx_valid) begin
                    mem_addr <= rx_data[5:0];
                    state    <= S_G_LEN;
                end
                // Page read: byte count.
                S_G_LEN: if (rx_valid) begin
                    remaining <= rx_data[6:0];
                    state     <= (rx_data[6:0] == 7'd0) ? S_IDLE : S_RD_SEND;
                end
                // Send the addressed byte once the transmitter is free.
                S_RD_SEND: if (!tx_busy) begin
                    tx_data   <= mem_rdata;
                    tx_start  <= 1'b1;
                    remaining <= remaining - 7'd1;
                    gap       <= 2'd0;
                    state     <= S_RD_GAP;
                end
                // Let the transmitter raise busy, then continue or finish.
                S_RD_GAP: begin
                    gap <= gap + 2'd1;
                    if (gap == 2'd2) begin
                        // Last byte done, or step to the next address.
                        if (remaining == 7'd0) state <= S_IDLE;
                        else begin
                            mem_addr <= mem_addr + 6'd1;
                            state    <= S_RD_SEND;
                        end
                    end
                end
                // Send the single reply byte once the transmitter is free.
                S_REPLY: if (!tx_busy) begin
                    tx_data  <= reply;
                    tx_start <= 1'b1;
                    state    <= S_IDLE;
                end
                // C/D/F/M take a block index 0..7.
                S_BLOCK_IDX: if (rx_valid) begin
                    if (rx_data > 8'd7) begin reply<=8'h3F; state<=S_REPLY; end
                    else begin
                        idx<=rx_data[2:0]; bi<=2'd0; wi<=2'd0; pf<=1'b0;
                        mem_addr<=6'd0; reply<=8'h4B; ctag<=3'd0; cmask<=4'd0;
                        case (operation)
                            // Metadata of the block (zero if the block is empty).
                            8'h4D: begin
                                tx_data<=e_valid[rx_data[2:0]] ?
                                    {1'b1,e_mask[rx_data[2:0]],e_tag[rx_data[2:0]]} : 8'd0;
                                meta_length<=e_valid[rx_data[2:0]] ? {3'd0,e_len[rx_data[2:0]]} : 8'd0;
                                state<=S_META_SEND;
                            end
                            // Compress the scratch window into an unused block.
                            8'h43: begin
                                if (e_valid[rx_data[2:0]]) begin reply<=8'h45; state<=S_REPLY; end
                                else begin wide<=1'b0; found<=1'b0; phase<=PH_BASE; state<=S_RW; end
                            end
                            // Decompress a block into the scratch window: copy its payload first.
                            8'h44: begin
                                if (!e_valid[rx_data[2:0]]) begin reply<=8'h45; state<=S_REPLY; end
                                else begin
                                    cp_src<=HEAP_BASE+e_off[rx_data[2:0]]; cp_dst<=6'd0;
                                    cp_cnt<={1'b0,e_len[rx_data[2:0]]}; cp_ret<=R_LOAD;
                                    stag<=e_tag[rx_data[2:0]]; smask<=e_mask[rx_data[2:0]];
                                    state<=S_CP_CHK;
                                end
                            end
                            // Free a block: shift the heap bytes above it down.
                            default: begin
                                if (!e_valid[rx_data[2:0]]) begin reply<=8'h45; state<=S_REPLY; end
                                else begin
                                    cp_dst<=HEAP_BASE+e_off[rx_data[2:0]];
                                    cp_src<=HEAP_BASE+e_off[rx_data[2:0]]+{1'b0,e_len[rx_data[2:0]]};
                                    cp_cnt<=heap_end-e_off[rx_data[2:0]]-{1'b0,e_len[rx_data[2:0]]};
                                    cp_ret<=R_FREE; state<=S_CP_CHK;
                                end
                            end
                        endcase
                    end
                end
                // Read one 32-bit word, a byte per cycle, shifting into cur.
                S_RW: begin
                    cur <= {mem_rdata, cur[31:8]};
                    if (bi==2'd3) begin bi<=2'd0; state<=S_RW_DONE; end
                    else begin bi<=bi+2'd1; mem_addr<=mem_addr+6'd1; end
                end
                // A word is in cur: act on it according to the phase.
                S_RW_DONE: begin
                    case (phase)
                        // Find the first word that is not an immediate; it becomes the base.
                        PH_BASE: begin
                            if (wi==2'd0) begin base<=cur; found<=!imm_fit; end
                            else if (!found && !imm_fit) begin base<=cur; found<=1'b1; end
                            if (wi==2'd3) begin
                                wi<=2'd0; phase<=PH_CHECK; all_fit<=1'b1; zero_ok<=1'b1;
                                rep_ok<=1'b1; cmask<=4'd0; mem_addr<=6'd0;
                            end else begin wi<=wi+2'd1; mem_addr<={2'b00,wi+2'd1,2'b00}; end
                            bi<=2'd0; state<=S_RW;
                        end
                        // Test every word against the base before touching the memory.
                        PH_CHECK: begin
                            cmask[wi] <= imm_fit;
                            if (!delta_fit) all_fit<=1'b0;
                            if (cur!=32'd0) zero_ok<=1'b0;
                            if (cur!=base) rep_ok<=1'b0;
                            if (wi==2'd3) state<=S_DECIDE;
                            else begin wi<=wi+2'd1; mem_addr<={2'b00,wi+2'd1,2'b00}; bi<=2'd0; state<=S_RW; end
                        end
                        // Write deltas in place, delaying delta 0 until word 1 has been read.
                        PH_PACK: begin
                            case (wi)
                                2'd0: begin d0<=alu_result[15:0]; wi<=2'd1; mem_addr<=6'd4; bi<=2'd0; state<=S_RW; end
                                2'd1: begin
                                    cur <= wide ? {alu_result[15:0],d0} : {16'd0,alu_result[7:0],d0[7:0]};
                                    nb <= wide ? 4'd4 : 4'd2; mem_addr<=6'd4;
                                    after<=A_PACK; state<=S_WRITE;
                                end
                                default: begin
                                    cur <= {16'd0,alu_result[15:0]};
                                    nb <= wide ? 4'd2 : 4'd1;
                                    mem_addr <= {2'b00,delta_addr(wide,wi)};
                                    after<=A_PACK; state<=S_WRITE;
                                end
                            endcase
                        end
                        // Decode: the base word is in cur.
                        default: begin
                            base<=cur;
                            if (stag==3'd2) begin
                                wi<=2'd1; nb<=4'd4; mem_addr<=6'd4; after<=A_DEC2; state<=S_WRITE;
                            end else begin
                                wi<=2'd3; pf<=1'b0; bi<=2'd0;
                                mem_addr<={2'b00,delta_addr(wide,2'd3)}; state<=S_DR;
                            end
                        end
                    endcase
                end
                // Choose the format from the scan results.
                S_DECIDE: begin
                    bi<=2'd0; wi<=2'd0; mem_addr<=6'd0;
                    if (zero_ok) begin ctag<=3'd1; cmask<=4'd0; state<=S_COMMIT; end
                    else if (rep_ok) begin
                        ctag<=3'd2; cmask<=4'd0; cur<=32'd0; nb<=4'd12;
                        mem_addr<=6'd4; after<=A_COMMIT; state<=S_WRITE;
                    end
                    else if (all_fit) begin ctag<=wide ? 3'd4 : 3'd3; phase<=PH_PACK; state<=S_RW; end
                    else if (!wide) begin wide<=1'b1; found<=1'b0; phase<=PH_BASE; state<=S_RW; end
                    else begin ctag<=3'd0; cmask<=4'd0; state<=S_COMMIT; end
                end
                // Write nb bytes taken from cur, one byte per two cycles.
                S_WRITE: begin
                    mem_wdata<=cur[7:0]; mem_we<=1'b1; cur<={8'd0,cur[31:8]};
                    nb<=nb-4'd1; state<=S_WSTEP;
                end
                // The write strobe is active this cycle; then move to the next byte or finish.
                S_WSTEP: begin
                    if (nb==4'd0) state<=S_AFTER;
                    else begin mem_addr<=mem_addr+6'd1; state<=S_WRITE; end
                end
                // A write run has finished: continue the operation.
                S_AFTER: begin
                    case (after)
                        // Pack: next word, or write the base and clear the unused tail.
                        A_PACK: begin
                            if (wi==2'd3) begin
                                cur<=base; nb<=4'd4; mem_addr<=6'd0; after<=A_BASE; state<=S_WRITE;
                            end else begin
                                wi<=wi+2'd1; mem_addr<={2'b00,wi+2'd1,2'b00}; bi<=2'd0; state<=S_RW;
                            end
                        end
                        A_BASE: begin
                            cur<=32'd0; nb<=wide ? 4'd4 : 4'd8;
                            mem_addr<=wide ? 6'd12 : 6'd8;
                            after<=A_COMMIT; state<=S_WRITE;
                        end
                        A_COMMIT: state<=S_COMMIT;
                        // Decompress: next word, using the delta read ahead for word 0.
                        A_DEC: begin
                            if (wi==2'd0) state<=S_COMMIT;
                            else if (wi==2'd1) begin wi<=2'd0; state<=S_DCOMP; end
                            else begin
                                wi<=wi-2'd1; bi<=2'd0;
                                mem_addr<={2'b00,delta_addr(wide,wi-2'd1)}; state<=S_DR;
                            end
                        end
                        // Copy the base word into words wi+1.. (formats 1 and 2 decompress).
                        default: begin
                            if (wi==2'd3) state<=S_COMMIT;
                            else begin
                                wi<=wi+2'd1; cur<=base; nb<=4'd4;
                                mem_addr<={2'b00,wi+2'd1,2'b00}; after<=A_DEC2; state<=S_WRITE;
                            end
                        end
                    endcase
                end
                // Record the new scratch tag and mask; a compress then goes on to allocate.
                S_COMMIT: begin
                    stag<=ctag; smask<=cmask;
                    state <= (operation==8'h43) ? S_CFIT : S_REPLY;
                end
                // Check that the compressed payload fits in the heap, then copy it there.
                S_CFIT: begin
                    if ({1'b0,plen} > heap_free) begin
                        // No room: undo the compression so the scratch window keeps the raw data.
                        reply<=8'h45;
                        if (stag==3'd0) state<=S_REPLY;
                        else begin operation<=8'h44; state<=S_DEC_START; end
                    end else begin
                        cp_src<=6'd0; cp_dst<=HEAP_BASE+heap_end; cp_cnt<={1'b0,plen};
                        cp_ret<=R_STORE; state<=S_CP_CHK;
                    end
                end
                // Start decompressing the payload in the scratch window according to stag.
                S_DEC_START: begin
                    bi<=2'd0; wi<=2'd0; pf<=1'b0; mem_addr<=6'd0; ctag<=3'd0; cmask<=4'd0;
                    operation<=8'h44;
                    if (stag==3'd0) state<=S_REPLY;
                    else if (stag==3'd1) begin
                        // Zero block: write four zero words.
                        base<=32'd0; cur<=32'd0; nb<=4'd4; after<=A_DEC2; state<=S_WRITE;
                    end else begin
                        // Formats 2, 3 and 4: read the base word first.
                        wide<=(stag==3'd4); phase<=PH_DBASE; state<=S_RW;
                    end
                end
                // Copy loop head: finish when no bytes are left, else copy the next byte.
                S_CP_CHK: begin
                    if (cp_cnt==6'd0) begin
                        case (cp_ret)
                            // Payload stored: fill in the block table entry and grow the heap.
                            R_STORE: begin
                                e_valid[idx]<=1'b1; e_tag[idx]<=stag; e_mask[idx]<=smask;
                                e_off[idx]<=heap_end; e_len[idx]<=plen;
                                heap_end<=heap_end+{1'b0,plen};
                                reply<=8'h4B; state<=S_REPLY;
                            end
                            // Payload loaded into the scratch window: expand it.
                            R_LOAD: state<=S_DEC_START;
                            // Heap compacted: close the gap in the table and shrink the heap.
                            default: begin
                                for (i=0;i<8;i=i+1)
                                    if (e_valid[i] && e_off[i]>e_off[idx]) e_off[i]<=e_off[i]-{1'b0,e_len[idx]};
                                e_valid[idx]<=1'b0;
                                heap_end<=heap_end-{1'b0,e_len[idx]};
                                reply<=8'h4B; state<=S_REPLY;
                            end
                        endcase
                    end else begin mem_addr<=cp_src; state<=S_CP_A; end
                end
                // The address is applied; wait one cycle for the combinational read.
                S_CP_A: state<=S_CP_B;
                // Read data is valid: write it to the destination address.
                S_CP_B: begin
                    mem_wdata<=mem_rdata; mem_addr<=cp_dst; mem_we<=1'b1; state<=S_CP_C;
                end
                // The write strobe is active this cycle; advance the pointers.
                S_CP_C: begin
                    cp_src<=cp_src+6'd1; cp_dst<=cp_dst+6'd1; cp_cnt<=cp_cnt-6'd1; state<=S_CP_CHK;
                end
                // Read one delta (1 or 2 bytes) into d, or into d0 when prefetching.
                S_DR: begin
                    if (!wide) begin
                        if (pf) d0<={{8{mem_rdata[7]}},mem_rdata}; else d<={{8{mem_rdata[7]}},mem_rdata};
                        bi<=2'd0; state<=S_DR_END;
                    end else if (bi==2'd0) begin
                        if (pf) d0[7:0]<=mem_rdata; else d[7:0]<=mem_rdata;
                        bi<=2'd1; mem_addr<=mem_addr+6'd1;
                    end else begin
                        if (pf) d0[15:8]<=mem_rdata; else d[15:8]<=mem_rdata;
                        bi<=2'd0; state<=S_DR_END;
                    end
                end
                // Before word 1 is written, delta 0 must be read because its bytes are overwritten.
                S_DR_END: begin
                    if (!pf && wi==2'd1) begin pf<=1'b1; mem_addr<=6'd4; bi<=2'd0; state<=S_DR; end
                    else state<=S_DCOMP;
                end
                // Rebuild one word (delta plus base) and write its four bytes.
                S_DCOMP: begin
                    cur<=alu_result; nb<=4'd4; mem_addr<={2'b00,wi,2'b00}; after<=A_DEC; state<=S_WRITE;
                end
                S_META_SEND: if (!tx_busy) begin
                    tx_start<=1'b1; gap<=2'd0; state<=S_META_GAP;
                end
                S_META_GAP: begin
                    gap<=gap+2'd1;
                    if (gap==2'd2) begin reply<=meta_length; state<=S_REPLY; end
                end
                // Any other state code returns to idle.
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
