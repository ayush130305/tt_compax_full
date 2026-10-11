## How it works

COMPAX is a UART-controlled 64-byte flip-flop memory with a lossless Base-Delta-Immediate
(BDI) codec and a block allocator. The allocator packs compressed 16-byte blocks into a
heap, so compressible data takes less physical space than it would uncompressed.

Physical map (64 bytes):

- Bytes 0..15: the scratch window. The host writes raw data here, and decompressed data appears here.
- Bytes 16..63: a packed 48-byte heap holding the compressed payloads back to back, with no gaps.

A table of 8 logical blocks (128 logical bytes) records, for each block, whether it is in use,
its format, immediate mask, heap offset and payload length. The codec works directly in the
flop memory, a byte per cycle, with no block buffer.

Commands at 115200 baud, 8N1, with a 25 MHz input clock. Numeric values are hexadecimal.

- W (0x57), addr, data: write one raw byte (addr 0..63); reply K (0x4B).
- R (0x52), addr: read one physical byte.
- P (0x50), addr, n, n data bytes: raw page write; reply K. The address wraps at 64.
- G (0x47), addr, n: read n physical bytes.
- C (0x43), blk: compress the scratch window and store it as block blk (0..7). Reply K.
  Reply E (0x45) if the block is already in use or the heap has no room. After E the scratch
  window still holds the original raw data.
- D (0x44), blk: decompress block blk into the scratch window; reply K, or E if the block is empty.
  The block stays stored.
- F (0x46), blk: free block blk and compact the heap; reply K, or E if the block is empty.
- M (0x4D), blk: return two bytes: metadata and payload length. Metadata bit 7 is the in-use flag,
  bits 6:3 are the immediate-selector bits and bits 2:0 are the format. An empty block returns 00, 00.
- Q (0x51): return two bytes: free heap bytes (0..48) and the in-use bitmap (bit i is block i).
- Unknown commands or block indices above 7: reply ? (0x3F).

Formats: 0 raw (16 bytes), 1 all zero (0 bytes), 2 repeated 32-bit word (4 bytes),
3 base32 plus four signed 8-bit deltas (8 bytes), 4 base32 plus four signed 16-bit deltas (12 bytes).
Selector bit i chooses zero as the base for word i. The base is the first word that does not fit as
an immediate, or word 0 if all fit. Arithmetic wraps modulo 2^32. The encoder picks the smallest
lossless format.

Capacity: a raw block costs 16 heap bytes, so incompressible data fits 3 blocks. Zero blocks cost
nothing, so all 8 blocks can be zero blocks. Mixed data fits in between, up to 128 logical bytes.

Notes: C leaves the scratch window holding the compressed payload (the original data is stored in
the heap). Raw commands access all 64 bytes, including the heap. Reset clears the block table
and heap pointer but not the memory contents. Wait for a command's full reply before sending the
next command; there is no receive FIFO or partial-command timeout.

## How to test

Connect a 3.3 V USB-serial adapter to ui[0] (RX) and uo[0] (TX), 115200 8N1, and supply a 25 MHz clock.

- Send Q; expect 30, 00 (48 bytes free, no blocks).
- Send P, 00, 10, then sixteen 00 bytes; expect K.
- Send C, 00; expect K. Send M, 00; expect 81, 00 (in use, zero format, 0 bytes).
- Send P, 00, 10, then four repeats of 78 56 34 12; expect K. Send C, 01; expect K.
- Send M, 01; expect 82, 04. Send Q; expect 2C, 03.
- Send P, 00, 10, sixteen FF bytes; send D, 01; expect K. Send G, 00, 10; expect 78 56 34 12 four times.
- Send F, 00; expect K. Send Q; expect 2C, 02.

## External hardware

A USB-to-UART adapter (3.3 V logic) is the only external hardware.
