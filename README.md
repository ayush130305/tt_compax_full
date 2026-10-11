# COMPAX: 64 B compressed memory with allocator

UART-controlled 64-byte flop memory for Tiny Tapeout (SKY130, 3x2 tiles, 25 MHz) with a lossless
Base-Delta-Immediate (BDI) codec and a block allocator.

- 16-byte scratch window plus a packed 48-byte heap.
- 8 logical 16-byte blocks (128 logical bytes), stored compressed.
- Commands: W R P G (raw access), C D F (compress, decompress, free), M Q (metadata, status).
- Five BDI formats: raw, zero, repeated word, base32+delta8, base32+delta16.

See `docs/info.md` for the command reference. Tests: `test/test.py` (cocotb through the UART pins)
and `test/allocator_tb.sv` (randomized allocator test against a reference encoder).
