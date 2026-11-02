# I3C status — 29 October 2026

I3C is working and tested on this board: 400 kHz discovery, 1 MHz ENTDAA,
12 MHz SDR reads, dynamic address 0x30, 1,024-byte read blocks. The 3,000-frame run
reported zero transaction failures. Recovery now attempts I3C again before fallback.

See [the full report](docs/PERFORMANCE_2026-10-29.txt) and
[setup instructions](START_HERE.txt). Earlier investigation notes are archived in
[the historical notes](docs/history/I3C_NOTES-2026-10-28.md).
