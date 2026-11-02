# Package validation

Packaging check: 2 November 2026.

- Clean firmware build passed with ARM GCC 13.2.1 and CMake 4.4.4. Existing misleading-indentation compiler warnings remain; no firmware logic was changed for packaging.
- Rebuilt BIN matches the previously hardware-tested firmware byte for byte.
- Firmware BIN SHA256: `ea1075eb7f0f12f735df120dee12236b106e754ef67b6cc3acea71944a521ecf`.
- Native host processing library rebuilt successfully on x86_64.
- 46 dashboard regression tests and 8 firmware-tool regression tests passed.
- UNO Q snapshot Python sources match the main dashboard sources.

This packaging pass did not flash a board or repeat live sensor testing. Earlier live testing observed approximately 35–36 received FPS and exercised USB acquisition, pause/resume and reconnection. It does not establish absolute depth accuracy, maximum range or guaranteed performance on every assembled PCB. Verify a USB-only cold boot and known-distance targets on your own board.

SHA256SUMS covers the publication files. Rebuilding or editing files changes their checksums.
