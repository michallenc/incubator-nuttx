OpenOCD connection:

program nuttx.bin 0x00400000
mww 0x400e0c04 0x5a00010b
reset