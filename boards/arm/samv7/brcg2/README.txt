Device names
  - ttyS0     - debug serial
  - ttyS1     - safety MCU serial
  - ttyS2     - to RS485
  - ttyACM0   - USB serial (needs to be connected first - see commands sercon and serdis)
  - buttons   - Encoder button
  - can0      - MCAN0
  - fb0       - Framebuffer driver for LCD display
  - gpio_enc  - Position of the encoder
  - gpio0-7   - input gpio pins for ADDR0-7
  - gpio9     - output gpio pin for WARN (to safety MCU)

GPIO_ENC

The encoder can be accessed by NuttX qencoder structure.

  - ioctl(fd, QEIOC_POSITION, (unsigned long)((uintptr_t)&position)) to get current position
  - ioctl(fd, QEIOC_RESET) to reset position to zero

Configurations:
  nsh - basic configuration without any peripherals, just console
  brcg2 - full configuration with set peripherals (CAN, Encoder, LCD display, GPIOS ...)

OpenOCD connection (with st-link):

openocd -f boards/arm/samv7/brcg2/scripts/atmel_same70_stlink-v2

telnet localhost 4444

program nuttx.bin 0x00400000
mww 0x400e0c04 0x5a00010b - might be neccessary for the first time otherwise the code is not run from flash
reset
