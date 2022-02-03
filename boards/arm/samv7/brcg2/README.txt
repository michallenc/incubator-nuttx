Device names
  - dev/adc0  - AFEC0
  - dev/adc1  - AFEC1
  - buttons   - SW button and encoder button
  - can0      - MCAN0
  - fb0       - Framebuffer driver for LCD display
  - gpio_enc  - Position of the encoder

GPIO_ENC

The encoder can be accessed by NuttX qencoder structure.

  - ioctl(fd, QEIOC_POSITION, (unsigned long)((uintptr_t)&position)) to get current position
  - ioctl(fd, QEIOC_RESET) to reset position to zero

Configurations:
  nsh - basic configuration without any peripherals, just console
  an2c - full configuration with set peripherals (ADC, CAN, Encoder, LCD display...)

OpenOCD connection:

program nuttx.bin 0x00400000
mww 0x400e0c04 0x5a00010b - might be neccessary for the first time
reset
