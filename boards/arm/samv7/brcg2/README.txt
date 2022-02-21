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
  - mmcsd0    - SD CARD (automatically mounted to /mnt/sdcard0)
  - smart0    - QSPI flash

GPIO_ENC

The encoder can be accessed by NuttX qencoder structure.

  - ioctl(fd, QEIOC_POSITION, (unsigned long)((uintptr_t)&position)) to get current position
  - ioctl(fd, QEIOC_RESET) to reset position to zero

QSPI FLASH

The support for QSPI flash is selected with CONFIG_BRCG2_FLASH=y. The flash is accessed via
Smart FS system, i.e dev/smart0.

Flash initialization:

nsh> mksmartfs dev/smart0  (only needed for the first time to create Smart FS)
nsh> mount -t smartfs dev/smart0 /mnt/location

Flash is mounted to /mnt/location after that. The flash can be erased by

nsh> flash_eraseall dev/smart0

NOTE: The board needs to be restarted before Smart FS can be initialized again on a formated flash.

Partition support can be enabled by CONFIG_BRCG2_FLASH_PART=y. FLASH then appears as

  - smart0p0
  - smart0p1 etc.

Configurations:
  nsh - basic configuration without any peripherals, just console
  brcg2 - full configuration with set peripherals (CAN, Encoder, LCD display, GPIOS ...)

OpenOCD connection (with st-link):

openocd -f boards/arm/samv7/brcg2/scripts/atmel_same70_stlink-v2

telnet localhost 4444

program nuttx.bin 0x00400000
mww 0x400e0c04 0x5a00010b - might be neccessary for the first time otherwise the code is not run from flash
reset
