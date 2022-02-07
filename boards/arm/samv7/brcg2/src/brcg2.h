/****************************************************************************
 * boards/arm/samv7/brcg2/src/brcg2.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_SAMV7_AN2C_SRC_AN2C_H
#define __BOARDS_ARM_SAMV7_AN2C_SRC_AN2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "sam_gpio.h"
#include "hardware/sam_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/*
#define HAVE_HSMCI           1
#define HAVE_AUTOMOUNTER     1
#define HAVE_USB             1
#define HAVE_USBDEV          1
#define HAVE_USBMONITOR      1
#define HAVE_NETWORK         1
#define HAVE_MACADDR         1
#define HAVE_MTDCONFIG       1
#define HAVE_PROGMEM_CHARDEV 1
#define HAVE_I2CTOOL         1
#define HAVE_MRF24J40        1
#define HAVE_XBEE            1*/

#define HAVE_HSMCI           1
#define HAVE_USB             1
#define HAVE_USBDEV          1
//#define HAVE_AUTOMOUNTER     1

/* HSMCI */

/* Can't support MMC/SD if the card interface is not enabled */

#if !defined(CONFIG_SAMV7_HSMCI0)
#  undef HAVE_HSMCI
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_HSMCI) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No MMC/SD support
#  undef HAVE_HSMCI
#endif

/* We need PIO interrupts on GPIOD to support card detect interrupts */

#if defined(HAVE_HSMCI) && !defined(CONFIG_SAMV7_GPIOA_IRQ)
#  warning PIOA interrupts not enabled.  No MMC/SD support.
#  undef HAVE_HSMCI
#endif

/* MMC/SD minor numbers */

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

#ifndef CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#endif

#if CONFIG_NSH_MMCSDSLOTNO != 0
#  error SAME70 has only one MMC/SD slot (CONFIG_NSH_MMCSDSLOTNO)
#  undef CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#endif

#define HSMCI0_SLOTNO CONFIG_NSH_MMCSDSLOTNO
#define HSMCI0_MINOR  CONFIG_NSH_MMCSDMINOR

/* Automounter.  Currently only works with HSMCI. */

#if !defined(CONFIG_FS_AUTOMOUNTER) || !defined(HAVE_HSMCI)
#  undef HAVE_AUTOMOUNTER
#  undef CONFIG_SAMV7_HSMCI0_AUTOMOUNT
#endif

#ifndef CONFIG_SAMV7_HSMCI0_AUTOMOUNT
#  undef HAVE_AUTOMOUNTER
#endif

/* USB Device */

/* CONFIG_USBDEV must be defined, or there is no USB
 * device.
 */

#if !defined(CONFIG_USBDEV)
#  undef HAVE_USB
#  undef HAVE_USBDEV
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

/* Networking and AT24-based MTD config */

#if !defined(CONFIG_NET) || !defined(CONFIG_SAMV7_EMAC)
#  undef HAVE_NETWORK
#  undef HAVE_MACADDR
#endif

#if !defined(CONFIG_SAMV7_TWIHS0) || !defined(CONFIG_MTD_AT24XX)
#  undef HAVE_MACADDR
#  undef HAVE_MTDCONFIG
#endif

#if defined(CONFIG_NSH_NOMAC) || !defined(CONFIG_AT24XX_EXTENDED)
#  undef HAVE_MACADDR
#endif

#if !defined(CONFIG_MTD_CONFIG)
#  undef HAVE_MTDCONFIG
#endif

/* On-chip Programming Memory */

#if !defined(CONFIG_SAMV7_PROGMEM) || !defined(CONFIG_MTD_PROGMEM)
#  undef HAVE_PROGMEM_CHARDEV
#endif

/* This is the on-chip progmem memory driver minor number */

#define PROGMEM_MTD_MINOR 0

/* Do we need to register I2C drivers on behalf of the I2C tool? */

#if !defined(CONFIG_SYSTEM_I2CTOOL) || !defined(CONFIG_I2C_DRIVER)
#  undef HAVE_I2CTOOL
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define SAME70_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define SAME70_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif


/* LEDs
 *
 * A single LED is available driven by PA24.
 */

#define GPIO_LED0     (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                       GPIO_PORT_PIOA | GPIO_PIN24)

/* Buttons
 * BRCg2 has one button GPIO_ENC_SW (software button on user encoder) on pin
 * PA_18
 */

#define GPIO_ENC_SW   (GPIO_INPUT | GPIO_CFG_DEFAULT | \
                       GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN18) /* PA_18 */

#define GPIO_ENC_SW_INT   SAM_IRQ_PA18

/* Encoder */

#define GPIO_ENC_A    (GPIO_INPUT | GPIO_CFG_DEFAULT | \
                       GPIO_INT_RISING | GPIO_PORT_PIOA | GPIO_PIN8)  /* PA_8 */
#define GPIO_ENC_B    (GPIO_INPUT | GPIO_CFG_DEFAULT | \
                       GPIO_INT_RISING | GPIO_PORT_PIOA | GPIO_PIN7)  /* PA_7 */

#define GPIO_ENC_A_INT    SAM_IRQ_PA8
#define GPIO_ENC_B_INT    SAM_IRQ_PA7

/* LCD dispay */

#define GPIO_LCD_RST   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                        GPIO_PORT_PIOD | GPIO_PIN26)    /* PD_26 */

#define GPIO_LCD_CD    (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                        GPIO_PORT_PIOD | GPIO_PIN27)    /* PD_27 */
#define GPIO_LCD_CS    (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                        GPIO_PORT_PIOD | GPIO_PIN25)    /* PD_25 */
#define GPIO_LCD_BL    (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                        GPIO_PORT_PIOD | GPIO_PIN19)    /* PD_19 */

/* USB device */

#define GPIO_VBUSON (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                     GPIO_PORT_PIOA | GPIO_PIN22)       /* PA_22 */

/* ADM2483 driver */

#define GPIO_ADM2483_EN (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                         GPIO_PORT_PIOD | GPIO_PIN18)       /* PD_18 */

/* ADDR0-7 pins */

#define BOARD_NGPIOIN     8         /* Amount of GPIO Input pins */

#define GPIO_ADDR0    (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOD | \
                       GPIO_PIN11)  /* PD_11 */
#define GPIO_ADDR1    (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOD | \
                       GPIO_PIN10)  /* PD_10 */
#define GPIO_ADDR2    (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOD | \
                       GPIO_PIN12)  /* PD_12 */
#define GPIO_ADDR3    (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOA | \
                       GPIO_PIN3)  /* PA_03 */
#define GPIO_ADDR4    (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOA | \
                       GPIO_PIN5)  /* PA_05 */
#define GPIO_ADDR5    (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOD | \
                       GPIO_PIN11)  /* PD_11 */
#define GPIO_ADDR6    (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOD | \
                       GPIO_PIN14)  /* PD_14 */
#define GPIO_ADDR7    (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOD | \
                       GPIO_PIN28)  /* PD_28 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sdram_config
 *
 * Description:
 *   Configures the on-board SDRAM.  SAME70 Xplained features one external
 *   IS42S16100E-7BLI, 512Kx16x2, 10ns, SDRAM. SDRAM0 is connected to chip
 *   select NCS1.
 *
 *  Input Parameters:
 *     None
 *
 *  Assumptions:
 *    The DDR memory regions is configured as strongly ordered memory.
 *    When we complete initialization of SDRAM and it is ready for use,
 *    we will make DRAM into normal memory.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_SDRAMC
void sam_sdram_config(void);
#else
#  define sam_sdram_config(t)
#endif

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int sam_bringup(void);
#endif

/****************************************************************************
 * Name:  sam_adm2483_enable
 *
 * Description:
 *   Called from sam_bringup(), enables adm2483 driver.
 *
 ****************************************************************************/

#ifdef CONFIG_BRCG2_ADM2483_USART
void sam_adm2483_enable(void);
#endif

/****************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAME70-XPLD board.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_SPI
void sam_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: sam_hsmci_initialize
 *
 * Description:
 *   Initialize HSMCI support
 *
 ****************************************************************************/

#ifdef HAVE_HSMCI
int sam_hsmci_initialize(int slot, int minor);
#else
# define sam_hsmci_initialize(s,m) (-ENOSYS)
#endif

/****************************************************************************
 * Name:  sam_usbinitialize
 *
 * Description:
 *   Called from stm32_boardinitialize very early in initialization to setup
 *   USB- related GPIO pins for the SAME70-XPLD board.
 *
 ****************************************************************************/

#ifdef HAVE_USB
void sam_usbinitialize(void);
#endif

/****************************************************************************
 * Name: sam_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 * Return Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int sam_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: sam_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_MCAN
int sam_can_setup(void);
#endif

/****************************************************************************
 * Name: sam_afec_initialize
 *
 * Description:
 *   Initialize and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_AFEC
int sam_afec_setup(void);
#endif

/****************************************************************************
 * Name: sam_gpio_enc_init
 *
 * Description:
 *   Initialize GPIO encoder
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_QENCODER
int sam_gpio_enc_init(void);
#endif

/****************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if the card in the MMCSD slot is write protected
 *
 ****************************************************************************/

#ifdef HAVE_HSMCI
bool sam_writeprotected(int slotno);
#else
#  define sam_writeprotected(slotno) (false)
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMV7_SAME70_XPLAINED_SRC_SAME70_XPLAINED_H */
