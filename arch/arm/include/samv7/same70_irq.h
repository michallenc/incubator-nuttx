/****************************************************************************
 * arch/arm/include/samv7/same70_irq.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_SAMV7_SAME70_IRQ_H
#define __ARCH_ARM_INCLUDE_SAMV7_SAME70_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* SAMV71 Peripheral Identifiers */

#define SAM_PID_SUPC           (0)  /* Supply Controller */
#define SAM_PID_RSTC           (1)  /* Reset Controller */
#define SAM_PID_RTC            (2)  /* Real Time Clock */
#define SAM_PID_RTT            (3)  /* Real Time Timer */
#define SAM_PID_WDT            (4)  /* Watchdog Timer */
#define SAM_PID_PMC            (5)  /* Power Management Controller */
#define SAM_PID_EFC            (6)  /* Embedded Flash Controller */
#define SAM_PID_UART0          (7)  /* Universal Asynchronous Receiver Transmitter 0 */
#define SAM_PID_UART1          (8)  /* Universal Asynchronous Receiver Transmitter 1 */
#define SAM_PID_SMC            (9)  /* Static Memory Controller */
#define SAM_PID_PIOA          (10)  /* Parallel I/O Controller A */
#define SAM_PID_PIOB          (11)  /* Parallel I/O Controller B */
#define SAM_PID_PIOC          (12)  /* Parallel I/O Controller C */
#define SAM_PID_USART0        (13)  /* USART 0 */
#define SAM_PID_USART1        (14)  /* USART 1 */
#define SAM_PID_USART2        (15)  /* USART 2 */
#define SAM_PID_PIOD          (16)  /* Parallel I/O Controller D */
#define SAM_PID_PIOE          (17)  /* Parallel I/O Controller E */
#define SAM_PID_HSMCI0        (18)  /* High Speed Multimedia Card Interface */
#define SAM_PID_TWIHS0        (19)  /* Two-Wire Interface 0 */
#define SAM_PID_TWIHS1        (20)  /* Two-Wire Interface 1 */
#define SAM_PID_SPI0          (21)  /* Serial Peripheral Interface 0 */
#define SAM_PID_SSC0          (22)  /* Synchronous Serial Controller */
#define SAM_PID_TC0           (23)  /* Timer Counter 0 */
#define SAM_PID_TC1           (24)  /* Timer Counter 1 */
#define SAM_PID_TC2           (25)  /* Timer Counter 2 */
#define SAM_PID_TC3           (26)  /* Timer Counter 3 */
#define SAM_PID_TC4           (27)  /* Timer Counter 4 */
#define SAM_PID_TC5           (28)  /* Timer Counter 5 */
#define SAM_PID_AFEC0         (29)  /* Analog Front End 0 */
#define SAM_PID_DACC          (30)  /* Digital to Analog Converter */
#define SAM_PID_PWM0          (31)  /* Pulse Width Modulation Controller 0 */
#define SAM_PID_ICM           (32)  /* Integrity Check Monitor */
#define SAM_PID_ACC           (33)  /* Analog Comparator */
#define SAM_PID_USBHS         (34)  /* USB Host / Device Controller */
#define SAM_PID_MCAN00        (35)  /* CAN0 IRQ line 0 */
#define SAM_PID_MCAN01        (36)  /* CAN0 IRQ line 1 */
#define SAM_PID_MCAN10        (37)  /* CAN1 IRQ line 0 */
#define SAM_PID_MCAN11        (38)  /* CAN1 IRQ line 1 */
#define SAM_PID_EMAC0         (39)  /* Ethernet MAC */
#define SAM_PID_AFEC1         (40)  /* Analog Front End 1 */
#define SAM_PID_TWIHS2        (41)  /* Two-Wire Interface 2 */
#define SAM_PID_SPI1          (42)  /* Serial Peripheral Interface 1 */
#define SAM_PID_QSPI          (43)  /* Quad I/O Serial Peripheral Interface */
#define SAM_PID_UART2         (44)  /* Universal Asynchronous Receiver Transmitter 2 */
#define SAM_PID_UART3         (45)  /* Universal Asynchronous Receiver Transmitter 3 */
#define SAM_PID_UART4         (46)  /* Universal Asynchronous Receiver Transmitter 4 */
#define SAM_PID_TC6           (47)  /* Timer Counter 6 */
#define SAM_PID_TC7           (48)  /* Timer Counter 7 */
#define SAM_PID_TC8           (49)  /* Timer Counter 8 */
#define SAM_PID_TC9           (50)  /* Timer Counter 9 */
#define SAM_PID_TC10          (51)  /* Timer Counter 10 */
#define SAM_PID_TC11          (52)  /* Timer Counter 11 */
#define SAM_PID_RESERVED53    (53)  /* Reserved */
#define SAM_PID_RESERVED54    (54)  /* Reserved */
#define SAM_PID_RESERVED55    (55)  /* Reserved */
#define SAM_PID_AES           (56)  /* Advanced Encryption Standard */
#define SAM_PID_TRNG          (57)  /* True Random Number Generator */
#define SAM_PID_XDMAC         (58)  /* Central DMA Controller */
#define SAM_PID_ISI           (59)  /* Image Sensor Interface */
#define SAM_PID_PWM1          (60)  /* Pulse Width Modulation Controller 1 */
#define SAM_PID_FPU           (61)  /* ARM Floating Point Unit interrupt */
#define SAM_PID_SDRAMC        (62)  /* SDRAM Controller */
#define SAM_PID_RSWDT         (63)  /* Reinforced Safety Watchdog Timer */
#define SAM_PID_CCW           (64)  /* ARM Cache ECC Warning */
#define SAM_PID_CCF           (65)  /* ARM Cache ECC Fault */
#define SAM_PID_EMACQ1        (66)  /* EMAC Queue 1 Interrupt */
#define SAM_PID_EMACQ2        (67)  /* EMAC Queue 2 Interrupt */
#define SAM_PID_FPIXC         (68)  /* ARM Cache ECC Warning */
#define SAM_PID_I2SC0         (69)  /* Inter-IC Sound Controller 0 */
#define SAM_PID_I2SC1         (70)  /* Inter-IC Sound Controller 1 */
#define SAM_PID_EMACQ3        (71)  /* EMAC Queue 3 Interrupt */
#define SAM_PID_EMACQ4        (72)  /* EMAC Queue 4 Interrupt */
#define SAM_PID_EMACQ5        (73)  /* EMAC Queue 5 Interrupt */

#define NR_PIDS               (74)  /* Number of peripheral identifiers */

/* External interrupts (priority levels >= 256 */

#define SAM_IRQ_SUPC          (SAM_IRQ_EXTINT+SAM_PID_SUPC)   /* Supply Controller */
#define SAM_IRQ_RSTC          (SAM_IRQ_EXTINT+SAM_PID_RSTC)   /* Reset Controller */
#define SAM_IRQ_RTC           (SAM_IRQ_EXTINT+SAM_PID_RTC)    /* Real Time Clock */
#define SAM_IRQ_RTT           (SAM_IRQ_EXTINT+SAM_PID_RTT)    /* Real Time Timer */
#define SAM_IRQ_WDT           (SAM_IRQ_EXTINT+SAM_PID_WDT)    /* Watchdog Timer */
#define SAM_IRQ_PMC           (SAM_IRQ_EXTINT+SAM_PID_PMC)    /* Power Management Controller */
#define SAM_IRQ_EEFC0         (SAM_IRQ_EXTINT+SAM_PID_EFC)    /* Embedded Flash Controller */
#define SAM_IRQ_UART0         (SAM_IRQ_EXTINT+SAM_PID_UART0)  /* Universal Asynchronous Receiver Transmitter 0 */
#define SAM_IRQ_UART1         (SAM_IRQ_EXTINT+SAM_PID_UART1)  /* Universal Asynchronous Receiver Transmitter 1 */
#define SAM_IRQ_SMC           (SAM_IRQ_EXTINT+SAM_PID_SMC)    /* Static Memory Controller */
#define SAM_IRQ_PIOA          (SAM_IRQ_EXTINT+SAM_PID_PIOA)   /* Parallel I/O Controller A */
#define SAM_IRQ_PIOB          (SAM_IRQ_EXTINT+SAM_PID_PIOB)   /* Parallel I/O Controller B */
#define SAM_IRQ_PIOC          (SAM_IRQ_EXTINT+SAM_PID_PIOC)   /* Parallel I/O Controller C */
#define SAM_IRQ_USART0        (SAM_IRQ_EXTINT+SAM_PID_USART0) /* USART 0 */
#define SAM_IRQ_USART1        (SAM_IRQ_EXTINT+SAM_PID_USART1) /* USART 1 */
#define SAM_IRQ_USART2        (SAM_IRQ_EXTINT+SAM_PID_USART2) /* USART 2 */
#define SAM_IRQ_PIOD          (SAM_IRQ_EXTINT+SAM_PID_PIOD)   /* Parallel I/O Controller D */
#define SAM_IRQ_PIOE          (SAM_IRQ_EXTINT+SAM_PID_PIOE)   /* Parallel I/O Controller E */
#define SAM_IRQ_HSMCI0        (SAM_IRQ_EXTINT+SAM_PID_HSMCI0) /* High Speed Multimedia Card Interface */
#define SAM_IRQ_TWIHS0        (SAM_IRQ_EXTINT+SAM_PID_TWIHS0) /* Two-Wire Interface 0 */
#define SAM_IRQ_TWIHS1        (SAM_IRQ_EXTINT+SAM_PID_TWIHS1) /* Two-Wire Interface 1 */
#define SAM_IRQ_SPI0          (SAM_IRQ_EXTINT+SAM_PID_SPI0)   /* Serial Peripheral Interface 0 */
#define SAM_IRQ_SSC0          (SAM_IRQ_EXTINT+SAM_PID_SSC0)   /* Synchronous Serial Controller */
#define SAM_IRQ_TC0           (SAM_IRQ_EXTINT+SAM_PID_TC0)    /* Timer Counter 0 */
#define SAM_IRQ_TC1           (SAM_IRQ_EXTINT+SAM_PID_TC1)    /* Timer Counter 1 */
#define SAM_IRQ_TC2           (SAM_IRQ_EXTINT+SAM_PID_TC2)    /* Timer Counter 2 */
#define SAM_IRQ_TC3           (SAM_IRQ_EXTINT+SAM_PID_TC3)    /* Timer Counter 3 */
#define SAM_IRQ_TC4           (SAM_IRQ_EXTINT+SAM_PID_TC4)    /* Timer Counter 4 */
#define SAM_IRQ_TC5           (SAM_IRQ_EXTINT+SAM_PID_TC5)    /* Timer Counter 5 */
#define SAM_IRQ_AFEC0         (SAM_IRQ_EXTINT+SAM_PID_AFEC0)  /* Analog Front End 0 */
#define SAM_IRQ_DACC          (SAM_IRQ_EXTINT+SAM_PID_DACC)   /* Digital to Analog Converter */
#define SAM_IRQ_PWM0          (SAM_IRQ_EXTINT+SAM_PID_PWM0)   /* Pulse Width Modulation Controller 0 */
#define SAM_IRQ_ICM           (SAM_IRQ_EXTINT+SAM_PID_ICM)    /* Integrity Check Monitor */
#define SAM_IRQ_ACC           (SAM_IRQ_EXTINT+SAM_PID_ACC)    /* Analog Comparator */
#define SAM_IRQ_USBHS         (SAM_IRQ_EXTINT+SAM_PID_USBHS)  /* USB Host / Device Controller */
#define SAM_IRQ_MCAN00        (SAM_IRQ_EXTINT+SAM_PID_MCAN00) /* CAN0 IRQ line 0 */
#define SAM_IRQ_MCAN01        (SAM_IRQ_EXTINT+SAM_PID_MCAN01) /* CAN0 IRQ line 1 */
#define SAM_IRQ_MCAN10        (SAM_IRQ_EXTINT+SAM_PID_MCAN10) /* CAN1 IRQ line 0 */
#define SAM_IRQ_MCAN11        (SAM_IRQ_EXTINT+SAM_PID_MCAN11) /* CAN1 IRQ line 1 */
#define SAM_IRQ_EMAC0         (SAM_IRQ_EXTINT+SAM_PID_EMAC0)  /* Ethernet MAC */
#define SAM_IRQ_AFEC1         (SAM_IRQ_EXTINT+SAM_PID_AFEC1)  /* Analog Front End 1 */
#define SAM_IRQ_TWIHS2        (SAM_IRQ_EXTINT+SAM_PID_TWIHS2) /* Two-Wire Interface 2 */
#define SAM_IRQ_SPI1          (SAM_IRQ_EXTINT+SAM_PID_SPI1)   /* Serial Peripheral Interface 1 */
#define SAM_IRQ_QSPI          (SAM_IRQ_EXTINT+SAM_PID_QSPI)   /* Quad I/O Serial Peripheral Interface */
#define SAM_IRQ_UART2         (SAM_IRQ_EXTINT+SAM_PID_UART2)  /* Universal Asynchronous Receiver Transmitter 2 */
#define SAM_IRQ_UART3         (SAM_IRQ_EXTINT+SAM_PID_UART3)  /* Universal Asynchronous Receiver Transmitter 3 */
#define SAM_IRQ_UART4         (SAM_IRQ_EXTINT+SAM_PID_UART4)  /* Universal Asynchronous Receiver Transmitter 4 */
#define SAM_IRQ_TC6           (SAM_IRQ_EXTINT+SAM_PID_TC6)    /* Timer Counter 6 */
#define SAM_IRQ_TC7           (SAM_IRQ_EXTINT+SAM_PID_TC7)    /* Timer Counter 7 */
#define SAM_IRQ_TC8           (SAM_IRQ_EXTINT+SAM_PID_TC8)    /* Timer Counter 8 */
#define SAM_IRQ_TC9           (SAM_IRQ_EXTINT+SAM_PID_TC9)    /* Timer Counter 9 */
#define SAM_IRQ_TC10          (SAM_IRQ_EXTINT+SAM_PID_TC10)   /* Timer Counter 10 */
#define SAM_IRQ_TC11          (SAM_IRQ_EXTINT+SAM_PID_TC11)   /* Timer Counter 11 */

#define SAM_IRQ_RESERVED53    (SAM_IRQ_EXTINT+SAM_PID_RESERVED53) /* Reserved */
#define SAM_IRQ_RESERVED54    (SAM_IRQ_EXTINT+SAM_PID_RESERVED54) /* Reserved */
#define SAM_IRQ_RESERVED55    (SAM_IRQ_EXTINT+SAM_PID_RESERVED55) /* Reserved */

#define SAM_IRQ_AES           (SAM_IRQ_EXTINT+SAM_PID_AES)    /* AES */
#define SAM_IRQ_TRNG          (SAM_IRQ_EXTINT+SAM_PID_TRNG)   /* True Random Number Generator */
#define SAM_IRQ_XDMAC         (SAM_IRQ_EXTINT+SAM_PID_XDMAC)  /* Central DMA Controller */
#define SAM_IRQ_ISI           (SAM_IRQ_EXTINT+SAM_PID_ISI)    /* Image Sensor Interface */
#define SAM_IRQ_PWM1          (SAM_IRQ_EXTINT+SAM_PID_PWM1)   /* Pulse Width Modulation Controller 1 */
#define SAM_IRQ_FPU           (SAM_IRQ_EXTINT+SAM_PID_FPU)    /* ARM Floating Point Unit interrupt */
#define SAM_IRQ_SDRAMC        (SAM_IRQ_EXTINT+SAM_PID_SDRAMC) /* SDRAM Controller */
#define SAM_IRQ_RSWDT         (SAM_IRQ_EXTINT+SAM_PID_RSWDT)  /* Reinforced Safety Watchdog Timer */
#define SAM_IRQ_CCW           (SAM_IRQ_EXTINT+SAM_PID_CCW)    /* ARM Cache ECC Warning */
#define SAM_IRQ_CCF           (SAM_IRQ_EXTINT+SAM_PID_CCF)    /* ARM Cache ECC Fault */
#define SAM_IRQ_EMACQ1        (SAM_IRQ_EXTINT+SAM_PID_EMACQ1) /* EMAC Queue 1 Interrupt */
#define SAM_IRQ_EMACQ2        (SAM_IRQ_EXTINT+SAM_PID_EMACQ2) /* EMAC Queue 2 Interrupt */
#define SAM_IRQ_FPIXC         (SAM_IRQ_EXTINT+SAM_PID_FPIXC)  /* ARM Cache ECC Warning */
#define SAM_IRQ_I2SC0         (SAM_IRQ_EXTINT+SAM_PID_I2SC0)  /* Inter-IC Sound Controller 0 */
#define SAM_IRQ_I2SC1         (SAM_IRQ_EXTINT+SAM_PID_I2SC1)  /* Inter-IC Sound Controller 1 */
#define SAM_IRQ_EMACQ3        (SAM_IRQ_EXTINT+SAM_PID_EMACQ3) /* EMAC Queue 3 Interrupt */
#define SAM_IRQ_EMACQ4        (SAM_IRQ_EXTINT+SAM_PID_EMACQ4) /* EMAC Queue 4 Interrupt */
#define SAM_IRQ_EMACQ5        (SAM_IRQ_EXTINT+SAM_PID_EMACQ5) /* EMAC Queue 5 Interrupt */

#define SAM_IRQ_NEXTINT       NR_PIDS                         /* Total number of external interrupt numbers */
#define SAM_IRQ_NIRQS         (SAM_IRQ_EXTINT+NR_PIDS)        /* The number of real IRQs */

/* GPIO interrupts (derived from SAM_IRQ_PIOA..E) */

#ifdef CONFIG_SAMV7_GPIOA_IRQ
#  define SAM_IRQ_GPIOA_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT)
#  define SAM_IRQ_PA0         (SAM_IRQ_GPIOA_PINS+0)          /* GPIOA, PIN 0 */
#  define SAM_IRQ_PA1         (SAM_IRQ_GPIOA_PINS+1)          /* GPIOA, PIN 1 */
#  define SAM_IRQ_PA2         (SAM_IRQ_GPIOA_PINS+2)          /* GPIOA, PIN 2 */
#  define SAM_IRQ_PA3         (SAM_IRQ_GPIOA_PINS+3)          /* GPIOA, PIN 3 */
#  define SAM_IRQ_PA4         (SAM_IRQ_GPIOA_PINS+4)          /* GPIOA, PIN 4 */
#  define SAM_IRQ_PA5         (SAM_IRQ_GPIOA_PINS+5)          /* GPIOA, PIN 5 */
#  define SAM_IRQ_PA6         (SAM_IRQ_GPIOA_PINS+6)          /* GPIOA, PIN 6 */
#  define SAM_IRQ_PA7         (SAM_IRQ_GPIOA_PINS+7)          /* GPIOA, PIN 7 */
#  define SAM_IRQ_PA8         (SAM_IRQ_GPIOA_PINS+8)          /* GPIOA, PIN 8 */
#  define SAM_IRQ_PA9         (SAM_IRQ_GPIOA_PINS+9)          /* GPIOA, PIN 9 */
#  define SAM_IRQ_PA10        (SAM_IRQ_GPIOA_PINS+10)         /* GPIOA, PIN 10 */
#  define SAM_IRQ_PA11        (SAM_IRQ_GPIOA_PINS+11)         /* GPIOA, PIN 11 */
#  define SAM_IRQ_PA12        (SAM_IRQ_GPIOA_PINS+12)         /* GPIOA, PIN 12 */
#  define SAM_IRQ_PA13        (SAM_IRQ_GPIOA_PINS+13)         /* GPIOA, PIN 13 */
#  define SAM_IRQ_PA14        (SAM_IRQ_GPIOA_PINS+14)         /* GPIOA, PIN 14 */
#  define SAM_IRQ_PA15        (SAM_IRQ_GPIOA_PINS+15)         /* GPIOA, PIN 15 */
#  define SAM_IRQ_PA16        (SAM_IRQ_GPIOA_PINS+16)         /* GPIOA, PIN 16 */
#  define SAM_IRQ_PA17        (SAM_IRQ_GPIOA_PINS+17)         /* GPIOA, PIN 17 */
#  define SAM_IRQ_PA18        (SAM_IRQ_GPIOA_PINS+18)         /* GPIOA, PIN 18 */
#  define SAM_IRQ_PA19        (SAM_IRQ_GPIOA_PINS+19)         /* GPIOA, PIN 19 */
#  define SAM_IRQ_PA20        (SAM_IRQ_GPIOA_PINS+20)         /* GPIOA, PIN 20 */
#  define SAM_IRQ_PA21        (SAM_IRQ_GPIOA_PINS+21)         /* GPIOA, PIN 21 */
#  define SAM_IRQ_PA22        (SAM_IRQ_GPIOA_PINS+22)         /* GPIOA, PIN 22 */
#  define SAM_IRQ_PA23        (SAM_IRQ_GPIOA_PINS+23)         /* GPIOA, PIN 23 */
#  define SAM_IRQ_PA24        (SAM_IRQ_GPIOA_PINS+24)         /* GPIOA, PIN 24 */
#  define SAM_IRQ_PA25        (SAM_IRQ_GPIOA_PINS+25)         /* GPIOA, PIN 25 */
#  define SAM_IRQ_PA26        (SAM_IRQ_GPIOA_PINS+26)         /* GPIOA, PIN 26 */
#  define SAM_IRQ_PA27        (SAM_IRQ_GPIOA_PINS+27)         /* GPIOA, PIN 27 */
#  define SAM_IRQ_PA28        (SAM_IRQ_GPIOA_PINS+28)         /* GPIOA, PIN 28 */
#  define SAM_IRQ_PA29        (SAM_IRQ_GPIOA_PINS+29)         /* GPIOA, PIN 29 */
#  define SAM_IRQ_PA30        (SAM_IRQ_GPIOA_PINS+30)         /* GPIOA, PIN 30 */
#  define SAM_IRQ_PA31        (SAM_IRQ_GPIOA_PINS+31)         /* GPIOA, PIN 31 */
#  define SAM_NGPIOAIRQS      32
#else
#  define SAM_NGPIOAIRQS      0
#endif

#ifdef CONFIG_SAMV7_GPIOB_IRQ
#  define SAM_IRQ_GPIOB_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + SAM_NGPIOAIRQS)
#  define SAM_IRQ_PB0         (SAM_IRQ_GPIOB_PINS+0)          /* GPIOB, PIN 0 */
#  define SAM_IRQ_PB1         (SAM_IRQ_GPIOB_PINS+1)          /* GPIOB, PIN 1 */
#  define SAM_IRQ_PB2         (SAM_IRQ_GPIOB_PINS+2)          /* GPIOB, PIN 2 */
#  define SAM_IRQ_PB3         (SAM_IRQ_GPIOB_PINS+3)          /* GPIOB, PIN 3 */
#  define SAM_IRQ_PB4         (SAM_IRQ_GPIOB_PINS+4)          /* GPIOB, PIN 4 */
#  define SAM_IRQ_PB5         (SAM_IRQ_GPIOB_PINS+5)          /* GPIOB, PIN 5 */
#  define SAM_IRQ_PB6         (SAM_IRQ_GPIOB_PINS+6)          /* GPIOB, PIN 6 */
#  define SAM_IRQ_PB7         (SAM_IRQ_GPIOB_PINS+7)          /* GPIOB, PIN 7 */
#  define SAM_IRQ_PB8         (SAM_IRQ_GPIOB_PINS+8)          /* GPIOB, PIN 8 */
#  define SAM_IRQ_PB9         (SAM_IRQ_GPIOB_PINS+9)          /* GPIOB, PIN 9 */
#  define SAM_IRQ_PB10        (SAM_IRQ_GPIOB_PINS+10)         /* GPIOB, PIN 10 */
#  define SAM_IRQ_PB11        (SAM_IRQ_GPIOB_PINS+11)         /* GPIOB, PIN 11 */
#  define SAM_IRQ_PB12        (SAM_IRQ_GPIOB_PINS+12)         /* GPIOB, PIN 12 */
#  define SAM_IRQ_PB13        (SAM_IRQ_GPIOB_PINS+13)         /* GPIOB, PIN 13 */
#  define SAM_IRQ_PB14        (SAM_IRQ_GPIOB_PINS+14)         /* GPIOB, PIN 14 */
#  define SAM_IRQ_PB15        (SAM_IRQ_GPIOB_PINS+15)         /* GPIOB, PIN 15 */
#  define SAM_IRQ_PB16        (SAM_IRQ_GPIOB_PINS+16)         /* GPIOB, PIN 16 */
#  define SAM_IRQ_PB17        (SAM_IRQ_GPIOB_PINS+17)         /* GPIOB, PIN 17 */
#  define SAM_IRQ_PB18        (SAM_IRQ_GPIOB_PINS+18)         /* GPIOB, PIN 18 */
#  define SAM_IRQ_PB19        (SAM_IRQ_GPIOB_PINS+19)         /* GPIOB, PIN 19 */
#  define SAM_IRQ_PB20        (SAM_IRQ_GPIOB_PINS+20)         /* GPIOB, PIN 20 */
#  define SAM_IRQ_PB21        (SAM_IRQ_GPIOB_PINS+21)         /* GPIOB, PIN 21 */
#  define SAM_IRQ_PB22        (SAM_IRQ_GPIOB_PINS+22)         /* GPIOB, PIN 22 */
#  define SAM_IRQ_PB23        (SAM_IRQ_GPIOB_PINS+23)         /* GPIOB, PIN 23 */
#  define SAM_IRQ_PB24        (SAM_IRQ_GPIOB_PINS+24)         /* GPIOB, PIN 24 */
#  define SAM_IRQ_PB25        (SAM_IRQ_GPIOB_PINS+25)         /* GPIOB, PIN 25 */
#  define SAM_IRQ_PB26        (SAM_IRQ_GPIOB_PINS+26)         /* GPIOB, PIN 26 */
#  define SAM_IRQ_PB27        (SAM_IRQ_GPIOB_PINS+27)         /* GPIOB, PIN 27 */
#  define SAM_IRQ_PB28        (SAM_IRQ_GPIOB_PINS+28)         /* GPIOB, PIN 28 */
#  define SAM_IRQ_PB29        (SAM_IRQ_GPIOB_PINS+29)         /* GPIOB, PIN 29 */
#  define SAM_IRQ_PB30        (SAM_IRQ_GPIOB_PINS+30)         /* GPIOB, PIN 30 */
#  define SAM_IRQ_PB31        (SAM_IRQ_GPIOB_PINS+31)         /* GPIOB, PIN 31 */
#  define SAM_NGPIOBIRQS      32
#else
#  define SAM_NGPIOBIRQS      0
#endif

#ifdef CONFIG_SAMV7_GPIOC_IRQ
#  define SAM_IRQ_GPIOC_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + SAM_NGPIOAIRQS + \
                               SAM_NGPIOBIRQS)
#  define SAM_IRQ_PC0         (SAM_IRQ_GPIOC_PINS+0)          /* GPIOC, PIN 0 */
#  define SAM_IRQ_PC1         (SAM_IRQ_GPIOC_PINS+1)          /* GPIOC, PIN 1 */
#  define SAM_IRQ_PC2         (SAM_IRQ_GPIOC_PINS+2)          /* GPIOC, PIN 2 */
#  define SAM_IRQ_PC3         (SAM_IRQ_GPIOC_PINS+3)          /* GPIOC, PIN 3 */
#  define SAM_IRQ_PC4         (SAM_IRQ_GPIOC_PINS+4)          /* GPIOC, PIN 4 */
#  define SAM_IRQ_PC5         (SAM_IRQ_GPIOC_PINS+5)          /* GPIOC, PIN 5 */
#  define SAM_IRQ_PC6         (SAM_IRQ_GPIOC_PINS+6)          /* GPIOC, PIN 6 */
#  define SAM_IRQ_PC7         (SAM_IRQ_GPIOC_PINS+7)          /* GPIOC, PIN 7 */
#  define SAM_IRQ_PC8         (SAM_IRQ_GPIOC_PINS+8)          /* GPIOC, PIN 8 */
#  define SAM_IRQ_PC9         (SAM_IRQ_GPIOC_PINS+9)          /* GPIOC, PIN 9 */
#  define SAM_IRQ_PC10        (SAM_IRQ_GPIOC_PINS+10)         /* GPIOC, PIN 10 */
#  define SAM_IRQ_PC11        (SAM_IRQ_GPIOC_PINS+11)         /* GPIOC, PIN 11 */
#  define SAM_IRQ_PC12        (SAM_IRQ_GPIOC_PINS+12)         /* GPIOC, PIN 12 */
#  define SAM_IRQ_PC13        (SAM_IRQ_GPIOC_PINS+13)         /* GPIOC, PIN 13 */
#  define SAM_IRQ_PC14        (SAM_IRQ_GPIOC_PINS+14)         /* GPIOC, PIN 14 */
#  define SAM_IRQ_PC15        (SAM_IRQ_GPIOC_PINS+15)         /* GPIOC, PIN 15 */
#  define SAM_IRQ_PC16        (SAM_IRQ_GPIOC_PINS+16)         /* GPIOC, PIN 16 */
#  define SAM_IRQ_PC17        (SAM_IRQ_GPIOC_PINS+17)         /* GPIOC, PIN 17 */
#  define SAM_IRQ_PC18        (SAM_IRQ_GPIOC_PINS+18)         /* GPIOC, PIN 18 */
#  define SAM_IRQ_PC19        (SAM_IRQ_GPIOC_PINS+19)         /* GPIOC, PIN 19 */
#  define SAM_IRQ_PC20        (SAM_IRQ_GPIOC_PINS+20)         /* GPIOC, PIN 20 */
#  define SAM_IRQ_PC21        (SAM_IRQ_GPIOC_PINS+21)         /* GPIOC, PIN 21 */
#  define SAM_IRQ_PC22        (SAM_IRQ_GPIOC_PINS+22)         /* GPIOC, PIN 22 */
#  define SAM_IRQ_PC23        (SAM_IRQ_GPIOC_PINS+23)         /* GPIOC, PIN 23 */
#  define SAM_IRQ_PC24        (SAM_IRQ_GPIOC_PINS+24)         /* GPIOC, PIN 24 */
#  define SAM_IRQ_PC25        (SAM_IRQ_GPIOC_PINS+25)         /* GPIOC, PIN 25 */
#  define SAM_IRQ_PC26        (SAM_IRQ_GPIOC_PINS+26)         /* GPIOC, PIN 26 */
#  define SAM_IRQ_PC27        (SAM_IRQ_GPIOC_PINS+27)         /* GPIOC, PIN 27 */
#  define SAM_IRQ_PC28        (SAM_IRQ_GPIOC_PINS+28)         /* GPIOC, PIN 28 */
#  define SAM_IRQ_PC29        (SAM_IRQ_GPIOC_PINS+29)         /* GPIOC, PIN 29 */
#  define SAM_IRQ_PC30        (SAM_IRQ_GPIOC_PINS+30)         /* GPIOC, PIN 30 */
#  define SAM_IRQ_PC31        (SAM_IRQ_GPIOC_PINS+31)         /* GPIOC, PIN 31 */
#  define SAM_NGPIOCIRQS      32
#else
#  define SAM_NGPIOCIRQS      0
#endif

#ifdef CONFIG_SAMV7_GPIOD_IRQ
#  define SAM_IRQ_GPIOD_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + SAM_NGPIOAIRQS + \
                               SAM_NGPIOBIRQS + SAM_NGPIOCIRQS)
#  define SAM_IRQ_PD0         (SAM_IRQ_GPIOD_PINS+0)          /* GPIOD, PIN 0 */
#  define SAM_IRQ_PD1         (SAM_IRQ_GPIOD_PINS+1)          /* GPIOD, PIN 1 */
#  define SAM_IRQ_PD2         (SAM_IRQ_GPIOD_PINS+2)          /* GPIOD, PIN 2 */
#  define SAM_IRQ_PD3         (SAM_IRQ_GPIOD_PINS+3)          /* GPIOD, PIN 3 */
#  define SAM_IRQ_PD4         (SAM_IRQ_GPIOD_PINS+4)          /* GPIOD, PIN 4 */
#  define SAM_IRQ_PD5         (SAM_IRQ_GPIOD_PINS+5)          /* GPIOD, PIN 5 */
#  define SAM_IRQ_PD6         (SAM_IRQ_GPIOD_PINS+6)          /* GPIOD, PIN 6 */
#  define SAM_IRQ_PD7         (SAM_IRQ_GPIOD_PINS+7)          /* GPIOD, PIN 7 */
#  define SAM_IRQ_PD8         (SAM_IRQ_GPIOD_PINS+8)          /* GPIOD, PIN 8 */
#  define SAM_IRQ_PD9         (SAM_IRQ_GPIOD_PINS+9)          /* GPIOD, PIN 9 */
#  define SAM_IRQ_PD10        (SAM_IRQ_GPIOD_PINS+10)         /* GPIOD, PIN 10 */
#  define SAM_IRQ_PD11        (SAM_IRQ_GPIOD_PINS+11)         /* GPIOD, PIN 11 */
#  define SAM_IRQ_PD12        (SAM_IRQ_GPIOD_PINS+12)         /* GPIOD, PIN 12 */
#  define SAM_IRQ_PD13        (SAM_IRQ_GPIOD_PINS+13)         /* GPIOD, PIN 13 */
#  define SAM_IRQ_PD14        (SAM_IRQ_GPIOD_PINS+14)         /* GPIOD, PIN 14 */
#  define SAM_IRQ_PD15        (SAM_IRQ_GPIOD_PINS+15)         /* GPIOD, PIN 15 */
#  define SAM_IRQ_PD16        (SAM_IRQ_GPIOD_PINS+16)         /* GPIOD, PIN 16 */
#  define SAM_IRQ_PD17        (SAM_IRQ_GPIOD_PINS+17)         /* GPIOD, PIN 17 */
#  define SAM_IRQ_PD18        (SAM_IRQ_GPIOD_PINS+18)         /* GPIOD, PIN 18 */
#  define SAM_IRQ_PD19        (SAM_IRQ_GPIOD_PINS+19)         /* GPIOD, PIN 19 */
#  define SAM_IRQ_PD20        (SAM_IRQ_GPIOD_PINS+20)         /* GPIOD, PIN 20 */
#  define SAM_IRQ_PD21        (SAM_IRQ_GPIOD_PINS+21)         /* GPIOD, PIN 21 */
#  define SAM_IRQ_PD22        (SAM_IRQ_GPIOD_PINS+22)         /* GPIOD, PIN 22 */
#  define SAM_IRQ_PD23        (SAM_IRQ_GPIOD_PINS+23)         /* GPIOD, PIN 23 */
#  define SAM_IRQ_PD24        (SAM_IRQ_GPIOD_PINS+24)         /* GPIOD, PIN 24 */
#  define SAM_IRQ_PD25        (SAM_IRQ_GPIOD_PINS+25)         /* GPIOD, PIN 25 */
#  define SAM_IRQ_PD26        (SAM_IRQ_GPIOD_PINS+26)         /* GPIOD, PIN 26 */
#  define SAM_IRQ_PD27        (SAM_IRQ_GPIOD_PINS+27)         /* GPIOD, PIN 27 */
#  define SAM_IRQ_PD28        (SAM_IRQ_GPIOD_PINS+28)         /* GPIOD, PIN 28 */
#  define SAM_IRQ_PD29        (SAM_IRQ_GPIOD_PINS+29)         /* GPIOD, PIN 29 */
#  define SAM_IRQ_PD30        (SAM_IRQ_GPIOD_PINS+30)         /* GPIOD, PIN 30 */
#  define SAM_IRQ_PD31        (SAM_IRQ_GPIOD_PINS+31)         /* GPIOD, PIN 31 */
#  define SAM_NGPIODIRQS      32
#else
#  define SAM_NGPIODIRQS      0
#endif

#ifdef CONFIG_SAMV7_GPIOE_IRQ
#  define SAM_IRQ_GPIOE_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + SAM_NGPIOAIRQS + \
                               SAM_NGPIOBIRQS + SAM_NGPIOCIRQS + SAM_NGPIODIRQS)
#  define SAM_IRQ_PE0         (SAM_IRQ_GPIOE_PINS+0)          /* GPIOE, PIN 0 */
#  define SAM_IRQ_PE1         (SAM_IRQ_GPIOE_PINS+1)          /* GPIOE, PIN 1 */
#  define SAM_IRQ_PE2         (SAM_IRQ_GPIOE_PINS+2)          /* GPIOE, PIN 2 */
#  define SAM_IRQ_PE3         (SAM_IRQ_GPIOE_PINS+3)          /* GPIOE, PIN 3 */
#  define SAM_IRQ_PE4         (SAM_IRQ_GPIOE_PINS+4)          /* GPIOE, PIN 4 */
#  define SAM_IRQ_PE5         (SAM_IRQ_GPIOE_PINS+5)          /* GPIOE, PIN 5 */
#  define SAM_IRQ_PE6         (SAM_IRQ_GPIOE_PINS+6)          /* GPIOE, PIN 6 */
#  define SAM_IRQ_PE7         (SAM_IRQ_GPIOE_PINS+7)          /* GPIOE, PIN 7 */
#  define SAM_IRQ_PE8         (SAM_IRQ_GPIOE_PINS+8)          /* GPIOE, PIN 8 */
#  define SAM_IRQ_PE9         (SAM_IRQ_GPIOE_PINS+9)          /* GPIOE, PIN 9 */
#  define SAM_IRQ_PE10        (SAM_IRQ_GPIOE_PINS+10)         /* GPIOE, PIN 10 */
#  define SAM_IRQ_PE11        (SAM_IRQ_GPIOE_PINS+11)         /* GPIOE, PIN 11 */
#  define SAM_IRQ_PE12        (SAM_IRQ_GPIOE_PINS+12)         /* GPIOE, PIN 12 */
#  define SAM_IRQ_PE13        (SAM_IRQ_GPIOE_PINS+13)         /* GPIOE, PIN 13 */
#  define SAM_IRQ_PE14        (SAM_IRQ_GPIOE_PINS+14)         /* GPIOE, PIN 14 */
#  define SAM_IRQ_PE15        (SAM_IRQ_GPIOE_PINS+15)         /* GPIOE, PIN 15 */
#  define SAM_IRQ_PE16        (SAM_IRQ_GPIOE_PINS+16)         /* GPIOE, PIN 16 */
#  define SAM_IRQ_PE17        (SAM_IRQ_GPIOE_PINS+17)         /* GPIOE, PIN 17 */
#  define SAM_IRQ_PE18        (SAM_IRQ_GPIOE_PINS+18)         /* GPIOE, PIN 18 */
#  define SAM_IRQ_PE19        (SAM_IRQ_GPIOE_PINS+19)         /* GPIOE, PIN 19 */
#  define SAM_IRQ_PE20        (SAM_IRQ_GPIOE_PINS+20)         /* GPIOE, PIN 20 */
#  define SAM_IRQ_PE21        (SAM_IRQ_GPIOE_PINS+21)         /* GPIOE, PIN 21 */
#  define SAM_IRQ_PE22        (SAM_IRQ_GPIOE_PINS+22)         /* GPIOE, PIN 22 */
#  define SAM_IRQ_PE23        (SAM_IRQ_GPIOE_PINS+23)         /* GPIOE, PIN 23 */
#  define SAM_IRQ_PE24        (SAM_IRQ_GPIOE_PINS+24)         /* GPIOE, PIN 24 */
#  define SAM_IRQ_PE25        (SAM_IRQ_GPIOE_PINS+25)         /* GPIOE, PIN 25 */
#  define SAM_IRQ_PE26        (SAM_IRQ_GPIOE_PINS+26)         /* GPIOE, PIN 26 */
#  define SAM_IRQ_PE27        (SAM_IRQ_GPIOE_PINS+27)         /* GPIOE, PIN 27 */
#  define SAM_IRQ_PE28        (SAM_IRQ_GPIOE_PINS+28)         /* GPIOE, PIN 28 */
#  define SAM_IRQ_PE29        (SAM_IRQ_GPIOE_PINS+29)         /* GPIOE, PIN 29 */
#  define SAM_IRQ_PE30        (SAM_IRQ_GPIOE_PINS+30)         /* GPIOE, PIN 30 */
#  define SAM_IRQ_PE31        (SAM_IRQ_GPIOE_PINS+31)         /* GPIOE, PIN 31 */
#  define SAM_NGPIOEIRQS      32
#else
#  define SAM_NGPIOEIRQS      0
#endif

/* Total number of IRQ numbers */

#define NR_IRQS               (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + \
                               SAM_NGPIOAIRQS + SAM_NGPIOBIRQS + SAM_NGPIOCIRQS + \
                               SAM_NGPIODIRQS + SAM_NGPIOEIRQS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_SAMV7_SAME70_IRQ_H */
