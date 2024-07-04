/****************************************************************************
 * arch/arm/src/mcx/hardware/mcxa/mcxa_memorymap.h
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

#ifndef __ARCH_ARM_SRC_MCX_HARDWARE_MCXA_MCXA_MEMORYMAP_H
#define __ARCH_ARM_SRC_MCX_HARDWARE_MCXA_MCXA_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System memory map */

/* Peripherals */

#define MCX_INPUTMUX0_BASE           0x40001000
#define MCX_I3C0_BASE                0x40002000
#define MCX_CTRIMER0_BASE            0x40004000
#define MCX_CTRIMER1_BASE            0x40005000
#define MCX_CTRIMER2_BASE            0x40006000
#define MCX_FREQME2_BASE             0x40009000
#define MCX_UTICK0_BASE              0x4000b000
#define MCX_WWDT0_BASE               0x4000c000
#define MCX_LPI2C0_BASE              0x4009a000
#define MCX_LPSPI0_BASE              0x4009c000
#define MCX_LPSPI1_BASE              0x4009d000
#define MCX_LPUART0_BASE             0x4009f000
#define MCX_LPUART0_BASE             0x400a0000
#define MCX_LPUART0_BASE             0x400a1000
#define MCX_QDC0_BASE                0x400a7000
#define MCX_USB0_BASE                0x400a4000
#define MCX_FLEXPWM0_BASE            0x400a9000
#define MCX_LPTMR0_BASE              0x400ab000
#define MCX_OSTIMER0_BASE            0x400ae000
#define MCX_WAKETIMER0_BASE          0x400ad000
#define MCX_HSADC0_BASE              0x400af000
#define MCX_CMP0_BASE                0x400b1000
#define MCX_CMP1_BASE                0x400b2000
#define MCX_PORT0_BASE               0x400bc000
#define MCX_PORT1_BASE               0x400bd000
#define MCX_PORT2_BASE               0x400be000
#define MCX_PORT3_BASE               0x400bf000
#define MCX_CRC0_BASE                0x4008a000
#define MCX_MBC0_BASE                0x4008e000
#define MCX_CDOG0_BASE               0x40100000
#define MCX_DBGMB0_BASE              0x40101000
#define MCX_RGPIO0_BASE              0x40102000
#define MCX_RGPIO1_BASE              0x40103000
#define MCX_RGPIO2_BASE              0x40104000
#define MCX_RGPIO3_BASE              0x40105000

#endif /* __ARCH_ARM_SRC_MCX_HARDWARE_MCXA_MCXA_MEMORYMAP_H */
