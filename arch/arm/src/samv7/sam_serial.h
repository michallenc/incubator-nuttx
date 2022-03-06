/****************************************************************************
 * arch/arm/src/samv7/sam_xdmac.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_SERIAL_H
#define __ARCH_ARM_SRC_SAMV7_SAM_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* No DMA ops */

#undef SERIAL_HAVE_NODMA_OPS
#if defined(CONFIG_SAMV7_USART0) && !defined(CONFIG_USART0_RXDMA) &&   \
    !defined(CONFIG_USART1_TXDMA)
#  define SERIAL_HAVE_NODMA_OPS
#elif defined(CONFIG_SAMV7_USART1) && !defined(CONFIG_USART1_RXDMA) && \
    !defined(CONFIG_USART2_TXDMA)
#  define SERIAL_HAVE_NODMA_OPS
#elif defined(CONFIG_SAMV7_USART2) && !defined(CONFIG_USART2_RXDMA) && \
    !defined(CONFIG_USART2_TXDMA)
#  define SERIAL_HAVE_NODMA_OPS
#elif defined(CONFIG_SAMV7_UART0) && !defined(CONFIG_UART0_RXDMA) &&  \
    !defined(CONFIG_UART0_TXDMA)
#  define SERIAL_HAVE_NODMA_OPS
#elif defined(CONFIG_SAMV7_UART1) && !defined(CONFIG_UART1_RXDMA) &&  \
    !defined(CONFIG_UART1_TXDMA)
#  define SERIAL_HAVE_NODMA_OPS
#elif defined(CONFIG_SAMV7_UART2) && !defined(CONFIG_UART2_RXDMA) &&  \
    !defined(CONFIG_UART2_TXDMA)
#  define SERIAL_HAVE_NODMA_OPS
#elif defined(CONFIG_SAMV7_UART3) && !defined(CONFIG_UART3_RXDMA) &&  \
    !defined(CONFIG_UART3_TXDMA)
#  define SERIAL_HAVE_NODMA_OPS
#elif defined(CONFIG_SAMV7_UART4) && !defined(CONFIG_UART4_RXDMA) &&  \
    !defined(CONFIG_UART4_TXDMA)
#  define SERIAL_HAVE_NODMA_OPS
#endif

/* RX+TX DMA ops */

#undef SERIAL_HAVE_RXTXDMA_OPS
#if defined(CONFIG_USART0_RXDMA) && defined(CONFIG_USART0_TXDMA)
#  define SERIAL_HAVE_RXTXDMA_OPS
#elif defined(CONFIG_USART1_RXDMA) && defined(CONFIG_USART1_TXDMA)
#  define SERIAL_HAVE_RXTXDMA_OPS
#elif defined(CONFIG_USART2_RXDMA) && defined(CONFIG_USART2_TXDMA)
#  define SERIAL_HAVE_RXTXDMA_OPS
#elif defined(CONFIG_UART0_RXDMA) && defined(CONFIG_UART0_TXDMA)
#  define SERIAL_HAVE_RXTXDMA_OPS
#elif defined(CONFIG_UART1_RXDMA) && defined(CONFIG_UART1_TXDMA)
#  define SERIAL_HAVE_RXTXDMA_OPS
#elif defined(CONFIG_UART2_RXDMA) && defined(CONFIG_UART2_TXDMA)
#  define SERIAL_HAVE_RXTXDMA_OPS
#elif defined(CONFIG_UART3_RXDMA) && defined(CONFIG_UART3_TXDMA)
#  define SERIAL_HAVE_RXTXDMA_OPS
#elif defined(CONFIG_UART4_RXDMA) && defined(CONFIG_UART4_TXDMA)
#  define SERIAL_HAVE_RXTXDMA_OPS
#endif

/* TX DMA ops */

#undef SERIAL_HAVE_TXDMA_OPS
#if !defined(CONFIG_USART0_RXDMA) && defined(CONFIG_USART0_TXDMA)
#  define SERIAL_HAVE_TXDMA_OPS
#elif !defined(CONFIG_USART1_RXDMA) && defined(CONFIG_USART1_TXDMA)
#  define SERIAL_HAVE_TXDMA_OPS
#elif !defined(CONFIG_USART2_RXDMA) && defined(CONFIG_USART2_TXDMA)
#  define SERIAL_HAVE_TXDMA_OPS
#elif !defined(CONFIG_UART0_RXDMA) && defined(CONFIG_UART0_TXDMA)
#  define SERIAL_HAVE_TXDMA_OPS
#elif !defined(CONFIG_UART1_RXDMA) && defined(CONFIG_UART1_TXDMA)
#  define SERIAL_HAVE_TXDMA_OPS
#elif !defined(CONFIG_UART2_RXDMA) && defined(CONFIG_UART2_TXDMA)
#  define SERIAL_HAVE_TXDMA_OPS
#elif !defined(CONFIG_UART3_RXDMA) && defined(CONFIG_UART3_TXDMA)
#  define SERIAL_HAVE_TXDMA_OPS
#elif !defined(CONFIG_UART4_RXDMA) && defined(CONFIG_UART4_TXDMA)
#  define SERIAL_HAVE_TXDMA_OPS
#endif

/* RX DMA ops */

#undef SERIAL_HAVE_RXDMA_OPS
#if defined(CONFIG_USART0_RXDMA) && !defined(CONFIG_USART0_TXDMA)
#  define SERIAL_HAVE_RXDMA_OPS
#elif defined(CONFIG_USART1_RXDMA) && !defined(CONFIG_USART1_TXDMA)
#  define SERIAL_HAVE_RXDMA_OPS
#elif defined(CONFIG_USART2_RXDMA) && !defined(CONFIG_USART2_TXDMA)
#  define SERIAL_HAVE_RXDMA_OPS
#elif defined(CONFIG_UART0_RXDMA) && !defined(CONFIG_UART0_TXDMA)
#  define SERIAL_HAVE_RXDMA_OPS
#elif defined(CONFIG_UART1_RXDMA) && !defined(CONFIG_UART1_TXDMA)
#  define SERIAL_HAVE_RXDMA_OPS
#elif defined(CONFIG_UART2_RXDMA) && !defined(CONFIG_UART2_TXDMA)
#  define SERIAL_HAVE_RXDMA_OPS
#elif defined(CONFIG_UART3_RXDMA) && !defined(CONFIG_UART3_TXDMA)
#  define SERIAL_HAVE_RXDMA_OPS
#elif defined(CONFIG_UART4_RXDMA) && !defined(CONFIG_UART4_TXDMA)
#  define SERIAL_HAVE_RXDMA_OPS
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_serial_dma_poll
 *
 * Description:
 *   Checks receive DMA buffers for received bytes that have not accumulated
 *   to the point where the DMA half/full interrupt has triggered.
 *
 *   This function should be called from a timer or other periodic context.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
void sam_serial_dma_poll(void)
#endif

#endif /* __ARCH_ARM_SRC_SAMV7_SAM_SERIAL_H */