/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_boot.h
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

#ifndef __ARCH_ARM64_SRC_BCM2711_BCM2711_BOOT_H
#define __ARCH_ARM64_SRC_BCM2711_BCM2711_BOOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "arm64_arch.h"
#include "arm64_internal.h"
#include <arch/chip/chip.h>
#include <nuttx/compiler.h>
#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

/****************************************************************************
 * Name: bcm2711_board_initialize
 *
 * Description:
 *   All BCM2711 architectures must provide the following entry point.  This
 *   entry point is called in the initialization phase -- after
 *   bcm2711_memory_initialize and after all memory has been configured and
 *   mapped but before any devices have been initialized.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bcm2711_board_initialize(void);

#endif // __ASSEMBLY__

#endif // __ARCH_ARM64_SRC_BCM2711_BCM2711_BOOT_H
