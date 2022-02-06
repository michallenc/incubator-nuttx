/****************************************************************************
 * boards/arm/samv7/brcg2/src/sam_adm2483.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include "arm_arch.h"
#include "sam_gpio.h"
#include "brcg2.h"

#ifdef CONFIG_BRCG2_ADM2483_USART

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sam_adm2483_enable
 *
 * Description:
 *   Called from sam_bringup(), enables adm2483 driver.
 *
 ****************************************************************************/

void sam_adm2483_enable(void)
{
  /* Set on the DE pin (PD_18) and enable the driver */

  sam_configgpio(GPIO_ADM2483_EN);
  sam_gpiowrite(GPIO_ADM2483_EN, 1);
}

#endif /* CONFIG_BRCG2_ADM2483_USART */