/****************************************************************************
 * arch/arm/src/samv7/sam_afec.c
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "hardware/sam_matrix.h"
#include "hardware/sam_pinmap.h"
#include "sam_periphclks.h"
#include "sam_gpio.h"
#include "sam_afec.h"

#ifdef CONFIG_ADC

#if defined(CONFIG_SAMV7_AFEC0) || defined(CONFIG_SAMV7_AFEC1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_MAX_CHANNELS 11

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_dev_s
{
  FAR const struct adc_callback_s *cb;  /* Upper driver callback */
  uint8_t  intf;                        /* ADC number (i.e. ADC1, ADC2) */
  uint32_t base;                        /* ADC register base */
  uint8_t  initialized;                 /* ADC initialization counter */
  int      irq;                         /* ADC IRQ number */
  int      nchannels;                   /* Number of configured ADC channels */
  uint8_t  chanlist[ADC_MAX_CHANNELS];  /* ADC channel list */
  uint8_t  current;                     /* Current channel being converted */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

 /****************************************************************************
 * Private Data
 ****************************************************************************/

 static const struct adc_ops_s g_adcops =
{
  .ao_bind     = adc_bind,
  .ao_reset    = adc_reset,
  .ao_setup    = adc_setup,
  .ao_shutdown = adc_shutdown,
  .ao_rxint    = adc_rxint,
  .ao_ioctl    = adc_ioctl,
};

#ifdef CONFIG_SAMV7_AFEC0
static struct samv7_dev_s g_adcpriv0 =
{
  .irq         = SAM_IRQ_AFEC0,
  .intf        = 0,
  .initialized = 0,
  .base        = SAM_AFEC0_BASE,
};

static struct adc_dev_s g_adcdev0 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv0,
};

gpio_pinset_t g_adcpinlist0[ADC_MAX_CHANNELS] =
{
    GPIO_AFE0_AD0,
    GPIO_AFE0_AD1,
    GPIO_AFE0_AD2
    GPIO_AFE0_AD3,
    GPIO_AFE0_AD4,
    GPIO_AFE0_AD5,
    GPIO_AFE0_AD6,
    GPIO_AFE0_AD7,
    GPIO_AFE0_AD8,
    GPIO_AFE0_AD0,
    GPIO_AFE0_AD10,
};
#endif

#ifdef CONFIG_SAMV7_AFEC1
static struct samv7_dev_s g_adcpriv1 =
{
  .irq         = SAM_IRQ_AFEC1,
  .intf        = 1,
  .initialized = 0,
  .base        = SAM_AFEC1_BASE,
};

static struct adc_dev_s g_adcdev1 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv0,
};

gpio_pinset_t g_adcpinlist0[ADC_MAX_CHANNELS] =
{
    GPIO_AFE1_AD0,
    GPIO_AFE1_AD1,
    GPIO_AFE1_AD2
    GPIO_AFE1_AD3,
    GPIO_AFE1_AD4,
    GPIO_AFE1_AD5,
    GPIO_AFE1_AD6,
    GPIO_AFE1_AD7,
    GPIO_AFE1_AD8,
    GPIO_AFE1_AD0,
    GPIO_AFE1_AD10,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

 /****************************************************************************
 * Public Functions
 ****************************************************************************/

 /****************************************************************************
 * Name: sam_afec_initialize
 *
 * Description:
 *   Initialize the adc
 *
 * Input Parameters:
 *   intf      - ADC number (1 or 2)
 *   chanlist  - The list of channels
 *   nchannels - Number of channels
 *
 * Returned Value:
 *   Valid can device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *sam_afec_initialize(int intf,
                                          FAR const uint8_t *chanlist,
                                          int nchannels)
{
  FAR struct adc_dev_s *dev;
  FAR struct samv7_dev_s *priv;

  DEBUGASSERT(nchannels > 0);

  switch (intf)
    {
#ifdef CONFIG_SAMV7_AFEC0
      case 0:
        {
          dev = &g_adcdev0;
          break;
        }
#endif /* CONFIG_SAMV7_AFEC0 */

#ifdef CONFIG_SAMV7_AFEC1
      case 1:
        {
          dev = &g_adcdev1;
          break;
        }
#endif /* CONFIG_SAMV7_AFEC1 */

      default:
        {
          aerr("ERROR: Tried to initialize invalid ADC: %d\n", intf);
          return NULL;
        }
    }

  priv = (FAR struct imxrt_dev_s *)dev->ad_priv;

  priv->nchannels = nchannels;
  memcpy(priv->chanlist, chanlist, nchannels);

  ainfo("intf: %d nchannels: %d\n", priv->intf, priv->nchannels);

  return dev;
}

#endif /* CONFIG_SAMV7_AFEC0 || CONFIG_SAMV7_AFEC1 */

#endif /* CONFIG_SAMV7_ADC */