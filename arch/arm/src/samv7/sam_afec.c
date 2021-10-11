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

struct samv7_dev_s
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

static void afec_putreg(FAR struct samv7_dev_s *priv, uint32_t offset,
                       uint32_t value);
static uint32_t afec_getreg(FAR struct samv7_dev_s *priv, uint32_t offset);
static void afec_modifyreg(FAR struct samv7_dev_s *priv, uint32_t offset,
                          uint32_t clearbits, uint32_t setbits);

/* ADC methods */

static int  adc_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void adc_reset(FAR struct adc_dev_s *dev);
static int  adc_setup(FAR struct adc_dev_s *dev);
static void adc_shutdown(FAR struct adc_dev_s *dev);
static void adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);
static int  adc_interrupt(int irq, void *context, FAR void *arg);

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
    GPIO_AFE0_AD2,
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
    GPIO_AFE1_AD2,
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

static void afec_putreg(FAR struct samv7_dev_s *priv, uint32_t offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static uint32_t afec_getreg(FAR struct samv7_dev_s *priv, uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static void afec_modifyreg(FAR struct samv7_dev_s *priv, uint32_t offset,
                          uint32_t clearbits, uint32_t setbits)
{
  modifyreg32(priv->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int adc_bind(FAR struct adc_dev_s *dev,
                    FAR const struct adc_callback_s *callback)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void adc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  irqstate_t flags;

  printf("in adc_reset 1\n");

  flags = enter_critical_section();

  /* Do nothing if ADC instance is currently in use */

  if (priv->initialized > 0)
    {
      goto exit_leave_critical;
    }

  /* Configure clock gating */

  switch (priv->intf)
    {
#ifdef CONFIG_SAMV7_AFEC0
      case 0:
        sam_afec0_enableclk();
        break;
#endif
#ifdef CONFIG_SAMV7_AFEC1
      case 1:
        sam_afec1_enableclk();
        break;
#endif
      default:
        aerr("ERROR: Tried to reset non-existing ADC: %d\n", priv->intf);
        goto exit_leave_critical;
    }

  leave_critical_section(flags);


  printf("in adc_reset 2\n");

  /* Configure ADC */

  uint32_t afec_acr = AFEC_ACR_PGA0EN | AFEC_ACR_PGA1EN;
  afec_putreg(priv, SAM_AFEC_ACR_OFFSET, afec_acr);

  /* Pad configuration */

  gpio_pinset_t *pinlist = NULL;
  switch (priv->intf)
    {
#ifdef CONFIG_SAMV7_AFEC0
      case 1:
        pinlist = g_adcpinlist0;
        break;
#endif
#ifdef CONFIG_SAMV7_AFEC1
      case 2:
        pinlist = g_adcpinlist1;
        break;
#endif
      default:
        /* We have already checked the intf number earlier in this function,
         * so we should never get here.
         */

        return;
    }

  /* Disable all channels */

  afec_putreg(priv, SAM_AFEC_CHDR_OFFSET, AFEC_CHALL);

  gpio_pinset_t pinset = 0;
  uint32_t afec_cher;
  uint32_t afec_ier;
  for (int i = 0; i < priv->nchannels; i++)
    {
      DEBUGASSERT(priv->chanlist[i] < ADC_MAX_CHANNELS);
      pinset = pinlist[priv->chanlist[i]];
      sam_configgpio(pinset);

      /* Enable the corresponding channel */

      afec_cher = AFEC_CH(priv->chanlist[i]);
      afec_modifyreg(priv, SAM_AFEC_CHER_OFFSET, AFEC_CHALL, afec_cher);

      afec_ier = AFEC_INT_EOC(priv->chanlist[i]);
      afec_modifyreg(priv, SAM_AFEC_IER_OFFSET, AFEC_INT_EOCALL, afec_ier);

    }

  return;

exit_leave_critical:
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;

  /* Do nothing when the ADC device is already set up */

  if (priv->initialized > 0)
    {
      return OK;
    }

  priv->initialized++;

  int ret = irq_attach(priv->irq, adc_interrupt, dev);
  if (ret < 0)
    {
      ainfo("irq_attach failed: %d\n", ret);
      return ret;
    }

  up_enable_irq(priv->irq);

  /* Start the first conversion */
  printf("conv started\n");
  priv->current = 0;

  uint32_t afec_cselr = AFEC_CSELR_CSEL(priv->chanlist[priv->current]);
  afec_putreg(priv, SAM_AFEC_CSELR_OFFSET, afec_cselr);

  uint32_t afec_cr = AFEC_CR_START;
  afec_putreg(priv, SAM_AFEC_CR_OFFSET, afec_cr);

  return ret;
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;

  up_disable_irq(priv->irq);

  /* Then detach the ADC interrupt handler. */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   cmd - command
 *   arg - arguments passed with command
 *
 * Returned Value:
 *
 ****************************************************************************/

static int adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  int ret = -ENOTTY;

  switch (cmd)
    {
      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = priv->nchannels;
        }
        break;

      default:
        {
          aerr("ERROR: Unknown cmd: %d\n", cmd);
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   ADC interrupt handler
 *
 ****************************************************************************/

static int adc_interrupt(int irq, void *context, FAR void *arg)
{
  FAR struct adc_dev_s *dev = (FAR struct adc_dev_s *)arg;
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  int32_t data;
  printf("interrupt\n");
  if ((afec_getreg(priv, SAM_AFEC_ISR_OFFSET) & AFEC_INT_EOC(priv->chanlist[priv->current])) != 0)
    {
      /* Read data. This also clears the COCO bit. */

      data = (int32_t)afec_getreg(priv, SAM_AFEC_CDR_OFFSET);

      if (priv->cb != NULL)
        {
          DEBUGASSERT(priv->cb->au_receive != NULL);
          priv->cb->au_receive(dev, priv->chanlist[priv->current],  data);
        }

      /* Set the channel number of the next channel that will complete
       * conversion.
       */

      priv->current++;

      if (priv->current >= priv->nchannels)
        {
          /* Restart the conversion sequence from the beginning */

          priv->current = 0;
        }

      /* Start the next conversion */

      uint32_t afec_cselr = AFEC_CSELR_CSEL(priv->chanlist[priv->current]);
      afec_putreg(priv, SAM_AFEC_CSELR_OFFSET, afec_cselr);

    }

  /* There are no interrupt flags left to clear */

  return OK;
}

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

  priv = (FAR struct samv7_dev_s *)dev->ad_priv;

  priv->nchannels = nchannels;
  memcpy(priv->chanlist, chanlist, nchannels);

  ainfo("intf: %d nchannels: %d\n", priv->intf, priv->nchannels);

  return dev;
}

#endif /* CONFIG_SAMV7_AFEC0 || CONFIG_SAMV7_AFEC1 */

#endif /* CONFIG_SAMV7_ADC */