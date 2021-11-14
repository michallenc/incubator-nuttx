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
#include <nuttx/wqueue.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "hardware/sam_matrix.h"
#include "hardware/sam_pinmap.h"
#include "hardware/sam_pio.h"
#include "sam_periphclks.h"
#include "sam_gpio.h"
#include "sam_afec.h"
#include "sam_xdmac.h"

#ifdef CONFIG_ADC

#if defined(CONFIG_SAMV7_AFEC0) || defined(CONFIG_SAMV7_AFEC1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_MAX_CHANNELS 11

#ifdef CONFIG_SAMV7_AFEC_DMA
#define DMA_FLAGS  (DMACH_FLAG_FIFOCFG_LARGEST | \
     DMACH_FLAG_PERIPHPID(29) | \
     DMACH_FLAG_PERIPHH2SEL | DMACH_FLAG_PERIPHISPERIPH |  \
     DMACH_FLAG_PERIPHWIDTH_16BITS | DMACH_FLAG_PERIPHCHUNKSIZE_1 | \
     DMACH_FLAG_MEMPID_MAX | DMACH_FLAG_MEMAHB_AHB_IF0 | \
     DMACH_FLAG_MEMWIDTH_16BITS | DMACH_FLAG_MEMINCREMENT | \
     DMACH_FLAG_MEMCHUNKSIZE_1 | DMACH_FLAG_MEMBURST_4)
#endif

#if !defined(CONFIG_SAMV7_AFEC_DMA)
#  undef  CONFIG_SAMV7_AFEC_DMASAMPLES
#  define CONFIG_SAMV7_AFEC_DMASAMPLES 1
#elif !defined(CONFIG_SAMV7_AFEC_DMASAMPLES)
#  error CONFIG_SAMV7_AFEC_DMASAMPLES must be defined
#elif CONFIG_SAMV7_AFEC_DMASAMPLES < 2
#  warning Values of CONFIG_SAMV7_AFEC_DMASAMPLES < 2 are inefficient
#endif

#define SAMV7_AFEC_SAMPLES (CONFIG_SAMV7_AFEC_DMASAMPLES * 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct samv7_dev_s
{
  FAR const struct adc_callback_s *cb;  /* Upper driver callback */
  uint8_t  intf;                        /* ADC number (i.e. ADC1, ADC2) */
  uint32_t base;                        /* ADC register base */
  uint8_t  initialized;                 /* ADC initialization counter */
  uint8_t  resolution;                  /* ADC resolution (SAMV7_AFECn_RES) */
  int      irq;                         /* ADC IRQ number */
  int      nchannels;                   /* Number of configured channels */
  uint8_t  chanlist[ADC_MAX_CHANNELS];  /* ADC channel list */
  uint8_t  current;                     /* Current channel being converted */

#ifdef CONFIG_SAMV7_AFEC_DMA
  volatile bool odd;     /* Odd buffer is in use */
  volatile bool ready;   /* Worker has completed the last set of samples */
  volatile bool enabled; /* DMA data transfer is enabled */
  int nsamples;
  DMA_HANDLE dma;        /* Handle for DMA channel */
  struct work_s work;    /* Supports the interrupt handling "bottom half" */

  /* DMA sample data buffer */

  uint16_t evenbuf[SAMV7_AFEC_SAMPLES];
  uint16_t oddbuf[SAMV7_AFEC_SAMPLES];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SAMV7_AFEC_DMA
static void sam_afec_dmadone(void *arg);
static void sam_afec_dmacallback(DMA_HANDLE handle, void *arg, int result);
static int  sam_afec_dmasetup(struct adc_dev_s *dev, FAR uint8_t *buffer,
                             size_t buflen);
static void sam_afec_dmastart(struct adc_dev_s *dev);
#endif

static void afec_putreg(FAR struct samv7_dev_s *priv, uint32_t offset,
                       uint32_t value);
static uint32_t afec_getreg(FAR struct samv7_dev_s *priv, uint32_t offset);

/* ADC methods */

static int  afec_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void afec_reset(FAR struct adc_dev_s *dev);
static int  afec_setup(FAR struct adc_dev_s *dev);
static void afec_shutdown(FAR struct adc_dev_s *dev);
static void afec_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  afec_ioctl(FAR struct adc_dev_s *dev,
                       int cmd, unsigned long arg);
static int  afec_interrupt(int irq, void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = afec_bind,
  .ao_reset    = afec_reset,
  .ao_setup    = afec_setup,
  .ao_shutdown = afec_shutdown,
  .ao_rxint    = afec_rxint,
  .ao_ioctl    = afec_ioctl,
};

#ifdef CONFIG_SAMV7_AFEC0
static struct samv7_dev_s g_adcpriv0 =
{
  .irq         = SAM_IRQ_AFEC0,
  .intf        = 0,
  .initialized = 0,
  .resolution  = CONFIG_SAMV7_AFEC0_RES,
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
  .resolution  = CONFIG_SAMV7_AFEC1_RES,
  .base        = SAM_AFEC1_BASE,
};

static struct adc_dev_s g_adcdev1 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv1,
};

gpio_pinset_t g_adcpinlist1[ADC_MAX_CHANNELS] =
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


/****************************************************************************
 * DMA Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam_afec_dmadone
 *
 * Description:
 *   This function executes on the worker thread.  It is scheduled by
 *   sam_adc_dmacallback at the complete of each DMA sequenece.  There is
 *   and interlock using ping-pong buffers and boolean values to prevent
 *   overrunning the worker thread:
 *
 *     oddbuf[]/evenbuf[] - Ping pong buffers are used.  The DMA collects
 *       data in one buffer while the worker thread processes data in the
 *       other.
 *     odd - If true, then DMA is active in the oddbuf[]; evenbuf[] holds
 *       completed DMA data.
 *     ready - Ping ponging is halted while ready is false;  If data overrun
 *       occurs, then sample data will be lost on one sequence.  The worker
 *       thread sets ready when it has completed processing the last sample
 *       data.
 *
 * Input Parameters:
 *   arg - The ADC private data structure cast to (void *)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_AFEC_DMA
static void sam_afec_dmadone(void *arg)
{
  printf("in dmadone\n");
  FAR struct adc_dev_s *dev = (FAR struct adc_dev_s *)arg;
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  uint16_t *buffer;
  uint16_t *next;
  uint16_t sample;
  int chan;
  int i;

  ainfo("ready=%d enabled=%d\n", priv->enabled, priv->ready);
  DEBUGASSERT(priv != NULL && !priv->ready);

  /* If the DMA transfer is not enabled, just ignore the data (and do not
   * start the next DMA transfer).
   */

  if (priv->enabled)
    {
      /* Toggle to the next buffer.
       *
       *   buffer - The buffer on which the DMA has just completed
       *   next   - The buffer in which to start the next DMA
       */

      if (priv->odd)
        {
          buffer    = priv->oddbuf;
          next      = priv->evenbuf;
          priv->odd = false;
        }
      else
        {
          buffer    = priv->evenbuf;
          next      = priv->oddbuf;
          priv->odd = true;
        }

      /* Restart the DMA conversion as quickly as possible using the next
       * buffer.
       *
       * REVISIT: In the original design, toggling the ping-pong buffers and
       * restarting the DMA was done in the interrupt handler so that the
       * next buffer could be filling while the current buffer is being
       * processed here on the worker thread.  But, unfortunately,
       * sam_adcm_dmasetup() cannot be called from an interrupt handler.
       *
       * A consequence of this is that there is a small window from the time
       * that the last set of samples was taken, the worker thread runs, and
       * the follow logic restarts the DMA in which samples could be lost!
       *
       * Without the interrupt level DMA restart logic, there is not really
       * any good reason to support the ping-poing buffers at all.
       */

      sam_afec_dmasetup(dev, (FAR uint8_t *)next,
                       priv->nsamples * sizeof(uint16_t));

      /* Invalidate the DMA buffer so that we are guaranteed to reload the
       * newly DMAed data from RAM.
       */

      up_invalidate_dcache((uintptr_t)buffer,
                           (uintptr_t)buffer +
                           priv->nsamples * sizeof(uint16_t));

      /* Process each sample */

      printf("processing samples\n");

      for (i = 0; i < priv->nsamples; i++, buffer++)
        {
          /* Get the sample and the channel number */

          chan   = (int)((*buffer & AFEC_LCDR_CHANB_MASK) >>
                    AFEC_LCDR_CHANB_SHIFT);
          sample = ((*buffer & AFEC_LCDR_LDATA_MASK) >> AFEC_LCDR_LDATA_SHIFT);

          /* Verify the upper-half driver has bound its callback functions */

          if (priv->cb != NULL)
            {
              /* Give the sample data to the ADC upper half */

              DEBUGASSERT(priv->cb->au_receive != NULL);
              priv->cb->au_receive(dev, chan, sample);
            }
        }
    }

  /* We are ready to handle the next sample sequence */

  priv->ready = true;
}

/****************************************************************************
 * Name: sam_afec_dmastart
 *
 * Description:
 *   Initiate DMA sampling.
 *
 ****************************************************************************/

static void sam_afec_dmastart(struct adc_dev_s *dev)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;

  /* Make sure that the worker is available and that DMA is not disabled */

  if (priv->ready && priv->enabled)
    {
      priv->odd = false;  /* Start with the even buffer */
      sam_afec_dmasetup(dev, (FAR uint8_t *)priv->evenbuf,
                       priv->nsamples * sizeof(uint16_t));
    }
}

/****************************************************************************
 * Name: sam_afec_dmacallback
 *
 * Description:
 *   Called when one ADC DMA sequence completes.  This function defers
 *   processing of the samples to sam_adc_dmadone which runs on the worker
 *   thread.
 *
 ****************************************************************************/

static void sam_afec_dmacallback(DMA_HANDLE handle, void *arg, int result)
{
  printf("beggining of the callbck\n");
  FAR struct adc_dev_s *dev = (FAR struct adc_dev_s *)arg;
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  int ret;

  ainfo("ready=%d enabled=%d\n", priv->enabled, priv->ready);
  DEBUGASSERT(priv->ready);

  /* Check of the bottom half is keeping up with us.
   *
   * ready == false:  Would mean that the worker thready has not ran since
   *   the last DMA callback.
   * enabled == false: Means that the upper half has asked us nicely to stop
   *   transferring DMA data.
   */

  if (priv->ready && priv->enabled)
    {
      /* Verify that the worker is available */

      DEBUGASSERT(priv->work.worker == NULL);

      /* Mark the work as busy and schedule the DMA done processing to
       * occur on the worker thread.
       */

      priv->ready = false;
      printf("in dmacalbac getting work queue\n");
      ret = work_queue(HPWORK, &priv->work, sam_afec_dmadone, dev, 0);
      if (ret != 0)
        {
          aerr("ERROR: Failed to queue work: %d\n", ret);
        }
    }

  /* REVISIT: There used to be logic here to toggle the ping-pong buffers and
   * to restart the DMA conversion.  This would allow refilling one buffer
   * while the worker processes the other buffer that was just filled. But,
   * unfortunately, sam_adcm_dmasetup() and dma_rxsetup cannot be called
   * from an interrupt handler.
   *
   * A consequence of this is that there is a small window from the time
   * that the last set of samples was taken, the worker thread runs, and the
   * logic on the worker thread restarts the DMA.  Samples trigger during
   * this window will be be lost!
   *
   * Without this logic, there is not really any strong reason to support
   * the ping-poing buffers at all.
   */
}

/****************************************************************************
 * Name: sam_afec_dmasetup
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For read transfers this may mean
 *   invalidating the data cache.
 *
 * Input Parameters:
 *   priv   - An instance of the ADC device interface
 *   buffer - The memory to DMA from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int sam_afec_dmasetup(FAR struct adc_dev_s *dev, FAR uint8_t *buffer,
                            size_t buflen)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  uint32_t paddr;
  uint32_t maddr;

  ainfo("buffer=%p buflen=%d\n", buffer, (int)buflen);
  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Physical address of the ADC LCDR register and of the buffer location in
   * RAM.
   */

  paddr = priv->base + SAM_AFEC_LCDR_OFFSET;
  maddr = (uintptr_t)buffer;

  /* Configure the RX DMA */

  sam_dmarxsetup(priv->dma, paddr, maddr, buflen);

  /* Start the DMA */

  sam_dmastart(priv->dma, sam_afec_dmacallback, dev);
  return OK;
}
#endif

/****************************************************************************
 * Name: afec_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int afec_bind(FAR struct adc_dev_s *dev,
                    FAR const struct adc_callback_s *callback)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: afec_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before afec_setup() and on error conditions.
 *
 ****************************************************************************/

static void afec_reset(FAR struct adc_dev_s *dev)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Do nothing if ADC instance is currently in use */

  if (priv->initialized > 0)
    {
      goto exit_leave_critical;
    }

  /* Stop any ongoing DMA */

#ifdef CONFIG_SAMV7_AFEC_DMA
  if (priv->dma)
    {
      sam_dmastop(priv->dma);
    }
#endif

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

  /* Software reset */

  afec_putreg(priv, SAM_AFEC_CR_OFFSET, AFEC_CR_SWRST);

  /* Configure Mode Register */

  uint32_t afec_mr = AFEC_MR_STARTUP_64 | AFEC_MR_PRESCAL(2) | AFEC_MR_ONE;
  afec_putreg(priv, SAM_AFEC_MR_OFFSET, afec_mr);

  /* Configure Extended Mode register */

  uint32_t afec_emr = AFEC_EMR_TAG | AFEC_EMR_STM | \
                      AFEC_EMR_RES(priv->resolution);
  afec_putreg(priv, SAM_AFEC_EMR_OFFSET, afec_emr);

  /* Configure Analog Control Register */

  uint32_t afec_acr = AFEC_ACR_PGA0EN | AFEC_ACR_PGA1EN | AFEC_ACR_IBCTL(2);
  afec_putreg(priv, SAM_AFEC_ACR_OFFSET, afec_acr);

  /* Pad configuration */

  gpio_pinset_t *pinlist = NULL;
  switch (priv->intf)
    {
#ifdef CONFIG_SAMV7_AFEC0
      case 0:
        pinlist = g_adcpinlist0;
        break;
#endif
#ifdef CONFIG_SAMV7_AFEC1
      case 1:
        pinlist = g_adcpinlist1;
        break;
#endif
      default:
        /* We have already checked the intf number earlier in this function,
         * so we should never get here.
         */

        return;
    }

  /* Desible write protection (should already be disabled by default) */

  afec_putreg(priv, SAM_AFEC_WPMR_OFFSET, AFEC_WPMR_WPKEY);

  /* Disable all channels */

  afec_putreg(priv, SAM_AFEC_CHDR_OFFSET, AFEC_CHALL);

  gpio_pinset_t pinset = 0;
  uint32_t afec_cher = 0;
  for (int i = 0; i < priv->nchannels; i++)
    {
      DEBUGASSERT(priv->chanlist[i] < ADC_MAX_CHANNELS);
      pinset = pinlist[priv->chanlist[i]];
      sam_configgpio(pinset);

      afec_putreg(priv, SAM_AFEC_CSELR_OFFSET,
                  AFEC_CSELR_CSEL(priv->chanlist[i]));
      afec_putreg(priv, SAM_AFEC_COCR_OFFSET, 0x200);

      afec_cher |= AFEC_CH(priv->chanlist[i]);
    }

  /* Enable channels */

  //afec_putreg(priv, SAM_AFEC_IER_OFFSET, AFEC_INT_DRDY);
  afec_putreg(priv, SAM_AFEC_CHER_OFFSET, afec_cher);

  return;

exit_leave_critical:
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: afec_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int afec_setup(FAR struct adc_dev_s *dev)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;

  /* Do nothing when the ADC device is already set up */

  if (priv->initialized > 0)
    {
      return OK;
    }

  priv->initialized++;

  int ret = irq_attach(priv->irq, afec_interrupt, dev);
  if (ret < 0)
    {
      ainfo("irq_attach failed: %d\n", ret);
      return ret;
    }

  //up_enable_irq(priv->irq);

  /* Start the first conversion */

  priv->current = 0;

  uint32_t afec_cselr = AFEC_CSELR_CSEL(priv->chanlist[priv->current]);
  afec_putreg(priv, SAM_AFEC_CSELR_OFFSET, afec_cselr);

#ifdef CONFIG_SAMV7_AFEC_DMA
  /* Initiate DMA transfers */

  priv->ready   = true;   /* Worker is available */
  priv->enabled = true;   /* Transfers are enabled */

  sam_afec_dmastart(dev);
#endif

  return ret;
}

/****************************************************************************
 * Name: afec_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void afec_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  uint32_t afec_ixr = 0;

#ifdef CONFIG_SAMV7_AFEC_DMA
  /* Ignore redundant requests */

  // if (priv->enabled != enable)
  //   {
  //     /* Set a flag.  If disabling, the DMA sequence will terminate at the
  //      * completion of the next DMA.
  //      */

  //     priv->enabled = enable;

  //     /* If enabling, then we need to restart the DMA transfer */

  //     sam_afec_dmastart(dev);
  //   }

#else

  for (int i = 0; i < priv->nchannels; i++)
    {
      afec_ixr |= AFEC_INT_EOC(priv->chanlist[i]);
    }

  /* Enable interrupts */

  if (enable)
    {
      afec_putreg(priv, SAM_AFEC_IER_OFFSET, afec_ixr);
    }
  else
    {
      afec_putreg(priv, SAM_AFEC_IDR_OFFSET, afec_ixr);
    }
#endif
}

/****************************************************************************
 * Name: afec_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void afec_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;

  /* Shutdown the ADC device only when not in use */

  priv->initialized--;

  if (priv->initialized > 0)
    {
      return;
    }

  /* Disable ADC interrupts */

  up_disable_irq(priv->irq);

  /* Then detach the ADC interrupt handler. */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: afec_ioctl
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
 *   OK in success or error value
 *
 ****************************************************************************/

static int afec_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          afec_putreg(priv, SAM_AFEC_CR_OFFSET, AFEC_CR_START);
        }
        break;
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
 * Name: afec_interrupt
 *
 * Description:
 *   ADC interrupt handler
 *
 ****************************************************************************/

static int afec_interrupt(int irq, void *context, FAR void *arg)
{
  FAR struct adc_dev_s *dev = (FAR struct adc_dev_s *)arg;
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  int32_t data;

  if ((afec_getreg(priv, SAM_AFEC_ISR_OFFSET) & \
                   AFEC_CH(priv->chanlist[priv->current])) != 0)
    {
      /* Read data */

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
 *   intf      - ADC number (0 or 1)
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

#ifdef CONFIG_SAMV7_AFEC_DMA
  priv->nsamples = priv->nchannels * CONFIG_SAMV7_AFEC_DMASAMPLES;
  priv->dma = sam_dmachannel(1, DMA_FLAGS);
#endif

  ainfo("intf: %d nchannels: %d\n", priv->intf, priv->nchannels);

  return dev;
}

#endif /* CONFIG_SAMV7_AFEC0 || CONFIG_SAMV7_AFEC1 */

#endif /* CONFIG_ADC */
