/****************************************************************************
 * boards/arm/samv7/brcg2/src/sam_enc.c
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

#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/board.h>
#include <nuttx/sensors/qencoder.h>
#include <arch/board/board.h>
#include <nuttx/irq.h>

#include "chip.h"
#include "arm_arch.h"
#include "sam_gpio.h"

#include "brcg2.h"

#ifdef CONFIG_SENSORS_QENCODER

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_qeconfig_s
{
  uint32_t  position;
  uint32_t  position_base;
  uint32_t  phase;
  uint32_t  error;
};

struct sam_gpio_enc_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the lower-
   * half callback structure:
   */

  FAR const struct qe_ops_s *ops;           /* Lower half callback structure */
  FAR const struct sam_qeconfig_s *config;  /* static configuration */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int board_gpio_enc_irqx(gpio_pinset_t pinset, int irq,
                             xcpt_t irqhandler, void *arg);
static int sam_gpio_enc_interrupt(int irq, FAR void *context,
                                     FAR void *arg);
static int sam_gpio_enc_position(FAR struct qe_lowerhalf_s *lower, FAR int32_t *pos);
static int sam_gpio_enc_setup(FAR struct qe_lowerhalf_s *lower);
static int sam_gpio_enc_shutdown(FAR struct qe_lowerhalf_s *lower);
static int sam_gpio_enc_reset(FAR struct qe_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

const uint32_t gpio_enc_pins[2] =
                                      {
                                        GPIO_ENC_A,
                                        GPIO_ENC_B
                                      };
const uint32_t gpio_enc_pins_int[2] =
                                      {
                                        GPIO_ENC_A_INT,
                                        GPIO_ENC_B_INT,
                                      };


static const struct qe_ops_s g_qecallbacks =
{
  .setup     = sam_gpio_enc_setup,
  .shutdown  = sam_gpio_enc_shutdown,
  .position  = sam_gpio_enc_position,
  .setposmax = NULL,
  .reset     = sam_gpio_enc_reset,
  .ioctl     = NULL,
};

static struct sam_qeconfig_s sam_gpio_enc_config =
{
  .position = 0,
  .position_base = 0,
  .error = 0,
};

static struct sam_gpio_enc_lowerhalf_s sam_gpio_enc_priv =
{
  .ops = &g_qecallbacks,
  .config = &sam_gpio_enc_config,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gpio_enc_irqx
 *
 * Description:
 *   This function implements the core of the board_button_irq() logic.
 *
 ****************************************************************************/

static int board_gpio_enc_irqx(gpio_pinset_t pinset, int irq,
                             xcpt_t irqhandler, void *arg)
{
  irqstate_t flags;

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();

  /* Are we attaching or detaching? */

  if (irqhandler != NULL)
    {

      /* Configure the interrupt */

      sam_gpioirq(pinset);
      irq_attach(irq, irqhandler, arg);
      sam_gpioirqenable(irq);
    }
  else
    {

      /* Detach and disable the interrupt */

      irq_detach(irq);
      sam_gpioirqdisable(irq);
    }

  leave_critical_section(flags);
  return OK;
}

static int sam_gpio_enc_interrupt(int irq, FAR void *context,
                                     FAR void *arg)
{
  FAR struct sam_gpio_enc_lowerhalf_s *dev = (FAR struct sam_gpio_enc_lowerhalf_s *)arg;
  FAR struct sam_qeconfig_s *priv = (struct sam_qeconfig_s *)dev->config;

  unsigned int stateA;
  unsigned int stateB;
  int32_t incr;
  uint32_t new;
  uint32_t incr_mask;

  /* Read the status of encoder pins */

  stateA = sam_gpioread(gpio_enc_pins[0]);
  stateB = sam_gpioread(gpio_enc_pins[1]);

  new = (stateB << 1 | (stateA^stateB));
  incr = ((new - priv->position + 1) & 3) - 1;
  incr_mask = (int32_t)((1 - incr) >> 2);

  /* Increment position */
  priv->position += incr & ~incr_mask;

  /* Count error */

  priv->error -= incr_mask;

  return OK;
}

/****************************************************************************
 * Name: sam_position
 *
 * Description:
 *   Return the current position measurement.
 *
 ****************************************************************************/

static int sam_gpio_enc_position(FAR struct qe_lowerhalf_s *lower, FAR int32_t *pos)
{
  FAR struct sam_gpio_enc_lowerhalf_s *priv =
    (FAR struct sam_gpio_enc_lowerhalf_s *)lower;
  FAR const struct sam_qeconfig_s *config = priv->config;

  *pos = config->position - config->position_base;
  return OK;
}

static int sam_gpio_enc_setup(FAR struct qe_lowerhalf_s *lower)
{
  return OK;
}

static int sam_gpio_enc_shutdown(FAR struct qe_lowerhalf_s *lower)
{
  return OK;
}

static int sam_gpio_enc_reset(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct sam_gpio_enc_lowerhalf_s *priv =
    (FAR struct sam_gpio_enc_lowerhalf_s *)lower;
  FAR struct sam_qeconfig_s *config = 
    (FAR struct sam_qeconfig_s *)priv->config;
  
  config->position = config->position_base;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_afecinitialize
 *
 * Description:
 *   Initialize and register the ADC driver.
 *
 ****************************************************************************/

int sam_gpio_enc_init(void)
{
  int i;
  int ret;

  struct sam_gpio_enc_lowerhalf_s * priv = (struct sam_gpio_enc_lowerhalf_s *)&sam_gpio_enc_priv;

  ret = qe_register("dev/gpio_enc", (FAR struct qe_lowerhalf_s *)priv);
  if (ret < 0)
    {
      snerr("ERROR: qe_register failed: %d\n", ret);
      return -ERROR;
    }

  for (i = 0; i < 2; i++)
    {
      sam_configgpio(gpio_enc_pins[i]);
      ret = board_gpio_enc_irqx(gpio_enc_pins[i], gpio_enc_pins_int[i], sam_gpio_enc_interrupt, priv);
    }

  return OK;
}

#endif /* CONFIG_SENSORS_QENCODER */
