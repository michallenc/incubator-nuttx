/****************************************************************************
 * boards/arm/samv7/p-an2c/src/sam_enc.c
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

#include "p-an2c.h"

#ifdef CONFIG_SENSORS_QENCODER

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_qeconfig_s
{
  int32_t  position;
  int32_t  phase;
};

struct sam_enc_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the lower-
   * half callback structure:
   */

  FAR const struct qe_ops_s *ops;             /* Lower half callback structure */
  FAR const struct sam_qeconfig_s *config;  /* static configuration */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int board_enc_irqx(gpio_pinset_t pinset, int irq,
                             xcpt_t irqhandler, void *arg);
static int sam_enc_interrupt(int irq, FAR void *context,
                                     FAR void *arg);
static int sam_position(FAR struct qe_lowerhalf_s *lower, FAR int32_t *pos);
static int sam_setup(FAR struct qe_lowerhalf_s *lower);
static int sam_shutdown(FAR struct qe_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

const int irc_phase_increment[16]= {
  /*0*/0,1,-1,0,
  /*4*/-1,0,0,1,
  /*8*/1,0,0,-1,
  /*12*/0,-1,1,0
};

const uint32_t enc_pins[NUM_BUTTONS]     =
                                            {
                                              GPIO_ENC_A,
                                              GPIO_ENC_B
                                            };
const uint32_t enc_pins_int[NUM_BUTTONS] =
                                            {
                                              GPIO_ENC_A_INT,
                                              GPIO_ENC_B_INT,
                                            };


static const struct qe_ops_s g_qecallbacks =
{
  .setup     = sam_setup,
  .shutdown  = sam_shutdown,
  .position  = sam_position,
  .setposmax = NULL,
  .reset     = NULL,
  .ioctl     = NULL,
};

static const struct sam_qeconfig_s sam_enc_config =
{
  .position = 0,
  .phase = 0,
};

static struct sam_enc_lowerhalf_s sam_enc_priv =
{
  .ops = &g_qecallbacks,
  .config = &sam_enc_config,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_enc_irqx
 *
 * Description:
 *   This function implements the core of the board_button_irq() logic.
 *
 ****************************************************************************/

static int board_enc_irqx(gpio_pinset_t pinset, int irq,
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

static int sam_enc_interrupt(int irq, FAR void *context,
                                     FAR void *arg)
{
  FAR struct sam_enc_lowerhalf_s *dev = (FAR struct sam_enc_lowerhalf_s *)arg;
  FAR struct sam_qeconfig_s *priv = (struct sam_qeconfig_s *)dev->config;

  unsigned int stateA;
  unsigned int stateB;
  int32_t incr;
  int32_t phase;

  stateA = sam_gpioread(enc_pins[0]);
  stateB = sam_gpioread(enc_pins[1]);

  phase = (stateA << 1) | (stateB << 0);  

  incr = irc_phase_increment[priv->phase | (phase*4)];
  priv->position+=incr;
  priv->phase=phase;

  return OK;
}

/****************************************************************************
 * Name: sam_position
 *
 * Description:
 *   Return the current position measurement.
 *
 ****************************************************************************/

static int sam_position(FAR struct qe_lowerhalf_s *lower, FAR int32_t *pos)
{
  FAR struct sam_enc_lowerhalf_s *priv =
    (FAR struct sam_enc_lowerhalf_s *)lower;
  FAR const struct sam_qeconfig_s *config = priv->config;

  *pos = (int32_t)config->position;
  return OK;
}

static int sam_setup(FAR struct qe_lowerhalf_s *lower)
{
  return OK;
}

static int sam_shutdown(FAR struct qe_lowerhalf_s *lower)
{
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

int sam_enc_setup(void)
{
  int i;
  int ret;

  struct sam_enc_lowerhalf_s * priv = (struct sam_enc_lowerhalf_s *)&sam_enc_priv;

  ret = qe_register("dev/qe0", (FAR struct qe_lowerhalf_s *)priv);
  if (ret < 0)
    {
      snerr("ERROR: qe_register failed: %d\n", ret);
      return -ERROR;
    }

  for (i = 0; i < 2; i++)
    {
      sam_configgpio(enc_pins[i]);
      ret = board_enc_irqx(enc_pins[i], enc_pins_int[i], sam_enc_interrupt, priv);
    }

  return OK;
}

#endif /* CONFIG_SENSORS_QENCODER */
