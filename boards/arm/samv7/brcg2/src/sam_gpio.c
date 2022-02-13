/****************************************************************************
 * boards/arm/samv7/brcg2/src/sam_gpio.c
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

#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "chip.h"
#include "sam_gpio.h"
#include "brcg2.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct samgpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static int gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value);
#endif

#if BOARD_NGPIOOUT > 0
static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int gpout_write(FAR struct gpio_dev_s *dev, bool value);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static const struct gpio_operations_s gpin_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};
#endif


#if BOARD_NGPIOOUT > 0
static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};
#endif

#if BOARD_NGPIOIN > 0
/* This array maps the GPIO pins used as INPUT */

static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
  GPIO_ADDR0,
  GPIO_ADDR1,
  GPIO_ADDR2,
  GPIO_ADDR3,
  GPIO_ADDR4,
  GPIO_ADDR5,
  GPIO_ADDR6,
  GPIO_ADDR7,
};

static struct samgpio_dev_s g_gpin[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOOUT > 0
/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_WARN,
};

static struct samgpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static int gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct samgpio_dev_s *samgpio =
                        (FAR struct samgpio_dev_s *)dev;

  DEBUGASSERT(samgpio != NULL && value != NULL);
  DEBUGASSERT(samgpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading...\n");

  *value = sam_gpioread(g_gpioinputs[samgpio->id]);
  return OK;
}
#endif

#if BOARD_NGPIOOUT > 0
static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct samgpio_dev_s *samgpio =
                        (FAR struct samgpio_dev_s *)dev;

  DEBUGASSERT(samgpio != NULL && value != NULL);
  DEBUGASSERT(samgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = sam_gpioread(g_gpiooutputs[samgpio->id]);
  return OK;
}

static int gpout_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct samgpio_dev_s *samgpio =
                             (FAR struct samgpio_dev_s *)dev;

  DEBUGASSERT(samgpio != NULL);
  DEBUGASSERT(samgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  sam_gpiowrite(g_gpiooutputs[samgpio->id], value);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int sam_gpio_initialize(void)
{
  int i;
  int pincount = 0;

#if BOARD_NGPIOIN > 0
  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register(&g_gpin[i].gpio, pincount);

      /* Configure the pin that will be used as input */

      sam_configgpio(g_gpioinputs[i]);

      pincount++;
    }
#endif

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register(&g_gpout[i].gpio, pincount);

      /* Configure the pin that will be used as output */

      sam_gpiowrite(g_gpiooutputs[i], 0);
      sam_configgpio(g_gpiooutputs[i]);

      pincount++;
    }
#endif

  return 0;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
