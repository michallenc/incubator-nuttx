/****************************************************************************
 * boards/arm/sama5/sama5d3-xplained/src/sam_usb.c
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
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "arm_internal.h"
#include "sam_usbhosths.h"
#include "hardware/sam_usbhs.h"
#include "samv71-xult.h"

#if defined(CONFIG_SAMV7_USBHOSTHS) && defined(HAVE_USBHOST)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SAMV71XULT_USBHOST_PRIO
#  define CONFIG_SAMV71XULT_USBHOST_PRIO 50
#endif

#ifndef CONFIG_SAMV71XULT_USBHOST_STACKSIZE
#  define CONFIG_SAMV71XULT_USBHOST_STACKSIZE 1024
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Retained device driver handles */

static struct usbhost_connection_s *g_hostconn;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_waiter
 *
 * Description:
 *   Wait for USB devices to be connected to either the OHCI or EHCI hub.
 *
 ****************************************************************************/

static int usbhost_waiter(int argc, char *argv[])
{
  struct usbhost_connection_s *dev = g_hostconn;
  struct usbhost_hubport_s *hport;

  uinfo("Running\n");
  for (; ; )
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(dev, &hport));
      uinfo("%s\n", hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

          CONN_ENUMERATE(dev, hport);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ****************************************************************************/

int sam_usbhost_initialize(void)
{
  int ret;

#ifdef CONFIG_USBHOST_MSC
  /* Register the USB host Mass Storage Class */

  ret = usbhost_msc_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

  /* Get an instance of the USB HS interface */

  g_hostconn = sam_usbhosths_initialize(0);
  if (!g_hostconn)
    {
      uerr("ERROR: sam_usbhosths_initialize failed\n");
      return -ENODEV;
    }

  /* Start a thread to handle device connection. */

  ret = kthread_create("USBHost Monitor", CONFIG_SAMV71XULT_USBHOST_PRIO,
                       CONFIG_SAMV71XULT_USBHOST_STACKSIZE,
                       usbhost_waiter, NULL);
  if (ret < 0)
    {
      uerr("ERROR: Failed to create ehci_waiter task: %d\n", ret);
      return -ENODEV;
    }

  return OK;
}

#endif /* defined(CONFIG_SAMV7_USBHOSTHS) && defined(HAVE_USBHOST) */
