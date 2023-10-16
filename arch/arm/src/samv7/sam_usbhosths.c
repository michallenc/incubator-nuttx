/****************************************************************************
 * arch/arm/src/samv7/sam_usbhost.c
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

#include <nuttx/config.h>

#include <sys/param.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "barriers.h"

#include "chip.h"
#include "sam_periphclks.h"
#include "hardware/sam_usbhs.h"
#include "sam_clockconfig.h"
#include "sam_usbhost.h"

#if defined(CONFIG_USBHOST) && defined(CONFIG_SAMV7_USBHOSTHS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_USBHOST_HUB
# error CONFIG_USBHOST_HUB not yet supported for SAMv7!
#endif

/* Buffers must be aligned to the cache line size */

#define DCACHE_LINEMASK (ARMV7M_DCACHE_LINESIZE - 1)

/* Configurable size of a request/descriptor buffers */

#ifndef CONFIG_SAMV7_USBHOST_BUFSIZE
#  define CONFIG_SAMV7_USBHOST_BUFSIZE 128
#endif

#define SAMV7_USBHOST_BUFSIZE \
  ((CONFIG_SAMV7_USBHOST_BUFSIZE + DCACHE_LINEMASK) & ~DCACHE_LINEMASK)


/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* State of an endpoint */

enum sam_epstate_e
{
                              /* --- All Endpoints --- */

  USBHS_EPSTATE_DISABLED = 0, /* Endpoint is disabled */
  USBHS_EPSTATE_STALLED,      /* Endpoint is stalled */
  USBHS_EPSTATE_IDLE,         /* Endpoint is idle */
  USBHS_EPSTATE_SENDING,      /* Endpoint is sending data */
  USBHS_EPSTATE_SENDING_DMA,  /* Endpoint is sending data via DMA */
  USBHS_EPSTATE_NBUSYBK,      /* Endpoint DMA complete, waiting for
                               * NBUSYBK==0
                               */
  USBHS_EPSTATE_RECEIVING,    /* Endpoint is receiving data */
                              /* --- Endpoint 0 Only --- */
  USBHS_EPSTATE_EP0DATAOUT,   /* Endpoint 0 is receiving SETUP OUT data */
  USBHS_EPSTATE_EP0STATUSIN,  /* Endpoint 0 is sending SETUP status */
  USBHS_EPSTATE_EP0ADDRESS    /* Address change is pending completion of
                               * status
                               */
};

/* This structure describes one endpoint. */

struct sam_epinfo_s
{
  uint8_t epno:7;              /* Endpoint number */
  uint8_t dirin:1;             /* 1:IN endpoint 0:OUT endpoint */
  uint8_t devaddr:7;           /* Device address */
  uint8_t toggle:1;            /* Next data toggle */
#ifndef CONFIG_USBHOST_INT_DISABLE
  uint8_t interval;            /* Polling interval */
#endif
  uint8_t status;              /* Retained token status bits */
  volatile bool iocwait;       /* TRUE: Thread is waiting for TX completion */
  uint16_t maxpacket:11;       /* Maximum packet size */
  uint16_t xfrtype:2;          /* See USB_EP_ATTR_XFER_* definitions
                                * in usb.h
                                */
  uint16_t speed:2;            /* See USB_*_SPEED definitions in ehci.h */
  int result;                  /* The result of the transfer */
  uint32_t xfrd;               /* On completion, will hold the number of
                                * bytes transferred
                                */
  sem_t iocsem;                /* Semaphore used to wait for transfer
                                * completion
                                */
#ifdef CONFIG_USBHOST_ASYNCH
  usbhost_asynch_t callback;   /* Transfer complete callback */
  void *arg;                   /* Argument that accompanies the callback */
#endif

  /* These fields are used in the split-transaction protocol. */

  uint8_t hubaddr;             /* USB device address of the high-speed
                                * hub below which a full/low-speed device
                                * is attached.
                                */
  uint8_t hubport;             /* The port on the above high-speed hub. */
};

struct sam_rhport_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to struct sam_rhport_s.
   */

  struct usbhost_driver_s drvr;

  /* Root hub port status */

  volatile bool connected;     /* Connected to device */
  struct sam_epinfo_s ep0;     /* EP0 endpoint info */

  /* This is the hub port description understood by class drivers */

  struct usbhost_roothubport_s hport;
};

struct sam_usbhosths_s
{
  uint32_t base;
  uint32_t irq;
  uint32_t ier;
  volatile bool pscwait;
  volatile bool resetwait;
  bool lowpower;
  struct work_s work;
  rmutex_t lock;
  sem_t resetsem;
  sem_t pscsem;

  /* Address generation data */

  struct usbhost_devaddr_s devgen;

  /* Root Hub port*/

  struct sam_rhport_s rhport;
}

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t sam_getreg(struct sam_usbhosths_s *priv,
                                  uint32_t offset);
static inline void sam_putreg(struct sam_usbhosths_s *priv, uint32_t offset,
                              uint32_t regval);

static int sam_usbhs_disconnect_device(struct sam_usbhosths_s *priv);
static int sam_usbhs_connect_device(struct sam_usbhosths_s *priv);
static int sam_usbhs_interrupt_top(int irq, void *context, void *arg);
static int sam_usbhs_interrupt_bottom(int irq, void *context, void *arg);

/* USB Host Controller Operations *******************************************/

static int sam_wait(struct usbhost_connection_s *conn,
         struct usbhost_hubport_s **hport);
static int sam_enumerate(struct usbhost_connection_s *conn,
         struct usbhost_hubport_s *hport);

static int sam_ep0configure(struct usbhost_driver_s *drvr,
         usbhost_ep_t ep0, uint8_t funcaddr, uint8_t speed,
         uint16_t maxpacketsize);
static int sam_epalloc(struct usbhost_driver_s *drvr,
                       const struct usbhost_epdesc_s *epdesc,
                       usbhost_ep_t *ep);
static int sam_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int sam_alloc(struct usbhost_driver_s *drvr,
         uint8_t **buffer, size_t *maxlen);
static int sam_free(struct usbhost_driver_s *drvr, uint8_t *buffer);
static int sam_ioalloc(struct usbhost_driver_s *drvr,
         uint8_t **buffer, size_t buflen);
static int sam_iofree(struct usbhost_driver_s *drvr,
         uint8_t *buffer);
static int sam_ctrlin(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
         const struct usb_ctrlreq_s *req, uint8_t *buffer);
static int sam_ctrlout(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
         const struct usb_ctrlreq_s *req, const uint8_t *buffer);
static ssize_t sam_transfer(struct usbhost_driver_s *drvr,
         usbhost_ep_t ep, uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static int sam_asynch(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
         uint8_t *buffer, size_t buflen, usbhost_asynch_t callback,
         void *arg);
#endif
static int sam_cancel(struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static void sam_disconnect(struct usbhost_driver_s *drvr,
                           struct usbhost_hubport_s *hport);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct usbhost_connection_s g_hostconn =
{
  .wait = sam_wait,
  .enumerate = sam_enumerate,
};

static struct sam_usbhosths_s g_usbhosths =
{
  .base = SAM_USBHS_BASE,
  .irq  = SAM_IRQ_USBHS,
  .ier  = 0,
#ifdef CONFIG_SAMV7_USBHOSTHS_LOWPOWER
  .lowpower = true,
#else
  .lowpower = false,
#endif
  .lock = NXRMUTEX_INITIALIZER,
  .pscsem = SEM_INITIALIZER(0),
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_getreg
 *
 * Description:
 *   Get the contents of an SAMV7 register
 *
 ****************************************************************************/

static inline uint32_t sam_getreg(struct sam_usbhosths_s *priv,
                                  uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: sam_putreg
 *
 * Description:
 *   Set the contents of an SAMV7 register to a value
 *
 ****************************************************************************/

static inline void sam_putreg(struct sam_usbhosths_s *priv, uint32_t offset,
                              uint32_t regval)
{
  putreg32(regval, priv->base + offset);
}

/****************************************************************************
 * Name: sam_usbhs_connect_device
 *
 * Description:
 ****************************************************************************/

static int sam_usbhs_connect_device(struct sam_usbhosths_s *priv)
{
  struct sam_rhport_s *rhport = &priv->rhport;
  uint32_t regval;

  if (rhport->connected)
    {
      /* TODO: error, received con interrupt but already connected. */
    }

  /* Clear device connected interrupt and disable it. */

  sam_putreg(priv, SAM_USBHS_HSTICR_OFFSET,
             USBHS_HSTINT_DCONNI | USBHS_HSTINT_DDISCI);

  priv->ier &= USBHS_HSTINT_DCONNI;
  sam_putreg(priv, SAM_USBHS_HSTIDR_OFFSET, USBHS_HSTINT_DCONNI);

  /* Enable device disconnect interrupt */

  priv->ier |= USBHS_HSTINT_DDISCI;
  sam_putreg(priv, SAM_USBHS_HSTIER_OFFSET, USBHS_HSTINT_DDISCI);

  /* Handle connected device. */

  regval = sam_getreg(priv, SAM_USBHS_HSTCTRL_OFFSET);
  regval |= USBHS_HSTCTRL_SOFE;
  sam_putreg(priv, SAM_USBHS_HSTCTRL_OFFSET, regval);

  /* TODO: enter sleep mode (?) */

  rhport->connected = true;

  /* Notify any waiters */

  if (priv->pscwait)
    {
      nxsem_post(&priv->pscsem);
      priv->pscwait = false;
    }

  return OK;
}

/****************************************************************************
 * Name: sam_usbhs_disconnect_device
 *
 * Description:
 ****************************************************************************/

static int sam_usbhs_disconnect_device(struct sam_usbhosths_s *priv)
{
  struct sam_rhport_s *rhport = &priv->rhport;
  uint32_t regval;

  if (!rhport->connected)
    {
      /* TODO: error, received dis interrupt but already disconnected. */
    }

  /* Clear device disconnected interrupt and disable it. */

  sam_putreg(priv, SAM_USBHS_HSTICR_OFFSET,
             USBHS_HSTINT_DCONNI | USBHS_HSTINT_DDISCI);

  priv->ier &= USBHS_HSTINT_DDISCI;
  sam_putreg(priv, SAM_USBHS_HSTIDR_OFFSET, USBHS_HSTINT_DDISCI);

  /* Enable device connected interrupt */

  priv->ier |= USBHS_HSTINT_DCONNI;
  sam_putreg(priv, SAM_USBHS_HSTIER_OFFSET, USBHS_HSTINT_DCONNI);

  /* TODO: what next? */

  rhport->connected = false;

  /* Notify any waiters */

  if (priv->pscwait)
    {
      nxsem_post(&priv->pscsem);
      priv->pscwait = false;
    }

  return OK;
}

/****************************************************************************
 * Name: sam_usbhs_interrupt_bottom
 *
 * Description:
 ****************************************************************************/

static int sam_usbhs_interrupt_bottom(int irq, void *context, void *arg)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)arg;
  uint32_t pending = (uint32_t)context;
  uint32_t regval;

  /* Access should be exclusive -> lock mutex */

  nxrmutex_lock(priv->lock);

  if (pending & USBHS_HSTINT_HWUPI != 0)
    {
      /* TODO */
    }
  
  if (pending & USBHS_HSTINT_DCONNI != 0)
    {
      sam_usbhs_connect_device(priv);
    }

  if (pending & USBHS_HSTINT_DDISCI != 0)
    {
      sam_usbhs_disconnect_device(priv);
    }

  if (pending & USBHS_HSTINT_HSOFI != 0)
    {
      /* TODO */
    }
  
  if (pending & USBHS_HSTINT_RSTI)
    {
      /* Clear reset interrupt */

      sam_putreg(priv, SAM_USBHS_HSTICR_OFFSET, USBHS_HSTINT_RSTI);

      /* And release any thread waiting for reset. */

      if (priv->resetwait)
        {
          nxsem_post(&priv->resetsem);
        }
    }

  if (pending)
    {
      /* TODO: we went throught interrupts but still some are pending */
    }

  nxrmutex_unlock(priv->lock);
}

/****************************************************************************
 * Name: sam_usbhs_interrupt_top
 *
 * Description:
 ****************************************************************************/

static int sam_usbhs_interrupt_top(int irq, void *context, void *arg)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)arg;
  uint32_t enabled;
  uint32_t regval;
  uint32_t pending;

  enabled = priv->ier;
  regval  = sam_getreg(priv, SAM_USBHS_HSTISR_OFFSET);
  pending = regval & enabled;

  if (pending != 0)
    {
      /* There are some pending enabled interrupts. Interrupt handling
       * should be scheduled for a higher priority worker thread.
       */

      work_available(priv->work);
      work_queue(HPWORK, priv->work, sam_usbhs_interrupt_bottom,
                 (void *)pending, priv);
    }

  return OK;
}

/****************************************************************************
 * Name: sam_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a hub port.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from the
 *      call to the USB driver initialization logic.
 *   hport - The location to return the hub port descriptor that detected the
 *      connection related event.
 *
 * Returned Value:
 *   Zero (OK) is returned on success when a device is connected or
 *   disconnected. This function will not return until either (1) a device is
 *   connected or disconnect to/from any hub port or until (2) some failure
 *   occurs.  On a failure, a negated errno value is returned indicating the
 *   nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_wait(struct usbhost_connection_s *conn,
                    struct usbhost_hubport_s **hport)
{
  struct sam_usbhosths_s *priv = &g_usbhosths;
  struct sam_rhport_s *rhport = &priv->rhport;
  struct usbhost_hubport_s *connport = &rhport->hport.hport;
  irqstate_t flags;
  int rhpndx;
  int ret;

  /* Loop until the connection state changes on one of the oot hub ports or
   * until an error occurs.
   */

  flags = enter_critical_section();
  for (; ; )
    {
      if (rhport->connected != connport->connected)
        {
          connport->connected = rhport->connected;
          *hport = connport;
          leave_critical_section(flags);
          return OK;
        }

      /* No changes on the port. Wait for a connection/disconnection event
       * and check again
       */

      priv->pscwait = true;
      ret = nxsem_wait_uninterruptible(&priv->pscsem);
      if (ret < 0)
        {
          return ret;
        }
    }
}

/****************************************************************************
 * Name: sam_enumerate
 *
 * Description:
 *   Enumerate the connected device.  As part of this enumeration process,
 *   the driver will (1) get the device's configuration descriptor, (2)
 *   extract the class ID info from the configuration descriptor, (3) call
 *   usbhost_findclass() to find the class that supports this device, (4)
 *   call the create() method on the struct usbhost_registry_s interface
 *   to get a class instance, and finally (5) call the connect() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   conn  - The USB host connection instance obtained as a parameter from
 *           the call to the USB driver initialization logic.
 *   hport - The descriptor of the hub port that has the newly connected
 *           device.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_enumerate(struct usbhost_connection_s *conn,
                         struct usbhost_hubport_s *hport)
{
  struct sam_usbhosths_s *priv = &g_usbhosths;
  uint32_t regval;
  uint8_t
  int ret;

  DEBUGASSERT(hport);

  /* Get info about speed and configuration */

  /* Step 1: reset USB line */

  priv->resetwait = true;

  regval = sam_getreg(priv, SAM_USBHS_HSTCTRL_OFFSET);
  regval |= USBHS_HSTCTRL_RESET;
  sam_putreg(priv, SAM_USBHS_HSTCTRL_OFFSET, regval);

  ret = nxsem_wait(&priv->resetsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Step 2: Get device speed */

  //nxsig_usleep(100 * 1000);

  regval = sam_getreg(priv, SAM_USBHS_SR_OFFSET);
  speed = (regval & USBHS_SR_SPEED_MASK) >> USBHS_SR_SPEED_SHIFT;
  switch (speed)
    {
      case USBHS_SR_SPEED_FULL:
        hport->speed = USB_SPEED_FULL;
        break;
      case USBHS_SR_SPEED_HIGH:
        hport->speed = USB_SPEED_HIGH;
        break;
      case USBHS_SR_SPEED_LOW:
        hport->speed = USB_SPEED_LOW;
        break;
    }

  /* Then let the common usbhost_enumerate do the real enumeration. */

  //usbhost_vtrace1(EHCI_VTRACE1_CLASSENUM, HPORT(hport));
  ret = usbhost_enumerate(hport, &hport->devclass);
  if (ret < 0)
    {
      /* Failed to enumerate */

      //usbhost_trace2(EHCI_TRACE2_CLASSENUM_FAILED, HPORT(hport), -ret);

      /* If this is a root hub port, then marking the hub port not connected
       * will cause sam_wait() to return and we will try the connection
       * again.
       */

      hport->connected = false;
    }

  return ret;
}

/****************************************************************************
 * Name: sam_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support
 *   an external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   funcaddr - The USB address of the function containing the endpoint that
 *     EP0 controls.  A funcaddr of zero will be received if no address is
 *     yet assigned to the device.
 *   speed - The speed of the port USB_SPEED_LOW, _FULL, or _HIGH
 *   maxpacketsize - The maximum number of bytes that can be sent to or
 *    received from the endpoint in a single data packet
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_ep0configure(struct usbhost_driver_s *drvr,
                            usbhost_ep_t ep0, uint8_t funcaddr,
                            uint8_t speed, uint16_t maxpacketsize)
{
  struct sam_epinfo_s *epinfo = (struct sam_epinfo_s *)ep0;
  struct sam_usbhosths_s *priv = &g_usbhosths;
  int ret;

  //usbhost_vtrace2(EHCI_VTRACE2_EP0CONFIG, speed, funcaddr);
  DEBUGASSERT(drvr != NULL && epinfo != NULL && maxpacketsize < 2048);

  /* We must have exclusive access to  data structures. */

  ret = nxrmutex_lock(&priv->lock);
  if (ret >= 0)
    {
      /* Remember the new device address and max packet size */

      epinfo->devaddr   = funcaddr;
      epinfo->speed     = speed;
      epinfo->maxpacket = maxpacketsize;

      nxrmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: sam_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to allocate the request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_malloc().
 *
 *   This interface was optimized under a particular assumption.  It was
 *   assumed that the driver maintains a pool of small, pre-allocated buffers
 *   for descriptor traffic.  NOTE that size is not an input, but an output:
 *   The size of the pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *     which to return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in
 *     which to return the maximum size of the allocated buffer memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_alloc(struct usbhost_driver_s *drvr,
                     uint8_t **buffer, size_t *maxlen)
{
  int ret = -ENOMEM;
  DEBUGASSERT(drvr && buffer && maxlen);

  /* The only special requirements for transfer/descriptor buffers are that
   * (1) they be aligned to a cache line boundary and (2) they are a
   * multiple of the cache line size in length.
   */

  *buffer = (uint8_t *)
    kmm_memalign(ARMV7M_DCACHE_LINESIZE, SAMV7_USBHOST_BUFSIZE);
  if (*buffer)
    {
      *maxlen = SAMV7_USBHOST_BUFSIZE;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: sam_sw_initialize
 *
 * Description:
 *   TODO
 *
 ****************************************************************************/

static void sam_sw_initialize(struct sam_usbhosths_s *priv)
{
  struct sam_rhport_s *rhport = &priv->rhport;

  usbhost_devaddr_initialize(&priv->devgen);

  /* Initialize the device operations */

  rhport->drvr.ep0configure   = sam_ep0configure;
  rhport->drvr.epalloc        = sam_epalloc;
  rhport->drvr.epfree         = sam_epfree;
  rhport->drvr.alloc          = sam_alloc;
  rhport->drvr.free           = sam_free;
  rhport->drvr.ioalloc        = sam_ioalloc;
  rhport->drvr.iofree         = sam_iofree;
  rhport->drvr.ctrlin         = sam_ctrlin;
  rhport->drvr.ctrlout        = sam_ctrlout;
  rhport->drvr.transfer       = sam_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
  rhport->drvr.asynch         = sam_asynch;
#endif
  rhport->drvr.cancel         = sam_cancel;
  rhport->drvr.disconnect     = sam_disconnect;
  rhport->hport.pdevgen       = &priv->devgen;

  rhport->connected           = false;

  /* Initialize EP0 */

  rhport->ep0.xfrtype         = USB_EP_ATTR_XFER_CONTROL;
  rhport->ep0.speed           = USB_SPEED_FULL;
  rhport->ep0.maxpacket       = 8;
  nxsem_init(&rhport->ep0.iocsem, 0, 0);

  /* Initialize the public port representation */

  hport                       = &rhport->hport.hport;
  hport->drvr                 = &rhport->drvr;
#ifdef CONFIG_USBHOST_HUB
  hport->parent               = NULL;
#endif
  hport->ep0                  = &rhport->ep0;
  hport->port                 = i;
  hport->speed                = USB_SPEED_FULL;
}

/****************************************************************************
 * Name: sam_hw_initialize
 *
 * Description:
 *   TODO
 *
 ****************************************************************************/

static void sam_hw_initialize(struct sam_usbhosths_s *priv)
{
  uint32_t regval;

  sam_usbhs_enableclk();

  /* Disable USB controller. */

  regval  = sam_getreg(priv, SAM_USBHS_CTRL_OFFSET);
  regval &= ~USBHS_CTRL_USBE;
  sam_putreg(priv, SAM_USBHS_CTRL_OFFSET, regval);

  /* Clear UIMOD bitfield and set host mode */

  regval &= ~USBHS_CTRL_UIMOD_MASK;
  regval |= USBHS_CTRL_UIMOD_HOST;
  sam_putreg(priv, SAM_USBHS_CTRL_OFFSET, regval);

  /* Now enable USB peripheral */

  regval |= USBHS_CTRL_USBE;
  sam_putreg(priv, SAM_USBHS_CTRL_OFFSET, regval);

  /* Mode is selected by UIMOD bitfield */

  regval &= USBHS_CTRL_UIDE;
  sam_putreg(priv, SAM_USBHS_CTRL_OFFSET, regval);

  /* Set low power mode if required (only FS and LS available) in
   * that case.
   */

  regval = sam_getreg(priv, SAM_USBHS_HSTCTRL_OFFSET);
  regval &= USBHS_HSTCTRL_MASK;
  if (priv->lowpower)
    {
      regval |= USBHS_HSTCTRL_LOWPOWER;
    }
  else
    {
      regval |= USBHS_HSTCTRL_NORMAL;
    }

  sam_putreg(priv, SAM_USBHS_HSTCTRL_OFFSET, regval);
  
  /* Unfreeze clocking */

  regval = sam_getreg(priv, SAM_USBHS_CTRL_OFFSET);
  regval &= ~USBHS_CTRL_FRZCLK;
  sam_putreg(priv, SAM_USBHS_CTRL_OFFSET, regval);

  /* Clear interrupts */

  regval = USBHS_HSTINT_DCONNI | USBHS_HSTINT_DDISCI | USBHS_HSTINT_RSTI |
           USBHS_HSTINT_RSMEDI | USBHS_HSTINT_RXRSMI | USBHS_HSTINT_HSOFI |
           USBHS_HSTINT_HWUPI;
  sam_putreg(priv, SAM_USBHS_HSTICR_OFFSET, regval);

  /* Enable VBUS */

  regval = USBHS_SFR_VBUSRQS;
  sam_putreg(priv, SAM_USBHS_SFR_OFFSET, USBHS_SFR_VBUSRQS);

  /* Enable interrupts */

  priv->ier = USBHS_HSTINT_HWUPI | USBHS_HSTINT_DCONNI | USBHS_HSTINT_RSTI;
  sam_putreg(priv, SAM_USBHS_HSTIER_OFFSET, priv->ier);

  /* Enable the UPLL */

  sam_usbclock();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_usbhosths_initialize
 *
 * Description:
 *   Initialize USB HS host controller hardware.
 *
 * Input Parameters:
 *   controller - identifies which controller is to be initialized (so far
 *      SAMv7 has only one controller so this is always 1)
 *
 * Returned Value:
 *   And instance of the USB host interface.
 ****************************************************************************/

struct usbhost_connection_s *sam_usbhosths_initialize(int controller)
{
  struct usbhost_connection_s *hostconn = &g_hostconn;
  struct sam_usbhosths_s *priv = &g_usbhosths;
  int ret;

  DEBUGASSERT(controller == 0);

  /* Call sam_sw_initialize() */

  sam_sw_initialize(priv);

  /* Call sam_hw_initialize() to init HW peripheral */

  sam_hw_initialize(priv);

  /* Attach interrupt handler. */

  ret = irq_attach(priv->irq, sam_usbhs_interrupt, priv);
  if (ret != 0)
    {
      return NULL;
    }

  up_enable_irq(priv->irq);

  uinfo("USB Host controller %d initialized\n", controller);

  return hostconn;
}
