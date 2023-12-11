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

/* This structure describes one endpoint. */

struct sam_pipe_s
{
  struct usbhost_pipedesc_s *descb[3]; /* Pointers to this pipe descriptors */
  volatile uint8_t pipestate;          /* State of the pipe (see enum usb_h_pipe_state) */
  volatile uint8_t pipestatus;         /* Status of the pipe */
  volatile int8_t  pipestatus_general; /* Status of the pipe */
  volatile int8_t pipestate_general;
  int16_t result;                      /* The result of the transfer */
  uint32_t size;                       /* Expected transfer size */
  uint32_t count;                      /* Transfer count */
  uint8_t *data;                       /* Pointer to transfer data */
  int16_t pkt_timeout;                 /* Timeout between packets (500ms for data and 50ms for status), -1 if disabled */
  uint8_t zlp:1;                       /* Transfer ZLP support */

  uint8_t           stalled:1;    /* true: Endpoint is stalled */
  uint8_t           pending:1;    /* true: IN Endpoint stall is pending */
  uint8_t           halted:1;     /* true: Endpoint feature halted */
  uint8_t           zlpsent:1;    /* Zero length packet has been sent */
  uint8_t           txbusy:1;     /* Write request queue is busy (recursion avoidance kludge) */
  uint8_t           rxactive:1;   /* read request is active (for top of queue) */
  bool              inuse;           /* True: This pipe is "in use" */
  bool              in;              /* True: IN endpoint */
  uint8_t           idx;             /* Pipe index */
  uint8_t           epno;            /* Device endpoint number (0-127) */
  uint8_t           eptype;          /* See _EPTYPE_* definitions */
  uint8_t           funcaddr;        /* Device function address */
  uint8_t           speed;           /* Device speed */
  uint8_t           interval;        /* Interrupt/isochronous EP polling interval */
  uint16_t          maxpacket;       /* Max packet size */

  sem_t waitsem;               /* Channel wait semaphore */
  volatile bool waiter;        /* True: Thread is waiting for a channel event */

#ifdef CONFIG_USBHOST_ASYNCH
  usbhost_asynch_t callback;   /* Transfer complete callback */
  void *arg;                   /* Argument that accompanies the callback */
#endif
};

struct sam_usbhosths_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to struct sam_usbhosths_s.
   */

  struct usbhost_driver_s drvr;

  /* This is the hub port description understood by class drivers */

  struct usbhost_roothubport_s rhport;

  uint32_t base;
  uint32_t irq;
  uint32_t ier;

  /* Overall driver status */

  uint8_t           hoststate; /* State of the device (see enum sam_hoststate_e) */
  uint8_t           prevstate; /* Previous state of the device before SUSPEND */
  uint16_t          epavail;   /* Bitset of available endpoints */
  mutex_t           lock;      /* Support mutually exclusive access */
  bool              connected; /* Connected to device */
  bool              change;    /* Connection change */
  bool              pscwait;   /* True: Thread is waiting for a port event */
  bool              resetwait;
  bool              lowpower;
  uint8_t           smstate;   /* The state of the USB host state machine */
  uint8_t           irqset;    /* Set of enabled interrupts */
  uint8_t           xfrtype;   /* See enum _hxfrdn_e */
  sem_t             pscsem;    /* Semaphore to wait for a port event */
  sem_t resetsem;
  struct work_s work;

  uint16_t pipes_unfreeze; /* Pipes to unfreeze after wakeup */
  int8_t suspend_start;    /* Delayed suspend time in ms */
  int8_t resume_start;     /* Delayed resume time in ms */
  int8_t n_ctrl_req_user;  /* Control transfer request user count */
  int8_t n_sof_user;       /* SOF user count (callback, suspend, resume, ctrl request) */
  uint8_t pipe_pool_size;  /* Pipe pool size in number of pipes */

  /* Address generation data */

  struct usbhost_devaddr_s devgen;

  /* The pipe list */

  aligned_data(4)
  struct sam_pipe_s pipelist[SAM_USBHS_NENDPOINTS];

  /* Pipe descriptors 2 banks for each pipe */

  /* CTRL */

  usbhost_ep_t ep0; /* Root hub port EP0 description */
  aligned_data(4) uint8_t ctrl_buffer[SAM_EP0_MAXPACKET];
}

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t sam_getreg(struct sam_usbhosths_s *priv,
                                  uint32_t offset);
static inline void sam_putreg(struct sam_usbhosths_s *priv, uint32_t offset,
                              uint32_t regval);

static inline void sam_add_sof_user(struct sam_usbhosths_s *priv);

static int sam_usbhs_disconnect_device(struct sam_usbhosths_s *priv);
static int sam_usbhs_connect_device(struct sam_usbhosths_s *priv);
static int sam_usbhs_interrupt_top(int irq, void *context, void *arg);
static int sam_usbhs_interrupt_bottom(int irq, void *context, void *arg);

/* Control transfers */

static int  sam_ctrl_sendsetup(struct sam_usbhosths_s *priv,
                               struct sam_pipe_s *pipe,
                               const struct usb_ctrlreq_s *req);
static int  sam_ctrl_senddata(struct sam_usbhosths_s *priv,
                              struct sam_pipe_s *pipe,
                              uint8_t *buffer, unsigned int buflen);
static int  sam_ctrl_recvdata(struct sam_usbhosths_s *priv,
                              struct sam_pipe_s *pipe,
                              uint8_t *buffer, unsigned int buflen);
static int  sam_in_setup(struct sam_usbhosths_s *priv,
                         struct sam_pipe_s *pipe);
static int  sam_out_setup(struct sam_usbhosths_s *priv,
                          struct sam_pipe_s *pipe);

/* USB Host Controller Operations *******************************************/

static int sam_wait(struct usbhost_connection_s *conn,
         struct usbhost_hubport_s **hport);
static int sam_ioc_setup(struct sam_rhport_s *rhport,
         struct sam_epinfo_s *epinfo);
static int sam_ioc_wait(struct sam_epinfo_s *epinfo);
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
 * Name: sam_add_sof_user
 *
 * Description:
 *   Add one SOF IRQ user and enable SOF interrupt
 *
 ****************************************************************************/

static inline void sam_add_sof_user(struct sam_usbhosths_s *priv)
{
  priv->n_sof_user++;
  sam_putreg16(USBHS_HSTINT_HSOFI, SAM_USBHS_HSTIER_OFFSET);
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

  nxrmutex_lock(&priv->lock);

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
 * Name: sam_ioc_setup
 *
 * Description:
 *   Set the request for the IOC event well BEFORE enabling the transfer (as
 *   soon as we are absolutely committed to the to avoid transfer).  We do
 *   this to minimize race conditions.  This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 * Assumption:  The caller holds tex EHCI lock
 *
 ****************************************************************************/

static int sam_ioc_setup(struct sam_rhport_s *rhport,
                         struct sam_epinfo_s *epinfo)
{
  irqstate_t flags;
  int ret = -ENODEV;

  DEBUGASSERT(rhport && epinfo && !epinfo->iocwait);
#ifdef CONFIG_USBHOST_ASYNCH
  DEBUGASSERT(epinfo->callback == NULL);
#endif

  /* Is the device still connected? */

  flags = enter_critical_section();
  if (rhport->connected)
    {
      /* Then set iocwait to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer
       * completed.
       */

      epinfo->iocwait  = true;   /* We want to be awakened by IOC interrupt */
      epinfo->status   = 0;      /* No status yet */
      epinfo->xfrd     = 0;      /* Nothing transferred yet */
      epinfo->result   = -EBUSY; /* Transfer in progress */
#ifdef CONFIG_USBHOST_ASYNCH
      epinfo->callback = NULL;   /* No asynchronous callback */
      epinfo->arg      = NULL;
#endif
      ret              = OK;     /* We are good to go */
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: sam_ioc_wait
 *
 * Description:
 *   Wait for the IOC event.
 *
 * Assumption:  The caller does *NOT* hold the EHCI lock.  That would
 * cause a deadlock when the bottom-half, worker thread needs to take the
 * semaphore.
 *
 ****************************************************************************/

static int sam_ioc_wait(struct sam_epinfo_s *epinfo)
{
  int ret = OK;

  /* Wait for the IOC event.  Loop to handle any false alarm semaphore
   * counts.  Return an error if the task is canceled.
   */

  while (epinfo->iocwait)
    {
      ret = nxsem_wait_uninterruptible(&epinfo->iocsem);
      if (ret < 0)
        {
          break;
        }
    }

  return ret < 0 ? ret : epinfo->result;
}

/****************************************************************************
 * Name: sam_pipe_waitsetup
 *
 * Description:
 *   Set the request for the transfer complete event well
 *   BEFORE enabling the transfer (as soon as we are
 *   absolutely committed to the to avoid transfer).
 *   We do this to minimize race conditions.
 *   This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 * Assumptions:
 *  Called from a normal thread context BEFORE the transfer has been started.
 *
 ****************************************************************************/

static int sam_pipe_waitsetup(struct sam_usbhosths_s *priv,
                              struct sam_pipe_s *pipe)
{
  irqstate_t flags = enter_critical_section();
  int        ret   = -ENODEV;

  DEBUGASSERT(priv != NULL && pipe != NULL);

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set waiter to indicate that we expect
       * to be informed when either (1) the device is disconnected,
       * or (2) the transfer completed.
       */

      pipe->waiter   = true;
  #ifdef CONFIG_USBHOST_ASYNCH
      pipe->callback = NULL;
      pipe->arg      = NULL;
  #endif
      ret            = OK;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: sam_ctrl_sendsetup
 *
 * Description:
 *   Send an IN/OUT SETUP packet.
 *
 * Assumptions:
 *   This function is called only from the CTRLIN and CTRLOUT interfaces.
 *
 ****************************************************************************/

static int  sam_ctrl_sendsetup(struct sam_usbhosths_s *priv,
                               struct sam_pipe_s *pipe,
                               const struct usb_ctrlreq_s *req);
{
  clock_t start;
  clock_t elapsed;
  int ret;
  int i;

  /* Loop while the device reports NAK (and a timeout is not exceeded */

  start = clock_systime_ticks();
  do
    {
      /* Send the SETUP packet */

      pipe->data = (uint8_t *)req;
      pipe->size = USB_SIZEOF_CTRLREQ;
      pipe->count = 0;
      pipe->result = EBUSY;
      uinfo("pipe%d buffer:%p buflen:%d\n",
                                 pipe->idx,
                                 pipe->data,
                                 pipe->size);
      sam_pktdump("sam_ctrl_sendsetup", pipe->data, pipe->size);

      /* Set up for the wait BEFORE starting the transfer */

      ret = sam_pipe_waitsetup(priv, pipe);
      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_DEVDISCONN2, 0);
          return ret;
        }

      pipe->pipestate_general = USB_H_PIPE_S_SETUP;
      sam_add_sof_user(priv);
      priv->n_ctrl_req_user++;

      /* Make sure the peripheral address is correct */

      pipe->descb[0]->ctrlpipe &= ~USBHOST_CTRLPIPE_PDADDR_MASK;
      pipe->descb[0]->ctrlpipe |= USBHOST_CTRLPIPE_PDADDR(pipe->funcaddr);

      /* Write packet */

      sam_putreg8(USBHOST_PINTFLAG_TXSTP, SAM_USBHOST_PINTFLAG(pipe->idx));
      for (i = 0; i < USB_SIZEOF_CTRLREQ; i++)
          priv->ctrl_buffer[i] = pipe->data[i];

      pipe->descb[0]->addr = (uint32_t)pipe->data;
      pipe->descb[0]->pktsize &= ~(USBHOST_PKTSIZE_MPKTSIZE_MASK |
                                   USBHOST_PKTSIZE_BCNT_MASK);
      pipe->descb[0]->pktsize |= USBHOST_PKTSIZE_BCNT(USB_SIZEOF_CTRLREQ);

      pipe->descb[0]->ctrlpipe = USBHOST_CTRLPIPE_PDADDR(pipe->funcaddr) |
                      USBHOST_CTRLPIPE_PEPNUM(pipe->epno & USB_EPNO_MASK);
      uinfo("pipe%d pktsize=0x%x ctrl=0x%x status=0x%x\n",
                                    pipe->idx,
                                    pipe->descb[0]->pktsize,
                                    pipe->descb[0]->ctrlpipe,
                                    pipe->descb[0]->statuspipe);

      /* Send the SETUP token (always EP0) */

      sam_modifyreg8(USBHOST_PCFG_PTOKEN_MASK,
                     USBHOST_PCFG_PTOKEN_SETUP,
                     SAM_USBHOST_PCFG(pipe->idx));
      sam_putreg8(USBHOST_PSTATUS_DTGL, SAM_USBHOST_PSTATUSCLR(pipe->idx));
      sam_putreg8(USBHOST_PSTATUS_BK0RDY, SAM_USBHOST_PSTATUSSET(pipe->idx));
      sam_putreg8(USBHOST_PINTFLAG_TXSTP, SAM_USBHOST_PINTENSET(pipe->idx));
      sam_putreg8(USBHOST_PSTATUS_PFREEZE,
                  SAM_USBHOST_PSTATUSCLR(pipe->idx));

      /* Wait for the transfer to complete */

      ret = sam_pipe_wait(priv, pipe);

      /* Return on success and for all failures other than EAGAIN.  EAGAIN
       * means that the device NAKed the SETUP command and that we should
       * try a few more times.  NOTE:  The USB spec says that a peripheral
       * must always ACK a SETUP packet.
       */

      if (ret != -EAGAIN)
        {
          /* Output some debug information if the transfer failed */

          if (ret < 0)
            {
              usbhost_trace1(SAM_TRACE1_TRANSFER_FAILED2, ret);
            }

          /* Return the result in any event */

          return ret;
        }

      /* Get the elapsed time (in frames) */

      elapsed = clock_systime_ticks() - start;
    }
  while (elapsed < SAM_SETUP_DELAY);

  return -ETIMEDOUT;
}
static int  sam_ctrl_senddata(struct sam_usbhosths_s *priv,
                              struct sam_pipe_s *pipe,
                              uint8_t *buffer, unsigned int buflen);
static int  sam_ctrl_recvdata(struct sam_usbhosths_s *priv,
                              struct sam_pipe_s *pipe,
                              uint8_t *buffer, unsigned int buflen);
static int  sam_in_setup(struct sam_usbhosths_s *priv,
                         struct sam_pipe_s *pipe);
static int  sam_out_setup(struct sam_usbhosths_s *priv,
                          struct sam_pipe_s *pipe);

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
 * Name: sam_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_epalloc(struct usbhost_driver_s *drvr,
                       const struct usbhost_epdesc_s *epdesc,
                       usbhost_ep_t *ep)
{
  struct sam_epinfo_s *epinfo;
  struct usbhost_hubport_s *hport;

  /* Sanity check. */

  DEBUGASSERT(drvr != 0 && epdesc != NULL && epdesc->hport != NULL &&
              ep != NULL);
  hport = epdesc->hport;
  
  epinfo = kmm_zalloc(sizeof(struct sam_epinfo_s));
  if (!epinfo)
    {
      return -ENOMEM;
    }

  /* Initialize endpoint */
  
  epinfo->epno      = epdesc->addr;
  epinfo->dirin     = epdesc->in;
  epinfo->maxpacket = epdesc->mxpacketsize;
  epinfo->xfrtype   = epdesc->xfrtype;
  epinfo->speed     = hport->speed;
  nxsem_init(&epinfo->iocsem, 0, 0);

  *ep = (usbhost_ep_t)epinfo;
  return OK;
}

/****************************************************************************
 * Name: sam_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *           call to the class create() method.
 *   ep   - The endpint to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct sam_epinfo_s *epinfo = (struct sam_epinfo_s *)ep;

  /* There should not be any pending, transfers */

  DEBUGASSERT(drvr && epinfo && epinfo->iocwait == 0);

  /* Free the container */

  kmm_free(epinfo);
  return OK;
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
 * Name: sam_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to free that request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_free(struct usbhost_driver_s *drvr, uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

  /* No special action is require to free the transfer/descriptor buffer
   * memory
   */

  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: sam_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to
 *   allocate the request/descriptor memory.  If the underlying hardware
 *   does not support such "special" memory, this functions may simply map
 *   to kumm_malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are variable-
 *   sized.
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *            which to return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_ioalloc(struct usbhost_driver_s *drvr,
                       uint8_t **buffer, size_t buflen)
{
  DEBUGASSERT(drvr && buffer && buflen > 0);

  /* The only special requirements for I/O buffers are that (1) they be
   * aligned to a cache line boundary, (2) they are a multiple of the cache
   * line size in length, and (3) they might need to be user accessible
   * (depending on how the class driver implements its buffering).
   */

  buflen  = (buflen + DCACHE_LINEMASK) & ~DCACHE_LINEMASK;
  *buffer = kumm_memalign(ARMV7A_DCACHE_LINESIZE, buflen);
  return *buffer ? OK : -ENOMEM;
}

/****************************************************************************
 * Name: sam_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed
 *   more efficiently.  This method provides a mechanism to free that IO
 *   buffer memory.  If the underlying hardware does not support such
 *   "special" memory, this functions may simply map to kumm_free().
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_iofree(struct usbhost_driver_s *drvr, uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

  /* No special action is require to free the I/O buffer memory */

  kumm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: sam_ctrlin and sam_ctrlout
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one
 *   transfer may be queued; Neither these methods nor the transfer() method
 *   can be called again until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   ep0    - The control endpoint to send/receive the control request.
 *   req    - Describes the request to be sent.  This request must lie in
 *            memory created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *            responses.  This buffer must be large enough to hold the
 *            length value in the request description. buffer must have been
 *            allocated using DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same
 *   allocated memory.
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

static int sam_ctrlin(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                      const struct usb_ctrlreq_s *req,
                      uint8_t *buffer)
{
  struct sam_usbhosths_s *priv = (struct sam_usbhosths_s *)dvrv;
  struct sam_pipe_s *pipe;
  uint16_t buflen;
  ssize_t nbytes;
  clock_t start;
  clock_t elapsed;
  int retries;
  int ret;

  pipe = &priv->pipelist[(unsigned int)ep0];

#ifdef CONFIG_ENDIAN_BIG
  buflen = (uint16_t)req->len[0] << 8 | (uint16_t)req->len[1];
#else
  buflen = (uint16_t)req->len[1] << 8 | (uint16_t)req->len[0];
#endif

  /* Get exclusive access */
  
  ret = nxrmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < SAM_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request (TXSTP) */

      ret = sam_ctrl_sendsetup(priv, pipe, req);
      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_SENDSETUP_FAIL2, -ret);
          continue;
        }

      /* Get the start time.  Loop again until the timeout expires */

      start = clock_systime_ticks();
      do
        {
          /* Handle the IN data phase (if any) (TRCPT) */

          if (buflen > 0)
            {
              ret = sam_ctrl_recvdata(priv, pipe, buffer, buflen);
              if (ret < 0)
                {
                  usbhost_trace1(SAM_TRACE1_RECVDATA_FAIL, -ret);
                }
            }

          /* Handle the status OUT phase */

          if (ret == OK)
            {
              ret = sam_ctrl_senddata(priv, pipe, NULL, 0);
              if (ret == OK)
                {
                  /* All success transactions exit here */

                  nxmutex_unlock(&priv->lock);
                  return OK;
                }

              usbhost_trace1(SAM_TRACE1_SENDSTATUS_FAIL, ret < 0 ?
                                                       -ret : ret);
            }

          /* Get the elapsed time (in frames) */

          elapsed = clock_systime_ticks() - start;
        }
      while (elapsed < SAM_DATANAK_DELAY);
    }

  /* All failures exit here after all retries
   * and timeouts have been exhausted
   */

  nxmutex_unlock(&priv->lock);
  return -ETIMEDOUT;
}

static int sam_ctrlout(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                       const struct usb_ctrlreq_s *req,
                       const uint8_t *buffer)
{
  /* sam_ctrlin can handle both directions.  We just need to work around the
   * differences in the function signatures.
   */

  return sam_ctrlin(drvr, ep0, req, (uint8_t *)buffer);
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
