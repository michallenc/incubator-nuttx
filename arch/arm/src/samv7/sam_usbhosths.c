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

#define SAM_RETRY_COUNT     3   /* Number of ctrl transfer retries */

#define USB_CTRL_DPKT_TIMEOUT (500)         /* Timeout between control data packets : 500ms */
#define USB_CTRL_STAT_TIMEOUT (50)          /* Timeout of status packet : 50ms */

#define SAM_SETUP_DELAY         SEC2TICK(5) /* 5 seconds in system ticks */
#define SAM_DATANAK_DELAY       SEC2TICK(5) /* 5 seconds in system ticks */

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes one endpoint. */

struct sam_pipe_s
{
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
  uint8_t           epno;            /* Device endpoint number (0-10) */
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

static void sam_pipe_interrupt(struct sam_usbhosths_s *priv, int idx);
static int sam_usbhs_interrupt_top(int irq, void *context, void *arg);
static int sam_usbhs_interrupt_bottom(int irq, void *context, void *arg);

/* OUT transfers */

static void sam_send_continue(struct sam_usbhosths_s *priv,
                              struct sam_pipe_s *pipe);
static void sam_send_start(struct sam_usbhosths_s *priv,
                           struct sam_pipe_s *pipe);
static ssize_t sam_out_transfer(struct sam_usbhosths_s *priv,
                                struct sam_pipe_s *pipe,
                                uint8_t *buffer,
                                size_t buflen);

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

/* IN transfers */

static void sam_recv_continue(struct sam_usbhosths_s *priv,
                              struct sam_pipe_s *pipe);
static void sam_recv_restart(struct sam_usbhosths_s *priv,
                             struct sam_pipe_s *pipe);
static void sam_recv_start(struct sam_usbhosths_s *priv,
                           struct sam_pipe_s *pipe);
static ssize_t sam_in_transfer(struct sam_usbhosths_s *priv,
                               struct sam_pipe_s *pipe,
                               uint8_t *buffer,
                               size_t buflen);

/* Pipe management */

static int sam_pipe_alloc(struct sam_usbhosths_s *priv);
static inline void sam_pipe_free(struct sam_usbhosths_s *priv,
              int idx);
static void sam_pipe_configure(struct sam_usbhosths_s *priv, int idx);
static int sam_pipe_waitsetup(struct sam_usbhosths_s *priv,
              struct sam_pipe_s *pipe);
static int sam_pipe_wait(struct sam_usbhosths_s *priv,
              struct sam_pipe_s *pipe);
static void sam_pipe_wakeup(struct sam_usbhosths_s *priv,
              struct sam_pipe_s *pipe);
static int sam_ctrlep_alloc(struct sam_usbhosths_s *priv,
                            const struct usbhost_epdesc_s *epdesc,
                            usbhost_ep_t *ep);
static int sam_xfrep_alloc(struct sam_usbhosths_s *priv,
                           const struct usbhost_epdesc_s *epdesc,
                           usbhost_ep_t *ep);

/* Control/data transfer logic */

static void sam_transfer_terminate(struct sam_usbhosths_s *priv,
              struct sam_pipe_s *pipe, int result);
static void sam_transfer_abort(struct sam_usbhosths_s *priv,
              struct sam_pipe_s *pipe, int result);

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
 * Name: sam_pipe_alloc
 *
 * Description:
 *   Allocate a pipe.
 *
 ****************************************************************************/

static int sam_pipe_alloc(struct sam_usbhosths_s *priv)
{
  int idx;

  /* Search the table of pipes */

  for (idx = 0; idx < SAM_USB_NENDPOINTS; idx++)
    {
      /* Is this pipe available? */

      if (!priv->pipelist[idx].inuse)
        {
          /* Yes... make it "in use" and return the index */

          priv->pipelist[idx].inuse = true;
          return idx;
        }
    }

  /* All of the pipes are "in-use" */

  return -EBUSY;
}

/****************************************************************************
 * Name: sam_pipe_free
 *
 * Description:
 *   Free a previoiusly allocated pipe.
 *
 ****************************************************************************/

static void sam_pipe_free(struct sam_usbhosths_s *priv, int idx)
{
  struct sam_pipe_s *pipe = &priv->pipelist[idx];

  uinfo("pipe%d\n", idx);
  DEBUGASSERT((unsigned)idx < SAM_USB_NENDPOINTS);

  /* Halt the pipe */

  //sam_putreg8(0, SAM_USBHOST_PCFG(pipe->idx));

  /* Mark the pipe available */

  priv->pipelist[idx].inuse = false;
}

static const uint16_t psize_2_size[] =
{
  8,
  16,
  32,
  64,
  128,
  256,
  512,
  1024
};

/****************************************************************************
 * Name: sam_get_psize
 *
 * Description:
 *   Convert bank size of bytes to PIPCFG.PSIZE -> size Size of bytes
 *
 ****************************************************************************/

uint8_t sam_get_psize(uint16_t size)
{
  uint8_t i;

  for (i = 0; i < sizeof(psize_2_size) / sizeof(uint16_t); i++)
    {
      /* Size should be exactly PSIZE values */

      if (size <= psize_2_size[i])
      return i;
    }

  return 7;
}

/****************************************************************************
 * Name: sam_usbhs_connect_device
 *
 * Description:
 ****************************************************************************/

static int sam_usbhs_connect_device(struct sam_usbhosths_s *priv)
{
  uint32_t regval;

  if (priv->connected)
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

  priv->connected = true;

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
  uint32_t regval;

  if (!priv->connected)
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

  priv->connected = false;

  /* Notify any waiters */

  if (priv->pscwait)
    {
      nxsem_post(&priv->pscsem);
      priv->pscwait = false;
    }

  return OK;
}

/****************************************************************************
 * Name: sam_pipe_interrupt
 *
 * Description:
 *   Handle the USB pipe interrupt
 *
 ****************************************************************************/

static void sam_pipe_interrupt(struct sam_usbhost_s *priv, int idx)
{
  struct sam_pipe_s *pipe;
  uint32_t pipisr;
  uint32_t pipimr;
  uint32_t pending;
  uint32_t regval;

  DEBUGASSERT((unsigned)idx < SAM_USB_NENDPOINTS);

  /* Get the pipe structure */

  pipe = &priv->pipelist[idx];

  /* Get the pipe irq */

  pipisr = sam_getreg(priv, SAM_USBHS_HSTPIPISR(idx));
  pipimr = sam_getreg(priv, SAM_USBHS_HSTPIPIMR(idx));

  pending = pipisr & pipimr;
  uinfo("pipe%d PINTFLAG:0x" PRIx32 ", PINTENSET:0x" PRIx32 ","
        "PENDING:0x" PRIx32 "\n", idx, pipisr, pipimr, pending);

  if (pending & USBHS_HSTPIPINT_PERRI)
    {
      /* First clear the flag */

      sam_putreg(priv, SAM_USBHS_HSTPIPIDR(idx), USBHS_HSTPIPINT_PERRI);

      /* Get info about error */

      regval = sam_getreg(priv, SAM_USBHS_HSTPIPERR(idx));
      switch (regval & (USBHS_HSTPIPERR_DATATGL | USBHS_HSTPIPERR_DATAPID |
                        USBHS_HSTPIPERR_PID | USBHS_HSTPIPERR_TIMEOUT))
        {
          case USBHS_HSTPIPERR_DATATGL:
            pipe->pipestatus_general = USB_H_ERR;
          break;

          case USBHS_HSTPIPERR_TIMEOUT:
            pipe->pipestatus_general = USB_H_TIMEOUT;
            pipe->result = -ETIMEDOUT;
          break;

          case USBHS_HSTPIPERR_PID:
          case USBHS_HSTPIPERR_DATAPID:
          default:
            pipe->pipestatus_general = USB_H_ERR;
          break;
        }

      sam_transfer_abort(priv, pipe, USB_H_ERR);
      return;
    }

  if (pending & USBHS_HSTPIPINT_RXINI)
    {
      sam_putreg(priv, SAM_USBHS_HSTPIPIDR(idx), USBHS_HSTPIPINT_RXINI);

      /* New data available. Check whether pipe is really in pipe */

      if ((sam_getreg(priv, SAM_USBHS_HSTPIPCFG(idx)) &
          USBHS_HSTPIPCFG_PTOKEN_MASK) == USBHS_HSTPIPCFG_PTOKEN_IN)
        {
          if (idx > 0)
            {
              sam_recv_continue(priv, pipe);
            }

          pipe->result = 0;
          sam_pipe_wakeup(priv, pipe);
        }

      return;
    }

  if (pending & UBSHS_HSPIPINT_TXOUTI)
    {
      sam_putreg(priv, SAM_USBHS_HSTPIPIDR(idx), UBSHS_HSPIPINT_TXOUTI);

      if ((sam_getreg(priv, SAM_USBHS_HSTPIPCFG(idx)) &
          USBHS_HSTPIPCFG_PTOKEN_MASK) == USBHS_HSTPIPCFG_PTOKEN_OUT)
        {
          if (idx > 0)
            {
              sam_send_continue(priv, pipe);
            }

          pipe->result = 0;
          sam_pipe_wakeup(priv, pipe);
        }
    }

  if (pending & UBSHS_HSPIPINT_TXSTPI)
    {
      sam_putreg(priv, SAM_USBHS_HSTPIPIDR(idx), UBSHS_HSPIPINT_TXSTPI);

      if (priv->ctrl_buffer[0] & 0x80) /* 1 = Device to Host */
        {
          /* IN */

          uwarn("pipe%d IN TXSTP\n", idx);
          pipe->pipestate = USB_H_PIPE_S_DATI; /* Pipe in data IN stage */

          /* Start IN requests */

          pipe->result = 0;
          sam_pipe_wakeup(priv, pipe);
        }
      else
        {
          /* OUT */

          uwarn("pipe%d OUT TXSTP\n", idx);
          if (priv->ctrl_buffer[6] || priv->ctrl_buffer[7]) /* setup packet wLength[2] */
            {
              pipe->pipestate = USB_H_PIPE_S_DATO; /* Pipe in data OUT stage */

              /* Start OUT */

              pipe->result = 0;
              sam_pipe_wakeup(priv, pipe);
            }
          else
            {
              /* No DATA phase */

              uwarn("pipe%d OUT TXSTP ZLP\n", idx);
              pipe->pipestate = USB_H_PIPE_S_STATI; /* Pipe in control status IN stage */

              /* Start IN ZLP request */

              pipe->result = 0;
              sam_pipe_wakeup(priv, pipe);
            }
        }

      return;
    }
}

/****************************************************************************
 * Name: sam_usbhs_interrupt_bottom
 *
 * Description:
 *  TODO
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
      /* Acknowledge interrupt */

      sam_putreg(priv, SAM_USBHS_HSTICR_OFFSET, USBHS_HSTINT_HSOFI);
    }

  if (pending & USBHS_HSTINT_RSTI != 0)
    {
      /* Clear reset interrupt */

      sam_putreg(priv, SAM_USBHS_HSTICR_OFFSET, USBHS_HSTINT_RSTI);

      /* And release any thread waiting for reset. */

      if (priv->resetwait)
        {
          nxsem_post(&priv->resetsem);
        }
    }

   if (pending & USBHS_HSTINT_PEP_MASK != 0)
      {
        /* Process each pending pipe interrupt */

        for (i = 0; i < SAM_USBHS_NENDPOINTS; i++)
          {
            /* Is there an interrupt pending for pipe i? */

            if ((pending & USBHS_HSTINT_PEP(i)) != 0)
              {
                /* Yes.. process the pipe i interrupt */

                sam_pipe_interrupt(priv, i);
              }
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
 *  TODO
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
  struct usbhost_hubport_s *connport = &priv->rhport.hport;
  irqstate_t flags;
  int rhpndx;
  int ret;

  /* Loop until the connection state changes on one of the oot hub ports or
   * until an error occurs.
   */

  flags = enter_critical_section();
  for (; ; )
    {
      if (priv->connected != connport->connected)
        {
          connport->connected = priv->connected;
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
 * Name: sam_pipe_configure
 *
 * Description:
 *   Configure or re-configure a host pipe.  Host pipes are configured
 *   when pipe is allocated and EP0 (only) is re-configured with the
 *   max packet size or device address changes.
 *
 ****************************************************************************/

static void sam_pipe_configure(struct sam_usbhosths_s *priv, int idx)
{
  struct sam_pipe_s *pipe = &priv->pipelist[idx];
  uint32_t regval;

  /* Enable pipe interrupts required for transfers on this pipe. */

  switch (pipe->eptype)
    {
    case USB_EP_ATTR_XFER_CONTROL:
    case USB_EP_ATTR_XFER_BULK:
      {
#ifdef HAVE_USBHOST_TRACE_VERBOSE
        uint16_t intrace;
        uint16_t outtrace;

        /* Determine the definitive trace ID to use below */

        if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL)
          {
            intrace  = SAM_VTRACE2_PIPECONF_CTRL_IN;
            outtrace = SAM_VTRACE2_PIPECONF_CTRL_OUT;
          }
        else
          {
            intrace  = SAM_VTRACE2_PIPECONF_BULK_IN;
            outtrace = SAM_VTRACE2_PIPECONF_BULK_OUT;
          }

        /* Interrupts required for CTRL and BULK endpoints */

        /* Additional setting for IN/OUT endpoints */

        if (pipe->in)
          {
            usbhost_vtrace2(intrace, idx, pipe->epno);
          }
        else
          {
            usbhost_vtrace2(outtrace, idx, pipe->epno);
          }
#endif
      }
      break;

    case USB_EP_ATTR_XFER_INT:
      {
        /* Interrupts required for INTR endpoints */

        /* Additional setting for IN endpoints */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
        if (pipe->in)
          {
            usbhost_vtrace2(SAM_VTRACE2_PIPECONF_INTR_IN, idx, pipe->epno);
          }
        else
          {
            usbhost_vtrace2(SAM_VTRACE2_PIPECONF_INTR_OUT, idx, pipe->epno);
          }
#endif
      }
      break;

    case USB_EP_ATTR_XFER_ISOC:
      {
        /* Interrupts required for ISOC endpoints */

        /* Additional setting for IN endpoints */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
        if (pipe->in)
          {
            usbhost_vtrace2(SAM_VTRACE2_PIPECONF_ISOC_IN, idx, pipe->epno);
          }
        else
          {
            usbhost_vtrace2(SAM_VTRACE2_PIPECONF_ISOC_OUT, idx, pipe->epno);
          }
#endif
      }
      break;
    }

  /* Pipe has to be enabled first */

  sam_putreg(priv, SAM_USBHS_HSTPIP, USBHS_HSTPIP_PEN(idx));

  /* Now we can configure the pipe */

  regval = USBHS_HSTPIPCFG_ALLOC | USBHS_HSTPIPCFG_PBK_1BANK | \
           USBHS_HSTPIPCFG_PSIZE_MASK(sam_get_psize(pipe->maxpacket));

  if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL)
    {
      regval |= USBHS_HSTPIPCFG_PTOKEN_SETUP;
    }
  else
    {
      regval |= pipe->in ? USBHS_HSTPIPCFG_PTOKEN_IN : USBHS_HSTPIPCFG_PTOKEN_OUT;
    }

  regval |= USBHS_HSTPIPCFG_PTYPE_MASK(pipe->eptype) | \
            USBHS_HSTPIPCFG_PEPNUM(pipe->epno);

  sam_putreg(priv, SAM_USBHS_HSTPIPCFG(idx), regval);

  /* Check whether the pipe is correctly configured */

  regval = sam_getreg(priv, SAM_USBHS_HSTPIPISR(idx));
  if (regval & USBHS_DEVEPTISR_CFGOK == 0)
    {
      // TODO: activation failed
    }

  if (idx < 4)
    {
      sam_putreg(priv, SAM_USBHS_HSTADDR1,
                 USBHS_HSTADDR1_HSTADDRP(pipe->funcaddr, idx));
    }
  else if (idx < 8)
    {
      sam_putreg(priv, SAM_USBHS_HSTADDR2,
                 USBHS_HSTADDR2_HSTADDRP(pipe->funcaddr, idx));
    }
  else
    {
      sam_putreg(priv, SAM_USBHS_HSTADDR3,
                 USBHS_HSTADDR3_HSTADDRP(pipe->funcaddr, idx));
    }

  /* Enable pipe interrupts */

  sam_putreg(priv, SAM_USBHS_HSTPIPIER(idx), USBHS_HSTPIPINT_RXSTALLDI | \
             USBHS_HSTPIPINT_PERRI);
  regval = sam_getreg(priv, SAM_USBHS_HSTIER);
  regval |= USBHS_DEVINT_PEP(idx);
  sam_putreg(priv, SAM_USBHS_HSTIER, regval);
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
 * Name: sam_pipe_wait
 *
 * Description:
 *   Wait for a transfer on a pipe to complete.
 *
 * Assumptions:
 *   Called from a normal thread context
 *
 ****************************************************************************/

static int sam_pipe_wait(struct sam_usbhosths_s *priv,
                         struct sam_pipe_s *pipe)
{
  irqstate_t flags;
  int ret;

  /* Disable interrupts so that the following operations will be atomic. On
   * the host global interrupt needs to be disabled. However, here we disable
   * all interrupts to exploit that fact that interrupts will be re-enabled
   * while we wait.
   */

  flags = enter_critical_section();

  /* Loop, testing for an end of transfer condition.  The pipe 'result'
   * was set to EBUSY and 'waiter' was set to the pipe expecting the
   * response before the transfer was started; 'waiter' will be nullified
   * and 'result' will be set appropriately when the transfer is completed.
   */

  do
    {
      /* Wait for the transfer to complete.  NOTE the transfer may already
       * completed before we get here or the transfer may complete while we
       * wait here.
       */

      ret = nxsem_wait(&pipe->waitsem);

      /* nxsem_wait should succeed.  But it is possible that we could be
       * awakened by a signal too.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (pipe->waiter);

  /* The transfer is complete re-enable interrupts and return the result */

  ret = -(int)pipe->result;
  leave_critical_section(flags);
  return
}

/****************************************************************************
 * Name: sam_pipe_wakeup
 *
 * Description:
 *   A pipe transfer has completed... wakeup any threads waiting for the
 *   transfer to complete.
 *
 * Assumptions:
 *   This function is called from the transfer complete interrupt handler for
 *   the pipe.  Interrupts are disabled.
 *
 ****************************************************************************/

static void sam_pipe_wakeup(struct sam_usbhosths_s *priv,
                            struct sam_pipe_s *pipe)
{
  /* Is the transfer complete? */

  if (pipe->result != EBUSY)
    {
      /* Is there a thread waiting for this transfer to complete? */

      if (pipe->waiter)
        {
#ifdef CONFIG_USBHOST_ASYNCH
          /* Yes.. there should not also be a callback scheduled */

          DEBUGASSERT(pipe->callback == NULL);
#endif
          /* Wake'em up! */

          usbhost_vtrace2(pipe->in ? SAM_VTRACE2_PIPEWAKEUP_IN :
                                     SAM_VTRACE2_PIPEWAKEUP_OUT,
                          pipe->epno, pipe->result);

          nxsem_post(&pipe->waitsem);
          pipe->waiter = false;
        }

     #ifdef CONFIG_USBHOST_ASYNCH
      /* No.. is an asynchronous callback expected
       * when the transfer completes?
       */

      else if (pipe->callback)
        {
          /* Handle continuation of IN/OUT pipes */

          if (pipe->in)
            {
              sam_in_next(priv, pipe);
            }
          else
            {
              sam_out_next(priv, pipe);
            }
        }
     #endif
    }
}

/****************************************************************************
 * Name: sam_ctrlep_alloc
 *
 * Description:
 *   Allocate a container and pipes for control pipe.
 *
 * Input Parameters:
 *   priv - The private USB host driver state.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_ctrlep_alloc(struct sam_usbhosths_s *priv,
                            const struct usbhost_epdesc_s *epdesc,
                            usbhost_ep_t *ep)
{
  struct usbhost_hubport_s *hport;
  struct sam_pipe_s *pipe;
  int idx;

  /* Sanity check.  NOTE that this method should only be called if
   * a device is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(epdesc->hport != NULL);
  hport = epdesc->hport;

  idx = sam_pipe_alloc(priv);
  if (idx < 0)
    {
      usbhost_trace1(SAM_TRACE1_PIPEALLOC_FAIL, -idx);
      uerr("ERROR: Failed to allocate a host pipe\n");
      return -ENOMEM;
    }

  pipe            = &priv->pipelist[idx];
  pipe->epno      = epdesc->addr & USB_EPNO_MASK;
  pipe->in        = false;
  pipe->eptype    = USB_EP_ATTR_XFER_CONTROL;
  pipe->funcaddr  = hport->funcaddr;
  pipe->speed     = hport->speed;
  pipe->interval  = 0;
  pipe->maxpacket = SAM_EP0_MAXPACKET;

  /* Configure control OUT pipe */

  pipe->pipestate_general = USB_H_PIPE_S_CFG;
  sam_pipe_configure(priv, idx);

  /* Return a pointer to the control pipe container as the pipe "handle" */

  *ep = (usbhost_ep_t)idx;
  return OK;
}

/****************************************************************************
 * Name: sam_xfrep_alloc
 *
 * Description:
 *   Allocate and configure one unidirectional pipe.
 *
 * Input Parameters:
 *   priv - The private USB host driver state.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated
 *   errno value is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int sam_xfrep_alloc(struct sam_usbhosths_s *priv,
                           const struct usbhost_epdesc_s *epdesc,
                           usbhost_ep_t *ep)
{
  struct usbhost_hubport_s *hport;
  struct sam_pipe_s *pipe;
  int idx;

  /* Sanity check.  NOTE that this method should only be called if a device
   * is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(epdesc->hport != NULL);
  hport = epdesc->hport;
  DEBUGASSERT(hport != NULL);

  /* Allocate a host pipe for the endpoint */

  idx = sam_pipe_alloc(priv);
  if (idx < 0)
    {
      usbhost_trace1(SAM_TRACE1_PIPEALLOC_FAIL, -idx);
      uerr("ERROR: Failed to allocate a host pipe\n");
      return -ENOMEM;
    }

  /* Decode the endpoint descriptor to initialize the pipe data structures.
   * Note:  Here we depend on the fact that the endpoint point type is
   * encoded in the same way in the endpoint descriptor as it is in the OTG
   * HS hardware.
   */

  pipe            = &priv->pipelist[idx];
  pipe->epno      = epdesc->addr & USB_EPNO_MASK;
  pipe->in        = epdesc->in;
  pipe->eptype    = epdesc->xfrtype;
  pipe->funcaddr  = hport->funcaddr;
  pipe->speed     = hport->speed;
  pipe->interval  = epdesc->interval;
  pipe->maxpacket = epdesc->mxpacketsize;
  pipe->pipestate_general = pipe->in ? USB_H_PIPE_S_DATI : USB_H_PIPE_S_DATO;

  /* Then configure the endpoint */

  sam_pipe_configure(priv, idx);

  /* Return the endpoint number as the endpoint "handle" */

  *ep = (usbhost_ep_t)idx;
  return OK;
}

/****************************************************************************
 * Name: sam_transfer_terminate
 *
 * Description:
 *   Terminate a IN or OUT transfer due to an error (or because a zero-
 *   length OUT transfer occurred).
 *
 * Returned value:
 *   OK     - Transfer successful
 *  -EAGAIN - If devices NAKs the transfer.
 *  -EPERM  - If the endpoint stalls
 *  -BUSY   - The transfer is not complete
 *  -EIO    - Other, undecoded error
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sam_transfer_terminate(struct sam_usbhosths_s *priv,
                                   struct sam_pipe_s *pipe,
                                   int result)
{
  /* Wake up any waiters for the end of transfer event */

  sam_pipe_wakeup(priv, pipe);

  if (pipe->pipestate_general < USB_H_PIPE_S_SETUP ||
      pipe->pipestate_general > USB_H_PIPE_S_STATO)
    {
      /* Not busy */

      return;
    }

  if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL)
    {
      if (priv->n_ctrl_req_user)
        priv->n_ctrl_req_user--;
      if (priv->n_sof_user)
        priv->n_sof_user--;
    }

  pipe->pipestate_general  = USB_H_PIPE_S_IDLE;
  pipe->pipestatus_general = result;

  /* Suspend delayed due to control request: start it */

  if (priv->n_ctrl_req_user == 0 && priv->suspend_start < 0)
    {
      uint8_t i;
      if (priv->n_ctrl_req_user)
        {
          /* Delay suspend after setup requests */

          priv->suspend_start = -1;
          return;
        }

      /* Save pipe freeze states and freeze pipes */

      priv->pipes_unfreeze = 0;
      for (i = 0; i < SAM_USB_NENDPOINTS; i++)
        {
          /* Skip frozen pipes */

          if ((sam_getreg(priv, SAM_USBHS_HSTPIPISR(i)) &
              USBHS_HSTPIPINT_PFREEZEI) >> 17)
            {
              continue;
            }

          /* Log unfrozen pipes */

          priv->pipes_unfreeze |= 1 << i;

          /* Freeze it to suspend */

          sam_putreg(priv, SAM_USBHS_HSTPIPIER(i), USBHS_HSTPIPINT_PFREEZEI);
        }

      /* Wait 3 SOFs before entering in suspend state */

      sam_add_sof_user(priv);
      priv->suspend_start = 3;
    }
}

static void sam_transfer_abort(struct sam_usbhosths_s *priv,
                               struct sam_pipe_s *pipe,
                               int code)
{
  uint32_t regval;
  /* Stop transfer */

  sam_putreg(priv, SAM_USBHS_HSTPIPIER(pipe->idx), USBHS_HSTPIPINT_PFREEZEI);

  /* Update byte count */

  if (pipe->in == 0)
    {
      regval = sam_getreg(priv, SAM_USBHS_HSTPIPISR(epno));
      pipe->count += (regval & USBHS_HSTPIPISR_PBYCT_MASK) >>
                      USBHS_HSTPIPISR_PBYCT_SHIFT;
    }

  /* Disable interrupts */

  // TODO

  sam_transfer_terminate(priv, pipe, code);
}

/****************************************************************************
 * Name: sam_send_continue
 *
 * Description:
 *   Continue the send operation started by sam_send_start().
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sam_send_continue(struct sam_usbhost_s *priv,
                              struct sam_pipe_s *pipe)
{
  uint8_t  *src;
  uint32_t size;
  uint32_t count;
  uint32_t n_tx = 0;
  uint32_t n_remain;
  uint32_t regval;

  if (pipe->pipestate_general == USB_H_PIPE_S_STATO)
    {
      /* Control status : ZLP OUT done */

      sam_transfer_terminate(priv, pipe, OK);
      return;
    }
  else if (pipe->pipestate_general != USB_H_PIPE_S_DATO)
    {
      return;
    }

  /* Reset packet timeout for control pipes */

  if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL)
    pipe->pkt_timeout = USB_CTRL_DPKT_TIMEOUT;

  regval = sam_getreg(priv, SAM_USBHS_HSTPIPCFG(epno));
  n_tx = (regval & USBHS_HSTPIPCFG_PSIZE_MASK) >> USBHS_HSTPIPCFG_PSIZE_SHIFT;

  /* ZLP cleared if it's short packet */

  if (n_tx < pipe->maxpacket)
    {
      pipe->zlp = 0;
    }

  src = pipe->data;
  size = pipe->size;
  count = pipe->count;

  if (n_tx)
    {
      count += n_tx;
      pipe->count = count;
    }

  n_remain = size - count;

  /* Now set n_tx to next transfer size */

  n_tx = n_remain > 16320 ? 16320 : n_remain;

  /* For Control, all data is done, to STATUS stage */

  if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL &&
                     pipe->count >= pipe->size &&
                     !pipe->zlp)
    {
      pipe->pipestate = USB_H_PIPE_S_STATI;

      /* Start IN ZLP request */

      pipe->pkt_timeout = USB_CTRL_STAT_TIMEOUT;
      sam_recv_restart(priv, pipe);
      return;
    }

  /* All transfer done, including ZLP */

  if (count >= size && !pipe->zlp)
    {
      /* At least one bank there, wait to freeze pipe */

      if (pipe->eptype != USB_EP_ATTR_XFER_CONTROL)
        {
          /* Busy interrupt when all banks are empty */

          sam_transfer_terminate(priv, pipe, OK);
        }
    }
  else
    {
      regval = sam_getreg(priv, SAM_USBHS_HSTPIPCFG(epno));
      regval &= ~USBHS_HSTPIPCFG_PTOKEN_MASK;
      regval |= USBHS_HSTPIPCFG_PTOKEN_OUT;
      sam_putreg(priv, SAM_USBHS_HSTPIPCFG(epno), regval);

      sam_putreg(priv, SAM_USBHS_HSTPIPICR(epno), USBHS_HSTPIPINT_TXOUTI);

      /* Write packet in the FIFO buffer */

      fifo = (uint8_t *)
        ((uint32_t *)SAM_USBHSRAM_BASE + (EPT_FIFO_SIZE * epno));

      for (; pipe->size; pipe->size--)
        {
          *fifo++ = *src[count]++;
        }

      MEMORY_SYNC();

      sam_putreg(priv, SAM_USBHS_HSTPIPIER(epno), USBHS_HSTPIPINT_TXOUTI);
      sam_putreg(priv, SAM_USBHS_HSTPIPIDR(epno), USBHS_HSTPIPINT_FIFOCONI |
                  USBHS_HSTPIPINT_PFREEZEI);
    }
}

/****************************************************************************
 * Name: sam_send_start
 *
 * Description:
 *   Start at transfer on the selected IN or OUT pipe.
 *
 ****************************************************************************/

static void sam_send_start(struct sam_usbhost_s *priv,
                           struct sam_pipe_s *pipe)
{
  volatile uint8_t *fifo;
  uint32_t regval;
  uint8_t psize;
  uint8_t epno = pipe->idx;

  /* Set up the initial state of the transfer */

  usbhost_vtrace2(SAM_VTRACE2_STARTTRANSFER1, pipe->idx, pipe->size);

  pipe->result = EBUSY;
  pipe->count = 0;

  regval = sam_getreg(priv, SAM_USBHS_HSTPIPCFG(epno));
  psize = (regval & USBHS_HSTPIPCFG_PSIZE_MASK) >> USBHS_HSTPIPCFG_PSIZE_SHIFT;

  regval &= ~USBHS_HSTPIPCFG_PTOKEN_MASK;
  regval |= USBHS_HSTPIPCFG_PTOKEN_OUT;
  sam_putreg(priv, SAM_USBHS_HSTPIPCFG(epno), regval);

  sam_putreg(priv, SAM_USBHS_HSTPIPICR(epno), USBHS_HSTPIPINT_TXOUTI);

  /* Write packet in the FIFO buffer */

  fifo = (uint8_t *)
    ((uint32_t *)SAM_USBHSRAM_BASE + (EPT_FIFO_SIZE * epno));

  for (; pipe->size; pipe->size--)
    {
      *fifo++ = *pipe->data++;
    }

  MEMORY_SYNC();

  sam_putreg(priv, SAM_USBHS_HSTPIPIER(epno), USBHS_HSTPIPINT_TXOUTI);
  sam_putreg(priv, SAM_USBHS_HSTPIPIDR(epno), USBHS_HSTPIPINT_FIFOCONI |
              USBHS_HSTPIPINT_PFREEZEI);
}

/****************************************************************************
 * Name: sam_out_transfer
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' through an OUT pipe.
 *
 * Assumptions:
 *   This function is called only from the TRANSFER
 *   interface.  The lock, for example,
 *   must be relinquished before waiting.
 *
 ****************************************************************************/

static ssize_t sam_out_transfer(struct sam_usbhost_s *priv,
                                struct sam_pipe_s *pipe,
                                uint8_t *buffer, size_t buflen)
{
  clock_t start;
  clock_t elapsed;
  size_t xfrlen;
  ssize_t xfrd;
  int ret;

  /* Loop until the transfer completes (i.e., buflen is decremented to zero)
   * or a fatal error occurs (any error other than a simple NAK)
   */

  start = clock_systime_ticks();
  xfrd  = 0;

  while (buflen > 0)
    {
      /* Transfer one packet at a time.  The hardware is capable of queueing
       * multiple OUT packets, but I just haven't figured out how to handle
       * the case where a single OUT packet in the group is NAKed.
       */

      xfrlen = MIN(pipe->maxpacket, buflen);
      pipe->data = buffer;
      pipe->size = xfrlen;
      pipe->count = 0;
      uinfo("pipe%d buffer:%p buflen:%d\n",
                                 pipe->idx,
                                 pipe->data,
                                 pipe->size);

      /* Set up for the wait BEFORE starting the transfer */

      ret = sam_pipe_waitsetup(priv, pipe);
      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_DEVDISCONN1, 0);
          return (ssize_t)ret;
        }

      /* Set up for the transfer based
       * on the direction and the endpoint type
       */

      ret = sam_out_setup(priv, pipe);

      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_OUTSETUP_FAIL1, -ret);
          return (ssize_t)ret;
        }

      /* Wait for the transfer to complete and get the result */

      ret = sam_pipe_wait(priv, pipe);

      /* Handle transfer failures */

      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_TRANSFER_FAILED1, ret);

          /* Check for a special case:  If (1) the transfer was NAKed and (2)
           * no SNDFIFO empty or Rx FIFO not-empty event occurred, then we
           * should be able to just flush the Rx and SNDFIFOs and try again.
           * We can detect this latter case because then the transfer buffer
           * pointer and buffer size will be unaltered.
           */

          elapsed = clock_systime_ticks() - start;
          if (ret != -EAGAIN ||                /* Not a NAK condition OR */
              elapsed >= SAM_DATANAK_DELAY ||  /* Timeout has elapsed OR */
              pipe->count > 0)                 /* Data has been partially transferred */
            {
              /* Break out and return the error */

              usbhost_trace1(SAM_TRACE1_PIPEWAIT_FAIL, -ret);
              return (ssize_t)ret;
            }

          /* Get the device a little time to catch up.
           * Then retry the transfer
           * using the same buffer pointer and length.
           */

          nxsig_usleep(20 * 1000);
        }
      else
        {
          /* Successfully transferred. Update the buffer pointer and length */

          buffer += xfrlen;
          buflen -= xfrlen;
          xfrd   += pipe->count;
        }
    }

  return xfrd;
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

static int sam_ctrl_sendsetup(struct sam_usbhosths_s *priv,
                              struct sam_pipe_s *pipe,
                              const struct usb_ctrlreq_s *req);
{
  volatile uint8_t *fifo;
  clock_t start;
  clock_t elapsed;
  unsigned int epno;
  int ret;
  int i;

  /* Loop while the device reports NAK (and a timeout is not exceeded) */

  epno = pipe->idx;

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

      /* Fill SETUP token */

      regval = sam_getreg(priv, SAM_USBHS_HSTPIPCFG(epno));
      regval &= ~USBHS_HSTPIPCFG_PTOKEN_MASK;
      regval |= USBHS_HSTPIPCFG_PTOKEN_SETUP;
      sam_putreg(priv, SAM_USBHS_HSTPIPCFG(epno), regval);

      sam_putreg(priv, SAM_USBHS_HSTPIPICR(epno), USBHS_HSTPIPINT_TXSTPI);

      for (i = 0; i < USB_SIZEOF_CTRLREQ; i++)
        {
          priv->ctrl_buffer[i] = pipe->data[i];
        }

      /* Write packet in the FIFO buffer */

      fifo = (uint8_t *)
        ((uint32_t *)SAM_USBHSRAM_BASE + (EPT_FIFO_SIZE * epno));

      for (; pipe->size; pipe->size--)
        {
          *fifo++ = *pipe->data++;
        }

      MEMORY_SYNC();

      sam_putreg(priv, SAM_USBHS_HSTPIPIER(epno), USBHS_HSTPIPINT_TXSTPI);
      sam_putreg(priv, SAM_USBHS_HSTPIPIDR(epno), USBHS_HSTPIPINT_FIFOCONI |
                 USBHS_HSTPIPINT_PFREEZEI);

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

/****************************************************************************
 * Name: sam_ctrl_senddata
 *
 * Description:
 *   Send data in the data phase of an OUT control transfer.  Or send status
 *   in the status phase of an IN control transfer
 *
 * Assumptions:
 *   This function is called only from the CTRLOUT interface.
 *
 ****************************************************************************/

static int sam_ctrl_senddata(struct sam_usbhosths_s *priv,
                             struct sam_pipe_s *pipe,
                             uint8_t *buffer, unsigned int buflen)
{
  int ret;

  uinfo("pipe%d buffer:%p buflen:%d\n", pipe->idx, buffer, buflen);

  /* Save buffer information */

  pipe->pipestate_general = USB_H_PIPE_S_DATO;
  pipe->in = false;
  pipe->data = buffer;
  pipe->size = buflen;
  pipe->count = 0;

  /* Set up for the wait BEFORE starting the transfer */

  ret = sam_pipe_waitsetup(priv, pipe);
  if (ret < 0)
    {
      usbhost_trace1(SAM_TRACE1_DEVDISCONN3, 0);
      return ret;
    }

  /* Start the transfer */

  sam_send_start(priv, pipe);

  /* Wait for the transfer to complete and return the result */

  return sam_pipe_wait(priv, pipe);
}

/****************************************************************************
 * Name: sam_ctrl_recvdata
 *
 * Description:
 *   Receive data in the data phase of an IN control transfer.
 *   Or receive status in the status phase of
 *   an OUT control transfer.
 *
 * Assumptions:
 *   This function is called only from the CTRLIN interface.
 *
 ****************************************************************************/

static int sam_ctrl_recvdata(struct sam_usbhosths_s *priv,
                             struct sam_pipe_s *pipe,
                             uint8_t *buffer, unsigned int buflen)
{
  int ret;

  /* Save buffer information */

  pipe->pipestate_general = USB_H_PIPE_S_DATI;

  pipe->in = true;
  pipe->data = buffer;
  pipe->size = buflen;
  pipe->count = 0;

  /* Set up for the wait BEFORE starting the transfer */

  ret = sam_pipe_waitsetup(priv, pipe);
  if (ret < 0)
    {
      usbhost_trace1(SAM_TRACE1_DEVDISCONN4, 0);
      return ret;
    }

  /* Start the transfer */

  sam_recv_start(priv, pipe);

  /* Wait for the transfer to complete and return the result */

  ret = sam_pipe_wait(priv, pipe);

  return ret;
}

/****************************************************************************
 * Name: sam_in_transfer
 *
 * Description:
 *   Transfer 'buflen' bytes into 'buffer' from an IN pipe.
 *
 * Assumptions:
 *   This function is called only from the TRANSFER.
 *   The lock, for example, must be relinquished before waiting.
 *
 ****************************************************************************/

static ssize_t sam_in_transfer(struct sam_usbhost_s *priv,
                               struct sam_pipe_s *pipe,
                               uint8_t *buffer, size_t buflen)
{
  clock_t start;
  ssize_t xfrd;
  int ret;

  /* Loop until the transfer completes (i.e., buflen is decremented to zero)
   * or a fatal error occurs any error other than a simple NAK. NAK would
   * simply indicate the end of the transfer (short-transfer).
   */

  pipe->data = buffer;
  pipe->size = buflen;
  pipe->count = 0;
  xfrd = 0;

  start = clock_systime_ticks();
  while (pipe->count < pipe->size)
    {
      /* Set up for the wait BEFORE starting the transfer */

      ret = sam_pipe_waitsetup(priv, pipe);
      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_DEVDISCONN5, 0);
          return (ssize_t)ret;
        }

      /* Set up for the transfer based on the direction
       * and the endpoint type
       */

      ret = sam_in_setup(priv, pipe);

      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_INSETUP_FAIL1, -ret);
          return (ssize_t)ret;
        }

      /* Wait for the transfer to complete and get the result */

      ret = sam_pipe_wait(priv, pipe);

      /* EAGAIN indicates that the device NAKed the transfer. */

      if (ret < 0)
        {
          /* The transfer failed.  If we received a NAK, return all data
           * buffered so far (if any).
           */

          if (ret == -EAGAIN)
            {
              /* Was data buffered prior to the NAK? */

              if (xfrd > 0)
                {
                  return xfrd;
                }
              else
                {
                  useconds_t delay;

                  /* Get the elapsed time. Has the timeout elapsed?
                   * If not then try again.
                   */

                  clock_t elapsed = clock_systime_ticks() - start;
                  if (elapsed >= SAM_DATANAK_DELAY)
                    {
                      /* Timeout out... break out returning the NAK as
                       * as a failure.
                       */

                      return (ssize_t)ret;
                    }

                  /* Wait a bit before retrying after a NAK. */

                  if (pipe->eptype == USB_EP_ATTR_XFER_INT)
                    {
                      /* For interrupt (and isochronous) endpoints, the
                       * polling rate is determined by the bInterval field
                       * of the endpoint descriptor (in units of frames
                       * which we treat as milliseconds here).
                       */

                      if (pipe->interval > 0)
                        {
                          /* Convert the delay to units of microseconds */

                          delay = (useconds_t)pipe->interval * 1000;
                        }
                      else
                        {
                          /* Out of range! For interrupt endpoints, the valid
                           * range is 1-255 frames.  Assume one frame.
                           */

                          delay = 1000;
                        }
                    }
                  else
                    {
                      /* For Isochronous endpoints, bInterval must be 1. Bulk
                       * endpoints do not have a polling interval.  Rather,
                       * the should wait until data is received.
                       *
                       * REVISIT:  For bulk endpoints this 1 msec delay is
                       * only intended to give the CPU a break from the bulk
                       * EP tight polling loop. But are there performance
                       * issues?
                       */

                      delay = 1000;
                    }

                      /* Wait for the next polling interval. For interrupt and
                       * isochronous endpoints, this is necessary to assure
                       * the polling interval.  It is used in other cases only
                       * to prevent the polling from consuming too much CPU
                       * bandwidth.
                       *
                       * Small delays could require more resolution than is
                       * provided by the system timer. For example, if the
                       * system timer resolution is 10MS, then
                       * nxsig_usleep(1000) will actually request a delay 20MS
                       * (due to both quantization and rounding).
                       *
                       * REVISIT: So which is better?  To ignore tiny delays
                       * and hog the system bandwidth?  Or to wait for an
                       * excessive amount and destroy system throughput?
                       */

                  if (delay > CONFIG_USEC_PER_TICK)
                    {
                      nxsig_usleep(delay - CONFIG_USEC_PER_TICK);
                    }
                }
            }
          else
            {
              /* Some unexpected, fatal error occurred. */

              usbhost_trace1(SAM_TRACE1_TRANSFER_FAILED3, -ret);

              /* Break out and return the error */

              return (ssize_t)ret;
            }
        }
      else
        {
          /* Successfully received another chunk of data... add that to the
           * running total.  Then continue reading until we read 'buflen'
           * bytes of data or until the devices NAKs (implying a short
           * packet).
           */

          xfrd += pipe->count;
        }
    }

  return xfrd;
}

/****************************************************************************
 * Name: sam_in_setup
 *
 * Description:
 *   Initiate an IN transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int sam_in_setup(struct sam_usbhost_s *priv,
                        struct sam_pipe_s *pipe)
{
  uinfo("pipe%d\n", pipe->idx);

  /* Set up for the transfer based on the direction and the endpoint type */

  switch (pipe->eptype)
    {
      default:
      case USB_EP_ATTR_XFER_CONTROL: /* Control */
        {
          /* This kind of transfer on control endpoints other than EP0 are
           * not currently supported
           */

          return -ENOSYS;
        }

      case USB_EP_ATTR_XFER_ISOC: /* Isochronous */
        {
          /* Set up the IN DATA0 PID */

          usbhost_vtrace2(SAM_VTRACE2_ISOCIN, pipe->idx, pipe->size);
        }
        break;

      case USB_EP_ATTR_XFER_BULK: /* Bulk */
        {
          usbhost_vtrace2(SAM_VTRACE2_BULKIN, pipe->idx, pipe->size);
          pipe->pipestate_general = pipe->in ?
                USB_H_PIPE_S_DATI : USB_H_PIPE_S_DATO;
        }
        break;

      case USB_EP_ATTR_XFER_INT: /* Interrupt */
        {
          usbhost_vtrace2(SAM_VTRACE2_INTRIN, pipe->idx, pipe->size);
        }
        break;
    }

  /* Start the transfer. */

  sam_recv_start(priv, pipe);
  return OK;
}

/****************************************************************************
 * Name: sam_out_setup
 *
 * Description:
 *   Initiate an OUT transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int sam_out_setup(struct sam_usbhost_s *priv,
                         struct sam_pipe_s *pipe)
{
  /* Set up for the transfer based on the direction and the endpoint type */

  switch (pipe->eptype)
    {
      default:
      case USB_EP_ATTR_XFER_CONTROL: /* Control */
        {
          /* This kind of transfer on control endpoints other than EP0 are
           * not currently supported
           */

          return -ENOSYS;
        }

      case USB_EP_ATTR_XFER_ISOC: /* Isochronous */
        {
          /* Set up the IN DATA0 PID */

          usbhost_vtrace2(SAM_VTRACE2_ISOCOUT,
                          pipe->idx, pipe->size);
        }
        break;

      case USB_EP_ATTR_XFER_BULK: /* Bulk */
        {
          usbhost_vtrace2(SAM_VTRACE2_BULKOUT,
                          pipe->idx, pipe->size);
          pipe->pipestate_general = pipe->in ?
                           USB_H_PIPE_S_DATI : USB_H_PIPE_S_DATO;
        }
        break;

      case USB_EP_ATTR_XFER_INT: /* Interrupt */
        {
          usbhost_vtrace2(SAM_VTRACE2_INTROUT,
                          pipe->idx, pipe->size);
        }
        break;
    }

  /* Start the transfer */

  sam_send_start(priv, pipe);
  return OK;
}

/****************************************************************************
 * Name: sam_recv_continue
 *
 * Description:
 *   Continue the receive operation started by sam_recv_start().  This
 *   function is called from the interrupt handler worker when an interrupt
 *   indicates that new, incoming data is available.
 *
 ****************************************************************************/

static void sam_recv_continue(struct sam_usbhosths_s *priv,
                              struct sam_pipe_s *pipe)
{
  uint8_t *src;
  uint8_t *dst;
  uint32_t size;
  uint32_t count;
  uint32_t i;
  uint32_t n_rx = 0;
  uint32_t n_remain;
  uint32_t regval;
  volatile const uint8_t *fifo;
  uint8_t epno = pipe->idx;
  bool shortpkt = false;
  bool full = false;

  if (pipe->pipestate_general == USB_H_PIPE_S_STATI)
    {
      /* Control status : ZLP IN done */

      sam_transfer_terminate(priv, pipe, OK);
      return;
    }
  else if (pipe->pipestate_general != USB_H_PIPE_S_DATI)
    {
      return;
    }

  /* Read byte count */

  regval = sam_getreg(priv, SAM_USBHS_HSTPIPISR(epno));
  n_rx = (regval & USBHS_HSTPIPISR_PBYCT_MASK) >> USBHS_HSTPIPISR_PBYCT_SHIFT;
  if (n_rx < pipe->maxpacket)
    {
      shortpkt = true;
    }

  /* Retrieve packet from the FIFO */

  if (n_rx)
    {
      dst = pipe->data;
      size = pipe->size;
      count = pipe->count;
      n_remain = size - count;

      fifo = (volatile const uint8_t *)
        SAM_USBHSRAM_BASE  + (EPT_FIFO_SIZE * epno);

      dst = &dst[count];
      if (n_rx >= n_remain)
        {
          n_rx = n_remain;
          full = true;
        }

      count += n_rx;
      for (i = 0; i < n_rx; i++)
        {
          *dst++ = *fifo++;
        }

      MEMORY_SYNC();

      pipe->count = count;
    }

  /* Reset timeout for control pipes */

  if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL)
    {
      pipe->pkt_timeout = USB_CTRL_DPKT_TIMEOUT;
    }

  /* Clear FIFO status */

  sam_putreg(priv, SAM_USBHS_HSTPIPIDR(epno), USBHS_HSTPIPINT_FIFOCONI);

  /* Finish on error or short packet */

  if (full || shortpkt)
    {
      if (pipe->eptype == USB_EP_ATTR_XFER_CONTROL)
        {
          // TODO
        }
      else
        {
          sam_transfer_terminate(priv, pipe, OK);
        }
    }
  else
    {
      /* Just wait another packet */

      sam_recv_restart(priv, pipe);
    }
}

/****************************************************************************
 * Name: sam_recv_restart
 *
 * Description:
 *   Start/Re-start the transfer on the selected IN or OUT pipe
 *
 ****************************************************************************/

static void sam_recv_restart(struct sam_usbhosths_s *priv,
                             struct sam_pipe_s *pipe)
{
  /* Send the IN token. */

  uint8_t epno = pipe->idx;

  regval = sam_getreg(priv, SAM_USBHS_HSTPIPCFG(epno));
  regval &= ~USBHS_HSTPIPCFG_PTOKEN_MASK;
  regval |= USBHS_HSTPIPCFG_PTOKEN_IN;
  sam_putreg(priv, SAM_USBHS_HSTPIPCFG(epno), regval);

  sam_putreg(priv, SAM_USBHS_HSTPIPICR(epno), USBHS_HSTPIPINT_RXINI |
             USBHS_HSTPIPINT_SHRTPCKTI);

  sam_putreg(priv, SAM_USBHS_HSTPIPIER(epno), USBHS_HSTPIPINT_RXINI);

  sam_putreg(priv, SAM_USBHS_HSTPIPIDR(epno), USBHS_HSTPIPINT_FIFOCONI |
             USBHS_HSTPIPINT_PFREEZEI);
}

/****************************************************************************
 * Name: sam_recv_start
 *
 * Description:
 *   Start at transfer on the selected IN or OUT pipe.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sam_recv_start(struct sam_usbhosths_s *priv,
                           struct sam_pipe_s *pipe)
{
  /* Set up the initial state of the transfer */

  usbhost_vtrace2(SAM_VTRACE2_STARTTRANSFER2, pipe->idx, pipe->size);

  pipe->result = EBUSY;
  pipe->count = 0;

  /* Start the transfer. */

  sam_recv_restart(priv, pipe);
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
  struct sam_usbhosths_s *priv = (struct sam_usbhosths_s *)drvr;
  struct sam_pipe_s *pipe;
  int ret;

  //usbhost_vtrace2(EHCI_VTRACE2_EP0CONFIG, speed, funcaddr);
  DEBUGASSERT(drvr != NULL && epinfo != NULL && maxpacketsize < 2048);

  /* We must have exclusive access to data structures. */

  ret = nxrmutex_lock(&priv->lock);
  if (ret >= 0)
    {
      /* Remember the new device address and max packet size */

      pipe            = &priv->pipelist[(unsigned int)ep0];
      pipe->funcaddr  = funcaddr;
      pipe->speed     = speed;
      pipe->maxpacket = maxpacketsize;
      sam_pipe_configure(priv, pipe->idx);

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
  struct sam_usbhosths_s *priv = (struct sam_usbhosths_s *)drvr;
  int ret;

  /* Sanity check.  NOTE that this method should only be called if a device
   * is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(drvr != 0 && epdesc != NULL && ep != NULL);

  /* We must have exclusive access to the USB
   * host hardware and state structures
   */

  nxmutex_lock(&priv->lock);

  /* Handler control pipes differently from other endpoint types.  This is
   * because the normal, "transfer" endpoints are unidirectional an require
   * only a single pipe.  Control endpoints, however, are bi-diretional
   * and require two pipes, one for the IN and one for the OUT direction.
   */

  if (epdesc->xfrtype == USB_EP_ATTR_XFER_CONTROL)
    {
      ret = sam_ctrlep_alloc(priv, epdesc, ep);
    }
  else
    {
      ret = sam_xfrep_alloc(priv, epdesc, ep);
    }

  nxmutex_unlock(&priv->lock);
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

          nxsig_usleep(200);
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
  struct sam_usbhosths_s *priv = (struct sam_usbhosths_s *)drvr;
  struct sam_pipe_s *pipe;
  uint16_t buflen;
  clock_t start;
  clock_t elapsed;
  int retries;
  int ret;

  DEBUGASSERT(priv != NULL && req != NULL &&
             (unsigned int)ep0 < SAM_USB_NENDPOINTS);
  usbhost_vtrace2(SAM_VTRACE2_CTRLOUT, req->type, req->req);

  pipe = &priv->pipelist[(unsigned int)ep0];

  /* Extract values from the request */

  buflen = sam_getle16(req->len);
  uinfo("type:0x%02x req:0x%02x value:0x%02x%02x index:0x%02x%02x len:%d\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], buflen);

  /* We must have exclusive access to the
   * USB host hardware and state structures
   */

  nxmutex_lock(&priv->lock);

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < SAM_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request */

      ret = sam_ctrl_sendsetup(priv, pipe, req);
      if (ret < 0)
        {
          usbhost_trace1(SAM_TRACE1_SENDSETUP_FAIL1, -ret);
          continue;
        }

      /* Get the start time.  Loop again until the timeout expires */

      start = clock_systime_ticks();
      do
        {
          /* Handle the data OUT phase (if any) */

          if (buflen > 0)
            {
              /* Start DATA out transfer (only one DATA packet) */

              ret = sam_ctrl_senddata(priv, pipe,
                                     (uint8_t *)buffer, buflen);
              if (ret < 0)
                {
                  usbhost_trace1(SAM_TRACE1_SENDDATA_FAIL, -ret);
                }
            }

          /* Handle the status IN phase */

          if (ret == OK)
            {
              ret = sam_ctrl_recvdata(priv, pipe, NULL, 0);
              if (ret == OK)
                {
                  /* All success transactins exit here */

                  nxmutex_unlock(&priv->lock);
                  return OK;
                }

              usbhost_trace1(SAM_TRACE1_RECVSTATUS_FAIL,
                             ret < 0 ? -ret : ret);
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

/****************************************************************************
 * Name: sam_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request, blocking until the transfer completes.
 *   Only one transfer may be  queued; Neither this method nor the ctrlin
 *   or ctrlout methods can be called again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on
 *     which to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *    received (IN endpoint). Buffer must have been allocated using
 *    DRVR_ALLOC.
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Value:
 *   On success, a non-negative value is returned that indicates the number
 *   of bytes successfully transferred.  On a failure, a negated errno value
 *   is returned that indicates the nature of the failure:
 *
 *     EAGAIN - If devices NAKs the transfer (or NYET or other error where
 *              it may be appropriate to restart the entire transaction).
 *     EPERM  - If the endpoint stalls
 *     EIO    - On a TX or data toggle error
 *     EPIPE  - Overrun errors
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static ssize_t sam_transfer(struct usbhost_driver_s *drvr,
                            usbhost_ep_t ep,
                            uint8_t *buffer,
                            size_t buflen)
{
  struct sam_usbhost_s *priv = (struct sam_usbhost_s *)drvr;
  struct sam_pipe_s *pipe;
  unsigned int idx = (unsigned int)ep;
  ssize_t nbytes;

  uwarn("pipe%d buffer:%p buflen:%d\n",  idx, buffer, buflen);

  DEBUGASSERT(priv && buffer && idx < SAM_USB_NENDPOINTS && buflen > 0);
  pipe = &priv->pipelist[idx];

  /* We must have exclusive access to the
   * USB host hardware and state structures
   */

  nxmutex_lock(&priv->lock);

  /* Handle IN and OUT transfer slightly differently */

  if (pipe->in)
    {
      nbytes = sam_in_transfer(priv, pipe, buffer, buflen);
    }
  else
    {
      nbytes = sam_out_transfer(priv, pipe, buffer, buflen);
    }

  nxmutex_unlock(&priv->lock);
  return nbytes;
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
  struct usbhost_driver_s *drvr;
  struct usbhost_hubport_s *hport;
  int epno;

  /* Initialize the device operations */

  drvr                 = &priv->drvr;
  drvr->ep0configure   = sam_ep0configure;
  drvr->epalloc        = sam_epalloc;
  drvr->epfree         = sam_epfree;
  drvr->alloc          = sam_alloc;
  drvr->free           = sam_free;
  drvr->ioalloc        = sam_ioalloc;
  drvr->iofree         = sam_iofree;
  drvr->ctrlin         = sam_ctrlin;
  drvr->ctrlout        = sam_ctrlout;
  drvr->transfer       = sam_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
  drvr->asynch         = sam_asynch;
#endif
  drvr->cancel         = sam_cancel;
#ifdef CONFIG_USBHOST_HUB
  drvr->connect        = sam_connect;
#endif
  drvr->disconnect     = sam_disconnect;

  /* Initialize the public port representation */

  hport                = &priv->rhport.hport;
  hport->drvr          = drvr;
#ifdef CONFIG_USBHOST_HUB
  hport->parent        = NULL;
#endif
  hport->ep0           = priv->ep0;
  hport->speed         = USB_SPEED_FULL;

  usbhost_devaddr_initialize(&priv->devgen);
  priv->rhport.pdevgen = &priv->devgen;

  for (epno = 0; epno < SAM_USB_NENDPOINTS; epno++)
    {
      priv->pipelist[epno].idx = epno;
      nxsem_init(&priv->pipelist[epno].waitsem, 0, 0);
    }

  sam_reset_pipes(priv, false);
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
