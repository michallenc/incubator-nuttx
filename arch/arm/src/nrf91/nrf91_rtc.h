/****************************************************************************
 * arch/arm/src/nrf91/nrf91_rtc.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_NRF91_NRF91_RTC_H
#define __ARCH_ARM_SRC_NRF91_NRF91_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#define NRF91_RTC_START(d)                ((d)->ops->start(d))
#define NRF91_RTC_STOP(d)                 ((d)->ops->stop(d))
#define NRF91_RTC_CLEAR(d)                ((d)->ops->clear(d))
#define NRF91_RTC_TRGOVRFLW(d)            ((d)->ops->trgovrflw(d))
#define NRF91_RTC_GETCOUNTER(d, c)        ((d)->ops->getcounter(d, c))
#define NRF91_RTC_SETCC(d, i, cc)         ((d)->ops->setcc(d, i, cc))
#define NRF91_RTC_GETCC(d, i, cc)         ((d)->ops->setcc(d, i, cc))
#define NRF91_RTC_SETPRE(d, pre)          ((d)->ops->setpre(d, pre))
#define NRF91_RTC_SETISR(d, hnd, arg)     ((d)->ops->setisr(d, hnd, arg))
#define NRF91_RTC_ENABLEINT(d, s)         ((d)->ops->enableint(d, s))
#define NRF91_RTC_DISABLEINT(d, s)        ((d)->ops->disableint(d, s))
#define NRF91_RTC_CHECKINT(d, s)          ((d)->ops->checkint(d, s))
#define NRF91_RTC_ACKINT(d, s)            ((d)->ops->ackint(d, s))
#define NRF91_RTC_ENABLEEVT(d, s)         ((d)->ops->enableevt(d, s))
#define NRF91_RTC_DISABLEEVT(d, s)        ((d)->ops->disableevt(d, s))
#define NRF91_RTC_GETBASE(d)              ((d)->ops->getbase(d))

/* These are defined for direct access to registers, which is needed in some
 * critical parts where access speed is important
 */

#define NRF91_RTC_GETCOUNTER_REG(base)    (getreg32(base + NRF91_RTC_COUNTER_OFFSET))
#define NRF91_RTC_SETCC_REG(base, ch, cc) (putreg32(cc, base + NRF91_RTC_CC_OFFSET(ch)))
#define NRF91_RTC_GETCC_REG(base, ch)     (getreg32(base + NRF91_RTC_CC_OFFSET(ch)))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* RTC CC index */

enum nrf91_rtc_cc_e
{
  NRF91_RTC_CC0 = 0,
  NRF91_RTC_CC1 = 1,
  NRF91_RTC_CC2 = 2,
  NRF91_RTC_CC3 = 3,
};

/* RTC Interrupts/Events */

enum nrf91_rtc_evt_e
{
  NRF91_RTC_EVT_COMPARE0 = 0,
  NRF91_RTC_EVT_COMPARE1 = 1,
  NRF91_RTC_EVT_COMPARE2 = 2,
  NRF91_RTC_EVT_COMPARE3 = 3,
  NRF91_RTC_EVT_TICK     = 4,
  NRF91_RTC_EVT_OVRFLW   = 5,
};

/* NRF91 RTC device */

struct nrf91_rtc_dev_s
{
  struct nrf91_rtc_ops_s *ops;
};

/* NRF91 RTC ops */

struct nrf91_rtc_ops_s
{
  /* RTC tasks */

  int (*start)(struct nrf91_rtc_dev_s *dev);
  int (*stop)(struct nrf91_rtc_dev_s *dev);
  int (*clear)(struct nrf91_rtc_dev_s *dev);
  int (*trgovrflw)(struct nrf91_rtc_dev_s *dev);

  /* RTC operations */

  int (*getcounter)(struct nrf91_rtc_dev_s *dev, uint32_t *cc);
  int (*setcc)(struct nrf91_rtc_dev_s *dev, uint8_t i, uint32_t cc);
  int (*getcc)(struct nrf91_rtc_dev_s *dev, uint8_t i, uint32_t *cc);
  int (*setpre)(struct nrf91_rtc_dev_s *dev, uint16_t pre);

  /* RTC interrupts */

  int (*setisr)(struct nrf91_rtc_dev_s *dev, xcpt_t handler, void *arg);
  int (*enableint)(struct nrf91_rtc_dev_s *dev, uint8_t source);
  int (*disableint)(struct nrf91_rtc_dev_s *dev, uint8_t source);
  int (*checkint)(struct nrf91_rtc_dev_s *dev, uint8_t source);
  int (*ackint)(struct nrf91_rtc_dev_s *dev, uint8_t source);

  /* RTC events */

  int (*enableevt)(struct nrf91_rtc_dev_s *dev, uint8_t evt);
  int (*disableevt)(struct nrf91_rtc_dev_s *dev, uint8_t evt);

  /* Utility */

  uint32_t (*getbase)(struct nrf91_rtc_dev_s *dev);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct nrf91_rtc_dev_s *nrf91_rtc_init(int rtc);
int nrf91_rtc_deinit(struct nrf91_rtc_dev_s *dev);

#endif /* __ARCH_ARM_SRC_NRF91_NRF91_RTC_H */
