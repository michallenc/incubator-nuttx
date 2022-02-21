/****************************************************************************
 * boards/arm/samv7/brcg2/src/sam_w25qxxxjv.c
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
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include "sam_qspi.h"

#include "brcg2.h"

#ifdef HAVE_W25QXXXJV
#  include <nuttx/spi/qspi.h>
#  include <nuttx/mtd/mtd.h>
#  include <nuttx/mtd/configdata.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing the watchdog
 * timer
 */

#define W25QXXXJV_QSPI_PORT 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_w25qxxxjv_initialize
 *
 * Description:
 *   Initialize and register the W25QXXXJV FLASH file system.
 *
 ****************************************************************************/

int sam_w25qxxxjv_initialize(int minor)
{
  int ret;
#ifdef HAVE_W25QXXXJV
  FAR struct qspi_dev_s *qspi;
  FAR struct mtd_dev_s *mtd;
  FAR struct mtd_geometry_s geo;

  /* Get the 1SPI port */

  qspi = sam_qspi_initialize(W25QXXXJV_QSPI_PORT);
  if (!qspi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize QSPI port %d\n",
             W25QXXXJV_QSPI_PORT);
      return -ENODEV;
    }

  /* Now bind the SPI interface to the W25 SPI FLASH driver */

  mtd = w25qxxxjv_initialize(qspi, true);
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind QSPI port %d to the Winbond"
                      "W25QXXXJV FLASH driver\n", W25QXXXJV_QSPI_PORT);
      return -ENODEV;
    }

#ifndef CONFIG_FS_SMARTFS
  /* And use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(minor, mtd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Initialize the FTL layer\n");
      return ret;
    }
#else
  /* Initialize to provide SMARTFS on the MTD interface */

  /* Get the geometry of the FLASH device */

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mtd->ioctl failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_BRCG2_FLASH_PART
    {
      int partno;
      int partsize;
      int partoffset;
      int partszbytes;
      int erasesize;
      const char *partstring = CONFIG_BRCG2_FLASH_PART_LIST;
      const char *ptr;
      FAR struct mtd_dev_s *mtd_part;
      char  partref[4];

      /* Now create a partition on the FLASH device */

      partno = 0;
      ptr = partstring;
      partoffset = 0;

      /* Get the Flash erase size */

      erasesize = geo.erasesize;

      while (*ptr != '\0')
        {
          /* Get the partition size */

          partsize = atoi(ptr);
          partszbytes = (partsize << 10); /* partsize is defined in KB */

          /* Check if partition size is bigger then erase block */

          if (partszbytes < erasesize)
            {
              syslog(LOG_ERR,
                     "ERROR: Partition size is lesser than erasesize!\n");
              return -1;
            }

          /* Check if partition size is multiple of erase block */

          if ((partszbytes % erasesize) != 0)
            {
              syslog(LOG_ERR,
                     "ERROR: Partition size isn't multiple of erasesize!\n");
              return -1;
            }

          mtd_part = mtd_partition(mtd, partoffset, partszbytes / erasesize);
          partoffset += partszbytes / erasesize;

#ifdef CONFIG_BRCG2_FLASH_CONFIG_PART
          /* Test if this is the config partition */

          if (CONFIG_BRCG2_FLASH_CONFIG_PART_NUMBER == partno)
            {
              /* Register the partition as the config device */

              mtdconfig_register(mtd_part);
            }
          else
#endif
            {
              /* Now initialize a SMART Flash block device and bind it
               * to the MTD device.
               */

#if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
              sprintf(partref, "p%d", partno);
              smart_initialize(CONFIG_BRCG2_FLASH_MINOR,
                               mtd_part, partref);
#endif
            }

          /* Update the pointer to point to the next size in the list */

          while ((*ptr >= '0') && (*ptr <= '9'))
            {
              ptr++;
            }

          if (*ptr == ',')
            {
              ptr++;
            }

          /* Increment the part number */

          partno++;
        }
    }
#else /* CONFIG_BRCG2_FLASH_PART */

  /* Configure the device with no partition support */

  smart_initialize(CONFIG_BRCG2_FLASH_MINOR, mtd, NULL);

#endif /* CONFIG_BRCG2_FLASH_PART */
#endif /* CONFIG_FS_SMARTFS */
#endif /* HAVE_W25 */

  return OK;
}
