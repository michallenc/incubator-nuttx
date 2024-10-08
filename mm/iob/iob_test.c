/****************************************************************************
 * mm/iob/iob_test.c
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
 ****************************************************************************/

/****************************************************************************
 * Unit test driver.  This is of historical interest only since it requires
 * and custom build setup and modifications to the iob source and header
 * files.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "iob.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t buffer1[16384];
static uint8_t buffer2[16384];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void dump_chain(struct iob_s *iob)
{
  struct iob_s *head = iob;
  unsigned int pktlen;
  int n;

  printf("=========================================================\n");
  printf("pktlen: %d\n", iob->io_pktlen);

  n = 0;
  pktlen = 0;

  while (iob)
    {
      printf("%d. len=%d, offset=%d\n", n, iob->io_len, iob->io_offset);

      pktlen += iob->io_len;
      iob = iob->io_flink;
      n++;
    }

  if (pktlen != head->io_pktlen)
    {
      printf("ERROR: Bad packet length=%u, actual=%u\n",
             head->io_pktlen, pktlen);
    }

  printf("=========================================================\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   A simple unit test for the I/O buffer logic
 *
 ****************************************************************************/

int main(int argc, char **argv)
{
  struct iob_s *iob;
  int nbytes;
  int i;

  iob_initialize();
  iob = iob_alloc(false);

  for (i = 0; i < 4096; i++)
    {
      buffer1[i] = (uint8_t)(i & 0xff);
    }

  memset(buffer2, 0xff, 4096);

  iob_copyin(iob, buffer2, 47, 0, false);
  printf("Copy IN: 47, offset 0\n");
  dump_chain(iob);

  iob_copyin(iob, buffer1, 4096, 47, false);
  printf("Copy IN: 4096, offset 47\n");
  dump_chain(iob);

  nbytes = iob_copyout(buffer2, iob, 4096, 47);
  printf("Copy OUT: %d, offset 47\n", nbytes);

  if (memcmp(buffer1, buffer2, nbytes) != 0)
    {
      fprintf(stderr, "Buffer1 does not match buffer2\n");
    }

  iob = iob_trimhead(iob, 47);
  printf("Trim: 47 from the beginning of the list\n");
  dump_chain(iob);

  iob = iob_trimtail(iob, 493);
  printf("Trim: 493 from the end of the list\n");
  dump_chain(iob);

  nbytes = iob_copyout(buffer2, iob, 4096, 0);
  printf("Copy OUT: %d, offset 0\n", nbytes);

  if (memcmp(buffer1, buffer2, nbytes) != 0)
    {
      fprintf(stderr, "Buffer1 does not match buffer2\n");
    }

  iob = iob_trimhead(iob, 1362);
  printf("Trim: 1362 from the beginning of the list\n");
  dump_chain(iob);

  nbytes = iob_copyout(buffer2, iob, 4096, 0);
  printf("Copy OUT: %d, offset 0\n", nbytes);

  if (memcmp(&buffer1[1362], buffer2, nbytes) != 0)
    {
      fprintf(stderr, "Buffer1 does not match buffer2\n");
    }

  iob = iob_pack(iob);
  printf("Packed\n");
  dump_chain(iob);

  nbytes = iob_copyout(buffer2, iob, 4096, 0);
  printf("Copy OUT: %d, offset 0\n", nbytes);

  if (memcmp(&buffer1[1362], buffer2, nbytes) != 0)
    {
      fprintf(stderr, "Buffer1 does not match buffer2\n");
    }

  nbytes = iob->io_pktlen;

  iob_reserve(iob, 55);
  printf("Reserve: adjust offset to 55\n");
  dump_chain(iob);

  if (iob->io_offset != 55 || iob->io_pktlen + 55 != nbytes)
    {
      fprintf(stderr, "Offset or packet length wrong\n");
    }

  iob_reserve(iob, 28);
  printf("Reserve: adjust offset to 28\n");
  dump_chain(iob);

  if (iob->io_offset != 28 || iob->io_pktlen + 28 != nbytes)
    {
      fprintf(stderr, "Offset or packet length wrong\n");
    }

  while (iob) iob = iob_free(iob);
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Name: my_assert
 *
 * Description:
 *   A stand-in for the NuttX assertion routine.
 *
 ****************************************************************************/

void my_assert(bool value)
{
  if (!value)
    {
      fprintf(stderr, "Assertion failed\n");

      abort();
    }
}
