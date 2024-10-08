/****************************************************************************
 * libs/libc/stdio/lib_remove.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: remove
 *
 * Description:
 *   The remove() function causes the object denoted by path to be removed.
 *   The function is equivalent to unlink() or rmdir().
 *
 * Input Parameters:
 *   path - A pointer to a file or directory to be removed.
 *
 * Returned Value:
 *   0(OK) on success; -1(ERROR) on failure with errno set appropriately:
 *
 *   For returned errno values, see unlink or rmdir.
 *
 ****************************************************************************/

int remove(FAR const char *path)
{
  /* First try to unlink since this is
   * more frequently the necessary action.
   */

  if (unlink(path) != 0 && (get_errno() != EISDIR || rmdir(path) != 0))
    {
      /* Cannot remove the object for whatever reason. */

      return -1;
    }

  return 0;
}
