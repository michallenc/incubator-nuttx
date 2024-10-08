/****************************************************************************
 * libs/libc/tls/task_tls.c
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

#include <errno.h>
#include <nuttx/tls.h>

#if defined(CONFIG_TLS_TASK_NELEM) && CONFIG_TLS_TASK_NELEM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_tls_get_value
 *
 * Description:
 *   Return an the task local storage data value associated with 'tlsindx'
 *
 * Input Parameters:
 *   tlsindex - Index of task local storage data element to return
 *
 * Returned Value:
 *   The value of TLS element associated with 'tlsindex'. Errors are not
 *   reported.  Zero is returned in the event of an error, but zero may also
 *   be valid value and returned when there is no error.  The only possible
 *   error would be if tlsindex < 0 or tlsindex >=CONFIG_TLS_TASK_NELEM.
 *
 ****************************************************************************/

uintptr_t task_tls_get_value(int tlsindex)
{
  FAR struct task_info_s *info = task_get_info();

  if (tlsindex >= 0 && tlsindex < CONFIG_TLS_TASK_NELEM)
    {
      return info->ta_telem[tlsindex];
    }

  return 0;
}

/****************************************************************************
 * Name: task_tls_set_value
 *
 * Description:
 *   Set the task local storage element associated with the 'tlsindex' to
 *   'tlsvalue'
 *
 * Input Parameters:
 *   tlsindex - Index of task local storage data element to set
 *   tlsvalue - The new value of the task local storage data element
 *
 * Returned Value:
 *   Zero is returned on success, a negated errno value is return on
 *   failure:
 *
 *     EINVAL - tlsindex is not in range.
 *
 ****************************************************************************/

int task_tls_set_value(int tlsindex, uintptr_t tlsvalue)
{
  FAR struct task_info_s *info = task_get_info();

  if (tlsindex >= 0 && tlsindex < CONFIG_TLS_TASK_NELEM)
    {
      info->ta_telem[tlsindex] = tlsvalue;
    }
  else
    {
      return -ERANGE;
    }

  return OK;
}

#endif
