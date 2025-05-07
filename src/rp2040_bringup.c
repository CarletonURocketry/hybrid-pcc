/****************************************************************************
 * boards/arm/rp2040/hybrid-pcc/src/rp2040_bringup.c
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

#include <debug.h>
#include <stddef.h>

#include <nuttx/fs/fs.h>

#include <arch/board/board.h>

#include "rp2040_pico.h"

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "rp2040_common_bringup.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

#ifdef CONFIG_USERLED
#include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_SENSORS_NAU7802
#include <nuttx/sensors/nau7802.h>
#include "rp2040_i2c.h"
#endif

#ifdef CONFIG_SENSORS_MCP9600
#include <nuttx/sensors/mcp9600.h>
#include "rp2040_i2c.h"
#endif

#ifdef CONFIG_ADC_ADS1115
#include "rp2040_i2c.h"
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ads1115.h>
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_bringup
 ****************************************************************************/

int rp2040_bringup(void)
{
#ifdef CONFIG_ARCH_BOARD_COMMON

  int ret = rp2040_common_bringup();
  if (ret < 0)
  {
    return ret;
  }

#endif /* CONFIG_ARCH_BOARD_COMMON */

  /* --- Place any board specific bringup code here --- */

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
  {
    syslog(LOG_ERR,
           "ERROR: userled_lower_initialize() failed: %d\n", ret);
  }
#endif

#ifdef CONFIG_SENSORS_NAU7802

  /* Try to register NAU7802 device on I2C0 */

  ret = nau7802_register(rp2040_i2cbus_initialize(0), 0, 0x2A);
  if (ret < 0)
  {
    syslog(LOG_ERR, "ERROR: couldn't initialize NAU7802: %d\n", ret);
  }
#endif

#ifdef CONFIG_ADC_ADS1115
  int ads1115_addrs[3] = {0x48, 0x49, 0x4A};

  /* Register the ADS1115 ADC driver */

  for (int i = 0; i < 3; i++)
  {
    struct adc_dev_s *ads1115 =
        ads1115_initialize(rp2040_i2cbus_initialize(0), ads1115_addrs[i]);
    if (ads1115 == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize ADS1115 at address 0x%02X\n",
             ads1115_addrs[i]);
      continue; // Skip to next device if initialization fails
    }

    char devpath[16];
    snprintf(devpath, sizeof(devpath), "/dev/adc%d", i);

    ret = adc_register(devpath, ads1115);
    if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register ADS1115 device driver at %s: %d\n",
             devpath, ret);
    }
  }

#endif

#ifdef CONFIG_SENSORS_MCP9600

  /* Register both thermocouple amplifiers at 0x66 and 0x67 */

  ret = mcp9600_register(rp2040_i2cbus_initialize(0), 0x66, 1, 2, 3);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to register MCP9600 at 0x66: %d\n", ret);
  }

  ret = mcp9600_register(rp2040_i2cbus_initialize(0), 0x67, 4, 5, 6);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to register MCP9600 at 0x67: %d\n", ret);
  }

#endif

  return OK;
}
