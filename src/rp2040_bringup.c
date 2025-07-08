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

#ifdef CONFIG_RP2040_PWM5
#include <nuttx/panic_notifier.h>
#include <nuttx/timers/pwm.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#endif

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

#ifdef CONFIG_SENSORS_MCP9600
#include <nuttx/sensors/mcp9600.h>
#include "rp2040_i2c.h"
#endif

#ifdef CONFIG_ADC_ADS1115
#include "rp2040_i2c.h"
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ads1115.h>
#endif

#ifdef CONFIG_RP2040_PWM5
static struct notifier_block panic_notifier;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_RP2040_PWM5
static int panic_open_dump_valve(struct notifier_block *nb, unsigned long action, void *data) {
    int fd;
    int err;
    struct pwm_info_s pwm_config;

    fd = open("/dev/pwm5", O_RDWR);
    if (fd < 0) {
        return -1;
    }

    /* WARNING: make sure this matches the configuration settings for the dump
     * valve.
     * TODO: this should probably be a Kconfig option.
     */

    pwm_config.frequency = 250;
    pwm_config.channels[1].channel = 1;
    pwm_config.channels[1].cpol = 1;
    pwm_config.channels[1].dcpol = 0;
    pwm_config.channels[1].duty = 0x4000;

    err = ioctl(fd, PWMIOC_SETCHARACTERISTICS, &pwm_config);
    if (err) {
        return -1;
    }

    err = ioctl(fd, PWMIOC_START, NULL);
    up_udelay(1e6);
    return err;
}
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

#ifdef CONFIG_ADC_ADS1115
  uint8_t ads1115_addrs[3] = {0x48, 0x49, 0x4A};
  char *ads1115_devpaths[3] = {"/dev/adc0", "/dev/adc1", "/dev/adc2"};

  for (int i = 0; i < 3; i++)
  {
    struct adc_dev_s *ads1115 =
        ads1115_initialize(rp2040_i2cbus_initialize(0), ads1115_addrs[i]);

    if (ads1115 == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize ADS1115 at address 0x%02X\n",
             ads1115_addrs[i]);
      continue;
    }

    ret = adc_register(ads1115_devpaths[i], ads1115);
    if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register ADS1115 device driver at %s: %d\n",
             ads1115_devpaths[i], ret);
    }
  }
#endif

#ifdef CONFIG_SENSORS_NAU7802
  ret = nau7802_register(rp2040_i2cbus_initialize(0), 0, 0x2A);
  if (ret < 0)
  {
    syslog(LOG_ERR, "ERROR: couldn't initialize NAU7802: %d\n", ret);
  }
#endif

#ifdef CONFIG_SENSORS_MCP9600
  /*
    Registration args for the topics are: hot junction, cold junction, delta (in this order)
    cold junction gets ambient temperature topic
    hot junction and delta get temperature topic
    common bringup registers the default sensors with the following args: 1, 2, 3
    the following registration will set the hot junction to have topics 2 and 5
  */

  ret = mcp9600_register(rp2040_i2cbus_initialize(0), 0x66, 2, 0, 4);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Could not register MCP9600 at 0x66: %d\n", ret);
  }

  ret = mcp9600_register(rp2040_i2cbus_initialize(0), 0x67, 5, 1, 6);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Could not register MCP9600 at 0x67: %d\n", ret);
  }
#endif

  /* Dump valve panic handler to open on crash.
   * WARNING: This panic handler duplicates the PWM configuration of the
   * application. Ensure it is up to date.
   */
#ifdef CONFIG_RP2040_PWM5
    panic_notifier.notifier_call = panic_open_dump_valve;
    panic_notifier.priority = 0;
    panic_notifier_chain_register(&panic_notifier);
#endif

  return OK;
}
