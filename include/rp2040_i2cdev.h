/****************************************************************************
 * boards/arm/rp2040/hybrid-pcc/include/rp2040_i2cdev.h
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

#ifndef __BOARDS_ARM_RP2040_HYBRID_PCC_INCLUDE_RP2040_I2CDEV_H
#define __BOARDS_ARM_RP2040_HYBRID_PCC_INCLUDE_RP2040_I2CDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_i2cdev_initialize
 *
 * Description:
 *   Initialize i2c driver and register the /dev/i2c device.
 *
 * Input Parameters:
 *   bus - The RP2040 I2C bus to initialize.  0: I2C0, 1: I2C1.  Which ones
 *         are actually available depends on the board configuration.
 *
 ****************************************************************************/

#ifdef CONFIG_RP2040_I2C_DRIVER
int board_i2cdev_initialize(int bus);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_RP2040_HYBRID_PCC_INCLUDE_RP2040_I2CDEV_H */
