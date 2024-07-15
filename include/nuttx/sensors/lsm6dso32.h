/****************************************************************************
 * include/nuttx/sensors/lsm6dso32.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LSM6DSO32_H
#define __INCLUDE_NUTTX_SENSORS_LSM6DSO32_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s; /* Forward reference */

/****************************************************************************
 * Name: lsm6dso32_register
 *
 * Description:
 *   Registers the LSM6DSO32 interface as 'devpath'
 *
 * Input Parameters:
 *   devpath  - The full path to the interface to register. E.g., "/dev/imu0"
 *   i2c      - I2C interface for chip communications
 *   addr     - I2C slave address for the sensor
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm6dso32_register(FAR const char *path, struct i2c_master_s *i2c,
                       uint8_t addr);

#endif // __INCLUDE_NUTTX_SENSORS_LSM6DSO32_H
