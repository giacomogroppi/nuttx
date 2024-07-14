/****************************************************************************
 * drivers/sensors/lsm6dso32.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <time.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/sensors/lsm6dso32.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM6DSO32)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/

struct lsm6dso32_dev_s {
    FAR struct i2c_master_s *i2c; /* I2C interface */
    uint8_t addr;                 /* I2C address */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    bool unlinked; /* If true, driver has been unlinked */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    int16_t crefs; /* Number of open references */
#endif
    mutex_t devlock;
};

/** See datasheet section 8, and section 9 for descriptions. */
enum lsm6dso32_reg_e {
    FUNC_CFG_ACCESS = 0x01,          /**< Enable embedded functions register. */
    PIN_CTRL = 0x02,                 /**< SDO pin pull-up enable/disable register. */
    FIFO_CTRL1 = 0x07,               /**< FIFO control register 1. */
    FIFO_CTRL2 = 0x08,               /**< FIFO control register 2. */
    FIFO_CTRL3 = 0x09,               /**< FIFO control register 3. */
    FIFO_CTRL4 = 0x0A,               /**< FIFO control register 4. */
    COUNTER_BDR_REG1 = 0x0B,         /**< Counter batch data register 1. */
    COUNTER_BDR_REG2 = 0x0C,         /**< Counter batch data register 2. */
    INT1_CTRL = 0x0D,                /**< INT1 pin control register. */
    INT2_CTRL = 0x0E,                /**< INT2 pin control register. */
    WHO_AM_I = 0x0F,                 /**< One byte fixed at 0x6C. */
    CTRL1_XL = 0x10,                 /**< Accelerometer control register 1. */
    CTRL2_G = 0x11,                  /**< Gyroscope control register 2. */
    CTRL3_C = 0x12,                  /**< Control register 3. */
    CTRL4_C = 0x13,                  /**< Control register 4. */
    CTRL5_C = 0x14,                  /**< Control register 5. */
    CTRL6_C = 0x15,                  /**< Control register 6. */
    CTRL7_G = 0x16,                  /**< Control register 7. */
    CTRL8_XL = 0x17,                 /**< Control register 8. */
    CTRL9_XL = 0x18,                 /**< Control register 9. */
    CTRL10_C = 0x19,                 /**< Control register 10. */
    ALL_INT_SRC = 0x1A,              /**< Source register for all interrupts. */
    WAKE_UP_SRC = 0x1B,              /**< Wake-up interrupt source register. */
    TAP_SRC = 0x1C,                  /**< Tap source register. */
    D6D_SRC = 0x1D,                  /**< Portrait, landscape, face-up and face-down source register. */
    STATUS_REG = 0x1E,               /**< Status register. */
    OUT_TEMP_L = 0x20,               /**< Temperature data output register (low byte). */
    OUT_TEMP_H = 0x21,               /**< Temperature data output register (high byte). */
    OUTX_L_G = 0x22,                 /**< Angular rate in X direction output register (low byte). */
    OUTX_H_G = 0x23,                 /**< Angular rate in X direction output register (high byte). */
    OUTY_L_G = 0x24,                 /**< Angular rate in Y direction output register (low byte). */
    OUTY_H_G = 0x25,                 /**< Angular rate in Y direction output register (high byte). */
    OUTZ_L_G = 0x26,                 /**< Angular rate in Z direction output register (low byte). */
    OUTZ_H_G = 0x27,                 /**< Angular rate in Z direction output register (high byte). */
    OUTX_L_A = 0x28,                 /**< Linear acceleration in X direction output register (low byte). */
    OUTX_H_A = 0x29,                 /**< Linear acceleration in X direction output register (high byte). */
    OUTY_L_A = 0x2A,                 /**< Linear acceleration in Y direction output register (low byte). */
    OUTY_H_A = 0x2B,                 /**< Linear acceleration in Y direction output register (high byte). */
    OUTZ_L_A = 0x2C,                 /**< Linear acceleration in Z direction output register (low byte). */
    OUTZ_H_A = 0x2D,                 /**< Linear acceleration in Z direction output register (high byte). */
    EMB_FUNC_STATUS_MAINPAGE = 0x35, /**< Embedded function status register. */
    FSM_STATUS_A_MAINPAGE = 0x36,    /**< Finite state machine status register. */
    FSM_STATUS_B_MAINPAGE = 0x37,    /**< Finite state machine status register. */
    STATUS_MASTER_MAINPAGE = 0x39,   /** Sensor hub source register. */
    FIFO_STATUS1 = 0x3A,             /**< FIFO status register 1. */
    FIFO_STATUS2 = 0x3B,             /**< FIFO status register 2. */
    TIMESTAMP0 = 0x40,               /**< Timestamp output register 1 (LSB). */
    TIMESTAMP1 = 0x41,               /**< Timestamp output register 2. */
    TIMESTAMP2 = 0x42,               /**< Timestamp output register 3. */
    TIMESTAMP3 = 0x43,               /**< Timestamp output register 4 (MSB). */
    TAP_CFG0 = 0x56,           /**< Activity/inactivity functions, config of filtering and tap recognition functions. */
    TAP_CFG1 = 0x57,           /**< Tap configuration register. */
    TAP_CFG2 = 0x58,           /**< Enables interrupt and inactivity functions, and tap recognition functions. */
    TAP_THS_6D = 0x59,         /**< Portrait/landscape position and tap function threshold register. */
    INT_DUR2 = 0x5A,           /**< Tap recognition function setting register. */
    WAKE_UP_THS = 0x5B,        /**< Single/double-tap selection and wake-up configuration. */
    WAKE_UP_DUR = 0x5C,        /**< Free-fall, wakeup and sleep mode functions duration setting register. */
    FREE_FALL = 0x5D,          /**< Free-fall function duration setting register. */
    MD1_CFG = 0x5E,            /**< Functions routing on INT1 register. */
    MD2_CFG = 0x5F,            /**< Functions routing on INT2 register. */
    I3C_BUS_AVB = 0x62,        /**< I3B_BUS_AVB register. */
    INTERNAL_FREQ_FINE = 0x63, /**< Internal frequency register. */
    X_OFS_USR = 0x73,          /**< Accelerometer x-axis user offset correction. */
    Y_OFS_USR = 0x74,          /**< Accelerometer y-axis user offset correction. */
    Z_OFS_USR = 0x75,          /**< Accelerometer z-axis user offset correction. */
    FIFO_DATA_OUT_TAG = 0x78,  /**< FIFO tag register. */
    FIFO_DATA_OUT_X_L = 0x79,  /**< FIFO data output X (low byte). */
    FIFO_DATA_OUT_X_H = 0x7A,  /**< FIFO data output X (high byte). */
    FIFO_DATA_OUT_Y_L = 0x7B,  /**< FIFO data output Y (low byte). */
    FIFO_DATA_OUT_Y_H = 0x7C,  /**< FIFO data output Y (high byte). */
    FIFO_DATA_OUT_Z_L = 0x7D,  /**< FIFO data output Z (low byte). */
    FIFO_DATA_OUT_Z_H = 0x7E,  /**< FIFO data output Z (high byte). */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int lsm6dso32_open(FAR struct file *filep);
static int lsm6dso32_close(FAR struct file *filep);
#endif
static ssize_t lsm6dso32_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t lsm6dso32_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int lsm6dso32_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int lsm6dso32_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lsm6dso32fops = {
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    .open = lsm6dso32_open,
    .close = lsm6dso32_close,
#else
    .open = NULL,
    .close = NULL,
#endif
    .read = lsm6dso32_read,
    .write = lsm6dso32_write,
    .seek = NULL,
    .ioctl = lsm6dso32_ioctl,
    .mmap = NULL,
    .truncate = NULL,
    .poll = NULL,
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    .unlink = lsm6dso32_unlink,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#endif // defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM6DSO32)
