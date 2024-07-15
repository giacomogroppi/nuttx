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
#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/lsm6dso32.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM6DSO32)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LSM6DSO32_FREQ
#define CONFIG_LSM6DSO32_FREQ I2C_SPEED_FAST
#endif // CONFIG_LSM6DSO32_FREQ

#ifndef CONFIG_LSM6DSO32_ADDR
#define CONFIG_LSM6DSO32_ADDR 0x6b
#endif // CONFIG_LSM6DSO32_ADDR

/****************************************************************************
 * Private
 ****************************************************************************/

struct lsm6dso32_dev_s
{
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

/* See datasheet section 8, and section 9 for descriptions. */

enum lsm6dso32_reg_e
{
    FUNC_CFG_ACCESS = 0x01,  /* Enable embedded functions register. */
    PIN_CTRL = 0x02,         /* SDO pin pull-up enable/disable register. */
    FIFO_CTRL1 = 0x07,       /* FIFO control register 1. */
    FIFO_CTRL2 = 0x08,       /* FIFO control register 2. */
    FIFO_CTRL3 = 0x09,       /* FIFO control register 3. */
    FIFO_CTRL4 = 0x0a,       /* FIFO control register 4. */
    COUNTER_BDR_REG1 = 0x0b, /* Counter batch data register 1. */
    COUNTER_BDR_REG2 = 0x0c, /* Counter batch data register 2. */
    INT1_CTRL = 0x0d,        /* INT1 pin control register. */
    INT2_CTRL = 0x0e,        /* INT2 pin control register. */
    WHO_AM_I = 0x0f,         /* One byte fixed at 0x6C. */
    CTRL1_XL = 0x10,         /* Accelerometer control register 1. */
    CTRL2_G = 0x11,          /* Gyroscope control register 2. */
    CTRL3_C = 0x12,          /* Control register 3. */
    CTRL4_C = 0x13,          /* Control register 4. */
    CTRL5_C = 0x14,          /* Control register 5. */
    CTRL6_C = 0x15,          /* Control register 6. */
    CTRL7_G = 0x16,          /* Control register 7. */
    CTRL8_XL = 0x17,         /* Control register 8. */
    CTRL9_XL = 0x18,         /* Control register 9. */
    CTRL10_C = 0x19,         /* Control register 10. */
    ALL_INT_SRC = 0x1a,      /* Source register for all interrupts. */
    WAKE_UP_SRC = 0x1b,      /* Wake-up interrupt source register. */
    TAP_SRC = 0x1c,          /* Tap source register. */
    D6D_SRC = 0x1d,          /* Portrait, landscape, face-up and face-down
                              * source register. */
    STATUS_REG = 0x1e,       /* Status register. */

    OUT_TEMP_L = 0x20, /* Temperature data output register (low byte). */
    OUT_TEMP_H = 0x21, /* Temperature data output register (high byte). */

    OUTX_L_G = 0x22, /* Angular rate in X direction output register
                      * (low byte). */
    OUTX_H_G = 0x23, /* Angular rate in X direction output register
                      * (high byte). */
    OUTY_L_G = 0x24, /* Angular rate in Y direction output register
                      * (low byte). */
    OUTY_H_G = 0x25, /* Angular rate in Y direction output register
                      * (high byte). */
    OUTZ_L_G = 0x26, /* Angular rate in Z direction output register
                      * (low byte). */
    OUTZ_H_G = 0x27, /* Angular rate in Z direction output register
                      * (high byte). */
    OUTX_L_A = 0x28, /* Linear acceleration in X direction output register
                      * (low byte). */
    OUTX_H_A = 0x29, /* Linear acceleration in X direction output register
                      * (high byte). */
    OUTY_L_A = 0x2a, /* Linear acceleration in Y direction output register
                      * (low byte). */
    OUTY_H_A = 0x2b, /* Linear acceleration in Y direction output register
                      * (high byte). */
    OUTZ_L_A = 0x2c, /* Linear acceleration in Z direction output register
                      * (low byte). */
    OUTZ_H_A = 0x2d, /* Linear acceleration in Z direction output register
                      * (high byte). */

    EMB_FUNC_STATUS_MAINPAGE = 0x35, /* Embedded function status register. */

    FSM_STATUS_A_MAINPAGE = 0x36, /* Finite state machine status register. */
    FSM_STATUS_B_MAINPAGE = 0x37, /* Finite state machine status register. */

    STATUS_MASTER_MAINPAGE = 0x39, /* Sensor hub source register. */
    FIFO_STATUS1 = 0x3a,           /* FIFO status register 1. */
    FIFO_STATUS2 = 0x3b,           /* FIFO status register 2. */
    TIMESTAMP0 = 0x40,             /* Timestamp output register 1 (LSB). */
    TIMESTAMP1 = 0x41,             /* Timestamp output register 2. */
    TIMESTAMP2 = 0x42,             /* Timestamp output register 3. */
    TIMESTAMP3 = 0x43,             /* Timestamp output register 4 (MSB). */

    TAP_CFG0 = 0x56, /* Activity/inactivity functions, config of filtering
                      * and tap recognition functions. */
    TAP_CFG1 = 0x57, /* Tap configuration register. */
    TAP_CFG2 = 0x58, /* Enables interrupt and inactivity functions, and tap
                      * recognition functions. */

    TAP_THS_6D = 0x59,  /* Portrait/landscape position and tap function
                         * threshold register. */
    INT_DUR2 = 0x5a,    /* Tap recognition function setting register. */
    WAKE_UP_THS = 0x5b, /* Single/double-tap selection and wake-up
                         * configuration. */
    WAKE_UP_DUR = 0x5c, /* Free-fall, wakeup and sleep mode functions
                         * duration setting register. */
    FREE_FALL = 0x5d,   /* Free-fall function duration setting register. */
    MD1_CFG = 0x5e,     /* Functions routing on INT1 register. */
    MD2_CFG = 0x5f,     /* Functions routing on INT2 register. */
    I3C_BUS_AVB = 0x62, /* I3B_BUS_AVB register. */

    INTERNAL_FREQ_FINE = 0x63, /* Internal frequency register. */

    X_OFS_USR = 0x73, /* Accelerometer x-axis user offset correction. */
    Y_OFS_USR = 0x74, /* Accelerometer y-axis user offset correction. */
    Z_OFS_USR = 0x75, /* Accelerometer z-axis user offset correction. */

    FIFO_DATA_OUT_TAG = 0x78, /* FIFO tag register. */
    FIFO_DATA_OUT_X_L = 0x79, /* FIFO data output X (low byte). */
    FIFO_DATA_OUT_X_H = 0x7a, /* FIFO data output X (high byte). */
    FIFO_DATA_OUT_Y_L = 0x7b, /* FIFO data output Y (low byte). */
    FIFO_DATA_OUT_Y_H = 0x7c, /* FIFO data output Y (high byte). */
    FIFO_DATA_OUT_Z_L = 0x7d, /* FIFO data output Z (low byte). */
    FIFO_DATA_OUT_Z_H = 0x7e, /* FIFO data output Z (high byte). */
};

/* Embedded functions registers. See section 10 of data sheet, and section 11
 * for description.
 */

enum lsm6dso32_embreg_e
{
    PAGE_SEL = 0x02,        /* Enable advanced features dedicated page. */
    EMB_FUNC_EN_A = 0x04,   /* Embedded functions enable register. */
    EMB_FUNC_EN_B = 0x05,   /* Embedded functions enable register. */
    PAGE_ADDRESS = 0x08,    /* Page address register. */
    PAGE_VALUE = 0x09,      /* Page value register. */
    EMB_FUNC_INT1 = 0x0a,   /* INT1 pin control register. */
    FSM_INT1_A = 0x0b,      /* INT1 pin control register. */
    FSM_INT1_B = 0x0c,      /* INT1 pin control register. */
    EMB_FUNC_INT2 = 0x0e,   /* INT2 pin control register. */
    FSM_INT2_A = 0x0f,      /* INT2 pin control register. */
    FSM_INT2_B = 0x10,      /* INT2 pin control register. */
    EMB_FUNC_STATUS = 0x12, /* Embedded function status register. */
    FSM_STATUS_A = 0x13,    /* Finite state machine status register. */
    FSM_STATUS_B = 0x14,    /* Finite state machine status register. */

    PAGE_RW = 0x17, /* Enable read and write mode of advanced features
                     * dedicated page. */

    EMB_FUNC_FIFO_CFG = 0x44, /* Embedded functions batching configuration
                               * register. */

    FSM_ENABLE_A = 0x46, /* FSM enable register. */
    FSM_ENABLE_B = 0x47, /* FSM enable register. */

    FSM_LONG_COUNTER_L = 0x48, /* FSM long counter status register
                                * (low byte). */
    FSM_LONG_COUNTER_H = 0x49, /* FSM long counter status register
                                * (high byte). */

    FSM_LONG_COUNTER_CLEAR = 0x4a, /* FSM long counter reset register. */
    FSM_OUTS1 = 0x4c,              /* FSM output register. */
    FSM_OUTS2 = 0x4d,              /* FSM output register. */
    FSM_OUTS3 = 0x4e,              /* FSM output register. */
    FSM_OUTS4 = 0x4f,              /* FSM output register. */
    FSM_OUTS5 = 0x50,              /* FSM output register. */
    FSM_OUTS6 = 0x51,              /* FSM output register. */
    FSM_OUTS7 = 0x52,              /* FSM output register. */
    FSM_OUTS8 = 0x53,              /* FSM output register. */
    FSM_OUTS9 = 0x54,              /* FSM output register. */
    FSM_OUTS10 = 0x55,             /* FSM output register. */
    FSM_OUTS11 = 0x56,             /* FSM output register. */
    FSM_OUTS12 = 0x57,             /* FSM output register. */
    FSM_OUTS13 = 0x58,             /* FSM output register. */
    FSM_OUTS14 = 0x59,             /* FSM output register. */
    FSM_OUTS15 = 0x5a,             /* FSM output register. */
    FSM_OUTS16 = 0x5b,             /* FSM output register. */

    EMB_FUNC_ODR_CFG_B = 0x5f, /* Finite state machine output data rate
                                * configuration register. */

    STEP_COUNTER_L = 0x62,  /* Step counter output register (low byte). */
    STEP_COUNTER_H = 0x63,  /* Step counter output register (high byte). */
    EMB_FUNC_SRC = 0x64,    /* Embedded function source register. */
    EMB_FUNC_INIT_A = 0x66, /* Embedded functions initialization register. */
    EMB_FUNC_INIT_B = 0x67, /* Embedded functions initialization register. */
};

/* Embedded advanced features registers for page 0. See section 12 of data
 * sheet, section 13 for descriptions.
 */

enum lsm6dso32_advembregp0_e
{
    MAG_SENSITIVITY_L = 0xba, /* External magnetometer sensitivity value
                               * register for FSM (low byte). */
    MAG_SENSITIVITY_H = 0xbb, /* External magnetometer sensitivity value
                               * register for FSM (high byte). */

    MAG_OFFX_L = 0xc0,  /* Offset for x-axis hard-iron compensation register
                         * (low byte). */
    MAG_OFFX_H = 0xc1,  /* Offset for x-axis hard-iron compensation register
                         * (high byte). */
    MAG_OFFY_L = 0xc2,  /* Offset for y-axis hard-iron compensation register
                         * (low byte). */
    MAG_OFFY_H = 0xc3,  /* Offset for y-axis hard-iron compensation register
                         * (high byte). */
    MAG_OFFZ_L = 0xc4,  /* Offset for z-axis hard-iron compensation register
                         * (low byte). */
    MAG_OFFZ_H = 0xc5,  /* Offset for z-axis hard-iron compensation register
                         * (high byte). */
    MAG_SI_XX_L = 0xc6, /* Soft-iron (3x3 symmetric) matrix correction
                         * register (low byte). */
    MAG_SI_XX_H = 0xc7, /* Soft-iron (3x3 symmetric) matrix correction
                         * register (high byte). */
    MAG_SI_XY_L = 0xc8, /* Soft-iron (3x3 symmetric) matrix correction
                         * register (low byte). */
    MAG_SI_XY_H = 0xc9, /* Soft-iron (3x3 symmetric) matrix correction
                         * register (high byte). */
    MAG_SI_XZ_L = 0xca, /* Soft-iron (3x3 symmetric) matrix correction
                         * register (low byte). */
    MAG_SI_XZ_H = 0xcb, /* Soft-iron (3x3 symmetric) matrix correction
                         * register (high byte). */
    MAG_SI_YY_L = 0xcc, /* Soft-iron (3x3 symmetric) matrix correction
                         * register (low byte). */
    MAG_SI_YY_H = 0xcd, /* Soft-iron (3x3 symmetric) matrix correction
                         * register (high byte). */
    MAG_SI_YZ_L = 0xce, /* Soft-iron (3x3 symmetric) matrix correction
                         * register (low byte). */
    MAG_SI_YZ_H = 0xcf, /* Soft-iron (3x3 symmetric) matrix correction
                         * register (high byte). */
    MAG_SI_ZZ_L = 0xd0, /* Soft-iron (3x3 symmetric) matrix correction
                         * register (low byte). */
    MAG_SI_ZZ_H = 0xd1, /* Soft-iron (3x3 symmetric) matrix correction
                         * register (high byte). */
    MAG_CFG_A = 0xd4,   /* External magnetometer coordinates (z and y axes)
                         * rotation register. */
    MAG_CFG_B = 0xd5,   /* External magnetometer coordinates (z and y axes)
                         * rotation register. */
};

/* Embedded advanced features registers for page 1. See section 12 of data
 * sheet, section 13 for descriptions.
 */

enum lsm6dso32_advembregp1_e
{
    FSM_LC_TIMEOUT_L = 0x7a, /* FSM long counter timeout register
                              * (low byte). */
    FSM_LC_TIMEOUT_H = 0x7b, /* FSM long counter timeout register
                              * (high byte). */
    FSM_PROGRAMS = 0x7c,     /* FSM programs register. */
    FSM_START_ADD_L = 0x7e,  /* FSM start address register (low byte). */
    FSM_START_ADD_H = 0x7f,  /* FSM start address register (high byte). */
    PEDO_CMD_REG = 0x83,     /* Pedometer configuration register. */

    PEDO_DEB_STEPS_CONF = 0x84, /* Pedometer debounce configuration
                                 * register. */
    PEDO_SC_DELTAT_L = 0xd0,    /* Time period register for step detection on
                                 * delta time (low byte). */
    PEDO_SC_DELTAT_H = 0xd1,    /* Time period register for step detection on
                                 * delta time (high byte). */
};

/* Sensor hub registers. See section 14 of data sheet, section 15 for
 * descriptions.
 */

enum lsm6dso32_senreg_e
{
    SENSOR_HUB_1 = 0x02,  /* Sensor hub output register. */
    SENSOR_HUB_2 = 0x03,  /* Sensor hub output register. */
    SENSOR_HUB_3 = 0x04,  /* Sensor hub output register. */
    SENSOR_HUB_4 = 0x05,  /* Sensor hub output register. */
    SENSOR_HUB_5 = 0x06,  /* Sensor hub output register. */
    SENSOR_HUB_6 = 0x07,  /* Sensor hub output register. */
    SENSOR_HUB_7 = 0x08,  /* Sensor hub output register. */
    SENSOR_HUB_8 = 0x09,  /* Sensor hub output register. */
    SENSOR_HUB_9 = 0x0a,  /* Sensor hub output register. */
    SENSOR_HUB_10 = 0x0b, /* Sensor hub output register. */
    SENSOR_HUB_11 = 0x0c, /* Sensor hub output register. */
    SENSOR_HUB_12 = 0x0d, /* Sensor hub output register. */
    SENSOR_HUB_13 = 0x0e, /* Sensor hub output register. */
    SENSOR_HUB_14 = 0x0f, /* Sensor hub output register. */
    SENSOR_HUB_15 = 0x10, /* Sensor hub output register. */
    SENSOR_HUB_16 = 0x11, /* Sensor hub output register. */
    SENSOR_HUB_17 = 0x12, /* Sensor hub output register. */
    SENSOR_HUB_18 = 0x13, /* Sensor hub output register. */
    MASTER_CONFIG = 0x14, /* Master configuration register. */

    SLV0_ADD = 0x15,    /* I2C slave address of sensor 1 register. */
    SLV0_SUBADD = 0x16, /* Address of register on sensor 1 external
                         * register. */
    SLV0_CONFIG = 0x17, /* Sensor 1 configuration and sensor hub settings
                         * register. */
    SLV2_ADD = 0x28,    /* I2C slave address of sensor 2 register. */
    SLV2_SUBADD = 0x29, /* Address of register on sensor 2 external
                         * register. */
    SLV2_CONFIG = 0x2a, /* Sensor 2 configuration and sensor hub settings
                         * register. */
    SLV1_ADD = 0x3b,    /* I2C slave address of sensor 3 register. */
    SLV1_SUBADD = 0x3c, /* Address of register on sensor 3 external
                         * register. */
    SLV1_CONFIG = 0x3d, /* Sensor 3 configuration and sensor hub settings
                         * register. */
    SLV3_ADD = 0x4e,    /* I2C slave address of sensor 4 register. */
    SLV3_SUBADD = 0x4f, /* Address of register on sensor 4 external
                         * register. */
    SLV3_CONFIG = 0x20, /* Sensor 4 configuration and sensor hub settings
                         * register. */

    DATAWRITE_SLV0 = 0x21, /* Data to be written to the slave device
                            * register. */
    STATUS_MASTER = 0x22,  /* Sensor hub source register. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int lsm6dso32_open(FAR struct file *filep);
static int lsm6dso32_close(FAR struct file *filep);
#endif
static ssize_t lsm6dso32_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t lsm6dso32_write(FAR struct file *filep, FAR const char *buffer,
                               size_t buflen);
static int lsm6dso32_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int lsm6dso32_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lsm6dso32_fops = {
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

/****************************************************************************
 * Name: lsm6dso32_read_regs
 *
 * Description:
 *    Read `nbytes` from the LSM6DSO32 into `data` buffer starting at
 *register address `reg`.
 *
 * Returns: 0 if okay, negated errno otherwise.
 *
 ****************************************************************************/

static int lsm6dso32_read_regs(struct lsm6dso32_dev_s const *dev, uint8_t reg,
                               uint8_t *data, ssize_t nbytes) {
    DEBUGASSERT(nbytes > 0);

    struct i2c_msg_s msg[2] = {
        /* Write register address. */

        {
            .frequency = CONFIG_LSM6DSO32_FREQ,
            .addr = dev->addr,
            .flags = I2C_M_NOSTOP,
            .buffer = &reg,
            .length = sizeof(reg),
        },

        /* Read return from registers. */

        {
            .frequency = CONFIG_LSM6DSO32_FREQ,
            .addr = dev->addr,
            .flags = I2C_M_READ,
            .buffer = data,
            .length = nbytes,
        },
    };

    return I2C_TRANSFER(dev->i2c, msg, 2);
}

/****************************************************************************
 * Name: lsm6dso32_read_reg
 *
 * Description:
 *    Read a single register from the LSM6DSO32 into `data` buffer.
 *
 * Returns: 0 if okay, negated errno otherwise.
 *
 ****************************************************************************/

static int lsm6dso32_read_reg(struct lsm6dso32_dev_s const *dev, uint8_t reg,
                              uint8_t *data) {
    return lsm6dso32_read_regs(dev, reg, data, 1);
}

/****************************************************************************
 * Name: lsm6dso32_write_regs
 *
 * Description:
 *    Write `nbytes` of `data` to the registers starting at address `reg`.
 *
 * Returns: 0 if okay, negated errno otherwise.
 *
 ****************************************************************************/

static int lsm6dso32_write_regs(struct lsm6dso32_dev_s const *dev,
                                uint8_t reg, uint8_t *data, ssize_t nbytes) {
    struct i2c_msg_s msg[2] = {
        /* Write start register address. */

        {
            .addr = dev->addr,
            .frequency = CONFIG_LSM6DSO32_FREQ,
            .flags = I2C_M_NOSTOP,
            .buffer = &reg,
            .length = sizeof(reg),
        },

        /* Write data. */

        {
            .addr = dev->addr,
            .frequency = CONFIG_LSM6DSO32_FREQ,
            .flags = 0,
            .buffer = data,
            .length = nbytes,
        },
    };

    return I2C_TRANSFER(dev->i2c, msg, 2);
}

/****************************************************************************
 * Name: lsm6dso32_write_reg
 *
 * Description:
 *    Write a byte of `data` to the specified register, `reg`.
 *
 * Returns: 0 if okay, negated errno otherwise.
 *
 ****************************************************************************/

static int lsm6dso32_write_reg(struct lsm6dso32_dev_s const *dev, uint8_t reg,
                               uint8_t data) {
    return lsm6dso32_write_regs(dev, reg, &data, 1);
}

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS

/****************************************************************************
 * Name: lsm6dso32_open
 *
 * Description:
 *    open handler for LSM6DSO32
 *
 * Returns: 0 if okay, negated errno otherwise.
 *
 ****************************************************************************/

static int lsm6dso32_open(FAR struct file *filep) {
    FAR struct inode *inode = filep->f_inode;
    FAR struct lsm6dso32_dev_s *priv = inode->i_private;
    int err;

    /* Get exclusive access */

    err = nxmutex_lock(&priv->devlock);
    if (err) return err;

    /* Increment the count of open references on the driver */

    priv->crefs++;
    DEBUGASSERT(priv->crefs > 0);

    nxmutex_unlock(&priv->devlock);
    return 0;
}

#endif // CONFIG_DISABLE_PSEUDOFS_OPERATIONS

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS

/****************************************************************************
 * Name: lsm6dso32_close
 *
 * Description:
 *    close handler for LSM6DSO32
 *
 * Returns: 0 if okay, negated errno otherwise.
 *
 ****************************************************************************/

static int lsm6dso32_close(FAR struct file *filep) {
    FAR struct inode *inode = filep->f_inode;
    FAR struct lsm6dso32_dev_s *priv = inode->i_private;
    int err;

    /* Get exclusive access */

    err = nxmutex_lock(&priv->devlock);
    if (err) return err;

    /* Decrement the count of open references on the driver */

    DEBUGASSERT(priv->crefs > 0);
    priv->crefs--;

    /* If the count has decremented to zero and the driver has been unlinked,
     * then free memory now.
     */

    if (priv->crefs <= 0 && priv->unlinked)
    {
        nxmutex_destroy(&priv->devlock);
        kmm_free(priv);
        return 0;
    }

    nxmutex_unlock(&priv->devlock);
    return 0;
}

#endif // CONFIG_DISABLE_PSEUDOFS_OPERATIONS

/****************************************************************************
 * Name: lsm6dso32_read
 *
 * Description:
 *    read handler for LSM6DSO32
 *
 * Returns: Number of bytes read.
 *
 ****************************************************************************/

static ssize_t lsm6dso32_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen) {
    FAR struct inode *inode = filep->f_inode;
    FAR struct lsm6dso32_dev_s *priv = inode->i_private;
    ssize_t length = 0;
    int err;

    /* Get exclusive access */

    err = nxmutex_lock(&priv->devlock);
    if (err) return 0;

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    if (priv->unlinked)
    {
        /* Do not allow operations on unlinked sensors. This allows
         * sensor use on hot swappable I2C bus.
         */

        nxmutex_unlock(&priv->devlock);
        return 0;
    }
#endif

    uint8_t data;
    err = lsm6dso32_read_reg(priv, WHO_AM_I, &data);
    if (err)
    {
        nxmutex_unlock(&priv->devlock);
        return 0;
    }

    length = snprintf(buffer, buflen, "ID: %02x\n", data);
    if (length > buflen) length = buflen;

    nxmutex_unlock(&priv->devlock);
    return length;
}

/****************************************************************************
 * Name: lsm6dso32_write
 *
 * Description:
 *    write handler for LSM6DSO32
 *
 * Returns: Negated errno number (not implemented)
 *
 ****************************************************************************/
static ssize_t lsm6dso32_write(FAR struct file *filep, FAR const char *buffer,
                               size_t buflen) {
    return -ENOSYS;
}

/****************************************************************************
 * Name: lsm6dso32_ioctl
 *
 * Description:
 *    ioctl handler for LSM6DSO32
 *
 * Returns: 0 if okay, negated errno otherwise.
 *
 ****************************************************************************/

static int lsm6dso32_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg) {
    FAR struct inode *inode = filep->f_inode;
    FAR struct lsm6dso32_dev_s *priv = inode->i_private;
    int err = OK;

    switch (cmd)
    {
    case SNIOC_WHO_AM_I:
        err = lsm6dso32_read_reg(priv, WHO_AM_I, (uint8_t *)arg);
        break;
    }

    return err;
}

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS

/****************************************************************************
 * Name: lsm6dso32_unlink
 *
 * Description:
 *    unlink handler for LSM6DSO32
 *
 * Returns: 0 if okay, negated errno otherwise.
 *
 ****************************************************************************/

static int lsm6dso32_unlink(FAR struct inode *inode) {
    FAR struct lsm6dso32_dev_s *priv;
    int err;

    DEBUGASSERT(inode->i_private != NULL);
    priv = inode->i_private;

    /* Get exclusive access */

    err = nxmutex_lock(&priv->devlock);
    if (err) return err;

    /* Are there open references to the driver data structure? */

    if (priv->crefs <= 0)
    {
        nxmutex_destroy(&priv->devlock);
        kmm_free(priv);
        return 0;
    }

    /* No... just mark the driver as unlinked and free the resources when
     * the last client closes their reference to the driver.
     */

    priv->unlinked = true;
    nxmutex_unlock(&priv->devlock);
    return 0;
}

#endif // CONFIG_DISABLE_PSEUDOFS_OPERATIONS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
                       uint8_t addr) {
    FAR struct lsm6dso32_dev_s *priv;
    int err;

    DEBUGASSERT(i2c != NULL);
    DEBUGASSERT(addr == 0x6b || addr == 0x6a);

    /* Initialize the device structure. */

    priv = kmm_malloc(sizeof(struct lsm6dso32_dev_s));
    if (priv == NULL)
    {
        snerr("ERROR: Failed to allocate LSM6DSO32 device instance.\n");
        return -ENOMEM;
    }

    memset(priv, 0, sizeof(*priv));
    err = nxmutex_init(&priv->devlock);
    if (err) return err;
    priv->i2c = i2c;
    priv->addr = addr;

    /* Register the device node. */

    err = register_driver(path, &g_lsm6dso32_fops, 0666, priv);
    if (err)
    {
        snerr("ERROR: Failed to register LSM6DSO32 interface: %d\n", err);
        nxmutex_destroy(&priv->devlock);
        kmm_free(priv);
        return err;
    }

    return 0;
}

#endif // defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM6DSO32)
