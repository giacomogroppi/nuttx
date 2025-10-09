/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_sdio.c
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/mmcsd.h>
#include <nuttx/sdio.h>
#include <nuttx/semaphore.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>

#include "arm64_arch.h"
#include "arm64_gic.h"
#include "bcm2711_gpio.h"
#include "bcm2711_mailbox.h"
#include "bcm2711_sdio.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The clock rate to use when asking the SD for its ID in Hz (<400kHz) */

#define EMMC_ID_RATE (400000)

/* The clock rate to use during normal operation with the SD card
 * NOTE: taken from LLD again
 * TODO: make this configurable in Kconfig for the user.
 */

#define EMMC_NORMAL_RATE (25000000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bcm2711_sdio_dev_s
{
  struct sdio_dev_s dev; /* SDIO device for upper-half */

  const uint32_t base; /* Peripheral base address */
  uint32_t baseclk;    /* Base clock rate */
  const int slotno;    /* The slot number */
  int err;             /* The error reported from the IRQ handler */
  const uint8_t clkid; /* EMMC clock ID for mailbox to enable */
  sem_t wait;          /* Wait semaphore */

  /* Callback support */

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
  sdio_eventset_t cbevents; /* Callback events */
  void *cbarg;              /* Callback argument */
  worker_t cb;              /* Callback that was registered */
  struct work_s cbwork;     /* Callback work queue */
#endif

  enum sdio_clock_e cur_rate; /* Current clock rate */
  sdio_statset_t status;      /* Device status */
  bool inited;                /* Whether the device has been initialized */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SDIO_MUXBUS
static int bcm2711_lock(FAR struct sdio_dev_s *dev, bool lock);
#endif
static void bcm2711_reset(FAR struct sdio_dev_s *dev);
static sdio_capset_t bcm2711_capabilities(FAR struct sdio_dev_s *dev);
static sdio_statset_t bcm2711_status(FAR struct sdio_dev_s *dev);
static void bcm2711_widebus(FAR struct sdio_dev_s *dev, bool enable);
static void bcm2711_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate);
static int bcm2711_attach(FAR struct sdio_dev_s *dev);

static int bcm2711_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t arg);

#ifdef CONFIG_SDIO_BLOCKSETUP
static void bcm2711_blocksetup(FAR struct sdio_dev_s *dev,
                               unsigned int blocklen, unsigned int nblocks);
#endif

static int bcm2711_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
                             size_t nbytes);
static int bcm2711_sendsetup(FAR struct sdio_dev_s *dev,
                             FAR const uint8_t *buffer, size_t nbytes);
static int bcm2711_cancel(FAR struct sdio_dev_s *dev);
static int bcm2711_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd);
static int bcm2711_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd,
                             FAR uint32_t *rshort);
static int bcm2711_recvshortcrc(FAR struct sdio_dev_s *dev, uint32_t cmd,
                                FAR uint32_t *rshort);
static int bcm2711_recvnotimpl(FAR struct sdio_dev_s *dev, uint32_t cmd,
                               FAR uint32_t *rnotimpl);
static int bcm2711_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd,
                            FAR uint32_t rlong[4]);

static void bcm2711_waitenable(FAR struct sdio_dev_s *dev,
                               sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t bcm2711_eventwait(FAR struct sdio_dev_s *dev);
static void bcm2711_callbackenable(FAR struct sdio_dev_s *dev,
                                   sdio_eventset_t eventset);

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
static int bcm2711_registercallback(FAR struct sdio_dev_s *dev,
                                    worker_t callback, FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_emmc_irqinit = false;

#ifdef CONFIG_BCM2711_EMMC1
#error "EMMC1 is not yet supported/tested."
#endif

#ifdef CONFIG_BCM2711_EMMC2
static struct bcm2711_sdio_dev_s g_emmc2 =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock = bcm2711_lock,
#endif

    .reset = bcm2711_reset,
    .capabilities = bcm2711_capabilities,
    .status = bcm2711_status,
    .widebus = bcm2711_widebus,
    .clock = bcm2711_clock,
    .attach = bcm2711_attach,

    .sendcmd = bcm2711_sendcmd,

#ifdef CONFIG_SDIO_BLOCKSETUP
    .blocksetup = NULL,
#endif

    .recvsetup = bcm2711_recvsetup,
    .sendsetup = bcm2711_sendsetup,
    .cancel = bcm2711_cancel,
    .waitresponse = bcm2711_waitresponse,
    .recv_r1 = bcm2711_recvshortcrc,
    .recv_r2 = bcm2711_recvlong,
    .recv_r3 = bcm2711_recvshort,
    .recv_r4 = bcm2711_recvnotimpl,
    .recv_r5 = bcm2711_recvnotimpl,
    .recv_r6 = bcm2711_recvshortcrc,
    .recv_r7 = bcm2711_recvshort,

    .waitenable = bcm2711_waitenable,
    .eventwait = bcm2711_eventwait,
    .callbackenable = bcm2711_callbackenable,
#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
    .registercallback = bcm2711_registercallback,
#endif

    /* TODO: DMA implementation */

    .gotextcsd = NULL,
  },
  .base = BCM_EMMC2_BASEADDR,
  .wait = SEM_INITIALIZER(0),
  .slotno = 2,
  .clkid = MBOX_CLK_EMMC2,
  .err = 0,
  .status = 0,
  .cur_rate = CLOCK_SDIO_DISABLED,
  .inited = false,

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
  .cb = NULL,
  .cbarg = NULL,
  .cbevents = 0,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_clock_divider
 *
 * Description:
 *   Calculates clock divider for target clock rate with some known base
 *   clock.
 *   NOTE: This was taken from rockytriton/LLD, a bare-metal RPi EMMC driver.
 *
 * Input Parameters:
 *   base   - The base block rate to be divided
 *   target - The target clock rate
 *
 * Returned Value:
 *   The calculated clock divider that will get as close as possible to the
 *   target rate.
 *
 ****************************************************************************/

static uint32_t get_clock_divider(uint32_t base_clock, uint32_t target_rate)
{
  uint32_t target_div = 1;
  uint32_t freqSel;
  uint32_t upper;
  uint32_t ret;

  if (target_rate <= base_clock)
    {
      target_div = base_clock / target_rate;

      if (base_clock % target_rate)
        {
          target_div = 0;
        }
    }

  int div = -1;
  for (int fb = 31; fb >= 0; fb--)
    {
      uint32_t bt = (1 << fb);

      if (target_div & bt)
        {
          div = fb;
          target_div &= ~(bt);

          if (target_div)
            {
              div++;
            }

          break;
        }
    }

  /* Out of bounds, clip division to as much as possible */

  if (div == -1 || div >= 32)
    {
      div = 31;
    }

  if (div != 0)
    {
      div = (1 << (div - 1));
    }

  if (div >= 0x400)
    {
      div = 0x3FF;
    }

  freqSel = div & 0xff;
  upper = (div >> 8) & 0x3;
  ret = (freqSel << 8) | (upper << 6) | (0 << 5);

  return ret;
}

/****************************************************************************
 * Name: bcm2711_clk_waitstable
 *
 * Description:
 *   Busy-waits until the EMMC controller clock has become stable.
 *   TODO: can we do better than a busy wait?
 *
 * Input Parameters:
 *   priv - The device whose EMMC controller clock we are waiting for
 *
 ****************************************************************************/

static void bcm2711_clk_waitstable(struct bcm2711_sdio_dev_s *priv)
{
  while (!(getreg32(BCM_SDIO_CONTROL1(priv->base)) &
           BCM_SDIO_CONTROL1_CLK_STABLE))
    ;
}

/****************************************************************************
 * Name: bcm2711_lock
 *
 * Description:
 *   Locks the bus.Function calls low-level multiplexed bus routines to
 *   resolve bus requests and acknowledge issues.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   lock   - TRUE to lock, FALSE to unlock.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_MUXBUS
static int bcm2711_lock(FAR struct sdio_dev_s *dev, bool lock)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: bcm2711_reset
 *
 * Description:
 *   Reset the SDIO controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bcm2711_reset(FAR struct sdio_dev_s *dev)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;

  /* Reset is performed by resetting host controller. */

  mcinfo("Resetting %d...", priv->slotno);

  putreg32(BCM_SDIO_CONTROL1_SRST_HC, BCM_SDIO_CONTROL1(priv->base));

  /* Now we busy wait until the controller reports that it's done resetting by
   * setting all the reset flags to 0.
   * TODO: can we do better than a busy wait?
   */

  while ((getreg32(BCM_SDIO_CONTROL1(priv->base)) &
          (BCM_SDIO_CONTROL1_SRST_HC | BCM_SDIO_CONTROL1_SRST_DATA |
           BCM_SDIO_CONTROL1_SRST_CMD)) != 0)
    ;

  /* Enable VDD1 bus power for SD card */

  modreg32(0x0f << 8, BCM_SDIO_CONTROL1_CLK_FREQ8,
           BCM_SDIO_CONTROL1(priv->base));

  // TODO: reset semaphore, wdog and etc.

  nxsem_reset(&priv->wait, 0);
  mcinfo("Reset %d", priv->slotno);
}

/****************************************************************************
 * Name: bcm2711_capabilities
 *
 * Description:
 *   Get capabilities (and limitations) of the SDIO driver (optional)
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see SDIO_CAPS_* defines)
 *
 ****************************************************************************/

static sdio_capset_t bcm2711_capabilities(FAR struct sdio_dev_s *dev)
{
  (void)(dev);
  return SDIO_CAPS_4BIT | SDIO_CAPS_8BIT | SDIO_CAPS_MMC_HS_MODE;
}

/****************************************************************************
 * Name: bcm2711_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see SDIO_STATUS_* defines)
 *
 ****************************************************************************/

static sdio_statset_t bcm2711_status(FAR struct sdio_dev_s *dev)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  sdio_statset_t stat = 0;

  mcinfo("Getting %d status", priv->slotno);

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
  /* TODO: for now, unsure how to detect card so always lie that there is a
   * card in the slot and have the MMCSD upper-half perform the probe.
   */
#endif

  stat |= SDIO_STATUS_PRESENT;

  // TODO detect if card is wrprotected or if a card is present
  return stat;
}

/****************************************************************************
 * Name: bcm2711_widebus
 *
 * Description:
 *   Called after change in Bus width has been selected (via ACMD6).  Most
 *   controllers will need to perform some special operations to work
 *   correctly in the new bus mode.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   wide - true: wide bus (4-bit) bus mode enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bcm2711_widebus(FAR struct sdio_dev_s *dev, bool enable)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  mcinfo("%d widebus: %d", priv->slotno, enable);
  // TODO: do something about bus width
}

/****************************************************************************
 * Name: bcm2711_clock
 *
 * Description:
 *   Enable/disable SDIO clocking
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   rate - Specifies the clocking to use (see enum sdio_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bcm2711_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  uint32_t divider;
  bool enable;

  mcinfo("Setting clock rate for EMMC%d.", priv->slotno);

  /* If the desired rate already matches the current rate, we're done. */

  if (rate == priv->cur_rate)
    {
      mcinfo("New rate matches current rate.");
      return;
    }

  switch (rate)
    {
    case CLOCK_SDIO_DISABLED:
      enable = false;
      divider = 0;
      mcinfo("EMMC%d clock disabled.", priv->slotno);
      break;

    case CLOCK_IDMODE:
      mcinfo("EMMC%d clock in ID mode.", priv->slotno);
      enable = true;

      /* Calculate the appropriate clock divider */

      divider = get_clock_divider(priv->baseclk, EMMC_ID_RATE);
      // TODO: what else
      break;

    case CLOCK_MMC_TRANSFER:
      enable = true;
      // TODO: decide on transfer rates, using LLD value for now
      divider = get_clock_divider(priv->baseclk, EMMC_NORMAL_RATE);
      break;

    case CLOCK_SD_TRANSFER_1BIT:
      enable = true;
      // TODO: decide on transfer rates, using LLD value for now
      divider = get_clock_divider(priv->baseclk, EMMC_NORMAL_RATE);
      break;

    case CLOCK_SD_TRANSFER_4BIT:
      enable = true;
      // TODO: decide on transfer rates, using LLD value for now
      divider = get_clock_divider(priv->baseclk, EMMC_NORMAL_RATE);
      break;

    default:
      DEBUGASSERT(false && "Should never reach here.");
      return;
    }

  mcinfo("Base rate: %u, divider: %08x", priv->baseclk, divider);

  /* First, disable the clock before making changes. */

  modreg32(0, BCM_SDIO_CONTROL1_CLK_EN | BCM_SDIO_CONTROL1_CLK_INTLEN,
           BCM_SDIO_CONTROL1(priv->base));

  /* Set the appropriate clock divider */

  modreg32(divider,
           BCM_SDIO_CONTROL1_CLK_FREQ8 | BCM_SDIO_CONTROL1_CLK_FREQ_MS2,
           BCM_SDIO_CONTROL1(priv->base));

  // TODO: Set data timeout unit exponent (DATA_TOUNIT)
  // This was taken from LLD code, not sure what it does

  modreg32(11 << 16, BCM_SDIO_CONTROL1_DATA_TOUNIT,
           BCM_SDIO_CONTROL1(priv->base));

  /* Enable the clock and wait for it to stabilize.
   * TODO: LLD does this differently, check back if this doesn't work
   */

  if (enable)
    {
      modreg32(~0, BCM_SDIO_CONTROL1_CLK_EN | BCM_SDIO_CONTROL1_CLK_INTLEN,
               BCM_SDIO_CONTROL1(priv->base));
      bcm2711_clk_waitstable(priv);
    }

  mcinfo("Clock rate updated!");

  /* If we got all the way here, the clock rate was set successfully (as far
   * as we know), so we can update the current clock rate to save computation
   * later.
   */

  priv->cur_rate = rate;
}

/****************************************************************************
 * Name: bcm2711_emmc_handler_internal
 *
 * Description:
 *   Interrupt handling logic that works for all Arasan interfaces
 *
 ****************************************************************************/

static int bcm2711_emmc_handler_internal(struct bcm2711_sdio_dev_s *priv)
{
  uint32_t flags;

  /* Get interrupt flags */

  flags = getreg32(BCM_SDIO_INTERRUPT(priv->base));
  mcinfo("%d interrupts %08x", priv->slotno, flags);

  /* Check for any errors and set the error value accordingly */

  // TODO: acknowledge error ints
  priv->err = 0;

  if (flags &
      (BCM_SDIO_INTERRUPT_ERR | BCM_SDIO_INTERRUPT_CCRC_ERR |
       BCM_SDIO_INTERRUPT_CEND_ERR | BCM_SDIO_INTERRUPT_CBAD_ERR |
       BCM_SDIO_INTERRUPT_DCRC_ERR | BCM_SDIO_INTERRUPT_DEND_ERR |
       BCM_SDIO_INTERRUPT_ACMD_ERR))
    {
      priv->err = -EIO;
    }
  else if (flags &
           (BCM_SDIO_INTERRUPT_CTO_ERR | BCM_SDIO_INTERRUPT_DTO_ERR))
    {
      priv->err = -ETIMEDOUT;
    }
  else if (flags & BCM_SDIO_INTERRUPT_CARD)
    {
      mcinfo("Card interrupt!"); // TODO: remove
    }

  /* TODO: For now, we acknowledge all interrupts so that we can move on
   * without the handler getting called forever. This is because the interrupt
   * handler logic only seems to work on 'level', not 'edge', so we must
   * immediately acknowledge the interrupts.
   */

  putreg32(flags, BCM_SDIO_INTERRUPT(priv->base));

  return OK;
}

/****************************************************************************
 * Name: bcm2711_emmc_handler
 *
 * Description:
 *   Interrupt handler for EMMC interrupt
 *
 ****************************************************************************/

static int bcm2711_emmc_handler(int irq, void *context, void *arg)
{
  (void)(irq);
  (void)(context);
  (void)(arg);
  int err;

  /* Handle interrupts for all interfaces */

  if (getreg32(BCM_SDIO_INTERRUPT(g_emmc2.base)))
    {
      err = bcm2711_emmc_handler_internal(&g_emmc2);
      if (err)
        {
          return err;
        }
    }
  else
    {
      // TODO: Since EMMC1 is not supported, we should never be here
      mcerr("Non-EMMC2 interrupt even though unsupported!");
      DEBUGASSERT(false && "Non-EMMC2 interrupt even though unsupported!");
    }

  return OK;
}

/****************************************************************************
 * Name: bcm2711_attach
 *
 * Description:
 *   Attach and prepare interrupts
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

static int bcm2711_attach(FAR struct sdio_dev_s *dev)
{
  int err;
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;

  /* No need to register interrupt handler twice if already registered */

  if (g_emmc_irqinit)
    {
      mcinfo("EMMC interrupt handler already attached.");
      return 0;
    }

  err = irq_attach(BCM_IRQ_VC_EMMC, bcm2711_emmc_handler, NULL);
  if (err < 0)
    {
      mcerr("Couldn't attach EMMC interrupt handler: %d", err);
      return err;
    }

  mcinfo("EMMC interrupt handler attached for slot %d.", priv->slotno);

  /* Enable and unmask all interrupts.
   * TODO: should I only enable a subset of these?
   */

  putreg32(BCM_SDIO_IRPT_MASK_ALL, BCM_SDIO_IRPT_MASK(priv->base));
  putreg32(BCM_SDIO_IRPT_EN_ALL, BCM_SDIO_IRPT_EN(priv->base));

  /* Enable the interrupt handler */

  arm64_gic_irq_set_priority(BCM_IRQ_VC_EMMC, 0, IRQ_TYPE_LEVEL);
  up_enable_irq(BCM_IRQ_VC_EMMC);
  g_emmc_irqinit = true;
  mcinfo("EMMC IRQ enabled.");
  return 0;
}

/****************************************************************************
 * Name: bcm2711_sendcmd
 *
 * Description:
 *   Send the SDIO command
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command to send.  See 32-bit command definitions above.
 *   arg  - 32-bit argument required with some commands
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int bcm2711_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t arg)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  uint32_t cmdtm_val = 0;

  /* Put argument in ARG1 /unless/ the argument is to be sent with ACMD23.
   * Arguments must be put in the register before the command is sent.
   */

  if (cmd == SD_ACMD23)
    {
      putreg32(arg, BCM_SDIO_ARG2(priv->base));
    }
  else
    {
      putreg32(arg, BCM_SDIO_ARG1(priv->base));
    }

  mcinfo("Argument register populated");

  /* Set the CMDTM register flags according to the command */

  /* Set the response type */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      cmdtm_val |= BCM_SDIO_CMDTM_CMD_RSPNS_TYPE_NONE;
      break;
    case MMCSD_R1B_RESPONSE:
      /* TODO: 48 bit busy (?) */
      cmdtm_val |= BCM_SDIO_CMDTM_CMD_RSPNS_TYPE_48B;
      break;
    case MMCSD_R1_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      cmdtm_val |= BCM_SDIO_CMDTM_CMD_RSPNS_TYPE_48;
      break;
    case MMCSD_R2_RESPONSE:
      cmdtm_val |= BCM_SDIO_CMDTM_CMD_RSPNS_TYPE_136;
      break;
    }

  /* Set index of command */

  cmdtm_val |= ((cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT)
               << BCM_SDIO_CMDTM_CMD_INDEX_SHIFT;

  /* Populate command register with correct values */

  mcinfo("Setting command register: %08x.", cmdtm_val);
  putreg32(cmdtm_val, BCM_SDIO_CMDTM(priv->base));
  return 0;
}

/****************************************************************************
 * Name: bcm2711_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card in non-DMA
 *   (interrupt driven mode).  This method will do whatever controller setup
 *   is necessary.  This would be called for SD memory just BEFORE sending
 *   CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc.  Normally,
 *   SDIO_WAITEVENT will be called to receive the indication that the
 *   transfer is complete.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer in which to receive the data
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int bcm2711_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
                             size_t nbytes)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  mcinfo("Slot %d", priv->slotno);
  // TODO
  return 0;
}

/****************************************************************************
 * Name: bcm2711_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card.  This
 *   method will do whatever controller setup is necessary.  This would be
 *   called for SD memory just AFTER sending CMD24 (WRITE_BLOCK), CMD25
 *   (WRITE_MULTIPLE_BLOCK), ... and before SDIO_SENDDATA is called.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer containing the data to send
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int bcm2711_sendsetup(FAR struct sdio_dev_s *dev,
                             FAR const uint8_t *buffer, size_t nbytes)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  mcinfo("Slot %d", priv->slotno);
  // TODO
  return 0;
}

/****************************************************************************
 * Name: bcm2711_cancel
 *
 * Description:
 *   Cancel the data transfer setup of SDIO_RECVSETUP, SDIO_SENDSETUP,
 *   SDIO_DMARECVSETUP or SDIO_DMASENDSETUP.  This must be called to cancel
 *   the data transfer setup if, for some reason, you cannot perform the
 *   transfer.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int bcm2711_cancel(FAR struct sdio_dev_s *dev)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  mcinfo("Slot %d", priv->slotno);
  // TODO:
  return 0;
}

/****************************************************************************
 * Name: bcm2711_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.  This
 *   function should be called even after sending commands that have no
 *   response (such as CMD0) to make sure that the hardware is ready to
 *   receive the next command.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int bcm2711_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  mcinfo("Slot %d", priv->slotno);
  // TODO
  return 0;
}

/****************************************************************************
 * Name: bcm2711_recvshortcrc
 *
 * Description:
 *
 * Input Parameters:
 *   dev - An instance of the SD card device interface
 *   cmd - The command that was sent
 *   rshort - The buffer to recv into
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a failure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.). The implementation only
 *   assures that the response is returned intact and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int bcm2711_recvshortcrc(FAR struct sdio_dev_s *dev, uint32_t cmd,
                                FAR uint32_t *rshort)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  mcinfo("Slot %d", priv->slotno);
  // TODO
  return 0;
}

/****************************************************************************
 * Name: bcm2711_recvshort
 *
 * Description:
 *
 * Input Parameters:
 *   dev - An instance of the SD card device interface
 *   cmd - The command that was sent
 *   rshort - The buffer to recv into
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a failure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.). The implementation only
 *   assures that the response is returned intact and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int bcm2711_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd,
                             FAR uint32_t *rshort)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  mcinfo("Slot %d", priv->slotno);
  // TODO
  return 0;
}

/****************************************************************************
 * Name: bcm2711_recvlong
 *
 * Description:
 *
 * Input Parameters:
 *   dev - An instance of the SD card device interface
 *   cmd - The command that was sent
 *   rlong - The buffer to receive into
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure. Here a
 *   failure means only a failure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.). The implementation only
 *   assures that the response is returned intact and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int bcm2711_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd,
                            FAR uint32_t rlong[4])
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  mcinfo("Slot %d", priv->slotno);
  // TODO
  return 0;
}

/****************************************************************************
 * Name: bcm2711_recvnotimpl
 *
 * Description: Handler for unimplemented receive functionality
 *
 * Input Parameters:
 *   dev - An instance of the SD card device interface
 *   cmd - The command that was sent
 *   rnotimpl - Unused
 *
 * Returned Value: -ENOSYS
 *
 ****************************************************************************/

static int bcm2711_recvnotimpl(FAR struct sdio_dev_s *dev, uint32_t cmd,
                               FAR uint32_t *rnotimpl)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  (void)(cmd);
  (void)(rnotimpl);

  // TODO: look at LPC43xx impl and see if anything else is necessary

  mcerr("Called unimplemented receive for slot %d\n", priv->slotno);
  return -ENOSYS;
}

/****************************************************************************
 * Name: bcm2711_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling either calling SDIO_DMARECVSETUP,
 *   SDIO_DMASENDSETUP, or SDIO_WAITEVENT.  This is the recommended
 *   ordering:
 *
 *     SDIO_WAITENABLE:    Discard any pending interrupts, enable event(s)
 *                         of interest
 *     SDIO_DMARECVSETUP/
 *     SDIO_DMASENDSETUP:  Setup the logic that will trigger the event the
 *                         event(s) of interest
 *     SDIO_WAITEVENT:     Wait for the event of interest (which might
 *                         already have occurred)
 *
 *   This sequence should eliminate race conditions between the command/
 *   transfer setup and the subsequent events.
 *
 *   The enabled events persist until either (1) SDIO_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) SDIO_EVENTWAIT
 *   returns.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOWAIT_*
 *              definitions). 0=disable; 1=enable.
 *   timeout  - Maximum time in milliseconds to wait.  Zero means immediate
 *              timeout with no wait.  The timeout value is ignored if
 *              SDIOWAIT_TIMEOUT is not included in the waited-for eventset.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bcm2711_waitenable(FAR struct sdio_dev_s *dev,
                               sdio_eventset_t eventset, uint32_t timeout)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  uint32_t ints_en = 0;
  mcinfo("Slot %d waitenable for events %04x", priv->slotno, eventset);

  /* Disable event-related interrupts and clear pending interrupts.
   * TODO: is clearing & disabling all interrupts too much?
   */

  putreg32(0, BCM_SDIO_INTERRUPT(priv->base)); /* Clear pending */
  putreg32(0, BCM_SDIO_IRPT_EN(priv->base));   /* Disable all */

  mcinfo("Cleared and disabled interrupts");

  /* Enable interrupts for the new event set */

  if (eventset & SDIOWAIT_CMDDONE)
    {
      ints_en |= BCM_SDIO_IRPT_EN_CMD_DONE;
    }

  if (eventset & SDIOWAIT_RESPONSEDONE)
    {
      // TODO
    }

  if (eventset & SDIOWAIT_TRANSFERDONE)
    {
      // TODO
      ints_en |= BCM_SDIO_IRPT_EN_DATA_DONE;
    }

  if (eventset & SDIOWAIT_TIMEOUT)
    {
      ints_en |= BCM_SDIO_IRPT_EN_DTO_ERR | BCM_SDIO_IRPT_EN_CTO_ERR;
    }

  if (eventset & SDIOWAIT_ERROR)
    {
      ints_en |= BCM_SDIO_IRPT_EN_CCRC_ERR | BCM_SDIO_IRPT_EN_CEND_ERR |
                 BCM_SDIO_IRPT_EN_CBAD_ERR | BCM_SDIO_IRPT_EN_DCRC_ERR |
                 BCM_SDIO_IRPT_EN_DEND_ERR | BCM_SDIO_IRPT_EN_ACMD_ERR;
    }

  putreg32(0, BCM_SDIO_IRPT_MASK(priv->base));     /* Clear mask */
  putreg32(ints_en, BCM_SDIO_IRPT_EN(priv->base)); /* Enable new interrupts */

  mcinfo("Enabled new interrupts");
}

/****************************************************************************
 * Name: bcm2711_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when SDIO_EVENTWAIT
 *   returns.  SDIO_WAITEVENTS must be called again before SDIO_EVENTWAIT
 *   can be used again.
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  Should always
 *   be non-zero.  All events are disabled after the wait concludes.
 *
 ****************************************************************************/

static sdio_eventset_t bcm2711_eventwait(FAR struct sdio_dev_s *dev)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  mcinfo("Slot %d", priv->slotno);
  // TODO
  return 0;
}

/****************************************************************************
 * Name: bcm2711_callback
 *
 * Description:
 *  Perform the registered callback when events apply.
 *
 * Input Parameters:
 *   priv      - The device to perform the callback for
 *
 ****************************************************************************/

static void bcm2711_callback(struct bcm2711_sdio_dev_s *priv)
{
  /* All done here if no callback is registered, or callback events are
   * disabled
   */

  if (priv->cb == NULL || priv->cbevents == 0)
    {
      return;
    }

  if (priv->status & SDIO_STATUS_PRESENT)
    {
      /* If card is present and we don't care about insertion events, do
       * nothing.
       */

      if (!(priv->cbevents & SDIOMEDIA_INSERTED))
        {
          return;
        }
    }
  else
    {
      /* If card is not present and we don't care about ejection events, do
       * nothing.
       */

      if (!(priv->cbevents & SDIOMEDIA_EJECTED))
        {
          return;
        }
    }

  /* Some event that we care about happened, so call the callback and then
   * immediately disable it. Don't directly call the callback in an interrupt
   * context.
   */

  if (up_interrupt_context())
    {
      work_queue(HPWORK, &priv->cbwork, priv->cb, priv->cbarg, 0);
    }
  else
    {
      priv->cb(priv->cbarg);
    }

  priv->cbevents = 0; /* Disabled for future */
}

/****************************************************************************
 * Name: bcm2711_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in SDIO_REGISTERCALLBACK.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this method.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOMEDIA_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bcm2711_callbackenable(FAR struct sdio_dev_s *dev,
                                   sdio_eventset_t eventset)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  mcinfo("Enabling callback on slot %d for events %02x\n", priv->slotno,
         eventset);

  priv->cbevents = eventset;
  bcm2711_callback(priv); /* Immediately check events */
}

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)

/****************************************************************************
 * Name: bcm2711_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change. Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to SDIO_CALLBACKENABLE.
 *
 *   NOTE: High-priority work queue support is required.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The function to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

static int bcm2711_registercallback(FAR struct sdio_dev_s *dev,
                                    worker_t callback, FAR void *arg)
{
  struct bcm2711_sdio_dev_s *priv = (struct bcm2711_sdio_dev_s *)dev;
  mcinfo("Callback %p registered for %d", callback, priv->slotno);

  /* Register this callback and disable it */

  priv->cb = callback;
  priv->cbarg = arg;
  priv->cbevents = 0;

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_sdio_initialize
 *
 * Description:
 *   Initialize the BCM2711 SDIO peripheral for normal operation.
 *
 * Input Parameters:
 *   slotno - 1 for EMMC1, 2 for EMMC2
 *
 * Returned Value:
 *   A reference to an SDIO interface structure.
 *   NULL is returned on failures.
 *
 ****************************************************************************/

struct sdio_dev_s *bcm2711_sdio_initialize(int slotno)
{
  int err;
  struct bcm2711_sdio_dev_s *priv = NULL;

  /* NOTE:
   * According to https://ultibo.org/wiki/Unit_BCM2711:
   * - EMMC0 is an Arasan controller
   *   - No card detect pin
   *   - No write protect pin
   *   - Routed to 22/27 (ALT3) or 48-53 (ALT3)
   *   - GPIO pins 34-49 (ALT3) provide SDIO control to Wifi
   * - EMMC1 is non-SDHCI compliant device
   *   - Can be routed to 22-27 (ALT0) or 48-53 (ALT0), but only 22-27 can be
   *   used.
   * - EMMC2 is an SDHCI compliant device, doesn't appear on GPIO pins,
   *   connected to SD card slot
   *   - BCM2838_GPPINMUX register routes EMMC0 to SD-card slot, making EMMC2
   *   usable.
   */

  switch (slotno)
    {
    case 1:
      mcerr("EMMC1 currently unsupported/untested.");
      return NULL;
    case 2:
      priv = &g_emmc2;
      break;
    default:
      mcerr("No SDIO slot number '%d'", slotno);
      return NULL;
    }

  /* If the device was already initialized, return it */

  if (priv->inited)
    {
      return &priv->dev;
    }

  /* Configure GPIO pins for the interface TODO: have I matched pins to right
   * interface? LLD controls EMMC2 only and sets all these pins during init */

  switch (priv->slotno) {
    case 1:
      // TODO
      break;
    case 2:
      bcm2711_gpio_set_func(34, BCM_GPIO_INPUT);
      bcm2711_gpio_set_func(35, BCM_GPIO_INPUT);
      bcm2711_gpio_set_func(36, BCM_GPIO_INPUT);
      bcm2711_gpio_set_func(37, BCM_GPIO_INPUT);
      bcm2711_gpio_set_func(38, BCM_GPIO_INPUT);
      bcm2711_gpio_set_func(39, BCM_GPIO_INPUT);

      bcm2711_gpio_set_func(48, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(49, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(50, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(51, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(52, BCM_GPIO_FUNC3);
      break;
    }

  /* Reset device */

  bcm2711_reset(&priv->dev);

  /* Enable correct clock.
   * TODO: clocks seem to be enabled by default. Currently `setclken`
   * returns 0x80000008 response code so I'm ignoring this out for now.
   */

  err = bcm2711_mbox_setclken(priv->clkid, true);
  if (err != 0 && err != -EAGAIN)
    {
      mcerr("Couldn't enable EMMC%d clock: %d", priv->slotno, err);
      return NULL;
    }

  /* Determine the base clock rate.
   * NOTE: This driver assumes that it is in complete control of the EMMC
   * base clocks, and that they will not change without its knowledge.
   *
   * TODO: This call also returns 0x80000008 response code, but the rate
   * value returned is reasonable (100MHz). For now, I am ignoring the
   * 0x80000008 response code, not sure why that happens though.
   */

  err = bcm2711_mbox_getclkrate(priv->clkid, &priv->baseclk, false);
  if (err != -EAGAIN && err != 0)
    {
      mcerr("Couldn't determine base clock rate for EMMC%d: %d\n",
            priv->slotno, err);
      return NULL;
    }

  mcinfo("EMMC%d base clock: %uHz\n", priv->slotno, priv->baseclk);

  /* Special case: EMMC2 accesses SD card, so ensure it is powered and
   * power is stable.
   */

  if (priv->slotno == 2)
    {
      err = bcm2711_mbox_setpwr(MBOX_PDOM_SDCARD, true, true);
      if (err)
        {
          mcerr("Couldn't power SD card: %d\n", err);
          return NULL;
        }
    }

  priv->inited = true;
  return &priv->dev;
}
