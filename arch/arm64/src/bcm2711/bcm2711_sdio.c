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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mmcsd.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm64_arch.h"
#include "arm64_gic.h"
#include "chip.h"
#include "bcm2711_sdio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bcm2711_sdio_dev_s
{
  struct sdio_dev_s dev; /* SDIO device for upper-half */

  uint32_t base; /* Peripheral base address */
  int slotno;    /* The slot number */
  int err;       /* The error code reported from the interrupt handler */
  sem_t wait;    /* Wait semaphore */

  /* Callback support */

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
  sdio_eventset_t cbevents; /* Callback events */
  void *cbarg;              /* Callback argument */
  worker_t cb;              /* Callback that was registered */
  struct work_s cbwork;     /* Callback work queue */
#endif

  sdio_statset_t status; /* Device status */
  bool inited;           /* Whether the device has been initialized */
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
  .err = 0,
  .status = 0,
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

  /* Reset is performed by resetting all of the circuits. TODO: is this
   * all that's necessary?
   */

  mcinfo("Resetting %d...", priv->slotno);

  putreg32(BCM_SDIO_CONTROL1_SRST_HC | BCM_SDIO_CONTROL1_SRST_CMD |
               BCM_SDIO_CONTROL1_SRST_DATA,
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

  switch (rate)
    {
    case CLOCK_SDIO_DISABLED:
      modreg32(0, BCM_SDIO_CONTROL1_CLK_EN, BCM_SDIO_CONTROL1(priv->base));
      mcinfo("%d clock disabled", priv->slotno);
      break;
    case CLOCK_IDMODE:
      modreg32(BCM_SDIO_CONTROL1_CLK_EN, BCM_SDIO_CONTROL1_CLK_EN, BCM_SDIO_CONTROL1(priv->base));
      mcinfo("%d clock ID mode", priv->slotno);
      // TODO: what else
      break;
    case CLOCK_MMC_TRANSFER:
      modreg32(BCM_SDIO_CONTROL1_CLK_EN, BCM_SDIO_CONTROL1_CLK_EN, BCM_SDIO_CONTROL1(priv->base));
      // TODO
      break;
    case CLOCK_SD_TRANSFER_1BIT:
      modreg32(BCM_SDIO_CONTROL1_CLK_EN, BCM_SDIO_CONTROL1_CLK_EN, BCM_SDIO_CONTROL1(priv->base));
      // TODO
      break;
    case CLOCK_SD_TRANSFER_4BIT:
      modreg32(BCM_SDIO_CONTROL1_CLK_EN, BCM_SDIO_CONTROL1_CLK_EN, BCM_SDIO_CONTROL1(priv->base));
      // TODO
      break;
    default:
      DEBUGASSERT(false && "Should never reach here.");
      break;
    }
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
  uint32_t interrupts;

  /* Verify that EMMC2 received the interrupt, since EMMC1 is not yet
   * supported.
   */

  interrupts = getreg32(BCM_SDIO_INTERRUPT(g_emmc2.base));
  mcinfo("%d interrupts %08x", g_emmc2.slotno, interrupts);
  if (interrupts == 0)
    {
      return OK; /* Nothing to do */
    }

  /* Check for any errors and set the error value accordingly */

  // TODO: acknowledge error ints
  g_emmc2.err = 0;

  if (interrupts &
      (BCM_SDIO_INTERRUPT_ERR | BCM_SDIO_INTERRUPT_CCRC_ERR |
       BCM_SDIO_INTERRUPT_CEND_ERR | BCM_SDIO_INTERRUPT_CBAD_ERR |
       BCM_SDIO_INTERRUPT_DCRC_ERR | BCM_SDIO_INTERRUPT_DEND_ERR |
       BCM_SDIO_INTERRUPT_ACMD_ERR))
    {
      g_emmc2.err = -EIO;
    }
  else if (interrupts &
           (BCM_SDIO_INTERRUPT_CTO_ERR | BCM_SDIO_INTERRUPT_DTO_ERR))
    {
      g_emmc2.err = -ETIMEDOUT;
    }
  else if (interrupts & BCM_SDIO_INTERRUPT_CARD)
    {
      mcinfo("Card interrupt!"); // TODO: remove
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

  // TODO: any actual device specific interrupt settings that need to be
  // enabled here?

  /* Enable the interrupt handler */

  arm64_gic_irq_set_priority(BCM_IRQ_VC_EMMC, 0, IRQ_TYPE_EDGE);
  up_enable_irq(BCM_IRQ_VC_EMMC);
  g_emmc_irqinit = true;
  mcinfo("EMMC IRQ enabled.");

  // TODO: remove
  // For fun, force an error interrupt
  putreg32(BCM_SDIO_IRPT_EN_CARD, BCM_SDIO_IRPT_EN(g_emmc2.base));
  mcinfo("Enabled interrupts: %08x.", getreg32(BCM_SDIO_IRPT_EN(g_emmc2.base)));
  mcinfo("Masked interrupts: %08x.", getreg32(BCM_SDIO_IRPT_MASK(g_emmc2.base)));

  mcinfo("Forcing card interrupt.");
  putreg32(BCM_SDIO_FORCE_IRPT_CARD, BCM_SDIO_FORCE_IRPT(g_emmc2.base));

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
                               sdio_eventset_t eventset, uint32_t timeout) {
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
  struct bcm2711_sdio_dev_s *priv = NULL;

  switch (slotno)
    {
    case 2:
      priv = &g_emmc2;
      mcinfo("EMMC2 initialized.");
      break;
    case 1:
      mcwarn("EMMC1 currently unsupported/untested.");
    default:
      mcerr("No SDIO slot number '%d'", slotno);
      return NULL;
    }

  /* Reset device */

  if (!priv->inited)
    {
      bcm2711_reset(&priv->dev);
      priv->inited = true;
    }

  return &priv->dev;
}
