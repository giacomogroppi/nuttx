/***************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_lowputc.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include "arm64_arch.h"
#include "bcm2711_gpio.h"
#include "hardware/bcm2711_aux.h"
#include <nuttx/config.h>

/***************************************************************************
 * Pre-processor definitions
 ***************************************************************************/

/* Clock starts at 5MHz. */

#define SYSTEM_CLOCK_FREQUENCY 500000000

/* Early serial baud rate. */

#ifndef BCM_EARLYSERIAL_BAUD
#define BCM_EARLYSERIAL_BAUD 115200
#endif // BCM_EARLYSERIAL_BAUD

/* Baud rate calculation */

#define AUX_MU_BAUD(baud) ((SYSTEM_CLOCK_FREQUENCY / (baud * 8)) - 1)

/***************************************************************************
 * Public Functions
 ***************************************************************************/

#ifdef CONFIG_ARCH_EARLY_PRINT

/***************************************************************************
 * Name: arm64_earlyprintinit
 *
 * Description:
 *   Configure BCM2711 Mini UART for polling driven operation.
 *
 ***************************************************************************/

static inline void write32 (unsigned long nAddress, unsigned nValue)
{
	*(unsigned volatile *) nAddress = nValue;
}

typedef struct
{
   volatile unsigned data;                // UART Data Register
   volatile unsigned status_error;        // UART Receive Status Register/Error Clear Register
   const 	unsigned reserved1[4];           	// Reserved: 4(0x4) bytes
   volatile unsigned flag;                // UART Flag Register
   const 	unsigned reserved2[1];            	// Reserved: 1(0x1) bytes
   volatile unsigned lp_counter;          // UART Low-power Counter Register
   volatile unsigned integer_br;          // UART Integer Baud Rate Register
   volatile unsigned fractional_br;       // UART Fractional Baud Rate Register
   volatile unsigned line_control;        // UART Line Control Register
   volatile unsigned control;             // UART Control Register
   volatile unsigned isr_fifo_level_sel;  // UART Interrupt FIFO level Select Register
   volatile unsigned isr_mask;            // UART Interrupt Mask Set/Clear Register
   volatile unsigned raw_isr_status;      // UART Raw Interrupt Status Register
   volatile unsigned masked_isr_status;   // UART Masked Interrupt Status Register
   volatile unsigned isr_clear;           // UART Interrupt Clear Register
   volatile unsigned DMA_control;         // UART DMA control Register
} Pl011_Uart;

void pl011_uart_putc(volatile Pl011_Uart * ptr_uart, char c)
{

	//wait until txFIFO is not full
#define UART_FR_TXFF             (1 << 5)
	while(ptr_uart->flag & UART_FR_TXFF);

	ptr_uart->data = c;

}

#ifdef CONFIG_RUNNING_ON_XEN
# define ARM_BASE_UART 0x22000000
#else
# define ARM_BASE_UART 0x107D001000UL
#endif // CONFIG_RUNNING_ON_XEN

#define LCRH_WLEN8_MASK		(3 << 5)
#define CR_UART_EN_MASK		(1 << 0)
#define CR_TXE_MASK		(1 << 8)
#define CR_RXE_MASK		(1 << 9)

#define ARM_UART_DR			(ARM_BASE_UART + 0x00)
#define ARM_UART_FR     	(ARM_BASE_UART + 0x18)
#define ARM_UART_IBRD   	(ARM_BASE_UART + 0x24)
#define ARM_UART_FBRD   	(ARM_BASE_UART + 0x28)
#define ARM_UART_LCRH   	(ARM_BASE_UART + 0x2C)
#define ARM_UART_CR     	(ARM_BASE_UART + 0x30)
#define ARM_UART_IFLS   	(ARM_BASE_UART + 0x34)
#define ARM_UART_IMSC   	(ARM_BASE_UART + 0x38)
#define ARM_UART_RIS    	(ARM_BASE_UART + 0x3C)
#define ARM_UART_MIS    	(ARM_BASE_UART + 0x40)
#define ARM_UART_ICR    	(ARM_BASE_UART + 0x44)

void print_message(const char *message)
{
  for (; *message != '\0'; message++) {
    if (*message == '\r')
      continue;
    if (*message == '\n') {
      pl011_uart_putc((Pl011_Uart*) ARM_BASE_UART, '\r');
      pl011_uart_putc((Pl011_Uart*) ARM_BASE_UART, '\n');
    }
    pl011_uart_putc((Pl011_Uart*) ARM_BASE_UART, *message);
  }
}

void arm64_earlyprintinit(char ch)
{
  /*
  // Enable Mini UART 

  modreg32(BCM_AUX_ENABLE_MU, BCM_AUX_ENABLE_MU, BCM_AUX_ENABLES);

  // Disable interrupts. 

  modreg32(0, (BCM_AUX_MU_IER_RXD | BCM_AUX_MU_IER_TXD),
           BCM_AUX_MU_IER_REG);

  // Disable TX and RX of the UART 

  modreg32(0, BCM_AUX_MU_CNTL_RXENABLE, BCM_AUX_MU_CNTL_REG);
  modreg32(0, BCM_AUX_MU_CNTL_TXENABLE, BCM_AUX_MU_CNTL_REG);

  // Put the UART in 8 bit mode 

  modreg32(BCM_AUX_MU_LCR_DATA8B, BCM_AUX_MU_LCR_DATA8B,
           BCM_AUX_MU_LCR_REG);

  // Ensure RTS line is low. 

  modreg32(0, BCM_AUX_MU_MCR_RTS, BCM_AUX_MU_MCR_REG);

  // Clear the TX and RX FIFOs

  putreg32(BCM_AUX_MU_IIR_RXCLEAR | BCM_AUX_MU_IIR_TXCLEAR,
           BCM_AUX_MU_IIR_REG);

  // Set baud rate.

  putreg32(AUX_MU_BAUD(BCM_EARLYSERIAL_BAUD), BCM_AUX_MU_BAUD_REG);

  // GPIO 14 and GPIO 15 are used as TX and RX.

  // Turn off pull-up/pull-down resistors.

  bcm2711_gpio_set_pulls(14, false, false);
  bcm2711_gpio_set_pulls(15, false, false);

  // Use alternative function 5 (UART1).

  bcm2711_gpio_set_func(14, BCM_GPIO_FUNC5);
  bcm2711_gpio_set_func(15, BCM_GPIO_FUNC5);

  // Enable TX and RX again.

  modreg32(BCM_AUX_MU_CNTL_TXENABLE, BCM_AUX_MU_CNTL_TXENABLE,
           BCM_AUX_MU_CNTL_REG);
  modreg32(BCM_AUX_MU_CNTL_RXENABLE, BCM_AUX_MU_CNTL_RXENABLE,
           BCM_AUX_MU_CNTL_REG);

  */

    const unsigned nBaudrate = 115200;
    const unsigned nClockRate = 44000000;
      
    unsigned nBaud16 = nBaudrate * 16;
    unsigned nIntDiv = nClockRate / nBaud16;
    unsigned nFractDiv2 = (nClockRate % nBaud16) * 8 / nBaudrate;
    unsigned nFractDiv = nFractDiv2 / 2 + nFractDiv2 % 2;

    write32 (ARM_UART_IMSC, 0);
    write32 (ARM_UART_ICR,  0x7FF);
    write32 (ARM_UART_IBRD, nIntDiv);
    write32 (ARM_UART_FBRD, nFractDiv);

    // line parameters
    unsigned nLCRH = LCRH_WLEN8_MASK;
    write32 (ARM_UART_LCRH, nLCRH);

    write32 (ARM_UART_CR, CR_UART_EN_MASK | CR_TXE_MASK | CR_RXE_MASK);
}

/***************************************************************************
 * Name: arm64_lowputc
 *
 * Description:
 *   Output a byte with as few system dependencies as possible.
 *   This implementation uses the BCM2711's Mini UART with polling
 *   to output bytes.
 *
 ***************************************************************************/

void arm64_lowputc(char ch)
{
  pl011_uart_putc((Pl011_Uart*) ARM_BASE_UART, ch);
}

#endif // CONFIG_ARCH_EARLY_PRINT
