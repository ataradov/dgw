/*
 * Copyright (c) 2016, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "samd21.h"
#include "hal_gpio.h"
#include "spi_master.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(MISO,            A, 4);
HAL_GPIO_PIN(MOSI,            A, 6);
HAL_GPIO_PIN(SCLK,            A, 7);
HAL_GPIO_PIN(SS,              A, 5);
#define SPI_SERCOM            SERCOM0
#define SPI_SERCOM_PMUX       PORT_PMUX_PMUXE_D_Val
#define SPI_SERCOM_GCLK_ID    SERCOM0_GCLK_ID_CORE
#define SPI_SERCOM_CLK_GEN    0
#define SPI_SERCOM_APBCMASK   PM_APBCMASK_SERCOM0

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
int spi_init(int freq, int mode)
{
  int baud = F_CPU / (2 * freq) - 1;

  if (baud < 0)
    baud = 0;

  if (baud > 255)
    baud = 255;

  freq = F_CPU / (2 * (baud + 1));

  HAL_GPIO_MISO_in();
  HAL_GPIO_MISO_pmuxen(SPI_SERCOM_PMUX);

  HAL_GPIO_MOSI_out();
  HAL_GPIO_MOSI_pmuxen(SPI_SERCOM_PMUX);

  HAL_GPIO_SCLK_out();
  HAL_GPIO_SCLK_pmuxen(SPI_SERCOM_PMUX);

  HAL_GPIO_SS_out();
  HAL_GPIO_SS_set();

  PM->APBCMASK.reg |= SPI_SERCOM_APBCMASK;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SPI_SERCOM_GCLK_ID) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(SPI_SERCOM_CLK_GEN);

  SPI_SERCOM->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_SWRST;
  while (SPI_SERCOM->SPI.CTRLA.reg & SERCOM_SPI_CTRLA_SWRST);

  SPI_SERCOM->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN;

  SPI_SERCOM->SPI.BAUD.reg = baud;

  SPI_SERCOM->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_ENABLE |
      SERCOM_SPI_CTRLA_DIPO(0) | SERCOM_SPI_CTRLA_DOPO(1) |
      ((mode & 1) ? SERCOM_SPI_CTRLA_CPHA : 0) |
      ((mode & 2) ? SERCOM_SPI_CTRLA_CPOL : 0) |
      SERCOM_SPI_CTRLA_MODE_SPI_MASTER;

  return freq;
}

//-----------------------------------------------------------------------------
void spi_ss(int state)
{
  while (!SPI_SERCOM->SPI.INTFLAG.bit.DRE);
  HAL_GPIO_SS_write(state);
}

//-----------------------------------------------------------------------------
uint8_t spi_write_byte(uint8_t byte)
{
  SPI_SERCOM->SPI.DATA.reg = byte;
  while (!SPI_SERCOM->SPI.INTFLAG.bit.RXC);
  return SPI_SERCOM->SPI.DATA.reg;
}

