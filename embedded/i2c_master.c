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
#include "i2c_master.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(SDA,             A, 8);
HAL_GPIO_PIN(SCL,             A, 9);
#define I2C_SERCOM            SERCOM2
#define I2C_SERCOM_PMUX       PORT_PMUX_PMUXE_D_Val
#define I2C_SERCOM_GCLK_ID    SERCOM2_GCLK_ID_CORE
#define I2C_SERCOM_CLK_GEN    0
#define I2C_SERCOM_APBCMASK   PM_APBCMASK_SERCOM2

#define T_RISE                215e-9 // Depends on the board, actually

enum
{
  I2C_TRANSFER_WRITE = 0,
  I2C_TRANSFER_READ  = 1,
};

enum
{
  I2C_PINS_SDA = (1 << 0),
  I2C_PINS_SCL = (1 << 1),
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
int i2c_init(int freq)
{
  int baud = ((float)F_CPU / freq - (float)F_CPU * T_RISE - 10.0) / 2.0;

  if (baud < 0)
    baud = 0;
  else if (baud > 255)
    baud = 255;

  freq = (float)F_CPU / (2.0 * (5.0 + baud) + (float)F_CPU * T_RISE);

  PM->APBCMASK.reg |= I2C_SERCOM_APBCMASK;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(I2C_SERCOM_GCLK_ID) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(I2C_SERCOM_CLK_GEN);

  I2C_SERCOM->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_SWRST;
  while (I2C_SERCOM->I2CM.CTRLA.reg & SERCOM_I2CM_CTRLA_SWRST);

  I2C_SERCOM->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;
  while (I2C_SERCOM->I2CM.SYNCBUSY.reg);

  I2C_SERCOM->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD(baud);
  while (I2C_SERCOM->I2CM.SYNCBUSY.reg);

  I2C_SERCOM->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_ENABLE |
      SERCOM_I2CM_CTRLA_MODE_I2C_MASTER |
      SERCOM_I2CM_CTRLA_SDAHOLD(3);
  while (I2C_SERCOM->I2CM.SYNCBUSY.reg);

  I2C_SERCOM->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_BUSSTATE(1);

  HAL_GPIO_SDA_in();
  HAL_GPIO_SDA_clr();
  HAL_GPIO_SDA_pmuxen(I2C_SERCOM_PMUX);

  HAL_GPIO_SCL_in();
  HAL_GPIO_SCL_clr();
  HAL_GPIO_SCL_pmuxen(I2C_SERCOM_PMUX);

  return freq;
}

//-----------------------------------------------------------------------------
bool i2c_start(int addr)
{
  I2C_SERCOM->I2CM.INTFLAG.reg = SERCOM_I2CM_INTFLAG_ERROR;

  I2C_SERCOM->I2CM.ADDR.reg = addr;

  while (0 == (I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB) &&
         0 == (I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_SB));

  if (I2C_SERCOM->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK ||
      I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_ERROR)
  {
    I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool i2c_stop(void)
{
  if ((I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB) ||
      (I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_SB))
  {
    I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);
  }

  return true;
}

//-----------------------------------------------------------------------------
bool i2c_read_byte(uint8_t *byte, bool last)
{
  while (1)
  {
    int flags = I2C_SERCOM->I2CM.INTFLAG.reg;

    if (flags & SERCOM_I2CM_INTFLAG_SB)
      break;

    if (flags & (SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_ERROR))
      return false;
  }

  if (last)
    I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_ACKACT | SERCOM_I2CM_CTRLB_CMD(3);
  else
    I2C_SERCOM->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;

  *byte = I2C_SERCOM->I2CM.DATA.reg;

  return true;
}

//-----------------------------------------------------------------------------
bool i2c_write_byte(uint8_t byte)
{
  I2C_SERCOM->I2CM.DATA.reg = byte;

  while (1)
  {
    int flags = I2C_SERCOM->I2CM.INTFLAG.reg;

    if (flags & SERCOM_I2CM_INTFLAG_MB)
      break;

    if (flags & (SERCOM_I2CM_INTFLAG_SB | SERCOM_I2CM_INTFLAG_ERROR))
      return false;
  }

  if (I2C_SERCOM->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK)
  {
    I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool i2c_busy(int addr)
{
  bool busy;

  I2C_SERCOM->I2CM.ADDR.reg = addr | I2C_TRANSFER_WRITE;

  while (0 == (I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB));

  busy = (0 != (I2C_SERCOM->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK));

  I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);

  return busy;
}

//-----------------------------------------------------------------------------
void i2c_pins(int mask, int value)
{
  if (mask & I2C_PINS_SDA)
  {
    HAL_GPIO_SDA_out();
    HAL_GPIO_SDA_write(value & I2C_PINS_SDA);
  }
  else
  {
    HAL_GPIO_SDA_in();
    HAL_GPIO_SDA_clr();
  }

  if (mask & I2C_PINS_SCL)
  {
    HAL_GPIO_SCL_out();
    HAL_GPIO_SCL_write(value & I2C_PINS_SCL);
  }
  else
  {
    HAL_GPIO_SCL_in();
    HAL_GPIO_SCL_clr();
  }

  HAL_GPIO_SDA_pmuxdis();
  HAL_GPIO_SCL_pmuxdis();
}

