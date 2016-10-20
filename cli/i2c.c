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
#include <string.h>
#include <unistd.h>
#include "i2c.h"
#include "dgw.h"
#include "main.h"

/*- Definitions -------------------------------------------------------------*/
enum
{
  CMD_I2C_INIT     = 0x00,
  CMD_I2C_START    = 0x01,
  CMD_I2C_STOP     = 0x02,
  CMD_I2C_READ     = 0x03,
  CMD_I2C_WRITE    = 0x04,
  CMD_I2C_PINS     = 0x05,
};

enum
{
  I2C_TRANSFER_WRITE = 0,
  I2C_TRANSFER_READ  = 1,
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
int i2c_init(int freq)
{
  uint8_t buf[10];

  buf[0] = CMD_I2C_INIT;
  buf[1] = (freq >> 0) & 0xff;
  buf[2] = (freq >> 8) & 0xff;
  buf[3] = (freq >> 16) & 0xff;
  buf[4] = (freq >> 24) & 0xff;
  dgw_cmd(buf, sizeof(buf), 5);

  return ((uint32_t)buf[4] << 24) | ((uint32_t)buf[3] << 16) | ((uint32_t)buf[2] << 8) | buf[1];
}

//-----------------------------------------------------------------------------
bool i2c_write(int addr, uint8_t *data, int size)
{
  uint8_t buf[100];
  int offs = 0;

  buf[0] = CMD_I2C_START;
  buf[1] = addr | I2C_TRANSFER_WRITE;
  dgw_cmd(buf, sizeof(buf), 2);

  if (0 == buf[0])
  {
    warning("start fail (w)");
    return false;
  }

  while (size)
  {
    int sz = (size > 62) ? 62 : size;

    buf[0] = CMD_I2C_WRITE;
    buf[1] = sz;
    memcpy(&buf[2], &data[offs], sz);
    dgw_cmd(buf, sizeof(buf), 2 + sz);

    if (0 == buf[0])
    {
      warning("write fail");
      return false;
    }

    size -= sz;
    offs += sz;
  }

  buf[0] = CMD_I2C_STOP;
  dgw_cmd(buf, sizeof(buf), 1);

  if (0 == buf[0])
  {
    warning("stop fail (w)");
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool i2c_read(int addr, uint8_t *data, int size)
{
  uint8_t buf[100];
  int offs = 0;

  buf[0] = CMD_I2C_START;
  buf[1] = addr | I2C_TRANSFER_READ;
  dgw_cmd(buf, sizeof(buf), 2);

  if (0 == buf[0])
  {
    warning("start fail (r)");
    return false;
  }

  while (size)
  {
    int sz = (size > 61) ? 61 : size;

    buf[0] = CMD_I2C_READ;
    buf[1] = sz;
    buf[2] = (size == sz);
    dgw_cmd(buf, sizeof(buf), 3);

    if (0 == buf[0])
    {
      warning("read fail");
      return false;
    }

    memcpy(&data[offs], &buf[2], sz);

    size -= sz;
    offs += sz;
  }

  buf[0] = CMD_I2C_STOP;
  dgw_cmd(buf, sizeof(buf), 1);

  if (0 == buf[0])
  {
    warning("stop fail (r)");
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool i2c_write_read(int addr, uint8_t *wdata, int wsize, uint8_t *rdata, int rsize)
{
  uint8_t buf[100];
  int offs;

  buf[0] = CMD_I2C_START;
  buf[1] = addr | I2C_TRANSFER_WRITE;
  dgw_cmd(buf, sizeof(buf), 2);

  if (0 == buf[0])
  {
    warning("start fail (w @ wr)");
    return false;
  }

  offs = 0;

  while (wsize)
  {
    int sz = (wsize > 62) ? 62 : wsize;

    buf[0] = CMD_I2C_WRITE;
    buf[1] = sz;
    memcpy(&buf[2], &wdata[offs], sz);
    dgw_cmd(buf, sizeof(buf), 2 + sz);

    if (0 == buf[0])
    {
      warning("write fail (wr)");
      return false;
    }

    wsize -= sz;
    offs += sz;
  }

  buf[0] = CMD_I2C_START;
  buf[1] = addr | I2C_TRANSFER_READ;
  dgw_cmd(buf, sizeof(buf), 2);

  if (0 == buf[0])
  {
    warning("start fail (r @ wr)");
    return false;
  }

  offs = 0;

  while (rsize)
  {
    int sz = (rsize > 61) ? 61 : rsize;

    buf[0] = CMD_I2C_READ;
    buf[1] = sz;
    buf[2] = (rsize == sz);
    dgw_cmd(buf, sizeof(buf), 3);

    if (0 == buf[0])
    {
      warning("read fail (wr)");
      return false;
    }

    memcpy(&rdata[offs], &buf[2], sz);

    rsize -= sz;
    offs += sz;
  }

  buf[0] = CMD_I2C_STOP;
  dgw_cmd(buf, sizeof(buf), 1);

  if (0 == buf[0])
  {
    warning("stop fail (wr)");
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
void i2c_write_reg(int addr, int int_addr, uint32_t value, int size)
{
  uint8_t buf[16];

  buf[0] = (int_addr >> 8) & 0xff;
  buf[1] = int_addr & 0xff;

  for (int i = 0; i < size; i++)
    buf[2 + i] = (value >> (i * 8)) & 0xff;

  i2c_write(addr, buf, 2 + size);
}

//-----------------------------------------------------------------------------
uint32_t i2c_read_reg(int addr, int int_addr, int size)
{
  uint8_t buf[16];
  uint32_t res = 0;

  buf[0] = (int_addr >> 8) & 0xff;
  buf[1] = int_addr & 0xff;
  i2c_write(addr, buf, 2);

  i2c_read(addr, (uint8_t *)&res, size);

  return res;
}

//-----------------------------------------------------------------------------
void i2c_write_buffer(int addr, int int_addr, uint8_t *data, int size)
{
  uint8_t buf[512];

  buf[0] = (int_addr >> 8) & 0xff;
  buf[1] = int_addr & 0xff;
  memcpy(&buf[2], data, size);

  i2c_write(addr, buf, size + 2);
}

//-----------------------------------------------------------------------------
void i2c_read_buffer(int addr, int int_addr, uint8_t *data, int size)
{
  uint8_t buf[512];

  buf[0] = (int_addr >> 8) & 0xff;
  buf[1] = int_addr & 0xff;
  i2c_write(addr, buf, 2);

  i2c_read(addr, data, size);
}

//-----------------------------------------------------------------------------
void i2c_pins(int mask, int value)
{
  uint8_t buf[10];

  buf[0] = CMD_I2C_PINS;
  buf[1] = mask;
  buf[2] = value;
  dgw_cmd(buf, sizeof(buf), 3);
}

