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
#include "spi.h"
#include "dgw.h"
#include "main.h"

/*- Definitions -------------------------------------------------------------*/
enum
{
  CMD_SPI_INIT     = 0x10,
  CMD_SPI_SS       = 0x11,
  CMD_SPI_TRANSFER = 0x12,
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
int spi_init(int freq, int mode)
{
  uint8_t buf[10];

  buf[0] = CMD_SPI_INIT;
  buf[1] = (freq >> 0) & 0xff;
  buf[2] = (freq >> 8) & 0xff;
  buf[3] = (freq >> 16) & 0xff;
  buf[4] = (freq >> 24) & 0xff;
  buf[5] = mode;
  dgw_cmd(buf, sizeof(buf), 6);

  return ((uint32_t)buf[4] << 24) | ((uint32_t)buf[3] << 16) | ((uint32_t)buf[2] << 8) | buf[1];
}

//-----------------------------------------------------------------------------
void spi_ss(int state)
{
  uint8_t buf[10];

  buf[0] = CMD_SPI_SS;
  buf[1] = state;
  dgw_cmd(buf, sizeof(buf), 2);
}

//-----------------------------------------------------------------------------
void spi_transfer(uint8_t *wdata, uint8_t *rdata, int size)
{
  uint8_t buf[100];
  int offs = 0;

  while (size)
  {
    int sz = (size > 62) ? 62 : size;

    buf[0] = CMD_SPI_TRANSFER;
    buf[1] = sz;

    if (wdata)
      memcpy(&buf[2], &wdata[offs], sz);
    else
      memset(&buf[2], 0, sz);

    dgw_cmd(buf, sizeof(buf), 2 + sz);

    if (rdata)
      memcpy(&rdata[offs], &buf[1], sz);

    size -= sz;
    offs += sz;
  }
}

