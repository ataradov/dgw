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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include "gpio.h"
#include "dgw.h"
#include "main.h"

/*- Definitions -------------------------------------------------------------*/
enum
{
  CMD_GPIO_CONFIG  = 0x10,
  CMD_GPIO_READ    = 0x11,
  CMD_GPIO_WRITE   = 0x12,
};

/*- Types -------------------------------------------------------------------*/

/*- Variables ---------------------------------------------------------------*/

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void gpio_configure(int index, int conf)
{
  uint8_t buf[100];

  buf[0] = CMD_GPIO_CONFIG;
  buf[1] = 1; // count
  buf[2] = index;
  buf[3] = conf;

  dgw_cmd(buf, sizeof(buf), 4);
}

//-----------------------------------------------------------------------------
int gpio_read(int index)
{
  uint8_t buf[100];

  buf[0] = CMD_GPIO_READ;
  buf[1] = 1; // count
  buf[2] = index;

  dgw_cmd(buf, sizeof(buf), 3);

  return buf[1];
}

//-----------------------------------------------------------------------------
void gpio_write(int index, int value)
{
  uint8_t buf[100];

  buf[0] = CMD_GPIO_WRITE;
  buf[1] = 1; // count
  buf[2] = index;
  buf[3] = value;

  dgw_cmd(buf, sizeof(buf), 4);
}
