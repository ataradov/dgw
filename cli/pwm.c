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
#include "main.h"
#include "dgw.h"
#include "pwm.h"

/*- Definitions -------------------------------------------------------------*/
enum
{
  CMD_PWM_INIT     = 0x80,
  CMD_PWM_WRITE    = 0x81,
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void pwm_init(int prescaler, int period)
{
  uint8_t buf[10];

  buf[0] = CMD_PWM_INIT;
  buf[1] = prescaler;
  buf[2] = (period >> 0) & 0xff;
  buf[3] = (period >> 8) & 0xff;
  buf[4] = (period >> 16) & 0xff;
  buf[5] = (period >> 24) & 0xff;
  dgw_cmd(buf, sizeof(buf), 6);
}

//-----------------------------------------------------------------------------
void pwm_write(int channel, int value)
{
  uint8_t buf[10];

  buf[0] = CMD_PWM_WRITE;
  buf[1] = channel;
  buf[2] = (value >> 0) & 0xff;
  buf[3] = (value >> 8) & 0xff;
  buf[4] = (value >> 16) & 0xff;
  buf[5] = (value >> 24) & 0xff;
  dgw_cmd(buf, sizeof(buf), 6);
}

