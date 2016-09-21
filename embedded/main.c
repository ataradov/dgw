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
#include "samd21.h"
#include "hal_gpio.h"
#include "nvm_data.h"
#include "debug.h"
#include "usb.h"
#include "i2c_master.h"
#include "gpio.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(LED,      B, 30)

#define APP_EP_SEND    1
#define APP_EP_RECV    2

enum
{
  CMD_I2C_INIT     = 0x00,
  CMD_I2C_START    = 0x01,
  CMD_I2C_STOP     = 0x02,
  CMD_I2C_READ     = 0x03,
  CMD_I2C_WRITE    = 0x04,

  CMD_GPIO_CONFIG  = 0x10,
  CMD_GPIO_READ    = 0x11,
  CMD_GPIO_WRITE   = 0x12,
};

/*- Variables ---------------------------------------------------------------*/
ALIGNED(4) uint8_t app_usb_recv_buffer[64];
ALIGNED(4) uint8_t app_usb_send_buffer[64]; // TODO: use buffers from components
ALIGNED(4) uint8_t app_response_buffer[64];

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void irq_handler_tc3(void)
{
  if (TC3->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1))
  {
    HAL_GPIO_LED_toggle();
    TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
  }
}

//-----------------------------------------------------------------------------
static void timer_init(void)
{
  PM->APBCMASK.reg |= PM_APBCMASK_TC3;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC3_GCLK_ID) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ |
      TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_PRESCSYNC_RESYNC;

  TC3->COUNT16.COUNT.reg = 0;

  TC3->COUNT16.CC[0].reg = (F_CPU / 1000ul / 1024) * 500;
  TC3->COUNT16.COUNT.reg = 0;

  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

  TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC(1);
  NVIC_EnableIRQ(TC3_IRQn);
}

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  uint32_t coarse, fine;

  SYSCTRL->OSC8M.bit.PRESC = 0;

  SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET |
      SYSCTRL_INTFLAG_DFLLRDY;

  NVMCTRL->CTRLB.bit.RWS = 2;

  coarse = NVM_READ_CAL(NVM_DFLL48M_COARSE_CAL);
  fine = NVM_READ_CAL(NVM_DFLL48M_FINE_CAL);

  SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000);
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine);

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM |
      SYSCTRL_DFLLCTRL_BPLCKC | SYSCTRL_DFLLCTRL_CCDIS | SYSCTRL_DFLLCTRL_STABLE;

  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) |
      GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  asm volatile ("cpsie i");
}
/*
//-----------------------------------------------------------------------------
static uint32_t get_uint32(uint8_t *data)
{
  return ((uint32_t)data[0] << 0) | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

//-----------------------------------------------------------------------------
static void set_uint32(uint8_t *data, uint32_t value)
{
  data[0] = (value >> 0) & 0xff;
  data[1] = (value >> 8) & 0xff;
  data[2] = (value >> 16) & 0xff;
  data[3] = (value >> 24) & 0xff;
}
*/
//-----------------------------------------------------------------------------
void usb_send_callback(void)
{
}

//-----------------------------------------------------------------------------
void usb_recv_callback(void)
{
  app_response_buffer[0] = app_usb_recv_buffer[0];
  app_response_buffer[1] = true;

  if (CMD_I2C_INIT == app_usb_recv_buffer[0])
  {
  }
  else if (CMD_I2C_START == app_usb_recv_buffer[0])
  {
    int addr = app_usb_recv_buffer[1];
    app_response_buffer[1] = i2c_start(addr);
  }
  else if (CMD_I2C_STOP == app_usb_recv_buffer[0])
  {
    app_response_buffer[1] = i2c_stop();
  }
  else if (CMD_I2C_READ == app_usb_recv_buffer[0])
  {
    int i;

    for (i = 0; i < app_usb_recv_buffer[1]; i++)
    {
      bool last = ((i == (app_usb_recv_buffer[1] - 1)) && app_usb_recv_buffer[2]);

      if (!i2c_read_byte(&app_response_buffer[3 + i], last))
      {
        app_response_buffer[1] = false;
        break;
      }
    }

    app_response_buffer[2] = i;
  }
  else if (CMD_I2C_WRITE == app_usb_recv_buffer[0])
  {
    int i;

    for (i = 0; i < app_usb_recv_buffer[1]; i++)
    {
      if (!i2c_write_byte(app_usb_recv_buffer[2 + i]))
      {
        app_response_buffer[1] = false;
        break;
      }
    }

    app_response_buffer[2] = i;
  }
  else if (CMD_GPIO_CONFIG == app_usb_recv_buffer[0])
  {
    int cnt = app_usb_recv_buffer[1];

    for (int i = 0; i < cnt; i++)
    {
      int index = app_usb_recv_buffer[2 + i*2];
      int conf = app_usb_recv_buffer[3 + i*2];

      gpio_configure(index, conf);
    }
  }
  else if (CMD_GPIO_READ == app_usb_recv_buffer[0])
  {
    int cnt = app_usb_recv_buffer[1];

    for (int i = 0; i < cnt; i++)
    {
      int index = app_usb_recv_buffer[2 + i];

      app_response_buffer[2 + i] = gpio_read(index);
    }
  }
  else if (CMD_GPIO_WRITE == app_usb_recv_buffer[0])
  {
    int cnt = app_usb_recv_buffer[1];

    for (int i = 0; i < cnt; i++)
    {
      int index = app_usb_recv_buffer[2 + i*2];
      int value = app_usb_recv_buffer[3 + i*2];

      gpio_write(index, value);
    }
  }

  usb_send(APP_EP_SEND, app_response_buffer, sizeof(app_response_buffer), usb_send_callback);

  usb_recv(APP_EP_RECV, app_usb_recv_buffer, sizeof(app_usb_recv_buffer), usb_recv_callback);
}

//-----------------------------------------------------------------------------
void usb_configuration_callback(int config)
{
  usb_recv(APP_EP_RECV, app_usb_recv_buffer, sizeof(app_usb_recv_buffer), usb_recv_callback);

  (void)config;
}

//-----------------------------------------------------------------------------
int main(void)
{
  sys_init();
  timer_init();
  debug_init();

  debug_puts("\r\n--- start ---\r\n");

  usb_init();
  i2c_init();
  gpio_init();

  HAL_GPIO_LED_out();
  HAL_GPIO_LED_clr();

  while (1)
  {
  }

  return 0;
}

