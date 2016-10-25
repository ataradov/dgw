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
#include "pwm.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(PWM_0,   A, 10)
HAL_GPIO_PIN(PWM_1,   A, 11)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void pwm_init(int prescaler, int period)
{
  HAL_GPIO_PWM_0_out();
  HAL_GPIO_PWM_0_pmuxen(HAL_GPIO_PMUX_F);

  HAL_GPIO_PWM_1_out();
  HAL_GPIO_PWM_1_pmuxen(HAL_GPIO_PMUX_F);

  PM->APBCMASK.reg |= PM_APBCMASK_TCC0;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TCC0_GCLK_ID) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  TCC0->CTRLA.reg = TCC_CTRLA_SWRST;
  while (TCC0->CTRLA.reg & TCC_CTRLA_SWRST);

  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER(prescaler) | TCC_CTRLA_PRESCSYNC_PRESC;
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  TCC0->PER.reg = period;
  TCC0->COUNT.reg = 0;
  TCC0->CC[2].reg = 0;
  TCC0->CC[3].reg = 0;
  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
}

//-----------------------------------------------------------------------------
void pwm_write(int channel, int value)
{
  TCC0->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  TCC0->COUNT.reg = 0;
  TCC0->CC[channel + 2].reg = value;
  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
}

