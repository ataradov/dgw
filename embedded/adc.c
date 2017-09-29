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
#include "nvm_data.h"
#include "dac.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(ADC,      A, 3)

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void adc_init(void)
{
  HAL_GPIO_ADC_in();
  HAL_GPIO_ADC_pmuxen(HAL_GPIO_PMUX_B);

  PM->APBCMASK.reg |= PM_APBCMASK_ADC;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(ADC_GCLK_ID) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  ADC->CTRLA.reg = ADC_CTRLA_SWRST;
  while (ADC->CTRLA.reg & ADC_CTRLA_SWRST);

  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1 | ADC_REFCTRL_REFCOMP;
  ADC->CTRLB.reg = ADC_CTRLB_RESSEL_16BIT | ADC_CTRLB_PRESCALER_DIV32;
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_128;
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS_PIN1 | ADC_INPUTCTRL_MUXNEG_GND |
      ADC_INPUTCTRL_GAIN_DIV2;
  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(NVM_READ_CAL(ADC_BIASCAL)) |
      ADC_CALIB_LINEARITY_CAL(NVM_READ_CAL(ADC_LINEARITY));

  ADC->CTRLA.reg = ADC_CTRLA_ENABLE;
}

//-----------------------------------------------------------------------------
int adc_read(void)
{
  ADC->SWTRIG.reg = ADC_SWTRIG_START;
  while (0 == (ADC->INTFLAG.reg & ADC_INTFLAG_RESRDY));

  return ADC->RESULT.reg;
}

