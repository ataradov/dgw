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
#include "gpio.h"

/*- Definitions -------------------------------------------------------------*/
#define GPIO_COUNT   8

HAL_GPIO_PIN(0,   B, 0);
HAL_GPIO_PIN(1,   B, 1);
HAL_GPIO_PIN(2,   B, 2);
HAL_GPIO_PIN(3,   B, 3);
HAL_GPIO_PIN(4,   B, 4);
HAL_GPIO_PIN(5,   B, 5);
HAL_GPIO_PIN(6,   B, 6);
HAL_GPIO_PIN(7,   B, 7);

enum
{
  GPIO_0_MSK   = (1 << 0),
  GPIO_1_MSK   = (1 << 1),
  GPIO_2_MSK   = (1 << 2),
  GPIO_3_MSK   = (1 << 3),
  GPIO_4_MSK   = (1 << 4),
  GPIO_5_MSK   = (1 << 5),
  GPIO_6_MSK   = (1 << 6),
  GPIO_7_MSK   = (1 << 7),
};

enum
{
  GPIO_CONF_DISABLE  = 1 << 0,
  GPIO_CONF_INPUT    = 1 << 1,
  GPIO_CONF_OUTPUT   = 1 << 2,
  GPIO_CONF_PULLUP   = 1 << 3,
  GPIO_CONF_PULLDOWN = 1 << 4,
  GPIO_CONF_SET      = 1 << 3, // Intentional overlap with PULLUP / PULLDOWN
  GPIO_CONF_CLR      = 1 << 4,
};

/*- Variables ---------------------------------------------------------------*/
static int gpio_config[GPIO_COUNT];

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
#define GEN_CONFIG_FN(i) \
static void gpio_config_fn_##i(int conf) \
{ \
  if (conf & GPIO_CONF_DISABLE) \
  { \
    HAL_GPIO_##i##_in(); \
    HAL_GPIO_##i##_clr(); \
    HAL_GPIO_##i##_pullen(0); \
  } \
  else if (conf & GPIO_CONF_INPUT) \
  { \
    HAL_GPIO_##i##_in(); \
    HAL_GPIO_##i##_pullen(conf & (GPIO_CONF_PULLUP | GPIO_CONF_PULLDOWN)); \
    HAL_GPIO_##i##_write(conf & GPIO_CONF_PULLUP); \
  } \
  else if (conf & GPIO_CONF_OUTPUT) \
  { \
    HAL_GPIO_##i##_out(); \
    HAL_GPIO_##i##_pullen(0); \
    HAL_GPIO_##i##_write(conf & GPIO_CONF_SET); \
  } \
  gpio_config[i] = conf; \
}

//-----------------------------------------------------------------------------
GEN_CONFIG_FN(0)
GEN_CONFIG_FN(1)
GEN_CONFIG_FN(2)
GEN_CONFIG_FN(3)
GEN_CONFIG_FN(4)
GEN_CONFIG_FN(5)
GEN_CONFIG_FN(6)
GEN_CONFIG_FN(7)

//-----------------------------------------------------------------------------
void gpio_init(void)
{
  for (int i = 0; i < GPIO_COUNT; i++)
    gpio_configure(i, GPIO_CONF_DISABLE);
}

//-----------------------------------------------------------------------------
void gpio_configure(int index, int conf)
{
  if (0 == index)
    gpio_config_fn_0(conf);
  else if (1 == index)
    gpio_config_fn_1(conf);
  else if (2 == index)
    gpio_config_fn_2(conf);
  else if (3 == index)
    gpio_config_fn_3(conf);
  else if (4 == index)
    gpio_config_fn_4(conf);
  else if (5 == index)
    gpio_config_fn_5(conf);
  else if (6 == index)
    gpio_config_fn_6(conf);
  else if (7 == index)
    gpio_config_fn_7(conf);
}

//-----------------------------------------------------------------------------
int gpio_read(int index)
{
  if (0 == index)
    return HAL_GPIO_0_read();
  else if (1 == index)
    return HAL_GPIO_1_read();
  else if (2 == index)
    return HAL_GPIO_2_read();
  else if (3 == index)
    return HAL_GPIO_3_read();
  else if (4 == index)
    return HAL_GPIO_4_read();
  else if (5 == index)
    return HAL_GPIO_5_read();
  else if (6 == index)
    return HAL_GPIO_6_read();
  else if (7 == index)
    return HAL_GPIO_7_read();
  else
    return 0;
}

//-----------------------------------------------------------------------------
void gpio_write(int index, int value)
{
  if (0 == (gpio_config[index] & GPIO_CONF_OUTPUT))
    return;

  if (0 == index)
    HAL_GPIO_0_write(value);
  else if (1 == index)
    HAL_GPIO_1_write(value);
  else if (2 == index)
    HAL_GPIO_2_write(value);
  else if (3 == index)
    HAL_GPIO_3_write(value);
  else if (4 == index)
    HAL_GPIO_4_write(value);
  else if (5 == index)
    HAL_GPIO_5_write(value);
  else if (6 == index)
    HAL_GPIO_6_write(value);
  else if (7 == index)
    HAL_GPIO_7_write(value);
}

