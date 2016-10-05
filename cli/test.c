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
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/*- Definitions -------------------------------------------------------------*/
#define EEPROM_ADDR   (0x54 << 1)
#define TEMP_ADDR     (0x4f << 1)
#define GPIO_1        6
#define GPIO_2        7
#define GPIO_SD       5
#define GPIO_LED      2

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void init(void)
{
  int freq;

  printf("--- Initialization ---\n");

  freq = i2c_init(400000);
  printf("I2C frequency = %d\n", freq);

  freq = spi_init(8000000);
  printf("SPI frequency = %d\n", freq);

  printf("\n");
}

//-----------------------------------------------------------------------------
static void eeprom_test(void)
{
  uint8_t buf[1024];

  printf("--- EEPROM Test ---\n");

  srand(time(NULL));

  for (int i = 0; i < (int)sizeof(buf); i++) 
    buf[i] = rand();

  verbose("Writing 1KB of data... ");

  for (int addr = 0; addr < (int)sizeof(buf); addr += 16)
  {
    uint8_t data[17];
    int i2c_addr = EEPROM_ADDR | ((addr >> 8) << 1);

    data[0] = addr & 0xff;
    memcpy(&data[1], &buf[addr], 16);

    while (!i2c_write(i2c_addr, data, 17));

    verbose(".");
  }

  verbose("done\n");

  verbose("Verifying... ");

  for (int addr = 0; addr < (int)sizeof(buf); addr += 16)
  {
    uint8_t data[16];
    int i2c_addr = EEPROM_ADDR | ((addr >> 8) << 1);

    data[0] = addr & 0xff;

    while (!i2c_write(i2c_addr, data, 1));
    while (!i2c_read(i2c_addr, data, 16));

    for (int i = 0; i < 16; i++)
    {
      if (buf[addr + i] != data[i])
      {
        warning("EEPROM error at 0x%04x - expected 0x%02x, got 0x%02x",
            addr, buf[addr + i], data[i]);
      }
    }

    verbose(".");
  }

  verbose("done\n");

  printf("\n");
}

//-----------------------------------------------------------------------------
static void temp_test(void)
{
  uint8_t buf[3];
  float temp;

  printf("--- Temperature Test ---\n");

  // Enable 12-bit mode
  buf[0] = 0x01;
  buf[1] = 0x60;
  buf[2] = 0x00;
  i2c_write(TEMP_ADDR, buf, 3);

  buf[0] = 0x00;
  i2c_write(TEMP_ADDR, buf, 1);

  // Read the remperature
  i2c_read(TEMP_ADDR, buf, 2);

  temp = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]) / 256.0;

  printf("Temperature = %0.2f C\n", temp);

  printf("\n");
}

//-----------------------------------------------------------------------------
static void gpio_test(void)
{
  bool shorted = true;

  printf("--- GPIO Test ---\n");

  // GPIO
  gpio_configure(GPIO_1, GPIO_CONF_OUTPUT | GPIO_CONF_SET);
  gpio_configure(GPIO_2, GPIO_CONF_INPUT | GPIO_CONF_PULLUP);

  gpio_write(GPIO_1, 1);
  shorted &= (1 == gpio_read(GPIO_2));

  gpio_write(GPIO_1, 0);
  shorted &= (0 == gpio_read(GPIO_2));

  printf("GPIO 1 and 2 are %s\n", shorted ? "shorted" : "open");

  // LED
  verbose("Blinking an LED... ");

  gpio_configure(GPIO_LED, GPIO_CONF_OUTPUT | GPIO_CONF_CLR);

  for (int i = 0; i < 5; i++)
  {
    verbose("on ");
    gpio_write(GPIO_LED, 0);
    usleep(500*1000);

    verbose("off ");
    gpio_write(GPIO_LED, 1);
    usleep(500*1000);
  }

  printf("\n\n");
}

//-----------------------------------------------------------------------------
static void sd_card_test(void)
{
  printf("--- SD Card Test ---\n");

  gpio_configure(GPIO_SD, GPIO_CONF_INPUT | GPIO_CONF_PULLUP);

  printf("SD Card is %s\n", gpio_read(GPIO_SD) ? "not inserted" : "inserted");





  printf("\n");
}

//-----------------------------------------------------------------------------
void test(void)
{
  init();
  eeprom_test();
  temp_test();
  gpio_test();
  sd_card_test();
}

