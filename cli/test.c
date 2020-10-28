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
#include "adc.h"
#include "dac.h"
#include "pwm.h"
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
static void eeprom_test(void)
{
  uint8_t buf[1024];

  printf("--- EEPROM Test ---\n");

  i2c_init(400000);

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

  i2c_init(400000);

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
static uint8_t sdc_crc7(uint8_t *data, int size)
{
  uint8_t crc = 0;

  for (int j = 0; j < size; j++)
  {
    uint8_t byte = data[j];

    for (int i = 0; i < 8; i++)
    {
      crc <<= 1;

      if ((byte & 0x80) ^ (crc & 0x80))
        crc ^= 0x09;

      byte <<= 1;
    }
  }

  return (crc << 1) | 1;
}

//-----------------------------------------------------------------------------
static uint8_t spi_write_byte(uint8_t value)
{
  spi_transfer(&value, &value, 1);
  return value;
}

//-----------------------------------------------------------------------------
static uint8_t sdc_command(uint8_t index, uint32_t arg)
{
  uint8_t buf[9];

  buf[0] = 0xff;
  buf[1] = index | 0x40;
  buf[2] = (arg >> 24) & 0xff;
  buf[3] = (arg >> 16) & 0xff;
  buf[4] = (arg >> 8) & 0xff;
  buf[5] = (arg >> 0) & 0xff;
  buf[6] = sdc_crc7(&buf[1], 5);
  buf[7] = 0xff;
  buf[8] = 0xff;

  spi_transfer(buf, buf, 9);

  return buf[8];
}

//-----------------------------------------------------------------------------
static void sdc_prepare(void)
{
  spi_ss(1);
  spi_init(100000, 0);

  for (int i = 0; i < 10; i++)
    spi_write_byte(0xff);

  spi_ss(0);

  while (0x01 != sdc_command(0/*SDC_GO_IDLE_STATE*/, 0));

  sdc_command(8, 0x000001aa);
  while (0xff != spi_write_byte(0xff));

  while (1)
  {
    sdc_command(55/*SDC_APP_CMD*/, 0);

    if (0 == sdc_command(41/*SDC_SEND_OP_COND*/, 0x40000000))
      break;
  }
}

//-----------------------------------------------------------------------------
static void sd_card_test(void)
{
  bool inserted;

  printf("--- SD Card Test ---\n");

  spi_init(8000000, 0);

  gpio_configure(GPIO_SD, GPIO_CONF_INPUT | GPIO_CONF_PULLUP);

  inserted = !gpio_read(GPIO_SD);

  printf("SD Card is %s\n", inserted ? "inserted" : "not inserted");

  if (inserted)
  {
    uint8_t cid[16];

    sdc_prepare();

    sdc_command(10/*SDC_SEND_CID*/, 0);

    while (0x00 != spi_write_byte(0xff));
    while (0xfe != spi_write_byte(0xfe));

    memset(cid, 0xff, sizeof(cid));
    spi_transfer(cid, cid, sizeof(cid));

    if (sdc_crc7(cid, 15) == cid[15])
    {
      char name[6];
      uint32_t sn;
      int date;

      memcpy(name, (char *)&cid[3], 5);
      name[5] = 0;

      sn = ((uint32_t)cid[9] << 24) | ((uint32_t)cid[10] << 16) | ((uint32_t)cid[11] << 8) | cid[12];
      date = ((uint16_t)cid[13] << 8) | cid[14];

      printf("Manufacturer ID  : 0x%02x\n", cid[0]);
      printf("Application ID   : %c%c\n", cid[1], cid[2]);
      printf("Product Name     : %s\n", name);
      printf("Product Revision : %d.%d\n", cid[8] / 16, cid[8] % 16);
      printf("Product S/N      : 0x%08x\n", sn);
      printf("Manufacture Date : %d / %d\n", date % 16, 2000 + date / 16);
    }
    else
    {
      printf("CID response CRC fail\n");
    }

    spi_ss(1);
  }

  printf("\n");
}

//-----------------------------------------------------------------------------
static void adc_dac_test(void)
{
  printf("--- ADC/DAC Test ---\n");

  adc_init();
  dac_init();

  for (int dac_value = 0; dac_value < 1024; dac_value += 93)
  {
    int adc_value;

    dac_write(dac_value);
    adc_value = adc_read() >> 6;

    printf("DAC = %4d, ADC = %4d (error = %d)\n", dac_value, adc_value,
        dac_value - adc_value);
  }
}

//-----------------------------------------------------------------------------
static void pwm_test(void)
{
  pwm_init(PWM_PRESCALER_1, 10000);

  pwm_write(0, 5000);
  pwm_write(1, 1000);
}

//-----------------------------------------------------------------------------
void test(void)
{
  eeprom_test();
  temp_test();
  gpio_test();
  sd_card_test();
  adc_dac_test();
  pwm_test();
}

