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
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <libudev.h>
#include "edbg.h"
#include "dgw.h"

/*- Variables ---------------------------------------------------------------*/
static int dgw_fd = -1;
static uint8_t hid_buffer[1024 + 1];
static int report_size = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
int dgw_enumerate(dgw_t *dgws, int size)
{
  struct udev *udev;
  struct udev_enumerate *enumerate;
  struct udev_list_entry *devices, *dev_list_entry;
  struct udev_device *dev, *parent;
  int rsize = 0;

  udev = udev_new();
  check(udev, "unable to create udev object");

  enumerate = udev_enumerate_new(udev);
  udev_enumerate_add_match_subsystem(enumerate, "hidraw");
  udev_enumerate_scan_devices(enumerate);
  devices = udev_enumerate_get_list_entry(enumerate);

  udev_list_entry_foreach(dev_list_entry, devices)
  {
    const char *path;

    path = udev_list_entry_get_name(dev_list_entry);
    dev = udev_device_new_from_syspath(udev, path);

    parent = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");

    if (NULL == parent)
      continue;

    if (rsize < size)
    {
      const char *serial = udev_device_get_sysattr_value(parent, "serial");
      const char *manufacturer = udev_device_get_sysattr_value(parent, "manufacturer");
      const char *product = udev_device_get_sysattr_value(parent, "product");

      dgws[rsize].path = strdup(udev_device_get_devnode(dev));
      dgws[rsize].serial = serial ? strdup(serial) : "<unknown>";
      dgws[rsize].manufacturer = manufacturer ? strdup(manufacturer) : "<unknown>";
      dgws[rsize].product = product ? strdup(product) : "<unknown>";
      dgws[rsize].vid = strtol(udev_device_get_sysattr_value(parent, "idVendor"), NULL, 16);
      dgws[rsize].pid = strtol(udev_device_get_sysattr_value(parent, "idProduct"), NULL, 16);

      if (strstr(dgws[rsize].product, "Data Gateway"))
        rsize++;
    }

    udev_device_unref(parent);
  }

  udev_enumerate_unref(enumerate);
  udev_unref(udev);

  return rsize;
}

//-----------------------------------------------------------------------------
static int parse_hid_report_desc(uint8_t *data, int size)
{
  uint32_t count = 0;
  uint32_t input = 0;
  uint32_t output = 0;

  // This is a very primitive parser, but CMSIS-DAP descriptors are pretty uniform
  for (int i = 0; i < size; )
  {
    int prefix = data[i++];
    int bTag = (prefix >> 4) & 0x0f;
    int bType = (prefix >> 2) & 0x03;
    int bSize = prefix & 0x03;

    bSize = (3 == bSize) ? 4 : bSize;

    if (1 == bType && 9 == bTag)
    {
      count = 0;

      for (int j = 0; j < bSize; j++)
        count |= (data[i + j] << (j * 8));
    }
    else if (0 == bType && 8 == bTag)
      input = count;
    else if (0 == bType && 9 == bTag)
      output = count;

    i += bSize;
  }

  if (input != output)
    error_exit("input and output report sizes do not match");

  if (64 != input && 512 != input && 1024 != input)
    error_exit("detected report size (%d) is not 64, 512 or 1024", input);

  return input;
}

//-----------------------------------------------------------------------------
void dgw_open(dgw_t *dgw)
{
  struct hidraw_report_descriptor rpt_desc;
  struct hidraw_devinfo info;
  int desc_size, res;

  dgw_fd = open(dgw->path, O_RDWR);

  if (dgw_fd < 0)
    perror_exit("unable to open device");

  memset(&rpt_desc, 0, sizeof(rpt_desc));
  memset(&info, 0, sizeof(info));

  res = ioctl(dgw_fd, HIDIOCGRDESCSIZE, &desc_size);
  if (res < 0)
    perror_exit("dgw ioctl()");

  rpt_desc.size = desc_size;
  res = ioctl(dgw_fd, HIDIOCGRDESC, &rpt_desc);
  if (res < 0)
    perror_exit("dgw ioctl()");

  report_size = parse_hid_report_desc(rpt_desc.value, rpt_desc.size);
}

//-----------------------------------------------------------------------------
void dgw_close(void)
{
  if (dgw_fd)
    close(dgw_fd);
}

//-----------------------------------------------------------------------------
int dgw_get_report_size(void)
{
  return report_size;
}

//-----------------------------------------------------------------------------
int dgw_cmd(uint8_t *data, int size, int rsize)
{
  char cmd = data[0];
  int res;

  memset(hid_buffer, 0xff, report_size + 1);

  hid_buffer[0] = 0x00; // Report ID
  memcpy(&hid_buffer[1], data, rsize);

  res = write(dgw_fd, hid_buffer, report_size + 1);
  if (res < 0)
    perror_exit("dgw write()");

  res = read(dgw_fd, hid_buffer, report_size + 1);
  if (res < 0)
    perror_exit("dgw read()");

  check(res, "empty response received");

  check(hid_buffer[0] == cmd, "invalid response received");

  res--;
  memcpy(data, &hid_buffer[1], (size < res) ? size : res);

  return res;
}

