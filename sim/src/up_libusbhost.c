/*******************************************************************************
 * arch/sim/src/up_libusbhost.c
 *
 *   Copyright (C) 2015 Brennan Ashton. All rights reserved.
 *   Authors: Brennan Ashton <bashton@brennanashton.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include <libusb-1.0/libusb.h>
/*****************************************************************************
 * Private Types
 ****************************************************************************/

static int hotplug_callback(struct libusb_context *ctx,
                            struct libusb_device *dev,
                            libusb_hotplug_event event,
                            void *user_data)
{
  int ret;
  struct libusb_device_descriptor desc;

  if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED)
    {
      ret = libusb_get_device_descriptor(dev, &desc);
      if (ret != LIBUSB_SUCCESS)
        {
          printf("Could not read device descriptor\n");
        }
      else
        {
          printf("Something about the event\n");
        }
    }
  return 0;
}

int sim_libusb_hotplug_initialize(void)
{
  return 0;
}

int sim_libusb_initialize(void)
{
  libusb_init(NULL);
  return 0;
}
