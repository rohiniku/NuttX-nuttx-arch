/*******************************************************************************
 * arch/sim/src/up_usbhost.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>

#include "up_internal.h"

#if defined(CONFIG_USBHOST) && defined(CONFIG_SIM_USB)

/*****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to stuct sim_usbhost_s.
   */

  struct usbhost_driver_s drvr;
  struct usbhost_roothubport_s rhport;
};

static struct sim_usbhost_s g_usbhost;

static inline void sim_sw_initialize(struct sim_usbhost_s *priv)
{
  struct usbhost_driver_s *drvr;

  /* Initialize the device operations */

  drvr                 = &priv->drvr;
  drvr->ep0configure   = NULL;
  drvr->epalloc        = NULL;
  drvr->epfree         = NULL;
  drvr->alloc          = NULL;
  drvr->free           = NULL;
  drvr->ioalloc        = NULL;
  drvr->iofree         = NULL;
  drvr->ctrlin         = NULL;
  drvr->ctrlout        = NULL;
  drvr->transfer       = NULL;
#ifdef CONFIG_USBHOST_ASYNCH
  drvr->asynch         = NULL;
#endif
  drvr->cancel         = NULL;
#ifdef CONFIG_USBHOST_HUB
  drvr->connect        = NULL;
#endif
  drvr->disconnect     = NULL;
}

int sim_usbhost_drvr_initialize(void)
{
  struct sim_usbhost_s *priv = &g_usbhost;

  sim_libusb_initialize();
  sim_sw_initialize(priv);

  return OK;
}

int sim_usbhost_hotplug_initialize(void)
{
  int ret;
  udbg("Registering libusb hotplug detection callbacks\n");
  ret = sim_libusb_hotplug_initialize();
  return ret;
}
#endif /* CONFIG_USBHOST && CONFIG_SIM_USB */
