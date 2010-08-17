/*
 *  tusbd/driver.c
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "usbd.h"
struct minor_descriptor *g_pminorcontrol = NULL;
int g_major = -1;
struct waitable_queue g_exclusions;

static struct cdev s_cdev;

static int __init usbd_init(void)
{
	int result;

	TRACE("usbd_init: ++\n");

	do
	{
		result = usbd_cdev_init_module(&g_major);

		if( result != 0 )
		{
			TRACE("usbd_init: init_cdev_module failed. Error number %d\n", result);
			break;
		}

		result = usbd_cdev_create(&s_cdev);
		if( result != 0 )
		{
			TRACE("usbd_init: create_cdev failed. Error number %d\n", result);
			break;
		}

		g_pminorcontrol = usbd_mc_alloc_descriptor();

		if( g_pminorcontrol == NULL )
		{
			result = -1;
			TRACE("usbd_init: usbd_mc_alloc_descriptor. Error number %d\n", result);
			break;
		}

		result = usbd_cdev_alloc_minor(g_pminorcontrol);
		if( result != 0 )
		{
			TRACE("usbd_init: usbd_cdev_alloc_minor failed. Error number %d\n", result);
			break;
		}

#ifdef _USBD_ENABLE_STUB_
		usbd_wq_init(&g_exclusions);

		result = usbd_usbdevice_init_module();
		if( result != 0 )
		{
			TRACE("usbd_init: usbd_usbdevice_init_module failed. Error number %d\n", result);
			break;
		}
#endif

#ifdef _USBD_ENABLE_VHCI_
		result = usbd_vhci_device_init_module();
		if( result != 0 )
		{
			TRACE("usbd_init: usbd_usbdevice_init_module failed. Error number %d\n", result);
			break;
		}
#endif


	} while(0);

	TRACE("usbd_init: --\n");

	return result;
}

static void __exit usbd_exit(void)
{
#ifdef _USBD_ENABLE_STUB_
	usbd_usbdevice_deinit_module();
#endif

#ifdef _USBD_ENABLE_VHCI_
	usbd_vhci_device_deinit_module();
#endif

	usbd_cdev_free_minor(0);
	usbd_cdev_destroy(&s_cdev);
	usbd_cdev_deinit_module();
#ifdef _USBD_ENABLE_STUB_
	usbd_wq_deinit(&g_exclusions);
#endif
	TRACE("usbd_exit\n");
}

module_init(usbd_init);
module_exit(usbd_exit);
MODULE_LICENSE("GPL");
