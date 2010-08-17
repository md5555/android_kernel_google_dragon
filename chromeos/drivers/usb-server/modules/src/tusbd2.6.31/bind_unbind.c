/*
 *  tusbd/bind_unbind.c
 *
 *  Copyright (C) 2007-2008 IncentivesPro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The following code is from 2.6.21 and 2.6.15 kernel sources
 *
 */
#include "usbd.h"
#include <linux/kobject.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)


int driver_probe_device(struct device_driver * drv, struct device * dev);

struct bus_type * get_bus(struct bus_type * bus)
{
	return bus ? container_of(subsys_get(&bus->subsys), struct bus_type, subsys) : NULL;
}

void put_bus(struct bus_type * bus)
{
	subsys_put(&bus->subsys);
}


DECLARE_MUTEX(found_dev_lock);
static struct device *found_dev;
static int usbd_bus_find_device_helper(struct device *dev, void *data)
{
	const char *name = data;

	if (strcmp(name, dev->bus_id) == 0)
	{
		found_dev = get_device(dev);
		return 1;
	}
	return 0;

}

struct device * usbd_bus_find_device(struct bus_type *bus, const char *bus_id)
{
    struct device *result;
    down(&found_dev_lock);
    found_dev = NULL;
    bus_for_each_dev(bus, NULL, (void*)bus_id, usbd_bus_find_device_helper);
    result = found_dev;
    up(&found_dev_lock);
    return result;
}


int driver_unbind(struct device_driver *drv, const char *buf)
{
	struct bus_type *bus = get_bus(drv->bus);
	struct device *dev;
	int err = -ENODEV;

	dev = usbd_bus_find_device(bus, (void *)buf);
	if (dev && dev->driver == drv) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,12)
		if (dev->parent)	// Needed for USB
			down(&dev->parent->sem);
#endif			
		device_release_driver(dev);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,12)
		if (dev->parent)
			up(&dev->parent->sem);
#endif
		err = 0;
	}
	put_device(dev);
	put_bus(bus);
	return err;
}

int driver_bind(struct device_driver *drv, const char *buf)
{
	struct bus_type *bus;
	struct device *dev;
	int err = -ENODEV;
	
	TRACE("driver_bind: ++\n");
	TRACE("driver_bind: getting bus...\n");
	bus = get_bus(drv->bus);
	TRACE("driver_bind: finding device...\n");
	dev = usbd_bus_find_device(bus, (void *)buf);
	if (dev && dev->driver == NULL) 
	{
	    TRACE("driver_bind: device found\n");
	    if (dev->parent)	// Needed for USB
	    {
		TRACE("driver_bind: locking parent...\n");
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,12)
		down(&dev->parent->sem);
#endif
	    }
	    TRACE("driver_bind: locking device...\n");
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,12)
	    down(&dev->sem);
#endif
	    TRACE("driver_bind: probing device...\n");
	    err = driver_probe_device(drv, dev);
	    TRACE("driver_bind: unlocking device...\n");
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,12)
	    up(&dev->sem);
#endif
	    if (dev->parent)
	    {
    		TRACE("driver_bind: unlocking parent...\n");
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,12)
		up(&dev->parent->sem);
#endif
	    }

	    if (err > 0) 		// success
	    	err = 0;
	    else if (err == 0)	// driver didn't accept device
	    	err = -ENODEV;
	} else
	{
	    TRACE("driver_bind: device not found\n");
	}
	put_device(dev);
	put_bus(bus);
	TRACE("driver_bind: -- result=%d\n", err);
	return err;
}

int driver_probe_device(struct device_driver * drv, struct device * dev)
{
	int ret = 0;

	if (drv->bus->match && !drv->bus->match(dev, drv))
		goto Done;

	TRACE("%s: Matched Device %s with Driver %s\n",
		 drv->bus->name, dev->bus_id, drv->name);

	dev->driver = drv;

	if (drv->probe) {
		ret = drv->probe(dev);
		if (ret) {
			dev->driver = NULL;
			goto ProbeFailed;
		}
	}

	device_bind_driver(dev);
	ret = 1;
	TRACE("%s: Bound Device %s to Driver %s\n",
		 drv->bus->name, dev->bus_id, drv->name);
	goto Done;

 ProbeFailed:
	if (ret == -ENODEV || ret == -ENXIO) {
		// Driver matched, but didn't support device
		// or device not found.
		// Not an error; keep going.
		//
		ret = 0;
	} else {
		// driver matched but the probe failed
		TRACE("%s: probe of %s failed with error %d\n",
		       drv->name, dev->bus_id, ret);
	}
 Done:
	return ret;
}

/***************************************************************************************************/

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
struct usb_device *usbd_find_usb_device_by_busid(const char *busid, struct usb_device *root_dev)
{
    struct bus_type *bus;
    bus = find_bus("usb");
    if(bus)
    {
	struct device *dev =  usbd_bus_find_device(bus, busid);
	
	if(dev) return to_usb_device(dev);
    }
    return NULL;
}
#else

extern struct list_head usb_bus_list;
extern struct mutex usb_bus_list_lock;

struct usb_device *usbd_find_usb_device_by_busid(const char *busid, struct usb_device *root_dev)
{
    struct usb_device *child_dev = NULL;
    struct usb_device *result = NULL;

    int child;

    if( root_dev == NULL )
    {
        struct usb_bus *bus;
        
        // loop through all USB busses
	mutex_lock(&usb_bus_list_lock);
	list_for_each_entry(bus, &usb_bus_list, bus_list) 
	{
    	    // locate root hub and loop through all its children 
	    if (!bus->root_hub)
		continue;

	    TRACE("roothub bus_id=%s\n", bus->root_hub->dev.bus_id);
		
	    usb_lock_device(bus->root_hub);
	    result = usbd_find_usb_device_by_busid(busid, bus->root_hub);
	    usb_unlock_device(bus->root_hub);
	    
	    if( result ) 
		break;
	}
	mutex_unlock(&usb_bus_list_lock);
	return result;
    } else
    {
	TRACE("usbd_find_device_by_busid: root device vid=%.4x pid=%.4x busid=%s\n", 
	    le16_to_cpu(root_dev->descriptor.idVendor), 
	    le16_to_cpu(root_dev->descriptor.idProduct), 
	    root_dev->dev.bus_id);

	if( strncmp(root_dev->dev.bus_id, busid, BUS_ID_SIZE) == 0 ) 
	{
	    get_device(&root_dev->dev);
	    return root_dev;
	}

	// look through all of the children of this device
	for (child = 0; child < root_dev->maxchild; ++child)
	{
	    if (root_dev->children[child]) 
	    {
    		child_dev = root_dev->children[child];
		usb_lock_device(child_dev);
		TRACE("usbd_find_device_by_busid: child device vid=%.4x pid=%.4x bus_id: %s\n", 
		    le16_to_cpu(child_dev->descriptor.idVendor), 
		    le16_to_cpu(child_dev->descriptor.idProduct),
		    child_dev->dev.bus_id);
		result = usbd_find_usb_device_by_busid(busid, child_dev);
		usb_unlock_device(child_dev);
		
		if( result ) break;
    	    }
	}
	return result;
    }
}
#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)

int usbd_bind_device_to_driver(struct usbdevice_descriptor *device, struct usb_driver* driver)
{
    struct usb_device *dev = NULL;
    struct usb_host_config *cfg;
	
    TRACE("usbd_bind_device_to_driver: ++ driver is %s\n", driver ? driver->name : "NO DRIVER");
	
    dev = usbd_find_usb_device_by_busid(device->bus_id, NULL);
	
    if( dev == NULL ) 
    {
	TRACE("usbd_bind_device_to_driver: device not found!\n");
        return -1;
    }

    cfg = dev->actconfig;
    
    if( cfg == NULL )
    {
	TRACE("usbd_bind_device_to_driver: nothing to bind, device has no active configuration!\n");
    } else
    {
	int i;
	TRACE("usbd_bind_device_to_driver: found active configuration\n");
	for(i=0; i<USB_MAXINTERFACES; i++)
	{
	    struct usb_interface *intf;
	    if(cfg->interface[i])
	    {
	        int res;
	        intf=cfg->interface[i];
    	        TRACE("usbd_bind_device_to_driver: found interface %d busid=%s driver=%.8x\n", i, intf->dev.bus_id, (u32)intf->dev.driver);
	        if(intf->dev.driver) 
	        {
		    TRACE("usbd_bind_device_to_driver: trying to unbind...\n");
    		    res = driver_unbind(intf->dev.driver, intf->dev.bus_id);
		    TRACE("usbd_bind_device_to_driver: driver_unbind returned %d\n", res);
		}
		
		if( driver )
		{
        	    TRACE("usbd_bind_device_to_driver: trying to bind...\n");
		    res = driver_bind(
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
			&driver->driver, 
#else
			&driver->drvwrap.driver,
#endif
			intf->dev.bus_id);
    		    TRACE("usbd_bind_device_to_driver: driver_uind returned %d\n", res);
    		}
	    }
	}
    }
    
    put_device(&dev->dev);

    TRACE("usbd_bind_device_to_driver: --\n");
    return 0;
}

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
