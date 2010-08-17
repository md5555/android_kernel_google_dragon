#ifndef _USBD_FILTER_H_
#define _USBD_FILTER_H_

#include "nettypes.h"

static int usbd_is_autosharing_allowed(
	const uint16 vid, 
	const uint16 pid, 
	const uint16 rev,
	const uint8 deviceClass, 
	const uint8 deviceSubClass, 
	const uint8 deviceProtocol, 
	const void * reserved)
{
	return 1;
}

#endif // _USBD_FILTER_H_
