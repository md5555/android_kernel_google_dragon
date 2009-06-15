#ifndef _TRACE_FS_H
#define _TRACE_FS_H

#include <linux/fs.h>
#include <linux/tracepoint.h>

DECLARE_TRACE(do_sys_open,
	TP_PROTO(struct file *filp, int flags, int mode, long fd),
		TP_ARGS(filp, flags, mode, fd));

#endif
