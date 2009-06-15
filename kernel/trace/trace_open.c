/*
 * trace open calls
 * Copyright (C) 2009 Intel Corporation
 *
 * Based extensively on trace_sched_switch.c
 * Copyright (C) 2007 Steven Rostedt <srostedt@redhat.com>
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/kallsyms.h>
#include <linux/uaccess.h>
#include <linux/ftrace.h>
#include <trace/fs.h>

#include "trace.h"


static struct trace_array	*ctx_trace;
static int __read_mostly	open_trace_enabled;
static atomic_t			open_ref;

static void probe_do_sys_open(struct file *filp, int flags, int mode, long fd)
{
	char *buf;
	char *fname;

	if (!atomic_read(&open_ref))
		return;

	if (!open_trace_enabled)
		return;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return;

	fname = d_path(&filp->f_path, buf, PAGE_SIZE);
	if (IS_ERR(fname))
		goto out;

	trace_printk("%s: open(\"%s\", %d, %d) = %ld\n",
		current->comm, fname, flags, mode, fd);

out:
	kfree(buf);
}

static void open_trace_reset(struct trace_array *tr)
{
	tr->time_start = ftrace_now(tr->cpu);
	tracing_reset_online_cpus(tr);
}

static int open_trace_register(void)
{
	int ret;

	ret = register_trace_do_sys_open(probe_do_sys_open);
	if (ret) {
		pr_info("open trace: Could not activate tracepoint"
			" probe to do_open\n");
	}

	return ret;
}

static void open_trace_unregister(void)
{
	unregister_trace_do_sys_open(probe_do_sys_open);
}

static void open_trace_start(void)
{
	long ref;

	ref = atomic_inc_return(&open_ref);
	if (ref == 1)
		open_trace_register();
}

static void open_trace_stop(void)
{
	long ref;

	ref = atomic_dec_and_test(&open_ref);
	if (ref)
		open_trace_unregister();
}

void open_trace_start_cmdline_record(void)
{
	open_trace_start();
}

void open_trace_stop_cmdline_record(void)
{
	open_trace_stop();
}

static void open_start_trace(struct trace_array *tr)
{
	open_trace_reset(tr);
	open_trace_start_cmdline_record();
	open_trace_enabled = 1;
}

static void open_stop_trace(struct trace_array *tr)
{
	open_trace_enabled = 0;
	open_trace_stop_cmdline_record();
}

static int open_trace_init(struct trace_array *tr)
{
	ctx_trace = tr;

	open_start_trace(tr);
	return 0;
}

static void reset_open_trace(struct trace_array *tr)
{
	open_stop_trace(tr);
}

static struct tracer open_trace __read_mostly =
{
	.name		= "open",
	.init		= open_trace_init,
	.reset		= reset_open_trace,
};

__init static int init_open_trace(void)
{
	int ret = 0;

	if (atomic_read(&open_ref))
		ret = open_trace_register();
	if (ret) {
		pr_info("error registering open trace\n");
		return ret;
	}
	return register_tracer(&open_trace);
}
device_initcall(init_open_trace);

