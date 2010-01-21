/*
 * drivers/input/keyboard/tegra-kbc.c
 *
 * Keyboard class input driver for the NVIDIA Tegra SoC internal matrix
 * keyboard controller
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>

#include <mach/nvrm_linux.h>
#include <nvddk_kbc.h>
#include <nvos.h>
#include <nvodm_kbc_keymapping.h>


static int tegra_kbc_event(struct input_dev *dev, unsigned int type,
	unsigned int code, int value)
{
	return 0;
}

struct NvOdmKeyVirtTableDetail **key_tab = NULL;
NvU32 key_tab_total = 0;

struct tegra_kbc_driver_data {
	struct input_dev	*input_dev;
	struct task_struct	*task;
	NvOsSemaphoreHandle	semaphore;
	NvDdkKbcHandle		ddkHandle;
	int			done;
};

#define in_table(_code, _tabl) \
	(((_code)>=(_tabl)->StartScanCode) && ((_code)<=(_tabl)->EndScanCode))

#define table_size(_tabl) ((_tabl)->EndScanCode - (_tabl)->StartScanCode + 1)

static NvU32 tegra_kbc_handle_keyev(struct tegra_kbc_driver_data *kbc)
{
	NvDdkKbcKeyEvent key_ev[16];
	NvU32 codes[16];
	NvU32 k_idx;
	NvU32 l_idx;
	NvU32 EventCount;
	NvU32 WaitTime;
	NvU32 i;
	NvU32 value;
	NvU32 key;

	WaitTime = NvDdkKbcGetKeyEvents(kbc->ddkHandle,
		&EventCount, codes, key_ev);

	for (i = 0; i < EventCount; i++) {
		if (key_ev[i] == NvDdkKbcKeyEvent_KeyPress) {
			value = 1;
		} else if (key_ev[i] == NvDdkKbcKeyEvent_KeyRelease) {
			value = 0;
		} else
			continue;

		key = 0;

		for (l_idx = 0; l_idx < key_tab_total; ++l_idx) {
			if (!in_table(codes[i], key_tab[l_idx]))
				continue;
			k_idx = codes[i] - key_tab[l_idx]->StartScanCode;
			key = key_tab[l_idx]->pVirtualKeyTable[k_idx];
		}

		input_report_key(kbc->input_dev, key, value);
	}

	if (WaitTime) {
		NvOsSleepMS(WaitTime);
	}

	return WaitTime;
}

static int tegra_kbc_thread(void *pdata)
{
	struct tegra_kbc_driver_data *kbc = pdata;
	NvU32 loop;

	for (;;) {
		/* FIXME should we use a NvOsSemaphoreWaitTimeout instead? */
		NvOsSemaphoreWait(kbc->semaphore);
		if (kbc->done)
			break;
		do {
			loop = tegra_kbc_handle_keyev(kbc);
		} while (loop);
	}

	return 0;
}

static void tegra_kbc_cleanup(struct tegra_kbc_driver_data *kbc)
{
	if (!kbc)
		return;

	if (kbc->task) {
		kbc->done = 1;
		NvOsSemaphoreSignal(kbc->semaphore);
		kthread_stop(kbc->task);
	}
	if (kbc->ddkHandle) {
		NvDdkKbcStop(kbc->ddkHandle);
		NvDdkKbcClose(kbc->ddkHandle);
	}

	if (kbc->semaphore)
		NvOsSemaphoreDestroy(kbc->semaphore);

	if (kbc->input_dev)
		input_free_device(kbc->input_dev);

	kfree(kbc);
}

static int __init tegra_kbc_probe(struct platform_device *pdev)
{
	struct tegra_kbc_driver_data *kbc = NULL;
	struct input_dev *input_dev = NULL;
	int err;
	NvError nverr;
	NvU32 l_idx;
	NvU32 k_idx;
	NvU32 TotalKey;

	kbc = kzalloc(sizeof(struct tegra_kbc_driver_data), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (input_dev == NULL || kbc == NULL) {
		input_free_device(input_dev);
		kfree(kbc);
		err = -ENOMEM;
		pr_err("tegra_kbc_probe: Failed to allocate input device\n");
		return err;
	}
	nverr = NvOsSemaphoreCreate(&kbc->semaphore, 0);
	if (nverr != NvSuccess)	{
		err = -1;
		pr_err("tegra_kbc_probe: Semaphore creation failed\n");
		goto fail;
	}

	nverr = NvDdkKbcOpen (s_hRmGlobal, &kbc->ddkHandle);
	if (nverr != NvSuccess) {
		err = -1;
		pr_err("tegra_kbc_probe: NvDdkKbcOpen failed\n");
		goto fail;
	}

	nverr = NvDdkKbcStart(kbc->ddkHandle, kbc->semaphore);
	if (nverr != NvSuccess) {
		err = -1;
		pr_err("tegra_kbc_probe: NvDdkKbcStart failed\n");
		goto fail;
	}

	kbc->task = kthread_create(tegra_kbc_thread, kbc, "tegra_kbc_thread");
	if(kbc->task == NULL) {
		err = -1;
		goto fail;
	}
	wake_up_process( kbc->task );

	kbc->input_dev = input_dev;
	input_dev->event = tegra_kbc_event;
	input_dev->name = "tegra-kbc";
	__set_bit(EV_KEY, input_dev->evbit);

	key_tab_total = NvOdmKbcKeyMappingGetVirtualKeyMappingList(&key_tab);

	for (l_idx = 0; l_idx < key_tab_total; ++l_idx) {
		TotalKey = table_size(key_tab[l_idx]);
		for (k_idx = 0; k_idx < TotalKey; ++k_idx) {
			__set_bit(key_tab[l_idx]->pVirtualKeyTable[k_idx],
				input_dev->keybit);
		}
	}

	platform_set_drvdata(pdev, kbc);

	err = input_register_device(input_dev);
	if (err) {
		pr_err("tegra_kbc_probe: Unable to register %s input device\n",
			input_dev->name);
		goto fail;
	}

	return 0;

fail:
	tegra_kbc_cleanup(kbc);	  
	return err;
}

static int tegra_kbc_remove(struct platform_device *pdev)
{
	struct tegra_kbc_driver_data *kbc = platform_get_drvdata(pdev);
	tegra_kbc_cleanup(kbc);
	return 0;
}

static int tegra_kbc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_kbc_driver_data *kbc = platform_get_drvdata(pdev);
	NvError e = NvError_Success;

	if (!kbc)
		return -1;

	if (!kbc->ddkHandle) {
		printk("%s: device handle is NULL\n", __func__);
		return -1;
	}

	/* power down hardware */
	e = NvDdkKbcSuspend(kbc->ddkHandle);
	if (e != NvSuccess) {
		printk("%s: hardware power down fail\n", __func__);
		return -1;
	}

	return 0;
}

static int tegra_kbc_resume(struct platform_device *pdev)
{
	struct tegra_kbc_driver_data *kbc = platform_get_drvdata(pdev);
	NvError e = NvError_Success;

	if (!kbc)
		return -1;

	if (!kbc->ddkHandle) {
		printk("%s: device handle is NULL\n", __func__);
		return -1;
	}

	/* power up hardware */
	e = NvDdkKbcResume(kbc->ddkHandle);
	if (e != NvSuccess) {
		printk("%s: hardware power up fail\n", __func__);
		return -1;
	}

	return 0;
}

static struct platform_driver tegra_kbc_driver = {
	.probe		= tegra_kbc_probe,
	.remove		= tegra_kbc_remove,
	.suspend	= tegra_kbc_suspend,
	.resume		= tegra_kbc_resume,
	.driver		= {
		.name	= "tegra_kbc",
	},
};

static int __devinit tegra_kbc_init(void)
{
	return platform_driver_register(&tegra_kbc_driver);
}

static void __exit tegra_kbc_exit(void)
{
	platform_driver_unregister(&tegra_kbc_driver);
}

module_init(tegra_kbc_init);
module_exit(tegra_kbc_exit);

MODULE_DESCRIPTION("Tegra Key board controller driver");

