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

	for (;;)
	{
		/* FIXME should we use a NvOsSemaphoreWaitTimeout instead? */
		NvOsSemaphoreWait(kbc->semaphore);
		do {
			loop = tegra_kbc_handle_keyev(kbc);
		} while (loop);
	}

	return 0;
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
		goto err_semaphore_create_failed;
	}

	kbc->task = kthread_create(tegra_kbc_thread, kbc, "tegra_kbc_thread");
	if(kbc->task == NULL) {
		err = -1;
		goto err_kthread_create_failed;
	}
	wake_up_process( kbc->task );

	nverr = NvDdkKbcOpen (s_hRmGlobal, &kbc->ddkHandle);
	if (nverr != NvSuccess) {
		err = -1;
		pr_err("tegra_kbc_probe: NvDdkKbcOpen failed\n");
		goto err_ddk_open_failed;
	}
	nverr = NvDdkKbcStart(kbc->ddkHandle, kbc->semaphore);
	if (nverr != NvSuccess) {
		err = -1;
		pr_err("tegra_kbc_probe: NvDdkKbcStart failed\n");
		goto err_ddk_start_failed;
	}

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
		goto err_input_register_device_failed;
	}

	return 0;

err_input_register_device_failed:
err_ddk_start_failed:
	NvDdkKbcClose(kbc->ddkHandle);
err_ddk_open_failed:
	/* FIXME How to destroy the thread? Maybe we should use workqueues? */
err_kthread_create_failed:
	NvOsSemaphoreDestroy(kbc->semaphore);
err_semaphore_create_failed:
	kfree(kbc);
	input_free_device(input_dev);
	return err;
}

static int tegra_kbc_remove(struct platform_device *pdev)
{
	struct tegra_kbc_driver_data *kbc = platform_get_drvdata(pdev);

	/* FIXME How to destroy the thread? Maybe we should use workqueues? */
	input_unregister_device(kbc->input_dev);
	/* NvOsSemaphoreDestroy(kbc->semaphore); */
	kfree(kbc);
	return 0;
}

static struct platform_driver tegra_kbc_driver = {
	.probe		= tegra_kbc_probe,
	.remove		= tegra_kbc_remove,
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

