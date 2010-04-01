/*
 * sound/soc/tegra/tegra_soc_audio.c
 *
 * ALSA SOC driver for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
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
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/clk.h>
#include <asm/mach-types.h>
#include <asm/hardware/scoop.h>
#include <linux/io.h>
#include "nvodm_query_discovery.h"
#include "tegra_transport.h"

struct codec_setup_data {
	unsigned dem0_pin;
	unsigned dem1_pin;
	unsigned pdad_pin;
	unsigned pdda_pin;
};

extern struct snd_soc_codec_device soc_codec_dev_tegra_generic_codec;
extern struct snd_soc_dai tegra_generic_codec_dai;

static struct platform_device *tegra_snd_device;
NvU64 codec_guid;

struct devlite_setup_data {
    int i2c_bus;
    unsigned short i2c_address;
};

#define NVODM_CODEC_MAX_CLOCKS 3

static unsigned int clock_frequencies[NVODM_CODEC_MAX_CLOCKS];

static int set_clock_source_on_codec(NvU64 codec_guid,int IsEnable)
{
	const NvOdmPeripheralConnectivity *p_connectivity = NULL;
	unsigned int clock_instances[NVODM_CODEC_MAX_CLOCKS];
	unsigned int num_clocks;
	p_connectivity = NvOdmPeripheralGetGuid(codec_guid);
	if (p_connectivity == NULL)
		return NV_FALSE;

	if (IsEnable) {
		if (!NvOdmExternalClockConfig(codec_guid, NV_FALSE,
					      clock_instances,
					      clock_frequencies, &num_clocks))
			return NV_FALSE;
	} else {
		if (!NvOdmExternalClockConfig(codec_guid,
					      NV_TRUE,
					      clock_instances,
					      clock_frequencies,
					      &num_clocks));
		return NV_FALSE;
	}
	return NV_TRUE;
}

static int tegra_hifi_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	/* Set codec DAI configuration */
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret;

	ret = snd_soc_dai_set_fmt(codec_dai,SND_SOC_DAIFMT_I2S);

	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops tegra_hifi_ops = {
	.hw_params = tegra_hifi_hw_params,
};

static int tegra_codec_init(struct snd_soc_codec *codec)
{
	return 0;
}

extern struct snd_soc_dai tegra_i2s_rpc_dai;
extern struct snd_soc_platform tegra_soc_platform;

static struct snd_soc_dai_link tegra_board_dai = {
	.name = "tegra-generic-codec",
	.stream_name = "tegra-codec-rpc",
	.cpu_dai = &tegra_i2s_rpc_dai,
	.codec_dai = &tegra_generic_codec_dai,
	.init = tegra_codec_init,
	.ops = &tegra_hifi_ops,
};

static struct snd_soc_card tegra_board = {
	.name = "tegra",
	.platform = &tegra_soc_platform,
	.dai_link = &tegra_board_dai,
	.num_links = 1,
};

static struct snd_soc_device tegra_board_snd_devdata = {
	.card = &tegra_board,
	.codec_dev = &soc_codec_dev_tegra_generic_codec,
};

#define BOARD_ID_E1162  (0x0B3E) /* Decimal  1162. => ((11<<8) | 62)*/
#define BOARD_SKU_5541  (0x3729) /* Decimal  5541. => ((55<<8) | 41) / 512MB @ 400MHz */
#define BOARD_SKU_5641  (0x3829) /* Decimal  5641. => ((56<<8) | 41) / 1GB @ 400MHz */

#define WOLFSON_8753_CODEC_GUID NV_ODM_GUID('w','o','l','f','8','7','5','3')
#define WOLFSON_8903_CODEC_GUID NV_ODM_GUID('w','o','l','f','8','9','0','3')
#define WOLFSON_8903_I2C_1_CODEC_GUID NV_ODM_GUID('w','o','8','9','0','3','_','1')

static int __init tegra_soc_init(void)
{
	int ret;
	int i;
	NvOdmBoardInfo BoardInfo;
	const NvOdmPeripheralConnectivity *pPerConnectivity = NULL;
	struct devlite_setup_data devlite_setup;

	memset(&devlite_setup,0,sizeof(struct devlite_setup_data));


	tegra_snd_device = platform_device_alloc("soc-audio", -1);
	if (!tegra_snd_device)
		return -ENOMEM;

	platform_set_drvdata(tegra_snd_device, &tegra_board_snd_devdata);
	tegra_board_snd_devdata.dev = &tegra_snd_device->dev;

	ret = platform_device_add(tegra_snd_device);
	if (ret) {
		snd_printk(KERN_ERR "tegra audio device could not be added \n");
		platform_device_put(tegra_snd_device);
		return ret;
	}

	if(!strcmp(NvOdmQueryPlatform(),"SO-DIMM") || !strcmp(NvOdmQueryPlatform(),"Harmony") || !strcmp(NvOdmQueryPlatform(),"GrandCanyon"))
	{
		if (NvOdmPeripheralGetBoardInfo((BOARD_ID_E1162), &BoardInfo))
		{
			if ( (BoardInfo.SKU == BOARD_SKU_5541 || BoardInfo.SKU == BOARD_SKU_5641) && (BoardInfo.Fab >= 001) )
			{
				codec_guid = WOLFSON_8903_I2C_1_CODEC_GUID;
				devlite_setup.i2c_bus = 1;
			}
			else
			{
				codec_guid = WOLFSON_8903_CODEC_GUID;
				devlite_setup.i2c_bus = 0;
			}
		}
		else
		{
			codec_guid = WOLFSON_8903_CODEC_GUID;
			devlite_setup.i2c_bus = 0;
		}

		pPerConnectivity = NvOdmPeripheralGetGuid(codec_guid);

		if (pPerConnectivity == NULL)
			return -EFAULT;

		for (i = 0; i < pPerConnectivity->NumAddress; ++i)
		{
			if ((pPerConnectivity->AddressList[i].Interface == NvOdmIoModule_I2c_Pmu) ||
			    (pPerConnectivity->AddressList[i].Interface == NvOdmIoModule_I2c))
			{
				break;
			}
		}

		if (i == pPerConnectivity->NumAddress)
			return -EFAULT;

		devlite_setup.i2c_address = (pPerConnectivity->AddressList[i].Address >> 1);


	}
	else
	{
		// whistler codec guid
		codec_guid = NV_ODM_GUID('w','o','l','f','8','7','5','3');
	}

	set_clock_source_on_codec(codec_guid,NV_TRUE);

	if(!strcmp(NvOdmQueryPlatform(),"SO-DIMM") || !strcmp(NvOdmQueryPlatform(),"Harmony") || !strcmp(NvOdmQueryPlatform(),"GrandCanyon"))
	{
		struct i2c_board_info info;
		struct i2c_adapter *adapter;
		struct i2c_client *client;

		/* Enable clock source on codec. */
		memset(&info, 0, sizeof(struct i2c_board_info));
		info.addr = devlite_setup.i2c_address;
		strlcpy(info.type, "wm8903", I2C_NAME_SIZE);
		adapter = i2c_get_adapter(devlite_setup.i2c_bus);
		if (!adapter) {
			printk(KERN_ERR "can't get i2c adapter %d\n",
			       devlite_setup.i2c_bus);
			ret = -EFAULT;
			goto end;
		}
		client = i2c_new_device(adapter, &info);
		i2c_put_adapter(adapter);
		if (!client) {
			printk(KERN_ERR "can't add i2c device at 0x%x\n",
			       (unsigned int)info.addr);
			ret = -EFAULT;
			goto end;
		}
	}
end:
	if (ret != 0)
		platform_device_unregister(tegra_snd_device);

	return ret;
}

static void __exit tegra_soc_exit(void)
{
	set_clock_source_on_codec(codec_guid,0);
	platform_device_unregister(tegra_snd_device);
}

module_init(tegra_soc_init);
module_exit(tegra_soc_exit);

/* Module information */
MODULE_DESCRIPTION("Tegra SoC Sound");
MODULE_LICENSE("GPL");
