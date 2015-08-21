/*
 * rt5677-hotword.h  --  RT5677 ALSA SoC audio codec driver
 *
 * Copyright 2013 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __RT5677_HOTWORD_H__
#define __RT5677_HOTWORD_H__

#ifdef CONFIG_SND_SOC_RT5677_HOTWORD

int rt5677_hotword_init(struct device *dev);
int rt5677_hotword_free(struct device *dev);

#else /* !CONFIG_SND_SOC_RT5677_HOTWORD */

static inline int rt5677_hotword_init(struct device *dev)
{
	return 0;
}

static inline int rt5677_hotword_free(struct device *dev)
{
	return 0;
}

#endif /* CONFIG_SND_SOC_RT5677_HOTWORD */

#endif /* __RT5677_HOTWORD_H__ */
