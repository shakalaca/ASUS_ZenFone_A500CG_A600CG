/*
 * rt5647_ioctl.h  --  RT5647 ALSA SoC audio driver IO control
 *
 * Copyright 2012 Realtek Microelectronics
 * Author: Bard <bardliao@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __RT5647_IOCTL_H__
#define __RT5647_IOCTL_H__

#include <sound/hwdep.h>
#include <linux/ioctl.h>

enum {
	NORMAL = 0,
	CLUB,
	SPK,
	HP,
	MODE_NUM,
};

enum {
	EQ_CH_DAC = 0,
	EQ_CH_ADC,
	EQ_CH_NUM,
};

struct rt5647_eq_parameter {
	u8 reg;
	u16 val;
};

#define EQ_REG_NUM 56
typedef struct  hweq_s {
	struct rt5647_eq_parameter par[EQ_REG_NUM];
	unsigned int ctrl;
} hweq_t;

int rt5647_ioctl_common(struct snd_hwdep *hw, struct file *file,
			unsigned int cmd, unsigned long arg);
int rt5647_update_eqmode(
	struct snd_soc_codec *codec, int channel, int mode);

#endif /* __RT5647_IOCTL_H__ */
