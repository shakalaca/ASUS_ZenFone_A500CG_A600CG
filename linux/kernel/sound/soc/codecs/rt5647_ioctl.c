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

#define DEBUG 1
#include <linux/spi/spi.h>
#include <sound/soc.h>
#include "rt_codec_ioctl.h"
#include "rt5647_ioctl.h"
#include "rt5647.h"
#include <linux/HWVersion.h>

extern int Read_PROJ_ID(void);

hweq_t hweq_param[] = {
	{/* NORMAL */
		{
			{0, 0},
		},
		0x0000,
	},
	{/* CLUB */
		{
			{0xd0, 0x0635},
			{0xd1, 0x0cd9},
			{0xd2, 0xf405},
			{0xd3, 0x0635},
			{0xd4, 0x0cd9},
			{0xd5, 0xf405},
		},
		0x0006,
	},
	{/* SPK */
		{
			{0xae, 0xc4ef},
			{0xaf, 0x1c13},
			{0xb0, 0xf805},
			{0xc4, 0x1f68},
			{0xc5, 0x0094},
			{0xc6, 0x1f69},
			{0xb1, 0xc4ef},
			{0xb2, 0x1c13},
			{0xb3, 0xf805},
			{0xc7, 0x1f68},
			{0xc8, 0x0094},
			{0xc9, 0x1f69},
		},
		0x0044,
	},
	{/* HP */
		{
			{0, 0},
		},
		0x0000,
	},
};
#define RT5647_HWEQ_LEN ARRAY_SIZE(hweq_param)

/* Realtek:  0xae,   0xaf,   0xb0,   0xc4,   0xc5,   0xc6 (from eqreg[EQ_CH_NUM][EQ_REG_NUM]) */
/* Realtek:  0xb1,   0xb2,   0xb3,   0xc7,   0xc8,   0xc9 */
hweq_t hweq_param_SPK_A500CG = {
	{
		{0xae, 0xc4ef},
		{0xaf, 0x1c13},
		{0xb0, 0xf805},
		{0xc4, 0x1f68},
		{0xc5, 0x0094},
		{0xc6, 0x1f69},
		{0xb1, 0xc4ef},
		{0xb2, 0x1c13},
		{0xb3, 0xf805},
		{0xc7, 0x1f68},
		{0xc8, 0x0094},
		{0xc9, 0x1f69},
	},
	0x0044,
};
hweq_t hweq_param_SPK_A600CG = {
	{
		{0xae, 0xc501},
		{0xaf, 0x1c7c},
		{0xb0, 0xf483},
		{0xc4, 0x1f68},
		{0xc5, 0x0094},
		{0xc6, 0x1f69},
		{0xb1, 0xc501},
		{0xb2, 0x1c7c},
		{0xb3, 0xf483},
		{0xc7, 0x1f68},
		{0xc8, 0x0094},
		{0xc9, 0x1f69},
	},
	0x0044,
};
hweq_t hweq_param_SPK_A502CG = {
	{
		{0xae, 0xc4ef},
		{0xaf, 0x1c13},
		{0xb0, 0xf805},
		{0xc4, 0x1f68},
		{0xc5, 0x0094},
		{0xc6, 0x1f69},
		{0xb1, 0xc4ef},
		{0xb2, 0x1c13},
		{0xb3, 0xf805},
		{0xc7, 0x1f68},
		{0xc8, 0x0094},
		{0xc9, 0x1f69},
	},
	0x0040,
};

int rt5647_update_eqmode(
	struct snd_soc_codec *codec, int channel, int mode)
{
	struct rt_codec_ops *ioctl_ops = rt_codec_get_ioctl_ops();
	int i, upd_reg, reg, mask;

	if (codec == NULL ||  mode >= RT5647_HWEQ_LEN)
		return -EINVAL;

	dev_dbg(codec->dev, "%s(): mode=%d\n", __func__, mode);
	if (mode != NORMAL) {

		for (i = 0; i < EQ_REG_NUM; i++) {
			if (hweq_param[mode].par[i].reg) {
				if (mode == SPK && (Read_PROJ_ID() == PROJ_ID_A500CG || Read_PROJ_ID() == PROJ_ID_A501CG_BZ || Read_PROJ_ID() == PROJ_ID_A501CG
						|| Read_PROJ_ID() == PROJ_ID_A500CG_ID || Read_PROJ_ID() == PROJ_ID_A501CG_ID)) {
					ioctl_ops->index_write(codec, hweq_param_SPK_A500CG.par[i].reg, hweq_param_SPK_A500CG.par[i].val);

				} else if (mode == SPK && (Read_PROJ_ID() == PROJ_ID_A600CG || Read_PROJ_ID() == PROJ_ID_A601CG)) {
					ioctl_ops->index_write(codec, hweq_param_SPK_A600CG.par[i].reg, hweq_param_SPK_A600CG.par[i].val);

				} else if (mode == SPK && Read_PROJ_ID() == PROJ_ID_A502CG) {
					ioctl_ops->index_write(codec, hweq_param_SPK_A502CG.par[i].reg, hweq_param_SPK_A502CG.par[i].val);

				} else {
					ioctl_ops->index_write(codec, hweq_param[mode].par[i].reg, hweq_param[mode].par[i].val);

				}
			} else
				break;
		}
	}

	switch (channel) {
	case EQ_CH_DAC:
		reg = RT5647_EQ_CTRL2;
		mask = 0x33fe;
		upd_reg = RT5647_EQ_CTRL1;
		break;
	case EQ_CH_ADC:
		reg = RT5647_ADC_EQ_CTRL2;
		mask = 0x01bf;
		upd_reg = RT5647_ADC_EQ_CTRL1;
		break;
	default:
		pr_err("Invalid EQ channel\n");
		return -EINVAL;
	}

	if (mode == SPK && (Read_PROJ_ID() == PROJ_ID_A500CG || Read_PROJ_ID() == PROJ_ID_A501CG_BZ || Read_PROJ_ID() == PROJ_ID_A501CG
			|| Read_PROJ_ID() == PROJ_ID_A500CG_ID || Read_PROJ_ID() == PROJ_ID_A501CG_ID)) {
		snd_soc_update_bits(codec, reg, mask, hweq_param_SPK_A500CG.ctrl);
		pr_debug("%s: apply A500CG hweq_param ctrl\n", __func__);

	} else if (mode == SPK && (Read_PROJ_ID() == PROJ_ID_A600CG || Read_PROJ_ID() == PROJ_ID_A601CG)) {
		snd_soc_update_bits(codec, reg, mask, hweq_param_SPK_A600CG.ctrl);
		pr_debug("%s: apply A600CG hweq_param ctrl\n", __func__);

	} else if (mode == SPK && Read_PROJ_ID() == PROJ_ID_A502CG) {
		snd_soc_update_bits(codec, reg, mask, hweq_param_SPK_A502CG.ctrl);
		pr_debug("%s: apply A502CG hweq_param ctrl\n", __func__);

	} else {
		snd_soc_update_bits(codec, reg, mask, hweq_param[mode].ctrl);
		pr_debug("%s: apply hweq_param ctrl\n", __func__);
	}

	reg = snd_soc_read(codec, upd_reg);
	snd_soc_write(codec, upd_reg, reg | RT5647_EQ_UPD);
	snd_soc_update_bits(codec, upd_reg, RT5647_EQ_UPD, 0);

	return 0;
}

int rt5647_ioctl_common(struct snd_hwdep *hw, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct snd_soc_codec *codec = hw->private_data;
	struct rt_codec_cmd __user *_rt_codec = (struct rt_codec_cmd *)arg;
	struct rt_codec_cmd rt_codec;
	/* struct rt_codec_ops *ioctl_ops = rt_codec_get_ioctl_ops(); */
	int *buf;
	static int eq_mode[EQ_CH_NUM];

	if (copy_from_user(&rt_codec, _rt_codec, sizeof(rt_codec))) {
		dev_err(codec->dev, "copy_from_user faild\n");
		return -EFAULT;
	}
	dev_dbg(codec->dev, "%s(): rt_codec.number=%d, cmd=%d\n",
			__func__, rt_codec.number, cmd);
	buf = kmalloc(sizeof(*buf) * rt_codec.number, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;
	if (copy_from_user(buf, rt_codec.buf, sizeof(*buf) * rt_codec.number))
		goto err;


	switch (cmd) {
	case RT_SET_CODEC_HWEQ_IOCTL:
		if (eq_mode == *buf)
			break;
		eq_mode[*buf] = *(buf + 1);
		rt5647_update_eqmode(codec, eq_mode[*buf], *buf);
		break;

	case RT_GET_CODEC_ID:
		*buf = snd_soc_read(codec, RT5647_VENDOR_ID2);
		if (copy_to_user(rt_codec.buf, buf, sizeof(*buf) * rt_codec.number))
			goto err;
		break;
	default:
		break;
	}

	kfree(buf);
	return 0;

err:
	kfree(buf);
	return -EFAULT;
}
EXPORT_SYMBOL_GPL(rt5647_ioctl_common);
