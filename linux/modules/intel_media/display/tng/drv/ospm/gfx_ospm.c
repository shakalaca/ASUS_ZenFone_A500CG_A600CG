/**************************************************************************
 * Copyright (c) 2012, Intel Corporation.
 * All Rights Reserved.

 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Hitesh K. Patel <hitesh.k.patel@intel.com>
 */

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <asm/intel-mid.h>

#include "psb_drv.h"
#include "gfx_ospm.h"
#include "gfx_freq.h"
#include "pmu_tng.h"
#include "tng_wa.h"

#define	USE_GFX_PM_FUNC			0

/* WRAPPER Offset 0x160024 */
#define GFX_STATUS_OFFSET		0x24

/* define which island to power down */
#define GFX_ALL		(GFX_SLC_LDO_SSC | \
					GFX_SLC_SSC | \
					GFX_SDKCK_SSC | \
					GFX_RSCD_SSC)

#define GFX_POWER_UP(x) \
	pmu_nc_set_power_state(x, OSPM_ISLAND_UP, GFX_SS_PM0)

#define GFX_POWER_DOWN(x) \
	pmu_nc_set_power_state(x, OSPM_ISLAND_DOWN, GFX_SS_PM0)

static u32 gfx_island_selected = GFX_SLC_LDO_SSC | GFX_SLC_SSC |
	GFX_SDKCK_SSC | GFX_RSCD_SSC;

enum GFX_ISLAND_STATUS {
	POWER_ON = 0,		/* No gating (clk or power) */
	CLOCK_GATED,		/* Clock Gating */
	SOFT_RESET,		/* Soft Reset */
	POWER_OFF,		/* Powered off or Power gated.*/
};

static int (*pSuspend_func)(void) = NULL;
static int (*pResume_func)(void) = NULL;

/**
  * gpu_freq_code_to_mhz() - Given frequency as a code (as defined for *_PM1
  * register), return frequency in mhz.
  * @freq_code_in - Input: A frequency code as specified for *_PM1 registers.
  * Function return value: corresponding frequency in MHz or < 0 if error.
  */
static int gpu_freq_code_to_mhz(int freq_code_in)
{
	int freq_mhz_out;

	switch (freq_code_in) {
	case IP_FREQ_100_00:
		freq_mhz_out = 100;
		break;
	case IP_FREQ_106_67:
		freq_mhz_out = 106;
		break;
	case IP_FREQ_133_30:
		freq_mhz_out = 133;
		break;
	case IP_FREQ_160_00:
		freq_mhz_out = 160;
		break;
	case IP_FREQ_177_78:
		freq_mhz_out = 177;
		break;
	case IP_FREQ_200_00:
		freq_mhz_out = 200;
		break;
	case IP_FREQ_213_33:
		freq_mhz_out = 213;
		break;
	case IP_FREQ_266_67:
		freq_mhz_out = 266;
		break;
	case IP_FREQ_320_00:
		freq_mhz_out = 320;
		break;
	case IP_FREQ_355_56:
		freq_mhz_out = 355;
		break;
	case IP_FREQ_400_00:
		freq_mhz_out = 400;
		break;
	case IP_FREQ_533_33:
		freq_mhz_out = 533;
		break;
	case IP_FREQ_640_00:
		freq_mhz_out = 640;
		break;
	case IP_FREQ_800_00:
		freq_mhz_out = 800;
		break;
	default:
		printk(KERN_ALERT "%s: Invalid freq code: %#x\n", __func__,
			freq_code_in);
		return -EINVAL;
	}

	return freq_mhz_out;
}

/**
 * mrfl_pwr_cmd_gfx - Change graphics power state.
 * Change island power state in the require sequence.
 *
 * @gfx_mask: Mask of islands to be changed.
 * @new_state: 0 for power-off, 1 for power-on.
 */
#ifdef USE_GFX_INTERNAL_PM_FUNC
static int mrfl_pwr_cmd_gfx(u32 gfx_mask, int new_state)
{
	/*
	 * pwrtab - gfx pwr sub-islands in required power-up order and
	 * in reverse of required power-down order.
	 */
	static const u32 pwrtab[] = {
		GFX_SLC_LDO_SHIFT,
		GFX_SLC_SHIFT,
		GFX_SDKCK_SHIFT,
		GFX_RSCD_SHIFT,
	};
	const int pwrtablen = ARRAY_SIZE(pwrtab);
	int i;
	int j;
	int ret;
	u32 ns_mask;
	u32 done_mask;
	u32 this_mask;
	u32 pwr_state_prev;

	pwr_state_prev = intel_mid_msgbus_read32(PUNIT_PORT, GFX_SS_PM0);

	if (new_state == OSPM_ISLAND_UP)
		ns_mask = TNG_COMPOSITE_I0;
	else
		ns_mask = TNG_COMPOSITE_D3;

	/*  Call underlying function separately for each step in the
	    power sequence. */
	done_mask = 0;
	for (i = 0; i < pwrtablen ; i++) {
		if (new_state == OSPM_ISLAND_UP)
			j = i;
		else
			j = pwrtablen - i - 1;

		done_mask |= TNG_SSC_MASK << pwrtab[j];
		this_mask = gfx_mask & done_mask;
		if (this_mask) {
		/*  FIXME - if (new_state == 0), check for required
			    conditions per the SAS. */
			ret = pmu_set_power_state_tng(GFX_SS_PM0,
					this_mask, ns_mask);
			if (ret)
			return ret;
		}

#if A0_WORKAROUNDS
		/**
		 * If turning some power on, and the power to be on includes SLC,
		 * and SLC was not previously on, then setup some registers.
	 */
		if ((new_state == OSPM_ISLAND_UP)
			&& (pwrtab[j] == GFX_SLC_SHIFT)
			&& ((pwr_state_prev >> GFX_SLC_SHIFT) != TNG_SSC_I0))
			apply_A0_workarounds(OSPM_GRAPHICS_ISLAND, 1);
#endif

		if ((gfx_mask & ~done_mask) == 0)
			break;
	}

	return 0;
}
#endif

/**
 * pm_cmd_freq_wait() - Wait for frequency valid via specified register.
 * Optionally, return realized frequency to caller.
 * @reg_freq: The frequency control register.  One of *_PM1.
 * @freq_code_rlzd - If non-NULL, pointer to receive the realized Tangier
 * frequency code.
 */
static int pm_cmd_freq_wait(u32 reg_freq, u32 *freq_code_rlzd)
{
	int tcount;
	u32 freq_val;

	for (tcount = 0; ; tcount++) {
		freq_val = intel_mid_msgbus_read32(PUNIT_PORT, reg_freq);
		if ((freq_val & IP_FREQ_VALID) == 0)
			break;
		if (tcount > 500) {
			WARN(1, "%s: P-Unit freq request wait timeout",
				__func__);
			return -EBUSY;
		}
		udelay(1);
	}

	if (freq_code_rlzd) {
		*freq_code_rlzd = ((freq_val >> IP_FREQ_STAT_POS) &
			IP_FREQ_MASK);
	}

	return 0;
}


/**
 * pm_cmd_freq_set() - Set operating frequency via specified register.
 * Optionally, return realized frequency to caller.
 * @reg_freq: The frequency control register.  One of *_PM1.
 * @freq_code: Tangier frequency code.
 * @p_freq_code_rlzd - If non-NULL, pointer to receive the realized Tangier
 * frequency code.
 */
static int pm_cmd_freq_set(u32 reg_freq, u32 freq_code, u32 *p_freq_code_rlzd)
{
	u32 freq_val;
	u32 freq_code_realized;
	int rva;

	rva = pm_cmd_freq_wait(reg_freq, NULL);
	if (rva < 0) {
		printk(KERN_ALERT "%s: pm_cmd_freq_wait 1 failed\n", __func__);
		return rva;
	}

	freq_val = IP_FREQ_VALID | freq_code;
	intel_mid_msgbus_write32(PUNIT_PORT, reg_freq, freq_val);

	rva = pm_cmd_freq_wait(reg_freq, &freq_code_realized);
	if (rva < 0) {
		printk(KERN_ALERT "%s: pm_cmd_freq_wait 2 failed\n", __func__);
		return rva;
	}

	if (p_freq_code_rlzd)
		*p_freq_code_rlzd = freq_code_realized;

	return rva;
}


/**
 * pm_cmd_freq_from_code() - Set operating frequency via specified register.
 * Optionally, return realized frequency to caller.
 * @reg_freq: The frequency control register.  One of *_PM1.
 * @freq_code: Tangier frequency code.
 * @function return value: - <0 if error, or frequency in MHz.
 */
int gpu_freq_set_from_code(int freq_code)
{
	u32 freq_realized_code;
	int rva;

	rva = pm_cmd_freq_set(GFX_SS_PM1, freq_code, &freq_realized_code);
	if (rva < 0)
		return rva;

	return gpu_freq_code_to_mhz(freq_realized_code);
}
EXPORT_SYMBOL(gpu_freq_set_from_code);


/**
  * gpu_freq_mhz_to_code() - Given frequency in MHz, return frequency code
  * used for frequency control.
  * Always pick the code less than equal to the integer MHz value.
  * @freq_mhz_in - Input: A MHz frequency specification.
  * @*p_freq_out - Out: The quantized MHz frequency specification.
  * Function return value: frequency code as in register definition.
  */
int gpu_freq_mhz_to_code(int freq_mhz_in, int *p_freq_out)
{
	int freq_code;
	int freq_out;

	if (freq_mhz_in >= 800) {
		freq_code = IP_FREQ_800_00;	/* 800.00 */
		freq_out = 800;
	} else if (freq_mhz_in >= 640) {
		freq_code = IP_FREQ_640_00;	/* 640.00 */
		freq_out = 640;
	} else if (freq_mhz_in >= 533) {
		freq_code = IP_FREQ_533_33;	/* 533.33 */
		freq_out = 533;
	} else if (freq_mhz_in >= 400) {
		freq_code = IP_FREQ_400_00;	/* 400.00 */
		freq_out = 400;
	} else if (freq_mhz_in >= 355) {
		freq_code = IP_FREQ_355_56;	/* 355.56 */
		freq_out = 355;
	} else if (freq_mhz_in >= 320) {
		freq_code = IP_FREQ_320_00;	/* 320.00 */
		freq_out = 320;
	} else if (freq_mhz_in >= 266) {
		freq_code = IP_FREQ_266_67;	/* 266.67 */
		freq_out = 266;
	} else if (freq_mhz_in >= 213) {
		freq_code = IP_FREQ_213_33;	/* 213.33 */
		freq_out = 213;
	} else if (freq_mhz_in >= 200) {
		freq_code = IP_FREQ_200_00;	/* 200.00 */
		freq_out = 200;
	} else if (freq_mhz_in >= 177) {
		freq_code = IP_FREQ_177_78;	/* 177.78 */
		freq_out = 177;
	} else if (freq_mhz_in >= 160) {
		freq_code = IP_FREQ_160_00;	/* 160.00 */
		freq_out = 160;
	} else if (freq_mhz_in >= 133) {
		freq_code = IP_FREQ_133_30;	/* 133.30 */
		freq_out = 133;
	} else if (freq_mhz_in >= 106) {
		freq_code = IP_FREQ_106_67;	/* 106.67 */
		freq_out = 106;
	} else {
		freq_code = IP_FREQ_100_00;	/* 100.00 */
		freq_out = 100;
	}

	*p_freq_out = freq_out;

	return freq_code;
}
EXPORT_SYMBOL(gpu_freq_mhz_to_code);

void gpu_freq_set_suspend_func(int (*suspend_func)(void))
{
	pSuspend_func = suspend_func;
	OSPM_DPF("OSPM: suspend \n");
}
EXPORT_SYMBOL(gpu_freq_set_suspend_func);

void gpu_freq_set_resume_func(int (*resume_func)(void))
{
	pResume_func = resume_func;
	OSPM_DPF("OSPM: Resume \n");
}
EXPORT_SYMBOL(gpu_freq_set_resume_func);

/***********************************************************
 * All Graphics Island
 ***********************************************************/
static bool first_boot = true;
/**
 * ospm_gfx_power_up
 *
 * Power up graphics islands
 * Sequence & flow from SAS
 */
static bool ospm_gfx_power_up(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret = true;
	u32 gfx_all = gfx_island_selected;
	int error = 0;

	if(pResume_func){
		error = (*pResume_func)();
		if(error){
			OSPM_DPF("OSPM: Could not resume DFRGX");
			return false;
		}
	}

	OSPM_DPF("Pre-power-up status = 0x%08lX\n",
		intel_mid_msgbus_read32(PUNIT_PORT, NC_PM_SSS));

	if (first_boot) {
		gfx_all = GFX_ALL;
		first_boot = false;
	}

#ifdef USE_GFX_INTERNAL_PM_FUNC
	ret = mrfl_pwr_cmd_gfx(GFX_ALL, OSPM_ISLAND_UP);
#else
	if (gfx_all & GFX_SLC_LDO_SSC)
		ret = GFX_POWER_UP(PMU_LDO);

	if (gfx_all & GFX_SLC_SSC)
		ret = GFX_POWER_UP(PMU_SLC);

	/*
	 * This workarounds are only needed for TNG A0/A1 silicon.
	 * Any TNG SoC which is newer than A0/A1 won't need this.
	 */
	if (!IS_TNG_B0(dev))
	{
		/**
		* If turning some power on, and the power to be on includes SLC,
		* and SLC was not previously on, then setup some registers.
		*/
		if (gfx_all & GFX_SLC_SSC)
			apply_A0_workarounds(OSPM_GRAPHICS_ISLAND, 1);
	}

	if (gfx_all & GFX_SDKCK_SSC)
		ret = GFX_POWER_UP(PMU_SDKCK);

	if (gfx_all & GFX_RSCD_SSC)
		ret = GFX_POWER_UP(PMU_RSCD);
#endif /*USE_GFX_INTERNAL_PM_FUNC*/

	OSPM_DPF("Post-power-up status = 0x%08lX\n",
		intel_mid_msgbus_read32(PUNIT_PORT, NC_PM_SSS));

	return !ret;
}

/**
 * ospm_gfx_power_down
 *
 * Power down Graphics islands
 * Sequence & flow from SAS
 */
static bool ospm_gfx_power_down(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret = true;
	int error = 0;

	OSPM_DPF("OSPM: ospm_gfx_power_down \n");

	if(pSuspend_func){
	error = (*pSuspend_func)();
		if(error){
			OSPM_DPF("OSPM :Could not suspend DFRGX");
			return false;
		}
	}

	OSPM_DPF("Pre-power-off Status = 0x%08lX\n",
		intel_mid_msgbus_read32(PUNIT_PORT, NC_PM_SSS));

	/* power down every thing */
	if (gfx_island_selected & GFX_RSCD_SSC)
		ret = GFX_POWER_DOWN(PMU_RSCD);

	if (gfx_island_selected & GFX_SDKCK_SSC)
		ret = GFX_POWER_DOWN(PMU_SDKCK);

	if (gfx_island_selected & GFX_SLC_SSC)
		ret = GFX_POWER_DOWN(PMU_SLC);

	if (gfx_island_selected & GFX_SLC_LDO_SSC)
		ret = GFX_POWER_DOWN(PMU_LDO);

	OSPM_DPF("Post-power-off Status = 0x%08lX\n",
		intel_mid_msgbus_read32(PUNIT_PORT, NC_PM_SSS));

	return !ret;
}

/**
 * ospm_gfx_init
 *
 * Graphics power island init
 */
void ospm_gfx_init(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	OSPM_DPF("%s\n", __func__);
	p_island->p_funcs->power_up = ospm_gfx_power_up;
	p_island->p_funcs->power_down = ospm_gfx_power_down;
	p_island->p_dependency = NULL;
}
