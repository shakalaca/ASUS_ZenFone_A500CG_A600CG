/**
 * df_rgx.c - devfreq driver for IMG rgx graphics in Tangier.
 *
 * To-do - before merge:
 * - Ensure this driver can be loaded as a built-in or via insmod in init.rc
 *   (early on).  It hangs now, probably due to accessing frequency register
 *   when device is not yet initialized/powered.
 * - See above.  Ensure that rgx frequency control is not attempted until
 *   device is known to be ready, with power on.
 *
 * To-do:
 * - Ensure that this driver also works as built-in (not just module).
 * - Add and use utilization computation based on performance counters.
 * - Change or remove program symbol names that start with gbp_*
 * - Expose more information and control through sysfs or debugfs.
 * - Polling interval should be 5ms, so use hrtimer and separate work thread
 *   instead of using devfreq polling based on HZ.
 * - When Tangier supports frequency-changed interrupt, use it to update
 *   rgx driver notion of clock frequency.
 *
 * Notes:
 * Not using "opp" - operating power points.
 */

/**
 * Description:
 *  Early devfreq driver for rgx.  Utilization measures and on-demand
 *  frequency control will be added later.  For now, only thermal
 *  conditions and sysfs file inputs are taken into account.
 *
 *  This driver currently only allows frequencies between 200MHz and
 *  533 MHz.
 *
 *  This driver observes the limits set by the values in:
 *
 *      sysfs file                           initial value (KHz)
 *      ---------------------------------    -------------------
 *      /sys/class/devfreq/dfrgx/min_freq    200000
 *      /sys/class/devfreq/dfrgx/max_freq    320000
 *  and provides current frequency from:
 *      /sys/class/devfreq/dfrgx/cur_freq
 *
 *  With current development silicon, instability is a real possibility
 *  at 400 MHz and higher.
 *
 *  While the driver is informed that a thermal condition exists, it
 *  reduces the gpu frequency to 200 MHz.
 *
 *  Temporary:
 *      No use of performance counters.
 *      No utilization computation.
 *      Uses governor "devfreq_powersave", although with throttling if hot.
 *
 *  It would be nice to have more sysfs or debugfs files for testing purposes.
 *
 *  All DEBUG printk messages start with "dfrgx:" for easy searching of
 *  dmesg output.
 *
 *  To test with the module: insmod /lib/modules/dfrgx.ko
 *  To unload module: rmmod dfrgx
 *
 *  See files under /sys/class/devfreq/dfrgx .
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/suspend.h>

#include <linux/thermal.h>
#include <asm/errno.h>

#include <linux/devfreq.h>

#include <governor.h>

#include <ospm/gfx_freq.h>
#include "dev_freq_debug.h"
#include "dev_freq_graphics_pm.h"
#define DFRGX_GLOBAL_ENABLE_DEFAULT 1

#define DF_RGX_NAME_DEV    "dfrgx"
#define DF_RGX_NAME_DRIVER "dfrgxdrv"

#define DFRGX_HEADING DF_RGX_NAME_DEV ": "

/* FIXME - Temporary limits to frequency range, pending further testing. */
#define DF_RGX_FREQ_KHZ_MIN             200000
#define DF_RGX_FREQ_KHZ_MAX             533000

#define DF_RGX_FREQ_KHZ_MIN_INITIAL     DF_RGX_FREQ_KHZ_MIN
#define DF_RGX_FREQ_KHZ_MAX_INITIAL     320000

#define DF_RGX_INITIAL_FREQ_KHZ         320000

#define DF_RGX_THERMAL_LIMITED_FREQ_KHZ 200000

/* DF_RGX_POLLING_INTERVAL_MS - Polling interval in milliseconds.
 * FIXME - Need to have this be 5 ms, but have to workaround HZ tick usage.
 */
#define DF_RGX_POLLING_INTERVAL_MS 50

/**
 * Potential governors:
 *     #define GOVERNOR_TO_USE devfreq_simple_ondemand
 *     #define GOVERNOR_TO_USE devfreq_performance
 *     #define GOVERNOR_TO_USE devfreq_powersave
 */

#define GOVERNOR_TO_USE devfreq_performance


/**
 * THERMAL_COOLING_DEVICE_MAX_STATE - The maximum cooling state that this
 * driver (as a thermal cooling device by reducing frequency) supports.
 */
#define THERMAL_COOLING_DEVICE_MAX_STATE 1


struct busfreq_data {
	struct device        *dev;
	struct devfreq       *devfreq;
	struct notifier_block pm_notifier;
	struct mutex          lock;
	bool                  disabled;
	unsigned long int     bf_freq_mhz_rlzd;

	struct thermal_cooling_device *gbp_cooldv_hdl;
	int                   gbp_cooldv_state_cur;
	int                   gbp_cooldv_state_prev;
	int                   gbp_cooldv_state_highest;
	int                   gbp_cooldv_state_override;
};

/* df_rgx_created_dev - Pointer to created device, if any. */
static struct platform_device *df_rgx_created_dev;

/*Need to check if this is the 1st request*/
static int firstRequest = 1;


/**
 * Module parameters:
 *
 * - can be updated (if permission allows) via writing:
 *     /sys/module/dfrgx/parameters/<name>
 * - can be set at module load time:
 *     insmod /lib/modules/dfrgx.ko enable=0
 * - For built-in drivers, can be on kernel command line:
 *     dfrgx.enable=0
 */

/**
 * module parameter "enable" is not writable in sysfs as there is presently
 * no code to detect the transition between 0 and 1.
 */
static unsigned int mprm_enable = DFRGX_GLOBAL_ENABLE_DEFAULT;
module_param_named(enable, mprm_enable, uint, S_IRUGO);

static unsigned int mprm_verbosity = 2;
module_param_named(verbosity, mprm_verbosity, uint, S_IRUGO|S_IWUSR);


#define DRIVER_AUTHOR "Intel Corporation"
#define DRIVER_DESC "devfreq driver for rgx graphics"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/**
 *  MODULE_VERSION - Allows specification of a module version.
 *  Version of form [<epoch>:]<version>[-<extra-version>].
 *  Or for CVS/RCS ID version, everything but the number is stripped.
 * <epoch>: A (small) unsigned integer which allows you to start versions
 *          anew. If not mentioned, it's zero.  eg. "2:1.0" is after
 *          "1:2.0".
 * <version>: The <version> may contain only alphanumerics and the
 *          character `.'.  Ordered by numeric sort for numeric parts,
 *          ascii sort for ascii parts (as per RPM or DEB algorithm).
 * <extraversion>: Like <version>, but inserted for local
 *          customizations, eg "rh3" or "rusty1".

 * Using this automatically adds a checksum of the .c files and the
 * local headers in "srcversion".
 *
 * Also, if the module is under drivers/staging, this causes a warning to
 * be issued:
 *     <mname>: module is from the staging directory, the quality is unknown,
 *     you have been warned.
 *
 * Example invocation:
 *     MODULE_VERSION("0.1");
 */


/**
 * set_desired_frequency_khz() - Set gpu frequency.
 * @bfdata: Pointer to private data structure
 * @freq_khz: Desired frequency in KHz (not MHz).
 * Returns: <0 if error, 0 if success, but no frequency update, or
 * realized frequency in KHz.
 */
static long set_desired_frequency_khz(struct busfreq_data *bfdata,
	unsigned long freq_khz)
{
	int sts;
	struct devfreq *df;
	unsigned long freq_req;
	unsigned long freq_limited;
	unsigned long freq_mhz;
	unsigned int freq_mhz_quantized;
	u32 freq_code;
	int thermal_state;

	sts = 0;

	/* Warning - this function may be called from devfreq_add_device,
	 * but if it is, bfdata->devfreq will not yet be set.
	 */
	df = bfdata->devfreq;

	if (!df) {
	    /*
	     * Initial call, so set initial frequency.	Limits from min_freq
	     * and max_freq would not have been applied by caller.
	     */
	    freq_req = DF_RGX_INITIAL_FREQ_KHZ;
	}
	else if ((freq_khz == 0) && df->previous_freq)
		freq_req = df->previous_freq;
	else
		freq_req = freq_khz;

	DFRGX_DPF(DFRGX_DEBUG_HIGH, "%s: entry, caller requesting %luKHz\n",
		__func__, freq_khz);

	if (freq_req < DF_RGX_FREQ_KHZ_MIN)
		freq_limited = DF_RGX_FREQ_KHZ_MIN;
	else if (freq_req > DF_RGX_FREQ_KHZ_MAX)
		freq_limited = DF_RGX_FREQ_KHZ_MAX;
	else
		freq_limited = freq_req;

	if (bfdata->gbp_cooldv_state_override >= 0)
		thermal_state = bfdata->gbp_cooldv_state_override;
	else
		thermal_state = bfdata->gbp_cooldv_state_cur;

	if (thermal_state != 0) {
		if (freq_limited > DF_RGX_THERMAL_LIMITED_FREQ_KHZ)
			freq_limited = DF_RGX_THERMAL_LIMITED_FREQ_KHZ;
	}

	if (df && (freq_limited == df->previous_freq))
		return df->previous_freq;

	freq_mhz = freq_limited / 1000;

	mutex_lock(&bfdata->lock);

	if (bfdata->disabled)
		goto out;

	freq_code = gpu_freq_mhz_to_code(freq_mhz, &freq_mhz_quantized);

	if (bfdata->bf_freq_mhz_rlzd != freq_mhz_quantized) {
		sts = gpu_freq_set_from_code(freq_code);
		if (sts < 0) {
			DFRGX_DPF(DFRGX_DEBUG_MED,
				"%s: error (%d) from gpu_freq_set_from_code for %uMHz\n",
				__func__, sts, freq_mhz_quantized);
			goto out;
		} else {
			bfdata->bf_freq_mhz_rlzd = sts;
			DFRGX_DPF(DFRGX_DEBUG_HIGH, "%s: freq MHz(requested, realized) = %u, %lu\n",
				__func__, freq_mhz_quantized,
				bfdata->bf_freq_mhz_rlzd);
		}

		if (df) {
			/*
			 * Setting df->previous_freq will be redundant
			 * when called from target dispatch function, but
			 * not otherwise.
			 */
			df->previous_freq = bfdata->bf_freq_mhz_rlzd * 1000;
		}
	}

	sts = bfdata->bf_freq_mhz_rlzd * 1000;

out:
	mutex_unlock(&bfdata->lock);

	return sts;
}


/**
 * df_rgx_bus_target - Request setting of a new frequency.
 * @*p_freq: Input: desired frequency in KHz, output: realized freq in KHz.
 * @flags: DEVFREQ_FLAG_* - not used by this implementation.
 */
static int df_rgx_bus_target(struct device *dev, unsigned long *p_freq,
			      u32 flags)
{
	struct platform_device *pdev;
	struct busfreq_data *bfdata;
	int ret = 0;
	(void) flags;

	pdev = container_of(dev, struct platform_device, dev);
	bfdata = platform_get_drvdata(pdev);

	/*FIXME: Need to rethink about this scenario*/
	if(firstRequest){
		*p_freq = DF_RGX_INITIAL_FREQ_KHZ;
		firstRequest = 0;
		goto out;
	}

	if(!df_rgx_is_active()){
		return -EBUSY;
	}

	ret = set_desired_frequency_khz(bfdata, *p_freq);
	if (ret <= 0)
		return ret;

	*p_freq = ret;

out:
	return 0;
}


/**
 * df_rgx_bus_get_dev_status() - Update current status, including:
 * - stat->current_frequency - Frequency in KHz.
 * - stat->total_time
 * - stat->busy_time
 * Note: total_time and busy_time have arbitrary units, as they are
 * used only as ratios.
 * Utilization is busy_time / total_time .
 */
static int df_rgx_bus_get_dev_status(struct device *dev,
				      struct devfreq_dev_status *stat)
{
	struct busfreq_data *bfdata = dev_get_drvdata(dev);

	DFRGX_DPF(DFRGX_DEBUG_LOW, "%s: entry\n", __func__);

	stat->current_frequency = bfdata->bf_freq_mhz_rlzd * 1000;

	/* FIXME - Compute real utilization statistics. */
	stat->total_time = 100;
	stat->busy_time = 50;

	return 0;
}


/**
 * tcd_get_max_state() - thermal cooling device callback get_max_state.
 * @tcd: Thermal cooling device structure.
 * @pms: Pointer to integer through which output value is stored.
 *
 * Invoked via interrupt/callback.
 * Function return value: 0 if success, otherwise -error.
 * Execution context: non-atomic
 */
static int tcd_get_max_state(struct thermal_cooling_device *tcd,
	unsigned long *pms)
{
	*pms = THERMAL_COOLING_DEVICE_MAX_STATE;

	return 0;
}


/**
 * tcd_get_cur_state() - thermal cooling device callback get_cur_state.
 * @tcd: Thermal cooling device structure.
 * @pcs: Pointer to integer through which output value is stored.
 *
 * Invoked via interrupt/callback.
 * Function return value: 0 if success, otherwise -error.
 * Execution context: non-atomic
 */
static int tcd_get_cur_state(struct thermal_cooling_device *tcd,
	unsigned long *pcs)
{
	struct busfreq_data *bfdata = (struct busfreq_data *) tcd->devdata;
	*pcs = bfdata->gbp_cooldv_state_cur;

	return 0;
}


/**
 * tcd_set_cur_state() - thermal cooling device callback set_cur_state.
 * @tcd: Thermal cooling device structure.
 * @cs: Input state.
 *
 * Invoked via interrupt/callback.
 * Function return value: 0 if success, otherwise -error.
 * Execution context: non-atomic
 */
static int tcd_set_cur_state(struct thermal_cooling_device *tcd,
	unsigned long cs)
{
	struct devfreq *df;
	struct busfreq_data *bfdata;

	bfdata = (struct busfreq_data *) tcd->devdata;

	if (cs > THERMAL_COOLING_DEVICE_MAX_STATE)
		cs = THERMAL_COOLING_DEVICE_MAX_STATE;

	/* If state change between zero and non-zero... */
	if (!!bfdata->gbp_cooldv_state_cur != !!cs) {
		bfdata->gbp_cooldv_state_prev = bfdata->gbp_cooldv_state_cur;
		bfdata->gbp_cooldv_state_cur = cs;

		if (bfdata->gbp_cooldv_state_highest <
			bfdata->gbp_cooldv_state_cur) {
			bfdata->gbp_cooldv_state_highest =
				bfdata->gbp_cooldv_state_cur;
		}

		if (mprm_verbosity >= 2)
			DFRGX_DPF(DFRGX_DEBUG_HIGH, "Thermal state changed from %d to %d\n",
				bfdata->gbp_cooldv_state_prev,
				bfdata->gbp_cooldv_state_cur);

		df = bfdata->devfreq;
		if (df) {
			mutex_lock(&df->lock);
			update_devfreq(df);
			mutex_unlock(&df->lock);
		}
	}

	return 0;
}


/**
 * df_rgx_bus_exit() - An optional callback that is called when devfreq is
 * removing the devfreq object due to error or from devfreq_remove_device()
 * call. If the user has registered devfreq->nb at a notifier-head, this is
 * the time to unregister it.
 */
static void df_rgx_bus_exit(struct device *dev)
{
	struct busfreq_data *bfdata = dev_get_drvdata(dev);
	(void) bfdata;

	DFRGX_DPF(DFRGX_DEBUG_LOW, "%s: entry\n", __func__);

	/*  devfreq_unregister_opp_notifier(dev, bfdata->devfreq); */
}


static struct devfreq_dev_profile df_rgx_devfreq_profile = {
	.initial_freq	= DF_RGX_INITIAL_FREQ_KHZ,
	.polling_ms	= DF_RGX_POLLING_INTERVAL_MS,
	.target		= df_rgx_bus_target,
	.get_dev_status	= df_rgx_bus_get_dev_status,
	.exit		= df_rgx_bus_exit,
};


/**
 * busfreq_mon_reset() - Initialize or reset monitoring
 * hardware state as desired.
 */
static void busfreq_mon_reset(struct busfreq_data *bfdata)
{
	/*  FIXME - reset monitoring? */
}


static int df_rgx_busfreq_pm_notifier_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct busfreq_data *bfdata = container_of(this, struct busfreq_data,
						 pm_notifier);
	DFRGX_DPF(DFRGX_DEBUG_LOW, "%s: entry\n", __func__);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		/* Set Fastest and Deactivate DVFS */
		mutex_lock(&bfdata->lock);
		bfdata->disabled = true;
		mutex_unlock(&bfdata->lock);
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		/* Reactivate */
		mutex_lock(&bfdata->lock);
		bfdata->disabled = false;
		mutex_unlock(&bfdata->lock);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static int df_rgx_busfreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct busfreq_data *bfdata;
	struct devfreq *df;
	int sts = 0;

	DFRGX_DPF(DFRGX_DEBUG_LOW, "%s: entry\n", __func__);

	/* dev_err(dev, "example.\n"); */

	bfdata = kzalloc(sizeof(struct busfreq_data), GFP_KERNEL);
	if (bfdata == NULL) {
		dev_err(dev, "Cannot allocate memory.\n");
		return -ENOMEM;
	}

	bfdata->pm_notifier.notifier_call = df_rgx_busfreq_pm_notifier_event;
	bfdata->dev = dev;
	mutex_init(&bfdata->lock);

	platform_set_drvdata(pdev, bfdata);

	busfreq_mon_reset(bfdata);

	df = devfreq_add_device(dev, &df_rgx_devfreq_profile,
					   &GOVERNOR_TO_USE, NULL);

	if (IS_ERR(df)) {
		sts = PTR_ERR(bfdata->devfreq);
		goto err_000;
	}

	bfdata->devfreq = df;

	df->min_freq = DF_RGX_FREQ_KHZ_MIN_INITIAL;
	df->max_freq = DF_RGX_FREQ_KHZ_MAX_INITIAL;

	bfdata->gbp_cooldv_state_override = -1;

	{
		static const char *tcd_type = "gpu_burst";
		static const struct thermal_cooling_device_ops tcd_ops = {
			.get_max_state = tcd_get_max_state,
			.get_cur_state = tcd_get_cur_state,
			.set_cur_state = tcd_set_cur_state,
		};
		struct thermal_cooling_device *tcdhdl;

		/**
		  * Example: Thermal zone "type"s and temps milli-deg-C.
		  * These are just examples and are not specific to our usage.
		  *   type              temp
		  *   --------          -------
		  *   skin0             15944
		  *   skin1             22407
		  *   msicdie           37672
		  *
		  * See /sys/class/thermal/thermal_zone<i>
		  * See /sys/class/thermal/cooling_device<i>
		  */

		tcdhdl = thermal_cooling_device_register(
			(char *) tcd_type, bfdata, &tcd_ops);
		if (IS_ERR(tcdhdl)) {
			DFRGX_DPF(DFRGX_DEBUG_HIGH, "Cooling device registration failed: %ld\n",
				-PTR_ERR(tcdhdl));
			sts = PTR_ERR(tcdhdl);
			goto err_001;
		}
		bfdata->gbp_cooldv_hdl = tcdhdl;
	}

	sts = register_pm_notifier(&bfdata->pm_notifier);
	if (sts) {
		dev_err(dev, "Failed to setup pm notifier\n");
		goto err_002;
	}

	DFRGX_DPF(DFRGX_DEBUG_HIGH, "%s: success\n", __func__);

	return 0;

err_002:
	thermal_cooling_device_unregister(bfdata->gbp_cooldv_hdl);
	bfdata->gbp_cooldv_hdl = NULL;
err_001:
	devfreq_remove_device(bfdata->devfreq);
err_000:
	platform_set_drvdata(pdev, NULL);
	mutex_destroy(&bfdata->lock);
	kfree(bfdata);
	return sts;
}

static int df_rgx_busfreq_remove(struct platform_device *pdev)
{
	struct busfreq_data *bfdata = platform_get_drvdata(pdev);

	unregister_pm_notifier(&bfdata->pm_notifier);
	devfreq_remove_device(bfdata->devfreq);
	mutex_destroy(&bfdata->lock);
	kfree(bfdata);

	return 0;
}

static int df_rgx_busfreq_resume(struct device *dev)
{
	struct busfreq_data *bfdata = dev_get_drvdata(dev);

	DFRGX_DPF(DFRGX_DEBUG_LOW, "%s: entry\n", __func__);

	busfreq_mon_reset(bfdata);
	return 0;
}


static const struct dev_pm_ops df_rgx_busfreq_pm = {
	.resume	= df_rgx_busfreq_resume,
};

static const struct platform_device_id df_rgx_busfreq_id[] = {
	{ DF_RGX_NAME_DEV, 0 },
	{ "", 0 },
};


static struct platform_driver df_rgx_busfreq_driver = {
	.probe	= df_rgx_busfreq_probe,
	.remove	= df_rgx_busfreq_remove,
	.id_table = df_rgx_busfreq_id,
	.driver = {
		.name	= DF_RGX_NAME_DRIVER,
		.owner	= THIS_MODULE,
		.pm	= &df_rgx_busfreq_pm,
	},
};


static struct platform_device * __init df_rgx_busfreq_device_create(void)
{
	struct platform_device *pdev;
	int ret;

	pdev = platform_device_alloc(DF_RGX_NAME_DEV, -1);
	if (!pdev) {
		pr_err("%s: platform_device_alloc failed\n",
			DF_RGX_NAME_DEV);
		return NULL;
	}

	ret = platform_device_add(pdev);
	if (ret < 0) {
		pr_err("%s: platform_device_add failed\n",
			DF_RGX_NAME_DEV);
		platform_device_put(pdev);
		return ERR_PTR(ret);
	}

	return pdev;
}

static int __init df_rgx_busfreq_init(void)
{
	struct platform_device *pdev;
	int ret;

	if (!mprm_enable) {
		DFRGX_DPF(DFRGX_DEBUG_HIGH, "%s: %s: disabled\n",
			DF_RGX_NAME_DRIVER, __func__);
		return -ENODEV;
	}

	gpu_freq_set_suspend_func(&df_rgx_suspend);

	gpu_freq_set_resume_func(&df_rgx_resume);

	DFRGX_DPF(DFRGX_DEBUG_HIGH,"%s: %s: starting\n", DF_RGX_NAME_DRIVER, __func__);

	pdev = df_rgx_busfreq_device_create();
	if (IS_ERR(pdev))
		return PTR_ERR(pdev);
	if (!pdev)
		return -ENOMEM;

	df_rgx_created_dev = pdev;

	ret = platform_driver_register(&df_rgx_busfreq_driver);

	DFRGX_DPF(DFRGX_DEBUG_HIGH, "%s: %s: success\n", DF_RGX_NAME_DRIVER, __func__);

	return ret;
}
late_initcall(df_rgx_busfreq_init);

static void __exit df_rgx_busfreq_exit(void)
{
	struct platform_device *pdev = df_rgx_created_dev;
	struct busfreq_data *bfdata = platform_get_drvdata(pdev);

	DFRGX_DPF(DFRGX_DEBUG_LOW, "%s:\n", __func__);

	if (bfdata && bfdata->gbp_cooldv_hdl) {
		thermal_cooling_device_unregister(bfdata->gbp_cooldv_hdl);
		bfdata->gbp_cooldv_hdl = NULL;
	}

	platform_driver_unregister(&df_rgx_busfreq_driver);

	/* Most state reset is done by function df_rgx_busfreq_remove,
	 * including invocation of:
	 * - unregister_pm_notifier
	 * - devfreq_remove_device
	 * - mutex_destroy(&bfdata->lock);
	 * - kfree(bfdata);
	*/

	if (pdev)
		platform_device_unregister(pdev);
}
module_exit(df_rgx_busfreq_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RGX busfreq driver with devfreq framework");
MODULE_AUTHOR("Dale B Stimson <dale.b.stimson@intel.com>");
