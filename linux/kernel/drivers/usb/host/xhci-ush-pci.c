/*
 * Intel MID Platform BayTrail XHCI Controller PCI Bus Glue.
 *
 * Copyright (c) 2013, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License 2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/pci.h>
#include <linux/module.h>
#include <linux/wakelock.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/xhci-ush-hsic-pci.h>
#include "xhci.h"

static struct pci_dev	*pci_dev;

static int ush_hsic_start_host(struct pci_dev  *pdev);
static int ush_hsic_stop_host(struct pci_dev *pdev);
static int create_device_files();
static void remove_device_files();

static int enabling_disabling;
static int hsic_enable;
static struct ush_hsic_priv hsic;

/* Workaround for OSPM, set PMCMD to ask SCU
 * power gate EHCI controller and DPHY
 */
static void hsic_enter_exit_d3(int enter_exit)
{
	if (enter_exit) {
		printk(KERN_CRIT "HSIC Enter D0I3!\n");
		pci_set_power_state(pci_dev, PCI_D3hot);
	} else {
		printk(KERN_CRIT "HSIC Exit D0I3!\n");
		pci_set_power_state(pci_dev, PCI_D0);
	}
}

/* HSIC AUX GPIO irq handler */
static irqreturn_t hsic_aux_gpio_irq(int irq, void *data)
{
	struct device *dev = data;

	dev_dbg(dev,
		"%s---> hsic aux gpio request irq: %d\n",
		__func__, irq);

	if (hsic.hsic_aux_irq_enable == 0) {
		dev_dbg(dev,
			"%s---->AUX IRQ is disabled\n", __func__);
		return IRQ_HANDLED;
	}

	if (delayed_work_pending(&hsic.hsic_aux)) {
		dev_dbg(dev,
			"%s---->Delayed work pending\n", __func__);
		return IRQ_HANDLED;
	}

	if (hsic.modem_dev == NULL) {
		dev_dbg(dev,
			"%s---->No enumeration ignore aux\n", __func__);
		return IRQ_HANDLED;
	}

	hsic.hsic_aux_finish = 0;
	schedule_delayed_work(&hsic.hsic_aux, 0);
	dev_dbg(dev,
		"%s<----\n", __func__);

	return IRQ_HANDLED;
}

/* HSIC Wakeup GPIO irq handler */
static irqreturn_t hsic_wakeup_gpio_irq(int irq, void *data)
{
	struct device *dev = data;

	dev_dbg(dev,
		"%s---> hsic wakeup gpio request irq: %d\n",
		__func__, irq);
	if (hsic.hsic_wakeup_irq_enable == 0) {
		dev_dbg(dev,
			"%s---->Wakeup IRQ is disabled\n", __func__);
		return IRQ_HANDLED;
	}

	queue_work(hsic.work_queue, &hsic.wakeup_work);
	dev_dbg(dev,
		"%s<----\n", __func__);

	return IRQ_HANDLED;
}

static int hsic_aux_irq_init(void)
{
	int retval;

	dev_dbg(&pci_dev->dev,
		"%s---->\n", __func__);
	if (hsic.hsic_aux_irq_enable) {
		dev_dbg(&pci_dev->dev,
			"%s<----AUX IRQ is enabled\n", __func__);
		return 0;
	}
	hsic.hsic_aux_irq_enable = 1;
	gpio_direction_input(hsic.aux_gpio);
	retval = request_irq(gpio_to_irq(hsic.aux_gpio),
			hsic_aux_gpio_irq,
			IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING,
			"hsic_disconnect_request", &pci_dev->dev);
	if (retval) {
		dev_err(&pci_dev->dev,
			"unable to request irq %i, err: %d\n",
			gpio_to_irq(hsic.aux_gpio), retval);
		goto err;
	}

	lnw_gpio_set_alt(hsic.aux_gpio, 0);
	dev_dbg(&pci_dev->dev,
		"%s<----\n", __func__);
	return retval;

err:
	hsic.hsic_aux_irq_enable = 0;
	free_irq(gpio_to_irq(hsic.aux_gpio), &pci_dev->dev);
	return retval;
}

static int hsic_wakeup_irq_init(void)
{
	int retval;

	dev_dbg(&pci_dev->dev,
		"%s---->\n", __func__);
	if (hsic.hsic_wakeup_irq_enable) {
		dev_dbg(&pci_dev->dev,
			"%s<----Wakeup IRQ is enabled\n", __func__);
		return 0;
	}
	hsic.hsic_wakeup_irq_enable = 1;
	gpio_direction_input(hsic.wakeup_gpio);
	retval = request_irq(gpio_to_irq(hsic.wakeup_gpio),
			hsic_wakeup_gpio_irq,
			IRQF_SHARED | IRQF_TRIGGER_RISING,
			"hsic_remote_wakeup_request", &pci_dev->dev);
	if (retval) {
		dev_err(&pci_dev->dev,
			"unable to request irq %i, err: %d\n",
			gpio_to_irq(hsic.wakeup_gpio), retval);
		goto err;
	}

	lnw_gpio_set_alt(hsic.wakeup_gpio, 0);
	dev_dbg(&pci_dev->dev,
		"%s<----\n", __func__);

	return retval;

err:
	hsic.hsic_wakeup_irq_enable = 0;
	free_irq(gpio_to_irq(hsic.wakeup_gpio), &pci_dev->dev);
	return retval;
}

/* Init HSIC AUX GPIO */
static int hsic_aux_gpio_init(void)
{
	int		retval = 0;

	dev_dbg(&pci_dev->dev,
		"%s---->\n", __func__);
	hsic.aux_gpio = USH_HSIC_AUX1_GPIO;
	if (gpio_is_valid(hsic.aux_gpio)) {
		retval = gpio_request(hsic.aux_gpio, "hsic_aux");
		if (retval < 0) {
			dev_err(&pci_dev->dev,
				"Request GPIO %d with error %d\n",
				hsic.aux_gpio, retval);
			retval = -ENODEV;
			goto err1;
		}
	} else {
		retval = -ENODEV;
		goto err1;
	}

	pr_debug("%s----> Enable AUX irq\n", __func__);
	retval = hsic_aux_irq_init();
	if (retval) {
		dev_err(&pci_dev->dev,
			"unable to request IRQ\n");
		goto err2;
	}

	dev_dbg(&pci_dev->dev,
		"%s<----\n", __func__);
	return retval;

err2:
	gpio_free(hsic.aux_gpio);
err1:
	return retval;
}

/* Init HSIC AUX2 GPIO as side band remote wakeup source */
static int hsic_wakeup_gpio_init(void)
{
	int		retval = 0;

	dev_dbg(&pci_dev->dev,
		"%s---->\n", __func__);
	hsic.wakeup_gpio = USH_HSIC_WAKEUP_GPIO;
	if (gpio_is_valid(hsic.wakeup_gpio)) {
		retval = gpio_request(hsic.wakeup_gpio, "hsic_wakeup");
		if (retval < 0) {
			dev_err(&pci_dev->dev,
				"Request GPIO %d with error %d\n",
				hsic.wakeup_gpio, retval);
			retval = -ENODEV;
			goto err;
		}
	} else {
		retval = -ENODEV;
		goto err;
	}

	gpio_direction_input(hsic.wakeup_gpio);
	dev_dbg(&pci_dev->dev,
		"%s<----\n", __func__);
	return retval;

err:
	gpio_free(hsic.wakeup_gpio);
	return retval;
}

static void hsic_aux_irq_free(void)
{
	dev_dbg(&pci_dev->dev,
		"%s---->\n", __func__);
	if (hsic.hsic_aux_irq_enable) {
		hsic.hsic_aux_irq_enable = 0;
		free_irq(gpio_to_irq(hsic.aux_gpio), &pci_dev->dev);
	}
	dev_dbg(&pci_dev->dev,
		"%s<----\n", __func__);
	return;
}

static void hsic_wakeup_irq_free(void)
{
	dev_dbg(&pci_dev->dev,
		"%s---->\n", __func__);
	if (hsic.hsic_wakeup_irq_enable) {
		hsic.hsic_wakeup_irq_enable = 0;
		free_irq(gpio_to_irq(hsic.wakeup_gpio), &pci_dev->dev);
	}
	dev_dbg(&pci_dev->dev,
		"%s<----\n", __func__);
	return;
}

/* the root hub will call this callback when device added/removed */
void hsic_notify(struct usb_device *udev, unsigned action)
{
	int retval;
	struct pci_dev *pdev = to_pci_dev(udev->bus->controller);

	printk(KERN_ERR "pdev device ID: %d, portnum: %d",
			pdev->device, udev->portnum);
	/* Ignore and only valid for HSIC. Filter out
	 * the USB devices added by other USB2 host driver */
	if (pdev->device != USH_PCI_ID)
		return;

	/* Ignore USB devices on external hub */
	if (udev->parent && udev->parent->parent)
		return;

	switch (action) {
	case USB_DEVICE_ADD:
		pr_debug("Notify HSIC add device\n");
		/* Root hub */
		if (!udev->parent) {
			if (udev->speed == USB_SPEED_HIGH) {
				pr_debug("%s rh device set\n", __func__);
				hsic.rh_dev = udev;
				pr_debug("%s Disable autosuspend\n", __func__);
				pm_runtime_set_autosuspend_delay(&udev->dev,
					hsic.bus_inactivityDuration);
				usb_disable_autosuspend(udev);
			}
		} else {
			if (udev->portnum != HSIC_USH_PORT) {
				pr_debug("%s ignore ush ports except port5\n",
						__func__);
				pr_debug("%s ush ports %d\n", __func__,
						udev->portnum);
				break;
			}

			/* Modem devices */
			hsic.modem_dev = udev;
			pm_runtime_set_autosuspend_delay
				(&udev->dev, hsic.port_inactivityDuration);

			if (hsic.remoteWakeup_enable) {
				pr_debug("%s Modem dev remote wakeup enabled\n",
						 __func__);
				device_set_wakeup_capable
					(&hsic.modem_dev->dev, 1);
				device_set_wakeup_capable
					(&hsic.rh_dev->dev, 1);
			} else {
				pr_debug("%s Modem dev remote wakeup disabled\n",
						 __func__);
				device_set_wakeup_capable
					(&hsic.modem_dev->dev, 0);
				device_set_wakeup_capable
					(&hsic.rh_dev->dev, 0);
			}

			usb_disable_autosuspend(hsic.modem_dev);
			usb_disable_autosuspend(hsic.rh_dev);
#if 0
			if (hsic.autosuspend_enable) {
				pr_debug("%s----> enable autosuspend\n",
					 __func__);
				usb_enable_autosuspend(udev->parent);
				hsic_wakeup_irq_init();
			}

			if (hsic.autosuspend_enable == 0) {
				pr_debug("%s Modem dev autosuspend disable\n",
						 __func__);
				usb_disable_autosuspend(hsic.modem_dev);
			}
#endif
		}
		break;
	case USB_DEVICE_REMOVE:
		pr_debug("Notify HSIC delete device\n");
		/* Root hub */
		if (udev->speed != USB_SPEED_HIGH) {
			pr_debug("%s ignore ss port\n", __func__);
			break;
		}
		if (!udev->parent) {
			pr_debug("%s rh_dev deleted\n", __func__);
			hsic.rh_dev = NULL;
		} else {
			/* Modem devices */
			pr_debug("%s----> modem dev deleted\n", __func__);
			hsic.modem_dev = NULL;
			usb_disable_autosuspend(hsic.rh_dev);
		}
		break;
	case MODEM_WORK_FLUSH:
		if (udev == hsic.modem_dev) {
			pr_debug("Notify MODEM work flush\n");
			synchronize_irq(gpio_to_irq(hsic.aux_gpio));
			flush_work(&hsic.hsic_aux);
		}
		break;
	default:
		pr_debug("Notify action not supported\n");
		break ;
	}
	return;
}

static int clear_port_feature(struct usb_device *hdev, int port1, int feature)
{
	return usb_control_msg(hdev, usb_sndctrlpipe(hdev, 0),
		USB_REQ_CLEAR_FEATURE, USB_RT_PORT, feature,
		port1, NULL, 0, 1000);
}

static int set_port_feature(struct usb_device *hdev, int port1, int feature)
{
	return usb_control_msg(hdev, usb_sndctrlpipe(hdev, 0),
		USB_REQ_SET_FEATURE, USB_RT_PORT, feature,
		port1, NULL, 0, 1000);
}

static void ush_hsic_port_disable(struct pci_dev *pdev)
{
	printk(KERN_ERR "%s---->\n", __func__);
#if 1
	if (hsic.modem_dev) {
		struct usb_device *hdev;

		dev_dbg(&pci_dev->dev,
			"%s----> disconnect modem\n", __func__);
		hdev = hsic.modem_dev->parent;
		usb_disable_autosuspend(hsic.modem_dev);
		usb_disable_autosuspend(hsic.rh_dev);
		if (hdev->children[HSIC_USH_PORT - 1] == hsic.modem_dev) {
			printk(KERN_ERR "%s----> usb disconnect\n", __func__);
			usb_disconnect(&hsic.modem_dev);
			hdev->children[HSIC_USH_PORT - 1] = NULL;
		}
	}
#endif
	if (hsic.rh_dev) {
		dev_dbg(&pci_dev->dev,
			"%s----> disable port\n", __func__);
		printk(KERN_ERR "%s----> disable PP\n", __func__);
		clear_port_feature(hsic.rh_dev, HSIC_USH_PORT,
				USB_PORT_FEAT_POWER);
	}
	hsic.hsic_stopped = 1;
	hsic_enable = 0;
}

static void ush_hsic_port_enable(struct pci_dev *pdev)
{
	printk(KERN_ERR "%s---->\n", __func__);
	if (hsic.rh_dev) {
		dev_dbg(&pci_dev->dev,
			"%s----> enable port\n", __func__);
		printk(KERN_ERR "%s----> Enable PP\n", __func__);
		set_port_feature(hsic.rh_dev, HSIC_USH_PORT,
				USB_PORT_FEAT_POWER);
	}
	hsic.hsic_stopped = 0;
	hsic_enable = 1;
}

#ifdef START_STOP_HOST
static void hsic_aux_work(struct work_struct *work)
{
	dev_dbg(&pci_dev->dev,
		"%s---->\n", __func__);
	mutex_lock(&hsic.hsic_mutex);
#if 0
	/* Free the aux irq */
	hsic_aux_irq_free();
	msleep(800);
	dev_dbg(&pci_dev->dev,
		"%s---->AUX IRQ is disabled\n", __func__);
#endif

	if (hsic.hsic_stopped == 0)
		ush_hsic_stop_host(pci_dev);

	usleep_range(hsic.reenumeration_delay,
			hsic.reenumeration_delay + 1000);
	ush_hsic_start_host(pci_dev);
	mutex_unlock(&hsic.hsic_mutex);

	hsic.hsic_aux_finish = 1;
	wake_up(&hsic.aux_wq);
	dev_dbg(&pci_dev->dev,
		"%s<----\n", __func__);
	return;
}
#else
static void hsic_aux_work(struct work_struct *work)
{
	dev_dbg(&pci_dev->dev,
		"%s---->\n", __func__);
	mutex_lock(&hsic.hsic_mutex);
#if 0
	/* Free the aux irq */
	hsic_aux_irq_free();
	msleep(800);
	dev_dbg(&pci_dev->dev,
		"%s---->AUX IRQ is disabled\n", __func__);
#endif

	if (hsic.hsic_stopped == 0)
		ush_hsic_port_disable(pci_dev);

	usleep_range(hsic.reenumeration_delay,
			hsic.reenumeration_delay + 1000);
	ush_hsic_port_enable(pci_dev);
	mutex_unlock(&hsic.hsic_mutex);

	hsic.hsic_aux_finish = 1;
	wake_up(&hsic.aux_wq);
	dev_dbg(&pci_dev->dev,
		"%s<----\n", __func__);
	return;
}
#endif

static void wakeup_work(struct work_struct *work)
{
	dev_dbg(&pci_dev->dev,
		"%s---->\n", __func__);
	if (hsic.modem_dev == NULL) {
		dev_dbg(&pci_dev->dev,
			"%s---->Modem not created\n", __func__);
		return -ENODEV;
	}

	mutex_lock(&hsic.hsic_mutex);
	/* Free the wakeup irq */
	dev_dbg(&pci_dev->dev,
		"%s---->Wakeup IRQ is disabled\n", __func__);
	pm_runtime_get_sync(&hsic.modem_dev->dev);
	usleep_range(5000, 6000);
	pm_runtime_put_sync(&hsic.modem_dev->dev);
	mutex_unlock(&hsic.hsic_mutex);

	dev_dbg(&pci_dev->dev,
		"%s<----\n", __func__);
	return;
}

static ssize_t
show_registers(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_hcd	*hcd = dev_get_drvdata(dev);
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	char			*next;
	unsigned		size;
	unsigned		t;

	next = buf;
	size = PAGE_SIZE;

	pm_runtime_get_sync(dev);
	usleep_range(1000, 1100);

	t = scnprintf(next, size,
		"\n"
		"USBCMD = 0x%08x\n"
		"USBSTS = 0x%08x\n"
		"PORTSC1 = 0x%08x\n"
		"PORTSC2 = 0x%08x\n"
		"PORTSC3 = 0x%08x\n"
		"PORTSC4 = 0x%08x\n"
		"PORTSC5 = 0x%08x\n"
		"PORTSC6 = 0x%08x\n"
		"PORTPMSC1 = 0x%08x\n"
		"PORTPMSC2 = 0x%08x\n"
		"PORTPMSC3 = 0x%08x\n"
		"PORTPMSC4 = 0x%08x\n"
		"PORTPMSC5 = 0x%08x\n"
		"PORTPMSC6 = 0x%08x\n"
		"PORTLI1 = 0x%08x\n"
		"PORTLI2 = 0x%08x\n"
		"PORTLI3 = 0x%08x\n"
		"PORTLI4 = 0x%08x\n"
		"PORTLI5 = 0x%08x\n"
		"PORTLI6 = 0x%08x\n",
		xhci_readl(xhci, &xhci->op_regs->command),
		xhci_readl(xhci, &xhci->op_regs->status),
		xhci_readl(xhci, &xhci->op_regs->port_status_base),
		xhci_readl(xhci, &xhci->op_regs->port_status_base + 4),
		xhci_readl(xhci, &xhci->op_regs->port_status_base + 8),
		xhci_readl(xhci, &xhci->op_regs->port_status_base + 12),
		xhci_readl(xhci, &xhci->op_regs->port_status_base + 16),
		xhci_readl(xhci, &xhci->op_regs->port_status_base + 20),
		xhci_readl(xhci, &xhci->op_regs->port_power_base),
		xhci_readl(xhci, &xhci->op_regs->port_power_base + 4),
		xhci_readl(xhci, &xhci->op_regs->port_power_base + 8),
		xhci_readl(xhci, &xhci->op_regs->port_power_base + 12),
		xhci_readl(xhci, &xhci->op_regs->port_power_base + 16),
		xhci_readl(xhci, &xhci->op_regs->port_power_base),
		xhci_readl(xhci, &xhci->op_regs->port_link_base + 4),
		xhci_readl(xhci, &xhci->op_regs->port_link_base + 8),
		xhci_readl(xhci, &xhci->op_regs->port_link_base + 12),
		xhci_readl(xhci, &xhci->op_regs->port_link_base + 16),
		xhci_readl(xhci, &xhci->op_regs->port_link_base + 20)
		);

	pm_runtime_put_sync(dev);
	usleep_range(1000, 1100);

	size -= t;
	next += t;

	return PAGE_SIZE - size;
}

static DEVICE_ATTR(registers, S_IRUGO, show_registers, NULL);

static ssize_t hsic_reenumeration_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", hsic.reenumeration_delay);
}

static ssize_t hsic_reenumeration_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;
	unsigned delay;

	if (size > HSIC_DELAY_SIZE) {
		dev_dbg(dev, "Invalid, size = %d\n", size);
		return -EINVAL;
	}

	if (sscanf(buf, "%d", &delay) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	mutex_lock(&hsic.hsic_mutex);
	hsic.reenumeration_delay = delay;
	dev_dbg(dev, "reenumeration delay: %d\n",
		hsic.reenumeration_delay);
	mutex_unlock(&hsic.hsic_mutex);
	return size;
}

static DEVICE_ATTR(reenumeration_delay, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		hsic_reenumeration_delay_show,
		 hsic_reenumeration_delay_store);


/* Interfaces for host resume */
static ssize_t hsic_host_resume_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	dev_dbg(dev, "wakeup hsic\n");
	queue_work(hsic.work_queue, &hsic.wakeup_work);

	return -EINVAL;
}

static DEVICE_ATTR(host_resume, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		NULL, hsic_host_resume_store);

static ssize_t hsic_port_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", hsic_enable);
}

#ifdef START_STOP_HOST
static ssize_t hsic_port_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;
	int org_req;

	if (size > HSIC_ENABLE_SIZE)
		return -EINVAL;

	if (sscanf(buf, "%d", &org_req) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

#if 0
	/* Free the aux irq */
	hsic_aux_irq_free();
	dev_dbg(dev,
		"%s---->AUX IRQ is disabled\n", __func__);
#endif

	if (delayed_work_pending(&hsic.hsic_aux)) {
		dev_dbg(dev,
			"%s---->Wait for delayed work finish\n",
			 __func__);
		retval = wait_event_interruptible(hsic.aux_wq,
						hsic.hsic_aux_finish);
		if (retval < 0)
			return retval;

		if (org_req)
			return size;
	}

	mutex_lock(&hsic.hsic_mutex);
	if (org_req) {
		dev_dbg(dev, "enable hsic\n");

		/* add this due to hcd release
			 doesn't set hcd to NULL */
		if (hsic.hsic_stopped == 0)
			ush_hsic_stop_host(pci_dev);
		usleep_range(5000, 6000);
		ush_hsic_start_host(pci_dev);
	} else {
		dev_dbg(dev, "disable hsic\n");
		/* add this due to hcd release
			 doesn't set hcd to NULL */
		if (hsic.hsic_stopped == 0)
			ush_hsic_stop_host(pci_dev);

	}
	mutex_unlock(&hsic.hsic_mutex);
	return size;
}
#else
static ssize_t hsic_port_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;
	int org_req;

	if (size > HSIC_ENABLE_SIZE)
		return -EINVAL;

	if (sscanf(buf, "%d", &org_req) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

#if 0
	/* Free the aux irq */
	hsic_aux_irq_free();
	dev_dbg(dev,
		"%s---->AUX IRQ is disabled\n", __func__);
#endif

	if (delayed_work_pending(&hsic.hsic_aux)) {
		dev_dbg(dev,
			"%s---->Wait for delayed work finish\n",
			 __func__);
		retval = wait_event_interruptible(hsic.aux_wq,
						hsic.hsic_aux_finish);
		if (retval < 0)
			return retval;

		if (org_req)
			return size;
	}

	mutex_lock(&hsic.hsic_mutex);
	if (org_req) {
		dev_dbg(dev, "enable hsic\n");

		/* add this due to hcd release
			 doesn't set hcd to NULL */
		if (hsic.hsic_stopped == 0)
			ush_hsic_port_disable(pci_dev);
		usleep_range(5000, 6000);
		ush_hsic_port_enable(pci_dev);
	} else {
		dev_dbg(dev, "disable hsic\n");
		/* add this due to hcd release
			 doesn't set hcd to NULL */
		if (hsic.hsic_stopped == 0)
			ush_hsic_port_disable(pci_dev);

	}
	mutex_unlock(&hsic.hsic_mutex);
	return size;
}
#endif

static DEVICE_ATTR(hsic_enable, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		hsic_port_enable_show, hsic_port_enable_store);

static ssize_t hsic_port_inactivityDuration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", hsic.port_inactivityDuration);
}

static ssize_t hsic_port_inactivityDuration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;
	unsigned duration;

	if (size > HSIC_DURATION_SIZE) {
		dev_dbg(dev, "Invalid, size = %d\n", size);
		return -EINVAL;
	}

	if (sscanf(buf, "%d", &duration) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	mutex_lock(&hsic.hsic_mutex);
	hsic.port_inactivityDuration = duration;
	dev_dbg(dev, "port Duration: %d\n",
		hsic.port_inactivityDuration);
	if (hsic.modem_dev != NULL) {
		pm_runtime_set_autosuspend_delay
		(&hsic.modem_dev->dev, hsic.port_inactivityDuration);
	}

	mutex_unlock(&hsic.hsic_mutex);
	return size;
}

static DEVICE_ATTR(L2_inactivityDuration, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		hsic_port_inactivityDuration_show,
		 hsic_port_inactivityDuration_store);

/* Interfaces for L2 suspend */
static ssize_t hsic_autosuspend_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", hsic.autosuspend_enable);
}

static ssize_t hsic_autosuspend_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;
	int org_req;

	if (size > HSIC_ENABLE_SIZE) {
		dev_dbg(dev, "Invalid, size = %d\n", size);
		return -EINVAL;
	}

	if (sscanf(buf, "%d", &org_req) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	mutex_lock(&hsic.hsic_mutex);
	hsic.autosuspend_enable = org_req;
	if (hsic.modem_dev != NULL) {
		if (hsic.autosuspend_enable == 0) {
			dev_dbg(dev, "Modem dev autosuspend disable\n");
			usb_disable_autosuspend(hsic.modem_dev);
		} else {
			dev_dbg(dev, "Enable auto suspend\n");
			usb_enable_autosuspend(hsic.modem_dev);
			hsic_wakeup_irq_init();
		}
	}
	if (hsic.rh_dev != NULL) {
		if (hsic.autosuspend_enable == 0) {
			dev_dbg(dev, "port autosuspend disable\n");
			usb_disable_autosuspend(hsic.rh_dev);
		} else {
			dev_dbg(dev, "port Enable auto suspend\n");
			usb_enable_autosuspend(hsic.rh_dev);
		}
	}

	mutex_unlock(&hsic.hsic_mutex);
	return size;
}

static DEVICE_ATTR(L2_autosuspend_enable, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		hsic_autosuspend_enable_show,
		 hsic_autosuspend_enable_store);

static ssize_t hsic_bus_inactivityDuration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", hsic.bus_inactivityDuration);
}

static ssize_t hsic_bus_inactivityDuration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;
	unsigned duration;

	if (size > HSIC_DURATION_SIZE) {
		dev_dbg(dev, "Invalid, size = %d\n", size);
		return -EINVAL;
	}

	if (sscanf(buf, "%d", &duration) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	mutex_lock(&hsic.hsic_mutex);
	hsic.bus_inactivityDuration = duration;
	dev_dbg(dev, "bus Duration: %d\n",
		hsic.bus_inactivityDuration);
	if (hsic.rh_dev != NULL)
		pm_runtime_set_autosuspend_delay
			(&hsic.rh_dev->dev, hsic.bus_inactivityDuration);

	mutex_unlock(&hsic.hsic_mutex);
	return size;
}

static DEVICE_ATTR(bus_inactivityDuration,
		S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		hsic_bus_inactivityDuration_show,
		 hsic_bus_inactivityDuration_store);

static ssize_t hsic_remoteWakeup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", hsic.remoteWakeup_enable);
}

static ssize_t hsic_remoteWakeup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;
	int org_req;

	if (size > HSIC_ENABLE_SIZE) {
		dev_dbg(dev, "Invalid, size = %d\n", size);
		return -EINVAL;
	}

	if (sscanf(buf, "%d", &org_req) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	mutex_lock(&hsic.hsic_mutex);
	hsic.remoteWakeup_enable = org_req;

	if ((hsic.modem_dev != NULL) &&
		(hsic.rh_dev != NULL)) {
		if (hsic.remoteWakeup_enable) {
			dev_dbg(dev, "Modem dev remote wakeup enabled\n");
			device_set_wakeup_capable(&hsic.modem_dev->dev, 1);
			device_set_wakeup_capable(&hsic.rh_dev->dev, 1);
		} else {
			dev_dbg(dev, "Modem dev remote wakeup disabled\n");
			device_set_wakeup_capable(&hsic.modem_dev->dev, 0);
			device_set_wakeup_capable(&hsic.rh_dev->dev, 0);
		}
		pm_runtime_get_sync(&hsic.modem_dev->dev);
		pm_runtime_put_sync(&hsic.modem_dev->dev);
	}

	mutex_unlock(&hsic.hsic_mutex);
	return size;
}

static DEVICE_ATTR(remoteWakeup, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		hsic_remoteWakeup_show, hsic_remoteWakeup_store);

static int create_device_files()
{
	int retval;

	retval = device_create_file(&pci_dev->dev,
			&dev_attr_registers);
	if (retval < 0) {
		dev_dbg(&pci_dev->dev,
			"error create registers\n");
		goto dump_registers;
	}

	hsic.reenumeration_delay = USH_REENUM_DELAY;
	retval = device_create_file(&pci_dev->dev,
			&dev_attr_reenumeration_delay);
	if (retval < 0) {
		dev_dbg(&pci_dev->dev,
			"error create reenumeration delay\n");
		goto reenumeration_delay;
	}

	retval = device_create_file(&pci_dev->dev, &dev_attr_hsic_enable);
	if (retval < 0) {
		dev_dbg(&pci_dev->dev, "error create hsic_enable\n");
		goto hsic_enable;
	}

	retval = device_create_file(&pci_dev->dev, &dev_attr_host_resume);
	if (retval < 0) {
		dev_dbg(&pci_dev->dev, "error create host_resume\n");
		goto host_resume;
	}

	hsic.autosuspend_enable = HSIC_AUTOSUSPEND;
	retval = device_create_file(&pci_dev->dev,
			 &dev_attr_L2_autosuspend_enable);
	if (retval < 0) {
		dev_dbg(&pci_dev->dev, "Error create autosuspend_enable\n");
		goto autosuspend;
	}

	hsic.port_inactivityDuration = HSIC_PORT_INACTIVITYDURATION;
	retval = device_create_file(&pci_dev->dev,
			 &dev_attr_L2_inactivityDuration);
	if (retval < 0) {
		dev_dbg(&pci_dev->dev, "Error create port_inactiveDuration\n");
		goto port_duration;
	}

	hsic.bus_inactivityDuration = HSIC_BUS_INACTIVITYDURATION;
	retval = device_create_file(&pci_dev->dev,
			 &dev_attr_bus_inactivityDuration);
	if (retval < 0) {
		dev_dbg(&pci_dev->dev, "Error create bus_inactiveDuration\n");
		goto bus_duration;
	}

	hsic.remoteWakeup_enable = HSIC_REMOTEWAKEUP;
	retval = device_create_file(&pci_dev->dev, &dev_attr_remoteWakeup);
	if (retval == 0)
		return retval;

	dev_dbg(&pci_dev->dev, "Error create remoteWakeup\n");

	device_remove_file(&pci_dev->dev, &dev_attr_bus_inactivityDuration);
bus_duration:
	device_remove_file(&pci_dev->dev, &dev_attr_L2_inactivityDuration);
port_duration:
	device_remove_file(&pci_dev->dev, &dev_attr_L2_autosuspend_enable);
autosuspend:
	device_remove_file(&pci_dev->dev, &dev_attr_host_resume);
host_resume:
	device_remove_file(&pci_dev->dev, &dev_attr_hsic_enable);
hsic_enable:
	device_remove_file(&pci_dev->dev, &dev_attr_reenumeration_delay);
reenumeration_delay:
	device_remove_file(&pci_dev->dev, &dev_attr_registers);
dump_registers:
	return retval;
}

static void remove_device_files()
{
	device_remove_file(&pci_dev->dev, &dev_attr_L2_autosuspend_enable);
	device_remove_file(&pci_dev->dev, &dev_attr_L2_inactivityDuration);
	device_remove_file(&pci_dev->dev, &dev_attr_bus_inactivityDuration);
	device_remove_file(&pci_dev->dev, &dev_attr_remoteWakeup);
	device_remove_file(&pci_dev->dev, &dev_attr_host_resume);
	device_remove_file(&pci_dev->dev, &dev_attr_hsic_enable);
	device_remove_file(&pci_dev->dev, &dev_attr_reenumeration_delay);
	device_remove_file(&pci_dev->dev, &dev_attr_registers);
}

/*
 * We need to register our own PCI probe function (instead of the USB core's
 * function) in order to create a second roothub under xHCI.
 */
static int xhci_ush_pci_probe(struct pci_dev *dev,
		const struct pci_device_id *id)
{
	int retval;
	struct xhci_hcd *xhci;
	struct hc_driver *driver;
	struct usb_hcd *hcd;

	driver = (struct hc_driver *)id->driver_data;
	pci_dev = dev;

	/* AUX GPIO init */
	retval = hsic_aux_gpio_init();
	if (retval < 0) {
		dev_err(&dev->dev, "AUX GPIO init fail\n");
		retval = -ENODEV;
	}

	/* AUX GPIO init */
	retval = hsic_wakeup_gpio_init();
	if (retval < 0) {
		dev_err(&dev->dev, "Wakeup GPIO init fail\n");
		retval = -ENODEV;
	}

	/* Register the USB 2.0 roothub.
	 * FIXME: USB core must know to register the USB 2.0 roothub first.
	 * This is sort of silly, because we could just set the HCD driver flags
	 * to say USB 2.0, but I'm not sure what the implications would be in
	 * the other parts of the HCD code.
	 */
	retval = usb_hcd_pci_probe(dev, id);
	if (retval)
		return retval;

	/* USB 2.0 roothub is stored in the PCI device now. */
	hcd = dev_get_drvdata(&dev->dev);
	xhci = hcd_to_xhci(hcd);
	xhci->shared_hcd = usb_create_shared_hcd(driver, &dev->dev,
				pci_name(dev), hcd);
	if (!xhci->shared_hcd) {
		retval = -ENOMEM;
		goto dealloc_usb2_hcd;
	}

	/* Set the xHCI pointer before xhci_pci_setup() (aka hcd_driver.reset)
	 * is called by usb_add_hcd().
	 */
	*((struct xhci_hcd **) xhci->shared_hcd->hcd_priv) = xhci;

	if (hsic.hsic_enable_created == 0) {
		retval = create_device_files();
		if (retval < 0) {
			dev_dbg(&dev->dev, "error create device files\n");
			goto dealloc_usb2_hcd;
		}

		hsic.hsic_enable_created = 1;
	}

	if (hsic.hsic_mutex_init == 0) {
		mutex_init(&hsic.hsic_mutex);
		hsic.hsic_mutex_init = 1;
	}

	if (hsic.aux_wq_init == 0) {
		init_waitqueue_head(&hsic.aux_wq);
		hsic.aux_wq_init = 1;
	}

	hsic.work_queue = create_singlethread_workqueue("hsic");
	INIT_WORK(&hsic.wakeup_work, wakeup_work);
	INIT_DELAYED_WORK(&(hsic.hsic_aux), hsic_aux_work);

	retval = usb_add_hcd(xhci->shared_hcd, dev->irq,
			IRQF_SHARED);
	if (retval)
		goto put_usb3_hcd;
	/* Roothub already marked as USB 3.0 speed */

	/* Enable Controller wakeup capability */
	device_set_wakeup_enable(&dev->dev, true);

	/* Enable runtime pm ability */
	hcd->rpm_control = 1;
	hcd->rpm_resume = 0;
	pm_runtime_set_active(&dev->dev);

	/* Check here to avoid to call pm_runtime_put_noidle() twice */
	if (!pci_dev_run_wake(dev))
		pm_runtime_put_noidle(&dev->dev);

	pm_runtime_allow(&dev->dev);
	hsic.hsic_stopped = 0;
	hsic_enable = 1;
	return 0;

put_usb3_hcd:
	usb_put_hcd(xhci->shared_hcd);
dealloc_usb2_hcd:
	usb_hcd_pci_remove(dev);
	return retval;
}

static void xhci_ush_pci_remove(struct pci_dev *dev)
{
	struct xhci_hcd *xhci;

	xhci = hcd_to_xhci(pci_get_drvdata(dev));
	if (xhci->shared_hcd) {
		usb_remove_hcd(xhci->shared_hcd);
		usb_put_hcd(xhci->shared_hcd);
	}

	if (!pci_dev_run_wake(dev))
		pm_runtime_get_noresume(&dev->dev);

	pm_runtime_forbid(&dev->dev);

	usb_hcd_pci_remove(dev);

	/* Free the aux irq */
	hsic_aux_irq_free();
	hsic_wakeup_irq_free();
	gpio_free(hsic.aux_gpio);
	gpio_free(hsic.wakeup_gpio);

	hsic.hsic_stopped = 1;
	hsic_enable = 0;

	kfree(xhci);
}

#ifdef CONFIG_PM_SLEEP
static int xhci_ush_hcd_pci_suspend_noirq(struct device *dev)
{
	int	retval;

	dev_dbg(dev, "%s --->\n", __func__);
	mutex_lock(&hsic.hsic_mutex);
	if (hsic.hsic_stopped == 1) {
		dev_dbg(dev, "hsic stopped return\n");
		mutex_unlock(&hsic.hsic_mutex);
		return 0;
	}

	retval = usb_hcd_pci_pm_ops.suspend_noirq(dev);
	mutex_unlock(&hsic.hsic_mutex);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}

static int xhci_ush_hcd_pci_suspend(struct device *dev)
{
	int     retval;

	dev_dbg(dev, "%s --->\n", __func__);
	mutex_lock(&hsic.hsic_mutex);
	if (hsic.hsic_stopped == 1) {
		dev_dbg(dev, "hsic stopped return\n");
		mutex_unlock(&hsic.hsic_mutex);
		return 0;
	}
	mutex_unlock(&hsic.hsic_mutex);

	retval = usb_hcd_pci_pm_ops.suspend(dev);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}


static int xhci_ush_hcd_pci_resume_noirq(struct device *dev)
{
	int                     retval;

	dev_dbg(dev, "%s --->\n", __func__);
	mutex_lock(&hsic.hsic_mutex);
	if (hsic.hsic_stopped == 1) {
		dev_dbg(dev, "hsic stopped return\n");
		mutex_unlock(&hsic.hsic_mutex);
		return 0;
	}

	retval = usb_hcd_pci_pm_ops.resume_noirq(dev);
	mutex_unlock(&hsic.hsic_mutex);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}


static int xhci_ush_hcd_pci_resume(struct device *dev)
{
	int     retval;

	dev_dbg(dev, "%s --->\n", __func__);
	mutex_lock(&hsic.hsic_mutex);
	if (hsic.hsic_stopped == 1) {
		dev_dbg(dev, "hsic stopped return\n");
		mutex_unlock(&hsic.hsic_mutex);
		return 0;
	}
	mutex_unlock(&hsic.hsic_mutex);

	retval = usb_hcd_pci_pm_ops.resume(dev);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}

#else
#define xhci_ush_hcd_pci_suspend_noirq   NULL
#define xhci_ush_hcd_pci_suspend         NULL
#define xhci_ush_hcd_pci_resume_noirq    NULL
#define xhci_ush_hcd_pci_resume          NULL
#endif


#ifdef CONFIG_PM_RUNTIME
static int xhci_ush_hcd_pci_runtime_suspend(struct device *dev)
{
	int     retval;

	dev_dbg(dev, "%s --->\n", __func__);
	if (hsic.hsic_stopped == 1) {
		dev_dbg(dev, "hsic stopped return\n");
		return 0;
	}

	retval = usb_hcd_pci_pm_ops.runtime_suspend(dev);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}

static int xhci_ush_hcd_pci_runtime_resume(struct device *dev)
{
	struct pci_dev          *pci_dev = to_pci_dev(dev);
	struct usb_hcd          *hcd = pci_get_drvdata(pci_dev);
	int                     retval;

	dev_dbg(dev, "%s --->\n", __func__);
	if (hsic.hsic_stopped == 1) {
		dev_dbg(dev, "hsic stopped return\n");
		return 0;
	}

	retval = usb_hcd_pci_pm_ops.runtime_resume(dev);
	if (hcd->rpm_control) {
		if (hcd->rpm_resume) {
			struct device           *rpm_dev = hcd->self.controller;
			hcd->rpm_resume = 0;
			pm_runtime_put(rpm_dev);
		}
	}
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}
#else
#define xhci_ush_hcd_pci_runtime_suspend NULL
#define xhci_ush_hcd_pci_runtime_resume  NULL
#endif


/* called after powerup, by probe or system-pm "wakeup" */
static int xhci_pci_reinit(struct xhci_hcd *xhci, struct pci_dev *pdev)
{
	/*
	 * TODO: Implement finding debug ports later.
	 * TODO: see if there are any quirks that need to be added to handle
	 * new extended capabilities.
	 */

	/* PCI Memory-Write-Invalidate cycle support is optional (uncommon) */
	if (!pci_set_mwi(pdev))
		xhci_dbg(xhci, "MWI active\n");

	xhci_dbg(xhci, "Finished xhci_pci_reinit\n");
	return 0;
}


void xhci_ush_pci_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	return;
}
/* called during probe() after chip reset completes */
static int xhci_ush_pci_setup(struct usb_hcd *hcd)
{
	struct xhci_hcd         *xhci;
	struct pci_dev          *pdev = to_pci_dev(hcd->self.controller);
	int                     retval;

	retval = xhci_gen_setup(hcd, xhci_ush_pci_quirks);
	if (retval)
		return retval;

	xhci = hcd_to_xhci(hcd);
	if (!usb_hcd_is_primary_hcd(hcd))
		return 0;

	pci_read_config_byte(pdev, XHCI_SBRN_OFFSET, &xhci->sbrn);
	xhci_dbg(xhci, "Got SBRN %u\n", (unsigned int) xhci->sbrn);

	/* Find any debug ports */
	retval = xhci_pci_reinit(xhci, pdev);
	if (!retval)
		return retval;

	kfree(xhci);
	return retval;
}


#ifdef CONFIG_PM
static int xhci_ush_pci_suspend(struct usb_hcd *hcd, bool do_wakeup)
{
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	int     retval = 0;

	if (hcd->state != HC_STATE_SUSPENDED ||
		xhci->shared_hcd->state != HC_STATE_SUSPENDED)
		return -EINVAL;

	retval = xhci_suspend(xhci);

	return retval;
}

static int xhci_ush_pci_resume(struct usb_hcd *hcd, bool hibernated)
{
	struct xhci_hcd         *xhci = hcd_to_xhci(hcd);
	struct pci_dev          *pdev = to_pci_dev(hcd->self.controller);
	int                     retval = 0;

	/* The BIOS on systems with the Intel Panther Point chipset may or may
	 * not support xHCI natively.  That means that during system resume, it
	 * may switch the ports back to EHCI so that users can use their
	 * keyboard to select a kernel from GRUB after resume from hibernate.
	 *
	 * The BIOS is supposed to remember whether the OS had xHCI ports
	 * enabled before resume, and switch the ports back to xHCI when the
	 * BIOS/OS semaphore is written, but we all know we can't trust BIOS
	 * writers.
	 *
	 * Unconditionally switch the ports back to xHCI after a system resume.
	 * We can't tell whether the EHCI or xHCI controller will be resumed
	 * first, so we have to do the port switchover in both drivers.  Writing
	 * a '1' to the port switchover registers should have no effect if the
	 * port was already switched over.
	 */
	if (usb_is_intel_switchable_xhci(pdev))
		usb_enable_xhci_ports(pdev);

	retval = xhci_resume(xhci, hibernated);
	return retval;
}
#endif

static const struct hc_driver xhci_ush_pci_hc_driver = {
	.description =          "Baytrail-USH",
	.product_desc =         "xHCI Host Controller",
	.hcd_priv_size =        sizeof(struct xhci_hcd *),

	/*
	* generic hardware linkag
	 */
	.irq =                  xhci_irq,
	.flags =                HCD_MEMORY | HCD_USB3 | HCD_SHARED,

	/*
	 * basic lifecycle operations
	 */
	.reset =                xhci_ush_pci_setup,
	.start =                xhci_run,
#ifdef CONFIG_PM
	.pci_suspend =          xhci_ush_pci_suspend,
	.pci_resume =           xhci_ush_pci_resume,
#endif
	.stop =                 xhci_stop,
	.shutdown =             xhci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =          xhci_urb_enqueue,
	.urb_dequeue =          xhci_urb_dequeue,
	.alloc_dev =            xhci_alloc_dev,
	.free_dev =             xhci_free_dev,
	.alloc_streams =        xhci_alloc_streams,
	.free_streams =         xhci_free_streams,
	.add_endpoint =         xhci_add_endpoint,
	.drop_endpoint =        xhci_drop_endpoint,
	.endpoint_reset =       xhci_endpoint_reset,
	.check_bandwidth =      xhci_check_bandwidth,
	.reset_bandwidth =      xhci_reset_bandwidth,
	.address_device =       xhci_address_device,
	.update_hub_device =    xhci_update_hub_device,
	.reset_device =         xhci_discover_or_reset_device,

	/*
	 * scheduling support
	 */
	.get_frame_number =     xhci_get_frame,

	/* Root hub support */
	.hub_control =          xhci_hub_control,
	.hub_status_data =      xhci_hub_status_data,
	.bus_suspend =          xhci_bus_suspend,
	.bus_resume =           xhci_bus_resume,
	/*
	 * call back when device connected and addressed
	 */
	.update_device =        xhci_update_device,
	.set_usb2_hw_lpm =      xhci_set_usb2_hardware_lpm,
};

static DEFINE_PCI_DEVICE_TABLE(xhci_ush_pci_ids) = {
	{
		.vendor =       PCI_VENDOR_ID_INTEL,
		.device =       0x0F35,
		.subvendor =    PCI_ANY_ID,
		.subdevice =    PCI_ANY_ID,
		.driver_data =  (unsigned long) &xhci_ush_pci_hc_driver,
	},
	{ /* end: all zeroes */ }
};

static const struct dev_pm_ops xhci_ush_pm_ops = {
	.suspend        = xhci_ush_hcd_pci_suspend,
	.suspend_noirq  = xhci_ush_hcd_pci_suspend_noirq,
	.resume         = xhci_ush_hcd_pci_resume,
	.resume_noirq   = xhci_ush_hcd_pci_resume_noirq,
	.runtime_suspend = xhci_ush_hcd_pci_runtime_suspend,
	.runtime_resume = xhci_ush_hcd_pci_runtime_resume,
};

static struct pci_driver xhci_ush_driver = {
	.name = "BYT-USH",
	.id_table =     xhci_ush_pci_ids,

	.probe =        xhci_ush_pci_probe,
	.remove =       xhci_ush_pci_remove,

#ifdef CONFIG_PM_SLEEP
	.driver =       {
		.pm = &xhci_ush_pm_ops
	},
#endif
	.shutdown =     usb_hcd_pci_shutdown,
};

static int ush_hsic_start_host(struct pci_dev  *pdev)
{
	int		retval;

	pm_runtime_get_sync(&pdev->dev);
	enabling_disabling = 1;
	retval = xhci_ush_pci_probe(pdev, xhci_ush_driver.id_table);
	if (retval)
		dev_dbg(&pdev->dev, "Failed to start host\n");
	enabling_disabling = 0;
	pm_runtime_put(&pdev->dev);

	return retval;
}

static int ush_hsic_stop_host(struct pci_dev *pdev)
{
	pm_runtime_get_sync(&pdev->dev);
	enabling_disabling = 1;
	xhci_ush_pci_remove(pdev);
	enabling_disabling = 0;
	pm_runtime_put(&pdev->dev);

	return 0;
}

int xhci_register_ush_pci(void)
{
	return pci_register_driver(&xhci_ush_driver);
}

void xhci_unregister_ush_pci(void)
{
	return pci_unregister_driver(&xhci_ush_driver);
}
