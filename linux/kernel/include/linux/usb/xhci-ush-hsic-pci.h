#ifndef XHCI_USH_HSIC_PCI_h
#define XHCI_USH_HSIC_PCI_h

#include <linux/usb.h>

#define USH_HSIC_AUX1_GPIO       136
#define USH_HSIC_WAKEUP_GPIO     152
#define HSIC_HUB_RESET_TIME   10
#define HSIC_ENABLE_SIZE      2
#define HSIC_DURATION_SIZE    7
#define HSIC_DELAY_SIZE       8
#define HSIC_USH_PORT         5

#define HSIC_AUTOSUSPEND                     0
#define HSIC_PORT_INACTIVITYDURATION              500
#define HSIC_BUS_INACTIVITYDURATION              500
#define HSIC_REMOTEWAKEUP                       1

#define USH_PCI_ID           0x0F35
#define USH_REENUM_DELAY     600000

struct ush_hsic_priv {
	struct delayed_work  hsic_aux;
	wait_queue_head_t    aux_wq;
	struct mutex         hsic_mutex;
	unsigned             hsic_mutex_init:1;
	unsigned             aux_wq_init:1;
	unsigned             hsic_aux_irq_enable:1;
	unsigned             hsic_wakeup_irq_enable:1;
	unsigned             hsic_aux_finish:1;
	unsigned             hsic_enable_created:1;
	unsigned             hsic_lock_init:1;
	unsigned             hsic_stopped:1;

	unsigned             remoteWakeup_enable;
	unsigned             autosuspend_enable;
	unsigned             aux_gpio;
	unsigned             wakeup_gpio;
	unsigned             port_inactivityDuration;
	unsigned             bus_inactivityDuration;
	unsigned             reenumeration_delay;
	spinlock_t           hsic_lock;
	/* Root hub device */
	struct usb_device           *rh_dev;
	struct usb_device           *modem_dev;
	struct workqueue_struct     *work_queue;
	struct work_struct          wakeup_work;
};

enum {
	PROBE,
	REMOVE
};

void hsic_notify(struct usb_device *udev, unsigned action);

#endif
