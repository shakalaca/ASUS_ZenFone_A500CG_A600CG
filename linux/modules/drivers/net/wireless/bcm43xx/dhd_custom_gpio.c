/*
* Customer code to add GPIO control during WLAN start/stop
* Copyright (C) 1999-2013, Broadcom Corporation
* 
*      Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2 (the "GPL"),
* available at http://www.broadcom.com/licenses/GPLv2.php, with the
* following added to such license:
* 
*      As a special exception, the copyright holders of this software give you
* permission to link this software with independent modules, and to copy and
* distribute the resulting executable under terms of your choice, provided that
* you also meet, for each linked independent module, the terms and conditions of
* the license of that module.  An independent module is a module which is not
* derived from this software.  The special exception does not apply to any
* modifications of the software.
* 
*      Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
*
* $Id: dhd_custom_gpio.c 389250 2013-03-06 02:05:03Z $
*/

#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <bcmutils.h>

#include <dngl_stats.h>
#include <dhd.h>

#include <wlioctl.h>
#include <wl_iw.h>

#include <asm/intel-mid.h>
#include <linux/gpio.h>

#define WL_ERROR(x) printf x
#define WL_TRACE(x)

#ifdef CUSTOMER_HW
extern  void bcm_wlan_power_off(int);
extern  void bcm_wlan_power_on(int);
#endif /* CUSTOMER_HW */
#if defined(CUSTOMER_HW2)


#ifdef CONFIG_WIFI_CONTROL_FUNC
int wifi_set_power(int on, unsigned long msec);
int wifi_get_irq_number(unsigned long *irq_flags_ptr);
int wifi_get_gpioen_number(void);
int wifi_get_mac_addr(unsigned char *buf);
void *wifi_get_country_code(char *ccode);
#else
int wifi_set_power(int on, unsigned long msec) { return -1; }
int wifi_get_irq_number(unsigned long *irq_flags_ptr) { return -1; }
int wifi_get_gpioen_number(void) { return -1; }
int wifi_get_mac_addr(unsigned char *buf) { return -1; }
void *wifi_get_country_code(char *ccode) { return NULL; }
#endif /* CONFIG_WIFI_CONTROL_FUNC */
#endif 

#if defined(OOB_INTR_ONLY)

#if defined(BCMLXSDMMC)
extern int sdioh_mmc_irq(int irq);
#endif /* (BCMLXSDMMC)  */

#if defined(CUSTOMER_HW3)
#include <mach/gpio.h>
#endif

/* Customer specific Host GPIO defintion  */
static int dhd_oob_gpio_num = -1;

module_param(dhd_oob_gpio_num, int, 0644);
MODULE_PARM_DESC(dhd_oob_gpio_num, "DHD oob gpio number");

/* This function will return:
 *  1) return :  Host gpio interrupt number per customer platform
 *  2) irq_flags_ptr : Type of Host interrupt as Level or Edge
 *
 *  NOTE :
 *  Customer should check his platform definitions
 *  and his Host Interrupt spec
 *  to figure out the proper setting for his platform.
 *  Broadcom provides just reference settings as example.
 *
 */
int dhd_customer_oob_irq_map(unsigned long *irq_flags_ptr)
{
	int  host_oob_irq = 0;

#if defined(CUSTOMER_HW2) || defined(CUSTOMER_HW4)
	host_oob_irq = wifi_get_irq_number(irq_flags_ptr);

	if (gpio_request(host_oob_irq, "bcm43xx_irq") < 0) {
		WL_ERROR(("%s: Error on gpio_request bcm43xx_irq: %d\n", __func__, host_oob_irq));
		return 1;
	}
	if (gpio_direction_input(host_oob_irq) < 0) {
		WL_ERROR(("%s: Error on gpio_direction_input\n", __func__));
		return 1;
	}
	if (gpio_set_debounce(host_oob_irq, 0) < 0)
		WL_ERROR(("%s: Error on gpio_set_debounce\n", __func__));

	host_oob_irq = gpio_to_irq(host_oob_irq);

#else
#if defined(CUSTOM_OOB_GPIO_NUM)
	if (dhd_oob_gpio_num < 0) {
		dhd_oob_gpio_num = CUSTOM_OOB_GPIO_NUM;
	}
#endif /* CUSTOMER_OOB_GPIO_NUM */

	if (dhd_oob_gpio_num < 0) {
		WL_ERROR(("%s: ERROR customer specific Host GPIO is NOT defined \n",
		__FUNCTION__));
		return (dhd_oob_gpio_num);
	}

	WL_ERROR(("%s: customer specific Host GPIO number is (%d)\n",
	         __FUNCTION__, dhd_oob_gpio_num));

#if defined CUSTOMER_HW
	host_oob_irq = MSM_GPIO_TO_INT(dhd_oob_gpio_num);
#elif defined CUSTOMER_HW3
	gpio_request(dhd_oob_gpio_num, "oob irq");
	host_oob_irq = gpio_to_irq(dhd_oob_gpio_num);
	gpio_direction_input(dhd_oob_gpio_num);
#endif /* CUSTOMER_HW */
#endif 

	return (host_oob_irq);
}
#endif 

int dhd_customer_oob_irq_unmap(void)
{
#if defined(CUSTOMER_HW2) || defined(CUSTOMER_HW4)
	int  host_oob_irq = 0;
	long unsigned int irq_flags_ptr;

	host_oob_irq = wifi_get_irq_number(&irq_flags_ptr);
	if (host_oob_irq < 0)
		WL_ERROR(("%s: wifi_get_irq_number returned %d\n",
				__func__, host_oob_irq));
	else {
		if (gpio_set_debounce(host_oob_irq, 1) < 0)
			WL_ERROR(("%s: Error on gpio_set_debounce\n",
				__func__));
		gpio_free(host_oob_irq);
	}
#endif

	return 0;
}

/* Customer function to control hw specific wlan gpios */
void
dhd_customer_gpio_wlan_ctrl(int onoff)
{
	switch (onoff) {
		case WLAN_RESET_OFF:
			WL_TRACE(("%s: call customer specific GPIO to insert WLAN RESET\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_off(2);
#endif /* CUSTOMER_HW */
#if defined(CUSTOMER_HW2)
			wifi_set_power(0, 0);
#endif
			WL_ERROR(("=========== WLAN placed in RESET ========\n"));
		break;

		case WLAN_RESET_ON:
			WL_TRACE(("%s: callc customer specific GPIO to remove WLAN RESET\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_on(2);
#endif /* CUSTOMER_HW */
#if defined(CUSTOMER_HW2)
			wifi_set_power(1, 0);
#endif
			WL_ERROR(("=========== WLAN going back to live  ========\n"));
		break;

		case WLAN_POWER_OFF:
			WL_TRACE(("%s: call customer specific GPIO to turn off WL_REG_ON\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_off(1);
#endif /* CUSTOMER_HW */
		break;

		case WLAN_POWER_ON:
			WL_TRACE(("%s: call customer specific GPIO to turn on WL_REG_ON\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_on(1);
			/* Lets customer power to get stable */
			OSL_DELAY(200);
#endif /* CUSTOMER_HW */
		break;
	}
}

#ifdef GET_CUSTOM_MAC_ENABLE
#define MAC_ADDRESS_LEN 12

int wifi_get_mac_addr_intel(unsigned char *buf){
	int ret = 0;
	int i;
	struct file *fp = NULL;
	unsigned char c_mac[MAC_ADDRESS_LEN];
	char fname[]="/config/wifi/mac.txt";

	WL_TRACE(("%s Enter\n", __FUNCTION__));

	fp = dhd_os_open_image(fname);
	if (fp== NULL){
		WL_ERROR(("%s: unable to open %s\n",__FUNCTION__, fname));
		return 1;
	}

	if ( dhd_os_get_image_block(c_mac, MAC_ADDRESS_LEN, fp) != MAC_ADDRESS_LEN ){
		WL_ERROR(("%s: Error on reading mac address from %s \n",__FUNCTION__, fname));
		dhd_os_close_image(fp);
		return 1;
	}
	dhd_os_close_image(fp);

	for (i =0; i< MAC_ADDRESS_LEN ; i+=2){
		c_mac[i] = bcm_isdigit(c_mac[i]) ? c_mac[i]-'0' : bcm_toupper(c_mac[i])-'A'+10;
		c_mac[i+1] = bcm_isdigit(c_mac[i+1]) ? c_mac[i+1]-'0' : bcm_toupper(c_mac[i+1])-'A'+10;

		buf[i/2] = c_mac[i]*16 + c_mac[i+1];
	}

	WL_TRACE(("%s: read from file mac address: %x:%x:%x:%x:%x:%x\n",
			 __FUNCTION__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]));

	return ret;
}


/* Function to get custom MAC address */
int
dhd_custom_get_mac_address(unsigned char *buf)
{
	int ret = 0;

	WL_TRACE(("%s Enter\n", __FUNCTION__));
	if (!buf)
		return -EINVAL;

	/* Customer access to MAC address stored outside of DHD driver */
#if defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
	ret = wifi_get_mac_addr_intel(buf);
#endif

#ifdef EXAMPLE_GET_MAC
	/* EXAMPLE code */
	{
		struct ether_addr ea_example = {{0x00, 0x11, 0x22, 0x33, 0x44, 0xFF}};
		bcopy((char *)&ea_example, buf, sizeof(struct ether_addr));
	}
#endif /* EXAMPLE_GET_MAC */

	return ret;
}
#endif /* GET_CUSTOM_MAC_ENABLE */

/* Customized Locale table : OPTIONAL feature */
const struct cntry_locales_custom translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
#ifdef EXAMPLE_TABLE
	{"",   "XY", 4},  /* Universal if Country code is unknown or empty */
	{"US", "US", 69}, /* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69}, /* input ISO "CA" to : US regrev 69 */
	{"EU", "EU", 5},  /* European union countries to : EU regrev 05 */
	{"AT", "EU", 5},
	{"BE", "EU", 5},
	{"BG", "EU", 5},
	{"CY", "EU", 5},
	{"CZ", "EU", 5},
	{"DK", "EU", 5},
	{"EE", "EU", 5},
	{"FI", "EU", 5},
	{"FR", "EU", 5},
	{"DE", "EU", 5},
	{"GR", "EU", 5},
	{"HU", "EU", 5},
	{"IE", "EU", 5},
	{"IT", "EU", 5},
	{"LV", "EU", 5},
	{"LI", "EU", 5},
	{"LT", "EU", 5},
	{"LU", "EU", 5},
	{"MT", "EU", 5},
	{"NL", "EU", 5},
	{"PL", "EU", 5},
	{"PT", "EU", 5},
	{"RO", "EU", 5},
	{"SK", "EU", 5},
	{"SI", "EU", 5},
	{"ES", "EU", 5},
	{"SE", "EU", 5},
	{"GB", "EU", 5},
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3}, /* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3},
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"TR", "TR", 0},
	{"NO", "NO", 0},
#endif /* EXMAPLE_TABLE */
};


/* Customized Locale convertor
*  input : ISO 3166-1 country abbreviation
*  output: customized cspec
*/
void get_customized_country_code(char *country_iso_code, wl_country_t *cspec)
{
#if defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39))

	struct cntry_locales_custom *cloc_ptr;

	if (!cspec)
		return;

	cloc_ptr = wifi_get_country_code(country_iso_code);
	if (cloc_ptr) {
		strlcpy(cspec->ccode, cloc_ptr->custom_locale, WLC_CNTRY_BUF_SZ);
		cspec->rev = cloc_ptr->custom_locale_rev;
	}
	return;
#else
	int size, i;

	size = ARRAYSIZE(translate_custom_table);

	if (cspec == 0)
		 return;

	if (size == 0)
		 return;

	for (i = 0; i < size; i++) {
		if (strcmp(country_iso_code, translate_custom_table[i].iso_abbrev) == 0) {
			memcpy(cspec->ccode,
				translate_custom_table[i].custom_locale, WLC_CNTRY_BUF_SZ);
			cspec->rev = translate_custom_table[i].custom_locale_rev;
			return;
		}
	}
#ifdef EXAMPLE_TABLE
	/* if no country code matched return first universal code from translate_custom_table */
	memcpy(cspec->ccode, translate_custom_table[0].custom_locale, WLC_CNTRY_BUF_SZ);
	cspec->rev = translate_custom_table[0].custom_locale_rev;
#endif /* EXMAPLE_TABLE */
	return;
#endif /* defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)) */
}
