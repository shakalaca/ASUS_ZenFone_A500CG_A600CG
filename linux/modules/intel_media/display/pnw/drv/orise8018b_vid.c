/*
 * Copyright (c)  2012 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicensen
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */

#include "displays/orise8018b_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include "psb_drv.h"
#include <linux/lnw_gpio.h>
#include <linux/init.h>
#include <asm/intel_scu_pmic.h>

#include <linux/HWVersion.h>

extern int Read_PROJ_ID(void);
static int board_proj_id=0;

#define ORISE8018B_PANEL_NAME	"OTM8018B"

#define ORISE8018B_DEBUG 1
#define ENABLE_CSC_GAMMA 1
struct delayed_work orise8018b_panel_reset_delay_work;
struct workqueue_struct *orise8018b_panel_reset_delay_wq;

static int orise8018b_vid_drv_ic_reset_workaround(struct mdfld_dsi_config *dsi_config) {

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	mdfld_dsi_send_mcs_short_lp(sender, 0x11, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdelay(10);
	mdfld_dsi_send_mcs_short_lp(sender, 0x29, 0, 0, MDFLD_DSI_SEND_PACKAGE);

	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL) {
		printk("[DISP] %s MDFLD_DSI_CONTROL_ABNORMAL !!\n", __func__);
		return -EIO;
	}

	return 0;
}

static void orise8018b_vid_panel_reset_delay_work(struct work_struct *work)
{
//	printk("[DEBUG] %s\n", __func__);
	orise8018b_vid_drv_ic_reset_workaround(panel_reset_dsi_config);
	queue_delayed_work(orise8018b_panel_reset_delay_wq, &orise8018b_panel_reset_delay_work, msecs_to_jiffies(3000));
}

/*
 * GPIO pin definition
 */
#define PMIC_GPIO_BACKLIGHT_EN	0x7E
#define PMIC_GPIO_VEMMC2CNT	0xDA 	//panel power control 2.8v


#define DISP_RST_N		57


#define PWM_SOC_ENABLE 1
#define PWMCTRL_REG 0xffae9000
#define PWMCTRL_SIZE 0x80
static void __iomem *pwmctrl_mmio;
#define PWM_ENABLE_GPIO 49
//#define PWM_BASE_UNIT 0x444 //5,000Hz
#define PWM_BASE_UNIT 0x1555 //25,000Hz


#if PWM_SOC_ENABLE
union sst_pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

static int pwm_configure(int duty)
{
	union sst_pwmctrl_reg pwmctrl;

	/*Read the PWM register to make sure there is no pending
	*update.
	*/
	pwmctrl.full = readl(pwmctrl_mmio);

	/*check pwnswupdate bit */
	if (pwmctrl.part.pwmswupdate)
		return -EBUSY;
	pwmctrl.part.pwmswupdate = 0x1;
	pwmctrl.part.pwmbu = PWM_BASE_UNIT;
	pwmctrl.part.pwmtd = duty;
	writel(pwmctrl.full,  pwmctrl_mmio);

	return 0;
}

static void pwm_enable(){
	union sst_pwmctrl_reg pwmctrl;

	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	/*Enable the PWM by setting PWM enable bit to 1 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 1;
	writel(pwmctrl.full, pwmctrl_mmio);
}

static void pwm_disable(){
	union sst_pwmctrl_reg pwmctrl;
	/*setting PWM enable bit to 0 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 0;
	writel(pwmctrl.full,  pwmctrl_mmio);

	gpio_set_value(PWM_ENABLE_GPIO, 0);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, 0);
}
#endif

struct orise8018b_vid_data{
	u8 project_id;
	unsigned int gpio_lcd_en;
	unsigned int gpio_bl_en;
	unsigned int gpio_stb1_en;
	unsigned int gpio_stb2_en;
	unsigned int gpio_lcd_rst;
};

struct mipi_dsi_cmd{
	int delay;
	int len;
	u8 *commands;
};

struct mipi_dsi_cmd_orise{
	int gamma_enable;
	int delay;
	int len1;
	u8 *commands1;
	int len2;
	u8 *commands2;
};


static struct orise8018b_vid_data gpio_settings_data;
static struct mdfld_dsi_config *orise8018b_dsi_config;

#define CMD_SIZE(x) (sizeof((x)) / sizeof((x)[0]))


/* ====Initial settings==== */
/* OTM8018B settings */
static u8 cm_000[] = {0x00, 0x00};
static u8 cm_001[] = {0xFF, 0x80, 0x09, 0x01};
static u8 cm_002[] = {0x00, 0x80};
static u8 cm_003[] = {0xFF, 0x80, 0x09};
static u8 cm_004[] = {0x00, 0xB4};
static u8 cm_005[] = {0xC0, 0x55};
static u8 cm_006[] = {0x00, 0x82};
static u8 cm_007[] = {0xC5, 0xA3};
static u8 cm_008[] = {0x00, 0x90};
static u8 cm_009[] = {0xC5, 0x96, 0x79};
static u8 cm_010[] = {0x00, 0x00};
static u8 cm_011[] = {0xD8, 0x77, 0x77};
static u8 cm_012[] = {0x00, 0x81};		//65HZ
static u8 cm_013[] = {0xC1, 0x66};
static u8 cm_014[] = {0x00, 0xA3};
static u8 cm_015[] = {0xC0, 0x1B};
static u8 cm_016[] = {0x00, 0x81};
static u8 cm_017[] = {0xC4, 0x83};
static u8 cm_018[] = {0x00, 0x92};
static u8 cm_019[] = {0xC5, 0x01};
static u8 cm_020[] = {0x00, 0xB1};
static u8 cm_021[] = {0xC5, 0xA9};
static u8 cm_022[] = {0x00, 0xA6};
static u8 cm_023[] = {0xC1, 0x01, 0x00, 0x00};
static u8 cm_024[] = {0x00, 0x80};
static u8 cm_025[] = {0xC4, 0x30};
static u8 cm_026[] = {0x00, 0xC0};
static u8 cm_027[] = {0xC5, 0x00};
static u8 cm_028[] = {0x00, 0x8B};
static u8 cm_029[] = {0xB0, 0x40};
static u8 cm_030[] = {0x00, 0x88};
static u8 cm_031[] = {0xC4, 0x80};
static u8 cm_032[] = {0x00, 0xA0};
static u8 cm_033[] = {0xC1, 0xEA};
static u8 cm_034[] = {0x00, 0xB2};
static u8 cm_035[] = {0xF5, 0x15, 0x00, 0x15, 0x00};
static u8 cm_036[] = {0x00, 0x93};
static u8 cm_037[] = {0xC5, 0x03};
static u8 cm_038[] = {0x00, 0x90};
static u8 cm_039[] = {0xB3, 0x02};
static u8 cm_040[] = {0x00, 0x92};
static u8 cm_041[] = {0xB3, 0x45};
static u8 cm_042[] = {0x00, 0x80};
static u8 cm_043[] = {0xC0, 0x00, 0x5F, 0x00, 0x10, 0x10};
static u8 cm_044[] = {0x00, 0x90};
static u8 cm_045[] = {0xC0, 0x00, 0x44, 0x00, 0x00, 0x00, 0x03};
static u8 cm_046[] = {0x00, 0x80};
static u8 cm_047[] = {0xCE, 0x8B, 0x03, 0x00, 0x8A, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_048[] = {0x00, 0x90};
static u8 cm_049[] = {0xCE, 0x23, 0x58, 0x00, 0x23, 0x59, 0x00, 0x33, 0x5B, 0x00, 0x33, 0x5C, 0x00, 0x00, 0x00};
static u8 cm_050[] = {0x00, 0xA0};
static u8 cm_051[] = {0xCE, 0x38, 0x09, 0x03, 0x5B, 0x00, 0x00, 0x00, 0x38, 0x08, 0x03, 0x5C, 0x00, 0x00, 0x00};
static u8 cm_052[] = {0x00, 0xB0};
static u8 cm_053[] = {0xCE, 0x38, 0x07, 0x03, 0x5D, 0x00, 0x00, 0x00, 0x38, 0x06, 0x03, 0x5E, 0x00, 0x00, 0x00};
static u8 cm_054[] = {0x00, 0xC0};
static u8 cm_055[] = {0xCE, 0x38, 0x05, 0x03, 0x5F, 0x00, 0x00, 0x00, 0x38, 0x04, 0x03, 0x60, 0x00, 0x00, 0x00};
static u8 cm_056[] = {0x00, 0xD0};
static u8 cm_057[] = {0xCE, 0x38, 0x03, 0x03, 0x61, 0x00, 0x00, 0x00, 0x38, 0x02, 0x03, 0x62, 0x00, 0x00, 0x00};
static u8 cm_058[] = {0x00, 0x80};
static u8 cm_059[] = {0xCF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_060[] = {0x00, 0x90};
static u8 cm_061[] = {0xCF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_062[] = {0x00, 0xA0};
static u8 cm_063[] = {0xCF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_064[] = {0x00, 0xB0};
static u8 cm_065[] = {0xCF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_066[] = {0x00, 0xC0};
static u8 cm_067[] = {0xCF, 0x3D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
static u8 cm_068[] = {0x00, 0xD0};
static u8 cm_069[] = {0xCF, 0x00};
static u8 cm_070[] = {0x00, 0x80};
static u8 cm_071[] = {0xCB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_072[] = {0x00, 0x90};
static u8 cm_073[] = {0xCB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_074[] = {0x00, 0xA0};
static u8 cm_075[] = {0xCB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_076[] = {0x00, 0xB0};
static u8 cm_077[] = {0xCB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_078[] = {0x00, 0xC0};
static u8 cm_079[] = {0xCB, 0x54, 0x54, 0x54, 0x54, 0x54, 0x54, 0x54, 0x54, 0x54, 0x54, 0x54, 0x00, 0x00, 0x00, 0x00};
static u8 cm_080[] = {0x00, 0xD0};
static u8 cm_081[] = {0xCB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x54, 0x54, 0x54, 0x54, 0x54, 0x54, 0x54, 0x54, 0x54};
static u8 cm_082[] = {0x00, 0xE0};
static u8 cm_083[] = {0xCB, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_084[] = {0x00, 0xF0};
static u8 cm_085[] = {0xCB, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static u8 cm_086[] = {0x00, 0x80};
static u8 cm_087[] = {0xCC, 0x02, 0x0A, 0x0C, 0x0E, 0x10, 0x21, 0x22, 0x25, 0x25, 0x08};
static u8 cm_088[] = {0x00, 0x90};
static u8 cm_089[] = {0xCC, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x09, 0x0B, 0x0D, 0x0F};
static u8 cm_090[] = {0x00, 0xA0};
static u8 cm_091[] = {0xCC, 0x21, 0x22, 0x25, 0x25, 0x07, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_092[] = {0x00, 0xB0};
static u8 cm_093[] = {0xCC, 0x05, 0x09, 0x0F, 0x0D, 0x0B, 0x21, 0x22, 0x25, 0x25, 0x07};
static u8 cm_094[] = {0x00, 0xC0};
static u8 cm_095[] = {0xCC, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x0A, 0x10, 0x0E, 0x0C};
static u8 cm_096[] = {0x00, 0xD0};
static u8 cm_097[] = {0xCC, 0x21, 0x22, 0x25, 0x25, 0x08, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_098[] = {0x00, 0x00};
static u8 cm_099[] = {0xE1, 0x03, 0x0B, 0x10, 0x0D, 0x06, 0x0B, 0x09, 0x08, 0x06, 0x09, 0x10, 0x0A, 0x10, 0x11, 0x0A, 0x03};
static u8 cm_100[] = {0x00, 0x00};
static u8 cm_101[] = {0xE2, 0x03, 0x0B, 0x10, 0x0D, 0x06, 0x0B, 0x09, 0x08, 0x06, 0x09, 0x10, 0x0A, 0x10, 0x11, 0x0A, 0x03};
static u8 cm_102[] = {0x00, 0x00};
static u8 cm_103[] = {0xFF, 0x00, 0x00, 0x00};
static u8 cm_104[] = {0x00, 0x80};
static u8 cm_105[] = {0xFF, 0x00, 0x00};

static u8 sleep_out[] = {0x11};
static u8 sleep_in[] = {0x10};
static u8 display_on[] = {0x29};
static u8 display_off[] = {0x28};


static struct mipi_dsi_cmd_orise a502cg_power_on_table[] = {
	{0, 0, sizeof(cm_004), cm_004, sizeof(cm_005), cm_005},
	{0, 0, sizeof(cm_006), cm_006, sizeof(cm_007), cm_007},
	{0, 0, sizeof(cm_008), cm_008, sizeof(cm_009), cm_009},
	{0, 0, sizeof(cm_010), cm_010, sizeof(cm_011), cm_011},
	{0, 0, sizeof(cm_012), cm_012, sizeof(cm_013), cm_013},
	{0, 0, sizeof(cm_014), cm_014, sizeof(cm_015), cm_015},
	{0, 0, sizeof(cm_016), cm_016, sizeof(cm_017), cm_017},
	{0, 0, sizeof(cm_018), cm_018, sizeof(cm_019), cm_019},
	{0, 0, sizeof(cm_020), cm_020, sizeof(cm_021), cm_021},
	{0, 0, sizeof(cm_022), cm_022, sizeof(cm_023), cm_023},
	{0, 0, sizeof(cm_024), cm_024, sizeof(cm_025), cm_025},
	{0, 0, sizeof(cm_026), cm_026, sizeof(cm_027), cm_027},
	{0, 0, sizeof(cm_028), cm_028, sizeof(cm_029), cm_029},
	{0, 0, sizeof(cm_030), cm_030, sizeof(cm_031), cm_031},
	{0, 0, sizeof(cm_032), cm_032, sizeof(cm_033), cm_033},
	{0, 0, sizeof(cm_034), cm_034, sizeof(cm_035), cm_035},
	{0, 0, sizeof(cm_036), cm_036, sizeof(cm_037), cm_037},
	{0, 0, sizeof(cm_038), cm_038, sizeof(cm_039), cm_039},
	{0, 0, sizeof(cm_040), cm_040, sizeof(cm_041), cm_041},
	{0, 0, sizeof(cm_042), cm_042, sizeof(cm_043), cm_043},
	{0, 0, sizeof(cm_044), cm_044, sizeof(cm_045), cm_045},
	{0, 0, sizeof(cm_046), cm_046, sizeof(cm_047), cm_047},
	{0, 0, sizeof(cm_048), cm_048, sizeof(cm_049), cm_049},
	{0, 0, sizeof(cm_050), cm_050, sizeof(cm_051), cm_051},
	{0, 0, sizeof(cm_052), cm_052, sizeof(cm_053), cm_053},
	{0, 0, sizeof(cm_054), cm_054, sizeof(cm_055), cm_055},
	{0, 0, sizeof(cm_056), cm_056, sizeof(cm_057), cm_057},
	{0, 0, sizeof(cm_058), cm_058, sizeof(cm_059), cm_059},
	{0, 0, sizeof(cm_060), cm_060, sizeof(cm_061), cm_061},
	{0, 0, sizeof(cm_062), cm_062, sizeof(cm_063), cm_063},
	{0, 0, sizeof(cm_064), cm_064, sizeof(cm_065), cm_065},
	{0, 0, sizeof(cm_066), cm_066, sizeof(cm_067), cm_067},
	{0, 0, sizeof(cm_068), cm_068, sizeof(cm_069), cm_069},
	{0, 0, sizeof(cm_070), cm_070, sizeof(cm_071), cm_071},
	{0, 0, sizeof(cm_072), cm_072, sizeof(cm_073), cm_073},
	{0, 0, sizeof(cm_074), cm_074, sizeof(cm_075), cm_075},
	{0, 0, sizeof(cm_076), cm_076, sizeof(cm_077), cm_077},
	{0, 0, sizeof(cm_078), cm_078, sizeof(cm_079), cm_079},
	{0, 0, sizeof(cm_080), cm_080, sizeof(cm_081), cm_081},
	{0, 0, sizeof(cm_082), cm_082, sizeof(cm_083), cm_083},
	{0, 0, sizeof(cm_084), cm_084, sizeof(cm_085), cm_085},
	{0, 0, sizeof(cm_086), cm_086, sizeof(cm_087), cm_087},
	{0, 0, sizeof(cm_088), cm_088, sizeof(cm_089), cm_089},
	{0, 0, sizeof(cm_090), cm_090, sizeof(cm_091), cm_091},
	{0, 0, sizeof(cm_092), cm_092, sizeof(cm_093), cm_093},
	{0, 0, sizeof(cm_094), cm_094, sizeof(cm_095), cm_095},
	{0, 0, sizeof(cm_096), cm_096, sizeof(cm_097), cm_097},
	{1, 0, sizeof(cm_098), cm_098, sizeof(cm_099), cm_099},
	{1, 0, sizeof(cm_100), cm_100, sizeof(cm_101), cm_101},
};


static int send_mipi_cmd_orise(struct mdfld_dsi_pkg_sender * sender,
				struct mipi_dsi_cmd_orise *cmd) {
	int err = 0;
	int i;
	int r;
	u8 data3[20]={0};

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	if (cmd->gamma_enable) {
		mdfld_dsi_send_mcs_short_lp(sender, cmd->commands1[0], cmd->commands1[1], 1, MDFLD_DSI_SEND_PACKAGE);
		for (i=0; i<(cmd->len2 - 1); i++)
			mdfld_dsi_send_mcs_short_lp(sender, cmd->commands2[0], cmd->commands2[i+1], 1, MDFLD_DSI_SEND_PACKAGE);
	} else {

		for (i=0; i<(cmd->len2 - 1); i++) {
			mdfld_dsi_send_mcs_short_lp(sender, cmd->commands1[0], cmd->commands1[1]+i, 1, MDFLD_DSI_SEND_PACKAGE);
			mdfld_dsi_send_mcs_short_lp(sender, cmd->commands2[0], cmd->commands2[1+i], 1, MDFLD_DSI_SEND_PACKAGE);
		}

#if 0
		printk("-----------------------\n");
		r = mdfld_dsi_send_mcs_short_lp(sender, 0x0 , cmd->commands1[1], 1, 0);
		r = mdfld_dsi_read_gen_lp(sender,cmd->commands2[0],0,1,data3, cmd->len2-1);

		printk("read: %d, 0x%02x%02x",r,cmd->commands2[0], cmd->commands1[1]);
		for(i=0;i<cmd->len2-1;i++){
			printk(" 0x%02x", data3[i]);
		}
		printk("\n");
#endif
	}

	if (err != 0 || sender->status) {
		printk("[DISP] %s : sent failed with status=%d\n", __func__, sender->status);
		return -EIO;
	}

	if (cmd->delay)
		mdelay(cmd->delay);

	return 0;

}


static int send_mipi_cmd_gen(struct mdfld_dsi_pkg_sender * sender,
				struct mipi_dsi_cmd *cmd) {
	int err = 0;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	switch(cmd->len) {
		case 1:
			err = mdfld_dsi_send_gen_short_lp(sender,
				cmd->commands[0],
				0,
				1,
				MDFLD_DSI_SEND_PACKAGE);
			break;
		case 2:
			err = mdfld_dsi_send_gen_short_lp(sender,
				cmd->commands[0],
				cmd->commands[1],
				2,
				MDFLD_DSI_SEND_PACKAGE);
			break;
		default:
			err = mdfld_dsi_send_gen_long_lp(sender,
				cmd->commands,
				cmd->len,
				MDFLD_DSI_SEND_PACKAGE);
			break;
	}

	if (err != 0 || sender->status) {
		printk("[DISP] %s : sent failed with status=%d\n", __func__, sender->status);
		return -EIO;
	}

	if (cmd->delay)
		mdelay(cmd->delay);

	return 0;

}

static int send_mipi_cmd_mcs(struct mdfld_dsi_pkg_sender * sender,
				struct mipi_dsi_cmd *cmd) {
	int err = 0;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	switch(cmd->len) {
		case 1:
			err = mdfld_dsi_send_mcs_short_lp(sender,
				cmd->commands[0],
				0,
				0,
				MDFLD_DSI_SEND_PACKAGE);
			break;
		case 2:
			err = mdfld_dsi_send_mcs_short_lp(sender,
				cmd->commands[0],
				cmd->commands[1],
				1,
				MDFLD_DSI_SEND_PACKAGE);
			break;
		default:
			err = mdfld_dsi_send_mcs_long_lp(sender,
				cmd->commands,
				cmd->len,
				MDFLD_DSI_SEND_PACKAGE);
			break;
	}

	if (err != 0 || sender->status) {
		printk("[DISP] %s : sent failed with status=%d\n", __func__, sender->status);
		return -EIO;
	}

	if (cmd->delay)
		mdelay(cmd->delay);

	return 0;

}

static int orise8018b_vid_drv_ic_init(struct mdfld_dsi_config *dsi_config){

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	struct orise8018b_vid_data *pdata = &gpio_settings_data;
	int i;
	u8 data2[3] = {0};
	printk("[DISP] %s\n", __func__);


	/* panel initial settings */
	mdfld_dsi_read_mcs_lp(sender, 0xB9, data2, 3);
	mdelay(5);
	gpio_direction_output(pdata->gpio_lcd_rst, 1);
	usleep_range(10000, 11000);
	gpio_direction_output(pdata->gpio_lcd_rst, 0);
	usleep_range(10000, 11000);
	gpio_direction_output(pdata->gpio_lcd_rst, 1);
	usleep_range(10000, 11000);

	mdfld_dsi_send_mcs_short_lp(sender, cm_000[0], cm_000[1], 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender, cm_001, sizeof(cm_001), MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, cm_002[0], cm_002[1], 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender, cm_003, sizeof(cm_003), MDFLD_DSI_SEND_PACKAGE);

	for(i = 0; i < ARRAY_SIZE(a502cg_power_on_table); i++)
			send_mipi_cmd_orise(sender, &a502cg_power_on_table[i]);

	mdfld_dsi_send_mcs_short_lp(sender, cm_102[0], cm_102[1], 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender, cm_103, sizeof(cm_103), MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, cm_104[0], cm_104[1], 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender, cm_105, sizeof(cm_105), MDFLD_DSI_SEND_PACKAGE);


	mdfld_dsi_send_mcs_short_lp(sender, 0x11, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdelay(120);
	mdfld_dsi_send_mcs_short_lp(sender, 0x29, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdelay(10);


	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL) {
		printk("[DISP] %s MDFLD_DSI_CONTROL_ABNORMAL !!\n", __func__);
		return -EIO;
	}

	return 0;
}

static void
orise8018b_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	struct drm_device *dev = dsi_config->dev;
#if ENABLE_CSC_GAMMA
	struct csc_setting csc = {	.pipe = 0,
								.type = CSC_REG_SETTING,
								.enable_state = true,
								.data_len = CSC_REG_COUNT,
								.data.csc_reg_data = {
									0x400, 0x0, 0x4000000, 0x0, 0x0, 0x400}
							 };
	struct gamma_setting gamma = {	.pipe = 0,
									.type = GAMMA_REG_SETTING,
									.enable_state = true,
									.data_len = GAMMA_10_BIT_TABLE_COUNT,
									.gamma_tableX100 = {
										0x000000, 0x020202, 0x040404, 0x060606,
										0x080808, 0x0A0A0A, 0x0C0C0C, 0x0E0E0E,
										0x101010, 0x121212, 0x141414, 0x161616,
										0x181818, 0x1A1A1A, 0x1C1C1C, 0x1E1E1E,
										0x202020, 0x222222, 0x242424, 0x262626,
										0x282828, 0x2A2A2A, 0x2C2C2C, 0x2E2E2E,
										0x303030, 0x323232, 0x343434, 0x363636,
										0x383838, 0x3A3A3A, 0x3C3C3C, 0x3E3E3E,
										0x404040, 0x424242, 0x444444, 0x464646,
										0x484848, 0x4A4A4A, 0x4C4C4C, 0x4E4E4E,
										0x505050, 0x525252, 0x545454, 0x565656,
										0x585858, 0x5A5A5A, 0x5C5C5C, 0x5E5E5E,
										0x606060, 0x626262, 0x646464, 0x666666,
										0x686868, 0x6A6A6A, 0x6C6C6C, 0x6E6E6E,
										0x707070, 0x727272, 0x747474, 0x767676,
										0x787878, 0x7A7A7A, 0x7C7C7C, 0x7E7E7E,
										0x808080, 0x828282, 0x848484, 0x868686,
										0x888888, 0x8A8A8A, 0x8C8C8C, 0x8E8E8E,
										0x909090, 0x929292, 0x949494, 0x969696,
										0x989898, 0x9A9A9A, 0x9C9C9C, 0x9E9E9E,
										0xA0A0A0, 0xA2A2A2, 0xA4A4A4, 0xA6A6A6,
										0xA8A8A8, 0xAAAAAA, 0xACACAC, 0xAEAEAE,
										0xB0B0B0, 0xB2B2B2, 0xB4B4B4, 0xB6B6B6,
										0xB8B8B8, 0xBABABA, 0xBCBCBC, 0xBEBEBE,
										0xC0C0C0, 0xC2C2C2, 0xC4C4C4, 0xC6C6C6,
										0xC8C8C8, 0xCACACA, 0xCCCCCC, 0xCECECE,
										0xD0D0D0, 0xD2D2D2, 0xD4D4D4, 0xD6D6D6,
										0xD8D8D8, 0xDADADA, 0xDCDCDC, 0xDEDEDE,
										0xE0E0E0, 0xE2E2E2, 0xE4E4E4, 0xE6E6E6,
										0xE8E8E8, 0xEAEAEA, 0xECECEC, 0xEEEEEE,
										0xF0F0F0, 0xF2F2F2, 0xF4F4F4, 0xF6F6F6,
										0xF8F8F8, 0xFAFAFA, 0xFCFCFC, 0xFEFEFE,
										0x010000, 0x010000, 0x010000}
								 };

#endif
	printk("[DISP] %s\n", __func__);
	/* Reconfig lane configuration */
	dsi_config->lane_count = 2;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;
	dsi_config->enable_gamma_csc = ENABLE_GAMMA | ENABLE_CSC;

	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;
	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x3f;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x15;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x3;
	hw_ctx->lp_byteclk = 0x6;
	hw_ctx->clk_lane_switch_time_cnt = 0x15000a;
	hw_ctx->dphy_param = 0x150c340f;

	/* Setup video mode format */
	hw_ctx->video_mode_format = 0xf;

	/* Set up func_prg, RGB888(0x200) */
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/* Setup mipi port configuration */
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;

	orise8018b_dsi_config = dsi_config;

#if ENABLE_CSC_GAMMA
	if (dsi_config->enable_gamma_csc & ENABLE_CSC) {
		/* setting the tuned csc setting */
		drm_psb_enable_color_conversion = 1;
		mdfld_intel_crtc_set_color_conversion(dev, &csc);
	}

	if (dsi_config->enable_gamma_csc & ENABLE_GAMMA) {
		/* setting the tuned gamma setting */
		drm_psb_enable_gamma = 1;
//		mdfld_intel_crtc_set_gamma(dev, &gamma);
	}
#endif
}

static int orise8018b_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;

	printk("[DISP] %s\n", __func__);

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
			OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
		    (dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
			psb_enable_vblank(dev, pipe);
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static int orise8018b_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	printk("[DISP] %s\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	usleep_range(1000, 1200);

	/* Send TURN_ON packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
					    MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	/* EN_VDD_BL*/
#if PWM_SOC_ENABLE
	pwm_enable();
#endif
//	intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0x01);
//	intel_scu_ipc_iowrite8(PMIC_GPIO_VEMMC2CNT, 0x06);

	queue_delayed_work(orise8018b_panel_reset_delay_wq, &orise8018b_panel_reset_delay_work, msecs_to_jiffies(5000));

	return 0;
}

static int orise8018b_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
//	struct orise8018b_vid_data *pdata = &gpio_settings_data;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	printk("[DISP] %s\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	cancel_delayed_work_sync(&orise8018b_panel_reset_delay_work);

	/* Turn off the backlight*/
#if PWM_SOC_ENABLE
	pwm_disable();
#endif
	usleep_range(1000, 1500);

	/* Send SHUT_DOWN packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
					    MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}

	/* Send power off command*/
	mdfld_dsi_send_mcs_short_lp(sender, 0x28, 0x00, 0, 0);
	mdfld_dsi_send_mcs_short_lp(sender, 0x10, 0x00, 0, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL) {
		printk("[DISP] %s MDFLD_DSI_CONTROL_ABNORMAL !!\n", __func__);
		return -EIO;
	}
	usleep_range(50000, 55000);
	/* Driver IC power off sequence*/
//	intel_scu_ipc_iowrite8(PMIC_GPIO_VEMMC2CNT, 0x04);
#if 0 /* Skip put down the HW_RST pin */
	gpio_direction_output(pdata->gpio_lcd_rst, 0);
	usleep_range(120000, 121000);
#endif
	return 0;
}

static int orise8018b_vid_reset(struct mdfld_dsi_config *dsi_config)
{
//	struct orise8018b_vid_data *pdata = &gpio_settings_data;
	printk("[DISP] %s\n", __func__);

      // start the initial state
//	intel_scu_ipc_iowrite8(PMIC_GPIO_VEMMC2CNT, 0x04);
//	usleep_range(1000, 1500);
//	intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0x00);

	// start power on sequence
//	intel_scu_ipc_iowrite8(PMIC_GPIO_VEMMC2CNT, 0x06);
//	usleep_range(5000, 5500);

 /* Move the RST control to driverIC init */
/*
	gpio_direction_output(pdata->gpio_lcd_rst, 1);
	usleep_range(10000, 11000);
	gpio_direction_output(pdata->gpio_lcd_rst, 0);
	usleep_range(10000, 11000);
	gpio_direction_output(pdata->gpio_lcd_rst, 1);
	usleep_range(10000, 11000);
*/
	return 0;
}


#define PWM0CLKDIV1 0x61
#define PWM0CLKDIV0 0x62

#define PWM0DUTYCYCLE 0x67
#define DUTY_VALUE_MAX 0x63


#define BRI_SETTING_MIN 10
//#define BRI_SETTING_DEF 170
#define BRI_SETTING_MAX 255

static int orise8018b_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	int duty_val = 0;
	int ret = 0;
	unsigned int pwm_min, pwm_max;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
#if PWM_SOC_ENABLE
	pwm_min = 5;
	pwm_max = 255;

	if (level <= 0) {
		duty_val = 0;
	} else if (level > 0 && (level < BRI_SETTING_MIN)) {
		duty_val = pwm_min;
	} else if ((level >= BRI_SETTING_MIN) && (level <= BRI_SETTING_MAX)) {
		duty_val = (level - BRI_SETTING_MIN) * (pwm_max - pwm_min) /
			(BRI_SETTING_MAX - BRI_SETTING_MIN) + pwm_min;
	} else if (level > BRI_SETTING_MAX)
		duty_val = pwm_max;

	pwm_configure(duty_val);
#else
	duty_val = ((DUTY_VALUE_MAX + 1) * level) / 255;
	ret = intel_scu_ipc_iowrite8(PWM0DUTYCYCLE, (duty_val > DUTY_VALUE_MAX ? DUTY_VALUE_MAX : duty_val));
	if (ret)
		DRM_ERROR("write brightness duty value faild\n");
#endif

	printk("[DISP] brightness level = %d , duty_val = %d\n", level, duty_val);

	return 0;
}

struct drm_display_mode *orise8018b_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	printk("[DISP] %s\n", __func__);
	/* RECOMMENDED PORCH SETTING
		HSA=15, HBP=30, HFP=36
		VSA=3,   VBP=12, VFP=22	 */
	mode->hdisplay = 480;
	mode->vdisplay = 854;
	mode->hsync_start = mode->hdisplay + 64;
	mode->hsync_end = mode->hsync_start + 4;
	mode->htotal = mode->hsync_end + 60;
	mode->vsync_start = mode->vdisplay + 32;
	mode->vsync_end = mode->vsync_start + 4;
	mode->vtotal = mode->vsync_end + 30;

	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void orise8018b_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	pi->width_mm = 61;
	pi->height_mm = 109;
}

static int orise8018b_vid_gpio_init(void)
{
	int ret;
	struct orise8018b_vid_data *pdata = &gpio_settings_data;

	printk("[DISP] %s\n", __func__);

	pdata->gpio_lcd_rst = get_gpio_by_name("DISP_RST_N");

	ret = gpio_request(pdata->gpio_lcd_rst, "DISP_RST_N");
	if (ret < 0)
		DRM_ERROR("Faild to get panel GPIO DISP_RST_N:%d\n", pdata->gpio_lcd_rst);

//	intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0x01);

	return 0;
}

static int orise8018b_vid_brightness_init(void)
{
	int ret = 0;

	printk("[DISP] %s\n", __func__);
#if PWM_SOC_ENABLE
	pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG,PWMCTRL_SIZE);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);
#else
	ret = intel_scu_ipc_iowrite8(PWM0CLKDIV1, 0x00);
	if (!ret)
		ret = intel_scu_ipc_iowrite8(PWM0CLKDIV0, 0x25);

	if (ret)
		printk("[DISP] %s: PWM0CLKDIV set failed\n", __func__);
	else
		printk("[DISP] PWM0CLKDIV set to 0x%04x\n", 0x25);
#endif

	return ret;
}


#ifdef ORISE8018B_DEBUG
static int send_mipi_ret = -1;
static int read_mipi_ret = -1;
static u8 read_mipi_data = 0;

static ssize_t send_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0, x1=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(orise8018b_dsi_config);

    sscanf(buf, "%x,%x", &x0, &x1);

//	send_mipi_ret = mdfld_dsi_send_mcs_short_lp(sender,x0,x1,1,0);
	send_mipi_ret = mdfld_dsi_send_gen_short_lp(sender,x0,x1,2,0);

	DRM_INFO("[DISPLAY] send %x,%x : ret = %d\n",x0,x1,send_mipi_ret);

    return count;
}

static ssize_t send_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",send_mipi_ret);
}


static ssize_t read_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(orise8018b_dsi_config);

    sscanf(buf, "%x", &x0);

    read_mipi_ret = mdfld_dsi_read_mcs_lp(sender,x0,&read_mipi_data,1);
    if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
        read_mipi_ret = -EIO;

	DRM_INFO("[DISPLAY] read 0x%x :ret=%d data=0x%x\n", x0, read_mipi_ret, read_mipi_data);

    return count;
}

static ssize_t read_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "ret=%d data=0x%x\n",read_mipi_ret,read_mipi_data);
}

DEVICE_ATTR(send_mipi_orise8018b,S_IRUGO | S_IWUSR, send_mipi_show,send_mipi_store);
DEVICE_ATTR(read_mipi_orise8018b,S_IRUGO | S_IWUSR, read_mipi_show,read_mipi_store);


static struct attribute *orise8018b_attrs[] = {
        &dev_attr_send_mipi_orise8018b.attr,
        &dev_attr_read_mipi_orise8018b.attr,
        NULL
};

static struct attribute_group orise8018b_attr_group = {
        .attrs = orise8018b_attrs,
        .name = "orise8018b",
};

#endif

void orise8018b_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret = 0;
	printk("[DISP] %s\n", __func__);

	/* Read Project ID */
	board_proj_id = Read_PROJ_ID();

	p_funcs->get_config_mode = orise8018b_vid_get_config_mode;
	p_funcs->get_panel_info = orise8018b_vid_get_panel_info;
	p_funcs->dsi_controller_init = orise8018b_vid_dsi_controller_init;
	p_funcs->detect = orise8018b_vid_detect;
	p_funcs->power_on = orise8018b_vid_power_on;
	p_funcs->drv_ic_init = orise8018b_vid_drv_ic_init;
	p_funcs->power_off = orise8018b_vid_power_off;
	p_funcs->reset = orise8018b_vid_reset;
	p_funcs->set_brightness = orise8018b_vid_set_brightness;

	ret = orise8018b_vid_gpio_init();
	if (ret)
		DRM_ERROR("Faild to request GPIO for ORISE8018B panel\n");

	ret = orise8018b_vid_brightness_init();
	if (ret)
		DRM_ERROR("Faild to initilize PWM of MSCI\n");

	printk("[DISP] HSD panel reset workqueue init!\n");
	INIT_DELAYED_WORK(&orise8018b_panel_reset_delay_work, orise8018b_vid_panel_reset_delay_work);
	orise8018b_panel_reset_delay_wq = create_workqueue("orise8018b_panel_reset_delay_timer");
	if (unlikely(!orise8018b_panel_reset_delay_wq)) {
		printk("%s : unable to create Panel reset workqueue\n", __func__);
	}

#ifdef ORISE8018B_DEBUG
    sysfs_create_group(&dev->dev->kobj, &orise8018b_attr_group);
#endif

}

static int orise8018b_vid_shutdown(struct platform_device *pdev)
{
	struct orise8018b_vid_data *pdata = &gpio_settings_data;
	printk("[DISP] %s\n", __func__);

	intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0);
	orise8018b_vid_set_brightness(orise8018b_dsi_config, 0);
	orise8018b_vid_power_off(orise8018b_dsi_config);
	usleep_range(50000, 55000);
	gpio_direction_output(pdata->gpio_lcd_rst, 0);
	usleep_range(120000, 121000);

	intel_scu_ipc_iowrite8(PMIC_GPIO_VEMMC2CNT, 0x04);

	return 0;
}


static int orise8018b_vid_lcd_probe(struct platform_device *pdev)
{
	printk("[DISP] %s: ORISE8018B panel detected\n", __func__);
	intel_mid_panel_register(orise8018b_vid_init);

	return 0;
}

struct platform_driver orise8018b_lcd_driver = {
	.probe	= orise8018b_vid_lcd_probe,
	.shutdown = orise8018b_vid_shutdown,
	.driver	= {
		.name	= ORISE8018B_PANEL_NAME,
		.owner	= THIS_MODULE,
	},
};
