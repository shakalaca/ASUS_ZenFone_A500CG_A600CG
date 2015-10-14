/* include/linux/cm3628.h
 *
 * Copyright (C) 2012 Capella Microsystems Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_CM3628_H
#define __LINUX_CM3628_H

#define CM3628_I2C_NAME "cm3628"

/* Define Slave Address*/

/****** modified start  *****/
#define     ALS_slave_address   (0xC0 >>1)
#define     PS_slave_address    (0xC2 >>1)
//#define     ALS_slave_address   (0xC0 )
//#define     PS_slave_address    (0xC2 )
/****** modified end  *****/


/****** modified start  *****/
/*Define Interrupt address*/
#define     interrupt_add       (0x2C>>1)
//#define     interrupt_add       0x2C
/****** modified end  *****/

#define ALS_CALIBRATED      0x6E9F
#define PS_CALIBRATED         0x509F

/*Define ALS Command Code*/
#define     ALS_cmd_cmd         0x00
#define     ALS_average         0x01
#define     ALS_high_thd_msb    0x02
#define     ALS_high_thd_lsb    0x03
#define     ALS_low_thd_msb     0x04
#define     ALS_low_thd_lsb     0x05

/* Define PS Command Code*/
#define     PS_cmd_cmd          0x00
#define     PS_thd              0x01
#define     PS_cancel           0x02
#define     PS_default          0x03

/*cm3628*/
/*for ALS command 00h*/
#define CM3628_ALS_IT_50ms  (0 << 6)
#define CM3628_ALS_IT_100ms     (1 << 6)
#define CM3628_ALS_IT_200ms     (2 << 6)
#define CM3628_ALS_IT_400ms     (3 << 6)
#define CM3628_ALS_PERS_1       (0 << 4)
#define CM3628_ALS_PERS_2       (1 << 4)
#define CM3628_ALS_PERS_4       (2 << 4)
#define CM3628_ALS_PERS_8       (3 << 4)
#define CM3628_ALS_BIT2_Default_1       (1 << 2)
#define CM3628_ALS_INT_EN       (1 << 1) /*enable/disable Interrupt*/
#define CM3628_ALS_SD           (1 << 0)/*enable/disable ALS func*/

/*for ALS command 01h*/
#define CM3628_ALS_RES          (1 << 6)
#define CM3628_ALS_AV_1     (0 << 4)
#define CM3628_ALS_AV_2     (1 << 4)
#define CM3628_ALS_AV_4     (2 << 4)
#define CM3628_ALS_AV_8     (3 << 4)
#define CM3628_ALS_HS           (1 << 3)

/*for PS command 00h*/
#define CM3628_PS_DR_1_80       (0 << 6)
#define CM3628_PS_DR_1_160  (1 << 6)
#define CM3628_PS_DR_1_320  (2 << 6)
#define CM3628_PS_DR_1_640  (3 << 6)
#define CM3628_PS_IT_1T         (0 << 4)
#define CM3628_PS_IT_1_3T       (1 << 4)
#define CM3628_PS_IT_1_6T       (2 << 4)
#define CM3628_PS_IT_2T         (3 << 4)
#define CM3628_PS_PERS_1        (0 << 2)
#define CM3628_PS_PERS_2        (1 << 2)
#define CM3628_PS_PERS_3        (2 << 2)
#define CM3628_PS_PERS_4        (3 << 2)
#define CM3628_PS_INT_EN        (1 << 1)/*enable/disable Interrupt*/
#define CM3628_PS_SD            (1 << 0)/*enable/disable PS func, 1:disable , 0: enable*/

/*for PS command 03h*/
#define CM3628_PS_MS            (1 << 4)
#define CM3628_PS_HYS           (1 << 0)
#define CM3628_PS_SMART_PERS    (1 << 1)
#define CM3628_IT_4X_ES         (1 << 6)/*for integration time*/
#define CM3628_IT_4X_MP         (1 << 7)/*for integration time*/

// added start  -----------------------
extern unsigned int ps_kparam1;
extern unsigned int ps_kparam2;
extern struct blocking_notifier_head psensor_notifier_list;
// added end  --------------------------




struct cm3628_platform_data {
    int intr;
    unsigned short levels[10];
    unsigned short golden_adc;
    int (*power)(int, unsigned char); /* power to the chip */
    unsigned char ALS_IT;
    unsigned char ALS_PERS;
    unsigned char ALS_AV;
    unsigned char ALS_HS;
    unsigned char PS_DR;
    unsigned char PS_IT;
    unsigned char PS_PERS;
    unsigned char PS_MS;
    unsigned char PS_HYS;
    unsigned char ls_slave_addr;
    unsigned char ps_slave_addr;
    unsigned char check_interrupt_add;
    unsigned char ps_thd_set;
    unsigned char inte_cancel_set;
    /*command code 0x02, intelligent cancel level, for ps calibration*/
    unsigned char ps_conf2_val; /* PS_CONF2 value */
    unsigned char ps_calibration_rule;
    unsigned char ps_conf1_val;
    unsigned char *mapping_table;
    unsigned char mapping_size;
    unsigned char ps_base_index;
    unsigned char enable_polling_ignore;
    unsigned char ps_thd_no_cal;
    unsigned char ps_thd_with_cal;
    unsigned char ls_cmd;
    unsigned char ps_adc_offset;
    unsigned char ps_adc_offset2;
    unsigned char ps_debounce;
    unsigned short ps_delay_time;
    unsigned char ps_reset_thd;
};

#endif  // end of __LINUX_CM3628_H

