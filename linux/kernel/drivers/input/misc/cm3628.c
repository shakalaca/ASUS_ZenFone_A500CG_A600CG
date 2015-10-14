/*vers/i2c/chips/cm3628.c - cm3628 optical sensors driver
 *   
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
/***************************
 obj-$(CONFIG_SENSORS_CM3628)   += cm3628.o
 ***************************/
///////////////////////////////////////////////

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/cm3628.h>
#include <linux/capella_cm3602.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/HWVersion.h>

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

#define POLLING_PROXIMITY 1
#define NO_IGNORE_BOOT_MODE 1

#define NEAR_DELAY_TIME ((100 * HZ) / 1000)

#ifdef POLLING_PROXIMITY
#define POLLING_DELAY       200
#define TH_ADD          3
#endif
/*
typedef signed char int8_t;
typedef short int16_t;
typedef long int32_t;
typedef long long int64_t;

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;
typedef unsigned long long uint64_t;

typedef int int_fast8_t;
typedef int int_fast16_t;
typedef long int_fast32_t;

typedef unsigned int uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned long uint_fast32_t;
*/



static int record_init_fail = 0;
static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

#ifdef POLLING_PROXIMITY
static void polling_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(polling_work, polling_do_work);
#endif

static void report_near_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(report_near_work, report_near_do_work);

struct cm3628_info {
    struct class *cm3628_class;
    struct device *ls_dev;
    struct device *ps_dev;

    struct input_dev *ls_input_dev;
    struct input_dev *ps_input_dev;

    struct early_suspend early_suspend;
    struct i2c_client *i2c_client;
    struct workqueue_struct *lp_wq;

    int intr_pin;

    int als_enable;

    int ps_enable;
    int ps_irq_flag;
    int led;

    unsigned short *adc_table;
    unsigned short cali_table[10];
    int irq;

    int ls_calibrate;

    int (*power)(int, unsigned char); /* power to the chip */

    unsigned long als_kadc;
    unsigned long als_gadc;
    unsigned short golden_adc;

    struct wake_lock ps_wake_lock;
    int psensor_opened;
    int lightsensor_opened;
    unsigned char ls_slave_addr;
    unsigned short ps_slave_addr;
    unsigned short check_interrupt_add;
    unsigned char ps_thd_set;
    unsigned char original_ps_thd_set;
    int current_level;
    unsigned short current_adc;
    unsigned char inte_cancel_set;
    /*command code 0x02, intelligent cancel level, for ps calibration*/
    unsigned char ps_conf2_val; /* PS_CONF2 value */
    unsigned char ps_conf1_val;
    int ps_pocket_mode;

    unsigned long j_start;
    unsigned long j_end;
    int mfg_mode;

    unsigned char *mapping_table;
    unsigned char mapping_size;
    unsigned char ps_base_index;
    unsigned char ps_thd_no_cal;
    unsigned char ps_thd_with_cal;
    unsigned char enable_polling_ignore;
    unsigned short ls_cmd;
    unsigned char ps_adc_offset;
    unsigned char ps_adc_offset2;
    unsigned char ps_debounce;
    unsigned short ps_delay_time;
    unsigned char ps_reset_thd;
    unsigned char ps_calibration_rule;
};
static unsigned char ps_cancel_set;
static unsigned char ps_offset_adc;
static unsigned char ps_offset_adc2;
struct cm3628_info *lp_info;
int enable_log = 0;
int fLevel=-1;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static int lightsensor_enable(struct cm3628_info *lpi);
static int lightsensor_disable(struct cm3628_info *lpi);
static int initial_cm3628(struct cm3628_info *lpi);
static void psensor_initial_cmd(struct cm3628_info *lpi);

int32_t als_kadc;

static int I2C_RxData(unsigned short slaveAddr, unsigned char *rxData, int length)
{
    unsigned char loop_i;
    int val;
    struct cm3628_info *lpi = lp_info;

    struct i2c_msg msgs[] = {
        {
            .addr = slaveAddr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = rxData,
        },
    };

    for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {

        if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 1) > 0)
            break;

        val = gpio_get_value(lpi->intr_pin);
        /*check intr GPIO when i2c error*/
        if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
            D("[ALS+PS][CM3628-AD3 error] %s, i2c err, slaveAddr 0x%x ISR gpio %d  = %d, record_init_fail %d \n",
                    __func__, slaveAddr, lpi->intr_pin, val, record_init_fail);

        msleep(10);
    }
    if (loop_i >= I2C_RETRY_COUNT) {
        printk(KERN_ERR "[ALS+PS_ERR][CM3628-AD3 error] %s retry over %d\n",
                __func__, I2C_RETRY_COUNT);
        return -EIO;
    }

    return 0;
}

static int I2C_TxData(unsigned short slaveAddr, unsigned char *txData, int length)
{
    unsigned char loop_i;
    int val;
    struct cm3628_info *lpi = lp_info;
    struct i2c_msg msg[] = {
        {
            .addr = slaveAddr,
            .flags = 0,
            .len = length,
            .buf = txData,
        },
    };

    for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
        if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
            break;

        val = gpio_get_value(lpi->intr_pin);
        /*check intr GPIO when i2c error*/
        if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
            D("[ALS+PS][CM3628-AD3 error] %s, i2c err, slaveAddr 0x%x, register 0x%x, value 0x%x, ISR gpio%d  = %d, record_init_fail %d\n",
                    __func__, slaveAddr, txData[0], txData[1], lpi->intr_pin, val, record_init_fail);

        msleep(10);
    }

    if (loop_i >= I2C_RETRY_COUNT) {
        printk(KERN_ERR "[PS_ERR][CM3628-AD3 error] %s retry over %d\n",
                __func__, I2C_RETRY_COUNT);
        return -EIO;
    }

    return 0;
}


static int _cm3628_I2C_Read_Byte(unsigned short slaveAddr, unsigned char *pdata)
{
    unsigned char buffer = 0;
    int ret = 0;

    if (pdata == NULL)
        return -EFAULT;

    ret = I2C_RxData(slaveAddr, &buffer, 1);
    if (ret < 0) {
        pr_err(
                "[ALS+PS_ERR][CM3628-AD3 error]%s: I2C_RxData fail, slave addr: 0x%x\n",
                __func__, slaveAddr);
        return ret;
    }

    *pdata = buffer;
#if 0
    /* Debug use */
    printk(KERN_DEBUG "[CM3628-AD3] %s: I2C_RxData[0x%x] = 0x%x\n",
            __func__, slaveAddr, buffer);
#endif
    return ret;
}

static int _cm3628_I2C_Write_Byte(unsigned short SlaveAddress,
        unsigned char cmd, unsigned char data)
{
    char buffer[2];
    int ret = 0;
#if 0
    /* Debug use */
    printk(KERN_DEBUG
            "[CM3628-AD3] %s: _cm3628_I2C_Write_Byte[0x%x, 0x%x, 0x%x]\n",
            __func__, SlaveAddress, cmd, data);
#endif
    buffer[0] = cmd;
    buffer[1] = data;

    ret = I2C_TxData(SlaveAddress, buffer, 2);
    
    
    if (ret < 0) {
        pr_err("[ALS+PS_ERR][CM3628-AD3 error]%s: I2C_TxData fail\n", __func__);
        return -EIO;
    }

    return ret;
}
static int get_ls_adc_value(unsigned short *als_step, bool resume)
{

    struct cm3628_info *lpi = lp_info;
    unsigned char lsb, msb;
    int ret = 0;
    char cmd = 0;

    if (als_step == NULL)
        return -EFAULT;

    if (resume) {
        cmd = (CM3628_ALS_IT_50ms | CM3628_ALS_PERS_1 |
                CM3628_ALS_BIT2_Default_1);/* disable CM3628_ALS_INT_EN */
        D("[LS][CM3628-AD3] %s:resume %d\n",
                __func__, resume);
    } else
        cmd = (lpi->ls_cmd |
                CM3628_ALS_BIT2_Default_1);/* disable CM3628_ALS_INT_EN */
    ret = _cm3628_I2C_Write_Byte(lpi->ls_slave_addr, ALS_cmd_cmd, cmd);
    if (ret < 0) {
        pr_err(
                "[LS][CM3628-AD3 error]%s: _cm3628_I2C_Write_Byte fail\n",
                __func__);
        return -EIO;
    }

    /* Read ALS data: LSB */
    ret = _cm3628_I2C_Read_Byte(lpi->ls_slave_addr, &lsb);
    if (ret < 0) {
        pr_err(
                "[LS][CM3628-AD3 error]%s: _cm3628_I2C_Read_Byte LSB fail\n",
                __func__);
        return -EIO;
    }

    /* Read ALS data: MSB */
    ret = _cm3628_I2C_Read_Byte(lpi->ls_slave_addr, &msb);
    if (ret < 0) {
        pr_err(
                "[LS][CM3628-AD3 error]%s: _cm3628_I2C_Read_Byte MSB fail\n",
                __func__);
        return -EIO;
    }


    *als_step = (unsigned short)msb;
    *als_step <<= 8;
    *als_step |= (unsigned short)lsb;

    D("[LS][CM3628-AD3] %s: raw adc = 0x%X, ls_calibrate = %d\n",
            __func__, *als_step, lpi->ls_calibrate);


    if (!lpi->ls_calibrate) {
        *als_step = (*als_step) * lpi->als_gadc / lpi->als_kadc;
        if (*als_step > 0xFFFF)
            *als_step = 0xFFFF;
    }


    return ret;
}

static int get_ps_adc_value(unsigned char *data)
{
    int ret = 0;
    struct cm3628_info *lpi = lp_info;

    if (data == NULL)
        return -EFAULT;
    ret = _cm3628_I2C_Read_Byte(lpi->ps_slave_addr, data);
    if (ret < 0) {
        pr_err(
                "[PS_ERR][CM3628-AD3 error]%s: _cm3628_I2C_Read_Byte MSB fail\n",
                __func__);
        return -EIO;
    }

    return ret;
}

static int set_lsensor_range(unsigned short low_thd, unsigned short high_thd)
{
    int ret = 0;
    struct cm3628_info *lpi = lp_info;

    unsigned char high_msb;
    unsigned char high_lsb;
    unsigned char low_msb;
    unsigned char low_lsb;
    /*D("[CM3628-AD3] %s: low_thd = 0x%X, high_thd = 0x%x \n",
      __func__, low_thd, high_thd);*/
    high_msb = (unsigned char) (high_thd >> 8);
    high_lsb = (unsigned char) (high_thd & 0x00ff);
    low_msb  = (unsigned char) (low_thd >> 8);
    low_lsb  = (unsigned char) (low_thd & 0x00ff);

    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr,
            ALS_high_thd_msb, high_msb);
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr,
            ALS_high_thd_lsb, high_lsb);
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr,
            ALS_low_thd_msb, low_msb);
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr,
            ALS_low_thd_lsb, low_lsb);

    return ret;
}

static void report_near_do_work(struct work_struct *w)
{
    struct cm3628_info *lpi = lp_info;

    D("[PS][CM3628-AD3]  %s: delay 500ms, report proximity NEAR\n", __func__);

    input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 0);
    input_sync(lpi->ps_input_dev);
    wake_lock_timeout(&(lpi->ps_wake_lock), 2*HZ);
}
static void report_psensor_input_event(struct cm3628_info *lpi, int interrupt_flag)
{
    unsigned char ps_data;
    int val, ret = 0;

    if (interrupt_flag == 1 && lpi->ps_enable == 0) {
        /*P-sensor disable but interrupt occur. It might init fail when power on.workaround: reinit*/
        D("[PS][CM3628-AD3] proximity err, ps_enable %d, but intrrupt occur, record_init_fail %d, interrupt_flag %d\n",
                lpi->ps_enable, record_init_fail, interrupt_flag);
        return;
    }
    if (lpi->ps_debounce == 1 &&
            lpi->mfg_mode != NO_IGNORE_BOOT_MODE)
        cancel_delayed_work(&report_near_work);

    lpi->j_end = jiffies;
    /*D("%s: j_end = %lu", __func__, lpi->j_end);*/
    ret = get_ps_adc_value(&ps_data);/*check i2c result*/
    if (ret == 0) {
        val = (ps_data >= lpi->ps_thd_set) ? 0 : 1;      
    } else {/*i2c err, report far to workaround*/
        val = 1;
        ps_data = 0;
        D("[PS][CM3628-AD3] proximity i2c err, report %s, ps_data=%d, record_init_fail %d\n",
                val ? "FAR" : "NEAR", ps_data, record_init_fail);
    }
    if (lpi->ps_debounce == 1 &&
            lpi->mfg_mode != NO_IGNORE_BOOT_MODE) {
        if (val == 0) {
            D("[PS][CM3628-AD3] delay proximity %s, ps_data=%d\n", val ? "FAR" : "NEAR", ps_data);
            queue_delayed_work(lpi->lp_wq, &report_near_work,
                    msecs_to_jiffies(lpi->ps_delay_time));
            return;
        } else {
            /* dummy report */
            input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, -1);
            input_sync(lpi->ps_input_dev);
        }
    }
    D("[PS][CM3628-AD3] proximity %s, ps_data=%d\n", val ? "FAR" : "NEAR", ps_data);
    if ((lpi->enable_polling_ignore == 1) && (val == 0) &&
            (lpi->mfg_mode != NO_IGNORE_BOOT_MODE) &&
            (time_before(lpi->j_end, (lpi->j_start + NEAR_DELAY_TIME)))) {
        D("[PS][CM3628-AD3] Ignore NEAR event\n");
        lpi->ps_pocket_mode = 1;
    } else {
        /* 1 is close, 0 is far */
//                int index = 0;              
//            for(index=0; index<5; index++){  D("[PS][CM3628-AD3] index = %d\n",index);
                input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, val);
                input_sync(lpi->ps_input_dev);
//              }  
    }

    wake_lock_timeout(&(lpi->ps_wake_lock), 2*HZ);
}

static void enable_als_int(void)/*enable als interrupt*/
{
    char cmd = 0;
    struct cm3628_info *lpi = lp_info;
    int ret = 0;

    cmd = (lpi->ls_cmd |
            CM3628_ALS_BIT2_Default_1 | CM3628_ALS_INT_EN);

    ret = _cm3628_I2C_Write_Byte(lpi->ls_slave_addr, ALS_cmd_cmd, cmd);
    if (ret != 0) {
        lpi->als_enable = 0;
        D("[LS][CM3628-AD3] L-sensor i2c err, enable interrupt error\n");
    } else
        lpi->als_enable = 1;
}

static void report_lsensor_input_event(struct cm3628_info *lpi, bool resume)
{/*when resume need report a data, so the paramerter need to quick reponse*/
    unsigned short adc_value = 0;
    int level = 0, i, ret = 0;

    mutex_lock(&als_get_adc_mutex);

    ret = get_ls_adc_value(&adc_value, resume);
    if (resume)
        adc_value = adc_value*8;/*because the CM3628_ALS_IT for 400ms - >50ms*/

    for (i = 0; i < 10; i++) {
        if (adc_value <= (*(lpi->adc_table + i))) {
            level = i;
            if (*(lpi->adc_table + i))
                break;
        }
        if ( i == 9) {/*avoid  i = 10, because 'cali_table' of size is 10 */
            level = i;
            break;
        }
    }
    ret = set_lsensor_range(((i == 0) || (adc_value == 0)) ? 0 :
            *(lpi->cali_table + (i - 1)) + 1,
            *(lpi->cali_table + i));

    if (ret < 0)
        printk(KERN_ERR "[LS][CM3628-AD3 error] %s fail\n", __func__);

    if ((i == 0) || (adc_value == 0))
        D("[LS][CM3628-AD3] %s: ADC=0x%03X, Level=%d, l_thd equal 0, h_thd = 0x%x \n",
                __func__, adc_value, level, *(lpi->cali_table + i));
    else
        D("[LS][CM3628-AD3] %s: ADC=0x%03X, Level=%d, l_thd = 0x%x, h_thd = 0x%x \n",
                __func__, adc_value, level, *(lpi->cali_table + (i - 1)) + 1, *(lpi->cali_table + i));

    lpi->current_level = level;
    lpi->current_adc = adc_value;
    /*D("[CM3628-AD3] %s: *(lpi->cali_table + (i - 1)) + 1 = 0x%X, *(lpi->cali_table + i) = 0x%x \n", __func__, *(lpi->cali_table + (i - 1)) + 1, *(lpi->cali_table + i));*/
    if(fLevel>=0){
        D("[LS][CM3628-AD3] L-sensor force level enable level=%d fLevel=%d\n",level,fLevel);
        level=fLevel;
    }
    input_report_abs(lpi->ls_input_dev, ABS_MISC, level);
    input_sync(lpi->ls_input_dev);
    enable_als_int();
    mutex_unlock(&als_get_adc_mutex);

}

void enable_ps_int(int cmd_value)
{
    struct cm3628_info *lpi = lp_info;
    int ret;

    lpi->ps_enable = 1;
    /* settng command code(0x01) = 0x03*/
    _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_thd, lpi->ps_thd_set);
    /*_cm3628_I2C_Write_Byte(lpi->ps_slave_addr, PS_cmd_cmd, 0x32);*/
    ret = _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_cmd_cmd, cmd_value);
    if (ret != 0) {
        lpi->ps_enable = 0;
        D("[PS][CM3628-AD3] P-sensor i2c err, enable interrupt error\n");
    }
}

void disable_ps_int(void)/*disable ps interrupt*/
{
    struct cm3628_info *lpi = lp_info;

    _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_cmd_cmd,  lpi->ps_conf1_val);
}


static void sensor_irq_do_work(struct work_struct *work)
{
    struct cm3628_info *lpi = lp_info;

    unsigned char add = 0;

    /*check ALS or PS*/
    _cm3628_I2C_Read_Byte(lpi->check_interrupt_add, &add);
    /*D("[PS][CM3628-AD3] %s: check_interrupt_add = 0x%03X, add = 0x%x , add>>1 = 0x%x\n",
      __func__, lpi->check_interrupt_add, add, add>>1);*/
    add = add>>1;
    if (add == lpi->ls_slave_addr ) {
        report_lsensor_input_event(lpi, 0);
    } else if (add == lpi->ps_slave_addr) {
        report_psensor_input_event(lpi, 1);
    } else
        pr_err("[PS][CM3628-AD3 error]%s error: unkown interrupt: 0x%x!\n",
                __func__, add);

    enable_irq(lpi->irq);
}

#ifdef POLLING_PROXIMITY
static unsigned char mid_value(unsigned char value[], unsigned char size)
{
    int i = 0, j = 0;
    unsigned short temp = 0;

    if (size < 3)
        return 0;

    for (i = 0; i < (size - 1); i++)
        for (j = (i + 1); j < size; j++)
            if (value[i] > value[j]) {
                temp = value[i];
                value[i] = value[j];
                value[j] = temp;
            }
    return value[((size - 1) / 2)];
}

static int get_stable_ps_adc_value(unsigned char *ps_adc)
{
    unsigned char value[3] = {0, 0, 0}, mid_val = 0;
    int ret = 0;
    int i = 0;
    int wait_count = 0;
    struct cm3628_info *lpi = lp_info;

    for (i = 0; i < 3; i++) {
        /*wait interrupt GPIO high*/
        while (gpio_get_value(lpi->intr_pin) == 0) {
            msleep(10);
            wait_count++;
            if (wait_count > 12) {
                pr_err("[PS_ERR][CM3628-AD3 error]%s: interrupt GPIO low,"
                        " get_ps_adc_value\n", __func__);
                return -EIO;
            }
        }

        ret = get_ps_adc_value(&value[i]);
        if (ret < 0) {
            pr_err("[PS_ERR][CM3628-AD3 error]%s: get_ps_adc_value\n",
                    __func__);
            return -EIO;
        }

        if (wait_count < 60/10) {/*wait gpio less than 60ms*/
            msleep(60 - (10*wait_count));
        }
        wait_count = 0;
    }

    /*D("Sta_ps: Before sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
      value[0], value[1], value[2]);*/
    mid_val = mid_value(value, 3);
    /*D("Sta_ps: After sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
      value[0], value[1], value[2]);*/
    *ps_adc = (mid_val & 0xFF);

    return 0;
}

static void polling_do_work(struct work_struct *w)
{
    struct cm3628_info *lpi = lp_info;
    unsigned char ps_adc = 0;
    int i = 0;
    int ret = 0;

    /*D("lpi->ps_enable = %d\n", lpi->ps_enable);*/
    if (lpi->ps_enable == 0)
        return;

    ret = get_stable_ps_adc_value(&ps_adc);
    /*D("[CM3628-AD3] polling: ps_adc = 0x%02X, ps_next_base_value = 0x%02X,"
      " ps_thd_set = 0x%02X\n",
      ps_adc, lpi->mapping_table[lpi->ps_base_index],
      lpi->ps_thd_set);*/

    if ((ps_adc == 0) || (ret < 0)) {
        queue_delayed_work(lpi->lp_wq, &polling_work,
                msecs_to_jiffies(POLLING_DELAY));
        return;
    }

    for (i = lpi->ps_base_index; i >= 1; i--) {
        if (ps_adc > lpi->mapping_table[i])
            break;
        else if ((ps_adc > lpi->mapping_table[(i-1)]) &&
                (ps_adc <= lpi->mapping_table[i])) {
            lpi->ps_base_index = (i-1);

            if (i == (lpi->mapping_size - 1))
                lpi->ps_thd_set = 0xFF;
            else
                lpi->ps_thd_set = (lpi->mapping_table[i] +
                        TH_ADD);

            /* settng command code(0x01) = 0x03*/
            _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
                    PS_thd, lpi->ps_thd_set);
            D("[PS][CM3628-AD3] SET THD: lpi->ps_thd_set = %d\n",
                    lpi->ps_thd_set);
            break;
        }
    }

    queue_delayed_work(lpi->lp_wq, &polling_work,
            msecs_to_jiffies(POLLING_DELAY));
}
#endif

static irqreturn_t cm3628_irq_handler(int irq, void *data)
{
    struct cm3628_info *lpi = data;

    disable_irq_nosync(lpi->irq);
    if (enable_log)
        D("[PS][CM3628-AD3] %s\n", __func__);

    queue_work(lpi->lp_wq, &sensor_irq_work);

    return IRQ_HANDLED;
}

static int als_power(int enable)
{
    struct cm3628_info *lpi = lp_info;

    if (lpi->power)
        lpi->power(LS_PWR_ON, 1);

    return 0;
}

static void ls_initial_cmd(struct cm3628_info *lpi)
{
    char cmd = 0;
    cmd = (lpi->ls_cmd |
            CM3628_ALS_BIT2_Default_1 | CM3628_ALS_SD);
    /*must disable l-sensor interrupt befrore IST create*//*disable ALS func*/
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr,
            ALS_cmd_cmd, cmd);
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr,
            ALS_high_thd_msb, 0x00);
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr,
            ALS_high_thd_lsb, 0x00);
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr,
            ALS_low_thd_msb, 0x00);
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr,
            ALS_low_thd_lsb, 0x00);
}
static void psensor_intelligent_cancel_cmd(struct cm3628_info *lpi)
{/*for ps calibration*/
    _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_cancel, lpi->inte_cancel_set);
}
static void psensor_initial_cmd(struct cm3628_info *lpi)
{
    _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_cmd_cmd, lpi->ps_conf1_val);
    _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_thd, lpi->ps_thd_set);
    /*settng command code(0x01) = 0x03*/
    psensor_intelligent_cancel_cmd(lpi);
    _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_default, lpi->ps_conf2_val);/*for integration time*/
    D("[PS][CM3628-AD3] %s, finish\n", __func__);
}

static int psensor_enable(struct cm3628_info *lpi)
{
    int ret;
    char cmd = 0;
#ifdef POLLING_PROXIMITY
    unsigned char ps_adc = 0;
#endif

    D("[PS][CM3628-AD3] %s\n", __func__);
    if (lpi->ps_enable) {
        D("[PS][CM3628-AD3] %s: already enabled\n", __func__);
        return 0;
    }
    lpi->j_start = jiffies;
    /*D("%s: j_start = %lu", __func__, lpi->j_start);*/

    /* dummy report */
    input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, -1);
    input_sync(lpi->ps_input_dev);

    psensor_initial_cmd(lpi);

    if (lpi->enable_polling_ignore == 1 &&
            lpi->mfg_mode != NO_IGNORE_BOOT_MODE) {
        /* default report FAR */
        input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 1);
        input_sync(lpi->ps_input_dev);
    } else
        report_psensor_input_event(lpi, 0);

    cmd = ( lpi->ps_conf1_val | CM3628_PS_INT_EN);
    enable_ps_int(cmd);

    ret = irq_set_irq_wake(lpi->irq, 1);
    if (ret < 0) {
        pr_err(
                "[PS][CM3628-AD3 error]%s: fail to enable irq %d as wake interrupt\n",
                __func__, lpi->irq);
        return ret;
    }

#ifdef POLLING_PROXIMITY
    if (lpi->enable_polling_ignore == 1) {
        if (lpi->mfg_mode != NO_IGNORE_BOOT_MODE) {
            ret = get_stable_ps_adc_value(&ps_adc);
            D("[PS][CM3628-AD3] INITIAL ps_adc = 0x%02X", ps_adc);
            if ((ret == 0) && (lpi->mapping_table != NULL) &&
                    ((ps_adc >= lpi->ps_thd_set - 1)))
                queue_delayed_work(lpi->lp_wq, &polling_work,
                        msecs_to_jiffies(POLLING_DELAY));
        }
    }
#endif
    return ret;
}
static int psensor_set_disable(struct cm3628_info *lpi)
{
    int ret = -EIO;
    unsigned char add = 0;

    ret = _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_cmd_cmd, lpi->ps_conf1_val|CM3628_PS_SD);
    if (ret < 0) {
        pr_err("[PS][CM3628-AD3 error]%s: disable psensor fail\n", __func__);
        _cm3628_I2C_Read_Byte(lpi->check_interrupt_add, &add);
        pr_err("[PS][CM3628-AD3 error] %s: check_interrupt_add = 0x%03X, add = 0x%x , add>>1 = 0x%x\n",
                __func__, lpi->check_interrupt_add, add, add>>1);
    }
    return ret;
}
static int psensor_disable(struct cm3628_info *lpi)
{
    int ret = -EIO;
    unsigned char retry_count = 0;

    lpi->ps_pocket_mode = 0;
    D("[PS][CM3628-AD3] %s\n", __func__);
    if (!lpi->ps_enable) {
        D("[PS][CM3628-AD3] %s: already disabled\n", __func__);
        return 0;
    }

    ret = irq_set_irq_wake(lpi->irq, 0);
    if (ret < 0) {
        pr_err(
                "[PS][CM3628-AD3 error]%s: fail to disable irq %d as wake interrupt\n",
                __func__, lpi->irq);
        return ret;
    }
    while (1) {
        ret = psensor_set_disable (lpi);
        retry_count++;
        if (ret >= 0 || retry_count == 3) {
            break;
        }
    }
    if (ret < 0) {
        pr_err("[PS][CM3628-AD3 error]%s: retry disable psensor fail\n", __func__);
        return ret;
    }

    lpi->ps_enable = 0;

#ifdef POLLING_PROXIMITY
    if (lpi->enable_polling_ignore == 1 && lpi->mfg_mode != NO_IGNORE_BOOT_MODE) {
        cancel_delayed_work(&polling_work);
        lpi->ps_base_index = (lpi->mapping_size - 1);

        lpi->ps_thd_set = lpi->original_ps_thd_set;
        /* settng command code(0x01) = 0x03*/
        _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
                PS_thd, lpi->ps_thd_set);
    }
#endif
    return ret;
}

static int psensor_open(struct inode *inode, struct file *file)
{
    struct cm3628_info *lpi = lp_info;

    D("[PS][CM3628-AD3] %s\n", __func__);

    if (lpi->psensor_opened)
        return -EBUSY;

    lpi->psensor_opened = 1;

    return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
    struct cm3628_info *lpi = lp_info;

    D("[PS][CM3628-AD3] %s\n", __func__);

    lpi->psensor_opened = 0;

    return psensor_disable(lpi);
}

static long psensor_ioctl(struct file *file, unsigned int cmd,
        unsigned long arg)
{
    int val;
    struct cm3628_info *lpi = lp_info;

    D("[PS][CM3628-AD3] %s cmd %d\n", __func__, _IOC_NR(cmd));

    switch (cmd) {
        case CAPELLA_CM3602_IOCTL_ENABLE:
            if (get_user(val, (unsigned long __user *)arg))
                return -EFAULT;
            if (val)
                return psensor_enable(lpi);
            else
                return psensor_disable(lpi);
            break;
        case CAPELLA_CM3602_IOCTL_GET_ENABLED:
            return put_user(lpi->ps_enable, (unsigned long __user *)arg);
            break;
        default:
            pr_err("[PS][CM3628-AD3 error]%s: invalid cmd %d\n",
                    __func__, _IOC_NR(cmd));
            return -EINVAL;
    }
}

static const struct file_operations psensor_fops = {
    .owner = THIS_MODULE,
    .open = psensor_open,
    .release = psensor_release,
    .unlocked_ioctl = psensor_ioctl
};

struct miscdevice psensor_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "cm3602",
    .fops = &psensor_fops
};

void lightsensor_set_kvalue(struct cm3628_info *lpi)
{
    if (!lpi) {
        pr_err("[LS][CM3628-AD3 error]%s: ls_info is empty\n", __func__);
        return;
    }

    D("[LS][CM3628-AD3] %s: ALS calibrated als_kadc=0x%x\n",
            __func__, als_kadc);

    D("[PS][CM3628-AD3] lpi->als_kadc = 0x%x\n", lpi->als_kadc); 
    D("[PS][CM3628-AD3] lpi->golden_adc = 0x%x\n", lpi->golden_adc);
    if (als_kadc >> 16 == ALS_CALIBRATED)
        lpi->als_kadc = als_kadc & 0xFFFF;
    else {
        lpi->als_kadc = 0;
        D("[LS][CM3628-AD3] %s: no ALS calibrated\n", __func__);
    }

    if (lpi->als_kadc && lpi->golden_adc > 0) {
        lpi->als_kadc = (lpi->als_kadc > 0 && lpi->als_kadc < 0x1000) ?
            lpi->als_kadc : lpi->golden_adc;
        lpi->als_gadc = lpi->golden_adc;
    } else {
        lpi->als_kadc = 1;
        lpi->als_gadc = 1;
    }
    D("[LS][CM3628-AD3] %s: als_kadc=0x%x, als_gadc=0x%x\n",
            __func__, lpi->als_kadc, lpi->als_gadc);
}

void psensor_set_kvalue(struct cm3628_info *lpi)
{
    unsigned char iteration = 0;
    D("[PS][CM3628-AD3] %s: PS calibrated ps_kparam1 = 0x%x, ps_kparam2 = 0x%x\n",
            __func__, ps_kparam1, ps_kparam2);
    /*only use ps_kparam2 for cm3628*/
    if (ps_kparam1 >> 16 == PS_CALIBRATED) {
        iteration = (unsigned char)((ps_kparam2>>8) & 0xFF);
        lpi->inte_cancel_set = (unsigned char)(ps_kparam2 & 0xFF);
        D("[PS][CM3628-AD3] %s: PS calibrated inte_cancel_set = 0x%x\n",
                __func__, lpi->inte_cancel_set);
    } else {
        if (lpi->ps_calibration_rule >= 1) {
            lpi->ps_thd_set = lpi->ps_thd_no_cal;
            D("[PS][CM3628-AD3] %s: Proximity change threshold %d, no calibration\n",
                    __func__, lpi->ps_thd_set);
        }
        D("[PS][CM3628-AD3] %s: Proximity not calibrated\n", __func__);
    }

}

static int lightsensor_update_table(struct cm3628_info *lpi)
{
    unsigned short data[10];
    int i;
    for (i = 0; i < 10; i++) {
        if (*(lpi->adc_table + i) < 0xFFFF) {
            data[i] = *(lpi->adc_table + i)
                * lpi->als_kadc / lpi->als_gadc;
        } else {
            data[i] = *(lpi->adc_table + i);
        }
        D("[LS][CM3628-AD3] %s: Calibrated adc_table: data[%d], %x\n",
                __func__, i, data[i]);
    }
    memcpy(lpi->cali_table, data, 20);
    return 0;
}

static int lightsensor_enable(struct cm3628_info *lpi)
{
    int ret = 0;
    unsigned char cmd = 0;

    mutex_lock(&als_enable_mutex);
    D("[LS][CM3628-AD3] %s\n", __func__);

    cmd = (CM3628_ALS_IT_50ms | CM3628_ALS_PERS_1 |
            CM3628_ALS_BIT2_Default_1);
    ret = _cm3628_I2C_Write_Byte(lpi->ls_slave_addr, ALS_cmd_cmd, cmd);
    if (ret < 0)
        pr_err(
                "[LS][CM3628-AD3 error]%s: set auto light sensor fail\n",
                __func__);
    else {
        msleep(50);/*wait for 50 ms for the first report adc*/
        /* report an invalid value first to ensure we
         * trigger an event when adc_level is zero.
         */
        input_report_abs(lpi->ls_input_dev, ABS_MISC, -1);
        input_sync(lpi->ls_input_dev);
        report_lsensor_input_event(lpi, 1);/*resume, IOCTL and DEVICE_ATTR*/
    }

    mutex_unlock(&als_enable_mutex);
    return ret;
}

static int lightsensor_disable(struct cm3628_info *lpi)
{
    int ret = 0;
    char cmd = 0;

    mutex_lock(&als_disable_mutex);

    D("[LS][CM3628-AD3] %s\n", __func__);

    cmd = (CM3628_ALS_BIT2_Default_1 | CM3628_ALS_SD);
    ret = _cm3628_I2C_Write_Byte(lpi->ls_slave_addr, ALS_cmd_cmd, cmd);
    if (ret < 0)
        pr_err("[LS][CM3628-AD3 error]%s: disable auto light sensor fail\n",
                __func__);
    else
        lpi->als_enable = 0;

    mutex_unlock(&als_disable_mutex);
    return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
    struct cm3628_info *lpi = lp_info;
    int rc = 0;

    D("[LS][CM3628-AD3] %s\n", __func__);
    if (lpi->lightsensor_opened) {
        pr_err("[LS][CM3628-AD3 error]%s: already opened\n", __func__);
        rc = -EBUSY;
    }
    lpi->lightsensor_opened = 1;
    return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
    struct cm3628_info *lpi = lp_info;

    D("[LS][CM3628-AD3] %s\n", __func__);
    lpi->lightsensor_opened = 0;
    return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
        unsigned long arg)
{
    int rc, val;
    struct cm3628_info *lpi = lp_info;

    /*D("[CM3628-AD3] %s cmd %d\n", __func__, _IOC_NR(cmd));*/

    switch (cmd) {
        case LIGHTSENSOR_IOCTL_ENABLE:
            if (get_user(val, (unsigned long __user *)arg)) {
                rc = -EFAULT;
                break;
            }
            D("[LS][CM3628-AD3] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
                    __func__, val);
            rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
            break;
        case LIGHTSENSOR_IOCTL_GET_ENABLED:
            val = lpi->als_enable;
            D("[LS][CM3628-AD3] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
                    __func__, val);
            rc = put_user(val, (unsigned long __user *)arg);
            break;
        default:
            pr_err("[LS][CM3628-AD3 error]%s: invalid cmd %d\n",
                    __func__, _IOC_NR(cmd));
            rc = -EINVAL;
    }

    return rc;
}

static const struct file_operations lightsensor_fops = {
    .owner = THIS_MODULE,
    .open = lightsensor_open,
    .release = lightsensor_release,
    .unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "lightsensor",
    .fops = &lightsensor_fops
};


static ssize_t ps_adc_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{

    unsigned char value = 0;
    int ret;
    struct cm3628_info *lpi = lp_info;
    int value1;

    value1 = gpio_get_value(lpi->intr_pin);

    ret = get_ps_adc_value(&value);
    if (lpi->ps_calibration_rule == 1) {
        D("[PS][CM3628-AD3] %s: ps ADC value=0x%x, adc offset 0x%x\n",
                __func__, value, lpi->ps_adc_offset);
        if (value >= lpi->ps_adc_offset )
            value = value -lpi->ps_adc_offset;
        else
            value = 0;
    } else if (lpi->ps_calibration_rule == 2 && lpi->ps_adc_offset > 0) {/*for saga*/
        D("[PS][CM3628-AD3] %s: ps ADC value=0x%x, adc offset 0x%x, adc offset2 0x%x,\n",
                __func__, value, lpi->ps_adc_offset,  lpi->ps_adc_offset2);
        if (value < 20 ) {
            if (value >= lpi->ps_adc_offset )
                value = value -lpi->ps_adc_offset;
            else
                value = 0;
        } else
            value = value - lpi->ps_adc_offset2;
    }

    ret = sprintf(buf, "ADC[0x%03X], ENABLE = %d, intr_pin = %d,"
            " ps_pocket_mode = %d\n", value, lpi->ps_enable, value1,
            lpi->ps_pocket_mode);

    return ret;
}

static ssize_t ps_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int ps_en;
    struct cm3628_info *lpi = lp_info;

    ps_en = -1;
    sscanf(buf, "%d", &ps_en);

    if (ps_en != 0 && ps_en != 1
            && ps_en != 10 && ps_en != 13 && ps_en != 16)
        return -EINVAL;

    if (ps_en) {
        D("[PS][CM3628-AD3] %s: ps_en=%d\n",
                __func__, ps_en);
        psensor_enable(lpi);
    } else
        psensor_disable(lpi);

    D("[PS][CM3628-AD3] %s\n", __func__);

    return count;
}

static DEVICE_ATTR(ps_adc, 0664, ps_adc_show, ps_enable_store);

unsigned PS_cmd_test_value;
static ssize_t ps_parameters_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{

    int ret;
    struct cm3628_info *lpi = lp_info;

    ret = sprintf(buf, "PS THD reg [0x%x]= 0x%x, PS Cmd reg [0x%x]= 0x%x\n",
            PS_thd, lpi->ps_thd_set,
            PS_cmd_cmd, PS_cmd_test_value);

    return ret;
}

static ssize_t ps_parameters_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{

    struct cm3628_info *lpi = lp_info;
    char *token[10];
    int i;

    printk(KERN_INFO "[PS][CM3628-AD3] %s\n", buf);
    for (i = 0; i < 2; i++)
        token[i] = strsep((char **)&buf, " ");

    lpi->ps_thd_set = simple_strtoul(token[0], NULL, 16);
    PS_cmd_test_value = simple_strtoul(token[1], NULL, 16);
    printk(KERN_INFO
            "[PS][CM3628-AD3]Set lpi->ps_thd_set = 0x%x, PS_cmd_cmd:value = 0x%x\n",
            lp_info->ps_thd_set, PS_cmd_test_value);
    enable_ps_int(PS_cmd_test_value);

    D("[PS][CM3628-AD3] %s\n", __func__);

    return count;
}

static DEVICE_ATTR(ps_parameters, 0664,
        ps_parameters_show, ps_parameters_store);

static ssize_t ps_kadc_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret = 0;
    struct cm3628_info *lpi = lp_info;

    if (ps_kparam1 >> 16 == PS_CALIBRATED)
        ret = sprintf(buf, "(P-sensor calibrated,"
                "intelligent cancellation level)"
                " = (0x%x)\n",
                lpi->inte_cancel_set);
    else
        ret = sprintf(buf, "(P-sensor no calibrated,"
                "intelligent cancellation level)"
                " = (0x%x)\n",
                lpi->inte_cancel_set);

    return ret;
}

static ssize_t ps_kadc_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int param1, param2;
    int value = 0;
    struct cm3628_info *lpi = lp_info;

    sscanf(buf, "0x%x 0x%x", &param1, &param2);

    D("[PS]%s: store value = 0x%X, 0x%X\n",
            __func__, param1, param2);
    value = (param2>>8) & 0xFF;
    if (lpi->ps_calibration_rule >= 1 || lpi->ps_reset_thd == 1) {
        lpi->ps_thd_set = lpi->ps_thd_with_cal;
        D("[PS][CM3628-AD3] %s: Proximity change threshold %d, after calibration\n",
                __func__, lpi->ps_thd_set);
    }
    if (lpi->ps_enable)
        enable_ps_int(lpi->ps_conf1_val |CM3628_PS_INT_EN);

    lpi->inte_cancel_set = (param2 & 0xFF);
    ps_cancel_set = lpi->inte_cancel_set;
    psensor_intelligent_cancel_cmd(lpi);

    D("[PS]%s: lpi ->inte_cancel_set = 0x%X\n",
            __func__, lpi->inte_cancel_set);

    return count;
}

static DEVICE_ATTR(ps_kadc, 0664, ps_kadc_show, ps_kadc_store);

static ssize_t ps_conf2_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret = 0;
    struct cm3628_info *lpi = lp_info;

    ret = sprintf(buf, "PS_CONF2 = 0x%x\n", lpi->ps_conf2_val);

    return ret;
}

static ssize_t ps_conf2_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int code;
    struct cm3628_info *lpi = lp_info;

    sscanf(buf, "0x%x", &code);

    D("[PS]%s: store value = 0x%x\n", __func__, code);

    lpi->ps_conf2_val = code;
    _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_default, lpi->ps_conf2_val);/*for integration time*/

    D("[PS]%s: PS_CONF2 = 0x%x\n", __func__, lpi->ps_conf2_val);

    return count;
}

static DEVICE_ATTR(ps_conf2, 0664, ps_conf2_show, ps_conf2_store);

static unsigned char PS_CONF1 = 0;
static ssize_t ps_conf1_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "PS_CONF1 = 0x%x\n", PS_CONF1);
}
static ssize_t ps_conf1_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int code;
    struct cm3628_info *lpi = lp_info;

    sscanf(buf, "0x%x", &code);

    D("[PS]%s: store value = 0x%x\n", __func__, code);

    PS_CONF1 = code;
    _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_cmd_cmd, PS_CONF1);

    D("[PS]%s: PS_CONF1 = 0x%x\n", __func__, PS_CONF1);

    return count;
}
static DEVICE_ATTR(ps_conf1, 0664, ps_conf1_show, ps_conf1_store);

static unsigned char PS_THD = 0;
static ssize_t ps_thd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    enable_log = !enable_log;
    return sprintf(buf, "PS_THD = 0x%x\n", PS_THD);
}
static ssize_t ps_thd_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int code;
    struct cm3628_info *lpi = lp_info;

    sscanf(buf, "0x%x", &code);

    D("[PS]%s: store value = 0x%x\n", __func__, code);

    PS_THD = code;
    _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_thd, PS_THD);

    D("[PS]%s: PS_THD = 0x%x\n", __func__, PS_THD);

    return count;
}
static DEVICE_ATTR(ps_thd, 0664, ps_thd_show, ps_thd_store);


static ssize_t ps_canc_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret = 0;
    struct cm3628_info *lpi = lp_info;

    ret = sprintf(buf, "PS_CANC = 0x%x\n", lpi->inte_cancel_set);

    return ret;
}
static ssize_t ps_canc_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int code;
    struct cm3628_info *lpi = lp_info;

    sscanf(buf, "0x%x", &code);

    D("[PS]%s: store value = 0x%x\n", __func__, code);
    if ((lpi->mfg_mode == NO_IGNORE_BOOT_MODE) && code == 0 &&
            lpi->ps_reset_thd) {
        lpi->ps_thd_set = 0xFE;
        D("[PS]%s: change ps_thd_set = 0x%x for calibration\n", __func__, lpi->ps_thd_set);
        _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
                PS_thd, lpi->ps_thd_set);
    }
    lpi->inte_cancel_set = (unsigned char)code;
    psensor_intelligent_cancel_cmd(lpi);

    D("[PS]%s: PS_CANC = 0x%x\n", __func__, lpi->inte_cancel_set);

    return count;
}
static DEVICE_ATTR(ps_canc, 0664, ps_canc_show, ps_canc_store);

static ssize_t ps_hw_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret = 0;
    struct cm3628_info *lpi = lp_info;


    ret = sprintf(buf, "PS: reg 0x00 = 0x%x, reg 0x01 = 0x%x , reg 0x02 = 0x%x , reg 0x03 = 0x%x, ps_adc_offset= 0x%x, LS: reg 0x00 = 0x%x\n",
            lpi->ps_conf1_val, lpi->ps_thd_set, lpi->inte_cancel_set, lpi->ps_conf2_val,
            lpi->ps_adc_offset, lpi->ls_cmd);

    return ret;
}
static ssize_t ps_hw_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int code;
    struct cm3628_info *lpi = lp_info;

    sscanf(buf, "0x%x", &code);

    D("[PS]%s: store value = 0x%x\n", __func__, code);
    if (code == 1) {
        lpi->inte_cancel_set = 0;
        lpi->ps_adc_offset = 0;
        lpi->ps_adc_offset2 = 0;
        psensor_intelligent_cancel_cmd(lpi);
        D("[PS]%s: reset cancellation =%d,  adc offset = %d,  and adc offset2 = %d\n",
                __func__, lpi->inte_cancel_set, lpi->ps_adc_offset,
                lpi->ps_adc_offset2);
    } else {
        lpi->inte_cancel_set = ps_cancel_set;
        lpi->ps_adc_offset = ps_offset_adc;
        lpi->ps_adc_offset2 = ps_offset_adc2;
        psensor_intelligent_cancel_cmd(lpi);
        if ((lpi->mfg_mode == NO_IGNORE_BOOT_MODE) && lpi->ps_reset_thd) {
            lpi->ps_thd_set = lpi->ps_thd_with_cal;
            _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
                    PS_thd, lpi->ps_thd_set);
        }
        D("[PS]%s: recover cancellation =%d,  adc offset = %d,  and adc offset2 = %d\n",
                __func__, lpi->inte_cancel_set, lpi->ps_adc_offset,
                lpi->ps_adc_offset2);
    }

    return count;
}
static DEVICE_ATTR(ps_hw, 0664, ps_hw_show, ps_hw_store);

static ssize_t ls_adc_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret;
    struct cm3628_info *lpi = lp_info;

    /*because 3628 is interrupt mode*/
    report_lsensor_input_event(lpi, 0);

    D("[LS][CM3628-AD3] %s: ADC = 0x%04X, Level = %d \n",
            __func__, lpi->current_adc, lpi->current_level);
    ret = sprintf(buf, "ADC[0x%04X] => level %d\n",
            lpi->current_adc, lpi->current_level);

    return ret;
}

static DEVICE_ATTR(ls_adc, 0664, ls_adc_show, NULL);

static ssize_t ls_enable_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    /*Todo: becase CM3628 registers can't be read, need to think ho w*/
    int ret = 0;
    struct cm3628_info *lpi = lp_info;

    ret = sprintf(buf, "Light sensor Auto Enable = %d\n",
            lpi->als_enable);

    return ret;
}

static ssize_t ls_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int ret = 0;
    int ls_auto;
    struct cm3628_info *lpi = lp_info;

    ls_auto = -1;
    sscanf(buf, "%d", &ls_auto);

    if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
        return -EINVAL;

    if (ls_auto) {
        lpi->ls_calibrate = (ls_auto == 147) ? 1 : 0;
        ret = lightsensor_enable(lpi);
    } else {
        lpi->ls_calibrate = 0;
        ret = lightsensor_disable(lpi);
    }

    D("[LS][CM3628-AD3] %s: lpi->als_enable = %d, lpi->ls_calibrate = %d, ls_auto=%d\n",
            __func__, lpi->als_enable, lpi->ls_calibrate, ls_auto);

    if (ret < 0)
        pr_err(
                "[LS][CM3628-AD3 error]%s: set auto light sensor fail\n",
                __func__);

    return count;
}

static DEVICE_ATTR(ls_auto, 0664,
        ls_enable_show, ls_enable_store);

static ssize_t ls_kadc_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct cm3628_info *lpi = lp_info;
    int ret;

    ret = sprintf(buf, "kadc = 0x%x, gadc = 0x%x, kadc while this boot"
            " = 0x%x\n",
            lpi->als_kadc, lpi->als_gadc, als_kadc);

    return ret;
}

static ssize_t ls_kadc_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct cm3628_info *lpi = lp_info;
    int kadc_temp = 0;

    sscanf(buf, "%d", &kadc_temp);
    if (kadc_temp <= 0 || lpi->golden_adc <= 0) {
        printk(KERN_ERR "[LS][CM3628-AD3 error] %s: kadc_temp=0x%x, als_gadc=0x%x\n",
                __func__, kadc_temp, lpi->golden_adc);
        return -EINVAL;
    }
    mutex_lock(&als_get_adc_mutex);
    lpi->als_kadc = kadc_temp;
    lpi->als_gadc = lpi->golden_adc;
    printk(KERN_INFO "[LS]%s: als_kadc=0x%x, als_gadc=0x%x\n",
            __func__, lpi->als_kadc, lpi->als_gadc);

    if (lightsensor_update_table(lpi) < 0)
        printk(KERN_ERR "[LS][CM3628-AD3 error] %s: update ls table fail\n", __func__);
    mutex_unlock(&als_get_adc_mutex);
    return count;
}

static DEVICE_ATTR(ls_kadc, 0664, ls_kadc_show, ls_kadc_store);

static ssize_t ls_adc_table_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned length = 0;
    int i;

    for (i = 0; i < 10; i++) {
        length += sprintf(buf + length,
                "[CM3628-AD3]Get adc_table[%d] =  0x%x ; %d, Get cali_table[%d] =  0x%x ; %d, \n",
                i, *(lp_info->adc_table + i),
                *(lp_info->adc_table + i),
                i, *(lp_info->cali_table + i),
                *(lp_info->cali_table + i));
    }
    return length;
}

static ssize_t ls_adc_table_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{

    struct cm3628_info *lpi = lp_info;
    char *token[10];
    unsigned short tempdata[10];
    int i;

    printk(KERN_INFO "[LS][CM3628-AD3]%s\n", buf);
    for (i = 0; i < 10; i++) {
        token[i] = strsep((char **)&buf, " ");
        tempdata[i] = simple_strtoul(token[i], NULL, 16);
        if (tempdata[i] < 1 || tempdata[i] > 0xffff) {
            printk(KERN_ERR
                    "[LS][CM3628-AD3 error] adc_table[%d] =  0x%x Err\n",
                    i, tempdata[i]);
            return count;
        }
    }
    mutex_lock(&als_get_adc_mutex);
    for (i = 0; i < 10; i++) {
        lpi->adc_table[i] = tempdata[i];
        printk(KERN_INFO
                "[LS][CM3628-AD3]Set lpi->adc_table[%d] =  0x%x\n",
                i, *(lp_info->adc_table + i));
    }
    if (lightsensor_update_table(lpi) < 0)
        printk(KERN_ERR "[LS][CM3628-AD3 error] %s: update ls table fail\n",
                __func__);
    mutex_unlock(&als_get_adc_mutex);
    D("[LS][CM3628-AD3] %s\n", __func__);

    return count;
}

static DEVICE_ATTR(ls_adc_table, 0664,
        ls_adc_table_show, ls_adc_table_store);

static unsigned char ALS_CONF1 = 0;
static ssize_t ls_conf1_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "ALS_CONF1 = %x\n", ALS_CONF1);
}
static ssize_t ls_conf1_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct cm3628_info *lpi = lp_info;
    int value = 0;
    sscanf(buf, "0x%x", &value);

    ALS_CONF1 = value;
    printk(KERN_INFO "[LS]set ALS_CONF1 = %x\n", ALS_CONF1);
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr, ALS_cmd_cmd, ALS_CONF1);
    return count;
}
static DEVICE_ATTR(ls_conf1, 0664, ls_conf1_show, ls_conf1_store);

static unsigned char ALS_CONF2 = 0;
static ssize_t ls_conf2_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "ALS_CONF2 = %x\n", ALS_CONF2);
}
static ssize_t ls_conf2_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct cm3628_info *lpi = lp_info;
    int value = 0;
    sscanf(buf, "0x%x", &value);

    ALS_CONF2 = value;
    printk(KERN_INFO "[LS]set ALS_CONF2 = %x\n", ALS_CONF2);
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr, ALS_average, ALS_CONF2);
    return count;
}
static DEVICE_ATTR(ls_conf2, 0664, ls_conf2_show, ls_conf2_store);

static ssize_t ls_fLevel_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "fLevel = %d\n", fLevel);
}
static ssize_t ls_fLevel_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct cm3628_info *lpi = lp_info;
    int value=0;
    sscanf(buf, "%d", &value);
    (value>=0)?(value=min(value,10)):(value=max(value,-1));
    fLevel=value;
    input_report_abs(lpi->ls_input_dev, ABS_MISC, fLevel);
    input_sync(lpi->ls_input_dev);
    printk(KERN_INFO "[LS]set fLevel = %d\n", fLevel);

    msleep(1000);
    fLevel=-1;
    return count;
}
static DEVICE_ATTR(ls_flevel, 0664, ls_fLevel_show, ls_fLevel_store);

static int lightsensor_setup(struct cm3628_info *lpi)
{
    int ret;

    lpi->ls_input_dev = input_allocate_device();
    if (!lpi->ls_input_dev) {
        pr_err(
                "[LS][CM3628-AD3 error]%s: could not allocate ls input device\n",
                __func__);
        return -ENOMEM;
    }
    lpi->ls_input_dev->name = "lightsensor-level";
    set_bit(EV_ABS, lpi->ls_input_dev->evbit);
    input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

    ret = input_register_device(lpi->ls_input_dev);
    if (ret < 0) {
        pr_err("[LS][CM3628-AD3 error]%s: can not register ls input device\n",
                __func__);
        goto err_free_ls_input_device;
    }

    ret = misc_register(&lightsensor_misc);
    if (ret < 0) {
        pr_err("[LS][CM3628-AD3 error]%s: can not register ls misc device\n",
                __func__);
        goto err_unregister_ls_input_device;
    }

    return ret;

err_unregister_ls_input_device:
    input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
    input_free_device(lpi->ls_input_dev);
    return ret;
}

static int psensor_setup(struct cm3628_info *lpi)
{
    int ret;

    lpi->ps_input_dev = input_allocate_device();
    if (!lpi->ps_input_dev) {
        pr_err(
                "[PS][CM3628-AD3 error]%s: could not allocate ps input device\n",
                __func__);
        return -ENOMEM;
    }
    lpi->ps_input_dev->name = "proximity";
    set_bit(EV_ABS, lpi->ps_input_dev->evbit);
    input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    ret = input_register_device(lpi->ps_input_dev);
    if (ret < 0) {
        pr_err(
                "[PS][CM3628-AD3 error]%s: could not register ps input device\n",
                __func__);
        goto err_free_ps_input_device;
    }

    ret = misc_register(&psensor_misc);
    if (ret < 0) {
        pr_err(
                "[PS][CM3628-AD3 error]%s: could not register ps misc device\n",
                __func__);
        goto err_unregister_ps_input_device;
    }

    return ret;

err_unregister_ps_input_device:
    input_unregister_device(lpi->ps_input_dev);
err_free_ps_input_device:
    input_free_device(lpi->ps_input_dev);
    return ret;
}


static int initial_cm3628(struct cm3628_info *lpi)
{
    int val, ret, fail_counter = 0;
    unsigned char add = 0;

    val = gpio_get_value(lpi->intr_pin);

    D("[PS][CM3628-AD3] %s, INTERRUPT GPIO val = %d\n", __func__, val);

check_interrupt_gpio:
    if (fail_counter > 0) {
        ret =_cm3628_I2C_Read_Byte(lpi->check_interrupt_add, &add);
        D("[PS][CM3628-AD3] %s, check_interrupt_add value = 0x%x, ret %d\n",
                __func__, add, ret);
    }
    ret = _cm3628_I2C_Write_Byte(lpi->ls_slave_addr,
            ALS_cmd_cmd, CM3628_ALS_BIT2_Default_1);/*0x04 bit[3:2] */
    if ((ret < 0) && (fail_counter < 10)) {
        fail_counter++;
        val = gpio_get_value(lpi->intr_pin);
        D("[LS][CM3628-AD3] %s, interrupt GPIO val = %d, , inital fail_counter %d\n",
                __func__, val, fail_counter);
        goto    check_interrupt_gpio;
    }
    ret = _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_cmd_cmd, 0x00);/*first init need 0x0*/
    if ((ret < 0) && (fail_counter < 10)) {
        fail_counter++;
        val = gpio_get_value(lpi->intr_pin);
        D("[PS][CM3628-AD3] %s, interrupt GPIO val = %d, , inital fail_counter %d\n",
                __func__, val, fail_counter);
        goto    check_interrupt_gpio;
    }
    if (fail_counter >= 10) {
        D("[PS][CM3628-AD3] %s, initial fail_counter = %d\n", __func__, fail_counter);
        if (record_init_fail == 0)
            record_init_fail = 1;
        return -ENOMEM;/*If devices without cm3628 chip and did not probe driver*/
    }

    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr,
            ALS_average, CM3628_ALS_RES);/*0x40 ALS_RES ALS reset enable*/
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr, ALS_high_thd_msb, 0x00);/*high Threshold*/
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr, ALS_high_thd_lsb, 0x00);
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr, ALS_low_thd_msb, 0x00);/*low Threshold*/
    _cm3628_I2C_Write_Byte(lpi->ls_slave_addr, ALS_low_thd_lsb, 0x00);
    _cm3628_I2C_Write_Byte(lpi->ps_slave_addr, PS_thd, 0x00);/*first init need 0x0*/
    _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_cancel, 0x00);/*first init need 0x0*/
    _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_default, 0x00);/*for integration time *//*first init need 0x0*/
    msleep(10);

    return 0;
}


static int cm3628_setup(struct cm3628_info *lpi)
{
    int ret = 0;

    als_power(1);
    msleep(5);
    ret = gpio_request(lpi->intr_pin, "gpio_cm3628_intr");
    if (ret < 0) {
        pr_err("[PS][CM3628-AD3 error]%s: gpio %d request failed (%d)\n",
                __func__, lpi->intr_pin, ret);
        return ret;
    }

    ret = gpio_direction_input(lpi->intr_pin);
    if (ret < 0) {
        pr_err(
                "[PS][CM3628-AD3 error]%s: fail to set gpio %d as input (%d)\n",
                __func__, lpi->intr_pin, ret);
        goto fail_free_intr_pin;
    }


    ret = initial_cm3628(lpi);
    if (ret < 0) {
        pr_err(
                "[PS_ERR][CM3628-AD3 error]%s: fail to initial cm3628 (%d)\n",
                __func__, ret);
        goto fail_free_intr_pin;
    }
    ls_initial_cmd(lpi);
    psensor_initial_cmd(lpi);
 
//   printk("[LS][CM3628-AD3] in cm3628_setup, Before _cm3628_I2C_Write_Byte\n");
 
    _cm3628_I2C_Write_Byte(lpi->ps_slave_addr,
            PS_cmd_cmd,
            lpi->ps_conf1_val |CM3628_PS_SD);
 
//     printk("[LS][CM3628-AD3] in cm3628_setup, After _cm3628_I2C_Write_Byte\n");
    /*Default disable P sensor*/

    lpi->led = 0;

    ret = request_any_context_irq(lpi->irq,
            cm3628_irq_handler,
            IRQF_TRIGGER_LOW,
            "cm3628",
            lpi);
    if (ret < 0) {
        pr_err(
                "[PS][CM3628-AD3 error]%s: req_irq(%d) fail for gpio %d (%d)\n",
                __func__, lpi->irq,
                lpi->intr_pin, ret);
        goto fail_free_intr_pin;
    }

    return ret;

fail_free_intr_pin:
    gpio_free(lpi->intr_pin);
    return ret;
}

static void cm3628_early_suspend(struct early_suspend *h)
{
    struct cm3628_info *lpi = lp_info;

    D("[LS][CM3628-AD3] %s\n", __func__);

    if (lpi->als_enable){
        lightsensor_disable(lpi); 
    }
    
   
}

static void cm3628_late_resume(struct early_suspend *h)
{
    struct cm3628_info *lpi = lp_info;

    D("[LS][CM3628-AD3] %s\n", __func__);

    if (!lpi->als_enable){
        lightsensor_enable(lpi);     
      }

      

}
static int cm3628_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret = 0;
    struct cm3628_info *lpi;
    struct cm3628_platform_data *pdata;

    D("[PS][CM3628-AD3] %s\n", __func__);


    lpi = kzalloc(sizeof(struct cm3628_info), GFP_KERNEL);
    if (!lpi)
        return -ENOMEM;

    /*D("[CM3628-AD3] %s: client->irq = %d\n", __func__, client->irq);*/

    lpi->i2c_client = client;
    pdata = client->dev.platform_data;
    if (!pdata) {
        pr_err("[PS][CM3628-AD3 error]%s: Assign platform_data error!!\n",
                __func__);
        ret = -EBUSY;
        goto err_platform_data_null;
    }
/******* modified start  *******/
    // Instead of set in intel-mid.c, irq value is set here.   
    lpi->irq =  gpio_to_irq(pdata->intr);

/******* modified end *******/

    /******** modified start  ********/ 
    //lpi->mfg_mode = board_mfg_mode(); //commented out 
      lpi->mfg_mode = NO_IGNORE_BOOT_MODE;
    /*******modified end  ************/
	i2c_set_clientdata(client, lpi);
    lpi->intr_pin = pdata->intr;
     D("[PS][CM3628-AD3] pdata->intr = 0x%x\n", pdata->intr);

    
    lpi->adc_table = pdata->levels;
    lpi->golden_adc = pdata->golden_adc;
    lpi->power = pdata->power;
    lpi->ls_slave_addr = pdata->ls_slave_addr;
    D("[PS][CM3628-AD3] pdata->ls_slave_addr = 0x%x\n", pdata->ls_slave_addr);


    lpi->ps_slave_addr = pdata->ps_slave_addr;
    D("[PS][CM3628-AD3] pdata->ps_slave_addr = 0x%x\n", pdata->ps_slave_addr);
    lpi->check_interrupt_add = pdata->check_interrupt_add;
    lpi->ps_thd_set = pdata->ps_thd_set;
    lpi->ps_conf2_val = pdata->ps_conf2_val;
    lpi->ps_calibration_rule = pdata->ps_calibration_rule;
    lpi->ps_conf1_val = pdata->ps_conf1_val;
    lpi->j_start = 0;
    lpi->j_end = 0;
    lpi->mapping_table = pdata->mapping_table;
    lpi->mapping_size = pdata->mapping_size;
    lpi->ps_base_index = (pdata->mapping_size - 1);
    lpi->enable_polling_ignore = pdata->enable_polling_ignore;
    lpi->ps_thd_no_cal = pdata->ps_thd_no_cal;
    lpi->ps_thd_with_cal  = pdata->ps_thd_with_cal;
    lpi->ls_cmd  = pdata->ls_cmd;
    lpi->ps_adc_offset = pdata->ps_adc_offset;
    lpi->ps_adc_offset2 = pdata->ps_adc_offset2;
    lpi->ps_debounce = pdata->ps_debounce;
    lpi->ps_delay_time = pdata->ps_delay_time;
    lpi->ps_reset_thd = pdata->ps_reset_thd;
    D("[PS][CM3628-AD3] %s: ls_cmd 0x%x, ps_adc_offset=0x%x, ps_debounce=0x%x\n",
            __func__, lpi->ls_cmd, lpi->ps_adc_offset, lpi->ps_debounce);
    if (pdata->ls_cmd == 0) {
        lpi->ls_cmd  = CM3628_ALS_IT_400ms | CM3628_ALS_PERS_4;
    }

    lp_info = lpi;

    mutex_init(&als_enable_mutex);
    mutex_init(&als_disable_mutex);
    mutex_init(&als_get_adc_mutex);

    ret = lightsensor_setup(lpi);
    if (ret < 0) {
        pr_err("[LS][CM3628-AD3 error]%s: lightsensor_setup error!!\n",
                __func__);
        goto err_lightsensor_setup;
    }

    ret = psensor_setup(lpi);
    if (ret < 0) {
        pr_err("[PS][CM3628-AD3 error]%s: psensor_setup error!!\n",
                __func__);
        goto err_psensor_setup;
    }

    lightsensor_set_kvalue(lpi);
    ret = lightsensor_update_table(lpi);
    if (ret < 0) {
        pr_err("[LS][CM3628-AD3 error]%s: update ls table fail\n",
                __func__);
        goto err_lightsensor_update_table;
    }

    lpi->lp_wq = create_singlethread_workqueue("cm3628_wq");
    if (!lpi->lp_wq) {
        pr_err("[PS][CM3628-AD3 error]%s: can't create workqueue\n", __func__);
        ret = -ENOMEM;
        goto err_create_singlethread_workqueue;
    }

    wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

    psensor_set_kvalue(lpi);

#ifdef POLLING_PROXIMITY
    if (lpi->enable_polling_ignore == 1)
        lpi->original_ps_thd_set = lpi->ps_thd_set;
#endif
    ret = cm3628_setup(lpi);
    if (ret < 0) {
        pr_err("[PS_ERR][CM3628-AD3 error]%s: cm3628_setup error!\n", __func__);
        goto err_cm3628_setup;
    }
    ps_cancel_set = lpi->inte_cancel_set;
    ps_offset_adc = lpi->ps_adc_offset;
    ps_offset_adc2 = lpi->ps_adc_offset2;
    lpi->cm3628_class = class_create(THIS_MODULE, "optical_sensors");
    if (IS_ERR(lpi->cm3628_class)) {
        ret = PTR_ERR(lpi->cm3628_class);
        lpi->cm3628_class = NULL;
        goto err_create_class;
    }

    lpi->ls_dev = device_create(lpi->cm3628_class,
            NULL, 0, "%s", "lightsensor");
    if (unlikely(IS_ERR(lpi->ls_dev))) {
        ret = PTR_ERR(lpi->ls_dev);
        lpi->ls_dev = NULL;
        goto err_create_ls_device;
    }

    /* register the attributes */
    ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc);
    if (ret)
        goto err_create_ls_device_file;

    /* register the attributes */
    ret = device_create_file(lpi->ls_dev, &dev_attr_ls_auto);
    if (ret)
        goto err_create_ls_device_file;

    /* register the attributes */
    ret = device_create_file(lpi->ls_dev, &dev_attr_ls_kadc);
    if (ret)
        goto err_create_ls_device_file;

    ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc_table);
    if (ret)
        goto err_create_ls_device_file;

    ret = device_create_file(lpi->ls_dev, &dev_attr_ls_conf1);
    if (ret)
        goto err_create_ls_device_file;

    ret = device_create_file(lpi->ls_dev, &dev_attr_ls_conf2);
    if (ret)
        goto err_create_ls_device_file;

    ret = device_create_file(lpi->ls_dev, &dev_attr_ls_flevel);
    if (ret)
        goto err_create_ls_device_file;

    lpi->ps_dev = device_create(lpi->cm3628_class,
            NULL, 0, "%s", "proximity");
    if (unlikely(IS_ERR(lpi->ps_dev))) {
        ret = PTR_ERR(lpi->ps_dev);
        lpi->ps_dev = NULL;
        goto err_create_ls_device_file;
    }

    /* register the attributes */
    ret = device_create_file(lpi->ps_dev, &dev_attr_ps_adc);
    if (ret)
        goto err_create_ps_device;

    ret = device_create_file(lpi->ps_dev,
            &dev_attr_ps_parameters);
    if (ret)
        goto err_create_ps_device;

    /* register the attributes */
    ret = device_create_file(lpi->ps_dev, &dev_attr_ps_kadc);
    if (ret)
        goto err_create_ps_device;

    /* register the attributes */
    ret = device_create_file(lpi->ps_dev, &dev_attr_ps_conf2);
    if (ret)
        goto err_create_ps_device;

    /* register the attributes */
    ret = device_create_file(lpi->ps_dev, &dev_attr_ps_canc);
    if (ret)
        goto err_create_ps_device;

    /* register the attributes */
    ret = device_create_file(lpi->ps_dev, &dev_attr_ps_conf1);
    if (ret)
        goto err_create_ps_device;

    /* register the attributes */
    ret = device_create_file(lpi->ps_dev, &dev_attr_ps_thd);
    if (ret)
        goto err_create_ps_device;

    ret = device_create_file(lpi->ps_dev, &dev_attr_ps_hw);
    if (ret)
        goto err_create_ps_device;

    lpi->early_suspend.level =
        EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    lpi->early_suspend.suspend = cm3628_early_suspend;
    lpi->early_suspend.resume = cm3628_late_resume;
    register_early_suspend(&lpi->early_suspend);
 /********* added start  ********/
//    lightsensor_enable(lpi);
//    psensor_enable(lpi);
 /******** added end by  *********/
    D("[PS][CM3628-AD3] lpi->ps_enable = %x\n", lpi->ps_enable);
    D("[PS][CM3628-AD3] lpi->als_enable = %x\n", lpi->als_enable);
    D("[PS][CM3628-AD3] %s: Probe success!\n", __func__);

    return ret;

err_create_ps_device:
    device_unregister(lpi->ps_dev);
err_create_ls_device_file:
    device_unregister(lpi->ls_dev);
err_create_ls_device:
    class_destroy(lpi->cm3628_class);
err_create_class:
err_cm3628_setup:
    destroy_workqueue(lpi->lp_wq);
    wake_lock_destroy(&(lpi->ps_wake_lock));
    mutex_destroy(&als_enable_mutex);
    mutex_destroy(&als_disable_mutex);
    mutex_destroy(&als_get_adc_mutex);
    input_unregister_device(lpi->ls_input_dev);
    input_free_device(lpi->ls_input_dev);
    input_unregister_device(lpi->ps_input_dev);
    input_free_device(lpi->ps_input_dev);
err_create_singlethread_workqueue:
err_lightsensor_update_table:
    misc_deregister(&psensor_misc);
err_psensor_setup:
    misc_deregister(&lightsensor_misc);
err_lightsensor_setup:
err_platform_data_null:
    kfree(lpi);
    return ret;
}

static const struct i2c_device_id cm3628_i2c_id[] = {
    {CM3628_I2C_NAME, 0},
    {}
};

static struct i2c_driver cm3628_driver = {
    .probe = cm3628_probe,
    
    .id_table = cm3628_i2c_id,
    .driver = {
        .name = CM3628_I2C_NAME,
        .owner = THIS_MODULE,
    },
};
static int __init cm3628_init(void)
{
	    return -1;        
}

static void __exit cm3628_exit(void)
{
    i2c_del_driver(&cm3628_driver);
}

module_init(cm3628_init);
module_exit(cm3628_exit);

MODULE_DESCRIPTION("CM3628 Driver");
MODULE_LICENSE("GPL");

