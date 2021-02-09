/* drivers/input/misc/elan_epl2182.c - light and proxmity sensors driver
 * Copyright (C) 2011-2014 ELAN Corporation.
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
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#include <linux/elan_interface.h>
/*********************************************************
 * configuration
*********************************************************/
/* 0 is polling mode, 1 is interrupt mode*/
#define NO_P_SENSOR
#ifndef NO_P_SENSOR
#define PS_INTERRUPT_MODE		1
#else
#define PS_INTERRUPT_MODE		0
#endif

#define PS_POLLING_RATE			500

#define ALS_POLLING_RATE		1000	/* msec */

#define LUX_PER_COUNT			440
//#define  DEBUG
#ifdef DEBUG
#define epl_info(fmt, ...) \
	printk(pr_fmt(fmt), ##__VA_ARGS__)
#else
#define epl_info(fmt, ...) \
	no_printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#endif

#define LOG_FUN(f)		epl_info("%s  %d\n", __func__, __LINE__)
/**********************************************************
 * configuration
**********************************************************/
enum CMC_MODE {
	CMC_MODE_ALS = 0x00,
	CMC_MODE_PS = 0x10,
};
#define TXBYTES				2
#define RXBYTES				2

#define POLLING_MODE_HS		0
#define HS_INTT_CENTER			4 //EPL_INTT_PS_80 //EPL_INTT_PS_48
static int HS_INTT 				= HS_INTT_CENTER;

#define PS_DELAY			55
#define ALS_DELAY			100 //55
#define HS_DELAY 			30

#define PACKAGE_SIZE			2
#define I2C_RETRY_COUNT			10
#define P_INTT				1

#define PS_INTT				4
#define ALS_INTT			12// 4 yuansf modify for light sensor sensitivty

#define PS_H_THRESHOLD 2000
#define PS_L_THRESHOLD 1000

#ifndef NO_P_SENSOR
#define PS_AUTO_ENABLE	1
#else
#define PS_AUTO_ENABLE	0
#endif
#if PS_AUTO_ENABLE
#define DYN_L_OFFSET	500
#define DYN_H_OFFSET	700
#define DYN_CONDITION	30000
#endif

static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);

#if PS_INTERRUPT_MODE
static void epl_sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(epl_sensor_irq_work, epl_sensor_irq_do_work);
#endif

static void report_polling_do_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(report_polling_work, report_polling_do_work);
static void polling_do_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(polling_work, polling_do_work);

// for heart rate
#define PS_DRIVE				EPL_DRIVE_120MA
static struct mutex sensor_mutex;
#if PS_INTERRUPT_MODE
static bool change_int_time = false;
#endif
static int hs_count=0;
static int hs_idx=0;
static int show_hs_raws_flag=0;
static int hs_als_flag=0;

/* primitive raw data from I2C */
struct epl_raw_data {
	u8 raw_bytes[PACKAGE_SIZE];
    u16 renvo;

	u16 ps_int_state;
	u16 ps_ch1_raw;
	u16 als_ch1_raw;
	u16 ps_state;
	u16 ps_sta;
    u16 hs_data[200];
#if PS_AUTO_ENABLE
	u16 ps_min_raw;
	u16 ps_condition;
	u16 ps_cal_h;
	u16 ps_cal_l;
#endif
};

struct elan_epl_data {
	struct i2c_client *client;
	struct regulator *vdd;
	struct regulator *vio;

	unsigned int als_poll_delay;
	unsigned int ps_poll_delay;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;

	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;

	struct workqueue_struct *epl_wq;
	int intr_pin;
	int (*power) (int on);

	int ps_opened;
	int als_opened;

	unsigned int ps_th_l;	/* P_SENSOR_LTHD */
	unsigned int ps_th_h;	/* P_SENSOR_HTHD */
	int enable_pflag;
	int enable_lflag;
	int enable_hflag;
	int l_suspend;
	int read_flag;
	int irq_gpio;
	unsigned int irq_gpio_flags;
};

static struct wake_lock g_ps_wlock;
struct elan_epl_data *epl_data;
static struct epl_raw_data gRawData;
static int dual_count;

#ifndef NO_P_SENSOR
static const char ElanPsensorName[] = "proximity";
#endif
static const char ElanALsensorName[] = "light";

static struct sensors_classdev sensors_light_cdev = {
	.name = "light",
	.vendor = "elan",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "30000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 1000,	/* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

#ifndef NO_P_SENSOR
static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "elan",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 1000,	/* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif

/*
//====================I2C write operation===============//
//regaddr: ELAN Register Address.
//bytecount: How many bytes to be written to register via i2c bus.
//txbyte: I2C bus transmit byte(s).
//Single byte(0X01) transmit only slave address.
//data: setting value.
//
// Example: If you want to write single byte
//to 0x1D register address, show below
//elan_sensor_I2C_Write(client,0x1D,	0x01,		0X02,	0xff);
//
//
*/
static int elan_sensor_I2C_Write(struct i2c_client *client, uint8_t regaddr,
				 uint8_t bytecount, uint8_t txbyte,
				 uint8_t data)
{
	uint8_t buffer[2];
	int ret = 0;
	int retry;

	buffer[0] = (regaddr << 3) | bytecount;
	buffer[1] = data;

	for (retry = 0; retry < I2C_RETRY_COUNT; retry++) {
		ret = i2c_master_send(client, buffer, txbyte);

		if (ret == txbyte)
			break;
		msleep(25);
	}

	if (retry >= I2C_RETRY_COUNT) {
		pr_err(KERN_ERR "i2c write retry over %d\n", I2C_RETRY_COUNT);
		return -EINVAL;
	}

	return ret;
}

static int elan_sensor_I2C_Read(struct i2c_client *client)
{
	uint8_t buffer[RXBYTES];
	int ret = 0, i = 0;
	int retry;

	for (retry = 0; retry < I2C_RETRY_COUNT; retry++) {

		ret = i2c_master_recv(client, buffer, RXBYTES);
		if (ret == RXBYTES)
			break;
		msleep(25);
	}

	if (retry >= I2C_RETRY_COUNT) {
		pr_err("i2c read retry over %d\n", I2C_RETRY_COUNT);
		return -EINVAL;
	}

	for (i = 0; i < PACKAGE_SIZE; i++)
		gRawData.raw_bytes[i] = buffer[i];

	return ret;
}

#if PS_INTERRUPT_MODE
static int elan_epl2182_I2C_Read_long(struct i2c_client *client, int bytecount)
{
    uint8_t buffer[bytecount];
    int ret = 0, i =0;
    int retry;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_recv(client, buffer, bytecount);

        if (ret == bytecount)
            break;

        epl_info("i2c read error,RXBYTES %d\r\n",ret);
        mdelay(10);
    }

    if(retry>=I2C_RETRY_COUNT)
    {
        epl_info("i2c read retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    for(i=0; i<bytecount; i++)
        gRawData.raw_bytes[i] = buffer[i];

    return ret;
}

static void epl2182_read_hs(void)
{

    struct elan_epl_data *epld = epl_data;
    struct i2c_client *client = epld->client;
    int max_frame = 200;
    int idx = hs_idx+hs_count;
    u16 data;

 mutex_lock(&sensor_mutex);
    elan_sensor_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
    elan_epl2182_I2C_Read_long(client, 2);
    data=(gRawData.raw_bytes[1]<<8)|gRawData.raw_bytes[0];


    if(data>60800&& HS_INTT>HS_INTT_CENTER-5)
    {
        HS_INTT--;
        change_int_time=true;
    }
    else if(data>6400 && data <25600 && HS_INTT<HS_INTT_CENTER+5)
    {
        HS_INTT++;
        change_int_time=true;
    }
    else
    {
        change_int_time=false;

        if(idx>=max_frame)
            idx-=max_frame;

        gRawData.hs_data[idx] = data;

        if(hs_count>=max_frame)
        {
            hs_idx++;
            if(hs_idx>=max_frame)
                hs_idx=0;
        }

        hs_count++;
        if(hs_count>=max_frame)
            hs_count=max_frame;
    }
    mutex_unlock(&sensor_mutex);

}
#endif

static void elan_sensor_restart_work(void)
{
	struct elan_epl_data *epld = epl_data;
	cancel_delayed_work(&polling_work);
	cancel_delayed_work(&report_polling_work);
	queue_delayed_work(epld->epl_wq, &polling_work, msecs_to_jiffies(10));
}

#if PS_AUTO_ENABLE
static void dyn_ps_cal(struct elan_epl_data *epld)
{
	if((gRawData.ps_ch1_raw < gRawData.ps_min_raw)
	&& (gRawData.ps_sta != 1)
	&& (gRawData.ps_condition <= DYN_CONDITION))
	{
		gRawData.ps_min_raw = gRawData.ps_ch1_raw;
		epld->ps_th_l = gRawData.ps_ch1_raw + DYN_L_OFFSET;
		epld->ps_th_h = gRawData.ps_ch1_raw + DYN_H_OFFSET;
		set_psensor_intr_threshold(epld->ps_th_l, epld->ps_th_h);
		epl_info("dyn ps raw = %d, min = %d, condition = %d\n dyn h_thre = %d, l_thre = %d, ps_state = %d",
		gRawData.ps_ch1_raw, gRawData.ps_min_raw, gRawData.ps_condition,epld->ps_th_h, epld->ps_th_l, gRawData.ps_state);
	}
}
#endif

static void epl2182_hs_enable(struct elan_epl_data *epld, bool interrupt, bool full_enable)
{
    int ret;
    uint8_t regdata = 0;
    struct i2c_client *client = epld->client;

    if(full_enable)
    {

        regdata = PS_DRIVE | (interrupt? EPL_INT_FRAME_ENABLE : EPL_INT_DISABLE);
        ret = elan_sensor_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02, regdata);

        regdata = EPL_SENSING_1_TIME | EPL_PS_MODE | EPL_L_GAIN | EPL_S_SENSING_MODE;
        ret = elan_sensor_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

        regdata = HS_INTT<<4 | EPL_PST_1_TIME | EPL_12BIT_ADC;
        ret = elan_sensor_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

        ret = elan_sensor_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);


    }

    ret = elan_sensor_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
}

static int elan_sensor_psensor_enable(struct elan_epl_data *epld)
{
	int ret;
	uint8_t regdata = 0;
	struct i2c_client *client = epld->client;
	LOG_FUN();
	epl_info("--- Proximity sensor Enable ---\n");

	ret =
	    elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
				  EPL_INT_DISABLE);

	regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_L_GAIN;
	regdata =
	    regdata | (PS_INTERRUPT_MODE ? EPL_C_SENSING_MODE :
		       EPL_S_SENSING_MODE);
	ret =
	    elan_sensor_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, regdata);

	regdata = PS_INTT << 4 | EPL_PST_1_TIME | EPL_14BIT_ADC;
	ret =
	    elan_sensor_I2C_Write(client, REG_1, W_SINGLE_BYTE, 0X02, regdata);

	set_psensor_intr_threshold(epld->ps_th_l, epld->ps_th_h);

	ret =
	    elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0X02,
				  EPL_C_RESET);
	ret =
	    elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02,
				  EPL_C_START_RUN);
        msleep(PS_DELAY);
#if PS_INTERRUPT_MODE
	if (epld->enable_pflag) {		
		elan_sensor_I2C_Write(client, REG_13, R_SINGLE_BYTE, 0x01, 0);
		elan_sensor_I2C_Read(client);
		gRawData.ps_state = !((gRawData.raw_bytes[0] & 0x04) >> 2);
		gRawData.ps_sta = ((gRawData.raw_bytes[0]&0x02)>>1);

#if PS_AUTO_ENABLE
		elan_sensor_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
		elan_sensor_I2C_Read(client);
		gRawData.ps_ch1_raw = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];

	    	elan_sensor_I2C_Write(client,REG_14,R_TWO_BYTE,0x01,0x00);
	    	elan_sensor_I2C_Read(client);
		gRawData.ps_condition= ((gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0]);

		dyn_ps_cal(epld);
#endif

		if (gRawData.ps_state != gRawData.ps_int_state) {
			elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE,
					      0x02, EPL_INT_FRAME_ENABLE);
		} else {
			elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE,
					      0x02, EPL_INT_ACTIVE_LOW);
		}

	} else {
		elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
				      EPL_INT_ACTIVE_LOW);
	}
#endif

	if (ret != 0x02)
		epl_info("P-sensor i2c err\n");

	return ret;
}

static int elan_sensor_lsensor_enable(struct elan_epl_data *epld)
{
	int ret;
	uint8_t regdata = 0;
	struct i2c_client *client = epld->client;
	LOG_FUN();
	epl_info("--- ALS sensor Enable --\n");

	regdata = EPL_INT_DISABLE;
	ret =
	    elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, regdata);

	regdata =
	    EPL_S_SENSING_MODE | EPL_SENSING_8_TIME | EPL_ALS_MODE |
	    EPL_AUTO_GAIN;
	ret =
	    elan_sensor_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, regdata);

	regdata = ALS_INTT << 4 | EPL_PST_1_TIME | EPL_10BIT_ADC;
	ret =
	    elan_sensor_I2C_Write(client, REG_1, W_SINGLE_BYTE, 0X02, regdata);

	ret =
	    elan_sensor_I2C_Write(client, REG_10, W_SINGLE_BYTE, 0x02,
				  EPL_GO_MID);
	ret =
	    elan_sensor_I2C_Write(client, REG_11, W_SINGLE_BYTE, 0x02,
				  EPL_GO_LOW);

	ret =
	    elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0X02,
				  EPL_C_RESET);
	ret =
	    elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02,
				  EPL_C_START_RUN);

	if (ret != 0x02)
		epl_info(" ALS-sensor i2c err\n");

	return ret;

}

/*
//====================elan_epl_ps_poll_rawdata===============//
//polling method for proximity sensor detect.
//Report proximity sensor raw data.
//Report "ABS_DISTANCE" event to HAL layer.
//Variable "value" 0 and 1 to represent
//which distance from psensor to target(human's face..etc).
//value: 0 represent near.
//value: 1 represent far.
*/
static void elan_epl_ps_poll_rawdata(void)
{
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;

	elan_sensor_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02,
			      EPL_DATA_LOCK);

	elan_sensor_I2C_Write(client, REG_13, R_SINGLE_BYTE, 0x01, 0);
	elan_sensor_I2C_Read(client);
	gRawData.ps_state = !((gRawData.raw_bytes[0] & 0x04) >> 2);
	elan_sensor_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
	elan_sensor_I2C_Read(client);
	gRawData.ps_ch1_raw =
	    (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];

	elan_sensor_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02,
			      EPL_DATA_UNLOCK);

	epl_info("### ps_ch1_raw_data  (%d), value(%d) ###\n\n",
		 gRawData.ps_ch1_raw, gRawData.ps_state);

	input_report_abs(epld->ps_input_dev, ABS_DISTANCE, gRawData.ps_state);
	input_sync(epld->ps_input_dev);
}

static void elan_epl_als_rawdata(void)
{
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;
	uint32_t lux;
	ktime_t ts;

	ts = ktime_get_boottime();
	elan_sensor_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
	elan_sensor_I2C_Read(client);
	gRawData.als_ch1_raw =
	    (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];

	lux = (gRawData.als_ch1_raw * LUX_PER_COUNT) / 1000 * 15 / 100;
	if (lux > 20000)
		lux = 20000;

	epl_info("-------------------  ALS raw = %d, lux = %d\n\n",
		 gRawData.als_ch1_raw, lux);

	input_report_abs(epld->als_input_dev, ABS_MISC, lux);
	input_event(epld->als_input_dev, EV_SYN, SYN_TIME_SEC,
			ktime_to_timespec(ts).tv_sec);
	input_event(epld->als_input_dev, EV_SYN, SYN_TIME_NSEC,
			ktime_to_timespec(ts).tv_nsec);
	input_sync(epld->als_input_dev);
}

/*
//====================set_psensor_intr_threshold===============//
//low_thd: The value is psensor interrupt low threshold.
//high_thd:	The value is psensor interrupt hihg threshold.
//When psensor rawdata > hihg_threshold, interrupt pin will be pulled low.
//After interrupt occur, psensor rawdata < low_threshold,
//interrupt pin will be pulled high.
*/
static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
	int ret = 0;
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;

	uint8_t high_msb, high_lsb, low_msb, low_lsb;

	high_msb = (uint8_t) (high_thd >> 8);
	high_lsb = (uint8_t) (high_thd & 0x00ff);
	low_msb = (uint8_t) (low_thd >> 8);
	low_lsb = (uint8_t) (low_thd & 0x00ff);

	elan_sensor_I2C_Write(client, REG_2, W_SINGLE_BYTE, 0x02, high_lsb);
	elan_sensor_I2C_Write(client, REG_3, W_SINGLE_BYTE, 0x02, high_msb);
	elan_sensor_I2C_Write(client, REG_4, W_SINGLE_BYTE, 0x02, low_lsb);
	elan_sensor_I2C_Write(client, REG_5, W_SINGLE_BYTE, 0x02, low_msb);

	return ret;
}

#if PS_INTERRUPT_MODE
static void epl_sensor_irq_do_work(struct work_struct *work)
{
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;
	int mode = 0;
	LOG_FUN();
    if(epld->enable_hflag == 1)
    {
        epl2182_read_hs();
        epl2182_hs_enable(epld, true, change_int_time);
    }

	if(epld->enable_pflag == 1)
	{
		elan_sensor_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02,
					  EPL_DATA_LOCK);
		elan_sensor_I2C_Write(client, REG_13, R_SINGLE_BYTE, 0x01, 0);
		elan_sensor_I2C_Read(client);
		mode = gRawData.raw_bytes[0] & (3 << 4);

		if (mode == CMC_MODE_PS && epld->enable_pflag) {
			gRawData.ps_int_state = !((gRawData.raw_bytes[0] & 0x04) >> 2);
			elan_epl_ps_poll_rawdata();
		} else {
			epl_info("error: interrupt in als\n");
		}

		elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
				      EPL_INT_ACTIVE_LOW);

		elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02,
				      EPL_DATA_UNLOCK);
	}
	

	enable_irq(client->irq);
}

static irqreturn_t elan_sensor_irq_handler(int irqNo, void *handle)
{
	struct i2c_client *client = (struct i2c_client *)handle;
	struct elan_epl_data *epld = i2c_get_clientdata(client);

	disable_irq_nosync(client->irq);
	queue_work(epld->epl_wq, &epl_sensor_irq_work);

	return IRQ_HANDLED;
}
#endif

static void report_polling_do_work(struct work_struct *work)
{
	struct elan_epl_data *epld = epl_data;

	if (dual_count == CMC_MODE_PS)
		elan_epl_ps_poll_rawdata();
	else if (dual_count == CMC_MODE_ALS)
		elan_epl_als_rawdata();

	if (epld->enable_pflag) {
		elan_sensor_psensor_enable(epld);

		if (PS_INTERRUPT_MODE == 0) {
			dual_count = CMC_MODE_PS;
			queue_delayed_work(epld->epl_wq, &report_polling_work,
					   msecs_to_jiffies
					   (epld->ps_poll_delay));
		}
	}
}

static void polling_do_work(struct work_struct *work)
{
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;

	bool isInterleaving = epld->enable_pflag == 1 && epld->enable_lflag == 1 && epld->l_suspend ==0;
	bool isAlsOnly = epld->enable_pflag == 0 && epld->enable_lflag == 1 && epld->l_suspend == 0;
	bool isPsOnly = epld->enable_pflag == 1 && (epld->enable_lflag == 0 || epld->l_suspend == 1);

	cancel_delayed_work(&polling_work);
	cancel_delayed_work(&report_polling_work);

	epl_info("enable_pflag = %d, enable_lflag = %d\n", epld->enable_pflag,
		 epld->enable_lflag);

	if (isAlsOnly || isInterleaving) {
		elan_sensor_lsensor_enable(epld);
		dual_count = CMC_MODE_ALS;

		queue_delayed_work(epld->epl_wq, &report_polling_work,
				   msecs_to_jiffies(ALS_DELAY));
		queue_delayed_work(epld->epl_wq, &polling_work,
				   msecs_to_jiffies(epld->als_poll_delay));
	} else if (isPsOnly || PS_AUTO_ENABLE) {
		elan_sensor_psensor_enable(epld);
		dual_count = CMC_MODE_PS;

		if (PS_INTERRUPT_MODE) {
			/* do nothing */
		} else {
			queue_delayed_work(epld->epl_wq, &report_polling_work,
					   msecs_to_jiffies(PS_DELAY));
			queue_delayed_work(epld->epl_wq, &polling_work,
					   msecs_to_jiffies
					   (epld->ps_poll_delay));
		}
	} else {
		elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
				      EPL_INT_DISABLE);
		elan_sensor_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02,
				      EPL_S_SENSING_MODE);
	}
}

static ssize_t elan_ls_operationmode_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	uint16_t mode = 0;
	/* struct elan_epl_data *epld = epl_data; */
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_epl_data *epld = i2c_get_clientdata(client);

	sscanf(buf, "%hu", &mode);
	epl_info("==>[operation mode]=%d\n", mode);

	if (mode == 0) {
		epld->enable_lflag = 0;
		epld->enable_pflag = 0;
	} else if (mode == 1) {
		epld->enable_lflag = 1;
		epld->enable_pflag = 0;
	} 
    #ifndef NO_P_SENSOR
       else if (mode == 2) {
		epld->enable_lflag = 0;
		epld->enable_pflag = 1;
	} else if (mode == 3) {
		epld->enable_lflag = 1;
		epld->enable_pflag = 1;
	} 
    #endif
       else {
		epl_info("0: none\n1: als only\n2: ps only\n3: interleaving");
	}

	elan_sensor_restart_work();
	return count;
}

static ssize_t elan_ls_operationmode_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct elan_epl_data *epld = epl_data;
	long *tmp = (long *)buf;
	uint16_t mode = 0;
	LOG_FUN();

	if (epld->enable_pflag == 0 && epld->enable_lflag == 0)
		mode = 0;
	else if (epld->enable_pflag == 0 && epld->enable_lflag == 1)
		mode = 1;
	else if (epld->enable_pflag == 1 && epld->enable_lflag == 0)
		mode = 2;
	else if (epld->enable_pflag == 1 && epld->enable_lflag == 1)
		mode = 3;

	tmp[0] = mode;

	/*return sprintf(buf, "%d\n", mode); */
	return snprintf(buf, PAGE_SIZE, "%d\n", mode);
}

static ssize_t elan_ls_status_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct elan_epl_data *epld = epl_data;
	u16 ch1;

	if (!epl_data) {
		epl_info("epl_data is null!!\n");
		return 0;
	}
	elan_sensor_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02,
			      EPL_DATA_LOCK);

	elan_sensor_I2C_Write(epld->client, REG_16, R_TWO_BYTE, 0x01, 0x00);
	elan_sensor_I2C_Read(epld->client);
	ch1 = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];
	epl_info("ch1 raw_data = %d\n", ch1);

	elan_sensor_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02,
			      EPL_DATA_UNLOCK);

	/* return sprintf(buf, "%d\n", ch1); */
	return snprintf(buf, PAGE_SIZE, "%d\n", ch1);
}

static ssize_t epl2182_show_renvo(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t len = 0;

	LOG_FUN();
    epl_info("gRawData.renvo=0x%x \r\n", gRawData.renvo);

    len += snprintf(buf+len, PAGE_SIZE-len, "%x",gRawData.renvo);

    return len;
}

static ssize_t epl2182_store_hs_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct elan_epl_data *obj = epl_data;
    bool enable_als = obj->enable_lflag == 1 && obj->l_suspend == 0;
	LOG_FUN();

    sscanf(buf, "%hu",&mode);


    if(mode){
        if(enable_als == true){
            obj->l_suspend = 1;
            hs_als_flag = 1;
            msleep(ALS_DELAY);
        }
        obj->enable_hflag = 1;
    }
    else{
        obj->enable_hflag = 0;

        if(hs_als_flag == 1){
            obj->l_suspend = 0;
            hs_als_flag = 0;
        }

		elan_sensor_lsensor_enable(obj);
	}

    if(mode)
    {
        hs_idx=0;
        hs_count=0;
		epl2182_hs_enable(obj,true,true);
    }


    //epl2182_restart_polling();
    return count;
}

static ssize_t epl2182_show_hs_raws(struct device *dev, struct device_attribute *attr, char *buf)
{
    u16 *tmp = (u16*)buf;
    u16 length= hs_count;
    int byte_count=2+length*2;
    int i=0;
    int start = hs_idx;
    LOG_FUN();
    mutex_lock(&sensor_mutex);
    tmp[0]= length;
    if(length == 0){
        tmp[0] = 1;
        length = 1;
        show_hs_raws_flag = 1;
    }
    for(i=start; i<length; i++){
        if(show_hs_raws_flag == 1){
            tmp[i+1] = 0;
            show_hs_raws_flag = 0;
        }
        else{
            tmp[i+1] = gRawData.hs_data[i];
        }

    }

    hs_count=0;
    hs_idx=0;
    mutex_unlock(&sensor_mutex);

    return byte_count;
}


static DEVICE_ATTR(elan_ls_operationmode, S_IROTH | S_IWOTH,
		   elan_ls_operationmode_show, elan_ls_operationmode_store);
static DEVICE_ATTR(elan_ls_status, S_IROTH | S_IWOTH, elan_ls_status_show,
		   NULL);
static DEVICE_ATTR(elan_renvo,    			S_IROTH | S_IWOTH, epl2182_show_renvo,   				NULL);
static DEVICE_ATTR(hs_enable,				S_IROTH | S_IWOTH, NULL, epl2182_store_hs_enable);
static DEVICE_ATTR(hs_raws,					S_IROTH | S_IWOTH, epl2182_show_hs_raws, NULL);

static struct attribute *ets_attributes[] = {
	&dev_attr_elan_ls_operationmode.attr,
	&dev_attr_elan_ls_status.attr,
	&dev_attr_elan_renvo.attr,
	&dev_attr_hs_enable.attr,
	&dev_attr_hs_raws.attr,
	NULL,
};

static struct attribute_group ets_attr_group = {
	.attrs = ets_attributes,
};

static int elan_als_open(struct inode *inode, struct file *file)
{
	struct elan_epl_data *epld = epl_data;

	LOG_FUN();

	if (epld->als_opened)
		return -EBUSY;
	epld->als_opened = 1;

	return 0;
}

static int elan_als_read(struct file *file, char __user *buffer, size_t count,
			 loff_t *ppos)
{
	struct elan_epl_data *epld = epl_data;
	int buf[1];
	if (epld->read_flag == 1) {
		buf[0] = gRawData.als_ch1_raw;
		if (copy_to_user(buffer, &buf, sizeof(buf)))
			return 0;
		epld->read_flag = 0;
		return 12;
	} else {
		return 0;
	}
}

static int elan_als_release(struct inode *inode, struct file *file)
{
	struct elan_epl_data *epld = epl_data;

	LOG_FUN();

	epld->als_opened = 0;

	return 0;
}

static long elan_als_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	int flag;
	unsigned long buf[1];
	struct elan_epl_data *epld = epl_data;

	void __user *argp = (void __user *)arg;

	epl_info("als io ctrl cmd %d\n", _IOC_NR(cmd));

	switch (cmd) {
	case ELAN_EPL6800_IOCTL_GET_LFLAG:

		epl_info("elan ambient-light IOCTL Sensor get lflag\n");
		flag = epld->enable_lflag;
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;

		epl_info("elan ambient-light Sensor get lflag %d\n", flag);
		break;

	case ELAN_EPL6800_IOCTL_ENABLE_LFLAG:

		epl_info("elan ambient-light IOCTL Sensor set lflag\n");
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if (flag < 0 || flag > 1)
			return -EINVAL;

		epld->enable_lflag = flag;
		elan_sensor_restart_work();

		epl_info("elan ambient-light Sensor set lflag %d\n", flag);
		break;

	case ELAN_EPL6800_IOCTL_GETDATA:
		buf[0] = (unsigned long)gRawData.als_ch1_raw;
		if (copy_to_user(argp, &buf, sizeof(buf)))
			return -EFAULT;

		break;

	default:
		pr_err("invalid cmd %d\n", _IOC_NR(cmd));
		return -EINVAL;
	}

	return 0;

}

static const struct file_operations elan_als_fops = {
	.owner = THIS_MODULE,
	.open = elan_als_open,
	.read = elan_als_read,
	.release = elan_als_release,
	.unlocked_ioctl = elan_als_ioctl,
};

static struct miscdevice elan_als_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "elan_als",
	.fops = &elan_als_fops,
};

#ifndef NO_P_SENSOR
static int elan_ps_open(struct inode *inode, struct file *file)
{
	struct elan_epl_data *epld = epl_data;

	LOG_FUN();

	if (epld->ps_opened)
		return -EBUSY;

	epld->ps_opened = 1;

	return 0;
}

static int elan_ps_release(struct inode *inode, struct file *file)
{
	struct elan_epl_data *epld = epl_data;

	LOG_FUN();

	epld->ps_opened = 0;

	return 0;
}

static long elan_ps_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	int value;
	int flag;
	struct elan_epl_data *epld = epl_data;

	void __user *argp = (void __user *)arg;

	epl_info("ps io ctrl cmd %d\n", _IOC_NR(cmd));

	switch (cmd) {
	case ELAN_EPL6800_IOCTL_GET_PFLAG:

		epl_info("elan Proximity Sensor IOCTL get pflag\n");
		flag = epld->enable_pflag;
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;

		epl_info("elan Proximity Sensor get pflag %d\n", flag);
		break;

	case ELAN_EPL6800_IOCTL_ENABLE_PFLAG:
		epl_info("elan Proximity IOCTL Sensor set pflag\n");
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if (flag < 0 || flag > 1)
			return -EINVAL;

		epld->enable_pflag = flag;
		elan_sensor_restart_work();

		epl_info("elan Proximity Sensor set pflag %d\n", flag);
		break;

	case ELAN_EPL6800_IOCTL_GETDATA:

		value = gRawData.ps_ch1_raw;
		if (copy_to_user(argp, &value, sizeof(value)))
			return -EFAULT;

		epl_info("elan proximity Sensor get data (%d)\n", value);
		break;

	default:
		pr_err("invalid cmd %d\n", _IOC_NR(cmd));
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations elan_ps_fops = {
	.owner = THIS_MODULE,
	.open = elan_ps_open,
	.release = elan_ps_release,
	.unlocked_ioctl = elan_ps_ioctl
};

static struct miscdevice elan_ps_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "elan_ps",
	.fops = &elan_ps_fops
};
#endif

static int initial_sensor(struct elan_epl_data *epld)
{
	struct i2c_client *client = epld->client;

	int ret = 0;

	epl_info("initial_sensor enter!\n");

	ret = elan_sensor_I2C_Read(client);

	if (ret < 0)
		return -EINVAL;

	elan_sensor_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0x02,
			      EPL_S_SENSING_MODE);
	elan_sensor_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
			      EPL_INT_DISABLE);
	set_psensor_intr_threshold(epld->ps_th_l, epld->ps_th_h);

	msleep(20);

	epld->enable_lflag = 0;
	epld->enable_pflag = 0;

	return ret;
}

static int elan_power_on(struct elan_epl_data *epld, bool on)
{
	int rc;

	LOG_FUN();

	if (!on)
		goto power_off;

	rc = regulator_enable(epld->vdd);
	if (rc) {
		dev_err(&epld->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(epld->vio);
	if (rc) {
		dev_err(&epld->client->dev,
			"Regulator vio enable failed rc=%d\n", rc);
		regulator_disable(epld->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(epld->vdd);
	if (rc) {
		dev_err(&epld->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(epld->vio);
	if (rc) {
		dev_err(&epld->client->dev,
			"Regulator vio disable failed rc=%d\n", rc);
	}

	return rc;
}

static int elan_power_init(struct elan_epl_data *epld, bool on)
{
	int rc;

	LOG_FUN();

	if (!on)
		goto pwr_deinit;

	epld->vdd = regulator_get(&epld->client->dev, "vdd");
	if (IS_ERR(epld->vdd)) {
		rc = PTR_ERR(epld->vdd);
		dev_err(&epld->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(epld->vdd) > 0) {
		rc = regulator_set_voltage(epld->vdd, EPL2182_VDD_MIN_UV,
					   EPL2182_VDD_MAX_UV);
		if (rc) {
			dev_err(&epld->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	epld->vio = regulator_get(&epld->client->dev, "vio");
	if (IS_ERR(epld->vio)) {
		rc = PTR_ERR(epld->vio);
		dev_err(&epld->client->dev,
			"Regulator get failed vio rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(epld->vio) > 0) {
		rc = regulator_set_voltage(epld->vio, EPL2182_VIO_MIN_UV,
					   EPL2182_VIO_MAX_UV);
		if (rc) {
			dev_err(&epld->client->dev,
				"Regulator set_vtg failed vio rc=%d\n", rc);
			goto reg_vio_put;
		}
	}

	return 0;

reg_vio_put:
	regulator_put(epld->vio);
reg_vdd_set_vtg:
	if (regulator_count_voltages(epld->vdd) > 0)
		regulator_set_voltage(epld->vdd, 0, EPL2182_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(epld->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(epld->vdd) > 0)
		regulator_set_voltage(epld->vdd, 0, EPL2182_VDD_MAX_UV);

	regulator_put(epld->vdd);

	if (regulator_count_voltages(epld->vio) > 0)
		regulator_set_voltage(epld->vio, 0, EPL2182_VIO_MAX_UV);

	regulator_put(epld->vio);
	return 0;
}

static int lightsensor_setup(struct elan_epl_data *epld)
{
	int err = 0;
	epl_info("lightsensor_setup enter.\n");

	epld->als_input_dev = input_allocate_device();
	if (!epld->als_input_dev) {
		pr_err("could not allocate ls input device\n");
		return -ENOMEM;
	}
	epld->als_input_dev->name = ElanALsensorName;
	set_bit(EV_ABS, epld->als_input_dev->evbit);
	input_set_abs_params(epld->als_input_dev, ABS_MISC, 0, 9, 0, 0);

	err = input_register_device(epld->als_input_dev);
	if (err < 0) {
		pr_err("can not register ls input device\n");
		goto err_free_ls_input_device;
	}

	err = misc_register(&elan_als_device);
	if (err < 0) {
		pr_err("can not register ls misc device\n");
		goto err_unregister_ls_input_device;
	}

	return err;

err_unregister_ls_input_device:
	input_unregister_device(epld->als_input_dev);
err_free_ls_input_device:
	input_free_device(epld->als_input_dev);
	return err;
}

#ifndef NO_P_SENSOR
static int psensor_setup(struct elan_epl_data *epld)
{
	int err = 0;
	epl_info("psensor_setup enter.\n");

	epld->ps_input_dev = input_allocate_device();
	if (!epld->ps_input_dev) {
		pr_err("could not allocate ps input device\n");
		return -ENOMEM;
	}
	epld->ps_input_dev->name = ElanPsensorName;

	set_bit(EV_ABS, epld->ps_input_dev->evbit);
	input_set_abs_params(epld->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	err = input_register_device(epld->ps_input_dev);
	if (err < 0) {
		pr_err("could not register ps input device\n");
		goto err_free_ps_input_device;
	}

	err = misc_register(&elan_ps_device);
	if (err < 0) {
		pr_err("could not register ps misc device\n");
		goto err_unregister_ps_input_device;
	}

	return err;

err_unregister_ps_input_device:
	input_unregister_device(epld->ps_input_dev);
err_free_ps_input_device:
	input_free_device(epld->ps_input_dev);
	return err;
}
#endif

#if PS_INTERRUPT_MODE
static int setup_interrupt(struct elan_epl_data *epld)
{
	struct i2c_client *client = epld->client;

	int err = 0;
	msleep(20);
	LOG_FUN();
	if (gpio_is_valid(epld->irq_gpio)) {
		err = gpio_request(epld->irq_gpio, "elan_irq_gpio");
		if (err)
			dev_err(&client->dev, "irq gpio request failed");

		err = gpio_direction_input(epld->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
		}
	}

	err =
	    request_irq(client->irq, elan_sensor_irq_handler,
			IRQF_TRIGGER_FALLING, "EPL2182", (void *)client);
	if (err < 0) {
		pr_err("request irq pin %d fail for gpio\n", err);
		goto exit_free_irq;
	}

	irq_set_irq_wake(client->irq, 1);

	return err;

exit_free_irq:
	free_irq(client->irq, client);
	return err;
}
#endif

#ifndef NO_P_SENSOR
static int elan_enable_ps_sensor(struct i2c_client *client, int val)
{
	struct elan_epl_data *epld = i2c_get_clientdata(client);
	LOG_FUN();
	pr_debug("epl2182 enable PS sensor -> %d\n", val);

	if ((val != 0) && (val != 1)) {
		pr_debug("%s:store unvalid value=%d\n", __func__, val);
		return -EINVAL;
	}

	if (val == 1) {
		if (epld->enable_pflag == 0)
			epld->enable_pflag = 1;
	} else {
		if (epld->enable_pflag == 1)
			epld->enable_pflag = 0;
	}

	elan_sensor_restart_work();

	return 0;
}
#endif

static int elan_enable_als_sensor(struct i2c_client *client, int val)
{
	struct elan_epl_data *epld = i2c_get_clientdata(client);

	LOG_FUN();
	pr_debug("%s: val=%d\n", __func__, val);

	if ((val != 0) && (val != 1)) {
		pr_err("%s: invalid value (val = %d)\n", __func__, val);
		return -EINVAL;
	}

	if (val == 1) {
		if (epld->enable_lflag == 0)
			epld->enable_lflag = 1;

	} else {
		if (epld->enable_lflag == 1)
			epld->enable_lflag = 0;
	}
	elan_sensor_restart_work();

	return 0;
}

static int elan_als_set_enable(struct sensors_classdev *sensors_cdev,
			       unsigned int enable)
{
	struct elan_epl_data *epld =
	    container_of(sensors_cdev, struct elan_epl_data, als_cdev);

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	return elan_enable_als_sensor(epld->client, enable);
}

#ifndef NO_P_SENSOR
static int elan_ps_set_enable(struct sensors_classdev *sensors_cdev,
			      unsigned int enable)
{
	struct elan_epl_data *epld =
	    container_of(sensors_cdev, struct elan_epl_data, ps_cdev);

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

#if PS_AUTO_ENABLE	
	if(enable == 1){
		gRawData.ps_min_raw = 0xffff;
	}
#endif

	return elan_enable_ps_sensor(epld->client, enable);
}
#endif

/*at least 500ms, "echo A > poll_delay" means poll A msec! */
static ssize_t elan_als_poll_delay(struct sensors_classdev *sensors_cdev,
				   unsigned int delay_msec)
{
	struct elan_epl_data *epld =
	    container_of(sensors_cdev, struct elan_epl_data, als_cdev);

	if (delay_msec < ALS_POLLING_RATE / 2)	/*at least 500 ms */
		delay_msec = ALS_POLLING_RATE / 2;

	epld->als_poll_delay = delay_msec;	/* convert us => ms */

	if (epld->enable_lflag == 1) {
		cancel_delayed_work(&polling_work);
		cancel_delayed_work(&report_polling_work);
		queue_delayed_work(epld->epl_wq, &polling_work,
				   msecs_to_jiffies(10));
	}
	return 0;
}

#ifndef NO_P_SENSOR
/*at least 500ms, "echo A > poll_delay" means poll A msec! */
static ssize_t elan_ps_poll_delay(struct sensors_classdev *sensors_cdev,
				  unsigned int delay_msec)
{
	struct elan_epl_data *epld =
	    container_of(sensors_cdev, struct elan_epl_data, ps_cdev);

	if (delay_msec < PS_POLLING_RATE)	/* at least 500 ms */
		delay_msec = PS_POLLING_RATE;

	epld->ps_poll_delay = delay_msec;	/* convert us => ms */

	if (epld->enable_pflag == 1) {
		cancel_delayed_work(&polling_work);
		cancel_delayed_work(&report_polling_work);
		queue_delayed_work(epld->epl_wq, &polling_work,
				   msecs_to_jiffies(10));
	}
	return 0;
}
#endif

static int elan_sensor_suspend(struct i2c_client *client, pm_message_t mesg)
{
	LOG_FUN();

	if (epl_data->enable_pflag == 0) {
		elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02,
				      EPL_C_P_DOWN);
		cancel_delayed_work(&polling_work);
	}
	return 0;
}

static int elan_sensor_resume(struct i2c_client *client)
{
	struct elan_epl_data *epld = epl_data;
	LOG_FUN();

	if (epld->enable_pflag | epld->enable_lflag) {
		elan_sensor_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02,
				      EPL_C_P_UP);
	}

	if (epld->enable_pflag)
		elan_sensor_restart_work();

	return 0;
}

static int sensor_parse_dt(struct device *dev, struct elan_epl_data *epld)
{
	struct device_node *np = dev->of_node;
	struct i2c_client *client;
#ifndef NO_P_SENSOR
	unsigned int tmp;
	int rc = 0;
#endif

	client = epld->client;

	LOG_FUN();
	/* irq gpio */
	epld->irq_gpio = of_get_named_gpio_flags(np, "epl2182,irq-gpio",
						 0, &epld->irq_gpio_flags);
	if (epld->irq_gpio < 0)
		return epld->irq_gpio;
#ifndef NO_P_SENSOR
	/* ps tuning data */
	rc = of_property_read_u32(np, "epl2182,prox_th_min", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read prox_th_min\n");
		return rc;
	}

	epld->ps_th_l = tmp;

	rc = of_property_read_u32(np, "epl2182,prox_th_max", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read prox_th_max\n");
		return rc;
	}

	epld->ps_th_h = tmp;
#endif
	return 0;
}
static int i2c_reg_check(struct i2c_client *client)
{
	epl_info("chip id REG 0x00 value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x00));
	epl_info("chip id REG 0x01 value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x08));
	epl_info("chip id REG 0x02 value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x10));
	epl_info("chip id REG 0x03 value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x18));
	epl_info("chip id REG 0x04 value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x20));
	epl_info("chip id REG 0x05 value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x28));
	epl_info("chip id REG 0x06 value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x30));
	epl_info("chip id REG 0x07 value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x38));
	epl_info("chip id REG 0x09 value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x48));
	epl_info("chip id REG 0x0D value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x68));
	epl_info("chip id REG 0x0E value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x70));
	epl_info("chip id REG 0x0F value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x71));
	epl_info("chip id REG 0x10 value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x80));
	epl_info("chip id REG 0x11 value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x88));
	epl_info("chip id REG 0x13 value = %8x\n",
		 i2c_smbus_read_byte_data(client, 0x98));

	return 0;
}

static int elan_sensor_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int err = 0, ret = 0;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct elan_epl_data *epld;
       struct platform_device *sensor_dev;

	epl_info("elan sensor probe enter.\n");

	epld = kzalloc(sizeof(struct elan_epl_data), GFP_KERNEL);
	if (!epld)
		return -ENOMEM;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"No supported i2c func what we need?!!\n");
		err = -ENOTSUPP;
		goto i2c_fail;
	}
	epld->als_poll_delay = ALS_POLLING_RATE;
    #ifndef NO_P_SENSOR
	epld->ps_poll_delay = PS_POLLING_RATE;
    #endif
	epld->client = client;
    #ifndef NO_P_SENSOR
	epld->ps_th_h = PS_H_THRESHOLD;
	epld->ps_th_l = PS_L_THRESHOLD;
    #endif
	epld->l_suspend = 0;
	i2c_set_clientdata(client, epld);

	epl_data = epld;

	err = sensor_parse_dt(&client->dev, epld);
	if (err) {
		pr_err("%s: sensor_parse_dt() err\n", __func__);
		return err;
	}
	err = elan_power_init(epld, true);
	if (err)
		dev_err(&client->dev, "power init failed");

	err = elan_power_on(epld, true);
	if (err)
		dev_err(&client->dev, "power on failed");
	/*check i2c func and register value*/
	i2c_reg_check(client);

    mutex_init(&sensor_mutex);
    elan_sensor_I2C_Write(client,REG_19,R_TWO_BYTE,0x01,0x00);
    elan_sensor_I2C_Read(client);
    gRawData.renvo = (gRawData.raw_bytes[1]<<8)|gRawData.raw_bytes[0];

	epld->epl_wq = create_singlethread_workqueue("elan_sensor_wq");
	if (!epld->epl_wq) {
		pr_err("can't create workqueue\n");
		err = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	err = lightsensor_setup(epld);
	if (err < 0) {
		pr_err("lightsensor_setup error!!\n");
		goto err_lightsensor_setup;
	}
#ifndef NO_P_SENSOR
	err = psensor_setup(epld);
	if (err < 0) {
		pr_err("psensor_setup error!!\n");
		goto err_psensor_setup;
	}
#endif

	err = initial_sensor(epld);
	if (err < 0) {
		pr_err("fail to initial sensor (%d)\n", err);
		goto err_sensor_setup;
	}

#if PS_INTERRUPT_MODE
	err = setup_interrupt(epld);
	if (err < 0) {
		pr_err("setup error!\n");
		goto err_sensor_setup;
	}
#endif
	wake_lock_init(&g_ps_wlock, WAKE_LOCK_SUSPEND, "ps_wakelock");
	sensor_dev = platform_device_register_simple("elan_alsps", -1, NULL, 0);
    if (IS_ERR(sensor_dev))
    {
        dev_err(&client->dev, "sensor_dev_init: error\n");
        goto err_fail;
    }

    err = sysfs_create_group(&sensor_dev->dev.kobj, &ets_attr_group);
    if (err !=0)
    {
        dev_err(&client->dev,"%s:create sysfs group error", __func__);
        goto err_fail;
    }
	

             err = sysfs_create_group(&client->dev.kobj, &ets_attr_group);
	if (err != 0) {
		dev_err(&client->dev, "%s:create sysfs group error", __func__);
		goto err_fail;
	}

	/* Register to sensors class */
	epld->als_cdev = sensors_light_cdev;
	epld->als_cdev.sensors_enable = elan_als_set_enable;
	epld->als_cdev.sensors_poll_delay = elan_als_poll_delay;

#ifndef NO_P_SENSOR
	epld->ps_cdev = sensors_proximity_cdev;
	epld->ps_cdev.sensors_enable = elan_ps_set_enable;
	epld->ps_cdev.sensors_poll_delay = elan_ps_poll_delay;
#endif    

	ret = sensors_classdev_register(&epld->als_input_dev->dev, &epld->als_cdev);
	if (ret) {
		pr_err("%s: Unable to register to sensors class: %d\n",
		       __func__, ret);
		goto exit_remove_sysfs_group;
	}

#ifndef NO_P_SENSOR
	ret = sensors_classdev_register(&client->dev, &epld->ps_cdev);
	if (ret) {
		pr_err("%s: Unable to register to sensors class: %d\n",
		       __func__, ret);
		goto exit_create_class_sysfs;
	}
#endif

	epl_info("sensor probe success.\n");

	return err;

#ifndef NO_P_SENSOR
exit_create_class_sysfs:
	sensors_classdev_unregister(&epld->als_cdev);
#endif
exit_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &ets_attr_group);
err_fail:
	free_irq(client->irq, client);
err_sensor_setup:
    #ifndef NO_P_SENSOR
	misc_deregister(&elan_ps_device);
	input_unregister_device(epld->ps_input_dev);
	input_free_device(epld->ps_input_dev);
err_psensor_setup:
    #endif
	input_unregister_device(epld->als_input_dev);
	input_free_device(epld->als_input_dev);
	misc_deregister(&elan_als_device);
err_lightsensor_setup:
	destroy_workqueue(epld->epl_wq);
err_create_singlethread_workqueue:
i2c_fail:
	kfree(epld);
	return err;
}

static int elan_sensor_remove(struct i2c_client *client)
{
	struct elan_epl_data *epld = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s: enter.\n", __func__);

	input_unregister_device(epld->als_input_dev);
    #ifndef NO_P_SENSOR
	input_unregister_device(epld->ps_input_dev);
    #endif
	input_free_device(epld->als_input_dev);
    #ifndef NO_P_SENSOR
	input_free_device(epld->ps_input_dev);
	misc_deregister(&elan_ps_device);
    #endif
	misc_deregister(&elan_als_device);
	free_irq(epld->irq_gpio, epld);
	destroy_workqueue(epld->epl_wq);
	kfree(epld);
	return 0;
}

static const struct i2c_device_id elan_sensor_id[] = {
	{ELAN_LS_2182, 0},
	{}
};

static struct of_device_id elan_match_table[] = {
	{.compatible = "elan,epl2182",},
	{},
};

static struct i2c_driver elan_sensor_driver = {
	.probe = elan_sensor_probe,
	.remove = elan_sensor_remove,
	.suspend = elan_sensor_suspend,
	.resume = elan_sensor_resume,
	.driver = {
		   .name = ELAN_LS_2182,
		   .owner = THIS_MODULE,
		   .of_match_table = elan_match_table,
		   },
	.id_table = elan_sensor_id,
};

static int __init elan_sensor_init(void)
{
	return i2c_add_driver(&elan_sensor_driver);
}

static void __exit elan_sensor_exit(void)
{
	i2c_del_driver(&elan_sensor_driver);
}

module_init(elan_sensor_init);
module_exit(elan_sensor_exit);

MODULE_AUTHOR("Renato Pan <renato.pan@eminent-tek.com>");
MODULE_DESCRIPTION("ELAN epl2182 driver");
MODULE_LICENSE("GPL");
