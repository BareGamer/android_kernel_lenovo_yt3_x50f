/*
 * Copyright (C) 2012 Senodia.
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
#include "st480.h"
#include <linux/sensors.h>

#define VENDOR_NAME			"SENODIA"
#define MODULE_NAME		"ST480"

#define BIST_SINGLE_MEASUREMENT_MODE_CMD 0x38
#define ONE_INIT_BIST_TEST 0x01
#define BIST_READ_MEASUREMENT_CMD 0x48


struct st480_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct delayed_work work;
	struct sensors_classdev cdev;
	struct class *st480_class;
	struct device *factory_device;
	unsigned int poll_interval;
	rwlock_t lock;
	atomic_t m_flag;
	atomic_t enable;
};

static struct sensors_classdev sensors_cdev = {
	.name = "st480",
	.vendor = "Senodia",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "3200",
	.resolution = "0.1",
	.sensor_power = "0.4",
	.min_delay = 1000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct st480_data *st480;


static atomic_t mv_flag;
static atomic_t rm_flag;
static atomic_t mrv_flag;

static atomic_t open_count;
static atomic_t open_flag;
static atomic_t reserve_open_flag;

volatile static short st480_delay = ST480_DEFAULT_DELAY;

struct mag_3{
	s16  mag_x,
	mag_y,
	mag_z;
};
volatile static struct mag_3 mag;

//static struct kobject *st480_kobj;

/*
 * i2c transfer
 * read/write
 */
static int st480_i2c_transfer_data(struct i2c_client *client, int len, char *buf, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr  =  client->addr,
			.flags  =  0,
			.len  =  len,
			.buf  =  buf,
		},
		{
			.addr  =  client->addr,
			.flags  = I2C_M_RD,
			.len  =  length,
			.buf  =  buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		dev_err(&client->dev, "%s error:%d\n", __func__, ret);
	return (ret == 2) ? 0 : ret;
}

/*
 * Device detect and init
 *
 */
static int st480_setup(struct i2c_client *client)
{
	int ret;
	unsigned char buf[5];

	memset(buf, 0, 5);

	buf[0] = READ_REGISTER_CMD;
	buf[1] = 0x00;
	ret = 0;

#ifdef IC_CHECK
	while (st480_i2c_transfer_data(client, 2, buf, 3) != 0)
	{
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 2, buf, 3)==0)
		{
			break;
		}
		if (ret > MAX_FAILURE_COUNT)
		{
			return -EIO;
		}
	}

	if (buf[2] != ST480_DEVICE_ID)
	{
		return -ENODEV;
	}
#endif

//init register step 1
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = ONE_INIT_DATA_HIGH;
	buf[2] = ONE_INIT_DATA_LOW;
	buf[3] = ONE_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1) != 0)
	{
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1)==0)
		{
			break;
		}
		if (ret > MAX_FAILURE_COUNT)
		{
			return -EIO;
		}
	}

//init register step 2
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = TWO_INIT_DATA_HIGH;
	buf[2] = TWO_INIT_DATA_LOW;
	buf[3] = TWO_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1)!=0) {
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1)==0)
		{
			break;
		}
		if (ret > MAX_FAILURE_COUNT)
		{
			return -EIO;
		}
	}

//disable temperature compensation register
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = TEMP_DATA_HIGH;
	buf[2] = TEMP_DATA_LOW;
	buf[3] = TEMP_REG;

	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1)!=0)
	{
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1)==0)
		{
			break;
		}
		if (ret > MAX_FAILURE_COUNT)
		{
			return -EIO;
		}
	}

//set calibration register
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = CALIBRATION_DATA_HIGH;
	buf[2] = CALIBRATION_DATA_LOW;
	buf[3] = CALIBRATION_REG;
	ret = 0;
	while (st480_i2c_transfer_data(client, 4, buf, 1)!=0)
	{
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 4, buf, 1)==0)
		{
			break;
		}
		if (ret > MAX_FAILURE_COUNT)
		{
			return -EIO;
		}
	}

//set mode config
	buf[0] = SINGLE_MEASUREMENT_MODE_CMD;
	ret=0;
	while (st480_i2c_transfer_data(client, 1, buf, 1)!=0)
	{
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(client, 1, buf, 1)==0)
		{
			break;
		}
		if( ret > MAX_FAILURE_COUNT)
		{
			return -EIO;
		}
	}

	return 0;
}

static void set_sensor_attr(struct device *dev,
                struct device_attribute *attributes[])
{
        int i;

        for (i = 0; attributes[i] != NULL; i++)
                if ((device_create_file(dev, attributes[i])) < 0)
                        pr_err("[SENSOR CORE] fail device_create_file"\
                                "(dev, attributes[%d])\n", i);
}

int sensors_register(struct class *st480_class, struct device *dev, void *drvdata,
                struct device_attribute *attributes[], char *name)
{
        int ret = 0;

        dev = device_create(st480_class, NULL, 0, drvdata, "%s", name);
        if (IS_ERR(dev)) {
                ret = PTR_ERR(dev);
                pr_err("[SENSORS CORE] device_create failed![%d]\n", ret);
                return ret;
        }

        set_sensor_attr(dev, attributes);

        return ret;
}

void sensors_unregister(struct device *dev,
                struct device_attribute *attributes[])
{
        int i;

        for (i = 0; attributes[i] != NULL; i++)
                device_remove_file(dev, attributes[i]);
}

static void st480_work_func(void)
{
	char buffer[9];
	int ret;

	memset(buffer, 0, 9);

	buffer[0] = READ_MEASUREMENT_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 1, buffer, 9)!=0)
	{
		ret++;
		msleep(1);
		if (st480_i2c_transfer_data(st480->client, 1, buffer, 9)==0)
		{
			break;
		}
		if(ret > MAX_FAILURE_COUNT)
		{
			return;
		}
	}
	printk("st480 run in \n");

	if (!((buffer[0]>>4) & 0X01)) {
		if (ST480MB_SIZE_2X2)
		{
                #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_0)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_90)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_180)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_270)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #endif
                #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_0)
				mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_90)
				mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_180)
				mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_270)
				mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #endif
                #endif
		}
		else if(ST480MW_SIZE_1_6X1_6)
		{
		#if defined (CONFIG_ST480_BOARD_LOCATION_FRONT)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_0)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_90)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_180)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_270)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #endif
                #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_0)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_90)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_180)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_270)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #endif
                #endif
		}
		else if(ST480MC_SIZE_1_2X1_2)
		{
		#if defined (CONFIG_ST480_BOARD_LOCATION_FRONT)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_0)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_90)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_180)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_270)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #endif
                #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_0)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_90)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_180)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_270)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #endif
                #endif
		}

		if (((buffer[1]<<8)|(buffer[2])) > 46244)
		{
			mag.mag_x = mag.mag_x * (1 + (70/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
			mag.mag_y = mag.mag_y * (1 + (70/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
			mag.mag_z = mag.mag_z * (1 + (70/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
		}
		else if (((buffer[1]<<8)|(buffer[2])) < 46244)
		{
			mag.mag_x = mag.mag_x * (1 + (60/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
			mag.mag_y = mag.mag_y * (1 + (60/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
			mag.mag_z = mag.mag_z * (1 + (60/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
		}

		SENODIADBG("st480 raw data: x = %d, y = %d, z = %d \n",mag.mag_x,mag.mag_y,mag.mag_z);
		printk("st480 raw data: x = %d, y = %d, z = %d \n",mag.mag_x,mag.mag_y,mag.mag_z);

		
		}else 
			dev_err(&st480->client->dev, "ecc error detected!\n");

			buffer[0] = SINGLE_MEASUREMENT_MODE_CMD;
			ret=0;

			while (st480_i2c_transfer_data(st480->client, 1, buffer, 1)!=0)
		{
			ret++;
			msleep(1);
			if (st480_i2c_transfer_data(st480->client, 1, buffer, 1)==0)
			{
				break;
			}
			if (ret > MAX_FAILURE_COUNT)
			{
				return;
			}
		  }
	
	//set mode config
	
}

static void st480_input_func(struct work_struct *work)
{
	struct st480_data *st480 = container_of((struct delayed_work *)work, struct st480_data, work);

	SENODIAFUNC("st480_input_func");
	st480_work_func();

	if (atomic_read(&st480->m_flag) || atomic_read(&mv_flag) || atomic_read(&rm_flag) || atomic_read(&mrv_flag)) {
		input_report_abs(st480->input_dev, ABS_X, mag.mag_x);
		input_report_abs(st480->input_dev, ABS_Y, mag.mag_y);
		input_report_abs(st480->input_dev, ABS_Z, mag.mag_z);
		input_sync(st480->input_dev);
	}

	schedule_delayed_work(&st480->work, msecs_to_jiffies(st480->poll_interval));
}

static void ecs_closedone(void)
{
	SENODIAFUNC("ecs_closedone");
	// atomic_set(&m_flag, 0);
	atomic_set(&mv_flag, 0);
	atomic_set(&rm_flag, 0);
	atomic_set(&mrv_flag, 0);
}

/***** st480 functions ***************************************/
static int st480_open(struct inode *inode, struct file *file)
{
	int ret = -1;

	SENODIAFUNC("st480_open");

	if (atomic_cmpxchg(&open_count, 0, 1) == 0) {
		if (atomic_cmpxchg(&open_flag, 0, 1) == 0) {
			atomic_set(&reserve_open_flag, 1);
			ret = 0;
		}
	}

	if (atomic_read(&reserve_open_flag))
		schedule_delayed_work(&st480->work, msecs_to_jiffies(st480_delay));
	return ret;
}

static int st480_release(struct inode *inode, struct file *file)
{
	SENODIAFUNC("st480_release");

	atomic_set(&reserve_open_flag, 0);
	atomic_set(&open_flag, 0);
	atomic_set(&open_count, 0);

	ecs_closedone();

	cancel_delayed_work(&st480->work);
	return 0;
}

#if OLD_KERNEL_VERSION
static int
st480_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
#else
static long
st480_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	void __user *argp = (void __user *)arg;
	short flag;

	SENODIADBG("enter %s\n", __func__);

	switch (cmd) {
		case MSENSOR_IOCTL_ST480_SET_MFLAG:
		case MSENSOR_IOCTL_ST480_SET_MVFLAG:
		case MSENSOR_IOCTL_ST480_SET_RMFLAG:
		case MSENSOR_IOCTL_ST480_SET_MRVFLAG:
			if (copy_from_user(&flag, argp, sizeof(flag))) {
				return -EFAULT;
			}
			if (flag < 0 || flag > 1) {
				return -EINVAL;
			}
			break;
		case MSENSOR_IOCTL_ST480_SET_DELAY:
			if (copy_from_user(&flag, argp, sizeof(flag))) {
				return -EFAULT;
			}
			break;
		default:
			break;
	}

	switch (cmd) {
		case MSENSOR_IOCTL_ST480_SET_MFLAG:
			//atomic_set(&m_flag, flag);
			SENODIADBG("MFLAG is set to %d", flag);
			break;
		case MSENSOR_IOCTL_ST480_GET_MFLAG:
			//flag = atomic_read(&m_flag);
			SENODIADBG("Mflag = %d\n",flag);
			break;
		case MSENSOR_IOCTL_ST480_SET_MVFLAG:
			atomic_set(&mv_flag, flag);
			SENODIADBG("MVFLAG is set to %d", flag);
			break;
		case MSENSOR_IOCTL_ST480_GET_MVFLAG:
			flag = atomic_read(&mv_flag);
			SENODIADBG("MVflag = %d\n",flag);
			break;
		case MSENSOR_IOCTL_ST480_SET_RMFLAG:
			atomic_set(&rm_flag, flag);
			SENODIADBG("RMFLAG is set to %d", flag);
			break;
		case MSENSOR_IOCTL_ST480_GET_RMFLAG:
			flag = atomic_read(&rm_flag);
			SENODIADBG("RMflag = %d\n",flag);
			break;
		case MSENSOR_IOCTL_ST480_SET_MRVFLAG:
			atomic_set(&mrv_flag, flag);
			SENODIADBG("MRVFLAG is set to %d", flag);
			break;
		case MSENSOR_IOCTL_ST480_GET_MRVFLAG:
			flag = atomic_read(&mrv_flag);
			SENODIADBG("MRVflag = %d\n",flag);
			break;
		case MSENSOR_IOCTL_ST480_SET_DELAY:
			st480_delay = flag;
			break;
		case MSENSOR_IOCTL_ST480_GET_DELAY:
			flag = st480_delay;
			break;
		default:
			return -ENOTTY;
	}

	switch (cmd) {
		case MSENSOR_IOCTL_ST480_GET_MFLAG:
		case MSENSOR_IOCTL_ST480_GET_MVFLAG:
		case MSENSOR_IOCTL_ST480_GET_RMFLAG:
		case MSENSOR_IOCTL_ST480_GET_MRVFLAG:
		case MSENSOR_IOCTL_ST480_GET_DELAY:
			if (copy_to_user(argp, &flag, sizeof(flag))) {
				return -EFAULT;
			}
			break;
		default:
			break;
	}

	return 0;
}


static struct file_operations st480_fops = {
	.owner = THIS_MODULE,
	.open = st480_open,
	.release = st480_release,
#if OLD_KERNEL_VERSION
	.ioctl = st480_ioctl,
#else
	.unlocked_ioctl = st480_ioctl,
#endif
};


static struct miscdevice st480_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "st480",
	.fops = &st480_fops,
};

/*********************************************/
#if ST480_AUTO_TEST
static int sensor_test_read(void)

{
        st480_work_func();
        return 0;
}

static int auto_test_read(void *unused)
{
        while(1){
                sensor_test_read();
                msleep(200);
        }
        return 0;
}
#endif

static int st480_set_enable(struct st480_data *st480, bool on)
{
	int retval = 0;
	struct i2c_client *client = st480->client;

	dev_info(&client->dev, "enable:%s\n", on ? "on" : "off");

	if (on) {
		schedule_delayed_work(&st480->work, msecs_to_jiffies(st480->poll_interval));
		atomic_set(&st480->enable, 1);
	} else {
		cancel_delayed_work(&st480->work);
		atomic_set(&st480->enable, 0);
	}
	return retval;
}

static int st480_set_poll_delay(struct st480_data *st480, unsigned int msecs)
{
	/* FIXME: must larger than ST480 minimun delay 28 ms, otherwise ecc error */
	if (msecs <= 28)
		msecs = 28;

	write_lock(&st480->lock);
	st480->poll_interval = msecs;
	write_unlock(&st480->lock);

	return 0;
}

static int st480_cdev_set_enable(struct sensors_classdev *sensors_cdev,
							unsigned int enable)
{
	struct st480_data *st480 =
		container_of(sensors_cdev, struct st480_data, cdev);

	return st480_set_enable(st480, enable);
}

static int st480_cdev_set_poll_delay(struct sensors_classdev *sensors_cdev,
								unsigned int msecs)
{
	struct st480_data *st480 =
		container_of(sensors_cdev, struct st480_data, cdev);

	if (msecs != st480->poll_interval)
		return st480_set_poll_delay(st480, msecs);

	return 0;
}

static int st480_single_measure_mode_bist(void)
{
	char buffer[1];
	int ret;

	/*set mode config*/
	buffer[0] = BIST_SINGLE_MEASUREMENT_MODE_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 1, buffer, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(st480->client, 1, buffer, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}
	return 0;
}

static ssize_t st480_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

static ssize_t st480_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", MODULE_NAME);
}

static ssize_t st480_selftest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	int sf_ret = 0;
	unsigned char buffer[4] = {0,0,0,0};
	s16 zh[3] = {0};

	pr_info("[SENSOR]: %s start\n", __func__);

	if (atomic_read(&st480->enable) == 1)
		cancel_delayed_work(&st480->work);

	usleep_range(30000, 30100);

	zh[0] = mag.mag_z;

/*self test*/
	buffer[0] = WRITE_REGISTER_CMD;
	buffer[1] = ONE_INIT_BIST_TEST;
	buffer[2] = ONE_INIT_DATA_LOW;
	buffer[3] = ONE_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 4, buffer, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(st480->client, 4, buffer, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT) {
			sf_ret = -1;
			pr_err("[SENSOR]: %s i2c transfer data error\n",__func__);
		}
	}

	usleep_range(150000, 150100);
	st480_single_measure_mode_bist();

	usleep_range(30000, 30100);

	buffer[0] = BIST_READ_MEASUREMENT_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 1, buffer, 3) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(st480->client, 1, buffer, 3) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT) {
			sf_ret = -1;
			pr_err("[SENSOR]: %s i2c transfer data error\n",__func__);
		}
	}

	if (((buffer[0]>>4) & 0X01)) {
		sf_ret = -1;
		pr_err("[SENSOR]: %s error\n", __func__);
	}

	zh[1] = (buffer[1]<<8)|buffer[2];

	zh[2] = (abs(zh[1])) - (abs(zh[0]));

	pr_info("[SENSOR]: %s zh[0] = %d, zh[1] = %d, zh[2] = %d\n",
		__func__, zh[0], zh[1], zh[2]);

/*Reduce to the initial setup*/
	buffer[0] = WRITE_REGISTER_CMD;
	buffer[1] = ONE_INIT_DATA_HIGH;
	buffer[2] = ONE_INIT_DATA_LOW;
	buffer[3] = ONE_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 4, buffer, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(st480->client, 4, buffer, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT) {
			sf_ret = -1;
			pr_err("[SENSOR]: %s i2c transfer data error\n",__func__);
		}
	}

/*set mode*/
	buffer[0] = SINGLE_MEASUREMENT_MODE_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(st480->client, 1, buffer, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(st480->client, 1, buffer, 1) == 0)
			break;

		if (ret > MAX_FAILURE_COUNT){
			sf_ret = -1;
			pr_err("[SENSOR]: %s i2c transfer data error\n",__func__);
		}
	}

	if ((abs(zh[2]) <= 90) || (abs(zh[2]) >= 180)) {
		sf_ret = -1;
		pr_err("[SENSOR]: %s BIST test error\n", __func__);
	}
	if (sf_ret == 0)
		pr_info("[SENSOR]: %s success\n", __func__);
	else
		pr_info("[SENSOR]: %s fail\n", __func__);

	if (atomic_read(&st480->enable) == 1) {
		schedule_delayed_work(&st480->work,
				msecs_to_jiffies(st480->poll_interval));
	}

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
			zh[0], zh[1], zh[2]);
}

static DEVICE_ATTR(name, S_IRUGO, st480_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, st480_vendor_show, NULL);
static DEVICE_ATTR(selftest, S_IRUGO, st480_selftest, NULL);

static struct device_attribute *sensor_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_selftest,
	NULL,
};

static int st480_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
#if ST480_AUTO_TEST
        struct task_struct *thread;
#endif

	SENODIAFUNC("st480_probe");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "SENODIA st480_probe: check_functionality failed.\n");
		err = -ENODEV;
		goto exit0;
	}

	/* Allocate memory for driver data */
	st480 = kzalloc(sizeof(struct st480_data), GFP_KERNEL);
	if (!st480) {
		printk(KERN_ERR "SENODIA st480_probe: memory allocation failed.\n");
		err = -ENOMEM;
		goto exit1;
	}

	st480->client = client;

	i2c_set_clientdata(client, st480);

	client->addr = 0x0c;
	
	INIT_DELAYED_WORK(&st480->work, st480_input_func);

	if (st480_setup(st480->client) != 0) {
		dev_err(&client->dev, "st480 setup error!\n");
		goto exit2;
	}

	/* Declare input device */
	st480->input_dev = input_allocate_device();
	if (!st480->input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit3;
	}

	/* Setup input device */
	set_bit(EV_ABS, st480->input_dev->evbit);

	/* x-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(st480->input_dev, ABS_X, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
	/* y-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(st480->input_dev, ABS_Y, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
	/* z-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(st480->input_dev, ABS_Z, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
	/* Set name */
	st480->input_dev->name = "compass";
	st480->input_dev->id.bustype = BUS_I2C;

	/* Register */
	err = input_register_device(st480->input_dev);
	if (err) {
		dev_err(&client->dev, "SENODIA st480_probe: Unable to register input device\n");
		goto exit4;
	}

	err = misc_register(&st480_device);
	if (err) {
		dev_err(&client->dev, "SENODIA st480_probe: st480_device register failed\n");
		goto exit5;
	}

	rwlock_init(&st480->lock);
	/* As default, report all information */
	atomic_set(&st480->m_flag, 1);
	atomic_set(&mv_flag, 1);
	atomic_set(&rm_flag, 1);
	atomic_set(&mrv_flag, 1);
	st480->poll_interval = ST480_DEFAULT_DELAY;
	st480->cdev = sensors_cdev;
	st480->cdev.sensors_enable = st480_cdev_set_enable;
	st480->cdev.sensors_poll_delay = st480_cdev_set_poll_delay;
	err = sensors_classdev_register(&client->dev, &st480->cdev);
	if (err) {
		dev_err(&client->dev, "sensors class register failed!\n");
		goto exit6;
	}

	st480->st480_class = class_create(THIS_MODULE, "st480");
	if (IS_ERR(st480->st480_class)) {
		pr_err("%s, create st480_class is failed.(err=%ld)\n",
				__func__, IS_ERR(st480->st480_class));
		goto exit7;
	}

	err = sensors_register(st480->st480_class, st480->factory_device, st480, 
				sensor_attrs, MODULE_NAME);
	if (err) {
		pr_err("%s, failed to sensors_register (%d)\n", 
			__func__, err);
		goto exit8;
	}

#if ST480_AUTO_TEST
	thread=kthread_run(auto_test_read,NULL,"st480_read_test");
#endif

	printk("st480 probe done.");
	return 0;

exit8:
	class_destroy(st480->st480_class);
exit7:
exit6:
	misc_deregister(&st480_device);
exit5:
	input_unregister_device(st480->input_dev);
exit4:
	input_free_device(st480->input_dev);
exit3:
exit2:
	kfree(st480);
exit1:
exit0:
	return err;

}

static int st480_remove(struct i2c_client *client)
{
	struct st480_data *st480 = i2c_get_clientdata(client);

	sensors_classdev_unregister(&st480->cdev);
	atomic_set(&st480->m_flag, 0);
	misc_deregister(&st480_device);
	input_unregister_device(st480->input_dev);
	input_free_device(st480->input_dev);
	cancel_delayed_work(&st480->work);
	i2c_set_clientdata(client, NULL);
	class_destroy(st480->st480_class);
	sensors_unregister(st480->factory_device, sensor_attrs);
	kfree(st480);
	return 0;
}

static const struct i2c_device_id st480_id_table[] = {
	{ ST480_I2C_NAME, 0 },
	{ },
};

static struct of_device_id st480_match_table[] = {
	{ .compatible = "senodia,st480", },
	{ },
};

static struct i2c_driver st480_driver = {
	.probe		= st480_probe,
	.remove 	= st480_remove,
	.id_table	= st480_id_table,
	.driver = {
		.name = ST480_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = st480_match_table,
	},
};

static int __init st480_init(void)
{
	return i2c_add_driver(&st480_driver);
}

static void __exit st480_exit(void)
{
	i2c_del_driver(&st480_driver);
}

module_init(st480_init);
module_exit(st480_exit);

MODULE_AUTHOR("Tori Xu <xuezhi_xu@senodia.com>");
MODULE_DESCRIPTION("senodia st480 linux driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.0.0");
