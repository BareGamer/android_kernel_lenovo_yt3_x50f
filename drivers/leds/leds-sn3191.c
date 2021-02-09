/*
 * File name: leds-sn3191.c
 * Description	: Driver for breath led SN3191, SI-EN.
 *
 * Author: xuke@longcheer.net (Jacky Xu)
 *
 * Version:
 * 2014-11-22 Add some interfaces requested by Meizu.
 * 2014-11-11 Original version, support the base light-on by PWM & one shot method.
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#else
#include <linux/earlysuspend.h>
#endif
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/leds.h>
#include <linux/kthread.h>

#define VTG_MIN_UV		2850000
#define VTG_MAX_UV		2850000
#define I2C_VTG_MIN_UV	1800000
#define I2C_VTG_MAX_UV	1800000

#define SN_REG_RESET		0x2f
#define SN_REG_POWER		0x00
#define SN_REG_SET_MODE		0x02
#define SN_REG_CURRENT		0x03
#define SN_REG_PWM_VAL		0x04
#define SN_REG_SET_DATA		0x07
#define SN_REG_SELF_T0		0x0a
#define SN_REG_SELF_T1_T2	0x10
#define SN_REG_SELF_T3_T4	0x16
#define SN_REG_TIME_UPDATE	0x1c

#define SN_CURRENT_5mA		0x08
#define SN_CURRENT_10mA		0x04
#define SN_CURRENT_17mA		0x10
#define SN_CURRENT_30mA		0x0c
#define SN_CURRENT_42mA		0x00

#define SN_TIME_NOT_SET	0xff

#define SN_T0_0s		0x00
#define SN_T0_0_13s		0x10
#define SN_T0_0_26s		0x20
#define SN_T0_0_52s		0x30
#define SN_T0_1_04s		0x40
#define SN_T0_2_08s		0x50
#define SN_T0_4_16s		0x60
#define SN_T0_8_32s		0x70
#define SN_T0_16_64s	0x80
#define SN_T0_33_28s	0x90
#define SN_T0_66_56s	0xa0

#define SN_T1_0_13s		0x00
#define SN_T1_0_26s		0x20
#define SN_T1_0_52s		0x40
#define SN_T1_1_04s		0x60
#define SN_T1_2_08s		0x80
#define SN_T1_4_16s		0xa0
#define SN_T1_8_32s		0xc0
#define SN_T1_16_64s	0xe0

#define SN_T2_0s		0x00
#define SN_T2_0_13s		0x02
#define SN_T2_0_26s		0x04
#define SN_T2_0_52s		0x06
#define SN_T2_1_04s		0x08
#define SN_T2_2_08s		0x0a
#define SN_T2_4_16s		0x0c
#define SN_T2_8_32s		0x0e
#define SN_T2_16_64s	0x10

#define SN_T3_0_13s		0x00
#define SN_T3_0_26s		0x20
#define SN_T3_0_52s		0x40
#define SN_T3_1_04s		0x60
#define SN_T3_2_08s		0x80
#define SN_T3_4_16s		0xa0
#define SN_T3_8_32s		0xc0
#define SN_T3_16_64s	0xe0

#define SN_T4_0s		0x00
#define SN_T4_0_13s		0x02
#define SN_T4_0_26s		0x04
#define SN_T4_0_52s		0x06
#define SN_T4_1_04s		0x08
#define SN_T4_2_08s		0x0a
#define SN_T4_4_16s		0x0c
#define SN_T4_8_32s		0x0e
#define SN_T4_16_64s	0x10
#define SN_T4_33_28s	0x12
#define SN_T4_66_56s	0x14

enum sn3191_led_breath_mode {
	SN_MODE_PWM = 0,
	SN_MODE_ONE_SHOT,
};

enum flash_by_touch_state {
	Brightness_Down = 0,
	Brightness_Upper,
};

struct sn3191_breath_ctrl {
	u8 t0;		// start
	u8 t1;		// rise
	u8 t2;		// hold high
	u8 t3;		// sink
	u8 t4;		// stop
};

struct sn3191_led_pdata {
	struct i2c_client *client;
	struct led_classdev led_cdev;
#if defined(CONFIG_FB)
	struct notifier_block led_fb_notif;
#elif defined (CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend led_early_suspend;
#endif
	struct mutex led_lock;
	struct delayed_work led_work;
	u32 led_brightness;
	u32 led_current;
	u32 led_breath_mode;
	struct sn3191_breath_ctrl ctrl;
	struct delayed_work pwm_work;
	unsigned long pwm_interval;
	u32 led_brightness_base;
	enum flash_by_touch_state brightness_state;
	
	struct task_struct *pwm_thread;
	
	int sdb_gpio;
	u32 sdb_gpio_flags;
	int power_ldo_gpio;
	u32 power_ldo_gpio_flags;
	bool i2c_pull_up;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	struct pinctrl *led_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

/* Effect flash with breath. */
struct sn3191_breath_ctrl led_effect_flash = {SN_T0_0_26s, SN_T1_1_04s, SN_T2_0s, SN_T3_1_04s, SN_T4_1_04s};
struct sn3191_breath_ctrl led_effect_flash2 = {SN_T0_0_26s, SN_T1_1_04s, SN_T2_0s, SN_T3_1_04s, SN_T4_4_16s};
//struct sn3191_breath_ctrl led_effect_flash2 = {SN_T0_0s, SN_T1_1_04s, SN_T1_1_04s, SN_T3_1_04s, SN_T4_1_04s};

static struct timer_list s_timer;
static int s_timer_flag = 0;

static void second_timer_handle(unsigned long arg)
{
	s_timer_flag = 1;
	printk("zhouhehe-second_timer_handle start \n");
}

int sn3191_led_off(struct sn3191_led_pdata *data)
{
	struct sn3191_led_pdata *pdata = data;
	int ret = 0;

	pr_debug("%s\n", __func__);
	mutex_lock(&pdata->led_lock);
	ret = i2c_smbus_write_byte_data(pdata->client, SN_REG_RESET, 0xff);		// reset
	if (!ret)
	{
		i2c_smbus_write_byte_data(pdata->client, SN_REG_POWER, 0x00);		// power off
	}
	mutex_unlock(&pdata->led_lock);

	gpio_set_value(pdata->sdb_gpio, 0);

	return ret;
}

void sn3191_led_pwm_ctrl(struct sn3191_led_pdata *data, bool power)
{
	struct sn3191_led_pdata *pdata = data;
	bool on_off = power;

	pr_debug("%s, on=%d, led_current=%d, led_brightness=%d\n", __func__, on_off, pdata->led_current, pdata->led_brightness);
	mutex_lock(&pdata->led_lock);
	if (on_off)
	{
		gpio_set_value(pdata->sdb_gpio, 1);
		i2c_smbus_write_byte_data(pdata->client, SN_REG_SET_MODE, 0x00);	// PWM mode
		i2c_smbus_write_byte_data(pdata->client, SN_REG_POWER, 0x20);		// power on
		i2c_smbus_write_byte_data(pdata->client, SN_REG_CURRENT, pdata->led_current);		// set current to 17.5mA
		i2c_smbus_write_byte_data(pdata->client, SN_REG_PWM_VAL, pdata->led_brightness);	// set PWM value
		i2c_smbus_write_byte_data(pdata->client, SN_REG_SET_DATA, 0xff);	// update register
	}
	else
	{
		i2c_smbus_write_byte_data(pdata->client, SN_REG_SET_MODE, 0x00);	// PWM mode
		i2c_smbus_write_byte_data(pdata->client, SN_REG_POWER, 0x00);		// power off
		gpio_set_value(pdata->sdb_gpio, 0);
	}
	mutex_unlock(&pdata->led_lock);
}

void sn3191_led_pwm_set_brightness_work(struct work_struct *work)
{
	struct sn3191_led_pdata *pdata = container_of(work, struct sn3191_led_pdata, pwm_work.work);
	int value = 0;
	
	pr_debug("%s, led_brightness_base=%d, led_brightness=%d, brightness_state=%d\n", 
		__func__, pdata->led_brightness_base, pdata->led_brightness, pdata->brightness_state);

	mutex_lock(&pdata->led_lock);
	if (pdata->brightness_state == Brightness_Upper)
	{
		value = ++pdata->led_brightness;
		// The top brightness is 2.5 times of the original value, requested by Meizu.
		if ((value >= ((pdata->led_brightness_base<<1) + (pdata->led_brightness_base>>1))) || (value >= 0xff))
		{
			pdata->brightness_state = Brightness_Down;
		}
	}
	else
	{
		value = --pdata->led_brightness;
	}
	
	i2c_smbus_write_byte_data(pdata->client, SN_REG_PWM_VAL, value);	// set PWM value
	i2c_smbus_write_byte_data(pdata->client, SN_REG_SET_DATA, 0xff);	// update register
	mutex_unlock(&pdata->led_lock);

	pr_debug("%s, value=%d, led_brightness_base=%d, led_brightness=%d\n", 
		__func__, value, pdata->led_brightness_base, pdata->led_brightness);
	if (pdata->led_brightness_base != pdata->led_brightness)
	{
		schedule_delayed_work(&pdata->pwm_work, pdata->pwm_interval);
	}
}

int sn3191_led_pwm_set_brightness_thread(void *data)
{
	struct sn3191_led_pdata *pdata = data;
	unsigned int value = 0;
	int ret = 0;
	
	pr_debug("%s, led_brightness_base=%d, led_brightness=%d, brightness_state=%d\n", 
		__func__, pdata->led_brightness_base, pdata->led_brightness, pdata->brightness_state);

	do
	{
		pr_debug("%s, do touch breath, brightness_state=%d\n", __func__, pdata->brightness_state);
		if (mutex_lock_interruptible(&pdata->led_lock) < 0)
			break;
		do
		{
			if (pdata->brightness_state == Brightness_Upper)
			{
				if (pdata->led_brightness < 0xff)
					value = ++pdata->led_brightness;
				// The top brightness is 2.5 times of the original value, requested by Meizu.
				if ((value >= ((pdata->led_brightness_base<<1) + (pdata->led_brightness_base>>1))) || (value >= 0xff))
					pdata->brightness_state = Brightness_Down;
			}
			else
			{
				if (pdata->led_brightness)
					value = --pdata->led_brightness;
				if ((value <= pdata->led_brightness_base) || (value == 0))
					break;
			}
			
			i2c_smbus_write_byte_data(pdata->client, SN_REG_PWM_VAL, value);	// set PWM value
			i2c_smbus_write_byte_data(pdata->client, SN_REG_SET_DATA, 0xff);	// update register
			
			pr_debug("%s, value=%d, led_brightness_base=%d, led_brightness=%d\n", 
				__func__, value, pdata->led_brightness_base, pdata->led_brightness);
		} while (pdata->led_brightness_base != pdata->led_brightness);
		
		mutex_unlock(&pdata->led_lock);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}while (!kthread_should_stop());

	return ret;
}

void sn3191_led_self_breath_ctrl(struct sn3191_led_pdata *data, bool power)
{
	struct sn3191_led_pdata *pdata = data;
	bool on_off = power;

	if (on_off)
	{
		gpio_set_value(pdata->sdb_gpio, 1);
		
		mutex_lock(&pdata->led_lock);
		i2c_smbus_write_byte_data(pdata->client, SN_REG_SET_MODE, 0x20);		// one shot mode
		i2c_smbus_write_byte_data(pdata->client, SN_REG_POWER, 0x20);		// power on
		i2c_smbus_write_byte_data(pdata->client, SN_REG_CURRENT, pdata->led_current);		// set current
		
		i2c_smbus_write_byte_data(pdata->client, SN_REG_SELF_T0, pdata->ctrl.t0);		// set T0
		i2c_smbus_write_byte_data(pdata->client, SN_REG_SELF_T1_T2, pdata->ctrl.t1|pdata->ctrl.t2);		// set T1 & T2
		i2c_smbus_write_byte_data(pdata->client, SN_REG_SELF_T3_T4, pdata->ctrl.t3|pdata->ctrl.t4);		// set T3 & T4
		i2c_smbus_write_byte_data(pdata->client, SN_REG_TIME_UPDATE, 0x00);		// time update

		i2c_smbus_write_byte_data(pdata->client, SN_REG_PWM_VAL, 0xff);		// one shot mode
		i2c_smbus_write_byte_data(pdata->client, SN_REG_SET_DATA, 0xff);		// power on
		mutex_unlock(&pdata->led_lock);
	}
	else
	{
		sn3191_led_off(pdata);
	}
}

void sn3191_led_ctrl(struct sn3191_led_pdata *data, int mode, bool power)
{
	struct sn3191_led_pdata *pdata = data;
	bool on_off = power;
	
	switch (mode)
	{
		case SN_MODE_PWM:
			sn3191_led_pwm_ctrl(pdata, on_off);
			break;
		case SN_MODE_ONE_SHOT:
			sn3191_led_self_breath_ctrl(pdata, on_off);
			break;
		default:
			break;
	}
}

static void sn3191_led_set_brightness(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);

	pr_debug("%s, value=%d\n", __func__, value);
	
	mutex_lock(&pdata->led_lock);
	pdata->led_brightness = led_cdev->brightness = value;
	mutex_unlock(&pdata->led_lock);

	if (pdata->led_brightness)
		sn3191_led_ctrl(pdata, pdata->led_breath_mode, true);
	else
		sn3191_led_ctrl(pdata, pdata->led_breath_mode, false);
}
static void sn3191_led_set_brightness_bool(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	if(value)value=255;

	pr_debug("%s, value=%d\n", __func__, value);
	printk("%s, zhouhehe value=%d\n", __func__, value);
	
	mutex_lock(&pdata->led_lock);
	pdata->led_brightness = led_cdev->brightness = value;
	mutex_unlock(&pdata->led_lock);

	if (pdata->led_brightness)
		sn3191_led_ctrl(pdata, pdata->led_breath_mode, true);
	else
		sn3191_led_ctrl(pdata, pdata->led_breath_mode, false);
}

static enum led_brightness sn3191_led_get_brightness(struct led_classdev *led_cdev)
{
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);

	pr_debug("%s, brightness=%d\n", __func__, pdata->led_brightness);

	return pdata->led_brightness;
}

static void sn3191_led_set_breath_param(struct led_classdev *led_cdev, struct sn3191_breath_ctrl *data, bool value)
{
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	bool enable = value;

	pr_debug("%s, t0=0x%x, t1=0x%x, t2=0x%x, t3=0x%x, t4=0x%x\n", __func__, 
		data->t0, data->t1, data->t2, data->t3, data->t4);
	
	mutex_lock(&pdata->led_lock);
	pdata->ctrl = *data;
	mutex_unlock(&pdata->led_lock);
	
	if (enable && (pdata->ctrl.t1 || pdata->ctrl.t2 || pdata->ctrl.t3))
		sn3191_led_self_breath_ctrl(pdata, true);
	else
		sn3191_led_self_breath_ctrl(pdata, false);
}

static struct sn3191_breath_ctrl* sn3191_led_get_breath_param(struct led_classdev *led_cdev)
{
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);

	pr_debug("%s, t0=0x%x, t1=0x%x, t2=0x%x, t3=0x%x, t4=0x%x\n", __func__, 
		pdata->ctrl.t0, pdata->ctrl.t1, pdata->ctrl.t2, pdata->ctrl.t3, pdata->ctrl.t4);

	return &pdata->ctrl;
}

static ssize_t led_set_max_current_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	u32 curr_val = 0;

	pr_debug("%s, current=%d\n", __func__, pdata->led_current);
	switch (pdata->led_current)
	{
		case SN_CURRENT_5mA:
			curr_val = 5;
			break;
		case SN_CURRENT_10mA:
			curr_val = 10;
			break;
		case SN_CURRENT_17mA:
			curr_val = 17;
			break;
		case SN_CURRENT_30mA:
			curr_val = 30;
			break;
		case SN_CURRENT_42mA:
			curr_val = 42;
			break;
		default:
			curr_val = 5;
			break;
	}
	return sprintf(buf, "max current: %dmA\n", curr_val);
}

static ssize_t led_set_max_current_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	unsigned long value;
	int ret = 0;
	u32 curr_val = 0;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;
	
	pr_debug("%s, buf=%s, count=%d, state=%ld\n", __func__, buf, count, value);

	if (value >= 42)
		curr_val = SN_CURRENT_42mA;
	else if (value >= 30)
		curr_val = SN_CURRENT_30mA;
	else if (value >= 17)
		curr_val = SN_CURRENT_17mA;
	else if (value >= 10)
		curr_val = SN_CURRENT_10mA;
	else
		curr_val = SN_CURRENT_5mA;

	mutex_lock(&pdata->led_lock);
	pdata->led_current = curr_val;
	mutex_unlock(&pdata->led_lock);

	if (pdata->led_brightness)
	{
		sn3191_led_pwm_ctrl(pdata, true);
	}

	return count;
}

static DEVICE_ATTR(led_max_current, 0644, led_set_max_current_show, led_set_max_current_store);

static ssize_t led_set_brightness_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);

	pr_debug("%s, brightness=%d\n", __func__, pdata->led_brightness);
	return sprintf(buf, "%d\n", pdata->led_brightness);
}

static ssize_t led_set_brightness_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long value;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;
	
	pr_debug("%s, buf=%s, count=%d, state=%ld\n", __func__, buf, count, value);

	sn3191_led_set_brightness(led_cdev, value);

	return count;
}

static DEVICE_ATTR(led_brightness, 0644, led_set_brightness_show, led_set_brightness_store);

static ssize_t led_set_breath_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);

	pr_debug("%s, brightness=%d\n", __func__, pdata->led_breath_mode);
	return sprintf(buf, "mode: %d (0:pwm,1:one_short)\n", pdata->led_breath_mode);
}

static ssize_t led_set_breath_mode_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	unsigned long value;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;
	
	pr_debug("%s, buf=%s, count=%d, state=%ld\n", __func__, buf, count, value);
	mutex_lock(&pdata->led_lock);
	pdata->led_breath_mode = value;
	mutex_unlock(&pdata->led_lock);

	return count;
}

static DEVICE_ATTR(led_breath_mode, 0644, led_set_breath_mode_show, led_set_breath_mode_store);

static ssize_t led_set_breath_t0_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_breath_ctrl *pctrl = NULL;

	pctrl = sn3191_led_get_breath_param(led_cdev);
	pr_debug("%s, pctrl.t0=0x%02x\n", __func__, pctrl->t0);
	return sprintf(buf, "t0:0x%02x\n", pctrl->t0);
}

static ssize_t led_set_breath_t0_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	struct sn3191_breath_ctrl *pctrl = NULL;
	int value;
	ssize_t ret = -EINVAL;

	ret = sscanf(buf, "%x", &value);
	if (ret <= 0)
		return ret;
	
	pr_debug("%s, buf=%s, value=0x%x\n", __func__, buf, value);
	mutex_lock(&pdata->led_lock);
	pctrl = sn3191_led_get_breath_param(led_cdev);
	pctrl->t0 = value;
	mutex_unlock(&pdata->led_lock);
	sn3191_led_set_breath_param(led_cdev, pctrl, true);

	return count;
}

static DEVICE_ATTR(led_breath_t0, 0644, led_set_breath_t0_show, led_set_breath_t0_store);

static ssize_t led_set_breath_t1_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_breath_ctrl *pctrl = NULL;

	pctrl = sn3191_led_get_breath_param(led_cdev);
	pr_debug("%s, pctrl.t1=0x%02x\n", __func__, pctrl->t1);
	return sprintf(buf, "t1:0x%02x\n", pctrl->t1);
}

static ssize_t led_set_breath_t1_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	struct sn3191_breath_ctrl *pctrl = NULL;
	int value;
	ssize_t ret = -EINVAL;

	ret = sscanf(buf, "%x", &value);
	if (ret <= 0)
		return ret;
	
	pr_debug("%s, buf=%s, value=0x%x\n", __func__, buf, value);
	mutex_lock(&pdata->led_lock);
	pctrl = sn3191_led_get_breath_param(led_cdev);
	pctrl->t1 = value;
	mutex_unlock(&pdata->led_lock);
	sn3191_led_set_breath_param(led_cdev, pctrl, true);

	return count;
}

static DEVICE_ATTR(led_breath_t1, 0644, led_set_breath_t1_show, led_set_breath_t1_store);

static ssize_t led_set_breath_t2_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_breath_ctrl *pctrl = NULL;

	pctrl = sn3191_led_get_breath_param(led_cdev);
	pr_debug("%s, pctrl.t2=0x%02x\n", __func__, pctrl->t2);
	return sprintf(buf, "t2:0x%02x\n", pctrl->t2);
}

static ssize_t led_set_breath_t2_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	struct sn3191_breath_ctrl *pctrl = NULL;
	unsigned long value;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 8, &value);
	if (ret)
		return ret;
	
	pr_debug("%s, buf=%s, value=%ld\n", __func__, buf, value);
	mutex_lock(&pdata->led_lock);
	pctrl = sn3191_led_get_breath_param(led_cdev);
	pctrl->t2 = value;
	mutex_unlock(&pdata->led_lock);
	sn3191_led_set_breath_param(led_cdev, pctrl, true);

	return count;
}

static DEVICE_ATTR(led_breath_t2, 0644, led_set_breath_t2_show, led_set_breath_t2_store);

static ssize_t led_set_breath_t3_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_breath_ctrl *pctrl = NULL;

	pctrl = sn3191_led_get_breath_param(led_cdev);
	pr_debug("%s, pctrl.t3=0x%02x\n", __func__, pctrl->t3);
	return sprintf(buf, "t3:0x%02x\n", pctrl->t3);
}

static ssize_t led_set_breath_t3_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	struct sn3191_breath_ctrl *pctrl = NULL;
	int value;
	ssize_t ret = -EINVAL;

	ret = sscanf(buf, "%x", &value);
	if (ret <= 0)
		return ret;
	
	pr_debug("%s, buf=%s, value=0x%x\n", __func__, buf, value);
	mutex_lock(&pdata->led_lock);
	pctrl = sn3191_led_get_breath_param(led_cdev);
	pctrl->t3 = value;
	mutex_unlock(&pdata->led_lock);
	sn3191_led_set_breath_param(led_cdev, pctrl, true);

	return count;
}

static DEVICE_ATTR(led_breath_t3, 0644, led_set_breath_t3_show, led_set_breath_t3_store);

static ssize_t led_set_breath_t4_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_breath_ctrl *pctrl = NULL;

	pctrl = sn3191_led_get_breath_param(led_cdev);
	pr_debug("%s, pctrl.t4=0x%02x\n", __func__, pctrl->t4);
	return sprintf(buf, "t4:0x%02x\n", pctrl->t4);
}

static ssize_t led_set_breath_t4_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	struct sn3191_breath_ctrl *pctrl = NULL;
	int value;
	ssize_t ret = -EINVAL;

	ret = sscanf(buf, "%x", &value);
	if (ret <= 0)
		return ret;
	
	printk("%s, buf=%s, value=0x%x\n", __func__, buf, value);
	mutex_lock(&pdata->led_lock);
	pctrl = sn3191_led_get_breath_param(led_cdev);
	pctrl->t4 = value;
	mutex_unlock(&pdata->led_lock);
	sn3191_led_set_breath_param(led_cdev, pctrl, true);

	return count;
}

static DEVICE_ATTR(led_breath_t4, 0644, led_set_breath_t4_show, led_set_breath_t4_store);

static ssize_t led_set_breath_effect_flash_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sn3191_breath_ctrl *pctrl = NULL;

	pctrl = &led_effect_flash;
	printk("%s, t0=0x%02x, t1=0x%02x, t2=0x%02x, t3=0x%02x, t4=0x%02x\n", __func__, 
		pctrl->t0, pctrl->t1, pctrl->t2, pctrl->t3, pctrl->t4);
	return sprintf(buf, "effect flash: t0=0x%02x, t1=0x%02x, t2=0x%02x, t3=0x%02x, t4=0x%02x\n", 
		pctrl->t0, pctrl->t1, pctrl->t2, pctrl->t3, pctrl->t4);
}

static ssize_t led_set_breath_effect_flash_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	struct sn3191_breath_ctrl *pctrl = NULL;
	unsigned long value;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;
	
	pr_debug("%s, buf=%s, count=%d, state=%ld\n", __func__, buf, count, value);
	mutex_lock(&pdata->led_lock);
	pctrl = &led_effect_flash;
	mutex_unlock(&pdata->led_lock);
	sn3191_led_set_breath_param(led_cdev, pctrl, (bool)value);

	return count;
}

static DEVICE_ATTR(led_flash, 0644, led_set_breath_effect_flash_show, led_set_breath_effect_flash_store);

static ssize_t led_set_breath_effect_flash_by_touch_old_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sn3191_breath_ctrl *pctrl = NULL;

	pctrl = &led_effect_flash;
	pr_debug("%s, t0=0x%x, t1=0x%x, t2=0x%x, t3=0x%x, t4=0x%x\n", __func__, 
		pctrl->t0, pctrl->t1, pctrl->t2, pctrl->t3, pctrl->t4);
	return sprintf(buf, "effect flash: t0=0x%x, t1=0x%x, t2=0x%x, t3=0x%x, t4=0x%x\n", 
		pctrl->t0, pctrl->t1, pctrl->t2, pctrl->t3, pctrl->t4);
}

static ssize_t led_set_breath_effect_flash_by_touch_old_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	unsigned long value;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;
	
	pr_debug("%s, buf=%s, count=%d, state=%ld, led_brightness=%d\n", __func__, 
		buf, count, value, pdata->led_brightness);

	mutex_lock(&pdata->led_lock);
	pdata->brightness_state = Brightness_Upper;
	pdata->led_brightness_base = pdata->led_brightness;
	mutex_unlock(&pdata->led_lock);
	sn3191_led_pwm_ctrl(pdata, true);

	cancel_delayed_work_sync(&pdata->pwm_work);
	schedule_delayed_work(&pdata->pwm_work, pdata->pwm_interval);
	
	return count;
}

static DEVICE_ATTR(led_flash_by_touch_old, 0644, led_set_breath_effect_flash_by_touch_old_show, led_set_breath_effect_flash_by_touch_old_store);

static ssize_t led_set_breath_effect_flash_by_touch_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	unsigned int max_brightness = (pdata->led_brightness<<1) + (pdata->led_brightness>>1);

	pr_debug("%s, led_brightness_base=%d, max brightness=%d\n", __func__, 
		pdata->led_brightness, max_brightness);
	return sprintf(buf, "Base brightness:%d, Max brightness:%d (2.5 times requested by Meizu)\n", 
		pdata->led_brightness, max_brightness);
}

static ssize_t led_set_breath_effect_flash_by_touch_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	unsigned long value;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;
	
	pr_debug("%s, buf=%s, count=%d, state=%ld, led_brightness=%d\n", __func__, 
		buf, count, value, pdata->led_brightness);

	mutex_lock(&pdata->led_lock);
	pdata->brightness_state = Brightness_Upper;
	pdata->led_brightness_base = pdata->led_brightness;
	mutex_unlock(&pdata->led_lock);
	sn3191_led_pwm_ctrl(pdata, true);

	if (!pdata->pwm_thread)
	{
		pdata->pwm_thread = kthread_create(sn3191_led_pwm_set_brightness_thread,
							 pdata, "breath led pwm thread");
	}
	
	if (!IS_ERR(pdata->pwm_thread))
	{
		pr_debug("%s, wake_up_process pwm_thread\n", __func__);
		wake_up_process(pdata->pwm_thread);
	}
	else
		pr_err("%s, wake_up_process pwm_thread error\n", __func__);
	
	return count;
}

static DEVICE_ATTR(led_flash_by_touch, 0644, led_set_breath_effect_flash_by_touch_show, led_set_breath_effect_flash_by_touch_store);


static ssize_t red_led_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	int value;
	ssize_t ret = -EINVAL;

	ret = sscanf(buf, "%d", &value);
	if (ret <= 0)
		return ret;
	
	pr_debug("%s, buf=%s, count=%d, state=%d\n", __func__, buf, count, value);

	if(value == 0)

		sn3191_led_set_brightness(led_cdev, 0);
	else
		if(value == 21){
			//dai tia jia
		}
		else
			sn3191_led_set_brightness(led_cdev, 255);
	

	return count;
}

static DEVICE_ATTR(red_led, 0644, NULL, red_led_store);

static ssize_t red_blink_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sn3191_led_pdata *pdata = container_of(led_cdev, struct sn3191_led_pdata, led_cdev);
	struct sn3191_breath_ctrl *pctrl = NULL;
	unsigned long value;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;

	if(value == 0)
	{
		sn3191_led_set_brightness(led_cdev, 0);
		return count;
	}

	if(value == 21){
		sn3191_led_set_brightness(led_cdev, 0);

		pr_debug("%s, buf=%s, count=%d, state=%ld\n", __func__, buf, count, value);
		mutex_lock(&pdata->led_lock);
		pctrl = &led_effect_flash;
		mutex_unlock(&pdata->led_lock);
		sn3191_led_set_breath_param(led_cdev, pctrl, (bool)value);

		s_timer_flag = 0;
		printk("zhouhehe-red_blink_store 1111\n");
		mod_timer(&s_timer,jiffies + 280);
		while(1)
		{
			if(s_timer_flag)break;
			msleep(10);
		}
		sn3191_led_set_brightness(led_cdev, 0);
		printk("zhouhehe-red_blink_store 222\n");
		// dai tian jia 
		return count;
        }

	pr_debug("%s, buf=%s, count=%d, state=%ld\n", __func__, buf, count, value);
	sn3191_led_set_brightness(led_cdev, 0);
	mutex_lock(&pdata->led_lock);
	if(value == 16)
		pctrl = &led_effect_flash2;
	else
		pctrl = &led_effect_flash;
	mutex_unlock(&pdata->led_lock);
	sn3191_led_set_breath_param(led_cdev, pctrl, (bool)value);
	
	return count;
}

static DEVICE_ATTR(blink, 0644, NULL, red_blink_store);

static struct attribute *breath_led_attrs[] = {
	&dev_attr_led_max_current.attr,
	&dev_attr_led_brightness.attr,
	&dev_attr_led_breath_mode.attr,
	&dev_attr_led_breath_t0.attr,
	&dev_attr_led_breath_t1.attr,
	&dev_attr_led_breath_t2.attr,
	&dev_attr_led_breath_t3.attr,
	&dev_attr_led_breath_t4.attr,
	&dev_attr_led_flash.attr,
	&dev_attr_led_flash_by_touch_old.attr,
	&dev_attr_led_flash_by_touch.attr,
	&dev_attr_red_led.attr,
	&dev_attr_blink.attr,
	NULL
};

static struct attribute_group breath_led_attr_group = {
	.attrs = breath_led_attrs
};

static int sn3191_led_power_on(struct i2c_client *client, bool on)
{
	struct sn3191_led_pdata *pdata = i2c_get_clientdata(client);
	int rc = 0;

	if (!on)
		goto power_off;

	if (gpio_is_valid(pdata->power_ldo_gpio))
	{
		printk("%s, power_ldo_gpio\n", __func__);
		gpio_set_value(pdata->power_ldo_gpio, 1);
	}

	printk("%s, regulator\n", __func__);
	rc = regulator_enable(pdata->vdd);
	if (rc)
	{
		dev_err(&client->dev, "Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	if (pdata->i2c_pull_up)
	{
		rc = regulator_enable(pdata->vcc_i2c);
		if (rc)
		{
			dev_err(&client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
//			regulator_disable(data->vdd);
		}
	}
	return rc;

power_off:
	if (gpio_is_valid(pdata->power_ldo_gpio))
	{
		gpio_set_value(pdata->power_ldo_gpio, 0);
	}

	rc = regulator_disable(pdata->vdd);
	if (rc)
	{
		dev_err(&client->dev, "Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(pdata->vcc_i2c);
	if (rc)
	{
		dev_err(&client->dev, "Regulator vcc_i2c disable failed rc=%d\n", rc);
	}

	return rc;
}

static int sn3191_led_power_init(struct i2c_client *client, bool on)
{
	struct sn3191_led_pdata *pdata = i2c_get_clientdata(client);
	int rc = 0;

	if (!on)
		goto pwr_deinit;
	
	if (gpio_is_valid(pdata->power_ldo_gpio))
	{
		printk("%s, power_ldo_gpio\n", __func__);
		rc = gpio_request(pdata->power_ldo_gpio, "sn3191_led_ldo");
		if (rc)
		{
			printk("irq gpio request failed\n");
			goto Err_gpio_request;
		}
		
		rc = gpio_direction_output(pdata->power_ldo_gpio, 1);
		if (rc)
		{
			printk("set_direction for irq gpio failed\n");
			goto free_ldo_gpio;
		}
	}

	printk("%s, regulator\n", __func__);
	pdata->vdd = regulator_get(&client->dev, "vdd");
	if (IS_ERR(pdata->vdd))
	{
		rc = PTR_ERR(pdata->vdd);
		dev_err(&client->dev, "Regulator get failed vdd rc=%d\n", rc);
		goto Err_regulator_get;
	}

	if (regulator_count_voltages(pdata->vdd) > 0)
	{
		rc = regulator_set_voltage(pdata->vdd, VTG_MIN_UV, VTG_MAX_UV);
		if (rc)
		{
			dev_err(&client->dev, "Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	if (pdata->i2c_pull_up)
	{
		pdata->vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
		if (IS_ERR(pdata->vcc_i2c))
		{
			rc = PTR_ERR(pdata->vcc_i2c);
			dev_err(&client->dev, "Regulator get failed vcc_i2c rc=%d\n", rc);
			goto reg_vdd_set_vtg;
		}

		if (regulator_count_voltages(pdata->vcc_i2c) > 0)
		{
			rc = regulator_set_voltage(pdata->vcc_i2c, I2C_VTG_MIN_UV, I2C_VTG_MAX_UV);
			if (rc)
			{
				dev_err(&client->dev, "Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
				goto reg_vcc_i2c_put;
			}
		}
	}
	return 0;

reg_vcc_i2c_put:
	if (pdata->i2c_pull_up)
		regulator_put(pdata->vcc_i2c);
reg_vdd_set_vtg:
	if (!gpio_is_valid(pdata->power_ldo_gpio))
		if (regulator_count_voltages(pdata->vdd) > 0)
			regulator_set_voltage(pdata->vdd, 0, VTG_MAX_UV);
reg_vdd_put:
free_ldo_gpio:	
	if (gpio_is_valid(pdata->power_ldo_gpio))
		gpio_free(pdata->power_ldo_gpio);
	else
		regulator_put(pdata->vdd);
Err_regulator_get:
Err_gpio_request:
	return rc;
pwr_deinit:
	if (gpio_is_valid(pdata->power_ldo_gpio))
		gpio_free(pdata->power_ldo_gpio);
	else
	{
		if (regulator_count_voltages(pdata->vdd) > 0)
			regulator_set_voltage(pdata->vdd, 0, VTG_MAX_UV);
		regulator_put(pdata->vdd);
	}

	if (pdata->i2c_pull_up)
	{
		if (regulator_count_voltages(pdata->vcc_i2c) > 0)
			regulator_set_voltage(pdata->vcc_i2c, 0, I2C_VTG_MAX_UV);
		regulator_put(pdata->vcc_i2c);
	}
	return 0;
}

static int sn3191_led_pinctrl_init(struct i2c_client *client)
{
	int retval;
	struct sn3191_led_pdata *data = i2c_get_clientdata(client);
	
	/* Get pinctrl if target uses pinctrl */
	data->led_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->led_pinctrl))
	{
		dev_dbg(&client->dev, "Target does not use pinctrl\n");
		retval = PTR_ERR(data->led_pinctrl);
		data->led_pinctrl = NULL;
		return retval;
	}
    
	data->gpio_state_active	= pinctrl_lookup_state(data->led_pinctrl, "breath_led_active");
	if (IS_ERR_OR_NULL(data->gpio_state_active))
	{
		printk("%s Can not get ts default pinstate\n", __func__);
		retval = PTR_ERR(data->gpio_state_active);
		data->led_pinctrl = NULL;
		return retval;
	}

	data->gpio_state_suspend = pinctrl_lookup_state(data->led_pinctrl, "breath_led_suspend");
	if (IS_ERR_OR_NULL(data->gpio_state_suspend))
	{
		dev_err(&client->dev,	"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(data->gpio_state_suspend);
		data->led_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int sn3191_led_pinctrl_select(struct i2c_client *client, bool on)
{
	struct sn3191_led_pdata *data = i2c_get_clientdata(client);
	struct pinctrl_state *pins_state;
	int ret;
	
	pins_state = on ? data->gpio_state_active : data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state))
	{
		ret = pinctrl_select_state(data->led_pinctrl, pins_state);
		if (ret)
		{
			dev_err(&client->dev, "can not set %s pins\n",
				on ? "breath_led_active" : "breath_led_suspend");
			return ret;
		}
	}
	else
	{
		dev_err(&client->dev,	"not a valid '%s' pinstate\n",
			on ? "breath_led_active" : "breath_led_suspend");
	}

	return 0;
}

int sn3191_led_set_gpio(struct sn3191_led_pdata *data)
{
	struct sn3191_led_pdata *pdata = data;
	int ret = 0;

	printk("%s, sdb_gpio=%d\n", __func__, pdata->sdb_gpio);
	if (gpio_is_valid(pdata->sdb_gpio))
	{
		ret = gpio_request_one(pdata->sdb_gpio, GPIOF_DIR_OUT | GPIOF_INIT_LOW, "sn3191_led_sdb");
		if (ret != 0) {
			printk("Failed to request sn3191_led_sdb: %d\n", ret);
			return -1;
		}
		printk("%s, sdb_gpio=%d\n", __func__, pdata->sdb_gpio);
		
		gpio_set_value(pdata->sdb_gpio, 1);
	}

	return 0;
}

static int sn3191_led_parse_dt(struct device *dev, struct sn3191_led_pdata *data)
{
	struct device_node *np = dev->of_node;
	struct sn3191_led_pdata *pdata = data;
	int ret = 0;

	printk("%s\n", __func__);
	/* gpio info */
	pdata->sdb_gpio = of_get_named_gpio_flags(np, "si-en,sdb-gpio", 0, &pdata->sdb_gpio_flags);
	if (pdata->sdb_gpio < 0)
		printk("%s, sdb_gpio not exist!\n", __func__);
	else
		printk("%s, sdb_gpio=%d\n", __func__, pdata->sdb_gpio);

	pdata->power_ldo_gpio = of_get_named_gpio_flags(np, "si-en,power_ldo-gpio", 0, &pdata->power_ldo_gpio_flags);
	if (pdata->power_ldo_gpio < 0)
		printk("%s, power_ldo_gpio not exist!\n", __func__);
	else
		printk("%s, power_ldo_gpio=%d\n", __func__, pdata->power_ldo_gpio);

	pdata->i2c_pull_up = of_property_read_bool(np, "si-en,i2c-pull-up");
	ret = of_property_read_string(np, "si-en,led-name", &pdata->led_cdev.name);
	if (ret)
		printk("%s: missing %s in dt node\n", __func__, "si-en,led-name");
	else
		printk("%s, led-name=%s\n", __func__, pdata->led_cdev.name);

	printk("%s done\n", __func__);
	return 0;
}


static int sn3191_led_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct sn3191_led_pdata *pdata;
	struct sn3191_breath_ctrl *pctrl = NULL;
	int ret = 0;

	printk("%s\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

    if (client->dev.of_node)
	{
		pdata = devm_kzalloc(&client->dev, sizeof(struct sn3191_led_pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocated sn3191_led_pdata\n");
			return -ENOMEM;
		}
		
		ret = sn3191_led_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "DT parsing failed\n");
			goto Err_parse_dt;
		}
    }

	pdata->client = client;
	i2c_set_clientdata(client, pdata);
	dev_set_drvdata(&client->dev, pdata);

	mutex_init(&pdata->led_lock);

	sn3191_led_set_gpio(pdata);
	msleep(10);
	
	ret = sn3191_led_pinctrl_init(client);
	if (!ret && pdata->led_pinctrl)
	{
		ret = sn3191_led_pinctrl_select(client, true);
		if (ret < 0)
			goto Err_pinctrl_init;
	}
	
	ret = sn3191_led_power_init(client, true);
	if (ret) {
		dev_err(&client->dev, "power init failed");
		goto Err_power_init;
	}

	ret = sn3191_led_power_on(client, true);
	if (ret) {
		dev_err(&client->dev, "power on failed");
		goto Err_power_on;
	}
	
	ret = sn3191_led_off(pdata);
	if (ret)
		goto Err_led_off;

	/* Power up IC */
	pdata->led_current = SN_CURRENT_5mA;
	pdata->led_brightness = 0xff;		// PWM 100%
	pdata->led_breath_mode = SN_MODE_PWM;
	pdata->ctrl.t0 = SN_T0_0s;
	pdata->ctrl.t1 = SN_T1_1_04s;
	pdata->ctrl.t2 = SN_T2_0s;
	pdata->ctrl.t3 = SN_T3_1_04s;
	pdata->ctrl.t4 = SN_T4_0s;

#if 0
	sn3191_led_pwm_ctrl(pdata, true);
#endif

	INIT_DELAYED_WORK(&pdata->pwm_work, sn3191_led_pwm_set_brightness_work);
	pdata->pwm_interval = 2;
	pdata->brightness_state = Brightness_Upper;

//	pdata->led_cdev.name = "sn3191-breath-led";		// indicated in dtsi.
	pdata->led_cdev.brightness = LED_OFF;
	pdata->led_cdev.max_brightness = LED_FULL;
	pdata->led_cdev.brightness_set = sn3191_led_set_brightness_bool; 
//	pdata->led_cdev.brightness_set = sn3191_led_set_brightness; 
	pdata->led_cdev.brightness_get = sn3191_led_get_brightness;

	ret = led_classdev_register(&pdata->client->dev, &pdata->led_cdev);
	if (ret < 0) {
		dev_err(&pdata->client->dev, "couldn't register LED %s\n",
							pdata->led_cdev.name);
		goto Err_led_classdev;
	}
	
	
	/* Register sysfs hooks */                                                  
	ret = sysfs_create_group(&pdata->led_cdev.dev->kobj, &breath_led_attr_group);
	if (ret)
		goto Err_sysfs_create;

//----------zhouhehe add----------------	//
	init_timer(&s_timer);
	s_timer.function=second_timer_handle;
	s_timer_flag = 0;
	s_timer.expires=jiffies + 250;
	add_timer(&s_timer);

	mutex_lock(&pdata->led_lock);
	pctrl = &led_effect_flash;
	mutex_unlock(&pdata->led_lock);
	sn3191_led_set_breath_param(&pdata->led_cdev, pctrl, 1);
		
	while(1)
	{
		if(s_timer_flag)break;
		msleep(10);
	}
	sn3191_led_set_brightness(&pdata->led_cdev, 0);
//----------zhouhehe add----------------	//

	printk("%s done\n", __func__);
	return 0;

Err_sysfs_create:
	led_classdev_unregister(&pdata->led_cdev);
Err_led_classdev:
	sn3191_led_power_on(client, false);
Err_led_off:
Err_power_on:
	sn3191_led_power_init(client, false);
Err_power_init:
	if (pdata->led_pinctrl)
		if (sn3191_led_pinctrl_select(client, false) < 0)
			pr_err("Cannot get idle pinctrl state\n");
Err_pinctrl_init:
	if (pdata->led_pinctrl)
		pinctrl_put(pdata->led_pinctrl);
	i2c_set_clientdata(client, NULL);
Err_parse_dt:
	devm_kfree(&client->dev, pdata);
	printk("%s err, ret=%d\n", __func__, ret);
	return ret;
}

static int sn3191_led_remove(struct i2c_client *client)
{
	struct sn3191_led_pdata *pdata = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&pdata->pwm_work);
	led_classdev_unregister(&pdata->led_cdev);
	sn3191_led_power_on(client, false);
	sn3191_led_power_init(client, false);
	
	if (pdata->led_pinctrl)
		if (sn3191_led_pinctrl_select(client, false) < 0)
			pr_err("Cannot get idle pinctrl state\n");
		
	if (pdata->led_pinctrl)
		pinctrl_put(pdata->led_pinctrl);

	mutex_destroy(&pdata->led_lock);
	i2c_set_clientdata(client, NULL);
	devm_kfree(&client->dev, pdata);

	return 0;
}

static const struct i2c_device_id sn3191_led_id[] = {
	{"sn3191_led", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, sn3191_led_id);

#ifdef CONFIG_OF
static struct of_device_id sn3191_led_match_table[] = {
	{ .compatible = "si-en,sn3191",},
	{ },
};
#else
#define mms_match_table NULL
#endif

static struct i2c_driver sn3191_led_driver = {
	.probe		= sn3191_led_probe,
	.remove		= sn3191_led_remove,
	.driver		= {
		.name	= "sn3191_led",
		.owner = THIS_MODULE,
		.of_match_table = sn3191_led_match_table,
	},
	.id_table	= sn3191_led_id,
};

static int __init sn3191_led_init(void)
{
	printk("%s\n", __func__);
	return i2c_add_driver(&sn3191_led_driver);
}

static void __exit sn3191_led_exit(void)
{
	return i2c_del_driver(&sn3191_led_driver);
}

module_init(sn3191_led_init);
module_exit(sn3191_led_exit);

MODULE_AUTHOR("Jacky Xu <xuke@longcheer.net>");
MODULE_VERSION("2014.11.11");
MODULE_DESCRIPTION("SI-EN Breath LED Driver, designed for Meizu phone.");
MODULE_LICENSE("GPL");

