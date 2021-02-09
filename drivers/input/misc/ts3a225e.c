/* drivers/hwmon/mt6516/amit/IQS128.c - IQS128/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/ts3a225e.h>

/******************************************************************************
 * configuration
*******************************************************************************/
#define TS3A225E_DEV_NAME     "TS3A225E"

static struct i2c_client *ts3a225e_i2c_client = NULL;
static const struct i2c_device_id ts3a225e_i2c_id[] = {{"TS3A225E",0},{}};

static int ts3a225e_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	uint8_t devicve_id[1];
	
	printk("ts3a225e_i2c_probe \n");

	ts3a225e_i2c_client = client;

	ts3a225e_read_byte(0x01, &devicve_id[0]);
	printk("ts3a225e_i2c_probe ID=%x \n", devicve_id[0]);
    ts3a225e_write_byte(0x02, 0x07);
    ts3a225e_read_byte(0x02, &devicve_id[0]);
	printk("ts3a225e_i2c_probe 02=%x \n", devicve_id[0]);
    ts3a225e_write_byte(0x03, 0x01);
    ts3a225e_read_byte(0x03, &devicve_id[0]);
	printk("ts3a225e_i2c_probe 03=%x \n", devicve_id[0]);
    ts3a225e_read_byte(0x04, &devicve_id[0]);
	printk("ts3a225e_i2c_probe 04=%x \n", devicve_id[0]);
    ts3a225e_read_byte(0x05, &devicve_id[0]);
	printk("ts3a225e_i2c_probe 05=%x \n", devicve_id[0]);
    ts3a225e_read_byte(0x06, &devicve_id[0]);
	printk("ts3a225e_i2c_probe 06=%x \n", devicve_id[0]);
    ts3a225e_read_byte(0x07, &devicve_id[0]);
	printk("ts3a225e_i2c_probe 07=%x \n", devicve_id[0]);

	return 0;
}

static DEFINE_MUTEX(ts3a225e_i2c_access);

int ts3a225e_read_byte(uint8_t reg_addr, uint8_t *data)
{
    struct i2c_client *client = ts3a225e_i2c_client;
    int retry;
    struct i2c_msg msg[] = {
			{
			 .addr = (client != NULL? client->addr : 0) ,
			 .flags = 0,
			 .len = 1,
			 .buf = &reg_addr,
			},

			{
			 .addr = (client != NULL? client->addr : 0),
			 .flags = I2C_M_RD,
			 .len = 1,
			 .buf = data,
			 },
		};
    if (client == NULL) {
        printk("ts3a225e client is null, maybe init fail  \n");
        return -EIO;
    }
    mutex_lock(&ts3a225e_i2c_access);
    for (retry = 0; retry < 5; retry++) {
    	if (i2c_transfer(client->adapter, msg,
    				ARRAY_SIZE(msg)) > 0)
    		break;
    	else
    		mdelay(1);
    }

    if (5 <= retry) {
    	dev_err(&client->dev, "I2C xfer error");
                 mutex_unlock(&ts3a225e_i2c_access);    
    	return -EIO;
    }
    mutex_unlock(&ts3a225e_i2c_access);    
    return 0;
}

int ts3a225e_write_byte(uint8_t reg_addr, uint8_t data)
{
    u8 buffer[2];
    int retry;
    struct i2c_client *client = ts3a225e_i2c_client;
    struct i2c_msg msg[] = {
    	{
    	 .addr = (client != NULL? client->addr : 0),
    	 .flags = 0,
    	 .len = 2,
    	 .buf = buffer,
    	 },
    };

    if (client == NULL) {
        printk("ts3a225e client is null, maybe init fail  \n");
        return -EIO;
    }
    mutex_lock(&ts3a225e_i2c_access);
    buffer[0] = reg_addr;
    buffer[1] = data;
    for (retry = 0; retry < 5; retry++) {
    	if (i2c_transfer(client->adapter, msg,
    				ARRAY_SIZE(msg)) > 0) {
    		break;
    	} else {
    		mdelay(1);
    	}
    }
    if (5 <= retry) {
        dev_err(&client->dev, "I2C xfer error");
        mutex_unlock(&ts3a225e_i2c_access);
    	 return -EIO;
    }
    mdelay(2);
    mutex_unlock(&ts3a225e_i2c_access);
    return 0;
}

/******************************************************************************
 * extern functions
*******************************************************************************/
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id audio_switch_of_match[] = {
	{ .compatible = "hs_switch,ts3a225e", },
	{},
};
#endif
static struct i2c_driver ts3a225e_i2c_driver =
{
    .probe      = ts3a225e_i2c_probe,
    .id_table   = ts3a225e_i2c_id,
    .driver = {
        .name = TS3A225E_DEV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = audio_switch_of_match,
    },
};

/*----------------------------------------------------------------------------*/
static int __init ts3a225e_init(void)
{
	printk("ts3a225e_init \n");
	return i2c_add_driver(&ts3a225e_i2c_driver);
}
/*----------------------------------------------------------------------------*/
static void __exit ts3a225e_exit(void)
{
	printk("ts3a225e_exit \n");
       i2c_del_driver(&ts3a225e_i2c_driver);
}
/*----------------------------------------------------------------------------*/
module_init(ts3a225e_init);
module_exit(ts3a225e_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("TS3A225E driver");
MODULE_LICENSE("GPL");

