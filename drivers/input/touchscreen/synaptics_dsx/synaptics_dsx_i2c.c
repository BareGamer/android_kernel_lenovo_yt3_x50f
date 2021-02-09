/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/input/synaptics_dsx.h>
#include "synaptics_dsx_core.h"

#define SYN_I2C_RETRY_TIMES 10

/*
#define I2C_BURST_LIMIT 255
*/
/*
#define XFER_MSGS_LIMIT 8
*/

static struct synaptics_dsx_hw_interface hw_if;

static struct platform_device *synaptics_dsx_i2c_device;

#ifdef CONFIG_OF
static int parse_dt(struct device *dev, struct synaptics_dsx_board_data *bdata)
{
	int retval;
	u32 value;
	const char *name;
	//struct property *prop;
	struct device_node *np = dev->of_node;

	bdata->irq_gpio = of_get_named_gpio_flags(np,
			"synaptics,irq-gpio", 0, NULL);

	printk("%s: done parsing synaptics,irq-gpio\n", __func__);
	msleep(20);

	retval = of_property_read_u32(np, "synaptics,irq-on-state",
			&value);
	if (retval < 0)
		bdata->irq_on_state = 0;
	else
		bdata->irq_on_state = value;

	printk("%s: done parsing synaptics,irq-on-state\n", __func__);
	msleep(20);

	retval = of_property_read_u32(np, "synaptics,irq-flags", &value);
	if (retval < 0)
		return retval;
	else
		bdata->irq_flags = value;

	printk("%s: done parsing synaptics,irq-flags\n", __func__);
	msleep(20);

	retval = of_property_read_string(np, "synaptics,pwr-reg-name", &name);
	if (retval == -EINVAL)
		bdata->pwr_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->pwr_reg_name = name;

	printk("%s: done parsing synaptics,pwr-reg-name\n", __func__);
	msleep(20);

	retval = of_property_read_string(np, "synaptics,bus-reg-name", &name);
	if (retval == -EINVAL)
		bdata->bus_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->bus_reg_name = name;

	printk("%s: done parsing synaptics,bus-reg-name\n", __func__);
	msleep(20);

#if 0
	if (of_property_read_bool(np, "synaptics,power-gpio")) {
		bdata->power_gpio = of_get_named_gpio_flags(np,
				"synaptics,power-gpio", 0, NULL);
		retval = of_property_read_u32(np, "synaptics,power-on-state",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->power_on_state = value;
	} else {
		bdata->power_gpio = -1;
	}

	printk("%s: done parsing synaptics,power-gpio\n", __func__);
#else	

	bdata->power_gpio = of_get_named_gpio_flags(np,
			"synaptics,power-gpio", 0, NULL);
	retval = of_property_read_u32(np, "synaptics,power-on-state",
			&value);
	if (retval < 0) {
		printk("Failed to parse synaptics,power-on-state\n");
		return retval;
	}
	else
		bdata->power_on_state = value;

	printk("%s: done parsing synaptics,power-gpio\n", __func__);
	msleep(20);	
#endif


#if 0	
	if (of_property_read_bool(np, "synaptics,power-delay-ms")) {
		retval = of_property_read_u32(np, "synaptics,power-delay-ms",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->power_delay_ms = value;
	} else {
		bdata->power_delay_ms = 0;
	}
#else
	retval = of_property_read_u32(np, "synaptics,power-delay-ms",
				&value);
	if (retval < 0){
		printk("%s: Failed to parse synaptics,power-delay-ms\n", __func__);
		bdata->power_delay_ms = 250;	
		return retval;
	}else
		bdata->power_delay_ms = value;

#endif
	printk("%s: done parsing synaptics,power-delay-ms, delay time = %d\n", __func__, bdata->power_delay_ms);
	msleep(20);

	retval = of_property_read_u32(np, "synaptics,reset-delay-ms",
			&value);
	if (retval < 0){
		printk("Failed to parse reset delay \n");
		bdata->reset_delay_ms = 250; 
		return retval;
	}else{
		bdata->reset_delay_ms = value;
	}

	printk("%s: done parsing synaptics,reset-delay-ms = %d\n", __func__, bdata->reset_delay_ms);
	msleep(20);

#if 0
	if (of_property_read_bool(np, "synaptics,reset-gpio")) {
		
		printk("check point 1\n");
		
		bdata->reset_gpio = of_get_named_gpio_flags(np,
				"synaptics,reset-gpio", 0, NULL);
		retval = of_property_read_u32(np, "synaptics,reset-on-state",
				&value);
				
		printk("check point 2\n");
		
		if (retval < 0)
			return retval;
		else
			bdata->reset_on_state = value;
		retval = of_property_read_u32(np, "synaptics,reset-active-ms",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->reset_active_ms = value;
	} else {
		bdata->reset_gpio = -1;
	}

	printk("%s: done parsing synaptics,reset-gpio\n", __func__);
	msleep(20);	
#else
	bdata->reset_gpio = of_get_named_gpio_flags(np,
			"synaptics,reset-gpio", 0, NULL);
	retval = of_property_read_u32(np, "synaptics,reset-on-state",
			&value);
				
	if (retval < 0)
		return retval;
	else
		bdata->reset_on_state = value;
	retval = of_property_read_u32(np, "synaptics,reset-active-ms",
			&value);
	if (retval < 0)
		return retval;
	else
		bdata->reset_active_ms = value;
		
	printk("%s: done parsing synaptics,reset-gpio =%d, reset-on-state = %d, reset_active_ms = %d\n"
		, __func__, bdata->reset_gpio, bdata->reset_on_state, bdata->reset_active_ms);
	msleep(20);	
		
#endif
	
	

	return 0;
}
#endif

static void synaptics_rmi4_i2c_check_addr(struct synaptics_rmi4_data *rmi4_data,
		struct i2c_client *i2c)
{
	if (hw_if.board_data->ub_i2c_addr == -1)
		return;

	if (hw_if.board_data->i2c_addr == i2c->addr)
		hw_if.board_data->i2c_addr = hw_if.board_data->ub_i2c_addr;
	else
		hw_if.board_data->i2c_addr = i2c->addr;

	return;
}

static int synaptics_rmi4_i2c_set_page(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr)
{
	int retval;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[1];

	msg[0].addr = hw_if.board_data->i2c_addr;
	msg[0].flags = 0;
	msg[0].len = PAGE_SELECT_LEN;
	msg[0].buf = buf;

	page = ((addr >> 8) & MASK_8BIT);
	buf[0] = MASK_8BIT;
	buf[1] = page;

	if (page != rmi4_data->current_page) {
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			if (i2c_transfer(i2c->adapter, msg, 1) == 1) {
				rmi4_data->current_page = page;
				retval = PAGE_SELECT_LEN;
				break;
			}
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: I2C retry %d\n",
					__func__, retry + 1);
			msleep(20);

			if (retry == SYN_I2C_RETRY_TIMES / 2) {
				synaptics_rmi4_i2c_check_addr(rmi4_data, i2c);
				msg[0].addr = hw_if.board_data->i2c_addr;
			}
		}
	} else {
		retval = PAGE_SELECT_LEN;
	}

	return retval;
}

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
#ifdef I2C_BURST_LIMIT
	unsigned char ii;
	unsigned char rd_msgs = ((length - 1) / I2C_BURST_LIMIT) + 1;
#else
	unsigned char rd_msgs = 1;
#endif
	unsigned char index = 0;
	unsigned char xfer_msgs;
	unsigned char remaining_msgs;
	unsigned short i2c_addr;
	unsigned short data_offset = 0;
	unsigned short remaining_length = length;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_adapter *adap = i2c->adapter;
	struct i2c_msg msg[rd_msgs + 1];

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	msg[0].addr = hw_if.board_data->i2c_addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf;

#ifdef I2C_BURST_LIMIT
	for (ii = 0; ii < (rd_msgs - 1); ii++) {
		msg[ii + 1].addr = hw_if.board_data->i2c_addr;
		msg[ii + 1].flags = I2C_M_RD;
		msg[ii + 1].len = I2C_BURST_LIMIT;
		msg[ii + 1].buf = &data[data_offset];
		data_offset += I2C_BURST_LIMIT;
		remaining_length -= I2C_BURST_LIMIT;
	}
#endif

	msg[rd_msgs].addr = hw_if.board_data->i2c_addr;
	msg[rd_msgs].flags = I2C_M_RD;
	msg[rd_msgs].len = remaining_length;
	msg[rd_msgs].buf = &data[data_offset];

	buf = addr & MASK_8BIT;

	remaining_msgs = rd_msgs + 1;

	while (remaining_msgs) {
#ifdef XFER_MSGS_LIMIT
		if (remaining_msgs > XFER_MSGS_LIMIT)
			xfer_msgs = XFER_MSGS_LIMIT;
		else
			xfer_msgs = remaining_msgs;
#else
		xfer_msgs = remaining_msgs;
#endif
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			retval = i2c_transfer(adap, &msg[index], xfer_msgs);
			if (retval == xfer_msgs)
				break;

			dev_err(rmi4_data->pdev->dev.parent,
					"%s: I2C retry %d\n",
					__func__, retry + 1);
			msleep(20);

			if (retry == SYN_I2C_RETRY_TIMES / 2) {
				synaptics_rmi4_i2c_check_addr(rmi4_data, i2c);
				i2c_addr = hw_if.board_data->i2c_addr;
				msg[0].addr = i2c_addr;
#ifdef I2C_BURST_LIMIT
				for (ii = 0; ii < (rd_msgs - 1); ii++)
					msg[ii + 1].addr = i2c_addr;
#endif
				msg[rd_msgs].addr = i2c_addr;
			}
		}

		if (retry == SYN_I2C_RETRY_TIMES) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: I2C read over retry limit\n",
					__func__);
			retval = -EIO;
			goto exit;
		}

		remaining_msgs -= xfer_msgs;
		index += xfer_msgs;
	}

	retval = length;

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return retval;
}

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[1];

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	msg[0].addr = hw_if.board_data->i2c_addr;
	msg[0].flags = 0;
	msg[0].len = length + 1;
	msg[0].buf = buf;

	buf[0] = addr & MASK_8BIT;
	retval = secure_memcpy(&buf[1], length, &data[0], length, length);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to copy data\n",
				__func__);
		goto exit;
	}

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(i2c->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);

		if (retry == SYN_I2C_RETRY_TIMES / 2) {
			synaptics_rmi4_i2c_check_addr(rmi4_data, i2c);
			msg[0].addr = hw_if.board_data->i2c_addr;
		}
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C write over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return retval;
}

static struct synaptics_dsx_bus_access bus_access = {
	.type = BUS_I2C,
	.read = synaptics_rmi4_i2c_read,
	.write = synaptics_rmi4_i2c_write,
};

static void synaptics_rmi4_i2c_dev_release(struct device *dev)
{
	kfree(synaptics_dsx_i2c_device);

	return;
}

static int synaptics_rmi4_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval;

	printk("%s\n", __func__);
	msleep(20);

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data commands not supported by host\n",
				__func__);
		return -EIO;
	}

	printk("%s: checked i2c functionality\n", __func__);
	msleep(20);

	synaptics_dsx_i2c_device = kzalloc(
			sizeof(struct platform_device),
			GFP_KERNEL);
	if (!synaptics_dsx_i2c_device) {
		dev_err(&client->dev,
				"%s: Failed to allocate memory for synaptics_dsx_i2c_device\n",
				__func__);
		return -ENOMEM;
	}

	printk("%s: allocated memory for synaptics_dsx_i2c_device\n", __func__);
	msleep(20);

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		printk("%s: identified of node\n", __func__);
		msleep(20);

		hw_if.board_data = devm_kzalloc(&client->dev,
				sizeof(struct synaptics_dsx_board_data),
				GFP_KERNEL);
		if (!hw_if.board_data) {
			dev_err(&client->dev,
					"%s: Failed to allocate memory for board data\n",
					__func__);
			return -ENOMEM;
		}

		printk("%s: allocated memory for board data\n", __func__);
		msleep(20);

		hw_if.board_data->cap_button_map = devm_kzalloc(&client->dev,
				sizeof(struct synaptics_dsx_button_map),
				GFP_KERNEL);
		if (!hw_if.board_data->cap_button_map) {
			dev_err(&client->dev,
					"%s: Failed to allocate memory for 0D button map\n",
					__func__);
			return -ENOMEM;
		}

		printk("%s: allocated memory for 0D button map\n", __func__);
		msleep(20);

		hw_if.board_data->vir_button_map = devm_kzalloc(&client->dev,
				sizeof(struct synaptics_dsx_button_map),
				GFP_KERNEL);
		if (!hw_if.board_data->vir_button_map) {
			dev_err(&client->dev,
					"%s: Failed to allocate memory for virtual button map\n",
					__func__);
			return -ENOMEM;
		}

		printk("%s: allocated memory for virtual button map\n", __func__);
		msleep(20);

		parse_dt(&client->dev, hw_if.board_data);

		printk("%s: done parsing dt\n", __func__);
		msleep(20);
	}
#else
	hw_if.board_data = client->dev.platform_data;
#endif

	hw_if.bus_access = &bus_access;
	hw_if.board_data->i2c_addr = client->addr;

	synaptics_dsx_i2c_device->name = PLATFORM_DRIVER_NAME;
	synaptics_dsx_i2c_device->id = 0;
	synaptics_dsx_i2c_device->num_resources = 0;
	synaptics_dsx_i2c_device->dev.parent = &client->dev;
	synaptics_dsx_i2c_device->dev.platform_data = &hw_if;
	synaptics_dsx_i2c_device->dev.release = synaptics_rmi4_i2c_dev_release;

	printk("%s: registering platform device\n", __func__);
	msleep(20);

	retval = platform_device_register(synaptics_dsx_i2c_device);
	if (retval) {
		dev_err(&client->dev,
				"%s: Failed to register platform device\n",
				__func__);
		return -ENODEV;
	}

	printk("%s: done registering platform device\n", __func__);
	msleep(20);

	return 0;
}

static int synaptics_rmi4_i2c_remove(struct i2c_client *client)
{
	platform_device_unregister(synaptics_dsx_i2c_device);

	return 0;
}

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{I2C_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

#ifdef CONFIG_OF
static struct of_device_id synaptics_rmi4_of_match_table[] = {
	{
		.compatible = "synaptics,dsx-i2c",
	},
	{},
};
MODULE_DEVICE_TABLE(of, synaptics_rmi4_of_match_table);
#else
#define synaptics_rmi4_of_match_table NULL
#endif

static struct i2c_driver synaptics_rmi4_i2c_driver = {
	.driver = {
		.name = I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = synaptics_rmi4_of_match_table,
	},
	.probe = synaptics_rmi4_i2c_probe,
	.remove = synaptics_rmi4_i2c_remove,
	.id_table = synaptics_rmi4_id_table,
};

int synaptics_rmi4_bus_init(void)
{
	return i2c_add_driver(&synaptics_rmi4_i2c_driver);
}
EXPORT_SYMBOL(synaptics_rmi4_bus_init);

void synaptics_rmi4_bus_exit(void)
{
	i2c_del_driver(&synaptics_rmi4_i2c_driver);

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_bus_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX I2C Bus Support Module");
MODULE_LICENSE("GPL v2");
