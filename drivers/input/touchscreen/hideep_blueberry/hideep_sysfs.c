/*******************************************************************************
 * Copyright (C) 2014 HiDeep, Inc.
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
 *******************************************************************************/
#include "hideep.h"
#include <linux/input/hideep_blueberry_ts.h> // slot
/*******************************************************************************
 * sysfs subsystem 
 *
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/

static ssize_t hideep_rawdata_show(struct device *dev, struct device_attribute *attr, char *buf)
 {
	int index;
	int index1;
	short row_size = 0;
	int range_size = 0;
	int error = 0;
	int i;
	struct rawdata_info *info = NULL;
	char *pbuf;

	info = (struct rawdata_info *)kzalloc(sizeof(struct rawdata_info), GFP_KERNEL);
	if (!info) {
		dbg_err("malloc failed");
		error = -ENOMEM;
		goto out;
	}

	info->used_size = 0;
	error = hideep_get_rawdata(info);
	if (error) {
		dbg_err("can't get rawdata %d", error);
		error = -EBUSY;
		goto out;
	}
	pbuf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	row_size = info->buff[0];
	range_size = info->buff[1];
	i=0;
	snprintf(pbuf+i,PAGE_SIZE-i,"rx: %d, tx : %d\n", row_size, range_size);
	

	for (index=0; row_size*index+2 < info->used_size; index++) {
		if (0 == index) {
			i = strlen(pbuf);
			snprintf(pbuf+i,PAGE_SIZE-i,"rawdata:\n");
		}
		for (index1=0; index1 < row_size; index1++) {
			i = strlen(pbuf);
			snprintf(pbuf+i,PAGE_SIZE-i,"%d,", info->buff[2+row_size*index+index1]);
		}
		//index1 = 0;
		i = strlen(pbuf);
		snprintf(pbuf+i,PAGE_SIZE-i,"\n");

		if ((range_size -1) == index) {
			i = strlen(pbuf);
			snprintf(pbuf+i,PAGE_SIZE-i,"diff:\n");
		}
	}
	error = 0;
	scnprintf(buf, PAGE_SIZE, "%s", pbuf);

out:
	if (info)
		kfree(info);
	if(pbuf)
		kfree(pbuf);
	dbg_fun("done");
	return error;
}
#ifdef HIDEEP_SELF_TEST
static ssize_t hideep_self_test_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    int i;
    int count;

    dbg_fun();
    
	mutex_lock(&private_ts->io_lock); // Mutex lock - front of function
    hideep_test_mode(private_ts);
	mutex_unlock(&private_ts->io_lock); // Mutex lock - front of function
    ret = 0;
    //TX short
    for(count = 0, i = 0; i < TX_NUM; i++){
    	if(private_ts->TXshortResult[i]){
    		if(count ==0)
    		  dbg_err("TX[%d]\n",i);
    		else
    		  dbg_err(",TX[%d]\n",i);
    		count++;
            dbg_fun("tx short");
    		ret = -1;
    	}
    }
    if(count)
    	dbg_err( "TX(s) are short!\n");
    //TX open
    for(count = 0, i = 0; i < TX_NUM; i++){
    	if(private_ts->TXopenResult[i]){
    		if(count ==0)
    		  dbg_err( "TX[%d]\n",i);
    		else
    		  dbg_err( ",TX[%d]\n",i);
    		count++;
            dbg_fun("tx open");
    		ret = -1;
    	}
    }
    if(count)
    	dbg_err( "TX(s) are open!\n");
    	
   
    //RX short
    for(count = 0, i = 0; i < TX_NUM; i++){
    	if(private_ts->RXshortResult[i]){
    		if(count ==0)
    		  dbg_err( "RX[%d]\n",i);
    		else
    		  dbg_err( ",RX[%d]\n",i);
    		count++;
            dbg_fun("rx short");
    		ret = -1;
    	}
    }
    if(count)
    	dbg_err( "RX(s) are short!\n");
    //RX open
    for(count = 0, i = 0; i < TX_NUM; i++){
    	if(private_ts->RXopenResult[i]){
    		if(count ==0)
    		  dbg_err( "RX[%d]\n",i);
    		else
    		  dbg_err( ",RX[%d]\n",i);
    		count++;
            dbg_fun("rs open");
    		ret = -1;
    	}
    }
    if(count)
    	dbg_err( "RX(s) are open!\n");
    if(ret == 0)
        return scnprintf(buf, PAGE_SIZE, "PASS\n");
    return scnprintf(buf, PAGE_SIZE, "FAIL\n");
}
static DEVICE_ATTR(selftest, S_IRWXUGO, hideep_self_test_read, NULL);

#endif
static ssize_t hideep_update_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;

    dbg_fun();    
    ret = hideep_update(dev, HIDEEP_MANUAL_FW);
    if (ret<0)
    {
        dev_err(dev, "The firmware update failed(%d)\n", ret);
        count = ret;
    }
    return count;
}
static DEVICE_ATTR(update, S_IRWXUGO, NULL, hideep_update_write);

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct hideep_data *ts_drv = dev_get_drvdata(dev);
    unsigned char vs[10];

    if(check_version(ts_drv->client,vs)>=0){
        if((vs[8] & 0xf0) ==0)
            return scnprintf(buf, PAGE_SIZE, "[Vendor] O-Film, [FW] v%d.%02d, [IC] IST940E\n", 
                         vs[__HIDEEP_MAJOR_VERSION__+6], vs[__HIDEEP_MINOR_VERSION__+6]);
        else
            return scnprintf(buf, PAGE_SIZE, "[Vendor] GIS, [FW] v%d.%02d, [IC] IST940E\n", 
                         vs[__HIDEEP_MAJOR_VERSION__+6], vs[__HIDEEP_MINOR_VERSION__+6]);
    }
    else
        return scnprintf(buf, PAGE_SIZE, "can't read out version no.\n");
}
static DEVICE_ATTR(version, S_IRWXUGO, version_show, NULL);


static ssize_t driver_ver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "driver version = v%d.%02d.%02d\n",DD_VERSION_MAJOR,DD_VERSION_MIDDLE,DD_VERSION_MINOR);
}
static DEVICE_ATTR(driver_ver, S_IRWXUGO, driver_ver_show, NULL);


static ssize_t loglevel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "loglevel = %d",g_loglevel);
}
static ssize_t loglevel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) 
{
    ssize_t ret = -EINVAL;  
    char *after;   
	size_t count;
	
	
	printk("%s\n",__FUNCTION__);

	after = NULL;
	
	g_loglevel = simple_strtoll(buf,NULL,10);
	count = (size_t)(after-buf);
    if (count == size) {  
        ret = count;  
    	}
    return ret;
}

static DEVICE_ATTR(loglevel, S_IRWXUGO, loglevel_show, loglevel_store);
static DEVICE_ATTR(test, S_IRWXUGO, hideep_rawdata_show, NULL);

/*******************************************************************************
 * attribute group
 *******************************************************************************/
static struct attribute *hideep_ts_sysfs_entries[] =
{
    &dev_attr_test.attr,
    &dev_attr_update.attr,
    &dev_attr_version.attr,
    &dev_attr_driver_ver.attr,
    &dev_attr_loglevel.attr,
#ifdef HIDEEP_SELF_TEST
    &dev_attr_selftest.attr,
#endif
    NULL
};
static struct attribute_group hideep_ts_attr_group =
{
    .attrs  = hideep_ts_sysfs_entries,
};
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
int
hideep_sysfs_init(struct hideep_data *ts)
{
    int ret;
    struct  i2c_client *client = ts->client;
    /* Create the files associated with this kobject */
    ret = sysfs_create_group(&client->dev.kobj, &hideep_ts_attr_group);
    printk("device : %s \n", client->dev.kobj.name);
    ret = sysfs_create_link(NULL,&client->dev.kobj, "hideep");
    if(ret){
        dbg_err("fail to create link ret =0x%x",ret);
    }
    return ret;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
int
hideep_sysfs_exit(struct hideep_data *ts)
{
    struct  i2c_client *client = ts->client;
    sysfs_remove_group(&client->dev.kobj, &hideep_ts_attr_group);
    return 0;
}
