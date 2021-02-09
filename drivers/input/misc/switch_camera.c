#include <linux/init.h>	
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif


#undef CDBG

#define CDBG(fmt, args...) pr_err(fmt, ##args)
#define KEY_CAM_SWITCH_OPEN 0x256
#define KEY_CAM_SWITCH_CLOSE 0x257

int irq_function_irq=0;
extern void write_front_register(void);
extern void write_back_register(void);

extern void write_register_panorma(void);
extern void write_register_no_panorma(void);
int get_gpio_state(void);

extern int start_flag;
extern int poweer_down_camera_state;
static struct timer_list s_timer;
//?¡§šº¡À?¡Â?š¢11š¬?

struct switch_camera_irq
{
	int gpio;
	int irq;
	int state;
	char *irq_name;
};

struct switch_camera_info
{
	//	struct mutex io_lock;
	struct switch_camera_irq out_pin;
	struct switch_camera_irq mvout_pin;
	struct pinctrl *pinctrl;
	struct pinctrl_state *default_state;
	struct pinctrl_state *sleep_state;
	struct regulator	*vdd;
	struct input_dev *ipdev;

};
static struct switch_camera_irq *irq_data=NULL;
static struct switch_camera_info *switch_camera_info=NULL;
static int switch_camera_gpio_ctl1=0;
static int switch_camera_gpio_ctl2=0;
static int switch_camera_gpio_flag=0;
static int switch_camera_timer_flag=0;
static int first_flag=0;
struct work_struct switch_camera_wq_back;
struct work_struct switch_camera_wq_front;

struct work_struct switch_camera_panorma;

static int temp=0;
//static int temp_flag=0;

static char strstrstr[7]="front";                                                                                   
static char strstrstr_1[6]="back";                                                                                  
int sysfs_flag=1;   
static struct kobject *kobj;   
static ssize_t camera_attr_show(struct device *dev,struct device_attribute *attr, char *buf);  
static ssize_t camera_attr_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count);  
static struct device_attribute camera_attr={  
	.attr = {    
		.name ="camera",  
		.mode = S_IRUGO | S_IWUGO | S_IXUGO         
	},                                     
	.show	= camera_attr_show,		
	.store	= camera_attr_store,   
};    
static ssize_t camera_attr_show(struct device *dev,struct device_attribute *attr, char *buf) 
{                                       
	//	int rc=0;   
	sysfs_flag=get_gpio_state();

	if(sysfs_flag==1) 
		snprintf(buf, 7, "%s\n",strstrstr);    
	if(sysfs_flag==0)   
		snprintf(buf, 6, "%s\n",strstrstr_1);   
	return strlen(buf)+1;        
}       
static ssize_t camera_attr_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)   
{
	unsigned int input;
	printk("camera_attr_store: start.....................\n");
	if (sscanf(buf, "%u", &input) != 1) { 
		return -EINVAL;
	}

	printk("camera_attr_store:  end.....................\n");   
	return 1;
}


int is_panorma=0;                                                                                  
static char panorma_strs[8]="panorma";                                                                                  
static struct kobject *kobj_1;   
static ssize_t panorma_attr_show(struct device *dev,struct device_attribute *attr, char *buf);  

static struct device_attribute panorma_attr={  
	.attr = {    
		.name ="panorma",  
		.mode = S_IRUGO | S_IWUGO | S_IXUGO         
	},                                     
	.show	= panorma_attr_show,		
	.store	= NULL,   
};  

//write to userspace
static ssize_t panorma_attr_show(struct device *dev,struct device_attribute *attr, char *buf) 
{                                       
	snprintf(buf, 8, "%s\n",panorma_strs); 
	is_panorma=1;  
	printk("wusheng panorma\n");
	return strlen(buf)+1;        
}       

static char no_panorma_strs[11]="no_panorma";                                                                                  
static struct kobject *kobj_2;   
static ssize_t no_panorma_attr_show(struct device *dev,struct device_attribute *attr, char *buf);  
static struct device_attribute no_panorma_attr={  
	.attr = {    
		.name ="no_panorma",  
		.mode = S_IRUGO | S_IWUGO | S_IXUGO         
	},                                     
	.show	= no_panorma_attr_show,		
	.store	= NULL,   
};  

//write to userspace
static ssize_t no_panorma_attr_show(struct device *dev,struct device_attribute *attr, char *buf) 
{                                       
	snprintf(buf, 11, "%s\n",no_panorma_strs); 
	is_panorma=0;   
	printk("wusheng no panorma\n");
	return strlen(buf)+1;        
}       



void switch_camera_do_work_back(struct work_struct *work_1)
{
	if(start_flag==1)
		write_back_register();
}
void switch_camera_do_work_front(struct work_struct *work_1)
{
	if(start_flag==1)
		write_front_register();
}

void switch_camera_panorma_do_work(struct work_struct *work_1)
{
	if(start_flag==1)
		write_register_panorma();

}
static void second_timer_handle(unsigned long arg)
{

	if(first_flag == 0)
		CDBG(" [first]-----second_timer_handle\n");
	else
		CDBG(" -----second_timer_handle\n");


	//**************************first_flag: start ***************************************
	if(first_flag == 0)
	{
		first_flag = 1;
		switch_camera_gpio_ctl1 = gpio_get_value(switch_camera_info->out_pin.gpio);
		CDBG(" [first]:gpio_28 = %d\n",  switch_camera_gpio_ctl1);

		switch_camera_gpio_ctl2 = gpio_get_value(switch_camera_info->mvout_pin.gpio);
		CDBG(" [first]:gpio_97 = %d\n",   switch_camera_gpio_ctl2);


		if(switch_camera_gpio_ctl1 ==1 && switch_camera_gpio_ctl2==1)
		{
			if(switch_camera_timer_flag)return;
			switch_camera_timer_flag=1;

			switch_camera_gpio_flag=1;
			CDBG(" [first]-----input front camera\n");

		}
		else if(switch_camera_gpio_ctl1 ==0 ||switch_camera_gpio_ctl2==0)
		{
			if(!switch_camera_timer_flag)return;
			switch_camera_timer_flag=0;

			//	switch_camera_gpio_flag=0;//Áœžögpio¶ŒÀ­µÍ
			CDBG(" [first]-----input back camera\n");
		}

	}
	//**************************first_flag:end ***************************************

	if(is_panorma==0)
	{

		if(switch_camera_gpio_ctl1 ==switch_camera_gpio_flag ||switch_camera_gpio_ctl2==switch_camera_gpio_flag)
		{
			CDBG(" -----***********************************:%d\n",switch_camera_gpio_flag);



			if(switch_camera_gpio_flag!=poweer_down_camera_state&&poweer_down_camera_state!=-1) {
				poweer_down_camera_state=-1;
				temp = switch_camera_gpio_flag;
			}
			else
			{
				if(switch_camera_gpio_flag==0)
				{	
					CDBG(" -----back camera module\n");
					sysfs_flag=0;



					if(temp==1)
					{
						input_report_key(switch_camera_info->ipdev, KEY_CAM_SWITCH_OPEN, 1);      
						input_report_key(switch_camera_info->ipdev, KEY_CAM_SWITCH_OPEN, 0);
						input_sync(switch_camera_info->ipdev);
						CDBG(" -----input_report_key back camera event\n");

						schedule_work(&switch_camera_wq_back);				

					}
					temp=0;

				}
				else
				{
					CDBG(" -----front camera module\n");

					sysfs_flag=1;

					if(temp==0)
					{


						input_report_key(switch_camera_info->ipdev, KEY_CAM_SWITCH_CLOSE, 1);
						input_report_key(switch_camera_info->ipdev, KEY_CAM_SWITCH_CLOSE, 0); 
						input_sync(switch_camera_info->ipdev);
						CDBG(" -----input_report_key front camera event\n");

						schedule_work(&switch_camera_wq_front);
					}
					temp=1;
				}
			}
		}
	}
	else
	{
		if(switch_camera_gpio_ctl1 ==switch_camera_gpio_flag ||switch_camera_gpio_ctl2==switch_camera_gpio_flag)
		{
			CDBG(" -----***********************************:%d\n",switch_camera_gpio_flag);
			if(switch_camera_gpio_flag==0)
			{	
				sysfs_flag=0;		//back
				temp=0;
			}
			else
			{
				sysfs_flag=1;
				temp=1;
			}

		} 
		schedule_work(&switch_camera_panorma);
	}
	if(switch_camera_info->out_pin.irq)
		enable_irq(switch_camera_info->out_pin.irq);	
	if(switch_camera_info->mvout_pin.irq)
		enable_irq(switch_camera_info->mvout_pin.irq);	
}



static irqreturn_t switch_camera_interrupt(int irq, void *data)
{
	int switch_camera_interrupt_flag=0;
	irq_data = data;

	if(switch_camera_info->out_pin.irq)
		disable_irq_nosync(switch_camera_info->out_pin.irq);	
	if(switch_camera_info->mvout_pin.irq)
		disable_irq_nosync(switch_camera_info->mvout_pin.irq);	
	//disable_irq_nosync(irq);
	irq_function_irq=1;
	CDBG("\n\n :switch_camera_interrupt  start...\n");

	switch_camera_gpio_ctl1 = gpio_get_value(switch_camera_info->out_pin.gpio);
	CDBG(" :gpio_28 = %d\n",  switch_camera_gpio_ctl1);

	switch_camera_gpio_ctl2 = gpio_get_value(switch_camera_info->mvout_pin.gpio);
	CDBG(" :gpio_97 = %d\n",   switch_camera_gpio_ctl2);

	if(switch_camera_gpio_ctl1 ==1 && switch_camera_gpio_ctl2==1)
	{
		switch_camera_interrupt_flag=1;
	}
	else if(switch_camera_gpio_ctl1 ==0 ||switch_camera_gpio_ctl2==0)
	{
		switch_camera_interrupt_flag=0;
	}

	CDBG(" -----****************************open timer\n");
	switch_camera_gpio_flag=switch_camera_interrupt_flag;
	mod_timer(&s_timer,jiffies + 20);  //mod_timer(struct timer_list *timer, unsigned long expires)

	//enable_irq(irq);
	return IRQ_HANDLED;
}

static int switch_camera_init_irq(struct switch_camera_irq *switch_camera)
{
	int rc = -1;
	struct switch_camera_irq *data = switch_camera;


	if (gpio_is_valid(data->gpio)) {
		rc = gpio_request(data->gpio, switch_camera->irq_name);
		if (rc < 0) {
			pr_err("switch_camera_probe: gpio_request fail rc=%d\n", rc);
			return rc ;
		}

		rc = gpio_direction_input(data->gpio);
		if (rc < 0) {
			pr_err("switch_camera_probe: gpio_direction_input fail rc=%d\n", rc);
			return rc ;
		}
		data->state = gpio_get_value(data->gpio);
		pr_info("%s:[%s] state = %d\n", __func__, switch_camera->irq_name, data->state);

		data->irq = gpio_to_irq(data->gpio);

		//       irq_set_irq_wake(data->irq,1);
		rc = request_threaded_irq(data->irq, switch_camera_interrupt, NULL,
				IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
				switch_camera->irq_name, data);
		if (rc < 0) {
			pr_err("switch_camera: request_irq fail rc=%d\n", rc);
			return rc ;
		}

	}else{
		pr_err("switch_camera irq gpio not provided\n");
		return rc ;
	}
	return 0;
}

static int switch_camera_parse_dt(struct device *dev, struct switch_camera_info *pdata)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags = OF_GPIO_ACTIVE_LOW;
	int rc;

	pdata->out_pin.gpio = of_get_named_gpio_flags(np, "switch_camera,irq-out",
			0, &flags);
	if (pdata->out_pin.gpio < 0) {
		pr_err("switch_camera get out_pin gpio fail\n");
		return -EINVAL;
	} else {
		pdata->out_pin.irq_name = "switch_camera-out";
	}

	pdata->mvout_pin.gpio = of_get_named_gpio_flags(np, "switch_camera,irq-mvout",
			0, &flags);
	if (pdata->mvout_pin.gpio < 0) {
		pr_err("switch_camera get mvout_pin gpio fail\n");
	} else {
		pdata->mvout_pin.irq_name = "switch_camera-mvout";
	}

	pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pdata->pinctrl)) {
		pr_err("%s: Unable to get pinctrl handle\n", __func__);
		return -EINVAL;
	}
	pdata->default_state = pinctrl_lookup_state(pdata->pinctrl, "default");
	if (IS_ERR(pdata->default_state)) {
		pr_err("%s: Unable to get pinctrl active handle\n", __func__);
		return -EINVAL;
	}
	pdata->sleep_state = pinctrl_lookup_state(pdata->pinctrl, "sleep");
	if (IS_ERR(pdata->sleep_state)) {
		pr_err("%s: Unable to get pinctrl active handle\n", __func__);
		return -EINVAL;
	}

	rc = pinctrl_select_state(pdata->pinctrl , pdata->default_state);
	if (rc) {
		pr_err("%s: pinctrl_select_state fail\n", __func__);
	}

	pdata->vdd = regulator_get(dev, "vdd");
	if (IS_ERR(pdata->vdd)) {
		rc = PTR_ERR(pdata->vdd);
		pr_err("%s: Regulator get failed vdd rc=%d\n", __func__, rc);
		return rc;
	}

	pr_info("switch_camera_parse_dt success \n");
	return 0;
}
static int switch_camera_power_supply_on(struct switch_camera_info *pdata)
{
	int rc=0;

	if (regulator_count_voltages(pdata->vdd) > 0) {
		rc = regulator_set_voltage(pdata->vdd,
				1800000, 1800000);
		if (rc) {
			pr_err("%s: Regulator set failed vdd rc=%d\n",__func__, rc);
		}
	}

	rc = regulator_enable(pdata->vdd);
	if (rc) {
		pr_err("%s: Regulator vdd enable failed rc=%d\n", __func__, rc);
	}

	pr_info("%s success \n",__func__);
	return rc;
}

int switch_camera_suspend(void)
{	

	disable_irq(switch_camera_info->out_pin.irq);	
	disable_irq(switch_camera_info->mvout_pin.irq);	
	//	pr_info("%s: %d camera_state=%d\n", __func__,__LINE__,pdata->camera_state);	
	return 0;
}
int switch_camera_resume(void)
{
	enable_irq(switch_camera_info->out_pin.irq);	
	enable_irq(switch_camera_info->mvout_pin.irq);	
	//	pr_info("%s: %d camera_state=%d\n", __func__,__LINE__,pdata->camera_state);
	return 0;
}

int get_gpio_state(void)
{
	int i=0;
	switch_camera_gpio_ctl1 = gpio_get_value(switch_camera_info->out_pin.gpio);
	//       CDBG(" get_gpio_state:gpio_28 = %d\n",  switch_camera_gpio_ctl1);

	switch_camera_gpio_ctl2 = gpio_get_value(switch_camera_info->mvout_pin.gpio);
	//      CDBG(" get_gpio_state:gpio_97 = %d\n",   switch_camera_gpio_ctl2);

	if(switch_camera_gpio_ctl1 ==1 && switch_camera_gpio_ctl2==1)
	{
		i=1;

	}
	else if(switch_camera_gpio_ctl1 ==0 ||switch_camera_gpio_ctl2==0)
	{
		i=0;

	}
	return i;
}

static int switch_camera_probe(struct platform_device *pdev)
{
	int rc = 0;
	int err;
	int ret=0;
	pr_info("%s: enter \n", __func__);

	if (pdev->dev.of_node) {
		switch_camera_info = kzalloc(sizeof(struct switch_camera_info), GFP_KERNEL);		  
		if (!switch_camera_info) {
			pr_err("%s: failed to alloc memory for module data\n",__func__);
			return -ENOMEM;
		}
		err = switch_camera_parse_dt(&pdev->dev, switch_camera_info);
		if (err) {
			dev_err(&pdev->dev, "switch_camera_probe DT parsing failed\n");
			goto free_struct;
		}
	} else{
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, switch_camera_info);
	/*input system config*/		
	switch_camera_info->ipdev = input_allocate_device();		
	if (!switch_camera_info->ipdev) {			
		pr_err("switch_camera_probe: input_allocate_device fail\n");			
		goto input_error;		
	}		
	switch_camera_info->ipdev->name = "camera-switch-input";		
	input_set_capability(switch_camera_info->ipdev, EV_KEY, KEY_CAM_SWITCH_OPEN);		
	input_set_capability(switch_camera_info->ipdev, EV_KEY, KEY_CAM_SWITCH_CLOSE);		
	set_bit(INPUT_PROP_NO_DUMMY_RELEASE, switch_camera_info->ipdev->propbit);		
	rc = input_register_device(switch_camera_info->ipdev);		
	if (rc) {			
		pr_err("switch_camera_probe: input_register_device fail rc=%d\n", rc);			
		goto input_error;		
	}

	rc = switch_camera_power_supply_on(switch_camera_info);
	if (rc) {
		pr_err("switch_camera_probe: switch_camera_power_supply_on fail rc=%d\n", rc);
		goto input_error;
	}
	msleep(1);



	/*interrupt config*/
	rc = switch_camera_init_irq(&switch_camera_info->out_pin);
	if (switch_camera_info->mvout_pin.gpio > 0)
		rc |= switch_camera_init_irq(&switch_camera_info->mvout_pin);

	if (rc) {
		pr_err("switch_camera_probe: switch_camera_init_irq fail rc=%d\n", rc);
		goto err_irq;
	}



	INIT_WORK(&switch_camera_wq_back,switch_camera_do_work_back);
	INIT_WORK(&switch_camera_wq_front,switch_camera_do_work_front);
	INIT_WORK(&switch_camera_panorma,switch_camera_panorma_do_work);
	//  switch_camera_init_device_name();

	init_timer(&s_timer);
	s_timer.function=second_timer_handle;
	s_timer.expires=jiffies + 20;//??¡ì¡§o?š€???|š¬??¡§2¡§o?š€??
	add_timer(&s_timer);
	//???¡§???¡ì¡§o?š€????šº?????¡ì¡§o?š€????¡§?¡§¡§???¡ì¡§o?š€???¡§¡é???š€¡§a?D?šº?2?šŠ??šš????¡ì¡§o?š€???
	kobj=kobject_create_and_add("cameradir",NULL);
	if(1){	
		ret = sysfs_create_file(kobj,&camera_attr.attr);	
		if (ret < 0) {		
			printk("%s: Failed to create sysfs attributes\n",__func__);	
		}
	}
	kobj_1=kobject_create_and_add("panormadir",NULL);
	if(1){	
		ret = sysfs_create_file(kobj_1,&panorma_attr.attr);	

		if (ret < 0) {		
			printk("%s: Failed to create panorma sysfs attributes\n",__func__);	
		}
	}
	kobj_2=kobject_create_and_add("no_panormadir",NULL);

	if(1){	
		ret = sysfs_create_file(kobj_2,&no_panorma_attr.attr);	

		if (ret < 0) {		
			printk("%s: Failed to create no panorma sysfs attributes\n",__func__);	
		}
	}


	pr_err("switch_camera_probe end\n");
	return 0;

err_irq:
	gpio_free(switch_camera_info->out_pin.gpio);
	gpio_free(switch_camera_info->mvout_pin.gpio);
	platform_set_drvdata(pdev, NULL);
input_error:
	platform_set_drvdata(pdev, NULL);
free_struct:
	kfree(switch_camera_info);

	return rc;
}

static int switch_camera_remove(struct platform_device *pdev)
{
	struct switch_camera_info *data = platform_get_drvdata(pdev);
	pr_err("switch_camera_remove\n");
	free_irq(data->out_pin.irq, data);
	gpio_free(data->out_pin.gpio);

	free_irq(data->mvout_pin.irq, data);
	gpio_free(data->mvout_pin.gpio);
	del_timer(&s_timer);
	return 0;
}


static struct of_device_id of_match_table[] = {
	{ .compatible = "cameradet,switch_camera", },
	{ },
}; 

static struct platform_driver switch_camera_driver = {
	.probe                = switch_camera_probe,
	.remove                = switch_camera_remove,
	.driver                = {
		.name        = "switch_camera",
		.owner        = THIS_MODULE,
		.of_match_table = of_match_table,
	},
};
static int __init switch_camera_init(void)
{
	return platform_driver_register(&switch_camera_driver);
}

static void __exit switch_camera_exit(void)
{
	platform_driver_unregister(&switch_camera_driver);
}
module_init(switch_camera_init);
module_exit(switch_camera_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(" wusheng@longcheer.net");
