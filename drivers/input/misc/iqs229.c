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
#include <linux/fb.h>

#define KEY_CAP_FAR                0x252
#define KEY_CAP_NEAR               0x253

struct iqs229_irq
{
	int gpio;
	int irq;
	int state;
	char *irq_name;
};

enum iqs229_suspend_stat{
	IQS_RESUME = 0,
	IQS_SUSPEND,
};

struct iqs229_info
{
	struct work_struct irq_work;
	struct iqs229_irq irq_out;
	int movout;
	int calling_stat;
	enum iqs229_suspend_stat suspend_stat;
	struct pinctrl *pinctrl;
	struct pinctrl_state *default_state;
	struct pinctrl_state *sleep_state;
	struct regulator	*vio;
	struct regulator	*vdd;
	struct input_dev *ipdev;
	struct kobject *iqs_kobj;
	#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	#endif
};

static struct iqs229_info *iqs_pdata;
static int iqs_enable_flag;
static ssize_t iqs229_attr_enable_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 2, "%d\n", iqs_enable_flag);
}
static ssize_t iqs229_attr_enable_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	int rc;

	if (sscanf(buf, "%u", &input) != 1) {
		return -EINVAL;
	}

	if(iqs_pdata==NULL) return 0;
	if(input){
		//capsensor reset when wake up
		if (gpio_is_valid(iqs_pdata->movout)) {
			rc = gpio_direction_input(iqs_pdata->movout);
			if (rc < 0) {
				pr_err("%s: gpio_direction_output fail rc=%d\n",__func__, rc);
			}
		}
		//enable baseband irq
	        irq_set_irq_wake(iqs_pdata->irq_out.irq,1);
	        enable_irq(iqs_pdata->irq_out.irq);
		iqs_enable_flag = 1;
	}else{
		iqs_enable_flag = 0;
		//capsensor report move far
		input_report_key(iqs_pdata->ipdev, KEY_CAP_FAR, 1);
		input_report_key(iqs_pdata->ipdev, KEY_CAP_FAR, 0);
		input_sync(iqs_pdata->ipdev);
		//disable baseband irq
		irq_set_irq_wake(iqs_pdata->irq_out.irq,0);
		disable_irq(iqs_pdata->irq_out.irq);
		//capsensor sleep mode
		if (gpio_is_valid(iqs_pdata->movout)) {
			rc = gpio_direction_output(iqs_pdata->movout, 0);
			if (rc < 0) {
				pr_err("%s: gpio_direction_output fail rc=%d\n",__func__, rc);
			}
			gpio_set_value(iqs_pdata->movout, 0);
			msleep(100);//>80ms
		}else{
			rc = -EINVAL;
		}
	}

	return count;
}
static DEVICE_ATTR(enable, 0660, iqs229_attr_enable_show, iqs229_attr_enable_store);

static int iqs229_reseed(struct iqs229_info *pdata)
{
	int rc = 0;
	if (gpio_is_valid(pdata->movout)) {
		rc = gpio_direction_output(pdata->movout, 0);
		if (rc < 0) {
			pr_err("%s: gpio_direction_output fail rc=%d\n",__func__, rc);
			return rc ;
		}
		gpio_set_value(pdata->movout, 0);
		msleep(30);//>20ms && <40ms
		pr_info("%s: for capsensor\n",__func__);
		rc = gpio_direction_input(pdata->movout);
		if (rc < 0) {
			pr_err("%s: gpio_direction_input fail rc=%d\n",__func__, rc);
			return rc ;
		}
	}else{
		rc = -EINVAL;
	}
	return rc;
}
static ssize_t iqs229_attr_reseed_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 2, "%d\n", iqs_enable_flag);//do not care
}
static ssize_t iqs229_attr_reseed_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1) {
		return -EINVAL;
	}

	if(iqs_pdata==NULL) return 0;
	if(input){
		iqs_enable_flag = 0;
	        disable_irq(iqs_pdata->irq_out.irq);
	        irq_set_irq_wake(iqs_pdata->irq_out.irq,0);

		iqs229_reseed(iqs_pdata);

	        irq_set_irq_wake(iqs_pdata->irq_out.irq,1);
	        enable_irq(iqs_pdata->irq_out.irq);
		iqs_enable_flag = 1;
	}

	return count;
}
static DEVICE_ATTR(reseed, 0660, iqs229_attr_reseed_show, iqs229_attr_reseed_store);

static ssize_t iqs229_attr_calling_stat_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1) {
		return -EINVAL;
	}

	if(iqs_pdata==NULL) return 0;
	iqs_pdata->calling_stat = input ? 1 : 0;

	/*reset capsensor for each calling*/
	if(iqs_pdata->calling_stat){
		iqs_enable_flag = 0;
	        disable_irq(iqs_pdata->irq_out.irq);
	        irq_set_irq_wake(iqs_pdata->irq_out.irq,0);

		iqs229_reseed(iqs_pdata);

	        irq_set_irq_wake(iqs_pdata->irq_out.irq,1);
	        enable_irq(iqs_pdata->irq_out.irq);
		iqs_enable_flag = 1;
	}

	return count;
}
static DEVICE_ATTR(callingstat, 0644, NULL, iqs229_attr_calling_stat_store);

static struct attribute *iqs229_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_reseed.attr,
	&dev_attr_callingstat.attr,
	NULL
};
static struct attribute_group iqs229_attribute_group = {
	.attrs = iqs229_attributes
};

#if defined(CONFIG_FB)
static int iqs229_suspend(struct iqs229_info *pdata)
{
	int rc;
	if(iqs_pdata==NULL) return 0;
	iqs_pdata->suspend_stat = IQS_SUSPEND;
	if(iqs_enable_flag && !iqs_pdata->calling_stat){//not in calling
		iqs_enable_flag = 0;
		//capsensor report move far
		input_report_key(iqs_pdata->ipdev, KEY_CAP_FAR, 1);
		input_report_key(iqs_pdata->ipdev, KEY_CAP_FAR, 0);
		input_sync(iqs_pdata->ipdev);
		//disable baseband irq
		irq_set_irq_wake(iqs_pdata->irq_out.irq,0);
		disable_irq(iqs_pdata->irq_out.irq);
		//capsensor sleep mode
		if (gpio_is_valid(iqs_pdata->movout)) {
			rc = gpio_direction_output(iqs_pdata->movout, 0);
			if (rc < 0) {
				pr_err("%s: gpio_direction_output fail rc=%d\n",__func__, rc);
			}
			gpio_set_value(iqs_pdata->movout, 0);
			msleep(100);//>80ms
		}else{
			rc = -EINVAL;
		}
	}
	pr_info("%s: %d calling_stat=%d\n", __func__,__LINE__,pdata->calling_stat);
	return 0;
}

static int iqs229_resume(struct iqs229_info *pdata)
{
	int rc;
	if(iqs_pdata==NULL) return 0;
	iqs_pdata->suspend_stat = IQS_RESUME;
	if(!iqs_enable_flag){
		//capsensor reset when wake up
		if (gpio_is_valid(iqs_pdata->movout)) {
			rc = gpio_direction_input(iqs_pdata->movout);
			if (rc < 0) {
				pr_err("%s: gpio_direction_output fail rc=%d\n",__func__, rc);
			}
		}
		//enable baseband irq
	        irq_set_irq_wake(iqs_pdata->irq_out.irq,1);
	        enable_irq(iqs_pdata->irq_out.irq);
		iqs_enable_flag = 1;
	}
	pr_info("%s: %d calling_stat=%d\n", __func__,__LINE__,pdata->calling_stat);
	return 0;
}

static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct iqs229_info *pdata=NULL;
	struct fb_event *evdata = data;
	int *blank;

	pdata = container_of(self, struct iqs229_info, fb_notif);
	if (evdata && evdata->data && event == FB_EVENT_BLANK && pdata) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			iqs229_resume(pdata);
		else if (*blank == FB_BLANK_POWERDOWN)
			iqs229_suspend(pdata);
	}
	return 0;
}
#endif

static irqreturn_t iqs229_interrupt(int irq, void *data)
{
	struct iqs229_irq *irq_data = data;
	int iqs229_gpio_value;
	struct iqs229_info *pdata;

	disable_irq_nosync(irq);
	pdata = container_of(irq_data, struct iqs229_info, irq_out);
	if (gpio_is_valid(irq_data->gpio))
		iqs229_gpio_value = gpio_get_value(irq_data->gpio);
	if(iqs229_gpio_value != irq_data->state){
		irq_data->state = iqs229_gpio_value;
		schedule_work(&pdata->irq_work);
	}
	//enable_irq(irq);
	return IRQ_HANDLED;
}

static int iqs229_init_irq(struct iqs229_irq *iqs229)
{
	int rc = -1;
	struct iqs229_irq *data = iqs229;

	if (gpio_is_valid(data->gpio)) {
		rc = gpio_request(data->gpio, iqs229->irq_name);
		if (rc < 0) {
			pr_err("%s: gpio_request fail rc=%d\n",__func__, rc);
			return rc ;
		}

		rc = gpio_direction_input(data->gpio);
		if (rc < 0) {
			pr_err("%s: gpio_direction_input fail rc=%d\n",__func__, rc);
			return rc ;
		}
		data->state = gpio_get_value(data->gpio);
		pr_info("%s:[%s] state = %d\n", __func__, iqs229->irq_name, data->state);

		data->irq = gpio_to_irq(data->gpio);

		irq_set_irq_wake(data->irq,1);
		rc = request_threaded_irq(data->irq, iqs229_interrupt, NULL,
						IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
						iqs229->irq_name, data);
		if (rc < 0) {
			pr_err("%s request_irq fail rc=%d\n", __func__,rc);
			return rc ;
		}
	}else{
		pr_err("%s irq gpio %d not provided\n",__func__, data->gpio);
		return rc ;
	}
	return 0;
}

static void iqs229_irq_work(struct work_struct *work)
{
	struct iqs229_info *pdata=NULL;
	struct iqs229_irq *irq_data;
	unsigned int key_code=0;
	int rc;

	pdata = container_of((struct work_struct *)work, struct iqs229_info, irq_work);
	if(pdata == NULL) return;
	irq_data = &pdata->irq_out;

	key_code = irq_data->state ? KEY_CAP_FAR : KEY_CAP_NEAR;
	input_report_key(pdata->ipdev, key_code, 1);
	input_report_key(pdata->ipdev, key_code, 0);
	input_sync(pdata->ipdev);
	enable_irq(pdata->irq_out.irq);
	pr_info("%s irq_data->state=%d\n",__func__,irq_data->state);

	if(iqs_enable_flag && pdata->suspend_stat==IQS_SUSPEND && !pdata->calling_stat){//not in calling
		iqs_enable_flag = 0;
		//capsensor report move far
		input_report_key(pdata->ipdev, KEY_CAP_FAR, 1);
		input_report_key(pdata->ipdev, KEY_CAP_FAR, 0);
		input_sync(pdata->ipdev);
		//disable baseband irq
		irq_set_irq_wake(pdata->irq_out.irq,0);
		disable_irq(pdata->irq_out.irq);
		//capsensor sleep mode
		if (gpio_is_valid(pdata->movout)) {
			rc = gpio_direction_output(pdata->movout, 0);
			if (rc < 0) {
				pr_err("%s: gpio_direction_output fail rc=%d\n",__func__, rc);
			}
			gpio_set_value(pdata->movout, 0);
			msleep(100);//>80ms
		}else{
			rc = -EINVAL;
		}
	}
}

static int iqs229_parse_dt(struct device *dev, struct iqs229_info *pdata)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags = OF_GPIO_ACTIVE_LOW;
	int rc;

	pdata->irq_out.gpio = of_get_named_gpio_flags(np, "iqs229,irq-out",
				0, &flags);
	if (pdata->irq_out.gpio < 0) {
              pr_err("%s get irq_out gpio fail\n",__func__);
		return -EINVAL;
       } else {
              pdata->irq_out.irq_name = "iqs229-out";
       }

       pdata->movout = of_get_named_gpio_flags(np, "iqs229,mvout",
				0, &flags);
	if (pdata->movout < 0) {
              pr_err("%s get movout gpio fail\n",__func__);
		return -EINVAL;
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

       pdata->vio = regulator_get(dev, "vio");
	if (IS_ERR(pdata->vio)) {
		rc = PTR_ERR(pdata->vio);
		pr_err("%s: Regulator get failed vio rc=%d\n", __func__, rc);
		return rc;
	}

       pdata->vdd = regulator_get(dev, "vdd");
	if (IS_ERR(pdata->vdd)) {
		rc = PTR_ERR(pdata->vdd);
		pr_err("%s: Regulator get failed vdd rc=%d\n", __func__, rc);
		return rc;
	}

	pr_debug("%s success\n",__func__);
	return 0;
}

static int iqs229_power_supply_on(struct iqs229_info *pdata)
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

	if (regulator_count_voltages(pdata->vio) > 0) {
		rc = regulator_set_voltage(pdata->vio,
			1800000, 1800000);
		if (rc) {
			pr_err("%s: Regulator set failed vio rc=%d\n",__func__, rc);
		}
	}

	rc = regulator_enable(pdata->vio);
	if (rc) {
		pr_err("%s: Regulator vio enable failed rc=%d\n", __func__, rc);
	}

	return rc;
}
static int iqs229_power_supply_off(struct iqs229_info *pdata)
{
	int rc=0;

	if (IS_ERR(pdata->vdd)) {
		rc = PTR_ERR(pdata->vdd);
		pr_err("%s: Regulator get failed vdd rc=%d\n", __func__, rc);
		return rc;
	}

	rc = regulator_disable(pdata->vdd);
	if (rc) {
		pr_err("%s: Regulator vdd diable failed rc=%d\n", __func__, rc);
	}

	if (regulator_count_voltages(pdata->vdd) > 0)
		regulator_set_voltage(pdata->vdd, 0, 1800000);

	regulator_put(pdata->vdd);

	if (IS_ERR(pdata->vio)) {
		rc = PTR_ERR(pdata->vio);
		pr_err("%s: Regulator get failed vdd rc=%d\n", __func__, rc);
		return rc;
	}

	rc = regulator_disable(pdata->vio);
	if (rc) {
		pr_err("%s: Regulator vdd diable failed rc=%d\n", __func__, rc);
	}

	if (regulator_count_voltages(pdata->vio) > 0)
		regulator_set_voltage(pdata->vio, 0, 1800000);

	regulator_put(pdata->vdd);
	return rc;
}

static int iqs229_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct iqs229_info *pdata;

	pr_debug("%s: enter \n", __func__);
		
	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(struct iqs229_info), GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: failed to alloc memory for module data\n",__func__);
			return -ENOMEM;
		}
		rc = iqs229_parse_dt(&pdev->dev, pdata);
		if (rc) {
			pr_err("%s parsing failed\n",__func__);
			goto free_struct;
		}
	} else{
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, pdata);
	/*input system config*/
	pdata->ipdev = input_allocate_device();
	if (!pdata->ipdev) {
		pr_err("%s: input_allocate_device fail\n",__func__);
		goto input_alloc_error;
	}
	pdata->ipdev->name = "capsensor";
	input_set_capability(pdata->ipdev, EV_KEY, KEY_CAP_FAR);
	input_set_capability(pdata->ipdev, EV_KEY, KEY_CAP_NEAR);
	set_bit(INPUT_PROP_NO_DUMMY_RELEASE, pdata->ipdev->propbit);
	rc = input_register_device(pdata->ipdev);
	if (rc) {
		pr_err("%s: input_register_device fail rc=%d\n",__func__, rc);
		goto input_register_error;
	}

	rc = iqs229_power_supply_on(pdata);
	if (rc) {
		pr_err("%s: iqs229_power_supply_on fail rc=%d\n",__func__, rc);
		goto input_error;
	}
	msleep(1);

	INIT_WORK(&pdata->irq_work, iqs229_irq_work);

	/*interrupt config*/
	rc = iqs229_init_irq(&pdata->irq_out);
	if (rc) {
		pr_err("%s: iqs229_init_irq fail rc=%d\n", __func__, rc);
		goto err_irq;
       }

	if (gpio_is_valid(pdata->movout)) {
		rc = gpio_request(pdata->movout, "movout");
		if (rc < 0) {
			pr_err("%s: gpio_request fail rc=%d\n",__func__, rc);
			goto err_gpio_req;
		}

		rc = gpio_direction_input(pdata->movout);
		if (rc < 0) {
			pr_err("%s: gpio_direction_input fail rc=%d\n", __func__, rc);
			goto err_gpio_dir;
		}
	}

	pdata->iqs_kobj=kobject_create_and_add("capsensor",NULL);
	rc = sysfs_create_group(pdata->iqs_kobj,&iqs229_attribute_group);
	if (rc < 0) {
		pr_err("%s: Failed to create sysfs attributes\n",__func__);
		goto err_gpio_dir;
	}

#if defined(CONFIG_FB)
	pdata->fb_notif.notifier_call = fb_notifier_callback;
	rc = fb_register_client(&pdata->fb_notif);
	if (rc){
		pr_err("%s: Unable to register fb_notifier: %d\n",__func__, rc);
		goto free_notifier;
	}
#endif


	iqs_pdata = pdata;
	iqs_enable_flag = 1;
	enable_irq(pdata->irq_out.irq);

       pr_debug("%s end\n",__func__);
       return 0;

#if defined(CONFIG_FB)
free_notifier:
	sysfs_remove_group(pdata->iqs_kobj,&iqs229_attribute_group);
#endif
err_gpio_dir:
	if(gpio_is_valid(pdata->movout))
		gpio_free(pdata->movout);
err_gpio_req:
	if(gpio_is_valid(pdata->irq_out.gpio))
		gpio_free(pdata->irq_out.gpio);
err_irq:
	cancel_work_sync(&pdata->irq_work);
	iqs229_power_supply_off(pdata);
input_error:
	input_unregister_device(pdata->ipdev);
input_register_error:
	input_free_device(pdata->ipdev);
input_alloc_error:
	platform_set_drvdata(pdev, NULL);
free_struct:
	kfree(pdata);

	return rc;
}

static int iqs229_remove(struct platform_device *pdev)
{
	struct iqs229_info *pdata = platform_get_drvdata(pdev);

#if defined(CONFIG_FB)
	fb_unregister_client(&pdata->fb_notif);
#endif
	sysfs_remove_group(pdata->iqs_kobj,&iqs229_attribute_group);
	if(pdata->irq_out.irq)
		free_irq(pdata->irq_out.irq, pdata);
	if(gpio_is_valid(pdata->irq_out.gpio))
		gpio_free(pdata->irq_out.gpio);
	if(gpio_is_valid(pdata->movout))
		gpio_free(pdata->movout);

	cancel_work_sync(&pdata->irq_work);
	iqs229_power_supply_off(pdata);

	input_unregister_device(pdata->ipdev);
	input_free_device(pdata->ipdev);
	platform_set_drvdata(pdev, NULL);
	kfree(pdata);
	return 0;
}


static struct of_device_id of_match_table[] = {
	{ .compatible = "cap,iqs229", },
	{ },
}; 

static struct platform_driver iqs229_driver = {
	.probe                = iqs229_probe,
	.remove                = iqs229_remove,
	.driver                = {
		.name        = "iqs229",
		.owner        = THIS_MODULE,
		.of_match_table = of_match_table,
	},
};

static int __init iqs229_init(void)
{
	return platform_driver_register(&iqs229_driver);
}

static void __exit iqs229_exit(void)
{
	platform_driver_unregister(&iqs229_driver);
}

module_init(iqs229_init);
module_exit(iqs229_exit);
MODULE_DESCRIPTION("Cap Sensor Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(" @longcheer.net");
