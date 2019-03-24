/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include "tusb544.h"
#include <linux/clk.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <linux/extcon.h>

#include <linux/usb/usbpd.h>
#include <linux/power_supply.h>

enum port_states {
	PORT_NONE,
	PORT_USB,	
	PORT_DP
};

struct delayed_work *g_sm_work;
enum port_states g_port_states;

#define REGISTER_GERNEAL_0A 0x0A
#define CTLSEL_MASK (BIT(0) | BIT(1))
#define CTLSEL_SHIFT 0
#define USB_ONLY 1
#define DP_ONLY 2

#define FLIP_MASK BIT(2)
#define FLIP_SHIFT 2
#define NO_FLIP 0
#define WITH_FLIP 1


struct tusb544_platform_data {
	unsigned int en_gpio;
	unsigned int hpd_gpio;
};

static const struct of_device_id msm_match_table[] = {
	{.compatible = "ti,tusb544"},
	{}
};

enum tusb544_state {
	TUSB544_DEFAULT = 0,
	TUSB544_TEST,
	TUSB544_REGISTER,
};

struct tusb544_dev {
	struct device *dev;
	struct	i2c_client	*client;

	/* GPIO variables */
	unsigned int		en_gpio;
	unsigned int		hpd_gpio;

	struct tusb544_platform_data *pdata;

	struct delayed_work	sm_work;

	enum tusb544_state	tusb544_state;
	u8 orientation;

	struct power_supply	*usb_psy;
	struct notifier_block	psy_nb;

	struct usbpd *pd;
	struct usbpd_svid_handler svid_handler;
	enum port_states port;
	enum power_supply_typec_mode typec_mode;
	enum power_supply_type	psy_type;
	bool			vbus_present;
	enum plug_orientation plug_orientation;

	bool isRunning;
};

static int tusb544_write(struct i2c_client *client, unsigned char addr, unsigned char val)
{
	int ret = 0;
	unsigned char buf[2] = { addr, val };

	ret = i2c_master_send(client, buf, 2);
	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_send Error\n", __func__);
		return -ENXIO;
	}

	return ret;
}

static int tusb544_read(struct i2c_client *client, unsigned char addr, unsigned char *val)
{
	int ret = 0;

	ret = i2c_master_send(client, &addr, 1);
	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_send Error\n", __func__);
		goto err_nfcc_hw_check;
	}
	/* hardware dependent delay */
	//msleep(30);

	/* Read Response of RESET command */
	ret = i2c_master_recv(client, val, 1);

	//dev_err(&client->dev,"%s: addr = %x (%x)\n", __func__, addr, *val);

	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_recv Error\n", __func__);
		goto err_nfcc_hw_check;
	}

	goto done;

err_nfcc_hw_check:
	ret = -ENXIO;
	dev_err(&client->dev,
		"%s: - NFCC HW not available\n", __func__);
done:
	return ret;
}

static int tusb544_hw_check(struct i2c_client *client, struct tusb544_dev *tusb544_dev)
{
	int ret = 0;
	unsigned char nci_reset_rsp[6];

	if (gpio_is_valid(tusb544_dev->en_gpio))
		gpio_direction_output(tusb544_dev->en_gpio, 1);

	msleep(30);

	/* Read Response of RESET command */
	ret = i2c_master_recv(client, nci_reset_rsp,
		sizeof(nci_reset_rsp));

	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_recv Error\n", __func__);
		goto err_hw_check;
	}

	dev_err(&client->dev,"%s: %x %x %x %x %x %x \n",
		__func__,
		nci_reset_rsp[0],	nci_reset_rsp[1], nci_reset_rsp[2],
		nci_reset_rsp[3],	nci_reset_rsp[4], nci_reset_rsp[5]);

	goto done;

err_hw_check:
	ret = -ENXIO;
	dev_err(&client->dev,
		"%s: - HW not available\n", __func__);
done:
	if (gpio_is_valid(tusb544_dev->en_gpio))
		gpio_direction_output(tusb544_dev->en_gpio, 0);
	return ret;
}

static int nfc_parse_dt(struct device *dev, struct tusb544_platform_data *pdata)
{
	int r = 0;
	struct device_node *np = dev->of_node;

	pdata->en_gpio = of_get_named_gpio(np, "fih,redriver-en", 0);
	if ((!gpio_is_valid(pdata->en_gpio)))
	{
		pr_err("%s gpio is invalid\n", __func__);
		return -EINVAL;
	}

	if (gpio_is_valid(pdata->en_gpio)) {
		r = gpio_request(pdata->en_gpio, "redriver_gpio_en");
		if (r) {
			pr_err("%s: unable to request gpio [%d]\n",
				__func__,
				pdata->en_gpio);
			return -EINVAL;
		}
	}

	r = gpio_direction_output(pdata->en_gpio, 1);

	pdata->hpd_gpio = of_get_named_gpio(np, "fih,hpd-gpio", 0);
	if ((!gpio_is_valid(pdata->hpd_gpio)))
	{
		pr_err("%s gpio is invalid\n", __func__);
		return -EINVAL;
	}

	if (gpio_is_valid(pdata->hpd_gpio)) {
		r = gpio_request(pdata->hpd_gpio, "redriver_gpio_hpd");
		if (r) {
			pr_err("%s: unable to request gpio [%d]\n",
				__func__,
				pdata->hpd_gpio);
			return -EINVAL;
		}
	}

	r = gpio_direction_output(pdata->hpd_gpio, 0);
	return r;
}

int tusb544_notify_dp_status(bool connected)
{
	pr_err("%s connected = %d\n", __func__, connected);

	if (connected == true)
		g_port_states = PORT_DP;
	else 
		g_port_states = PORT_NONE;

	schedule_delayed_work(g_sm_work, 0);

	return 0;
}

EXPORT_SYMBOL(tusb544_notify_dp_status);


static void tusb544_sm_work(struct work_struct *w)
{
	struct tusb544_dev *tusb544_dev = container_of(w, struct tusb544_dev, sm_work.work);
	unsigned char value = 0;

	if (tusb544_dev->tusb544_state == TUSB544_DEFAULT) 
	{		
		tusb544_dev->usb_psy = power_supply_get_by_name("usb");
		if (!tusb544_dev->usb_psy) {
			pr_err("Could not get USB power_supply, deferring probe\n");
		}
		tusb544_dev->tusb544_state = TUSB544_REGISTER;
	} 
	else if (tusb544_dev->tusb544_state == TUSB544_REGISTER)
	{
		enum plug_orientation orientiration;
		u8 flipValue, ctlselValue;

		pr_err("%s typec mode:%d present:%d type:%d orientation:%d\n",
			__func__,
			tusb544_dev->typec_mode, tusb544_dev->vbus_present, tusb544_dev->psy_type,
			tusb544_dev->plug_orientation);

		// cable is disappear
		if (tusb544_dev->typec_mode == POWER_SUPPLY_TYPEC_NONE) {
			if (gpio_is_valid(tusb544_dev->en_gpio))
				gpio_direction_output(tusb544_dev->en_gpio, 0);
			tusb544_dev->isRunning = false;
			return;
		}

		// cable is present
		if (tusb544_dev->isRunning == false) {
			if (gpio_is_valid(tusb544_dev->en_gpio))
			gpio_direction_output(tusb544_dev->en_gpio, 1);
			tusb544_dev->isRunning = true;
			// Enable cable mode
			tusb544_write(tusb544_dev->client, REGISTER_GERNEAL_0A, 0x41);
		}

		orientiration = tusb544_dev->plug_orientation;

		if (g_port_states == PORT_DP)
			ctlselValue = DP_ONLY;
		else
			ctlselValue = USB_ONLY;

		if (g_port_states == PORT_DP)
		{
			gpio_direction_output(tusb544_dev->hpd_gpio, 1);
		} else {
			gpio_direction_output(tusb544_dev->hpd_gpio, 0);
		}

		if (orientiration == ORIENTATION_CC2)
			flipValue = WITH_FLIP;
		else
			flipValue = NO_FLIP;

		tusb544_read(tusb544_dev->client, REGISTER_GERNEAL_0A, &value);
		pr_err("%s read value = %d\n",__func__, value);
		value &= ~FLIP_MASK;
		value |= (flipValue << FLIP_SHIFT);

		value &= ~CTLSEL_MASK;
		value |= (ctlselValue << CTLSEL_SHIFT);
		
		pr_err("%s try to write value = %d\n",__func__, value);
		tusb544_write(tusb544_dev->client, REGISTER_GERNEAL_0A, value);
	}
	
	return ;
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct tusb544_dev *pd = container_of(nb, struct tusb544_dev, psy_nb);
	union power_supply_propval val;
	enum power_supply_typec_mode typec_mode;
	int ret;
	
	if (ptr != pd->usb_psy || evt != PSY_EVENT_PROP_CHANGED)
		return 0;

	ret = power_supply_get_property(pd->usb_psy,
		POWER_SUPPLY_PROP_TYPEC_MODE, &val);
	if (ret) {
		pr_err("Unable to read USB TYPEC_MODE: %d\n", ret);
		return ret;
	}

	typec_mode = val.intval;

	ret = power_supply_get_property(pd->usb_psy,
			POWER_SUPPLY_PROP_PE_START, &val);
	if (ret) {
		pr_err("Unable to read USB PROP_PE_START: %d\n", ret);
		return ret;
	}

	/* Don't proceed if PE_START=0 as other props may still change */
	if (!val.intval &&
			typec_mode != POWER_SUPPLY_TYPEC_NONE)
		return 0;

	ret = power_supply_get_property(pd->usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &val);
	if (ret) {
		pr_err("Unable to read USB PRESENT: %d\n", ret);
		return ret;
	}

	pd->vbus_present = val.intval;

	ret = power_supply_get_property(pd->usb_psy,
			POWER_SUPPLY_PROP_TYPE, &val);
	if (ret) {
		pr_err("Unable to read USB TYPE: %d\n", ret);
		return ret;
	}

	pd->psy_type = val.intval;

	if (pd->typec_mode == typec_mode)
		return 0;

	pd->typec_mode = typec_mode;

	ret = power_supply_get_property(pd->usb_psy,
			POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION, &val);
	if (ret) {
		pr_err("Unable to read USB TYPE: %d\n", ret);
		return ret;
	}

	pd->plug_orientation = val.intval;

	pr_err("%s typec mode:%d present:%d type:%d orientation:%d\n",
		__func__,
		typec_mode, pd->vbus_present, pd->psy_type,
		pd->plug_orientation);

	schedule_delayed_work(&pd->sm_work, 0);

	return 0;
}

static int tusb544_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int r = 0;
	struct tusb544_platform_data *platform_data;
	struct tusb544_dev *tusb544_dev;
	int ret;

	dev_err(&client->dev, "%s: enter\n", __func__);

	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
			sizeof(struct tusb544_platform_data), GFP_KERNEL);
		if (!platform_data) {
			r = -ENOMEM;
			goto err_platform_data;
		}

		r = nfc_parse_dt(&client->dev, platform_data);
		if (r)
			goto err_free_data;
	} else
		platform_data = client->dev.platform_data;

	if (platform_data == NULL) {
		dev_err(&client->dev, "%s: failed\n", __func__);
		r = -ENODEV;
		goto err_platform_data;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: need I2C_FUNC_I2C\n", __func__);
		r = -ENODEV;
		goto err_free_data;
	}

	tusb544_dev = kzalloc(sizeof(*tusb544_dev), GFP_KERNEL);
	if (tusb544_dev == NULL) {
		r = -ENOMEM;
		goto err_free_data;
	}

	tusb544_dev->client = client;
	tusb544_dev->en_gpio = platform_data->en_gpio;
	tusb544_dev->hpd_gpio = platform_data->hpd_gpio;
	tusb544_dev->pdata = platform_data;
	tusb544_dev->tusb544_state = TUSB544_DEFAULT;
	tusb544_dev->typec_mode = POWER_SUPPLY_TYPEC_NONE;
	tusb544_dev->isRunning= false;

	i2c_set_clientdata(client, tusb544_dev);
	tusb544_dev->dev = &client->dev;

	INIT_DELAYED_WORK(&tusb544_dev->sm_work, tusb544_sm_work);

	g_sm_work = &tusb544_dev->sm_work;

	tusb544_hw_check(client, tusb544_dev);

	// Regist psy change
	tusb544_dev->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&tusb544_dev->psy_nb);
	if (ret)
		pr_err("%s fail to register power supply\n",__func__);

	// to statup a register timer for usb notifier
	schedule_delayed_work(&tusb544_dev->sm_work, (msecs_to_jiffies(5000)));

	return 0;

err_free_data:
	if (client->dev.of_node)
		devm_kfree(&client->dev, platform_data);
err_platform_data:
	dev_err(&client->dev,
	"%s: probing failed, check hardware\n",
		 __func__);
	return r;
}


static int tusb544_remove(struct i2c_client *client)
{
	int ret = 0;
	struct tusb544_dev *tusb544_dev;

	tusb544_dev = i2c_get_clientdata(client);
	if (!tusb544_dev) {
		dev_err(&client->dev,
		"%s: device doesn't exist anymore\n", __func__);
		ret = -ENODEV;
		goto err;
	}

	kfree(tusb544_dev);
err:
	return ret;
}

static int tusb544_suspend(struct device *device)
{
	return 0;
}

static int tusb544_resume(struct device *device)
{
	return 0;
}

static const struct i2c_device_id tusb544_id[] = {
	{"tusb544-i2c", 0},
	{}
};

static const struct dev_pm_ops tusb544_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tusb544_suspend, tusb544_resume)
};

static struct i2c_driver tusb544 = {
	.id_table = tusb544_id,
	.probe = tusb544_probe,
	.remove = tusb544_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tusb544",
		.of_match_table = msm_match_table,
		.pm = &tusb544_pm_ops,
	},
};

/*
 * module load/unload record keeping
 */
static int __init tusb544_dev_init(void)
{
	return i2c_add_driver(&tusb544);
}
module_init(tusb544_dev_init);

static void __exit tusb544_dev_exit(void)
{
	i2c_del_driver(&tusb544);
}
module_exit(tusb544_dev_exit);

MODULE_DESCRIPTION("TI TUSB544 Redriver");
MODULE_LICENSE("GPL v2");
