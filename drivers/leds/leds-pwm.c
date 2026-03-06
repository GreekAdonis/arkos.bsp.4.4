/*
 * linux/drivers/leds-pwm.c
 *
 * simple PWM based LED control
 *
 * Copyright 2009 Luotao Fu @ Pengutronix (l.fu@pengutronix.de)
 *
 * based on leds-gpio.c by Raphael Assenat <raph@8d.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/fb.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>

struct led_pwm_data {
	struct led_classdev	cdev;
	struct pwm_device	*pwm;
	struct work_struct	work;
	unsigned int		active_low;
	unsigned int		period;
	int			duty;
	bool			can_sleep;
	/* battery brightness scaling */
	bool			battery_scale;
	struct delayed_work	battery_work;
	struct power_supply	*psy;
};

struct led_pwm_priv {
	int num_leds;
	struct led_pwm_data leds[0];
};

static void __led_pwm_set(struct led_pwm_data *led_dat)
{
	int new_duty = led_dat->duty;

	pwm_config(led_dat->pwm, new_duty, led_dat->period);

	if (new_duty == 0)
		pwm_disable(led_dat->pwm);
	else
		pwm_enable(led_dat->pwm);
}

static void led_pwm_work(struct work_struct *work)
{
	struct led_pwm_data *led_dat =
		container_of(work, struct led_pwm_data, work);

	__led_pwm_set(led_dat);
}

static void led_pwm_set(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct led_pwm_data *led_dat =
		container_of(led_cdev, struct led_pwm_data, cdev);
	unsigned int max = led_dat->cdev.max_brightness;
	unsigned long long duty =  led_dat->period;

	duty *= brightness;
	do_div(duty, max);

	if (led_dat->active_low)
		duty = led_dat->period - duty;

	led_dat->duty = duty;

	if (led_dat->can_sleep)
		schedule_work(&led_dat->work);
	else
		__led_pwm_set(led_dat);
}

static void led_pwm_battery_work(struct work_struct *work)
{
	struct led_pwm_data *led_dat =
		container_of(to_delayed_work(work), struct led_pwm_data, battery_work);
	union power_supply_propval val;
	int capacity, brightness;

	if (!led_dat->psy || !led_dat->battery_scale)
		return;

	if (power_supply_get_property(led_dat->psy, POWER_SUPPLY_PROP_CAPACITY, &val))
		goto reschedule;

	capacity = val.intval;
	if (capacity < 0)
		capacity = 0;
	if (capacity > 100)
		capacity = 100;

	/*
	 * Brightness mapping:
	 *   51-100% -> 255 (full brightness)
	 *   30-50%  -> 140 (dim)
	 *   0-29%   -> 0   (off)
	 */
	if (capacity >= 51)
		brightness = 255;
	else if (capacity >= 30)
		brightness = 140;
	else
		brightness = 0;

	led_pwm_set(&led_dat->cdev, brightness);

reschedule:
	schedule_delayed_work(&led_dat->battery_work, msecs_to_jiffies(5000));
}

static int led_pwm_battery_init(struct device *dev, struct led_pwm_data *led_dat)
{
	led_dat->psy = power_supply_get_by_name("battery");
	if (!led_dat->psy) {
		dev_info(dev, "battery power supply not found, battery_scale disabled\n");
		return -ENODEV;
	}

	INIT_DELAYED_WORK(&led_dat->battery_work, led_pwm_battery_work);
	schedule_delayed_work(&led_dat->battery_work, msecs_to_jiffies(1000));

	dev_info(dev, "battery_scale enabled for %s\n", led_dat->cdev.name);
	return 0;
}

static void led_pwm_battery_exit(struct led_pwm_data *led_dat)
{
	cancel_delayed_work_sync(&led_dat->battery_work);
	if (led_dat->psy) {
		power_supply_put(led_dat->psy);
		led_dat->psy = NULL;
	}
}

static inline size_t sizeof_pwm_leds_priv(int num_leds)
{
	return sizeof(struct led_pwm_priv) +
		      (sizeof(struct led_pwm_data) * num_leds);
}

static void led_pwm_cleanup(struct led_pwm_priv *priv)
{
	while (priv->num_leds--) {
		struct led_pwm_data *led_dat = &priv->leds[priv->num_leds];
		
		if (led_dat->battery_scale)
			led_pwm_battery_exit(led_dat);
		
		led_classdev_unregister(&led_dat->cdev);
		if (led_dat->can_sleep)
			cancel_work_sync(&led_dat->work);
	}
}

static int led_pwm_add(struct device *dev, struct led_pwm_priv *priv,
		       struct led_pwm *led, struct device_node *child)
{
	struct led_pwm_data *led_data = &priv->leds[priv->num_leds];
	struct pwm_args pargs;
	int ret;

	led_data->active_low = led->active_low;
	led_data->cdev.name = led->name;
	led_data->cdev.default_trigger = led->default_trigger;
	led_data->cdev.brightness_set = led_pwm_set;
	led_data->cdev.brightness = LED_OFF;
	led_data->cdev.max_brightness = led->max_brightness;
	led_data->cdev.flags = LED_CORE_SUSPENDRESUME;

	if (child)
		led_data->pwm = devm_of_pwm_get(dev, child, NULL);
	else
		led_data->pwm = devm_pwm_get(dev, led->name);
	if (IS_ERR(led_data->pwm)) {
		ret = PTR_ERR(led_data->pwm);
		dev_err(dev, "unable to request PWM for %s: %d\n",
			led->name, ret);
		return ret;
	}

	led_data->can_sleep = pwm_can_sleep(led_data->pwm);
	if (led_data->can_sleep)
		INIT_WORK(&led_data->work, led_pwm_work);

	/*
	 * FIXME: pwm_apply_args() should be removed when switching to the
	 * atomic PWM API.
	 */
	pwm_apply_args(led_data->pwm);

	pwm_get_args(led_data->pwm, &pargs);

	led_data->period = pargs.period;
	if (!led_data->period && (led->pwm_period_ns > 0))
		led_data->period = led->pwm_period_ns;

	ret = led_classdev_register(dev, &led_data->cdev);
	if (ret == 0) {
		priv->num_leds++;
		led_pwm_set(&led_data->cdev, led_data->cdev.brightness);
	} else {
		dev_err(dev, "failed to register PWM led for %s: %d\n",
			led->name, ret);
	}

	return ret;
}

static int led_pwm_create_of(struct device *dev, struct led_pwm_priv *priv)
{
	struct device_node *child;
	struct led_pwm led;
	int ret = 0;

	memset(&led, 0, sizeof(led));

	for_each_child_of_node(dev->of_node, child) {
		const char *state;
		led.name = of_get_property(child, "label", NULL) ? :
			   child->name;

		led.default_trigger = of_get_property(child,
						"linux,default-trigger", NULL);
		led.active_low = of_property_read_bool(child, "active-low");
		of_property_read_u32(child, "max-brightness",
				     &led.max_brightness);

		ret = led_pwm_add(dev, priv, &led, child);
		if (ret) {
			of_node_put(child);
			break;
		}

		/* Handle default-state property */
		state = of_get_property(child, "default-state", NULL);
		if (state) {
			struct led_pwm_data *led_data = &priv->leds[priv->num_leds - 1];
			if (!strcmp(state, "on"))
				led_pwm_set(&led_data->cdev, led_data->cdev.max_brightness);
			else if (!strcmp(state, "off"))
				led_pwm_set(&led_data->cdev, LED_OFF);
		}

		/* Handle battery-scale property */
		if (of_property_read_bool(child, "battery-scale")) {
			struct led_pwm_data *led_data = &priv->leds[priv->num_leds - 1];
			led_data->battery_scale = true;
			led_pwm_battery_init(dev, led_data);
		}
	}

	return ret;
}

static int led_pwm_probe(struct platform_device *pdev)
{
	struct led_pwm_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct led_pwm_priv *priv;
	int count, i;
	int ret = 0;

	if (pdata)
		count = pdata->num_leds;
	else
		count = of_get_child_count(pdev->dev.of_node);

	if (!count)
		return -EINVAL;

	priv = devm_kzalloc(&pdev->dev, sizeof_pwm_leds_priv(count),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (pdata) {
		for (i = 0; i < count; i++) {
			ret = led_pwm_add(&pdev->dev, priv, &pdata->leds[i],
					  NULL);
			if (ret)
				break;
		}
	} else {
		ret = led_pwm_create_of(&pdev->dev, priv);
	}

	if (ret) {
		led_pwm_cleanup(priv);
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int led_pwm_remove(struct platform_device *pdev)
{
	struct led_pwm_priv *priv = platform_get_drvdata(pdev);

	led_pwm_cleanup(priv);

	return 0;
}

static const struct of_device_id of_pwm_leds_match[] = {
	{ .compatible = "pwm-leds", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_leds_match);

static struct platform_driver led_pwm_driver = {
	.probe		= led_pwm_probe,
	.remove		= led_pwm_remove,
	.driver		= {
		.name	= "leds_pwm",
		.of_match_table = of_pwm_leds_match,
	},
};

module_platform_driver(led_pwm_driver);

MODULE_AUTHOR("Luotao Fu <l.fu@pengutronix.de>");
MODULE_DESCRIPTION("PWM LED driver for PXA");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-pwm");
