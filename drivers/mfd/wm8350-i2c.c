/*
 * wm8350-i2c.c  --  Generic I2C driver for Wolfson WM8350 PMIC
 *
 * Copyright 2007, 2008 Wolfson Microelectronics PLC.
 *
 * Author: Liam Girdwood
 *         linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/mfd/wm8350/core.h>
#include <linux/regmap.h>
#include <linux/slab.h>

static int wm8350_i2c_probe(struct i2c_client *i2c)
{
	struct wm8350 *wm8350;
	struct wm8350_platform_data *pdata = dev_get_platdata(&i2c->dev);
	int ret = 0;

	wm8350 = devm_kzalloc(&i2c->dev, sizeof(struct wm8350), GFP_KERNEL);
	if (wm8350 == NULL)
		return -ENOMEM;

	wm8350->regmap = devm_regmap_init_i2c(i2c, &wm8350_regmap);
	if (IS_ERR(wm8350->regmap)) {
		ret = PTR_ERR(wm8350->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	i2c_set_clientdata(i2c, wm8350);
	wm8350->dev = &i2c->dev;

	return wm8350_device_init(wm8350, i2c->irq, pdata);
}

static int wm8350_i2c_remove(struct i2c_client *i2c)
{
	struct wm8350 *wm8350 = i2c_get_clientdata(i2c);

	wm8350_device_exit(wm8350);

	return 0;
}


static struct i2c_driver wm8350_i2c_driver = {
	.driver = {
		   .name = "wm8350",
	},
	.probe2 = wm8350_i2c_probe,
	.remove = wm8350_i2c_remove,
};

static int __init wm8350_i2c_init(void)
{
	return i2c_add_driver(&wm8350_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(wm8350_i2c_init);

static void __exit wm8350_i2c_exit(void)
{
	i2c_del_driver(&wm8350_i2c_driver);
}
module_exit(wm8350_i2c_exit);

MODULE_DESCRIPTION("I2C support for the WM8350 AudioPlus PMIC");
MODULE_LICENSE("GPL");
