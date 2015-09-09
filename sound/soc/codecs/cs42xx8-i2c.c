/*
 * Cirrus Logic CS42448/CS42888 Audio CODEC DAI I2C driver
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * Author: Nicolin Chen <Guangyu.Chen@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>

#include "cs42xx8.h"

static int cs42xx8_i2c_probe(struct i2c_client *i2c)
{
	int ret = cs42xx8_probe(&i2c->dev,
			devm_regmap_init_i2c(i2c, &cs42xx8_regmap_config));
	if (ret)
		return ret;

	pm_runtime_enable(&i2c->dev);
	pm_request_idle(&i2c->dev);

	return 0;
}

static int cs42xx8_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	pm_runtime_disable(&i2c->dev);

	return 0;
}

static struct i2c_driver cs42xx8_i2c_driver = {
	.driver = {
		.name = "cs42xx8",
		.pm = &cs42xx8_pm,
		.of_match_table = cs42xx8_of_match,
	},
	.probe2 = cs42xx8_i2c_probe,
	.remove = cs42xx8_i2c_remove,
};

module_i2c_driver(cs42xx8_i2c_driver);

MODULE_DESCRIPTION("Cirrus Logic CS42448/CS42888 ALSA SoC Codec I2C Driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
