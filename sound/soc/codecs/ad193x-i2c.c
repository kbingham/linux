/*
 * AD1936/AD1937 audio driver
 *
 * Copyright 2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>

#include <sound/soc.h>

#include "ad193x.h"

static int ad193x_i2c_probe(struct i2c_client *client)
{
	struct regmap_config config;

	config = ad193x_regmap_config;
	config.val_bits = 8;
	config.reg_bits = 8;

	return ad193x_probe(&client->dev, devm_regmap_init_i2c(client, &config));
}

static int ad193x_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static struct i2c_driver ad193x_i2c_driver = {
	.driver = {
		.name = "ad193x",
	},
	.probe2 = ad193x_i2c_probe,
	.remove   = ad193x_i2c_remove,
};
module_i2c_driver(ad193x_i2c_driver);

MODULE_DESCRIPTION("ASoC AD1936/AD1937 audio CODEC driver");
MODULE_AUTHOR("Barry Song <21cnbao@gmail.com>");
MODULE_LICENSE("GPL");
