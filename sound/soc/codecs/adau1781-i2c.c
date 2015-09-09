/*
 * Driver for ADAU1381/ADAU1781 CODEC
 *
 * Copyright 2014 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#include "adau1781.h"

static int adau1781_i2c_probe(struct i2c_client *client)
{
	struct regmap_config config;

	config = adau1781_regmap_config;
	config.val_bits = 8;
	config.reg_bits = 16;

	return adau1781_probe(&client->dev,
		devm_regmap_init_i2c(client, &config),
		id->driver_data, NULL);
}

static int adau1781_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static struct i2c_driver adau1781_i2c_driver = {
	.driver = {
		.name = "adau1781",
	},
	.probe2 = adau1781_i2c_probe,
	.remove = adau1781_i2c_remove,
};
module_i2c_driver(adau1781_i2c_driver);

MODULE_DESCRIPTION("ASoC ADAU1381/ADAU1781 CODEC I2C driver");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
