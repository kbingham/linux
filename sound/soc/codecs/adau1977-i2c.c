/*
 * ADAU1977/ADAU1978/ADAU1979 driver
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

#include "adau1977.h"

static int adau1977_i2c_probe(struct i2c_client *client)
{
	struct regmap_config config;

	config = adau1977_regmap_config;
	config.val_bits = 8;
	config.reg_bits = 8;

	return adau1977_probe(&client->dev,
		devm_regmap_init_i2c(client, &config),
		id->driver_data, NULL);
}

static int adau1977_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static struct i2c_driver adau1977_i2c_driver = {
	.driver = {
		.name = "adau1977",
	},
	.probe2 = adau1977_i2c_probe,
	.remove = adau1977_i2c_remove,
};
module_i2c_driver(adau1977_i2c_driver);

MODULE_DESCRIPTION("ASoC ADAU1977/ADAU1978/ADAU1979 driver");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
