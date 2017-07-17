/*
 * Maxim MAX9286 GMSL Deserializer Driver
 *
 * Copyright (C) 2016 Renesas Electronics Corporation
 * Copyright (C) 2015 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#define MAX9286_NUM_PORTS	1	/* Number of ports (usually 4, can be lower for debugging) */

#define MAXIM_I2C_I2C_SPEED_400KHZ	(0x5 << 2) /* 339 kbps */
#define MAXIM_I2C_I2C_SPEED_100KHZ	(0x3 << 2) /* 105 kbps */
#define MAXIM_I2C_SPEED			MAXIM_I2C_I2C_SPEED_100KHZ

/*
 * MCU powered IMI cameras require delay between power-on and RCar access to
 * avoid i2c bus conflicts.
 */
#define MAXIM_IMI_MCU_V0_DELAY	8000	/* delay for powered MCU firmware v0 */
#define MAXIM_IMI_MCU_V1_DELAY	3000	/* delay for powered MCU firmware v1 */
#define MAXIM_IMI_MCU_NO_DELAY	0	/* delay for unpowered MCU  */
#define MAXIM_IMI_MCU_DELAY	MAXIM_IMI_MCU_V0_DELAY

/*
 * I2C MAP.
 *
 *		CAM0	CAM1	CAM2	CAM3
 * MAX9286	0x48+1	0x48+2	0x48+3	0x48+4	- deserializer
 * MAX9271	0x40+1	0x40+2	0x40+3	0x40+4	- serializer
 * OV10635	0x30+1	0x30+2	0x30+3	0x30+4	- sensor
 */

#define DES0			0x4c
#define DES1			0x6c
#define SER			0x51
#define CAM			0x60
#define BROADCAST		0x6f

static const u8 maxim_map[][8] = {
	{ SER  + 0, SER  + 1, SER  + 2, SER  + 3,
	  SER  + 4, SER  + 5, SER  + 6, SER  + 7 },
	{ CAM  + 0, CAM  + 1, CAM  + 2, CAM  + 3,
	  CAM  + 4, CAM  + 5, CAM  + 6, CAM  + 7 },
};

struct max9286_device {
	struct i2c_client *client;
	struct regulator *regulator;
	struct i2c_mux_core *mux;
};

static inline int max9286_write(struct max9286_device *dev, u8 reg, u8 val)
{
	int ret;

	dev_dbg(&dev->client->dev, "%s(0x%02x, 0x%02x)\n", __func__, reg, val);

	ret = i2c_smbus_write_byte_data(dev->client, reg, val);
	if (ret < 0)
		dev_dbg(&dev->client->dev,
			"%s: register 0x%02x write failed (%d)\n",
			__func__, reg, ret);

	return ret;
}

/* -----------------------------------------------------------------------------
 * I2C Multiplexer
 */

static int max9286_i2c_mux_select(struct i2c_mux_core *muxc, u32 chan)
{
	struct max9286_device *dev = i2c_mux_priv(muxc);

	return max9286_write(dev, 0x0a, 0xf0 | (0x01 << chan));
}

static int max9286_i2c_mux_deselect(struct i2c_mux_core *muxc, u32 chan)
{
	struct max9286_device *dev = i2c_mux_priv(muxc);

	return max9286_write(dev, 0x0a, 0xf0);
}

static int max9286_i2c_mux_init(struct max9286_device *dev)
{
	unsigned int i;
	int ret;

	if (!i2c_check_functionality(dev->client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -ENODEV;

	dev->mux = i2c_mux_alloc(dev->client->adapter, &dev->client->dev,
				 MAX9286_NUM_PORTS, 0, I2C_MUX_LOCKED,
				 max9286_i2c_mux_select,
				 max9286_i2c_mux_deselect);
	dev_info(&dev->client->dev, "%s: mux %p\n", __func__, dev->mux);
	if (!dev->mux)
		return -ENOMEM;

	dev->mux->priv = dev;

	for (i = 0; i < MAX9286_NUM_PORTS; ++i) {
		ret = i2c_mux_add_adapter(dev->mux, 0, i, 0);
		if (ret < 0)
			goto error;
	}

	return 0;

error:
	i2c_mux_del_adapters(dev->mux);
	return ret;
}

/* -----------------------------------------------------------------------------
 * Probe/Remove
 */

static int max9286_setup(struct max9286_device *dev)
{
	unsigned short des_addr = dev->client->addr;
	unsigned int cam_idx, cam_offset;

	switch (des_addr) {
	case DES0:
		cam_offset = 0;
		dev->client->addr = DES1;		/* MAX9286-CAMx I2C */
		max9286_write(dev, 0x0a, 0x00);
				/* disable reverse control for all cams */
		dev->client->addr = des_addr;
		break;
	case DES1:
		cam_offset = 4;
		break;
	default:
		return -EINVAL;
	}

	for (cam_idx = cam_offset; cam_idx < MAX9286_NUM_PORTS + cam_offset; cam_idx++) {
		/*
		 * SETUP CAMx (MAX9286/MAX9271/OV10635) I2C
		 */
		dev_info(&dev->client->dev,
			 "SETUP CAM%d(MAX9286/MAX9271/OV10635)I2C: 0x%x<->0x%x<->0x%x\n",
			 cam_idx, des_addr,
			 maxim_map[0][cam_idx], maxim_map[1][cam_idx]);

		/* Reverse channel setup */
		dev->client->addr = des_addr;		/* MAX9286-CAMx I2C */
		max9286_write(dev, 0x0a,
			0xf0 | (1 << (cam_idx - cam_offset)));
				/* enable reverse control only for cam_idx */
		mdelay(2);
			/* wait 2ms after any change of reverse
			*  channel settings
			*/

		dev->client->addr = des_addr;		/* MAX9286-CAMx I2C */
		max9286_write(dev, 0x3f, 0x4f);
			/* enable custom reverse channel & first pulse length */
		max9286_write(dev, 0x34, 0xa2 | MAXIM_I2C_SPEED);
			/* enable artificial ACKs, I2C speed set */
		mdelay(2);
			/* wait 2ms after any change of reverse
			*  channel settings
			*/
		max9286_write(dev, 0x3b, 0x1e);
			/* first pulse length rise time changed
			*  from 300ns to 200ns
			*/
		mdelay(2);
			/* wait 2ms after any change of
			*  reverse channel settings
			*/

		dev->client->addr = 0x40;		/* MAX9271-CAMx I2C */
		i2c_smbus_read_byte(dev->client);	/* ping to wake-up */
		max9286_write(dev, 0x04, 0x43);
				/* wake-up, enable reverse_control/conf_link */
		mdelay(5);	/* wait 5ms for conf_link to establish */
		max9286_write(dev, 0x08, 0x1);
			/* reverse channel receiver high threshold enable */
		mdelay(2);
			/* wait 2ms after any change of reverse
			*  channel settings
			*/

		dev->client->addr = des_addr;		/* MAX9286-CAMx I2C */
		max9286_write(dev, 0x3b, 0x19);
			/* reverse channel increase amplitude 170mV
			*  to compensate high threshold enabled
			*/
		mdelay(2);
			/* wait 2ms after any change of reverse
			*  channel settings
			*/

		/* re-setup for the case of s/w reboot */
		dev->client->addr = 0x40;		/* MAX9271-CAMx I2C */
		max9286_write(dev, 0x04, 0x43);
			/* wake-up, enable reverse_control/conf_link */
		mdelay(5);	/* wait 5ms for conf_link to establish */
		max9286_write(dev, 0x08, 0x1);
			/* reverse channel receiver high threshold enable */
		mdelay(2);	/* wait 2ms after any change of reverse
				*  channel settings
				*/

		/* Initial setup */
		dev->client->addr = des_addr;		/* MAX9286-CAMx I2C */
		max9286_write(dev, 0x15, 0x13);
			/* disable CSI output, VC is set accordingly
			*  to Link number
			*/
		if (MAX9286_NUM_PORTS == 1)
			max9286_write(dev, 0x12, 0x33);
			/* enable CSI-2 Lane D0 only, DBL mode, YUV422 8-bit*/
		else
			max9286_write(dev, 0x12, 0xf3);
			/* enable CSI-2 Lanes D[0:3], DBL mode, YUV422  8-bit*/
#define FSYNC_PERIOD	(1280*800*2)
#if 0
		max9286_write(dev, 0x01, 0x00);
			/* manual: FRAMESYNC set manually
			*  via [0x06:0x08] regs
			*/
#endif
		max9286_write(dev, 0x06, FSYNC_PERIOD & 0xff);
		max9286_write(dev, 0x07, (FSYNC_PERIOD >> 8) & 0xff);
		max9286_write(dev, 0x08, FSYNC_PERIOD >> 16);
		if (MAX9286_NUM_PORTS == 1) {
			max9286_write(dev, 0x01, 0xc0);
				/* ECU (aka MCU) based FrameSync using
				*  GPI-to-GPO
				*/
			max9286_write(dev, 0x00, 0xe1);
				/* enable GMSL link 0, auto detect link
				*  used for CSI clock source
				*/
			max9286_write(dev, 0x69, 0x0e);
				/* Mask Links 1 2 3, unmask link 0 */
		} else {
			max9286_write(dev, 0x01, 0x02);
				/* automatic: FRAMESYNC taken
				*  from the slowest Link
				*/
			max9286_write(dev, 0x00, 0xef);
				/* enable GMSL links [0:3], auto detect link
				* used for CSI clock source
				*/
		}
		max9286_write(dev, 0x0c, 0x89);
			/* enable HS/VS encoding, use D14/15 for HS/VS,
			* invert VS
			*/

		/* GMSL setup */
		dev->client->addr = 0x40;		/* MAX9271-CAMx I2C */
		max9286_write(dev, 0x0d, 0x22 | MAXIM_I2C_SPEED);
				/* disable artificial ACK, I2C speed set */
		max9286_write(dev, 0x07, 0x94);
			/* RAW/YUV, PCLK rising edge, HS/VS encoding enabled */
#if 0
		max9286_write(dev, 0x02, 0xff);
			/* spread spectrum +-4%, pclk range automatic,
				Gbps automatic  */
#endif
		dev->client->addr = des_addr;		/* MAX9286-CAMx I2C */
		max9286_write(dev, 0x34, 0x22 | MAXIM_I2C_SPEED);
				/* disable artificial ACK, I2C speed set */
		mdelay(2);			/* wait 2ms */

		/* I2C translator setup */
		dev->client->addr = 0x40;		/* MAX9271-CAMx I2C */
		max9286_write(dev, 0x09, maxim_map[1][cam_idx] << 1);
							/* OV10635 I2C new */
		max9286_write(dev, 0x0A, 0x30 << 1);
							/* OV10635 I2C */
		max9286_write(dev, 0x0B, BROADCAST << 1);
							/* broadcast I2C */
		max9286_write(dev, 0x0C, maxim_map[0][cam_idx] << 1);
						/* MAX9271-CAMx I2C new */

		/* I2C addresses change */
		dev->client->addr = 0x40;		/* MAX9271-CAMx I2C */
		max9286_write(dev, 0x01, des_addr << 1);
						/* MAX9286-CAM0 I2C new */
		max9286_write(dev, 0x00, maxim_map[0][cam_idx] << 1);
						/* MAX9271-CAM0 I2C new */

		/* make sure that the conf_link enabled -
		*  needed for reset/reboot, due to I2C runtime changeing
		*/
		dev->client->addr = maxim_map[0][cam_idx];
							/* MAX9271-CAMx I2C new */
		max9286_write(dev, 0x04, 0x43);
				/* wake-up, enable reverse_control/conf_link */
		mdelay(5);	/* wait 5ms for conf_link to establish */
	}

	/* Reverse channel setup */
	dev->client->addr = des_addr;			/* MAX9286-CAMx I2C */
	max9286_write(dev, 0x1b, 0x0f);
				/* enable equalizer for all links */
	if (MAX9286_NUM_PORTS == 1)
		max9286_write(dev, 0x0a, 0xf1);
				/* enable reverse control only for link0 */
	else
		max9286_write(dev, 0x0a, 0xff);
				/* enable reverse control for all cams */
	mdelay(2);	/* wait 2ms after any change of reverse
			* channel settings
			*/

	/* NOTE: release des_addr to be used in sensor subdevice */
	dev->client->addr = des_addr - 0x40;

	return 0;
}

static int max9286_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct max9286_device *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->client = client;
	i2c_set_clientdata(client, dev);

	dev->regulator = regulator_get(&client->dev, "poc");
	if (IS_ERR(dev->regulator)) {
		if (PTR_ERR(dev->regulator) != -EPROBE_DEFER)
			dev_err(&client->dev,
				"Unable to get PoC regulator (%ld)\n",
				PTR_ERR(dev->regulator));
		ret = PTR_ERR(dev->regulator);
		goto err_free;
	}

	ret = regulator_enable(dev->regulator);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to turn PoC on\n");
		goto err_regulator;
	}

	/*
	 * Powered MCU IMI cameras need delay between power-on and R-Car access
	 * to avoid i2c bus conflicts since linux kernel does not support i2c
	 * multi-mastering, IMI MCU is master and R-Car is also master. The i2c
	 * bus conflict results in R-Car i2c IP stall.
	 */
	msleep(MAXIM_IMI_MCU_DELAY);

	ret = max9286_setup(dev);
	if (ret < 0)
		goto err_regulator;

	ret = max9286_i2c_mux_init(dev);
	if (ret < 0)
		goto err_regulator;

	return 0;

err_regulator:
	regulator_put(dev->regulator);
err_free:
	kfree(dev);
	return ret;
}

static int max9286_remove(struct i2c_client *client)
{
	struct max9286_device *dev = i2c_get_clientdata(client);

	i2c_mux_del_adapters(dev->mux);

	regulator_disable(dev->regulator);
	regulator_put(dev->regulator);

	kfree(dev);

	return 0;
}

static const struct of_device_id max9286_dt_ids[] = {
	{ .compatible = "maxim,max9286" },
	{},
};
MODULE_DEVICE_TABLE(of, max9286_dt_ids);

static const struct i2c_device_id max9286_id[] = {
	{ "max9286", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max9286_id);

static struct i2c_driver max9286_i2c_driver = {
	.driver	= {
		.name		= "max9286",
		.of_match_table	= of_match_ptr(max9286_dt_ids),
	},
	.probe		= max9286_probe,
	.remove		= max9286_remove,
	.id_table	= max9286_id,
};

module_i2c_driver(max9286_i2c_driver);

MODULE_DESCRIPTION("Maxim MAX9286 GMSL Deserializer Driver");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
