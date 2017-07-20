/*
 * IMI RDACM20 GMSL Camera Driver
 *
 * Copyright (C) 2016 Renesas Electronics Corporation
 * Copyright (C) 2015 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

/*
 * The camera is mode of an Omnivision OV10635 sensor connected to a Maxim
 * MAX9271 GMSL serializer.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#include "rdacm20-ov10635.h"

#define MAX9271_I2C_ADDRESS		0x40

/* Register 0x04 */
#define MAX9271_SEREN			BIT(7)
#define MAX9271_CLINKEN			BIT(6)
#define MAX9271_PRBSEN			BIT(5)
#define MAX9271_SLEEP			BIT(4)
#define MAX9271_INTTYPE_I2C		(0 << 2)
#define MAX9271_INTTYPE_UART		(1 << 2)
#define MAX9271_INTTYPE_NONE		(2 << 2)
#define MAX9271_REVCCEN			BIT(1)
#define MAX9271_FWDCCEN			BIT(0)
/* Register 0x09 */
#define MAX9271_ID			0x09
/* Register 0x0f */
#define MAX9271_GPIO5OUT		BIT(5)
#define MAX9271_GPIO4OUT		BIT(4)
#define MAX9271_GPIO3OUT		BIT(3)
#define MAX9271_GPIO2OUT		BIT(2)
#define MAX9271_GPIO1OUT		BIT(1)
#define MAX9271_SETGPO			BIT(0)

#define OV10635_I2C_ADDRESS		0x30

#define OV10635_SOFTWARE_RESET		0x0103
#define OV10635_PID			0x300a
#define OV10635_VER			0x300b
#define OV10635_SC_CMMN_SCCB_ID		0x300c
#define OV10635_SC_CMMN_SCCB_ID_SELECT	BIT(0)
#define OV10635_VERSION			0xa635

#define OV10635_WIDTH			1280
#define OV10635_HEIGHT			800
#define OV10635_FORMAT			MEDIA_BUS_FMT_UYVY8_2X8
/* #define OV10635_FORMAT			MEDIA_BUS_FMT_UYVY10_2X10 */

struct rdacm20_device {
	struct i2c_client		*client;
	struct i2c_client		*sensor;
	struct v4l2_subdev		sd;
	struct media_pad		pad;
	struct v4l2_ctrl_handler	ctrls;
};

static inline struct rdacm20_device *sd_to_rdacm20(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rdacm20_device, sd);
}

static inline struct rdacm20_device *i2c_to_rdacm20(struct i2c_client *client)
{
	return sd_to_rdacm20(i2c_get_clientdata(client));
}

static int max9271_read(struct rdacm20_device *dev, u8 reg)
{
	int ret;

	dev_dbg(&dev->client->dev, "%s(0x%02x)\n", __func__, reg);

	ret = i2c_smbus_read_byte_data(dev->client, reg);
	if (ret < 0)
		dev_dbg(&dev->client->dev,
			"%s: register 0x%02x read failed (%d)\n",
			__func__, reg, ret);

	return ret;
}

static int max9271_write(struct rdacm20_device *dev, u8 reg, u8 val)
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

static int ov10635_read16(struct rdacm20_device *dev, u16 reg)
{
	u8 buf[2] = { reg >> 8, reg & 0xff };
	int ret;

	ret = i2c_master_send(dev->sensor, buf, 2);
	if (ret == 2)
		ret = i2c_master_recv(dev->sensor, buf, 2);

	if (ret < 0) {
		dev_dbg(&dev->client->dev,
			"%s: register 0x%04x read failed (%d)\n",
			__func__, reg, ret);
		return ret;
	}

	return (buf[0] << 8) | buf[1];
}

static int ov10635_write(struct rdacm20_device *dev, u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	int ret;

	dev_dbg(&dev->client->dev, "%s(0x%04x, 0x%02x)\n", __func__, reg, val);

	ret = i2c_master_send(dev->sensor, buf, 3);
	if (ret < 0)
		dev_dbg(&dev->client->dev,
			"%s: register 0x%04x write failed (%d)\n",
			__func__, reg, ret);

	return ret < 0 ? ret : 0;
}

static int ov10635_set_regs(struct rdacm20_device *dev,
			    const struct ov10635_reg *regs,
			    unsigned int nr_regs)
{
	unsigned int i;
	int ret;

	for (i = 0; i < nr_regs; i++) {
		ret = ov10635_write(dev, regs[i].reg, regs[i].val);
#if 0 /* Do not stop on write fail .... */
		if (ret)
			return ret;
#endif
	}

	return 0;
}

static int rdacm20_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rdacm20_device *dev = sd_to_rdacm20(sd);

	if (enable) {
		/* Enable the serial link. */
		max9271_write(dev, 0x04, MAX9271_SEREN | MAX9271_REVCCEN |
			      MAX9271_FWDCCEN);
	} else {
		/* Disable the serial link. */
		max9271_write(dev, 0x04, MAX9271_CLINKEN | MAX9271_REVCCEN |
			      MAX9271_FWDCCEN);
	}

	/*
	 * Wait for the link to be established to have access to the control
	 * channel.
	 */
	usleep_range(2000, 5000);

	return 0;
}

static int rdacm20_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2;

	return 0;
}

static int rdacm20_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = OV10635_FORMAT;

	return 0;
}

static int rdacm20_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

	if (format->pad)
		return -EINVAL;

	mf->width	= OV10635_WIDTH;
	mf->height	= OV10635_HEIGHT;
	mf->code	= OV10635_FORMAT;
	mf->colorspace	= V4L2_COLORSPACE_SMPTE170M;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static struct v4l2_subdev_video_ops rdacm20_video_ops = {
	.s_stream	= rdacm20_s_stream,
	.g_mbus_config	= rdacm20_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops rdacm20_subdev_pad_ops = {
	.enum_mbus_code = rdacm20_enum_mbus_code,
	.get_fmt	= rdacm20_get_fmt,
	.set_fmt	= rdacm20_get_fmt,
};

static struct v4l2_subdev_ops rdacm20_subdev_ops = {
	.video		= &rdacm20_video_ops,
	.pad		= &rdacm20_subdev_pad_ops,
};

static int rdacm20_initialize(struct rdacm20_device *dev)
{
	u32 addrs[2];
	int ret;

	ret = of_property_read_u32_array(dev->client->dev.of_node, "reg",
					 addrs, ARRAY_SIZE(addrs));
	if (ret < 0) {
		dev_err(&dev->client->dev, "Invalid DT reg property\n");
		return -EINVAL;
	}

	/*
	 * FIXME: The MAX9271 boots at a default address that we will change to
	 * the address specified in DT. Set the client address back to the
	 * default for initial communication.
	 */
	dev->client->addr = MAX9271_I2C_ADDRESS;

	/* Verify communication with the MAX9271. */
	ret = max9271_read(dev, 0x1e);
	if (ret < 0) {
		dev_err(&dev->client->dev, "MAX9271 ID read failed (%d)\n",
			ret);
		return ret;
	}

	if (ret != MAX9271_ID) {
		dev_err(&dev->client->dev, "MAX9271 ID mismatch (0x%02x)\n",
			ret);
		return -ENXIO;
	}

	/* Change the MAX9271 I2C address. */
	ret = max9271_write(dev, 0x00, addrs[0] << 1);
	if (ret < 0) {
		dev_err(&dev->client->dev,
			"MAX9271 I2C address change failed (%d)\n", ret);
		return ret;
	}
	dev->client->addr = addrs[0];

	/*
	 * Disable the serial link and enable the configuration link to allow
	 * the control channel to operate in a low-speed mode in the absence of
	 * the serial link clock.
	 *
	 * The serializer temporarily disables the reverse control channel for
	 * 350µs after starting/stopping the forward serial link, but the
	 * deserializer synchronization time isn't clearly documented.
	 *
	 * According to the serializer datasheet we should wait 3ms, while
	 * according to the deserializer datasheet we should wait 5ms. In
	 * practice a 2ms delay seems to be enough.
	 */
	max9271_write(dev, 0x04, MAX9271_CLINKEN | MAX9271_REVCCEN |
		      MAX9271_FWDCCEN);
	usleep_range(2000, 5000);

	/* Reset and verify communication with the OV10635. */
#ifdef RDACM20_SENSOR_HARD_RESET
	/* Cycle the OV10635 reset signal connected to the MAX9271 GPIO1. */
	max9271_write(dev, 0x0f, 0xff & ~(MAX9271_GPIO1OUT | MAX9271_SETGPO));
	mdelay(10);
	max9271_write(dev, 0x0f, 0xff & ~MAX9271_SETGPO);
#else
	/* Perform a software reset. */
	ret = ov10635_write(dev, OV10635_SOFTWARE_RESET, 1);
	if (ret < 0) {
		dev_err(&dev->client->dev, "OV10635 reset failed (%d)\n", ret);
		return -ENXIO;
	}

	udelay(100);
#endif

	ret = ov10635_read16(dev, OV10635_PID);
	if (ret < 0) {
		dev_err(&dev->client->dev, "OV10635 ID read failed (%d)\n",
			ret);
		return -ENXIO;
	}

	if (ret != OV10635_VERSION) {
		dev_err(&dev->client->dev, "OV10635 ID mismatch (0x%04x)\n",
			ret);
		return -ENXIO;
	}

	dev_info(&dev->client->dev, "Identified MAX9271 + OV10635 device\n");

	/* Change the sensor I2C address. */
	ret = ov10635_write(dev, OV10635_SC_CMMN_SCCB_ID,
			    (addrs[1] << 1) | OV10635_SC_CMMN_SCCB_ID_SELECT);
	if (ret < 0) {
		dev_err(&dev->client->dev,
			"OV10635 I2C address change failed (%d)\n", ret);
		return ret;
	}
	dev->sensor->addr = addrs[1];

	/* Program the 0V10635 initial configuration. */
	ret = ov10635_set_regs(dev, ov10635_regs_wizard,
			       ARRAY_SIZE(ov10635_regs_wizard));
	if (ret)
		return ret;

	return 0;
}

static int rdacm20_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct rdacm20_device *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->client = client;

	/* Create the dummy I2C client for the sensor. */
	dev->sensor = i2c_new_dummy(client->adapter, OV10635_I2C_ADDRESS);
	if (!dev->sensor) {
		ret = -ENXIO;
		goto error;
	}

	/* Initialize the hardware. */
	ret = rdacm20_initialize(dev);
	if (ret < 0)
		goto error;

	/* Initialize and register the subdevice. */
	v4l2_i2c_subdev_init(&dev->sd, client, &rdacm20_subdev_ops);
	dev->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	v4l2_ctrl_handler_init(&dev->ctrls, 1);
	/*
	 * FIXME: Compute the real pixel rate. The 50 MP/s value comes from the
	 * hardcoded frequency in the BSP CSI-2 receiver driver.
	 */
	v4l2_ctrl_new_std(&dev->ctrls, NULL, V4L2_CID_PIXEL_RATE, 50000000,
			  50000000, 1, 50000000);
	dev->sd.ctrl_handler = &dev->ctrls;

	ret = dev->ctrls.error;
	if (ret)
		goto error;

	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.flags |= MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&dev->sd.entity, 1, &dev->pad);
	if (ret < 0)
		goto error;

	ret = v4l2_async_register_subdev(&dev->sd);
	if (ret)
		goto error;

	return 0;

error:
	media_entity_cleanup(&dev->sd.entity);
	if (dev->sensor)
		i2c_unregister_device(dev->sensor);
	kfree(dev);

	dev_err(&client->dev, "failed to probe @ 0x%02x (%s)\n",
		client->addr, client->adapter->name);

	return ret;
}

static int rdacm20_remove(struct i2c_client *client)
{
	struct rdacm20_device *dev = i2c_to_rdacm20(client);

	v4l2_async_unregister_subdev(&dev->sd);
	media_entity_cleanup(&dev->sd.entity);
	i2c_unregister_device(dev->sensor);
	kfree(dev);

	return 0;
}

static void rdacm20_shutdown(struct i2c_client *client)
{
	struct rdacm20_device *dev = i2c_to_rdacm20(client);

	/* make sure stream off during shutdown (reset/reboot) */
	rdacm20_s_stream(&dev->sd, 0);
}

static const struct i2c_device_id rdacm20_id[] = {
	{ "rdacm20", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rdacm20_id);

static const struct of_device_id rdacm20_of_ids[] = {
	{ .compatible = "imi,rdacm20", },
	{ }
};
MODULE_DEVICE_TABLE(of, rdacm20_of_ids);

static struct i2c_driver rdacm20_i2c_driver = {
	.driver	= {
		.name	= "rdacm20",
		.of_match_table = rdacm20_of_ids,
	},
	.probe		= rdacm20_probe,
	.remove		= rdacm20_remove,
	.shutdown	= rdacm20_shutdown,
	.id_table	= rdacm20_id,
};

module_i2c_driver(rdacm20_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for MAX9286<->MAX9271<->OV10635");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
