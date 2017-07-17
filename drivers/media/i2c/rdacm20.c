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

#define OV10635_PID			0x300a
#define OV10635_VER			0x300b
#define OV10635_VERSION_REG		0xa635
#define OV10635_VERSION(pid, ver)	(((pid) << 8) | ((ver) & 0xff))

#define OV10635_WIDTH			1280
#define OV10635_HEIGHT			800
#define OV10635_FORMAT			MEDIA_BUS_FMT_UYVY8_2X8
/* #define OV10635_FORMAT			MEDIA_BUS_FMT_UYVY10_2X10 */

struct rdacm20_device {
	struct i2c_client		*client;
	struct v4l2_subdev		sd;
	struct media_pad		pad;
	struct v4l2_ctrl_handler	ctrls;
};

/*
 * I2C MAP.
 *
 *		CAM0	CAM1	CAM2	CAM3
 * MAX9286	0x48+1	0x48+2	0x48+3	0x48+4	- deserializer
 * MAX9271	0x40+1	0x40+2	0x40+3	0x40+4	- serializer
 * OV10635	0x30+1	0x30+2	0x30+3	0x30+4	- sensor
 */

#define DES0		0x4c
#define DES1		0x6c
#define SER		0x51
#define CAM		0x60

static const u8 maxim_map[][8] = {
	{ DES0 + 0, DES0 + 0, DES0 + 0, DES0 + 0,
	  DES1 + 0, DES1 + 0, DES1 + 0, DES1 + 0 },
	{ SER  + 0, SER  + 1, SER  + 2, SER  + 3,
	  SER  + 4, SER  + 5, SER  + 6, SER  + 7 },
};

static inline struct rdacm20_device *sd_to_rdacm20(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rdacm20_device, sd);
}

static inline struct rdacm20_device *i2c_to_rdacm20(struct i2c_client *client)
{
	return sd_to_rdacm20(i2c_get_clientdata(client));
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

static int ov10635_read(struct rdacm20_device *dev, u16 reg, u8 *val)
{
	u8 buf[2] = { reg >> 8, reg & 0xff };
	int ret;

	ret = i2c_master_send(dev->client, buf, 2);
	if (ret == 2)
		ret = i2c_master_recv(dev->client, buf, 1);

	if (ret < 0) {
		dev_dbg(&dev->client->dev,
			"%s: register 0x%04x read failed (%d)\n",
			__func__, reg, ret);
		return ret;
	}

	*val = buf[0];
	return 0;
}

static int ov10635_write(struct rdacm20_device *dev, u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	int ret;

	dev_dbg(&dev->client->dev, "%s(0x%04x, 0x%02x)\n", __func__, reg, val);

	ret = i2c_master_send(dev->client, buf, 3);
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
	int cam_idx = dev->client->addr - CAM;
	int tmp_addr;

	if (!(dev->client->addr == (CAM + 0) || dev->client->addr == (CAM + 4)))
		return 0;

	/* switch to GMSL conf_link to access sensor registers */
	tmp_addr = dev->client->addr;
	dev->client->addr = maxim_map[0][cam_idx];	/* MAX9286-CAMx */
	max9271_write(dev, 0x15, 0x93);
		/* disable CSI output,
		 *	VC is set accordingly to Link number,
		 *	BIT7 magic must be set
		 */
#if 0
	max9271_write(dev, 0x0a, 0xff);
				/* enable reverse_control/conf_link */
#endif
	mdelay(5);
				/* wait 5ms for conf_link to establish */
	dev->client->addr = maxim_map[1][cam_idx];	/* MAX9271-CAMx */
#if 0
	max9271_write(dev, 0x04, 0x43);
				/* enable reverse_control/conf_link */
#endif
	mdelay(5);		/* wait 5ms for conf_link to establish */
	dev->client->addr = tmp_addr;
#if 0
	ov10635_write(dev, 0x0100, enable); /* stream on/off */
#endif
	if (enable) {
		/* switch to GMSL serial_link for streaming video */
		tmp_addr = dev->client->addr;
		dev->client->addr = maxim_map[1][cam_idx];	/* MAX9271-CAMx */
		max9271_write(dev, 0x04, 0x83);
				/* enable reverse_control/serial_link */
		mdelay(2);
				/* wait 2ms after changing reverse_control */
		dev->client->addr = maxim_map[0][cam_idx];	/* MAX9286-CAMx */
#if 0
		max9271_write(dev, 0x0a, 0xf0);
			/* disable reverse_control, enable serial_link */
#endif
		max9271_write(dev, 0x15, 0x9b);
			/* enable CSI output,
			 *  VC is set accordingly to Link number,
			 *  BIT7 magic must be set
			 */
		mdelay(5);
			/* wait 2ms after changing reverse_control */
		dev->client->addr = tmp_addr;
	}

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
	int tmp_addr, cam_idx;
	u8 pid, ver;
	int ret;

	/* check and show product ID and manufacturer ID */
	ret = ov10635_read(dev, OV10635_PID, &pid);
	if (ret)
		return ret;
	ret = ov10635_read(dev, OV10635_VER, &ver);
	if (ret)
		return ret;

	if (OV10635_VERSION(pid, ver) != OV10635_VERSION_REG) {
		dev_err(&dev->client->dev, "Product ID error %x:%x\n", pid, ver);
		return -ENODEV;
	}

	dev_info(&dev->client->dev, "ov10635 Product ID %x Manufacturer ID %x\n",
		 pid, ver);

#if !defined(MAXIM_IMI_MCU_POWERED)
#if 0
	/* IMI camera has GPIO1 routed to OV10635 reset pin */
	dev->client->addr = maxim_map[1][cam_idx];	/* OV10635-CAMx I2C new */
	max9271_write(dev, 0x0f, 0xfc);
					/* GPIO1 low, ov10635 in reset */
	mdelay(10);
	max9271_write(dev, 0x0f, 0xfe);
				/* GPIO1 high, ov10635 out from reset */
#else
	/* s/w reset sensor */
	ov10635_write(dev, 0x103, 0x1);
	udelay(100);
#endif

	/* Program wizard registers */
	ret = ov10635_set_regs(dev, ov10635_regs_wizard,
			       ARRAY_SIZE(ov10635_regs_wizard));
	if (ret)
		return ret;
#endif

	cam_idx = dev->client->addr - CAM;
	/* switch to GMSL serial_link for streaming video */
	tmp_addr = dev->client->addr;
	dev->client->addr = maxim_map[1][cam_idx];	/* MAX9271-CAMx */
	max9271_write(dev, 0x04, 0x83);
			/* enable reverse_control/serial_link */
	mdelay(2);	/* wait 2ms after changing reverse_control */
	dev->client->addr = tmp_addr;

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
		goto cleanup;

	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.flags |= MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&dev->sd.entity, 1, &dev->pad);
	if (ret < 0)
		goto cleanup;

	ret = rdacm20_initialize(dev);
	if (ret < 0)
		goto cleanup;

	ret = v4l2_async_register_subdev(&dev->sd);
	if (ret)
		goto cleanup;

	return 0;

cleanup:
	media_entity_cleanup(&dev->sd.entity);
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
