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
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define MAX9286_MAX_PORTS	4
#define MAX9286_N_PADS		5
#define MAX9286_SRC_PAD		4
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
	struct v4l2_subdev sd;
	struct media_pad pads[MAX9286_N_PADS];
	struct regulator *regulator;
	struct i2c_mux_core *mux;
	unsigned int mux_channel;
	unsigned int nports;

	struct v4l2_ctrl_handler	ctrls;

	struct v4l2_async_notifier notifier;

	unsigned int nserializers;
	struct serializer {
		struct v4l2_async_subdev asd;
		struct v4l2_subdev *sd;
		struct fwnode_handle *fwnode;
		unsigned int src_pad;
	} serializers[MAX9286_MAX_PORTS];
};

static inline struct max9286_device *sd_to_max9286(struct v4l2_subdev *sd)
{
	return container_of(sd, struct max9286_device, sd);
}

static inline struct max9286_device *notifier_to_max9286(
		struct v4l2_async_notifier *notifier)
{
	return container_of(notifier, struct max9286_device, notifier);
}

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

	if (dev->mux_channel == chan)
		return 0;

	dev->mux_channel = chan;
	max9286_write(dev, 0x0a, 0xf0 | (0x01 << chan));

	return 0;
}

static int max9286_i2c_mux_init(struct max9286_device *dev)
{
	unsigned int i;
	int ret;

	if (!i2c_check_functionality(dev->client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -ENODEV;

	dev->mux = i2c_mux_alloc(dev->client->adapter, &dev->client->dev,
				 dev->nports, 0, I2C_MUX_LOCKED,
				 max9286_i2c_mux_select, NULL);
	dev_info(&dev->client->dev, "%s: mux %p\n", __func__, dev->mux);
	if (!dev->mux)
		return -ENOMEM;

	dev->mux->priv = dev;

	for (i = 0; i < dev->nports; ++i) {
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
 * v4l2 subdev
 */
static int max9286_notify_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct max9286_device *dev = notifier_to_max9286(notifier);
	struct serializer *serializer = &dev->serializers[dev->nserializers++];
	int ret;

	v4l2_set_subdev_hostdata(subdev, dev);

	ret = media_entity_get_fwnode_pad(&subdev->entity,
					  serializer->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(&dev->client->dev,
			"Failed to find pad for %s\n", subdev->name);
		return ret;
	}

	serializer->sd = subdev;
	serializer->src_pad = ret;
	dev_dbg(&dev->client->dev, "Bound %s pad: %d\n", subdev->name, ret);

	return 0;
}

static void max9286_notify_unbind(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_subdev *asd)
{
	struct max9286_device *dev = notifier_to_max9286(notifier);
	unsigned int i;

	for (i = 0; i < dev->nports; i++)
		if (dev->serializers[i].sd == subdev) {
			dev->serializers[i].sd = NULL;
			return;
		}
}

static int max9286_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct max9286_device *dev = notifier_to_max9286(notifier);
	unsigned int i;
	int ret;

	for (i = 0; i < dev->nports; i++) {
		ret = media_create_pad_link(&dev->serializers[i].sd->entity,
					    dev->serializers[i].src_pad,
					    &dev->sd.entity, i,
					    MEDIA_LNK_FL_ENABLED |
					    MEDIA_LNK_FL_IMMUTABLE);
		if (ret) {
			dev_err(&dev->client->dev,
				"Unable to link %s:%u -> %s:%u\n",
				dev->serializers[i].sd->name,
				dev->serializers[i].src_pad,
				dev->sd.name, i);
			return ret;
		}
	}

	return v4l2_device_register_subdev_nodes(dev->sd.v4l2_dev);

	return 0;
}

static int max9286_registered(struct v4l2_subdev *sd)
{
	struct max9286_device *dev = sd_to_max9286(sd);
	struct v4l2_async_subdev **subdevs;
	unsigned int i;
	int ret;

	subdevs = devm_kcalloc(&dev->client->dev,
			       dev->nports, sizeof(*subdevs), GFP_KERNEL);
	if (!subdevs)
		return -ENOMEM;

	dev_dbg(&dev->client->dev,
		"%s: Claim %d serializer subdevices for 0x%02x subnotifier\n",
		__func__, dev->nports, dev->client->addr);

	memset(&dev->serializers[0], 0, sizeof(dev->serializers));
	for (i = 0; i < dev->nports; i++)
		subdevs[i] = &dev->serializers[i].asd;

	dev->notifier.num_subdevs = dev->nports;
	dev->notifier.subdevs = subdevs;
	dev->notifier.bound = max9286_notify_bound;
	dev->notifier.unbind = max9286_notify_unbind;
	dev->notifier.complete = max9286_notify_complete;

	ret = v4l2_async_subnotifier_register(&dev->sd, &dev->notifier);
	if (ret)
		return ret;

	return 0;
}

static void max9286_unregistered(struct v4l2_subdev *sd)
{
	struct max9286_device *dev = sd_to_max9286(sd);

	v4l2_async_subnotifier_unregister(&dev->notifier);
}

static const struct v4l2_subdev_internal_ops max9286_subdev_internal_ops  = {
	.registered	= max9286_registered,
	.unregistered	= max9286_unregistered,
};

static int max9286_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2;

	return 0;
}

static int max9286_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct max9286_device *dev = sd_to_max9286(sd);
	struct serializer *serializer;
	unsigned int i;
	int ret;

	if (enable) {
		/*
		 * Enable CSI output, VC set according to link number.
		 * Bit 7 must be set (chip manual says it's 0 and reserved)
		 */
		max9286_write(dev, 0x15, 0x9b);

		for (i = 0; i < dev->nports; i++) {
			serializer = &dev->serializers[i];
			ret = v4l2_subdev_call(serializer->sd, video,
					       s_stream, 1);
			if (ret)
				goto err_stop_stream;
		}
	} else {
		max9286_write(dev, 0x15, 0x13);
		for (i = 0; i < dev->nports; i++) {
			serializer = &dev->serializers[i];
			v4l2_subdev_call(serializer->sd, video, s_stream, 0);
		}
	}

	return 0;

err_stop_stream:
	for (; i > 0; i--) {
		serializer = &dev->serializers[i - 1];
		v4l2_subdev_call(serializer->sd, video, s_stream, 0);
	}

	return ret;
}

static int max9286_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_2X8;

	return 0;
}

static int max9286_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

	mf->width	= 1280;
	mf->height	= 800;
	mf->code	= MEDIA_BUS_FMT_UYVY8_2X8;
	mf->colorspace	= V4L2_COLORSPACE_SMPTE170M;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static const struct v4l2_subdev_video_ops max9286_video_ops = {
	.s_stream	= max9286_s_stream,
	.g_mbus_config	= max9286_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops max9286_pad_ops = {
	.enum_mbus_code = max9286_enum_mbus_code,
	.get_fmt	= max9286_get_fmt,
	.set_fmt	= max9286_get_fmt,
};

static struct v4l2_subdev_ops max9286_subdev_ops = {
	.video		= &max9286_video_ops,
	.pad		= &max9286_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Probe/Remove
 */

static int max9286_setup(struct max9286_device *dev)
{
	unsigned short des_addr = dev->client->addr;
	unsigned int cam_idx, cam_offset;
	unsigned int linken_mask = 0xf & ((1 << dev->nports) - 1);

	switch (des_addr) {
	case DES0:
		cam_offset = 0;
		break;
	case DES1:
		cam_offset = 4;
		break;
	default:
		return -EINVAL;
	}

	for (cam_idx = cam_offset; cam_idx < dev->nports + cam_offset; cam_idx++) {
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

	/*
	 * Enable GMSL links, mask unused ones and autodetect link
	 * used as CSI clock source. Enable automask and autocomeback
	 * to protect against malfunctioning links.
	 *
	 * FIXME/TODO: Double check to see if we want to enable autocomeback and
	 * automask
	 */
	max9286_write(dev, 0x00, 0xe0 | linken_mask);
	max9286_write(dev, 0x69, 0x30 | (0xf & ~linken_mask));

	/* Video format setup */
	/* Disable CSI output, VC is set accordingly to Link number */
	max9286_write(dev, 0x15, 0x13);

	/*
	 * FIXME: once this driver will have an endpoint, retrieve
	 * CSI lanes number from there, and set image format properly.
	 * For now, it stays hardcoded to 1 lane only to comply with
	 * current VIN settings.
	 */
	/* Enable CSI-2 Lane D0 only, DBL mode, YUV422 8-bit*/
	max9286_write(dev, 0x12, 0x33);

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

	if (dev->nports == 1)
		/* ECU (aka MCU) based FrameSync using GPI-to-GPO */
		max9286_write(dev, 0x01, 0xc0);
	else
		/* Automatic: FRAMESYNC taken from the slowest Link */
		max9286_write(dev, 0x01, 0x02);

	/* Enable HS/VS encoding, use D14/15 for HS/VS, invert VS */
	max9286_write(dev, 0x0c, 0x89);

	return 0;
}

static const struct of_device_id max9286_dt_ids[] = {
	{ .compatible = "maxim,max9286" },
	{},
};
MODULE_DEVICE_TABLE(of, max9286_dt_ids);

static int max9286_init(struct device *dev, void *data)
{
	struct max9286_device *max9286_dev;
	struct i2c_client *client;
	unsigned int i;
	int ret;

	if (!dev->of_node ||
	    !of_match_node(max9286_dt_ids, dev->of_node))
		/* skip non-max9286 devices */
		return 0;

	client = to_i2c_client(dev);
	max9286_dev = i2c_get_clientdata(client);

	/* -1 as one of the available children is "ports" node */
	max9286_dev->nports = of_get_available_child_count(dev->of_node) - 1;
	if (!max9286_dev->nports)
		return 0;

	ret = max9286_setup(max9286_dev);
	if (ret) {
		dev_err(dev, "Unable to setup max9286 0x%x\n",
			client->addr);
		return ret;
	}

	v4l2_i2c_subdev_init(&max9286_dev->sd, client, &max9286_subdev_ops);
	max9286_dev->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	v4l2_ctrl_handler_init(&max9286_dev->ctrls, 1);
	/*
	 * FIXME: Compute the real pixel rate. The 50 MP/s value comes from the
	 * hardcoded frequency in the BSP CSI-2 receiver driver.
	 */
	v4l2_ctrl_new_std(&max9286_dev->ctrls, NULL, V4L2_CID_PIXEL_RATE,
			  50000000, 50000000, 1, 50000000);
	max9286_dev->sd.ctrl_handler = &max9286_dev->ctrls;
	ret = max9286_dev->ctrls.error;
	if (ret)
		return ret;

	max9286_dev->sd.internal_ops = &max9286_subdev_internal_ops;
	max9286_dev->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;

	max9286_dev->pads[MAX9286_SRC_PAD].flags = MEDIA_PAD_FL_SOURCE;
	for (i = 0; i < MAX9286_SRC_PAD; i++)
		max9286_dev->pads[i].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&max9286_dev->sd.entity, MAX9286_N_PADS,
				     max9286_dev->pads);
	if (ret)
		return ret;

	ret = v4l2_async_register_subdev(&max9286_dev->sd);
	if (ret < 0) {
		dev_err(dev, "Unable to register subdevice max9286 0x%02x\n",
			client->addr);
		return ret;
	}

	ret = max9286_i2c_mux_init(max9286_dev);
	if (ret) {
		dev_err(dev, "Unable to initialize mux channels max9286 0x%x\n",
			client->addr);
		goto err_subdev_unregister;
	}

	return 0;

err_subdev_unregister:
	v4l2_async_unregister_subdev(&max9286_dev->sd);

	return ret;
}

static int max9286_is_bound(struct device *dev, void *data)
{
	struct device *this = data;
	int ret;

	if (dev == this)
		return 0;

	if (!dev->of_node ||
	    !of_match_node(max9286_dt_ids, dev->of_node))
		/* skip non-max9286 devices */
		return 0;

	ret = device_is_bound(dev);
	if (!ret)
		return -EPROBE_DEFER;

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

	/*
	 * We can have multiple MAX9286 instances on the same physical I2C
	 * bus, and I2C children behind ports of separate MAX9286 instances
	 * having the same I2C address. As the MAX9286 starts by default with
	 * all ports enabled, we need to disable all ports on all MAX9286
	 * instances before proceeding to further initialize the devices and
	 * instantiate children.
	 *
	 * Start by just disabling all channels on the current device. Then,
	 * if all other MAX9286 on the parent bus have been probed, proceed
	 * to initialize them all, including the current one.
	 */
	dev->mux_channel = MAX9286_MAX_PORTS;
	dev->nserializers = 0;
	max9286_write(dev, 0x0a, 0x00);
	ret = device_for_each_child(client->dev.parent, &client->dev,
				    max9286_is_bound);
	if (ret)
		return 0;

	dev_dbg(&client->dev,
		"All max9286 probed: start initialization sequence\n");
	return device_for_each_child(client->dev.parent, NULL,
				     max9286_init);

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

	v4l2_async_unregister_subdev(&dev->sd);

	regulator_disable(dev->regulator);
	regulator_put(dev->regulator);

	kfree(dev);

	return 0;
}

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
