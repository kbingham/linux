// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim MAX9286 GMSL Deserializer Driver
 *
 * Copyright (C) 2017-2018 Jacopo Mondi
 * Copyright (C) 2017-2018 Kieran Bingham
 * Copyright (C) 2017-2018 Laurent Pinchart
 * Copyright (C) 2017-2018 Niklas Söderlund
 * Copyright (C) 2016 Renesas Electronics Corporation
 * Copyright (C) 2015 Cogent Embedded, Inc.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fwnode.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

/* Register 0x00 */
#define MAX9286_MSTLINKSEL_AUTO		(7 << 5)
#define MAX9286_MSTLINKSEL(n)		((n) << 5)
#define MAX9286_EN_VS_GEN		BIT(4)
#define MAX9286_LINKEN(n)		(1 << (n))
/* Register 0x01 */
#define MAX9286_FSYNCMODE_ECU		(3 << 6)
#define MAX9286_FSYNCMODE_EXT		(2 << 6)
#define MAX9286_FSYNCMODE_INT_OUT	(1 << 6)
#define MAX9286_FSYNCMODE_INT_HIZ	(0 << 6)
#define MAX9286_GPIEN			BIT(5)
#define MAX9286_ENLMO_RSTFSYNC		BIT(2)
#define MAX9286_FSYNCMETH_AUTO		(2 << 0)
#define MAX9286_FSYNCMETH_SEMI_AUTO	(1 << 0)
#define MAX9286_FSYNCMETH_MANUAL	(0 << 0)
#define MAX9286_REG_FSYNC_PERIOD_L	0x06
#define MAX9286_REG_FSYNC_PERIOD_M	0x07
#define MAX9286_REG_FSYNC_PERIOD_H	0x08
/* Register 0x0a */
#define MAX9286_FWDCCEN(n)		(1 << ((n) + 4))
#define MAX9286_REVCCEN(n)		(1 << (n))
/* Register 0x0c */
#define MAX9286_HVEN			BIT(7)
#define MAX9286_EDC_6BIT_HAMMING	(2 << 5)
#define MAX9286_EDC_6BIT_CRC		(1 << 5)
#define MAX9286_EDC_1BIT_PARITY		(0 << 5)
#define MAX9286_DESEL			BIT(4)
#define MAX9286_INVVS			BIT(3)
#define MAX9286_INVHS			BIT(2)
#define MAX9286_HVSRC_D0		(2 << 0)
#define MAX9286_HVSRC_D14		(1 << 0)
#define MAX9286_HVSRC_D18		(0 << 0)
/* Register 0x12 */
#define MAX9286_CSILANECNT(n)		(((n) - 1) << 6)
#define MAX9286_CSIDBL			BIT(5)
#define MAX9286_DBL			BIT(4)
#define MAX9286_DATATYPE_USER_8BIT	(11 << 0)
#define MAX9286_DATATYPE_USER_YUV_12BIT	(10 << 0)
#define MAX9286_DATATYPE_USER_24BIT	(9 << 0)
#define MAX9286_DATATYPE_RAW14		(8 << 0)
#define MAX9286_DATATYPE_RAW11		(7 << 0)
#define MAX9286_DATATYPE_RAW10		(6 << 0)
#define MAX9286_DATATYPE_RAW8		(5 << 0)
#define MAX9286_DATATYPE_YUV422_10BIT	(4 << 0)
#define MAX9286_DATATYPE_YUV422_8BIT	(3 << 0)
#define MAX9286_DATATYPE_RGB555		(2 << 0)
#define MAX9286_DATATYPE_RGB565		(1 << 0)
#define MAX9286_DATATYPE_RGB888		(0 << 0)
/* Register 0x15 */
#define MAX9286_VC(n)			((n) << 5)
#define MAX9286_VCTYPE			BIT(4)
#define MAX9286_CSIOUTEN		BIT(3)
#define MAX9286_0X15_RESV		(3 << 0)
/* Register 0x1b */
#define MAX9286_SWITCHIN(n)		(1 << ((n) + 4))
#define MAX9286_ENEQ(n)			(1 << (n))
/* Register 0x31 */
#define MAX9286_FSYNC_LOCKED		BIT(6)
/* Register 0x34 */
#define MAX9286_I2CLOCACK		BIT(7)
#define MAX9286_I2CSLVSH_1046NS_469NS	(3 << 5)
#define MAX9286_I2CSLVSH_938NS_352NS	(2 << 5)
#define MAX9286_I2CSLVSH_469NS_234NS	(1 << 5)
#define MAX9286_I2CSLVSH_352NS_117NS	(0 << 5)
#define MAX9286_I2CMSTBT_837KBPS	(7 << 2)
#define MAX9286_I2CMSTBT_533KBPS	(6 << 2)
#define MAX9286_I2CMSTBT_339KBPS	(5 << 2)
#define MAX9286_I2CMSTBT_173KBPS	(4 << 2)
#define MAX9286_I2CMSTBT_105KBPS	(3 << 2)
#define MAX9286_I2CMSTBT_84KBPS		(2 << 2)
#define MAX9286_I2CMSTBT_28KBPS		(1 << 2)
#define MAX9286_I2CMSTBT_8KBPS		(0 << 2)
#define MAX9286_I2CSLVTO_NONE		(3 << 0)
#define MAX9286_I2CSLVTO_1024US		(2 << 0)
#define MAX9286_I2CSLVTO_256US		(1 << 0)
#define MAX9286_I2CSLVTO_64US		(0 << 0)
/* Register 0x3b */
#define MAX9286_REV_TRF(n)		((n) << 5)
#define MAX9286_REV_AMP(n)		((((n) - 30) / 10) << 1) /* in mV */
#define MAX9286_REV_AMP_X		BIT(0)
/* Register 0x3f */
#define MAX9286_EN_REV_CFG		BIT(6)
#define MAX9286_REV_FLEN(n)		((n) - 20)
/* Register 0x69 */
#define MAX9286_LFLTBMONMASKED		BIT(7)
#define MAX9286_LOCKMONMASKED		BIT(6)
#define MAX9286_AUTOCOMBACKEN		BIT(5)
#define MAX9286_AUTOMASKEN		BIT(4)
#define MAX9286_MASKLINK(n)		((n) << 0)

#define MAX9286_NUM_GMSL		4
#define MAX9286_N_PADS			5
#define MAX9286_SRC_PAD			4

#define MAXIM_I2C_I2C_SPEED_400KHZ	MAX9286_I2CMSTBT_339KBPS
#define MAXIM_I2C_I2C_SPEED_100KHZ	MAX9286_I2CMSTBT_105KBPS
#define MAXIM_I2C_SPEED			MAXIM_I2C_I2C_SPEED_100KHZ

struct max9286_source {
	struct v4l2_async_subdev asd;
	struct v4l2_subdev *sd;
	struct fwnode_handle *fwnode;
};

#define asd_to_max9286_source(_asd) \
	container_of(_asd, struct max9286_source, asd)

struct max9286_device {
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pads[MAX9286_N_PADS];
	struct regulator *regulator;
	struct dentry *dbgroot;
	bool poc_enabled;
	int streaming;

	struct i2c_mux_core *mux;
	unsigned int mux_channel;

	struct v4l2_ctrl_handler ctrls;

	unsigned int nsources;
	unsigned int source_mask;
	unsigned int route_mask;
	struct max9286_source sources[MAX9286_NUM_GMSL];
	struct v4l2_async_subdev *subdevs[MAX9286_NUM_GMSL];
	struct v4l2_async_notifier notifier;
};

static struct max9286_source *next_source(struct max9286_device *max9286,
					  struct max9286_source *source)
{
	if (!source)
		source = &max9286->sources[0];
	else
		source++;

	for (; source < &max9286->sources[MAX9286_NUM_GMSL]; source++) {
		if (source->fwnode)
			return source;
	}

	return NULL;
}

#define for_each_source(max9286, source) \
	for (source = NULL; (source = next_source(max9286, source)); )

#define to_index(max9286, source) (source - &max9286->sources[0])

#define sd_to_max9286(_sd) container_of(_sd, struct max9286_device, sd)

/* -----------------------------------------------------------------------------
 * I2C IO
 */

static int max9286_read(struct max9286_device *dev, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(dev->client, reg);
	if (ret < 0)
		dev_err(&dev->client->dev,
			"%s: register 0x%02x read failed (%d)\n",
			__func__, reg, ret);

	return ret;
}

static int max9286_write(struct max9286_device *dev, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(dev->client, reg, val);
	if (ret < 0)
		dev_err(&dev->client->dev,
			"%s: register 0x%02x write failed (%d)\n",
			__func__, reg, ret);

	return ret;
}

/* -----------------------------------------------------------------------------
 * DebugFS
 */

#define DEBUGFS_RO_ATTR(name)						\
static int name##_open(struct inode *inode, struct file *file)		\
{									\
	return single_open(file, name, inode->i_private);		\
}									\
static const struct file_operations name##_fops = {			\
	.owner = THIS_MODULE,						\
	.open = name##_open,						\
	.llseek = seq_lseek,						\
	.read = seq_read,						\
	.release = single_release,					\
}

static int max9286_config_video_detect(struct max9286_device *dev,
				       struct seq_file *s)
{
	int reg_49 = max9286_read(dev, 0x49);
	unsigned int i;

	if (reg_49 < 0)
		return reg_49;

	seq_puts(s, "                  :  0   1   2   3\n");
	seq_puts(s, "Configuration Link:");
	for (i = 0; i < 4; i++) {
		int link = (reg_49 & BIT(i + 4));

		seq_printf(s, " %3s", link ? " O " : "xxx");
	}
	seq_puts(s, "\n");

	seq_puts(s, "Video Link        :");
	for (i = 0; i < 4; i++) {
		int link = (reg_49 & BIT(i));

		seq_printf(s, " %3s", link ? " O " : "xxx");
	}
	seq_puts(s, "\n");

	return 0;
}

static int max9286_vs_period(struct max9286_device *dev, struct seq_file *s)
{
	int l, m, h;
	unsigned int frame_length;

	l = max9286_read(dev, 0x5b);
	m = max9286_read(dev, 0x5c);
	h = max9286_read(dev, 0x5d);

	if (l < 0 || m < 0 || h << 0)
		return -ENODEV;

	frame_length = (h << 16) + (m << 8) + l;

	seq_printf(s, "Calculated VS Period (pxclk) : %u\n", frame_length);

	return 0;
}

static int max9286_master_link(struct max9286_device *dev, struct seq_file *s)
{
	int reg_71 = max9286_read(dev, 0x71);
	unsigned int link = (reg_71 >> 4) & 0x03;

	if (reg_71 < 0)
		return reg_71;

	seq_printf(s, "Master link selected : %u\n", link);

	return 0;
}

static int max9286_debugfs_info(struct seq_file *s, void *p)
{
	struct max9286_device *dev = s->private;

	max9286_config_video_detect(dev, s);
	max9286_vs_period(dev, s);
	max9286_master_link(dev, s);

	return 0;
}

DEBUGFS_RO_ATTR(max9286_debugfs_info);

static int max9286_debugfs_init(struct max9286_device *dev)
{
	char name[32];

	snprintf(name, sizeof(name), "max9286-%s", dev_name(&dev->client->dev));

	dev->dbgroot = debugfs_create_dir(name, NULL);
	if (!dev->dbgroot)
		return -ENOMEM;

	/*
	 * dentry pointers are discarded, and remove_recursive is used to
	 * cleanup the tree. DEBUGFS_RO_ATTR defines the file operations with
	 * the _fops extension to the function name
	 */
	debugfs_create_file("info", 0444, dev->dbgroot, dev,
			    &max9286_debugfs_info_fops);

	return 0;
}

static void max9286_debugfs_remove(struct max9286_device *dev)
{
	debugfs_remove_recursive(dev->dbgroot);
}

/* -----------------------------------------------------------------------------
 * I2C Multiplexer
 */

static int max9286_i2c_mux_close(struct max9286_device *dev)
{

	/* FIXME: See note in max9286_i2c_mux_select() */
	if (dev->streaming)
		return 0;
	/*
	 * Ensure that both the forward and reverse channel are disabled on the
	 * mux, and that the channel ID is invalidated to ensure we reconfigure
	 * on the next select call.
	 */
	dev->mux_channel = -1;
	max9286_write(dev, 0x0a, 0x00);
	usleep_range(3000, 5000);

	return 0;
}

static int max9286_i2c_mux_select(struct i2c_mux_core *muxc, u32 chan)
{
	struct max9286_device *dev = i2c_mux_priv(muxc);

	/*
	 * FIXME: This state keeping is a hack and do the job. It should
	 * be should be reworked. One option to consider is that once all
	 * cameras are programmed the mux selection logic should be disabled
	 * and all all reverse and forward channels enable all the time.
	 *
	 * In any case this logic with a int that have two states should be
	 * reworked!
	 */
	if (dev->streaming == 1) {
		max9286_write(dev, 0x0a, 0xff);
		dev->streaming = 2;
		return 0;
	} else if (dev->streaming == 2) {
		return 0;
	}

	if (dev->mux_channel == chan)
		return 0;

	dev->mux_channel = chan;

	max9286_write(dev, 0x0a, MAX9286_FWDCCEN(chan) | MAX9286_REVCCEN(chan));

	/*
	 * We must sleep after any change to the forward or reverse channel
	 * configuration
	 */
	usleep_range(3000, 5000);

	return 0;
}

static int max9286_i2c_mux_init(struct max9286_device *dev)
{
	struct max9286_source *source;
	int ret;

	if (!i2c_check_functionality(dev->client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -ENODEV;

	dev->mux = i2c_mux_alloc(dev->client->adapter, &dev->client->dev,
				 dev->nsources, 0, I2C_MUX_LOCKED,
				 max9286_i2c_mux_select, NULL);
	if (!dev->mux)
		return -ENOMEM;

	dev->mux->priv = dev;

	for_each_source(dev, source) {
		unsigned int index = to_index(dev, source);

		ret = i2c_mux_add_adapter(dev->mux, 0, index, 0);
		if (ret < 0)
			goto error;
	}

	return 0;

error:
	i2c_mux_del_adapters(dev->mux);
	return ret;
}

static void max9286_configure_i2c(struct max9286_device *dev, bool localack)
{
	u8 config = MAX9286_I2CSLVSH_469NS_234NS | MAX9286_I2CSLVTO_1024US |
		    MAXIM_I2C_SPEED;

	if (localack)
		config |= MAX9286_I2CLOCACK;

	max9286_write(dev, 0x34, config);
	usleep_range(3000, 5000);
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev
 */

static int max9286_notify_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct max9286_device *dev = sd_to_max9286(notifier->sd);
	struct max9286_source *source = asd_to_max9286_source(asd);
	unsigned int index = to_index(dev, source);
	unsigned int src_pad;
	int ret;

	ret = media_entity_get_fwnode_pad(&subdev->entity,
					  source->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(&dev->client->dev,
			"Failed to find pad for %s\n", subdev->name);
		return ret;
	}

	source->sd = subdev;
	src_pad = ret;

	ret = media_create_pad_link(&source->sd->entity, src_pad,
				    &dev->sd.entity, index,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(&dev->client->dev,
			"Unable to link %s:%u -> %s:%u\n",
			source->sd->name, src_pad, dev->sd.name, index);
		return ret;
	}

	dev_dbg(&dev->client->dev, "Bound %s pad: %u on index %u\n",
		subdev->name, src_pad, index);

	return 0;
}

static void max9286_notify_unbind(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_subdev *asd)
{
	struct max9286_source *source = asd_to_max9286_source(asd);

	source->sd = NULL;
}

static const struct v4l2_async_notifier_operations max9286_notify_ops = {
	.bound = max9286_notify_bound,
	.unbind = max9286_notify_unbind,
};

static int max9286_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct max9286_device *dev = sd_to_max9286(sd);
	struct max9286_source *source;
	unsigned int i;
	bool sync = false;
	int ret;

	if (enable) {
		/* FIXME: See note in max9286_i2c_mux_select() */
		dev->streaming = 1;

		/* Start all cameras */
		for_each_source(dev, source) {
			ret = v4l2_subdev_call(source->sd, video, s_stream, 1);
			if (ret)
				return ret;
		}

		/* Wait until frame synchronization is locked */
		for (i = 0; i < 100; i++) {
			if (max9286_read(dev, 0x31) & MAX9286_FSYNC_LOCKED) {
				sync = true;
				break;
			}
			usleep_range(2000, 50000);
		}

		if (!sync) {
			dev_err(&dev->client->dev,
				"Failed to get frame synchronization\n");
			return -EINVAL;
		}

		/*
		 * Enable CSI output, VC set according to link number.
		 * Bit 7 must be set (chip manual says it's 0 and reserved)
		 */
		max9286_write(dev, 0x15, 0x80 | MAX9286_VCTYPE |
			      MAX9286_CSIOUTEN | MAX9286_0X15_RESV);
	} else {
		max9286_write(dev, 0x15, MAX9286_VCTYPE | MAX9286_0X15_RESV);

		/* Stop all cameras */
		for_each_source(dev, source)
			v4l2_subdev_call(source->sd, video, s_stream, 0);

		/* FIXME: See note in max9286_i2c_mux_select() */
		dev->streaming = 0;
	}

	return 0;
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

	if (format->pad >= MAX9286_SRC_PAD)
		return -EINVAL;

	mf->width	= 1280;
	mf->height	= 800;
	mf->code	= MEDIA_BUS_FMT_UYVY8_2X8;
	mf->colorspace	= V4L2_COLORSPACE_SMPTE170M;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int max9286_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
			   struct v4l2_mbus_frame_desc *fd)
{
	struct max9286_device *dev = sd_to_max9286(sd);
	unsigned int i;

	memset(fd, 0, sizeof(*fd));

	for (i = 0; i < dev->nsources; i++) {
		fd->entry[i].stream = i;
		fd->entry[i].bus.csi2.channel = i;
		fd->entry[i].bus.csi2.data_type = 0x1e;
	}

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = dev->nsources;

	return 0;
}

static int max9286_get_routing(struct v4l2_subdev *sd,
				 struct v4l2_subdev_routing *routing)
{
	struct max9286_device *dev = sd_to_max9286(sd);
	struct v4l2_subdev_route *r = routing->routes;
	struct max9286_source *source;

	/* There is one route per sink pad. */
	if (routing->num_routes < dev->nsources) {
		routing->num_routes = dev->nsources;
		return -ENOSPC;
	}

	routing->num_routes = dev->nsources;

	for_each_source(dev, source) {
		unsigned int index = to_index(dev, source);

		r->sink_pad = index;
		r->sink_stream = 0;
		r->source_pad = MAX9286_SRC_PAD;
		r->source_stream = index;
		r->flags = dev->route_mask & BIT(index) ?
			V4L2_SUBDEV_ROUTE_FL_ACTIVE : 0;
		r++;
	}

	return 0;
}

static int max9286_set_routing(struct v4l2_subdev *sd,
			       struct v4l2_subdev_routing *routing)
{

	struct max9286_device *dev = sd_to_max9286(sd);
	struct v4l2_subdev_route *r = routing->routes;
	unsigned int i;

	if (routing->num_routes > dev->nsources)
		return -ENOSPC;

	for (i = 0; i < routing->num_routes; i++) {
		if (r->sink_pad >= MAX9286_SRC_PAD ||
		    r->sink_stream != 0 ||
		    r->source_pad != MAX9286_SRC_PAD ||
		    r->source_stream != r->sink_pad)
			return -EINVAL;

		if (r->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE)
			dev->route_mask |= BIT(r->sink_pad);
		else
			dev->route_mask &= ~BIT(r->sink_pad);

		r++;
	}

	return 0;
}

static const struct v4l2_subdev_video_ops max9286_video_ops = {
	.s_stream	= max9286_s_stream,
};

static const struct v4l2_subdev_pad_ops max9286_pad_ops = {
	.enum_mbus_code = max9286_enum_mbus_code,
	.get_fmt	= max9286_get_fmt,
	.set_fmt	= max9286_get_fmt,
	.get_frame_desc = max9286_get_frame_desc,
	.get_routing	= max9286_get_routing,
	.set_routing	= max9286_set_routing,
};

static const struct v4l2_subdev_ops max9286_subdev_ops = {
	.video		= &max9286_video_ops,
	.pad		= &max9286_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Probe/Remove
 */

static int max9286_setup(struct max9286_device *dev)
{
	/*
	 * Link ordering values for all enabled links combinations. Orders must
	 * be assigned sequentially from 0 to the number of enabled links
	 * without leaving any hole for disabled links. We thus assign orders to
	 * enabled links first, and use the remaining order values for disabled
	 * links are all links must have a different order value;
	 */
	static const u8 link_order[] = {
		(3 << 6) | (2 << 4) | (1 << 2) | (0 << 0), /* xxxx */
		(3 << 6) | (2 << 4) | (1 << 2) | (0 << 0), /* xxx0 */
		(3 << 6) | (2 << 4) | (0 << 2) | (1 << 0), /* xx0x */
		(3 << 6) | (2 << 4) | (1 << 2) | (0 << 0), /* xx10 */
		(3 << 6) | (0 << 4) | (2 << 2) | (1 << 0), /* x0xx */
		(3 << 6) | (1 << 4) | (2 << 2) | (0 << 0), /* x1x0 */
		(3 << 6) | (1 << 4) | (0 << 2) | (2 << 0), /* x10x */
		(3 << 6) | (1 << 4) | (1 << 2) | (0 << 0), /* x210 */
		(0 << 6) | (3 << 4) | (2 << 2) | (1 << 0), /* 0xxx */
		(1 << 6) | (3 << 4) | (2 << 2) | (0 << 0), /* 1xx0 */
		(1 << 6) | (3 << 4) | (0 << 2) | (2 << 0), /* 1x0x */
		(2 << 6) | (3 << 4) | (1 << 2) | (0 << 0), /* 2x10 */
		(1 << 6) | (0 << 4) | (3 << 2) | (2 << 0), /* 10xx */
		(2 << 6) | (1 << 4) | (3 << 2) | (0 << 0), /* 21x0 */
		(2 << 6) | (1 << 4) | (0 << 2) | (3 << 0), /* 210x */
		(3 << 6) | (2 << 4) | (1 << 2) | (0 << 0), /* 3210 */
	};

	/*
	 * Set the I2C bus speed.
	 *
	 * Enable I2C Local Acknowledge during the probe sequences of the camera
	 * only. This should be disabled after the mux is initialised.
	 */
	max9286_configure_i2c(dev, true);

	/*
	 * Reverse channel setup
	 *
	 * - Enable custom reverse channel configuration (through register 0x3f)
	 *   and set the first pulse length to 35 clock cycles.
	 * - Increase the reverse channel amplitude to 170mV to accommodate the
	 *   high threshold enabled by the serializer driver.
	 */
	max9286_write(dev, 0x3f, MAX9286_EN_REV_CFG | MAX9286_REV_FLEN(35));
	max9286_write(dev, 0x3b, MAX9286_REV_TRF(1) | MAX9286_REV_AMP(70) |
		      MAX9286_REV_AMP_X);

#ifdef MAX9286_NEED_EQUALIZER
	/*
	 * Enable equalizer for all links to compensate for long cable
	 * attenuation.
	 */
	max9286_write(dev, 0x1b, MAX9286_ENEQ(3) | MAX9286_ENEQ(2) |
		      MAX9286_ENEQ(1) | MAX9286_ENEQ(0));
#endif

	/*
	 * Enable GMSL links, mask unused ones and autodetect link
	 * used as CSI clock source. Enable automask and autocomeback
	 * to protect against malfunctioning links.
	 *
	 * FIXME/TODO: Double check to see if we want to enable autocomeback and
	 * automask
	 */
	max9286_write(dev, 0x00, MAX9286_MSTLINKSEL_AUTO | dev->route_mask);
	max9286_write(dev, 0x0b, link_order[dev->route_mask]);
	max9286_write(dev, 0x69, MAX9286_AUTOCOMBACKEN | MAX9286_AUTOMASKEN |
		      (0xf & ~dev->route_mask));

	/* Video format setup */
	/* Disable CSI output, VC is set accordingly to Link number */
	max9286_write(dev, 0x15, MAX9286_VCTYPE | MAX9286_0X15_RESV);

	/*
	 * FIXME: once this driver will have an endpoint, retrieve
	 * CSI lanes number from there, and set image format properly.
	 * For now, it stays hardcoded to 4 lane only to comply with
	 * current VIN settings.
	 */
	/* Enable CSI-2 Lane D0-D3 only, DBL mode, YUV422 8-bit*/
	max9286_write(dev, 0x12, MAX9286_CSIDBL | MAX9286_DBL |
		      MAX9286_CSILANECNT(4) | MAX9286_DATATYPE_YUV422_8BIT);

	/* Automatic: FRAMESYNC taken from the slowest Link */
	max9286_write(dev, 0x01, MAX9286_FSYNCMODE_INT_HIZ |
		      MAX9286_FSYNCMETH_AUTO);

	/* Enable HS/VS encoding, use D14/15 for HS/VS, invert VS */
	max9286_write(dev, 0x0c, MAX9286_HVEN | MAX9286_INVVS |
		      MAX9286_HVSRC_D14);

	/*
	 * Wait for 2ms to allow the link to resynchronize after the
	 * configuration change.
	 */
	usleep_range(2000, 5000);

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
	struct fwnode_handle *ep;
	unsigned int i;
	int ret;

	if (!dev->of_node ||
	    !of_match_node(max9286_dt_ids, dev->of_node))
		/* skip non-max9286 devices */
		return 0;

	client = to_i2c_client(dev);
	max9286_dev = i2c_get_clientdata(client);

	/* Enable the bus power */
	ret = regulator_enable(max9286_dev->regulator);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to turn PoC on\n");
		return ret;
	}

	max9286_dev->poc_enabled = true;

	ret = max9286_setup(max9286_dev);
	if (ret) {
		dev_err(dev, "Unable to setup max9286\n");
		goto err_regulator;
	}

	/* This should expose the GPIO to provide a gpio-hog */

	/* Enable camera power */
	max9286_write(max9286_dev, 0x0f, 0xd0);

	usleep_range(7000000, 8000000);

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
		goto err_regulator;

	max9286_dev->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;

	max9286_dev->pads[MAX9286_SRC_PAD].flags = MEDIA_PAD_FL_SOURCE;
	for (i = 0; i < MAX9286_SRC_PAD; i++)
		max9286_dev->pads[i].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&max9286_dev->sd.entity, MAX9286_N_PADS,
				     max9286_dev->pads);
	if (ret)
		goto err_regulator;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 4, -1);
	if (!ep) {
		dev_err(dev, "Unable to retrieve endpoint on \"port@4\"\n");
		ret = -ENOENT;
		goto err_regulator;
	}
	max9286_dev->sd.fwnode = ep;

	ret = v4l2_async_register_subdev(&max9286_dev->sd);
	if (ret < 0) {
		dev_err(dev, "Unable to register subdevice\n");
		goto err_put_node;
	}

	ret = max9286_i2c_mux_init(max9286_dev);
	if (ret) {
		dev_err(dev, "Unable to initialize I2C multiplexer\n");
		goto err_subdev_unregister;
	}

	/*
	 * Re-configure I2C with local acknowledge disabled after cameras
	 * have probed
	 */
	max9286_configure_i2c(max9286_dev, false);

	/* Leave the mux channels disabled until they are selected */
	max9286_i2c_mux_close(max9286_dev);

	return 0;

err_subdev_unregister:
	v4l2_async_unregister_subdev(&max9286_dev->sd);
	max9286_i2c_mux_close(max9286_dev);
err_put_node:
	fwnode_handle_put(ep);
err_regulator:
	regulator_disable(max9286_dev->regulator);
	max9286_dev->poc_enabled = false;

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

static struct device_node *max9286_get_i2c_by_id(struct device_node *parent,
						 u32 id)
{
	struct device_node *child;

	for_each_child_of_node(parent, child) {
		u32 i2c_id = 0;

		if (of_node_cmp(child->name, "i2c") != 0)
			continue;
		of_property_read_u32(child, "reg", &i2c_id);
		if (id == i2c_id)
			return child;
	}

	return NULL;
}

static int max9286_check_i2c_bus_by_id(struct device *dev, int id)
{
	struct device_node *i2c_np;

	i2c_np = max9286_get_i2c_by_id(dev->of_node, id);
	if (!i2c_np) {
		dev_err(dev, "Failed to find corresponding i2c@%u\n", id);
		return -ENODEV;
	}

	if (!of_device_is_available(i2c_np)) {
		dev_dbg(dev, "Skipping port %u with disabled I2C bus\n", id);
		of_node_put(i2c_np);
		return -ENODEV;
	}

	of_node_put(i2c_np);

	return 0;
}

static void max9286_cleanup_dt(struct max9286_device *max9286)
{
	struct max9286_source *source;

	/*
	 * Not strictly part of the DT, but the notifier is registered during
	 * max9286_parse_dt(), and the notifier holds references to the fwnodes
	 * thus the cleanup is here to mirror the registration.
	 */
	v4l2_async_notifier_unregister(&max9286->notifier);

	/* Release our FWNode references */
	for_each_source(max9286, source) {
		fwnode_handle_put(source->fwnode);
		source->fwnode = NULL;
	}
}

static int max9286_parse_dt(struct max9286_device *max9286)
{
	struct device *dev = &max9286->client->dev;
	struct device_node *ep_np = NULL;

	for_each_endpoint_of_node(dev->of_node, ep_np) {
		struct max9286_source *source;
		struct of_endpoint ep;

		of_graph_parse_endpoint(ep_np, &ep);
		dev_dbg(dev, "Endpoint %pOF on port %d",
			ep.local_node, ep.port);

		/* Skip the source port */
		if (ep.port == MAX9286_NUM_GMSL)
			continue;

		if (ep.port > MAX9286_NUM_GMSL) {
			dev_err(dev, "Invalid endpoint %s on port %d",
				of_node_full_name(ep.local_node),
				ep.port);

			continue;
		}

		/* Skip if the corresponding GMSL link is unavailable */
		if (max9286_check_i2c_bus_by_id(dev, ep.port))
			continue;

		if (max9286->sources[ep.port].fwnode) {
			dev_err(dev,
				"Multiple port endpoints are not supported: %d",
				ep.port);

			continue;
		}

		source = &max9286->sources[ep.port];
		source->fwnode = fwnode_graph_get_remote_endpoint(
						of_fwnode_handle(ep_np));
		if (!source->fwnode) {
			dev_err(dev,
				"Endpoint %pOF has no remote endpoint connection\n",
				ep.local_node);

			continue;
		}

		source->asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
		source->asd.match.fwnode = source->fwnode;

		max9286->subdevs[max9286->nsources] = &source->asd;
		max9286->source_mask |= BIT(ep.port);
		max9286->nsources++;
	}

	/* Do not register subdevs if there are no devices */
	if (!max9286->nsources)
		return 0;

	max9286->route_mask = max9286->source_mask;

	max9286->notifier.ops = &max9286_notify_ops;
	max9286->notifier.subdevs = max9286->subdevs;
	max9286->notifier.num_subdevs = max9286->nsources;

	return v4l2_async_subdev_notifier_register(&max9286->sd,
						   &max9286->notifier);
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

	ret = max9286_parse_dt(dev);
	if (ret)
		return ret;

	dev->regulator = regulator_get(&client->dev, "poc");
	if (IS_ERR(dev->regulator)) {
		if (PTR_ERR(dev->regulator) != -EPROBE_DEFER)
			dev_err(&client->dev,
				"Unable to get PoC regulator (%ld)\n",
				PTR_ERR(dev->regulator));
		ret = PTR_ERR(dev->regulator);
		goto err_free;
	}

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
	max9286_i2c_mux_close(dev);

	/*
	 * Disable the I2C Auto-Acknowledge by configuring with the default
	 * I2C parameters (minus auto-ack)
	 */
	max9286_configure_i2c(dev, false);

	/* Add any userspace support before we return early */
	max9286_debugfs_init(dev);

	ret = device_for_each_child(client->dev.parent, &client->dev,
				    max9286_is_bound);
	if (ret)
		return 0;

	dev_dbg(&client->dev,
		"All max9286 probed: start initialization sequence\n");
	ret = device_for_each_child(client->dev.parent, NULL,
				    max9286_init);
	if (ret < 0)
		goto err_regulator;

	/* Leave the mux channels disabled until they are selected */
	max9286_i2c_mux_close(dev);

	return 0;

err_regulator:
	regulator_put(dev->regulator);
	max9286_i2c_mux_close(dev);
	max9286_configure_i2c(dev, false);
	max9286_debugfs_remove(dev);
err_free:
	max9286_cleanup_dt(dev);
	kfree(dev);

	return ret;
}

static int max9286_remove(struct i2c_client *client)
{
	struct max9286_device *dev = i2c_get_clientdata(client);

	/* Remove all Debugfs / sysfs entries */
	max9286_debugfs_remove(dev);

	i2c_mux_del_adapters(dev->mux);

	fwnode_handle_put(dev->sd.fwnode);
	v4l2_async_unregister_subdev(&dev->sd);

	if (dev->poc_enabled)
		regulator_disable(dev->regulator);
	regulator_put(dev->regulator);

	max9286_cleanup_dt(dev);
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
