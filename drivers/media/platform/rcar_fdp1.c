/*
 * Renesas RCar Fine Display Processor
 *
 * ...
 *
 * Author: Kieran Bingham, <kieran@bingham.xyz>
 * Copyright (c) 2016 Renesas Electronics Corporation.
 *
 * This code is developed and inspired from the vim2m, rcar_jpu,
 * and m2m-deinterlace drivers.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-vmalloc.h>

static unsigned debug;
module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "activate debug info");

/* Min Width/Height/Height-Field */
#define MIN_W 80
#define MIN_H 80
#define MIN_HF 40

#define MAX_W 3840
#define MAX_H 2160
#define MAX_HF 1080

#define DIM_ALIGN_MASK 7 /* 8-byte alignment for line length */

/* Flags that indicate a format can be used for capture/output */
#define FDP1_CAPTURE	(1 << 0)
#define FDP1_OUTPUT	(1 << 1)

#define MEM2MEM_NAME		"rcar_fdp1"

/* Per queue */
#define MEM2MEM_DEF_NUM_BUFS	VIDEO_MAX_FRAME
/* In bytes, per queue */
#define MEM2MEM_VID_MEM_LIMIT	(16 * 1024 * 1024)

/* Default transaction time in msec */
#define MEM2MEM_DEF_TRANSTIME	40
#define MEM2MEM_COLOR_STEP	(0xff >> 4)
#define MEM2MEM_NUM_TILES	8

/* Flags that indicate processing mode */
#define MEM2MEM_HFLIP	(1 << 0)
#define MEM2MEM_VFLIP	(1 << 1)

#define dprintk(dev, fmt, arg...) \
	v4l2_dbg(1, debug, &dev->v4l2_dev, "%s: " fmt, __func__, ## arg)


/**
 * struct fdp1_fmt - The FDP1 internal format data
 * @fourcc: the fourcc code, to match the V4L2 API
 * @depth: pixel depth
 * @fmt: 7-bit format code for the fdp1 hardware
 * @num_planes: number of planes
 * @types: types of queue this format is applicable to
 */
struct fdp1_fmt {
	u32	fourcc;
	u8	depth;
	u8	fmt;
	u8	num_planes;
	u8	types;
};

static struct fdp1_fmt formats[] = {

	/* RGB Formats are only supported by the Write Pixel Formatter */
	/*	FourCC	      depth  fmt  np   type       */
	{ V4L2_PIX_FMT_RGB332,    8, 0x00, 1, FDP1_CAPTURE },
	{ V4L2_PIX_FMT_XRGB555X, 16, 0x04, 1, FDP1_CAPTURE },
	{ V4L2_PIX_FMT_RGB565X,  16, 0x06, 1, FDP1_CAPTURE }, /* Big Endian */
	/* Not mapping the 18 bit (6-6-6) formats */
	{ V4L2_PIX_FMT_ARGB32,   24, 0x13, 1, FDP1_CAPTURE },
	/* XRGB Can be supported on the Capture, as A is arbitrary anyway */
	{ V4L2_PIX_FMT_XRGB32,   24, 0x13, 1, FDP1_CAPTURE },
	/* 0x14 = RGBA8888 */
	{ V4L2_PIX_FMT_RGB24,    24, 0x15, 1, FDP1_CAPTURE }, /* RGB 8-8-8 */
	{ V4L2_PIX_FMT_BGR24,    24, 0x18, 1, FDP1_CAPTURE },

	/* ARGB444 is confusing. uapi/linux/videodev2.h represents it as aaaarrrrggggbbbb
	 * however in https://linuxtv.org/downloads/v4l-dvb-apis/packed-rgb.html
	 * it is shown as g3 g2	g1 g0 b3 b2 b1 b0 | a3 a2 a1 a0 r3 r2 r1 r0
	 */
	{ V4L2_PIX_FMT_ARGB444,  16, 0x19, 1, FDP1_CAPTURE }, // TESTME I'm probably the wrong endianness !!!
	{ V4L2_PIX_FMT_ARGB555X, 16, 0x1B, 1, FDP1_CAPTURE },
	 /* Arbitrary Alpha Value, hence can re-use 0x1B */
	{ V4L2_PIX_FMT_XRGB555X, 16, 0x1B, 1, FDP1_CAPTURE },
};

#define NUM_FORMATS ARRAY_SIZE(formats)

/* Per-queue, driver-specific private data */
struct fdp1_q_data {
	unsigned int		width;
	unsigned int		height;
	unsigned int		sizeimage;
	struct fdp1_fmt	*fmt;
	struct v4l2_pix_format_mplane format;
	unsigned int sequence;
};

enum {
	V4L2_M2M_SRC = 0,
	V4L2_M2M_DST = 1,
};

#define V4L2_CID_TRANS_TIME_MSEC	(V4L2_CID_USER_BASE + 0x1000)
#define V4L2_CID_TRANS_NUM_BUFS		(V4L2_CID_USER_BASE + 0x1001)

static struct fdp1_fmt *find_format(struct v4l2_format *f)
{
	struct fdp1_fmt *fmt;
	unsigned int k;

	for (k = 0; k < NUM_FORMATS; k++) {
		fmt = &formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			break;
	}

	if (k == NUM_FORMATS)
		return NULL;

	return &formats[k];
}

struct fdp1_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	vfd;

	atomic_t		num_inst;
	struct mutex		dev_mutex;
	spinlock_t		irqlock;

	struct timer_list	timer;

	struct v4l2_m2m_dev	*m2m_dev;
};

struct fdp1_ctx {
	struct v4l2_fh		fh;
	struct fdp1_dev		*fdp1;

	struct v4l2_ctrl_handler hdl;

	/* Processed buffers in this transaction */
	u8			num_processed;

	/* Transaction length (i.e. how many buffers per transaction) */
	u32			translen;
	/* Transaction time (i.e. simulated processing time) in milliseconds */
	u32			transtime;

	/* Abort requested by m2m */
	int			aborting;

	/* Processing mode */
	int			mode;

	enum v4l2_colorspace	colorspace;

	/* Source and destination queue data */
	struct fdp1_q_data   out_q; /* HW Source */
	struct fdp1_q_data   cap_q; /* HW Destination */

};

static inline struct fdp1_ctx *file2ctx(struct file *file)
{
	return container_of(file->private_data, struct fdp1_ctx, fh);
}

static struct fdp1_q_data *get_q_data(struct fdp1_ctx *ctx,
					 enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->out_q;
	else
		return &ctx->cap_q;

	/* TODO: Do we want to restrict buffer types, to say
	 * V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
	 * and
	 * V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
	 * + the non MPLANE variants perhaps?
	 */

}

static int device_process(struct fdp1_ctx *ctx,
			  struct vb2_v4l2_buffer *in_vb,
			  struct vb2_v4l2_buffer *out_vb)
{
#ifdef OLD_CPU_DEINT
	struct fdp1_dev *fdp1 = ctx->fdp1;
	struct fdp1_q_data *q_data;
	u8 *p_in, *p_out;
	int x, y, t, w;
	int tile_w, bytes_left;
	int width, height, bytesperline;

	q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);

	width	= q_data->width;
	height	= q_data->height;
	bytesperline	= (q_data->width * q_data->fmt->depth) >> 3;

	p_in = vb2_plane_vaddr(&in_vb->vb2_buf, 0);
	p_out = vb2_plane_vaddr(&out_vb->vb2_buf, 0);
	if (!p_in || !p_out) {
		v4l2_err(&fdp1->v4l2_dev,
			 "Acquiring kernel pointers to buffers failed\n");
		return -EFAULT;
	}

	if (vb2_plane_size(&in_vb->vb2_buf, 0) >
			vb2_plane_size(&out_vb->vb2_buf, 0)) {
		v4l2_err(&fdp1->v4l2_dev, "Output buffer is too small\n");
		return -EINVAL;
	}

	tile_w = (width * (q_data[V4L2_M2M_DST].fmt->depth >> 3))
		/ MEM2MEM_NUM_TILES;
	bytes_left = bytesperline - tile_w * MEM2MEM_NUM_TILES;
	w = 0;

	out_vb->sequence =
		get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE)->sequence++;
	in_vb->sequence = q_data->sequence++;
	out_vb->vb2_buf.timestamp = in_vb->vb2_buf.timestamp;

	if (in_vb->flags & V4L2_BUF_FLAG_TIMECODE)
		out_vb->timecode = in_vb->timecode;
	out_vb->field = in_vb->field;
	out_vb->flags = in_vb->flags &
		(V4L2_BUF_FLAG_TIMECODE |
		 V4L2_BUF_FLAG_KEYFRAME |
		 V4L2_BUF_FLAG_PFRAME |
		 V4L2_BUF_FLAG_BFRAME |
		 V4L2_BUF_FLAG_TSTAMP_SRC_MASK);

	switch (ctx->mode) {
	case MEM2MEM_HFLIP | MEM2MEM_VFLIP:
		p_out += bytesperline * height - bytes_left;
		for (y = 0; y < height; ++y) {
			for (t = 0; t < MEM2MEM_NUM_TILES; ++t) {
				if (w & 0x1) {
					for (x = 0; x < tile_w; ++x)
						*--p_out = *p_in++ +
							MEM2MEM_COLOR_STEP;
				} else {
					for (x = 0; x < tile_w; ++x)
						*--p_out = *p_in++ -
							MEM2MEM_COLOR_STEP;
				}
				++w;
			}
			p_in += bytes_left;
			p_out -= bytes_left;
		}
		break;

	case MEM2MEM_HFLIP:
		for (y = 0; y < height; ++y) {
			p_out += MEM2MEM_NUM_TILES * tile_w;
			for (t = 0; t < MEM2MEM_NUM_TILES; ++t) {
				if (w & 0x01) {
					for (x = 0; x < tile_w; ++x)
						*--p_out = *p_in++ +
							MEM2MEM_COLOR_STEP;
				} else {
					for (x = 0; x < tile_w; ++x)
						*--p_out = *p_in++ -
							MEM2MEM_COLOR_STEP;
				}
				++w;
			}
			p_in += bytes_left;
			p_out += bytesperline;
		}
		break;

	case MEM2MEM_VFLIP:
		p_out += bytesperline * (height - 1);
		for (y = 0; y < height; ++y) {
			for (t = 0; t < MEM2MEM_NUM_TILES; ++t) {
				if (w & 0x1) {
					for (x = 0; x < tile_w; ++x)
						*p_out++ = *p_in++ +
							MEM2MEM_COLOR_STEP;
				} else {
					for (x = 0; x < tile_w; ++x)
						*p_out++ = *p_in++ -
							MEM2MEM_COLOR_STEP;
				}
				++w;
			}
			p_in += bytes_left;
			p_out += bytes_left - 2 * bytesperline;
		}
		break;

	default:
		for (y = 0; y < height; ++y) {
			for (t = 0; t < MEM2MEM_NUM_TILES; ++t) {
				if (w & 0x1) {
					for (x = 0; x < tile_w; ++x)
						*p_out++ = *p_in++ +
							MEM2MEM_COLOR_STEP;
				} else {
					for (x = 0; x < tile_w; ++x)
						*p_out++ = *p_in++ -
							MEM2MEM_COLOR_STEP;
				}
				++w;
			}
			p_in += bytes_left;
			p_out += bytes_left;
		}
	}
#endif // #ifdef OLD_CPU_DEINT

	return 0;
}

static void schedule_irq(struct fdp1_dev *fdp1, int msec_timeout)
{
	dprintk(fdp1, "Scheduling a simulated irq\n");
	mod_timer(&fdp1->timer, jiffies + msecs_to_jiffies(msec_timeout));
}

/*
 * mem2mem callbacks
 */

/**
 * job_ready() - check whether an instance is ready to be scheduled to run
 */
static int job_ready(void *priv)
{
	struct fdp1_ctx *ctx = priv;

	if (v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) < ctx->translen
	    || v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) < ctx->translen) {
		dprintk(ctx->fdp1, "Not enough buffers available\n");
		return 0;
	}

	return 1;
}

static void job_abort(void *priv)
{
	struct fdp1_ctx *ctx = priv;

	/* Will cancel the transaction in the next interrupt handler */
	ctx->aborting = 1;
}

/* device_run() - prepares and starts the device
 *
 * This simulates all the immediate preparations required before starting
 * a device. This will be called by the framework when it decides to schedule
 * a particular instance.
 */
static void device_run(void *priv)
{
	struct fdp1_ctx *ctx = priv;
	struct fdp1_dev *fdp1 = ctx->fdp1;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;

	src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	device_process(ctx, src_buf, dst_buf);

	/* Run a timer, which simulates a hardware irq  */
	schedule_irq(fdp1, ctx->transtime);
}

static void device_isr(unsigned long priv)
{
	struct fdp1_dev *fdp1_dev = (struct fdp1_dev *)priv;
	struct fdp1_ctx *curr_ctx;
	struct vb2_v4l2_buffer *src_vb, *dst_vb;
	unsigned long flags;

	curr_ctx = v4l2_m2m_get_curr_priv(fdp1_dev->m2m_dev);

	if (NULL == curr_ctx) {
		pr_err("Instance released before the end of transaction\n");
		return;
	}

	src_vb = v4l2_m2m_src_buf_remove(curr_ctx->fh.m2m_ctx);
	dst_vb = v4l2_m2m_dst_buf_remove(curr_ctx->fh.m2m_ctx);

	curr_ctx->num_processed++;

	spin_lock_irqsave(&fdp1_dev->irqlock, flags);
	v4l2_m2m_buf_done(src_vb, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst_vb, VB2_BUF_STATE_DONE);
	spin_unlock_irqrestore(&fdp1_dev->irqlock, flags);

	if (curr_ctx->num_processed == curr_ctx->translen
	    || curr_ctx->aborting) {
		dprintk(curr_ctx->fdp1, "Finishing transaction\n");
		curr_ctx->num_processed = 0;
		v4l2_m2m_job_finish(fdp1_dev->m2m_dev, curr_ctx->fh.m2m_ctx);
	} else {
		device_run(curr_ctx);
	}
}

/*
 * video ioctls
 */
static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strncpy(cap->driver, MEM2MEM_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, MEM2MEM_NAME, sizeof(cap->card) - 1);
	snprintf(cap->bus_info, sizeof(cap->bus_info),
			"platform:%s", MEM2MEM_NAME);
	cap->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int enum_fmt(struct v4l2_fmtdesc *f, u32 type)
{
	int i, num;
	struct fdp1_fmt *fmt;

	num = 0;

	for (i = 0; i < NUM_FORMATS; ++i) {
		if (formats[i].types & type) {
			if (num == f->index)
				break;
			++num;
		}
	}

	/* Format not found */
	if (i >= NUM_FORMATS)
		return -EINVAL;

	/* Format found */
	f->pixelformat = formats[i].fourcc;

	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, FDP1_CAPTURE);
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, FDP1_OUTPUT);
}

static int vidioc_g_fmt(struct fdp1_ctx *ctx, struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct fdp1_q_data *q_data;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);

	f->fmt.pix.width	= q_data->width;
	f->fmt.pix.height	= q_data->height;
	f->fmt.pix.field	= V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat	= q_data->fmt->fourcc;
	f->fmt.pix.bytesperline	= (q_data->width * q_data->fmt->depth) >> 3;
	f->fmt.pix.sizeimage	= q_data->sizeimage;
	f->fmt.pix.colorspace	= ctx->colorspace;

	return 0;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return vidioc_g_fmt(file2ctx(file), f);
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return vidioc_g_fmt(file2ctx(file), f);
}

static int vidioc_try_fmt(struct v4l2_format *f, struct fdp1_fmt *fmt)
{
	/* V4L2 specification suggests the driver corrects the format struct
	 * if any of the dimensions is unsupported */
	if (f->fmt.pix.height < MIN_H)
		f->fmt.pix.height = MIN_H;
	else if (f->fmt.pix.height > MAX_H)
		f->fmt.pix.height = MAX_H;

	if (f->fmt.pix.width < MIN_W)
		f->fmt.pix.width = MIN_W;
	else if (f->fmt.pix.width > MAX_W)
		f->fmt.pix.width = MAX_W;

	f->fmt.pix.width &= ~DIM_ALIGN_MASK;
	f->fmt.pix.bytesperline = (f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.bytesperline;
	f->fmt.pix.field = V4L2_FIELD_NONE;

	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct fdp1_fmt *fmt;
	struct fdp1_ctx *ctx = file2ctx(file);

	fmt = find_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = formats[0].fourcc;
		fmt = find_format(f);
	}
	if (!(fmt->types & FDP1_CAPTURE)) {
		v4l2_err(&ctx->fdp1->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}
	f->fmt.pix.colorspace = ctx->colorspace;

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_try_fmt_vid_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct fdp1_fmt *fmt;
	struct fdp1_ctx *ctx = file2ctx(file);

	fmt = find_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = formats[0].fourcc;
		fmt = find_format(f);
	}
	if (!(fmt->types & FDP1_OUTPUT)) {
		v4l2_err(&ctx->fdp1->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}
	if (!f->fmt.pix.colorspace)
		f->fmt.pix.colorspace = V4L2_COLORSPACE_REC709;

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_s_fmt(struct fdp1_ctx *ctx, struct v4l2_format *f)
{
	struct fdp1_q_data *q_data;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);
	if (!q_data)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		v4l2_err(&ctx->fdp1->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	q_data->fmt		= find_format(f);
	q_data->width		= f->fmt.pix.width;
	q_data->height		= f->fmt.pix.height;
	q_data->sizeimage	= q_data->width * q_data->height
				* q_data->fmt->depth >> 3;

	dprintk(ctx->fdp1,
		"Setting format for type %d, wxh: %dx%d, fmt: %d\n",
		f->type, q_data->width, q_data->height, q_data->fmt->fourcc);

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;

	ret = vidioc_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	return vidioc_s_fmt(file2ctx(file), f);
}

static int vidioc_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct fdp1_ctx *ctx = file2ctx(file);
	int ret;

	ret = vidioc_try_fmt_vid_out(file, priv, f);
	if (ret)
		return ret;

	ret = vidioc_s_fmt(file2ctx(file), f);
	if (!ret)
		ctx->colorspace = f->fmt.pix.colorspace;
	return ret;
}

static int fdp1_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct fdp1_ctx *ctx =
		container_of(ctrl->handler, struct fdp1_ctx, hdl);

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			ctx->mode |= MEM2MEM_HFLIP;
		else
			ctx->mode &= ~MEM2MEM_HFLIP;
		break;

	case V4L2_CID_VFLIP:
		if (ctrl->val)
			ctx->mode |= MEM2MEM_VFLIP;
		else
			ctx->mode &= ~MEM2MEM_VFLIP;
		break;

	case V4L2_CID_TRANS_TIME_MSEC:
		ctx->transtime = ctrl->val;
		break;

	case V4L2_CID_TRANS_NUM_BUFS:
		ctx->translen = ctrl->val;
		break;

	default:
		v4l2_err(&ctx->fdp1->v4l2_dev, "Invalid control\n");
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops fdp1_ctrl_ops = {
	.s_ctrl = fdp1_s_ctrl,
};


static const struct v4l2_ioctl_ops fdp1_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,

	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	= vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	= vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	= vidioc_s_fmt_vid_cap,

	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out	= vidioc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out	= vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out	= vidioc_s_fmt_vid_out,

	.vidioc_reqbufs		= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf	= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf		= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf		= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf	= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs	= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf		= v4l2_m2m_ioctl_expbuf,

	.vidioc_streamon	= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff	= v4l2_m2m_ioctl_streamoff,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};


/*
 * Queue operations
 */

static int fdp1_queue_setup(struct vb2_queue *vq,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct fdp1_ctx *ctx = vb2_get_drv_priv(vq);
	struct fdp1_q_data *q_data;
	unsigned int size, count = *nbuffers;

	q_data = get_q_data(ctx, vq->type);

	size = q_data->width * q_data->height * q_data->fmt->depth >> 3;

	while (size * count > MEM2MEM_VID_MEM_LIMIT)
		(count)--;
	*nbuffers = count;

	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = size;

	/*
	 * videobuf2-vmalloc allocator is context-less so no need to set
	 * alloc_ctxs array.
	 */

	dprintk(ctx->fdp1, "get %d buffer(s) of size %d each.\n", count, size);

	return 0;
}

static int fdp1_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct fdp1_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct fdp1_q_data *q_data;

	dprintk(ctx->fdp1, "type: %d\n", vb->vb2_queue->type);

	q_data = get_q_data(ctx, vb->vb2_queue->type);
	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (vbuf->field == V4L2_FIELD_ANY)
			vbuf->field = V4L2_FIELD_NONE;
		if (vbuf->field != V4L2_FIELD_NONE) {
			dprintk(ctx->fdp1, "%s field isn't supported\n",
					__func__);
			return -EINVAL;
		}
	}

	if (vb2_plane_size(vb, 0) < q_data->sizeimage) {
		dprintk(ctx->fdp1, "%s data will not fit into plane (%lu < %lu)\n",
				__func__, vb2_plane_size(vb, 0), (long)q_data->sizeimage);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, q_data->sizeimage);

	return 0;
}

static void fdp1_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct fdp1_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int fdp1_start_streaming(struct vb2_queue *q, unsigned count)
{
	struct fdp1_ctx *ctx = vb2_get_drv_priv(q);
	struct fdp1_q_data *q_data = get_q_data(ctx, q->type);

	q_data->sequence = 0;
	return 0;
}

static void fdp1_stop_streaming(struct vb2_queue *q)
{
	struct fdp1_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vbuf;
	unsigned long flags;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(q->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (vbuf == NULL)
			return;
		spin_lock_irqsave(&ctx->fdp1->irqlock, flags);
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
		spin_unlock_irqrestore(&ctx->fdp1->irqlock, flags);
	}
}

static struct vb2_ops fdp1_qops = {
	.queue_setup	 = fdp1_queue_setup,
	.buf_prepare	 = fdp1_buf_prepare,
	.buf_queue	 = fdp1_buf_queue,
	.start_streaming = fdp1_start_streaming,
	.stop_streaming  = fdp1_stop_streaming,
	.wait_prepare	 = vb2_ops_wait_prepare,
	.wait_finish	 = vb2_ops_wait_finish,
};

static int queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct fdp1_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &fdp1_qops;
	src_vq->mem_ops = &vb2_vmalloc_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->fdp1->dev_mutex;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &fdp1_qops;
	dst_vq->mem_ops = &vb2_vmalloc_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->fdp1->dev_mutex;

	return vb2_queue_init(dst_vq);
}

static const struct v4l2_ctrl_config fdp1_ctrl_trans_time_msec = {
	.ops = &fdp1_ctrl_ops,
	.id = V4L2_CID_TRANS_TIME_MSEC,
	.name = "Transaction Time (msec)",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.def = MEM2MEM_DEF_TRANSTIME,
	.min = 1,
	.max = 10001,
	.step = 1,
};

static const struct v4l2_ctrl_config fdp1_ctrl_trans_num_bufs = {
	.ops = &fdp1_ctrl_ops,
	.id = V4L2_CID_TRANS_NUM_BUFS,
	.name = "Buffers Per Transaction",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.def = 1,
	.min = 1,
	.max = MEM2MEM_DEF_NUM_BUFS,
	.step = 1,
};

/*
 * File operations
 */
static int fdp1_open(struct file *file)
{
	struct fdp1_dev *fdp1 = video_drvdata(file);
	struct fdp1_ctx *ctx = NULL;
	int rc = 0;

	if (mutex_lock_interruptible(&fdp1->dev_mutex))
		return -ERESTARTSYS;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		rc = -ENOMEM;
		goto open_unlock;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	ctx->fdp1 = fdp1;


	/* Perform v4l2_ctrl_handler_setup(hdl); if desired */

	/* M2M_SRC = Out
	 * M2M_DST = Cap */

	ctx->out_q.fmt = &formats[0];
	ctx->out_q.width = 1920;
	ctx->out_q.height = 1080;
	ctx->out_q.sizeimage =
		ctx->out_q.width *
		ctx->out_q.height *
		(ctx->out_q.fmt->depth >> 3);
	ctx->cap_q = ctx->out_q;

	/* Better for above
	__fdp1_try_fmt(ctx, &ctx->out_q.fmtinfo, &ctx->out_q.format,
		      V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	__fdp1_try_fmt(ctx, &ctx->cap_q.fmtinfo, &ctx->cap_q.format,
		      V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	 */

	/* Configure default parameters. */

	ctx->colorspace = V4L2_COLORSPACE_REC709;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(fdp1->m2m_dev, ctx, &queue_init);

	if (IS_ERR(ctx->fh.m2m_ctx)) {
		rc = PTR_ERR(ctx->fh.m2m_ctx);

		// No handler yet ... v4l2_ctrl_handler_free(hdl);
		kfree(ctx);
		goto open_unlock;
	}

	v4l2_fh_add(&ctx->fh);
	atomic_inc(&fdp1->num_inst);

	dprintk(fdp1, "Created instance: %p, m2m_ctx: %p\n",
		ctx, ctx->fh.m2m_ctx);

open_unlock:
	mutex_unlock(&fdp1->dev_mutex);
	return rc;
}

static int fdp1_release(struct file *file)
{
	struct fdp1_dev *fdp1 = video_drvdata(file);
	struct fdp1_ctx *ctx = file2ctx(file);

	dprintk(fdp1, "Releasing instance %p\n", ctx);

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->hdl);
	mutex_lock(&fdp1->dev_mutex);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&fdp1->dev_mutex);
	kfree(ctx);

	atomic_dec(&fdp1->num_inst);

	return 0;
}

static const struct v4l2_file_operations fdp1_fops = {
	.owner		= THIS_MODULE,
	.open		= fdp1_open,
	.release	= fdp1_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static struct video_device fdp1_videodev = {
	.name		= MEM2MEM_NAME,
	.vfl_dir	= VFL_DIR_M2M,
	.fops		= &fdp1_fops,
	.ioctl_ops	= &fdp1_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release_empty,
};

static struct v4l2_m2m_ops m2m_ops = {
	.device_run	= device_run,
	.job_ready	= job_ready,
	.job_abort	= job_abort,
};

static int fdp1_probe(struct platform_device *pdev)
{
	struct fdp1_dev *fdp1;
	struct video_device *vfd;
	int ret;

	fdp1 = devm_kzalloc(&pdev->dev, sizeof(*fdp1), GFP_KERNEL);
	if (!fdp1)
		return -ENOMEM;

	spin_lock_init(&fdp1->irqlock);

	ret = v4l2_device_register(&pdev->dev, &fdp1->v4l2_dev);
	if (ret)
		return ret;

	atomic_set(&fdp1->num_inst, 0);
	mutex_init(&fdp1->dev_mutex);

	fdp1->vfd = fdp1_videodev;
	vfd = &fdp1->vfd;
	vfd->lock = &fdp1->dev_mutex;
	vfd->v4l2_dev = &fdp1->v4l2_dev;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
	if (ret) {
		v4l2_err(&fdp1->v4l2_dev, "Failed to register video device\n");
		goto unreg_dev;
	}

	video_set_drvdata(vfd, fdp1);
	snprintf(vfd->name, sizeof(vfd->name), "%s", fdp1_videodev.name);
	v4l2_info(&fdp1->v4l2_dev,
			"Device registered as /dev/video%d\n", vfd->num);


	setup_timer(&fdp1->timer, device_isr, (long)fdp1);
	platform_set_drvdata(pdev, fdp1);

	fdp1->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(fdp1->m2m_dev)) {
		v4l2_err(&fdp1->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(fdp1->m2m_dev);
		goto err_m2m;
	}

	return 0;

err_m2m:
	v4l2_m2m_release(fdp1->m2m_dev);
	video_unregister_device(&fdp1->vfd);
unreg_dev:
	v4l2_device_unregister(&fdp1->v4l2_dev);

	return ret;
}

static int fdp1_remove(struct platform_device *pdev)
{
	struct fdp1_dev *fdp1 = platform_get_drvdata(pdev);

	v4l2_info(&fdp1->v4l2_dev, "Removing " MEM2MEM_NAME);
	v4l2_m2m_release(fdp1->m2m_dev);
	del_timer_sync(&fdp1->timer);
	video_unregister_device(&fdp1->vfd);
	v4l2_device_unregister(&fdp1->v4l2_dev);

	return 0;
}

static const struct of_device_id fdp1_dt_ids[] = {
	{ .compatible = "renesas,fdp1-r8a7795" }, /* H3 */
	{ .compatible = "renesas,fdp1-r8a7796" }, /* M3-W */
	{ .compatible = "renesas,rcar-gen3-fdp1" },
	{ },
};
MODULE_DEVICE_TABLE(of, fdp1_dt_ids);

static struct platform_driver fdp1_pdrv = {
	.probe		= fdp1_probe,
	.remove		= fdp1_remove,
	.driver		= {
		.name	= MEM2MEM_NAME,
		.of_match_table = fdp1_dt_ids,
	},
};

module_platform_driver(fdp1_pdrv);

MODULE_DESCRIPTION("Renesas R-Car Fine Display Processor");
MODULE_AUTHOR("Kieran Bingham, <kieran@bingham.xyz>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
MODULE_ALIAS("platform:" MEM2MEM_NAME);
