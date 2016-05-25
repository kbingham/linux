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
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>
#include <media/rcar-fcp.h>

#include <linux/debugfs.h>

#ifdef CONFIG_ARM
/* Don't perform register read/writes on qemu / 32 bit build */
#define QEMU_TESTING
#endif

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
#define MEMALIGN 8

#define DIM_ALIGN_MASK 7 /* 8-byte alignment for line length */

/* Flags that indicate a format can be used for capture/output */
#define FDP1_CAPTURE	(1 << 0)
#define FDP1_OUTPUT	(1 << 1)

#define DRIVER_NAME		"rcar_fdp1"

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

/*
 * FDP1 registers and bits
 */

/* FDP1 start register - Imm */
#define CTL_CMD			0x0000
#define CTL_CMD_STRCMD		BIT(0)

/* Sync generator register - Imm */
#define CTL_SGCMD		0x0004
#define CTL_SGCMD_SGEN		BIT(0)

/* Register set end register - Imm */
#define CTL_REGEND		0x0008
#define CTL_REGEND_REGEND	BIT(0)

/* Channel activation register - Vupdt */
#define CTL_CHACT		0x000C
#define CTL_CHACT_SMW		BIT(9)
#define CTL_CHACT_WR		BIT(8)
#define CTL_CHACT_SMR		BIT(3)
#define CTL_CHACT_RD2		BIT(2)
#define CTL_CHACT_RD1		BIT(1)
#define CTL_CHACT_RD0		BIT(0)

/* Operation Mode Register - Vupdt */
#define CTL_OPMODE		0x0010
#define CTL_OPMODE_PRG		BIT(4)
#define CTL_OPMODE_VIMD		GENMASK(1,0) //0x0003
#define CTL_OPMODE_INTERRUPT	0
#define CTL_OPMODE_BEST_EFFORT	1
#define CTL_OPMODE_NO_INTERRUPT	2

#define CTL_VPERIOD		0x0014
#define CTL_CLKCTRL		0x0018
#define CTL_CLKCTRL_CSTP_N	BIT(0)

/* Software reset register */
#define CTL_SRESET		0x001C
#define CTL_SRESET_SRST		BIT(0)

/* Control status register (V-update-status) */
#define CTL_STATUS		0x0024
#define CTL_STATUS_VINT_CNT(x)	((x & 0xff) >> 16)
#define CTL_STATUS_SGREGSET	BIT(10)
#define CTL_STATUS_SGVERR	BIT(9)
#define CTL_STATUS_SGFREND	BIT(8)
#define CTL_STATUS_BSY		BIT(0)

#define CTL_VCYCLE_STATUS	0x0028

/* Interrupt enable register */
#define CTL_IRQENB		0x0038
/* Interrupt status register */
#define CTL_IRQSTA		0x003C
/* Interrupt control register */
#define CTL_IRQFSET		0x0040

/* Common IRQ Bit settings */
#define CTL_IRQ_VERE		BIT(16)
#define CTL_IRQ_VINTE		BIT(4)
#define CTL_IRQ_FREE		BIT(0)
#define CTL_IRQ_MASK		(CTL_IRQ_VERE | CTL_IRQ_VINTE | CTL_IRQ_FREE)

/* RPF */
#define RPF_SIZE		0x0060
#define RPF_FORMAT		0x0064
#define RPF_PSTRIDE		0x0068

/* RPF0 Source Component Y Address register */
#define RPF0_ADDR_Y		0x006C

/* RPF1 Current Picture Registers */
#define RPF1_ADDR_Y		0x0078
#define RPF1_ADDR_C0		0x007C
#define RPF1_ADDR_C1		0x0080

/* RPF2 next picture register */
#define RPF2_ADDR_Y		0x0084

#define RPF_SMSK_ADDR		0x0090
#define RPF_SWAP		0x0094

/* WPF */
#define WPF_FORMAT		0x00C0
#define WPF_FORMAT_PDV_MASK
#define WPF_FORMAT_FCNL		BIT(20)
#define WPF_FORMAT_WSPYCS	BIT(15)
#define WPF_FORMAT_WSPUVS	BIT(14)
#define WPF_FORMAT_DITH
#define WPF_FORMAT_WRTM
#define WPF_FORMAT_CSC		BIT(8)

#define WPF_RND_CTL		0x00C4
#define WPF_PSTRIDE		0x00C8

/* WPF Destination picture */
#define WPF_ADDR_Y		0x00CC
#define WPF_ADDR_C0		0x00D0
#define WPF_ADDR_C1		0x00D4
#define WPF_SWAP		0x00D8

/* IPC */
#define IPC_MODE		0x0100
#define IPC_MODE_DLI		BIT(8)
#define IPC_MODE_DIM		GENMASK(2,0) // 7
#define IPC_MODE_DIM_ADAPT2D3D	0
#define IPC_MODE_DIM_FIXED2D	1
#define IPC_MODE_DIM_FIXED3D	2
#define IPC_MODE_DIM_PREVFIELD	3
#define IPC_MODE_DIM_NEXTFIELD	4

#define IPC_SMSK_THRESH		0x0104
#define IPC_COMB_DET		0x0108
#define IPC_MOTDEC		0x010C

/* DLI registers */
#define IPC_DLI_BLEND		0x0120
#define IPC_DLI_HGAIN		0x0124
#define IPC_DLI_SPRS		0x0128
#define IPC_DLI_ANGLE		0x012C
#define IPC_DLI_ISOPIX0		0x0130
#define IPC_DLI_ISOPIX1		0x0134

/* Sensor registers */
#define IPC_SENSOR_TH0		0x0140
#define IPC_SENSOR_CTL0		0x0170
#define IPC_SENSOR_CTL1		0x0174
#define IPC_SENSOR_CTL2		0x0178
#define IPC_SENSOR_CTL3		0x017C

/* Line memory pixel number register - Vupdt */
#define IPC_LMEM		0x01E0

/* Internal Data (HW Version) */
#define IP_INTDATA 		0x0800
#define IP_H3			0x02010101
#define IP_M3W			0x02010202

/* DebugFS Regsets */
#define FDP1_DBFS_REG(reg) { #reg, reg }

static struct debugfs_reg32 fdp1_regset[] = {
	FDP1_DBFS_REG(CTL_CMD),
	FDP1_DBFS_REG(CTL_SGCMD),
	FDP1_DBFS_REG(CTL_REGEND),
	FDP1_DBFS_REG(CTL_CHACT),
	FDP1_DBFS_REG(CTL_OPMODE),
	FDP1_DBFS_REG(CTL_VPERIOD),
	FDP1_DBFS_REG(CTL_CLKCTRL),
	FDP1_DBFS_REG(CTL_SRESET),
	FDP1_DBFS_REG(CTL_STATUS),
	FDP1_DBFS_REG(CTL_VCYCLE_STATUS),
	FDP1_DBFS_REG(CTL_IRQENB),
	FDP1_DBFS_REG(CTL_IRQSTA),
	FDP1_DBFS_REG(CTL_IRQFSET),
	FDP1_DBFS_REG(RPF_SIZE),
	FDP1_DBFS_REG(RPF_FORMAT),
	FDP1_DBFS_REG(RPF_PSTRIDE),
	FDP1_DBFS_REG(RPF0_ADDR_Y),
	FDP1_DBFS_REG(RPF1_ADDR_Y),
	FDP1_DBFS_REG(RPF1_ADDR_C0),
	FDP1_DBFS_REG(RPF1_ADDR_C1),
	FDP1_DBFS_REG(RPF2_ADDR_Y),
	FDP1_DBFS_REG(RPF_SMSK_ADDR),
	FDP1_DBFS_REG(RPF_SWAP),
	FDP1_DBFS_REG(WPF_FORMAT),
	FDP1_DBFS_REG(WPF_RND_CTL),
	FDP1_DBFS_REG(WPF_PSTRIDE),
	FDP1_DBFS_REG(WPF_ADDR_Y),
	FDP1_DBFS_REG(WPF_ADDR_C0),
	FDP1_DBFS_REG(WPF_ADDR_C1),
	FDP1_DBFS_REG(WPF_SWAP),
	FDP1_DBFS_REG(IPC_MODE),
	FDP1_DBFS_REG(IPC_SMSK_THRESH),
	FDP1_DBFS_REG(IPC_COMB_DET),
	FDP1_DBFS_REG(IPC_MOTDEC),
	FDP1_DBFS_REG(IPC_DLI_BLEND),
	FDP1_DBFS_REG(IPC_DLI_HGAIN),
	FDP1_DBFS_REG(IPC_DLI_SPRS),
	FDP1_DBFS_REG(IPC_DLI_ANGLE),
	FDP1_DBFS_REG(IPC_DLI_ISOPIX0),
	FDP1_DBFS_REG(IPC_DLI_ISOPIX1),
	FDP1_DBFS_REG(IPC_SENSOR_TH0),
	FDP1_DBFS_REG(IPC_SENSOR_CTL0),
	FDP1_DBFS_REG(IPC_SENSOR_CTL1),
	FDP1_DBFS_REG(IPC_SENSOR_CTL2),
	FDP1_DBFS_REG(IPC_SENSOR_CTL3),
	FDP1_DBFS_REG(IPC_LMEM),
	FDP1_DBFS_REG(IP_INTDATA),
};

#define NUM_FDP1_REGSETS ARRAY_SIZE(fdp1_regset)


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
	u8	bpp[3];
	u32	fmt;
	u8	num_planes;
	u8	types;
};

static struct fdp1_fmt formats[] = {
	/* RGB Formats are only supported by the Write Pixel Formatter */
	/*	FourCC	            BPP[3]    fmt  np   type       */
	{ V4L2_PIX_FMT_RGB332,   { 8, 0, 0}, 0x00, 1, FDP1_CAPTURE },
	{ V4L2_PIX_FMT_XRGB555X, {16, 0, 0}, 0x04, 1, FDP1_CAPTURE },
	{ V4L2_PIX_FMT_RGB565X,  {16, 0, 0}, 0x06, 1, FDP1_CAPTURE }, /* Big Endian */
	/* Not mapping the 18 bit (6-6-6) formats */
	{ V4L2_PIX_FMT_ARGB32,   {24, 0, 0}, 0x13, 1, FDP1_CAPTURE },
	/* XRGB Can be supported on the Capture, as A is arbitrary anyway */
	{ V4L2_PIX_FMT_XRGB32,   {24, 0, 0}, 0x13, 1, FDP1_CAPTURE },
	/* 0x14 = RGBA8888 */
	{ V4L2_PIX_FMT_RGB24,    {24, 0, 0}, 0x15, 1, FDP1_CAPTURE }, /* RGB 8-8-8 */
	{ V4L2_PIX_FMT_BGR24,    {24, 0, 0}, 0x18, 1, FDP1_CAPTURE },

	/* ARGB444 is confusing. uapi/linux/videodev2.h represents it as aaaarrrrggggbbbb
	 * however in https://linuxtv.org/downloads/v4l-dvb-apis/packed-rgb.html
	 * it is shown as g3 g2	g1 g0 b3 b2 b1 b0 | a3 a2 a1 a0 r3 r2 r1 r0
	 * We can use the
	 */
	{ V4L2_PIX_FMT_ARGB444,  {16, 0, 0}, 0x19, 1, FDP1_CAPTURE }, // TESTME I'm probably the wrong endianness !!!
	{ V4L2_PIX_FMT_ARGB555X, {16, 0, 0}, 0x1B, 1, FDP1_CAPTURE },
	 /* Arbitrary Alpha Value, hence can re-use 0x1B */
	{ V4L2_PIX_FMT_XRGB555X, {16, 0, 0}, 0x1B, 1, FDP1_CAPTURE },


	/* YUV Formats */
	//{ V4L2_PIX_FMT_,  { 8,16, 0}, 0x40, 2, FDP1_CAPTURE | FDP1_OUTPUT }, /* YCbCr4:4:4 semi */
	{ V4L2_PIX_FMT_NV16,  { 8, 8, 0}, 0x41, 2, FDP1_CAPTURE | FDP1_OUTPUT }, /* YCbCr4:2:2 semi (NV16, NV61) */
	{ V4L2_PIX_FMT_NV61,  { 8, 8, 0}, 0x41 | WPF_FORMAT_WSPUVS,
			2, FDP1_CAPTURE | FDP1_OUTPUT }, /* YCbCr4:2:2 semi (NV16, NV61) */
	{ V4L2_PIX_FMT_NV12,  { 8, 8, 0}, 0x42, 2, FDP1_CAPTURE | FDP1_OUTPUT }, /* YCbCr4:2:0 semi (NV12, NV21) */
	{ V4L2_PIX_FMT_NV21,  { 8, 8, 0}, 0x42 | WPF_FORMAT_WSPUVS,
			2, FDP1_CAPTURE | FDP1_OUTPUT }, /* YCbCr4:2:0 semi (NV12, NV21) */

	//{ V4L2_PIX_FMT_,  {24, 0, 0}, 0x46, 1, FDP1_CAPTURE | FDP1_OUTPUT }, /* YCbCr4:4:4 interleaved */
	{ V4L2_PIX_FMT_UYVY,  {16, 0, 0}, 0x47, 1, FDP1_CAPTURE | FDP1_OUTPUT }, /* YCbCr4:2:2 interleaved type 0 */
	{ V4L2_PIX_FMT_YUYV,  {16, 0, 0}, 0x47 | WPF_FORMAT_WSPYCS,
			1, FDP1_CAPTURE | FDP1_OUTPUT }, /* RSPYCS=1 */
	{ V4L2_PIX_FMT_YVYU,  {16, 0, 0}, 0x47 | WPF_FORMAT_WSPYCS | WPF_FORMAT_WSPUVS,
			1, FDP1_CAPTURE | FDP1_OUTPUT }, /* RSPUVS=1 RSPYCS=1 */

	//{ V4L2_PIX_FMT_,  888, 0x48, 1, FDP1_CAPTURE | FDP1_OUTPUT }, /* YCbCr4:2:2 interleaved type 1 */

	/* V4L2_PIX_FMT_YVU4??M is the same except the Cr data is stored
	 * in the second plane and the Cb data in the third plane.  */

	/* Should be able to map Planar formats (contiguous single plane)
	 * to these fmt's too. Should be just a matter of getting the correct
	 * address to the hardware */
	{ V4L2_PIX_FMT_YUV444M,  { 8, 8, 8}, 0x4A, 3, FDP1_CAPTURE | FDP1_OUTPUT }, /* YCbCr4:4:4 */
	{ V4L2_PIX_FMT_YVU444M,  { 8, 8, 8}, 0x4A, 3, FDP1_CAPTURE | FDP1_OUTPUT },

	{ V4L2_PIX_FMT_YUV422M,  { 8, 4, 4}, 0x4B, 3, FDP1_CAPTURE | FDP1_OUTPUT }, /* YCbCr4:2:2 */
	{ V4L2_PIX_FMT_YVU422M,  { 8, 4, 4}, 0x4B, 3, FDP1_CAPTURE | FDP1_OUTPUT },
	/* 3 Planes in a contiguous buffer ...
	 * will have to verify that the hardware can support alignment ? */
	{ V4L2_PIX_FMT_YUV422P,  {16, 0, 0}, 0x4B, 1, FDP1_CAPTURE | FDP1_OUTPUT },

	{ V4L2_PIX_FMT_YUV420M,  { 8, 2, 2}, 0x4C, 3, FDP1_CAPTURE | FDP1_OUTPUT }, /* YCbCr4:2:0 */
	{ V4L2_PIX_FMT_YVU420M,  { 8, 2, 2}, 0x4C, 3, FDP1_CAPTURE | FDP1_OUTPUT },


};

#define NUM_FORMATS ARRAY_SIZE(formats)

/* FDP1 Lookup tables range from 0...255 only */
typedef unsigned char fdp1_lut;

fdp1_lut fdp1_diff_adj[256] = {
	0x00,0x24,0x43,0x5E,0x76,0x8C,0x9E,0xAF,
	0xBD,0xC9,0xD4,0xDD,0xE4,0xEA,0xEF,0xF3,
	0xF6,0xF9,0xFB,0xFC,0xFD,0xFE,0xFE,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

fdp1_lut fdp1_sad_adj[256] = {
	0x00,0x24,0x43,0x5E,0x76,0x8C,0x9E,0xAF,
	0xBD,0xC9,0xD4,0xDD,0xE4,0xEA,0xEF,0xF3,
	0xF6,0xF9,0xFB,0xFC,0xFD,0xFE,0xFE,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

fdp1_lut fdp1_bld_gain[256] = {
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80
};

fdp1_lut fdp1_dif_gain[256] = {
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80
};

fdp1_lut fdp1_mdet[256] = {
	0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
	0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
	0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,
	0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,
	0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,
	0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,
	0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,
	0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,
	0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,
	0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,
	0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,
	0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F,
	0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,
	0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,
	0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,
	0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,0x7F,
	0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,
	0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x8F,
	0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,
	0x98,0x99,0x9A,0x9B,0x9C,0x9D,0x9E,0x9F,
	0xA0,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,
	0xA8,0xA9,0xAA,0xAB,0xAC,0xAD,0xAE,0xAF,
	0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,
	0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF,
	0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,
	0xC8,0xC9,0xCA,0xCB,0xCC,0xCD,0xCE,0xCF,
	0xD0,0xD1,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,
	0xD8,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,0xDF,
	0xE0,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,
	0xE8,0xE9,0xEA,0xEB,0xEC,0xED,0xEE,0xEF,
	0xF0,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,
	0xF8,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF
};

/* Per-queue, driver-specific private data */
struct fdp1_q_data {
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

static struct fdp1_fmt *fdp1_find_format(u32 pixelformat,
					 unsigned int fmt_type)
{
	struct fdp1_fmt *fmt;
	unsigned int k;

	for (k = 0; k < NUM_FORMATS; k++) {
		fmt = &formats[k];
		if ((fmt->fourcc == pixelformat) && (fmt->types & fmt_type))
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

	void __iomem		*regs;
	unsigned int		irq;
	struct device		*dev;
	void			*alloc_ctx;
	struct timer_list	timer;

	struct rcar_fcp_device	*fcp;
	struct v4l2_m2m_dev	*m2m_dev;

	struct dentry 		*dbgroot;
	struct dentry		*regset_dentry;
	struct debugfs_regset32 regset;
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

static inline struct fdp1_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct fdp1_ctx, fh);
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

static u32 fdp1_read(struct fdp1_dev *fdp1, unsigned int reg)
{
	dprintk(fdp1, "Read from %p\n", fdp1->regs + reg);
	return ioread32(fdp1->regs + reg);
}

static void fdp1_write(struct fdp1_dev *fdp1, u32 val, unsigned int reg)
{
	dprintk(fdp1, "Write to %p\n", fdp1->regs + reg);
	iowrite32(val, fdp1->regs + reg);
}


void fdp1_print_regs32(struct fdp1_dev *fdp1)
{
	int i;
	const struct debugfs_reg32 *regs = fdp1->regset.regs;

	for (i = 0; i < fdp1->regset.nregs; i++, regs++)
		dprintk(fdp1, "%s = 0x%08x\n", regs->name,
			   readl(fdp1->regset.base + regs->offset));
}

struct fdp1_plane_addrs
{
	unsigned long plane0;
	unsigned long plane1;
	unsigned long plane2;
};

static struct fdp1_plane_addrs vb2_dc_to_pa(struct vb2_v4l2_buffer *buf,
		unsigned int planes)
{
	struct fdp1_plane_addrs pa = { 0 };
	struct vb2_buffer *vb2buf = &buf->vb2_buf;

	switch (planes)
	{
	case 3:	pa.plane2 = vb2_dma_contig_plane_dma_addr(vb2buf, 2);
		/* Fall through */
	case 2:	pa.plane1 = vb2_dma_contig_plane_dma_addr(vb2buf, 1);
		/* Fall through */
	case 1:	pa.plane0 = vb2_dma_contig_plane_dma_addr(vb2buf, 0);
		break;
	default:
		BUG_ON(1);
	}

	return pa;
}

/* IPC registers are to be programmed with constant values */
static void fdp1_set_ipc_dli(struct fdp1_ctx *ctx)
{
	struct fdp1_dev *fdp1 = ctx->fdp1;
	struct fdp1_q_data *src_q_data = &ctx->out_q;

	fdp1_write(fdp1, 0x00010002, IPC_SMSK_THRESH);
	fdp1_write(fdp1, 0x00200040, IPC_COMB_DET);
	fdp1_write(fdp1, 0x00008020, IPC_MOTDEC);

	fdp1_write(fdp1, 0x0080FF02, IPC_DLI_BLEND);
	fdp1_write(fdp1, 0x001000FF, IPC_DLI_HGAIN);
	fdp1_write(fdp1, 0x009004FF, IPC_DLI_SPRS);
	fdp1_write(fdp1, 0x0004080C, IPC_DLI_ANGLE);
	fdp1_write(fdp1, 0xFF10FF10, IPC_DLI_ISOPIX0);
	fdp1_write(fdp1, 0x0000FF10, IPC_DLI_ISOPIX1);
}


static void fdp1_set_ipc_sensor(struct fdp1_ctx *ctx)
{
	struct fdp1_dev *fdp1 = ctx->fdp1;
	struct fdp1_q_data *src_q_data = &ctx->out_q;
	unsigned int xe, ye;
	unsigned int x0, x1;
	unsigned int hsize = src_q_data->format.width;
	unsigned int vsize = src_q_data->format.height;

	return;

	fdp1_write(fdp1, 0x20208080, IPC_SENSOR_TH0);
	fdp1_write(fdp1, (2<<12)|(2<<8)|1, IPC_SENSOR_CTL0); // Tidy me up
	fdp1_write(fdp1, 0x00, IPC_SENSOR_CTL1);

	if (src_q_data->format.field != V4L2_FIELD_NONE)
		ye = (vsize * 2) - 1;
	else
		ye = vsize - 1;

	xe = src_q_data->format.width - 1;
	fdp1_write(fdp1, (xe << 16) | ye, IPC_SENSOR_CTL2); // Tidy me up

	x0 = hsize / 3;
	x1 = 2 * hsize / 3;

	fdp1_write(fdp1, (x0 << 16) | (x1), IPC_SENSOR_CTL3);
}

static int device_process(struct fdp1_ctx *ctx,
			  struct vb2_v4l2_buffer *src_buf,
			  struct vb2_v4l2_buffer *dst_buf)
{
	struct fdp1_dev *fdp1 = ctx->fdp1;
	struct fdp1_q_data *src_q_data = &ctx->out_q;
	struct fdp1_q_data *dst_q_data = &ctx->cap_q;

	struct fdp1_plane_addrs src_addr;
	struct fdp1_plane_addrs dst_addr;

	unsigned int opmode, ipcmode;
	unsigned int channels;
	unsigned int picture_size, stride_y, stride_c;
	unsigned int format;

	/* Obtain physical addresses for the HW */
	src_addr = vb2_dc_to_pa(src_buf, src_q_data->fmt->num_planes);
	dst_addr = vb2_dc_to_pa(dst_buf, dst_q_data->fmt->num_planes);

	src_buf->sequence = src_q_data->sequence++;
	dst_buf->sequence = dst_q_data->sequence++;

	dst_buf->vb2_buf.timestamp = src_buf->vb2_buf.timestamp;

	if (src_buf->flags & V4L2_BUF_FLAG_TIMECODE)
		dst_buf->timecode = src_buf->timecode;
	dst_buf->field = src_buf->field;
	dst_buf->flags = src_buf->flags &
		(V4L2_BUF_FLAG_TIMECODE |
		 V4L2_BUF_FLAG_KEYFRAME |
		 V4L2_BUF_FLAG_PFRAME |
		 V4L2_BUF_FLAG_BFRAME |
		 V4L2_BUF_FLAG_TSTAMP_SRC_MASK);

	dprintk(fdp1, "\n\n");

	/* Debug the world ... Lets see what we have going through */
	dprintk(fdp1, "SRC[%d]: 0x%08lx 0x%08lx 0x%08lx (%dx%d, 0x%x)\n",
			src_buf->vb2_buf.index,
			src_addr.plane0, src_addr.plane1, src_addr.plane2,
			src_q_data->format.width, src_q_data->format.height,
			src_q_data->fmt->fmt);
	dprintk(fdp1, "DST[%d]: 0x%08lx 0x%08lx 0x%08lx (%dx%d, 0x%x)\n",
			dst_buf->vb2_buf.index,
			dst_addr.plane0, dst_addr.plane1, dst_addr.plane2,
			dst_q_data->format.width, dst_q_data->format.height,
			dst_q_data->fmt->fmt);


	/* Non-immediate registers */

	/* First Frame only? ... */
	fdp1_write(fdp1, CTL_CLKCTRL_CSTP_N, CTL_CLKCTRL);

	/* Set the mode, and configuration */
	opmode = (CTL_OPMODE_PRG | CTL_OPMODE_INTERRUPT);
	channels = (CTL_CHACT_WR | CTL_CHACT_RD1);
	ipcmode = IPC_MODE_DLI | IPC_MODE_DIM_FIXED2D;

	fdp1_write(fdp1, channels,	CTL_CHACT);
	fdp1_write(fdp1, opmode,	CTL_OPMODE);
	fdp1_write(fdp1, ipcmode,	IPC_MODE);

	/* Configure clocking */
#define MHZ (1000*1000)
	fdp1_write(fdp1, (200*MHZ)/ 1 /*FPS*/,	CTL_VPERIOD);

	/* DLI Static Configuration */
	fdp1_set_ipc_dli(ctx);

	/* Sensor Configuration */
	fdp1_set_ipc_sensor(ctx);

	/* Enable All Interrupts */
	fdp1_write(fdp1, CTL_IRQ_MASK, CTL_IRQENB);
	dprintk(fdp1, "CycleStatus = %d\n", fdp1_read(fdp1, CTL_VCYCLE_STATUS));

	/* Setup the source picture */

	/* Picture size is common to Source AND Destination frames */
	picture_size = ((src_q_data->format.width & GENMASK(12,0)) << 16);
	picture_size |= (src_q_data->format.height & GENMASK(12,0));
	dprintk(fdp1, "RPF_SIZE: 0x%08x\n", picture_size);
	fdp1_write(fdp1, picture_size, RPF_SIZE);

	/* Stride Y */
	stride_y = src_q_data->format.width * src_q_data->fmt->bpp[0] / 8;
	/* Stride CbCr */
	stride_c = src_q_data->format.width * src_q_data->fmt->bpp[1] / 8;
	fdp1_write(fdp1, (stride_y << 16) | (stride_c & 0xFFFF), RPF_PSTRIDE );

	fdp1_write(fdp1, src_q_data->fmt->fmt, RPF_FORMAT);
	fdp1_write(fdp1, src_addr.plane0, RPF1_ADDR_Y);
	fdp1_write(fdp1, src_addr.plane1, RPF1_ADDR_C0);
	fdp1_write(fdp1, src_addr.plane2, RPF1_ADDR_C1);

	/* Setup the Dest Picture */

	/* Stride Y */
	stride_y = dst_q_data->format.width * dst_q_data->fmt->bpp[0] / 8;
	/* Stride CbCr */
	/* stride_c is not used for RGB formats, can be 0...? */
	stride_c = dst_q_data->format.width * dst_q_data->fmt->bpp[1] / 8;
	fdp1_write(fdp1, (stride_y << 16) | (stride_c & 0xFFFF), WPF_PSTRIDE );

	format = dst_q_data->fmt->fmt; /* Output Format Code */
	if (dst_q_data->fmt->fmt <= 0x1B) /* Last RGB fmt code */
		format |= WPF_FORMAT_CSC; /* Enable Colour Space conversion */

	fdp1_write(fdp1, format, WPF_FORMAT);

	if (format & WPF_FORMAT_CSC)
		dprintk(fdp1, "Output is RGB - CSC Enabled\n");

	fdp1_write(fdp1, dst_addr.plane0, WPF_ADDR_Y);
	fdp1_write(fdp1, dst_addr.plane1, WPF_ADDR_C0);
	fdp1_write(fdp1, dst_addr.plane2, WPF_ADDR_C1);

	/* Line Memory Pixel Number Register for linear access */
	fdp1_write(fdp1, 1024, IPC_LMEM);

	/* Finally, the Immediate Registers */

	/* Start the command */
	 /* set after all relevant registers are surely set except for
	  * REGEND and SGCMD	  */
	fdp1_write(fdp1, CTL_CMD_STRCMD, CTL_CMD);

	/* Register End */
	/* Registers will update to HW at next VINT */
	fdp1_write(fdp1, CTL_REGEND_REGEND, CTL_REGEND);

	/* Enable VINT Generator */
	fdp1_write(fdp1, CTL_SGCMD_SGEN, CTL_SGCMD);



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
	bytesperline	= (q_data->width * q_data->fmt->bpp[0]) >> 3;

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

	/* Immediate abort sequence */
	fdp1_write(ctx->fdp1, 0, CTL_SGCMD);
	fdp1_write(ctx->fdp1, CTL_SRESET_SRST, CTL_SRESET);
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

#ifdef QEMU_TESTING
	/* Run a timer, which simulates a hardware irq  */
	schedule_irq(fdp1, ctx->transtime);
#endif
}

static void device_isr(unsigned long priv)
{
	struct fdp1_dev *fdp1 = (struct fdp1_dev *)priv;
	struct fdp1_ctx *curr_ctx;
	struct vb2_v4l2_buffer *src_vb, *dst_vb;
	unsigned long flags;

	curr_ctx = v4l2_m2m_get_curr_priv(fdp1->m2m_dev);

	if (NULL == curr_ctx) {
		pr_err("Instance released before the end of transaction\n");
		return;
	}

	src_vb = v4l2_m2m_src_buf_remove(curr_ctx->fh.m2m_ctx);
	dst_vb = v4l2_m2m_dst_buf_remove(curr_ctx->fh.m2m_ctx);

	curr_ctx->num_processed++;

	spin_lock_irqsave(&fdp1->irqlock, flags);
	v4l2_m2m_buf_done(src_vb, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst_vb, VB2_BUF_STATE_DONE);
	spin_unlock_irqrestore(&fdp1->irqlock, flags);

	if (curr_ctx->num_processed == curr_ctx->translen
	    || curr_ctx->aborting) {
		dprintk(curr_ctx->fdp1, "Finishing transaction\n");
		curr_ctx->num_processed = 0;
		v4l2_m2m_job_finish(fdp1->m2m_dev, curr_ctx->fh.m2m_ctx);
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
	strncpy(cap->driver, DRIVER_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, DRIVER_NAME, sizeof(cap->card) - 1);
	snprintf(cap->bus_info, sizeof(cap->bus_info),
			"platform:%s", DRIVER_NAME);
	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
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

static int fdp1_enum_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, FDP1_CAPTURE);
}

static int fdp1_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, FDP1_OUTPUT);
}

static int fdp1_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct fdp1_q_data *q_data;
	struct fdp1_ctx *ctx = fh_to_ctx(priv);

	if (!v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type))
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);
	f->fmt.pix_mp = q_data->format;

	return 0;
}

static int __fdp1_try_fmt(struct fdp1_ctx *ctx, struct fdp1_fmt **fmtinfo,
			  struct v4l2_pix_format_mplane *pix,
			  enum v4l2_buf_type type)
{
	struct fdp1_fmt *fmt;
	unsigned int fmt_type;
	unsigned int i, bpl = 0;

	fmt_type = V4L2_TYPE_IS_OUTPUT(type) ? FDP1_OUTPUT : FDP1_CAPTURE;

	fmt = fdp1_find_format(pix->pixelformat, fmt_type);
	if (!fmt) {
		dev_dbg(ctx->fdp1->dev, "unknown format; set default format\n");
		/* YUV Type compatible with both OUTPUT/CAPTURE */
		fmt = fdp1_find_format(V4L2_PIX_FMT_YUV420M, fmt_type);
	}

	pix->pixelformat = fmt->fourcc;
	// TODO: pix->colorspace = fmt->colorspace;
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = fmt->num_planes;
	memset(pix->reserved, 0, sizeof(pix->reserved));

	pix->width = clamp_t(unsigned int, pix->width, MIN_W, MAX_W);
	pix->height = clamp_t(unsigned int, pix->height, MIN_H, MAX_H);

	for (i = 0; i < pix->num_planes; ++i)
		bpl = max(bpl, pix->plane_fmt[i].bytesperline);

	bpl = clamp_t(unsigned int, bpl, pix->width, MAX_W);
	bpl = round_up(bpl, MEMALIGN);

	for (i = 0; i < pix->num_planes; ++i) {
		pix->plane_fmt[i].bytesperline = bpl;
		/* Todo: This just doesn't sound right below ...
		 * Why would each plane be based upon the full height/bpl + bpp?
		 * Maybe I'm just too sleepy now!
		 */
		pix->plane_fmt[i].sizeimage = bpl * pix->height * fmt->bpp[i] / 8;
		memset(pix->plane_fmt[i].reserved, 0,
		       sizeof(pix->plane_fmt[i].reserved));
	}

	if (fmtinfo)
		*fmtinfo = fmt;

	return 0;
}

static int fdp1_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct fdp1_fmt *fmt;
	struct fdp1_ctx *ctx = fh_to_ctx(priv);

	if (!v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type))
		return -EINVAL;

	return __fdp1_try_fmt(ctx, NULL, &f->fmt.pix_mp, f->type);
}

static int fdp1_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct fdp1_ctx *ctx = fh_to_ctx(priv);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct fdp1_q_data *q_data;
	struct fdp1_fmt *fmtinfo;
	int ret;

	vq = v4l2_m2m_get_vq(m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		v4l2_err(&ctx->fdp1->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	ret = __fdp1_try_fmt(ctx, &fmtinfo, &f->fmt.pix_mp, f->type);
	if (ret < 0)
		return ret;

	q_data = get_q_data(ctx, f->type);
	q_data->format = f->fmt.pix_mp;
	q_data->fmt = fmtinfo;

	dprintk(ctx->fdp1,
		"Setting format for type %d, wxh: %dx%d, fmt: %4s (%d)\n",
			f->type, q_data->format.width, q_data->format.height,
			(char*)&q_data->fmt->fourcc, q_data->fmt->fourcc);

	return 0;
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

	.vidioc_enum_fmt_vid_cap_mplane = fdp1_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out_mplane = fdp1_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_cap_mplane	= fdp1_g_fmt,
	.vidioc_g_fmt_vid_out_mplane	= fdp1_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane	= fdp1_try_fmt,
	.vidioc_try_fmt_vid_out_mplane	= fdp1_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane	= fdp1_s_fmt,
	.vidioc_s_fmt_vid_out_mplane	= fdp1_s_fmt,

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
	unsigned int i;

	q_data = get_q_data(ctx, vq->type);

	if (*nplanes) {
		if (*nplanes != q_data->format.num_planes)
			return -EINVAL;

		for (i = 0; i < *nplanes; i++) {
			unsigned int q_size = q_data->format.plane_fmt[i].sizeimage;

			if (sizes[i] < q_size)
				return -EINVAL;

			alloc_ctxs[i] = ctx->fdp1->alloc_ctx;
		}
		return 0;
	}

	*nplanes = q_data->format.num_planes;

	for (i = 0; i < *nplanes; i++) {
		sizes[i] = q_data->format.plane_fmt[i].sizeimage;
		alloc_ctxs[i] = ctx->fdp1->alloc_ctx;
	}

	dprintk(ctx->fdp1, "get %d buffer(s) of size [%d,%d,%d] each.\n",
			*nbuffers,
			sizes[0],
			*nplanes > 1 ? sizes[1] : 0,
			*nplanes > 2 ? sizes[2] : 0);

	return 0;
}

static int fdp1_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct fdp1_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct fdp1_q_data *q_data;
	unsigned int i;

	dprintk(ctx->fdp1, "type: %d\n", vb->vb2_queue->type);

	q_data = get_q_data(ctx, vb->vb2_queue->type);

	/* We only support progressive CAPTURE */
	if (!V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (vbuf->field == V4L2_FIELD_ANY)
			vbuf->field = V4L2_FIELD_NONE;
		if (vbuf->field != V4L2_FIELD_NONE) {
			dprintk(ctx->fdp1,
				"%s field isn't supported on capture\n",
				__func__);
			return -EINVAL;
		}
	}

	for (i = 0; i < q_data->format.num_planes; i++) {
		unsigned long size = q_data->format.plane_fmt[i].sizeimage;

		if (vb2_plane_size(vb, i) < size) {
			dev_err(ctx->fdp1->dev,
				"%s: data will not fit into plane [%d/%d] (%lu < %lu)\n",
			       __func__,
			       i, q_data->format.num_planes,
			       vb2_plane_size(vb, i), size);
			return -EINVAL;
		}

		/* We have known size formats all around */
		vb2_set_plane_payload(vb, i, size);
	}

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
	src_vq->mem_ops = &vb2_dma_contig_memops;
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
	dst_vq->mem_ops = &vb2_dma_contig_memops;
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

	/* Initialise controls */

	ctx->transtime = 40;
	ctx->translen = 1;

	v4l2_ctrl_handler_init(&ctx->hdl, 4);
	v4l2_ctrl_new_std(&ctx->hdl, &fdp1_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ctx->hdl, &fdp1_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_custom(&ctx->hdl, &fdp1_ctrl_trans_time_msec, NULL);
	v4l2_ctrl_new_custom(&ctx->hdl, &fdp1_ctrl_trans_num_bufs, NULL);
	if (ctx->hdl.error) {
		rc = ctx->hdl.error;
		v4l2_ctrl_handler_free(&ctx->hdl);
		goto open_unlock;
	}

	ctx->fh.ctrl_handler = &ctx->hdl; // don't set the handle unless it's registered!
	v4l2_ctrl_handler_setup(&ctx->hdl);

	/* Configure default parameters. */
	__fdp1_try_fmt(ctx, &ctx->out_q.fmt, &ctx->out_q.format,
		      V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	__fdp1_try_fmt(ctx, &ctx->cap_q.fmt, &ctx->cap_q.format,
		      V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	ctx->colorspace = V4L2_COLORSPACE_REC709;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(fdp1->m2m_dev, ctx, &queue_init);

	if (IS_ERR(ctx->fh.m2m_ctx)) {
		rc = PTR_ERR(ctx->fh.m2m_ctx);

		v4l2_ctrl_handler_free(&ctx->hdl);
		kfree(ctx);
		goto open_unlock;
	}

	dprintk(fdp1, "pm_runtime_get_sync(fdp1->dev:0x%p)\n", fdp1->dev);
	pm_runtime_get_sync(fdp1->dev);

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
	struct fdp1_ctx *ctx = fh_to_ctx(file->private_data);

	dprintk(fdp1, "Releasing instance %p\n", ctx);

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->hdl);
	mutex_lock(&fdp1->dev_mutex);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&fdp1->dev_mutex);
	kfree(ctx);

	atomic_dec(&fdp1->num_inst);

	dprintk(fdp1, "pm_runtime_put(fdp1->dev:0x%p)\n", fdp1->dev);
	pm_runtime_put(fdp1->dev);

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
	.name		= DRIVER_NAME,
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

static irqreturn_t fdp1_irq_handler(int irq, void *dev_id)
{
	struct fdp1_dev *fdp1 = dev_id;
	struct fdp1_ctx *ctx;

	unsigned int int_status, scratch;

	int_status = fdp1_read(fdp1, CTL_IRQSTA);

	dprintk(fdp1, "IRQ: 0x%x %s%s%s\n", int_status,
			int_status & CTL_IRQ_VERE ? "[Error]" : "[!E]",
			int_status & CTL_IRQ_VINTE ? "[VSync]" : "[!V]",
			int_status & CTL_IRQ_FREE ? "[FrameEnd]" : "[!F]");

	dprintk(fdp1, "CycleStatus = %d\n", fdp1_read(fdp1, CTL_VCYCLE_STATUS));

	scratch = fdp1_read(fdp1, CTL_STATUS);
	dprintk(fdp1, "Control Status = 0x%08x : VINT_CNT = %d %s:%s:%s:%s\n",
			scratch, (scratch >> 16),
			scratch & CTL_STATUS_SGREGSET ? "RegSet" : "",
			scratch & CTL_STATUS_SGVERR ? "Vsync Error" : "",
			scratch & CTL_STATUS_SGFREND ? "FrameEnd" : "",
			scratch & CTL_STATUS_BSY ? "Busy.." : "");
	dprintk(fdp1, "***********************************\n");
	fdp1_print_regs32(fdp1);
	dprintk(fdp1, "***********************************\n");

	/* Spurious interrupt */
	if (!((CTL_IRQ_MASK) & int_status))
		return IRQ_NONE;

	/* Clear interrupts */
	fdp1_write(fdp1, ~(int_status & CTL_IRQ_MASK), CTL_IRQSTA);

	/* Work completed Release the frames ... */
	//if ((CTL_IRQ_VERE | CTL_IRQ_FREE) & int_status)
		device_isr((unsigned long)fdp1);

	return IRQ_HANDLED;
}

static int fdp1_debugfs_init(struct fdp1_dev *fdp1)
{

	/* Debug FS Regset registration */
	fdp1->regset.base = fdp1->regs;
	fdp1->regset.regs = fdp1_regset;
	fdp1->regset.nregs = NUM_FDP1_REGSETS;

	fdp1->dbgroot = debugfs_create_dir(fdp1->v4l2_dev.name, NULL);
	if (!fdp1->dbgroot)
		return -ENOMEM;

	fdp1->regset_dentry = debugfs_create_regset32("regs",
			S_IRUGO, fdp1->dbgroot, &fdp1->regset);

        if (!fdp1->regset_dentry) {
                debugfs_remove_recursive(fdp1->dbgroot);
                return -ENOMEM;
        }

	return 0;
}

static void fdp1_debugfs_remove(struct fdp1_dev *fdp1)
{
	debugfs_remove_recursive(fdp1->dbgroot);
}

static int fdp1_probe(struct platform_device *pdev)
{
	struct fdp1_dev *fdp1;
	struct video_device *vfd;
	struct device_node *fcp_node;
	struct resource *res;
	int ret;
	int hw_version;

	fdp1 = devm_kzalloc(&pdev->dev, sizeof(*fdp1), GFP_KERNEL);
	if (!fdp1)
		return -ENOMEM;

	spin_lock_init(&fdp1->irqlock);
	fdp1->dev = &pdev->dev;
	platform_set_drvdata(pdev, fdp1);

#ifndef QEMU_TESTING
	/* memory-mapped registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fdp1->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fdp1->regs))
		return PTR_ERR(fdp1->regs);

	/* interrupt service routine registration */
	fdp1->irq = ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, fdp1->irq, fdp1_irq_handler, 0,
			       dev_name(&pdev->dev), fdp1);
	if (ret) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", fdp1->irq);
		return ret;
	}

	/* FCP (optional) */
	fcp_node = of_parse_phandle(pdev->dev.of_node, "renesas,fcp", 0);
	if (fcp_node) {
		dprintk(fdp1, "Found an FCP Node Looking for Device\n");
		fdp1->fcp = rcar_fcp_get(fcp_node);
		of_node_put(fcp_node);
		if (IS_ERR(fdp1->fcp)) {
			dev_err(&pdev->dev, "FCP not found (%ld)\n",
				PTR_ERR(fdp1->fcp));
			return PTR_ERR(fdp1->fcp);
		}
	}
#endif

	/* Bring the device up ready for reading registers */
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	dprintk(fdp1, "**********************************\n");

#ifndef QEMU_TESTING
	hw_version = fdp1_read(fdp1, IP_INTDATA);
	switch (hw_version)
	{
	case IP_H3:
		dprintk(fdp1, "FDP1 Version R-Car H3\n");
		break;
	case IP_M3W:
		dprintk(fdp1, "FDP1 Version R-Car M3-W\n");
		break;
	default:
		dprintk(fdp1, "FDP1 Version unidentified: 0x%08x\n", hw_version);
		break;
	}

#endif
	ret = v4l2_device_register(&pdev->dev, &fdp1->v4l2_dev);
	if (ret)
		return ret;

	atomic_set(&fdp1->num_inst, 0);
	mutex_init(&fdp1->dev_mutex);

	fdp1->vfd = fdp1_videodev;
	vfd = &fdp1->vfd;
	vfd->lock = &fdp1->dev_mutex;
	vfd->v4l2_dev = &fdp1->v4l2_dev;


	/* Memory allocation contexts */
	fdp1->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(fdp1->alloc_ctx)) {
		v4l2_err(&fdp1->v4l2_dev, "Failed to init memory allocator\n");
		ret = PTR_ERR(fdp1->alloc_ctx);
		goto unreg_dev;
	}


	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
	if (ret) {
		v4l2_err(&fdp1->v4l2_dev, "Failed to register video device\n");
		goto vb2_allocator_rollback;
	}

	video_set_drvdata(vfd, fdp1);
	snprintf(vfd->name, sizeof(vfd->name), "%s", fdp1_videodev.name);
	v4l2_info(&fdp1->v4l2_dev,
			"Device registered as /dev/video%d\n", vfd->num);


	setup_timer(&fdp1->timer, device_isr, (long)fdp1);

	fdp1->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(fdp1->m2m_dev)) {
		v4l2_err(&fdp1->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(fdp1->m2m_dev);
		goto err_m2m;
	}

	/* Register debug fs entries */
	fdp1_debugfs_init(fdp1);

	pm_runtime_put(&pdev->dev);

	dprintk(fdp1, "------------------------------------------\n");

	return 0;

err_m2m:
	v4l2_m2m_release(fdp1->m2m_dev);
	video_unregister_device(&fdp1->vfd);

vb2_allocator_rollback:
	vb2_dma_contig_cleanup_ctx(fdp1->alloc_ctx);

unreg_dev:
	v4l2_device_unregister(&fdp1->v4l2_dev);

	dprintk(fdp1, ":-( ??????????????????????????????????????\n");

	return ret;
}

static int fdp1_remove(struct platform_device *pdev)
{
	struct fdp1_dev *fdp1 = platform_get_drvdata(pdev);

	v4l2_info(&fdp1->v4l2_dev, "Removing " DRIVER_NAME);
	fdp1_debugfs_remove(fdp1);
	v4l2_m2m_release(fdp1->m2m_dev);
	del_timer_sync(&fdp1->timer);
	video_unregister_device(&fdp1->vfd);
	v4l2_device_unregister(&fdp1->v4l2_dev);
	vb2_dma_contig_cleanup_ctx(fdp1->alloc_ctx);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static int fdp1_pm_runtime_suspend(struct device *dev)
{
	struct fdp1_dev *fdp1 = dev_get_drvdata(dev);

	rcar_fcp_disable(fdp1->fcp);

	return 0;
}

static int fdp1_pm_runtime_resume(struct device *dev)
{
	struct fdp1_dev *fdp1 = dev_get_drvdata(dev);

	return rcar_fcp_enable(fdp1->fcp);
}

static const struct dev_pm_ops fdp1_pm_ops = {
	SET_RUNTIME_PM_OPS(fdp1_pm_runtime_suspend, fdp1_pm_runtime_resume, NULL)
};

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
		.name	= DRIVER_NAME,
		.of_match_table = fdp1_dt_ids,
		.pm	= &fdp1_pm_ops,
	},
};

module_platform_driver(fdp1_pdrv);

MODULE_DESCRIPTION("Renesas R-Car Fine Display Processor");
MODULE_AUTHOR("Kieran Bingham, <kieran@bingham.xyz>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
MODULE_ALIAS("platform:" DRIVER_NAME);
