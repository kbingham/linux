/*
 * rcar-fcp.h  --  R-Car Frame Compression Processor Driver
 *
 * Copyright (C) 2016 Renesas Electronics Corporation
 *
 * Contact: Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __MEDIA_RCAR_FCP_H__
#define __MEDIA_RCAR_FCP_H__

struct device_node;
struct rcar_fcp_device;

#define FCP_VCR		0x0000
#define FCP_RST		0x0010
#define FCP_STA		0x0018

#if IS_ENABLED(CONFIG_VIDEO_RENESAS_FCP)
struct rcar_fcp_device *rcar_fcp_get(const struct device_node *np);
void rcar_fcp_put(struct rcar_fcp_device *fcp);
int rcar_fcp_enable(struct rcar_fcp_device *fcp);
void rcar_fcp_disable(struct rcar_fcp_device *fcp);
void __iomem * rcar_fcp_regs(struct rcar_fcp_device *fcp);
u32 rcar_fcp_read(struct rcar_fcp_device *fcp, unsigned int reg);
#else
static inline struct rcar_fcp_device *rcar_fcp_get(const struct device_node *np)
{
	return ERR_PTR(-ENOENT);
}
static inline void rcar_fcp_put(struct rcar_fcp_device *fcp) { }
static inline int rcar_fcp_enable(struct rcar_fcp_device *fcp)
{
	return -ENOSYS;
}
static inline void rcar_fcp_disable(struct rcar_fcp_device *fcp) { }
static inline void __iomem * rcar_fcp_regs(struct rcar_fcp_device *fcp)
{
	return NULL;
}
static u32 rcar_fcp_read(struct rcar_fcp_device *fcp, unsigned int reg)
{
	return 0;
}
#endif

#endif /* __MEDIA_RCAR_FCP_H__ */
