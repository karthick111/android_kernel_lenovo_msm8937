/*
 * linux/sound/soc/codecs/tiload.h
 *
 *
 * Copyright (C) 2011 Mistral Solutions Pvt Ltd.
 *               2015 Texas Instruments Inc.
 *
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 *
 *
 *
 */

#ifndef _TILOAD_H
#define _TILOAD_H

#include <sound/soc.h>
#include "tas2555-misc.h"

#define BPR_REG(book, page, reg)		(((book * 256 * 128) + \
						 (page * 128)) + reg)

/* typedefs required for the included header files */
typedef char *string;

typedef struct {
	unsigned char nBook;
	unsigned char nPage;
	unsigned char nRegister;
} BPR;

/* defines */
#define DEVICE_NAME     "tiload_node"
#define TILOAD_IOC_MAGIC   0xE0
#define TILOAD_IOMAGICNUM_GET  _IOR(TILOAD_IOC_MAGIC, 1, int)
#define TILOAD_IOMAGICNUM_SET  _IOW(TILOAD_IOC_MAGIC, 2, int)
#define TILOAD_BPR_READ _IOR(TILOAD_IOC_MAGIC, 3, BPR)
#define TILOAD_BPR_WRITE _IOW(TILOAD_IOC_MAGIC, 4, BPR)
#define TILOAD_IOCTL_SET_CONFIG _IOW(TILOAD_IOC_MAGIC, 5, int)


int tiload_driver_init(struct tas2555_priv *pTAS2555);

#endif
