// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/ion.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <linux/crc32.h>
#include "virtio_fastrpc_core.h"
#include "virtio_fastrpc_mem.h"
#include "virtio_fastrpc_queue.h"

#define M_FDLIST			16
#define M_CRCLIST			64
#define M_DSP_PERF_LIST		12
#define M_KERNEL_PERF_LIST (PERF_KEY_MAX)
#define M_DSP_PERF_LIST		12

#define FASTRPC_DMAHANDLE_NOMAP	16

#define VIRTIO_FASTRPC_CMD_OPEN			1
#define VIRTIO_FASTRPC_CMD_CLOSE		2
#define VIRTIO_FASTRPC_CMD_INVOKE		3
#define VIRTIO_FASTRPC_CMD_MMAP			4
#define VIRTIO_FASTRPC_CMD_MUNMAP		5
#define VIRTIO_FASTRPC_CMD_CONTROL		6
#define VIRTIO_FASTRPC_CMD_GET_DSP_INFO	7

#define STATIC_PD			0
#define DYNAMIC_PD			1
#define GUEST_OS			2

#define FASTRPC_STATIC_HANDLE_KERNEL	1
#define FASTRPC_STATIC_HANDLE_LISTENER	3
#define FASTRPC_STATIC_HANDLE_MAX	20

#define UNSIGNED_PD_SUPPORT 1
#define PERF_CAPABILITY   (1 << 1)

#define PERF_END (void)0

#define PERF(enb, cnt, ff) \
	{\
		struct timespec64 startT = {0};\
		uint64_t *counter = cnt;\
		if (enb && counter) {\
			ktime_get_real_ts64(&startT);\
		} \
		ff ;\
		if (enb && counter) {\
			*counter += getnstimediff(&startT);\
		} \
	}

#define GET_COUNTER(perf_ptr, offset)  \
	(perf_ptr != NULL ?\
		(((offset >= 0) && (offset < PERF_KEY_MAX)) ?\
			(uint64_t *)(perf_ptr + offset)\
				: (uint64_t *)NULL) : (uint64_t *)NULL)

enum fastrpc_perfkeys {
	PERF_COUNT = 0,
	PERF_FLUSH = 1,
	PERF_MAP = 2,
	PERF_COPY = 3,
	PERF_LINK = 4,
	PERF_GETARGS = 5,
	PERF_PUTARGS = 6,
	PERF_INVARGS = 7,
	PERF_INVOKE = 8,
	PERF_TID = 9,
	PERF_KEY_MAX = 10,
};

enum virtio_fastrpc_invoke_attr {
	/* bit0, 1: FE/BE crc enabled, 0: FE/BE crc disabled */
	VIRTIO_FASTRPC_INVOKE_CRC = 1 << 0,
	VIRTIO_FASTRPC_INVOKE_PERF = 1 << 1,
	VIRTIO_FASTRPC_INVOKE_ASYNC = 1 << 2,
};

enum fastrpc_buf_attr {
	/* bit0, 1: ION buffer, 0: normal buffer*/
	FASTRPC_BUF_ATTR_ION = 1 << 0,
	/* bit1, 1: buffer data is in an intermediate buffer, 0: buffer data is in invoke message */
	FASTRPC_BUF_ATTR_INTERNAL = 1 << 1,
	/* bit2, 1: cached buffer, 0: uncached buffer */
	FASTRPC_BUF_ATTR_CACHED = 1 << 2,
};

static uint32_t kernel_capabilities[FASTRPC_MAX_ATTRIBUTES -
FASTRPC_MAX_DSP_ATTRIBUTES] = {
	PERF_CAPABILITY	/* PERF_LOGGING_V2_SUPPORT feature is supported, unsupported = 0 */
};

struct virt_fastrpc_buf {
	u32 attrs;
	u32 crc;
	u64 pv;	/* buffer virtual address */
	u64 buf_len;	/* buffer length */
	u64 offset;	/* buffer offset */
	u64 payload_len;	/* payload length */
} __packed;

struct virt_fastrpc_sgl {
	u64 pv;		/* buffer physical address*/
	u64 len;	/* buffer length */
};

struct virt_fastrpc_dmahandle {
	u32 fd;
	u32 offset;
};

struct virt_open_msg {
	struct virt_msg_hdr hdr;	/* virtio fastrpc message header */
	u32 domain;			/* DSP domain id */
	u32 pd;				/* DSP PD */
	u32 attrs;			/* DSP PD attributes */
} __packed;

struct virt_cap_msg {
	struct virt_msg_hdr hdr;	/* virtio fastrpc message header */
	u32 domain;		/* DSP domain id */
	u32 dsp_caps[FASTRPC_MAX_DSP_ATTRIBUTES];	/* DSP capability */
} __packed;

struct virt_control_msg {
	struct virt_msg_hdr hdr;	/* virtio fastrpc message header */
	u32 enable;			/* latency control enable */
	u32 latency;			/* latency value */
} __packed;

struct virt_invoke_msg {
	struct virt_msg_hdr hdr;	/* virtio fastrpc message header */
	u32 handle;			/* remote handle */
	u32 sc;				/* scalars describing the data */
	u32 attrs;
	struct virt_fastrpc_buf pra[0];	/* remote arguments list */
} __packed;

struct virt_mmap_msg {
	struct virt_msg_hdr hdr;	/* virtio fastrpc message header */
	u32 nents;                      /* number of map entries */
	u32 flags;			/* mmap flags */
	u64 size;			/* mmap length */
	u64 vapp;			/* application virtual address */
	u64 vdsp;			/* dsp address */
	struct virt_fastrpc_sgl sgl[0]; /* sg list */
} __packed;

struct virt_munmap_msg {
	struct virt_msg_hdr hdr;	/* virtio fastrpc message header */
	u64 vdsp;			/* dsp address */
	u64 size;			/* mmap length */
} __packed;

static inline uint64_t ptr_to_uint64(void *ptr)
{
	uint64_t addr = (uint64_t)((uintptr_t)ptr);
	return addr;
}

static inline int64_t getnstimediff(struct timespec64 *start)
{
	int64_t ns;
	struct timespec64 ts, b;

	ktime_get_real_ts64(&ts);
	b = timespec64_sub(ts, *start);
	ns = timespec64_to_ns(&b);
	return ns;
}

enum fastrpc_proc_attr {
	/* Macro for Debug attr */
	FASTRPC_MODE_DEBUG	= 1 << 0,
	/* Macro for Ptrace */
	FASTRPC_MODE_PTRACE	= 1 << 1,
	/* Macro for CRC Check */
	FASTRPC_MODE_CRC	= 1 << 2,
	/* Macro for Unsigned PD */
	FASTRPC_MODE_UNSIGNED_MODULE	= 1 << 3,
	/* Macro for Adaptive QoS */
	FASTRPC_MODE_ADAPTIVE_QOS	= 1 << 4,
	/* Macro for System Process */
	FASTRPC_MODE_SYSTEM_PROCESS	= 1 << 5,
	/* Macro for Prvileged Process */
	FASTRPC_MODE_PRIVILEGED	= (1 << 6),
};

static struct virt_fastrpc_msg *virt_alloc_msg(struct fastrpc_file *fl, int size)
{
	struct fastrpc_apps *me = fl->apps;
	struct virt_fastrpc_msg *msg;
	void *buf;
	unsigned long flags;
	int i;

	if (size > me->buf_size) {
		dev_err(me->dev, "message is too big (%d)\n", size);
		return NULL;
	}

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return NULL;

	buf = get_a_tx_buf(fl);
	if (!buf) {
		dev_err(me->dev, "can't get tx buffer\n");
		kfree(msg);
		return NULL;
	}

	msg->txbuf = buf;
	init_completion(&msg->work);
	spin_lock_irqsave(&me->msglock, flags);
	for (i = 0; i < FASTRPC_MSG_MAX; i++) {
		if (!me->msgtable[i]) {
			me->msgtable[i] = msg;
			msg->msgid = i;
			break;
		}
	}
	spin_unlock_irqrestore(&me->msglock, flags);

	if (i == FASTRPC_MSG_MAX) {
		dev_err(me->dev, "message queue is full\n");
		kfree(msg);
		return NULL;
	}
	return msg;
}

static void virt_free_msg(struct fastrpc_file *fl, struct virt_fastrpc_msg *msg)
{
	struct fastrpc_apps *me = fl->apps;
	unsigned long flags;

	spin_lock_irqsave(&me->msglock, flags);
	if (me->msgtable[msg->msgid] == msg)
		me->msgtable[msg->msgid] = NULL;
	else
		dev_err(me->dev, "can't find msg %d in table\n", msg->msgid);
	spin_unlock_irqrestore(&me->msglock, flags);

	kfree(msg);
}

static void context_list_ctor(struct fastrpc_ctx_lst *me)
{
	INIT_HLIST_HEAD(&me->interrupted);
	INIT_HLIST_HEAD(&me->pending);
}

struct fastrpc_file *fastrpc_file_alloc(void)
{
	int err = 0;
	struct fastrpc_file *fl = NULL;

	VERIFY(err, NULL != (fl = kzalloc(sizeof(*fl), GFP_KERNEL)));
	if (err)
		return NULL;
	context_list_ctor(&fl->clst);
	spin_lock_init(&fl->hlock);
	INIT_HLIST_HEAD(&fl->maps);
	INIT_HLIST_HEAD(&fl->cached_bufs);
	INIT_HLIST_HEAD(&fl->remote_bufs);
	fl->tgid = current->tgid;
	fl->tgid_open = current->tgid;
	fl->mode = FASTRPC_MODE_SERIAL;
	fl->domain = -1;
	fl->cid = -1;
	fl->dsp_proc_init = 0;
	mutex_init(&fl->map_mutex);
	return fl;
}

static void context_free(struct fastrpc_invoke_ctx *ctx)
{
	int i;
	struct fastrpc_file *fl = ctx->fl;
	struct fastrpc_apps *me = fl->apps;
	struct virt_invoke_msg *rsp = NULL;
	int nbufs = REMOTE_SCALARS_INBUFS(ctx->sc) +
		REMOTE_SCALARS_OUTBUFS(ctx->sc);

	spin_lock(&fl->hlock);
	hlist_del_init(&ctx->hn);
	spin_unlock(&fl->hlock);

	mutex_lock(&fl->map_mutex);
	for (i = 0; i < nbufs; i++)
		fastrpc_mmap_free(fl, ctx->maps[i], 0);
	mutex_unlock(&fl->map_mutex);

	if (ctx->msg) {
		rsp = ctx->msg->rxbuf;
		if (rsp)
			fastrpc_rxbuf_send(fl, rsp, me->buf_size);

		virt_free_msg(fl, ctx->msg);
		ctx->msg = NULL;
	}
	if (ctx->desc) {
		for (i = 0; i < nbufs; i++) {
			if (ctx->desc[i].buf)
				fastrpc_buf_free(ctx->desc[i].buf, 1);
		}
		kfree(ctx->desc);
		ctx->desc = NULL;
	}

	if (fl->profile)
		kfree(ctx->perf);

	kfree(ctx);
}

static void fastrpc_context_list_dtor(struct fastrpc_file *fl)
{
	struct fastrpc_ctx_lst *clst = &fl->clst;
	struct fastrpc_invoke_ctx *ictx = NULL, *ctxfree;
	struct hlist_node *n;

	do {
		ctxfree = NULL;
		spin_lock(&fl->hlock);
		hlist_for_each_entry_safe(ictx, n, &clst->interrupted, hn) {
			hlist_del_init(&ictx->hn);
			ctxfree = ictx;
			break;
		}
		spin_unlock(&fl->hlock);
		if (ctxfree)
			context_free(ctxfree);
	} while (ctxfree);
	do {
		ctxfree = NULL;
		spin_lock(&fl->hlock);
		hlist_for_each_entry_safe(ictx, n, &clst->pending, hn) {
			hlist_del_init(&ictx->hn);
			ctxfree = ictx;
			break;
		}
		spin_unlock(&fl->hlock);
		if (ctxfree)
			context_free(ctxfree);
	} while (ctxfree);
}

static void fastrpc_remote_buf_list_free(struct fastrpc_file *fl)
{
	struct fastrpc_buf *buf, *free;

	do {
		struct hlist_node *n;

		free = NULL;
		spin_lock(&fl->hlock);
		hlist_for_each_entry_safe(buf, n, &fl->remote_bufs, hn_rem) {
			free = buf;
			break;
		}
		spin_unlock(&fl->hlock);
		if (free)
			fastrpc_buf_free(free, 0);
	} while (free);
}

static void fastrpc_cached_buf_list_free(struct fastrpc_file *fl)
{
	struct fastrpc_buf *buf, *free;

	do {
		struct hlist_node *n;

		free = NULL;
		spin_lock(&fl->hlock);
		hlist_for_each_entry_safe(buf, n, &fl->cached_bufs, hn) {
			hlist_del_init(&buf->hn);
			free = buf;
			break;
		}
		spin_unlock(&fl->hlock);
		if (free)
			fastrpc_buf_free(free, 0);
	} while (free);
}

static int virt_fastrpc_close(struct fastrpc_file *fl)
{
	struct fastrpc_apps *me = fl->apps;
	struct virt_msg_hdr *vmsg, *rsp = NULL;
	struct virt_fastrpc_msg *msg;
	int err;

	if (fl->cid < 0) {
		dev_err(me->dev, "close: channel id %d is invalid\n", fl->cid);
		return -EINVAL;
	}

	msg = virt_alloc_msg(fl, sizeof(*vmsg));
	if (!msg) {
		dev_err(me->dev, "%s: no memory\n", __func__);
		return -ENOMEM;
	}

	vmsg = (struct virt_msg_hdr *)msg->txbuf;
	vmsg->pid = fl->tgid;
	vmsg->tid = current->pid;
	vmsg->cid = fl->cid;
	vmsg->cmd = VIRTIO_FASTRPC_CMD_CLOSE;
	vmsg->len = sizeof(*vmsg);
	vmsg->msgid = msg->msgid;
	vmsg->result = 0xffffffff;

	err = fastrpc_txbuf_send(fl, vmsg, sizeof(*vmsg));
	if (err)
		goto bail;

	wait_for_completion(&msg->work);

	rsp = msg->rxbuf;
	if (!rsp)
		goto bail;

	err = rsp->result;
bail:
	if (rsp)
		fastrpc_rxbuf_send(fl, rsp, me->buf_size);

	virt_free_msg(fl, msg);

	return err;
}

int fastrpc_file_free(struct fastrpc_file *fl)
{
	struct fastrpc_mmap *map = NULL, *lmap = NULL;

	if (!fl)
		return 0;

	virt_fastrpc_close(fl);

	kfree(fl->debug_buf);

	spin_lock(&fl->hlock);
	fl->file_close = 1;
	spin_unlock(&fl->hlock);

	fastrpc_context_list_dtor(fl);
	fastrpc_cached_buf_list_free(fl);
	fastrpc_remote_buf_list_free(fl);

	mutex_lock(&fl->map_mutex);
	do {
		struct hlist_node *n = NULL;

		lmap = NULL;
		hlist_for_each_entry_safe(map, n, &fl->maps, hn) {
			hlist_del_init(&map->hn);
			lmap = map;
			break;
		}
		fastrpc_mmap_free(fl, lmap, 1);
	} while (lmap);
	mutex_unlock(&fl->map_mutex);

	mutex_destroy(&fl->map_mutex);
	kfree(fl);
	return 0;
}

static int context_restore_interrupted(struct fastrpc_file *fl,
					struct fastrpc_ioctl_invoke *invoke,
					struct fastrpc_invoke_ctx **po)
{
	int err = 0;
	struct fastrpc_apps *me = fl->apps;
	struct fastrpc_invoke_ctx *ctx = NULL, *ictx = NULL;
	struct hlist_node *n;

	spin_lock(&fl->hlock);
	hlist_for_each_entry_safe(ictx, n, &fl->clst.interrupted, hn) {
		if (ictx->pid == current->pid) {
			if (invoke->sc != ictx->sc || ictx->fl != fl) {
				err = -EINVAL;
				dev_err(me->dev,
					"interrupted sc (0x%x) or fl (%pK) does not match with invoke sc (0x%x) or fl (%pK)\n",
					ictx->sc, ictx->fl, invoke->sc, fl);
			} else {
				ctx = ictx;
				hlist_del_init(&ctx->hn);
				hlist_add_head(&ctx->hn, &fl->clst.pending);
			}
			break;
		}
	}
	spin_unlock(&fl->hlock);
	if (ctx)
		*po = ctx;
	return err;
}

static int context_alloc(struct fastrpc_file *fl,
			struct fastrpc_ioctl_invoke_perf *invokefd,
			struct fastrpc_invoke_ctx **po)
{
	int err = 0, bufs, size = 0;
	struct fastrpc_invoke_ctx *ctx = NULL;
	struct fastrpc_ctx_lst *clst = &fl->clst;
	struct fastrpc_ioctl_invoke *invoke = &invokefd->inv;

	bufs = REMOTE_SCALARS_LENGTH(invoke->sc);
	size = bufs * sizeof(*ctx->lpra) + bufs * sizeof(*ctx->maps) +
		sizeof(*ctx->fds) * (bufs) +
		sizeof(*ctx->attrs) * (bufs);

	VERIFY(err, NULL != (ctx = kzalloc(sizeof(*ctx) + size, GFP_KERNEL)));
	if (err)
		goto bail;

	INIT_HLIST_NODE(&ctx->hn);
	hlist_add_fake(&ctx->hn);
	ctx->fl = fl;
	ctx->maps = (struct fastrpc_mmap **)(&ctx[1]);
	ctx->lpra = (remote_arg_t *)(&ctx->maps[bufs]);
	ctx->fds = (int *)(&ctx->lpra[bufs]);
	ctx->attrs = (unsigned int *)(&ctx->fds[bufs]);

	K_COPY_FROM_USER(err, 0, (void *)ctx->lpra, invoke->pra,
			bufs * sizeof(*ctx->lpra));
	if (err)
		goto bail;

	if (invokefd->fds) {
		K_COPY_FROM_USER(err, 0, ctx->fds, invokefd->fds,
				bufs * sizeof(*ctx->fds));
		if (err)
			goto bail;
	} else {
		ctx->fds = NULL;
	}
	if (invokefd->attrs) {
		K_COPY_FROM_USER(err, 0, ctx->attrs, invokefd->attrs,
				bufs * sizeof(*ctx->attrs));
		if (err)
			goto bail;
	}
	ctx->sc = invoke->sc;
	ctx->handle = invoke->handle;
	ctx->pid = current->pid;
	ctx->tgid = fl->tgid;
	ctx->crc = (uint32_t *)invokefd->crc;
	ctx->perf_dsp = (uint64_t *)invokefd->perf_dsp;
	ctx->perf_kernel = (uint64_t *)invokefd->perf_kernel;

	if (ctx->fl->profile) {
		ctx->perf = kzalloc(sizeof(*(ctx->perf)), GFP_KERNEL);
		VERIFY(err, !IS_ERR_OR_NULL(ctx->perf));
		if (err) {
			kfree(ctx->perf);
			err = -ENOMEM;
			goto bail;
		}
		memset(ctx->perf, 0, sizeof(*(ctx->perf)));
		ctx->perf->tid = fl->tgid;
	}

	spin_lock(&fl->hlock);
	hlist_add_head(&ctx->hn, &clst->pending);
	spin_unlock(&fl->hlock);

	*po = ctx;
bail:
	if (ctx && err)
		context_free(ctx);
	return err;
}

static void context_save_interrupted(struct fastrpc_invoke_ctx *ctx)
{
	struct fastrpc_ctx_lst *clst = &ctx->fl->clst;

	spin_lock(&ctx->fl->hlock);
	hlist_del_init(&ctx->hn);
	hlist_add_head(&ctx->hn, &clst->interrupted);
	spin_unlock(&ctx->fl->hlock);
}

static void calc_compare_crc(struct fastrpc_invoke_ctx *ctx, unsigned char *va,
		int len, uint32_t *lcrc, uint32_t *rcrc)
{
	struct fastrpc_apps *me = ctx->fl->apps;

	if (unlikely(ctx->crc)) {
		*lcrc = crc32_be(0, va, len);
		if (rcrc && (*rcrc != *lcrc))
			dev_err(me->dev, "FE/BE crc is not matched 0x%x:0x%x\n",
					*lcrc, *rcrc);
	}
}

static int get_args(struct fastrpc_invoke_ctx *ctx)
{
	struct fastrpc_file *fl = ctx->fl;
	struct fastrpc_apps *me = fl->apps;
	struct virt_invoke_msg *vmsg;
	int inbufs = REMOTE_SCALARS_INBUFS(ctx->sc);
	int outbufs = REMOTE_SCALARS_OUTBUFS(ctx->sc);
	int i, err = 0, bufs, handles, total;
	remote_arg_t *lpra = ctx->lpra;
	int *fds = ctx->fds;
	struct virt_fastrpc_buf *rpra;
	struct virt_fastrpc_dmahandle *handle;
	uint64_t *fdlist;
	uint32_t *crclist = NULL;
	uint32_t *early_hint = NULL;
	uint64_t *perf_dsp_list = NULL;
	unsigned int *attrs = ctx->attrs;
	struct fastrpc_mmap **maps = ctx->maps;
	size_t copylen = 0, size = 0, handle_len = 0, metalen;
	char *payload;
	uint64_t *perf_counter = NULL;

	bufs = inbufs + outbufs;
	handles = REMOTE_SCALARS_INHANDLES(ctx->sc)
		+ REMOTE_SCALARS_OUTHANDLES(ctx->sc);
	total = REMOTE_SCALARS_LENGTH(ctx->sc);

	if (ctx->fl->profile)
		perf_counter = (uint64_t *)ctx->perf + PERF_COUNT;

	/* calculate len required for copying */
	PERF(ctx->fl->profile, GET_COUNTER(perf_counter, PERF_MAP),
	for (i = 0; i < bufs; i++) {
		size_t len = lpra[i].buf.len;

		if (!len)
			continue;
		if (fds && (fds[i] != -1)) {
			/* map ion buffers */
			mutex_lock(&fl->map_mutex);
			err = fastrpc_mmap_create(fl, fds[i],
					(uintptr_t)lpra[i].buf.pv,
					len, 0, &maps[i]);
			mutex_unlock(&fl->map_mutex);
			if (err)
				goto bail;
			len = maps[i]->table->nents *
				sizeof(struct virt_fastrpc_sgl);
		}
		copylen += len;
		if (i < inbufs)
			ctx->outbufs_offset += len;
	}
	PERF_END);

	mutex_lock(&fl->map_mutex);
	for (i = bufs; i < total; i++) {
		int dmaflags = 0;

		if (attrs && (attrs[i] & FASTRPC_ATTR_NOMAP))
			dmaflags = FASTRPC_DMAHANDLE_NOMAP;
		if (fds && (fds[i] != -1)) {
			err = fastrpc_mmap_create(fl, fds[i],
					0, 0, dmaflags, &maps[i]);
			if (err) {
				mutex_unlock(&fl->map_mutex);
				goto bail;
			}
			handle_len += maps[i]->table->nents *
					sizeof(struct virt_fastrpc_sgl);
		}
	}
	mutex_unlock(&fl->map_mutex);

	metalen = sizeof(*vmsg) + total * sizeof(*rpra)
		+ handles * sizeof(struct virt_fastrpc_dmahandle)
		+ sizeof(uint64_t) * M_FDLIST + sizeof(uint32_t) * M_CRCLIST
		+ sizeof(uint32_t) + sizeof(uint64_t) * M_DSP_PERF_LIST;
	size = metalen + copylen + handle_len;
	if (size > me->buf_size) {
		/* if user buffer contents exceed virtio buffer limits,
		 * try to alloc an internal buffer to copy
		 */
		copylen = 0;
		ctx->outbufs_offset = 0;
		ctx->desc = kcalloc(bufs, sizeof(*ctx->desc), GFP_KERNEL);
		if (!ctx->desc) {
			err = -ENOMEM;
			goto bail;
		}
		for (i = 0; i < bufs; i++) {
			size_t len = lpra[i].buf.len;

			if (maps[i]) {
				len = maps[i]->table->nents *
					sizeof(struct virt_fastrpc_sgl);
				ctx->desc[i].type = FASTRPC_BUF_TYPE_ION;
			} else if (len < PAGE_SIZE) {
				ctx->desc[i].type = FASTRPC_BUF_TYPE_NORMAL;
			} else {
				ctx->desc[i].type = FASTRPC_BUF_TYPE_INTERNAL;
				len = PAGE_ALIGN(len);
				err = fastrpc_buf_alloc(fl, len, 0,
						0, 0, &ctx->desc[i].buf);
				if (err)
					goto bail;
				len = ctx->desc[i].buf->sgt.nents *
					sizeof(struct virt_fastrpc_sgl);
			}
			copylen += len;
			if (i < inbufs)
				ctx->outbufs_offset += len;
		}
		size = metalen + copylen + handle_len;
	}

	ctx->msg = virt_alloc_msg(fl, size);
	if (!ctx->msg) {
		err = -ENOMEM;
		goto bail;
	}

	ctx->size = size;
	vmsg = (struct virt_invoke_msg *)ctx->msg->txbuf;
	vmsg->hdr.pid = fl->tgid;
	vmsg->hdr.tid = current->pid;
	vmsg->hdr.cid = fl->cid;
	vmsg->hdr.cmd = VIRTIO_FASTRPC_CMD_INVOKE;
	vmsg->hdr.len = size;
	vmsg->hdr.msgid = ctx->msg->msgid;
	vmsg->hdr.result = 0xffffffff;
	vmsg->handle = ctx->handle;
	vmsg->sc = ctx->sc;
	vmsg->attrs = ctx->crc ? VIRTIO_FASTRPC_INVOKE_CRC : 0;
	rpra = (struct virt_fastrpc_buf *)vmsg->pra;
	handle = (struct virt_fastrpc_dmahandle *)&rpra[total];
	fdlist = (uint64_t *)&handle[handles];
	crclist = (uint32_t *)&fdlist[M_FDLIST];
	early_hint = (uint32_t *)&crclist[M_CRCLIST];
	perf_dsp_list = (uint64_t *)&early_hint[1];
	payload = (char *)&perf_dsp_list[M_DSP_PERF_LIST];

	memset(fdlist, 0, sizeof(uint64_t) * M_FDLIST + sizeof(uint32_t) * M_CRCLIST +
		sizeof(uint64_t) * M_DSP_PERF_LIST + sizeof(uint32_t));

	PERF(ctx->fl->profile, GET_COUNTER(perf_counter, PERF_COPY),
	for (i = 0; i < bufs; i++) {
		size_t len = lpra[i].buf.len;
		struct sg_table *table;
		struct virt_fastrpc_sgl *sgbuf;
		struct scatterlist *sgl = NULL;
		uint64_t buf = ptr_to_uint64(lpra[i].buf.pv);
		struct vm_area_struct *vma;
		int index = 0;
		uint64_t offset = 0;

		rpra[i].attrs = 0;
		if (maps[i]) {
			table = maps[i]->table;
			rpra[i].attrs |= FASTRPC_BUF_ATTR_ION;
			if (maps[i]->dma_flags & ION_FLAG_CACHED)
				rpra[i].attrs |= FASTRPC_BUF_ATTR_CACHED;
			rpra[i].crc = 0;
			rpra[i].pv = buf;
			rpra[i].buf_len = len;
			rpra[i].payload_len = table->nents *
				sizeof(struct virt_fastrpc_sgl);
			sgbuf = (struct virt_fastrpc_sgl *)payload;
			for_each_sg(table->sgl, sgl, table->nents, index) {
				sgbuf[index].pv = sg_dma_address(sgl);
				sgbuf[index].len = sg_dma_len(sgl);
			}
			calc_compare_crc(ctx, (uint8_t *)payload, (int)rpra[i].payload_len,
					&(rpra[i].crc), NULL);
			down_read(&current->mm->mmap_sem);
			VERIFY(err, NULL != (vma = find_vma(current->mm, maps[i]->va)));
			if (err) {
				up_read(&current->mm->mmap_sem);
				goto bail;
			}
			offset = buf - vma->vm_start;
			up_read(&current->mm->mmap_sem);
			VERIFY(err, offset + len <= (uintptr_t)maps[i]->size);
			if (err) {
				dev_err(me->dev,
						"buffer address is invalid for the fd passed for %d address 0x%llx and size %zu\n",
						i, (uintptr_t)lpra[i].buf.pv, lpra[i].buf.len);
				err = -EFAULT;
				goto bail;
			}
			rpra[i].offset = offset;
			payload += rpra[i].payload_len;
		} else if (ctx->desc &&
			   ctx->desc[i].type == FASTRPC_BUF_TYPE_INTERNAL) {
			table = &ctx->desc[i].buf->sgt;
			rpra[i].attrs |= FASTRPC_BUF_ATTR_INTERNAL;
			rpra[i].crc = 0;
			rpra[i].pv = buf;
			rpra[i].buf_len = len;
			rpra[i].payload_len = table->nents *
				sizeof(struct virt_fastrpc_sgl);
			sgbuf = (struct virt_fastrpc_sgl *)payload;
			for_each_sg(table->sgl, sgl, table->nents, index) {
				sgbuf[index].pv = page_to_phys(sg_page(sgl));
				sgbuf[index].len = sgl->length;
			}
			calc_compare_crc(ctx, (uint8_t *)payload, (int)rpra[i].payload_len,
					&(rpra[i].crc), NULL);
			if (i < inbufs && len) {
				K_COPY_FROM_USER(err, 0, ctx->desc[i].buf->va,
						lpra[i].buf.pv, len);
				if (err)
					goto bail;

			}
			rpra[i].offset = 0;
			payload += rpra[i].payload_len;
		} else {
			/* copy non ion buffers */
			rpra[i].crc = 0;
			rpra[i].pv = buf;
			rpra[i].buf_len = len;
			rpra[i].payload_len = len;
			if (i < inbufs && len) {
				K_COPY_FROM_USER(err, 0, payload,
						lpra[i].buf.pv, len);
				if (err)
					goto bail;
				calc_compare_crc(ctx, (uint8_t *)payload, (int)len,
						&(rpra[i].crc), NULL);
			}
			rpra[i].offset = 0;
			payload += len;
		}
	}
	PERF_END);

	for (i = bufs; i < total; i++) {
		struct sg_table *table;
		struct virt_fastrpc_sgl *sgbuf;
		struct scatterlist *sgl = NULL;
		int index = 0, hlist;

		if (fds && maps[i]) {
			/* fill in dma handle list */
			hlist = i - bufs;
			handle[hlist].fd = fds[i];
			handle[hlist].offset = (uint32_t)(uintptr_t)lpra[i].buf.pv;
			/* copy dma handle sglist to data area */
			table = maps[i]->table;
			rpra[i].attrs = FASTRPC_BUF_ATTR_ION;
			if (maps[i]->dma_flags & ION_FLAG_CACHED)
				rpra[i].attrs |= FASTRPC_BUF_ATTR_CACHED;
			rpra[i].buf_len = lpra[i].buf.len;
			rpra[i].payload_len = table->nents *
				sizeof(struct virt_fastrpc_sgl);
			sgbuf = (struct virt_fastrpc_sgl *)payload;
			for_each_sg(table->sgl, sgl, table->nents, index) {
				sgbuf[index].pv = sg_dma_address(sgl);
				sgbuf[index].len = sg_dma_len(sgl);
			}
			calc_compare_crc(ctx, (uint8_t *)payload, (int) rpra[i].payload_len,
					&(rpra[i].crc), NULL);
			rpra[i].offset = 0;
			payload += rpra[i].payload_len;
		}
	}
bail:
	return err;
}

static int put_args(struct fastrpc_invoke_ctx *ctx)
{
	int err = 0;
	struct fastrpc_apps *me = ctx->fl->apps;
	struct fastrpc_file *fl = ctx->fl;
	struct virt_fastrpc_msg *msg = ctx->msg;
	struct virt_invoke_msg *rsp = NULL;
	int inbufs = REMOTE_SCALARS_INBUFS(ctx->sc);
	int outbufs = REMOTE_SCALARS_OUTBUFS(ctx->sc);
	int i, bufs, handles, total;
	remote_arg_t *lpra = ctx->lpra;
	struct virt_fastrpc_buf *rpra;
	struct virt_fastrpc_dmahandle *handle;
	uint32_t crc;
	uint64_t *fdlist;
	uint32_t *crclist = NULL;
	uint32_t *early_hint = NULL;
	uint64_t *perf_dsp_list = NULL;
	struct fastrpc_mmap **maps = ctx->maps, *mmap = NULL;
	char *payload;

	if (!msg) {
		dev_err(me->dev, "%s: ctx msg is NULL\n", __func__);
		err = -EINVAL;
		goto bail;
	}
	rsp = msg->rxbuf;
	if (!rsp) {
		dev_err(me->dev, "%s: response invoke msg is NULL\n", __func__);
		err = -EINVAL;
		goto bail;
	}
	err = rsp->hdr.result;
	if (err)
		goto bail;

	bufs = inbufs + outbufs;
	handles = REMOTE_SCALARS_INHANDLES(ctx->sc)
		+ REMOTE_SCALARS_OUTHANDLES(ctx->sc);
	total = REMOTE_SCALARS_LENGTH(ctx->sc);

	rpra = (struct virt_fastrpc_buf *)rsp->pra;
	handle = (struct virt_fastrpc_dmahandle *)&rpra[total];
	fdlist = (uint64_t *)&handle[handles];
	crclist = (uint32_t *)(fdlist + M_FDLIST);
	early_hint = (uint32_t *)(crclist + M_CRCLIST);
	perf_dsp_list = (uint64_t *)(early_hint + 1);
	payload = (char *)&perf_dsp_list[M_DSP_PERF_LIST] + ctx->outbufs_offset;

	for (i = inbufs; i < bufs; i++) {
		if (maps[i]) {
			mutex_lock(&fl->map_mutex);
			fastrpc_mmap_free(fl, maps[i], 0);
			mutex_unlock(&fl->map_mutex);
			maps[i] = NULL;
		} else if (ctx->desc &&
			   ctx->desc[i].type == FASTRPC_BUF_TYPE_INTERNAL) {
			K_COPY_TO_USER(err, 0, lpra[i].buf.pv,
					ctx->desc[i].buf->va, lpra[i].buf.len);
			if (err)
				goto bail;
		} else {
			calc_compare_crc(ctx, (uint8_t *)payload, (int)rpra[i].buf_len,
					&crc, &(rpra[i].crc));

			K_COPY_TO_USER(err, 0, lpra[i].buf.pv,
					payload, rpra[i].buf_len);
			if (err)
				goto bail;
		}
		payload += rpra[i].payload_len;
	}

	mutex_lock(&fl->map_mutex);
	if (total) {
		for (i = 0; i < M_FDLIST; i++) {
			if (!fdlist[i])
				break;
			if (!fastrpc_mmap_find(fl, (int)fdlist[i], 0, 0,
						0, 0, &mmap))
				fastrpc_mmap_free(fl, mmap, 0);
		}
	}
	mutex_unlock(&fl->map_mutex);
	if (ctx->crc && crclist && rpra)
		K_COPY_TO_USER(err, 0, ctx->crc,
				crclist, M_CRCLIST * sizeof(uint32_t));

bail:
	return err;
}

static int virt_fastrpc_invoke(struct fastrpc_file *fl, struct fastrpc_invoke_ctx *ctx)
{
	struct fastrpc_apps *me = fl->apps;
	struct virt_fastrpc_msg *msg = ctx->msg;
	struct virt_invoke_msg *vmsg;
	int err = 0;

	if (!msg) {
		dev_err(me->dev, "%s: ctx msg is NULL\n", __func__);
		err = -EINVAL;
		goto bail;
	}
	vmsg = (struct virt_invoke_msg *)msg->txbuf;
	if (!vmsg) {
		dev_err(me->dev, "%s: invoke msg is NULL\n", __func__);
		err = -EINVAL;
		goto bail;
	}

	err = fastrpc_txbuf_send(fl, vmsg, ctx->size);
bail:
	return err;
}

static void fastrpc_update_invoke_count(uint32_t handle, uint64_t *perf_counter,
		struct timespec64 *invoket)
{
	if (handle != FASTRPC_STATIC_HANDLE_LISTENER) {
		uint64_t *count = GET_COUNTER(perf_counter, PERF_INVOKE);

		if (count)
			*count += getnstimediff(invoket);
	}
	if (handle > FASTRPC_STATIC_HANDLE_MAX) {
		uint64_t *count = GET_COUNTER(perf_counter, PERF_COUNT);

		if (count)
			*count += 1;
	}
}

int fastrpc_internal_invoke(struct fastrpc_file *fl,
			uint32_t mode, struct fastrpc_ioctl_invoke_perf *inv)
{
	struct fastrpc_ioctl_invoke *invoke = &inv->inv;
	struct fastrpc_apps *me = fl->apps;
	struct fastrpc_invoke_ctx *ctx = NULL;
	int err = 0, interrupted = 0;
	struct timespec64 invoket = {0};
	uint64_t *perf_counter = NULL;

	VERIFY(err, invoke->handle != FASTRPC_STATIC_HANDLE_KERNEL);
	if (err) {
		dev_err(me->dev, "user application %s trying to send a kernel RPC message to channel %d\n",
			current->comm, fl->domain);
		goto bail;
	}

	VERIFY(err, fl->domain >= 0 && fl->domain < me->num_channels);
	if (err) {
		dev_err(me->dev, "user application %s domain is not set\n",
				current->comm);
		err = -EBADR;
		goto bail;
	}

	if (fl->profile)
		ktime_get_real_ts64(&invoket);

	VERIFY(err, 0 == context_restore_interrupted(fl, invoke, &ctx));
	if (err)
		goto bail;
	if (ctx)
		goto wait;

	VERIFY(err, 0 == context_alloc(fl, inv, &ctx));
	if (err)
		goto bail;

	if (fl->profile)
		perf_counter = (uint64_t *)ctx->perf + PERF_COUNT;

	PERF(fl->profile, GET_COUNTER(perf_counter, PERF_GETARGS),
	VERIFY(err, 0 == get_args(ctx));
	PERF_END);
	if (err)
		goto bail;

	PERF(fl->profile, GET_COUNTER(perf_counter, PERF_LINK),
	VERIFY(err, 0 == virt_fastrpc_invoke(fl, ctx));
	PERF_END);
	if (err)
		goto bail;

wait:
	interrupted = wait_for_completion_interruptible(&ctx->msg->work);
	VERIFY(err, 0 == (err = interrupted));
	if (err)
		goto bail;
	PERF(fl->profile, GET_COUNTER(perf_counter, PERF_PUTARGS),
	VERIFY(err, 0 == put_args(ctx));
	PERF_END);
	if (err)
		goto bail;
bail:
	if (ctx && interrupted == -ERESTARTSYS)
		context_save_interrupted(ctx);
	else if (ctx) {
		if (fl->profile && !interrupted)
			fastrpc_update_invoke_count(invoke->handle,
					perf_counter, &invoket);

		if (fl->profile && ctx->perf && ctx->perf_kernel)
			K_COPY_TO_USER_WITHOUT_ERR(0, ctx->perf_kernel,
					ctx->perf, M_KERNEL_PERF_LIST*sizeof(uint64_t));
		context_free(ctx);
	}

	return err;
}

static int virt_fastrpc_munmap(struct fastrpc_file *fl, uintptr_t raddr,
				size_t size)
{
	struct fastrpc_apps *me = fl->apps;
	struct virt_munmap_msg *vmsg, *rsp = NULL;
	struct virt_fastrpc_msg *msg;
	int err;

	msg = virt_alloc_msg(fl, sizeof(*vmsg));
	if (!msg)
		return -ENOMEM;

	vmsg = (struct virt_munmap_msg *)msg->txbuf;
	vmsg->hdr.pid = fl->tgid;
	vmsg->hdr.tid = current->pid;
	vmsg->hdr.cid = fl->cid;
	vmsg->hdr.cmd = VIRTIO_FASTRPC_CMD_MUNMAP;
	vmsg->hdr.len = sizeof(*vmsg);
	vmsg->hdr.msgid = msg->msgid;
	vmsg->hdr.result = 0xffffffff;
	vmsg->vdsp = raddr;
	vmsg->size = size;

	err = fastrpc_txbuf_send(fl, vmsg, sizeof(*vmsg));
	if (err)
		goto bail;

	wait_for_completion(&msg->work);

	rsp = msg->rxbuf;
	if (!rsp)
		goto bail;

	err = rsp->hdr.result;
bail:
	if (rsp)
		fastrpc_rxbuf_send(fl, rsp, me->buf_size);
	virt_free_msg(fl, msg);

	return err;
}

int fastrpc_internal_munmap(struct fastrpc_file *fl,
				   struct fastrpc_ioctl_munmap *ud)
{
	int err = 0;
	struct fastrpc_apps *me = fl->apps;
	struct fastrpc_mmap *map = NULL;
	struct fastrpc_buf *rbuf = NULL, *free = NULL;
	struct hlist_node *n;

	VERIFY(err, fl->dsp_proc_init == 1);
	if (err) {
		dev_err(me->dev, "%s: user application %s trying to unmap without initialization\n",
			 __func__, current->comm);
		err = -EBADR;
		goto bail;
	}

	spin_lock(&fl->hlock);
	hlist_for_each_entry_safe(rbuf, n, &fl->remote_bufs, hn_rem) {
		if (rbuf->raddr && (rbuf->flags == ADSP_MMAP_ADD_PAGES)) {
			if ((rbuf->raddr == ud->vaddrout) &&
				(rbuf->size == ud->size)) {
				free = rbuf;
				break;
			}
		}
	}
	spin_unlock(&fl->hlock);

	if (free) {
		VERIFY(err, !virt_fastrpc_munmap(fl, free->raddr, free->size));
		if (err)
			goto bail;
		fastrpc_buf_free(rbuf, 0);
		return err;
	}

	mutex_lock(&fl->map_mutex);
	VERIFY(err, !fastrpc_mmap_remove(fl, ud->vaddrout, ud->size, &map));
	mutex_unlock(&fl->map_mutex);
	if (err) {
		dev_err(me->dev, "mapping not found to unmap va 0x%lx, len 0x%x\n",
				ud->vaddrout, (unsigned int)ud->size);
		goto bail;
	}
	VERIFY(err, !virt_fastrpc_munmap(fl, map->raddr, map->size));
	if (err)
		goto bail;
	mutex_lock(&fl->map_mutex);
	fastrpc_mmap_free(fl, map, 0);
	mutex_unlock(&fl->map_mutex);
bail:
	if (err && map) {
		mutex_lock(&fl->map_mutex);
		fastrpc_mmap_add(fl, map);
		mutex_unlock(&fl->map_mutex);
	}
	return err;
}

int fastrpc_internal_munmap_fd(struct fastrpc_file *fl,
				struct fastrpc_ioctl_munmap_fd *ud)
{
	int err = 0;
	struct fastrpc_apps *me = fl->apps;
	struct fastrpc_mmap *map = NULL;

	VERIFY(err, fl->dsp_proc_init == 1);
	if (err) {
		dev_err(me->dev, "%s: user application %s trying to unmap without initialization\n",
			__func__, current->comm);
		err = -EBADR;
		goto bail;
	}
	mutex_lock(&fl->map_mutex);
	if (fastrpc_mmap_find(fl, ud->fd, ud->va, ud->len, 0, 0, &map)) {
		dev_err(me->dev, "mapping not found to unmap fd 0x%x, va 0x%lx, len 0x%x\n",
			ud->fd, ud->va, (unsigned int)ud->len);
		err = -1;
		mutex_unlock(&fl->map_mutex);
		goto bail;
	}
	if (map)
		fastrpc_mmap_free(fl, map, 0);
	mutex_unlock(&fl->map_mutex);
bail:
	return err;
}

static int virt_fastrpc_mmap(struct fastrpc_file *fl, uint32_t flags,
			uintptr_t va, struct scatterlist *table,
			unsigned int nents, size_t size, uintptr_t *raddr)
{
	struct fastrpc_apps *me = fl->apps;
	struct virt_mmap_msg *vmsg, *rsp = NULL;
	struct virt_fastrpc_msg *msg;
	struct virt_fastrpc_sgl *sgbuf;
	int err, sgbuf_size, total_size;
	struct scatterlist *sgl = NULL;
	int sgl_index = 0;

	sgbuf_size = nents * sizeof(*sgbuf);
	total_size = sizeof(*vmsg) + sgbuf_size;

	msg = virt_alloc_msg(fl, total_size);
	if (!msg)
		return -ENOMEM;

	vmsg = (struct virt_mmap_msg *)msg->txbuf;
	vmsg->hdr.pid = fl->tgid;
	vmsg->hdr.tid = current->pid;
	vmsg->hdr.cid = fl->cid;
	vmsg->hdr.cmd = VIRTIO_FASTRPC_CMD_MMAP;
	vmsg->hdr.len = total_size;
	vmsg->hdr.msgid = msg->msgid;
	vmsg->hdr.result = 0xffffffff;
	vmsg->flags = flags;
	vmsg->size = size;
	vmsg->vapp = va;
	vmsg->vdsp = 0;
	vmsg->nents = nents;
	sgbuf = vmsg->sgl;

	for_each_sg(table, sgl, nents, sgl_index) {
		if (sg_dma_len(sgl)) {
			sgbuf[sgl_index].pv = sg_dma_address(sgl);
			sgbuf[sgl_index].len = sg_dma_len(sgl);
		} else {
			sgbuf[sgl_index].pv = page_to_phys(sg_page(sgl));
			sgbuf[sgl_index].len = sgl->length;
		}
	}

	err = fastrpc_txbuf_send(fl, vmsg, total_size);
	if (err)
		goto bail;

	wait_for_completion(&msg->work);

	rsp = msg->rxbuf;
	if (!rsp)
		goto bail;

	err = rsp->hdr.result;
	if (err)
		goto bail;
	*raddr = (uintptr_t)rsp->vdsp;
bail:
	if (rsp)
		fastrpc_rxbuf_send(fl, rsp, me->buf_size);
	virt_free_msg(fl, msg);

	return err;
}

int fastrpc_internal_mmap(struct fastrpc_file *fl,
				 struct fastrpc_ioctl_mmap *ud)
{
	struct fastrpc_apps *me = fl->apps;
	struct fastrpc_mmap *map = NULL;
	struct fastrpc_buf *rbuf = NULL;
	unsigned long dma_attr = 0;
	uintptr_t raddr = 0;
	int err = 0;

	VERIFY(err, fl->dsp_proc_init == 1);
	if (err) {
		dev_err(me->dev, "%s: user application %s trying to map without initialization\n",
			__func__, current->comm);
		err = -EBADR;
		goto bail;
	}
	if (ud->flags == ADSP_MMAP_ADD_PAGES) {
		if (ud->vaddrin) {
			err = -EINVAL;
			dev_err(me->dev, "%s: %s: ERROR: adding user allocated pages is not supported\n",
					current->comm, __func__);
			goto bail;
		}
		dma_attr = DMA_ATTR_NO_KERNEL_MAPPING;
		err = fastrpc_buf_alloc(fl, ud->size, dma_attr, ud->flags,
								1, &rbuf);
		if (err)
			goto bail;
		err = virt_fastrpc_mmap(fl, ud->flags, 0, rbuf->sgt.sgl,
					rbuf->sgt.nents, rbuf->size, &raddr);
		if (err)
			goto bail;
		rbuf->raddr = raddr;
	} else {
		uintptr_t va_to_dsp;

		mutex_lock(&fl->map_mutex);
		VERIFY(err, !fastrpc_mmap_create(fl, ud->fd,
				(uintptr_t)ud->vaddrin, ud->size,
				 ud->flags, &map));
		mutex_unlock(&fl->map_mutex);
		if (err)
			goto bail;

		if (ud->flags == ADSP_MMAP_HEAP_ADDR ||
			ud->flags == ADSP_MMAP_REMOTE_HEAP_ADDR)
			va_to_dsp = 0;
		else
			va_to_dsp = (uintptr_t)map->va;

		VERIFY(err, 0 == virt_fastrpc_mmap(fl, ud->flags, va_to_dsp,
					map->table->sgl, map->table->nents,
					map->size, &raddr));
		if (err)
			goto bail;
		map->raddr = raddr;
	}
	ud->vaddrout = raddr;
 bail:
	if (err && map) {
		mutex_lock(&fl->map_mutex);
		fastrpc_mmap_free(fl, map, 0);
		mutex_unlock(&fl->map_mutex);
	}
	return err;
}

static int virt_fastrpc_control(struct fastrpc_file *fl,
				struct fastrpc_ctrl_latency *lp)
{
	struct fastrpc_apps *me = fl->apps;
	struct virt_control_msg *vmsg, *rsp = NULL;
	struct virt_fastrpc_msg *msg;
	int err;

	msg = virt_alloc_msg(fl, sizeof(*vmsg));
	if (!msg)
		return -ENOMEM;

	vmsg = (struct virt_control_msg *)msg->txbuf;
	vmsg->hdr.pid = fl->tgid;
	vmsg->hdr.tid = current->pid;
	vmsg->hdr.cid = fl->cid;
	vmsg->hdr.cmd = VIRTIO_FASTRPC_CMD_CONTROL;
	vmsg->hdr.len = sizeof(*vmsg);
	vmsg->hdr.msgid = msg->msgid;
	vmsg->hdr.result = 0xffffffff;
	vmsg->enable = lp->enable;
	vmsg->latency = lp->latency;

	err = fastrpc_txbuf_send(fl, vmsg, sizeof(*vmsg));
	if (err)
		goto bail;

	wait_for_completion(&msg->work);

	rsp = msg->rxbuf;
	if (!rsp)
		goto bail;

	err = rsp->hdr.result;
bail:
	if (rsp)
		fastrpc_rxbuf_send(fl, rsp, me->buf_size);
	virt_free_msg(fl, msg);

	return err;
}

int fastrpc_internal_control(struct fastrpc_file *fl,
					struct fastrpc_ioctl_control *cp)
{
	struct fastrpc_apps *me = fl->apps;
	int err = 0;

	VERIFY(err, !IS_ERR_OR_NULL(fl) && !IS_ERR_OR_NULL(fl->apps));
	if (err)
		goto bail;
	VERIFY(err, !IS_ERR_OR_NULL(cp));
	if (err)
		goto bail;

	switch (cp->req) {
	case FASTRPC_CONTROL_LATENCY:
		if (!(me->has_control)) {
			dev_err(me->dev, "qos setting is not supported\n");
			err = -ENOTTY;
			goto bail;
		}
		virt_fastrpc_control(fl, &cp->lp);
		break;
	case FASTRPC_CONTROL_KALLOC:
		cp->kalloc.kalloc_support = 1;
		break;
	default:
		err = -ENOTTY;
		break;
	}
bail:
	return err;
}

static int fastrpc_set_process_info(struct fastrpc_file *fl)
{
	int err = 0, buf_size = 0;
	char strpid[PID_SIZE];
	char cur_comm[TASK_COMM_LEN];

	memcpy(cur_comm, current->comm, TASK_COMM_LEN);
	cur_comm[TASK_COMM_LEN - 1] = '\0';
	fl->tgid = current->tgid;

	/*
	 * Third-party apps don't have permission to open the fastrpc device, so
	 * it is opened on their behalf by DSP HAL. This is detected by
	 * comparing current PID with the one stored during device open.
	 */
	if (current->tgid != fl->tgid_open)
		fl->untrusted_process = true;
	scnprintf(strpid, PID_SIZE, "%d", current->pid);
	if (fl->apps->debugfs_root) {
		buf_size = strlen(cur_comm) + strlen("_")
			+ strlen(strpid) + 1;

		spin_lock(&fl->hlock);
		if (fl->debug_buf_alloced_attempted) {
			spin_unlock(&fl->hlock);
			return err;
		}
		fl->debug_buf_alloced_attempted = 1;
		spin_unlock(&fl->hlock);
		fl->debug_buf = kzalloc(buf_size, GFP_KERNEL);

		if (!fl->debug_buf) {
			err = -ENOMEM;
			spin_lock(&fl->hlock);
			fl->debug_buf_alloced_attempted = 0;
			spin_unlock(&fl->hlock);
			return err;
		}
		scnprintf(fl->debug_buf, buf_size, "%.10s%s%d",
			cur_comm, "_", current->pid);
		fl->debugfs_file = debugfs_create_file(fl->debug_buf, 0644,
			fl->apps->debugfs_root, fl, fl->apps->debugfs_fops);
		if (IS_ERR_OR_NULL(fl->debugfs_file)) {
			dev_warn(fl->apps->dev, "%s: %s: failed to create debugfs file %s\n",
				cur_comm, __func__, fl->debug_buf);
			fl->debugfs_file = NULL;
			kfree(fl->debug_buf);
			fl->debug_buf = NULL;
			spin_lock(&fl->hlock);
			fl->debug_buf_alloced_attempted = 0;
			spin_unlock(&fl->hlock);
		}
	}
	return err;
}

int fastrpc_ioctl_get_info(struct fastrpc_file *fl,
					uint32_t *info)
{
	int err = 0;
	uint32_t domain;
	struct fastrpc_channel_ctx *chan;

	VERIFY(err, fl != NULL);
	if (err)
		goto bail;
	err = fastrpc_set_process_info(fl);
	if (err)
		goto bail;

	if (fl->domain == -1) {
		domain = *info;
		VERIFY(err, domain < fl->apps->num_channels);
		if (err)
			goto bail;
		chan = &fl->apps->channel[domain];
		/* Check to see if the device node is non-secure */
		if (fl->dev_minor == MINOR_NUM_DEV) {
			/*
			 * If an app is trying to offload to a secure remote
			 * channel by opening the non-secure device node, allow
			 * the access if the subsystem supports unsigned
			 * offload. Untrusted apps will be restricted from
			 * offloading to signed PD using DSP HAL.
			 */
			if (chan->secure == true
			&& !chan->unsigned_support) {
				dev_err(fl->apps->dev,
				"cannot use domain %d with non-secure device\n", domain);
				err = -EACCES;
				goto bail;
			}
		}
		fl->domain = domain;
	}
	*info = 1;
bail:
	return err;
}

static int virt_fastrpc_open(struct fastrpc_file *fl,
		struct fastrpc_ioctl_init_attrs *uproc)
{
	struct fastrpc_apps *me = fl->apps;
	struct virt_open_msg *vmsg, *rsp = NULL;
	struct virt_fastrpc_msg *msg;
	int err;

	msg = virt_alloc_msg(fl, sizeof(*vmsg));
	if (!msg) {
		dev_err(me->dev, "%s: no memory\n", __func__);
		return -ENOMEM;
	}

	vmsg = (struct virt_open_msg *)msg->txbuf;
	vmsg->hdr.pid = fl->tgid;
	vmsg->hdr.tid = current->pid;
	vmsg->hdr.cid = -1;
	vmsg->hdr.cmd = VIRTIO_FASTRPC_CMD_OPEN;
	vmsg->hdr.len = sizeof(*vmsg);
	vmsg->hdr.msgid = msg->msgid;
	vmsg->hdr.result = 0xffffffff;
	vmsg->domain = fl->domain;
	vmsg->pd = fl->pd;
	vmsg->attrs = uproc->attrs;


	err = fastrpc_txbuf_send(fl, vmsg, sizeof(*vmsg));
	if (err)
		goto bail;
	wait_for_completion(&msg->work);

	rsp = msg->rxbuf;
	if (!rsp)
		goto bail;

	err = rsp->hdr.result;
	if (err)
		goto bail;
	if (rsp->hdr.cid < 0) {
		dev_err(me->dev, "open: channel id %d is invalid\n", rsp->hdr.cid);
		err = -EINVAL;
		goto bail;
	}
	fl->cid = rsp->hdr.cid;
bail:
	if (rsp)
		fastrpc_rxbuf_send(fl, rsp, me->buf_size);
	virt_free_msg(fl, msg);

	return err;
}

int fastrpc_init_process(struct fastrpc_file *fl,
				struct fastrpc_ioctl_init_attrs *uproc)
{
	int err = 0;
	struct fastrpc_ioctl_init *init = &uproc->init;
	int domain = fl->domain;
	struct fastrpc_channel_ctx *chan = &fl->apps->channel[domain];

	if (chan->unsigned_support && fl->dev_minor == MINOR_NUM_DEV) {
		/*
		 * Make sure third party applications
		 * can spawn only unsigned PD when
		 * channel configured as secure.
		 */
		if (chan->secure && !(uproc->attrs & FASTRPC_MODE_UNSIGNED_MODULE)) {
			err = -ECONNREFUSED;
			goto bail;
		}
	} else if (!(chan->unsigned_support) && (uproc->attrs & FASTRPC_MODE_UNSIGNED_MODULE)) {
		err = -ECONNREFUSED;
		goto bail;
	}

	switch (init->flags) {
	case FASTRPC_INIT_CREATE:
		fl->pd = DYNAMIC_PD;
		/* Untrusted apps are not allowed to offload to signedPD on DSP. */
		if (fl->untrusted_process) {
			VERIFY(err, uproc->attrs & FASTRPC_MODE_UNSIGNED_MODULE);
			if (err) {
				err = -ECONNREFUSED;
				dev_err(fl->apps->dev,
					"untrusted app trying to offload to signed remote process\n");
				goto bail;
			}
		}

		fl->procattrs = uproc->attrs;
		break;
	case FASTRPC_INIT_CREATE_STATIC:
	case FASTRPC_INIT_ATTACH:
	case FASTRPC_INIT_ATTACH_SENSORS:
		err = -ECONNREFUSED;
		goto bail;
	default:
		return -ENOTTY;
	}
	err = virt_fastrpc_open(fl, uproc);
	if (err)
		goto bail;
	fl->dsp_proc_init = 1;
bail:
	return err;
}

static int virt_fastrpc_get_dsp_info(struct fastrpc_file *fl,
		u32 *dsp_attributes)
{
	struct fastrpc_apps *me = fl->apps;
	struct virt_cap_msg *vmsg, *rsp = NULL;
	struct virt_fastrpc_msg *msg;
	int err;

	msg = virt_alloc_msg(fl, sizeof(*vmsg));
	if (!msg) {
		dev_err(me->dev, "%s: no memory\n", __func__);
		return -ENOMEM;
	}

	vmsg = (struct virt_cap_msg *)msg->txbuf;
	vmsg->hdr.pid = fl->tgid;
	vmsg->hdr.tid = current->pid;
	vmsg->hdr.cid = -1;
	vmsg->hdr.cmd = VIRTIO_FASTRPC_CMD_GET_DSP_INFO;
	vmsg->hdr.len = sizeof(*vmsg);
	vmsg->hdr.msgid = msg->msgid;
	vmsg->hdr.result = 0xffffffff;
	vmsg->domain = fl->domain;
	memset(vmsg->dsp_caps, 0, FASTRPC_MAX_DSP_ATTRIBUTES * sizeof(u32));

	err = fastrpc_txbuf_send(fl, vmsg, sizeof(*vmsg));
	if (err)
		goto bail;
	wait_for_completion(&msg->work);

	rsp = msg->rxbuf;
	if (!rsp)
		goto bail;

	err = rsp->hdr.result;
	if (err)
		goto bail;
	memcpy(dsp_attributes, rsp->dsp_caps, FASTRPC_MAX_DSP_ATTRIBUTES * sizeof(u32));
bail:
	if (rsp)
		fastrpc_rxbuf_send(fl, rsp, me->buf_size);
	virt_free_msg(fl, msg);

	return err;
}

static int fastrpc_get_info_from_kernel(
		struct fastrpc_ioctl_capability *cap,
		struct fastrpc_file *fl)
{
	int err = 0;
	uint32_t domain = cap->domain, attribute_ID = cap->attribute_ID;
	struct fastrpc_dsp_capabilities *dsp_cap_ptr = NULL;

	/*
	 * Check if number of attribute IDs obtained from userspace
	 * is less than the number of attribute IDs supported by
	 * kernel
	 */
	if (attribute_ID >= FASTRPC_MAX_ATTRIBUTES) {
		err = -EOVERFLOW;
		goto bail;
	}

	dsp_cap_ptr = &fl->apps->channel[domain].dsp_cap_kernel;

	if (attribute_ID >= FASTRPC_MAX_DSP_ATTRIBUTES) {
		/* Driver capability, pass it to user */
		memcpy(&cap->capability,
			&kernel_capabilities[attribute_ID -
			FASTRPC_MAX_DSP_ATTRIBUTES],
			sizeof(cap->capability));
	} else if (!dsp_cap_ptr->is_cached) {
		/*
		 * Information not on kernel, query device for information
		 * and cache on kernel
		 */
		err = virt_fastrpc_get_dsp_info(fl,
			  dsp_cap_ptr->dsp_attributes);
		if (err)
			goto bail;

		fl->apps->channel[domain].unsigned_support =
			!!(dsp_cap_ptr->dsp_attributes[UNSIGNED_PD_SUPPORT]);

		memcpy(&cap->capability,
			&dsp_cap_ptr->dsp_attributes[attribute_ID],
			sizeof(cap->capability));

		dsp_cap_ptr->is_cached = 1;
	} else {
		/* Information on Kernel, pass it to user */
		memcpy(&cap->capability,
			&dsp_cap_ptr->dsp_attributes[attribute_ID],
			sizeof(cap->capability));
	}
bail:
	return err;
}

int fastrpc_ioctl_get_dsp_info(struct fastrpc_ioctl_capability *cap,
		void *param, struct fastrpc_file *fl)
{
	int err = 0;

	K_COPY_FROM_USER(err, 0, cap, param, sizeof(struct fastrpc_ioctl_capability));
	if (err)
		goto bail;

	VERIFY(err, cap->domain < fl->apps->num_channels);
	if (err) {
		err = -ECHRNG;
		goto bail;
	}
	cap->capability = 0;

	err = fastrpc_get_info_from_kernel(cap, fl);
	if (err)
		goto bail;
	K_COPY_TO_USER(err, 0, &((struct fastrpc_ioctl_capability *)
				param)->capability, &cap->capability, sizeof(cap->capability));
bail:
	return err;
}
