/*
 * linux/sound/soc/pxa/pxa910-squ.c
 *
 * Base on linux/sound/soc/pxa/pxa2xx-pcm.c
 *
 * Copyright (C) 2007 Marvell International Ltd.
 * Author: Bin Yang <bin.yang@marvell.com> 
 * 			 Yael Sheli Chemla<yael.s.shemla@marvell.com> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>

#include <plat/regs-ssp.h>
#include <plat/imm.h>
#include <mach/pxa910-squ.h>
#include "pxa3xx-pcm.h"
#include "pxa910-squ.h"

#if defined(CONFIG_DVFM)
#include <mach/dvfm.h>

static int dvfm_dev_idx;

static void set_dvfm_constraint(void)
{
	/* Disable Lowpower mode */
	dvfm_disable_op_name("apps_idle", dvfm_dev_idx);
	dvfm_disable_op_name("apps_sleep", dvfm_dev_idx);
	dvfm_disable_op_name("sys_sleep", dvfm_dev_idx);
}

static void unset_dvfm_constraint(void)
{
	/* Enable Lowpower mode */
	dvfm_enable_op_name("apps_idle", dvfm_dev_idx);
	dvfm_enable_op_name("apps_sleep", dvfm_dev_idx);
	dvfm_enable_op_name("sys_sleep", dvfm_dev_idx);
}
#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

static const struct snd_pcm_hardware pxa910_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 32,
	.period_bytes_max	= 8192 - 32,
	.periods_min		= 1,
	.periods_max		= PAGE_SIZE/sizeof(pxa910_squ_desc),
	.buffer_bytes_max	= 24 * 1024,
	.fifo_size		= 32,
};

struct pxa910_runtime_data {
	int dma_ch;
	struct pxa3xx_pcm_dma_params *params;
	void *squ_desc_array;
	dma_addr_t squ_desc_array_phys;
};

static unsigned int immid = -1;

static DECLARE_WAIT_QUEUE_HEAD(dma_wq);

static void *pxa910_sram_alloc(int size, u32 *p)
{
	void *v;
	
	if(immid == -1)
		immid = imm_register_kernel("pxa910 audio");
	
	v = imm_malloc(size, IMM_MALLOC_HARDWARE | IMM_MALLOC_SRAM, immid);
	if(v != NULL)
		*p = imm_get_physical(v, immid);
	memset(v, 0xee, size);
	return v;
}

static void pxa910_sram_free(void *v)
{
	return imm_free(v, immid);
}

static int pxa910_sram_mmap_writecombine(struct vm_area_struct *vma,
			  void *cpu_addr, dma_addr_t dma_addr, size_t size)
{
	unsigned long user_size;
	unsigned long off = vma->vm_pgoff;
	u32 ret;
	
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	user_size = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
	ret = remap_pfn_range(vma, vma->vm_start,
					      __phys_to_pfn(dma_addr) + off,
					      user_size << PAGE_SHIFT,
					      vma->vm_page_prot);
	return ret;
}



static void pxa910_squ_dma_irq(int dma_ch, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct pxa910_runtime_data *prtd = substream->runtime->private_data;
	if (SDISR(dma_ch) & 0x1) {
		snd_pcm_period_elapsed(substream);
	} else {
		printk(KERN_ERR "%s: SQU error on channel %d \n",
			prtd->params->name, dma_ch);
	}
	SDISR(dma_ch) = 0;
}


static int pxa910_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa910_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct pxa3xx_pcm_dma_params *dma = rtd->dai->cpu_dai->dma_data;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	pxa910_squ_desc *squ_desc;
	dma_addr_t dma_buff_phys, next_desc_phys;
	int ret;

	/* return if this is a bufferless transfer e.g.
	 * codec <--> BT codec or GSM modem -- lg FIXME */
	 if (!dma)
		return 0;

	 if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	 {
		dma->dcmd = SDCR_DST_ADDR_HOLD | SDCR_SRC_ADDR_INC |
			SDCR_SSPMOD | SDCR_DMA_BURST_32B | SDCR_FETCHND;
	 } else {
		dma->dcmd = SDCR_SRC_ADDR_HOLD | SDCR_DST_ADDR_INC |
			SDCR_SSPMOD | SDCR_DMA_BURST_32B | SDCR_FETCHND;
	 }

	/* this may get called several times by oss emulation
	 * with different params */
	if (prtd->params == NULL) {
		prtd->params = dma;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			ret = pxa910_request_squ(prtd->params->name, SQU_PRIO_LOW,
						pxa910_squ_dma_irq, substream);
			if (ret != 0)
				return -ENODEV;
		} else {
			ret = pxa910_request_squ(prtd->params->name, SQU_PRIO_LOW,
						pxa910_squ_dma_irq, substream);
			if (ret < 0)
				return ret;
			if (ret == 0) {
				ret = pxa910_request_squ(prtd->params->name, SQU_PRIO_LOW,
							pxa910_squ_dma_irq, substream);
				pxa910_free_squ(0);
				if (ret != 1)
					return -ENODEV;
			}
		}
		prtd->dma_ch = ret;
	} else if (prtd->params != dma) {
		pxa910_free_squ(prtd->dma_ch);
		prtd->params = dma;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			ret = pxa910_request_squ(prtd->params->name, SQU_PRIO_LOW,
						pxa910_squ_dma_irq, substream);
			if (ret != 0)
				return -ENODEV;
		} else {
			ret = pxa910_request_squ(prtd->params->name, SQU_PRIO_LOW,
						pxa910_squ_dma_irq, substream);
			if (ret < 0)
				return ret;
			if (ret == 0) {
				ret = pxa910_request_squ(prtd->params->name, SQU_PRIO_LOW,
							pxa910_squ_dma_irq, substream);
				pxa910_free_squ(0);
				if (ret != 1)
					return -ENODEV;
			}
		}
		prtd->dma_ch = ret;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;

	next_desc_phys = prtd->squ_desc_array_phys;
	dma_buff_phys = runtime->dma_addr;
		
	squ_desc = prtd->squ_desc_array;
	do {
		next_desc_phys += sizeof(pxa910_squ_desc);

		squ_desc->nxt_desc = next_desc_phys;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			squ_desc->src_addr = dma_buff_phys;
			squ_desc->dst_addr = prtd->params->dev_addr;
		} else {
			squ_desc->src_addr = prtd->params->dev_addr;
			squ_desc->dst_addr = dma_buff_phys;
		}
		if (period > totsize)
			period = totsize;
		squ_desc->byte_cnt = period;
		squ_desc++;
		dma_buff_phys += period;
	} while (totsize -= period);
	squ_desc[-1].nxt_desc = prtd->squ_desc_array_phys;

	return 0;
}

static int pxa910_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct pxa910_runtime_data *prtd = substream->runtime->private_data;

	if (prtd->dma_ch != -1) {
		snd_pcm_set_runtime_buffer(substream, NULL);
		pxa910_free_squ(prtd->dma_ch);
		prtd->dma_ch = -1;
	}

	return 0;
}

static int pxa910_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct pxa910_runtime_data *prtd = substream->runtime->private_data;

	SDCR(prtd->dma_ch) = (prtd->params->dcmd) & (~SDCR_CHANEN);
	SDIMR(prtd->dma_ch) = SDIMR_COMP ;

	return 0;
}

static int pxa910_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct pxa910_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		set_dvfm_constraint();
		SDNDPR(prtd->dma_ch) = prtd->squ_desc_array_phys;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			SDSAR(prtd->dma_ch) = substream->runtime->dma_addr;
		else
			SDDAR(prtd->dma_ch) = substream->runtime->dma_addr;
		SDCR(prtd->dma_ch) |= SDCR_CHANEN;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		unset_dvfm_constraint();
		SDCR(prtd->dma_ch) &= ~SDCR_CHANEN;
		wake_up(&dma_wq);
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
		SDCR(prtd->dma_ch) |= SDCR_CHANEN;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		SDNDPR(prtd->dma_ch) = prtd->squ_desc_array_phys;
		SDCR(prtd->dma_ch) |= SDCR_CHANEN;
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t
pxa910_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa910_runtime_data *prtd = runtime->private_data;
	dma_addr_t ptr;
	snd_pcm_uframes_t x;

	ptr = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			 SDSAR(prtd->dma_ch) : SDDAR(prtd->dma_ch);

	x = bytes_to_frames(runtime, ptr - runtime->dma_addr);

	if (x == runtime->buffer_size)
		x = 0;
	return x;
}

static int pxa910_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	struct pxa910_runtime_data *prtd;
	int ret;

	snd_soc_set_runtime_hwparams(substream, &pxa910_pcm_hardware);

	/*
	 * For mysterious reasons (and despite what the manual says)
	 * playback samples are lost if the DMA count is not a multiple
	 * of the DMA burst size.  Let's add a rule to enforce that.
	 */
	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_integer(runtime,
					SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	prtd = kzalloc(sizeof(struct pxa910_runtime_data), GFP_KERNEL);
	if (prtd == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	prtd->dma_ch = -1;
	prtd->squ_desc_array =
		prtd->squ_desc_array = pxa910_sram_alloc(PAGE_SIZE, &prtd->squ_desc_array_phys);
	if (!prtd->squ_desc_array) {
		ret = -ENOMEM;
		goto err1;
	}

	runtime->private_data = prtd;
	return 0;

 err1:
	kfree(prtd);
 out:
	return ret;
}

static int pxa910_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa910_runtime_data *prtd = runtime->private_data;

	pxa910_sram_free(prtd->squ_desc_array);
	kfree(prtd);
	return 0;
}

static int pxa910_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pxa910_runtime_data *prtd = runtime->private_data;

	return pxa910_sram_mmap_writecombine(vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
}

struct snd_pcm_ops pxa910_pcm_ops = {
	.open		= pxa910_pcm_open,
	.close		= pxa910_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= pxa910_pcm_hw_params,
	.hw_free	= pxa910_pcm_hw_free,
	.prepare	= pxa910_pcm_prepare,
	.trigger	= pxa910_pcm_trigger,
	.pointer	= pxa910_pcm_pointer,
	.mmap		= pxa910_pcm_mmap,
};

static int pxa910_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;

	size_t size = pxa910_pcm_hardware.buffer_bytes_max;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = pxa910_sram_alloc(size, &buf->addr);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void pxa910_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		pxa910_sram_free(buf->area);
		buf->area = NULL;
	}
}

static u64 pxa910_pcm_dmamask = DMA_32BIT_MASK;

int pxa910_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
	struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &pxa910_pcm_dmamask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_32BIT_MASK;

	if (dai->playback.channels_min) {
		ret = pxa910_pcm_preallocate_dma_buffer(pcm,SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		ret = pxa910_pcm_preallocate_dma_buffer(pcm,SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

struct snd_soc_platform pxa910_soc_platform = {
	.name		= "pxa910-audio",
	.pcm_ops 	= &pxa910_pcm_ops,
	.pcm_new	= pxa910_pcm_new,
	.pcm_free	= pxa910_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(pxa910_soc_platform);

static int __init pxa910_pcm_modinit(void)
{
#ifdef CONFIG_DVFM
	dvfm_register("Sound", &dvfm_dev_idx);
#endif
	return snd_soc_register_platform(&pxa910_soc_platform);
}
module_init(pxa910_pcm_modinit);

static void __exit pxa910_pcm_modexit(void)
{
	snd_soc_unregister_platform(&pxa910_soc_platform);
#ifdef CONFIG_DVFM
	dvfm_unregister("Sound", &dvfm_dev_idx);
#endif
}
module_exit(pxa910_pcm_modexit);

MODULE_AUTHOR("bin.yang@marvell.com");
MODULE_DESCRIPTION("PXA910 SQU DMA module");
MODULE_LICENSE("GPL");
