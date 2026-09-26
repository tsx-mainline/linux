// SPDX-License-Identifier: GPL-2.0
//
// Amlogic Meson8 AUDIN: I2S input (FIFO0) as an ASoC front-end.
//
// The AUDIN block (CBUS 0x2800..0x28bf) receives I2S data on up to four
// data lines and writes it to a ring buffer in memory through FIFO0. It has
// no bit or frame clock generator of its own for the I2S input: in the
// configuration used by the vendor kernel it takes both from the AIU, whose
// encoder (the "I2S Encoder" back-end DAI of the AIU driver) runs the bus.
// This driver is therefore only a front-end: its capture DAI is connected
// to the "I2S Encoder Capture" stream of the AIU through DAPM, and the AIU
// encoder starts the clocks.
//
// The FIFO writes each channel as a 32-bit word with the 24-bit sample in
// bits 23..0, and it does not interleave the channels frame by frame: every
// 64 bytes hold 8 samples of the left channel followed by the same 8
// samples of the right channel (vendor sound/soc/aml/m8/aml_i2s.c,
// aml_i2s_copy_capture()). ALSA cannot describe that layout, so the FIFO
// writes into a private ring and a timer converts the completed blocks to
// interleaved S16_LE or S32_LE frames in the ALSA buffer (which is then
// mmap-able as usual). At 48 kHz stereo that is 384 KB/s of copying.
//
// Register programming: vendor aml_audio_hw.c (i2sin_fifo0_set_buf,
// audio_in_i2s_enable, audio_in_i2s_wr_ptr) and arch/arm/mach-meson8/
// include/mach/audio.h (bit fields).

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#define AUDIN_I2SIN_CTRL		0x040	/* 0x2810 */
#define  AUDIN_I2SIN_CTRL_DIR		BIT(0)	/* bit/frame clock: 1 = output */
#define  AUDIN_I2SIN_CTRL_CLK_SEL	BIT(1)	/* bit clock: 1 = from the AIU */
#define  AUDIN_I2SIN_CTRL_LRCLK_SEL	BIT(2)	/* frame clock: 1 = from the AIU */
#define  AUDIN_I2SIN_CTRL_POS_SYNC	BIT(3)
#define  AUDIN_I2SIN_CTRL_LRCLK_SKEW	GENMASK(6, 4)
#define  AUDIN_I2SIN_CTRL_LRCLK_INVT	BIT(7)
#define  AUDIN_I2SIN_CTRL_SIZE		GENMASK(9, 8)	/* 0: 16, 1: 18, 2: 20, 3: 24 bit */
#define  AUDIN_I2SIN_CTRL_CHAN_EN	GENMASK(13, 10)	/* one bit per data line */
#define  AUDIN_I2SIN_CTRL_EN		BIT(15)
#define AUDIN_FIFO0_START		0x080	/* 0x2820 */
#define AUDIN_FIFO0_END			0x084	/* 0x2821 */
#define AUDIN_FIFO0_PTR			0x088	/* 0x2822 */
#define AUDIN_FIFO0_INTR		0x08c	/* 0x2823 */
#define AUDIN_FIFO0_RDPTR		0x090	/* 0x2824 */
#define AUDIN_FIFO0_CTRL		0x094	/* 0x2825 */
#define  AUDIN_FIFO0_CTRL_EN		BIT(0)
#define  AUDIN_FIFO0_CTRL_RST		BIT(1)
#define  AUDIN_FIFO0_CTRL_LOAD		BIT(2)	/* load the start address */
#define  AUDIN_FIFO0_CTRL_DIN_SEL	GENMASK(5, 3)	/* 1 = I2S in */
#define  AUDIN_FIFO0_CTRL_ENDIAN	GENMASK(10, 8)
#define  AUDIN_FIFO0_CTRL_CHAN		GENMASK(14, 11)
#define  AUDIN_FIFO0_CTRL_UG		BIT(15)	/* urgent DDR requests */
#define AUDIN_FIFO0_CTRL1		0x098	/* 0x2826 */
#define  AUDIN_FIFO0_CTRL1_DIN_POS	GENMASK(1, 0)
#define  AUDIN_FIFO0_CTRL1_DIN_BYTE_NUM	GENMASK(3, 2)
#define  AUDIN_FIFO0_CTRL1_DEST_SEL	GENMASK(5, 4)
#define AUDIN_MAX_REGISTER		0x2fc	/* 0x28bf */

#define AUDIN_DIN_SEL_I2S		1
#define AUDIN_ENDIAN			4	/* vendor value: 32-bit LE words */
#define AUDIN_BLOCK_BYTES		64	/* 8 frames: 8 x L then 8 x R */
#define AUDIN_BLOCK_FRAMES		8
#define AUDIN_HW_FRAME_BYTES		(AUDIN_BLOCK_BYTES / AUDIN_BLOCK_FRAMES)
#define AUDIN_RING_BYTES		SZ_64K	/* ~170 ms at 48 kHz */
#define AUDIN_TIMER_NS			(4 * NSEC_PER_MSEC)

/*
 * Bring-up aids: a non-zero i2sin_ctrl replaces the
 * computed AUDIN_I2SIN_CTRL value (without the enable bit), raw=1 copies the
 * FIFO words unconverted (S32_LE stereo only), to check the memory layout.
 */
static uint i2sin_ctrl;
module_param(i2sin_ctrl, uint, 0644);
MODULE_PARM_DESC(i2sin_ctrl, "override AUDIN_I2SIN_CTRL (0 = computed)");
static bool raw;
module_param(raw, bool, 0644);
MODULE_PARM_DESC(raw, "copy the FIFO words without de-interleaving");

struct audin {
	struct device *dev;
	struct regmap *map;
	struct clk_bulk_data *clks;
	int num_clks;
	bool pos_sync;

	void *ring;			/* FIFO0 destination */
	dma_addr_t ring_dma;

	/* stream state, protected by lock */
	spinlock_t lock;
	struct hrtimer timer;
	struct snd_pcm_substream *substream;
	bool running;
	bool raw;
	unsigned int hw_off;		/* next unread byte in the ring */
	snd_pcm_uframes_t pos;		/* next frame in the ALSA buffer */
	snd_pcm_uframes_t period_acc;
};

static const struct snd_pcm_hardware audin_pcm_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_CONTINUOUS,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = 1,
	.channels_max = 2,
	.period_bytes_min = 64,
	.period_bytes_max = 64 * 1024,
	.periods_min = 2,
	.periods_max = 1024,
	.buffer_bytes_max = 256 * 1024,
};

static u32 audin_i2sin_ctrl(struct audin *audin)
{
	if (i2sin_ctrl)
		return i2sin_ctrl & ~AUDIN_I2SIN_CTRL_EN;

	/*
	 * Vendor i2sin_fifo0_set_buf() for an I2S input with the SoC as clock
	 * master (audioin_mode = I2SIN_MASTER_MODE, audio_in_source = 0):
	 * data line 0, 24 bits, frame clock inverted with a skew of 1, both
	 * clocks from the AIU. POS_SYNC is set by the vendor machine driver
	 * for the ZL380xx (CPU DAI format SND_SOC_DAIFMT_IB_NF).
	 */
	return FIELD_PREP(AUDIN_I2SIN_CTRL_CHAN_EN, BIT(0)) |
	       FIELD_PREP(AUDIN_I2SIN_CTRL_SIZE, 3) |
	       AUDIN_I2SIN_CTRL_LRCLK_INVT |
	       FIELD_PREP(AUDIN_I2SIN_CTRL_LRCLK_SKEW, 1) |
	       (audin->pos_sync ? AUDIN_I2SIN_CTRL_POS_SYNC : 0) |
	       AUDIN_I2SIN_CTRL_LRCLK_SEL |
	       AUDIN_I2SIN_CTRL_CLK_SEL |
	       AUDIN_I2SIN_CTRL_DIR;
}

/* current FIFO0 write position as an offset in the ring, or -EIO */
static int audin_hw_offset(struct audin *audin)
{
	unsigned int ptr;

	/* writing 1 latches the write pointer (vendor audio_in_i2s_wr_ptr) */
	regmap_write(audin->map, AUDIN_FIFO0_PTR, 1);
	regmap_read(audin->map, AUDIN_FIFO0_PTR, &ptr);
	ptr &= ~(AUDIN_BLOCK_BYTES - 1);

	if (ptr < audin->ring_dma || ptr >= audin->ring_dma + AUDIN_RING_BYTES)
		return -EIO;

	return ptr - audin->ring_dma;
}

/* convert one 64-byte FIFO block (8 frames) to @frames interleaved frames */
static void audin_convert_block(struct audin *audin,
				struct snd_pcm_runtime *runtime,
				const u32 *blk, snd_pcm_uframes_t pos,
				unsigned int frames)
{
	unsigned int ch, i, channels = runtime->channels;
	void *dst = runtime->dma_area + frames_to_bytes(runtime, pos);

	if (audin->raw) {
		/* S32_LE stereo: the FIFO words as they are */
		memcpy(dst, blk, frames * AUDIN_HW_FRAME_BYTES);
		return;
	}

	for (i = 0; i < frames; i++) {
		for (ch = 0; ch < channels; ch++) {
			/* 24-bit sample in bits 23..0 */
			u32 w = blk[ch * AUDIN_BLOCK_FRAMES + i] << 8;

			if (runtime->format == SNDRV_PCM_FORMAT_S16_LE)
				((s16 *)dst)[i * channels + ch] = (s16)(w >> 16);
			else
				((s32 *)dst)[i * channels + ch] = (s32)w;
		}
	}
}

/*
 * Copy what the FIFO wrote since the last call into the ALSA buffer.
 * Returns true when a period boundary was crossed. Called with the lock.
 */
static bool audin_update(struct audin *audin)
{
	struct snd_pcm_runtime *runtime = audin->substream->runtime;
	unsigned int avail, frames;
	int hw;

	hw = audin_hw_offset(audin);
	if (hw < 0)
		return false;

	avail = (hw - audin->hw_off + AUDIN_RING_BYTES) % AUDIN_RING_BYTES;

	while (avail >= AUDIN_BLOCK_BYTES) {
		const u32 *blk = audin->ring + audin->hw_off;

		/* a block never straddles the end of the ALSA buffer */
		frames = min_t(snd_pcm_uframes_t, AUDIN_BLOCK_FRAMES,
			       runtime->buffer_size - audin->pos);
		audin_convert_block(audin, runtime, blk, audin->pos, frames);

		audin->pos = (audin->pos + frames) % runtime->buffer_size;
		audin->period_acc += frames;
		audin->hw_off = (audin->hw_off + AUDIN_BLOCK_BYTES) %
				AUDIN_RING_BYTES;
		avail -= AUDIN_BLOCK_BYTES;
	}

	if (audin->period_acc < runtime->period_size)
		return false;

	audin->period_acc %= runtime->period_size;
	return true;
}

static enum hrtimer_restart audin_timer_fn(struct hrtimer *timer)
{
	struct audin *audin = container_of(timer, struct audin, timer);
	struct snd_pcm_substream *substream;
	bool elapsed;

	spin_lock(&audin->lock);
	if (!audin->running) {
		spin_unlock(&audin->lock);
		return HRTIMER_NORESTART;
	}
	elapsed = audin_update(audin);
	substream = audin->substream;
	spin_unlock(&audin->lock);

	if (elapsed)
		snd_pcm_period_elapsed(substream);

	hrtimer_forward_now(timer, ns_to_ktime(AUDIN_TIMER_NS));
	return HRTIMER_RESTART;
}

static int audin_fifo_start(struct audin *audin)
{
	unsigned int ptr;
	int tries;

	/* vendor audio_in_i2s_enable(1): reset FIFO0 until it reads back start */
	for (tries = 0; tries < 100; tries++) {
		regmap_update_bits(audin->map, AUDIN_FIFO0_CTRL,
				   AUDIN_FIFO0_CTRL_RST, AUDIN_FIFO0_CTRL_RST);
		regmap_write(audin->map, AUDIN_FIFO0_PTR, 0);
		regmap_read(audin->map, AUDIN_FIFO0_PTR, &ptr);
		if (ptr == audin->ring_dma)
			break;
	}
	if (tries == 100) {
		dev_err(audin->dev, "FIFO0 reset failed: ptr %#x, start %#llx\n",
			ptr, (unsigned long long)audin->ring_dma);
		return -EIO;
	}

	regmap_update_bits(audin->map, AUDIN_I2SIN_CTRL, AUDIN_I2SIN_CTRL_EN,
			   AUDIN_I2SIN_CTRL_EN);
	return 0;
}

static int audin_dai_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct audin *audin = snd_soc_dai_get_drvdata(dai);
	unsigned long flags;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		ret = audin_fifo_start(audin);
		if (ret)
			return ret;
		spin_lock_irqsave(&audin->lock, flags);
		audin->hw_off = 0;
		audin->running = true;
		spin_unlock_irqrestore(&audin->lock, flags);
		hrtimer_start(&audin->timer, ns_to_ktime(AUDIN_TIMER_NS),
			      HRTIMER_MODE_REL_SOFT);
		return 0;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		spin_lock_irqsave(&audin->lock, flags);
		audin->running = false;
		spin_unlock_irqrestore(&audin->lock, flags);
		regmap_update_bits(audin->map, AUDIN_I2SIN_CTRL,
				   AUDIN_I2SIN_CTRL_EN, 0);
		/* the callback may be waiting for the stream lock: no cancel */
		hrtimer_try_to_cancel(&audin->timer);
		return 0;

	default:
		return -EINVAL;
	}
}

static int audin_dai_prepare(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct audin *audin = snd_soc_dai_get_drvdata(dai);
	u32 start = audin->ring_dma;

	/* vendor i2sin_fifo0_set_buf() */
	regmap_update_bits(audin->map, AUDIN_I2SIN_CTRL,
			   AUDIN_I2SIN_CTRL_EN, 0);
	regmap_write(audin->map, AUDIN_FIFO0_START, start);
	regmap_write(audin->map, AUDIN_FIFO0_PTR, start);
	regmap_write(audin->map, AUDIN_FIFO0_END, start + AUDIN_RING_BYTES - 8);
	regmap_write(audin->map, AUDIN_FIFO0_CTRL,
		     AUDIN_FIFO0_CTRL_EN |
		     AUDIN_FIFO0_CTRL_LOAD |
		     FIELD_PREP(AUDIN_FIFO0_CTRL_DIN_SEL, AUDIN_DIN_SEL_I2S) |
		     FIELD_PREP(AUDIN_FIFO0_CTRL_ENDIAN, AUDIN_ENDIAN) |
		     FIELD_PREP(AUDIN_FIFO0_CTRL_CHAN, 2) |
		     AUDIN_FIFO0_CTRL_UG);
	/* destination 0 (memory), 3 bytes per sample, position 0 */
	regmap_write(audin->map, AUDIN_FIFO0_CTRL1,
		     FIELD_PREP(AUDIN_FIFO0_CTRL1_DIN_BYTE_NUM, 2));
	regmap_write(audin->map, AUDIN_I2SIN_CTRL, audin_i2sin_ctrl(audin));

	audin->pos = 0;
	audin->period_acc = 0;
	audin->hw_off = 0;

	return 0;
}

static int audin_dai_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct audin *audin = snd_soc_dai_get_drvdata(dai);

	audin->raw = raw;
	if (audin->raw && (params_channels(params) != 2 ||
			   params_format(params) != SNDRV_PCM_FORMAT_S32_LE)) {
		dev_err(dai->dev, "raw mode needs S32_LE stereo\n");
		return -EINVAL;
	}

	return 0;
}

static int audin_dai_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct audin *audin = snd_soc_dai_get_drvdata(dai);
	int ret;

	snd_soc_set_runtime_hwparams(substream, &audin_pcm_hw);

	/* the conversion works in whole FIFO blocks of 8 frames */
	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
					 AUDIN_BLOCK_FRAMES);
	if (ret)
		return ret;
	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
					 AUDIN_BLOCK_FRAMES);
	if (ret)
		return ret;

	ret = clk_bulk_prepare_enable(audin->num_clks, audin->clks);
	if (ret)
		return ret;

	audin->substream = substream;
	return 0;
}

static void audin_dai_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct audin *audin = snd_soc_dai_get_drvdata(dai);

	hrtimer_cancel(&audin->timer);
	regmap_write(audin->map, AUDIN_FIFO0_CTRL, 0);
	clk_bulk_disable_unprepare(audin->num_clks, audin->clks);
	audin->substream = NULL;
}

static const struct snd_soc_dai_ops audin_dai_ops = {
	.startup	= audin_dai_startup,
	.shutdown	= audin_dai_shutdown,
	.hw_params	= audin_dai_hw_params,
	.prepare	= audin_dai_prepare,
	.trigger	= audin_dai_trigger,
};

static struct snd_soc_dai_driver audin_dai_drv = {
	.name = "AUDIN FIFO0",
	.capture = {
		.stream_name	= "FIFO0 Capture",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min	= 8000,
		.rate_max	= 192000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &audin_dai_ops,
};

static snd_pcm_uframes_t audin_pointer(struct snd_soc_component *component,
				       struct snd_pcm_substream *substream)
{
	struct audin *audin = snd_soc_component_get_drvdata(component);

	return READ_ONCE(audin->pos);
}

static int audin_sync_stop(struct snd_soc_component *component,
			   struct snd_pcm_substream *substream)
{
	struct audin *audin = snd_soc_component_get_drvdata(component);

	hrtimer_cancel(&audin->timer);
	return 0;
}

static int audin_pcm_new(struct snd_soc_component *component,
			       struct snd_soc_pcm_runtime *rtd)
{
	/* filled by the CPU, not by DMA: plain pages are enough */
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_VMALLOC, NULL,
				       0, 0);
	return 0;
}

static const struct snd_soc_component_driver audin_component = {
	.name		= "AUDIN",
	.pointer	= audin_pointer,
	.sync_stop	= audin_sync_stop,
	.pcm_new	= audin_pcm_new,
#ifdef CONFIG_DEBUG_FS
	.debugfs_prefix	= "audin",
#endif
};

static const struct regmap_config audin_regmap_cfg = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= AUDIN_MAX_REGISTER,
};

static int audin_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *regs;
	struct audin *audin;
	int ret;

	audin = devm_kzalloc(dev, sizeof(*audin), GFP_KERNEL);
	if (!audin)
		return -ENOMEM;
	audin->dev = dev;
	spin_lock_init(&audin->lock);
	hrtimer_setup(&audin->timer, audin_timer_fn, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL_SOFT);
	platform_set_drvdata(pdev, audin);

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	audin->map = devm_regmap_init_mmio(dev, regs, &audin_regmap_cfg);
	if (IS_ERR(audin->map))
		return dev_err_probe(dev, PTR_ERR(audin->map),
				     "failed to init regmap\n");

	audin->num_clks = devm_clk_bulk_get_all(dev, &audin->clks);
	if (audin->num_clks < 0)
		return dev_err_probe(dev, audin->num_clks,
				     "failed to get the clocks\n");

	ret = device_reset_optional(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to reset\n");

	audin->pos_sync = device_property_read_bool(dev, "amlogic,i2s-pos-sync");

	/* FIFO0 takes 32-bit addresses */
	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	audin->ring = dmam_alloc_coherent(dev, AUDIN_RING_BYTES,
					  &audin->ring_dma, GFP_KERNEL);
	if (!audin->ring)
		return -ENOMEM;
	if (!IS_ALIGNED(audin->ring_dma, AUDIN_BLOCK_BYTES))
		return dev_err_probe(dev, -EINVAL, "ring not 64-byte aligned\n");

	return devm_snd_soc_register_component(dev, &audin_component,
					       &audin_dai_drv, 1);
}

static const struct of_device_id audin_of_match[] = {
	{ .compatible = "amlogic,meson8-audin" },
	{ .compatible = "amlogic,meson8b-audin" },
	{}
};
MODULE_DEVICE_TABLE(of, audin_of_match);

static struct platform_driver audin_pdrv = {
	.probe = audin_probe,
	.driver = {
		.name = "meson-audin",
		.of_match_table = audin_of_match,
	},
};
module_platform_driver(audin_pdrv);

MODULE_DESCRIPTION("Amlogic Meson8 AUDIN I2S input driver");
MODULE_LICENSE("GPL");
