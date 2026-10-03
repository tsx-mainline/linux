// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Amlogic Meson8 MIPI CSI-2 receiver and capture driver
 *
 * The receive path has four parts:
 *  - an analog front end in the HHI registers (HHI_CSI_PHY_CNTL0..2),
 *  - an Amlogic MIPI D-PHY,
 *  - a Synopsys DesignWare CSI-2 host (version 1.02),
 *  - an Amlogic CSI-2 adapter on the VPU bus.
 *
 * The adapter can send the stream to VDIN, or write each frame directly to
 * memory. This driver uses the memory mode: the adapter writes one frame into
 * the DDR window between CSI2_DDR_START_ADDR and CSI2_DDR_END_ADDR and raises
 * an interrupt at the end of the frame. The driver then gives it the next
 * buffer.
 *
 * CSI2_GEN_CTRL1 maps the bytes of each 32-bit word. Its reset value 0xe4
 * stores each word in reversed byte order. The driver sets 0x1b, so memory
 * holds the bytes in bus order.
 *
 * The media graph is: sensor -> meson8-csi2 (subdev) -> meson8-csi2-capture.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

#define MESON8_CSI2_DRV_NAME		"meson8-csi2"

/* HHI registers (offsets in the HHI system controller) */
#define HHI_CSI_PHY_CNTL0		0x34c
#define HHI_CSI_PHY_CNTL1		0x350
#define HHI_CSI_PHY_CNTL2		0x354

#define CSI_PHY_CNTL0_ON		0xfdc1fa87
#define CSI_PHY_CNTL0_OFF		0xfcc1f780
#define CSI_PHY_CNTL1_PU(x)		((x) << 16)
#define CSI_PHY_CNTL1_CTRL2		0xffff

/* D-PHY registers */
#define PHY_CTRL			0x00
#define  PHY_CTRL_SOFT_RESET		BIT(31)
#define  PHY_CTRL_CLK_CHANNEL_B		BIT(21)
#define  PHY_CTRL_SHTDWN_CLK_LANE	BIT(4)
#define  PHY_CTRL_SHTDWN_DATA_LANES	GENMASK(3, 0)
#define PHY_CLK_LANE_CTRL		0x04
#define  PHY_CLK_LANE_CTRL_BASE		0xc0
#define  PHY_CLK_LANE_CTRL_DIV(x)	((x) << 3)
#define PHY_DATA_LANE_CTRL		0x08
#define PHY_DATA_LANE_CTRL1		0x0c
#define PHY_TCLK_MISS			0x10
#define PHY_TCLK_SETTLE			0x14
#define PHY_THS_EXIT			0x18
#define PHY_THS_SKIP			0x1c
#define PHY_THS_SETTLE			0x20
#define PHY_TINIT			0x24
#define PHY_TULPS_C			0x28
#define PHY_TULPS_S			0x2c
#define PHY_TMBIAS			0x30
#define PHY_TLP_EN_W			0x34
#define PHY_TLPOK			0x38
#define PHY_TWD_INIT			0x3c
#define PHY_TWD_HS			0x40

/* DesignWare CSI-2 host registers */
#define HOST_N_LANES			0x04
#define HOST_PHY_SHUTDOWNZ		0x08
#define HOST_DPHY_RSTZ			0x0c
#define HOST_CSI2_RESETN		0x10

/* CSI-2 adapter registers */
#define ADAP_CLK_RESET			0x00
#define  ADAP_CLK_ENABLE_DWC		BIT(3)
#define  ADAP_CLK_AUTO_GATE_OFF		BIT(2)
#define  ADAP_CLK_ENABLE		BIT(1)
#define  ADAP_SW_RESET			BIT(0)
#define ADAP_GEN_CTRL0			0x04
#define  ADAP_DDR_EN			BIT(26)
#define  ADAP_A_BRST_NUM(x)		((x) << 20)
#define  ADAP_A_ID(x)			((x) << 14)
#define  ADAP_URGENT_EN			BIT(13)
#define  ADAP_ALL_TO_MEM		BIT(4)
#define  ADAP_VC_EN(x)			BIT(x)
#define ADAP_DDR_START_ADDR		0x0c
#define ADAP_DDR_END_ADDR		0x10
#define ADAP_INT_CTRL_STAT		0x14
#define  ADAP_INT_FRAME_STATUS		BIT(17)
#define  ADAP_INT_FRAME_EN		BIT(1)
#define  ADAP_INT_CLEAR_ALL		GENMASK(18, 16)
#define ADAP_MEM_PIXEL_BYTE_CNT		0x34
#define ADAP_MEM_PIXEL_LINE_CNT		0x38
#define ADAP_ERR_STAT0			0x50
#define ADAP_GEN_CTRL1			0x54
#define  ADAP_BYTE_ORDER_BUS		0x1b

#define MESON8_CSI2_PAD_SINK		0
#define MESON8_CSI2_PAD_SRC		1
#define MESON8_CSI2_NUM_PADS		2

#define MESON8_CSI2_MIN_WIDTH		32
#define MESON8_CSI2_MIN_HEIGHT		16
#define MESON8_CSI2_MAX_WIDTH		2592
#define MESON8_CSI2_MAX_HEIGHT		1944
#define MESON8_CSI2_DEF_WIDTH		640
#define MESON8_CSI2_DEF_HEIGHT		480

/* The clock edge check of the clock lane needs at least this rate */
#define MESON8_CSI2_CLK_CHECK_MIN_HZ	40000000

struct meson8_csi2_format {
	u32 code;		/* media bus code on the CSI-2 bus */
	u32 fourcc;		/* pixel format in memory */
};

static const struct meson8_csi2_format meson8_csi2_formats[] = {
	{ MEDIA_BUS_FMT_UYVY8_1X16, V4L2_PIX_FMT_UYVY },
	{ MEDIA_BUS_FMT_VYUY8_1X16, V4L2_PIX_FMT_VYUY },
	{ MEDIA_BUS_FMT_YUYV8_1X16, V4L2_PIX_FMT_YUYV },
	{ MEDIA_BUS_FMT_YVYU8_1X16, V4L2_PIX_FMT_YVYU },
};

struct meson8_csi2_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct meson8_csi2 {
	struct device *dev;
	void __iomem *host;
	void __iomem *phy;
	void __iomem *adap;
	struct regmap *hhi;
	struct clk *clk_csi;
	struct clk *clk_phy;
	int irq;

	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct v4l2_mbus_config_mipi_csi2 bus;

	/* receiver subdev */
	struct v4l2_subdev sd;
	struct media_pad pads[MESON8_CSI2_NUM_PADS];
	struct media_pad *src_pad;	/* sensor pad while streaming */

	/* capture video device */
	struct video_device vdev;
	struct media_pad vdev_pad;
	struct vb2_queue queue;
	struct mutex lock;		/* serializes the video device ioctls */
	struct media_pipeline pipe;
	struct v4l2_pix_format pix;
	const struct meson8_csi2_format *fmt;

	spinlock_t buf_lock;		/* protects the fields below */
	struct list_head bufs;
	struct meson8_csi2_buffer *active;
	bool ddr_on;
	bool streaming;
	u32 sequence;
	unsigned int bad_frames;
};

static inline struct meson8_csi2 *sd_to_csi2(struct v4l2_subdev *sd)
{
	return container_of(sd, struct meson8_csi2, sd);
}

static inline struct meson8_csi2_buffer *
to_csi2_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct meson8_csi2_buffer, vb);
}

static const struct meson8_csi2_format *meson8_csi2_find_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(meson8_csi2_formats); i++)
		if (meson8_csi2_formats[i].code == code)
			return &meson8_csi2_formats[i];
	return NULL;
}

static const struct meson8_csi2_format *meson8_csi2_find_fourcc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(meson8_csi2_formats); i++)
		if (meson8_csi2_formats[i].fourcc == fourcc)
			return &meson8_csi2_formats[i];
	return NULL;
}

/* -----------------------------------------------------------------------------
 * Hardware: analog front end, D-PHY and host
 */

static void meson8_csi2_rx_start(struct meson8_csi2 *csi, s64 link_freq)
{
	static const u8 pu_mask[] = { 0x05, 0x07, 0x0f, 0x1f };
	unsigned int lanes = csi->bus.num_data_lanes;
	unsigned long phy_rate = clk_get_rate(csi->clk_phy);
	u64 cycle_ps, ui_ps, settle;
	unsigned int div = 0;
	u32 val;

	/*
	 * HS settle: use the minimum of the D-PHY window (85 ns + 6 UI). The
	 * vendor value (the middle of the window) is too long for the OV5640:
	 * at a 248 MHz link only 12 to 24 PHY cycles work, not 25 or more.
	 */
	cycle_ps = div64_u64(1000000000000ULL, phy_rate);
	ui_ps = div64_u64(1000000000000ULL, 2 * link_freq);
	settle = div64_u64(85000 + 6 * ui_ps, cycle_ps);

	/*
	 * The clock lane checks the HS clock edges with the clock divided by
	 * 2^div. A too long divided period makes the clock lane leave HS mode.
	 */
	while (div < 4 && (link_freq >> (div + 1)) >= MESON8_CSI2_CLK_CHECK_MIN_HZ)
		div++;

	dev_dbg(csi->dev, "link %lld Hz, phy %lu Hz, settle %llu, clock check div %u\n",
		link_freq, phy_rate, settle, 1 << div);

	/* Analog front end, clock channel A */
	regmap_write(csi->hhi, HHI_CSI_PHY_CNTL0, CSI_PHY_CNTL0_ON);
	regmap_write(csi->hhi, HHI_CSI_PHY_CNTL1,
		     CSI_PHY_CNTL1_PU(pu_mask[lanes - 1]) | CSI_PHY_CNTL1_CTRL2);

	/* D-PHY */
	val = readl(csi->phy + PHY_CTRL);
	writel(val | PHY_CTRL_SOFT_RESET, csi->phy + PHY_CTRL);
	val &= ~(PHY_CTRL_SOFT_RESET | PHY_CTRL_CLK_CHANNEL_B |
		 PHY_CTRL_SHTDWN_CLK_LANE | PHY_CTRL_SHTDWN_DATA_LANES);
	val |= PHY_CTRL_SHTDWN_DATA_LANES & ~GENMASK(lanes - 1, 0);
	writel(val, csi->phy + PHY_CTRL);

	writel(PHY_CLK_LANE_CTRL_BASE | PHY_CLK_LANE_CTRL_DIV(div),
	       csi->phy + PHY_CLK_LANE_CTRL);
	writel(0x8, csi->phy + PHY_TCLK_MISS);
	writel(0x1c, csi->phy + PHY_TCLK_SETTLE);
	writel(0x1c, csi->phy + PHY_THS_EXIT);
	writel(0x9, csi->phy + PHY_THS_SKIP);
	writel(settle, csi->phy + PHY_THS_SETTLE);
	writel(0x4e20, csi->phy + PHY_TINIT);
	writel(0x100, csi->phy + PHY_TMBIAS);
	writel(0x1000, csi->phy + PHY_TULPS_C);
	writel(0x100, csi->phy + PHY_TULPS_S);
	writel(0xc, csi->phy + PHY_TLP_EN_W);
	writel(0x100, csi->phy + PHY_TLPOK);
	writel(0x400000, csi->phy + PHY_TWD_INIT);
	writel(0x400000, csi->phy + PHY_TWD_HS);
	writel(0, csi->phy + PHY_DATA_LANE_CTRL);
	/* data lane pipeline and the HS sync error check */
	writel(0x1ff, csi->phy + PHY_DATA_LANE_CTRL1);

	/* Host */
	writel(0, csi->host + HOST_CSI2_RESETN);
	writel(~0U, csi->host + HOST_CSI2_RESETN);
	writel(~0U, csi->host + HOST_DPHY_RSTZ);
	writel(lanes - 1, csi->host + HOST_N_LANES);
	writel(~0U, csi->host + HOST_PHY_SHUTDOWNZ);
}

static void meson8_csi2_rx_stop(struct meson8_csi2 *csi)
{
	u32 val;

	val = readl(csi->phy + PHY_CTRL);
	val |= PHY_CTRL_SHTDWN_CLK_LANE | PHY_CTRL_SHTDWN_DATA_LANES;
	writel(val, csi->phy + PHY_CTRL);
	writel(val | PHY_CTRL_SOFT_RESET, csi->phy + PHY_CTRL);

	writel(0, csi->host + HOST_PHY_SHUTDOWNZ);
	writel(0, csi->host + HOST_DPHY_RSTZ);
	writel(0, csi->host + HOST_CSI2_RESETN);

	regmap_write(csi->hhi, HHI_CSI_PHY_CNTL0, CSI_PHY_CNTL0_OFF);
	regmap_write(csi->hhi, HHI_CSI_PHY_CNTL1, CSI_PHY_CNTL1_CTRL2);
	regmap_write(csi->hhi, HHI_CSI_PHY_CNTL2, 0);
}

/* -----------------------------------------------------------------------------
 * Hardware: adapter in memory mode
 */

static void meson8_csi2_adap_set_buffer(struct meson8_csi2 *csi,
					struct meson8_csi2_buffer *buf)
{
	dma_addr_t addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);

	writel(addr, csi->adap + ADAP_DDR_START_ADDR);
	writel(addr + csi->pix.sizeimage, csi->adap + ADAP_DDR_END_ADDR);
}

static void meson8_csi2_adap_ddr(struct meson8_csi2 *csi, bool on)
{
	u32 val = readl(csi->adap + ADAP_GEN_CTRL0);

	if (on)
		val |= ADAP_DDR_EN;
	else
		val &= ~ADAP_DDR_EN;
	writel(val, csi->adap + ADAP_GEN_CTRL0);
	csi->ddr_on = on;
}

/* Called with buf_lock held */
static void meson8_csi2_adap_start(struct meson8_csi2 *csi)
{
	writel(ADAP_SW_RESET, csi->adap + ADAP_CLK_RESET);
	writel(0, csi->adap + ADAP_CLK_RESET);

	writel(ADAP_A_BRST_NUM(0x3f) | ADAP_A_ID(3) | ADAP_URGENT_EN |
	       ADAP_ALL_TO_MEM | ADAP_VC_EN(0), csi->adap + ADAP_GEN_CTRL0);
	writel(ADAP_BYTE_ORDER_BUS, csi->adap + ADAP_GEN_CTRL1);
	csi->ddr_on = false;
	if (csi->active) {
		meson8_csi2_adap_set_buffer(csi, csi->active);
		meson8_csi2_adap_ddr(csi, true);
	}

	writel(ADAP_INT_CLEAR_ALL, csi->adap + ADAP_INT_CTRL_STAT);
	writel(ADAP_INT_FRAME_EN, csi->adap + ADAP_INT_CTRL_STAT);
	writel(0, csi->adap + ADAP_ERR_STAT0);
	writel(ADAP_CLK_ENABLE | ADAP_CLK_ENABLE_DWC,
	       csi->adap + ADAP_CLK_RESET);
}

static void meson8_csi2_adap_stop(struct meson8_csi2 *csi)
{
	meson8_csi2_adap_ddr(csi, false);
	writel(readl(csi->adap + ADAP_GEN_CTRL0) & ~GENMASK(3, 0),
	       csi->adap + ADAP_GEN_CTRL0);
	writel(ADAP_INT_CLEAR_ALL, csi->adap + ADAP_INT_CTRL_STAT);
	writel(ADAP_SW_RESET | ADAP_CLK_AUTO_GATE_OFF,
	       csi->adap + ADAP_CLK_RESET);
}

static irqreturn_t meson8_csi2_irq(int irq, void *data)
{
	struct meson8_csi2 *csi = data;
	struct meson8_csi2_buffer *buf;
	u32 stat, bytes, lines, err;

	stat = readl(csi->adap + ADAP_INT_CTRL_STAT);
	if (!(stat & ADAP_INT_FRAME_STATUS))
		return IRQ_NONE;

	bytes = readl(csi->adap + ADAP_MEM_PIXEL_BYTE_CNT) & 0xffff;
	lines = readl(csi->adap + ADAP_MEM_PIXEL_LINE_CNT);
	err = readl(csi->adap + ADAP_ERR_STAT0);
	writel(ADAP_INT_FRAME_STATUS | ADAP_INT_FRAME_EN,
	       csi->adap + ADAP_INT_CTRL_STAT);
	writel(0, csi->adap + ADAP_ERR_STAT0);

	spin_lock(&csi->buf_lock);

	if (!csi->streaming)
		goto out;

	buf = csi->active;
	if (buf) {
		if (!err && csi->ddr_on && bytes == csi->pix.bytesperline &&
		    lines == csi->pix.height) {
			buf->vb.vb2_buf.timestamp = ktime_get_ns();
			buf->vb.sequence = csi->sequence;
			buf->vb.field = V4L2_FIELD_NONE;
			vb2_set_plane_payload(&buf->vb.vb2_buf, 0,
					      csi->pix.sizeimage);
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
			csi->active = NULL;
		} else {
			/* incomplete or bad frame: keep the buffer */
			csi->bad_frames++;
			dev_dbg_ratelimited(csi->dev,
					    "bad frame: %u bytes, %u lines, error 0x%x\n",
					    bytes, lines, err);
		}
	}
	csi->sequence++;

	if (!csi->active) {
		buf = list_first_entry_or_null(&csi->bufs,
					       struct meson8_csi2_buffer, list);
		if (buf) {
			list_del(&buf->list);
			csi->active = buf;
		}
	}

	if (csi->active) {
		meson8_csi2_adap_set_buffer(csi, csi->active);
		if (!csi->ddr_on)
			meson8_csi2_adap_ddr(csi, true);
	} else if (csi->ddr_on) {
		meson8_csi2_adap_ddr(csi, false);
	}

out:
	spin_unlock(&csi->buf_lock);
	return IRQ_HANDLED;
}

static int meson8_csi2_power_on(struct meson8_csi2 *csi)
{
	int ret;

	ret = clk_prepare_enable(csi->clk_phy);
	if (ret)
		return ret;
	ret = clk_prepare_enable(csi->clk_csi);
	if (ret)
		clk_disable_unprepare(csi->clk_phy);
	return ret;
}

static void meson8_csi2_power_off(struct meson8_csi2 *csi)
{
	clk_disable_unprepare(csi->clk_csi);
	clk_disable_unprepare(csi->clk_phy);
}

/* -----------------------------------------------------------------------------
 * Receiver subdev
 */

static int meson8_csi2_enable_streams(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      u32 pad, u64 streams_mask)
{
	struct meson8_csi2 *csi = sd_to_csi2(sd);
	struct v4l2_subdev *src_sd;
	s64 link_freq;
	int ret;

	csi->src_pad = media_pad_remote_pad_unique(&csi->pads[MESON8_CSI2_PAD_SINK]);
	if (IS_ERR(csi->src_pad)) {
		ret = PTR_ERR(csi->src_pad);
		csi->src_pad = NULL;
		return ret;
	}
	src_sd = media_entity_to_v4l2_subdev(csi->src_pad->entity);

	link_freq = v4l2_get_link_freq(csi->src_pad, 0, 0);
	if (link_freq <= 0) {
		dev_err(csi->dev, "no link frequency from %s: %lld\n",
			src_sd->name, link_freq);
		csi->src_pad = NULL;
		return link_freq ? link_freq : -EINVAL;
	}

	meson8_csi2_rx_start(csi, link_freq);

	ret = v4l2_subdev_enable_streams(src_sd, csi->src_pad->index, BIT(0));
	if (ret) {
		dev_dbg(csi->dev, "%s stream enable failed: %d\n", src_sd->name, ret);
		meson8_csi2_rx_stop(csi);
		csi->src_pad = NULL;
	}
	return ret;
}

static int meson8_csi2_disable_streams(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       u32 pad, u64 streams_mask)
{
	struct meson8_csi2 *csi = sd_to_csi2(sd);

	if (csi->src_pad)
		v4l2_subdev_disable_streams(media_entity_to_v4l2_subdev(csi->src_pad->entity),
					    csi->src_pad->index, BIT(0));
	csi->src_pad = NULL;
	meson8_csi2_rx_stop(csi);
	return 0;
}

static int meson8_csi2_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad == MESON8_CSI2_PAD_SRC) {
		if (code->index)
			return -EINVAL;
		code->code = v4l2_subdev_state_get_format(state, code->pad)->code;
		return 0;
	}

	if (code->index >= ARRAY_SIZE(meson8_csi2_formats))
		return -EINVAL;
	code->code = meson8_csi2_formats[code->index].code;
	return 0;
}

static int meson8_csi2_enum_frame_size(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || !meson8_csi2_find_code(fse->code))
		return -EINVAL;

	fse->min_width = MESON8_CSI2_MIN_WIDTH;
	fse->max_width = MESON8_CSI2_MAX_WIDTH;
	fse->min_height = MESON8_CSI2_MIN_HEIGHT;
	fse->max_height = MESON8_CSI2_MAX_HEIGHT;
	return 0;
}

static int meson8_csi2_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt;

	if (format->pad == MESON8_CSI2_PAD_SRC)
		return v4l2_subdev_get_fmt(sd, state, format);

	if (!meson8_csi2_find_code(format->format.code))
		format->format.code = meson8_csi2_formats[0].code;
	format->format.width = clamp_t(u32, ALIGN(format->format.width, 2),
				       MESON8_CSI2_MIN_WIDTH,
				       MESON8_CSI2_MAX_WIDTH);
	format->format.height = clamp_t(u32, format->format.height,
					MESON8_CSI2_MIN_HEIGHT,
					MESON8_CSI2_MAX_HEIGHT);
	format->format.field = V4L2_FIELD_NONE;

	fmt = v4l2_subdev_state_get_format(state, MESON8_CSI2_PAD_SINK);
	*fmt = format->format;
	fmt = v4l2_subdev_state_get_format(state, MESON8_CSI2_PAD_SRC);
	*fmt = format->format;
	return 0;
}

static int meson8_csi2_init_state(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {
		.pad = MESON8_CSI2_PAD_SINK,
		.format = {
			.width = MESON8_CSI2_DEF_WIDTH,
			.height = MESON8_CSI2_DEF_HEIGHT,
			.code = meson8_csi2_formats[0].code,
			.field = V4L2_FIELD_NONE,
			.colorspace = V4L2_COLORSPACE_SRGB,
			.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
			.quantization = V4L2_QUANTIZATION_DEFAULT,
			.xfer_func = V4L2_XFER_FUNC_DEFAULT,
		},
	};

	return meson8_csi2_set_fmt(sd, state, &format);
}

static const struct v4l2_subdev_pad_ops meson8_csi2_pad_ops = {
	.enum_mbus_code = meson8_csi2_enum_mbus_code,
	.enum_frame_size = meson8_csi2_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = meson8_csi2_set_fmt,
	.enable_streams = meson8_csi2_enable_streams,
	.disable_streams = meson8_csi2_disable_streams,
};

static const struct v4l2_subdev_ops meson8_csi2_subdev_ops = {
	.pad = &meson8_csi2_pad_ops,
};

static const struct v4l2_subdev_internal_ops meson8_csi2_internal_ops = {
	.init_state = meson8_csi2_init_state,
};

static const struct media_entity_operations meson8_csi2_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* -----------------------------------------------------------------------------
 * Capture video device
 */

static void meson8_csi2_try_fmt(struct v4l2_pix_format *pix,
				const struct meson8_csi2_format **fmtp)
{
	const struct meson8_csi2_format *fmt;

	fmt = meson8_csi2_find_fourcc(pix->pixelformat);
	if (!fmt)
		fmt = &meson8_csi2_formats[0];

	pix->width = clamp_t(u32, ALIGN(pix->width, 2), MESON8_CSI2_MIN_WIDTH,
			     MESON8_CSI2_MAX_WIDTH);
	pix->height = clamp_t(u32, pix->height, MESON8_CSI2_MIN_HEIGHT,
			      MESON8_CSI2_MAX_HEIGHT);
	/* the adapter writes the lines with no padding */
	v4l2_fill_pixfmt(pix, fmt->fourcc, pix->width, pix->height);
	pix->field = V4L2_FIELD_NONE;
	if (pix->colorspace == V4L2_COLORSPACE_DEFAULT)
		pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pix->colorspace);
	pix->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(false, pix->colorspace,
							  pix->ycbcr_enc);
	pix->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(pix->colorspace);

	if (fmtp)
		*fmtp = fmt;
}

static int meson8_csi2_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	strscpy(cap->driver, MESON8_CSI2_DRV_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Amlogic Meson8 CSI-2 capture", sizeof(cap->card));
	return 0;
}

static int meson8_csi2_enum_fmt(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	const struct meson8_csi2_format *fmt;

	if (f->mbus_code) {
		fmt = meson8_csi2_find_code(f->mbus_code);
		if (!fmt || f->index)
			return -EINVAL;
	} else {
		if (f->index >= ARRAY_SIZE(meson8_csi2_formats))
			return -EINVAL;
		fmt = &meson8_csi2_formats[f->index];
	}
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int meson8_csi2_g_fmt(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct meson8_csi2 *csi = video_drvdata(file);

	f->fmt.pix = csi->pix;
	return 0;
}

static int meson8_csi2_try_fmt_vid_cap(struct file *file, void *priv,
				       struct v4l2_format *f)
{
	meson8_csi2_try_fmt(&f->fmt.pix, NULL);
	return 0;
}

static int meson8_csi2_s_fmt(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct meson8_csi2 *csi = video_drvdata(file);

	if (vb2_is_busy(&csi->queue))
		return -EBUSY;

	meson8_csi2_try_fmt(&f->fmt.pix, &csi->fmt);
	csi->pix = f->fmt.pix;
	return 0;
}

static int meson8_csi2_enum_framesizes(struct file *file, void *priv,
				       struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || !meson8_csi2_find_fourcc(fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = MESON8_CSI2_MIN_WIDTH;
	fsize->stepwise.max_width = MESON8_CSI2_MAX_WIDTH;
	fsize->stepwise.step_width = 2;
	fsize->stepwise.min_height = MESON8_CSI2_MIN_HEIGHT;
	fsize->stepwise.max_height = MESON8_CSI2_MAX_HEIGHT;
	fsize->stepwise.step_height = 1;
	return 0;
}

static const struct v4l2_ioctl_ops meson8_csi2_ioctl_ops = {
	.vidioc_querycap		= meson8_csi2_querycap,
	.vidioc_enum_fmt_vid_cap	= meson8_csi2_enum_fmt,
	.vidioc_g_fmt_vid_cap		= meson8_csi2_g_fmt,
	.vidioc_s_fmt_vid_cap		= meson8_csi2_s_fmt,
	.vidioc_try_fmt_vid_cap		= meson8_csi2_try_fmt_vid_cap,
	.vidioc_enum_framesizes		= meson8_csi2_enum_framesizes,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations meson8_csi2_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.unlocked_ioctl	= video_ioctl2,
	.read		= vb2_fop_read,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
};

static int meson8_csi2_video_link_validate(struct media_link *link)
{
	struct video_device *vdev =
		media_entity_to_video_device(link->sink->entity);
	struct v4l2_subdev *sd =
		media_entity_to_v4l2_subdev(link->source->entity);
	struct meson8_csi2 *csi = video_get_drvdata(vdev);
	struct v4l2_subdev_format src = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = link->source->index,
	};
	int ret;

	ret = v4l2_subdev_call_state_active(sd, pad, get_fmt, &src);
	if (ret)
		return ret;

	if (src.format.width != csi->pix.width ||
	    src.format.height != csi->pix.height ||
	    src.format.code != csi->fmt->code) {
		dev_dbg(csi->dev,
			"format mismatch: 0x%04x/%ux%u on the bus, 0x%04x/%ux%u in memory\n",
			src.format.code, src.format.width, src.format.height,
			csi->fmt->code, csi->pix.width, csi->pix.height);
		return -EPIPE;
	}
	return 0;
}

static const struct media_entity_operations meson8_csi2_video_entity_ops = {
	.link_validate = meson8_csi2_video_link_validate,
};

static int meson8_csi2_queue_setup(struct vb2_queue *q,
				   unsigned int *num_buffers,
				   unsigned int *num_planes,
				   unsigned int sizes[],
				   struct device *alloc_devs[])
{
	struct meson8_csi2 *csi = vb2_get_drv_priv(q);

	if (*num_planes)
		return sizes[0] < csi->pix.sizeimage ? -EINVAL : 0;

	*num_planes = 1;
	sizes[0] = csi->pix.sizeimage;
	return 0;
}

static int meson8_csi2_buf_prepare(struct vb2_buffer *vb)
{
	struct meson8_csi2 *csi = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < csi->pix.sizeimage)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, csi->pix.sizeimage);
	return 0;
}

static void meson8_csi2_buf_queue(struct vb2_buffer *vb)
{
	struct meson8_csi2 *csi = vb2_get_drv_priv(vb->vb2_queue);
	struct meson8_csi2_buffer *buf = to_csi2_buffer(to_vb2_v4l2_buffer(vb));
	unsigned long flags;

	spin_lock_irqsave(&csi->buf_lock, flags);
	if (csi->streaming && !csi->active) {
		/* the adapter is idle: give it this buffer now */
		csi->active = buf;
		meson8_csi2_adap_set_buffer(csi, buf);
		meson8_csi2_adap_ddr(csi, true);
	} else {
		list_add_tail(&buf->list, &csi->bufs);
	}
	spin_unlock_irqrestore(&csi->buf_lock, flags);
}

static void meson8_csi2_return_buffers(struct meson8_csi2 *csi,
				       enum vb2_buffer_state state)
{
	struct meson8_csi2_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&csi->buf_lock, flags);
	if (csi->active) {
		vb2_buffer_done(&csi->active->vb.vb2_buf, state);
		csi->active = NULL;
	}
	list_for_each_entry_safe(buf, tmp, &csi->bufs, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&csi->buf_lock, flags);
}

static int meson8_csi2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct meson8_csi2 *csi = vb2_get_drv_priv(q);
	unsigned long flags;
	int ret;

	ret = video_device_pipeline_start(&csi->vdev, &csi->pipe);
	if (ret) {
		dev_dbg(csi->dev, "pipeline start failed: %d\n", ret);
		goto err_return;
	}

	ret = meson8_csi2_power_on(csi);
	if (ret)
		goto err_pipeline;

	spin_lock_irqsave(&csi->buf_lock, flags);
	csi->sequence = 0;
	csi->bad_frames = 0;
	csi->active = list_first_entry_or_null(&csi->bufs,
					       struct meson8_csi2_buffer, list);
	if (csi->active)
		list_del(&csi->active->list);
	meson8_csi2_adap_start(csi);
	csi->streaming = true;
	spin_unlock_irqrestore(&csi->buf_lock, flags);

	ret = v4l2_subdev_enable_streams(&csi->sd, MESON8_CSI2_PAD_SRC, BIT(0));
	if (ret) {
		dev_dbg(csi->dev, "stream enable failed: %d\n", ret);
		goto err_adap;
	}

	return 0;

err_adap:
	spin_lock_irqsave(&csi->buf_lock, flags);
	csi->streaming = false;
	meson8_csi2_adap_stop(csi);
	spin_unlock_irqrestore(&csi->buf_lock, flags);
	synchronize_irq(csi->irq);
	meson8_csi2_power_off(csi);
err_pipeline:
	video_device_pipeline_stop(&csi->vdev);
err_return:
	meson8_csi2_return_buffers(csi, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void meson8_csi2_stop_streaming(struct vb2_queue *q)
{
	struct meson8_csi2 *csi = vb2_get_drv_priv(q);
	unsigned long flags;

	v4l2_subdev_disable_streams(&csi->sd, MESON8_CSI2_PAD_SRC, BIT(0));

	spin_lock_irqsave(&csi->buf_lock, flags);
	csi->streaming = false;
	meson8_csi2_adap_stop(csi);
	spin_unlock_irqrestore(&csi->buf_lock, flags);
	synchronize_irq(csi->irq);
	meson8_csi2_power_off(csi);

	dev_dbg(csi->dev, "stream stop: %u frames, %u bad\n",
		csi->sequence, csi->bad_frames);

	meson8_csi2_return_buffers(csi, VB2_BUF_STATE_ERROR);
	video_device_pipeline_stop(&csi->vdev);
}

static const struct vb2_ops meson8_csi2_vb2_ops = {
	.queue_setup		= meson8_csi2_queue_setup,
	.buf_prepare		= meson8_csi2_buf_prepare,
	.buf_queue		= meson8_csi2_buf_queue,
	.start_streaming	= meson8_csi2_start_streaming,
	.stop_streaming		= meson8_csi2_stop_streaming,
};

static int meson8_csi2_register_video(struct meson8_csi2 *csi)
{
	struct video_device *vdev = &csi->vdev;
	struct vb2_queue *q = &csi->queue;
	int ret;

	csi->pix.pixelformat = meson8_csi2_formats[0].fourcc;
	csi->pix.width = MESON8_CSI2_DEF_WIDTH;
	csi->pix.height = MESON8_CSI2_DEF_HEIGHT;
	meson8_csi2_try_fmt(&csi->pix, &csi->fmt);

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->drv_priv = csi;
	q->buf_struct_size = sizeof(struct meson8_csi2_buffer);
	q->ops = &meson8_csi2_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 1;
	q->lock = &csi->lock;
	q->dev = csi->dev;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	strscpy(vdev->name, MESON8_CSI2_DRV_NAME "-capture", sizeof(vdev->name));
	vdev->fops = &meson8_csi2_fops;
	vdev->ioctl_ops = &meson8_csi2_ioctl_ops;
	vdev->v4l2_dev = &csi->v4l2_dev;
	vdev->queue = q;
	vdev->lock = &csi->lock;
	vdev->release = video_device_release_empty;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE | V4L2_CAP_IO_MC;
	vdev->entity.ops = &meson8_csi2_video_entity_ops;
	video_set_drvdata(vdev, csi);

	csi->vdev_pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&vdev->entity, 1, &csi->vdev_pad);
	if (ret)
		return ret;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		media_entity_cleanup(&vdev->entity);
		return ret;
	}

	ret = media_create_pad_link(&csi->sd.entity, MESON8_CSI2_PAD_SRC,
				    &vdev->entity, 0,
				    MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		video_unregister_device(vdev);
		media_entity_cleanup(&vdev->entity);
	}
	return ret;
}

/* -----------------------------------------------------------------------------
 * Probe and remove
 */

static int meson8_csi2_notify_bound(struct v4l2_async_notifier *notifier,
				    struct v4l2_subdev *sd,
				    struct v4l2_async_connection *asc)
{
	struct meson8_csi2 *csi = container_of(notifier, struct meson8_csi2,
					       notifier);

	return v4l2_create_fwnode_links_to_pad(sd,
					       &csi->pads[MESON8_CSI2_PAD_SINK],
					       MEDIA_LNK_FL_ENABLED |
					       MEDIA_LNK_FL_IMMUTABLE);
}

static int meson8_csi2_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct meson8_csi2 *csi = container_of(notifier, struct meson8_csi2,
					       notifier);

	return v4l2_device_register_subdev_nodes(&csi->v4l2_dev);
}

static const struct v4l2_async_notifier_operations meson8_csi2_notify_ops = {
	.bound = meson8_csi2_notify_bound,
	.complete = meson8_csi2_notify_complete,
};

static int meson8_csi2_parse_endpoint(struct meson8_csi2 *csi)
{
	struct v4l2_fwnode_endpoint vep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct v4l2_async_connection *asc;
	struct fwnode_handle *ep;
	unsigned int i;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(csi->dev), 0, 0, 0);
	if (!ep)
		return dev_err_probe(csi->dev, -ENOTCONN, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	if (ret) {
		dev_err_probe(csi->dev, ret, "cannot parse the endpoint\n");
		goto out;
	}

	csi->bus = vep.bus.mipi_csi2;
	for (i = 0; i < csi->bus.num_data_lanes; i++) {
		if (csi->bus.data_lanes[i] != i + 1) {
			ret = dev_err_probe(csi->dev, -EINVAL,
					    "data lane reordering is not supported\n");
			goto out;
		}
	}

	asc = v4l2_async_nf_add_fwnode_remote(&csi->notifier, ep,
					      struct v4l2_async_connection);
	if (IS_ERR(asc))
		ret = PTR_ERR(asc);
out:
	fwnode_handle_put(ep);
	return ret;
}

static int meson8_csi2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct meson8_csi2 *csi;
	struct resource *res;
	int ret;

	csi = devm_kzalloc(dev, sizeof(*csi), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;
	csi->dev = dev;
	mutex_init(&csi->lock);
	spin_lock_init(&csi->buf_lock);
	INIT_LIST_HEAD(&csi->bufs);
	platform_set_drvdata(pdev, csi);

	csi->host = devm_platform_ioremap_resource_byname(pdev, "host");
	if (IS_ERR(csi->host))
		return PTR_ERR(csi->host);
	csi->phy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(csi->phy))
		return PTR_ERR(csi->phy);

	/*
	 * The adapter registers are inside the VPU register range, which the
	 * display driver claims. Map them without a region request.
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "adapter");
	if (!res)
		return dev_err_probe(dev, -EINVAL, "no adapter registers\n");
	csi->adap = devm_ioremap(dev, res->start, resource_size(res));
	if (!csi->adap)
		return -ENOMEM;

	csi->hhi = syscon_regmap_lookup_by_phandle(dev->of_node,
						   "amlogic,hhi-sysctrl");
	if (IS_ERR(csi->hhi))
		return dev_err_probe(dev, PTR_ERR(csi->hhi),
				     "no HHI system controller\n");

	csi->clk_csi = devm_clk_get(dev, "csi");
	if (IS_ERR(csi->clk_csi))
		return dev_err_probe(dev, PTR_ERR(csi->clk_csi), "no csi clock\n");
	csi->clk_phy = devm_clk_get(dev, "phy");
	if (IS_ERR(csi->clk_phy))
		return dev_err_probe(dev, PTR_ERR(csi->clk_phy), "no phy clock\n");

	csi->irq = platform_get_irq(pdev, 0);
	if (csi->irq < 0)
		return csi->irq;
	ret = devm_request_irq(dev, csi->irq, meson8_csi2_irq, 0,
			       dev_name(dev), csi);
	if (ret)
		return ret;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/* media and V4L2 devices */
	csi->mdev.dev = dev;
	strscpy(csi->mdev.model, "Amlogic Meson8 CSI-2", sizeof(csi->mdev.model));
	media_device_init(&csi->mdev);
	csi->v4l2_dev.mdev = &csi->mdev;
	ret = v4l2_device_register(dev, &csi->v4l2_dev);
	if (ret)
		goto err_media_cleanup;

	/* receiver subdev */
	v4l2_subdev_init(&csi->sd, &meson8_csi2_subdev_ops);
	csi->sd.internal_ops = &meson8_csi2_internal_ops;
	csi->sd.owner = THIS_MODULE;
	csi->sd.dev = dev;
	csi->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	csi->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	csi->sd.entity.ops = &meson8_csi2_entity_ops;
	strscpy(csi->sd.name, MESON8_CSI2_DRV_NAME, sizeof(csi->sd.name));
	v4l2_set_subdevdata(&csi->sd, csi);
	csi->pads[MESON8_CSI2_PAD_SINK].flags = MEDIA_PAD_FL_SINK |
						MEDIA_PAD_FL_MUST_CONNECT;
	csi->pads[MESON8_CSI2_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&csi->sd.entity, MESON8_CSI2_NUM_PADS,
				     csi->pads);
	if (ret)
		goto err_v4l2_unregister;
	ret = v4l2_subdev_init_finalize(&csi->sd);
	if (ret)
		goto err_entity_cleanup;
	ret = v4l2_device_register_subdev(&csi->v4l2_dev, &csi->sd);
	if (ret)
		goto err_subdev_cleanup;

	ret = meson8_csi2_register_video(csi);
	if (ret)
		goto err_subdev_unregister;

	ret = media_device_register(&csi->mdev);
	if (ret)
		goto err_video_unregister;

	/* sensor */
	v4l2_async_nf_init(&csi->notifier, &csi->v4l2_dev);
	csi->notifier.ops = &meson8_csi2_notify_ops;
	ret = meson8_csi2_parse_endpoint(csi);
	if (ret)
		goto err_nf_cleanup;
	ret = v4l2_async_nf_register(&csi->notifier);
	if (ret)
		goto err_nf_cleanup;

	return 0;

err_nf_cleanup:
	v4l2_async_nf_cleanup(&csi->notifier);
	media_device_unregister(&csi->mdev);
err_video_unregister:
	video_unregister_device(&csi->vdev);
	media_entity_cleanup(&csi->vdev.entity);
err_subdev_unregister:
	v4l2_device_unregister_subdev(&csi->sd);
err_subdev_cleanup:
	v4l2_subdev_cleanup(&csi->sd);
err_entity_cleanup:
	media_entity_cleanup(&csi->sd.entity);
err_v4l2_unregister:
	v4l2_device_unregister(&csi->v4l2_dev);
err_media_cleanup:
	media_device_cleanup(&csi->mdev);
	mutex_destroy(&csi->lock);
	return ret;
}

static void meson8_csi2_remove(struct platform_device *pdev)
{
	struct meson8_csi2 *csi = platform_get_drvdata(pdev);

	v4l2_async_nf_unregister(&csi->notifier);
	v4l2_async_nf_cleanup(&csi->notifier);
	media_device_unregister(&csi->mdev);
	video_unregister_device(&csi->vdev);
	media_entity_cleanup(&csi->vdev.entity);
	v4l2_device_unregister_subdev(&csi->sd);
	v4l2_subdev_cleanup(&csi->sd);
	media_entity_cleanup(&csi->sd.entity);
	v4l2_device_unregister(&csi->v4l2_dev);
	media_device_cleanup(&csi->mdev);
	mutex_destroy(&csi->lock);
}

static const struct of_device_id meson8_csi2_of_match[] = {
	{ .compatible = "amlogic,meson8-mipi-csi2" },
	{ }
};
MODULE_DEVICE_TABLE(of, meson8_csi2_of_match);

static struct platform_driver meson8_csi2_driver = {
	.probe = meson8_csi2_probe,
	.remove = meson8_csi2_remove,
	.driver = {
		.name = MESON8_CSI2_DRV_NAME,
		.of_match_table = meson8_csi2_of_match,
	},
};
module_platform_driver(meson8_csi2_driver);

MODULE_DESCRIPTION("Amlogic Meson8 MIPI CSI-2 receiver and capture driver");
MODULE_LICENSE("GPL");
