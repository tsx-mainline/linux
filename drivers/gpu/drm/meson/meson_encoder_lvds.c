// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Amlogic Meson8m2 LVDS encoder: ENCL video encoder, LCD timing controller
 * (TCON) and the LVDS transmitter with its PHY.
 *
 * The register sequence follows the Amlogic 3.10 vendor driver
 * (arch/arm/mach-meson8/lcd/lcd_config.c: set_pll_lcd(), set_venc_lcd(),
 * set_tcon_lcd(), set_control_lvds(), init_phy_lvds() and
 * lcd_ports_ctrl_lvds()). Every value is derived from the DRM mode and the
 * panel bus format, so the same code drives the 1280x800 and the 1024x600
 * panels of the Crestron TSW-1060 / TSW-760.
 *
 * Pixel clock path (Meson8m2 only):
 *
 *   VID2 PLL -> VIID pre_div (/1) -> VIID post_div (/7) -> viid_pll mux
 *   -> VCLK2 XD divider (/1) -> vclk2_in_sel -> vclk2_div1 -> cts_encl
 *
 * The VID2 PLL runs at 7x the pixel clock: the LVDS serializer clock.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/media-bus-format.h>
#include <linux/mfd/syscon.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_device.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_simple_kms_helper.h>

#include "meson_drv.h"
#include "meson_encoder_lvds.h"
#include "meson_registers.h"
#include "meson_venc.h"
#include "meson_vpp.h"

/* --- register programming (host-tested) --- */

/* HHI registers, byte offsets in the HHI system controller */
#define HHI_EDP_TX_PHY_CNTL0		0x270 /* 0x9c */
#define		HHI_EDP_TX_PHY_RESET		BIT(16)
#define HHI_DSI_LVDS_EDP_CNTL0		0x344 /* 0xd1 */
#define		HHI_DPHY_SEL_LVDS		1 /* 0: MIPI DSI, 2: eDP */
#define HHI_DSI_LVDS_EDP_CNTL1		0x348 /* 0xd2 */
#define		HHI_DPHY_EDP_SEL		BIT(4)
#define HHI_DIF_CSI_PHY_CNTL1		0x364 /* 0xd9 */
#define HHI_DIF_CSI_PHY_CNTL2		0x368 /* 0xda */
#define HHI_DIF_CSI_PHY_CNTL3		0x36c /* 0xdb */
#define		HHI_DIF_CSI_PHY_LANES		GENMASK(15, 11)
#define HHI_LVDS_TX_PHY_CNTL0		0x378 /* 0xde */
#define		HHI_LVDS_TX_PHY_SER_EN		GENMASK(20, 16)
#define		HHI_LVDS_TX_PHY_FIFO_CLK_SEL	GENMASK(7, 6)

/* DIF/CSI PHY lane enables in HHI_DIF_CSI_PHY_CNTL3[15:11] */
#define LVDS_LANE_3			BIT(0)
#define LVDS_LANE_2			BIT(1)
#define LVDS_LANE_CLK			BIT(2)
#define LVDS_LANE_1			BIT(3)
#define LVDS_LANE_0			BIT(4)

#define HHI_DIF_CSI_PHY_CNTL2_ON	0x000665b7
#define HHI_DIF_CSI_PHY_CNTL3_ON	0x84070000
#define HHI_DIF_CSI_PHY_CNTL2_OFF	0x00060000
#define HHI_DIF_CSI_PHY_CNTL3_OFF	0x00200000

/* LVDS_GEN_CNTL */
#define LVDS_GEN_FIFO_WR_EN		BIT(0)
#define LVDS_GEN_FIFO_EN		BIT(3)
#define LVDS_GEN_FIFO_CLK_DIV		GENMASK(5, 4)
#define		LVDS_GEN_FIFO_CLK_DIV7		1

/* LVDS_PHY_CLK_CNTL */
#define LVDS_PHY_CLK_DIV_RESET_N	BIT(15)

/* LVDS_PACK_CNTL_ADDR */
#define LVDS_PACK_REPACK		GENMASK(1, 0)
#define LVDS_PACK_ODD_EVEN		BIT(2)
#define LVDS_PACK_LSB_FIRST		BIT(4)
#define LVDS_PACK_PN_SWAP		BIT(5)
#define LVDS_PACK_DUAL			BIT(6)
#define LVDS_PACK_USE_TCON		BIT(7)
#define LVDS_PACK_BITS			GENMASK(9, 8) /* 0: 10, 1: 8, 2: 6 bit */
#define LVDS_PACK_R_SEL			GENMASK(11, 10)
#define LVDS_PACK_G_SEL			GENMASK(13, 12)
#define LVDS_PACK_B_SEL			GENMASK(15, 14)

/* L_POL_CNTL_ADDR */
#define L_POL_HS_LOW			BIT(0)
#define L_POL_VS_LOW			BIT(1)
#define L_POL_DE_LOW			BIT(2)
#define L_POL_TCON_HS_SEL		BIT(3)
#define L_POL_TCON_VS_SEL		BIT(4)
#define L_POL_TCON_DE_SEL		BIT(5)
#define L_POL_CPH1_POL			BIT(6)

/* ENCL_VIDEO_MODE_ADV: vendor value 0x8, "sampling rate 1" */
#define MESON_LVDS_ENCL_MODE_ADV	ENCL_VIDEO_MODE_ADV_VFIFO_EN

/*
 * The TCON starts DE this many ENCL pixel clocks after the ENCL active
 * video start (vendor LVDS_DELAY).
 */
#define MESON_LVDS_TCON_DELAY		8

/* ENCL sync outputs for the VPU; the vendor uses fixed values */
#define MESON_LVDS_HSO_BEGIN		10
#define MESON_LVDS_HSO_END		16
#define MESON_LVDS_VSO_HSTART		10
#define MESON_LVDS_VSO_VSTART		0

/* TCON colour defaults (vendor rgb_base_coeff) */
#define MESON_LVDS_RGB_BASE		0xf0
#define MESON_LVDS_RGB_COEFF		0x74a

/* PHY output swing, vendor lvds_vswing_ctrl[], level 1 (0.4 V) is default */
static const u32 meson_lvds_vswing[] = { 0x028, 0x048, 0x088, 0x0c8, 0x0f8 };
#define MESON_LVDS_VSWING_DEFAULT	1

/* clock limits from the vendor driver (lcd_config.h) */
#define MESON_LVDS_PLL_DCO_MIN		1200000000ULL
#define MESON_LVDS_PLL_DCO_MAX		3000000000ULL
#define MESON_LVDS_PRE_DIV_MAX_IN	1500000000ULL
#define MESON_LVDS_ENCL_MAX		333000000ULL
#define MESON_LVDS_PRE_DIV_MAX		6

struct meson_lvds_cfg {
	/* panel interface */
	unsigned int bits;
	unsigned int repack;
	unsigned int pn_swap;
	unsigned int vswing;
	unsigned int lanes;		/* HHI_DIF_CSI_PHY_CNTL3[15:11] */

	/* clocks */
	unsigned long long pixel_rate;	/* ENCL, Hz */
	unsigned long long pll_rate;	/* VID2 PLL output = VIID pre_div in */
	unsigned long long lvds_rate;	/* VIID pre_div out: 7 x pixel */
	unsigned int pre_div;

	/* ENCL */
	u32 max_pxcnt;
	u32 max_lncnt;
	u32 havon_begin;
	u32 havon_end;
	u32 vavon_bline;
	u32 vavon_eline;
	u32 hso_begin;
	u32 hso_end;
	u32 vso_begin;
	u32 vso_end;
	u32 vso_bline;
	u32 vso_eline;

	/* TCON */
	u32 pol_cntl;
	u32 dith_cntl;
	u32 de_hs, de_he, de_vs, de_ve;
	u32 hs_hs, hs_he, hs_vs, hs_ve;
	u32 vs_hs, vs_he, vs_vs, vs_ve;

	/* LVDS */
	u32 pack_cntl;
};

/* The mode fields the encoder needs, so the calculation can run on a host. */
struct meson_lvds_timing {
	unsigned int clock;		/* kHz */
	unsigned int hdisplay, hsync_start, hsync_end, htotal;
	unsigned int vdisplay, vsync_start, vsync_end, vtotal;
	bool hsync_high, vsync_high;
};

static int meson_lvds_bus_format(u32 bus_format, struct meson_lvds_cfg *cfg)
{
	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:	/* vesa-24 */
		cfg->bits = 8;
		cfg->repack = 1;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:	/* jeida-24 */
		cfg->bits = 8;
		cfg->repack = 0;
		break;
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:	/* vesa-18 */
		cfg->bits = 6;
		cfg->repack = 0;
		break;
	default:
		return -EINVAL;
	}

	cfg->lanes = LVDS_LANE_CLK | LVDS_LANE_0 | LVDS_LANE_1 | LVDS_LANE_2;
	if (cfg->bits == 8)
		cfg->lanes |= LVDS_LANE_3;

	return 0;
}

/*
 * Like the vendor generate_clk_parameter() for LVDS: the VIID post divider
 * is fixed to 7 and the XD divider to 1; the smallest pre divider whose
 * input can be made by the PLL (DCO 1.2..3 GHz, OD /1 /2 /4) wins.
 */
static int meson_lvds_calc_clocks(unsigned long long pixel_rate,
				  struct meson_lvds_cfg *cfg)
{
	static const unsigned int od[] = { 4, 2, 1 };
	unsigned int pre, i;

	if (!pixel_rate || pixel_rate > MESON_LVDS_ENCL_MAX)
		return -EINVAL;

	for (pre = 1; pre <= MESON_LVDS_PRE_DIV_MAX; pre++) {
		unsigned long long pll = pixel_rate * 7 * pre;

		if (pll > MESON_LVDS_PRE_DIV_MAX_IN)
			break;

		for (i = 0; i < ARRAY_SIZE(od); i++) {
			unsigned long long dco = pll * od[i];

			if (dco >= MESON_LVDS_PLL_DCO_MIN &&
			    dco <= MESON_LVDS_PLL_DCO_MAX) {
				cfg->pixel_rate = pixel_rate;
				cfg->lvds_rate = pixel_rate * 7;
				cfg->pll_rate = pll;
				cfg->pre_div = pre;
				return 0;
			}
		}
	}

	return -ERANGE;
}

static int meson_lvds_calc(const struct meson_lvds_timing *t, u32 bus_format,
			   struct meson_lvds_cfg *cfg)
{
	unsigned int hsync_width, hsync_bp, vsync_width, vsync_bp;
	unsigned int video_on_pixel, video_on_line;
	unsigned int de_hstart, de_vstart;
	int ret;

	memset(cfg, 0, sizeof(*cfg));

	ret = meson_lvds_bus_format(bus_format, cfg);
	if (ret)
		return ret;

	cfg->pn_swap = 0;
	cfg->vswing = MESON_LVDS_VSWING_DEFAULT;

	ret = meson_lvds_calc_clocks((unsigned long long)t->clock * 1000, cfg);
	if (ret)
		return ret;

	/* the TCON needs MESON_LVDS_TCON_DELAY + 1 blanking pixels */
	if (t->htotal < t->hdisplay + MESON_LVDS_TCON_DELAY + 1 ||
	    t->vtotal <= t->vdisplay ||
	    t->hsync_end < t->hsync_start || t->htotal < t->hsync_end ||
	    t->vsync_end < t->vsync_start || t->vtotal < t->vsync_end)
		return -EINVAL;

	hsync_width = t->hsync_end - t->hsync_start;
	hsync_bp = t->htotal - t->hsync_end;
	vsync_width = t->vsync_end - t->vsync_start;
	vsync_bp = t->vtotal - t->vsync_end;

	/* vendor lcd_tcon_config() */
	video_on_pixel = t->htotal - t->hdisplay - 1 - MESON_LVDS_TCON_DELAY;
	video_on_line = t->vtotal - t->vdisplay;

	/* vendor set_venc_lcd() */
	cfg->max_pxcnt = t->htotal - 1;
	cfg->max_lncnt = t->vtotal - 1;
	cfg->havon_begin = video_on_pixel;
	cfg->havon_end = t->hdisplay - 1 + video_on_pixel;
	cfg->vavon_bline = video_on_line;
	cfg->vavon_eline = t->vdisplay - 1 + video_on_line;
	cfg->hso_begin = MESON_LVDS_HSO_BEGIN;
	cfg->hso_end = MESON_LVDS_HSO_END;
	cfg->vso_begin = MESON_LVDS_VSO_HSTART;
	cfg->vso_end = MESON_LVDS_VSO_HSTART;
	cfg->vso_bline = MESON_LVDS_VSO_VSTART;
	cfg->vso_eline = MESON_LVDS_VSO_VSTART + 2;

	/* vendor lcd_tcon_config() with h_offset = v_offset = 0 */
	de_hstart = t->htotal - t->hdisplay - 1;
	de_vstart = t->vtotal - t->vdisplay;

	cfg->de_hs = de_hstart;
	cfg->de_he = (de_hstart + t->hdisplay) % t->htotal;
	cfg->de_vs = de_vstart;
	cfg->de_ve = (de_vstart + t->vdisplay - 1) % t->vtotal;

	cfg->hs_hs = (de_hstart + t->htotal - hsync_bp - hsync_width) % t->htotal;
	cfg->hs_he = (de_hstart + t->htotal - hsync_bp) % t->htotal;
	cfg->hs_vs = 0;
	cfg->hs_ve = t->vtotal - 1;

	cfg->vs_hs = cfg->hs_hs;	/* vsync_h_phase = 0 */
	cfg->vs_he = cfg->hs_hs;
	cfg->vs_vs = (de_vstart + t->vtotal - vsync_bp - vsync_width) % t->vtotal;
	cfg->vs_ve = (de_vstart + t->vtotal - vsync_bp) % t->vtotal;

	/* vendor set_tcon_lcd(), LVDS case; clock polarity 0 */
	cfg->pol_cntl = L_POL_TCON_DE_SEL | L_POL_TCON_VS_SEL | L_POL_TCON_HS_SEL;
	if (!t->hsync_high)
		cfg->pol_cntl |= L_POL_HS_LOW;
	if (!t->vsync_high)
		cfg->pol_cntl |= L_POL_VS_LOW;

	cfg->dith_cntl = cfg->bits == 8 ? 0x400 : 0x600;

	/* vendor set_control_lvds(): R, G, B on the default positions */
	cfg->pack_cntl = FIELD_PREP(LVDS_PACK_REPACK, cfg->repack) |
			 (cfg->pn_swap ? LVDS_PACK_PN_SWAP : 0) |
			 FIELD_PREP(LVDS_PACK_BITS, cfg->bits == 8 ? 1 : 2) |
			 FIELD_PREP(LVDS_PACK_R_SEL, 0) |
			 FIELD_PREP(LVDS_PACK_G_SEL, 1) |
			 FIELD_PREP(LVDS_PACK_B_SEL, 2);

	return 0;
}

/* HHI side, before the clocks: vendor vclk_set_lcd() */
static void meson_lvds_hhi_prepare(struct regmap *hhi)
{
	/* reset the eDP TX PHY, the DPHY is shared */
	regmap_write(hhi, HHI_EDP_TX_PHY_CNTL0, HHI_EDP_TX_PHY_RESET);
	/* select the LVDS/VID2 path, not eDP */
	regmap_update_bits(hhi, HHI_DSI_LVDS_EDP_CNTL1, HHI_DPHY_EDP_SEL, 0);
}

/* vendor set_pll_lcd(), after vclk_set_lcd(): serializer and FIFO clocks */
static void meson_lvds_set_clk_div(void __iomem *io_base, struct regmap *hhi)
{
	/* all serializers on, divide by 7 */
	regmap_write(hhi, HHI_LVDS_TX_PHY_CNTL0,
		     FIELD_PREP(HHI_LVDS_TX_PHY_SER_EN, 0x1f) |
		     FIELD_PREP(HHI_LVDS_TX_PHY_FIFO_CLK_SEL, 1));

	writel_bits_relaxed(LVDS_GEN_FIFO_CLK_DIV,
			    FIELD_PREP(LVDS_GEN_FIFO_CLK_DIV,
				       LVDS_GEN_FIFO_CLK_DIV7),
			    io_base + _REG(LVDS_GEN_CNTL));

	writel_bits_relaxed(LVDS_PHY_CLK_DIV_RESET_N, 0,
			    io_base + _REG(LVDS_PHY_CLK_CNTL));
	udelay(5);
	writel_bits_relaxed(LVDS_PHY_CLK_DIV_RESET_N, LVDS_PHY_CLK_DIV_RESET_N,
			    io_base + _REG(LVDS_PHY_CLK_CNTL));
}

/* vendor set_venc_lcd() */
static void meson_lvds_set_venc(void __iomem *io_base,
				const struct meson_lvds_cfg *cfg)
{
	writel_relaxed(0, io_base + _REG(ENCL_VIDEO_EN));

	/* VIU1 and VIU2 to ENCL */
	writel_relaxed(MESON_VIU_VPP_MUX_ENCL, io_base + _REG(VPU_VIU_VENC_MUX_CTRL));

	writel_relaxed(0, io_base + _REG(ENCL_VIDEO_MODE));
	writel_relaxed(MESON_LVDS_ENCL_MODE_ADV, io_base + _REG(ENCL_VIDEO_MODE_ADV));
	writel_relaxed(ENCL_VIDEO_FILT_CTRL_BYPASS_FILTER,
		       io_base + _REG(ENCL_VIDEO_FILT_CTRL));

	writel_relaxed(cfg->max_pxcnt, io_base + _REG(ENCL_VIDEO_MAX_PXCNT));
	writel_relaxed(cfg->max_lncnt, io_base + _REG(ENCL_VIDEO_MAX_LNCNT));
	writel_relaxed(cfg->havon_begin, io_base + _REG(ENCL_VIDEO_HAVON_BEGIN));
	writel_relaxed(cfg->havon_end, io_base + _REG(ENCL_VIDEO_HAVON_END));
	writel_relaxed(cfg->vavon_bline, io_base + _REG(ENCL_VIDEO_VAVON_BLINE));
	writel_relaxed(cfg->vavon_eline, io_base + _REG(ENCL_VIDEO_VAVON_ELINE));

	writel_relaxed(cfg->hso_begin, io_base + _REG(ENCL_VIDEO_HSO_BEGIN));
	writel_relaxed(cfg->hso_end, io_base + _REG(ENCL_VIDEO_HSO_END));
	writel_relaxed(cfg->vso_begin, io_base + _REG(ENCL_VIDEO_VSO_BEGIN));
	writel_relaxed(cfg->vso_end, io_base + _REG(ENCL_VIDEO_VSO_END));
	writel_relaxed(cfg->vso_bline, io_base + _REG(ENCL_VIDEO_VSO_BLINE));
	writel_relaxed(cfg->vso_eline, io_base + _REG(ENCL_VIDEO_VSO_ELINE));

	/* same as the vendor: RGB, no zero-blanking */
	writel_relaxed(ENCL_VIDEO_RGBIN_RGB, io_base + _REG(ENCL_VIDEO_RGBIN_CTRL));

	writel_relaxed(1, io_base + _REG(ENCL_VIDEO_EN));
}

/* vendor set_tcon_lcd(), LVDS case; the gamma table is loaded by the caller */
static void meson_lvds_set_tcon(void __iomem *io_base,
				const struct meson_lvds_cfg *cfg)
{
	writel_relaxed(MESON_LVDS_RGB_BASE, io_base + _REG(L_RGB_BASE_ADDR));
	writel_relaxed(MESON_LVDS_RGB_COEFF, io_base + _REG(L_RGB_COEFF_ADDR));
	writel_relaxed(cfg->dith_cntl, io_base + _REG(L_DITH_CNTL_ADDR));
	writel_relaxed(cfg->pol_cntl, io_base + _REG(L_POL_CNTL_ADDR));

	writel_relaxed(cfg->de_hs, io_base + _REG(L_DE_HS_ADDR));
	writel_relaxed(cfg->de_he, io_base + _REG(L_DE_HE_ADDR));
	writel_relaxed(cfg->de_vs, io_base + _REG(L_DE_VS_ADDR));
	writel_relaxed(cfg->de_ve, io_base + _REG(L_DE_VE_ADDR));

	writel_relaxed(cfg->hs_hs, io_base + _REG(L_HSYNC_HS_ADDR));
	writel_relaxed(cfg->hs_he, io_base + _REG(L_HSYNC_HE_ADDR));
	writel_relaxed(cfg->hs_vs, io_base + _REG(L_HSYNC_VS_ADDR));
	writel_relaxed(cfg->hs_ve, io_base + _REG(L_HSYNC_VE_ADDR));

	writel_relaxed(cfg->vs_hs, io_base + _REG(L_VSYNC_HS_ADDR));
	writel_relaxed(cfg->vs_he, io_base + _REG(L_VSYNC_HE_ADDR));
	writel_relaxed(cfg->vs_vs, io_base + _REG(L_VSYNC_VS_ADDR));
	writel_relaxed(cfg->vs_ve, io_base + _REG(L_VSYNC_VE_ADDR));
}

/* vendor set_control_lvds() */
static void meson_lvds_set_control(void __iomem *io_base,
				   const struct meson_lvds_cfg *cfg)
{
	writel_bits_relaxed(LVDS_GEN_FIFO_EN, 0, io_base + _REG(LVDS_GEN_CNTL));

	/* black while blanking */
	writel_relaxed(0, io_base + _REG(LVDS_BLANK_DATA_HI));
	writel_relaxed(0, io_base + _REG(LVDS_BLANK_DATA_LO));

	writel_relaxed(cfg->pack_cntl, io_base + _REG(LVDS_PACK_CNTL_ADDR));

	writel_bits_relaxed(LVDS_GEN_FIFO_WR_EN, LVDS_GEN_FIFO_WR_EN,
			    io_base + _REG(LVDS_GEN_CNTL));
}

/* vendor init_dphy() + init_phy_lvds() */
static void meson_lvds_phy_init(void __iomem *io_base, struct regmap *hhi,
				const struct meson_lvds_cfg *cfg)
{
	regmap_write(hhi, HHI_DSI_LVDS_EDP_CNTL0, HHI_DPHY_SEL_LVDS);

	writel_relaxed(0xfff, io_base + _REG(LVDS_SER_EN));
	writel_relaxed(0xffff, io_base + _REG(LVDS_PHY_CNTL0));
	writel_relaxed(0xff00, io_base + _REG(LVDS_PHY_CNTL1));
	writel_relaxed(0x007f, io_base + _REG(LVDS_PHY_CNTL4));

	regmap_write(hhi, HHI_DIF_CSI_PHY_CNTL1, meson_lvds_vswing[cfg->vswing]);
	regmap_write(hhi, HHI_DIF_CSI_PHY_CNTL2, HHI_DIF_CSI_PHY_CNTL2_ON);
	regmap_write(hhi, HHI_DIF_CSI_PHY_CNTL3, HHI_DIF_CSI_PHY_CNTL3_ON);
}

/* vendor lcd_ports_ctrl_lvds(ON) */
static void meson_lvds_ports_on(void __iomem *io_base, struct regmap *hhi,
				const struct meson_lvds_cfg *cfg)
{
	writel_bits_relaxed(LVDS_GEN_FIFO_EN, LVDS_GEN_FIFO_EN,
			    io_base + _REG(LVDS_GEN_CNTL));
	regmap_update_bits(hhi, HHI_DIF_CSI_PHY_CNTL3, HHI_DIF_CSI_PHY_LANES,
			   FIELD_PREP(HHI_DIF_CSI_PHY_LANES, cfg->lanes));
}

/* vendor lcd_ports_ctrl_lvds(OFF) and _disable_lcd_driver() */
static void meson_lvds_ports_off(void __iomem *io_base, struct regmap *hhi)
{
	regmap_write(hhi, HHI_DIF_CSI_PHY_CNTL1, 0);
	regmap_write(hhi, HHI_DIF_CSI_PHY_CNTL2, HHI_DIF_CSI_PHY_CNTL2_OFF);
	regmap_write(hhi, HHI_DIF_CSI_PHY_CNTL3, HHI_DIF_CSI_PHY_CNTL3_OFF);

	writel_bits_relaxed(LVDS_GEN_FIFO_EN, 0, io_base + _REG(LVDS_GEN_CNTL));
	writel_relaxed(0, io_base + _REG(ENCL_VIDEO_EN));
}

/* --- end of register programming --- */

enum {
	LVDS_CLK_VID2_PLL = 0,
	LVDS_CLK_VIID_PLL_PRE_DIV,
	LVDS_CLK_VIID_PLL_POST_DIV,
	LVDS_CLK_VIID_PLL,
	LVDS_CLK_VID2_PLL_FINAL_DIV,
	LVDS_CLK_VCLK2_IN_SEL,
	LVDS_CLK_VCLK2_DIV1,
	LVDS_CLK_CTS_ENCL_SEL,
	LVDS_CLK_NUM
};

static const char * const meson_lvds_clk_names[LVDS_CLK_NUM] = {
	[LVDS_CLK_VID2_PLL]		= "vid2_pll",
	[LVDS_CLK_VIID_PLL_PRE_DIV]	= "viid_pll_pre_div",
	[LVDS_CLK_VIID_PLL_POST_DIV]	= "viid_pll_post_div",
	[LVDS_CLK_VIID_PLL]		= "viid_pll",
	[LVDS_CLK_VID2_PLL_FINAL_DIV]	= "vid2_pll_final_div",
	[LVDS_CLK_VCLK2_IN_SEL]		= "vclk2_in_sel",
	[LVDS_CLK_VCLK2_DIV1]		= "vclk2_div1",
	[LVDS_CLK_CTS_ENCL_SEL]		= "cts_encl_sel",
};

/*
 * Interface gates the vendor switches on for the LCD (switch_lcd_mod_gate),
 * in its order. vclk2_encl is needed for the ENCL vsync interrupt.
 */
static const char * const meson_lvds_gate_names[] = {
	"vclk2_encl",
	"vclk2_vencl",
	"edp",
};

/* HHI_VIID_DIVIDER_CNTL and HHI_VIID_CLK_CNTL reset lines */
enum {
	LVDS_RESET_VIID_PLL_PRE = 0,
	LVDS_RESET_VIID_PLL_POST,
	LVDS_RESET_VIID_PLL_SOFT_PRE,
	LVDS_RESET_VIID_PLL_SOFT_POST,
	LVDS_RESET_VCLK2_SOFT,
	LVDS_RESET_NUM
};

struct meson_encoder_lvds {
	struct drm_encoder encoder;
	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;
	struct meson_drm *priv;
	struct regmap *hhi;

	struct clk_bulk_data clks[LVDS_CLK_NUM];
	struct clk_bulk_data gates[ARRAY_SIZE(meson_lvds_gate_names)];
	struct clk *cts_encl;
	struct reset_control_bulk_data resets[LVDS_RESET_NUM];
	bool has_resets;

	struct clk *exclusive[LVDS_CLK_NUM];
	unsigned int num_exclusive;
	bool clk_enabled;
};

#define bridge_to_meson_encoder_lvds(x) \
	container_of(x, struct meson_encoder_lvds, bridge)

static void meson_lvds_timing_from_mode(const struct drm_display_mode *mode,
					struct meson_lvds_timing *t)
{
	t->clock = mode->clock;
	t->hdisplay = mode->hdisplay;
	t->hsync_start = mode->hsync_start;
	t->hsync_end = mode->hsync_end;
	t->htotal = mode->htotal;
	t->vdisplay = mode->vdisplay;
	t->vsync_start = mode->vsync_start;
	t->vsync_end = mode->vsync_end;
	t->vtotal = mode->vtotal;
	t->hsync_high = !!(mode->flags & DRM_MODE_FLAG_PHSYNC);
	t->vsync_high = !!(mode->flags & DRM_MODE_FLAG_PVSYNC);
}

static void meson_lvds_dump_cfg(struct meson_encoder_lvds *lvds,
				const struct meson_lvds_cfg *cfg)
{
	struct drm_device *drm = lvds->priv->drm;

	drm_dbg_kms(drm, "LVDS: %u bit, repack %u, pn_swap %u, vswing %u (0x%03x), lanes 0x%02x\n",
		    cfg->bits, cfg->repack, cfg->pn_swap, cfg->vswing,
		    meson_lvds_vswing[cfg->vswing], cfg->lanes);
	drm_dbg_kms(drm, "LVDS: pixel %llu Hz, lvds %llu Hz, pll %llu Hz, pre_div %u\n",
		    cfg->pixel_rate, cfg->lvds_rate, cfg->pll_rate, cfg->pre_div);
	drm_dbg_kms(drm, "ENCL: max_px %u max_ln %u havon %u..%u vavon %u..%u\n",
		    cfg->max_pxcnt, cfg->max_lncnt, cfg->havon_begin,
		    cfg->havon_end, cfg->vavon_bline, cfg->vavon_eline);
	drm_dbg_kms(drm, "ENCL: hso %u..%u vso %u..%u line %u..%u\n",
		    cfg->hso_begin, cfg->hso_end, cfg->vso_begin, cfg->vso_end,
		    cfg->vso_bline, cfg->vso_eline);
	drm_dbg_kms(drm, "TCON: pol 0x%02x dith 0x%03x de %u..%u/%u..%u\n",
		    cfg->pol_cntl, cfg->dith_cntl,
		    cfg->de_hs, cfg->de_he, cfg->de_vs, cfg->de_ve);
	drm_dbg_kms(drm, "TCON: hs %u..%u/%u..%u vs %u..%u/%u..%u\n",
		    cfg->hs_hs, cfg->hs_he, cfg->hs_vs, cfg->hs_ve,
		    cfg->vs_hs, cfg->vs_he, cfg->vs_vs, cfg->vs_ve);
	drm_dbg_kms(drm, "LVDS: pack_cntl 0x%04x\n", cfg->pack_cntl);
}

static void meson_lvds_dump_regs(struct meson_encoder_lvds *lvds)
{
	static const struct { const char *name; u32 reg; } vregs[] = {
		{ "ENCL_VIDEO_EN", ENCL_VIDEO_EN },
		{ "ENCL_VIDEO_MAX_PXCNT", ENCL_VIDEO_MAX_PXCNT },
		{ "ENCL_VIDEO_MAX_LNCNT", ENCL_VIDEO_MAX_LNCNT },
		{ "ENCL_VIDEO_HAVON_BEGIN", ENCL_VIDEO_HAVON_BEGIN },
		{ "ENCL_VIDEO_VAVON_BLINE", ENCL_VIDEO_VAVON_BLINE },
		{ "VPU_VIU_VENC_MUX_CTRL", VPU_VIU_VENC_MUX_CTRL },
		{ "L_POL_CNTL", L_POL_CNTL_ADDR },
		{ "L_DE_HS", L_DE_HS_ADDR },
		{ "LVDS_PACK_CNTL", LVDS_PACK_CNTL_ADDR },
		{ "LVDS_GEN_CNTL", LVDS_GEN_CNTL },
		{ "LVDS_PHY_CLK_CNTL", LVDS_PHY_CLK_CNTL },
		{ "VENC_INTCTRL", VENC_INTCTRL },
	};
	static const struct { const char *name; u32 reg; } hregs[] = {
		{ "HHI_DSI_LVDS_EDP_CNTL0", HHI_DSI_LVDS_EDP_CNTL0 },
		{ "HHI_DSI_LVDS_EDP_CNTL1", HHI_DSI_LVDS_EDP_CNTL1 },
		{ "HHI_DIF_CSI_PHY_CNTL1", HHI_DIF_CSI_PHY_CNTL1 },
		{ "HHI_DIF_CSI_PHY_CNTL2", HHI_DIF_CSI_PHY_CNTL2 },
		{ "HHI_DIF_CSI_PHY_CNTL3", HHI_DIF_CSI_PHY_CNTL3 },
		{ "HHI_LVDS_TX_PHY_CNTL0", HHI_LVDS_TX_PHY_CNTL0 },
	};
	struct drm_device *drm = lvds->priv->drm;
	void __iomem *io_base = lvds->priv->io_base;
	unsigned int i, val;

	for (i = 0; i < ARRAY_SIZE(vregs); i++)
		drm_dbg_kms(drm, "  VCBUS[0x%04x] %-24s = 0x%08x\n", vregs[i].reg,
			    vregs[i].name, readl_relaxed(io_base + _REG(vregs[i].reg)));

	for (i = 0; i < ARRAY_SIZE(hregs); i++) {
		regmap_read(lvds->hhi, hregs[i].reg, &val);
		drm_dbg_kms(drm, "  CBUS[0x%04x] %-24s = 0x%08x\n",
			    0x1000 + hregs[i].reg / 4, hregs[i].name, val);
	}
}

static void meson_lvds_clk_release(struct meson_encoder_lvds *lvds)
{
	while (lvds->num_exclusive)
		clk_rate_exclusive_put(lvds->exclusive[--lvds->num_exclusive]);
}

static int meson_lvds_set_rate_exclusive(struct meson_encoder_lvds *lvds,
					 unsigned int id, unsigned long long rate)
{
	struct clk *clk = lvds->clks[id].clk;
	int ret;

	ret = clk_set_rate_exclusive(clk, rate);
	if (ret) {
		drm_err(lvds->priv->drm, "LVDS: failed to set %s to %llu Hz: %d\n",
			meson_lvds_clk_names[id], rate, ret);
		return ret;
	}

	lvds->exclusive[lvds->num_exclusive++] = clk;

	drm_dbg_kms(lvds->priv->drm, "LVDS: %s = %lu Hz (requested %llu)\n",
		    meson_lvds_clk_names[id], clk_get_rate(clk), rate);

	return 0;
}

static int meson_lvds_set_parent(struct meson_encoder_lvds *lvds,
				 unsigned int id, unsigned int parent)
{
	int ret;

	ret = clk_set_parent(lvds->clks[id].clk, lvds->clks[parent].clk);
	if (ret)
		drm_err(lvds->priv->drm, "LVDS: failed to set the parent of %s to %s: %d\n",
			meson_lvds_clk_names[id], meson_lvds_clk_names[parent],
			ret);
	else
		drm_dbg_kms(lvds->priv->drm, "LVDS: %s parent = %s\n",
			    meson_lvds_clk_names[id], meson_lvds_clk_names[parent]);

	return ret;
}

/*
 * The dividers use CLK_SET_RATE_PARENT and the VID2 PLL is fractional, so a
 * plain clk_set_rate(cts_encl) may pick another divider split that gives the
 * pixel clock but not the 7x LVDS bit clock. Fix the topology step by step.
 */
static int meson_lvds_clk_enable(struct meson_encoder_lvds *lvds,
				 const struct meson_lvds_cfg *cfg)
{
	int ret;

	ret = meson_lvds_set_rate_exclusive(lvds, LVDS_CLK_VID2_PLL, cfg->pll_rate);
	if (ret)
		goto err;

	ret = meson_lvds_set_rate_exclusive(lvds, LVDS_CLK_VIID_PLL_PRE_DIV,
					    cfg->lvds_rate);
	if (ret)
		goto err;

	ret = meson_lvds_set_parent(lvds, LVDS_CLK_VIID_PLL,
				    LVDS_CLK_VIID_PLL_POST_DIV);
	if (ret)
		goto err;

	ret = meson_lvds_set_rate_exclusive(lvds, LVDS_CLK_VIID_PLL_POST_DIV,
					    cfg->pixel_rate);
	if (ret)
		goto err;

	ret = meson_lvds_set_rate_exclusive(lvds, LVDS_CLK_VID2_PLL_FINAL_DIV,
					    cfg->pixel_rate);
	if (ret)
		goto err;

	ret = meson_lvds_set_parent(lvds, LVDS_CLK_VCLK2_IN_SEL,
				    LVDS_CLK_VID2_PLL_FINAL_DIV);
	if (ret)
		goto err;

	ret = meson_lvds_set_parent(lvds, LVDS_CLK_CTS_ENCL_SEL,
				    LVDS_CLK_VCLK2_DIV1);
	if (ret)
		goto err;

	/* vendor vclk_set_lcd(): pulse the VIID divider and VCLK2 resets */
	if (lvds->has_resets) {
		reset_control_bulk_assert(LVDS_RESET_NUM, lvds->resets);
		udelay(5);
		reset_control_bulk_deassert(LVDS_RESET_NUM, lvds->resets);
		udelay(5);
	}

	ret = clk_prepare_enable(lvds->cts_encl);
	if (ret) {
		drm_err(lvds->priv->drm, "LVDS: failed to enable cts_encl: %d\n", ret);
		goto err;
	}

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(lvds->gates), lvds->gates);
	if (ret) {
		drm_err(lvds->priv->drm, "LVDS: failed to enable the gates: %d\n", ret);
		clk_disable_unprepare(lvds->cts_encl);
		goto err;
	}

	lvds->clk_enabled = true;

	drm_dbg_kms(lvds->priv->drm, "LVDS: cts_encl = %lu Hz\n",
		    clk_get_rate(lvds->cts_encl));

	return 0;

err:
	meson_lvds_clk_release(lvds);
	return ret;
}

static void meson_lvds_clk_disable(struct meson_encoder_lvds *lvds)
{
	if (lvds->clk_enabled) {
		clk_bulk_disable_unprepare(ARRAY_SIZE(lvds->gates), lvds->gates);
		clk_disable_unprepare(lvds->cts_encl);
		lvds->clk_enabled = false;
	}

	meson_lvds_clk_release(lvds);
}

static u32 meson_encoder_lvds_bus_format(struct drm_bridge_state *bridge_state,
					 struct drm_connector *connector)
{
	if (bridge_state && bridge_state->output_bus_cfg.format &&
	    bridge_state->output_bus_cfg.format != MEDIA_BUS_FMT_FIXED)
		return bridge_state->output_bus_cfg.format;

	if (connector && connector->display_info.num_bus_formats)
		return connector->display_info.bus_formats[0];

	return MEDIA_BUS_FMT_RGB888_1X7X4_SPWG;
}

static int meson_encoder_lvds_attach(struct drm_bridge *bridge,
				     struct drm_encoder *encoder,
				     enum drm_bridge_attach_flags flags)
{
	struct meson_encoder_lvds *lvds = bridge_to_meson_encoder_lvds(bridge);

	return drm_bridge_attach(encoder, lvds->next_bridge, &lvds->bridge, flags);
}

static enum drm_mode_status
meson_encoder_lvds_mode_valid(struct drm_bridge *bridge,
			      const struct drm_display_info *info,
			      const struct drm_display_mode *mode)
{
	struct meson_lvds_timing t;
	struct meson_lvds_cfg cfg;
	u32 bus_format = info->num_bus_formats ? info->bus_formats[0] :
			 MEDIA_BUS_FMT_RGB888_1X7X4_SPWG;

	if (mode->flags & (DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLSCAN |
			   DRM_MODE_FLAG_DBLCLK))
		return MODE_BAD;

	meson_lvds_timing_from_mode(mode, &t);
	if (meson_lvds_calc(&t, bus_format, &cfg))
		return MODE_BAD;

	return MODE_OK;
}

static int meson_encoder_lvds_atomic_check(struct drm_bridge *bridge,
					   struct drm_bridge_state *bridge_state,
					   struct drm_crtc_state *crtc_state,
					   struct drm_connector_state *conn_state)
{
	struct meson_encoder_lvds *lvds = bridge_to_meson_encoder_lvds(bridge);
	struct meson_lvds_timing t;
	struct meson_lvds_cfg cfg;
	u32 bus_format;
	int ret;

	bus_format = meson_encoder_lvds_bus_format(bridge_state,
						   conn_state->connector);

	meson_lvds_timing_from_mode(&crtc_state->adjusted_mode, &t);
	ret = meson_lvds_calc(&t, bus_format, &cfg);
	if (ret)
		drm_dbg_kms(lvds->priv->drm,
			    "LVDS: mode " DRM_MODE_FMT " bus format 0x%04x rejected: %d\n",
			    DRM_MODE_ARG(&crtc_state->adjusted_mode), bus_format, ret);

	return ret;
}

static void meson_encoder_lvds_atomic_enable(struct drm_bridge *bridge,
					     struct drm_atomic_commit *state)
{
	struct meson_encoder_lvds *lvds = bridge_to_meson_encoder_lvds(bridge);
	struct meson_drm *priv = lvds->priv;
	void __iomem *io_base = priv->io_base;
	struct drm_bridge_state *bridge_state;
	struct drm_connector_state *conn_state;
	struct drm_crtc_state *crtc_state;
	struct drm_connector *connector;
	struct meson_lvds_timing t;
	struct meson_lvds_cfg cfg;
	u32 bus_format;
	int ret;

	connector = drm_atomic_get_new_connector_for_encoder(state, bridge->encoder);
	if (WARN_ON(!connector))
		return;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (WARN_ON(!conn_state))
		return;

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (WARN_ON(!crtc_state))
		return;

	bridge_state = drm_atomic_get_new_bridge_state(state, bridge);
	bus_format = meson_encoder_lvds_bus_format(bridge_state, connector);

	drm_dbg_kms(priv->drm, "LVDS: enable " DRM_MODE_FMT ", bus format 0x%04x\n",
		    DRM_MODE_ARG(&crtc_state->adjusted_mode), bus_format);

	meson_lvds_timing_from_mode(&crtc_state->adjusted_mode, &t);
	ret = meson_lvds_calc(&t, bus_format, &cfg);
	if (WARN_ON(ret))
		return;

	meson_lvds_dump_cfg(lvds, &cfg);

	meson_lvds_hhi_prepare(lvds->hhi);

	ret = meson_lvds_clk_enable(lvds, &cfg);
	if (ret)
		return;

	meson_lvds_set_clk_div(io_base, lvds->hhi);
	meson_lvds_set_venc(io_base, &cfg);
	meson_encl_load_gamma(priv);
	meson_lvds_set_tcon(io_base, &cfg);
	meson_lvds_set_control(io_base, &cfg);
	meson_lvds_phy_init(io_base, lvds->hhi, &cfg);
	meson_lvds_ports_on(io_base, lvds->hhi, &cfg);

	priv->venc.current_mode = MESON_VENC_MODE_LVDS;

	/* the CRTC may have armed the vsync IRQ for the previous encoder */
	if (readl_relaxed(io_base + _REG(VENC_INTCTRL)))
		meson_venc_enable_vsync(priv);

	meson_lvds_dump_regs(lvds);
}

static void meson_encoder_lvds_atomic_disable(struct drm_bridge *bridge,
					      struct drm_atomic_commit *state)
{
	struct meson_encoder_lvds *lvds = bridge_to_meson_encoder_lvds(bridge);
	struct meson_drm *priv = lvds->priv;

	drm_dbg_kms(priv->drm, "LVDS: disable\n");

	meson_lvds_ports_off(priv->io_base, lvds->hhi);
	meson_lvds_clk_disable(lvds);

	meson_lvds_dump_regs(lvds);
}

static const struct drm_bridge_funcs meson_encoder_lvds_bridge_funcs = {
	.attach = meson_encoder_lvds_attach,
	.mode_valid = meson_encoder_lvds_mode_valid,
	.atomic_check = meson_encoder_lvds_atomic_check,
	.atomic_enable = meson_encoder_lvds_atomic_enable,
	.atomic_disable = meson_encoder_lvds_atomic_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
};

static int meson_encoder_lvds_get_resources(struct meson_encoder_lvds *lvds)
{
	struct meson_drm *priv = lvds->priv;
	struct device *dev = priv->dev;
	unsigned int i;
	int ret;

	lvds->hhi = syscon_regmap_lookup_by_phandle(dev->of_node,
						    "amlogic,hhi-sysctrl");
	if (IS_ERR(lvds->hhi))
		return dev_err_probe(dev, PTR_ERR(lvds->hhi),
				     "LVDS: failed to get the HHI regmap\n");

	for (i = 0; i < LVDS_CLK_NUM; i++)
		lvds->clks[i].id = meson_lvds_clk_names[i];

	ret = devm_clk_bulk_get(dev, LVDS_CLK_NUM, lvds->clks);
	if (ret)
		return dev_err_probe(dev, ret, "LVDS: failed to get the clocks\n");

	for (i = 0; i < ARRAY_SIZE(meson_lvds_gate_names); i++)
		lvds->gates[i].id = meson_lvds_gate_names[i];

	ret = devm_clk_bulk_get_optional(dev, ARRAY_SIZE(lvds->gates),
					 lvds->gates);
	if (ret)
		return dev_err_probe(dev, ret, "LVDS: failed to get the gates\n");

	lvds->cts_encl = priv->vid_clks[VPU_VID_CLK_CTS_ENCL].clk;
	if (!lvds->cts_encl)
		return dev_err_probe(dev, -ENOENT, "LVDS: no cts_encl clock\n");

	lvds->resets[LVDS_RESET_VIID_PLL_PRE].id = "viid_pll_pre";
	lvds->resets[LVDS_RESET_VIID_PLL_POST].id = "viid_pll_post";
	lvds->resets[LVDS_RESET_VIID_PLL_SOFT_PRE].id = "viid_pll_soft_pre";
	lvds->resets[LVDS_RESET_VIID_PLL_SOFT_POST].id = "viid_pll_soft_post";
	lvds->resets[LVDS_RESET_VCLK2_SOFT].id = "vclk2_soft";

	ret = devm_reset_control_bulk_get_optional_exclusive(dev, LVDS_RESET_NUM,
							     lvds->resets);
	if (ret)
		return dev_err_probe(dev, ret, "LVDS: failed to get the resets\n");

	lvds->has_resets = true;
	for (i = 0; i < LVDS_RESET_NUM; i++)
		if (!lvds->resets[i].rstc)
			lvds->has_resets = false;

	if (!lvds->has_resets)
		dev_warn(dev, "LVDS: VIID divider resets missing, a cold start may fail\n");

	return 0;
}

int meson_encoder_lvds_probe(struct meson_drm *priv)
{
	struct meson_encoder_lvds *lvds;
	struct drm_connector *connector;
	struct device_node *remote;
	int ret;

	/* LVDS output: port 3 */
	remote = of_graph_get_remote_node(priv->dev->of_node, 3, 0);
	if (!remote) {
		dev_dbg(priv->dev, "LVDS output not available\n");
		return 0;
	}
	of_node_put(remote);

	lvds = devm_drm_bridge_alloc(priv->dev, struct meson_encoder_lvds,
				     bridge, &meson_encoder_lvds_bridge_funcs);
	if (IS_ERR(lvds))
		return PTR_ERR(lvds);

	lvds->priv = priv;

	ret = meson_encoder_lvds_get_resources(lvds);
	if (ret)
		return ret;

	/* the panel, wrapped in a panel bridge */
	lvds->next_bridge = devm_drm_of_get_bridge(priv->dev, priv->dev->of_node,
						   3, 0);
	if (IS_ERR(lvds->next_bridge))
		return dev_err_probe(priv->dev, PTR_ERR(lvds->next_bridge),
				     "LVDS: failed to find the panel\n");

	lvds->bridge.of_node = priv->dev->of_node;
	lvds->bridge.type = DRM_MODE_CONNECTOR_LVDS;

	drm_bridge_add(&lvds->bridge);

	ret = drm_simple_encoder_init(priv->drm, &lvds->encoder,
				      DRM_MODE_ENCODER_LVDS);
	if (ret) {
		dev_err_probe(priv->dev, ret, "LVDS: failed to init the encoder\n");
		goto err_remove_bridge;
	}

	lvds->encoder.possible_crtcs = BIT(0);

	ret = drm_bridge_attach(&lvds->encoder, &lvds->bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret) {
		dev_err_probe(priv->dev, ret, "LVDS: failed to attach the bridge\n");
		goto err_remove_bridge;
	}

	connector = drm_bridge_connector_init(priv->drm, &lvds->encoder);
	if (IS_ERR(connector)) {
		ret = dev_err_probe(priv->dev, PTR_ERR(connector),
				    "LVDS: failed to create the connector\n");
		goto err_remove_bridge;
	}

	priv->encoders[MESON_ENC_LVDS] = lvds;

	dev_dbg(priv->dev, "LVDS encoder initialized\n");

	return 0;

err_remove_bridge:
	drm_bridge_remove(&lvds->bridge);
	return ret;
}

void meson_encoder_lvds_remove(struct meson_drm *priv)
{
	struct meson_encoder_lvds *lvds = priv->encoders[MESON_ENC_LVDS];

	if (lvds) {
		drm_bridge_remove(&lvds->bridge);
		priv->encoders[MESON_ENC_LVDS] = NULL;
	}
}
