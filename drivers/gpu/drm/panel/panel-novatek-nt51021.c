// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT51021 MIPI DSI panel driver
 *
 * The NT51021 is a timing controller for 1200x1920 video mode panels. The
 * init sequence and the timings of the EDT 10.1" module come from the vendor
 * device tree of the Crestron TSW-1070 ("nt51021b_1200p_video").
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

/* Write 0xa5 to unlock the vendor registers, 0x00 to lock them. */
#define NT51021_UNLOCK			0x8f

struct nt51021_panel_desc {
	const struct drm_display_mode mode;
	unsigned int lanes;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	void (*init)(struct mipi_dsi_multi_context *dsi_ctx);
	/* Reset sequence: deassert, assert, deassert, each step in ms */
	unsigned int reset_ms[3];
};

struct nt51021 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	const struct nt51021_panel_desc *desc;
	struct regulator_bulk_data *supplies;
	struct gpio_desc *reset_gpio;
	enum drm_panel_orientation orientation;
};

static const struct regulator_bulk_data nt51021_supplies[] = {
	{ .supply = "vccio" },
	{ .supply = "vdd" },
};

static inline struct nt51021 *to_nt51021(struct drm_panel *panel)
{
	return container_of(panel, struct nt51021, panel);
}

static void nt51021_reset(struct nt51021 *ctx)
{
	const unsigned int *ms = ctx->desc->reset_ms;

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(ms[0]);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(ms[1]);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(ms[2]);
}

static int nt51021_prepare(struct drm_panel *panel)
{
	struct nt51021 *ctx = to_nt51021(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(nt51021_supplies), ctx->supplies);
	if (ret < 0)
		return ret;

	nt51021_reset(ctx);

	ctx->desc->init(&dsi_ctx);
	if (dsi_ctx.accum_err) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(nt51021_supplies), ctx->supplies);
	}

	return dsi_ctx.accum_err;
}

static int nt51021_disable(struct drm_panel *panel)
{
	struct nt51021 *ctx = to_nt51021(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int nt51021_unprepare(struct drm_panel *panel)
{
	struct nt51021 *ctx = to_nt51021(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(nt51021_supplies), ctx->supplies);

	return 0;
}

static int nt51021_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct nt51021 *ctx = to_nt51021(panel);

	return drm_connector_helper_get_modes_fixed(connector, &ctx->desc->mode);
}

static enum drm_panel_orientation nt51021_get_orientation(struct drm_panel *panel)
{
	struct nt51021 *ctx = to_nt51021(panel);

	return ctx->orientation;
}

static const struct drm_panel_funcs nt51021_panel_funcs = {
	.prepare = nt51021_prepare,
	.disable = nt51021_disable,
	.unprepare = nt51021_unprepare,
	.get_modes = nt51021_get_modes,
	.get_orientation = nt51021_get_orientation,
};

/*
 * The vendor sends the two-byte commands as DCS long writes. The driver
 * sends them as DCS short writes with a parameter, like the JD9366 module of
 * the same board family, which works that way.
 */
static void crestron_tswxx70_nt51021b_init(struct mipi_dsi_multi_context *dsi_ctx)
{
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, NT51021_UNLOCK, 0xa5);
	mipi_dsi_msleep(dsi_ctx, 5);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MIPI_DCS_SOFT_RESET, 0x00);
	mipi_dsi_msleep(dsi_ctx, 20);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, NT51021_UNLOCK, 0xa5);
	mipi_dsi_msleep(dsi_ctx, 5);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x83, 0xaa);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x84, 0x11);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xa9, 0x4b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x83, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x84, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, NT51021_UNLOCK, 0x00);

	mipi_dsi_dcs_exit_sleep_mode_multi(dsi_ctx);
	mipi_dsi_msleep(dsi_ctx, 150);
	mipi_dsi_dcs_set_display_on_multi(dsi_ctx);
	mipi_dsi_msleep(dsi_ctx, 50);
}

/* EDT 10.1" module of the Crestron TSW-1070 (vendor "nt51021b_1200p_video"). */
static const struct nt51021_panel_desc crestron_tswxx70_nt51021b_desc = {
	.mode = {
		.clock		= (1200 + 80 + 4 + 60) * (1920 + 35 + 1 + 25) * 60 / 1000,

		.hdisplay	= 1200,
		.hsync_start	= 1200 + 80,
		.hsync_end	= 1200 + 80 + 4,
		.htotal		= 1200 + 80 + 4 + 60,

		.vdisplay	= 1920,
		.vsync_start	= 1920 + 35,
		.vsync_end	= 1920 + 35 + 1,
		.vtotal		= 1920 + 35 + 1 + 25,

		.width_mm	= 135,
		.height_mm	= 216,
		.type		= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
	},
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
		      MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS |
		      MIPI_DSI_MODE_LPM,
	.init = crestron_tswxx70_nt51021b_init,
	.reset_ms = { 30, 20, 30 },
};

static int nt51021_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct nt51021 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct nt51021, panel,
				   &nt51021_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->desc = of_device_get_match_data(dev);
	if (!ctx->desc)
		return -ENODEV;

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(nt51021_supplies),
					    nt51021_supplies, &ctx->supplies);
	if (ret < 0)
		return ret;

	/* Hold the panel in reset until prepare() runs the reset sequence. */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ret = of_drm_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get orientation\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = ctx->desc->lanes;
	dsi->format = ctx->desc->format;
	dsi->mode_flags = ctx->desc->mode_flags;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	ctx->panel.prepare_prev_first = true;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void nt51021_remove(struct mipi_dsi_device *dsi)
{
	struct nt51021 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id nt51021_of_match[] = {
	{
		.compatible = "crestron,tswxx70-nt51021b-edt",
		.data = &crestron_tswxx70_nt51021b_desc,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nt51021_of_match);

static struct mipi_dsi_driver nt51021_driver = {
	.probe = nt51021_probe,
	.remove = nt51021_remove,
	.driver = {
		.name = "panel-novatek-nt51021",
		.of_match_table = nt51021_of_match,
	},
};
module_mipi_dsi_driver(nt51021_driver);

MODULE_DESCRIPTION("Novatek NT51021 MIPI DSI panel driver");
MODULE_LICENSE("GPL");
