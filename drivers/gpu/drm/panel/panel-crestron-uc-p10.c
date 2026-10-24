// SPDX-License-Identifier: GPL-2.0-only
/*
 * Crestron UC-P10 built-in MIPI-DSI panel.
 *
 * The panel is a 1200x1920 video-mode panel with a Himax HX8279-family
 * controller: register 0xb0 selects the page, page 1 holds the GOA mux,
 * page 2 the analog gamma and page 3 the GOA timing.
 *
 * The stock firmware sends a flat list of DCS short writes. The list and the
 * delays come from the "panel-init-sequence-himax" and
 * "display-timings-himax" properties of the stock 1.0.6.20 device tree.
 * The structured panel-himax-hx8279 driver uses generic writes and writes
 * registers that this list leaves at their reset value, so this driver
 * replays the stock list as it is.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct ucp10_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *supply;
	struct gpio_desc *reset_gpio;
	enum drm_panel_orientation orientation;
};

/*
 * One entry for each stock init command. The stock list stores each command
 * as [type, delay_ms, length, payload]. This table keeps the payload (the DCS
 * command byte and its parameter) and the delay after the command.
 * mipi_dsi_dcs_write_buffer() selects the packet type from the payload
 * length. A 2-byte payload goes out as type 0x15 and a 1-byte payload as
 * type 0x05, as in the stock list. The last two entries are the standard
 * exit-sleep (0x11) and set-display-on (0x29) commands.
 */
struct ucp10_init_cmd {
	u8 delay;
	u8 len;
	u8 data[2];
};

#define UCP10_CMD(d, ...) {						\
	.delay = (d),							\
	.len = sizeof((const u8[]){ __VA_ARGS__ }),			\
	.data = { __VA_ARGS__ },					\
}

static const struct ucp10_init_cmd ucp10_init_cmds[] = {
	UCP10_CMD(0, 0xb0, 0x01),
	UCP10_CMD(0, 0xc3, 0x0f),
	UCP10_CMD(0, 0xc4, 0x00),
	UCP10_CMD(0, 0xc5, 0x00),
	UCP10_CMD(0, 0xc6, 0x00),
	UCP10_CMD(0, 0xc7, 0x00),
	UCP10_CMD(0, 0xc8, 0x0d),
	UCP10_CMD(0, 0xc9, 0x12),
	UCP10_CMD(0, 0xca, 0x11),
	UCP10_CMD(0, 0xcd, 0x1d),
	UCP10_CMD(0, 0xce, 0x1b),
	UCP10_CMD(0, 0xcf, 0x0b),
	UCP10_CMD(0, 0xd0, 0x09),
	UCP10_CMD(0, 0xd1, 0x07),
	UCP10_CMD(0, 0xd2, 0x05),
	UCP10_CMD(0, 0xd3, 0x01),
	UCP10_CMD(0, 0xd7, 0x10),
	UCP10_CMD(0, 0xd8, 0x00),
	UCP10_CMD(0, 0xd9, 0x00),
	UCP10_CMD(0, 0xda, 0x00),
	UCP10_CMD(0, 0xdb, 0x00),
	UCP10_CMD(0, 0xdc, 0x0e),
	UCP10_CMD(0, 0xdd, 0x12),
	UCP10_CMD(0, 0xde, 0x11),
	UCP10_CMD(0, 0xe1, 0x1e),
	UCP10_CMD(0, 0xe2, 0x1c),
	UCP10_CMD(0, 0xe3, 0x0c),
	UCP10_CMD(0, 0xe4, 0x0a),
	UCP10_CMD(0, 0xe5, 0x08),
	UCP10_CMD(0, 0xe6, 0x06),
	UCP10_CMD(0, 0xe7, 0x02),
	UCP10_CMD(0, 0xb0, 0x03),
	UCP10_CMD(0, 0xbe, 0x03),
	UCP10_CMD(0, 0xcc, 0x44),
	UCP10_CMD(0, 0xc8, 0x07),
	UCP10_CMD(0, 0xc9, 0x05),
	UCP10_CMD(0, 0xca, 0x42),
	UCP10_CMD(0, 0xcd, 0x3e),
	UCP10_CMD(0, 0xcf, 0x60),
	UCP10_CMD(0, 0xd2, 0x04),
	UCP10_CMD(0, 0xd3, 0x04),
	UCP10_CMD(0, 0xd4, 0x01),
	UCP10_CMD(0, 0xd5, 0x00),
	UCP10_CMD(0, 0xd6, 0x03),
	UCP10_CMD(0, 0xd7, 0x04),
	UCP10_CMD(0, 0xd9, 0x01),
	UCP10_CMD(0, 0xdb, 0x01),
	UCP10_CMD(0, 0xe4, 0xf0),
	UCP10_CMD(0, 0xe5, 0x0a),
	UCP10_CMD(0, 0xb0, 0x00),
	UCP10_CMD(0, 0xbd, 0x46),
	UCP10_CMD(0, 0xba, 0x8f),
	UCP10_CMD(0, 0xc2, 0x08),
	UCP10_CMD(0, 0xc4, 0x10),
	UCP10_CMD(0, 0xb0, 0x02),
	UCP10_CMD(0, 0xc0, 0x00),
	UCP10_CMD(0, 0xc1, 0x0a),
	UCP10_CMD(0, 0xc2, 0x20),
	UCP10_CMD(0, 0xc3, 0x24),
	UCP10_CMD(0, 0xc4, 0x23),
	UCP10_CMD(0, 0xc5, 0x29),
	UCP10_CMD(0, 0xc6, 0x23),
	UCP10_CMD(0, 0xc7, 0x1c),
	UCP10_CMD(0, 0xc8, 0x19),
	UCP10_CMD(0, 0xc9, 0x17),
	UCP10_CMD(0, 0xca, 0x17),
	UCP10_CMD(0, 0xcb, 0x18),
	UCP10_CMD(0, 0xcc, 0x1a),
	UCP10_CMD(0, 0xcd, 0x1e),
	UCP10_CMD(0, 0xce, 0x20),
	UCP10_CMD(0, 0xcf, 0x23),
	UCP10_CMD(0, 0xd0, 0x07),
	UCP10_CMD(0, 0xd1, 0x00),
	UCP10_CMD(0, 0xd2, 0x00),
	UCP10_CMD(0, 0xd3, 0x0a),
	UCP10_CMD(0, 0xd4, 0x13),
	UCP10_CMD(0, 0xd5, 0x1c),
	UCP10_CMD(0, 0xd6, 0x1a),
	UCP10_CMD(0, 0xd7, 0x13),
	UCP10_CMD(0, 0xd8, 0x17),
	UCP10_CMD(0, 0xd9, 0x1c),
	UCP10_CMD(0, 0xda, 0x19),
	UCP10_CMD(0, 0xdb, 0x17),
	UCP10_CMD(0, 0xdc, 0x17),
	UCP10_CMD(0, 0xdd, 0x18),
	UCP10_CMD(0, 0xde, 0x1a),
	UCP10_CMD(0, 0xdf, 0x1e),
	UCP10_CMD(0, 0xe0, 0x20),
	UCP10_CMD(0, 0xe1, 0x23),
	UCP10_CMD(0, 0xe2, 0x07),
	UCP10_CMD(0, 0xb0, 0x05),
	UCP10_CMD(0, 0xc0, 0x00),
	UCP10_CMD(0, 0xc2, 0x00),
	UCP10_CMD(0, 0xd9, 0x00),
	UCP10_CMD(0, 0xdb, 0x00),
	UCP10_CMD(255, 0x11),	/* exit sleep mode */
	UCP10_CMD(20, 0x29),	/* set display on */
};

static inline struct ucp10_panel *to_ucp10_panel(struct drm_panel *panel)
{
	return container_of(panel, struct ucp10_panel, panel);
}

static void ucp10_reset(struct ucp10_panel *ctx)
{
	/* reset-gpios is active low: the value 1 asserts the reset. */
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(20);				/* reset-delay-ms */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);				/* init-delay-ms */
}

static int ucp10_prepare(struct drm_panel *panel)
{
	struct ucp10_panel *ctx = to_ucp10_panel(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	unsigned int i;
	int ret;

	ret = regulator_enable(ctx->supply);
	if (ret < 0)
		return ret;

	msleep(20);				/* prepare-delay-ms */

	ucp10_reset(ctx);

	for (i = 0; i < ARRAY_SIZE(ucp10_init_cmds); i++) {
		const struct ucp10_init_cmd *cmd = &ucp10_init_cmds[i];

		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, cmd->data, cmd->len);
		if (cmd->delay)
			mipi_dsi_msleep(&dsi_ctx, cmd->delay);
	}

	if (dsi_ctx.accum_err) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_disable(ctx->supply);
		return dsi_ctx.accum_err;
	}

	msleep(35);				/* enable-delay-ms */

	return 0;
}

static int ucp10_unprepare(struct drm_panel *panel)
{
	struct ucp10_panel *ctx = to_ucp10_panel(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->supply);

	return 0;
}

static int ucp10_disable(struct drm_panel *panel)
{
	struct ucp10_panel *ctx = to_ucp10_panel(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	static const u8 cmd_display_off[] = { 0x28 };
	static const u8 cmd_enter_sleep[] = { 0x10 };

	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, cmd_display_off,
					sizeof(cmd_display_off));
	mipi_dsi_msleep(&dsi_ctx, 20);		/* disable-delay-ms */
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, cmd_enter_sleep,
					sizeof(cmd_enter_sleep));
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

/*
 * 1200x1920 with a 120 MHz pixel clock (about 47 Hz at this blanking).
 * The horizontal timing is the stock display-timings-himax value: hsync 12,
 * hbp 32, hfp 40. The vertical timing keeps the stock vsync 4 and vtotal 1984,
 * but uses vbp 22 and vfp 38 instead of the stock 25 and 35. With vbp 25 the
 * panel starts its frame 3 lines early and the last 3 lines wrap to the top.
 * A test on the hardware with DSI VID_VBP_LINES 22, VID_VFP_LINES 38 and VOP
 * VACT_ST 26 removed the wrap. HSYNC and VSYNC are active low.
 */
static const struct drm_display_mode ucp10_mode = {
	.clock = 120000,		/* VPLL 1200 MHz / 10, exact integer divide */
	.hdisplay = 1200,
	.hsync_start = 1200 + 40,
	.hsync_end = 1200 + 40 + 12,
	.htotal = 1200 + 40 + 12 + 32,
	.vdisplay = 1920,
	.vsync_start = 1920 + 38,
	.vsync_end = 1920 + 38 + 4,
	.vtotal = 1920 + 38 + 4 + 22,
	.width_mm = 120,
	.height_mm = 170,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int ucp10_get_modes(struct drm_panel *panel,
			   struct drm_connector *connector)
{
	struct ucp10_panel *ctx = to_ucp10_panel(panel);
	int ret;

	ret = drm_connector_helper_get_modes_fixed(connector, &ucp10_mode);

	/*
	 * TODO: Remove once all drm drivers call
	 * drm_connector_set_orientation_from_panel()
	 */
	drm_connector_set_panel_orientation(connector, ctx->orientation);

	return ret;
}

static enum drm_panel_orientation ucp10_get_orientation(struct drm_panel *panel)
{
	struct ucp10_panel *ctx = to_ucp10_panel(panel);

	return ctx->orientation;
}

static const struct drm_panel_funcs ucp10_panel_funcs = {
	.prepare = ucp10_prepare,
	.unprepare = ucp10_unprepare,
	.disable = ucp10_disable,
	.get_modes = ucp10_get_modes,
	.get_orientation = ucp10_get_orientation,
};

static int ucp10_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct ucp10_panel *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct ucp10_panel, panel,
				   &ucp10_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(ctx->supply))
		return dev_err_probe(dev, PTR_ERR(ctx->supply),
				     "Failed to get power regulator\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ret = of_drm_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get orientation\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET;

	/* The DSI host must power the PHY before the init commands. */
	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void ucp10_remove(struct mipi_dsi_device *dsi)
{
	struct ucp10_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ucp10_of_match[] = {
	{ .compatible = "crestron,uc-p10-himax-panel" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ucp10_of_match);

static struct mipi_dsi_driver ucp10_driver = {
	.probe = ucp10_probe,
	.remove = ucp10_remove,
	.driver = {
		.name = "panel-crestron-uc-p10",
		.of_match_table = ucp10_of_match,
	},
};
module_mipi_dsi_driver(ucp10_driver);

MODULE_DESCRIPTION("Crestron UC-P10 MIPI-DSI panel driver");
MODULE_LICENSE("GPL");
