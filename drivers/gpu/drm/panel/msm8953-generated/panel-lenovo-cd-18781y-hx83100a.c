// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2024 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct hx83100a_800p {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpio;
};

static inline struct hx83100a_800p *to_hx83100a_800p(struct drm_panel *panel)
{
	return container_of(panel, struct hx83100a_800p, panel);
}

static void hx83100a_800p_reset(struct hx83100a_800p *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(50);
}

static int hx83100a_800p_on(struct hx83100a_800p *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;

	mipi_dsi_dcs_write_seq(dsi, 0x11, 0x00);
	msleep(120);
	mipi_dsi_dcs_write_seq(dsi, 0x29, 0x00);
	msleep(20);
	mipi_dsi_dcs_write_seq(dsi, 0xb9, 0x83, 0x10, 0x0a);
	usleep_range(5000, 6000);
	mipi_dsi_dcs_write_seq(dsi, 0xc9,
			       0x1f, 0x00, 0x08, 0x1e, 0x81, 0x1e, 0x00);
	usleep_range(5000, 6000);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x24);
	usleep_range(5000, 6000);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_POWER_SAVE, 0x02);
	usleep_range(5000, 6000);
	mipi_dsi_dcs_write_seq(dsi, 0xca,
			       0x40, 0x3c, 0x38, 0x34, 0x33, 0x32, 0x30, 0x2c,
			       0x28);
	usleep_range(5000, 6000);

	return 0;
}

static int hx83100a_800p_off(struct hx83100a_800p *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	msleep(150);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}

	return 0;
}

static int hx83100a_800p_prepare(struct drm_panel *panel)
{
	struct hx83100a_800p *ctx = to_hx83100a_800p(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	hx83100a_800p_reset(ctx);

	ret = hx83100a_800p_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
		return ret;
	}

	return 0;
}

static int hx83100a_800p_unprepare(struct drm_panel *panel)
{
	struct hx83100a_800p *ctx = to_hx83100a_800p(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = hx83100a_800p_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

	return 0;
}

static const struct drm_display_mode hx83100a_800p_mode = {
	.clock = (800 + 40 + 40 + 40) * (1280 + 412 + 4 + 8) * 60 / 1000,
	.hdisplay = 800,
	.hsync_start = 800 + 40,
	.hsync_end = 800 + 40 + 40,
	.htotal = 800 + 40 + 40 + 40,
	.vdisplay = 1280,
	.vsync_start = 1280 + 412,
	.vsync_end = 1280 + 412 + 4,
	.vtotal = 1280 + 412 + 4 + 8,
	.width_mm = 107,
	.height_mm = 172,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int hx83100a_800p_get_modes(struct drm_panel *panel,
				   struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &hx83100a_800p_mode);
}

static const struct drm_panel_funcs hx83100a_800p_panel_funcs = {
	.prepare = hx83100a_800p_prepare,
	.unprepare = hx83100a_800p_unprepare,
	.get_modes = hx83100a_800p_get_modes,
};

static int hx83100a_800p_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct hx83100a_800p *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->supplies[0].supply = "vsn";
	ctx->supplies[1].supply = "vsp";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	drm_panel_init(&ctx->panel, dev, &hx83100a_800p_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void hx83100a_800p_remove(struct mipi_dsi_device *dsi)
{
	struct hx83100a_800p *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id hx83100a_800p_of_match[] = {
	{ .compatible = "lenovo,cd-18781y-hx83100a" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hx83100a_800p_of_match);

static struct mipi_dsi_driver hx83100a_800p_driver = {
	.probe = hx83100a_800p_probe,
	.remove = hx83100a_800p_remove,
	.driver = {
		.name = "panel-hx83100a-800p",
		.of_match_table = hx83100a_800p_of_match,
	},
};
module_mipi_dsi_driver(hx83100a_800p_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for hx83100a 800p video mode dsi panel");
MODULE_LICENSE("GPL");
