/*
 * Copyright (C) 2014 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/gpio/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <drm/panel/panel-jdi-lpm102a188a.h>

#include <video/mipi_display.h>

static struct backlight_jdi_init {
	u8 reg;
	u8 value;
} backlight_init_table[] = {
	{
		.reg = I2C_LP8557_CONFIG,
		.value = LP8557_CONFIG_BRTMODE_PWM | LP8557_CONFIG_AUTO_LEDEN,
	},
	{
		.reg = I2C_LP8557_CURRENT,
		.value = LP8557_CURRENT_MAXCUR_18MA,
	},
	{
		.reg = I2C_LP8557_BOOST,
		.value = LP8557_BOOST_FREQ_1MHZ | LP8557_BOOST_BCOMP_OPTION_1,
	},
	{
		.reg = I2C_LP8557_LEDEN,
		.value = LP8557_LEDEN_4V_0V | LP8557_LEDEN_6_CURRENT_SINKS,
	},
	{
		.reg = I2C_LP8557_SMOOTHING_STEP,
		.value = LP8557_SMOOTHING_HEAVY | LP8557_SMOOTHING_STEP_200MS,
	},
	{
		.reg = I2C_LP8557_PGEN,
		.value = LP8557_PGEN_PFREQ_9_8KHZ | LP8557_PGEN_THRESHOLD_FULL,
	},
};

struct panel_jdi {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;
	struct mipi_dsi_device *slave;
	struct backlight_device *bl;
	struct i2c_client *client;
	struct i2c_client *touch;

	struct regulator *supply;
	struct regulator *ddi_supply;
	int enable_gpio;
	unsigned long enable_gpio_flags;
	int reset_gpio;
	unsigned long reset_gpio_flags;
	int ts_reset_gpio;
	unsigned long ts_reset_gpio_flags;

	const struct drm_display_mode *mode;

	struct mutex lock;
	bool enabled;
	int brightness;

	struct dentry *debugfs_entry;
	u8 current_register;
};

static inline struct panel_jdi *to_panel_jdi(struct drm_panel *panel)
{
	return container_of(panel, struct panel_jdi, base);
}

static int backlight_jdi_set_power(struct panel_jdi *jdi, bool power)
{
	int ret;
	u32 value = 0;

	if (power)
		value = 1;

	ret = i2c_smbus_write_byte_data(jdi->client, I2C_LP8557_COMMAND, value);
	if (ret) {
		DRM_ERROR("Failed to write backlight command reg %d", ret);
		return ret;
	}
	return 0;
}

static int backlight_jdi_write_display_brightness(struct panel_jdi *jdi,
						  u32 brightness)
{
	int ret;
	u8 data;

	if (jdi->brightness == brightness)
		return 0;

	if (brightness == 0) {
		ret = backlight_jdi_set_power(jdi, false);
		if (ret) {
			DRM_ERROR("Failed to disable backlight ret=%d\n", ret);
			return ret;
		}
	} else if (jdi->brightness == 0) {
		ret = backlight_jdi_set_power(jdi, true);
		if (ret) {
			DRM_ERROR("Failed to enable backlight ret=%d\n", ret);
			return ret;
		}
	}
	data = RSP_WRITE_DISPLAY_BRIGHTNESS(brightness);

	ret = mipi_dsi_dcs_write(jdi->slave,
			MIPI_DCS_RSP_WRITE_DISPLAY_BRIGHTNESS, &data, 1);
	if (ret < 1) {
		DRM_ERROR("failed to write display brightness: %d\n", ret);
		return ret;
	}

	return 0;
}

static int backlight_jdi_update_status(struct backlight_device *bl)
{
	struct panel_jdi *jdi = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int ret;

	if (bl->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
		brightness = 0;

	mutex_lock(&jdi->lock);

	if (!jdi->enabled) {
		ret = 0;
		goto out;
	}

	ret = backlight_jdi_write_display_brightness(jdi, brightness);
	if (ret) {
		DRM_ERROR("write_display_brightness failed with %d\n", ret);
		goto out;
	}

	jdi->brightness = brightness;
out:
	mutex_unlock(&jdi->lock);
	return ret;
}

static int panel_jdi_write_control_display(struct panel_jdi *jdi)
{
	int ret;
	u8 data;

	data = RSP_WRITE_CONTROL_DISPLAY_BL_ON |
			RSP_WRITE_CONTROL_DISPLAY_BCTRL_LEDPWM |
			RSP_WRITE_CONTROL_DISPLAY_DD_ON;

	ret = mipi_dsi_dcs_write(jdi->dsi, MIPI_DCS_RSP_WRITE_CONTROL_DISPLAY,
			&data, 1);
	if (ret < 1) {
		DRM_ERROR("failed to write control display: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_write(jdi->slave, MIPI_DCS_RSP_WRITE_CONTROL_DISPLAY,
			&data, 1);
	if (ret < 1) {
		DRM_ERROR("failed to write control display: %d\n", ret);
		return ret;
	}

	return 0;
}

static int panel_jdi_write_adaptive_brightness_control(struct panel_jdi *jdi)
{
	int ret;
	u8 data;

	data = RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL_C_UI;

	ret = mipi_dsi_dcs_write(jdi->dsi,
			MIPI_DCS_RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL, &data,
			1);
	if (ret < 1) {
		DRM_ERROR("failed to set adaptive brightness ctrl: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_write(jdi->slave,
			MIPI_DCS_RSP_WRITE_ADAPTIVE_BRIGHTNESS_CONTROL, &data,
			1);
	if (ret < 1) {
		DRM_ERROR("failed to set adaptive brightness ctrl: %d\n", ret);
		return ret;
	}

	return 0;
}

static int panel_jdi_write_dcdc_registers(struct panel_jdi *jdi)
{
	u8 payload[3];
	int write_len;
	int ret;

	/* Clear the manufacturer command access protection */
	write_len = 0;
	payload[write_len++] = MIPI_DCS_RSP_MFG_COMMAND_ACCESS_PROTECT;
	payload[write_len++] = RSP_MFG_COMMAND_ACCESS_PROTECT_OFF;
	ret = mipi_dsi_generic_write(jdi->dsi, payload, write_len);
	if (ret < 0) {
		DRM_ERROR("failed to write generic packet for 0xB0: %d\n", ret);
		return ret;
	}
	ret = mipi_dsi_generic_write(jdi->slave, payload, 2);
	if (ret < 0) {
		DRM_ERROR("failed to write generic packet for 0xB0: %d\n", ret);
		return ret;
	}

	/*
	 * Change the VGH/VGL divide rations to move the noise generated by the
	 * TCONN. This should hopefully avoid interaction with the backlight
	 * controller.
	 */
	write_len = 0;
	payload[write_len++] = MIPI_DSI_RSP_MFG_POWER_CONTROL_FUNCTION;
	payload[write_len++] = RSP_MFG_POWER_CONTROL_PARAM1_VGH_330_DIV |
			       RSP_MFG_POWER_CONTROL_PARAM1_DEFAULT;
	payload[write_len++] = RSP_MFG_POWER_CONTROL_PARAM2_VGL_410_DIV |
			       RSP_MFG_POWER_CONTROL_PARAM2_DEFAULT;
	ret = mipi_dsi_generic_write(jdi->dsi, payload, write_len);
	if (ret < 0) {
		DRM_ERROR("failed to write generic packet for 0xD0: %d\n", ret);
		return ret;
	}
	ret = mipi_dsi_generic_write(jdi->slave, payload, write_len);
	if (ret < 0) {
		DRM_ERROR("failed to write generic packet for 0xD0: %d\n", ret);
		return ret;
	}

	if (ret < 0)
		return ret;
	else
		return 0;
}

static int panel_jdi_disable(struct drm_panel *panel)
{
	struct panel_jdi *jdi = to_panel_jdi(panel);
	int ret = 0;

	mutex_lock(&jdi->lock);
	if (!jdi->enabled)
		goto out;

	if (!panel->connector || panel->connector->dpms == DRM_MODE_DPMS_ON)
		goto out;

	if (jdi->touch)
		pm_runtime_force_suspend(&jdi->touch->dev);

	ret = backlight_jdi_write_display_brightness(jdi, 0);
	if (ret < 0)
		DRM_ERROR("failed to set backlight on: %d\n", ret);

	ret = mipi_dsi_dcs_set_display_off(jdi->dsi);
	if (ret < 0)
		DRM_ERROR("failed to set display off: %d\n", ret);
	ret = mipi_dsi_dcs_set_display_off(jdi->slave);
	if (ret < 0)
		DRM_ERROR("failed to set display off: %d\n", ret);

	/* Specified by JDI @ 50ms, subject to change */
	msleep(50);

	ret = mipi_dsi_dcs_enter_sleep_mode(jdi->dsi);
	if (ret < 0)
		DRM_ERROR("failed to enter sleep mode: %d\n", ret);
	ret = mipi_dsi_dcs_enter_sleep_mode(jdi->slave);
	if (ret < 0)
		DRM_ERROR("failed to enter sleep mode: %d\n", ret);

	/* Specified by JDI @ 150ms, subject to change */
	msleep(150);

	jdi->brightness = 0;
out:
	mutex_unlock(&jdi->lock);
	return ret;
}

static int panel_jdi_unprepare(struct drm_panel *panel)
{
	struct panel_jdi *jdi = to_panel_jdi(panel);

	mutex_lock(&jdi->lock);
	if (!jdi->enabled)
		goto out;

	if (!panel->connector || panel->connector->dpms == DRM_MODE_DPMS_ON)
		goto out;

	gpio_set_value(jdi->reset_gpio,
		(jdi->reset_gpio_flags & GPIO_ACTIVE_LOW) ? 0 : 1);

	/* T4 = 1ms */
	usleep_range(1000, 3000);

	gpio_set_value(jdi->enable_gpio,
		(jdi->enable_gpio_flags & GPIO_ACTIVE_LOW) ? 1 : 0);

	/* T5 = 2ms */
	usleep_range(2000, 4000);

	jdi->enabled = false;

out:
	mutex_unlock(&jdi->lock);
	return 0;
}

static int backlight_jdi_prepare(struct panel_jdi *jdi)
{
	struct i2c_client *cl = jdi->client;
	int ret, i;
	bool reinitialize = false;

	/*
	 * First read the values from the controller. If they're already
	 * initialized, exit. This avoids having to turn off the controller,
	 * which causes a brief blink.
	 */
	for (i = 0; i < ARRAY_SIZE(backlight_init_table); i++) {
		struct backlight_jdi_init *init = &backlight_init_table[i];
		ret = i2c_smbus_read_byte_data(cl, init->reg);
		if (ret < 0) {
			DRM_ERROR("Failed to read bl reg %x %d\n", init->reg,
				ret);
			reinitialize = true;
			break;
		} else if (ret != init->value) {
			reinitialize = true;
			break;
		}
	}
	if (!reinitialize)
		return 0;

	ret = backlight_jdi_set_power(jdi, false);
	if (ret) {
		DRM_ERROR("Failed to disable backlight ret=%d", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(backlight_init_table); i++) {
		struct backlight_jdi_init *init = &backlight_init_table[i];
		ret = i2c_smbus_write_byte_data(cl, init->reg, init->value);
		if (ret < 0) {
			DRM_ERROR("Failed to init bl reg %x %d\n", init->reg,
				ret);
			return ret;
		}
	}

	ret = backlight_jdi_set_power(jdi, true);
	if (ret) {
		DRM_ERROR("Failed to enable backlight ret=%d", ret);
		return ret;
	}

	return ret;
}

static int panel_jdi_prepare(struct drm_panel *panel)
{
	struct panel_jdi *jdi = to_panel_jdi(panel);
	int ret = 0;

	mutex_lock(&jdi->lock);

	if (!jdi->enabled) {
		gpio_set_value(jdi->enable_gpio,
			(jdi->enable_gpio_flags & GPIO_ACTIVE_LOW) ? 0 : 1);

		/* T3 = 10ms */
		usleep_range(10000, 15000);

		gpio_set_value(jdi->reset_gpio,
			(jdi->reset_gpio_flags & GPIO_ACTIVE_LOW) ? 1 : 0);

		/* Specified by JDI @ 3ms, subject to change */
		usleep_range(3000, 5000);
	}

	if (jdi->bl) {
		ret = backlight_jdi_prepare(jdi);
		if (ret)
			DRM_ERROR("failed to prepare backlight: %d\n", ret);
	}

	ret = mipi_dsi_dcs_set_column_address(jdi->dsi, 0,
				jdi->mode->hdisplay / 2 - 1);
	if (ret < 0)
		DRM_ERROR("failed to set column address: %d\n", ret);

	ret = mipi_dsi_dcs_set_column_address(jdi->slave, 0,
				jdi->mode->hdisplay / 2 - 1);
	if (ret < 0)
		DRM_ERROR("failed to set column address: %d\n", ret);

	ret = mipi_dsi_dcs_set_page_address(jdi->dsi, 0,
				jdi->mode->vdisplay - 1);
	if (ret < 0)
		DRM_ERROR("failed to set page address: %d\n", ret);

	ret = mipi_dsi_dcs_set_page_address(jdi->slave, 0,
				jdi->mode->vdisplay - 1);
	if (ret < 0)
		DRM_ERROR("failed to set page address: %d\n", ret);

	ret = mipi_dsi_dcs_set_tear_scanline(jdi->dsi,
					     jdi->mode->vdisplay - 16);
	if (ret < 0)
		DRM_ERROR("failed to set tear scanline: %d\n", ret);

	ret = mipi_dsi_dcs_set_tear_scanline(jdi->slave,
					     jdi->mode->vdisplay - 16);
	if (ret < 0)
		DRM_ERROR("failed to set tear scanline: %d\n", ret);

	ret = mipi_dsi_dcs_set_tear_on(jdi->dsi,
					MIPI_DSI_DCS_TEAR_MODE_VBLANK);
        if (ret < 0)
		DRM_ERROR("failed to set tear on: %d\n", ret);

	ret = mipi_dsi_dcs_set_tear_on(jdi->slave,
					MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	 if (ret < 0)
		 DRM_ERROR("failed to set tear on: %d\n", ret);


	ret = mipi_dsi_dcs_set_address_mode(jdi->dsi, false, false, false,
			false, false, false, false, false);
	if (ret < 0)
		DRM_ERROR("failed to set address mode: %d\n", ret);

	ret = mipi_dsi_dcs_set_address_mode(jdi->slave, false, false,
			false, false, false, false, false, false);
	if (ret < 0)
		DRM_ERROR("failed to set address mode: %d\n", ret);

	ret = mipi_dsi_dcs_set_pixel_format(jdi->dsi, MIPI_DCS_PIXEL_FMT_24BIT);
	if (ret < 0)
		DRM_ERROR("failed to set pixel format: %d\n", ret);

	ret = mipi_dsi_dcs_set_pixel_format(jdi->slave,
				MIPI_DCS_PIXEL_FMT_24BIT);
	if (ret < 0)
		DRM_ERROR("failed to set pixel format: %d\n", ret);

	ret = panel_jdi_write_control_display(jdi);
	if (ret < 0)
		DRM_ERROR("failed to write control display: %d\n", ret);

	ret = panel_jdi_write_adaptive_brightness_control(jdi);
	if (ret < 0)
		DRM_ERROR("failed to set adaptive brightness ctrl: %d\n", ret);

	if (!jdi->enabled) {
		ret = mipi_dsi_dcs_exit_sleep_mode(jdi->dsi);
		if (ret < 0)
			DRM_ERROR("failed to exit sleep mode: %d\n", ret);

		ret = mipi_dsi_dcs_exit_sleep_mode(jdi->slave);
		if (ret < 0)
			DRM_ERROR("failed to exit sleep mode: %d\n", ret);

		/*
		 * Spec'd by JDI @>67ms, between SleepOUT and deasserting touch
		 * reset
		 */
		msleep(70);

		/*
		 * We need to wait 150ms between mipi_dsi_dcs_exit_sleep_mode()
		 * and mipi_dsi_dcs_set_display_on().
		 */
		msleep(80);
	}

	ret = panel_jdi_write_dcdc_registers(jdi);
	if (ret)
		DRM_ERROR("failed to write dc registers %d\n", ret);

	mutex_unlock(&jdi->lock);
	return ret;
}

static int panel_jdi_enable(struct drm_panel *panel)
{
	struct panel_jdi *jdi = to_panel_jdi(panel);
	int ret;

	mutex_lock(&jdi->lock);

	ret = mipi_dsi_dcs_set_display_on(jdi->dsi);
	if (ret < 0)
		DRM_ERROR("failed to set display on: %d\n", ret);

	ret = mipi_dsi_dcs_set_display_on(jdi->slave);
	if (ret < 0)
		DRM_ERROR("failed to set display on: %d\n", ret);

	if (!jdi->enabled && jdi->touch)
		pm_runtime_force_resume(&jdi->touch->dev);

	if (gpio_is_valid(jdi->ts_reset_gpio)) {
		gpio_set_value(jdi->ts_reset_gpio,
			(jdi->ts_reset_gpio_flags & GPIO_ACTIVE_LOW) ?  0 : 1);
		usleep_range(9000, 10000);
		gpio_set_value(jdi->ts_reset_gpio,
			(jdi->ts_reset_gpio_flags & GPIO_ACTIVE_LOW) ?  1 : 0);
	}

	jdi->enabled = true;

	mutex_unlock(&jdi->lock);

	ret = backlight_jdi_update_status(jdi->bl);
	if (ret)
		DRM_ERROR("failed to update backlight: %d\n", ret);

	return ret;
}

static const struct drm_display_mode default_mode = {
	.clock = 321384,
	.hdisplay = 2560,
	.hsync_start = 2560 + 80,
	.hsync_end = 2560 + 80 + 80,
	.htotal = 2560 + 80 + 80 + 80,
	.vdisplay = 1800,
	.vsync_start = 1800 + 4,
	.vsync_end = 1800 + 4 + 4,
	.vtotal = 1800 + 4 + 4 + 4,

	.vrefresh = 60,
};

static int panel_jdi_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
	struct drm_display_info *info = &panel->connector->display_info;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		DRM_ERROR("failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(panel->connector, mode);

	info->width_mm = 211;
	info->height_mm = 148;
	info->bpc = 8;
	info->tearing_effect = 0;
	info->supports_psr = true;

	return 1;
}

static int jdi_register_get(void *data, u64 *val)
{
	struct panel_jdi *jdi = (struct panel_jdi*) data;

	*val = jdi->current_register;

	return 0;
}

static int jdi_register_set(void *data, u64 val)
{
	struct panel_jdi *jdi = (struct panel_jdi*) data;

	jdi->current_register = val;

	return 0;
}

static int jdi_value_get(void *data, u64 *val)
{
	struct panel_jdi *jdi = (struct panel_jdi*) data;
	struct mipi_dsi_device *dsi = jdi->dsi;
	int ret;
	u8 value = 0;

	ret = mipi_dsi_dcs_read(dsi, jdi->current_register, &value, 1);
	if (ret < 1) {
		DRM_ERROR("failed to write control display: %d\n", ret);
		return ret;
	}

	*val = value;

	return 0;
}

static int jdi_value_set(void *data, u64 val)
{
	struct panel_jdi *jdi = (struct panel_jdi*) data;
	struct mipi_dsi_device *dsi = jdi->dsi;
	int ret;
	u8 value = val;

	ret = mipi_dsi_dcs_write(dsi, jdi->current_register, &value, 1);
	if (ret < 1) {
		DRM_ERROR("failed to write control display: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_write(jdi->slave, jdi->current_register, &value, 1);
	if (ret < 1) {
		DRM_ERROR("failed to write control display: %d\n", ret);
		return ret;
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(jdi_register_fops,
			jdi_register_get, jdi_register_set,
			"%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(jdi_value_fops,
			jdi_value_get, jdi_value_set,
			"%llu\n");

static int panel_jdi_debugfs_init(struct panel_jdi *jdi)
{
	jdi->debugfs_entry = debugfs_create_dir("jdi-lpm102a188a", NULL);

	debugfs_create_file("register", S_IWUSR | S_IRUGO, jdi->debugfs_entry,
				jdi, &jdi_register_fops);

	debugfs_create_file("value", S_IWUSR | S_IRUGO, jdi->debugfs_entry,
				jdi, &jdi_value_fops);

	return 0;
}

static void panel_jdi_debugfs_cleanup(struct panel_jdi *jdi)
{
	debugfs_remove_recursive(jdi->debugfs_entry);
}

static const struct drm_panel_funcs panel_jdi_funcs = {
	.prepare = panel_jdi_prepare,
	.enable = panel_jdi_enable,
	.disable = panel_jdi_disable,
	.unprepare = panel_jdi_unprepare,
	.get_modes = panel_jdi_get_modes,
};

static const struct backlight_ops backlight_jdi_funcs = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = backlight_jdi_update_status,
};

static const struct backlight_properties backlight_jdi_props = {
	.type = BACKLIGHT_RAW,
	.brightness = 0,
	.resume_brightness = -1,
	.max_brightness = 0xFF,
};

static int panic_reset_gpio;
static int panic_reset_level;
static int panel_jdi_panic(struct notifier_block *n, unsigned long ununsed,
			   void *panic_str)
{
	/* assume the GPIO control is not sleeping (e.g. no I2C) */
	gpio_set_value(panic_reset_gpio, panic_reset_level);
	mdelay(16);

	return 0;
}

static struct notifier_block paniced = {
	.notifier_call = panel_jdi_panic,
};

static const struct of_device_id jdi_of_match[] = {
	{ .compatible = "jdi,lpm102a188a", },
	{ }
};
MODULE_DEVICE_TABLE(of, jdi_of_match);

static int panel_jdi_setup_primary(struct mipi_dsi_device *dsi,
				   struct device_node *np)
{
	struct panel_jdi *jdi;
	struct mipi_dsi_device *slave;
	enum of_gpio_flags gpio_flags;
	struct device_node *i2c_np, *touch_np;
	int ret;

	slave = of_find_mipi_dsi_device_by_node(np);
	of_node_put(np);

	if (!slave)
		return -EPROBE_DEFER;

	jdi = devm_kzalloc(&dsi->dev, sizeof(*jdi), GFP_KERNEL);
	if (!jdi) {
		ret = -ENOMEM;
		goto out_slave;
	}

	mipi_dsi_set_drvdata(dsi, jdi);
	jdi->mode = &default_mode;
	jdi->slave = slave;
	jdi->dsi = dsi;
	mutex_init(&jdi->lock);

	jdi->enabled = of_property_read_bool(dsi->dev.of_node, "panel-boot-on");

	jdi->supply = devm_regulator_get(&dsi->dev, "power");
	if (IS_ERR(jdi->supply)) {
		ret = PTR_ERR(jdi->supply);
		goto out_slave;
	}

	jdi->ddi_supply = devm_regulator_get(&dsi->dev, "ddi");
	if (IS_ERR(jdi->ddi_supply)) {
		ret = PTR_ERR(jdi->ddi_supply);
		goto out_slave;
	}

	jdi->enable_gpio = of_get_named_gpio_flags(dsi->dev.of_node,
				"enable-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(jdi->enable_gpio)) {
		DRM_ERROR("enable gpio not found\n");
		ret = -ENODEV;
		goto out_slave;
	}

	if (gpio_flags & OF_GPIO_ACTIVE_LOW)
		jdi->enable_gpio_flags |= GPIO_ACTIVE_LOW;

	ret = devm_gpio_request(&dsi->dev, jdi->enable_gpio, "jdi-enable");
	if (ret < 0) {
		DRM_ERROR("Request enable gpio failed: %d\n", ret);
		goto out_slave;
	}

	ret = gpio_direction_output(jdi->enable_gpio,
		(jdi->enable_gpio_flags & GPIO_ACTIVE_LOW) ? 0 : 1);
	if (ret) {
		DRM_ERROR("Request enable gpio output failed: %d\n",
			  ret);
		goto out_slave;
	}

	jdi->reset_gpio = of_get_named_gpio_flags(dsi->dev.of_node,
				"reset-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(jdi->reset_gpio)) {
		DRM_ERROR("reset gpio not found: %d\n", ret);
		ret = -ENODEV;
		goto out_slave;
	}

	if (gpio_flags & OF_GPIO_ACTIVE_LOW)
		jdi->reset_gpio_flags |= GPIO_ACTIVE_LOW;

	ret = devm_gpio_request(&dsi->dev, jdi->reset_gpio, "jdi-reset");
	if (ret < 0) {
		DRM_ERROR("Request reset gpio failed: %d\n", ret);
		goto out_slave;
	}

	ret = gpio_direction_output(jdi->reset_gpio,
		(jdi->reset_gpio_flags & GPIO_ACTIVE_LOW) ? 1 : 0);
	if (ret) {
		DRM_ERROR("Request reset gpio output failed: %d\n",
			  ret);
		goto out_slave;
	}

	jdi->ts_reset_gpio = of_get_named_gpio_flags(dsi->dev.of_node,
				"ts-reset-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(jdi->ts_reset_gpio)) {
		DRM_INFO("ts reset gpio not found: %d\n", ret);
	} else {
		if (gpio_flags & OF_GPIO_ACTIVE_LOW)
			jdi->ts_reset_gpio_flags |= GPIO_ACTIVE_LOW;

		ret = devm_gpio_request(&dsi->dev, jdi->ts_reset_gpio,
			"jdi-ts-reset");
		if (ret < 0) {
			DRM_ERROR("Request ts reset gpio failed: %d\n", ret);
			goto out_slave;
		}

		ret = gpio_direction_output(jdi->ts_reset_gpio,
			(jdi->ts_reset_gpio_flags & GPIO_ACTIVE_LOW) ? 1 : 0);
		if (ret) {
			DRM_ERROR("Request ts reset gpio output failed: %d\n",
				  ret);
			goto out_slave;
		}
	}

	i2c_np = of_parse_phandle(dsi->dev.of_node, "backlight", 0);
	if (i2c_np) {
		jdi->client = of_find_i2c_device_by_node(i2c_np);
		of_node_put(i2c_np);
		if (!jdi->client) {
			DRM_ERROR("Could not find backlight i2c client\n");
			ret = -ENODEV;
			goto out_slave;
		}
	}

	touch_np = of_parse_phandle(dsi->dev.of_node, "touch", 0);
	if (touch_np) {
		jdi->touch = of_find_i2c_device_by_node(touch_np);
		of_node_put(touch_np);
		if (!jdi->touch) {
			DRM_ERROR("Could not find touch i2c client\n");
			ret = -ENODEV;
			goto out_slave;
		}
	}

	ret = regulator_enable(jdi->supply);
	if (ret < 0) {
		DRM_ERROR("failed to enable supply: %d\n", ret);
		goto out_client;
	}

	/* T1 = 2ms */
	usleep_range(2000, 4000);

	ret = regulator_enable(jdi->ddi_supply);
	if (ret < 0) {
		DRM_ERROR("failed to enable ddi_supply: %d\n", ret);
		goto out_supply;
	}

	/* T2 = 1ms */
	usleep_range(1000, 3000);

	drm_panel_init(&jdi->base);
	jdi->base.dev = &dsi->dev;
	jdi->base.funcs = &panel_jdi_funcs;

	ret = drm_panel_add(&jdi->base);
	if (ret < 0) {
		DRM_ERROR("drm_panel_add failed: %d\n", ret);
		goto out_ddi_supply;
	}

	if (jdi->client) {
		jdi->bl = devm_backlight_device_register(&dsi->dev,
					"lpm102a188a-backlight", &dsi->dev,
					jdi, &backlight_jdi_funcs,
					&backlight_jdi_props);
		if (IS_ERR(jdi->bl))
			goto out_panel;
	}

	/* ensure we put the panel under reset on panic */
	panic_reset_gpio = jdi->reset_gpio;
	panic_reset_level = (jdi->reset_gpio_flags & GPIO_ACTIVE_LOW) ? 0 : 1;
	atomic_notifier_chain_register(&panic_notifier_list, &paniced);

	panel_jdi_debugfs_init(jdi);

	return 0;

out_panel:
	drm_panel_detach(&jdi->base);
out_ddi_supply:
	regulator_disable(jdi->ddi_supply);
out_supply:
	regulator_disable(jdi->supply);
out_client:
	if (jdi->client)
		put_device(&jdi->client->dev);
out_slave:
	put_device(&slave->dev);

	return ret;
}

static int panel_jdi_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct device_node *np;
	int ret;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = 0;

	/* if this device is the primary link, initialize the panel */
	np = of_parse_phandle(dsi->dev.of_node, "slave", 0);
	if (np) {
		ret = panel_jdi_setup_primary(dsi, np);
		if (ret)
			return ret;
	}

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		return ret;

	return 0;
}

static int panel_jdi_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct panel_jdi *jdi = mipi_dsi_get_drvdata(dsi);
	int ret;

	/* nothing to do for the secondary interface */
	if (!jdi)
		return 0;

	atomic_notifier_chain_unregister(&panic_notifier_list, &paniced);

	if (jdi->bl)
		devm_backlight_device_unregister(&dsi->dev, jdi->bl);

	panel_jdi_disable(&jdi->base);

	regulator_disable(jdi->ddi_supply);

	/* T6 = 2ms plus some time to discharge capacitors */
	usleep_range(7000, 9000);

	regulator_disable(jdi->supply);

	/* Specified by JDI @ 20ms, subject to change */
	msleep(20);

	drm_panel_detach(&jdi->base);
	drm_panel_remove(&jdi->base);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		DRM_ERROR("failed to detach from DSI host: %d\n", ret);

	if (jdi->client)
		put_device(&jdi->client->dev);
	put_device(&jdi->slave->dev);

	return 0;
}

static void panel_jdi_dsi_shutdown(struct mipi_dsi_device *dsi)
{
	struct panel_jdi *jdi = mipi_dsi_get_drvdata(dsi);
	struct device *parent;

	if (!jdi)
		return;

	/* ensure we shutdown the MIPI link first */
	parent = get_device(dsi->dev.parent);
	if (parent) {
		if (parent->driver->shutdown)
			parent->driver->shutdown(parent);
		put_device(parent);
	}

	panel_jdi_debugfs_cleanup(jdi);
	/* Turn off panel power rails */
	panel_jdi_dsi_remove(dsi);
}

static struct mipi_dsi_driver panel_jdi_dsi_driver = {
	.driver = {
		.name = "panel-jdi-lpm102a188a-dsi",
		.of_match_table = jdi_of_match,
	},
	.probe = panel_jdi_dsi_probe,
	.remove = panel_jdi_dsi_remove,
	.shutdown = panel_jdi_dsi_shutdown,
};
module_mipi_dsi_driver(panel_jdi_dsi_driver);
MODULE_AUTHOR("Sean Paul <seanpaul@chromium.org>");
MODULE_DESCRIPTION("DRM Driver for JDI LPM102A188A");
MODULE_LICENSE("GPL and additional rights");
