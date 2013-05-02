/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2012, LGE Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/gpio_event.h>

#include <mach/vreg.h>
#include <mach/rpc_server_handset.h>
#include <mach/board.h>

/* keypad */
#include <linux/mfd/pm8xxx/pm8921.h>

/* i2c */
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>

#include <linux/earlysuspend.h>
#include <linux/input/lge_touch_core.h>
#include <mach/board_lge.h>
#include "board-mako.h"

/* TOUCH GPIOS */
#define SYNAPTICS_TS_I2C_SDA                 	8
#define SYNAPTICS_TS_I2C_SCL                 	9
#define SYNAPTICS_TS_I2C_INT_GPIO            	6
#define TOUCH_RESET                             33

#define TOUCH_FW_VERSION                        1

/* touch screen device */
#define APQ8064_GSBI3_QUP_I2C_BUS_ID            3

int rmi_1320_power_on(int on)
{
	int rc = -EINVAL;
	static struct regulator *vreg_l15 = NULL;
	static struct regulator *vreg_l22 = NULL;

	pr_debug("%s: called with on = %d.\n", __func__, on);

	/* 3.3V_TOUCH_VDD, VREG_L15: 2.75 ~ 3.3 */
	if (!vreg_l15) {
		vreg_l15 = regulator_get(NULL, "touch_vdd");
		if (IS_ERR(vreg_l15)) {
			pr_err("%s: regulator get of 8921_l15 failed (%ld)\n",
					__func__,
			       PTR_ERR(vreg_l15));
			rc = PTR_ERR(vreg_l15);
			vreg_l15 = NULL;
			return rc;
		}
	}
	/* 1.8V_TOUCH_IO, VREG_L22: 1.7 ~ 2.85 */
	if (!vreg_l22) {
		vreg_l22 = regulator_get(NULL, "touch_io");
		if (IS_ERR(vreg_l22)) {
			pr_err("%s: regulator get of 8921_l22 failed (%ld)\n",
					__func__,
			       PTR_ERR(vreg_l22));
			rc = PTR_ERR(vreg_l22);
			vreg_l22 = NULL;
			return rc;
		}
	}

	rc = regulator_set_voltage(vreg_l15, 3300000, 3300000);
	rc |= regulator_set_voltage(vreg_l22, 1800000, 1800000);
	if (rc < 0) {
		printk(KERN_INFO "%s: cannot control regulator\n",
		       __func__);
		return rc;
	}

	if (on) {
		pr_debug("%s: touch enable\n", __func__);
		regulator_enable(vreg_l15);
		regulator_enable(vreg_l22);
	} else {
		pr_debug("%s: touch disable\n", __func__);
		regulator_disable(vreg_l15);
		regulator_disable(vreg_l22);
	}

	return rc;
}

static struct touch_power_module touch_pwr = {
	.use_regulator = 0,
	.vdd = "8921_l15",
	.vdd_voltage = 3300000,
	.vio = "8921_l22",
	.vio_voltage = 1800000,
	.power = rmi_1320_power_on,
};

#ifdef CONFIG_RMI4_CORE

#include <linux/rmi.h>

#undef LGE_TOUCH_NAME
#define LGE_TOUCH_NAME "rmi_i2c"

struct rmi_pin_data {
	u32 reset_pin;
	struct touch_power_module *pwr;
};

struct rmi_pin_data gpio_data = {
	.reset_pin = TOUCH_RESET,
	.pwr = &touch_pwr,
};

int rmi_setup_power(struct touch_power_module* pwr)
{
	struct regulator *regulator_vdd;
	struct regulator *regulator_vio;
	int ret = 0;

	pr_debug("%s: called.\n", __func__);

	if (pwr->use_regulator) {
		pr_debug("%s: setting up regulators.\n", __func__);

		regulator_vdd =
			regulator_get_exclusive(NULL, pwr->vdd);
		if (IS_ERR(regulator_vdd)) {
			pr_err("%s FAIL: regulator_get_vdd - %s\n", __func__,
							pwr->vdd);
			ret = -EPERM;
			goto err_get_vdd_failed;
		}

		regulator_vio = regulator_get_exclusive(NULL,
							pwr->vio);
		if (IS_ERR(regulator_vio)) {
			pr_err("%s FAIL: regulator_get_vio - %s\n", __func__,
							pwr->vio);
			ret = -EPERM;
			goto err_get_vio_failed;
		}

		if (pwr->vdd_voltage > 0) {
			ret = regulator_set_voltage(regulator_vdd,
						pwr->vdd_voltage,
						pwr->vdd_voltage);
			if (ret < 0) {
				pr_err("%s FAIL: VDD voltage setting"
						" - (%duV)\n", __func__,
						pwr->vdd_voltage);
				ret = -EPERM;
				goto err_set_voltage;
			}
		}

		if (pwr->vio_voltage > 0) {
			ret = regulator_set_voltage(regulator_vio,
						pwr->vio_voltage,
						pwr->vio_voltage);
			if (ret < 0) {
				pr_err("%s FAIL: VIO voltage setting"
						" - (%duV)\n", __func__,
						pwr->vio_voltage);
				ret = -EPERM;
				goto err_set_voltage;
			}
		}
		pr_debug("%s: setup complete.\n", __func__);
	}
	else {
		pr_err("%s: Hey, nothing to set up here.\n", __func__);
	}

	return ret;

err_set_voltage:
	if (pwr->use_regulator) {
		regulator_put(regulator_vio);
	}
err_get_vio_failed:
	if (pwr->use_regulator) {
		regulator_put(regulator_vdd);
	}
err_get_vdd_failed:
	return ret;
}

static int rmi_gpio_setup(void *gpio_data, bool configure)
{
	struct rmi_pin_data *data = gpio_data;

	pr_info("%s: called with configure=%d.\n", __func__, configure);
	if (configure) {
 		int retval = 0; // rmi_setup_power(data->pwr);
// 		if (!retval)
// 			return retval;

		if (data->reset_pin > 0) {
			pr_debug("%s: Configuring reset_pin %d.\n", __func__, data->reset_pin);
			retval = gpio_request(data->reset_pin, "touch_reset");
			if (retval < 0) {
				pr_err("%s FAIL: touch_reset gpio_request\n", __func__);
				return retval;
			}
			pr_debug("%s: configured reset pin.\n", __func__);
			gpio_direction_output(data->reset_pin, 0);
			gpio_set_value(data->reset_pin, 1);
		}
	}
	return rmi_1320_power_on(configure);
}

static struct rmi_device_platform_data mako_ts_data = {
	.attn_gpio = SYNAPTICS_TS_I2C_INT_GPIO,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.gpio_data = &gpio_data,
	.gpio_config = rmi_gpio_setup,
};

#else

static struct touch_device_caps touch_caps = {
	.button_support = 0,
	.is_width_major_supported = 1,
	.is_width_minor_supported = 0,
	.is_pressure_supported = 1,
	.is_id_supported = 1,
	.max_width_major = 15,
	.max_width_minor = 15,
	.max_pressure = 0xFF,
	.max_id = 10,
	.lcd_x = 768,
	.lcd_y = 1280,
	.x_max = 1536-1,
	.y_max = 2560-1,
};

static struct touch_operation_role touch_role = {
	.operation_mode = INTERRUPT_MODE,
	.key_type = KEY_NONE,
	.report_mode = REDUCED_REPORT_MODE,
	.delta_pos_threshold = 1,
	.orientation = 0,
	.booting_delay = 400,
	.reset_delay = 20,
	.suspend_pwr = POWER_OFF,
	.resume_pwr = POWER_ON,
	.jitter_filter_enable = 0,
	.jitter_curr_ratio = 30,
	.accuracy_filter_enable = 0,
	.irqflags = IRQF_TRIGGER_FALLING,
	.show_touches = 0,
	.pointer_location = 0,
};

static struct touch_platform_data mako_ts_data = {
	.int_pin = SYNAPTICS_TS_I2C_INT_GPIO,
	.reset_pin = TOUCH_RESET,
	.maker = "Synaptics",
	.caps = &touch_caps,
	.role = &touch_role,
	.pwr = &touch_pwr,
};
#endif

static struct i2c_board_info synaptics_ts_info[] = {
	[0] = {
		I2C_BOARD_INFO(LGE_TOUCH_NAME, 0x20),
		.platform_data = &mako_ts_data,
		.irq = MSM_GPIO_TO_INT(SYNAPTICS_TS_I2C_INT_GPIO),
	},
};

void __init apq8064_init_input(void)
{
	printk(KERN_INFO "[Touch D] %s: NOT DCM KDDI, rmi driver %s\n",
	       __func__, LGE_TOUCH_NAME);
	i2c_register_board_info(APQ8064_GSBI3_QUP_I2C_BUS_ID,
				&synaptics_ts_info[0], 1);
}