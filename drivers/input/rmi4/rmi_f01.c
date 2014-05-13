/*
 * Copyright (c) 2011-2014 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/kconfig.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "rmi_driver.h"
#include "rmi_f01.h"

static int rmi_f01_alloc_memory(struct rmi_function *fn,
				int num_of_irq_regs)
{
	struct f01_data *f01;

	f01 = devm_kzalloc(&fn->dev, sizeof(struct f01_data), GFP_KERNEL);
	if (!f01) {
		dev_err(&fn->dev, "Failed to allocate fn_01_data.\n");
		return -ENOMEM;
	}

	f01->device_control.interrupt_enable = devm_kzalloc(&fn->dev,
			sizeof(u8) * (num_of_irq_regs),
			GFP_KERNEL);
	if (!f01->device_control.interrupt_enable) {
		dev_err(&fn->dev, "Failed to allocate interrupt enable.\n");
		return -ENOMEM;
	}
	fn->data = f01;

	return 0;
}

#define PACKAGE_ID_BYTES 4
#define BUILD_ID_BYTES 3

int rmi_f01_read_properties(struct rmi_device *rmi_dev, u16 query_base_addr,
				   struct f01_basic_properties *props)
{
	u8 basic_query[RMI_F01_BASIC_QUERY_LEN];
	u8 info_buf[4];
	int error;
	int i;
	u16 query_addr = query_base_addr;
	u16 prod_info_addr;

	error = rmi_read_block(rmi_dev, query_base_addr,
			       basic_query, sizeof(basic_query));
	if (error < 0) {
		dev_err(&rmi_dev->dev, "Failed to read device query registers.\n");
		return error;
	}

	/* Now parse what we got */
	props->manufacturer_id = basic_query[0];

	props->has_lts = basic_query[1] & RMI_F01_QRY1_HAS_LTS;
	props->has_adjustable_doze =
			basic_query[1] & RMI_F01_QRY1_HAS_ADJ_DOZE;
	props->has_adjustable_doze_holdoff =
			basic_query[1] & RMI_F01_QRY1_HAS_ADJ_DOZE_HOFF;
	props->has_query42 = basic_query[1] & RMI_F01_QRY1_HAS_PROPS_2;

	props->productinfo =
			((basic_query[2] & RMI_F01_QRY2_PRODINFO_MASK) << 7) |
			(basic_query[3] & RMI_F01_QRY2_PRODINFO_MASK);

	snprintf(props->dom, sizeof(props->dom), "20%02d/%02d/%02d",
		 basic_query[5] & RMI_F01_QRY5_YEAR_MASK,
		 basic_query[6] & RMI_F01_QRY6_MONTH_MASK,
		 basic_query[7] & RMI_F01_QRY7_DAY_MASK);

	memcpy(props->product_id, &basic_query[11], RMI_PRODUCT_ID_LENGTH);
	props->product_id[RMI_PRODUCT_ID_LENGTH] = '\0';
	query_addr += 11;


	error = rmi_read_block(rmi_dev, query_addr, props->product_id,
				RMI_PRODUCT_ID_LENGTH);
	if (error < 0) {
		dev_err(&rmi_dev->dev, "Failed to read product ID.\n");
		return error;
	}
	props->product_id[RMI_PRODUCT_ID_LENGTH] = '\0';

	/* We'll come back and use this later, depending on some other query
	 * bits.
	 */
	prod_info_addr = query_addr + 6;

	query_addr += RMI_PRODUCT_ID_LENGTH;
	if (props->has_lts) {
		error = rmi_read(rmi_dev, query_addr, info_buf);
		if (error < 0) {
			dev_err(&rmi_dev->dev, "Failed to read LTS info.\n");
			return error;
		}
		props->slave_asic_rows = info_buf[0] &
				RMI_F01_QRY21_SLAVE_ROWS_MASK;
		props->slave_asic_columns = (info_buf[1] &
				RMI_F01_QRY21_SLAVE_COLUMNS_MASK) >> 3;
		query_addr++;
	}

	if (props->has_sensor_id) {
		error = rmi_read(rmi_dev, query_addr, &props->sensor_id);
		if (error < 0) {
			dev_err(&rmi_dev->dev, "Failed to read sensor ID.\n");
			return error;
		}
		query_addr++;
	}

	/* Maybe skip a block of undefined LTS registers. */
	if (props->has_lts)
		query_addr += RMI_F01_LTS_RESERVED_SIZE;

	if (props->has_query42) {
		error = rmi_read(rmi_dev, query_addr, info_buf);
		if (error < 0) {
			dev_err(&rmi_dev->dev, "Failed to read additional properties.\n");
			return error;
		}
		props->has_ds4_queries = info_buf[0] &
				RMI_F01_QRY42_DS4_QUERIES;
		props->has_multi_physical = info_buf[0] &
				RMI_F01_QRY42_MULTI_PHYS;
		props->has_guest = info_buf[0] & RMI_F01_QRY42_GUEST;
		props->has_swr = info_buf[0] & RMI_F01_QRY42_SWR;
		props->has_nominal_report_rate = info_buf[0] &
				RMI_F01_QRY42_NOMINAL_REPORT;
		props->has_recalibration_interval = info_buf[0] &
				RMI_F01_QRY42_RECAL_INTERVAL;
		query_addr++;
	}

	if (props->has_ds4_queries) {
		error = rmi_read(rmi_dev, query_addr, &props->ds4_query_length);
		if (error < 0) {
			dev_err(&rmi_dev->dev, "Failed to read DS4 query length size.\n");
			return error;
		}
		query_addr++;
	}

	for (i = 1; i <= props->ds4_query_length; i++) {
		u8 val;
		error = rmi_read(rmi_dev, query_addr, &val);
		query_addr++;
		if (error < 0) {
			dev_err(&rmi_dev->dev, "Failed to read F01_RMI_QUERY43.%02d, code: %d.\n",
				i, error);
			continue;
		}
		switch (i) {
		case 1:
			props->has_package_id_query = val &
					RMI_F01_QRY43_01_PACKAGE_ID;
			props->has_build_id_query = val &
					RMI_F01_QRY43_01_BUILD_ID;
			props->has_reset_query = val & RMI_F01_QRY43_01_RESET;
			props->has_maskrev_query = val &
					RMI_F01_QRY43_01_PACKAGE_ID;
			break;
		case 2:
			props->has_i2c_control = val & RMI_F01_QRY43_02_I2C_CTL;
			props->has_spi_control = val & RMI_F01_QRY43_02_SPI_CTL;
			props->has_attn_control = val &
					RMI_F01_QRY43_02_ATTN_CTL;
			props->has_win8_vendor_info = val &
					RMI_F01_QRY43_02_WIN8;
			props->has_timestamp = val & RMI_F01_QRY43_02_TIMESTAMP;
			break;
		case 3:
			props->has_tool_id_query = val &
					RMI_F01_QRY43_03_TOOL_ID;
			props->has_fw_revision_query = val &
					RMI_F01_QRY43_03_FW_REVISION;
			break;
		default:
			dev_warn(&rmi_dev->dev, "No handling for F01_RMI_QUERY43.%02d.\n",
				 i);
		}
	}

	/* If present, the ASIC package ID registers are overlaid on the
	 * product ID. Go back to the right address (saved previously) and
	 * read them.
	 */
	if (props->has_package_id_query) {
		error = rmi_read_block(rmi_dev, prod_info_addr, info_buf,
				PACKAGE_ID_BYTES);
		if (error < 0)
			dev_warn(&rmi_dev->dev, "Failed to read package ID.\n");
		else {
			u16 *val = (u16 *)info_buf;
			props->package_id = le16_to_cpu(*val);
			val = (u16 *)(info_buf + 2);
			props->package_rev = le16_to_cpu(*val);
		}
	}
	prod_info_addr++;

	/* The firmware build id (if present) is similarly overlaid on product
	 * ID registers.  Go back again and read that data.
	 */
	if (props->has_build_id_query) {
		error = rmi_read_block(rmi_dev, prod_info_addr, info_buf,
				BUILD_ID_BYTES);
		if (error < 0)
			dev_warn(&rmi_dev->dev, "Failed to read FW build ID.\n");
		else {
			u16 *val = (u16 *)info_buf;
			props->build_id = le16_to_cpu(*val);
			props->build_id += info_buf[2] * 65536;
			dev_info(&rmi_dev->dev, "FW build ID: %#08x (%u).\n",
				props->build_id, props->build_id);
		}
	}

	return 0;
}

static int rmi_f01_initialize(struct rmi_function *fn)
{
	u8 temp;
	int error;
	u16 ctrl_base_addr;
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct rmi_driver_data *driver_data = dev_get_drvdata(&rmi_dev->dev);
	struct f01_data *data = fn->data;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);

	mutex_init(&data->control_mutex);

	/*
	 * Set the configured bit and (optionally) other important stuff
	 * in the device control register.
	 */
	ctrl_base_addr = fn->fd.control_base_addr;
	error = rmi_read_block(rmi_dev, fn->fd.control_base_addr,
			&data->device_control.ctrl0,
			sizeof(data->device_control.ctrl0));
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read F01 control.\n");
		return error;
	}
	switch (pdata->power_management.nosleep) {
	case RMI_F01_NOSLEEP_DEFAULT:
		break;
	case RMI_F01_NOSLEEP_OFF:
		data->device_control.ctrl0 &= ~RMI_F01_CRTL0_NOSLEEP_BIT;
		break;
	case RMI_F01_NOSLEEP_ON:
		data->device_control.ctrl0 |= RMI_F01_CRTL0_NOSLEEP_BIT;
		break;
	}

	/*
	 * Sleep mode might be set as a hangover from a system crash or
	 * reboot without power cycle.  If so, clear it so the sensor
	 * is certain to function.
	 */
	if ((data->device_control.ctrl0 & RMI_F01_CTRL0_SLEEP_MODE_MASK) !=
			RMI_SLEEP_MODE_NORMAL) {
		dev_warn(&fn->dev,
			 "WARNING: Non-zero sleep mode found. Clearing...\n");
		data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	}

	data->device_control.ctrl0 |= RMI_F01_CRTL0_CONFIGURED_BIT;

	error = rmi_write_block(rmi_dev, fn->fd.control_base_addr,
				&data->device_control.ctrl0,
				sizeof(data->device_control.ctrl0));
	if (error < 0) {
		dev_err(&fn->dev, "Failed to write F01 control.\n");
		return error;
	}

	data->irq_count = driver_data->irq_count;
	data->num_of_irq_regs = driver_data->num_of_irq_regs;
	ctrl_base_addr += sizeof(u8);

	data->interrupt_enable_addr = ctrl_base_addr;
	error = rmi_read_block(rmi_dev, ctrl_base_addr,
				data->device_control.interrupt_enable,
				sizeof(u8) * (data->num_of_irq_regs));
	if (error < 0) {
		dev_err(&fn->dev,
			"Failed to read F01 control interrupt enable register.\n");
		goto error_exit;
	}

	ctrl_base_addr += data->num_of_irq_regs;

	/* dummy read in order to clear irqs */
	error = rmi_read(rmi_dev, fn->fd.data_base_addr + 1, &temp);
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read Interrupt Status.\n");
		return error;
	}

	error = rmi_f01_read_properties(rmi_dev, fn->fd.query_base_addr,
		&data->properties);
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read F01 properties.\n");
		return error;
	}
	dev_info(&fn->dev, "found RMI device, manufacturer: %s, product: %s, fw build: %d.\n",
		 data->properties.manufacturer_id == 1 ?
							"Synaptics" : "unknown",
		 data->properties.product_id, data->properties.build_id);

	/* read control register */
	if (data->properties.has_adjustable_doze) {
		data->doze_interval_addr = ctrl_base_addr;
		ctrl_base_addr++;

		if (pdata->power_management.doze_interval) {
			data->device_control.doze_interval =
				pdata->power_management.doze_interval;
			error = rmi_write(rmi_dev, data->doze_interval_addr,
					data->device_control.doze_interval);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to configure F01 doze interval register.\n");
				goto error_exit;
			}
		} else {
			error = rmi_read(rmi_dev, data->doze_interval_addr,
					&data->device_control.doze_interval);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to read F01 doze interval register.\n");
				goto error_exit;
			}
		}

		data->wakeup_threshold_addr = ctrl_base_addr;
		ctrl_base_addr++;

		if (pdata->power_management.wakeup_threshold) {
			data->device_control.wakeup_threshold =
				pdata->power_management.wakeup_threshold;
			error = rmi_write(rmi_dev, data->wakeup_threshold_addr,
					data->device_control.wakeup_threshold);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to configure F01 wakeup threshold register.\n");
				goto error_exit;
			}
		} else {
			error = rmi_read(rmi_dev, data->wakeup_threshold_addr,
					&data->device_control.wakeup_threshold);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to read F01 wakeup threshold register.\n");
				goto error_exit;
			}
		}
	}

	if (data->properties.has_adjustable_doze_holdoff) {
		data->doze_holdoff_addr = ctrl_base_addr;
		ctrl_base_addr++;

		if (pdata->power_management.doze_holdoff) {
			data->device_control.doze_holdoff =
				pdata->power_management.doze_holdoff;
			error = rmi_write(rmi_dev, data->doze_holdoff_addr,
					data->device_control.doze_holdoff);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to configure F01 doze holdoff register.\n");
				goto error_exit;
			}
		} else {
			error = rmi_read(rmi_dev, data->doze_holdoff_addr,
					&data->device_control.doze_holdoff);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to read F01 doze holdoff register.\n");
				goto error_exit;
			}
		}
	}

	error = rmi_read_block(rmi_dev, fn->fd.data_base_addr,
		&data->device_status, sizeof(data->device_status));
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read device status.\n");
		goto error_exit;
	}

	driver_data->f01_bootloader_mode =
			RMI_F01_STATUS_BOOTLOADER(data->device_status);
	if (driver_data->f01_bootloader_mode)
		dev_warn(&rmi_dev->dev,
			 "WARNING: RMI4 device is in bootloader mode!\n");


	if (RMI_F01_STATUS_UNCONFIGURED(data->device_status)) {
		dev_err(&fn->dev,
			"Device was reset during configuration process, status: %#02x!\n",
			RMI_F01_STATUS_CODE(data->device_status));
		error = -EINVAL;
		goto error_exit;
	}

	return 0;

 error_exit:
	kfree(data);
	return error;
}

static int rmi_f01_config(struct rmi_function *fn)
{
	struct f01_data *data = fn->data;
	int retval;

	retval = rmi_write_block(fn->rmi_dev, fn->fd.control_base_addr,
				 &data->device_control.ctrl0,
				 sizeof(data->device_control.ctrl0));
	if (retval < 0) {
		dev_err(&fn->dev, "Failed to write device_control.reg.\n");
		return retval;
	}

	retval = rmi_write_block(fn->rmi_dev, data->interrupt_enable_addr,
				 data->device_control.interrupt_enable,
				 sizeof(u8) * data->num_of_irq_regs);

	if (retval < 0) {
		dev_err(&fn->dev, "Failed to write interrupt enable.\n");
		return retval;
	}
	if (data->properties.has_lts) {
		retval = rmi_write_block(fn->rmi_dev, data->doze_interval_addr,
					 &data->device_control.doze_interval,
					 sizeof(u8));
		if (retval < 0) {
			dev_err(&fn->dev, "Failed to write doze interval.\n");
			return retval;
		}
	}

	if (data->properties.has_adjustable_doze) {
		retval = rmi_write_block(fn->rmi_dev,
					 data->wakeup_threshold_addr,
					 &data->device_control.wakeup_threshold,
					 sizeof(u8));
		if (retval < 0) {
			dev_err(&fn->dev, "Failed to write wakeup threshold.\n");
			return retval;
		}
	}

	if (data->properties.has_adjustable_doze_holdoff) {
		retval = rmi_write_block(fn->rmi_dev, data->doze_holdoff_addr,
					 &data->device_control.doze_holdoff,
					 sizeof(u8));
		if (retval < 0) {
			dev_err(&fn->dev, "Failed to write doze holdoff.\n");
			return retval;
		}
	}
	return 0;
}

static int rmi_f01_probe(struct rmi_function *fn)
{
	struct rmi_driver_data *driver_data =
			dev_get_drvdata(&fn->rmi_dev->dev);
	int error;

	error = rmi_f01_alloc_memory(fn, driver_data->num_of_irq_regs);
	if (error)
		return error;

	error = rmi_f01_initialize(fn);
	if (error)
		return error;

	return 0;
}

static void rmi_f01_remove(struct rmi_function *fn)
{
	/* Placeholder for now. */
}

#ifdef CONFIG_PM_SLEEP
static int rmi_f01_suspend(struct device *dev)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f01_data *data = fn->data;
	int error;

	data->old_nosleep = data->device_control.ctrl0 &
					RMI_F01_CRTL0_NOSLEEP_BIT;
	data->device_control.ctrl0 &= ~RMI_F01_CRTL0_NOSLEEP_BIT;

	data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	data->device_control.ctrl0 |= RMI_SLEEP_MODE_SENSOR_SLEEP;

	error = rmi_write_block(rmi_dev,
				fn->fd.control_base_addr,
				&data->device_control.ctrl0,
				sizeof(data->device_control.ctrl0));
	if (error < 0) {
		dev_err(&fn->dev, "Failed to write sleep mode. Code: %d.\n",
			error);
		if (data->old_nosleep)
			data->device_control.ctrl0 |=
					RMI_F01_CRTL0_NOSLEEP_BIT;
		data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
		data->device_control.ctrl0 |= RMI_SLEEP_MODE_NORMAL;
		return error;
	}

	return 0;
}

static int rmi_f01_resume(struct device *dev)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f01_data *data = fn->data;
	int error;

	if (data->old_nosleep)
		data->device_control.ctrl0 |= RMI_F01_CRTL0_NOSLEEP_BIT;

	data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	data->device_control.ctrl0 |= RMI_SLEEP_MODE_NORMAL;

	error = rmi_write_block(rmi_dev, fn->fd.control_base_addr,
				&data->device_control.ctrl0,
				sizeof(data->device_control.ctrl0));
	if (error < 0) {
		dev_err(&fn->dev,
			"Failed to restore normal operation. Code: %d.\n",
			error);
		return error;
	}

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(rmi_f01_pm_ops, rmi_f01_suspend, rmi_f01_resume);

static int rmi_f01_attention(struct rmi_function *fn,
			     unsigned long *irq_bits)
{
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f01_data *data = fn->data;
	int retval;

	retval = rmi_read_block(rmi_dev, fn->fd.data_base_addr,
		&data->device_status, sizeof(data->device_status));
	if (retval < 0) {
		dev_err(&fn->dev, "Failed to read device status, code: %d.\n",
			retval);
		return retval;
	}

	if (RMI_F01_STATUS_UNCONFIGURED(data->device_status)) {
		dev_warn(&fn->dev, "Device reset detected.\n");
		retval = rmi_dev->driver->reset_handler(rmi_dev);
		if (retval < 0)
			return retval;
	}
	return 0;
}

static struct rmi_function_handler rmi_f01_handler = {
	.driver = {
		.name	= "rmi_f01",
		.pm	= &rmi_f01_pm_ops,
		/*
		 * Do not allow user unbinding F01 as it is critical
		 * function.
		 */
		.suppress_bind_attrs = true,
	},
	.func		= 0x01,
	.probe		= rmi_f01_probe,
	.remove		= rmi_f01_remove,
	.config		= rmi_f01_config,
	.attention	= rmi_f01_attention,
};

int __init rmi_register_f01_handler(void)
{
	return rmi_register_function_handler(&rmi_f01_handler);
}

void rmi_unregister_f01_handler(void)
{
	rmi_unregister_function_handler(&rmi_f01_handler);
}
