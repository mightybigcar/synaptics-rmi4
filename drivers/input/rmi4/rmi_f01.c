/*
 * Copyright (c) 2011-2013 Synaptics Incorporated
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

#define FUNCTION_NUMBER 0x01

static int rmi_f01_alloc_memory(struct rmi_function_dev *fn_dev,
	int num_of_irq_regs)
{
	struct f01_data *f01;

	f01 = devm_kzalloc(&fn_dev->dev, sizeof(struct f01_data), GFP_KERNEL);
	if (!f01) {
		dev_err(&fn_dev->dev, "Failed to allocate fn_01_data.\n");
		return -ENOMEM;
	}

	f01->device_control.interrupt_enable = devm_kzalloc(&fn_dev->dev,
			sizeof(u8)*(num_of_irq_regs),
			GFP_KERNEL);
	if (!f01->device_control.interrupt_enable) {
		dev_err(&fn_dev->dev, "Failed to allocate interrupt enable.\n");
		return -ENOMEM;
	}
	fn_dev->data = f01;

	return 0;
}

static void get_board_and_rev(struct rmi_function_dev *fn_dev,
			struct rmi_driver_data *driver_data)
{
	struct f01_data *data = fn_dev->data;
	int retval;
	int board = 0, rev = 0;
	int i;
	static const char * const pattern[] = {
		"tm%4d-%d", "s%4d-%d", "s%4d-ver%1d"};
	u8 product_id[RMI_PRODUCT_ID_LENGTH+1];

	for (i = 0; i < strlen(data->product_id); i++)
		product_id[i] = tolower(data->product_id[i]);
	product_id[i] = '\0';

	for (i = 0; i < ARRAY_SIZE(pattern); i++) {
		retval = sscanf(product_id, pattern[i], &board, &rev);
		if (retval)
			break;
	}

	/* save board and rev data in the rmi_driver_data */
	driver_data->board = board;
	driver_data->rev = rev;
	dev_dbg(&fn_dev->dev, "From product ID %s, set board: %d rev: %d\n",
			product_id, driver_data->board, driver_data->rev);
}

#define PACKAGE_ID_BYTES 4
#define BUILD_ID_BYTES 3

static int rmi_f01_initialize(struct rmi_function_dev *fn_dev)
{
	u8 temp;
	int error;
	u16 query_addr = fn_dev->fd.query_base_addr;
	u16 prod_info_addr;
	u8 info_buf[4];
	u16 ctrl_base_addr = fn_dev->fd.control_base_addr;
	struct rmi_device *rmi_dev = fn_dev->rmi_dev;
	struct rmi_driver_data *driver_data = dev_get_drvdata(&rmi_dev->dev);
	struct f01_data *data = fn_dev->data;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);
	u8 basic_query[RMI_F01_BASIC_QUERY_LEN];

	mutex_init(&data->control_mutex);

	/* Set the configured bit and (optionally) other important stuff
	 * in the device control register. */
	error = rmi_read_block(rmi_dev, fn_dev->fd.control_base_addr,
			&data->device_control.ctrl0,
			sizeof(data->device_control.ctrl0));
	if (error < 0) {
		dev_err(&fn_dev->dev, "Failed to read F01 control.\n");
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

	/* Sleep mode might be set as a hangover from a system crash or
	 * reboot without power cycle.  If so, clear it so the sensor
	 * is certain to function.
	 */
	if ((data->device_control.ctrl0 & RMI_F01_CTRL0_SLEEP_MODE_MASK) !=
			RMI_SLEEP_MODE_NORMAL) {
		dev_warn(&fn_dev->dev,
			 "WARNING: Non-zero sleep mode found. Clearing...\n");
		data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	}

	/* Set this to indicate that we've initialized the sensor.  This will
	 * CLEAR the unconfigured bit in the status registers.  If we ever
	 * see unconfigured become set again, we'll know that the sensor has
	 * reset for some reason.
	 */
	data->device_control.ctrl0 |= RMI_F01_CRTL0_CONFIGURED_BIT;

	error = rmi_write_block(rmi_dev, fn_dev->fd.control_base_addr,
			&data->device_control.ctrl0,
			sizeof(data->device_control.ctrl0));
	if (error < 0) {
		dev_err(&fn_dev->dev, "Failed to write F01 control.\n");
		return error;
	}

	data->irq_count = driver_data->irq_count;
	data->num_of_irq_regs = driver_data->num_of_irq_regs;
	ctrl_base_addr += sizeof(u8);

	data->interrupt_enable_addr = ctrl_base_addr;
	error = rmi_read_block(rmi_dev, ctrl_base_addr,
			data->device_control.interrupt_enable,
			sizeof(u8)*(data->num_of_irq_regs));
	if (error < 0) {
		dev_err(&fn_dev->dev, "Failed to read F01 control interrupt enable register.\n");
		goto error_exit;
	}
	ctrl_base_addr += data->num_of_irq_regs;

	/* dummy read in order to clear irqs */
	error = rmi_read(rmi_dev, fn_dev->fd.data_base_addr + 1, &temp);
	if (error < 0) {
		dev_err(&fn_dev->dev, "Failed to read Interrupt Status.\n");
		return error;
	}

	/* read queries */
	error = rmi_read_block(rmi_dev, fn_dev->fd.query_base_addr,
				basic_query, sizeof(basic_query));
	if (error < 0) {
		dev_err(&fn_dev->dev, "Failed to read device query registers.\n");
		return error;
	}

	/* Now parse what we got */
	data->properties.manufacturer_id = basic_query[0];

	data->properties.has_lts = basic_query[1] & RMI_F01_QRY1_HAS_LTS;
	data->properties.has_adjustable_doze =
				basic_query[1] & RMI_F01_QRY1_HAS_ADJ_DOZE;
	data->properties.has_adjustable_doze_holdoff =
				basic_query[1] & RMI_F01_QRY1_HAS_ADJ_DOZE_HOFF;

	snprintf(data->properties.dom, sizeof(data->properties.dom),
		"20%02d/%02d/%02d",
		basic_query[5] & RMI_F01_QRY5_YEAR_MASK,
		basic_query[6] & RMI_F01_QRY6_MONTH_MASK,
		basic_query[7] & RMI_F01_QRY7_DAY_MASK);

	memcpy(data->properties.product_id, &basic_query[11],
	       RMI_PRODUCT_ID_LENGTH);
	data->properties.product_id[RMI_PRODUCT_ID_LENGTH] = '\0';

	data->properties.productinfo =
		((basic_query[2] & RMI_F01_QRY2_PRODINFO_MASK) << 7) |
		(basic_query[3] & RMI_F01_QRY2_PRODINFO_MASK);
	query_addr += sizeof(basic_query);

	/* Now read some specialized query registers. */
	error = rmi_read_block(rmi_dev, query_addr, data->serialization,
				F01_SERIALIZATION_SIZE);
	if (error < 0) {
		dev_err(&fn_dev->dev, "Failed to read device serialization.\n");
		return error;
	}
	query_addr += F01_SERIALIZATION_SIZE;

	error = rmi_read_block(rmi_dev, query_addr, data->product_id,
				RMI_PRODUCT_ID_LENGTH);
	if (error < 0) {
		dev_err(&fn_dev->dev, "Failed to read product ID.\n");
		return error;
	}
	data->product_id[RMI_PRODUCT_ID_LENGTH] = '\0';
	get_board_and_rev(fn_dev, driver_data);

	/* The ASIC package ID registers are overlaid on the product info.
	 * Offset to the right address and read them.
	 */
	prod_info_addr = query_addr + 6;
	error = rmi_read_block(rmi_dev, prod_info_addr, info_buf,
				PACKAGE_ID_BYTES);
	if (error < 0)
		dev_warn(&fn_dev->dev, "Failed to read package ID.\n");
	else {
		u16 *val = (u16 *)info_buf;
		data->package_id = le16_to_cpu(*val);
		val = (u16 *)(info_buf + 2);
		data->package_rev = le16_to_cpu(*val);
	}

	/* The firmware build identifier is similarly overlaid on product info.
	 * Offset again and read that data.
	 */
	prod_info_addr++;
	error = rmi_read_block(rmi_dev, prod_info_addr, info_buf,
				BUILD_ID_BYTES);
	if (error < 0)
		dev_warn(&fn_dev->dev, "Failed to read build ID.\n");
	else {
		u16 *val = (u16 *)info_buf;
		data->build_id = le16_to_cpu(*val);
		data->build_id += info_buf[2] * 65536;
	}
	dev_info(&fn_dev->dev, "found RMI device, manufacturer: %s, product: %s, date: %s\n",
		 data->properties.manufacturer_id == 1 ?
		 "synaptics" : "unknown", data->product_id, data->properties.dom);
	dev_info(&fn_dev->dev, "Package: ID %#06x, rev %#06x; FW build ID: %#08x (%u).\n",
		data->package_id, data->package_rev,
		data->build_id, data->build_id);

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
				dev_err(&fn_dev->dev, "Failed to configure F01 doze interval register.\n");
				goto error_exit;
			}
		} else {
			error = rmi_read(rmi_dev, data->doze_interval_addr,
					&data->device_control.doze_interval);
			if (error < 0) {
				dev_err(&fn_dev->dev, "Failed to read F01 doze interval register.\n");
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
				dev_err(&fn_dev->dev, "Failed to configure F01 wakeup threshold register.\n");
				goto error_exit;
			}
		} else {
			error = rmi_read(rmi_dev, data->wakeup_threshold_addr,
					&data->device_control.wakeup_threshold);
			if (error < 0) {
				dev_err(&fn_dev->dev, "Failed to read F01 wakeup threshold register.\n");
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
				dev_err(&fn_dev->dev, "Failed to configure F01 doze holdoff register.\n");
				goto error_exit;
			}
		} else {
			error = rmi_read(rmi_dev, data->doze_holdoff_addr,
					&data->device_control.doze_holdoff);
			if (error < 0) {
				dev_err(&fn_dev->dev, "Failed to read F01 doze holdoff register.\n");
				goto error_exit;
			}
		}
	}

	error = rmi_read_block(rmi_dev, fn_dev->fd.data_base_addr,
		&data->device_status, sizeof(data->device_status));
	if (error < 0) {
		dev_err(&fn_dev->dev, "Failed to read device status.\n");
		goto error_exit;
	}

	if (RMI_F01_STATUS_UNCONFIGURED(data->device_status)) {
		dev_err(&fn_dev->dev, "Device reset during configuration process, status: %#02x!\n",
				RMI_F01_STATUS_CODE(data->device_status));
		error = -EINVAL;
		goto error_exit;
	}

	driver_data->f01_bootloader_mode =
		RMI_F01_STATUS_BOOTLOADER(data->device_status);
	if (RMI_F01_STATUS_BOOTLOADER(data->device_status))
		dev_warn(&rmi_dev->dev,
			 "WARNING: RMI4 device is in bootloader mode!\n");

	return error;

 error_exit:
	kfree(data);
	return error;
}

static int rmi_f01_config(struct rmi_function_dev *fn_dev)
{
	struct f01_data *data = fn_dev->data;
	int retval;

	retval = rmi_write_block(fn_dev->rmi_dev, fn_dev->fd.control_base_addr,
			&data->device_control.ctrl0,
			sizeof(data->device_control.ctrl0));
	if (retval < 0) {
		dev_err(&fn_dev->dev, "Failed to write device_control.reg.\n");
		return retval;
	}

	retval = rmi_write_block(fn_dev->rmi_dev, data->interrupt_enable_addr,
			data->device_control.interrupt_enable,
			sizeof(u8)*(data->num_of_irq_regs));

	if (retval < 0) {
		dev_err(&fn_dev->dev, "Failed to write interrupt enable.\n");
		return retval;
	}

	if (data->properties.has_adjustable_doze) {
		retval = rmi_write_block(fn_dev->rmi_dev,
					data->doze_interval_addr,
					&data->device_control.doze_interval,
					sizeof(u8));
		if (retval < 0) {
			dev_err(&fn_dev->dev, "Failed to write doze interval.\n");
			return retval;
		}
		retval = rmi_write_block(
				fn_dev->rmi_dev, data->wakeup_threshold_addr,
				&data->device_control.wakeup_threshold,
				sizeof(u8));
		if (retval < 0) {
			dev_err(&fn_dev->dev, "Failed to write wakeup threshold.\n");
			return retval;
		}
	}

	if (data->properties.has_adjustable_doze_holdoff) {
		retval = rmi_write_block(fn_dev->rmi_dev,
					data->doze_holdoff_addr,
					&data->device_control.doze_holdoff,
					sizeof(u8));
		if (retval < 0) {
			dev_err(&fn_dev->dev, "Failed to write doze holdoff.\n");
			return retval;
		}
	}
	return 0;
}

static int rmi_f01_probe(struct rmi_function_dev *fn_dev)
{
	struct rmi_driver_data *driver_data =
			dev_get_drvdata(&fn_dev->rmi_dev->dev);
	int error;

	dev_dbg(&fn_dev->dev, "%s called.\n", __func__);

	error = rmi_f01_alloc_memory(fn_dev, driver_data->num_of_irq_regs);
	if (error < 0)
		return error;

	error = rmi_f01_initialize(fn_dev);
	if (error < 0)
		return error;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rmi_f01_suspend(struct rmi_function_dev *fn_dev)
{
	struct rmi_device *rmi_dev = fn_dev->rmi_dev;
	struct f01_data *data = fn_dev->data;
	int error = 0;

	data->old_nosleep = data->device_control.ctrl0 &
		RMI_F01_CRTL0_NOSLEEP_BIT;
	data->device_control.ctrl0 &= ~RMI_F01_CRTL0_NOSLEEP_BIT;

	data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	data->device_control.ctrl0 |= RMI_SLEEP_MODE_SENSOR_SLEEP;

	error = rmi_write_block(rmi_dev,
			fn_dev->fd.control_base_addr,
			 &data->device_control.ctrl0,
			 sizeof(data->device_control.ctrl0));
	if (error < 0) {
		dev_err(&fn_dev->dev, "Failed to write sleep mode. Code: %d.\n",
			error);
		if (data->old_nosleep)
			data->device_control.ctrl0 |= RMI_F01_CRTL0_NOSLEEP_BIT;
		data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
		data->device_control.ctrl0 |= RMI_SLEEP_MODE_NORMAL;
		return error;
	}

	return 0;
}

static int rmi_f01_resume(struct rmi_function_dev *fn_dev)
{
	struct rmi_device *rmi_dev = fn_dev->rmi_dev;
	struct f01_data *data = fn_dev->data;
	int error;

	if (data->old_nosleep)
		data->device_control.ctrl0 |= RMI_F01_CRTL0_NOSLEEP_BIT;

	data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	data->device_control.ctrl0 |= RMI_SLEEP_MODE_NORMAL;

	error = rmi_write_block(rmi_dev, fn_dev->fd.control_base_addr,
				&data->device_control.ctrl0,
			 sizeof(data->device_control.ctrl0));
	if (error < 0) {
		dev_err(&fn_dev->dev,
			"Failed to restore normal operation. Code: %d.\n",
			error);
		return error;
	}

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static int rmi_f01_attention(struct rmi_function_dev *fn_dev,
						unsigned long *irq_bits)
{
	struct rmi_device *rmi_dev = fn_dev->rmi_dev;
	struct f01_data *data = fn_dev->data;
	int retval;

	retval = rmi_read_block(rmi_dev, fn_dev->fd.data_base_addr,
		&data->device_status, sizeof(data->device_status));
	if (retval < 0) {
		dev_err(&fn_dev->dev, "Failed to read device status, code: %d.\n",
			retval);
		return retval;
	}
	if (RMI_F01_STATUS_UNCONFIGURED(data->device_status)) {
		dev_warn(&fn_dev->dev, "Device reset detected.\n");
		retval = rmi_dev->driver->reset_handler(rmi_dev);
		if (retval < 0)
			return retval;
	}
	return 0;
}

struct rmi_function_driver rmi_f01_driver = {
	.driver = {
		.name = "rmi_f01",
		/*
		 * Do not allow user unbinding of F01 as it is a critical
		 * function.
		 */
		.suppress_bind_attrs = true,
	},
	.func      = FUNCTION_NUMBER,
	.probe     = rmi_f01_probe,
	.config    = rmi_f01_config,
	.attention = rmi_f01_attention,

#ifdef	CONFIG_PM
#if defined(CONFIG_HAS_EARLYSUSPEND)
	.early_suspend = rmi_f01_suspend,
	.late_resume = rmi_f01_resume,
#else
	.suspend = rmi_f01_suspend,
	.resume = rmi_f01_resume,
#endif  /* defined(CONFIG_HAS_EARLYSUSPEND) && !def... */
#endif  /* CONFIG_PM */
};
