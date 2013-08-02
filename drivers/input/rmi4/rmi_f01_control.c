/*
 * Copyright (c) 2011-2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/fs.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>

#include "rmi_control.h"
#include "rmi_driver.h"
#include "rmi_f01.h"

struct f01_ctl_data {
	struct rmi_function *f01_dev;
	struct rmi_control_handler_data hdata;
};

#ifdef	CONFIG_RMI4_DEBUG
struct f01_debugfs_data {
	bool done;
	struct rmi_function *fn;
};

static int f01_debug_open(struct inode *inodep, struct file *filp)
{
	struct f01_debugfs_data *data;
	struct rmi_function *fn = inodep->i_private;

	data = kzalloc(sizeof(struct f01_debugfs_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->fn = fn;
	filp->private_data = data;
	return 0;
}

static int f01_debug_release(struct inode *inodep, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}

static ssize_t interrupt_enable_read(struct file *filp, char __user *buffer,
				     size_t size, loff_t *offset) {
	int i;
	int len;
	int total_len = 0;
	char local_buf[size];
	char *current_buf = local_buf;
	struct f01_debugfs_data *data = filp->private_data;
	struct f01_data *f01 = data->fn->data;

	if (data->done)
		return 0;

	data->done = 1;

	/* loop through each irq value and copy its
	 * string representation into buf */
	for (i = 0; i < f01->irq_count; i++) {
		int irq_reg;
		int irq_shift;
		int interrupt_enable;

		irq_reg = i / 8;
		irq_shift = i % 8;
		interrupt_enable =
		    ((f01->device_control.interrupt_enable[irq_reg]
			>> irq_shift) & 0x01);

		/* get next irq value and write it to buf */
		len = snprintf(current_buf, size - total_len,
			"%u ", interrupt_enable);
		/* bump up ptr to next location in buf if the
		 * snprintf was valid.  Otherwise issue an error
		 * and return. */
		if (len > 0) {
			current_buf += len;
			total_len += len;
		} else {
			dev_err(&data->fn->dev, "Failed to build interrupt_enable buffer, code = %d.\n",
						len);
			return snprintf(local_buf, size, "unknown\n");
		}
	}
	len = snprintf(current_buf, size - total_len, "\n");
	if (len > 0)
		total_len += len;
	else
		dev_warn(&data->fn->dev, "%s: Failed to append carriage return.\n",
			 __func__);

	if (copy_to_user(buffer, local_buf, total_len))
		return -EFAULT;

	return total_len;
}

static ssize_t interrupt_enable_write(struct file *filp,
		const char __user *buffer, size_t size, loff_t *offset) {
	int retval;
	char buf[size];
	char *local_buf = buf;
	int i;
	int irq_count = 0;
	int irq_reg = 0;
	struct f01_debugfs_data *data = filp->private_data;
	struct f01_data *f01 = data->fn->data;

	retval = copy_from_user(buf, buffer, size);
	if (retval)
		return -EFAULT;

	for (i = 0; i < f01->irq_count && *local_buf != 0;
	     i++, local_buf += 2) {
		int irq_shift;
		int interrupt_enable;
		int result;

		irq_reg = i / 8;
		irq_shift = i % 8;

		/* get next interrupt mapping value and store and bump up to
		 * point to next item in local_buf */
		result = sscanf(local_buf, "%u", &interrupt_enable);
		if ((result != 1) ||
			(interrupt_enable != 0 && interrupt_enable != 1)) {
			dev_err(&data->fn->dev, "Interrupt enable[%d] is not a valid value 0x%x.\n",
				i, interrupt_enable);
			return -EINVAL;
		}
		if (interrupt_enable == 0) {
			f01->device_control.interrupt_enable[irq_reg] &=
				(1 << irq_shift) ^ 0xFF;
		} else
			f01->device_control.interrupt_enable[irq_reg] |=
				(1 << irq_shift);
		irq_count++;
	}

	/* Make sure the irq count matches */
	if (irq_count != f01->irq_count) {
		dev_err(&data->fn->dev, "Interrupt enable count of %d doesn't match device count of %d.\n",
			 irq_count, f01->irq_count);
		return -EINVAL;
	}

	/* write back to the control register */
	retval = rmi_write_block(data->fn->rmi_dev,
			f01->interrupt_enable_addr,
			f01->device_control.interrupt_enable,
			f01->num_of_irq_regs);
	if (retval < 0) {
		dev_err(&data->fn->dev, "Could not write interrupt_enable mask to %#06x\n",
			f01->interrupt_enable_addr);
		return retval;
	}

	return size;
}

static const struct file_operations interrupt_enable_fops = {
	.owner = THIS_MODULE,
	.open = f01_debug_open,
	.release = f01_debug_release,
	.read = interrupt_enable_read,
	.write = interrupt_enable_write,
};

static int rmi_f01_setup_debugfs(struct f01_ctl_data *ctl_data)
{
	struct rmi_function *fn = ctl_data->f01_dev;
	struct dentry *entry;

	if (!fn->debugfs_root)
		return -ENODEV;

	entry = debugfs_create_file("interrupt_enable", RMI_RW_ATTR,
			fn->debugfs_root, fn, &interrupt_enable_fops);
	if (!entry)
		dev_warn(&fn->dev,
			 "Failed to create debugfs interrupt_enable.\n");

	return 0;
}

#else
#define rmi_f01_setup_debugfs(fn) 0
#endif

static ssize_t rmi_fn_01_productinfo_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%#06x\n",
			data->properties.productinfo);
}

static ssize_t rmi_fn_01_productid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%s\n", data->product_id);
}

static ssize_t rmi_fn_01_manufacturer_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n",
			data->properties.manufacturer_id);
}

static ssize_t rmi_fn_01_slave_rows_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->properties.slave_asic_rows);
}

static ssize_t rmi_fn_01_slave_columns_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->properties.slave_asic_columns);
}

static ssize_t rmi_fn_01_sensor_id_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->properties.sensor_id);
}

static ssize_t rmi_fn_01_serialization_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	int i, n, count = 0;
	char *local_buf = buf;

	for (i = 0; i < F01_SERIALIZATION_SIZE; i++) {
		n = snprintf(local_buf, PAGE_SIZE - count, "%02X ",
			     data->serialization[i]);
		count += n;
		local_buf += n;
	}
	return count;
}

static ssize_t rmi_fn_01_reset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	unsigned int reset;
	int error = 0;

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;
	if (reset < 0 || reset > 1)
		return -EINVAL;

	/* Per spec, 0 has no effect, so we skip it entirely. */
	if (reset) {
		/* Command register always reads as 0, so just use a local. */
		u8 command = RMI_F01_CMD_DEVICE_RESET;

		error = rmi_write_block(fn->rmi_dev,
					 fn->fd.command_base_addr,
					 &command, sizeof(command));
		if (error < 0) {
			dev_err(dev, "Failed to issue reset command, code = %d.",
						error);
			return error;
		}
	}

	return count;
}

static ssize_t rmi_fn_01_sleepmode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned int value = data->device_control.ctrl0 &
					RMI_F01_CTRL0_SLEEP_MODE_MASK;

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t rmi_fn_01_sleepmode_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || !RMI_IS_VALID_SLEEPMODE(new_value)) {
		dev_err(dev, "%s: Invalid sleep mode %s.", __func__, buf);
		return -EINVAL;
	}

	retval = mutex_lock_interruptible(&data->control_mutex);
	if (retval)
		return retval;

	dev_dbg(dev, "Setting sleep mode to %ld.", new_value);

	data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	data->device_control.ctrl0 |= new_value;

	retval = rmi_write_block(fn->rmi_dev, fn->fd.control_base_addr,
				 &data->device_control.ctrl0,
			  sizeof(data->device_control.ctrl0));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write sleep mode, code %d.\n", retval);

	mutex_unlock(&data->control_mutex);
	return retval;
}

static ssize_t rmi_fn_01_nosleep_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned int value = !!(data->device_control.ctrl0 &
					RMI_F01_CRTL0_NOSLEEP_BIT);

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t rmi_fn_01_nosleep_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 1) {
		dev_err(dev, "%s: Invalid nosleep bit %s.", __func__, buf);
		return -EINVAL;
	}

	retval = mutex_lock_interruptible(&data->control_mutex);
	if (retval)
		return retval;

	if (new_value)
		data->device_control.ctrl0 |= RMI_F01_CRTL0_NOSLEEP_BIT;
	else
		data->device_control.ctrl0 &= ~RMI_F01_CRTL0_NOSLEEP_BIT;

	retval = rmi_write_block(fn->rmi_dev, fn->fd.control_base_addr,
				 &data->device_control.ctrl0,
			  sizeof(data->device_control.ctrl0));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write nosleep bit.\n");

	mutex_unlock(&data->control_mutex);
	return retval;
}

static ssize_t rmi_fn_01_chargerinput_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned int value = !!(data->device_control.ctrl0 &
	RMI_F01_CRTL0_CHARGER_BIT);

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t rmi_fn_01_chargerinput_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 1) {
		dev_err(dev, "%s: Invalid chargerinput bit %s.", __func__, buf);
		return -EINVAL;
	}

	retval = mutex_lock_interruptible(&data->control_mutex);
	if (retval)
		return retval;

	if (new_value)
		data->device_control.ctrl0 |= RMI_F01_CRTL0_CHARGER_BIT;
	else
		data->device_control.ctrl0 &= ~RMI_F01_CRTL0_CHARGER_BIT;

	retval = rmi_write_block(fn->rmi_dev, fn->fd.control_base_addr,
				 &data->device_control.ctrl0,
			  sizeof(data->device_control.ctrl0));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write chargerinput bit.\n");

	mutex_unlock(&data->control_mutex);
	return retval;
}

static ssize_t rmi_fn_01_reportrate_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	int value = !!(data->device_control.ctrl0 &
					RMI_F01_CRTL0_REPORTRATE_BIT);

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t rmi_fn_01_reportrate_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 1) {
		dev_err(dev, "%s: Invalid reportrate bit %s.", __func__, buf);
		return -EINVAL;
	}

	retval = mutex_lock_interruptible(&data->control_mutex);
	if (retval)
		return retval;

	if (new_value)
		data->device_control.ctrl0 |= RMI_F01_CRTL0_REPORTRATE_BIT;
	else
		data->device_control.ctrl0 &= ~RMI_F01_CRTL0_REPORTRATE_BIT;

	retval = rmi_write_block(fn->rmi_dev, fn->fd.control_base_addr,
				 &data->device_control.ctrl0,
			  sizeof(data->device_control.ctrl0));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write reportrate bit.\n");

	mutex_unlock(&data->control_mutex);
	return retval;
}

static ssize_t rmi_fn_01_interrupt_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	int i, len, total_len = 0;
	char *current_buf = buf;

	/* loop through each irq value and copy its
	 * string representation into buf */
	for (i = 0; i < data->irq_count; i++) {
		int irq_reg;
		int irq_shift;
		int interrupt_enable;

		irq_reg = i / 8;
		irq_shift = i % 8;
		interrupt_enable =
		    ((data->device_control.interrupt_enable[irq_reg]
			>> irq_shift) & 0x01);

		/* get next irq value and write it to buf */
		len = snprintf(current_buf, PAGE_SIZE - total_len,
			"%u ", interrupt_enable);
		/* bump up ptr to next location in buf if the
		 * snprintf was valid.  Otherwise issue an error
		 * and return. */
		if (len > 0) {
			current_buf += len;
			total_len += len;
		} else {
			dev_err(dev, "Failed to build interrupt_enable buffer, code = %d.\n",
						len);
			return snprintf(buf, PAGE_SIZE, "unknown\n");
		}
	}
	len = snprintf(current_buf, PAGE_SIZE - total_len, "\n");
	if (len > 0)
		total_len += len;
	else
		dev_warn(dev, "%s: Failed to append carriage return.\n",
			 __func__);
	return total_len;

}

static ssize_t rmi_fn_01_doze_interval_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_control.doze_interval);

}

static ssize_t rmi_fn_01_doze_interval_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;
	u16 ctrl_addr;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 255) {
		dev_err(dev, "%s: Invalid doze interval %s.", __func__, buf);
		return -EINVAL;
	}

	data->device_control.doze_interval = new_value;
	ctrl_addr = fn->fd.control_base_addr + sizeof(u8) +
			(sizeof(u8)*(data->num_of_irq_regs));
	dev_dbg(dev, "doze_interval store address %x, value %d",
		ctrl_addr, data->device_control.doze_interval);

	retval = rmi_write_block(fn->rmi_dev, data->doze_interval_addr,
			&data->device_control.doze_interval,
			sizeof(u8));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write doze interval.\n");
	return retval;

}

static ssize_t rmi_fn_01_wakeup_threshold_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_control.wakeup_threshold);
}

static ssize_t rmi_fn_01_wakeup_threshold_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 255) {
		dev_err(dev, "%s: Invalid wakeup threshold %s.", __func__, buf);
		return -EINVAL;
	}

	data->device_control.doze_interval = new_value;
	retval = rmi_write_block(fn->rmi_dev, data->wakeup_threshold_addr,
			&data->device_control.wakeup_threshold,
			sizeof(u8));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write wakeup threshold.\n");
	return retval;

}

static ssize_t rmi_fn_01_doze_holdoff_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_control.doze_holdoff);

}

static ssize_t rmi_fn_01_doze_holdoff_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 255) {
		dev_err(dev, "%s: Invalid doze holdoff %s.", __func__, buf);
		return -EINVAL;
	}

	data->device_control.doze_interval = new_value;
	retval = rmi_write_block(fn->rmi_dev, data->doze_holdoff_addr,
			&data->device_control.doze_holdoff,
			sizeof(u8));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write doze holdoff.\n");
	return retval;

}

static ssize_t rmi_fn_01_configured_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned int value = !!(data->device_control.ctrl0 &
					RMI_F01_CRTL0_CONFIGURED_BIT);

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t rmi_fn_01_unconfigured_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			RMI_F01_STATUS_UNCONFIGURED(data->device_status));
}

static ssize_t rmi_fn_01_flashprog_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function *fn = to_rmi_function(dev);

	data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			RMI_F01_STATUS_BOOTLOADER(data->device_status));
}

static ssize_t rmi_fn_01_statuscode_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function *fn = to_rmi_function(dev);

	data = fn->data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n",
			RMI_F01_STATUS_CODE(data->device_status));
}

static struct device_attribute dev_attr_doze_interval =
		__ATTR(doze_interval, RMI_RW_ATTR,
			rmi_fn_01_doze_interval_show,
				rmi_fn_01_doze_interval_store);
static struct device_attribute dev_attr_wakeup_threshold =
		__ATTR(wakeup_threshold, RMI_RW_ATTR,
			rmi_fn_01_wakeup_threshold_show,
			rmi_fn_01_wakeup_threshold_store);
static struct device_attribute dev_attr_doze_holdoff =
		__ATTR(doze_holdoff, RMI_RW_ATTR,
			rmi_fn_01_doze_holdoff_show,
			rmi_fn_01_doze_holdoff_store);

static struct device_attribute dev_attr_productinfo =
	__ATTR(productinfo, RMI_RO_ATTR,
	       rmi_fn_01_productinfo_show, NULL);
static struct device_attribute dev_attr_productid =
	__ATTR(productid, RMI_RO_ATTR,
	       rmi_fn_01_productid_show, NULL);
static struct device_attribute dev_attr_manufacturer =
	__ATTR(manufacturer, RMI_RO_ATTR,
	       rmi_fn_01_manufacturer_show, NULL);
static struct device_attribute dev_attr_slave_rows =
	__ATTR(slave_rows, RMI_RO_ATTR,
		rmi_fn_01_slave_rows_show, NULL);
static struct device_attribute dev_attr_slave_columns =
	__ATTR(slave_columns, RMI_RO_ATTR,
	       rmi_fn_01_slave_columns_show, NULL);
static struct device_attribute dev_attr_sensor_id =
	__ATTR(sensor_id, RMI_RO_ATTR,
		rmi_fn_01_sensor_id_show, NULL);

/* control register access */
static struct device_attribute dev_attr_sleepmode =
	__ATTR(sleepmode, RMI_RW_ATTR,
	       rmi_fn_01_sleepmode_show, rmi_fn_01_sleepmode_store);
static struct device_attribute dev_attr_nosleep =
	__ATTR(nosleep, RMI_RW_ATTR,
	       rmi_fn_01_nosleep_show, rmi_fn_01_nosleep_store);
static struct device_attribute dev_attr_chargerinput =
	__ATTR(chargerinput, RMI_RW_ATTR,
	       rmi_fn_01_chargerinput_show, rmi_fn_01_chargerinput_store);
static struct device_attribute dev_attr_reportrate =
	__ATTR(reportrate, RMI_RW_ATTR,
	       rmi_fn_01_reportrate_show, rmi_fn_01_reportrate_store);
/* We don't want arbitrary callers changing the interrupt enable mask,
 * so it's read only.
 */
static struct device_attribute dev_attr_interrupt_enable =
	__ATTR(interrupt_enable, RMI_RO_ATTR,
	       rmi_fn_01_interrupt_enable_show, NULL);

/* We make configured RO, since the driver uses that to look for
 * resets.  We don't want someone faking us out by changing that
 * bit.
 */
static struct device_attribute dev_attr_configured =
	__ATTR(configured, RMI_RO_ATTR,
	       rmi_fn_01_configured_show, NULL);

/* Command register access. */
static struct device_attribute dev_attr_reset =
	__ATTR(reset, RMI_WO_ATTR,
	       NULL, rmi_fn_01_reset_store);

/* Status register access. */
static struct device_attribute dev_attr_unconfigured =
	__ATTR(unconfigured, RMI_RO_ATTR,
	       rmi_fn_01_unconfigured_show, NULL);
static struct device_attribute dev_attr_flashprog =
	__ATTR(flashprog, RMI_RO_ATTR,
	       rmi_fn_01_flashprog_show, NULL);
static struct device_attribute dev_attr_statuscode =
	__ATTR(statuscode, RMI_RO_ATTR,
	       rmi_fn_01_statuscode_show, NULL);
static struct device_attribute dev_attr_serialization =
	__ATTR(serialization, RMI_RO_ATTR,
	       rmi_fn_01_serialization_show, NULL);

static struct attribute *attrs[] = {
	&dev_attr_productinfo.attr,
	&dev_attr_productid.attr,
	&dev_attr_manufacturer.attr,
	&dev_attr_sleepmode.attr,
	&dev_attr_nosleep.attr,
	&dev_attr_chargerinput.attr,
	&dev_attr_reportrate.attr,
	&dev_attr_interrupt_enable.attr,
	&dev_attr_configured.attr,
	&dev_attr_reset.attr,
	&dev_attr_unconfigured.attr,
	&dev_attr_flashprog.attr,
	&dev_attr_statuscode.attr,
	&dev_attr_serialization.attr,
	NULL
};

static struct attribute_group fn01_attrs = GROUP(attrs);


static int rmi_f01_create_sysfs(struct rmi_function *fn)
{
	int retval = 0;
	struct f01_data *data = fn->data;
	dev_dbg(&fn->dev, "Creating sysfs files.");
	if (sysfs_create_group(&fn->dev.kobj, &fn01_attrs) < 0) {
		dev_err(&fn->dev, "Failed to create query sysfs files.");
		return -ENODEV;
	}
	if (data->properties.has_adjustable_doze) {
		retval = sysfs_create_file(&fn->dev.kobj,
			&dev_attr_doze_interval.attr);
		if (retval < 0) {
			dev_err(&fn->dev, "Failed to create sysfs file for doze internal.");
			goto err_remove_sysfs_group;
		}
	}
	if (data->properties.has_adjustable_doze) {
		retval = sysfs_create_file(&fn->dev.kobj,
			&dev_attr_wakeup_threshold.attr);
		if (retval < 0) {
			dev_err(&fn->dev, "Failed to create sysfs file for wakeup threshold.");
			goto err_remove_sysfs_doze_interval;
		}
	}
	if (data->properties.has_adjustable_doze_holdoff) {
		retval = sysfs_create_file(&fn->dev.kobj,
			&dev_attr_doze_holdoff.attr);
		if (retval < 0) {
			dev_err(&fn->dev, "Failed to create sysfs file for doze holdoff.");
			goto err_remove_sysfs_wakeup_threshold;
		}
	}
	if (data->properties.has_lts) {
		retval = sysfs_create_file(&fn->dev.kobj,
			&dev_attr_slave_rows.attr);
		dev_warn(&fn->dev, "Failed to creat sysfs file for slave rows.\n");
		retval = sysfs_create_file(&fn->dev.kobj,
			&dev_attr_slave_columns.attr);
		dev_warn(&fn->dev, "Failed to creat sysfs file for slave columns.\n");
	}
	if (data->properties.has_sensor_id) {
		retval = sysfs_create_file(&fn->dev.kobj,
					   &dev_attr_sensor_id.attr);
		dev_warn(&fn->dev, "Failed to creat sysfs file for sensor id.\n");
	}

	return 0;

err_remove_sysfs_wakeup_threshold:
	sysfs_remove_file(&fn->dev.kobj, &dev_attr_wakeup_threshold.attr);

err_remove_sysfs_doze_interval:
	sysfs_remove_file(&fn->dev.kobj, &dev_attr_doze_interval.attr);

err_remove_sysfs_group:
	sysfs_remove_group(&fn->dev.kobj, &fn01_attrs);
	return retval;
}

static int f01_ctl_cleanup(struct rmi_control_handler_data *hdata)
{
	struct rmi_function *fn;
	struct f01_ctl_data *ctl_data;
	struct f01_data *fn_data;

	ctl_data = container_of(hdata, struct f01_ctl_data, hdata);
	fn = ctl_data->f01_dev;

	debugfs_remove_recursive(fn->debugfs_root);

	sysfs_remove_group(&fn->dev.kobj, &fn01_attrs);

	fn_data = fn->data;

	if (fn_data->properties.has_lts)
		sysfs_remove_file(&fn->dev.kobj,
				  &dev_attr_doze_interval.attr);

	if (fn_data->properties.has_adjustable_doze)
		sysfs_remove_file(&fn->dev.kobj,
				  &dev_attr_wakeup_threshold.attr);

	if (fn_data->properties.has_adjustable_doze_holdoff)
		sysfs_remove_file(&fn->dev.kobj,
				  &dev_attr_doze_holdoff.attr);

	devm_kfree(&fn->dev, ctl_data);

	return 0;
}

static struct rmi_control_handler_data *f01_ctl_attach(struct device *dev, void *data)
{
	struct rmi_function *fn;
	struct f01_ctl_data *ctl_data;
	int retval;

	fn = to_rmi_function(dev);

	ctl_data = devm_kzalloc(dev, sizeof(struct f01_ctl_data), GFP_KERNEL);
	if (!ctl_data)
		return NULL;
	ctl_data->f01_dev = fn;

	retval = rmi_f01_create_sysfs(fn);
	if (retval)
		dev_warn(dev, "Failed to create F01 sysfs structures.\n");

	rmi_f01_setup_debugfs(ctl_data);

	return &ctl_data->hdata;
}

static struct rmi_control_handler handler = {
	.name = "f01",
	.dev_type = &rmi_function_type,
	.function_id = 0x01,
	.attach = f01_ctl_attach,
	.remove = f01_ctl_cleanup,
};

static int __init f01_ctl_init(void)
{
	int error = 0;
	pr_debug("%s: initialization.\n", __func__);

	error = rmi_register_control_handler(&handler);
	if (error)
		pr_warn("%s: WARNING failed to register control handler, code=%d.\n",
			__func__, error);

	pr_debug("%s: done\n", __func__);
	return error;
}

static void __exit f01_ctl_exit(void)
{
	pr_debug("%s: exiting.\n", __func__);

	rmi_unregister_control_handler(&handler);
}

module_init(f01_ctl_init);
module_exit(f01_ctl_exit);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com>");
MODULE_DESCRIPTION("RMI4 F01 Controls");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);