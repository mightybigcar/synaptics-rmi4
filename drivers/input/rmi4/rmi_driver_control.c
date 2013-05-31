/*
 * Copyright (c) 2011-2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/kconfig.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>

#include "rmi_control.h"
#include "rmi_driver.h"


struct driver_ctl_data {
	struct rmi_device *rmi_dev;

#ifdef	CONFIG_RMI4_DEBUG
	// TODO: Move delay into SPI module.
	struct dentry *debugfs_delay;
#endif

	struct rmi_control_handler_data hdata;
};

#ifdef	CONFIG_RMI4_DEBUG
struct driver_debugfs_data {
	bool done;
	loff_t pos;
	struct rmi_device *rmi_dev;
};

static int debug_open(struct inode *inodep, struct file *filp)
{
	struct driver_debugfs_data *data;
	struct rmi_device *rmi_dev;

	rmi_dev = inodep->i_private;
	data = kzalloc(sizeof(struct driver_debugfs_data),
				GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->rmi_dev = inodep->i_private;
	filp->private_data = data;
	return 0;
}

static int debug_release(struct inode *inodep, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}

static ssize_t delay_read(struct file *filp, char __user *buffer, size_t size,
		    loff_t *offset) {
	struct driver_debugfs_data *data = filp->private_data;
	struct rmi_device_platform_data *pdata =
			data->rmi_dev->xport->dev->platform_data;
	int retval;
	char *local_buf;

	if (data->done)
		return 0;

	data->done = 1;

	local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
	if (!local_buf)
		return -ENOMEM;

	retval = snprintf(local_buf, size, "%d %d %d %d %d\n",
		pdata->spi_data.read_delay_us, pdata->spi_data.write_delay_us,
		pdata->spi_data.block_delay_us,
		pdata->spi_data.pre_delay_us, pdata->spi_data.post_delay_us);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		retval = -EFAULT;
	kfree(local_buf);

	return retval;
}

static ssize_t delay_write(struct file *filp, const char __user *buffer,
			   size_t size, loff_t *offset) {
	struct driver_debugfs_data *data = filp->private_data;
	struct rmi_device_platform_data *pdata =
			data->rmi_dev->xport->dev->platform_data;
	int retval;
	char *local_buf;
	unsigned int new_read_delay;
	unsigned int new_write_delay;
	unsigned int new_block_delay;
	unsigned int new_pre_delay;
	unsigned int new_post_delay;

	local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
	if (!local_buf)
		return -ENOMEM;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval) {
		kfree(local_buf);
		return -EFAULT;
	}

	retval = sscanf(local_buf, "%u %u %u %u %u", &new_read_delay,
			&new_write_delay, &new_block_delay,
			&new_pre_delay, &new_post_delay);
	kfree(local_buf);

	if (retval != 5) {
		dev_err(&data->rmi_dev->dev,
			"Incorrect number of values provided for delay.");
		return -EINVAL;
	}
	dev_dbg(&data->rmi_dev->dev,
		 "Setting delays to %u %u %u %u %u.\n", new_read_delay,
		 new_write_delay, new_block_delay, new_pre_delay,
		 new_post_delay);
	pdata->spi_data.read_delay_us = new_read_delay;
	pdata->spi_data.write_delay_us = new_write_delay;
	pdata->spi_data.block_delay_us = new_block_delay;
	pdata->spi_data.pre_delay_us = new_pre_delay;
	pdata->spi_data.post_delay_us = new_post_delay;

	return size;
}

static const struct file_operations delay_fops = {
	.owner = THIS_MODULE,
	.open = debug_open,
	.release = debug_release,
	.read = delay_read,
	.write = delay_write,
};

static int setup_debugfs(struct driver_ctl_data *ctl_data)
{
	struct rmi_device *rmi_dev = ctl_data->rmi_dev;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_transport_info *info = &rmi_dev->xport->info;
	int retval = 0;

	if (!rmi_dev->debugfs_root)
		return 0;


	if (IS_ENABLED(CONFIG_RMI4_SPI) && !strncmp("spi", info->proto, 3)) {
		ctl_data->debugfs_delay = debugfs_create_file("delay",
				RMI_RW_ATTR, rmi_dev->debugfs_root, rmi_dev,
				&delay_fops);
		if (!ctl_data->debugfs_delay || IS_ERR(ctl_data->debugfs_delay)) {
			dev_warn(&rmi_dev->dev, "Failed to create debugfs delay.\n");
			ctl_data->debugfs_delay = NULL;
		}
	}

	if (!debugfs_create_u32_array("transport_stats", RMI_RO_ATTR,
		rmi_dev->debugfs_root, (u32 *)&info->tx_count, 6))
		dev_warn(&rmi_dev->dev,
			 "Failed to create debugfs transport_stats\n");


	if (debugfs_create_bool("irq_debug", RMI_RW_ATTR, rmi_dev->debugfs_root,
			&data->irq_debug))
		dev_warn(&rmi_dev->dev, "Failed to create debugfs irq_debug.\n");

	if (!debugfs_create_u32("attn_count", RMI_RO_ATTR,
				rmi_dev->debugfs_root, &data->attn_count))
		dev_warn(&rmi_dev->dev, "Failed to create debugfs attn_count.\n");

	return retval;
}

static void teardown_debugfs(struct driver_ctl_data *data)
{
	debugfs_remove_recursive(data->rmi_dev->debugfs_root);
	if (data->debugfs_delay)
		debugfs_remove(data->debugfs_delay);
}
#else
#define teardown_debugfs(rmi_dev)
#define setup_debugfs(rmi_dev) 0
#endif

/* sysfs show and store fns for driver attributes */

static ssize_t rmi_driver_bsr_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->bsr);
}

static ssize_t rmi_driver_bsr_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int retval;
	unsigned long val;
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	/* need to convert the string data to an actual value */
	retval = strict_strtoul(buf, 10, &val);
	if (retval < 0 || val > 255) {
		dev_err(dev, "Invalid value '%s' written to BSR.\n", buf);
		return -EINVAL;
	}

	retval = rmi_write(rmi_dev, BSR_LOCATION, (u8)val);
	if (retval < 0) {
		dev_err(dev, "%s : failed to write bsr %lu to %#06x\n",
			__func__, val, BSR_LOCATION);
		return retval;
	}

	data->bsr = val;

	return count;
}

static struct device_attribute bsr_attribute = __ATTR(bsr, RMI_RW_ATTR,
	       rmi_driver_bsr_show, rmi_driver_bsr_store);

static int driver_ctl_cleanup(struct rmi_control_handler_data *hdata)
{
	struct device *dev = hdata->dev;
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *driver_data = dev_get_drvdata(&rmi_dev->dev);
	struct driver_ctl_data *ctl_data =
			container_of(hdata, struct driver_ctl_data, hdata);

	teardown_debugfs(ctl_data);

	if (driver_data->pdt_props.has_bsr)
		device_remove_file(&rmi_dev->dev, &bsr_attribute);

	devm_kfree(dev, ctl_data);

	return 0;
}

static struct rmi_control_handler_data *driver_ctl_attach(struct device *dev, void *data)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct driver_ctl_data *ctl_data;
	int retval;
	struct rmi_driver_data *driver_data = dev_get_drvdata(&rmi_dev->dev);

	rmi_dev = to_rmi_device(dev);
	dev_dbg(dev, "%s called.\n", __func__);

	ctl_data = devm_kzalloc(dev, sizeof(struct driver_ctl_data), GFP_KERNEL);
	if (!ctl_data)
		return NULL;
	ctl_data->rmi_dev = rmi_dev;

	dev_dbg(dev, "Checking BSR.\n");
	if (driver_data && driver_data->pdt_props.has_bsr) {
		retval = device_create_file(dev, &bsr_attribute);
		if (retval < 0)
			dev_warn(dev, "Failed to create sysfs file bsr.\n");
	}

	setup_debugfs(ctl_data);

	return &ctl_data->hdata;
}

static struct rmi_control_handler handler = {
	.name = "driver",
	.dev_type = &rmi_device_type,
	.attach = driver_ctl_attach,
	.remove = driver_ctl_cleanup,
};

static int __init driver_ctl_init(void)
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

static void __exit driver_ctl_exit(void)
{
	pr_debug("%s: exiting.\n", __func__);

	rmi_unregister_control_handler(&handler);
}

module_init(driver_ctl_init);
module_exit(driver_ctl_exit);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com>");
MODULE_DESCRIPTION("RMI4 Driver Controls");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);