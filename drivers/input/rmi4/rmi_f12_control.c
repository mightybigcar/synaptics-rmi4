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
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>

#include "rmi_control.h"
#include "rmi_driver.h"
#include "rmi_f12.h"

struct f12_ctl_data {
	struct rmi_function *f12_dev;
	struct rmi_control_handler_data handler_data;
	struct f12_data *f12;
};

#ifdef CONFIG_RMI4_DEBUG

static int rmi_f12_setup_debugfs(struct f12_ctl_data *ctl_data)
{
	char fname[NAME_BUFFER_SIZE];
	struct rmi_function *fn = ctl_data->f12_dev;
	struct dentry *f12_root;

	if (!fn->debugfs_root)
		return -ENODEV;

	snprintf(fname, NAME_BUFFER_SIZE, "input0");
	f12_root = debugfs_create_dir(fname, fn->debugfs_root);
	if (!f12_root) {
		dev_warn(&fn->dev,
			 "Failed to create debugfs directory %s\n",
			 fname);
		return -ENOMEM;
	}

	return 0;
}

struct f12_debugfs_data {
	loff_t pos;
	struct rmi_function *fn;
};

static int f12_open(struct inode *inodep, struct file *filp)
{
	struct f12_debugfs_data *data;
	struct rmi_function *fn;

	fn = inodep->i_private;
	data = kzalloc(sizeof(struct f12_debugfs_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->fn = inodep->i_private;
	filp->private_data = data;
	return 0;
}

static int f12_release(struct inode *inodep, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}

static loff_t debug_seek(struct file *filp, loff_t offset, int whence)
{
	struct f12_debugfs_data *data = filp->private_data;
	int new_pos;

	switch (whence) {
	case SEEK_SET:
		new_pos = offset;
		break;
	case SEEK_CUR:
		new_pos = data->pos + offset;
		break;
	default:
		dev_err(&data->fn->dev, "Invalid whence of %d.\n", whence);
		return -EINVAL;
	}

	if (new_pos < 0) {
		dev_err(&data->fn->dev, "Invalid position %d.\n", new_pos);
		return -EINVAL;
	}

	data->pos = new_pos;
	return data->pos;
}

static const struct file_operations report_count_fops = {
	.owner = THIS_MODULE,
	.open = f12_open,
	.release = f12_release,
	.llseek = debug_seek,
};

#else
#define rmi_f12_setup_debugfs(d) 0
#endif
/* End adding debugfs */

static ssize_t f12_suppress_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
        struct rmi_function *fn = NULL;
        struct f12_data *f12;
        unsigned int suppress;

        fn = to_rmi_function(dev);
        if (fn == NULL)
                return -ENODEV;

        f12 = fn->data;
        if (f12 == NULL)
                return -ENODEV;

        if (sscanf(buf, "%u", &suppress) != 1)
                return -EINVAL;
        if (suppress > 1)
                return -EINVAL;

        f12->suppress = suppress;

        return count;
}

static ssize_t f12_suppress_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
        struct rmi_function *fn;
        struct f12_data *f12;

        fn = to_rmi_function(dev);
        if (fn == NULL)
                return -ENODEV;

        f12 = fn->data;
        if (f12 == NULL)
                return -ENODEV;

        return snprintf(buf, PAGE_SIZE, "%u\n", f12->suppress);
}

static ssize_t f12_suppress_highw_store(struct device *dev,
                                                        struct device_attribute *attr,
                                                        const char *buf, size_t count)
{
        struct rmi_function *fn = NULL;
        struct f12_data *f12;
        unsigned int suppress_highw;

        fn = to_rmi_function(dev);
        if (fn == NULL)
                return -ENODEV;

        f12 = fn->data;
        if (f12 == NULL)
                return -ENODEV;

        if (sscanf(buf, "%u", &suppress_highw) != 1)
                return -EINVAL;
        if (suppress_highw > 15)
                return -EINVAL;

        f12->suppress_highw = suppress_highw;

        return count;
}

static ssize_t f12_suppress_highw_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
        struct rmi_function *fn;
        struct f12_data *f12;

        fn = to_rmi_function(dev);
        if (fn == NULL)
                return -ENODEV;

        f12 = fn->data;
        if (f12 == NULL)
                return -ENODEV;

        return snprintf(buf, PAGE_SIZE, "%u\n", f12->suppress_highw);
}

static struct device_attribute dev_attr_suppress =
        __ATTR(suppress, RMI_RW_ATTR, f12_suppress_show, f12_suppress_store);
static struct device_attribute dev_attr_suppress_highw =
        __ATTR(suppress_highw, RMI_RW_ATTR, f12_suppress_highw_show,
                f12_suppress_highw_store);

static struct attribute *attrs[] = {
	&dev_attr_suppress.attr,
	&dev_attr_suppress_highw.attr,
	NULL,
};
static struct attribute_group fn12_attrs = GROUP(attrs);

static int f12_ctl_cleanup(struct rmi_control_handler_data *hdata)
{
	struct f12_ctl_data *ctl_data;
	struct device *dev;

	ctl_data = container_of(hdata, struct f12_ctl_data, handler_data);

	if (ctl_data->f12_dev->debugfs_root) {
		debugfs_remove_recursive(ctl_data->f12_dev->debugfs_root);
		ctl_data->f12_dev->debugfs_root = NULL;
	}
	sysfs_remove_group(&ctl_data->f12_dev->dev.kobj, &fn12_attrs);

	dev = hdata->dev;
	devm_kfree(dev, ctl_data);

	return 0;
}

static struct rmi_control_handler_data *f12_ctl_attach(struct device *dev, void *data)
{
	struct rmi_function *fn;
	struct f12_data *f12_data;
	struct f12_ctl_data *ctl_data;

	fn = to_rmi_function(dev);
	dev_dbg(dev, "%s called.\n", __func__);

	ctl_data = devm_kzalloc(dev, sizeof(struct f12_ctl_data), GFP_KERNEL);
	if (!ctl_data)
		return NULL;
	ctl_data->f12_dev = fn;

	f12_data = fn->data;
	ctl_data->f12 = f12_data;

	rmi_f12_setup_debugfs(ctl_data);

	if (sysfs_create_group(&fn->dev.kobj, &fn12_attrs) < 0) {
		dev_warn(&fn->dev, "Failed to create query sysfs files.");
	}

	return &ctl_data->handler_data;
}

static struct rmi_control_handler handler = {
	.name = "f12",
	.dev_type = &rmi_function_type,
	.function_id = 0x12,
	.attach = f12_ctl_attach,
	.remove = f12_ctl_cleanup,
};

static int __init f12_ctl_init(void)
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

static void __exit f12_ctl_exit(void)
{
	pr_debug("%s: exiting.\n", __func__);

	rmi_unregister_control_handler(&handler);
}

module_init(f12_ctl_init);
module_exit(f12_ctl_exit);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com>");
MODULE_DESCRIPTION("RMI4 F12 Controls");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
