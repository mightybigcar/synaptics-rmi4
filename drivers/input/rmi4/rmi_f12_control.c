/*
 * Copyright (c) 2014 Synaptics Incorporated
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

struct rmi_f12_ctl_data {
	struct rmi_function *f12_dev;
	struct rmi_control_handler_data handler_data;
};

#ifdef CONFIG_RMI4_DEBUG

static int rmi_f12_setup_debugfs(struct rmi_f12_ctl_data *ctl_data)
{
	struct rmi_function *fn = ctl_data->f12_dev;
	struct f12_data *f12 = fn->data;
	struct dentry *entry;

	if (!fn->debugfs_root)
		return -ENODEV;

	entry = debugfs_create_u16("x_max",
				   S_IRUGO, fn->debugfs_root, &f12->x_max);
	if (!entry)
		dev_warn(&fn->dev, "Failed to create debugfs x_max.\n");

	entry = debugfs_create_u16("y_max",
				   S_IRUGO, fn->debugfs_root, &f12->y_max);
	if (!entry)
		dev_warn(&fn->dev, "Failed to create debugfs y_max.\n");

	return 0;
}
#endif

static int f12_ctl_cleanup(struct rmi_control_handler_data *hdata)
{
	struct rmi_f12_ctl_data *ctl_data;

	dev_dbg(hdata->dev, "%s called.\n", __func__);

	ctl_data = container_of(hdata, struct rmi_f12_ctl_data, handler_data);

	debugfs_remove_recursive(ctl_data->f12_dev->debugfs_root);
// 	sysfs_remove_group(&ctl_data->f12_dev->dev.kobj, &fn12_attrs);
//

	devm_kfree(hdata->dev, ctl_data);

	return 0;
}

static struct rmi_control_handler_data *f12_ctl_attach(struct device *dev, void *data)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct rmi_f12_ctl_data *ctl_data;

	dev_dbg(dev, "%s called.\n", __func__);

	ctl_data = devm_kzalloc(dev, sizeof(struct rmi_f12_ctl_data), GFP_KERNEL);
	if (!ctl_data)
		return NULL;
	ctl_data->f12_dev = fn;

// 	if (sysfs_create_group(&fn->dev.kobj, &fn12_attrs) < 0) {
// 		dev_warn(&fn->dev, "Failed to create query sysfs files.");
// 	}
	rmi_f12_setup_debugfs(ctl_data);

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
