/*
 * Copyright (c) 2011-2013 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/kconfig.h>
#include <linux/list.h>
#include <linux/pm.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/debugfs.h>

#include "rmi_bus.h"
#include "rmi_control.h"
#include "rmi_driver.h"

DEFINE_MUTEX(rmi_bus_mutex);

#ifdef CONFIG_RMI4_DEBUG
static struct dentry *rmi_bus_debugfs_root;
#endif


/*
 * *RMI Physical devices
 *
 * Physical RMI device consists of several functions serving particular
 * purpose. For example F11 is a 2D touch sensor while F10 is a generic
 * function present in every RMI device.
 */
static void rmi_release_device(struct device *dev)
{
}

struct device_type rmi_device_type = {
	.name = "rmi_sensor",
	.release = rmi_release_device,
};
EXPORT_SYMBOL_GPL(rmi_device_type);

#if CONFIG_RMI4_DEBUG

static void rmi_physical_setup_debugfs(struct rmi_device *rmi_dev)
{
	rmi_dev->debugfs_root = debugfs_create_dir(dev_name(&rmi_dev->dev),
						   rmi_bus_debugfs_root);
	if (!rmi_dev->debugfs_root)
		dev_warn(&rmi_dev->dev, "Failed to create sensor debugfs root.\n");
}

static void rmi_physical_teardown_debugfs(struct rmi_device *rmi_dev)
{
	if (rmi_dev->debugfs_root)
		debugfs_remove_recursive(rmi_dev->debugfs_root);
}

#else

static void rmi_physical_setup_debugfs(struct rmi_device *rmi_dev)
{
}

static void rmi_physical_teardown_debugfs(struct rmi_device *rmi_dev)
{
}

#endif

/**
 * rmi_register_transport_device - register a transport connection on the RMI
 * bus.  Transport drivers provide communication with an RMI4 devices residing
 * on a bus such as SPI, I2C, and so on.
 *
 * @transport: the device to register
 */
int rmi_register_transport_device(struct rmi_transport_device *xport)
{
	static atomic_t transport_dev_count = ATOMIC_INIT(0);
	struct rmi_device_platform_data *pdata = xport->dev->platform_data;
	struct rmi_device *rmi_dev;
	int error;

	if (!pdata) {
		dev_err(xport->dev, "no platform data!\n");
		return -EINVAL;
	}

	rmi_dev = devm_kzalloc(xport->dev,
				sizeof(struct rmi_device), GFP_KERNEL);
	if (!rmi_dev)
		return -ENOMEM;

	rmi_dev->xport = xport;
	rmi_dev->number = atomic_inc_return(&transport_dev_count) - 1;

	dev_set_name(&rmi_dev->dev, "sensor%02d", rmi_dev->number);

	rmi_dev->dev.bus = &rmi_bus_type;
	rmi_dev->dev.type = &rmi_device_type;

	// FIXME: This assignment breaks the driver.
// 	rmi_dev->dev.driver = &rmi_physical_driver.driver;

	xport->rmi_dev = rmi_dev;

	error = device_register(&rmi_dev->dev);
	if (error)
		return error;

	dev_dbg(xport->dev, "%s: Registered %s as %s.\n", __func__,
		pdata->sensor_name, dev_name(&rmi_dev->dev));

	rmi_physical_setup_debugfs(rmi_dev);

	return 0;
}
EXPORT_SYMBOL_GPL(rmi_register_transport_device);

/**
 * rmi_unregister_transport_device - unregister a transport connection
 * @xport: the connection to unregister
 *
 */
void rmi_unregister_transport_device(struct rmi_transport_device *xport)
{
	struct rmi_device *rmi_dev = xport->rmi_dev;

	rmi_physical_teardown_debugfs(rmi_dev);

	device_unregister(&rmi_dev->dev);
}
EXPORT_SYMBOL_GPL(rmi_unregister_transport_device);


static bool rmi_is_physical_driver(struct device_driver *drv)
{
	return drv == &rmi_physical_driver.driver;
}

static int rmi_physical_remove(struct device *dev)
{
	struct rmi_driver *driver;
	struct rmi_device *rmi_dev = to_rmi_device(dev);

	driver = to_rmi_driver(dev->driver);

	if (driver->remove)
		return driver->remove(rmi_dev);
	return 0;
}

/* Function specific stuff */

static void rmi_release_function_dev(struct device *dev)
{
	struct rmi_function *fn = to_rmi_function(dev);
	kfree(fn);
}

struct device_type rmi_function_type = {
	.name    = "rmi_function",
	.release = rmi_release_function_dev,
};
EXPORT_SYMBOL_GPL(rmi_function_type);

#if CONFIG_RMI4_DEBUG

static void rmi_function_setup_debugfs(struct rmi_function *fn)
{
	char dirname[12];

	snprintf(dirname, sizeof(dirname), "F%02X", fn->fd.function_number);
	fn->debugfs_root = debugfs_create_dir(dirname,
						fn->rmi_dev->debugfs_root);
	if (!fn->debugfs_root)
		dev_warn(&fn->dev, "Failed to create debugfs dir.\n");
}

static void rmi_function_teardown_debugfs(struct rmi_function *fn)
{
	if (fn->debugfs_root)
		debugfs_remove_recursive(fn->debugfs_root);
}

#else

static void rmi_function_setup_debugfs(struct rmi_function *fn)
{
}

static void rmi_function_teardown_debugfs(struct rmi_function *fn)
{
}

#endif

static int rmi_function_match(struct device *dev, struct device_driver *drv)
{
	struct rmi_function_driver *fn_drv = to_rmi_function_driver(drv);
	struct rmi_function *fn = to_rmi_function(dev);

	return fn->fd.function_number == fn_drv->func;
}

static int rmi_function_probe(struct device *dev)
{
	struct rmi_function_driver *fn_drv;
	struct rmi_function *fn = to_rmi_function(dev);

	dev_dbg(dev, "%s called.\n", __func__);

	fn_drv = to_rmi_function_driver(dev->driver);

	if (fn_drv->probe)
		return fn_drv->probe(fn);

	return 0;
}

static int rmi_function_remove(struct device *dev)
{
	struct rmi_function_driver *fn_drv;
	struct rmi_function *fn = to_rmi_function(dev);

	fn_drv = to_rmi_function_driver(dev->driver);

	if (fn_drv->remove)
		return fn_drv->remove(fn);

	return 0;
}

int rmi_register_function_dev(struct rmi_function *fn)
{
	struct rmi_device *rmi_dev = fn->rmi_dev;
	int error;

	dev_set_name(&fn->dev, "%s.fn%02x", dev_name(&rmi_dev->dev),
		     fn->fd.function_number);

	fn->dev.parent = &rmi_dev->dev;
	fn->dev.type = &rmi_function_type;
	fn->dev.bus = &rmi_bus_type;

	error = device_register(&fn->dev);
	if (error) {
		dev_err(&rmi_dev->dev, "Failed device register function device %s.\n",
			dev_name(&fn->dev));
		return error;
	}

	dev_dbg(&rmi_dev->dev, "Registered F%02X.\n",
		fn->fd.function_number);

	rmi_function_setup_debugfs(fn);
	return 0;
}

void rmi_unregister_function_dev(struct rmi_function *fn)
{
	rmi_function_teardown_debugfs(fn);
	device_unregister(&fn->dev);
}

/**
 * rmi_register_function_driver - register a driver for an RMI function
 * @fn_drv: RMI driver that should be registered.
 * @module: pointer to module that implements the driver
 * @mod_name: name of the module implementing the driver
 *
 * This function performs additional setup of RMI function driver and
 * registers it with the RMI core so that it can be bound to
 * RMI function devices.
 */
int __rmi_register_function_driver(struct rmi_function_driver *fn_drv,
				   struct module *owner,
				   const char *mod_name)
{
	struct device_driver *driver = &fn_drv->driver;
	int error;

	driver->bus = &rmi_bus_type;
	driver->owner = owner;
	driver->mod_name = mod_name;
	driver->probe = rmi_function_probe;
	driver->remove = rmi_function_remove;

	error = driver_register(&fn_drv->driver);
	if (error) {
		pr_err("driver_register() failed for %s, error: %d\n",
		       fn_drv->driver.name, error);
		return error;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(__rmi_register_function_driver);

/**
 * rmi_unregister_function_driver - unregister given RMI function driver
 * @fn_drv: RMI driver that should be unregistered.
 *
 * This function unregisters given function driver from RMI core which
 * causes it to be unbound from the function devices.
 */
void rmi_unregister_function_driver(struct rmi_function_driver *fn_drv)
{
	driver_unregister(&fn_drv->driver);
}
EXPORT_SYMBOL_GPL(rmi_unregister_function_driver);

/* Bus specific stuff */

static int rmi_bus_match(struct device *dev, struct device_driver *drv)
{
	bool sensor = rmi_is_physical_device(dev);

	/* First see if types are not compatible.
	 */
	if (sensor != rmi_is_physical_driver(drv))
		return 0;

	return sensor || rmi_function_match(dev, drv);
}

static int rmi_bus_remove(struct device *dev)
{
	if (rmi_is_function_device(dev))
		return rmi_function_remove(dev);
	else if (rmi_is_physical_device(dev))
		return rmi_physical_remove(dev);
	return -EINVAL;
}

#ifdef CONFIG_PM
static int rmi_bus_suspend(struct device *dev)
{
	struct device_driver *driver = dev->driver;
	const struct dev_pm_ops *pm;

	if (!driver)
		return 0;

	pm = driver->pm;
	if (pm && pm->suspend)
		return pm->suspend(dev);
	if (driver->suspend)
		return driver->suspend(dev, PMSG_SUSPEND);

	return 0;
}

static int rmi_bus_resume(struct device *dev)
{
	struct device_driver *driver = dev->driver;
	const struct dev_pm_ops *pm;

	if (!driver)
		return 0;

	pm = driver->pm;
	if (pm && pm->resume)
		return pm->resume(dev);
	if (driver->resume)
		return driver->resume(dev);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(rmi_bus_pm_ops,
			 rmi_bus_suspend, rmi_bus_resume);

struct bus_type rmi_bus_type = {
	.name		= "rmi",
	.match		= rmi_bus_match,
	.remove		= rmi_bus_remove,
	.pm		= &rmi_bus_pm_ops,
};
EXPORT_SYMBOL_GPL(rmi_bus_type);

/**
 * rmi_for_each_dev - provides a way for other parts of the system to enumerate
 * the devices on the RMI bus.
 *
 * @data - will be passed into the callback function.
 * @func - will be called for each device.
 */
int rmi_for_each_dev(void *data, int (*func)(struct device *dev, void *data))
{
	int retval;
	mutex_lock(&rmi_bus_mutex);
	retval = bus_for_each_dev(&rmi_bus_type, NULL, data, func);
	mutex_unlock(&rmi_bus_mutex);
	return retval;
}
EXPORT_SYMBOL_GPL(rmi_for_each_dev);

#ifdef CONFIG_RMI4_DEBUG
static void rmi_bus_setup_debugfs(void)
{
	rmi_bus_debugfs_root = debugfs_create_dir(rmi_bus_type.name, NULL);
	if (!rmi_bus_debugfs_root)
		pr_err("%s: Failed to create bus debugfs root.\n",
		       __func__);
}

static void rmi_bus_teardown_debugfs(void)
{
	if (rmi_bus_debugfs_root)
		debugfs_remove_recursive(rmi_bus_debugfs_root);
}
#else
static void rmi_bus_setup_debugfs(void)
{
}

static void rmi_bus_teardown_debugfs(void)
{
}
#endif

static int __init rmi_bus_init(void)
{
	int error;

	mutex_init(&rmi_bus_mutex);

	error = bus_register(&rmi_bus_type);
	if (error) {
		pr_err("%s: error registering the RMI bus: %d\n",
			__func__, error);
		return error;
	}

	rmi_bus_setup_debugfs();

	error = rmi_register_function_driver(&rmi_f01_driver);
	if (error) {
		pr_err("%s: error registering the RMI F01 driver: %d\n",
			__func__, error);
		goto err_unregister_bus;
	}

	error = rmi_register_sensor_driver();
	if (error) {
		pr_err("%s: error registering the RMI sensor driver: %d\n",
			__func__, error);
		goto err_unregister_f01;
	}

	return 0;

err_unregister_f01:
	rmi_unregister_function_driver(&rmi_f01_driver);
err_unregister_bus:
	bus_unregister(&rmi_bus_type);
	return error;
}

static void __exit rmi_bus_exit(void)
{
	/*
	 * We should only ever get here if all drivers are unloaded, so
	 * all we have to do at this point is unregister ourselves.
	 */
	rmi_bus_teardown_debugfs();
	rmi_unregister_sensor_driver();
	rmi_unregister_function_driver(&rmi_f01_driver);
	bus_unregister(&rmi_bus_type);
}

module_init(rmi_bus_init);
module_exit(rmi_bus_exit);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com");
MODULE_DESCRIPTION("RMI bus");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
