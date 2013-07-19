/*
 * Copyright (c) 2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/device.h>

#include "rmi_driver.h"
#include "rmi_control.h"


static struct rmi_control_handler_data *get_handler_data(struct device *dev,
				struct rmi_control_handler *handler)
{
	struct rmi_control_handler_data *hdata = NULL;

	list_for_each_entry(hdata, &handler->list, list) {
		if (hdata->dev == dev)
			return hdata;
	}

	return NULL;
}

static int rmi_ctl_cleanup(struct device *dev, void *data)
{
	struct rmi_control_handler *handler = (struct rmi_control_handler *) data;
	struct rmi_control_handler_data *hdata;

	if (dev->type != handler->dev_type)
		return 0;

	hdata = get_handler_data(dev, handler);
	if (hdata) {
		list_del(&hdata->list);
		handler->remove(hdata);
	}
	return 0;
}

static int rmi_ctl_attach(struct device *dev, void *data)
{
	struct rmi_control_handler *handler = (struct rmi_control_handler *) data;
	struct rmi_control_handler_data *hdata;

	if (dev->type != handler->dev_type)
		return 0;

	if (dev->type == &rmi_function_type) {
		struct rmi_function *fn = to_rmi_function(dev);
		if (fn->fd.function_number != handler->function_id)
			return 0;
	}

	dev_dbg(dev, "%s: control match with %s.\n", __func__, handler->name);

	hdata = handler->attach(dev, handler);
	if (!hdata) {
		hdata->dev = dev;
		INIT_LIST_HEAD(&hdata->list);
		list_add(&hdata->list, &handler->list);
	}

	return 0;
}

static int rmi_ctl_notifier_call(struct notifier_block *nb,
				unsigned long action, void *data)
{
	struct device *dev = data;
	struct rmi_control_handler *handler = container_of(nb, struct rmi_control_handler, notifier);

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		dev_dbg(dev, "%s: added device.\n", __func__);
		break;
	case BUS_NOTIFY_DEL_DEVICE:
		dev_dbg(dev, "%s: removed device.\n", __func__);
		break;
	case BUS_NOTIFY_BIND_DRIVER:
		dev_dbg(dev, "%s: driver arriving.\n", __func__);
		break;
	case BUS_NOTIFY_BOUND_DRIVER:
		dev_dbg(dev, "%s: driver bound.\n", __func__);
		return rmi_ctl_attach(dev, handler);
	case BUS_NOTIFY_UNBIND_DRIVER:
		dev_dbg(dev, "%s: driver going away.\n", __func__);
		return rmi_ctl_cleanup(dev, handler);
	case BUS_NOTIFY_UNBOUND_DRIVER:
		dev_dbg(dev, "%s: driver went away.\n", __func__);
		break;
	default:
		dev_dbg(dev, "%s: unknown action %lu.\n", __func__, action);
		break;
	}
	return 0;
}

/** Called to register a handler with the control/debug system.
 */
int rmi_register_control_handler(struct rmi_control_handler *handler) {
	int error;

	pr_debug("%s: registering handler for %s.\n", __func__, handler->name);

	handler->notifier.notifier_call = rmi_ctl_notifier_call;
	INIT_LIST_HEAD(&handler->list);

	/* Ask the bus to let us know when new devices appear. */
	error = bus_register_notifier(&rmi_bus_type, &handler->notifier);
	if (error) {
		pr_err("%s: failed to register bus notifier, code=%d.\n",
		       __func__, error);
		return error;
	}

	/* Bind to any previously existing sensors. */
	rmi_for_each_dev(handler, rmi_ctl_attach);
	return 0;
}
EXPORT_SYMBOL(rmi_register_control_handler);

/** Called to register a handler with the control/debug system.
 */
void rmi_unregister_control_handler(struct rmi_control_handler *handler) {
	pr_debug("%s: unregistering handler %s.\n", __func__, handler->name);
	pr_warn("WARNING %s is not implemented yet.\n", __func__);
}
EXPORT_SYMBOL(rmi_unregister_control_handler);
