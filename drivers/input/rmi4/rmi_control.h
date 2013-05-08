/*
 * Copyright (c) 2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef RMI_CONTROL_H
#define RMI_CONTROL_H

#include <linux/device.h>
#include <linux/list.h>

struct rmi_control_handler_data {
	struct device *dev;
	struct list_head list;
};

/** Information relating to control/debug handling implementations.
 *
 * @name - useful for diagnostics
 * @dev_type - the type of device the handler is interested in.
 * @function_id - the RMI4 function ID it is interested in (ignored if 0 or
 * dev_type == rmi_device_type);
 * @attach - called if a device appears on the bus that matches the parameters
 * of this handler.
 * @remove - called when the device disappears from the bus.
 *
 * @notifier - used by the control/debug system to accept notifications for
 * this handler.
 * @list - used by the control/debug system to keep track of handlers.
 */
struct rmi_control_handler {
	char name[32];
	struct device_type *dev_type;
	u8 function_id;

	struct rmi_control_handler_data * (*attach) (struct device *dev, void *data);
	int (*remove) (struct rmi_control_handler_data *hdata);

	struct notifier_block notifier;
	struct list_head list;
};


int rmi_register_control_handler(struct rmi_control_handler *handler);
void rmi_unregister_control_handler(struct rmi_control_handler *handler);

#endif