/*
 * Copyright (c) 2011-2013 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _RMI_BUS_H
#define _RMI_BUS_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/debugfs.h>
#include <linux/rmi.h>

struct rmi_device;

/**
 * struct rmi_function - represents the implementation of an RMI4
 * function for a particular device (basically, a driver for that RMI4 function)
 *
 * @fd: The function descriptor of the RMI function
 * @rmi_dev: Pointer to the RMI device associated with this function container
 * @dev: The device associated with this particular function.
 *
 * @num_of_irqs: The number of irqs needed by this function
 * @irq_pos: The position in the irq bitfield this function holds
 * @irq_mask: For convience, can be used to mask IRQ bits off during ATTN
 * interrupt handling.
 * @data: Private data pointer
 *
 * @node: entry in device's list of functions
 * @debugfs_root: used during debugging
 */
struct rmi_function {
	struct rmi_function_descriptor fd;
	struct rmi_device *rmi_dev;
	struct device dev;
	int num_of_irqs;
	int irq_pos;
	unsigned long *irq_mask;
	void *data;
	struct list_head node;

#ifdef CONFIG_RMI4_DEBUG
	struct dentry *debugfs_root;
#endif
};

#define to_rmi_function(d)	container_of(d, struct rmi_function, dev)

bool rmi_is_function_device(struct device *dev);

int __must_check rmi_register_function(struct rmi_function *);
void rmi_unregister_function(struct rmi_function *);

/**
 * struct rmi_function_handler - driver routines for a particular RMI function.
 *
 * @func: The RMI function number
 * @reset: Called when a reset of the touch sensor is detected.  The routine
 * should perform any out-of-the-ordinary reset handling that might be
 * necessary.  Restoring of touch sensor configuration registers should be
 * handled in the config() callback, below.
 * @config: Called when the function container is first initialized, and
 * after a reset is detected.  This routine should write any necessary
 * configuration settings to the device.
 * @attention: Called when the IRQ(s) for the function are set by the touch
 * sensor.
 * @suspend: Should perform any required operations to suspend the particular
 * function.
 * @resume: Should perform any required operations to resume the particular
 * function.
 *
 * All callbacks are expected to return 0 on success, error code on failure.
 */
struct rmi_function_handler {
	struct device_driver driver;

	u8 func;

	int (*probe)(struct rmi_function *fn);
	void (*remove)(struct rmi_function *fn);
	int (*config)(struct rmi_function *fn);
	int (*reset)(struct rmi_function *fn);
	int (*attention)(struct rmi_function *fn, unsigned long *irq_bits);
};

#define to_rmi_function_handler(d) \
		container_of(d, struct rmi_function_handler, driver)

int __must_check __rmi_register_function_handler(struct rmi_function_handler *,
						 struct module *, const char *);
#define rmi_register_function_handler(handler) \
	__rmi_register_function_handler(handler, THIS_MODULE, KBUILD_MODNAME)

void rmi_unregister_function_handler(struct rmi_function_handler *);

/**
 * struct rmi_driver - driver for an RMI4 sensor on the RMI bus.
 *
 * @driver: Device driver model driver
 * @irq_handler: Callback for handling irqs
 * @reset_handler: Called when a reset is detected.
 * @get_func_irq_mask: Callback for calculating interrupt mask
 * @store_irq_mask: Callback for storing and replacing interrupt mask
 * @restore_irq_mask: Callback for restoring previously stored interrupt mask
 * @store_productid: Callback for cache product id from function 01
 * @data: Private data pointer
 *
 */
struct rmi_driver {
	struct device_driver driver;

	int (*irq_handler)(struct rmi_device *rmi_dev, int irq);
	int (*reset_handler)(struct rmi_device *rmi_dev);
	int (*store_irq_mask)(struct rmi_device *rmi_dev,
				unsigned long *new_interupts);
	int (*restore_irq_mask)(struct rmi_device *rmi_dev);
	int (*store_productid)(struct rmi_device *rmi_dev);
	int (*set_input_params)(struct rmi_device *rmi_dev,
			struct input_dev *input);
	void *data;
};

#define to_rmi_driver(d) \
	container_of(d, struct rmi_driver, driver);

/**
 * struct rmi_transport_stats - diagnostic information about the RMI transport
 * device, used in the xport_info debugfs file.
 *
 * @proto String indicating the protocol being used.
 * @tx_count Number of transmit operations.
 * @tx_errs  Number of errors encountered during transmit operations.
 * @tx_bytes Number of bytes transmitted.
 * @rx_count Number of receive operations.
 * @rx_errs  Number of errors encountered during receive operations.
 * @rx_bytes Number of bytes received.
 */
struct rmi_transport_stats {
	unsigned long tx_count;
	unsigned long tx_errs;
	size_t tx_bytes;
	unsigned long rx_count;
	unsigned long rx_errs;
	size_t rx_bytes;
};

/**
 * struct rmi_transport_dev - represent an RMI transport device
 *
 * @dev: Pointer to the communication device, e.g. i2c or spi
 * @rmi_dev: Pointer to the RMI device
 * @irq_thread: if not NULL, the sensor driver will use this instead of the
 * default irq_thread implementation.
 * @hard_irq: if not NULL, the sensor driver will use this for the hard IRQ
 * handling
 * @proto_name: name of the transport protocol (SPI, i2c, etc)
 * @ops: pointer to transport operations implementation
 * @stats: transport statistics
 *
 * The RMI transport device implements the glue between different communication
 * buses such as I2C and SPI.
 *
 */
struct rmi_transport_dev {
	struct device *dev;
	struct rmi_device *rmi_dev;

	irqreturn_t (*irq_thread)(int irq, void *p);
	irqreturn_t (*hard_irq)(int irq, void *p);

	const char *proto_name;
	const struct rmi_transport_ops *ops;
	struct rmi_transport_stats stats;
};

/**
 * struct rmi_transport_ops - defines transport protocol operations.
 *
 * @write_block: Writing a block of data to the specified address
 * @read_block: Read a block of data from the specified address.
 */
struct rmi_transport_ops {
	int (*write_block)(struct rmi_transport_dev *xport, u16 addr,
			   const void *buf, size_t len);
	int (*read_block)(struct rmi_transport_dev *xport, u16 addr,
			  void *buf, size_t len);

	int (*enable_device) (struct rmi_transport_dev *xport);
	void (*disable_device) (struct rmi_transport_dev *xport);
};

/**
 * struct rmi_device - represents an RMI4 sensor device on the RMI bus.
 *
 * @dev: The device created for the RMI bus
 * @number: Unique number for the device on the bus.
 * @driver: Pointer to associated driver
 * @xport: Pointer to the transport interface
 * @debugfs_root: base for this particular sensor device.
 *
 */
struct rmi_device {
	struct device dev;
	int number;

	struct rmi_driver *driver;
	struct rmi_transport_dev *xport;

#ifdef CONFIG_RMI4_DEBUG
	struct dentry *debugfs_root;
#endif
};

#define to_rmi_device(d) container_of(d, struct rmi_device, dev)
#define to_rmi_platform_data(d) ((d)->xport->dev->platform_data)

bool rmi_is_physical_device(struct device *dev);

/**
 * rmi_read - read a single byte
 * @d: Pointer to an RMI device
 * @addr: The address to read from
 * @buf: The read buffer
 *
 * Reads a byte of data using the underlaying transport protocol in to buf. It
 * returns zero or a negative error code.
 */
static inline int rmi_read(struct rmi_device *d, u16 addr, void *buf)
{
	return d->xport->ops->read_block(d->xport, addr, buf, 1);
}

/**
 * rmi_read_block - read a block of bytes
 * @d: Pointer to an RMI device
 * @addr: The start address to read from
 * @buf: The read buffer
 * @len: Length of the read buffer
 *
 * Reads a block of byte data using the underlaying transport protocol in
 * to buf.  It returns the amount of bytes read or a negative error code.
 */
static inline int rmi_read_block(struct rmi_device *d, u16 addr, void *buf,
				 const int len)
{
	return d->xport->ops->read_block(d->xport, addr, buf, len);
}

/**
 * rmi_write - write a single byte
 * @d: Pointer to an RMI device
 * @addr: The address to write to
 * @data: The data to write
 *
 * Writes a byte from buf using the underlaying transport protocol. It
 * returns zero or a negative error code.
 */
static inline int rmi_write(struct rmi_device *d, u16 addr, const u8 data)
{
	return d->xport->ops->write_block(d->xport, addr, &data, 1);
}

/**
 * rmi_write_block - write a block of bytes
 * @d: Pointer to an RMI device
 * @addr: The start address to write to
 * @buf: The write buffer
 * @len: Length of the write buffer
 *
 * Writes a block of byte data from buf using the underlaying transport
 * protocol.  It returns the amount of bytes written or a negative error code.
 */
static inline int rmi_write_block(struct rmi_device *d, u16 addr,
				  const void *buf, const int len)
{
	return d->xport->ops->write_block(d->xport, addr, buf, len);
}

int rmi_register_transport_device(struct rmi_transport_dev *xport);
void rmi_unregister_transport_device(struct rmi_transport_dev *xport);
int rmi_for_each_dev(void *data, int (*func)(struct device *dev, void *data));

/**
 * module_rmi_driver() - Helper macro for registering a function driver
 * @__rmi_driver: rmi_function_handler struct
 *
 * Helper macro for RMI4 function drivers which do not do anything special
 * in module init/exit. This eliminates a lot of boilerplate. Each module
 * may only use this macro once, and calling it replaces module_init()
 * and module_exit().
 */
#define module_rmi_driver(__rmi_driver)			\
	module_driver(__rmi_driver,			\
		      rmi_register_function_handler,	\
		      rmi_unregister_function_handler)


extern struct bus_type rmi_bus_type;

#endif
