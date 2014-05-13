/*
 * Copyright (c) 2011-2013 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include "rmi_driver.h"

#define BUFFER_SIZE_INCREMENT 32

/**
 * struct rmi_i2c_xport - stores information for i2c communication
 *
 * @xport: The transport interface structure
 *
 * @page_mutex: Locks current page to avoid changing pages in unexpected ways.
 * @page: Keeps track of the current virtual page
 *
 * @tx_buf: Buffer used for transmitting data to the sensor over i2c.
 * @tx_buf_size: Size of the buffer
 */
struct rmi_i2c_xport {
	struct rmi_transport_dev xport;
	struct i2c_client *client;

	struct mutex page_mutex;
	int page;

	u8 *tx_buf;
	size_t tx_buf_size;
};

#define RMI_PAGE_SELECT_REGISTER 0xff
#define RMI_I2C_PAGE(addr) (((addr) >> 8) & 0xff)

/*
 * rmi_set_page - Set RMI page
 * @xport: The pointer to the rmi_transport_dev struct
 * @page: The new page address.
 *
 * RMI devices have 16-bit addressing, but some of the transport
 * implementations (like SMBus) only have 8-bit addressing. So RMI implements
 * a page address at 0xff of every page so we can reliable page addresses
 * every 256 registers.
 *
 * The page_mutex lock must be held when this function is entered.
 *
 * Returns zero on success, non-zero on failure.
 */
static int rmi_set_page(struct rmi_i2c_xport *rmi_i2c, u8 page)
{
	struct i2c_client *client = rmi_i2c->client;
	u8 txbuf[2] = {RMI_PAGE_SELECT_REGISTER, page};
	int retval;

	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval != sizeof(txbuf)) {
		dev_err(&client->dev,
			"%s: set page failed: %d.", __func__, retval);
		return (retval < 0) ? retval : -EIO;
	}

	rmi_i2c->page = page;
	return 0;
}

static int rmi_i2c_write_block(struct rmi_transport_dev *xport, u16 addr,
			       const void *buf, size_t len)
{
	struct rmi_i2c_xport *rmi_i2c =
		container_of(xport, struct rmi_i2c_xport, xport);
	struct i2c_client *client = rmi_i2c->client;
	size_t tx_size = len + 1;
	int retval;

	mutex_lock(&rmi_i2c->page_mutex);

	if (!rmi_i2c->tx_buf || rmi_i2c->tx_buf_size < tx_size) {
		if (rmi_i2c->tx_buf)
			devm_kfree(&client->dev, rmi_i2c->tx_buf);
		rmi_i2c->tx_buf_size = tx_size + BUFFER_SIZE_INCREMENT;
		rmi_i2c->tx_buf = devm_kzalloc(&client->dev,
					       rmi_i2c->tx_buf_size,
					       GFP_KERNEL);
		if (!rmi_i2c->tx_buf) {
			rmi_i2c->tx_buf_size = 0;
			retval = -ENOMEM;
			goto exit;
		}
	}

	rmi_i2c->tx_buf[0] = addr & 0xff;
	memcpy(rmi_i2c->tx_buf + 1, buf, len);

	if (RMI_I2C_PAGE(addr) != rmi_i2c->page) {
		retval = rmi_set_page(rmi_i2c, RMI_I2C_PAGE(addr));
		if (retval)
			goto exit;
	}

	retval = i2c_master_send(client, rmi_i2c->tx_buf, tx_size);
	if (retval == tx_size)
		retval = 0;
	else if (retval >= 0)
		retval = -EIO;

exit:
	dev_dbg(&client->dev,
		"write %zd bytes at %#06x: %d (%*ph)\n",
		len, addr, retval, (int)len, buf);

	xport->stats.tx_count++;
	if (retval)
		xport->stats.tx_errs++;
	else
		xport->stats.tx_bytes += len;

	mutex_unlock(&rmi_i2c->page_mutex);
	return retval;
}

static int rmi_i2c_read_block(struct rmi_transport_dev *xport, u16 addr,
			      void *buf, size_t len)
{
	struct rmi_i2c_xport *rmi_i2c =
		container_of(xport, struct rmi_i2c_xport, xport);
	struct i2c_client *client = rmi_i2c->client;
	u8 addr_offset = addr & 0xff;
	int retval;
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.len	= sizeof(addr_offset),
			.buf	= &addr_offset,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		},
	};

	mutex_lock(&rmi_i2c->page_mutex);

	if (RMI_I2C_PAGE(addr) != rmi_i2c->page) {
		retval = rmi_set_page(rmi_i2c, RMI_I2C_PAGE(addr));
		if (retval)
			goto exit;
	}

	retval = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (retval == ARRAY_SIZE(msgs))
		retval = 0; /* success */
	else if (retval >= 0)
		retval = -EIO;

exit:
	dev_dbg(&client->dev,
		"read %zd bytes at %#06x: %d (%*ph)\n",
		len, addr, retval, (int)len, buf);

	xport->stats.rx_count++;
	if (retval < 0)
		xport->stats.rx_errs++;
	else
		xport->stats.rx_bytes += len;

	mutex_unlock(&rmi_i2c->page_mutex);
	return retval;
}

static const struct rmi_transport_ops rmi_i2c_ops = {
	.write_block	= rmi_i2c_write_block,
	.read_block	= rmi_i2c_read_block,
};

static int rmi_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	const struct rmi_device_platform_data *pdata =
				dev_get_platdata(&client->dev);
	struct rmi_i2c_xport *rmi_i2c;
	int retval;

	if (!pdata) {
		dev_err(&client->dev, "no platform data\n");
		return -EINVAL;
	}

	dev_dbg(&client->dev, "Probing %s at %#02x (GPIO %d).\n",
		pdata->sensor_name ? pdata->sensor_name : "-no name-",
		client->addr, pdata->attn_gpio);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"adapter does not support required functionality.\n");
		return -ENODEV;
	}

	if (pdata->gpio_config) {
		retval = pdata->gpio_config(pdata->gpio_data, true);
		if (retval < 0) {
			dev_err(&client->dev, "Failed to configure GPIOs, code: %d.\n",
				retval);
			return retval;
		}
	} else {
		dev_warn(&client->dev, "No gpio_config().\n");
	}

	rmi_i2c = devm_kzalloc(&client->dev, sizeof(struct rmi_i2c_xport),
				GFP_KERNEL);
	if (!rmi_i2c)
		return -ENOMEM;

	rmi_i2c->client = client;
	mutex_init(&rmi_i2c->page_mutex);

	rmi_i2c->xport.dev = &client->dev;
	rmi_i2c->xport.proto_name = "i2c";
	rmi_i2c->xport.ops = &rmi_i2c_ops;

	/*
	 * Setting the page to zero will (a) make sure the PSR is in a
	 * known state, and (b) make sure we can talk to the device.
	 */
	retval = rmi_set_page(rmi_i2c, 0);
	if (retval) {
		dev_err(&client->dev, "Failed to set page select to 0.\n");
		return retval;
	}

	retval = rmi_register_transport_device(&rmi_i2c->xport);
	if (retval) {
		dev_err(&client->dev, "Failed to register transport driver at 0x%.2X.\n",
			client->addr);
		goto err_gpio;
	}

	i2c_set_clientdata(client, rmi_i2c);

	dev_info(&client->dev, "registered rmi i2c driver at %#04x.\n",
			client->addr);
	return 0;

err_gpio:
	if (pdata->gpio_config)
		pdata->gpio_config(pdata->gpio_data, false);

	return retval;
}

static int rmi_i2c_remove(struct i2c_client *client)
{
	const struct rmi_device_platform_data *pdata =
				dev_get_platdata(&client->dev);
	struct rmi_i2c_xport *rmi_i2c = i2c_get_clientdata(client);

	rmi_unregister_transport_device(&rmi_i2c->xport);

	if (pdata->gpio_config)
		pdata->gpio_config(pdata->gpio_data, false);

	return 0;
}

static const struct i2c_device_id rmi_id[] = {
	{ "rmi_i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rmi_id);

static struct i2c_driver rmi_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi_i2c"
	},
	.id_table	= rmi_id,
	.probe		= rmi_i2c_probe,
	.remove		= rmi_i2c_remove,
};

module_i2c_driver(rmi_i2c_driver);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com>");
MODULE_DESCRIPTION("RMI I2C driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
