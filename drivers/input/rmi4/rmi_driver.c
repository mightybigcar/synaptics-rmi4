/*
 * Copyright (c) 2011-2014 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This driver provides the core support for a single RMI4-based device.
 *
 * The RMI4 specification can be found here (URL split for line length):
 *
 * http://www.synaptics.com/sites/default/files/
 *      511-000136-01-Rev-E-RMI4%20Intrfacing%20Guide.pdf
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/bitmap.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kconfig.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include "rmi_bus.h"
#include "rmi_driver.h"
#include "rmi_f01.h"

#define HAS_NONSTANDARD_PDT_MASK 0x40
#define RMI4_MAX_PAGE 0xff
#define RMI4_PAGE_SIZE 0x100

#define RMI_PDT_ENTRY_SIZE 6
#define RMI_PDT_FUNCTION_VERSION_MASK   0x60
#define RMI_PDT_INT_SOURCE_COUNT_MASK   0x07

#define PDT_START_SCAN_LOCATION 0x00e9
#define PDT_END_SCAN_LOCATION	0x0005
#define RMI4_END_OF_PDT(id) ((id) == 0x00 || (id) == 0xff)

#define DEFAULT_POLL_INTERVAL_MS	13

#define IRQ_DEBUG(data) (IS_ENABLED(CONFIG_RMI4_DEBUG) && data->irq_debug)

static irqreturn_t rmi_irq_thread(int irq, void *p)
{
	struct rmi_transport_dev *xport = p;
	struct rmi_device *rmi_dev = xport->rmi_dev;
	struct rmi_driver *driver = rmi_dev->driver;
	struct rmi_device_platform_data *pdata = xport->dev->platform_data;
	struct rmi_driver_data *data;

	data = dev_get_drvdata(&rmi_dev->dev);

	if (IRQ_DEBUG(data))
		dev_dbg(xport->dev, "ATTN gpio, value: %d.\n",
				gpio_get_value(pdata->attn_gpio));

	if (gpio_get_value(pdata->attn_gpio) == pdata->attn_polarity) {
		data->attn_count++;
		if (driver && driver->irq_handler && rmi_dev)
			driver->irq_handler(rmi_dev, irq);
	}

	return IRQ_HANDLED;
}

static int process_interrupt_requests(struct rmi_device *rmi_dev);

static void rmi_poll_work(struct work_struct *work)
{
	struct rmi_driver_data *data =
			container_of(work, struct rmi_driver_data, poll_work);
	struct rmi_device *rmi_dev = data->rmi_dev;

	process_interrupt_requests(rmi_dev);
}

/*
 * This is the timer function for polling - it simply has to schedule work
 * and restart the timer.
 */
static enum hrtimer_restart rmi_poll_timer(struct hrtimer *timer)
{
	struct rmi_driver_data *data =
			container_of(timer, struct rmi_driver_data, poll_timer);

	if (!data->enabled)
		return HRTIMER_NORESTART;
	if (!work_pending(&data->poll_work))
		schedule_work(&data->poll_work);
	hrtimer_start(&data->poll_timer, data->poll_interval, HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static int enable_polling(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	dev_dbg(&rmi_dev->dev, "Polling enabled.\n");
	INIT_WORK(&data->poll_work, rmi_poll_work);
	hrtimer_init(&data->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->poll_timer.function = rmi_poll_timer;
	hrtimer_start(&data->poll_timer, data->poll_interval, HRTIMER_MODE_REL);

	return 0;
}

static void disable_polling(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	dev_dbg(&rmi_dev->dev, "Polling disabled.\n");
	hrtimer_cancel(&data->poll_timer);
	cancel_work_sync(&data->poll_work);
}

static void disable_sensor(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	if (!data->enabled)
		return;

	if (!data->irq)
		disable_polling(rmi_dev);

	if (rmi_dev->xport->ops->disable_device)
		rmi_dev->xport->ops->disable_device(rmi_dev->xport);

	if (data->irq) {
		disable_irq(data->irq);
		free_irq(data->irq, rmi_dev->xport);
	}

	data->enabled = false;
}

static int enable_sensor(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_transport_dev *xport;
	int retval = 0;

	if (data->enabled)
		return 0;

	if (rmi_dev->xport->ops->enable_device) {
		retval = rmi_dev->xport->ops->enable_device(rmi_dev->xport);
		if (retval)
			return retval;
	}

	xport = rmi_dev->xport;
	if (data->irq) {
		retval = request_threaded_irq(data->irq,
				xport->hard_irq ? xport->hard_irq : NULL,
				xport->irq_thread ?
					xport->irq_thread : rmi_irq_thread,
				data->irq_flags,
				dev_name(&rmi_dev->dev), xport);
		if (retval)
			return retval;
	} else {
		retval = enable_polling(rmi_dev);
		if (retval < 0)
			return retval;
	}

	data->enabled = true;

	return process_interrupt_requests(rmi_dev);
}

static void rmi_free_function_list(struct rmi_device *rmi_dev)
{
	struct rmi_function *fn, *tmp;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	data->f01_container = NULL;

	/* Doing it in the reverse order so F01 will be removed last */
	list_for_each_entry_safe_reverse(fn, tmp,
					 &data->function_list, node) {
		list_del(&fn->node);
		kfree(fn->irq_mask);
		rmi_unregister_function(fn);
	}
}

static int reset_one_function(struct rmi_function *fn)
{
	struct rmi_function_handler *fh;
	int retval = 0;

	if (!fn || !fn->dev.driver)
		return 0;

	fh = to_rmi_function_handler(fn->dev.driver);
	if (fh->reset) {
		retval = fh->reset(fn);
		if (retval < 0)
			dev_err(&fn->dev, "Reset failed with code %d.\n",
				retval);
	}

	return retval;
}

static int configure_one_function(struct rmi_function *fn)
{
	struct rmi_function_handler *fh;
	int retval = 0;

	if (!fn || !fn->dev.driver)
		return 0;

	fh = to_rmi_function_handler(fn->dev.driver);
	if (fh->config) {
		retval = fh->config(fn);
		if (retval < 0)
			dev_err(&fn->dev, "Config failed with code %d.\n",
				retval);
	}

	return retval;
}

static int rmi_driver_process_reset_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function *entry;
	int retval;

	list_for_each_entry(entry, &data->function_list, node) {
		retval = reset_one_function(entry);
		if (retval < 0)
			return retval;
	}

	return 0;
}

static int rmi_driver_process_config_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function *entry;
	int retval;

	list_for_each_entry(entry, &data->function_list, node) {
		retval = configure_one_function(entry);
		if (retval < 0)
			return retval;
	}

	return 0;
}

static void process_one_interrupt(struct rmi_driver_data *data,
				  struct rmi_function *fn)
{
	struct rmi_function_handler *fh;

	if (!fn || !fn->dev.driver)
		return;

	fh = to_rmi_function_handler(fn->dev.driver);
	if (fn->irq_mask && fh->attention) {
		bitmap_and(data->fn_irq_bits, data->irq_status, fn->irq_mask,
				data->irq_count);
		if (!bitmap_empty(data->fn_irq_bits, data->irq_count))
			fh->attention(fn, data->fn_irq_bits);
	}
}

static int process_interrupt_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct device *dev = &rmi_dev->dev;
	struct rmi_function *entry;
	int error;

	error = rmi_read_block(rmi_dev,
				data->f01_container->fd.data_base_addr + 1,
				data->irq_status, data->num_of_irq_regs);
	if (error < 0) {
		dev_err(dev, "Failed to read irqs, code=%d\n", error);
		return error;
	}

	mutex_lock(&data->irq_mutex);
	bitmap_and(data->irq_status, data->irq_status, data->current_irq_mask,
	       data->irq_count);
	/*
	 * At this point, irq_status has all bits that are set in the
	 * interrupt status register and are enabled.
	 */
	mutex_unlock(&data->irq_mutex);

	/*
	 * It would be nice to be able to use irq_chip to handle these
	 * nested IRQs.  Unfortunately, most of the current customers for
	 * this driver are using older kernels (3.0.x) that don't support
	 * the features required for that.  Once they've shifted to more
	 * recent kernels (say, 3.3 and higher), this should be switched to
	 * use irq_chip.
	 */
	list_for_each_entry(entry, &data->function_list, node)
		if (entry->irq_mask)
			process_one_interrupt(data, entry);

	return 0;
}

/**
 * rmi_driver_set_input_params - set input device id and other data.
 *
 * @rmi_dev: Pointer to an RMI device
 * @input: Pointer to input device
 *
 */
static int rmi_driver_set_input_params(struct rmi_device *rmi_dev,
				struct input_dev *input)
{
	/* FIXME: set up parent */
	input->name = SYNAPTICS_INPUT_DEVICE_NAME;
	input->id.vendor  = SYNAPTICS_VENDOR_ID;
	input->id.bustype = BUS_RMI;
	return 0;
}

/**
 * This pair of functions allows functions like function 54 to request to have
 * other interrupts disabled until the restore function is called. Only one
 * store happens at a time.
 */
static int rmi_driver_irq_save(struct rmi_device *rmi_dev,
				unsigned long *new_ints)
{
	int retval = 0;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct device *dev = &rmi_dev->dev;

	mutex_lock(&data->irq_mutex);
	if (!data->irq_stored) {
		/* Save current enabled interrupts */
		retval = rmi_read_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				data->irq_mask_store, data->num_of_irq_regs);
		if (retval < 0) {
			dev_err(dev, "%s: Failed to read enabled interrupts!",
								__func__);
			goto error_unlock;
		}
		/*
		 * Disable every interrupt except for function 54
		 * TODO:Will also want to not disable function 1-like functions.
		 * No need to take care of this now, since there's no good way
		 * to identify them.
		 */
		retval = rmi_write_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				new_ints, data->num_of_irq_regs);
		if (retval < 0) {
			dev_err(dev, "%s: Failed to change enabled interrupts!",
								__func__);
			goto error_unlock;
		}
		bitmap_copy(data->current_irq_mask, new_ints, data->irq_count);
		data->irq_stored = true;
	} else {
		retval = -ENOSPC; /* No space to store IRQs.*/
		dev_err(dev, "Attempted to save IRQs when already stored!");
	}

error_unlock:
	mutex_unlock(&data->irq_mutex);
	return retval;
}

static int rmi_driver_irq_restore(struct rmi_device *rmi_dev)
{
	int retval = 0;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct device *dev = &rmi_dev->dev;

	mutex_lock(&data->irq_mutex);

	if (data->irq_stored) {
		retval = rmi_write_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				data->irq_mask_store, data->num_of_irq_regs);
		if (retval < 0) {
			dev_err(dev, "%s: Failed to write enabled interupts!",
								__func__);
			goto error_unlock;
		}
		memcpy(data->current_irq_mask, data->irq_mask_store,
					data->num_of_irq_regs * sizeof(u8));
		data->irq_stored = false;
	} else {
		retval = -EINVAL;
		dev_err(dev, "%s: Attempted to restore values when not stored!",
			__func__);
	}

error_unlock:
	mutex_unlock(&data->irq_mutex);
	return retval;
}

static int rmi_driver_irq_handler(struct rmi_device *rmi_dev, int irq)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	might_sleep();
	/*
	 * Can get called before the driver is fully ready to deal with
	 * interrupts.
	 */
	if (!data || !data->f01_container) {
		dev_dbg(&rmi_dev->dev,
			 "Not ready to handle interrupts yet!\n");
		return 0;
	}

	return process_interrupt_requests(rmi_dev);
}

static int rmi_driver_reset_handler(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	int error;

	/*
	 * Can get called before the driver is fully ready to deal with
	 * this situation.
	 */
	if (!data || !data->f01_container) {
		dev_warn(&rmi_dev->dev,
			 "Not ready to handle reset yet!\n");
		return 0;
	}

	error = rmi_driver_process_reset_requests(rmi_dev);
	if (error < 0)
		return error;


	error = rmi_driver_process_config_requests(rmi_dev);
	if (error < 0)
		return error;

	if (data->irq_stored) {
		error = rmi_driver_irq_restore(rmi_dev);
		if (error < 0)
			return error;
	}

	return 0;
}

/*
 * Construct a function's IRQ mask. This should be called once and stored.
 */
int rmi_driver_irq_get_mask(struct rmi_device *rmi_dev,
		struct rmi_function *fn) {
	int i;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	/* call devm_kcalloc when it will be defined in kernel in future */
	fn->irq_mask = devm_kzalloc(&rmi_dev->dev,
			BITS_TO_LONGS(data->irq_count)*sizeof(unsigned long),
			GFP_KERNEL);

	if (fn->irq_mask) {
		for (i = 0; i < fn->num_of_irqs; i++)
			set_bit(fn->irq_pos+i, fn->irq_mask);
		return 0;
	} else
		return -ENOMEM;
}

static int rmi_read_pdt_entry(struct rmi_device *rmi_dev, struct pdt_entry *entry,
			u16 pdt_address)
{
	u8 buf[RMI_PDT_ENTRY_SIZE];
	int error;

	error = rmi_read_block(rmi_dev, pdt_address, buf, RMI_PDT_ENTRY_SIZE);
	if (error) {
		dev_err(&rmi_dev->dev, "Read PDT entry at %#06x failed, code: %d.\n",
				pdt_address, error);
		return error;
	}

	entry->page_start = pdt_address / RMI4_PAGE_SIZE;
	entry->query_base_addr = buf[0];
	entry->command_base_addr = buf[1];
	entry->control_base_addr = buf[2];
	entry->data_base_addr = buf[3];
	entry->interrupt_source_count = buf[4] & RMI_PDT_INT_SOURCE_COUNT_MASK;
	entry->function_version = (buf[4] & RMI_PDT_FUNCTION_VERSION_MASK) >> 5;
	entry->function_number = buf[5];

	return 0;
}

static void rmi_driver_copy_pdt_to_fd(const struct pdt_entry *pdt,
				      struct rmi_function_descriptor *fd)
{
	fd->query_base_addr = pdt->query_base_addr + pdt->page_start;
	fd->command_base_addr = pdt->command_base_addr + pdt->page_start;
	fd->control_base_addr = pdt->control_base_addr + pdt->page_start;
	fd->data_base_addr = pdt->data_base_addr + pdt->page_start;
	fd->function_number = pdt->function_number;
	fd->interrupt_source_count = pdt->interrupt_source_count;
	fd->function_version = pdt->function_version;
}

static int rmi_create_function(struct rmi_device *rmi_dev, void *ctx,
			   const struct pdt_entry *pdt)
{
	struct device *dev = &rmi_dev->dev;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);
	int *current_irq_count = ctx;
	struct rmi_function *fn;
	int i;
	int error;

	dev_dbg(dev, "Initializing F%02X for %s.\n",
		pdt->function_number, pdata->sensor_name);

	fn = kzalloc(sizeof(struct rmi_function), GFP_KERNEL);
	if (!fn) {
		dev_err(dev, "Failed to allocate memory for F%02X\n",
			pdt->function_number);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&fn->node);
	rmi_driver_copy_pdt_to_fd(pdt, &fn->fd);

	fn->rmi_dev = rmi_dev;
	fn->num_of_irqs = pdt->interrupt_source_count;

	fn->irq_pos = *current_irq_count;
	*current_irq_count += fn->num_of_irqs;

	fn->irq_mask = kzalloc(
		BITS_TO_LONGS(data->irq_count) * sizeof(unsigned long),
		GFP_KERNEL);
	if (!fn->irq_mask) {
		dev_err(dev, "%s: Failed to create irq_mask for F%02X.\n",
			__func__, pdt->function_number);
		error = -ENOMEM;
		goto err_free_mem;
	}

	for (i = 0; i < fn->num_of_irqs; i++)
 		set_bit(fn->irq_pos + i, fn->irq_mask);

	error = rmi_register_function(fn);
	if (error)
		goto err_free_irq_mask;

	if (pdt->function_number == 0x01)
		data->f01_container = fn;
	list_add_tail(&fn->node, &data->function_list);

	return RMI_SCAN_CONTINUE;

err_free_irq_mask:
	kfree(fn->irq_mask);
err_free_mem:
	kfree(fn);
	return error;
}

/* Indicates that flash programming is enabled (bootloader mode). */
#define RMI_F01_STATUS_BOOTLOADER(status)	(!!((status) & 0x40))

/*
 * Given the PDT entry for F01, read the device status register to determine
 * if we're stuck in bootloader mode or not.
 *
 */
int rmi_check_bootloader_mode(struct rmi_device *rmi_dev,
			  const struct pdt_entry *pdt)
{
	int error;
	u8 device_status;

	error = rmi_read(rmi_dev, pdt->data_base_addr, &device_status);
	if (error) {
		dev_err(&rmi_dev->dev, "Failed to read device status.\n");
		return error;
	}

	return RMI_F01_STATUS_BOOTLOADER(device_status);
}

static int rmi_count_irqs(struct rmi_device *rmi_dev,
			 void *ctx, const struct pdt_entry *pdt)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	int *irq_count = ctx;

	*irq_count += pdt->interrupt_source_count;
	if (pdt->function_number == 0x01) {
		data->f01_bootloader_mode =
			rmi_check_bootloader_mode(rmi_dev, pdt);
		if (data->f01_bootloader_mode)
			dev_warn(&rmi_dev->dev,
				"WARNING: RMI4 device is in bootloader mode!\n");
	}

	return RMI_SCAN_CONTINUE;
}

static int rmi_initial_reset(struct rmi_device *rmi_dev,
			     void *clbk_ctx, const struct pdt_entry *pdt)
{
	int retval;

	if (pdt->function_number == 0x01) {
		u8 cmd_buf = RMI_F01_CMD_DEVICE_RESET;
		const struct rmi_device_platform_data *pdata =
				to_rmi_platform_data(rmi_dev);

		retval = rmi_write_block(rmi_dev, pdt->command_base_addr,
					 &cmd_buf, 1);
		if (retval < 0) {
			dev_err(&rmi_dev->dev,
				"Initial reset failed. Code = %d.\n", retval);
			return retval;
		}
		mdelay(pdata->reset_delay_ms ?: RMI_F01_DEFAULT_RESET_DELAY_MS);

		return RMI_SCAN_DONE;
	}

	/* F01 should always be on page 0. If we don't find it there, fail. */
	return pdt->page_start == 0 ? RMI_SCAN_CONTINUE : -ENODEV;
}

static int rmi_scan_pdt_page(struct rmi_device *rmi_dev,
			     int page,
			     void *ctx,
			     int (*callback)(struct rmi_device *rmi_dev,
					     void *ctx,
					     const struct pdt_entry *entry))
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct pdt_entry pdt_entry;
	u16 page_start = RMI4_PAGE_SIZE * page;
	u16 pdt_start = page_start + PDT_START_SCAN_LOCATION;
	u16 pdt_end = page_start + PDT_END_SCAN_LOCATION;
	u16 addr;
	int error;
	int retval;

	for (addr = pdt_start; addr >= pdt_end; addr -= RMI_PDT_ENTRY_SIZE) {
		error = rmi_read_pdt_entry(rmi_dev, &pdt_entry, addr);
		if (error)
			return error;

		if (RMI4_END_OF_PDT(pdt_entry.function_number))
			break;

		retval = callback(rmi_dev, ctx, &pdt_entry);
		if (retval != RMI_SCAN_CONTINUE)
			return retval;
	}

	return data->f01_bootloader_mode ? RMI_SCAN_DONE : RMI_SCAN_CONTINUE;
}

int rmi_scan_pdt(struct rmi_device *rmi_dev, void *ctx,
		int (*callback)(struct rmi_device *rmi_dev,
				void *ctx, const struct pdt_entry *entry))
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	int page;
	int retval = RMI_SCAN_CONTINUE;

	/*
	 * TODO: With F01 and reflash as part of the core now, is this
	 * lock still required?
	 */
	mutex_lock(&data->pdt_mutex);

	for (page = 0; page <= RMI4_MAX_PAGE; page++) {
		retval = rmi_scan_pdt_page(rmi_dev, page, ctx, callback);
		if (retval != RMI_SCAN_CONTINUE)
			break;
	}

	mutex_unlock(&data->pdt_mutex);
	return retval < 0 ? retval : 0;
}


#ifdef CONFIG_PM_SLEEP
static int rmi_driver_suspend(struct device *dev)
{
	struct rmi_driver_data *data;
	int retval = 0;
	struct rmi_device *rmi_dev = to_rmi_device(dev);

	data = dev_get_drvdata(&rmi_dev->dev);

	mutex_lock(&data->suspend_mutex);

	if (data->pre_suspend) {
		retval = data->pre_suspend(data->pm_data);
		if (retval)
			goto exit;
	}

	disable_sensor(rmi_dev);

	if (data->post_suspend)
		retval = data->post_suspend(data->pm_data);

exit:
	mutex_unlock(&data->suspend_mutex);
	return retval;
}

static int rmi_driver_resume(struct device *dev)
{
	struct rmi_driver_data *data;
	int retval = 0;
	struct rmi_device *rmi_dev = to_rmi_device(dev);

	data = dev_get_drvdata(&rmi_dev->dev);
	mutex_lock(&data->suspend_mutex);

	if (data->pre_resume) {
		retval = data->pre_resume(data->pm_data);
		if (retval)
			goto exit;
	}

	retval = enable_sensor(rmi_dev);
	if (retval)
		goto exit;


	if (data->post_resume) {
		retval = data->post_resume(data->pm_data);
		if (retval)
			goto exit;
	}

	data->suspended = false;
exit:
	mutex_unlock(&data->suspend_mutex);
	return retval;
}

#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(rmi_driver_pm, rmi_driver_suspend, rmi_driver_resume);

static int rmi_driver_remove(struct device *dev)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	const struct rmi_device_platform_data *pdata =
					to_rmi_platform_data(rmi_dev);
	const struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	disable_sensor(rmi_dev);
	rmi_free_function_list(rmi_dev);

	if (data->gpio_held)
		gpio_free(pdata->attn_gpio);

	kfree(data->irq_status);
	kfree(data);

	return 0;
}

static int rmi_driver_probe(struct device *dev)
{
	struct rmi_driver *rmi_driver;
	struct rmi_driver_data *data;
	const struct rmi_device_platform_data *pdata;
	struct rmi_device *rmi_dev;
	size_t size;
	void *irq_memory;
	int irq_count;
	int retval;

	dev_dbg(dev, "%s: Starting probe.\n", __func__);

	if (!rmi_is_physical_device(dev)) {
		dev_dbg(dev, "Not a physical device.\n");
		return -ENODEV;
	}

	rmi_dev = to_rmi_device(dev);
	rmi_driver = to_rmi_driver(dev->driver);
	rmi_dev->driver = rmi_driver;

	pdata = to_rmi_platform_data(rmi_dev);

	data = devm_kzalloc(dev, sizeof(struct rmi_driver_data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "%s: Failed to allocate driver data.\n", __func__);
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&data->function_list);
	data->rmi_dev = rmi_dev;
	dev_set_drvdata(&rmi_dev->dev, data);
	mutex_init(&data->pdt_mutex);

	/*
	 * Right before a warm boot, the sensor might be in some unusual state,
	 * such as F54 diagnostics, or F34 bootloader mode after a firmware
	 * or configuration update.  In order to clear the sensor to a known
	 * state and/or apply any updates, we issue a initial reset to clear any
	 * previous settings and force it into normal operation.
	 *
	 * We have to do this before actually building the PDT because
	 * the reflash updates (if any) might cause various registers to move
	 * around.
	 *
	 * For a number of reasons, this initial reset may fail to return
	 * within the specified time, but we'll still be able to bring up the
	 * driver normally after that failure.  This occurs most commonly in
	 * a cold boot situation (where then firmware takes longer to come up
	 * than from a warm boot) and the reset_delay_ms in the platform data
	 * has been set too short to accomodate that.  Since the sensor will
	 * eventually come up and be usable, we don't want to just fail here
	 * and leave the customer's device unusable.  So we warn them, and
	 * continue processing.
	 */
	retval = rmi_scan_pdt(rmi_dev, NULL, rmi_initial_reset);
	if (retval < 0)
		dev_warn(dev, "RMI initial reset failed! Continuing in spite of this.\n");

	retval = rmi_read(rmi_dev, PDT_PROPERTIES_LOCATION, &data->pdt_props);
	if (retval < 0) {
		/*
		 * we'll print out a warning and continue since
		 * failure to get the PDT properties is not a cause to fail
		 */
		dev_warn(dev, "Could not read PDT properties from %#06x (code %d). Assuming 0x00.\n",
			 PDT_PROPERTIES_LOCATION, retval);
	}

	/*
	 * We need to count the IRQs and allocate their storage before scanning
	 * the PDT and creating the function entries, because adding a new
	 * function can trigger events that result in the IRQ related storage
	 * being accessed.
	 */
	dev_dbg(dev, "Counting IRQs.\n");
	irq_count = 0;
	retval = rmi_scan_pdt(rmi_dev, &irq_count, rmi_count_irqs);
	if (retval) {
		retval = -ENODEV;
		dev_err(dev, "IRQ counting for %s failed with code %d.\n",
			pdata->sensor_name, retval);
		goto err_free_mem;
	}
	data->irq_count = irq_count;
	data->num_of_irq_regs = (data->irq_count + 7) / 8;

	mutex_init(&data->irq_mutex);

	size = BITS_TO_LONGS(data->irq_count) * sizeof(unsigned long);
	irq_memory = kzalloc(size * 4, GFP_KERNEL);
	if (!irq_memory) {
		dev_err(dev, "Failed to allocate memory for irq masks.\n");
		goto err_free_mem;
	}

	data->irq_status	= irq_memory + size * 0;
	data->fn_irq_bits	= irq_memory + size * 1;
	data->current_irq_mask	= irq_memory + size * 2;
	data->irq_mask_store	= irq_memory + size * 3;

	/*
	 * XXX need to make sure we create F01 first...
	 * XXX or do we?  It might not be required in the updated structure.
	 */
	irq_count = 0;
	dev_dbg(dev, "Creating functions.");
	retval = rmi_scan_pdt(rmi_dev, &irq_count, rmi_create_function);
	if (retval < 0) {
		dev_err(dev, "Function creation failed with code %d.\n",
			retval);
		goto err_destroy_functions;
	}

	if (!data->f01_container) {
		dev_err(dev, "missing F01 container!\n");
		retval = -EINVAL;
		goto err_destroy_functions;
	}

	retval = rmi_read_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				data->current_irq_mask, data->num_of_irq_regs);
	if (retval < 0) {
		dev_err(dev, "%s: Failed to read current IRQ mask.\n",
			__func__);
		goto err_destroy_functions;
	}

	if (IS_ENABLED(CONFIG_PM)) {
		data->pm_data = pdata->pm_data;
		data->pre_suspend = pdata->pre_suspend;
		data->post_suspend = pdata->post_suspend;
		data->pre_resume = pdata->pre_resume;
		data->post_resume = pdata->post_resume;

		mutex_init(&data->suspend_mutex);
	}

	if (gpio_is_valid(pdata->attn_gpio)) {
		static const char GPIO_LABEL[] = "attn";
		unsigned long gpio_flags = GPIOF_DIR_IN;

		data->irq = gpio_to_irq(pdata->attn_gpio);
		if (pdata->level_triggered) {
			data->irq_flags = IRQF_ONESHOT |
				((pdata->attn_polarity == RMI_ATTN_ACTIVE_HIGH)
				? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW);
		} else {
			data->irq_flags =
				(pdata->attn_polarity == RMI_ATTN_ACTIVE_HIGH)
				? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
		}

		if (IS_ENABLED(CONFIG_RMI4_DEV))
			gpio_flags |= GPIOF_EXPORT;

		retval = gpio_request_one(pdata->attn_gpio, gpio_flags,
					  GPIO_LABEL);
		if (retval) {
			dev_warn(dev, "WARNING: Failed to request ATTN gpio %d, code=%d.\n",
				 pdata->attn_gpio, retval);
			retval = 0;
		} else {
			dev_info(dev, "Obtained ATTN gpio %d.\n",
					pdata->attn_gpio);
			data->gpio_held = true;
			if (IS_ENABLED(CONFIG_RMI4_DEV)) {
				retval = gpio_export_link(dev,
						GPIO_LABEL, pdata->attn_gpio);
				if (retval) {
					dev_warn(dev,
						"WARNING: Failed to symlink ATTN gpio!\n");
					retval = 0;
				} else {
					dev_info(dev, "Exported ATTN gpio %d.",
						pdata->attn_gpio);
				}
			}
		}
	} else {
		data->poll_interval = ktime_set(0,
			(pdata->poll_interval_ms ? pdata->poll_interval_ms :
			DEFAULT_POLL_INTERVAL_MS) * 1000 * 1000);
	}

	if (data->f01_container->dev.driver) {
		/* Driver already bound, so enable ATTN now. */
		enable_sensor(rmi_dev);
	}

	return 0;

err_destroy_functions:
	rmi_free_function_list(rmi_dev);
	kfree(irq_memory);
err_free_mem:
	if (gpio_is_valid(pdata->attn_gpio))
		gpio_free(pdata->attn_gpio);
	kfree(data);
	return retval < 0 ? retval : 0;
}

struct rmi_driver rmi_physical_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi_physical",
		.bus	= &rmi_bus_type,
		.pm	= &rmi_driver_pm,
		.probe = rmi_driver_probe,
		.remove = rmi_driver_remove,
	},
	.irq_handler = rmi_driver_irq_handler,
	.reset_handler = rmi_driver_reset_handler,
	.store_irq_mask = rmi_driver_irq_save,
	.restore_irq_mask = rmi_driver_irq_restore,
	.set_input_params = rmi_driver_set_input_params,
};

bool rmi_is_physical_driver(struct device_driver *drv)
{
	return drv == &rmi_physical_driver.driver;
}

int __init rmi_register_physical_driver(void)
{
	int error;

	error = driver_register(&rmi_physical_driver.driver);
	if (error) {
		pr_err("%s: driver register failed, code=%d.\n", __func__,
		       error);
		return error;
	}

	return 0;
}

void __exit rmi_unregister_physical_driver(void)
{
	driver_unregister(&rmi_physical_driver.driver);
}
