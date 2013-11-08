/*
 * Copyright (c) 2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#define FUNCTION_DATA f05_data
#define FUNCTION_NUMBER 0x05

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/kconfig.h>
#include <linux/rmi.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include "rmi_driver.h"
#include "rmi_f01.h"
#include "rmi_f05.h"

/* character device name for fast image transfer (if enabled)
 * the device_register will add another '0' to this, making it
 * "f05rawsensor00"
 */
#define RAW_IMAGE_F05_CHAR_DEVICE_NAME "f05rawsensor0"

/** A note about RMI4 F05 register structure.
 *
 * Function 05 providces image reporting functions that allow direct visibility
 * into image sensing.  Only some devices support this feature.
 *
 * Function 05 image reporting provides direct access to low-level capacitance
 * data before processing and interpretation (for example, to derive finger or
 * palm status) for testing and debugging purposes.
 *
 * Controls for specific modes of operation are not contained within the image
 * reporting function. For example:
 *	Controls for algorithms used in image data processing and
 *	interpretation. Scaling sensitivities.
 *
 * The image data reported by Function 05 is a matrix of signed integer values,
 * corresponding to each intersection in the grid of transmitter and receiver
 * electrodes used for transcapacitive sensing.  It does not map these values
 * to an X, Y coordinate space.  It also reports button capacitances on an
 * additional last row (or rows).
 *
 * Operation in normal reporting modes after or during the use of Function 05
 * is not provided, nand may required a reset command to return the sensor to
 * normal operation.
 */

/**
 * @get_image - Setting this bit requests an image to be captured for reading
 * from F05 data registers.  The host should wait for the resulting interrupt
 * before attempting to read the data.  In order to ensure coherency of the
 * image during the reading process, reporting of a new image is stalled until
 * the get_image bit is again set to 1.
 *
 * @force_zero - Setting this bit requests a new baseline image to be taken
 * causing the delta image to be forced to zero(for all pexels).  The host
 * should wait (poll) for the force_zero bit to clear before attempting to next
 * read for report_data. The next baseline read from report_data is from the
 * frame following the forced zero of the delta image.
 */
struct f05_commands {
	u8 reserved1:2;
	u8 get_image:1;
	u8 reserved2:2;
	u8 force_zero:1;
	u8 reserved3:2;
} __attribute__((__packed__));

/**
 * @num_of_rx_electrodes - This 6-bit field reports the number of sensor
 * electrodes available in the design.
 * @num_of_tx_electrodes - This 6-bit field reports the number of drive
 * electrodes available in the design.
 * @has16_bit_delta - This bit field reports the size of the delta image when
 * report_mode = 2.
 *	if it is 0, a 1-byte delta image will be reported.
 *	if it is 1, a 2-byte delta iamge will be reported.
 * @izeOfF05ImageWindow - This field indicates the number of register bytes
 * mapped in f05 image data registers - f05_analog_data2.  The value is
 * 2 x num_of_rx_electrodes.
 */
struct f05_device_queries {
	/* query0 */
	u8 num_of_rx_electrodes:6;
	u8 reserved0:2;
	/* query1 */
	u8 num_of_tx_electrodes:6;
	u8 reserved1:2;
	/* query2 */
	u8 reserved2:8;
	/* query3 */
	u8 reserved3:7;
	u8 has16_bit_delta:1;
	/* query4 */
	u8 size_f05_image_window:8;
} __attribute__((__packed__));

/**
 * @no_auto_cal - When set, this bit prevents normal AutoCalibration.
 * AutoCalibration allows the sensor to adjust its operation to veriations
 * in temperature and environmental conditions and should not normally be
 * disabled.
 */
struct f05_device_controls {
	u8 reverved1:4;
	u8 no_auto_cal:1;
	u8 reverved2:3;
} __attribute__((__packed__));

/**
 * @report_index - the Baseline Image Line (report_mode=1) field specifies
 * the line number (transmitter electrode number, 0
 * through num_of_tx_electrodes) of the image data.
 * @report_mode - Specifies the report mode for the data in the report_data
 * registers.
 */
struct f05_device_data {
	u8 reserved;
	u8 report_index:6;
	u8 report_mode:2;
} __attribute__((__packed__));

/**
 * @get_image - Setting this bit request an image to be capture for reeading
 * from f05 data registers.  The host should wait for the resulting interrupt
 * before attempting to read the data.  In order to ensure coherency of the
 * image during the reading process, reporting of a new image is stalled until
 * the get_image bit is again set to '1'.
 * @force_zero - setting this bit requests a new baseline image to be taken
 * causing the delta image to be forced to zero(for all pixels). The host
 * should wait (poll) for the force_zero bit to clear before attempting to next
 * read from report_data.  The next baseline read from
 * the frame following the forced zero of the delta image.
 */
struct f05_device_commands {
	u8 reset1:2;
	u8 get_image:1;
	u8 reserved1:2;
	u8 force_zero:1;
	u8 reserved2:2;
} __attribute__((__packed__));

/**
 * @mutex_file_op lock for file and status operations.
 * @raw_data_dev The main character device structure
 * @busy this is true if someone already has access to the device.
 */
struct f05_raw_data_char_dev {
	struct mutex mutex_file_op;
	bool busy;

	struct cdev raw_data_dev;
	struct class *raw_data_device_class;
};

/**
 *
 * @f05_device_queries - F05 device specific query registers.
 * @f05_device_controls - F05 device specific control registers.
 * @f05_device_data - F05 device specific data registers.
 * @f05_device_commands - F05 device specific command registers.
 * @report_data - This replicated register contains on line of the image.
 * Each 1 - or 2 -byte value contains the baseline or delta image pixel data
 * for the corresponsding receiver number and the transmitter selected by
 * report_index.
 * @tmp_buffer - Buffer for pending read.
 * @report_size - number of bytes in the current report.
 * @row_size - number of bytes per row in the current report.
 */
struct f05_data {
	struct f05_device_queries dev_query;
	struct f05_device_controls dev_control;
	struct f05_device_data dev_data;
	struct f05_device_commands dev_command;
	u8 *report_data;
	u8 *tmp_buffer;
	u8 report_size;
	u8 row_size;
	u8 max_report_size;
	struct mutex status_mutex;
	signed char status;
	struct f05_raw_data_char_dev raw_data_feed;
	struct rmi_function *fn;
	bool data_ready;
	wait_queue_head_t wq;

};

/** Update the status.  This should be called with status_mutex held.
 */
static void set_status(struct f05_data *f05, unsigned char new_status)
{
	if (!mutex_is_locked(&f05->status_mutex))
		dev_err(&f05->fn->dev, "ERROR: status_mutex not held for set_status() call!\n");
	f05->status = new_status;
}

/**
 * f05_char_devnode - return device permission
 *
 * @dev: char device structure
 * @mode: file permission
 *
 */
static char *f05_char_devnode(struct device *dev, mode_t *mode)
{
	if (!mode)
		return NULL;
	/* rmi** */
	/**mode = 0666*/
	*mode = (S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH);
	dev_dbg(dev, "%s: setting mode of %s to %#09o\n", __func__,
		RAW_IMAGE_F05_CHAR_DEVICE_NAME, *mode);
	return kasprintf(GFP_KERNEL, "%s", dev_name(dev));
}

/**
 * f05_char_dev_cleanup - release memory or unregister driver
 * @data: instance data for a particular device.
 *
 */
static void f05_char_dev_cleanup(struct f05_raw_data_char_dev *data)
{
	dev_t devno;

	/* Get rid of our char dev entries */
	if (data) {
		devno = data->raw_data_dev.dev;

		if (data->raw_data_device_class)
			device_destroy(data->raw_data_device_class, devno);

		cdev_del(&data->raw_data_dev);

		/* cleanup_module is never called if registering failed */
		unregister_chrdev_region(devno, 1);
	}
}

static ssize_t f05_char_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct rmi_function *fn;
	struct f05_data *f05 = NULL;
	struct f05_raw_data_char_dev *my_dev = NULL;
	ssize_t ret_value  = 0;

	if (count == 0) {
		pr_err("%s: count = %d -- no space to copy output to!!!\n",
			__func__, count);
		return -ENOMEM;
	}

	if (!filp) {
		pr_err("%s: called with NULL file pointer\n", __func__);
		return -EINVAL;
	}
	my_dev = filp->private_data;

	if (!my_dev) {
		pr_err("%s: called with NULL private_data\n", __func__);
		return -EINVAL;
	}

	f05 = container_of(my_dev, struct f05_data, raw_data_feed);

	fn = f05->fn;

#define WAIT_CONDITION (f05->data_ready || \
	!(f05->status == F05_STATE_PENDING || f05->status == F05_STATE_RUN))
	mutex_lock(&f05->status_mutex);
	while (!WAIT_CONDITION) {
		mutex_unlock(&f05->status_mutex);
		wait_event_timeout(f05->wq, WAIT_CONDITION, 100);
		mutex_lock(&f05->status_mutex);
	}

	switch (f05->status) {
	/* State is OK. */
	case F05_STATE_RUN:
	case F05_STATE_PENDING:
		if (count < f05->report_size) {
			dev_err(&fn->dev, "%s: count = %d but need %d bytes -- not enough space\n",
				__func__, count, f05->report_size);
			ret_value = -EINVAL;
			goto exit_unlock;
		}
		break;
	/* Anything else is just wrong */
	case F05_STATE_IDLE:
		dev_err(&fn->dev, "ERROR: read() in idle state.\n");
		ret_value = -ECONNRESET;
		goto exit_unlock;
	case F05_STATE_ERROR:
		dev_err(&fn->dev, "ERROR: read() in error state - cleared.\n");
		ret_value = -EIO;
		set_status(f05, F05_STATE_READY);
		f05->data_ready = false;
		goto exit_unlock;
	case F05_STATE_READY:
		dev_err(&fn->dev, "ERROR: read() in ready state.\n");
		ret_value = -EIO;
		f05->data_ready = false;
		goto exit_unlock;
	default:
		dev_err(&fn->dev, "ERROR: F05 is in unrecognized state %d.\n",
			f05->status);
		ret_value = -EIO;
		set_status(f05, F05_STATE_READY);
		f05->data_ready = false;
		goto exit_unlock;
	}

	if (!f05->data_ready) {
		dev_warn(&fn->dev, "WARNING: No data available for read.\n");
		ret_value = -ENODATA;
		goto exit_unlock;
	}

	/* By the time we get here, we should be in an acceptable state, with
	 * data ready to send to the user.
	 */

	ret_value = copy_to_user((void __user *)buf,
				 (const void *)f05->report_data,
				 f05->report_size);
	if (ret_value) {
		dev_err(&fn->dev, "ERROR: Failed to copy F05 data.\n");
		f05->data_ready = false;
		ret_value = -EFAULT;
		goto exit_unlock;
	}
	ret_value = count;
	*f_pos += f05->report_size;
	f05->data_ready = false;

exit_unlock:
	mutex_unlock(&f05->status_mutex);
	return ret_value;
}

static int do_set_report_mode(struct f05_data *f05, u8 report_mode)
{
	struct rmi_function *fn = f05->fn;
	int report_size;
	u8 row_size;
	int retval;

	switch (report_mode) {
		/* this is baseline capacitance image data */
	case 1:
		row_size = f05->dev_query.num_of_rx_electrodes * 2;
		break;
		/* this is delta image data */
	case 2:
		if (f05->dev_query.has16_bit_delta == 0)
			row_size = f05->dev_query.num_of_rx_electrodes;
		else
			row_size = f05->dev_query.num_of_rx_electrodes * 2;
		break;
	default:
		dev_err(&fn->dev, "%s: Report mode %d is unrecognized.\n",
			__func__, report_mode);
		return -EINVAL;
	}

	report_size = row_size * f05->dev_query.num_of_tx_electrodes;

	mutex_lock(&f05->status_mutex);
	if (f05->status == F05_STATE_READY) {
		f05->data_ready = false;
		f05->dev_data.report_mode = report_mode;
		f05->report_size = report_size;
		f05->row_size = row_size;

		retval = rmi_write_block(fn->rmi_dev,
					fn->fd.data_base_addr,
					(u8 *) &f05->dev_data,
					sizeof(f05->dev_data));
		if (retval < 0)
			dev_err(&fn->dev, "%s: Could not write report mode %d, code: %d.\n",
				__func__, report_mode, retval);
	} else {
		dev_err(&fn->dev, "%s: Report type cannot change in state %d.\n",
			__func__, f05->status);
		retval =  -EINVAL;
	}
	mutex_unlock(&f05->status_mutex);

	return retval;
}

static void do_pause(struct f05_data *f05)
{
	struct rmi_function *fn = f05->fn;
	struct rmi_driver *driver = fn->rmi_dev->driver;
	int retval;

	mutex_lock(&f05->status_mutex);
	set_status(f05, F05_STATE_READY);
	f05->data_ready = false;

	f05->dev_data.report_mode = 0;
	retval = rmi_write_block(fn->rmi_dev, fn->fd.data_base_addr,
			&f05->dev_data, sizeof(f05->dev_data));
	if (retval < 0) {
		dev_warn(&fn->dev, "WARNING: Failed to clear F05 report mode, code; %d.\n",
			retval);
	}

	driver->restore_irq_mask(fn->rmi_dev);

	mutex_unlock(&f05->status_mutex);
	wake_up(&f05->wq);
}

static void do_stop(struct f05_data *f05)
{
	struct rmi_function *fn = f05->fn;
	struct rmi_driver *driver = fn->rmi_dev->driver;
	struct rmi_driver_data *driver_data;
	struct rmi_device_platform_data *pdata;
	struct rmi_function *f01_dev;
	int retval;

	driver_data = dev_get_drvdata(&fn->rmi_dev->dev);
	pdata = to_rmi_platform_data(fn->rmi_dev);
	f01_dev = driver_data->f01_dev;

	mutex_lock(&f05->status_mutex);
	if (f05->status == F05_STATE_IDLE) {
		mutex_unlock(&f05->status_mutex);
		return;
	}
	set_status(f05, F05_STATE_IDLE);
	f05->data_ready = false;

	f05->dev_data.report_mode = 0;
	/* Write 0 to the Report Mode back to the first Block
		* Data registers. */
	retval = rmi_write_block(fn->rmi_dev, fn->fd.data_base_addr,
			&f05->dev_data, sizeof(f05->dev_data));
	if (retval < 0) {
		dev_warn(&fn->dev, "%s : Could not write report mode to 0x%x\n",
			__func__, fn->fd.data_base_addr);
	}

	mutex_unlock(&f05->status_mutex);

	driver->restore_irq_mask(fn->rmi_dev);

	retval = rmi_write(fn->rmi_dev, f01_dev->fd.command_base_addr,
			   RMI_F01_CMD_DEVICE_RESET);
	if (retval < 0)
		dev_warn(&fn->rmi_dev->dev,
				"WARNING - post-F05 reset failed, code: %d.\n",
				retval);
	msleep(pdata->reset_delay_ms);
}

/** Request the next report from the sensor.
 */
static int request_next_report(struct f05_data *f05) {
	int retval = 0;

	mutex_lock(&f05->status_mutex);

	if (f05->status != F05_STATE_RUN) {
		dev_err(&f05->fn->dev, "ERROR: Can't request report in state %d.\n",
			f05->status);
		retval = -EIO;
		goto exit_unlock;
	}

	f05->dev_command.force_zero = 0;
	f05->dev_command.get_image = 1;
	retval = rmi_write_block(f05->fn->rmi_dev,
			f05->fn->fd.command_base_addr,
			&f05->dev_command, sizeof(f05->dev_command));
	if (retval < 0) {
		dev_err(&f05->fn->dev, "ERROR: Failed to request report, code: %d.\n",
			retval);
		set_status(f05, F05_STATE_ERROR);
		goto exit_unlock;
	}
	set_status(f05, F05_STATE_PENDING);

exit_unlock:
	mutex_unlock(&f05->status_mutex);
	return retval;
}

/*
 * f05_char_dev_write: - use to write data into RMI stream
 * First byte is indication of parameter to change
 *
 * @filep : file structure for write
 * @buf: user-level buffer pointer contains data to be written
 * @count: number of byte be be written
 * @f_pos: offset (starting register address)
 *
 * @return number of bytes written from user buffer (buf) if succeeds
 *         negative number if error occurs.
 */
static ssize_t f05_char_dev_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *f_pos)
{
	struct f05_data *f05 = NULL;
	struct f05_raw_data_char_dev *my_dev = NULL;
	struct rmi_function *fn;
	struct rmi_driver *driver;
	struct rmi_driver_data *driver_data;
	struct rmi_function *f01_dev;
	char tmpbuf[2];
	char command;
	int retval = 0;
	if (!filp) {
		dev_err(&fn->dev, "%s: called with NULL file pointer\n",
			__func__);
		return -EINVAL;
	}
	my_dev = filp->private_data;

	if (!my_dev) {
		dev_err(&fn->dev, "%s: called with NULL private_data\n",
			__func__);
		return -EINVAL;
	}
	f05 = container_of(my_dev, struct f05_data, raw_data_feed);
	fn = f05->fn;
	driver = fn->rmi_dev->driver;
	driver_data = dev_get_drvdata(&fn->rmi_dev->dev);

	retval = copy_from_user(tmpbuf, buf, count);
	command = tmpbuf[0];
	switch (command) {
	case F05_REPORT_SET_TYPE:
		if (count < 1) {
			dev_err(&fn->dev, "%s: missing report type.\n",
						__func__);
			retval = -EINVAL;
			goto error_exit;
		}
		retval = do_set_report_mode(f05, (u8)tmpbuf[1]);
		if (retval < 0)
			goto error_exit;
		break;
	case F05_REPORT_START:
		/* Overwrite and store interrupts */
		mutex_lock(&f05->status_mutex);
		if (f05->status == F05_STATE_READY) {
			DECLARE_BITMAP(irq_bits, driver_data->num_of_irq_regs);
			if (driver->store_irq_mask) {
				f01_dev = driver_data->f01_dev;
				bitmap_or(irq_bits, f01_dev->irq_mask,
					fn->irq_mask,
					driver_data->irq_count);

				driver->store_irq_mask(fn->rmi_dev,
								irq_bits);
			}
			set_status(f05, F05_STATE_RUN);
			f05->data_ready = false;
			mutex_unlock(&f05->status_mutex);
			request_next_report(f05);
		} else {
			dev_err(&fn->dev, "%s: cannot start report in state %d.\n",
				__func__, f05->status);
			mutex_unlock(&f05->status_mutex);
			retval =  -EINVAL;
			goto error_exit;
		}
		break;
	case F05_REPORT_PAUSE:
		do_pause(f05);
		break;
	case F05_REPORT_STOP:
		do_stop(f05);
		break;
	case F05_REPORT_FORCE_ZERO:
		mutex_lock(&f05->status_mutex);
		if (f05->status == F05_STATE_READY) {
			f05->data_ready = false;
			f05->dev_command.get_image = 0;
			f05->dev_command.force_zero = 1;
			retval = rmi_write_block(f05->fn->rmi_dev,
					f05->fn->fd.command_base_addr,
					&f05->dev_command,
					sizeof(f05->dev_command));
			mutex_unlock(&f05->status_mutex);
			if (retval < 0) {
				dev_err(&fn->dev, "%s: Could not write command register, code: %d.\n",
					__func__, retval);
				goto error_exit;
			}
		} else {
			dev_err(&fn->dev, "%s: cannot force zero in state %d.\n",
				__func__, f05->status);
			mutex_unlock(&f05->status_mutex);
			retval =  -EINVAL;
			goto error_exit;
		}
		break;
	default:
		dev_err(&fn->dev, "%s: invalid command %d.\n", __func__,
			command);
		retval = -EINVAL;
		goto error_exit;
	}
	retval = count;
error_exit:
	return retval;
}


/*
 * SynSens_char_dev_open: - get a new handle for reading raw Touch Sensor images
 * @inp : inode struture
 * @filp: file structure for read/write
 *
 * @return 0 if succeeds
 */
static int f05_char_dev_open(struct inode *inp, struct file *filp)
{
	struct f05_raw_data_char_dev *my_dev;
	int retval = 0;
	struct f05_data *f05;

	my_dev = container_of(inp->i_cdev, struct f05_raw_data_char_dev,
			      raw_data_dev);
	f05 = container_of(my_dev, struct f05_data, raw_data_feed);

	mutex_lock(&my_dev->mutex_file_op);
	if (my_dev->busy) {
		dev_warn(&f05->fn->dev, "Device is busy.\n");
		retval = -EBUSY;
	}
	else {
		my_dev->busy = true;
		filp->private_data = my_dev;
		mutex_lock(&f05->status_mutex);
		set_status(f05, F05_STATE_READY);
		f05->data_ready = false;
		mutex_unlock(&f05->status_mutex);
	}
	mutex_unlock(&my_dev->mutex_file_op);

	return retval;
}

/*
 *  SynSens_char_dev_release: - release an existing handle
 *  @inp: inode structure
 *  @filp: file structure for read/write
 *
 *  @return 0 if succeeds
 */
static int f05_char_dev_release(struct inode *inp, struct file *filp)
{
	struct f05_data *f05 = NULL;
	struct f05_raw_data_char_dev *my_dev = NULL;

	my_dev = filp->private_data;

	if (!my_dev) {
		pr_err("ERROR: %s called with NULL private_data.\n", __func__);
		return -EINVAL;
	}
	f05 = container_of(my_dev, struct f05_data, raw_data_feed);

	do_stop(f05);
	mutex_lock(&f05->status_mutex);
	set_status(f05, F05_STATE_IDLE);
	f05->data_ready = false;
	mutex_unlock(&f05->status_mutex);
	mutex_lock(&my_dev->mutex_file_op);
	my_dev->busy = false;
	mutex_unlock(&my_dev->mutex_file_op);

	return 0;
}

static const struct file_operations f05_char_dev_fops = {
	.owner =    THIS_MODULE,
	.write =    f05_char_dev_write,
	.read =     f05_char_dev_read,
	.open =     f05_char_dev_open,
	.release =  f05_char_dev_release,
};

static ssize_t rmi_fn_05_num_of_rx_electrodes_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct f05_data *data = NULL;
	struct rmi_function *fn = to_rmi_function(dev);

	data = fn->data;

	return snprintf(buf, PAGE_SIZE, "0x%x\n",
		data->dev_query.num_of_rx_electrodes);
}

static ssize_t rmi_fn_05_num_of_tx_electrodes_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct f05_data *data = NULL;
	struct rmi_function *fn = to_rmi_function(dev);

	data = fn->data;

	return snprintf(buf, PAGE_SIZE, "0x%x\n",
		data->dev_query.num_of_tx_electrodes);
}

static ssize_t rmi_fn_05_has_delta16_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct f05_data *data = NULL;
	struct rmi_function *fn = to_rmi_function(dev);

	data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
		data->dev_query.has16_bit_delta);
}

static ssize_t rmi_fn_05_image_window_size_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct f05_data *data = NULL;
	struct rmi_function *fn = to_rmi_function(dev);

	data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
		data->dev_query.size_f05_image_window);
}

static ssize_t  rmi_fn_05_no_auto_cal_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct f05_data *data = NULL;
	struct rmi_function *fn = to_rmi_function(dev);

	data = fn->data;

	return snprintf(buf, PAGE_SIZE,
			"%d\n", data->dev_control.no_auto_cal);
}

static ssize_t  rmi_fn_05_no_auto_cal_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct f05_data *data = NULL;
	unsigned long new_value;
	int retval;
	struct rmi_function *fn = to_rmi_function(dev);

	data = fn->data;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 1) {
		dev_err(dev, "%s: Invalid no auto cal bit %s.", __func__, buf);
		return -EINVAL;
	}

	data->dev_control.no_auto_cal = new_value;
	retval = rmi_write_block(fn->rmi_dev, fn->fd.control_base_addr,
			&data->dev_control,
			sizeof(data->dev_control));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write no auto cal bit, code: %d.\n",
			retval);
	return retval;
}

/* query register access */
static struct device_attribute dev_attr_num_of_rx_electrodes =
	__ATTR(num_of_rx_electrodes, RMI_RO_ATTR,
		rmi_fn_05_num_of_rx_electrodes_show, NULL);

static struct device_attribute dev_attr_num_of_tx_electrodes =
	__ATTR(num_of_tx_electrodes, RMI_RO_ATTR,
		rmi_fn_05_num_of_tx_electrodes_show, NULL);

static struct device_attribute dev_attr_has_delta16 =
	__ATTR(has_delta16, RMI_RO_ATTR,
		rmi_fn_05_has_delta16_show, NULL);

static struct device_attribute dev_attr_image_window_size =
	__ATTR(image_window_size, RMI_RO_ATTR,
		rmi_fn_05_image_window_size_show, NULL);

/* control register access */
static struct device_attribute dev_attr_no_auto_cal =
	__ATTR(no_auto_cal, RMI_RW_ATTR,
	       rmi_fn_05_no_auto_cal_show, rmi_fn_05_no_auto_cal_store);

static struct attribute *attrs[] = {
	&dev_attr_num_of_rx_electrodes.attr,
	&dev_attr_num_of_tx_electrodes.attr,
	&dev_attr_has_delta16.attr,
	&dev_attr_image_window_size.attr,
	&dev_attr_no_auto_cal.attr,
	NULL
};

static struct attribute_group fn05_attrs = GROUP(attrs);

/*store dynamically allocated major number of char device*/
static int rmi_f05_char_dev_major_num;

/*
 * f05_raw_data_char_dev_register - register char device
 * called from init
 *
 * @phy: a pointer to an rmi_transport_devices structure
 *
 * @return: zero if suceeds
 */
static int f05_raw_data_char_dev_register(struct f05_data *f05)
{
	dev_t dev_no;
	int err;
	int result;
	struct device *device_ptr;
	struct f05_raw_data_char_dev *char_dev;

	if (!f05) {
		pr_err("%s - ERROR: No RMI F05 data structure instance.\n",
			__func__);
		return -EINVAL;
	}
	char_dev = &f05->raw_data_feed;

	if (rmi_f05_char_dev_major_num) {
		dev_no = MKDEV(rmi_f05_char_dev_major_num, 0);
		result = register_chrdev_region(dev_no, 1,
					RAW_IMAGE_F05_CHAR_DEVICE_NAME);
	} else {
		result = alloc_chrdev_region(&dev_no, 0, 1,
					     RAW_IMAGE_F05_CHAR_DEVICE_NAME);
		/* let kernel allocate a major for us */
		rmi_f05_char_dev_major_num = MAJOR(dev_no);
	}

	if (result < 0)
		return result;


	mutex_init(&char_dev->mutex_file_op);


	/* initialize the device */
	cdev_init(&char_dev->raw_data_dev, &f05_char_dev_fops);

	char_dev->raw_data_dev.owner = THIS_MODULE;

	/* tell the linux kernel to add the device */
	err = cdev_add(&char_dev->raw_data_dev, dev_no, 1);

	if (err) {
		dev_err(&f05->fn->dev, "%s: Error %d adding f05_raw_data_char_dev.\n",
			__func__, err);
		return err;
	}

	/* create device node */
	f05->raw_data_feed.raw_data_device_class =
		class_create(THIS_MODULE, RAW_IMAGE_F05_CHAR_DEVICE_NAME);

	if (IS_ERR(f05->raw_data_feed.raw_data_device_class)) {
		dev_err(&f05->fn->dev, "%s: Failed to create /dev/%s.\n",
				__func__, RAW_IMAGE_F05_CHAR_DEVICE_NAME);

		f05_char_dev_cleanup(char_dev);
		return -ENODEV;
	}

	/* setup permission */
	f05->raw_data_feed.raw_data_device_class->devnode = f05_char_devnode;

	/* class creation */
	device_ptr = device_create(
		f05->raw_data_feed.raw_data_device_class,
		NULL, dev_no, NULL,
		RAW_IMAGE_F05_CHAR_DEVICE_NAME"%d",
		MINOR(dev_no));

	if (IS_ERR(device_ptr)) {
		dev_err(&f05->fn->dev, "Failed to create raw_data_read device.\n");
		f05_char_dev_cleanup(char_dev);
		return -ENODEV;
	}

	return 0;
}


static int rmi_f05_initialize(struct rmi_function *fn)
{
	struct f05_data *f05;
	u16 query_base_addr;
	u16 control_base_addr;
	u8 num_of_rx_electrodes;
	u8 num_of_tx_electrodes;
	int rc;
	struct rmi_device *rmi_dev = fn->rmi_dev;

	f05 = devm_kzalloc(&fn->dev, sizeof(struct f05_data), GFP_KERNEL);
	if (!f05) {
		dev_err(&fn->dev, "Failed to allocate fn_05_data.\n");
		return -ENOMEM;
	}
	fn->data = f05;
	f05->fn = fn;

	query_base_addr = fn->fd.query_base_addr;
	control_base_addr = fn->fd.control_base_addr;
	rc = rmi_read_block(rmi_dev, query_base_addr, &f05->dev_query,
				sizeof(struct f05_device_queries));

	if (rc < 0) {
		dev_err(&fn->dev, "Failed to read f05 query register, code: %d.\n",
			rc);
		return rc;
	}

	rc = rmi_read(rmi_dev, control_base_addr, &f05->dev_control);
	if (rc < 0) {
		dev_err(&fn->dev, "Failed to read f05 control register, code: %d.\n",
			rc);
		return rc;
	}

	num_of_rx_electrodes = f05->dev_query.num_of_rx_electrodes;
	num_of_tx_electrodes = f05->dev_query.num_of_tx_electrodes + 1;
	f05->max_report_size = num_of_tx_electrodes * num_of_rx_electrodes *  2;
	f05->report_data = devm_kzalloc(&fn->dev, f05->max_report_size,
					GFP_KERNEL);
	f05->tmp_buffer = devm_kzalloc(&fn->dev, f05->max_report_size,
					GFP_KERNEL);

	mutex_init(&f05->status_mutex);
	mutex_lock(&f05->status_mutex);
	set_status(f05, F05_STATE_IDLE);
	f05->data_ready = false;
	mutex_unlock(&f05->status_mutex);
	init_waitqueue_head(&f05->wq);
	f05_raw_data_char_dev_register(f05);
	return 0;
}


static int rmi_f05_probe(struct rmi_function *fn)
{
	int rc;

	rc = rmi_f05_initialize(fn);
	if (rc < 0)
		return rc;

	if (sysfs_create_group(&fn->dev.kobj, &fn05_attrs) < 0) {
		dev_err(&fn->dev, "Failed to create query sysfs files.");
		return -ENODEV;
	}
	return 0;
}

static int rmi_f05_remove(struct rmi_function *fn)
{
	struct f05_data *f05 = fn->data;
	sysfs_remove_group(&fn->dev.kobj, &fn05_attrs);
	class_destroy(f05->raw_data_feed.raw_data_device_class);
	return 0;
}

static int rmi_f05_reset(struct rmi_function *fn)
{
	do_stop(fn->data);
	return 0;
}

static int rmi_f05_attention(struct rmi_function *fn,
						unsigned long *irq_bits)
{
	u8 row_size;
	int i;
	int retval = 0;
	struct f05_data *f05;
	u8 *tmp_ptr;
	u8 *stash;
	int report_size;

	f05 = fn->data;

	mutex_lock(&f05->status_mutex);
	if (f05->status != F05_STATE_PENDING) {
		mutex_unlock(&f05->status_mutex);
		return 0;
	}
	report_size = f05->report_size;
	row_size = f05->row_size;
	mutex_unlock(&f05->status_mutex);

	tmp_ptr = f05->tmp_buffer;
	for (i = 0; i < f05->dev_query.num_of_tx_electrodes; i++) {
		f05->dev_data.report_index = i;
		retval = rmi_write_block(fn->rmi_dev,
				fn->fd.data_base_addr,
				&f05->dev_data, sizeof(f05->dev_data));
		if (retval < 0) {
			dev_err(&fn->dev, "Failed to write to report index! code: %d.\n",
				retval);
			goto error_exit;
		} else
			retval = rmi_read_block(fn->rmi_dev,
				(fn->fd.data_base_addr + sizeof(f05->dev_data)),
				tmp_ptr, row_size);
		if (retval < 0) {
			dev_err(&fn->dev, "Failed to read report data! code: %d.\n",
				retval);
			goto error_exit;
		}
		tmp_ptr += row_size;
	}
	mutex_lock(&f05->status_mutex);
	if (f05->status != F05_STATE_PENDING) {
		/* State changed while we were handling the read. */
		mutex_unlock(&f05->status_mutex);
		goto exit;
	}

	stash = f05->report_data;
	f05->report_data = f05->tmp_buffer;
	f05->tmp_buffer = stash;
	f05->data_ready = true;
	set_status(f05, F05_STATE_RUN);
	mutex_unlock(&f05->status_mutex);
	wake_up(&f05->wq);
	request_next_report(f05);
	goto exit;

error_exit:
	mutex_lock(&f05->status_mutex);
	f05->data_ready = false;
	set_status(f05, F05_STATE_ERROR);
	mutex_unlock(&f05->status_mutex);
	wake_up(&f05->wq);
exit:
	return 0;
}
static struct rmi_function_driver function_driver = {
	.driver = {
		.name = "rmi_f05",
	},
	.func = FUNCTION_NUMBER,
	.probe = rmi_f05_probe,
	.remove = rmi_f05_remove,
	.reset = rmi_f05_reset,
	.attention = rmi_f05_attention,
};

module_rmi_function_driver(function_driver);

MODULE_AUTHOR("Vivian Ly <vly@synaptics.com>");
MODULE_DESCRIPTION("RMI F05 module");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
