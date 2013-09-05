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
#include "rmi_f11.h"

struct f11_sensor_ctl_data {
	struct f11_2d_sensor *sensor;
};

struct f11_ctl_data {
	struct rmi_function *f11_dev;
	struct rmi_control_handler_data handler_data;
	struct f11_sensor_ctl_data sensor_data[F11_MAX_NUM_OF_SENSORS];
};

#ifdef CONFIG_RMI4_DEBUG

struct sensor_debugfs_data {
	bool done;
	struct f11_2d_sensor *sensor;
};

static int sensor_debug_open(struct inode *inodep, struct file *filp)
{
	struct sensor_debugfs_data *data;
	struct f11_2d_sensor *sensor = inodep->i_private;

	data = kzalloc(sizeof(struct sensor_debugfs_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->sensor = sensor;
	filp->private_data = data;
	return 0;
}

static int sensor_debug_release(struct inode *inodep, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}
static ssize_t maxPos_read(struct file *filp, char __user *buffer, size_t size,
		    loff_t *offset) {
	int retval;
	char *local_buf;
	struct sensor_debugfs_data *data = filp->private_data;

	if (data->done)
		return 0;

	local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
	if (!local_buf)
		return -ENOMEM;

	data->done = 1;

	retval = snprintf(local_buf, size, "%u %u\n",
			data->sensor->max_x,
			data->sensor->max_y);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		retval = -EFAULT;
	kfree(local_buf);

	return retval;
}

static const struct file_operations maxPos_fops = {
	.owner = THIS_MODULE,
	.open = sensor_debug_open,
	.release = sensor_debug_release,
	.read = maxPos_read,
};


static ssize_t flip_read(struct file *filp, char __user *buffer, size_t size,
		    loff_t *offset) {
	int retval;
	char *local_buf;
	struct sensor_debugfs_data *data = filp->private_data;

	if (data->done)
		return 0;

	local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
	if (!local_buf)
		return -ENOMEM;

	data->done = 1;

	retval = snprintf(local_buf, size, "%u %u\n",
			data->sensor->axis_align.flip_x,
			data->sensor->axis_align.flip_y);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		retval = -EFAULT;
	kfree(local_buf);

	return retval;
}

static ssize_t flip_write(struct file *filp, const char __user *buffer,
			   size_t size, loff_t *offset) {
	int retval;
	char *local_buf;
	unsigned int new_X;
	unsigned int new_Y;
	struct sensor_debugfs_data *data = filp->private_data;

	local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
	if (!local_buf)
		return -ENOMEM;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval) {
		kfree(local_buf);
		return -EFAULT;
	}

	retval = sscanf(local_buf, "%u %u", &new_X, &new_Y);
	kfree(local_buf);
	if (retval != 2 || new_X > 1 || new_Y > 1)
		return -EINVAL;

	data->sensor->axis_align.flip_x = new_X;
	data->sensor->axis_align.flip_y = new_Y;

	return size;
}

static const struct file_operations flip_fops = {
	.owner = THIS_MODULE,
	.open = sensor_debug_open,
	.release = sensor_debug_release,
	.read = flip_read,
	.write = flip_write,
};

static ssize_t delta_threshold_read(struct file *filp, char __user *buffer,
		size_t size, loff_t *offset) {
	int retval;
	char *local_buf;
	struct sensor_debugfs_data *data = filp->private_data;
	struct f11_data *f11 = data->sensor->fn->data;
	struct f11_2d_ctrl *ctrl = &f11->dev_controls;

	if (data->done)
		return 0;

	local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
	if (!local_buf)
		return -ENOMEM;

	data->done = 1;

	retval = snprintf(local_buf, size, "%u %u\n",
			ctrl->ctrl0_9[RMI_F11_DELTA_X_THRESHOLD],
			ctrl->ctrl0_9[RMI_F11_DELTA_Y_THRESHOLD]);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		retval = -EFAULT;
	kfree(local_buf);

	return retval;

}

static ssize_t delta_threshold_write(struct file *filp,
		const char __user *buffer, size_t size, loff_t *offset) {
	int retval;
	char *local_buf;
	unsigned int new_X, new_Y;
	u8 save_X, save_Y;
	int rc;
	struct sensor_debugfs_data *data = filp->private_data;
	struct f11_data *f11 = data->sensor->fn->data;
	struct f11_2d_ctrl *ctrl = &f11->dev_controls;
	struct rmi_device *rmi_dev =  data->sensor->fn->rmi_dev;

	local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
	if (!local_buf)
		return -ENOMEM;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval) {
		kfree(local_buf);
		return -EFAULT;
	}

	retval = sscanf(local_buf, "%u %u", &new_X, &new_Y);
	kfree(local_buf);
	if (retval != 2 || new_X > 1 || new_Y > 1)
		return -EINVAL;

	save_X = ctrl->ctrl0_9[RMI_F11_DELTA_X_THRESHOLD];
	save_Y = ctrl->ctrl0_9[RMI_F11_DELTA_Y_THRESHOLD];

	ctrl->ctrl0_9[RMI_F11_DELTA_X_THRESHOLD] = new_X;
	ctrl->ctrl0_9[RMI_F11_DELTA_Y_THRESHOLD] = new_Y;
	rc = rmi_write_block(rmi_dev, ctrl->ctrl0_9_address, ctrl->ctrl0_9, 10);
	if (rc < 0) {
		dev_warn(&data->sensor->fn->dev,
			"Failed to write to delta_threshold. Code: %d.\n",
			rc);
		ctrl->ctrl0_9[RMI_F11_DELTA_X_THRESHOLD] = save_X;
		ctrl->ctrl0_9[RMI_F11_DELTA_Y_THRESHOLD] = save_Y;
	}

	return size;
}

static const struct file_operations delta_threshold_fops = {
	.owner = THIS_MODULE,
	.open = sensor_debug_open,
	.release = sensor_debug_release,
	.read = delta_threshold_read,
	.write = delta_threshold_write,
};

static ssize_t offset_read(struct file *filp, char __user *buffer, size_t size,
		    loff_t *offset) {
	int retval;
	char *local_buf;
	struct sensor_debugfs_data *data = filp->private_data;

	if (data->done)
		return 0;

	local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
	if (!local_buf)
		return -ENOMEM;

	data->done = 1;

	retval = snprintf(local_buf, size, "%u %u\n",
			data->sensor->axis_align.offset_X,
			data->sensor->axis_align.offset_Y);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		retval = -EFAULT;
	kfree(local_buf);

	return retval;
}

static ssize_t offset_write(struct file *filp, const char __user *buffer,
			   size_t size, loff_t *offset)
{
	int retval;
	char *local_buf;
	int new_X;
	int new_Y;
	struct sensor_debugfs_data *data = filp->private_data;

	local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
	if (!local_buf)
		return -ENOMEM;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval) {
		kfree(local_buf);
		return -EFAULT;
	}
	retval = sscanf(local_buf, "%u %u", &new_X, &new_Y);
	kfree(local_buf);
	if (retval != 2)
		return -EINVAL;

	data->sensor->axis_align.offset_X = new_X;
	data->sensor->axis_align.offset_Y = new_Y;

	return size;
}

static const struct file_operations offset_fops = {
	.owner = THIS_MODULE,
	.open = sensor_debug_open,
	.release = sensor_debug_release,
	.read = offset_read,
	.write = offset_write,
};

static ssize_t clip_read(struct file *filp, char __user *buffer, size_t size,
		    loff_t *offset) {
	int retval;
	char *local_buf;
	struct sensor_debugfs_data *data = filp->private_data;

	if (data->done)
		return 0;

	local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
	if (!local_buf)
		return -ENOMEM;

	data->done = 1;

	retval = snprintf(local_buf, size, "%u %u %u %u\n",
			data->sensor->axis_align.clip_X_low,
			data->sensor->axis_align.clip_X_high,
			data->sensor->axis_align.clip_Y_low,
			data->sensor->axis_align.clip_Y_high);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		retval = -EFAULT;
	kfree(local_buf);

	return retval;
}

static ssize_t clip_write(struct file *filp, const char __user *buffer,
			   size_t size, loff_t *offset)
{
	int retval;
	char *local_buf;
	unsigned int new_X_low, new_X_high, new_Y_low, new_Y_high;
	struct sensor_debugfs_data *data = filp->private_data;

	local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
	if (!local_buf)
		return -ENOMEM;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval) {
		kfree(local_buf);
		return -EFAULT;
	}

	retval = sscanf(local_buf, "%u %u %u %u",
		&new_X_low, &new_X_high, &new_Y_low, &new_Y_high);
	kfree(local_buf);
	if (retval != 4)
		return -EINVAL;

	if (new_X_low >= new_X_high || new_Y_low >= new_Y_high)
		return -EINVAL;

	data->sensor->axis_align.clip_X_low = new_X_low;
	data->sensor->axis_align.clip_X_high = new_X_high;
	data->sensor->axis_align.clip_Y_low = new_Y_low;
	data->sensor->axis_align.clip_Y_high = new_Y_high;

	return size;
}

static const struct file_operations clip_fops = {
	.owner = THIS_MODULE,
	.open = sensor_debug_open,
	.release = sensor_debug_release,
	.read = clip_read,
	.write = clip_write,
};

static void rmi_f11_setup_sensor_debugfs(struct f11_sensor_ctl_data *sensor_data)
{
	int retval = 0;
	char fname[NAME_BUFFER_SIZE];
	struct f11_2d_sensor *sensor = sensor_data->sensor;
	struct rmi_function *fn = sensor->fn;
	struct dentry *sensor_root;
	struct dentry *entry;

	if (!fn->debugfs_root)
		return;

	snprintf(fname, NAME_BUFFER_SIZE, "input%u", sensor->sensor_index);
	sensor_root = debugfs_create_dir(fname, fn->debugfs_root);
	if (!sensor_root) {
		dev_warn(&fn->dev,
			 "Failed to create debugfs directory %s for sensor %d\n",
			 fname, sensor->sensor_index);
		return;
	}

	retval = snprintf(fname, NAME_BUFFER_SIZE, "maxPos");
	entry = debugfs_create_file(fname, RMI_RO_ATTR,
				sensor_root, sensor, &maxPos_fops);
	if (!entry)
		dev_warn(&fn->dev, "Failed to create debugfs %s.\n",
			 fname);

	retval = snprintf(fname, NAME_BUFFER_SIZE, "flip");
	entry = debugfs_create_file(fname, RMI_RW_ATTR,
				sensor_root, sensor, &flip_fops);
	if (!entry)
		dev_warn(&fn->dev, "Failed to create debugfs %s.\n",
			 fname);

	retval = snprintf(fname, NAME_BUFFER_SIZE, "clip");
	entry = debugfs_create_file(fname, RMI_RW_ATTR,
				sensor_root, sensor, &clip_fops);
	if (!entry)
		dev_warn(&fn->dev, "Failed to create debugfs %s.\n",
			 fname);

	retval = snprintf(fname, NAME_BUFFER_SIZE, "delta_threshold");
	entry = debugfs_create_file(fname, RMI_RW_ATTR,
				sensor_root, sensor, &delta_threshold_fops);
	if (!entry)
		dev_warn(&fn->dev, "Failed to create debugfs %s.\n",
			 fname);

	retval = snprintf(fname, NAME_BUFFER_SIZE, "offset");
	entry = debugfs_create_file(fname, RMI_RW_ATTR,
				sensor_root, sensor, &offset_fops);
	if (!entry)
		dev_warn(&fn->dev, "Failed to create debugfs %s.\n",
			 fname);

	retval = snprintf(fname, NAME_BUFFER_SIZE, "swap");
	entry = debugfs_create_bool(fname, RMI_RW_ATTR,
				sensor_root, &sensor->axis_align.swap_axes);
	if (!entry)
		dev_warn(&fn->dev,
			"Failed to create debugfs swap for sensor %d.\n",
			 sensor->sensor_index);

	retval = snprintf(fname, NAME_BUFFER_SIZE, "type_a");
	entry = debugfs_create_bool(fname, RMI_RW_ATTR,
				sensor_root, &sensor->type_a);
	if (!entry)
		dev_warn(&fn->dev,
			 "Failed to create debugfs type_a for sensor %d.\n",
			 sensor->sensor_index);

	return;
}

struct f11_debugfs_data {
	loff_t pos;
	struct rmi_function *fn;
};

static int f11_debug_open(struct inode *inodep, struct file *filp)
{
	struct f11_debugfs_data *data;
	struct rmi_function *fn;

	fn = inodep->i_private;
	data = kzalloc(sizeof(struct f11_debugfs_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->fn = inodep->i_private;
	filp->private_data = data;
	return 0;
}

static int f11_debug_release(struct inode *inodep, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}

static loff_t debug_seek(struct file *filp, loff_t offset, int whence)
{
	struct f11_debugfs_data *data = filp->private_data;
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

static ssize_t report_count_read(struct file *filp, char __user *buffer, size_t size,
		    loff_t *offset) {
	struct f11_debugfs_data *data = filp->private_data;
	struct rmi_function *fn = data->fn;
	struct f11_data *f11 = fn->data;
	int retval;
	char *local_buf;
	int buf_len;
	int new_pos = data->pos + *offset;

	if (new_pos < 0) {
		dev_err(&fn->dev, "Invalid position %d.\n", new_pos);
		return -EINVAL;
	}

	local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
	if (!local_buf)
		return -ENOMEM;

	buf_len = snprintf(local_buf, size, "%llu", f11->report_count);
	if (buf_len <= 0)
		retval = -EFAULT;
	else if (new_pos >= buf_len)
		retval = 0;
	else {
		int copy_count = buf_len - new_pos;
		local_buf += new_pos;
		if (copy_to_user(buffer, local_buf, copy_count))
			retval = -EFAULT;
		else {
			retval = copy_count;
			data->pos = new_pos + copy_count;
		}
	}
	kfree(local_buf);

	return retval;
}

static const struct file_operations report_count_fops = {
	.owner = THIS_MODULE,
	.open = f11_debug_open,
	.release = f11_debug_release,
	.llseek = debug_seek,
	.read = report_count_read,
};

static ssize_t query_write(struct file *filp,
			   const char __user *buffer, size_t size, loff_t *offset) {
	char buf[size];
	unsigned int query;
	int retval;
	struct f11_debugfs_data *data = filp->private_data;
	struct f11_data *f11 = data->fn->data;
	struct device *dev = &data->fn->dev;
	struct f11_2d_sensor_queries *props = &f11->sensors[0].sens_query;

	retval = copy_from_user(buf, buffer, size);
	if (retval)
		return -EFAULT;

	if (sscanf(buf, "%u", &query) != 1)
		return -EINVAL;

	dev_dbg(&data->fn->dev, "QUERY %d\n", query);
	switch (query) {
	case 0:
		dev_dbg(dev, "Nr sensors: %d\n", f11->nr_sensors);
		dev_dbg(dev, "has_query9? %d\n", f11->has_query9);
		dev_dbg(dev, "has_query11? %d\n", f11->has_query11);
		dev_dbg(dev, "has_query12? %d\n", f11->has_query12);
		dev_dbg(dev, "has_query27? %d\n", f11->has_query27);
		dev_dbg(dev, "has_query28? %d\n", f11->has_query28);
		break;
	case 1:
		dev_dbg(dev, "Nr fingers: %d\n", props->nr_fingers);
		dev_dbg(dev, "has_rel? %d\n", props->has_rel);
		dev_dbg(dev, "has_abs? %d\n", props->has_abs);
		dev_dbg(dev, "has_gestures? %d\n", props->has_gestures);
		dev_dbg(dev, "has_sensitivity_adjust? %d\n", props->has_sensitivity_adjust);
		dev_dbg(dev, "configurable? %d\n", props->configurable);
		break;
	case 2:
		dev_dbg(dev, "Nr X electrodes: %d\n", props->nr_x_electrodes);
		break;
	case 3:
		dev_dbg(dev, "Nr Y electrodes: %d\n", props->nr_y_electrodes);
		break;
	case 4:
		dev_dbg(dev, "Max electrodes: %d\n", props->max_electrodes);
		break;
	case 5:
		dev_dbg(dev, "Abs data size: %d\n", props->abs_data_size);
		dev_dbg(dev, "has_anchored_finger? %d\n", props->has_anchored_finger);
		dev_dbg(dev, "has_adj_hyst? %d\n", props->has_adj_hyst);
		dev_dbg(dev, "has_dribble? %d\n", props->has_dribble);
		dev_dbg(dev, "has_bending_correction? %d\n", props->has_bending_correction);
		dev_dbg(dev, "has_large_object_suppression? %d\n", props->has_large_object_suppression);
		dev_dbg(dev, "has_jitter_filter? %d\n", props->has_jitter_filter);
		break;
	case 6:
		dev_dbg(dev, "Query 6 register: %#04x\n", props->f11_2d_query6);
		break;
	case 7:
		if (props->has_gestures) {
			dev_dbg(dev, "has_single_tap? %d\n", props->has_single_tap);
			dev_dbg(dev, "has_tap_n_hold? %d\n", props->has_tap_n_hold);
			dev_dbg(dev, "has_double_tap? %d\n", props->has_double_tap);
			dev_dbg(dev, "has_early_tap? %d\n", props->has_early_tap);
			dev_dbg(dev, "has_flick? %d\n", props->has_flick);
			dev_dbg(dev, "has_press? %d\n", props->has_press);
			dev_dbg(dev, "has_pinch? %d\n", props->has_pinch);
			dev_dbg(dev, "has_chiral? %d\n", props->has_chiral);
		} else
			dev_dbg(dev, "Not present.\n");
		break;
	case 8:
		if (props->has_gestures) {
			dev_dbg(dev, "has_palm_det? %d\n", props->has_palm_det);
			dev_dbg(dev, "has_rotate? %d\n", props->has_rotate);
			dev_dbg(dev, "has_touch_shapes? %d\n", props->has_touch_shapes);
			dev_dbg(dev, "has_scroll_zones? %d\n", props->has_scroll_zones);
			dev_dbg(dev, "has_individual_scroll_zones? %d\n", props->has_individual_scroll_zones);
			dev_dbg(dev, "has_mf_scroll? %d\n", props->has_mf_scroll);
			dev_dbg(dev, "has_mf_edge_motion? %d\n", props->has_mf_edge_motion);
			dev_dbg(dev, "has_mf_scroll_inertia? %d\n", props->has_mf_scroll_inertia);
		} else
			dev_dbg(dev, "Not present.\n");
		break;
	case 9:
		if (f11->has_query9) {
			dev_dbg(dev, "has_pen? %d\n", props->has_pen);
			dev_dbg(dev, "has_proximity? %d\n", props->has_proximity);
			dev_dbg(dev, "has_palm_det_sensitivity? %d\n", props->has_palm_det_sensitivity);
			dev_dbg(dev, "has_suppress_on_palm_detect? %d\n", props->has_suppress_on_palm_detect);
			dev_dbg(dev, "has_two_pen_thresholds? %d\n", props->has_two_pen_thresholds);
			dev_dbg(dev, "has_contact_geometry? %d\n", props->has_contact_geometry);
			dev_dbg(dev, "has_pen_hover_discrimination? %d\n", props->has_pen_hover_discrimination);
			dev_dbg(dev, "has_pen_filters? %d\n", props->has_pen_filters);
		} else
			dev_dbg(dev, "Not present.\n");
		break;
	case 10:
		if (props->has_touch_shapes)
			dev_dbg(dev, "Nr touch shapes: %d\n", props->nr_touch_shapes);
		else
			dev_dbg(dev, "Not present.\n");
		break;
	case 11:
		if (f11->has_query11) {
			dev_dbg(dev, "has_z_tuning? %d\n", props->has_z_tuning);
			dev_dbg(dev, "has_algorithm_selection? %d\n", props->has_algorithm_selection);
			dev_dbg(dev, "has_w_tuning? %d\n", props->has_w_tuning);
			dev_dbg(dev, "has_pitch_info? %d\n", props->has_pitch_info);
			dev_dbg(dev, "has_finger_size? %d\n", props->has_finger_size);
			dev_dbg(dev, "has_segmentation_aggressiveness? %d\n", props->has_segmentation_aggressiveness);
			dev_dbg(dev, "has_XY_clip? %d\n", props->has_XY_clip);
			dev_dbg(dev, "has_drumming_filter? %d\n", props->has_drumming_filter);
		} else
			dev_dbg(dev, "Not present.\n");
		break;
	case 12:
		if (f11->has_query12) {
			dev_dbg(dev, "has_gapless_finger? %d\n", props->has_gapless_finger);
			dev_dbg(dev, "has_gapless_finger_tuning? %d\n", props->has_gapless_finger_tuning);
			dev_dbg(dev, "has_8bit_w? %d\n", props->has_8bit_w);
			dev_dbg(dev, "has_adjustable_mapping? %d\n", props->has_adjustable_mapping);
			dev_dbg(dev, "has_info2? %d\n", props->has_info2);
			dev_dbg(dev, "has_physical_props? %d\n", props->has_physical_props);
			dev_dbg(dev, "has_finger_limit? %d\n", props->has_finger_limit);
			dev_dbg(dev, "has_linear_coeff_2? %d\n", props->has_linear_coeff_2);
		} else
			dev_dbg(dev, "Not present.\n");
		break;
	case 13:
		if (props->has_jitter_filter) {
			dev_dbg(dev, "jitter_filter_type: %d\n", props->jitter_filter_type);
			dev_dbg(dev, "jitter_filter_type: %d\n", props->jitter_filter_type);
		} else
			dev_dbg(dev, "Not present.\n");
		break;
	case 14:
		if (props->has_info2) {
			dev_dbg(dev, "light_control: %d\n", props->light_control);
			dev_dbg(dev, "is_clear? %d\n", props->is_clear);
			dev_dbg(dev, "clickpad_props: %d\n", props->clickpad_props);
			dev_dbg(dev, "mouse_buttons: %d\n", props->mouse_buttons);
			dev_dbg(dev, "has_advanced_gestures? %d\n", props->has_advanced_gestures);
		} else
			dev_dbg(dev, "Not present.\n");
		break;
	case 27:
		if (f11->has_query27) {
			// TBD
		} else
			dev_dbg(dev, "Not present.\n");
		break;
	case 28:
		if (f11->has_query28) {
			// TBD
		} else
			dev_dbg(dev, "Not present.\n");
		break;
	default:
		dev_dbg(&data->fn->dev, "Unknown or invalid query.\n");
		return -EINVAL;
	}

	return size;
}

static const struct file_operations query_fops = {
	.owner = THIS_MODULE,
	.open = f11_debug_open,
	.release = f11_debug_release,
	.write = query_write,
};

static int rmi_f11_setup_debugfs(struct f11_ctl_data *ctl_data)
{
	struct rmi_function *fn = ctl_data->f11_dev;
	struct f11_data *f11 = fn->data;
	struct dentry *entry;

	if (!fn->debugfs_root)
		return -ENODEV;

	entry = debugfs_create_u16("rezero_wait",
		RMI_RW_ATTR, fn->debugfs_root, &f11->rezero_wait_ms);
	if (!entry)
		dev_warn(&fn->dev, "Failed to create debugfs rezero_wait.\n");

	entry = debugfs_create_file("report_count", RMI_RO_ATTR,
				fn->debugfs_root, fn, &report_count_fops);
	if (!entry || IS_ERR(entry))
		dev_warn(&fn->dev, "Failed to create debugfs report_count.\n");

	entry = debugfs_create_file("query", RMI_RO_ATTR,
				    fn->debugfs_root, fn, &query_fops);
	if (!entry || IS_ERR(entry))
		dev_warn(&fn->dev, "Failed to create debugfs query.\n");

	return 0;
}

#else
#define rmi_f11_setup_sensor_debugfs(s) 0
#define rmi_f11_setup_debugfs(d) 0
#endif
/* End adding debugfs */

static ssize_t f11_rezero_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function *fn = NULL;
	unsigned int rezero;
	int retval = 0;

	fn = to_rmi_function(dev);

	if (sscanf(buf, "%u", &rezero) != 1)
		return -EINVAL;
	if (rezero > 1)
		return -EINVAL;

	/* Per spec, 0 has no effect, so we skip it entirely. */
	if (rezero) {
		retval = rmi_write(fn->rmi_dev, fn->fd.command_base_addr,
				   RMI_F11_REZERO);
		if (retval < 0) {
			dev_err(dev, "%s: failed to issue rezero command, error = %d.",
				__func__, retval);
			return retval;
		}
	}

	return count;
}

static ssize_t f11_suppress_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
        struct rmi_function *fn = NULL;
        struct f11_data *data;
        unsigned int suppress;
        int i;

        fn = to_rmi_function(dev);
        if (fn == NULL)
                return -ENODEV;

        data = fn->data;
        if (data == NULL)
                return -ENODEV;

        if (sscanf(buf, "%u", &suppress) != 1)
                return -EINVAL;
        if (suppress > 1)
                return -EINVAL;

        for (i = 0; i < (data->nr_sensors + 1); i++)
                data->sensors[i].suppress = suppress;

        return count;
}

static ssize_t f11_suppress_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
        struct rmi_function *fn;
        struct f11_data *data;

        fn = to_rmi_function(dev);
        if (fn == NULL)
                return -ENODEV;

        data = fn->data;
        if (data == NULL)
                return -ENODEV;

        return snprintf(buf, PAGE_SIZE, "%u\n",
                        data->sensors[0].suppress);
}

static ssize_t f11_suppress_highw_store(struct device *dev,
                                                        struct device_attribute *attr,
                                                        const char *buf, size_t count)
{
        struct rmi_function *fn = NULL;
        struct f11_data *data;
        unsigned int suppress_highw;
        int i;

        fn = to_rmi_function(dev);
        if (fn == NULL)
                return -ENODEV;

        data = fn->data;
        if (data == NULL)
                return -ENODEV;

        if (sscanf(buf, "%u", &suppress_highw) != 1)
                return -EINVAL;
        if (suppress_highw > 15)
                return -EINVAL;

        for (i = 0; i < (data->nr_sensors + 1); i++)
                data->sensors[i].suppress_highw = suppress_highw;

        return count;
}

static ssize_t f11_suppress_highw_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
        struct rmi_function *fn;
        struct f11_data *data;

        fn = to_rmi_function(dev);
        if (fn == NULL)
                return -ENODEV;

        data = fn->data;
        if (data == NULL)
                return -ENODEV;

        return snprintf(buf, PAGE_SIZE, "%u\n",
                        data->sensors[0].suppress_highw);
}

static struct device_attribute dev_attr_rezero =
	__ATTR(rezero, RMI_WO_ATTR, NULL, f11_rezero_store);
static struct device_attribute dev_attr_suppress =
        __ATTR(suppress, RMI_RW_ATTR, f11_suppress_show, f11_suppress_store);
static struct device_attribute dev_attr_suppress_highw =
        __ATTR(suppress_highw, RMI_RW_ATTR, f11_suppress_highw_show,
                f11_suppress_highw_store);

static struct attribute *attrs[] = {
	&dev_attr_rezero.attr,
	&dev_attr_suppress.attr,
	&dev_attr_suppress_highw.attr,
	NULL,
};
static struct attribute_group fn11_attrs = GROUP(attrs);

static int f11_ctl_cleanup(struct rmi_control_handler_data *hdata)
{
	struct f11_ctl_data *ctl_data;
	struct device *dev;

	ctl_data = container_of(hdata, struct f11_ctl_data, handler_data);

	debugfs_remove_recursive(ctl_data->f11_dev->debugfs_root);
	sysfs_remove_group(&ctl_data->f11_dev->dev.kobj, &fn11_attrs);

	dev = hdata->dev;
	devm_kfree(dev, ctl_data);

	return 0;
}

static struct rmi_control_handler_data *f11_ctl_attach(struct device *dev, void *data)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f11_data *f11 = fn->data;
	struct f11_ctl_data *ctl_data;
	int i;

	dev_dbg(dev, "%s called.\n", __func__);

	ctl_data = devm_kzalloc(dev, sizeof(struct f11_ctl_data), GFP_KERNEL);
	if (!ctl_data)
		return NULL;
	ctl_data->f11_dev = fn;

	/* Increase with one since number of sensors is zero based */
	for (i = 0; i < (f11->nr_sensors + 1); i++) {
		ctl_data->sensor_data[i].sensor = &f11->sensors[i];
#ifdef CONFIG_RMI4_DEBUG
		rmi_f11_setup_sensor_debugfs(&ctl_data->sensor_data[i]);
#endif
	}

	if (sysfs_create_group(&fn->dev.kobj, &fn11_attrs) < 0) {
		dev_warn(&fn->dev, "Failed to create query sysfs files.");
	}

#ifdef CONFIG_RMI4_DEBUG
	rmi_f11_setup_debugfs(ctl_data);
#endif

	return &ctl_data->handler_data;
}

static struct rmi_control_handler handler = {
	.name = "f11",
	.dev_type = &rmi_function_type,
	.function_id = 0x11,
	.attach = f11_ctl_attach,
	.remove = f11_ctl_cleanup,
};

static int __init f11_ctl_init(void)
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

static void __exit f11_ctl_exit(void)
{
	pr_debug("%s: exiting.\n", __func__);

	rmi_unregister_control_handler(&handler);
}

module_init(f11_ctl_init);
module_exit(f11_ctl_exit);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com>");
MODULE_DESCRIPTION("RMI4 F11 Controls");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
