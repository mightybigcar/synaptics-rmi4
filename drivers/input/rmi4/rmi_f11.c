/*
 * Copyright (c) 2011-2013 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#define FUNCTION_DATA f11_data
#define FUNCTION_NUMBER 0x11

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/kconfig.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "rmi_driver.h"
#include "rmi_f11.h"

#define F11_MAX_NUM_OF_FINGERS		10
#define F11_MAX_NUM_OF_TOUCH_SHAPES	16

#define F11_REL_POS_MIN		-128
#define F11_REL_POS_MAX		127

#define FINGER_STATE_MASK	0x03

#define F11_CTRL_SENSOR_MAX_X_POS_OFFSET	6
#define F11_CTRL_SENSOR_MAX_Y_POS_OFFSET	8

#define DEFAULT_XY_MAX 9999
#define DEFAULT_MAX_ABS_MT_PRESSURE 255
#define DEFAULT_MAX_ABS_MT_TOUCH 15
#define DEFAULT_MAX_ABS_MT_ORIENTATION 1
#define DEFAULT_MIN_ABS_MT_TRACKING_ID 1
#define DEFAULT_MAX_ABS_MT_TRACKING_ID 10

/** Utility for checking bytes in the gesture info registers.  This is done
 * often enough that we put it here to declutter the conditionals.
 */
static bool has_gesture_bits(const struct f11_2d_gesture_info *info,
			     const u8 byte) {
	return ((u8 *) info)[byte] != 0;
}

enum finger_state_values {
	F11_NO_FINGER	= 0x00,
	F11_PRESENT	= 0x01,
	F11_INACCURATE	= 0x02,
	F11_RESERVED	= 0x03
};

/** F11_INACCURATE state is overloaded to indicate pen present. */
#define F11_PEN F11_INACCURATE

static int get_tool_type(struct f11_2d_sensor *sensor, u8 finger_state)
{
	if (IS_ENABLED(CONFIG_RMI4_F11_PEN) &&
			sensor->sens_query.query9.has_pen &&
			finger_state == F11_PEN)
		return MT_TOOL_PEN;
	return MT_TOOL_FINGER;
}

static void rmi_f11_rel_pos_report(struct f11_2d_sensor *sensor, u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_f11_2d_axis_alignment *axis_align = &sensor->axis_align;
	s8 x, y;
	s8 temp;

	x = data->rel_pos[n_finger].delta_x;
	y = data->rel_pos[n_finger].delta_y;

	x = min(F11_REL_POS_MAX, max(F11_REL_POS_MIN, (int)x));
	y = min(F11_REL_POS_MAX, max(F11_REL_POS_MIN, (int)y));

	if (axis_align->swap_axes) {
		temp = x;
		x = y;
		y = temp;
	}
	if (axis_align->flip_x)
		x = min(F11_REL_POS_MAX, -x);
	if (axis_align->flip_y)
		y = min(F11_REL_POS_MAX, -y);

	if (x || y) {
		input_report_rel(sensor->input, REL_X, x);
		input_report_rel(sensor->input, REL_Y, y);
#ifdef RMI4_FUNCTION_SPECIFIC_INPUT_DEVICE
		input_report_rel(sensor->mouse_input, REL_X, x);
		input_report_rel(sensor->mouse_input, REL_Y, y);
#endif
	}
#ifdef RMI4_FUNCTION_SPECIFIC_INPUT_DEVICE
	input_sync(sensor->mouse_input);
#endif
}

static int rmi_f11_abs_pos_report(struct f11_2d_sensor *sensor,
				   u8 finger_state, u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_f11_2d_axis_alignment *axis_align = &sensor->axis_align;
	struct f11_abs_pos_data *abs;
	int temp;
	int send_report = 1;


	if (finger_state) {
		abs = &sensor->abs_pos_data[n_finger];
		abs->x = ((data->abs_pos[n_finger].x_msb << 4) |
			data->abs_pos[n_finger].x_lsb);
		abs->y = ((data->abs_pos[n_finger].y_msb << 4) |
			data->abs_pos[n_finger].y_lsb);
		abs->z = data->abs_pos[n_finger].z;
		abs->w_x = data->abs_pos[n_finger].w_x;
		abs->w_y = data->abs_pos[n_finger].w_y;
		abs->w_max = max(abs->w_x, abs->w_y);
		abs->w_min = min(abs->w_x, abs->w_y);

		if (sensor->suppress_highw > 0
			&& sensor->suppress_highw <= abs->w_max)
		{
			dev_dbg(&sensor->fn->dev,
				"Suppressing finger (%d)  because of high W (%d)\n",
					n_finger, abs->w_max);
			send_report = 0;
		}

		if (axis_align->swap_axes) {
			temp = abs->x;
			abs->x = abs->y;
			abs->y = temp;
			temp = abs->w_x;
			abs->w_x = abs->w_y;
			abs->w_y = temp;
		}

		abs->orientation = abs->w_x > abs->w_y ? 1 : 0;

		if (axis_align->flip_x)
			abs->x = max(sensor->max_x - abs->x, 0);

		if (axis_align->flip_y)
			abs->y = max(sensor->max_y - abs->y, 0);

		/*
		* here checking if X offset or y offset are specified is
		*  redundant.  We just add the offsets or, clip the values
		*
		* note: offsets need to be done before clipping occurs,
		* or we could get funny values that are outside
		* clipping boundaries.
		*/
		abs->x += axis_align->offset_X;
		abs->y += axis_align->offset_Y;
		abs->x =  max(axis_align->clip_X_low, abs->x);
		abs->y =  max(axis_align->clip_Y_low, abs->y);
		if (axis_align->clip_X_high)
			abs->x = min(axis_align->clip_X_high, abs->x);
		if (axis_align->clip_Y_high)
			abs->y =  min(axis_align->clip_Y_high, abs->y);

#if 0
		dev_dbg(&sensor->fn_dev->dev,
			"finger[%d]:%d - x:%d y:%d z:%d w_max:%d w_min:%d\n",
			n_finger, finger_state, abs->x, abs->y, abs->z, abs->w_max, abs->w_min);
#endif
	}

	return send_report;
}

static void rmi_f11_send_abs_pos_report(struct f11_2d_sensor *sensor,
					u8 finger_state, u8 n_finger)
{
	struct f11_abs_pos_data *abs = &sensor->abs_pos_data[n_finger];

	/* Some UIs ignore W of zero, so we fudge it to 1 for pens.  This
	 * only appears to be an issue when reporting pens, not plain old
	 * fingers. */
	if (IS_ENABLED(CONFIG_RMI4_F11_PEN) &&
			get_tool_type(sensor, finger_state) == MT_TOOL_PEN) {
		abs->w_max = max(1, abs->w_max);
		abs->w_min = max(1, abs->w_min);
	}

	if (sensor->type_a) {
		input_report_abs(sensor->input, ABS_MT_TRACKING_ID, n_finger);
		input_report_abs(sensor->input, ABS_MT_TOOL_TYPE,
					get_tool_type(sensor, finger_state));
	} else {
		input_mt_slot(sensor->input, n_finger);
		input_mt_report_slot_state(sensor->input,
			get_tool_type(sensor, finger_state), finger_state);
	}

	if (finger_state) {
		input_report_abs(sensor->input, ABS_MT_PRESSURE, abs->z);
		input_report_abs(sensor->input, ABS_MT_TOUCH_MAJOR, abs->w_max);
		input_report_abs(sensor->input, ABS_MT_TOUCH_MINOR, abs->w_min);
		input_report_abs(sensor->input, ABS_MT_ORIENTATION, abs->orientation);
		input_report_abs(sensor->input, ABS_MT_POSITION_X, abs->x);
		input_report_abs(sensor->input, ABS_MT_POSITION_Y, abs->y);
#if 0
		dev_dbg(&sensor->fn->dev,
			"finger[%d]:%d - x:%d y:%d z:%d w_max:%d w_min:%d\n",
			n_finger, finger_state, abs->x, abs->y, abs->z, abs->w_max, abs->w_min);
#endif
	}
	/* MT sync between fingers */
	if (sensor->type_a)
		input_mt_sync(sensor->input);

	if (sensor->sensor_type == rmi_f11_sensor_touchpad)
		input_mt_report_pointer_emulation(sensor->input, true);
}

#ifdef CONFIG_RMI4_VIRTUAL_BUTTON
static int rmi_f11_virtual_button_handler(struct f11_2d_sensor *sensor)
{
	int i;
	int x;
	int y;
	struct rmi_f11_virtualbutton_map *virtualbutton_map;
	struct virtualbutton_map virtualbutton;

	if (sensor->sens_query.has_gestures &&
				sensor->data.gest_1->single_tap) {
		virtualbutton_map = &sensor->virtual_buttons;
		x = ((sensor->data.abs_pos[0].x_msb << 4) |
			sensor->data.abs_pos[0].x_lsb);
		y = ((sensor->data.abs_pos[0].y_msb << 4) |
			sensor->data.abs_pos[0].y_lsb);
		for (i = 0; i < virtualbutton_map->buttons; i++) {
			virtualbutton = virtualbutton_map->map[i];
			if (x >= virtualbutton.x &&
				x < (virtualbutton.x + virtualbutton.width) &&
				y >= virtualbutton.y &&
				y < (virtualbutton.y + virtualbutton.height)) {
				input_report_key(sensor->input,
					virtualbutton_map->map[i].code, 1);
				input_report_key(sensor->input,
					virtualbutton_map->map[i].code, 0);
				input_sync(sensor->input);
				return 0;
			}
		}
	}
	return 0;
}
#else
#define rmi_f11_virtual_button_handler(sensor)
#endif

static void rmi_f11_finger_handler(struct f11_data *f11,
				   struct f11_2d_sensor *sensor)
{
	const u8 *f_state = sensor->data.f_state;
	u8 finger_state;
	u8 finger_pressed_count;
	u8 i;
	u8 report_fingers = 1;
	int ret_val;

	if (sensor->suppress)
		return;

	for (i = 0, finger_pressed_count = 0; i < sensor->nbr_fingers; i++) {
		/* Possible of having 4 fingers per f_statet register */
		finger_state = (f_state[i / 4] >> (2 * (i % 4))) &
					FINGER_STATE_MASK;
		if (finger_state == F11_RESERVED) {
			pr_err("%s: Invalid finger state[%d]:0x%02x.", __func__,
					i, finger_state);
			continue;
		} else if ((finger_state == F11_PRESENT) ||
				(finger_state == F11_INACCURATE)) {
			finger_pressed_count++;
		}

		if (sensor->data.abs_pos) {
			ret_val = rmi_f11_abs_pos_report(sensor, finger_state, i);
			if (!ret_val)
				report_fingers = 0;

		}

		if (sensor->sensor_type != rmi_f11_sensor_touchpad && sensor->data.rel_pos)
			rmi_f11_rel_pos_report(sensor, i);
	}
	
	/* Only send abs packets when no contact exceeds the high W threshold */
	if (sensor->data.abs_pos && report_fingers) {
		for (i = 0; i < sensor->nbr_fingers; i++) {
			finger_state = (f_state[i / 4] >> (2 * (i % 4))) &
						FINGER_STATE_MASK;
			rmi_f11_send_abs_pos_report(sensor, finger_state, i);
		}
	}

	input_report_key(sensor->input, BTN_TOUCH, finger_pressed_count);
	input_sync(sensor->input);
}

static int f11_2d_construct_data(struct f11_2d_sensor *sensor)
{
	struct f11_2d_sensor_queries *query = &sensor->sens_query;
	struct f11_2d_data *data = &sensor->data;
	int i;

	sensor->nbr_fingers = (query->info.number_of_fingers == 5 ? 10 :
				query->info.number_of_fingers + 1);

	sensor->pkt_size = DIV_ROUND_UP(sensor->nbr_fingers, 4);

	if (query->info.has_abs) {
		sensor->pkt_size += (sensor->nbr_fingers * 5);
		sensor->abs_size = sensor->pkt_size;
	}

	if (query->info.has_rel)
		sensor->pkt_size +=  (sensor->nbr_fingers * 2);

	/* Check if F11_2D_Query7 is non-zero */
	if (has_gesture_bits(&query->gesture_info, 0))
		sensor->pkt_size += sizeof(u8);

	/* Check if F11_2D_Query7 or F11_2D_Query8 is non-zero */
	if (has_gesture_bits(&query->gesture_info, 0) ||
				has_gesture_bits(&query->gesture_info, 1))
		sensor->pkt_size += sizeof(u8);

	if (query->gesture_info.has_pinch || query->gesture_info.has_flick
			|| query->gesture_info.has_rotate) {
		sensor->pkt_size += 3;
		if (!query->gesture_info.has_flick)
			sensor->pkt_size--;
		if (!query->gesture_info.has_rotate)
			sensor->pkt_size--;
	}

	if (query->gesture_info.has_touch_shapes)
		sensor->pkt_size +=
			DIV_ROUND_UP(query->ts_info.nbr_touch_shapes + 1, 8);

	sensor->data_pkt = kzalloc(sensor->pkt_size, GFP_KERNEL);
	if (!sensor->data_pkt)
		return -ENOMEM;

	data->f_state = sensor->data_pkt;
	i = DIV_ROUND_UP(sensor->nbr_fingers, 4);

	if (query->info.has_abs) {
		data->abs_pos = (struct f11_2d_data_1_5 *)
				&sensor->data_pkt[i];
		i += (sensor->nbr_fingers * 5);

		sensor->abs_pos_data = kzalloc(sensor->nbr_fingers
			* sizeof(struct f11_abs_pos_data), GFP_KERNEL);
	}

	if (query->info.has_rel) {
		data->rel_pos = (struct f11_2d_data_6_7 *)
				&sensor->data_pkt[i];
		i += (sensor->nbr_fingers * 2);
	}

	if (has_gesture_bits(&query->gesture_info, 0)) {
		data->gest_1 = (struct f11_2d_data_8 *)&sensor->data_pkt[i];
		i++;
	}

	if (has_gesture_bits(&query->gesture_info, 0) ||
				has_gesture_bits(&query->gesture_info, 1)) {
		data->gest_2 = (struct f11_2d_data_9 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->gesture_info.has_pinch) {
		data->pinch = (struct f11_2d_data_10 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->gesture_info.has_flick) {
		if (query->gesture_info.has_pinch) {
			data->flick = (struct f11_2d_data_10_12 *)data->pinch;
			i += 2;
		} else {
			data->flick = (struct f11_2d_data_10_12 *)
					&sensor->data_pkt[i];
			i += 3;
		}
	}

	if (query->gesture_info.has_rotate) {
		if (query->gesture_info.has_flick) {
			data->rotate = (struct f11_2d_data_11_12 *)
					(data->flick + 1);
		} else {
			data->rotate = (struct f11_2d_data_11_12 *)
					&sensor->data_pkt[i];
			i += 2;
		}
	}

	if (query->gesture_info.has_touch_shapes)
		data->shapes = (struct f11_2d_data_13 *)&sensor->data_pkt[i];

	return 0;
}

static int f11_read_control_regs(struct rmi_function *fn,
				struct f11_2d_ctrl *ctrl, u16 ctrl_base_addr) {
	struct rmi_device *rmi_dev = fn->rmi_dev;
	u16 read_address = ctrl_base_addr;
	int error = 0;

	ctrl->ctrl0_9_address = read_address;
	error = rmi_read_block(rmi_dev, read_address, ctrl->ctrl0_9,
		sizeof(*ctrl->ctrl0_9));
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read ctrl0, code: %d.\n",
			error);
		return error;
	}
	read_address += sizeof(*ctrl->ctrl0_9);

	if (ctrl->ctrl10) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl10, sizeof(*ctrl->ctrl10));
		if (error < 0) {
			dev_err(&fn->dev,
				"Failed to read ctrl10, code: %d.\n", error);
			return error;
		}
		read_address += sizeof(*ctrl->ctrl10);
	}

	if (ctrl->ctrl11) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl11, sizeof(*ctrl->ctrl11));
		if (error < 0) {
			dev_err(&fn->dev,
				"Failed to read ctrl11, code: %d.\n", error);
			return error;
		}
		read_address += sizeof(*ctrl->ctrl11);
	}

	if (ctrl->ctrl14) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl14, sizeof(*ctrl->ctrl14));
		if (error < 0) {
			dev_err(&fn->dev,
				"Failed to read ctrl14, code: %d.\n", error);
			return error;
		}
		read_address += sizeof(*ctrl->ctrl14);
	}

	if (ctrl->ctrl15) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl15, sizeof(*ctrl->ctrl15));
		if (error < 0) {
			dev_err(&fn->dev,
				"Failed to read ctrl15, code: %d.\n", error);
			return error;
		}
		read_address += sizeof(*ctrl->ctrl15);
	}

	if (ctrl->ctrl16) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl16, sizeof(*ctrl->ctrl16));
		if (error < 0) {
			dev_err(&fn->dev,
				"Failed to read ctrl16, code: %d.\n", error);
			return error;
		}
		read_address += sizeof(*ctrl->ctrl16);
	}

	if (ctrl->ctrl17) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl17, sizeof(*ctrl->ctrl17));
		if (error < 0) {
			dev_err(&fn->dev,
				"Failed to read ctrl17, code: %d.\n", error);
			return error;
		}
		read_address += sizeof(*ctrl->ctrl17);
	}

	if (ctrl->ctrl18_19) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl18_19, sizeof(*ctrl->ctrl18_19));
		if (error < 0) {
			dev_err(&fn->dev,
				"Failed to read ctrl18_19, code: %d.\n", error);
			return error;
		}
		read_address += sizeof(*ctrl->ctrl18_19);
	}

	if (ctrl->ctrl20_21) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl20_21, sizeof(*ctrl->ctrl20_21));
		if (error < 0) {
			dev_err(&fn->dev,
				"Failed to read ctrl20_21, code: %d.\n", error);
			return error;
		}
		read_address += sizeof(*ctrl->ctrl20_21);
	}

	if (ctrl->ctrl22_26) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl22_26, sizeof(*ctrl->ctrl22_26));
		if (error < 0) {
			dev_err(&fn->dev,
				"Failed to read ctrl22_26, code: %d.\n", error);
			return error;
		}
		read_address += sizeof(*ctrl->ctrl22_26);
	}

	if (ctrl->ctrl27) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl27, sizeof(*ctrl->ctrl27));
		if (error < 0) {
			dev_err(&fn->dev,
				"Failed to read ctrl27, code: %d.\n", error);
			return error;
		}
		read_address += sizeof(*ctrl->ctrl27);
	}

	if (ctrl->ctrl28) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl28, sizeof(*ctrl->ctrl28));
		if (error < 0) {
			dev_err(&fn->dev,
				"Failed to read ctrl28, code: %d.\n", error);
			return error;
		}
		read_address += sizeof(*ctrl->ctrl28);
	}

	if (ctrl->ctrl29_30) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl29_30, sizeof(*ctrl->ctrl29_30));
		if (error < 0) {
			dev_err(&fn->dev,
				"Failed to read ctrl29_30, code: %d.\n", error);
			return error;
		}
		read_address += sizeof(*ctrl->ctrl29_30);
	}
	return 0;
}

static int f11_allocate_control_regs(struct rmi_function *fn,
				struct f11_2d_device_query *device_query,
				struct f11_2d_sensor_queries *sensor_query,
				struct f11_2d_ctrl *ctrl,
				u16 ctrl_base_addr) {

	ctrl->ctrl0_9 = devm_kzalloc(&fn->dev,
				     sizeof(struct f11_2d_ctrl0_9), GFP_KERNEL);
	if (!ctrl->ctrl0_9)
		return -ENOMEM;
	if (has_gesture_bits(&sensor_query->gesture_info, 0)) {
		ctrl->ctrl10 = devm_kzalloc(&fn->dev,
			sizeof(struct f11_2d_ctrl10), GFP_KERNEL);
		if (!ctrl->ctrl10)
			return -ENOMEM;
	}

	if (has_gesture_bits(&sensor_query->gesture_info, 1)) {
		ctrl->ctrl11 = devm_kzalloc(&fn->dev,
			sizeof(struct f11_2d_ctrl11), GFP_KERNEL);
		if (!ctrl->ctrl11)
			return -ENOMEM;
	}

	if (device_query->has_query9 && sensor_query->query9.has_pen) {
		ctrl->ctrl20_21 = devm_kzalloc(&fn->dev,
			sizeof(struct f11_2d_ctrl20_21), GFP_KERNEL);
		if (!ctrl->ctrl20_21)
			return -ENOMEM;
	}

	if (device_query->has_query9 && sensor_query->query9.has_proximity) {
		ctrl->ctrl22_26 = devm_kzalloc(&fn->dev,
			sizeof(struct f11_2d_ctrl22_26), GFP_KERNEL);
		if (!ctrl->ctrl22_26)
			return -ENOMEM;
	}

	if (device_query->has_query9 &&
		(sensor_query->query9.has_palm_det_sensitivity ||
		sensor_query->query9.has_suppress_on_palm_detect)) {
		ctrl->ctrl27 = devm_kzalloc(&fn->dev,
			sizeof(struct f11_2d_ctrl27), GFP_KERNEL);
		if (!ctrl->ctrl27)
			return -ENOMEM;
	}

	if (sensor_query->gesture_info.has_multi_finger_scroll) {
		ctrl->ctrl28 = devm_kzalloc(&fn->dev,
			sizeof(struct f11_2d_ctrl28), GFP_KERNEL);
		if (!ctrl->ctrl28)
			return -ENOMEM;
	}

	if (device_query->has_query11 &&
			sensor_query->features_1.has_z_tuning) {
		ctrl->ctrl29_30 = devm_kzalloc(&fn->dev,
			sizeof(struct f11_2d_ctrl29_30), GFP_KERNEL);
		if (!ctrl->ctrl29_30)
			return -ENOMEM;
	}

	return 0;
}

static int f11_write_control_regs(struct rmi_function *fn,
					struct f11_2d_sensor_queries *query,
					struct f11_2d_ctrl *ctrl,
					u16 ctrl_base_addr)
{
	struct rmi_device *rmi_dev = fn->rmi_dev;
	u16 write_address = ctrl_base_addr;
	int error;

	error = rmi_write_block(rmi_dev, write_address,
				ctrl->ctrl0_9,
				 sizeof(*ctrl->ctrl0_9));
	if (error < 0)
		return error;
	write_address += sizeof(ctrl->ctrl0_9);

	if (ctrl->ctrl10) {
		error = rmi_write_block(rmi_dev, write_address,
					ctrl->ctrl10, sizeof(*ctrl->ctrl10));
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl11) {
		error = rmi_write_block(rmi_dev, write_address,
					ctrl->ctrl11, sizeof(*ctrl->ctrl11));
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl14) {
		error = rmi_write_block(rmi_dev, write_address,
				ctrl->ctrl14, sizeof(ctrl->ctrl14));
		if (error < 0)
			return error;
		write_address += sizeof(*ctrl->ctrl15);
	}

	if (ctrl->ctrl15) {
		error = rmi_write_block(rmi_dev, write_address,
				ctrl->ctrl15, sizeof(*ctrl->ctrl15));
		if (error < 0)
			return error;
		write_address += sizeof(*ctrl->ctrl15);
	}

	if (ctrl->ctrl16) {
		error = rmi_write_block(rmi_dev, write_address,
				ctrl->ctrl16, sizeof(*ctrl->ctrl16));
		if (error < 0)
			return error;
		write_address += sizeof(*ctrl->ctrl16);
	}

	if (ctrl->ctrl17) {
		error = rmi_write_block(rmi_dev, write_address,
				ctrl->ctrl17, sizeof(*ctrl->ctrl17));
		if (error < 0)
			return error;
		write_address += sizeof(*ctrl->ctrl17);
	}

	if (ctrl->ctrl18_19) {
		error = rmi_write_block(rmi_dev, write_address,
			ctrl->ctrl18_19, sizeof(*ctrl->ctrl18_19));
		if (error < 0)
			return error;
		write_address += sizeof(*ctrl->ctrl18_19);
	}

	if (ctrl->ctrl20_21) {
		error = rmi_write_block(rmi_dev, write_address,
			ctrl->ctrl20_21, sizeof(*ctrl->ctrl20_21));
		if (error < 0)
			return error;
		write_address += sizeof(*ctrl->ctrl20_21);
	}

	if (ctrl->ctrl22_26) {
		error = rmi_write_block(rmi_dev, write_address,
			ctrl->ctrl22_26, sizeof(*ctrl->ctrl22_26));
		if (error < 0)
			return error;
		write_address += sizeof(*ctrl->ctrl22_26);
	}

	if (ctrl->ctrl27) {
		error = rmi_write_block(rmi_dev, write_address,
			ctrl->ctrl27, sizeof(*ctrl->ctrl27));
		if (error < 0)
			return error;
		write_address += sizeof(*ctrl->ctrl27);
	}

	if (ctrl->ctrl28) {
		error = rmi_write_block(rmi_dev, write_address,
			ctrl->ctrl28, sizeof(*ctrl->ctrl28));
		if (error < 0)
			return error;
		write_address += sizeof(*ctrl->ctrl28);
	}

	if (ctrl->ctrl29_30) {
		error = rmi_write_block(rmi_dev, write_address,
					ctrl->ctrl29_30,
					sizeof(struct f11_2d_ctrl29_30));
		if (error < 0)
			return error;
		write_address += sizeof(struct f11_2d_ctrl29_30);
	}

	return 0;
}

static int rmi_f11_get_query_parameters(struct rmi_device *rmi_dev,
			struct f11_2d_device_query *dev_query,
			struct f11_2d_sensor_queries *sensor_query,
			u16 query_base_addr)
{
	int query_size;
	int rc;

	rc = rmi_read_block(rmi_dev, query_base_addr,
			    &sensor_query->info, sizeof(sensor_query->info));
	if (rc < 0)
		return rc;
	query_size = sizeof(sensor_query->info);

	if (sensor_query->info.has_abs) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&sensor_query->abs_info);
		if (rc < 0)
			return rc;
		query_size++;
	}

	if (sensor_query->info.has_rel) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&sensor_query->f11_2d_query6);
		if (rc < 0)
			return rc;
		query_size++;
	}

	if (sensor_query->info.has_gestures) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
					&sensor_query->gesture_info,
					sizeof(sensor_query->gesture_info));
		if (rc < 0)
			return rc;
		query_size += sizeof(sensor_query->gesture_info);
	}

	if (dev_query->has_query9) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
					&sensor_query->query9,
					sizeof(sensor_query->query9));
		if (rc < 0)
			return rc;
		query_size += sizeof(sensor_query->query9);
	}

	if (sensor_query->gesture_info.has_touch_shapes) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
					&sensor_query->ts_info,
					sizeof(sensor_query->ts_info));
		if (rc < 0)
			return rc;
		query_size += sizeof(sensor_query->ts_info);
	}

	if (dev_query->has_query11) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
					&sensor_query->features_1,
					sizeof(sensor_query->features_1));
		if (rc < 0)
			return rc;
		query_size += sizeof(sensor_query->features_1);
	}

	if (dev_query->has_query12) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
					&sensor_query->features_2,
					sizeof(sensor_query->features_2));
		if (rc < 0)
			return rc;
		query_size += sizeof(sensor_query->features_2);
	}

	if (sensor_query->abs_info.has_jitter_filter) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
					&sensor_query->jitter_filter,
					sizeof(sensor_query->jitter_filter));
		if (rc < 0)
			return rc;
		query_size += sizeof(sensor_query->jitter_filter);
	}

	if (dev_query->has_query12 && sensor_query->features_2.has_info2) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
				    &sensor_query->info_2,
					sizeof(sensor_query->info_2));
		if (rc < 0)
			return rc;
		query_size += sizeof(sensor_query->info_2);
	}

	return query_size;
}

/* This operation is done in a number of places, so we have a handy routine
 * for it.
 */
static void f11_set_abs_params(struct rmi_function *fn, int index)
{
	struct f11_data *f11 = fn->data;
	struct f11_2d_sensor *sensor = &f11->sensors[index];
	struct input_dev *input = sensor->input;

	int device_x_max =
		f11->dev_controls.ctrl0_9->sensor_max_x_pos;
	int device_y_max =
		f11->dev_controls.ctrl0_9->sensor_max_y_pos;
	int x_min, x_max, y_min, y_max;
	unsigned int input_flags;
	//int res_x, res_y;

	/* We assume touchscreen unless demonstrably a touchpad or specified
	 * as a touchpad in the platform data
	 */
	if (sensor->sensor_type == rmi_f11_sensor_touchpad ||
			(sensor->sens_query.features_2.has_info2 &&
				!sensor->sens_query.info_2.is_clear))
		input_flags = INPUT_PROP_POINTER;
	else
		input_flags = INPUT_PROP_DIRECT;
	set_bit(input_flags, input->propbit);

	if (sensor->axis_align.swap_axes) {
		int temp = device_x_max;
		device_x_max = device_y_max;
		device_y_max = temp;
	}
	/* Use the max X and max Y read from the device, or the clip values,
	 * whichever is stricter.
	 */
	x_min = sensor->axis_align.clip_X_low;
	if (sensor->axis_align.clip_X_high)
		x_max = min((int) device_x_max,
			sensor->axis_align.clip_X_high);
	else
		x_max = device_x_max;

	y_min = sensor->axis_align.clip_Y_low;
	if (sensor->axis_align.clip_Y_high)
		y_max = min((int) device_y_max,
			sensor->axis_align.clip_Y_high);
	else
		y_max = device_y_max;

	dev_dbg(&fn->dev, "Set ranges X=[%d..%d] Y=[%d..%d].",
			x_min, x_max, y_min, y_max);

	input_set_abs_params(input, ABS_X, x_min, x_max, 0, 0);
	input_set_abs_params(input, ABS_Y, y_min, y_max, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, DEFAULT_MAX_ABS_MT_PRESSURE, 0, 0);

	//res_x = (x_max - x_min) / 102;
	//res_y = (y_max - y_min) / 68;

	//input_abs_set_res(input, ABS_X, res_x);
	//input_abs_set_res(input, ABS_Y, res_y);

	input_set_abs_params(input, ABS_MT_PRESSURE, 0,
			DEFAULT_MAX_ABS_MT_PRESSURE, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR,
			0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR,
			0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(input, ABS_MT_ORIENTATION,
			0, DEFAULT_MAX_ABS_MT_ORIENTATION, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID,
			DEFAULT_MIN_ABS_MT_TRACKING_ID,
			DEFAULT_MAX_ABS_MT_TRACKING_ID, 0, 0);
	/* TODO get max_x_pos (and y) from control registers. */
	input_set_abs_params(input, ABS_MT_POSITION_X,
			x_min, x_max, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y,
			y_min, y_max, 0, 0);
	//input_abs_set_res(input, ABS_MT_POSITION_X, res_x);
	//input_abs_set_res(input, ABS_MT_POSITION_Y, res_y);
	if (!sensor->type_a)
		input_mt_init_slots(input, sensor->nbr_fingers, 0);
	if (IS_ENABLED(CONFIG_RMI4_F11_PEN) &&
			sensor->sens_query.query9.has_pen)
		input_set_abs_params(input, ABS_MT_TOOL_TYPE,
				     0, MT_TOOL_MAX, 0, 0);
	else
		input_set_abs_params(input, ABS_MT_TOOL_TYPE,
				     0, MT_TOOL_FINGER, 0, 0);
}

static int rmi_f11_initialize(struct rmi_function *fn)
{
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f11_data *f11;
	struct f11_2d_ctrl *ctrl;
	u8 query_offset;
	u16 query_base_addr;
	u16 control_base_addr;
	u16 max_x_pos, max_y_pos, temp;
	int rc;
	int i;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);

	f11 = devm_kzalloc(&fn->dev, sizeof(struct f11_data), GFP_KERNEL);
	if (!f11)
		return -ENOMEM;

	fn->data = f11;
	f11->rezero_wait_ms = pdata->f11_rezero_wait;

	query_base_addr = fn->fd.query_base_addr;
	control_base_addr = fn->fd.control_base_addr;

	rc = rmi_read(rmi_dev, query_base_addr, &f11->dev_query);
	if (rc < 0)
		return rc;

	query_offset = (query_base_addr + 1);
	/* Increase with one since number of sensors is zero based */
	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		struct f11_2d_sensor *sensor = &f11->sensors[i];
		sensor->sensor_index = i;
		sensor->fn = fn;

		rc = rmi_f11_get_query_parameters(rmi_dev, &f11->dev_query,
				&sensor->sens_query, query_offset);
		if (rc < 0)
			return rc;
		query_offset += rc;

		rc = f11_allocate_control_regs(fn,
				&f11->dev_query, &sensor->sens_query,
				&f11->dev_controls, control_base_addr);
		if (rc < 0) {
			dev_err(&fn->dev,
				"Failed to allocate F11 control params.\n");
			return rc;
		}

		rc = f11_read_control_regs(fn, &f11->dev_controls,
				control_base_addr);
		if (rc < 0) {
			dev_err(&fn->dev,
				"Failed to read F11 control params.\n");
			return rc;
		}

		if (i < pdata->f11_sensor_count) {
			sensor->axis_align =
				pdata->f11_sensor_data[i].axis_align;
			sensor->virtual_buttons =
				pdata->f11_sensor_data[i].virtual_buttons;
			sensor->type_a = pdata->f11_sensor_data[i].type_a;
			sensor->sensor_type =
					pdata->f11_sensor_data[i].sensor_type;
			sensor->suppress_highw =
					pdata->f11_sensor_data[i].suppress_highw;
		}

		rc = rmi_read_block(rmi_dev,
			control_base_addr + F11_CTRL_SENSOR_MAX_X_POS_OFFSET,
			(u8 *)&max_x_pos, sizeof(max_x_pos));
		if (rc < 0)
			return rc;

		rc = rmi_read_block(rmi_dev,
			control_base_addr + F11_CTRL_SENSOR_MAX_Y_POS_OFFSET,
			(u8 *)&max_y_pos, sizeof(max_y_pos));
		if (rc < 0)
			return rc;

		if (sensor->axis_align.swap_axes) {
			temp = max_x_pos;
			max_x_pos = max_y_pos;
			max_y_pos = temp;
		}
		sensor->max_x = max_x_pos;
		sensor->max_y = max_y_pos;

		pr_info("%s: before f11_2d_construct_data\n", __func__);
		rc = f11_2d_construct_data(sensor);
		if (rc < 0)
			return rc;
		pr_info("%s: after f11_2d_construct_data\n", __func__);

		ctrl = &f11->dev_controls;
		if (sensor->axis_align.delta_x_threshold) {
			ctrl->ctrl0_9->delta_x_threshold =
				sensor->axis_align.delta_x_threshold;
			rc = rmi_write_block(rmi_dev,
					ctrl->ctrl0_9_address,
					ctrl->ctrl0_9,
					sizeof(*ctrl->ctrl0_9));
			if (rc < 0)
				dev_warn(&fn->dev, "Failed to write to delta_x_threshold %d. Code: %d.\n",
					i, rc);

		}

		if (sensor->axis_align.delta_y_threshold) {
			ctrl->ctrl0_9->delta_y_threshold =
				sensor->axis_align.delta_y_threshold;
			rc = rmi_write_block(rmi_dev,
					ctrl->ctrl0_9_address,
					ctrl->ctrl0_9,
					sizeof(*ctrl->ctrl0_9));
			if (rc < 0)
				dev_warn(&fn->dev, "Failed to write to delta_y_threshold %d. Code: %d.\n",
					i, rc);
		}
	}

	mutex_init(&f11->dev_controls_mutex);
	return 0;
}

static void register_virtual_buttons(struct rmi_function *fn,
				     struct f11_2d_sensor *sensor) {
	int j;

	if (!sensor->sens_query.info.has_gestures)
		return;
	if (!sensor->virtual_buttons.buttons) {
		dev_warn(&fn->dev, "No virtual button platform data for 2D sensor %d.\n",
			 sensor->sensor_index);
		return;
	}
	/* call devm_kcalloc when it will be defined in kernel */
	sensor->button_map = devm_kzalloc(&fn->dev,
			sensor->virtual_buttons.buttons,
			GFP_KERNEL);
	if (!sensor->button_map) {
		dev_err(&fn->dev, "Failed to allocate the virtual button map.\n");
		return;
	}

	/* manage button map using input subsystem */
	sensor->input->keycode = sensor->button_map;
	sensor->input->keycodesize = sizeof(u8);
	sensor->input->keycodemax = sensor->virtual_buttons.buttons;

	/* set bits for each button... */
	for (j = 0; j < sensor->virtual_buttons.buttons; j++) {
		sensor->button_map[j] =  sensor->virtual_buttons.map[j].code;
		set_bit(sensor->button_map[j], sensor->input->keybit);
	}
}

static int rmi_f11_register_devices(struct rmi_function *fn)
{
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f11_data *f11 = fn->data;
	struct input_dev *input_dev;
#ifdef RMI4_FUNCTION_SPECIFIC_INPUT_DEVICE
	struct input_dev *input_dev_mouse;
	struct rmi_driver *driver = rmi_dev->driver;
#endif
	int i;
	struct rmi_driver_data *driver_data = dev_get_drvdata(&rmi_dev->dev);
	int sensors_itertd = 0;
	int rc;
	int board, version;

	board = driver_data->board;
	version = driver_data->rev;

	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		struct f11_2d_sensor *sensor = &f11->sensors[i];
		sensors_itertd = i;
#ifdef RMI4_FUNCTION_SPECIFIC_INPUT_DEVICE
		input_dev = input_allocate_device();
		if (!input_dev) {
			rc = -ENOMEM;
			goto error_unregister;
		}
		sensor->input = input_dev;
		if (driver->set_input_params) {
			rc = driver->set_input_params(rmi_dev, input_dev);
			if (rc < 0) {
				dev_err(&fn->dev,
				"%s: Error in setting input device.\n",
				__func__);
				goto error_unregister;
			}
		}
		sprintf(sensor->input_phys, "%s.abs%d/input0",
			dev_name(&fn->dev), i);
		input_dev->phys = sensor->input_phys;
		input_dev->dev.parent = &rmi_dev->dev;
		input_set_drvdata(input_dev, f11);
#else
		input_dev = driver_data->input;
		sensor->input = input_dev;
#endif

		set_bit(EV_SYN, input_dev->evbit);
		set_bit(EV_ABS, input_dev->evbit);
		input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

		f11_set_abs_params(fn, i);

		if (sensor->sensor_type != rmi_f11_sensor_touchpad
			&& sensor->sens_query.info.has_rel)
		{
			set_bit(EV_REL, input_dev->evbit);
			set_bit(REL_X, input_dev->relbit);
			set_bit(REL_Y, input_dev->relbit);
		}
#ifdef RMI4_FUNCTION_SPECIFIC_INPUT_DEVICE
		rc = input_register_device(input_dev);
		if (rc < 0) {
			input_free_device(input_dev);
			sensor->input = NULL;
			goto error_unregister;
		}
#endif

		if (IS_ENABLED(CONFIG_RMI4_VIRTUAL_BUTTON))
			register_virtual_buttons(fn, sensor);

#ifdef RMI4_FUNCTION_SPECIFIC_INPUT_DEVICE
		if (sensor->sens_query.info.has_rel) {
			/*create input device for mouse events  */
			input_dev_mouse = input_allocate_device();
			if (!input_dev_mouse) {
				rc = -ENOMEM;
				goto error_unregister;
			}

			sensor->mouse_input = input_dev_mouse;
			if (driver->set_input_params) {
				rc = driver->set_input_params(rmi_dev,
					input_dev_mouse);
				if (rc < 0) {
					dev_err(&fn->dev,
					"%s: Error in setting input device.\n",
					__func__);
					goto error_unregister;
				}
			}
			sprintf(sensor->input_phys_mouse, "%s.rel%d/input0",
				dev_name(&fn->dev), i);
			set_bit(EV_REL, input_dev_mouse->evbit);
			set_bit(REL_X, input_dev_mouse->relbit);
			set_bit(REL_Y, input_dev_mouse->relbit);

			set_bit(BTN_MOUSE, input_dev_mouse->evbit);
			/* Register device's buttons and keys */
			set_bit(EV_KEY, input_dev_mouse->evbit);
			set_bit(BTN_LEFT, input_dev_mouse->keybit);
			set_bit(BTN_MIDDLE, input_dev_mouse->keybit);
			set_bit(BTN_RIGHT, input_dev_mouse->keybit);

			rc = input_register_device(input_dev_mouse);
			if (rc < 0) {
				input_free_device(input_dev_mouse);
				sensor->mouse_input = NULL;
				goto error_unregister;
			}

			set_bit(BTN_RIGHT, input_dev_mouse->keybit);
		}
#endif
		if (sensor->sensor_type == rmi_f11_sensor_touchpad) {
			set_bit(EV_KEY, input_dev->evbit);
			set_bit(BTN_LEFT, input_dev->keybit);

			set_bit(BTN_TOOL_FINGER, input_dev->keybit);
			set_bit(BTN_TOOL_DOUBLETAP, input_dev->keybit);
			set_bit(BTN_TOOL_TRIPLETAP, input_dev->keybit);
			set_bit(BTN_TOOL_QUADTAP, input_dev->keybit);
			set_bit(BTN_TOOL_QUINTTAP, input_dev->keybit);
		}

	}

	return 0;

#ifdef RMI4_FUNCTION_SPECIFIC_INPUT_DEVICE
error_unregister:
	for (; sensors_itertd > 0; sensors_itertd--) {
		if (f11->sensors[sensors_itertd].input) {
			if (f11->sensors[sensors_itertd].mouse_input) {
				input_unregister_device(
				   f11->sensors[sensors_itertd].mouse_input);
				f11->sensors[sensors_itertd].mouse_input = NULL;
			}
			input_unregister_device(f11->sensors[i].input);
			f11->sensors[i].input = NULL;
		}
	}
#endif

	return rc;
}

#ifdef RMI4_FUNCTION_SPECIFIC_INPUT_DEVICE
static void rmi_f11_free_devices(struct rmi_function *fn)
{
	struct f11_data *f11 = fn->data;
	int i;
	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		if (f11->sensors[i].input)
			input_unregister_device(f11->sensors[i].input);
		if (f11->sensors[i].mouse_input)
			input_unregister_device(f11->sensors[i].mouse_input);
	}
}
#endif

static int rmi_f11_config(struct rmi_function *fn)
{
	struct f11_data *f11 = fn->data;
	int i;
	int rc;

	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		rc = f11_write_control_regs(fn, &f11->sensors[i].sens_query,
				&f11->dev_controls, fn->fd.query_base_addr);
		if (rc < 0)
			return rc;
	}

	return 0;
}

int rmi_f11_attention(struct rmi_function *fn,
						unsigned long *irq_bits)
{
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f11_data *f11 = fn->data;
	u16 data_base_addr = fn->fd.data_base_addr;
	u16 data_base_addr_offset = 0;
	int error;
	int i;

	f11->report_count++;

	for (i = 0; i < f11->dev_query.nbr_of_sensors + 1; i++) {
		if (rmi_dev->xport->attn_data) {
			memcpy(f11->sensors[0].data_pkt,
				rmi_dev->xport->attn_data,
				f11->sensors[i].abs_size);
			rmi_dev->xport->attn_data += f11->sensors[i].abs_size;
			rmi_dev->xport->attn_size -= f11->sensors[i].abs_size;
		} else {
			error = rmi_read_block(rmi_dev,
					data_base_addr + data_base_addr_offset,
					f11->sensors[i].data_pkt,
					f11->sensors[i].pkt_size);
			if (error < 0)
				return error;
		}
	

		rmi_f11_finger_handler(f11, &f11->sensors[i]);
		rmi_f11_virtual_button_handler(&f11->sensors[i]);
		data_base_addr_offset += f11->sensors[i].pkt_size;
	}

	return 0;
}

#ifdef CONFIG_PM
static int rmi_f11_resume(struct device *dev)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f11_data *data = fn->data;
	/* Command register always reads as 0, so we can just use a local. */
	struct f11_2d_commands commands = {
		.rezero = true,
	};
	int retval = 0;

	dev_dbg(&fn->dev, "Resuming...\n");
	if (!data->rezero_wait_ms)
		return 0;

	mdelay(data->rezero_wait_ms);

	retval = rmi_write_block(rmi_dev, fn->fd.command_base_addr,
			&commands, sizeof(commands));
	if (retval < 0) {
		dev_err(&fn->dev, "%s: failed to issue rezero command, error = %d.",
			__func__, retval);
		return retval;
	}

	return retval;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(rmi_f11_pm_ops, NULL, rmi_f11_resume);

static int rmi_f11_remove(struct rmi_function *fn)
{
#ifdef RMI4_FUNCTION_SPECIFIC_INPUT_DEVICE
	rmi_f11_free_devices(fn);
#endif
	return 0;
}

static int rmi_f11_probe(struct rmi_function *fn)
{
	int rc;

	dev_dbg(&fn->dev, "%s called.\n", __func__);

	rc = rmi_f11_initialize(fn);
	if (rc < 0)
		return rc;

	rc = rmi_f11_register_devices(fn);
	if (rc < 0)
		return rc;

	return 0;
}

static struct rmi_function_driver function_driver = {
	.driver = {
		.name = "rmi_f11",
		.pm = &rmi_f11_pm_ops,
	},
	.func = FUNCTION_NUMBER,
	.probe = rmi_f11_probe,
	.remove = rmi_f11_remove,
	.config = rmi_f11_config,
	.attention = rmi_f11_attention,
};

module_rmi_function_driver(function_driver);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com");
MODULE_DESCRIPTION("RMI F11 module");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
