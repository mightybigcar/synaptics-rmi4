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

enum finger_state_values {
	F11_NO_FINGER	= 0x00,
	F11_PRESENT	= 0x01,
	F11_INACCURATE	= 0x02,
	F11_PRODUCT_SPECIFIC	= 0x03
};

/** F11_INACCURATE state is overloaded to indicate pen present. */
#define F11_PEN F11_INACCURATE

static int get_tool_type(struct f11_2d_sensor *sensor, u8 finger_state)
{
	if (IS_ENABLED(CONFIG_RMI4_F11_PEN) &&
			sensor->sens_query.has_pen &&
			finger_state == F11_PEN)
		return MT_TOOL_PEN;
	return MT_TOOL_FINGER;
}

static void rmi_f11_rel_pos_report(struct f11_2d_sensor *sensor, u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_2d_axis_alignment *axis_align = &sensor->axis_align;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(sensor->fn->rmi_dev);
	s8 x, y;
	s8 temp;

	x = data->rel_pos[n_finger * 2];
	y = data->rel_pos[n_finger * 2 + 1];

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
		if (!pdata->unified_input_device) {
			input_report_rel(sensor->mouse_input, REL_X, x);
			input_report_rel(sensor->mouse_input, REL_Y, y);
		}
	}
	if (!pdata->unified_input_device)
		input_sync(sensor->mouse_input);
}

static int rmi_f11_abs_pos_report(struct f11_2d_sensor *sensor,
				   u8 finger_state, u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_2d_axis_alignment *axis_align = &sensor->axis_align;
	struct f11_abs_pos_data *abs;
	int temp;
	int send_report = 1;
	u8 abs_base = n_finger * RMI_F11_ABS_BYTES;


	if (finger_state) {
		abs = &sensor->abs_pos_data[n_finger];
		abs->x = (data->abs_pos[abs_base] << 4) |
			(data->abs_pos[abs_base + 2] & 0x0F);
		abs->y = (data->abs_pos[abs_base + 1] << 4) |
			(data->abs_pos[abs_base + 2] >> 4);
		abs->w_x = data->abs_pos[abs_base + 3] & 0x0F;
		abs->w_y = data->abs_pos[abs_base + 3] >> 4;
		abs->w_max = max(abs->w_x, abs->w_y);
		abs->w_min = min(abs->w_x, abs->w_y);
		abs->z = data->abs_pos[abs_base + 4];

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

	if (sensor->sensor_type == rmi_sensor_touchpad)
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
		(sensor->data.gest_1[0] & RMI_F11_SINGLE_TAP)) {
		virtualbutton_map = &sensor->virtual_buttons;
		x = (data->abs_pos[0] << 4) | (data->abs_pos[2] & 0x0F);
		y = (data->abs_pos[0] << 4) | (data->abs_pos[2] >> 4);
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

static void rmi_f11_shape_handler(struct f11_2d_sensor *sensor)
{
	u8 i;

	if (!(sensor->data.gest_2[0] & RMI_F11_SHAPE))
		return;

	for (i = 0; i < sensor->sens_query.nr_touch_shapes; i++) {
		bool pressed = !!(sensor->data.shapes[i / 8] & (1 << (i % 8)));
		if (pressed)
			dev_dbg(&sensor->fn->dev, "Shape %d pressed.\n", i);
		// TODO: report this
	}
}

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
		/* Possible of having 4 fingers per f_state register */
		finger_state = (f_state[i / 4] >> (2 * (i % 4))) &
					FINGER_STATE_MASK;
		if (finger_state == F11_PRODUCT_SPECIFIC) {
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

		if (sensor->sensor_type != rmi_sensor_touchpad && sensor->data.rel_pos)
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

	if (sensor->data.shapes)
		rmi_f11_shape_handler(sensor);

	input_sync(sensor->input);
}

static int f11_2d_construct_data(struct f11_2d_sensor *sensor)
{
	struct f11_2d_sensor_queries *query = &sensor->sens_query;
	struct f11_2d_data *data = &sensor->data;
	int i;

	sensor->nbr_fingers = (query->nr_fingers == 5 ? 10 :
				query->nr_fingers + 1);

	sensor->pkt_size = DIV_ROUND_UP(sensor->nbr_fingers, 4);

	if (query->has_abs) {
		sensor->pkt_size += (sensor->nbr_fingers * 5);
		sensor->abs_size = sensor->pkt_size;
	}

	if (query->has_rel)
		sensor->pkt_size +=  (sensor->nbr_fingers * 2);

	/* Check if F11_2D_Query7 is non-zero */
	if (query->query7_nonzero)
		sensor->pkt_size += sizeof(u8);

	/* Check if F11_2D_Query7 or F11_2D_Query8 is non-zero */
	if (query->query7_nonzero || query->query8_nonzero)
		sensor->pkt_size += sizeof(u8);

	if (query->has_pinch || query->has_flick || query->has_rotate) {
		sensor->pkt_size += 3;
		if (!query->has_flick)
			sensor->pkt_size--;
		if (!query->has_rotate)
			sensor->pkt_size--;
	}

	if (query->has_touch_shapes)
		sensor->pkt_size +=
			DIV_ROUND_UP(query->nr_touch_shapes + 1, 8);

	sensor->data_pkt = kzalloc(sensor->pkt_size, GFP_KERNEL);
	if (!sensor->data_pkt)
		return -ENOMEM;

	data->f_state = sensor->data_pkt;
	i = DIV_ROUND_UP(sensor->nbr_fingers, 4);

	if (query->has_abs) {
		data->abs_pos = &sensor->data_pkt[i];
		i += (sensor->nbr_fingers * RMI_F11_ABS_BYTES);

		sensor->abs_pos_data = kzalloc(sensor->nbr_fingers
			* sizeof(struct f11_abs_pos_data), GFP_KERNEL);
	}

	if (query->has_rel) {
		data->rel_pos = &sensor->data_pkt[i];
		i += (sensor->nbr_fingers * RMI_F11_REL_BYTES);
	}

	if (query->query7_nonzero) {
		data->gest_1 = &sensor->data_pkt[i];
		i++;
	}

	if (query->query7_nonzero || query->query8_nonzero) {
		data->gest_2 = &sensor->data_pkt[i];
		i++;
	}

	if (query->has_pinch) {
		data->pinch = &sensor->data_pkt[i];
		i++;
	}

	if (query->has_flick) {
		if (query->has_pinch) {
			data->flick = data->pinch;
			i += 2;
		} else {
			data->flick = &sensor->data_pkt[i];
			i += 3;
		}
	}

	if (query->has_rotate) {
		if (query->has_flick) {
			data->rotate = data->flick + 1;
		} else {
			data->rotate = &sensor->data_pkt[i];
			i += 2;
		}
	}

	if (query->has_touch_shapes)
		data->shapes = &sensor->data_pkt[i];

	return 0;
}

static int f11_read_control_regs(struct rmi_function *fn,
				struct f11_2d_ctrl *ctrl, u16 ctrl_base_addr) {
	struct rmi_device *rmi_dev = fn->rmi_dev;
	int error;

	ctrl->ctrl0_9_address = ctrl_base_addr;
	error = rmi_read_block(rmi_dev, ctrl_base_addr, ctrl->ctrl0_9, 10);
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read ctrl0, code: %d.\n", error);
		return error;
	}

	return 0;
}

static int f11_write_control_regs(struct rmi_function *fn,
					struct f11_2d_sensor_queries *query,
					struct f11_2d_ctrl *ctrl,
					u16 ctrl_base_addr)
{
	struct rmi_device *rmi_dev = fn->rmi_dev;
	int error;

	error = rmi_write_block(rmi_dev, ctrl_base_addr, ctrl->ctrl0_9, 10);
	if (error < 0)
		return error;

	return 0;
}

static int rmi_f11_get_query_parameters(struct rmi_device *rmi_dev,
			struct f11_data *f11,
			struct f11_2d_sensor_queries *sensor_query,
			u16 query_base_addr)
{
	int query_size;
	int rc;
	u8 query_buf[4];

	rc = rmi_read_block(rmi_dev, query_base_addr, query_buf, 4);
	if (rc < 0)
		return rc;

	sensor_query->nr_fingers = query_buf[0] & RMI_F11_NR_FINGERS_MASK;
	sensor_query->has_rel = !!(query_buf[0] & RMI_F11_HAS_REL);
	sensor_query->has_abs = !!(query_buf[0] & RMI_F11_HAS_ABS);
	sensor_query->has_gestures = !!(query_buf[0] & RMI_F11_HAS_GESTURES);
	sensor_query->has_sensitivity_adjust =
	!!(query_buf[0] && RMI_F11_HAS_SENSITIVITY_ADJ);
	sensor_query->configurable = !!(query_buf[0] & RMI_F11_CONFIGURABLE);

	sensor_query->nr_x_electrodes =
				query_buf[1] & RMI_F11_NR_ELECTRODES_MASK;
	sensor_query->nr_y_electrodes =
				query_buf[2] & RMI_F11_NR_ELECTRODES_MASK;
	sensor_query->max_electrodes =
				query_buf[3] & RMI_F11_NR_ELECTRODES_MASK;

	query_size = 4;

	if (sensor_query->has_abs) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size, query_buf);
		if (rc < 0)
			return rc;

		sensor_query->abs_data_size =
			query_buf[0] & RMI_F11_ABS_DATA_SIZE_MASK;
		sensor_query->has_anchored_finger =
			!!(query_buf[0] & RMI_F11_HAS_ANCHORED_FINGER);
		sensor_query->has_adj_hyst =
			!!(query_buf[0] & RMI_F11_HAS_ADJ_HYST);
		sensor_query->has_dribble =
			!!(query_buf[0] & RMI_F11_HAS_DRIBBLE);
		sensor_query->has_bending_correction =
			!!(query_buf[0] & RMI_F11_HAS_BENDING_CORRECTION);
		sensor_query->has_large_object_suppression =
		!!(query_buf[0] && RMI_F11_HAS_LARGE_OBJECT_SUPPRESSION);
		sensor_query->has_jitter_filter =
			!!(query_buf[0] & RMI_F11_HAS_JITTER_FILTER);
		query_size++;
	}

	if (sensor_query->has_rel) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&sensor_query->f11_2d_query6);
		if (rc < 0)
			return rc;
		query_size++;
	}

	if (sensor_query->has_gestures) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
					query_buf, 2);
		if (rc < 0)
			return rc;

		sensor_query->has_single_tap =
			!!(query_buf[0] & RMI_F11_HAS_SINGLE_TAP);
		sensor_query->has_tap_n_hold =
			!!(query_buf[0] & RMI_F11_HAS_TAP_AND_HOLD);
		sensor_query->has_double_tap =
			!!(query_buf[0] & RMI_F11_HAS_DOUBLE_TAP);
		sensor_query->has_early_tap =
			!!(query_buf[0] & RMI_F11_HAS_EARLY_TAP);
		sensor_query->has_flick =
			!!(query_buf[0] & RMI_F11_HAS_FLICK);
		sensor_query->has_press =
			!!(query_buf[0] & RMI_F11_HAS_PRESS);
		sensor_query->has_pinch =
			!!(query_buf[0] & RMI_F11_HAS_PINCH);
		sensor_query->has_chiral =
			!!(query_buf[0] & RMI_F11_HAS_CHIRAL);

		/* query 8 */
		sensor_query->has_palm_det =
			!!(query_buf[1] & RMI_F11_HAS_PALM_DET);
		sensor_query->has_rotate =
			!!(query_buf[1] & RMI_F11_HAS_ROTATE);
		sensor_query->has_touch_shapes =
			!!(query_buf[1] & RMI_F11_HAS_TOUCH_SHAPES);
		sensor_query->has_scroll_zones =
			!!(query_buf[1] & RMI_F11_HAS_SCROLL_ZONES);
		sensor_query->has_individual_scroll_zones =
			!!(query_buf[1] & RMI_F11_HAS_INDIVIDUAL_SCROLL_ZONES);
		sensor_query->has_mf_scroll =
			!!(query_buf[1] & RMI_F11_HAS_MF_SCROLL);
		sensor_query->has_mf_edge_motion =
			!!(query_buf[1] & RMI_F11_HAS_MF_EDGE_MOTION);
		sensor_query->has_mf_scroll_inertia =
			!!(query_buf[1] & RMI_F11_HAS_MF_SCROLL_INERTIA);

		sensor_query->query7_nonzero = !!(query_buf[0]);
		sensor_query->query8_nonzero = !!(query_buf[1]);

		query_size += 2;
	}

	if (f11->has_query9) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
				    query_buf, 1);
		if (rc < 0)
			return rc;

		sensor_query->has_pen =
			!!(query_buf[0] & RMI_F11_HAS_PEN);
		sensor_query->has_proximity =
			!!(query_buf[0] & RMI_F11_HAS_PROXIMITY);
		sensor_query->has_palm_det_sensitivity =
			!!(query_buf[0] & RMI_F11_HAS_PALM_DET_SENSITIVITY);
		sensor_query->has_suppress_on_palm_detect =
			!!(query_buf[0] & RMI_F11_HAS_SUPPRESS_ON_PALM_DETECT);
		sensor_query->has_two_pen_thresholds =
			!!(query_buf[0] & RMI_F11_HAS_TWO_PEN_THRESHOLDS);
		sensor_query->has_contact_geometry =
			!!(query_buf[0] & RMI_F11_HAS_CONTACT_GEOMETRY);
		sensor_query->has_pen_hover_discrimination =
			!!(query_buf[0] & RMI_F11_HAS_PEN_HOVER_DISCRIMINATION);
		sensor_query->has_pen_filters =
			!!(query_buf[0] & RMI_F11_HAS_PEN_FILTERS);

		query_size++;
	}

	if (sensor_query->has_touch_shapes) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
					query_buf, 1);
		if (rc < 0)
			return rc;

		sensor_query->nr_touch_shapes = query_buf[0] &
				RMI_F11_NR_TOUCH_SHAPES_MASK;

		query_size++;
	}

	if (f11->has_query11) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
				    query_buf, 1);
		if (rc < 0)
			return rc;

		sensor_query->has_z_tuning =
			!!(query_buf[0] & RMI_F11_HAS_Z_TUNING);
		sensor_query->has_algorithm_selection =
			!!(query_buf[0] & RMI_F11_HAS_ALGORITHM_SELECTION);
		sensor_query->has_w_tuning =
			!!(query_buf[0] & RMI_F11_HAS_W_TUNING);
		sensor_query->has_pitch_info =
			!!(query_buf[0] & RMI_F11_HAS_PITCH_INFO);
		sensor_query->has_finger_size =
			!!(query_buf[0] & RMI_F11_HAS_FINGER_SIZE);
		sensor_query->has_segmentation_aggressiveness =
			!!(query_buf[0] & RMI_F11_HAS_SEGMENTATION_AGGRESSIVENESS);
		sensor_query->has_XY_clip =
			!!(query_buf[0] & RMI_F11_HAS_XY_CLIP);
		sensor_query->has_drumming_filter =
			!!(query_buf[0] & RMI_F11_HAS_DRUMMING_FILTER);

		query_size++;
	}

	if (f11->has_query12) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
				    query_buf, 1);
		if (rc < 0)
			return rc;

		sensor_query->has_gapless_finger =
			!!(query_buf[0] & RMI_F11_HAS_GAPLESS_FINGER);
		sensor_query->has_gapless_finger_tuning =
			!!(query_buf[0] & RMI_F11_HAS_GAPLESS_FINGER_TUNING);
		sensor_query->has_8bit_w =
			!!(query_buf[0] & RMI_F11_HAS_8BIT_W);
		sensor_query->has_adjustable_mapping =
			!!(query_buf[0] & RMI_F11_HAS_ADJUSTABLE_MAPPING);
		sensor_query->has_info2 =
			!!(query_buf[0] & RMI_F11_HAS_INFO2);
		sensor_query->has_physical_props =
			!!(query_buf[0] & RMI_F11_HAS_PHYSICAL_PROPS);
		sensor_query->has_finger_limit =
			!!(query_buf[0] & RMI_F11_HAS_FINGER_LIMIT);
		sensor_query->has_linear_coeff_2 =
			!!(query_buf[0] & RMI_F11_HAS_LINEAR_COEFF);

		query_size++;
	}

	if (sensor_query->has_jitter_filter) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
				    query_buf, 1);
		if (rc < 0)
			return rc;

		sensor_query->jitter_window_size = query_buf[0] &
			RMI_F11_JITTER_WINDOW_MASK;
		sensor_query->jitter_filter_type = (query_buf[0] &
			RMI_F11_JITTER_FILTER_MASK) >>
			RMI_F11_JITTER_FILTER_SHIFT;

		query_size++;
	}

	if (f11->has_query12 && sensor_query->has_info2) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
				    query_buf, 1);
		if (rc < 0)
			return rc;

		sensor_query->light_control =
			query_buf[0] & RMI_F11_LIGHT_CONTROL_MASK;
		sensor_query->is_clear =
			!!(query_buf[0] & RMI_F11_IS_CLEAR);
		sensor_query->clickpad_props =
			(query_buf[0] & RMI_F11_CLICKPAD_PROPS_MASK) >>
			RMI_F11_CLICKPAD_PROPS_SHIFT;
		sensor_query->mouse_buttons =
			(query_buf[0] & RMI_F11_MOUSE_BUTTONS_MASK) >>
			RMI_F11_MOUSE_BUTTONS_SHIFT;
		sensor_query->has_advanced_gestures =
			!!(query_buf[0] & RMI_F11_HAS_ADVANCED_GESTURES);

		query_size++;
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
	int device_x_max = le16_to_cpu(*(f11->dev_controls.ctrl0_9 + 6));
	int device_y_max = le16_to_cpu(*(f11->dev_controls.ctrl0_9 + 8));
	int x_min, x_max, y_min, y_max;
	unsigned int input_flags;
	int res_x, res_y;

	/* We assume touchscreen unless demonstrably a touchpad or specified
	 * as a touchpad in the platform data
	 */
	if (sensor->sensor_type == rmi_sensor_touchpad ||
			(sensor->sens_query.has_info2 &&
				!sensor->sens_query.is_clear))
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

	res_x = (x_max - x_min) / sensor->x_mm;
	res_y = (y_max - y_min) / sensor->y_mm;

	input_abs_set_res(input, ABS_X, res_x);
	input_abs_set_res(input, ABS_Y, res_y);

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
	input_abs_set_res(input, ABS_MT_POSITION_X, res_x);
	input_abs_set_res(input, ABS_MT_POSITION_Y, res_y);
	if (!sensor->type_a)
		input_mt_init_slots(input, sensor->nbr_fingers, 0);
	if (IS_ENABLED(CONFIG_RMI4_F11_PEN) && sensor->sens_query.has_pen)
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
	u8 buf;

	f11 = devm_kzalloc(&fn->dev, sizeof(struct f11_data), GFP_KERNEL);
	if (!f11)
		return -ENOMEM;

	fn->data = f11;
	f11->rezero_wait_ms = pdata->f11_rezero_wait;

	query_base_addr = fn->fd.query_base_addr;
	control_base_addr = fn->fd.control_base_addr;

	rc = rmi_read(rmi_dev, query_base_addr, &buf);
	if (rc < 0)
		return rc;

	f11->nr_sensors = buf & RMI_F11_NR_SENSORS_MASK;
	f11->has_query9 = !!(buf & RMI_F11_HAS_QUERY9);
	f11->has_query11 = !!(buf & RMI_F11_HAS_QUERY11);
	f11->has_query12 = !!(buf & RMI_F11_HAS_QUERY12);
	f11->has_query27 = !!(buf & RMI_F11_HAS_QUERY27);
	f11->has_query28 = !!(buf & RMI_F11_HAS_QUERY28);

	query_offset = (query_base_addr + 1);
	/* Increase with one since number of sensors is zero based */
	for (i = 0; i < (f11->nr_sensors + 1); i++) {
		struct f11_2d_sensor *sensor = &f11->sensors[i];
		sensor->sensor_index = i;
		sensor->fn = fn;

		rc = rmi_f11_get_query_parameters(rmi_dev, f11,
				&sensor->sens_query, query_offset);
		if (rc < 0)
			return rc;
		query_offset += rc;

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
			sensor->x_mm = pdata->f11_sensor_data[i].x_mm;
			sensor->y_mm = pdata->f11_sensor_data[i].y_mm;
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
			ctrl->ctrl0_9[RMI_F11_DELTA_X_THRESHOLD] =
				sensor->axis_align.delta_x_threshold;
			rc = rmi_write_block(rmi_dev, ctrl->ctrl0_9_address,
					     ctrl->ctrl0_9, 10);
			if (rc < 0)
				dev_warn(&fn->dev, "Failed to write to delta_x_threshold %d. Code: %d.\n",
					i, rc);

		}

		if (sensor->axis_align.delta_y_threshold) {
			ctrl->ctrl0_9[RMI_F11_DELTA_Y_THRESHOLD] =
				sensor->axis_align.delta_y_threshold;
			rc = rmi_write_block(rmi_dev, ctrl->ctrl0_9_address,
					ctrl->ctrl0_9, 10);
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

	if (!sensor->sens_query.has_gestures)
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
	struct input_dev *input_dev_mouse;
	struct rmi_driver *driver = rmi_dev->driver;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);
	int i;
	struct rmi_driver_data *driver_data = dev_get_drvdata(&rmi_dev->dev);
	int sensors_itertd = 0;
	int rc;
	int board, version;

	board = driver_data->board;
	version = driver_data->rev;

	for (i = 0; i < (f11->nr_sensors + 1); i++) {
		struct f11_2d_sensor *sensor = &f11->sensors[i];
		sensors_itertd = i;
		if (pdata->unified_input_device) {
			input_dev = driver_data->input;
			sensor->input = input_dev;
		} else {
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
		} 

		set_bit(EV_SYN, input_dev->evbit);
		set_bit(EV_ABS, input_dev->evbit);
		input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

		f11_set_abs_params(fn, i);

		if (sensor->sensor_type != rmi_sensor_touchpad && sensor->sens_query.has_rel) {
			set_bit(EV_REL, input_dev->evbit);
			set_bit(REL_X, input_dev->relbit);
			set_bit(REL_Y, input_dev->relbit);
		}

		if (!pdata->unified_input_device) {
			rc = input_register_device(input_dev);
			if (rc < 0) {
				input_free_device(input_dev);
				sensor->input = NULL;
				goto error_unregister;
			}
		}

		if (IS_ENABLED(CONFIG_RMI4_VIRTUAL_BUTTON))
			register_virtual_buttons(fn, sensor);

		if (!pdata->unified_input_device && sensor->sens_query.has_rel) {
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

		if (sensor->sensor_type == rmi_sensor_touchpad) {
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

error_unregister:
	if (!pdata->unified_input_device) {
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
	}

	return rc;
}

static void rmi_f11_free_devices(struct rmi_function *fn)
{
	struct f11_data *f11 = fn->data;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(fn->rmi_dev);
	int i;
	if (!pdata->unified_input_device) {
		for (i = 0; i < (f11->nr_sensors + 1); i++) {
			if (f11->sensors[i].input)
				input_unregister_device(f11->sensors[i].input);
			if (f11->sensors[i].mouse_input)
				input_unregister_device(f11->sensors[i].mouse_input);
		}
	}
}

static int rmi_f11_config(struct rmi_function *fn)
{
	struct f11_data *f11 = fn->data;
	int i;
	int rc;

	for (i = 0; i < (f11->nr_sensors + 1); i++) {
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

	for (i = 0; i < f11->nr_sensors + 1; i++) {
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
	int retval = 0;

	dev_dbg(&fn->dev, "Resuming...\n");
	if (!data->rezero_wait_ms)
		return 0;

	mdelay(data->rezero_wait_ms);

	retval = rmi_write(rmi_dev, fn->fd.command_base_addr, RMI_F11_REZERO);
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
	rmi_f11_free_devices(fn);
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
