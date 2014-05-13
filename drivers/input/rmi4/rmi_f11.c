/*
 * Copyright (c) 2011-2013 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#define FUNCTION_DATA f11_data

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/kconfig.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include "rmi_driver.h"

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
#define NAME_BUFFER_SIZE 256
#define FUNCTION_NUMBER 0x11

/** A note about RMI4 F11 register structure.
 *
 * The properties for
 * a given sensor are described by its query registers.  The number of query
 * registers and the layout of their contents are described by the F11 device
 * queries as well as the sensor query information.
 *
 * Similarly, each sensor has control registers that govern its behavior.  The
 * size and layout of the control registers for a given sensor can be determined
 * by parsing that sensors query registers.
 *
 * And in a likewise fashion, each sensor has data registers where it reports
 * its touch data and other interesting stuff.  The size and layout of a
 * sensors data registers must be determined by parsing its query registers.
 *
 * The short story is that we need to read and parse a lot of query
 * registers in order to determine the attributes of a sensor. Then
 * we need to use that data to compute the size of the control and data
 * registers for sensor.
 *
 * The end result is that we have a number of structs that aren't used to
 * directly generate the input events, but their size, location and contents
 * are critical to determining where the data we are interested in lives.
 *
 * At this time, the driver does not yet comprehend all possible F11
 * configuration options, but it should be sufficient to cover 99% of RMI4 F11
 * devices currently in the field.
 */

/**
 * @rezero - writing this to the F11 command register will cause the sensor to
 * calibrate to the current capacitive state.
 */
#define RMI_F11_REZERO  0x01

#define RMI_F11_HAS_QUERY9              (1 << 3)
#define RMI_F11_HAS_QUERY11             (1 << 4)
#define RMI_F11_HAS_QUERY12             (1 << 5)
#define RMI_F11_HAS_QUERY27             (1 << 6)
#define RMI_F11_HAS_QUERY28             (1 << 7)

/** Defs for Query 1 */

#define RMI_F11_NR_FINGERS_MASK 0x07
#define RMI_F11_HAS_REL                 (1 << 3)
#define RMI_F11_HAS_ABS                 (1 << 4)
#define RMI_F11_HAS_GESTURES            (1 << 5)
#define RMI_F11_HAS_SENSITIVITY_ADJ     (1 << 6)
#define RMI_F11_CONFIGURABLE            (1 << 7)

/** Defs for Query 2, 3, and 4. */
#define RMI_F11_NR_ELECTRODES_MASK      0x7F

/** Defs for Query 5 */

#define RMI_F11_ABS_DATA_SIZE_MASK      0x03
#define RMI_F11_HAS_ANCHORED_FINGER     (1 << 2)
#define RMI_F11_HAS_ADJ_HYST            (1 << 3)
#define RMI_F11_HAS_DRIBBLE             (1 << 4)
#define RMI_F11_HAS_BENDING_CORRECTION  (1 << 5)
#define RMI_F11_HAS_LARGE_OBJECT_SUPPRESSION    (1 << 6)
#define RMI_F11_HAS_JITTER_FILTER       (1 << 7)

/** Defs for Query 7 */
#define RMI_F11_HAS_SINGLE_TAP                  (1 << 0)
#define RMI_F11_HAS_TAP_AND_HOLD                (1 << 1)
#define RMI_F11_HAS_DOUBLE_TAP                  (1 << 2)
#define RMI_F11_HAS_EARLY_TAP                   (1 << 3)
#define RMI_F11_HAS_FLICK                       (1 << 4)
#define RMI_F11_HAS_PRESS                       (1 << 5)
#define RMI_F11_HAS_PINCH                       (1 << 6)
#define RMI_F11_HAS_CHIRAL                      (1 << 7)

/** Defs for Query 8 */
#define RMI_F11_HAS_PALM_DET                    (1 << 0)
#define RMI_F11_HAS_ROTATE                      (1 << 1)
#define RMI_F11_HAS_TOUCH_SHAPES                (1 << 2)
#define RMI_F11_HAS_SCROLL_ZONES                (1 << 3)
#define RMI_F11_HAS_INDIVIDUAL_SCROLL_ZONES     (1 << 4)
#define RMI_F11_HAS_MF_SCROLL                   (1 << 5)
#define RMI_F11_HAS_MF_EDGE_MOTION              (1 << 6)
#define RMI_F11_HAS_MF_SCROLL_INERTIA           (1 << 7)

/** Defs for Query 9. */
#define RMI_F11_HAS_PEN                         (1 << 0)
#define RMI_F11_HAS_PROXIMITY                   (1 << 1)
#define RMI_F11_HAS_PALM_DET_SENSITIVITY        (1 << 2)
#define RMI_F11_HAS_SUPPRESS_ON_PALM_DETECT     (1 << 3)
#define RMI_F11_HAS_TWO_PEN_THRESHOLDS          (1 << 4)
#define RMI_F11_HAS_CONTACT_GEOMETRY            (1 << 5)
#define RMI_F11_HAS_PEN_HOVER_DISCRIMINATION    (1 << 6)
#define RMI_F11_HAS_PEN_FILTERS                 (1 << 7)

/** Defs for Query 10. */
#define RMI_F11_NR_TOUCH_SHAPES_MASK            0x1F

/** Defs for Query 11 */

#define RMI_F11_HAS_Z_TUNING                    (1 << 0)
#define RMI_F11_HAS_ALGORITHM_SELECTION         (1 << 1)
#define RMI_F11_HAS_W_TUNING                    (1 << 2)
#define RMI_F11_HAS_PITCH_INFO                  (1 << 3)
#define RMI_F11_HAS_FINGER_SIZE                 (1 << 4)
#define RMI_F11_HAS_SEGMENTATION_AGGRESSIVENESS (1 << 5)
#define RMI_F11_HAS_XY_CLIP                     (1 << 6)
#define RMI_F11_HAS_DRUMMING_FILTER             (1 << 7)

/** Defs for Query 12. */

#define RMI_F11_HAS_GAPLESS_FINGER              (1 << 0)
#define RMI_F11_HAS_GAPLESS_FINGER_TUNING       (1 << 1)
#define RMI_F11_HAS_8BIT_W                      (1 << 2)
#define RMI_F11_HAS_ADJUSTABLE_MAPPING          (1 << 3)
#define RMI_F11_HAS_INFO2                       (1 << 4)
#define RMI_F11_HAS_PHYSICAL_PROPS              (1 << 5)
#define RMI_F11_HAS_FINGER_LIMIT                (1 << 6)
#define RMI_F11_HAS_LINEAR_COEFF                (1 << 7)

/** Defs for Query 13. */

#define RMI_F11_JITTER_WINDOW_MASK              0x1F
#define RMI_F11_JITTER_FILTER_MASK              0x60
#define RMI_F11_JITTER_FILTER_SHIFT             5

/** Defs for Query 14. */
#define RMI_F11_LIGHT_CONTROL_MASK              0x03
#define RMI_F11_IS_CLEAR                        (1 << 2)
#define RMI_F11_CLICKPAD_PROPS_MASK             0x18
#define RMI_F11_CLICKPAD_PROPS_SHIFT            3
#define RMI_F11_MOUSE_BUTTONS_MASK              0x60
#define RMI_F11_MOUSE_BUTTONS_SHIFT             5
#define RMI_F11_HAS_ADVANCED_GESTURES           (1 << 7)

#define RMI_F11_QUERY_SIZE                      4
#define RMI_F11_QUERY_GESTURE_SIZE              2

#define F11_LIGHT_CTL_NONE 0x00
#define F11_LUXPAD	   0x01
#define F11_DUAL_MODE      0x02

#define F11_NOT_CLICKPAD     0x00
#define F11_HINGED_CLICKPAD  0x01
#define F11_UNIFORM_CLICKPAD 0x02

/**
 * Query registers 1 through 4 are always present.
 *
 * @nr_fingers - describes the maximum number of fingers the 2-D sensor
 * supports.
 * @has_rel - the sensor supports relative motion reporting.
 * @has_abs - the sensor supports absolute poition reporting.
 * @has_gestures - the sensor supports gesture reporting.
 * @has_sensitivity_adjust - the sensor supports a global sensitivity
 * adjustment.
 * @configurable - the sensor supports various configuration options.
 * @num_of_x_electrodes -  the maximum number of electrodes the 2-D sensor
 * supports on the X axis.
 * @num_of_y_electrodes -  the maximum number of electrodes the 2-D sensor
 * supports on the Y axis.
 * @max_electrodes - the total number of X and Y electrodes that may be
 * configured.
 *
 * Query 5 is present if the has_abs bit is set.
 *
 * @abs_data_size - describes the format of data reported by the absolute
 * data source.  Only one format (the kind used here) is supported at this
 * time.
 * @has_anchored_finger - then the sensor supports the high-precision second
 * finger tracking provided by the manual tracking and motion sensitivity
 * options.
 * @has_adjust_hyst - the difference between the finger release threshold and
 * the touch threshold.
 * @has_dribble - the sensor supports the generation of dribble interrupts,
 * which may be enabled or disabled with the dribble control bit.
 * @has_bending_correction - Bending related data registers 28 and 36, and
 * control register 52..57 are present.
 * @has_large_object_suppression - control register 58 and data register 28
 * exist.
 * @has_jitter_filter - query 13 and control 73..76 exist.
 *
 * Gesture information queries 7 and 8 are present if has_gestures bit is set.
 *
 * @has_single_tap - a basic single-tap gesture is supported.
 * @has_tap_n_hold - tap-and-hold gesture is supported.
 * @has_double_tap - double-tap gesture is supported.
 * @has_early_tap - early tap is supported and reported as soon as the finger
 * lifts for any tap event that could be interpreted as either a single tap
 * or as the first tap of a double-tap or tap-and-hold gesture.
 * @has_flick - flick detection is supported.
 * @has_press - press gesture reporting is supported.
 * @has_pinch - pinch gesture detection is supported.
 * @has_palm_det - the 2-D sensor notifies the host whenever a large conductive
 * object such as a palm or a cheek touches the 2-D sensor.
 * @has_rotate - rotation gesture detection is supported.
 * @has_touch_shapes - TouchShapes are supported.  A TouchShape is a fixed
 * rectangular area on the sensor that behaves like a capacitive button.
 * @has_scroll_zones - scrolling areas near the sensor edges are supported.
 * @has_individual_scroll_zones - if 1, then 4 scroll zones are supported;
 * if 0, then only two are supported.
 * @has_mf_scroll - the multifinger_scrolling bit will be set when
 * more than one finger is involved in a scrolling action.
 *
 * Convenience for checking bytes in the gesture info registers.  This is done
 * often enough that we put it here to declutter the conditionals
 *
 * @query7_nonzero - true if none of the query 7 bits are set
 * @query8_nonzero - true if none of the query 8 bits are set
 *
 * Query 9 is present if the has_query9 is set.
 *
 * @has_pen - detection of a stylus is supported and registers F11_2D_Ctrl20
 * and F11_2D_Ctrl21 exist.
 * @has_proximity - detection of fingers near the sensor is supported and
 * registers F11_2D_Ctrl22 through F11_2D_Ctrl26 exist.
 * @has_palm_det_sensitivity -  the sensor supports the palm detect sensitivity
 * feature and register F11_2D_Ctrl27 exists.
 * @has_two_pen_thresholds - is has_pen is also set, then F11_2D_Ctrl35 exists.
 * @has_contact_geometry - the sensor supports the use of contact geometry to
 * map absolute X and Y target positions and registers F11_2D_Data18
 * through F11_2D_Data27 exist.
 *
 * Touch shape info (query 10) is present if has_touch_shapes is set.
 *
 * @nr_touch_shapes - the total number of touch shapes supported.
 *
 * Query 11 is present if the has_query11 bit is set in query 0.
 *
 * @has_z_tuning - if set, the sensor supports Z tuning and registers
 * F11_2D_Ctrl29 through F11_2D_Ctrl33 exist.
 * @has_algorithm_selection - controls choice of noise suppression algorithm
 * @has_w_tuning - the sensor supports Wx and Wy scaling and registers
 * F11_2D_Ctrl36 through F11_2D_Ctrl39 exist.
 * @has_pitch_info - the X and Y pitches of the sensor electrodes can be
 * configured and registers F11_2D_Ctrl40 and F11_2D_Ctrl41 exist.
 * @has_finger_size -  the default finger width settings for the
 * sensor can be configured and registers F11_2D_Ctrl42 through F11_2D_Ctrl44
 * exist.
 * @has_segmentation_aggressiveness - the sensor’s ability to distinguish
 * multiple objects close together can be configured and register F11_2D_Ctrl45
 * exists.
 * @has_XY_clip -  the inactive outside borders of the sensor can be
 * configured and registers F11_2D_Ctrl46 through F11_2D_Ctrl49 exist.
 * @has_drumming_filter - the sensor can be configured to distinguish
 * between a fast flick and a quick drumming movement and registers
 * F11_2D_Ctrl50 and F11_2D_Ctrl51 exist.
 *
 * Query 12 is present if hasQuery12 bit is set.
 *
 * @has_gapless_finger - control registers relating to gapless finger are
 * present.
 * @has_gapless_finger_tuning - additional control and data registers relating
 * to gapless finger are present.
 * @has_8bit_w - larger W value reporting is supported.
 * @has_adjustable_mapping - TBD
 * @has_info2 - the general info query14 is present
 * @has_physical_props - additional queries describing the physical properties
 * of the sensor are present.
 * @has_finger_limit - indicates that F11 Ctrl 80 exists.
 * @has_linear_coeff - indicates that F11 Ctrl 81 exists.
 *
 * Query 13 is present if Query 5's has_jitter_filter bit is set.
 * @jitter_window_size - used by Design Studio 4.
 * @jitter_filter_type - used by Design Studio 4.
 *
 * Query 14 is present if query 12's has_general_info2 flag is set.
 *
 * @light_control - Indicates what light/led control features are present, if
 * any.
 * @is_clear - if set, this is a clear sensor (indicating direct pointing
 * application), otherwise it's opaque (indicating indirect pointing).
 * @clickpad_props - specifies if this is a clickpad, and if so what sort of
 * mechanism it uses
 * @mouse_buttons - specifies the number of mouse buttons present (if any).
 * @has_advanced_gestures - advanced driver gestures are supported.
 */
struct f11_2d_sensor_queries {
	/* query1 */
	u8 nr_fingers;
	bool has_rel;
	bool has_abs;
	bool has_gestures;
	bool has_sensitivity_adjust;
	bool configurable;

	/* query2 */
	u8 nr_x_electrodes;

	/* query3 */
	u8 nr_y_electrodes;

	/* query4 */
	u8 max_electrodes;

	/* query5 */
	u8 abs_data_size;
	bool has_anchored_finger;
	bool has_adj_hyst;
	bool has_dribble;
	bool has_bending_correction;
	bool has_large_object_suppression;
	bool has_jitter_filter;

	u8 f11_2d_query6;

	/* query 7 */
	bool has_single_tap;
	bool has_tap_n_hold;
	bool has_double_tap;
	bool has_early_tap;
	bool has_flick;
	bool has_press;
	bool has_pinch;
	bool has_chiral;

	bool query7_nonzero;

	/* query 8 */
	bool has_palm_det;
	bool has_rotate;
	bool has_touch_shapes;
	bool has_scroll_zones;
	bool has_individual_scroll_zones;
	bool has_mf_scroll;
	bool has_mf_edge_motion;
	bool has_mf_scroll_inertia;

	bool query8_nonzero;

	/* Query 9 */
	bool has_pen;
	bool has_proximity;
	bool has_palm_det_sensitivity;
	bool has_suppress_on_palm_detect;
	bool has_two_pen_thresholds;
	bool has_contact_geometry;
	bool has_pen_hover_discrimination;
	bool has_pen_filters;

	/* Query 10 */
	u8 nr_touch_shapes;

	/* Query 11. */
	bool has_z_tuning;
	bool has_algorithm_selection;
	bool has_w_tuning;
	bool has_pitch_info;
	bool has_finger_size;
	bool has_segmentation_aggressiveness;
	bool has_XY_clip;
	bool has_drumming_filter;

	/* Query 12 */
	bool has_gapless_finger;
	bool has_gapless_finger_tuning;
	bool has_8bit_w;
	bool has_adjustable_mapping;
	bool has_info2;
	bool has_physical_props;
	bool has_finger_limit;
	bool has_linear_coeff_2;

	/* Query 13 */
	u8 jitter_window_size;
	u8 jitter_filter_type;

	/* Query 14 */
	u8 light_control;
	bool is_clear;
	u8 clickpad_props;
	u8 mouse_buttons;
	bool has_advanced_gestures;
};

/* Defs for Ctrl0. */
#define RMI_F11_REPORT_MODE_MASK        0x07
#define RMI_F11_ABS_POS_FILT            (1 << 3)
#define RMI_F11_REL_POS_FILT            (1 << 4)
#define RMI_F11_REL_BALLISTICS          (1 << 5)
#define RMI_F11_DRIBBLE                 (1 << 6)
#define RMI_F11_REPORT_BEYOND_CLIP      (1 << 7)

/* Defs for Ctrl1. */
#define RMI_F11_PALM_DETECT_THRESH_MASK 0x0F
#define RMI_F11_MOTION_SENSITIVITY_MASK 0x30
#define RMI_F11_MANUAL_TRACKING         (1 << 6)
#define RMI_F11_MANUAL_TRACKED_FINGER   (1 << 7)

#define RMI_F11_DELTA_X_THRESHOLD       2
#define RMI_F11_DELTA_Y_THRESHOLD       3

#define RMI_F11_CTRL_REG_COUNT          10

struct f11_2d_ctrl {
	u8              ctrl0_9[RMI_F11_CTRL_REG_COUNT];
	u16             ctrl0_9_address;
};

#define RMI_F11_ABS_BYTES 5
#define RMI_F11_REL_BYTES 2

/* Defs for Data 8 */

#define RMI_F11_SINGLE_TAP              (1 << 0)
#define RMI_F11_TAP_AND_HOLD            (1 << 1)
#define RMI_F11_DOUBLE_TAP              (1 << 2)
#define RMI_F11_EARLY_TAP               (1 << 3)
#define RMI_F11_FLICK                   (1 << 4)
#define RMI_F11_PRESS                   (1 << 5)
#define RMI_F11_PINCH                   (1 << 6)

/* Defs for Data 9 */

#define RMI_F11_PALM_DETECT                     (1 << 0)
#define RMI_F11_ROTATE                          (1 << 1)
#define RMI_F11_SHAPE                           (1 << 2)
#define RMI_F11_SCROLLZONE                      (1 << 3)
#define RMI_F11_GESTURE_FINGER_COUNT_MASK       0x70

/** Handy pointers into our data buffer.
 *
 * @f_state - start of finger state registers.
 * @abs_pos - start of absolute position registers (if present).
 * @rel_pos - start of relative data registers (if present).
 * @gest_1  - gesture flags (if present).
 * @gest_2  - gesture flags & finger count (if present).
 * @pinch   - pinch motion register (if present).
 * @flick   - flick distance X & Y, flick time (if present).
 * @rotate  - rotate motion and finger separation.
 * @multi_scroll - chiral deltas for X and Y (if present).
 * @scroll_zones - scroll deltas for 4 regions (if present).
 */
struct f11_2d_data {
	u8	*f_state;
	u8	*abs_pos;
	s8	*rel_pos;
	u8	*gest_1;
	u8	*gest_2;
	s8	*pinch;
	u8	*flick;
	u8	*rotate;
	u8	*shapes;
	s8	*multi_scroll;
	s8	*scroll_zones;
};

/**
 * @axis_align - controls parameters that are useful in system prototyping
 * and bring up.
 * @sens_query - query registers for this particular sensor.
 * @data - the data reported by this sensor, mapped into a collection of
 * structs.
 * @max_x - The maximum X coordinate that will be reported by this sensor.
 * @max_y - The maximum Y coordinate that will be reported by this sensor.
 * @nbr_fingers - How many fingers can this sensor report?
 * @data_pkt - buffer for data reported by this sensor.
 * @pkt_size - number of bytes in that buffer.
 * @sensor_index - identifies this particular 2D touch sensor
 * @type_a - some early RMI4 2D sensors do not reliably track the finger
 * position when two fingers are on the device.  When this is true, we
 * assume we have one of those sensors and report events appropriately.
 * @sensor_type - indicates whether we're touchscreen or touchpad.
 * @input - input device for absolute pointing stream
 * @mouse_input - input device for relative pointing stream.
 * @input_phys - buffer for the absolute phys name for this sensor.
 * @input_phys_mouse - buffer for the relative phys name for this sensor.
 */
struct f11_2d_sensor {
	struct rmi_f11_2d_axis_alignment axis_align;
	struct f11_2d_sensor_queries sens_query;
	struct f11_2d_data data;
	u16 max_x;
	u16 max_y;
	u8 nbr_fingers;
	u8 *data_pkt;
	int pkt_size;
	u8 sensor_index;
	u32 type_a;	/* boolean but debugfs API requires u32 */
	enum rmi_f11_sensor_type sensor_type;
	struct input_dev *input;
	struct input_dev *mouse_input;
	struct rmi_function *fn;
	char input_phys[NAME_BUFFER_SIZE];
	char input_phys_mouse[NAME_BUFFER_SIZE];
};

/** Data pertaining to F11 in general.  For per-sensor data, see struct
 * f11_2d_sensor.
 *
 * @dev_query - F11 device specific query registers.
 * @dev_controls - F11 device specific control registers.
 * @dev_controls_mutex - lock for the control registers.
 * @rezero_wait_ms - if nonzero, upon resume we will wait this many
 * milliseconds before rezeroing the sensor(s).  This is useful in systems with
 * poor electrical behavior on resume, where the initial calibration of the
 * sensor(s) coming out of sleep state may be bogus.
 * @sensors - per sensor data structures.
 */
struct f11_data {
	bool has_query9;
	bool has_query11;
	bool has_query12;
	bool has_query27;
	bool has_query28;
	struct f11_2d_ctrl dev_controls;
	struct mutex dev_controls_mutex;
	u16 rezero_wait_ms;
	struct f11_2d_sensor sensor;
};

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
			sensor->sens_query.has_pen &&
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
		input_report_rel(sensor->mouse_input, REL_X, x);
		input_report_rel(sensor->mouse_input, REL_Y, y);
	}
	input_sync(sensor->mouse_input);
}

static void rmi_f11_abs_pos_report(struct f11_data *f11,
				   struct f11_2d_sensor *sensor,
				   u8 finger_state, u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_f11_2d_axis_alignment *axis_align = &sensor->axis_align;
	u16 x, y, z;
	int w_x, w_y, w_max, w_min, orient;
	u8 abs_base = n_finger * RMI_F11_ABS_BYTES;

	x = y = z = 0;
	w_x = w_y = w_max = w_min = orient = 0;

	if (finger_state) {
		x = (data->abs_pos[abs_base] << 4) |
			(data->abs_pos[abs_base + 2] & 0x0F);
		y = (data->abs_pos[abs_base + 1] << 4) |
			(data->abs_pos[abs_base + 2] >> 4);
		w_x = data->abs_pos[abs_base + 3] & 0x0F;
		w_y = data->abs_pos[abs_base + 3] >> 4;
		w_max = max(w_x, w_y);
		w_min = min(w_x, w_y);
		z = data->abs_pos[abs_base + 4];

		if (axis_align->swap_axes) {
			int temp;
			temp = x;
			x = y;
			y = temp;
			temp = w_x;
			w_x = w_y;
			w_y = temp;
		}

		orient = w_x > w_y ? 1 : 0;

		if (axis_align->flip_x)
			x = max(sensor->max_x - x, 0);

		if (axis_align->flip_y)
			y = max(sensor->max_y - y, 0);

		/*
		* here checking if X offset or y offset are specified is
		*  redundant.  We just add the offsets or, clip the values
		*
		* note: offsets need to be done before clipping occurs,
		* or we could get funny values that are outside
		* clipping boundaries.
		*/
		x += axis_align->offset_x;
		y += axis_align->offset_y;
		x =  max(axis_align->clip_x_low, x);
		y =  max(axis_align->clip_y_low, y);
		if (axis_align->clip_x_high)
			x = min(axis_align->clip_x_high, x);
		if (axis_align->clip_y_high)
			y =  min(axis_align->clip_y_high, y);

	}

	/* Some UIs ignore W of zero, so we fudge it to 1 for pens.  This
	 * only appears to be an issue when reporting pens, not plain old
	 * fingers. */
	if (IS_ENABLED(CONFIG_RMI4_F11_PEN) &&
			get_tool_type(sensor, finger_state) == MT_TOOL_PEN) {
		w_max = max(1, w_max);
		w_min = max(1, w_min);
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
		input_report_abs(sensor->input, ABS_MT_PRESSURE, z);
		input_report_abs(sensor->input, ABS_MT_TOUCH_MAJOR, w_max);
		input_report_abs(sensor->input, ABS_MT_TOUCH_MINOR, w_min);
		input_report_abs(sensor->input, ABS_MT_ORIENTATION, orient);
		input_report_abs(sensor->input, ABS_MT_POSITION_X, x);
		input_report_abs(sensor->input, ABS_MT_POSITION_Y, y);
		dev_dbg(&sensor->fn->dev,
			"finger[%d]:%d - x:%d y:%d z:%d w_max:%d w_min:%d\n",
			n_finger, finger_state, x, y, z, w_max, w_min);
	}
	/* MT sync between fingers */
	if (sensor->type_a)
		input_mt_sync(sensor->input);
}

static void rmi_f11_finger_handler(struct f11_data *f11,
				   struct f11_2d_sensor *sensor)
{
	const u8 *f_state = sensor->data.f_state;
	u8 finger_state;
	u8 finger_pressed_count;
	u8 i;

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

		if (sensor->data.abs_pos)
			rmi_f11_abs_pos_report(f11, sensor, finger_state, i);

		if (sensor->data.rel_pos)
			rmi_f11_rel_pos_report(sensor, i);
	}
//	input_mt_sync(sensor->input);

	input_report_key(sensor->input, BTN_TOUCH, finger_pressed_count);

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

	if (query->has_abs)
		sensor->pkt_size += (sensor->nbr_fingers * 5);

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
	int error = 0;

	ctrl->ctrl0_9_address = ctrl_base_addr;
	error = rmi_read_block(rmi_dev, ctrl_base_addr, ctrl->ctrl0_9,
				RMI_F11_CTRL_REG_COUNT);
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

	error = rmi_write_block(rmi_dev, ctrl_base_addr, ctrl->ctrl0_9,
				RMI_F11_CTRL_REG_COUNT);
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
	u8 query_buf[RMI_F11_QUERY_SIZE];

	rc = rmi_read_block(rmi_dev, query_base_addr, query_buf,
				RMI_F11_QUERY_SIZE);
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

	query_size = RMI_F11_QUERY_SIZE;

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
					query_buf, RMI_F11_QUERY_GESTURE_SIZE);
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
		rc = rmi_read(rmi_dev, query_base_addr + query_size, query_buf);
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
		rc = rmi_read(rmi_dev, query_base_addr + query_size, query_buf);
		if (rc < 0)
			return rc;

		sensor_query->nr_touch_shapes = query_buf[0] &
				RMI_F11_NR_TOUCH_SHAPES_MASK;

		query_size++;
	}

	if (f11->has_query11) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size, query_buf);
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
			!!(query_buf[0] &
				RMI_F11_HAS_SEGMENTATION_AGGRESSIVENESS);
		sensor_query->has_XY_clip =
			!!(query_buf[0] & RMI_F11_HAS_XY_CLIP);
		sensor_query->has_drumming_filter =
			!!(query_buf[0] & RMI_F11_HAS_DRUMMING_FILTER);

		query_size++;
	}

	if (f11->has_query12) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size, query_buf);
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
		rc = rmi_read(rmi_dev, query_base_addr + query_size, query_buf);
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
		rc = rmi_read(rmi_dev, query_base_addr + query_size, query_buf);
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
static void f11_set_abs_params(struct rmi_function *fn)
{
	struct f11_data *f11 = fn->data;
	struct f11_2d_sensor *sensor = &f11->sensor;
	struct input_dev *input = sensor->input;
	/* These two lines are not doing what we want them to.  So we use
	 * some shifts instead.
	int device_x_max = le16_to_cpu(*(f11->dev_controls.ctrl0_9 + 6));
	int device_y_max = le16_to_cpu(*(f11->dev_controls.ctrl0_9 + 8));
	 */
	u16 device_x_max = f11->dev_controls.ctrl0_9[6] |
			((f11->dev_controls.ctrl0_9[7] & 0x0F) << 8);
	u16 device_y_max = f11->dev_controls.ctrl0_9[8] |
			((f11->dev_controls.ctrl0_9[9] & 0x0F) << 8);
	u16 x_min, x_max, y_min, y_max;
	unsigned int input_flags;

	/* We assume touchscreen unless demonstrably a touchpad or specified
	 * as a touchpad in the platform data
	 */
	if (sensor->sensor_type == rmi_f11_sensor_touchpad ||
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
	x_min = sensor->axis_align.clip_x_low;
	if (sensor->axis_align.clip_x_high)
		x_max = min(device_x_max,
			sensor->axis_align.clip_x_high);
	else
		x_max = device_x_max;

	y_min = sensor->axis_align.clip_y_low;
	if (sensor->axis_align.clip_y_high)
		y_max = min(device_y_max,
			sensor->axis_align.clip_y_high);
	else
		y_max = device_y_max;

	dev_dbg(&fn->dev, "Set ranges X=[%d..%d] Y=[%d..%d].",
			x_min, x_max, y_min, y_max);

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
	if (!sensor->type_a)
		input_mt_init_slots(input, sensor->nbr_fingers);
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
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);
	struct f11_2d_sensor *sensor;
	u8 buf;

	dev_dbg(&fn->dev, "Initializing F11 values for %s.\n",
		 pdata->sensor_name);

	/*
	** init instance data, fill in values and create any sysfs files
	*/
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

	f11->has_query9 = !!(buf & RMI_F11_HAS_QUERY9);
	f11->has_query11 = !!(buf & RMI_F11_HAS_QUERY11);
	f11->has_query12 = !!(buf & RMI_F11_HAS_QUERY12);
	f11->has_query27 = !!(buf & RMI_F11_HAS_QUERY27);
	f11->has_query28 = !!(buf & RMI_F11_HAS_QUERY28);

	query_offset = (query_base_addr + 1);
	sensor = &f11->sensor;
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

	if (pdata->f11_sensor_data) {
		sensor->axis_align =
			pdata->f11_sensor_data->axis_align;
		sensor->type_a = pdata->f11_sensor_data->type_a;
		sensor->sensor_type =
				pdata->f11_sensor_data->sensor_type;
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

	rc = f11_2d_construct_data(sensor);
	if (rc < 0)
		return rc;

	ctrl = &f11->dev_controls;
	if (sensor->axis_align.delta_x_threshold) {
		ctrl->ctrl0_9[RMI_F11_DELTA_X_THRESHOLD] =
			sensor->axis_align.delta_x_threshold;
		rc = rmi_write_block(rmi_dev, ctrl->ctrl0_9_address,
				ctrl->ctrl0_9,
				RMI_F11_CTRL_REG_COUNT);
		if (rc < 0)
			dev_warn(&fn->dev, "Failed to write to delta_x_threshold. Code: %d.\n",
				rc);

	}

	if (sensor->axis_align.delta_y_threshold) {
		ctrl->ctrl0_9[RMI_F11_DELTA_Y_THRESHOLD] =
			sensor->axis_align.delta_y_threshold;
		rc = rmi_write_block(rmi_dev, ctrl->ctrl0_9_address,
				ctrl->ctrl0_9, RMI_F11_CTRL_REG_COUNT);
		if (rc < 0)
			dev_warn(&fn->dev, "Failed to write to delta_y_threshold. Code: %d.\n",
				rc);
	}

	mutex_init(&f11->dev_controls_mutex);
	return 0;
}

static int rmi_f11_register_devices(struct rmi_function *fn)
{
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f11_data *f11 = fn->data;
	struct input_dev *input_dev;
	struct input_dev *input_dev_mouse;
	struct rmi_driver *driver = rmi_dev->driver;
	struct f11_2d_sensor *sensor = &f11->sensor;
	int rc;

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
	sprintf(sensor->input_phys, "%s.abs/input0",
		dev_name(&fn->dev));
	input_dev->phys = sensor->input_phys;
	input_dev->dev.parent = &rmi_dev->dev;
	input_set_drvdata(input_dev, f11);

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	f11_set_abs_params(fn);

	if (sensor->sens_query.has_rel) {
		set_bit(EV_REL, input_dev->evbit);
		set_bit(REL_X, input_dev->relbit);
		set_bit(REL_Y, input_dev->relbit);
	}
	rc = input_register_device(input_dev);
	if (rc < 0) {
		input_free_device(input_dev);
		sensor->input = NULL;
		goto error_unregister;
	}

	if (sensor->sens_query.has_rel) {
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
		sprintf(sensor->input_phys_mouse, "%s.rel/input0",
			dev_name(&fn->dev));
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

	return 0;

error_unregister:
	if (f11->sensor.input) {
		if (f11->sensor.mouse_input) {
			input_unregister_device(f11->sensor.mouse_input);
			f11->sensor.mouse_input = NULL;
		}
		input_unregister_device(f11->sensor.input);
		f11->sensor.input = NULL;
	}

	return rc;
}

static void rmi_f11_free_devices(struct rmi_function *fn)
{
	struct f11_data *f11 = fn->data;

	if (f11->sensor.input)
		input_unregister_device(f11->sensor.input);
	if (f11->sensor.mouse_input)
		input_unregister_device(f11->sensor.mouse_input);
}

static int rmi_f11_config(struct rmi_function *fn)
{
	struct f11_data *f11 = fn->data;
	int rc;

	rc = f11_write_control_regs(fn, &f11->sensor.sens_query,
			   &f11->dev_controls, fn->fd.query_base_addr);
	if (rc < 0)
		return rc;

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

	error = rmi_read_block(rmi_dev,
			data_base_addr + data_base_addr_offset,
			f11->sensor.data_pkt,
			f11->sensor.pkt_size);
	if (error < 0)
		return error;

	rmi_f11_finger_handler(f11, &f11->sensor);
	data_base_addr_offset += f11->sensor.pkt_size;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rmi_f11_resume(struct device *dev)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f11_data *data = fn->data;
	int error;

	dev_dbg(&fn->dev, "Resuming...\n");
	if (!data->rezero_wait_ms)
		return 0;

	mdelay(data->rezero_wait_ms);

	error = rmi_write(rmi_dev, fn->fd.command_base_addr, RMI_F11_REZERO);
	if (error < 0) {
		dev_err(&fn->dev,
			"%s: failed to issue rezero command, error = %d.",
			__func__, error);
		return error;
	}

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(rmi_f11_pm_ops, NULL, rmi_f11_resume);

static int rmi_f11_probe(struct rmi_function *fn)
{
	int error;

	error = rmi_f11_initialize(fn);
	if (error)
		return error;

	error = rmi_f11_register_devices(fn);
	if (error)
		return error;

	return 0;
}

static void rmi_f11_remove(struct rmi_function *fn)
{
	rmi_f11_free_devices(fn);
}

static struct rmi_function_handler rmi_f11_handler = {
	.driver = {
		.name	= "rmi_f11",
		.pm	= &rmi_f11_pm_ops,
	},
	.func		= 0x11,
	.probe		= rmi_f11_probe,
	.remove		= rmi_f11_remove,
	.config		= rmi_f11_config,
	.attention	= rmi_f11_attention,
};

module_rmi_driver(rmi_f11_handler);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com");
MODULE_DESCRIPTION("RMI F11 module");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
