/*
 * Copyright (c) 2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef RMI_F11_CONTROL_H
#define RMI_F11_CONTROL_H

#define F11_MAX_NUM_OF_SENSORS 8

#define NAME_BUFFER_SIZE 256

/** A note about RMI4 F11 register structure.
 *
 *  There may be one or more individual 2D touch surfaces associated with an
 * instance for F11.  For example, a handheld device might have a touchscreen
 * display on the front, and a touchpad on the back.  F11 represents these touch
 * surfaces as individual sensors, up to 7 on a given RMI4 device.
 *
 * The properties for
 * a given sensor are described by its query registers.  The number of query
 * registers and the layout of their contents are described by the F11 device
 * queries as well as the per-sensor query information.  The query registers
 * for sensor[n+1] immediately follow those for sensor[n], so the start address
 * of the sensor[n+1] queries can only be computed if you know the size of the
 * sensor[n] queries.  Because each of the sensors may have different
 * properties, the size of the query registers for each sensor must be
 * calculated on a sensor by sensor basis.
 *
 * Similarly, each sensor has control registers that govern its behavior.  The
 * size and layout of the control registers for a given sensor can be determined
 * by parsing that sensors query registers.  The control registers for
 * sensor[n+1] immediately follow those for sensor[n], so you can only know
 * the start address for the sensor[n+1] controls if you know the size (and
 * location) of the sensor[n] controls.
 *
 * And in a likewise fashion, each sensor has data registers where it reports
 * its touch data and other interesting stuff.  The size and layout of a
 * sensors data registers must be determined by parsing its query registers.
 * The data registers for sensor[n+1] immediately follow those for sensor[n],
 * so you can only know the start address for the sensor[n+1] controls if you
 * know the size (and location) of the sensor[n] controls.
 *
 * The short story is that we need to read and parse a lot of query
 * registers in order to determine the attributes of a sensor[0].  Then
 * we need to use that data to compute the size of the control and data
 * registers for sensor[0].  Once we have that figured out, we can then do
 * the same thing for each subsequent sensor.
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
 * @rezero - writing 1 to this will cause the sensor to calibrate to the
 * current capacitive state.
 */
struct f11_2d_commands {
	u8 rezero:1;
	u8 reserved:7;
} __attribute__((__packed__));

/** This query is always present, and is on a per device basis.  All other
 * queries are on a per-sensor basis.
 *
 * @nbr_of_sensors - the number of 2D sensors on the touch device.
 * @has_query9 - indicates the F11_2D_Query9 register exists.
 * @has_query11 - indicates the F11_2D_Query11 register exists.
 * @has_query12 - indicates the F11_2D_Query12 register exists.
 */
struct f11_2d_device_query {
	u8 nbr_of_sensors:3;
	u8 has_query9:1;
	u8 has_query11:1;
	u8 has_query12:1;
	u8 has_query27:1;
	u8 has_query28:1;
} __attribute__((__packed__));

/** Query registers 1 through 4 are always present.
 * @number_of_fingers - describes the maximum number of fingers the 2-D sensor
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
 */
struct f11_2d_sensor_info {
	/* query1 */
	u8 number_of_fingers:3;
	u8 has_rel:1;
	u8 has_abs:1;
	u8 has_gestures:1;
	u8 has_sensitivity_adjust:1;
	u8 configurable:1;
	/* query2 */
	u8 num_of_x_electrodes:7;
	u8 reserved_1:1;
	/* query3 */
	u8 num_of_y_electrodes:7;
	u8 reserved_2:1;
	/* query4 */
	u8 max_electrodes:7;
	u8 reserved_3:1;
} __attribute__((__packed__));

/** Query 5 - this is present if the has_abs bit is set.
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
 */
struct f11_2d_abs_info {
	u8 abs_data_size:2;
	u8 has_anchored_finger:1;
	u8 has_adj_hyst:1;
	u8 has_dribble:1;
	u8 has_bending_correction:1;
	u8 has_large_object_suppression:1;
	u8 has_jitter_filter:1;
} __attribute__((__packed__));

/** Gesture information queries 7 and 8 are present if has_gestures bit is set.
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
 * @has_multi_finger_scroll - the multifinger_scrolling bit will be set when
 * more than one finger is involved in a scrolling action.
 */
struct f11_2d_gesture_info {
	u8 has_single_tap:1;
	u8 has_tap_n_hold:1;
	u8 has_double_tap:1;
	u8 has_early_tap:1;
	u8 has_flick:1;
	u8 has_press:1;
	u8 has_pinch:1;
	u8 has_chiral:1;

	u8 has_palm_det:1;
	u8 has_rotate:1;
	u8 has_touch_shapes:1;
	u8 has_scroll_zones:1;
	u8 has_individual_scroll_zones:1;
	u8 has_multi_finger_scroll:1;
	u8 has_mf_edge_motion:1;
	u8 has_mf_scroll_inertia:1;
} __attribute__((__packed__));

/**
 * @has_pen - detection of a stylus is supported and registers F11_2D_Ctrl20
 * and F11_2D_Ctrl21 exist.
 * @has_proximity - detection of fingers near the sensor is supported and
 * registers F11_2D_Ctrl22 through F11_2D_Ctrl26 exist.
 * @has_palm_det_sensitivity -  the sensor supports the palm detect sensitivity
 * feature and register F11_2D_Ctrl27 exists.
 * @has_two_pen_thresholds - is has_pen is also set, then F11_2D_Ctrl35 exists.
 * @has_contact_geometry - the sensor supports the use of contact geometry to
 * map absolute X and Y target positions and registers F11_2D_Data18.* through
 * F11_2D_Data27 exist.
 */
struct f11_2d_query9 {
	u8 has_pen:1;
	u8 has_proximity:1;
	u8 has_palm_det_sensitivity:1;
	u8 has_suppress_on_palm_detect:1;
	u8 has_two_pen_thresholds:1;
	u8 has_contact_geometry:1;
	u8 has_pen_hover_discrimination:1;
	u8 has_pen_filters:1;
} __attribute__((__packed__));

/** Touch shape info (query 10) is present if has_touch_shapes is set.
 *
 * @nbr_touch_shapes - the total number of touch shapes supported.
 */
struct f11_2d_ts_info {
	u8 nbr_touch_shapes:5;
	u8 reserved:3;
} __attribute__((__packed__));

/** Query 11 is present if the has_query11 bit is set in query 0.
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
 */
struct f11_2d_query11 {
	u8 has_z_tuning:1;
	u8 has_algorithm_selection:1;
	u8 has_w_tuning:1;
	u8 has_pitch_info:1;
	u8 has_finger_size:1;
	u8 has_segmentation_aggressiveness:1;
	u8 has_XY_clip:1;
	u8 has_drumming_filter:1;
} __attribute__((__packed__));

/**
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
 */
struct f11_2d_query12 {
	u8 has_gapless_finger:1;
	u8 has_gapless_finger_tuning:1;
	u8 has_8bit_w:1;
	u8 has_adjustable_mapping:1;
	u8 has_info2:1;
	u8 has_physical_props:1;
	u8 has_finger_limit:1;
	u8 has_linear_coeff_2:1;
} __attribute__((__packed__));

/** This register is present if Query 5's has_jitter_filter bit is set.
 * @jitter_window_size - used by Design Studio 4.
 * @jitter_filter_type - used by Design Studio 4.
 */
struct f11_2d_query13 {
	u8 jtter_window_size:5;
	u8 jitter_filter_type:2;
	u8 reserved:1;
} __attribute__((__packed__));

/** This register is present if query 12's has_general_info2 flag is set.
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
struct f11_2d_query14 {
	u8 light_control:2;
	u8 is_clear:1;
	u8 clickpad_props:2;
	u8 mouse_buttons:2;
	u8 has_advanced_gestures:1;
} __attribute__((__packed__));

#define F11_LIGHT_CTL_NONE 0x00
#define F11_LUXPAD	   0x01
#define F11_DUAL_MODE      0x02

#define F11_NOT_CLICKPAD     0x00
#define F11_HINGED_CLICKPAD  0x01
#define F11_UNIFORM_CLICKPAD 0x02

/** See notes above for information about specific query register sets.
 */
struct f11_2d_sensor_queries {
	struct f11_2d_sensor_info info;
	struct f11_2d_abs_info abs_info;
	u8 f11_2d_query6;
	struct f11_2d_gesture_info gesture_info;
	struct f11_2d_query9 query9;
	struct f11_2d_ts_info ts_info;
	struct f11_2d_query11 features_1;
	struct f11_2d_query12 features_2;
	struct f11_2d_query13 jitter_filter;
	struct f11_2d_query14 info_2;
};

/**
 * @reporting_mode - controls how often finger position data is reported.
 * @abs_pos_filt - when set, enables various noise and jitter filtering
 * algorithms for absolute reports.
 * @rel_pos_filt - when set, enables various noise and jitter filtering
 * algorithms for relative reports.
 * @rel_ballistics - enables ballistics processing for the relative finger
 * motion on the 2-D sensor.
 * @dribble - enables the dribbling feature.
 * @report_beyond_clip - when this is set, fingers outside the active area
 * specified by the x_clip and y_clip registers will be reported, but with
 * reported finger position clipped to the edge of the active area.
 * @palm_detect_thresh - the threshold at which a wide finger is considered a
 * palm. A value of 0 inhibits palm detection.
 * @motion_sensitivity - specifies the threshold an anchored finger must move
 * before it is considered no longer anchored.  High values mean more
 * sensitivity.
 * @man_track_en - for anchored finger tracking, whether the host (1) or the
 * device (0) determines which finger is the tracked finger.
 * @man_tracked_finger - when man_track_en is 1, specifies whether finger 0 or
 * finger 1 is the tracked finger.
 * @delta_x_threshold - 2-D position update interrupts are inhibited unless
 * the finger moves more than a certain threshold distance along the X axis.
 * @delta_y_threshold - 2-D position update interrupts are inhibited unless
 * the finger moves more than a certain threshold distance along the Y axis.
 * @velocity - When rel_ballistics is set, this register defines the
 * velocity ballistic parameter applied to all relative motion events.
 * @acceleration - When rel_ballistics is set, this register defines the
 * acceleration ballistic parameter applied to all relative motion events.
 * @sensor_max_x_pos - the maximum X coordinate reported by the sensor.
 * @sensor_max_y_pos - the maximum Y coordinate reported by the sensor.
 */
struct f11_2d_ctrl0_9 {
	/* F11_2D_Ctrl0 */
	u8 reporting_mode:3;
	u8 abs_pos_filt:1;
	u8 rel_pos_filt:1;
	u8 rel_ballistics:1;
	u8 dribble:1;
	u8 report_beyond_clip:1;
	/* F11_2D_Ctrl1 */
	u8 palm_detect_thres:4;
	u8 motion_sensitivity:2;
	u8 man_track_en:1;
	u8 man_tracked_finger:1;
	/* F11_2D_Ctrl2 and 3 */
	u8 delta_x_threshold:8;
	u8 delta_y_threshold:8;
	/* F11_2D_Ctrl4 and 5 */
	u8 velocity:8;
	u8 acceleration:8;
	/* F11_2D_Ctrl6 thru 9 */
	u16 sensor_max_x_pos:12;
	u8 ctrl7_reserved:4;
	u16 sensor_max_y_pos:12;
	u8 ctrl9_reserved:4;
} __attribute__((__packed__));

/**
 * @single_tap_int_enable - enable tap gesture recognition.
 * @tap_n_hold_int_enable - enable tap-and-hold gesture recognition.
 * @double_tap_int_enable - enable double-tap gesture recognition.
 * @early_tap_int_enable - enable early tap notification.
 * @flick_int_enable - enable flick detection.
 * @press_int_enable - enable press gesture recognition.
 * @pinch_int_enable - enable pinch detection.
 */
struct f11_2d_ctrl10 {
	u8 single_tap_int_enable:1;
	u8 tap_n_hold_int_enable:1;
	u8 double_tap_int_enable:1;
	u8 early_tap_int_enable:1;
	u8 flick_int_enable:1;
	u8 press_int_enable:1;
	u8 pinch_int_enable:1;
	u8 reserved:1;
} __attribute__((__packed__));

/**
 * @palm_detect_int_enable - enable palm detection feature.
 * @rotate_int_enable - enable rotate gesture detection.
 * @touch_shape_int_enable - enable the TouchShape feature.
 * @scroll_zone_int_enable - enable scroll zone reporting.
 * @multi_finger_scroll_int_enable - enable the multfinger scroll feature.
 */
struct f11_2d_ctrl11 {
	u8 palm_detect_int_enable:1;
	u8 rotate_int_enable:1;
	u8 touch_shape_int_enable:1;
	u8 scroll_zone_int_enable:1;
	u8 multi_finger_scroll_int_enable:1;
	u8 reserved:3;
} __attribute__((__packed__));

/**
 * @sens_adjustment - allows a host to alter the overall sensitivity of a
 * 2-D sensor. A positive value in this register will make the sensor more
 * sensitive than the factory defaults, and a negative value will make it
 * less sensitive.
 * @hyst_adjustment - increase the touch/no-touch hysteresis by 2 Z-units for
 * each one unit increment in this setting.
 */
struct f11_2d_ctrl14 {
	s8 sens_adjustment:5;
	u8 hyst_adjustment:3;
} __attribute__((__packed__));

/**
 * @max_tap_time - the maximum duration of a tap, in 10-millisecond units.
 */
struct f11_2d_ctrl15 {
	u8 max_tap_time:8;
} __attribute__((__packed__));

/**
 * @min_press_time - The minimum duration required for stationary finger(s) to
 * generate a press gesture, in 10-millisecond units.
 */
struct f11_2d_ctrl16 {
	u8 min_press_time:8;
} __attribute__((__packed__));

/**
 * @max_tap_distance - Determines the maximum finger movement allowed during
 * a tap, in 0.1-millimeter units.
 */
struct f11_2d_ctrl17 {
	u8 max_tap_distance:8;
} __attribute__((__packed__));

/**
 * @min_flick_distance - the minimum finger movement for a flick gesture,
 * in 1-millimeter units.
 * @min_flick_speed - the minimum finger speed for a flick gesture, in
 * 10-millimeter/second units.
 */
struct f11_2d_ctrl18_19 {
	u8 min_flick_distance:8;
	u8 min_flick_speed:8;
} __attribute__((__packed__));

/**
 * @pen_detect_enable - enable reporting of stylus activity.
 * @pen_jitter_filter_enable - Setting this enables the stylus anti-jitter
 * filter.
 * @pen_z_threshold - This is the stylus-detection lower threshold. Smaller
 * values result in higher sensitivity.
 */
struct f11_2d_ctrl20_21 {
	u8 pen_detect_enable:1;
	u8 pen_jitter_filter_enable:1;
	u8 ctrl20_reserved:6;
	u8 pen_z_threshold:8;
} __attribute__((__packed__));

/**
 * These are not accessible through sysfs yet.
 *
 * @proximity_detect_int_en - enable proximity detection feature.
 * @proximity_jitter_filter_en - enables an anti-jitter filter on proximity
 * data.
 * @proximity_detection_z_threshold - the threshold for finger-proximity
 * detection.
 * @proximity_delta_x_threshold - In reduced-reporting modes, this is the
 * threshold for proximate-finger movement in the direction parallel to the
 * X-axis.
 * @proximity_delta_y_threshold - In reduced-reporting modes, this is the
 * threshold for proximate-finger movement in the direction parallel to the
 * Y-axis.
 * * @proximity_delta_Z_threshold - In reduced-reporting modes, this is the
 * threshold for proximate-finger movement in the direction parallel to the
 * Z-axis.
 */
struct f11_2d_ctrl22_26 {
	/* control 22 */
	u8 proximity_detect_int_en:1;
	u8 proximity_jitter_filter_en:1;
	u8 f11_2d_ctrl6_b3__7:6;

	/* control 23 */
	u8 proximity_detection_z_threshold;

	/* control 24 */
	u8 proximity_delta_x_threshold;

	/* control 25 */
	u8 proximity_delta_y_threshold;

	/* control 26 */
	u8 proximity_delta_z_threshold;
} __attribute__((__packed__));

/**
 * @palm_detecy_sensitivity - When this value is small, smaller objects will
 * be identified as palms; when this value is large, only larger objects will
 * be identified as palms. 0 represents the factory default.
 * @suppress_on_palm_detect - when set, all F11 interrupts except palm_detect
 * are suppressed while a palm is detected.
 */
struct f11_2d_ctrl27 {
	s8 palm_detect_sensitivity:4;
	u8 suppress_on_palm_detect:1;
	u8 f11_2d_ctrl27_b5__7:3;
} __attribute__((__packed__));

/**
 * @multi_finger_scroll_mode - allows choice of multi-finger scroll mode and
 * determines whether and how X or Y displacements are reported.
 * @edge_motion_en - enables the edge_motion feature.
 * @multi_finger_scroll_momentum - controls the length of time that scrolling
 * continues after fingers have been lifted.
 */
struct f11_2d_ctrl28 {
	u8 multi_finger_scroll_mode:2;
	u8 edge_motion_en:1;
	u8 f11_2d_ctrl28b_3:1;
	u8 multi_finger_scroll_momentum:4;
} __attribute__((__packed__));

/**
 * @z_touch_threshold - Specifies the finger-arrival Z threshold. Large values
 * may cause smaller fingers to be rejected.
 * @z_touch_hysteresis - Specifies the difference between the finger-arrival
 * Z threshold and the finger-departure Z threshold.
 */
struct f11_2d_ctrl29_30 {
	u8 z_touch_threshold;
	u8 z_touch_hysteresis;
} __attribute__((__packed__));


struct f11_2d_ctrl {
	struct f11_2d_ctrl0_9		 *ctrl0_9;
	u16				ctrl0_9_address;
	struct f11_2d_ctrl10		*ctrl10;
	struct f11_2d_ctrl11		*ctrl11;
	u8				ctrl12_size;
	struct f11_2d_ctrl14		*ctrl14;
	struct f11_2d_ctrl15		*ctrl15;
	struct f11_2d_ctrl16		*ctrl16;
	struct f11_2d_ctrl17		*ctrl17;
	struct f11_2d_ctrl18_19		*ctrl18_19;
	struct f11_2d_ctrl20_21		*ctrl20_21;
	struct f11_2d_ctrl22_26		*ctrl22_26;
	struct f11_2d_ctrl27		*ctrl27;
	struct f11_2d_ctrl28		*ctrl28;
	struct f11_2d_ctrl29_30		*ctrl29_30;
};

/**
 * @x_msb - top 8 bits of X finger position.
 * @y_msb - top 8 bits of Y finger position.
 * @x_lsb - bottom 4 bits of X finger position.
 * @y_lsb - bottom 4 bits of Y finger position.
 * @w_y - contact patch width along Y axis.
 * @w_x - contact patch width along X axis.
 * @z - finger Z value (proxy for pressure).
 */
struct f11_2d_data_1_5 {
	u8 x_msb;
	u8 y_msb;
	u8 x_lsb:4;
	u8 y_lsb:4;
	u8 w_y:4;
	u8 w_x:4;
	u8 z;
} __attribute__((__packed__));

/**
 * @delta_x - relative motion along X axis.
 * @delta_y - relative motion along Y axis.
 */
struct f11_2d_data_6_7 {
	s8 delta_x;
	s8 delta_y;
} __attribute__((__packed__));

/**
 * @single_tap - a single tap was recognized.
 * @tap_and_hold - a tap-and-hold gesture was recognized.
 * @double_tap - a double tap gesture was recognized.
 * @early_tap - a tap gesture might be happening.
 * @flick - a flick gesture was detected.
 * @press - a press gesture was recognized.
 * @pinch - a pinch gesture was detected.
 */
struct f11_2d_data_8 {
	u8 single_tap:1;
	u8 tap_and_hold:1;
	u8 double_tap:1;
	u8 early_tap:1;
	u8 flick:1;
	u8 press:1;
	u8 pinch:1;
} __attribute__((__packed__));

/**
 * @palm_detect - a palm or other large object is in contact with the sensor.
 * @rotate - a rotate gesture was detected.
 * @shape - a TouchShape has been activated.
 * @scrollzone - scrolling data is available.
 * @finger_count - number of fingers involved in the reported gesture.
 */
struct f11_2d_data_9 {
	u8 palm_detect:1;
	u8 rotate:1;
	u8 shape:1;
	u8 scrollzone:1;
	u8 finger_count:3;
} __attribute__((__packed__));

/**
 * @pinch_motion - when a pinch gesture is detected, this is the change in
 * distance between the two fingers since this register was last read.
 */
struct f11_2d_data_10 {
	s8 pinch_motion;
} __attribute__((__packed__));

/**
 * @x_flick_dist - when a flick gesture is detected,  the distance of flick
 * gesture in X direction.
 * @y_flick_dist - when a flick gesture is detected,  the distance of flick
 * gesture in Y direction.
 * @flick_time - the total time of the flick gesture, in 10ms units.
 */
struct f11_2d_data_10_12 {
	s8 x_flick_dist;
	s8 y_flick_dist;
	u8 flick_time;
} __attribute__((__packed__));

/**
 * @motion - when a rotate gesture is detected, the accumulated distance
 * of the rotate motion. Clockwise motion is positive and counterclockwise
 * motion is negative.
 * @finger_separation - when a rotate gesture is detected, the distance
 * between the fingers.
 */
struct f11_2d_data_11_12 {
	s8 motion;
	u8 finger_separation;
} __attribute__((__packed__));

/**
 * @shape_n - a bitmask of the currently activate TouchShapes (if any).
 */
struct f11_2d_data_13 {
	u8 shape_n;
} __attribute__((__packed__));

/**
 * @horizontal - chiral scrolling distance in the X direction.
 * @vertical - chiral scrolling distance in the Y direction.
 */
struct f11_2d_data_14_15 {
	s8 horizontal;
	s8 vertical;
} __attribute__((__packed__));

/**
 * @x_low - scroll zone motion along the lower edge of the sensor.
 * @y_right - scroll zone motion along the right edge of the sensor.
 * @x_upper - scroll zone motion along the upper edge of the sensor.
 * @y_left - scroll zone motion along the left edge of the sensor.
 */
struct f11_2d_data_14_17 {
	s8 x_low;
	s8 y_right;
	s8 x_upper;
	s8 y_left;
} __attribute__((__packed__));

struct f11_2d_data {
	u8				*f_state;
	const struct f11_2d_data_1_5	*abs_pos;
	const struct f11_2d_data_6_7	*rel_pos;
	const struct f11_2d_data_8	*gest_1;
	const struct f11_2d_data_9	*gest_2;
	const struct f11_2d_data_10	*pinch;
	const struct f11_2d_data_10_12	*flick;
	const struct f11_2d_data_11_12	*rotate;
	const struct f11_2d_data_13	*shapes;
	const struct f11_2d_data_14_15	*multi_scroll;
	const struct f11_2d_data_14_17	*scroll_zones;
};

struct f11_abs_pos_data {
        int x;
        int y;
        int z;
        int w_x;
        int w_y;
        int w_min;
        int w_max;
        int orientation;
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
	int abs_size;
	int pkt_size;
	struct f11_abs_pos_data *abs_pos_data;
	u8 sensor_index;
	u8 *button_map;
	struct rmi_f11_virtualbutton_map virtual_buttons;
	u32 type_a;
	enum rmi_f11_sensor_type sensor_type;
	struct input_dev *input;
#ifdef RMI4_FUNCTION_SPECIFIC_INPUT_DEVICE
	struct input_dev *mouse_input;
	char input_phys_mouse[NAME_BUFFER_SIZE];
	char input_phys[NAME_BUFFER_SIZE];
#endif
	struct rmi_function *fn;
	u8 suppress;
	u8 suppress_highw;
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
	struct f11_2d_device_query dev_query;
	struct f11_2d_ctrl dev_controls;
	struct mutex dev_controls_mutex;
	u16 rezero_wait_ms;
	u64 report_count;
	struct f11_2d_sensor sensors[F11_MAX_NUM_OF_SENSORS];
};
#endif
