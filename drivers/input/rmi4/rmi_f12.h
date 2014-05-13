/*
 * Copyright (c) 2013-2014 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef RMI_F12_CONTROL_H
#define RMI_F12_CONTROL_H

/**
 * Describes the size and subpacket presence for a given packet register.
 * @nr_subpackets - how many subpackets can there be in this register (not
 * necessarily the number that are present).
 * @subpackets - a bitmap indicating which subpacket registers are present.
 * @offset - offset of this register from the base address for the register
 * type it defines.
 */
struct rmi_register_desc {
	u8 size;
	u8 nr_subpackets;
	unsigned long *subpackets;
	u16 offset;
};

/** Register descriptors provide an extensible way to describe which registers
 * are or are not present for a function on a given device.
 *
 * @presence_size - says how many bytes are in the register presence registers
 * @structure_size - says how many bytes are in the register structure
 * registers
 * @presence - if a register is present in the function, the corresponding bit
 * in this bitmap is set.
 * @nr_registers - the total number of registers in the structure array.
 * @structure - a sparse array of rmi_register_desc that describe how which
 * subpackets are present in each defined packet register.
 */
struct rmi_reg_descriptor {
	u8	presence_size;
	u16	structure_size;
	u16	presence_bits;
	unsigned long *presence;
	u16	nr_registers;
	struct flex_array *structure;
};

/**
 * Register descriptors for a given function.
 */
struct rmi_descriptors {
	struct rmi_reg_descriptor query;
	struct rmi_reg_descriptor control;
	struct rmi_reg_descriptor data;
};

/**
 * Query 0 describes the general properties of an F12 sensor.
 *
 * @has_register_descriptors - if 1, the F12 packet register format is support.
 */
struct rmi_f12_general_info {
	u8 has_register_descriptors:1;
	u8 reserved:7;
} __attribute__((__packed__));


/**
 * Control registers for overall sensor operating parameters.
 */
struct f12_ctl_sensor_tuning {
	u8 *buffer;
	u16 *x_max_le;
	u16 *y_max_le;
	u16 *rx_pitch_le;
	u16 *tx_pitch_le;
	u8 *rx_clip_low;
	u8 *rx_clip_high;
	u8 *tx_clip_low;
	u8 *tx_clip_high;
	u8 *nr_rx;
	u8 *nr_tx;
};

#define NAME_BUFFER_SIZE 256

/**
 * Data pertaining to a given F12 device.
 *
 * @info - the general information query
 * @desc - register decsriptors as determined by reading the query registers.
 * @buf_size - how many bytes in the object data buffer
 * @object_buf - buffer for reading object (finger) position data
 * @max_objects - the maximum number of objects (fingers) that might be
 * reported.
 * @object_address - RMI4 register address for the object position data
 * @x_max - maximum X coordinate
 * @y_max - maximum Y coordinate
 * @sensor_tuning - sensor tuning control registers.
 * @input - input device for reporting positions.
 * @input_phys - string for naming the input device.
 */
struct f12_data {
	struct rmi_f12_general_info info;
	struct rmi_descriptors desc;
	u8 buf_size;
	u8 *object_buf;
	u8 max_objects;
	u16 object_address;

	u16 x_max;
	u16 y_max;

	struct f12_ctl_sensor_tuning sensor_tuning;

	struct input_dev *input;
	char input_phys[NAME_BUFFER_SIZE];
};

#endif