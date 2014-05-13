/*
 * Copyright (c) 2012, 2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#define FUNCTION_NUMBER 0x12

#include <linux/kernel.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/flex_array.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/rmi.h>

#include "rmi_driver.h"
#include "rmi_f12.h"


/** If CTRL28 is present, it represents a bitmask of the information to be
 * reported for each finger/object.
 */
#define F12_REPORT_TYPE (1 << 0)
#define F12_REPORT_X_LSB (1 << 1)
#define F12_REPORT_X_MSB (1 << 2)
#define F12_REPORT_Y_LSB (1 << 3)
#define F12_REPORT_Y_MSB (1 << 4)
#define F12_REPORT_Z (1 << 5)
#define F12_REPORT_WX (1 << 6)
#define F12_REPORT_WY (1 << 7)

/** By default, we're interested in all the data for a finger.
 */
#define F12_REPORT_DEFAULT (F12_REPORT_TYPE | \
	F12_REPORT_X_LSB | F12_REPORT_X_MSB | \
	F12_REPORT_Y_LSB | F12_REPORT_Y_MSB | \
	F12_REPORT_Z | F12_REPORT_WX | F12_REPORT_WY)
#define F12_REPORT_ENABLES_REG 28

/**
 * Conveniently returns true if the register is present.
 */
static bool rmi_has_register(struct rmi_reg_descriptor *desc, int reg)
{
	return test_bit(reg, desc->presence);
}

/**
 * Conveniently returns the number of bytes in a given register.
 */
static int rmi_register_size(struct rmi_reg_descriptor *desc, int reg)
{
	struct rmi_register_desc *rdesc;

	rdesc = flex_array_get(desc->structure, reg);
	if (rdesc)
		return rdesc->size;
	return 0;
}

/**
 * Conveniently returns the offset of a given register.
 */
static int rmi_register_offset(struct rmi_reg_descriptor *desc, int reg)
{
	struct rmi_register_desc *rdesc;

	rdesc = flex_array_get(desc->structure, reg);
	if (rdesc)
		return rdesc->offset;
	return 0;
}

/**
 * Conveniently returns the number of defined subpackets in a given register.
 * Note that this is different than nr_subpackets, which is the maximum number
 * of subpackets that could be defined in the descriptor.
 */
static int rmi_register_subpackets(struct rmi_reg_descriptor *desc, int reg)
{
	struct rmi_register_desc *rdesc;

	rdesc = flex_array_get(desc->structure, reg);
	if (rdesc)
		return bitmap_weight(rdesc->subpackets, rdesc->nr_subpackets);
	return 0;
}

/**
 * Conveniently returns true if the specified register contains the desired
 * subpacket.
 */
static bool rmi_register_has_subpacket(struct rmi_reg_descriptor *desc, int reg, int sp)
{
	struct rmi_register_desc *rdesc;

	rdesc = flex_array_get(desc->structure, reg);
	if (rdesc && sp < rdesc->nr_subpackets)
		return test_bit(sp, rdesc->subpackets);
	return false;
}

/**
 * Reads and parses the register descriptor.
 *
 * @fn - the function device we're working with
 * @desc - will be filled in with the parsed data
 * @address - the starting address for the register descriptor
 */
static int rmi_read_reg_descriptor(struct rmi_function *fn,
				   struct rmi_reg_descriptor *desc,
				   u16 address)
{
	struct rmi_device *rmi_dev = fn->rmi_dev;
	int retval;
	u8 *buf;
	int i, j;
	int offset;
	int reg_offset = 0;
	int nr_regs;
	int reg;
	int bitpos;
	int lastbit;
	struct rmi_register_desc reg_desc;

	retval = rmi_read(rmi_dev, address, &desc->presence_size);
	if (retval < 0)
		return retval;
	address++;

	buf = devm_kzalloc(&fn->dev, desc->presence_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	retval = rmi_read_block(rmi_dev, address, buf, desc->presence_size);
	if (retval < 0)
		goto exit;
	address++;

	desc->structure_size = buf[0];
	offset = 1;

        if (desc->structure_size == 0) {
		desc->structure_size = buf[1] | (buf[2] << 8);
		offset += 2;
	}
	nr_regs = desc->presence_size - offset;
	desc->presence_bits = nr_regs * 8;
	desc->presence = devm_kzalloc(&fn->dev,
			BITS_TO_LONGS(desc->presence_bits)*sizeof(unsigned long), GFP_KERNEL);
	if (!desc->presence) {
		retval = -ENOMEM;
		goto exit;
	}

	bitpos = 0;
	for (j = 0; j < nr_regs; j++) {
		for (i = 0; i < 8; i++) {
			if (buf[offset] & (1 << i)) {
				set_bit(bitpos, desc->presence);
				lastbit = bitpos;
			}
			bitpos++;
		}
		offset++;
	}

	devm_kfree(&fn->dev, buf);
	buf = devm_kzalloc(&fn->dev, desc->structure_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	desc->structure = flex_array_alloc(sizeof(struct rmi_register_desc),
					   desc->presence_bits, GFP_KERNEL);
	if (!desc->structure) {
		retval = -ENOMEM;
		goto exit;
	}

	retval = rmi_read_block(rmi_dev, address, buf, desc->structure_size);
	if (retval < 0)
		goto exit;

	reg = find_first_bit(desc->presence, desc->presence_bits);
	offset = 0;
	while (reg < desc->presence_bits) {
		bool done = false;
		int base;

		reg_desc.nr_subpackets = 0;
		reg_desc.size = buf[offset];
		reg_desc.offset = reg_offset;
		reg_offset += reg_desc.size;
		offset++;
		if (reg_desc.size == 0) {
			reg_desc.size = buf[offset] | (buf[offset+1] << 8);
			offset += 2;
		}
		if (reg_desc.size == 0) {
			reg_desc.size = buf[offset] | (buf[offset+1] << 8) |
				(buf[offset+2] << 16) | (buf[offset+3] << 24);
			offset += 4;
		}
		base = offset;
		while (!done) {
			reg_desc.nr_subpackets += 7;
			done = (buf[offset] & 0x80) == 0;
			offset++;
		}
		reg_desc.subpackets = devm_kzalloc(&fn->dev,
			BITS_TO_LONGS(reg_desc.nr_subpackets)*sizeof(unsigned long), GFP_KERNEL);
		if (!reg_desc.subpackets) {
			retval = -ENOMEM;
			goto exit;
		}
		offset = base;
		bitpos = 0;
		done = false;
		while (!done) {
			for (i = 0; i < 7; i++) {
				if (buf[offset] & (0x01 << i)) {
					set_bit(bitpos, reg_desc.subpackets);
				}
				bitpos++;
			}
			done = (buf[offset] & 0x80) == 0;
			offset++;
		}
		retval = flex_array_put(desc->structure, reg, &reg_desc, GFP_KERNEL);
		if (retval)
			dev_warn(&fn->dev, "Failed to put reg %d structure info. Code: %d.\n", reg, retval);
		reg = find_next_bit(desc->presence, desc->presence_bits, reg+1);
	}

	retval = 0;

exit:
	devm_kfree(&fn->dev, buf);
	return retval;
}

/** Handy diagnostic routine for dumping register descriptors.
 */
static void rmi_dump_descriptor(struct rmi_function *fn,
				struct rmi_reg_descriptor *desc,
				char* desc_name)
{
	int reg;

	dev_dbg(&fn->dev, "F%02X %s has %d presence bits.\n", fn->fd.function_number, desc_name, desc->presence_bits);
	for (reg = 0; reg < desc->presence_bits; reg++) {
		dev_dbg(&fn->dev, "F%02X %s%02d - present: %d.\n", fn->fd.function_number, desc_name, reg, rmi_has_register(desc, reg));
	}
	reg = find_first_bit(desc->presence, desc->presence_bits);
}

static void rmi_dump_reg_descriptors(struct rmi_function *fn,
				struct rmi_descriptors *descs)
{
	rmi_dump_descriptor(fn, &descs->control, "CTRL");
}

/**
 * Reads and parses a set of register descriptors.
 */
static int rmi_read_descriptors(struct rmi_function *fn,
				struct rmi_descriptors *descs,
				u16 address) {
	int retval;

	retval = rmi_read_reg_descriptor(fn, &descs->query, address);
	if (retval) {
		dev_err(&fn->dev, "Failed to read query descriptor, code %d\n",
			retval);
		return retval;
	}
	address += 3;

	retval = rmi_read_reg_descriptor(fn, &descs->control, address);
	if (retval) {
		dev_err(&fn->dev, "Failed to read control descriptor, code %d\n",
			retval);
		return retval;
	}
	address += 3;

	retval = rmi_read_reg_descriptor(fn, &descs->data, address);
	if (retval)
		dev_err(&fn->dev, "Failed to read data descriptor, code %d\n",
			retval);

	rmi_dump_reg_descriptors(fn, descs);
	return retval;
}

#define F12_OBJECT_NONE 0
#define F12_OBJECT_FINGER 1
#define F12_OBJECT_STYLUS 2
#define F12_OBJECT_PALM 3
#define F12_OBJECT_UNCLASSIFIED 4

/**
 * Data for a single object (pen, finger, palm, whatever).
 *
 * @type - says what, if anything, has been detected by the sensor.
 * @pos_x - x location of the object, little endian.
 * @pos_y - y location of the object, little endian.
 * @z - z value for this object.
 * @wx - W value parallel to the X axis.
 * @wy - W value parallel to the Y axis.
 */
struct rmi_f12_object_data {
	u8 type:8;
	u16 pos_x:16;
	u16 pos_y:16;
	u8 z:8;
	u8 wx:8;
	u8 wy:8;
} __attribute__((__packed__));

#define DEFAULT_XY_MAX 65535
#define DEFAULT_MAX_ABS_MT_PRESSURE 255
#define DEFAULT_MAX_ABS_MT_TOUCH 15
#define DEFAULT_MAX_ABS_MT_ORIENTATION 1
#define DEFAULT_MIN_ABS_MT_TRACKING_ID 1

#define F12_FINGER_DATA_REG 1
#define F12_SENSOR_TUNING_REG 8
#define F12_COORD_MAX_SP 0

/**
 * Read the Sensor Tuning control register and set operating parameters
 * appropriately.
 */
static void read_sensor_tuning(struct rmi_function *fn) {
	struct f12_data *f12 = fn->data;
	int buffer_size;
	int offset = 0;
	int retval;

	f12->x_max = f12->y_max = DEFAULT_XY_MAX;

	buffer_size = rmi_register_size(&f12->desc.control, F12_SENSOR_TUNING_REG);
	if (!buffer_size)
		return;

	f12->sensor_tuning.buffer = devm_kzalloc(&fn->dev, buffer_size, GFP_KERNEL);
	if (!f12->sensor_tuning.buffer)
		return;

	retval = rmi_read_block(fn->rmi_dev, fn->fd.control_base_addr,
				f12->sensor_tuning.buffer, buffer_size);
	if (retval < 0) {
		dev_warn(&fn->dev, "WARNING: Failed to read sensor tuning (code: %d).  Using default values.\n",
			 retval);
		return;
	}

	if (rmi_register_has_subpacket(&f12->desc.control, F12_SENSOR_TUNING_REG, F12_COORD_MAX_SP)) {
		f12->sensor_tuning.x_max_le = (u16*) &f12->sensor_tuning.buffer[offset];
		offset += 2;
		f12->sensor_tuning.y_max_le = (u16*) &f12->sensor_tuning.buffer[offset];
		offset += 2;
		f12->x_max = le16_to_cpu(*f12->sensor_tuning.x_max_le);
		f12->y_max = le16_to_cpu(*f12->sensor_tuning.y_max_le);
	}
}

static int get_tool_type(struct rmi_f12_object_data *object)
{
	if (object->type == F12_OBJECT_STYLUS)
		return MT_TOOL_PEN;
	return MT_TOOL_FINGER;
}

static void report_one_object(struct f12_data *f12, struct rmi_f12_object_data *object, int slot)
{
	input_mt_slot(f12->input, slot);
	input_mt_report_slot_state(f12->input, get_tool_type(object), object->type);

	if (object->type) {
		u16 w_min = object->wx;
		u16 w_max = object->wy;

		if (object->wx > object->wy) {
			w_min = object->wy;
			w_max = object->wx;
		}
		input_report_abs(f12->input, ABS_MT_PRESSURE, object->z);
		input_report_abs(f12->input, ABS_MT_TOUCH_MAJOR, w_max);
		input_report_abs(f12->input, ABS_MT_TOUCH_MINOR, w_min);
		input_report_abs(f12->input, ABS_MT_ORIENTATION,
			object->wx > object->wy);
		input_report_abs(f12->input, ABS_MT_POSITION_X,
			le16_to_cpu(object->pos_x));
		input_report_abs(f12->input, ABS_MT_POSITION_Y,
			le16_to_cpu(object->pos_y));
		pr_debug("rmi finger[%d]:%d - x:%d y:%d z:%d wx:%d wy:%d\n",
			slot, object->type, le16_to_cpu(object->pos_x),
			le16_to_cpu(object->pos_y), object->z,
			object->wx, object->wy);
	}
}

static void report_objects(struct f12_data *f12)
{
	int i;
	int object_count = 0;

	for (i = 0; i < f12->max_objects; i++) {
		struct rmi_f12_object_data *object = (struct rmi_f12_object_data *) &f12->object_buf[i * 8];
		if (object->type)
			object_count++;
		report_one_object(f12, object, i);
	}
	// TODO update lines for newer kernel
	input_report_key(f12->input, BTN_TOUCH, object_count);
	input_sync(f12->input);
}

static int rmi_f12_attention(struct rmi_function *fn,
			     unsigned long *irq_nr_regs)
{
	int retval;
	struct f12_data *f12 = fn->data;

	retval = rmi_read_block(fn->rmi_dev, f12->object_address,
				f12->object_buf, f12->buf_size);
	if (retval < 0) {
		dev_err(&fn->dev, "Failed to read object data. Code: %d.\n",
			retval);
		return retval;
	}

	report_objects(f12);
	return 0;
}

static void rmi_f12_remove(struct rmi_function *fn)
{
	struct f12_data *f12 = fn->data;

	input_unregister_device(f12->input);
	devm_kfree(&fn->dev, f12->object_buf);
	devm_kfree(&fn->dev, f12);
	fn->data = NULL;
}

static int rmi_f12_probe(struct rmi_function *fn)
{
	struct f12_data *f12;
	int retval;
	struct input_dev *input_dev;
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct rmi_driver *driver = rmi_dev->driver;

	dev_dbg(&fn->dev, "%s called.\n", __func__);

	f12 = devm_kzalloc(&fn->dev, sizeof(struct f12_data), GFP_KERNEL);
	if (!f12)
		return -ENOMEM;

	fn->data = f12;

	retval = rmi_read_block(fn->rmi_dev, fn->fd.query_base_addr,
		&f12->info, sizeof(f12->info));
	if (retval < 0) {
		dev_err(&fn->dev, "Failed to read general info register, code %d\n",
			retval);
		goto error_free_data;
	}
	if (!f12->info.has_register_descriptors) {
		dev_err(&fn->dev, "Behavior of F12 without register descriptors is undefined.\n");
		retval = -ENODEV;
		goto error_free_data;
	}

	retval = rmi_read_descriptors(fn, &f12->desc,
				      fn->fd.query_base_addr + 1);
	if (retval)
		return retval;

	if (!rmi_has_register(&f12->desc.data, F12_FINGER_DATA_REG)) {
		dev_err(&fn->dev, "Finger data registers are missing!\n");
		retval = -ENODEV;
		goto error_free_data;
	}

	if (rmi_has_register(&f12->desc.control, F12_REPORT_ENABLES_REG)) {
		u16 addr = fn->fd.control_base_addr + rmi_register_offset(&f12->desc.control, F12_REPORT_ENABLES_REG);
		u8 enables = F12_REPORT_DEFAULT;
		dev_dbg(&fn->dev, "Writing enables %#04x to %#06x.\n", enables, addr);
		retval = rmi_write(rmi_dev, addr, enables);
		if (retval < 0) {
			dev_err(&fn->dev, "Failed to write F12 enables, code: %d.\n",
				retval);
			return retval;
		}
	}

	f12->buf_size = rmi_register_size(&f12->desc.data, F12_FINGER_DATA_REG);
	f12->object_buf = devm_kzalloc(&fn->dev, f12->buf_size * sizeof(u8), GFP_KERNEL);
	if (!f12->object_buf) {
		retval = -ENOMEM;
		goto error_free_data;
	}
	f12->max_objects = rmi_register_subpackets(&f12->desc.data, F12_FINGER_DATA_REG);
	f12->object_address = fn->fd.data_base_addr + rmi_register_offset(&f12->desc.data, F12_FINGER_DATA_REG);

	read_sensor_tuning(fn);

	input_dev = input_allocate_device();
	if (!input_dev) {
		retval = -ENOMEM;
		goto error_free_data;
	}
	if (driver->set_input_params) {
		retval = driver->set_input_params(fn->rmi_dev, input_dev);
		if (retval < 0) {
			dev_err(&fn->dev, "Error in setting input device.\n");
			goto error_free_dev;
		}
	}
	sprintf(f12->input_phys, "%s/input0", dev_name(&fn->dev));
	input_dev->phys = f12->input_phys;
	input_dev->dev.parent = &rmi_dev->dev;
	input_set_drvdata(input_dev, f12);

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	input_mt_init_slots(input_dev, f12->max_objects);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			0, f12->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			0, f12->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0,
			DEFAULT_MAX_ABS_MT_PRESSURE, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR,
			0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
			0, DEFAULT_MAX_ABS_MT_ORIENTATION, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
			DEFAULT_MIN_ABS_MT_TRACKING_ID,
			f12->max_objects, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
				0, MT_TOOL_MAX, 0, 0);

	retval = input_register_device(input_dev);
	if (retval < 0)
		goto error_free_dev;
	f12->input = input_dev;

	return 0;

error_free_dev:
	input_free_device(input_dev);

error_free_data:
	devm_kfree(&fn->dev, f12->object_buf);
	devm_kfree(&fn->dev, f12);
	return retval;
}

static struct rmi_function_handler function_handler = {
	.driver = {
		.name = "rmi_f12",
	},
	.func = FUNCTION_NUMBER,
	.probe = rmi_f12_probe,
	.remove = rmi_f12_remove,
	.attention = rmi_f12_attention,
};

module_rmi_driver(function_handler);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com");
MODULE_DESCRIPTION("F12 2D pointing");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);