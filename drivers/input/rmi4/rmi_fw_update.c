/*
 * Copyright (c) 2012 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/ihex.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/time.h>
#include "rmi_driver.h"
#include "rmi_f01.h"
#include "rmi_f34.h"

#define HAS_BSR_MASK 0x20

#define CHECKSUM_OFFSET 0
#define IO_OFFSET 0x06
#define BOOTLOADER_VERSION_OFFSET 0x07
#define IMAGE_SIZE_OFFSET 0x08
#define CONFIG_SIZE_OFFSET 0x0C
#define PACKAGE_ID_OFFSET 0x1A
#define FW_BUILD_ID_OFFSET 0x50

#define RMI_PRODUCT_INFO_LENGTH 2

#define IMG_PRODUCT_ID_OFFSET 0x10
#define IMG_PRODUCT_INFO_OFFSET 0x1E

#define ENABLE_WAIT_US (300 * 1000)

/** Image file V5, Option 0
 */
struct image_header {
	u32 checksum;
	unsigned int image_size;
	unsigned int config_size;
	u8 options;
	u8 io;
	u32 fw_build_id;
	u32 package_id;
	u8 bootloader_version;
	u8 product_id[RMI_PRODUCT_ID_LENGTH + 1];
	u8 product_info[RMI_PRODUCT_INFO_LENGTH];
};

static u32 extract_u32(const u8 *ptr)
{
	return (u32)ptr[0] +
		(u32)ptr[1] * 0x100 +
		(u32)ptr[2] * 0x10000 +
		(u32)ptr[3] * 0x1000000;
}

struct rmi_reflash_data {
	struct rmi_device *rmi_dev;
	bool force;
	ulong busy;
	char *img_name;
	char *name_buf;
	size_t name_buf_len;
	struct pdt_entry f01_pdt;
	struct f01_basic_properties f01_props;
	u8 device_status;
	struct pdt_entry f34_pdt;
	u8 bootloader_id[2];
	union f34_query_regs f34_queries;
	union f34_control_status f34_controls;
	const u8 *firmware_data;
	const u8 *config_data;
	struct work_struct reflash_work;
};

MODULE_PARM_DESC(param, "Name of the RMI4 firmware image");

#define RMI4_IMAGE_FILE_REV1_OFFSET 30
#define RMI4_IMAGE_FILE_REV2_OFFSET 31
#define IMAGE_FILE_CHECKSUM_SIZE 4
#define FIRMWARE_IMAGE_AREA_OFFSET 0x100

static void extract_header(const u8 *data, int pos, struct image_header *header)
{
	header->checksum = extract_u32(&data[pos + CHECKSUM_OFFSET]);
	header->io = data[pos + IO_OFFSET];
	header->bootloader_version = data[pos + BOOTLOADER_VERSION_OFFSET];
	header->image_size = extract_u32(&data[pos + IMAGE_SIZE_OFFSET]);
	header->config_size = extract_u32(&data[pos + CONFIG_SIZE_OFFSET]);
	if (header->io == 1) {
		header->fw_build_id = extract_u32(&data[pos + FW_BUILD_ID_OFFSET]);
		header->package_id = extract_u32(&data[pos + PACKAGE_ID_OFFSET]);
	}
	memcpy(header->product_id, &data[pos + IMG_PRODUCT_ID_OFFSET],
	       RMI_PRODUCT_ID_LENGTH);
	header->product_id[RMI_PRODUCT_ID_LENGTH] = 0;
	memcpy(header->product_info, &data[pos + IMG_PRODUCT_INFO_OFFSET],
	       RMI_PRODUCT_INFO_LENGTH);
}

static int rmi_find_functions(struct rmi_device *rmi_dev,
		void *ctx, const struct pdt_entry *pdt)
{
 	struct rmi_reflash_data *data = ctx;

	if (pdt->page_start > 0)
		return RMI_SCAN_DONE;

	if (pdt->function_number == 0x01)
		memcpy(&data->f01_pdt, pdt, sizeof(struct pdt_entry));
	else if (pdt->function_number == 0x34)
		memcpy(&data->f34_pdt, pdt, sizeof(struct pdt_entry));

	return RMI_SCAN_CONTINUE;
}

static int find_f01_and_f34(struct rmi_reflash_data* data)
{
	struct rmi_device *rmi_dev = data->rmi_dev;
	int retval;

	data->f01_pdt.function_number = data->f34_pdt.function_number = 0;
	retval = rmi_scan_pdt(rmi_dev, data, rmi_find_functions);
	if (retval < 0)
		return retval;

	if (!data->f01_pdt.function_number) {
		dev_err(&rmi_dev->dev, "Failed to find F01 for reflash.\n");
		return -ENODEV;
	}

	if (!data->f34_pdt.function_number) {
		dev_err(&rmi_dev->dev, "Failed to find F34 for reflash.\n");
		return -ENODEV;
	}

	return 0;
}

static int read_f34_controls(struct rmi_reflash_data *data)
{
	int retval;

	retval = rmi_read(data->rmi_dev, data->f34_controls.address,
			  data->f34_controls.regs);
	if (retval < 0)
		return retval;

	return 0;
}

static int read_f01_status(struct rmi_reflash_data *data)
{
	int retval;

	retval = rmi_read(data->rmi_dev, data->f01_pdt.data_base_addr,
			  &data->device_status);
	if (retval < 0)
		return retval;

	return 0;
}

#define MIN_SLEEP_TIME_US 50
#define MAX_SLEEP_TIME_US 100

/* Wait until the status is idle and we're ready to continue */
static int wait_for_idle(struct rmi_reflash_data *data, int timeout_ms)
{
	int timeout_count = ((timeout_ms * 1000) / MAX_SLEEP_TIME_US) + 1;
	int count = 0;
	union f34_control_status *controls = &data->f34_controls;
	int retval;

	do {
		if (count || timeout_count == 1)
			usleep_range(MIN_SLEEP_TIME_US, MAX_SLEEP_TIME_US);
		retval = read_f34_controls(data);
		count++;
		if (retval < 0)
			continue;
		else if (IS_IDLE(controls)) {
			if (!data->f34_controls.program_enabled) {
				/** This works around a bug in certain device
				 * firmwares, where the idle state is reached,
				 * but the program_enabled bit is not yet set.
				 */
				dev_warn(&data->rmi_dev->dev, "Yikes!  We're not enabled!\n");
				msleep(1000);
				read_f34_controls(data);
			}
			return 0;
		}
	} while (count < timeout_count);

	dev_err(&data->rmi_dev->dev,
		"ERROR: Timeout waiting for idle status, last status: %#04x.\n",
		controls->regs[0]);
	dev_err(&data->rmi_dev->dev, "Command: %#04x\n", controls->command);
	dev_err(&data->rmi_dev->dev, "Status:  %#04x\n", controls->status);
	dev_err(&data->rmi_dev->dev, "Enabled: %d\n",
			controls->program_enabled);
	dev_err(&data->rmi_dev->dev, "Idle:    %d\n", IS_IDLE(controls));
	return -ETIMEDOUT;
}

static int read_f34_queries(struct rmi_reflash_data *data)
{
	int retval;
	u8 id_str[3];

	retval = rmi_read_block(data->rmi_dev, data->f34_pdt.query_base_addr,
				data->bootloader_id, 2);
	if (retval < 0) {
		dev_err(&data->rmi_dev->dev,
			"Failed to read F34 bootloader_id (code %d).\n",
			retval);
		return retval;
	}
	retval = rmi_read_block(data->rmi_dev, data->f34_pdt.query_base_addr+2,
			data->f34_queries.regs,
			ARRAY_SIZE(data->f34_queries.regs));
	if (retval < 0) {
		dev_err(&data->rmi_dev->dev,
			"Failed to read F34 queries (code %d).\n", retval);
		return retval;
	}
	data->f34_queries.block_size =
			le16_to_cpu(data->f34_queries.block_size);
	data->f34_queries.fw_block_count =
			le16_to_cpu(data->f34_queries.fw_block_count);
	data->f34_queries.config_block_count =
			le16_to_cpu(data->f34_queries.config_block_count);
	id_str[0] = data->bootloader_id[0];
	id_str[1] = data->bootloader_id[1];
	id_str[2] = 0;

	dev_dbg(&data->rmi_dev->dev, "F34 bootloader id: %s (%#04x %#04x)\n",
		id_str, data->bootloader_id[0], data->bootloader_id[1]);
	dev_dbg(&data->rmi_dev->dev, "F34 has config id: %d\n",
		data->f34_queries.has_config_id);
	dev_dbg(&data->rmi_dev->dev, "F34 unlocked:      %d\n",
		data->f34_queries.unlocked);
	dev_dbg(&data->rmi_dev->dev, "F34 regMap:        %d\n",
		data->f34_queries.reg_map);
	dev_dbg(&data->rmi_dev->dev, "F34 block size:    %d\n",
		data->f34_queries.block_size);
	dev_dbg(&data->rmi_dev->dev, "F34 fw blocks:     %d\n",
		data->f34_queries.fw_block_count);
	dev_dbg(&data->rmi_dev->dev, "F34 config blocks: %d\n",
		data->f34_queries.config_block_count);

	data->f34_controls.address = data->f34_pdt.data_base_addr +
			F34_BLOCK_DATA_OFFSET + data->f34_queries.block_size;

	return 0;
}

static int write_bootloader_id(struct rmi_reflash_data *data)
{
	int retval;
	struct rmi_device *rmi_dev = data->rmi_dev;

	retval = rmi_write_block(rmi_dev,
			data->f34_pdt.data_base_addr + F34_BLOCK_DATA_OFFSET,
			data->bootloader_id, ARRAY_SIZE(data->bootloader_id));
	if (retval < 0) {
		dev_err(&rmi_dev->dev,
			"Failed to write bootloader ID. Code: %d.\n", retval);
		return retval;
	}

	return 0;
}

static int write_f34_command(struct rmi_reflash_data *data, u8 command)
{
	int retval;
	struct rmi_device *rmi_dev = data->rmi_dev;

	retval = rmi_write(rmi_dev, data->f34_controls.address, command);
	if (retval < 0) {
		dev_err(&rmi_dev->dev,
			"Failed to write F34 command %#04x. Code: %d.\n",
			command, retval);
		return retval;
	}

	return 0;
}

static int enter_flash_programming(struct rmi_reflash_data *data)
{
	int retval;
	struct rmi_device *rmi_dev = data->rmi_dev;
	u8 f01_control_0;

	retval = write_bootloader_id(data);
	if (retval < 0)
		return retval;

	dev_dbg(&rmi_dev->dev, "Enabling flash programming.\n");
	retval = write_f34_command(data, F34_ENABLE_FLASH_PROG);
	if (retval < 0)
		return retval;

	retval = wait_for_idle(data, F34_ENABLE_WAIT_MS);
	if (retval) {
		dev_err(&rmi_dev->dev, "Did not reach idle state after %d ms. Code: %d.\n",
			F34_ENABLE_WAIT_MS, retval);
		return retval;
	}
	if (!data->f34_controls.program_enabled) {
		dev_err(&rmi_dev->dev, "Reached idle, but programming not enabled (current status register: %#04x).\n",
					data->f34_controls.regs[0]);
		return -EINVAL;
	}
	dev_dbg(&rmi_dev->dev, "HOORAY! Programming is enabled!\n");

	retval = find_f01_and_f34(data);
	if (retval) {
		dev_err(&rmi_dev->dev, "Failed to rescan pdt.  Code: %d.\n",
			retval);
		return retval;
	}

	retval = read_f01_status(data);
	if (retval) {
		dev_err(&rmi_dev->dev, "Failed to read F01 status after enabling reflash. Code: %d.\n",
			retval);
		return retval;
	}
	if (!RMI_F01_STATUS_BOOTLOADER(data->device_status)) {
		dev_err(&rmi_dev->dev, "Device reports as not in flash programming mode.\n");
		return -EINVAL;
	}

	retval = read_f34_queries(data);
	if (retval) {
		dev_err(&rmi_dev->dev, "F34 queries failed, code = %d.\n",
			retval);
		return retval;
	}

	retval = rmi_read(rmi_dev, data->f01_pdt.control_base_addr,
			  &f01_control_0);
	if (retval < 0) {
		dev_err(&rmi_dev->dev, "F01_CTRL_0 read failed, code = %d.\n",
			retval);
		return retval;
	}
	f01_control_0 |= RMI_F01_CRTL0_NOSLEEP_BIT;
	f01_control_0 = (f01_control_0 & ~0x03) | RMI_SLEEP_MODE_NORMAL;

	retval = rmi_write(rmi_dev, data->f01_pdt.control_base_addr,
			   f01_control_0);
	if (retval < 0) {
		dev_err(&rmi_dev->dev, "F01_CTRL_0 write failed, code = %d.\n",
			retval);
		return retval;
	}

	return 0;
}

static void reset_device(struct rmi_reflash_data *data)
{
	int retval;
	struct rmi_device_platform_data *pdata =
		to_rmi_platform_data(data->rmi_dev);

	dev_dbg(&data->rmi_dev->dev, "Resetting...\n");
	retval = rmi_write(data->rmi_dev, data->f01_pdt.command_base_addr,
			   RMI_F01_CMD_DEVICE_RESET);
	if (retval < 0)
		dev_warn(&data->rmi_dev->dev,
			 "WARNING - post-flash reset failed, code: %d.\n",
			 retval);
	msleep(pdata->reset_delay_ms ?: RMI_F01_DEFAULT_RESET_DELAY_MS);
	dev_dbg(&data->rmi_dev->dev, "Reset completed.\n");
}

/*
 * Send data to the device one block at a time.
 */
static int write_blocks(struct rmi_reflash_data *data, u8 *block_ptr,
			u16 block_count, u8 cmd)
{
	int block_num;
	u8 zeros[] = {0, 0};
	int retval;
	u16 addr = data->f34_pdt.data_base_addr + F34_BLOCK_DATA_OFFSET;

	retval = rmi_write_block(data->rmi_dev, data->f34_pdt.data_base_addr,
				 zeros, ARRAY_SIZE(zeros));
	if (retval < 0) {
		dev_err(&data->rmi_dev->dev, "Failed to write initial zeros. Code=%d.\n",
			retval);
		return retval;
	}

	for (block_num = 0; block_num < block_count; ++block_num) {
		retval = rmi_write_block(data->rmi_dev, addr, block_ptr,
					 data->f34_queries.block_size);
		if (retval < 0) {
			dev_err(&data->rmi_dev->dev, "Failed to write block %d. Code=%d.\n",
				block_num, retval);
			return retval;
		}

		retval = write_f34_command(data, cmd);
		if (retval) {
			dev_err(&data->rmi_dev->dev, "Failed to write command for block %d. Code=%d.\n",
				block_num, retval);
			return retval;
		}


		retval = wait_for_idle(data, F34_IDLE_WAIT_MS);
		if (retval) {
			dev_err(&data->rmi_dev->dev, "Failed to go idle after writing block %d. Code=%d.\n",
				block_num, retval);
			return retval;
		}

		block_ptr += data->f34_queries.block_size;
	}

	return 0;
}

static int write_firmware(struct rmi_reflash_data *data)
{
	return write_blocks(data, (u8 *) data->firmware_data,
		data->f34_queries.fw_block_count, F34_WRITE_FW_BLOCK);
}

static int write_configuration(struct rmi_reflash_data *data)
{
	return write_blocks(data, (u8 *) data->config_data,
		data->f34_queries.config_block_count, F34_WRITE_CONFIG_BLOCK);
}

static void reflash_firmware(struct rmi_reflash_data *data)
{
	struct timespec start;
	struct timespec end;
	s64 duration_ns;
	int retval = 0;

	retval = enter_flash_programming(data);
	if (retval) {
		dev_err(&data->rmi_dev->dev, "Failed to enter flash programming (code: %d).\n",
			retval);
		return;
	}

	retval = write_bootloader_id(data);
	if (retval) {
		dev_err(&data->rmi_dev->dev, "Failed to enter write bootloader ID (code: %d).\n",
			retval);
		return;
	}

	dev_dbg(&data->rmi_dev->dev, "Erasing FW...\n");
	getnstimeofday(&start);
	retval = write_f34_command(data, F34_ERASE_ALL);
	if (retval) {
		dev_err(&data->rmi_dev->dev, "Erase failed (code: %d).\n",
			retval);
		return;
	}

	retval = wait_for_idle(data, F34_ERASE_WAIT_MS);
	if (retval) {
		dev_err(&data->rmi_dev->dev,
			"Failed to reach idle state. Code: %d.\n", retval);
		return;
	}
	getnstimeofday(&end);
	duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
	dev_dbg(&data->rmi_dev->dev,
		 "Erase complete, time: %lld ns.\n", duration_ns);

	if (data->firmware_data) {
		dev_dbg(&data->rmi_dev->dev, "Writing firmware...\n");
		getnstimeofday(&start);
		retval = write_firmware(data);
		if (retval) {
			dev_err(&data->rmi_dev->dev,
				"Failed to write FW (code: %d).\n", retval);
			return;
		}
		getnstimeofday(&end);
		duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
		dev_dbg(&data->rmi_dev->dev,
			 "Done writing FW, time: %lld ns.\n", duration_ns);
	}

	if (data->config_data) {
		dev_dbg(&data->rmi_dev->dev, "Writing configuration...\n");
		getnstimeofday(&start);
		retval = write_configuration(data);
		if (retval) {
			dev_err(&data->rmi_dev->dev,
				"Failed to write config (code: %d).\n", retval);
			return;
		}
		getnstimeofday(&end);
		duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
		dev_dbg(&data->rmi_dev->dev,
			 "Done writing config, time: %lld ns.\n", duration_ns);
	}
}

/* Returns false if the firmware should not be reflashed.
 */
static bool go_nogo(struct rmi_reflash_data *data, struct image_header *header)
{
// 	int retval;

	if (data->force) {
		dev_dbg(&data->rmi_dev->dev, "Reflash force flag in effect.\n");
		return true;
	}

// 	if (data->f01_props.productinfo[0] < header->product_info[0] ||
// 		data->f01_props.productinfo[1] < header->product_info[1]) {
// 		dev_info(&data->rmi_dev->dev,
// 			 "FW product ID is older than image product ID.\n");
// 		return true;
// 	}

	if (header->io == 1) {
		if (header->fw_build_id > data->f01_props.build_id) {
			dev_dbg(&data->rmi_dev->dev, "Image file has newer Packrat.\n");
			return true;
		}
		else
			dev_dbg(&data->rmi_dev->dev, "Image file has lower Packrat ID than device.\n");
	}

// 	retval = read_f01_status(data);
// 	if (retval)
// 		dev_err(&data->rmi_dev->dev,
// 			"Failed to read F01 status. Code: %d.\n", retval);
//
// 	dev_dbg(&data->rmi_dev->dev, "Flash prog bit at go/nogo: %d\n",
// 			RMI_F01_STATUS_BOOTLOADER(data->device_status));
// 	return RMI_F01_STATUS_BOOTLOADER(data->device_status);
	return false;
}

#define NAME_BUFFER_SIZE 64
#define FW_NAME_FORMAT "%s.img"

static void rmi4_fw_update(struct rmi_device *rmi_dev)
{
	struct timespec start;
	struct timespec end;
	s64 duration_ns;
	char *firmware_name;
	const struct firmware *fw_entry = NULL;
	int retval;
	struct image_header *header = NULL;
	u8 pdt_props;
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data* data = drv_data->reflash_data;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);

	dev_dbg(&rmi_dev->dev, "%s called.\n", __func__);
	dev_dbg(&rmi_dev->dev, "force: %d\n", data->force);
	dev_dbg(&rmi_dev->dev, "img_name: %s\n", data->img_name);
	dev_dbg(&rmi_dev->dev, "firmware_name: %s\n", pdata->firmware_name);

	getnstimeofday(&start);

	firmware_name = kcalloc(NAME_BUFFER_SIZE, sizeof(char), GFP_KERNEL);
	if (!firmware_name) {
		dev_err(&rmi_dev->dev, "Failed to allocate firmware_name.\n");
		goto done;
	}
	header = kzalloc(sizeof(struct image_header), GFP_KERNEL);
	if (!header) {
		dev_err(&rmi_dev->dev, "Failed to allocate header.\n");
		goto done;
	}

	retval = rmi_read(rmi_dev, PDT_PROPERTIES_LOCATION, &pdt_props);
	if (retval < 0) {
		dev_warn(&rmi_dev->dev,
			 "Failed to read PDT props at %#06x (code %d). Assuming 0x00.\n",
			 PDT_PROPERTIES_LOCATION, retval);
	}
	if (pdt_props & RMI_PDT_PROPS_HAS_BSR) {
		dev_warn(&rmi_dev->dev,
			 "Firmware update for LTS not currently supported.\n");
		goto done;
	}

	retval = rmi_f01_read_properties(rmi_dev, data->f01_pdt.query_base_addr,
					 &data->f01_props);
	if (retval) {
		dev_err(&rmi_dev->dev, "F01 queries failed, code = %d.\n",
			retval);
		goto done;
	}
	retval = read_f34_queries(data);
	if (retval) {
		dev_err(&rmi_dev->dev, "F34 queries failed, code = %d.\n",
			retval);
		goto done;
	}
	if (data->img_name && strlen(data->img_name)) {
		dev_dbg(&rmi_dev->dev, "Using sysfs firmware name: %s\n", data->img_name);
		snprintf(firmware_name, NAME_BUFFER_SIZE, FW_NAME_FORMAT,
			data->img_name);
	} else if (pdata->firmware_name && strlen(pdata->firmware_name)) {
		dev_dbg(&rmi_dev->dev, "Using platform data firmware name: %s\n", pdata->firmware_name);
		snprintf(firmware_name, NAME_BUFFER_SIZE, FW_NAME_FORMAT,
			pdata->firmware_name);
	} else {
		if (!strlen(data->f01_props.product_id)) {
			dev_err(&rmi_dev->dev, "Product ID is missing or empty - will not reflash.\n");
			goto done;
		}
		dev_dbg(&rmi_dev->dev, "Using F01 product ID: %s.\n", data->f01_props.product_id);
		snprintf(firmware_name, NAME_BUFFER_SIZE, FW_NAME_FORMAT,
			data->f01_props.product_id);
	}
	dev_info(&rmi_dev->dev, "Requesting %s.\n", firmware_name);
	retval = request_firmware(&fw_entry, firmware_name, &rmi_dev->dev);
	if (retval != 0) {
		dev_err(&rmi_dev->dev, "Firmware %s not available, code = %d\n",
			firmware_name, retval);
		goto done;
	}

	dev_dbg(&rmi_dev->dev, "Got firmware, size: %d.\n", fw_entry->size);
	extract_header(fw_entry->data, 0, header);
	dev_dbg(&rmi_dev->dev, "Img checksum:           %#08X\n",
		header->checksum);
	dev_dbg(&rmi_dev->dev, "Img io:                 %#04X\n",
		header->io);
	dev_dbg(&rmi_dev->dev, "Img image size:         %d\n",
		header->image_size);
	dev_dbg(&rmi_dev->dev, "Img config size:        %d\n",
		header->config_size);
	dev_dbg(&rmi_dev->dev, "Img bootloader version: %d\n",
		header->bootloader_version);
	dev_dbg(&rmi_dev->dev, "Img product id:         %s\n",
		header->product_id);
	dev_dbg(&rmi_dev->dev, "Img product info:       %#04x %#04x\n",
		header->product_info[0], header->product_info[1]);
	if (header->io == 1) {
		dev_dbg(&rmi_dev->dev, "Img Packrat:            %d\n",
			header->fw_build_id);
		dev_dbg(&rmi_dev->dev, "Img package:            %d\n",
			header->package_id);
	}

	if (header->image_size)
		data->firmware_data = fw_entry->data + F34_FW_IMAGE_OFFSET;
	if (header->config_size)
		data->config_data = fw_entry->data + F34_FW_IMAGE_OFFSET +
			header->image_size;

	if (go_nogo(data, header)) {
		dev_dbg(&rmi_dev->dev, "Go/NoGo said go.\n");
		reflash_firmware(data);
		reset_device(data);
	} else
		dev_dbg(&rmi_dev->dev, "Go/NoGo said don't reflash.\n");

	if (fw_entry)
		release_firmware(fw_entry);


done:
	getnstimeofday(&end);
	duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
	dev_dbg(&rmi_dev->dev, "Time to reflash: %lld ns.\n", duration_ns);

	kfree(data);
	kfree(firmware_name);
	kfree(header);
	return;
}

/*
 * We also reflash the device if (a) in kernel reflashing is
 * enabled, and (b) the reflash module decides it requires reflashing.
 *
 * We have to do this before actually building the PDT because the reflash
 * might cause various registers to move around.
 */
static int rmi_device_reflash(struct rmi_device *rmi_dev)
{
	struct device *dev = &rmi_dev->dev;
	struct rmi_driver_data *drv_data = dev_get_drvdata(dev);
	struct rmi_reflash_data* data = drv_data->reflash_data;
	int retval;

	retval = find_f01_and_f34(data);
	if (retval < 0)
		return retval;

	rmi4_fw_update(rmi_dev);

	clear_bit(0, &data->busy);

	return 0;
}

static ssize_t rmi_driver_img_name_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data* data = drv_data->reflash_data;

	return snprintf(buf, PAGE_SIZE, "%s\n", data->img_name);
}

static ssize_t rmi_driver_img_name_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data* data = drv_data->reflash_data;

	if (data->name_buf_len < count) {
		kfree(data->name_buf);
		data->name_buf = kzalloc(count, GFP_KERNEL);
		data->name_buf_len = count;
	}

	if (!count) {
		data->img_name = NULL;
		return count;
	}

	strncpy(data->name_buf, buf, count);
	data->img_name = strstrip(data->name_buf);

	return count;
}

static ssize_t rmi_driver_force_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data* data = drv_data->reflash_data;

	return snprintf(buf, PAGE_SIZE, "%u\n", data->force);
}

static ssize_t rmi_driver_force_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data* data = drv_data->reflash_data;
	int retval;
	unsigned long val;

	if (test_and_set_bit(0, &data->busy))
		return -EBUSY;

	retval = kstrtoul(buf, 10, &val);
	if (retval)
		count = retval;
	else
		data->force = !!val;

	clear_bit(0, &data->busy);

	return count;
}

static ssize_t rmi_driver_reflash_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data* data = drv_data->reflash_data;

	return snprintf(buf, PAGE_SIZE, "%u\n", test_bit(0, &data->busy));
}

static ssize_t rmi_driver_reflash_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int retval;
	unsigned long val;
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data* data = drv_data->reflash_data;

	retval = kstrtoul(buf, 10, &val);
	if (retval)
		return retval;

	if (test_and_set_bit(0, &data->busy))
		return -EBUSY;

	if (val)
		/*
		 * TODO: Here we start a work thread to go do the reflash, but
		 * maybe we can just use request_firmware_timeout().
		 */
		schedule_work(&data->reflash_work);
	else
		clear_bit(0, &data->busy);

	return count;
}

static void rmi_reflash_work(struct work_struct *work)
{
	struct rmi_reflash_data *data =
		container_of(work, struct rmi_reflash_data, reflash_work);
	struct rmi_device *rmi_dev = data->rmi_dev;
	int error;

	dev_dbg(&rmi_dev->dev, "%s runs.\n", __func__);
	error = rmi_device_reflash(rmi_dev);
	if (error < 0)
		dev_err(&rmi_dev->dev, "Reflash attempt failed with code: %d.",
			error);
	clear_bit(0, &data->busy);
}

static DEVICE_ATTR(reflash_force,
			(S_IRUGO | S_IWUGO),
			rmi_driver_force_show, rmi_driver_force_store);
static DEVICE_ATTR(reflash_name,
			(S_IRUGO | S_IWUGO),
			rmi_driver_img_name_show, rmi_driver_img_name_store);
static DEVICE_ATTR(reflash,
			(S_IRUGO | S_IWUGO),
			rmi_driver_reflash_show, rmi_driver_reflash_store);

static struct attribute *reflash_attrs[] = {
	&dev_attr_reflash_force.attr,
	&dev_attr_reflash_name.attr,
	&dev_attr_reflash.attr,
	NULL
};

static const struct attribute_group reflash_attributes = {
	.attrs = reflash_attrs,
};

/**
 * Initialize the reflash support structures for the specified device.
 */
void rmi_reflash_init(struct rmi_device* rmi_dev)
{
	int error;
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data *data;

	dev_dbg(&rmi_dev->dev, "%s called.\n", __func__);

	data = devm_kzalloc(&rmi_dev->dev, sizeof(struct rmi_reflash_data), GFP_KERNEL);

	error = sysfs_create_group(&rmi_dev->dev.kobj, &reflash_attributes);
	if (error) {
		dev_warn(&rmi_dev->dev, "Failed to create reflash sysfs attributes.\n");
		return;
	}

	INIT_WORK(&data->reflash_work, rmi_reflash_work);
	data->rmi_dev = rmi_dev;
	drv_data->reflash_data = data;
}

/** Clean up the devices reflash support structures.
 */
void rmi_reflash_cleanup(struct rmi_device* rmi_dev)
{
	struct rmi_driver_data *drv_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_reflash_data *data = drv_data->reflash_data;

	sysfs_remove_group(&rmi_dev->dev.kobj, &reflash_attributes);
	devm_kfree(&rmi_dev->dev, data);
}
