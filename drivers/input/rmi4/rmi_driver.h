/*
 * Copyright (c) 2011-2013 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _RMI_DRIVER_H
#define _RMI_DRIVER_H

#include <linux/ctype.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/rmi.h>

#include "rmi_bus.h"
#include "rmi_version.h"

#define SYNAPTICS_INPUT_DEVICE_NAME "Synaptics RMI4 Touch Sensor"
#define SYNAPTICS_VENDOR_ID 0x06cb


#define DEFAULT_RESET_DELAY_MS	100

#define PDT_PROPERTIES_LOCATION 0x00EF
#define BSR_LOCATION 0x00FE

#define RMI_PDT_PROPS_HAS_BSR 0x02

struct rmi_driver_data {
	struct list_head function_list;
	struct rmi_device *rmi_dev;

	struct rmi_function *f01_dev;
	bool f01_bootloader_mode;

	u32 attn_count;
	u32 irq_debug;
	int irq;
	int irq_flags;
	int num_of_irq_regs;
	int irq_count;
	unsigned long *irq_status;
	unsigned long *current_irq_mask;
	unsigned long *irq_mask_store;
	bool irq_stored;
	struct mutex irq_mutex;

	/* Following are used when polling. */
	struct hrtimer poll_timer;
	struct work_struct poll_work;
	ktime_t poll_interval;
	struct mutex pdt_mutex;
	u8 pdt_props;
	u8 bsr;

	int board;
	int rev;

	bool enabled;
#ifdef CONFIG_PM
	bool suspended;
	struct mutex suspend_mutex;

	void *pm_data;
	int (*pre_suspend) (const void *pm_data);
	int (*post_suspend) (const void *pm_data);
	int (*pre_resume) (const void *pm_data);
	int (*post_resume) (const void *pm_data);
#endif

	void *data;
};


#define RMI_PDT_ENTRY_SIZE 6
#define RMI_PDT_FUNCTION_VERSION_MASK	0x60
#define RMI_PDT_INT_SOURCE_COUNT_MASK	0x07

#define PDT_START_SCAN_LOCATION 0x00e9
#define PDT_END_SCAN_LOCATION	0x0005
#define RMI4_END_OF_PDT(id) ((id) == 0x00 || (id) == 0xff)

struct pdt_entry {
	u8 query_base_addr;
	u8 command_base_addr;
	u8 control_base_addr;
	u8 data_base_addr;
	u8 interrupt_source_count;
	u8 function_version;
	u8 function_number;
};

int rmi_read_pdt_entry(struct rmi_device *rmi_dev, struct pdt_entry *entry,
			   u16 pdt_address);

#ifdef	CONFIG_RMI4_FWLIB
extern void rmi4_fw_update(struct rmi_device *rmi_dev,
		struct pdt_entry *f01_pdt, struct pdt_entry *f34_pdt);
#else
#define rmi4_fw_update(rmi_dev, f01_pdt, f34_pdt) 0
#endif

extern struct rmi_driver rmi_physical_driver;
extern struct rmi_function_driver rmi_f01_driver;

int rmi_register_sensor_driver(void);
void rmi_unregister_sensor_driver(void);

#endif
