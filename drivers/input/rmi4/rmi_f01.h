/*
 * Copyright (c) 2012 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#ifndef _RMI_F01_H
#define _RMI_F01_H

#define RMI_PRODUCT_ID_LENGTH    10
#define RMI_PRODUCT_INFO_LENGTH   2

#define RMI_DATE_CODE_LENGTH      3

#define PRODUCT_ID_OFFSET 0x10
#define PRODUCT_INFO_OFFSET 0x1E

#define F01_RESET_MASK 0x01

/* Force a firmware reset of the sensor */
#define RMI_F01_CMD_DEVICE_RESET	1

#define F01_SERIALIZATION_SIZE 7

/* Various F01_RMI_QueryX bits */

#define RMI_F01_QRY1_CUSTOM_MAP		(1 << 0)
#define RMI_F01_QRY1_NON_COMPLIANT	(1 << 1)
#define RMI_F01_QRY1_HAS_LTS		(1 << 2)
#define RMI_F01_QRY1_HAS_SENSOR_ID	(1 << 3)
#define RMI_F01_QRY1_HAS_CHARGER_INP	(1 << 4)
#define RMI_F01_QRY1_HAS_ADJ_DOZE	(1 << 5)
#define RMI_F01_QRY1_HAS_ADJ_DOZE_HOFF	(1 << 6)
#define RMI_F01_QRY1_HAS_PROPS_2	(1 << 7)

#define RMI_F01_QRY5_YEAR_MASK		0x1f
#define RMI_F01_QRY6_MONTH_MASK		0x0f
#define RMI_F01_QRY7_DAY_MASK		0x1f

#define RMI_F01_QRY2_PRODINFO_MASK	0x7f

#define RMI_F01_BASIC_QUERY_LEN		21 /* From Query 00 through 20 */

#define RMI_F01_QRY21_SLAVE_ROWS_MASK   0x07
#define RMI_F01_QRY21_SLAVE_COLUMNS_MASK 0x38

#define RMI_F01_LTS_RESERVED_SIZE 19

#define RMI_F01_QRY42_DS4_QUERIES	(1 << 0)
#define RMI_F01_QRY42_MULTI_PHYS	(1 << 1)
#define RMI_F01_QRY42_GUEST		(1 << 2)
#define RMI_F01_QRY42_SWR		(1 << 3)
#define RMI_F01_QRY42_NOMINAL_REPORT	(1 << 4)
#define RMI_F01_QRY42_RECAL_INTERVAL	(1 << 5)

#define RMI_F01_QRY43_01_PACKAGE_ID     (1 << 0)
#define RMI_F01_QRY43_01_BUILD_ID       (1 << 1)
#define RMI_F01_QRY43_01_RESET          (1 << 2)
#define RMI_F01_QRY43_01_MASK_REV       (1 << 3)

#define RMI_F01_QRY43_02_I2C_CTL	(1 << 0)
#define RMI_F01_QRY43_02_SPI_CTL	(1 << 1)
#define RMI_F01_QRY43_02_ATTN_CTL	(1 << 2)
#define RMI_F01_QRY43_02_WIN8		(1 << 3)
#define RMI_F01_QRY43_02_TIMESTAMP	(1 << 4)

#define RMI_F01_QRY43_03_TOOL_ID	(1 << 0)
#define RMI_F01_QRY43_03_FW_REVISION	(1 << 1)

#define RMI_F01_QRY44_RST_ENABLED	(1 << 0)
#define RMI_F01_QRY44_RST_POLARITY	(1 << 1)
#define RMI_F01_QRY44_PULLUP_ENABLED	(1 << 2)
#define RMI_F01_QRY44_RST_PIN_MASK	0xF0

#define RMI_TOOL_ID_LENGTH		16
#define RMI_FW_REVISION_LENGTH		16

struct f01_basic_properties {
	u8 manufacturer_id;
	bool has_lts;
	bool has_sensor_id;
	bool has_adjustable_doze;
	bool has_adjustable_doze_holdoff;
	bool has_query42;
	char dom[11]; /* YYYY/MM/DD + '\0' */
	u8 product_id[RMI_PRODUCT_ID_LENGTH + 1];
	u16 productinfo;

	/* These are meaningful only if has_lts is true. */
	u8 slave_asic_rows;
	u8 slave_asic_columns;

	/* This is meaningful only if has_sensor_id is true. */
	u8 sensor_id;

	/* These are meaningful only if has_query42 is true. */
	bool has_ds4_queries;
	bool has_multi_physical;
	bool has_guest;
	bool has_swr;
	bool has_nominal_report_rate;
	bool has_recalibration_interval;

	/* Tells how many of the Query43.xx registers are present.
	 */
	u8 ds4_query_length;

	/* Query 43.1 */
	bool has_package_id_query;
	bool has_build_id_query;
	bool has_reset_query;
	bool has_maskrev_query;

	/* Query 43.2 */
	bool has_i2c_control;
	bool has_spi_control;
	bool has_attn_control;
	bool has_win8_vendor_info;
	bool has_timestamp;

	/* Query 43.3 */
	bool has_tool_id_query;
	bool has_fw_revision_query;

	/* Query 44 */
	bool reset_enabled;
	bool reset_polarity;
	bool pullup_enabled;
	u8 reset_pin;

	/* Query 45 */
	char tool_id[RMI_TOOL_ID_LENGTH + 1];

	/* Query 46 */
	char fw_revision[RMI_FW_REVISION_LENGTH + 1];
};

/** The status code field reports the most recent device status event.
 * @no_error - should be self explanatory.
 * @reset_occurred - no other event was seen since the last reset.
 * @invalid_config - general device configuration has a problem.
 * @device_failure - general device hardware failure.
 * @config_crc - configuration failed memory self check.
 * @firmware_crc - firmware failed memory self check.
 * @crc_in_progress - bootloader is currently testing config and fw areas.
 */
enum rmi_device_status {
	no_error = 0x00,
	reset_occurred = 0x01,
	invalid_config = 0x02,
	device_failure = 0x03,
	config_crc = 0x04,
	firmware_crc = 0x05,
	crc_in_progress = 0x06
};


/* F01 device status bits */

/* Most recent device status event */
#define RMI_F01_STATUS_CODE(status)		((status) & 0x0f)
/* Indicates that flash programming is enabled (bootloader mode). */
#define RMI_F01_STATUS_BOOTLOADER(status)	(!!((status) & 0x40))
/* The device has lost its configuration for some reason. */
#define RMI_F01_STATUS_UNCONFIGURED(status)	(!!((status) & 0x80))



/* Control register bits */

/*
* Sleep mode controls power management on the device and affects all
* functions of the device.
*/
#define RMI_F01_CTRL0_SLEEP_MODE_MASK	0x03

#define RMI_SLEEP_MODE_NORMAL		0x00
#define RMI_SLEEP_MODE_SENSOR_SLEEP	0x01
#define RMI_SLEEP_MODE_RESERVED0	0x02
#define RMI_SLEEP_MODE_RESERVED1	0x03

#define RMI_IS_VALID_SLEEPMODE(mode) \
(mode >= RMI_SLEEP_MODE_NORMAL && mode <= RMI_SLEEP_MODE_RESERVED1)

/*
 * This bit disables whatever sleep mode may be selected by the sleep_mode
 * field and forces the device to run at full power without sleeping.
 */
#define RMI_F01_CRTL0_NOSLEEP_BIT	(1 << 2)

/*
 * When this bit is set, the touch controller employs a noise-filtering
 * algorithm designed for use with a connected battery charger.
 */
#define RMI_F01_CRTL0_CHARGER_BIT	(1 << 5)

/*
 * Sets the report rate for the device. The effect of this setting is
 * highly product dependent. Check the spec sheet for your particular
 * touch sensor.
 */
#define RMI_F01_CRTL0_REPORTRATE_BIT	(1 << 6)

/*
 * Written by the host as an indicator that the device has been
 * successfully configured.
 */
#define RMI_F01_CRTL0_CONFIGURED_BIT	(1 << 7)

/**
 * @ctrl0 - see documentation in rmi_f01.h.
 * @interrupt_enable - A mask of per-function interrupts on the touch sensor.
 * @doze_interval - controls the interval between checks for finger presence
 * when the touch sensor is in doze mode, in units of 10ms.
 * @wakeup_threshold - controls the capacitance threshold at which the touch
 * sensor will decide to wake up from that low power state.
 * @doze_holdoff - controls how long the touch sensor waits after the last
 * finger lifts before entering the doze state, in units of 100ms.
 */
struct f01_device_control {
	u8 ctrl0;
	u8 *interrupt_enable;
	u8 doze_interval;
	u8 wakeup_threshold;
	u8 doze_holdoff;
};


/*
 *
 * @serialization - 7 bytes of device serialization data.  The meaning of
 * these bytes varies from product to product, consult your product spec sheet.
 */
struct f01_data {
	struct f01_device_control device_control;
	struct mutex control_mutex;

	u8 device_status;

	struct f01_basic_properties properties;
	u8 serialization[F01_SERIALIZATION_SIZE];
	u8 product_id[RMI_PRODUCT_ID_LENGTH+1];

	u16 package_id;
	u16 package_rev;
	u32 build_id;

	u16 interrupt_enable_addr;
	u16 doze_interval_addr;
	u16 wakeup_threshold_addr;
	u16 doze_holdoff_addr;

	int irq_count;
	int num_of_irq_regs;

#ifdef	CONFIG_PM
	bool suspended;
	bool old_nosleep;
#endif
};

#endif
