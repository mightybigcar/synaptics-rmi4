/*
 * Copyright (c) 2011-2013 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _RMI_H
#define _RMI_H
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/list.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/debugfs.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

enum rmi_attn_polarity {
	RMI_ATTN_ACTIVE_LOW = 0,
	RMI_ATTN_ACTIVE_HIGH = 1
};

/**
 * struct rmi_axis_alignment - target axis alignment
 * @swap_axes: set to TRUE if desired to swap x- and y-axis
 * @flip_x: set to TRUE if desired to flip direction on x-axis
 * @flip_y: set to TRUE if desired to flip direction on y-axis
 * @clip_X_low - reported X coordinates below this setting will be clipped to
 *               the specified value
 * @clip_X_high - reported X coordinates above this setting will be clipped to
 *               the specified value
 * @clip_Y_low - reported Y coordinates below this setting will be clipped to
 *               the specified value
 * @clip_Y_high - reported Y coordinates above this setting will be clipped to
 *               the specified value
 * @offset_X - this value will be added to all reported X coordinates
 * @offset_Y - this value will be added to all reported Y coordinates
 * @rel_report_enabled - if set to true, the relative reporting will be
 *               automatically enabled for this sensor.
 */
struct rmi_2d_axis_alignment {
	u32 swap_axes;
	bool flip_x;
	bool flip_y;
	int clip_X_low;
	int clip_Y_low;
	int clip_X_high;
	int clip_Y_high;
	int offset_X;
	int offset_Y;
	u8 delta_x_threshold;
	u8 delta_y_threshold;
};

/**
 * struct virtualbutton_map - describes rectangular areas of a 2D sensor that
 * will be used by the driver to generate button events.
 *
 * @x - the x position of the low order corner of the rectangle, in RMI4
 * position units.
 * @y - the y position of the low order corner of the rectangle, in RMI4
 * position units.
 * @width - the width of the rectangle, in RMI4 position units.
 * @height - the height of the rectangle, in RMI4 position units.
 * @code - the input subsystem key event code that will be generated when a
 * tap occurs within the rectangle.
 */
struct virtualbutton_map {
	u16 x;
	u16 y;
	u16 width;
	u16 height;
	u16 code;
};

/**
 * struct rmi_f11_virtualbutton_map - provides a list of virtual buttons for
 * a 2D sensor.
 *
 * @buttons - the number of entries in the map.
 * @map - an array of virtual button descriptions.
 */
struct rmi_f11_virtualbutton_map {
	u8 buttons;
	struct virtualbutton_map *map;
};

/** This is used to override any hints an F11 2D sensor might have provided
 * as to what type of sensor it is.
 *
 * @rmi_sensor_default - do not override, determine from F11_2D_QUERY14 if
 * available.
 * @rmi_sensor_touchscreen - treat the sensor as a touchscreen (direct
 * pointing).
 * @rmi_sensor_touchpad - thread the sensor as a touchpad (indirect
 * pointing).
 */
enum rmi_sensor_type {
	rmi_sensor_default = 0,
	rmi_sensor_touchscreen,
	rmi_sensor_touchpad
};

/**
 * struct rmi_f11_sensor_data - overrides defaults for a single F11 2D sensor.
 * @axis_align - provides axis alignment overrides (see above).
 * @virtual_buttons - describes areas of the touch sensor that will be treated
 *                    as buttons.
 * @type_a - all modern RMI F11 firmwares implement Multifinger Type B
 * protocol.  Set this to true to force MF Type A behavior, in case you find
 * an older sensor.
 * @sensor_type - Forces the driver to treat the sensor as an indirect
 * pointing device (touchpad) rather than a direct pointing device
 * (touchscreen).  This is useful when F11_2D_QUERY14 register is not
 * available.
 */
struct rmi_f11_sensor_data {
	struct rmi_2d_axis_alignment axis_align;
	struct rmi_f11_virtualbutton_map virtual_buttons;
	bool type_a;
	enum rmi_sensor_type sensor_type;
	u8 suppress_highw;
	int x_mm;
	int y_mm;
};

struct rmi_f12_sensor_data {
	struct rmi_2d_axis_alignment axis_align;
	enum rmi_sensor_type sensor_type;
	u8 suppress_highw;
	int x_mm;
	int y_mm;
};

/**
 * struct rmi_f01_power - override default power management settings.
 *
 */
enum rmi_f01_nosleep {
	RMI_F01_NOSLEEP_DEFAULT = 0,
	RMI_F01_NOSLEEP_OFF = 1,
	RMI_F01_NOSLEEP_ON = 2
};

/**
 * struct rmi_f01_power_management -When non-zero, these values will be written
 * to the touch sensor to override the default firmware settigns.  For a
 * detailed explanation of what each field does, see the corresponding
 * documention in the RMI4 specification.
 *
 * @nosleep - specifies whether the device is permitted to sleep or doze (that
 * is, enter a temporary low power state) when no fingers are touching the
 * sensor.
 * @wakeup_threshold - controls the capacitance threshold at which the touch
 * sensor will decide to wake up from that low power state.
 * @doze_holdoff - controls how long the touch sensor waits after the last
 * finger lifts before entering the doze state, in units of 100ms.
 * @doze_interval - controls the interval between checks for finger presence
 * when the touch sensor is in doze mode, in units of 10ms.
 */
struct rmi_f01_power_management {
	enum rmi_f01_nosleep nosleep;
	u8 wakeup_threshold;
	u8 doze_holdoff;
	u8 doze_interval;
};

/**
 * struct rmi_button_map - used to specify the initial input subsystem key
 * event codes to be generated by buttons (or button like entities) on the
 * touch sensor.
 * @nbuttons - length of the button map.
 * @map - the key event codes for the corresponding buttons on the touch
 * sensor.
 */
struct rmi_button_map {
	u8 nbuttons;
	u8 *map;
};

struct rmi_f30_gpioled_map {
	u8 ngpioleds;
	u16 *map;
};

/**
 * struct rmi_device_platform_data_spi - provides parameters used in SPI
 * communications.  All Synaptics SPI products support a standard SPI
 * interface; some also support what is called SPI V2 mode, depending on
 * firmware and/or ASIC limitations.  In V2 mode, the touch sensor can
 * support shorter delays during certain operations, and these are specified
 * separately from the standard mode delays.
 *
 * @block_delay - for standard SPI transactions consisting of both a read and
 * write operation, the delay (in microseconds) between the read and write
 * operations.
 * @split_read_block_delay_us - for V2 SPI transactions consisting of both a
 * read and write operation, the delay (in microseconds) between the read and
 * write operations.
 * @read_delay_us - the delay between each byte of a read operation in normal
 * SPI mode.
 * @write_delay_us - the delay between each byte of a write operation in normal
 * SPI mode.
 * @split_read_byte_delay_us - the delay between each byte of a read operation
 * in V2 mode.
 * @pre_delay_us - the delay before the start of a SPI transaction.  This is
 * typically useful in conjunction with custom chip select assertions (see
 * below).
 * @post_delay_us - the delay after the completion of an SPI transaction.  This
 * is typically useful in conjunction with custom chip select assertions (see
 * below).
 * @cs_assert - For systems where the SPI subsystem does not control the CS/SSB
 * line, or where such control is broken, you can provide a custom routine to
 * handle a GPIO as CS/SSB.  This routine will be called at the beginning and
 * end of each SPI transaction.  The RMI SPI implementation will wait
 * pre_delay_us after this routine returns before starting the SPI transfer;
 * and post_delay_us after completion of the SPI transfer(s) before calling it
 * with assert==FALSE.
 */
struct rmi_device_platform_data_spi {
	int block_delay_us;
	int split_read_block_delay_us;
	int read_delay_us;
	int write_delay_us;
	int split_read_byte_delay_us;
	int pre_delay_us;
	int post_delay_us;

	void *cs_assert_data;
	int (*cs_assert) (const void *cs_assert_data, const bool assert);
};

struct rmi_device_sensor_vendor {
	int vendor_id;
	const char * vendor_name;
};

/**
 * struct rmi_device_platform_data - system specific configuration info.
 *
 * @sensor_name - this is used for various diagnostic messages.
 *
 * @firmware_name - if specified will override default firmware name,
 * for reflashing.
 *
 * @attn_gpio - the index of a GPIO that will be used to provide the ATTN
 * interrupt from the touch sensor.
 * @attn_polarity - indicates whether ATTN is active high or low.
 * @level_triggered - by default, the driver uses edge triggered interrupts.
 * However, this can cause problems with suspend/resume on some platforms.  In
 * that case, set this to 1 to use level triggered interrupts.
 * @gpio_config - a routine that will be called when the driver is loaded to
 * perform any platform specific GPIO configuration, and when it is unloaded
 * for GPIO de-configuration.  This is typically used to configure the ATTN
 * GPIO and the I2C or SPI pins, if necessary.
 * @gpio_data - platform specific data to be passed to the GPIO configuration
 * function.
 *
 * @poll_interval_ms - the time in milliseconds between reads of the interrupt
 * status register.  This is ignored if attn_gpio is non-zero.
 *
 * @reset_delay_ms - after issuing a reset command to the touch sensor, the
 * driver waits a few milliseconds to give the firmware a chance to
 * to re-initialize.  You can override the default wait period here.
 *
 * @spi_data - override default settings for SPI delays and SSB management (see
 * above).
 *
 * @f11_sensor_data - an array of platform data for individual F11 2D sensors.
 * @f11_sensor_count - the length of f11_sensor_data array.  Extra entries will
 * be ignored; if there are too few entries, all settings for the additional
 * sensors will be defaulted.
 * @f11_rezero_wait - if non-zero, this is how may milliseconds the F11 2D
 * sensor(s) will wait before being be rezeroed on exit from suspend.  If
 * this value is zero, the F11 2D sensor(s) will not be rezeroed on resume.
 * @pre_suspend - this will be called before any other suspend operations are
 * done.
 * @power_management - overrides default touch sensor doze mode settings (see
 * above)
 * @f19_button_map - provide initial input subsystem key mappings for F19.
 * @f1a_button_map - provide initial input subsystem key mappings for F1A.
 * @gpioled_map - provides initial settings for GPIOs and LEDs controlled by
 * F30.
 * @f41_button_map - provide initial input subsystem key mappings for F41.
 *
 * @post_suspend - this will be called after all suspend operations are
 * completed.  This is the ONLY safe place to power off an RMI sensor
 * during the suspend process.
 * @pre_resume - this is called before any other resume operations.  If you
 * powered off the RMI4 sensor in post_suspend(), then you MUST power it back
 * here, and you MUST wait an appropriate time for the ASIC to come up
 * (100ms to 200ms, depending on the sensor) before returning.
 * @pm_data - this will be passed to the various (pre|post)_(suspend/resume)
 * functions.
 */
struct rmi_device_platform_data {
	char *sensor_name;	/* Used for diagnostics. */

	int attn_gpio;
	enum rmi_attn_polarity attn_polarity;
	bool level_triggered;
	void *gpio_data;
	int (*gpio_config)(void *gpio_data, bool configure);

	int poll_interval_ms;

	int reset_delay_ms;

	struct rmi_device_platform_data_spi spi_data;

	/* function handler pdata */
	struct rmi_f11_sensor_data *f11_sensor_data;
	struct rmi_f12_sensor_data *f12_sensor_data;
	u8 f11_sensor_count;
	u16 f11_rezero_wait;
	struct rmi_f01_power_management power_management;
	struct rmi_button_map *f19_button_map;
	struct rmi_button_map *f1a_button_map;
	struct rmi_f30_gpioled_map *gpioled_map;
	struct rmi_button_map *f41_button_map;
	u8 unified_input_device;

#ifdef CONFIG_RMI4_FWLIB
	char *firmware_name;
	u8 multi_sensor_vendor_count;
	struct rmi_device_sensor_vendor * sensor_vendor_id;
#endif

#ifdef	CONFIG_PM
	void *pm_data;
	int (*pre_suspend) (const void *pm_data);
	int (*post_suspend) (const void *pm_data);
	int (*pre_resume) (const void *pm_data);
	int (*post_resume) (const void *pm_data);
#endif
};

/**
 * struct rmi_function_descriptor - RMI function base addresses
 *
 * @query_base_addr: The RMI Query base address
 * @command_base_addr: The RMI Command base address
 * @control_base_addr: The RMI Control base address
 * @data_base_addr: The RMI Data base address
 * @interrupt_source_count: The number of irqs this RMI function needs
 * @function_number: The RMI function number
 *
 * This struct is used when iterating the Page Description Table. The addresses
 * are 16-bit values to include the current page address.
 *
 */
struct rmi_function_descriptor {
	u16 query_base_addr;
	u16 command_base_addr;
	u16 control_base_addr;
	u16 data_base_addr;
	u8 interrupt_source_count;
	u8 function_number;
	u8 function_version;
};


/**
 * Helper fn to convert a byte array representing a 16 bit value in the RMI
 * endian-ness to a 16-bit value in the native processor's specific endianness.
 * We don't use ntohs/htons here because, well, we're not dealing with
 * a pair of 16 bit values. Casting dest to u16* wouldn't work, because
 * that would imply knowing the byte order of u16 in the first place.  The
 * same applies for using shifts and masks.
 */
static inline u16 batohs(u8 *src)
{
	return src[1] << 8 | src[0];
}
/**
 * Helper function to convert a 16 bit value (in host processor endianess) to
 * a byte array in the RMI endianess for u16s.  See above comment for
 * why we dont us htons or something like that.
 */
static inline void hstoba(u8 *dest, u16 src)
{
	dest[0] = src & 0xFF;
	dest[1] =  src >> 8;
}

#endif
