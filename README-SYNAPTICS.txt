This is a tarball of the current Synaptics kernel.org touch device driver, as
under development.  Included in the tarball are the full sources for the driver,
along with some related files.

NOTE: This code is under development, and is likely to change substantially
over the near future.  Not all features may work as expected, and the behavior
of some features may change over time.

IMPORTANT: DO NOT just unpack this tarball directly into your kernel tree.
Please follow the steps below.

These files and instructions are targeted to Linux kernel versions 3.4.x and up.
For instructions on use with early versions, please contact Christopher
Heiny <cheiny@synaptics.com>.

Files in the tarball are:

      README-SYNAPTICS.txt
          This file.
      CHANGELOG-SYNAPTICS.txt
          Important changes since the last tarball.
      include/linux/rmi.h
          Public header file for the driver.
      drivers/input/rmi4/rmi*.[ch]
          Headers and source files for the driver.
      drivers/input/rmi4/Makefile
          Builds the driver.
      drivers/input/rmi4/Kconfig
          Config file
      drivers/input/Makefile
          For reference.
      drivers/input/Kconfig
          For reference.
      drivers/hid/hid-core.c
          Example of entries in the hid_have_special_driver[] for Synaptics touch devices so
	  that the RMI4 driver will be loaded for our devices.
      arch/arm/mach-omap2/board-omap4panda.c
          A testing board file, for reference.
      arch/arm/configs/panda_defconfig
          A testing kernel configuration, for reference.
      arch/arm/mach-msm/lge/mako/board-mako-input.c
          Board file for Nexus 4, for reference.
      arch/arm/configs/mako_defconfig
          Kernel config file for Nexus 4, for reference.
      include/linux/input.h
          Example of BUS_RMI definition (pre v3.8).
      include/uapi/linux/input.h
          Example of BUS_RMI definition (newer kernels moved the BUS_* definitions, post v3.8).


To apply the new driver codebase to your current system, please follow
the steps below:

+ This step is needed ONLY if you were previous using an older version of the
Synaptics driver that had sources in drivers/input/touchscreen.  In this case
you need to do the following before proceeding to the next step.
    * remove the files drivers/input/touchscreen/rmi*
    * edit drivers/input/touchscreen/Makefile and
drivers/input/touchscreen/KConfig to remove RMI driver references


+ If you are updating from a previous tarball of the Synaptics driver, review
the file CHANGELOG-SYNAPTICS.txt for any important changes that might have
occurred since the last tarball was issued.


+ copy the drivers/input/rmi4 folder from the tarball into your kernel tree


+ copy include/linux/rmi.h from the tarball into include/linux in your
kernel tree


+ For kernels through version 3.6.xxx, add to the following line to
include/linux/input.h, immediately after the line defining BUS_SPI
    #define BUS_RMI                   0x1D
For version 3.7.xxx kernels, the input definitions have moved around.  Find
the file that defines BUS_SPI, and add the above line to it, after the BUS_SPI
definition.

+ Add entries to the hid_have_special_driver[] list in drivers/hid/hid-core.c.
The last two entries in the included hid-core.c file with the vendor ID USB_VENDOR_ID_SYNAPTICS
should be added to the hid_have_special_driver[] in your kernel tree. For kernels 3.10.xxx, the
HID_DEVICE macro added a group parameter which should be HID_GROUP_ANY. Older kernels should
remove the second parameter.

+ edit drivers/input/Makefile and drivers/input/KConfig to reference
drivers/input/rmi4 appropriately (see the reference files in the tarball)


+ use make menuconfig to update your configuration and select the appropriate
RMI4 features for you sensor.  You may safely omit function implementations
that are not present on your sensor.


+ (alternative to make menuconfig) edit your defconfig file to remove
TOUCHSCREEN_SYNAPTICS_RMI4_I2C and related symbols (if they are present);
replace them with the appropriate CONFIG_RMI4_XXX symbols.  See the
panda_defconfig file in the tarball for reference, and the
drivers/input/rmi4/Kconfig file for details on each setting.  The following
lines are required as a minimum for a functional RMI4 device:
    CONFIG_RMI4_CORE=y
and one of either
    CONFIG_RMI4_I2C=y
or
    CONFIG_RMI4_SPI=y
or
    CONFIG_RMI4_HID=y (for HID/I2C or HID/USB)
depending on your system.  You must also enable any RMI4 functions that are
present on your sensor, for example to enable F11 (2D pointing) and F34
(device reflash), add the lines
    CONFIG_RMI4_F11=y
    CONFIG_RMI4_F34=y
Consult with your product spec to determine just which functions are present
on your sensor. New sensors may use F12 for (2D pointing). If your sensor support F12
the add the line
    CONFIG_RMI4_F12=y

The minimal configuration for an F11 sensor on I2C is as follows:
    CONFIG_RMI4_CORE=y
    CONFIG_RMI4_I2C=y
    CONFIG_RMI4_F11=y
which will provide a driver for a touchpad or touchscreen.


+ update your board file to specify the appropriate platform data.  See the
attached board_omap4panda.c file for reference.  Also, see notes about
polling vs. interrupts, below.


+ do a make distclean to get rid of old object files and binaries


+ make defconfig to apply your configuration updates


+ finally, rebuild your kernel

HID/I2C
----------------------
To support HID/I2C devices your kernel tree will need to have the i2c-hid driver
(drivers/hid/i2c-hid/i2c-hid.c). This driver was added to the 3.8 kernels so older
kernels will need to have this driver added. Checking out a copy of the 3.8 kernel and 
copying it into the 3.4 or 3.5 kernel tree seems to work. For kernels newer then 3.11.xxx,
git commit 811adb9622de310efbb661531c3ec0ae5d2b2bc0 will need to be applied to i2c-hid.c to
support HID output reports on our devices.

POLLING VS. INTERRUPTS (Not applicable for HID/I2C or HID/USB)
----------------------

The RMI4 interface is designed to be driven by a sensor interrupt (ATTN) from
the touch sensor to the host controller CPU.  This allows the system to operate
most efficiently in terms of CPU utilization, bus traffic, power consumption,
and user response time.

However, during the prototyping phase of new systems that utilize RMI4 sensors,
it sometimes happens that the ATTN line is not accessible for some reason (such
as not being routed from the sensor to the host CPU, missing pull-ups, and so
on).  In that case, you may wish to use polling to access the device.

You can configure the RMI4 driver to poll your device by setting the attn_gpio
field of rmi_device_platform_data to 0.  This indicates to the driver that no
ATTN pin is available, and it will automatically poll the sensor for touch
data.

The default poll interval is 13 milliseconds.  You can adjust this up or down
using the poll_interval_ms field of rmi_device_platform_data.

