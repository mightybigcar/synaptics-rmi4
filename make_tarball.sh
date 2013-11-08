#!/bin/bash
#
# Script to build a driver tarball and put it in the public tarball location.
#

tardate=`date --rfc-3339=date`
tarfile=synaptics-rmi4-driver-${tardate}.tgz
tartag=tarball-${tardate}

echo Creating ${tarfile}...
tar -czf /tmp/${tarfile} \
    --exclude drivers/input/rmi4/rmi_f30.c drivers/input/rmi4/rmi_f31.c \
    README-SYNAPTICS.txt CHANGELOG-SYNAPTICS.txt \
    Documentation/ABI/testing/*-rmi4 Documentation/input/rmi*.txt \
    drivers/input/Makefile drivers/input/Kconfig drivers/input/rmi4/Kconfig \
    drivers/input/rmi4/Makefile drivers/input/rmi4/*.[ch] include/linux/rmi.h \
    arch/arm/configs/panda_defconfig arch/arm/mach-omap2/board-omap4panda.c \
    include/linux/kconfig.h include/linux/input.h include/linux/device.h \
    include/linux/i2c.h \
    firmware/Makefile firmware/rmi4/*

retval=$?
if [ "${retval}" -ne "0" ] ; then
    echo "Exiting..."
    exit ${retval}
fi

echo Copying to venom...
scp /tmp/${tarfile} venom:/home/cheiny/public_html/kernel.org/tarball/
retval=$?
if [ "${retval}" -ne "0" ] ; then
    echo "Exiting..."
    exit ${retval}
fi

echo Creating tag \'${tartag}\' in git...
git tag ${tartag}
retval=$?
if [ "${retval}" -ne "0" ] ; then
    echo "Exiting..."
    exit ${retval}
fi

echo Pushing tag into git...
git push --tags origin synaptics-rmi4
retval=$?
if [ "${retval}" -ne "0" ] ; then
    echo "Exiting..."
    exit ${retval}
fi

echo "Success!"
