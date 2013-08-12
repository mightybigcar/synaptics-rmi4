#ifndef RMI_VERSION_H
#define RMI_VERSION_H

#define RMI_VERSION_MAJOR 1
#define RMI_VERSION_MINOR 8
#define RMI_VERSION_SUBMINOR 5

#define RMI_VERSION_BRANCH rmihid
#define RMI_EXTRA_NUMBER 1
#define RMI_EXTRA_STRING rmihid.0

#define rmi_tostr(s) #s
#define rmi_str(s) rmi_tostr(s)
#define RMI_DRIVER_VERSION rmi_str(RMI_VERSION_MAJOR) "." \
	rmi_str(RMI_VERSION_MINOR) "." \
	rmi_str(RMI_VERSION_SUBMINOR) "-" \
	rmi_str(RMI_EXTRA_STRING)

#endif