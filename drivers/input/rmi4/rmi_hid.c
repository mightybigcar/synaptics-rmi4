/*
 *  Copyright (c) 2013 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/hid.h>
#include <linux/interrupt.h>
#include <linux/kconfig.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include "rmi_driver.h"

#define to_hid_device(d) container_of(d, struct hid_device, dev)

#define RMI_WRITE_REPORT_ID		0x9 /* Output Report */
#define RMI_READ_ADDR_REPORT_ID		0xa /* Output Report */
#define RMI_READ_DATA_REPORT_ID		0xb /* Input Report */
#define RMI_ATTN_REPORT_ID		0xc /* Input Report */
#define RMI_SET_RMI_MODE_REPORT_ID	0xf /* Feature Report */

/* report lengths 
 * TODO: figure out lengths from report descriptor
 */
#define RMI_HID_INPUT_REPORT_LEN	30
#define RMI_HID_OUTPUT_REPORT_LEN	21
#define RMI_HID_FEATURE_REPORT_LEN	2

#define RMI_VENDOR_SPECIFIC_USAGE_PAGE	0xFF00
#define RMI_VENDOR_SPECIFIC_USGAGE	0x1

/* flags */
#define RMI_HID_READ_REQUEST_PENDING	(1 << 0)
#define RMI_HID_READ_DATA_PENDING	(1 << 1)
#define RMI_HID_STARTED			(1 << 2)

#define RMI_HID_INPUT_REPORT_QUEUE_LEN	8

#define TOUCHPAD_WAKE_SYSTEM

enum rmi_hid_mode_type {
	RMI_HID_MODE_OFF 			= 0,
	RMI_HID_MODE_ATTN_REPORTS		= 1,
	RMI_HID_MODE_NO_PACKED_ATTN_REPORTS	= 2,
};

struct rmi_hid_read_output_report {
	u8  deprecatedReadCount;
	u16 addr;
	u16 readCount;
} __attribute__((__packed__));

struct rmi_hid_read_input_report {
	u8 readCount;
	u8 data[0];
} __attribute__((__packed__));

struct rmi_hid_write_output_report {
	u8  writeCount;
	u16 addr;
	u8  data[0];
} __attribute__((__packed__));

struct rmi_hid_feature_report {
	u8  mode;
} __attribute__((__packed__));

struct rmi_hid_attn_report {
	u8  interruptSources;
	u8  data[0];
} __attribute__((__packed__));

struct rmi_hid_report {
	u8  hidReportID;
	union {
		struct rmi_hid_read_output_report	read_out_report;
		struct rmi_hid_read_input_report 	read_in_report;
		struct rmi_hid_write_output_report	write_out_report;
		struct rmi_hid_feature_report		feature_report;
		struct rmi_hid_attn_report		attn_report;
		u8   data[0];
	} __attribute__((__packed__));
} __attribute__((__packed__));

static u16 tm2735_button_codes[] = {BTN_LEFT, BTN_RIGHT, BTN_MIDDLE};

static struct rmi_f30_gpioled_map tm2735_gpioled_map = {
	.ngpioleds = ARRAY_SIZE(tm2735_button_codes),
	.map = tm2735_button_codes,
};

static struct rmi_f11_sensor_data tm2735_sensor_data[] = { {
	.axis_align = {
		.flip_y = 1,
	},
	.sensor_type = rmi_f11_sensor_touchpad,
	.suppress_highw = 0,
} };

static int rmi_hid_set_mode(struct hid_device *hdev, u8 mode);

int rmi_hid_post_resume(const void *pm_data)
{
	struct hid_device *hdev = (struct hid_device *)pm_data;
	return rmi_hid_set_mode(hdev, RMI_HID_MODE_ATTN_REPORTS);
}

static struct rmi_device_platform_data tm2735_platformdata = {
	.sensor_name = "TM2735",
	.gpioled_map = &tm2735_gpioled_map,
	.f11_sensor_data = tm2735_sensor_data,
	.f11_sensor_count = ARRAY_SIZE(tm2735_sensor_data),
	.post_resume = rmi_hid_post_resume,
};

#define BUFFER_SIZE_INCREMENT 32
/**
 * struct rmi_hid_data - stores information for hid communication
 *
 * @page_mutex: Locks current page to avoid changing pages in unexpected ways.
 * @page: Keeps track of the current virtual page
 * @xport: Pointer to the transport interface
 *
 * @debug_buf: Buffer used for exposing buffer contents using dev_dbg
 * @debug_buf_size: Size of the debug buffer.
 *
 * @comms_debug: Latest data read/written for debugging HID communications
 * @debugfs_comms: Debugfs file for debugging HID communications
 *
 */
struct rmi_hid_data {
        struct mutex page_mutex;
        int page;
        struct rmi_transport_device *xport;

        u8 *debug_buf;
        int debug_buf_size;

	wait_queue_head_t wait;		/* For waiting for read data */
	u8 writeReport[RMI_HID_OUTPUT_REPORT_LEN];
	u8 readReport[RMI_HID_INPUT_REPORT_LEN];

	//struct mutex input_queue_mutex;
	spinlock_t input_queue_lock;
	struct rmi_hid_report * input_queue;
	int input_queue_head;
	int input_queue_tail;
	struct work_struct attn_report_work;

	unsigned long flags;

        bool comms_debug;
#ifdef CONFIG_RMI4_DEBUG
        struct dentry *debugfs_comms;
#endif
};

#ifdef CONFIG_RMI4_DEBUG

/**
 * struct hid_debugfs_data - stores information for debugfs
 *
 * @done: Indicates that we are done reading debug data. Subsequent reads
 * will return EOF.
 * @hid_data: Pointer to the hid data
 *
 */
struct hid_debugfs_data {
        bool done;
        struct rmi_hid_data *hid_data;
};

static int debug_open(struct inode *inodep, struct file *filp)
{
        struct hid_debugfs_data *data;

        data = kzalloc(sizeof(struct hid_debugfs_data), GFP_KERNEL);
        if (!data)
                return -ENOMEM;

        data->hid_data = inodep->i_private;
        filp->private_data = data;
        return 0;
}

static int debug_release(struct inode *inodep, struct file *filp)
{
        kfree(filp->private_data);
        return 0;
}

static ssize_t comms_debug_read(struct file *filp, char __user *buffer,
                size_t size, loff_t *offset) {
        int retval;
        char *local_buf;
        struct hid_debugfs_data *dfs = filp->private_data;
        struct rmi_hid_data *data = dfs->hid_data;

        if (dfs->done)
                return 0;

        local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
        if (!local_buf)
                return -ENOMEM;

        dfs->done = 1;

        retval = snprintf(local_buf, PAGE_SIZE, "%u\n", data->comms_debug);

        if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
                retval = -EFAULT;
        kfree(local_buf);

        return retval;
}

static ssize_t comms_debug_write(struct file *filp, const char __user *buffer,
                           size_t size, loff_t *offset) {
        int retval;
        char *local_buf;
        unsigned int new_value;
        struct hid_debugfs_data *dfs = filp->private_data;
        struct rmi_hid_data *data = dfs->hid_data;

        local_buf = kcalloc(size, sizeof(u8), GFP_KERNEL);
        if (!local_buf)
                return -ENOMEM;
        retval = copy_from_user(local_buf, buffer, size);
        if (retval) {
                kfree(local_buf);
                return -EFAULT;
        }

        retval = sscanf(local_buf, "%u", &new_value);
        kfree(local_buf);
        if (retval != 1 || new_value > 1)
                return -EINVAL;

        data->comms_debug = new_value;

        return size;
}


static const struct file_operations comms_debug_fops = {
        .owner = THIS_MODULE,
        .open = debug_open,
        .release = debug_release,
        .read = comms_debug_read,
        .write = comms_debug_write,
};

static int setup_debugfs(struct rmi_device *rmi_dev, struct rmi_hid_data *data)
{
        if (!rmi_dev->debugfs_root)
                return -ENODEV;

        data->debugfs_comms = debugfs_create_file("comms_debug", RMI_RW_ATTR,
                        rmi_dev->debugfs_root, data, &comms_debug_fops);
        if (!data->debugfs_comms || IS_ERR(data->debugfs_comms)) {
                dev_warn(&rmi_dev->dev, "Failed to create debugfs comms_debug.\n");
                data->debugfs_comms = NULL;
        }

        return 0;
}

static void teardown_debugfs(struct rmi_hid_data *data)
{
        if (data->debugfs_comms)
                debugfs_remove(data->debugfs_comms);
}
#else
#define setup_debugfs(rmi_dev, data) 0
#define teardown_debugfs(data)
#endif

#define COMMS_DEBUG(data) (IS_ENABLED(CONFIG_RMI4_DEBUG) && data->comms_debug)

#define RMI_PAGE_SELECT_REGISTER 0xff
#define RMI_HID_PAGE(addr) (((addr) >> 8) & 0xff)

static char *transport_proto_name = "hid";

static int rmi_hid_write_report(struct hid_device *hdev,
				struct rmi_hid_report * report, int len);
/*
 * rmi_set_page - Set RMI page
 * @xport: The pointer to the rmi_transport_device struct
 * @page: The new page address.
 *
 * RMI devices have 16-bit addressing, but some of the physical
 * implementations (like SMBus) only have 8-bit addressing. So RMI implements
 * a page address at 0xff of every page so we can reliable page addresses
 * every 256 registers.
 *
 * The page_mutex lock must be held when this function is entered.
 *
 * Returns zero on success, non-zero on failure.
 */
static int rmi_set_page(struct rmi_transport_device *xport, u8 page)
{
        struct hid_device *hdev = to_hid_device(xport->dev);
        struct rmi_hid_data *data = xport->data;
        u8 txbuf[RMI_HID_OUTPUT_REPORT_LEN];
	struct rmi_hid_report * writeReport = (struct rmi_hid_report *)txbuf;
        int retval;

	memset(txbuf, 0, RMI_HID_OUTPUT_REPORT_LEN);

        if (COMMS_DEBUG(data))
                dev_dbg(&hdev->dev, "writes 3 bytes: %02x %02x\n",
                        txbuf[0], txbuf[1]);

        xport->info.tx_count++;
        xport->info.tx_bytes += sizeof(txbuf);

	writeReport->hidReportID = RMI_WRITE_REPORT_ID;
	writeReport->write_out_report.writeCount = 1;
	writeReport->write_out_report.addr = 0xFF;
	writeReport->write_out_report.data[0] = page;

        retval = rmi_hid_write_report(hdev, writeReport,
					RMI_HID_OUTPUT_REPORT_LEN);
        if (retval != RMI_HID_OUTPUT_REPORT_LEN) {
                xport->info.tx_errs++;
                dev_err(&hdev->dev,
                        "%s: set page failed: %d.", __func__, retval);
                return retval;
        }

        data->page = page;
        return 0;
}

static int copy_to_debug_buf(struct device *dev, struct rmi_hid_data *data,
                             const u8 *buf, const int len)
{
        int i;
        int n = 0;
        char *temp;
        int dbg_size = 3 * len + 1;

        if (!data->debug_buf || data->debug_buf_size < dbg_size) {
                if (data->debug_buf)
                        devm_kfree(dev, data->debug_buf);
                data->debug_buf_size = dbg_size + BUFFER_SIZE_INCREMENT;
                data->debug_buf = devm_kzalloc(dev, data->debug_buf_size,
                                               GFP_KERNEL);
                if (!data->debug_buf) {
                        data->debug_buf_size = 0;
                        return -ENOMEM;
                }
        }
        temp = data->debug_buf;

        for (i = 0; i < len; i++) {
                n = sprintf(temp, " %02x", buf[i]);
                temp += n;
        }

        return 0;
}

static int rmi_hid_set_mode(struct hid_device *hdev, u8 mode)
{
	int ret;
	u8 txbuf[2] = {RMI_SET_RMI_MODE_REPORT_ID, mode};

	ret = hdev->hid_output_raw_report(hdev, txbuf, sizeof(txbuf), 
			HID_FEATURE_REPORT);
	if (ret < 0) {
		dev_err(&hdev->dev, "unable to set rmi mode to %d (%d)\n", mode,
			ret);
		return ret;
	}

	return 0;
}

static int rmi_hid_write_report(struct hid_device *hdev,
				struct rmi_hid_report * report, int len)
{
	int ret;

	ret = hdev->hid_output_raw_report(hdev, (void *)report, len,
		HID_OUTPUT_REPORT);
	if (ret < 0) {
		dev_err(&hdev->dev, "failed to write hid report (%d)\n", ret);
		return ret;
	}

	return ret;
}

static int rmi_hid_write_block(struct rmi_transport_device *xport, u16 addr,
			       const void *buf, const int len)
{
        struct hid_device *hdev = to_hid_device(xport->dev);
	struct rmi_hid_data *data = xport->data;
	struct rmi_hid_report * writeReport = (struct rmi_hid_report *)data->writeReport;
	int ret;

	mutex_lock(&data->page_mutex);

	if (RMI_HID_PAGE(addr) != data->page) {
		ret = rmi_set_page(xport, RMI_HID_PAGE(addr));
		if (ret < 0)
			goto exit;
	}

	if (COMMS_DEBUG(data)) {
		ret = copy_to_debug_buf(&hdev->dev, data, (u8 *) buf, len);
		if (!ret)
			dev_dbg(&hdev->dev, "writes %d bytes at %#06x:%s\n",
				len, addr, data->debug_buf);
	}

	xport->info.tx_count++;
	xport->info.tx_bytes += len;

	writeReport->hidReportID = RMI_WRITE_REPORT_ID;
	writeReport->write_out_report.writeCount = len;
	writeReport->write_out_report.addr = addr;
	memcpy(&writeReport->write_out_report.data, buf, len);

	ret = rmi_hid_write_report(hdev, writeReport,
					RMI_HID_OUTPUT_REPORT_LEN);
	if (ret != RMI_HID_OUTPUT_REPORT_LEN) {
		dev_err(&hdev->dev, "failed to send output report (%d)\n", ret);
		goto exit;
	}

exit:
	mutex_unlock(&data->page_mutex);
	return ret;
}

static int rmi_hid_read_block(struct rmi_transport_device *xport, u16 addr,
			      void *buf, const int len)
{
        struct hid_device *hdev = to_hid_device(xport->dev);
	struct rmi_hid_data *data = xport->data;
	struct rmi_hid_report * writeReport = (struct rmi_hid_report *)data->writeReport;
	struct rmi_hid_report * readReport = (struct rmi_hid_report *)data->readReport;
	int ret;
	int bytes_read;
	int bytes_needed;
	int retries;

	mutex_lock(&data->page_mutex);

	if (RMI_HID_PAGE(addr) != data->page) {
		ret = rmi_set_page(xport, RMI_HID_PAGE(addr));
		if (ret < 0)
			goto exit;
	}

	for (retries = 5; retries > 0; retries--) {
		writeReport->hidReportID = RMI_READ_ADDR_REPORT_ID;
		writeReport->read_out_report.deprecatedReadCount = 0;
		writeReport->read_out_report.addr = addr;
		writeReport->read_out_report.readCount = len;

		if (COMMS_DEBUG(data)) {
			ret = copy_to_debug_buf(&hdev->dev, data, (u8 *) writeReport, len);
			if (!ret)
				dev_dbg(&hdev->dev, "wrote %d bytes at %#06x:%s\n",
					len, addr, data->debug_buf);
		}

		set_bit(RMI_HID_READ_REQUEST_PENDING, &data->flags);

		ret = rmi_hid_write_report(hdev, writeReport,
					RMI_HID_OUTPUT_REPORT_LEN);
		if (ret != RMI_HID_OUTPUT_REPORT_LEN) {
			clear_bit(RMI_HID_READ_REQUEST_PENDING, &data->flags);
			dev_err(&hdev->dev,
				"failed to write request output report (%d)\n", ret);
			goto exit;
		}

		bytes_read = 0;
		bytes_needed = len;
		while (bytes_read < len) {
			if (!wait_event_timeout(data->wait, 
					test_bit(RMI_HID_READ_DATA_PENDING, &data->flags),
					msecs_to_jiffies(1000)))
			{
					dev_info(&hdev->dev, "%s: timeout elapsed\n", __func__);
						ret = -ENODATA;
						break;
			} else {
				if (readReport->hidReportID != RMI_READ_DATA_REPORT_ID) {
					ret = -ENODATA;
					goto exit;
				}

				memcpy(buf + bytes_read, readReport->read_in_report.data,
					readReport->read_in_report.readCount < bytes_needed 
					? readReport->read_in_report.readCount : bytes_needed);
				if (COMMS_DEBUG(data)) {
					ret = copy_to_debug_buf(&hdev->dev, data,
							(u8 *) buf + bytes_read,
							readReport->read_in_report.readCount);
					if (!ret)
						dev_dbg(&hdev->dev, "read %d bytes at %#06x:%s\n",
							readReport->read_in_report.readCount, addr,
							data->debug_buf);
				}
				bytes_read += readReport->read_in_report.readCount;
				bytes_needed -= readReport->read_in_report.readCount;
				clear_bit(RMI_HID_READ_DATA_PENDING, &data->flags);
			}
		}

		if (bytes_read == len)
			break;
	}

	xport->info.rx_count++;
	xport->info.rx_bytes += len;
	ret = len;

exit:
	clear_bit(RMI_HID_READ_REQUEST_PENDING, &data->flags);
	mutex_unlock(&data->page_mutex);
	return ret;
}

static void rmi_hid_attn_report_work(struct work_struct *work)
{
	struct rmi_hid_data * hdata = container_of(work, struct rmi_hid_data,
						attn_report_work);
	struct rmi_transport_device * xport = hdata->xport;
	u8 data[RMI_HID_INPUT_REPORT_LEN];
	struct rmi_hid_report * rmi_report = (struct rmi_hid_report *)data;
	struct rmi_hid_report * queue_report;
	struct rmi_driver_data * drv_data;
	unsigned long lock_flags;

	spin_lock_irqsave(&hdata->input_queue_lock, lock_flags);
	while (hdata->input_queue_head != hdata->input_queue_tail) {
		memset(rmi_report, 0, RMI_HID_INPUT_REPORT_LEN);

		queue_report = (struct rmi_hid_report *)(((u8*)hdata->input_queue)
				+ RMI_HID_INPUT_REPORT_LEN * hdata->input_queue_head);
		memcpy(rmi_report, queue_report, RMI_HID_INPUT_REPORT_LEN);
		hdata->input_queue_head = (hdata->input_queue_head + 1)
					% RMI_HID_INPUT_REPORT_QUEUE_LEN;
		spin_unlock_irqrestore(&hdata->input_queue_lock, lock_flags);

		dev_dbg(&xport->rmi_dev->dev, "attn = %*ph\n", RMI_HID_INPUT_REPORT_LEN,
			rmi_report);

		if (test_bit(RMI_HID_STARTED, &hdata->flags)) {
			/* process it! */
			drv_data = dev_get_drvdata(&xport->rmi_dev->dev);
			*(drv_data->irq_status) = rmi_report->attn_report.interruptSources;
			xport->attn_data = rmi_report->attn_report.data;
			xport->attn_size = RMI_HID_INPUT_REPORT_LEN -
						1 /* report id */ -
						1 /* interrupt sources */;
			xport->rmi_dev->driver->process(xport->rmi_dev);
		}
		spin_lock_irqsave(&hdata->input_queue_lock, lock_flags);
	}
	spin_unlock_irqrestore(&hdata->input_queue_lock, lock_flags);
}

static int rmi_hid_raw_event(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size)
{
	struct rmi_transport_device *xport = hid_get_drvdata(hdev);
	struct rmi_hid_data * hdata = xport->data;
	struct rmi_hid_report * queue_report;
	struct rmi_hid_report * rmi_report = (struct rmi_hid_report *)data;
	int sched_work = 0;
	unsigned long lock_flags;

	if (rmi_report->hidReportID == RMI_READ_DATA_REPORT_ID) {
		if (test_bit(RMI_HID_READ_REQUEST_PENDING, &hdata->flags)) {
			memcpy(&hdata->readReport, rmi_report,
				size < RMI_HID_INPUT_REPORT_LEN ? size
				: RMI_HID_INPUT_REPORT_LEN);
			set_bit(RMI_HID_READ_DATA_PENDING, &hdata->flags);
			wake_up(&hdata->wait);
		} else
			dev_info(&hdev->dev, "NO READ REQUEST PENDING\n");

	} else if (rmi_report->hidReportID == RMI_ATTN_REPORT_ID
			&& test_bit(RMI_HID_STARTED, &hdata->flags))
	{
		spin_lock_irqsave(&hdata->input_queue_lock, lock_flags);
		if (hdata->input_queue_head == hdata->input_queue_tail)
			sched_work = 1;

		queue_report = (struct rmi_hid_report *)(((u8*)hdata->input_queue)
				+ RMI_HID_INPUT_REPORT_LEN * hdata->input_queue_tail);
		memcpy(queue_report, data, size < RMI_HID_INPUT_REPORT_LEN ? size
			: RMI_HID_INPUT_REPORT_LEN);
		hdata->input_queue_tail = (hdata->input_queue_tail + 1)
			% RMI_HID_INPUT_REPORT_QUEUE_LEN;
		spin_unlock_irqrestore(&hdata->input_queue_lock, lock_flags);

		if (sched_work)
			schedule_work(&hdata->attn_report_work);
	}

	return 0;
}

static void rmi_hid_post_reset(struct rmi_transport_device *xport)
{
        struct hid_device *hdev = to_hid_device(xport->dev);
	rmi_hid_set_mode(hdev, RMI_HID_MODE_ATTN_REPORTS);
}

static int rmi_hid_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct rmi_transport_device *xport = NULL;
	struct rmi_hid_data *data = NULL;
	unsigned int connect_mask = HID_CONNECT_DEFAULT;
	int ret;

	dev_dbg(&hdev->dev, "%s\n", __func__);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		goto err_free;
	}

	ret = hid_hw_start(hdev, connect_mask);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		goto err_free;
	}

	xport = devm_kzalloc(&hdev->dev, sizeof(struct rmi_transport_device),
				GFP_KERNEL);
	if (!xport) {
		ret = -ENOMEM;
		goto hid_stop;
	}

	data = devm_kzalloc(&hdev->dev, sizeof(struct rmi_hid_data),
				GFP_KERNEL);
	if (!data) {
		ret =-ENOMEM;
		goto hid_stop;
	}

	data->xport = xport;

	xport->data = data;
	xport->dev = &hdev->dev;

	xport->write_block = rmi_hid_write_block;
	xport->read_block = rmi_hid_read_block;
	xport->info.proto_type = RMI_PROTOCOL_HID;
	xport->info.proto = transport_proto_name;
	xport->post_reset = rmi_hid_post_reset;
	hid_set_drvdata(hdev, xport);

	data->input_queue = devm_kzalloc(&hdev->dev, RMI_HID_INPUT_REPORT_LEN
				* RMI_HID_INPUT_REPORT_QUEUE_LEN, GFP_KERNEL);
	if (!data->input_queue) {
		ret = -ENOMEM;
		goto hid_stop;
	}

	tm2735_platformdata.pm_data = hdev;
	xport->dev->platform_data = &tm2735_platformdata;

#ifdef TOUCHPAD_WAKE_SYSTEM
	if (tm2735_platformdata.f11_sensor_data[0].sensor_type == rmi_f11_sensor_touchpad) {
		device_init_wakeup(hdev->dev.parent, 1);
	}
#endif

	spin_lock_init(&data->input_queue_lock);
	data->input_queue_head = 0;
	data->input_queue_tail = 0;
	INIT_WORK(&data->attn_report_work, rmi_hid_attn_report_work);

	init_waitqueue_head(&data->wait);

	mutex_init(&data->page_mutex);

	dev_dbg(&hdev->dev, "Opening low level driver\n");
	hdev->ll_driver->open(hdev);

	/* Allow incoming hid reports */
	hid_device_io_start(hdev);

	ret = rmi_hid_set_mode(hdev, RMI_HID_MODE_ATTN_REPORTS);
	if (ret < 0) {
		dev_err(&hdev->dev, "failed to set rmi mode\n");
		goto rmi_read_failed;
	}

	ret = rmi_set_page(xport, 0);
	if (ret < 0) {
		dev_err(&hdev->dev, "failed to set page select to 0.\n");
		goto rmi_read_failed;
	}

	ret = rmi_register_transport_device(xport);
	if (ret) {
		dev_err(&hdev->dev, "failed to register transport device at %s\n",
			hdev->phys);
		goto rmi_read_failed;
	}

	if (!xport->probe_succeeded) {
		dev_err(&hdev->dev, "Probe failed in rmi_driver\n");
		ret = -ENODEV;
		goto rmi_driver_probe_failed;
	}

	set_bit(RMI_HID_STARTED, &data->flags);

	if (IS_ENABLED(CONFIG_RMI4_DEBUG))
		ret = setup_debugfs(xport->rmi_dev, data);

	dev_info(&hdev->dev, "registered rmi hid driver at %s\n", hdev->phys);

	return 0;

rmi_driver_probe_failed:
	rmi_unregister_transport_device(xport);

rmi_read_failed:
	hdev->ll_driver->close(hdev);

hid_stop:
	hid_hw_stop(hdev);

err_free:
	return ret;
}

static void rmi_hid_remove(struct hid_device *hdev)
{
	struct rmi_transport_device *xport = hid_get_drvdata(hdev);
	struct rmi_hid_data * hdata = xport->data;

	clear_bit(RMI_HID_STARTED, &hdata->flags);
	cancel_work_sync(&hdata->attn_report_work);

	if (IS_ENABLED(CONFIG_RMI4_DEBUG))
		teardown_debugfs(xport->data);
	
	rmi_unregister_transport_device(xport);

	hdev->ll_driver->close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id rmi_hid_id[] = {
	{ HID_DEVICE(BUS_I2C, HID_GROUP_ANY, 0x06cb, HID_ANY_ID),
		.driver_data = 0 },
	{ HID_USB_DEVICE(0x06cb, HID_ANY_ID),
		.driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(hid, rmi_hid_id);

static struct hid_driver rmi_hid_driver = {
	.name = "rmi_hid",
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi_hid",
	},
	.id_table	= rmi_hid_id,
	.probe		= rmi_hid_probe,
	.remove		= rmi_hid_remove,
	.raw_event	= rmi_hid_raw_event,
};

static int __init rmi_hid_init(void)
{
	int ret;

	ret = hid_register_driver(&rmi_hid_driver);
	if (ret)
		pr_err("can't register rmi_hid driver (%d)\n", ret);
	else
		pr_info("Successfully registered rmi_hid driver\n");

	return ret;
}

static void __exit rmi_hid_exit(void)
{
	hid_unregister_driver(&rmi_hid_driver);
}

late_initcall(rmi_hid_init);
module_exit(rmi_hid_exit);

MODULE_AUTHOR("Andrew Duggan <aduggan@synaptics.com>");
MODULE_DESCRIPTION("RMI HID driver");
MODULE_LICENSE("GPL");
