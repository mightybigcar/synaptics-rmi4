/*
 * Copyright (c) 2013 Synaptics Incorporated
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


#ifndef _RMI_F05_H
#define _RMI_F05_H

#define F05_STATE_IDLE		0
#define F05_STATE_READY		1
#define F05_STATE_RUN		2
#define F05_STATE_PENDING	3
#define F05_STATE_ERROR		4

#define F05_REPORT_STOP		0
#define F05_REPORT_START	1
#define F05_REPORT_SET_TYPE	2
#define F05_REPORT_FORCE_ZERO	3
// PAUSE should eventually replace STOP.
#define F05_REPORT_PAUSE	4

#endif
