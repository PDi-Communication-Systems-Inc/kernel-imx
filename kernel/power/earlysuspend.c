/* kernel/power/earlysuspend.c
 *
 * Copyright (C) 2005-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/earlysuspend.h>
#include <linux/gpio.h>		// JTS - needed for gpio_set_value()
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/syscalls.h> /* sys_sync */
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/delay.h> /* msleep */

#include <linux/usbdevice_fs.h> /* PDi - USBDEVFS_RESET */

#include "power.h"

// {JTS 7/10/2013 - from board-mx6q_sabreauto.c
#define AR6MX_TTL_DO0	          IMX_GPIO_NR(2, 6)
#define AR6MX_TTL_DO1             IMX_GPIO_NR(2, 7)
//Doge 8/25/2015 add LCD backlight control
#define AR6MX_BL0_EN		  IMX_GPIO_NR(1, 13)
// JTS}

//Start : PDi - USB power maps - from board-mx6q_sabreauto.c
#define SABRESD_USB_V1_PWR	        IMX_GPIO_NR(5, 13)
#define SABRESD_USB_V2_PWR          IMX_GPIO_NR(5, 14)
//End 

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
	DEBUG_VERBOSE = 1U << 3,
};
static int debug_mask = DEBUG_USER_STATE;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static DEFINE_MUTEX(early_suspend_lock);
static LIST_HEAD(early_suspend_handlers);
static void early_suspend(struct work_struct *work);
static void late_resume(struct work_struct *work);
static DECLARE_WORK(early_suspend_work, early_suspend);
static DECLARE_WORK(late_resume_work, late_resume);
static DEFINE_SPINLOCK(state_lock);
enum {
	SUSPEND_REQUESTED = 0x1,
	SUSPENDED = 0x2,
	SUSPEND_REQUESTED_AND_SUSPENDED = SUSPEND_REQUESTED | SUSPENDED,
};
static int state;
	
int usbHUBReset()
{
	int fd;
    int rc;
	
	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open("/dev/bus/usb/002/002", O_WRONLY, 0);
	if (fd < 0) {
        pr_err("Error opening /dev/bus/usb/002/002 file");
        return 1;
     }

	pr_info("Resetting USB device /dev/bus/usb/002/002\n");
    rc = sys_ioctl(fd, USBDEVFS_RESET, 0);
    if (rc < 0) {
        pr_err("Error in ioctl");
        return 1;
    }
    pr_info("Reset successful\n");
    sys_close(fd);	 
	
	set_fs(old_fs);
	return 0;	
}

void register_early_suspend(struct early_suspend *handler)
{
	struct list_head *pos;

	mutex_lock(&early_suspend_lock);
	list_for_each(pos, &early_suspend_handlers) {
		struct early_suspend *e;
		e = list_entry(pos, struct early_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	if ((state & SUSPENDED) && handler->suspend)
		handler->suspend(handler);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(register_early_suspend);

void unregister_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(unregister_early_suspend);

static void early_suspend(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;
	
	gpio_set_value(AR6MX_TTL_DO0, 0);	// JTS - 7/9/2013 drop DO0 low for sleep
	mdelay(1);
	gpio_set_value(AR6MX_TTL_DO1, 0);   // JTS - 7/9/2013 drop DO1 low for sleep
	mdelay(1);
	gpio_set_value(AR6MX_BL0_EN, 0);	// Doge - 8/25/2015 bring B/L Off

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED)
		state |= SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("early_suspend: abort, state %d\n", state);
		mutex_unlock(&early_suspend_lock);
		goto abort;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: call handlers\n");
	list_for_each_entry(pos, &early_suspend_handlers, link) {
		if (pos->suspend != NULL) {
			if (debug_mask & DEBUG_VERBOSE)
				pr_info("early_suspend: calling %pf\n", pos->suspend);
			pos->suspend(pos);
		}
	}
	mutex_unlock(&early_suspend_lock);

	//printk(KERN_DEBUG "*** PDi Powering down usb ports *** ");		       			
    //    gpio_set_value(SABRESD_USB_V1_PWR, 0); 
	//mdelay(1);
    //    gpio_set_value(SABRESD_USB_V2_PWR, 0);
	//msleep(3000);

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: sync\n");

	sys_sync();
abort:
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED)
		wake_unlock(&main_wake_lock);
	spin_unlock_irqrestore(&state_lock, irqflags);
}

static void late_resume(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPENDED)
		state &= ~SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("late_resume: abort, state %d\n", state);
		goto abort;
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: call handlers\n");
	list_for_each_entry_reverse(pos, &early_suspend_handlers, link) {
		if (pos->resume != NULL) {
			if (debug_mask & DEBUG_VERBOSE)
				pr_info("late_resume: calling %pf\n", pos->resume);

			pos->resume(pos);
		}
	}

	//printk(KERN_DEBUG "*** PDi Powering up usb ports *** ");
    //    gpio_set_value(SABRESD_USB_V1_PWR, 1); 
	//mdelay(1);
    //    gpio_set_value(SABRESD_USB_V2_PWR, 1);
	//mdelay(1);
		
	gpio_set_value(AR6MX_TTL_DO0, 1);	// JTS - 7/9/2013 bring DO0 high for wake 
	mdelay(1);
	gpio_set_value(AR6MX_TTL_DO1, 1);	// JTS - 7/9/2013 bring DO1 high for wake
	mdelay(1);
	gpio_set_value(AR6MX_BL0_EN, 1);	// Doge - 8/25/2015 bring B/L On
 
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: done\n");
abort:
	mutex_unlock(&early_suspend_lock);
}

void request_suspend_state(suspend_state_t new_state)
{
	unsigned long irqflags;
	int old_sleep;

	spin_lock_irqsave(&state_lock, irqflags);
	old_sleep = state & SUSPEND_REQUESTED;
	if (debug_mask & DEBUG_USER_STATE) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("request_suspend_state: %s (%d->%d) at %lld "
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			new_state != PM_SUSPEND_ON ? "sleep" : "wakeup",
			requested_suspend_state, new_state,
			ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	}
	if (!old_sleep && new_state != PM_SUSPEND_ON) {
	
		state |= SUSPEND_REQUESTED;
 
		queue_work(suspend_work_queue, &early_suspend_work);

	} else if (old_sleep && new_state == PM_SUSPEND_ON) {

		state &= ~SUSPEND_REQUESTED;

	/*	printk(KERN_DEBUG "*** PDi Powering down usb ports *** ");
        	gpio_set_value(SABRESD_USB_V1_PWR, 1); 
        	gpio_set_value(SABRESD_USB_V2_PWR, 1); */
 
		wake_lock(&main_wake_lock);
		queue_work(suspend_work_queue, &late_resume_work);
	}
	//PDi - Start - Reset USB Hub
	//usbHUBReset();** Does not Fix the touchscreen freeze issue at NIH **
	//PDi - End - Reset USB Hub
	requested_suspend_state = new_state;
	spin_unlock_irqrestore(&state_lock, irqflags);
}

suspend_state_t get_suspend_state(void)
{
	return requested_suspend_state;
}
