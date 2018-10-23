/*
 * Basic Linux Kernel module RF433 using GPIO interrupts.
 *
 * Copyright (c) 2017 Alexey Maslyukov
 * All rights reserved
 *
 * Special thanks for Suat Ozgur
 * RCSwitch - Arduino libary for remote control outlet switches
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
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/major.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <mach/gpio_id.h>

#ifdef	CONFIG_MACH_RDA8810
#define GPIO_FOR_RX_SIGNAL	GPIO_A4
#else
#define GPIO_FOR_RX_SIGNAL	4
#endif
#define DEVICE_NAME 		"rx433"
#define BUFFER_SZ			256

#define SPRTN_LIM 4300 // Separation limit
#define RCV_TLRNC 60   // Receive tolerance
#define MAX_CHNG  67

/* Last Interrupt timestamp */
static struct timespec lastTime;
struct rcvdVal {
	unsigned long val;
	unsigned int bitLen;
	unsigned int delay;
	unsigned int proto;
};	

static unsigned int timings[MAX_CHNG];
struct high_N_low {
	unsigned char high;
	unsigned char low;
};

struct rxProtocol {
	int plsLen;
	struct high_N_low syncF;
	struct high_N_low zero;
	struct high_N_low one;
	bool inversion;
};

static struct rxProtocol proto[] = {
		{ 350, {  1, 31 }, {  1,  3 }, {  3,  1 }, false },    // protocol 1
		{ 650, {  1, 10 }, {  1,  2 }, {  2,  1 }, false },    // protocol 2
		{ 100, { 30, 71 }, {  4, 11 }, {  9,  6 }, false },    // protocol 3
		{ 380, {  1,  6 }, {  1,  3 }, {  3,  1 }, false },    // protocol 4
		{ 500, {  6, 14 }, {  1,  2 }, {  2,  1 }, false },    // protocol 5
		{ 450, { 23,  1 }, {  1,  2 }, {  2,  1 }, true }      // protocol 6 (HT6P20B)
};
enum {
	numProto = sizeof(proto) / sizeof(proto[0])
};


/* Define GPIOs for RX signal */
static struct gpio signals[] = {
		{ GPIO_FOR_RX_SIGNAL, GPIOF_IN, "RX433 Signal" },	// Rx signal
};
module_param_named( gpio, signals[0].gpio, int, GPIO_FOR_RX_SIGNAL );

// RX433 device struct
struct rx433 {
	unsigned int tail, head;
	struct rcvdVal vals[BUFFER_SZ];
	bool overflow;
	int irq;
	wait_queue_head_t wait;
};

static struct rx433 RX433;

/*********************************************************************
 *             Interface with userspace (file operations)            *
 *********************************************************************/

static int rx433_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int rx433_release(struct inode *inode, struct file *file)
{
	//printk(KERN_INFO "%s\n", __func__);

	return 0;
}

static ssize_t rx433_read(struct file *file, char __user *buffer,
		size_t count, loff_t *ppos)
{
	char uninitialized_var(c);
	ssize_t read = 0;
	int error;
	char buf[256];

	//printk(KERN_INFO "%s (%d/%lu)\n", __func__, count, *ppos);

	for (;;) {
		if (RX433.head == RX433.tail &&
				(file->f_flags & O_NONBLOCK))
			return -EAGAIN;

		if (count == 0)
			break;

		if (RX433.head != RX433.tail) {
			sprintf(buf,"%lX\n",RX433.vals[RX433.tail].val);
			read = strlen(buf);
			if( copy_to_user(buffer,buf,read+1) != 0 ) {
				printk(KERN_ERR "RX433 - Error writing to char device");
				return -EFAULT;
			}
			RX433.tail = (RX433.tail + 1) % (BUFFER_SZ-1);
		}

		if (read)
			break;

		if (!(file->f_flags & O_NONBLOCK)) {
			error = wait_event_interruptible(RX433.wait,
					RX433.head != RX433.tail);
			if (error)
				return error;
		}
	}

	return read;
}

static ssize_t rx433_write(struct file *file, const char __user *buf,
		size_t count, loff_t *pos)
{
	return -EINVAL;
}

static unsigned int rx433_poll(struct file *file, poll_table *wait)
{
	unsigned int mask;

	printk(KERN_INFO "%s\n", __func__);

	poll_wait(file, &RX433.wait, wait);

	mask = POLLOUT | POLLWRNORM;
	if (RX433.head != RX433.tail)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static struct file_operations rx433_fops = {
		.owner		= THIS_MODULE,
		.open		= rx433_open,
		.release	= rx433_release,
		.read		= rx433_read,
		.write		= rx433_write,
		.poll		= rx433_poll,
		.llseek		= noop_llseek,
};

static struct miscdevice rx433_dev = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = DEVICE_NAME,
		.fops = &rx433_fops,
};

/*********************************************************************
 *                   Interface with serio port                       *
 *********************************************************************/

/*
 * The interrupt service routine called on every pin status change
 */
static bool findProto(const int p, unsigned int cngcnt) {
	unsigned int i;

	struct rxProtocol *pro = &proto[p-1];

	unsigned long code = 0;
	//Assuming the longer pulse length is the pulse captured in timings[0]
	const unsigned int syncLen =  ((pro->syncF.low) > (pro->syncF.high)) ? (pro->syncF.low) : (pro->syncF.high);
	const unsigned int delay = timings[0] / syncLen;
	const unsigned int delayT = delay * RCV_TLRNC / 100;

	const unsigned int invI = (pro->inversion) ? (2) : (1);

	for (i = invI; i < cngcnt - 1; i += 2) {
		code <<= 1;
		if (abs(timings[i] - (delay * pro->zero.high)) < delayT &&
				abs(timings[i + 1] - (delay * pro->zero.low)) < delayT) {
			// zero
		} else if (abs(timings[i] - (delay * pro->one.high)) < delayT &&
				abs(timings[i + 1] - (delay * pro->one.low)) < delayT) {
			// one
			code |= 1;
		} else {
			// Failed
			return false;
		}
	}

	if (cngcnt > 7) {    // ignore very short transmissions: no device sends them, so this must be noise
		RX433.vals[RX433.head].val = code;
		RX433.vals[RX433.head].bitLen = (cngcnt - 1) / 2;
		RX433.vals[RX433.head].delay = delay;
		RX433.vals[RX433.head].proto = p;
		RX433.head = ( RX433.head + 1 )  & (BUFFER_SZ-1);
		if (RX433.head == RX433.tail) {
			// overflow
			RX433.tail = ( RX433.tail + 1 ) & (BUFFER_SZ-1);
			if ( RX433.overflow == false ) {
				printk(KERN_ERR "RX433 - Buffer Overflow - IRQ will be missed");
				RX433.overflow = true;
			}
		} else {
			RX433.overflow = false;
		}

		return true;
	}

	return false;
}

static irqreturn_t rx433_isr(int irq, void *data) {

	static unsigned int  cngcnt = 0;
	static unsigned int  rptcnt = 0;

	struct timespec currTime;
	struct timespec delta;
	unsigned long dura;
	unsigned int i;

	getnstimeofday(&currTime);
	delta = timespec_sub(currTime, lastTime);
	dura = ((long long)delta.tv_sec * 1000000)+(delta.tv_nsec/1000);

	if (dura > SPRTN_LIM) {
		if (abs(dura-timings[0]) < 200) {
			rptcnt++;
			if (rptcnt == 2) {
				for(i = 1; i <= numProto; i++) {
					if (findProto(i, cngcnt)) {
						wake_up_interruptible(&RX433.wait);
						break;
					}
				}
				rptcnt = 0;
			}
		}
		cngcnt = 0;
	}

	// detect overflow
	if (cngcnt >= MAX_CHNG) {
		cngcnt = 0;
		rptcnt = 0;
	}

	timings[cngcnt++] = dura;

	lastTime = currTime;  

	return IRQ_HANDLED;
}

/*
 * Module init function
 */
static int __init rx433_init(void)
{
	int ret = 0;
	printk(KERN_INFO "RX433 - initialization\n");

	init_waitqueue_head(&RX433.wait);
	RX433.head = RX433.tail = 0;
	RX433.overflow = false;

	getnstimeofday(&lastTime);

	ret = gpio_request_array(signals, ARRAY_SIZE(signals));
	if (ret) {
		printk(KERN_ERR "RX433 - Unable to request GPIOs for RX Signals: %d\n", ret);
		goto fail2;
	}

	RX433.irq = gpio_to_irq(signals[0].gpio);
	if(RX433.irq < 0) {
		ret = RX433.irq;
		printk(KERN_ERR "RX433 - Unable to request IRQ: %d\n", ret);
		goto fail2;
	}
	printk(KERN_INFO "RX433 - Successfully requested RX IRQ # %d on GPIO %d\n", RX433.irq, signals[0].gpio);
	ret = request_irq(RX433.irq, rx433_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_DISABLED, "rx433#rx", NULL);
	if(ret) {
		printk(KERN_ERR "RX433 - Unable to request IRQ: %d\n", ret);
		goto fail3;
	}

	misc_register(&rx433_dev);

	return 0;

fail3:
	free_irq(RX433.irq, NULL);

fail2:
	gpio_free_array(signals, ARRAY_SIZE(signals));

	return ret;	
}

/**
 * Module exit function
 */
static void __exit rx433_exit(void)
{
	printk(KERN_INFO "RX433 - exit/unload\n");

	wake_up_interruptible(&RX433.wait);

	misc_deregister(&rx433_dev);

	free_irq(RX433.irq, NULL);

	gpio_free_array(signals, ARRAY_SIZE(signals));

}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexey Maslyukov<m_a_o@mail.ru>");
MODULE_DESCRIPTION("Linux Kernel Module for rx433 shield");

module_init(rx433_init);
module_exit(rx433_exit);
