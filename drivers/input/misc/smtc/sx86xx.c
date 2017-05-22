/*! \file sx86xx.c
 * \brief  Helper functions for Interrupt handling on SXxx products
 *
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

//#define DEBUG

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/input/smtc/misc/sx86xx.h> /* main struct, interrupt,init,pointers */


#ifdef USE_THREADED_IRQ
static void sx86XX_process_interrupt(psx86XX_t this,u8 nirqlow)
{
	int status = 0;
	int counter = 0;
	if (!this) {
		printk(KERN_ERR "sx86XX_worker_func, NULL sx86XX_t\n");
		return;
	}
	/* since we are not in an interrupt don't need to disable irq. */
	status = this->refreshStatus(this);
	counter = -1;
	dev_dbg(this->pdev, "Worker - Refresh Status %d\n",status);
	while((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
		dev_dbg(this->pdev, "Looping Counter %d\n",counter);
		if (((status>>counter) & 0x01) && (this->statusFunc[counter])) {
			dev_dbg(this->pdev, "Function Pointer Found. Calling\n");
			this->statusFunc[counter](this);
		}
	}
	if (unlikely(this->useIrqTimer && nirqlow)) {
		/* In case we need to send a timer for example on a touchscreen
		 * checking penup, perform this here
		 */
		cancel_delayed_work(&this->dworker);
		schedule_delayed_work(&this->dworker,msecs_to_jiffies(this->irqTimeout));
		dev_info(this->pdev,"Schedule Irq timer");
	} 
}


static void sx86XX_worker_func(struct work_struct *work)
{
	psx86XX_t this = 0;
	if (work) {
		this = container_of(work,sx86XX_t,dworker.work);
		if (!this) {
			printk(KERN_ERR "sx86XX_worker_func, NULL sx86XX_t\n");
			return;
		}
		if ((!this->get_nirq_low) || (!this->get_nirq_low())) {
			/* only run if nirq is high */
			sx86XX_process_interrupt(this,0);
		}
	} else {
		printk(KERN_ERR "sx86XX_worker_func, NULL work_struct\n");
	}
}

static irqreturn_t sx86XX_interrupt_thread(int irq, void *data)
{
	psx86XX_t this = 0;
	this = data;
	mutex_lock(&this->mutex);
	dev_dbg(this->pdev, "sx86XX_irq\n");
	if ((!this->get_nirq_low) || this->get_nirq_low()) {
		sx86XX_process_interrupt(this,1);
	}
	else
		dev_err(this->pdev, "sx86XX_irq - nirq read high\n");
	mutex_unlock(&this->mutex);
	return IRQ_HANDLED;
}
#else
static void sx86XX_schedule_work(psx86XX_t this, unsigned long delay)
{
	unsigned long flags;
	if (this) {
		dev_dbg(this->pdev, "sx86XX_schedule_work()\n");
		spin_lock_irqsave(&this->lock,flags);
		/* Stop any pending penup queues */
		cancel_delayed_work(&this->dworker);
		schedule_delayed_work(&this->dworker,delay);
		spin_unlock_irqrestore(&this->lock,flags);
	}
	else
		printk(KERN_ERR "sx86XX_schedule_work, NULL psx86XX_t\n");
} 

static irqreturn_t sx86XX_irq(int irq, void *pvoid)
{
	psx86XX_t this = 0;
	if (pvoid) {
		this = (psx86XX_t)pvoid;
		dev_dbg(this->pdev, "sx86XX_irq\n");
		if ((!this->get_nirq_low) || this->get_nirq_low()) {
			dev_dbg(this->pdev, "sx86XX_irq - Schedule Work\n");
			sx86XX_schedule_work(this,0);
		}
		else
			dev_err(this->pdev, "sx86XX_irq - nirq read high\n");
	}
	else
		printk(KERN_ERR "sx86XX_irq, NULL pvoid\n");
	return IRQ_HANDLED;
}

static void sx86XX_worker_func(struct work_struct *work)
{
	psx86XX_t this = 0;
	int status = 0;
	int counter = 0;
	u8 nirqLow = 0;
	if (work) {
		this = container_of(work,sx86XX_t,dworker.work);

		if (!this) {
			printk(KERN_ERR "sx86XX_worker_func, NULL sx86XX_t\n");
			return;
		}
		if (unlikely(this->useIrqTimer)) {
			if ((!this->get_nirq_low) || this->get_nirq_low()) {
				nirqLow = 1;
			}
		}
		/* since we are not in an interrupt don't need to disable irq. */
		status = this->refreshStatus(this);
		counter = -1;
		dev_dbg(this->pdev, "Worker - Refresh Status %d\n",status);
		while((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
			dev_dbg(this->pdev, "Looping Counter %d\n",counter);
			if (((status>>counter) & 0x01) && (this->statusFunc[counter])) {
				dev_dbg(this->pdev, "Function Pointer Found. Calling\n");
				this->statusFunc[counter](this);
			}
		}
		if (unlikely(this->useIrqTimer && nirqLow))
		{ /* Early models and if RATE=0 for newer models require a penup timer */
			/* Queue up the function again for checking on penup */
			sx86XX_schedule_work(this,msecs_to_jiffies(this->irqTimeout));
		}
	} else {
		printk(KERN_ERR "sx86XX_worker_func, NULL work_struct\n");
	}
}
#endif

static void sx86XX_resume_worker_func(struct work_struct *work)
{
	psx86XX_t this = 0;
	if (work) {
		this = container_of(work,sx86XX_t,resume_dworker.work);

		if (!this) {
			printk(KERN_ERR "sx86XX_resume_worker_func, NULL sx86XX_t\n");
			return;
		}

		this->init(this);
		enable_irq(this->irq);

	} else {
		printk(KERN_ERR "sx86XX_resume_worker_func, NULL work_struct\n");
	}
}


void sx86XX_suspend(psx86XX_t this)
{
	cancel_delayed_work_sync(&this->resume_dworker);

	if (this)
		disable_irq(this->irq);
}
void sx86XX_resume(psx86XX_t this)
{
	if (this) {
#ifdef USE_THREADED_IRQ
		mutex_lock(&this->mutex);
		/* Just in case need to reset any uncaught interrupts */
		sx86XX_process_interrupt(this,0);
		mutex_unlock(&this->mutex);
#else
		sx86XX_schedule_work(this,0);
#endif
		/*if (this->init)
			this->init(this);
		enable_irq(this->irq);*/
		schedule_delayed_work(&this->resume_dworker, msecs_to_jiffies(1000));
	}
}

#ifdef CONFIG_HAS_WAKELOCK
/*TODO: Should actually call the device specific suspend/resume
 * As long as the kernel suspend/resume is setup, the device
 * specific ones will be called anyways
 */
extern suspend_state_t get_suspend_state(void);
void sx86XX_early_suspend(struct early_suspend *h)
{
	psx86XX_t this = 0;
	dev_dbg(this->pdev, "inside sx86XX_early_suspend()\n");
	this = container_of(h, sx86XX_t, early_suspend);
	sx86XX_suspend(this);
	dev_dbg(this->pdev, "exit sx86XX_early_suspend()\n");
}

void sx86XX_late_resume(struct early_suspend *h)
{
	psx86XX_t this = 0;
	dev_dbg(this->pdev, "inside sx86XX_late_resume()\n");
	this = container_of(h, sx86XX_t, early_suspend);
	sx86XX_resume(this);
	dev_dbg(this->pdev, "exit sx86XX_late_resume()\n");
}
#endif

int sx86XX_init(psx86XX_t this)
{
	int err = 0;
	if (this && this->pDevice)
	{

#ifdef USE_THREADED_IRQ

		/* initialize worker function */
		INIT_DELAYED_WORK(&this->dworker, sx86XX_worker_func);


		/* initialize mutex */
		mutex_init(&this->mutex);
		/* initailize interrupt reporting */
		this->irq_disabled = 0;
		err = request_threaded_irq(this->irq, NULL, sx86XX_interrupt_thread,
				IRQF_TRIGGER_FALLING, this->pdev->driver->name,
				this);
#else
		/* initialize spin lock */
		spin_lock_init(&this->lock);

		/* initialize worker function */
		INIT_DELAYED_WORK(&this->dworker, sx86XX_worker_func);

		/* initailize interrupt reporting */
		this->irq_disabled = 0;
		err = request_irq(this->irq, sx86XX_irq, IRQF_TRIGGER_FALLING,
				this->pdev->driver->name, this);
#endif
		if (err) {
			dev_err(this->pdev, "irq %d busy?:%d\n", this->irq, err);
			return err;
		}
#ifdef USE_THREADED_IRQ
		dev_info(this->pdev, "registered with threaded irq (%d)\n", this->irq);
#else
		dev_info(this->pdev, "registered with irq (%d)\n", this->irq);
#endif
#ifdef CONFIG_HAS_WAKELOCK	
		this->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		this->early_suspend.suspend = sx86XX_early_suspend;
		this->early_suspend.resume = sx86XX_late_resume;
		register_early_suspend(&this->early_suspend);
		if (has_wake_lock(WAKE_LOCK_SUSPEND) == 0 && 
				get_suspend_state() == PM_SUSPEND_ON)
			sx86XX_early_suspend(&this->early_suspend);
#endif //CONFIG_HAS_WAKELOCK

		INIT_DELAYED_WORK(&this->resume_dworker, sx86XX_resume_worker_func);

		/* call init function pointer (this should initialize all registers */
		if (this->init)
			return this->init(this);
		dev_err(this->pdev,"No init function!!!!\n");
	}
	return -ENOMEM;
}

int sx86XX_remove(psx86XX_t this)
{
	if (this) {
		cancel_delayed_work_sync(&this->dworker); /* Cancel the Worker Func */
		/*destroy_workqueue(this->workq); */
#ifdef CONFIG_HAS_WAKELOCK
		unregister_early_suspend(&this->early_suspend);
#endif
		free_irq(this->irq, this);
		kfree(this);
		return 0;
	}
	return -ENOMEM;
}

