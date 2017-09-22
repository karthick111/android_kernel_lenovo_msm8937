/*! @file vfsSpiDrv.c
*******************************************************************************
**  SPI Driver Interface Functions
**
**  This file contains the SPI driver interface functions.
**
**  Copyright (C) 2011-2013 Validity Sensors, Inc.
**  This program is free software; you can redistribute it and/or
**  modify it under the terms of the GNU General Public License
**  as published by the Free Software Foundation; either version 2
**  of the License, or (at your option) any later version.
**
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU General Public License for more details.
**
**  You should have received a copy of the GNU General Public License
**  along with this program; if not, write to the Free Software
**  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
*/
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/compat.h>
#include <asm-generic/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <dt-bindings/msm/msm-bus-ids.h>
#include <linux/semaphore.h>
#include "vfsSpiDrv.h"

#define VALIDITY_PART_NAME "validity_fingerprint"
#define VFSSPI_WAKE_TIME   (5 * HZ)

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_mutex);
static struct class *vfsspi_device_class;
static int gpio_irq;
static int ignore_ioctl = 0;

#ifdef CONFIG_OF
static struct of_device_id validity_metallica_table[] = {
	{ .compatible = "validity,metallicatee",},
	{ },
};
#else
#define validity_metallica_table NULL
#endif


/*
 * vfsspi_devData - The spi driver private structure
 * @devt:Device ID
 * @vfs_spi_lock:The lock for the spi device
 * @spi:The spi device
 * @device_entry:Device entry list
 * @buffer_mutex:The lock for the transfer buffer
 * @is_opened:Indicates that driver is opened
 * @buffer:buffer for transmitting data
 * @null_buffer:buffer for transmitting zeros
 * @stream_buffer:buffer for transmitting data stream
 * @stream_buffer_size:streaming buffer size
 * @drdy_pin:DRDY GPIO pin number
 * @sleep_pin:Sleep GPIO pin number
 * @user_pid:User process ID, to which the kernel signal
 *	indicating DRDY event is to be sent
 * @signal_id:Signal ID which kernel uses to indicating
 *	user mode driver that DRDY is asserted
 * @current_spi_speed:Current baud rate of SPI master clock
 */
struct vfsspi_device_data {
	dev_t devt;
	struct cdev cdev;
	spinlock_t vfs_spi_lock;
	struct platform_device *spi;
	struct list_head device_entry;
	struct mutex buffer_mutex;
	unsigned int is_opened;
	unsigned char *buffer;
	unsigned char *null_buffer;
	unsigned char *stream_buffer;
	size_t stream_buffer_size;
	unsigned int drdy_pin;
	unsigned int sleep_pin;
#if DO_CHIP_SELECT
	unsigned int cs_pin;
#endif
	int user_pid;
	int signal_id;
	unsigned int current_spi_speed;
	unsigned int is_drdy_irq_enabled;
	unsigned int drdy_ntf_type;
	struct mutex kernel_lock;

	/* power */
	struct regulator	*vdd_reg;
	unsigned int		power_3p3v;
	unsigned int		power_1p8v;
	int					power_enabled;

	/* irq wake */
	int	irq_wake_enabled;

	/* wake lock to ensoure fingerprint handled */
	struct wake_lock	wake_lock;
	int					wake_lock_acquired;
	struct timer_list	wake_unlock_timer;

	/* work queue & worker */
	struct work_struct	irq_worker;

	/* checking irq */
	struct semaphore	irq_check_sem;
	int					irq_check_flag;
};

#ifdef VFSSPI_32BIT
/*
 * Used by IOCTL compat command:
 *		 VFSSPI_IOCTL_RW_SPI_MESSAGE
 *
 * @rx_buffer:pointer to retrieved data
 * @tx_buffer:pointer to transmitted data
 * @len:transmitted/retrieved data size
 */
struct vfsspi_compat_ioctl_transfer {
	compat_uptr_t rx_buffer;
	compat_uptr_t tx_buffer;
	unsigned int len;
};
#endif

static int vfsspi_sendDrdyEventFd(struct vfsspi_device_data *vfsSpiDev);
static int vfsspi_sendDrdyNotify(struct vfsspi_device_data *vfsSpiDev);
static int vfsspi_ioctl_disable_irq_wake(struct vfsspi_device_data *data);
static void vfsspi_hardReset(struct vfsspi_device_data *vfsspi_device);
static int vfsspi_remove(struct platform_device *spi);
static int vfsspi_disableIrq(struct vfsspi_device_data *vfsspi_device);

static void vfsspi_wake_unlock(struct vfsspi_device_data *data)
{
	pr_debug("%s: enter\n", __func__);

	if (data->wake_lock_acquired) {
		wake_unlock(&data->wake_lock);
		data->wake_lock_acquired = 0;
	}
}

static void vfsspi_wake_unlock_timer_handler(unsigned long ptr)
{
	struct vfsspi_device_data *data = (struct vfsspi_device_data*)ptr;

	pr_debug("%s: enter\n", __func__);
	vfsspi_wake_unlock(data);
}

static void vfsspi_wake_lock_delayed_unlock(struct vfsspi_device_data *data)
{
	pr_debug("%s: enter\n", __func__);

	if (!data->wake_lock_acquired) {
		wake_lock(&data->wake_lock);
		data->wake_lock_acquired = 1;
	}
	mod_timer(&data->wake_unlock_timer, jiffies + VFSSPI_WAKE_TIME);
}

static int vfsspi_ioctl_power_init(struct platform_device *spi, struct vfsspi_device_data *data)
{
	int err = 0;

	pr_debug("%s: enter\n", __func__);

	data->power_enabled = 0;
	data->power_3p3v = VFSSPI_PW3P3V_PIN;
	data->power_3p3v = of_get_named_gpio(spi->dev.of_node, "synaptics,gpio_3p3v", 0);
	if (data->power_3p3v == 0) {
		pr_err("%s: fail to get power_3p3v\n", __func__);
		return -1;
	}
	data->power_1p8v = VFSSPI_PW1P8V_PIN;
	data->power_1p8v = of_get_named_gpio(spi->dev.of_node, "synaptics,gpio_1p8v", 0);
	if (data->power_1p8v == 0) {
		pr_err("%s: fail to get power_1p8v\n", __func__);
		return -1;
	}

	err = gpio_direction_output(data->power_3p3v, 1);
	if (err < 0) {
		pr_err("%s: fail to set output for power_3p3v\n", __func__);
		return -2;
	}
	err = gpio_direction_output(data->power_1p8v, 1);
	if (err < 0) {
		pr_err("%s: fail to set output for power_1p8v\n", __func__);
		return -2;
	}

	return 0;
}

static int vfsspi_ioctl_power_on(struct vfsspi_device_data *data)
{
	pr_debug("%s: enter\n", __func__);

	if (data->power_enabled) {
		return 0;
	}

	gpio_set_value(data->power_3p3v, 1);
	gpio_set_value(data->power_1p8v, 1);
	data->power_enabled = 1;

	return 0;
}

static int vfsspi_ioctl_power_off(struct vfsspi_device_data *data)
{
	pr_debug("%s: enter\n", __func__);

	// DO NOT enable the irq wake, otherwise system can't suspend
	vfsspi_ioctl_disable_irq_wake(data);

	if (!data->power_enabled) {
		return 0;
	}


	gpio_set_value(data->power_1p8v, 0);
	gpio_set_value(data->power_3p3v, 0);
	data->power_enabled = 0;

	return 0;
}

static int vfsspi_ioctl_power_uninit(struct vfsspi_device_data *data)
{
	if (data->power_enabled) {
		vfsspi_ioctl_power_off(data);
	}

	return 0;
}

static int vfsspi_ioctl_enable_irq_wake(struct vfsspi_device_data *data)
{
	pr_debug("%s: enter\n", __func__);

	// make sure that power is enabled
	if (!data->power_enabled) {
		pr_err("%s: power is off, DO NOT enable irq wake.\n", __func__);
		vfsspi_ioctl_disable_irq_wake(data);
		return -1;
	}

	// make sure that interrupt is enabled
	if (data->is_drdy_irq_enabled != DRDY_IRQ_ENABLE) {
		pr_err("%s: drdy irq is disabled, DO NOT enable irq wake.\n", __func__);
		vfsspi_ioctl_disable_irq_wake(data);
		return -2;
	}

	if (data->irq_wake_enabled) {
		return 0;
	}

	if (enable_irq_wake(gpio_irq)) {
		pr_err("%s: fail to enable_irq_wake\n", __func__);
		return -3;
	}

	data->irq_wake_enabled = 1;

	return 0;
}

static int vfsspi_ioctl_disable_irq_wake(struct vfsspi_device_data *data)
{
	pr_debug("%s: enter\n", __func__);

	if (!data->irq_wake_enabled) {
		return 0;
	}

	if (disable_irq_wake(gpio_irq)) {
		pr_err("%s: fail to disable_irq_wake\n", __func__);
		return -1;
	}

	data->irq_wake_enabled = 0;

	return 0;
}

static int vfsspi_drv_suspend(struct platform_device* spi, pm_message_t msg)
{
	return 0;
}

static int vfsspi_drv_resume(struct platform_device* spi)
{
	struct vfsspi_device_data *vfsspi_device;

	pr_debug("%s: enter\n", __func__);

	vfsspi_device = platform_get_drvdata(spi);
	if (!vfsspi_device->power_enabled) {
		vfsspi_ioctl_power_on(vfsspi_device);
	}

	return 0;
}

static void vfsspi_drv_shutdown(struct platform_device* spi)
{
	struct vfsspi_device_data *vfsspi_device = NULL;
	pr_info("%s()\n", __func__);

	ignore_ioctl = 1;
	vfsspi_device = platform_get_drvdata(spi);
	if (vfsspi_device) {
		vfsspi_disableIrq(vfsspi_device);
		vfsspi_ioctl_power_off(vfsspi_device);
	}
}

static void vfsspi_irq_worker(struct work_struct *arg)
{
	struct vfsspi_device_data *vfsspi_device = container_of(arg, struct vfsspi_device_data, irq_worker);

	/* notify HAL service */
	vfsspi_wake_lock_delayed_unlock(vfsspi_device);
	vfsspi_sendDrdyNotify(vfsspi_device);

	/* update irq check flag */
	vfsspi_device->irq_check_flag = 1;
	up(&vfsspi_device->irq_check_sem);

	printk("%s: exit\n", __func__);
}

static int vfsspi_send_drdy_signal(struct vfsspi_device_data *vfsspi_device)
{
	struct task_struct *t;
	int ret = 0;

	pr_debug("%s: enter\n", __func__);

	if (vfsspi_device->user_pid != 0) {
		rcu_read_lock();
		/* find the task_struct associated with userpid */
		pr_debug("%s: Searching task with PID=%08x\n",
			__func__, vfsspi_device->user_pid);
		t = pid_task(find_pid_ns(vfsspi_device->user_pid, &init_pid_ns),
				 PIDTYPE_PID);
		if (t == NULL) {
			pr_debug("%s: No such pid\n", __func__);
			rcu_read_unlock();
			return -ENODEV;
		}
		rcu_read_unlock();
		/* notify DRDY signal to user process */
		ret = send_sig_info(vfsspi_device->signal_id,
					(struct siginfo *)1, t);
		if (ret < 0)
			pr_err("%s: Error sending signal\n", __func__);

	} else {
		pr_err("%s: pid not received yet\n", __func__);
	}

	return ret;
}

static int vfsspi_register_drdy_signal(struct vfsspi_device_data *vfsspi_device,
					   unsigned long arg)
{
	struct vfsspi_ioctl_register_signal usr_signal;
	if (copy_from_user(&usr_signal, (void __user *)arg, sizeof(usr_signal)) != 0) {
		pr_err("%s: Failed copy from user.\n", __func__);
		return -EFAULT;
	} else {
		vfsspi_device->user_pid = usr_signal.user_pid;
		vfsspi_device->signal_id = usr_signal.signal_id;
	}
	return 0;
}

static irqreturn_t vfsspi_irq(int irq, void *context)
{
	struct vfsspi_device_data *vfsspi_device = context;

	pr_debug("%s: enter\n", __func__);

	/* Linux kernel is designed so that when you disable
	an edge-triggered interrupt, and the edge happens while
	the interrupt is disabled, the system will re-play the
	interrupt at enable time.
	Therefore, we are checking DRDY GPIO pin state to make sure
	if the interrupt handler has been called actually by DRDY
	interrupt and it's not a previous interrupt re-play */
	if (gpio_get_value(vfsspi_device->drdy_pin) == DRDY_ACTIVE_STATUS) {
		schedule_work(&vfsspi_device->irq_worker);
	}

	return IRQ_HANDLED;
}

static int vfsspi_sendDrdyEventFd(struct vfsspi_device_data *vfsSpiDev)
{
	struct task_struct *t;
	struct file *efd_file = NULL;
	struct eventfd_ctx *efd_ctx = NULL;	int ret = 0;

	pr_debug("%s: enter\n", __func__);

	if (vfsSpiDev->user_pid != 0) {
		rcu_read_lock();
		/* find the task_struct associated with userpid */
		pr_debug("%s: Searching task with PID=%08x\n", __func__, vfsSpiDev->user_pid);
		t = pid_task(find_pid_ns(vfsSpiDev->user_pid, &init_pid_ns),
			PIDTYPE_PID);
		if (t == NULL) {
			pr_debug("%s: No such pid\n", __func__);
			rcu_read_unlock();
			return -ENODEV;
		}
		efd_file = fcheck_files(t->files, vfsSpiDev->signal_id);
		rcu_read_unlock();

		if (efd_file == NULL) {
			pr_debug("%s: No such efd_file\n", __func__);
			return -ENODEV;
		}

		efd_ctx = eventfd_ctx_fileget(efd_file);
		if (efd_ctx == NULL) {
			pr_debug("%s: eventfd_ctx_fileget is failed\n", __func__);
			return -ENODEV;
		}

		/* notify DRDY eventfd to user process */
		eventfd_signal(efd_ctx, 1);

		/* Release eventfd context */
		eventfd_ctx_put(efd_ctx);
	}

	return ret;
}

static int vfsspi_sendDrdyNotify(struct vfsspi_device_data *vfsSpiDev)
{
	int ret = 0;

	pr_debug("%s: enter\n", __func__);

	if (vfsSpiDev->drdy_ntf_type == VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD) {
		ret = vfsspi_sendDrdyEventFd(vfsSpiDev);
	} else {
		ret = vfsspi_send_drdy_signal(vfsSpiDev);
	}

	return ret;
}

static int vfsspi_enableIrq(struct vfsspi_device_data *vfsspi_device)
{
	pr_debug("%s: enter\n", __func__);

	if (vfsspi_device->is_drdy_irq_enabled == DRDY_IRQ_ENABLE) {
		pr_debug("%s: DRDY irq already enabled\n", __func__);
		return -EINVAL;
	}

	enable_irq(gpio_irq);
	vfsspi_device->is_drdy_irq_enabled = DRDY_IRQ_ENABLE;
	vfsspi_ioctl_enable_irq_wake(vfsspi_device);

	return 0;
}

static int vfsspi_disableIrq(struct vfsspi_device_data *vfsspi_device)
{
	pr_debug("%s: enter\n", __func__);

	if (vfsspi_device->is_drdy_irq_enabled == DRDY_IRQ_DISABLE) {
		pr_debug("%s: DRDY irq already disabled\n", __func__);
		return -EINVAL;
	}

	disable_irq_nosync(gpio_irq);
	vfsspi_ioctl_disable_irq_wake(vfsspi_device);
	vfsspi_device->is_drdy_irq_enabled = DRDY_IRQ_DISABLE;

	return 0;
}
static int vfsspi_set_drdy_int(struct vfsspi_device_data *vfsspi_device,
				   unsigned long arg)
{
	unsigned short drdy_enable_flag;
	if (copy_from_user(&drdy_enable_flag, (void __user *)arg,
			   sizeof(drdy_enable_flag)) != 0) {
		pr_err("%s: Failed copy from user.\n", __func__);
		return -EFAULT;
	}
	if (drdy_enable_flag == 0) {
		vfsspi_wake_lock_delayed_unlock(vfsspi_device);
		vfsspi_disableIrq(vfsspi_device);
	} else {
		vfsspi_enableIrq(vfsspi_device);
		/* Workaround the issue where the system
		  misses DRDY notification to host when
		  DRDY pin was asserted before enabling
		  device.*/
		if (gpio_get_value(vfsspi_device->drdy_pin) ==
			DRDY_ACTIVE_STATUS) {
			vfsspi_sendDrdyNotify(vfsspi_device);
		}
		//vfsspi_wake_unlock(vfsspi_device);
	}
	return 0;
}

static void vfsspi_hardReset(struct vfsspi_device_data *vfsspi_device)
{
	pr_debug("%s: enter\n", __func__);

	if (vfsspi_device != NULL) {
		gpio_set_value(vfsspi_device->sleep_pin, 0);
		mdelay(1);
		gpio_set_value(vfsspi_device->sleep_pin, 1);
		mdelay(5);
	}
}


static void vfsspi_suspend(struct vfsspi_device_data *vfsspi_device)
{
	pr_debug("%s: enter\n", __func__);

	if (vfsspi_device != NULL) {
		spin_lock(&vfsspi_device->vfs_spi_lock);
		gpio_set_value(vfsspi_device->sleep_pin, 0);
		spin_unlock(&vfsspi_device->vfs_spi_lock);
	}
}

static long vfsspi_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
	int ret_val = 0;
	struct vfsspi_device_data *vfsspi_device = NULL;

	pr_debug("%s: enter\n", __func__);

	if (_IOC_TYPE(cmd) != VFSSPI_IOCTL_MAGIC) {
		pr_err("%s: invalid magic. cmd=0x%X Received=0x%X Expected=0x%X\n",
			__func__, cmd, _IOC_TYPE(cmd), VFSSPI_IOCTL_MAGIC);
		return -ENOTTY;
	}

	if (ignore_ioctl) {
		pr_err("%s: ignore ioctl\n", __func__);
		return -EFAULT;
	}

	vfsspi_device = filp->private_data;
	mutex_lock(&vfsspi_device->buffer_mutex);
	switch (cmd) {
	case VFSSPI_IOCTL_POWER_ON:
		printk("%s: VFSSPI_IOCTL_POWER_ON\n", __func__);
		ret_val = 0;
		if (!vfsspi_device->power_enabled) {
			ret_val = vfsspi_ioctl_power_on(vfsspi_device);
			mdelay(15);
			vfsspi_hardReset(vfsspi_device);
		}
		break;
	case VFSSPI_IOCTL_POWER_OFF:
		// keep wake lock for some special senario
		vfsspi_wake_lock_delayed_unlock(vfsspi_device);
		printk("%s: VFSSPI_IOCTL_POWER_OFF\n", __func__);
		ret_val = vfsspi_ioctl_power_off(vfsspi_device);
		break;
	case VFSSPI_IOCTL_SET_SPI_CONFIGURATION:
		printk("%s: VFSSPI_IOCTL_SET_SPI_CONFIGURATION\n", __func__);
		break;
	case VFSSPI_IOCTL_RESET_SPI_CONFIGURATION:
		// keep wake lock for some special senario
		vfsspi_wake_lock_delayed_unlock(vfsspi_device);
		printk("%s: VFSSPI_IOCTL_RESET_SPI_CONFIGURATION\n", __func__);
		break;
	case VFSSPI_IOCTL_DEVICE_RESET:
		printk("%s: VFSSPI_IOCTL_DEVICE_RESET:\n", __func__);
		vfsspi_hardReset(vfsspi_device);
		ret_val = 0;
		break;
	case VFSSPI_IOCTL_DEVICE_SUSPEND:
	{
		printk("%s: VFSSPI_IOCTL_DEVICE_SUSPEND:\n", __func__);
		vfsspi_suspend(vfsspi_device);
		ret_val = 0;
		break;
	}
	case VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL:
		printk("%s: VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL, %08x\n", __func__, (unsigned int)arg);
		ret_val = vfsspi_register_drdy_signal(vfsspi_device, arg);
		break;
	case VFSSPI_IOCTL_SET_DRDY_INT:
		printk("%s: VFSSPI_IOCTL_SET_DRDY_INT, %08x\n", __func__, (unsigned int)arg);
		ret_val = vfsspi_set_drdy_int(vfsspi_device, arg);
		break;
	case VFSSPI_IOCTL_SELECT_DRDY_NTF_TYPE:
		{
			vfsspi_iocSelectDrdyNtfType_t drdyTypes;

			printk("%s: VFSSPI_IOCTL_SELECT_DRDY_NTF_TYPE\n", __func__);

			if (copy_from_user(&drdyTypes, (void __user *)arg,
				sizeof(vfsspi_iocSelectDrdyNtfType_t)) != 0) {
					pr_debug("%s: copy from user failed.\n", __func__);
					ret_val = -EFAULT;
			} else {
				if (0 != (drdyTypes.supportedTypes & VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD)) {
					vfsspi_device->drdy_ntf_type = VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD;
				} else {
					vfsspi_device->drdy_ntf_type = VFSSPI_DRDY_NOTIFY_TYPE_SIGNAL;
				}
				drdyTypes.selectedType = vfsspi_device->drdy_ntf_type;
				if (copy_to_user((void __user *)arg, &(drdyTypes),
					sizeof(vfsspi_iocSelectDrdyNtfType_t)) == 0) {
						ret_val = 0;
				} else {
					pr_debug("%s: copy to user failed\n", __func__);
				}
			}
			break;
		}
	default:
		ret_val = -EFAULT;
		break;
	}
	mutex_unlock(&vfsspi_device->buffer_mutex);
	return ret_val;
}

static int vfsspi_open(struct inode *inode, struct file *filp)
{
	struct vfsspi_device_data *vfsspi_device = NULL;
	int status = -ENXIO;

	pr_debug("%s: enter\n", __func__);

	mutex_lock(&device_list_mutex);

	list_for_each_entry(vfsspi_device, &device_list, device_entry) {
		if (vfsspi_device->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status == 0) {
		mutex_lock(&vfsspi_device->kernel_lock);
		if (vfsspi_device->is_opened != 0) {
			status = -EBUSY;
			pr_err("%s: is_opened != 0, -EBUSY\n", __func__);
			goto vfsspi_open_out;
		}
		vfsspi_device->user_pid = 0;
		vfsspi_device->drdy_ntf_type = VFSSPI_DRDY_NOTIFY_TYPE_SIGNAL;
		if (vfsspi_device->buffer != NULL) {
			pr_err("%s: buffer != NULL\n", __func__);
			goto vfsspi_open_out;
		}
		vfsspi_device->null_buffer =
			kmalloc(DEFAULT_BUFFER_SIZE, GFP_KERNEL);
		if (vfsspi_device->null_buffer == NULL) {
			status = -ENOMEM;
			pr_err("%s: null_buffer == NULL, -ENOMEM\n", __func__);
			goto vfsspi_open_out;
		}
		vfsspi_device->buffer =
			kmalloc(DEFAULT_BUFFER_SIZE, GFP_KERNEL);
		if (vfsspi_device->buffer == NULL) {
			status = -ENOMEM;
			kfree(vfsspi_device->null_buffer);
			pr_err("%s: buffer == NULL, -ENOMEM\n", __func__);
			goto vfsspi_open_out;
		}
		vfsspi_device->is_opened = 1;
		filp->private_data = vfsspi_device;
		nonseekable_open(inode, filp);

vfsspi_open_out:
		mutex_unlock(&vfsspi_device->kernel_lock);
	}
	mutex_unlock(&device_list_mutex);
	return status;
}


static int vfsspi_release(struct inode *inode, struct file *filp)
{
	struct vfsspi_device_data *vfsspi_device = NULL;
	int				   status	 = 0;

	pr_debug("%s: enter\n", __func__);

	mutex_lock(&device_list_mutex);
	vfsspi_device = filp->private_data;
	filp->private_data = NULL;
	vfsspi_device->is_opened = 0;
	if (vfsspi_device->buffer != NULL) {
		kfree(vfsspi_device->buffer);
		vfsspi_device->buffer = NULL;
	}

	if (vfsspi_device->null_buffer != NULL) {
		kfree(vfsspi_device->null_buffer);
		vfsspi_device->null_buffer = NULL;
	}

	mutex_unlock(&device_list_mutex);
	return status;
}

/* sysfs interface */
static ssize_t irq_check_get(struct device *device,
				 struct device_attribute *attribute,
				 char* buffer)
{
	struct vfsspi_device_data *vfsspi_device = dev_get_drvdata(device);

	return scnprintf(buffer, PAGE_SIZE, "%i", vfsspi_device->irq_check_flag);
}

static ssize_t irq_check_set(struct device *device,
				 struct device_attribute *attribute,
				 const char *buffer, size_t count)
{
	struct vfsspi_device_data *vfsspi_device = dev_get_drvdata(device);
	int err = 0;
	int retry = 0;
	unsigned int power_enabled;
	unsigned int is_drdy_enabled;

	pr_info("%s: enter\n", __func__);
	vfsspi_device->irq_check_flag = 0;
	power_enabled = vfsspi_device->power_enabled;
	is_drdy_enabled = vfsspi_device->is_drdy_irq_enabled;

	/* disable irq before test */
	if (is_drdy_enabled) {
		pr_info("%s: disable irq\n", __func__);
		vfsspi_disableIrq(vfsspi_device);
	}

	/* power off before test */
	if (power_enabled) {
		pr_info("%s: power off\n", __func__);
		vfsspi_ioctl_power_off(vfsspi_device);
	}

	/* perform hard reset & wait for irq */
	retry = 0;
	while (retry++ < 5) {
		/* power on & enable irq */
		pr_info("%s: power on\n", __func__);
		vfsspi_ioctl_power_on(vfsspi_device);
		mdelay(15);
		pr_info("%s: enable irq\n", __func__);
		vfsspi_enableIrq(vfsspi_device);
		mdelay(15);

		vfsspi_hardReset(vfsspi_device);
		err = down_timeout(&vfsspi_device->irq_check_sem, msecs_to_jiffies(200));
		if (err) {
			pr_err("%s: timeout\n", __func__);
		}

		if (vfsspi_device->irq_check_flag) {
			pr_info("%s: got irq!\n", __func__);
			break;
		}

		/* disable irq & power off */
		pr_info("%s: disable irq\n", __func__);
		vfsspi_disableIrq(vfsspi_device);
		pr_info("%s: power off\n", __func__);
		vfsspi_ioctl_power_off(vfsspi_device);
	}

	/* restore irq */
	if (is_drdy_enabled != DRDY_IRQ_ENABLE) {
		pr_info("%s: need to disable irq (restore)\n", __func__);
		vfsspi_disableIrq(vfsspi_device);
	}

	/* restore power */
	if (!power_enabled) {
		pr_info("%s: need to disable power (restore)\n", __func__);
		vfsspi_ioctl_power_off(vfsspi_device);
	}

	return count;
}
static DEVICE_ATTR(irq_check, S_IRUSR | S_IWUSR, irq_check_get, irq_check_set);

static ssize_t shutdown_set(struct device *device,
				 struct device_attribute *attribute,
				 const char *buffer, size_t count)
{
	struct vfsspi_device_data *vfsspi_device = dev_get_drvdata(device);

	pr_info("%s()\n", __func__);
	if (vfsspi_device) {
		vfsspi_drv_shutdown(vfsspi_device->spi);
	}

	return count;
}
static DEVICE_ATTR(shutdown, S_IRUSR | S_IWUSR, NULL, shutdown_set);

static struct attribute* attributes[] =
{
	&dev_attr_irq_check.attr,
	&dev_attr_shutdown.attr,
	NULL
};

static struct attribute_group attr_group =
{
	.attrs = attributes,
};

/* file operations associated with device */
static const struct file_operations vfsspi_fops = {
	.owner   = THIS_MODULE,
	.unlocked_ioctl   = vfsspi_ioctl,
	.open	= vfsspi_open,
	.release = vfsspi_release,
};

static int vfsspi_probe(struct platform_device *spi)
{
	int status = 0;
	struct vfsspi_device_data *vfsspi_device;
	struct device *dev;

	pr_info("%s: enter\n", __func__);

	vfsspi_device = kzalloc(sizeof(*vfsspi_device), GFP_KERNEL);
	if (vfsspi_device == NULL) {
		pr_err("%s: no memory to init driver data.\n", __func__);
		return -ENOMEM;
	}

	/* Initialize driver data. */
	vfsspi_device->irq_wake_enabled = 0;
	vfsspi_device->current_spi_speed = SLOW_BAUD_RATE;
	vfsspi_device->spi = spi;
	spin_lock_init(&vfsspi_device->vfs_spi_lock);
	mutex_init(&vfsspi_device->buffer_mutex);
	mutex_init(&vfsspi_device->kernel_lock);
	INIT_LIST_HEAD(&vfsspi_device->device_entry);

	/* init wake lock */
	vfsspi_device->wake_lock_acquired = 0;
	wake_lock_init(&vfsspi_device->wake_lock, WAKE_LOCK_SUSPEND, "fingerprint_wakelock");

	/* init work queue for interrupt */
	INIT_WORK(&vfsspi_device->irq_worker, vfsspi_irq_worker);
	init_timer(&vfsspi_device->wake_unlock_timer);
	vfsspi_device->wake_unlock_timer.expires = jiffies - 1;
	vfsspi_device->wake_unlock_timer.function = vfsspi_wake_unlock_timer_handler;
	vfsspi_device->wake_unlock_timer.data = (unsigned long)vfsspi_device;
	add_timer(&vfsspi_device->wake_unlock_timer);

	/* Power up for the sensor */
	if (vfsspi_ioctl_power_init(spi, vfsspi_device)) {
		goto vfsspi_probe_power_init_failed;

	}
	if (vfsspi_ioctl_power_on(vfsspi_device)) {
		goto vfsspi_probe_power_on_failed;
	}

	vfsspi_device->drdy_pin  = VFSSPI_DRDY_PIN;
	vfsspi_device->sleep_pin = VFSSPI_SLEEP_PIN;

#ifndef CONFIG_OF
	if (gpio_request(vfsspi_device->drdy_pin, "vfsspi_drdy") < 0) {
		status = -EBUSY;
		goto vfsspi_probe_drdy_failed;
	}

	if (gpio_request(vfsspi_device->sleep_pin, "vfsspi_sleep")) {
		status = -EBUSY;
		goto vfsspi_probe_sleep_failed;
	}
#else
	if(spi->dev.of_node){
		vfsspi_device->drdy_pin = of_get_named_gpio(spi->dev.of_node, "synaptics,gpio_drdy", 0);
		if(vfsspi_device->drdy_pin == 0) {
			pr_err("%s: Failed to get drdy_pin gpio.\n", __func__);
			return 0;
		}
		vfsspi_device->sleep_pin = of_get_named_gpio(spi->dev.of_node, "synaptics,gpio_sleep", 0);
		if(vfsspi_device->sleep_pin == 0) {
			pr_err("%s: Failed to get sleep_pin gpio.\n", __func__);
			return 0;
		}
	} else {
		pr_err("%s: init gpio failed.\n", __func__);
	}
#endif

#if DO_CHIP_SELECT
	pr_debug("%s: HANDLING CHIP SELECT\n", __func__);
	vfsspi_device->cs_pin  = VFSSPI_CS_PIN;
#ifndef CONFIG_OF
	if (gpio_request(vfsspi_device->cs_pin, "vfsspi_cs") < 0) {
		status = -EBUSY;
		goto vfsspi_probe_cs_failed;
	}
#else
	vfsspi_device->cs_pin = of_get_named_gpio(spi->dev.of_node, "synaptics,gpio_cs", 0);
	pr_info("%s: vfsspi_device->cs_pin = %d\n", __func__, (int)vfsspi_device->cs_pin);
	if(vfsspi_device->cs_pin == 0) {
		pr_err("%s: Failed to get cs_pin gpio.\n", __func__);
		return 0;
	}
#endif
	status = gpio_direction_output(vfsspi_device->cs_pin, 1);
	if (status < 0) {
		pr_err("%s: gpio_direction_input CS failed\n", __func__);
		status = -EBUSY;
		goto vfsspi_probe_gpio_init_failed;
	}
	gpio_set_value(vfsspi_device->cs_pin, 1);
#endif // end DO_CHIP_SELECT

	status = gpio_direction_output(vfsspi_device->sleep_pin, 1);
	if (status < 0) {
		pr_err("%s: gpio_direction_output SLEEP failed\n", __func__);
		status = -EBUSY;
		goto vfsspi_probe_gpio_init_failed;
	}

	status = gpio_direction_input(vfsspi_device->drdy_pin);
	if (status < 0) {
		pr_err("%s: gpio_direction_input DRDY failed\n", __func__);
		status = -EBUSY;
		goto vfsspi_probe_gpio_init_failed;
	}

	/* register interrupt */
	gpio_irq = gpio_to_irq(vfsspi_device->drdy_pin);
	if (gpio_irq < 0) {
		pr_err("%s: gpio_to_irq failed\n", __func__);
		status = -EBUSY;
		goto vfsspi_probe_gpio_init_failed;
	}

	if (request_irq(gpio_irq, vfsspi_irq, IRQF_TRIGGER_RISING,
			"vfsspi_irq", vfsspi_device) < 0) {
		pr_err("%s: request_irq failed\n", __func__);
		status = -EBUSY;
		goto vfsspi_probe_irq_failed;
	}

	vfsspi_device->is_drdy_irq_enabled = DRDY_IRQ_ENABLE;

	mutex_lock(&device_list_mutex);
	/* Create device node */
	/* register major number for character device */
	status = alloc_chrdev_region(&(vfsspi_device->devt),
					 0, 1, VALIDITY_PART_NAME);
	if (status < 0) {
		pr_err("%s: alloc_chrdev_region failed\n", __func__);
		goto vfsspi_probe_alloc_chardev_failed;
	}

	cdev_init(&(vfsspi_device->cdev), &vfsspi_fops);
	vfsspi_device->cdev.owner = THIS_MODULE;
	status = cdev_add(&(vfsspi_device->cdev), vfsspi_device->devt, 1);
	if (status < 0) {
		pr_err("%s: cdev_add failed\n", __func__);
		unregister_chrdev_region(vfsspi_device->devt, 1);
		goto vfsspi_probe_cdev_add_failed;
	}

	vfsspi_device_class = class_create(THIS_MODULE, VALIDITY_PART_NAME);
	if (IS_ERR(vfsspi_device_class)) {
		pr_err("%s: class_create() is failed - unregister chrdev.\n", __func__);
		cdev_del(&(vfsspi_device->cdev));
		unregister_chrdev_region(vfsspi_device->devt, 1);
		status = PTR_ERR(vfsspi_device_class);
		goto vfsspi_probe_class_create_failed;
	}

	/* create /dev/vfsspi */
	dev = device_create(vfsspi_device_class, &spi->dev,
				vfsspi_device->devt, vfsspi_device, "vfsspi");
	status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	if (status == 0)
		list_add(&vfsspi_device->device_entry, &device_list);
	mutex_unlock(&device_list_mutex);
	if (status != 0)  {
		pr_err("%s: vfsspi create status: %d\n", __func__, (int)status);
		goto vfsspi_probe_failed;
	}

	/* init irq check */
	sema_init(&vfsspi_device->irq_check_sem, -1);
	vfsspi_device->irq_check_flag = 0;
	status = sysfs_create_group(&spi->dev.kobj, &attr_group);
	if (status) {
		pr_err("%s: fail to create sysfs devices\n", __func__);
		goto vfsspi_probe_sysfs_create_failed;
	}

	/* bind driver data */
	platform_set_drvdata(spi, vfsspi_device);
	dev_set_drvdata(&spi->dev, vfsspi_device);
	ignore_ioctl = 0;

	/* perform hw reset */
	mdelay(15);
	vfsspi_hardReset(vfsspi_device);

	pr_info("%s: vfsspi_probe successful\n", __func__);

	return 0;

vfsspi_probe_sysfs_create_failed:
	list_del(&vfsspi_device->device_entry);
	device_destroy(vfsspi_device_class, vfsspi_device->devt);
	class_destroy(vfsspi_device_class);
vfsspi_probe_failed:
vfsspi_probe_class_create_failed:
	cdev_del(&(vfsspi_device->cdev));
vfsspi_probe_cdev_add_failed:
	unregister_chrdev_region(vfsspi_device->devt, 1);
vfsspi_probe_alloc_chardev_failed:
vfsspi_probe_irq_failed:
	free_irq(gpio_irq, vfsspi_device);
vfsspi_probe_gpio_init_failed:
#if DO_CHIP_SELECT
#ifndef CONFIG_OF
		gpio_free(vfsspi_device->cs_pin);
vfsspi_probe_cs_failed:
#endif
#endif

#ifndef CONFIG_OF
	gpio_free(vfsspi_device->sleep_pin);
vfsspi_probe_sleep_failed:
	gpio_free(vfsspi_device->drdy_pin);
vfsspi_probe_drdy_failed:
#endif

vfsspi_probe_power_on_failed:
	vfsspi_ioctl_power_uninit(vfsspi_device);
vfsspi_probe_power_init_failed:
	kfree(vfsspi_device);
	mutex_destroy(&vfsspi_device->buffer_mutex);
	mutex_destroy(&vfsspi_device->kernel_lock);
	pr_err("%s: vfsspi_probe failed!!\n", __func__);
	return status;
}

static int vfsspi_remove(struct platform_device *spi)
{
	int status = 0;

	struct vfsspi_device_data *vfsspi_device = NULL;

	pr_info("%s()\n", __func__);
	ignore_ioctl = 1;
	vfsspi_device = platform_get_drvdata(spi);
	if (vfsspi_device != NULL) {
		vfsspi_ioctl_power_off(vfsspi_device);
		spin_lock_irq(&vfsspi_device->vfs_spi_lock);
		vfsspi_device->spi = NULL;
		platform_set_drvdata(spi, NULL);
		spin_unlock_irq(&vfsspi_device->vfs_spi_lock);

		mutex_lock(&device_list_mutex);

		free_irq(gpio_irq, vfsspi_device);

#ifndef CONFIG_OF
#if DO_CHIP_SELECT
		gpio_free(vfsspi_device->cs_pin);
#endif
		gpio_free(vfsspi_device->sleep_pin);
		gpio_free(vfsspi_device->drdy_pin);
#endif
		/* Remove device entry. */
		list_del(&vfsspi_device->device_entry);
		device_destroy(vfsspi_device_class, vfsspi_device->devt);
		class_destroy(vfsspi_device_class);
		cdev_del(&(vfsspi_device->cdev));
		unregister_chrdev_region(vfsspi_device->devt, 1);

		mutex_destroy(&vfsspi_device->buffer_mutex);
		mutex_destroy(&vfsspi_device->kernel_lock);

		kfree(vfsspi_device);
		mutex_unlock(&device_list_mutex);

		pr_info("%s: fingerprint driver removed\n", __func__);
	}

	return status;
}


struct platform_driver vfsspi_spi = {
	.driver = {
		.name  = VALIDITY_PART_NAME,
		.owner = THIS_MODULE,
		.of_match_table = validity_metallica_table,
	},
	.probe  = vfsspi_probe,
	.remove = vfsspi_remove,
	.suspend  = vfsspi_drv_suspend,
	.resume = vfsspi_drv_resume,
	.shutdown = vfsspi_drv_shutdown,
};

static int __init vfsspi_init(void)
{
	int status = 0;

	pr_debug("%s: enter\n", __func__);

	status = platform_driver_register(&vfsspi_spi);
	if (status < 0) {
		pr_err("%s: register driver() is failed\n", __func__);
		return status;
	}
	pr_debug("%s: init is successful\n", __func__);

	return status;
}

static void __exit vfsspi_exit(void)
{
	pr_debug("%s: enter\n", __func__);

	platform_driver_unregister(&vfsspi_spi);
}

module_init(vfsspi_init);
module_exit(vfsspi_exit);

MODULE_DESCRIPTION("Validity FPS sensor");
MODULE_LICENSE("GPL");
