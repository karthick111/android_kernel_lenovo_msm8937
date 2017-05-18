#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/proc_fs.h>

#include <linux/hardware_info.h>

//static int hardwareinfo_set_defaults(void);
static DEFINE_MUTEX(hardwareinfo_mutex);

char Lcm_name[HARDWARE_MAX_ITEM_LENGTH];//req  wuzhenzhen.wt 20140901 add for hardware info
static char hardwareinfo_name[HARDWARE_MAX_ITEM][HARDWARE_MAX_ITEM_LENGTH];

int hardwareinfo_set_prop(int cmd, const char *name)
{
	if(cmd < 0 || cmd >= HARDWARE_MAX_ITEM)
		return -1;

	strcpy(hardwareinfo_name[cmd], name);

	return 0;
}
EXPORT_SYMBOL_GPL(hardwareinfo_set_prop);

/*
static int hardwareinfo_set_defaults()
{
	hardwareinfo_set_prop(HARDWARE_ACCELEROMETER, "BMI120");
	hardwareinfo_set_prop(HARDWARE_ALSPS, "Epl259x");
	hardwareinfo_set_prop(HARDWARE_GYROSCOPE, "BMI120");
	hardwareinfo_set_prop(HARDWARE_MAGNETOMETER, "St480m");

	return 0;
}
*/
//int (*open) (struct inode *, struct file *);
static int hardwareinfo_open(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "%s()...\n", __func__);
	filp->private_data = &hardwareinfo_mutex;
	return 0;
}

//int (*release) (struct inode *, struct file *);
static int hardwareinfo_release(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "%s()...\n", __func__);
	filp->private_data = 0;
	return 0;
}

//ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
static ssize_t hardwareinfo_read(struct file *filp, char __user *buf, size_t size, loff_t * ppos)
{
	char buf_k[HARDWARE_MAX_ITEM_LENGTH + 10];
	int i;
	unsigned long count;
	unsigned long pos;
	int ret = 0;
	
//	printk(KERN_INFO "%s(), size=%x, *ppos=%x\n", __func__, size, *ppos);

	if (*ppos)
		return 0;	/* the end */

	mutex_lock(&hardwareinfo_mutex);
	pos = count = 0;
	for (i = 0; i < HARDWARE_MAX_ITEM; i++) {
		printk(KERN_INFO "%s(), i=%d, %s\n", __func__, i, hardwareinfo_name[i]);
		ret = sprintf(buf_k, "%d, %s\n", i, hardwareinfo_name[i]);
		count = strlen(buf_k);
		if (pos + count > size)
			break;
		if (copy_to_user(buf + pos, buf_k, count))
			break;
		pos += count;
		printk(KERN_INFO "%s(), pos=%lu\n", __func__, pos);
	}
	*ppos += pos;	/* increase offset */

	mutex_unlock(&hardwareinfo_mutex);
	printk(KERN_INFO "%s(), pos=%lu\n", __func__, pos);

	return pos;
}

//ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
static ssize_t hardwareinfo_write(struct file *filp, const char __user *buf, size_t size, loff_t * ppos)
{
	char buf_k[HARDWARE_MAX_ITEM_LENGTH + 10];
	char info[HARDWARE_MAX_ITEM_LENGTH];
	int n;
	int ret = 0;
	
//	printk(KERN_INFO "%s(), size=%lu, *ppos=%llu\n", __func__, size, *ppos);

	if (*ppos >= size)
		return 0;

	mutex_lock(&hardwareinfo_mutex);
	if(size > sizeof(buf_k))
		size = sizeof(buf_k);
	if(!copy_from_user(buf_k, buf, size)) {
		ret = sscanf(buf_k, "%d,%64s", &n, info);
		if (n < HARDWARE_MAX_ITEM) {
			printk(KERN_INFO "%s(), n=%d, %s\n", __func__, n, info);
			strcpy(hardwareinfo_name[n], info);
		}
	}
	//*ppos += size;	/* increase offset */

	mutex_unlock(&hardwareinfo_mutex);
	//printk(KERN_INFO "%s(), size=%u\n", __func__, size);

	return size;
}

static long hardwareinfo_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	int ret = 0, hardwareinfo_num;
	void __user *data = (void __user *)arg;

	//char info[HARDWARE_MAX_ITEM_LENGTH];
	
	switch (cmd) { 
	case HARDWARE_LCD_GET:
		hardwareinfo_set_prop(HARDWARE_LCD, Lcm_name);//req  wuzhenzhen.wt 20140901 add for hardware info
		hardwareinfo_num = HARDWARE_LCD;
		break;
	case HARDWARE_TP_GET:
		hardwareinfo_num = HARDWARE_TP;
		break;
	case HARDWARE_FLASH_GET:
		hardwareinfo_num = HARDWARE_FLASH;
		break;
	case HARDWARE_FRONT_CAM_GET:
		hardwareinfo_num = HARDWARE_FRONT_CAM;
		break;
	case HARDWARE_BACK_CAM_GET:
		hardwareinfo_num = HARDWARE_BACK_CAM;
		break;
	case HARDWARE_BT_GET:
		hardwareinfo_set_prop(HARDWARE_BT, "Qualcomm");
		hardwareinfo_num = HARDWARE_BT;
		break;
	case HARDWARE_WIFI_GET:
		hardwareinfo_set_prop(HARDWARE_WIFI, "Qualcomm");
		hardwareinfo_num = HARDWARE_WIFI;
		break;
	case HARDWARE_ACCELEROMETER_GET:
		hardwareinfo_num = HARDWARE_ACCELEROMETER;
		break;
	case HARDWARE_ALSPS_GET:
		hardwareinfo_num = HARDWARE_ALSPS;
		break;
	case HARDWARE_GYROSCOPE_GET:
		hardwareinfo_num = HARDWARE_GYROSCOPE;
		break;
	case HARDWARE_MAGNETOMETER_GET:
		hardwareinfo_num = HARDWARE_MAGNETOMETER;
		break;
	case HARDWARE_GPS_GET:
		hardwareinfo_set_prop(HARDWARE_GPS, "Qualcomm");
	    hardwareinfo_num = HARDWARE_GPS;
		break;
	case HARDWARE_FM_GET:
		hardwareinfo_set_prop(HARDWARE_FM, "Qualcomm");
	    hardwareinfo_num = HARDWARE_FM;		
		break;
	case HARDWARE_BATTERY_ID_GET:
		hardwareinfo_num = HARDWARE_BATTERY_ID;
		break;		
	case HARDWARE_BACK_CAM_MOUDULE_ID_GET:
		hardwareinfo_num = HARDWARE_BACK_CAM_MOUDULE_ID;
		break;
	case HARDWARE_FRONT_CAM_MODULE_ID_GET:
		hardwareinfo_num = HARDWARE_FRONT_CAM_MOUDULE_ID;
		break;		
	case HARDWARE_BOARD_ID_GET:
		hardwareinfo_num = HARDWARE_BOARD_ID;
		break;	
	case HARDWARE_BACK_CAM_MOUDULE_ID_SET:
		if(copy_from_user(hardwareinfo_name[HARDWARE_BACK_CAM_MOUDULE_ID], data,strlen(data)))
		{
			pr_err("wgz copy_from_user error");
			ret =  -EINVAL;
		}
		goto set_ok;
		break;
	case HARDWARE_FRONT_CAM_MODULE_ID_SET:
		if(copy_from_user(hardwareinfo_name[HARDWARE_FRONT_CAM_MOUDULE_ID], data,strlen(data)))
		{
			pr_err("wgz copy_from_user error");
			ret =  -EINVAL;
		}
		goto set_ok;
		break;
	default:
		ret = -EINVAL;
		goto err_out;
	}
	memset(data, 0, HARDWARE_MAX_ITEM_LENGTH);//clear the buffer
	if (copy_to_user(data, hardwareinfo_name[hardwareinfo_num], strlen(hardwareinfo_name[hardwareinfo_num]))){
		//printk("%s, copy to usr error\n", __func__);
		ret =  -EINVAL;
	}
set_ok:
err_out:
	return ret;
}


static struct file_operations hardwareinfo_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = hardwareinfo_ioctl,
	.compat_ioctl = hardwareinfo_ioctl,
	.open = hardwareinfo_open,
	.release = hardwareinfo_release,
	.read = hardwareinfo_read,
	.write = hardwareinfo_write,
};

static struct miscdevice hardwareinfo_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hardwareinfo",
	.fops = &hardwareinfo_fops,
};


static int __init hardwareinfo_init_module(void)
{
	int ret, i;

	for(i = 0; i < HARDWARE_MAX_ITEM; i++)
		strcpy(hardwareinfo_name[i], "NULL");
	
	ret = misc_register(&hardwareinfo_device);
	if(ret < 0){
		//printk("%s, misc_register error\n", __func__);
		return -ENODEV;
	}
	//printk("%s, misc_register sucessful\n", __func__);
//	hardwareinfo_set_defaults();

	return 0;
}

static void __exit hardwareinfo_exit_module(void)
{
	misc_deregister(&hardwareinfo_device);
}

module_init(hardwareinfo_init_module);
module_exit(hardwareinfo_exit_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ming He <heming@wingtech.com>");


