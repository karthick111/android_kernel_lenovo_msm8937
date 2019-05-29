#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/fb.h>
#include <linux/uaccess.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/seq_file.h>
#include <linux/suspend.h>
#include <linux/kobject.h>
#include<linux/init.h>
#include<linux/kthread.h>
#include<linux/sched.h>
#include<linux/string.h>
#include<linux/ioctl.h>
#include <linux/regulator/consumer.h>
#include <linux/msm_iommu_domains.h>
#include <linux/leds.h>
#include "uc1617s.h"

/*#define DISPLAY_DEBUG */
/*#define SUPPORT_PINMUX */
/*#define TEST_MODE */
/*#define AUTO_ENABLE_BKLIGHT */

#define CMD	1
#define DATA	2

#define SPI_IO(num)     _IO('O', num)
#define DISPLAY_OFF     SPI_IO(1)
#define DISPLAY_ON      SPI_IO(2)
#define SNOOZE_OFF      SPI_IO(3)
#define SNOOZE_ON       SPI_IO(4)
#define DISPLAY_COMMIT  SPI_IO(5)
#define DISPLAY_BLANK	SPI_IO(6)

#define NONE		0
#define THIRD_PARTY	1

#define OFF	false
#define ON	true

#ifdef SUPPORT_PINMUX
#define PINCTRL_STATE_ACTIVE	"spi_default"
#define PINCTRL_STATE_SUSPEND	"spi_sleep"
#endif

DEFINE_LED_TRIGGER(bl_led_trigger);

struct uc1617sfb_par;
static int uc1617s_power_ctrl(struct uc1617sfb_par *par,
				int enable_state);
static int uc1617sfb_uc1617s_init(struct uc1617sfb_par *par);
struct uc1617sfb_ops {
	int (*init)(struct uc1617sfb_par *);
	int (*remove)(struct uc1617sfb_par *);
};

struct uc1617sfb_par {
	struct spi_device *spi;
	struct fb_info *info;
	struct uc1617sfb_ops *ops;
	struct regulator *vdd_vreg;
	struct regulator *avdd_vreg;
	struct pwm_device *pwm;
	u32 height;
	u32 width;
	u32 page_offset;
	u32 pwm_period;
	int reset;
	int datacmd;
	bool power_state;
	int bklt_ctrl;
	int bklt_level;
#if defined(CONFIG_FB) && defined(AUTO_ENABLE_BKLIGHT)
	struct notifier_block fb_notif;
#endif
#ifdef SUPPORT_PINMUX
	struct pinctrl *uc1617s_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
#endif
	u8 *blank_buf;
	bool display_screen_state; /* Either ON or OFF*/
};

struct uc1617sfb_par *lpar = NULL;
static void uc1617sfb_coordinate_reset(struct uc1617sfb_par *par);

static struct fb_var_screeninfo uc1617sfb_var = {
	.bits_per_pixel = 2,
};

static struct fb_fix_screeninfo uc1617sfb_fix = {
	.id	     = "UC1617S",
	.type	   = FB_TYPE_PACKED_PIXELS,
	.visual	 = FB_VISUAL_MONO10,
	.xpanstep       = 0,
	.ypanstep       = 0,
	.ywrapstep      = 0,
	.accel	  = FB_ACCEL_NONE,
};

static inline int uc1617sfb_write_dcx(struct uc1617sfb_par *par,
					int dcx, u8 cmd)
{
	int ret = 0;
	u8 *wr8;

	wr8 = kzalloc(sizeof(u8), GFP_KERNEL);
	if (!wr8)
		return -ENOMEM;
	*wr8 = cmd;
	/* Set the DCX = 0 before send command and DCX = 1 before send data */
	if (dcx == CMD)
		gpio_set_value(par->datacmd, 0);
	else if (dcx == DATA)
		gpio_set_value(par->datacmd, 1);
	ret = spi_write(par->spi, wr8, 1);
	if (ret)
		dev_err(&par->spi->dev, "Couldn't send SPI command.\n");
	kfree(wr8);

	return ret;
}

static void uc1617sfb_coordinate_reset(struct uc1617sfb_par *par)
{
	pr_err("Co-ordinate set for Window\n");
	uc1617sfb_write_dcx(par, CMD, 0x00);
	uc1617sfb_write_dcx(par, CMD, 0x60);
	uc1617sfb_write_dcx(par, CMD, 0x70);
	uc1617sfb_write_dcx(par, CMD, 0xf4);
	uc1617sfb_write_dcx(par, CMD, 0x00);
	uc1617sfb_write_dcx(par, CMD, 0xf5);
	uc1617sfb_write_dcx(par, CMD, 0x00);
	uc1617sfb_write_dcx(par, CMD, 0xf6);
	uc1617sfb_write_dcx(par, CMD, 0x17);
	uc1617sfb_write_dcx(par, CMD, 0xf7);
	uc1617sfb_write_dcx(par, CMD, 0x5f);
	uc1617sfb_write_dcx(par, CMD, 0xf9);
}
static int uc1617sfb_update_display(struct uc1617sfb_par *par)
{
	u8 *wr_data;
	u8 *vmem = par->info->screen_base;
#ifdef DISPLAY_DEBUG
	int i;
#endif

	wr_data = kzalloc(par->info->fix.smem_len, GFP_KERNEL);
	if (!wr_data)
		return -ENOMEM;
	memset(wr_data, 0x00, par->info->fix.smem_len);
	/* Copty the Share buffer to local buffer */
	memcpy(wr_data, vmem, par->info->fix.smem_len);

#ifdef DISPLAY_DEBUG
	/* Dump buffer before Write */
	pr_err("============================ Data Buffer =============================");
	for (i = 0; i < par->info->fix.smem_len/8; i++) {
		pr_err("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
			wr_data[i*8], wr_data[i*8+1], wr_data[i*8+2],
			wr_data[i*8+3], wr_data[i*8+4], wr_data[i*8+5],
			wr_data[i*8+6], wr_data[i*8+7]);
		msleep(2);
	}
#endif
	/* Reset the co-ordinate to biggening of display */
	uc1617sfb_coordinate_reset(par);
	gpio_set_value(par->datacmd, 1);
	spi_write(par->spi, wr_data, par->info->fix.smem_len);
	gpio_set_value(par->datacmd, 0);
	kfree(wr_data);

	return 0;
}


static ssize_t uc1617sfb_write(struct fb_info *info,
				const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct uc1617sfb_par *par = info->par;
	unsigned long total_size;
	unsigned long p = *ppos;
	u8 __iomem *dst;

	total_size = info->fix.smem_len;
	if (p > total_size)
		return -EINVAL;
	if (count + p > total_size)
		count = total_size - p;
	if (!count)
		return -EINVAL;
	dst = (void *) (info->screen_base + p);
	if (copy_from_user(dst, buf, count))
		return -EFAULT;
	if (par->display_screen_state == ON) {
		if (uc1617sfb_update_display(par))
			pr_err("Failed to transfer data\n");
	}
	/* *ppos += count; */
	return count;
}

static void uc1617sfb_fillrect(struct fb_info *info,
				const struct fb_fillrect *rect)
{
	struct uc1617sfb_par *par = info->par;

	sys_fillrect(info, rect);
	if (par->display_screen_state == ON)
		uc1617sfb_update_display(par);
}

static void uc1617sfb_copyarea(struct fb_info *info,
		const struct fb_copyarea *area)
{
	struct uc1617sfb_par *par = info->par;

	sys_copyarea(info, area);
	if (par->display_screen_state == ON)
		uc1617sfb_update_display(par);
}

static void uc1617sfb_imageblit(struct fb_info *info,
				const struct fb_image *image)
{
	struct uc1617sfb_par *par = info->par;

	sys_imageblit(info, image);
	if (par->display_screen_state == ON)
		uc1617sfb_update_display(par);
}

#if 0
static int uc1617sfb_mmap(struct fb_info *info,
			struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long start;
	unsigned int len;

	start = info->fix.smem_start;
	len = info->fix.smem_len;

	/* check range */
	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;
	if (offset > info->fix.smem_len - size)
		return -EINVAL;

	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);

	return remap_pfn_range(vma, vma->vm_start,
			virt_to_phys((void *)start) >> PAGE_SHIFT,
			len, vma->vm_page_prot);
}
#endif

static int uc1617sfb_ioctl(struct fb_info *info,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct uc1617sfb_par *par;

	if (info == NULL)
		return -EINVAL;

	par = info->par;

	switch (cmd) {
	case DISPLAY_OFF:
		uc1617sfb_coordinate_reset(par);
		gpio_set_value(par->datacmd, 1);
		spi_write(par->spi, par->blank_buf, par->info->fix.smem_len);
		uc1617sfb_write_dcx(par, CMD, 0xAC);
		ret = uc1617s_power_ctrl(par, 0);
		if (par->bklt_ctrl == THIRD_PARTY)
			led_trigger_event(bl_led_trigger, 0);
		par->display_screen_state = OFF;
		break;
	case DISPLAY_ON:
		ret = uc1617s_power_ctrl(par, 1);
		uc1617sfb_update_display(par);
		par->display_screen_state = ON;
		break;
	case DISPLAY_COMMIT:
		if (par->display_screen_state == ON)
			uc1617sfb_update_display(par);
		break;
	case DISPLAY_BLANK:
		uc1617sfb_coordinate_reset(par);
		gpio_set_value(par->datacmd, 1);
		spi_write(par->spi, par->blank_buf, par->info->fix.smem_len);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static struct fb_ops uc1617sfb_node = {
	.owner	  = THIS_MODULE,
	.fb_read	= fb_sys_read,
	.fb_write       = uc1617sfb_write,
	.fb_fillrect    = uc1617sfb_fillrect,
	.fb_copyarea    = uc1617sfb_copyarea,
	.fb_imageblit   = uc1617sfb_imageblit,
	/*.fb_mmap	= uc1617sfb_mmap, */
	.fb_ioctl       = uc1617sfb_ioctl,
};

#ifdef CONFIG_FB_DEFERRED_IO
static void uc1617sfb_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
	uc1617sfb_update_display(info->par);
}

static struct fb_deferred_io uc1617sfb_defio = {
	.delay	  = HZ,
	.deferred_io    = uc1617sfb_deferred_io,
};
#endif

static int uc1617s_power_ctrl(struct uc1617sfb_par *par,
				int enable_state)
{
	int rc = 0;

	if (enable_state) {
#ifdef SUPPORT_PINMUX
		rc = pinctrl_select_state(par->uc1617s_pinctrl,
				par->pinctrl_state_active);
		if (rc < 0)
			pr_err("failed to select pin to active state %d", rc);
#endif
		if (!par->power_state) {
			if (par->avdd_vreg) {
				rc = regulator_enable(par->avdd_vreg);
				pr_err("ST7735 enable avdd pass, rc=%d\n", rc);
				if (rc) {
					pr_err("ST7735 enable avdd failed,\
						rc=%d\n", rc);
					return -ENODEV;
				}
			}
			if (par->vdd_vreg) {
				rc = regulator_enable(par->vdd_vreg);
				pr_err("ST7735 enable vdd pass, rc=%d\n", rc);
				if (rc) {
					pr_err("ST7735 enable vdd failed,\
						rc=%d\n", rc);
					return -ENODEV;
				}
			}


			par->power_state = true;
		}
		/* Delay is required to stable the power sequence */
		msleep(2);
			/* Reset LCD panel */
		gpio_set_value(par->reset, 1);
		msleep(20);
		gpio_set_value(par->reset, 0);
		msleep(20);
		gpio_set_value(par->reset, 1);
		msleep(20);
		/* After reset Slave device init the display */
		if (par->ops->init)
			par->ops->init(par);
		if (par->bklt_ctrl == THIRD_PARTY)
			led_trigger_event(bl_led_trigger, par->bklt_level);
	} else if (!enable_state) {
		if (par->ops->remove)
			par->ops->remove(par);
		if (par->power_state) {
#ifdef SUPPORT_PINMUX
			rc = pinctrl_select_state(par->uc1617s_pinctrl,
					par->pinctrl_state_suspend);
			if (rc < 0)
				pr_err("failed to select pin to suspend\
					state %d", rc);
#endif
			if (par->vdd_vreg) {
				rc = regulator_disable(par->vdd_vreg);
				pr_err("ST7735 disable vdd pass, rc=%d\n", rc);
				if (rc) {
					pr_err("ST7735 disable vdd failed,\
						rc=%d\n", rc);
					return -ENODEV;
				}
			}
			if (par->avdd_vreg) {
				rc = regulator_disable(par->avdd_vreg);
				pr_err("ST7735 disable vdd pass, rc=%d\n", rc);
				if (rc) {
					pr_err("ST7735 disable avdd failed,\
						rc=%d\n", rc);
					return -ENODEV;
				}
			}
			par->power_state = false;
			gpio_set_value(par->reset, 0);
		}
	}

	return 0;
}

static int uc1617sfb_uc1617s_remove(struct uc1617sfb_par *par)
{
	return 0;
}

static int uc1617sfb_uc1617s_init(struct uc1617sfb_par *par)
{
	u8 *blank_buf;
#ifdef TEST_MODE
	u8 __iomem *dst;
#endif
	blank_buf = devm_kzalloc(&par->spi->dev, par->info->fix.smem_len,
					GFP_KERNEL);
	memset(blank_buf, 0x00, par->info->fix.smem_len);

	gpio_set_value(par->datacmd, 0);
	msleep(100);
	uc1617sfb_write_dcx(par, CMD, 0xE2);
	msleep(50);
	uc1617sfb_write_dcx(par, CMD, 0xEB);
	uc1617sfb_write_dcx(par, CMD, 0x24);
	uc1617sfb_write_dcx(par, CMD, 0x2A);
	uc1617sfb_write_dcx(par, CMD, 0x2F);
	uc1617sfb_write_dcx(par, CMD, 0x81);
	uc1617sfb_write_dcx(par, CMD, 0x44);
	uc1617sfb_write_dcx(par, CMD, 0x84);
	uc1617sfb_write_dcx(par, CMD, 0x89);
	uc1617sfb_write_dcx(par, CMD, 0xA2);
	uc1617sfb_write_dcx(par, CMD, 0xC4);
	uc1617sfb_write_dcx(par, CMD, 0xA7);
	uc1617sfb_write_dcx(par, CMD, 0xF1);
	uc1617sfb_write_dcx(par, CMD, 0x5F);
	uc1617sfb_write_dcx(par, CMD, 0xAD);
	gpio_set_value(par->datacmd, 1);
	/*Write blank buffer */
	uc1617sfb_coordinate_reset(par);
	msleep(50);
	gpio_set_value(par->datacmd, 1);
	spi_write(par->spi, blank_buf, par->info->fix.smem_len);
#ifdef TEST_MODE
	gpio_set_value(par->datacmd, 0);
	uc1617sfb_coordinate_reset(par);
	msleep(50);
	gpio_set_value(par->datacmd, 1);
	memcpy(blank_buf, logo, par->info->fix.smem_len);
	dst = (void *) (par->info->screen_base);
	memcpy(dst, blank_buf, par->info->fix.smem_len);

	if (uc1617sfb_update_display(par))
		pr_err("Failed to write log\n");
#endif
	return 0;
}

static struct uc1617sfb_ops uc1617sfb_uc1617s_ops = {
	.init   = uc1617sfb_uc1617s_init,
	.remove = uc1617sfb_uc1617s_remove,
};

static const struct of_device_id uc1617sfb_of_match[] = {
	{
		.compatible = "rocktech,uc1617sfb-spi",
		.data = (void *)&uc1617sfb_uc1617s_ops,
	},
	{},
};

MODULE_DEVICE_TABLE(of, uc1617sfb_of_match);

#if defined(CONFIG_FB) && defined(AUTO_ENABLE_BKLIGHT)
static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	static int *blank;

	if (evdata && evdata->data) {
		if (event == FB_EVENT_BLANK) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK) {
				uc1617sfb_coordinate_reset(lpar);
				msleep(50);
				gpio_set_value(lpar->datacmd, 1);
				uc1617sfb_update_display(lpar);
				if (lpar->bklt_ctrl == THIRD_PARTY) {
					led_trigger_event(bl_led_trigger,
						lpar->bklt_level);
					lpar->display_screen_state = ON;
				}
			} else if (*blank == FB_BLANK_POWERDOWN ||
				*blank == FB_BLANK_VSYNC_SUSPEND) {
				uc1617sfb_coordinate_reset(lpar);
				msleep(50);
				gpio_set_value(lpar->datacmd, 1);
				spi_write(lpar->spi, lpar->blank_buf,
						lpar->info->fix.smem_len);
				if (lpar->bklt_ctrl == THIRD_PARTY) {
					led_trigger_event(bl_led_trigger, 0);
					lpar->display_screen_state = OFF;
				}
			}
		}
	}

	return 0;
}
#endif

#ifdef TEST_MODE
static ssize_t uc1617sfb_display_color(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	u8 __iomem *dst;
	u8 *temp_buffer = NULL;
	int i = 0;

	 if (!lpar)
		return -ENOMEM;

	temp_buffer = kzalloc(lpar->info->fix.smem_len, GFP_KERNEL);
	if (temp_buffer == NULL)
		return -ENOMEM;
	memset(temp_buffer, 0x00, lpar->info->fix.smem_len);

	switch (buf[0]) {
	case 'r':
		pr_err("Display RED image\n");
		for (i = 0; i < lpar->info->fix.smem_len/2; i++) {
			temp_buffer[i*2] = 0xf8;
			temp_buffer[(i*2)+1] = 0x00;
		}
		break;
	case 'g':
		pr_err("Display GREEN image\n");
		for (i = 0; i < lpar->info->fix.smem_len/2; i++) {
			temp_buffer[i*2] = 0x0f;
			temp_buffer[(i*2)+1] = 0xd0;
		}
		break;
	case 'b':
		pr_err("Display BLUE image\n");
		for (i = 0; i < lpar->info->fix.smem_len/2; i++) {
			temp_buffer[i*2] = 0x00;
			temp_buffer[(i*2)+1] = 0x1f;
		}
		break;
	case 'c':
		pr_err("Display BLANK image\n");
		memset(temp_buffer, 0x00, lpar->info->fix.smem_len);
		break;
	case 'l':
		gpio_set_value(lpar->datacmd, 1);
		pr_err("Display LOGO image\n");
		/*spi_write(lpar->spi, logo, lpar->info->fix.smem_len);*/
		pr_err("##### logo SIZE: %d\n", sizeof(logo)/sizeof(logo[0]));
		memcpy(temp_buffer, logo, lpar->info->fix.smem_len);
		break;
	default:
		pr_err("Display all 0xff image\n");
		memset(temp_buffer, 0xFF, lpar->info->fix.smem_len);
	}

	dst = (void *) (lpar->info->screen_base);
	memcpy(dst, temp_buffer, lpar->info->fix.smem_len);

	if (uc1617sfb_update_display(lpar))
		pr_err("Test mode data transfer failed\n");

	kfree(temp_buffer);
	return count;
}

static ssize_t uc1617sfb_display_on_off(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	switch (buf[0]) {
	case 'o':
	case 'O':
		pr_err("TURN ON DISPLAY\n");
		if (lpar->bklt_ctrl == THIRD_PARTY)
			led_trigger_event(bl_led_trigger, 200);
		break;
	case 'f':
	case 'F':
		pr_err("TURN OFF DISPLAY\n");
		if (lpar->bklt_ctrl == THIRD_PARTY)
			led_trigger_event(bl_led_trigger, 0);
		break;
	default:
		pr_err("Default TURN OFF DISPLAY\n");
		if (lpar->bklt_ctrl == THIRD_PARTY)
			led_trigger_event(bl_led_trigger, 0);
	}
	return count;
}

static struct kobj_attribute uc1617sfb_kobj_attr =
		__ATTR(display_color, 0600,
			NULL, uc1617sfb_display_color);
static struct kobj_attribute uc1617sfb_kobj_attr_led =
		__ATTR(display_turn, 0600,
			NULL, uc1617sfb_display_on_off);
#endif

static int uc1617sfb_parse_dt(struct uc1617sfb_par *par)
{
	int ret = 0;
	const char *data;
	struct device_node *node = par->spi->dev.of_node;

	if (!node) {
		pr_err("No device tree data found!\n");
		return -EINVAL;
	}
	par->reset = of_get_named_gpio(node,
			"rocktech,reset-gpios", 0);
	if (!gpio_is_valid(par->reset))
		ret = -EINVAL;
	par->datacmd = of_get_named_gpio(node,
			"rocktech,datacmd-gpios", 0);
	if (!gpio_is_valid(par->datacmd))
		ret = -EINVAL;
	if (of_property_read_u32(node, "rocktech,width", &par->width))
		par->width = 96;
	if (of_property_read_u32(node, "rocktech,height", &par->height))
		par->width = 96;
	if (of_property_read_u32(node, "rocktech,page-offset",
					&par->page_offset))
		par->page_offset = 1;
	par->bklt_ctrl = NONE;
	data = of_get_property(node, "rocktech,bklt-ctrl", NULL);
	if (!strcmp(data, "third-party"))
		par->bklt_ctrl = THIRD_PARTY;
	if (of_property_read_u32(node, "rocktech,default-bklt",
					&par->bklt_level))
		par->bklt_level = 200;
	par->vdd_vreg = regulator_get(&par->spi->dev, "vdd");
	if (IS_ERR(par->vdd_vreg)) {
		pr_err("ST7735 %s could not get vdd,", __func__);
		ret = -EINVAL;
	}
	par->avdd_vreg = regulator_get(&par->spi->dev, "avdd");
	if (IS_ERR(par->avdd_vreg)) {
		pr_err("ST7735 %s could not get avdd,", __func__);
		ret = -EINVAL;
	}
	return ret;
}

static int uc1617sfb_config_init(struct uc1617sfb_par *par)
{
	int ret = 0;
	struct spi_device *spi = par->spi;

#ifdef SUPPORT_PINMUX
	/* Get pinctrl if target uses pinctrl */
	par->uc1617s_pinctrl = devm_pinctrl_get(&(spi->dev));
	if (IS_ERR_OR_NULL(par->uc1617s_pinctrl)) {
		ret = PTR_ERR(par->uc1617s_pinctrl);
		dev_err(&spi->dev,
				"Target does not use pinctrl %d\n", ret);
		goto err_pinctrl_get;
	}

	par->pinctrl_state_active = pinctrl_lookup_state(par->uc1617s_pinctrl,
							PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(par->pinctrl_state_active)) {
		ret = PTR_ERR(par->pinctrl_state_active);
		dev_err(&spi->dev,
				"Can not lookup %s pinstate %d\n",
				PINCTRL_STATE_ACTIVE, ret);
		goto err_pinctrl_lookup;
	}

	par->pinctrl_state_suspend = pinctrl_lookup_state(par->uc1617s_pinctrl,
							PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(par->pinctrl_state_suspend)) {
		ret = PTR_ERR(par->pinctrl_state_suspend);
		dev_err(&spi->dev, "Can not lookup %s pinstate %d\n",
				PINCTRL_STATE_SUSPEND, ret);
		goto err_pinctrl_lookup;
	}

#endif
	ret = devm_gpio_request_one(&spi->dev, par->reset,
			GPIOF_OUT_INIT_HIGH, "lcd-reset");
	if (ret) {
		dev_err(&spi->dev, "failed to request gpio %d: %d\n",
				par->reset, ret);
		ret = -EINVAL;
	}
	ret = devm_gpio_request_one(&spi->dev, par->datacmd,
			GPIOF_OUT_INIT_HIGH, "lcd-datacmd");
	if (ret) {
		dev_err(&spi->dev, "failed to request gpio %d: %d\n",
				par->datacmd, ret);
		ret = -EINVAL;
	}

	/* Set voltage for regulator */
	if (par->vdd_vreg && par->avdd_vreg) {
		ret = regulator_set_voltage(par->vdd_vreg, 1800000, 1800000);
		pr_err("ST7735 seting voltage1\n");
		if (ret) {
			pr_err("ST7735 vdd_vreg->set_voltage failed,\
				rc=%d\n", ret);
			ret = -EINVAL;
		}
		ret = regulator_set_voltage(par->avdd_vreg, 2850000, 2850000);
		pr_err("ST7735 seting voltage2\n");
		if (ret) {
			pr_err("ST7735 avdd_vreg->set_voltage failed,\
				rc=%d\n", ret);
			ret = -EINVAL;
		}
	}

	if (par->bklt_ctrl == THIRD_PARTY)
		led_trigger_register_simple("bkl-trigger", &bl_led_trigger);

	return ret;

#ifdef SUPPORT_PINMUX
err_pinctrl_lookup:
	devm_pinctrl_put(par->uc1617s_pinctrl);
err_pinctrl_get:
	par->uc1617s_pinctrl = NULL;
	return ret;
#endif
}

unsigned long uc1617sfb_alloc_buffer(u32 buf_size)
{
	u32 order, size;
	unsigned long virt_addr, addr;

	size = PAGE_ALIGN(buf_size);
	order = get_order(size);
	virt_addr = __get_free_pages(GFP_KERNEL, order);
	addr = virt_addr;
	if (virt_addr) {
		while (size > 0) {
			SetPageReserved(virt_to_page(addr));
			addr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
	}

	return virt_addr;
}

static int uc1617sfb_probe(struct spi_device *spi)
{
	struct fb_info *info;
	struct uc1617sfb_par *par;
	const struct of_device_id *of_match;
	u32 vmem_size;
	u8 *vmem;
	int ret;

	info = framebuffer_alloc(sizeof(struct uc1617sfb_par), &spi->dev);
	if (!info) {
		dev_err(&spi->dev, "Couldn't allocate framebuffer.\n");
		return -ENOMEM;
	}
	par = info->par;
	par->info = info;
	par->spi = spi;
	par->power_state = false;
	of_match = of_match_device(uc1617sfb_of_match, &spi->dev);
	if (!of_match) {
		ret = -EINVAL;
		goto fb_alloc_error;
	}
	par->ops = (struct uc1617sfb_ops *)of_match->data;
	ret = uc1617sfb_parse_dt(par);
	if (ret)
		goto fb_alloc_error;
	info->fbops = &uc1617sfb_node;
	info->fix = uc1617sfb_fix;
	info->var = uc1617sfb_var;
	info->var.xres = par->width;
	info->var.xres_virtual = par->width;
	info->var.yres = par->height;
	info->var.yres_virtual = par->height;
	info->fix.line_length = (par->width * info->var.bits_per_pixel)/8;
	/* Set the FB parameter as per the BPP */
	info->var.red.length = 1;
	info->var.red.offset = 0;
	info->var.green.length = 1;
	info->var.green.offset = 0;
	info->var.blue.length = 1;
	info->var.blue.offset = 0;

	vmem_size = info->fix.line_length * par->height;
	/*vmem = devm_kzalloc(&spi->dev, vmem_size, GFP_KERNEL); */
	vmem = (u8 *)uc1617sfb_alloc_buffer(vmem_size);
	if (!vmem) {
		dev_err(&spi->dev, "Couldn't allocate graphical memory.\n");
		ret = -ENOMEM;
		goto fb_alloc_error;
	}
	/* Allocated memory to display screen buffer */
	info->screen_base = (u8 __iomem *)vmem;
	info->fix.smem_start = (unsigned long)vmem;
	info->fix.smem_len = vmem_size;

#ifdef CONFIG_FB_DEFERRED_IO
	info->fbdefio = &uc1617sfb_defio;
	fb_deferred_io_init(info);
#endif
	spi_set_drvdata(spi, info);
	/* init RESET, DCX gpio and regulators */
	ret = uc1617sfb_config_init(par);
	if (ret) {
		pr_err("Failed to configure GPIO or Regulator\n");
		goto gpio_fail_error;
	}

	ret = uc1617s_power_ctrl(par, 1);
	if (ret) {
		pr_err("Failed to Enable regulator\n");
		goto gpio_fail_error;
	}

	/* Register to Firmware */
	ret = spi_register_framebuffer(info);
	if (ret) {
		dev_err(&spi->dev, "Couldn't register the framebuffer\n");
		goto panel_init_error;
	}
#if defined(CONFIG_FB) && defined(AUTO_ENABLE_BKLIGHT)
	par->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&par->fb_notif);
	if (ret) {
		dev_err(&spi->dev,
			"Unable to register fb_notifier %d\n", ret);
		goto panel_init_error;
	}
#endif

	par->blank_buf = devm_kzalloc(&par->spi->dev, par->info->fix.smem_len,
					GFP_KERNEL);
	memset(par->blank_buf, 0x00, par->info->fix.smem_len);
	lpar = par;
#ifdef TEST_MODE
	ret = sysfs_create_file(&(spi->dev.kobj), &uc1617sfb_kobj_attr.attr);
	if (ret)
		dev_err(&spi->dev, "Failed to create sysfs node\n");
	ret = sysfs_create_file(&(spi->dev.kobj),
				&uc1617sfb_kobj_attr_led.attr);
#endif
	dev_info(&spi->dev,
	"SPI%d: %s fb device registered, using %d bytes of video memory\n",
	info->node, info->fix.id, vmem_size);
	return 0;

panel_init_error:
	if (par->ops->remove)
		par->ops->remove(par);
gpio_fail_error:
#ifdef CONFIG_FB_DEFERRED_IO
	fb_deferred_io_cleanup(info);
#endif
fb_alloc_error:
	framebuffer_release(info);
	return ret;
}

static int uc1617sfb_remove(struct spi_device *spi)
{
	struct fb_info *info = spi_get_drvdata(spi);
	struct uc1617sfb_par *par = info->par;

	unregister_framebuffer(info);
	if (par->ops->remove)
		par->ops->remove(par);
	fb_deferred_io_cleanup(info);
	framebuffer_release(info);
	return 0;
}

void uc1617sfb_shutdown(struct spi_device *spi)
{
	/* Clear the screen */
	struct fb_info *info = spi_get_drvdata(spi);
	struct uc1617sfb_par *par = info->par;

	uc1617sfb_coordinate_reset(par);
	gpio_set_value(par->datacmd, 1);
	spi_write(par->spi, par->blank_buf, par->info->fix.smem_len);
	uc1617sfb_write_dcx(par, CMD, 0xAC);
	uc1617s_power_ctrl(par, 0);
	if (par->bklt_ctrl == THIRD_PARTY)
		led_trigger_event(bl_led_trigger, 0);
	par->display_screen_state = OFF;
}

static struct spi_driver uc1617sfb_driver = {
	.probe = uc1617sfb_probe,
	.remove = uc1617sfb_remove,
	.shutdown = uc1617sfb_shutdown,
	.driver = {
		.name = "uc1617sfb",
		.of_match_table = of_match_ptr(uc1617sfb_of_match),
		.owner = THIS_MODULE,
	},
};

module_spi_driver(uc1617sfb_driver);

MODULE_DESCRIPTION("FB driver for the Sitronix ST7735S LCD controller");
MODULE_LICENSE("Dual BSD/GPL");
