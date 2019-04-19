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

/* #define DISPLAY_DEBUG */
#define TEST_MODE
/* #define SUPPORT_PINMUX */

#define CMD	1
#define DATA	2

#define SPI_IO(num)     _IO('O', num)
#define DISPLAY_OFF     SPI_IO(1)
#define DISPLAY_ON      SPI_IO(2)
#define SNOOZE_OFF      SPI_IO(3)
#define SNOOZE_ON       SPI_IO(4)
#define DISPLAY_COMMIT  SPI_IO(5)

#ifdef SUPPORT_PINMUX
#define PINCTRL_STATE_ACTIVE	"spi_default"
#define PINCTRL_STATE_SUSPEND	"spi_sleep"
#endif

struct st7735sfb_par;
struct st7735sfb_ops {
	int (*init)(struct st7735sfb_par *);
	int (*remove)(struct st7735sfb_par *);
};

struct st7735sfb_par {
	struct spi_device *spi;
	struct fb_info *info;
	struct st7735sfb_ops *ops;
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
#ifdef SUPPORT_PINMUX
	struct pinctrl *st7735s_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
#endif
};

#ifdef TEST_MODE
struct st7735sfb_par *gpar = NULL;
#endif

static struct fb_var_screeninfo st7735sfb_var = {
	.bits_per_pixel = 16,
};

static struct fb_fix_screeninfo st7735sfb_fix = {
	.id	     = "ST7735S",
	.type	   = FB_TYPE_PACKED_PIXELS,
	.visual	 = FB_VISUAL_MONO10,
	.xpanstep       = 0,
	.ypanstep       = 0,
	.ywrapstep      = 0,
	.accel	  = FB_ACCEL_NONE,
};

static inline int st7735sfb_write_dcx(struct st7735sfb_par *par,
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

static void st7735sfb_coordinate_reset(struct st7735sfb_par *par)
{
	u16 x1 = 0;
	u16 y1 = 0;
	u16 x2 = par->width;
	u16 y2 = par->height;

	st7735sfb_write_dcx(par, CMD, 0x2c);

	/*     program x begin and end */
	st7735sfb_write_dcx(par, CMD, 0x2a);
	st7735sfb_write_dcx(par, DATA, ((x1&0xFF00)>>8));
	st7735sfb_write_dcx(par, DATA, (x1&0xFF));
	st7735sfb_write_dcx(par, DATA, ((x2&0xFF00)>>8));
	st7735sfb_write_dcx(par, DATA, (x2&0xFF));

	/*     program y begin and end */
	st7735sfb_write_dcx(par, CMD, 0x2b);
	st7735sfb_write_dcx(par, DATA, ((y1&0xFF00)>>8));
	st7735sfb_write_dcx(par, DATA, (y1&0xFF));
	st7735sfb_write_dcx(par, DATA, ((y2&0xFF00)>>8));
	st7735sfb_write_dcx(par, DATA, (y2&0xFF));
	st7735sfb_write_dcx(par, CMD, 0x2c);

}
static int st7735sfb_update_display(struct st7735sfb_par *par)
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
	/* Reset the co-ordinate to biggening of display with command 0x2c */
	st7735sfb_write_dcx(par, CMD, 0x2c);
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
	gpio_set_value(par->datacmd, 1);
	spi_write(par->spi, wr_data, par->info->fix.smem_len);
	kfree(wr_data);

	return 0;
}


static ssize_t st7735sfb_write(struct fb_info *info,
				const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct st7735sfb_par *par = info->par;
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
	if (st7735sfb_update_display(par))
		pr_err("Failed to transfer data\n");
	*ppos += count;
	return count;
}

static void st7735sfb_fillrect(struct fb_info *info,
				const struct fb_fillrect *rect)
{
	struct st7735sfb_par *par = info->par;

	sys_fillrect(info, rect);
	st7735sfb_update_display(par);
}

static void st7735sfb_copyarea(struct fb_info *info,
				const struct fb_copyarea *area)
{
	struct st7735sfb_par *par = info->par;

	sys_copyarea(info, area);
	st7735sfb_update_display(par);
}

static void st7735sfb_imageblit(struct fb_info *info,
				const struct fb_image *image)
{
	struct st7735sfb_par *par = info->par;

	sys_imageblit(info, image);
	st7735sfb_update_display(par);
}

static int st7735sfb_mmap(struct fb_info *info,
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

static int st7735sfb_ioctl(struct fb_info *info,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct st7735sfb_par *par;

	if (info == NULL)
		return -EINVAL;

	par = info->par;

	switch (cmd) {
	case DISPLAY_OFF:
		break;
	case DISPLAY_ON:
		break;
	case DISPLAY_COMMIT:
		st7735sfb_update_display(par);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static struct fb_ops st7735sfb_node = {
	.owner	  = THIS_MODULE,
	.fb_read	= fb_sys_read,
	.fb_write       = st7735sfb_write,
	.fb_fillrect    = st7735sfb_fillrect,
	.fb_copyarea    = st7735sfb_copyarea,
	.fb_imageblit   = st7735sfb_imageblit,
	.fb_mmap	= st7735sfb_mmap,
	.fb_ioctl       = st7735sfb_ioctl,
};

#ifdef CONFIG_FB_DEFERRED_IO
static void st7735sfb_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
	st7735sfb_update_display(info->par);
}

static struct fb_deferred_io st7735sfb_defio = {
	.delay	  = HZ,
	.deferred_io    = st7735sfb_deferred_io,
};
#endif

static int st7735s_power_ctrl(struct st7735sfb_par *par,
				int enable_state)
{
	int rc = 0;

	if (enable_state) {
#ifdef SUPPORT_PINMUX
		rc = pinctrl_select_state(par->st7735s_pinctrl,
				par->pinctrl_state_active);
		if (rc < 0)
			pr_err("failed to select pin to active state %d", rc);
#endif
		if (!par->power_state) {
			if (par->avdd_vreg) {
				rc = regulator_enable(par->avdd_vreg);
				pr_err("ST7735 enable avdd pass, rc=%d\n", rc);
				if (rc) {
					pr_err("ST7735 enable avdd failed, \
						rc=%d\n", rc);
					return -ENODEV;
				}
			}
			if (par->vdd_vreg) {
				rc = regulator_enable(par->vdd_vreg);
				pr_err("ST7735 enable vdd pass, rc=%d\n", rc);
				if (rc) {
					pr_err("ST7735 enable vdd failed, \
						rc=%d\n", rc);
					return -ENODEV;
				}
			}


			par->power_state = true;
		}
		/* Delay is required to stable the power sequence */
		msleep(100);
			/* Reset LCD panel */
		gpio_set_value(par->reset, 1);
		msleep(20);
		gpio_set_value(par->reset, 0);
		msleep(20);
		gpio_set_value(par->reset, 1);
		msleep(140);
		/* After reset Slave device init the display */
		if (par->ops->init)
			par->ops->init(par);
	} else if (!enable_state) {
		if (par->ops->remove)
			par->ops->remove(par);
		if (par->power_state) {
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
static int st7735sfb_st7735s_remove(struct st7735sfb_par *par)
{
	return 0;
}

static int st7735sfb_st7735s_init(struct st7735sfb_par *par)
{
	u8 *blank_buf;

	blank_buf = devm_kzalloc(&par->spi->dev, 18432, GFP_KERNEL);
	memset(blank_buf, 0x00, 18432);

	gpio_set_value(par->datacmd, 0);

	st7735sfb_write_dcx(par, CMD, 0xb1);
	st7735sfb_write_dcx(par, DATA, 0x01);
	st7735sfb_write_dcx(par, DATA, 0x08);
	st7735sfb_write_dcx(par, DATA, 0x05);

	st7735sfb_write_dcx(par, CMD, 0xb2);
	st7735sfb_write_dcx(par, DATA, 0x02);
	st7735sfb_write_dcx(par, DATA, 0x23);
	st7735sfb_write_dcx(par, DATA, 0x22);

	st7735sfb_write_dcx(par, CMD, 0xb3);
	st7735sfb_write_dcx(par, DATA, 0x02);
	st7735sfb_write_dcx(par, DATA, 0x23);
	st7735sfb_write_dcx(par, DATA, 0x22);
	st7735sfb_write_dcx(par, DATA, 0x02);
	st7735sfb_write_dcx(par, DATA, 0x23);
	st7735sfb_write_dcx(par, DATA, 0x22);

	st7735sfb_write_dcx(par, CMD, 0xB4);
	st7735sfb_write_dcx(par, DATA, 0x03);

	st7735sfb_write_dcx(par, CMD, 0xC0);
	st7735sfb_write_dcx(par, DATA, 0x82);
	st7735sfb_write_dcx(par, DATA, 0x02);
	st7735sfb_write_dcx(par, DATA, 0x84);

	st7735sfb_write_dcx(par, CMD, 0xC2);
	st7735sfb_write_dcx(par, DATA, 0x0A);
	st7735sfb_write_dcx(par, DATA, 0x00);

	st7735sfb_write_dcx(par, CMD, 0xC3);
	st7735sfb_write_dcx(par, DATA, 0x8A);
	st7735sfb_write_dcx(par, DATA, 0x2E);

	st7735sfb_write_dcx(par, CMD, 0xC4);
	st7735sfb_write_dcx(par, DATA, 0x8A);
	st7735sfb_write_dcx(par, DATA, 0xAA);

	st7735sfb_write_dcx(par, CMD, 0xC5);
	st7735sfb_write_dcx(par, DATA, 0x0B);

	st7735sfb_write_dcx(par, CMD, 0x3A);
	st7735sfb_write_dcx(par, DATA, 0x05);

	st7735sfb_write_dcx(par, CMD, 0x36);
	st7735sfb_write_dcx(par, DATA, 0xC8);

	st7735sfb_write_dcx(par, CMD, 0xe0);
	st7735sfb_write_dcx(par, DATA, 0x03);
	st7735sfb_write_dcx(par, DATA, 0x15);
	st7735sfb_write_dcx(par, DATA, 0x0C);
	st7735sfb_write_dcx(par, DATA, 0x12);
	st7735sfb_write_dcx(par, DATA, 0x3A);
	st7735sfb_write_dcx(par, DATA, 0x32);
	st7735sfb_write_dcx(par, DATA, 0x2B);
	st7735sfb_write_dcx(par, DATA, 0x2D);
	st7735sfb_write_dcx(par, DATA, 0x2B);
	st7735sfb_write_dcx(par, DATA, 0x28);
	st7735sfb_write_dcx(par, DATA, 0x30);
	st7735sfb_write_dcx(par, DATA, 0x3C);
	st7735sfb_write_dcx(par, DATA, 0x00);
	st7735sfb_write_dcx(par, DATA, 0x02);
	st7735sfb_write_dcx(par, DATA, 0x03);
	st7735sfb_write_dcx(par, DATA, 0x10);

	st7735sfb_write_dcx(par, CMD, 0xe1);
	st7735sfb_write_dcx(par, DATA, 0x03);
	st7735sfb_write_dcx(par, DATA, 0x15);
	st7735sfb_write_dcx(par, DATA, 0x0C);
	st7735sfb_write_dcx(par, DATA, 0x12);
	st7735sfb_write_dcx(par, DATA, 0x27);
	st7735sfb_write_dcx(par, DATA, 0x25);
	st7735sfb_write_dcx(par, DATA, 0x21);
	st7735sfb_write_dcx(par, DATA, 0x28);
	st7735sfb_write_dcx(par, DATA, 0x28);
	st7735sfb_write_dcx(par, DATA, 0x27);
	st7735sfb_write_dcx(par, DATA, 0x30);
	st7735sfb_write_dcx(par, DATA, 0x3C);
	st7735sfb_write_dcx(par, DATA, 0x00);
	st7735sfb_write_dcx(par, DATA, 0x01);
	st7735sfb_write_dcx(par, DATA, 0x02);
	st7735sfb_write_dcx(par, DATA, 0x10);

	st7735sfb_write_dcx(par, CMD, 0x11);
	msleep(120);
	/* Configure the display co-ordinate */
	st7735sfb_coordinate_reset(par);
	/* Write Blank buffer to keep screen clean */
	gpio_set_value(par->datacmd, 1);
	spi_write(par->spi, blank_buf, 18432);

	st7735sfb_write_dcx(par, CMD, 0x29);
	msleep(120);
	return 0;
}

static struct st7735sfb_ops st7735sfb_st7735s_ops = {
	.init   = st7735sfb_st7735s_init,
	.remove = st7735sfb_st7735s_remove,
};

static const struct of_device_id st7735sfb_of_match[] = {
	{
		.compatible = "sitronix,st7735sfb-spi",
		.data = (void *)&st7735sfb_st7735s_ops,
	},
	{},
};

MODULE_DEVICE_TABLE(of, st7735sfb_of_match);

#ifdef TEST_MODE
static ssize_t st7735sfb_display_color(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	u8 __iomem *dst;
	u8 *temp_buffer = NULL;

	 if (!gpar)
		return -ENOMEM;

	temp_buffer = kzalloc(gpar->info->fix.smem_len, GFP_KERNEL);
	if (temp_buffer == NULL)
		return -ENOMEM;
	memset(temp_buffer, 0x00, gpar->info->fix.smem_len);

	switch (buf[0]) {
	case 'r':
		pr_err("Display RED image\n");
		break;
	case 'g':
		pr_err("Display GREEN image\n");
		break;
	case 'b':
		pr_err("Display BLUE image\n");
		break;
	default:
		pr_err("Display all 0xff image\n");
		memset(temp_buffer, 0xFF, gpar->info->fix.smem_len);
	}

	dst = (void *) (gpar->info->screen_base);
	memcpy(dst, temp_buffer, gpar->info->fix.smem_len);

	if (st7735sfb_update_display(gpar))
		pr_err("Test mode data transfer failed\n");

	kfree(temp_buffer);
	return count;
}

static struct kobj_attribute st7735sfb_kobj_attr =
		__ATTR(display_color, 0600,
			NULL, st7735sfb_display_color);
#endif

static int st7735sfb_parse_dt(struct st7735sfb_par *par)
{
	int ret = 0;
	struct device_node *node = par->spi->dev.of_node;

	if (!node) {
		pr_err("No device tree data found!\n");
		return -EINVAL;
	}
	par->reset = of_get_named_gpio(node,
			"sitronix,reset-gpios", 0);
	if (!gpio_is_valid(par->reset))
		ret = -EINVAL;
	par->datacmd = of_get_named_gpio(node,
			"sitronix,datacmd-gpios", 0);
	if (!gpio_is_valid(par->datacmd))
		ret = -EINVAL;
	if (of_property_read_u32(node, "sitronix,width", &par->width))
		par->width = 96;
	if (of_property_read_u32(node, "sitronix,height", &par->height))
		par->width = 96;
	if (of_property_read_u32(node, "sitronix,page-offset",
					&par->page_offset))
		par->page_offset = 1;

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

static int st7735sfb_config_init(struct st7735sfb_par *par)
{
	int ret = 0;
	struct spi_device *spi = par->spi;

#ifdef SUPPORT_PINMUX
	/* Get pinctrl if target uses pinctrl */
	par->st7735s_pinctrl = devm_pinctrl_get(&(spi->dev));
	if (IS_ERR_OR_NULL(par->st7735s_pinctrl)) {
		ret = PTR_ERR(par->st7735s_pinctrl);
		dev_err(&spi->dev,
				"Target does not use pinctrl %d\n", ret);
		goto err_pinctrl_get;
	}

	par->pinctrl_state_active = pinctrl_lookup_state(par->st7735s_pinctrl,
							PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(par->pinctrl_state_active)) {
		ret = PTR_ERR(par->pinctrl_state_active);
		dev_err(&spi->dev,
				"Can not lookup %s pinstate %d\n",
				PINCTRL_STATE_ACTIVE, ret);
		goto err_pinctrl_lookup;
	}

	par->pinctrl_state_suspend = pinctrl_lookup_state(par->st7735s_pinctrl,
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
			pr_err("ST7735 vdd_vreg->set_voltage failed, rc=%d\n",
				ret);
			ret = -EINVAL;
		}
		ret = regulator_set_voltage(par->avdd_vreg, 2850000, 2850000);
		pr_err("ST7735 seting voltage2\n");
		if (ret) {
			pr_err("ST7735 avdd_vreg->set_voltage failed, rc=%d\n",
				ret);
			ret = -EINVAL;
		}
	}
	return ret;

#ifdef SUPPORT_PINMUX
err_pinctrl_lookup:
	devm_pinctrl_put(par->st7735s_pinctrl);
err_pinctrl_get:
	par->st7735s_pinctrl = NULL;
	return ret;
#endif
}

static int st7735sfb_probe(struct spi_device *spi)
{
	struct fb_info *info;
	struct st7735sfb_par *par;
	const struct of_device_id *of_match;
	u32 vmem_size;
	u8 *vmem;
	int ret;

	info = framebuffer_alloc(sizeof(struct st7735sfb_par), &spi->dev);
	if (!info) {
		dev_err(&spi->dev, "Couldn't allocate framebuffer.\n");
		return -ENOMEM;
	}
	par = info->par;
	par->info = info;
	par->spi = spi;
	par->power_state = false;
	of_match = of_match_device(st7735sfb_of_match, &spi->dev);
	if (!of_match) {
		ret = -EINVAL;
		goto fb_alloc_error;
	}
	par->ops = (struct st7735sfb_ops *)of_match->data;
	ret = st7735sfb_parse_dt(par);
	if (ret)
		goto fb_alloc_error;
	info->fbops = &st7735sfb_node;
	info->fix = st7735sfb_fix;
	info->var = st7735sfb_var;
	info->var.xres = par->width;
	info->var.xres_virtual = par->width;
	info->var.yres = par->height;
	info->var.yres_virtual = par->height;
	info->fix.line_length = (par->width * info->var.bits_per_pixel)/8;
	/* Set the FB parameter as per the BPP */
	if (info->var.bits_per_pixel == 16) {
		info->var.red.length = 5;
		info->var.red.offset = 11;
		info->var.green.length = 6;
		info->var.green.offset = 5;
		info->var.blue.length = 5;
		info->var.blue.offset = 0;
	} else {
		info->var.red.length = 1;
		info->var.red.offset = 0;
		info->var.green.length = 1;
		info->var.green.offset = 0;
		info->var.blue.length = 1;
		info->var.blue.offset = 0;
	}

	vmem_size = roundup((info->fix.line_length * par->height), PAGE_SIZE);
	vmem = devm_kzalloc(&spi->dev, vmem_size, GFP_KERNEL);
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
	info->fbdefio = &st7735sfb_defio;
	fb_deferred_io_init(info);
#endif
	spi_set_drvdata(spi, info);
	/* init RESET, DCX gpio and regulators */
	ret = st7735sfb_config_init(par);
	if (ret) {
		pr_err("Failed to configure GPIO or Regulator\n");
		goto gpio_fail_error;
	}

	ret = st7735s_power_ctrl(par, 1);
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

#ifdef TEST_MODE
	gpar = par;
	ret = sysfs_create_file(&(spi->dev.kobj), &st7735sfb_kobj_attr.attr);
	if (ret)
		dev_err(&spi->dev, "Failed to create sysfs node\n");
#endif
	dev_info(&spi->dev,
	"FB%d: %s fb device registered, using %d bytes of video memory\n",
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

static int st7735sfb_remove(struct spi_device *spi)
{
	struct fb_info *info = spi_get_drvdata(spi);
	struct st7735sfb_par *par = info->par;

	unregister_framebuffer(info);
	if (par->ops->remove)
		par->ops->remove(par);
	fb_deferred_io_cleanup(info);
	framebuffer_release(info);
	return 0;
}

static struct spi_driver st7735sfb_driver = {
	.probe = st7735sfb_probe,
	.remove = st7735sfb_remove,
	.driver = {
		.name = "st7735sfb",
		.of_match_table = of_match_ptr(st7735sfb_of_match),
		.owner = THIS_MODULE,
	},
};

module_spi_driver(st7735sfb_driver);

MODULE_DESCRIPTION("FB driver for the Sitronix ST7735S LCD controller");
MODULE_LICENSE("Dual BSD/GPL");
