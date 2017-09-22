/*
 * Touchscreen common interface
 * add by Lenovo-sw wengjun1
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include "tpd_property.h"

static int tpd_suspend_status_flag = 0;
void synaptics_set_gesture_ctrl(unsigned int val);
int get_tpd_info(void);
int get_array_flag(void);
int get_glove_ctrl(void);
void set_glove_ctrl(unsigned int val);

void set_tpd_suspend_status(int mode)
{
	tpd_suspend_status_flag = mode;
	synaptics_set_gesture_ctrl(mode);
}
EXPORT_SYMBOL(set_tpd_suspend_status);

int get_tpd_suspend_status(void)
{
	return tpd_suspend_status_flag;
}
EXPORT_SYMBOL(get_tpd_suspend_status);

static ssize_t lenovo_tpd_suspend_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,"%d\n", get_tpd_suspend_status());
}

static ssize_t lenovo_tpd_suspend_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	int mode;
	int res = sscanf(buf, "%d", &mode);

	if (res != 1) {
		pr_err("%s: Expected 1 number\n", __func__);
		return -EINVAL;
	} else
		set_tpd_suspend_status(mode);

	return size;
}

static struct kobj_attribute lenovo_tpd_suspend_attr = {
	.attr = {
		.name = "tpd_suspend_status",
		.mode = S_IRUGO | S_IWUSR,
	},
	.show = &lenovo_tpd_suspend_show,
	.store = &lenovo_tpd_suspend_store,
};

static ssize_t lenovo_tpd_glove_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,"%d\n", get_glove_ctrl());
}

static ssize_t lenovo_tpd_glove_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t size)
{
	int mode;
	int res = sscanf(buf, "%d", &mode);

	if (res != 1) {
		pr_err("%s: Expected 1 number\n", __func__);
		return -EINVAL;
	} else
		set_glove_ctrl(mode);

	return size;
}

static struct kobj_attribute lenovo_tpd_glove_attr = {
	.attr = {
		.name = "tpd_glove_status",
		.mode = S_IRUGO | S_IWUSR,
	},
	.show = &lenovo_tpd_glove_show,
	.store = &lenovo_tpd_glove_store,
};

static ssize_t lenovo_tpd_info_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	ret = get_tpd_info();

	if (!have_correct_setting)
		return sprintf(buf, "Incorrect setting\n");
	else
		return sprintf(buf, "Product ID=%s; Build ID=%x; Config ID=0x%08x;\n",
				tpd_info_t->product_id, tpd_info_t->build_id,
				tpd_info_t->config_id);
}

static struct kobj_attribute lenovo_tpd_info_attr = {
	.attr = {
		.name = "lenovo_tpd_info",
		.mode = S_IRUGO,
	},
	.show = &lenovo_tpd_info_show,
};

static ssize_t lenovo_flag_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", get_array_flag());
}

static struct kobj_attribute lenovo_tpd_flag_attr = {
	.attr = {
		.name = "lenovo_flag",
		.mode = S_IRUGO,
	},
	.show = &lenovo_flag_show,
};

static struct attribute *tpd_properties_attrs[] = {
	&lenovo_tpd_suspend_attr.attr,
	&lenovo_tpd_glove_attr.attr,
	&lenovo_tpd_info_attr.attr,
	&lenovo_tpd_flag_attr.attr,
	NULL
};

static struct attribute_group tpd_properties_attr_group = {
	.attrs = tpd_properties_attrs,
};

struct kobject *properties_kobj;

void property_init(void)
{
	int ret = 0;

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
				&tpd_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("%s: Failed to create board_properties\n", __func__);
}
EXPORT_SYMBOL(property_init);
