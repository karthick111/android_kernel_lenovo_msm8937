/*
 * Touchscreen common interface
 * add by Lenovo-sw wengjun1
 */

#ifndef _TPD_PROPERTY_H_
#define _TPD_PROPERTY_H_

struct tpd_version_info {
	char *product_id;
	unsigned int build_id;
	unsigned int config_id;
	unsigned int cfg_version;
};
extern struct tpd_version_info *tpd_info_t;
extern unsigned int have_correct_setting;

#endif
