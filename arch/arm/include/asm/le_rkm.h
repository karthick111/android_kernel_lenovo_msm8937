/*
filename: le_rkm.h

this is the lenovo replay kernel message implement.

By RKM,the offline log in andorid can backup previous system logs,
such as kernel dmesg log, tz logs; it also can access current
system lifetime's boot logs,such as sbl logs, lk logs, even tz logs.

In kernel mode, the RKM can get the lk debug log message
In ramdump mode, the RKM can get the lastkmsg info

History:
    Apr, 2014  KerryXi  Initial the rkm feature,save lastkmsg in ramdump
    Nov, 2014  KerryXi  Extend the kernel log buffer structure member
    Mar, 2015  KerryXi  Implement dynamic ddr start address that is board
                        Kernel address
    Apr, 2016  KerryXi  Platform dependent memory address for parsed
                        lastkmsg text

Copyright Lenovo 2014-2016
*/

#ifndef _LE_RKM_H_
#define _LE_RKM_H_

#ifdef CONFIG_LENOVO_DEBUG_RKM
/*rkm backup message layout, page unit
offset page 0: the rkm overall message area, include version info,kernel
		banner,etc. reserve 1 pages
offset page 1: current lk buf log information,reserve two pages,8k
offset page 3: last system kernel dmesg log.due to the dmesglog length maybe
		not fix, so it should stay tail. reserve 512k
*/
#define RKM_OVERVIEW_LOG_OFFSET        0
#define RKM_LK_BUF_LOG_OFFSET          1
#define RKM_LAST_KERNEL_LOG_BUF_OFFSET 3

//previous system kernel log_buf message
#define RKM_LOG_BUF_HEADER_TABLE_MAGIC1 0x686d6b72
#define RKM_LOG_BUF_HEADER_TABLE_MAGIC2 0x7461626c
#define RKM_LOG_BUF_HEADER_TABLE_VERSION 1

//uint is page size
#define MAX_RKM_LOG_BUF_LEN 512
#define MAX_RKM_LK_BUF_LEN 2

#define PHYSIC_MEM_BASE_ADDR			PHYS_OFFSET
#define RKM_SCAN_KERNEL_LOG_BUF_START_OFFSET 	(PHYSIC_MEM_BASE_ADDR + 0x1200000)
#define RKM_SCAN_KERNEL_LOG_BUF_END 		(PHYSIC_MEM_BASE_ADDR + 0x2300000)

typedef struct {
	unsigned int magic1;    //0x686d6b72  "rkmh"
	unsigned int magic2;    //0x7461626c  "lbat"
	unsigned int version;
	unsigned int bss_start;
	unsigned int bss_stop;
	unsigned int log_buf_pa;
	unsigned int log_buf_len;
	unsigned int log_first_idx_pa;
	unsigned int log_next_idx_pa;
	unsigned int log_struct_size;
} rkm_log_buf_header_table_t;

void rkm_init_log_buf_header(char *addr,int len,char *first_idx_pa,char *last_idx_pa,int log_struct_size);

void arm_rkm_log_backup(void);

int early_init_dt_scan_boot_log(unsigned long node, const char *uname,
				     int depth, void *data);

int kernel_log_buf_text_parser(char *kernel_log_buf, char *text_buf, int size);
#endif
#endif  //_LE_RKM_H_
