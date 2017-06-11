
#ifndef __LOW_STORAGE_H
#define __LOW_STORAGE_H

#define LIMIT_DATA_SIZE
#define LIMIT_SDCARD_SIZE
#ifdef LIMIT_SDCARD_SIZE           // for  fuse over data, to prevent the sdcard make data to full
#define DATA_FREE_SIZE_TH_DEFAULT (50*1024*1024)  // 50MB fuse free for data
#endif
extern long long data_free_size_th;
extern long long data_free_page;
#define CHECK_1TH  (10 * 1024 * 1024)
#define CHECK_2TH  (2 * 1024 * 1024)
extern long long store;
//EXPORT_SYMBOL(data_free_size_th);
extern dev_t fuse_data_dev;
#endif

