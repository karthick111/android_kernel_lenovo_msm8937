/*
  Tony Sun : Add for Get nv data from modem using SMEM.

  2014.11.19 Tony Sun, Modify for Android L 64-bits
    SMEM pointer psmem_nv->nv_bt used as memcpy source address will cause Alignment Fault.
	define struct smem_nv gsmem_nv to avoid this issue.
  2014.3.31	Tony Sun, Modify for new linux version. 
  	Replace create_proc_entry with proc_create.
  	using new smem_alloc()
  	Move file node to /msm_nv/
*/

#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <soc/qcom/smem.h>
//#include "smd_private.h"
//#include "include/mach/proc_comm.h"

#define PROC_ENTRY_NV	"msm_nv"
#define PROC_MAC_WIFI	"mac_wifi"
#define PROC_MAC_BT		"mac_bt"
#define PROC_MODEM_SN1     "lnvsn1"
#define PROC_MODEM_SN2     "lnvshowcode"
#define PROC_MODEM_IMEI1     "lnvimei1"
#define PROC_MODEM_IMEI2     "lnvimei2"
#define PROC_MODEM_MEID     "lnvmeid"
#define PROC_MODEM_HWID     "lnvhwid"
#define PROC_MODEM_STATION     "lnvstation"
#define PROC_MODEM_FLOW  "lnvflow"

#define NV_WIFI_ADDR_SIZE	6
#define NV_BT_ADDR_SIZE		6
/*[BEGIN]qiuff1 20140421 add for addr_size*/
#define NV_SN1_ADDR_SIZE    25
#define NV_SN2_ADDR_SIZE    25
#define NV_MEID_ADDR_SIZE    15
#define NV_IMEI1_ADDR_SIZE    15
#define NV_IMEI2_ADDR_SIZE    15
#define NV_HWID_ADDR_SIZE    10
#define NV_STATION_ADDR_SIZE    20
#define NV_FLOW_ADDR_SIZE    20
/*[END]qiuff1 20140421 add for addr_size*/

#define NV_MAX_SIZE		512
/* [BEGIN] guohh1 20131011 add for FACTORYDATACHECK */
#define NV_OTHERS_SIZE   (NV_MAX_SIZE - NV_WIFI_ADDR_SIZE - NV_BT_ADDR_SIZE-32-32-16-16-16 -16 - 32-32)
/* [END   ] guohh1 20131011 add for FACTORYDATACHECK*/

struct smem_nv {
	unsigned char nv_wifi[NV_WIFI_ADDR_SIZE];
	unsigned char nv_bt[NV_BT_ADDR_SIZE];
	/* [BEGIN] guohh1 20131011 add for FACTORYDATACHECK */
	unsigned char nv_sn1[32];
	unsigned char nv_sn2[32];
	unsigned char nv_meid[16];
	unsigned char nv_imei1[16];
	unsigned char nv_imei2[16];
	unsigned char nv_hwid[16];
	unsigned char nv_station[32];
	unsigned char nv_flow[32];
	/* [END   ] guohh1 20131011 add for FACTORYDATACHECK*/
	unsigned char nv_others[NV_OTHERS_SIZE];	
};

struct smem_nv gsmem_nv;
struct smem_nv * psmem_nv = NULL;

static struct smem_nv * smem_read_nv(void)
{
	struct smem_nv * buf;
	buf = smem_alloc(SMEM_ID_VENDOR_READ_NV, NV_MAX_SIZE, 0, SMEM_ANY_HOST_FLAG);
	if(!buf) 
		printk(KERN_ERR "SMEM_ID_VENDOR_READ_NV smem_alloc failed\n");

	gsmem_nv = *buf;	

	return &gsmem_nv;	
}

int wlan_get_nv_mac(char* buf)
{
	int ret = -1;
        if (!psmem_nv) {
		psmem_nv = smem_read_nv();
	}

	if (!psmem_nv){
		printk(KERN_ERR "Could not get smem for wlan mac nv\n");
	        return ret;
	}

	printk(KERN_ERR "wifi addr  = 0x %02x %02x %02x %02x %02x %02x\n",
		psmem_nv->nv_wifi[0],psmem_nv->nv_wifi[1],psmem_nv->nv_wifi[2],
		psmem_nv->nv_wifi[3],psmem_nv->nv_wifi[4],psmem_nv->nv_wifi[5]);
	memcpy( buf, psmem_nv->nv_wifi, NV_WIFI_ADDR_SIZE);
	return 0;
}
EXPORT_SYMBOL_GPL(wlan_get_nv_mac);

static int wifi_proc_read(struct seq_file *seq, void *offset)
{
	if (!psmem_nv) {
		psmem_nv = smem_read_nv();
	}
	
	if (!psmem_nv)  {
		printk(KERN_ERR "Read nv failed!\n");
		return 0;
	}

	printk(KERN_DEBUG "wifi addr  = 0x %02x %02x %02x %02x %02x %02x\n",
		psmem_nv->nv_wifi[0],psmem_nv->nv_wifi[1],psmem_nv->nv_wifi[2],
		psmem_nv->nv_wifi[3],psmem_nv->nv_wifi[4],psmem_nv->nv_wifi[5]);
	
	return seq_write(seq, psmem_nv->nv_wifi, NV_WIFI_ADDR_SIZE);
}

static int wifi_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, wifi_proc_read, NULL);
}

static const struct file_operations fops_wifi = {
	.owner	 = THIS_MODULE,
	.open	 = wifi_proc_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = seq_release
};


static int bt_proc_read(struct seq_file *seq, void *offset)
{
	if (!psmem_nv) {
		psmem_nv = smem_read_nv();
	}
	
	if (!psmem_nv)  {
		printk(KERN_ERR "Read nv failed!\n");
		return 0;
	}

	printk(KERN_DEBUG "bt addr  = 0x %02x %02x %02x %02x %02x %02x\n",
		psmem_nv->nv_bt[0],psmem_nv->nv_bt[1],psmem_nv->nv_bt[2],
		psmem_nv->nv_bt[3],psmem_nv->nv_bt[4],psmem_nv->nv_bt[5]);

	return seq_write(seq, psmem_nv->nv_bt, NV_BT_ADDR_SIZE);
}

static int bt_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bt_proc_read, NULL);
}

static const struct file_operations fops_bt = {
	.owner	 = THIS_MODULE,
	.open	 = bt_proc_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = seq_release
};

/*[BEGIN] qiuff1  20140414 add for datacheck*/
static int sn1_proc_read(struct seq_file *seq, void *offset)
{
	if (!psmem_nv) {
		psmem_nv = smem_read_nv();
	}
	
	if (!psmem_nv)  {
		printk(KERN_ERR "Read nv failed!\n");
		return 0;
	}
	return seq_write(seq, psmem_nv->nv_sn1, NV_SN1_ADDR_SIZE);
}
static int sn2_proc_read(struct seq_file *seq, void *offset)
{
	if (!psmem_nv) {
		psmem_nv = smem_read_nv();
	}
	
	if (!psmem_nv)  {
		printk(KERN_ERR "Read nv failed!\n");
		return 0;
	}

	return seq_write(seq, psmem_nv->nv_sn2, NV_SN2_ADDR_SIZE);
}
static int meid_proc_read(struct seq_file *seq, void *offset)
{
	if (!psmem_nv) {
		psmem_nv = smem_read_nv();
	}
	
	if (!psmem_nv)  {
		printk(KERN_ERR "Read nv failed!\n");
		return 0;
	}
	return seq_write(seq, psmem_nv->nv_meid, NV_MEID_ADDR_SIZE);
}
static int imei1_proc_read(struct seq_file *seq, void *offset)
{
	if (!psmem_nv) {
		psmem_nv = smem_read_nv();
	}
	
	if (!psmem_nv)  {
		printk(KERN_ERR "Read nv failed!\n");
		return 0;
	}

	return seq_write(seq, psmem_nv->nv_imei1, NV_IMEI1_ADDR_SIZE);
}
static int imei2_proc_read(struct seq_file *seq, void *offset)
{
	if (!psmem_nv) {
		psmem_nv = smem_read_nv();
	}
	
	if (!psmem_nv)  {
		printk(KERN_ERR "Read nv failed!\n");
		return 0;
	}
	return seq_write(seq, psmem_nv->nv_imei2, NV_IMEI2_ADDR_SIZE);
}
static int hwid_proc_read(struct seq_file *seq, void *offset)
{
	if (!psmem_nv) {
		psmem_nv = smem_read_nv();
	}
	
	if (!psmem_nv)  {
		printk(KERN_ERR "Read nv failed!\n");
		return 0;
	}
	return seq_write(seq, psmem_nv->nv_hwid, NV_HWID_ADDR_SIZE);
}
static int station_proc_read(struct seq_file *seq, void *offset)
{
	if (!psmem_nv) {
		psmem_nv = smem_read_nv();
	}
	
	if (!psmem_nv)  {
		printk(KERN_ERR "Read nv failed!\n");
		return 0;
	}
	return seq_write(seq, psmem_nv->nv_station, NV_STATION_ADDR_SIZE);
}
static int flow_proc_read(struct seq_file *seq, void *offset)
{
	if (!psmem_nv) {
		psmem_nv = smem_read_nv();
	}
	
	if (!psmem_nv)  {
		printk(KERN_ERR "Read nv failed!\n");
		return 0;
	}
	return seq_write(seq, psmem_nv->nv_flow, NV_FLOW_ADDR_SIZE);
}
static int sn1_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sn1_proc_read, NULL);
}
static int sn2_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sn2_proc_read, NULL);
}
static int meid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, meid_proc_read, NULL);
}
static int imei1_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, imei1_proc_read, NULL);
}
static int imei2_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, imei2_proc_read, NULL);
}
static int hwid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, hwid_proc_read, NULL);
}
static int station_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, station_proc_read, NULL);
}
static int flow_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, flow_proc_read, NULL);
}
static const struct file_operations fops_sn1 = {
	.owner	 = THIS_MODULE,
	.open	 = sn1_proc_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = seq_release
};
static const struct file_operations fops_sn2 = {
	.owner	 = THIS_MODULE,
	.open	 = sn2_proc_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = seq_release
};
static const struct file_operations fops_meid = {
	.owner	 = THIS_MODULE,
	.open	 = meid_proc_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = seq_release
};
static const struct file_operations fops_imei1 = {
	.owner	 = THIS_MODULE,
	.open	 = imei1_proc_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = seq_release
};
static const struct file_operations fops_imei2 = {
	.owner	 = THIS_MODULE,
	.open	 = imei2_proc_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = seq_release
};
static const struct file_operations fops_hwid= {
	.owner	 = THIS_MODULE,
	.open	 = hwid_proc_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = seq_release
};
static const struct file_operations fops_station= {
	.owner	 = THIS_MODULE,
	.open	 = station_proc_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = seq_release
};
static const struct file_operations fops_flow= {
	.owner	 = THIS_MODULE,
	.open	 = flow_proc_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = seq_release
};

/*[END] qiuff1  20140414 add for datacheck*/


static void show_nv(void)
{
	struct proc_dir_entry *proc_parent;
	struct proc_dir_entry *wifi_addr_entry;
	struct proc_dir_entry *bt_addr_entry;
	/* [BEGIN] guohh1 20131011 add for FACTORYDATACHECK */
	struct proc_dir_entry *sn1_addr_entry;
	struct proc_dir_entry *sn2_addr_entry;
	struct proc_dir_entry *meid_addr_entry;
	struct proc_dir_entry *imei1_addr_entry;
	struct proc_dir_entry *imei2_addr_entry;
	struct proc_dir_entry *hwid_addr_entry;
	struct proc_dir_entry *station_addr_entry;
	/* [END   ] guohh1 20131011 add for FACTORYDATACHECK*/
	/*[BEGIN]qiuff1 20140411  add for station information*/
	struct proc_dir_entry *flow_addr_entry;  
	/*[END]qiuff1 20140411  add for station information*/ 

	proc_parent = proc_mkdir(PROC_ENTRY_NV, NULL);
	if (!proc_parent) {
		printk(KERN_ERR "Create proc %s using for smem nv faild!\n",PROC_ENTRY_NV );
		return;
	}
	
	wifi_addr_entry = proc_create(PROC_MAC_WIFI, 0, proc_parent, &fops_wifi);
	if (!wifi_addr_entry) {
		printk(KERN_ERR "Create proc %s/%s using for smem nv faild!\n",PROC_ENTRY_NV, PROC_MAC_WIFI);
	}
	
	bt_addr_entry = proc_create(PROC_MAC_BT, 0, proc_parent, &fops_bt);
	if (!bt_addr_entry) {
		printk(KERN_ERR "Create proc %s/%s using for smem nv faild!\n",PROC_ENTRY_NV, PROC_MAC_BT);
	}
	

 	/* [BEGIN] qiuff1 20140414 modify for datacheck */
	sn1_addr_entry = proc_create(PROC_MODEM_SN1, 0, proc_parent, &fops_sn1);
	if (!sn1_addr_entry) {
		printk(KERN_ERR "Create proc %s/%s using for smem nv faild!\n",PROC_ENTRY_NV, PROC_MODEM_SN1);
	}
	sn2_addr_entry = proc_create(PROC_MODEM_SN2, 0, proc_parent, &fops_sn2);
	if (!sn2_addr_entry) {
		printk(KERN_ERR "Create proc %s/%s using for smem nv faild!\n",PROC_ENTRY_NV, PROC_MODEM_SN2);
	}
	meid_addr_entry = proc_create(PROC_MODEM_MEID, 0, proc_parent, &fops_meid);
	if (! meid_addr_entry) {
		printk(KERN_ERR "Create proc %s/%s using for smem nv faild!\n",PROC_ENTRY_NV, PROC_MODEM_MEID);
	}
	imei1_addr_entry = proc_create(PROC_MODEM_IMEI1, 0, proc_parent, &fops_imei1);
	if (!imei1_addr_entry) {
		printk(KERN_ERR "Create proc %s/%s using for smem nv faild!\n",PROC_ENTRY_NV,PROC_MODEM_IMEI1);
	}
	imei2_addr_entry = proc_create(PROC_MODEM_IMEI2, 0, proc_parent, &fops_imei2);
	if (!imei2_addr_entry) {
		printk(KERN_ERR "Create proc %s/%s using for smem nv faild!\n",PROC_ENTRY_NV, PROC_MODEM_IMEI2);
	}
	hwid_addr_entry = proc_create(PROC_MODEM_HWID, 0, proc_parent, &fops_hwid);
	if (!hwid_addr_entry) {
		printk(KERN_ERR "Create proc %s/%s using for smem nv faild!\n",PROC_ENTRY_NV, PROC_MODEM_HWID);
	}
	station_addr_entry = proc_create(PROC_MODEM_STATION, 0,  proc_parent, &fops_station);
	if (!station_addr_entry) {
		printk(KERN_ERR "Create proc %s/%s using for smem nv faild!\n",PROC_ENTRY_NV, PROC_MODEM_STATION);
	}
	flow_addr_entry = proc_create(PROC_MODEM_FLOW, 0,  proc_parent, &fops_flow);
	if (!flow_addr_entry) {
		printk(KERN_ERR "Create proc %s/%s using for smem nv faild!\n",PROC_ENTRY_NV, PROC_MODEM_FLOW);
	}
      /* [END] qiuff1 20140414 add for FACTORYDATACHECK*/

}

int __init lephone_nv_init(void)
{
	show_nv();
	return 0;
}

//run lephone_nv_init in late_initcall
late_initcall(lephone_nv_init);
