
#ifndef _SX86XX_H_
#define _SX86XX_H_

/* User is able to use the older method for receiving
 * interrupts, or use the newer more preferred method
 * of threaded interrupts.
 */
//#define USE_THREADED_IRQ

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#define MAX_NUM_STATUS_BITS (8)

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif

typedef struct sx86XX sx86XX_t, *psx86XX_t;
struct sx86XX 
{
	void * bus; /* either i2c_client or spi_client */

	struct device *pdev; /* common device struction for linux */

	void *pDevice; /* device specific struct pointer */

	/* Function Pointers */
	int (*init)(psx86XX_t this); /* (re)initialize device */

	/* since we are trying to avoid knowing registers, create a pointer to a
	 * common read register which would be to read what the interrupt source
	 * is from 
	 */
	int (*refreshStatus)(psx86XX_t this); /* read register status */

	int (*get_nirq_low)(void); /* get whether nirq is low (platform data) */

	/* array of functions to call for corresponding status bit */
	void (*statusFunc[MAX_NUM_STATUS_BITS])(psx86XX_t this); 

#if defined(USE_THREADED_IRQ)
	struct mutex mutex;
#else  
	spinlock_t	      lock; /* Spin Lock used for nirq worker function */
#endif 
	int irq; /* irq number used */

	/* whether irq should be ignored.. cases if enable/disable irq is not used
	 * or does not work properly */
	char irq_disabled;

	u8 useIrqTimer; /* older models need irq timer for pen up cases */

	int irqTimeout; /* msecs only set if useIrqTimer is true */

	/* struct workqueue_struct	*ts_workq;  */  /* if want to use non default */
	struct delayed_work dworker; /* work struct for worker function */

	struct delayed_work resume_dworker; /* work struct for worker function */

#ifdef CONFIG_HAS_WAKELOCK
	struct early_suspend early_suspend;  /* early suspend data  */
#endif  

	int nirq_gpio;

	int read_flag;
	u8 read_reg;
};

void sx86XX_suspend(psx86XX_t this);
void sx86XX_resume(psx86XX_t this);
int sx86XX_init(psx86XX_t this);
int sx86XX_remove(psx86XX_t this);

#endif // _SX86XX_H_

