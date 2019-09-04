/*Presure values Reading Register*/
#define PRS_B2 0x00
#define PRS_B1 0x01
#define PRS_B0 0x02

/*init Registers*/
#define FIFO_STS 0x0B
#define SET_PRES 0x08
#define CHIPID 0x0D

/*values to Sensors*/
#define READ_PRES 0x05
#define READ_PRES_DIS 0x00
/*coniguration*/
#define SPL06_BARO_MIN_POLL_INTERVAL_MS  8
#define SPL06_BARO_MAX_POLL_INTERVAL_MS  1000
#define SPL06_BARO_DEFAULT_POLL_INTERVAL_MS  250

/*hr timer*/
#define MS_TO_NS(x) (x * 1E6L)
#define NSEC_PER_MSEC   1000000L
#define ABS_PR		ABS_PRESSURE


