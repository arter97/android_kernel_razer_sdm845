#ifndef __FIH_BATTERY_BBS_H
#define __FIH_BATTERY_BBS_H

#define BBS_LOG 1


/* Error Code: 11/Charger */
#define BBS_CHARGER_BATTERY_MISS()\
do { printk("BBox::UEC;11::2;Battery miss\n"); } while (0)
#define BBS_CHARGER_READ_FAILED()\
do { printk("BBox::UEC;11::3;Charger read failed\n"); } while (0)
#define BBS_CHARGER_WRITE_FAILED()\
do { printk("BBox::UEC;11::4;Charger write failed\n"); } while (0)
#define BBS_CHARGER_WEAK()\
do { printk("BBox::UEC;11::5;Charger weak\n"); } while (0)
#define BBS_CHARGER_OVP()\
do { printk("BBox::UEC;11::6;Charger OVP\n"); } while (0)
#define BBS_CHARGER_USBIN_UV()\
do { printk("BBox::UEC;11::7;USBIN_UV / Weak charger\n"); } while (0)

/* Error Code: 12/GasGauge */
#define BBS_FG_READ_FAILED()\
do { printk("BBox::UEC;12::2;FG read failed\n"); } while (0)
#define BBS_FG_WRITE_FAILED()\
do { printk("BBox::UEC;12::3;FG write failed\n"); } while (0)

/* Error Code: 49/Battery */
#define BBS_BATTERY_VOLTAGE_DROP()\
do { printk("BBox::UEC;49::0;Battery voltage drop (UVLO)\n"); } while (0)
#define BBS_BATTERY_REACH_SHUTDOWN_TEMPERATURE()\
do { printk("BBox::UEC;49::1;Battery reach shutdown temperature\n"); } while (0)
#define BBS_BATTERY_AGING()\
do { printk("BBox::UEC;49::2;Battery aging\n"); } while (0)
#define BBS_BATTERY_VOLTAGE_LOW()\
do { printk("BBox::UEC;49::3;Battery voltage low\n"); } while (0)
#define BBS_BATTERY_ID_MISS()\
do { printk("BBox::UEC;49::4;Battery ID miss/out-of-range\n"); } while (0)

/* Basic Info: 0/BATID */
#define BBS_BATTERY_ID(x)\
do { printk("BBox::UPD;0::%d\n", x); } while (0)

/* Basic Info: 49/BATT_FCC_MAX */
#define BBS_BATTERY_FCC_MAX(x)\
do { printk("BBox::UPD;49::%lld\n", x); } while (0)

/* Basic Info: 50/BATT_FCC */
#define BBS_BATTERY_FCC(x, y)\
do { printk("BBox::UPD;50::%d::%lld\n", x, y); } while (0)

/* Basic Info: 51/IN_CURR */
#define BBS_INPUT_CURRENT(x)\
do { printk("BBox::UPD;51::%d\n", x); } while (0)

/* Basic Info: 72/RST_GAUGE */
#define BBS_FG_RESET()\
do { printk("BBox::UPD;72::\n"); } while (0)


#endif /* __FIH_BATTERY_BBS_H */
