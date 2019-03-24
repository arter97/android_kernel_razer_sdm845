#ifndef P9221_WIRELESS_CHARGER_H
#define P9221_WIRELESS_CHARGER_H

#define pr_fmt(fmt) "[wlc] %s|%d: " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/iio/consumer.h>


/* ========================================================================== */
/* ========================================================================== */
/* print */
enum debug_flag {
	PR_ENTRY	= BIT(0),
	PR_DC		= BIT(1),
	PR_TEMP		= BIT(2),
	PR_I2C		= BIT(3),
	PR_PROP		= BIT(4),
};

static int debug_mask;
module_param_named(debug_mask, debug_mask, int, 0600);

#define pr_dbg(flag, fmt, ...)							\
do {										\
	if (debug_mask & flag) {						\
		pr_info(fmt, ##__VA_ARGS__);					\
	} else {								\
		pr_debug(fmt, ##__VA_ARGS__);					\
	}									\
} while (0)


/* ========================================================================== */
/* ========================================================================== */
/* device tree */
static const char *NAME_GPIO_EN			= "p9221,gpio-en";
static const char *NAME_GPIO_INT		= "p9221,gpio-int";
static const char *NAME_FOD_BPP			= "p9221,fod-bpp";
static const char *NAME_FOD_EPP			= "p9221,fod-epp";
static const char *NAME_TEMP_CONTROL_ENABLE	= "p9221,temp-control-enable";
static const char *NAME_TEMP_RANGES		= "p9221,temp-ranges";
static const char *NAME_TEMP_HYSTERESIS		= "p9221,temp-hysteresis";
static const char *NAME_TEMP_CHAN		= "skin_temp";


/* ========================================================================== */
/* ========================================================================== */
/* Interrupt Status Register */
enum int_status {
	BIT_OVER_CURRENT		= BIT(0),
	BIT_OVER_VOLTAGE		= BIT(1),
	BIT_OVER_TEMPERATURE		= BIT(2),
	BIT_VOLTAGE_CHANGED		= BIT(7),
};

/* End Power Transfer */
enum EPT {
	BIT_EPT_Code_Charge_Complete		= BIT(0),
};

/* Command Register */
enum cmd {
	BIT_TOGGLE_LDO_ON_OFF		= BIT(1),
	BIT_SEND_END_POWER_TRANSFER	= BIT(3),
	BIT_SEND_BATTERY_CHARGE_STATUS	= BIT(4),
	BIT_CLEAR_INTERRUPT		= BIT(5),
};

/* i2c */
enum i2c_xfer_type {
	I2C_READ = 0,
	I2C_WRITE,
};

enum i2c_data_size {
	I2C_DATA_BYTE = 1,
	I2C_DATA_WORD,
};

enum {NUM_BYTE_ADDR = 2};	/* two-byte address */
enum {MAX_BYTE_DATA = 2};	/* max: two-byte data */

/* Power Profile */
enum pp_type {
	NPP = 0,	/* No Power Profile */
	BPP,		/* Baseline Power Profile */
	EPP,		/* Extended Power Profile */
};

enum {PP_THRESHOLD_UV = 6500000};

/* voltage */
enum voltage_uv {
	UV_5V	= 5000000,
	UV_9V	= 9000000,
	UV_12V	= 12000000,
};

/* fod */
enum {NUM_FOD_PARM = 12};

/* temp */
enum {MAX_TEMP_RANGES = 5};


/* bad cycle detect */
enum wlc_state {
	WLC_ATTACHED = 0,
	WLC_DETACHED,
};

enum {MAX_BAD_CYCLE_COUNT = 2};
enum {MIN_GOOD_CYCLE_TIME_MS = 3000};


/* ========================================================================== */
/* ========================================================================== */
/* temp */
struct temp_range {
	int temp_low;
	int temp_high;
	int bpp_uv;
	int bpp_ua;
	int epp_uv;
	int epp_ua;
};

struct temp_control {
	bool			enable;
	int			index;
	struct temp_range	range[MAX_TEMP_RANGES];
	int			hysteresis;
};

/* chip */
struct p9221_chip {
	struct i2c_client	*client;
	struct device		*dev;

	/* dtsi */
	int			gpio_en;		/* Enable pin */
	int			gpio_int;		/* Interrupt pin */
	u8			fod_bpp[NUM_FOD_PARM];
	u8			fod_epp[NUM_FOD_PARM];

	/* locks */
	struct mutex		read_write_lock;

	/* power supplies */
	struct power_supply	*psy;
	struct power_supply	*dc_psy;
	struct power_supply	*bms_psy;
	struct power_supply	*batt_psy;

	/* notifiers */
	struct notifier_block	nb;

	/* work */
	struct work_struct	p9221_change_work;
	struct work_struct	p9221_End_Power_Transfer_work;
	struct delayed_work	p9221_fod_work;
	struct delayed_work	p9221_temp_work;

	/* power profile */
	enum pp_type		power_profile;
	bool			is_epp_supported;

	/* temperature */
	struct temp_control	*temp_ctl;
	struct iio_channel	*temp_chan;

	/* bad cycle detect */
	int			bad_cycle_count;
	bool			output_off;
};

/* reg */
struct p9221_reg {
	u8 addr;
	u8 size;
};


/* ========================================================================== */
/* ========================================================================== */
/* reg */
static struct p9221_reg reg_chip_id			= {0x00, I2C_DATA_WORD};
static struct p9221_reg reg_chip_rev			= {0x02, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_customer_id			= {0x03, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_fw_major_rev		= {0x04, I2C_DATA_WORD};
static struct p9221_reg reg_fw_minor_rev		= {0x06, I2C_DATA_WORD};
//static struct p9221_reg reg_sram_fw_major_rev		= {0x1C, I2C_DATA_WORD}; //Hidden
//static struct p9221_reg reg_sram_fw_minor_rev		= {0x1E, I2C_DATA_WORD}; //Hidden
static struct p9221_reg reg_status			= {0x34, I2C_DATA_WORD};
static struct p9221_reg reg_int_status			= {0x36, I2C_DATA_WORD};
static struct p9221_reg reg_int_enable			= {0x38, I2C_DATA_WORD};
static struct p9221_reg reg_battery_charge_status		= {0x3A, I2C_DATA_BYTE};
static struct p9221_reg reg_ept_code			= {0x3B, I2C_DATA_BYTE};
static struct p9221_reg reg_adc_vout			= {0x3C, I2C_DATA_WORD};
static struct p9221_reg reg_vout_set			= {0x3E, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_vrect_adj			= {0x3F, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_adc_vrect			= {0x40, I2C_DATA_WORD};
static struct p9221_reg reg_rx_iout			= {0x44, I2C_DATA_WORD};
static struct p9221_reg reg_adc_die_temp		= {0x46, I2C_DATA_WORD};
static struct p9221_reg reg_op_freq			= {0x48, I2C_DATA_WORD};
static struct p9221_reg reg_ilim_set			= {0x4A, I2C_DATA_BYTE}; //Hidden
//static struct p9221_reg reg_align_x			= {0x4B, I2C_DATA_BYTE};
//static struct p9221_reg reg_align_y			= {0x4C, I2C_DATA_BYTE};
static struct p9221_reg reg_sys_op_mode			= {0x4D, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_command			= {0x4E, I2C_DATA_BYTE};
//static struct p9221_reg reg_ppp_header		= {0x50, I2C_DATA_BYTE}; //Hidden
//static struct p9221_reg reg_ppp_byte_1		= {0x51, I2C_DATA_BYTE}; //Hidden
//static struct p9221_reg reg_ppp_byte_2		= {0x52, I2C_DATA_BYTE}; //Hidden
//static struct p9221_reg reg_ppp_byte_3		= {0x53, I2C_DATA_BYTE}; //Hidden
//static struct p9221_reg reg_ppp_byte_4		= {0x54, I2C_DATA_BYTE}; //Hidden
//static struct p9221_reg reg_ppp_byte_5		= {0x55, I2C_DATA_BYTE}; //Hidden
//static struct p9221_reg reg_bc_header			= {0x58, I2C_DATA_BYTE}; //Hidden
//static struct p9221_reg reg_bc_byte_1			= {0x59, I2C_DATA_BYTE}; //Hidden
//static struct p9221_reg reg_bc_byte_2			= {0x5A, I2C_DATA_BYTE}; //Hidden
//static struct p9221_reg reg_bc_byte_3			= {0x5B, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_fod_0_a			= {0x68, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_fod_0_b			= {0x69, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_fod_1_a			= {0x6A, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_fod_1_b			= {0x6B, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_fod_2_a			= {0x6C, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_fod_2_b			= {0x6D, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_fod_3_a			= {0x6E, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_fod_3_b			= {0x6F, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_fod_4_a			= {0x70, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_fod_4_b			= {0x71, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_fod_5_a			= {0x72, I2C_DATA_BYTE}; //Hidden
static struct p9221_reg reg_fod_5_b			= {0x73, I2C_DATA_BYTE}; //Hidden


/* ========================================================================== */
/* ========================================================================== */
/* reg */
static inline int REG_TO_VOUT_UV(u16 reg)
{
	return ((u64)reg * 6 * 2100000) / 4095;
}

static inline int REG_TO_VRECT_UV(u16 reg)
{
	return ((u64)reg * 10 * 2100000) / 4095;
}

static inline int REG_TO_IOUT_UA(u16 reg)
{
	return ((u64)reg * 2 * 2100000) / 4095;
}

static inline int REG_TO_TDIE_DC(u16 reg)
{
	return ((((u64)reg - 1350) * 830) / 444) - 2730;
}

static inline int REG_TO_FOP_KHZ(u16 reg)
{
	return (64 * 6000) / (u64)reg;
}

static inline u16 UV_TO_REG_VOUT_SET(int value)
{
	return (value / 100000) - 35;
}

static inline u16 UV_TO_REG_ILIM_SET(int value)
{
	return (value / 100000) - 1;
}

/* temp */
static inline bool is_between(int left, int right, int value)
{
	if (left >= right && left >= value && value >= right) {
		return true;
	}
	if (left <= right && left <= value && value <= right) {
		return true;
	}
	return false;
}

#endif /* P9221_WIRELESS_CHARGER_H */
