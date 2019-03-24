/*
 * Wireless charger driver for IDT P9221
 *
 * Copyright (C) 2018 FIH Foxconn. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "p9221-wireless-charger.h"

/* battery */
static int get_batt_capacity(struct p9221_chip *chip, union power_supply_propval *val)
{
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy) {
			pr_err("power_supply_get_by_name err\n");
			return -ENODEV;
		}
	}

	rc = power_supply_get_property(chip->batt_psy,
					POWER_SUPPLY_PROP_CAPACITY, val);
	if (rc < 0) {
		pr_err("power_supply_get_property err %d\n", rc);
		return rc;
	}

	return rc;
}

/* dc */
static bool is_dc_present(struct p9221_chip *chip)
{
	union power_supply_propval val = {0, };
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	if (!chip->dc_psy) {
		chip->dc_psy = power_supply_get_by_name("dc");
		if (!chip->dc_psy) {
			pr_err("power_supply_get_by_name err\n");
			return false;
		}
	}

	rc = power_supply_get_property(chip->dc_psy,
					POWER_SUPPLY_PROP_PRESENT, &val);
	if (rc < 0) {
		pr_err("power_supply_get_property err %d\n", rc);
		return false;
	}

	return (bool)val.intval;
}

static bool is_dc_online(struct p9221_chip *chip)
{
	union power_supply_propval val = {0, };
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	if (!chip->dc_psy) {
		chip->dc_psy = power_supply_get_by_name("dc");
		if (!chip->dc_psy) {
			pr_err("power_supply_get_by_name err\n");
			return false;
		}
	}

	rc = power_supply_get_property(chip->dc_psy,
					POWER_SUPPLY_PROP_ONLINE, &val);
	if (rc < 0) {
		pr_err("power_supply_get_property err %d\n", rc);
		return false;
	}

	return (bool)val.intval;
}

static int set_dc_current_max(struct p9221_chip *chip, int current_ua)
{
	union power_supply_propval val = {0, };
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	if (!chip->dc_psy) {
		chip->dc_psy = power_supply_get_by_name("dc");
		if (!chip->dc_psy) {
			pr_err("power_supply_get_by_name err\n");
			return -ENODEV;
		}
	}

	val.intval = current_ua;
	rc = power_supply_set_property(chip->dc_psy,
					POWER_SUPPLY_PROP_CURRENT_MAX, &val);
	if (rc < 0) {
		pr_err("power_supply_set_property err %d\n", rc);
		return rc;
	}

	return rc;
}

static int get_dc_current_max(struct p9221_chip *chip, int *current_ua)
{
	union power_supply_propval val = {0, };
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	*current_ua = -EINVAL;
	if (!chip->dc_psy) {
		chip->dc_psy = power_supply_get_by_name("dc");
		if (!chip->dc_psy) {
			pr_err("power_supply_get_by_name err\n");
			return -ENODEV;
		}
	}

	rc = power_supply_get_property(chip->dc_psy,
					POWER_SUPPLY_PROP_CURRENT_MAX, &val);
	if (rc < 0) {
		pr_err("power_supply_get_property err %d\n", rc);
		return rc;
	}
	*current_ua = val.intval;

	return rc;
}

/* temp */
static int get_bms_temp(struct p9221_chip *chip, int *temp)
{
	union power_supply_propval val = {0, };
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	*temp = -EINVAL;
	if (!chip->bms_psy) {
		chip->bms_psy = power_supply_get_by_name("bms");
		if (!chip->bms_psy) {
			pr_err("power_supply_get_by_name err\n");
			return -ENODEV;
		}
	}

	rc = power_supply_get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_TEMP, &val);
	if (rc < 0) {
		pr_err("power_supply_get_property err %d\n", rc);
		return rc;
	}
	*temp = val.intval;

	return rc;
}

static int get_skin_temp(struct p9221_chip *chip, int *temp)
{
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	*temp = -EINVAL;
	if (IS_ERR_OR_NULL(chip->temp_chan)) {
		chip->temp_chan = iio_channel_get(chip->dev, NAME_TEMP_CHAN);
	}
	if (IS_ERR(chip->temp_chan)) {
		pr_err("iio_channel_get %s err %ld\n",
			NAME_TEMP_CHAN, PTR_ERR(chip->temp_chan));
		return PTR_ERR(chip->temp_chan);
	}

	rc = iio_read_channel_processed(chip->temp_chan, temp);
	if (rc < 0) {
		pr_err("iio_read_channel_processed %s err %d\n",
			NAME_TEMP_CHAN, rc);
		return rc;
	}
	*temp /= 100;

	return rc;
}

static int get_temp(struct p9221_chip *chip, int *temp)
{
	int skin_temp = 0, bms_temp = 0;
	int rc1 = 0, rc2 = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	*temp = -EINVAL;
	#if 0 //NTC position design different with internal project.
	rc1 = get_skin_temp(chip, &skin_temp);
	if (rc1 < 0) {
		pr_err("get_skin_temp err %d\n", rc1);
	}
	#endif
	rc2 = get_bms_temp(chip, &bms_temp);
	if (rc2 < 0) {
		pr_err("get_bms_temp err %d\n", rc2);
	}
	if (rc1 < 0 && rc2 < 0) {
		return -EINVAL;
	}
	*temp = max(skin_temp, bms_temp);
	pr_dbg(PR_TEMP, "skin_temp=%d, bms_temp=%d\n", skin_temp, bms_temp);

	return 0;
}

static int get_temp_index(struct p9221_chip *chip, int temp, int *new_index)
{
	int current_index = chip->temp_ctl->index;
	struct temp_range *range = chip->temp_ctl->range;
	int hysteresis = chip->temp_ctl->hysteresis;
	int i;

	pr_dbg(PR_ENTRY, "Enter\n");

	*new_index = -EINVAL;
	/* if temp is lower than the min allowed range, return -ENODATA */
	if (temp < range[0].temp_low) {
		return -ENODATA;
	}

	/* find the matching index without hysteresis */
	for (i = 0; i < MAX_TEMP_RANGES; i++) {
		if (!range[i].temp_high && !range[i].temp_low) {
			/* found first invalid table entry; exit loop */
			break;
		}
		if (is_between(range[i].temp_low, range[i].temp_high, temp)) {
			*new_index = i;
			/* found the matching index; exit loop */
			break;
		}
	}

	/* if no matching index was found, */
	if (*new_index == -EINVAL) {
		/* return -ENODATA (array of zeros due to invalid dt property) */
		if (i == 0) {
			pr_err("array of zeros due to invalid dt property\n");
			return -ENODATA;
		}
		/* choose the last index (temp exceeds the max allowed range) */
		*new_index = (i - 1);
	}

	/* if no current_index, return this new index (no hysterisis) */
	if (current_index == -EINVAL) {
		return 0;
	}

	/* check for hysteresis if it's in the neighbourhood of current index */
	if ((*new_index == current_index + 1) &&
		(temp < range[*new_index].temp_low + hysteresis)) {
		pr_dbg(PR_TEMP, "temp=%d < %d (%d + %d): stay in %d (skip +1)\n",
			temp, range[*new_index].temp_low + hysteresis,
			range[*new_index].temp_low, hysteresis, current_index);
		/*
		 * temp is not higher by hysteresis amount
		 * stay in the current index,
		 */
		*new_index = current_index;
	} else if ((*new_index == current_index - 1) &&
		(temp > range[*new_index].temp_high - hysteresis)) {
		pr_dbg(PR_TEMP, "temp=%d > %d (%d - %d): stay in %d (skip -1)\n",
			temp, range[*new_index].temp_high - hysteresis,
			range[*new_index].temp_high, hysteresis, current_index);
		/*
		 * temp is not lower by hysteresis amount
		 * stay in the current index,
		 */
		*new_index = current_index;
	}

	return 0;
}

/* i2c */
static int p9221_i2c_xfer(struct i2c_client *client,
				enum i2c_xfer_type read_write, u8 addr,
				enum i2c_data_size size, u16 *data)
{
	unsigned char msgbuf0[NUM_BYTE_ADDR + MAX_BYTE_DATA] = {0, addr};
	unsigned char msgbuf1[MAX_BYTE_DATA] = {0};
	int msgnum = (read_write == I2C_WRITE) ? 1 : 2;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = NUM_BYTE_ADDR,
			.buf = msgbuf0,
		}, {
			.addr = client->addr,
			.flags = client->flags | I2C_M_RD,
			.len = size,
			.buf = msgbuf1,
		},
	};
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	if (read_write == I2C_WRITE) {
		msg[0].len += size;
		switch (size) {
		case I2C_DATA_BYTE:
			msgbuf0[2] = *data;
			pr_dbg(PR_I2C, "write[0x%02x] byte(0x%02x)\n",
					msgbuf0[1], msgbuf0[2]);
			break;
		case I2C_DATA_WORD:
			msgbuf0[2] = *data & 0xff;
			msgbuf0[3] = *data >> 8;
			pr_dbg(PR_I2C, "write[0x%02x] word(0x%02x, 0x%02x)\n",
					msgbuf0[1], msgbuf0[3], msgbuf0[2]);
			break;
		default:
			pr_err("Unsupported size %d\n", size);
			return -EOPNOTSUPP;
		}
	}

	rc = i2c_transfer(client->adapter, msg, msgnum);
	if (rc < 0) {
		pr_err("i2c_transfer err %d\n", rc);
		return rc;
	}

	if (read_write == I2C_READ) {
		switch (size) {
		case I2C_DATA_BYTE:
			*data = msgbuf1[0];
			pr_dbg(PR_I2C, "read[0x%02x] byte(0x%02x)\n",
					msgbuf0[1], msgbuf1[0]);
			break;
		case I2C_DATA_WORD:
			*data = msgbuf1[0] | (msgbuf1[1] << 8);
			pr_dbg(PR_I2C, "read[0x%02x] word(0x%02x, 0x%02x)\n",
					msgbuf0[1], msgbuf1[1], msgbuf1[0]);
			break;
		default:
			pr_err("Unsupported size %d\n", size);
			return -EOPNOTSUPP;
		}
	}

	return rc;
}

/* reg */
static int p9221_read(struct p9221_chip *chip, struct p9221_reg reg, u16 *data)
{
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	if (!is_dc_present(chip)) {
		pr_dbg(PR_DC, "!is_dc_present\n");
		return -ENODEV;
	}

	mutex_lock(&chip->read_write_lock);
	pm_stay_awake(chip->dev);
	rc = p9221_i2c_xfer(chip->client, I2C_READ, reg.addr, reg.size, data);
	if (rc < 0) {
		pr_err("p9221_i2c_xfer 0x%02x err %d\n", reg.addr, rc);
	}
	pm_relax(chip->dev);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int p9221_write(struct p9221_chip *chip, struct p9221_reg reg, u16 data)
{
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	if (!is_dc_present(chip)) {
		pr_dbg(PR_DC, "!is_dc_present\n");
		return -ENODEV;
	}

	mutex_lock(&chip->read_write_lock);
	pm_stay_awake(chip->dev);
	rc = p9221_i2c_xfer(chip->client, I2C_WRITE, reg.addr, reg.size, &data);
	if (rc < 0) {
		pr_err("p9221_i2c_xfer 0x%02x err %d\n", reg.addr, rc);
	}
	pm_relax(chip->dev);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int p9221_set_vout_set(struct p9221_chip *chip, int voltage_uv)
{
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	rc = p9221_write(chip, reg_vout_set, UV_TO_REG_VOUT_SET(voltage_uv));
	if (rc < 0) {
		pr_err("p9221_write err %d\n", rc);
		return rc;
	}

	return rc;
}

static int p9221_set_ilim_set(struct p9221_chip *chip, int current_ua)
{
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	rc = p9221_write(chip, reg_ilim_set, UV_TO_REG_ILIM_SET(current_ua));
	if (rc < 0) {
		pr_err("p9221_write err %d\n", rc);
		return rc;
	}

	return rc;
}

static int p9221_set_temp_ctl(struct p9221_chip *chip, int index)
{
	int voltage_uv, current_ua;
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	if (chip->is_epp_supported) {
		voltage_uv = chip->temp_ctl->range[index].epp_uv;
		current_ua = chip->temp_ctl->range[index].epp_ua;
	} else {
		voltage_uv = chip->temp_ctl->range[index].bpp_uv;
		current_ua = chip->temp_ctl->range[index].bpp_ua;
	}
	pr_dbg(PR_TEMP, "voltage_uv=%d, current_ua=%d\n", voltage_uv, current_ua);

	rc = p9221_set_vout_set(chip, voltage_uv);
	if (rc < 0) {
		pr_err("p9221_set_vout_set err %d\n", rc);
		return rc;
	}
	rc = set_dc_current_max(chip, current_ua);
	if (rc < 0) {
		pr_err("set_dc_current_max err %d\n", rc);
		return rc;
	}

	return rc;
}

static int p9221_set_fod(struct p9221_chip *chip, enum pp_type power_profile)
{
	u8 fod[NUM_FOD_PARM];
	int rc = 0;
	int i;

	pr_dbg(PR_ENTRY, "Enter\n");

	switch (power_profile) {
	case BPP:
		for (i = 0; i < NUM_FOD_PARM; i++) {
			fod[i] = chip->fod_bpp[i];
		}
		break;
	case EPP:
		for (i = 0; i < NUM_FOD_PARM; i++) {
			fod[i] = chip->fod_epp[i];
		}
		break;
	default:
		pr_err("Unsupported power_profile %d\n", power_profile);
		return -EOPNOTSUPP;
	}

	rc = p9221_write(chip, reg_fod_0_a, fod[0]);
	rc = p9221_write(chip, reg_fod_0_b, fod[1]);
	rc = p9221_write(chip, reg_fod_1_a, fod[2]);
	rc = p9221_write(chip, reg_fod_1_b, fod[3]);
	rc = p9221_write(chip, reg_fod_2_a, fod[4]);
	rc = p9221_write(chip, reg_fod_2_b, fod[5]);
	rc = p9221_write(chip, reg_fod_3_a, fod[6]);
	rc = p9221_write(chip, reg_fod_3_b, fod[7]);
	rc = p9221_write(chip, reg_fod_4_a, fod[8]);
	rc = p9221_write(chip, reg_fod_4_b, fod[9]);
	rc = p9221_write(chip, reg_fod_5_a, fod[10]);
	rc = p9221_write(chip, reg_fod_5_b, fod[11]);
	if (rc < 0) {
		pr_err("p9221_write err %d\n", rc);
		return rc;
	}

	return rc;
}

/* device tree */
static int p9221_parse_dt_gpio(struct p9221_chip *chip, struct device_node *node)
{
	pr_dbg(PR_ENTRY, "Enter\n");

	/* NAME_GPIO_EN */
	chip->gpio_en = of_get_named_gpio(node, NAME_GPIO_EN, 0);
	if (chip->gpio_en < 0) {
		pr_err("of_get_named_gpio %s err\n", NAME_GPIO_EN);
		return -EINVAL;
	}
	pr_info("%s=%d\n", NAME_GPIO_EN, chip->gpio_en);

	/* NAME_GPIO_INT */
	chip->gpio_int = of_get_named_gpio(node, NAME_GPIO_INT, 0);
	if (chip->gpio_int < 0) {
		pr_err("of_get_named_gpio %s err\n", NAME_GPIO_INT);
		return -EINVAL;
	}
	pr_info("%s=%d\n", NAME_GPIO_INT, chip->gpio_int);

	return 0;
}

static int p9221_parse_dt_fod(struct p9221_chip *chip, struct device_node *node)
{
	int rc = 0;
	int i;

	pr_dbg(PR_ENTRY, "Enter\n");

	/* NAME_FOD_BPP */
	rc = of_property_count_elems_of_size(node, NAME_FOD_BPP, sizeof(u8));
	if (rc != NUM_FOD_PARM) {
		pr_err("of_property_count_elems_of_size err %d\n", rc);
		return -EINVAL;
	}
	rc = of_property_read_u8_array(node, NAME_FOD_BPP, chip->fod_bpp,
					NUM_FOD_PARM);
	if (rc < 0) {
		pr_err("of_property_read_u8_array err %d\n", rc);
		return rc;
	}
	pr_info("%s=[ ", NAME_FOD_BPP);
	for (i = 0; i < NUM_FOD_PARM; i++) {
		pr_cont("%02x ", chip->fod_bpp[i]);
	}
	pr_cont("]\n");

	/* NAME_FOD_EPP */
	rc = of_property_count_elems_of_size(node, NAME_FOD_EPP, sizeof(u8));
	if (rc != NUM_FOD_PARM) {
		pr_err("of_property_count_elems_of_size err %d\n", rc);
		return -EINVAL;
	}
	rc = of_property_read_u8_array(node, NAME_FOD_EPP, chip->fod_epp,
					NUM_FOD_PARM);
	if (rc < 0) {
		pr_err("of_property_read_u8_array err %d\n", rc);
		return rc;
	}
	pr_info("%s=[ ", NAME_FOD_EPP);
	for (i = 0; i < NUM_FOD_PARM; i++) {
		pr_cont("%02x ", chip->fod_epp[i]);
	}
	pr_cont("]\n");

	return rc;
}

static int p9221_parse_dt_temp(struct p9221_chip *chip, struct device_node *node)
{
	int num_elem_per_entry, num_elem, num_entry;
	int rc = 0;
	int i;

	pr_dbg(PR_ENTRY, "Enter\n");

	/* NAME_TEMP_CONTROL_ENABLE */
	chip->temp_ctl->enable = of_property_read_bool(node,
							NAME_TEMP_CONTROL_ENABLE);
	pr_info("%s=%d\n", NAME_TEMP_CONTROL_ENABLE, chip->temp_ctl->enable);
	if (!chip->temp_ctl->enable) {
		return rc;
	}

	/* NAME_TEMP_RANGES */
	rc = of_property_count_elems_of_size(node, NAME_TEMP_RANGES, sizeof(u32));
	if (rc < 0) {
		pr_err("of_property_count_elems_of_size err %d\n", rc);
		return rc;
	}

	num_elem = rc;
	num_elem_per_entry = sizeof(struct temp_range) / sizeof(u32);
	if (num_elem % num_elem_per_entry) {
		pr_err("elements(%d) should be multiple of %d\n",
			num_elem, num_elem_per_entry);
		return -EINVAL;
	}

	num_entry = num_elem / num_elem_per_entry;
	if (num_entry > MAX_TEMP_RANGES) {
		pr_err("entries(%d) should be less than %d\n",
			num_entry, MAX_TEMP_RANGES);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(node, NAME_TEMP_RANGES,
					(u32 *)chip->temp_ctl->range, num_elem);
	if (rc < 0) {
		pr_err("of_property_read_u32_array err %d\n", rc);
		return rc;
	}
	for (i = 0; i < num_entry; i++) {
		pr_info("%s[%d]=%3d~%3d(dC): "
			"BPP:%7d(uV)/%7d(uA), EPP:%7d(uV)/%7d(uA)\n",
			NAME_TEMP_RANGES, i,
			chip->temp_ctl->range[i].temp_low,
			chip->temp_ctl->range[i].temp_high,
			chip->temp_ctl->range[i].bpp_uv,
			chip->temp_ctl->range[i].bpp_ua,
			chip->temp_ctl->range[i].epp_uv,
			chip->temp_ctl->range[i].epp_ua);

		if (chip->temp_ctl->range[i].temp_low >
			chip->temp_ctl->range[i].temp_high) {
			pr_err("temp_low(%d) should be <= %d\n",
				chip->temp_ctl->range[i].temp_low,
				chip->temp_ctl->range[i].temp_high);
			return -EINVAL;
		}
		if (i != 0) {
			if (chip->temp_ctl->range[i].temp_low !=
				chip->temp_ctl->range[i-1].temp_high + 1) {
				pr_err("temp_low(%d) should be %d\n",
					chip->temp_ctl->range[i].temp_low,
					chip->temp_ctl->range[i-1].temp_high + 1);
				return -EINVAL;
			}
		}

		if (chip->temp_ctl->range[i].bpp_uv > UV_5V) {
			pr_err("bpp_uv(%d) should be <= %d\n",
				chip->temp_ctl->range[i].bpp_uv, UV_5V);
			return -EINVAL;
		}
		if (chip->temp_ctl->range[i].epp_uv > UV_9V) {
			pr_err("epp_uv(%d) should be <= %d\n",
				chip->temp_ctl->range[i].epp_uv, UV_9V);
			return -EINVAL;
		}
	}

	/* NAME_TEMP_HYSTERESIS */
	rc = of_property_read_u32(node, NAME_TEMP_HYSTERESIS,
					&chip->temp_ctl->hysteresis);
	if (rc < 0) {
		pr_err("of_property_read_u32 err %d\n", rc);
	}
	pr_info("%s=%d\n", NAME_TEMP_HYSTERESIS, chip->temp_ctl->hysteresis);

	return rc;
}

static int p9221_parse_dt(struct p9221_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	if (!node) {
		pr_err("device tree node missing\n");
		return -EINVAL;
	}

	/* gpio */
	rc = p9221_parse_dt_gpio(chip, node);
	if (rc < 0) {
		pr_err("p9221_parse_dt_gpio err %d\n", rc);
		return rc;
	}

	/* fod */
	rc = p9221_parse_dt_fod(chip, node);
	if (rc < 0) {
		pr_err("p9221_parse_dt_fod err %d\n", rc);
		return rc;
	}

	/* temp */
	rc = p9221_parse_dt_temp(chip, node);
	if (rc < 0) {
		pr_err("p9221_parse_dt_temp err %d\n", rc);
		return rc;
	}

	return rc;
}

/* irq */
static int p9221_irq_enable(struct p9221_chip *chip)
{
	int rc = 0;

	rc = p9221_write(chip, reg_int_enable, BIT_OVER_CURRENT |
						BIT_OVER_VOLTAGE |
						BIT_OVER_TEMPERATURE |
						BIT_VOLTAGE_CHANGED);
	if (rc < 0) {
		pr_err("p9221_write err %d\n", rc);
		return rc;
	}

	return rc;
}

static irqreturn_t p9221_irq_handler(int irq, void *dev_id)
{
	struct p9221_chip *chip = dev_id;
	u16 reg = 0;
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	rc = p9221_read(chip, reg_int_status, &reg);
	if (rc >= 0) {
		pr_info("Interrupt status=0x%04x\n", reg);
		if (reg & BIT_OVER_CURRENT) {
			pr_info("IRQ: current limit has been exceeded\n");
		}
		if (reg & BIT_OVER_VOLTAGE) {
			pr_info("IRQ: rectifier over-voltage condition exists\n");
		}
		if (reg & BIT_OVER_TEMPERATURE) {
			pr_info("IRQ: over-temperature condition exists\n");
		}
		if (reg & BIT_VOLTAGE_CHANGED) {
			pr_info("IRQ: output voltage changed\n");
		}
	}

	return IRQ_HANDLED;
}

/* gpio */
static int p9221_setup_gpio(struct p9221_chip *chip)
{
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	/* Enable pin */
	if (!gpio_is_valid(chip->gpio_en)) {
		pr_err("gpio_is_valid %d err\n", chip->gpio_en);
		return -EINVAL;
	}

	rc = devm_gpio_request(chip->dev, chip->gpio_en, NAME_GPIO_EN);
	if (rc < 0) {
		pr_err("devm_gpio_request %d err %d\n", chip->gpio_en, rc);
		return rc;
	}

	rc = gpio_direction_output(chip->gpio_en, 0);
	if (rc < 0) {
		pr_err("gpio_direction_output %d err %d\n", chip->gpio_en, rc);
		return rc;
	}

	/* Interrupt pin */
	if (!gpio_is_valid(chip->gpio_int)) {
		pr_err("gpio_is_valid %d err\n", chip->gpio_int);
		return -EINVAL;
	}

	rc = devm_gpio_request(chip->dev, chip->gpio_int, NAME_GPIO_INT);
	if (rc < 0) {
		pr_err("devm_gpio_request %d err %d\n", chip->gpio_int, rc);
		return rc;
	}

	rc = gpio_direction_input(chip->gpio_int);
	if (rc < 0) {
		pr_err("gpio_direction_input %d err %d\n", chip->gpio_int, rc);
		return rc;
	}

	rc = devm_request_threaded_irq(chip->dev, gpio_to_irq(chip->gpio_int),
					NULL, p9221_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"p9221_irq", chip);
	if (rc < 0) {
		pr_err("devm_request_threaded_irq %d err %d\n",
			gpio_to_irq(chip->gpio_int), rc);
		return rc;
	}

	return rc;
}

/* work */
static void p9221_fod_work(struct work_struct *work)
{
	struct p9221_chip *chip = container_of(work, struct p9221_chip,
						p9221_fod_work.work);
	u16 reg = 0;
	enum pp_type power_profile;
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	rc = p9221_read(chip, reg_adc_vout, &reg);
	if (rc < 0) {
		pr_err("p9221_read err %d\n", rc);
		goto end;
	}

	if (REG_TO_VOUT_UV(reg) < PP_THRESHOLD_UV) {
		power_profile = BPP;
	} else {
		power_profile = EPP;
		chip->is_epp_supported = true;
	}

	if (chip->power_profile != power_profile) {
		pr_info("Vout=%d: power_profile=%d to %d\n",
			REG_TO_VOUT_UV(reg), chip->power_profile, power_profile);
		rc = p9221_set_fod(chip, power_profile);
		if (rc < 0) {
			pr_err("p9221_set_fod err %d\n", rc);
			goto end;
		}
		chip->power_profile = power_profile;
	}
end:
	schedule_delayed_work(&chip->p9221_fod_work, msecs_to_jiffies(1000));
}

static void p9221_temp_work(struct work_struct *work)
{
	struct p9221_chip *chip = container_of(work, struct p9221_chip,
						p9221_temp_work.work);
	int temp = 0;
	int index = 0;
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	rc = get_temp(chip, &temp);
	if (rc < 0) {
		pr_err("get_temp err %d\n", rc);
		goto end;
	}

	rc = get_temp_index(chip, temp, &index);
	if (rc < 0) {
		pr_err("get_temp_index err %d\n", rc);
		goto end;
	}

	if (chip->temp_ctl->index != index) {
		pr_info("temp=%d: index=%d to %d\n",
			temp, chip->temp_ctl->index, index);
		rc = p9221_set_temp_ctl(chip, index);
		if (rc < 0) {
			pr_err("p9221_set_temp_ctl err %d\n", rc);
			goto end;
		}
		chip->temp_ctl->index = index;
	}
end:
	schedule_delayed_work(&chip->p9221_temp_work, msecs_to_jiffies(5000));
}

/*
         <- T1 ->     <- T2 ->     <- T3 ->          <- T4 ->
          ------       ------       ------            ------
         |      |     |      |     |      |          |      |
         |  C1  |     |  C2  |     |  C3  |   ....   |  C4  |
         |      |     |      |     |      |          |      |
Vout-----        -----        -----        ----------        ----------
         ^      ^                  ^                 ^
         |      |                  |                 |
      Attached  |                  |                On after re-Attached
             Detached             Off if the last n cycles are bad cycles
*/
static void p9221_bad_cycle_detect(struct p9221_chip *chip, enum wlc_state state)
{
	static ktime_t kt_attached;
	ktime_t kt_detached;
	s64 cycle_time_ms;
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	if (state == WLC_ATTACHED) {
		kt_attached = ktime_get();
		/* Check bad cycle count */
		if (chip->bad_cycle_count == MAX_BAD_CYCLE_COUNT) {
			pr_info("the last %d cycles are bad cycles: output off\n",
					chip->bad_cycle_count);
			rc = p9221_write(chip, reg_command,
						BIT_TOGGLE_LDO_ON_OFF |
						BIT_SEND_END_POWER_TRANSFER);
			if (rc < 0) {
				pr_err("p9221_write err %d\n", rc);
				return;
			}
			chip->output_off = true;
			chip->bad_cycle_count = 0;
		} else {
			chip->output_off = false;
		}
	} else if (kt_attached.tv64 > 0 && !chip->output_off) {
		kt_detached = ktime_get();
		cycle_time_ms = ktime_ms_delta(kt_detached, kt_attached);
		/* Check cycle time */
		pr_info("cycle_time_ms=%lld\n", cycle_time_ms);
		if (cycle_time_ms < MIN_GOOD_CYCLE_TIME_MS) {
			chip->bad_cycle_count++;
		} else {
			chip->bad_cycle_count = 0;
		}
	}
}

static void p9221_change_work(struct work_struct *work)
{
	struct p9221_chip *chip = container_of(work, struct p9221_chip,
						p9221_change_work);

	pr_dbg(PR_ENTRY, "Enter\n");

	if (is_dc_present(chip)) {
		pr_info("Wireless charger is attached\n");
		p9221_bad_cycle_detect(chip, WLC_ATTACHED);
		p9221_irq_enable(chip);
		p9221_set_ilim_set(chip, 1600000);
		chip->power_profile = NPP;
		chip->is_epp_supported = false;
		schedule_delayed_work(&chip->p9221_fod_work, msecs_to_jiffies(0));
		if (chip->temp_ctl->enable) {
			chip->temp_ctl->index = -EINVAL;
			schedule_delayed_work(&chip->p9221_temp_work,
						msecs_to_jiffies(5000));
		}
	} else {
		pr_info("Wireless charger is dettached\n");
		p9221_bad_cycle_detect(chip, WLC_DETACHED);
		cancel_delayed_work_sync(&chip->p9221_fod_work);
		if (chip->temp_ctl->enable) {
			cancel_delayed_work_sync(&chip->p9221_temp_work);
		}
	}
}

#define FULL_CAPACITY	100
static void p9221_End_Power_Transfer_work(struct work_struct *work)
{
	struct p9221_chip *chip = container_of(work, struct p9221_chip,
						p9221_End_Power_Transfer_work);
	union power_supply_propval val = {0, };
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");
	if (is_dc_present(chip)) {
		get_batt_capacity(chip, &val);
		if (val.intval == FULL_CAPACITY){
			//SS behaver "SC100"
			rc = p9221_write(chip, reg_battery_charge_status,
						0x64);
			if (rc < 0) {
				pr_err("p9221_write err %d\n", rc);
				return;
			}
			rc = p9221_write(chip, reg_command,
						0x10);
			if (rc < 0) {
				pr_err("p9221_write err %d\n", rc);
				return;
			}
			//standard Qi comment to send "charge completed"
			/*
			rc = p9221_write(chip, reg_ept_code,
						BIT_EPT_Code_Charge_Complete);
			if (rc < 0) {
				pr_err("p9221_write err %d\n", rc);
				return;
			}
			rc = p9221_write(chip, reg_command,
						BIT_SEND_END_POWER_TRANSFER);
			if (rc < 0) {
				pr_err("p9221_write err %d\n", rc);
				return;
			}
			*/
		}
	}
}

/* notifier */
static int p9221_notifier_call(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct p9221_chip *chip = container_of(nb, struct p9221_chip, nb);
	struct power_supply *psy = data;

	pr_dbg(PR_ENTRY, "Enter\n");

	if (!strcmp(psy->desc->name, "dc")) {
		if (!chip->dc_psy) {
			chip->dc_psy = psy;
		}
		if (event == PSY_EVENT_PROP_CHANGED) {
			schedule_work(&chip->p9221_change_work);
		}
	}
	else {
		schedule_work(&chip->p9221_End_Power_Transfer_work);
	}

	return NOTIFY_OK;
}

static int p9221_register_notifier(struct p9221_chip *chip)
{
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	chip->nb.notifier_call = p9221_notifier_call;
	rc = power_supply_reg_notifier(&chip->nb);
	if (rc < 0) {
		pr_err("power_supply_reg_notifier err %d\n", rc);
		return rc;
	}

	return rc;
}

/* power supply */
static enum power_supply_property p9221_properties[] = {
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGER_TEMP,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
	POWER_SUPPLY_PROP_BUCK_FREQ,
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
};

static int p9221_get_property(struct power_supply *psy,
				enum power_supply_property prop,
				union power_supply_propval *val)
{
	struct p9221_chip *chip = power_supply_get_drvdata(psy);
	u16 reg = 0;
	int rc = 0;

	pr_dbg(PR_ENTRY, "Enter\n");

	val->intval = -EINVAL;
	switch (prop) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = chip->dev->driver->name;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = is_dc_present(chip);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = is_dc_online(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = p9221_read(chip, reg_adc_vout, &reg);
		if (rc >= 0) {
			val->intval = REG_TO_VOUT_UV(reg);
		} else {
			val->intval = 0;
			rc = 0;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = p9221_read(chip, reg_adc_vrect, &reg);
		if (rc >= 0) {
			val->intval = REG_TO_VRECT_UV(reg);
		} else {
			val->intval = 0;
			rc = 0;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = p9221_read(chip, reg_rx_iout, &reg);
		if (rc >= 0) {
			val->intval = REG_TO_IOUT_UA(reg);
		} else {
			val->intval = 0;
			rc = 0;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = get_dc_current_max(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGER_TEMP:
		rc = p9221_read(chip, reg_adc_die_temp, &reg);
		if (rc >= 0) {
			val->intval = REG_TO_TDIE_DC(reg);
		}
		break;
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
		/* do not query RRADC if charger is not present */
		if (is_dc_present(chip)) {
			rc = get_skin_temp(chip, &val->intval);
		}
		break;
	case POWER_SUPPLY_PROP_BUCK_FREQ:
		rc = p9221_read(chip, reg_op_freq, &reg);
		if (rc >= 0) {
			val->intval = REG_TO_FOP_KHZ(reg);
		}
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		val->intval = chip->output_off;
		break;
	default:
		pr_err_ratelimited("Unsupported prop %d\n", prop);
		return -EINVAL;
	}
	if (rc < 0) {
		pr_dbg(PR_PROP, "get_property %d err %d\n", prop, rc);
		return -ENODATA;
	}

	return 0;
}

static struct power_supply_desc psy_desc = {
	.name			= "wireless",
	.type			= POWER_SUPPLY_TYPE_WIRELESS,
	.properties		= p9221_properties,
	.num_properties		= ARRAY_SIZE(p9221_properties),
	.get_property		= p9221_get_property,
};

static int p9221_register_power_supply(struct p9221_chip *chip)
{
	struct power_supply_config psy_cfg = {};

	pr_dbg(PR_ENTRY, "Enter\n");

	psy_cfg.drv_data = chip;
	psy_cfg.num_supplicants = 0;

	chip->psy = devm_power_supply_register(chip->dev, &psy_desc, &psy_cfg);
	if (IS_ERR(chip->psy)) {
		pr_err("devm_power_supply_register err %ld\n", PTR_ERR(chip->psy));
		return PTR_ERR(chip->psy);
	}

	return 0;
}

/* sysfs */
#define ATTR_SHOW(x)								\
static ssize_t p9221_##x##_show(struct device *dev,				\
				struct device_attribute *attr, char *buf)	\
{										\
	struct p9221_chip *chip = dev_get_drvdata(dev);				\
	u16 reg = 0;								\
	int rc = 0;								\
										\
	pr_dbg(PR_ENTRY, "Enter\n");						\
										\
	rc = p9221_read(chip, reg_##x, &reg);					\
	if (rc < 0) {								\
		return scnprintf(buf, PAGE_SIZE, "%s(0x%02x)=%d(err)\n",	\
					#x, reg_##x.addr, rc);			\
	}									\
										\
	if (reg_##x.size == I2C_DATA_BYTE) {					\
		return scnprintf(buf, PAGE_SIZE, "%s(0x%02x)=0x%02x\n",		\
					#x, reg_##x.addr, reg);			\
	} else {								\
		return scnprintf(buf, PAGE_SIZE, "%s(0x%02x)=0x%04x\n",		\
					#x, reg_##x.addr, reg);			\
	}									\
}										\
										\
static DEVICE_ATTR(reg_##x, S_IRUGO | S_IWUSR | S_IWGRP, p9221_##x##_show, NULL)

#if defined(CONFIG_FIH_FOD_TEST)
static ssize_t p9221_fod_for_license_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t len)
{
	struct p9221_chip *chip = dev_get_drvdata(dev);
	unsigned fod[12];
	int res;
	int rc = 0;
	int i;

	pr_dbg(PR_ENTRY, "Enter\n");
	res = sscanf(buf, "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
		&fod[0], &fod[1], &fod[2], &fod[3], &fod[4], &fod[5], &fod[6], &fod[7], &fod[8], &fod[9], &fod[10], &fod[11]);
	if (res == 12 && len == 60)	{
		
		for (i = 0; i < 12; i++) {
			pr_info("set fod=>  0x%02x\n", fod[i]);
		}

		rc = p9221_write(chip, reg_fod_0_a, fod[0]);
		rc = p9221_write(chip, reg_fod_0_b, fod[1]);
		rc = p9221_write(chip, reg_fod_1_a, fod[2]);
		rc = p9221_write(chip, reg_fod_1_b, fod[3]);
		rc = p9221_write(chip, reg_fod_2_a, fod[4]);
		rc = p9221_write(chip, reg_fod_2_b, fod[5]);
		rc = p9221_write(chip, reg_fod_3_a, fod[6]);
		rc = p9221_write(chip, reg_fod_3_b, fod[7]);
		rc = p9221_write(chip, reg_fod_4_a, fod[8]);
		rc = p9221_write(chip, reg_fod_4_b, fod[9]);
		rc = p9221_write(chip, reg_fod_5_a, fod[10]);
		rc = p9221_write(chip, reg_fod_5_b, fod[11]);
		if (rc < 0) {
			pr_err("p9221_write err %d\n", rc);
			return rc;
		}
	}
	else
		pr_info("set fod=>  fail\n");

	return len;
}
static DEVICE_ATTR(fod_for_license, S_IRUGO | S_IWUSR | S_IWGRP, NULL, p9221_fod_for_license_store);
#endif /* CONFIG_FIH_FOD_TEST */

ATTR_SHOW(chip_id);
ATTR_SHOW(chip_rev);
ATTR_SHOW(customer_id);
ATTR_SHOW(fw_major_rev);
ATTR_SHOW(fw_minor_rev);
ATTR_SHOW(status);
ATTR_SHOW(int_status);
ATTR_SHOW(int_enable);
ATTR_SHOW(ept_code);
ATTR_SHOW(vout_set);
ATTR_SHOW(vrect_adj);
ATTR_SHOW(ilim_set);
ATTR_SHOW(sys_op_mode);
ATTR_SHOW(command);
ATTR_SHOW(fod_0_a);
ATTR_SHOW(fod_0_b);
ATTR_SHOW(fod_1_a);
ATTR_SHOW(fod_1_b);
ATTR_SHOW(fod_2_a);
ATTR_SHOW(fod_2_b);
ATTR_SHOW(fod_3_a);
ATTR_SHOW(fod_3_b);
ATTR_SHOW(fod_4_a);
ATTR_SHOW(fod_4_b);
ATTR_SHOW(fod_5_a);
ATTR_SHOW(fod_5_b);

static struct attribute *p9221_attrs[] = {
	&dev_attr_reg_chip_id.attr,
	&dev_attr_reg_chip_rev.attr,
	&dev_attr_reg_customer_id.attr,
	&dev_attr_reg_fw_major_rev.attr,
	&dev_attr_reg_fw_minor_rev.attr,
	&dev_attr_reg_status.attr,
	&dev_attr_reg_int_status.attr,
	&dev_attr_reg_int_enable.attr,
	&dev_attr_reg_ept_code.attr,
	&dev_attr_reg_vout_set.attr,
	&dev_attr_reg_vrect_adj.attr,
	&dev_attr_reg_ilim_set.attr,
	&dev_attr_reg_sys_op_mode.attr,
	&dev_attr_reg_command.attr,
	&dev_attr_reg_fod_0_a.attr,
	&dev_attr_reg_fod_0_b.attr,
	&dev_attr_reg_fod_1_a.attr,
	&dev_attr_reg_fod_1_b.attr,
	&dev_attr_reg_fod_2_a.attr,
	&dev_attr_reg_fod_2_b.attr,
	&dev_attr_reg_fod_3_a.attr,
	&dev_attr_reg_fod_3_b.attr,
	&dev_attr_reg_fod_4_a.attr,
	&dev_attr_reg_fod_4_b.attr,
	&dev_attr_reg_fod_5_a.attr,
	&dev_attr_reg_fod_5_b.attr,
#if defined(CONFIG_FIH_FOD_TEST)
	&dev_attr_fod_for_license.attr,
#endif /* CONFIG_FIH_FOD_TEST */
	NULL,
};

static struct attribute_group p9221_attr_group = {
	.attrs = p9221_attrs
};

/* probe */
static int p9221_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct p9221_chip *chip;
	int rc = 0;

	pr_info("Begin: i2c slave address = 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality err\n");
		return -EIO;
	}

	chip = devm_kzalloc(&client->dev, sizeof(struct p9221_chip), GFP_KERNEL);
	if (!chip) {
		pr_err("devm_kzalloc err\n");
		return -ENOMEM;
	}
	chip->client = client;
	chip->dev = &client->dev;

	chip->temp_ctl = devm_kzalloc(&client->dev, sizeof(struct temp_control),
					GFP_KERNEL);
	if (!chip->temp_ctl) {
		pr_err("devm_kzalloc err\n");
		return -ENOMEM;
	}

	rc = p9221_parse_dt(chip);
	if (rc < 0) {
		pr_err("p9221_parse_dt err %d\n", rc);
		return rc;
	}

	rc = p9221_setup_gpio(chip);
	if (rc < 0) {
		pr_err("p9221_setup_gpio err %d\n", rc);
		return rc;
	}

	mutex_init(&chip->read_write_lock);
	INIT_WORK(&chip->p9221_change_work, p9221_change_work);
	INIT_WORK(&chip->p9221_End_Power_Transfer_work, p9221_End_Power_Transfer_work);
	INIT_DELAYED_WORK(&chip->p9221_fod_work, p9221_fod_work);
	if (chip->temp_ctl->enable) {
		INIT_DELAYED_WORK(&chip->p9221_temp_work, p9221_temp_work);
	}

	rc = p9221_register_notifier(chip);
	if (rc < 0) {
		pr_err("p9221_register_notifier err %d\n", rc);
		return rc;
	}

	rc = p9221_register_power_supply(chip);
	if (rc < 0) {
		pr_err("p9221_register_power_supply err %d\n", rc);
		return rc;
	}

	dev_set_drvdata(chip->dev, chip);
	rc = sysfs_create_group(&chip->dev->kobj, &p9221_attr_group);
	if (rc) {
		pr_err("sysfs_create_group err %d\n", rc);
		return rc;
	}

	schedule_work(&chip->p9221_change_work);

	pr_info("End\n");
	return rc;
}

static const struct of_device_id p9221_match_table[] = {
	{ .compatible = "idt,p9221", },
	{ }
};
MODULE_DEVICE_TABLE(of, p9221_match_table);

static struct i2c_device_id p9221_id_table[] = {
	{ "p9221", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, p9221_id_table);

static struct i2c_driver p9221_driver = {
	.driver = {
		.name		= "p9221",
		.owner		= THIS_MODULE,
		.of_match_table	= p9221_match_table,
	},
	.id_table	= p9221_id_table,
	.probe		= p9221_probe,
};
module_i2c_driver(p9221_driver);

MODULE_AUTHOR("Jimmy Hu <JimmyCYHu@fih-foxconn.com>");
MODULE_DESCRIPTION("Driver for IDT P9221 Wireless Charger");
MODULE_LICENSE("GPL v2");
