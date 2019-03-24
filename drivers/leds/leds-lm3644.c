/*
 * TI LM3644(TT) CAMERA FLASH LED Driver
 *
 * Copyright (C) 2017 FIH co.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Author:  PinyCHWu <pinychwu@fih-foxconn.com>
 *	    Nov. 2017
 *
 * This driver not handle below features:
 * - TX-pin
 * - IVFM (use default)
 * - Boost (use default)
 * - NTC
 * - IR mode
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/lm3644.h>
#include <linux/leds-lm3644.h>
#include "leds.h"
#include "cam_sensor_dev.h"

enum ledlist
{
    LED1 = 1,
    LED2,
};

enum lm3644_ledid
{
    ID_FLASH0 = 0x0,
    ID_FLASH1,
    ID_TORCH0,
    ID_TORCH1,
    ID_SWITCH0,
    ID_MAX
};

enum led_status
{
    LED_INVALID = 0,
    LED_IS_VALID,
    LED_REGISTER_DONE,

    NUM_OF_LED_STATUS
};

#define FLASH_MAX_CUR		1500
#define TORCH_MAX_CUR		180
#define TORCH_MAX_CUR_TT	360
struct led_data
{
    enum led_status status;
    unsigned int id;
    unsigned int mask;
    bool on;
    struct led_classdev cdev;
    //work
    struct work_struct	set_work;
    struct workqueue_struct *lm3644_queue;
    unsigned int		work_brightness;
};

enum gpio_status
{
    GPIO_INVALID = 0,
    GPIO_USED,
    GPIO_REQUESTED,

    NUM_OF_GPIO_STATUS
};

struct gpio_data
{
    enum gpio_status use_pin;
    unsigned int gpio;
};

struct lm3644_chip
{
    struct device_node *of_node;
    struct cam_subdev v4l2_dev_str;
    struct cam_hw_soc_info soc_info;
    struct device		*dev;
    struct regmap		*regmap;
    struct reg_default	base_setting[LM3644_REG_FLAG1];
    enum cci_i2c_master_t cci_i2c_master;
    struct camera_io_master io_master_info;
    struct regulator *vdd;
    struct mutex lm3644_mutex;

    unsigned int		use_pinctrl;
    struct pinctrl		*pinctrl;
    struct pinctrl_state	*state_default;

    struct gpio_data	hwen_pin;
    struct gpio_data	strobe_pin;
    struct gpio_data	torch_pin;
    struct gpio_data	tx_pin;

    unsigned int		tt_flag;
    unsigned int		torch_ramp_duration;
    unsigned int		flash_timeout;
    unsigned int		torch_override;
    unsigned int		flash_override;

    unsigned int		led_fault_detect;
    unsigned int		boost_mode;
    unsigned int		boost_freq;
    unsigned int		boost_climit;

    //sysfs
    unsigned int		read_reg;
    unsigned int		write_reg;

    //unused
    unsigned int		enable_ntc;

    struct led_data		leds[ID_MAX];
};


//Thease default value is from spec.
static const struct reg_default default_reg_setting[LM3644_REG_FLAG1] =  //FLAG1 reg is the next one behind TEMP reg.
{
    [0]			= {0,			0x00},
    [LM3644_REG_EN]		= {LM3644_REG_EN,	0x80},
    [LM3644_REG_IVFM]	= {LM3644_REG_IVFM,	0x01},
    [LM3644_REG_FLASH1_BR]	= {LM3644_REG_FLASH1_BR,0xBF},
    [LM3644_REG_FLASH2_BR]	= {LM3644_REG_FLASH2_BR,0x3F},
    [LM3644_REG_TORCH1_BR]	= {LM3644_REG_TORCH1_BR,0xBF},
    [LM3644_REG_TORCH2_BR]	= {LM3644_REG_TORCH2_BR,0x3F},
    [LM3644_REG_BOOST]	= {LM3644_REG_BOOST,	0x09},
    [LM3644_REG_TIME]	= {LM3644_REG_TIME,	0x1A},
    [LM3644_REG_TEMP]	= {LM3644_REG_TEMP,	0x08},
};

static const struct regmap_config lm3644_regmap =
{
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = LM3644_REG_LAST_FLASH,
    .reg_defaults = default_reg_setting,
    .num_reg_defaults = ARRAY_SIZE(default_reg_setting),
};

static const char default_led_name[ID_MAX][8] =
{
    "flash0",
    "flash1",
    "torch0",
    "torch1",
    "switch0"
};

struct led_data *led_tmp;

static int lm3644_suspend(struct lm3644_chip *chip)
{
    int ret;

    flush_workqueue(chip->leds[ID_SWITCH0].lm3644_queue);

    gpio_direction_output(chip->hwen_pin.gpio, 0);

    ret = camera_io_release(&(chip->io_master_info));
    if (ret < 0)
        pr_err("[LM3644] release camera io  ret: %d",ret);

    ret = regulator_disable(chip->vdd);
    if (ret < 0)
        pr_err("[LM3644] disable cci regulator ret: %d",ret);

    return 0;
}

static int lm3644_resume(struct lm3644_chip *chip)
{
    int ret;

    ret = regulator_enable(chip->vdd);
    if (ret < 0)
        pr_err("%s regulator_enable failed: rc: %d", __func__, ret);

    ret = camera_io_init(&(chip->io_master_info));
    if (ret < 0)
        pr_err("%s cci_release failed: rc: %d", __func__, ret);

    gpio_direction_output(chip->hwen_pin.gpio, 1);

    return 0;
}

static int lm3644_i2c_read(struct camera_io_master *io_master_info, unsigned int reg,
                           unsigned int *val)
{
    int rc = 0;
    rc = camera_io_dev_read( io_master_info, reg, val, CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);

    return 0;
}

static int lm3644_i2c_write(struct camera_io_master *io_master_info, unsigned int reg,
                            unsigned int val)
{
    int rc = 0;
    struct cam_sensor_i2c_reg_setting write_setting;
    struct cam_sensor_i2c_reg_array i2c_reg_array;

    i2c_reg_array.reg_addr = reg;
    i2c_reg_array.reg_data = val;
    i2c_reg_array.delay = 1;
    write_setting.reg_setting = &i2c_reg_array;
    write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    write_setting.size = 1;
    write_setting.delay = 0;

    rc = camera_io_dev_write( io_master_info, &write_setting);
    return 0;
}


static int lm3644_reg_settings(struct lm3644_chip *chip)
{
    int ret, i;
    unsigned int reg, data, index;

    if (!chip)
        return -ENODEV;

    ret = lm3644_i2c_read(&(chip->io_master_info), LM3644_REG_ID, &data);
    if (ret < 0)
        return -EIO;

    if ((data & REG_ID_SILICON_REV_MASK) == REV_LM3644TT)
        chip->tt_flag = 1;
    else
        chip->tt_flag = 0;

    //Get power-on/reset register settings from device
    memcpy(chip->base_setting, default_reg_setting, sizeof(struct reg_default) * ARRAY_SIZE(chip->base_setting));
    for (i = 0; i < ARRAY_SIZE(chip->base_setting); i++)
    {
        reg = chip->base_setting[i].reg;
        data = 0,
        ret = lm3644_i2c_read(&(chip->io_master_info), reg, &data);
        if (ret < 0 )
        {
            dev_err(chip->dev, "read 0x%02X fail: %d\n", reg, ret);
            return ret;
        }
        chip->base_setting[i].def = data;
    }

    //update LM3644_REG_EN register
    index = LM3644_REG_EN;
    reg = chip->base_setting[index].reg;
    data = chip->base_setting[index].def;
    data &= ~REG_EN_STB_PIN_MASK;
    if (chip->strobe_pin.use_pin == GPIO_REQUESTED)
        data |= REG_EN_STB_PIN_ENABLE;
    else
        data |= REG_EN_STB_PIN_DISABLE;

    data &= ~REG_EN_TORCH_PIN_MASK;
    if (chip->torch_pin.use_pin == GPIO_REQUESTED)
        data |= REG_EN_TORCH_PIN_ENABLE;
    else
        data |= REG_EN_TORCH_PIN_DISABLE;

    data &= ~REG_EN_TX_PIN_MASK;
    if (chip->tx_pin.use_pin == GPIO_REQUESTED)
        data |= REG_EN_TX_PIN_ENABLE;
    else
        data |= REG_EN_TX_PIN_DISABLE;

    ret = lm3644_i2c_write(&(chip->io_master_info), reg, data);
    if (ret < 0)
    {
        dev_err(chip->dev, "write reg 0x%02X fail:%d\n", reg, ret);
        return ret;
    }
    chip->base_setting[index].def = data;


    //update LM3644_REG_TIME register
    index = LM3644_REG_TIME;
    reg = chip->base_setting[index].reg;
    data = chip->base_setting[index].def;
    data &= ~REG_TIME_TORCH_RAMP_MASK;
    for (i = 0; i < NUM_OF_TORCH_RAMP_TIME; i++)
    {
        if (chip->torch_ramp_duration < torch_ramp_time_ms[i])
            break;
    }
    if (i > 0)
        i--;
    data |= REG_TIME_TORCH_RAMP(i);

    data &= ~REG_TIME_FLASH_TIMEOUT_MASK;
    if (chip->tt_flag)
    {
        for (i = 0; i < NUM_OF_TT_TIMEOUT; i++)
        {
            if (chip->flash_timeout < tt_flash_timeout_ms[i])
                break;
        }
    }
    else
    {
        for (i = 0; i < NUM_OF_TIMEOUT; i++)
        {
            if (chip->flash_timeout < flash_timeout_ms[i])
                break;
        }
    }
    if (i > 0)
        i--;
    data |= REG_TIME_FLASH_TIMEOUT(i);

    ret = lm3644_i2c_write(&(chip->io_master_info), reg, data);
    if (ret < 0)
    {
        dev_err(chip->dev, "write reg 0x%02X fail:%d\n", reg, ret);
        return ret;
    }
    chip->base_setting[index].def = data;

    //update LM3644_REG_TORCH1_BR register
    index = LM3644_REG_TORCH1_BR;
    reg = chip->base_setting[index].reg;
    data = chip->base_setting[index].def;
    data &= ~REG_TORCH1_BR_OVERRIDE_MASK;
    if (chip->torch_override)
        data |= REG_TORCH1_BR_OVERRIDE_ENABLE;
    else
        data |= REG_TORCH1_BR_OVERRIDE_DISABLE;

    ret = lm3644_i2c_write(&(chip->io_master_info), reg, data);
    if (ret < 0)
    {
        dev_err(chip->dev, "write reg 0x%02X fail:%d\n", reg, ret);
        return ret;
    }
    chip->base_setting[index].def = data;

    //update LM3644_REG_FLASH1_BR register
    index = LM3644_REG_FLASH1_BR;
    reg = chip->base_setting[index].reg;
    data = chip->base_setting[index].def;
    data &= ~REG_FLASH1_BR_OVERRIDE_MASK;
    if (chip->torch_override)
        data |= REG_FLASH1_BR_OVERRIDE_ENABLE;
    else
        data |= REG_FLASH1_BR_OVERRIDE_DISABLE;

    ret = lm3644_i2c_write(&(chip->io_master_info), reg, data);
    if (ret < 0)
    {
        dev_err(chip->dev, "write reg 0x%02X fail:%d\n", reg, ret);
        return ret;
    }
    chip->base_setting[index].def = data;

    //update LM3644_REG_BOOST register
    index = LM3644_REG_BOOST;
    reg = chip->base_setting[index].reg;
    data = chip->base_setting[index].def;
    data &= ~REG_BOOST_LED_SHORT_MASK;
    if (chip->led_fault_detect == LED_DETECT_ENABLE)
        data |= REG_BOOST_LED_SHORT_ENABLE;
    else
        data |= REG_BOOST_LED_SHORT_DISABLE;

    data &= ~REG_BOOST_MODE_MASK;
    if (chip->boost_mode == BOOST_NORMAL)
        data |= REG_BOOST_NORMAL_MODE;
    else
        data |= REG_BOOST_PASS_MODE;

    data &= ~REG_BOOST_FREQ_SEL_MASK;
    if (chip->boost_freq == BOOST_FREQ_2MHZ)
        data |= REG_BOOST_FREQ_2MHZ;
    else
        data |= REG_BOOST_FREQ_4MHZ;

    data &= ~REG_BOOST_CLIMIT_MASK;
    if (chip->boost_climit == BOOST_CLIMIT_2P8A)
        data |= REG_BOOST_CLIMIT_2P8A;
    else
        data |= REG_BOOST_CLIMIT_1P9A;

    ret = lm3644_i2c_write(&(chip->io_master_info), reg, data);
    if (ret < 0)
    {
        dev_err(chip->dev, "write reg 0x%02X fail:%d\n", reg, ret);
        return ret;
    }
    chip->base_setting[index].def = data;

    return 0;
}

static void lm3644_read_flag(struct lm3644_chip *chip)
{

    int ret;
    unsigned int flag1, flag2;

    ret = lm3644_i2c_read(&(chip->io_master_info), LM3644_REG_FLAG1, &flag1);
    ret |= lm3644_i2c_read(&(chip->io_master_info), LM3644_REG_FLAG2, &flag2);

    if (ret < 0)
        dev_err(chip->dev, "i2c access fail.\n");

    dev_info(chip->dev, "[flag1] 0x%02X, [flag2] 0x%02X\n", flag1, flag2);
}

static unsigned int calculate_current_to_regval(struct lm3644_chip *chip, unsigned int id, unsigned int brightness)
{
    unsigned int base = 0, multiple = 0;

    brightness *= 1000;
    switch (id)
    {
    case ID_FLASH0:
    case ID_FLASH1:
        base = 10900; //10.9mA
        multiple = 11725; //11.725mA
        if (brightness > FLASH_MAX_CUR * 1000)
            brightness = FLASH_MAX_CUR * 1000;
        if (brightness < base)
            brightness = base;
        break;
    case ID_TORCH0:
    case ID_TORCH1:
        if (chip->tt_flag)
        {
            base = 1954; //1.954mA
            multiple = 2800; //2.8mA
            if (brightness > TORCH_MAX_CUR_TT * 1000)
                brightness = TORCH_MAX_CUR_TT * 1000;
        }
        else
        {
            base = 977; //0.977mA
            multiple = 1400; //1.4mA
            if (brightness > TORCH_MAX_CUR * 1000)
                brightness = TORCH_MAX_CUR * 1000;
        }
        if (brightness < base)
            brightness = base;
        break;
    default:
        dev_err(chip->dev, "unexpecteed id:%d", id);
        return 0;
    };

    return (brightness - base) / multiple;
}

static int lm3644_update_brightness_level(struct lm3644_chip *chip, unsigned int id, unsigned int brightness)
{
    unsigned int reg, data, mask, reg_val;
    int ret;

    if (!chip)
        return -ENODEV;
    mutex_lock(&chip->lm3644_mutex);

    reg_val = calculate_current_to_regval(chip, id, brightness);
    dev_dbg(chip->dev, "Change led[%d] current %dmA to regval %d\n", id, brightness, reg_val);

    switch (id)
    {
    case ID_FLASH0:
        reg = LM3644_REG_FLASH1_BR;
        mask = REG_FLASH1_BR_LVL_MASK;
        data = REG_FLASH1_BR_LVL(reg_val);
        break;
    case ID_FLASH1:
        reg = LM3644_REG_FLASH2_BR;
        mask = REG_FLASH2_BR_LVL_MASK;
        data = REG_FLASH2_BR_LVL(reg_val);
        break;
    case ID_TORCH0:
        reg = LM3644_REG_TORCH1_BR;
        mask = REG_TORCH1_BR_LVL_MASK;
        data = REG_TORCH1_BR_LVL(reg_val);
        break;
    case ID_TORCH1:
        reg = LM3644_REG_TORCH2_BR;
        mask = REG_TORCH2_BR_LVL_MASK;
        data = REG_TORCH2_BR_LVL(reg_val);
        break;
    default:
        dev_err(chip->dev, "Wrong LED-ID:%d\n", id);
        return -EINVAL;
    }


    ret = lm3644_i2c_write(&(chip->io_master_info), reg, data);
    if (ret < 0)
    {
        dev_err(chip->dev, "write reg 0x%02X fail:%d\n", reg, ret);
        pr_err("[LM3644] write reg 0x%02X fail:%d\n", reg, ret);
        return ret;
    }

    //if (regmap_update_bits(chip->regmap, reg, mask, data)) {
    //	dev_err(chip->dev, "i2c access fail.\n");
    //	return -EIO;
    //}
    mutex_unlock(&chip->lm3644_mutex);

    return 0;
}

static int lm3644_led_enable(struct lm3644_chip *chip, unsigned int id, int enable)
{
    unsigned int i, data, mode, led, use_strobe, use_torch, gpio, on, flash_on, torch_on;

    if (!chip)
        return -ENODEV;

    mode = MODE_STANDBY;
    led = gpio = on = flash_on = torch_on = 0;
    if (lm3644_i2c_read(&(chip->io_master_info), LM3644_REG_EN, &data))
    {
        dev_err(chip->dev, "get LM3644_REG_EN fail\n");
        return -EIO;
    }

    use_strobe = (data & REG_EN_STB_PIN_MASK) && chip->strobe_pin.use_pin;
    use_torch = (data & REG_EN_TORCH_PIN_MASK) && chip->torch_pin.use_pin;

    if (enable)
    {
        //find flash led on status first
        for (on = 0, i = ID_FLASH0; i < ID_TORCH0; i++)
        {
            if (chip->leds[i].on)
            {
                on |= (1 << (i%2));
                flash_on = 1;
            }
        }

        if (!flash_on)   //if no flash, find torch led on status
        {
            for (on = 0, i = ID_TORCH0; i < ID_SWITCH0; i++)
            {
                if (chip->leds[i].on)
                {
                    on |= (1 << (i%2));
                    torch_on = 1;
                }
            }
        }
    }
    //after all, based on swtich led mask to filter the control leds
    on &= chip->leds[id].mask;

    if (on & LED1)
        led |= REG_EN_LED1_ON;
    else
        led |= REG_EN_LED1_OFF;

    if (on & LED2)
        led |= REG_EN_LED2_ON;
    else
        led |= REG_EN_LED2_OFF;
    data &= ~(REG_EN_LED1_MASK | REG_EN_LED2_MASK);
    data |= led;

    if (flash_on)
    {
        if (!use_strobe)
            mode = MODE_FLASH;
        else
            gpio = chip->strobe_pin.gpio;
    }
    else if (torch_on)
    {
        if (!use_torch)
            mode = MODE_TORCH;
        else
            gpio = chip->torch_pin.gpio;
    }
    data &= ~REG_EN_MODE_MASK;
    data |= REG_EN_MODE(mode);

    if (lm3644_i2c_write(&(chip->io_master_info), LM3644_REG_EN, data))
    {
        dev_err(chip->dev, "Write LM3644_REG_EN fail\n");
        return -EIO;
    }

    if (enable)
    {
        if (gpio)
        {
            gpio_set_value(gpio, 1);
        }
    }
    else
    {
        if (use_strobe)
            gpio_set_value(chip->strobe_pin.gpio, 0);
        if (use_torch)
            gpio_set_value(chip->torch_pin.gpio, 0);
    }

    dev_dbg(chip->dev, "ledon:%d, flash:%d, torch:%d (reg:0x%02X)\n", on, flash_on, torch_on, data);
    return 0;
}

static void lm3644_led_set_work(struct work_struct *work)
{
   // struct led_data *led = container_of(work, struct led_data, set_work);
    struct led_data *led = led_tmp;
    struct lm3644_chip *chip = container_of(led, struct lm3644_chip, leds[led->id]);
    int i;
    unsigned int brightness[ID_MAX];

    if (!led || !chip)
        return;

    brightness[ID_FLASH0] = chip->leds[ID_FLASH1].work_brightness;
    brightness[ID_FLASH1] = chip->leds[ID_FLASH0].work_brightness;
    brightness[ID_TORCH0] = chip->leds[ID_TORCH1].work_brightness;
    brightness[ID_TORCH1] = chip->leds[ID_TORCH0].work_brightness;
    brightness[ID_SWITCH0] = chip->leds[ID_SWITCH0].work_brightness;

    //mutex_unlock(&chip->lm3644_mutex);

    lm3644_reg_settings(chip);
    for (i = 0; i < ID_SWITCH0;i++)
    {
        if (brightness[i])
        {
            lm3644_update_brightness_level(chip, i, brightness[i]);
            chip->leds[i].on = 1;
        }
        else
        {
            chip->leds[i].on = 0;
        }
    }


    if (led->id > ID_TORCH1)   //switch to handle enable parts
    {
        lm3644_led_enable(chip, led->id, !!brightness[led->id]);
        lm3644_read_flag(chip);
    }

    return;
}

#define WORK_QUEUE_NAME "lm3644_event"
static DECLARE_WORK(lm3644_work, lm3644_led_set_work);

static void lm3644_leds_brightness_set(struct led_classdev *cdev, enum led_brightness brightness)
{
    struct led_data *led = container_of(cdev, struct led_data, cdev);
    struct lm3644_chip *chip = container_of(led, struct lm3644_chip, leds[led->id]);

    if (!led || !chip)
        return;

    brightness = min(brightness, cdev->max_brightness);

    chip->leds[led->id].work_brightness = brightness;
    if (led->id == ID_SWITCH0)
    {
    	led_tmp = &chip->leds[4];
        queue_work(chip->leds[led->id].lm3644_queue, &lm3644_work);
    }

    return;
}

static void lm3644_led_unregister(struct lm3644_chip *chip)
{
    int i;

    if (!chip)
        return;

    for (i = 0; i < ID_MAX; i++)
    {
        if (chip->leds[i].status == LED_REGISTER_DONE)
        {
            led_classdev_unregister(&chip->leds[i].cdev);
            chip->leds[i].status = LED_INVALID;
        }
    }
}

static int lm3644_led_register(struct lm3644_chip *chip)
{
    int i, ret;

    if (!chip)
        return -ENODEV;

    for (i = 0; i < ID_MAX; i++)
    {
        if (!chip->leds[i].status)
            continue;

        chip->leds[i].cdev.brightness = 0;
        chip->leds[i].cdev.brightness_set = lm3644_leds_brightness_set;
        ret = led_classdev_register(chip->dev, &chip->leds[i].cdev);
        if (ret < 0)
        {
            lm3644_led_unregister(chip);
            pr_err("[LM3644] lm3644_led_register( ret: %d ) \n",ret);
            return 0;
        }
        chip->leds[i].status = LED_REGISTER_DONE;
        //INIT_WORK(&chip->leds[i].set_work, lm3644_led_set_work);
        chip->leds[i].work_brightness = 0;
    }
    chip->leds[ID_SWITCH0].lm3644_queue = create_singlethread_workqueue(WORK_QUEUE_NAME);
    return 0;
}

/* HACK code only for cam_flash_core.c */
int lm3644_flash_led_prepare(struct led_trigger *trig, int options, int *max_current)
{
    struct led_classdev *led_cdev;
    struct led_data *led;
    struct lm3644_chip *chip ;

    if (!trig)
    {
        pr_err("Invalid led_trigger provided\n");
        return -EINVAL;
    }

    led_cdev = trigger_to_lcdev(trig);
    led  = container_of(led_cdev, struct led_data, cdev);
    chip = container_of(led, struct lm3644_chip, leds[led->id]);
    if (!led_cdev)
    {
        pr_err("Invalid led_cdev in trigger %s\n", trig->name);
        return -EINVAL;
    }

    if (!(options & LM3644_PREPARE_OPTIONS_MASK))
    {
        pr_err("Invalid options %d\n", options);
        return -EINVAL;
    }

    if (options & LM3644_ENABLE_REGULATOR)
    {
        /* do nothing */
        lm3644_resume(chip);
    }

    if (options & LM3644_DISABLE_REGULATOR)
    {
        /* do nothing */
        lm3644_suspend(chip);
    }

    if (options & LM3644_QUERY_MAX_CURRENT)
    {
        /* do nothing but just return max flash current value */
        *max_current = FLASH_MAX_CUR;
    }

    return 0;
}

static ssize_t lm3644_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct lm3644_chip *chip = i2c_get_clientdata(client);
    int i = 0, ret = 0, len = 0;
    unsigned int data = 0;

    dev_dbg(chip->dev, "%s\n", __func__);
    if(!chip)
        return 0;

    if (chip->read_reg != 0xff)
    {
        ret = lm3644_i2c_read(&(chip->io_master_info), chip->read_reg, &data);
        if (ret)
            len = sprintf(buf, "Read 0x%02X fail: %d\n", chip->read_reg, ret);
        else
            len = sprintf(buf, "REG 0x%02X: 0x%02X\n", chip->read_reg, (data & 0xFF));
    }
    else
    {
        len = sprintf(buf, "Read all registers (0x00 ~ 0x%02X):\n", LM3644_REG_LAST_FLASH);
        for (i = 0; i <= LM3644_REG_LAST_FLASH; i++, data = 0)
        {
            ret = lm3644_i2c_read(&(chip->io_master_info), i, &data);
            if (ret)
                len += sprintf(buf + len, "Read 0x%02X fail: %d\n", i, ret);
            else
                len += sprintf(buf + len, "0x%02X ", (data & 0xFF));
        }
        len += sprintf(buf + len, "\n");
    }

    return len;
}

//Format:$register (0xff means read all)
static ssize_t lm3644_read_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct lm3644_chip *chip = i2c_get_clientdata(client);
    unsigned int reg = 0;

    dev_dbg(chip->dev, "%s\n", __func__);
    if (!chip)
        return size;

    sscanf(buf, "%X", &reg);
    reg &= 0xFF;
    if ((reg != 0xFF) && (reg > LM3644_REG_LAST_FLASH))
    {
        dev_err(chip->dev, "Reg 0x%2X is out of range\n", reg);
        return -EINVAL;
    }

    chip->read_reg = reg;
    return size;
}

static ssize_t lm3644_write_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct lm3644_chip *chip = i2c_get_clientdata(client);
    int ret = 0;
    unsigned int data = 0;

    dev_dbg(chip->dev, "%s\n", __func__);
    if(!chip)
        return 0;

    ret = lm3644_i2c_read(&(chip->io_master_info), chip->write_reg, &data);
    if (ret)
        return sprintf(buf, "Read 0x%02X fail: %d\n", chip->write_reg, ret);

    return sprintf(buf, "REG 0x%02X: 0x%02X\n", chip->write_reg, (data & 0xFF));
}

//Format:$register $data...
static ssize_t lm3644_write_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct lm3644_chip *chip = i2c_get_clientdata(client);
    unsigned int reg = 0, data = 0;
    int ret = 0;

    dev_dbg(chip->dev, "%s\n", __func__);
    if (!chip)
        return size;

    sscanf(buf, "%X %X", &reg, &data);
    reg &= 0xFF;
    data &= 0xFF;
    if (reg > LM3644_REG_LAST_FLASH)
    {
        dev_err(chip->dev, "Reg 0x%2X is out of range\n", reg);
        return -EINVAL;
    }

    chip->write_reg = reg;
    ret = lm3644_i2c_write(&(chip->io_master_info), reg, data);
    if (ret)
        dev_err(chip->dev, "Write reg 0x%02X as 0x%02X fail: %d\n", reg, data, ret);

    return size;
}

//Format: input any to run sw-reset and then restore the probed register value
static ssize_t lm3644_swreset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct lm3644_chip *chip = i2c_get_clientdata(client);
    unsigned int reg = 0, data = 0;
    int ret = 0, i = 0;

    dev_dbg(chip->dev, "%s\n", __func__);
    if (!chip)
        return size;

    ret = lm3644_i2c_write(&(chip->io_master_info), LM3644_REG_BOOST, REG_BOOST_SOFT_RESET);
    if (ret < 0)
    {
        dev_err(chip->dev, "write reg 0x%02X fail:%d\n", reg, ret);
        pr_err("[LM3644] write reg 0x%02X fail:%d\n", reg, ret);
        return ret;
    }

    //ret = regmap_update_bits(chip->regmap, LM3644_REG_BOOST, REG_BOOST_SOFT_RESET_MASK, REG_BOOST_SOFT_RESET);
    //if (ret)
    //	dev_err(chip->dev, "Write reset bit fail: %d\n", ret);

    msleep(100); //100ms

    for (i = 0; i < ARRAY_SIZE(chip->base_setting); i++)
    {
        reg = chip->base_setting[i].reg;
        data = chip->base_setting[i].def;
        dev_dbg(chip->dev, "Write 0x%02X as 0x%02X\n", reg, data);
        ret = lm3644_i2c_write(&(chip->io_master_info), reg, data);
        if (ret)
            dev_err(chip->dev, "Write reg 0x%02X as 0x%02X fail: %d\n", reg, data, ret);
    }

    return size;
}

static void lm3644_torch(struct lm3644_chip* chip, int id, unsigned int _brightness)
{
    unsigned int brightness[ID_MAX], i;

    lm3644_resume(chip);
    for (i = 0; i < ID_MAX; i++)
    {
        brightness[i] = 0;
    }

    brightness[id] = _brightness;
    if (_brightness)
    {
        brightness[ID_SWITCH0] = 1;
    } 
    else
    {
        brightness[ID_SWITCH0] = 0;
    }

    for (i = 0; i < ID_SWITCH0; i++)
    {
        lm3644_update_brightness_level(chip, i, brightness[i]);
        chip->leds[i].on = !!brightness[i];
    }

    lm3644_led_enable(chip, ID_SWITCH0, !!brightness[ID_SWITCH0]);
    lm3644_read_flag(chip);

    if (!_brightness)
    {
        lm3644_suspend(chip);
    }
}

static ssize_t lm3644_torch0_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct lm3644_chip *chip = i2c_get_clientdata(client);
    unsigned int brightness = 0;


    dev_dbg(chip->dev, "%s\n", __func__);
    if (!chip)
        return size;

    sscanf(buf, "%X", &brightness);

    lm3644_torch(chip, ID_TORCH1, brightness);
    //lm3644_reg_dump(chip);
    return size;
}

static ssize_t lm3644_torch1_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct lm3644_chip *chip = i2c_get_clientdata(client);
    unsigned int brightness = 0;


    dev_dbg(chip->dev, "%s\n", __func__);
    if (!chip)
        return size;

    sscanf(buf, "%X", &brightness);

    lm3644_torch(chip, ID_TORCH0, brightness);
    //lm3644_reg_dump(chip);
    return size;
}

static DEVICE_ATTR(read, 0644, lm3644_read_show, lm3644_read_store);
static DEVICE_ATTR(write, 0644, lm3644_write_show, lm3644_write_store);
static DEVICE_ATTR(swreset, 0200, NULL, lm3644_swreset_store);
static DEVICE_ATTR(torch0, 0200, NULL, lm3644_torch0_store);
static DEVICE_ATTR(torch1, 0200, NULL, lm3644_torch1_store);

static struct attribute *lm3644_debug_attr[] =
{
    &dev_attr_read.attr,
    &dev_attr_write.attr,
    &dev_attr_swreset.attr,
    &dev_attr_torch0.attr,
    &dev_attr_torch1.attr,
    NULL
};

const struct attribute_group lm3644_debug_attr_group =
{
    .name = "debug",
    .attrs = lm3644_debug_attr,
};

static int lm3644_hw_deinit(struct lm3644_chip *chip)
{
    if (!chip)
        return -ENODEV;

    if (chip->hwen_pin.use_pin == GPIO_REQUESTED)
    {
        gpio_direction_output(chip->hwen_pin.gpio, 0);
        gpio_free(chip->hwen_pin.gpio);
        chip->hwen_pin.use_pin = GPIO_INVALID;
    }
    if (chip->hwen_pin.use_pin == GPIO_REQUESTED)
    {
        gpio_direction_output(chip->strobe_pin.gpio, 0);
        gpio_free(chip->strobe_pin.gpio);
        chip->strobe_pin.use_pin = GPIO_INVALID;
    }
    if (chip->torch_pin.use_pin == GPIO_REQUESTED)
    {
        gpio_direction_output(chip->torch_pin.gpio, 0);
        gpio_free(chip->torch_pin.gpio);
        chip->torch_pin.use_pin = GPIO_INVALID;
    }
    if (chip->tx_pin.use_pin == GPIO_REQUESTED)
    {
        gpio_direction_output(chip->tx_pin.gpio, 0);
        gpio_free(chip->tx_pin.gpio);
        chip->tx_pin.use_pin = GPIO_INVALID;
    }

    return 0;
}

static int lm3644_hw_init(struct lm3644_chip *chip)
{
    int ret = 0;

    if (!chip)
        return -ENODEV;

    if (chip->use_pinctrl && chip->state_default)
        pinctrl_select_state(chip->pinctrl, chip->state_default);

    if (chip->hwen_pin.use_pin == GPIO_USED)
    {
        ret = gpio_request(chip->hwen_pin.gpio, "hwen-pin");
        if (ret < 0)
        {
            dev_err(chip->dev, "reqest gpio-%d fail:%d\n", chip->hwen_pin.gpio, ret);
            return -EIO;
        }
        else
        {
            chip->hwen_pin.use_pin = GPIO_REQUESTED;
            gpio_direction_output(chip->hwen_pin.gpio, 1);
        }
    }

    if (chip->strobe_pin.use_pin == GPIO_USED)
    {
        ret = gpio_request(chip->strobe_pin.gpio, "strobe-pin");
        if (ret < 0)
        {
            dev_err(chip->dev, "reqest gpio-%d fail:%d\n", chip->strobe_pin.gpio, ret);
            chip->strobe_pin.use_pin = GPIO_INVALID;
        }
        else
        {
            chip->strobe_pin.use_pin = GPIO_REQUESTED;
            gpio_direction_output(chip->strobe_pin.gpio, 0);
        }
    }

    if (chip->torch_pin.use_pin == GPIO_USED)
    {
        ret = gpio_request(chip->torch_pin.gpio, "torch-pin");
        if (ret < 0)
        {
            dev_err(chip->dev, "reqest gpio-%d fail:%d\n", chip->torch_pin.gpio, ret);
            chip->torch_pin.use_pin = GPIO_INVALID;
        }
        else
        {
            chip->torch_pin.use_pin = GPIO_REQUESTED;
            gpio_direction_output(chip->torch_pin.gpio, 0);
        }
    }

    if (chip->tx_pin.use_pin == GPIO_USED)
    {
        ret = gpio_request(chip->tx_pin.gpio, "tx-pin");
        if (ret < 0)
        {
            dev_err(chip->dev, "reqest gpio-%d fail:%d\n", chip->tx_pin.gpio, ret);
            chip->tx_pin.use_pin = GPIO_INVALID;
        }
        else
        {
            chip->tx_pin.use_pin = GPIO_REQUESTED;
            gpio_direction_output(chip->tx_pin.gpio, 0);
        }
    }

    return 0;
}

static int lm3644_parse_common_dts(struct lm3644_chip *chip, struct platform_device *client)
{
    struct device_node *node;
    int ret;

    if (!chip || !client)
        return -ENODEV;

    node = client->dev.of_node;
    if (!node)
    {
        dev_err(&client->dev, "No flash LED nodes defined\n");
        return -ENODEV;
    }

    chip->pinctrl = devm_pinctrl_get(&client->dev);
    if (IS_ERR_OR_NULL(chip->pinctrl))
        dev_err(&client->dev, "Failed to get pinctrl:%ld\n", PTR_ERR(chip->pinctrl));
    else
    {
        chip->use_pinctrl = 1;
        chip->state_default = pinctrl_lookup_state(chip->pinctrl, "default");
        if (IS_ERR(chip->state_default))
        {
            dev_err(&client->dev, "Could not get pinctrl default state:%ld\n", PTR_ERR(chip->state_default));
            chip->state_default = NULL;
            chip->use_pinctrl = 0;
        }
    }

    ret = of_get_named_gpio(node, "gpio-hwen", 0);
    if (ret >= 0)
    {
        chip->hwen_pin.use_pin = GPIO_USED;
        chip->hwen_pin.gpio = ret;
    }

    ret = of_get_named_gpio(node, "gpio-strobe", 0);
    if (ret >= 0)
    {
        chip->strobe_pin.use_pin = GPIO_USED;
        chip->strobe_pin.gpio = ret;
    }

    ret = of_get_named_gpio(node, "gpio-torch", 0);
    if (ret >= 0)
    {
        chip->torch_pin.use_pin = GPIO_USED;
        chip->torch_pin.gpio = ret;
    }

    ret = of_get_named_gpio(node, "gpio-tx", 0);
    if (ret >= 0)
    {
        chip->tx_pin.use_pin = GPIO_USED;
        chip->tx_pin.gpio = ret;
    }

    ret = of_property_read_u32(node, "torch-ramp-time-ms", &chip->torch_ramp_duration);
    if (ret < 0)
    {
        dev_err(&client->dev, "Get torch-ramp-time-ms fail: %d\n", ret);
        chip->torch_ramp_duration = torch_ramp_time_ms[TORCH_RAMP_1MS];
    }

    ret = of_property_read_u32(node, "flash-timeout-ms", &chip->flash_timeout);
    if (ret < 0)
    {
        dev_err(&client->dev, "Get flash-timeout-ms fail: %d\n", ret);
        chip->flash_timeout = tt_flash_timeout_ms[TT_TIMEOUT_600MS];
    }

    chip->torch_override = (unsigned int)of_property_read_bool(node, "torch-override");
    chip->flash_override = (unsigned int)of_property_read_bool(node, "flash-override");

    ret = of_property_read_u32(node, "led-fault-detect", &chip->led_fault_detect);
    if (ret < 0)
    {
        dev_err(&client->dev, "Get led-fault-detect fail: %d\n", ret);
        chip->led_fault_detect = LED_DETECT_ENABLE;
    }

    ret = of_property_read_u32(node, "boost-mode", &chip->boost_mode);
    if (ret < 0)
    {
        dev_err(&client->dev, "Get boost-mode fail: %d\n", ret);
        chip->boost_mode = BOOST_NORMAL;
    }

    ret = of_property_read_u32(node, "boost-freq", &chip->boost_freq);
    if (ret < 0)
    {
        dev_err(&client->dev, "Get boost-freq fail: %d\n", ret);
        chip->boost_freq = BOOST_FREQ_2MHZ;
    }

    ret = of_property_read_u32(node, "boost-climit", &chip->boost_climit);
    if (ret < 0)
    {
        dev_err(&client->dev, "Get boost-climit fail: %d\n", ret);
        chip->boost_climit = BOOST_CLIMIT_2P8A;
    }

    //unused
    chip->enable_ntc = (unsigned int)of_property_read_bool(node, "enable-ntc");

    return 0;
}

static int lm3644_parse_leds_dts(struct lm3644_chip *chip, struct platform_device *client)
{
    struct device_node *node, *temp;
    int led_id;
    int ret;

    if (!chip || !client)
        return -ENODEV;

    node = client->dev.of_node;
    for_each_available_child_of_node(node, temp)
    {
        led_id = ID_MAX;
        ret = of_property_read_u32(temp, "id", &led_id);
        if (ret)
        {
            dev_err(&client->dev, "Get led id failed:%d\n", ret);
            return -EINVAL;
        }

        if (led_id >= ID_MAX)
        {
            dev_err(&client->dev, "id out of range:%d\n", led_id);
            return -EINVAL;
        }
        chip->leds[led_id].id = led_id;
        chip->leds[led_id].mask = 0;
        chip->leds[led_id].status = LED_IS_VALID;

        ret = of_property_read_string(temp, "led-name", &chip->leds[led_id].cdev.name);
        if (ret)
        {
            dev_err(&client->dev, "id:%d get led-name error, use default\n", led_id);
            chip->leds[led_id].cdev.name = default_led_name[led_id];
        }

        ret = of_property_read_string(temp, "led-trigger", &chip->leds[led_id].cdev.default_trigger);
        if (ret)
        {
            dev_err(&client->dev, "id:%d get led-trigger error, use default\n", led_id);
            chip->leds[led_id].cdev.default_trigger = default_led_name[led_id];
        }

        ret = of_property_read_u32(temp, "max-brightness", &chip->leds[led_id].cdev.max_brightness);
        if (ret)
        {
            dev_err(&client->dev, "id:%d get max-brightness error, use default\n", led_id);
            if (led_id == ID_FLASH0 || led_id == ID_FLASH1)
            {
                chip->leds[led_id].cdev.max_brightness = FLASH_MAX_CUR;
            }
            else if (led_id == ID_TORCH0 || led_id == ID_TORCH1)
            {
                chip->leds[led_id].cdev.max_brightness = TORCH_MAX_CUR_TT;
            }
            else
            {
                chip->leds[led_id].cdev.max_brightness = LED_FULL;
            }
        }

        if (led_id > ID_TORCH1)   //only switch node need to use
        {
            ret = of_property_read_u32(temp, "led-mask", &chip->leds[led_id].mask);
            if (ret)
            {
                dev_err(&client->dev, "get led-mask error, wrong switch parameter\n");
                return -EINVAL;
            }
        }
    }

    return 0;
}

int32_t lm3644_update_i2c_info(struct lm3644_chip *ctrl)
{
    int32_t rc = 0;

    ctrl->cci_i2c_master = MASTER_0;
    ctrl->io_master_info.cci_client->sid = 0x63;
    ctrl->io_master_info.cci_client->retries = 3;
    ctrl->io_master_info.cci_client->id_map = 0;
    ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_STANDARD_MODE;

    return rc;
}
static int lm3644_parse_i2c_regulator(struct device *dev)
{
    struct lm3644_chip *ctrl = dev_get_drvdata(dev);
    int rc = 0;
	/* Initialize mutex */
	mutex_init(&ctrl->lm3644_mutex);

    ctrl->vdd = devm_regulator_get(dev, "vdd");
    if (IS_ERR(ctrl->vdd))
    {
        ctrl->vdd = NULL;
        pr_err("[LM3644] unable to get vdd\n");
        rc = -ENOENT;
    }

    rc = regulator_enable(ctrl->vdd);
    if (rc < 0)
        pr_err("%s regulator_enable failed: rc: %d", __func__, rc);

    rc = camera_io_init(&(ctrl->io_master_info));
    if (rc < 0)
        pr_err("%s cci_release failed: rc: %d", __func__, rc);

    return rc;
}


static int lm3644_probe(struct platform_device *client)
{
    struct lm3644_chip *chip;
    int ret;

    chip = devm_kzalloc(&client->dev, sizeof(struct lm3644_chip), GFP_KERNEL);
    if (!chip)
        return -ENOMEM;

    if (cam_cci_get_subdev() == NULL)
    {
        pr_err("wait cci driver probe\n");
        return -EPROBE_DEFER;
    }

    /*fill in platform device*/
    chip->v4l2_dev_str.pdev = client;
    chip->soc_info.pdev = client;
    chip->soc_info.dev = &client->dev;
    chip->soc_info.dev_name = client->name;
    chip->io_master_info.master_type = CCI_MASTER;

    chip->io_master_info.cci_client = kzalloc(sizeof(
                                          struct cam_sensor_cci_client), GFP_KERNEL);
    if (!(chip->io_master_info.cci_client))
    {
        pr_err("cci_client alloc fail\n");
        return -ENOMEM;
    }
    platform_set_drvdata(client, chip);
    v4l2_set_subdevdata(&chip->v4l2_dev_str.sd, chip);
    dev_set_drvdata(&client->dev, chip);

    lm3644_update_i2c_info(chip);

    /* Fill platform device id*/
    client->id = chip->soc_info.index;

    ret = lm3644_parse_i2c_regulator(&(client->dev));
    if (ret)
        pr_err("[LM3644] i2c pull high fail !!!!!!!!!!");

    ret = lm3644_parse_common_dts(chip, client);
    if (ret)
        goto common_dts_fail;

    ret = lm3644_parse_leds_dts(chip, client);
    if (ret)
        goto leds_dts_fail;

    ret = lm3644_hw_init(chip);
    if (ret)
        goto hwinit_fail;

    ret = lm3644_reg_settings(chip);
    if (ret)
        goto regsetting_fail;

    ret = lm3644_led_register(chip);
    if (ret)
        goto ledreg_fail;

    /* device files register */
    ret = sysfs_create_group(&client->dev.kobj, &lm3644_debug_attr_group);
    if (ret)
        goto sysfs_fail;
    gpio_direction_output(chip->hwen_pin.gpio, 0);
    ret = camera_io_release(&(chip->io_master_info));
    if (ret)
      pr_err("[LM3644] release camera io  ret: %d",ret);
    ret = regulator_disable(chip->vdd);
    if (ret)
      pr_err("[LM3644] disable cci regulator ret: %d",ret);
    return 0;

sysfs_fail:
ledreg_fail:
    lm3644_led_unregister(chip);
regsetting_fail:
hwinit_fail:
    lm3644_hw_deinit(chip);
leds_dts_fail:
common_dts_fail:
    regmap_exit(chip->regmap);
//regmap_fail:
    kfree(chip);
    return ret;
}

static int lm3644_remove(struct platform_device *pdev)
{
    struct lm3644_chip *chip = platform_get_drvdata(pdev);
    if (!chip)
    {
        pr_err("lm3644 device is NULL");
        return 0;
    }
    mutex_destroy(&chip->lm3644_mutex);
    sysfs_remove_group(&chip->dev->kobj, &lm3644_debug_attr_group);
    lm3644_led_unregister(chip);
    lm3644_hw_deinit(chip);
    regmap_exit(chip->regmap);
    kfree(chip->io_master_info.cci_client);
    chip->io_master_info.cci_client = NULL;
    devm_kfree(&pdev->dev, chip);

    return 0;
}

static const struct of_device_id lm3644_driver_dt_match[] =
{
    {.compatible = "ti,lm3644"},
    {}
};

MODULE_DEVICE_TABLE(of, lm3644_driver_dt_match);

static struct platform_driver lm3644_platform_driver =
{
    .probe = lm3644_probe,
    .driver = {
        .name = LM3644_DEV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = lm3644_driver_dt_match,
    },
    .remove = lm3644_remove,
};

static int __init lm3644_init(void)
{
    int rc = 0;

    rc = platform_driver_register(&lm3644_platform_driver);

    return rc;
}

static void __exit lm3644_exit(void)
{
    platform_driver_unregister(&lm3644_platform_driver);
}

MODULE_DESCRIPTION("TI flash sensor");
MODULE_LICENSE("GPL");

module_init(lm3644_init);
module_exit(lm3644_exit);


