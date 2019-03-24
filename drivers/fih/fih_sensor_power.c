#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/fb.h>

#define SENSOR_VDD_CONTROL

struct regulator *vdd;


static  int sensor_power_probe(struct platform_device *pdev)
{
    int ret=0;

    pr_info("[%s]\n",__FUNCTION__);

#ifdef SENSOR_VDD_CONTROL
    vdd = regulator_get(&pdev->dev, "vdd");
    if (IS_ERR(vdd))
    {
        ret = PTR_ERR(vdd);
        pr_err("Regulator get failed(vdd) ret=%d\n", ret);
        return ret;
    }

    if (regulator_count_voltages(vdd) > 0)
    {
        ret = regulator_set_voltage(vdd, 1800000, 1800000);
        if (ret)
        {
            pr_err("regulator_count_voltages(vdd) failed  ret=%d\n", ret);
            return ret;
        }
    }

    ret = regulator_enable(vdd);
    if (ret)
    {
        pr_err("Regulator vdd enable failed ret=%d\n", ret);
        return ret;
    }

#endif

    return ret;
}

static int sensor_power_remove(struct platform_device *pdev)
{

#ifdef SENSOR_VDD_CONTROL
    regulator_disable(vdd);
#endif

    pr_info("[%s]\n",__FUNCTION__);
    return 0;
}

static const struct of_device_id sensor_power_device_of_match[]  =
{
    { .compatible = "fih,sensor_power", },
    {},
};

static struct platform_driver sensor_power_driver =
{
    .driver = {
        .name = "sensor_power",
        .owner = THIS_MODULE,
        .of_match_table = sensor_power_device_of_match,
    },
    .probe = sensor_power_probe,
    .remove = sensor_power_remove,
};

module_platform_driver(sensor_power_driver);
MODULE_DEVICE_TABLE(of, sensor_power_device_of_match);

MODULE_LICENSE("Proprietary");
