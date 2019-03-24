#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#define FIH_PINCTRL_STATE_DEFAULT       "default"
#define FIH_PINCTRL_STATE_SLEEP         "sleep"

struct fih_gpio {
	struct platform_device *pdev;
	struct pinctrl *gpio_pinctrl;
	struct pinctrl_state *pinctrl_state_default;
	struct pinctrl_state *pinctrl_state_sleep;
};

static int fih_gpio_pinctrl_init(struct fih_gpio *fih_gpio_dev)
{
	int ret = 0;

	/* Get pinctrl if target uses pinctrl */
	fih_gpio_dev->gpio_pinctrl = devm_pinctrl_get(&(fih_gpio_dev->pdev->dev));
	if (IS_ERR_OR_NULL(fih_gpio_dev->gpio_pinctrl)) {
		ret = PTR_ERR(fih_gpio_dev->gpio_pinctrl);
		dev_dbg(&fih_gpio_dev->pdev->dev,
				"Target does not use pinctrl %d\n", ret);
		goto err_pinctrl_get;
	}

	fih_gpio_dev->pinctrl_state_default
		= pinctrl_lookup_state(fih_gpio_dev->gpio_pinctrl,
				FIH_PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(fih_gpio_dev->pinctrl_state_default)) {
		ret = PTR_ERR(fih_gpio_dev->pinctrl_state_default);
		dev_err(&fih_gpio_dev->pdev->dev,
				"Can not lookup %s pinstate %d\n",
				FIH_PINCTRL_STATE_DEFAULT, ret);
		goto err_pinctrl_lookup;
	}

	fih_gpio_dev->pinctrl_state_sleep
		= pinctrl_lookup_state(fih_gpio_dev->gpio_pinctrl,
				FIH_PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(fih_gpio_dev->pinctrl_state_sleep)) {
		ret = PTR_ERR(fih_gpio_dev->pinctrl_state_sleep);
		dev_err(&fih_gpio_dev->pdev->dev,
				"Can not lookup %s pinstate %d\n",
				FIH_PINCTRL_STATE_SLEEP, ret);
		goto err_pinctrl_lookup;
	}
	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(fih_gpio_dev->gpio_pinctrl);
err_pinctrl_get:
	fih_gpio_dev->gpio_pinctrl = NULL;
	return ret;
}

//extern int get_poffchg_flag(void);
static int fih_gpio_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct fih_gpio *fih_gpio_dev;

	fih_gpio_dev = kmalloc(sizeof *(fih_gpio_dev), GFP_KERNEL);
	fih_gpio_dev->pdev = pdev;

	ret = fih_gpio_pinctrl_init(fih_gpio_dev);
	if (!ret && fih_gpio_dev->gpio_pinctrl) {
		pr_err("%s: CONFIG FIH PINCTRL DEFAULT\n", __func__);
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		ret = pinctrl_select_state(fih_gpio_dev->gpio_pinctrl,
				fih_gpio_dev->pinctrl_state_default);
		if (ret < 0) {
			pr_err("failed to select pin to default state");
		}
	}

#if 0
	/* Disable it! If sleep function still not ready. */
	if (get_poffchg_flag()) {
		if (!ret && fih_gpio_dev->gpio_pinctrl) {
			pr_info("%s: CONFIG FIH PINCTRL SLEEP\n", __func__);
			ret = pinctrl_select_state(fih_gpio_dev->gpio_pinctrl,
					fih_gpio_dev->pinctrl_state_sleep);
			if (ret < 0) {
				pr_err("failed to select pin to sleep state");
			}
		}
	}
#endif
	return 0;
}

static struct of_device_id fih_gpio_of_match[] = {
	{ .compatible = "fih-gpio", },
	{ },
};
MODULE_DEVICE_TABLE(of, fih_gpio_of_match);

static struct platform_driver fih_gpio_driver = {
	.probe	= fih_gpio_probe,
	.driver	= {
		.name	= "fih-gpio",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(fih_gpio_of_match),
	}
};

static int __init fih_gpio_init(void)
{
	return platform_driver_register(&fih_gpio_driver);
}

static void __exit fih_gpio_exit(void)
{
	platform_driver_unregister(&fih_gpio_driver);
}

pure_initcall(fih_gpio_init);
module_exit(fih_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("John Chen <JohnPHChen@fih-foxconn.com>");
MODULE_DESCRIPTION("Driver to setup pin control for all GPIOs");
