#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/file.h>
#include <linux/uaccess.h>

static char gyroinfo[64];

static int fih_gyro_proc_read_info(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", gyroinfo);
	return 0;
}

static int fih_gyro_proc_open_info(struct inode *inode, struct file *file)
{
	return single_open(file, fih_gyro_proc_read_info, NULL);
}

static const struct file_operations fih_gyro_fops_info = {
	.open    = fih_gyro_proc_open_info,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int fih_gyro_property(struct platform_device *pdev)
{
	int rc = 0;
	static const char *p_chr;

	p_chr = of_get_property(pdev->dev.of_node, "fih-gyro,gyroinfo", NULL);
	if (!p_chr) {
		pr_info("%s:%d, gyroinfo not specified\n", __func__, __LINE__);
	} else {
		strlcpy(gyroinfo, p_chr, sizeof(gyroinfo));
		pr_info("%s: gyroinfo = %s\n", __func__, gyroinfo);
	}

	return rc;
}

static int fih_gyro_probe(struct platform_device *pdev)
{
	int rc = 0;

	if (!pdev || !pdev->dev.of_node) {
		pr_err("%s: Unable to load device node\n", __func__);
		return -ENOTSUPP;
	}

	rc = fih_gyro_property(pdev);
	if (rc) {
		pr_err("%s Unable to set property\n", __func__);
		return rc;
	}

	//proc_mkdir("AllHWList", NULL);
	proc_create("AllHWList/gyroinfo", 0, NULL, &fih_gyro_fops_info);

	return rc;
}

static int fih_gyro_remove(struct platform_device *pdev)
{
	remove_proc_entry("AllHWList/gyroinfo", NULL);

	return 0;
}

static const struct of_device_id fih_gyro_dt_match[] = {
	{ .compatible = "fih_gyro" },
	{}
};
MODULE_DEVICE_TABLE(of, fih_gyro_dt_match);

static struct platform_driver fih_gyro_driver = {
	.probe = fih_gyro_probe,
	.remove = fih_gyro_remove,
	.shutdown = NULL,
	.driver = {
		.name = "fih_gyro",
		.of_match_table = fih_gyro_dt_match,
	},
};

static int __init fih_gyro_init(void)
{
	int ret;

	ret = platform_driver_register(&fih_gyro_driver);
	if (ret) {
		pr_err("%s: failed!\n", __func__);
		return ret;
	}

	return ret;
}
module_init(fih_gyro_init);

static void __exit fih_gyro_exit(void)
{
	platform_driver_unregister(&fih_gyro_driver);
}
module_exit(fih_gyro_exit);
