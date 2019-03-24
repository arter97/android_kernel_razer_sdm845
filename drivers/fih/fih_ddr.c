#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/file.h>
#include <linux/uaccess.h>

static char draminfo[32];  //"UNKNOWN_ LPDDR4x 4096MB"
static char dramtest[8];   //"none", "pass" or "fail"

static int fih_ddr_proc_read_info(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", draminfo);
	return 0;
}

static int fih_ddr_proc_open_info(struct inode *inode, struct file *file)
{
	return single_open(file, fih_ddr_proc_read_info, NULL);
}

static const struct file_operations fih_ddr_fops_info = {
	.open    = fih_ddr_proc_open_info,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int fih_ddr_proc_read_test(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", dramtest);
	return 0;
}

static int fih_ddr_proc_open_test(struct inode *inode, struct file *file)
{
	return single_open(file, fih_ddr_proc_read_test, NULL);
}

static const struct file_operations fih_ddr_fops_test = {
	.open    = fih_ddr_proc_open_test,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int fih_ddr_property(struct platform_device *pdev)
{
	int rc = 0;
	static const char *p_chr;

	p_chr = of_get_property(pdev->dev.of_node, "fih-ddr,draminfo", NULL);
	if (!p_chr) {
		pr_info("%s:%d, draminfo not specified\n", __func__, __LINE__);
	} else {
		strlcpy(draminfo, p_chr, sizeof(draminfo));
		pr_info("%s: draminfo = %s\n", __func__, draminfo);
	}

	p_chr = of_get_property(pdev->dev.of_node, "fih-ddr,dramtest", NULL);
	if (!p_chr) {
		pr_info("%s:%d, dramtest not specified\n", __func__, __LINE__);
	} else {
		if (0 == strcmp(p_chr, "fail")) {
			snprintf(dramtest, sizeof(dramtest), "1");  //fail
		} else {
			snprintf(dramtest, sizeof(dramtest), "0");  //pass
		}
		pr_info("%s: dramtest = %s\n", __func__, dramtest);
	}

	return rc;
}

static int fih_ddr_probe(struct platform_device *pdev)
{
	int rc = 0;

	if (!pdev || !pdev->dev.of_node) {
		pr_err("%s: Unable to load device node\n", __func__);
		return -ENOTSUPP;
	}

	rc = fih_ddr_property(pdev);
	if (rc) {
		pr_err("%s Unable to set property\n", __func__);
		return rc;
	}

	printk("BBox::UPD;101::%s\n", draminfo);

	proc_mkdir("AllHWList", NULL);
	proc_create("AllHWList/draminfo", 0, NULL, &fih_ddr_fops_info);
	proc_create("AllHWList/dramtest_result", 0, NULL, &fih_ddr_fops_test);

	return rc;
}

static int fih_ddr_remove(struct platform_device *pdev)
{
	remove_proc_entry ("AllHWList/dramtest_result", NULL);
	remove_proc_entry ("AllHWList/draminfo", NULL);

	return 0;
}

static const struct of_device_id fih_ddr_dt_match[] = {
	{.compatible = "fih_ddr"},
	{}
};
MODULE_DEVICE_TABLE(of, fih_ddr_dt_match);

static struct platform_driver fih_ddr_driver = {
	.probe = fih_ddr_probe,
	.remove = fih_ddr_remove,
	.shutdown = NULL,
	.driver = {
		.name = "fih_ddr",
		.of_match_table = fih_ddr_dt_match,
	},
};

static int __init fih_ddr_init(void)
{
	int ret;

	ret = platform_driver_register(&fih_ddr_driver);
	if (ret) {
		pr_err("%s: failed!\n", __func__);
		return ret;
	}

	return ret;
}
module_init(fih_ddr_init);

static void __exit fih_ddr_exit(void)
{
	platform_driver_unregister(&fih_ddr_driver);
}
module_exit(fih_ddr_exit);
