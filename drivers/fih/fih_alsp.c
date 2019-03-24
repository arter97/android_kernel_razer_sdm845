#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/file.h>
#include <linux/uaccess.h>

static char alspinfo[64];

static int fih_alsp_proc_read_info(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", alspinfo);
	return 0;
}

static int fih_alsp_proc_open_info(struct inode *inode, struct file *file)
{
	return single_open(file, fih_alsp_proc_read_info, NULL);
}

static const struct file_operations fih_alsp_fops_info = {
	.open    = fih_alsp_proc_open_info,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int fih_alsp_property(struct platform_device *pdev)
{
	int rc = 0;
	static const char *p_chr;

	p_chr = of_get_property(pdev->dev.of_node, "fih-alsp,alspinfo", NULL);
	if (!p_chr) {
		pr_info("%s:%d, alspinfo not specified\n", __func__, __LINE__);
	} else {
		strlcpy(alspinfo, p_chr, sizeof(alspinfo));
		pr_info("%s: alspinfo = %s\n", __func__, alspinfo);
	}

	return rc;
}

static int fih_alsp_probe(struct platform_device *pdev)
{
	int rc = 0;

	if (!pdev || !pdev->dev.of_node) {
		pr_err("%s: Unable to load device node\n", __func__);
		return -ENOTSUPP;
	}

	rc = fih_alsp_property(pdev);
	if (rc) {
		pr_err("%s Unable to set property\n", __func__);
		return rc;
	}

	//proc_mkdir("AllHWList", NULL);
	proc_create("AllHWList/alspinfo", 0, NULL, &fih_alsp_fops_info);

	return rc;
}

static int fih_alsp_remove(struct platform_device *pdev)
{
	remove_proc_entry("AllHWList/alspinfo", NULL);

	return 0;
}

static const struct of_device_id fih_alsp_dt_match[] = {
	{ .compatible = "fih_alsp" },
	{}
};
MODULE_DEVICE_TABLE(of, fih_alsp_dt_match);

static struct platform_driver fih_alsp_driver = {
	.probe = fih_alsp_probe,
	.remove = fih_alsp_remove,
	.shutdown = NULL,
	.driver = {
		.name = "fih_alsp",
		.of_match_table = fih_alsp_dt_match,
	},
};

static int __init fih_alsp_init(void)
{
	int ret;

	ret = platform_driver_register(&fih_alsp_driver);
	if (ret) {
		pr_err("%s: failed!\n", __func__);
		return ret;
	}

	return ret;
}
module_init(fih_alsp_init);

static void __exit fih_alsp_exit(void)
{
	platform_driver_unregister(&fih_alsp_driver);
}
module_exit(fih_alsp_exit);
