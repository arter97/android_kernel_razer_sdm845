#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/file.h>
#include <linux/uaccess.h>

static char pid[64];
static char bt_mac[128];
static char bt_mac_raw[64];
static char wifi_mac[64];
static char wifi_mac2[64];

extern char *saved_command_line;

static int fih_mfd_proc_read_pid(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", pid);
	return 0;
}

static int fih_mfd_proc_open_pid(struct inode *inode, struct file *file)
{
	return single_open(file, fih_mfd_proc_read_pid, NULL);
}

static const struct file_operations fih_mfd_fops_pid = {
	.open    = fih_mfd_proc_open_pid,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static void fih_mfd_read_bt_mac(void)
{
	unsigned int i, k;

	/* 0123456789AB -> 01:23:45:67:89:AB */
	memset(bt_mac, 0, sizeof(bt_mac));
	k = 0;
	for (i=0; i<strlen(bt_mac_raw); i++) {
		if ((i > 0)&&((i % 2) == 0)) bt_mac[k++] = ':';
		bt_mac[k++] = bt_mac_raw[i];
	}
}


static int fih_mfd_proc_read_wifi_mac(struct seq_file *m, void *v)
{
	char tmp[(sizeof(wifi_mac) * 2)];
	unsigned int i, k;

	/* 0123456789AB -> 01:23:45:67:89:AB */
	memset(tmp, 0, sizeof(tmp));
	k = 0;
	for (i=0; i<strlen(wifi_mac); i++) {
		if ((i > 0)&&((i % 2) == 0)) tmp[k++] = ':';
		tmp[k++] = wifi_mac[i];
	}

	seq_printf(m, "%s\n", tmp);

	return 0;
}

static int fih_mfd_proc_open_wifi_mac(struct inode *inode, struct file *file)
{
	return single_open(file, fih_mfd_proc_read_wifi_mac, NULL);
}

static const struct file_operations fih_mfd_fops_wifi_mac = {
	.open    = fih_mfd_proc_open_wifi_mac,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int fih_mfd_proc_read_wifi_mac2(struct seq_file *m, void *v)
{
	char tmp[(sizeof(wifi_mac2) * 2)];
	unsigned int i, k;

	/* 0123456789AB -> 01:23:45:67:89:AB */
	memset(tmp, 0, sizeof(tmp));
	k = 0;
	for (i=0; i<strlen(wifi_mac2); i++) {
		if ((i > 0)&&((i % 2) == 0)) tmp[k++] = ':';
		tmp[k++] = wifi_mac2[i];
	}

	seq_printf(m, "%s\n", tmp);

	return 0;
}

static int fih_mfd_proc_open_wifi_mac2(struct inode *inode, struct file *file)
{
	return single_open(file, fih_mfd_proc_read_wifi_mac2, NULL);
}

static const struct file_operations fih_mfd_fops_wifi_mac2 = {
	.open    = fih_mfd_proc_open_wifi_mac2,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int fih_mfd_property(struct platform_device *pdev)
{
	static const char *p_chr;

	p_chr = of_get_property(pdev->dev.of_node, "fih-mfd,pid", NULL);
	if (!p_chr) {
		pr_info("%s:%d, pid not specified\n", __func__, __LINE__);
	} else {
		strlcpy(pid, p_chr, sizeof(pid));
		pr_info("%s: pid = %s\n", __func__, pid);
	}

	p_chr = of_get_property(pdev->dev.of_node, "fih-mfd,bt_mac", NULL);
	if (!p_chr) {
		pr_info("%s:%d, bt_mac not specified\n", __func__, __LINE__);
	} else {
		strlcpy(bt_mac_raw, p_chr, sizeof(bt_mac_raw));
		pr_info("%s: bt_mac_raw = %s\n", __func__, bt_mac_raw);
	}

	p_chr = of_get_property(pdev->dev.of_node, "fih-mfd,wifi_mac", NULL);
	if (!p_chr) {
		pr_info("%s:%d, wifi_mac not specified\n", __func__, __LINE__);
	} else {
		strlcpy(wifi_mac, p_chr, sizeof(wifi_mac));
		pr_info("%s: wifi_mac = %s\n", __func__, wifi_mac);
	}

	p_chr = of_get_property(pdev->dev.of_node, "fih-mfd,wifi_mac2", NULL);
	if (!p_chr) {
		pr_info("%s:%d, wifi_mac2 not specified\n", __func__, __LINE__);
	} else {
		strlcpy(wifi_mac2, p_chr, sizeof(wifi_mac2));
		pr_info("%s: wifi_mac2 = %s\n", __func__, wifi_mac2);
	}

	return 0;
}

static int fih_mfd_probe(struct platform_device *pdev)
{
	int rc = 0;

	if (!pdev || !pdev->dev.of_node) {
		pr_err("%s: Unable to load device node\n", __func__);
		return -ENOTSUPP;
	}

	rc = fih_mfd_property(pdev);
	if (rc) {
		pr_err("%s Unable to set property\n", __func__);
		return rc;
	}

	proc_create("productid", 0, NULL, &fih_mfd_fops_pid);
	proc_create("wifi_mac", 0, NULL, &fih_mfd_fops_wifi_mac);
	proc_create("wifi_mac2", 0, NULL, &fih_mfd_fops_wifi_mac2);

	fih_mfd_read_bt_mac();

	return rc;
}

static int fih_mfd_remove(struct platform_device *pdev)
{
	remove_proc_entry ("wifi_mac2", NULL);
	remove_proc_entry ("wifi_mac", NULL);
	remove_proc_entry ("productid", NULL);
	return 0;
}

static const struct of_device_id fih_mfd_dt_match[] = {
	{.compatible = "fih_mfd"},
	{}
};
MODULE_DEVICE_TABLE(of, fih_mfd_dt_match);

static struct platform_driver fih_mfd_driver = {
	.probe = fih_mfd_probe,
	.remove = fih_mfd_remove,
	.shutdown = NULL,
	.driver = {
		.name = "fih_mfd",
		.of_match_table = fih_mfd_dt_match,
	},
};

static int __init fih_mfd_init(void)
{
	int ret;

	ret = platform_driver_register(&fih_mfd_driver);
	if (ret) {
		pr_err("%s: failed!\n", __func__);
		return ret;
	}

	return ret;
}
module_init(fih_mfd_init);

static void __exit fih_mfd_exit(void)
{
	platform_driver_unregister(&fih_mfd_driver);
}

module_param_string(bt_mac, bt_mac, sizeof(bt_mac), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_mac, "BT MAC address");

module_exit(fih_mfd_exit);
