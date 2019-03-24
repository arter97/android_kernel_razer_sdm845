#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <fih/fih_ufs.h>

#define FIH_PROC_DIR   "AllHWList"
#define FIH_PROC_PATH  "AllHWList/ufsinfo"
#define FIH_PROC_SIZE  FIH_UFSINFO_SIZE

static char fih_proc_data[FIH_PROC_SIZE] = "unknown";

void fih_ufs_setup(char *info)
{
	snprintf(fih_proc_data, sizeof(fih_proc_data), "%s", info);
}

static int fih_proc_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%s", fih_proc_data);
	return 0;
}

static int fih_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_proc_read, NULL);
}

static const struct file_operations fih_proc_fops = {
	.open    = fih_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int __init fih_proc_init(void)
{
	if (proc_create(FIH_PROC_PATH, 0, NULL, &fih_proc_fops) == NULL) {
		proc_mkdir(FIH_PROC_DIR, NULL);
		if (proc_create(FIH_PROC_PATH, 0, NULL, &fih_proc_fops) == NULL) {
			pr_err("fail to create proc/%s\n", FIH_PROC_PATH);
			return (1);
		}
	}
	return (0);
}

static void __exit fih_proc_exit(void)
{
	remove_proc_entry(FIH_PROC_PATH, NULL);
}

module_init(fih_proc_init);
module_exit(fih_proc_exit);
