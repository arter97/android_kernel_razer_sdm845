#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of_fdt.h>

#define FIH_PROC_DIR   "AllHWList"
#define FIH_PROC_PATH  "AllHWList/cpuinfo"

static const char *fih_proc_data;

static int fih_proc_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", fih_proc_data);
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
	fih_proc_data = of_flat_dt_get_machine_name();

	if (proc_create(FIH_PROC_PATH, 0, NULL, &fih_proc_fops) == NULL) {
		proc_mkdir(FIH_PROC_DIR, NULL);
		if (proc_create(FIH_PROC_PATH, 0, NULL, &fih_proc_fops) == NULL) {
			pr_err("fail to create proc/%s\n", FIH_PROC_PATH);
			return 1;
		}
	}
	return 0;
}

static void __exit fih_proc_exit(void)
{
	remove_proc_entry(FIH_PROC_PATH, NULL);
}

module_init(fih_proc_init);
module_exit(fih_proc_exit);
