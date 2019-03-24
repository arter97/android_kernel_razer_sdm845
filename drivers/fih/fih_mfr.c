#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>

/* ref. kernel/drivers/soc/qcom/pil-q6v5-mss.c */
#define MAX_SSR_REASON_LEN  256U

static char *fih_mfr_name = "modemfailurereason";
static char fih_mfr_temp[MAX_SSR_REASON_LEN];

DEFINE_RAW_SPINLOCK(fih_mfr_lock);

void fih_mfr_update(char *reason)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&fih_mfr_lock, flags);

	memset(fih_mfr_temp, 0, sizeof(fih_mfr_temp));
	snprintf(fih_mfr_temp, sizeof(fih_mfr_temp), "%s\n", reason);

	raw_spin_unlock_irqrestore(&fih_mfr_lock, flags);

	return;
}

static void *my_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos) return NULL;  //exit start
	return (void *)pos;     //just for return
}

static int my_seq_show(struct seq_file *s, void *v)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&fih_mfr_lock, flags);

	seq_write(s, fih_mfr_temp, strlen(fih_mfr_temp));

	/* clean the failure reasen buffer after reading it every time */
	memset(fih_mfr_temp, 0, sizeof(fih_mfr_temp));

	raw_spin_unlock_irqrestore(&fih_mfr_lock, flags);

	return 0;
}

static void *my_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	++(*pos);  //make exit
	return NULL;
}

static void my_seq_stop(struct seq_file *s, void *v)
{
	return;
}

static struct seq_operations my_seq_ops = {
	.start = my_seq_start,
	.show  = my_seq_show,
	.next  = my_seq_next,
	.stop  = my_seq_stop,
};

static int my_seq_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &my_seq_ops);
};

static struct file_operations my_file_ops = {
	.owner   = THIS_MODULE,
	.open    = my_seq_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};

static int __init fih_mfr_init(void)
{
	struct proc_dir_entry *entry;

	entry = proc_create(fih_mfr_name, 0, NULL, &my_file_ops);
	if (!entry) {
		pr_err("%s: fail create proc\n", fih_mfr_name);
		return 1;
	}

	return 0;
}

static void __exit fih_mfr_exit(void)
{
	remove_proc_entry(fih_mfr_name, NULL);
}

module_init(fih_mfr_init);
module_exit(fih_mfr_exit);
