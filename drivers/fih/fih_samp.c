#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <fih/fih_mem.h>

static char *my_proc_name = "samp";
static unsigned int my_proc_addr = FIH_MEM_SAMP_ADDR;
static unsigned int my_proc_size = FIH_MEM_SAMP_SIZE;

static void *my_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos) return NULL;  //exit start
	return (void *)pos; //just for retrun
}

static int my_seq_show(struct seq_file *s, void *v)
{
	char *buf;

	buf = (char *)ioremap(my_proc_addr, my_proc_size);
	if (buf == NULL) {
		pr_err("%s: ioremap fail\n", my_proc_name);
		return 0;
	}

	seq_write(s, buf, my_proc_size);
	iounmap(buf);

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

static int __init my_module_init(void)
{
	struct proc_dir_entry *entry;

	entry = proc_create(my_proc_name, 0, NULL, &my_file_ops);
	if (!entry) {
		pr_err("%s: fail create proc\n", my_proc_name);
		return 1;
	}

	return 0;
}

static void __exit my_module_exit(void)
{
	remove_proc_entry(my_proc_name, NULL);
}

module_init(my_module_init);
module_exit(my_module_exit);
