#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <fih/fih_mem.h>

static char *my_proc_name = "mdm_nv_def";
static unsigned int my_proc_addr = FIH_MEM_NV_DEF_ADDR;
static unsigned int my_proc_size = FIH_MEM_NV_DEF_SIZE;
static unsigned int my_proc_len = 0;

static void *my_seq_start(struct seq_file *s, loff_t *pos)
{
	if (((*pos)*PAGE_SIZE) >= my_proc_len) return NULL;
	return (void *)((unsigned long) *pos+1);
}

static void *my_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	++*pos;
	return my_seq_start(s, pos);
}

static void my_seq_stop(struct seq_file *s, void *v)
{
	return;
}

static int my_seq_show(struct seq_file *s, void *v)
{
	long n = (long)v - 1;
	char *buf = (char *)ioremap(my_proc_addr, my_proc_size);

	if (buf == NULL) {
		return 0;
	}

	if (my_proc_len < (PAGE_SIZE*(n+1))) {
		seq_write(s, (buf+(PAGE_SIZE*n)), (my_proc_len - (PAGE_SIZE*n)));
	} else {
		seq_write(s, (buf+(PAGE_SIZE*n)), PAGE_SIZE);
	}

	iounmap(buf);

	return 0;
}

static struct seq_operations my_seq_ops = {
	.start = my_seq_start,
	.next  = my_seq_next,
	.stop  = my_seq_stop,
	.show  = my_seq_show
};

static int my_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &my_seq_ops);
};

static struct file_operations my_file_ops = {
	.owner   = THIS_MODULE,
	.open    = my_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};

static int __init my_module_init(void)
{
	my_proc_len = my_proc_size;
	proc_create(my_proc_name, 0, NULL, &my_file_ops);
	return 0;
}

static void __exit my_module_exit(void)
{
	remove_proc_entry(my_proc_name, NULL);
}

module_init(my_module_init);
module_exit(my_module_exit);
