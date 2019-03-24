#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <fih/fih_mem.h>

static char *my_proc_fold = "flog";
static char *my_proc_name = "flog/last_xlog";
static unsigned int my_proc_addr = FIH_MEM_LAST_XLOG_ADDR;
static unsigned int my_proc_size = FIH_MEM_LAST_XLOG_SIZE;

static void *my_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos) return NULL;  //exit start
	return (void *)pos; //just for retrun
}

static int my_seq_show(struct seq_file *s, void *v)
{
	char *c, *key_word, *log_data;
	unsigned int *log_size, *buf_size, max_size;

	c = (char *)ioremap(my_proc_addr, my_proc_size);
	if (c == NULL) {
		pr_err("%s: ioremap failed\n", my_proc_name);
		return 0;
	}

	key_word = (char *)(c);
	log_size = (unsigned int *)(c + 4);
	buf_size = (unsigned int *)(c + 8);
	log_data = (char *)(c + 12);
	max_size = my_proc_size - 12;

	if (memcmp(key_word, "DBGC", 4)) goto exit_show;
	if (*buf_size > max_size) goto exit_show;
	if (*log_size > *buf_size) goto exit_show;

	if (*log_size < *buf_size) {
		seq_write(s, (log_data + *log_size + 1), (*buf_size - *log_size - 1));
	}
	seq_write(s, log_data, *log_size);

exit_show:
	iounmap(c);

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
	//proc_mkdir(my_proc_fold, NULL);
	if (proc_create(my_proc_name, 0, NULL, &my_file_ops) == NULL) {
		proc_mkdir(my_proc_fold, NULL);
		if (proc_create(my_proc_name, 0, NULL, &my_file_ops) == NULL) {
			pr_err("fail to create proc/%s\n", my_proc_name);
			return (1);
		}
	}
	return 0;
}

static void __exit my_module_exit(void)
{
	remove_proc_entry(my_proc_name, NULL);
}

module_init(my_module_init);
module_exit(my_module_exit);
