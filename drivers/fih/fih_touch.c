#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "fih_touch.h"

#define FIH_PROC_DIR        "AllHWList"
#define FIH_PROC_TP_SELF_TEST   "AllHWList/tp_self_test"
#define FIH_PROC_TP_IC_FW_VER   "AllHWList/tp_fw_ver"
#define FIH_PROC_TP_UPGRADE     "AllHWList/tp_upgrade"
#define FIH_PROC_TP_FILE_FW_FW  "AllHWList/tp_fwim_ver"
#define FIH_PROC_TP_DOUBLE_TAP  "AllHWList/tp_double_tap"
#define FIH_PROC_TP_DOWN_GRADE  "AllHWList/tp_fw_back"
#define FIH_PROC_TP_VENDOR      "AllHWList/tp_vendor"
#define FIH_PROC_TP_PROX_STATUS  "AllHWList/tp_prox_status"
#define FIH_PROC_TP_UNLOCK_SCREEN  "AllHWList/tp_unlock_screen"

#ifdef CONFIG_FIH_A1N
#define FIH_PROC_TP_SIDE_TOUCH_STATUS  "AllHWList/tp_side_status"
#endif

//SW8-JH-ALT test+[
#define FIH_PROC_TP_ALT_RST         "AllHWList/tp_alt_rst"
#define FIH_PROC_TP_ALT_ST_COUNT    "AllHWList/tp_alt_st_count"
#define FIH_PROC_TP_ALT_ST_ENABLE   "AllHWList/tp_alt_st_enable"
#define FIH_PROC_TP_ALT_ST_DISABLE  "AllHWList/tp_alt_st_disable"
//SW8-JH-ALT test+]

extern bool ResultSelfTest;
extern void touch_selftest(void);
extern void touch_tpfwver_read(char *);
extern void touch_fwback_write(void);
extern int touch_fwback_read(void);
extern void touch_tpfwimver_read(char *fw_ver);
extern void touch_fwupgrade(int);
extern void touch_fwupgrade_read(char *);
extern void touch_vendor_read(char *);
//SW8-JH-ALT test+[
extern void touch_alt_rst(int);
extern int touch_alt_st_count(int);
extern void touch_alt_st_enable(int);
//SW8-JH-ALT test+]
struct fih_touch_cb touch_cb = {
    .touch_selftest = NULL,
    .touch_selftest_result = NULL,
    .touch_tpfwver_read = NULL,
    .touch_tpfwimver_read = NULL,
    .touch_fwupgrade = NULL,
    .touch_fwupgrade_read = NULL,
    .touch_vendor_read = NULL,
    .touch_vendor_id_read = NULL,
    .touch_double_tap_read = NULL,
    .touch_double_tap_write = NULL,
    .touch_alt_rst = NULL,
    .touch_alt_st_count = NULL,
    .touch_alt_st_enable = NULL,
    .touch_side_touch_enable = NULL,
    .touch_side_touch_status = NULL,
    .touch_unlock_write = NULL,
    .touch_unlock_read = NULL,
};
int tp_probe_success = 0;

char fih_touch[32] = "unknown";
void fih_info_set_touch(char *info)
{
    strcpy(fih_touch, info);
}

static int fih_touch_unlock_show(struct seq_file *m, void *v)
{
    if (touch_cb.touch_unlock_read != NULL)
    {
        pr_info("F@Touch unlock Status\n");
        seq_printf(m, "%d\n", touch_cb.touch_unlock_read());
    }
    return 0;
}

static int fih_touch_unlock_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, fih_touch_unlock_show, NULL);
};

static ssize_t fih_touch_unlock_proc_write(struct file *file, const char __user *buffer,
    size_t count, loff_t *ppos)
{
    char *buf;
    unsigned int enabled_unlock;

    buf = kzalloc(count, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    if (copy_from_user(buf, buffer, count))
        return -EFAULT;
    enabled_unlock = simple_strtoul(buf, NULL, 10);
    if((enabled_unlock != 0) &&  (enabled_unlock != 1))
    {
        pr_err("F@Touch %s, wrong value\n", __func__);
        return -EINVAL;
    }

    if (touch_cb.touch_unlock_write != NULL)
    {
        pr_info("F@Touch %s side touch\n", enabled_unlock ? "Enable" : "Disable");
        touch_cb.touch_unlock_write(enabled_unlock);
    }
    return count;

}
static struct file_operations touch_unlock_proc_file_ops = {
    .owner   = THIS_MODULE,
    .write   = fih_touch_unlock_proc_write,
    .open    = fih_touch_unlock_proc_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release
};

#ifdef CONFIG_FIH_A1N
static int fih_touch_side_touch_show(struct seq_file *m, void *v)
{
    if (touch_cb.touch_side_touch_status != NULL)
    {
        pr_info("F@Touch Side Touch Status\n");
        seq_printf(m, "%d\n", touch_cb.touch_side_touch_status());
    }
    return 0;
}

static int fih_touch_side_touch_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, fih_touch_side_touch_show, NULL);
};

static ssize_t fih_touch_side_touch_proc_write(struct file *file, const char __user *buffer,
    size_t count, loff_t *ppos)
{
    unsigned int enabled_side_touch;
    sscanf(buffer, "%u", &enabled_side_touch) ;

    if((enabled_side_touch != 0) &&  (enabled_side_touch != 1))
    {
        pr_err("F@Touch %s, wrong value\n", __func__);
        return -EINVAL;
    }

    if (touch_cb.touch_double_tap_write != NULL)
    {
        pr_info("F@Touch %s side touch\n", enabled_side_touch ? "Enable" : "Disable");
        touch_cb.touch_side_touch_enable(enabled_side_touch);
    }
    return count;

}
static struct file_operations touch_side_touch_proc_file_ops = {
    .owner   = THIS_MODULE,
    .write   = fih_touch_side_touch_proc_write,
    .open    = fih_touch_side_touch_proc_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release
};
#endif
//touch self test  start
static int fih_touch_self_test_show(struct seq_file *m, void *v)
{
    if (touch_cb.touch_selftest_result != NULL)
    {
        pr_info("F@Touch Touch Selftest Result Read\n");
        seq_printf(m, "%d\n", touch_cb.touch_selftest_result());
    }
    return 0;
}

static int fih_touch_self_test_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, fih_touch_self_test_show, NULL);
};

static ssize_t fih_touch_self_test_proc_write(struct file *file, const char __user *buffer,
    size_t count, loff_t *ppos)
{
    if(touch_cb.touch_selftest != NULL)
    {
    pr_info("F@Touch Do Touch Selftest\n");
        touch_cb.touch_selftest();
    }
    return count;
}
static struct file_operations touch_self_test_proc_file_ops = {
    .owner   = THIS_MODULE,
    .write   = fih_touch_self_test_proc_write,
    .open    = fih_touch_self_test_proc_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release
};

//touch self test  end

//touch_fwver start
static int fih_touch_read_fwver_show(struct seq_file *m, void *v)
{
    char fwver[30]={0};

    if(touch_cb.touch_tpfwver_read != NULL)
    {
        pr_info("F@Touch Read Touch Firmware Version\n");
        touch_cb.touch_tpfwver_read(fwver);
    seq_printf(m, "%s", fwver);
    }
    return 0;
}

static int fih_touch_fwver_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, fih_touch_read_fwver_show, NULL);
};

static struct file_operations touch_fwver_proc_file_ops = {
    .owner   = THIS_MODULE,
    .open    = fih_touch_fwver_proc_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release
};
//touch_fwver end

//touch_fwimver start
static int fih_touch_read_fwimver_show(struct seq_file *m, void *v)
{
    char fwimver[30]={0};
    if(touch_cb.touch_tpfwimver_read != NULL)
    {
        pr_info("F@Touch Read Touch Firmware Image Version\n");
        touch_cb.touch_tpfwimver_read(fwimver);
    seq_printf(m, "%s", fwimver);
    }
    return 0;
}

static int fih_touch_fwimver_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, fih_touch_read_fwimver_show, NULL);
};

static struct file_operations touch_fwimver_proc_file_ops = {
    .owner   = THIS_MODULE,
    .open    = fih_touch_fwimver_proc_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release
};
//touch_fwimver end

//touch_upgrade start
static int fih_touch_upgrade_read_show(struct seq_file *m, void *v)
{
    char upgrade_flag[10]={0};

    if(touch_cb.touch_fwupgrade_read != NULL)
    {
        pr_info("F@Touch Read Touch Upgrade Flag\n");
        touch_cb.touch_fwupgrade_read(upgrade_flag);
    seq_printf(m, "%s", upgrade_flag);
    }
    return 0;
}

static int fih_touch_upgrade_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, fih_touch_upgrade_read_show, NULL);
};

static ssize_t fih_touch_upgrade_proc_write(struct file *file, const char __user *buffer,
    size_t count, loff_t *ppos)
{
    char *buf;
    unsigned int res, input = 0;

    if (touch_cb.touch_fwupgrade == NULL)
        return -EINVAL;

    if (count < 1)
        return -EINVAL;

    buf = kzalloc(count, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    if (copy_from_user(buf, buffer, count))
        return -EFAULT;
    input = simple_strtoul(buf, NULL, 10);

    if (touch_cb.touch_fwupgrade != NULL)
    {
        pr_info("F@Touch FW upgrade\n");
        touch_cb.touch_fwupgrade(input);
    }

    if (res < 0)
    {
        kfree(buf);
        return res;
    }

    kfree(buf);

    /* claim that we wrote everything */
    return count;
    //////////////////////////////
    #if 0
    int input;

    if (sscanf(buffer, "%u", &input) != 1)
    {
         return -EINVAL;
    }

    if(touch_cb.touch_fwupgrade != NULL)
    {
        pr_info("F@Touch Write Touch Upgrade(%d)\n", input);
        touch_cb.touch_fwupgrade(input);
    }
    return count;
    #endif
}

static struct file_operations touch_upgrade_proc_file_ops = {
    .owner   = THIS_MODULE,
    .write   = fih_touch_upgrade_proc_write,
    .open    = fih_touch_upgrade_proc_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release
};
//touch_upgrade end

//touch_vendor start
static int fih_touch_read_vendor_show(struct seq_file *m, void *v)
{
    char vendor[30]={0};
    if (touch_cb.touch_vendor_read != NULL)
    {
        pr_info("F@Touch Read Touch Vendor\n");
        touch_cb.touch_vendor_read(vendor);
    seq_printf(m, "%s", vendor);
    }
    return 0;
}

static int fih_touch_vendor_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, fih_touch_read_vendor_show, NULL);
};

static struct file_operations touch_vendor_proc_file_ops = {
    .owner   = THIS_MODULE,
    .open    = fih_touch_vendor_proc_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release
};
//touch_vendor end
//touch_fwback start
static int fih_touch_fwback_read_show(struct seq_file *m, void *v)
{
    if (touch_cb.touch_fwback_read != NULL)
    {
        pr_info("F@Touch Read Touch Firmware Flag\n");
        seq_printf(m, "%d\n", touch_cb.touch_fwback_read());
    }
    return 0;
}

static int fih_touch_fwback_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, fih_touch_fwback_read_show, NULL);
};

static ssize_t fih_touch_fwback_proc_write(struct file *file, const char __user *buffer,
    size_t count, loff_t *ppos)
{
    if (touch_cb.touch_fwback_write != NULL)
    {
        pr_info("F@Touch Write Touch Firmware Flag To 1\n");
        touch_cb.touch_fwback_write();
    }
    return count;
}

static struct file_operations touch_fwback_proc_file_ops = {
    .owner   = THIS_MODULE,
    .write   = fih_touch_fwback_proc_write,
    .open    = fih_touch_fwback_proc_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release
};
//touch_fwback end

//SW8-DH-Double_Tap-00+[
static int fih_touch_double_tap_read_show(struct seq_file *m, void *v)
{
    if (touch_cb.touch_double_tap_read != NULL)
    {
        pr_info("F@Touch Read Double tap \n");
        seq_printf(m, "%u\n", touch_cb.touch_double_tap_read());
    }
    return 0;
}

static int fih_touch_double_tap_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, fih_touch_double_tap_read_show, NULL);
};

static ssize_t fih_touch_double_tap_proc_write(struct file *file, const char __user *buffer,
    size_t count, loff_t *ppos)
{
    char *buf;
    unsigned int res, double_tap = 0;

    if (touch_cb.touch_double_tap_write == NULL)
        return -EINVAL;

    if (count < 1)
        return -EINVAL;

    buf = kzalloc(count, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    if (copy_from_user(buf, buffer, count))
        return -EFAULT;
    double_tap = simple_strtoul(buf, NULL, 10);

    if((double_tap != 0) && (double_tap !=1))
    {
        pr_err("F@Touch %s, wrong value, double_tap = %d, *buf = %x\n", __func__, double_tap, *buf);
        return -EINVAL;
    }
    if (touch_cb.touch_double_tap_write != NULL)
    {
        pr_info("F@Touch Set double tap\n");
        touch_cb.touch_double_tap_write(double_tap);
    }

    if (res < 0)
    {
        kfree(buf);
        return res;
    }

    kfree(buf);

    /* claim that we wrote everything */
    return count;
}


static int fih_touch_prox_status_read_show(struct seq_file *m, void *v)
{
    if (touch_cb.touch_prox_status_read != NULL)
    {
        pr_info("F@Touch Read prox_status\n");
        seq_printf(m, "%u\n", touch_cb.touch_prox_status_read());
    }
    return 0;
}

static int fih_touch_prox_status_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, fih_touch_prox_status_read_show, NULL);
};

static ssize_t fih_touch_prox_status_proc_write(struct file *file, const char __user *buffer,
    size_t count, loff_t *ppos)
{
    unsigned int prox_status = 0;
    char *buf;
    unsigned int res;
    if (touch_cb.touch_prox_status_write == NULL)
        return -EINVAL;

    if (count < 1)
        return -EINVAL;

    buf = kzalloc(count, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    if (copy_from_user(buf, buffer, count))
        return -EFAULT;
    prox_status = simple_strtoul(buf, NULL, 10);
    if((prox_status != 0) && (prox_status !=1))
    {
        pr_err("F@Touch, prox_status %s, wrong value, prox_status = %d\n", __func__, prox_status);
        return -EINVAL;
    }
    if (touch_cb.touch_prox_status_write != NULL)
    {
        pr_info("F@Touch Set prox_status\n");
        touch_cb.touch_prox_status_write(prox_status);
    }

    if (res < 0)
    {
        kfree(buf);
        return res;
    }

    kfree(buf);

    /* claim that we wrote everything */
    return count;
}


static struct file_operations touch_double_tap_proc_file_ops = {
    .owner   = THIS_MODULE,
    .write   = fih_touch_double_tap_proc_write,
    .open    = fih_touch_double_tap_proc_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release
};
static struct file_operations touch_prox_status_proc_file_ops = {
    .owner   = THIS_MODULE,
    .write   = fih_touch_prox_status_proc_write,
    .open    = fih_touch_prox_status_proc_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release
};

//SW8-DH-Double_Tap-00+]
//SW8-JH-ALT test+[
static int fih_touch_alt_rst_show(struct seq_file *m, void *v)
{
    return 0;
}

static int fih_touch_alt_rst_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, fih_touch_alt_rst_show, NULL);
};

static ssize_t fih_touch_alt_rst_proc_write(struct file *file, const char __user *buffer,
       size_t count, loff_t *ppos)
{
    unsigned int ALT_Reset = 0;
    char *buf;
    unsigned int res;
    if (touch_cb.touch_alt_rst == NULL)
        return -EINVAL;

    if (count < 1)
        return -EINVAL;

    buf = kzalloc(count, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    if (copy_from_user(buf, buffer, count))
        return -EFAULT;
    ALT_Reset = simple_strtoul(buf, NULL, 10);
    if(ALT_Reset !=1)
    {
        pr_err("F@Touch %s, wrong value\n", __func__);
        return -EINVAL;
    }
    if (touch_cb.touch_alt_rst != NULL)
    {
        pr_info("F@Touch Write Touch ALT Reset(%d)\n", ALT_Reset);
        touch_cb.touch_alt_rst();
    }

    if (res < 0)
    {
        kfree(buf);
        return res;
    }

    kfree(buf);

    /* claim that we wrote everything */
    return count;
}

static struct file_operations touch_alt_rst_file_ops = {
    .owner   = THIS_MODULE,
    .open    = fih_touch_alt_rst_proc_open,
    .write   = fih_touch_alt_rst_proc_write,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release
};

static int fih_touch_alt_st_count_show(struct seq_file *m, void *v)
{
    int count=0;

    if(touch_cb.touch_alt_st_count != NULL)
    {
        pr_info("F@Touch Read ALT ST Test count\n");
        count = touch_cb.touch_alt_st_count();
        seq_printf(m, "%d", count);
    }
    return 0;
}

static int fih_touch_alt_st_count_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, fih_touch_alt_st_count_show, NULL);
};

static struct file_operations touch_alt_st_count_file_ops = {
    .owner   = THIS_MODULE,
    .open    = fih_touch_alt_st_count_proc_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release
};

static int fih_touch_alt_st_enable_show(struct seq_file *m, void *v)
{
    return 0;
}

static int fih_touch_alt_st_enable_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, fih_touch_alt_st_enable_show, NULL);
};

static ssize_t fih_touch_alt_st_enable_proc_write(struct file *file, const char __user *buffer,
       size_t count, loff_t *ppos)
{
    unsigned int ALT_st_enable = 0;
    char *buf;
    unsigned int res;
    if (touch_cb.touch_alt_st_enable == NULL)
        return -EINVAL;

    if (count < 1)
        return -EINVAL;

    buf = kzalloc(count, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    if (copy_from_user(buf, buffer, count))
        return -EFAULT;
    ALT_st_enable = simple_strtoul(buf, NULL, 10);
    if((ALT_st_enable !=1) || (ALT_st_enable != 0))
    {
        pr_err("F@Touch %s, wrong value\n", __func__);
        return -EINVAL;
    }
    if (touch_cb.touch_alt_st_enable != NULL)
    {
        pr_info("F@Touch Touch ALT enable(%d)\n", ALT_st_enable);
        touch_cb.touch_alt_st_enable(ALT_st_enable);
    }

    if (res < 0)
    {
        kfree(buf);
        return res;
    }

    kfree(buf);

    /* claim that we wrote everything */
    return count;
}
static struct file_operations touch_alt_st_enable_file_ops = {
    .owner   = THIS_MODULE,
    .open    = fih_touch_alt_st_enable_proc_open,
    .write   = fih_touch_alt_st_enable_proc_write,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release
};
//SW8-JH-ALT test+]
static int __init fih_touch_init(void)
{
    int retry_count = 50;
    while(!tp_probe_success && retry_count-- > 0)
    {
         if((retry_count % 10) == 9)
             pr_err("waitng for touch panel probe ready %d\n",retry_count%10);
             msleep(100);
    };
    if(tp_probe_success)
    {
        pr_err("panel probe success, create proc file\n");
        //F@Touch Self Test
        if (proc_create(FIH_PROC_TP_SELF_TEST, 0, NULL, &touch_self_test_proc_file_ops) == NULL)
        {
            proc_mkdir(FIH_PROC_DIR, NULL);
            if (proc_create(FIH_PROC_TP_SELF_TEST, 0, NULL, &touch_self_test_proc_file_ops) == NULL)
            {
            pr_err("fail to create proc/%s\n", FIH_PROC_TP_SELF_TEST);
            return (1);
            }
        }

        //F@Touch Read IC's firmware version
        if (proc_create(FIH_PROC_TP_IC_FW_VER, 0, NULL, &touch_fwver_proc_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_TP_IC_FW_VER);
            return (1);
        }

        //F@Touch Read Touch firmware image version
        if (proc_create(FIH_PROC_TP_FILE_FW_FW, 0, NULL, &touch_fwimver_proc_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_TP_FILE_FW_FW);
            return (1);
        }

        //F@Touch Firmware Upgrade
        if (proc_create(FIH_PROC_TP_UPGRADE, 0, NULL, &touch_upgrade_proc_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_TP_UPGRADE);
            return (1);
        }

        //F@Touch Get Vendor name
        if (proc_create(FIH_PROC_TP_VENDOR, 0, NULL, &touch_vendor_proc_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_TP_VENDOR);
            return (1);
        }
        if (proc_create(FIH_PROC_TP_DOWN_GRADE, 0, NULL, &touch_fwback_proc_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_TP_DOWN_GRADE);
            return (1);
        }
        if (proc_create(FIH_PROC_TP_DOUBLE_TAP, 0, NULL, &touch_double_tap_proc_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_TP_DOUBLE_TAP);
            return (1);
        }
        if (proc_create(FIH_PROC_TP_PROX_STATUS, 0, NULL, &touch_prox_status_proc_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_TP_PROX_STATUS);
            return (1);
        }
//SW8-JH-ALT test+[
        //F@Touch ALT test
        if (proc_create(FIH_PROC_TP_ALT_RST, 0, NULL, &touch_alt_rst_file_ops) == NULL)
        {
               pr_err("fail to create proc/%s\n", FIH_PROC_TP_ALT_RST);
               return (1);
        }
        if (proc_create(FIH_PROC_TP_ALT_ST_COUNT, 0, NULL, &touch_alt_st_count_file_ops) == NULL)
        {
               pr_err("fail to create proc/%s\n", FIH_PROC_TP_ALT_ST_COUNT);
               return (1);
        }
        if (proc_create(FIH_PROC_TP_ALT_ST_ENABLE, 0, NULL, &touch_alt_st_enable_file_ops) == NULL)
        {
               pr_err("fail to create proc/%s\n", FIH_PROC_TP_ALT_ST_ENABLE);
               return (1);
        }
        if (proc_create(FIH_PROC_TP_UNLOCK_SCREEN, 0, NULL, &touch_unlock_proc_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_TP_UNLOCK_SCREEN);
            return (1);
        }
//SW8-JH-ALT test+]
#ifdef CONFIG_FIH_A1N
        if (proc_create(FIH_PROC_TP_SIDE_TOUCH_STATUS, 0, NULL, &touch_side_touch_proc_file_ops) == NULL)
        {
            pr_err("fail to create proc/%s\n", FIH_PROC_TP_SIDE_TOUCH_STATUS);
            return (1);
        }
#endif
    }
    else
    {
        pr_err("No panel probe, Fail to create proc file\n");
    }

    return (0);
}

static void __exit fih_touch_exit(void)
{
    if(tp_probe_success)
    {
        remove_proc_entry(FIH_PROC_TP_SELF_TEST, NULL);
        remove_proc_entry(FIH_PROC_TP_IC_FW_VER, NULL);
        remove_proc_entry(FIH_PROC_TP_FILE_FW_FW, NULL);
        remove_proc_entry(FIH_PROC_TP_UPGRADE, NULL);
        remove_proc_entry(FIH_PROC_TP_VENDOR, NULL);
    }
}

late_initcall(fih_touch_init);
module_exit(fih_touch_exit);
