/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "%s: " fmt, KBUILD_MODNAME

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/uaccess.h>
#include <asm/arch_timer.h>
#include <soc/qcom/smem.h>
#include "rpmh_master_stat.h"

#define UNIT_DIST 0x14
#define REG_VALID 0x0
#define REG_DATA_LO 0x4
#define REG_DATA_HI 0x8

#define GET_ADDR(REG, UNIT_NO) (REG + (UNIT_DIST * UNIT_NO))

enum master_smem_id {
	MPSS = 605,
	ADSP,
	CDSP,
	SLPI,
	GPU,
	DISPLAY,
};

enum master_pid {
	PID_APSS = 0,
	PID_MPSS = 1,
	PID_ADSP = 2,
	PID_SLPI = 3,
	PID_CDSP = 5,
	PID_GPU = PID_APSS,
	PID_DISPLAY = PID_APSS,
};

enum profile_data {
	POWER_DOWN_START,
	POWER_UP_END,
	POWER_DOWN_END,
	POWER_UP_START,
	ALT_UNIT,
	NUM_UNIT = ALT_UNIT,
};

struct msm_rpmh_master_data {
	char *master_name;
	enum master_smem_id smem_id;
	enum master_pid pid;
};

static const struct msm_rpmh_master_data rpmh_masters[] = {
	{"MPSS", MPSS, PID_MPSS},
	{"ADSP", ADSP, PID_ADSP},
	{"CDSP", CDSP, PID_CDSP},
	{"SLPI", SLPI, PID_SLPI},
	{"GPU", GPU, PID_GPU},
	{"DISPLAY", DISPLAY, PID_DISPLAY},
};

struct msm_rpmh_master_stats {
	uint32_t version_id;
	uint32_t counts;
	uint64_t last_entered;
	uint64_t last_exited;
	uint64_t accumulated_duration;
};

struct msm_rpmh_profile_unit {
	uint64_t value;
	uint64_t valid;
};

struct rpmh_master_stats_prv_data {
	struct kobj_attribute ka;
	struct kobject *kobj;
};

static struct msm_rpmh_master_stats apss_master_stats;
static void __iomem *rpmh_unit_base;
static uint32_t use_alt_unit;

static DEFINE_MUTEX(rpmh_stats_mutex);

static ssize_t msm_rpmh_master_stats_print_data(char *prvbuf, ssize_t length,
				struct msm_rpmh_master_stats *record,
				const char *name)
{
	uint64_t temp_accumulated_duration = record->accumulated_duration;
	/*
	 * If a master is in sleep when reading the sleep stats from SMEM
	 * adjust the accumulated sleep duration to show actual sleep time.
	 * This ensures that the displayed stats are real when used for
	 * the purpose of computing battery utilization.
	 */
	if (record->last_entered > record->last_exited)
		temp_accumulated_duration +=
				(arch_counter_get_cntvct()
				- record->last_entered);

	return snprintf(prvbuf, length, "%s\n\tVersion:0x%x\n"
			"\tSleep Count:0x%x\n"
			"\tSleep Last Entered At:0x%llx\n"
			"\tSleep Last Exited At:0x%llx\n"
			"\tSleep Accumulated Duration:0x%llx\n\n",
			name, record->version_id, record->counts,
			record->last_entered, record->last_exited,
			temp_accumulated_duration);
}

static ssize_t msm_rpmh_master_stats_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	ssize_t length;
	int i = 0;
	unsigned int size = 0;
	struct msm_rpmh_master_stats *record = NULL;

	mutex_lock(&rpmh_stats_mutex);

	/* First Read APSS master stats */

	length = msm_rpmh_master_stats_print_data(buf, PAGE_SIZE,
						&apss_master_stats, "APSS");

	/* Read SMEM data written by other masters */

	for (i = 0; i < ARRAY_SIZE(rpmh_masters); i++) {
		record = (struct msm_rpmh_master_stats *) smem_get_entry(
					rpmh_masters[i].smem_id, &size,
					rpmh_masters[i].pid, 0);
		if (!IS_ERR_OR_NULL(record) && (PAGE_SIZE - length > 0))
			length += msm_rpmh_master_stats_print_data(
					buf + length, PAGE_SIZE - length,
					record,
					rpmh_masters[i].master_name);
	}

	mutex_unlock(&rpmh_stats_mutex);

	return length;
}

static inline void msm_rpmh_apss_master_stats_update(
				struct msm_rpmh_profile_unit *profile_unit)
{
	apss_master_stats.counts++;
	apss_master_stats.last_entered = profile_unit[POWER_DOWN_END].value;
	apss_master_stats.last_exited = profile_unit[POWER_UP_START].value;
	apss_master_stats.accumulated_duration +=
					(apss_master_stats.last_exited
					- apss_master_stats.last_entered);
}

void msm_rpmh_master_stats_update(void)
{
	int i;
	struct msm_rpmh_profile_unit profile_unit[NUM_UNIT];

	if (!rpmh_unit_base)
		return;

	for (i = POWER_DOWN_END; i < NUM_UNIT; i++) {
		if (i == use_alt_unit) {
			profile_unit[i].value = readl_relaxed(
						rpmh_unit_base + GET_ADDR(
						REG_DATA_LO, ALT_UNIT));
			profile_unit[i].value |= ((uint64_t)
						readl_relaxed(
						rpmh_unit_base + GET_ADDR(
						REG_DATA_HI, ALT_UNIT)) << 32);
			continue;
		}

		profile_unit[i].valid = readl_relaxed(rpmh_unit_base +
						GET_ADDR(REG_VALID, i));

		/*
		 * Do not update APSS stats if valid bit is not set.
		 * It means APSS did not execute cx-off sequence.
		 * This can be due to fall through at some point.
		 */

		if (!(profile_unit[i].valid & BIT(REG_VALID)))
			return;

		profile_unit[i].value = readl_relaxed(rpmh_unit_base +
						GET_ADDR(REG_DATA_LO, i));
		profile_unit[i].value |= ((uint64_t)
					readl_relaxed(rpmh_unit_base +
					GET_ADDR(REG_DATA_HI, i)) << 32);
	}
	msm_rpmh_apss_master_stats_update(profile_unit);
}
EXPORT_SYMBOL(msm_rpmh_master_stats_update);

#ifdef CONFIG_FIH_RPMH_MASTER_STATS
#define RPMH_INFO_LENGTH	512
#define NOTIFY_THRES	(30000)

struct fih_rpmh_master_data {
	char rpmh_buf[RPMH_INFO_LENGTH];
	bool notify;
	uint64_t apss_last_exited_ms;
	uint64_t master_last_entered_ms;
	uint64_t master_last_exited_ms;
	uint64_t master_accumulated_duration_ms;
	uint64_t master_enabled_time_ms;
};

static struct fih_rpmh_master_data fih_rpmh_data;

static ssize_t fih_rpmh_master_stats_print_data(char *prvbuf, ssize_t length,
				struct msm_rpmh_master_stats *record,
				const char *name)
{	/* QCT raw format */
	#if 0
	return snprintf(prvbuf, length, "(%s, 0x%x, 0x%x, 0x%llx, 0x%llx, 0x%llx, (0x%d)), ",
			name, record->version_id, record->counts,
			record->last_exited, record->last_entered,
			record->accumulated_duration,
			(record->last_entered > record->last_exited) ? 0 : 1);
	#else
	/* Our old format with converted time as ms */
	fih_rpmh_data.master_last_entered_ms = record->last_entered;
	fih_rpmh_data.master_last_exited_ms = record->last_exited;
	fih_rpmh_data.master_accumulated_duration_ms = record->accumulated_duration;

	(void)do_div(fih_rpmh_data.master_last_entered_ms, 19200);
	(void)do_div(fih_rpmh_data.master_last_exited_ms, 19200);
	(void)do_div(fih_rpmh_data.master_accumulated_duration_ms, 19200);

	fih_rpmh_data.master_enabled_time_ms = 0;

	if (strncmp(name, "APSS", 4) == 0) {
		fih_rpmh_data.apss_last_exited_ms = fih_rpmh_data.master_last_exited_ms;

	} else if (record->last_exited > record->last_entered) {
		/* When sub-system enabled, calculate the enabled time. */
		if (fih_rpmh_data.apss_last_exited_ms > fih_rpmh_data.master_last_exited_ms)
			fih_rpmh_data.master_enabled_time_ms = fih_rpmh_data.apss_last_exited_ms -
				fih_rpmh_data.master_last_exited_ms;
	}

	return snprintf(prvbuf, length, "[PM] sleep_info.%s - %9llu(0x%d), %9llu, %9llu, %8llu mS.\n",
			name,
			fih_rpmh_data.master_last_exited_ms,
			(record->last_exited > record->last_entered) ? 1 : 0,
			fih_rpmh_data.master_last_entered_ms,
			fih_rpmh_data.master_accumulated_duration_ms,
			fih_rpmh_data.master_enabled_time_ms);
	#endif
}

void check_msm_rpmh_master_stats(void)
{
	ssize_t length;
	int i = 0;
	unsigned int size = 0;
	struct msm_rpmh_master_stats *record = NULL;


	mutex_lock(&rpmh_stats_mutex);

	fih_rpmh_data.notify = false;

	/* First Read APSS master stats */

	length = fih_rpmh_master_stats_print_data(fih_rpmh_data.rpmh_buf, RPMH_INFO_LENGTH,
						&apss_master_stats, "APSS");

	/* Read SMEM data written by other masters */

	for (i = 0; i < ARRAY_SIZE(rpmh_masters); i++) {
		record = (struct msm_rpmh_master_stats *) smem_get_entry(
					rpmh_masters[i].smem_id, &size,
					rpmh_masters[i].pid, 0);

		if (!IS_ERR_OR_NULL(record)) {
			if (RPMH_INFO_LENGTH - length > 0)
				length += fih_rpmh_master_stats_print_data(
						fih_rpmh_data.rpmh_buf + length, RPMH_INFO_LENGTH - length,
						record,
						rpmh_masters[i].master_name);

			/* Check if any sub-system still wakeup and over threshold. */
			if (fih_rpmh_data.master_enabled_time_ms > NOTIFY_THRES) {
				fih_rpmh_data.notify = true;
			}
		}
	}

	mutex_unlock(&rpmh_stats_mutex);

	/* Output rpmh master info for debugging. */
	if (fih_rpmh_data.notify)
		pr_info("%s", fih_rpmh_data.rpmh_buf);
}
EXPORT_SYMBOL(check_msm_rpmh_master_stats);
#endif /* CONFIG_FIH_RPMH_MASTER_STATS */

static int msm_rpmh_master_stats_probe(struct platform_device *pdev)
{
	struct rpmh_master_stats_prv_data *prvdata = NULL;
	struct kobject *rpmh_master_stats_kobj = NULL;
	int ret = -ENOMEM;

	if (!pdev)
		return -EINVAL;

	prvdata = devm_kzalloc(&pdev->dev, sizeof(*prvdata), GFP_KERNEL);
	if (!prvdata)
		return ret;

	rpmh_master_stats_kobj = kobject_create_and_add(
					"rpmh_stats",
					power_kobj);
	if (!rpmh_master_stats_kobj)
		return ret;

	prvdata->kobj = rpmh_master_stats_kobj;

	sysfs_attr_init(&prvdata->ka.attr);
	prvdata->ka.attr.mode = 0444;
	prvdata->ka.attr.name = "master_stats";
	prvdata->ka.show = msm_rpmh_master_stats_show;
	prvdata->ka.store = NULL;

	ret = sysfs_create_file(prvdata->kobj, &prvdata->ka.attr);
	if (ret) {
		pr_err("sysfs_create_file failed\n");
		goto fail_sysfs;
	}

	ret = of_property_read_u32(pdev->dev.of_node,
					"qcom,use-alt-unit",
					&use_alt_unit);
	if (ret)
		use_alt_unit = -1;

	rpmh_unit_base = of_iomap(pdev->dev.of_node, 0);
	if (!rpmh_unit_base) {
		pr_err("Failed to get rpmh_unit_base\n");
		ret = -ENOMEM;
		goto fail_iomap;
	}

	apss_master_stats.version_id = 0x1;
	platform_set_drvdata(pdev, prvdata);
	return ret;

fail_iomap:
	sysfs_remove_file(prvdata->kobj, &prvdata->ka.attr);
fail_sysfs:
	kobject_put(prvdata->kobj);
	return ret;
}

static int msm_rpmh_master_stats_remove(struct platform_device *pdev)
{
	struct rpmh_master_stats_prv_data *prvdata;

	if (!pdev)
		return -EINVAL;

	prvdata = (struct rpmh_master_stats_prv_data *)
				platform_get_drvdata(pdev);

	sysfs_remove_file(prvdata->kobj, &prvdata->ka.attr);
	kobject_put(prvdata->kobj);
	platform_set_drvdata(pdev, NULL);
	iounmap(rpmh_unit_base);
	rpmh_unit_base = NULL;

	return 0;
}

static const struct of_device_id rpmh_master_table[] = {
	{.compatible = "qcom,rpmh-master-stats-v1"},
	{},
};

static struct platform_driver msm_rpmh_master_stats_driver = {
	.probe	= msm_rpmh_master_stats_probe,
	.remove = msm_rpmh_master_stats_remove,
	.driver = {
		.name = "msm_rpmh_master_stats",
		.of_match_table = rpmh_master_table,
	},
};

module_platform_driver(msm_rpmh_master_stats_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM RPMH Master Statistics driver");
MODULE_ALIAS("platform:msm_rpmh_master_stat_log");
