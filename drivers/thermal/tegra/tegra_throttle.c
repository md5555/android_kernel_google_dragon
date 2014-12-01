/*
 * tegra_throttle.c
 *
 * Copyright (c) 2012, 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/thermal.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#include <soc/tegra/tegra_emc.h>
#include <soc/tegra/tegra-dvfs.h>

enum throttle_cap_freqs {
	CAP_CPU = 0,
	CAP_GPU,
	CAP_CLK_START,
	CAP_C3BUS = CAP_CLK_START,
	CAP_SCLK,
	CAP_EMC,
	NUM_OF_CAP_FREQS,
};

struct tegra_throttle_table {
	unsigned long cap_freqs[NUM_OF_CAP_FREQS];
};

struct tegra_balanced_throttle {
	const struct cpufreq_frequency_table *cpu_freq_table;
	unsigned long cpu_throttle_lowest_speed;
	unsigned long bthrot_speed;
};

struct tegra_throttle_cap_data {
	const char *cap_name;
	struct clk *cap_clk;
	unsigned long cap_freq;
	unsigned long max_freq;
};

/*
 * The instance of balanced throttle.
 * This structure is used to describe the behavior of
 * a certain balanced throttle device.
 */
struct tegra_balanced_throttle_ins {
	struct thermal_cooling_device *cdev;
	struct device_node *np;
	struct list_head node;
	unsigned long cur_state;
	int throt_tab_size;
	struct tegra_throttle_table *throt_tab;
	struct tegra_balanced_throttle *b_throt;
	struct tegra_throttle_cap_data *cap_data;
	int cap_data_size;
#ifdef CONFIG_DEBUG_FS
	struct dentry *d_tab;
#endif
};

#define MAX_THROT_TABLE_SIZE	(64)
#define NO_CAP			(ULONG_MAX) /* no cap */

/*
 * Locked when adding/removing throttle device to/from bthrot_list.
 * Locked when throttling cap.throttle.xxx clocks.
 */
static DEFINE_MUTEX(bthrot_list_lock);
static LIST_HEAD(bthrot_list);

static bool is_notify;
unsigned long cpu_bthrot_speed;

static struct tegra_throttle_cap_data tegra_cap_freqs_table[] = {
	{ .cap_name = "cap.throttle.c3bus" },
	{ .cap_name = "cap.throttle.sclk" },
	{ .cap_name = "cap.throttle.emc" },
};

static unsigned long
clip_to_table(struct tegra_balanced_throttle_ins *bthrot_ins,
		unsigned long cpu_freq)
{
	struct tegra_balanced_throttle *bthrot = bthrot_ins->b_throt;
	int i;

	if (!bthrot->cpu_freq_table)
		return -EINVAL;

	for (i = 0;
	     bthrot->cpu_freq_table[i].frequency != CPUFREQ_TABLE_END;
	     i++) {
		if (bthrot->cpu_freq_table[i].frequency > cpu_freq)
			break;
	}
	i = (i == 0) ? 0 : i-1;

	return bthrot->cpu_freq_table[i].frequency;
}

static int tegra_throttle_get_max_state(struct thermal_cooling_device *cdev,
					unsigned long *max_state)
{
	struct tegra_balanced_throttle_ins *bthrot_ins = cdev->devdata;

	*max_state = bthrot_ins->throt_tab_size;

	return 0;
}

static int tegra_throttle_get_cur_state(struct thermal_cooling_device *cdev,
					unsigned long *cur_state)
{
	struct tegra_balanced_throttle_ins *bthrot_ins = cdev->devdata;

	*cur_state = bthrot_ins->cur_state;

	return 0;
}

static void
tegra_throttle_set_cap_clk(struct tegra_balanced_throttle_ins *bthrot_ins,
			struct tegra_throttle_table *throt_tab,
			int cap_clk_index)
{
	struct tegra_throttle_cap_data *tegra_cap_freqs = bthrot_ins->cap_data;
	unsigned long cap_rate, clk_rate;
	int cap_offset = cap_clk_index - CAP_CLK_START;

	cap_rate = throt_tab->cap_freqs[cap_clk_index];
	if (cap_rate == NO_CAP)
		clk_rate = tegra_cap_freqs[cap_offset].max_freq;
	else
		clk_rate = cap_rate * 1000UL;

	if (tegra_cap_freqs[cap_offset].cap_freq != clk_rate) {
		clk_set_rate(tegra_cap_freqs[cap_offset].cap_clk, clk_rate);
		tegra_cap_freqs[cap_offset].cap_freq = clk_rate;
	}
}

static void
tegra_throttle_cap_freqs_update(struct tegra_balanced_throttle_ins *bthrot_ins,
				struct tegra_throttle_table *throt_tab,
				bool performance_up)
{
	int i;
	int max_cap_clock = CAP_CLK_START + bthrot_ins->cap_data_size;

	if (performance_up) {
		/* performance up : throttle less */
		for (i = max_cap_clock - 1; i >= CAP_CLK_START; i--)
			tegra_throttle_set_cap_clk(bthrot_ins, throt_tab, i);
	} else {
		/* performance down : throttle more */
		for (i = CAP_CLK_START; i < max_cap_clock; i++)
			tegra_throttle_set_cap_clk(bthrot_ins, throt_tab, i);
	}
}

static int
tegra_throttle_set_cur_state(struct thermal_cooling_device *cdev,
			     unsigned long cur_state)
{
	struct tegra_balanced_throttle_ins *bthrot_ins = cdev->devdata;
	struct tegra_balanced_throttle *bthrot = bthrot_ins->b_throt;
	int max_cap_clock = CAP_CLK_START + bthrot_ins->cap_data_size;
	bool performance_up;
	int i, ret;
	unsigned long bthrot_speed, save_state;
	struct tegra_throttle_table *throt_entry;
	struct tegra_throttle_table cur_throt_freq;

	if (bthrot->cpu_freq_table == NULL)
		return 0;

	if (bthrot_ins->cur_state == cur_state)
		return 0;

	for (i = 0; i < NUM_OF_CAP_FREQS; i++)
		cur_throt_freq.cap_freqs[i] = NO_CAP;

	performance_up = bthrot_ins->cur_state >= cur_state;
	save_state = bthrot_ins->cur_state;
	bthrot_ins->cur_state = cur_state;

	mutex_lock(&bthrot_list_lock);
	list_for_each_entry(bthrot_ins, &bthrot_list, node) {
		if (!bthrot_ins->cur_state)
			continue;

		throt_entry = &bthrot_ins->throt_tab[bthrot_ins->cur_state-1];
		for (i = 0; i < max_cap_clock; i++) {
			cur_throt_freq.cap_freqs[i] = min(
					cur_throt_freq.cap_freqs[i],
					throt_entry->cap_freqs[i]);
		}
	}

	bthrot_ins = cdev->devdata;

	/* update cap clocks */
	tegra_throttle_cap_freqs_update(bthrot_ins, &cur_throt_freq,
					performance_up);

	/* update cpu clock */
	bthrot_speed = clip_to_table(bthrot_ins,
					cur_throt_freq.cap_freqs[CAP_CPU]);
	cpu_bthrot_speed = bthrot_speed;
	ret = cpufreq_update_policy(0);
	if (ret)
		goto error;

	/* TODO: no support gpu freq scalling yet */
	/* update gpu clock */

	mutex_unlock(&bthrot_list_lock);
	return 0;
error:
	bthrot_ins = cdev->devdata;
	bthrot_ins->cur_state = save_state;

	mutex_unlock(&bthrot_list_lock);
	return ret;
}

static struct thermal_cooling_device_ops tegra_throttle_cooling_ops = {
	.get_max_state = tegra_throttle_get_max_state,
	.get_cur_state = tegra_throttle_get_cur_state,
	.set_cur_state = tegra_throttle_set_cur_state,
};

/**
 * tegra_throttle_cpufreq_policy_notifier - Notifier callback for cpufreq policy
 * @nb:	struct notifier_block * with callback info.
 * @event: value showing cpufreq event for which this function invoked.
 * @data: callback-specific data
 *
 * Callback to hijack the notification on cpufreq policy transition.
 * Every time there is a change in policy, we will intercept and
 * update the cpufreq policy with thermal throttling constraints.
 *
 * Return: 0 (success)
 */
static int tegra_throttle_cpufreq_policy_notifier(struct notifier_block *nb,
						  unsigned long event,
						  void *data)
{
	struct cpufreq_policy *policy = data;

	if (event != CPUFREQ_ADJUST)
		return 0;

	if (cpu_bthrot_speed == NO_CAP)
		return 0;

	/* Limit max freq to be within tegra_throttle limit. */
	if (cpu_bthrot_speed < policy->min)
		cpu_bthrot_speed = policy->min;

	if (policy->max != cpu_bthrot_speed)
		cpufreq_verify_within_limits(policy, 0, cpu_bthrot_speed);

	return 0;
}

/* Notifier for cpufreq policy change */
static struct notifier_block tegra_throttle_cpufreq_notifier_block = {
	.notifier_call = tegra_throttle_cpufreq_policy_notifier,
};

#ifdef CONFIG_DEBUG_FS
static int table_show(struct seq_file *s, void *data)
{
	struct tegra_balanced_throttle_ins *bthrot_ins = s->private;
	int max_cap_clock = CAP_CLK_START + bthrot_ins->cap_data_size;
	int i, j;

	seq_printf(s, "%-7s %-10s %-10s %-10s %-10s %-10s\n",
		   "index", "CPU-freq", "GPU-freq", "C3bus-freq",
		   "SCLK-freq", "EMC-freq");
	for (i = 0; i < bthrot_ins->throt_tab_size; i++) {
		/* CPU FREQ */
		seq_printf(s, "[%-2d] %-2s %-10lu",
			   i, "=", bthrot_ins->throt_tab[i].cap_freqs[CAP_CPU]);

		/* GPU FREQ and other DVFS module FREQS */
		for (j = CAP_GPU; j < max_cap_clock; j++)
			if (bthrot_ins->throt_tab[i].cap_freqs[j] == NO_CAP)
				seq_printf(s, " %-10s", "NO_CAP");
			else
				seq_printf(s, " %-10lu",
					bthrot_ins->throt_tab[i].cap_freqs[j]);
		seq_puts(s, "\n");
	}

	return 0;
}

static int table_open(struct inode *inode, struct file *file)
{
	return single_open(file, table_show, inode->i_private);
}

static const struct file_operations table_fops = {
	.open		= table_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct dentry *throttle_debugfs_root;
#endif /* CONFIG_DEBUG_FS */

static struct thermal_cooling_device *balanced_throttle_register(
				struct tegra_balanced_throttle_ins *bthrot_ins,
				char *type)
{
#ifdef CONFIG_DEBUG_FS
	char name[32];
#endif

	bthrot_ins->cdev = thermal_of_cooling_device_register(
						bthrot_ins->np,
						type,
						bthrot_ins,
						&tegra_throttle_cooling_ops);

	if (IS_ERR(bthrot_ins->cdev)) {
		bthrot_ins->cdev = NULL;
		return ERR_PTR(-ENODEV);
	}

	mutex_lock(&bthrot_list_lock);
	list_add(&bthrot_ins->node, &bthrot_list);
	mutex_unlock(&bthrot_list_lock);

#ifdef CONFIG_DEBUG_FS
	sprintf(name, "%s_throttle_table", type);
	bthrot_ins->d_tab = debugfs_create_file(name, 0644,
						throttle_debugfs_root,
						bthrot_ins, &table_fops);
#endif

	return bthrot_ins->cdev;
}

static void
balanced_throttle_unregister(struct tegra_balanced_throttle_ins *bthrot_ins)
{
	thermal_cooling_device_unregister(bthrot_ins->cdev);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove(bthrot_ins->d_tab);
#endif

	mutex_lock(&bthrot_list_lock);
	list_del(&bthrot_ins->node);
	mutex_unlock(&bthrot_list_lock);
}

static int tegra_throttle_init(struct platform_device *pdev,
				struct tegra_throttle_cap_data *cap_freqs_table,
				int cap_freqs_table_size)
{
	struct tegra_balanced_throttle_ins *bthrot_ins;
	struct tegra_balanced_throttle *bthrot;
	const struct cpufreq_frequency_table *cpu_freq_table;
	struct tegra_throttle_cap_data *cap_data;
	int i;

	cpu_freq_table = cpufreq_frequency_get_table(0);
	if (!cpu_freq_table) {
		dev_warn(&pdev->dev,
			 "Cannot get cpufreq table data\n");
		return -EPROBE_DEFER;
	}

	for (i = 0; i < cap_freqs_table_size; i++) {
		struct clk *c;

		dev_info(&pdev->dev,
			 "Setting clock %s\n", cap_freqs_table[i].cap_name);
		c = devm_clk_get(&pdev->dev, cap_freqs_table[i].cap_name);
		if (IS_ERR(c)) {
			dev_warn(&pdev->dev,
				 "Cannot get clock %s\n",
				 cap_freqs_table[i].cap_name);
			return -EPROBE_DEFER;
		}

		cap_freqs_table[i].cap_clk = c;
		cap_freqs_table[i].max_freq = clk_round_rate(c, ULONG_MAX);
		cap_freqs_table[i].cap_freq = cap_freqs_table[i].max_freq;
	}

	bthrot_ins = devm_kzalloc(&pdev->dev, sizeof(*bthrot_ins),
				   GFP_KERNEL);
	if (!bthrot_ins)
		return -ENOMEM;

	bthrot = devm_kzalloc(&pdev->dev, sizeof(*bthrot), GFP_KERNEL);
	if (!bthrot)
		return -ENOMEM;

	cap_data = devm_kzalloc(&pdev->dev,
				sizeof(*cap_data) * cap_freqs_table_size,
				GFP_KERNEL);
	if (!cap_data)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, bthrot_ins);
	bthrot_ins->b_throt = bthrot;

	bthrot->cpu_freq_table = cpu_freq_table;
	bthrot->cpu_throttle_lowest_speed =
			bthrot->cpu_freq_table[0].frequency;
	bthrot->bthrot_speed = NO_CAP;

	memcpy(cap_data, cap_freqs_table,
		sizeof(cap_freqs_table[0]) * cap_freqs_table_size);
	bthrot_ins->cap_data = cap_data;
	bthrot_ins->cap_data_size = cap_freqs_table_size;

#ifdef CONFIG_DEBUG_FS
	if (!throttle_debugfs_root)
		throttle_debugfs_root = debugfs_create_dir("tegra_throttle",
							   NULL);
#endif

	cpu_bthrot_speed = NO_CAP;

	if (!is_notify) {
		cpufreq_register_notifier(
					&tegra_throttle_cpufreq_notifier_block,
					CPUFREQ_POLICY_NOTIFIER);
		is_notify = true;
	}

	dev_info(&pdev->dev, "Init done\n");

	return 0;
}

static void tegra_throttle_deinit(void)
{
	cpufreq_unregister_notifier(&tegra_throttle_cpufreq_notifier_block,
				    CPUFREQ_POLICY_NOTIFIER);
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(throttle_debugfs_root);
#endif
}

static int tegra_throt_parse_dt(struct platform_device *pdev,
				char *property_name,
				struct tegra_balanced_throttle_ins *throttle)
{
	struct tegra_throttle_table *throttle_table;
	struct device_node *t_dn, *tc_dn;
	const __be32 *prop;
	int num;

	prop = of_get_property(pdev->dev.of_node, property_name, NULL);
	if (!prop) {
		dev_err(&pdev->dev, "No throttle table %s\n",
			property_name);
		return -EINVAL;
	}

	t_dn = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!t_dn) {
		dev_err(&pdev->dev, "Can't find phandle of %s\n",
			property_name);
		return -EINVAL;
	}

	num = of_get_child_count(t_dn);
	if (!num) {
		dev_err(&pdev->dev, "No throttle states in throttle table\n");
		of_node_put(t_dn);
		return -EINVAL;
	}

	throttle_table = devm_kzalloc(&pdev->dev,
				      sizeof(struct tegra_throttle_table) * num,
				      GFP_KERNEL);
	if (!throttle_table) {
		dev_err(&pdev->dev, "Allocate throttle table failed\n");
		of_node_put(t_dn);
		return -ENOMEM;
	}

	num = 0;
	for_each_child_of_node(t_dn, tc_dn) {
		u32 val;

		if (of_property_read_u32(tc_dn, "cpu-freq", &val)) {
			dev_err(&pdev->dev, "Missing cpu-freq, ignore this state\n");
			continue;
		} else {
			throttle_table[num].cap_freqs[CAP_CPU] = val;
		}

		if (of_property_read_u32(tc_dn, "gpu-freq", &val))
			throttle_table[num].cap_freqs[CAP_GPU] = NO_CAP;
		else
			throttle_table[num].cap_freqs[CAP_GPU] = val;

		if (of_property_read_u32(tc_dn, "c3bus-freq", &val))
			throttle_table[num].cap_freqs[CAP_C3BUS] = NO_CAP;
		else
			throttle_table[num].cap_freqs[CAP_C3BUS] = val;

		if (of_property_read_u32(tc_dn, "sclk-freq", &val))
			throttle_table[num].cap_freqs[CAP_SCLK] = NO_CAP;
		else
			throttle_table[num].cap_freqs[CAP_SCLK] = val;

		if (of_property_read_u32(tc_dn, "emc-freq", &val))
			throttle_table[num].cap_freqs[CAP_EMC] = NO_CAP;
		else
			throttle_table[num].cap_freqs[CAP_EMC] = val;

		num++;
	}

	throttle->throt_tab_size = num;
	throttle->throt_tab = throttle_table;

	return 0;
}

/* Check if cpu clock, emc scaling and dvfs are ready. */
static bool tegra_throttle_cap_clk_is_ready(struct platform_device *pdev)
{
	int ret;

	if (!cpufreq_get(0)) {
		dev_warn(&pdev->dev, "CPU clocks are not ready.\n");
		return false;
	}

	if (of_machine_is_compatible("nvidia,tegra210"))
		ret = tegra210_emc_is_ready();
	else if (of_machine_is_compatible("nvidia,tegra124"))
		ret = tegra124_emc_is_ready();
	else
		ret = false;

	if (ret != true) {
		dev_warn(&pdev->dev, "Emc scalling is not ready.\n");
		return false;
	}

	if (tegra_dvfs_core_count_thermal_states(TEGRA_DVFS_CORE_THERMAL_FLOOR)
	    < 0) {
		dev_warn(&pdev->dev, "DVFS for cap_clks are not ready.\n");
		return false;
	}

	return true;
}

static int tegra_throttle_probe(struct platform_device *pdev)
{
	struct tegra_balanced_throttle_ins *b_throt_ins;
	int ret;

	if (!tegra_throttle_cap_clk_is_ready(pdev)) {
		dev_warn(&pdev->dev,
			 "Throttle cap clks are not ready.\n");
		return -EPROBE_DEFER;
	}

	ret = tegra_throttle_init(pdev, tegra_cap_freqs_table,
				  ARRAY_SIZE(tegra_cap_freqs_table));
	if (ret)
		return ret;

	b_throt_ins = platform_get_drvdata(pdev);

	/* register balanced cooling devices */
	if (tegra_throt_parse_dt(pdev, "balanced-states", b_throt_ins)) {
		dev_err(&pdev->dev, "Parse throttle table failed.\n");
		return -EINVAL;
	}

	b_throt_ins->np = pdev->dev.of_node;

	balanced_throttle_register(b_throt_ins, (char *)b_throt_ins->np->name);

	dev_info(&pdev->dev,
		 "Tegra throttling initialized\n");

	return 0;
}

static int tegra_throttle_remove(struct platform_device *pdev)
{
	struct tegra_balanced_throttle_ins *throt = platform_get_drvdata(pdev);

	balanced_throttle_unregister(throt);
	tegra_throttle_deinit();
	return 0;
}

static const struct of_device_id tegra_throttle_match[] = {
	{ .compatible = "nvidia,balanced-throttle", },
	{},
};

static struct platform_driver tegra_throttle_driver = {
	.probe	= tegra_throttle_probe,
	.remove	= tegra_throttle_remove,
	.driver = {
		.name	= "tegra-throttle",
		.owner	= THIS_MODULE,
		.of_match_table = tegra_throttle_match,
	},
};
module_platform_driver(tegra_throttle_driver);

MODULE_DESCRIPTION("Tegra Balanced Throttle Driver");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL");
