/*
 *  Copyright (C) 2008-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/regulator/machine.h>
#include <linux/proc_fs.h>
#include <linux/cpufreq.h>
#include <linux/iram_alloc.h>
#include <linux/fsl_devices.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>
#include <asm/tlb.h>
#include <asm/delay.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#ifdef CONFIG_ARCH_MX50
#include <mach/iomux-mx50.h>
#endif
#include "crm_regs.h"

#define DATABAHN_CTL_REG0	0
#define DATABAHN_CTL_REG19	0x4c
#define DATABAHN_CTL_REG79	0x13c
#define DATABAHN_PHY_REG25	0x264
#define MX53_OFFSET 0x20000000

static struct cpu_op *cpu_op_tbl;
static int cpu_op_nr;
static struct clk *cpu_clk;
static struct mxc_pm_platform_data *pm_data;
static int databahn_mode;

static void __iomem *pll1_base;

#if defined(CONFIG_CPU_FREQ)
static int org_freq;
extern int cpufreq_suspended;
extern int set_cpu_freq(int wp);
#endif


static struct device *pm_dev;
struct clk *gpc_dvfs_clk;
extern void cpu_do_suspend_workaround(u32 sdclk_iomux_addr);
extern void mx50_suspend(u32 databahn_addr);
extern struct cpu_op *(*get_cpu_op)(int *wp);
extern void __iomem *ccm_base;
extern void __iomem *databahn_base;
extern void da9053_suspend_cmd(void);
extern void da9053_resume_dump(void);
extern void pm_da9053_i2c_init(u32 base_addr);

extern int iram_ready;
void *suspend_iram_base;
void (*suspend_in_iram)(void *param1, void *param2, void* param3) = NULL;
void __iomem *suspend_param1;

static int mx5_suspend_enter(suspend_state_t state)
{
	if (gpc_dvfs_clk == NULL)
		gpc_dvfs_clk = clk_get(NULL, "gpc_dvfs_clk");
	/* gpc clock is needed for SRPG */
	clk_enable(gpc_dvfs_clk);
	switch (state) {
	case PM_SUSPEND_MEM:
		mxc_cpu_lp_set(STOP_POWER_OFF);
		break;
	case PM_SUSPEND_STANDBY:
		mxc_cpu_lp_set(WAIT_UNCLOCKED_POWER_OFF);
		break;
	default:
		return -EINVAL;
	}

	if (tzic_enable_wake(0) != 0)
		return -EAGAIN;

	if (state == PM_SUSPEND_MEM) {
		if (!cpu_is_mx53()) {
			local_flush_tlb_all();
			flush_cache_all();
		}

		if (pm_data && pm_data->suspend_enter)
			pm_data->suspend_enter();
		if (cpu_is_mx51() || cpu_is_mx53()) {
			/* Run the suspend code from iRAM. */
			suspend_in_iram(suspend_param1, NULL, NULL);

			if (!cpu_is_mx53()) {
				/*clear the EMPGC0/1 bits */
				__raw_writel(0, MXC_SRPG_EMPGC0_SRPGCR);
				__raw_writel(0, MXC_SRPG_EMPGC1_SRPGCR);
			}
		} else {
			if (cpu_is_mx50()) {
				/* Store the LPM mode of databanhn */
				databahn_mode = __raw_readl(
					databahn_base + DATABAHN_CTL_REG20);

				/* Suspend now. */
				suspend_in_iram(databahn_base,
						ccm_base, pll1_base);

				/* Restore the LPM databahn_mode. */
				__raw_writel(databahn_mode,
					databahn_base + DATABAHN_CTL_REG20);

			}
		}
		if (pm_data && pm_data->suspend_exit)
			pm_data->suspend_exit();
	} else {
			cpu_do_idle();
	}
	clk_disable(gpc_dvfs_clk);

	return 0;
}


/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mx5_suspend_prepare(void)
{
#if defined(CONFIG_CPU_FREQ)
#define MX53_SUSPEND_CPU_WP 400000000
	struct cpufreq_freqs freqs;
	u32 suspend_wp = 0;
	org_freq = clk_get_rate(cpu_clk);
	/* workaround for mx53 to suspend on 400MHZ wp */
	if (cpu_is_mx53())
		for (suspend_wp = 0; suspend_wp < cpu_op_nr; suspend_wp++)
			if (cpu_op_tbl[suspend_wp].cpu_rate
				== MX53_SUSPEND_CPU_WP)
				break;
	if (suspend_wp == cpu_op_nr)
		suspend_wp = 0;
	pr_info("suspend wp cpu=%d\n", cpu_op_tbl[suspend_wp].cpu_rate);
	freqs.old = org_freq / 1000;
	freqs.new = cpu_op_tbl[suspend_wp].cpu_rate / 1000;
	freqs.cpu = 0;
	freqs.flags = 0;

	cpufreq_suspended = 1;
	if (clk_get_rate(cpu_clk) != cpu_op_tbl[suspend_wp].cpu_rate) {
		set_cpu_freq(cpu_op_tbl[suspend_wp].cpu_rate);
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
#endif
	return 0;
}

/*
 * Called before devices are re-setup.
 */
static void mx5_suspend_finish(void)
{
#if defined(CONFIG_CPU_FREQ)
	struct cpufreq_freqs freqs;

	freqs.old = clk_get_rate(cpu_clk) / 1000;
	freqs.new = org_freq / 1000;
	freqs.cpu = 0;
	freqs.flags = 0;

	cpufreq_suspended = 0;

	if (org_freq != clk_get_rate(cpu_clk)) {
		set_cpu_freq(org_freq);
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
#endif
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void mx5_suspend_end(void)
{
}

static int mx5_pm_valid(suspend_state_t state)
{
	return (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX);
}

struct platform_suspend_ops mx5_suspend_ops = {
	.valid = mx5_pm_valid,
	.prepare = mx5_suspend_prepare,
	.enter = mx5_suspend_enter,
	.finish = mx5_suspend_finish,
	.end = mx5_suspend_end,
};

static int __devinit mx5_pm_probe(struct platform_device *pdev)
{
	pm_dev = &pdev->dev;
	pm_data = pdev->dev.platform_data;

	return 0;
}

static struct platform_driver mx5_pm_driver = {
	.driver = {
		   .name = "mx5_pm",
		   },
	.probe = mx5_pm_probe,
};

#define SUSPEND_ID_MX51 1
#define SUSPEND_ID_MX53 3
#define SUSPEND_ID_NONE 4
static int __init pm_init(void)
{
	unsigned long iram_paddr;
	void *cpaddr;

	pr_info("Static Power Management for Freescale i.MX5\n");
	if (platform_driver_register(&mx5_pm_driver) != 0) {
		printk(KERN_ERR "mx5_pm_driver register failed\n");
		return -ENODEV;
	}
	if (cpu_is_mx51())
		pll1_base = MX51_IO_ADDRESS(MX51_PLL1_BASE_ADDR);
	else if (cpu_is_mx53())
		pll1_base = MX53_IO_ADDRESS(MX53_PLL1_BASE_ADDR);
	else if (cpu_is_mx50())
		pll1_base = MX50_IO_ADDRESS(MX50_PLL1_BASE_ADDR);

	suspend_param1 = 0;
	suspend_set_ops(&mx5_suspend_ops);
	/* Move suspend routine into iRAM */
	cpaddr = iram_alloc(SZ_4K, &iram_paddr);
	/* Need to remap the area here since we want the memory region
		 to be executable. */
	suspend_iram_base = __arm_ioremap(iram_paddr, SZ_4K,
					  MT_MEMORY_NONCACHED);
	pr_info("cpaddr = %x suspend_iram_base=%x\n", (unsigned int)cpaddr,
					(unsigned int)suspend_iram_base);

	if (cpu_is_mx51() || cpu_is_mx53()) {
		suspend_param1 =
			cpu_is_mx51() ? (void *)SUSPEND_ID_MX51: \
					(void *)SUSPEND_ID_MX53;
		memcpy(suspend_iram_base, cpu_do_suspend_workaround,
			SZ_4K);
	} else if (cpu_is_mx50()) {
		/*
		 * Need to run the suspend code from IRAM as the DDR needs
		 * to be put into self refresh mode manually.
		 */
		memcpy(suspend_iram_base, mx50_suspend, SZ_4K);

		suspend_param1 = databahn_base;
	}
	suspend_in_iram = (void *)suspend_iram_base;

	cpu_op_tbl = get_cpu_op(&cpu_op_nr);

	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk)) {
		printk(KERN_DEBUG "%s: failed to get cpu_clk\n", __func__);
		return PTR_ERR(cpu_clk);
	}
	printk(KERN_INFO "PM driver module loaded\n");

	return 0;
}


static void __exit pm_cleanup(void)
{
	/* Unregister the device structure */
	platform_driver_unregister(&mx5_pm_driver);
}

module_init(pm_init);
module_exit(pm_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("PM driver");
MODULE_LICENSE("GPL");
