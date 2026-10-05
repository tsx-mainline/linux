// SPDX-License-Identifier: GPL-2.0-only
/*
 * Watchdog driver for Ricoh RN5T618 PMIC
 *
 * Copyright (C) 2014 Beniamino Galvani <b.galvani@gmail.com>
 */

#include <linux/device.h>
#include <linux/mfd/rn5t618.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/watchdog.h>

#define DRIVER_NAME "rn5t618-wdt"

static bool nowayout = WATCHDOG_NOWAYOUT;
static unsigned int timeout;

module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Initial watchdog timeout in seconds");

module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static bool early;
module_param(early, bool, 0);
MODULE_PARM_DESC(early, "Start the watchdog at probe and never stop it. The watchdog "
		 "core pings it while no user space daemon holds it (default=0)");

/* Timeout in seconds while the kernel pings the watchdog after an early start */
#define RN5T618_WDT_EARLY_TIMEOUT	32

struct rn5t618_wdt {
	struct watchdog_device wdt_dev;
	struct rn5t618 *rn5t618;
	struct notifier_block reboot_nb;
};

/*
 * This array encodes the values of WDOGTIM field for the supported
 * watchdog expiration times. If the watchdog is not accessed before
 * the timer expiration, the PMU generates an interrupt and if the CPU
 * doesn't clear it within one second the system is restarted.
 */
static const struct {
	u8 reg_val;
	unsigned int time;
} rn5t618_wdt_map[] = {
	{ 0, 1 },
	{ 1, 8 },
	{ 2, 32 },
	{ 3, 128 },
};

static int rn5t618_wdt_set_timeout(struct watchdog_device *wdt_dev,
				   unsigned int t)
{
	struct rn5t618_wdt *wdt = watchdog_get_drvdata(wdt_dev);
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(rn5t618_wdt_map); i++) {
		if (rn5t618_wdt_map[i].time + 1 >= t)
			break;
	}

	if (i == ARRAY_SIZE(rn5t618_wdt_map))
		return -EINVAL;

	ret = regmap_update_bits(wdt->rn5t618->regmap, RN5T618_WATCHDOG,
				 RN5T618_WATCHDOG_WDOGTIM_M,
				 rn5t618_wdt_map[i].reg_val);
	if (!ret)
		wdt_dev->timeout = rn5t618_wdt_map[i].time;

	return ret;
}

static int rn5t618_wdt_start(struct watchdog_device *wdt_dev)
{
	struct rn5t618_wdt *wdt = watchdog_get_drvdata(wdt_dev);
	int ret;

	ret = rn5t618_wdt_set_timeout(wdt_dev, wdt_dev->timeout);
	if (ret)
		return ret;

	/* enable repower-on */
	ret = regmap_update_bits(wdt->rn5t618->regmap, RN5T618_REPCNT,
				 RN5T618_REPCNT_REPWRON,
				 RN5T618_REPCNT_REPWRON);
	if (ret)
		return ret;

	/* enable watchdog */
	ret = regmap_update_bits(wdt->rn5t618->regmap, RN5T618_WATCHDOG,
				 RN5T618_WATCHDOG_WDOGEN,
				 RN5T618_WATCHDOG_WDOGEN);
	if (ret)
		return ret;

	/* enable watchdog interrupt */
	return regmap_update_bits(wdt->rn5t618->regmap, RN5T618_PWRIREN,
				  RN5T618_PWRIRQ_IR_WDOG,
				  RN5T618_PWRIRQ_IR_WDOG);
}

static int rn5t618_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct rn5t618_wdt *wdt = watchdog_get_drvdata(wdt_dev);

	/*
	 * With early=1 the PMU watchdog stays on after a magic close. The
	 * watchdog core pings it from the kernel, so a hang during the
	 * shutdown or in the restart path still resets the board. The PMU
	 * clears the watchdog on its power-off and repower sequence.
	 */
	if (early) {
		set_bit(WDOG_HW_RUNNING, &wdt_dev->status);
		return 0;
	}

	return regmap_update_bits(wdt->rn5t618->regmap, RN5T618_WATCHDOG,
				  RN5T618_WATCHDOG_WDOGEN, 0);
}

static int rn5t618_wdt_ping(struct watchdog_device *wdt_dev)
{
	struct rn5t618_wdt *wdt = watchdog_get_drvdata(wdt_dev);
	unsigned int val;
	int ret;

	/* The counter is restarted after a R/W access to watchdog register */
	ret = regmap_read(wdt->rn5t618->regmap, RN5T618_WATCHDOG, &val);
	if (ret)
		return ret;

	ret = regmap_write(wdt->rn5t618->regmap, RN5T618_WATCHDOG, val);
	if (ret)
		return ret;

	/* Clear pending watchdog interrupt */
	return regmap_update_bits(wdt->rn5t618->regmap, RN5T618_PWRIRQ,
				  RN5T618_PWRIRQ_IR_WDOG, 0);
}

/*
 * With early=1 a halt or a power-off without a PMU power-off would end in
 * a watchdog reset. Switch the watchdog off for these two. A restart keeps
 * it on until the PMU repower sequence clears it.
 */
static int rn5t618_wdt_reboot(struct notifier_block *nb, unsigned long code,
			      void *data)
{
	struct rn5t618_wdt *wdt = container_of(nb, struct rn5t618_wdt,
					       reboot_nb);

	if (code == SYS_HALT || code == SYS_POWER_OFF)
		regmap_update_bits(wdt->rn5t618->regmap, RN5T618_WATCHDOG,
				   RN5T618_WATCHDOG_WDOGEN, 0);

	return NOTIFY_DONE;
}

static const struct watchdog_info rn5t618_wdt_info = {
	.options	= WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE |
			  WDIOF_KEEPALIVEPING,
	.identity	= DRIVER_NAME,
};

static const struct watchdog_ops rn5t618_wdt_ops = {
	.owner          = THIS_MODULE,
	.start          = rn5t618_wdt_start,
	.stop           = rn5t618_wdt_stop,
	.ping           = rn5t618_wdt_ping,
	.set_timeout    = rn5t618_wdt_set_timeout,
};

static int rn5t618_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rn5t618 *rn5t618 = dev_get_drvdata(dev->parent);
	struct rn5t618_wdt *wdt;
	int min_timeout, max_timeout;
	int ret;
	unsigned int val;

	wdt = devm_kzalloc(dev, sizeof(struct rn5t618_wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	min_timeout = rn5t618_wdt_map[0].time;
	max_timeout = rn5t618_wdt_map[ARRAY_SIZE(rn5t618_wdt_map) - 1].time;

	wdt->rn5t618 = rn5t618;
	wdt->wdt_dev.info = &rn5t618_wdt_info;
	wdt->wdt_dev.ops = &rn5t618_wdt_ops;
	wdt->wdt_dev.min_timeout = min_timeout;
	wdt->wdt_dev.max_timeout = max_timeout;
	wdt->wdt_dev.timeout = max_timeout;
	wdt->wdt_dev.parent = dev;

	/* Read out previous power-off factor */
	ret = regmap_read(wdt->rn5t618->regmap, RN5T618_POFFHIS, &val);
	if (ret)
		return ret;

	if (val & RN5T618_POFFHIS_VINDET)
		wdt->wdt_dev.bootstatus = WDIOF_POWERUNDER;
	else if (val & RN5T618_POFFHIS_WDG)
		wdt->wdt_dev.bootstatus = WDIOF_CARDRESET;

	watchdog_set_drvdata(&wdt->wdt_dev, wdt);
	watchdog_init_timeout(&wdt->wdt_dev, timeout, dev);
	watchdog_set_nowayout(&wdt->wdt_dev, nowayout);

	platform_set_drvdata(pdev, wdt);

	/*
	 * With early=1 the watchdog covers the boot before the user space
	 * daemon opens it. WDOG_HW_RUNNING makes the watchdog core ping it
	 * until then (CONFIG_WATCHDOG_HANDLE_BOOT_ENABLED or
	 * watchdog.handle_boot_enabled=1), and watchdog.open_timeout can limit
	 * that time.
	 */
	if (early) {
		if (!timeout)
			wdt->wdt_dev.timeout = RN5T618_WDT_EARLY_TIMEOUT;
		ret = rn5t618_wdt_start(&wdt->wdt_dev);
		if (ret)
			return ret;
		set_bit(WDOG_HW_RUNNING, &wdt->wdt_dev.status);
		wdt->reboot_nb.notifier_call = rn5t618_wdt_reboot;
		ret = devm_register_reboot_notifier(dev, &wdt->reboot_nb);
		if (ret)
			return ret;
		dev_info(dev, "started at probe, timeout %u s\n",
			 wdt->wdt_dev.timeout);
	}

	return devm_watchdog_register_device(dev, &wdt->wdt_dev);
}

static struct platform_driver rn5t618_wdt_driver = {
	.probe = rn5t618_wdt_probe,
	.driver = {
		.name	= DRIVER_NAME,
	},
};

module_platform_driver(rn5t618_wdt_driver);

MODULE_ALIAS("platform:rn5t618-wdt");
MODULE_AUTHOR("Beniamino Galvani <b.galvani@gmail.com>");
MODULE_DESCRIPTION("RN5T618 watchdog driver");
MODULE_LICENSE("GPL v2");
