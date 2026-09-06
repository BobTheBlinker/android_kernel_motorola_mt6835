
// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 AWINIC Technology CO., LTD
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include "flashlight.h"
#include "flashlight-dt.h"
#include "flashlight-core.h"
#include <linux/pinctrl/consumer.h>

//#define COMPATIBLE_OCP8111A
#define TORCH_MODE_PWM_EN
#ifdef TORCH_MODE_PWM_EN
#include "mtk_pwm.h"
#include "mtk_pwm_hal_pub.h"
#endif

#define AW3641E_DRIVER_VERSION "V1.1.6"

#define CONFIG_OF				1
#define AW3641E_NSEC_TIME		1000
#define AW3641E_NAME			"aw3641e"

#define AW3641E_CHANNEL_NUM		1
#define AW3641E_CHANNEL_CH1		0

#define AW3641E_LEVEL_TORCH		10
#define AW3641E_LEVEL_FLASH		6
#define AW3641E_LEVEL_SIZE		(AW3641E_LEVEL_TORCH+AW3641E_LEVEL_FLASH)

#define AW3641E_PINCTRL_STATE_EN_LOW		"gpio_low_171"
#define AW3641E_PINCTRL_STATE_EN_HIGH		"gpio_high_171"

#define AW3641E_PINCTRL_STATE_FLASH_LOW		"gpio_low_148"
#define AW3641E_PINCTRL_STATE_FLASH_HIGH	"gpio_high_148"

#ifdef TORCH_MODE_PWM_EN
#define PWM_FLASH
#define AW3641E_PWM_NUM						1				//pwm_1
#define MAX_DATA_WIDTH						559

//used for decrease torch current
#define AW3641E_PINCTRL_STATE_PWM_EN		"output_pwm_171"
static struct pinctrl_state *aw3641e_pwm_en;

//used for increase torch current
#define AW3641E_PINCTRL_STATE_PWM_FLASH		"output_pwm_148"
static struct pinctrl_state *aw3641e_pwm_flash;

#endif

static struct pinctrl		*aw3641e_en_pinctrl;
static struct pinctrl_state *aw3641e_en_high;
static struct pinctrl_state *aw3641e_en_low;

static struct pinctrl		*aw3641e_flash_pinctrl;
static struct pinctrl_state *aw3641e_flash_high;
static struct pinctrl_state *aw3641e_flash_low;

static struct hrtimer timer_close;
static struct work_struct aw3641e_work;


static unsigned int g_level;
static int flash_level = -1;
static int torch_level;
static spinlock_t aw3641e_lock;

static unsigned int aw3641e_timeout_ms[AW3641E_CHANNEL_NUM];

#ifdef COMPATIBLE_OCP8111A
/*
	[0:1]  -> torch mode
	42 -> 17%I, 109 -> 43%I

	[2] -> flash mode (timeout 220ms)
	1 -> 100%I
*/
static const unsigned char aw3641e_current_level[AW3641E_LEVEL_SIZE] =
	{42, 109, /*pwm torch mode*/
	 1/*flash mode*/};
#else
static const int aw3641e_current[AW3641E_LEVEL_SIZE] = {
	 80, 130, 180, 231, 281, 332, 382, 433, 483, 508,
	544, 653, 762, 871, 980, 1089
};

/*
	[0:9]  -> torch mode
	0%, 2.5%, 5%, 7.5%, 10%, 12.5%, 15%, 17.5%, 20%, 22.5%

	[10:16] -> flash mode (timeout 1.3s)
	16 -> 30%, 15 -> 40%, 14 -> 50%, 13 -> 60%, 12 -> 70%, 11 -> 80%, 10 -> 90%, 9  -> 100%
*/
static const unsigned char aw3641e_current_level[AW3641E_LEVEL_SIZE] = {
	0, 15, 31, 47, 63, 139, 167, 195, 223, 237,/*pwm torch mode*/
	14, 13, 12, 11, 10, 9/*flash mode*/};
#endif

static void aw3641e_torch_on(void);
static void aw3641e_flash_on(void);
static void aw3641e_flash_off(void);

static int aw3641e_is_torch(int level)
{
	pr_info("%s +.\n", __func__);
	if (level > AW3641E_LEVEL_TORCH - 1)
		return -1;

	return 0;
}

static int aw3641e_verify_level(int level)
{
	pr_info("%s +.\n", __func__);

	if (level <= 0)
		level = 0;
	else if (level >= AW3641E_LEVEL_SIZE - 1)
		level = AW3641E_LEVEL_SIZE - 1;

	return level;
}

#ifdef TORCH_MODE_PWM_EN
static int aw3641e_set_torch_brightness(int level)
{
	int ret = 0;
	struct pwm_spec_config pwm_setting;
	mdelay(3);
	if(IS_ERR(aw3641e_pwm_flash)) {
		pr_info("aw3641e_pwm_flash is NULL.\n");
	} else {
		ret = pinctrl_select_state(aw3641e_en_pinctrl, aw3641e_pwm_flash);
		pr_info("aw3641e_pwm_flash set ret = %d.\n",ret);
	}

	memset(&pwm_setting, 0, sizeof(struct pwm_spec_config));
	pwm_setting.pwm_no = AW3641E_PWM_NUM;
	pwm_setting.mode = PWM_MODE_OLD;
	pwm_setting.clk_src = PWM_CLK_OLD_MODE_BLOCK;
	pwm_setting.clk_div = CLK_DIV1;
#if 0 //fifo mode
	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 3;
	pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = level;
	pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 50;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = 7;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = 7;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;

	pr_info("11pwm_no:%d mode:%d src:%d div:%d STOP_BITPOS_VALUE:%d HDURATION:%d LDURATION:%d SEND_DATA0:%d: SEND_DATA1:%d\n", 
		pwm_setting.pwm_no, pwm_setting.mode, pwm_setting.clk_src, pwm_setting.clk_div, pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE,
		pwm_setting.PWM_MODE_FIFO_REGS.HDURATION, pwm_setting.PWM_MODE_FIFO_REGS.LDURATION,
		pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0, pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1);
#else //old mode
	pwm_setting.PWM_MODE_OLD_REGS.THRESH = level;
	pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = MAX_DATA_WIDTH;
	pwm_setting.PWM_MODE_OLD_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_OLD_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_OLD_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_OLD_REGS.WAVE_NUM = 0;

	pr_info("pwm_no:%d mode:%d src:%d div:%d THRESH:%d DATA_WIDTH:%d\n", pwm_setting.pwm_no, pwm_setting.mode, pwm_setting.clk_src, pwm_setting.clk_div, pwm_setting.PWM_MODE_OLD_REGS.THRESH, pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH);
#endif
	pwm_set_spec_config(&pwm_setting);
	mt_pwm_clk_sel_hal(pwm_setting.pwm_no, CLK_26M);

	return 0;
}

void aw3641e_torch_on(void)
{
	int ret = 0;
	pr_info("%s.torch_level:%d\n", __func__, torch_level);
#ifdef PWM_FLASH
	if(IS_ERR(aw3641e_en_high)) {
		pr_err("aw3641e_en_high is NULL.\n");
	} else {
		ret = pinctrl_select_state(aw3641e_en_pinctrl, aw3641e_en_high);
		pr_info("aw3641e_en_high set ret = %d.\n",ret);
	}

	if (torch_level == 0) {
		if(IS_ERR(aw3641e_flash_low)){
			pr_err("aw3641e_flash_low is NULL.\n");
		} else {
			ret = pinctrl_select_state(aw3641e_flash_pinctrl, aw3641e_flash_low);
			pr_info("aw3641e_flash_low ret = %d.\n",ret);
		}
	} else {
		aw3641e_set_torch_brightness(torch_level);
	}
#else
	if(IS_ERR(aw3641e_flash_low)){
		pr_err("aw3641e_flash_low is NULL.\n");
	} else {
		ret = pinctrl_select_state(aw3641e_flash_pinctrl, aw3641e_flash_low);
		pr_info("aw3641e_flash_low 11ret = %d.\n",ret);
	}

	if(IS_ERR(aw3641e_pwm_en)){
		pr_info("aw3641e_pwm_en is NULL.\n");
	} else {
		ret = pinctrl_select_state(aw3641e_en_pinctrl, aw3641e_pwm_en);
		pr_info("aw3641e_pwm_en set ret = %d.\n",ret);
	}
	aw3641e_set_torch_brightness(torch_level);
#endif

}
#else
void aw3641e_torch_on(void)
{
	int ret = 0;
	pr_info("%s +.\n", __func__);

	if(IS_ERR(aw3641e_flash_low)){
		pr_err("aw3641e_flash_low is NULL.\n");
	} else {
		ret = pinctrl_select_state(aw3641e_flash_pinctrl, aw3641e_flash_low);
		pr_info("aw3641e_flash_low ret = %d.\n",ret);
	}

	if(IS_ERR(aw3641e_en_high)) {
		pr_err("aw3641e_en_high is NULL.\n");
	} else {
		ret = pinctrl_select_state(aw3641e_en_pinctrl, aw3641e_en_high);
		pr_info("aw3641e_en_high end ret = %d.\n",ret);
	}
}
#endif



void aw3641e_flash_on(void)
{
	int i = 0;
	int ret = 0;
#ifdef TORCH_MODE_PWM_EN
	//mt_pwm_disable(0,0);
#endif
	pr_info("%s.flash_level = %d.\n", __func__, flash_level);

	if(IS_ERR(aw3641e_flash_high)){
		pr_err("aw3641e_flash_high is NULL.\n");
	} else {
		ret = pinctrl_select_state(aw3641e_flash_pinctrl, aw3641e_flash_high);
		pr_info("aw3641e_flash_high set ret= %d.\n",ret);
	}

	for (i = 0; i < flash_level - 1; i++) {
		if(IS_ERR(aw3641e_en_high)) {
			pr_err("aw3641e_en_high is NULL.\n");
		} else {
			ret = pinctrl_select_state(aw3641e_en_pinctrl, aw3641e_en_high);
			pr_info("aw3641e_en_high index:%d ret = %d.\n",i, ret);
		}
		udelay(2);

		if(IS_ERR(aw3641e_en_low)) {
			pr_err("aw3641e_en_low is NULL.\n");
		} else {
			ret = pinctrl_select_state(aw3641e_en_pinctrl, aw3641e_en_low);
			pr_info("aw3641e_en_low index:%d ret = %d.\n",i, ret);
		}
		udelay(2);
	}

	if(IS_ERR(aw3641e_en_high)) {
		pr_err("aw3641e_en_high is NULL.\n");
	} else {
		ret = pinctrl_select_state(aw3641e_en_pinctrl, aw3641e_en_high);
		pr_info("aw3641e_en_high end ret = %d.\n",ret);
	}
}

void aw3641e_flash_off(void)
{
	int ret =0;
	pr_info("%s +.\n", __func__);

#ifdef TORCH_MODE_PWM_EN
	mt_pwm_disable(0,0);
#endif

	if(IS_ERR(aw3641e_en_low)) {
		pr_err("aw3641e_en_low is NULL.\n");
	} else {
		ret = pinctrl_select_state(aw3641e_en_pinctrl, aw3641e_en_low);
		pr_info("aw3641e_en_low ret = %d.\n", ret);
	}

	if(IS_ERR(aw3641e_flash_low)){
		pr_err("aw3641e_en_low is NULL.\n");
	} else {
		ret = pinctrl_select_state(aw3641e_flash_pinctrl, aw3641e_flash_low);
		pr_info("aw3641e_en_low ret = %d.\n",ret);
	}
	udelay(500);
}

static void aw3641e_work_disable(struct work_struct *data)
{
	pr_info("%s +.\n", __func__);

	aw3641e_flash_off();
}

static enum hrtimer_restart aw3641e_timer_close(struct hrtimer *timer)
{
	schedule_work(&aw3641e_work);
	return HRTIMER_NORESTART;
}

int aw3641e_timer_start(int channel, ktime_t ktime)
{
	pr_info("%s +.\n", __func__);

	if (channel == AW3641E_CHANNEL_CH1)
		hrtimer_start(&timer_close, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("%s, Error channel\n", __func__);
		return -1;
	}

	return 0;
}

int aw3641e_timer_cancel(int channel)
{
	pr_info("%s +.\n", __func__);

	if (channel == AW3641E_CHANNEL_CH1)
		hrtimer_cancel(&timer_close);
	else {
		pr_err("Error channel\n");
		return -EINVAL;
	}

	return 0;
}

void aw3641e_enable(int channel)
{
	pr_info("%s,g_level = %d.\n", __func__, g_level);

	if (!aw3641e_is_torch(g_level)) {
		/* torch mode */
		aw3641e_torch_on();
	} else {
		/* flash mode */
		aw3641e_flash_on();
	}

}

void aw3641e_disable(int channel)
{
	pr_info("%s +.\n", __func__);

	if (channel == AW3641E_CHANNEL_CH1) {
		aw3641e_flash_off();
	} else {
		pr_err("%s, Error channel\n", __func__);
		return;
	}
}

static int aw3641e_set_level(int channel, int lel)
{
	pr_info("%s +.\n", __func__);

	g_level = aw3641e_verify_level(lel);

	if (channel == AW3641E_CHANNEL_CH1) {
		if (!aw3641e_is_torch(g_level)) {
			torch_level = aw3641e_current_level[g_level];
		} else {
			flash_level = aw3641e_current_level[g_level];
		}
	} else {
		pr_err("%s, Error channel\n", __func__);
		return -EINVAL;
	}

	return 0;
}

/****************************************************************
 *		flashlights platform interface			*
 *****************************************************************/
static int aw3641e_open(void)
{
	return 0;
}

static int aw3641e_release(void)
{
	hrtimer_cancel(&timer_close);
	return 0;
}

static int aw3641e_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	pr_info("%s +.\n", __func__);
	/* modify flash level of fl_arg->arg acrodding your level for test */
	/*
	 * if (fl_arg->arg == 1)
	 *	fl_arg->arg = 10;
	 */
	/* verify channel */
	if (channel < 0 || channel >= AW3641E_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_info("%s, FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				__func__, channel, (int)fl_arg->arg);
		aw3641e_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("%s,FLASH_IOC_SET_DUTY(%d): %d\n",
				__func__, channel, (int)fl_arg->arg);
		aw3641e_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("%s,FLASH_IOC_SET_ONOFF(%d): %d\n",
				__func__, channel, (int)fl_arg->arg);
		if (fl_arg->arg >= 1) {
			if (aw3641e_timeout_ms[channel]) {
				ktime =
				ktime_set(aw3641e_timeout_ms[channel] / 1000,
				(aw3641e_timeout_ms[channel] % 1000) * 1000000);
				aw3641e_timer_start(channel, ktime);
			}
			aw3641e_enable(channel);
		} else {
			aw3641e_disable(channel);
			aw3641e_timer_cancel(channel);
		}
		break;
	//wangqiang
	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = AW3641E_LEVEL_SIZE;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = AW3641E_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = aw3641e_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = aw3641e_current[fl_arg->arg];
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

int aw3641e_set_driver(int set)
{
	/* init chip and set usage count */
	return 0;
}

static ssize_t aw3641e_strobe_store(struct flashlight_arg arg)
{
	aw3641e_set_level(arg.ct, arg.level);
	return 0;
}

static struct flashlight_operations aw3641e_ops = {
	aw3641e_open,
	aw3641e_release,
	aw3641e_ioctl,
	aw3641e_strobe_store,
	aw3641e_set_driver
};

static ssize_t pwm_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret = 0;

	if (!kstrtoul(buf, 0, &val)) {
		if (val < 0) {
			val = 0;
		} else if(val > MAX_DATA_WIDTH) {
			val = MAX_DATA_WIDTH;
		}
		torch_level = val;
		aw3641e_torch_on();
		//for hardware test,if need
		//mdelay(2000);
		//aw3641e_flash_off();
	}

	return ret ? ret : count;
}

static DEVICE_ATTR_WO(pwm_level);

static ssize_t flashMode_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret = 0;

	if (!kstrtoul(buf, 0, &val)) {
		if (val < 1) {
			val = 1;
		} else if(val > 16) {
			val = 16;
		}
		flash_level = val;
		aw3641e_flash_on();
	}

	return ret ? ret : count;
}

static DEVICE_ATTR_WO(flashMode_level);

static struct attribute *aw3641e_attributes[] = {
	&dev_attr_pwm_level.attr,
	&dev_attr_flashMode_level.attr,
	NULL
};

static struct attribute_group aw3641e_attribute_group = {
	.attrs = aw3641e_attributes
};

/********************************************************************
 *			driver probe
 *********************************************************************/
static int aw3641e_probe(struct platform_device *dev)
{
	int ret;
	pr_info("%s Probe start.\n", __func__);

	/* Flashlight EN pin initialization */
	aw3641e_en_pinctrl = devm_pinctrl_get(&dev->dev);
	if (IS_ERR(aw3641e_en_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(aw3641e_en_pinctrl);
		return ret;
	}

	aw3641e_en_high = pinctrl_lookup_state(
			aw3641e_en_pinctrl, AW3641E_PINCTRL_STATE_EN_HIGH);
	if (IS_ERR(aw3641e_en_high)) {
		pr_err("Failed to init (%s)\n",
			AW3641E_PINCTRL_STATE_EN_HIGH);
		ret = PTR_ERR(aw3641e_en_high);
	}

	aw3641e_en_low = pinctrl_lookup_state(
			aw3641e_en_pinctrl, AW3641E_PINCTRL_STATE_EN_LOW);
	if (IS_ERR(aw3641e_en_low)) {
		pr_err("Failed to init (%s)\n", AW3641E_PINCTRL_STATE_EN_LOW);
		ret = PTR_ERR(aw3641e_en_low);
	}

	/* Flashlight FLASH pin initialization */
	aw3641e_flash_pinctrl = devm_pinctrl_get(&dev->dev);
	if (IS_ERR(aw3641e_flash_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(aw3641e_flash_pinctrl);
		return ret;
	}

	aw3641e_flash_high = pinctrl_lookup_state(
			aw3641e_flash_pinctrl, AW3641E_PINCTRL_STATE_FLASH_HIGH);
	if (IS_ERR(aw3641e_flash_high)) {
		pr_err("Failed to init (%s)\n",
			AW3641E_PINCTRL_STATE_FLASH_HIGH);
		ret = PTR_ERR(aw3641e_flash_high);
	}

	aw3641e_flash_low = pinctrl_lookup_state(
			aw3641e_flash_pinctrl, AW3641E_PINCTRL_STATE_FLASH_LOW);
	if (IS_ERR(aw3641e_flash_low)) {
		pr_err("Failed to init (%s)\n", AW3641E_PINCTRL_STATE_FLASH_LOW);
		ret = PTR_ERR(aw3641e_flash_low);
	}

#ifdef TORCH_MODE_PWM_EN
	/* Flashlight en pwm pin initialization */
	aw3641e_pwm_en = pinctrl_lookup_state(
			aw3641e_en_pinctrl, AW3641E_PINCTRL_STATE_PWM_EN);
	if (IS_ERR(aw3641e_pwm_en)) {
		pr_err("Failed to init (%s)\n", AW3641E_PINCTRL_STATE_PWM_EN);
		ret = PTR_ERR(aw3641e_pwm_en);
	}

	/* Flashlight flash pwm pin initialization */
	aw3641e_pwm_flash = pinctrl_lookup_state(
			aw3641e_flash_pinctrl, AW3641E_PINCTRL_STATE_PWM_FLASH);
	if (IS_ERR(aw3641e_pwm_flash)) {
		pr_err("Failed to pwm init (%s)\n", AW3641E_PINCTRL_STATE_PWM_FLASH);
		ret = PTR_ERR(aw3641e_pwm_flash);
	}
#endif
/*
	struct device_node *node = dev->dev.of_node;

	flash_en_gpio = of_get_named_gpio(node, "flash-en-gpio", 0);
	if ((!gpio_is_valid(flash_en_gpio))) {
		pr_err("%s: dts don't provide flash_en_gpio\n", __func__);
		return -EINVAL;
	}
	pr_info("%s prase dts with flash_en_gpio success.\n", __func__);

	flash_sel_gpio = of_get_named_gpio(node, "flash-sel-gpio", 0);
	if ((!gpio_is_valid(flash_sel_gpio))) {
		pr_err("%s: dts don't provide flash_sel_gpio\n", __func__);
		return -EINVAL;
	}
	pr_info("%s prase dts with flash_sel_gpio success.\n", __func__);

	if (devm_gpio_request_one(&dev->dev, flash_en_gpio,
				  GPIOF_DIR_OUT | GPIOF_INIT_LOW,
		      "Flash-En")) {
		pr_err("%s, gpio Flash-En failed\n", __func__);
		return -1;
	}

	if (devm_gpio_request_one(&dev->dev, flash_sel_gpio,
				  GPIOF_DIR_OUT | GPIOF_INIT_LOW,
		      "Flash-SEL")) {
		pr_err("%s, gpio Flash-SEL failed\n", __func__);
		goto err_gpio;
	}
*/
	/* register flashlight operations */
	if (flashlight_dev_register(AW3641E_NAME, &aw3641e_ops)) {
		pr_err("Failed to register flashlight device.\n");
	}

	/* init work queue */
	INIT_WORK(&aw3641e_work, aw3641e_work_disable);

	spin_lock_init(&aw3641e_lock);

	/* init timer close */
	hrtimer_init(&timer_close, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer_close.function = aw3641e_timer_close;
	aw3641e_timeout_ms[AW3641E_CHANNEL_CH1] = 100;

	ret = sysfs_create_group(&dev->dev.kobj, &aw3641e_attribute_group);
	if (ret) {
		dev_info(&dev->dev, "%s error creating sysfs attr files\n", __func__);
		goto err_sysfs;
	}

	pr_info("%s Probe finish.\n", __func__);

	return 0;

err_sysfs:
	sysfs_remove_group(&dev->dev.kobj, &aw3641e_attribute_group);

	return -EINVAL;
}
static int aw3641e_remove(struct platform_device *dev)
{
	flashlight_dev_unregister(AW3641E_NAME);
	sysfs_remove_group(&dev->dev.kobj, &aw3641e_attribute_group);
	//devm_gpio_free(&dev->dev, flash_en_gpio);
	//devm_gpio_free(&dev->dev, flash_sel_gpio);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw3641e_of_match[] = {
	{.compatible = "awinic,aw3641e"},
	{},
};
MODULE_DEVICE_TABLE(of, aw3641e_of_match);
#else
static struct platform_device aw3641e_platform_device[] = {
	{
		.name = AW3641E_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, aw3641e_platform_device);
#endif

static struct platform_driver aw3641e_platform_driver = {
	.probe = aw3641e_probe,
	.remove = aw3641e_remove,
	.driver = {
		.name = AW3641E_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw3641e_of_match,
#endif
	},
};

static int __init aw3641e_flash_init(void)
{
	int ret = 0;

	pr_info("%s driver version %s.\n", __func__, AW3641E_DRIVER_VERSION);

#ifndef CONFIG_OF
	ret = platform_device_register(&aw3641e_platform_device);
	if (ret) {
		pr_err("%s,Failed to register platform device\n", __func__);
		return ret;
	}
#endif
	ret = platform_driver_register(&aw3641e_platform_driver);
	if (ret) {
		pr_err("%s,Failed to register platform driver\n", __func__);
		return ret;
	}
	return 0;
}

static void __exit aw3641e_flash_exit(void)
{
	pr_info("%s +.\n", __func__);

	platform_driver_unregister(&aw3641e_platform_driver);
}

module_init(aw3641e_flash_init);
module_exit(aw3641e_flash_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("shiqiang@awinic.com");
MODULE_DESCRIPTION("GPIO Flash driver");
