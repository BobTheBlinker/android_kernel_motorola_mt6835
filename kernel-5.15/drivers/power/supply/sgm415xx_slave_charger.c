// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */
#define pr_fmt(fmt) "[sgm415xx_secondary_chg][%s]" fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include "charger_class.h"
#include "mtk_battery.h"
#include "mtk_charger.h"
#include <linux/iio/consumer.h>
#include "tcpm.h"
#include "tcpci_core.h"
#if IS_ENABLED(CONFIG_HQ_SYSFS_SUPPORT)
#include <linux/hqsysfs.h>
#endif
#include "sgm415xx_charger.h"
#include <linux/syscore_ops.h>
#define SGM4154x_REG_NUM    (0xF)

// static const unsigned int BOOST_VOLT_LIMIT[] = {
// 	4850000, 5000000, 5150000, 5300000
// };

#if (defined(__SGM41542_CHIP_ID__) || defined(__SGM41541_CHIP_ID__)|| defined(__SGM41543_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	1200000, 2000000
};
#else
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	500000, 1200000
};
#endif


#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))

static const unsigned int IPRECHG_CURRENT_STABLE[] = {
	5000, 10000, 15000, 20000, 30000, 40000, 50000, 60000,
	80000, 100000, 120000, 140000, 160000, 180000, 200000, 240000
};

static const unsigned int ITERM_CURRENT_STABLE[] = {
	5000, 10000, 15000, 20000, 30000, 40000, 50000, 60000,
	80000, 100000, 120000, 140000, 160000, 180000, 200000, 240000
};
#endif


static const struct charger_properties sgm4154x_chg_props = {
	.alias_name = SGM4154x_NAME,
};

enum {
	SGM_DP_DM_VOL_HIZ,
	SGM_DP_DM_VOL_0P0,
	SGM_DP_DM_VOL_0P6,
	SGM_DP_DM_VOL_3P3,
};

enum SGM4154x_QC_VOLT {
	QC_20_5000mV,
	QC_20_9000mV,
	QC_20_12000mV,
};

enum vindpm_track {
	SGM4154x_TRACK_DIS,
	SGM4154x_TRACK_200,
	SGM4154x_TRACK_250,
	SGM4154x_TRACK_300,
};

enum {
	SGM4154X_NONSTAND_APPLE_12W = 2400,
	SGM4154X_NONSTAND_APPLE_10W = 2100,
	SGM4154X_NONSTAND_APPLE_5W = 1000,
	SGM4154X_NONSTAND_SAMSUNG_10W = 2000,
};

/**********************************************************
 *
 *   [Global Variable]
 *
 *********************************************************/
static int sgm4154x_set_boost_current_limit(struct charger_device *chg_dev, u32 uA);
static int sgm4154x_enable_charging(struct charger_device *chg_dev, bool en);
static struct charger_device *sgm_chg_dev;

/**********************************************************
 *
 *   [I2C Function For Read/Write sgm4154x]
 *
 *********************************************************/
static int __sgm4154x_read_byte(struct sgm4154x_device *sgm, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(sgm->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int __sgm4154x_write_byte(struct sgm4154x_device *sgm, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(sgm->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
			   val, reg, ret);
		return ret;
	}
	return 0;
}

static int sgm4154x_read_reg(struct sgm4154x_device *sgm, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4154x_read_byte(sgm, reg, data);
	mutex_unlock(&sgm->i2c_rw_lock);

	return ret;
}

#if 0
static int sgm4154x_write_reg(struct sgm4154x_device *sgm, u8 reg, u8 val)
{
	int ret;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4154x_write_byte(sgm, reg, val);
	mutex_unlock(&sgm->i2c_rw_lock);

	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

	return ret;
}
#endif

static int sgm4154x_update_bits(struct sgm4154x_device *sgm, u8 reg,
					u8 mask, u8 val)
{
	int ret;
	u8 tmp;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4154x_read_byte(sgm, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= val & mask;

	ret = __sgm4154x_write_byte(sgm, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&sgm->i2c_rw_lock);
	return ret;
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/

 static int sgm4154x_set_watchdog_timer(struct sgm4154x_device *sgm, int time)
{
	int ret;
	u8 reg_val;

	if (time == 0)
		reg_val = SGM4154x_WDT_TIMER_DISABLE;
	else if (time == 40)
		reg_val = SGM4154x_WDT_TIMER_40S;
	else if (time == 80)
		reg_val = SGM4154x_WDT_TIMER_80S;
	else
		reg_val = SGM4154x_WDT_TIMER_160S;

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_5,
				SGM4154x_WDT_TIMER_MASK, reg_val);

	return ret;
}

 #if 0
 static int sgm4154x_get_term_curr(struct sgm4154x_device *sgm)
{
	int ret;
	u8 reg_val;
	int curr;
	int offset = SGM4154x_TERMCHRG_I_MIN_uA;

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_3, &reg_val);
	if (ret)
		return ret;

	reg_val &= SGM4154x_TERMCHRG_CUR_MASK;
	curr = reg_val * SGM4154x_TERMCHRG_CURRENT_STEP_uA + offset;
	return curr;
}

static int sgm4154x_get_prechrg_curr(struct sgm4154x_device *sgm)
{
	int ret;
	u8 reg_val;
	int curr;
	int offset = SGM4154x_PRECHRG_I_MIN_uA;

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_3, &reg_val);
	if (ret)
		return ret;

	reg_val = (reg_val&SGM4154x_PRECHRG_CUR_MASK)>>4;
	curr = reg_val * SGM4154x_PRECHRG_CURRENT_STEP_uA + offset;
	return curr;
}

static int sgm4154x_get_ichg_curr(struct sgm4154x_device *sgm)
{
	int ret;
	u8 ichg;
	unsigned int curr;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_2, &ichg);
	if (ret)
		return ret;

	ichg &= SGM4154x_ICHRG_I_MASK;
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	if (ichg <= 0x8)
		curr = ichg * 5000;
	else if (ichg <= 0xF)
		curr = 40000 + (ichg - 0x8) * 10000;
	else if (ichg <= 0x17)
		curr = 110000 + (ichg - 0xF) * 20000;
	else if (ichg <= 0x20)
		curr = 270000 + (ichg - 0x17) * 30000;
	else if (ichg <= 0x30)
		curr = 540000 + (ichg - 0x20) * 60000;
	else if (ichg <= 0x3C)
		curr = 1500000 + (ichg - 0x30) * 120000;
	else
		curr = 3000000;
#else
	curr = ichg * SGM4154x_ICHRG_I_STEP_uA;
#endif
	return curr;
}
#endif

static int sgm4154x_set_term_curr(struct sgm4154x_device *sgm, int uA)
{
	u8 reg_val;

	pr_info("uA = %d\n", uA);
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))

	for(reg_val = 1; reg_val < 16 && uA >= ITERM_CURRENT_STABLE[reg_val]; reg_val++)
		;
	reg_val--;
#else
	if (uA < SGM4154x_TERMCHRG_I_MIN_uA)
		uA = SGM4154x_TERMCHRG_I_MIN_uA;
	else if (uA > SGM4154x_TERMCHRG_I_MAX_uA)
		uA = SGM4154x_TERMCHRG_I_MAX_uA;

	reg_val = (uA - SGM4154x_TERMCHRG_I_MIN_uA) / SGM4154x_TERMCHRG_CURRENT_STEP_uA;

	if ( (uA - SGM4154x_TERMCHRG_I_MIN_uA) % SGM4154x_TERMCHRG_CURRENT_STEP_uA )
		reg_val++;
#endif

	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_3,
				  SGM4154x_TERMCHRG_CUR_MASK, reg_val);
}

static int sgm4154x_set_prechrg_curr(struct sgm4154x_device *sgm, int uA)
{
	u8 reg_val;

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	for(reg_val = 1; reg_val < 16 && uA >= IPRECHG_CURRENT_STABLE[reg_val]; reg_val++)
		;
	reg_val--;
#else
	if (uA < SGM4154x_PRECHRG_I_MIN_uA)
		uA = SGM4154x_PRECHRG_I_MIN_uA;
	else if (uA > SGM4154x_PRECHRG_I_MAX_uA)
		uA = SGM4154x_PRECHRG_I_MAX_uA;

	reg_val = (uA - SGM4154x_PRECHRG_I_MIN_uA) / SGM4154x_PRECHRG_CURRENT_STEP_uA;
#endif
	reg_val = reg_val << 4;
	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_3,
				  SGM4154x_PRECHRG_CUR_MASK, reg_val);
}

static int sgm4154x_set_ichrg_curr(struct charger_device *chg_dev, unsigned int uA)
{
	int ret;
	u8 reg_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	pr_info("set curr %d uA\n", uA);
	if (uA < SGM4154x_ICHRG_I_MIN_uA)
		uA = SGM4154x_ICHRG_I_MIN_uA;
	else if ( uA > sgm->init_data.max_ichg)
		uA = sgm->init_data.max_ichg;
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	if (uA <= 40000)
		reg_val = uA / 5000;
	else if (uA <= 110000)
		reg_val = 0x08 + (uA -40000) / 10000;
	else if (uA <= 270000)
		reg_val = 0x0F + (uA -110000) / 20000;
	else if (uA <= 540000)
		reg_val = 0x17 + (uA -270000) / 30000;
	else if (uA <= 1500000)
		reg_val = 0x20 + (uA -540000) / 60000;
	else if (uA <= 2940000)
		reg_val = 0x30 + (uA -1500000) / 120000;
	else
		reg_val = 0x3d;
#else

	reg_val = uA / SGM4154x_ICHRG_I_STEP_uA;
#endif
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_2,
				  SGM4154x_ICHRG_I_MASK, reg_val);

	return ret;
}

static int sgm4154x_set_chrg_volt(struct charger_device *chg_dev, unsigned int chrg_volt)
{
	int ret;
	u8 reg_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	if(IS_ERR_OR_NULL(sgm))
		return -ENODEV;
	if (chrg_volt < SGM4154x_VREG_V_MIN_uV)
		chrg_volt = SGM4154x_VREG_V_MIN_uV;
	else if (chrg_volt > sgm->init_data.max_vreg)
		chrg_volt = sgm->init_data.max_vreg;

	reg_val = (chrg_volt-SGM4154x_VREG_V_MIN_uV) / SGM4154x_VREG_V_STEP_uV;
	reg_val++;
	reg_val = reg_val<<3;
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_4,
				  SGM4154x_VREG_V_MASK, reg_val);
	return ret;
}

static int sgm4154x_get_chrg_volt(struct charger_device *chg_dev,unsigned int *volt)
{
	int ret;
	u8 vreg_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_4, &vreg_val);
	if (ret)
		return ret;

	vreg_val = (vreg_val & SGM4154x_VREG_V_MASK)>>3;

	if (15 == vreg_val)
		*volt = 4352000; //default
	else if (vreg_val < 25)
		*volt = vreg_val*SGM4154x_VREG_V_STEP_uV + SGM4154x_VREG_V_MIN_uV;

	return 0;
}

static int sgm4154x_get_vindpm_offset_os(struct sgm4154x_device *sgm)
{
	int ret;
	u8 reg_val;

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_f, &reg_val);
	if (ret)
		return ret;

	reg_val = reg_val & SGM4154x_VINDPM_OS_MASK;

	return reg_val;
}

static int sgm4154x_set_vindpm_offset_os(struct sgm4154x_device *sgm,u8 offset_os)
{
	int ret;

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_f,
				  SGM4154x_VINDPM_OS_MASK, offset_os);

	if (ret){
		pr_err("fail\n");
		return ret;
	}

	return ret;
}
static int sgm4154x_set_input_volt_lim(struct charger_device *chg_dev, unsigned int vindpm)
{
	int ret;
	unsigned int offset;
	u8 reg_val;
	u8 os_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	dev_info(sgm->dev, "%s  vindpm =%d\n", __func__, vindpm);

	if (vindpm < SGM4154x_VINDPM_V_MIN_uV ||
		vindpm > SGM4154x_VINDPM_V_MAX_uV)
		 return -EINVAL;

	if (vindpm < 5900000){
		os_val = 0;
		offset = 3900000;
	}
	else if (vindpm >= 5900000 && vindpm < 7500000){
		os_val = 1;
		offset = 5900000; //uv
	}
	else if (vindpm >= 7500000 && vindpm < 10500000){
		os_val = 2;
		offset = 7500000; //uv
	}
	else{
		os_val = 3;
		offset = 10500000; //uv
	}

	sgm4154x_set_vindpm_offset_os(sgm,os_val);
	reg_val = (vindpm - offset) / SGM4154x_VINDPM_STEP_uV;

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_6,
			SGM4154x_VINDPM_V_MASK, reg_val);

	return ret;
}

static int sgm4154x_get_input_volt_lim(struct sgm4154x_device *sgm)
{
	int ret = 0;
	int offset = 3900000;
	u8 vlim = 0;
	int temp = 0;

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_6, &vlim);
	if (ret)
		return ret;

	temp = sgm4154x_get_vindpm_offset_os(sgm);
	if (0 == temp)
		offset = 3900000; //uv
	else if (1 == temp)
		offset = 5900000;
	else if (2 == temp)
		offset = 7500000;
	else if (3 == temp)
		offset = 10500000;

	temp = offset + (vlim & 0x0F) * SGM4154x_VINDPM_STEP_uV;
	return temp;
}

static int sgm4154x_set_termination_current(struct charger_device *chg_dev, u32 uA)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	if(IS_ERR_OR_NULL(sgm))
		return -ENODEV;


	return sgm4154x_set_term_curr(sgm, uA);
}

static int sgm4154x_set_input_curr_lim(struct charger_device *chg_dev, unsigned int iindpm)
{
	int ret;
	u8 reg_val;
	u8 reg_remain = 0;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	if (iindpm == 0)
		iindpm = SGM4154x_IINDPM_I_MIN_uA;

	dev_info(sgm->dev, "%s: charge curr = %duA, pd_stop_charge = %d\n", __func__, iindpm, sgm->pd_stop_charge);

	if (iindpm < SGM4154x_IINDPM_I_MIN_uA ||
			iindpm > SGM4154x_IINDPM_I_MAX_uA ||
			sgm->pd_stop_charge)
		return -EINVAL;

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	reg_val = (iindpm-SGM4154x_IINDPM_I_MIN_uA) / SGM4154x_IINDPM_STEP_uA;
#else
	if (iindpm >= SGM4154x_IINDPM_I_MIN_uA && iindpm <= 3100000){//default
		reg_val = (iindpm-SGM4154x_IINDPM_I_MIN_uA) / SGM4154x_IINDPM_STEP_uA;
		reg_remain = (iindpm-SGM4154x_IINDPM_I_MIN_uA) % SGM4154x_IINDPM_STEP_uA;
	}
	else if (iindpm > 3100000 && iindpm < SGM4154x_IINDPM_I_MAX_uA)
		reg_val = 0x1E;
	else
		reg_val = SGM4154x_IINDPM_I_MASK;
#endif
	if(reg_remain != 0){
		reg_val++;
	}
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_0,
				  SGM4154x_IINDPM_I_MASK, reg_val);
	return ret;
}

static int sgm4154x_get_input_curr_lim(struct charger_device *chg_dev,unsigned int *ilim)
{
	int ret;
	u8 reg_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_0, &reg_val);
	if (ret)
		return ret;
	if (SGM4154x_IINDPM_I_MASK == (reg_val & SGM4154x_IINDPM_I_MASK))
		*ilim =  SGM4154x_IINDPM_I_MAX_uA;
	else
		*ilim = (reg_val & SGM4154x_IINDPM_I_MASK)*SGM4154x_IINDPM_STEP_uA + SGM4154x_IINDPM_I_MIN_uA;

	return 0;
}

static int sgm4154x_set_hiz_en (struct charger_device *chg_dev, bool hiz_en)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	if(IS_ERR_OR_NULL(sgm))
		return -ENODEV;

	sgm->hiz_en = hiz_en;
	/* Sycamore code for SYCAMORE-18307 by kangkai4 at 20250312 start */
	if (hiz_en) {
		sgm4154x_set_input_curr_lim(chg_dev, SGM4154x_IINDPM_I_MIN_uA);
	}
	//sgm4154x_enable_charging(chg_dev, !hiz_en);
	/* Sycamore code for SYCAMORE-18307 by kangkai4 at 20250312 end */

	return 0;
}

static int sgm4154x_is_enable_hz (struct charger_device *chg_dev, bool *hiz_en)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	*hiz_en = sgm->hiz_en;
	return 0;
}

static int sgm4154x_enable_charging(struct charger_device *chg_dev,bool en)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	int ret = 0;

	sgm->charge_enabled = en;
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1,
					SGM4154x_CHRG_EN, en ? SGM4154x_CHRG_EN : 0);
	dev_info(sgm->dev, "%s charger %s\n", en ? "enable" : "disable",
					!ret ? "successfully" : "failed");

	return ret;
}

static int sgm4154x_is_enable_charging(struct charger_device *chg_dev, bool *en)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	*en = sgm->charge_enabled;
	return 0;
}

static int sgm4154x_set_recharge_volt(struct sgm4154x_device *sgm, int mV)
{
	u8 reg_val;

	reg_val = (mV - SGM4154x_VRECHRG_OFFSET_mV) / SGM4154x_VRECHRG_STEP_mV;

	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_4,
				  SGM4154x_VRECHARGE, reg_val);
}

static int sgm4154x_set_wdt_rst(struct sgm4154x_device *sgm, bool is_rst)
{
	u8 val;

	if (is_rst)
		val = SGM4154x_WDT_RST_MASK;
	else
		val = 0;
	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1,
				  SGM4154x_WDT_RST_MASK, val);
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
static int sgm4154x_dump_register(struct charger_device *chg_dev)
{

	unsigned char i = 0;
	unsigned int ret = 0;
	unsigned char sgm4154x_reg[SGM4154x_REG_NUM+1] = { 0 };
	u8 buffer[1024] = {0};
	u8 *ptr = buffer;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	if(IS_ERR_OR_NULL(sgm))
		return -ENODEV;

	for (i = 0; i < SGM4154x_REG_NUM + 1; i++) {
		ret = sgm4154x_read_reg(sgm,i, &sgm4154x_reg[i]);
		if (ret == 0)
			ptr += sprintf(ptr, "[0x%0x=0x%0x]",
				   i, sgm4154x_reg[i]);
	}

	pr_err("%s\n", buffer);

	return 0;
}


/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
static int sgm4154x_hw_chipid_detect(struct sgm4154x_device *sgm)
{
	int ret = 0;
	u8 val = 0;
	ret = sgm4154x_read_reg(sgm,SGM4154x_CHRG_CTRL_b,&val);
	if (ret < 0)
	{
		pr_info("read SGM4154x_CHRG_CTRL_b fail\n");
		return ret;
	}
	val = val & SGM4154x_PN_MASK;
	pr_info("Reg[0x0B]=0x%x\n", val);

	return val;
}


static int sgm4154x_reset_watch_dog_timer(struct charger_device
		*chg_dev)
{
	int ret;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	pr_info("charging_reset_watch_dog_timer\n");

	ret = sgm4154x_set_wdt_rst(sgm,0x1);	/* RST watchdog */

	return ret;
}


static int sgm4154x_get_charging_status(struct charger_device *chg_dev,
					   bool *is_done)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	if (sgm->state.chrg_stat == SGM4154x_TERM_CHRG)
		*is_done = true;
	else
		*is_done = false;

	return 0;
}


static int sgm4154x_set_en_timer(struct sgm4154x_device *sgm)
{
	int ret;

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_5,
				SGM4154x_SAFETY_TIMER_EN, SGM4154x_SAFETY_TIMER_EN);

	return ret;
}

static int sgm4154x_set_disable_timer(struct sgm4154x_device *sgm)
{
	int ret;

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_5,
				SGM4154x_SAFETY_TIMER_EN, 0);

	return ret;
}

static int sgm4154x_enable_safetytimer(struct charger_device *chg_dev,bool en)
{
	int ret = 0;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);


	if (en)
		ret = sgm4154x_set_en_timer(sgm);
	else
		ret = sgm4154x_set_disable_timer(sgm);
	return ret;
}

static int sgm4154x_get_is_safetytimer_enable(struct charger_device
		*chg_dev,bool *en)
{
	int ret = 0;
	u8 val = 0;

	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_read_reg(sgm,SGM4154x_CHRG_CTRL_5,&val);
	if (ret < 0)
	{
		pr_info("read SGM4154x_CHRG_CTRL_5 fail\n");
		return ret;
	}
	*en = !!(val & SGM4154x_SAFETY_TIMER_EN);
	return 0;
}

#if (defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
static int sgm4154x_en_pe_current_partern(struct charger_device
		*chg_dev,bool is_up)
{
	int ret = 0;

	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
				SGM4154x_EN_PUMPX, SGM4154x_EN_PUMPX);
	if (ret < 0)
	{
		pr_info("read SGM4154x_CHRG_CTRL_d fail\n");
		return ret;
	}
	if (is_up)
		ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
				SGM4154x_PUMPX_UP, SGM4154x_PUMPX_UP);
	else
		ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
				SGM4154x_PUMPX_DN, SGM4154x_PUMPX_DN);
	return ret;
}
#endif


static int sgm4154x_hw_init(struct sgm4154x_device *sgm)
{
	int ret = 0;
	struct power_supply_battery_info bat_info = { };

	bat_info.constant_charge_current_max_ua =
			SGM4154x_ICHRG_I_DEF_uA;

	bat_info.constant_charge_voltage_max_uv =
			SGM4154x_VREG_V_DEF_uV;

	bat_info.precharge_current_ua =
			SGM4154x_PRECHRG_I_DEF_uA;

	bat_info.charge_term_current_ua =
			SGM4154x_TERMCHRG_I_DEF_uA;

	sgm->init_data.max_ichg =
			SGM4154x_ICHRG_I_MAX_uA;

	sgm->init_data.max_vreg =
			SGM4154x_VREG_V_MAX_uV;

	sgm4154x_set_watchdog_timer(sgm,0);

	ret = sgm4154x_set_ichrg_curr(sgm_chg_dev,
				bat_info.constant_charge_current_max_ua);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_prechrg_curr(sgm, bat_info.precharge_current_ua);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_chrg_volt(sgm_chg_dev,
				bat_info.constant_charge_voltage_max_uv);
	if (ret)
		goto err_out;

/* Sycamore code for SYCAMORE-5257 by zhoujj21 at 20250108 start */
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_f, //8mv ft
				GENMASK(7, 6), (0x1 << 6));
	if (ret)
		goto err_out;
/* Sycamore code for SYCAMORE-5257 by zhoujj21 at 20250108 end */

	ret = sgm4154x_set_term_curr(sgm, bat_info.charge_term_current_ua);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_input_curr_lim(sgm_chg_dev, sgm->init_data.ilim);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_recharge_volt(sgm, 100);//100~200mv
	if (ret)
		goto err_out;

	// OTG current setting
	sgm4154x_set_boost_current_limit(sgm_chg_dev, 1200000); //1.2A

/* Sycamore code for SYCAMORE-4311 by zhoujj21 at 20241210 start */
	sgm4154x_set_disable_timer(sgm);
/* Sycamore code for SYCAMORE-4311 by zhoujj21 at 20241210 end */

	dev_notice(sgm->dev, "ichrg_curr:%d prechrg_curr:%d chrg_vol:%d"
		" term_curr:%d input_curr_lim:%d",
		bat_info.constant_charge_current_max_ua,
		bat_info.precharge_current_ua,
		bat_info.constant_charge_voltage_max_uv,
		bat_info.charge_term_current_ua,
		sgm->init_data.ilim);

	return 0;
err_out:
	return ret;

}

static int sgm4154x_parse_dt(struct sgm4154x_device *sgm)
{
	int ret;

	ret = device_property_read_u32(sgm->dev,
					   "input-voltage-limit-microvolt",
					   &sgm->init_data.vlim);
	if (ret)
		sgm->init_data.vlim = SGM4154x_VINDPM_DEF_uV;

	if (sgm->init_data.vlim > SGM4154x_VINDPM_V_MAX_uV ||
		sgm->init_data.vlim < SGM4154x_VINDPM_V_MIN_uV)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
					   "input-current-limit-microamp",
					   &sgm->init_data.ilim);
	if (ret)
		sgm->init_data.ilim = SGM4154x_IINDPM_DEF_uA;

	if (sgm->init_data.ilim > SGM4154x_IINDPM_I_MAX_uA ||
		sgm->init_data.ilim < SGM4154x_IINDPM_I_MIN_uA)
		return -EINVAL;


	return 0;
}


static int sgm4154x_get_ibus(struct charger_device *chg_dev, u32 *ibus)
{
	int ret;
	u8 reg_val;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_0, &reg_val);
	if (ret)
		return ret;
	if (SGM4154x_IINDPM_I_MASK == (reg_val & SGM4154x_IINDPM_I_MASK))
		*ibus =  SGM4154x_IINDPM_I_MAX_uA;
	else
		*ibus = (reg_val & SGM4154x_IINDPM_I_MASK)*SGM4154x_IINDPM_STEP_uA + SGM4154x_IINDPM_I_MIN_uA;

	if (sgm->charging_done_force_5v)
		*ibus = 0;

	pr_info("ibus = %d\n", *ibus);
	return ret;
}


static int sgm4154x_enable_shipmode(struct charger_device *chg_dev, bool en)
{
	u8 val = 0;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	if(IS_ERR_OR_NULL(sgm))
		return -ENODEV;

	pr_err("en:%d\n", en);

	if (en) {
		val = SGM4154x_BATFET_OFF << SGM4154x_BATFET_DIS_SHIFT;
	} else {
		val = SGM4154x_BATFET_ON << SGM4154x_BATFET_DIS_SHIFT;
	}

	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_7, SGM4154x_BATFET_DIS_MASK,
				   val);
}

static void shipmode_sycore_shutdown(void) {
	struct sgm4154x_device *sgm = charger_get_data(sgm_chg_dev);
    int ret = 0;
	if(IS_ERR_OR_NULL(sgm))
		return;

    if (sgm->shipping_mode_en) {
        pr_info("sgm slave shipmode_sycore_shutdown %d\n", sgm->shipping_mode_en);
        sgm->shipping_mode_en =false;
        ret = sgm4154x_enable_shipmode(sgm->chg_dev, true);
        if (ret < 0)
            pr_info("set ship mode fail\n");
        else
            pr_info("set ship mode pass\n");
    }
}

static struct syscore_ops shipmode_syscore_ops = {
	.shutdown	= shipmode_sycore_shutdown,
};

static int sgm4154x_enable_shipmode_en(struct charger_device *chg_dev, bool en)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	sgm->shipping_mode_en = en;
	pr_info("en:%d\n", en);

	return 0;
}

static int sgm4154x_get_is_power_path_enable(struct charger_device *chg_dev, bool *en){
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	int vlim;

	if (IS_ERR_OR_NULL(sgm))
		return -ENODEV;

	vlim = sgm4154x_get_input_volt_lim(sgm);
	if (vlim < 0)
		return vlim;

	*en = (vlim == VINDPM_MAX) ? false : true;

	return 0;
}

static int sgm4154x_enable_power_path(struct charger_device *chg_dev, bool en)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	u32 mivr;
	mivr = (en ? VINDPM_DEFAULT : VINDPM_MAX);
	sgm4154x_set_input_volt_lim(sgm->chg_dev,mivr);
	return 0;
}

static int sgm4154x_set_boost_current_limit(struct charger_device *chg_dev, u32 uA)
{
	int ret = 0;
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	if (uA == BOOST_CURRENT_LIMIT[0]){
		ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_2, SGM4154x_BOOST_LIM,
					 0);
	}

	else if (uA == BOOST_CURRENT_LIMIT[1]){
		ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_2, SGM4154x_BOOST_LIM,
					 BIT(7));
	}
	return ret;
}

static int sgm4154x_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	switch (event) {
	case EVENT_FULL:
		sgm->charging_done_force_5v = true;
		break;
	case EVENT_RECHARGE:
		sgm->charging_done_force_5v = false;
		break;
	case EVENT_DISCHARGE:
		break;
	default:
		break;
	}
	return 0;
}

static int sgm4154x_enable_vbus(struct regulator_dev *rdev)
{
	int ret = 0;
	struct sgm4154x_device *sgm = charger_get_data(sgm_chg_dev);

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1, SGM4154x_OTG_EN,
					 SGM4154x_OTG_EN);
	dev_info(sgm->dev, " sgm4154x_enable_vbus  ret  =  %d\n", ret);
	sgm4154x_dump_register(sgm->chg_dev);
	return ret;
}

static int sgm4154x_disable_vbus(struct regulator_dev *rdev)
{
	int ret = 0;
	struct sgm4154x_device *sgm = charger_get_data(sgm_chg_dev);

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1, SGM4154x_OTG_EN,
					 0);
	dev_info(sgm->dev, " sgm4154x_disable_vbus  ret  =  %d\n", ret);
	sgm4154x_dump_register(sgm->chg_dev);

	return ret;
}

int sgm4154x_is_enabled_vbus(struct regulator_dev *rdev)
{
	u8 temp = 0;
	struct sgm4154x_device *sgm = charger_get_data(sgm_chg_dev);

	sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_1, &temp);
	pr_err("sgm4154x_is_enabled_vbus temp =  %x\n", temp);
	sgm4154x_dump_register(sgm->chg_dev);
	return (temp&SGM4154x_OTG_EN)? 1 : 0;
}

static int sgm4154x_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;

	pr_info("en:%d\n", en);
	if (en) {
		ret = sgm4154x_enable_vbus(NULL);
	} else {
		ret = sgm4154x_disable_vbus(NULL);
	}
	return ret;
}

static int sgm4154x_pd_stop_charge(struct charger_device *chg_dev, bool enable)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	dev_info(sgm->dev, "%s: pd_stop_charge = %d\n", __func__, enable);

	sgm->pd_stop_charge = enable;
	if(enable){
		sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_0,
					SGM4154x_IINDPM_I_MASK, 0);
	}

	return 0;
}

static struct charger_ops sgm4154x_chg_ops = {
	/* Normal charging */
	.dump_registers = sgm4154x_dump_register,
	.enable = sgm4154x_enable_charging,
	.is_enabled = sgm4154x_is_enable_charging,
	.get_charging_current = NULL,
	.set_charging_current = sgm4154x_set_ichrg_curr,
	.get_input_current = sgm4154x_get_input_curr_lim,
	.set_input_current = sgm4154x_set_input_curr_lim,
	.set_eoc_current = sgm4154x_set_termination_current,
	.get_constant_voltage = sgm4154x_get_chrg_volt,
	.set_constant_voltage = sgm4154x_set_chrg_volt,
	.kick_wdt = sgm4154x_reset_watch_dog_timer,
	.set_mivr = sgm4154x_set_input_volt_lim,
	.is_charging_done = sgm4154x_get_charging_status,
	.enable_hz = sgm4154x_set_hiz_en,
	.is_enable_hz = sgm4154x_is_enable_hz,

	/* Safety timer */
	.enable_safety_timer = sgm4154x_enable_safetytimer,
	.is_safety_timer_enabled = sgm4154x_get_is_safetytimer_enable,

	/* Power path */
	.enable_powerpath = sgm4154x_enable_power_path,
	.is_powerpath_enabled = sgm4154x_get_is_power_path_enable,

	/* OTG */
	.set_boost_current_limit = sgm4154x_set_boost_current_limit,
	.enable_otg = sgm4154x_enable_otg,
	.event = sgm4154x_do_event,
	.get_ibus_adc = sgm4154x_get_ibus,
	.enable_ship = sgm4154x_enable_shipmode_en,

	/* PD2.0 */
	.pd_stop_charge = sgm4154x_pd_stop_charge,

	/* PE+/PE+20 */
#if (defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
	.send_ta_current_pattern = sgm4154x_en_pe_current_partern,
#else
	.send_ta_current_pattern = NULL,
#endif
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = NULL,
	.enable_cable_drop_comp = NULL,
};

static int sgm4154x_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct device *dev = &client->dev;
	struct sgm4154x_device *sgm = NULL;

	char *name = NULL;

	pr_info("\n");

	sgm = devm_kzalloc(dev, sizeof(*sgm), GFP_KERNEL);
	if (!sgm)
		return -ENODEV;

	sgm->client = client;
	sgm->dev = dev;

	mutex_init(&sgm->lock);
	mutex_init(&sgm->i2c_rw_lock);

	i2c_set_clientdata(client, sgm);

	ret = sgm4154x_hw_chipid_detect(sgm);
	if (ret != SGM4154x_PN_ID) {
		pr_info("sgm4154x_hw_chipid_detect failed !!!\n");
		return ret;
	}

	ret = sgm4154x_parse_dt(sgm);
	if (ret)
		return ret;

	name = devm_kasprintf(sgm->dev, GFP_KERNEL, "%s","sgm4154x suspend wakelock");
	sgm->charger_wakelock =	wakeup_source_register(sgm->dev, name);

	/* Register charger device */
	sgm->chg_dev = charger_device_register("secondary_chg",
						&client->dev, sgm,
						&sgm4154x_chg_ops,
						&sgm4154x_chg_props);
	if (IS_ERR_OR_NULL(sgm->chg_dev)) {
		pr_info("charger_device_register failed\n");
		ret = PTR_ERR(sgm->chg_dev);
		return ret;
	}
	sgm_chg_dev=sgm->chg_dev;


	ret = sgm4154x_hw_init(sgm);
	if (ret) {
		dev_err(dev, "sgm4154x_hw_init error\n");
		return ret;
	}


	#if IS_ENABLED(CONFIG_HQ_SYSFS_SUPPORT)
	hq_regiser_hw_info(HWID_SECONDARY_CHARGER, "SGM41513D");
	#endif

	sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_a, SGM4154x_VSYS_STAT,
					 SGM4154x_VSYS_STAT);
	sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_a, SGM4154x_THERM_STAT,
					 SGM4154x_THERM_STAT);
	sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_7, SGM4154X_BATFET_RESET_MASK,
					SGM4154x_BATFET_RESET_OFF);

	sgm->state.chrg_stat = 0;
	sgm->is_hvdcp = false;
	sgm->charging_done_force_5v = false;
	sgm->shipping_mode_en = false;
	sgm->charge_enabled = true;
	sgm->pd_stop_charge = false;


	register_syscore_ops(&shipmode_syscore_ops);

	dev_err(dev, "sgm4154x_driver_probe successfully!\n");

	return ret;
}

static int sgm4154x_charger_remove(struct i2c_client *client)
{
	struct sgm4154x_device *sgm = i2c_get_clientdata(client);

	mutex_destroy(&sgm->lock);
	mutex_destroy(&sgm->i2c_rw_lock);

	return 0;
}

static void sgm4154x_charger_shutdown(struct i2c_client *client)
{
	int ret = 0;
	struct sgm4154x_device *sgm = i2c_get_clientdata(client);
	pr_info("sgm4154x_charger_shutdown\n");

	ret = sgm4154x_enable_charging(sgm->chg_dev, false);
	if (ret) {
		pr_err("Failed to disable charger, ret = %d\n", ret);
	}

	if (sgm->shipping_mode_en)
		sgm4154x_enable_shipmode(sgm->chg_dev, sgm->shipping_mode_en);
}

static const struct i2c_device_id sgm4154x_i2c_ids[] = {
	{ "sgm41541", 0 },
	{ "sgm41542", 1 },
	{ "sgm41543", 2 },
	{ "sgm41543D", 3 },
	{ "sgm41513", 4 },
	{ "sgm41513A", 5 },
	{ "sgm41513D", 6 },
	{ "sgm41516", 7 },
	{ "sgm41516D", 8 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sgm4154x_i2c_ids);

static const struct of_device_id sgm4154x_of_match[] = {
	{ .compatible = "sgm,sgm41543_slave", },
	{ },
};
MODULE_DEVICE_TABLE(of, sgm4154x_of_match);


static struct i2c_driver sgm4154x_driver = {
	.driver = {
		.name = "sgm4154x-slave-charger",
		.of_match_table = sgm4154x_of_match,
	},
	.probe = sgm4154x_driver_probe,
	.remove = sgm4154x_charger_remove,
	.shutdown = sgm4154x_charger_shutdown,
	.id_table = sgm4154x_i2c_ids,
};
module_i2c_driver(sgm4154x_driver);

MODULE_AUTHOR(" qhq <Allen_qin@sg-micro.com>");
MODULE_DESCRIPTION("sgm4154x charger driver");
MODULE_LICENSE("GPL v2");
