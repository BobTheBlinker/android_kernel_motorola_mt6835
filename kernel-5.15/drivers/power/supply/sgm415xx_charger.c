// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */
#define pr_fmt(fmt) "[sgm415xx_primary_chg][%s]" fmt, __func__

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
#include <linux/phy/phy.h>
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

#define R_CHARGER_1				330
#define R_CHARGER_2				39
#define PHY_MODE_BC11_SET 1
#define PHY_MODE_BC11_CLR 2
/* Sycamore code for SYCAMORE-4967 by kangkai4 at 20241226 start */
#define PHY_MODE_DPPULLUP_SET 5
#define PHY_MODE_DPPULLUP_CLR 6
/* Sycamore code for SYCAMORE-4967 by kangkai4 at 20241226 end */

//extern void Charger_Detect_Init(void);
//extern void Charger_Detect_Release(void);

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

static enum power_supply_usb_type sgm4154x_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_ACA,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID,
	POWER_SUPPLY_USB_TYPE_PD,
};

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
static struct power_supply_desc sgm4154x_power_supply_desc;
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

/* Sycamore code for SYCAMORE-3753 by zhoujj21 at 20241213 start */
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
static void sgm4154x_set_term_rang_select(struct sgm4154x_device *sgm, bool en)
{
	sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_ITERM_RANG_MASK, en ? 1 : 0);
	dev_info(sgm->dev, "in charge mode iterm rang %s\n", en ? "default" : " x 6");
}
#endif

static int sgm4154x_set_term_curr(struct sgm4154x_device *sgm, int uA)
{
	u8 reg_val;

	pr_info("uA = %d\n", uA);
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	sgm4154x_set_term_rang_select(sgm, false);
	uA /= 6;

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
/* Sycamore code for SYCAMORE-166 by zhoujj21 at 20241213 end */

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

	dev_info(sgm->dev, "%s disable_powerpath = %d, vindpm =%d\n", __func__, sgm->disable_powerpath, vindpm);
	if (vindpm < SGM4154x_VINDPM_V_MIN_uV ||
		vindpm > SGM4154x_VINDPM_V_MAX_uV ||
			sgm->disable_powerpath)
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

static int sgm4154x_charger_get_vbus(struct charger_device *chg_dev, u32 *vbus)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);
	static struct iio_channel *chan_vbus = NULL;
	int ret = 0;
	int val = 0;
	*vbus = 0;

	if (IS_ERR(chan_vbus) || chan_vbus == NULL) {
		chan_vbus = devm_iio_channel_get(
			&sgm->client->dev, "pmic_vbus");
		if (IS_ERR(chan_vbus) || chan_vbus == NULL) {
			pr_info("not ready!\n");
			return -ENODEV;
		}
	}

	if (!IS_ERR(chan_vbus) && chan_vbus != NULL) {
		ret = iio_read_channel_processed(chan_vbus, &val);
		if (ret < 0)
			pr_info("read fail,ret=%d\n",  ret);
	} else {
		pr_info("chan error %p\n", chan_vbus);
		ret = -EOPNOTSUPP;
	}

	 *vbus = (((R_CHARGER_1 +
			R_CHARGER_2) * 100 * val) /
			R_CHARGER_2) / 100;

	return  ret;
}

static int sgm4154x_get_vbus(void)
{
	u32 vbus = 0;
	if(IS_ERR_OR_NULL(sgm_chg_dev))
		return -ENODEV;
	sgm4154x_charger_get_vbus(sgm_chg_dev, &vbus);
	return vbus;
}

static bool sgm4154x_is_hvdcp(struct charger_device *chg_dev)
{
	struct sgm4154x_device *sgm = dev_get_drvdata(&chg_dev->dev);
	return sgm->is_hvdcp;
}

static int sgm4154x_enable_qc20_hvdcp_9v(struct sgm4154x_device *sgm)
{
	int ret;
	uint8_t dp_val, dm_val;

	//dp and dm connected,dp 0.6V dm 0.6V
	dp_val = 0x2<<3;
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6V
	if (ret){
		return ret;
	}
	dm_val = 0;
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0V
	if (ret){
		return ret;
	}
	/* Sycamore code for SYCAMORE-27150 yuxb1 at 20251124 start */
	mdelay(1600);
	/* Sycamore code for SYCAMORE-27150 yuxb1 at 20251124 end */
	// dp 3.3v and dm 0.6v out 9V
	dp_val = SGM4154x_DP_VSEL_MASK;
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 3.3v
	if (ret)
		return ret;

	dm_val = 0x2<<1;
	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0.6v

	return ret;
}

static int32_t sgm4154x_set_dpdm(
	struct sgm4154x_device *sgm, uint8_t dp_val, uint8_t dm_val)
{
	uint8_t data_reg = 0;

	uint8_t mask = SGM4154x_DP_VSEL_MASK|SGM4154x_DM_VSEL_MASK;

	data_reg  = (dp_val  << SGM4154x_DP_VOLT_SHIFT) & SGM4154x_DP_VSEL_MASK;
	data_reg |= (dm_val  << SGM4154x_DM_VOLT_SHIFT) & SGM4154x_DM_VSEL_MASK;

	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_d,
				  mask, data_reg);

}

 int sgm4154x_charging_set_hvdcp20(
		struct sgm4154x_device *sgm, uint32_t vbus_target)
{
	int32_t ret = 0;

	pr_err("Set vbus target %dv\n", vbus_target);
	switch (vbus_target) {
	case 5000:
		ret = sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_0P6, SGM_DP_DM_VOL_0P0);
		break;
	case 9000:
		ret = sgm4154x_enable_qc20_hvdcp_9v(sgm);
		break;
	case 12000:
		ret = sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_0P6, SGM_DP_DM_VOL_0P6);
		break;
	default:
		ret = sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_HIZ, SGM_DP_DM_VOL_HIZ);
		break;
	}

	return (ret < 0) ? ret : 0;
}

int32_t sgm4154x_charging_enable_hvdcp30(struct sgm4154x_device *sgm, bool enable)
{
//	sgm4154x_dbg("enable %d\n", enable);
	//enter continuous mode DP 0.6, DM 3.3
	return sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_0P6, SGM_DP_DM_VOL_3P3);
}

int32_t sgm4154x_charging_set_hvdcp30(struct sgm4154x_device *sgm, bool increase)
{
	int32_t ret = 0;

//	sgm4154x_dbg("increase %d\n", increase);
	if (increase) {
		//DP 3.3, DM 3.3
		ret = sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_3P3, SGM_DP_DM_VOL_3P3);
		if (ret < 0)
			return ret;
		//need test
		msleep(50);
		//DP 0.6, DM 3.3
		ret = sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_0P6, SGM_DP_DM_VOL_3P3);
		if (ret < 0)
			return ret;
		msleep(50);
	} else {
		//DP 0.6, DM 3.3
		ret = sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_0P6, SGM_DP_DM_VOL_0P6);
		if (ret < 0)
			return ret;
		//need test
		msleep(50);
		//DP 0.6, DM 3.3
		ret = sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_0P6, SGM_DP_DM_VOL_3P3);
		if (ret < 0)
			return ret;
		msleep(50);
	}
	return 0;
}

/* Sycamore code for SYCAMORE-6484 by zhoujj21 at 20250106 start */
/* Sycamore code for SYCAMORE-4930 by zhoujj21 at 20241226 start */
int sgm4154x_qc3p0_request_vbus(struct sgm4154x_device *sgm, int mv)
{
	int count = 0, i;
	int ret = 0;
	int vbus = 0;
/*
	ret = atomic_read(&sgm->qc3p0_done);
	if(!ret){
		dev_err(sgm->dev, "qc3p0 not done\n");
		return -EPERM;
	}
*/
	atomic_set(&sgm->qc3p0_done, 0);

	sgm4154x_charging_enable_hvdcp30(sgm,true);
	msleep(200);

	count  = (mv - sgm->qc_vbus) / 200;
	for (i = 0; i < abs(count); i++) {
		if (count > 0)
			ret |= sgm4154x_charging_set_hvdcp30(sgm, true);
		else
			ret |= sgm4154x_charging_set_hvdcp30(sgm, false);

		vbus = sgm4154x_get_vbus();
		dev_info(sgm->dev, "%s, count = %d, i = %d, mv =%d, sgm->qc_vbus =%d, vbus = %d\n", __func__, count, i, mv, sgm->qc_vbus, vbus);
		if (!sgm->state.online) {
			dev_err(sgm->dev, "vbus_gd become 0, exit process\n");
			ret = -EPERM;
			break;
		}
	}

	if (ret >= 0) {
		sgm->qc_vbus = mv;
		atomic_set(&sgm->qc3p0_done, 1);
		dev_info(sgm->dev, "qc3p0 set successfully\n");
	} else {
		sgm->qc_vbus = 5000;
		atomic_set(&sgm->qc3p0_done, 1);
		dev_info(sgm->dev, "qc3p0 set fail\n");
	}

	return ret;
}
/* Sycamore code for SYCAMORE-4930 by zhoujj21 at 20241226 end */
/* Sycamore code for SYCAMORE-6484 by zhoujj21 at 20250106 end */

static int sgm4154x_get_state(struct sgm4154x_device *sgm,
				 struct sgm4154x_state *state)
{
	u8 chrg_stat;
	u8 fault;
	u8 otg_fault = 0;
	u8 chrg_param_0,chrg_param_1,chrg_param_2;
	int ret;

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_STAT, &chrg_stat);
	if (ret){
		ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_STAT, &chrg_stat);
		if (ret){
			pr_err("read SGM4154x_CHRG_STAT fail\n");
			return ret;
		}
	}
	state->chrg_type = chrg_stat & SGM4154x_VBUS_STAT_MASK;
	state->chrg_stat = chrg_stat & SGM4154x_CHG_STAT_MASK;
	state->online = !!(chrg_stat & SGM4154x_PG_STAT);
	state->therm_stat = !!(chrg_stat & SGM4154x_THERM_STAT);
	state->vsys_stat = !!(chrg_stat & SGM4154x_VSYS_STAT);

	pr_err("chrg_type =%d,chrg_stat =%d online = %d\n", state->chrg_type, state->chrg_stat, state->online);


	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_FAULT, &fault);
	if (ret){
		pr_err("read SGM4154x_CHRG_FAULT fail\n");
		return ret;
	}
	state->chrg_fault = fault;
	otg_fault = fault & SGM4154x_BOOST_FAULT;
	if (otg_fault){
		ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1, SGM4154x_OTG_EN, 0);
	}
	state->ntc_fault = fault & SGM4154x_TEMP_MASK;
	state->health = state->ntc_fault;
	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_0, &chrg_param_0);
	if (ret){
		pr_err("read SGM4154x_CHRG_CTRL_0 fail\n");
		return ret;
	}
	state->hiz_en = !!(chrg_param_0 & SGM4154x_HIZ_EN);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_5, &chrg_param_1);
	if (ret){
		pr_err(" read SGM4154x_CHRG_CTRL_5 fail\n");
		return ret;
	}
	state->term_en = !!(chrg_param_1 & SGM4154x_TERM_EN);

	ret = sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_a, &chrg_param_2);
	if (ret){
		pr_err("read SGM4154x_CHRG_CTRL_a fail\n");
		return ret;
	}
	state->vbus_gd = !!(chrg_param_2 & SGM4154x_VBUS_GOOD);

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

static bool sgm4154x_get_is_pdo(void)
{
	static struct tcpc_device *tcpc_dev;
	if (IS_ERR_OR_NULL(tcpc_dev)) {
		tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
		if(IS_ERR_OR_NULL(tcpc_dev)) {
			pr_info("failed to find tcpc_dev!!!\n");
			return false;
		}
	}
	return  tcpc_dev->pd_port.pe_data.pe_ready;
}
/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/

static int sgm4154x_dpdm_switch_chg(struct device *dev,
						bool switch_to_chg)
{
	struct phy *phy;
	int mode = 0;
	int ret;

	if (dev == NULL) {
		pr_err("%s,dev is null\n", __func__);
		return 0;
	}

	dev_info(dev, "phy_set_mode_ext  %d\n", switch_to_chg);
	mode = switch_to_chg ? PHY_MODE_BC11_SET : PHY_MODE_BC11_CLR;
	phy = phy_get(dev, "usb2-phy");
	if (IS_ERR_OR_NULL(phy)) {
		dev_info(dev, "phy_get fail\n");
		return -EINVAL;
	}

	ret = phy_set_mode_ext(phy, PHY_MODE_USB_DEVICE, mode);
	if (ret)
		dev_info(dev, "phy_set_mode_ext fail\n");

	phy_put(dev, phy);

	return 0;
}

/* Sycamore code for SYCAMORE-4967 by kangkai4 at 20241226 start */
static void Usbphy_Dp_Pullup_Set(struct device *dev)
{
	struct phy *phy;
	int ret;

	if (dev == NULL) {
		pr_err("%s,dev is null\n", __func__);
		return;
	}

	phy = phy_get(dev, "usb2-phy");
	if (IS_ERR_OR_NULL(phy)) {
		dev_info(dev, "%s, phy_get fail\n", __func__);
		return;
	}

	udelay(50);

	ret = phy_set_mode_ext(phy, PHY_MODE_USB_DEVICE, PHY_MODE_DPPULLUP_SET);
	if (ret)
		dev_info(dev, "%s, phy_set_mode_ext fail\n", __func__);

}


static void Usbphy_Dp_Pullup_Clr(struct device *dev)
{
	struct phy *phy;
	int ret;

	if (dev == NULL) {
		pr_err("%s,dev is null\n", __func__);
		return;
	}

	phy = phy_get(dev, "usb2-phy");
	if (IS_ERR_OR_NULL(phy)) {
		dev_info(dev, "%s, phy_get fail\n", __func__);
		return;
	}

	ret = phy_set_mode_ext(phy, PHY_MODE_USB_DEVICE, PHY_MODE_DPPULLUP_CLR);

	if (ret)
		dev_info(dev, "%s, phy_set_mode_ext fail\n", __func__);

	udelay(1);

}
/* Sycamore code for SYCAMORE-4967 by kangkai4 at 20241226 end */

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

static int sgm4154x_force_dpdm(struct sgm4154x_device *sgm)
{

	/* Sycamore code for SYCAMORE-5393 by liugong2 at 20250111 start */
	if (sgm4154x_get_is_pdo()) {
		//Charger_Detect_Init();
		sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_HIZ, SGM_DP_DM_VOL_HIZ);
		pr_info("bc1.2 detect stop, Priority PD/PPS\n");
		return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_7, SGM4154X_FORCE_DPDM_MASK,
				   SGM4154X_FORCE_DPDM);
	}
	sgm4154x_dpdm_switch_chg(sgm->dev, true);
	msleep(5);
	/* Sycamore code for SYCAMORE-5393 by liugong2 at 20250111 end */

	return sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_7, SGM4154X_FORCE_DPDM_MASK,
				   SGM4154X_FORCE_DPDM);
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

static int sgm4154x_set_vindpm_track(struct sgm4154x_device *sgm,
						enum vindpm_track track)
{
	int ret;

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_7,
				SGM4154x_VINDPM_TRACK_MASK, track);

	if (ret < 0)
		pr_err("set vindpm_track fail\n");

	return ret;
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

static enum power_supply_property sgm4154x_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_HEALTH,
};

static int sgm4154x_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_HEALTH:
		return true;
	default:
		return false;
	}
}
static int sgm4154x_charger_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	/* Sycamore code for SYCAMORE-219 by kangkai4 at 20241118 start */
	struct sgm4154x_device *sgm = power_supply_get_drvdata(psy);
	/* Sycamore code for SYCAMORE-219 by kangkai4 at 20241118 end */
	int ret = -EINVAL;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sgm4154x_set_input_curr_lim(sgm_chg_dev, val->intval);
		break;
	/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20250205 start */
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sgm4154x_set_input_volt_lim(sgm_chg_dev, val->intval);
		break;
	/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20250205 end */
	/* Sycamore code for SYCAMORE-219 by kangkai4 at 20241118 start */
	case POWER_SUPPLY_PROP_HEALTH:
		sgm->health_status = val->intval;
		ret = 0;
		break;
	/* Sycamore code for SYCAMORE-219 by kangkai4 at 20241118 end */
	default:
		return -EINVAL;
	}

	return ret;
}

static int sgm4154x_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sgm4154x_device *sgm = power_supply_get_drvdata(psy);
	struct sgm4154x_state state;
	int ret = 0;

	mutex_lock(&sgm->lock);
	state = sgm->state;
	mutex_unlock(&sgm->lock);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!state.chrg_type || (state.chrg_type == SGM4154x_OTG_MODE))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (state.chrg_stat == SGM4154x_TERM_CHRG)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		/* Sycamore code for SYCAMORE-4325by kangkai4 at 20241206 start */
		else if (!sgm->charge_enabled)
		/* Sycamore code for SYCAMORE-4325by kangkai4 at 20241206 end */
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	/* Sycamore code for SYCAMORE-219 by kangkai4 at 20241118 start */
	case POWER_SUPPLY_PROP_HEALTH:
			val->intval = sgm->health_status;
			break;
	/* Sycamore code for SYCAMORE-219 by kangkai4 at 20241118 end */
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch (state.chrg_stat) {
		case SGM4154x_PRECHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case SGM4154x_FAST_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		case SGM4154x_TERM_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case SGM4154x_NOT_CHRGING:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = sgm->psy_usb_type;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = SGM4154x_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = SGM4154x_NAME;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.vbus_gd;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = state.vbus_gd;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = sgm4154x_power_supply_desc.type;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = sgm4154x_get_vbus();
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		break;
	/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20250205 start */
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		sgm4154x_get_input_curr_lim(sgm->chg_dev,&val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		val->intval = sgm4154x_get_input_volt_lim(sgm);
		break;
	/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20250205 end */
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = sgm4154x_get_vbus() * 1000; /* uv */
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		switch (sgm->psy_usb_type) {
			case POWER_SUPPLY_USB_TYPE_SDP: /* 500mA */
				val->intval = 500000;
				break;
			case POWER_SUPPLY_USB_TYPE_CDP: /* 1500mA */
			case POWER_SUPPLY_USB_TYPE_ACA:
				val->intval = 1500000;
				break;
			case POWER_SUPPLY_USB_TYPE_DCP: /* 2000mA */
				val->intval = 2000000;
				break;
			default :
				break;
		}
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static bool sgm4154x_dpdm_detect_is_done(struct sgm4154x_device * sgm)
{
	u8 chrg_stat;
	int ret;

	ret = sgm4154x_read_reg(sgm, SGM4154x_INPUT_DET, &chrg_stat);
	if(ret) {
		dev_err(sgm->dev, "Check DPDM detecte error\n");
	}

	return (chrg_stat&SGM4154x_DPDM_ONGOING)?true:false;
}

/* Sycamore code for SYCAMORE-6484 by zhoujj21 at 20250106 start */
static void sgm4154x_chg_type_hvdcp_work(struct work_struct *work)
{
	int vbus = 0;
	static int hvdcp_detect_count = 0;
	struct sgm4154x_device * sgm =
		container_of(work, struct sgm4154x_device, hvdcp_work.work);
	if(IS_ERR_OR_NULL(sgm)) {
		pr_err("Cann't get sgm \n");
		return ;
	}

	if(sgm4154x_get_is_pdo()) {
		sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_HIZ, SGM_DP_DM_VOL_HIZ);
		hvdcp_detect_count = 0;
		return;
	}

	sgm4154x_charging_set_hvdcp20(sgm, HVDCP20_VBUS);
	msleep(300);

	if (!sgm->state.online) {
		hvdcp_detect_count = 0;
		return;
	}

	vbus = sgm4154x_get_vbus();
	if (vbus > HVDCP_VBUS) {
		pr_info("HVDCP2.0 detect VBUS:%dmV", vbus);
		sgm->qc_vbus = HVDCP20_VBUS;
		sgm->is_hvdcp = true;
		sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_ACA;
		sgm4154x_set_input_volt_lim(sgm->chg_dev, VINDPM_HVDCP);
		power_supply_changed(sgm->charger);
		schedule_delayed_work(&sgm->hvdcp30_work, msecs_to_jiffies(100)); // request qc3.0
	} else {
		pr_info("HVDCP2.0 detect fail VBUS:%dmV", vbus);
		sgm->qc_vbus = DCP_VBUS;
		sgm->is_hvdcp = false;
		hvdcp_detect_count++;
		if (hvdcp_detect_count <= HVDCP_DETECT_COUNT ) {
			//sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_0P6, SGM_DP_DM_VOL_HIZ);
			schedule_delayed_work(&sgm->hvdcp_work, msecs_to_jiffies(1400));
		} else {
			hvdcp_detect_count = 0;
		}
		sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_0P6, SGM_DP_DM_VOL_HIZ);
	}
}

static void sgm4154x_chg_type_hvdcp30_work(struct work_struct *work)
{
	int vbus = 0;
	static int hvdcp_detect_count = 0;
	struct sgm4154x_device * sgm =
		container_of(work, struct sgm4154x_device, hvdcp30_work.work);
	if(IS_ERR_OR_NULL(sgm)) {
		pr_err("Cann't get sgm \n");
		return ;
	}

	if(sgm4154x_get_is_pdo()) {
		sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_HIZ, SGM_DP_DM_VOL_HIZ);
		hvdcp_detect_count = 0;
		return;
	}

	if (hvdcp_detect_count == 0) {
		sgm4154x_qc3p0_request_vbus(sgm, HVDCP30_VBUS);
		msleep(100);
	}

	if (!sgm->state.online) {
		hvdcp_detect_count = 0;
		return;
	}

	vbus = sgm4154x_get_vbus();
	if (vbus > HVDCP20_VBUS + 200)
	{
		pr_info("HVDCP3.0 detect VBUS:%dmV", vbus);
		sgm->qc_vbus = HVDCP30_VBUS;
		sgm->is_hvdcp30 = true;
		hvdcp_detect_count = 0;
		sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_ACA;
		sgm4154x_set_input_volt_lim(sgm->chg_dev, VINDPM_HVDCP);
		power_supply_changed(sgm->charger);
	} else {
		pr_info("HVDCP3.0 detect fail VBUS:%dmV", vbus);
		sgm->qc_vbus = HVDCP20_VBUS;
		sgm->is_hvdcp30 = false;
		hvdcp_detect_count++;
		if (hvdcp_detect_count <= HVDCP_DETECT_COUNT ) {
			schedule_delayed_work(&sgm->hvdcp30_work, msecs_to_jiffies(1400));
		} else {
			sgm4154x_charging_set_hvdcp20(sgm, HVDCP20_VBUS);
			hvdcp_detect_count = 0;
		}
	}
}
/* Sycamore code for SYCAMORE-6484 by zhoujj21 at 20250106 end */

static void charger_monitor_work_func(struct work_struct *work)
{
	int ret = 0;
	struct sgm4154x_device * sgm = NULL;
	struct delayed_work *charge_monitor_work = NULL;
	struct sgm4154x_state state;

	charge_monitor_work = container_of(work, struct delayed_work, work);
	if(charge_monitor_work == NULL) {
		pr_err("Cann't get charge_monitor_work\n");
		return ;
	}
	sgm = container_of(charge_monitor_work, struct sgm4154x_device, charge_monitor_work);
	if(IS_ERR_OR_NULL(sgm)) {
		pr_err("Cann't get sgm \n");
		return ;
	}

	ret = sgm4154x_get_state(sgm, &state);
	if (ret) {
		pr_err("get reg state error\n");
		goto OUT;
	}
	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	if(!sgm->state.vbus_gd) {
		dev_err(sgm->dev, "Vbus not present, disable charge\n");
		sgm4154x_enable_charging(sgm->chg_dev, false);
		goto OUT;
	}
	if(!state.online)
	{
		dev_err(sgm->dev, "Vbus not online\n");
		goto OUT;
	}
	sgm4154x_dump_register(sgm->chg_dev);

OUT:
	schedule_delayed_work(&sgm->charge_monitor_work, 10*HZ);
}

static irqreturn_t sgm4154x_irq_handler_thread(int irq, void *private);
static void sgm4154x_force_detection_dwork_handler(struct work_struct *work)
{
	int ret;
	bool hiz_en = false;
	struct sgm4154x_device *sgm = container_of(work,
										struct sgm4154x_device,
										bc12_run_dwork.work);

	if (!sgm->state.online)
		return;

	if (!sgm->charger_wakelock->active)
		__pm_stay_awake(sgm->charger_wakelock);

	sgm->bc12_rerun_count++;
	sgm->bc12_done = true;

	ret = sgm4154x_force_dpdm(sgm);
	if (ret) {
		pr_err("force dpdm failed(%d)\n", ret);
		return;
	}

	msleep(1000);
	pr_err("force dpdm\n");
	ret = sgm4154x_is_enable_hz(sgm->chg_dev, &hiz_en);
	if (ret) {
		pr_err("get hiz mode fail ret=%d\n", ret);
		__pm_relax(sgm->charger_wakelock);
		return;
	}
	if (hiz_en) {
		pr_info("hiz is enable,return\n");
		__pm_relax(sgm->charger_wakelock);
		return;
	}

	//sgm4154x_irq_handler_thread(sgm->client->irq, sgm);
	__pm_relax(sgm->charger_wakelock);

}

static irqreturn_t sgm4154x_irq_handler_thread(int irq, void *private)
{
	struct sgm4154x_device * sgm = private;
	static int vbus_gd = 0;
	bool pre_vbus_gd = false;
	struct sgm4154x_state state;
	int non_stand_current_limit = 0;
	int ret;

	pr_info("%s\n",__func__);

	ret = sgm4154x_get_state(sgm, &state);
	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	pre_vbus_gd = vbus_gd;
	vbus_gd = sgm->state.online;

	/* Sycamore code for SYCAMORE-5393 by liugong2 at 20250111 start */
	if (!pre_vbus_gd && sgm->state.vbus_gd) {
		dev_err(sgm->dev, "adapter/usb inserted \n");
		sgm4154x_dpdm_switch_chg(sgm->dev, true);
		sgm4154x_enable_charging(sgm->chg_dev, true);
		sgm4154x_set_ichrg_curr(sgm->chg_dev,100000);
		sgm4154x_set_input_curr_lim(sgm->chg_dev,100000);
		sgm4154x_set_vindpm_track(sgm, SGM4154x_TRACK_200);
	}
	/* Sycamore code for SYCAMORE-5393 by liugong2 at 20250111 end */

	if(!pre_vbus_gd && vbus_gd){
		dev_err(sgm->dev, "adapter/usb inserted \n");
		//Charger_Detect_Init();
		sgm4154x_dpdm_switch_chg(sgm->dev, true);
		sgm->bc12_rerun_count = 0;
		sgm4154x_set_ichrg_curr(sgm->chg_dev,100000);
		sgm4154x_set_input_curr_lim(sgm->chg_dev,100000);
		/* Sycamore code for SYCAMORE-4325by kangkai4 at 20241206 start */
		sgm4154x_enable_charging(sgm->chg_dev, true);
		/* Sycamore code for SYCAMORE-4325by kangkai4 at 20241206 end */
		sgm4154x_set_vindpm_track(sgm, SGM4154x_TRACK_200);
		schedule_delayed_work(&sgm->bc12_run_dwork, msecs_to_jiffies(50));
		sgm->qc_vbus = 5000;
		atomic_set(&sgm->qc3p0_done, 0);
		pr_err("SGM4154x charger type: NON_STANDARD\n");
		goto err;
	} else if (pre_vbus_gd && !vbus_gd){
		dev_err(sgm->dev, "adapter/usb removed \n");
		//Charger_Detect_Release();
		sgm->disable_powerpath = 0;
		sgm->pd_stop_charge = false;
		sgm4154x_dpdm_switch_chg(sgm->dev, false);
		/* Sycamore code for SYCAMORE-6484 by zhoujj21 at 20250106 start */
		sgm->is_hvdcp30 = false;
		/* Sycamore code for SYCAMORE-6484 by zhoujj21 at 20250106 end */
		sgm->is_hvdcp = false;
		sgm->charging_done_force_5v = false;
		sgm->non_std_rerun = false;
		sgm->bc12_done = false;
		sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		/* Sycamore code for SYCAMORE-16982 yuxb1 at 20250321 start */
		sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		/* Sycamore code for SYCAMORE-16982 yuxb1 at 20250321 end */
		sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_0,
				  SGM4154x_HIZ_EN, 0);
		cancel_delayed_work_sync(&sgm->bc12_run_dwork);
		sgm4154x_set_term_curr(sgm, SGM4154x_TERMCHRG_I_DEF_uA);
	}

	if(!sgm->state.vbus_gd) {
		dev_err(sgm->dev, "Vbus not present, disable charge\n");
		sgm4154x_enable_charging(sgm->chg_dev, false);
		goto err;
	}

	if(!state.online) {
		dev_err(sgm->dev, "Vbus not online\n");
		goto err;
	}

	if (!sgm->bc12_done) {
		dev_err(sgm->dev, "bc12 is not good\n");
		goto err;
	}

	if(!sgm4154x_dpdm_detect_is_done(sgm)) {
		dev_err(sgm->dev, "DPDM detecte not done, disable charge\n");
		goto err;
	}

	switch(sgm->state.chrg_type) {
		case SGM4154x_USB_SDP:
			sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB;
			sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
			//Charger_Detect_Release();
			sgm4154x_dpdm_switch_chg(sgm->dev, false);
			pr_err("SGM4154x charger type: SDP\n");
			break;

		case SGM4154x_USB_CDP:
			sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
			sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
			pr_err("SGM4154x charger type: CDP\n");
			//Charger_Detect_Release();
			sgm4154x_dpdm_switch_chg(sgm->dev, false);
			/* Sycamore code for SYCAMORE-4967 by kangkai4 at 20241226 start */
			Usbphy_Dp_Pullup_Set(sgm->dev);
			msleep(100);
			Usbphy_Dp_Pullup_Clr(sgm->dev);
			/* Sycamore code for SYCAMORE-4967 by kangkai4 at 20241226 end */
			break;

		case SGM4154x_USB_DCP:
			sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
			pr_err("SGM4154x charger type: DCP\n");
			if(!sgm4154x_get_is_pdo() && !sgm->is_hvdcp) {
				//sgm4154x_set_dpdm(sgm, SGM_DP_DM_VOL_0P6, SGM_DP_DM_VOL_HIZ);
				schedule_delayed_work(&sgm->hvdcp_work, msecs_to_jiffies(100));
			}
			break;
		case SGM4154x_NON_STANDARD:
			if (sgm->non_std_rerun) {
				schedule_delayed_work(&sgm->bc12_run_dwork, HZ);
				sgm->non_std_rerun = false;
			}
			sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			ret = sgm4154x_get_input_curr_lim(sgm->chg_dev,&non_stand_current_limit);
			if (!ret) {
				switch(non_stand_current_limit) {
				case SGM4154x_APPLE_1A:
				   sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
				   break;
				case SGM4154x_APPLE_2A:
				case SGM4154x_APPLE_2P1A:
				case SGM4154x_APPLE_2P4A:
				   sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
				   break;
				default:
				   sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
				   break;
			   }
			}
			break;
		case SGM4154x_UNKNOWN:
			sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			sgm->psy_usb_type = POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID;
			pr_err("SGM4154x charger type: UNKNOWN\n");
			/* Sycamore code for SYCAMORE-9207 by kangkai4 at 20240121 start */
			if ((sgm->bc12_rerun_count < 60) && !sgm4154x_get_is_pdo()) {
				schedule_delayed_work(&sgm->bc12_run_dwork, msecs_to_jiffies(2000));
			/* Sycamore code for SYCAMORE-9207 by kangkai4 at 20240121 end */
				sgm->non_std_rerun = true;
			}
			break;

		default:
			pr_err("SGM4154x charger type: default\n");
			return IRQ_HANDLED;
	}

	sgm4154x_enable_charging(sgm->chg_dev, true);
	sgm4154x_dump_register(sgm->chg_dev);

err:
	power_supply_changed(sgm->charger);
	dev_err(sgm->dev, "Relax wakelock\n");

/* Sycamore code for SYCAMORE-8398 by zhoujj21 at 20250208 start */
	/* cancel delayed when adapter/usb removed */
	if (pre_vbus_gd && !vbus_gd) {
		cancel_delayed_work_sync(&sgm->hvdcp_work);
		cancel_delayed_work_sync(&sgm->hvdcp30_work);
	}
/* Sycamore code for SYCAMORE-8398 by zhoujj21 at 20250208 end */
	return IRQ_HANDLED;
}

static char *sgm4154x_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger",
};

static struct power_supply_desc sgm4154x_power_supply_desc = {
	.name = "charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = sgm4154x_usb_type,
	.num_usb_types = ARRAY_SIZE(sgm4154x_usb_type),
	.properties = sgm4154x_power_supply_props,
	.num_properties = ARRAY_SIZE(sgm4154x_power_supply_props),
	.get_property = sgm4154x_charger_get_property,
	.set_property = sgm4154x_charger_set_property,
	.property_is_writeable = sgm4154x_property_is_writeable,
};

static int sgm4154x_power_supply_init(struct sgm4154x_device *sgm,
							struct device *dev)
{
	struct power_supply_config psy_cfg = { .drv_data = sgm,
						.of_node = dev->of_node, };

	psy_cfg.supplied_to = sgm4154x_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(sgm4154x_charger_supplied_to);

	sgm->charger = devm_power_supply_register(sgm->dev,
						 &sgm4154x_power_supply_desc,
						 &psy_cfg);
	if (IS_ERR(sgm->charger))
		return -EINVAL;

	/* Sycamore code for SYCAMORE-219 by kangkai4 at 20241118 start */
	sgm->health_status = POWER_SUPPLY_HEALTH_GOOD;
	/* Sycamore code for SYCAMORE-219 by kangkai4 at 20241118 end */

	return 0;
}

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

	/* Sycamore code for SYCAMORE-4693 by kangkai4 at 20241216 start */
	ret = sgm4154x_set_recharge_volt(sgm, 200);//100~200mv
	/* Sycamore code for SYCAMORE-4693 by kangkai4 at 20241216 end */
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
	int irq_gpio = 0, irqn = 0;

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

	irq_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio))
	{
		pr_err("%d gpio get failed\n", irq_gpio);
		return -EINVAL;
	}
	ret = gpio_request(irq_gpio, "sgm4154x irq pin");
	if (ret) {
		pr_err("%d gpio request failed\n", irq_gpio);
		return ret;
	}
	gpio_direction_input(irq_gpio);
	irqn = gpio_to_irq(irq_gpio);
	if (irqn < 0) {
		dev_err(sgm->dev, "%d gpio_to_irq failed\n", irqn);
		return irqn;
	}
	sgm->client->irq = irqn;

	sgm->bc12_done = false;

	return 0;
}

static int sgm4154x_enable_vbus(struct regulator_dev *rdev)
{
	int ret = 0;
	struct sgm4154x_device *sgm = charger_get_data(sgm_chg_dev);

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1, SGM4154x_OTG_EN,
					 SGM4154x_OTG_EN);
	dev_info(sgm->dev, "sgm4154x_enable_vbus  ret  =  %d\n", ret);
	sgm4154x_dump_register(sgm->chg_dev);
	return ret;
}

static int sgm4154x_disable_vbus(struct regulator_dev *rdev)
{
	int ret = 0;
	struct sgm4154x_device *sgm = charger_get_data(sgm_chg_dev);

	ret = sgm4154x_update_bits(sgm, SGM4154x_CHRG_CTRL_1, SGM4154x_OTG_EN,
					 0);
	dev_info(sgm->dev, "sgm4154x_disable_vbus  ret  =  %d\n", ret);
	sgm4154x_dump_register(sgm->chg_dev);

	return ret;
}

static int sgm4154x_is_enabled_vbus(struct regulator_dev *rdev)
{
	u8 temp = 0;
	struct sgm4154x_device *sgm = charger_get_data(sgm_chg_dev);

	sgm4154x_read_reg(sgm, SGM4154x_CHRG_CTRL_1, &temp);
	pr_err("sgm4154x_is_enabled_vbus temp =  %x\n", temp);
	sgm4154x_dump_register(sgm->chg_dev);
	return (temp&SGM4154x_OTG_EN)? 1 : 0;
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
        pr_info("sgm shipmode_sycore_shutdown %d\n", sgm->shipping_mode_en);
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
	if (sgm->disable_powerpath == !en)
		return 0;

	mivr = (en ? VINDPM_DEFAULT : VINDPM_MAX);
	sgm4154x_set_input_volt_lim(sgm->chg_dev,mivr);
	sgm->disable_powerpath = !en;

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

/* Sycamore code for SYCAMORE-6484 by zhoujj21 at 20250106 start */
/* Sycamore code for SYCAMORE-188 by zhoujj21 at 20241220 start */
static int sgm4154x_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);

	switch (event) {
	case EVENT_FULL:
		//sgm->charging_done_force_5v = true;
		//if(sgm4154x_is_hvdcp(chg_dev)) {
			//sgm4154x_charging_set_hvdcp20(sgm, 5000);
			//sgm4154x_qc3p0_request_vbus(sgm, HVDCP30_VBUS);
		//}
		break;
	case EVENT_RECHARGE:
		//sgm->charging_done_force_5v = false;
		//if(sgm4154x_is_hvdcp(chg_dev) && sgm4154x_get_vbus() < HVDCP_VBUS)
			//sgm4154x_charging_set_hvdcp20(sgm, 9000);
			//sgm4154x_qc3p0_request_vbus(sgm, HVDCP30_VBUS);
		break;
	case EVENT_DISCHARGE:
		break;
	default:
		break;
	}
	power_supply_changed(sgm->charger);
	return 0;
}

/* Sycamore code for SYCAMORE-188 by zhoujj21 at 20241220 start */
static int sgm4154x_qc_request_vbus(struct charger_device *chg_dev, u32 mv)
{
	struct sgm4154x_device *sgm = charger_get_data(chg_dev);
	int count = 0, i;
	int ret = 0;
	int vbus = 0;
	int vbus_diff = 0;
	static int pre_vbus = 0;

	ret = atomic_read(&sgm->qc3p0_done);
	if(!ret) {
		dev_err(sgm->dev, "qc3p0 not done\n");
		return -EPERM;
	}

	if (sgm->is_hvdcp && !sgm->is_hvdcp30)
	{
		/* Sycamore code for SYCAMORE-8810 by liugong2 at 20250113 start */
		if (mv < HVDCP_VBUS)
			sgm4154x_set_input_volt_lim(sgm->chg_dev, VINDPM_DEFAULT);
		/* Sycamore code for SYCAMORE-8810 by liugong2 at 20250113 end */
		dev_info(sgm->dev, "It's qc2.0 charger\n");
		return sgm4154x_charging_set_hvdcp20(sgm, mv);
	}

	if (!sgm->is_hvdcp30)
	{
		dev_info(sgm->dev, "It's not qc30 charger\n");
		return -EINVAL;
	}
	pre_vbus = sgm->qc_vbus;

	if (mv < HVDCP_VBUS)
		sgm4154x_set_input_volt_lim(sgm->chg_dev, VINDPM_DEFAULT);

	vbus_diff = mv - sgm->qc_vbus;
	count = abs(vbus_diff) /200;

	for (i = 0; i < count; i++) {
		if (vbus_diff > 0)
			ret |= sgm4154x_charging_set_hvdcp30(sgm, true);
		else
			ret |= sgm4154x_charging_set_hvdcp30(sgm, false);
		vbus = sgm4154x_get_vbus();
		dev_info(sgm->dev, "%s, vbus_diff = %d, count = %d, i = %d, mv = %d, sgm->qc_vbus =%d, vbus = %d\n", __func__, vbus_diff, count, i, mv, sgm->qc_vbus, vbus);
	}

	vbus = sgm4154x_get_vbus();
	sgm4154x_set_input_volt_lim(sgm->chg_dev, vbus > HVDCP_VBUS ? VINDPM_HVDCP : VINDPM_DEFAULT);

	if (ret >= 0) {
		sgm->qc_vbus = mv;
		dev_info(sgm->dev, "qc vbus resquest successfully\n");
	} else {
		sgm->qc_vbus = pre_vbus;
		dev_info(sgm->dev, "qc vbus resquest failed\n");
	}

    return ret;
}
/* Sycamore code for SYCAMORE-188 by zhoujj21 at 20241220 end */
/* Sycamore code for SYCAMORE-6484 by zhoujj21 at 20250106 end */
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
	.get_vbus_adc = sgm4154x_charger_get_vbus,
	.is_hvdcp = sgm4154x_is_hvdcp,
	.enable_hz = sgm4154x_set_hiz_en,
	.is_enable_hz = sgm4154x_is_enable_hz,

	/* Safety timer */
	.enable_safety_timer = sgm4154x_enable_safetytimer,
	.is_safety_timer_enabled = sgm4154x_get_is_safetytimer_enable,

	/* Power path */
	.enable_powerpath = sgm4154x_enable_power_path,
	.is_powerpath_enabled = sgm4154x_get_is_power_path_enable,

	/* OTG */
	.enable_otg = sgm4154x_enable_otg,
	.set_boost_current_limit = sgm4154x_set_boost_current_limit,
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

/* Sycamore code for SYCAMORE-188 by zhoujj21 at 20241220 start */
	.qc_request_vbus = sgm4154x_qc_request_vbus,
/* Sycamore code for SYCAMORE-188 by zhoujj21 at 20241220 end */
};


static const struct regulator_ops sgm4154x_vbus_ops = {
	.enable = sgm4154x_enable_vbus,
	.disable = sgm4154x_disable_vbus,
	.is_enabled = sgm4154x_is_enabled_vbus,
};

static const struct regulator_desc sgm4154x_otg_rdesc = {
	.of_match = "usb-otg-vbus",
	.name = "usb-otg-vbus",
	.ops = &sgm4154x_vbus_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5150000,
	.n_voltages = 1,
};

static int sgm4154x_vbus_regulator_register(struct sgm4154x_device *sgm)
{
	struct regulator_config config = {};
	int ret = 0;
	/* otg regulator */
	config.dev = sgm->dev;
	config.driver_data = sgm;
	sgm->otg_rdev = devm_regulator_register(sgm->dev,
			&sgm4154x_otg_rdesc, &config);
	sgm->otg_rdev->constraints->valid_ops_mask |= REGULATOR_CHANGE_STATUS;
	if (IS_ERR(sgm->otg_rdev)) {
		ret = PTR_ERR(sgm->otg_rdev);
		pr_info("register otg regulator failed (%d)\n", ret);
	}
	return ret;
}

/* Sycamore code for OJUNIPER-1221 by zhoujj21 at 20241129 start */
static bool is_shipmode_enable(void)
{
    static struct device_node *np = NULL;
    const char *bootparams;
    static bool state = 0;

    if(np != NULL) {
        return state;
    }

    np = of_find_node_by_path("/chosen");
    of_property_read_string(np, "bootargs", &bootparams);
    if (!bootparams)
        pr_err("%s: failed to get bootargs property\n", __func__);
    else if (strstr(bootparams, "shipmode.state=1")) {
        state = true;
    }

    return state;
}
/* Sycamore code for OJUNIPER-1221 by zhoujj21 at 20241129 end */

static int sgm4154x_driver_probe(struct i2c_client *client, const struct i2c_device_id * id)
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
	sgm->chg_dev = charger_device_register("primary_chg",
						&client->dev, sgm,
						&sgm4154x_chg_ops,
						&sgm4154x_chg_props);
	if (IS_ERR_OR_NULL(sgm->chg_dev)) {
		pr_info("charger_device_register failed\n");
		ret = PTR_ERR(sgm->chg_dev);
		return ret;
	}
	sgm_chg_dev=sgm->chg_dev;

	INIT_DELAYED_WORK(&sgm->charge_monitor_work, charger_monitor_work_func);
	INIT_DELAYED_WORK(&sgm->hvdcp_work, sgm4154x_chg_type_hvdcp_work);
	INIT_DELAYED_WORK(&sgm->bc12_run_dwork, sgm4154x_force_detection_dwork_handler);
	/* Sycamore code for SYCAMORE-6484 by zhoujj21 at 20250106 start */
	INIT_DELAYED_WORK(&sgm->hvdcp30_work, sgm4154x_chg_type_hvdcp30_work);
	sgm->is_hvdcp30 = false;
	/* Sycamore code for SYCAMORE-6484 by zhoujj21 at 20250106 end */
	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sgm4154x_irq_handler_thread,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						dev_name(&client->dev), sgm);
		if (ret)
			return ret;
		enable_irq_wake(client->irq);
	}

	ret = sgm4154x_power_supply_init(sgm, dev);
	if (ret) {
		pr_err("sgm4154x_power_supply_init error\n");
		return ret;
	}

	ret = sgm4154x_hw_init(sgm);
	if (ret) {
		dev_err(dev, "sgm4154x_hw_init error\n");
		return ret;
	}

	ret = sgm4154x_vbus_regulator_register(sgm);
	if (ret) {
		dev_err(dev, "sgm4154x_vbus_regulator_register error\n");
		return ret;
	}

	#if IS_ENABLED(CONFIG_HQ_SYSFS_SUPPORT)
	hq_regiser_hw_info(HWID_PRIMARY_CHARGER, "SGM41513D");
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
/* Sycamore code for OJUNIPER-1221 by zhoujj21 at 20241129 start */
	sgm->shipping_mode_en = is_shipmode_enable() ? true : false;
/* Sycamore code for OJUNIPER-1221 by zhoujj21 at 20241129 end */
	sgm->charge_enabled = true;

	sgm4154x_irq_handler_thread(client->irq, sgm);
	schedule_delayed_work(&sgm->charge_monitor_work, 100);

    register_syscore_ops(&shipmode_syscore_ops);

	dev_err(dev, "sgm4154x_driver_probe successfully!\n");

	return ret;
}

static int sgm4154x_charger_remove(struct i2c_client *client)
{
	struct sgm4154x_device *sgm = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&sgm->charge_monitor_work);

	regulator_unregister(sgm->otg_rdev);

	power_supply_unregister(sgm->charger);

	mutex_destroy(&sgm->lock);
	mutex_destroy(&sgm->i2c_rw_lock);

	return 0;

}

static void sgm4154x_charger_shutdown(struct i2c_client *client)
{
	int ret = 0;
	struct sgm4154x_device *sgm = i2c_get_clientdata(client);
	pr_info("sgm4154x_charger_shutdown\n");

	ret = sgm4154x_enable_charging(sgm->chg_dev, sgm->state.vbus_gd);
	if (ret) {
		pr_err("Failed to disable charger, ret = %d\n", ret);
	}
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
	{ .compatible = "sgm,sgm41541", },
	{ .compatible = "sgm,sgm41542", },
	{ .compatible = "sgm,sgm41543", },
	{ .compatible = "sgm,sgm41543D", },
	{ .compatible = "sgm,sgm41513", },
	{ .compatible = "sgm,sgm41513A", },
	{ .compatible = "sgm,sgm41513D", },
	{ .compatible = "sgm,sgm41516", },
	{ .compatible = "sgm,sgm41516D", },
	{ },
};
MODULE_DEVICE_TABLE(of, sgm4154x_of_match);


static struct i2c_driver sgm4154x_driver = {
	.driver = {
		.name = "sgm4154x-charger",
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
