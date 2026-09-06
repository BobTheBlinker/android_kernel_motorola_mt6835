// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 MediaTek Inc.
 */

#define pr_fmt(fmt) "[fg_mm801310c][%s]" fmt, __func__

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/pm_wakeup.h>
#include "tcpm.h"
#include "tcpci_core.h"
#if IS_ENABLED(CONFIG_HQ_SYSFS_SUPPORT)
#include <linux/hqsysfs.h>
#endif
/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20241231 start */
#include <linux/reboot.h> /* For kernel_power_off() */
#include <linux/iio/consumer.h>
/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20241231 end */
#include "adapter_class.h"

/* Sycamore code for SYCAMORE-4334 by zhoujj21 at 20241213 start */
/* Define macros:
 * 1 : Charging protection is control by kernel layer
 * 0 : Charging protection is control by user layer
 */
#define CHG_PROTECTION_SETTING_ENABLE 1
#define BM_CAPCAITY_THR    50
#define BM2_CAPCAITY_THR  70
/* Sycamore code for SYCAMORE-4334 by zhoujj21 at 20241213 end */

#define DRIVER_VERSION			"1.0.0"
#define REG_CTRL_0			0x00
#define REG_TEMPERATURE			0x06
#define REG_VOLTAGE			0x08
#define REG_CURRENT			0x14
#define REG_TIME_TO_EMPTY_AVG       0x16
#define REG_RSOC			0x2c
#define REG_BLOCKDATAOFFSET		0x3e
#define REG_CYCLECOUNT                      0x2a
#define REG_BLOCKDATA			0x40
#define CMD_FULL_CAPACITY      0x12
#define CMD_CHG_VOLTAGE             0x30
#define CMD_PRODUCT_DATE            0xF2
#define CMD_ACTIVATE_DATE           0xF3
#define CMD_LENOVO_SETTING          0xF4
#define MM8013C_DEFAULT_SOC_VALUE		50
#define MM8013C_DEFAULT_TEMP_VALUE		250
#define MM8013C_DEFAULT_VOLAGE_VALUE		3900
#define MM8013C_DEFAULT_CURRENT_VALUE		500
#define MM8013_DEFAULT_CYCLECOUNT 1
#define REG_HEALTHD_STATUS                  0x2E
#define BATTERY_STRING_MAX 22
#define BATTERY_QMAX			7040
#define BATTERY_CV			4520
#define BATT_COLD_H_THRESH 0
#define BATT_COOL_H_THRESH 150
#define BATT_GOOD_H_THRESH 450
#define BATT_WARM_H_THRESH 500
#define RECHARGE_SOC 95
enum bm_aging_level {
	BM_LEVEL_0,
	BM_LEVEL_1,
	BM_LEVEL_2,
	BM_LEVEL_3,
	BM_LEVEL_4,
	BM_LEVEL_5,
};

struct bm_aging {
	int chg_cv;
	int bm_cv;
};

enum bp_lvl {
	BP_LEVEL_0, // disable bp
	BP_LEVEL_1, // eu
	BP_LEVEL_2, // non-eu
	BP_LEVEL_3, // bypass charging
	BP_LEVEL_4, //bypass_setting
	BP_LEVEL_MAX,//input_suspend
};

struct bp_setting {
	bool enable;
	u16  low_capacity;
	u16  high_capacity;
};

static struct bp_setting bp_setting = {0, 40, 60};
static struct bp_setting bp_setting_eu = {0, 0, 80};
/* Sycamore code for SYCAMORE-4173 by kangkai4 at 20241129 start */
static struct bp_setting bypass_setting = {0, 5, 20};
/* Sycamore code for SYCAMORE-4173 by kangkai4 at 20241129 end */

struct bm_aging aging_table[] = {
	[BM_LEVEL_0] = {4520, 4520},
	[BM_LEVEL_1] = {4520, 4520},
	[BM_LEVEL_2] = {4480, 4460},
	[BM_LEVEL_3] = {4430, 4400},
	[BM_LEVEL_4] = {4330, 4300},
	[BM_LEVEL_5] = {4330, 4300}, //bm v2
};

enum sysfs_property {
	POWER_SUPPLY_PROP_PRODUCT_DATE = 0,
	POWER_SUPPLY_PROP_ACTIVATE_DATE,
	POWER_SUPPLY_PROP_PROTECTION_SETTING,
	POWER_SUPPLY_PROP_PROTECTION_SETTING_EU,
	POWER_SUPPLY_PROP_RECHARGE_SETTING,
	POWER_SUPPLY_PROP_BATTERY_PROTECTION,
	POWER_SUPPLY_PROP_BATTERY_MAINTENANCE,
	POWER_SUPPLY_PROP_BATTERY_MAINTENANCE_V2,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_SOH,
	POWER_SUPPLY_PROP_BATT_ID,
	POWER_SUPPLY_PROP_BM_CV,
	POWER_SUPPLY_PROP_FAKE_CC,
	POWER_SUPPLY_PROP_FAKE_SOC,
	POWER_SUPPLY_PROP_FAKE_TEMP,
	POWER_SUPPLY_PROP_REAL_TYPE,
	POWER_SUPPLY_PROP_BYPASS_SETTING,
	POWER_SUPPLY_PROP_BATTERY_USED_STATE,
};

struct sysfs_info {
	struct device_attribute attr;
	enum sysfs_property prop;
	bool report_uevent;
};

enum hq_batteryID
{
	HQ_BATTERY_UNKNOW,
	HQ_BATTERY_SWD,
	HQ_BATTERY_FMT,
	HQ_BATTERY_NINGDE
};

#if IS_ENABLED(CONFIG_HQ_SYSFS_SUPPORT)
static char battery_manufacturer[HQ_BATTERY_NINGDE + 1][BATTERY_STRING_MAX]={
	"UNKNOW",
	"SUNWODA_4V48_7040MAH",
	"FMT",
	"NINGDE_4V48_7040MAH"
};
#endif

enum mm8013_reg_func {
	PRODUCT_DATE = 0,
	ACTIVATE_DATE,
	PROTECTION_SETTING,
	PROTECTION_SETTING_EU,
	RECHARGE_SETTING,
	BATTERY_MAINTENANCE,
	BATTERY_MAINTENANCE_V20,
	BATTERY_USED,
	BYPASS_SETTING,
};

struct reg_storage{
	u32 reg;
	int base_address;
	int len;
};
#define MM8013_REG_STORAGE(_reg, _offset_address, _len) \
{ \
	.reg = _reg,	\
	.base_address = _offset_address,	\
	.len = _len, \
}

static const struct reg_storage mm8013_reg_storage_ary[] = {
	[PRODUCT_DATE] = MM8013_REG_STORAGE(CMD_PRODUCT_DATE, 0, 3), // 0xF2: byte[0-2]
	[ACTIVATE_DATE] = MM8013_REG_STORAGE(CMD_ACTIVATE_DATE, 0, 3), // 0xF3: byte[0-2]
	[PROTECTION_SETTING] = MM8013_REG_STORAGE(CMD_LENOVO_SETTING, 0 ,3),  // 0xF4: byte[0-2]
	[PROTECTION_SETTING_EU] = MM8013_REG_STORAGE(CMD_LENOVO_SETTING, 3 ,3), // 0xF4: byte[3-5]
	[RECHARGE_SETTING] = MM8013_REG_STORAGE(CMD_LENOVO_SETTING, 6 ,1), // 0xF4:byte[6]
	[BATTERY_MAINTENANCE] = MM8013_REG_STORAGE(CMD_LENOVO_SETTING, 7 ,1), // 0xF4: byte[7]
	[BATTERY_MAINTENANCE_V20] = MM8013_REG_STORAGE(CMD_LENOVO_SETTING, 8 ,1), // 0xF4: byte[8]
	[BATTERY_USED] = MM8013_REG_STORAGE(CMD_LENOVO_SETTING, 9 ,1), // 0xF4: byte[9]
	[BYPASS_SETTING] = MM8013_REG_STORAGE(CMD_LENOVO_SETTING, 10 ,3), // 0xF4: byte[10~12]
};

#define SYSFS_FIELD_RW(_name, _prop, _report)							\
{												\
	.attr = __ATTR(_name, 0644, battery_charge_sysfs_show, battery_charge_sysfs_store),	\
	.prop = _prop,										\
	.report_uevent = _report,								\
}

#define SYSFS_FIELD_RO(_name, _prop, _report)				\
{									\
	.attr = __ATTR(_name, 0444, battery_charge_sysfs_show, NULL),	\
	.prop = _prop,							\
	.report_uevent = _report,					\
}

/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20241231 start */
struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};
/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20241231 end */

struct battery_info {
	int status;
	int health;
	int present;
	int technology;
	int cycle_count;
	int capacity;
	int current_now;
	int current_avg;
	int voltage_now;
	int charger_full;
	int charger_counter;
	int battery_temp;
	int capacoty_level;
	int time_to_full_now;
	int charger_full_design;
	int constant_charge_voltage;
};

struct mm8013_chip {
	struct device *dev;
	struct i2c_client *client;
	struct power_supply_desc battery;
	struct power_supply *mm8013_psy;
	struct power_supply *chg_psy;
	struct power_supply *chg_control_psy;
	struct battery_info bat_data;
	struct delayed_work mm8013_monitor_work;
	struct work_struct usb_type_work;
	struct delayed_work register_hwinfo_work;
	bool is_probe_done;
	struct mutex i2c_rw_lock;
	struct wakeup_source *charger_wakelock;
	u32 cdev_unseal_code;
	struct mutex suspend_lock;
	bool suspended;
	int real_type;
	u32 bootmode;
};

struct mm8013_chip *chip;
int batt_soc = 50;
int batt_temp = 250;
int batt_ibat = 4000;
int batt_vbat = 4000;
int batt_cycle_count = 0;
int batt_ttf = 0;
int batt_volt = 4520;
int fake_cc = -1;
int fake_soc = -1;
int fake_temp = -EINVAL;
int charge_full = 0;
int rechg_precent = 0;
bool batt_maintain_en = false;
bool batt_maintain_v2_en = false;
bool batt_bp_en = false;
bool bypass_chg_en = false;
bool g_input_suspend = false;
bool mm8013_init;
int bypass_settting;
static u32 password = 0x56781234;
static struct power_supply *chg_psy = NULL;
static bool chg_to_full = false;
u32 g_protection_setting_eu = 0;
u32 g_bypass_setting = 0;

static int mm8013_get_charging_enabled(bool *en);
static int mm8013_get_usb_type(void);
static int mm8013_get_charger_online(void);
static int mm8013_get_type(void);
static int mm8013_get_charger_status(void);

static int mm8013_write_reg(struct i2c_client *client, u8 reg, u16 value)
{
	int ret;
	struct mm8013_chip *chip = i2c_get_clientdata(client);

	if(chip->suspended)
		return -EBUSY;

	if (!client->adapter)
		return -ENODEV;

	mutex_lock(&chip->i2c_rw_lock);
	mdelay(4);
	ret = i2c_smbus_write_word_data(client, reg, value);
	if (ret < 0)
		dev_info(&client->dev, "%s: err %d\n", __func__, ret);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int mm8013_read_reg(struct i2c_client *client, u8 reg)
{
	int ret;
	struct mm8013_chip *chip = i2c_get_clientdata(client);

	if(chip->suspended)
		return -EBUSY;

	if (!client->adapter)
		return -ENODEV;

	mutex_lock(&chip->i2c_rw_lock);
	mdelay(4);
	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0)
		dev_info(&client->dev, "%s: err %d\n", __func__, ret);
	mutex_unlock(&chip->i2c_rw_lock);

	return ret;
}

static int mm8013_battery_i2c_bulk_read(struct i2c_client *client, u8 reg, u8 *data, int len)
{
	int ret;

	if (!mm8013_init)
		return -ENODEV;

	if(chip->suspended)
		return -EBUSY;

	if (!client->adapter)
		return -ENODEV;

	mutex_lock(&chip->i2c_rw_lock);
	mdelay(4);
	ret = i2c_smbus_read_i2c_block_data(client, reg, len, data);
	if (ret < 0)
		dev_info(&client->dev, "%s: err %d\n", __func__, ret);
	mutex_unlock(&chip->i2c_rw_lock);

	if (ret != len)
		return -EINVAL;
	return 0;
}

static int mm8013_battery_i2c_bulk_write(struct i2c_client *client, u8 reg, u8 *data, int len)
{
	struct i2c_msg msg;
	u8 buf[33];
	int ret;

	if (!mm8013_init)
		return -ENODEV;

	if(chip->suspended)
		return -EBUSY;

	if (!client->adapter)
		return -ENODEV;

	buf[0] = reg;
	memcpy(&buf[1], data, len);

	msg.buf = buf;
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len + 1;

	mutex_lock(&chip->i2c_rw_lock);
	mdelay(4);
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		dev_info(&client->dev, "%s: err %d\n", __func__, ret);
	mutex_unlock(&chip->i2c_rw_lock);

	if (ret != 1)
		return -EINVAL;
	return 0;
}

static int mm8013_read_date(struct i2c_client *client, int target, u32 unseal_code, int address, int num)
{
	int ret;
	u8 data[32];
	int date;
	int i;

	if (!mm8013_init)
		return -ENODEV;
	/*
	 * Send Unseal code[0]
	 *   Write command 0x00
	 *   and Unseal code data0
	 */
	data[0] = (u8)(unseal_code & 0xFF);
	data[1] = (u8)((unseal_code >> 8) & 0xFF);
	ret = mm8013_battery_i2c_bulk_write(client, 0x00, data, 2);
	if (ret < 0)
	{
		return -1;
	}

	/*
	 * Send Unseal code[1]
	 *   Write command 0x00
	 *   and Unseal code data1
	 */
	data[0] = (u8)((unseal_code >> 16) & 0xFF);
	data[1] = (u8)((unseal_code >> 24) & 0xFF);
	ret = mm8013_battery_i2c_bulk_write(client, 0x00, data, 2);
	if (ret < 0)
	{
		return -1;
	}

	/*
	 * User NVM Write Setting
	 *   Write command 0x61
	 *   and Data 0x00
	 */
	data[0] = (u8)0x00;
	ret = mm8013_battery_i2c_bulk_write(client, 0x61, data, 1);
	if (ret < 0)
	{
		return -1;
	}

	/*
	 * Manufacture A/B Request
	 *   Write command 0x3E
	 *   and Data 0x00F2/0x00F3
	 */
	data[0] = (u8)(target & 0xFF);
	data[1] = (u8)((target >> 8) & 0xFF);
	ret = mm8013_battery_i2c_bulk_write(client, 0x3E, data, 2);
	if (ret < 0)
	{
		return -1;
	}

	/*
	 * Read Manufacture A/B Request
	 *   Read command 0x40
	 *   and get 32bytes data
	 */
	ret = mm8013_battery_i2c_bulk_read(client, 0x40, data, 32);
	if (ret < 0)
	{
		return -1;
	}

	// date = data[0] & 0xFF;			/* Day:	  [0] */
	// date |= (data[1] & 0xFF) << 8;	/* Month: [1] */
	// date |= (data[2] & 0xFF) << 16; /* Year:  [2] */
	date = 0;
	for (i = 0; i < num ; i++)
	{
		date |= (data[address+i] & 0xFF) <<( 8 * i);
	}
	pr_info("date 0x%x, data[3] 0x%x\n", date, data[3]);

	if ((data[3] != 0xff) && (target == CMD_PRODUCT_DATE))
		return 0;

	/*
	 * Seal Set Request
	 *   Write command 0x00
	 *   and Data 0x0020
	 *   -> wait 100msec
	 */
	data[0] = (u8)0x20;
	data[1] = (u8)0x00;
	ret = mm8013_battery_i2c_bulk_write(client, 0x00, data, 2);
	msleep(100);
	if (ret < 0)
	{
		return -1;
	}

	return date;
}
static int mm8013_write_date(struct i2c_client *client, int target, u32 unseal_code, u32 date, int address, int num)
{
	int ret;
	u8 data[32];
	int sum;
	int i;

	if (!mm8013_init)
		return -ENODEV;
	/*
	 * Send Unseal code[0]
	 *   Write command 0x00
	 *   and Unseal code data0
	 */
	data[0] = (u8)(unseal_code & 0xFF);
	data[1] = (u8)((unseal_code >> 8) & 0xFF);
	ret = mm8013_battery_i2c_bulk_write(client, 0x00, data, 2);
	if (ret < 0)
	{
		return -1;
	}

	/*
	 * Send Unseal code[1]
	 *   Write command 0x00
	 *   and Unseal code data1
	 */
	data[0] = (u8)((unseal_code >> 16) & 0xFF);
	data[1] = (u8)((unseal_code >> 24) & 0xFF);
	ret = mm8013_battery_i2c_bulk_write(client, 0x00, data, 2);
	if (ret < 0)
	{
		return -1;
	}

	/*
	 * User NVM Write Setting
	 *   Write command 0x61
	 *   and Data 0x00
	 */
	data[0] = (u8)0x00;
	ret = mm8013_battery_i2c_bulk_write(client, 0x61, data, 1);
	if (ret < 0)
	{
		return -1;
	}

	/*
	 * Manufacture A/B Request
	 *   Write command 0x3E
	 *   and Data 0x00F2/0x00F3
	 */
	data[0] = (u8)(target & 0xFF);
	data[1] = (u8)((target >> 8) & 0xFF);
	ret = mm8013_battery_i2c_bulk_write(client, 0x3E, data, 2);
	if (ret < 0)
	{
		return -1;
	}

	ret = mm8013_battery_i2c_bulk_read(client, 0x40, data, 32);
	if (ret < 0)
	{
		return -1;
	}

	for(i=0; i<num; i++)
	{
		data[address+i] = (u8) (date >> (8 * i) & 0xFF);
		pr_info("date %x\n", data[address+i]);
	}

	/*
	 * Write Manufacture A/B data
	 *   Write command 0x40
	 *   and factory data (any size)
	 */
	// data[0] = (u8)(date & 0xFF);		 /* Day:   [0] */
	// data[1] = (u8)((date >> 8) & 0xFF);	 /* Month: [1] */
	// data[2] = (u8)((date >> 16) & 0xFF); /* Year:  [2] */
	// data[3] = (u8)((date >> 24) & 0xFF); /* Year:  [2] */
	ret = mm8013_battery_i2c_bulk_write(client, 0x40, data, 32);
	if (ret < 0)
	{
		return -1;
	}

	/*
	 * Read Manufacture A/B Request
	 *   Read command 0x40
	 *   and get 32bytes data
	 */
	ret = mm8013_battery_i2c_bulk_read(client, 0x40, data, 32);
	if (ret < 0)
	{
		return -1;
	}

	/*
	 * Calculate byte checksum of Write Data (32bytes)
	 *   Calculate checksum of 32bytes data
	 */
	sum = 0;
	for (i = 0; i < 32; i++)
	{
		sum += (int)data[i];
	}
	sum &= 0xFF;

	/*
	 * Invert checksum data
	 */
	sum = 0xFF - sum;

	/*
	 * Write Inverted checksum data
	 *   Write command 0x60
	 *   and Data <checksum>
	 */
	data[0] = (u8)sum;
	ret = mm8013_battery_i2c_bulk_write(client, 0x60, data, 1);
	if (ret < 0)
	{
		return -1;
	}

	/*
	 * Wait for update
	 *   Wait 100msec
	 */
	msleep(110);

	/*
	 * Seal Set Request
	 *   Write command 0x00
	 *   and Data 0x0020
	 *   -> wait 100msec
	 */
	data[0] = (u8)0x20;
	data[1] = (u8)0x00;
	ret = mm8013_battery_i2c_bulk_write(client, 0x00, data, 2);
	msleep(110);
	if (ret < 0)
	{
		return -1;
	}

	return 0;
}

static int mm8013_read_battery_date(struct mm8013_chip *chip,const struct reg_storage* rand)
{

	int ret = 0;

	chip->cdev_unseal_code = password & 0xFFFFFFFFL;
	pr_info("cdev_unseal_code=0x%x", chip->cdev_unseal_code);

	ret = mm8013_read_date(chip->client, rand->reg, chip->cdev_unseal_code, rand->base_address ,rand->len);
	if (ret < 0)
	{
		dev_info(&chip->client->dev,"failed to read battery target=0x%x\n", rand->reg);
		return -1;
	}
	return ret;
}


static int mm8013_write_battery_date(struct mm8013_chip *chip, const struct reg_storage* rand, u32 data)
{
	int ret;

	if (!mm8013_init)
		return -ENODEV;

	chip->cdev_unseal_code = password & 0xFFFFFFFFL;
	/* Go to UserNVM writing sequence */
	ret = mm8013_write_date(chip->client, rand->reg, chip->cdev_unseal_code, data, rand->base_address ,rand->len);
	if (ret != 0)
	{
		dev_info(&chip->client->dev,"failed to write target=0x%x\n", rand->reg);
		return -1;
	}

	return 0;
}

static int mm8013c10_get_battery_maintenance(void)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_read_battery_date(chip, &mm8013_reg_storage_ary[BATTERY_MAINTENANCE] );

	pr_info("get get_battery_maintenance_v1=%d\n", ret);

	if(ret == 0xFF)
		ret = 0;

	return ret;
}

static int mm8013c10_get_battery_maintenance_v2(void)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_read_battery_date(chip, &mm8013_reg_storage_ary[BATTERY_MAINTENANCE_V20] );

	pr_info("get get_battery_maintenance_v2=%d\n", ret);

	if(ret == 0xFF)
		ret = 0;

	return ret;
}

/* Sycamore code for SYCAMORE-5565 by zhoujj21 at 20241231 start */
static int mm8013c10_set_battery_maintenance(int data)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_write_battery_date(chip, &mm8013_reg_storage_ary[BATTERY_MAINTENANCE], (u32)data);
	pr_info("set_battery_maintenance=%d\n", ret);

	if(ret == 0xFF)
		ret = 0;

	return ret;
}

static int mm8013c10_set_battery_maintenance_v2(int data)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_write_battery_date(chip, &mm8013_reg_storage_ary[BATTERY_MAINTENANCE_V20], (u32)data);
	pr_info("set_battery_maintenance_v2=%d\n", ret);

	if(ret == 0xFF)
		ret = 0;

	return ret;
}
/* Sycamore code for SYCAMORE-5565 by zhoujj21 at 20241231 end */

/* Sycamore code for SYCAMORE-5747 by zhoujj21 at 20250102 start */
/* state value
 * 1 : battery is being used
 * 0 : battery is not being used
*/
static int mm8013c10_get_battery_used_state(void)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_read_battery_date(chip, &mm8013_reg_storage_ary[BATTERY_USED] );

	pr_info("get get_battery_used_state=%d\n", ret);

	if(ret != 0x01)
		ret = 0;

	return ret;
}

static int mm8013c10_set_battery_used_state(int data)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	if ((data != 0x00) && (data != 0x01))
		return -EINVAL;

	ret = mm8013_write_battery_date(chip, &mm8013_reg_storage_ary[BATTERY_USED], (u32)data);
	pr_info("set_battery_used_state=%d, ret=%d\n", data, ret);

	if(ret < 0)
		ret = 0;

	return ret;
}
/* Sycamore code for SYCAMORE-5747 by zhoujj21 at 20250102 end */

static int mm8013c10_get_battery_recharging_setting(void)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_read_battery_date(chip, &mm8013_reg_storage_ary[RECHARGE_SETTING] );

	pr_info("get get_battery_recharging=%d\n", ret);

	if(ret == 0xFF)
		ret = 0;

	return ret;
}

int mm8013_get_battery_manufacatur_date(int *date)
{
	char arry_buf[32] = {0};
	int ret = 0;
	if (!mm8013_init)
		return -ENODEV;

	*date = mm8013_read_battery_date(chip, &mm8013_reg_storage_ary[PRODUCT_DATE] );
	pr_info("get manufacatur_date=0x%x\n", *date);
	sprintf(arry_buf, "%x\n", *date);
	ret = kstrtol(arry_buf, 10, (long *)date);
	pr_info("get manufacatur_date=%d\n", *date);

	return ret;
}

int mm8013_get_battery_start_used_date(void)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_read_battery_date(chip, &mm8013_reg_storage_ary[ACTIVATE_DATE] );
	pr_info("get start_used_date=%d\n", ret);
	if(ret == 0xffffff)
		ret = 0;

	return ret;
}

int mm8013_set_battery_start_used_date(int data)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	pr_info("set battery_start_used_date=%d\n", data);
	ret = mm8013_write_battery_date(chip,  &mm8013_reg_storage_ary[ACTIVATE_DATE] , (u32)data);

	return ret;
}

int mm8013_get_protection_setting(void)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_read_battery_date(chip, &mm8013_reg_storage_ary[PROTECTION_SETTING] );

	pr_info("get production_setting_info=%d\n", ret);

	if(ret == 0xffffff)
		ret = 0;

	return ret;
}

int mm8013_get_protection_setting_eu(void)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_read_battery_date(chip, &mm8013_reg_storage_ary[PROTECTION_SETTING_EU] );

	pr_info("get production_setting_info=%d\n", ret);

	if(ret == 0xffffff)
		ret = 0;

	return ret;
}

int mm8013_get_bypass_setting(void)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_read_battery_date(chip, &mm8013_reg_storage_ary[BYPASS_SETTING] );

	pr_info("get bypass_setting_info=%d\n", ret);

	if(ret == 0xffffff)
		ret = 0;

	return ret;
}

int mm8013_parse_protection_setting(int data)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	bp_setting.enable = (u16)(data / 100000);
	bp_setting.low_capacity = (u16)((data / 1000) % 100);
	bp_setting.high_capacity = (u16)(data % 1000);
	pr_info("[LENOVO_CHG_BP] : bp_setting: [%d, %d, %d]",
	bp_setting.enable, bp_setting.low_capacity, bp_setting.high_capacity);

	return ret;
}

int mm8013_parse_protection_setting_eu(int data)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	bp_setting_eu.enable = (u16)(data / 100000);
	bp_setting_eu.low_capacity = (u16)((data / 1000) % 100);
	bp_setting_eu.high_capacity = (u16)(data % 1000);
	pr_info("[LENOVO_CHG_BP] : bp_setting_eu: [%d, %d, %d]",
	bp_setting_eu.enable, bp_setting_eu.low_capacity, bp_setting_eu.high_capacity);

/* Sycamore code for SYCAMORE-14314 by zhoujj21 at 20250226 start */
	/* when cycle count is divided by 20, allow charging to full */
	if (bp_setting_eu.enable && (bp_setting_eu.high_capacity == 100)) {
		chg_to_full = true;
	} else {
		chg_to_full = false;
	}
/* Sycamore code for SYCAMORE-14314 by zhoujj21 at 20250226 end */
	return ret;
}

/* Sycamore code for SYCAMORE-4173 by kangkai4 at 20241129 start */
int mm8013_parse_bypass_setting(int data)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	bypass_setting.enable = (u16)(data / 100000);
	bypass_setting.low_capacity = (u16)((data / 1000) % 100);
	bypass_setting.high_capacity = (u16)(data % 1000);
	pr_info("[LENOVO_CHG_BP] : bypass_setting: [%d, %d, %d]",
	bypass_setting.enable, bypass_setting.low_capacity, bypass_setting.high_capacity);

	return ret;
}
/* Sycamore code for SYCAMORE-4173 by kangkai4 at 20241129 end */

int mm8013_set_protection_setting(int data)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_write_battery_date(chip, &mm8013_reg_storage_ary[PROTECTION_SETTING] , (u32)data);

	return ret;
}

int mm8013_set_protection_setting_eu(int data)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_write_battery_date(chip, &mm8013_reg_storage_ary[PROTECTION_SETTING_EU] , (u32)data);

	return ret;
}

int mm8013_set_bypass_setting(int data)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_write_battery_date(chip, &mm8013_reg_storage_ary[BYPASS_SETTING] , (u32)data);

	return ret;
}

int mm8013_get_recharge_setting(void)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	ret = mm8013_read_battery_date(chip, &mm8013_reg_storage_ary[RECHARGE_SETTING] );
	pr_info("get recharge_setting_info=%d\n", ret);
	if (ret == 0xff)
		ret = 0;

	return ret;
}

int mm8013_set_recharge_setting(int data)
{
	int ret = 0;

	if (!mm8013_init)
		return -ENODEV;

	pr_info("set recharge_setting_info=%d\n", data);
	ret = mm8013_write_battery_date(chip, &mm8013_reg_storage_ary[RECHARGE_SETTING] , (u32)data);

	return ret;
}

static int mm8013_get_soc(int *val)
{
	int soc = 0;
	*val = batt_soc;
	if(!mm8013_init)
		return -ENODEV;

	if (fake_soc > 0 && fake_soc <= 100) {
		*val = fake_soc;
		return 0;
	}

	soc = mm8013_read_reg(chip->client, REG_RSOC);
	*val = (soc >= 0) ? soc : *val;

	return 0;
}

static int mm8013_get_full_capacity(int *val)
{
	int full_capacity = 0;

	if(!mm8013_init)
		return -ENODEV;

	full_capacity = mm8013_read_reg(chip->client, CMD_FULL_CAPACITY);
	*val = full_capacity > 0 ? full_capacity : *val;

	return 0;
}

/* Sycamore code for SYCAMORE-4326 by zhoujj21 at 20241223 start */
static int mm8013_get_ttf(int *result)
{
	static int pre_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	static int pre_ibat = 4000;
	static int count = 0;
	*result = batt_ttf;

	if (!mm8013_init)
		return -ENODEV;

	if (charge_full == 0)
		mm8013_get_full_capacity(&charge_full);

	// ttf(sec) = (capacity(mAh) / ibat(mA) ) * 3600(sec/h)
	if ((batt_ibat <= 0) || (pre_ibat <= 0) || (batt_soc == 100) || (pre_type != chip->real_type)) {
		*result = 0;
		 count = 0;
	} else if (((batt_ibat - pre_ibat) < 200) && ((batt_ibat - pre_ibat) > -200) && (pre_ibat > 200)) {
		if (count++ > 4) {
			*result =  (u32)((charge_full  * 36 * (100 - batt_soc)) / batt_ibat) ;
			count = 0;
		}
	}

	pre_ibat = batt_ibat;
	pre_type = chip->real_type;

	return 0;
}
/* Sycamore code for SYCAMORE-4326 by zhoujj21 at 20241223 end */

/* Sycamore code for SYCAMORE-337 by zhoujj21 at 20241119 start */
static int mm8013_get_tte(int *val)
{
	int tte = 0;
	*val = 600; // 10h
	if(!mm8013_init)
		return -ENODEV;

	tte = mm8013_read_reg(chip->client, REG_TIME_TO_EMPTY_AVG);
	if (tte >= 0)
		*val = tte;

	return 0;
}
/* Sycamore code for SYCAMORE-337 by zhoujj21 at 20241119 end */

static int mm8013_get_cycle_count(int *val)
{
	int cycle_count = 0;
	*val = batt_cycle_count;
	if(!mm8013_init)
		return -ENODEV;

	if (fake_cc > 0) {
		*val = fake_cc;
		return 0;
	}

	cycle_count = mm8013_read_reg(chip->client, REG_CYCLECOUNT);
	if (cycle_count >= 0)
		*val = cycle_count;

	return 0;
}

static int mm8013_get_voltage(int *val)
{
	int volt = 0;
	*val = batt_vbat;
	if(!mm8013_init)
		return -ENODEV;

	volt = mm8013_read_reg(chip->client, REG_VOLTAGE);
	*val = (volt >= 0) ? volt : *val;
	return 0;
}

static int mm8013_get_current(int *val)
{
	int curr = 0;
	*val = batt_ibat;
	if(!mm8013_init)
		return -ENODEV;

	curr = mm8013_read_reg(chip->client, REG_CURRENT);
	if (curr > 32767)
		curr -= 65536;
	*val = curr;

	return 0;
}

static int mm8013_get_temperature(int *val)
{
	int temp = 0;
	*val = batt_temp;
	if(!mm8013_init)
		return -ENODEV;

	if (fake_temp != -EINVAL) {
		*val = fake_temp;
		return 0;
	}

	temp = mm8013_read_reg(chip->client, REG_TEMPERATURE);
	*val = (temp >= 0) ? (temp - 2731) : *val;

	return 0;
}

static int mm8013_set_volt(int req)
{
	int ret = 0;

	if(!mm8013_init)
		return -ENODEV;

	ret = mm8013_write_reg(chip->client, CMD_CHG_VOLTAGE, req);
	if (ret < 0) {
		pr_info("set CMD_CHG_VOLTAGE<%d> failed!\n", ret);
		return ret;
	}
	pr_info("set CMD_CHG_VOLTAGE<%d> req:%d\n", ret, req);

	return ret;
}

static int mm8013_get_volt(int *val)
{
	int max_volt = 0;

	if(!mm8013_init)
		return -ENODEV;

	max_volt = mm8013_read_reg(chip->client, CMD_CHG_VOLTAGE);
	if (max_volt < 0) {
		*val = BATTERY_CV;
		pr_info("[%s] get CMD_CHG_VOLTAGE failed!\n",
				__func__);
		return max_volt;
	}
	*val = max_volt;
	return 0;
}

/* Sycamore code for SYCAMORE-5752 by zhoujj21 at 20241231 start */
static int mm8013_get_battery_healthd(int *val)
{
	int soh = 0;

	if(!mm8013_init)
		return -ENODEV;

	soh = mm8013_read_reg(chip->client, REG_HEALTHD_STATUS);
	if(soh < 0) {
		*val = 100;
		return soh;
	}
	/* Sycamore code for SYCAMORE-24456 by kangkai4 at 20250818 start */
	*val = soh + 3;
	/* Sycamore code for SYCAMORE-24456 by kangkai4 at 20250818 end */

	if (*val > 100)
		*val = 100;

	return 0;
}
/* Sycamore code for SYCAMORE-5752 by zhoujj21 at 20241231 end */

static int mm8013_get_bat_id(void)
{
	int ret = 0;
	int id;

	if(!mm8013_init)
		return HQ_BATTERY_UNKNOW;


	ret = mm8013_write_reg(chip->client, 0x0, 0x08);
	if (ret < 0)
		pr_err("write %d error!\n", 0x0);

	ret = mm8013_read_reg(chip->client, 0x00);
	pr_info("batt_id:%x!\n", ret);

	switch(ret)
	{
		case 0x0101:
			id = HQ_BATTERY_SWD;break;
		case 0x0102:
			id = HQ_BATTERY_FMT;break;
		case 0x0103:
			id = HQ_BATTERY_NINGDE;break;
		default:
			id = HQ_BATTERY_UNKNOW;
	}

	return id;
}

static int mm8013_checkdevice(void)
{
	int ret;
	int count = 3;

	while(count--){
		ret = mm8013_write_reg(chip->client, 0x0, 0x0008);
		if (ret < 0)
			pr_err("write %d error!\n", 0x0);

		ret = mm8013_read_reg(chip->client, 0x0);
		if (ret > 0)
			break;

	}
	pr_err("count=%d,ret=0x%02X\n", count, ret);

	return ret;
}

static enum power_supply_property mm8013_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
};

int check_cap_level(int uisoc)
{
	if (uisoc >= 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (uisoc >= 80 && uisoc < 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	else if (uisoc >= 20 && uisoc < 80)
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	else if (uisoc > 0 && uisoc < 20)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (uisoc == 0)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
}

static int get_battery_health(void)
{
	int battery_status = POWER_SUPPLY_HEALTH_COLD;
	if (batt_temp < BATT_COLD_H_THRESH)
		battery_status = POWER_SUPPLY_HEALTH_COLD;
	else if (batt_temp < BATT_COOL_H_THRESH)
		battery_status = POWER_SUPPLY_HEALTH_COOL;
	else if (batt_temp < BATT_GOOD_H_THRESH)
		battery_status = POWER_SUPPLY_HEALTH_GOOD;
	else if (batt_temp <= BATT_WARM_H_THRESH)
		battery_status = POWER_SUPPLY_HEALTH_WARM;
	else
		battery_status = POWER_SUPPLY_HEALTH_OVERHEAT;
	return battery_status;
}

static int mm8013_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret  = 0;
	union power_supply_propval online, status, vbus;
	bool charging_enabled = false;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = batt_soc;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mm8013_get_voltage(&val->intval);
		val->intval = ((val->intval) * 1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		mm8013_get_current(&val->intval);
		val->intval = ((val->intval) * 1000);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		mm8013_get_temperature(&val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	/* mtk add */
	case POWER_SUPPLY_PROP_STATUS:
		if (IS_ERR_OR_NULL(chip->chg_psy)) {
			chip->chg_psy =
				power_supply_get_by_name("charger");
			pr_info("[%s]: get charger phandle fail\n", __func__);
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		}
		if (!IS_ERR_OR_NULL(chip->chg_psy)) {
			power_supply_get_property(chip->chg_psy,
				POWER_SUPPLY_PROP_ONLINE, &online);

			power_supply_get_property(chip->chg_psy,
				POWER_SUPPLY_PROP_STATUS, &status);

			power_supply_get_property(chip->chg_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbus);

			mm8013_get_charging_enabled(&charging_enabled);

			/* Sycamore code for SYCAMORE-4930 by zhoujj21 at 20241226 start */
			if (!online.intval) {
				batt_ttf = 0;
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			}
			/* Sycamore code for SYCAMORE-4655 by zhoujj21 at 20241213 start */
			else if (bypass_setting.enable && batt_soc >= bypass_setting.high_capacity)
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			else if (bp_setting.enable && batt_soc >= bp_setting.high_capacity)
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			else if (bp_setting_eu.enable && batt_soc >= bp_setting_eu.high_capacity && chg_to_full == false)
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			/* Sycamore code for SYCAMORE-4655 by zhoujj21 at 20241213 end */
			/* Sycamore code for SYCAMORE-20670 by kangkai4 at 20250415 start */
			else if (status.intval == POWER_SUPPLY_STATUS_FULL) {
				if (batt_soc != 100)
					val->intval = POWER_SUPPLY_STATUS_CHARGING;
				else
					val->intval = status.intval;
			} else if (vbus.intval > 4400 && charging_enabled)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval  = status.intval;
			/* Sycamore code for SYCAMORE-20670 by kangkai4 at 20250415 end */
			/* Sycamore code for SYCAMORE-4930 by zhoujj21 at 20241226 end */
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_battery_health();
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = batt_cycle_count;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = charge_full *1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		mm8013_get_soc(&val->intval);
		val->intval = (val->intval) * ((BATTERY_QMAX * 1000) / 100);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = check_cap_level(batt_soc);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = check_cap_level(batt_soc);
		if ((ret == POWER_SUPPLY_CAPACITY_LEVEL_FULL) ||
			(ret == POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN))
			val->intval = 0;
		else
			val->intval = batt_ttf;
		break;
/* Sycamore code for SYCAMORE-337 by zhoujj21 at 20241119 start */
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		mm8013_get_tte(&val->intval);
		break;
/* Sycamore code for SYCAMORE-337 by zhoujj21 at 20241119 end */
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = BATTERY_QMAX * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = batt_volt;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

/* Sycamore code for SYCAMORE-5256 by kangkai4 at 20241231 start */
static bool get_pdo_adapter(void)
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
/* Sycamore code for SYCAMORE-5256 by kangkai4 at 20241231 end */

static void mm8013_external_power_changed(struct power_supply *psy)
{
	if (chip->is_probe_done == false) {
		pr_info("is_probe_done:%d\n",  chip->is_probe_done);
		return;
	}

	schedule_work(&chip->usb_type_work);
	power_supply_changed(chip->mm8013_psy);
}

static const char * const power_supply_usb_real_type_text[] = {
	"Unknown", "SDP", "DCP", "CDP", "ACA", "C",
	"PD", "PD_DRP", "PD_PPS", "BrickID"
};

static void battery_chg_update_usb_type_work(struct work_struct *work)
{
	int usb_type = 0, charger_online = 0, status = 0;
	static struct adapter_device *pd_adapter = NULL;
	int pd_type = 0;
	if (chip->is_probe_done == false) {
		pr_info("is_probe_done:%d\n",  chip->is_probe_done);
		return;
	}

	//type = mm8013_get_type();
	usb_type = mm8013_get_usb_type();
	charger_online = mm8013_get_charger_online();
	status = mm8013_get_charger_status();

	if (usb_type != POWER_SUPPLY_USB_TYPE_UNKNOWN && charger_online && status != POWER_SUPPLY_STATUS_FULL && batt_soc != 100) {
		if (!chip->charger_wakelock->active) {
			__pm_stay_awake(chip->charger_wakelock);
			pr_info("hold wakelock\n");
		}
	} else if (chip->charger_wakelock->active) {
		__pm_relax(chip->charger_wakelock);
		pr_info("release wakelock\n");
	}
	/* DOCOMOGIN code for DDOCOMOGIN-2199 by kangkai4 at 20250727 start */
	if (charger_online) {
		if (pd_adapter == NULL){
			pd_adapter = get_adapter_by_name("pd_adapter");
		}
		if (pd_adapter != NULL){
			pd_type = adapter_dev_get_property(pd_adapter, PD_TYPE);
		}
	}


	if (pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30 || pd_type == MTK_PD_CONNECT_PE_READY_SNK || pd_type ==MTK_PD_CONNECT_PE_READY_SNK_APDO)
		chip->real_type = POWER_SUPPLY_USB_TYPE_PD;
	else
		chip->real_type = usb_type;

	/* DOCOMOGIN code for DDOCOMOGIN-2199 by kangkai4 at 20250727 end */
}

bool check_if_bypass_and_real_type_fit_vbus(int real_type);
int battery_charge_sysfs_get_property(struct power_supply *psy, enum sysfs_property prop, union power_supply_propval *val)
{
	int soh = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_PRODUCT_DATE:
		mm8013_get_battery_manufacatur_date(&val->intval);
		break;
	case POWER_SUPPLY_PROP_ACTIVATE_DATE:
		val->intval = mm8013_get_battery_start_used_date();
		break;
	case POWER_SUPPLY_PROP_PROTECTION_SETTING:
		val->intval = mm8013_get_protection_setting();
		break;
	case POWER_SUPPLY_PROP_PROTECTION_SETTING_EU:
		val->intval = mm8013_get_protection_setting_eu();
		break;
	case POWER_SUPPLY_PROP_RECHARGE_SETTING:
		val->intval =  mm8013_get_recharge_setting();
		break;
	case POWER_SUPPLY_PROP_BATTERY_PROTECTION:
		val->intval = batt_bp_en;
		break;
	case POWER_SUPPLY_PROP_BATTERY_MAINTENANCE:
		val->intval = batt_maintain_en;
		break;
	case POWER_SUPPLY_PROP_BATTERY_MAINTENANCE_V2:
		val->intval = batt_maintain_v2_en;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = !bypass_chg_en;
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		val->intval = g_input_suspend;
		break;
	case POWER_SUPPLY_PROP_SOH:
		mm8013_get_battery_healthd(&soh);
		val->intval = soh;
		break;
	case POWER_SUPPLY_PROP_BATT_ID:
		val->intval = mm8013_get_bat_id();
		break;
	case POWER_SUPPLY_PROP_BM_CV:
		val->intval = batt_volt;
		break;
	case POWER_SUPPLY_PROP_FAKE_CC:
		val->intval = fake_cc;
		break;
	case POWER_SUPPLY_PROP_FAKE_SOC:
		val->intval = fake_soc;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = chip->real_type;
		/* Sycamore code for SYCAMORE-5256 by kangkai4 at 20241231 start */
		if (get_pdo_adapter())
			val->intval = POWER_SUPPLY_USB_TYPE_PD;
		else if (check_if_bypass_and_real_type_fit_vbus(chip->real_type) == false)
			val->intval = POWER_SUPPLY_USB_TYPE_DCP;
		/* Sycamore code for SYCAMORE-5256 by kangkai4 at 20241231 end */
		break;
	case POWER_SUPPLY_PROP_FAKE_TEMP:
		val->intval = fake_temp;
		break;
	case POWER_SUPPLY_PROP_BYPASS_SETTING:
		val->intval = mm8013_get_bypass_setting();
		break;
/* Sycamore code for SYCAMORE-5747 by zhoujj21 at 20250102 start */
	case POWER_SUPPLY_PROP_BATTERY_USED_STATE:
		val->intval = mm8013c10_get_battery_used_state();
		break;
/* Sycamore code for SYCAMORE-5747 by zhoujj21 at 20250102 end */
	default:
		return -EINVAL;
	}
	return 0;
}
bool check_if_bypass_and_real_type_fit_vbus(int real_type){
	union power_supply_propval vbus;

	power_supply_get_property(chip->chg_psy,POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbus);
	if(bypass_setting.enable){
		if(vbus.intval < 7000 && real_type == POWER_SUPPLY_USB_TYPE_ACA){
			pr_err("[%s]:vbus = %d, too low for real_type:%d\n",__func__,vbus.intval,real_type);
			return false;
		}
	}
	return true;
}

int battery_charge_sysfs_set_property(struct power_supply *psy, enum sysfs_property prop, const union power_supply_propval *val)
{
	pr_info("set %d vaule = 0x%0x\n", prop, val->intval);
	switch (prop) {
	case POWER_SUPPLY_PROP_ACTIVATE_DATE:
		mm8013_set_battery_start_used_date(val->intval);
		break;
/* Sycamore code for SYCAMORE-4334 by zhoujj21 at 20241213 start */
	case POWER_SUPPLY_PROP_PROTECTION_SETTING:
		if (CHG_PROTECTION_SETTING_ENABLE == 1) {
			mm8013_parse_protection_setting(val->intval);
			mm8013_set_protection_setting(val->intval);
			mod_delayed_work(system_wq, &chip->mm8013_monitor_work, 0);
		}
		break;
	case POWER_SUPPLY_PROP_PROTECTION_SETTING_EU:
		mm8013_parse_protection_setting_eu(val->intval);
		mm8013_set_protection_setting_eu(val->intval);
		mod_delayed_work(system_wq, &chip->mm8013_monitor_work, 0);
		break;
	case POWER_SUPPLY_PROP_RECHARGE_SETTING:
		mm8013_set_recharge_setting(val->intval);
		mod_delayed_work(system_wq, &chip->mm8013_monitor_work, 0);
		break;
	case POWER_SUPPLY_PROP_BATTERY_PROTECTION:
		if (CHG_PROTECTION_SETTING_ENABLE == 0) {
			batt_bp_en = val->intval ? true : false;
			mod_delayed_work(system_wq, &chip->mm8013_monitor_work, 0);
		}
		break;
/* Sycamore code for SYCAMORE-5565 by zhoujj21 at 20241231 start */
	case POWER_SUPPLY_PROP_BATTERY_MAINTENANCE:
		batt_maintain_en = val->intval ? true : false;
		mm8013c10_set_battery_maintenance((int)batt_maintain_en);
		break;
	case POWER_SUPPLY_PROP_BATTERY_MAINTENANCE_V2:
		batt_maintain_v2_en = val->intval ? true : false;
		mm8013c10_set_battery_maintenance_v2((int)batt_maintain_v2_en);
		break;
/* Sycamore code for SYCAMORE-5565 by zhoujj21 at 20241231 end */
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		if (CHG_PROTECTION_SETTING_ENABLE == 0) {
			bypass_chg_en = val->intval ? false : true;
			mod_delayed_work(system_wq, &chip->mm8013_monitor_work, 0);
		}
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		if (CHG_PROTECTION_SETTING_ENABLE == 0) {
			g_input_suspend = val->intval ? true : false;
			mod_delayed_work(system_wq, &chip->mm8013_monitor_work, 0);
		}
		break;
/* Sycamore code for SYCAMORE-4334 by zhoujj21 at 20241213 end */
	case POWER_SUPPLY_PROP_BM_CV:
		mm8013_set_volt(val->intval);
		break;
	case POWER_SUPPLY_PROP_FAKE_CC:
		fake_cc = val->intval;
		break;
	case POWER_SUPPLY_PROP_FAKE_SOC:
		fake_soc = val->intval;
		break;
	case POWER_SUPPLY_PROP_FAKE_TEMP:
		fake_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_BYPASS_SETTING:
		bypass_settting = val->intval;
		mm8013_parse_bypass_setting(val->intval);
		mm8013_set_bypass_setting(val->intval);
		mod_delayed_work(system_wq, &chip->mm8013_monitor_work, 0);
		break;
/* Sycamore code for SYCAMORE-5747 by zhoujj21 at 20250102 start */
	case POWER_SUPPLY_PROP_BATTERY_USED_STATE:
		mm8013c10_set_battery_used_state(val->intval);
		break;
/* Sycamore code for SYCAMORE-5747 by zhoujj21 at 20250102 end */
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t battery_charge_sysfs_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy = NULL;
	struct sysfs_info *sysfs_attr;
	union power_supply_propval pval = {0};
	int val;
	int ret;

	ret = kstrtos32(buf, 10, &val);
	if (ret < 0)
		return count;

	pval.intval = val;

	psy = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(psy)) {
		pr_err("%s invalid psy\n", __func__);
		return count;
	}

	sysfs_attr = container_of(attr, struct sysfs_info, attr);
	if (!sysfs_attr) {
		pr_err("%s invalid sysfs_attr\n", __func__);
		return count;
	}

	battery_charge_sysfs_set_property(psy, sysfs_attr->prop, &pval);

	return count;
}

static const char *get_usb_type_name(u32 usb_type)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(power_supply_usb_real_type_text); i++) {
		if (i == usb_type)
			return power_supply_usb_real_type_text[i];
	}

	return "Unknown";
}

static ssize_t battery_charge_sysfs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = NULL;
	struct sysfs_info *sysfs_attr;
	union power_supply_propval pval = {0};
	ssize_t len = 0;

	psy = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(psy)) {
		pr_err("%s invalid psy\n", __func__);
		return 0;
	}

	sysfs_attr = container_of(attr, struct sysfs_info, attr);
	if (!sysfs_attr) {
		pr_err("%s invalid sysfs_attr\n", __func__);
		return 0;
	}

	battery_charge_sysfs_get_property(psy, sysfs_attr->prop, &pval);
	if (sysfs_attr->prop == POWER_SUPPLY_PROP_REAL_TYPE)
		len = scnprintf(buf, PAGE_SIZE, "%s\n", get_usb_type_name(pval.intval));
	else
		len = scnprintf(buf, PAGE_SIZE, "%d\n", pval.intval);

	return len;
}

static struct sysfs_info battery_sysfs_field_tbl[] = {
	SYSFS_FIELD_RO(product_date, POWER_SUPPLY_PROP_PRODUCT_DATE, false),
	SYSFS_FIELD_RW(activate_date, POWER_SUPPLY_PROP_ACTIVATE_DATE, false),
	SYSFS_FIELD_RW(protection_setting, POWER_SUPPLY_PROP_PROTECTION_SETTING, false),
	SYSFS_FIELD_RW(protection_setting_eu, POWER_SUPPLY_PROP_PROTECTION_SETTING_EU, false),
	SYSFS_FIELD_RW(recharge_setting, POWER_SUPPLY_PROP_RECHARGE_SETTING, false),
	SYSFS_FIELD_RW(battery_protection, POWER_SUPPLY_PROP_BATTERY_PROTECTION, false),
	SYSFS_FIELD_RW(battery_maintenance, POWER_SUPPLY_PROP_BATTERY_MAINTENANCE, false),
	SYSFS_FIELD_RW(battery_maintenance_v2, POWER_SUPPLY_PROP_BATTERY_MAINTENANCE_V2, false),
	SYSFS_FIELD_RW(charging_enabled, POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, false),
	SYSFS_FIELD_RW(input_suspend, POWER_SUPPLY_PROP_INPUT_SUSPEND, false),
	SYSFS_FIELD_RW(soh, POWER_SUPPLY_PROP_SOH, false),
	SYSFS_FIELD_RW(batt_id, POWER_SUPPLY_PROP_BATT_ID, false),
	SYSFS_FIELD_RW(batt_cv, POWER_SUPPLY_PROP_BM_CV, false),
	SYSFS_FIELD_RW(fake_cc, POWER_SUPPLY_PROP_FAKE_CC, false),
	SYSFS_FIELD_RW(fake_soc, POWER_SUPPLY_PROP_FAKE_SOC, false),
	SYSFS_FIELD_RW(fake_temp, POWER_SUPPLY_PROP_FAKE_TEMP, false),
	SYSFS_FIELD_RO(real_type, POWER_SUPPLY_PROP_REAL_TYPE, false),
	SYSFS_FIELD_RW(bypass_setting, POWER_SUPPLY_PROP_BYPASS_SETTING, false),
/* Sycamore code for SYCAMORE-5747 by zhoujj21 at 20250102 start */
	SYSFS_FIELD_RW(battery_used, POWER_SUPPLY_PROP_BATTERY_USED_STATE, false),
/* Sycamore code for SYCAMORE-5747 by zhoujj21 at 20250102 end */
};

static struct attribute
	*battery_sysfs_attrs[ARRAY_SIZE(battery_sysfs_field_tbl) + 1];

static const struct attribute_group battery_sysfs_attr_group = {
	.attrs = battery_sysfs_attrs,
};

static int battery_create_sysfs(void)
{
	int i = 0, length = 0, rc = 0;

	length = ARRAY_SIZE(battery_sysfs_field_tbl);
	for (i = 0; i < length; i++)
		battery_sysfs_attrs[i] = &battery_sysfs_field_tbl[i].attr.attr;
	battery_sysfs_attrs[length] = NULL;
	rc = sysfs_create_group(&chip->mm8013_psy->dev.kobj, &battery_sysfs_attr_group);
	if (rc) {
		pr_err("Failed to create battery_sysfs\n");
		return rc;
	}

	return rc;
}

static int mm8013_get_usb_type(void)
{
	int ret = 0;
	union power_supply_propval prop = {0};
	static struct power_supply *psy = NULL;
	if (IS_ERR(psy) || psy == NULL) {
		psy = power_supply_get_by_name("charger");
	if (IS_ERR(psy) || psy == NULL) {
			pr_debug("not ready!\n");
			return -ENODEV;
		}
	}

	ret = power_supply_get_property(psy,
		POWER_SUPPLY_PROP_USB_TYPE, &prop);
	if (ret || prop.intval < 0)
		return -EINVAL;

	return prop.intval;
}

__maybe_unused static int mm8013_get_type(void)
{
	int ret = 0;
	union power_supply_propval prop = {0};
	static struct power_supply *psy = NULL;
	if (IS_ERR(psy) || psy == NULL) {
		psy = power_supply_get_by_name("charger");
	if (IS_ERR(psy) || psy == NULL) {
			pr_debug("not ready!\n");
			return -ENODEV;
		}
	}

	ret = power_supply_get_property(psy,
		POWER_SUPPLY_PROP_TYPE, &prop);
	if (ret || prop.intval < 0)
		return -EINVAL;

	return prop.intval;
}

static int mm8013_get_charger_online(void)
{
	int ret = 0;
	union power_supply_propval prop = {0};
	static struct power_supply *psy = NULL;
	if (IS_ERR(psy) || psy == NULL) {
		psy = power_supply_get_by_name("charger");
	if (IS_ERR(psy) || psy == NULL) {
			pr_debug("not ready!\n");
			return -ENODEV;
		}
	}

	ret = power_supply_get_property(psy,
		POWER_SUPPLY_PROP_ONLINE, &prop);
	if (ret || prop.intval < 0)
		return -EINVAL;

	return prop.intval;
}

static int mm8013_get_charger_status(void)
{
	int ret = 0;
	union power_supply_propval prop = {0};
	static struct power_supply *psy = NULL;
	if (IS_ERR(psy) || psy == NULL) {
		psy = power_supply_get_by_name("charger");
	if (IS_ERR(psy) || psy == NULL) {
			pr_debug("not ready!\n");
			return -ENODEV;
		}
	}

	ret = power_supply_get_property(psy,
		POWER_SUPPLY_PROP_STATUS, &prop);
	if (ret < 0 || prop.intval < 0)
		return -EINVAL;

	return prop.intval;
}

static int mm8013_set_charging_enable(bool en)
{
	int ret = 0;
	union power_supply_propval val;
	 static struct power_supply *psy = NULL;
	if (IS_ERR(psy) || psy == NULL) {
		psy = power_supply_get_by_name("mtk-master-charger");
		if (IS_ERR(psy) || psy == NULL) {
			pr_info("not ready!\n");
			return -ENODEV;
		}
	}

	val.intval = en ? 1 : 0;
	ret = power_supply_set_property(psy,
		POWER_SUPPLY_PROP_CURRENT_NOW, &val);

	return ret;
}

static int mm8013_set_input_suspend(bool en)
{
	int ret = 0;
	union power_supply_propval val;
	static struct power_supply *psy = NULL;
	if (IS_ERR(psy) || psy == NULL) {
		psy = power_supply_get_by_name("mtk-master-charger");
		if (IS_ERR(psy) || psy == NULL) {
			pr_info("not ready!\n");
			return -ENODEV;
		}
	}

	val.intval = en ? 1 : 0;
	ret = power_supply_set_property(psy,
		POWER_SUPPLY_PROP_POWER_NOW, &val);

	return ret;
}

/* Sycamore code for SYCAMORE-21802 by kangkai4 at 20250419 start */
static int mm8013_set_charger_eoc(bool en)
{
	int ret = 0;
	union power_supply_propval val;
	static struct power_supply *psy = NULL;
	if (IS_ERR(psy) || psy == NULL) {
		psy = power_supply_get_by_name("mtk-master-charger");
		if (IS_ERR(psy) || psy == NULL) {
			pr_info("not ready!\n");
			return -ENODEV;
		}
	}

	val.intval = en ? 1 : 0;
	ret = power_supply_set_property(psy,
		POWER_SUPPLY_PROP_ENERGY_AVG, &val);

	return ret;
}
/* Sycamore code for SYCAMORE-21802 by kangkai4 at 20250419 end */

static int mm8013_get_input_suspend(bool *en)
{
	int ret = 0;
	union power_supply_propval prop;
	static struct power_supply *psy = NULL;
	if (IS_ERR(psy) || psy == NULL) {
		psy = power_supply_get_by_name("mtk-master-charger");
	if (IS_ERR(psy) || psy == NULL) {
			pr_info("not ready!\n");
			return -ENODEV;
		}
	}

	ret = power_supply_get_property(psy,
		POWER_SUPPLY_PROP_POWER_NOW, &prop);
	if (ret || prop.intval < 0)
		return -EINVAL;
	*en = prop.intval;

	return ret;
}

static int mm8013_get_charging_enabled(bool *en)
{
	int ret = 0;
	union power_supply_propval prop;
	static struct power_supply *psy = NULL;
	if (IS_ERR(psy) || psy == NULL) {
		psy = power_supply_get_by_name("mtk-master-charger");
		if (IS_ERR(psy) || psy == NULL) {
			pr_info("not ready!\n");
			return -ENODEV;
		}
	}

	ret = power_supply_get_property(psy,
		POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
	if (ret || prop.intval < 0)
		return -EINVAL;
	*en = prop.intval;

	return ret;
}

static int mm8013_set_constant_voltage(int cv)
{
	int ret = 0;
	union power_supply_propval val;
	static struct power_supply *psy = NULL;
	if (IS_ERR(psy) || psy == NULL) {
		psy = power_supply_get_by_name("mtk-master-charger");
		if (IS_ERR(psy) || psy == NULL) {
			pr_info("not ready!\n");
			return -ENODEV;
		}
	}

	val.intval = cv;
	ret = power_supply_set_property(psy,
		POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &val);

	return ret;
}

static struct bm_aging *get_bm_aging_table(int index)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(aging_table); i++) {
		if (i == index)
			return &aging_table[i];
	}
	return &aging_table[0];
}

/* Sycamore code for SYCAMORE-5966 by zhoujj21 at 20250102 start */
static int lenovo_charging_protection_check(void)
{
	static u16  s_bp_index = BP_LEVEL_0;
	static int s_soc = 0;
	u16 low_capacity = 0;
	u16 high_capacity = 100;
	u16 bp_index = BP_LEVEL_0;
	bool input_suspend = 0;
	int ret = 0;
	/* Sycamore code for SYCAMORE-4693 by kangkai4 at 20241216 start */
	int status =0;
	/* Sycamore code for SYCAMORE-4693 by kangkai4 at 20241216 end */
	static bool s_chg_to_full = false;
	/* Sycamore code for SYCAMORE-15912 by kangkai4 at 20250314 start */
	static int pre_real_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	/* Sycamore code for SYCAMORE-15912 by kangkai4 at 20250314 end */
/* Sycamore code for OJUNIPER-2279  by yuxb1 at 2025 0318 start */
	//if (chip->real_type != POWER_SUPPLY_USB_TYPE_UNKNOWN)
/* Sycamore code for OJUNIPER-2279  by yuxb1 at 2025 0318 end */
	{
		if (g_input_suspend)
		{
			bp_index = BP_LEVEL_MAX;
		}
		else if (bypass_setting.enable)
		{
			bp_index = BP_LEVEL_4;
			low_capacity = bypass_setting.low_capacity;
			high_capacity = bypass_setting.high_capacity;
		}
		else if (bypass_chg_en)
		{
			bp_index = BP_LEVEL_3;
		}
		else if (bp_setting.enable)
		{
			bp_index = BP_LEVEL_2;
			low_capacity = bp_setting.low_capacity;
			high_capacity = bp_setting.high_capacity;
		}
		else if (bp_setting_eu.enable)
		{
			bp_index = BP_LEVEL_1;
			if (rechg_precent > 0)
			{
				low_capacity = (bp_setting_eu.high_capacity * rechg_precent / 100);
			}
			else
			{
				low_capacity = bp_setting_eu.low_capacity;
			}
			high_capacity = bp_setting_eu.high_capacity;

			/* when cycle count is divided by 20, allow charging to full */
			/*if ((batt_cycle_count % 20) == 0 &&
				bp_setting_eu.high_capacity == 100) {
				chg_to_full = true;
			} else if ((batt_cycle_count % 20) != 0 &&
				bp_setting_eu.high_capacity != 100) {
				chg_to_full = false;
			}*/
		} else {
			/* Sycamore code for SYCAMORE-21802 by kangkai4 at 20250419 start */
			status = mm8013_get_charger_status();
			if (status == POWER_SUPPLY_STATUS_FULL && batt_soc <= RECHARGE_SOC && (get_battery_health() < POWER_SUPPLY_HEALTH_WARM))
			{
				mm8013_set_charger_eoc(true);
				mm8013_set_charging_enable(false);
				mm8013_set_charging_enable(true);
				pr_info("%s,recharged!!!", __func__);
			}
			/* Sycamore code for SYCAMORE-21802 by kangkai4 at 20250419 end */
		}
	}

	pr_info("[LENOVO_CHG_BP] : bp_index = [%d, %d], bp_soc = [%d, %d] bp_thr = [%d, %d], rechg_pct = %d, bypass_chg_en = %d, ctf = [%d, %d]",
			bp_index, s_bp_index, batt_soc, s_soc, low_capacity, high_capacity, rechg_precent, bypass_chg_en, chg_to_full, s_chg_to_full);

	/* Sycamore code for SYCAMORE-15912 by kangkai4 at 20250314 start */
	if (s_bp_index == bp_index && s_soc == batt_soc && chg_to_full == s_chg_to_full && pre_real_type == chip->real_type)
		return 0;
	/* Sycamore code for SYCAMORE-15912 by kangkai4 at 20250314 end */

	switch (bp_index)
	{
		case BP_LEVEL_0:
			 /* restore state when close bp during charging */
			if (s_bp_index != BP_LEVEL_0)
			{
				mm8013_set_input_suspend(false);
				mm8013_set_charging_enable(true);
			}
			break;
		case BP_LEVEL_1://eu
			if (batt_soc >= high_capacity && chg_to_full == false)
			{
				mm8013_set_charging_enable(false);
			}
			else if (batt_soc <= low_capacity || chg_to_full == true)
			{
				mm8013_set_charging_enable(true);
				mm8013_get_input_suspend(&input_suspend);
				if (input_suspend != 0)
				{
					mm8013_set_input_suspend(false);
				}
			}
			break;
		case BP_LEVEL_2://non-eu
			if (batt_soc > high_capacity)
			{
				mm8013_set_charging_enable(false);
				mm8013_set_input_suspend(true);
			}
			else if (batt_soc == high_capacity)
			{
				mm8013_set_charging_enable(false);
				mm8013_get_input_suspend(&input_suspend);
				if (input_suspend != 0)
				{
					mm8013_set_input_suspend(false);
				}
			}
			else if (batt_soc <= low_capacity)
			{
				mm8013_set_charging_enable(true);
				mm8013_get_input_suspend(&input_suspend);
				if (input_suspend != 0)
				{
					mm8013_set_input_suspend(false);
				}
			}
			break;
		case BP_LEVEL_3://bypass charging
			mm8013_set_charging_enable(false);
			break;
		case BP_LEVEL_4://bypass_setting
			if (batt_soc > high_capacity)
			{
				mm8013_set_charging_enable(false);
				mm8013_set_input_suspend(true);
			}
			else if (batt_soc == high_capacity)
			{
				mm8013_set_charging_enable(false);
				mm8013_get_input_suspend(&input_suspend);
				if (input_suspend != 0)
				{
					mm8013_set_input_suspend(false);
				}
			}
			else if (batt_soc <= low_capacity)
			{
				mm8013_set_charging_enable(true);
				mm8013_get_input_suspend(&input_suspend);
				if (input_suspend != 0)
				{
					mm8013_set_input_suspend(false);
				}
			}
			break;
		case BP_LEVEL_MAX://input suspend
			mm8013_set_input_suspend(true);
			break;
		default:
			break;
	}
	/* DOCOMOGIN code for DOCOMOGIN-2076 by kangkai4 at 20250717 start */
	if(chip->real_type == POWER_SUPPLY_USB_TYPE_UNKNOWN) {
		mm8013_set_charging_enable(true);
		mm8013_set_input_suspend(false);
	}
	/* DOCOMOGIN code for DOCOMOGIN-2076 by kangkai4 at 20250717 end */

	s_bp_index = bp_index;
	s_soc = batt_soc;
	s_chg_to_full = chg_to_full;
	/* Sycamore code for SYCAMORE-15912 by kangkai4 at 20250314 start */
	pre_real_type = chip->real_type;
	/* Sycamore code for SYCAMORE-15912 by kangkai4 at 20250314 end */

	return ret;
}
/* Sycamore code for SYCAMORE-5966 by zhoujj21 at 20250102 end */

static int lenovo_batt_maintenance_check(void)
{
	static int bat_bm_index = -EINVAL;
	int bm_index = BM_LEVEL_0;
	int ret = 0;
	int bm_cv = 4520;
	int chg_cv = 4520;

	if (bypass_setting.enable) {
		bm_index = BM_LEVEL_0; //close battery maintenance
	}
	else if (batt_maintain_v2_en)
	{
		bm_index = BM_LEVEL_5;
	}
	else if (batt_maintain_en)
	{
		if (batt_cycle_count >= 1050)
			bm_index = BM_LEVEL_4;
		else if (1020 <= batt_cycle_count && batt_cycle_count < 1050)
			bm_index = BM_LEVEL_3;
		else if (150 <= batt_cycle_count && batt_cycle_count < 1020)
			bm_index = BM_LEVEL_2;
		else
			bm_index = BM_LEVEL_1;
	}
	else
	{
		bm_index = BM_LEVEL_0;
	}
/* Sycamore code for SYCAMORE-4334 by zhoujj21 at 20241213 start */
	if (bat_bm_index == bm_index)
		return ret;

	if ((batt_maintain_en && batt_soc <= BM_CAPCAITY_THR) ||
		(batt_maintain_v2_en && batt_soc <= BM2_CAPCAITY_THR) ||
		(!batt_maintain_en && !batt_maintain_v2_en))
	{
		bm_cv = get_bm_aging_table(bm_index)->bm_cv;
		ret = mm8013_set_volt(bm_cv);
		if (ret != 0)
			pr_err("failed to write battery voltage.");

		chg_cv = get_bm_aging_table(bm_index)->chg_cv;
		ret = mm8013_set_constant_voltage(chg_cv * 1000);
		if (ret != 0)
			pr_err("failed to write constant voltage.");
		bat_bm_index = bm_index;
	}

	pr_info("[LENOVO_CHG_BM] : bm_index = [%d, %d], bm = [%d, %d], soc = %d, cv = [%d, %d], cc = %d",
			bm_index, bat_bm_index, batt_maintain_en, batt_maintain_v2_en, batt_soc, bm_cv, chg_cv, batt_cycle_count);
/* Sycamore code for SYCAMORE-4334 by zhoujj21 at 20241213 end */
	return ret;
}

/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20241231 start */
static int  lenovo_batt_maintenance_init_fv(void)
 {
	int ret = 0;
	int temp_cv = 0;
	int level = BM_LEVEL_0;
	int chg_cv = 4520;
	int bm_cv = batt_volt;

	for (level = BM_LEVEL_0; level <= BM_LEVEL_5; level++) {
		temp_cv = get_bm_aging_table(level)->bm_cv;
		if (temp_cv == bm_cv)
			chg_cv = get_bm_aging_table(level)->chg_cv;
	}

	if (batt_maintain_en || batt_maintain_v2_en) {
		ret = mm8013_set_constant_voltage(chg_cv * 1000);
		if (ret != 0)
			pr_err("failed to write constant voltage.");
	}

	pr_info("[LENOVO_CHG_BM] : bm = %d, bm_v2 = %d, chg_cv = %d, bm_cv = %d",
		batt_maintain_en, batt_maintain_v2_en, chg_cv, bm_cv);

    return ret;
}

static int mm8013_get_pmic_tbatt_vadc(u32 *tbat_vadc)
{
	static struct iio_channel *chan_tbatt_vadc = NULL;
	int ret = 0;
	int val = 0;
	//*vbus = 0;

	if (IS_ERR(chan_tbatt_vadc) || chan_tbatt_vadc == NULL) {
		chan_tbatt_vadc = devm_iio_channel_get(
			&chip->client->dev, "pmic_battery_temp");
		if (IS_ERR(chan_tbatt_vadc) || chan_tbatt_vadc == NULL) {
			pr_info("not ready!\n");
			return -ENODEV;
		}
	}

	if (!IS_ERR(chan_tbatt_vadc) && chan_tbatt_vadc != NULL) {
		ret = iio_read_channel_processed(chan_tbatt_vadc, &val);
		if (ret < 0)
			pr_info("read fail,ret=%d\n",  ret);
	} else {
		pr_info("chan error %p\n", chan_tbatt_vadc);
		ret = -EOPNOTSUPP;
	}

	*tbat_vadc = val;

	return  ret;
}

/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20250205 start */
static int mm8013_set_input_current_limit(int curr)
{
	int ret = 0;
	union power_supply_propval val;

	if (IS_ERR(chg_psy) || chg_psy == NULL) {
		chg_psy = power_supply_get_by_name("charger");
		if (IS_ERR(chg_psy) || chg_psy == NULL) {
			pr_info("not ready!\n");
			return -ENODEV;
		}
	}

	val.intval = curr;
	ret = power_supply_set_property(chg_psy,
		POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);

	return ret;
}

static int mm8013_set_input_voltage_limit(int cv)
{
	int ret = 0;
	union power_supply_propval val;

	if (IS_ERR(chg_psy) || chg_psy == NULL) {
		chg_psy = power_supply_get_by_name("charger");
		if (IS_ERR(chg_psy) || chg_psy == NULL) {
			pr_info("not ready!\n");
			return -ENODEV;
		}
	}

	val.intval = cv;
	ret = power_supply_set_property(chg_psy,
		POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT, &val);

	return ret;
}

static int mm8013_get_charger_supplier_num(void)
{
	int ret = 0;
	union power_supply_propval val = {.intval = 0,  .strval = "0"};

	if (IS_ERR(chg_psy) || chg_psy == NULL) {
		chg_psy = power_supply_get_by_name("charger");
		if (IS_ERR(chg_psy) || chg_psy == NULL) {
			pr_info("not ready!\n");
			return -ENODEV;
		}
	}

	ret = power_supply_get_property(chg_psy,
		POWER_SUPPLY_PROP_MODEL_NAME, &val);

	if (strcmp(val.strval, "sgm41513D") == 0) {
		pr_info("%s, charger name=%s\n", __func__, val.strval);
		return 1;
	}

	return 0;
}

#define BATT_TEMP_VADC_THR 2000 // mV
static int lenovo_batt_shutdown_check(struct mm8013_chip *chip)
{
	static int shutdown_check_cnt = 0;
	u32 tbat_vadc;

	if (chip->bootmode != 0 && chip->bootmode != 8) //non  normal and offmode return
		return 0;

	if ( chip->real_type == POWER_SUPPLY_USB_TYPE_UNKNOWN)
		return 0;

	mm8013_get_pmic_tbatt_vadc(&tbat_vadc);
	if (tbat_vadc > BATT_TEMP_VADC_THR) {
		shutdown_check_cnt++;
	} else {
		shutdown_check_cnt = 0;
		return 0;
	}

	if (shutdown_check_cnt >= 2) {
		shutdown_check_cnt = 0;
		pr_info("battery missing, enter shutdown!!!\n");
		msleep(10);
		if (mm8013_get_charger_supplier_num() == 1) {
			pr_info("sgm41513d, forced power outage!!!\n");
			mm8013_set_input_voltage_limit(4200000);
			msleep(50);
			mm8013_set_input_current_limit(200000);
		} else {
			pr_info("sc8960x, kernel power off!!!\n");
			kernel_power_off();
		}
	}

	pr_info("tbat_vadc = %d, TYPE = %d, CNT = %d, bootmode = %d\n",
				tbat_vadc, chip->real_type, shutdown_check_cnt, chip->bootmode);
	return 0;
}
/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20250205 end */

/* Sycamore code for SYCAMORE-8919 by kangkai4 at 20250311 start */
#define VBUS_VACD_THER 7000 // mV
void lenovo_handle_bypass_setting(struct mm8013_chip *chip)
{
	union power_supply_propval   vbus;

	if (IS_ERR_OR_NULL(chip->chg_psy)) {
		chip->chg_psy =
			power_supply_get_by_name("charger");
		pr_info("[%s]: get charger phandle fail\n", __func__);
	} else {
		power_supply_get_property(chip->chg_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbus);

		if (vbus.intval < VBUS_VACD_THER && chip->real_type  == POWER_SUPPLY_USB_TYPE_ACA)  {
			power_supply_changed(chip->chg_psy);
			pr_info("[%s] power_supply_changed\n", __func__);
		}
	}
}
/* Sycamore code for SYCAMORE-8919 by kangkai4 at 20250311 end */

static void monitor_handler(struct work_struct *work)
{
	struct mm8013_chip *chip = container_of(work,
										struct mm8013_chip,
										mm8013_monitor_work.work);
	static int soc = 0;
	static int tbat_stage = 0;
	static bool bm_init =false;

	mm8013_get_soc(&batt_soc);
	mm8013_get_voltage(&batt_vbat);
	mm8013_get_current(&batt_ibat);
	mm8013_get_temperature(&batt_temp);
	mm8013_get_ttf(&batt_ttf);
	mm8013_get_cycle_count(&batt_cycle_count);
	mm8013_get_volt(&batt_volt);
	lenovo_charging_protection_check();
	lenovo_batt_shutdown_check(chip);

	/* battery charging maintenance check */
        if (bm_init == false) {
		lenovo_batt_maintenance_init_fv();
		bm_init = true;
        } else {
		lenovo_batt_maintenance_check();
	}

	if (soc != batt_soc || tbat_stage != batt_temp / 5) {
		soc = batt_soc;
		tbat_stage = batt_temp / 5;
		power_supply_changed(chip->mm8013_psy);
	}

	/* Sycamore code for SYCAMORE-8919 by kangkai4 at 20250311 start */
	if (bypass_setting.enable) {
		lenovo_handle_bypass_setting(chip);
	}
	/* Sycamore code for SYCAMORE-8919 by kangkai4 at 20250311 end */

	pr_info("SOC = %d, VBAT = %d, TBAT = %d, IBAT = %d, CYCLE = %d, BM_CV = %d, TTF = %d, real_type = %d",
					batt_soc, batt_vbat, batt_temp, batt_ibat, batt_cycle_count, batt_volt, batt_ttf, chip->real_type);

	if (chip->real_type == POWER_SUPPLY_USB_TYPE_UNKNOWN) {
		batt_ttf = 0;
		if (abs(batt_ibat) > 2000)
			schedule_delayed_work(&chip->mm8013_monitor_work, msecs_to_jiffies(2000));
		else
			schedule_delayed_work(&chip->mm8013_monitor_work, msecs_to_jiffies(5000));
	} else {
		schedule_delayed_work(&chip->mm8013_monitor_work, msecs_to_jiffies(2000));
	}
}
/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20241231 end */

#if IS_ENABLED(CONFIG_HQ_SYSFS_SUPPORT)
static void register_hwinfo_work(struct work_struct *work)
{
	int ret = mm8013_get_bat_id();

	if(hq_regiser_hw_info(HWID_BATTERY, battery_manufacturer[ret]) == 0) {
	pr_info("mm8013hq_regiser_hw_info success \n");
	} else {
		pr_info("mm8013hq_regiser_hw_info failed retry\n");
		schedule_delayed_work(&chip->register_hwinfo_work,
							msecs_to_jiffies(1000));
	}
}
#endif

void mm8013_init_setting(void)
{
	u32 protection_setting = 0;

	protection_setting = mm8013_get_protection_setting();
	mm8013_parse_protection_setting(protection_setting);
	g_protection_setting_eu = mm8013_get_protection_setting_eu();
	mm8013_parse_protection_setting_eu(g_protection_setting_eu);
	g_bypass_setting = mm8013_get_bypass_setting();
	mm8013_parse_bypass_setting(g_bypass_setting);
	batt_maintain_en = mm8013c10_get_battery_maintenance();
	batt_maintain_v2_en = mm8013c10_get_battery_maintenance_v2();
	rechg_precent = mm8013c10_get_battery_recharging_setting();
	mm8013_get_full_capacity(&charge_full);

	pr_info("[LENOVO_CHG_BP] : bp = [%d %d %d],  bp_eu = [%d %d %d], bm = %d, bm_v2 = %d, rechg_pct = %d",
			bp_setting.enable, bp_setting.low_capacity, bp_setting.high_capacity,
			bp_setting_eu.enable, bp_setting_eu.low_capacity, bp_setting_eu.high_capacity,
			batt_maintain_en, batt_maintain_v2_en, rechg_precent);
}

u32 get_protection_setting_eu (void)
{
	return g_protection_setting_eu;
}
EXPORT_SYMBOL(get_protection_setting_eu);

u32 get_bypass_setting(void){
	return g_bypass_setting;
}
EXPORT_SYMBOL(get_bypass_setting);

/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20241231 start */
static int mm8013_parse_dts(struct mm8013_chip *chip)
{
	struct device_node *boot_node = NULL;
	struct tag_bootmode *tag = NULL;
	chip->bootmode = 0;

	boot_node = of_find_node_by_path("/chosen");
	if (!boot_node)
		pr_err("%s: failed to get boot mode phandle\n", __func__);
	else {
		tag = (struct tag_bootmode *)of_get_property(boot_node,
							"atag,boot", NULL);
		if (!tag)
			pr_err("%s: failed to get atag,boot\n", __func__);
		else {
			pr_info("size:0x%x tag:0x%x bootmode:0x%x boottype:0x%x\n",
				tag->size, tag->tag, tag->bootmode, tag->boottype);
			chip->bootmode = tag->bootmode;
		}
	}
	return 0;
}
/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20241231 end */

static int mm8013_probe(struct i2c_client *client,  const struct i2c_device_id * id)
{
	struct device *cdev = &client->dev;
	int ret;
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg = {0};

	mm8013_init = false;
	pr_info("start!!!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		pr_info("failed: %s smbus data not supported!\n", __func__);
		return -EIO;
	}

	chip = devm_kzalloc(cdev, sizeof(struct mm8013_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->dev = &client->dev;

	mutex_init(&chip->i2c_rw_lock);
	mutex_init(&chip->suspend_lock);
	i2c_set_clientdata(client, chip);

	ret = mm8013_checkdevice();
	if (ret < 0) {
		pr_info("failed to access\n");
	} else {
		mm8013_init = true;
	}

	chip->real_type = 0;
	chip->suspended = false;
	chip->charger_wakelock = wakeup_source_register(NULL, "fgmm8013_wakelock");
	INIT_WORK(&chip->usb_type_work,
					battery_chg_update_usb_type_work);
	INIT_DELAYED_WORK(&chip->mm8013_monitor_work,
						monitor_handler);

#if IS_ENABLED(CONFIG_HQ_SYSFS_SUPPORT)
	INIT_DELAYED_WORK(&chip->register_hwinfo_work,
					register_hwinfo_work);
#endif

	psy_desc = devm_kzalloc(&client->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc)
		return -ENOMEM;

	psy_cfg.drv_data = chip;
	psy_desc->name = "battery";
	psy_desc->type = POWER_SUPPLY_TYPE_BATTERY;
	psy_desc->properties = mm8013_battery_props;
	psy_desc->num_properties = ARRAY_SIZE(mm8013_battery_props);
	psy_desc->get_property = mm8013_get_property;
	psy_desc->external_power_changed = mm8013_external_power_changed;
	psy_desc->set_property = NULL;

	chip->mm8013_psy = power_supply_register(&client->dev, psy_desc, &psy_cfg);
	if (IS_ERR(chip->mm8013_psy)) {
		ret = PTR_ERR(chip->mm8013_psy);
		pr_info("failed to register battery: %d\n", ret);
		return ret;
	}

/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20241231 start */
	mm8013_parse_dts(chip);
/* Sycamore code for SYCAMORE-5024 by zhoujj21 at 20241231 end */
	mm8013_init_setting();
	battery_create_sysfs();

	schedule_delayed_work(&chip->mm8013_monitor_work,
						msecs_to_jiffies(0));

#if IS_ENABLED(CONFIG_HQ_SYSFS_SUPPORT)
	schedule_delayed_work(&chip->register_hwinfo_work,
						msecs_to_jiffies(1000*10));
#endif

	pr_info("%s success!\n", __func__);
	chip->is_probe_done = true;

	return 0;
}

static	struct i2c_device_id mm8013_id_table[] = {
	{ "mm8013", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mm8013_id_table);

static const struct of_device_id mm8013_match_table[] = {
	{ .compatible = "mediatek,mm8013",},
	{},
};
static int mm8013_suspend(struct device *dev)
{
	struct mm8013_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->suspend_lock);
	chip->suspended = true;
	mutex_unlock(&chip->suspend_lock);

	return 0;
}

static int mm8013_resume(struct device *dev)
{
	struct mm8013_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->suspend_lock);
	chip->suspended = false;
	mutex_unlock(&chip->suspend_lock);

	return 0;
}

static SIMPLE_DEV_PM_OPS(mm8013_pm_ops, mm8013_suspend, mm8013_resume);
static struct i2c_driver mm8013_i2c_driver = {
	.driver    = {
		.name  = "mm8013",
		.owner = THIS_MODULE,
		.of_match_table = mm8013_match_table,
		.pm = &mm8013_pm_ops,
	},
	.probe	   = mm8013_probe,
	.id_table  = mm8013_id_table,
};

static int __init mm8013_i2c_init(void)
{
	return i2c_add_driver(&mm8013_i2c_driver);
}
static void __exit mm8013_i2c_exit(void)
{
	i2c_del_driver(&mm8013_i2c_driver);
}

module_init(mm8013_i2c_init);
module_exit(mm8013_i2c_exit);
MODULE_DESCRIPTION("I2c bus driver for mm8013x gauge");
MODULE_LICENSE("GPL v2");
