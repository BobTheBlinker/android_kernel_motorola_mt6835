/*
 * Copyright (C) 2015 MediaTek Inc.
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

#include <linux/hqsysfs.h>
//#include "hqsys_misc.h"
//#include "hqsys_pcba.h"
#include <linux/of.h>
#include <linux/types.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/gpio.h>
#ifdef CONFIG_HQ_SYSFS_SUPPORT
#include <linux/hqsysfs.h>
#endif
#define HQ_SYS_FS_VER "2022-1230 V2.0"
#define emmc_file "/sys/block/mmcblk0/size"
#define emmc_len  17
#define EMMC_VENDOR_CMP_SIZE  2

/* sycamore code for SYCAMORE-10161 by kexinzhang2 at 2025/1/23 start */
#define SKU_CMDLINE	"androidboot.product.hardware.skuinfo="
/* sycamore code for SYCAMORE-10161 by kexinzhang2 at 2025/1/23 end */

#define STG_CMDLINE	"androidboot.stage="


static HW_INFO(HWID_VER, ver);
static HW_INFO(HWID_SUMMARY, hw_summary);
static HW_INFO(HWID_DDR, ram);
static HW_INFO(HWID_EMMC, emmc);
static HW_INFO(HWID_LCM, lcm);
static HW_INFO(HWID_BIAS_IC,lcm_bias_ic);
static HW_INFO(HWID_CTP, ctp);
static HW_INFO(HWID_SUB_CAM, sub_cam);	  //sub
static HW_INFO(HWID_SUB_CAM_2, sub2_cam);  //main0
static HW_INFO(HWID_MAIN_CAM, main0_cam);  //main1
static HW_INFO(HWID_MAIN_CAM_2, main2_cam);  //main2
static HW_INFO(HWID_MAIN_CAM_3, main3_cam);  //main3

static HW_INFO(HWID_MAIN_LENS, main_cam_len);
static HW_INFO(HWID_FLASHLIGHT, flashlight);
static HW_INFO(HWID_GSENSOR, gsensor);
static HW_INFO(HWID_ALSPS, alsps);
static HW_INFO(HWID_MSENSOR, msensor);
static HW_INFO(HWID_GYRO, gyro);
static HW_INFO(HWID_BAROMETER, barometer);
static HW_INFO(HWID_SAR, sar);
static HW_INFO(HWID_IRDA, irda);
static HW_INFO(HWID_FUEL_GAUGE_IC, fuel_gauge_ic);
static HW_INFO(HWID_NFC, nfc);
static HW_INFO(HWID_FP, fingerprint);
//static HW_INFO(HWID_TEE,tee);
static HW_INFO(HWID_PCBA, pcba_config);
static HW_INFO(HWID_PMIC_VERSION, pmic_version);
static HW_INFO(HWID_AUDIO, audio_PA);
static HW_INFO(HWID_PCBA_INFO, pcba_info);
static HW_INFO(HWID_SMARTPA, smartpa);
static HW_INFO(HWID_AUDIO_CODEC, audio_codec);
static HW_INFO(HWID_SIMSTATE, sim_state);
static HW_INFO(HWID_PRIMARY_CHARGER, primary_charger);
static HW_INFO(HWID_SECONDARY_CHARGER, secondary_charger);
static HW_INFO(HWID_PD_IC, pd_ic);
static HW_INFO(HWID_BATTERY, battery_manufacturer);
/* sycamore code for SYCAMORE-3317 by zhoujj21 at 2024/11/19 start */
static HW_INFO(HWID_LEDSTATE, led_state);
/* sycamore code for SYCAMORE-3317 by zhoujj21 at 2024/11/19 end */

static struct attribute *huaqin_attrs[] = {
	&hw_info_ver.attr,
	&hw_info_hw_summary.attr,
	&hw_info_ram.attr,
	&hw_info_emmc.attr,
	&hw_info_lcm.attr,
	&hw_info_lcm_bias_ic.attr,
	&hw_info_ctp.attr,
	&hw_info_sub_cam.attr,
	&hw_info_main0_cam.attr,
	&hw_info_sub2_cam.attr,
	&hw_info_main2_cam.attr,
	&hw_info_main3_cam.attr,

	&hw_info_main_cam_len.attr,
	&hw_info_flashlight.attr,
	&hw_info_gsensor.attr,
	&hw_info_alsps.attr,
	&hw_info_msensor.attr,
	&hw_info_gyro.attr,
	&hw_info_barometer.attr,
	&hw_info_sar.attr,
	&hw_info_irda.attr,
	&hw_info_fuel_gauge_ic.attr,
	&hw_info_nfc.attr,
	&hw_info_fingerprint.attr,
	&hw_info_pcba_config.attr,
	&hw_info_pmic_version.attr,
	&hw_info_audio_PA.attr,
	&hw_info_pcba_info.attr,
	&hw_info_smartpa.attr,
	&hw_info_audio_codec.attr,
	&hw_info_sim_state.attr,
	&hw_info_primary_charger.attr,
	&hw_info_secondary_charger.attr,
	&hw_info_pd_ic.attr,
	&hw_info_battery_manufacturer.attr,
/* sycamore code for SYCAMORE-3317 by zhoujj21 at 2024/11/19 start */
	&hw_info_led_state.attr,
/* sycamore code for SYCAMORE-3317 by zhoujj21 at 2024/11/19 end */
	NULL
};
ATTRIBUTE_GROUPS(huaqin);

#if 0
extern struct mmc_card *memory_info_card;
static unsigned int get_emmc_information(void)
{
	u64 raw_device_capacity = 0;

	if (!memory_info_card) {
		pr_err("memory_info_card is NULL %d\n", raw_device_capacity);
		return raw_device_capacity;
	}

	raw_device_capacity = memory_info_card->ext_csd.sectors;
	raw_device_capacity = ((((raw_device_capacity * 512) / 1024) / 1024) / 1024);

	if (raw_device_capacity > 512 && raw_device_capacity < 1024) {
		raw_device_capacity = 1024;
	} else if (raw_device_capacity > 256) {
		raw_device_capacity = 512;
	} else if (raw_device_capacity > 128) {
		raw_device_capacity = 256;
	} else if (raw_device_capacity > 64) {
		raw_device_capacity = 128;
	} else if (raw_device_capacity > 32) {
		raw_device_capacity = 64;
	} else if (raw_device_capacity > 16) {
		raw_device_capacity = 32;
	} else if (raw_device_capacity > 8) {
		raw_device_capacity = 16;
	} else if (raw_device_capacity > 6) {
		raw_device_capacity = 8;
	} else if (raw_device_capacity > 4) {
		raw_device_capacity = 6;
	} else if (raw_device_capacity > 3) {
		raw_device_capacity = 4;
	} else {
		pr_err("unknown emmc size %d\n", raw_device_capacity);
		raw_device_capacity = 0;
	}

	return raw_device_capacity;
}
#endif

static unsigned int get_ram_size(void)
{
    unsigned int ram_size, temp_size;
    struct sysinfo info;

    si_meminfo(&info);

    temp_size =  (info.totalram << (PAGE_SHIFT - 10)) / 1024;
    if (temp_size > 7168) {
        ram_size = 8;
    }  else if (temp_size > 5120) {
        ram_size = 6;
    } else if (temp_size > 3072) {
        ram_size = 4;
    } else if (temp_size > 2048) {
        ram_size = 3;
    } else if (temp_size > 1024) {
        ram_size = 2;
    } else if(temp_size > 512) {
        ram_size = 1;
    } else {
        ram_size = 0;
    }

    return ram_size;
}

static ssize_t huaqin_show(struct kobject *kobj, struct attribute *a, char *buf)
{
	ssize_t count = 0;
	//int i = 0;
	struct hw_info *hw = container_of(a, struct hw_info, attr);
	//struct PCBA_MSG *msg;
	//char ufs_vendor_name[emmc_len] = {0};

	//msg = get_pcba_msg();

	if (NULL == hw) {
		return sprintf(buf, "Data error\n");
	}

	if (HWID_VER == hw->hw_id) {
		count = sprintf(buf, "%s\n", HQ_SYS_FS_VER);
	} else if (HWID_SUMMARY == hw->hw_id) {
		//iterate all device and output the detail
		int iterator = 0;
		struct hw_info *curent_hw = NULL;
		struct attribute *attr = huaqin_attrs[iterator];

		while (attr) {
			curent_hw = container_of(attr, struct hw_info, attr);
			iterator += 1;
			attr = huaqin_attrs[iterator];

			if (curent_hw->hw_exist && (NULL != curent_hw->hw_device_name)) {
				count += sprintf(buf+count, "%s: %s\n", curent_hw->attr.name, curent_hw->hw_device_name);
			}
		}
	}
	 else if (HWID_PCBA == hw->hw_id) {
		/* sycamore code for SYCAMORE-54 by kexinzhang2 at 2024/11/15 start */
		char *sku_str = NULL;
		char *stage_str = NULL;
		char *board_id_str = NULL;
		int size = 0;
		struct device_node *board_id;
		board_id = of_find_node_by_path("/board_id");
		if(!board_id) {
			pr_err("warning: not find node: '/board_id'\n");
		}

		/* sycamore code for SYCAMORE-10161 by kexinzhang2 at 2025/1/23 start */
		sku_str = (char *)of_get_property(board_id, "androidboot.product.hardware.skuinfo", (int *)&size);
		/* sycamore code for SYCAMORE-10161 by kexinzhang2 at 2025/1/23 end */
		stage_str = (char *)of_get_property(board_id, "androidboot.stage", (int *)&size);
		board_id_str = (char *)of_get_property(board_id, "component_id", (int *)&size);

		count = sprintf(buf, "PCBA-%s-%s-%s\n", sku_str, stage_str, board_id_str);
		return count;
		/* sycamore code for SYCAMORE-54 by kexinzhang2 at 2024/11/15 end */
	}
	 else if (HWID_PCBA_INFO == hw->hw_id) {
		/* sycamore code for SYCAMORE-528 by chenzm13 at 2024/12/2 start */
		char *hwboardid_str = NULL;
		int size = 0;
		struct device_node *board_id;
		board_id = of_find_node_by_path("/board_id");
		if(!board_id) {
			pr_err("warning: not find node: '/board_id'\n");
		}

		hwboardid_str = (char *)of_get_property(board_id,
			"androidboot.hwboardid", (int *)&size);
		pr_err("hwboardid_str = %s\n", hwboardid_str);

		count = sprintf(buf, "%s\n", hwboardid_str);
		return count;
		/* sycamore code for SYCAMORE-528 by chenzm13 at 2024/12/2 end */
	}
	 else if (HWID_PMIC_VERSION == hw->hw_id) {
		count = sprintf(buf, "%s\n", "PMIC: MT6360U V1.2");
	}
	//else if (HWID_EMMC == hw->hw_id) {
	//	count = sprintf(buf, "emmc=%dG\n", get_emmc_information());
	//}
	else if (HWID_DDR == hw->hw_id) {
		count = sprintf(buf, "ram=%dG\n", get_ram_size());
	/* Clove code for CLOVE-117 by liyl69 at 2024/09/14 end */
	}
/* sycamore code for SYCAMORE-3317 by zhoujj21 at 2024/11/19 start */
	else if (HWID_LEDSTATE == hw->hw_id) {
		int ledState = gpio_get_value(320); //get gpio21 state
		if(ledState == 1)
			count = sprintf(buf, "%s\n", "support");
		else if(ledState == 0)
			count = sprintf(buf, "%s\n", "Not support");
		else
			count = sprintf(buf, "%s\n", "UNKNOW");
		return count;
	}
/* sycamore code for SYCAMORE-3317 by zhoujj21 at 2024/11/19 end */
	else if (HWID_SIMSTATE == hw->hw_id) {
		/* sycamore code for SYCAMORE-120 by zhourh6 at 2024/11/16 start */
		int simState = gpio_get_value(313); //get gpio14 state
		/* sycamore code for SYCAMORE-120 by zhourh6 at 2024/11/16 end */
		if(simState == 1)
			count = sprintf(buf, "%s\n", "Insert");
		else if(simState == 0)
			count = sprintf(buf, "%s\n", "DontInsert");
		else
			count = sprintf(buf, "%s\n", "UNKNOW");
		return count;
	} else {
		if (0 == hw->hw_exist) {
			count = sprintf(buf, "Not support\n");
		} else if (NULL == hw->hw_device_name) {
			count = sprintf(buf, "Installed with no device Name\n");
		} else {
			count = sprintf(buf, "%s\n", hw->hw_device_name);
		}
	}

	return count;
}

static ssize_t huaqin_store(struct kobject *kobj, struct attribute *a, const char *buf, size_t count)
{
	return count;
}

/* huaqin object */
static struct kobject huaqin_kobj;
static const struct sysfs_ops huaqin_sysfs_ops = {
	.show = huaqin_show,
	.store = huaqin_store,
};

/* huaqin type */
static struct kobj_type huaqin_ktype = {
	.sysfs_ops = &huaqin_sysfs_ops,
	.default_groups = huaqin_groups
};

/* huaqin device class */
static struct class  *huaqin_class;
static struct device *huaqin_hw_device;


int register_kboj_under_hqsysfs(struct kobject *kobj, struct kobj_type *ktype, const char *fmt, ...)
{
	return kobject_init_and_add(kobj, ktype, &(huaqin_hw_device->kobj), fmt);
}
EXPORT_SYMBOL_GPL(register_kboj_under_hqsysfs);

static int __init create_sysfs(void)
{
	int ret;

	/* create class (device model) */
	huaqin_class = class_create(THIS_MODULE, HUAQIN_CLASS_NAME);
	if (IS_ERR(huaqin_class)) {
		pr_err("%s fail to create class\n", __func__);
		return -1;
	}

	huaqin_hw_device = device_create(huaqin_class, NULL, MKDEV(0, 0), NULL, HUAIN_INTERFACE_NAME);
	if (IS_ERR(huaqin_hw_device)) {
		pr_warn("fail to create device\n");
		return -1;
	}

	/* add kobject */
	ret = kobject_init_and_add(&huaqin_kobj, &huaqin_ktype, &(huaqin_hw_device->kobj), HUAQIN_HWID_NAME);
	if (ret < 0) {
		pr_err("%s fail to add kobject\n", __func__);
		return ret;
	}

	return 0;
}

int hq_deregister_hw_info(enum hardware_id id, char *device_name)
{
	int ret = 0;
	int find_hw_id = 0;
	int iterator = 0;

	struct hw_info *hw = NULL;
	struct attribute *attr = huaqin_attrs[iterator];

	if (NULL == device_name) {
		pr_err("[%s]: device_name does not allow empty\n", __func__);
		ret = -2;
		goto err;
	}

	while (attr) {
		hw = container_of(attr, struct hw_info, attr);

		iterator += 1;
		attr = huaqin_attrs[iterator];

		if (NULL == hw) {
			continue;
		}

		if (id == hw->hw_id) {
			find_hw_id = 1;

			if (0 == hw->hw_exist) {
				pr_err("[%s]: device has not registed hw->id:0x%x . Cant be deregistered\n"
					, __func__
					, hw->hw_id);

				ret = -4;
				goto err;
			} else if (NULL == hw->hw_device_name) {

				pr_err("[%s]:hw_id is 0x%x Device name cant be NULL\n"
					, __func__
					, hw->hw_id);
				ret = -5;
				goto err;
			} else {
				if (0 == strncmp(hw->hw_device_name, device_name, strlen(hw->hw_device_name))) {
					hw->hw_device_name = NULL;
					hw->hw_exist = 0;
				} else{
					pr_err("[%s]: hw_id is 0x%x Registered device name %s, want to deregister: %s\n"
						, __func__
						, hw->hw_id
						, hw->hw_device_name
						, device_name);
					ret = -6;
					goto err;
				}
			}

			goto err;

		} else
			continue;

	}

	if (0 == find_hw_id) {
		pr_err("[%s]: Cant find correct hardware_id: 0x%x\n", __func__, id);
		ret = -3;
	}

err:
	return ret;

}

static char *audio_pa;
int hq_regiser_hw_info(enum hardware_id id, char *device_name)
{
	int ret = 0;
	int find_hw_id = 0;
	int iterator = 0;

	struct hw_info *hw = NULL;
	struct attribute *attr = huaqin_attrs[iterator];
	if (NULL == device_name) {
		pr_err("[%s]: device_name does not allow empty\n", __func__);
		ret = -2;
		goto err;
	}

	while (attr) {
		hw = container_of(attr, struct hw_info, attr);

		iterator += 1;
		attr = huaqin_attrs[iterator];

		if (NULL == hw) {
			continue;
		}

		if (id == hw->hw_id) {
			find_hw_id = 1;

			if (hw->hw_exist) {
				pr_err("[%s]: device has already registed hw->id:0x%x hw_device_name:%s\n"
					, __func__
					, hw->hw_id
					, hw->hw_device_name);
				ret = -4;
				goto err;
			}
			switch (hw->hw_id) {
			case HWID_AUDIO:
				audio_pa = device_name;
				break;
			default:
				break;
			}
			switch (hw->hw_id) {
				/*
					if(map_cam_drv_to_vendor(device_name))
						hw->hw_device_name = map_cam_drv_to_vendor(device_name);
					else
						hw->hw_device_name = "Can't find Camera Vendor";
					break;
				*/
			default:
					hw->hw_device_name = device_name;
					break;
			}
			hw->hw_exist = 1;
			goto err;

		} else
			continue;
	}
	if (0 == find_hw_id) {
		pr_err("[%s]: Cant find correct hardware_id: 0x%x\n", __func__, id);
		ret = -3;
	}
err:
	return ret;
}
EXPORT_SYMBOL(hq_regiser_hw_info);

#include <linux/proc_fs.h>
#include <linux/of_gpio.h>
#define PROC_BOOT_REASON_FILE "boot_status"
#define SDC_DETECT_STATUS "sdc_det_gpio_status"

#define SDC_DETECT_GPIO  343

static struct proc_dir_entry *boot_reason_proc;
static struct proc_dir_entry *sdc_detect_status;
static unsigned int boot_into_factory;
static int boot_reason_proc_show(struct seq_file *file, void *data)
{
	char temp[40] = {0};

	sprintf(temp, "%d\n", boot_into_factory);
	seq_printf(file, "%s\n", temp);
	return 0;
}

static int boot_reason_proc_open (struct inode *inode, struct file *file)
{
	return single_open(file, boot_reason_proc_show, inode->i_private);
}

static const struct proc_ops boot_reason_proc_fops = {
	.proc_open = boot_reason_proc_open,
	.proc_read = seq_read,
};
/*
static int __init get_boot_rease(char *str)
{
	if (strcmp("boot_with_factory", str) == 0) {
		boot_into_factory = 1;
	}

	return 0;
}
__setup("androidboot.boot_reason=", get_boot_rease);
*/
static int sdc_detect_proc_show(struct seq_file *file, void *data)
{
	int gpio_value = -1;
	gpio_value = gpio_get_value(SDC_DETECT_GPIO);
	seq_printf(file, "%d\n", gpio_value ? 1 : 0);
	return 0;
}

static int sdc_detect_proc_open (struct inode *inode, struct file *file)
{
	return single_open(file, sdc_detect_proc_show, inode->i_private);
}

static const struct proc_ops sdc_detect_proc_fops = {
	.proc_open = sdc_detect_proc_open,
	.proc_read = seq_read,
};

static int sdc_detect_init(void)
{
	sdc_detect_status = proc_create(SDC_DETECT_STATUS, 0644, NULL, &sdc_detect_proc_fops);
	if (sdc_detect_status == NULL) {
		pr_err("[%s]: create_proc_entry sdc_detect_status failed\n", __func__);
		return -1;
	}
	return 0;
}

static int __init hq_harware_init(void)
{
	/* create sysfs entry at /sys/class/huaqin/interface/hw_info */
	create_sysfs();

	boot_reason_proc = proc_create(PROC_BOOT_REASON_FILE, 0644, NULL, &boot_reason_proc_fops);

	if (boot_reason_proc == NULL) {
		pr_err("[%s]: create_proc_entry boot_reason_proc failed\n", __func__);
	}
	if (sdc_detect_init() < 0)
		pr_err("[%s]: create_proc_entry sdc_detect_proc failed\n", __func__);

	return 0;
}

char *get_audio_pa_vendor(void)
{
	return audio_pa;
}

EXPORT_SYMBOL(get_audio_pa_vendor);

core_initcall(hq_harware_init);
MODULE_AUTHOR("KaKa Ni <nigang@huaqin.com>");
MODULE_DESCRIPTION("Huaqin Hardware Info Driver");
MODULE_LICENSE("GPL");

