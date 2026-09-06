/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * Copyright (C) Shanghai FourSemi Semiconductor Co.,Ltd 2016-2023. All rights reserved.
 * 2023-12-28 File created.
 */

#ifndef __FRSM_I2CA_EXPORT_H__
#define __FRSM_I2CA_EXPORT_H__

int frsm_i2ca_get_ndev(void);
int frsm_i2ca_init_dev(int spkid, bool force);
int frsm_i2ca_set_scene(int spkid, int scene);
int frsm_i2ca_spk_switch(int spkid, bool on);

#endif // __FRSM_I2CA_EXPORT_H__

