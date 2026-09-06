// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#include <linux/hqsysfs.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

/*Sycamore code for SYCAMORE-4639 by zenghui4 at 2024.12.12 start*/
extern void update_firmware_for_lcm_esd_nt36523n(void);
extern bool nt36523n_esd_flag;
extern bool nt36523n_gesture_flag;
/*Sycamore code for SYCAMORE-4639 by zenghui4 at 2024.12.12 end*/

extern int _lcm_i2c_write_bytes(unsigned char addr, unsigned char value);
/*sycamore code for sycamore-8847 by liyc40 at  2025.1.14 start*/
extern int _lcm_i2c_read_bytes(unsigned char addr, unsigned char value);
/*sycamore code for sycamore-8847 by liyc40 at  2025.1.14 end*/
#define VAC (2560)
#define HAC (1600)
/*sycamore code for sycamore-9094 by liyc40 at  2025.1.15 start*/
/*sycamore code for sycamore-3748 by liyechun at 2024.11.14 start*/
#define DATA_RATE	1100
/*sycamore code for sycamore-3748 by liyechun at 2024.11.14 end*/
#define HSA		4
#define HBP		50
#define HFP		66
#define VSA		2
#define VBP		166
#define VFP		30
/*sycamore code for sycamore-9094 by liyc40 at  2025.1.15 end*/
#define FRAME_HEIGHT 	2560
#define FRAME_WIDTH	1600

#define DSC_ENABLE                  1
#define DSC_VER                     17
#define DSC_SLICE_MODE              1
#define DSC_RGB_SWAP                0
#define DSC_DSC_CFG                 34
#define DSC_RCT_ON                  1
#define DSC_BIT_PER_CHANNEL         8
#define DSC_DSC_LINE_BUF_DEPTH      9
#define DSC_BP_ENABLE               1
#define DSC_BIT_PER_PIXEL           128
#define DSC_SLICE_HEIGHT            32
#define DSC_SLICE_WIDTH             800
#define DSC_CHUNK_SIZE              800
#define DSC_XMIT_DELAY              512
#define DSC_DEC_DELAY               690
#define DSC_SCALE_VALUE             32
#define DSC_INCREMENT_INTERVAL      900
#define DSC_DECREMENT_INTERVAL      11
#define DSC_LINE_BPG_OFFSET         13
#define DSC_NFL_BPG_OFFSET          859
#define DSC_SLICE_BPG_OFFSET        545
#define DSC_INITIAL_OFFSET          6144
#define DSC_FINAL_OFFSET            4320
#define DSC_FLATNESS_MINQP          3
#define DSC_FLATNESS_MAXQP          12
#define DSC_RC_MODEL_SIZE           8192
#define DSC_RC_EDGE_FACTOR          6
#define DSC_RC_QUANT_INCR_LIMIT0    11
#define DSC_RC_QUANT_INCR_LIMIT1    11
#define DSC_RC_TGT_OFFSET_HI        3
#define DSC_RC_TGT_OFFSET_LO        3

/* sycamore code for sycamore-4725 by chenzm13 at  2024.12.13 start */
struct tag_videolfb {
	u64 fb_base;
	u32 islcmfound;
	u32 fps;
	u32 vram;
	char lcmname[64];
};
/* sycamore code for sycamore-4725 by chenzm13 at  2024.12.13 end */

unsigned int rc_buf_thresh[14]={896, 1792, 2688, 3584, 4480, 5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};
unsigned int range_min_qp[15]={0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 7, 13};
unsigned int range_max_qp[15]={4, 4, 5, 6, 7, 7, 7, 8, 9, 10, 11, 12, 13, 13, 15};
unsigned int range_bpg_ofs[15]={2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12};

static u32 fake_heigh = 2560;
static u32 fake_width = 1600;
static bool need_fake_resolution;

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;

	bool prepared;
	bool enabled;

	int error;
};

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void lcm_panel_init(struct lcm *ctx)
{
	pr_info("%s start \n", __func__);
        /*sycamore code for sycamore-3749 by liyc40 at  2024.11.26 start*/
        /*sycamore code for sycamore-4375 by liyc40 at  2024.12.2 start*/
	gpiod_set_value(ctx->reset_gpio, 1);
	mdelay(5);
	gpiod_set_value(ctx->reset_gpio, 0);
	mdelay(5);
	gpiod_set_value(ctx->reset_gpio, 1);
	mdelay(5);
	gpiod_set_value(ctx->reset_gpio, 0);
	mdelay(5);
	gpiod_set_value(ctx->reset_gpio, 1);
	mdelay(11);
        /*sycamore code for sycamore-4375 by liyc40 at  2024.12.2 end*/
        /*sycamore code for sycamore-3749 by liyc40 at  2024.11.26 end*/
	lcm_dcs_write_seq_static(ctx, 0xFF, 0xE0);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x14, 0x60);
	lcm_dcs_write_seq_static(ctx, 0x16, 0xC0);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x3B, 0x03,0xA8,0x1E,0x04,0x04,0x00);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xB9, 0x28,0x00,0xC7,0x00,0x80,0xA0,0x40,0x70,0x40);
	lcm_dcs_write_seq_static(ctx, 0xBC, 0x33);
        /*sycamore code for sycamore-3749 by liyechun at  2024.11.25 start*/
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x27);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xD0, 0x13);
	lcm_dcs_write_seq_static(ctx, 0xD1, 0x54);
	lcm_dcs_write_seq_static(ctx, 0xDE, 0x43);
	lcm_dcs_write_seq_static(ctx, 0xDF, 0x02);

        lcm_dcs_write_seq_static(ctx, 0xFF, 0x26);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x13, 0x8B);
	lcm_dcs_write_seq_static(ctx, 0x19, 0x1A);
	lcm_dcs_write_seq_static(ctx, 0x1A, 0x2E);
	lcm_dcs_write_seq_static(ctx, 0x1B, 0x18);
	lcm_dcs_write_seq_static(ctx, 0x1C, 0x42);
	lcm_dcs_write_seq_static(ctx, 0x2A, 0x1A);
	lcm_dcs_write_seq_static(ctx, 0x2B, 0x2E);
	lcm_dcs_write_seq_static(ctx, 0xCD, 0x3E,0x66);
	lcm_dcs_write_seq_static(ctx, 0xCE, 0x3C,0x66);
	lcm_dcs_write_seq_static(ctx, 0xCF, 0x0A,0x00,0x20);
        /*sycamore code for sycamore-3749 by liyechun at  2024.11.25 end*/

        /*sycamore code for sycamore-17294 by liyc40 at  2025.3.10 start*/
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x20);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x5A, 0x14);
        /*sycamore code for sycamore-17294 by liyc40 at  2025.3.10 end*/

        /*sycamore code for sycamore-17294 by liyc40 at  2025.3.7 start*/
	lcm_dcs_write_seq_static(ctx, 0xFF,0x26);
	lcm_dcs_write_seq_static(ctx, 0xFB,0x01);
	lcm_dcs_write_seq_static(ctx, 0xA9,0x8C);
	lcm_dcs_write_seq_static(ctx, 0xAA,0x8D);
	lcm_dcs_write_seq_static(ctx, 0xAB,0x8E);
	lcm_dcs_write_seq_static(ctx, 0xAC,0x8F);
	lcm_dcs_write_seq_static(ctx, 0xAD,0x90);
	lcm_dcs_write_seq_static(ctx, 0xAE,0x91);
	lcm_dcs_write_seq_static(ctx, 0xAF,0x92);
	lcm_dcs_write_seq_static(ctx, 0xB0,0x93);
	lcm_dcs_write_seq_static(ctx, 0xB1,0x94);
	lcm_dcs_write_seq_static(ctx, 0xB2,0x95);
	lcm_dcs_write_seq_static(ctx, 0xB3,0x96);
	lcm_dcs_write_seq_static(ctx, 0xB4,0x97);
	lcm_dcs_write_seq_static(ctx, 0xB5,0x98);
	lcm_dcs_write_seq_static(ctx, 0xB6,0x99);
        /*sycamore code for sycamore-17294 by liyc40 at  2025.3.7 end*/

	/*sycamore code for sycamore-18475 by liyc40 at  2025.3.12 start*/
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x25);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x17, 0xCF);
	lcm_dcs_write_seq_static(ctx, 0x19, 0xFF);
	lcm_dcs_write_seq_static(ctx, 0x1B, 0x5B);
	lcm_dcs_write_seq_static(ctx, 0x1C, 0xF0);
	/*sycamore code for sycamore-18475 by liyc40 at  2025.3.12 end*/

	/*sycamore code for sycamore-5314 by liyc40 at  2024.12.31 start*/
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x27);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xDE, 0x43);
	lcm_dcs_write_seq_static(ctx, 0xDF, 0x00);
	/*sycamore code for sycamore-5314 by liyc40 at  2024.12.31 end*/
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x90, 0x03);
	lcm_dcs_write_seq_static(ctx, 0x91, 0x89, 0x28, 0x00, 0x20, 0xD2, 0x00, 0x02, 0xB2, 0x03, 0x84, 0x00, 0x0B, 0x03, 0x5B, 0x02, 0x21);
	lcm_dcs_write_seq_static(ctx, 0x92, 0x10,0xE0);
        /*sycamore code for sycamore-4375 by liyc40 at  2024.12.2 start*/
        lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
        lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
        /*sycamore code for sycamore-4375 by liyc40 at  2024.12.2 end*/
	lcm_dcs_write_seq_static(ctx, 0x11);
        /*sycamore code for sycamore-3749 by liyc40 at  2024.11.26 start*/
	mdelay(120);
	lcm_dcs_write_seq_static(ctx, 0x29);
	/*sycamore code for sycamore-5314 by liyc40 at  2024.12.31 start*/
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x55);
	lcm_dcs_write_seq_static(ctx, 0xF1, 0xAA);
	lcm_dcs_write_seq_static(ctx, 0xF2, 0x66);
	/*sycamore code for sycamore-5314 by liyc40 at  2024.12.31 end*/
	mdelay(20);
	pr_info("%s end \n", __func__);
        /*sycamore code for sycamore-3749 by liyc40 at  2024.11.26 end*/
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s start \n", __func__);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	pr_info("%s end \n", __func__);

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s start \n", __func__);

	if (!ctx->prepared)
		return 0;
	/*Sycamore code for SYCAMORE-7792 by liyc40 at 2025.1.13 start*/
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xAA);
	lcm_dcs_write_seq_static(ctx, 0xF1, 0x55);
	lcm_dcs_write_seq_static(ctx, 0xF2, 0x99);
	/*Sycamore code for SYCAMORE-7792 by liyc40 at 2025.1.13 end*/
	lcm_dcs_write_seq_static(ctx, 0x28);
	mdelay(50);
	lcm_dcs_write_seq_static(ctx, 0x10);
	mdelay(150);

	ctx->error = 0;
	ctx->prepared = false;
	/*Sycamore code for SYCAMORE-4639 by zenghui4 at 2024.12.12 start*/
	if (nt36523n_gesture_flag == true){
		pr_info("%s NVT tianma gesture mode on\n", __func__);
		return 0;
	}
	/*Sycamore code for SYCAMORE-4639 by zenghui4 at 2024.12.12 end*/
	gpiod_set_value(ctx->bias_neg, 0);
	mdelay(2);
	gpiod_set_value(ctx->bias_pos, 0);
	mdelay(2);

	pr_info("%s end \n", __func__);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;
        /*sycamore code for sycamore-8847 by liyc40 at  2025.1.14 start*/
	int read_recv = 0;
	pr_info("%s start \n", __func__);

	if (ctx->prepared)
		return 0;

	read_recv = _lcm_i2c_read_bytes(0x04, 1);
	pr_info("_lcm_i2c_read_bytes:%d\n", read_recv);
	if(read_recv != 0x06){
		_lcm_i2c_write_bytes(0x00, 0x12);
		_lcm_i2c_write_bytes(0x01, 0x12);

		_lcm_i2c_write_bytes(0x03, 0x43);
		_lcm_i2c_write_bytes(0x04, 0x09);

	}else{
		_lcm_i2c_write_bytes(0x00, 0x12);
		_lcm_i2c_write_bytes(0x01, 0x12);
		_lcm_i2c_write_bytes(0x02, 0x22);
	}
        /*sycamore code for sycamore-8847 by liyc40 at  2025.1.14 end*/
	mdelay(2);
	gpiod_set_value(ctx->bias_pos, 1);
	mdelay(2);
	gpiod_set_value(ctx->bias_neg, 1);
        /*sycamore code for sycamore-11067 by liyc40 at  2025.1.26 start*/
	mdelay(12);
        /*sycamore code for sycamore-11067 by liyc40 at  2025.1.26 end*/
	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	/*Sycamore code for SYCAMORE-4639 by zenghui4 at 2024.12.12 start*/
	if (nt36523n_esd_flag == true) {
		update_firmware_for_lcm_esd_nt36523n();
		pr_info("%s NVT tm esd protecion \n", __func__);
	}
	/*Sycamore code for SYCAMORE-4639 by zenghui4 at 2024.12.12 end*/

	pr_info("%s end \n", __func__);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s start \n", __func__);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	pr_info("%s end \n", __func__);

	return 0;
}

static struct drm_display_mode default_mode = {
	.clock = (HAC + HFP + HSA + HBP)*(VAC + VFP + VSA + VBP)*90/1000,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP,
	.vsync_end = VAC + VFP + VSA,
	.vtotal = VAC + VFP + VSA + VBP,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = DATA_RATE / 2,
	//.vfp_low_power = 4205,
	.cust_esd_check = 0,
        /*sycamore code for sycamore-3749 by liyechun at  2024.11.25 start*/
	.esd_check_enable = 1,
        /*sycamore code for sycamore-3749 by liyechun at  2024.11.25 end*/
	.lcm_esd_check_table[0] = {
        /*sycamore code for sycamore-3749 by liyc40 at  2024.11.26 start*/
		.cmd = 0x00, .count = 1, .para_list[0] = 0x9C,
        /*sycamore code for sycamore-3749 by liyc40 at  2024.11.26 end*/
	},

	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable                =  DSC_ENABLE,
		.ver                   =  DSC_VER,
		.slice_mode            =  DSC_SLICE_MODE,
		.rgb_swap              =  DSC_RGB_SWAP,
		.dsc_cfg               =  DSC_DSC_CFG,
		.rct_on                =  DSC_RCT_ON,
		.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
		.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
		.bp_enable             =  DSC_BP_ENABLE,
		.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
		.pic_height            =  FRAME_HEIGHT,
		.pic_width             =  FRAME_WIDTH,
		.slice_height          =  DSC_SLICE_HEIGHT,
		.slice_width           =  DSC_SLICE_WIDTH,
		.chunk_size            =  DSC_CHUNK_SIZE,
		.xmit_delay            =  DSC_XMIT_DELAY,
		.dec_delay             =  DSC_DEC_DELAY,
		.scale_value           =  DSC_SCALE_VALUE,
		.increment_interval    =  DSC_INCREMENT_INTERVAL,
		.decrement_interval    =  DSC_DECREMENT_INTERVAL,
		.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
		.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
		.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
		.initial_offset        =  DSC_INITIAL_OFFSET,
		.final_offset          =  DSC_FINAL_OFFSET,
		.flatness_minqp        =  DSC_FLATNESS_MINQP,
		.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
		.rc_model_size         =  DSC_RC_MODEL_SIZE,
		.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
		.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
		.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
		.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
		.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,

		.ext_pps_cfg.enable=1,
		.ext_pps_cfg.rc_buf_thresh=rc_buf_thresh,
	        .ext_pps_cfg.range_min_qp=range_min_qp,
                /*sycamore code for sycamore-10774 by liyc40 at  2025.2.11 start*/
	        .ext_pps_cfg.range_max_qp=range_max_qp,
                /*sycamore code for sycamore-10774 by liyc40 at  2025.2.11 end*/
	        .ext_pps_cfg.range_bpg_ofs=range_bpg_ofs,
		},
	.data_rate = DATA_RATE,
        /*sycamore code for sycamore-9094 by liyc40 at  2025.1.15 start*/
	.data_rate_khz = 1100113,
        /*sycamore code for sycamore-9094 by liyc40 at  2025.1.15 end*/
	/*sycamore code for sycamore-9397 by liyc40 at  2024.1.23 start*/
        .vdo_per_frame_lp_enable = 0,
        .lp_perline_en = 0,
	/*sycamore code for sycamore-9397 by liyc40 at  2024.1.23 end*/

};

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	gpiod_set_value(ctx->reset_gpio, on);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
#ifdef IF_ZERO
	struct lcm *ctx = panel_to_lcm(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3];
	unsigned char id[3] = {0x00, 0x00, 0x00};
	ssize_t ret;
#endif

	pr_info("%s success\n", __func__);
#ifdef IF_ZERO
	ret = mipi_dsi_dcs_read(dsi, 0x4, data, 3);
	if (ret < 0)
		dev_info("%s error\n", __func__);

	pr_info("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0] &&
			data[1] == id[1] &&
			data[2] == id[2])
		return 1;

	pr_info("ATA expect read data is %x %x %x\n",
			id[0], id[1], id[2]);
#endif
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	pr_info("%s backlight = %d\n", level);
	return 0;
}

static int lcm_get_virtual_heigh(void)
{
	return VAC;
}

static int lcm_get_virtual_width(void)
{
	return HAC;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
	.get_virtual_heigh = lcm_get_virtual_heigh,
	.get_virtual_width = lcm_get_virtual_width,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static void change_drm_disp_mode_params(struct drm_display_mode *mode)
{
	if (fake_heigh > 0 && fake_heigh < VAC) {
		mode->vsync_start = mode->vsync_start - mode->vdisplay
					+ fake_heigh;
		mode->vsync_end = mode->vsync_end - mode->vdisplay + fake_heigh;
		mode->vtotal = mode->vtotal - mode->vdisplay + fake_heigh;
		mode->vdisplay = fake_heigh;
	}
	if (fake_width > 0 && fake_width < HAC) {
		mode->hsync_start = mode->hsync_start - mode->hdisplay
					+ fake_width;
		mode->hsync_end = mode->hsync_end - mode->hdisplay + fake_width;
		mode->htotal = mode->htotal - mode->hdisplay + fake_width;
		mode->hdisplay = fake_width;
	}
}

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	if (need_fake_resolution) {
		change_drm_disp_mode_params(&default_mode);
	}
	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = 64;
	connector->display_info.height_mm = 129;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static void check_is_need_fake_resolution(struct device *dev)
{
	unsigned int ret = 0;

	ret = of_property_read_u32(dev->of_node, "fake_heigh", &fake_heigh);
	if (ret)
		need_fake_resolution = false;
	ret = of_property_read_u32(dev->of_node, "fake_width", &fake_width);
	if (ret)
		need_fake_resolution = false;
	if (fake_heigh > 0 && fake_heigh < VAC)
		need_fake_resolution = true;
	if (fake_width > 0 && fake_width < HAC)
		need_fake_resolution = true;
}

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	/* sycamore code for sycamore-4725 by chenzm13 at  2024.12.13 start */
	struct device_node *chosen;
	struct tag_videolfb *videolfb_tag = NULL;
	int ret;
	unsigned long size = 0;
	/*sycamore code for sycamore-18633 by liyc40 at  2024.12.23 start*/
	int prc_panel = 0;
	/*sycamore code for sycamore-18633 by liyc40 at  2024.12.23 end*/
	pr_info("sycamore-nt36672c-dsi-vdo-tianma %s start \n", __func__);

	chosen = of_find_node_by_path("/chosen");
	if (chosen) {
		videolfb_tag = (struct tag_videolfb *)of_get_property(chosen,"atag,videolfb",(int *)&size);
		if (!videolfb_tag) {
			pr_err("Invalid lcm name!!!");
			return -ENODEV;
		}
		pr_info("read lcm name : %s", videolfb_tag->lcmname);

		if (strcmp("panel_tianma_sycamore_90hz_drv", videolfb_tag->lcmname) == 0) {
			pr_info("This is sycamore-nt36672c-dsi-vdo-tianma\n");
		} else {
			pr_info("Not sycamore-nt36672c-dsi-vdo-tianma\n");
			return -ENODEV;
		}
	}
	/* sycamore code for sycamore-4725 by chenzm13 at  2024.12.13 end */

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO
					| MIPI_DSI_MODE_VIDEO_SYNC_PULSE
					| MIPI_DSI_MODE_LPM
					| MIPI_DSI_MODE_NO_EOT_PACKET
					| MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_info(dev, "cannot get bias_pos %ld\n",
			PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_info(dev, "cannot get bias_neg %ld\n",
			PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}

	/*sycamore code for sycamore-18633 by liyc40 at  2024.12.23 start*/
	ret = of_property_read_u32(dev->of_node, "prc_panel", &prc_panel);
	dev_info(dev, "prc_panel: %d ret:%d\n",prc_panel,ret);
	if(prc_panel){
		ext_params.physical_width_um = 148081;
		ext_params.physical_height_um = 236928;
	}else{
		ext_params.physical_width_um = 147408;
		ext_params.physical_height_um = 235852;
	}
	/*sycamore code for sycamore-18633 by liyc40 at  2024.12.23 end*/
	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	check_is_need_fake_resolution(dev);
	pr_info("%s end \n", __func__);
	hq_regiser_hw_info(HWID_LCM, "tianma_sycamore_90hz_drv");
	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	devm_gpiod_put(dev, ctx->bias_neg);
	devm_gpiod_put(dev, ctx->bias_pos);
	devm_gpiod_put(dev, ctx->reset_gpio);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

	return 0;
}

/* sycamore code for sycamore-4725 by chenzm13 at  2024.12.13 start */
static const struct of_device_id lcm_of_match[] = {
	{
	    .compatible = "panel,sycamore",
	},
	{}
};
/* sycamore code for sycamore-4725 by chenzm13 at  2024.12.13 end */

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel_tianma_sycamore_90hz_drv",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("MEDIATEK");
MODULE_DESCRIPTION("panel tianma sycamore 90hz drv");
MODULE_LICENSE("GPL v2");

