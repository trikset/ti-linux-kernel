/*
 * FB driver for the ST7789V LCD Controller
 *
 * Copyright (C) 2015 Dennis Menschel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <video/mipi_display.h>

#include "fbtft.h"

#define DRVNAME "fb_st7789v-tilcdc"

#define DEFAULT_GAMMA \
	"70 2C 2E 15 10 09 48 33 53 0B 19 18 20 25\n" \
	"70 2C 2E 15 10 09 48 33 53 0B 19 18 20 25"

struct da8xx_priv {
	char *reg_base;
	struct clk *clk;
};

/**
 * enum st7789v_command - ST7789V display controller commands
 *
 * @PORCTRL: porch setting
 * @GCTRL: gate control
 * @VCOMS: VCOM setting
 * @VDVVRHEN: VDV and VRH command enable
 * @VRHS: VRH set
 * @VDVS: VDV set
 * @VCMOFSET: VCOM offset set
 * @PWCTRL1: power control 1
 * @PVGAMCTRL: positive voltage gamma control
 * @NVGAMCTRL: negative voltage gamma control
 *
 * The command names are the same as those found in the datasheet to ease
 * looking up their semantics and usage.
 *
 * Note that the ST7789V display controller offers quite a few more commands
 * which have been omitted from this list as they are not used at the moment.
 * Furthermore, commands that are compliant with the MIPI DCS have been left
 * out as well to avoid duplicate entries.
 */
enum st7789v_command {
	PORCTRL = 0xB2,
	GCTRL = 0xB7,
	VCOMS = 0xBB,
	VDVVRHEN = 0xC2,
	VRHS = 0xC3,
	VDVS = 0xC4,
	VCMOFSET = 0xC5,
	PWCTRL1 = 0xD0,
	PVGAMCTRL = 0xE0,
	NVGAMCTRL = 0xE1,
};

#define MADCTL_BGR BIT(3) /* bitmask for RGB/BGR order */
#define MADCTL_MV BIT(5) /* bitmask for page/column order */
#define MADCTL_MX BIT(6) /* bitmask for column address order */
#define MADCTL_MY BIT(7) /* bitmask for page address order */

/* LCD Block */
#define  LCD_PID_REG				0x0
#define  LCD_CTRL_REG				0x4
#define  LCD_STAT_REG				0x8
#define  LCD_LIDD_CTRL_REG			0xc
#define  LCD_CS0_CONF_REG			0x10
#define  LCD_CS0_ADDR_REG			0x14
#define  LCD_CS0_DATA_REG			0x18
#define  LCD_RASTER_CTRL_REG			0x28
#define  LCD_RASTER_TIMING_0_REG		0x2C
#define  LCD_RASTER_TIMING_1_REG		0x30
#define  LCD_RASTER_TIMING_2_REG		0x34
#define  LCD_DMA_CTRL_REG			0x40
#define  LCD_DMA_FRM_BUF_BASE_ADDR_0_REG	0x44
#define  LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG	0x48
#define  LCD_DMA_FRM_BUF_BASE_ADDR_1_REG	0x4C
#define  LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG	0x50

static void da8xx_lcdc_reg_write(unsigned int val, char *addr)
{
	__raw_writel(val, addr);
}

static int da8xx_init_lcdc(struct fbtft_par *par)
{
	int ret;
	struct clk *clk;
	char *reg_base;
	struct da8xx_priv *priv;

	clk = clk_get_sys("da8xx_lcdc.0", "fck");
	if (IS_ERR(clk))
	{
		printk(KERN_ALERT "TI LCDC cannot get clock\n");
		return PTR_ERR(clk);
	}
	ret = clk_prepare_enable(clk);
	if (ret < 0)
	{
		printk(KERN_ALERT "TI LCDC cannot prepare_enable clock\n");
		clk_put(clk);
		return ret;
	}
	/* HARDCODED */
	reg_base = ioremap(0x01e13000, 0x1000);
	if (!reg_base)
	{
		clk_disable_unprepare(clk);
		clk_put(clk);
		return -ENXIO;
	}
	da8xx_lcdc_reg_write(0x0100, reg_base + LCD_CTRL_REG); // Select mode=LIDD, clkdiv=0
	da8xx_lcdc_reg_write(3, reg_base + LCD_LIDD_CTRL_REG); // Disable DMA, set async MPU80 mode
	da8xx_lcdc_reg_write(0x07FE0FFC, reg_base + LCD_CS0_CONF_REG); // Failsafe settings

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
	{
		iounmap(reg_base);
		clk_disable_unprepare(clk);
		clk_put(clk);
		return -ENOMEM;
	}
	priv->clk = clk;
	priv->reg_base = reg_base;
	par->extra = priv;
	return 0;
}

/**
 * init_display() - initialize the display controller
 *
 * @par: FBTFT parameter object
 *
 * Most of the commands in this init function set their parameters to the
 * same default values which are already in place after the display has been
 * powered up. (The main exception to this rule is the pixel format which
 * would default to 18 instead of 16 bit per pixel.)
 * Nonetheless, this sequence can be used as a template for concrete
 * displays which usually need some adjustments.
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int init_display(struct fbtft_par *par)
{
	int ret = da8xx_init_lcdc(par);
	if (ret)
		return ret;

	par->fbtftops.reset(par);

	/* turn off sleep mode */
	write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
	mdelay(120);

	/* set pixel format to RGB-565 */
	write_reg(par, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT);

	write_reg(par, PORCTRL, 0x08, 0x08, 0x00, 0x22, 0x22);

	/*
	 * VGH = 13.26V
	 * VGL = -10.43V
	 */
	write_reg(par, GCTRL, 0x35);

	/*
	 * VDV and VRH register values come from command write
	 * (instead of NVM)
	 */
	write_reg(par, VDVVRHEN, 0x01, 0xFF);

	/*
	 * VAP =  4.1V + (VCOM + VCOM offset + 0.5 * VDV)
	 * VAN = -4.1V + (VCOM + VCOM offset + 0.5 * VDV)
	 */
	write_reg(par, VRHS, 0x0B);

	/* VDV = 0V */
	write_reg(par, VDVS, 0x20);

	/* VCOM = 0.9V */
	write_reg(par, VCOMS, 0x20);

	/* VCOM offset = 0V */
	write_reg(par, VCMOFSET, 0x20);

	/*
	 * AVDD = 6.8V
	 * AVCL = -4.8V
	 * VDS = 2.3V
	 */
	write_reg(par, PWCTRL1, 0xA4, 0xA1);

	write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
	return 0;
}

/**
 * set_var() - apply LCD properties like rotation and BGR mode
 *
 * @par: FBTFT parameter object
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int set_var(struct fbtft_par *par)
{
	u8 madctl_par = 0;

	if (par->bgr)
		madctl_par |= MADCTL_BGR;
	switch (par->info->var.rotate) {
	case 0:
		break;
	case 90:
		madctl_par |= (MADCTL_MV | MADCTL_MY);
		break;
	case 180:
		madctl_par |= (MADCTL_MX | MADCTL_MY);
		break;
	case 270:
		madctl_par |= (MADCTL_MV | MADCTL_MX);
		break;
	default:
		return -EINVAL;
	}
	write_reg(par, MIPI_DCS_SET_ADDRESS_MODE, madctl_par);
	return 0;
}

/**
 * set_gamma() - set gamma curves
 *
 * @par: FBTFT parameter object
 * @curves: gamma curves
 *
 * Before the gamma curves are applied, they are preprocessed with a bitmask
 * to ensure syntactically correct input for the display controller.
 * This implies that the curves input parameter might be changed by this
 * function and that illegal gamma values are auto-corrected and not
 * reported as errors.
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int set_gamma(struct fbtft_par *par, u32 *curves)
{
	int i;
	int j;
	int c; /* curve index offset */

	/*
	 * Bitmasks for gamma curve command parameters.
	 * The masks are the same for both positive and negative voltage
	 * gamma curves.
	 */
	static const u8 gamma_par_mask[] = {
		0xFF, /* V63[3:0], V0[3:0]*/
		0x3F, /* V1[5:0] */
		0x3F, /* V2[5:0] */
		0x1F, /* V4[4:0] */
		0x1F, /* V6[4:0] */
		0x3F, /* J0[1:0], V13[3:0] */
		0x7F, /* V20[6:0] */
		0x77, /* V36[2:0], V27[2:0] */
		0x7F, /* V43[6:0] */
		0x3F, /* J1[1:0], V50[3:0] */
		0x1F, /* V57[4:0] */
		0x1F, /* V59[4:0] */
		0x3F, /* V61[5:0] */
		0x3F, /* V62[5:0] */
	};

	for (i = 0; i < par->gamma.num_curves; i++) {
		c = i * par->gamma.num_values;
		for (j = 0; j < par->gamma.num_values; j++)
			curves[c + j] &= gamma_par_mask[j];
		write_reg(
			par, PVGAMCTRL + i,
			curves[c + 0], curves[c + 1], curves[c + 2],
			curves[c + 3], curves[c + 4], curves[c + 5],
			curves[c + 6], curves[c + 7], curves[c + 8],
			curves[c + 9], curves[c + 10], curves[c + 11],
			curves[c + 12], curves[c + 13]);
	}
	return 0;
}

/**
 * blank() - blank the display
 *
 * @par: FBTFT parameter object
 * @on: whether to enable or disable blanking the display
 *
 * Return: 0 on success, < 0 if error occurred.
 */
static int blank(struct fbtft_par *par, bool on)
{
	if (on)
		write_reg(par, MIPI_DCS_SET_DISPLAY_OFF);
	else
		write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
	return 0;
}

static void write_register(struct fbtft_par *par, int len, ...)
{
	va_list args;
	u16 *buf = (u16*)par->buf;
	struct da8xx_priv *priv = (struct da8xx_priv*)(par->extra);
	char *base = priv->reg_base;
	int i;

	va_start(args, len);
	for (i = 0; i < len; i++)
		buf[i] = (u16)va_arg(args, unsigned int);
	va_end(args);
	fbtft_par_dbg_hex(DEBUG_WRITE_REGISTER, par, par->info->device, u16, buf, len, "%s: ", __func__);
	da8xx_lcdc_reg_write(buf[0], base + LCD_CS0_ADDR_REG);
	for (i = 1; i < len; i++)
		da8xx_lcdc_reg_write(buf[i], base + LCD_CS0_DATA_REG);
}

static int write_vmem(struct fbtft_par *par, size_t offset, size_t len)
{
	int i = 0;
	struct da8xx_priv *priv = (struct da8xx_priv*)(par->extra);
	char *base = priv->reg_base;
	for (i = 0; i < len; i += 2)
	{
		da8xx_lcdc_reg_write(*(u16*)(par->info->screen_buffer + offset + i), base + LCD_CS0_DATA_REG);
	}
	return 0;
}

static int verify_gpios(struct fbtft_par *par)
{
	if (par->gpio.reset < 0) {
		dev_err(par->info->device, "Missing 'reset' gpio. Aborting.\n");
		return -EINVAL;
	}

	return 0;
}

static struct fbtft_display display = {
	.regwidth = 8,
	.width = 240,
	.height = 320,
	.gamma_num = 2,
	.gamma_len = 14,
	.gamma = DEFAULT_GAMMA,
	.fbtftops = {
		.init_display = init_display,
		.set_var = set_var,
		.set_gamma = set_gamma,
		.blank = blank,
		.write_register = write_register,
		.write_vmem = write_vmem,
		.verify_gpios = verify_gpios,
	},
};

static int fbtft_driver_probe_pdev(struct platform_device *pdev)
{
	return fbtft_probe_common(&display, NULL, pdev);
}

static int fbtft_driver_remove_pdev(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct fbtft_par *par = info->par;
	struct da8xx_priv *priv = (struct da8xx_priv*)(par->extra);

	fbtft_remove_common(&pdev->dev, info);
	if (priv)
	{
		iounmap(priv->reg_base);
		clk_disable_unprepare(priv->clk);
		clk_put(priv->clk);
		kfree(priv);
	}
	return 0;
}

static const struct of_device_id dt_ids[] = {
	{ .compatible = "sitronix,st7789v-tilcdc" },
	{},
};

MODULE_DEVICE_TABLE(of, dt_ids);

static struct platform_driver fbtft_driver_platform_driver = {
	.driver = {
		.name   = DRVNAME,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(dt_ids),
	},
	.probe  = fbtft_driver_probe_pdev,
	.remove = fbtft_driver_remove_pdev,
};

static int __init fbtft_driver_module_init(void)
{
	return platform_driver_register(&fbtft_driver_platform_driver);
}

static void __exit fbtft_driver_module_exit(void)
{
	platform_driver_unregister(&fbtft_driver_platform_driver);
}

module_init(fbtft_driver_module_init);
module_exit(fbtft_driver_module_exit);

MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("platform:st7789v-tilcdc");

MODULE_DESCRIPTION("FB driver for the ST7789V LCD Controller on TI LCDC");
MODULE_AUTHOR("Kirill Smirnov");
MODULE_LICENSE("GPL");
