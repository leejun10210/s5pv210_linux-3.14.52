/* linux/arch/arm/mach-s5pv210/mach-smdkv210.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/device.h>
#include <linux/dm9000.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/pwm_backlight.h>
#include <linux/platform_data/s3c-hsotg.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <video/platform_lcd.h>
#include <video/samsung_fimd.h>

#include <mach/map.h>
#include <mach/regs-clock.h>

#include <plat/regs-serial.h>
#include <plat/regs-srom.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/adc.h>
#include <linux/platform_data/touchscreen-s3c2410.h>
#include <linux/platform_data/ata-samsung_cf.h>
#include <linux/platform_data/i2c-s3c2410.h>
#include <plat/keypad.h>
#include <plat/pm.h>
#include <plat/fb.h>
#include <plat/samsung-time.h>
#include <plat/backlight.h>
#include <plat/mfc.h>
#include <plat/clock.h>
#include <linux/input/kxtj9.h>

#include "common.h"

#define AT070TN92       1
#define DISP_MODE       AT070TN92


/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDKV210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDKV210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDKV210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdkv210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
};

static struct s3c_ide_platdata smdkv210_ide_pdata __initdata = {
	.setup_gpio	= s5pv210_ide_setup_gpio,
};

static uint32_t smdkv210_keymap[] __initdata = {
	/* KEY(row, col, keycode) */
	KEY(0, 3, KEY_1), KEY(0, 4, KEY_2), KEY(0, 5, KEY_3),
	KEY(0, 6, KEY_4), KEY(0, 7, KEY_5),
	KEY(1, 3, KEY_A), KEY(1, 4, KEY_B), KEY(1, 5, KEY_C),
	KEY(1, 6, KEY_D), KEY(1, 7, KEY_E)
};

static struct matrix_keymap_data smdkv210_keymap_data __initdata = {
	.keymap		= smdkv210_keymap,
	.keymap_size	= ARRAY_SIZE(smdkv210_keymap),
};

static struct samsung_keypad_platdata smdkv210_keypad_data __initdata = {
	.keymap_data	= &smdkv210_keymap_data,
	.rows		= 8,
	.cols		= 8,
};

#ifdef CONFIG_INPUT_KXTJ9

static int kxtj9_power_on(void)
{
	s3c_gpio_cfgpin(S5PV210_GPD0(3), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(S5PV210_GPD0(3), S3C_GPIO_PULL_DOWN);
	gpio_set_value(S5PV210_GPD0(3), 1);
}

static int kxtj9_power_off(void)
{
	s3c_gpio_cfgpin(S5PV210_GPD0(3), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(S5PV210_GPD0(3), S3C_GPIO_PULL_DOWN);
	gpio_set_value(S5PV210_GPD0(3), 1);

}

static struct kxtj9_platform_data kxtj9_pdata __initdata = {
	.min_interval = 50,
	.init_interval=1000,
	.res_12bit = RES_12BIT,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
//	.power_on = kxtj9_power_on,
//	.power_off = kxtj9_power_off,
};


#endif /* CONFIG_INPUT_KXTJ9 */


#define DM9000_BASE_ADDR	(S5PV210_PA_SROM_BANK1+0x300)		
static struct resource smdkv210_dm9000_resources[] = {
	[0] = DEFINE_RES_MEM(DM9000_BASE_ADDR, 4),
	[1] = DEFINE_RES_MEM(DM9000_BASE_ADDR + 4, 4),
	[2] = DEFINE_RES_NAMED(IRQ_EINT(10), 1, NULL, IORESOURCE_IRQ \
				| IORESOURCE_IRQ_HIGHLEVEL),
};

static struct dm9000_plat_data smdkv210_dm9000_platdata = {
	.flags		= DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM,
	.dev_addr	= { 0x00, 0x09, 0xc0, 0xff, 0xec, 0x48 },
};

static struct platform_device smdkv210_dm9000 = {
	.name		= "dm9000",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smdkv210_dm9000_resources),
	.resource	= smdkv210_dm9000_resources,
	.dev		= {
		.platform_data	= &smdkv210_dm9000_platdata,
	},
};

#ifdef CONFIG_S3C_FB_EK070TN93
static void smdkv210_ek070tn93_set_power(struct plat_lcd_data *pd,
					unsigned int power)
{
	printk("---------------%s----------------\n",__func__);
	if (power) {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(S5PV210_GPD0(0), GPIOF_OUT_INIT_LOW, "GPD0");
		gpio_free(S5PV210_GPD0(0));
#endif
		/* fire nRESET on power up */
		gpio_request_one(S5PV210_GPF3(5), GPIOF_OUT_INIT_HIGH, "GPF3");
		gpio_set_value(S5PV210_GPF3(5), 0);
		mdelay(10);
		gpio_set_value(S5PV210_GPF3(5), 1);
		mdelay(10);
		gpio_free(S5PV210_GPF3(5));
	} else {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(S5PV210_GPD0(0), GPIOF_OUT_INIT_HIGH, "GPD0");
		gpio_free(S5PV210_GPD0(0));
#endif
	}
}

static struct plat_lcd_data smdkv210_lcd_ek070tn93_data = {
	.set_power	= smdkv210_ek070tn93_set_power,
};

static struct platform_device smdkv210_lcd_ek070tn93 = {
	.name			= "platform-lcd",
	.dev.parent 	= &s3c_device_fb.dev,
	.dev.platform_data	= &smdkv210_lcd_ek070tn93_data,
};

static struct s3c_fb_pd_win smdkv210_fb_win0 = {
	.max_bpp	= 32,
	.default_bpp	= 24,
	.xres		= 1024,
	.yres		= 768,
};

static struct fb_videomode smdkv210_lcd_timing = {
	.left_margin	= 38,
	.right_margin	= 210,
	.upper_margin	= 18,
	.lower_margin	= 22,
	.hsync_len	= 10,
	.vsync_len	= 7,
	.xres		= 1024,
	.yres		= 768,
//	.pixclock	= 60,
};
#endif

#ifdef CONFIG_S3C_FB_LTE480WV
static void smdkv210_lte480wv_set_power(struct plat_lcd_data *pd,
					unsigned int power)
{
	if (power) {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(S5PV210_GPD0(3), GPIOF_OUT_INIT_HIGH, "GPD0");
		gpio_free(S5PV210_GPD0(3));
#endif

		/* fire nRESET on power up */
		gpio_request_one(S5PV210_GPH0(6), GPIOF_OUT_INIT_HIGH, "GPH0");

		gpio_set_value(S5PV210_GPH0(6), 0);
		mdelay(10);

		gpio_set_value(S5PV210_GPH0(6), 1);
		mdelay(10);

		gpio_free(S5PV210_GPH0(6));
	} else {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(S5PV210_GPD0(3), GPIOF_OUT_INIT_LOW, "GPD0");
		gpio_free(S5PV210_GPD0(3));
#endif
	}
}

static struct platform_device smdkv210_lcd_lte480wv = {
	.name			= "platform-lcd",
	.dev.parent		= &s3c_device_fb.dev,
	.dev.platform_data	= &smdkv210_lcd_lte480wv_data,
};

static struct s3c_fb_pd_win smdkv210_fb_win0 = {
	.max_bpp	= 32,
	.default_bpp	= 24,
	.xres		= 800,
	.yres		= 480,
};

static struct fb_videomode smdkv210_lcd_timing = {
	.left_margin	= 13,
	.right_margin	= 8,
	.upper_margin	= 7,
	.lower_margin	= 5,
	.hsync_len	= 3,
	.vsync_len	= 1,
	.xres		= 800,
	.yres		= 480,
};

#endif

#if defined(CONFIG_S3C_FB_LTE480WV) || defined(CONFIG_S3C_FB_EK070TN93)
static struct s3c_fb_platdata smdkv210_lcd0_pdata __initdata = {
	.win[0]		= &smdkv210_fb_win0,
	.vtiming	= &smdkv210_lcd_timing,
	.vidcon0	= (4<<6) |(1<<4)|(1<<0)|(1<<1),//VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
	.setup_gpio	= s5pv210_fb_gpio_setup_24bpp,
};
#endif

/* USB HOST */
static struct s3c_hsotg_plat smdkv210_ehci_pdata;

/* USB OTG */
static struct s3c_hsotg_plat smdkv210_hsotg_pdata;

static struct platform_device *smdkv210_devices[] __initdata = {
	&s3c_device_adc,
	&s3c_device_cfcon,
#ifdef CONFIG_S3C_DEV_FB	
	&s3c_device_fb,
#ifdef CONFIG_S3C_FB_EK070TN93
		&smdkv210_lcd_ek070tn93,
#endif	
#ifdef CONFIG_S3C_FB_LTE480WV
		&smdkv210_lcd_lte480wv,
#endif
#endif /* CONFIG_S3C_DEV_FB */

#ifdef CONFIG_S3C_DEV_HSMMC	
	&s3c_device_hsmmc0,
#endif /* CONFIG_S3C_DEV_HSMMC */

#ifdef CONFIG_S3C_DEV_HSMMC1
1	&s3c_device_hsmmc1,
#endif /* CONFIG_S3C_DEV_HSMMC1 */	
	
#ifdef CONFIG_S3C_DEV_HSMMC2
	&s3c_device_hsmmc2,
#endif /* CONFIG_S3C_DEV_HSMMC2 */	
		
#ifdef CONFIG_S3C_DEV_HSMMC3
	&s3c_device_hsmmc3,
#endif /* CONFIG_S3C_DEV_HSMMC3 */	
	&s3c_device_i2c0,
	
#ifdef CONFIG_S3C_DEV_I2C1
	&s3c_device_i2c1,
#endif
		
#ifdef CONFIG_S3C_DEV_I2C2
	&s3c_device_i2c2,
#endif

#ifdef CONFIG_SAMSUNG_DEV_PWM	
	&samsung_device_pwm,
#endif	
	&s3c_device_rtc,
	&s3c_device_ts,
#ifdef CONFIG_S5P_DEV_USB_EHCI
	&s3c_device_ehci,
#endif
	&s3c_device_usb_hsotg,
	&s3c_device_wdt,
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc_md,
	&s5p_device_jpeg,
	&s5p_device_mfc,
	&s5p_device_mfc_l,
	&s5p_device_mfc_r,
	&s5pv210_device_ac97,
	&s5pv210_device_iis0,
	&s5pv210_device_spdif,
	&samsung_asoc_idma,
//	&samsung_device_keypad,


#ifdef CONFIG_DM9000
	&smdkv210_dm9000
#endif /* CONFIG_DM9000 */
};

#ifdef CONFIG_DM9000
static void __init smdkv210_dm9000_init(void)
{
	unsigned int tmp;
	int ret = 0;

	/* Input mode */
	s3c_gpio_cfgpin(S5PV210_GPH1(2), S3C_GPIO_INPUT);
	s3c_gpio_setpull(S5PV210_GPH1(2), S3C_GPIO_PULL_NONE);
	ret = gpio_request(S5PV210_GPH1(2), "GPH1");
	if(ret)
		printk("mach-smdk210: request gpio GPH1(2) fail");
	else
	{
		s3c_gpio_cfgpin(S5PV210_GPH1(2), 0xf);
		s3c_gpio_setpull(S5PV210_GPH1(2), S3C_GPIO_PULL_NONE);
	}
	
	gpio_request(S5PV210_MP01(1), "nCS1");
	s3c_gpio_cfgpin(S5PV210_MP01(1), S3C_GPIO_SFN(2));
	gpio_free(S5PV210_MP01(1));

	tmp = (5 << S5P_SROM_BCX__TACC__SHIFT);
	__raw_writel(tmp, S5P_SROM_BC1);

	tmp = __raw_readl(S5P_SROM_BW);
	tmp &= (S5P_SROM_BW__CS_MASK << S5P_SROM_BW__NCS1__SHIFT);
	tmp |= (0xf << S5P_SROM_BW__NCS1__SHIFT);
	__raw_writel(tmp, S5P_SROM_BW);
}
#endif /* CONFIG_DM9000 */
static struct i2c_board_info smdkv210_i2c_devs0[] __initdata = {
#ifdef CONFIG_INPUT_KXTJ9
	{ 
	I2C_BOARD_INFO("kxtj9", 0x0f), 
	.platform_data = &kxtj9_pdata,
	},
#endif	/* CONFIG_INPUT_KXTJ9 */
	{ I2C_BOARD_INFO("wm8580", 0x1b), },
};

static struct i2c_board_info smdkv210_i2c_devs1[] __initdata = {
	/* To Be Updated */
#ifdef CONFIG_TOUCHSCREEN_EDT_FT5X06
	{ 
	I2C_BOARD_INFO("edt-ft5x06", 0x70>>1), 
	},
#endif	/* CONFIG_TOUCHSCREEN_EDT_FT5X06 */
};

static struct i2c_board_info smdkv210_i2c_devs2[] __initdata = {
	/* To Be Updated */
};
#ifdef CONFIG_SAMSUNG_DEV_BACKLIGHT
/* LCD Backlight data */
static struct samsung_bl_gpio_info smdkv210_bl_gpio_info = {
	.no = S5PV210_GPD0(0),
	.func = S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data smdkv210_bl_data = {
	.pwm_id = 3,
	.pwm_period_ns = 1000,
	.enable_gpio = -1,
};
#endif /* CONFIG_SAMSUNG_DEV_BACKLIGHT */
static void __init smdkv210_map_io(void)
{
	s5pv210_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(smdkv210_uartcfgs, ARRAY_SIZE(smdkv210_uartcfgs));
#ifdef CONFIG_SAMSUNG_DEV_PWM
	samsung_set_timer_source(SAMSUNG_PWM2, SAMSUNG_PWM4);
#endif
}

static void smdkv210_backlight_off(void)
{
	/* backlight enable pin low level */
	s3c_gpio_cfgpin(S5PV210_GPF3(5), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(S5PV210_GPF3(5), S3C_GPIO_PULL_DOWN);
	gpio_set_value(S5PV210_GPF3(5), 0);

	s3c_gpio_cfgpin(S5PV210_GPF3(5), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(S5PV210_GPF3(5), S3C_GPIO_PULL_UP);
	gpio_set_value(S5PV210_GPF3(5), 1);

}



static void __init smdkv210_reserve(void)
{
		printk("%s\n",__func__);
	s5p_mfc_reserve_mem(0x43000000, 8 << 20, 0x51000000, 8 << 20);
}

static void __init smdkv210_machine_init(void)
{
	printk("----------%s-------\n",__func__);
	s3c_pm_init();
#ifdef CONFIG_DM9000
	smdkv210_dm9000_init();
#endif  /* CONFIG_DM9000 */
//	smdkv210_backlight_off();
	s3c24xx_ts_set_platdata(NULL);

	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, smdkv210_i2c_devs0,
		ARRAY_SIZE(smdkv210_i2c_devs0));	
#ifdef CONFIG_S3C_DEV_I2C1
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, smdkv210_i2c_devs1,
			ARRAY_SIZE(smdkv210_i2c_devs1));
#endif

#ifdef CONFIG_S3C_DEV_I2C2
	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(2, smdkv210_i2c_devs2,
			ARRAY_SIZE(smdkv210_i2c_devs2));

#endif

	s3c_ide_set_platdata(&smdkv210_ide_pdata);

	s3c_fb_set_platdata(&smdkv210_lcd0_pdata);

	s3c_hsotg_set_platdata(&smdkv210_hsotg_pdata);
#ifdef CONFIG_S5P_DEV_USB_EHCI	
	s3c_ehci_set_platdata(&smdkv210_ehci_pdata);
#endif
	platform_add_devices(smdkv210_devices, ARRAY_SIZE(smdkv210_devices));
#ifdef CONFIG_SAMSUNG_DEV_BACKLIGHT
	samsung_bl_set(&smdkv210_bl_gpio_info, &smdkv210_bl_data);
#endif
}

MACHINE_START(SMDKV210, "SMDKV210")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.atag_offset	= 0x100,
	.init_irq	= s5pv210_init_irq,
	.map_io		= smdkv210_map_io,
	.init_machine	= smdkv210_machine_init,
	.init_time	= samsung_timer_init,
	.restart	= s5pv210_restart,
	.reserve	= &smdkv210_reserve,
MACHINE_END
