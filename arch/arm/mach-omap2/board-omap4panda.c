/*
 * Board support file for OMAP4430 based PandaBoard.
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Author: David Anders <x0132446@ti.com>
 *
 * Based on mach-omap2/board-4430sdp.c
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/spi/spi.h>
#include <linux/wl12xx.h>
#include <linux/netdevice.h>
#include <linux/if_ether.h>
#include <linux/ti_wilink_st.h>
#include <linux/omapfb.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/dmm.h>
#include <mach/id.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <video/omapdss.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mcspi.h>
#include <plat/mmc.h>
#include <plat/dma-44xx.h>
#include <video/omap-panel-generic-dpi.h>

#include "hsmmc.h"
#include "control.h"
#include "mux.h"
#include "common-board-devices.h"
#include "pm.h"

/* cyttsp */
#include <linux/cyttsp5_core.h>
#include <linux/cyttsp5_platform.h>

#define CYTTSP5_USE_I2C
/* #define CYTTSP5_USE_SPI */

#ifdef CYTTSP5_USE_I2C
#define CYTTSP5_I2C_TCH_ADR 0x24
#define CYTTSP5_LDR_TCH_ADR 0x24
#define CYTTSP5_I2C_IRQ_GPIO 38 /* J6.9, C19, GPMC_AD14/GPIO_38 */
#define CYTTSP5_I2C_RST_GPIO 37 /* J6.10, D18, GPMC_AD13/GPIO_37 */
#endif

#ifdef CYTTSP5_USE_SPI
/* Change GPIO numbers when using I2C and SPI at the same time
 * Following is possible alternative:
 * IRQ: J6.17, C18, GPMC_AD12/GPIO_36
 * RST: J6.24, D17, GPMC_AD11/GPIO_35
 */
#define CYTTSP5_SPI_IRQ_GPIO 38 /* J6.9, C19, GPMC_AD14/GPIO_38 */
#define CYTTSP5_SPI_RST_GPIO 37 /* J6.10, D18, GPMC_AD13/GPIO_37 */
#endif

/* Check GPIO numbers if both I2C and SPI are enabled */
#if defined(CYTTSP5_USE_I2C) && defined(CYTTSP5_USE_SPI)
#if CYTTSP5_I2C_IRQ_GPIO == CYTTSP5_SPI_IRQ_GPIO || \
	CYTTSP5_I2C_RST_GPIO == CYTTSP5_SPI_RST_GPIO
#error "GPIO numbers should be different when both I2C and SPI are on!"
#endif
#endif

#ifndef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT

#define CYTTSP5_HID_DESC_REGISTER 1

#define CY_VKEYS_X 720
#define CY_VKEYS_Y 1280
#define CY_MAXX 880
#define CY_MAXY 1280
#define CY_MINX 0
#define CY_MINY 0

#define CY_ABS_MIN_X CY_MINX
#define CY_ABS_MIN_Y CY_MINY
#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MIN_P 0
#define CY_ABS_MAX_P 255
#define CY_ABS_MIN_W 0
#define CY_ABS_MAX_W 255
#define CY_PROXIMITY_MIN_VAL	0
#define CY_PROXIMITY_MAX_VAL	1

#define CY_ABS_MIN_T 0

#define CY_ABS_MAX_T 15

/* Button to keycode conversion */
static u16 cyttsp5_btn_keys[] = {
	/* use this table to map buttons to keycodes (see input.h) */
	KEY_HOMEPAGE,		/* 172 */ /* Previously was KEY_HOME (102) */
				/* New Android versions use KEY_HOMEPAGE */
	KEY_MENU,		/* 139 */
	KEY_BACK,		/* 158 */
	KEY_SEARCH,		/* 217 */
	KEY_VOLUMEDOWN,		/* 114 */
	KEY_VOLUMEUP,		/* 115 */
	KEY_CAMERA,		/* 212 */
	KEY_POWER		/* 116 */
};

static struct touch_settings cyttsp5_sett_btn_keys = {
	.data = (uint8_t *)&cyttsp5_btn_keys[0],
	.size = ARRAY_SIZE(cyttsp5_btn_keys),
	.tag = 0,
};

static struct cyttsp5_core_platform_data _cyttsp5_core_platform_data = {
	.irq_gpio = CYTTSP5_I2C_IRQ_GPIO,
	.rst_gpio = CYTTSP5_I2C_RST_GPIO,
	.hid_desc_register = CYTTSP5_HID_DESC_REGISTER,
	.xres = cyttsp5_xres,
	.init = cyttsp5_init,
	.power = cyttsp5_power,
	.detect = cyttsp5_detect,
	.irq_stat = cyttsp5_irq_stat,
	.sett = {
		NULL,	/* Reserved */
		NULL,	/* Command Registers */
		NULL,	/* Touch Report */
		NULL,	/* Parade Data Record */
		NULL,	/* Test Record */
		NULL,	/* Panel Configuration Record */
		NULL,	/* &cyttsp5_sett_param_regs, */
		NULL,	/* &cyttsp5_sett_param_size, */
		NULL,	/* Reserved */
		NULL,	/* Reserved */
		NULL,	/* Operational Configuration Record */
		NULL, /* &cyttsp5_sett_ddata, *//* Design Data Record */
		NULL, /* &cyttsp5_sett_mdata, *//* Manufacturing Data Record */
		NULL,	/* Config and Test Registers */
		&cyttsp5_sett_btn_keys,	/* button-to-keycode table */
	},
	.flags = CY_CORE_FLAG_RESTORE_PARAMETERS,
	.easy_wakeup_gesture = CY_CORE_EWG_NONE,
};

static const int16_t cyttsp5_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0,
	ABS_MT_TOUCH_MINOR, 0, 255, 0, 0,
	ABS_MT_ORIENTATION, -127, 127, 0, 0,
	ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0,
	ABS_MT_DISTANCE, 0, 255, 0, 0,	/* Used with hover */
};

struct touch_framework cyttsp5_framework = {
	.abs = (uint16_t *)&cyttsp5_abs[0],
	.size = ARRAY_SIZE(cyttsp5_abs),
	.enable_vkeys = 0,
};

static struct cyttsp5_mt_platform_data _cyttsp5_mt_platform_data = {
	.frmwrk = &cyttsp5_framework,
	.flags = CY_MT_FLAG_INV_X | CY_MT_FLAG_INV_Y,
	.inp_dev_name = CYTTSP5_MT_NAME,
	.vkeys_x = CY_VKEYS_X,
	.vkeys_y = CY_VKEYS_Y,
};

static struct cyttsp5_btn_platform_data _cyttsp5_btn_platform_data = {
	.inp_dev_name = CYTTSP5_BTN_NAME,
};

static const int16_t cyttsp5_prox_abs[] = {
	ABS_DISTANCE, CY_PROXIMITY_MIN_VAL, CY_PROXIMITY_MAX_VAL, 0, 0,
};

struct touch_framework cyttsp5_prox_framework = {
	.abs = (uint16_t *)&cyttsp5_prox_abs[0],
	.size = ARRAY_SIZE(cyttsp5_prox_abs),
};

static struct cyttsp5_proximity_platform_data
		_cyttsp5_proximity_platform_data = {
	.frmwrk = &cyttsp5_prox_framework,
	.inp_dev_name = CYTTSP5_PROXIMITY_NAME,
};

static struct cyttsp5_platform_data _cyttsp5_platform_data = {
	.core_pdata = &_cyttsp5_core_platform_data,
	.mt_pdata = &_cyttsp5_mt_platform_data,
	.loader_pdata = &_cyttsp5_loader_platform_data,
	.btn_pdata = &_cyttsp5_btn_platform_data,
	.prox_pdata = &_cyttsp5_proximity_platform_data,
};

static ssize_t cyttsp5_virtualkeys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":"
		__stringify(KEY_BACK) ":1360:90:160:180:"
		__stringify(EV_KEY) ":"
		__stringify(KEY_MENU) ":1360:270:160:180:"
		__stringify(EV_KEY) ":"
		__stringify(KEY_HOMEPAGE) ":1360:450:160:180:"
		__stringify(EV_KEY) ":"
		__stringify(KEY_SEARCH) ":1360:630:160:180\n");
}

static struct kobj_attribute cyttsp5_virtualkeys_attr = {
	.attr = {
		.name = "virtualkeys.cyttsp5_mt",
		.mode = S_IRUGO,
	},
	.show = &cyttsp5_virtualkeys_show,
};

static struct attribute *cyttsp5_properties_attrs[] = {
	&cyttsp5_virtualkeys_attr.attr,
	NULL
};

static struct attribute_group cyttsp5_properties_attr_group = {
	.attrs = cyttsp5_properties_attrs,
};
#endif /* !CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT */

static void __init omap4_panda_cyttsp5_init(void)
#ifndef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
{
	struct kobject *properties_kobj;
	int ret = 0;

	/* Initialize muxes for GPIO pins */
#ifdef CYTTSP5_USE_I2C
	omap_mux_init_gpio(CYTTSP5_I2C_RST_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(CYTTSP5_I2C_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
#endif
#ifdef CYTTSP5_USE_SPI
	omap_mux_init_gpio(CYTTSP5_SPI_RST_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(CYTTSP5_SPI_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
#endif

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
				&cyttsp5_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("%s: failed to create board_properties\n", __func__);
}
#else /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT */
{
#ifdef CYTTSP5_USE_I2C
	omap_mux_init_gpio(CYTTSP5_I2C_RST_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(CYTTSP5_I2C_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
#endif
#ifdef CYTTSP5_USE_SPI
	omap_mux_init_gpio(CYTTSP5_SPI_RST_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(CYTTSP5_SPI_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
#endif
}
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT */

#define GPIO_HUB_POWER		1
#define GPIO_HUB_NRESET		62
#define GPIO_WIFI_PMENA		43
#define GPIO_WIFI_IRQ		53
#define HDMI_GPIO_CT_CP_HPD     60
#define HDMI_GPIO_HPD 63 /* Hot plug pin for HDMI */
#define HDMI_GPIO_LS_OE 41 /* Level shifter for HDMI */
#define TPS62361_GPIO   7

static int
panda_kim_suspend(struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int
panda_kim_resume(struct platform_device *pdev)
{
	return 0;
}

struct ti_st_plat_data panda_bt_platform_data = {
		.nshutdown_gpio = 46,
		.dev_name = "/dev/ttyO1",
		.flow_cntrl = 1,
		.baud_rate = 3000000,
		.suspend = panda_kim_suspend,
		.resume = panda_kim_resume,

};

static struct platform_device wl1271_device = {
	.name	= "kim",
	.id	= -1,
	.dev	= {
		.platform_data	= &panda_bt_platform_data,
	},
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static struct gpio_led gpio_leds[] = {
	{
		.name			= "pandaboard::status1",
		.default_trigger	= "heartbeat",
	},
	{
		.name			= "pandaboard::status2",
		.default_trigger	= "mmc0",
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static void __init panda_leds_init(void)
{
	if (cpu_is_omap443x()) {
		gpio_leds[0].gpio = 7;
		gpio_leds[1].gpio = 8;
	} else {
		gpio_leds[0].gpio = 110;
		gpio_leds[1].gpio = 8;
	}

	platform_device_register(&leds_gpio);
}

static struct platform_device *panda_devices[] __initdata = {
	&wl1271_device,
	&btwilink_device,
};

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static struct gpio panda_ehci_gpios[] __initdata = {
	{ GPIO_HUB_POWER,	GPIOF_OUT_INIT_LOW,  "hub_power"  },
	{ GPIO_HUB_NRESET,	GPIOF_OUT_INIT_LOW,  "hub_nreset" },
};

static void __init omap4_ehci_init(void)
{
	int ret;
	struct clk *phy_ref_clk;

	/* FREF_CLK3 provides the 19.2 MHz reference clock to the PHY */
	phy_ref_clk = clk_get(NULL, "auxclk3_ck");
	if (IS_ERR(phy_ref_clk)) {
		pr_err("Cannot request auxclk3\n");
		return;
	}
	clk_set_rate(phy_ref_clk, 19200000);
	clk_enable(phy_ref_clk);

	/* disable the power to the usb hub prior to init and reset phy+hub */
	ret = gpio_request_array(panda_ehci_gpios,
				 ARRAY_SIZE(panda_ehci_gpios));
	if (ret) {
		pr_err("Unable to initialize EHCI power/reset\n");
		return;
	}

	gpio_export(GPIO_HUB_POWER, 0);
	gpio_export(GPIO_HUB_NRESET, 0);
	gpio_set_value(GPIO_HUB_NRESET, 1);

	usbhs_init(&usbhs_bdata);

	/* enable power to hub */
	gpio_set_value(GPIO_HUB_POWER, 1);
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
	{
		.name		= "wl1271",
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply omap4_panda_vmmc5_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.4"),
};

static struct regulator_init_data panda_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(omap4_panda_vmmc5_supply),
	.consumer_supplies = omap4_panda_vmmc5_supply,
};

static struct fixed_voltage_config panda_vwlan = {
	.supply_name = "vwl1271",
	.microvolts = 1800000, /* 1.8V */
	.gpio = GPIO_WIFI_PMENA,
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &panda_vmmc5,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &panda_vwlan,
	},
};

struct wl12xx_platform_data omap_panda_wlan_data  __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	/* PANDA ref clock is 38.4 MHz */
	.board_ref_clock = 2,
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	if (!pdata) {
		dev_err(dev, "%s: NULL platform data\n", __func__);
		return -EINVAL;
	}
	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		 if (ret)
			dev_err(dev, "%s: Error card detect config(%d)\n",
				__func__, ret);
		 else
			pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed omap4_twl6030_hsmmc_set_late_init\n");
		return;
	}
	pdata = dev->platform_data;

	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static struct regulator_init_data omap4_panda_vaux1 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(omap4_panda_vmmc5_supply),
	.consumer_supplies      = omap4_panda_vmmc5_supply,
};

static struct regulator_init_data omap4_panda_vusim = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct twl4030_codec_data twl6040_codec = {
	/* single-step ramp for headset and handsfree */
	.hs_left_step	= 0x0f,
	.hs_right_step	= 0x0f,
	.hf_left_step	= 0x1d,
	.hf_right_step	= 0x1d,
};

static struct twl4030_vibra_data twl6040_vibra = {
	.vibldrv_res = 8,
	.vibrdrv_res = 3,
	.viblmotor_res = 10,
	.vibrmotor_res = 10,
	.vddvibl_uV = 0,	/* fixed volt supply - VBAT */
	.vddvibr_uV = 0,	/* fixed volt supply - VBAT */
};

static struct twl4030_audio_data twl6040_audio = {
	.audio_card_name = "Panda",
	.codec		= &twl6040_codec,
	.vibra		= &twl6040_vibra,
	.audpwron_gpio	= 127,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};
/* Panda board uses the common PMIC configuration */
static struct twl4030_platform_data omap4_panda_twldata = {
	.audio		= &twl6040_audio,
	/* Regulators */
	.vusim		= &omap4_panda_vusim,
	.vaux1		= &omap4_panda_vaux1,
};

/*
 * Display monitor features are burnt in their EEPROM as EDID data. The EEPROM
 * is connected as I2C slave device, and can be accessed at address 0x50
 */
static struct i2c_board_info panda_i2c_eeprom[] __initdata = {
	{
		I2C_BOARD_INFO("eeprom", 0x50),
	},
};

#ifndef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
static struct i2c_board_info panda_i2c4_devices[] __initdata = {
#ifdef CYTTSP5_USE_I2C
	{
		I2C_BOARD_INFO(CYTTSP5_I2C_NAME, CYTTSP5_I2C_TCH_ADR),
		.irq =  OMAP_GPIO_IRQ(CYTTSP5_I2C_IRQ_GPIO),
		.platform_data = &_cyttsp5_platform_data,
	},
#endif
};
#endif

static int __init omap4_panda_i2c_init(void)
{
	omap4_pmic_get_config(&omap4_panda_twldata, TWL_COMMON_PDATA_USB,
			TWL_COMMON_REGULATOR_VDAC |
			TWL_COMMON_REGULATOR_VAUX2 |
			TWL_COMMON_REGULATOR_VAUX3 |
			TWL_COMMON_REGULATOR_VMMC |
			TWL_COMMON_REGULATOR_VPP |
			TWL_COMMON_REGULATOR_VANA |
			TWL_COMMON_REGULATOR_VCXIO |
			TWL_COMMON_REGULATOR_VUSB |
			TWL_COMMON_REGULATOR_CLK32KG);
	omap4_pmic_init("twl6030", &omap4_panda_twldata);
	omap_register_i2c_bus(2, 400, NULL, 0);
	/*
	 * Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz
	 */
	omap_register_i2c_bus(3, 100, panda_i2c_eeprom,
					ARRAY_SIZE(panda_i2c_eeprom));
#ifndef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	omap_register_i2c_bus(4, 400, panda_i2c4_devices,
					ARRAY_SIZE(panda_i2c4_devices));
#endif
	return 0;
}

#ifdef CYTTSP5_USE_SPI
/*
 * This dummy structure is required, otherwise OMAP2 MCSPI driver
 * (assuming spi_board_info has a valid controller_data that is mapped to
 * omap2_mcspi_device_config) will cause a NULL pointer exception.
 */
static struct omap2_mcspi_device_config cyttsp5_mcspi_config = {
};
#endif

static struct spi_board_info panda_spi_devices[] __initdata = {
#ifndef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
#ifdef CYTTSP5_USE_SPI
	{
		.modalias	= CYTTSP5_SPI_NAME,
		.bus_num	= 1,
		.chip_select	= 0,
		.irq		= OMAP_GPIO_IRQ(CYTTSP5_SPI_IRQ_GPIO),
		.max_speed_hz	= 1000000,
		.mode		= SPI_MODE_0,
		.platform_data = &_cyttsp5_platform_data,
		.controller_data = &cyttsp5_mcspi_config,
	},
#endif
#endif
};

static void __init omap4_panda_spi_init(void)
{
	spi_register_board_info(panda_spi_devices,
					ARRAY_SIZE(panda_spi_devices));
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* WLAN IRQ - GPIO 53 */
/*	OMAP4_MUX(GPMC_NCS3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT), */
	/* WLAN POWER ENABLE - GPIO 43 */
/*	OMAP4_MUX(GPMC_A19, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT), */
	/* WLAN SDIO: MMC5 CMD */
	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 CLK */
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 DAT[0-3] */
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* gpio 0 - TFP410 PD */
	OMAP4_MUX(KPD_COL1, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* dispc2_data23 */
	OMAP4_MUX(USBB2_ULPITLL_STP, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data22 */
	OMAP4_MUX(USBB2_ULPITLL_DIR, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data21 */
	OMAP4_MUX(USBB2_ULPITLL_NXT, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data20 */
	OMAP4_MUX(USBB2_ULPITLL_DAT0, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data19 */
	OMAP4_MUX(USBB2_ULPITLL_DAT1, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data18 */
	OMAP4_MUX(USBB2_ULPITLL_DAT2, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data15 */
	OMAP4_MUX(USBB2_ULPITLL_DAT3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data14 */
	OMAP4_MUX(USBB2_ULPITLL_DAT4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data13 */
	OMAP4_MUX(USBB2_ULPITLL_DAT5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data12 */
	OMAP4_MUX(USBB2_ULPITLL_DAT6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data11 */
	OMAP4_MUX(USBB2_ULPITLL_DAT7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data10 */
	OMAP4_MUX(DPM_EMU3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data9 */
	OMAP4_MUX(DPM_EMU4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data16 */
	OMAP4_MUX(DPM_EMU5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data17 */
	OMAP4_MUX(DPM_EMU6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_hsync */
	OMAP4_MUX(DPM_EMU7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_pclk */
	OMAP4_MUX(DPM_EMU8, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_vsync */
	OMAP4_MUX(DPM_EMU9, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_de */
	OMAP4_MUX(DPM_EMU10, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data8 */
	OMAP4_MUX(DPM_EMU11, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data7 */
	OMAP4_MUX(DPM_EMU12, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data6 */
	OMAP4_MUX(DPM_EMU13, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data5 */
	OMAP4_MUX(DPM_EMU14, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data4 */
	OMAP4_MUX(DPM_EMU15, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data3 */
	OMAP4_MUX(DPM_EMU16, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data2 */
	OMAP4_MUX(DPM_EMU17, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data1 */
	OMAP4_MUX(DPM_EMU18, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data0 */
	OMAP4_MUX(DPM_EMU19, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static struct omap_device_pad serial2_pads[] __initdata = {
	OMAP_MUX_STATIC("uart2_cts.uart2_cts",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_rts.uart2_rts",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_rx.uart2_rx",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_tx.uart2_tx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};

static struct omap_device_pad serial3_pads[] __initdata = {
	OMAP_MUX_STATIC("uart3_cts_rctx.uart3_cts_rctx",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_rts_sd.uart3_rts_sd",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_rx_irrx.uart3_rx_irrx",
			 OMAP_PIN_INPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_tx_irtx.uart3_tx_irtx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};

static struct omap_device_pad serial4_pads[] __initdata = {
	OMAP_MUX_STATIC("uart4_rx.uart4_rx",
			 OMAP_PIN_INPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart4_tx.uart4_tx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};

static struct omap_board_data serial2_data __initdata = {
	.id             = 1,
	.pads           = serial2_pads,
	.pads_cnt       = ARRAY_SIZE(serial2_pads),
};

static struct omap_board_data serial3_data __initdata = {
	.id             = 2,
	.pads           = serial3_pads,
	.pads_cnt       = ARRAY_SIZE(serial3_pads),
};

static struct omap_board_data serial4_data __initdata = {
	.id             = 3,
	.pads           = serial4_pads,
	.pads_cnt       = ARRAY_SIZE(serial4_pads),
};

static inline void board_serial_init(void)
{
	struct omap_board_data bdata;

	bdata.flags     = 0;
	bdata.pads      = NULL;
	bdata.pads_cnt  = 0;
	bdata.id        = 0;
	/* pass dummy data for UART1 */
	omap_serial_init_port(&bdata);

	omap_serial_init_port(&serial2_data);
	omap_serial_init_port(&serial3_data);
	omap_serial_init_port(&serial4_data);
}
#else
#define board_mux	NULL

static inline void board_serial_init(void)
{
	omap_serial_init();
}
#endif

/* Display DVI */
#define PANDA_DVI_TFP410_POWER_DOWN_GPIO	0

static int omap4_panda_enable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(dssdev->reset_gpio, 1);
	return 0;
}

static void omap4_panda_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(dssdev->reset_gpio, 0);
}

/* Using generic display panel */
static struct panel_generic_dpi_data omap4_dvi_panel = {
	.name			= "generic_720p",
	.platform_enable	= omap4_panda_enable_dvi,
	.platform_disable	= omap4_panda_disable_dvi,
};

struct omap_dss_device omap4_panda_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.driver_name		= "generic_dpi_panel",
	.data			= &omap4_dvi_panel,
	.phy.dpi.data_lines	= 24,
	.reset_gpio		= PANDA_DVI_TFP410_POWER_DOWN_GPIO,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};

int __init omap4_panda_dvi_init(void)
{
	int r;

	/* Requesting TFP410 DVI GPIO and disabling it, at bootup */
	r = gpio_request_one(omap4_panda_dvi_device.reset_gpio,
				GPIOF_OUT_INIT_LOW, "DVI PD");
	if (r)
		pr_err("Failed to get DVI powerdown GPIO\n");

	return r;
}

static struct gpio panda_hdmi_gpios[] = {
	{ HDMI_GPIO_CT_CP_HPD,	GPIOF_OUT_INIT_HIGH, "hdmi_gpio_hpd"   },
	{ HDMI_GPIO_LS_OE,	GPIOF_OUT_INIT_HIGH, "hdmi_gpio_ls_oe" },
};

static void omap4_panda_hdmi_mux_init(void)
{
	u32 r;
	int status;
	/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
	omap_mux_init_signal("hdmi_hpd.hdmi_hpd",
				OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("gpmc_wait2.gpio_100",
			OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("hdmi_cec.hdmi_cec",
			OMAP_PIN_INPUT_PULLUP);
	/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
	omap_mux_init_signal("hdmi_ddc_scl.hdmi_ddc_scl",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("hdmi_ddc_sda.hdmi_ddc_sda",
			OMAP_PIN_INPUT_PULLUP);

	/* strong pullup on DDC lines using unpublished register */
	r = ((1 << 24) | (1 << 28));
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_1);

	gpio_request(HDMI_GPIO_HPD, NULL);
	omap_mux_init_gpio(HDMI_GPIO_HPD, OMAP_PIN_INPUT | OMAP_PULL_ENA);
	gpio_direction_input(HDMI_GPIO_HPD);

	status = gpio_request_array(panda_hdmi_gpios,
				    ARRAY_SIZE(panda_hdmi_gpios));
	if (status)
		pr_err("Cannot request HDMI GPIOs\n");
}

static struct omap_dss_device  omap4_panda_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},
		.hdmi	= {
			.regn	= 15,
			.regm2	= 1,
		},
	},
	.hpd_gpio = HDMI_GPIO_HPD,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

#ifdef CONFIG_MACH_OMAP4_PANDA_DVI_PRIMARY
static struct omap_dss_device *omap4_panda_dss_devices[] = {
	&omap4_panda_dvi_device,
	&omap4_panda_hdmi_device,
};

static struct omap_dss_board_info omap4_panda_dss_data = {
	.num_devices	= ARRAY_SIZE(omap4_panda_dss_devices),
	.devices	= omap4_panda_dss_devices,
	.default_device	= &omap4_panda_dvi_device,
};
#else
static struct omap_dss_device *omap4_panda_dss_devices[] = {
	&omap4_panda_hdmi_device,
	&omap4_panda_dvi_device,
};

static struct omap_dss_board_info omap4_panda_dss_data = {
	.num_devices	= ARRAY_SIZE(omap4_panda_dss_devices),
	.devices	= omap4_panda_dss_devices,
	.default_device	= &omap4_panda_hdmi_device,
};
#endif

void omap4_panda_display_init(void)
{
	int r;

	r = omap4_panda_dvi_init();
	if (r)
		pr_err("error initializing panda DVI\n");

	omap4_panda_hdmi_mux_init();
	omap_display_init(&omap4_panda_dss_data);
}

/*
 * These device paths represent the onboard USB <-> Ethernet bridge, and
 * the WLAN module on Panda, both of which need their random or all-zeros
 * mac address replacing with a per-cpu stable generated one
 */
static const char * const panda_fixup_mac_device_paths[] = {
	"usb1/1-1/1-1.1/1-1.1:1.0",
	"mmc1:0001:2",
};

static int panda_device_path_need_mac(struct device *dev)
{
	const char **try = (const char **)panda_fixup_mac_device_paths;
	const char *path;
	int count = ARRAY_SIZE(panda_fixup_mac_device_paths);
	const char *p;
	int len;
	struct device *devn;

	while (count--) {

		p = *try + strlen(*try);
		devn = dev;

		while (devn) {

			path = dev_name(devn);
			len = strlen(path);

			if ((p - *try) < len) {
				devn = NULL;
				continue;
			}

			p -= len;

			if (strncmp(path, p, len)) {
				devn = NULL;
				continue;
			}

			devn = devn->parent;
			if (p == *try)
				return count;

			if (devn != NULL && (p - *try) < 2)
				devn = NULL;

			p--;
			if (devn != NULL && *p != '/')
				devn = NULL;
		}

		try++;
	}

	return -ENOENT;
}

static int omap_panda_netdev_event(struct notifier_block *this,
						 unsigned long event, void *ptr)
{
	struct net_device *dev = ptr;
	struct sockaddr sa;
	int n;

	if (event != NETDEV_REGISTER)
		return NOTIFY_DONE;

	n = panda_device_path_need_mac(dev->dev.parent);
	if (n >= 0) {
		sa.sa_family = dev->type;
		omap2_die_id_to_ethernet_mac(sa.sa_data, n);
		dev->netdev_ops->ndo_set_mac_address(dev, &sa);
	}

	return NOTIFY_DONE;
}

static struct notifier_block omap_panda_netdev_notifier = {
	.notifier_call = omap_panda_netdev_event,
	.priority = 1,
};

#define PANDA_FB_RAM_SIZE                SZ_16M /* 1920×1080*4 * 2 */
static struct omapfb_platform_data panda_fb_pdata = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			[0] = {
				.size = PANDA_FB_RAM_SIZE,
			},
		},
	},
};

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
static const struct of_device_id omap_dt_match_table[] __initconst = {
	{ .compatible = "simple-bus", },
	{ .compatible = "ti,omap-infra", },
	{ }
};
#endif

static void __init omap4_panda_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;

	if (cpu_is_omap446x())
		twl6040_audio.audio_card_name = "PandaES";

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, NULL, package);

	omap_mux_init_gpio(GPIO_WIFI_IRQ, OMAP_PIN_INPUT |
			OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(GPIO_WIFI_PMENA, OMAP_PIN_OUTPUT);

	if (wl12xx_set_platform_data(&omap_panda_wlan_data))
		pr_err("error setting wl12xx data\n");

	register_netdevice_notifier(&omap_panda_netdev_notifier);

	omap4_panda_i2c_init();
	omap4_panda_spi_init();

	omap_enable_smartreflex_on_init();

	platform_add_devices(panda_devices, ARRAY_SIZE(panda_devices));
	panda_leds_init();
	platform_device_register(&omap_vwlan_device);
	board_serial_init();
	omap_sdrc_init(NULL, NULL);
	omap4_twl6030_hsmmc_init(mmc);
	omap4_ehci_init();
	usb_musb_init(&musb_board_data);
	omap_dmm_init();
	omapfb_set_platform_data(&panda_fb_pdata);
	omap4_panda_display_init();

	omap4_panda_cyttsp5_init();

	if (cpu_is_omap446x()) {
		/* Vsel0 = gpio, vsel1 = gnd */
		status = omap_tps6236x_board_setup(true, TPS62361_GPIO, -1,
				OMAP_PIN_OFF_OUTPUT_HIGH, -1);
		if (status)
			pr_err("TPS62361 initialization failed: %d\n", status);
	}

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	of_platform_populate(NULL, omap_dt_match_table, NULL, NULL);
#endif
}

static void __init omap4_panda_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

static const char *omap4_panda_match[] __initconst = {
	"ti,omap4-panda",
	NULL,
};

MACHINE_START(OMAP4_PANDA, "OMAP4 Panda board")
	/* Maintainer: David Anders - Texas Instruments Inc */
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap4_panda_map_io,
	.init_early	= omap4430_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= omap4_panda_init,
	.timer		= &omap4_timer,
	.dt_compat	= omap4_panda_match,
#ifdef CONFIG_ZONE_DMA
	.dma_zone_size  = 500 * 1024 * 1024,
#endif
MACHINE_END
