/*
 * Copyright (C) 2016 Stefan Roese <sr@denx.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <i2c.h>
#include <power/regulator.h>
#ifdef CONFIG_BOARD_CONFIG_EEPROM
#include <mvebu/cfg_eeprom.h>
#endif
#include <asm-generic/gpio.h>
#include <dt-bindings/gpio/armada-3700-gpio.h>
#include <dm/device.h>
#include <dm/lists.h>
#include <dm/pinctrl.h>
#include <dm/uclass.h>
#include <nmxx_common.h>

DECLARE_GLOBAL_DATA_PTR;

/* on Armada3700 rev2 devel-board, IO expander (with I2C address 0x22) bit
 * 14 is used as Serdes Lane 2 muxing, which could be used as SATA PHY or
 * USB3 PHY.
 */
enum COMPHY_LANE2_MUXING {
	COMPHY_LANE2_MUX_USB3,
	COMPHY_LANE2_MUX_SATA
};

/* IO expander I2C device */
#define I2C_IO_EXP_ADDR		0x22
#define I2C_IO_CFG_REG_0	0x6
#define I2C_IO_DATA_OUT_REG_0	0x2
#define I2C_IO_REG_0_SATA_OFF	2
#define I2C_IO_REG_0_USB_H_OFF	1
#define I2C_IO_COMPHY_SATA3_USB_MUX_BIT	14

/*
* For Armada3700 A0 chip, comphy serdes lane 2 could be used as PHY for SATA
* or USB3.
* For Armada3700 rev2 devel-board, pin 14 of IO expander PCA9555 with I2C
* address 0x22 is used as Serdes Lane 2 muxing; the pin needs to be set in
* output mode: high level is for SATA while low level is for USB3;
*/
static int board_comphy_usb3_sata_mux(enum COMPHY_LANE2_MUXING comphy_mux)
{
	int ret;
	u8 buf[8];
	struct udevice *i2c_dev;
	int i2c_byte, i2c_bit_in_byte;

	if (!of_machine_is_compatible("marvell,armada-3720-db-v2") &&
	    !of_machine_is_compatible("marvell,armada-3720-db-v3"))
		return 0;

	ret = i2c_get_chip_for_busnum(0, I2C_IO_EXP_ADDR, 1, &i2c_dev);
	if (ret) {
		printf("Cannot find PCA9555: %d\n", ret);
		return 0;
	}

	ret = dm_i2c_read(i2c_dev, I2C_IO_CFG_REG_0, buf, 2);
	if (ret) {
		printf("Failed to read IO expander value via I2C\n");
		return ret;
	}

	i2c_byte = I2C_IO_COMPHY_SATA3_USB_MUX_BIT / 8;
	i2c_bit_in_byte = I2C_IO_COMPHY_SATA3_USB_MUX_BIT % 8;

	/* Configure IO exander bit 14 of address 0x22 in output mode */
	buf[i2c_byte] &= ~(1 << i2c_bit_in_byte);
	ret = dm_i2c_write(i2c_dev, I2C_IO_CFG_REG_0, buf, 2);
	if (ret) {
		printf("Failed to set IO expander via I2C\n");
		return ret;
	}

	ret = dm_i2c_read(i2c_dev, I2C_IO_DATA_OUT_REG_0, buf, 2);
	if (ret) {
		printf("Failed to read IO expander value via I2C\n");
		return ret;
	}

	/* Configure output level for IO exander bit 14 of address 0x22 */
	if (comphy_mux == COMPHY_LANE2_MUX_SATA)
		buf[i2c_byte] |= (1 << i2c_bit_in_byte);
	else
		buf[i2c_byte] &= ~(1 << i2c_bit_in_byte);

	ret = dm_i2c_write(i2c_dev, I2C_IO_DATA_OUT_REG_0, buf, 2);
	if (ret) {
		printf("Failed to set IO expander via I2C\n");
		return ret;
	}

	return 0;
}

int board_early_init_f(void)
{
#ifdef CONFIG_BOARD_CONFIG_EEPROM
	cfg_eeprom_init();
#endif

#ifdef CONFIG_MVEBU_SYS_INFO
	/*
	 * Call this function to transfer data from address 0x4000000
	 * into a global struct, before code relocation.
	 */
	sys_info_init();
#endif
	return 0;
}

int board_usb2_vbus_init(void)
{
#if defined(CONFIG_DM_REGULATOR)
	struct udevice *regulator;
	int ret;

	/* lower usb vbus  */
	ret = regulator_get_by_platname("usb2-vbus", &regulator);
	if (ret) {
		debug("Cannot get usb2-vbus regulator\n");
		return 0;
	}

	ret = regulator_set_enable(regulator, false);
	if (ret) {
		error("Failed to turn OFF the VBUS regulator\n");
		return ret;
	}
#endif
	return 0;
}

int board_usb3_vbus_init(void)
{
#if defined(CONFIG_DM_REGULATOR)
	struct udevice *regulator;
	int ret;

	/* lower usb vbus  */
	ret = regulator_get_by_platname("usb3-vbus", &regulator);
	if (ret) {
		debug("Cannot get usb3-vbus regulator\n");
		return 0;
	}

	ret = regulator_set_enable(regulator, false);
	if (ret) {
		error("Failed to turn OFF the VBUS regulator\n");
		return ret;
	}
#endif
	return 0;
}

static int get_board_type(void)
{
	int value = 0, ret;
	unsigned int selector = 0, gpio[3];;
	struct udevice *dev;
	const struct pinctrl_ops *ops;

	/* set jtag function as gpio mode */
	ret = uclass_get_device(UCLASS_PINCTRL, 0, &dev);
	if (ret) {
		 printf("Cannot get armada-37xx-pinctrl udevice\n");
		 return 0;
	}
	ops = pinctrl_get_ops(dev);
	if (!ops) {
		dev_dbg(dev, "ops is not set.  Do not bind.\n");
		return 0;
	}
	ret = ops->pinmux_group_set(dev, selector, 1);
	if(ret) {
		printf("pinmux_group_set error\n");
		return 0;
	}
	
	mdelay(20);
	
	ret = gpio_lookup_name(VERCTL_0, NULL, NULL, &gpio[0]);
	if (ret) {
		printf("GPIO: '%s' not found\n", VERCTL_0);
		return 0;
	}
	
	ret = gpio_lookup_name(VERCTL_1, NULL, NULL, &gpio[1]);
	if (ret) {
		printf("GPIO: '%s' not found\n", VERCTL_1);
		return 0;
	}

	ret = gpio_lookup_name(VERCTL_2, NULL, NULL, &gpio[2]);
	if (ret) {
		printf("GPIO: '%s' not found\n", VERCTL_2);
		return 0;
	}

	gpio_free(gpio[0]);
	gpio_free(gpio[1]);
	gpio_free(gpio[2]);
	
	gpio_request(gpio[0], "verctl_0");
	gpio_request(gpio[1], "verctl_1");
	gpio_request(gpio[2], "verctl_2");
	
	gpio_direction_input(gpio[0]);
	gpio_direction_input(gpio[1]);
	gpio_direction_input(gpio[2]);
	
	mdelay(2);
	
	value += 1 * gpio_get_value(gpio[0]);
	value += 2 * gpio_get_value(gpio[1]);
	value += 4 * gpio_get_value(gpio[2]);

	switch(value) {
		case 0:
			gd->board_type = NM01;
			break;
		case 1:
			gd->board_type = NM02;
			break;
		case 2:
			gd->board_type = NM03;
			break;
		case 3:
			gd->board_type = NM04;
			break;
		case 5:
			gd->board_type = NM14;
			break;
		case 7:
			gd->board_type = NM05;
			break;
		case 6:
			gd->board_type = NM06;
			break;	
		default:
			gd->board_type = NMXX;
			break;
	}
	
	return 0;
}

static int gpio_reset_switch(void)
{
	unsigned int gpio;
	int ret;
	
	if (gd->board_type == NM01 || gd->board_type == NM02 || gd->board_type == NM03 || gd->board_type == NM04 || gd->board_type == NM14) {
		ret = gpio_lookup_name(PHY0_RESET_GPIO, NULL, NULL, &gpio);
		if (ret) {
			printf("GPIO: '%s' not found\n", PHY0_RESET_GPIO);
			return 0;
		}
		gpio_free(gpio);
		gpio_request(gpio, "phy0_rst");
		gpio_direction_output(gpio, 0);
		mdelay(20);
		gpio_set_value(gpio, 1);
	}

	if (gd->board_type == NM02) {
			ret = gpio_lookup_name(PHY1_RESET_GPIO, NULL, NULL, &gpio);
			if (ret) {
				printf("GPIO: '%s' not found\n", PHY1_RESET_GPIO);
				return 0;
			}
			gpio_free(gpio);
			gpio_request(gpio, "phy1_rst");
			gpio_direction_output(gpio, 0);
			mdelay(20);
			gpio_set_value(gpio, 1);
			
			/* POE RESET */
			ret = gpio_lookup_name(POE_RESET, NULL, NULL, &gpio);
			if (ret)
				printf("GPIO: '%s' not found\n", POE_RESET);
			gpio_free(gpio);
			gpio_request(gpio, "poe_rst");
			gpio_direction_output(gpio, 0);
			mdelay(20);
			gpio_set_value(gpio, 1);
			
			/* sfp tx enable */
			ret = gpio_lookup_name(I2C_IO_EXP_NUM4, NULL, NULL, &gpio);
			if (ret)
				printf("GPIO: '%s' not found\n", I2C_IO_EXP_NUM4);
			gpio_free(gpio);
			gpio_request(gpio, "sfp_tx0_enable");
			gpio_direction_output(gpio, 1);
			mdelay(10);
			gpio_set_value(gpio, 0);

			ret = gpio_lookup_name(I2C_IO_EXP_NUM5, NULL, NULL, &gpio);
			if (ret)
				printf("GPIO: '%s' not found\n", I2C_IO_EXP_NUM5);
			gpio_free(gpio);
			gpio_request(gpio, "sfp_tx1_enable");
			gpio_direction_output(gpio, 1);
			mdelay(10);
			gpio_set_value(gpio, 0);
	}else if (gd->board_type == NM03) {
			ret = gpio_lookup_name(BP1_CTL, NULL, NULL, &gpio);
			if (ret) {
				printf("GPIO: '%s' not found\n", BP1_CTL);
				return 0;
			}
			gpio_free(gpio);
			gpio_request(gpio, "bp1_ctl");
			gpio_direction_output(gpio, 1);

			ret = gpio_lookup_name(BP2_CTL, NULL, NULL, &gpio);
			if (ret) {
				printf("GPIO: '%s' not found\n", BP2_CTL);
				return 0;
			}
			gpio_free(gpio);
			gpio_request(gpio, "bp2_ctl");
			gpio_direction_output(gpio, 1);	

			ret = gpio_lookup_name(PW1_CTL, NULL, NULL, &gpio);
			if (ret) {
				printf("GPIO: '%s' not found\n", PW1_CTL);
				return 0;
			}
			gpio_free(gpio);
			gpio_request(gpio, "pw1_ctl");
			gpio_direction_input(gpio);

			ret = gpio_lookup_name(PW2_CTL, NULL, NULL, &gpio);
			if (ret) {
				printf("GPIO: '%s' not found\n", PW2_CTL);
				return 0;
			}
			gpio_free(gpio);
			gpio_request(gpio, "pw2_ctl");
			gpio_direction_input(gpio);
	}

	return 0;
}

static int init_sys_led(void)
{
	int ret;
	unsigned int gpio;
	
	if (gd->board_type == NM01 || gd->board_type == NM02 || gd->board_type == NM03) {
		ret = gpio_lookup_name(PWM1_LED, NULL, NULL, &gpio);
		if (ret) {
			printf("GPIO: '%s' not found\n", PWM1_LED);
			return 0;
		}
		gpio_free(gpio);
		gpio_request(gpio, "sys_led");
		gpio_direction_output(gpio, 0);
		
		if (gd->board_type == NM03) {
			ret = gpio_lookup_name(BP2_LED, NULL, NULL, &gpio);
			if (ret) {
				printf("GPIO: '%s' not found\n", BP2_LED);
				return 0;
			}
			gpio_free(gpio);
			gpio_request(gpio, "bp2_led");
			gpio_direction_output(gpio, 0);
			
			ret = gpio_lookup_name(BP1_LED, NULL, NULL, &gpio);
			if (ret) {
				printf("GPIO: '%s' not found\n", BP1_LED);
				return 0;
			}
			gpio_free(gpio);
			gpio_request(gpio, "bp1_led");
			gpio_direction_output(gpio, 0);

			ret = gpio_lookup_name(ALM_LED, NULL, NULL, &gpio);
			if (ret) {
				printf("GPIO: '%s' not found\n", ALM_LED);
				return 0;
			}
			gpio_free(gpio);
			gpio_request(gpio, "alm_led");
			gpio_direction_output(gpio, 1);
		}
	} else if (gd->board_type == NM04 || gd->board_type == NM14) {
		ret = gpio_lookup_name(PWM0_LED, NULL, NULL, &gpio);
		if (ret) {
			printf("GPIO: '%s' not found\n", PWM0_LED);
			return 0;
		}
		gpio_free(gpio);
		gpio_request(gpio, "lte_ctl");
		gpio_direction_output(gpio, 0);

		ret = gpio_lookup_name(PWM1_LED, NULL, NULL, &gpio);
		if (ret) {
			printf("GPIO: '%s' not found\n", PWM1_LED);
			return 0;
		}
		gpio_free(gpio);
		gpio_request(gpio, "alarm_led");
		gpio_direction_output(gpio, 1);

		ret = gpio_lookup_name(PWM2_LED, NULL, NULL, &gpio);
		if (ret) {
			printf("GPIO: '%s' not found\n", PWM2_LED);
			return 0;
		}
		gpio_free(gpio);
		gpio_request(gpio, "wifi_led");
		gpio_direction_output(gpio, 0);

		ret = gpio_lookup_name(PWM3_LED, NULL, NULL, &gpio);
		if (ret) {
			printf("GPIO: '%s' not found\n", PWM3_LED);
			return 0;
		}
		gpio_free(gpio);
		gpio_request(gpio, "lte_led");
		gpio_direction_output(gpio, 1);
	}

	return 0;
}

static int init_other(void)
{
	int ret;
	unsigned int gpio;

	if (gd->board_type == NM04 || gd->board_type == NM14)
	{
		ret = gpio_lookup_name(LTE_POWER_CTL, NULL, NULL, &gpio);
		if (ret) {
			printf("GPIO: '%s' not found\n", LTE_POWER_CTL);
			return 0;
		}
		gpio_free(gpio);
		gpio_request(gpio, "lte_power_ctl");
		gpio_direction_output(gpio, 1);
		mdelay(100);
		ret = gpio_lookup_name(LTE_RESET, NULL, NULL, &gpio);
		if (ret) {
			printf("GPIO: '%s' not found\n", LTE_RESET);
			return 0;
		}
		gpio_free(gpio);
		gpio_request(gpio, "lte_reset_gpio");
		gpio_direction_output(gpio, 0);
		mdelay(200);
		gpio_set_value(gpio, 1);

		ret = gpio_lookup_name(USB_POWER_CTL, NULL, NULL, &gpio);
		if (ret) {
			printf("GPIO: '%s' not found\n", USB_POWER_CTL);
			return 0;
		}
		gpio_free(gpio);
		gpio_request(gpio, "usb_power_ctl");
		gpio_direction_output(gpio, 1);
	}
	
	if (gd->board_type != NM03) {
		/* init HSC32EU hardware reset gpio status as output and default value is 1 */
		ret = gpio_lookup_name(HSC_RESET, NULL, NULL, &gpio);
		if (ret) {
			printf("GPIO: '%s' not found\n", HSC_RESET);
			return 0;
		}
		gpio_free(gpio);
		gpio_request(gpio, "hsc_reset");
		gpio_direction_output(gpio, 0);
		mdelay(20);
		gpio_set_value(gpio, 1);
	}
	return 0;
}

static int log_out(int enable)
{
#ifdef CONFIG_SILENT_CONSOLE
	if (enable)
		gd->flags &= ~GD_FLG_SILENT;
	else
		gd->flags |= GD_FLG_SILENT;
#endif
	return 0;
}

int nmxx_board_init(void)
{
	get_board_type();
	gpio_reset_switch();
	init_sys_led();
	init_other();

	return 0;
}

#define check_all_time 	4000
#define check_one_time 	200
#define CMD_LEN			256

int recovery_ops(void)
{
	int count = 0, flag = 0, ret, i;
	unsigned int gpio, gpio12, gpio14;
	char cmd[CMD_LEN] = {0};
	char *s;
	long value = 0;
	ulong all_ram_size = 0;

	#if defined(CONFIG_NMXX_ALI)
	ret = gpio_lookup_name(KEY_RESET, NULL, NULL, &gpio);
	if (ret) {
		printf("GPIO: '%s' not found\n", KEY_RESET);
		return 0;
	}
	gpio_free(gpio);
	gpio_request(gpio, "reset_key");
	gpio_direction_input(gpio);
	
	while (1) {
		if (gpio_get_value(gpio))
			break;
		else {
			mdelay(check_one_time);
			count++;
		}
		if (count > (check_all_time / check_one_time)) {
			flag = 1;
			break;
		}	
	}
	if (1 == flag) {
		memset(cmd, 0, CMD_LEN);
		sprintf(cmd, "mmc dev 1");
		run_command_list(cmd, -1, 0);

		memset(cmd, 0, CMD_LEN);
		sprintf(cmd, "setenv loadaddr 0x7000000");
		run_command_list(cmd, -1, 0);
		
		memset(cmd, 0, CMD_LEN);
		sprintf(cmd, "part start mmc 1 3 pstart1");
		run_command_list(cmd, -1, 0);

		memset(cmd, 0, CMD_LEN);
		sprintf(cmd, "part size mmc 1 3 psize1");
		run_command_list(cmd, -1, 0);

		memset(cmd, 0, CMD_LEN);
		sprintf(cmd, "part start mmc 1 1 pstart2");
		run_command_list(cmd, -1, 0);

		memset(cmd, 0, CMD_LEN);
		sprintf(cmd, "part size mmc 1 1 psize2");
		run_command_list(cmd, -1, 0);

		s = getenv("pstart1");
		if (NULL == s) {
			printf("Can't getting information for partition 3\n");
			goto error;
		}
		s = getenv("pstart2");
		if (NULL == s) {
			printf("Can't getting information for partition 1\n");
			goto error;
		}
		
		for (i = 0; i < CONFIG_NR_DRAM_BANKS; ++i) {
			if (gd->bd->bi_dram[i].size)
				all_ram_size += gd->bd->bi_dram[i].size;
		}
		if (all_ram_size == 0)
			all_ram_size = 0x20000000;
		value = simple_strtol(getenv("psize1"), NULL, 16);
		if (value > (all_ram_size - simple_strtol(getenv("loadaddr"), NULL, 16) - 0x100000)){
			printf("The recovery partition is too big\n");
			goto error;	
		}
		
		memset(cmd, 0, CMD_LEN);
		sprintf(cmd, "fstype mmc 1:3 type_fs");
		run_command_list(cmd, -1, 0);
		s = getenv("type_fs");
		if ((NULL == s) || (strcmp(s, "ext4"))) {
			printf("Recovery filesystem type isn't ext4\n");
			goto error;
		}

		printf("\n======> Start recovering system partition......\n");
		
		ret = gpio_lookup_name(PWM1_LED, NULL, NULL, &gpio12);
		if (ret) {
			printf("GPIO: '%s' not found\n", PWM1_LED);
			return 0;
		}
		gpio_direction_output(gpio12, 0);
		ret = gpio_lookup_name(PWM3_LED, NULL, NULL, &gpio14);
		if (ret) {
			printf("GPIO: '%s' not found\n", PWM3_LED);
			return 0;
		}
		gpio_direction_output(gpio14, 0);
		log_out(0);
		memset(cmd, 0, CMD_LEN);
		sprintf(cmd, "mmc read ${loadaddr} ${pstart1} ${psize1}");
		run_command_list(cmd, -1, 0);

		memset(cmd, 0, CMD_LEN);
		sprintf(cmd, "mmc erase ${pstart2} ${psize2}");
		run_command_list(cmd, -1, 0);

		memset(cmd, 0, CMD_LEN);
		sprintf(cmd, "mmc write ${loadaddr} ${pstart2} ${psize1}");
		run_command_list(cmd, -1, 0);

		gpio_direction_output(gpio12, 1);
		gpio_direction_output(gpio14, 1);
		log_out(1);
		printf("\n======> End recovering system partition,result OK\n\n");
	}
	#endif

	return 0;
error:
	log_out(1);
	return 0;
}

int board_init(void)
{
	/*board_usb2_vbus_init();

	board_usb3_vbus_init();*/

	/* adress of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
#ifdef CONFIG_OF_CONTROL
	printf("U-Boot DT blob at : %p\n", gd->fdt_blob);
#endif

	/* init nmxx board configuration */
	//nmxx_board_init();

	/* enable serdes lane 2 mux for sata phy */
	//board_comphy_usb3_sata_mux(COMPHY_LANE2_MUX_SATA);

	return 0;
}

/* Board specific AHCI / SATA enable code */
int board_ahci_enable(struct udevice *dev)
{
#if defined(CONFIG_DM_REGULATOR)
	int ret;
	struct udevice *regulator;

	ret = device_get_supply_regulator(dev, "power-supply",
					  &regulator);
	if (ret) {
		debug("%s: No sata power supply\n", dev->name);
		return 0;
	}

	ret = regulator_set_enable(regulator, true);
	if (ret) {
		error("Error enabling sata power supply\n");
		return ret;
	}
#endif
	return 0;
}
