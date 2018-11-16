/*
* ***************************************************************************
* Copyright (C) 2016 Marvell International Ltd.
* ***************************************************************************
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
* ***************************************************************************
*/

#include "mv88e6xxx.h"
#include <nmxx_common.h>
/* If the switch's ADDR[4:0] strap pins are strapped to zero, it will
 * use all 32 SMI bus addresses on its SMI bus, and all switch registers
 * will be directly accessible on some {device address,register address}
 * pair.  If the ADDR[4:0] pins are not strapped to zero, the switch
 * will only respond to SMI transactions to that specific address, and
 * an indirect addressing mechanism needs to be used to access its
 * registers.
 */

static struct mv88e6xxx_dev soho_dev[2];
static struct mv88e6xxx_dev *soho_dev_handle;
static int REG_PORT_BASE = REG_PORT_BASE_LEGACY;

DECLARE_GLOBAL_DATA_PTR;

static int mv88e6xxx_reg_wait_ready(struct mv88e6xxx_dev *dev)
{
	int ret;
	int i;
	int loop_timeout = 16;
	unsigned short val;
	const char *name = miiphy_get_current_dev();

	if (!name)
		return -ENXIO;

	for (i = 0; i < loop_timeout; i++) {
		ret = miiphy_read(name, dev->phy_addr, SMI_CMD, &val);
		if (ret < 0)
			return ret;
		if ((val & SMI_CMD_BUSY) == 0)
			return 0;
	}

	return -ETIMEDOUT;
}

int mv88e6xxx_read_register(struct mv88e6xxx_dev *dev, int port, int reg)
{
	int ret;
	unsigned short val;
	const char *name = miiphy_get_current_dev();

	if (!name)
		return -ENXIO;

	if (!dev)
		return -ENODEV;

	if (dev->addr_mode == 0) {
		ret = miiphy_read(name, port, reg, &val);
		if (ret < 0)
			return ret;
		else
			return (int)val;
	}

	/* Wait for the bus to become free. */
	ret = mv88e6xxx_reg_wait_ready(dev);
	if (ret < 0)
		return ret;

	/* Transmit the read command. */
	ret = miiphy_write(name, dev->phy_addr, SMI_CMD,
		SMI_CMD_OP_22_READ | ((port & SMI_CMD_DEV_ADDR_MASK) <<
		SMI_CMD_DEV_ADDR_SIZE) | (reg & SMI_CMD_REG_ADDR_MASK));
	if (ret < 0)
		return ret;

	/* Wait for the read command to complete. */
	ret = mv88e6xxx_reg_wait_ready(dev);
	if (ret < 0)
		return ret;

	/* Read the data. */
	ret = miiphy_read(name, dev->phy_addr, SMI_DATA, &val);
	if (ret < 0)
		return ret;

	return (int)val;
}

int mv88e6xxx_write_register(struct mv88e6xxx_dev *dev, int port, int reg,
		unsigned short val)
{
	int ret;
	const char *name = miiphy_get_current_dev();

	if (!name)
		return -ENXIO;

	if (!dev)
		return -ENODEV;

	if (dev->addr_mode == 0)
		return miiphy_write(name, port, reg, val);

	/* Wait for the bus to become free. */
	ret = mv88e6xxx_reg_wait_ready(dev);
	if (ret < 0)
		return ret;

	/* Transmit data to write. */
	ret = miiphy_write(name, dev->phy_addr,
			SMI_DATA, val);
	if (ret < 0)
		return ret;

	/* Transmit the write command. */
	ret = miiphy_write(name, dev->phy_addr, SMI_CMD,
		SMI_CMD_OP_22_WRITE | ((port & SMI_CMD_DEV_ADDR_MASK) <<
		SMI_CMD_DEV_ADDR_SIZE) | (reg & SMI_CMD_REG_ADDR_MASK));
	if (ret < 0)
		return ret;

	/* Wait for the read command to complete. */
	ret = mv88e6xxx_reg_wait_ready(dev);
	if (ret < 0)
		return ret;

	return 0;
}

static int mv88e6xxx_reg_wait_ready_indirect(struct mv88e6xxx_dev *dev)
{
	int ret, i;
	int loop_timeout = 16;

	for (i = 0; i < loop_timeout; i++) {
		ret = mv88e6xxx_read_register(dev, REG_GLOBAL2, GLOBAL2_SMI_OP);
		if (ret < 0)
			return ret;
		if (!(ret & GLOBAL2_SMI_OP_BUSY))
			return 0;
	}
	return -ETIMEDOUT;
}

static int mv88e6xxx_read_indirect(struct mv88e6xxx_dev *dev, int port, int reg)
{
	int ret;

	ret = mv88e6xxx_write_register(dev, REG_GLOBAL2, GLOBAL2_SMI_OP,
		GLOBAL2_SMI_OP_22_READ | ((port & SMI_CMD_DEV_ADDR_MASK) <<
		SMI_CMD_DEV_ADDR_SIZE) | (reg & SMI_CMD_REG_ADDR_MASK));
	if (ret < 0)
		return ret;

	ret = mv88e6xxx_reg_wait_ready_indirect(dev);
	if (ret < 0)
		return ret;

	ret = mv88e6xxx_read_register(dev, REG_GLOBAL2, GLOBAL2_SMI_DATA);

	return ret;
}

static int mv88e6xxx_write_indirect(struct mv88e6xxx_dev *dev, int port,
			int reg, unsigned short val)
{
	int ret;

	ret = mv88e6xxx_write_register(dev, REG_GLOBAL2, GLOBAL2_SMI_DATA, val);
	if (ret < 0)
		return ret;

	ret = mv88e6xxx_write_register(dev, REG_GLOBAL2, GLOBAL2_SMI_OP,
		GLOBAL2_SMI_OP_22_WRITE | ((port & SMI_CMD_DEV_ADDR_MASK) <<
		SMI_CMD_DEV_ADDR_SIZE) | (reg & SMI_CMD_REG_ADDR_MASK));

	return mv88e6xxx_reg_wait_ready_indirect(dev);
}

int mv88e6xxx_read_phy_register(struct mv88e6xxx_dev *dev, int port, int page,
			int reg)
{
	int ret;

	if (!dev)
		return -ENODEV;

	ret = mv88e6xxx_write_indirect(dev, port, SMI_PHY_PAGE_REG, page);
	if (ret < 0)
		goto restore_page_0;

	ret = mv88e6xxx_read_indirect(dev, port, reg);

restore_page_0:
	mv88e6xxx_write_indirect(dev, port, SMI_PHY_PAGE_REG, 0x0);

	return ret;
}

int mv88e6xxx_write_phy_register(struct mv88e6xxx_dev *dev, int port, int page,
			int reg, unsigned short val)
{
	int ret;

	if (!dev) {
		printf("Soho dev not initialized\n");
		return -1;
	}

	ret = mv88e6xxx_write_indirect(dev, port, SMI_PHY_PAGE_REG, page);
	if (ret < 0)
		goto restore_page_0;

	ret = mv88e6xxx_write_indirect(dev, port, reg, val);

restore_page_0:
	mv88e6xxx_write_indirect(dev, port, SMI_PHY_PAGE_REG, 0x0);

	return ret;
}

void mv88e6xxx_display_switch_info(struct mv88e6xxx_dev *dev)
{
	unsigned int product_num;

	if (dev->id < 0) {
		printf("No Switch Device Found\n");
		return;
	}

	product_num = ((unsigned int)dev->id)>>4;
	if (product_num == PORT_SWITCH_ID_PROD_NUM_6390 ||
	    product_num == PORT_SWITCH_ID_PROD_NUM_6290 ||
	    product_num == PORT_SWITCH_ID_PROD_NUM_6190) {
		printf("Switch    : SOHO\n");
		printf("Series    : Peridot\n");
		printf("Product # : %X\n", product_num);
		printf("Revision  : %X\n", dev->id & 0xf);
		if (dev->cpu_port != -1)
			printf("Cpu port  : %d\n", dev->cpu_port);
	} else if (product_num == PORT_SWITCH_ID_PROD_NUM_6141 ||
		   	   product_num == PORT_SWITCH_ID_PROD_NUM_6341 ||
			   product_num == PORT_SWITCH_ID_PROD_NUM_6172 ||
			   product_num == PORT_SWITCH_ID_PROD_NUM_6176) {
		printf("Switch    : SOHO\n");
		printf("Series    : Topaz\n");
		printf("Product # : %X\n", product_num);
		printf("Revision  : %X\n", dev->id & 0xf);
		if (dev->cpu_port != -1)
			printf("Cpu port  : %d\n", dev->cpu_port);
	} else {
		printf("Unknown switch with Device ID: 0x%X\n", dev->id);
	}

	return;
}

/* We expect the switch to perform auto negotiation if there is a real phy. */
int mv88e6xxx_get_link_status(struct mv88e6xxx_dev *dev, int port)
{
	int ret;

	ret = mv88e6xxx_read_register(dev, REG_PORT(port), PORT_STATUS);
	if (ret < 0)
		return ret;

	printf("Port: 0x%X, ", port);
	if (ret & PORT_STATUS_LINK) {
		printf("Link: UP, ");
	} else {
		printf("Link: Down\n");
		return 0;
	}

	if (ret & PORT_STATUS_DUPLEX)
		printf("Duplex: FULL, ");
	else
		printf("Duplex: HALF, ");

	if ((ret & PORT_STATUS_SPEED_MASK) == PORT_STATUS_SPEED_10)
		printf("Speed: 10 Mbps\n");
	else if ((ret & PORT_STATUS_SPEED_MASK) == PORT_STATUS_SPEED_100)
		printf("Speed: 100 Mbps\n");
	else if ((ret & PORT_STATUS_SPEED_MASK) == PORT_STATUS_SPEED_1000)
		printf("Speed: 1000 Mbps\n");
	else
		printf("Speed: Unknown\n");

	return 0;
}

int mv88e6xxx_get_switch_id(struct mv88e6xxx_dev *dev)
{
	int id, product_num;

	id = mv88e6xxx_read_register(dev, REG_PORT(0), PORT_SWITCH_ID);
	if (id < 0)
		return id;

	product_num = id >> 4;
	if (product_num == PORT_SWITCH_ID_PROD_NUM_6190 ||
	    product_num == PORT_SWITCH_ID_PROD_NUM_6290 ||
	    product_num == PORT_SWITCH_ID_PROD_NUM_6390) {
		/* Peridot switch port device address starts from 0 */
		REG_PORT_BASE = REG_PORT_BASE_PERIDOT;
		return id;
	} else if (product_num == PORT_SWITCH_ID_PROD_NUM_6141 ||
		   	   product_num == PORT_SWITCH_ID_PROD_NUM_6341 ||
		   	   product_num == PORT_SWITCH_ID_PROD_NUM_6172 ||
			   product_num == PORT_SWITCH_ID_PROD_NUM_6176) {
		/* Legacy switch port device address starts from 0x10 */
		return id;
	} else {
		return -ENODEV;
	}

	return 0;
}

int mv88e6xxx_initialize(const void *blob)
{
	int i, node;
	int ret, val;
	int port, mask_port, switch_num = 0;
	u16 reg;
	char *s;

	soho_dev_handle = NULL;

	/* Read Device Tree */
	node = fdt_node_offset_by_compatible(blob, -1, "marvell,mv88e6xxx");

	if (node == -FDT_ERR_NOTFOUND)
		return -ENXIO;

	/* Check whether switch node is enabled */
	ret = fdtdec_get_is_enabled(blob, node);
	if (ret == 0)
		return -EACCES;

	if ((gd->board_type == NM01) || (gd->board_type == NM04))
		switch_num = 1;
	else if (gd->board_type == NM02)
		switch_num = 2;
	else
		return -ENXIO;
	
	for (i = 0; i < switch_num; i++) {
		/* Initizalize Switch Device Structure */
		if (0 == i) 
			soho_dev[i].phy_addr = fdtdec_get_uint(blob, node, "phy-addr", 0);
		else
			soho_dev[i].phy_addr = 0xA;
		
		soho_dev[i].port_mask = fdtdec_get_int(blob, node, "port-mask", 0);
		if (gd->board_type == NM02)
			soho_dev[i].sfp_port = fdtdec_get_int(blob, node, "sfp-port", -1);
		else
			soho_dev[i].sfp_port = -1;
		
		if (soho_dev[i].phy_addr == 0)
			soho_dev[i].addr_mode = 0;  /* Single Addressing mode */
		else
			soho_dev[i].addr_mode = 1;  /* Multi Addressing mode */

		soho_dev[i].id = mv88e6xxx_get_switch_id(&soho_dev[i]);

		soho_dev[i].cpu_port = fdtdec_get_int(blob, node, "cpu-port", -1);
		if (soho_dev[i].cpu_port != -1) {
			reg = mv88e6xxx_read_register(&soho_dev[i],
						      REG_PORT(soho_dev[i].cpu_port),
						      PORT_PCS_CTRL);
			/* CPU port is forced link-up, duplex and 1GB speed */
			reg &= ~PORT_PCS_CTRL_UNFORCED;
			reg |= PORT_PCS_CTRL_FORCE_LINK |
			       PORT_PCS_CTRL_LINK_UP |
			       PORT_PCS_CTRL_DUPLEX_FULL |
			       PORT_PCS_CTRL_FORCE_DUPLEX |
			       PORT_PCS_CTRL_1000;
			if ((soho_dev[i].id >> 4) == PORT_SWITCH_ID_PROD_NUM_6341) {
				/* Configure RGMII Delay on cpu port */
				reg |= PORT_PCS_CTRL_FORCE_SPEED |
				       PORT_PCS_CTRL_RGMII_DELAY_TXCLK |
				       PORT_PCS_CTRL_RGMII_DELAY_RXCLK;
			}else if(((soho_dev[i].id >> 4) == PORT_SWITCH_ID_PROD_NUM_6172) ||
					 ((soho_dev[i].id >> 4) == PORT_SWITCH_ID_PROD_NUM_6176)) {
				/* Configure RGMII Delay on cpu port */
				reg |= PORT_PCS_CTRL_1000 |
					   PORT_PCS_CTRL_RGMII_DELAY_TXCLK |
					   PORT_PCS_CTRL_RGMII_DELAY_RXCLK;
			}
			ret = mv88e6xxx_write_register(&soho_dev[i],
						       REG_PORT(soho_dev[i].cpu_port),
						       PORT_PCS_CTRL, reg);
			if (ret)
				return ret;
		}

		/* Force port setup */
		for (port = 0; port < sizeof(soho_dev[i].port_mask) * 8; port++) {
			if (!(soho_dev[i].port_mask & BIT(port)))
				continue;

			/* Set port control register */
			mv88e6xxx_write_register(&soho_dev[i],
						 REG_PORT(port),
						 PORT_CONTROL,
						 PORT_CONTROL_STATE_FORWARDING |
						 PORT_CONTROL_FORWARD_UNKNOWN |
						 PORT_CONTROL_FORWARD_UNKNOWN_MC |
						 PORT_CONTROL_USE_TAG |
						 PORT_CONTROL_USE_IP |
						 PORT_CONTROL_TAG_IF_BOTH);
			/* Set port based vlan table */
			mv88e6xxx_write_register(&soho_dev[i],
						 REG_PORT(port),
						 PORT_BASE_VLAN,
						 soho_dev[i].port_mask & ~BIT(port));

			if (port == soho_dev[i].cpu_port || port == soho_dev[i].sfp_port)
				continue;

			/* Set phy copper control for lan ports */
			mv88e6xxx_write_phy_register(&soho_dev[i],
						     port,
						     0,
						     PHY_COPPER_CONTROL,
						     PHY_COPPER_CONTROL_SPEED_1G |
						     PHY_COPPER_CONTROL_DUPLEX |
						     PHY_COPPER_CONTROL_AUTO_NEG_EN);

			/* set port 4 as sfp for nm02 */
			if (gd->board_type == NM02) {
				if (soho_dev[i].sfp_port != -1) {
					reg = mv88e6xxx_read_register(&soho_dev[i],
						      REG_PORT(soho_dev[i].sfp_port),
						      PORT_PCS_CTRL);
				
					reg = mv88e6xxx_read_phy_register(&soho_dev[i],
								      0xf, 1, 0);
					reg &= ~ BIT(11);
					ret = mv88e6xxx_write_phy_register(&soho_dev[i],
								      0xf, 1, 0, reg);			   
					if (ret)
						return ret;
				}
			}
		}

		s = getenv("eth_mask");
		if(s)
			mask_port = (int)simple_strtol(s, NULL, 16);
		else 
			mask_port = 0x3FE;
		
		mask_port >>= i * 5;
		/* Turn on phy power for mv6176 */
		if(((soho_dev[i].id >> 4) == PORT_SWITCH_ID_PROD_NUM_6172) || ((soho_dev[i].id >> 4) == PORT_SWITCH_ID_PROD_NUM_6176)) {
			for (port = 0; port < 5; port++) {
				if (!(mask_port & (0x1 << port)))
					continue;
				if (port == soho_dev[i].sfp_port){
					reg = mv88e6xxx_read_phy_register(&soho_dev[i], 0xf, 1, 0);
					reg |= BIT(11);
					ret = mv88e6xxx_write_phy_register(&soho_dev[i], 0xf, 1, 0, reg);			   
					if (ret){
						printf("Failed: Write - switch port: 0x%X, page: 0xf, reg: 0x0, val: 0x%X, ret: %d\n", port, reg, ret);
						break;
					}
				}else{
					ret = mv88e6xxx_read_phy_register(&soho_dev[i], port, 0, 0);
					if (ret < 0){
						printf("Failed: Read - switch port: 0x%X, page: 0x0, reg: 0x0, ret: %d\n", port, ret);
						break;
					}
					ret |= PHY_COPPER_CONTROL_POWER_DOWN;
					val = ret;
					ret = mv88e6xxx_write_phy_register(&soho_dev[i], port, 0, 0,(unsigned short)val);
					if (ret < 0){
						printf("Failed: Write - switch port: 0x%X, page: 0x0, reg: 0x0, val: 0x%X, ret: %d\n", port, val, ret);
						break;
					}
				}
			}
		}
	}

	soho_dev_handle = &soho_dev[0];

	return 0;
}

struct mv88e6xxx_dev *mv88e6xxx_get_current_dev(int id)
{
	soho_dev_handle = &soho_dev[id];

	return soho_dev_handle;
}

static int sw_resolve_options(char *str)
{
	if (strcmp(str, "info") == 0)
		return SW_INFO;
	else if (strcmp(str, "read") == 0)
		return SW_READ;
	else if (strcmp(str, "write") == 0)
		return SW_WRITE;
	else if (strcmp(str, "phy_read") == 0)
		return SW_PHY_READ;
	else if (strcmp(str, "phy_write") == 0)
		return SW_PHY_WRITE;
	else if (strcmp(str, "dev") == 0)
		return SW_DEV;
	else if (strcmp(str, "link") == 0)
		return SW_LINK;
	else if (strcmp(str, "force") == 0)
		return SW_FORCE;
	else
		return SW_NA;
}

static int do_sw(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct mv88e6xxx_dev *dev;
	int port, reg, page, val = 0, ret = 0, id;

	dev = soho_dev_handle;

	if (!dev) {
		printf("Switch Device not found\n");
		return -ENODEV;
	}

	switch (sw_resolve_options(argv[1])) {
	case SW_DEV:
		if (argc != 3) {
			printf("Syntax Error: switch dev <id>\n");
			return 1;
		}
		id = (int)simple_strtoul(argv[2], NULL, 10);
		if ((id == 0) || (id == 1))
			dev = mv88e6xxx_get_current_dev(id);
		else
			printf("Param Error: id = 0 or 1\n");
		break;
	case SW_FORCE:
		if (argc != 2) {
			printf("Syntax Error: switch force\n");
			return 1;
		}
		

	case SW_INFO:
		mv88e6xxx_display_switch_info(dev);
		break;
	case SW_READ:
		if (argc < 4) {
			printf("Syntax Error: switch read <port> <reg>\n");
			return 1;
		}
		port = (int)simple_strtoul(argv[2], NULL, 16);
		reg  = (int)simple_strtoul(argv[3], NULL, 16);
		ret = mv88e6xxx_read_register(dev, REG_PORT(port), reg);
		if (ret < 0)
			printf("Failed: Read  - switch port: 0x%X, reg: 0x%X, ret: %d\n",
			       port, reg, ret);
		else
			printf("Read - switch port: 0x%X, reg: 0x%X, val: 0x%X\n",
			       port, reg, ret);
		break;

	case SW_WRITE:
		if (argc < 5) {
			printf("Syntax Error: switch write <port> <reg> <val>\n");
			return 1;
		}
		port = (int)simple_strtoul(argv[2], NULL, 16);
		reg  = (int)simple_strtoul(argv[3], NULL, 16);
		val  = (int)simple_strtoul(argv[4], NULL, 16);
		ret = mv88e6xxx_write_register(dev, REG_PORT(port), reg,
					       (unsigned short)val);
		if (ret < 0)
			printf("Failed: Write - switch port: 0x%X, reg: 0x%X, val: 0x%X, ret: %d\n",
			       port, reg, val, ret);
		else
			printf("Read  - switch port: 0x%X, reg: 0x%X, val: 0x%X\n",
			       port, reg, val);
		break;

	case SW_PHY_READ:
		if (argc < 5) {
			printf("Syntax Error: switch phy_read <port> <page> <reg>\n");
			return 1;
		}
		port = (int)simple_strtoul(argv[2], NULL, 16);
		page = (int)simple_strtoul(argv[3], NULL, 16);
		reg  = (int)simple_strtoul(argv[4], NULL, 16);
		ret = mv88e6xxx_read_phy_register(dev, port,
						  page, reg);
		if (ret < 0)
			printf("Failed: Read - switch port: 0x%X, page: 0x%X, reg: 0x%X\n, ret: %d",
			       port, page, reg, ret);
		else
			printf("Read - switch port: 0x%X, page: 0x%X, reg: 0x%X, val: 0x%X\n",
			       port, page, reg, ret);
		break;

	case SW_PHY_WRITE:
		if (argc < 6) {
			printf("Syntax Error: switch phy_write <port> <page> <reg> <val>\n");
			return 1;
		}
		port = (int)simple_strtoul(argv[2], NULL, 16);
		page = (int)simple_strtoul(argv[3], NULL, 16);
		reg  = (int)simple_strtoul(argv[4], NULL, 16);
		val  = (int)simple_strtoul(argv[5], NULL, 16);
		ret = mv88e6xxx_write_phy_register(dev, port,
						   page, reg,
						   (unsigned short)val);
		if (ret < 0)
			printf("Failed: Write - switch port: 0x%X, page: 0x%X, reg: 0x%X, val: 0x%X, ret: %d\n",
			       port, page, reg, val, ret);
		else
			printf("Read - switch port: 0x%X, page: 0x%X, reg: 0x%X, val: 0x%X\n",
			       port, page, reg, val);
		break;

	case SW_LINK:
		if (argc < 3) {
			printf("Error: Too few arguments\n");
			return 1;
		}
		port = (int)simple_strtoul(argv[2], NULL, 16);
		ret = mv88e6xxx_get_link_status(dev, port);
		break;

	case SW_NA:
		printf("\"switch %s\" - Wrong command. Try \"help switch\"\n",
		       argv[1]);

	default:
		break;
	}
	return 0;
}

/***************************************************/
U_BOOT_CMD(
	switch,	7,	1,	do_sw,
	"Switch Access commands",
	"dev <id> - switch switch dev\n"
	"force - enable all phy and sfp\n"
	"switch info - Display switch information\n"
	"switch read <port> <reg> - read switch register <reg> of a <port>\n"
	"switch write <port> <reg> <val> - write <val> to switch register <reg> of a <port>\n"
	"switch phy_read <port> <page> <reg> - read internal switch phy register <reg> at <page> of a switch <port>\n"
	"switch phy_write <port> <page> <reg> <val>- write <val> to internal phy register at <page> of a <port>\n"
	"switch link <port> - Display link state and speed of a <port>\n"
);
