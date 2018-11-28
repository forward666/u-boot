/*
 * Copyright (C) 2016 Stefan Roese <sr@denx.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _CONFIG_MVEBU_ARMADA_H
#define _CONFIG_MVEBU_ARMADA_H

#include <asm/arch/soc.h>

/*
 * High Level Configuration Options (easy to change)
 */

#define	CONFIG_SYS_TEXT_BASE	0x00000000

/* additions for new ARM relocation support */
#define CONFIG_SYS_SDRAM_BASE	0x00000000


/* auto boot */
#define CONFIG_PREBOOT
#if defined (CONFIG_NMXX_DP)
#define CONFIG_BAUDRATE			115200
#else
#define CONFIG_BAUDRATE			115200
#endif
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, \
					  115200, 230400, 460800, 921600 }

/* Default Env vars */
#define CONFIG_IPADDR			192.168.1.10	/* In order to cause an error */
#define CONFIG_SERVERIP			192.168.1.100	/* In order to cause an error */
#define CONFIG_NETMASK			255.255.255.0
#define CONFIG_GATEWAYIP		10.4.50.254
#define CONFIG_HAS_ETH1
#define CONFIG_HAS_ETH2
#define CONFIG_ETHPRIME			"eth0"
/* #define CONFIG_ROOTPATH                 "/srv/nfs/"*/ /* Default Dir for NFS */

#define CONFIG_ENV_OVERWRITE	/* ethaddr can be reprogrammed */
/*
 * For booting Linux, the board info and command line data
 * have to be in the first 8 MB of memory, since this is
 * the maximum mapped by the Linux kernel during initialization.
 */
#define CONFIG_CMDLINE_TAG		/* enable passing of ATAGs  */
#define CONFIG_INITRD_TAG		/* enable INITRD tag */
#define CONFIG_SETUP_MEMORY_TAGS	/* enable memory tag */

#define	CONFIG_SYS_CBSIZE	1024	/* Console I/O Buff Size */
#define	CONFIG_SYS_PBSIZE	(CONFIG_SYS_CBSIZE \
		+sizeof(CONFIG_SYS_PROMPT) + 16)	/* Print Buff */

/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN	(8 << 20) /* 8MiB for malloc() */

/*
 * Other required minimal configurations
 */
#define CONFIG_ARCH_CPU_INIT		/* call arch_cpu_init() */
#define CONFIG_SYS_LOAD_ADDR	0x00800000	/* default load adr- 8M */
#define CONFIG_SYS_MEMTEST_START 0x00800000	/* 8M */
#define CONFIG_SYS_MEMTEST_END	0x00ffffff	/*(_16M -1) */
#define CONFIG_SYS_RESET_ADDRESS 0xffff0000	/* Rst Vector Adr */
#define CONFIG_SYS_MAXARGS	32	/* max number of command args */

#define CONFIG_SYS_ALT_MEMTEST

#ifndef CONFIG_ENV_IS_IN_BOOTDEV
	#if defined(CONFIG_MVEBU_SPI_BOOT)
		#define CONFIG_ENV_IS_IN_SPI_FLASH
	#elif defined(CONFIG_MVEBU_NAND_BOOT)
		#define CONFIG_ENV_IS_IN_NAND
	#elif defined(CONFIG_MVEBU_MMC_BOOT)
		#define CONFIG_ENV_IS_IN_MMC
		#define CONFIG_SYS_MMC_ENV_PART	1
		/* 0 - DATA, 1 - BOOT0, 2 - BOOT1 */
	#elif defined(CONFIG_MVEBU_SPINAND_BOOT)
		#define CONFIG_ENV_IS_IN_SPI_NAND
	#endif
#else
	/* Need to define all these xxx_ENV_IS_IN_xxx for
	* including the appropriate env_xxx.o modules in the
	* final image, so the environment location can be
	* selected dynamically at the boot time.
	*/
	#ifdef CONFIG_SPI_FLASH
		#define CONFIG_ENV_IS_IN_SPI_FLASH
	#endif
	#define DEFAULT_BUBT_DST	"spi"
	#ifdef CONFIG_NAND_PXA3XX
		#define CONFIG_ENV_IS_IN_NAND
	#endif
#endif /* CONIFG_ENV_IS_IN_BOOTDEV */

/* End of 16M scrubbed by training in bootrom */
#define CONFIG_SYS_INIT_SP_ADDR         (CONFIG_SYS_TEXT_BASE + 0xFF0000)

/*
 * SPI Flash configuration
 */
#define CONFIG_ENV_SPI_BUS		0
#define CONFIG_ENV_SPI_CS		0

/* SPI NOR flash default params, used by sf commands */
#define CONFIG_SF_DEFAULT_SPEED		1000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE

/* Assume minimum flash/eMMC boot partition size of 4MB
 * and save the environment at the end of the boot device
*/
#define CONFIG_ENV_SIZE			(64 << 10) /* 64KiB */
#define CONFIG_ENV_SECT_SIZE		(64 << 10) /* 64KiB sectors */

#ifndef CONFIG_ENV_IS_IN_BOOTDEV
#if defined(CONFIG_ENV_IS_IN_NAND) || defined(CONFIG_ENV_IS_IN_SPI_NAND)
#define CONFIG_ENV_OFFSET		0x400000
#else
#define CONFIG_ENV_OFFSET		(0x200000 - CONFIG_ENV_SIZE)
#endif
#elif !defined(__ASSEMBLY__)
int boot_from_nand(void);
#define CONFIG_ENV_OFFSET	(boot_from_nand() ? \
		0x400000 : (0x400000 - CONFIG_ENV_SIZE))
#else
#define CONFIG_ENV_OFFSET		0x400000
#endif /* CONFIG_ENV_IS_IN_BOOTDEV */

#define CONFIG_USB_MAX_CONTROLLER_COUNT (CONFIG_SYS_USB_EHCI_MAX_ROOT_PORTS + \
					 CONFIG_SYS_USB_XHCI_MAX_ROOT_PORTS)

#ifdef CONFIG_USB
/* USB ethernet */
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_ETHER_MCS7830
#define CONFIG_USB_ETHER_RTL8152
#define CONFIG_USB_ETHER_SMSC95XX
#endif

/*
 * SATA/SCSI/AHCI configuration
 */
#ifdef CONFIG_MVEBU_SCSI
#define CONFIG_SCSI
#define CONFIG_SCSI_AHCI
#define CONFIG_SCSI_AHCI_PLAT
#define CONFIG_LIBATA
#define CONFIG_LBA48
#define CONFIG_SYS_64BIT_LBA

#define CONFIG_SYS_SCSI_MAX_SCSI_ID	2
#define CONFIG_SYS_SCSI_MAX_LUN		1
#define CONFIG_SYS_SCSI_MAX_DEVICE	(CONFIG_SYS_SCSI_MAX_SCSI_ID * \
					 CONFIG_SYS_SCSI_MAX_LUN)
#endif

/* MMC/SD IP block */
#define CONFIG_SUPPORT_VFAT

/*
 * The EEPROM ST M24C64 has 32 byte page write mode and takes up to 10 msec.
 */
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS 10

/* Emulation specific setting */
/************************************  PALLDIUM  ******************************/
#ifdef CONFIG_MVEBU_PALLADIUM
#define CONFIG_ENV_IS_NOWHERE

/* Overcome issue in emulation where writes
 * to address 0x0 might sometimes fail.
 */
#undef CONFIG_SYS_TEXT_BASE
#define	CONFIG_SYS_TEXT_BASE		0x00001000

/* Size of malloc() pool */
#undef CONFIG_SYS_MALLOC_LEN
#define CONFIG_SYS_MALLOC_LEN		(1 << 20) /* 1MiB for malloc() */

#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY		-1

#undef CONFIG_BAUDRATE
#define CONFIG_BAUDRATE			19200

#undef CONFIG_EXTRA_ENV_SETTINGS
#define CONFIG_EXTRA_ENV_SETTINGS	"kernel_addr=0x180000\0"	\
					"fdt_addr=0x1000000\0"		\
					"ramfs_addr=0x5000000\0"	\
					"fdt_high=0xa0000000\0"		\
					"initrd_high=0xffffffffffffffff\0"

#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND		"booti $kernel_addr " \
					"$ramfs_addr $fdt_addr"

/************************************  PALLDIUM  ******************************/
#else /*CONFIG_MVEBU_PALLADIUM*/

#include <config_distro_defaults.h>
#if 0
#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 1)

#include <config_distro_bootcmd.h>
#endif
#if defined (CONFIG_NMXX_SXF)
#define CONFIG_EXTRA_ENV_SETTINGS		\
	"boot_prefixes=/boot/\0" \
	"fdt_name=armada-3720-community.dtb\0"			\
	"image_name=openwrt-armada-ESPRESSObin-Image\0"			\
	"bootdev_normal=root=/dev/mmcblk0p2 rw rootwait\0"			\
	"bootdev_recovery=root=/dev/mmcblk0p1 rw rootwait\0"			\
	"boot_mode=normal\0"		\
	"recovery_name=recovery.bin\0"				\
	"rootfs_name=rootfs.bin\0"				\
	"update_opt=tftpboot $loadaddr ${image_name_t}; if test $? = 0; then " \
		"mmc dev 1; part start mmc 1 ${partition} pstart; part size mmc 1 ${partition} psize; mmc erase ${pstart} ${psize}; " \
		"mmc write ${loadaddr} ${pstart} ${filesize}; if test $? != 0; then echo Update failed....;  " \
		"else echo Update ${image_name_t} success^_^; fi; " \
		"fi\0" \
	"update_recovery=setenv partition 1; setenv image_name_t ${recovery_name}; run update_opt;\0" \
	"update_rootfs=setenv partition 2; setenv image_name_t ${rootfs_name}; run update_opt; \0" \
	"bootmenu_0=Boot system code via rootfs partition(default)=run bootcmd\0"		\
	"bootmenu_1=Boot system code via recovery partition=run bootcmd_recovery\0"		\
	"bootmenu_2=Load system code then write to rootfs partition via TFTP=run update_rootfs\0"		\
	"bootmenu_3=Load system code then write to recovery partition via TFTP=run update_recovery\0"		\
	"bootmenu_4=Show local ip and serverip=echo local ip(${ipaddr}) serverip(${serverip})\0"		\
	"bootcmd_recovery=setenv boot_mode recovery; run bootcmd\0"		\
	"get_images_mmc=mmc dev 1; ext4load mmc 1:1 $kernel_addr ${boot_prefixes}$image_name; "		\
		"ext4load mmc 1:1 $fdt_addr ${boot_prefixes}$fdt_name;\0"	\
	"get_bootdev=if test \"${boot_mode}\" = \"normal\"; then setenv bootdev ${bootdev_normal}; "			\
		"elif test \"${boot_mode}\" = \"recovery\"; then setenv bootdev ${bootdev_recovery}; "			\
		"else setenv bootdev  ; "	\
		"fi\0"		\
	"get_images_tftp=tftpboot $kernel_addr $image_name; "		\
		"tftpboot $fdt_addr $fdt_name;\0"	\
	"get_images_usb=usb reset;fatload usb 0 $kernel_addr $image_name; "		\
		"fatload usb 0 $fdt_addr $fdt_name;\0"	\
	"set_bootargs=run get_bootdev;setenv bootargs $console $bootdev\0"	\
	"bootcmd_tftp=run get_images_tftp; run set_bootargs; "		\
		"booti $kernel_addr - $fdt_addr\0"	\
	"bootcmd_mmc=run get_images_mmc; run set_bootargs; "		\
		"booti $kernel_addr - $fdt_addr\0"	\
	"bootcmd_usb=run get_images_usb; run set_bootargs; "		\
		"booti $kernel_addr - $fdt_addr\0"	\
	"kernel_addr=0x7000000\0"		\
	"initrd_addr=0xa00000\0"		\
	"initrd_size=0x2000000\0"		\
	"fdt_addr=0x6f00000\0"		\
	"loadaddr=0x7000000\0"			\
	"fdt_high=0xffffffffffffffff\0"		\
	"netdev=eth0\0"				\
	"ethaddr=00:51:82:11:22:00\0"		\
	"console=" CONFIG_DEFAULT_CONSOLE "\0"
#elif defined (CONFIG_NMXX_ALI)
#define CONFIG_EXTRA_ENV_SETTINGS		\
	"boot_prefixes=/boot/\0" \
	"fdt_name=fdt.dtb\0"			\
	"image_name=Image\0"			\
	"bootdev_normal=root=/dev/mmcblk1p1 rw rootwait\0"			\
	"bootdev_recovery=root=/dev/mmcblk1p3 rw rootwait\0"			\
	"boot_mode=normal\0"		\
	"partition=1\0"		\
	"get_images_mmc=mmc dev 1; ext4load mmc 1:${partition} $kernel_addr ${boot_prefixes}$image_name; "		\
		"ext4load mmc 1:${partition} $fdt_addr ${boot_prefixes}$fdt_name;\0"	\
	"get_bootdev=if test \"${boot_mode}\" = \"normal\"; then setenv bootdev ${bootdev_normal}; setenv partition 1; "			\
		"elif test \"${boot_mode}\" = \"recovery\"; then setenv bootdev ${bootdev_recovery}; setenv partition 3;"			\
		"else setenv bootdev  ; setenv partition 1; "	\
		"fi\0"		\
	"get_images_tftp=tftpboot $kernel_addr $image_name; "		\
		"tftpboot $fdt_addr $fdt_name;\0"	\
	"get_images_usb=usb reset;fatload usb 0 $kernel_addr $image_name; "		\
		"fatload usb 0 $fdt_addr $fdt_name;\0"	\
	"set_bootargs=run get_bootdev;setenv bootargs $console $bootdev\0"	\
	"bootcmd_tftp=run get_images_tftp; run set_bootargs; "		\
		"booti $kernel_addr - $fdt_addr\0"	\
	"bootcmd_mmc=run get_bootdev; run get_images_mmc; run set_bootargs; "		\
		"booti $kernel_addr - $fdt_addr\0"	\
	"bootcmd_usb=run get_images_usb; run set_bootargs; "		\
		"booti $kernel_addr - $fdt_addr\0"	\
	"kernel_addr=0x7000000\0"		\
	"initrd_addr=0xa00000\0"		\
	"initrd_size=0x2000000\0"		\
	"fdt_addr=0x6f00000\0"		\
	"loadaddr=0x7000000\0"			\
	"fdt_high=0xffffffffffffffff\0"		\
	"netdev=eth0\0"				\
	"ethaddr=00:51:82:11:22:00\0"		\
	"console=" CONFIG_DEFAULT_CONSOLE "\0"
#elif defined (CONFIG_NMXX_DP)
#define CONFIG_EXTRA_ENV_SETTINGS		\
	"boot_prefixes=/\0" \
	"fdt_name=armada-3720-nmxx.dtb\0"			\
	"image_name=Image\0"			\
	"boot_mode=-\0"		\
	"get_images_mmc=mmc dev 1; fatload mmc 1:1 $kernel_addr ${boot_prefixes}$image_name; " 	\
		"fatload mmc 1:1 $fdt_addr ${boot_prefixes}$fdt_name;\0"	\
	"get_bootdev=if test \"${boot_mode}\" = \"normal\"; then setenv bootdev ${bootdev_normal}; "			\
		"elif test \"${boot_mode}\" = \"recovery\"; then setenv bootdev ${bootdev_recovery}; "			\
		"else setenv bootdev  ; "	\
		"fi\0"		\
	"get_images_tftp=tftpboot $kernel_addr $image_name; "		\
		"tftpboot $fdt_addr $fdt_name;\0"	\
	"get_images_usb=usb reset;fatload usb 0 $kernel_addr $image_name; "		\
		"fatload usb 0 $fdt_addr $fdt_name;\0"	\
	"set_bootargs=run get_bootdev;setenv bootargs $console $bootdev\0"	\
	"bootcmd_tftp=run get_images_tftp; run set_bootargs; "		\
		"booti $kernel_addr - $fdt_addr\0"	\
	"bootcmd_mmc=run get_images_mmc; run set_bootargs; "		\
		"booti $kernel_addr - $fdt_addr\0"	\
	"bootcmd_usb=run get_images_usb; run set_bootargs; "		\
		"booti $kernel_addr - $fdt_addr\0"	\
	"kernel_addr=0x7000000\0"		\
	"initrd_addr=0xa00000\0"		\
	"initrd_size=0x2000000\0"		\
	"fdt_addr=0x6f00000\0"		\
	"loadaddr=0x7000000\0"			\
	"fdt_high=0xffffffffffffffff\0"		\
	"netdev=eth0\0"				\
	"ethaddr=00:51:82:11:22:00\0"		\
	"console=" CONFIG_DEFAULT_CONSOLE "\0"
#endif
#endif /*CONFIG_MVEBU_PALLADIUM*/
#define CONFIG_BOOTCOMMAND      "run bootcmd_mmc"

#endif /* _CONFIG_MVEBU_ARMADA_H */
