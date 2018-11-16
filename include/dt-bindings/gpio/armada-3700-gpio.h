/*
 * This header provides constants for binding marvell,armada-3700-gpio.
 *
 * The first cell in armada-3700's GPIO specifier is the GPIO bank reference.
 *
 * The second cell in armada-3700's GPIO specifier is the global GPIO ID. The macros below
 * provide names for this.
 *
 * The third cell contains standard flag values specified in gpio.h.
 */

#ifndef _DT_BINDINGS_GPIO_ARMADA_3700_GPIO_H
#define _DT_BINDINGS_GPIO_ARMADA_3700_GPIO_H

#include <dt-bindings/gpio/gpio.h>

#define MMC_RESET           	"GPIO110"
#define WLED           			"GPIO111"
#define SYS_LED        			"GPIO112"
#define LTE_LED        			"GPIO113"
#define PWM3_LED       			"GPIO114"
#define LTE_RESET       		"GPIO117"
#define KEY_RESET 	   			"GPIO120"
#define VERCTL_0      			"GPIO121"
#define VERCTL_1       			"GPIO122"
#define VERCTL_2       			"GPIO123"

#define LTE_POWER_CTL       	"GPIO20"
#define POE_RESET       		"GPIO21"
#define USB_POWER_CTL	   		"GPIO24"
#define HSC_RESET	   			"GPIO25"
#define LED4 	       			"GPIO221"
#define PHY0_RESET_GPIO 		"GPIO222"
#define PHY1_RESET_GPIO 		"GPIO223"

#define I2C_IO_EXP_NUM4			"gpio@21_4"
#define I2C_IO_EXP_NUM5			"gpio@21_5"

#endif
