/* include/linux/leds/rt8547_fled.h
 * Header of Richtek RT8547 Flash LED Driver
 *
 * Copyright (C) 2013 Richtek Technology Corp.
 * Author: Patrick Chang <patrick_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef LINUX_LEDS_RT8547_FLED_H
#define LINUX_LEDS_RT8547_FLED_H
#include <linux/kernel.h>
#include <linux/leds/rtfled.h>
#define RT8547_DRV_VER "1.2.2G"

/* 100 mA to 1600 mA */
#define RT8547_STROBE_CURRENT(mA) (((mA - 100) / 50) & 0x1f)
/* 100 mA to 400mA */
#define RT8547_TIMEOUT_LEVEL(mA) (((mA - 100) / 50) & 0x07)
/* 64 ms to 1216 ms */
#define RT8547_STROBE_TIMEOUT(ms) (((ms - 64) / 32) & 0x3f)
/* 25 mA to 400 mA */
#define RT8547_TORCH_CURRENT(mA) (((mA - 25) / 25) & 0x0f)
/* 3000mV to 3800mV */
#define RT8547_LV_PROTECTION(mV) (((mV - 3000) / 100) & 0x0f)


typedef struct rt8547_platform_data {
	int flen_gpio;
	int ctl_gpio;
	int flset_gpio;
	unsigned int fled_strobe_current;
	unsigned int fled_timeout_current_level;
	unsigned int fled_torch_current;
	unsigned int fled_strobe_timeout;
	unsigned int fled_lv_protection;
} rt8547_fled_platform_data_t;

typedef enum rt8547_torch_current{
	TORCH_CURRENT_25MA,
	TORCH_CURRENT_50MA,
	TORCH_CURRENT_75MA,
	TORCH_CURRENT_100MA,
	TORCH_CURRENT_125MA,
	TORCH_CURRENT_150MA,
	TORCH_CURRENT_175MA,
	TORCH_CURRENT_200MA,
	TORCH_CURRENT_225MA,
	TORCH_CURRENT_250MA,
	TORCH_CURRENT_275MA,
	TORCH_CURRENT_300MA,
	TORCH_CURRENT_325MA,
	TORCH_CURRENT_350MA,
	TORCH_CURRENT_375MA,
	TORCH_CURRENT_400MA,
	TORCH_CURRENT_MAX,
};

typedef enum rt8547_strobe_current{
	STROBE_CURRENT_100MA,
	STROBE_CURRENT_150MA,
	STROBE_CURRENT_200MA,
	STROBE_CURRENT_250MA,
	STROBE_CURRENT_300MA,
	STROBE_CURRENT_350MA,
	STROBE_CURRENT_400MA,
	STROBE_CURRENT_450MA,
	STROBE_CURRENT_500MA,
	STROBE_CURRENT_550MA,
	STROBE_CURRENT_600MA,
	STROBE_CURRENT_650MA,
	STROBE_CURRENT_700MA,
	STROBE_CURRENT_750MA,
	STROBE_CURRENT_800MA,
	STROBE_CURRENT_850MA,
	STROBE_CURRENT_900MA,
	STROBE_CURRENT_1000MA,
	STROBE_CURRENT_1050MA,
	STROBE_CURRENT_1100MA,
	STROBE_CURRENT_1150MA,
	STROBE_CURRENT_1200MA,
	STROBE_CURRENT_1250MA,
	STROBE_CURRENT_1300MA,
	STROBE_CURRENT_1350MA,
	STROBE_CURRENT_1400MA,
	STROBE_CURRENT_1450MA,
	STROBE_CURRENT_1500MA,
	STROBE_CURRENT_1550MA,
	STROBE_CURRENT_1600MA,
	STROBE_CURRENT_MAX,
};

#define RT8547_FLED_SW_RESET_REG    0x03
#define RT8547_FLED_SW_RESET_MASK   (1<<5)

#define RT8547_FLED_MODE_MASK	    (1<<4)

enum {
	RT8547_FLED_DUMMY = 0x00,
	RT8547_FLED_CONTROL1 = 0x01,
	RT8547_FLED_CONTROL2,
	RT8547_FLED_CONTROL3,
	RT8547_FLED_CONTROL4,
	RT8547_FLED_HIDDEN_CTRL7 = 0x07,
	RT8547_FLED_REG_NR,
};


int rt8547_reg_read_byte(rt_fled_info_t *fled_info, int addr);
int rt8547_reg_write_byte(rt_fled_info_t *fled_info, int addr, uint8_t data);
int rt8547_assign_bits(rt_fled_info_t *fled_info, int addr,
                       uint8_t mask, uint8_t data);
int rt8547_clr_bits(rt_fled_info_t *fled_info, int addr, uint8_t mask);
int rt8547_set_bits(rt_fled_info_t *fled_info, int addr, uint8_t mask);
ssize_t rt8547_control(flashlight_mode_t mode,int level);

#endif /*LINUX_LEDS_RT8547_FLED_H*/
