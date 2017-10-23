/*
* * Copyright (C) 2012 Spreadtrum Communications Inc.
* *
* * This software is licensed under the terms of the GNU General Public
* * License version 2, as published by the Free Software Foundation, and
* * may be copied, distributed, and modified under those terms.
* *
* * This program is distributed in the hope that it will be useful,
* * but WITHOUT ANY WARRANTY; without even the implied warranty of
* * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* * GNU General Public License for more details.
* */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <soc/sprd/hardware.h>
#include <soc/sprd/board.h>
#include <soc/sprd/adi.h>
#include <linux/leds.h>
#include <linux/leds/rt8547_fled.h>

int sprd_flash_on(void)
{
	rt8547_control(FLASHLIGHT_MODE_TORCH,TORCH_CURRENT_200MA);
	return 0;
}

int sprd_flash_high_light(void)
{
	rt8547_control(FLASHLIGHT_MODE_FLASH,STROBE_CURRENT_800MA);
	return 0;
}

int sprd_flash_close(void)
{
	rt8547_control(FLASHLIGHT_MODE_OFF,0);
	return 0;
}

int sprd_flash_cfg(struct sprd_flash_cfg_param *param, void *arg)
{
	return 0;
}
