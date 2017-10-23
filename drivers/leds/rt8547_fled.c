/* drivers/leds/rt8547_fled.c
 * RT8547 Flash LED Driver
 *
 * Copyright (C) 2013 Richtek Technology Corp.
 * Author: Patrick Chang <patrick_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/leds/rtfled.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/rtdefs.h>
#include <linux/leds/rt8547_fled.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define ALIAS_NAME "rt8547-fled"

#define RT8547_ADDR 0x99
#define LONG_DELAY 	60
#define SHORT_DELAY 4
#define START_DELAY 10
#define STOP_DELAY	1500


#define RT8547_INFO(format, args...) \
    printk(KERN_INFO "%s:%s() line-%d: " format, \
            ALIAS_NAME, __FUNCTION__, __LINE__, ## args)
#define RT8547_WARN(format, args...) \
    printk(KERN_WARNING "%s:%s() line-%d: " format, \
            ALIAS_NAME, __FUNCTION__, __LINE__, ## args)
#define RT8547_ERR(format, args...) \
    printk(KERN_ERR "%s:%s() line-%d: " format, \
            ALIAS_NAME, __FUNCTION__, __LINE__, ## args)

typedef struct rt8547_fled_info {
	rt_fled_info_t base;
	const rt8547_fled_platform_data_t *pdata;
	int flen;
	int ctl;
	int flset;
	uint8_t regs[RT8547_FLED_REG_NR];
	spinlock_t io_lock;
} rt8547_fled_info_t;

static inline int rt8547_send_bit(rt8547_fled_info_t *info,
                                  unsigned char bit)
{
	if (bit > 0)
	{
		gpio_set_value(info->flset, 0);
		udelay(SHORT_DELAY);
		gpio_set_value(info->flset, 1);
		udelay(LONG_DELAY);
	}
	else
	{
		gpio_set_value(info->flset, 0);
		udelay(LONG_DELAY);
		gpio_set_value(info->flset, 1);
		udelay(SHORT_DELAY);
	}
	return 0;
}

static inline int rt8547_send_byte(rt8547_fled_info_t *info,
                                   unsigned char byte)
{
	int i;
	
	/*send order is high bit to low bit */
	for (i=7; i>=0; i--)
		rt8547_send_bit(info, byte&(0x1<<i));
	return 0;
}

static inline int rt8547_send_special_byte(rt8547_fled_info_t *info,
                                           unsigned char byte)
{
	int i;
	//only send three bit for register address
	for (i=2; i>=0; i--)
		rt8547_send_bit(info, byte&(0x1<<i));
	return 0;
}

static inline int rt8547_start_xfer(rt8547_fled_info_t *info)
{
	//gpio_set_value(info->flen,1);
	//udelay(START_DELAY);
	gpio_set_value(info->flset,1);
	udelay(START_DELAY);

	return 0;
}

static inline int rt8547_stop_xfer(rt8547_fled_info_t *info)
{
	//redundant 1 bit as the stop condition
	rt8547_send_bit(info, 1);
	return 0;
}

static int __rt8547_send_data(rt8547_fled_info_t* info, int addr, uint8_t data)
{
	unsigned long flags;
	unsigned char xfer_data[3]; /* 0: adddr, 1: reg, 2: reg data */
	
	xfer_data[0] = RT8547_ADDR;
	xfer_data[1] = (unsigned char)addr;
	xfer_data[2] = (unsigned char)data;
	RT8547_INFO("rt8547 send -> 0: 0x%02x, 1: 0x%02x, 2: 0x%02x\n",
        xfer_data[0], xfer_data[1], xfer_data[2]);
	spin_lock_irqsave(&info->io_lock, flags);
	rt8547_start_xfer(info);
	rt8547_send_byte(info, xfer_data[0]);
	rt8547_send_special_byte(info, xfer_data[1]);
	rt8547_send_byte(info, xfer_data[2]);
	rt8547_stop_xfer(info);
	spin_unlock_irqrestore(&info->io_lock, flags);
	return 0;
}

static void rt8547_flush_cache_mapping(rt8547_fled_info_t* info)
{
	int i;
	for (i = 1; i <= RT8547_FLED_CONTROL4; i++)
		__rt8547_send_data(info, i, info->regs[i]);
}

static void rt8547_reset(rt8547_fled_info_t *info)
{
	/* restore register cache mapping to default value */
	info->regs[RT8547_FLED_DUMMY] = 0;
	info->regs[RT8547_FLED_CONTROL1] = 0x06;
	info->regs[RT8547_FLED_CONTROL2] = 0x12;
	info->regs[RT8547_FLED_CONTROL3] = 0x02;
	info->regs[RT8547_FLED_CONTROL4] = 0x0f;
	info->regs[RT8547_FLED_HIDDEN_CTRL7] = 0x67;
	__rt8547_send_data(info, RT8547_FLED_CONTROL3, RT8547_FLED_SW_RESET_MASK);
}

int rt8547_reg_read_byte(rt_fled_info_t *fled_info, int addr)
{
	rt8547_fled_info_t *info = (rt8547_fled_info_t *)fled_info;
	return info->regs[addr];
}
EXPORT_SYMBOL(rt8547_reg_read_byte);

int rt8547_reg_write_byte(rt_fled_info_t *fled_info, int addr, uint8_t data)
{
	rt8547_fled_info_t *info = (rt8547_fled_info_t *)fled_info;
	info->regs[addr] = data;
	if (addr == RT8547_FLED_SW_RESET_REG
        && (RT8547_FLED_SW_RESET_MASK&data) != 0)
	{
		/*do reset*/
		rt8547_reset(info);
	} else
	__rt8547_send_data(info, addr, data);
	return 0;
}
EXPORT_SYMBOL(rt8547_reg_write_byte);


int rt8547_assign_bits(rt_fled_info_t *fled_info, int addr,
                       uint8_t mask, uint8_t data)
{
	rt8547_fled_info_t *info = (rt8547_fled_info_t *)fled_info;
	uint8_t temp = info->regs[addr];

	temp &= (~mask);
	temp |= (mask & data);
	return rt8547_reg_write_byte(fled_info, addr, temp);
}

EXPORT_SYMBOL(rt8547_assign_bits);

int rt8547_clr_bits(rt_fled_info_t *fled_info, int addr, uint8_t mask)
{
	return rt8547_assign_bits(fled_info, addr, mask, 0);
}
EXPORT_SYMBOL(rt8547_clr_bits);

int rt8547_set_bits(rt_fled_info_t *fled_info, int addr, uint8_t mask)
{
	return rt8547_assign_bits(fled_info, addr, mask, mask);
}
EXPORT_SYMBOL(rt8547_set_bits);

static inline int rt8547_control_en(rt8547_fled_info_t *info, int en)
{
	if (en > 0)
		gpio_set_value(info->ctl, 1);
	else
		gpio_set_value(info->ctl, 0);
	RT8547_INFO("Set Ctrl En = %d\n", en);
	return 0;
}

static inline int rt8547_flash_en(rt8547_fled_info_t *info, int en)
{
	if (en > 0)
		gpio_set_value(info->flen, 1);
	else
		gpio_set_value(info->flen, 0);
	RT8547_INFO("Set Flash En = %d\n", en);
	return 0;
}

static inline int rt8547_power_on(rt8547_fled_info_t *info)
{
	gpio_set_value(info->flset, 1);
	return 0;
}

static inline int rt8547_power_off(rt8547_fled_info_t *info)
{
	gpio_set_value(info->flset, 0);
	udelay(STOP_DELAY);
	return 0;
}

static void rt8547_strobe(rt8547_fled_info_t *info)
{
	/* turn off and then turn on for strobing*/
	rt8547_flash_en(info, 0);
	rt8547_control_en(info, 0);
	udelay(START_DELAY);
	rt8547_control_en(info, 1);
	rt8547_flash_en(info, 1);
}

static void rt8547_set_led_on(rt8547_fled_info_t *info)
{
	rt8547_control_en(info, 1);
	rt8547_flash_en(info, 1);
}

static void rt8547_set_led_off(rt8547_fled_info_t *info)
{
	rt8547_flash_en(info, 0);
	rt8547_control_en(info, 0);
}

static void rt8547_set_torch_mode(rt8547_fled_info_t *info, uint8_t torch)
{
	if (torch)
		rt8547_set_bits(&info->base, RT8547_FLED_CONTROL3, RT8547_FLED_MODE_MASK);
	else
		rt8547_clr_bits(&info->base, RT8547_FLED_CONTROL3, RT8547_FLED_MODE_MASK);
}

static int rt8547_fled_set_mode(struct rt_fled_info *fled_info, flashlight_mode_t mode)
{
	rt8547_fled_info_t *info = (rt8547_fled_info_t *)fled_info;

	switch (mode)
	{
		case FLASHLIGHT_MODE_OFF:
			rt8547_set_led_off(info);
			rt8547_set_torch_mode(info, 1);
		break;
		case FLASHLIGHT_MODE_TORCH:
		case FLASHLIGHT_MODE_MIXED:
			rt8547_set_torch_mode(info, 1);
			rt8547_set_led_on(info);
		break;
		case FLASHLIGHT_MODE_FLASH:
			rt8547_set_led_off(info);
			rt8547_set_torch_mode(info, 0);
			break;
		default:
		return -EINVAL;
	}
	info->base.flashlight_dev->props.mode = mode;
	return 0;
}

static int rt8547_fled_get_mode(struct rt_fled_info *info)
{
	rt8547_fled_info_t *rt8547_fled_info = (rt8547_fled_info_t *)info;
	return rt8547_fled_info->base.flashlight_dev->props.mode;
}

static int rt8547_fled_strobe(struct rt_fled_info *fled_info)
{
	rt8547_fled_info_t *info = (rt8547_fled_info_t *)fled_info;
	switch (info->base.flashlight_dev->props.mode)
	{
		case FLASHLIGHT_MODE_FLASH:
			rt8547_strobe(info);
		break;
		case FLASHLIGHT_MODE_MIXED:
			rt8547_flash_en(info, 0);
			rt8547_set_torch_mode(info, 0);
			rt8547_strobe(info);
		break;
		default:
			RT8547_ERR("Error : not flash / mixed mode\n");
		return -EINVAL;
	}
	return 0;
}

static struct platform_device rt_fled_pdev = {
	.name = "rt-flash-led",
	.id = -1,
};

static void rt8547_gpio_deinit(const rt8547_fled_platform_data_t *pdata)
{
	gpio_direction_input(pdata->flen_gpio);
	gpio_free(pdata->flen_gpio);
	if (pdata->flen_gpio != pdata->ctl_gpio) {
		gpio_direction_input(pdata->ctl_gpio);
		gpio_free(pdata->ctl_gpio);
	}
	gpio_direction_input(pdata->flset_gpio);
	gpio_free(pdata->flset_gpio);
}

static int rt8547_gpio_init(const rt8547_fled_platform_data_t *pdata)
{
	int ret;

	if (!gpio_is_valid(pdata->flen_gpio) ||
		!gpio_is_valid(pdata->ctl_gpio) ||
		!gpio_is_valid(pdata->flset_gpio)) {
		RT8547_ERR("not valid gpio for one of three\n");
		return -EINVAL;
	}
	else {
		ret = gpio_request_one(pdata->flen_gpio, GPIOF_OUT_INIT_LOW, "rt8547_flen");
		if (ret < 0) {
			RT8547_ERR("request flen gpio fail\n");
			goto request_gpio1_fail;
		}
		if (pdata->flen_gpio != pdata->ctl_gpio) {
			ret = gpio_request_one(pdata->ctl_gpio, GPIOF_OUT_INIT_LOW, "rt8547_ctl");
			if (ret < 0) {
				RT8547_ERR("request ctl gpio fail\n");
				goto request_gpio2_fail;
			}
		}

		ret = gpio_request_one(pdata->flset_gpio, GPIOF_OUT_INIT_LOW, "rt8547_flset");
		if (ret < 0) {
			RT8547_ERR("request flset gpio fail\n");
			goto request_gpio3_fail;
		}
	}

	return 0;

request_gpio3_fail:
	if (pdata->ctl_gpio != pdata->flen_gpio)
		gpio_free(pdata->ctl_gpio);
request_gpio2_fail:
	gpio_free(pdata->flen_gpio);
request_gpio1_fail:
	return ret;
}

static struct flashlight_properties rt8547_fled_props =
{
	.type = FLASHLIGHT_TYPE_LED,
	.torch_brightness = 2,
	.torch_max_brightness = 15,
	.strobe_brightness = 0x12,
	.strobe_max_brightness = 31 - 1,
	.strobe_delay = 2,
	.strobe_timeout = 544,
	.alias_name = "rt8547-fled",
};

static int rt8547_fled_init(struct rt_fled_info *fled_info)
{
	rt8547_fled_info_t *info = (rt8547_fled_info_t *)fled_info;
	BUG_ON(info == NULL);
	rt8547_power_on(info);
	rt8547_reset(info);
    rt8547_flush_cache_mapping(info);
	RT8547_INFO("RT8547 initialize registers\n");
	
	/* Disable LV protection */
	rt8547_clr_bits(fled_info, RT8547_FLED_HIDDEN_CTRL7, 0x40);
	
	fled_info->hal->fled_set_torch_current_sel(fled_info,
                                            info->pdata->fled_torch_current);
	fled_info->hal->fled_set_timeout_level_sel(fled_info,
                                            info->pdata->fled_timeout_current_level);
	fled_info->hal->fled_set_strobe_current_sel(fled_info,
                                            info->pdata->fled_strobe_current);
	fled_info->hal->fled_set_strobe_timeout_sel(fled_info,
                                            info->pdata->fled_strobe_timeout);
	fled_info->hal->fled_set_lv_protection_sel(fled_info,
                                            info->pdata->fled_lv_protection);
	return 0;
}

static int rt8547_fled_suspend(struct rt_fled_info *fled_info, pm_message_t state)
{
	rt8547_fled_info_t *info = (rt8547_fled_info_t *)fled_info;
	RT8547_INFO("Suspend\n");
	//rt8547_power_off(info);
	return 0;
}

static int rt8547_fled_resume(struct rt_fled_info *fled_info)
{
	rt8547_fled_info_t *info = (rt8547_fled_info_t *)fled_info;
	RT8547_INFO("Resume\n");
	//rt8547_power_on(info);
	return 0;
}

/* Return value : -EINVAL => selector parameter is out of range, otherwise current in mA*/
static int rt8547_fled_troch_current_list(struct rt_fled_info *info, int selector)
{
	if (selector < 0 || selector > 15)
		return -EINVAL;
	return (25 + selector*25)*1000;
}


static int rt8547_fled_strobe_current_list(struct rt_fled_info *info, int selector)
{
	if (selector < 0 || selector >= 31)
		return -EINVAL;
	return (100 + selector*50)*1000;
}

static int rt8547_fled_timeout_level_list(struct rt_fled_info *info, int selector)
{
	if (selector < 0 || selector > 6)
		return -EINVAL;
	return (100 + selector*50)*1000;
}

/* Return value : -EINVAL => selector parameter is out of range, otherwise voltage in mV*/
static int rt8547_fled_lv_protection_list(struct rt_fled_info *info, int selector)
{
	if (selector < 0 || selector > 8)
		return -EINVAL;
	return 3000 + selector*100;
}
/* Return value : -EINVAL => selector parameter is out of range, otherwise time in ms*/
static int rt8547_fled_strobe_timeout_list(struct rt_fled_info *info, int selector)
{
	if (selector < 0 || selector > 0x24)
		return -EINVAL;
	return 64 + selector*32;
}

static int rt8547_fled_set_torch_current_sel(struct rt_fled_info *fled_info,
                                              int selector)
{
	int rc;
	RT8547_INFO("Set torch current to %d\n", selector);
	if (selector < 0 || selector >  fled_info->
			flashlight_dev->props.torch_max_brightness)
		return -EINVAL;
	rc = rt8547_assign_bits(fled_info, RT8547_FLED_CONTROL3,
							0x0f, selector);
	if (rc == 0)
		fled_info->flashlight_dev->props.torch_brightness = selector;
	return rc;
}

static int rt8547_fled_set_strobe_current_sel(struct rt_fled_info *fled_info,
                                               int selector)
{
	int rc;
	RT8547_INFO("Set strobe current to %d\n", selector);
	if (selector < 0 || selector >  fled_info->
			flashlight_dev->props.strobe_max_brightness)
		return -EINVAL;
	rc = rt8547_assign_bits(fled_info, RT8547_FLED_CONTROL2,
                            0x1f, selector);
	if (rc == 0)
		fled_info->flashlight_dev->props.strobe_brightness = selector;
	return 0;
}

static int rt8547_fled_set_timeout_level_sel(struct rt_fled_info *fled_info,
                                               int selector)
{
	RT8547_INFO("Set timeout level to %d\n", selector);
	if (selector < 0 || selector > 6)
		return -EINVAL;
	return rt8547_assign_bits(fled_info, RT8547_FLED_CONTROL2,
                            0xe0, selector << 5);
}

static int rt8547_fled_set_lv_protection_sel(struct rt_fled_info *fled_info,
                                            int selector)
{
	RT8547_INFO("Set lv protection to %d\n", selector);
	if (selector < 0 || selector > 8)
		return -EINVAL;
	return rt8547_assign_bits(fled_info, RT8547_FLED_CONTROL1,
                            0x0f, selector);
}

static int rt8547_fled_set_strobe_timeout_sel(struct rt_fled_info *fled_info,
                                               int selector)
{
	int rc;
	RT8547_INFO("Set strobe timeout to %d\n", selector);
	if (selector < 0 || selector > 0x24)
		return -EINVAL;
	rc = rt8547_assign_bits(fled_info, RT8547_FLED_CONTROL4,
                            0x3f, selector);
	if (rc == 0)
		fled_info->flashlight_dev->props.strobe_timeout =
	rt8547_fled_strobe_timeout_list(fled_info, selector);
	return rc;
}

ssize_t rt8547_control(flashlight_mode_t mode,int level)
{
	int ret;
	struct platform_device *pdev = container_of(&(rt_fled_pdev.dev.parent),struct platform_device,dev);
	rt8547_fled_info_t *fled_info = platform_get_drvdata(pdev);
	printk(KERN_ALERT"rt8547_control  flash_mode:0x%x,level:0x%x line %d .\n",mode,level,__LINE__);
	switch (mode)
	{
		case FLASHLIGHT_MODE_OFF:
			rt8547_set_led_off(fled_info);
		break;
		case FLASHLIGHT_MODE_TORCH:
			rt8547_set_torch_mode(fled_info, 1);
			rt8547_fled_set_torch_current_sel(fled_info, level);
			rt8547_set_led_on(fled_info);
		break;
		case FLASHLIGHT_MODE_FLASH:
			rt8547_set_torch_mode(fled_info, 0);
			rt8547_fled_set_strobe_current_sel(fled_info, level);
			rt8547_set_led_on(fled_info);
		break;
	}
	return ret;
}
EXPORT_SYMBOL(rt8547_control);

static ssize_t show_help(struct device* dev,
		struct device_attribute* attr,  char* buf)
{
	char help_str[] = "\n"
		"using: echo 0xxx > flash\n"
		"The upper two bits(6,7bit) are mode bits\n"
		"and the rest is the current value\n"
		"--------------for example----------------\n"
		"4x: Torch mode(echo 0x40 > flash)\n"
		"8x: flash mode(echo 0x80 > flash)\n"
		"0x: off   mode(echo 0x00 > flash)\n";

	struct platform_device *pdev = container_of(dev,struct platform_device,dev);
		if (!pdev) {
		pr_err("flash device is null\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", help_str);
}

static ssize_t rt8547_fled_store(struct device *dev,
              struct device_attribute *attr, const char *buf,size_t c)
{
	int ret;
	struct platform_device *pdev = container_of(dev,struct platform_device,dev);
	rt8547_fled_info_t *fled_info = platform_get_drvdata(pdev);
	unsigned long flash_mode,level;
	ret = kstrtoul(buf, 16, &flash_mode);
	printk(KERN_ALERT"rt8547_fled_store  buf=%s,flash_mode:%x line %d .\n",buf,flash_mode,__LINE__);
	rt8547_control(flash_mode>>6,flash_mode&0x3f);
	return c;
}
static DEVICE_ATTR(flash, 0777, show_help, rt8547_fled_store);


static int rt8547_fled_get_torch_current_sel(struct rt_fled_info *fled_info)
{
	int rc;
	rc = rt8547_reg_read_byte(fled_info, RT8547_FLED_CONTROL3);
	return rc & 0x0f;
}

static int rt8547_fled_get_strobe_current_sel(struct rt_fled_info *fled_info)
{
	int rc;
	rc = rt8547_reg_read_byte(fled_info, RT8547_FLED_CONTROL2);
	return rc & 0x1f;
}

static int rt8547_fled_get_timeout_level_sel(struct rt_fled_info *fled_info)
{
	int rc;
	rc = rt8547_reg_read_byte(fled_info, RT8547_FLED_CONTROL2);
	return (rc & 0xe0) >> 5;
}

static int rt8547_fled_get_lv_protection_sel(struct rt_fled_info *fled_info)
{
	int rc;
	rc = rt8547_reg_read_byte(fled_info, RT8547_FLED_CONTROL1);
	return rc & 0x0f;
}

static int rt8547_fled_get_strobe_timeout_sel(struct rt_fled_info *fled_info)
{
	int rc;
	rc = rt8547_reg_read_byte(fled_info, RT8547_FLED_CONTROL4);
	return rc & 0x3f;
}

static void rt8547_fled_shutdown(struct rt_fled_info *fled_info)
{
	rt8547_fled_info_t *info = (rt8547_fled_info_t *)fled_info;
	flashlight_set_mode(fled_info->flashlight_dev, FLASHLIGHT_MODE_OFF);
	rt8547_power_off(info);
	return;
}

static struct rt_fled_hal rt8547_fled_hal = {
	.fled_init = rt8547_fled_init,
	.fled_suspend = rt8547_fled_suspend,
	.fled_resume = rt8547_fled_resume,
	.fled_set_mode = rt8547_fled_set_mode,
	.fled_get_mode = rt8547_fled_get_mode,
	.fled_strobe = rt8547_fled_strobe,
	.fled_troch_current_list = rt8547_fled_troch_current_list,
	.fled_strobe_current_list = rt8547_fled_strobe_current_list,
	.fled_timeout_level_list = rt8547_fled_timeout_level_list,
	.fled_lv_protection_list = rt8547_fled_lv_protection_list,
	.fled_strobe_timeout_list = rt8547_fled_strobe_timeout_list,
	/* method to set */
	.fled_set_torch_current_sel = rt8547_fled_set_torch_current_sel,
	.fled_set_strobe_current_sel = rt8547_fled_set_strobe_current_sel,
	.fled_set_timeout_level_sel = rt8547_fled_set_timeout_level_sel,
	.fled_set_lv_protection_sel = rt8547_fled_set_lv_protection_sel,
	.fled_set_strobe_timeout_sel = rt8547_fled_set_strobe_timeout_sel,
	/* method to get */
	.fled_get_torch_current_sel = rt8547_fled_get_torch_current_sel,
	.fled_get_strobe_current_sel = rt8547_fled_get_strobe_current_sel,
	.fled_get_timeout_level_sel = rt8547_fled_get_timeout_level_sel,
	.fled_get_lv_protection_sel = rt8547_fled_get_lv_protection_sel,
	.fled_get_strobe_timeout_sel = rt8547_fled_get_strobe_timeout_sel,
	/* PM shutdown, optional */
	.fled_shutdown = rt8547_fled_shutdown,
};

#ifdef CONFIG_OF
static const struct of_device_id fled_dt_ids[] = {
	{ .compatible = "Richtek,rt8547-fled", },
	{},
};
MODULE_DEVICE_TABLE(of, fled_dt_ids);
#endif

static int rt8547_parse_dt(struct device_node *np, struct device *dev, rt8547_fled_platform_data_t *pdata)
{
	int ret;

	if (!np)
		return -EINVAL;

	pdata->flen_gpio = of_get_named_gpio(np,"flen-gpio", 0);
	if (pdata->flen_gpio < 0) {
		RT8547_ERR("%s: of_get_named_gpio failed: %d\n", __func__,pdata->flen_gpio);
		return -EINVAL;
	}

	pdata->ctl_gpio = of_get_named_gpio(np,"ctl-gpio", 0);
	if (pdata->ctl_gpio < 0) {
		RT8547_ERR("%s: of_get_named_gpio failed: %d\n", __func__,pdata->ctl_gpio);
		return -EINVAL;
	}

	pdata->flset_gpio = of_get_named_gpio(np,"flset-gpio", 0);
	if (pdata->flset_gpio < 0) {
		RT8547_ERR("%s: of_get_named_gpio failed: %d\n", __func__,pdata->flset_gpio);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "fled,strobe_current",&pdata->fled_strobe_current);
	pdata->fled_strobe_current = RT8547_STROBE_CURRENT(pdata->fled_strobe_current);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(np, "fled,timeout_current_level",&pdata->fled_timeout_current_level);
	pdata->fled_timeout_current_level = RT8547_TIMEOUT_LEVEL(pdata->fled_timeout_current_level);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(np, "fled,torch_current",&pdata->fled_torch_current);
	pdata->fled_torch_current = RT8547_TORCH_CURRENT(pdata->fled_torch_current);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(np, "fled,strobe_timeout",&pdata->fled_strobe_timeout);
	pdata->fled_strobe_timeout = RT8547_STROBE_TIMEOUT(pdata->fled_strobe_timeout);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(np, "fled,lv_protection",&pdata->fled_lv_protection);
	pdata->fled_lv_protection = RT8547_LV_PROTECTION(pdata->fled_lv_protection);
	printk("ethanhu add %d,%d,%d,%d,%d,%d,%d",pdata->flen_gpio,pdata->flset_gpio,pdata->fled_strobe_current,pdata->fled_timeout_current_level,pdata->fled_torch_current,
		pdata->fled_strobe_timeout,pdata->fled_lv_protection);
	if (ret < 0)
		return ret;

	return 0;
}

static int rt8547_fled_probe(struct platform_device *pdev)
{
	int ret;
	rt8547_fled_platform_data_t *pdata = pdev->dev.platform_data;
	rt8547_fled_info_t *fled_info;
	RT8547_INFO("Richtek RT8547 FlashLED driver probing...\n");
	/*BUG_ON(pdata == NULL);*/

	struct device_node *np = pdev->dev.of_node;

	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata) {
			pdata = devm_kzalloc(&pdev->dev,sizeof(*pdata), GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
		}
		ret = rt8547_parse_dt(np, &pdev->dev, pdata);
		if (ret)
		goto err_no_platform_data;
	}

	fled_info = kzalloc(sizeof(*fled_info), GFP_KERNEL);
	if (!fled_info) {
		ret = -ENOMEM;
		goto err_fled_nomem;
	}

	spin_lock_init(&fled_info->io_lock);
	fled_info->pdata = pdata;
	fled_info->flen = pdata->flen_gpio;
	fled_info->ctl = pdata->ctl_gpio;
	fled_info->flset = pdata->flset_gpio;
	fled_info->base.init_props = &rt8547_fled_props;
	fled_info->base.hal = &rt8547_fled_hal;
	ret = rt8547_gpio_init(pdata);
	if (ret < 0) {
		RT8547_ERR("Error : can't init GPIO (%d)\n", ret);
		goto err_init_gpio;
	}
	platform_set_drvdata(pdev, fled_info);
	rt_fled_pdev.dev.parent = &(pdev->dev);
	ret = device_create_file(&(pdev->dev), &dev_attr_flash);
	if (ret < 0) {
		printk(KERN_ALERT"Failed to create attribute val.");
		goto err_register_pdev;
	}

	ret = platform_device_register(&rt_fled_pdev);
	if (ret < 0)
		goto err_register_pdev;

	return 0;
err_register_pdev:
	rt8547_gpio_deinit(pdata);
err_init_gpio:
	kfree(fled_info);
err_no_platform_data:
	if (IS_ENABLED(CONFIG_OF))
		devm_kfree(&pdev->dev, (void *)pdata);
err_fled_nomem:
	return ret;
}

static int rt8547_fled_remove(struct platform_device *pdev)
{
	rt8547_fled_info_t *fled_info;
	RT8547_INFO("Richtek RT8547 FlashLED driver removing...\n");
	fled_info = platform_get_drvdata(pdev);
	platform_device_unregister(&rt_fled_pdev);
	rt8547_gpio_deinit(fled_info->pdata);
	kfree(fled_info);
	return 0;
}

#ifdef CONFIG_PM

static int __rt8547_fled_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rt8547_fled_info_t *info = dev_get_drvdata(&pdev->dev);
	BUG_ON(info == NULL);
	return 0;
}

static int __rt8547_fled_resume(struct platform_device *pdev)
{
	struct rt8547_fled_info_t *info = dev_get_drvdata(&pdev->dev);
	BUG_ON(info == NULL);
	return 0;
}
#else /* CONFIG_PM */
#define __rt8547_fled_suspend NULL
#define __rt8547_fled_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver rt8547_fled_driver = {
	.probe = rt8547_fled_probe,
	.remove = rt8547_fled_remove,
	.suspend = __rt8547_fled_suspend,
	.resume = __rt8547_fled_resume,
	.driver = {
		.name = "rt8547-fled",
		.owner = THIS_MODULE,
		.of_match_table = fled_dt_ids,
	},
};


static int __init rt8547_fled_module_init(void)
{
	return platform_driver_register(&rt8547_fled_driver);
}

static void __exit rt8547_fled_module_exit(void)
{
	platform_driver_unregister(&rt8547_fled_driver);
}

device_initcall(rt8547_fled_module_init);
module_exit(rt8547_fled_module_exit);

MODULE_DESCRIPTION("Richtek RT8547 FlashLED Driver");
MODULE_AUTHOR("Patrick Chang <patrick_chang@richtek.com>");
MODULE_VERSION(RT8547_DRV_VER);
MODULE_LICENSE("GPL");
