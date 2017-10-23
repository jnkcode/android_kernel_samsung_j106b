/* drivers/video/sprdfb/lcd_st7701_mipi.c
 *
 * Support for st7701 mipi LCD device
 *
 * Copyright (C) 2010 Spreadtrum
 */

//#include <asm/arch/sprd_lcd.h>
#include <linux/kernel.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include "../sprdfb_panel.h"
#include <mach/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <soc/sprd/regulator.h>

#define  LCD_DEBUG
#ifdef LCD_DEBUG
#define LCD_PRINT printk
#else
#define LCD_PRINT(...)
#endif

#define MAX_DATA   150

typedef struct LCM_Init_Code_tag {
	unsigned int tag;
	unsigned char data[MAX_DATA];
}LCM_Init_Code;

typedef struct LCM_force_cmd_code_tag{
	unsigned int datatype;
	LCM_Init_Code real_cmd_code;
}LCM_Force_Cmd_Code;

#define LCM_TAG_SHIFT 24
#define LCM_TAG_MASK  ((1 << 24) -1)
#define LCM_SEND(len) ((1 << LCM_TAG_SHIFT)| len)
#define LCM_SLEEP(ms) ((2 << LCM_TAG_SHIFT)| ms)
//#define ARRAY_SIZE(array) ( sizeof(array) / sizeof(array[0]))

#define LCM_TAG_SEND  (1<< 0)
#define LCM_TAG_SLEEP (1 << 1)

#define LCD_NEW_PANEL

static LCM_Init_Code init_data[] = {
	{LCM_SEND(1), {0x01}},
	{LCM_SLEEP(120)},	/*>120ms*/
	{LCM_SEND(8),  {6,   0, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x11} },
	{LCM_SEND(4),  {2,   0, 0xd1, 0x11} },
	{LCM_SEND(1), {0x11}},
	{LCM_SLEEP(120)},//>120ms
//	{LCM_SEND(1), {0x29}},
//	{LCM_SLEEP(20)}, //>20ms
	{LCM_SEND(8),  {6,   0, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x10} },



	{LCM_SEND(5),  {3,   0, 0xC0, 0x63 , 0x00} },

	{LCM_SEND(5),  {3,   0, 0xC1, 0x09 , 0x02} }, 

	{LCM_SEND(5),  {3,   0, 0xC2, 0x07 , 0x00} },
	

	{LCM_SEND(4),  {2,   0, 0xC7, 0x04 } },

	{LCM_SEND(4),  {2,   0, 0xCB, 0x12 } }, 

	{LCM_SEND(19), {17, 0, 0xB0, 0x00, 0x02, 0x86, 0x0F, 0x15, 0x0A, 0x03, \
			0x09, 0x09, 0x19, 0x08, 0x15, 0x12, 0x89, 0x0D, 0xD7} },
	{LCM_SEND(19), {17, 0, 0xB1, 0x00, 0x01, 0x87, 0x0F, 0x14, 0x08, 0x02, \
		        0x09, 0x09, 0x1B, 0x09, 0x18, 0x14, 0x8F, 0x14, 0xD7} },

  	{LCM_SEND(8),  {6,   0, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x11} },
  	{LCM_SEND(4),  {2,   0, 0xB0, 0x5F} },
  	{LCM_SEND(4),  {2,   0, 0xB1, 0x4A} },
  	{LCM_SEND(4),  {2,   0, 0xB2, 0x07} },
  	{LCM_SEND(4),  {2,   0, 0xB3, 0x80} },
  	{LCM_SEND(4),  {2,   0, 0xB5, 0x47} },
  	{LCM_SEND(4),  {2,   0, 0xB7, 0x8A} },
  	{LCM_SEND(4),  {2,   0, 0xB8, 0x20} },
  	{LCM_SEND(4),  {2,   0, 0xB9, 0x11} },
  	{LCM_SEND(4),  {2,   0, 0xBA, 0x00} },
  	{LCM_SEND(4),  {2,   0, 0xBB, 0x00} },		
  	{LCM_SEND(4),  {2,   0, 0xBC, 0x00} },


  	{LCM_SEND(4),  {2,   0, 0xC0, 0x03} },


  	{LCM_SEND(4),  {2,   0, 0xC1, 0x78} },
  	{LCM_SEND(4),  {2,   0, 0xC2, 0x78} },
  	{LCM_SEND(4),  {2,   0, 0xD0, 0x88} },
  	{LCM_SEND(6),  {4,   0, 0xE0, 0x00, 0x00, 0x02} },
	{LCM_SEND(14), {12,  0, 0xE1, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60} },
	{LCM_SEND(16), {14,  0, 0xE2, 0x30, 0x30, 0x40, 0x40, 0x29, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00} },
	{LCM_SEND(7),  {5,   0, 0xE3, 0x00, 0x00, 0x33, 0x33} },
	{LCM_SEND(5),  {3,   0, 0xE4, 0x44, 0x44} },
	{LCM_SEND(19), {17,  0, 0xE5, 0x06, 0x30, 0x3C, 0xA0, 0x08, 0x30, 0x3C, 0xA0, 0x0A, 0x30, 0x3C, 0xA0, 0x0C, 0x30, 0x3C, 0xA0} },
	{LCM_SEND(7),  {5,   0, 0xE6, 0x00, 0x00, 0x33, 0x33} },
	{LCM_SEND(5),  {3,   0, 0xE7, 0x44, 0x44} },
	{LCM_SEND(19), {17,  0, 0xE8, 0x05, 0x30, 0x3C, 0xA0, 0x07, 0x30, 0x3C, 0xA0, 0x09, 0x30, 0x3C, 0xA0, 0x0B, 0x30, 0x3C, 0xA0} },
	{LCM_SEND(10), {8,   0, 0xEB, 0x02, 0x01, 0xE4, 0xE4, 0x88, 0x00, 0x00} },
	{LCM_SEND(19), {17,  0, 0xED, 0xFA, 0xB0, 0x2F, 0xF4, 0x65, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0x56, 0x4F, 0xF2, 0x0B, 0xAF} },

	{LCM_SEND(8),  {6,   0, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x13} },
	{LCM_SEND(5),  {3,   0, 0xE6, 0x10, 0x49} },
	{LCM_SEND(4),  {2,   0, 0x36, 0x10} },
	{LCM_SEND(8),  {6,   0, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x00} },
	{LCM_SLEEP(120)},//>120ms

	/*Display On*/
	//{LCM_SEND(1), {0x11} },
	//{LCM_SLEEP(120)},	/*>120ms*/
	{LCM_SEND(1), {0x29} },
	{LCM_SLEEP(100)}
};




static LCM_Init_Code disp_on =  {LCM_SEND(1), {0x29}};

static LCM_Init_Code sleep_in[] =  {
	{LCM_SEND(1), {0x28}},
	{LCM_SLEEP(150)}, 	//>150ms
	{LCM_SEND(1), {0x10}},
	{LCM_SLEEP(150)},	//>150ms
};

static LCM_Init_Code sleep_out[] =  {
	{LCM_SEND(1), {0x11}},
	{LCM_SLEEP(120)},//>120ms
	{LCM_SEND(1), {0x29}},
	{LCM_SLEEP(20)}, //>20ms
};

extern void (*lcd_panel_cabc_pwm_bl)(int brightness);
//extern void backlight_control(int brigtness);
static int32_t st7701_mipi_init(struct panel_spec *self)
{
	int32_t i = 0;
	LCM_Init_Code *init = init_data;
	unsigned int tag;
	int ret = 0;

//	lcd_panel_cabc_pwm_bl = backlight_control;

	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_dcs_write_t mipi_dcs_write = self->info.mipi->ops->mipi_dcs_write;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	LCD_PRINT("zhouhehe   ---lcd_st7701_init\n");

	mipi_set_cmd_mode();
	mipi_eotp_set(0,0);

	for(i = 0; i < ARRAY_SIZE(init_data); i++){
		tag = (init->tag >>24);
		if(tag & LCM_TAG_SEND){
			ret = mipi_dcs_write(init->data, (init->tag & LCM_TAG_MASK));
			udelay(20);
			if(ret != 0){
				LCD_PRINT("thomas   ---lcd_st7701_init error i == %d\n", i);
			}

		}else if(tag & LCM_TAG_SLEEP){
			mdelay(init->tag & LCM_TAG_MASK);//udelay((init->tag & LCM_TAG_MASK) * 1000);
		}
		init++;
	}
	mipi_eotp_set(0,0);

	return 0;
}

static uint32_t st7701_readid(struct panel_spec *self)
{


	uint32_t  j =0;
	uint8_t read_data[4] = {0};
	int32_t read_rtn = 0;
	uint8_t param[2] = {0};
	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_force_write_t mipi_force_write = self->info.mipi->ops->mipi_force_write;
	mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	LCD_PRINT("lcd_st7701_mipi read id!\n");

	mipi_set_cmd_mode();
	mipi_eotp_set(0,0);

	for(j = 0; j < 4; j++){
		param[0] = 0x01;
		param[1] = 0x00;
		mipi_force_write(0x37, param, 2);
		read_rtn = mipi_force_read(0xda,1,&read_data[0]);
		read_rtn = mipi_force_read(0xdb,1,&read_data[1]);
		read_rtn = mipi_force_read(0xdc,1,&read_data[2]);

		LCD_PRINT("lcd_st7701_mipi read id 0xda value is 0x%x!\n",read_data[0]);
		LCD_PRINT("lcd_st7701_mipi read id 0xdb value is 0x%x!\n",read_data[1]);
		LCD_PRINT("lcd_st7701_mipi read id 0xdc value is 0x%x!\n",read_data[2]);
 
		//if(0x8 == read_data[0] ){  //st7701
		if((0x06 == read_data[0]) && (0xc0 == read_data[1]) && (0x60 == read_data[2])){  //st7701

			printk("lcd_st7701_mipi read id success!\n");
			//return 0x77a108;
			return 0x06c060;
		}
	}

	mipi_eotp_set(0,0);

	LCD_PRINT("lcd_st7701_mipi read id failed!\n");
	
	return 0;

//	return 0x77a108;
}
static int32_t kpled_power_on(int on)
{
	static struct regulator *KPLED_CTL = NULL;
	int err=0;
	if(KPLED_CTL == NULL) {
		KPLED_CTL = regulator_get(NULL, "vddkpled");
		if (IS_ERR(KPLED_CTL)) {
			printk("%s: regulator_get failed\n",__FUNCTION__);
			return -1;
		}
		regulator_enable(KPLED_CTL);
		err = regulator_set_voltage(KPLED_CTL,3000000,3000000);
		if (err)
		{
			printk("%s: regulator_set failed\n",__FUNCTION__);
			regulator_put(KPLED_CTL);
			KPLED_CTL=NULL;
			return -1;
		}
	}
	if(on)
		regulator_enable(KPLED_CTL);
	else
		regulator_disable(KPLED_CTL);
	return 0;
}


int32_t st7701_panel_after_suspend(struct panel_spec *self)
{
	return kpled_power_on(0);
}
int32_t st7701_panel_before_resume(struct panel_spec *self)
{
	return kpled_power_on(1);
}

static int32_t st7701_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	int32_t i = 0;
	LCM_Init_Code *sleep_in_out = NULL;
	unsigned int tag;
	int32_t size = 0;

	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_dcs_write_t mipi_dcs_write = self->info.mipi->ops->mipi_dcs_write;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	printk("kernel st7701_enter_sleep, is_sleep = %d\n", is_sleep);

	if(is_sleep){
		sleep_in_out = sleep_in;
		size = ARRAY_SIZE(sleep_in);
	}else{
		sleep_in_out = sleep_out;
		size = ARRAY_SIZE(sleep_out);
	}

	mipi_set_cmd_mode();
	mipi_eotp_set(0,0);

	for(i = 0; i <size ; i++){
		tag = (sleep_in_out->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_dcs_write(sleep_in_out->data, (sleep_in_out->tag & LCM_TAG_MASK));
		}else if(tag & LCM_TAG_SLEEP){
			msleep(sleep_in_out->tag & LCM_TAG_MASK);
		}
		sleep_in_out++;
	}
	mipi_eotp_set(0,0);

	return 0;
}

static struct panel_operations lcd_st7701_mipi_operations = {
	.panel_init = st7701_mipi_init,
	.panel_readid = st7701_readid,
	.panel_enter_sleep = st7701_enter_sleep,
	.panel_after_suspend=st7701_panel_after_suspend,
	.panel_before_resume=st7701_panel_before_resume,
};

static struct timing_rgb lcd_st7701_mipi_timing = {
	.hfp = 220,  /* unit: pixel */
	.hbp = 110,
	.hsync = 50,
	.vfp = 7, /*unit: line*/
	.vbp = 6,
	.vsync = 5,
};


static struct info_mipi lcd_st7701_mipi_info = {
	.work_mode  = SPRDFB_MIPI_MODE_VIDEO,
	.video_bus_width = 24, /*18,16*/
	.lan_number = 2,
	.phy_feq = 433*1000,
	.h_sync_pol = SPRDFB_POLARITY_POS,
	.v_sync_pol = SPRDFB_POLARITY_POS,
	.de_pol = SPRDFB_POLARITY_POS,
	.te_pol = SPRDFB_POLARITY_POS,
	.color_mode_pol = SPRDFB_POLARITY_NEG,
	.shut_down_pol = SPRDFB_POLARITY_NEG,
	.timing = &lcd_st7701_mipi_timing,
	.ops = NULL,
};

struct panel_spec lcd_st7701_mipi_spec = {
	.width = 480,
	.height = 800,
	.fps = 60,
	.type = LCD_MODE_DSI,
	.direction = LCD_DIRECT_NORMAL,
	.is_clean_lcd = true,
	.info = {
		.mipi = &lcd_st7701_mipi_info
	},
	.ops = &lcd_st7701_mipi_operations,
};
struct panel_cfg lcd_st7701_mipi = {
	/* this panel can only be main lcd */
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x06c060,
	.lcd_name = "lcd_st7701_mipi",
	.panel = &lcd_st7701_mipi_spec,
};

//temp code
static int __init lcd_st7701_mipi_init(void)
{

	return sprdfb_panel_register(&lcd_st7701_mipi);
}

subsys_initcall(lcd_st7701_mipi_init);

