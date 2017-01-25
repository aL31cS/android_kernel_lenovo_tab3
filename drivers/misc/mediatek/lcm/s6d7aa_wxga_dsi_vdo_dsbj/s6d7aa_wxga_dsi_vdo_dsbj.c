#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <asm-generic/gpio.h>

#include "lcm_drv.h"
#include "ddp_irq.h"

static unsigned int GPIO_LCD_PWR_EN;
static unsigned int GPIO_LCD_BL_EN;
/**
 * Local Constants
 */
#define FRAME_WIDTH		(800)
#define FRAME_HEIGHT		(1280)

#define REGFLAG_DELAY		0xFB
#define REGFLAG_END_OF_TABLE	0xFD   /* END OF REGISTERS MARKER */

#define LCM_DSI_CMD_MODE	0

/**
 * Local Variables
 */
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


/**
 * Local Functions
 */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)				lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg				lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#ifndef ASSERT
#define ASSERT(expr)					\
	do {						\
		if (expr)				\
			break;				\
		pr_debug("DDP ASSERT FAILED %s, %d\n",	\
		       __FILE__, __LINE__);		\
		BUG();					\
	} while (0)
#endif

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

/**
 * Note :
 *
 * Data ID will depends on the following rule.
 *
 * count of parameters > 1	=> Data ID = 0x39
 * count of parameters = 1	=> Data ID = 0x15
 * count of parameters = 0	=> Data ID = 0x05
 *
 * Structure Format :
 *
 * {DCS command, count of parameters, {parameter list}}
 * {REGFLAG_DELAY, milliseconds of time, {} },
 * ...
 *
 * Setting ending by predefined flag
 *
 * {REGFLAG_END_OF_TABLE, 0x00, {}}
 */
static struct LCM_setting_table lcm_initialization_setting[] = {
// page 1
	/*{0xF0, 5, {0x55,0xAA,0x52,0x08,0x01}},
	{0xCA, 1, {0x00}},
	{0xCE, 1, {0x34}},
	{0xBC, 2, {0x68,0x01}},
	{0xBD, 2, {0x68,0x01}},

// page 3 
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x03}},
	{0xB0, 4, {0x00,0x23,0x00,0x00}},
	{0xB2, 7, {0x01,0x00,0x06,0x04,0x00,0xD8,0x42}},
	{0xB3, 7, {0x01,0x00,0x05,0x04,0x00,0xD8,0x42}},
	{0xBA, 7, {0x85,0x03,0x00,0x04,0x01,0xD8,0x42}},
	{0xBB, 7, {0x85,0x03,0x00,0x03,0x01,0xD8,0x42}},

// page 2 
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x02}},
	{0xB0, 1, {0x40}},
	{0xD1, 16, {0x00,0x00,0x00,0x12,0x00,0x3C,0x00,0x55,0x00,0x78,0x00,0x9D,0x00,0xBE,0x00,0xF0}},
	{0xD2, 16, {0x01,0x18,0x01,0x59,0x01,0x8E,0x01,0xDD,0x02,0x1F,0x02,0x21,0x02,0x5D,0x02,0x9E}},
	{0xD3, 16, {0x02,0xC6,0x03,0x00,0x03,0x26,0x03,0x59,0x03,0x7A,0x03,0xA2,0x03,0xBB,0x03,0xD8}},
	{0xD4, 4, {0x03,0xF6,0x03,0xFF}},

// page 6 
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x06}},
	{0xB3, 1, {0x03}},
	{0xB1, 5, {0x19,0x18,0x18,0x02,0x3A}},
	{0xB2, 2, {0x3A,0x3A}},
	{0xB6, 5, {0x13,0x12,0x12,0x00,0x3A}},*/
	
	/*{0xF0, 5, {0x55,0xAA,0x52,0x08,0x01}},
		//{0xCA, 1, {0x00}},
		{0xCE, 1, {0x04}},
		{0xBC, 2, {0x68,0x01}},
		{0xBD, 2, {0x68,0x01}},
	
	// page 3 
		{0xFF, 4, {0xAA,0x55,0xA5,0x80}},
		{0x6F, 1, {0x09}},
		{0xF7, 1, {0x82}},
		{0x6F, 1, {0x0B}},
		{0xF7, 1, {0xE0}},
		{0x55, 1, {0x81},
	
	// page 2 
		{0x53, 1, {0x2C}},
		{0x51, 1, {0xFF}},*/

	{0xF0, 2, {0x5A,0x5A}},
	{0xF1, 2, {0x5A,0x5A}},
	{0xFC, 2, {0xA5,0x5A}},	

	{0xD0, 2, {0x00,0x10}},
	{0xBC, 1, {0x00}},
	{0xF3, 10, {0x01,0xC7,0xE2,0x62,0xF4,0xF7,0x77,0x3C,0x26,0x00}},
	{REGFLAG_DELAY, 10, {}},

	{0xC3, 3, {0x40,0x00,0x28}},
	{REGFLAG_DELAY, 50, {}},
	{0xF5, 1, {0xD6}},
	{0x11, 0, {}},

	{REGFLAG_DELAY, 120, {}},
	{0x29, 0, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


#if 0
static struct LCM_setting_table lcm_set_window[] = {
	{0x2A, 4, {0x00, 0x00, (FRAME_WIDTH >> 8), (FRAME_WIDTH&0xFF) } },
	{0x2B, 4, {0x00, 0x00, (FRAME_HEIGHT >> 8), (FRAME_HEIGHT&0xFF)} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
	/* Sleep Out */
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },

	/* Display ON */
	{0x29, 0, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	/* Sleep Mode On */
	{0x28,0,{}},
	{REGFLAG_DELAY, 40, {}},    //-----liken add
	{0xC3, 3, {0x40,0x00,0x20}},//-----liken add
	{0x10,0,{}},
	{REGFLAG_DELAY, 120, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;
		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
					table[i].para_list, force_update);
		}
	}
}

/**
 * LCM Driver Implementations
 */
static void lcm_get_gpio_infor(void)
{
	static struct device_node *node;

	//node = of_find_compatible_node(NULL, NULL, "mediatek,mt6735-dispsys");
	node = of_find_compatible_node(NULL, NULL, "mediatek,lcm");

	GPIO_LCD_PWR_EN = of_get_named_gpio(node, "lcm_power_gpio", 0);
	GPIO_LCD_BL_EN = of_get_named_gpio(node, "lcm_bl_gpio", 0);
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
	params->dsi.mode = SYNC_EVENT_VDO_MODE;
#endif

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	/* Not support in MT6573 */
	params->dsi.packet_size = 256;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count=FRAME_WIDTH*3;
	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 8;
	params->dsi.vertical_frontporch					= 8;
	params->dsi.vertical_active_line					= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 4;
	params->dsi.horizontal_backporch				= 132;
	params->dsi.horizontal_frontporch				= 24;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    	params->dsi.ssc_disable							= 1;
	params->dsi.cont_clock 	= 1;
	params->dsi.PLL_CLOCK = 234; 
	params->dsi.clk_lp_per_line_enable = 1;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
}

#if defined(CONFIG_CUSTOM_KERNEL_CAPTOUCH)
extern volatile int call_status;
#endif

static void lcm_init(void)
{
	printk("[LCM] lcm_init() enter\n");
	lcm_get_gpio_infor();
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, 1);
	MDELAY(30);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);

	push_table(lcm_initialization_setting,
		   sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	lcm_set_gpio_output(GPIO_LCD_BL_EN, 1);

}


static void lcm_suspend(void)
{
	printk("[LCM] lcm_suspend() enter\n");
	lcm_get_gpio_infor();

	lcm_set_gpio_output(GPIO_LCD_BL_EN, 0);
	push_table(lcm_deep_sleep_mode_in_setting,
		   sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(0);
	MDELAY(10);
#if defined(CONFIG_CUSTOM_KERNEL_CAPTOUCH)
	if (call_status == 0)
	{
		lcm_set_gpio_output(GPIO_LCD_PWR_EN, 0);
	}
#else
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, 0);
#endif
}


static void lcm_resume(void)
{	
	printk("[LCM] lcm_resume() enter\n");
	lcm_init();
}

#if 0
static void lcm_setpwm(unsigned int divider)
{
	/* TBD */
}


static unsigned int lcm_getpwm(unsigned int divider)
{
	/* ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk; */
	/* pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706 */
	unsigned int pwm_clk = 23706 / (1 << divider);

	return pwm_clk;
}
#endif

LCM_DRIVER s6d7aa_wxga_dsi_vdo_dsbj_lcm_drv = {
	.name		= "s6d7aa_wxga_dsi_vdo_dsbj",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
#if (LCM_DSI_CMD_MODE)
	/*.set_backlight	= lcm_setbacklight,*/
	/* .set_pwm        = lcm_setpwm, */
	/* .get_pwm        = lcm_getpwm, */
	/*.update         = lcm_update, */
#endif
};
