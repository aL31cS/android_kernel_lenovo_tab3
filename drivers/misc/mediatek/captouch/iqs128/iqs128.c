/* drivers/hwmon/mt6516/amit/IQS128.c - IQS128/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>

#include <linux/io.h>
#include "iqs128.h"
#include "upmu_sw.h"
#include "upmu_common.h"
#include <linux/gpio.h>
#include <linux/of_irq.h>

#include <linux/wakelock.h>
#include <linux/sched.h>

#include <captouch.h>
/******************************************************************************
 * configuration
*******************************************************************************/
#define IQS128_DEV_NAME     "IQS128"

/******************************************************************************
 * extern functions
*******************************************************************************/

/*----------------------------------------------------------------------------*/
#define CAPTOUCH_EINT_TOUCH	(1)
#define CAPTOUCH_EINT_NO_TOUCH	(0)
/*----------------------------------------------------------------------------*/
static struct work_struct captouch_eint_work;
int captouch_eint_status=0;
int captouch_irq;
unsigned int captouch_gpiopin;
unsigned int captouch_debounce;
unsigned int captouch_eint_type;

static int captouch_local_init(void);
static int captouch_remove(void);

/*----------------------------------------------------------------------------*/
static struct captouch_init_info iqs128_init_info = {
		.name = IQS128_DEV_NAME,
		.init = captouch_local_init,
		.uninit = captouch_remove,
	
};
/*-----------------------------------------------------------------------------*/
static irqreturn_t iqs128_eint_func(int irq, void *desc)
{
	//CAPTOUCH_LOG(" debug eint function performed!\n");
	disable_irq_nosync(captouch_irq);
	schedule_work(&captouch_eint_work);

	return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
int iqs128_setup_eint(void)
{	
	int ret;
	u32 ints[2] = { 0, 0 };
	u32 ints1[2] = { 0, 0 };	
	struct device_node *node = NULL;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_eint_as_int;
	struct platform_device *captouch_pdev = get_captouch_platformdev();

	CAPTOUCH_LOG("iqs128_setup_eint\n");

	/* gpio setting */
	pinctrl = devm_pinctrl_get(&captouch_pdev->dev);
	if (IS_ERR(pinctrl))
	{
		ret = PTR_ERR(pinctrl);
		CAPTOUCH_ERR("Cannot find captouch pinctrl!\n");
		return ret;
	}

	pins_default = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR(pins_default))
	{
		ret = PTR_ERR(pins_default);
		CAPTOUCH_ERR("Cannot find captouch pinctrl default!\n");
	}

	pins_eint_as_int = pinctrl_lookup_state(pinctrl, "state_eint_as_int");
	if (IS_ERR(pins_eint_as_int))
	{
		ret = PTR_ERR(pins_eint_as_int);
		CAPTOUCH_ERR("Cannot find captouch pinctrl state_eint_as_int!\n");
	}
	pinctrl_select_state(pinctrl, pins_eint_as_int);

	node = of_find_compatible_node(NULL, NULL, "mediatek, captou-eint");

	if (node)
	{
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
		captouch_gpiopin = ints[0];
		captouch_debounce = ints[1];
		captouch_eint_type = ints1[1];
		CAPTOUCH_LOG("ints[0] = %d, ints[1] = %d, ints1[1] = %d!!\n", ints[0], ints[1], ints1[1]);
		gpio_set_debounce(captouch_gpiopin, captouch_debounce);
		captouch_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(captouch_irq, iqs128_eint_func, IRQF_TRIGGER_NONE, "captou-eint", NULL);
		if (ret != 0) 
		{
			CAPTOUCH_ERR("EINT IRQ LINE NOT AVAILABLE\n");
		}
		else
		{
			CAPTOUCH_LOG("captouch set EINT finished, captouch_irq=%d, captouch_debounce=%d\n", captouch_irq, captouch_debounce);
		}
	}
	else
	{
		CAPTOUCH_ERR("%s can't find compatible node\n", __func__);
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static void iqs128_eint_work(struct work_struct *work)
{
	int err;

	if (captouch_eint_status == CAPTOUCH_EINT_NO_TOUCH)
	{
		captouch_eint_status = CAPTOUCH_EINT_TOUCH;
		if (captouch_eint_type == IRQ_TYPE_LEVEL_LOW)
		{
			irq_set_irq_type(captouch_irq, IRQ_TYPE_LEVEL_HIGH);
		}
		else
		{
			irq_set_irq_type(captouch_irq, IRQ_TYPE_LEVEL_LOW);
		}
	}
	else
	{
		captouch_eint_status = CAPTOUCH_EINT_NO_TOUCH;
		if (captouch_eint_type == IRQ_TYPE_LEVEL_LOW)
		{
			irq_set_irq_type(captouch_irq, IRQ_TYPE_LEVEL_LOW);
		}
		else
		{
			irq_set_irq_type(captouch_irq, IRQ_TYPE_LEVEL_HIGH);
		}
	}

	//let up layer to know
	if((err = captouch_report_interrupt_data(captouch_eint_status)))
	{
		CAPTOUCH_ERR("iqs128 call captouch_report_interrupt_data fail = %d\n", err);
	}	

	enable_irq(captouch_irq);
	
	return;
}

/*----------------------------------------------------------------------------*/
static ssize_t iqs128_show_enable(struct device_driver *ddri, char *buf)
{
	CAPTOUCH_LOG("iqs128_show_enable \n");

	enable_irq(captouch_irq);

	return sprintf(buf, "%u\n", 2);
}

static DRIVER_ATTR(iqs128_enable, S_IWUSR | S_IRUGO, iqs128_show_enable, NULL);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *iqs128_attr_list[] =
{
	&driver_attr_iqs128_enable,
};
/*----------------------------------------------------------------------------*/
static int iqs128_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(iqs128_attr_list) / sizeof(iqs128_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, iqs128_attr_list[idx]);
		if (err) {
			CAPTOUCH_ERR("driver_create_file (%s) = %d\n", iqs128_attr_list[idx]->attr.name,
				err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int captouch_local_init(void) 
{
	struct captouch_control_path cap_ctl={0};
	int err = 0;

	CAPTOUCH_FUN();

	INIT_WORK(&captouch_eint_work, iqs128_eint_work);

	iqs128_setup_eint();

	if((err = iqs128_create_attr(&iqs128_init_info.platform_diver_addr->driver)))
	{
		CAPTOUCH_ERR("iqs128 create attribute err = %d\n", err);
		return err;
	}

	err = captouch_register_control_path(&cap_ctl);
	if(err)
	{
		CAPTOUCH_ERR("captouch register fail = %d\n", err);
		return err;
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int captouch_remove(void)
{
	CAPTOUCH_FUN();
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init iqs128_init(void)
{
	CAPTOUCH_FUN();
	
	captouch_driver_add(&iqs128_init_info);
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit iqs128_exit(void)
{
	CAPTOUCH_FUN();	
}
/*----------------------------------------------------------------------------*/
module_init(iqs128_init);
module_exit(iqs128_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("IQS128 driver");
MODULE_LICENSE("GPL");
