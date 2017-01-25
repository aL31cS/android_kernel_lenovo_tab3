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
#include "iqs263.h"
#include "upmu_sw.h"
#include "upmu_common.h"
#include <linux/gpio.h>
#include <linux/of_irq.h>

#include <linux/wakelock.h>
#include <linux/sched.h>

#include <linux/dma-mapping.h>

#include <captouch.h>
/******************************************************************************
 * configuration
*******************************************************************************/
#define IQS263_DEV_NAME     "IQS263"

/******************************************************************************
 * extern functions
*******************************************************************************/
static DEFINE_MUTEX(iqs263_mutex);

/*----------------------------------------------------------------------------*/
#define CAPTOUCH_EINT_TOUCH	(1)
#define CAPTOUCH_EINT_NO_TOUCH	(0)
#define IQS263_SUPPORT_I2C_DMA
/*----------------------------------------------------------------------------*/
static struct work_struct captouch_eint_work;
int captouch_eint_status=0;
int captouch_irq;
unsigned int captouch_gpiopin;
unsigned int captouch_debounce;
unsigned int captouch_eint_type;
static int captouch_init_flag = 0;

static struct i2c_client *iqs263_i2c_client;

static const struct i2c_device_id iqs263_i2c_id[] = { {"IQS263", 0}, {} };

static int iqs263_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int iqs263_i2c_remove(struct i2c_client *client);
static int iqs263_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int iqs263_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int iqs263_i2c_resume(struct i2c_client *client);

static int captouch_local_init(void);
static int captouch_remove(void);

#ifdef CONFIG_OF
static const struct of_device_id captouch_of_match[] = {
	{.compatible = "mediatek,capsensor"},
	{},
};
#endif

static struct i2c_driver iqs263_i2c_driver = {
	.probe = iqs263_i2c_probe,
	.remove = iqs263_i2c_remove,
	.detect = iqs263_i2c_detect,
	.suspend = iqs263_i2c_suspend,
	.resume = iqs263_i2c_resume,
	.id_table = iqs263_i2c_id,
	.driver = {
		   .name = IQS263_DEV_NAME,
#ifdef CONFIG_OF
			.of_match_table = captouch_of_match,
#endif
		   },
};

/*----------------------------------------------------------------------------*/
static struct captouch_init_info iqs263_init_info = {
		.name = IQS263_DEV_NAME,
		.init = captouch_local_init,
		.uninit = captouch_remove,
	
};

#if defined(IQS263_SUPPORT_I2C_DMA)

static int iqs263_i2c_write_dma(struct i2c_client *client, uint8_t regaddr, uint8_t txbyte, uint8_t *data)
{
	uint8_t *v_buf = NULL;
	dma_addr_t p_buf;
	int ret = 0;
	struct i2c_msg msg;

	memset(&msg, 0, sizeof(struct i2c_msg));

	v_buf = dma_alloc_coherent(&(client->adapter->dev), txbyte, &p_buf, GFP_KERNEL|GFP_DMA32);

	mutex_lock(&iqs263_mutex);

	*v_buf = regaddr;
	memcpy((v_buf + 1), data, txbyte);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = txbyte + 1;
	msg.buf = (uint8_t *)p_buf;
	msg.ext_flag |= I2C_DMA_FLAG;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret != 1)
	{
		CAPTOUCH_LOG("iqs263 i2c write data error %d\r\n", ret);
	}

	mutex_unlock(&iqs263_mutex);

	return ret;
}

static int iqs263_i2c_read_dma(struct i2c_client *client, uint8_t regaddr, uint8_t rxbyte, uint8_t *data) 
{
	uint8_t *v_buf = NULL;
	dma_addr_t p_buf;
	int ret = 0;
	struct i2c_msg msg;

	memset(&msg, 0, sizeof(struct i2c_msg));

	v_buf = dma_alloc_coherent(&(client->adapter->dev), rxbyte, &p_buf, GFP_KERNEL|GFP_DMA32);

	mutex_lock(&iqs263_mutex);

	*v_buf = regaddr;
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = ((rxbyte & 0x1F)<<8) | 1;
	msg.buf = (uint8_t *)p_buf;
	msg.ext_flag = I2C_WR_FLAG | I2C_RS_FLAG | I2C_DMA_FLAG;

	ret = i2c_transfer(client->adapter, &msg, 1);

	memcpy(data, v_buf, rxbyte);

	if(ret != 1)
	{
		CAPTOUCH_LOG("iqs263_i2c_read err=%d\r\n", ret);
	}

	mutex_unlock(&iqs263_mutex);

	return ret;
}

#endif

static int iqs263_i2c_write(struct i2c_client *client, uint8_t regaddr, uint8_t txbyte, uint8_t *data)
{
	uint8_t buffer[8];
	int ret = 0;

	mutex_lock(&iqs263_mutex);

	buffer[0] = regaddr;
	buffer[1] = data[0];
	buffer[2] = data[1];
	buffer[3] = data[2];
	buffer[4] = data[3];
	buffer[5] = data[4];
	buffer[6] = data[5];
	buffer[7] = data[6];

	ret = i2c_master_send(client, buffer, txbyte);

	if (ret < 0)
	{
		CAPTOUCH_LOG("iqs263 i2c write %x error %d\r\n", regaddr, ret);

		mutex_unlock(&iqs263_mutex);

		return ret;
	}
		
	mutex_unlock(&iqs263_mutex);

	return ret;
}
/*-----------------------------------------------------------------------------*/
static int iqs263_i2c_read(struct i2c_client *client, uint8_t regaddr, uint8_t rxbyte, uint8_t *data)
{
	int ret = 0;
	struct i2c_msg msg;

	memset(&msg, 0, sizeof(struct i2c_msg));

	mutex_lock(&iqs263_mutex);

	data[0] = regaddr;
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = ((rxbyte & 0x1f)<<8) | 1;
	msg.buf = data;
	msg.ext_flag = I2C_WR_FLAG | I2C_RS_FLAG;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret != 1)
	{
		CAPTOUCH_LOG("iqs263_i2c_read ret=%d\r\n", ret);
	}
	
	mutex_unlock(&iqs263_mutex);

	return ret;
}
/*-----------------------------------------------------------------------------*/
static irqreturn_t iqs263_eint_func(int irq, void *desc)
{
	//CAPTOUCH_LOG(" debug eint function performed!\n");
	disable_irq_nosync(captouch_irq);
	schedule_work(&captouch_eint_work);

	return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
int iqs263_setup_eint(struct i2c_client *client)
{
	int ret;
	u32 ints[2] = { 0, 0 };
	u32 ints1[2] = { 0, 0 };	
	struct device_node *node = NULL;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_eint_as_int;
	struct platform_device *captouch_pdev = get_captouch_platformdev();

	CAPTOUCH_LOG("iqs263_setup_eint\n");

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
		ret = request_irq(captouch_irq, iqs263_eint_func, IRQF_TRIGGER_NONE, "captou-eint", NULL);
		if (ret != 0) 
		{
			CAPTOUCH_ERR("EINT IRQ LINE NOT AVAILABLE\n");
		}
		else
		{
			disable_irq(captouch_irq);
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
static void iqs263_eint_work(struct work_struct *work)
{
	int err=0;
	uint8_t buffer[8]={0};

	#if 0
	iqs263_i2c_read(iqs263_i2c_client, 0x0a, 8, buffer);

	CAPTOUCH_LOG("iqs263_i2c_read buffer[0]=%x \n", buffer[0]);
	CAPTOUCH_LOG("iqs263_i2c_read buffer[1]=%x \n", buffer[1]);
	CAPTOUCH_LOG("iqs263_i2c_read buffer[2]=%x \n", buffer[2]);
	CAPTOUCH_LOG("iqs263_i2c_read buffer[3]=%x \n", buffer[3]);
	CAPTOUCH_LOG("iqs263_i2c_read buffer[4]=%x \n", buffer[4]);
	CAPTOUCH_LOG("iqs263_i2c_read buffer[5]=%x \n", buffer[5]);
	CAPTOUCH_LOG("iqs263_i2c_read buffer[6]=%x \n", buffer[6]);
	CAPTOUCH_LOG("iqs263_i2c_read buffer[7]=%x \n", buffer[7]);
	#endif
	CAPTOUCH_LOG("iqs263_i2c_read captouch_init_flag=%d \n", captouch_init_flag);

	if (0 == captouch_init_flag)
	{
		buffer[0] = 0x02;
		err = iqs263_i2c_write(iqs263_i2c_client, 0x0d, 2, buffer);
		if (err >= 0)
		{
			captouch_init_flag = 1;
		}
	}
	else if (1 == captouch_init_flag)
	{
		buffer[0] = 0x50;
		buffer[1] = 0x05;
		buffer[2] = 0x05;
		buffer[3] = 0x10;
		buffer[4] = 0x03;
		buffer[5] = 0x00;
		buffer[6] = 0xff;
		err = iqs263_i2c_write(iqs263_i2c_client, 0x0a, 8, buffer);
		if (err >= 0)
		{
			captouch_init_flag = 2;
		}
	}
	else if (2 == captouch_init_flag)
	{
		buffer[0] = 0x00;
		buffer[1] = 0xDF;
		buffer[2] = 0xDD;
		err = iqs263_i2c_write(iqs263_i2c_client, 0x0b, 4, buffer);
		if (err >= 0)
		{
			captouch_init_flag = 3;
		}
	}
	else if (3 == captouch_init_flag)
	{
		buffer[0] = 0x3f;
		buffer[1] = 0x2a;
		buffer[2] = 0x37;
		buffer[3] = 0x00;
		buffer[4] = 0x14;
		err = iqs263_i2c_write(iqs263_i2c_client, 0x07, 6, buffer);
		if (err >= 0)
		{
			captouch_init_flag = 4;
		}
	}
	else if (4 == captouch_init_flag)
	{
		buffer[0] = 0x10;
		buffer[1] = 0x41;
		buffer[2] = 0x00;
		buffer[3] = 0x00;
		buffer[4] = 0x02;
		err = iqs263_i2c_write(iqs263_i2c_client, 0x09, 6, buffer);
		if (err >= 0)
		{
			captouch_init_flag = 5;
		}
	}
	else if (5 == captouch_init_flag)
	{
		err = iqs263_i2c_read(iqs263_i2c_client, 0x03, 1, buffer);
		CAPTOUCH_LOG("iqs263_i2c_read buffer[0]=%x \n", buffer[0]);
		#if 0
		if (err >= 0)
		{
			if (buffer[0] & 0x02)
			{
				captouch_eint_status = 1;
			}
			else
			{
				captouch_eint_status = 0;
			}

			//let up layer to know
			if((err = captouch_report_interrupt_data(captouch_eint_status)))
			{
				CAPTOUCH_ERR("iqs263 call captouch_report_interrupt_data fail = %d\n", err);
			}
		}
		#endif
	}	

	enable_irq(captouch_irq);
}
/*----------------------------------------------------------------------------*/
static ssize_t iqs263_show_enable(struct device_driver *ddri, char *buf)
{
	CAPTOUCH_LOG("iqs263_show_enable \n");

	enable_irq(captouch_irq);

	return sprintf(buf, "%u\n", 2);
}

static DRIVER_ATTR(iqs263_enable, S_IWUSR | S_IRUGO, iqs263_show_enable, NULL);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *iqs263_attr_list[] =
{
	&driver_attr_iqs263_enable,
};
/*----------------------------------------------------------------------------*/
static int iqs263_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(iqs263_attr_list) / sizeof(iqs263_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, iqs263_attr_list[idx]);
		if (err) {
			CAPTOUCH_ERR("driver_create_file (%s) = %d\n", iqs263_attr_list[idx]->attr.name,
				err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int iqs263_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(iqs263_attr_list) / sizeof(iqs263_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, iqs263_attr_list[idx]);

	return err;
}
/*----------------------------------------------------------------------------*/
static int iqs263_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct captouch_control_path cap_ctl={0};
	int err = 0;

	CAPTOUCH_FUN();

	INIT_WORK(&captouch_eint_work, iqs263_eint_work);

	iqs263_i2c_client = client;

	iqs263_setup_eint(client);

	CAPTOUCH_LOG("iqs263_init_client OK!\n");

	if((err = iqs263_create_attr(&iqs263_init_info.platform_diver_addr->driver)))
	{
		CAPTOUCH_ERR("iqs263 create attribute err = %d\n", err);
		return err;
	}

	err = captouch_register_control_path(&cap_ctl);
	if(err)
	{
		CAPTOUCH_ERR("captouch register fail = %d\n", err);
		return err;
	}

	CAPTOUCH_LOG("%s: OK\n", __func__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int iqs263_i2c_remove(struct i2c_client *client)
{
	int err;

	err = iqs263_delete_attr(&iqs263_init_info.platform_diver_addr->driver);
	if (err)
	{
		CAPTOUCH_ERR("IQS263_delete_attr fail: %d\n", err);
	}

	i2c_unregister_device(client);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int iqs263_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, IQS263_DEV_NAME);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int iqs263_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	int err = 0;

	CAPTOUCH_FUN();

	return err;
}
/*----------------------------------------------------------------------------*/
static int iqs263_i2c_resume(struct i2c_client *client)
{
	int err = 0;

	CAPTOUCH_FUN();

	return err;
}
/*----------------------------------------------------------------------------*/
static int captouch_local_init(void) 
{
	CAPTOUCH_FUN();

	if (i2c_add_driver(&iqs263_i2c_driver))
	{
		CAPTOUCH_ERR("add driver error\n");
		return -1;
	}

	CAPTOUCH_ERR("add driver iqs263_i2c_driver ok!\n");
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int captouch_remove(void)
{
	CAPTOUCH_FUN();
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init iqs263_init(void)
{
	CAPTOUCH_FUN();

	captouch_driver_add(&iqs263_init_info);
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit iqs263_exit(void)
{
	CAPTOUCH_FUN();	
}
/*----------------------------------------------------------------------------*/
module_init(iqs263_init);
module_exit(iqs263_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("IQS263 driver");
MODULE_LICENSE("GPL");
