/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*******************************************************************************
 *
 * Filename:
 * ---------
 *   AudDrv_Kernelc
 *
 * Project:
 * --------
 *    Audio smart pa Function
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 * $Revision: #1 $
 * $Modtime:$
 * $Log:$
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/
#include "AudDrv_NXP.h"

#define TFA_I2C_CHANNEL     (2)
#ifdef CONFIG_MTK_NXP_TFA9890
#define ECODEC_SLAVE_ADDR_WRITE 0x68
#define ECODEC_SLAVE_ADDR_READ  0x69
#else
#define ECODEC_SLAVE_ADDR_WRITE 0x6c
#define ECODEC_SLAVE_ADDR_READ  0x6d
#endif

#define I2C_MASTER_CLOCK       400
#define NXPEXTSPK_I2C_DEVNAME "mtksmartpa"

/*****************************************************************************
*           DEFINE AND CONSTANT
******************************************************************************
*/

#define AUDDRV_NXPSPK_NAME   "mtksmartpa"
#define AUDDRV_AUTHOR "MediaTek WCX"
#define RW_BUFFER_LENGTH (256)
#define smart_set_gpio(x) (x|0x80000000)

/*****************************************************************************
*           V A R I A B L E     D E L A R A T I O N
*******************************************************************************/

/* I2C variable */
static struct i2c_client *new_client = NULL;
static char WriteBuffer[RW_BUFFER_LENGTH];
static char ReadBuffer[RW_BUFFER_LENGTH];

static u8 *TfaI2CDMABuf_va = NULL;
#ifdef CONFIG_ARM64
static dma_addr_t TfaI2CDMABuf_pa;
#else
static u64 TfaI2CDMABuf_pa;
#endif
struct platform_device *spa_plt_dev = NULL;
struct pinctrl *spactrl = NULL;
struct pinctrl_state *spa_rst_h = NULL;
struct pinctrl_state *spa_rst_l = NULL;
struct pinctrl_state *spa_hpen_h = NULL;
struct pinctrl_state *spa_hpen_l = NULL;
struct pinctrl_state *spa_nxplrck_mode = NULL;
struct pinctrl_state *spa_nxpbck_mode = NULL;
struct pinctrl_state *spa_nxpdatai_mode = NULL;
struct pinctrl_state *spa_nxpdatao_mode = NULL;

int mtksmartpa_gpio_init(struct platform_device *pdev)
{
	int ret = 0;
	spactrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(spactrl)) {
		dev_err(&pdev->dev, "Cannot find smartpa pinctrl!");
		ret = PTR_ERR(spactrl);
	}
	
	spa_rst_h = pinctrl_lookup_state(spactrl, "aud_ext_dacrst_gpio1");
	if (IS_ERR(spa_rst_h)) {
		ret = PTR_ERR(spa_rst_h);
		printk("%s : pinctrl err, spa_rst_h\n", __func__);
	}
	spa_rst_l = pinctrl_lookup_state(spactrl, "aud_ext_dacrst_gpio0");
	if (IS_ERR(spa_rst_l)) {
		ret = PTR_ERR(spa_rst_l);
		printk("%s : pinctrl err, spa_rst_l\n", __func__);
	}
	
	spa_hpen_h = pinctrl_lookup_state(spactrl, "aud_ext_hpen_gpio1");
	if (IS_ERR(spa_hpen_h)) {
		ret = PTR_ERR(spa_hpen_h);
		printk("%s : pinctrl err, spa_hpen_h\n", __func__);
	}
	spa_hpen_l = pinctrl_lookup_state(spactrl, "aud_ext_hpen_gpio0");
	if (IS_ERR(spa_hpen_l)) {
		ret = PTR_ERR(spa_hpen_l);
		printk("%s : pinctrl err, spa_hpen_l\n", __func__);
	}
	
		spa_nxplrck_mode = pinctrl_lookup_state(spactrl, "nxplrck_mode");
	if (IS_ERR(spa_nxplrck_mode)) {
		ret = PTR_ERR(spa_nxplrck_mode);
		printk("%s : pinctrl err, spa_nxplrck_mode\n", __func__);
	}
		
	spa_nxpbck_mode = pinctrl_lookup_state(spactrl, "nxpbck_mode");
	if (IS_ERR(spa_nxpbck_mode)) {
		ret = PTR_ERR(spa_nxpbck_mode);
		printk("%s : pinctrl err, spa_nxpbck_mode\n", __func__);
	}

	spa_nxpdatai_mode = pinctrl_lookup_state(spactrl, "nxpdatai_mode");
	if (IS_ERR(spa_nxpdatai_mode)) {
		ret = PTR_ERR(spa_nxpdatai_mode);
		printk("%s : pinctrl err, spa_nxpdatai_mode\n", __func__);
	}
	
	spa_nxpdatao_mode = pinctrl_lookup_state(spactrl, "nxpdatao_mode");
	if (IS_ERR(spa_nxpdatao_mode)) {
		ret = PTR_ERR(spa_nxpdatao_mode);
		printk("%s : pinctrl err, spa_nxpdatao_mode\n", __func__);
	}
	
	return ret;
}

int mtksmartpa_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;

	switch (PwrType) {
	case SPARST:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(spactrl, spa_rst_l);
			else
				pinctrl_select_state(spactrl, spa_rst_h);
		} 
		break;
	case SPAHPEN:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(spactrl, spa_hpen_l);
			else
				pinctrl_select_state(spactrl, spa_hpen_h);
		}
		break;
	case SPANXPLRCK:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(spactrl, spa_nxplrck_mode);
		}
		break;
	case SPANXPBCK:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(spactrl, spa_nxpbck_mode);
		}
		break;
	case SPANXPDATAI:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(spactrl, spa_nxpdatai_mode);
		}
		break;
	case SPANXPDATAO:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(spactrl, spa_nxpdatao_mode);
		}
		break;
	default:
		break;
	};

	return ret;
}


/* new I2C register method */
static const struct i2c_device_id nxpExt_i2c_id[] = { {NXPEXTSPK_I2C_DEVNAME, 0}, {} };
static struct i2c_board_info __initdata  nxpExt_dev ={ I2C_BOARD_INFO(NXPEXTSPK_I2C_DEVNAME, 0x36) };

/* function declration */
static int NXPExtSpk_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int NXPExtSpk_i2c_remove(struct i2c_client *client);
void AudDrv_NXPSpk_Init(void);
bool NXPExtSpk_Register(void);
static int NXPExtSpk_register(void);
ssize_t NXPSpk_read_byte(u8 addr, u16 *returnData);
ssize_t NXPExt_write_byte(u8 addr, u8 writeData);
static int NXPExtSpk_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, NXPEXTSPK_I2C_DEVNAME);
    return 0;
}

static int NXPExtSpk_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    return 0;
}

static int NXPExtSpk_i2c_resume(struct i2c_client *client)
{
    return 0;
}
#ifdef CONFIG_OF
static const struct of_device_id mtk_i2c_smart_pa_of_ids[] =
{
    { .compatible = "mediatek,smart_pa", },
    {}
};
#endif
/* i2c driver */
static struct i2c_driver NXPExtSpk_i2c_driver =
{
    .probe = NXPExtSpk_i2c_probe,
    .remove = NXPExtSpk_i2c_remove,
    .detect     =  NXPExtSpk_i2c_detect,
    .suspend    =  NXPExtSpk_i2c_suspend,
    .resume     =  NXPExtSpk_i2c_resume,
    .id_table = nxpExt_i2c_id,
    .driver = {
        .name = NXPEXTSPK_I2C_DEVNAME,
#ifdef CONFIG_OF
    .of_match_table = mtk_i2c_smart_pa_of_ids,
#endif
    },
};


static int NXPExtSpk_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    new_client = client;
    new_client->timing = 400;
    printk("NXPExtSpk_i2c_probe \n");

#ifdef CONFIG_MTK_NXP_TFA9890
		pinctrl_select_state(spactrl, spa_rst_l);
		msleep(10);
		pinctrl_select_state(spactrl, spa_rst_h);
		msleep(10);
		pinctrl_select_state(spactrl, spa_rst_l);
#endif
		
#ifdef CONFIG_ARM64		
		TfaI2CDMABuf_va = (u8 *)dma_alloc_coherent(&(client->dev), 4096, &TfaI2CDMABuf_pa, GFP_KERNEL);
			if(!TfaI2CDMABuf_va)
			{
				NXP_ERROR("dma_alloc_coherent error\n");
				NXP_INFO("i2c_probe failed\n");
	      			return -1;
			}
#else
			TfaI2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &TfaI2CDMABuf_pa, GFP_KERNEL);
			if(!TfaI2CDMABuf_va)
			{
				NXP_ERROR("dma_alloc_coherent error\n");
				NXP_INFO("i2c_probe failed\n");
	      			return -1;
			}
#endif
		NXP_INFO("i2c_probe success\n");
    		return 0;
}

static int NXPExtSpk_i2c_remove(struct i2c_client *client)
{
    new_client = NULL;
    i2c_unregister_device(client);
    i2c_del_driver(&NXPExtSpk_i2c_driver);
	if(TfaI2CDMABuf_va)
	{
#ifdef CONFIG_ARM64	
		dma_free_coherent(&(client->dev), 4096, TfaI2CDMABuf_va, TfaI2CDMABuf_pa);
		TfaI2CDMABuf_va = NULL;
		TfaI2CDMABuf_pa = 0;
#else
		dma_free_coherent(NULL, 4096, TfaI2CDMABuf_va, TfaI2CDMABuf_pa);
		TfaI2CDMABuf_va = NULL;
		TfaI2CDMABuf_pa = 0;
#endif
	}
    msleep(1);
    return 0;
}

/* read write implementation */
/* read one register */
ssize_t NXPSpk_read_byte(u8 addr, u16 *returnData)
{
    
    char cmd_buf = addr;
    char readData[2] = {0};
    int ret = 0;
    if (!new_client)
    {
        printk("NXPSpk_read_byte I2C client not initialized!!");
        return -1;
    }
    ret = i2c_master_send(new_client, &cmd_buf, 1);
    if (ret < 0)
    {
        printk("NXPSpk_read_byte read sends command error!!\n");
        return -1;
    }
    ret = i2c_master_recv(new_client, readData, 2);
    printk("wuzhipeng ret = %d, %d,%d\n",ret,readData[0],readData[1]);
    if (ret < 0)
    {
        printk("NXPSpk_read_byte reads recv data error!!\n");
        return -1;
    }
    *returnData = (readData[0]<<8) | readData[1];
    /* printk("addr 0x%x data 0x%x\n", addr, readData); */
    return 0;
}

/* write register */
ssize_t NXPExt_write_byte(u8 addr, u8 writeData)
{
    char write_data[2] = { 0 };
    int ret = 0;
    if (!new_client)
    {
        printk("I2C client not initialized!!");
        return -1;
    }
    write_data[0] = addr;   /* ex. 0x01 */
    write_data[1] = writeData;
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0)
    {
        printk("write sends command error!!");
        return -1;
    }
    /* printk("addr 0x%x data 0x%x\n", addr, writeData); */
    return 0;
}


static int NXPExtSpk_register()
{
    printk("NXPExtSpk_register \n");
    i2c_register_board_info(TFA_I2C_CHANNEL, &nxpExt_dev, 1);
    if (i2c_add_driver(&NXPExtSpk_i2c_driver))
    {
        printk("fail to add device into i2c");
        return -1;
    }
    return 0;
}


bool NXPExtSpk_Register(void)
{
    printk("NXPExtSpk_Register \n");
    NXPExtSpk_register();
    return true;
}

void AudDrv_NXPSpk_Init(void)
{
    printk("Set GPIO for AFE I2S output to external DAC \n");
    pinctrl_select_state(spactrl, spa_nxplrck_mode);
    pinctrl_select_state(spactrl, spa_nxpdatai_mode);
    pinctrl_select_state(spactrl, spa_nxpbck_mode);
}

/*****************************************************************************
 * FILE OPERATION FUNCTION
 *  AudDrv_nxpspk_ioctl
 *
 * DESCRIPTION
 *  IOCTL Msg handle
 *
 *****************************************************************************
 */
static long AudDrv_nxpspk_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    /* printk("AudDrv_nxpspk_ioctl cmd = 0x%x arg = %lu\n", cmd, arg); */

    switch (cmd)
    {
        default:
        {
            /* printk("AudDrv_nxpspk_ioctl Fail command: %x\n", cmd); */
            ret = -1;
            break;
        }
    }
    return ret;
}

static int AudDrv_nxpspk_probe(struct platform_device *dev)
{
    int ret = 0;
    printk("AudDrv_nxpspk_probe \n");

    if (ret < 0)
    {
        printk("AudDrv_nxpspk_probe request_irq MT6582_AP_BT_CVSD_IRQ_LINE Fail \n");
    }
    NXPExtSpk_Register();
    mtksmartpa_gpio_init(dev);
    AudDrv_NXPSpk_Init();
    memset((void *)WriteBuffer, 0, RW_BUFFER_LENGTH);
    memset((void *)ReadBuffer, 0, RW_BUFFER_LENGTH);

    printk("-AudDrv_nxpspk_probe \n");
    return 0;
}

static int AudDrv_nxpspk_open(struct inode *inode, struct file *fp)
{
    return 0;
}

static int nxp_i2c_master_send(const struct i2c_client *client, const char *buf, int count)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;

	msg.timing = I2C_MASTER_CLOCK;

	if(count <= 8)
	{
		msg.addr = ((client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG));
		msg.ext_flag = client->ext_flag;
	}
	else
	{
		msg.addr = (client->addr & I2C_MASK_FLAG);
		msg.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG);
	}

	msg.flags = 0;

	msg.len = count;
	msg.buf = (char *)buf;

	ret = i2c_transfer(adap, &msg, 1);

	/*
	 * If everything went ok (i.e. 1 msg transmitted), return #bytes
	 * transmitted, else error code.
	 */
	return (ret == 1) ? count : ret;
}

static int nxp_i2c_master_recv(const struct i2c_client *client, char *buf, int count)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;

	msg.timing = I2C_MASTER_CLOCK;
	msg.flags = 0;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = (char *)buf;

	if(count <= 8)
	{
		msg.addr = ((client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG));
		msg.ext_flag = client->ext_flag;
	}
	else
	{
		msg.addr = (client->addr & I2C_MASK_FLAG);
		msg.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG);
	}

	ret = i2c_transfer(adap, &msg, 1);

	/*
	 * If everything went ok (i.e. 1 msg received), return #bytes received,
	 * else error code.
	 */
	return (ret == 1) ? count : ret;
}

static ssize_t AudDrv_nxpspk_write(struct file *fp, const char __user *data, size_t count, loff_t *offset)

{
	int i = 0;
	int ret;
	char *tmp;

	//if (count > 8192)
	//	count = 8192;

	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;
	if (copy_from_user(tmp,data,count)) {
		kfree(tmp);
		return -EFAULT;
	}

	//NXP_INFO("i2c-dev: i2c-%d writing %zu bytes.\n", iminor(file->f_path.dentry->d_inode), count);

	for(i = 0;  i < count; i++)
	{
		TfaI2CDMABuf_va[i] = tmp[i];
	}

	if(count <= 8)
	{
	    ///new_client->addr = new_client->addr & I2C_MASK_FLAG;  //cruson
		ret = nxp_i2c_master_send(new_client,tmp,count);
	}
	else
	{
	    //new_client->addr = new_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG |I2C_ENEXT_FLAG;  //cruson
		ret = nxp_i2c_master_send(new_client, (u8 *)TfaI2CDMABuf_pa, count);
	}
	kfree(tmp);
	return ret;
}

static ssize_t AudDrv_nxpspk_read(struct file *fp,  char __user *data, size_t count, loff_t *offset)
{
	int i = 0;
	char *tmp;
	int ret;

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;

	//NXP_INFO("i2c-dev: i2c-%d reading %zu bytes.\n", iminor(file->f_path.dentry->d_inode), count);

	if(count <= 8)
	{
	    //new_client->addr = new_client->addr & I2C_MASK_FLAG;  //cruson
		ret = nxp_i2c_master_recv(new_client,tmp,count);
	}
	else
	{
	    //new_client->addr = new_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG |I2C_ENEXT_FLAG;  //cruson
		ret = nxp_i2c_master_recv(new_client,(u8 *)TfaI2CDMABuf_pa,count);
		for(i = 0; i < count; i++)
		{
			tmp[i] = TfaI2CDMABuf_va[i];
		}
	}

	if (ret >= 0)
		ret = copy_to_user(data,tmp,count)?-EFAULT:ret;
	kfree(tmp);
	return ret;
}

/**************************************************************************
 * STRUCT
 *  File Operations and misc device
 *
 **************************************************************************/

static struct file_operations AudDrv_nxpspk_fops =
{
    .owner   = THIS_MODULE,
    .open    = AudDrv_nxpspk_open,
    .unlocked_ioctl   = AudDrv_nxpspk_ioctl,
    .write   = AudDrv_nxpspk_write,
    .read    = AudDrv_nxpspk_read,
};

#ifdef CONFIG_MTK_NXP_TFA9890
static struct miscdevice AudDrv_nxpspk_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "smartpa_i2c",
    .fops = &AudDrv_nxpspk_fops,
};
#else
static struct miscdevice AudDrv_nxpspk_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "nxpspk",
    .fops = &AudDrv_nxpspk_fops,
};
#endif

/***************************************************************************
 * FUNCTION
 *  AudDrv_nxpspk_mod_init / AudDrv_nxpspk_mod_exit
 *
 * DESCRIPTION
 *  Module init and de-init (only be called when system boot up)
 *
 **************************************************************************/

#ifdef CONFIG_OF
static const struct of_device_id mtk_smart_pa_of_ids[] =
{
    { .compatible = "mediatek,mtksmartpa", },
    {}
};
#endif

static struct platform_driver AudDrv_nxpspk =
{
    .probe = AudDrv_nxpspk_probe,
    .driver = {
        .name = NXPEXTSPK_I2C_DEVNAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = mtk_smart_pa_of_ids,
#endif
    },
};

//static struct platform_device *AudDrv_NXPSpk_dev;

static int AudDrv_nxpspk_mod_init(void)
{		
    int ret = 0;
    printk("+AudDrv_nxpspk_mod_init\n");

#ifndef CONFIG_OF
    printk("platform_device_alloc \n");
    AudDrv_NXPSpk_dev = platform_device_alloc("AudioMTKNXPSPK", -1);
    if (!AudDrv_NXPSpk_dev)
    {
        return -ENOMEM;
    }

    printk("platform_device_add \n");

    ret = platform_device_add(AudDrv_NXPSpk_dev);
    if (ret != 0)
    {
        platform_device_put(AudDrv_NXPSpk_dev);
        return ret;
    }
#endif
    /* Register platform DRIVER */

    ret = platform_driver_register(&AudDrv_nxpspk);
    if (ret)
    {
        printk("AudDrv Fail:%d - Register DRIVER\n", ret);
        return ret;
    }
    /* register MISC device */
    if ((ret = misc_register(&AudDrv_nxpspk_device)))
    {
        printk("AudDrv_nxpspk_mod_init misc_register Fail:%d\n", ret);
        return ret;
    }

    printk("-AudDrv_nxpspk_mod_init\n");
    return 0;
}

static void AudDrv_nxpspk_mod_exit(void)
{
    printk("+AudDrv_nxpspk_mod_exit\n");

    printk("-AudDrv_nxpspk_mod_exit\n");
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(AUDDRV_NXPSPK_NAME);
MODULE_AUTHOR(AUDDRV_AUTHOR);

module_init(AudDrv_nxpspk_mod_init);
module_exit(AudDrv_nxpspk_mod_exit);

