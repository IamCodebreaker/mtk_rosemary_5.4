/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
*/

/*
 *
 * Filename:
 * ---------
 *    mtk_auxadc.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of AUXADC common code
 *
 * Author:
 * -------
 * Zhong Wang
 *
 */

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/seq_file.h>

#include <linux/uaccess.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <mt-plat/sync_write.h>
#include <mt-plat/mtk_devinfo.h>

#include "mtk_auxadc.h"
#include "mtk_auxadc_hw.h"

#include <linux/of.h>
#include <linux/of_address.h>

static struct clk *clk_auxadc;
#if defined(AUXADC_CLK_CTR)
static struct clk *clk_top_auxadc1;
static struct clk *clk_top_auxadc2;
static struct clk *clk_top_aux_tp;
#endif

void __iomem *auxadc_base;
void __iomem *auxadc_apmix_base;
#if defined(EFUSE_CALI)
void __iomem *auxadc_efuse_base;
#endif

#include <linux/clk.h>

#define READ_REGISTER_UINT16(reg) (*(unsigned short * const)(reg))
#define INREG16(x)          \
	READ_REGISTER_UINT16((unsigned short *)((void *)(x)))
#define DRV_Reg16(addr)             INREG16(addr)
#define DRV_Reg(addr)               DRV_Reg16(addr)

#if defined(AUXADC_SPM)
#define READ_REGISTER_UINT32(reg) (*(unsigned int * const)(reg))
#define INREG32(x)          READ_REGISTER_UINT32((unsigned int *)((void *)(x)))
#define DRV_Reg32(addr)             INREG32(addr)
#endif

/******************************************************/
#define DRV_ClearBits(addr, data) {\
unsigned short temp;\
temp = DRV_Reg16(addr);\
temp &=  ~(data);\
mt_reg_sync_writew(temp, addr);\
}

#define DRV_SetBits(addr, data) {\
unsigned short temp;\
temp = DRV_Reg16(addr);\
temp |= (data);\
mt_reg_sync_writew(temp, addr);\
}

#define DRV_SetData(addr, bitmask, value) {\
unsigned short temp;\
temp = (~(bitmask)) & DRV_Reg16(addr);\
temp |= (value);\
mt_reg_sync_writew(temp, addr);\
}

#define AUXADC_DRV_ClearBits16(addr, data)           DRV_ClearBits(addr, data)
#define AUXADC_DRV_SetBits16(addr, data)             DRV_SetBits(addr, data)
#define AUXADC_DRV_WriteReg16(addr, data)            \
		mt_reg_sync_writew(data, addr)
#define AUXADC_DRV_ReadReg16(addr)                   DRV_Reg(addr)
#define AUXADC_DRV_SetData16(addr, bitmask, value)   \
		DRV_SetData(addr, bitmask, value)
#if defined(AUXADC_SPM)
#define AUXADC_DRV_ReadReg32(addr)                   DRV_Reg32(addr)
#endif

#define AUXADC_CLR_BITS(BS, REG)     {\
unsigned int temp;\
temp = DRV_Reg32(REG);\
temp &=  ~(BS);\
mt_reg_sync_writel(temp, REG);\
}

#define AUXADC_SET_BITS(BS, REG)     {\
unsigned int temp;\
temp = DRV_Reg32(REG);\
temp |= (BS);\
mt_reg_sync_writel(temp, REG);\
}

#define VOLTAGE_FULL_RANGE  1500	/* VA voltage */
#define AUXADC_PRECISE      4096	/* 12 bits */
/******************************************************/


/*****************************************************************************
 * Integrate with NVRAM
 */
#define AUXADC_CALI_DEVNAME    "mtk-adc-cali"
#define TAG                    "[AUXADC]"
#define TEST_ADC_CALI_PRINT _IO('k', 0)
#define SET_ADC_CALI_Slop   _IOW('k', 1, int)
#define SET_ADC_CALI_Offset _IOW('k', 2, int)
#define SET_ADC_CALI_Cal    _IOW('k', 3, int)
#define ADC_CHANNEL_READ    _IOW('k', 4, int)

struct adc_info {
	char channel_name[64];
	int channel_number;
	int reserve1;
	int reserve2;
	int reserve3;
};

static struct adc_info g_adc_info[ADC_CHANNEL_MAX];
static int auxadc_cali_slop[ADC_CHANNEL_MAX] = { 0 };
static int auxadc_cali_offset[ADC_CHANNEL_MAX] = { 0 };

static bool g_AUXADC_Cali;

static int auxadc_cali_cal[1] = { 0 };
static int auxadc_in_data[2] = { 1, 1 };
static int auxadc_out_data[2] = { 1, 1 };

static DEFINE_MUTEX(auxadc_mutex);
static DEFINE_MUTEX(mutex_get_cali_value);
static int adc_auto_set;
#if !defined(CONFIG_AUXADC_NOT_CONTROL_APMIXED_BASE)
static int adc_rtp_set = 1;
#endif

static dev_t auxadc_cali_devno;
static int auxadc_cali_major;
static struct cdev *auxadc_cali_cdev;
static struct class *auxadc_cali_class;

static struct task_struct *thread;
static int g_start_debug_thread;

static int g_adc_init_flag;

static u32 cali_reg;
static s32 cali_oe;
static s32 cali_ge;
static u32 cali_ge_a;
static u32 cali_oe_a;
static u32 gain;

static void mt_auxadc_update_cali(void)
{
	cali_oe = 0;
	cali_ge = 0;

#if defined(EFUSE_CALI)
#if defined(AUXADC_INDEX)
	cali_reg = get_devinfo_with_index(AUXADC_INDEX);
#else
	cali_reg = (*(unsigned int *const)(ADC_CALI_EN_A_REG));
#endif

	if (((cali_reg & ADC_CALI_EN_A_MASK) >> ADC_CALI_EN_A_SHIFT) != 0) {
		cali_oe_a = (cali_reg & ADC_OE_A_MASK) >> ADC_OE_A_SHIFT;
		cali_ge_a = ((cali_reg & ADC_GE_A_MASK) >> ADC_GE_A_SHIFT);
		/* In sw implement guide, ge should div 4096.
		 * But we don't do that now due to it will multi 4096 later
		 */
		cali_ge = cali_ge_a - 512;
		cali_oe = cali_oe_a - 512;
		gain = 1 + cali_ge;
		/* In sw implement guide, gain = 1 + GE = 1 + cali_ge / 4096,
		 * we doen't use the variable here
		 */
	}
#endif
}

static void mt_auxadc_get_cali_data(unsigned int rawdata, int data[4],
			bool enable_cali)
{

	if (enable_cali == true) {
#if defined(EFUSE_CALI)
		/* In sw implement guide, 4096 * gain = 4096 * (1 + GE)
		 * = 4096 * (1 + cali_ge / 4096) = 4096 + cali_ge)
		 */
		rawdata = rawdata - cali_oe;
		/* convert to volt */
		data[0] = (rawdata * 1500 / (4096 + cali_ge)) / 1000;
		/* convert to mv, need multiply 10 */
		data[1] = (rawdata * 150 / (4096 + cali_ge)) % 100;
		/* data[2] provide high precision mv */
		data[2] = (rawdata * 1500 / (4096 + cali_ge)) % 1000;
#else
		data[0] = (rawdata * 150 / AUXADC_PRECISE / 100);
		data[1] = ((rawdata * 150 / AUXADC_PRECISE) % 100);
		data[2] = ((rawdata * 1500 / AUXADC_PRECISE) % 1000);
#endif
	} else {
		data[0] = (rawdata * 150 / AUXADC_PRECISE / 100);
		data[1] = ((rawdata * 150 / AUXADC_PRECISE) % 100);
		data[2] = ((rawdata * 1500 / AUXADC_PRECISE) % 1000);
	}
}


#if defined(CONFIG_AUXADC_NOT_CONTROL_APMIXED_BASE)
static void mt_auxadc_disable_penirq(void)
{
}
#else
static u16 mt_tpd_read_adc(u16 pos)
{
	AUXADC_DRV_SetBits16((u16 *)AUXADC_TP_ADDR, pos);
	AUXADC_DRV_SetBits16((u16 *)AUXADC_TP_CON0, 0x01);
	/* wait for write finish */
	while (0x01 & AUXADC_DRV_ReadReg16((u16 *)AUXADC_TP_CON0))
		pr_debug(TAG "AUXADC_TP_CON0 waiting.\n");
	return AUXADC_DRV_ReadReg16((u16 *)AUXADC_TP_DATA0);
}

static void mt_auxadc_disable_penirq(void)
{
	if (adc_rtp_set) {
		adc_rtp_set = 0;
		AUXADC_DRV_SetBits16((u16 *)AUXADC_CON_RTP, 1);
		/* Turn off PENIRQ detection circuit */
		AUXADC_DRV_SetBits16((u16 *)AUXADC_TP_CMD, 1);
		/* run once touch function */
		mt_tpd_read_adc(TP_CMD_ADDR_X);
	}
}
#endif

/* HAL API */
static int IMM_auxadc_GetOneChannelValue(int dwChannel, int data[4],
		int *rawdata)
{
	unsigned int channel[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0 };
	/* int idle_count = 0; */
	int data_ready_count = 0;
	int ret = 0;

	mutex_lock(&mutex_get_cali_value);

	if (clk_auxadc) {
		ret = clk_prepare_enable(clk_auxadc);
		if (ret) {
			pr_err(TAG "hwEnableClock AUXADC failed.");
			mutex_unlock(&mutex_get_cali_value);
			return -1;
		}
	} else {
		pr_err(TAG "hwEnableClock AUXADC failed.");
		mutex_unlock(&mutex_get_cali_value);
		return -1;
	}

	if (dwChannel == PAD_AUX_XP || dwChannel == PAD_AUX_YM)
		mt_auxadc_disable_penirq();

	/* step1 check con2 if auxadc is busy */
	/* Remove because hw/sw may access in the same time */
	#if 0
	while (AUXADC_DRV_ReadReg16((u16 *)AUXADC_CON2) & 0x01) {
		mdelay(1);
		idle_count++;
		if (idle_count > 30) {
			pr_warn(TAG "wait for auxadc idle time out\n");
			mutex_unlock(&mutex_get_cali_value);
			return -1;
		}
	}
	#endif

	/* step2 clear bit */
	if (adc_auto_set == 0) {
		/* clear bit */
#if defined(CONFIG_MACH_MT6739)
		AUXADC_DRV_ClearBits16((u16 *)AUXADC_CON1, (1 << dwChannel));
#else
		AUXADC_DRV_WriteReg16((u16 *)AUXADC_CON1_CLR, (1 << dwChannel));
#endif
	}

	/* step3 read channel and make sure old ready bit == 0 */
	while (AUXADC_DRV_ReadReg16(AUXADC_DAT0 + dwChannel * 0x04) &
			(1 << 12)) {
		pr_debug(TAG "wait for channel[%d] ready\n", dwChannel);
		mdelay(1);
		data_ready_count++;
		if (data_ready_count > 30) {
			/* wait for idle time out */
			pr_err(TAG "wait for channel[%d] ready timeout\n",
			       dwChannel);
			mutex_unlock(&mutex_get_cali_value);
			return -2;
		}
	}

	/* step4 set bit to trigger sample */
	if (adc_auto_set == 0)
#if defined(CONFIG_MACH_MT6739)
		AUXADC_DRV_SetBits16((u16 *)AUXADC_CON1, (1 << dwChannel));
#else
		AUXADC_DRV_WriteReg16((u16 *)AUXADC_CON1_SET, (1 << dwChannel));
#endif
	/* step5 read channel and make sure ready bit == 1
	 * we must dealay here for hw sample cahnnel data
	 */
	udelay(25);
	while (0 == (AUXADC_DRV_ReadReg16(AUXADC_DAT0 + dwChannel * 0x04) &
			(1 << 12))) {
		mdelay(1);
		data_ready_count++;

		if (data_ready_count > 30) {
			/* wait for idle time out */
			pr_err(TAG "wait for channel[%d] data ready timeout\n",
					dwChannel);
			mutex_unlock(&mutex_get_cali_value);
			return -3;
		}
	}

	/* step6 read data */
	channel[dwChannel] = AUXADC_DRV_ReadReg16(AUXADC_DAT0 + dwChannel *
			0x04) & 0x0FFF;

	if (rawdata != NULL)
		*rawdata = channel[dwChannel];
	mt_auxadc_get_cali_data(channel[dwChannel], data, true);

	if (clk_auxadc) {
		clk_disable_unprepare(clk_auxadc);
	} else {
		pr_err(TAG "hwdisableClock AUXADC failed.");
		mutex_unlock(&mutex_get_cali_value);
		return -1;
	}

	mutex_unlock(&mutex_get_cali_value);

	return ret;

}

/* 1v == 1000000 uv */
/* this function voltage Unit is uv */
static int IMM_auxadc_GetOneChannelValue_Cali(int Channel, int *voltage)
{
	int ret = 0, data[4], rawvalue;
	u_int64_t temp_vol;
	int tmp;

	ret = IMM_auxadc_GetOneChannelValue(Channel, data, &rawvalue);
	if (ret) {
		pr_err(TAG "get raw value error %d\n", ret);
		return -1;
	}
	tmp = data[0] * 1000 + data[2];
	temp_vol = (u_int64_t) tmp * 1000;
	*voltage = temp_vol;
	return 0;
}

static void mt_auxadc_cal_prepare(void)
{
	/* no voltage calibration */
}

#if defined(CONFIG_AUXADC_NEED_POWER_ON)
static void mt_auxadc_power_on(void)
{
	/* power on ADC */
	AUXADC_DRV_SetBits16((u16 *)AUXADC_MISC, 1 << 14);
}
#else
static void mt_auxadc_power_on(void)
{
}
#endif

void mt_auxadc_hal_init(struct platform_device *dev)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, AUXADC_APMIX_NODE);
	if (node) {
		/* Setup IO addresses */
		auxadc_apmix_base = of_iomap(node, 0);
		pr_debug(TAG "auxadc_apmix_base=0x%p\n", auxadc_apmix_base);
	} else
		pr_err(TAG "auxadc_apmix_base error\n");

	node = of_find_compatible_node(NULL, NULL, AUXADC_NODE);
	if (!node)
		pr_err(TAG "find node failed\n");

	auxadc_base = of_iomap(node, 0);
	if (!auxadc_base)
		pr_err(TAG "base failed\n");

#if defined(EFUSE_CALI) && !defined(AUXADC_INDEX)
	node = of_find_compatible_node(NULL, NULL, "mediatek,efusec");
	if (!node)
		pr_err(TAG "find node failed\n");

	auxadc_efuse_base = of_iomap(node, 0);
	if (!auxadc_efuse_base)
		pr_err(TAG "auxadc_efuse_base base failed\n");

	pr_debug(TAG "auxadc_efuse_base:0x%p\n", auxadc_efuse_base);
#endif
	pr_debug(TAG "auxadc:0x%p\n", auxadc_base);

	mt_auxadc_cal_prepare();
	mt_auxadc_power_on();
}

static void mt_auxadc_hal_suspend(void)
{
	pr_debug(TAG "******** MT auxadc driver suspend!! ********\n");
#if defined(AUXADC_SPM)
	AUXADC_SET_BITS((0x3 << 6) | (0x3 << 16), AUXADC_TS_X_BUFFER);
	AUXADC_CLR_BITS(0xffff << 0, AUXADC_CON1);
#endif

#if !defined(AUXADC_CLOCK_BY_SPM)
	pr_debug(TAG "auxadc suspend disable clock.\n");
	clk_disable_unprepare(clk_auxadc);
#if defined(AUXADC_CLK_CTR)
	clk_disable_unprepare(clk_top_auxadc1);
	clk_disable_unprepare(clk_top_auxadc2);
	clk_disable_unprepare(clk_top_aux_tp);
#endif
#endif

}

static void mt_auxadc_hal_resume(void)
{
#if !defined(AUXADC_CLOCK_BY_SPM)
	int ret = 0;
#endif

	pr_debug(TAG "******** MT auxadc driver resume!! ********\n");
#if !defined(AUXADC_CLOCK_BY_SPM)
	ret = clk_prepare_enable(clk_auxadc);
	if (ret)
		pr_err(TAG "auxadc enable auxadc clk failed.");
#if defined(AUXADC_CLK_CTR)
	ret = clk_prepare_enable(clk_top_auxadc1);
	if (ret)
		pr_err(TAG "auxadc enable auxadc1 clk failed.");

	ret = clk_prepare_enable(clk_top_auxadc2);
	if (ret)
		pr_err(TAG "auxadc enable auxadc2 clk failed.");

	ret = clk_prepare_enable(clk_top_aux_tp);
	if (ret)
		pr_err(TAG "auxadc enable aux_tp clk failed.");
#endif
#endif
	mt_auxadc_power_on();
#if defined(AUXADC_SPM)
	AUXADC_CLR_BITS((0x3 << 6) | (0x3 << 16), AUXADC_TS_X_BUFFER);
#endif
}

static int mt_auxadc_dump_register(char *buf)
{
	pr_debug(TAG "AUXADC_CON0=%x AUXADC_CON1=%x AUXADC_CON2=%x\n",
		*(u16 *)AUXADC_CON0, *(u16 *)AUXADC_CON1, *(u16 *)AUXADC_CON2);

	return sprintf(buf, "AUXADC_CON0:%x\nAUXADC_CON1:%x\nAUXADC_CON2:%x\n",
		       *(u16 *)AUXADC_CON0, *(u16 *)AUXADC_CON1,
		       *(u16 *)AUXADC_CON2);
}

int IMM_IsAdcInitReady(void)
{
	return g_adc_init_flag;
}

int IMM_get_adc_channel_num(char *channel_name, int len)
{
	unsigned int i = 0;
	int ret = 0;

	if (channel_name == NULL) {
		pr_err(TAG "error: channel_name is NULL!\n");
		return -1;
	}

	pr_debug(TAG "name=%s, name_len=%d\n", channel_name, len);
	for (i = 0; i < ADC_CHANNEL_MAX; i++) {
		ret = strncmp(channel_name, g_adc_info[i].channel_name, len);
		if (!ret)
			return g_adc_info[i].channel_number;
	}
	pr_err(TAG "find channel number failed\n");
	return -1;
}

int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata)
{
	return IMM_auxadc_GetOneChannelValue(dwChannel, data, rawdata);
}

/* 1v == 1000000 uv
 * this function voltage Unit is uv
 */
int IMM_GetOneChannelValue_Cali(int Channel, int *voltage)
{
	return IMM_auxadc_GetOneChannelValue_Cali(Channel, voltage);
}

static long auxadc_cali_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int i = 0, ret = 0;
	long *user_data_addr;
	long *nvram_data_addr;

	mutex_lock(&auxadc_mutex);

	switch (cmd) {
	case TEST_ADC_CALI_PRINT:
		g_AUXADC_Cali = false;
		break;

	case SET_ADC_CALI_Slop:
		nvram_data_addr = (long *)arg;
		ret = copy_from_user(auxadc_cali_slop, nvram_data_addr, 36);
		g_AUXADC_Cali = false;
		/* Protection */
		for (i = 0; i < ADC_CHANNEL_MAX; i++) {
			if ((*(auxadc_cali_slop + i) == 0) || (*
					(auxadc_cali_slop + i) == 1))
				*(auxadc_cali_slop + i) = 1000;
		}
		for (i = 0; i < ADC_CHANNEL_MAX; i++)
			pr_debug(TAG "auxadc_cali_slop[%d] = %d\n",
					i, *(auxadc_cali_slop + i));
		break;

	case SET_ADC_CALI_Offset:
		nvram_data_addr = (long *)arg;
		ret = copy_from_user(auxadc_cali_offset, nvram_data_addr, 36);
		g_AUXADC_Cali = false;
		for (i = 0; i < ADC_CHANNEL_MAX; i++)
			pr_debug(TAG "auxadc_cali_offset[%d] = %d\n",
					i, *(auxadc_cali_offset + i));
		break;

	case SET_ADC_CALI_Cal:
		nvram_data_addr = (long *)arg;
		ret = copy_from_user(auxadc_cali_cal, nvram_data_addr, 4);
		/* enable calibration after setting AUXADC_CALI_Cal */
		g_AUXADC_Cali = true;
		if (auxadc_cali_cal[0] == 1)
			g_AUXADC_Cali = true;
		else
			g_AUXADC_Cali = false;
		break;

	case ADC_CHANNEL_READ:
		g_AUXADC_Cali = false;
		user_data_addr = (long *)arg;
		ret = copy_from_user(auxadc_in_data, user_data_addr, 8);
		ret = copy_to_user(user_data_addr, auxadc_out_data, 8);
		pr_debug(TAG "AUXADC Channel %d * %d times = %d\n",
				auxadc_in_data[0],
			 auxadc_in_data[1], auxadc_out_data[0]);
		break;

	default:
		g_AUXADC_Cali = false;
		break;
	}

	mutex_unlock(&auxadc_mutex);

	return 0;
}

#ifdef CONFIG_COMPAT
static long compat_auxadc_unlocked_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{

	if (!filp->f_op || !filp->f_op->unlocked_ioctl) {
		pr_err(TAG "copat unlocked ioctl fail.\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case TEST_ADC_CALI_PRINT:
	case SET_ADC_CALI_Slop:
	case SET_ADC_CALI_Offset:
	case SET_ADC_CALI_Cal:
	case ADC_CHANNEL_READ:
		return filp->f_op->unlocked_ioctl(filp, cmd,
			(unsigned long)compat_ptr(arg));
	default:
		pr_err(TAG "compat_ion_ioctl : No such command!! 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}
}
#endif

static int auxadc_cali_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int auxadc_cali_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations auxadc_cali_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = auxadc_cali_unlocked_ioctl,
	.open = auxadc_cali_open,
	.release = auxadc_cali_release,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_auxadc_unlocked_ioctl,
#endif
};

/*
 * Create File For EM : AUXADC_Channel_X_Slope/Offset
 */
#if ADC_CHANNEL_MAX > 0
static ssize_t show_AUXADC_Channel_0_Slope(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_slop + 0));
	pr_debug(TAG "[EM] AUXADC_Channel_0_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_0_Slope(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_0_Slope, 0664, show_AUXADC_Channel_0_Slope,
		   store_AUXADC_Channel_0_Slope);
static ssize_t show_AUXADC_Channel_0_Offset(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_offset + 0));
	pr_debug(TAG "[EM] AUXADC_Channel_0_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_0_Offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_0_Offset, 0664, show_AUXADC_Channel_0_Offset,
		   store_AUXADC_Channel_0_Offset);
#endif


#if ADC_CHANNEL_MAX > 1
static ssize_t show_AUXADC_Channel_1_Slope(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_slop + 1));
	pr_debug(TAG "[EM] AUXADC_Channel_1_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_1_Slope(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_1_Slope, 0664, show_AUXADC_Channel_1_Slope,
		   store_AUXADC_Channel_1_Slope);
static ssize_t show_AUXADC_Channel_1_Offset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_offset + 1));
	pr_debug(TAG "[EM] AUXADC_Channel_1_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_1_Offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_1_Offset, 0664, show_AUXADC_Channel_1_Offset,
		   store_AUXADC_Channel_1_Offset);
#endif


#if ADC_CHANNEL_MAX > 2
static ssize_t show_AUXADC_Channel_2_Slope(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_slop + 2));
	pr_debug(TAG "[EM] AUXADC_Channel_2_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_2_Slope(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_2_Slope, 0664, show_AUXADC_Channel_2_Slope,
		   store_AUXADC_Channel_2_Slope);
static ssize_t show_AUXADC_Channel_2_Offset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_offset + 2));
	pr_debug(TAG "[EM] AUXADC_Channel_2_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_2_Offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_2_Offset, 0664, show_AUXADC_Channel_2_Offset,
		   store_AUXADC_Channel_2_Offset);
#endif


#if ADC_CHANNEL_MAX > 3
static ssize_t show_AUXADC_Channel_3_Slope(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_slop + 3));
	pr_debug(TAG "[EM] AUXADC_Channel_3_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_3_Slope(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_3_Slope, 0664, show_AUXADC_Channel_3_Slope,
		   store_AUXADC_Channel_3_Slope);
static ssize_t show_AUXADC_Channel_3_Offset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_offset + 3));
	pr_debug(TAG "[EM] AUXADC_Channel_3_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_3_Offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_3_Offset, 0664, show_AUXADC_Channel_3_Offset,
		   store_AUXADC_Channel_3_Offset);
#endif


#if ADC_CHANNEL_MAX > 4
static ssize_t show_AUXADC_Channel_4_Slope(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_slop + 4));
	pr_debug(TAG "[EM] AUXADC_Channel_4_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_4_Slope(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_4_Slope, 0664, show_AUXADC_Channel_4_Slope,
		   store_AUXADC_Channel_4_Slope);
static ssize_t show_AUXADC_Channel_4_Offset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_offset + 4));
	pr_debug(TAG "[EM] AUXADC_Channel_4_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_4_Offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_4_Offset, 0664, show_AUXADC_Channel_4_Offset,
		   store_AUXADC_Channel_4_Offset);
#endif


#if ADC_CHANNEL_MAX > 5
static ssize_t show_AUXADC_Channel_5_Slope(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_slop + 5));
	pr_debug(TAG "[EM] AUXADC_Channel_5_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_5_Slope(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_5_Slope, 0664, show_AUXADC_Channel_5_Slope,
		   store_AUXADC_Channel_5_Slope);
static ssize_t show_AUXADC_Channel_5_Offset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_offset + 5));
	pr_debug(TAG "[EM] AUXADC_Channel_5_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_5_Offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_5_Offset, 0664, show_AUXADC_Channel_5_Offset,
		   store_AUXADC_Channel_5_Offset);
#endif


#if ADC_CHANNEL_MAX > 6
static ssize_t show_AUXADC_Channel_6_Slope(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_slop + 6));
	pr_debug(TAG "[EM] AUXADC_Channel_6_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_6_Slope(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_6_Slope, 0664, show_AUXADC_Channel_6_Slope,
		   store_AUXADC_Channel_6_Slope);
static ssize_t show_AUXADC_Channel_6_Offset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_offset + 6));
	pr_debug(TAG "[EM] AUXADC_Channel_6_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_6_Offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_6_Offset, 0664, show_AUXADC_Channel_6_Offset,
		   store_AUXADC_Channel_6_Offset);
#endif


#if ADC_CHANNEL_MAX > 7
static ssize_t show_AUXADC_Channel_7_Slope(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_slop + 7));
	pr_debug(TAG "[EM] AUXADC_Channel_7_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_7_Slope(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_debug(TAG "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_7_Slope, 0664, show_AUXADC_Channel_7_Slope,
		   store_AUXADC_Channel_7_Slope);
static ssize_t show_AUXADC_Channel_7_Offset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = (*(auxadc_cali_offset + 7));
	pr_debug(TAG "[EM] AUXADC_Channel_7_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_7_Offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
