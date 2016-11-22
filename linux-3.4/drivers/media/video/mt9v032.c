/*
 * Driver for MT9V032 CMOS Image Sensor from Micron
 *
 * Copyright (C) 2010, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 * Based on the MT9M001 driver,
 *
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

//ankt
#include <linux/clk.h>

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>
#include <linux/module.h>

#include <media/mt9v032.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include <linux/proc_fs.h>	/* Necessary because we use the proc fs */
#include <asm/uaccess.h>	/* for copy_from_user */

#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-sunxi.h>

#include "sunxi-vfe/csi_cci/cci_helper.h"

#include "sunxi-vfe/device/camera_cfg.h"
#include "sunxi-vfe/vfe_subdev.h"

#define MT9V032_PIXEL_ARRAY_HEIGHT			492
#define MT9V032_PIXEL_ARRAY_WIDTH			782

#define MT9V032_CHIP_VERSION				0x00
#define		MT9V032_CHIP_ID_REV1			0x1311
#define		MT9V032_CHIP_ID_REV3			0x1313
#define		MT9V034_CHIP_ID_REV1			0x1324
#define MT9V032_COLUMN_START				0x01
#define		MT9V032_COLUMN_START_MIN		1
#define		MT9V032_COLUMN_START_DEF		1
#define		MT9V032_COLUMN_START_MAX		752
#define MT9V032_ROW_START				0x02
#define		MT9V032_ROW_START_MIN			4
#define		MT9V032_ROW_START_DEF			5
#define		MT9V032_ROW_START_MAX			482
#define MT9V032_WINDOW_HEIGHT				0x03
#define		MT9V032_WINDOW_HEIGHT_MIN		1
#define		MT9V032_WINDOW_HEIGHT_DEF		480
#define		MT9V032_WINDOW_HEIGHT_MAX		480
#define MT9V032_WINDOW_WIDTH				0x04
#define		MT9V032_WINDOW_WIDTH_MIN		1
#define		MT9V032_WINDOW_WIDTH_DEF		752
#define		MT9V032_WINDOW_WIDTH_MAX		752
#define MT9V032_HORIZONTAL_BLANKING			0x05
#define		MT9V032_HORIZONTAL_BLANKING_MIN		43
#define		MT9V034_HORIZONTAL_BLANKING_MIN		61
#define		MT9V032_HORIZONTAL_BLANKING_MAX		1023
#define MT9V032_VERTICAL_BLANKING			0x06
#define		MT9V032_VERTICAL_BLANKING_MIN		4
#define		MT9V034_VERTICAL_BLANKING_MIN		2
#define		MT9V032_VERTICAL_BLANKING_MAX		3000
#define		MT9V034_VERTICAL_BLANKING_MAX		32288
#define MT9V032_CHIP_CONTROL				0x07
#define		MT9V032_CHIP_CONTROL_MASTER_MODE	(1 << 3)
#define		MT9V032_CHIP_CONTROL_DOUT_ENABLE	(1 << 7)
#define		MT9V032_CHIP_CONTROL_SEQUENTIAL		(1 << 8)
#define MT9V032_SHUTTER_WIDTH1				0x08
#define MT9V032_SHUTTER_WIDTH2				0x09
#define MT9V032_SHUTTER_WIDTH_CONTROL			0x0a
#define MT9V032_TOTAL_SHUTTER_WIDTH			0x0b
#define		MT9V032_TOTAL_SHUTTER_WIDTH_MIN		1
#define		MT9V034_TOTAL_SHUTTER_WIDTH_MIN		0
#define		MT9V032_TOTAL_SHUTTER_WIDTH_DEF		480
#define		MT9V032_TOTAL_SHUTTER_WIDTH_MAX		32767
#define		MT9V034_TOTAL_SHUTTER_WIDTH_MAX		32765
#define MT9V032_RESET					0x0c
#define MT9V032_READ_MODE				0x0d
#define		MT9V032_READ_MODE_ROW_BIN_MASK		(3 << 0)
#define		MT9V032_READ_MODE_ROW_BIN_SHIFT		0
#define		MT9V032_READ_MODE_COLUMN_BIN_MASK	(3 << 2)
#define		MT9V032_READ_MODE_COLUMN_BIN_SHIFT	2
#define		MT9V032_READ_MODE_ROW_FLIP		(1 << 4)
#define		MT9V032_READ_MODE_COLUMN_FLIP		(1 << 5)
#define		MT9V032_READ_MODE_DARK_COLUMNS		(1 << 6)
#define		MT9V032_READ_MODE_DARK_ROWS		(1 << 7)
#define MT9V032_PIXEL_OPERATION_MODE			0x0f
#define		MT9V034_PIXEL_OPERATION_MODE_HDR	(1 << 0)
#define		MT9V034_PIXEL_OPERATION_MODE_COLOR	(1 << 1)
#define		MT9V032_PIXEL_OPERATION_MODE_COLOR	(1 << 2)
#define		MT9V032_PIXEL_OPERATION_MODE_HDR	(1 << 6)
#define MT9V032_ANALOG_GAIN				0x35
#define		MT9V032_ANALOG_GAIN_MIN			16
#define		MT9V032_ANALOG_GAIN_DEF			16
#define		MT9V032_ANALOG_GAIN_MAX			64
#define MT9V032_MAX_ANALOG_GAIN				0x36
#define		MT9V032_MAX_ANALOG_GAIN_MAX		127
#define MT9V032_FRAME_DARK_AVERAGE			0x42
#define MT9V032_DARK_AVG_THRESH				0x46
#define		MT9V032_DARK_AVG_LOW_THRESH_MASK	(255 << 0)
#define		MT9V032_DARK_AVG_LOW_THRESH_SHIFT	0
#define		MT9V032_DARK_AVG_HIGH_THRESH_MASK	(255 << 8)
#define		MT9V032_DARK_AVG_HIGH_THRESH_SHIFT	8
#define MT9V032_ROW_NOISE_CORR_CONTROL			0x70
#define		MT9V034_ROW_NOISE_CORR_ENABLE		(1 << 0)
#define		MT9V034_ROW_NOISE_CORR_USE_BLK_AVG	(1 << 1)
#define		MT9V032_ROW_NOISE_CORR_ENABLE		(1 << 5)
#define		MT9V032_ROW_NOISE_CORR_USE_BLK_AVG	(1 << 7)
#define MT9V032_PIXEL_CLOCK				0x74
#define MT9V034_PIXEL_CLOCK				0x72
#define		MT9V032_PIXEL_CLOCK_INV_LINE		(1 << 0)
#define		MT9V032_PIXEL_CLOCK_INV_FRAME		(1 << 1)
#define		MT9V032_PIXEL_CLOCK_XOR_LINE		(1 << 2)
#define		MT9V032_PIXEL_CLOCK_CONT_LINE		(1 << 3)
#define		MT9V032_PIXEL_CLOCK_INV_PXL_CLK		(1 << 4)
#define MT9V032_TEST_PATTERN				0x7f
#define		MT9V032_TEST_PATTERN_DATA_MASK		(1023 << 0)
#define		MT9V032_TEST_PATTERN_DATA_SHIFT		0
#define		MT9V032_TEST_PATTERN_USE_DATA		(1 << 10)
#define		MT9V032_TEST_PATTERN_GRAY_MASK		(3 << 11)
#define		MT9V032_TEST_PATTERN_GRAY_NONE		(0 << 11)
#define		MT9V032_TEST_PATTERN_GRAY_VERTICAL	(1 << 11)
#define		MT9V032_TEST_PATTERN_GRAY_HORIZONTAL	(2 << 11)
#define		MT9V032_TEST_PATTERN_GRAY_DIAGONAL	(3 << 11)
#define		MT9V032_TEST_PATTERN_ENABLE		(1 << 13)
#define		MT9V032_TEST_PATTERN_FLIP		(1 << 14)
#define MT9V032_AEC_AGC_ENABLE				0xaf
#define		MT9V032_AEC_ENABLE			(1 << 0)
#define		MT9V032_AGC_ENABLE			(1 << 1)
#define MT9V034_AEC_MAX_SHUTTER_WIDTH			0xad
#define MT9V032_AEC_MAX_SHUTTER_WIDTH			0xbd
#define MT9V032_THERMAL_INFO				0xc1

#define VREF_POL          V4L2_MBUS_VSYNC_ACTIVE_HIGH
#define HREF_POL          V4L2_MBUS_HSYNC_ACTIVE_HIGH
#define CLK_POL           V4L2_MBUS_PCLK_SAMPLE_RISING

//struct vfe_dev *my_dev;
//struct pinctrl *my_pctrl;
//struct pinctrl *my_pctrl_0;

static int called=0;
bool asj = 0;
struct v4l2_subdev *subdev_apt;
struct mt9v032 {
	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_mbus_framefmt format;
	struct v4l2_rect crop;

	struct v4l2_ctrl_handler ctrls;

	struct mutex power_lock;
	int power_count;
	
	//struct device *dev;
	//struct pinctrl *pctrl;

	//ankt
	struct clk *clk;

	struct mt9v032_platform_data *pdata;
	u16 chip_control;
	u16 aec_agc;
};

//PROC=========================

#define PROCFS_MAX_SIZE		1024
#define PROCFS_NAME 		"asj"

static struct proc_dir_entry *Our_Proc_File;
static char procfs_buffer[PROCFS_MAX_SIZE];
static unsigned long procfs_buffer_size = 0;

struct i2c_client *my_client;
struct i2c_client *my_client_new;
static int mt9v032_read(struct i2c_client *client, const u8 reg);
static int mt9v032_write(struct i2c_client *client, const u8 reg, const u16 data);
int mt9v032_registered(struct v4l2_subdev *subdev1);


extern void vfe_muxto_i2c();
extern void vfe_muxto_csi();

short val = 0x0000;
int procfile_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data)
{
	int ret ;
	//int func = 3;
	//long unsigned int config_set;
	//char * dev_name = "mt9v032";
	//char * port_name_c = "PE12";
	//char * port_name_d = "PE13";
	//char *name = "twi2";
	//struct pinctrl *my_p = NULL;
	//devm_pinctrl_put(&client->adapter->dev.parent->pinctrl);
	//printk("\r\nmt9v032 dev name: %s\r\n",my_client->adapter->dev.kobj.name);
	//dev_set_name(&my_client->adapter->dev, "twi2");
	//printk("\r\nmt9v032 dev name: %s\r\n",my_client->adapter->dev.kobj.name);

	//my_p = devm_pinctrl_get_select(&my_client->adapter->dev, "default");
	//if (IS_ERR_OR_NULL(my_p)) {
	//	printk("mt9v032 request pinctrl handle for devicefailed!\n");
	//}
	//
	//
      	//config_set = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC,func);
        //pin_config_set(dev_name,port_name_c,config_set);
        //pin_config_set(dev_name,port_name_d,config_set);

	//devm_pinctrl_put(my_pctrl_0);
	//dev_set_name(&my_dev->pdev->dev,"twi2");
	//dev_set_name(&dev->pdev->dev,"csi%d",dev->id);
	//my_pctrl = devm_pinctrl_get_select(&my_dev->pdev->dev, "default");
	//if (IS_ERR_OR_NULL(twi_pctrl)) {
	//	printk("mt9v032 vip%d request pinctrl for device [%s] failed!\n", dev->id, dev_name(&my_dev->pdev->dev));
		//return -EINVAL;
	//}

	s32 data1 = mt9v032_read(my_client, MT9V032_CHIP_VERSION);
	data1 = mt9v032_read(my_client, MT9V032_PIXEL_OPERATION_MODE);
	//if(val == 0x0)
	//	val = 0x0100;
	//else
	//	val = 0;
	ret = mt9v032_write(my_client, MT9V032_PIXEL_OPERATION_MODE, 0);
	data1 = mt9v032_read(my_client, MT9V032_PIXEL_OPERATION_MODE);

	/*data1 = mt9v032_read(my_client, 0x47);
	data1 = mt9v032_read(my_client, 0x48);
	ret = mt9v032_write(my_client, 0x48, 0x007f);
	ret = mt9v032_write(my_client, 0x47, 0x0081);
	data1 = mt9v032_read(my_client, 0x47);
	data1 = mt9v032_read(my_client, 0x48);
*/

	//vfe_muxto_i2c();
	//mt9v032_registered(NULL);
	//devm_pinctrl_put(my_pctrl);
	//dev_set_name(&dev->pdev->dev,"csi%d",dev->id);
	//my_dev->pctrl = devm_pinctrl_get_select(&dev->pdev->dev, "default");
	//if (IS_ERR_OR_NULL(twi_pctrl)) {
	//	printk("mt9v032 vip%d request pinctrl for device [%s] failed!\n", dev->id, dev_name(&my_dev->pdev->dev));
		//return -EINVAL;
	//}

	return ret;
}

int procfile_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	if ( copy_from_user(procfs_buffer, buffer, procfs_buffer_size) ) {
		return -EFAULT;
	}

	return procfs_buffer_size;
}



//PROC=========================
static struct cci_driver cci_drv = {
	        .name = "mt9v032",//SENSOR_NAME,
	        .addr_width = 16,//CCI_BITS_16,
	        .data_width = 8,//CCI_BITS_8,
};

static void cci_device_release(struct device *dev)
{
	        return;
}
struct device my_cci_device_def =
{
	        .release = cci_device_release,
};

static struct mt9v032 *to_mt9v032(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9v032, subdev);
}

static int mt9v032_read(struct i2c_client *client, const u8 reg)
{

	//if(!called) {
	//	called = 1;
	//	vfe_muxto_i2c();
	//}
	s32 data = i2c_smbus_read_word_swapped(client, reg);
	//vfe_muxto_csi();
	dev_dbg(&client->dev, "%s: read 0x%04x from 0x%02x\n", __func__,
		data, reg);
	printk("%s: read 0x%04x from 0x%02x\n", __func__,
		data, reg);
	return data;
}

static int mt9v032_write(struct i2c_client *client, const u8 reg,
			 const u16 data)
{
	int ret;

	//if(!called) {
	//	called = 1;
		//vfe_muxto_i2c();
	//}
	dev_dbg(&client->dev, "%s: writing 0x%04x to 0x%02x\n", __func__,
		data, reg);
	printk("%s: writing 0x%04x to 0x%02x\n", __func__,
		data, reg);
	//return i2c_smbus_write_word_swapped(client, reg, data);
	//vfe_muxto_i2c();
	ret = i2c_smbus_write_word_swapped(client, reg, data);
	//vfe_muxto_csi();
	return ret;
}

static int mt9v032_set_chip_control(struct mt9v032 *mt9v032, u16 clear, u16 set)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mt9v032->subdev);
	u16 value = (mt9v032->chip_control & ~clear) | set;
	int ret;
#if 1
	ret = mt9v032_write(client, MT9V032_CHIP_CONTROL, value);
	if (ret < 0)
		return ret;
#endif

	mt9v032->chip_control = value;
	return 0;
}

static int
mt9v032_update_aec_agc(struct mt9v032 *mt9v032, u16 which, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mt9v032->subdev);
	u16 value = mt9v032->aec_agc;
	int ret;

	if (enable)
		value |= which;
	else
		value &= ~which;

	ret = mt9v032_write(client, MT9V032_AEC_AGC_ENABLE, value);
	if (ret < 0)
		return ret;

	mt9v032->aec_agc = value;
	return 0;
}

bool on_done = 0;
static int mt9v032_power_on(struct mt9v032 *mt9v032)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mt9v032->subdev);
	int ret;

	//ankt
	//if (mt9v032->pdata->set_clock) {
	//	mt9v032->pdata->set_clock(&mt9v032->subdev, 25000000);
	//	udelay(1);
	//}
	if(on_done == 1)
		return 0;
	
	//clk_set_rate(mt9v032->clk, 26600000);
	clk_prepare_enable(mt9v032->clk);
	udelay(1);

	//while(1) {
	s32 data = mt9v032_read(client, MT9V032_CHIP_VERSION);
	
	//if(data == 1324)
	//	break;
	//msleep(100);
	//}
	/* Reset the chip and stop data read out */
	ret = mt9v032_write(client, MT9V032_RESET, 1);
	if (ret < 0)
		return ret;

	ret = mt9v032_write(client, MT9V032_RESET, 0);
	if (ret < 0)
		return ret;

	on_done = 1;
	return mt9v032_write(client, MT9V032_CHIP_CONTROL, 0);
}

static void mt9v032_power_off(struct mt9v032 *mt9v032)
{
	//ankt
	//if (mt9v032->pdata->set_clock)
	//	mt9v032->pdata->set_clock(&mt9v032->subdev, 0);
	printk("mt9v disabling clk, poweroff\n");
	clk_disable_unprepare(mt9v032->clk);
}

static int __mt9v032_set_power(struct mt9v032 *mt9v032, bool on)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mt9v032->subdev);
	int ret;
	
	//ankt
	mt9v032->clk = devm_clk_get(&client->dev, NULL);
	if (IS_ERR(mt9v032->clk))
		return PTR_ERR(mt9v032->clk);
	printk("\r\nset_power--good\r\n");
	//

	if (!on) {
		mt9v032_power_off(mt9v032);
		return 0;
	}

	ret = mt9v032_power_on(mt9v032);
	if (ret < 0)
		return ret;

	/* Configure the pixel clock polarity */
	if (mt9v032->pdata && mt9v032->pdata->clk_pol) {
		ret = mt9v032_write(client, MT9V032_PIXEL_CLOCK,
				MT9V032_PIXEL_CLOCK_INV_PXL_CLK);
		if (ret < 0)
			return ret;
	}

	/* Disable the noise correction algorithm and restore the controls. */
	ret = mt9v032_write(client, MT9V032_ROW_NOISE_CORR_CONTROL, 0);
	if (ret < 0)
		return ret;

	return v4l2_ctrl_handler_setup(&mt9v032->ctrls);
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev video operations
 */

static struct v4l2_mbus_framefmt *
__mt9v032_get_pad_format(struct mt9v032 *mt9v032, struct v4l2_subdev_fh *fh,
			 unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &mt9v032->format;
	default:
		return NULL;
	}
}

static struct v4l2_rect *
__mt9v032_get_pad_crop(struct mt9v032 *mt9v032, struct v4l2_subdev_fh *fh,
		       unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &mt9v032->crop;
	default:
		return NULL;
	}
}

static int mt9v032_s_stream(struct v4l2_subdev *subdev, int enable)
{
	const u16 mode = MT9V032_CHIP_CONTROL_MASTER_MODE
		       | MT9V032_CHIP_CONTROL_DOUT_ENABLE
		       | MT9V032_CHIP_CONTROL_SEQUENTIAL;
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	struct mt9v032 *mt9v032 = to_mt9v032(subdev);
	struct v4l2_mbus_framefmt *format = &mt9v032->format;
	struct v4l2_rect *crop = &mt9v032->crop;
	unsigned int hratio;
	unsigned int vratio;
	int ret;

	printk("mt9v Stream on\n");

	if (!enable)
		return mt9v032_set_chip_control(mt9v032, mode, 0);

	/* Configure the window size and row/column bin */
	hratio = DIV_ROUND_CLOSEST(crop->width, format->width);
	vratio = DIV_ROUND_CLOSEST(crop->height, format->height);

	ret = mt9v032_write(client, MT9V032_READ_MODE,
		    (hratio - 1) << MT9V032_READ_MODE_ROW_BIN_SHIFT |
		    (vratio - 1) << MT9V032_READ_MODE_COLUMN_BIN_SHIFT);
	if (ret < 0)
		return ret;

	ret = mt9v032_write(client, MT9V032_COLUMN_START, crop->left);
	if (ret < 0)
		return ret;

	ret = mt9v032_write(client, MT9V032_ROW_START, crop->top);
	if (ret < 0)
		return ret;

	ret = mt9v032_write(client, MT9V032_WINDOW_WIDTH, crop->width);
	if (ret < 0)
		return ret;

	ret = mt9v032_write(client, MT9V032_WINDOW_HEIGHT, crop->height);
	if (ret < 0)
		return ret;

	ret = mt9v032_write(client, MT9V032_HORIZONTAL_BLANKING,
			    max(43, 660 - crop->width));
	if (ret < 0)
		return ret;

	/* Switch to master "normal" mode */
	return mt9v032_set_chip_control(mt9v032, 0, mode);
}

static int mt9v032_enum_mbus_code(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_SGRBG10_1X10;
	return 0;
}

static int mt9v032_enum_frame_size(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= 8 || fse->code != V4L2_MBUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width = MT9V032_WINDOW_WIDTH_DEF / fse->index;
	fse->max_width = fse->min_width;
	fse->min_height = MT9V032_WINDOW_HEIGHT_DEF / fse->index;
	fse->max_height = fse->min_height;

	return 0;
}

static int mt9v032_get_format(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *format)
{
	struct mt9v032 *mt9v032 = to_mt9v032(subdev);

	format->format = *__mt9v032_get_pad_format(mt9v032, fh, format->pad,
						   format->which);
	return 0;
}

static int mt9v032_set_format(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *format)
{
	struct mt9v032 *mt9v032 = to_mt9v032(subdev);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	unsigned int width;
	unsigned int height;
	unsigned int hratio;
	unsigned int vratio;

	__crop = __mt9v032_get_pad_crop(mt9v032, fh, format->pad,
					format->which);

	/* Clamp the width and height to avoid dividing by zero. */
	width = clamp_t(unsigned int, ALIGN(format->format.width, 2),
			max(__crop->width / 8, MT9V032_WINDOW_WIDTH_MIN),
			__crop->width);
	height = clamp_t(unsigned int, ALIGN(format->format.height, 2),
			 max(__crop->height / 8, MT9V032_WINDOW_HEIGHT_MIN),
			 __crop->height);

	hratio = DIV_ROUND_CLOSEST(__crop->width, width);
	vratio = DIV_ROUND_CLOSEST(__crop->height, height);

	__format = __mt9v032_get_pad_format(mt9v032, fh, format->pad,
					    format->which);
	__format->width = __crop->width / hratio;
	__format->height = __crop->height / vratio;

	format->format = *__format;

	return 0;
}

static int mt9v032_get_crop(struct v4l2_subdev *subdev,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_crop *crop)
{
	struct mt9v032 *mt9v032 = to_mt9v032(subdev);

	crop->rect = *__mt9v032_get_pad_crop(mt9v032, fh, crop->pad,
					     crop->which);
	return 0;
}

static int mt9v032_set_crop(struct v4l2_subdev *subdev,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_crop *crop)
{
	struct mt9v032 *mt9v032 = to_mt9v032(subdev);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	struct v4l2_rect rect;

	/* Clamp the crop rectangle boundaries and align them to a non multiple
	 * of 2 pixels to ensure a GRBG Bayer pattern.
	 */
	rect.left = clamp(ALIGN(crop->rect.left + 1, 2) - 1,
			  MT9V032_COLUMN_START_MIN,
			  MT9V032_COLUMN_START_MAX);
	rect.top = clamp(ALIGN(crop->rect.top + 1, 2) - 1,
			 MT9V032_ROW_START_MIN,
			 MT9V032_ROW_START_MAX);
	rect.width = clamp(ALIGN(crop->rect.width, 2),
			   MT9V032_WINDOW_WIDTH_MIN,
			   MT9V032_WINDOW_WIDTH_MAX);
	rect.height = clamp(ALIGN(crop->rect.height, 2),
			    MT9V032_WINDOW_HEIGHT_MIN,
			    MT9V032_WINDOW_HEIGHT_MAX);

	rect.width = min(rect.width, MT9V032_PIXEL_ARRAY_WIDTH - rect.left);
	rect.height = min(rect.height, MT9V032_PIXEL_ARRAY_HEIGHT - rect.top);

	__crop = __mt9v032_get_pad_crop(mt9v032, fh, crop->pad, crop->which);

	if (rect.width != __crop->width || rect.height != __crop->height) {
		/* Reset the output image size if the crop rectangle size has
		 * been modified.
		 */
		__format = __mt9v032_get_pad_format(mt9v032, fh, crop->pad,
						    crop->which);
		__format->width = rect.width;
		__format->height = rect.height;
	}

	*__crop = rect;
	crop->rect = rect;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev control operations
 */

#define V4L2_CID_TEST_PATTERN		(V4L2_CID_USER_BASE | 0x1001)

static int mt9v032_s_ctrl(struct v4l2_ctrl *ctrl);
static int _mt9v032_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return mt9v032_s_ctrl(ctrl);
}
static int mt9v032_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9v032 *mt9v032 =
			container_of(ctrl->handler, struct mt9v032, ctrls);
	struct i2c_client *client = v4l2_get_subdevdata(&mt9v032->subdev);
	u16 data;

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return mt9v032_update_aec_agc(mt9v032, MT9V032_AGC_ENABLE,
					      ctrl->val);

	case V4L2_CID_GAIN:
		return mt9v032_write(client, MT9V032_ANALOG_GAIN, ctrl->val);

	case V4L2_CID_EXPOSURE_AUTO:
		return mt9v032_update_aec_agc(mt9v032, MT9V032_AEC_ENABLE,
					      ctrl->val);

	case V4L2_CID_EXPOSURE:
		return mt9v032_write(client, MT9V032_TOTAL_SHUTTER_WIDTH,
				     ctrl->val);

	case V4L2_CID_TEST_PATTERN:
		switch (ctrl->val) {
		case 0:
			data = 0;
			break;
		case 1:
			data = MT9V032_TEST_PATTERN_GRAY_VERTICAL
			     | MT9V032_TEST_PATTERN_ENABLE;
			break;
		case 2:
			data = MT9V032_TEST_PATTERN_GRAY_HORIZONTAL
			     | MT9V032_TEST_PATTERN_ENABLE;
			break;
		case 3:
			data = MT9V032_TEST_PATTERN_GRAY_DIAGONAL
			     | MT9V032_TEST_PATTERN_ENABLE;
			break;
		default:
			data = (ctrl->val << MT9V032_TEST_PATTERN_DATA_SHIFT)
			     | MT9V032_TEST_PATTERN_USE_DATA
			     | MT9V032_TEST_PATTERN_ENABLE
			     | MT9V032_TEST_PATTERN_FLIP;
			break;
		}

		return mt9v032_write(client, MT9V032_TEST_PATTERN, data);
	default:
		break;
	}

	return 0;
}

static struct v4l2_ctrl_ops mt9v032_ctrl_ops = {
	.s_ctrl = mt9v032_s_ctrl,
};

static const struct v4l2_ctrl_config mt9v032_ctrls[] = {
	{
		.ops		= &mt9v032_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Test pattern",
		.min		= 0,
		.max		= 1023,
		.step		= 1,
		.def		= 0,
		.flags		= 0,
	}
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev core operations
 */

static int mt9v032_set_power(struct v4l2_subdev *subdev, int on)
{
	struct mt9v032 *mt9v032 = to_mt9v032(subdev);
	int ret = 0;
	
	printk("mt9v032_set_power %d\n", on);

	if(on == CSI_SUBDEV_PWR_ON)
		on = 1;
	else if(CSI_SUBDEV_PWR_OFF)
		on = 0;
	else if ( (on!=0) && (on!=1) )
		return 0;

	mutex_lock(&mt9v032->power_lock);

	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (mt9v032->power_count == !on) {
		//ret = __mt9v032_set_power(mt9v032, 1);
		ret = __mt9v032_set_power(mt9v032, !!on);
		if (ret < 0)
			goto done;
	}

	/* Update the power count. */
	//mt9v032->power_count += on ? 1 : -1;
	//WARN_ON(mt9v032->power_count < 0);

done:
	mutex_unlock(&mt9v032->power_lock);
	return ret;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */

#if 0
static const s64 mt9v034_link_freqs[] = {
	        13000000,
		        26600000,
			        27000000,
				        0,
};
static struct mt9v032_platform_data my_mt9v034_platform_data = {
	        .clk_pol        = 0,
		        .link_freqs     = mt9v034_link_freqs,
			        .link_def_freq  = 26600000,
};
#endif
//struct i2c_client *my_client;
//struct mt9v032 *my_mt9v032;
int mt9v032_registered(struct v4l2_subdev *subdev)
{
	//struct v4l2_subdev *subdev = subdev_apt;
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	struct mt9v032 *mt9v032 = to_mt9v032(subdev);
	s32 data;
	int ret;

	//if(asj==1) {
	//	subdev = subdev_apt;
	//	printk("\r\nmt9v032 good\r\n");
	//}
	//else {
	//	subdev = subdev1;
	//	printk("\r\nmt9v032 bad\r\n");
	//}
	//client = my_client;
	//if(client->addr == 0x24) {
	//	client->addr = 0x48;
	//	client->dev.platform_data = &my_mt9v034_platform_data;
	//}

	dev_info(&client->dev, "Probing MT9V032 at address 0x%02x\n",
			client->addr);
	
	//ret =__mt9v032_set_power(mt9v032, 1);
	//if (ret < 0)
	//	printk("\r\nmt9v032_probe 7\r\n");

	ret = mt9v032_power_on(mt9v032);
	if (ret < 0) {
		dev_err(&client->dev, "MT9V032 power up failed\n");
		return ret;
	}

	/* Read and check the sensor version */
	data = mt9v032_read(client, MT9V032_CHIP_VERSION);
	if (data != MT9V032_CHIP_ID_REV1 && data != MT9V032_CHIP_ID_REV3 && 
			data != MT9V034_CHIP_ID_REV1) {
		dev_err(&client->dev, "MT9V032 not detected, wrong version "
				"0x%04x\n", data);
		return -ENODEV;
	}

	//mt9v032_power_off(mt9v032);

	dev_info(&client->dev, "MT9V032 detected at address 0x%02x\n",
			client->addr);

	return ret;
}

static int mt9v032_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;

	printk("mt9v opened\n");
	crop = v4l2_subdev_get_try_crop(fh, 0);
	crop->left = MT9V032_COLUMN_START_DEF;
	crop->top = MT9V032_ROW_START_DEF;
	crop->width = MT9V032_WINDOW_WIDTH_DEF;
	crop->height = MT9V032_WINDOW_HEIGHT_DEF;

	format = v4l2_subdev_get_try_format(fh, 0);
	format->code = V4L2_MBUS_FMT_SGRBG10_1X10;
	format->width = MT9V032_WINDOW_WIDTH_DEF;
	format->height = MT9V032_WINDOW_HEIGHT_DEF;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;

	return mt9v032_set_power(subdev, 1);
}

static int mt9v032_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	printk("mt9 closed\n");
	return mt9v032_set_power(subdev, 0);
}

static int mt9v032_init(struct v4l2_subdev *sd, u32 val)
{
	printk("mt9v init called\n");
	const u16 mode = MT9V032_CHIP_CONTROL_MASTER_MODE
		       | MT9V032_CHIP_CONTROL_DOUT_ENABLE
		       | MT9V032_CHIP_CONTROL_SEQUENTIAL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9v032 *mt9v032 = to_mt9v032(sd);

	return mt9v032_set_chip_control(mt9v032,0, mode);
}

static int mt9v032_g_chip_ident(struct v4l2_subdev *sd,
		    struct v4l2_dbg_chip_ident *chip)
{
	  struct i2c_client *client = v4l2_get_subdevdata(sd);

	    return v4l2_chip_ident_i2c_client(client, chip, 0x1324, 0);
}


static int sensor_g_exif(struct v4l2_subdev *sd, struct sensor_exif_attribute *exif)
{
	int ret = 0;//, gain_val, exp_val;
	
	exif->fnumber = 220;
	exif->focal_length = 180;
	exif->brightness = 125;
	exif->flash_fire = 0;
	exif->iso_speed = 200;
	exif->exposure_time_num = 1;
	exif->exposure_time_den = 15;
	return ret;
}

static long mt9v032_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret=0;
	switch(cmd) {
		case GET_SENSOR_EXIF:
			sensor_g_exif(sd, (struct sensor_exif_attribute *)arg);
			break;
		default:
			return -EINVAL;
	}
	printk("mt9v032_ioctl !!\n");
	return ret;
}
static int mt9v032_g_mbus_config(struct v4l2_subdev *sd,
           struct v4l2_mbus_config *cfg)
{
  cfg->type = V4L2_MBUS_PARALLEL;
  cfg->flags = V4L2_MBUS_MASTER | VREF_POL | HREF_POL | CLK_POL ;
  
  return 0;
}
static int mt9v032_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
  struct v4l2_captureparm *cp = &parms->parm.capture;
  struct v4l2_fract *tpf = &cp->timeperframe;
  //struct sensor_info *info = to_state(sd);
  unsigned char div;
  
  printk("sensor_s_parm\n");
  
  if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE){
  	printk("parms->type!=V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
    return -EINVAL;
  }
  
  if (tpf->numerator == 0 || tpf->denominator == 0) {
    tpf->numerator = 1;
    tpf->denominator = 60;//30;/* Reset to full rate */
    printk("sensor frame rate reset to full rate!\n");
  }
  
  div = 60/(tpf->denominator/tpf->numerator);
  //div = 30/(tpf->denominator/tpf->numerator);
  if(div > 15 || div == 0)
  {
  	printk("SENSOR_FRAME_RATE=%d\n",60);//30);
  	printk("tpf->denominator=%d\n",tpf->denominator);
  	printk("tpf->numerator=%d\n",tpf->numerator);
    return -EINVAL;
  }
  
  printk("set frame rate %d\n",tpf->denominator/tpf->numerator);
  
	return 0;
}

static int mt9v032_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	//struct sensor_info *info = to_state(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	
	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->capturemode = V4L2_MODE_VIDEO;
	
	cp->timeperframe.numerator = 1;//info->tpf.numerator;
	cp->timeperframe.denominator = 60;//30;//info->tpf.denominator;
	 
	return 0;
}

static int mt9v032_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt) 
{
	return 0;
}
static int mt9v032_enum_size(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
  //if(fsize->index > N_WIN_SIZES-1)
  //	return -EINVAL;
  
  fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
  fsize->discrete.width = 768;//sensor_win_sizes[fsize->index].width;
  fsize->discrete.height = 480;//sensor_win_sizes[fsize->index].height;
  printk("%s %d width=%d height=%d\n", __func__, fsize->index,  fsize->discrete.width,  fsize->discrete.height);
  return 0;
}
static int mt9v032_enum_fmt(struct v4l2_subdev *sd, unsigned index,
                 enum v4l2_mbus_pixelcode *code)
{
  //if (index >= N_FMTS)
  //  return -EINVAL;

  printk("%s %d %x\n", __func__, index, *code);
  *code = V4L2_MBUS_FMT_SGRBG10_1X10;//sensor_formats[index].mbus_code;
  return 0;
}
static int mt9v032_try_fmt(struct v4l2_subdev *sd,
		             struct v4l2_mbus_framefmt *fmt)
{

  fmt->width = 768;//wsize->width;
  fmt->height = 480;//wsize->height;
  return 0;
}


static struct v4l2_subdev_core_ops mt9v032_subdev_core_ops = {
	.g_chip_ident = mt9v032_g_chip_ident,
  	.s_ctrl	 	= _mt9v032_s_ctrl,
	.s_power	= mt9v032_set_power,
	.init		= mt9v032_init,
  	.ioctl 		= mt9v032_ioctl,
};

static struct v4l2_subdev_video_ops mt9v032_subdev_video_ops = {
	.s_stream	= mt9v032_s_stream,
  	.try_mbus_fmt = mt9v032_try_fmt,
  	.enum_mbus_fmt = mt9v032_enum_fmt,
  	.enum_framesizes = mt9v032_enum_size,
  	.s_mbus_fmt 	= mt9v032_s_fmt,
  	.s_parm 	= mt9v032_s_parm,
  	.g_parm 	= mt9v032_g_parm,
  	.g_mbus_config 	= mt9v032_g_mbus_config,
};

static struct v4l2_subdev_pad_ops mt9v032_subdev_pad_ops = {
	.enum_mbus_code = mt9v032_enum_mbus_code,
	.enum_frame_size = mt9v032_enum_frame_size,
	.get_fmt = mt9v032_get_format,
	.set_fmt = mt9v032_set_format,
	.get_crop = mt9v032_get_crop,
	.set_crop = mt9v032_set_crop,
};

static struct v4l2_subdev_ops mt9v032_subdev_ops = {
	.core	= &mt9v032_subdev_core_ops,
	.video	= &mt9v032_subdev_video_ops,
	.pad	= &mt9v032_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops mt9v032_subdev_internal_ops = {
	.registered = mt9v032_registered,
	.open = mt9v032_open,
	.close = mt9v032_close,
};

#if 0
static struct device_attribute cci_device_attrs[] = {
	__ATTR(addr_width,  S_IWUSR | S_IRUGO, cci_device_addr_width_show, cci_device_addr_width_store),
	__ATTR(data_width,  S_IWUSR | S_IRUGO, cci_device_data_width_show, cci_device_data_width_store),
	__ATTR(read_value, S_IRUGO, cci_device_read_value_show, NULL),
	__ATTR(read_flag,  S_IWUSR | S_IRUGO, cci_device_read_flag_show, cci_device_read_flag_store),
	__ATTR(cci_client,  S_IWUSR | S_IRUGO, cci_sys_show, cci_sys_store),
};
#endif


static int cci_sys_register(struct cci_driver *drv_data)
{
	int i, ret;
	drv_data->cci_device = my_cci_device_def;
	dev_set_name(&drv_data->cci_device, drv_data->name);
	
	if (device_register(&drv_data->cci_device))
		printk("error device_register()\n");
	
	dev_set_drvdata(&drv_data->cci_device,drv_data);
#if 0
	/* sysfs entries */
	for (i = 0; i < ARRAY_SIZE(cci_device_attrs); i++) {
		ret = device_create_file(&drv_data->cci_device, &cci_device_attrs[i]);
		if (ret) {
			printk("device_create_file error\n");
			device_remove_file(&drv_data->cci_device, &drv_data->dev_attr_cci);
		}
	}
#endif
	return 0;
}

/* -----------------------------------------------------------------------------
 * Driver initialization and probing
 */

static int mt9v032_probe(struct i2c_client *client,
		const struct i2c_device_id *did)
{
	struct mt9v032 *mt9v032;
	unsigned int i;
	int ret;

	//ankt
	printk("\r\nmt9v032_probe 0\r\n");
	//if(asj ==1) {
	//	client = my_client;
	//	return 0;
	//}

	//if(client->addr == 0x24) {
	//	client->addr = 0x48;
	//	client->dev.platform_data = &my_mt9v034_platform_data;
	//}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&client->adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	printk("\r\nmt9v032_probe 1\r\n");
	mt9v032 = kzalloc(sizeof(*mt9v032), GFP_KERNEL);
	if (!mt9v032)
		return -ENOMEM;

	mt9v032->clk = devm_clk_get(&client->dev, "csi_m");
	if (IS_ERR(mt9v032->clk))
		return PTR_ERR(mt9v032->clk);

	mutex_init(&mt9v032->power_lock);
	mt9v032->pdata = client->dev.platform_data;
	printk("\r\nmt9v032_probe 2\r\n");

	v4l2_ctrl_handler_init(&mt9v032->ctrls, ARRAY_SIZE(mt9v032_ctrls) + 4);

	v4l2_ctrl_new_std(&mt9v032->ctrls, &mt9v032_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	v4l2_ctrl_new_std(&mt9v032->ctrls, &mt9v032_ctrl_ops,
			  V4L2_CID_GAIN, MT9V032_ANALOG_GAIN_MIN,
			  MT9V032_ANALOG_GAIN_MAX, 1, MT9V032_ANALOG_GAIN_DEF);
	v4l2_ctrl_new_std_menu(&mt9v032->ctrls, &mt9v032_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, 0,
			       V4L2_EXPOSURE_AUTO);
	v4l2_ctrl_new_std(&mt9v032->ctrls, &mt9v032_ctrl_ops,
			  V4L2_CID_EXPOSURE, MT9V032_TOTAL_SHUTTER_WIDTH_MIN,
			  MT9V032_TOTAL_SHUTTER_WIDTH_MAX, 1,
			  MT9V032_TOTAL_SHUTTER_WIDTH_DEF);
	printk("\r\nmt9v032_probe 3\r\n");

	for (i = 0; i < ARRAY_SIZE(mt9v032_ctrls); ++i)
		v4l2_ctrl_new_custom(&mt9v032->ctrls, &mt9v032_ctrls[i], NULL);

	mt9v032->subdev.ctrl_handler = &mt9v032->ctrls;

	if (mt9v032->ctrls.error)
		printk(KERN_INFO "%s: control initialization error %d\n",
		       __func__, mt9v032->ctrls.error);

	printk("\r\nmt9v032_probe 4\r\n");
	mt9v032->crop.left = MT9V032_COLUMN_START_DEF;
	mt9v032->crop.top = MT9V032_ROW_START_DEF;
	mt9v032->crop.width = MT9V032_WINDOW_WIDTH_DEF;
	mt9v032->crop.height = MT9V032_WINDOW_HEIGHT_DEF;

	mt9v032->format.code = V4L2_MBUS_FMT_SGRBG10_1X10;
	mt9v032->format.width = MT9V032_WINDOW_WIDTH_DEF;
	mt9v032->format.height = MT9V032_WINDOW_HEIGHT_DEF;
	mt9v032->format.field = V4L2_FIELD_NONE;
	mt9v032->format.colorspace = V4L2_COLORSPACE_SRGB;

	mt9v032->aec_agc = MT9V032_AEC_ENABLE | MT9V032_AGC_ENABLE;

	ret = 0;
#if 1
	v4l2_i2c_subdev_init(&mt9v032->subdev, client, &mt9v032_subdev_ops);
	mt9v032->subdev.internal_ops = &mt9v032_subdev_internal_ops;
	mt9v032->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	printk("\r\nmt9v032_probe 5\r\n");

	mt9v032->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&mt9v032->subdev.entity, 1, &mt9v032->pad, 0);
	if (ret < 0)
	//ankt
		kfree(mt9v032);
#endif
	//goto err;
	//mt9v032->subdev.dev = &client->dev;
	//ret = v4l2_async_register_subdev(&mt9v032->subdev);
	//if (ret < 0)
	//	goto err;
	//return 0;
//err:
	//media_entity_cleanup(&mt9v032->subdev.entity);
	//v4l2_ctrl_handler_free(&mt9v032->ctrls);
	//kfree(mt9v032);
	//ankt
	printk("\r\nmt9v032_probe 6\r\n");
	//cci_sys_register(&cci_drv);
	//if(client->addr == 0x48) {
		my_client = client;
		//my_client_new = kzalloc(sizeof(*my_client_new), GFP_KERNEL);

		//memcpy(my_client_new, client, sizeof(struct i2c_client));
		//my_client = my_client_new;

		//my_client_new.flags = client->flags;
		//my_client_new.addr = client->addr;
		//strcpy(my_client_new.name, client->name);
		//my_client_new.adapter = client->adapter;

		//my_client_new.driver = client->driver;
		//my_client_new.dev = client->dev;
		//my_client_new.irq = client->irq;
		//my_client_new.detected = client->detected;
		//my_client = &my_client_new;

	//	my_mt9v032 = mt9v032;
	//} else {
	//	mt9v032 = my_mt9v032;
	//}
#if 0
	subdev_apt = &mt9v032->subdev;
	mt9v032_registered(&mt9v032->subdev);
	mt9v032_registered(&mt9v032->subdev);
	mt9v032_registered(&mt9v032->subdev);
	mt9v032_registered(&mt9v032->subdev);
	asj = 1;
#endif
	//ret =__mt9v032_set_power(mt9v032, 1);
	//if (ret < 0)
	//	printk("\r\nmt9v032_probe 7\r\n");
	
	//proc
	/* create the /proc file */
	Our_Proc_File = create_proc_entry(PROCFS_NAME, 0644, NULL);
	
	if (Our_Proc_File == NULL) {
		remove_proc_entry(PROCFS_NAME, NULL);
		printk(KERN_ALERT "mt9v032 Error: Could not initialize /proc/%s\n",
		PROCFS_NAME);
		//return -ENOMEM;
	}
	
	Our_Proc_File->read_proc  = procfile_read;
	Our_Proc_File->write_proc = procfile_write;
	//Our_Proc_File->owner 	  = THIS_MODULE;
	Our_Proc_File->mode 	  = S_IFREG | S_IRUGO;
	Our_Proc_File->uid 	  = 0;
	Our_Proc_File->gid 	  = 0;
	Our_Proc_File->size 	  = 37;

	printk(KERN_INFO "mt9v032 /proc/%s created\n", PROCFS_NAME);


	//proc

	return ret;
}

static int mt9v032_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct mt9v032 *mt9v032 = to_mt9v032(subdev);
	
	//ankt
	//v4l2_async_unregister_subdev(subdev);
	//v4l2_ctrl_handler_free(&mt9v032->ctrls);
	//ankt
	v4l2_device_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);
	kfree(mt9v032);
	return 0;
}

static const struct i2c_device_id mt9v032_id[] = {
	{ "mt9v032", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9v032_id);

static struct i2c_driver mt9v032_driver = {
	.driver = {
		.name = "mt9v032",
	},
	.probe		= mt9v032_probe,
	.remove		= mt9v032_remove,
	.id_table	= mt9v032_id,
};

module_i2c_driver(mt9v032_driver);

MODULE_DESCRIPTION("Aptina MT9V032 Camera driver");
MODULE_AUTHOR("Laurent Pinchart <laurent.pinchart@ideasonboard.com>");
MODULE_LICENSE("GPL");
