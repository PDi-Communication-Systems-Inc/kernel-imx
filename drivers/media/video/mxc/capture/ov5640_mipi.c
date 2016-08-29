/*
 * Copyright (C) 2011-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/mipi_csi2.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"

#define MIN_FPS 15
#define MAX_FPS 30			                 // JAD was 30
#define DEFAULT_FPS 30		                 // JAD was 30

#define OV5640_XCLK_MIN 6000000
#define OV5640_XCLK_MAX 24000000             // JAD was 24000000

// #define OV5640_CHIP_ID_HIGH_BYTE	0x300A
// #define OV5640_CHIP_ID_LOW_BYTE	0x300B

/*enum ov5640_mode {
        ov5640_mode_MIN                 = 0,
        ov5640_mode_VGA_640_480         = 0,
        ov5640_mode_LCD_1370_768        = 1,  //JAD change?
 //       ov5640_mode_QVGA_320_240      = 1,
 //       ov5640_mode_NTSC_720_480      = 2,
 //       ov5640_mode_PAL_720_576       = 3,
 //       ov5640_mode_720P_1280_720     = 4, 
 //       ov5640_mode_1080P_1920_1080   = 5,
 //       ov5640_mode_QSXGA_2592_1944   = 6,
 //       ov5640_mode_QCIF_176_144      = 7,
        ov5640_mode_XGA_1370_768        = 2,  // was ov5640_mode_XGA_1024_768024
        ov5640_mode_MAX                 = 2,  // was 4
        ov5640_mode_INIT                = 0xff, /*only for sensor init*/
//};

enum ov5640_mode {
        ov5640_mode_MIN               = 0,
        ov5640_mode_VGA_640_480       = 0,
        ov5640_mode_QVGA_320_240      = 1,
        ov5640_mode_NTSC_720_480      = 2,
        ov5640_mode_PAL_720_576       = 3,
        ov5640_mode_720P_1280_720     = 4,
        ov5640_mode_XGA_1368_768024   = 5,  // was ov5640_mode_XGA_1024_768024		
        ov5640_mode_1080P_1920_1080   = 6,
        ov5640_mode_QSXGA_2592_1944   = 7,
        ov5640_mode_QCIF_176_144      = 8,
        ov5640_mode_MAX               = 6,  // was 4
        ov5640_mode_INIT              = 0xff, /*only for sensor init*/
};
enum ov5640_frame_rate {
	ov5640_15_fps,
	ov5640_30_fps
};

static int ov5640_framerates[] = {
	[ov5640_15_fps] = 15,
	[ov5640_30_fps] = 30,
};

/* image size under 1280 * 960 are SUBSAMPLING
 * image size upper 1280 * 960 are SCALING
 */
enum ov5640_downsize_mode {
	SUBSAMPLING,
	SCALING,
};

struct reg_value {
	u16 u16RegAddr; 
	u8  u8Val;
	u8  u8Mask;
	u32 u32Delay_ms;
};

struct ov5640_mode_info {
	enum ov5640_mode mode;
	enum ov5640_downsize_mode dn_mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

//
// Maintains the information on the current state of the sensor.
//
static struct sensor_data ov5640_data;
static int pwn_gpio, rst_gpio;
//
// JAD DS90ub740 Init Settings - out of reset - feed through
// 0x0018 is a general puropse mailbox location for testing 
// reads and writes.  CSI indirect address = 0x6c, data=0x6d, 
// For RGB888 instead of YUV422, change 0x006b from 0x50 to 0x00
// For Stripe bars instead of sequence, 0x0064, bit 2=1 (11->15)
static struct reg_value ub940_init_setting[] = {

    {0x0001, 0x02, 0, 0},                                              // Reset the part
	{0x006b, 0x50, 0, 0},{0x006c, 0x2e, 0, 0}, {0x006d, 0x40, 0, 0},   // Virtual Channel ID=1, YUV422 
                                                                       // Set Pattern Generator (indirect Registers)    
/*	{0x0066, 0x0e, 0, 0}, {0x0067, 0x02, 0, 0}, 
	{0x0066, 0x0e, 0, 0}, {0x0067, 0x02, 0, 0},                        
	{0x0066, 0x04, 0, 0}, {0x0067, 0x00, 0, 0}, 
	{0x0066, 0x05, 0, 0}, {0x0067, 0xb7, 0, 0},                        
 	{0x0066, 0x07, 0, 0}, {0x0067, 0x50, 0, 0}, 
	{0x0066, 0x08, 0, 0}, {0x0067, 0x05, 0, 0},                        
 	{0x0066, 0x0a, 0, 0}, {0x0067, 0x70, 0, 0}, 
	{0x0066, 0x0c, 0, 0}, {0x0067, 0xff, 0, 0},                        
 	{0x0066, 0x05, 0, 0}, {0x0067, 0xb7, 0, 0}, 
	{0x0066, 0x06, 0, 0}, {0x0067, 0x31, 0, 0},                        
 	{0x0066, 0x08, 0, 0}, {0x0067, 0x05, 0, 0}, 
	{0x0066, 0x09, 0, 0}, {0x0067, 0x30, 0, 0},                        
 	{0x0066, 0x0b, 0, 0}, {0x0067, 0x06, 0, 0}, 
	{0x0066, 0x0d, 0, 0}, {0x0067, 0x12, 0, 0},                        
          
    {0x0068, 0x01, 0, 0}, {0x0068, 0x11, 0, 0}, {0x0068, 0x21, 0, 0},  // Sequence to turn on pattern generator.
	{0x0068, 0x31, 0, 0}, {0x0065, 0x04, 0, 0}, {0x0068, 0x01, 0, 0},  // Turn Pat Gen on
	{0x0068, 0x11, 0, 0}, {0x0068, 0x21, 0, 0}, {0x0068, 0x31, 0, 0},  // Empty

    {0x0068, 0x31, 0, 0}, {0x0018, 0x00, 0, 0}, {0x0065, 0x05, 0, 0},  // Turn Pat Gen on
    {0x0039, 0x02, 0, 0}, {0x0064, 0x11, 0, 0}, {0x0066, 0x04, 0, 0},  // 
    {0x0066, 0x05, 0, 0}, {0x0066, 0x05, 0, 0}, {0x0066, 0x06, 0, 0},  // 
	{0x0066, 0x03, 0, 0}, {0x0068, 0x01, 0, 0}, {0x0068, 0x11, 0, 0},  // 
	{0x0068, 0x21, 0, 0}, {0x0068, 0x31, 0, 0}, {0x0018, 0x00, 0, 0},  //       
*/
};

static struct reg_value ub940_640x480_setting[] = {
    {0x0001, 0x02, 0, 0},                                              // Reset the part
	{0x006c, 0x2e, 0, 0}, {0x006d, 0x40, 0, 0}, {0x006b, 0x50, 0, 0},  // Virtual Channel ID=1, YUV422 
                                                                       // Set Pattern Generator (indirect Registers)    
	{0x0066, 0x07, 0, 0}, {0x0067, 0x80, 0, 0},                         
    {0x0066, 0x08, 0, 0}, {0x0067, 0x02, 0, 0},                        
	{0x0066, 0x08, 0, 0}, {0x0067, 0x02, 0, 0},                        
	{0x0066, 0x09, 0, 0}, {0x0067, 0x1e, 0, 0},   
	{0x0066, 0x0c, 0, 0}, {0x0067, 0x28, 0, 0},
    {0x0066, 0x04, 0, 0}, {0x0067, 0x20, 0, 0},
    {0x0066, 0x05, 0, 0}, {0x0067, 0xe3, 0, 0},
    {0x0066, 0x0a, 0, 0}, {0x0067, 0x60, 0, 0},
    {0x0066, 0x0d, 0, 0}, {0x0067, 0x19, 0, 0},
    {0x0066, 0x05, 0, 0}, {0x0067, 0xd3, 0, 0},
    {0x0066, 0x06, 0, 0}, {0x0067, 0x20, 0, 0},
    {0x0066, 0x0b, 0, 0}, {0x0067, 0x02, 0, 0},
    {0x0066, 0x0e, 0, 0}, {0x0067, 0x01, 0, 0},
    {0x0066, 0x0e, 0, 0}, {0x0067, 0x03, 0, 0},
    {0x0066, 0x03, 0, 0}, {0x0067, 0x08, 0, 0},
        
    {0x0068, 0x01, 0, 0}, {0x0068, 0x11, 0, 0}, {0x0068, 0x21, 0, 0},  // Sequence to turn on pattern generator.
	{0x0068, 0x31, 0, 0}, {0x0065, 0x04, 0, 0}, {0x0068, 0x01, 0, 0},  // Turn Pat Gen on
	{0x0068, 0x11, 0, 0}, {0x0068, 0x21, 0, 0}, {0x0068, 0x31, 0, 0},  // Empty

    {0x0068, 0x31, 0, 0}, {0x0018, 0x00, 0, 0}, {0x0065, 0x05, 0, 0},  // Turn Pat Gen on
    {0x0039, 0x02, 0, 0}, {0x0064, 0x11, 0, 0}, {0x0066, 0x04, 0, 0},  // 
    {0x0066, 0x05, 0, 0}, {0x0066, 0x05, 0, 0}, {0x0066, 0x06, 0, 0},  // 
	{0x0066, 0x03, 0, 0}, {0x0068, 0x01, 0, 0}, {0x0068, 0x11, 0, 0},  // 
	{0x0068, 0x21, 0, 0}, {0x0068, 0x31, 0, 0}, {0x0018, 0x00, 0, 0},  //       
};

// For RGB888 instead of YUV422, change 0x006b from 0x50 to 0x00
// For Stripe bars instead of sequence, 0x0064, bit 2=1 (11->15)
static struct reg_value ub940_1360x768_setting[] = {
    {0x0001, 0x02, 0, 0},                                              // Reset the part
	{0x006b, 0x50, 0, 0}, {0x006c, 0x2e, 0, 0}, {0x006d, 0x40, 0, 0},  // Virtual Channel ID=1, YUV422 
    // Set Pattern Generator (indirect Registers)   
	{0x0066, 0x0e, 0, 0}, {0x0067, 0x02, 0, 0}, 
	{0x0066, 0x0e, 0, 0}, {0x0067, 0x02, 0, 0},                        
	{0x0066, 0x04, 0, 0}, {0x0067, 0x00, 0, 0}, 
	{0x0066, 0x05, 0, 0}, {0x0067, 0xb7, 0, 0},                        
 	{0x0066, 0x07, 0, 0}, {0x0067, 0x50, 0, 0}, 
	{0x0066, 0x08, 0, 0}, {0x0067, 0x05, 0, 0},                        
 	{0x0066, 0x0a, 0, 0}, {0x0067, 0x70, 0, 0}, 
	{0x0066, 0x0c, 0, 0}, {0x0067, 0xff, 0, 0},                        
 	{0x0066, 0x05, 0, 0}, {0x0067, 0xb7, 0, 0}, 
	{0x0066, 0x06, 0, 0}, {0x0067, 0x31, 0, 0},                        
 	{0x0066, 0x08, 0, 0}, {0x0067, 0x05, 0, 0}, 
	{0x0066, 0x09, 0, 0}, {0x0067, 0x30, 0, 0},                        
 	{0x0066, 0x0b, 0, 0}, {0x0067, 0x06, 0, 0}, 
	{0x0066, 0x0d, 0, 0}, {0x0067, 0x12, 0, 0},                        
          
    {0x0068, 0x01, 0, 0}, {0x0068, 0x11, 0, 0}, {0x0068, 0x21, 0, 0},  // Sequence to turn on pattern generator.
	{0x0068, 0x31, 0, 0}, {0x0065, 0x04, 0, 0}, {0x0068, 0x01, 0, 0},  // Turn Pat Gen on
	{0x0068, 0x11, 0, 0}, {0x0068, 0x21, 0, 0}, {0x0068, 0x31, 0, 0},  // Empty

    {0x0068, 0x31, 0, 0}, {0x0018, 0x00, 0, 0}, {0x0065, 0x05, 0, 0},  // Turn Pat Gen on
    {0x0039, 0x02, 0, 0}, {0x0064, 0x15, 0, 0}, {0x0066, 0x04, 0, 0},  // Color bars-0x0064, bit 2=1
    {0x0066, 0x05, 0, 0}, {0x0066, 0x05, 0, 0}, {0x0066, 0x06, 0, 0},  // 
	{0x0066, 0x03, 0, 0}, {0x0068, 0x01, 0, 0}, {0x0068, 0x11, 0, 0},  // 
	{0x0068, 0x21, 0, 0}, {0x0068, 0x31, 0, 0}, {0x0018, 0x00, 0, 0},  // 
	
};

// Original MIPI Structures
//static struct reg_value ov5640_init_setting_30fps_VGA[] = {
//};

static struct reg_value ov5640_setting_30fps_VGA_640_480[] = {
};

static struct reg_value ov5640_setting_15fps_VGA_640_480[] = {
};


static struct ov5640_mode_info ov5640_mode_info_data[2][ov5640_mode_MAX + 1] = {    // JAD was 5
	{
		{ov5640_mode_VGA_640_480, SUBSAMPLING, 1368,  768,        // JAD was 640 x 480
		ov5640_setting_15fps_VGA_640_480,
		ARRAY_SIZE(ov5640_setting_15fps_VGA_640_480)},
	},
	{
		{ov5640_mode_VGA_640_480, SUBSAMPLING, 1368,  768,        // JAD was 640 x 480
		ov5640_setting_30fps_VGA_640_480,
		ARRAY_SIZE(ov5640_setting_30fps_VGA_640_480)},
	},
};

static struct fsl_mxc_camera_platform_data *camera_plat;

static int ov5640_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov5640_remove(struct i2c_client *client);

static s32 ub9xx_read_reg (u16 reg, u8 *val);
static s32 ub9xx_write_reg(u16 reg, u8  val);

// 
static const struct i2c_device_id ov5640_id[] = {
	{"ov5640_mipi", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov5640_id);

// 
static struct i2c_driver ov5640_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov5640_mipi",
		  },
	.probe    = ov5640_probe,
	.remove   = ov5640_remove,
	.id_table = ov5640_id,
};

// This can become 74ub9xx write function
// add addditional comments
static s32 ub9xx_write_reg(u16 reg, u8 val)
{
	u8 au8Buf[3] = {0};         // 2 address and 1 data value    
	au8Buf   [0] = reg & 0xff;  // JAD was - reg >> 8
	au8Buf   [1] = val;         // JAD was - reg & 0xff
	au8Buf   [2] = val;         // may be able to get rid of this
	if (i2c_master_send(ov5640_data.i2c_client, au8Buf, 2) < 0) {                // JAD was 3
//		pr_err(">>>> %s: error:reg=%x,val=%x\n",__func__, reg, val);             // JAD
		return -1;
	}

	return 0;
}

// This can become 74ub9xx read function
// add addditional comments
static s32 ub9xx_read_reg(u16 reg, u8 *val)
{
	u8 au8RegBuf[2] = {0};        // 2 address and 1 data value  
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg & 0xff;    // JAD was - reg >> 8
	au8RegBuf[1] = reg & 0x0000;  // JAD was - reg & 0xff

	if (1 != i2c_master_send(ov5640_data.i2c_client, au8RegBuf, 1)) {   // JAD was 2
//		pr_err(">>>> %s: write reg error:reg=%x\n",                     // JAD
//				__func__, reg);
		return -1;
	}

	if (1 != i2c_master_recv(ov5640_data.i2c_client, &u8RdVal, 1)) {
//		pr_err(">>>> %s: read reg error:reg=%x,val=%x\n",             //JAD
//				__func__, reg, u8RdVal);
		return -1;
	}
	*val = u8RdVal;

	return u8RdVal;
}

// static int prev_sysclk, prev_HTS;
//static int AE_low, AE_high, AE_Target = 52;

int OV5640_get_sysclk(void)
{
	 /* calculate sysclk */
	int xvclk = ov5640_data.mclk / 10000;
	int temp1, temp2;
	int Multiplier, PreDiv, VCO, SysDiv, Pll_rdiv, Bit_div2x = 1, sclk_rdiv, sysclk;
//	u8 temp;

	int sclk_rdiv_map[] = {1, 2, 4, 8};

//	temp1 = ov5640_read_reg(0x3034, &temp);
    temp1 = 24;
//	pr_err(">>>> %s 1 temp1: %i",temp1);  // JAD
	temp2 = temp1 & 0x0f;
	if (temp2 == 8 || temp2 == 10) {
		Bit_div2x = temp2 / 2;
	}

//	temp1 = ov5640_read_reg(0x3035, &temp);
	temp1 = 20;
//	pr_err(" >>>> %s 2 temp1: %i", temp1);  // JAD
	SysDiv = temp1>>4;
	if (SysDiv == 0) {
	    SysDiv = 16;
	}

//	temp1 = ov5640_read_reg(0x3036, &temp);
//	temp1 = 56;
//	pr_err("3 temp1: %i",temp1);
//	Multiplier = temp1;
//
//	temp1 = ov5640_read_reg(0x3037, &temp);
//	temp1 = 19;
//	pr_err("4 temp1: %i",temp1);
//	PreDiv = temp1 & 0x0f;
//	Pll_rdiv = ((temp1 >> 4) & 0x01) + 1;
//
//	temp1 = ov5640_read_reg(0x3108, &temp);
//	temp1 = 1;
//	pr_err("5 temp1: %i",temp1);
//	temp2 = temp1 & 0x03;
//	sclk_rdiv = sclk_rdiv_map[temp2];
//	VCO = xvclk * Multiplier / PreDiv;
//	sysclk = VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv;
//
//	/* Bypass Hack */
//	pr_err("sysclk1: %i", sysclk);              //5600
//	pr_err("sysclk freq: %i",ov5640_data.mclk); //24000000 (24MHz)
//	pr_err("xvclk: %i", xvclk);                 //2400

	sysclk = 5600;
	xvclk  = 2400;

	return sysclk;
}

/*
  This functions sets registers on the OV5640 by passing the 
  structures available for each mode, for example:
  reg_value ov5640_setting_30fps_VGA_640_480[]
*/
static int ov5640_download_firmware(struct reg_value *pModeSetting, s32 ArySize)
{
	register u32 Delay_ms = 0;
	register u16 RegAddr  = 0;
	register u8 Mask      = 0;
	register u8 Val       = 0;
	u8 RegVal             = 0;
	int i, retval         = 0;

	for (i = 0; i < ArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr  = pModeSetting->u16RegAddr;
		Val      = pModeSetting->u8Val;
		Mask     = pModeSetting->u8Mask;

		if (Mask) {
			retval = ub9xx_read_reg(RegAddr, &RegVal);
//			pr_err("%s: >>>> Get RegVal: %i", RegVal);    // JAD

			if (retval < 0)
				goto err;

			RegVal &= ~(u8)Mask;
			Val    &= Mask;
			Val    |= RegVal;
		}
		retval = ub9xx_write_reg(RegAddr, Val);

//		pr_err("retval 2: %i", retval);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:
	return retval;
}

/* sensor changes between scaling and subsampling
 * go through exposure calcualtion
 */
//static int ov5640_change_mode_exposure_calc(enum ov5640_frame_rate frame_rate,
//				enum ov5640_mode mode)
//{
//    return 0;
//}

static int ov5640_init_mode(enum ov5640_frame_rate frame_rate,
			    enum ov5640_mode mode, enum ov5640_mode orig_mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 ArySize = 0;
	int retval  = 0;
	void *mipi_csi2_info;
	u32 mipi_reg, msec_wait4stable = 0;
	enum ov5640_downsize_mode dn_mode, orig_dn_mode;

	pr_err ("\n\n>>>> ov5640_init_mode: build date = %s %s", __DATE__, __TIME__);
	pr_err (">>>> ov5640_init_mode: frame rate=%i, mode=%i, orig_mode=%i", frame_rate, mode, orig_mode);

    pModeSetting = ub940_1360x768_setting;         			        // JAD New for LVDS-CSI2
    ArySize = ARRAY_SIZE(ub940_1360x768_setting);                   // JAD
    retval = ov5640_download_firmware(pModeSetting, ArySize);       // JAD

	mipi_csi2_info = mipi_csi2_get_info();

	/* initial mipi dphy */
	if (mipi_csi2_info) {

		if (!mipi_csi2_get_status(mipi_csi2_info))
		{
//			pr_err (">>>> %s: Debug ov5640 init mode: Line: %i", __LINE__);
			mipi_csi2_enable(mipi_csi2_info);
		}

		if (mipi_csi2_get_status(mipi_csi2_info)) {
			mipi_csi2_set_lanes(mipi_csi2_info);

			/*Only reset MIPI CSI2 HW at sensor initialize*/
			if (mode == ov5640_mode_INIT) 
				mipi_csi2_reset(mipi_csi2_info);


			mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_YUV422);  // JAD force type MIPI_DT_RGB888 or MIPI_DT_YUV422
			
		} else {
//			pr_err(">>>> %s: Can not enable mipi csi2 driver!\n");
			return -1;
		}
	} else {
//		printk(KERN_ERR "Fail to get mipi_csi2_info!\n");
		return -1;
	}

	dn_mode = ov5640_mode_info_data[frame_rate][mode].dn_mode;
	orig_dn_mode = ov5640_mode_info_data[frame_rate][orig_mode].dn_mode;
	if (mode == ov5640_mode_INIT) {
//                pModeSetting = ov5640_init_setting_30fps_VGA;         // JAD remove
//                ArySize = ARRAY_SIZE(ov5640_init_setting_30fps_VGA);  // JAD remove   

		  ov5640_data.pix.width  =  1368;   //  JAD was 640
		  ov5640_data.pix.height =   768;   //  JAD was 480
//        retval = ov5640_download_firmware(pModeSetting, ArySize);
		if (retval < 0){
			goto err;
		}

	} else if ((dn_mode == SUBSAMPLING && orig_dn_mode == SCALING) ||
			(dn_mode == SCALING && orig_dn_mode == SUBSAMPLING)) {
		/* change between subsampling and scaling
		 * go through exposure calucation */
//		retval = ov5640_change_mode_exposure_calc(frame_rate, mode);
	} else {
		/* change inside subsampling or scaling
		 * download firmware directly */
//		retval = ov5640_change_mode_direct(frame_rate, mode);
	}

	if (retval < 0)
		goto err;

	if (frame_rate == ov5640_15_fps) {
		/* dump the first nine frames: 1/15*9 */
		msec_wait4stable = 600;

	} else if (frame_rate == ov5640_30_fps) {           // JAD was 30 fps
		/* dump the first nine frames: 1/30*9 */
		msec_wait4stable = 300;                         // JAD was 300

	}
	msleep(msec_wait4stable);

	if (mipi_csi2_info) {
		unsigned int i;
//        pr_err (">>>> %s: Debug ov5640 init mode: Line: %i", __LINE__);
		i = 0;
		/* wait for mipi sensor ready */
		mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
//	        pr_err (">>>> %s: 0 try: %i csi2 dphy status: 0x%X", i, mipi_reg);
		while ((mipi_reg != 0x330) && (i < 10)) { //original == 0x200
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
//		        pr_err (">>>> %s: 1 try:%i csi2 dphy status: 0x%X", i, mipi_reg);
			i++;
			msleep(10);
		}
		if (i >= 10) {
//			pr_err(">>>> %s: mipi csi2 can not receive sensor clk!\n");
			return -1;
		}

		i = 0;

		/* wait for mipi stable */
		mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);

		while ((mipi_reg != 0x0) && (i < 10)) {
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
			i++;
			msleep(10);
		}
		if (i >= 10) {
			pr_err("mipi csi2 can not reveive data correctly!\n");
			return -1;
		}
	}
err:
	return retval;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ov5640_data.mclk;
	p->if_type = V4L2_IF_TYPE_BT656;	                    // JAD Change this?  was V4L2_IF_TYPE_BT656
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;      // Was V4L2_IF_TYPE_BT656_MODE_BT_8BIT
	p->u.bt656.clock_min = OV5640_XCLK_MIN;
	p->u.bt656.clock_max = OV5640_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;                         /* JAD Indicate external vsync - was 0 */

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;

	sensor->on = on;

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ov5640_frame_rate frame_rate;
	enum ov5640_mode orig_mode;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe ->denominator = DEFAULT_FPS;
			timeperframe ->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 15)
			frame_rate = ov5640_15_fps;
		else if (tgt_fps == 30)
			frame_rate = ov5640_30_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		orig_mode = sensor->streamcap.capturemode;
//		ret = ov5640_init_mode(frame_rate,
//				(u32)a->parm.capture.capturemode, orig_mode);
		if (ret < 0) {
//			pr_err(">>>> %s: ov5640 init mode error 1\n");
			return ret;
		}

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = ov5640_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = ov5640_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = ov5640_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = ov5640_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = ov5640_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ov5640_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = ov5640_data.ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;

	pr_debug("In ov5640:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > ov5640_mode_MAX)
		return -EINVAL;

	fsize->pixel_format = ov5640_data.pix.pixelformat;
	fsize->discrete.width =	1368;							    // JAD was 640
//	fsize->discrete.width =
//			max(ov5640_mode_info_data[0][fsize->index].width,
//			    ov5640_mode_info_data[1][fsize->index].width);
//	fsize->discrete.height = 
    fsize->discrete.height =  768;							    // JAD was 480
//			max(ov5640_mode_info_data[0][fsize->index].height,
//			    ov5640_mode_info_data[1][fsize->index].height);
	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
		"ov5640_mipi_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{

	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > ov5640_mode_MAX)
		return -EINVAL;

	fmt->pixelformat = ov5640_data.pix.pixelformat;

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor_data *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	int ret;
	enum ov5640_frame_rate frame_rate;
	void *mipi_csi2_info;

	ov5640_data.on = true;

	/* mclk */
	tgt_xclk = ov5640_data.mclk;

//	set_mclk_rate(&ov5640_data.mclk, ov5640_data.mclk_source);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /   // JAD
		  sensor->streamcap.timeperframe.numerator;          // JAD

	if (tgt_fps == 15)
		frame_rate = ov5640_15_fps;
	else if (tgt_fps == 30)
		frame_rate = ov5640_30_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	frame_rate = ov5640_30_fps;                              // JAD - force FPS
	mipi_csi2_info = mipi_csi2_get_info();

	/* enable mipi csi2 */
	if (mipi_csi2_info)
		mipi_csi2_enable(mipi_csi2_info);
	else {
		printk(KERN_ERR "%s() in %s: Fail to get mipi_csi2_info!\n",
		       __func__, __FILE__);
		return -EPERM;
	}

	ret = ov5640_init_mode(frame_rate, ov5640_mode_INIT, ov5640_mode_INIT);

	return ret;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	void *mipi_csi2_info;

	mipi_csi2_info = mipi_csi2_get_info();

	/* disable mipi csi2 */
	if (mipi_csi2_info)
		if (mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_disable(mipi_csi2_info);

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ov5640_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*) ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*) ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func*) ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
/*	{vidioc_int_enum_frameintervals_num,
			(v4l2_int_ioctl_func *) ioctl_enum_frameintervals},  */
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *) ioctl_g_chip_ident},
};

// Structure need for V4L2 interface
static struct v4l2_int_slave ov5640_slave = {
	.ioctls = ov5640_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov5640_ioctl_desc),
};

// Structure need for V4L2 interface
static struct v4l2_int_device ov5640_int_device = {
	.module = THIS_MODULE,
	.name   = "ov5640",
	.type   = v4l2_int_type_slave,
	.u      = {
		.slave = &ov5640_slave,
	},
};

/*!
 * ov5640 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov5640_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int retval;
	u8 chip_id_high, chip_id_low;

	/* request power down pin */
	pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(pwn_gpio)) {
		dev_warn(dev, "no sensor pwdn pin available");
		return -EINVAL;
	}
	retval = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_HIGH,
					"ov5640_mipi_pwdn");
	if (retval < 0)
		return retval;

	/* request reset pin */
	rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(rst_gpio)) {
		dev_warn(dev, "no sensor reset pin available");
		return -EINVAL;
	}
	retval = devm_gpio_request_one(dev, rst_gpio, GPIOF_OUT_INIT_HIGH,
					"ov5640_mipi_reset");
	if (retval < 0)
		return retval;

	/* Set initial values for the sensor struct. */
	memset(&ov5640_data, 0, sizeof(ov5640_data));
	ov5640_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(ov5640_data.sensor_clk)) {
		/* assuming clock enabled by default */
		ov5640_data.sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(ov5640_data.sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					&(ov5640_data.mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(ov5640_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(ov5640_data.csi));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}

	clk_prepare_enable(ov5640_data.sensor_clk);

//	ov5640_data.io_init = ov5640_reset;
	ov5640_data.i2c_client = client;
	ov5640_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	ov5640_data.pix.width  = 1368;                                   //JAD
	ov5640_data.pix.height =  768;                                   //JAD
	ov5640_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ov5640_data.streamcap.capturemode = 0;
	ov5640_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ov5640_data.streamcap.timeperframe.numerator = 1;

//	ov5640_power_on(dev);

//	ov5640_reset();

//	ov5640_standby(0);
	retval = 0;
//	retval = ov5640_read_reg(OV5640_CHIP_ID_HIGH_BYTE, &chip_id_high);
//	if (retval < 0 || chip_id_high != 0x56) {
//		pr_warning("camera ov5640_mipi is not found\n");
//		clk_disable_unprepare(ov5640_data.sensor_clk);
//		return -ENODEV;
//	}
//	retval = ov5640_read_reg(OV5640_CHIP_ID_LOW_BYTE, &chip_id_low);
//	if (retval < 0 || chip_id_low != 0x40) {
//		pr_warning("camera ov5640_mipi is not found\n");
//		clk_disable_unprepare(ov5640_data.sensor_clk);
//		return -ENODEV;
//	}

//	ov5640_standby(1);

	ov5640_int_device.priv = &ov5640_data;
	retval = v4l2_int_device_register(&ov5640_int_device);

	clk_disable_unprepare(ov5640_data.sensor_clk);

	pr_info("camera ov5640_mipi is found\n");
	return retval;
}

/*!
 * ov5640 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov5640_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&ov5640_int_device);
	return 0;
}

/*!
 * ov5640 init function
 * Called by insmod ov5640_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ov5640_init(void)
{
	u8 err;

	err = i2c_add_driver(&ov5640_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
			__func__, err);

	return err;
}

/*!
 * OV5640 cleanup function
 * Called on rmmod ov5640_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov5640_clean(void)
{
	i2c_del_driver(&ov5640_i2c_driver);
}

module_init(ov5640_init);
module_exit(ov5640_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("OV5640 MIPI Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
