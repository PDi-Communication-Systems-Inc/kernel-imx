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
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <mach/mipi_csi2.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"

// runtime parameters
int xres = 1360;
module_param(xres, int, 0);
int yres = 768;
module_param(yres, int, 0);
int pgon = 1;
module_param(pgon, int, 0);

#define DS90UB940_XCLK_MIN 6000000
#define DS90UB940_XCLK_MAX 24000000             // JAD was 24000000

// ds90ub940 registers
#define UB9XX_RESET		0x01
#define UB9XX_MAILBOX_18	0x18
#define UB9XX_PGCTL		0x64
#define UB9XX_PGCFG		0x65
#define UB9XX_PGIA		0x66
#define UB9XX_PGID		0x67
#define UB9XX_CSICFG1		0x6b
#define UB9XX_CSIIA		0x6c
#define UB9XX_CSIID		0x6d

// Pattern Generator Indirect registers
#define UB9XX_SET_PG_INDIRECT(reg, val) {UB9XX_PGIA, (reg), 0, 0}, {UB9XX_PGID, (val), 0, 0}

#define UB9XX_PGCDC		0x03
#define UB9XX_PGTFS1		0x04
#define UB9XX_PGTFS2		0x05
#define UB9XX_PGTFS3		0x06
#define UB9XX_PGAFS1		0x07
#define UB9XX_PGAFS2		0x08
#define UB9XX_PGAFS3		0x09
#define UB9XX_PGHSW		0x0a
#define UB9XX_PGVSW		0x0b
#define UB9XX_PGHBP		0x0c
#define UB9XX_PGVBP		0x0d
#define UB9XX_PGSC		0x0e

// CSI Indirect registers
#define UB9XX_CSI_VC_ID		0x2e

struct reg_value {
	u8	reg; 
	u8	val;
	u8	mask;
	u32	delayMs;
};

//
// JAD DS90ub740 Init Settings - out of reset - feed through
// 0x0018 is a general puropse mailbox location for testing 
// reads and writes.  CSI indirect address = 0x6c, data=0x6d, 
// For RGB888 instead of YUV422, change 0x006b from 0x50 to 0x00
static struct reg_value ub940_default_setting[] = {

	// reset the part
	{UB9XX_RESET, 0x02, 0, 0},
	// virtual channel ID=1 (uses indirect register access)
	{UB9XX_CSIIA, UB9XX_CSI_VC_ID}, {UB9XX_CSIID, 0x40, 0, 0},
	// YUV422 output format
	{UB9XX_CSICFG1, 0x50, 0, 0},
};

enum ds90ub940_mode {
        ds90ub940_mode_MIN                 = 0,
        ds90ub940_mode_default             = 0,
        ds90ub940_mode_MAX                 = 0,
        ds90ub940_mode_INIT                = 0xff, /*only for sensor init*/
};

struct ds90ub940_mode_info {
	enum ds90ub940_mode	mode;
	u32			width;
	u32			height;
	u32			pixelFormat;
	u32			fps;
	struct reg_value	*init_data_ptr;
	u32			init_data_size;
};

static struct ds90ub940_mode_info ds90ub940_modes[] = {
	// height and width filled out in probe() per module params
	[ds90ub940_mode_default].mode = ds90ub940_mode_default,
	[ds90ub940_mode_default].pixelFormat = V4L2_PIX_FMT_YUYV,
	[ds90ub940_mode_default].fps = 30,
	[ds90ub940_mode_default].init_data_ptr = ub940_default_setting,
	[ds90ub940_mode_default].init_data_size = ARRAY_SIZE(ub940_default_setting),
};

//
// Maintains the information on the current state of the sensor.
//
static struct sensor_data ds90ub940_data;

//
// forward declarations
//
static int ds90ub940_download_firmware(struct reg_value *pModeSetting, s32 ArySize);

// For Stripe bars instead of sequence, 0x0064, bit 2=1 (11->15)
static int ub940_pattern_generator(	u32 totFrameW, u32 totFrameH,
					u32 actFrameW, u32 actFrameH,
					u32 hSyncWidth, u32 vSyncWidth,
					u32 hBackPorch, u32 vBackPorch,
					u32 cLockDiv,	u32 enable) {
	int retval;

	struct reg_value config_regs[] = {
		// total frame size
		UB9XX_SET_PG_INDIRECT(UB9XX_PGTFS1, totFrameW & 0xff),
		UB9XX_SET_PG_INDIRECT(UB9XX_PGTFS2, ((totFrameH & 0x0f) << 4) | ((totFrameW >> 8) & 0x0f)),
		UB9XX_SET_PG_INDIRECT(UB9XX_PGTFS3, (totFrameH >> 4) & 0xff),
		// active frame size
		UB9XX_SET_PG_INDIRECT(UB9XX_PGAFS1, actFrameW & 0xff),
		UB9XX_SET_PG_INDIRECT(UB9XX_PGAFS2, ((actFrameH & 0x0f) << 4) | ((actFrameW >> 8) & 0x0f)),
		UB9XX_SET_PG_INDIRECT(UB9XX_PGAFS3, (actFrameH >> 4) & 0xff),
		// sync width
		UB9XX_SET_PG_INDIRECT(UB9XX_PGHSW, hSyncWidth),
		UB9XX_SET_PG_INDIRECT(UB9XX_PGVSW, vSyncWidth),
		// back porch
		UB9XX_SET_PG_INDIRECT(UB9XX_PGHBP, hBackPorch),
		UB9XX_SET_PG_INDIRECT(UB9XX_PGVBP, vBackPorch),
		// H sync - normal, V sync inverted
		UB9XX_SET_PG_INDIRECT(UB9XX_PGSC, 0x02),
		// 200 Mhz clock divider (3 = 66.7Mhz)
		UB9XX_SET_PG_INDIRECT(UB9XX_PGCDC, cLockDiv),
		// Color bars
		{UB9XX_PGCTL, 0x04, 0x04, 0},
		// internal timing
		{UB9XX_PGCFG, 0x04, 0x00, 0},
	};

	struct reg_value disable_regs[] = {
		{UB9XX_PGCTL, 0x00, 0x01, 0},
	};
	
	struct reg_value enable_regs[] = {
		{UB9XX_PGCTL, 0x01, 0x01, 0},
	};

	// disable
	retval = ds90ub940_download_firmware(disable_regs, ARRAY_SIZE(disable_regs));

	if(enable && !retval) {
		retval = ds90ub940_download_firmware(config_regs, ARRAY_SIZE(config_regs));
		if(!retval)
			retval = ds90ub940_download_firmware(enable_regs, ARRAY_SIZE(enable_regs));
	}

	return retval;
}


// write 1 byte register address, followed by 1 byte read of value
static s32 ub9xx_write_reg(u8 reg, u8 val)
{
	u8 au8Buf[2] = {reg, val};
	int err = i2c_master_send(ds90ub940_data.i2c_client, au8Buf, 2);
	return err > 0 ? 0 : err;
}

// write 1 byte register address, followed by 1 byte read of value
static s32 ub9xx_read_reg(u8 reg, u8 *val)
{
	int err = i2c_master_send(ds90ub940_data.i2c_client, &reg, 1);
	if(err > 0)
		err = i2c_master_recv(ds90ub940_data.i2c_client, val, 1);
	return err > 0 ? 0 : err;
}

/*
  This functions sets registers on the DS90UB940
*/
static int ds90ub940_download_firmware(struct reg_value *pModeSetting, s32 ArySize)
{
	int i = 0, retval = 0;
	u8 tmpVal;
	struct reg_value *fw = pModeSetting;

	for(i=0; i < ArySize; i++, fw++) {
		// mask? read first
		if(fw->mask) {
			retval = ub9xx_read_reg(fw->reg, &tmpVal);
			if(retval)
				break;
			tmpVal &= ~fw->mask;
			tmpVal |= fw->val & fw->mask;
		}
		else
			tmpVal = fw->val;

		retval = ub9xx_write_reg(fw->reg, tmpVal);
		if(retval)
			break;
	}

	return retval;
}

static int ds90ub940_init_mode(u32 frame_rate, enum ds90ub940_mode mode)
{
	int retval;
	void *mipi_csi2_info;
	u32 mipi_reg;

	pr_debug("\n\n>>>> ds90ub940_init_mode: build date = %s %s\n", __DATE__, __TIME__);
	pr_debug(">>>> ds90ub940_init_mode: frame rate=%i, mode=%i\n", frame_rate, mode);

	// only initialize / set ds90ub40 when mode is set
	if (mode != ds90ub940_mode_INIT) {
		ds90ub940_data.pix.width  =  ds90ub940_modes[mode].width;
		ds90ub940_data.pix.height =  ds90ub940_modes[mode].height;
		retval = ds90ub940_download_firmware(ds90ub940_modes[mode].init_data_ptr,
							ds90ub940_modes[mode].init_data_size);
		if(retval)
			return retval;
		if(pgon) {
			// TODO how should parms other than active frame size change??
			retval = ub940_pattern_generator(1792, 795,
				ds90ub940_data.pix.width, ds90ub940_data.pix.height, 112, 6, 255, 18, 3, 1);
			if(retval)
				return retval;
		}
	}

	mipi_csi2_info = mipi_csi2_get_info();

	/* initial mipi dphy */
	if (mipi_csi2_info) {

		if (!mipi_csi2_get_status(mipi_csi2_info))
		{
			pr_debug(">>>> %s: Debug ds90ub940 init mode: Line: %i",__func__, __LINE__);
			mipi_csi2_enable(mipi_csi2_info);
		}

		if (mipi_csi2_get_status(mipi_csi2_info)) {
			mipi_csi2_set_lanes(mipi_csi2_info);

			/*Only reset MIPI CSI2 HW at sensor initialize*/
			if (mode == ds90ub940_mode_INIT) {
				mipi_csi2_reset(mipi_csi2_info);
			}

			mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_YUV422);  // JAD force type MIPI_DT_RGB888 or MIPI_DT_YUV422
			
		} else {
			pr_err(">>>> %s: Can not enable mipi csi2 driver!\n",__func__);
			return -1;
		}
	} else {
		pr_err("Fail to get mipi_csi2_info!\n");
		return -1;
	}

	// dump the first 9 frames
	if(frame_rate)
		msleep(9000 / frame_rate);

	/*
	Per section 40.6.6 of IMX6DQRM, the MIPI_CSI_PHY_STATE bits:

	31–12 This field is reserved.
	   11 Payload Bypass test mode for double ECC errors
	   10 Clock Lane in Stop state
	    9 Active Low. This signal indicates that the Clock Lane module
	      has entered the Ultra Low Power state
	    8 Indicates that the clock lane is actively receiving a DDR clock
	    7 Data Lane 3 in Stop state
	    6 Data Lane 2 in Stop state
	    5 Data Lane 1 in Stop state
	    4 Data Lane 0 in Stop state
	    3 Lane module 3 has entered the Ultra Low Power mode
	    2 Lane module 2 has entered the Ultra Low Power mode
	    1 Lane module 1 has entered the Ultra Low Power mode
	    0 Lane module 0 has entered the Ultra Low Power mode

	Observed status:
	   no clock from ds90ub940 - 0x200, with pattern generator internal clock - 0x300 or 0x6f0

	Note: since there is no bit that indicates "sensor clock available", checking b9 == 1
	*/

	if (mipi_csi2_info) {
		unsigned int i = 0;
	        pr_debug(">>>> %s: Debug ds90ub940 init mode: Line: %i", __func__,__LINE__);
		/* wait for mipi sensor ready */
		mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info) & 0x200;
	        pr_debug(">>>> %s: 0 try: %i csi2 dphy status: 0x%X", __func__,i, mipi_reg);
		while ((mipi_reg != 0x200) && (i < 10)) { //was changed to 0x330
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info) & 0x200;
		        pr_debug(">>>> %s: 1 try:%i csi2 dphy status: 0x%X", __func__,i, mipi_reg);
			i++;
			msleep(10);
		}
		if (i >= 10) {
			pr_err(">>>> %s: mipi csi2 can not receive sensor clk!\n",__func__);
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
			pr_err(">>>> %s: mipi csi2 can not reveive data correctly!\n",__func__);
			return -1;
		}
	}

	return 0;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err(">>>> %s: ERROR!! no slave device set!\n", __func__);
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ds90ub940_data.mclk;
	p->if_type = V4L2_IF_TYPE_BT656;	                    // JAD Change this?  was V4L2_IF_TYPE_BT656
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_BT_8BIT;      // Was V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT
	p->u.bt656.clock_min = DS90UB940_XCLK_MIN;
	p->u.bt656.clock_max = DS90UB940_XCLK_MAX;
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
 * Configures the sensor to use the requested mode.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	u32 new_mode = (u32)a->parm.capture.capturemode;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:

		if(new_mode > ds90ub940_mode_MAX) {
			// TODO once the higher level (DeviceAdapter() / OvDevice()) 
			// is fixed, change to error. In the meantime set to default
			new_mode = ds90ub940_mode_default;
			//pr_err(">>>> %s: invalid mode: %d\n",__func__, new_mode);
			//return -EINVAL;
		}

		ret = ds90ub940_init_mode(ds90ub940_modes[new_mode].fps, new_mode);
		if (ret < 0) {
			pr_err(">>>> %s: ds90ub940 init mode error 1\n",__func__);
			return ret;
		}

		// ignore new FPS change - since we said it's not a capability
		sensor->streamcap.timeperframe.numerator = 1;
		sensor->streamcap.timeperframe.denominator = ds90ub940_modes[new_mode].fps;
		sensor->streamcap.capturemode = new_mode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n", a->type);
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
	if (fsize->index > ds90ub940_mode_MAX)
		return -EINVAL;

	fsize->pixel_format = ds90ub940_modes[fsize->index].pixelFormat;
	fsize->discrete.width =	ds90ub940_modes[fsize->index].width;
	fsize->discrete.height =  ds90ub940_modes[fsize->index].height;

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
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "ds90ub940");

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
	if (fmt->index > ds90ub940_mode_MAX)
		return -EINVAL;

	fmt->pixelformat = ds90ub940_modes[fmt->index].pixelFormat;

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
	u32 tgt_xclk;	/* target xclk */
	int ret;
	void *mipi_csi2_info;

	ds90ub940_data.on = true;

	/* mclk */
	tgt_xclk = ds90ub940_data.mclk;

	set_mclk_rate(&ds90ub940_data.mclk, ds90ub940_data.mclk_source);

	mipi_csi2_info = mipi_csi2_get_info();

	/* enable mipi csi2 */
	if (mipi_csi2_info)
		mipi_csi2_enable(mipi_csi2_info);
	else {
		printk(KERN_ERR "Fail to get mipi_csi2_info!\n");
		return -EPERM;
	}

	// this will init mipi, not ds90ub940
	ret = ds90ub940_init_mode(0, ds90ub940_mode_INIT);

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
static struct v4l2_int_ioctl_desc ds90ub940_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*) ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*) ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},
	{vidioc_int_init_num, (v4l2_int_ioctl_func*) ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *) ioctl_g_chip_ident},
};

// Structure need for V4L2 interface
static struct v4l2_int_slave ds90ub940_slave = {
	.ioctls = ds90ub940_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ds90ub940_ioctl_desc),
};

// Structure need for V4L2 interface
static struct v4l2_int_device ds90ub940_int_device = {
	.module = THIS_MODULE,
	.name   = "ds90ub940",
	.type   = v4l2_int_type_slave,
	.u      = {
		.slave = &ds90ub940_slave,
	},
};

//
// ds90ub940 I2C probe function
//
// @param adapter            struct i2c_adapter *
// @return  Error code indicating success or failure
// 
static int ds90ub940_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int retval;
	u8 regval;

	struct fsl_mxc_camera_platform_data *plat_data = client->dev.platform_data;

	pr_debug("%s: >>>>  ds90ub940 camera probe function\n", __func__);

	// mode height and width per module params
	ds90ub940_modes[ds90ub940_mode_default].width = xres;
	ds90ub940_modes[ds90ub940_mode_default].height = yres;

	/* Set initial values for the sensor struct. */
	memset(&ds90ub940_data, 0, sizeof(ds90ub940_data));
	ds90ub940_data.mclk = plat_data->mclk;
	ds90ub940_data.mclk_source = plat_data->mclk_source;
	ds90ub940_data.csi = plat_data->csi;
	ds90ub940_data.io_init = plat_data->io_init;
	ds90ub940_data.i2c_client = client;

	// see if chip is answering i2c, reset then write / read back
	retval = ub9xx_write_reg(UB9XX_RESET, 0x02);
	if(!retval)
		retval = ub9xx_write_reg(UB9XX_MAILBOX_18, 0xed);
	if(!retval)
		retval = ub9xx_read_reg(UB9XX_MAILBOX_18, &regval);
	if(retval || (regval != 0xed)) {
		pr_err("%s i2c failure. err=%d val=0x%02x\n", __func__, retval, regval);
		return retval < 0 ? retval : -EINVAL;
	}

	// set everything to default mode
	ds90ub940_data.pix.pixelformat = ds90ub940_modes[ds90ub940_mode_default].pixelFormat;
	ds90ub940_data.pix.width  = ds90ub940_modes[ds90ub940_mode_default].width;
	ds90ub940_data.pix.height  = ds90ub940_modes[ds90ub940_mode_default].height;
	ds90ub940_data.streamcap.capturemode = ds90ub940_mode_default;
	ds90ub940_data.streamcap.timeperframe.denominator = ds90ub940_modes[ds90ub940_mode_default].fps;
	ds90ub940_data.streamcap.timeperframe.numerator = 1;

	if (plat_data->io_init)
		plat_data->io_init();

	if (plat_data->pwdn)
		plat_data->pwdn(0);

	ds90ub940_int_device.priv = &ds90ub940_data;                  // need this line
	retval = v4l2_int_device_register(&ds90ub940_int_device);  // need this line

	return retval;
}

/*!
 * ds90ub940 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ds90ub940_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&ds90ub940_int_device);
	return 0;
}

static const struct i2c_device_id ds90ub940_id[] = {
	{"ov5640_mipi", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ds90ub940_id);

// 
static struct i2c_driver ds90ub940_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov5640_mipi",
		  },
	.probe    = ds90ub940_probe,
	.remove   = ds90ub940_remove,
	.id_table = ds90ub940_id,
};

/*!
 * ds90ub940 init function
 * Called by insmod ds90ub940_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ds90ub940_init(void)
{
	u8 err;
	err = i2c_add_driver(&ds90ub940_i2c_driver);
	if (err != 0)
		pr_err(">>>> %s: driver registration failed, error=%d\n",__func__, err); // JAD

	return err;
}

/*!
 * DS90UB940 cleanup function
 * Called on rmmod ds90ub940_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ds90ub940_clean(void)
{
	i2c_del_driver(&ds90ub940_i2c_driver);
}

module_init(ds90ub940_init);
module_exit(ds90ub940_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("DS90UB940 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");

