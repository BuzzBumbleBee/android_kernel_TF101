/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  $
 */

/**
 *  @defgroup   COMPASSDL (Motion Library - Accelerometer Driver Layer)
 *  @brief      Provides the interface to setup and handle an accelerometers
 *              connected to the secondary I2C interface of the gyroscope.
 *
 *  @{
 *     @file   ami30x.c
 *     @brief  Magnetometer setup and handling methods for Aichi AMI304/AMI305
 *             compass.
*/

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#ifdef __KERNEL__
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#endif

#include "mpu.h"
#include "mlsl.h"
#include "mlos.h"

#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-compass"

#define AMI30X_REG_DATAX (0x10)
#define AMI30X_REG_STAT1 (0x18)
#define AMI30X_REG_CNTL1 (0x1B)
#define AMI30X_REG_CNTL2 (0x1C)
#define AMI30X_REG_CNTL3 (0x1D)

#define AMI30X_BIT_CNTL1_PC1  (0x80)
#define AMI30X_BIT_CNTL1_ODR1 (0x10)
#define AMI30X_BIT_CNTL1_FS1  (0x02)

#define AMI30X_BIT_CNTL2_IEN  (0x10)
#define AMI30X_BIT_CNTL2_DREN (0x08)
#define AMI30X_BIT_CNTL2_DRP  (0x04)
#define AMI30X_BIT_CNTL3_F0RCE (0x40)

#define AMI30X_CALIBRATION_PATH "/data/sensors/AMI304_Config.ini"

static int gain_x = 100, gain_y = 100, gain_z = 100;
static int offset_x = 0, offset_y = 0, offset_z = 0;
static bool flagLoadConfig = false;

static int access_calibration_file(void)
{
	char buf[256];
        int ret = 0;
	struct file *fp = NULL;
	mm_segment_t oldfs;
	int data[23];

	oldfs=get_fs();
	set_fs(get_ds());
	memset(buf, 0, sizeof(u8)*256);

	fp=filp_open(AMI30X_CALIBRATION_PATH, O_RDONLY, 0);
	if (!IS_ERR(fp)) {
		printk("ami304 open config file success\n");
		ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
		printk("ami304 config content is :%s\n", buf);
		sscanf(buf,"%6d\n%6d %6d %6d\n%6d %6d %6d\n%6d %6d %6d\n%6d %6d %6d\n%6d %6d %6d\n%6d %6d %6d\n%6d %6d %6d\n%6d",
			&data[0],
			&data[1], &data[2], &data[3],
			&data[4], &data[5], &data[6],
			&data[7], &data[8], &data[9],
			&data[10], &data[11], &data[12],
			&data[13], &data[14], &data[15],
			&data[16], &data[17], &data[18],
			&data[19], &data[20], &data[21],
			&data[22]);

		printk("%d %d %d\n", data[19], data[20], data[21]);
		if((data[19] != 100) ||
		   (data[20] > 150) || (data[20] < 50) ||
		   (data[21] > 150) || (data[21] < 50)){
			gain_x = 100;
			gain_y = 100;
			gain_z = 100;
		}else{
			gain_x = data[19];
			gain_y = data[20];
			gain_z = data[21];
		}
		printk("%d %d %d\n", gain_x, gain_y, gain_z);

		filp_close(fp, NULL);
		set_fs(oldfs);
		return 0;
	}
	else{
		printk("No ami304 calibration file\n");
		set_fs(oldfs);
		return -1;
	}

}

int ami30x_suspend(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{
	int result;
	unsigned char reg;
	printk("%s+\n", __func__);
	result =
	    MLSLSerialRead(mlsl_handle, pdata->address, AMI30X_REG_CNTL1,
			   1, &reg);
	ERROR_CHECK(result);

	reg &= ~(AMI30X_BIT_CNTL1_PC1|AMI30X_BIT_CNTL1_FS1);
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  AMI30X_REG_CNTL1, reg);
	ERROR_CHECK(result);
	printk("%s-\n", __func__);
	return result;
}

int ami30x_resume(void *mlsl_handle,
		  struct ext_slave_descr *slave,
		  struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	printk("%s+\n", __func__);
	/* Set CNTL1 reg to power model active */
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  AMI30X_REG_CNTL1,
				  AMI30X_BIT_CNTL1_PC1|AMI30X_BIT_CNTL1_FS1);
	ERROR_CHECK(result);
	/* Set CNTL2 reg to DRDY active high and enabled */
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  AMI30X_REG_CNTL2,
				  AMI30X_BIT_CNTL2_DREN |
				  AMI30X_BIT_CNTL2_DRP);
	ERROR_CHECK(result);
	/* Set CNTL3 reg to forced measurement period */
	result =
		MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  AMI30X_REG_CNTL3, AMI30X_BIT_CNTL3_F0RCE);
	printk("%s-\n", __func__);
	return result;
}

int ami30x_read(void *mlsl_handle,
		struct ext_slave_descr *slave,
		struct ext_slave_platform_data *pdata, unsigned char *data)
{
	unsigned char stat;
	int result = ML_SUCCESS;
	int x = 0, y = 0, z = 0;
	/* Read status reg and check if data ready (DRDY) */
	result =
	    MLSLSerialRead(mlsl_handle, pdata->address, AMI30X_REG_STAT1,
			   1, &stat);
	ERROR_CHECK(result);

	if (stat & 0x40) {
		if(!flagLoadConfig){
			access_calibration_file();
			flagLoadConfig = true;
		}
		result =
		    MLSLSerialRead(mlsl_handle, pdata->address,
				   AMI30X_REG_DATAX, 6,
				   (unsigned char *) data);
		ERROR_CHECK(result);
		/* start another measurement */
		result =
			MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					      AMI30X_REG_CNTL3,
					      AMI30X_BIT_CNTL3_F0RCE);
		ERROR_CHECK(result);
		//printk("%02x%02x %02x%02x %02x%02x\n", data[1], data[0], data[3], data[2], data[5], data[4]);
		x =  ((short)(data[1] << 8 | data[0]))*gain_x/100;
		y =  ((short)(data[3] << 8 | data[2]))*gain_y/100;
		z =  ((short)(data[5] << 8 | data[4]))*gain_z/100;
		if ( x >= 2047)
			x = 2047;
		if ( x < -2048)
			x = -2048;
	        if ( y >= 2047)
                        y = 2047;
                if ( y < -2048)
                        y = -2048;
	        if ( z >= 2047)
                        z = 2047;
                if ( z < -2048)
                        z = -2048;

		//printk("x : %d y : %d z : %d\n", x, y, z);
		data[0] = x & 0x000000FF;
		data[1] = ( x & 0x0000FF00) >> 8;
                data[2] = y & 0x000000FF;
                data[3] = ( y & 0x0000FF00) >> 8;
                data[4] = z & 0x000000FF;
                data[5] = ( z & 0x0000FF00) >> 8;
		//printk("%02x%02x %02x%02x %02x%02x\n", data[1], data[0], data[3], data[2], data[5], data[4]);

		return ML_SUCCESS;
	}

	return ML_ERROR_COMPASS_DATA_NOT_READY;
}

struct ext_slave_descr ami30x_descr = {
	/*.init             = */ NULL,
	/*.exit             = */ NULL,
	/*.suspend          = */ ami30x_suspend,
	/*.resume           = */ ami30x_resume,
	/*.read             = */ ami30x_read,
	/*.config           = */ NULL,
	/*.get_config       = */ NULL,
	/*.name             = */ "ami304",
	/*.type             = */ EXT_SLAVE_TYPE_COMPASS,
	/*.id               = */ COMPASS_ID_AMI30X,
	/*.reg              = */ 0x06,
	/*.len              = */ 6,
	/*.endian           = */ EXT_SLAVE_LITTLE_ENDIAN,
	/*.range            = */ {5461, 3333}
	/* For AMI305,the range field needs to be modified to {9830.4f}*/
};

struct ext_slave_descr *ami30x_get_slave_descr(void)
{
	return &ami30x_descr;
}
EXPORT_SYMBOL(ami30x_get_slave_descr);

/**
 *  @}
**/
