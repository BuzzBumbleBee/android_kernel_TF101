/* drivers/i2c/chips/ami304.c - AMI304 compass driver
 *
 * Copyright (C) 2009 AMIT Technology Inc.
 * Author: Kyle Chen <sw-support@amit-inc.com>
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
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/ami304.h>
#include <linux/kobject.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>

#define DEBUG 1

#define AMI304_DRV_NAME			"ami304"
#define DRIVER_VERSION			"2.0.13.21"

#define START_NORMAL (HZ/5)
#define START_HEAVY  (HZ/200)

static int poll_mode = 0;
static struct i2c_client *ami304_i2c_client = NULL;
static struct work_struct ami304_readmeasure_work;
static struct delayed_work ami304_i2c_test_work ;
static struct workqueue_struct *sensor_work_queue = NULL;

/* Insmod parameters */
I2C_CLIENT_INSMOD;

struct _ami304_data {
	rwlock_t lock;
	int chipset;
	int mode;
	int rate;
	int status;
	volatile int updated;
	unsigned char raw_data[6];	
} ami304_data;

typedef struct {
	int x;
	int y;
	int z;
}ami304_vec_t;
typedef struct {
	unsigned long pedo_step;
	unsigned long pedo_time;
	int pedo_stat;
}ami304_pedo_t;
struct _ami304mid_data {
	rwlock_t datalock;
	rwlock_t ctrllock;	
	int controldata[AMI304_CB_LENGTH];	
	int pedometerparam[AMI304_PD_LENGTH];
	int yaw;
	int roll;
	int pitch;
	ami304_vec_t nm;
	ami304_vec_t na;
	ami304_vec_t gyro;
	ami304_pedo_t pedo;	
	int status;
} ami304mid_data;

struct ami304_i2c_data {
	struct input_dev *input_dev;
	struct i2c_client *client;
	int irq;
	int gpio;
};

static atomic_t dev_open_count;
static atomic_t hal_open_count;
static atomic_t daemon_open_count;

static int AMI304_Chipset_Init(int mode, int chipset)
{
	u8 databuf[10];
	u8 regaddr;
	u8 ctrl1, ctrl2, ctrl3;
	unsigned char ctrl4[2];
	
	regaddr = AMI304_REG_CTRL1;
	i2c_master_send(ami304_i2c_client, &regaddr, 1);
	i2c_master_recv(ami304_i2c_client, &ctrl1, 1);

	regaddr = AMI304_REG_CTRL2;
	i2c_master_send(ami304_i2c_client, &regaddr, 1);
	i2c_master_recv(ami304_i2c_client, &ctrl2, 1);
	
	regaddr = AMI304_REG_CTRL3;
	i2c_master_send(ami304_i2c_client, &regaddr, 1);
	i2c_master_recv(ami304_i2c_client, &ctrl3, 1);		
	regaddr = AMI304_REG_CTRL4; //2 bytes
	i2c_master_send(ami304_i2c_client, &regaddr, 1);
	i2c_master_recv(ami304_i2c_client, &(ctrl4[0]), 2);
	
	databuf[0] = AMI304_REG_CTRL1;
	if( mode==AMI304_FORCE_MODE )
	{
		databuf[1] = ctrl1 | AMI304_CTRL1_PC1 | AMI304_CTRL1_FS1_FORCE;
		write_lock(&ami304_data.lock);
		ami304_data.mode = AMI304_FORCE_MODE;
		write_unlock(&ami304_data.lock);			
	}
	else	
	{
		databuf[1] = ctrl1 | AMI304_CTRL1_PC1 | AMI304_CTRL1_FS1_NORMAL | AMI304_CTRL1_ODR1;
		write_lock(&ami304_data.lock);
		ami304_data.mode = AMI304_NORMAL_MODE;
		write_unlock(&ami304_data.lock);			
	}
	i2c_master_send(ami304_i2c_client, databuf, 2);		
	
	databuf[0] = AMI304_REG_CTRL2;
	databuf[1] = ctrl2 | AMI304_CTRL2_DREN | AMI304_CTRL2_DRP;
	i2c_master_send(ami304_i2c_client, databuf, 2);		
	
	databuf[0] = AMI304_REG_CTRL3;
	databuf[1] = ctrl3 | AMI304_CTRL3_B0_LO_CLR;
	i2c_master_send(ami304_i2c_client, databuf, 2);				
	databuf[0] = AMI304_REG_CTRL4;	
	if( chipset == AMI304_CHIPSET ) //AMI304
	{
		ctrl4[1]   = ctrl4[1] & AMI304_CTRL4_COMPASS_MODE; 	 //0x5D
	}
	else	//AMI306
	{
		ctrl4[1]   = ctrl4[1] | AMI306_CTRL4_HIGHSPEED_MODE; //0x5D		
	}	
	databuf[1] = ctrl4[0];
	databuf[2] = ctrl4[1];
	i2c_master_send(ami304_i2c_client, databuf, 3);				
	return 0;
}

static int AMI304_SetMode(int newmode)
{
	int mode = 0;
	int chipset = 0;
	
	read_lock(&ami304_data.lock);
	mode = ami304_data.mode;
	chipset = ami304_data.chipset;
	read_unlock(&ami304_data.lock);		
	
	if (mode == newmode) 
		return 0;	
	
	if( newmode==AMI304_FORCE_MODE )
		flush_scheduled_work();
				
	return AMI304_Chipset_Init(newmode, chipset);
}

static int AMI304_ReadChipInfo(char *buf, int bufsize)
{
	if ((!buf)||(bufsize<=30))
		return -1;
		
	if (!ami304_i2c_client)
	{
		*buf = 0;
		return -2;
	}

	if (ami304_data.chipset == AMI306_CHIPSET)	
	{
		sprintf(buf, "AMI306 Chip");
	}
	else
	{
		sprintf(buf, "AMI304 Chip");
	}

	return 0;
}

static int AMI304_WIA(char *wia, int bufsize)
{
	char cmd;
	unsigned char databuf[10];

	if ((!wia)||(bufsize<=30))
		return -1;	
		
	if (!ami304_i2c_client)
	{
		*wia = 0;
		return -2;
	}

	cmd = AMI304_REG_WIA;
	i2c_master_send(ami304_i2c_client, &cmd, 1);	
	udelay(20);
	i2c_master_recv(ami304_i2c_client, &(databuf[0]), 1);	
	
	sprintf(wia, "%02x", databuf[0]);
	
	return 0;
}

static int Identify_AMI_Chipset(void)
{
	char strbuf[AMI304_BUFSIZE];
	int WIARet = 0;
	int ret;
	
	if( (ret=AMI304_WIA(strbuf, AMI304_BUFSIZE))!=0 )
		return ret;
		
	sscanf(strbuf, "%x", &WIARet);	
	
	if (WIARet == AMI306_WIA_VALUE)	
	{
		ami304_data.chipset = AMI306_CHIPSET;
	}
	else if (WIARet == AMI304_WIA_VALUE)
	{
		ami304_data.chipset = AMI304_CHIPSET;
	}
	else
		return -1;
	
	return 0;
}

static int AMI304_ReadSensorDataFromChip(void)
{
	char cmd;
	int res = 0;
	unsigned char databuf[6];
	
	memset(databuf, 0, sizeof(unsigned char)*6);
	if (!ami304_i2c_client)
	{
		return -2;
	}
	
	cmd = AMI304_REG_DATAXH;
	res = i2c_master_send(ami304_i2c_client, &cmd, 1);	
	if (res<=0)
		goto exit_AMI304_ReadSensorDataFromChip;
	udelay(20);		
	res = i2c_master_recv(ami304_i2c_client, databuf, 6);
	if (res<=0)
		goto exit_AMI304_ReadSensorDataFromChip;

        write_lock(&ami304_data.lock);
	memcpy(&ami304_data.raw_data[0], databuf, sizeof(unsigned char)*6);
	ami304_data.updated = 1;
	write_unlock(&ami304_data.lock);

exit_AMI304_ReadSensorDataFromChip:	
	if (res<=0) {
		return -1;
	}
	return 0;
}

static int AMI304_ReadSensorDataForceMode(void)
{
	int res = 0;
	int updated = 0;
    unsigned char databuf[6];
	unsigned char RetryCount = 0;

    memset(databuf, 0, sizeof(unsigned char)*6);
	if (!ami304_i2c_client)
	{
		return -2;
	}
	
	databuf[0] = AMI304_REG_CTRL3;
	databuf[1] = AMI304_CTRL3_FORCE_BIT;
	res = i2c_master_send(ami304_i2c_client, databuf, 2);	

	if (res<=0)
		goto exit_AMI304_ReadSensorDataForceMode;	
	
	write_lock(&ami304_data.lock);
	ami304_data.updated = 0;
	write_unlock(&ami304_data.lock);

	do	{	
		msleep(1);	
		read_lock(&ami304_data.lock);
		updated = ami304_data.updated;		
		read_unlock(&ami304_data.lock);	
		RetryCount++;
	} while (updated == 0 && RetryCount<5);	
	
	if(RetryCount>=5)
		printk(KERN_ERR "ami304 interrput timeout\n");
	
exit_AMI304_ReadSensorDataForceMode:	
	if (res<=0) {
		return -1;
	}
		
	return 0;
}

static int AMI304_ReadSensorData(char *buf, int bufsize)
{
	int mode = 0;	
	int res;
	unsigned char databuf[6];

	if ((!buf)||(bufsize<=80))
		return -1;
	
	memset(databuf, 0, sizeof(unsigned char)*6);	
	if (!ami304_i2c_client)
	{
		*buf = 0;
		return -2;
	}
		
	read_lock(&ami304_data.lock);	
	mode = ami304_data.mode;
	read_unlock(&ami304_data.lock);		

	if (mode == AMI304_FORCE_MODE)
	{
		res = AMI304_ReadSensorDataForceMode();	
		if (res<0)
		{
			sprintf(buf, "Error in reading sensor!");		
			return res;
		}		
	} 	
	
	read_lock(&ami304_data.lock);	
	memcpy(databuf, &ami304_data.raw_data[0], sizeof(unsigned char)*6);
	read_unlock(&ami304_data.lock);	
	sprintf(buf, "%02x %02x %02x %02x %02x %02x", databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);	
	return 0;
}

static int AMI304_ReadPostureData(char *buf, int bufsize)
{
	if ((!buf)||(bufsize<=80))
		return -1;

	read_lock(&ami304mid_data.datalock);
	sprintf(buf, "%d %d %d %d", ami304mid_data.yaw, ami304mid_data.pitch, ami304mid_data.roll, ami304mid_data.status);
	read_unlock(&ami304mid_data.datalock);
	return 0;
}

static int AMI304_ReadCaliData(char *buf, int bufsize)
{
	if ((!buf)||(bufsize<=80))
		return -1;

	read_lock(&ami304mid_data.datalock);
	sprintf(buf, "%d %d %d %d %d %d %d", ami304mid_data.nm.x, ami304mid_data.nm.y, ami304mid_data.nm.z,ami304mid_data.na.x,ami304mid_data.na.y,ami304mid_data.na.z,ami304mid_data.status);
	read_unlock(&ami304mid_data.datalock);
	return 0;
}

static int AMI304_ReadGyroData(char *buf, int bufsize)
{
	if ((!buf)||(bufsize<=80))
		return -1;

	read_lock(&ami304mid_data.datalock);
	sprintf(buf, "%d %d %d", ami304mid_data.gyro.x, ami304mid_data.gyro.y, ami304mid_data.gyro.z);
	read_unlock(&ami304mid_data.datalock);
	return 0;	
}

static int AMI304_ReadPedoData(char *buf, int bufsize)
{
	if ((!buf)||(bufsize<=80))
		return -1;

	read_lock(&ami304mid_data.datalock);
	sprintf(buf, "%ld %ld %d", ami304mid_data.pedo.pedo_step, ami304mid_data.pedo.pedo_time, ami304mid_data.pedo.pedo_stat);
	read_unlock(&ami304mid_data.datalock);
	return 0;
}

static int AMI304_ReadMiddleControl(char *buf, int bufsize)
{
	if ((!buf)||(bufsize<=80))
		return -1;

	read_lock(&ami304mid_data.ctrllock);
	sprintf(buf, "%d %d %d %d %d %d %d %d %d %d", 
		ami304mid_data.controldata[AMI304_CB_LOOPDELAY], ami304mid_data.controldata[AMI304_CB_RUN], ami304mid_data.controldata[AMI304_CB_ACCCALI], ami304mid_data.controldata[AMI304_CB_MAGCALI],
		ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS], ami304mid_data.controldata[AMI304_CB_PD_RESET], ami304mid_data.controldata[AMI304_CB_PD_EN_PARAM], ami304mid_data.controldata[AMI304_CB_UNDEFINE_1],
		ami304mid_data.controldata[AMI304_CB_UNDEFINE_2], ami304mid_data.controldata[AMI304_CB_UNDEFINE_3] );
	read_unlock(&ami304mid_data.ctrllock);
	return 0;
}

static int AMI304_Report_Value(int iEnable)
{
	struct ami304_i2c_data *data = i2c_get_clientdata(ami304_i2c_client);
	
	if( !iEnable )
		return -1;
		
	input_report_abs(data->input_dev, ABS_RX, ami304mid_data.yaw);	/* yaw */
	input_report_abs(data->input_dev, ABS_RY, ami304mid_data.pitch);/* pitch */
	input_report_abs(data->input_dev, ABS_RZ, ami304mid_data.roll);/* roll */
	input_report_abs(data->input_dev, ABS_RUDDER, ami304mid_data.status);/* status of orientation sensor */

	input_report_abs(data->input_dev, ABS_X, ami304mid_data.na.x);/* x-axis raw acceleration */
	input_report_abs(data->input_dev, ABS_Y, ami304mid_data.na.y);/* y-axis raw acceleration */
	input_report_abs(data->input_dev, ABS_Z, ami304mid_data.na.z);/* z-axis raw acceleration */

	input_report_abs(data->input_dev, ABS_HAT0X, ami304mid_data.nm.x);/* x-axis of raw magnetic vector */
	input_report_abs(data->input_dev, ABS_HAT0Y, ami304mid_data.nm.y);/* y-axis of raw magnetic vector */
	input_report_abs(data->input_dev, ABS_BRAKE, ami304mid_data.nm.z);/* z-axis of raw magnetic vector */
	input_report_abs(data->input_dev, ABS_WHEEL, ami304mid_data.status);/* status of magnetic sensor */

	input_report_abs(data->input_dev, ABS_HAT1X, ami304mid_data.gyro.x);/* x-axis of gyro sensor */
	input_report_abs(data->input_dev, ABS_HAT1Y, ami304mid_data.gyro.y);/* y-axis of gyro sensor */
	input_report_abs(data->input_dev, ABS_THROTTLE, ami304mid_data.gyro.z);/* z-axis of gyro sensor */
	
	input_sync(data->input_dev);
	   
	return 0;
}

static ssize_t show_chipinfo_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadChipInfo(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);		
}

static ssize_t show_sensordata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);			
}

static ssize_t show_posturedata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadPostureData(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);			
}

static ssize_t show_calidata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadCaliData(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);			
}

static ssize_t show_gyrodata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadGyroData(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);			
}
static ssize_t show_midcontrol_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadMiddleControl(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);			
}

static ssize_t store_midcontrol_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	write_lock(&ami304mid_data.ctrllock);
	memcpy(&ami304mid_data.controldata[0], buf, sizeof(int)*AMI304_CB_LENGTH);	
 	write_unlock(&ami304mid_data.ctrllock);		
	return count;			
}

static ssize_t show_mode_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int mode=0;
	read_lock(&ami304_data.lock);
	mode = ami304_data.mode;
	read_unlock(&ami304_data.lock);		
	return sprintf(buf, "%d\n", mode);			
}

static ssize_t store_mode_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int mode = 0;
	sscanf(buf, "%d", &mode);	
 	AMI304_SetMode(mode);
	return count;			
}

static ssize_t show_wia_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_WIA(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);			
}

static ssize_t show_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ami304_data.status);			
}

static DEVICE_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DEVICE_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DEVICE_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DEVICE_ATTR(calidata, S_IRUGO, show_calidata_value, NULL);
static DEVICE_ATTR(gyrodata, S_IRUGO, show_gyrodata_value, NULL);
static DEVICE_ATTR(midcontrol, S_IRUGO | S_IWUSR, show_midcontrol_value, store_midcontrol_value );
static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_mode_value, store_mode_value );
static DEVICE_ATTR(wia, S_IRUGO, show_wia_value, NULL);
static DEVICE_ATTR(compass_status, S_IRUGO, show_status, NULL);

static struct attribute *ami304_attributes[] = {
	&dev_attr_chipinfo.attr,
	&dev_attr_sensordata.attr,
	&dev_attr_posturedata.attr,
	&dev_attr_calidata.attr,
	&dev_attr_gyrodata.attr,
	&dev_attr_midcontrol.attr,
	&dev_attr_mode.attr,
	&dev_attr_wia.attr,
	&dev_attr_compass_status.attr,
	NULL
};

static struct attribute_group ami304_attribute_group = {
	.attrs = ami304_attributes
};


static void ami304_i2c_test(struct work_struct *work) //(i)pointer of work_struct
{
	int ret = 0;
	unsigned char buf[6];
	unsigned char wia;
	char cmd;
	unsigned char databuf[2];

	databuf[0] = AMI304_REG_CTRL3;
	databuf[1] = AMI304_CTRL3_FORCE_BIT;
	ret = i2c_master_send(ami304_i2c_client, databuf, 2);
	if(ret < 0)
		printk("ami304 i2c send data error");
	cmd = AMI304_REG_DATAXH;
	ret = i2c_master_send(ami304_i2c_client, &cmd, 1);
	if(ret < 0)
		printk("ami304 i2c send data error");
	ret = i2c_master_recv(ami304_i2c_client, buf, 6);
	if(ret < 0)
		printk("ami304 i2c receive data error");
	cmd = AMI304_REG_WIA;
	ret = i2c_master_send(ami304_i2c_client, &cmd, 1);
	if(ret < 0)
		printk("ami304 i2c send data error");
	ret = i2c_master_recv(ami304_i2c_client, &wia, 1);
	if(ret < 0)
		printk("ami304 i2c receive data error");
	if(wia != 0x47)
		printk("wrong who am i\n");
	if (poll_mode == START_HEAVY)
		msleep(5);
	queue_delayed_work(sensor_work_queue, &ami304_i2c_test_work, poll_mode) ;
}
static int ami304_open(struct inode *inode, struct file *file)
{
	int ret = -1;
	if( atomic_cmpxchg(&dev_open_count, 0, 1)==0 ) {
		printk(KERN_INFO "Open device node:ami304\n");
		ret = nonseekable_open(inode, file);
	}	
	return ret;
}

static int ami304_release(struct inode *inode, struct file *file)
{
	atomic_set(&dev_open_count, 0);
	printk(KERN_INFO "Release device node:ami304\n");		
	return 0;
}

static int ami304_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	char strbuf[AMI304_BUFSIZE];
	int controlbuf[AMI304_CB_LENGTH];
	int valuebuf[4];
	int calidata[7];
	int gyrodata[3];
	long pedodata[3];	
	int pedoparam[AMI304_PD_LENGTH];
	void __user *data;
	int retval=0;
	int mode=0,chipset=0;
	int iEnReport;

   //check the authority is root or not
    if(!capable(CAP_SYS_ADMIN)) {
        retval = -EPERM;
        goto err_out;
	}
		
	switch (cmd) {
		case AMI304_IOCTL_INIT:
			read_lock(&ami304_data.lock);
			mode = ami304_data.mode;
			chipset = ami304_data.chipset;
			read_unlock(&ami304_data.lock);
			AMI304_Chipset_Init(mode, chipset);			
			break;
		
		case AMI304_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadChipInfo(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
			break;

		case AMI304_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
			break;				
						
		case AMI304_IOCTL_READ_POSTUREDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadPostureData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
			break;			
	 
	 	case AMI304_IOCTL_WRITE_POSTUREDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&valuebuf, data, sizeof(valuebuf))) {
				retval = -EFAULT;
				goto err_out;
			}				
			write_lock(&ami304mid_data.datalock);
			ami304mid_data.yaw   = valuebuf[0];
			ami304mid_data.pitch = valuebuf[1];
			ami304mid_data.roll  = valuebuf[2];
			ami304mid_data.status = valuebuf[3];
			write_unlock(&ami304mid_data.datalock);		 	
	 		break;
	        case AMI304_IOCTL_READ_CALIDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadCaliData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
		        break;
	    
		case AMI304_IOCTL_WRITE_CALIDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&calidata, data, sizeof(calidata))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.datalock);			
			ami304mid_data.nm.x = calidata[0];
			ami304mid_data.nm.y = calidata[1];
			ami304mid_data.nm.z = calidata[2];
			ami304mid_data.na.x = calidata[3];
			ami304mid_data.na.y = calidata[4];
			ami304mid_data.na.z = calidata[5];
			ami304mid_data.status = calidata[6];
			write_unlock(&ami304mid_data.datalock);
	    		break;    

		case AMI304_IOCTL_READ_GYRODATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadGyroData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
			break;
			
		case AMI304_IOCTL_WRITE_GYRODATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&gyrodata, data, sizeof(gyrodata))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.datalock);			
			ami304mid_data.gyro.x = gyrodata[0];
			ami304mid_data.gyro.y = gyrodata[1];
			ami304mid_data.gyro.z = gyrodata[2];
			write_unlock(&ami304mid_data.datalock);		
			break;
			
		case AMI304_IOCTL_READ_PEDODATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadPedoData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
			break;

		case AMI304_IOCTL_WRITE_PEDODATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&pedodata, data, sizeof(pedodata))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.datalock);			
			ami304mid_data.pedo.pedo_step = pedodata[0];
			ami304mid_data.pedo.pedo_time = pedodata[1];
			ami304mid_data.pedo.pedo_stat = (int)pedodata[2];
			write_unlock(&ami304mid_data.datalock);  		
	        	break;
	        	
		case AMI304_IOCTL_READ_PEDOPARAM:
			read_lock(&ami304mid_data.ctrllock);
			memcpy(pedoparam, &ami304mid_data.pedometerparam[0], sizeof(pedoparam));
			read_unlock(&ami304mid_data.ctrllock);			
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_to_user(data, pedoparam, sizeof(pedoparam))) {
				retval = -EFAULT;
				goto err_out;
			}			
			break;
			
		case AMI304_IOCTL_WRITE_PEDOPARAM:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(pedoparam, data, sizeof(pedoparam))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.ctrllock);
			memcpy(&ami304mid_data.pedometerparam[0], pedoparam, sizeof(pedoparam));
			write_unlock(&ami304mid_data.ctrllock);
	       	break;	
	        
		case AMI304_IOCTL_READ_CONTROL:
			read_lock(&ami304mid_data.ctrllock);
			memcpy(controlbuf, &ami304mid_data.controldata[0], sizeof(controlbuf));
			read_unlock(&ami304mid_data.ctrllock);			
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_to_user(data, controlbuf, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}						        
	        	break;

		case AMI304_IOCTL_WRITE_CONTROL:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(controlbuf, data, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.ctrllock);
			memcpy(&ami304mid_data.controldata[0], controlbuf, sizeof(controlbuf));
			write_unlock(&ami304mid_data.ctrllock);		
			break;
			
		case AMI304_IOCTL_WRITE_MODE:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&mode, data, sizeof(mode))) {
				retval = -EFAULT;
				goto err_out;
			}		
			AMI304_SetMode(mode);				
			break;
			
		case AMI304_IOCTL_WRITE_REPORT:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&iEnReport, data, sizeof(iEnReport))) {
				retval = -EFAULT;
				goto err_out;
			}				
			AMI304_Report_Value(iEnReport);		
			break;
		
		case AMI304_IOCTL_READ_WIA:
			data = (void __user *) arg;
			if (data == NULL)
				break;		
			AMI304_WIA(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}								
			break;
		case AMI304_IOCTL_I2C_STRESS_TEST:
			switch (arg){
				case 0:
					printk("Compass: ami304 i2c test stop\n");
					cancel_delayed_work_sync(&ami304_i2c_test_work);
					break;
				case 1:
					printk("Compass: ami304 i2c test start (normal)\n");
					poll_mode = START_NORMAL;
					queue_delayed_work(sensor_work_queue, &ami304_i2c_test_work, poll_mode);
					break;
				case 2:
					printk("Compass: ami304 i2c test start (heavy)\n");
					poll_mode = START_HEAVY;
					queue_delayed_work(sensor_work_queue, &ami304_i2c_test_work, poll_mode);
					break;
			}
			break;
		default:
			printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			retval = -ENOIOCTLCMD;
			break;
	}
	
err_out:
	return retval;	
}

static int ami304daemon_open(struct inode *inode, struct file *file)
{
	int ret = -1;
	if( atomic_cmpxchg(&daemon_open_count, 0, 1)==0 ) {
		printk(KERN_INFO "Open device node:ami304daemon\n");
		ret = 0;
	}
	return ret;	
}

static int ami304daemon_release(struct inode *inode, struct file *file)
{
	atomic_set(&daemon_open_count, 0);
	printk(KERN_INFO "Release device node:ami304daemon\n");		
	return 0;
}

static int ami304daemon_ioctl(struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	int valuebuf[4];
	int calidata[7];
	int gyrodata[3];
	long pedodata[3];
	int controlbuf[AMI304_CB_LENGTH];
	char strbuf[AMI304_BUFSIZE];
	int pedoparam[AMI304_PD_LENGTH];	
	void __user *data;
	int retval=0;
	int mode;
	int iEnReport;
		
    //check the authority is root or not
    if(!capable(CAP_SYS_ADMIN)) {
        retval = -EPERM;
        goto err_out;
    }
		
	switch (cmd) {
			
		case AMI304DAE_IOCTL_GET_SENSORDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}
			break;
				
		case AMI304DAE_IOCTL_SET_POSTURE:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&valuebuf, data, sizeof(valuebuf))) {
				retval = -EFAULT;
				goto err_out;
			}				
			write_lock(&ami304mid_data.datalock);
			ami304mid_data.yaw   = valuebuf[0];
			ami304mid_data.pitch = valuebuf[1];
			ami304mid_data.roll  = valuebuf[2];
			ami304mid_data.status = valuebuf[3];
			write_unlock(&ami304mid_data.datalock);	
			break;		
			
		case AMI304DAE_IOCTL_SET_CALIDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&calidata, data, sizeof(calidata))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.datalock);			
			ami304mid_data.nm.x = calidata[0];
			ami304mid_data.nm.y = calidata[1];
			ami304mid_data.nm.z = calidata[2];
			ami304mid_data.na.x = calidata[3];
			ami304mid_data.na.y = calidata[4];
			ami304mid_data.na.z = calidata[5];
			ami304mid_data.status = calidata[6];
			write_unlock(&ami304mid_data.datalock);	
			break;								

        case AMI304DAE_IOCTL_SET_GYRODATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&gyrodata, data, sizeof(gyrodata))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.datalock);			
			ami304mid_data.gyro.x = gyrodata[0];
			ami304mid_data.gyro.y = gyrodata[1];
			ami304mid_data.gyro.z = gyrodata[2];
			write_unlock(&ami304mid_data.datalock);
        		break;
        
        case AMI304DAE_IOCTL_SET_PEDODATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&pedodata, data, sizeof(pedodata))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.datalock);			
			ami304mid_data.pedo.pedo_step = pedodata[0];
			ami304mid_data.pedo.pedo_time = pedodata[1];
			ami304mid_data.pedo.pedo_stat = (int)pedodata[2];
			write_unlock(&ami304mid_data.datalock);        
			break;
			
		case AMI304DAE_IOCTL_GET_PEDOPARAM:
			read_lock(&ami304mid_data.ctrllock);
			memcpy(pedoparam, &ami304mid_data.pedometerparam[0], sizeof(pedoparam));
			read_unlock(&ami304mid_data.ctrllock);			
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_to_user(data, pedoparam, sizeof(pedoparam))) {
				retval = -EFAULT;
				goto err_out;
			}					
			break;
			
		case AMI304DAE_IOCTL_SET_PEDOPARAM:
		    data = (void __user *) arg;			
			if (data == NULL)
				break;	
			if (copy_from_user(pedoparam, data, sizeof(pedoparam))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.ctrllock);
			memcpy(&ami304mid_data.pedometerparam[0], pedoparam, sizeof(pedoparam));
			write_unlock(&ami304mid_data.ctrllock);					
			break;	

		case AMI304DAE_IOCTL_GET_CONTROL:
			read_lock(&ami304mid_data.ctrllock);
			memcpy(controlbuf, &ami304mid_data.controldata[0], sizeof(controlbuf));
			read_unlock(&ami304mid_data.ctrllock);			
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_to_user(data, controlbuf, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}					
			break;		
			
		case AMI304DAE_IOCTL_SET_CONTROL:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(controlbuf, data, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.ctrllock);
			memcpy(&ami304mid_data.controldata[0], controlbuf, sizeof(controlbuf));
			write_unlock(&ami304mid_data.ctrllock);
			break;	
	
		case AMI304DAE_IOCTL_SET_MODE:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&mode, data, sizeof(mode))) {
				retval = -EFAULT;
				goto err_out;
			}		
			AMI304_SetMode(mode);				
			break;

		//Add for input_device sync			
		case AMI304DAE_IOCTL_SET_REPORT:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&iEnReport, data, sizeof(iEnReport))) {
				retval = -EFAULT;
				goto err_out;
			}				
			AMI304_Report_Value(iEnReport);
			break;
			
		case AMI304DAE_IOCTL_GET_WIA:
			data = (void __user *) arg;
			if (data == NULL)
				break;		
			AMI304_WIA(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}
			break;
						
		default:
			printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			retval = -ENOIOCTLCMD;
			break;
	}
	
err_out:
	return retval;	
}

static int ami304hal_open(struct inode *inode, struct file *file)
{
	atomic_inc_and_test(&hal_open_count);
	printk(KERN_INFO "Open device node:ami304hal %d times.\n", atomic_read(&hal_open_count));	
	return 0;
}

static int ami304hal_release(struct inode *inode, struct file *file)
{
	atomic_dec_and_test(&hal_open_count);
	printk(KERN_INFO "Release ami304hal, remainder is %d times.\n", atomic_read(&hal_open_count));	
	return 0;
}

static int ami304hal_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
    int controlbuf[AMI304_CB_LENGTH];
	char strbuf[AMI304_BUFSIZE];
	int pedoparam[AMI304_PD_LENGTH];		
	void __user *data;
	int retval=0;
		
	switch (cmd) {
		
		case AMI304HAL_IOCTL_GET_SENSORDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}			
			break;
									
		case AMI304HAL_IOCTL_GET_POSTURE:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadPostureData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
			break;			
	 
		case AMI304HAL_IOCTL_GET_CALIDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadCaliData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
		       break;

		case AMI304HAL_IOCTL_GET_GYRODATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadGyroData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
			break;
		case AMI304HAL_IOCTL_GET_PEDODATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadPedoData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
	       	break;
	        
		case AMI304HAL_IOCTL_GET_PEDOPARAM:
            		read_lock(&ami304mid_data.ctrllock);
			memcpy(pedoparam, &ami304mid_data.pedometerparam[0], sizeof(pedoparam));
			read_unlock(&ami304mid_data.ctrllock);			
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_to_user(data, pedoparam, sizeof(pedoparam))) {
				retval = -EFAULT;
				goto err_out;
			}			
			break;
			
		case AMI304HAL_IOCTL_SET_PEDOPARAM:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(pedoparam, data, sizeof(pedoparam))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.ctrllock);
			memcpy(&ami304mid_data.pedometerparam[0], pedoparam, sizeof(pedoparam));
			write_unlock(&ami304mid_data.ctrllock);
	       	break;	
	        
		case AMI304HAL_IOCTL_GET_CONTROL:
		       read_lock(&ami304mid_data.ctrllock);
			memcpy(controlbuf, &ami304mid_data.controldata[0], sizeof(controlbuf));
			read_unlock(&ami304mid_data.ctrllock);			
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_to_user(data, controlbuf, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}			
			break;

  	      case AMI304HAL_IOCTL_SET_CONTROL:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(controlbuf, data, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.ctrllock);
			memcpy(&ami304mid_data.controldata[0], controlbuf, sizeof(controlbuf));
			write_unlock(&ami304mid_data.ctrllock);
		       break;

		case AMI304HAL_IOCTL_GET_WIA:
			data = (void __user *) arg;
			if (data == NULL)
				break;		
			AMI304_WIA(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}
	       	break;

		default:
			printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			retval = -ENOIOCTLCMD;
			break;
	}
	
err_out:
	return retval;	
}

static struct file_operations ami304_fops = {
	.owner = THIS_MODULE,
	.open = ami304_open,
	.release = ami304_release,
	.unlocked_ioctl = ami304_ioctl,
};

static struct miscdevice ami304_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ami304",
	.fops = &ami304_fops,
};


static struct file_operations ami304daemon_fops = {
	.owner = THIS_MODULE,
	.open = ami304daemon_open,
	.release = ami304daemon_release,
	.unlocked_ioctl= ami304daemon_ioctl,
};

static struct miscdevice ami304daemon_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ami304daemon",
	.fops = &ami304daemon_fops,
};

static struct file_operations ami304hal_fops = {
	.owner = THIS_MODULE,
	.open = ami304hal_open,
	.release = ami304hal_release,
	.unlocked_ioctl = ami304hal_ioctl,
};

static struct miscdevice ami304hal_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ami304hal",
	.fops = &ami304hal_fops,
};


/*
 * interrupt service routine
 */
void ami304_read_measure(struct work_struct *work)
{
	AMI304_ReadSensorDataFromChip();
}
 
static irqreturn_t ami304_isr(int irq, void *dev_id)
{
	
	schedule_work(&ami304_readmeasure_work);
	return IRQ_HANDLED;
}

static int ami304_input_init(struct ami304_i2c_data *data)
{
	int err=0;
	
	data->input_dev = input_allocate_device();
	if (!data->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "ami304_i2c_detect: Failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	set_bit(EV_ABS, data->input_dev->evbit);
	/* yaw */
	input_set_abs_params(data->input_dev, ABS_RX, 0, (360*10), 0, 0);
	/* pitch */
	input_set_abs_params(data->input_dev, ABS_RY, -(180*10), (180*10), 0, 0);
	/* roll */
	input_set_abs_params(data->input_dev, ABS_RZ, -(90*10), (90*10), 0, 0);
	/* status of orientation sensor */	
	input_set_abs_params(data->input_dev, ABS_RUDDER, 0, 5, 0, 0);
	
	/* x-axis of raw acceleration and the range is -2g to +2g */
	input_set_abs_params(data->input_dev, ABS_X, -(1000*2), (1000*2), 0, 0);
	/* y-axis of raw acceleration and the range is -2g to +2g */
	input_set_abs_params(data->input_dev, ABS_Y, -(1000*2), (1000*2), 0, 0);
	/* z-axis of raw acceleration and the range is -2g to +2g */
	input_set_abs_params(data->input_dev, ABS_Z, -(1000*2), (1000*2), 0, 0);
	
	/* x-axis of raw magnetic vector and the range is -3g to +3g */
	input_set_abs_params(data->input_dev, ABS_HAT0X, -(4000*3), (4000*3), 0, 0);
	/* y-axis of raw magnetic vector and the range is -3g to +3g */
	input_set_abs_params(data->input_dev, ABS_HAT0Y, -(4000*3), (4000*3), 0, 0);
	/* z-axis of raw magnetic vector and the range is -3g to +3g */
	input_set_abs_params(data->input_dev, ABS_BRAKE, -(4000*3), (4000*3), 0, 0);
	/* status of magnetic sensor */
	input_set_abs_params(data->input_dev, ABS_WHEEL, 0, 5, 0, 0);	

	/* x-axis of gyro sensor */
	input_set_abs_params(data->input_dev, ABS_HAT1X, -10000, 10000, 0, 0);
	/* y-axis of gyro sensor */
	input_set_abs_params(data->input_dev, ABS_HAT1Y, -10000, 10000, 0, 0);
	/* z-axis of gyro sensor */
	input_set_abs_params(data->input_dev, ABS_THROTTLE, -10000, 10000, 0, 0);

	data->input_dev->name = "ami304_compass";
	//register input device
	err = input_register_device(data->input_dev);
	if (err) {
		printk(KERN_ERR
		       "ami304_i2c_detect: Unable to register input device: %s\n",
		       data->input_dev->name);
		goto exit_input_register_device_failed;
	}
		
	return 0;
exit_input_register_device_failed:
	input_free_device(data->input_dev);	
exit_input_dev_alloc_failed:
	return err;	
}

static int __devinit ami304_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
 	struct ami304_i2c_data *data;
       int err = 0;

	printk(KERN_INFO "\n\nEnter ami304_i2c_probe!!\n");
	if (!(data = kmalloc(sizeof(struct ami304_i2c_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct ami304_i2c_data));

	data->client = client;
	i2c_set_clientdata(client, data);
	ami304_i2c_client = data->client;		
	ami304_data.status = 0;
	if( (err=Identify_AMI_Chipset())!=0 )  //get ami304_data.chipset
	{
		printk(KERN_INFO "Failed to identify AMI_Chipset!\n");	
		return err;
	}
	data->irq = client->irq;
	data->gpio = irq_to_gpio(client->irq);
	tegra_gpio_enable(data->gpio);
	err = gpio_request(data->gpio, "ami304");
	if (err < 0) {
		printk(KERN_INFO "ami304 gpio_request() failed\n");
		goto exit_kfree;
	}

	err = gpio_direction_input(data->gpio);
	if (err < 0) {
		printk(KERN_INFO "Failed to configure input direction for"
			" GPIO %d, error %d\n", data->gpio, err);
		gpio_free(data->gpio);
		goto exit_kfree;
	}
	sensor_work_queue = create_singlethread_workqueue("i2c_ami304_wq");
	if(!sensor_work_queue){
		printk(KERN_ERR "failed to create workequeue thread\n");
		goto exit_kfree;
	}
	INIT_DELAYED_WORK(&ami304_i2c_test_work, ami304_i2c_test) ;
	INIT_WORK(&ami304_readmeasure_work, ami304_read_measure);	
	if (request_irq(data->irq, ami304_isr, IRQF_TRIGGER_RISING, "ami304", &client->dev))
	{
		printk(KERN_ERR "Request ami304 irq error!!\n");
		err = -1;
		goto exit_gpio_free;
	}	
	printk(KERN_ERR "Request ami304 irq successfully !!\n");	
	
	AMI304_Chipset_Init(AMI304_FORCE_MODE, ami304_data.chipset); // default is Force State	
	dev_info(&client->dev, "%s operating mode\n", ami304_data.mode? "force" : "normal");
	
	printk(KERN_INFO "Register input device!\n");	
	err = ami304_input_init(data);
	if(err)
		goto exit_irq_free;	
		
	//register misc device:ami304
	err = misc_register(&ami304_device);
	if (err) {
		printk(KERN_ERR "ami304_device register failed\n");
		goto exit_misc_device_register_failed;
	}	
	//register misc device:ami304daemon	
	err = misc_register(&ami304daemon_device);
	if (err) {
		printk(KERN_ERR "ami304daemon_device register failed\n");
		goto exit_misc_device_register_failed;
	}	
	//register misc device:ami304hal
	err = misc_register(&ami304hal_device);
	if (err) {
		printk(KERN_ERR "ami304hal_device register failed\n");
		goto exit_misc_device_register_failed;
	}	
	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &ami304_attribute_group);
	if (err)
		goto exit_sysfs_create_group_failed;
	ami304_data.status = 1;
       printk("%s () success ", __func__);	
	return 0;
exit_sysfs_create_group_failed:	
exit_misc_device_register_failed:
	input_free_device(data->input_dev);	
exit_irq_free:
	free_irq( data->irq, &client->dev );	
exit_gpio_free:
	gpio_free( data->gpio );	
exit_kfree:	
	kfree(data);
exit:
	return err;
}

static int __devexit ami304_i2c_remove(struct i2c_client *client)
{
	struct ami304_i2c_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &ami304_attribute_group);
	input_unregister_device(data->input_dev);
	flush_scheduled_work();
	free_irq( data->irq, &client->dev );	
	gpio_free( data->gpio );		
	kfree(i2c_get_clientdata(client));
	ami304_i2c_client = NULL;	
	misc_deregister(&ami304hal_device);
	misc_deregister(&ami304daemon_device);
	misc_deregister(&ami304_device);	
	return 0;
}

struct i2c_device_id ami304_idtable[] = {
	{ "ami304", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, ami304_idtable);

static struct i2c_driver ami304_i2c_driver = {
	.driver = {
		.name	= AMI304_DRV_NAME,
	},
	.probe			= ami304_i2c_probe,
	.remove			= __devexit_p(ami304_i2c_remove),
	.id_table		= ami304_idtable,	
};

static int __init ami304_init(void)
{
	int ret;
	
	printk(KERN_INFO "AMI304 MI sensor driver: init\n");
	printk(KERN_INFO "ami304: driver version:%s\n",DRIVER_VERSION);
	
	rwlock_init(&ami304mid_data.ctrllock);
	rwlock_init(&ami304mid_data.datalock);
	rwlock_init(&ami304_data.lock);
	memset(&ami304mid_data.controldata[0], 0, sizeof(int)*AMI304_CB_LENGTH);	
	ami304mid_data.controldata[AMI304_CB_LOOPDELAY] =    20;  // Loop Delay
	ami304mid_data.controldata[AMI304_CB_RUN] =     	  1;  // Run	
	ami304mid_data.controldata[AMI304_CB_ACCCALI] =       0;  // Start-AccCali
	ami304mid_data.controldata[AMI304_CB_MAGCALI] =    	1;   // Start-MagCali
	ami304mid_data.controldata[AMI304_CB_ACTIVESENSORS] = 0;  // Active Sensors
	ami304mid_data.controldata[AMI304_CB_PD_RESET] = 	  0;  // Pedometer not reset    
	ami304mid_data.controldata[AMI304_CB_PD_EN_PARAM] =   0;  // Disable parameters of Pedometer
	memset(&ami304mid_data.pedometerparam[0], 0, sizeof(int)*AMI304_PD_LENGTH);	
	atomic_set(&dev_open_count, 0);	
	atomic_set(&hal_open_count, 0);
	atomic_set(&daemon_open_count, 0);	

	ret = i2c_add_driver(&ami304_i2c_driver);
	if ( ret != 0 ) {
		printk(KERN_INFO "can not add i2c driver\n");
        return ret;
	}
	printk("%s (): success", __func__);
	return ret;	
}

static void __exit ami304_exit(void)
{	
	atomic_set(&dev_open_count, 0);
	atomic_set(&hal_open_count, 0);
	atomic_set(&daemon_open_count, 0);		
	i2c_del_driver(&ami304_i2c_driver);	
}

MODULE_AUTHOR("Kyle K.Y. Chen");
MODULE_DESCRIPTION("AMI304 MI-Sensor driver with DRDY");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(ami304_init);
module_exit(ami304_exit);
