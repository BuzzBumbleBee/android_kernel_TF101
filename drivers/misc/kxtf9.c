/*
 * kxtf9.c	simple support for the Kionix KXTF9 3D
 *		accelerometer.
 *
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <asm/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include<mach/board-ventana-misc.h>

#undef DUMP_REG

/* kxtf9 register address */
#define kxtf9_XOUT_HPF_LSB           0x00
#define kxtf9_XOUT_HPF_MSB           0x01
#define kxtf9_YOUT_HPF_LSB           0x02
#define kxtf9_YOUT_HPF_MSB           0x03
#define kxtf9_ZOUT_HPF_LSB           0x04
#define kxtf9_ZOUT_HPF_MSB           0x05

#define kxtf9_X_AXIS_LSB           0x06
#define kxtf9_X_AXIS_MSB           0x07
#define kxtf9_Y_AXIS_LSB           0x08
#define kxtf9_Y_AXIS_MSB           0x09
#define kxtf9_Z_AXIS_LSB           0x0A
#define kxtf9_Z_AXIS_MSB           0x0B

#define kxtf9_DCSTRESP              0x0C
// WHO_AM_I reg
#define kxtf9_CHIP_ID              0x0F

#define kxtf9_TILT_CUR             0x10
#define kxtf9_TILT_PRE             0x11

#define kxtf9_INT_SRC_REG1            0x15
#define kxtf9_INT_SRC_REG2            0x16

#define kxtf9_STATUS         0x18
#define kxtf9_INT_REL            0x1A
#define kxtf9_CTRL_REG1            0x1B
#define kxtf9_CTRL_REG2            0x1C
#define kxtf9_CTRL_REG3            0x1D

#define kxtf9_INT_CTRL_REG1            0x1E
#define kxtf9_INT_CTRL_REG2            0x1F
#define kxtf9_INT_CTRL_REG3            0x20

#define kxtf9_DATA_CTRL             0x21

#define kxtf9_TILT_TIMER             0x28
#define kxtf9_WUF_TIMER             0x29
#define kxtf9_TDT_TIMER             0x2B
#define kxtf9_TDT_H_THRESH         0x2C
#define kxtf9_TDT_L_THRESH         0x2D

#define kxtf9_TDT_TAP_TIMER            0x2E
#define kxtf9_TDT_TOTAL_TIMER        0x2F
#define kxtf9_TDT_LATENCY_TIMER    0x30
#define kxtf9_TDT_WINDOW_TIMER    0x31
#define kxtf9_SELF_TEST                   0x3A

#define kxtf9_WUF_THRESH         0x5A
#define kxtf9_TILT_ANGLE         0x5C
#define kxtf9_HYST_SET             0x5F
#define kxtf9_MAX_REGS	kxtf9_HYST_SET
/* kxtf9 register address info ends here*/

#define kxtf9_CHIP_ID_VAL      0x01        // POR value- device identification

//kxtf9 power mode
#define kxtf9_AccelPower_Fullrun 1
#define kxtf9_AccelPower_Standby 0

// set following macros based on requirement
#define TILT_ONLY_ENABLE 1
#define TAP_ONLY_ENABLE 1
#define MOTION_ONLY_ENABLE 1

#define SET_INT_RAISING_EDGE 0

#define KXTF9_IOC_MAGIC	0xf2
#define KXTF9_IOC_MAXNR	3
#define KXTF9_GET_DATA      _IOR(KXTF9_IOC_MAGIC,	2,	int)
#define KXTF9_I2C_TEST      _IOR(KXTF9_IOC_MAGIC,	3,	int)

#define START_NORMAL (HZ/5)
#define START_HEAVY  (HZ/200)

static struct input_dev *g_input_dev;
static struct delayed_work work ;
static struct delayed_work kxtf9_i2c_test_work ;
static struct workqueue_struct *sensor_work_queue = NULL;
static struct i2c_client *this_client = NULL;
static int poll_mode = 0;
/**
 * struct kxtf9_state - device related storage

 **/
struct kxtf9_data {
       struct input_dev	*input_dev;
	struct attribute_group  attrs;
	struct mutex            lock;
	unsigned long           mode;
	u8                      reg_cache[kxtf9_MAX_REGS];
	int                     eoc_gpio;
	int                     eoc_irq;
	int				status;
};


static int kxtf9_probe(struct i2c_client *client,
			   const struct i2c_device_id *id);
static int kxtf9_remove(struct i2c_client *client);
static int kxtf9_suspend(struct i2c_client *client, pm_message_t mesg);
static int kxtf9_resume(struct i2c_client *client);
static int kxtf9_Init(struct i2c_client *client);

static const struct i2c_device_id kxtf9_id[] = {
	{"kxtf9", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, kxtf9_id);

static struct i2c_driver kxtf9_driver = {
	.driver = {
		.name = "kxtf9",
		.owner = THIS_MODULE,
	},
	.probe = kxtf9_probe,
	.remove = kxtf9_remove,
	.resume         = kxtf9_resume,
	.suspend        = kxtf9_suspend,
	.id_table	= kxtf9_id,
};

static bool kxtf9_write_data(struct i2c_client *client,
		u8 reg, u8 val)
{
	u8 regval = 0;
	struct i2c_msg msg;
	u8 w_data[2];
	int ret = 0;

	struct kxtf9_data *data = i2c_get_clientdata(client);

	w_data[0] = reg;
	w_data[1] = val;

	dev_vdbg(&client->dev,"%s(): Writing Reg 0x%x to value 0x%x\n",
			__func__,w_data[0], w_data[1]);
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = w_data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Write to device fails status %x\n", ret);
		return false;
	}
	data->reg_cache[reg] = regval;
	return true;
}

static bool kxtf9_read_data(struct i2c_client *client,
		u8 reg, u8 length, u8 * buffer)
{
	struct i2c_msg msg[2];
	u8 w_data[2];
	int ret = 0;

	w_data[0] = reg;

	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_NOSTART;	/* set repeated start and write */
	msg[0].len = 1;
	msg[0].buf = w_data;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = length;
	msg[1].buf = buffer;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Read from device fails.\n");
		return false;
	}
	return true;
}	

static int KXTF9_ReadSensorData(int *buf)
{
	char cmd;
       u8 databuf[6];
 	int ax, ay, az;
	bool res = false;
	memset(databuf, 0, sizeof(u8)*6);
		
	if (!buf)
		return -1;
	if (!this_client)
	{
		*buf = 0;
		return -2;
	}
	
	cmd = kxtf9_X_AXIS_LSB;

	res =kxtf9_read_data(this_client, cmd, 6, databuf);	
	if (!res)
		goto exit_KXTF9_ReadSensorData;
	
	ax = (databuf[1] << 4) | ( databuf[0] >> 4);
	ay = (databuf[3] << 4) | ( databuf[2] >> 4);
	az = (databuf[5] << 4) | ( databuf[4] >> 4);

       if (ax & 0x800)
           ax |= 0xFFFFF000;
       if (ay & 0x800)
           ay |= 0xFFFFF000;
       if (az & 0x800)
           az |= 0xFFFFF000;
					
exit_KXTF9_ReadSensorData:	
	if (!res ) {
		printk("I2C error ");
		return -3;
	}
	else {
		buf[0] = ax;
		buf[1] = ay;
		buf[2] = az;
	}
	return 0;
}

int kxtf9_open(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__) ;
	return 0;          /* success */
}

int kxtf9_release(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__) ;
	return 0;          /* success */
}

static long kxtf9_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 1;
	int databuf[3];

	if (_IOC_TYPE(cmd) != KXTF9_IOC_MAGIC){
			printk("kxtf9  check ioc magic fail\n");
       	 return -ENOTTY;
	}
	if (_IOC_NR(cmd) > KXTF9_IOC_MAXNR){
		printk("kxtf9  check ioc maxnr fail\n");
		return -ENOTTY;
	}

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;
	
	 switch (cmd) {
        case KXTF9_GET_DATA:
			KXTF9_ReadSensorData(databuf);
			if (ASUSGetProjectID() == 101){
				databuf[0] = (1)*databuf[0];
				databuf[1] = (1)*databuf[1];
				databuf[2] = (1)*databuf[2];
			}else{
				databuf[0] = (-1)*databuf[0];
				databuf[1] = (1)*databuf[1];
				databuf[2] = (-1)*databuf[2];
			}
			if(copy_to_user( (void __user*)arg, databuf, sizeof(databuf)))
				return -EFAULT;
			break;		
	case KXTF9_I2C_TEST:
			switch(arg){
				case 0:
					printk("Accelerometer: kxtf9 i2c test stop\n");
					cancel_delayed_work_sync(&kxtf9_i2c_test_work);
					break;
				case 1:
					printk("Accelerometer: kxtf9 i2c test start (normal)\n");
					poll_mode = START_NORMAL;
					queue_delayed_work(sensor_work_queue, &kxtf9_i2c_test_work, poll_mode);
					break;
				case 2:
					printk("Accelerometer: kxtf9 i2c test start (heavy)\n");
					poll_mode = START_HEAVY;
					queue_delayed_work(sensor_work_queue, &kxtf9_i2c_test_work, poll_mode);
					break;
				default:
					printk("Accelerometer: kxtf9 i2c test start (wrong arg)\n");
					break;
				}
			break;
        default: /* redundant, as cmd was checked against MAXNR */
            return -ENOTTY;
	}
    return 0;
}

static struct file_operations kxtf9_fops = {
	.owner = THIS_MODULE,
	.open = kxtf9_open,
	.release = kxtf9_release,
	.unlocked_ioctl = kxtf9_ioctl,
};

static struct miscdevice kxtf9_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kxtf9",
	.fops = &kxtf9_fops,
};
static bool SetAccelerometerActive(struct i2c_client* client, bool State)
{
    u8 RegVal;

    // to clear PC1 of CTRL_REG1
    if (!kxtf9_read_data(client, kxtf9_CTRL_REG1,  1, &RegVal))
    {
        printk("\n %s-> %d failed\n", __func__,__LINE__);
        return false;
    }

    if (State)
        RegVal |= 0x80;
    else
        RegVal &= 0x7F;

    if(!kxtf9_write_data(client, kxtf9_CTRL_REG1, RegVal))
    {
        printk("\n %s-> %d failed\n", __func__,__LINE__);
        return false;
    }
    return true;
}

static int kxtf9_Init(struct i2c_client *client)
{
    u8 TestVal;
    u8 RegVal = 0;
    u8 length = 1;

    // wait for 110 msec.
    msleep(110);
    printk("\n kxtf9 I2C addr: 0x%x",client->addr);

    kxtf9_read_data(client, kxtf9_CHIP_ID, length, &TestVal);
    if (TestVal != kxtf9_CHIP_ID_VAL)
    {
        printk("\n Unknown kxtf9 ID = 0x%x\n", TestVal);
        goto error;
    }
    else
    {
        printk("\n kxtf9 ID is 0x%x\n", TestVal);
    }

#if SET_INT_RAISING_EDGE
    // set interrupt bits - IEN, IEA & IEL only active
    RegVal = 0x30;
#else
    RegVal = 0x20;
#endif
    if (!kxtf9_write_data(client, kxtf9_INT_CTRL_REG1, RegVal))
    {
        printk("\n Int Set failed\n");
    }

    // set ODR to 25 Hz
    RegVal = 0x0;
    if (!kxtf9_write_data(client, kxtf9_CTRL_REG3, RegVal))
    {
        printk("\n LPF freq set failed\n");
    }
    // Set WUF_TIMER to 2 ODR clock
    RegVal = 2;
    if (!kxtf9_write_data(client, kxtf9_WUF_TIMER, RegVal))
    {
        printk("\n LPF freq set failed\n");
    }

    RegVal = 0;
#if TILT_ONLY_ENABLE
    //Activate Tilt Position Function
    kxtf9_write_data(client, kxtf9_TILT_TIMER, 0x01);
    RegVal = 0x81;
#endif

#if TAP_ONLY_ENABLE
    // Activate Tap/Double Tap Function
    RegVal |= 0x84;
#endif

#if MOTION_ONLY_ENABLE
    // For getting continuous data from accelerometer, set DRDY = 1.
    //set PC1, RES & WUFE to 1.
    RegVal |= 0XC2;
#endif
    if(!kxtf9_write_data(client, kxtf9_CTRL_REG1, RegVal))
        goto error;

    printk("kxtf9_Init passed\n");
    return 0;
error:
    printk("kxtf9_Init failed\n");
    return -1;
}

static ssize_t show_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *data = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", data->status);
}

static SENSOR_DEVICE_ATTR(accelerometer_status, S_IRUGO, show_status, NULL, 1);
static SENSOR_DEVICE_ATTR(show, S_IRUGO, NULL, NULL, 1);

static struct attribute *kxtf9_attr[] = {
	&sensor_dev_attr_show.dev_attr.attr,
	&sensor_dev_attr_accelerometer_status.dev_attr.attr,
	NULL
};


static void kxtf9_ResetInterrupt(void)
{
    u8 Data[2];
	
	if (!kxtf9_read_data(this_client, kxtf9_INT_SRC_REG1, 2, Data))
		printk("%s(): read reg fail\n", __func__);
    // To clear the interrupts.
	if(!kxtf9_read_data(this_client, kxtf9_INT_REL, 1,  Data))
		printk("%s(): read reg fail\n", __func__);
}

static void kxtf9_i2c_test(struct work_struct *work)
{
	int databuf[3];

	KXTF9_ReadSensorData(databuf);
	if( poll_mode == START_HEAVY)
		msleep(5);
	queue_delayed_work(sensor_work_queue, &kxtf9_i2c_test_work, poll_mode);
}

static void kxtf9_push_data(struct work_struct *work)
{
	int databuf[3];
	struct kxtf9_data *data = i2c_get_clientdata(this_client);
	
	kxtf9_ResetInterrupt();
	KXTF9_ReadSensorData(databuf);
	if (ASUSGetProjectID() == 101){
		databuf[0] = (1)*databuf[0];
		databuf[1] = (1)*databuf[1];
		databuf[2] = (1)*databuf[2];
	}else{
		databuf[0] = (-1)*databuf[0];
		databuf[1] = (1)*databuf[1];
		databuf[2] = (-1)*databuf[2];
	}
	input_report_abs(data->input_dev, ABS_X, databuf[0]);
	input_report_abs(data->input_dev, ABS_Y, databuf[1]);
	input_report_abs(data->input_dev, ABS_Z, databuf[2]);
	input_sync(data->input_dev);
}

static irqreturn_t kxtf9_interrupt_handler(int irq, void *dev_id)
{
	queue_delayed_work(sensor_work_queue, &work, 0);
	return IRQ_HANDLED;
}

void kxtf9_dump_reg(struct i2c_client *client)
{
	u8 value, offset;

	for(offset=kxtf9_XOUT_HPF_LSB; offset<=kxtf9_MAX_REGS; offset++){
		kxtf9_read_data(client, offset, sizeof(value), &value);
		printk("kxtf9:%s:[0x%02X]=0x%02X\n", __FUNCTION__, offset, value);
	}
	return;
}

static int __devinit kxtf9_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	struct kxtf9_data *data;
	int err;

	data = kzalloc(sizeof (struct kxtf9_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}
	this_client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);
	data->status = 0;
	sensor_work_queue = create_singlethread_workqueue("i2c_kxtf9_wq");
	INIT_DELAYED_WORK(&work, kxtf9_push_data);
	INIT_DELAYED_WORK(&kxtf9_i2c_test_work, kxtf9_i2c_test);

	data->eoc_irq = client->irq;
	data->eoc_gpio = irq_to_gpio(client->irq);
	tegra_gpio_enable(data->eoc_gpio);
	err = gpio_request(data->eoc_gpio, "kxtf9");
	if (err < 0) {
		dev_err(&client->dev, "failed to request GPIO %d, error %d\n",
			data->eoc_gpio, err);
		goto exit_free;
	}
	err = gpio_direction_input(data->eoc_gpio);
	if (err < 0) {
		dev_err(&client->dev, "Failed to configure input direction for"
			" GPIO %d, error %d\n", data->eoc_gpio, err);
		gpio_free(data->eoc_gpio);
		goto exit_gpio;
	}
	err = request_irq(data->eoc_irq, kxtf9_interrupt_handler,
		IRQF_TRIGGER_FALLING | IRQF_DISABLED , "kxtf9", data);

	err = kxtf9_Init(client);
	if (err < 0) {
		dev_err(&client->dev, "kxtf9 initialization fails\n");
		goto exit_gpio;
	}

	dev_info(&client->dev, "%s chip found Gpio %d\n", client->name,
		 data->eoc_gpio);

#ifdef DUMP_REG
	kxtf9_dump_reg(client);
#endif
	data->input_dev = input_allocate_device();
	if (data->input_dev == NULL) {
		err = -ENOMEM;
		pr_err("kxtf9: Failed to allocate input device\n");
		goto exit_gpio;
	}
	g_input_dev = data->input_dev;	
	set_bit(EV_SYN, data->input_dev->evbit);
	set_bit(EV_KEY, data->input_dev->evbit);
	set_bit(EV_ABS, data->input_dev->evbit);

	input_set_abs_params(data->input_dev, ABS_X,
		-2048, 2048, 0, 0);
	input_set_abs_params(data->input_dev, ABS_Y,
		-2048, 2048, 0, 0);
	input_set_abs_params(data->input_dev, ABS_Z,
		-2048, 2048, 0, 0);
	data->input_dev->name = "kionix_kxtf9";
	err = input_register_device(data->input_dev);
	if (err) {
		pr_err("kxtf9 : Unable to register %s\
				input device\n", data->input_dev->name);
		goto exit_gpio;
	}
	/* Register sysfs hooks */
	data->attrs.attrs = kxtf9_attr;
	err = sysfs_create_group(&client->dev.kobj, &data->attrs);
	if (err) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		goto exit_input;
	}
	err = misc_register(&kxtf9_device);
	if (err) {
		dev_err(&client->dev, "Not able to create the misc device\n");
		goto exit_remove;
	}

	data->status = 1;
	printk("kxtf9 probe success\n");
	return 0;
exit_remove:
	sysfs_remove_group(&client->dev.kobj, &data->attrs);
exit_input:
	input_unregister_device(data->input_dev);
exit_gpio:
	gpio_free(data->eoc_gpio);
exit_free:
	kfree(data);
exit:
	return err;
}


static int kxtf9_remove(struct i2c_client *client)
{
	struct kxtf9_data *data = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s()\n", __func__);
	sysfs_remove_group(&client->dev.kobj, &data->attrs);
	input_unregister_device(data->input_dev);
	misc_deregister(&kxtf9_device);
	gpio_free(data->eoc_gpio);
	kfree(data);
	return 0;
}


static int kxtf9_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct kxtf9_data *data = i2c_get_clientdata(client);
	bool status = true;

	dev_dbg(&client->dev, "%s()\n", __func__);

	mutex_lock(&data->lock);
	status = SetAccelerometerActive(client, kxtf9_AccelPower_Standby);
	if (!status) {
		dev_err(&client->dev, "Error in setting stand by mode\n");
		mutex_unlock(&data->lock);
		return -EBUSY;
	}
	mutex_unlock(&data->lock);

	return 0;
}
static int kxtf9_resume(struct i2c_client *client)
{
	struct kxtf9_data *data = i2c_get_clientdata(client);
	bool status = true;

	dev_dbg(&client->dev, "%s()\n", __func__);

	mutex_lock(&data->lock);
	status = SetAccelerometerActive(client, kxtf9_AccelPower_Fullrun);
	if (!status) {
		dev_err(&client->dev, "Error in setting full run mode\n");
		mutex_unlock(&data->lock);
		return -EBUSY;
	}
	mutex_unlock(&data->lock);
	
	return 0;
}


static __init int kxtf9_init(void)
{
	pr_info("%s()\n", __func__);
	return i2c_add_driver(&kxtf9_driver);
}
module_init(kxtf9_init);

static __exit void kxtf9_exit(void)
{
	pr_info("%s()\n", __func__);
	i2c_del_driver(&kxtf9_driver);
}
module_exit(kxtf9_exit);


