#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>

#include <asm/gpio.h>

#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/delay.h>

//----
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/gpio.h>
//----

#include "dsp.h"


#undef DUMP_REG

#define DSP_IOC_MAGIC	0xf3
#define DSP_IOC_MAXNR	1
#define DSP_CONTROL	_IOW(DSP_IOC_MAGIC, 1,int)

#define START_RECORDING 1
#define END_RECORDING 0
#define PLAYBACK 2

#define DEVICE_NAME		"dsp_fm34"

struct i2c_client *fm34_client;

static int fm34_probe(struct i2c_client *client,
			   const struct i2c_device_id *id);
static int fm34_remove(struct i2c_client *client);
static int fm34_suspend(struct i2c_client *client, pm_message_t mesg);
static int fm34_resume(struct i2c_client *client);

static const struct i2c_device_id fm34_id[] = {
	{DEVICE_NAME, 0},
	{}
};

struct i2c_client *fm34_client;
struct fm34_chip *dsp_chip;
bool bConfigured=false;


MODULE_DEVICE_TABLE(i2c, fm34_id);

static struct i2c_driver fm34_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= DEVICE_NAME,
	},
	.probe		= fm34_probe,
	.remove		= fm34_remove,
	.resume         = fm34_resume,
	.suspend        = fm34_suspend,
	.id_table	= fm34_id,
};

void fm34_reset_DSP(void)
{
	gpio_set_value(TEGRA_GPIO_PH2, 0);
	msleep(10);
	FM34_INFO("GPIO = %d , state = %d\n", TEGRA_GPIO_PH2, gpio_get_value(TEGRA_GPIO_PH2));

	gpio_set_value(TEGRA_GPIO_PH2, 1);
	FM34_INFO("GPIO = %d , state = %d\n", TEGRA_GPIO_PH2, gpio_get_value(TEGRA_GPIO_PH2));

	return;
}

int fm34_config_DSP(void)
{
	int ret=0;
	struct i2c_msg msg[3];
	u8 buf1;

	if(!bConfigured){
		//access chip to check if acknowledgement.
		buf1=0xC0;
		/* Write register */
		msg[0].addr = dsp_chip->client->addr;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &buf1;

		ret = i2c_transfer(dsp_chip->client->adapter, msg, 1);
		if(ret < 0){
			FM34_INFO("DSP NOack, Failed to read 0x%x: %d\n", buf1, ret);
			return ret;
		}
		else
			FM34_INFO("DSP ACK,  read 0x%x: %d\n", buf1, ret);

		gpio_set_value(TEGRA_GPIO_PH3, 1); // Enable DSP
		fm34_reset_DSP();
		msleep(100);

		ret = i2c_master_send(dsp_chip->client, input_parameter, sizeof(input_parameter));
		FM34_INFO("sizeof(input_parameter) = %d\n", sizeof(input_parameter));

		if(ret == sizeof(input_parameter))
			bConfigured=true;

		msleep(100);
	}

	return ret;
}
EXPORT_SYMBOL(fm34_config_DSP);

static ssize_t fm34_show(struct device *class, struct device_attribute *attr, char *buf)
{
	struct fm34_chip *data = i2c_get_clientdata(to_i2c_client(class));

	return sprintf(buf, "%d\n", data->status);
}

static int fm34_chip_init(struct i2c_client *client)
{
	int rc = 0;

	//config RST# pin, default HIGH.
	tegra_gpio_enable(TEGRA_GPIO_PH2);
	rc = gpio_request(TEGRA_GPIO_PH2, "fm34_reset");
	if (rc) {
		FM34_ERR("gpio_request failed for input %d\n", TEGRA_GPIO_PH2);
	}

	rc = gpio_direction_output(TEGRA_GPIO_PH2, 1) ;
	if (rc) {
		FM34_ERR("gpio_direction_output failed for input %d\n", TEGRA_GPIO_PH2);
	}
	FM34_INFO("GPIO = %d , state = %d\n", TEGRA_GPIO_PH2, gpio_get_value(TEGRA_GPIO_PH2));

	gpio_set_value(TEGRA_GPIO_PH2, 1);

	//config PWDN# pin, default HIGH.
	tegra_gpio_enable(TEGRA_GPIO_PH3);
	rc = gpio_request(TEGRA_GPIO_PH3, "fm34_pwdn");
	if (rc) {
		FM34_ERR("gpio_request failed for input %d\n", TEGRA_GPIO_PH3);
	}

	rc = gpio_direction_output(TEGRA_GPIO_PH3, 1) ;
	if (rc) {
		FM34_ERR("gpio_direction_output failed for input %d\n", TEGRA_GPIO_PH3);
	}
	FM34_INFO("GPIO = %d , state = %d\n", TEGRA_GPIO_PH3, gpio_get_value(TEGRA_GPIO_PH3));

	gpio_set_value(TEGRA_GPIO_PH3, 1);

	return 0;
}

int fm34_open(struct inode *inode, struct file *filp)
{
	return 0;          /* success */
}


int fm34_release(struct inode *inode, struct file *filp)
{
	return 0;          /* success */
}

long fm34_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int retval = 0;
	static int recording_enabled = -1;

	if (_IOC_TYPE(cmd) != DSP_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > DSP_IOC_MAXNR) return -ENOTTY;

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 * access_ok: 1 (successful, accessable)
	 */

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

       /* cmd: the ioctl commend user-space asked */
	switch(cmd){
		case DSP_CONTROL:
			fm34_config_DSP();

			switch(arg){
				case START_RECORDING:
					FM34_INFO("Audio input event enabled, enable DSP\n");
					gpio_set_value(TEGRA_GPIO_PH3, 1); // Enable DSP
					recording_enabled = START_RECORDING;
					break;

				case END_RECORDING:
					FM34_INFO("Audio input event disabled, bypass DSP\n");
					gpio_set_value(TEGRA_GPIO_PH3, 0); // Bypass DSP
					recording_enabled = END_RECORDING;
					break;

				case PLAYBACK:
				default:
					if(recording_enabled != START_RECORDING){
						FM34_INFO("Audio output event enabled, bypass DSP\n");
						gpio_set_value(TEGRA_GPIO_PH3, 0); // Bypass DSP
					}
				break;
			}
		break;

	  default:  /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}
	return retval;
}


struct file_operations fm34_fops = {
	.owner =    THIS_MODULE,
	.unlocked_ioctl =	fm34_ioctl,
	.open =		fm34_open,
	.release =	fm34_release,
};


static SENSOR_DEVICE_ATTR(dsp_status, S_IRUGO, fm34_show, NULL, 1);

static struct attribute *fm34_attr[] = {
	&sensor_dev_attr_dsp_status.dev_attr.attr,
	NULL
};


static int fm34_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct fm34_chip *data;
	int err;

	dev_dbg(&client->dev, "%s()\n", __func__);

	data = kzalloc(sizeof (struct fm34_chip), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	dsp_chip=data;
	data->status = 0;

	i2c_set_clientdata(client, data);
	data->client = client;
	fm34_client= data->client;

	data->misc_dev.minor  = MISC_DYNAMIC_MINOR;
	data->misc_dev.name = DEVICE_NAME;
	data->misc_dev.fops = &fm34_fops;
	err = misc_register(&data->misc_dev);
		if (err) {
			pr_err("tegra_acc_probe: Unable to register %s misc device\n", data->misc_dev.name);
		goto exit_free;
			}

	/* Register sysfs hooks */
	data->attrs.attrs = fm34_attr;
	err = sysfs_create_group(&client->dev.kobj, &data->attrs);
	if (err) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		goto exit_free;
	}

	fm34_chip_init(dsp_chip->client);
	data->status = 1;

	bConfigured=false;
	fm34_config_DSP();

	pr_info("%s()\n", __func__);

	return 0;


exit_free:
	kfree(data);
exit:
	return err;
}

static int fm34_remove(struct i2c_client *client)
{
	struct fm34_chip *data = i2c_get_clientdata(client);

	misc_deregister(&data->misc_dev);
	dev_dbg(&client->dev, "%s()\n", __func__);
	pr_info("%s()\n", __func__);
	sysfs_remove_group(&client->dev.kobj, &data->attrs);

	kfree(data);
	return 0;
}

static int fm34_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
static int fm34_resume(struct i2c_client *client)
{
	return 0;
}

static int __init fm34_init(void)
{
	pr_info("%s()\n", __func__);
	return i2c_add_driver(&fm34_driver);
}

static void __exit fm34_exit(void)
{
	pr_info("%s()\n", __func__);
	i2c_del_driver(&fm34_driver);
}

module_init(fm34_init);
module_exit(fm34_exit);
