/*
 * A sensor driver for the  capacitive touch LDS6126.
 *
 * PureTouch.* Capacitive Touch Sensor IC Driver LDS6126.
 *
 * Copyright (c) 2010, ASUSTek Corporation.
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
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <asm/gpio.h>

/*#define DEBUG           1*/
#define VERBOSE_DEBUG   1

MODULE_DESCRIPTION("IDT Capacitive Sensor Driver LDS6126");
MODULE_LICENSE("GPL");

/*----------------------------------------------------------------------------
** Debug Utility
**----------------------------------------------------------------------------*/
#define CAP_SENSOR_DEBUG			1

#if CAP_SENSOR_DEBUG
#define CAP_SENSOR_INFO(format, arg...)	\
	printk(KERN_INFO "CAP_SENSOR: [%s] " format , __FUNCTION__ , ## arg)
#else
#define CAP_SENSOR_INFO(format, arg...)	 
#endif

#define CAP_SENSOR_ERR(format, arg...)	\
	printk(KERN_ERR "CAP_SENSOR: [%s] " format , __FUNCTION__ , ## arg)

#undef DUMP_REG
#define TEGRA_GPIO_PK2		82	/* RESET#_CAP */
#define TEGRA_GPIO_PX4		188	/* INT_CAP# */
#define TEGRA_GPIO_PU4		164	/* HOME_LED # */
#define TEGRA_GPIO_PR4		140	/* MENU_LED # */
#define TEGRA_GPIO_PR5		141	/* BACK_LED # */

/*----------------------------------------------------------------------------
** Global Variable
**----------------------------------------------------------------------------*/
struct cap_lds6126_data {
	struct input_dev	*input_dev;	 /* Pointer to input device */
	struct attribute_group attrs;
	int status;
};

struct cap_lds6126_data *cap_data;
struct i2c_client *cap_client;
static dev_t cap_lds6126_dev;
struct cdev *cap_lds6126_cdev;
static struct class *cap_lds6126_class;
static struct device *cap_lds6126_class_device;
static int cap_lds6126_major = 0;
static int cap_lds6126_minor = 0;
int time = 0;
struct work_struct work;
static struct workqueue_struct *cap_wq;

/*----------------------------------------------------------------------------
** FUNCTION DECLARATION
**----------------------------------------------------------------------------*/
static int __devinit cap_lds6126_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit cap_lds6126_remove(struct i2c_client *client);
static int cap_lds6126_create_input_dev(struct i2c_client *client);
static int cap_lds6126_suspend(struct i2c_client *client, pm_message_t mesg);
static int cap_lds6126_resume(struct i2c_client *client);
static u16 cap_lds6126_read_reg(struct i2c_client *client, u16 reg);
static bool cap_lds6126_write_reg(struct i2c_client* client, u16 reg, u16 val);
static int __init cap_lds6126_init(void);
static void __exit cap_lds6126_exit(void);
static irqreturn_t cap_lds6126_interrupt_handler(int irq, void *dev_id);
static void cap_lds6126_work_function(struct work_struct *dat);
static int cap_lds6126_config_irq(struct i2c_client *client);
static int	init_cap_sensor(void);
static int	init_led_sensor(void);
static void cap_lds6126_button_down_set(unsigned long index);
static void cap_lds6126_button_up_set(int index);
static int cap_lds6126_button_handler(int button);

/*----------------------------------------------------------------------------
** I2C Driver Structure
**----------------------------------------------------------------------------*/
static const struct i2c_device_id cap_lds6126_id[] = {
	{"cap_lds6126", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cap_lds6126_id);

static struct i2c_driver cap_lds6126_driver = {
	.driver = {
		.name	= "cap_lds6126",
		.owner	= THIS_MODULE,	
	},
	.probe		= cap_lds6126_probe,
	.remove		= __devexit_p(cap_lds6126_remove),
	.resume         = cap_lds6126_resume,
	.suspend        = cap_lds6126_suspend,
	.id_table	= cap_lds6126_id,
};

static ssize_t read_sysfs_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	cap_data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", cap_data->status);
}

DEVICE_ATTR(cap_status, 0777, read_sysfs_status, NULL);

static struct attribute *cap_lds6126_attr[] = {
	&dev_attr_cap_status.attr,
	NULL
};

/**********************************************************
**  Function: Capacitive touch driver I2C read operation
**  Parameter: I2C client, register read
**  Return value: if sucess, then returns the value of the register
**                      otherwise returns error code
************************************************************/
static u16 cap_lds6126_read_reg(struct i2c_client *client, u16 reg)
{
	struct i2c_msg msg[3];
	char data[2];
	u8 buf1[2];
	u16 result;
	int ret = 0;

	buf1[0] = (u8)reg >> 8;	/* Register Address(MSB) */
	buf1[1] = (u8)reg;	/* Register Address(LSB) */

	/* Write register */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf1;

	/* Read data */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = (u8 *)data;

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret < 0){
		dev_err(&client->dev, "Failed to read 0x%x: %d\n", reg, ret);
		return false;
	}

	result =  (data[1] & 0xff) | (u16)(data[0] << 8);
	
	return result;

}

/**********************************************************
**  Function: Capacitive touch driver I2C write operation
**  Parameter: I2C client, register read and the value written
**  Return value: if sucess, then returns 0
**                      otherwise returns error value
************************************************************/
static bool cap_lds6126_write_reg(struct i2c_client* client, u16 reg, u16 val)
{
	int err;
	struct i2c_msg msg[1];
	char data[4];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 4;
	msg->buf = data;

	data[0] = (u8)(reg >> 8);	/* Register Address(MSB) */
	data[1] = (u8)reg;	/* Register Address(LSB) */
	data[2] = (u8)(val >> 8);	/* Data Value(MSB) */
	data[3] = (u8)val;	/* Data Value(LSB) */

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return false;
	
	return true;

}

/**********************************************************
**  Function: Called to handle virtual button event
**  Parameter: button status
**  Return value: none
**
************************************************************/
static int cap_lds6126_button_handler(int button)
{
	static int iPreButton = -1;
	int iButton = -1;
	
	iButton = button;  							/* Button = 2, 4*/

	if(iButton != iPreButton && iButton!=0)			/* Button Down */
		cap_lds6126_button_down_set(iButton);

	if(button == 0)	/* Button up*/
		cap_lds6126_button_up_set(iPreButton);

	iPreButton = iButton;							/* Record the button */
	return 0;
}

/**********************************************************
**  Function: Called to handle virtual button DOWN event
**  Parameter: which button
**  Return value: none
**
************************************************************/
/* Do something after button pressed*/
static void cap_lds6126_button_down_set(unsigned long index)
{
	switch(index){
		case 2:
			printk("CAP_SENSOR: KEY_BACK Down\n");
		#if VERBOSE_DEBUG	
			CAP_SENSOR_INFO("%s:data->input_dev=0x%lX, key_number=%d, pressed=%d\n", __FUNCTION__, (long unsigned int)cap_data->input_dev, KEY_BACK, 1);
		#endif
			input_report_key(cap_data->input_dev, KEY_BACK, 1);	/* KEY_BACK Down */
			break;
		case 4:
			printk("CAP_SENSOR: KEY_MENU Down\n");
		#if VERBOSE_DEBUG	
			CAP_SENSOR_INFO("%s:data->input_dev=0x%lX, key_number=%d, pressed=%d\n", __FUNCTION__, (long unsigned int)cap_data->input_dev, KEY_MENU, 1);
		#endif
			input_report_key(cap_data->input_dev, KEY_MENU, 1);	/* KEY_MENU Down */
			break;
		default:
			CAP_SENSOR_ERR("Invalid Key Down\n");
	}
}

/**********************************************************
**  Function: Called to handle virtual button UP event
**  Parameter: which button
**  Return value: none
**
************************************************************/
static void cap_lds6126_button_up_set(int index)
{
	switch(index){
		case 2:
			printk("CAP_SENSOR: KEY_BACK Up\n");
		#if VERBOSE_DEBUG	
			CAP_SENSOR_INFO("%s:data->input_dev=0x%lX, key_number=%d, pressed=%d\n", __FUNCTION__, (long unsigned int)cap_data->input_dev, KEY_BACK, 0);
		#endif
			input_report_key(cap_data->input_dev, KEY_BACK, 0);	/* KEY_BACK Up */
			break;
		case 4:
			printk("CAP_SENSOR: KEY_MENU Up\n");
		#if VERBOSE_DEBUG	
			CAP_SENSOR_INFO("%s:data->input_dev=0x%lX, key_number=%d, pressed=%d\n", __FUNCTION__, (long unsigned int)cap_data->input_dev, KEY_MENU, 0);
		#endif
			input_report_key(cap_data->input_dev, KEY_MENU, 0);	/* KEY_MENU Up */
			break;
		default:										/* Other invalid keys combination pressed */
			input_report_key(cap_data->input_dev, KEY_BACK, 0);
			input_report_key(cap_data->input_dev, KEY_MENU, 0);
	}
}

/**********************************************************
**  Function: Called to interrupt handler for reading register data
**  Parameter: work struct
**  Return value: none
**  Channel C0: Menu key: 2
**  Channel C1: Back key: 4
************************************************************/
static void cap_lds6126_work_function(struct work_struct *dat)
{
	struct i2c_client *client ;
	u16 button_status;
#if VERBOSE_DEBUG
	int rc;
#endif
	int button_pressed;
	client = cap_client;

	/* Read the Touch status */
	button_status = cap_lds6126_read_reg(client,0x045);

#if VERBOSE_DEBUG
	CAP_SENSOR_INFO("the value of register 0x045(Touch status) is %x\n",button_status);
	rc = gpio_get_value(TEGRA_GPIO_PX4);
	CAP_SENSOR_INFO("The INT_CAP# state is %d\n",rc);	
#endif

	button_pressed = (button_status & 0x000f);
	cap_lds6126_button_handler(button_pressed);

	return ;

}

/**********************************************************
**  Function: Capacitive touch driver interrupt handler for INT1
**  Parameter: dedicated irq
**  Return value: if sucess, then returns IRQ_HANDLED
**
************************************************************/
static irqreturn_t cap_lds6126_interrupt_handler(int irq, void *dev_id)
{
	queue_work(cap_wq, &work);

	return IRQ_HANDLED;
}

/**********************************************************
**  Function: Capacitive touch driver configure the external gpio pin
**  Parameter: dedicated irq
**  Return value: if sucess, then returns 0
**
************************************************************/
static int cap_lds6126_config_irq(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = irq_to_gpio(client->irq);
	const char* label = "cap_lds6126" ; 

	CAP_SENSOR_INFO("INT configuration, GPIO = %d, IRQ = %d\n", gpio, client->irq);

	tegra_gpio_enable(gpio);
	rc = gpio_request(gpio, "cap_lds6126");

	if(rc){
		CAP_SENSOR_ERR("gpio_request failed for input %d\n", gpio);		
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	if(rc){
		CAP_SENSOR_ERR("gpio_direction_input failed for input %d\n", gpio);			
		goto err_gpio_direction_input_failed;
	}
	CAP_SENSOR_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(client->irq, cap_lds6126_interrupt_handler, IRQF_TRIGGER_FALLING, label, client);
	if(rc < 0){
		CAP_SENSOR_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, client->irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}	
	CAP_SENSOR_INFO("request IRQ = %d, rc = %d\n", client->irq, rc);
	return 0 ;

err_gpio_request_irq_fail :	
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;
}

/**********************************************************
**  Function: Capacitive touch driver device init: read the HelloPacket
**  Parameter: none
**  Return value: if sucess, then returns positive number
**                      otherwise returns negative number
************************************************************/
static int init_cap_sensor()
{
	struct i2c_client *client ;
	int ret=1;
#if VERBOSE_DEBUG	
	u16 temp;
#endif
	client = cap_client;

	CAP_SENSOR_INFO("Cap sensor initialization\n");

#if VERBOSE_DEBUG
	temp = cap_lds6126_read_reg(client, 0x01f);
	CAP_SENSOR_INFO("Manufacturer ID(0x1F) is %x\n",temp);
#endif

	/* COLD RESET */
	cap_lds6126_write_reg(client, 0x0000, 0x0000);
	/* TOUCH DISABLE */
	cap_lds6126_write_reg(client, 0x0040, 0x0030);
	/* DCM CONFIG Disable */
	cap_lds6126_write_reg(client, 0x000A, 0x0000);
	/* SCROLL CONFIG Disable */
	cap_lds6126_write_reg(client, 0x0074, 0x0000);

	/* PSEL, Enable C0 C1 */
	cap_lds6126_write_reg(client, 0x0041, 0x0006);
	cap_lds6126_write_reg(client, 0x0042, 0x0000);

	/* INTERRUPT */
	cap_lds6126_write_reg(client, 0x0043, 0x0006);
	cap_lds6126_write_reg(client, 0x0044, 0x0000);

	/* INTERRUPT PIN DEFINE, Read and Reset mode */
	cap_lds6126_write_reg(client, 0x0008, 0x0002);

	/* LED PSEL, Enable LED0 LED1, and Enable manual GANG mode*/
//	cap_lds6126_write_reg(client, 0x003F, 0x0006);
//	cap_lds6126_write_reg(client, 0x003E, 0x0000);

	/* LED DWC */
//	cap_lds6126_write_reg(client, 0x002F, 0xC030);

	/* LED0 Menu, Dimming, assignm0ent C1 menu due to SR LED placement */
//	cap_lds6126_write_reg(client, 0x0021, 0xF802);
//	cap_lds6126_write_reg(client, 0x0031, 0x0001);

	/* LED1 Back, Dimming, assignment C0 Back due to SR LED placement */
//	cap_lds6126_write_reg(client, 0x0022, 0xF801);
//	cap_lds6126_write_reg(client, 0x0032, 0x0001);

	/* Ambient Config */
	cap_lds6126_write_reg(client, 0x0051, 0x0A1F);
	/* Recalib Config */
	cap_lds6126_write_reg(client, 0x0052, 0x07FF);	
	/* Long Touch */
	cap_lds6126_write_reg(client, 0x0053, 0x07FF);

	/* SELC_Unit Configuration */
	cap_lds6126_write_reg(client, 0x004E, 0x5000);

	/* Noise Immunity */
//	cap_lds6126_write_reg(client, 0x0077, 0x8001);

	/* C0 ~ C1 Threshold : 200, lower threshold due to SR board */
	cap_lds6126_write_reg(client, 0x005F, 0x0001);
	cap_lds6126_write_reg(client, 0x0061, 0x0078);
	cap_lds6126_write_reg(client, 0x0062, 0x0078);

	/* Debounce & Hysteresis */
	cap_lds6126_write_reg(client, 0x0057, 0x4000);
	cap_lds6126_write_reg(client, 0x0075, 0x0002);

	/* Guard Channel Enable : C7 */
	cap_lds6126_write_reg(client, 0x007C, 0x0000);
	cap_lds6126_write_reg(client, 0x007D, 0x0000);

	/* Guard Channel Mask */
	cap_lds6126_write_reg(client, 0x007E, 0x0000);
	cap_lds6126_write_reg(client, 0x007F, 0x0000);	

	/* TOUCH ENABLE */
	cap_lds6126_write_reg(client, 0x0040, 0xB138);
	/* SOFT RESET */
	cap_lds6126_write_reg(client, 0x0001, 0x0000);

	CAP_SENSOR_INFO("Cap sensor initialization is DONE\n");

	return ret;

}

static int init_led_sensor()
{
	struct i2c_client *client ;

	client = cap_client;

	CAP_SENSOR_INFO("LED initialization\n");
	
	cap_lds6126_write_reg(client, 0x003F, 0x0006);
	cap_lds6126_write_reg(client, 0x003E, 0x8006);
	cap_lds6126_write_reg(client, 0x002F, 0xC03C);

	/* LED0 Menu, Dimming, assignm0ent C1 menu due to SR LED placement */
	cap_lds6126_write_reg(client, 0x0021, 0xF8C2);
	cap_lds6126_write_reg(client, 0x0031, 0x0002);

	/* LED1 Back, Dimming, assignment C0 Back due to SR LED placement */
	cap_lds6126_write_reg(client, 0x0022, 0xF8C1);
	cap_lds6126_write_reg(client, 0x0032, 0x0002);
	
	gpio_direction_output(TEGRA_GPIO_PU4,1);
	gpio_direction_output(TEGRA_GPIO_PR4,1);
	gpio_direction_output(TEGRA_GPIO_PR5,1);


	return 0;
}


/**********************************************************
**  Function: Called to I2C slave detection for capacitive touch driver
**  Parameter: I2C client
**  Return value: if success, returns 0
**
************************************************************/
static int __devinit cap_lds6126_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	cap_client = client;
	
	CAP_SENSOR_INFO("\n");

	cap_data = kzalloc(sizeof(struct cap_lds6126_data), GFP_KERNEL);
	if (!cap_data){
		CAP_SENSOR_ERR("Memory allocation fails\n");
		err = -ENOMEM;
		goto out;
	}

	/* Touch data processing workqueue initialization */
	INIT_WORK(&work, cap_lds6126_work_function);

	i2c_set_clientdata(cap_client, cap_data);				
	cap_client->flags = 0;
	strlcpy(cap_client->name, "cap_lds6126", I2C_NAME_SIZE);
	cap_data->status=0;	

	err = cap_lds6126_create_input_dev(client);
	if(err)
		CAP_SENSOR_ERR("Error creating input device: %d\n", err);

	cdev_add(cap_lds6126_cdev,cap_lds6126_dev,1);

	init_cap_sensor();
	init_led_sensor();

	/*Configure IRQ Pin*/
	cap_lds6126_config_irq(client);

	cap_data->attrs.attrs = cap_lds6126_attr;

	/* Register sysfs hooks */
	cap_data->attrs.attrs = cap_lds6126_attr;
	err = sysfs_create_group(&client->dev.kobj, &cap_data->attrs);
	if(err){
		dev_err(&client->dev, "Not able to create the sysfs\n");
	}

	cap_data->status=1;	

	return 0;

out:
	return err;
}

/**********************************************************
**  Function: Capacitive touch driver register to input subsystem
**  Parameter: i2c client
**  Return value: if sucess, then returns 0
**                      otherwise returns error code
**
************************************************************/
static int cap_lds6126_create_input_dev(struct i2c_client *client)
{
	int err = 0;

	/* Device related initialization */
	cap_data->input_dev = input_allocate_device();
	if (!cap_data->input_dev) {
		err = -ENOMEM;
		goto out;
	}

       CAP_SENSOR_INFO("Register as an input device\n");

	cap_data->input_dev->name = "Capacitive Sensor Driver LDS6126";
	cap_data->input_dev->phys = "/dev/input/cap";
	cap_data->input_dev->id.bustype = BUS_I2C;

	/* Tell the input system that we are processing abs */
	set_bit(EV_SYN, cap_data->input_dev->evbit);
	set_bit(EV_KEY, cap_data->input_dev->evbit);
	set_bit(KEY_BACK, cap_data->input_dev->keybit);
	set_bit(KEY_MENU, cap_data->input_dev->keybit);
	
	/* Register the Device */
	err = input_register_device(cap_data->input_dev);
	if(err){
		CAP_SENSOR_ERR("Unable to register %s input device\n",
		cap_data->input_dev->name);
		goto free_out;
        }

	return 0;

free_out:
	input_free_device(cap_data->input_dev);
out:
	return err;
}

/**********************************************************
**  Function: Called to remove devices from the adapter
**  Parameter: I2C client
**  Return value: if success, returns 0
**
************************************************************/
static int __devexit cap_lds6126_remove(struct i2c_client *client)
{
	struct cap_lds6126_data *cap_data = i2c_get_clientdata(client);

	CAP_SENSOR_INFO("\n");
	input_unregister_device(cap_data->input_dev);
	input_free_device(cap_data->input_dev);
	sysfs_remove_group(&client->dev.kobj, &cap_data->attrs);
	kfree(cap_data);
	
	return 0;
}

int cap_lds6126_open(struct inode *inode, struct file *filp)
{
	return 0;
}

struct file_operations cap_lds6126_fops = {
	.owner =    THIS_MODULE,
//	.ioctl =    cap_lds6126_ioctl,
	.open =     cap_lds6126_open,
};

/**********************************************************
**  Function: Suspend function: configure gpio and flush workqueue
**  Parameter:
**  Return value: none
**
************************************************************/
static int cap_lds6126_suspend(struct i2c_client *client, pm_message_t mesg)
{
	CAP_SENSOR_INFO("\n");
	cancel_work_sync(&work);
	flush_workqueue(cap_wq);
	cap_lds6126_write_reg(client, 0x003F, 0x0000);
	cap_lds6126_write_reg(client, 0x003E, 0x8000);
	gpio_direction_output(TEGRA_GPIO_PU4,0);
	gpio_direction_output(TEGRA_GPIO_PR4,0);
	gpio_direction_output(TEGRA_GPIO_PR5,0);

	return 0;
}

/**********************************************************
**  Function: Resume function: re-config interrupt pin setting
**  Parameter:
**  Return value: none
**
************************************************************/
static int cap_lds6126_resume(struct i2c_client *client)
{
	CAP_SENSOR_INFO("\n");
	queue_work(cap_wq, &work);
	cap_lds6126_write_reg(client, 0x003F, 0x000E);
	cap_lds6126_write_reg(client, 0x003E, 0x800E);
	gpio_direction_output(TEGRA_GPIO_PU4,1);
	gpio_direction_output(TEGRA_GPIO_PR4,1);
	gpio_direction_output(TEGRA_GPIO_PR5,1);
	return 0;
}

/**********************************************************
**  Function: Capacitive touch driver initialize
**  Parameter: none
**  Return value: if sucess, then returns 0
**                      otherwise returns error code
************************************************************/
static int __init cap_lds6126_init(void)
{
	int rc;
//	const char* label = "cap_lds6126_output";
	
	CAP_SENSOR_INFO("Init, Cap sensor is on I2C-GEN2\n");

	cap_wq = create_singlethread_workqueue("cap_wq");
	if(!cap_wq)
		return -ENOMEM;

   	if(cap_lds6126_major){
			cap_lds6126_dev = MKDEV(cap_lds6126_major, cap_lds6126_minor);
			rc = register_chrdev_region(cap_lds6126_dev, 1, "cap_lds6126");
    	}else{
            rc = alloc_chrdev_region(&cap_lds6126_dev, cap_lds6126_minor, 1,"cap_lds6126");
            cap_lds6126_major = MAJOR(cap_lds6126_dev);
    	}
    	if(rc < 0){
            CAP_SENSOR_ERR("can't get major %d\n", cap_lds6126_major);
            return rc;
    	}
	CAP_SENSOR_INFO("cdev_alloc\n");
	cap_lds6126_cdev = cdev_alloc();
	cap_lds6126_cdev->owner = THIS_MODULE;
	cap_lds6126_cdev->ops = &cap_lds6126_fops;

	cap_lds6126_class = class_create(THIS_MODULE, "cap_lds6126");
	cap_lds6126_class_device = device_create(cap_lds6126_class, NULL, MKDEV(cap_lds6126_major, cap_lds6126_minor), NULL, "cap_lds6126" );

	tegra_gpio_enable(TEGRA_GPIO_PU4);
	gpio_request(TEGRA_GPIO_PU4, "cap_lds6126");
	tegra_gpio_enable(TEGRA_GPIO_PR4);
	gpio_request(TEGRA_GPIO_PR4, "cap_lds6126_MENU");
	tegra_gpio_enable(TEGRA_GPIO_PR5);
	gpio_request(TEGRA_GPIO_PR5, "cap_lds6126_BACK");

	rc = i2c_add_driver(&cap_lds6126_driver);
	printk("CAP: rc = %d in %s\n",rc,__FUNCTION__);
	if(rc)
		CAP_SENSOR_ERR("i2c_add_driver fail\n");

	tegra_gpio_enable(TEGRA_GPIO_PK2);
	gpio_request(TEGRA_GPIO_PK2, "cap_lds6126");
	rc = gpio_get_value(TEGRA_GPIO_PK2);
	CAP_SENSOR_INFO("The RESET_CAP state is %d(High: ON)\n",rc);

	return 0;

}

/**********************************************************
**  Function: Capacitive touch driver exit function
**  Parameter: none
**  Return value: none
**
************************************************************/
static void __exit cap_lds6126_exit(void)
{
	CAP_SENSOR_INFO("\n");

	i2c_del_driver(&cap_lds6126_driver);
	if (cap_wq)
		destroy_workqueue(cap_wq);

	cdev_del(cap_lds6126_cdev);
	unregister_chrdev_region(cap_lds6126_dev, 1);
	class_destroy(cap_lds6126_class);
}

module_init(cap_lds6126_init);
module_exit(cap_lds6126_exit);
