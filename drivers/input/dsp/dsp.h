#ifndef _DSP_H
#define _DSP_H

#include "dsp_config.h"

/*
 * compiler option
 */
#define FM34_DEBUG			1

/*
 * Debug Utility
 */
#if FM34_DEBUG
#define FM34_INFO(format, arg...)	\
	printk(KERN_INFO "fm34: [%s] " format , __FUNCTION__ , ## arg)
#define FM34_I2C_DATA(array, i)	\
					do {		\
						for (i = 0; i < array[0]+1; i++) \
							FM34_INFO("i2c_data[%d] = 0x%x\n", i, array[i]);	\
					} while(0)
#else
#define FM34_INFO(format, arg...)
#define FM34_I2C_DATA(array, i)
#endif

#define FM34_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "fm34: [%s] " format , __FUNCTION__ , ## arg)

#define FM34_ERR(format, arg...)	\
	printk(KERN_ERR "fm34: [%s] " format , __FUNCTION__ , ## arg)

//-----------------------------------------

#define DRIVER_DESC     		"fm34 driver"
#define CONVERSION_TIME_MS		50

//-----------------------------------------
#define TEGRA_GPIO_PH2			58 // DSP_RST#: RST Audio DSP
#define TEGRA_GPIO_PH3			59 // DSP_PWDN#: Set Audio DSP to Power Down Mode

struct fm34_chip {
	struct input_dev	*indev;
	struct i2c_client	*client;
	struct attribute_group  attrs;
	struct miscdevice misc_dev;
	int 		        status;
	u8 i2c_data[32];
};

#endif
