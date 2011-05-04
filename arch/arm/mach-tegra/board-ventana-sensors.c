/*
 * arch/arm/mach-tegra/board-ventana-sensors.c
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/i2c.h>
#include <linux/akm8975.h>
#include <linux/mpu.h>
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <linux/nct1008.h>
#include <linux/regulator/consumer.h>	//Kenji+
#include <linux/err.h>			//Kenji+
#include <mach/gpio.h>

#include <linux/delay.h>       //ddebug
#include <media/ov5650.h>
#include <media/ov2710.h>
#ifdef CONFIG_VIDEO_YUV
#include <media/yuv_sensor.h>
#endif /* CONFIG_VIDEO_YUV */
#include <generated/mach-types.h>

#include "gpio-names.h"
#include "board.h"
#include "board-ventana.h"
#include <mach/board-ventana-misc.h>

#define LIGHT_IRQ_GPIO		TEGRA_GPIO_PZ2
#define PROX_IRQ_GPIO		TEGRA_GPIO_PG0
#define AKM8975_IRQ_GPIO	TEGRA_GPIO_PN5
#define CAMERA_POWER_GPIO	TEGRA_GPIO_PK3
#define CAMERA_CSI_MUX_SEL_GPIO	TEGRA_GPIO_PBB4
#define AC_PRESENT_GPIO		TEGRA_GPIO_PV3
#define CAP_IRQ_GPIO	TEGRA_GPIO_PX4
#define NCT1008_THERM2_GPIO	TEGRA_GPIO_PN6
#define OV5650_PWR_DN_GPIO      TEGRA_GPIO_PL0  //ddebug
#define OV5650_RST_L_GPIO       TEGRA_GPIO_PL6  //ddebug
#define YUV_SENSOR_OE_L_GPIO    TEGRA_GPIO_PL2
#define YUV_SENSOR_RST_GPIO     TEGRA_GPIO_PU2		//Kenji+
static struct regulator *reg_p_cam_avdd; /* LDO0 */
static struct regulator *reg_tegra_cam;    /* LDO6 */
enum {
	GPIO_FREE = 0,
	GPIO_REQUESTED,
};

struct camera_gpios {
	const char *name;
	int gpio;
	int enabled;
        int milliseconds;
        int requested;
};
#define CAMERA_GPIO(_name, _gpio, _enabled, _milliseconds, _requested)		\
	{						                        \
		.name = _name,				                        \
		.gpio = _gpio,				                        \
		.enabled = _enabled,			                        \
		.milliseconds = _milliseconds,				        \
		.requested = _requested,			                \
	}
extern void tegra_throttling_enable(bool enable);
static int ventana_camera_init(void)
{
//ddebug 	tegra_gpio_enable(CAMERA_POWER_GPIO);
//ddebug 	gpio_request(CAMERA_POWER_GPIO, "camera_power_en");
//ddebug 	gpio_direction_output(CAMERA_POWER_GPIO, 1);
//ddebug 	gpio_export(CAMERA_POWER_GPIO, false);
//ddebug
//ddebug 	tegra_gpio_enable(CAMERA_CSI_MUX_SEL_GPIO);
//ddebug 	gpio_request(CAMERA_CSI_MUX_SEL_GPIO, "camera_csi_sel");
//ddebug 	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 0);
//ddebug 	gpio_export(CAMERA_CSI_MUX_SEL_GPIO, false);
//ddebug
//ddebug 	return 0;
}

#ifdef CONFIG_VIDEO_OV5650
static struct camera_gpios ov5650_gpio_keys[] = {
	[0] = CAMERA_GPIO("cam_power_en", CAMERA_POWER_GPIO, 1, 0, GPIO_FREE),
	[1] = CAMERA_GPIO("cam_pwdn", OV5650_PWR_DN_GPIO, 0, 0, GPIO_FREE),
	[2] = CAMERA_GPIO("cam_rst_lo", OV5650_RST_L_GPIO, 1, 0, GPIO_FREE),
};

static int ventana_ov5650_power_on(void)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(ov5650_gpio_keys); i++) {
                tegra_gpio_enable(ov5650_gpio_keys[i].gpio);
		ret = gpio_request(ov5650_gpio_keys[i].gpio,
			ov5650_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
		gpio_direction_output(ov5650_gpio_keys[i].gpio,
			ov5650_gpio_keys[i].enabled);
                gpio_export(ov5650_gpio_keys[i].gpio, false);
	}
	return 0;
fail:
	while (i--)
		gpio_free(ov5650_gpio_keys[i].gpio);
	return ret;
}

static int ventana_ov5650_power_off(void)
{
        int i;
        i = ARRAY_SIZE(ov5650_gpio_keys);
	while (i--)
		gpio_free(ov5650_gpio_keys[i].gpio);
	return 0;
}

struct ov5650_platform_data ventana_ov5650_data = {
	.power_on = ventana_ov5650_power_on,
	.power_off = ventana_ov5650_power_off,
};
#endif /*CONFIG_VIDEO_OV5650*/
static int ventana_ov2710_power_on(void)
{
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 1);
	return 0;
}

static int ventana_ov2710_power_off(void)
{
	return 0;
}

struct ov2710_platform_data ventana_ov2710_data = {
	.power_on = ventana_ov2710_power_on,
	.power_off = ventana_ov2710_power_off,
};
#ifdef CONFIG_VIDEO_YUV
static struct camera_gpios yuv_sensor_gpio_keys[] = {
	[0] = CAMERA_GPIO("mipi_power_en", AVDD_DSI_CSI_ENB_GPIO, 1, 0, GPIO_FREE),
	[1] = CAMERA_GPIO("cam_power_en", CAMERA_POWER_GPIO, 0, 0, GPIO_FREE),
//	[1] = CAMERA_GPIO("yuv_sensor_oe_l", YUV_SENSOR_OE_L_GPIO, 0, 0, GPIO_FREE),
	[2] = CAMERA_GPIO("yuv_sensor_rst_lo", YUV_SENSOR_RST_GPIO, 1, 0, GPIO_FREE),	//Kenji+
};

static int yuv_sensor_power_on(void)
{
	int ret;
	int i;

  printk("yuv_sensor_power_on+\n");
  if (!reg_p_cam_avdd) {
    reg_p_cam_avdd = regulator_get(NULL, "p_cam_avdd");
    if (IS_ERR_OR_NULL(reg_p_cam_avdd)) {
      pr_err("EP101_ov5640_power_on LDO0: p_cam_avdd failed\n");
      reg_p_cam_avdd = NULL;
      return PTR_ERR(reg_p_cam_avdd);
    }
    regulator_set_voltage(reg_p_cam_avdd, 2850000, 2850000);
    pr_err("EP101_ov5640_power_on LDO0: p_cam_avdd OK\n");
    regulator_enable(reg_p_cam_avdd);
  }
  if (!reg_tegra_cam) {
//    reg_tegra_cam = regulator_get(NULL, "tegra_camera");
    reg_tegra_cam = regulator_get(NULL, "vcsi");
    if (IS_ERR_OR_NULL(reg_tegra_cam)) {
      pr_err("EP101_ov5640_power_on LDO6: p_tegra_cam failed\n");
      regulator_put(reg_p_cam_avdd);
      reg_tegra_cam = NULL;
      return PTR_ERR(reg_tegra_cam);
    }
    regulator_set_voltage(reg_tegra_cam, 1800000, 1800000);
    pr_err("EP101_ov5640_power_on LDO6: p_tegra_cam OK\n");
    regulator_enable(reg_tegra_cam);
  }

  for (i = 0; i < ARRAY_SIZE(yuv_sensor_gpio_keys); i++) {
    tegra_gpio_enable(yuv_sensor_gpio_keys[i].gpio);
    pr_info("gpio %d set to %d\n",yuv_sensor_gpio_keys[i].gpio, yuv_sensor_gpio_keys[i].enabled);
    ret = gpio_request(yuv_sensor_gpio_keys[i].gpio,
    yuv_sensor_gpio_keys[i].name);
    if (ret < 0) {
      pr_err("%s: gpio_request failed for gpio #%d\n",
      __func__, i);
      goto fail;
    }
    gpio_direction_output(yuv_sensor_gpio_keys[i].gpio,
    yuv_sensor_gpio_keys[i].enabled);
    gpio_export(yuv_sensor_gpio_keys[i].gpio, false);
  }
  printk("yuv_sensor_power_on-\n");
	return 0;
fail:
  regulator_disable(reg_p_cam_avdd);
  regulator_disable(reg_tegra_cam);
  regulator_put(reg_p_cam_avdd);
  regulator_put(reg_tegra_cam);
	while (i--)
		gpio_free(yuv_sensor_gpio_keys[i].gpio);
	return ret;
}

static int yuv_sensor_power_off(void)
{
  int i;

  printk("yuv_sensor_power_off+\n");
  for (i=ARRAY_SIZE(yuv_sensor_gpio_keys)-1;i>=0; i--) {
    pr_info("gpio %d set to %d\n",yuv_sensor_gpio_keys[i].gpio, !(yuv_sensor_gpio_keys[i].enabled));
    gpio_direction_output(yuv_sensor_gpio_keys[i].gpio,
    !(yuv_sensor_gpio_keys[i].enabled));
    gpio_export(yuv_sensor_gpio_keys[i].gpio, false);
  }

  if(reg_p_cam_avdd){
    regulator_disable(reg_p_cam_avdd);
    regulator_put(reg_p_cam_avdd);
    reg_p_cam_avdd = NULL;
  	pr_err("EP101_ov5640_power_off LDO0: p_cam_avdd OK\n");
  }
  if(reg_tegra_cam){
  	regulator_disable(reg_tegra_cam);
  	regulator_put(reg_tegra_cam);
    reg_tegra_cam = NULL;
  	pr_err("EP101_ov5640_power_off LDO6: p_tegra_cam OK\n");
  }

  i = ARRAY_SIZE(yuv_sensor_gpio_keys);
  while (i--)
    gpio_free(yuv_sensor_gpio_keys[i].gpio);
  printk("yuv_sensor_power_off-\n");
  return 0;
}

struct yuv_sensor_platform_data yuv_sensor_data = {
	.power_on = yuv_sensor_power_on,
	.power_off = yuv_sensor_power_off,
};

#define FRONT_CAMERA_POWER_GPIO	TEGRA_GPIO_PK4
#define FRONT_YUV_SENSOR_RST_GPIO     TEGRA_GPIO_PU3

static struct camera_gpios yuv_front_sensor_gpio_keys[] = {
	[0] = CAMERA_GPIO("mipi_power_en", AVDD_DSI_CSI_ENB_GPIO, 1, 0, GPIO_FREE),
	[1] = CAMERA_GPIO("cam_power_en", FRONT_CAMERA_POWER_GPIO, 0, 0, GPIO_FREE),
	[2] = CAMERA_GPIO("yuv_sensor_rst_lo", FRONT_YUV_SENSOR_RST_GPIO, 1, 0, GPIO_FREE),
};

static int yuv_front_sensor_power_on(void)
{
	int ret;
	int i;
  printk("yuv_front_sensor_power_on+\n");
  if (!reg_p_cam_avdd) {
    reg_p_cam_avdd = regulator_get(NULL, "p_cam_avdd");
    if (IS_ERR_OR_NULL(reg_p_cam_avdd)) {
      pr_err("AVDD Failed: p_cam_avdd failed\n");
      reg_p_cam_avdd = NULL;
      return PTR_ERR(reg_p_cam_avdd);
    }
    regulator_set_voltage(reg_p_cam_avdd, 2850000, 2850000);
    pr_err("EP101_mi1040_power_on LDO0: p_cam_avdd OK\n");
    regulator_enable(reg_p_cam_avdd);
  }
  if (!reg_tegra_cam) {
//    reg_tegra_cam = regulator_get(NULL, "tegra_camera");
    reg_tegra_cam = regulator_get(NULL, "vcsi");
    if (IS_ERR_OR_NULL(reg_tegra_cam)) {
      pr_err("IO Power Failed LDO6: p_tegra_cam failed\n");
      regulator_put(reg_p_cam_avdd);
      reg_tegra_cam = NULL;
      return PTR_ERR(reg_tegra_cam);
    }
    regulator_set_voltage(reg_tegra_cam, 1800000, 1800000);
    pr_err("EP101_mi1040_power_on LDO6: p_tegra_cam OK\n");
    regulator_enable(reg_tegra_cam);
  }

  for (i = 0; i < ARRAY_SIZE(yuv_front_sensor_gpio_keys); i++) {
    tegra_gpio_enable(yuv_front_sensor_gpio_keys[i].gpio);
    pr_info("gpio %d set to %d\n",yuv_front_sensor_gpio_keys[i].gpio,
      yuv_front_sensor_gpio_keys[i].enabled);
    ret = gpio_request(yuv_front_sensor_gpio_keys[i].gpio,
      yuv_front_sensor_gpio_keys[i].name);
    if (ret < 0) {
      pr_err("%s: gpio_request failed for gpio #%d\n",
      __func__, i);
      goto fail;
    }
    gpio_direction_output(yuv_front_sensor_gpio_keys[i].gpio,
    yuv_front_sensor_gpio_keys[i].enabled);
    gpio_export(yuv_front_sensor_gpio_keys[i].gpio, false);
  }
  printk("yuv_front_sensor_power_on-\n");
	return 0;
fail:
  regulator_disable(reg_p_cam_avdd);
  regulator_disable(reg_tegra_cam);
  regulator_put(reg_p_cam_avdd);
  regulator_put(reg_tegra_cam);
	while (i--)
		gpio_free(yuv_sensor_gpio_keys[i].gpio);
	return ret;
}

static int yuv_front_sensor_power_off(void)
{
  int i;

  printk("yuv_front_sensor_power_off+\n");
  for (i=ARRAY_SIZE(yuv_front_sensor_gpio_keys)-1;i>=0; i--) {
    pr_info("gpio %d set to %d\n",yuv_front_sensor_gpio_keys[i].gpio, !(yuv_front_sensor_gpio_keys[i].enabled));
    gpio_direction_output(yuv_front_sensor_gpio_keys[i].gpio,
    !(yuv_front_sensor_gpio_keys[i].enabled));
    gpio_export(yuv_front_sensor_gpio_keys[i].gpio, false);
  }
  if(reg_p_cam_avdd){
  	regulator_disable(reg_p_cam_avdd);
    regulator_put(reg_p_cam_avdd);
    reg_p_cam_avdd = NULL;
  	pr_err("EP101_mi1040_power_off LDO0: p_cam_avdd OK\n");
  }
  if(reg_tegra_cam){
  	regulator_disable(reg_tegra_cam);
  	regulator_put(reg_tegra_cam);
    reg_tegra_cam = NULL;
  	pr_err("EP101_mi1040_power_off LDO6: p_tegra_cam OK\n");
  }

  i = ARRAY_SIZE(yuv_front_sensor_gpio_keys);
  while (i--)
    gpio_free(yuv_front_sensor_gpio_keys[i].gpio);
  printk("yuv_front_sensor_power_off-\n");
  return 0;
}

struct yuv_sensor_platform_data yuv_front_sensor_data = {
	.power_on = yuv_front_sensor_power_on,
	.power_off = yuv_front_sensor_power_off,
};

struct yuv_sensor_platform_data yuv_rear_sensor2_data = {
	.power_on = yuv_sensor_power_on,
	.power_off = yuv_sensor_power_off,
};

#endif /* CONFIG_VIDEO_YUV */

#ifdef CONFIG_SENSORS_AK8975
static void ventana_akm8975_init(void)
{
	tegra_gpio_enable(AKM8975_IRQ_GPIO);
	gpio_request(AKM8975_IRQ_GPIO, "akm8975");
	gpio_direction_input(AKM8975_IRQ_GPIO);
}
#endif

static void ventana_bq20z75_init(void)
{
	tegra_gpio_enable(AC_PRESENT_GPIO);
	gpio_request(AC_PRESENT_GPIO, "ac_present");
	gpio_direction_input(AC_PRESENT_GPIO);
}

static void ventana_nct1008_init(void)
{
	tegra_gpio_enable(NCT1008_THERM2_GPIO);
	gpio_request(NCT1008_THERM2_GPIO, "temp_alert");
	gpio_direction_input(NCT1008_THERM2_GPIO);
}

static struct nct1008_platform_data ventana_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.conv_rate = 0x08,
	.offset = 0,
	.hysteresis = 0,
	.shutdown_ext_limit = 115,
	.shutdown_local_limit = 120,
	.throttling_ext_limit = 90,
	.alarm_fn = tegra_throttling_enable,
};

static const struct i2c_board_info ventana_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO("isl29018", 0x44),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ2),
	},
	{
		I2C_BOARD_INFO("ami304", 0x0E),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN5),
	},
	{
		I2C_BOARD_INFO("kxtf9", 0x0F),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN4),
	},
};

static const struct i2c_board_info ventana_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("bq20z45-battery", 0x0B),
		//.irq = TEGRA_GPIO_TO_IRQ(AC_PRESENT_GPIO),
	},
#ifdef CONFIG_ASUSEC
	{
		I2C_BOARD_INFO("asusec", 0x19),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS2),
	},
#endif
};

//ddebug static struct pca953x_platform_data ventana_tca6416_data = {
//ddebug 	.gpio_base      = TEGRA_NR_GPIOS + 4, /* 4 gpios are already requested by tps6586x */
//ddebug };

//ddebug static struct pca954x_platform_mode ventana_pca9546_modes[] = {
//ddebug 	{ .adap_id = 6, }, /* REAR CAM1 */
//ddebug 	{ .adap_id = 7, }, /* REAR CAM2 */
//ddebug 	{ .adap_id = 8, }, /* FRONT CAM3 */
//ddebug };

//ddebug static struct pca954x_platform_data ventana_pca9546_data = {
//ddebug 	.modes	  = ventana_pca9546_modes,
//ddebug 	.num_modes      = ARRAY_SIZE(ventana_pca9546_modes),
//ddebug };

//ddebug static const struct i2c_board_info ventana_i2c3_board_info_tca6416[] = {
//ddebug 	{
//ddebug 		I2C_BOARD_INFO("tca6416", 0x20),
//ddebug 		.platform_data = &ventana_tca6416_data,
//ddebug 	},
//ddebug };

//ddebug static const struct i2c_board_info ventana_i2c3_board_info_pca9546[] = {
//ddebug 	{
//ddebug 		I2C_BOARD_INFO("pca9546", 0x70),
//ddebug 		.platform_data = &ventana_pca9546_data,
//ddebug 	},
//ddebug };

static struct i2c_board_info ventana_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(NCT1008_THERM2_GPIO),
		.platform_data = &ventana_nct1008_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN6),
	},

#ifdef CONFIG_SENSORS_AK8975
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.irq = TEGRA_GPIO_TO_IRQ(AKM8975_IRQ_GPIO),
	},
#endif
};

//ddebug static struct i2c_board_info ventana_i2c7_board_info[] = {
static struct i2c_board_info ventana_i2c3_board_info[] = {  //ddebug
#ifdef CONFIG_VIDEO_OV5650
	{
//Kenji-		I2C_BOARD_INFO("ov5650", 0x36),
		I2C_BOARD_INFO("ov5650", 0x36),
		.platform_data = &ventana_ov5650_data,
	},
#endif /*CONFIG_VIDEO_OV5650*/
//ddebug - start
#ifdef CONFIG_VIDEO_YUV
	{
		I2C_BOARD_INFO("ov5640", 0x3C),
		.platform_data = &yuv_sensor_data,
 	},
#endif /* CONFIG_VIDEO_YUV */
//ddebug - end
};

//+ Warlock
#ifdef CONFIG_VIDEO_YUV
static struct i2c_board_info front_sensor_i2c3_board_info[] = {  //ddebug
	{
		I2C_BOARD_INFO("mi1040", 0x48),
		.platform_data = &yuv_front_sensor_data,
 	},
};
static struct i2c_board_info rear_sensor2_i2c3_board_info[] = {  //ddebug
	{
		I2C_BOARD_INFO("mi5140", 0x3D),
		.platform_data = &yuv_rear_sensor2_data,
}
};
#endif /* CONFIG_VIDEO_YUV */


//-

static struct i2c_board_info ventana_i2c8_board_info[] = {
	{
		I2C_BOARD_INFO("ov2710", 0x36),
		.platform_data = &ventana_ov2710_data,
	},
};


#ifdef CONFIG_MPU_SENSORS_MPU3050
#define SENSOR_MPU_NAME "mpu3050"
static struct mpu3050_platform_data mpu3050_data = {
	.int_config  = 0x10,
	.orientation = { 0, 1, 0, -1, 0, 0, 0, 0, 1 },  /* Orientation matrix for MPU on ventana */
	.level_shifter = 0,
	.accel = {
#ifdef CONFIG_MPU_SENSORS_KXTF9
	.get_slave_descr = get_accel_slave_descr,
#else
	.get_slave_descr = NULL,
#endif
	.adapt_num   = 0,
	.bus         = EXT_SLAVE_BUS_SECONDARY,
	.address     = 0x0F,
	.orientation = { 1, 0, 0, 0, 1, 0, 0, 0, 1 },  /* Orientation matrix for Kionix on ventana */
    .irq =  TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN4),
	},

	.compass = {
#ifdef CONFIG_MPU_SENSORS_AMI304
	.get_slave_descr = get_compass_slave_descr,
	.irq =  TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN5),
#else
	.get_slave_descr = NULL,
#endif
	.adapt_num   = 0,            /* bus number 0 on EP101 */
	.bus         = EXT_SLAVE_BUS_PRIMARY,
	.address     = 0x0E,
	.orientation = { -1, 0, 0, 0, 1, 0, 0, 0, -1 },  /* Orientation matrix for AMIT on ventana */
	},
};

static struct i2c_board_info __initdata mpu3050_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ4),
		.platform_data = &mpu3050_data,
	},
};

static void ventana_mpuirq_init(void)
{
	pr_info("*** MPU START *** ventana_mpuirq_init...\n");
	tegra_gpio_enable(TEGRA_GPIO_PZ4);
	gpio_request(TEGRA_GPIO_PZ4, SENSOR_MPU_NAME);
	gpio_direction_input(TEGRA_GPIO_PZ4);
        signed char orientationGyroEP102 [9] = { 0, 1, 0, -1, 0, 0, 0, 0, 1 };
        signed char orientationAccelEP102 [9] = { -1, 0, 0, 0, 1, 0, 0, 0, -1 };
        signed char orientationMagEP102 [9] = { 1, 0, 0, 0, -1, 0, 0, 0, -1 };
        signed char orientationGyroEP103 [9] = { -1, 0, 0, 0, -1, 0, 0, 0, 1 };
        signed char orientationAccelEP103 [9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        signed char orientationMagEP103 [9] = { 0, -1, 0, 1, 0, 0, 0, 0, 1 };
        if( ASUSGetProjectID() == 102 ){
           memcpy( mpu3050_data.orientation, orientationGyroEP102, sizeof(mpu3050_data.orientation));
           memcpy( mpu3050_data.accel.orientation, orientationAccelEP102, sizeof(mpu3050_data.accel.orientation));
           memcpy( mpu3050_data.compass.orientation, orientationMagEP102, sizeof(mpu3050_data.compass.orientation));
        }else if( ASUSGetProjectID() == 103 ){
           memcpy( mpu3050_data.orientation, orientationGyroEP103, sizeof(mpu3050_data.orientation));
           memcpy( mpu3050_data.accel.orientation, orientationAccelEP103, sizeof(mpu3050_data.accel.orientation));
           memcpy( mpu3050_data.compass.orientation, orientationMagEP103, sizeof(mpu3050_data.compass.orientation));
        }
	pr_info("*** MPU END *** ventana_mpuirq_init...\n");
}
#endif

static const struct i2c_board_info cap_i2c1_board_info[] = {
#ifdef CONFIG_SENSORS_CAP_LDS6126
	{
		I2C_BOARD_INFO("cap_lds6126", 0x2d),
		.irq = TEGRA_GPIO_TO_IRQ(CAP_IRQ_GPIO),
	},
#endif
};
static const struct i2c_board_info light_i2c1_board_info[] = {
#ifdef CONFIG_SENSORS_AL3000
	{
		I2C_BOARD_INFO("al3000a", 0x1c),
		.irq = TEGRA_GPIO_TO_IRQ(LIGHT_IRQ_GPIO),
	},
#endif
};
static const struct i2c_board_info proximity_i2c1_board_info[] = {
#ifdef CONFIG_SENSORS_PROX_LDS6202
	{
		I2C_BOARD_INFO("prox_lds6202", 0x2c),
		.irq = TEGRA_GPIO_TO_IRQ(PROX_IRQ_GPIO),
	},
#endif
};
int __init ventana_sensors_init(void)
{
	struct board_info BoardInfo;

#ifdef CONFIG_SENSORS_AK8975
	ventana_akm8975_init();
#endif
#ifdef CONFIG_MPU_SENSORS_MPU3050
	ventana_mpuirq_init();
#endif
	ventana_camera_init();

	ventana_nct1008_init();
	i2c_register_board_info(0, ventana_i2c0_board_info,
		ARRAY_SIZE(ventana_i2c0_board_info));

	i2c_register_board_info(2, ventana_i2c2_board_info,
		ARRAY_SIZE(ventana_i2c2_board_info));

	i2c_register_board_info(2, cap_i2c1_board_info,
                ARRAY_SIZE(cap_i2c1_board_info));

	i2c_register_board_info(2, light_i2c1_board_info,
		ARRAY_SIZE(light_i2c1_board_info));

	i2c_register_board_info(2, proximity_i2c1_board_info,
		ARRAY_SIZE(proximity_i2c1_board_info));
//+ ov5640 rear camera
	i2c_register_board_info(3, ventana_i2c3_board_info,
		ARRAY_SIZE(ventana_i2c3_board_info));
//-
//+ Front camera
	i2c_register_board_info(3, front_sensor_i2c3_board_info,
		ARRAY_SIZE(front_sensor_i2c3_board_info));
//-
//+ mi5140 rear camera
	i2c_register_board_info(3, rear_sensor2_i2c3_board_info,
		ARRAY_SIZE(rear_sensor2_i2c3_board_info));
//-

	i2c_register_board_info(4, ventana_i2c4_board_info,
		ARRAY_SIZE(ventana_i2c4_board_info));

//ddebug 	i2c_register_board_info(7, ventana_i2c7_board_info,
//ddebug 		ARRAY_SIZE(ventana_i2c7_board_info));

	i2c_register_board_info(8, ventana_i2c8_board_info,
		ARRAY_SIZE(ventana_i2c8_board_info));


#ifdef CONFIG_MPU_SENSORS_MPU3050
	i2c_register_board_info(0, mpu3050_i2c0_boardinfo,
		ARRAY_SIZE(mpu3050_i2c0_boardinfo));
#endif

	return 0;
}

#ifdef CONFIG_VIDEO_OV5650
//ddebug - start
//ddebug struct camera_gpios {
//ddebug 	const char *name;
//ddebug 	int gpio;
//ddebug 	int enabled;
//ddebug };
//ddebug
//ddebug #define CAMERA_GPIO(_name, _gpio, _enabled)		\
//ddebug 	{						\
//ddebug 		.name = _name,				\
//ddebug 		.gpio = _gpio,				\
//ddebug 		.enabled = _enabled,			\
//ddebug 	}
//ddebug
//ddebug
//ddebug static struct camera_gpios ov5650_gpio_keys[] = {
//ddebug 	[0] = OV5650_GPIO("en_avdd_csi", AVDD_DSI_CSI_ENB_GPIO, 1),
//ddebug 	[1] = OV5650_GPIO("cam2_pwdn", CAM2_PWR_DN_GPIO, 0),
//ddebug 	[2] = OV5650_GPIO("cam2_rst_lo", CAM2_RST_L_GPIO, 1),
//ddebug 	[3] = OV5650_GPIO("cam2_af_pwdn_lo", CAM2_AF_PWR_DN_L_GPIO, 0),
//ddebug 	[4] = OV5650_GPIO("cam2_ldo_shdn_lo", CAM2_LDO_SHUTDN_L_GPIO, 1),
//ddebug 	[5] = OV5650_GPIO("cam2_i2c_mux_rst_lo", CAM2_I2C_MUX_RST_GPIO, 1),
//ddebug };
//ddebug
//ddebug int __init ventana_ov5650_late_init(void)
//ddebug {
//ddebug 	int ret;
//ddebug 	int i;
//ddebug
//ddebug 	if (!machine_is_ventana())
//ddebug 		return 0;
//ddebug
//ddebug 	i2c_new_device(i2c_get_adapter(3), ventana_i2c3_board_info_tca6416);
//ddebug
//ddebug 	for (i = 0; i < ARRAY_SIZE(ov5650_gpio_keys); i++) {
//ddebug 		ret = gpio_request(ov5650_gpio_keys[i].gpio,
//ddebug 			ov5650_gpio_keys[i].name);
//ddebug 		if (ret < 0) {
//ddebug 			pr_err("%s: gpio_request failed for gpio #%d\n",
//ddebug 				__func__, i);
//ddebug 			goto fail;
//ddebug 		}
//ddebug 		gpio_direction_output(ov5650_gpio_keys[i].gpio,
//ddebug 			ov5650_gpio_keys[i].enabled);
//ddebug 		gpio_export(ov5650_gpio_keys[i].gpio, false);
//ddebug 	}
//ddebug
//ddebug 	i2c_new_device(i2c_get_adapter(3), ventana_i2c3_board_info_pca9546);
//ddebug
//ddebug 	return 0;
//ddebug
//ddebug fail:
//ddebug 	while (i--)
//ddebug 		gpio_free(ov5650_gpio_keys[i].gpio);
//ddebug 	return ret;
//ddebug }
//ddebug
//ddebug late_initcall(ventana_ov5650_late_init);

#endif /* CONFIG_VIDEO_OV5650 */
