/*
 * kernel/drivers/media/video/tegra
 *
 * Aptina MT9D115 sensor driver
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/yuv_sensor.h>

#define SENSOR_WIDTH_REG 0x2703
#define SENSOR_640_WIDTH_VAL 0x0280
#define MI1040_SENSOR_NAME "mi1040"

#define SEQUENCE_WAIT_MS  SENSOR_WAIT_MS
#define SEQUENCE_END      SENSOR_TABLE_END
#define SENSOR_BYTE_WRITE    (SEQUENCE_END+1)
#define SENSOR_WORD_WRITE    (SEQUENCE_END+2)
#define SENSOR_MASK_BYTE_WRITE  (SEQUENCE_END+3)
#define SENSOR_MASK_WORD_WRITE  (SEQUENCE_END+4)
#define SENSOR_BYTE_READ  (SEQUENCE_END+5)
#define SENSOR_WORD_READ  (SEQUENCE_END+6)

struct sensor_reg {
	u16 addr;
	u16 val;
};

struct sensor_reg_ex {
  u16 cmd;
	u16 addr;
	u16 val;
};

struct sensor_info {
	int mode;
	struct i2c_client *i2c_client;
	struct yuv_sensor_platform_data *pdata;
};

static struct sensor_info *info;

static struct sensor_reg_ex mode_1280x960[] =
{
//+
//[Initialization]
// Hardware Reset, need to be done by hardware or host processor.
// Reset
//POLL_FIELD= COMMAND_REGISTER, HOST_COMMAND_1, !=0, DELAY=10, TIMEOUT=100
// LOAD = Step1-Reset 			//Reset

{SENSOR_WORD_WRITE, 0x001A, 0x0001},
{SEQUENCE_WAIT_MS, 0, 10}, 	//delay=10
{SENSOR_WORD_WRITE, 0x001A, 0x0000},
{SEQUENCE_WAIT_MS, 0, 50}, 	//delay=50
{SENSOR_WORD_WRITE,0x301A, 0x0234} , // RESET_REGISTER
// [Step2-PLL_Timing]
// LOAD=PLL_settings
{SENSOR_WORD_WRITE,0x098E, 0x1000} , // LOGICAL_ADDRESS_ACCESS
{SENSOR_BYTE_WRITE,0xC97E, 0x01} , 	// CAM_SYSCTL_PLL_ENABLE
{SENSOR_WORD_WRITE,0xC980, 0x0120} , 	// CAM_SYSCTL_PLL_DIVIDER_M_N
{SENSOR_WORD_WRITE,0xC982, 0x0700} , 	// CAM_SYSCTL_PLL_DIVIDER_P
//;REG= 0xC984, 0x8040} , 	// CAM_PORT_OUTPUT_CONTROL

// LOAD=Timing_settings
{SENSOR_WORD_WRITE,0xC800, 0x0004} , 	// CAM_SENSOR_CFG_Y_ADDR_START
{SENSOR_WORD_WRITE,0xC802, 0x0004} , 	// CAM_SENSOR_CFG_X_ADDR_START
{SENSOR_WORD_WRITE,0xC804, 0x03CB} , 	// CAM_SENSOR_CFG_Y_ADDR_END
{SENSOR_WORD_WRITE,0xC806, 0x050B} , 	// CAM_SENSOR_CFG_X_ADDR_END
// data length is double words
{SENSOR_WORD_WRITE,0xC808, 0x02DC} ,
{SENSOR_WORD_WRITE,0xC80A, 0x6C00} , 	// CAM_SENSOR_CFG_PIXCLK
//
{SENSOR_WORD_WRITE,0xC80C, 0x0001} , 	// CAM_SENSOR_CFG_ROW_SPEED
{SENSOR_WORD_WRITE,0xC80E, 0x00DB} , 	// CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN
{SENSOR_WORD_WRITE,0xC810, 0x05B3} , 	// CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX
{SENSOR_WORD_WRITE,0xC812, 0x03EE} , 	// CAM_SENSOR_CFG_FRAME_LENGTH_LINES
{SENSOR_WORD_WRITE,0xC814, 0x0636} , 	// CAM_SENSOR_CFG_LINE_LENGTH_PCK
{SENSOR_WORD_WRITE,0xC816, 0x0060} , 	// CAM_SENSOR_CFG_FINE_CORRECTION
{SENSOR_WORD_WRITE,0xC818, 0x03C3} , 	// CAM_SENSOR_CFG_CPIPE_LAST_ROW
{SENSOR_WORD_WRITE,0xC834, 0x0000} , 	// CAM_SENSOR_CONTROL_READ_MODE
{SENSOR_WORD_WRITE,0xC854, 0x0000} , 	// CAM_CROP_WINDOW_XOFFSET
{SENSOR_WORD_WRITE,0xC856, 0x0000} , 	// CAM_CROP_WINDOW_YOFFSET
{SENSOR_WORD_WRITE,0xC858, 0x0500} , 	// CAM_CROP_WINDOW_WIDTH
{SENSOR_WORD_WRITE,0xC85A, 0x03C0} , 	// CAM_CROP_WINDOW_HEIGHT
{SENSOR_BYTE_WRITE,0xC85C, 0x03} , 	// CAM_CROP_CROPMODE
{SENSOR_WORD_WRITE,0xC868, 0x0500} , 	// CAM_OUTPUT_WIDTH
{SENSOR_WORD_WRITE,0xC86A, 0x03C0} , 	// CAM_OUTPUT_HEIGHT
{SENSOR_WORD_WRITE,0xC88C, 0x1E02} , 	// CAM_AET_MAX_FRAME_RATE
// -- Rev2, 02242011, Alias
// Changed to varied frame rate from 15~30fps
//{SENSOR_WORD_WRITE,0xC88E, 0x0F00} ,	//0x1E02} , 	// CAM_AET_MIN_FRAME_RATE
// -- Rev3, 03082011, Alias
// Changed to varied frame rate to 7.5~30fps
{SENSOR_WORD_WRITE,0xC88E, 0x0780} ,	//0x1E02} , 	// CAM_AET_MIN_FRAME_RATE
{SENSOR_WORD_WRITE,0xC914, 0x0000} , 	// CAM_STAT_AWB_CLIP_WINDOW_XSTART
{SENSOR_WORD_WRITE,0xC916, 0x0000} , 	// CAM_STAT_AWB_CLIP_WINDOW_YSTART
{SENSOR_WORD_WRITE,0xC918, 0x04FF} , 	// CAM_STAT_AWB_CLIP_WINDOW_XEND
{SENSOR_WORD_WRITE,0xC91A, 0x03BF} , 	// CAM_STAT_AWB_CLIP_WINDOW_YEND
{SENSOR_WORD_WRITE,0xC91C, 0x0000} , 	// CAM_STAT_AE_INITIAL_WINDOW_XSTART
{SENSOR_WORD_WRITE,0xC91E, 0x0000} , 	// CAM_STAT_AE_INITIAL_WINDOW_YSTART
{SENSOR_WORD_WRITE,0xC920, 0x00FF} , 	// CAM_STAT_AE_INITIAL_WINDOW_XEND
{SENSOR_WORD_WRITE,0xC922, 0x00BF} , 	// CAM_STAT_AE_INITIAL_WINDOW_YEND
{SENSOR_BYTE_WRITE,0xE801, 0x00} , 	// AUTO_BINNING_MODE


// [Step3-Recommended]
// [Sensor optimization]
{SENSOR_WORD_WRITE,0x316A, 0x8270} , 	// DAC_TXLO_ROW
{SENSOR_WORD_WRITE,0x316C, 0x8270} , 	// DAC_TXLO
{SENSOR_WORD_WRITE,0x3ED0, 0x2305} , 	// DAC_LD_4_5
{SENSOR_WORD_WRITE,0x3ED2, 0x77CF} , 	// DAC_LD_6_7
{SENSOR_WORD_WRITE,0x316E, 0x8202} , 	// DAC_ECL
{SENSOR_WORD_WRITE,0x3180, 0x87FF} , 	// DELTA_DK_CONTROL
{SENSOR_WORD_WRITE,0x30D4, 0x6080} , 	// COLUMN_CORRECTION
{SENSOR_WORD_WRITE,0xA802, 0x0008} , 	// AE_TRACK_MODE

// LOAD=Errata item 1
{SENSOR_WORD_WRITE,0x3E14, 0xFF39} , 	// SAMP_COL_PUP2

// LOAD=Errata item 2
{SENSOR_WORD_WRITE,0x301A, 0x8234} , 	// RESET_REGISTER

// -- Rev2, 02242011, Alias
// removed FW patch due to no binning application

// PGA parameter and APGA
//[Step4-APGA]
//[TP101_SOC1040_APGA]
{SENSOR_WORD_WRITE,0x098E, 0x495E} , 	// LOGICAL_ADDRESS_ACCESS [CAM_PGA_PGA_CONTROL]
{SENSOR_WORD_WRITE,0xC95E, 0x0000} , 	// CAM_PGA_PGA_CONTROL
{SENSOR_WORD_WRITE,0x3640, 0x02B0} , 	// P_G1_P0Q0
{SENSOR_WORD_WRITE,0x3642, 0x8063} , 	// P_G1_P0Q1
{SENSOR_WORD_WRITE,0x3644, 0x78D0} , 	// P_G1_P0Q2
{SENSOR_WORD_WRITE,0x3646, 0x50CC} , 	// P_G1_P0Q3
{SENSOR_WORD_WRITE,0x3648, 0x3511} , 	// P_G1_P0Q4
{SENSOR_WORD_WRITE,0x364A, 0x0110} , 	// P_R_P0Q0
{SENSOR_WORD_WRITE,0x364C, 0xBD8A} , 	// P_R_P0Q1
{SENSOR_WORD_WRITE,0x364E, 0x0CD1} , 	// P_R_P0Q2
{SENSOR_WORD_WRITE,0x3650, 0x24ED} , 	// P_R_P0Q3
{SENSOR_WORD_WRITE,0x3652, 0x7C11} , 	// P_R_P0Q4
{SENSOR_WORD_WRITE,0x3654, 0x0150} , 	// P_B_P0Q0
{SENSOR_WORD_WRITE,0x3656, 0x124C} , 	// P_B_P0Q1
{SENSOR_WORD_WRITE,0x3658, 0x3130} , 	// P_B_P0Q2
{SENSOR_WORD_WRITE,0x365A, 0x508C} , 	// P_B_P0Q3
{SENSOR_WORD_WRITE,0x365C, 0x21F1} , 	// P_B_P0Q4
{SENSOR_WORD_WRITE,0x365E, 0x0090} , 	// P_G2_P0Q0
{SENSOR_WORD_WRITE,0x3660, 0xBFCA} , 	// P_G2_P0Q1
{SENSOR_WORD_WRITE,0x3662, 0x0A11} , 	// P_G2_P0Q2
{SENSOR_WORD_WRITE,0x3664, 0x4F4B} , 	// P_G2_P0Q3
{SENSOR_WORD_WRITE,0x3666, 0x28B1} , 	// P_G2_P0Q4
{SENSOR_WORD_WRITE,0x3680, 0x50A9} , 	// P_G1_P1Q0
{SENSOR_WORD_WRITE,0x3682, 0xA04B} , 	// P_G1_P1Q1
{SENSOR_WORD_WRITE,0x3684, 0x0E2D} , 	// P_G1_P1Q2
{SENSOR_WORD_WRITE,0x3686, 0x73EC} , 	// P_G1_P1Q3
{SENSOR_WORD_WRITE,0x3688, 0x164F} , 	// P_G1_P1Q4
{SENSOR_WORD_WRITE,0x368A, 0xF829} , 	// P_R_P1Q0
{SENSOR_WORD_WRITE,0x368C, 0xC1A8} , 	// P_R_P1Q1
{SENSOR_WORD_WRITE,0x368E, 0xB0EC} , 	// P_R_P1Q2
{SENSOR_WORD_WRITE,0x3690, 0xE76A} , 	// P_R_P1Q3
{SENSOR_WORD_WRITE,0x3692, 0x69AF} , 	// P_R_P1Q4
{SENSOR_WORD_WRITE,0x3694, 0x378C} , 	// P_B_P1Q0
{SENSOR_WORD_WRITE,0x3696, 0xA70D} , 	// P_B_P1Q1
{SENSOR_WORD_WRITE,0x3698, 0x884F} , 	// P_B_P1Q2
{SENSOR_WORD_WRITE,0x369A, 0xEE8B} , 	// P_B_P1Q3
{SENSOR_WORD_WRITE,0x369C, 0x5DEF} , 	// P_B_P1Q4
{SENSOR_WORD_WRITE,0x369E, 0x27CC} , 	// P_G2_P1Q0
{SENSOR_WORD_WRITE,0x36A0, 0xCAAC} , 	// P_G2_P1Q1
{SENSOR_WORD_WRITE,0x36A2, 0x840E} , 	// P_G2_P1Q2
{SENSOR_WORD_WRITE,0x36A4, 0xDAA9} , 	// P_G2_P1Q3
{SENSOR_WORD_WRITE,0x36A6, 0xF00C} , 	// P_G2_P1Q4
{SENSOR_WORD_WRITE,0x36C0, 0x1371} , 	// P_G1_P2Q0
{SENSOR_WORD_WRITE,0x36C2, 0x272F} , 	// P_G1_P2Q1
{SENSOR_WORD_WRITE,0x36C4, 0x2293} , 	// P_G1_P2Q2
{SENSOR_WORD_WRITE,0x36C6, 0xE6D0} , 	// P_G1_P2Q3
{SENSOR_WORD_WRITE,0x36C8, 0xEC32} , 	// P_G1_P2Q4
{SENSOR_WORD_WRITE,0x36CA, 0x11B1} , 	// P_R_P2Q0
{SENSOR_WORD_WRITE,0x36CC, 0x7BAF} , 	// P_R_P2Q1
{SENSOR_WORD_WRITE,0x36CE, 0x5813} , 	// P_R_P2Q2
{SENSOR_WORD_WRITE,0x36D0, 0xB871} , 	// P_R_P2Q3
{SENSOR_WORD_WRITE,0x36D2, 0x8913} , 	// P_R_P2Q4
{SENSOR_WORD_WRITE,0x36D4, 0x4610} , 	// P_B_P2Q0
{SENSOR_WORD_WRITE,0x36D6, 0x7EEE} , 	// P_B_P2Q1
{SENSOR_WORD_WRITE,0x36D8, 0x0DF3} , 	// P_B_P2Q2
{SENSOR_WORD_WRITE,0x36DA, 0xB84F} , 	// P_B_P2Q3
{SENSOR_WORD_WRITE,0x36DC, 0xB532} , 	// P_B_P2Q4
{SENSOR_WORD_WRITE,0x36DE, 0x1171} , 	// P_G2_P2Q0
{SENSOR_WORD_WRITE,0x36E0, 0x13CF} , 	// P_G2_P2Q1
{SENSOR_WORD_WRITE,0x36E2, 0x22F3} , 	// P_G2_P2Q2
{SENSOR_WORD_WRITE,0x36E4, 0xE090} , 	// P_G2_P2Q3
{SENSOR_WORD_WRITE,0x36E6, 0x8133} , 	// P_G2_P2Q4
{SENSOR_WORD_WRITE,0x3700, 0x88AE} , 	// P_G1_P3Q0
{SENSOR_WORD_WRITE,0x3702, 0x00EA} , 	// P_G1_P3Q1
{SENSOR_WORD_WRITE,0x3704, 0x344F} , 	// P_G1_P3Q2
{SENSOR_WORD_WRITE,0x3706, 0xEC88} , 	// P_G1_P3Q3
{SENSOR_WORD_WRITE,0x3708, 0x3E91} , 	// P_G1_P3Q4
{SENSOR_WORD_WRITE,0x370A, 0xF12D} , 	// P_R_P3Q0
{SENSOR_WORD_WRITE,0x370C, 0xB0EF} , 	// P_R_P3Q1
{SENSOR_WORD_WRITE,0x370E, 0x77CD} , 	// P_R_P3Q2
{SENSOR_WORD_WRITE,0x3710, 0x7930} , 	// P_R_P3Q3
{SENSOR_WORD_WRITE,0x3712, 0x5C12} , 	// P_R_P3Q4
{SENSOR_WORD_WRITE,0x3714, 0x500C} , 	// P_B_P3Q0
{SENSOR_WORD_WRITE,0x3716, 0x22CE} , 	// P_B_P3Q1
{SENSOR_WORD_WRITE,0x3718, 0x2370} , 	// P_B_P3Q2
{SENSOR_WORD_WRITE,0x371A, 0x258F} , 	// P_B_P3Q3
{SENSOR_WORD_WRITE,0x371C, 0x3D30} , 	// P_B_P3Q4
{SENSOR_WORD_WRITE,0x371E, 0x370C} , 	// P_G2_P3Q0
{SENSOR_WORD_WRITE,0x3720, 0x03ED} , 	// P_G2_P3Q1
{SENSOR_WORD_WRITE,0x3722, 0x9AD0} , 	// P_G2_P3Q2
{SENSOR_WORD_WRITE,0x3724, 0x7ECF} , 	// P_G2_P3Q3
{SENSOR_WORD_WRITE,0x3726, 0x1093} , 	// P_G2_P3Q4
{SENSOR_WORD_WRITE,0x3740, 0x2391} , 	// P_G1_P4Q0
{SENSOR_WORD_WRITE,0x3742, 0xAAD0} , 	// P_G1_P4Q1
{SENSOR_WORD_WRITE,0x3744, 0x28F2} , 	// P_G1_P4Q2
{SENSOR_WORD_WRITE,0x3746, 0xBA4F} , 	// P_G1_P4Q3
{SENSOR_WORD_WRITE,0x3748, 0xC536} , 	// P_G1_P4Q4
{SENSOR_WORD_WRITE,0x374A, 0x1472} , 	// P_R_P4Q0
{SENSOR_WORD_WRITE,0x374C, 0xD110} , 	// P_R_P4Q1
{SENSOR_WORD_WRITE,0x374E, 0x2933} , 	// P_R_P4Q2
{SENSOR_WORD_WRITE,0x3750, 0xD0D1} , 	// P_R_P4Q3
{SENSOR_WORD_WRITE,0x3752, 0x9F37} , 	// P_R_P4Q4
{SENSOR_WORD_WRITE,0x3754, 0x34D1} , 	// P_B_P4Q0
{SENSOR_WORD_WRITE,0x3756, 0x1C6C} , 	// P_B_P4Q1
{SENSOR_WORD_WRITE,0x3758, 0x3FD2} , 	// P_B_P4Q2
{SENSOR_WORD_WRITE,0x375A, 0xCB72} , 	// P_B_P4Q3
{SENSOR_WORD_WRITE,0x375C, 0xBA96} , 	// P_B_P4Q4
{SENSOR_WORD_WRITE,0x375E, 0x1551} , 	// P_G2_P4Q0
{SENSOR_WORD_WRITE,0x3760, 0xB74F} , 	// P_G2_P4Q1
{SENSOR_WORD_WRITE,0x3762, 0x1672} , 	// P_G2_P4Q2
{SENSOR_WORD_WRITE,0x3764, 0x84F1} , 	// P_G2_P4Q3
{SENSOR_WORD_WRITE,0x3766, 0xC2D6} , 	// P_G2_P4Q4
{SENSOR_WORD_WRITE,0x3782, 0x01E0} , 	// CENTER_ROW
{SENSOR_WORD_WRITE,0x3784, 0x0280} , 	// CENTER_COLUMN
{SENSOR_WORD_WRITE,0x37C0, 0xA6EA} , 	// P_GR_Q5
{SENSOR_WORD_WRITE,0x37C2, 0x874B} , 	// P_RD_Q5
{SENSOR_WORD_WRITE,0x37C4, 0x85CB} , 	// P_BL_Q5
{SENSOR_WORD_WRITE,0x37C6, 0x968A} , 	// P_GB_Q5
{SENSOR_WORD_WRITE,0x098E, 0x0000} , 	//  LOGICAL addressing
{SENSOR_WORD_WRITE,0xC960, 0x0AF0} , 	//  CAM_PGA_L_CONFIG_COLOUR_TEMP
{SENSOR_WORD_WRITE,0xC962, 0x79E2} , 	//  CAM_PGA_L_CONFIG_GREEN_RED_Q14
{SENSOR_WORD_WRITE,0xC964, 0x5EC8} ,	//0x56C8} , 	//  CAM_PGA_L_CONFIG_RED_Q14
{SENSOR_WORD_WRITE,0xC966, 0x791F} , 	//  CAM_PGA_L_CONFIG_GREEN_BLUE_Q14
{SENSOR_WORD_WRITE,0xC968, 0x76EE} , 	//  CAM_PGA_L_CONFIG_BLUE_Q14
{SENSOR_WORD_WRITE,0xC96A, 0x0FA0} , 	//  CAM_PGA_M_CONFIG_COLOUR_TEMP
{SENSOR_WORD_WRITE,0xC96C, 0x7DFA} , 	//  CAM_PGA_M_CONFIG_GREEN_RED_Q14
{SENSOR_WORD_WRITE,0xC96E, 0x7DAF} , 	//  CAM_PGA_M_CONFIG_RED_Q14
{SENSOR_WORD_WRITE,0xC970, 0x7E02} , 	//  CAM_PGA_M_CONFIG_GREEN_BLUE_Q14
{SENSOR_WORD_WRITE,0xC972, 0x7E0A} , 	//  CAM_PGA_M_CONFIG_BLUE_Q14
{SENSOR_WORD_WRITE,0xC974, 0x1964} , 	//  CAM_PGA_R_CONFIG_COLOUR_TEMP
{SENSOR_WORD_WRITE,0xC976, 0x7CDC} , 	//  CAM_PGA_R_CONFIG_GREEN_RED_Q14
{SENSOR_WORD_WRITE,0xC978, 0x7838} ,	//0x7038} , 	//  CAM_PGA_R_CONFIG_RED_Q14
{SENSOR_WORD_WRITE,0xC97A, 0x7C2F} , 	//  CAM_PGA_R_CONFIG_GREEN_BLUE_Q14
{SENSOR_WORD_WRITE,0xC97C, 0x7792} , 	//  CAM_PGA_R_CONFIG_BLUE_Q14
{SENSOR_WORD_WRITE,0xC95E, 0x0003} , 	//  CAM_PGA_PGA_CONTROL
//

// [Step4-APGA]
{SENSOR_WORD_WRITE,0x098E, 0x0000} , 	// LOGICAL_ADDRESS_ACCESS
// [Step4-APGA]
{SENSOR_WORD_WRITE,0xC95E, 0x0003} , 	// CAM_PGA_PGA_CONTROL


// [Step5-AWB_CCM]1: LOAD=CCM
{SENSOR_WORD_WRITE,0xC892, 0x0267} , 	// CAM_AWB_CCM_L_0
{SENSOR_WORD_WRITE,0xC894, 0xFF1A} , 	// CAM_AWB_CCM_L_1
{SENSOR_WORD_WRITE,0xC896, 0xFFB3} , 	// CAM_AWB_CCM_L_2
{SENSOR_WORD_WRITE,0xC898, 0xFF80} , 	// CAM_AWB_CCM_L_3
{SENSOR_WORD_WRITE,0xC89A, 0x0166} , 	// CAM_AWB_CCM_L_4
{SENSOR_WORD_WRITE,0xC89C, 0x0003} , 	// CAM_AWB_CCM_L_5
{SENSOR_WORD_WRITE,0xC89E, 0xFF9A} , 	// CAM_AWB_CCM_L_6
{SENSOR_WORD_WRITE,0xC8A0, 0xFEB4} , 	// CAM_AWB_CCM_L_7
{SENSOR_WORD_WRITE,0xC8A2, 0x024D} , 	// CAM_AWB_CCM_L_8
{SENSOR_WORD_WRITE,0xC8A4, 0x01BF} , 	// CAM_AWB_CCM_M_0
{SENSOR_WORD_WRITE,0xC8A6, 0xFF01} , 	// CAM_AWB_CCM_M_1
{SENSOR_WORD_WRITE,0xC8A8, 0xFFF3} , 	// CAM_AWB_CCM_M_2
{SENSOR_WORD_WRITE,0xC8AA, 0xFF75} , 	// CAM_AWB_CCM_M_3
{SENSOR_WORD_WRITE,0xC8AC, 0x0198} , 	// CAM_AWB_CCM_M_4
{SENSOR_WORD_WRITE,0xC8AE, 0xFFFD} , 	// CAM_AWB_CCM_M_5
{SENSOR_WORD_WRITE,0xC8B0, 0xFF9A} , 	// CAM_AWB_CCM_M_6
{SENSOR_WORD_WRITE,0xC8B2, 0xFEE7} , 	// CAM_AWB_CCM_M_7
{SENSOR_WORD_WRITE,0xC8B4, 0x02A8} , 	// CAM_AWB_CCM_M_8
{SENSOR_WORD_WRITE,0xC8B6, 0x01D9} , 	// CAM_AWB_CCM_R_0
{SENSOR_WORD_WRITE,0xC8B8, 0xFF26} , 	// CAM_AWB_CCM_R_1
{SENSOR_WORD_WRITE,0xC8BA, 0xFFF3} , 	// CAM_AWB_CCM_R_2
{SENSOR_WORD_WRITE,0xC8BC, 0xFFB3} , 	// CAM_AWB_CCM_R_3
{SENSOR_WORD_WRITE,0xC8BE, 0x0132} , 	// CAM_AWB_CCM_R_4
{SENSOR_WORD_WRITE,0xC8C0, 0xFFE8} , 	// CAM_AWB_CCM_R_5
{SENSOR_WORD_WRITE,0xC8C2, 0xFFDA} , 	// CAM_AWB_CCM_R_6
{SENSOR_WORD_WRITE,0xC8C4, 0xFECD} , 	// CAM_AWB_CCM_R_7
{SENSOR_WORD_WRITE,0xC8C6, 0x02C2} , 	// CAM_AWB_CCM_R_8
{SENSOR_WORD_WRITE,0xC8C8, 0x0075} , 	// CAM_AWB_CCM_L_RG_GAIN
{SENSOR_WORD_WRITE,0xC8CA, 0x011C} , 	// CAM_AWB_CCM_L_BG_GAIN
{SENSOR_WORD_WRITE,0xC8CC, 0x009A} , 	// CAM_AWB_CCM_M_RG_GAIN
{SENSOR_WORD_WRITE,0xC8CE, 0x0105} , 	// CAM_AWB_CCM_M_BG_GAIN
{SENSOR_WORD_WRITE,0xC8D0, 0x00A4} , 	// CAM_AWB_CCM_R_RG_GAIN
{SENSOR_WORD_WRITE,0xC8D2, 0x00AC} , 	// CAM_AWB_CCM_R_BG_GAIN
{SENSOR_WORD_WRITE,0xC8D4, 0x0A8C} , 	// CAM_AWB_CCM_L_CTEMP
{SENSOR_WORD_WRITE,0xC8D6, 0x0F0A} , 	// CAM_AWB_CCM_M_CTEMP
{SENSOR_WORD_WRITE,0xC8D8, 0x1964} , 	// CAM_AWB_CCM_R_CTEMP

// LOAD=AWB
{SENSOR_WORD_WRITE,0xC914, 0x0000} , 	// CAM_STAT_AWB_CLIP_WINDOW_XSTART
{SENSOR_WORD_WRITE,0xC916, 0x0000} , 	// CAM_STAT_AWB_CLIP_WINDOW_YSTART
{SENSOR_WORD_WRITE,0xC918, 0x04FF} , 	// CAM_STAT_AWB_CLIP_WINDOW_XEND
{SENSOR_WORD_WRITE,0xC91A, 0x02CF} , 	// CAM_STAT_AWB_CLIP_WINDOW_YEND
{SENSOR_WORD_WRITE,0xC904, 0x0033} , 	// CAM_AWB_AWB_XSHIFT_PRE_ADJ
{SENSOR_WORD_WRITE,0xC906, 0x0040} , 	// CAM_AWB_AWB_YSHIFT_PRE_ADJ
{SENSOR_BYTE_WRITE,0xC8F2, 0x03} , 	// CAM_AWB_AWB_XSCALE
{SENSOR_BYTE_WRITE,0xC8F3, 0x02} , 	// CAM_AWB_AWB_YSCALE
{SENSOR_WORD_WRITE,0xC906, 0x003C} , 	// CAM_AWB_AWB_YSHIFT_PRE_ADJ
{SENSOR_WORD_WRITE,0xC8F4, 0x0000} , 	// CAM_AWB_AWB_WEIGHTS_0
{SENSOR_WORD_WRITE,0xC8F6, 0x0000} , 	// CAM_AWB_AWB_WEIGHTS_1
{SENSOR_WORD_WRITE,0xC8F8, 0x0000} , 	// CAM_AWB_AWB_WEIGHTS_2
{SENSOR_WORD_WRITE,0xC8FA, 0xE724} , 	// CAM_AWB_AWB_WEIGHTS_3
{SENSOR_WORD_WRITE,0xC8FC, 0x1583} , 	// CAM_AWB_AWB_WEIGHTS_4
{SENSOR_WORD_WRITE,0xC8FE, 0x2045} , 	// CAM_AWB_AWB_WEIGHTS_5
{SENSOR_WORD_WRITE,0xC900, 0x05DC} , 	// CAM_AWB_AWB_WEIGHTS_6
{SENSOR_WORD_WRITE,0xC902, 0x007C} , 	// CAM_AWB_AWB_WEIGHTS_7
{SENSOR_BYTE_WRITE,0xC90C, 0x80} , 	// CAM_AWB_K_R_L
{SENSOR_BYTE_WRITE,0xC90D, 0x80} , 	// CAM_AWB_K_G_L
{SENSOR_BYTE_WRITE,0xC90E, 0x80} , 	// CAM_AWB_K_B_L
{SENSOR_BYTE_WRITE,0xC90F, 0x88} , 	// CAM_AWB_K_R_R
{SENSOR_BYTE_WRITE,0xC910, 0x80} , 	// CAM_AWB_K_G_R
{SENSOR_BYTE_WRITE,0xC911, 0x80} , 	// CAM_AWB_K_B_R

// LOAD=Step7-CPIPE_Preference
//{SENSOR_WORD_WRITE,0xC926, 0x0020} , 	// CAM_LL_START_BRIGHTNESS
// -- Rev3, 03082011, Alias
// changed for fixing AE test at LV6
{SENSOR_WORD_WRITE,0xC926, 0x0060} , 	// CAM_LL_START_BRIGHTNESS
{SENSOR_WORD_WRITE,0xC928, 0x009A} , 	// CAM_LL_STOP_BRIGHTNESS
{SENSOR_WORD_WRITE,0xC946, 0x0070} , 	// CAM_LL_START_GAIN_METRIC
{SENSOR_WORD_WRITE,0xC948, 0x00F3} , 	// CAM_LL_STOP_GAIN_METRIC
//{SENSOR_WORD_WRITE,0xC952, 0x0020} , 	// CAM_LL_START_TARGET_LUMA_BM
// -- Rev3, 03082011, Alias
// changed for fixing AE test at LV6
{SENSOR_WORD_WRITE,0xC952, 0x0060} , 	// CAM_LL_START_TARGET_LUMA_BM
{SENSOR_WORD_WRITE,0xC954, 0x009A} , 	// CAM_LL_STOP_TARGET_LUMA_BM
{SENSOR_BYTE_WRITE,0xC92A, 0x80} , 	// CAM_LL_START_SATURATION
{SENSOR_BYTE_WRITE,0xC92B, 0x4B} , 	// CAM_LL_END_SATURATION
{SENSOR_BYTE_WRITE,0xC92C, 0x00} , 	// CAM_LL_START_DESATURATION
{SENSOR_BYTE_WRITE,0xC92D, 0xFF} , 	// CAM_LL_END_DESATURATION
{SENSOR_BYTE_WRITE,0xC92E, 0x3C} , 	// CAM_LL_START_DEMOSAIC
{SENSOR_BYTE_WRITE,0xC92F, 0x02} , 	// CAM_LL_START_AP_GAIN
{SENSOR_BYTE_WRITE,0xC930, 0x06} , 	// CAM_LL_START_AP_THRESH
{SENSOR_BYTE_WRITE,0xC931, 0x64} , 	// CAM_LL_STOP_DEMOSAIC
{SENSOR_BYTE_WRITE,0xC932, 0x01} , 	// CAM_LL_STOP_AP_GAIN
{SENSOR_BYTE_WRITE,0xC933, 0x0C} , 	// CAM_LL_STOP_AP_THRESH
{SENSOR_BYTE_WRITE,0xC934, 0x3C} , 	// CAM_LL_START_NR_RED
{SENSOR_BYTE_WRITE,0xC935, 0x3C} , 	// CAM_LL_START_NR_GREEN
{SENSOR_BYTE_WRITE,0xC936, 0x3C} , 	// CAM_LL_START_NR_BLUE
{SENSOR_BYTE_WRITE,0xC937, 0x0F} , 	// CAM_LL_START_NR_THRESH
{SENSOR_BYTE_WRITE,0xC938, 0x64} , 	// CAM_LL_STOP_NR_RED
{SENSOR_BYTE_WRITE,0xC939, 0x64} , 	// CAM_LL_STOP_NR_GREEN
{SENSOR_BYTE_WRITE,0xC93A, 0x64} , 	// CAM_LL_STOP_NR_BLUE
{SENSOR_BYTE_WRITE,0xC93B, 0x32} , 	// CAM_LL_STOP_NR_THRESH
// Rev3, 03082011, Alias
// changed for fixing AE test at LV6
//{SENSOR_WORD_WRITE,0xC93C, 0x0020} , 	// CAM_LL_START_CONTRAST_BM
{SENSOR_WORD_WRITE,0xC93C, 0x0060} , 	// CAM_LL_START_CONTRAST_BM
{SENSOR_WORD_WRITE,0xC93E, 0x009A} , 	// CAM_LL_STOP_CONTRAST_BM
{SENSOR_WORD_WRITE,0xC940, 0x00DC} , 	// CAM_LL_GAMMA
{SENSOR_BYTE_WRITE,0xC942, 0x38} , 	// CAM_LL_START_CONTRAST_GRADIENT
{SENSOR_BYTE_WRITE,0xC943, 0x30} , 	// CAM_LL_STOP_CONTRAST_GRADIENT
{SENSOR_BYTE_WRITE,0xC944, 0x50} , 	// CAM_LL_START_CONTRAST_LUMA_PERCENTAGE
{SENSOR_BYTE_WRITE,0xC945, 0x19} , 	// CAM_LL_STOP_CONTRAST_LUMA_PERCENTAGE
//
// Rev3, 03082011, Alias
// changed for fixing FTB
//{SENSOR_WORD_WRITE,0xC94A, 0x0230} , 	// CAM_LL_START_FADE_TO_BLACK_LUMA
{SENSOR_WORD_WRITE,0xC94A, 0x00F0} , 	// CAM_LL_START_FADE_TO_BLACK_LUMA
{SENSOR_WORD_WRITE,0xC94C, 0x0010} , 	// CAM_LL_STOP_FADE_TO_BLACK_LUMA
{SENSOR_WORD_WRITE,0xC94E, 0x01CD} , 	// CAM_LL_CLUSTER_DC_TH_BM
{SENSOR_BYTE_WRITE,0xC950, 0x05} , 	// CAM_LL_CLUSTER_DC_GATE_PERCENTAGE
{SENSOR_BYTE_WRITE,0xC951, 0x40} , 	// CAM_LL_SUMMING_SENSITIVITY_FACTOR
// Rev3, 03082011, Alias
// set AE target to default to 128
//{SENSOR_BYTE_WRITE,0xC87B, 0x1B} , 	// CAM_AET_TARGET_AVERAGE_LUMA_DARK
{SENSOR_BYTE_WRITE,0xC87A, 0x42} , 	// CAM_AET_TARGET_AVERAGE_LUMA
{SENSOR_BYTE_WRITE,0xC87B, 0x21} , 	// CAM_AET_TARGET_AVERAGE_LUMA_DARK
{SENSOR_BYTE_WRITE,0xC878, 0x0E} , 	// CAM_AET_AEMODE
{SENSOR_WORD_WRITE,0xC890, 0x0080} , 	// CAM_AET_TARGET_GAIN
{SENSOR_WORD_WRITE,0xC886, 0x0100} , 	// CAM_AET_AE_MAX_VIRT_AGAIN
// Rev3, 03082011, Alias
// modified for 2'nd BL
//{SENSOR_WORD_WRITE,0xC87C, 0x005A} , 	// CAM_AET_BLACK_CLIPPING_TARGET
{SENSOR_WORD_WRITE,0xC87C, 0x0010} , 	// CAM_AET_BLACK_CLIPPING_TARGET
{SENSOR_BYTE_WRITE,0xB42A, 0x05} , 	// CCM_DELTA_GAIN
{SENSOR_BYTE_WRITE,0xA80A, 0x20} , 	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
// LOAD=Step8-Features
{SENSOR_WORD_WRITE,0x098E, 0x0000} , 	// LOGICAL_ADDRESS_ACCESS
//{SENSOR_WORD_WRITE,0xC984, 0x8040} , 	// CAM_PORT_OUTPUT_CONTROL
{SENSOR_WORD_WRITE,0xC984, 0x8041} , 	// CAM_PORT_OUTPUT_CONTROL
{SENSOR_WORD_WRITE,0x001E, 0x0777} , 	// PAD_SLEW
//+
{SENSOR_WORD_WRITE,0x098E, 0xDC00} , 	// LOGICAL_ADDRESS_ACCESS [SYSMGR_NEXT_STATE]
{SENSOR_BYTE_WRITE,0xDC00, 0x28} , 	// SYSMGR_NEXT_STAGE
{SENSOR_WAIT_MS, 0,50} , //WK+
{SENSOR_WORD_WRITE,0x0080, 0x8002} , // COMMAND_REGISTER
//-
// -- Rev3, 03082011, Alias
// Removed below settings
#if 0
// LOAD=MIPI setting for SOC1040
{SENSOR_WORD_WRITE,0xC984, 0x8041} , 	// CAM_PORT_OUTPUT_CONTROL
{SENSOR_WORD_WRITE,0xC988, 0x0F00} , 	// CAM_PORT_MIPI_TIMING_T_HS_ZERO
{SENSOR_WORD_WRITE,0xC98A, 0x0B07} , 	// CAM_PORT_MIPI_TIMING_T_HS_EXIT_HS_TRAIL
{SENSOR_WORD_WRITE,0xC98C, 0x0D01} , 	// CAM_PORT_MIPI_TIMING_T_CLK_POST_CLK_PRE
{SENSOR_WORD_WRITE,0xC98E, 0x071D} , 	// CAM_PORT_MIPI_TIMING_T_CLK_TRAIL_CLK_ZERO
{SENSOR_WORD_WRITE,0xC990, 0x0006} , 	// CAM_PORT_MIPI_TIMING_T_LPX
{SENSOR_WORD_WRITE,0xC992, 0x0A0C} , 	// CAM_PORT_MIPI_TIMING_INIT_TIMING
{SENSOR_WORD_WRITE,0x3C5A, 0x0009} , 	// MIPI_DELAY_TRIM

// LOAD=Dual Lane Mipi Receiver Init
// LOAD=Enter Suspend
{SENSOR_BYTE_WRITE,0xDC00, 0x40} , 	// SYSMGR_NEXT_STATE
{SENSOR_WORD_WRITE,0x0080, 0x8002} , 	// COMMAND_REGISTER
//WK:..... POLL_FIELD=COMMAND_REGISTER, HOST_COMMAND_1, !=0, DELAY=10, TIMEOUT=100
{SENSOR_WAIT_MS, 0,100} , //WK+


//LOAD=Leave Suspend
{SENSOR_BYTE_WRITE,0xDC00, 0x34} , 	// SYSMGR_NEXT_STATE
{SENSOR_WORD_WRITE,0x0080, 0x8002} , 	// COMMAND_REGISTER
//WK:..... POLL_FIELD=COMMAND_REGISTER, HOST_COMMAND_1, !=0, DELAY=10, TIMEOUT=100
{SENSOR_WAIT_MS, 0,100} , //WK+

{SENSOR_WORD_WRITE,0x098E, 0x2802} , 	// LOGICAL_ADDRESS_ACCESS
{SENSOR_WORD_WRITE,0xA802, 0x0008} , 	// AE_TRACK_MODE
{SENSOR_BYTE_WRITE,0xC908, 0x01} , 	// CAM_AWB_SKIP_FRAMES
{SENSOR_BYTE_WRITE,0xC879, 0x01} , 	// CAM_AET_SKIP_FRAMES
{SENSOR_BYTE_WRITE,0xC909, 0x02} , 	// CAM_AWB_AWBMODE
{SENSOR_BYTE_WRITE,0xA80A, 0x18} , 	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
{SENSOR_BYTE_WRITE,0xA80B, 0x18} , 	// AE_TRACK_AE_DAMPENING_SPEED
{SENSOR_BYTE_WRITE,0xAC16, 0x18} , 	// AWB_PRE_AWB_RATIOS_TRACKING_SPEED
{SENSOR_BYTE_WRITE,0xC878, 0x0E} , 	// CAM_AET_AEMODE
{SENSOR_BYTE_WRITE,0xDC00, 0x28} , 	// SYSMGR_NEXT_STATE
{SENSOR_WAIT_MS, 0,50} , //delay=50
{SENSOR_WORD_WRITE,0x0080, 0x8002} , 	// COMMAND_REGISTER
//WK:......POLL_FIELD=COMMAND_REGISTER, HOST_COMMAND_1, !=0, DELAY=10, TIMEOUT=100
#endif
{SEQUENCE_END, 0x0000}
};

static struct sensor_reg_ex mode_1280x720_2[] =
{
//Seq comes from NV tuning for MI1040
// [720p 30fps Initialization]
// An optional preset that prompts the user for initialization decisions.
// For example, if their physical configuration uses MIPI.

// Load Steps in this order to ensure correct SOC bring-up
// LOAD = Step1-Reset 			//Reset

{SENSOR_WORD_WRITE, 0x001A, 0x0001},
{SEQUENCE_WAIT_MS, 0, 10}, 	//delay=10
{SENSOR_WORD_WRITE, 0x001A, 0x0000},
{SEQUENCE_WAIT_MS, 0, 50}, 	//delay=50
{SENSOR_WORD_WRITE, 0x301A, 0x0234},   		// MASK BAD FRAME
				//LOAD = Step2-{SENSOR_WORD_WRITE, 0x098E, 0x0000}, 		// set XDMA to logical addressing
{SENSOR_WORD_WRITE, 0xC97E, 0x01ba},		//cam_sysctl_pll_enable = 1
{SENSOR_WORD_WRITE, 0xC980, 0x0120},		//cam_sysctl_pll_divider_m_n = 288
{SENSOR_WORD_WRITE, 0xC982, 0x0700},		//cam_sysctl_pll_divider_p = 1792
{SEQUENCE_WAIT_MS, 0, 100}, 	//delay=50
{SENSOR_WORD_WRITE, 0xC988, 0x0F00},		//cam_port_mipi_timing_t_hs_zero = 3840
{SENSOR_WORD_WRITE, 0xC98A, 0x0B07},		//cam_port_mipi_timing_t_hs_exit_hs_trail = 2823
{SENSOR_WORD_WRITE, 0xC98C, 0x0D01},		//cam_port_mipi_timing_t_clk_post_clk_pre = 3329
{SENSOR_WORD_WRITE, 0xC98E, 0x071D},		//cam_port_mipi_timing_t_clk_trail_clk_zero = 1821
{SENSOR_WORD_WRITE, 0xC990, 0x0006},		//cam_port_mipi_timing_t_lpx = 6
{SENSOR_WORD_WRITE, 0xC992, 0x0A0C},		//cam_port_mipi_timing_init_timing = 2572
{SENSOR_WORD_WRITE, 0xC800, 0x007C},		//cam_sensor_cfg_y_addr_start = 124
{SENSOR_WORD_WRITE, 0xC802, 0x0004},		//cam_sensor_cfg_x_addr_start = 4
{SENSOR_WORD_WRITE, 0xC804, 0x0353},		//cam_sensor_cfg_y_addr_end = 851
{SENSOR_WORD_WRITE, 0xC806, 0x050B},		//cam_sensor_cfg_x_addr_end = 1291
{SENSOR_WORD_WRITE, 0xC808, 0x02DC},
{SENSOR_WORD_WRITE, 0xC80A, 0x6C00},		//cam_sensor_cfg_pixclk = 48000000
{SENSOR_WORD_WRITE, 0xC80C, 0x0001},		//cam_sensor_cfg_row_speed = 1
{SENSOR_WORD_WRITE, 0xC80E, 0x00DB},		//cam_sensor_cfg_fine_integ_time_min = 219
{SENSOR_WORD_WRITE, 0xC810, 0x05BD},		//cam_sensor_cfg_fine_integ_time_max = 1469
{SENSOR_WORD_WRITE, 0xC812, 0x03E8},		//cam_sensor_cfg_frame_length_lines = 1000
{SENSOR_WORD_WRITE, 0xC814, 0x0640},		//cam_sensor_cfg_line_length_pck = 1600
{SENSOR_WORD_WRITE, 0xC816, 0x0060},		//cam_sensor_cfg_fine_correction = 96
{SENSOR_WORD_WRITE, 0xC818, 0x02D3},		//cam_sensor_cfg_cpipe_last_row = 723
{SENSOR_WORD_WRITE, 0xC826, 0x0020},		//cam_sensor_cfg_reg_0_data = 32
{SENSOR_WORD_WRITE, 0xC834, 0x0000},		//cam_sensor_control_read_mode = 0
{SENSOR_WORD_WRITE, 0xC854, 0x0000},		//cam_crop_window_xoffset = 0
{SENSOR_WORD_WRITE, 0xC856, 0x0000},		//cam_crop_window_yoffset = 0
{SENSOR_WORD_WRITE, 0xC858, 0x0500},		//cam_crop_window_width = 1280
{SENSOR_WORD_WRITE, 0xC85A, 0x02D0},		//cam_crop_window_height = 720
{SENSOR_WORD_WRITE, 0xC85C, 0x0342},		//cam_crop_cropmode = 3
{SENSOR_WORD_WRITE, 0xC868, 0x0500},		//cam_output_width = 1280
{SENSOR_WORD_WRITE, 0xC86A, 0x02D0},		//cam_output_height = 720
{SEQUENCE_WAIT_MS, 0, 100}, 	//delay=50


{SENSOR_WORD_WRITE, 0xC878, 0x0f01},		//cam_aet_aemode = 0
{SENSOR_WORD_WRITE, 0xC88C, 0x1E00},		//cam_aet_max_frame_rate = 7680
{SENSOR_WORD_WRITE, 0xC88E, 0x1E00},		//cam_aet_min_frame_rate = 7680
{SENSOR_WORD_WRITE, 0xC914, 0x0000},		//cam_stat_awb_clip_window_xstart = 0
{SENSOR_WORD_WRITE, 0xC916, 0x0000},		//cam_stat_awb_clip_window_ystart = 0
{SENSOR_WORD_WRITE, 0xC918, 0x04FF},		//cam_stat_awb_clip_window_xend = 1279
{SENSOR_WORD_WRITE, 0xC91A, 0x02CF},		//cam_stat_awb_clip_window_yend = 719
{SENSOR_WORD_WRITE, 0xC91C, 0x0000},		//cam_stat_ae_initial_window_xstart = 0
{SENSOR_WORD_WRITE, 0xC91E, 0x0000},		//cam_stat_ae_initial_window_ystart = 0
{SENSOR_WORD_WRITE, 0xC920, 0x00FF},		//cam_stat_ae_initial_window_xend = 255
{SENSOR_WORD_WRITE, 0xC922, 0x008F},		//cam_stat_ae_initial_window_yend = 143
{SENSOR_WORD_WRITE, 0xE800, 0x0000}, 	// AUTO_BINNING_MODE
			//LOAD = Step3-Recommended		//Patch,Errata and Sensor optimization Setting
{SENSOR_WORD_WRITE, 0x316A, 0x8270}, 	// DAC_TXLO_ROW
{SENSOR_WORD_WRITE, 0x316C, 0x8270}, 	// DAC_TXLO
{SENSOR_WORD_WRITE, 0x3ED0, 0x3605}, 	// DAC_LD_4_5
{SENSOR_WORD_WRITE, 0x3ED2, 0x77FF}, 	// DAC_LD_6_7
{SENSOR_WORD_WRITE, 0x316E, 0xC233}, 	// DAC_ECL
{SENSOR_WORD_WRITE, 0x3180, 0x87FF}, 	// DELTA_DK_CONTROL
{SENSOR_WORD_WRITE, 0x30D4, 0x6080}, 	// COLUMN_CORRECTION
{SENSOR_WORD_WRITE, 0x098E, 0x2802}, 	// LOGICAL_ADDRESS_ACCESS [AE_TRACK_MODE]
{SENSOR_WORD_WRITE, 0xA802, 0x0008}, 	// AE_TRACK_MODE
{SENSOR_WORD_WRITE, 0x3E14, 0xFF39}, 	// SAMP_COL_PUP2
{SENSOR_WORD_WRITE, 0x301A, 0x8234}, 	// RESET_REGISTER
			//LOAD = Step4-APGA			//APGA  todo: Johnny to calibrate lens shading
			//LOAD = Step5-AWB_CCM			//AWB & CCM

{SENSOR_WORD_WRITE, 0x098E, 0x0000}, 	// LOGICAL_ADDRESS_ACCESS
{SENSOR_WORD_WRITE, 0xC984, 0x8041}, 	// CAM_PORT_OUTPUT_CONTROL
{SENSOR_WORD_WRITE, 0x001E, 0x0777}, 	// PAD_SLEW

{SENSOR_WORD_WRITE, 0x098E, 0xDC00}, 	// LOGICAL_ADDRESS_ACCESS [SYSMGR_NEXT_STATE]
{SENSOR_WORD_WRITE, 0xDC00, 0x2800}, 	// SYSMGR_NEXT_STATE
{SEQUENCE_WAIT_MS, 0, 50}, //delay=50
{SENSOR_WORD_WRITE, 0x0080, 0x8002}, 	// COMMAND_REGISTER
{SEQUENCE_END, 0x0000}
};

static struct sensor_reg mode_1280x720[] =
{
//Seq comes from NV tuning for MI1040
// [720p 30fps Initialization]
// An optional preset that prompts the user for initialization decisions.
// For example, if their physical configuration uses MIPI.

// Load Steps in this order to ensure correct SOC bring-up
// LOAD = Step1-Reset 			//Reset

{0x001A, 0x0001},
{SEQUENCE_WAIT_MS, 10}, 	//delay=10
{0x001A, 0x0000},
{SEQUENCE_WAIT_MS, 50}, 	//delay=50
{0x301A, 0x0234},   		// MASK BAD FRAME
				//LOAD = Step2-{0x098E, 0x0000}, 		// set XDMA to logical addressing
{0xC97E, 0x01ba},		//cam_sysctl_pll_enable = 1
{0xC980, 0x0120},		//cam_sysctl_pll_divider_m_n = 288
{0xC982, 0x0700},		//cam_sysctl_pll_divider_p = 1792
{SEQUENCE_WAIT_MS, 100}, 	//delay=50
{0xC988, 0x0F00},		//cam_port_mipi_timing_t_hs_zero = 3840
{0xC98A, 0x0B07},		//cam_port_mipi_timing_t_hs_exit_hs_trail = 2823
{0xC98C, 0x0D01},		//cam_port_mipi_timing_t_clk_post_clk_pre = 3329
{0xC98E, 0x071D},		//cam_port_mipi_timing_t_clk_trail_clk_zero = 1821
{0xC990, 0x0006},		//cam_port_mipi_timing_t_lpx = 6
{0xC992, 0x0A0C},		//cam_port_mipi_timing_init_timing = 2572
{0xC800, 0x007C},		//cam_sensor_cfg_y_addr_start = 124
{0xC802, 0x0004},		//cam_sensor_cfg_x_addr_start = 4
{0xC804, 0x0353},		//cam_sensor_cfg_y_addr_end = 851
{0xC806, 0x050B},		//cam_sensor_cfg_x_addr_end = 1291
{0xC808, 0x02DC},
{0xC80A, 0x6C00},		//cam_sensor_cfg_pixclk = 48000000
{0xC80C, 0x0001},		//cam_sensor_cfg_row_speed = 1
{0xC80E, 0x00DB},		//cam_sensor_cfg_fine_integ_time_min = 219
{0xC810, 0x05BD},		//cam_sensor_cfg_fine_integ_time_max = 1469
{0xC812, 0x03E8},		//cam_sensor_cfg_frame_length_lines = 1000
{0xC814, 0x0640},		//cam_sensor_cfg_line_length_pck = 1600
{0xC816, 0x0060},		//cam_sensor_cfg_fine_correction = 96
{0xC818, 0x02D3},		//cam_sensor_cfg_cpipe_last_row = 723
{0xC826, 0x0020},		//cam_sensor_cfg_reg_0_data = 32
{0xC834, 0x0000},		//cam_sensor_control_read_mode = 0
{0xC854, 0x0000},		//cam_crop_window_xoffset = 0
{0xC856, 0x0000},		//cam_crop_window_yoffset = 0
{0xC858, 0x0500},		//cam_crop_window_width = 1280
{0xC85A, 0x02D0},		//cam_crop_window_height = 720
{0xC85C, 0x0342},		//cam_crop_cropmode = 3
{0xC868, 0x0500},		//cam_output_width = 1280
{0xC86A, 0x02D0},		//cam_output_height = 720
{SEQUENCE_WAIT_MS, 100}, 	//delay=50


{0xC878, 0x0f01},		//cam_aet_aemode = 0
{0xC88C, 0x1E00},		//cam_aet_max_frame_rate = 7680
{0xC88E, 0x1E00},		//cam_aet_min_frame_rate = 7680
{0xC914, 0x0000},		//cam_stat_awb_clip_window_xstart = 0
{0xC916, 0x0000},		//cam_stat_awb_clip_window_ystart = 0
{0xC918, 0x04FF},		//cam_stat_awb_clip_window_xend = 1279
{0xC91A, 0x02CF},		//cam_stat_awb_clip_window_yend = 719
{0xC91C, 0x0000},		//cam_stat_ae_initial_window_xstart = 0
{0xC91E, 0x0000},		//cam_stat_ae_initial_window_ystart = 0
{0xC920, 0x00FF},		//cam_stat_ae_initial_window_xend = 255
{0xC922, 0x008F},		//cam_stat_ae_initial_window_yend = 143
{0xE800, 0x0000}, 	// AUTO_BINNING_MODE
			//LOAD = Step3-Recommended		//Patch,Errata and Sensor optimization Setting
{0x316A, 0x8270}, 	// DAC_TXLO_ROW
{0x316C, 0x8270}, 	// DAC_TXLO
{0x3ED0, 0x3605}, 	// DAC_LD_4_5
{0x3ED2, 0x77FF}, 	// DAC_LD_6_7
{0x316E, 0xC233}, 	// DAC_ECL
{0x3180, 0x87FF}, 	// DELTA_DK_CONTROL
{0x30D4, 0x6080}, 	// COLUMN_CORRECTION
{0x098E, 0x2802}, 	// LOGICAL_ADDRESS_ACCESS [AE_TRACK_MODE]
{0xA802, 0x0008}, 	// AE_TRACK_MODE
{0x3E14, 0xFF39}, 	// SAMP_COL_PUP2
{0x301A, 0x8234}, 	// RESET_REGISTER
			//LOAD = Step4-APGA			//APGA  todo: Johnny to calibrate lens shading
			//LOAD = Step5-AWB_CCM			//AWB & CCM

{0x098E, 0x0000}, 	// LOGICAL_ADDRESS_ACCESS
{0xC984, 0x8041}, 	// CAM_PORT_OUTPUT_CONTROL
{0x001E, 0x0777}, 	// PAD_SLEW

{0x098E, 0xDC00}, 	// LOGICAL_ADDRESS_ACCESS [SYSMGR_NEXT_STATE]
{0xDC00, 0x2800}, 	// SYSMGR_NEXT_STATE
{SEQUENCE_WAIT_MS, 50}, //delay=50
{0x0080, 0x8002}, 	// COMMAND_REGISTER
{SEQUENCE_END, 0x0000}
};



static struct sensor_reg_ex ColorEffect_None[] = {
//[2.1 Normal -- default]
{SENSOR_WORD_WRITE,0x098E, 0xC874} , 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
{SENSOR_BYTE_WRITE,0xC874, 0x00} , 	// CAM_SFX_CONTROL
{SENSOR_BYTE_WRITE,0xDC00, 0x28} , 	// SYSMGR_NEXT_STATE
{SENSOR_WORD_WRITE,0x0080, 0x8004} , 	// COMMAND_REGISTER
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg_ex ColorEffect_Mono[] = {
//[2.2 Mono color effect]
{SENSOR_WORD_WRITE,0x098E, 0xC874} , 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
{SENSOR_BYTE_WRITE,0xC874, 0x01} , 	// CAM_SFX_CONTROL
{SENSOR_BYTE_WRITE,0xDC00, 0x28} , 	// SYSMGR_NEXT_STATE
{SENSOR_WORD_WRITE,0x0080, 0x8004} , 	// COMMAND_REGISTER
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg_ex ColorEffect_Sepia[] = {
//[2.3 Sepia effect]
{SENSOR_WORD_WRITE,0x098E, 0xC874} , 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
{SENSOR_BYTE_WRITE,0xC874, 0x02} , 	// CAM_SFX_CONTROL
{SENSOR_BYTE_WRITE,0xDC00, 0x28} , 	// SYSMGR_NEXT_STATE
{SENSOR_WORD_WRITE,0x0080, 0x8004} , 	// COMMAND_REGISTER
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg_ex ColorEffect_Negative[] = {
//[2.4 Negative effect]
{SENSOR_WORD_WRITE,0x098E, 0xC874} , 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
{SENSOR_BYTE_WRITE,0xC874, 0x03} , 	// CAM_SFX_CONTROL
{SENSOR_BYTE_WRITE,0xDC00, 0x28} , 	// SYSMGR_NEXT_STATE
{SENSOR_WORD_WRITE,0x0080, 0x8004} , 	// COMMAND_REGISTER
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg_ex ColorEffect_Solarize[] = {
//[2.5 Solarize 1]
{SENSOR_WORD_WRITE,0x098E, 0xC874} , 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
{SENSOR_BYTE_WRITE,0xC874, 0x04} , 	// CAM_SFX_CONTROL
{SENSOR_BYTE_WRITE,0xDC00, 0x28} , 	// SYSMGR_NEXT_STATE
{SENSOR_WORD_WRITE,0x0080, 0x8004} , 	// COMMAND_REGISTER
{SENSOR_TABLE_END, 0x0000}
};

//Sensor ISP Not Support this function
static struct sensor_reg_ex ColorEffect_Posterize[] = {
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg_ex Whitebalance_Auto[] = {
//[4.1 AWB -- default]
{SENSOR_WORD_WRITE,0x098E, 0x0000} , 	// LOGICAL_ADDRESS_ACCESS
{SENSOR_BYTE_WRITE,0xC909, 0x02} , 	// CAM_AWB_AWBMODE
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg_ex Whitebalance_Incandescent[] = {
//[4.5 MWB: A Light]
{SENSOR_WORD_WRITE,0x098E, 0x0000} , 	// LOGICAL_ADDRESS_ACCESS
{SENSOR_BYTE_WRITE,0xC909, 0x00} , 	// CAM_AWB_AWBMODE
{SENSOR_WORD_WRITE,0xC8F0, 0x0A8C} , 	// CAM_AWB_COLOR_TEMPERATURE

{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg_ex Whitebalance_Daylight[] = {
//[4.2 MWB: D65]
{SENSOR_WORD_WRITE,0x098E, 0x0000} , 	// LOGICAL_ADDRESS_ACCESS
{SENSOR_BYTE_WRITE,0xC909, 0x00} , 	// CAM_AWB_AWBMODE
{SENSOR_WORD_WRITE,0xC8F0, 0x1964} , 	// CAM_AWB_COLOR_TEMPERATURE
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg_ex Whitebalance_Fluorescent[] = {
//[4.4 MWB: TL84]
{SENSOR_WORD_WRITE,0x098E, 0x0000} , 	// LOGICAL_ADDRESS_ACCESS
{SENSOR_BYTE_WRITE,0xC909, 0x00} , 	// CAM_AWB_AWBMODE
{SENSOR_WORD_WRITE,0xC8F0, 0x0E74} , 	// CAM_AWB_COLOR_TEMPERATURE
{SENSOR_TABLE_END, 0x0000}
};

//+ EV

static struct sensor_reg_ex EV_zero[] = {
//[3.3 EV0: 128 -- default]
{SENSOR_WORD_WRITE,0x098E, 0xC87A} , 	// LOGICAL_ADDRESS_ACCESS [CAM_AET_TARGET_AVERAGE_LUMA]
{SENSOR_BYTE_WRITE,0xC87A, 0x3C} , 	// CAM_AET_TARGET_AVERAGE_LUMA
{SENSOR_BYTE_WRITE,0xC87B, 0x1E} , 	// CAM_AET_TARGET_AVERAGE_LUMA_DARK
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg_ex EV_plus_1[] = {
//[3.4 EV+1: 138]
{SENSOR_WORD_WRITE,0x098E, 0xC87A} , 	// LOGICAL_ADDRESS_ACCESS [CAM_AET_TARGET_AVERAGE_LUMA]
{SENSOR_BYTE_WRITE,0xC87A, 0x42} , 	// CAM_AET_TARGET_AVERAGE_LUMA
{SENSOR_BYTE_WRITE,0xC87B, 0x21} , 	// CAM_AET_TARGET_AVERAGE_LUMA_DARK
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg_ex EV_plus_2[] = {
//[3.5 EV+2: 148]
{SENSOR_WORD_WRITE,0x098E, 0xC87A} , 	// LOGICAL_ADDRESS_ACCESS [CAM_AET_TARGET_AVERAGE_LUMA]
{SENSOR_BYTE_WRITE,0xC87A, 0x48} , 	// CAM_AET_TARGET_AVERAGE_LUMA
{SENSOR_BYTE_WRITE,0xC87B, 0x24} , 	// CAM_AET_TARGET_AVERAGE_LUMA_DARK
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg_ex EV_minus_1[] = {
//[3.2 EV-1: 118]
{SENSOR_WORD_WRITE,0x098E, 0xC87A} , 	// LOGICAL_ADDRESS_ACCESS [CAM_AET_TARGET_AVERAGE_LUMA]
{SENSOR_BYTE_WRITE,0xC87A, 0x36} , 	// CAM_AET_TARGET_AVERAGE_LUMA
{SENSOR_BYTE_WRITE,0xC87B, 0x1B} , 	// CAM_AET_TARGET_AVERAGE_LUMA_DARK
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg_ex EV_minus_2[] = {
//[3.1 EV-2: 108]
{SENSOR_WORD_WRITE,0x098E, 0xC87A} , 	// LOGICAL_ADDRESS_ACCESS [CAM_AET_TARGET_AVERAGE_LUMA]
{SENSOR_BYTE_WRITE,0xC87A, 0x32} , 	// CAM_AET_TARGET_AVERAGE_LUMA
{SENSOR_BYTE_WRITE,0xC87B, 0x19} , 	// CAM_AET_TARGET_AVERAGE_LUMA_DARK
{SENSOR_TABLE_END, 0x0000}
};

//-


static struct sensor_reg Autofocus_Trigger[] = {
//[8.2 AF - Full Scan ON]
//{0x098E, 0x10},
//{0x098F, 0x00}, 	// LOGICAL_ADDRESS_ACCESS [SEQ_STATE_CFG_1_AF]
//{0x8419, 0x05}, 	// SEQ_STATE_CFG_1_AF
//{0x8404, 0x06}, 	// SEQ_CMD
{0x098E, 0x10},
{0x098F, 0x00}, 	// LOGICAL_ADDRESS_ACCESS [AF_PROGRESS]
{0xB006, 0x01}, 	// AF_PROGRESS{SENSOR_TABLE_END, 0x0000}
{SENSOR_TABLE_END, 0x0000}
};


enum {
	SENSOR_MODE_1280x720,
	SENSOR_MODE_1280x960,
};

static struct sensor_reg *mode_table[] = {
	[SENSOR_MODE_1280x720]  = mode_1280x720,
};

static int sensor_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;

	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	memcpy(val, data+2, 1);
	*val=*val&0xff;

	return 0;
}

static int sensor_read_reg_word(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

  swap(*(data+2),*(data+3)); //swap high and low byte to match table format
	memcpy(val, data+2, 2);

	return 0;
}

static int sensor_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		       addr, val);
		pr_err("yuv_sensor : i2c transfer failed, count %x \n",
		       msg.addr);
//		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}
static int sensor_write_reg_word(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
  data[2] = (u8) (val >> 8);
	data[3] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		       addr, val);
		pr_err("yuv_sensor : i2c transfer failed, count %x \n",
		       msg.addr);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}

static int sensor_write_table(struct i2c_client *client,
			      const struct sensor_reg table[])
{
	int err;
	const struct sensor_reg *next;
	u16 val;

	pr_info("yuv %s\n",__func__);
	for (next = table; next->addr != SENSOR_TABLE_END; next++) {
		if (next->addr == SENSOR_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;

		err = sensor_write_reg_word(client, next->addr, val);
		if (err)
			return err;
	}
	return 0;
}

static int sensor_write_table_ex(struct i2c_client *client,
			      const struct sensor_reg_ex table[])
{
	int err;
	const struct sensor_reg_ex *next;
	u16 val;
  int i=0;

	pr_info("yuv %s\n",__func__);

	for (next = table; next->cmd != SENSOR_TABLE_END; next++) {
//    printk("%d: cmd,addr,value= %d, 0x%X, 0x%X\n", i, next->cmd, next->addr, next->val);
		if (next->cmd == SENSOR_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;

    if (next->cmd == SENSOR_BYTE_WRITE)
		  err = sensor_write_reg(client, next->addr, val);
    else if (next->cmd == SENSOR_WORD_WRITE)
		  err = sensor_write_reg_word(client, next->addr, val);
    else if (next->cmd == SENSOR_WORD_READ)
    {
		  err = sensor_read_reg_word(client, next->addr, &val);
      printk("reg[0x%X]=0x%X , err=%d\n",next->addr, val, err);
    }
		if (err)
			return err;
	}
	return 0;
}
static int get_sensor_current_width(struct i2c_client *client, u16 *val)
{
        int err;

        err = sensor_write_reg_word(client, 0x098c, 0x2703);
        if (err)
          return err;

        err = sensor_read_reg(client, 0x0990, val);

        if (err)
          return err;

        return 0;
}

static int sensor_set_mode(struct sensor_info *info, struct sensor_mode *mode)
{
	int sensor_table;
	int err;

	pr_info("%s: xres %u yres %u\n",__func__, mode->xres, mode->yres);

	if (mode->xres == 1280 && mode->yres == 720)
  {
		sensor_table = SENSOR_MODE_1280x720;
//    err = sensor_write_table(info->i2c_client, mode_table[sensor_table]);
    err = sensor_write_table_ex(info->i2c_client, mode_1280x720_2);
  }
	else if (mode->xres == 1280 && mode->yres == 960)
  {
		sensor_table = SENSOR_MODE_1280x960;
    err = sensor_write_table_ex(info->i2c_client, mode_1280x960);
  }
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	if (err)
	  return err;

	info->mode = sensor_table;
	return 0;
}

static long sensor_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct sensor_info *info = file->private_data;
        int err=0;

	pr_info("yuv %s\n",__func__);
	switch (cmd) {
	case SENSOR_IOCTL_SET_MODE:
	{
		struct sensor_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct sensor_mode))) {
			return -EFAULT;
		}

		return sensor_set_mode(info, &mode);
	}
	case SENSOR_IOCTL_GET_STATUS:
	{

		return 0;
	}
    case SENSOR_IOCTL_SET_COLOR_EFFECT:
    {
      u8 coloreffect;

      if (copy_from_user(&coloreffect,(const void __user *)arg,
        sizeof(coloreffect))) {
        return -EFAULT;
      }

      switch(coloreffect)
      {
        case YUV_ColorEffect_None:
          err = sensor_write_table_ex(info->i2c_client, ColorEffect_None);
          break;
        case YUV_ColorEffect_Mono:
          err = sensor_write_table_ex(info->i2c_client, ColorEffect_Mono);
          break;
        case YUV_ColorEffect_Sepia:
          err = sensor_write_table_ex(info->i2c_client, ColorEffect_Sepia);
          break;
        case YUV_ColorEffect_Negative:
          err = sensor_write_table_ex(info->i2c_client, ColorEffect_Negative);
          break;
        case YUV_ColorEffect_Solarize:
          err = sensor_write_table_ex(info->i2c_client, ColorEffect_Solarize);
          break;
        case YUV_ColorEffect_Posterize:
          err = sensor_write_table_ex(info->i2c_client, ColorEffect_Posterize);
          break;
        default:
          break;
      }

      if (err)
      return err;

      return 0;
    }
    case SENSOR_IOCTL_SET_WHITE_BALANCE:
    {
      u8 whitebalance;

      if (copy_from_user(&whitebalance,(const void __user *)arg,
        sizeof(whitebalance))) {
        return -EFAULT;
      }
      printk("0x%X,0x%X\n",SENSOR_CUSTOM_IOCTL_GET_EV,SENSOR_CUSTOM_IOCTL_SET_EV);
      switch(whitebalance)
      {
        case YUV_Whitebalance_Auto:
          printk("=================WB: Auto\n");
          err = sensor_write_table_ex(info->i2c_client, Whitebalance_Auto);
          break;
        case YUV_Whitebalance_Incandescent:
          printk("=================WB: Incandescent\n");
          err = sensor_write_table_ex(info->i2c_client, Whitebalance_Incandescent);
          break;
        case YUV_Whitebalance_Daylight:
          err = sensor_write_table_ex(info->i2c_client, Whitebalance_Daylight);
          printk("=================WB: Daylight\n");
          break;
        case YUV_Whitebalance_Fluorescent:
          printk("=================WB: Fluorescent\n");
          err = sensor_write_table_ex(info->i2c_client, Whitebalance_Fluorescent);
          break;
        default:
          break;
      }

      if (err)
        return err;

      return 0;
    }

    case SENSOR_IOCTL_SET_SCENE_MODE:
    {
            return 0;
    }
    case SENSOR_CUSTOM_IOCTL_SET_EV:
    {
      short ev;

      if (copy_from_user(&ev,(const void __user *)arg, sizeof(short)))
      {
        return -EFAULT;
      }

      printk("SET_EV as %d\n",ev);

      if (ev == -2)
        err = sensor_write_table_ex(info->i2c_client, EV_minus_2);
      else if (ev == -1)
        err = sensor_write_table_ex(info->i2c_client, EV_minus_1);
      else if (ev == 0)
        err = sensor_write_table_ex(info->i2c_client, EV_zero);
      else if (ev == 1)
        err = sensor_write_table_ex(info->i2c_client, EV_plus_1);
      else if (ev == 2)
        err = sensor_write_table_ex(info->i2c_client, EV_plus_2);
      else
        err = -1;

      if (err)
        return err;
      return 0;
    }

    case SENSOR_CUSTOM_IOCTL_GET_EV:
    {
      short ev;
      u16 val;

      sensor_write_reg_word(info->i2c_client, 0x098E, 0xC874);
      err = sensor_read_reg(info->i2c_client, 0xC87A, &val);

      printk("GET_EV: val=0x%X\n",val);

      if (err)
        return err;

      printk("GET_EV: val=0x%X\n",val);

      if (val <= 0x32)
        ev=-2;
      else if (val <= 0x36)
        ev=-1;
      else if (val <= 0x3c)
        ev=0;
      else if (val <= 0x42)
        ev=1;
      else if (val > 0x42)
        ev=2;
      if (copy_to_user((const void __user *)arg, &ev, sizeof(short)))
      {
        return -EFAULT;
      }
      if (err)
        return err;

      return 0;
    }

  	default:
  		return -EINVAL;
	}
	return 0;
}

static int sensor_open(struct inode *inode, struct file *file)
{
	pr_info("yuv %s\n",__func__);
	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	return 0;
}

int mi1040_sensor_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	file->private_data = NULL;
	return 0;
}


static const struct file_operations sensor_fileops = {
	.owner = THIS_MODULE,
	.open = sensor_open,
	.unlocked_ioctl = sensor_ioctl,
	.release = mi1040_sensor_release,
};

static struct miscdevice sensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MI1040_SENSOR_NAME,
	.fops = &sensor_fileops,
};

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("yuv %s\n",__func__);

	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);

	if (!info) {
		pr_err("yuv_sensor : Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&sensor_device);
	if (err) {
		pr_err("yuv_sensor : Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);
	return 0;
}

static int sensor_remove(struct i2c_client *client)
{
	struct sensor_info *info;

	pr_info("yuv %s\n",__func__);
	info = i2c_get_clientdata(client);
	misc_deregister(&sensor_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ MI1040_SENSOR_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = MI1040_SENSOR_NAME,
		.owner = THIS_MODULE,
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static int __init sensor_init(void)
{
	pr_info("yuv %s\n",__func__);
	return i2c_add_driver(&sensor_i2c_driver);
}

static void __exit sensor_exit(void)
{
	pr_info("yuv %s\n",__func__);
	i2c_del_driver(&sensor_i2c_driver);
}

module_init(sensor_init);
module_exit(sensor_exit);

