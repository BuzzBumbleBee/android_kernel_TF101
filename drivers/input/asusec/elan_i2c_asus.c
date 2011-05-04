#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/earlysuspend.h>
#include <linux/freezer.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/slab.h>

#include "elan_i2c_asus.h"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

static int Public_ETP_YMAX_V2;
static int Public_ETP_XMAX_V2;
static int Public_ETP_2FT_YMAX;	

static struct timer_list console_timer;
#define TIMEOUT_VALUE 1;


static int elan_i2c_asus_cmd(struct i2c_client *client,unsigned char *param, int command)
{
  
	u16 asus_ec_cmd;
	int ret;
	int retry = ELAN_RETRY_COUNT;
	int i;
	int retry_data_count;
	u8 i2c_data[16];
	int index;

	ELAN_INFO("command = 0x%x\n",command);
	asus_ec_cmd = (((command & 0x00ff) << 8) | 0xD4);
	ret = 0;
	ret = i2c_smbus_write_word_data(client, 0x64, asus_ec_cmd);
	if (ret < 0) {
		ELAN_ERR("Wirte to device fails status %x\n",ret);
		return ret;
	}
	msleep(CONVERSION_TIME_MS);	
	
	while(retry-- > 0){
		ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, i2c_data);
		if (ret < 0) {
			ELAN_ERR("Fail to read data, status %d\n", ret);
			return ret;
		}
		ASUSEC_I2C_DATA(i2c_data, index);
		if ((i2c_data[1] & ASUSEC_OBF_MASK) && 
			(i2c_data[1] & ASUSEC_AUX_MASK)){ 
			if (i2c_data[2] == PSMOUSE_RET_ACK){
				break;
			}
			else if (i2c_data[2] == PSMOUSE_RET_NAK){
				goto fail_elan_touchpad_i2c;
			}
		}		
		msleep(CONVERSION_TIME_MS/5);
	}
	
	retry_data_count = (command & 0x0f00) >> 8;
	for(i=1; i <= retry_data_count; i++){
		param[i-1] = i2c_data[i+2];	  
	}
	  
	return 0;
	
fail_elan_touchpad_i2c:	
	ELAN_ERR("fail to get touchpad response");
	return -1;
  
}




/*
 * Interpret complete data packets and report absolute mode input events for
 * hardware version 2. (6 byte packets)
 */

static int edge_detection(struct elantech_data *etd,int x,int y,int finger)
{
	 static int disable_edge = 0;
	 
	 edge_type edge = 0;
	 if ((etd->touch_on_x < (Public_ETP_XMAX_V2 * 9/10)) && (etd->touch_on_y > (Public_ETP_YMAX_V2 * 2/10)))
	   return edge;

	 if(disable_edge == 0){
		if( x > (Public_ETP_XMAX_V2 * 9/10))
			edge |= RIGHT_EDGE;
		if (y < (Public_ETP_YMAX_V2 * 2/10))
			edge |= BOTTOM_EDGE;
	 }

	 
	 if(edge == 0)
		disable_edge=1;
	 if(finger == 0)
		disable_edge=0;
	
	 return edge;
  
}


static void HandleTapProcessing(struct elantech_data *etd, edge_type edge,int finger,int finger_flag,unsigned int x, unsigned int y, int dx, int dy)
{
	static struct timespec time_now, touch_time ,leave_time;
	struct timespec sub_time;
	int delta_x,delta_y;
	int tap_move = 20;
	static int move_flag = 0;

	if (edge != 0 || finger > 1)
		return;

	if(finger == 0 && finger_flag == 1){
		move_flag = 0;
		getnstimeofday(&leave_time);
		if (etd->tap_num == TP_TIME_OUT || etd->tap_num == FINGER_MOVE)
			etd->tap_num = NO_FINGER_ON_TOUCHPAD; 
	}

	getnstimeofday(&time_now);

	if (finger==1 && finger_flag ==0 ){
		getnstimeofday(&touch_time);
		if (etd->tap_num == NO_FINGER_ON_TOUCHPAD){
			etd->tap_num = FIRST_TIME_FINGER_ON_TOUCH;
		}
	}
	if (etd->tap_num == FIRST_TIME_FINGER_ON_TOUCH){
		sub_time = timespec_sub(time_now,touch_time);
		if (etd->touch_on_x >= x)
			delta_x = etd->touch_on_x - x;
		else
			delta_x = x - etd->touch_on_x;

		if (etd->touch_on_y >= y)
			delta_y = etd->touch_on_y - y;
		else
			delta_y = y - etd->touch_on_y;

		if ((delta_x > (tap_move / 4)) || (delta_y > (tap_move / 4)) )
			move_flag = 1;
			
		if (sub_time.tv_sec <= 0 && move_flag == 0 && finger == 0 && finger_flag == 1)
			etd->tap_num = TP_CLICK;
		else if  (sub_time.tv_sec >= 1 &&  finger == 1 )
			etd->tap_num = TP_TIME_OUT;
		else if  (move_flag == 1 && finger == 1)
			etd->tap_num = FINGER_MOVE;
		
	}else if (etd->tap_num == TP_CLICK && finger == 1) {
		move_flag = 0;
		if (etd->touch_on_x >= etd->release_x)
			delta_x = etd->touch_on_x - etd->release_x;
		else
			delta_x = etd->release_x - etd->touch_on_x;

		if (etd->touch_on_y >= etd->release_y)
			delta_y = etd->touch_on_y - etd->release_y;
		else
			delta_y = etd->release_y - etd->touch_on_y;

		if (delta_x > (tap_move*2) || delta_y > (tap_move*2))
			move_flag = 1;
	
		if (finger == 1 && finger_flag == 0){
			if (move_flag == 0)
				etd->tap_num = TP_DRAG;
			else
				etd->tap_num = FIRST_TIME_FINGER_ON_TOUCH;
		}

	}else if (etd->tap_num == TP_DRAG && finger == 1) {
		sub_time = timespec_sub(time_now,touch_time);
		move_flag = 0;
		if (etd->touch_on_x >= x)
			delta_x = etd->touch_on_x - x;
		else
			delta_x = x - etd->touch_on_x;

		if (etd->touch_on_y >= y)
			delta_y = etd->touch_on_y - y;
		else
			delta_y = y - etd->touch_on_y;

		if (delta_x > tap_move || delta_y > tap_move)
			move_flag = 1;
			
		if (sub_time.tv_sec >= 1 || move_flag == 1)
			etd->tap_num = TP_DRAG_MOVE;

	}else if (etd->tap_num == TP_DRAG && finger == 0){
			etd->tap_num = TP_DOUBLE_CLICK;
	}
			
			 
}


static void timertapprocessing(unsigned long data)
{
	struct asusec_chip *ec_chip = (struct asusec_chip *)data;
	struct elantech_data *etd = ec_chip->private;
	struct input_dev *dev = etd->abs_dev;
		 
	if (etd->tap_num == TP_CLICK ){
		input_report_key(dev, BTN_LEFT,1);
		input_sync(dev);
		input_report_key(dev, BTN_LEFT,0);
		input_sync(dev);
		etd->tap_num=NO_FINGER_ON_TOUCHPAD;
	}else if (etd->tap_num == TP_DOUBLE_CLICK){
		input_report_key(dev, BTN_LEFT,1);
		input_sync(dev);
		input_report_key(dev, BTN_LEFT,0);
		input_sync(dev);
		input_report_key(dev, BTN_LEFT,1);
		input_sync(dev);
		input_report_key(dev, BTN_LEFT,0);
		input_sync(dev);
		etd->tap_num=NO_FINGER_ON_TOUCHPAD;
	}
		
}

static void HandleScrolling(edge_type edge,unsigned int x,unsigned int y,int finger,int *wheel,int *hwheel)
{
	  static unsigned int last_x,last_y;
	  unsigned int sub_x,sub_y;
	  static int frist_scroll=1;
  
	  if (edge && frist_scroll){
	      last_x = x;
	      last_y = y;
	      frist_scroll=0;
	  }
	  if (!finger)
	      frist_scroll=1;
	  
	  if ((edge & 0x08) == 0x08){
	      if (last_y > y){
			sub_y = last_y - y;
	      }else if (last_y < y){
			sub_y = y - last_y;
	      }
	      if (sub_y > 5){			 
			 *wheel = sub_y/5;
			 if (last_y > y){
				*wheel =  0 - *wheel;
			 }else if (last_y < y){
				*wheel =  *wheel; 
			 }
			 last_x = x;
			 last_y = y;
	      }
	  }else if ((edge & 0x01) == 0x01){
		if (last_x > x){
			sub_x = last_x - x;
		}else if (last_x < x){
			sub_x = x - last_x;
	      }
	      if (sub_x > 5){			
			 *hwheel = sub_x/5;
			 if (last_x > x){
				*hwheel = 0 - *hwheel;
			 }else if (last_x < x){
				*hwheel = *hwheel; 
			 }
			 last_x = x;
			 last_y = y;
	      }

	  }

}

static void HandleTwoFingerZoom(int finger,int x2_1,int y2_1,int x2_2,int y2_2,struct asusec_chip *ec_chip)
{
	struct elantech_data *etd = ec_chip->private;
	struct input_dev *dev = etd->abs_dev;
	int finger2_pressed;
	int x_mid, y_mid;
	finger2_pressed = finger==2;

	input_report_key(dev, BTN_2, finger2_pressed);
	if(finger==2){
		
	/*
		input_report_abs(dev, ABS_X, x2_1);
		input_report_abs(dev, ABS_Y, y2_1);
		input_report_abs(dev, ABS_HAT0X, x2_2);
		input_report_abs(dev, ABS_HAT0Y, y2_2);
		input_report_abs(dev, BTN_TOUCH, finger);
		input_report_abs(dev, ABS_PRESSURE, 100);
	*/	
		if (x2_1 && y2_1 && x2_2 && y2_2){
			x_mid = (x2_1 + x2_2)/2;
			y_mid = (y2_1 + y2_2)/2;
			
			input_report_abs(dev, ABS_MT_TRACKING_ID, 1);
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR, 100);
			input_report_abs(dev, ABS_MT_WIDTH_MAJOR, 7);
			input_report_abs(dev, ABS_MT_POSITION_X, x_mid);
			input_report_abs(dev, ABS_MT_POSITION_Y, y_mid);
			input_mt_sync(dev);
		
			/*
			input_report_abs(dev, ABS_MT_TRACKING_ID, 1);
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR, 100);
			input_report_abs(dev, ABS_MT_WIDTH_MAJOR, 7);
			input_report_abs(dev, ABS_MT_POSITION_X, x2_1);
			input_report_abs(dev, ABS_MT_POSITION_Y, y2_1);
			input_mt_sync(dev);
		
			input_report_abs(dev, ABS_MT_TRACKING_ID, 2);
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR, 100);
			input_report_abs(dev, ABS_MT_WIDTH_MAJOR, 7);
			input_report_abs(dev, ABS_MT_POSITION_X, x2_2);
			input_report_abs(dev, ABS_MT_POSITION_Y, y2_2);
			input_mt_sync(dev);
			*/
		}
		ELAN_INFO("x2_1=%d, y2_1=%d x2_2=%d, y2_2=%d \n",x2_1, y2_1, x2_2, y2_2);
		
	}else{
		input_report_abs(dev, BTN_TOUCH, 0);	
		input_report_abs(dev, ABS_PRESSURE, 0);
		input_report_abs(dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(dev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(dev);
		
	}
	input_sync(dev);
}
static void ComputeDelta(int *dx,int *dy,edge_type edge,int x,int y,int last_x ,int last_y,int finger,int finger_flag)
{
	static int temp_last_x,temp_last_y;
	int temp_dx,temp_dy;
	int temp_odd,radical;
	unsigned int speed_squared;
	static int move_frist_data=0;
	static int remainder_x,remainder_y;

	if (edge != 0 || finger != 1){
	     move_frist_data=0;
	     return;
	}
	
	if (finger_flag == 1){
		if(move_frist_data == 0){
			remainder_x=0;
			remainder_y=0;
			temp_last_x = last_x;
			temp_last_y = last_y;
			move_frist_data = 1;
		}
	}else 
		return;
	
	temp_dx = x - temp_last_x ;
	temp_dy = y - temp_last_y;
	speed_squared = (temp_dx * temp_dx) + (temp_dy * temp_dy);
	temp_odd = -1;
	radical = 0;
	
	while(1) {
	    temp_odd = temp_odd + 2;
	    if (speed_squared >= temp_odd){
			speed_squared = speed_squared - temp_odd;
			radical++;
	    }else
		 break;
	      

	}

	if (radical > 12)
	    radical = 12;
	else if (radical < 4)
	    radical = 4;

	*dx = (temp_dx * radical + remainder_x)/18;
	*dy = (temp_dy * radical + remainder_y)/18;
	
	if (*dx || *dy){
		  temp_last_x = last_x;
		  temp_last_y = last_y;  
		  remainder_x = (temp_dx * radical + remainder_x)%18;
		  remainder_y = (temp_dy * radical + remainder_y)%18;
	}
	
	
}

void elantech_report_absolute_to_related(struct asusec_chip *ec_chip,int *Null_data_times)
{
	struct elantech_data *etd = (struct elantech_data *) ec_chip->private;
	struct input_dev *dev = etd->abs_dev;
	unsigned char packet[6] = {0};
	int finger;
	int dx,dy;
	static unsigned int x,y;
	static unsigned int x2_1,y2_1;
	static unsigned int x2_2,y2_2;
	static unsigned int x3_1,y3_1;
	static unsigned int last_x,last_y;
	static unsigned int last_1_x2_1,last_1_y2_1;
	static unsigned int last_1_x2_2,last_1_y2_2;
	static unsigned int last_2_x2_1,last_2_y2_1;
	static unsigned int last_2_x2_2,last_2_y2_2;
	static unsigned int donot_jump_x2_1, donot_jump_x2_2;
	static unsigned int last_finger = 0;
	unsigned int temp_x2_1,temp_y2_1;
	unsigned int temp_x2_2,temp_y2_2;
	int count_i;
	
	int left,right,middle;
	edge_type edge;
	static int finger_flag=0;
	static int last_tap_num=0;
	int wheel,hwheel,i;

	for(i=0;i<6;i++){
		packet[i] = ec_chip->ec_data[i];
	}
	
	finger = (packet[0] & 0xc0) >> 6;
	
	 /* 3rd button emulation */
	left = (packet[0] & 0x01);
	right = (packet[0] & 0x02) >> 1;
	
	middle=0;
	if (left && right){
		middle = 1;
		left = 0;
		right = 0;
	}
	
	if(finger == 0 && finger_flag == 1){
		etd->release_x = x;
		etd->release_y = y;
	}	

        
	ELAN_INFO("finger = %d , last_finger=%d\n",finger,last_finger);
	if (finger != 2 ||  last_finger != 2){
		donot_jump_x2_2 = donot_jump_x2_1 = 0;
		last_1_x2_1 = last_1_y2_1 = 0;
		last_1_x2_2 = last_1_y2_2 = 0;
		last_2_x2_1 = last_2_y2_1 = 0;
		last_2_x2_2 = last_2_y2_2 = 0;
		*Null_data_times = 0;
	}
	last_finger = finger;	
	
	switch (finger) {
	    case 0:
			break;
	    case 1:
			x = (( packet[1] & 0x0f) << 8 ) | packet[2];
			y = (( packet[4] & 0x0f) << 8 ) | packet[5];
			break;
	    case 2:

			ELAN_INFO("Null_data_times=%d\n",*Null_data_times);
			if((packet[0]&0x0c)==0x04){
				x2_1 =(( packet[1] & 0x0f) << 8 ) | packet[2];
				y2_1 = Public_ETP_2FT_YMAX - ((( packet[4] & 0x0f) << 8 ) | packet[5]);
				
				if (y2_1 < 435){
					donot_jump_x2_1 = x2_1;
				}else
					x2_1 = donot_jump_x2_1;

				
				if (*Null_data_times == 0){
					last_1_x2_1 = x2_1;
					last_1_y2_1 = y2_1;
				}else{
					last_2_x2_1 = x2_1;
					last_2_y2_1 = y2_1;
				}
				
				ELAN_INFO("x2_1 =%d y2_1=%d last_1_x2_1=%d last_1_y2_1=%d last_2_x2_1=%d last_2_y2_1=%d\n",x2_1,y2_1,last_1_x2_1,last_1_y2_1,last_2_x2_1,last_2_y2_1);
				return;
			}else {
				x2_2 =(( packet[1] & 0x0f) << 8 ) | packet[2];
				y2_2 = Public_ETP_2FT_YMAX - ((( packet[4] & 0x0f) << 8 ) | packet[5]);
	
				if (y2_2 < 435){
					donot_jump_x2_2 = x2_2;
				}else
					x2_2 = donot_jump_x2_2;
				
				
				if (*Null_data_times == 0){
					last_1_x2_2 = x2_2;
					last_1_y2_2 = y2_2;
				}else{
					last_2_x2_2 = x2_2;
					last_2_y2_2 = y2_2;
				}

				ELAN_INFO("x2_2 =%d y2_2=%d last_1_x2_2=%d last_1_y2_2=%d last_2_x2_2=%d last_2_y2_2=%d\n",x2_2,y2_2,last_1_x2_2,last_1_y2_2,last_2_x2_2,last_2_y2_2);
				if (*Null_data_times != 0 && last_finger == 2){
					ELAN_INFO("last_1_x2_1=%d last_1_y2_1=%d last_2_x2_1=%d last_2_y2_1=%d\n",last_1_x2_1,last_1_y2_1,last_2_x2_1,last_2_y2_1);
					ELAN_INFO("last_1_x2_2=%d last_1_y2_2=%d last_2_x2_2=%d last_2_y2_2=%d\n",last_1_x2_2,last_1_y2_2,last_2_x2_2,last_2_y2_2);
					for (count_i = 0 ; count_i < *Null_data_times ; count_i++){
						if (last_2_x2_1 > last_1_x2_1)
							temp_x2_1 = last_1_x2_1 + ((last_2_x2_1 - last_1_x2_1) * (count_i + 1)/(*Null_data_times + 1)); 
						else
							temp_x2_1 = last_1_x2_1 - ((last_1_x2_1 - last_2_x2_1) * (count_i + 1)/(*Null_data_times + 1));

						if (last_2_y2_1 > last_1_y2_1)
							temp_y2_1 = last_1_y2_1 + ((last_2_y2_1 - last_1_y2_1) * (count_i + 1)/(*Null_data_times + 1)); 
						else
							temp_y2_1 = last_1_y2_1 - ((last_1_y2_1 - last_2_y2_1) * (count_i + 1)/(*Null_data_times + 1));
						
						//===================================================================================================
						if (last_2_x2_2 > last_1_x2_2)
							temp_x2_2 = last_1_x2_2 + ((last_2_x2_2 - last_1_x2_2) * (count_i + 1)/(*Null_data_times + 1)); 
						else
							temp_x2_2 = last_1_x2_2 - ((last_1_x2_2 - last_2_x2_2) * (count_i + 1)/(*Null_data_times + 1));

						if (last_2_y2_2 > last_1_y2_2)
							temp_y2_2 = last_1_y2_2 + ((last_2_y2_2 - last_1_y2_2) * (count_i + 1)/(*Null_data_times + 1)); 
						else
							temp_y2_2 = last_1_y2_2 - ((last_1_y2_2 - last_2_y2_2) * (count_i + 1)/(*Null_data_times + 1));
							
	  					HandleTwoFingerZoom(finger,temp_x2_1,temp_y2_1,temp_x2_2,temp_y2_2,ec_chip);

					}
							
					*Null_data_times = 0;
					last_1_x2_1 = last_2_x2_1;
					last_1_y2_1 = last_2_y2_1;
					last_1_x2_2 = last_2_x2_2;
					last_1_y2_2 = last_2_y2_2;

				}
			}
			break;
	    case 3:
			x3_1 = (( packet[1] & 0x0f) << 8 ) | packet[2];
			y3_1 = (( packet[4] & 0x0f) << 8 ) | packet[5];
			break;    
	    
	  }

	  if(finger == 1 && finger_flag == 0){	  
	    etd->touch_on_x = x;
	    etd->touch_on_y = y;	     
	  } 
      
	  //edge = edge_detection(etd,x,y,finger);
	  edge = 0;
	  
	  wheel = hwheel = 0;
	  HandleScrolling(edge,x,y,finger,&wheel,&hwheel);
	  HandleTwoFingerZoom(finger,x2_1,y2_1,x2_2,y2_2,ec_chip);
	  dx = dy = 0;	  
	  ComputeDelta(&dx,&dy,edge,x,y,last_x,last_y,finger,finger_flag);		
	  
	  HandleTapProcessing(etd,edge,finger,finger_flag,x,y,dx,dy);
	  if ((last_tap_num != etd->tap_num) && ((last_tap_num == TP_CLICK) || (last_tap_num == TP_DOUBLE_CLICK)))
		del_timer_sync(&console_timer);
	  if((last_tap_num != etd->tap_num) && ((etd->tap_num == TP_CLICK) || (etd->tap_num == TP_DOUBLE_CLICK))){
		mod_timer(&console_timer,jiffies+(HZ * 1/100));
	  }
	  else if((etd->tap_num == TP_DRAG || etd->tap_num == TP_DRAG_MOVE) && finger == 1){
		input_report_key(dev, BTN_LEFT,1);
		input_sync(dev);	
	  }else if ((last_tap_num == TP_DRAG || last_tap_num == TP_DRAG_MOVE) && finger == 0){
		input_report_key(dev, BTN_LEFT,0);
                input_sync(dev);
		etd->tap_num = NO_FINGER_ON_TOUCHPAD;

	  }

	  finger_flag = finger;
	  last_tap_num = etd->tap_num;
	  last_x = x;
	  last_y = y;

	  if (!(etd->tap_num == TP_DRAG || etd->tap_num == TP_DRAG_MOVE))
	    	input_report_key(dev, BTN_LEFT,    left);

	  input_report_rel(dev, REL_X, dx);
	  input_report_rel(dev, REL_Y, -dy);
	  input_report_rel(dev, REL_WHEEL, wheel);
	  input_report_rel(dev, REL_HWHEEL,hwheel);	  
	  input_report_key(dev, BTN_MIDDLE,  middle);
	  input_report_key(dev, BTN_RIGHT,   right);
	  input_sync(dev);
	  
}

/*
 * Put the touchpad into absolute mode
 */
 
static int elantech_set_absolute_mode(struct asusec_chip *ec_chip)
{
	
	struct i2c_client *client;
	unsigned char reg_10 = 0x03;	
		
	ELAN_INFO("elantech_set_absolute_mode 2\n");
	client = ec_chip->client;
	
	if ((!elan_i2c_asus_cmd(client, NULL, ETP_PS2_CUSTOM_COMMAND)) &&
	    (!elan_i2c_asus_cmd(client, NULL, ETP_REGISTER_RW)) &&
	    (!elan_i2c_asus_cmd(client, NULL, ETP_PS2_CUSTOM_COMMAND)) &&
	    (!elan_i2c_asus_cmd(client, NULL, 0x0010)) &&
	    (!elan_i2c_asus_cmd(client, NULL, ETP_PS2_CUSTOM_COMMAND)) &&
	    (!elan_i2c_asus_cmd(client, NULL, reg_10)) &&
	    (!elan_i2c_asus_cmd(client, NULL, PSMOUSE_CMD_SETSCALE11))) {
		
		return 0;
	}
	return -1; 
}


/*
 * Set the appropriate event bits for the input subsystem
 */
static int elantech_set_input_rel_params(struct asusec_chip *ec_chip)
{
	struct elantech_data *etd ;
	unsigned char param[3];
	int ret;
			 
	if ((!elan_i2c_asus_cmd(ec_chip->client, NULL, ETP_PS2_CUSTOM_COMMAND)) &&
	    (!elan_i2c_asus_cmd(ec_chip->client, NULL, 0x0000)) &&
	    (!elan_i2c_asus_cmd(ec_chip->client, param, PSMOUSE_CMD_GETINFO))){
		
		etd = ec_chip->private;
		if(etd->abs_dev){
			return 0;
		}
		
		Public_ETP_XMAX_V2 = (0x0F & param[0]) << 8 | param[1];
		Public_ETP_YMAX_V2 = (0xF0 & param[0]) << 4 | param[2];		
		
		etd->tap_num = NO_FINGER_ON_TOUCHPAD;
		init_timer(&console_timer);
		console_timer.function = timertapprocessing;
		console_timer.data = (unsigned long) ec_chip;

		etd->abs_dev = input_allocate_device();
		ELAN_INFO("1 elantech_touchscreen=%p\n",etd->abs_dev);
		if (etd->abs_dev != NULL){
			ELAN_INFO("2 elantech_touchscreen=%p\n",etd->abs_dev);
			Public_ETP_2FT_YMAX = Public_ETP_YMAX_V2;
			etd->abs_dev->name = "elantech_touchscreen";
			
			etd->abs_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REL) | BIT_MASK(EV_SYN);
			etd->abs_dev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) |
						      BIT_MASK(BTN_RIGHT) | BIT_MASK(BTN_MIDDLE);
			etd->abs_dev->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y);
			etd->abs_dev->relbit[0] |= BIT_MASK(REL_WHEEL);
		
			__set_bit(REL_WHEEL, etd->abs_dev->relbit);
			__set_bit(REL_HWHEEL,etd->abs_dev->relbit);
		
			set_bit(EV_SYN, etd->abs_dev->evbit);
			set_bit(EV_KEY, etd->abs_dev->evbit);
			set_bit(EV_ABS, etd->abs_dev->evbit);
			set_bit(BTN_TOUCH, etd->abs_dev->keybit);
			set_bit(BTN_2, etd->abs_dev->keybit);
			input_set_abs_params(etd->abs_dev, ABS_X, ETP_XMIN_V2, Public_ETP_XMAX_V2, 0, 0);
			input_set_abs_params(etd->abs_dev, ABS_Y, ETP_YMIN_V2, Public_ETP_YMAX_V2, 0, 0);
			input_set_abs_params(etd->abs_dev, ABS_HAT0X, ETP_2FT_XMIN, Public_ETP_XMAX_V2, 0, 0);
			input_set_abs_params(etd->abs_dev, ABS_HAT0Y, ETP_2FT_YMIN, Public_ETP_YMAX_V2, 0, 0);
			input_set_abs_params(etd->abs_dev, ABS_PRESSURE, 0, 255, 0, 0);
			input_set_abs_params(etd->abs_dev, ABS_TOOL_WIDTH,0,16,0,0);
			input_set_abs_params(etd->abs_dev, ABS_MT_POSITION_X, ETP_2FT_XMIN, Public_ETP_XMAX_V2, 0, 0);
			input_set_abs_params(etd->abs_dev, ABS_MT_POSITION_Y, ETP_2FT_YMIN, Public_ETP_YMAX_V2, 0, 0);
			input_set_abs_params(etd->abs_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
			input_set_abs_params(etd->abs_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
			input_set_abs_params(etd->abs_dev, ABS_MT_TRACKING_ID, 0, FINGER_NUM, 0, 0);
				
			ret=input_register_device(etd->abs_dev);
			if (ret) {
			      ELAN_ERR("Unable to register %s input device\n", etd->abs_dev->name);		  
			}
		}
		return 0;
	}
	return -1;
  
}


/*
 * Use magic knock to detect Elantech touchpad
 */


int elantech_detect(struct asusec_chip *ec_chip)
{
  
	struct i2c_client *client;
	unsigned char param[3];
	ELAN_INFO("2.6.2X-Elan-touchpad-2010-11-27\n");

	client = ec_chip->client;
	
	if (elan_i2c_asus_cmd(client,  NULL, PSMOUSE_CMD_DISABLE) ||
	    elan_i2c_asus_cmd(client,  NULL, PSMOUSE_CMD_SETSCALE11) ||
	    elan_i2c_asus_cmd(client,  NULL, PSMOUSE_CMD_SETSCALE11) ||
	    elan_i2c_asus_cmd(client,  NULL, PSMOUSE_CMD_SETSCALE11) ||
	    elan_i2c_asus_cmd(client, param, PSMOUSE_CMD_GETINFO)) {
		ELAN_ERR("sending Elantech magic knock failed.\n");
		return -1;
	}

	/*
	 * Report this in case there are Elantech models that use a different
	 * set of magic numbers
	 */
	if (param[0] != 0x3c ||param[1] != 0x03 || param[2]!= 0x00) {
		ELAN_ERR("unexpected magic knock result 0x%02x, 0x%02x, 0x%02x.\n",
			param[0], param[1],param[2]);
		return -1;
	}
	
	return 0;
}

/*
 * Initialize the touchpad and create sysfs entries
 */

int elantech_init(struct asusec_chip *ec_chip)
{  
	ELAN_INFO("Elan et1059 elantech_init\n");
	
	if (elantech_set_absolute_mode(ec_chip)){
		ELAN_ERR("failed to put touchpad into absolute mode.\n");
		return -1;
	}
	if (elantech_set_input_rel_params(ec_chip)){
		ELAN_ERR("failed to elantech_set_input_rel_params.\n");
		return -1;
	}
	//elan_i2c_asus_cmd(ec_chip->client,  NULL, PSMOUSE_CMD_ENABLE);
	return 0;
}
