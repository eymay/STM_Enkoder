/*
 * main.cpp
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#include <mainpp.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <nav_msgs/Odometry.h>
#include <custom.h>

//std_msgs::Int32 int32_msg_1;
//std_msgs::Int32 int32_msg_2;
//std_msgs::Int32 int32_msg_3;
//std_msgs::Int32 int32_msg_4;

 nav_msgs::Odometry odom_msg = nav_msgs::Odometry();

Encoder Encoder1, Encoder2;

struct Odom theOdom;

struct Message{
	std_msgs::Int32 int32_msg_pos, int32_msg_vel;
}Enc1_Msg, Enc2_Msg;

ros::NodeHandle nh;
ros::Publisher enkoder1_pub("Enkoder1", &Enc1_Msg.int32_msg_pos);
ros::Publisher enkoder2_pub("Enkoder2", &Enc2_Msg.int32_msg_pos);
ros::Publisher enkoder1_vel_pub("Enkoder1_Vel", &Enc1_Msg.int32_msg_vel);
ros::Publisher enkoder2_vel_pub("Enkoder2_Vel", &Enc2_Msg.int32_msg_vel);

ros::Publisher odom_pub("Odometry", &odom_msg);

/* class Encoder{
	public:
	uint32_t counter = 0;
	int32_t count = 0;
	int32_t position = 0;
	int speed = 0;
};
*/

void Speed_Publish(struct Encoder *enc){
		if(enc == &Encoder1){
			Enc1_Msg.int32_msg_vel.data = enc->speed;
			enkoder1_vel_pub.publish(&Enc1_Msg.int32_msg_vel);
		}if(enc == &Encoder2){
			Enc2_Msg.int32_msg_vel.data = enc->speed;
			enkoder2_vel_pub.publish(&Enc2_Msg.int32_msg_vel);
		}

}

void Position_Publish(struct Encoder *enc){
	if(enc == &Encoder1){
		Enc1_Msg.int32_msg_pos.data = enc->position;
		enkoder1_pub.publish(&Enc1_Msg.int32_msg_pos);
	}if(enc == &Encoder2){
		Enc2_Msg.int32_msg_pos.data = enc->position;
		enkoder2_pub.publish(&Enc2_Msg.int32_msg_pos);
	}
}

void Odom_Publish(struct Odom *od){
	//ros::Time begin = ros::Time::now();
	//odom_msg.header.stamp = begin;
	odom_msg.header.frame_id = "encoder_odom";
	odom_msg.child_frame_id = "base_link";
	odom_msg.pose.pose.position.x = od->x;
	odom_msg.pose.pose.position.y = od->y;
	odom_msg.pose.pose.orientation.z = sin(od->theta / 2);
	odom_msg.pose.pose.orientation.w = cos(od->theta / 2);
	odom_msg.twist.twist.linear.x = od->linear_velocity;
	odom_msg.twist.twist.angular.z = od->angular_velocity;
	odom_pub.publish(&odom_msg);
}




void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
    nh.advertise(enkoder1_pub);
  nh.advertise(enkoder2_pub);
  nh.advertise(enkoder1_vel_pub);
  nh.advertise(enkoder2_vel_pub);
  nh.advertise(odom_pub);

  //Encoder Encoder1, Encoder2;
}

void loop(void)
{



  nh.spinOnce();

  HAL_Delay(10);
}
