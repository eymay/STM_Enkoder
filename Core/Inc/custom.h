/*
 * custom.h
 *
 *  Created on: Jul 14, 2022
 *      Author: eymen
 */

#ifndef INC_CUSTOM_H_
#define INC_CUSTOM_H_

#ifdef __cplusplus
 extern "C" {
#endif
struct Encoder{
uint32_t counter;
int32_t count;
int32_t position;
int speed;
};

struct Odom{
	float x,y;
	float theta;
	float linear_velocity, angular_velocity;
};



 void Speed_Publish(struct Encoder *enc);
 void Position_Publish(struct Encoder *enc);
 void Odom_Publish(struct Odom *od);

#ifdef __cplusplus
}
#endif



#endif /* INC_CUSTOM_H_ */
