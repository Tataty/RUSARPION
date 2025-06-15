/*
 * SpiderLeg.h
 *
 *  Created on: 8 июл. 2023 г.
 *      Author: Admin
 */

#ifndef INC_SPIDERLEG_H_
#define INC_SPIDERLEG_H_

#include <Servo.h>
#include "rusath.h"

#define L1 75
#define L2 155
#define L1L1mL2L2 -18400
#define L1L1pL2L2 29650
#define twoL1 150
#define twoL1L2 23250

#define Lcf 65

class SpiderLeg{
public:
  vec2 center_pos;
  vec2 normal_pos;

  Servo servo_c;
  Servo servo_f;
  Servo servo_t;

  int delta_c;
  int delta_f;
  int delta_t;

  SpiderLeg(vec2 center_pos, Servo servo_c, Servo servo_f, Servo servo_t) : servo_c(servo_c), servo_f(servo_f), servo_t(servo_t) {
    this->center_pos = center_pos;

    float len_to_center	= sqrt(center_pos.x*center_pos.x + center_pos.y*center_pos.y);
    this->normal_pos.x	= center_pos.x / len_to_center;
    this->normal_pos.y	= center_pos.y / len_to_center;

    this->delta_c = servo_c.max_angle / 2;
    this->delta_f = servo_f.max_angle / 2;
    this->delta_t = 0;
  }

  void MoveToDELTA(){
    servo_c.SetAngle(delta_c);
    servo_f.SetAngle(delta_f);
    servo_t.SetAngle(delta_t);
  }

  void MoveXYZ(vec3 pos){
    pos.x -= center_pos.x; pos.z -= center_pos.y;

    int zeta = delta_c + DegreesWithNormalRadians(pos.x, pos.z);

    servo_c.SetAngle(zeta);

    MoveXY({sqrt(pos.x*pos.x + pos.z*pos.z) - Lcf, pos.y});
  }

  int DegreesWithNormalRadians(float x, float y){
    int angle = degrees(atan2(y, x) - atan2(normal_pos.y, normal_pos.x));

    if (angle > 180)
	return -180 + angle - 180;
    else if (angle < -180)
	return 180 + angle + 180;

    return angle;
  }

  void MoveXY(vec2 pos){
    float x2y2 	= pos.x * pos.x + pos.y * pos.y;
    int   alpha	= degrees(atan2(pos.y, pos.x) - acos((L1L1mL2L2 + x2y2) / (twoL1 * sqrt(x2y2))));
    int   beta 	= degrees(acos((L1L1pL2L2 - x2y2) / twoL1L2));

    alpha = delta_f + alpha;
    beta  = delta_t + 180 - beta;

    servo_f.SetAngle(alpha);
    servo_t.SetAngle(beta);
  }
};

#endif /* INC_SPIDERLEG_H_ */
