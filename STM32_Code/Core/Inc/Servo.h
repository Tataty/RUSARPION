/*
 * servo.h
 *
 *  Created on: 8 июл. 2023 г.
 *      Author: Admin
 */
#pragma once

#define USMIN  500
#define USMAX  2500

#include "pca9685.h"

class Servo {
public:
  pca9685_handle_t* pca;
  uint8_t servo;

  int max_angle;

  int min_limit_angle;
  int max_limit_angle;

  Servo(pca9685_handle_t* pca, uint8_t servo, int max_angle) {
    this->pca 		= pca;
    this->servo 	= servo;
    this->max_angle	= max_angle;

    min_limit_angle 	= 0;
    max_limit_angle 	= max_angle;
  }

  void SetLimit(int min_limit_angle, int max_limit_angle){
    this->min_limit_angle = min_limit_angle;
    this->max_limit_angle = max_limit_angle;
  }

  void SetAngle(int angle){
    //pca9685_set_channel_pwm_times(pca, servo, USMIN * 4096 / 20000 + (USMAX - USMIN) * angle * 4096 / max_angle / 20000);

    if (angle < min_limit_angle) angle = min_limit_angle;
    if (angle > max_limit_angle) angle = max_limit_angle;

    pca9685_set_channel_pwm_times(pca, servo, 102 + 410 * angle / max_angle);
  }
};
