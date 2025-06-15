/*
 * Spider.cpp
 *
 *  Created on: Jul 5, 2023
 *      Author: Admin
 */

#include "Spider.h"

void Spider::Update(){

  if (command_stop) {
      if (delta_stop <= 0) return;

      if (dir_stop) 	cycle 	+= speed_stop;
      else 		cycle 	-= speed_stop;
      delta_stop -= speed_stop;

      float cycle_phase = cycle > 1 ? cycle - 1 : cycle + 1;

      spl_r1.MoveArc(cycle	, speed_stop);
      spl_r2.MoveArc(cycle_phase, speed_stop);
      spl_r3.MoveArc(cycle	, speed_stop);

      spl_l1.MoveArc(cycle_phase, speed_stop);
      spl_l2.MoveArc(cycle	, speed_stop);
      spl_l3.MoveArc(cycle_phase, speed_stop);

      return;
  }

  cycle += speed;

  if (cycle > 2) cycle = 0;
  if (cycle < 0) cycle = 2;

  float cycle_phase = cycle > 1 ? cycle - 1 : cycle + 1;

  spl_r1.MoveArc(cycle	    , speed);
  spl_r2.MoveArc(cycle_phase, speed);
  spl_r3.MoveArc(cycle	    , speed);

  spl_l1.MoveArc(cycle_phase, speed);
  spl_l2.MoveArc(cycle	    , speed);
  spl_l3.MoveArc(cycle_phase, speed);

}

void Spider::SetMovementVector(vec2 movement_vec){

  float length = movement_vec.length();

  if (length <= 0) this->movement_vec = {0, 0};
  else if (length > 1) this->movement_vec = movement_vec / length;
  else this->movement_vec = movement_vec;

  CalculateSpider();
}

void Spider::SetRotatePower(float rotate_power){
  if (rotate_power > 1) rotate_power = 1;
  else if (rotate_power < -1) rotate_power = -1;
  this->rotate_power = rotate_power;

  CalculateSpider();
}

void Spider::CalculateSPL(MathSPL* math_spl){

  vec2 step = (math_spl->tangent * rotate_power) + movement_vec;

  float length_step = step.length();

  if (length_step <= 0) {
    math_spl->SetMoveDirection({0, 0});
    return;
  }

  if (length_step > 1)
    step = step / length_step;

  math_spl->SetMoveDirection(step * STEP_LEG);
}

void Spider::CalculateSpider(){
  CalculateSPL(&spl_r1);
  CalculateSPL(&spl_r2);
  CalculateSPL(&spl_r3);

  CalculateSPL(&spl_l1);
  CalculateSPL(&spl_l2);
  CalculateSPL(&spl_l3);
}

Spider::Spider(SpiderLeg* spl_r1, SpiderLeg* spl_r2, SpiderLeg* spl_r3,
	 SpiderLeg* spl_l1, SpiderLeg* spl_l2, SpiderLeg* spl_l3):
  spl_r1(spl_r1, {DELTA_X13, DELTA_Z13}),
  spl_r2(spl_r2, {DELTA_X2 , 0}),
  spl_r3(spl_r3, {DELTA_X13, -DELTA_Z13}),

  spl_l1(spl_l1, {-DELTA_X13, DELTA_Z13}),
  spl_l2(spl_l2, {-DELTA_X2 , 0}),
  spl_l3(spl_l3, {-DELTA_X13, -DELTA_Z13}){

  SetMovementVector({0, 0});
  SetRotatePower(0);
}
