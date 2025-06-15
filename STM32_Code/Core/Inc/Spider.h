/*
 * Spider.h
 *
 *  Created on: Jul 5, 2023
 *      Author: Admin
 */

#ifndef SRC_SPIDER_H_
#define SRC_SPIDER_H_

#include "SpiderLeg.h"

#define DELTA_X13 	215
#define DELTA_X2 	260
#define DELTA_Z13 	135

#define GROUND_Y 	110
#define AIR_Y 	 	40
#define STEP_LEG 	50

static float g_groundY = GROUND_Y;

struct math_arc{

  vec2 start_pos;
  vec2 end_pos;

  float high = AIR_Y;

  vec2 dec_se_pos;
  vec2 dec_es_pos;

  void SetPos(vec2 start_pos, vec2 end_pos){
    this->start_pos = start_pos;
    this->end_pos = end_pos;

    dec_se_pos = start_pos - end_pos;
    dec_es_pos = end_pos - start_pos;
  }

  vec3 GetPos(float cycle){
    if(cycle < 1){
      return {start_pos.x + dec_es_pos.x * cycle,
	      g_groundY - high * sin(cycle * M_PI),
	      start_pos.y + dec_es_pos.y * cycle};
    }else{
      cycle -= 1;
      return {end_pos.x + dec_se_pos.x * cycle,
	      g_groundY,
	      end_pos.y + dec_se_pos.y * cycle};
    }
  }
};

class MathSPL{
public:

  vec2 tangent;

  MathSPL(SpiderLeg* spl, vec2 main_pos){
    this->spl = spl;
    this->main_pos = main_pos;
    arc.SetPos(main_pos, main_pos);

    tangent = main_pos;
    tangent.rotate90();
    float length_tangent = tangent.length();
    if (length_tangent > 0) tangent = tangent / length_tangent;
  }

  void MoveArc(float cycle, float rate_change_dir){

    if (time_to_new_dir < 1){
      time_to_new_dir += rate_change_dir;

      presently_dir = last_dir + (new_dir - last_dir) * time_to_new_dir;
      arc.SetPos(main_pos - presently_dir, main_pos + presently_dir);
    }

    spl->MoveXYZ(arc.GetPos(cycle));
  }

  void SetMoveDirection(vec2 dir){
    last_dir = presently_dir;
    new_dir = dir;
    time_to_new_dir = 0;
  }

private:
  SpiderLeg* spl;
  math_arc arc;

  vec2 main_pos;

  vec2 last_dir;
  vec2 presently_dir = {0, 0};
  vec2 new_dir;

  float time_to_new_dir;
};

class Spider {
public:
  Spider(SpiderLeg* spl_r1, SpiderLeg* spl_r2, SpiderLeg* spl_r3,
	 SpiderLeg* spl_l1, SpiderLeg* spl_l2, SpiderLeg* spl_l3);

  void Update();


  void SetMovementVector(vec2 movement_vec);
  void SetRotatePower(float rotate_power);

  void SetMaxSpeed(float max_speed){

    this->max_speed = max_speed;
  }

  void SetSpeedCommand(uint8_t command_speed){

    if(command_speed == 0) {

	command_stop = true;
	speed_stop = speed;

	delta_stop = 1 - cycle;

	if (delta_stop < 0) {
	    dir_stop = false;
	    delta_stop = -delta_stop;
	} else {
	    dir_stop = true;
	}

    } else {
	command_stop = false;
    }

    speed = max_speed * ((float)command_speed / 255.f);
  }

private:

  void CalculateSPL(MathSPL* math_spl);
  void CalculateSpider();

  float cycle = 0;

  float speed = 0;
  float max_speed = 0;

  bool command_stop = false;
  bool dir_stop = 0;
  float delta_stop = 0;
  float speed_stop = 0;

  vec2 	movement_vec = {0, 0};
  float rotate_power = 0;

  MathSPL spl_r1;
  MathSPL spl_r2;
  MathSPL spl_r3;

  MathSPL spl_l1;
  MathSPL spl_l2;
  MathSPL spl_l3;
};

#endif /* SRC_SPIDER_H_ */
