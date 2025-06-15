/*
 * rusath.h
 *
 *  Created on: Jul 5, 2023
 *      Author: Admin
 */

#ifndef INC_RUSATH_H_
#define INC_RUSATH_H_

#include <cmath>

struct vec3 {
  float x;
  float y;
  float z;

  const vec3 operator+(const vec3& vec) const {
      return {x + vec.x, y + vec.y, z + vec.z};
  }
  const vec3 operator-(const vec3& vec) const {
      return {x - vec.x, y - vec.y, z - vec.z};
  }
  const vec3 operator-() const {
      return {-x, -y, -z};
  }
  const vec3 operator*(const float& mult) const {
      return {x * mult, y * mult, z * mult};
  }
  const vec3 operator/(const float& mult) const {
      return {x / mult, y / mult, z / mult};
  }
};

struct vec2 {
  float x;
  float y;

  float length(){
    return sqrt(x*x + y*y);
  }

  void rotate90(){
    float temp_y = y;
    y = -x;
    x = temp_y;
  }

  const vec2 operator+(const vec2& vec) const {
      return {x + vec.x, y + vec.y};
  }
  const vec2 operator-(const vec2& vec) const {
      return {x - vec.x, y - vec.y};
  }
  const vec2 operator-() const {
      return {-x, -y};
  }
  const vec2 operator*(const float& mult) const {
      return {x * mult, y * mult};
  }
  const vec2 operator/(const float& mult) const {
      return {x / mult, y / mult};
  }
};

static float degrees(float rad){
	return rad / M_PI * 180;
}

#endif /* INC_RUSATH_H_ */
