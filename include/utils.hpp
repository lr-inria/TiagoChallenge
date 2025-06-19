#ifndef UTILS_HPP_
#define UTILS_HPP_

/*************************************************
 * Define
 ************************************************/

#define PI_4 (7.85398163397e-1f)

/*************************************************
 * Structures
 ************************************************/

struct class Position
{
  float f_x;
  float f_y;
  float f_z;
};

struct class Quaternion
{
  float f_w;
  float f_x;
  float f_y;
  float f_z;
};

struct class State
{
  Position   position;
  Quaternion quaternion;
};

#ifndef // UTILS_HPP_
