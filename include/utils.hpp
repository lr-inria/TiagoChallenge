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
