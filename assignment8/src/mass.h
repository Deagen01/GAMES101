#ifndef MASS_H
#define MASS_H

#include "CGL/CGL.h"
#include "CGL/vector2D.h"

using namespace CGL;

struct Mass {
  Mass(Vector2D position, float mass, bool pinned)
      : start_position(position), position(position), last_position(position),
        mass(mass), pinned(pinned) {}

  float mass; //质量
  bool pinned;

  Vector2D start_position;
  Vector2D position;

  // explicit Verlet integration

  Vector2D last_position; //上一时刻的位置

  // explicit Euler integration

  Vector2D velocity; //当前质点的速度
  Vector2D forces; //当前节点受到的力
};

#endif /* MASS_H */
