#ifndef INIT_H
#define INIT_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

namespace ORCAStar {
  struct Init {
    geometry_msgs::Point start_position;
    geometry_msgs::Point goal_position;
    geometry_msgs::Twist initial_velocity;
    double radius;
  };
}

#endif
