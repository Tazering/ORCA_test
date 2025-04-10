#ifndef ORCA_INPUT_H
#define ORCA_INPUT_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <vector>

namespace ORCAStar {
  struct NeighbourData {
    std::vector<geometry_msgs::Point> pos;
    std::vector<geometry_msgs::Twist> vel;
    std::vector<double> rad;
  };

  struct ORCAInput {
    geometry_msgs::Point pos;
    geometry_msgs::Twist vel;
    NeighbourData neighbours;
  };
}

#endif
