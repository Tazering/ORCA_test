#ifndef AGENT_STATE_H
#define AGENT_STATE_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

namespace ORCAStar {
  struct AgentState {
    geometry_msgs::Point position;
    geometry_msgs::Twist velocity;
    double radius;
  };
}

#endif
