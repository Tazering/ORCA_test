#ifndef AGENT_VELOCITY_H
#define AGENT_VELOCITY_H

#include <geometry_msgs/Twist.h>

namespace ORCAStar {
  struct AgentVelocity {
    geometry_msgs::Twist velocity;
    // Add more fields if needed (e.g., preferred velocity)
  };
}

#endif
