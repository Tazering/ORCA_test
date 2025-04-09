#ifndef ROS_AGENT_H
#define ROS_AGENT_H

#include <ros/ros.h>
#include <orca_star/Init.h>
#include <orca_star/ORCAInput.h>
#include <orca_star/AgentVelocity.h>
#include <nav_msgs/GetMap.h>
#include "ORCAAgent.h"
#include "Geom.h"
#include "Map.h"

class ROSAgent {
public:
    ROSAgent(size_t id);
    void DoStep(const orca_star::ORCAInput &msg);
	size_t getId() const { return agentId; }	

private:
    ros::NodeHandle n;
    ros::Publisher ROSAgentPub;
    size_t agentId;
    ORCAAgent *agent;  // Changed from Agent to ORCAAgent
    AgentParam *param;
    EnvironmentOptions *options;
    size_t w, h;
    float cs;
    std::vector<std::vector<int>> grid;
};

#endif
