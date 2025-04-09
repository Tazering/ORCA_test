/*!
\file
\brief File contains program for single agent navigation node launching.
*/


#include "ROSAgent.h"
#include "ros/ros.h"
#include <orca_star/Init.h>

#include <sstream>
#include <orca_star/ORCAInput.h>
#include <visualization_msgs/Marker.h>
#include <orca_star/AgentState.h>

class ROSAgentWrapper {
public:
    ROSAgentWrapper(int id) : actor(id) {
        sub = nh.subscribe("/AgentStates", 10, &ROSAgentWrapper::callback, this);
        ROS_INFO("Subscribed to /AgentStates for agent %d", id);
    }

    void callback(const orca_star::AgentState::ConstPtr& msg) {
        ROS_INFO("Agent %zu received AgentStateMsg with %zu agents", actor.getId(), msg->pos.size());
        // Process the message if needed
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ROSAgent actor;
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "ROSAgent", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    int i = 0;
	ROS_INFO("Starting ROSAgent with id %d", i);	

    ros::param::get("~id", i);

	ROS_INFO("Agent ID: %d", i);
	ROSAgentWrapper wrapper(i);
	ROS_INFO("ROSAgent initialized");

	ros::ServiceClient client = n.serviceClient<orca_star::Init>("initServer");

	ROS_INFO("ervice client created for /initServer");
	orca_star::Init srv;
	if (client.waitForExistence(ros::Duration(5.0))) {
		
		ROS_INFO("Service /initServer exists, calling...");
        if (client.call(srv)) {
            ROS_INFO("Called /initServer successfully");
        } else {
            ROS_ERROR("Failed to call /initServer");
        }
    } else {
        ROS_ERROR("Service /initServer not available");
    }
	ROS_INFO("Spinning...");
	ros::spin();
	return 0;
}
