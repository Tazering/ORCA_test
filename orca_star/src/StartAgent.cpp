/*!
\file
\brief File contains program for single agent navigation node launching.
*/


#include "ROSAgent.h"
#include "ros/ros.h"
#include <orca_star/Init.h>

#include <sstream>
#include <orca_star/ORCAInput.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ROSAgent", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    int i = 0;
    ros::param::get("~id", i);

	ROSAgent actor(i);
	ros::ServiceClient client = n.serviceClient<orca_star::Init>("initServer");
	orca_star::Init srv;
	if (client.waitForExistence(ros::Duration(5.0))) {
        if (client.call(srv)) {
            ROS_INFO("Called /initServer successfully");
        } else {
            ROS_ERROR("Failed to call /initServer");
        }
    } else {
        ROS_ERROR("Service /initServer not available");
    }

	ros::spin();
	return 0;
}
