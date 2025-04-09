/*!
\file
\brief File contains program for multiagent simulation launching.
*/


#include "ROSMission.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROSMission");
    if (argc < 2) {
    	ROS_ERROR("No XML file provided! Usage: rosrun orca_star start_mission <path_to_xml>");
	return -1;
    }
    int agNum = 2;
    std::string file = argv[1];
    int t = 500;
    bool end = true;

    ros::param::get("~agents_number", agNum);
    ros::param::get("~task", file);
    ros::param::get("~threshhold", t);
    ros::param::get("~end", end);

    ROS_INFO("Received XML path: %s", file.c_str());
    ROSMission task = ROSMission(file, agNum, t, end);
    if(!task.PrepareSimulation())
    {
        ROS_ERROR("File problem!");
        ros::shutdown();
    }

    task.StartSimulation();
    ros::spin();

    return 0;
}
