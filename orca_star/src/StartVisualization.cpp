#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <orca_star/AgentState.h>

class Visualizer {
public:
    Visualizer() : nh_() {
        sub_ = nh_.subscribe("/AgentStates", 10, &Visualizer::callback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
        ROS_INFO("Visualizer subscribed to /AgentStates");
    }

    void callback(const orca_star::AgentState::ConstPtr& msg) {
        ROS_INFO("Visualizer received AgentStateMsg with %zu agents", msg->pos.size());
        for (size_t i = 0; i < msg->pos.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "agents";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = msg->pos[i].x;
            marker.pose.position.y = msg->pos[i].y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = msg->rad[i] * 2;
            marker.scale.y = msg->rad[i] * 2;
            marker.scale.z = msg->rad[i] * 2;
            marker.color.r = (i == 0) ? 1.0 : 0.0;
            marker.color.g = (i == 0) ? 0.0 : 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration(0.5);
            marker_pub_.publish(marker);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher marker_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "start_visualization");
    Visualizer viz;
    ros::spin();
    return 0;
}
