/*!
\file
\brief File contains ROSSimActor class implementation.
*/

#include "ROSSimActor.h"
#include "orca_star/AgentVelocity.h"
#include "orca_star/ORCAInput.h"
#include "orca_star/AgentState.h"

ROSSimActor::ROSSimActor()
{
    agNumber = 0;
    id = 0;
    initFlag = false;
    sightRad = 0.0;


    ros::ServiceClient client = n.serviceClient<orca_star::Init>("initServer");
    ros::service::waitForService("initServer");
    orca_star::Init srv;

    if (client.call(srv))
    {

        client.shutdown();
        ROS_DEBUG("Parameters received!");
        id = 0;
        sightRad = srv.request.sightRadius;
        initData = srv.response;
    }
    else
    {
        ROS_ERROR("Parameters not received!");
        exit(-1);
    }

    std::stringstream serverName;
    serverName << "initAgentServer_" << id;
    ros::ServiceServer initServer = n.advertiseService(serverName.str(), &ROSSimActor::InitAgent, this);

    while(!initFlag && ros::ok())
    {
        ros::spinOnce();
    }

    initServer.shutdown();

    agentVelocityMsg.id = id;
    agentVelocityMsg.vel = geometry_msgs::Point32();

    ROSSimActorPub = n.advertise<orca_star::AgentVelocity>("AgentVelocities", 1000);
    ROSSimActorSub = n.subscribe("AgentStates", 1000, &ROSSimActor::ReceiveAgentStates, this);

    std::stringstream inpTopicName;
    inpTopicName << "AgentInput_" << id;
    std::stringstream outTopicName;
    outTopicName << "AgentOutput_" << id;

    ROSSimActorToAgentPub = n.advertise<orca_star::ORCAInput>(inpTopicName.str(), 1000);
    ROSSimActorToAgentSub = n.subscribe(outTopicName.str(), 1000, &ROSSimActor::TransmitAgentVelocity, this);

    ROS_DEBUG("ROS Actor Init: %lu!", id);

    ros::spin();
}



ROSSimActor::~ROSSimActor(){}



void ROSSimActor::ReceiveAgentStates(const orca_star::AgentState &msg)
{
    ROS_DEBUG("Actor %lu start step", id);
    inputORCAMsg.pos = msg.pos[id];
    inputORCAMsg.vel = msg.vel[id];

    Point curr = {static_cast<float>(msg.pos[id].x), static_cast<float>(msg.pos[id].y)};

    inputORCAMsg.neighbours_pos.clear();
    inputORCAMsg.neighbours_vel.clear();
    inputORCAMsg.neighbours_rad.clear();

    for(size_t i = 0; i < msg.pos.size(); i++)
    {
        Point another = {static_cast<float>(msg.pos[i].x), static_cast<float>(msg.pos[i].y)};

        if(id != i && (curr-another).SquaredEuclideanNorm() < sightRad * sightRad)
        {
            inputORCAMsg.neighbours_pos.push_back(msg.pos[i]);
            inputORCAMsg.neighbours_vel.push_back(msg.vel[i]);
            inputORCAMsg.neighbours_rad.push_back(msg.rad[i]);
        }
    }


    ROSSimActorToAgentPub.publish(inputORCAMsg);
}

void ROSSimActor::TransmitAgentVelocity(const geometry_msgs::Point32 &msg)
{
    ROS_DEBUG("Actor %lu end step", id);

    agentVelocityMsg.id = id;
    agentVelocityMsg.vel = msg;
    ROSSimActorPub.publish(agentVelocityMsg);
}


bool ROSSimActor::InitAgent(orca_star::Init::Request &req, orca_star::Init::Response &res)
{
    res = initData;

    ROS_DEBUG("Sim Agent %lu\n Init", id);

    initFlag = true;

    return true;
}

