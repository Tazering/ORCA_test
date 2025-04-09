/*!
\file
\brief File contains ROSMission class implementation.
*/


#include "ROSMission.h"


ROSMission::ROSMission(std::string fileName, size_t agNum, int threashold , bool endOnFin)
{
    ROS_INFO("Loading XML: %s", fileName.c_str());
	taskReader = new XMLReader(fileName);
    ROS_INFO("XML loaded, parsing agents...");

    ROS_DEBUG("Mission Init!");
    agNumber = agNum;
    agCount = 0;
    agents = std::vector<Agent*>();

    stepsCount = 0;
    stepsTreshhold = threashold;
    endOnFinish = endOnFin;

    options = nullptr;
    missionResult = Summary();
    resultsLog = std::unordered_map<int, std::pair<bool, int>>();
    resultsLog.reserve(agNumber);

	if (!taskReader -> ReadData()) {
		ROS_ERROR("Failed to read XML data");
		throw std::runtime_error("XML parsing failed");	
	}	

	ROS_INFO("Data parsed, retrieving agents...");
	if (!taskReader->GetAgents(agents, agNumber)) {
        ROS_ERROR("Failed to retrieve agents");
        throw std::runtime_error("Agent retrieval failed");
    }
    
	ROS_INFO("Agents retrieved, retrieving map...");
    if (!taskReader->GetMap(&map)) {
        ROS_ERROR("Failed to retrieve map");
        throw std::runtime_error("Map retrieval failed");
    }
    
	ROS_INFO("Map retrieved, retrieving options...");
    if (!taskReader->GetEnvironmentOptions(&options)) {
        ROS_ERROR("Failed to retrieve environment options");
        throw std::runtime_error("Options retrieval failed");
    }
    
	ROS_INFO("Initializing agentStateMsg with agNumber=%zu", agNumber);
    agentStateMsg.pos.resize(agNumber);
    agentStateMsg.vel.resize(agNumber);
    agentStateMsg.rad.resize(agNumber);
    ROS_INFO("AgentStateMsg initialized: pos=%zu, vel=%zu, rad=%zu", 
             agentStateMsg.pos.size(), agentStateMsg.vel.size(), agentStateMsg.rad.size());

    ROSMissionPub = n.advertise<orca_star::AgentState>("AgentStates", 1000);
    loopRate = new ros::Rate(10);
    ROSMissionSub = n.subscribe("AgentVelocities", 1000, &ROSMission::UpdateVelocity, this);
}

ROSMission::~ROSMission()
{
   ROS_INFO("Entering destructor...");
   delete loopRate;
   delete taskReader;
   taskReader = nullptr;
   for (Agent* agent : agents) {
		delete agent;	   
   }
	
   agents.clear();
   delete options;
   options = nullptr;
   delete map;
   map = nullptr;
   ROS_INFO("Destructor completed");
}

bool ROSMission::ReadTask()
{
    return taskReader->ReadData() && taskReader->GetMap(&map) &&
           taskReader->GetAgents(agents, agNumber) &&
           taskReader->GetEnvironmentOptions(&options);
}

void ROSMission::StartSimulation()
{
	ROS_INFO("Entering StartSimulation...");

    while (ros::ok() && !IsFinished()) {
		
		if(initFlag) {
			UpdateState();
			ROS_INFO("Updated state");
		} else {
			ROS_WARN("Waiting for initFlag to be set");
		}
		
        GenerateAgentStateMsg();
		ROS_INFO("Publishing AgentStateMsg: pos = %zu", agentStateMsg.pos.size());
        ROSMissionPub.publish(agentStateMsg);
		
        ros::spinOnce();
        stepsCount++;

        loopRate->sleep();

    }
	ROS_INFO("StartSimulation completed");
	
	while(ros::ok()) {
		ROSMissionPub.publish(agentStateMsg);
		ros::spinOnce();
		loopRate -> sleep();
	}
}



void ROSMission::GenerateAgentStateMsg() {
	ROS_INFO("Generating AgentStateMsg for %zu agents", agents.size());
    for(size_t i = 0; i < agents.size(); i++) {
        auto a = agents[i];
        geometry_msgs::Point pos;
        pos.x = a->GetPosition().X();
        pos.y = a->GetPosition().Y();
        agentStateMsg.pos[i] = pos;
        geometry_msgs::Twist vel;
        vel.linear.x = a->GetVelocity().X();
        vel.linear.y = a->GetVelocity().Y();
        agentStateMsg.vel[i] = vel;  // Fix tmp to vel
        agentStateMsg.rad[i] = a->GetParam().radius;
    }
    ROSMissionPub.publish(agentStateMsg);
}

void ROSMission::UpdateVelocity(const orca_star::AgentVelocity &msg)
{
  size_t id = msg.id;
  ROS_DEBUG("Agent %lu Update Velocity", id);
  agents[id]->SetVelocity(Point(msg.vel.x, msg.vel.y));
}

void ROSMission::UpdateState()
{
    for(auto &a : agents)
    {
        Point vel = a->GetVelocity();
        Point newPos = a->GetPosition() + vel * options->timestep;
        a->SetPosition(newPos);
    }
}


bool ROSMission::PrepareSimulation()
{
	ROS_INFO("Entering PrepareSimulation...");

	if (!map || agents.empty() || !options) {
		ROS_ERROR("Data invalid");
		return false;
	}

    ROS_INFO("Data checks passed: %zu agents, map %p, options %p", agents.size(), map, options);
    initServer = n.advertiseService("initServer", &ROSMission::InitAgent, this);
    ROS_INFO("initServer advertised");
	initFlag = false;
    ROS_INFO("Simulation prepared successfully");
    return true;
}



bool ROSMission::InitAgent(orca_star::Init::Request &req, orca_star::Init::Response &res) {
	ROS_INFO("InitAgent called");
	initFlag = true;
    return true;
}


bool ROSMission::IsFinished()
{
    if(!endOnFinish)
    {
        return false;
    }

    if(stepsCount == stepsTreshhold)
    {
        return true;
    }

    bool result = true;
    for(auto &agent : agents)
    {
        result = result && agent->isFinished();
        if(!result)
            break;
    }

    return result;
}
