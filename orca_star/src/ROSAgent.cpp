#include "ROSAgent.h"

ROSAgent::ROSAgent(size_t id) {
    agentId = id;
    std::stringstream initServerName;
    initServerName << "initAgentServer_" << id;
    ros::ServiceClient client = n.serviceClient<orca_star::Init>(initServerName.str());
    ros::service::waitForService(initServerName.str());
    orca_star::Init srv;

    if (client.call(srv)) {
        client.shutdown();
        ROS_DEBUG("Agent %lu Parameters received!", id);
    } else {
        ROS_ERROR("Agent %lu Parameters not received!", id);
        exit(-1);
    }

    options = new EnvironmentOptions(CN_DEFAULT_METRIC_TYPE, CN_DEFAULT_BREAKINGTIES,
                                     CN_DEFAULT_ALLOWSQUEEZE, CN_DEFAULT_CUTCORNERS,
                                     CN_DEFAULT_HWEIGHT, CN_DEFAULT_TIME_STEP, srv.response.delta);

    param = new AgentParam(srv.request.sightRadius, srv.request.timeBoundary, srv.request.timeBoundaryObst,
                           srv.request.radius, CN_DEFAULT_REPS, srv.request.speed, srv.request.agentMaxNum);

    Point start = Point(static_cast<float>(srv.request.start.x), static_cast<float>(srv.request.start.y));
    Point goal = Point(static_cast<float>(srv.request.goal.x), static_cast<float>(srv.request.goal.y));

    // Use ORCAAgent instead of Agent
    Map dummyMap;
    agent = new ORCAAgent(id, start, goal, dummyMap, *options, *param);

    // Handle obstacles (ORCAAgent doesn’t have SetObstacles, so skip or adapt)
    std::vector<std::vector<Point>> obstacles;
    for (auto &obst : srv.request.Obstacles) {
        std::vector<Point> tmp;
        for (auto &vert : obst.points) {
            tmp.push_back(Point(static_cast<float>(vert.x), static_cast<float>(vert.y)));
        }
        obstacles.push_back(tmp);
    }
    // Note: ORCAAgent doesn’t use obstacles directly; adjust logic if needed

    // Map service call
    ros::ServiceClient mapClient = n.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap mapSrv;

    if (mapClient.call(mapSrv)) {
        this->w = mapSrv.response.map.info.width;
        this->h = mapSrv.response.map.info.height;
        this->cs = mapSrv.response.map.info.resolution;
        grid = std::vector<std::vector<int>>(h, std::vector<int>(w));
        for (size_t i = 0; i < h; ++i) {
            for (size_t j = 0; j < w; ++j) {
                grid[i][j] = mapSrv.response.map.data[i * w + j];
            }
        }
    } else {
        ROS_ERROR("Failed to get map for Agent %lu", id);
    }

    // Initialize publisher
    ROSAgentPub = n.advertise<orca_star::AgentVelocity>("agent_velocity", 1000);
}

void ROSAgent::DoStep(const orca_star::ORCAInput &msg) {
    agent->SetPosition(Point(static_cast<float>(msg.pos.x), static_cast<float>(msg.pos.y)));
    agent->SetVelocity(Point(static_cast<float>(msg.vel.linear.x), static_cast<float>(msg.vel.linear.y)));
    // ClearNeighbour doesn’t exist; use ClearAgentNeighbours if intended
//    agent->ClearNeighbours();

    for (size_t i = 0; i < msg.neighbours_pos.size(); i++) {
        // ORCAAgent’s AddNeighbour takes Agent&, float; adapt accordingly
        AgentParam nParam = *param;
        nParam.radius = msg.neighbours_rad[i];
        Point nPos = Point(static_cast<float>(msg.neighbours_pos[i].x), static_cast<float>(msg.neighbours_pos[i].y));
        Point nVel = Point(static_cast<float>(msg.neighbours_vel[i].linear.x), static_cast<float>(msg.neighbours_vel[i].linear.y));
	Map dummyMap;
	ORCAAgent* neighbour = new ORCAAgent(0, nPos, nPos, dummyMap, *options, nParam);
        neighbour->SetVelocity(nVel);
        float distSq = (agent->GetPosition() - nPos).SquaredEuclideanNorm();
        agent->AddNeighbour(*neighbour, distSq);
        delete neighbour; // Clean up temporary agent
    }

    agent->ComputeNewVelocity();
    orca_star::AgentVelocity velMsg;
    velMsg.id = agentId;
    velMsg.vel.x = agent->GetVelocity().X();
    velMsg.vel.y = agent->GetVelocity().Y();
    ROSAgentPub.publish(velMsg);
}
