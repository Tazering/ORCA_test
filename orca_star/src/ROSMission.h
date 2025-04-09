/*!
\file
\brief File contains ROSMission class.
*/

#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include "orca_star/AgentState.h"
#include "orca_star/AgentVelocity.h"
#include "orca_star/Init.h"
#include "Geom.h"
#include "XMLReader.h"
#include "Agent.h"
#include "Summary.h"

#ifndef ORCASTAR_ROSMISSION_H
#define ORCASTAR_ROSMISSION_H


/*!
* \brief Class is an implementation of multiagent navigation ROS module.
* \ingroup ORCAStarROS
*/

class ROSMission
{

    public:

        /*!
            \brief ROSMission constructor with parameters.
            Initialize multiagent simulation.
            Creates ROS topics and services.
            \param fileName Path to XML file with multiagent task
            \param agNum Number of agents
            \param threashold Simulation step threashold (default - no threashold)
            \param endOnFin End the simulation when the threshold is reached or when all agents reach their finish positions
        */
        ROSMission(const std::string &xmlPath);

        /*!
           \brief Default ROSMission destructor
        */
        ~ROSMission();

        /*!
         * \brief Prepare simulation of multiagen navigation.
         * Read task from XML file.
         * Create ROS service for agent initialization.
         *
         * \return Success of file reading
         */
        bool PrepareSimulation();

        /*!
         * \brief Start multiagent simulation execution
         */
        void StartSimulation();




    private:
        void GenerateAgentStateMsg();
        void UpdateVelocity(const orca_star::AgentVelocity &msg);
        void UpdateState();
        bool IsFinished();
        bool ReadTask();
        bool InitAgent(orca_star::Init::Request  &req, orca_star::Init::Response &res);
		ros::ServiceServer initServer;

        ///@cond
        size_t agNumber;
        size_t agCount;
        unsigned int stepsCount;
        unsigned int stepsTreshhold;


        orca_star::AgentState agentStateMsg;
        ros::NodeHandle n;
        ros::Publisher ROSMissionPub;
        ros::Subscriber ROSMissionSub;
        ros::Rate *loopRate;


        Reader *taskReader;
        std::vector<Agent *> agents;
        EnvironmentOptions *options;
        Map *map;
        Summary missionResult;
        std::unordered_map<int, std::pair<bool, int>> resultsLog;
        bool initFlag;
        bool endOnFinish;

        ///@endcond


};


#endif //ORCASTAR_ROSMISSION_H
