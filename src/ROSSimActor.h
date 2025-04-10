/*!
\file
\brief File contains ROSSimActor class.
*/


#include <vector>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include "orca_star/AgentState.h"
#include "orca_star/AgentVelocity.h"
#include "orca_star/ORCAInput.h"
#include "Geom.h"
#include "orca_star/Init.h"

#ifndef ORCASTAR_ROSSIMACTOR_H
#define ORCASTAR_ROSSIMACTOR_H

/*!
 * \brief The ROSSimActor class is implementation of the intermediate node between the simulation and the agent.
 * \ingroup ORCAStarROS
 */
class ROSSimActor
{
    public:
         /*!
         * \brief ROSSimActor default constructor
         */
        ROSSimActor();

        /*!
          \brief ROSSimActor destructor
          */
        ~ROSSimActor();

        /*!
         * \brief Receives all agents states and transmits neighbours states to single agent module.
         * \param msg All agents states message
         */
        void ReceiveAgentStates(const orca_star::AgentState &msg);

        /*!
         * \brief Receives computed velocity from agent module and transmits it to simulation module.
         * \param msg New velocity message.
         */
        void TransmitAgentVelocity(const geometry_msgs::Point32 &msg);



    private:

        //! \cond
        bool InitAgent(orca_star::Init::Request  &req, orca_star::Init::Response &res);


        size_t id;
        size_t agNumber ;
        float sightRad;

        orca_star::AgentVelocity agentVelocityMsg;
        orca_star::ORCAInput inputORCAMsg;
        orca_star::Init::Response initData;

        ros::NodeHandle n;
        ros::Publisher ROSSimActorPub;
        ros::Subscriber ROSSimActorSub; 

        ros::Publisher ROSSimActorToAgentPub;
        ros::Subscriber ROSSimActorToAgentSub;

        bool initFlag;

        //! \endcond
};



#endif //ORCASTAR_ROSSIMACTOR_H
