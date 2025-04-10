cmake_minimum_required(VERSION 3.0.2)
project(orca_star)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  message_generation
)


add_message_files(
  FILES
  AgentState.msg
  AgentVelocity.msg
  ORCAInput.msg
)

add_service_files(
  FILES
  Init.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs visualization_msgs nav_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/ORCAStarLib
  ${CMAKE_CURRENT_SOURCE_DIR}/tinyxml
)

add_executable(start_agent
  src/StartAgent.cpp
  src/ROSAgent.cpp
  ORCAStarLib/Agent.cpp
  ORCAStarLib/ORCAAgent.cpp
  ORCAStarLib/Map.cpp
  ORCAStarLib/EnvironmentOptions.cpp
  ORCAStarLib/Geom.cpp
)

add_executable(start_mission
  src/StartMission.cpp
  src/ROSMission.cpp
  ORCAStarLib/Agent.cpp
  ORCAStarLib/ORCAAgent.cpp
  ORCAStarLib/Map.cpp
  ORCAStarLib/EnvironmentOptions.cpp
  ORCAStarLib/XMLReader.cpp
  ORCAStarLib/ThetaStar.cpp
  ORCAStarLib/DirectPlanner.cpp
  ORCAStarLib/Geom.cpp
  tinyxml/tinyxml2.cpp
)

add_executable(start_actor
  src/StartActor.cpp
  src/ROSSimActor.cpp
  ORCAStarLib/Agent.cpp
  ORCAStarLib/ORCAAgent.cpp
  ORCAStarLib/Map.cpp
  ORCAStarLib/EnvironmentOptions.cpp
  ORCAStarLib/Geom.cpp
)

add_executable(start_visualization
  src/StartVisualization.cpp
  ORCAStarLib/Agent.cpp
  ORCAStarLib/ORCAAgent.cpp
  ORCAStarLib/Map.cpp
  ORCAStarLib/EnvironmentOptions.cpp
  ORCAStarLib/Geom.cpp
)

add_dependencies(start_agent ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(start_mission ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(start_actor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(start_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(start_agent ${catkin_LIBRARIES})
target_link_libraries(start_mission ${catkin_LIBRARIES})
target_link_libraries(start_actor ${catkin_LIBRARIES})
target_link_libraries(start_visualization ${catkin_LIBRARIES})
