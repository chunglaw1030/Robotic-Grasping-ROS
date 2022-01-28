/* Software License Agreement (MIT License)
 *
 *  Copyright (c) 2019-, Wing Chung Law
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <cw1_team_5/cw1_team_5.h>

///////////////////////////////////////////////////////////////////////////////

CW1::CW1(ros::NodeHandle &nh)
{
  /* Constructor function, this is run only when an object of the class is first
  created. The aim of this function is to initialise the class */

  nh_ = nh;

  // namespace for our ROS services, they will appear as "/namespace/srv_name"
  std::string service_ns = "/moveit_solution";

  // // advertise the services available from this node
  // set_arm_srv_ = nh_.advertiseService(service_ns + "/set_arm",
  //   &SrvClass::setArmCallback, this);
  // set_gripper_srv_ = nh_.advertiseService(service_ns + "/set_gripper",
  //   &SrvClass::setGripperCallback, this);
  // add_collision_srv_ = nh_.advertiseService(service_ns + "/add_collision",
  //   &SrvClass::addCollisionCallback, this);
  // remove_collision_srv_ = nh_.advertiseService(service_ns + "/remove_collision",
  //   &SrvClass::removeCollisionCallback, this);
  // pick_srv_ = nh_.advertiseService(service_ns + "/pick",
  //   &SrvClass::pickCallback, this);

  ROS_INFO("MoveIt! services initialisation finished, namespace: %s", 
    service_ns.c_str());
}

///////////////////////////////////////////////////////////////////////////////

void
CW1::task1Callback(cw1_world_spawner::Task1Service::Request &request);

{
  /* Perform pick and place*/

  return
}

///////////////////////////////////////////////////////////////////////////////

void
CW1::task2Callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response);

{
  /* Object detection and localization */

  return 
}

///////////////////////////////////////////////////////////////////////////////

void
CW1::task3Callback(cw1_world_spawner::Task3Service::Request &request);
{
  /* pick up blue objects */


  return 
}

///////////////////////////////////////////////////////////////////////////////

void 
CW1::task1(geometry_msgs::PoseStamped object_loc, 
  geometry_msgs::PoseStamped goal_loc);

{


}
///////////////////////////////////////////////////////////////////////////////

void 
CW1::task2(std_msgs::Float32 r, std_msgs::Float32 g, std_msgs::Float32 b);

{


}
///////////////////////////////////////////////////////////////////////////////

void
CW1::task3(std_msgs::Float32 r, std_msgs::Float32 g, std_msgs::Float32 b,
  eometry_msgs::PoseStamped goal_loc);

{
  
}