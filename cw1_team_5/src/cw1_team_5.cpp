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
  std::string service_ns = "/CW1";

  // task1_srv_ = nh_.advertiseService(service_ns + "/task1",
  //   &SrvClass::task1Callback, this);  
  // task2_srv_ = nh_.advertiseService(service_ns + "/task2",
  //   &SrvClass::task2Callback, this);  
  // task3_srv_ = nh_.advertiseService(service_ns + "/task3",
  //   &SrvClass::task3Callback, this);  
  task1_srv_ = nh_.advertiseService("/task1_start",
    &CW1::task1Callback, this);  
  task2_srv_ = nh_.advertiseService("/task2_start",
    &CW1::task2Callback, this);  
  task3_srv_ = nh_.advertiseService("/task3_start",
    &CW1::task3Callback, this);  


  ROS_INFO("MoveIt! services initialisation finished, namespace: %s", 
    service_ns.c_str());
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::task1Callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response)


{
  /* Perform pick and place*/
  // geometry_msgs::PoseStamped object_location
  // object_location.pose = request.object_loc

  // geometry_msgs::PoseStamped goal_location
  // goal_location.pose = request.goal_loc

  // bool success = task1(object_location, goal_location)
  geometry_msgs::Point object_location;
  object_location = request.object_loc.pose.position;
  pick(object_location);

  geometry_msgs::Point goal_location;
  goal_location = request.goal_loc.point;
  bool success = drop(goal_location);

  return success;
}

///////////////////////////////////////////////////////////////////////////////

void
CW1::task2Callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)

{
  /* Object detection and localization */

  std_msgs::Float32 r;
  std_msgs::Float32 g;
  std_msgs::Float32 b;
  r = request.r;
  g = request.g;
  b = request.b;

  ros::Subscriber sub_rgb =
  nh_.subscribe ("/r200/camera/depth_registered/points",
                1,
                task2(r, g, b));

  return;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::task3Callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* pick up blue objects */


  return 0;
}

///////////////////////////////////////////////////////////////////////////////

bool 
CW1::task1(geometry_msgs::PoseStamped object_loc, 
  geometry_msgs::PoseStamped goal_loc)

{
  // geometry_msgs::Point location
  // location.pose = object_loc
  
  // pick(object_loc.pose);
 
}
///////////////////////////////////////////////////////////////////////////////

void 
CW1::task2(std_msgs::Float32 r, std_msgs::Float32 g, std_msgs::Float32 b)

{


}
///////////////////////////////////////////////////////////////////////////////

bool
CW1::task3(std_msgs::Float32 r, std_msgs::Float32 g, std_msgs::Float32 b,
  geometry_msgs::PoseStamped goal_loc)

{
  
}

///////////////////////////////////////////////////////////////////////////////

void
CW1::findCentroid(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);

  // Perform the filtering
  applyVX (g_cloud_ptr, g_cloud_filtered);
  //applyPT (g_cloud_ptr, g_cloud_filtered);
  
  // Segment plane and cylinder
  findNormals (g_cloud_filtered);
  segPlane (g_cloud_filtered);
  segCylind (g_cloud_filtered);
  findCylPose (g_cloud_cylinder);
    
  // Publish the data
  //ROS_INFO ("Publishing Filtered Cloud 2");
  // pubFilteredPCMsg (g_pub_cloud, *g_cloud_filtered);
  pubFilteredPCMsg (g_pub_cloud, *g_cloud_cylinder);
  
  return;
}


///////////////////////////////////////////////////////////////////////////////

bool
CW1::moveArm(geometry_msgs::Pose target_pose)
{
  /* This function moves the move_group to the target position */

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::moveGripper(float width)
{
  /* this function moves the gripper fingers to a new position. Joints are:
      - panda_finger_joint1
      - panda_finger_joint2
  */

  // safety checks
  if (width > gripper_open_) width = gripper_open_;
  if (width < gripper_closed_) width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  hand_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::pick(geometry_msgs::Point position)
{
  /* This function picks up an object using a pose. The given point is where the
  centre of the gripper fingers will converge */

  // define grasping as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  // set the desired grasping pose
  geometry_msgs::Pose grasp_pose;
  grasp_pose.position = position;
  grasp_pose.orientation = grasp_orientation;
  grasp_pose.position.z += z_offset_;

  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose;
  approach_pose = grasp_pose;
  approach_pose.position.z += approach_distance_;

  /* Now perform the pick */

  bool success = true;

  ROS_INFO("Begining pick operation");

  // move the arm above the object
  success *= moveArm(approach_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to pick approach pose failed");
    return false;
  }

  // open the gripper
  success *= moveGripper(gripper_open_);

  if (not success) 
  {
    ROS_ERROR("Opening gripper prior to pick failed");
    return false;
  }

  // approach to grasping pose
  success *= moveArm(grasp_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
    return false;
  }

  // grasp!
  success *= moveGripper(gripper_closed_);

  if (not success) 
  {
    ROS_ERROR("Closing gripper to grasp failed");
    return false;
  }

  // retreat with object
  success *= moveArm(approach_pose);

  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed");
    return false;
  }

  ROS_INFO("Pick operation successful");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::drop(geometry_msgs::Point position)
{
  /* This function drops the picked object to the box
  centre of the gripper fingers will converge */

  // define grasping as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  // set the desired pose to approach top of box
  geometry_msgs::Pose drop_pose;
  drop_pose.position = position;
  drop_pose.orientation = grasp_orientation;
  drop_pose.position.z += approach_distance_;

  // set the desired pose to approach top of box
  geometry_msgs::Pose approach_box;
  approach_box = drop_pose;
  approach_box.position.z += approach_box_;

  /* Now perform the pick */

  bool success = true;

  ROS_INFO("Begining pick operation");

  // move the arm above the box
  success *= moveArm(approach_box);

  if (not success) 
  {
    ROS_ERROR("Moving arm to box failed");
    return false;
  }

  // move the arm to drop pose
  success *= moveArm(drop_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to drop pose failed");
    return false;
  }

  // open the gripper
  success *= moveGripper(gripper_open_);

  if (not success) 
  {
    ROS_ERROR("Opening gripper prior to pick failed");
    return false;
  }

  // move the arm above the box
  success *= moveArm(approach_box);

  if (not success) 
  {
    ROS_ERROR("Moving arm to box failed");
    return false;
  }

  // close gripper!
  success *= moveGripper(gripper_closed_);

  if (not success) 
  {
    ROS_ERROR("Closing gripper to grasp failed");
    return false;
  }

  ROS_INFO("Drop operation successful");

  return true;
}