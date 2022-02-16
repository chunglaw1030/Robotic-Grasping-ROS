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

// typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

CW1::CW1(ros::NodeHandle &nh):
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_plane (new PointC), // plane point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  g_tree_cluster_ptr (new pcl::search::KdTree<PointT>), // Euclidean Cluster KdTree
  g_cloud_normals (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_cloud_normals2 (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_inliers_plane (new pcl::PointIndices), // plane seg
  g_coeff_plane (new pcl::ModelCoefficients), // plane coeff
  debug_ (false),

  g_rgb_cloud (new PointC),
  g_cloud_filtered_rgb (new PointC),
  g_cloud_crop_hull (new PointC),
  g_red_purple_cube_cloud (new PointC)
  
{
  /* Constructor function, this is run only when an object of the class is first
  created. The aim of this function is to initialise the class */

  g_nh = nh;

  // namespace for our ROS services, they will appear as "/namespace/srv_name"
  std::string service_ns = "/CW1";

  task1_srv_ = g_nh.advertiseService("/task1_start",
    &CW1::task1Callback, this);  
  task2_srv_ = g_nh.advertiseService("/task2_start",
    &CW1::task2Callback, this);  
  task3_srv_ = g_nh.advertiseService("/task3_start",
    &CW1::task3Callback, this);  


  ROS_INFO("MoveIt! services initialisation finished, namespace: %s", 
    service_ns.c_str());

    // Define the publishers
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, true);
  g_pub_cloud_centroid = nh.advertise<sensor_msgs::PointCloud2> ("centroid_cloud", 1, true);
  g_pub_purple_red = nh.advertise<sensor_msgs::PointCloud2> ("purple_red_cubes", 1, true);
  g_pub_crop_hull = nh.advertise<sensor_msgs::PointCloud2> ("crop_hull", 1, true);
  g_pub_pose = nh.advertise<geometry_msgs::PointStamped> ("centroid", 5, true);
  
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::task1Callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response)


{
  /* Perform pick and place*/

  // g_home_postion = arm_group_.getCurrentJointValues ();
  geometry_msgs::Point object_location;
  object_location = request.object_loc.pose.position;
  
  geometry_msgs::Point goal_location;
  goal_location = request.goal_loc.point;
  addCollisionItems(goal_location);
  
  g_pick_offset_ = pick_offset_task1;
  pick(object_location);

  bool success = drop(goal_location);

  moveHomePosition(g_home_postion);

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::task2Callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)

{
  /* Object detection and localization */
  g_r.data = request.r.data * g_rgb_convert;
  g_g.data = request.g.data * g_rgb_convert;
  g_b.data = request.b.data * g_rgb_convert;
  
  q_x180deg =  tf2::Quaternion(q_x180deg_x, q_x180deg_y, q_x180deg_z, q_x180deg_w);

  // determine the grasping orientation
  q_object.setRPY(angle_offset2_, angle_offset2_, angle_offset_);
  q_result = q_x180deg * q_object;
  grasp_orientation = tf2::toMsg(q_result);
  
  g_task2_pose.position.x = g_task2_pose_x;
  g_task2_pose.position.y = g_task2_pose_y;
  g_task2_pose.position.z = g_task2_pose_z;
  g_task2_pose.orientation = grasp_orientation;

  moveArm(g_task2_pose);

  ROS_INFO("RGB values: %g %g %g", 
    g_r.data, g_g.data, g_b.data);
    
  if (g_r.data ==g_blue_r_value && g_g.data == g_blue_g_value
    && g_b.data == g_blue_b_value) 
  {
    ROS_INFO("Colour: blue");
    g_required_colour = "blue";
    g_hue = g_hue_blue;
  } 
  else if (g_r.data ==g_purple_r_value && g_g.data == g_purple_g_value
      && g_b.data == g_purple_b_value) 
  {
    ROS_INFO("Colour: Purple");
    g_required_colour = "purple";
    g_hue = g_hue_purple;
  } 
  else if (g_r.data ==g_red_r_value && g_g.data == g_red_g_value 
      && g_b.data == g_red_b_value) 
  {
    ROS_INFO("Colour: red");
    g_required_colour = "red";
    g_hue = g_hue_red;
  } 


  ros::Duration(1.2).sleep();

  euclideanCluster (g_cloud_filtered_rgb);

  ros::Duration(1.2).sleep();

  pubFilteredPCMsg (g_pub_cloud_centroid, *g_cloud_filtered_rgb);
  
  for (auto i: g_centroid_list)
    response.centroids.push_back(i);
  
  for (auto i: response.centroids)
    std::cout <<  "response centroid list: \n" << i ;

  if (g_centroid_list.empty())
    std::cout <<  "No "<< g_required_colour <<" cubes found! \n";

  return free;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::task3Callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  moveHomePosition(g_home_postion);
  // g_home_postion = arm_group_.getCurrentJointValues ();
  g_r.data = request.r.data * g_rgb_convert;
  g_g.data = request.g.data * g_rgb_convert;
  g_b.data = request.b.data * g_rgb_convert;

  q_x180deg =  tf2::Quaternion(q_x180deg_x, q_x180deg_y, q_x180deg_z, q_x180deg_w);

  // determine the grasping orientation
  q_object.setRPY(angle_offset1_, angle_offset1_, angle_offset_);
  q_result = q_x180deg * q_object;
  grasp_orientation = tf2::toMsg(q_result);

  // q_object2.setRPY(angle_offset2_, angle_offset2_, angle_offset_);
  // q_result2 = q_x180deg * q_object2;
  // geometry_msgs::Quaternion grasp_orientation2 = tf2::toMsg(q_result2);

  geometry_msgs::Point goal_location;
  goal_location = request.goal_loc.point;
  // g_box_location = goal_location;
  addCollisionItems(goal_location);

  g_pick_offset_ = pick_offset_task3;

  g_task3_pose1.position.x = g_task3_pose1_x;
  g_task3_pose1.position.y = g_task3_pose1_y;
  g_task3_pose1.position.z = g_task3_pose1_z;
  g_task3_pose1.orientation = grasp_orientation;

  g_task3_pose2 = g_task3_pose1;
  g_task3_pose2.position.x = g_task3_pose2_x;

  // g_task3_pose_test.position.x = 0.05;
  // g_task3_pose_test.position.y = 0;
  // g_task3_pose_test.position.z = 0.6;
  // g_task3_pose_test.orientation = grasp_orientation2;

  if (g_r.data ==g_blue_r_value && g_g.data == g_blue_g_value
      && g_b.data == g_blue_b_value) 
  {
    ROS_INFO("Colour: blue");
    g_required_colour = "blue";
    g_hue = g_hue_blue;
  } 
  else if (g_r.data ==g_purple_r_value && g_g.data == g_purple_g_value
      && g_b.data == g_purple_b_value) 
  {
    ROS_INFO("Colour: Purple");
    g_required_colour = "purple";
    g_hue = g_hue_purple;
  } 
  else if (g_r.data ==g_red_r_value && g_g.data == g_red_g_value 
      && g_b.data == g_red_b_value) 
  {
    ROS_INFO("Colour: red");
    g_required_colour = "red";
    g_hue = g_hue_red;
  } 

  // moveArm(g_task3_pose_test);
  moveArm(g_task3_pose1);
  
  ros::Duration(0.2).sleep();

  euclideanCluster (g_cloud_filtered_rgb);

  ros::Duration(0.2).sleep();
  // pubFilteredPCMsg (g_pub_cloud_centroid, *g_cloud_filtered_rgb);

  while (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      std::cout << 'centroid list: \n' << i ;

      geometry_msgs::Point object_location;
      object_location = i.point;
      pick(object_location);

      // geometry_msgs::Point goal_location;
      // goal_location = request.goal_loc.point;
      drop(goal_location);
    }

    // applyCropHull (g_cloud_filtered_rgb); // idk
    // moveArm(g_task3_pose_test);
    moveArm(g_task3_pose1);
    euclideanCluster (g_cloud_filtered_rgb);
    ros::Duration(0.2).sleep();

  }
  // applyCropHull (g_cloud_filtered_rgb,
  //     goal_location, g_cloud_crop_hull);

  moveArm(g_task3_pose2);

  euclideanCluster (g_cloud_filtered_rgb);

  while (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      std::cout << 'centroid list: \n' << i ;

      geometry_msgs::Point object_location;
      object_location = i.point;
      pick(object_location);

      drop(goal_location);
    }
    moveArm(g_task3_pose2);
    euclideanCluster (g_cloud_filtered_rgb);
    ros::Duration(0.2).sleep();
  }

  moveHomePosition(g_home_postion);


  return free;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::moveHomePosition(const std::vector< double > &group_variable_values)
{
  /* This function moves the move_group to the target position */

  // setup the target pose
  ROS_INFO("Moving to initial position");
  arm_group_.setJointValueTarget(group_variable_values);

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

bool
CW1::moveCartesian(geometry_msgs::Pose current_pose,
      geometry_msgs::Pose target_pose)

{
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose);
  waypoints.push_back(target_pose);
  
  moveit_msgs::RobotTrajectory trajectory;
  g_fraction = arm_group_.computeCartesianPath(waypoints, 
    g_eef_step, g_jump_threshold, trajectory);

  arm_group_.execute(trajectory);

  return free;
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
  q_x180deg =  tf2::Quaternion(q_x180deg_x, q_x180deg_y, q_x180deg_z, q_x180deg_w);

  // determine the grasping orientation
  q_object.setRPY(angle_offset1_, angle_offset1_, angle_offset_);
  q_result = q_x180deg * q_object;
  grasp_orientation = tf2::toMsg(q_result);

  // set the desired grasping pose
  geometry_msgs::Pose grasp_pose;
  grasp_pose.position = position;
  grasp_pose.orientation = grasp_orientation;
  grasp_pose.position.z += g_pick_offset_;

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
  // success *= moveArm(grasp_pose);
  success *= moveCartesian(approach_pose, grasp_pose);
  
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
  success *= moveCartesian(grasp_pose, approach_pose);

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
  q_x180deg =  tf2::Quaternion(q_x180deg_x, q_x180deg_y, q_x180deg_z, q_x180deg_w);

  // determine the grasping orientation
  q_object.setRPY(angle_offset1_, angle_offset1_, angle_offset_);
  q_result = q_x180deg * q_object;
  grasp_orientation = tf2::toMsg(q_result);

  // set the desired pose to drop cube
  geometry_msgs::Pose drop_pose;
  drop_pose.position = position;
  drop_pose.orientation = grasp_orientation;
  drop_pose.position.z += g_drop_distance_;

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

///////////////////////////////////////////////////////////////////////////////

void
CW1::colourFilter(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{

  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;

  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_rgb_cloud);
  applyVX (g_rgb_cloud, g_rgb_cloud);
  applyPT (g_rgb_cloud, g_rgb_cloud);
  segPlane (g_rgb_cloud);
  
  filterPurpleRedCubes(g_rgb_cloud);
  // applyCropHull (g_rgb_cloud); 
  pcl::PackedRGBComparison<PointT>::Ptr
    red_condition_lb(new pcl::PackedRGBComparison<PointT>("r", pcl::ComparisonOps::GE, g_r.data-g_rbg_tolerance));
  pcl::PackedRGBComparison<PointT>::Ptr
    red_condition_ub(new pcl::PackedRGBComparison<PointT>("r", pcl::ComparisonOps::LE, g_r.data+g_rbg_tolerance));
  pcl::PackedRGBComparison<PointT>::Ptr
    green_condition_lb(new pcl::PackedRGBComparison<PointT>("g", pcl::ComparisonOps::GE, g_g.data-g_rbg_tolerance));
  pcl::PackedRGBComparison<PointT>::Ptr
    green_condition_ub(new pcl::PackedRGBComparison<PointT>("g", pcl::ComparisonOps::LE, g_g.data+g_rbg_tolerance));
  pcl::PackedRGBComparison<PointT>::Ptr
    blue_condition_lb(new pcl::PackedRGBComparison<PointT>("b", pcl::ComparisonOps::GE, g_b.data-g_rbg_tolerance));
  pcl::PackedRGBComparison<PointT>::Ptr
    blue_condition_ub(new pcl::PackedRGBComparison<PointT>("b", pcl::ComparisonOps::LE, g_b.data+g_rbg_tolerance));
  pcl::PackedHSIComparison<PointT>::Ptr
    hue_condition_lb(new pcl::PackedHSIComparison<PointT>("h", pcl::ComparisonOps::GE, g_hue - g_hue_tolerance));
  pcl::PackedHSIComparison<PointT>::Ptr
    hue_condition_ub(new pcl::PackedHSIComparison<PointT>("h", pcl::ComparisonOps::LT, g_hue + g_hue_tolerance));
  
  pcl::ConditionAnd<PointT>::Ptr color_cond (new pcl::ConditionAnd<PointT> ());
  color_cond->addComparison (red_condition_lb);
  color_cond->addComparison (red_condition_ub);
  color_cond->addComparison (green_condition_lb);
  color_cond->addComparison (green_condition_ub);
  color_cond->addComparison (blue_condition_lb);
  color_cond->addComparison (green_condition_ub);
  color_cond->addComparison (hue_condition_lb);
  color_cond->addComparison (hue_condition_ub);

  // Build the filter
  color_filter.setInputCloud(g_rgb_cloud);
  color_filter.setCondition (color_cond);
  color_filter.filter(*g_cloud_filtered_rgb);

  pubFilteredPCMsg (g_pub_cloud_centroid, *g_cloud_filtered_rgb);

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::applyVX (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  g_vx.setInputCloud (in_cloud_ptr);
  g_vx.setLeafSize (g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter (*out_cloud_ptr);
  
  return;
}

void
CW1::applyCropHull (PointCPtr &in_cloud_ptr)
{
  // pcl::CropHull<pcl::PointXYZ> cropHullFilter;
  // boost::shared_ptr<PointCloud> hullCloud(new PointCloud());
  // boost::shared_ptr<PointCloud> hullPoints(new PointCloud());
  // std::vector<Vertices> hullPolygons;

  // THIS IS NOT USED


  PointT p1;
  p1.x = -g_box_location.x + box_centre_offset;
  p1.y = g_box_location.y - box_centre_offset;
  p1.z = g_box_location.z ;
  // hullCloud->push_back(p1);  }

  PointT p2;
  p2.x = -g_box_location.x + box_centre_offset;
  p2.y = g_box_location.y + box_centre_offset;
  p2.z = g_box_location.z ;
  // hullCloud->push_back(p);

  PointT p3;
  p3.x = -g_box_location.x - box_centre_offset;
  p3.y = g_box_location.y - box_centre_offset;
  p3.z = g_box_location.z ;
  // hullCloud->push_back(p);

  PointT p4;
  p4.x = -g_box_location.x - box_centre_offset;
  p4.y = g_box_location.y + box_centre_offset;
  p4.z = g_box_location.z ;
  // hullCloud->push_back(p);

  PointT p5;
  p5.x = -g_box_location.x + box_centre_offset;
  p5.y = g_box_location.y - box_centre_offset;
  p5.z = g_box_location.z - 2*box_centre_offset;
  // hullCloud->push_back(p1);

  PointT p6;
  p6.x = -g_box_location.x + box_centre_offset;
  p6.y = g_box_location.y + box_centre_offset;
  p6.z = g_box_location.z - 2*box_centre_offset;
  // hullCloud->push_back(p);

  PointT p7;
  p7.x = -g_box_location.x - box_centre_offset;
  p7.y = g_box_location.y - box_centre_offset;
  p7.z = g_box_location.z - 2*box_centre_offset;
  // hullCloud->push_back(p);

  PointT p8;
  p8.x = -g_box_location.x - box_centre_offset;
  p8.y = g_box_location.y + box_centre_offset;
  p8.z = g_box_location.z - 2*box_centre_offset;
  // hullCloud->push_back(p);


  pcl::PointCloud<PointT>::Ptr boundingbox_ptr (new PointC);//Used to store the boundary points

  pcl::PointCloud<PointT>::Ptr hull(new PointC);
  
  boundingbox_ptr->push_back(p1);
  boundingbox_ptr->push_back(p2);
  boundingbox_ptr->push_back(p3);
  boundingbox_ptr->push_back(p4);
  boundingbox_ptr->push_back(p5);
  boundingbox_ptr->push_back(p6);
  boundingbox_ptr->push_back(p7);
  boundingbox_ptr->push_back(p8);

  pcl::ConcaveHull<PointT> hull_calculator;
  std::vector<pcl::Vertices> polygons;

  hull_calculator.setInputCloud (boundingbox_ptr);
  hull_calculator.setAlpha (concave_hull_alpha);
  hull_calculator.reconstruct (*hull, polygons);
  int dim = hull_calculator.getDimension ();

// Crop Hull
  // CropHull<pcl::PointXYZ> crop_filter;
  g_crop_hull.setInputCloud (in_cloud_ptr);
  g_crop_hull.setHullCloud (hull);
  g_crop_hull.setHullIndices (polygons);
  g_crop_hull.setDim (dim);
  g_crop_hull.setCropOutside (false);
  g_crop_hull.filter (*g_cloud_crop_hull);

  pubFilteredPCMsg (g_pub_crop_hull, *g_cloud_crop_hull);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::applyPT (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("y");
  g_pt.setFilterLimits (g_pt_thrs_min, g_pt_thrs_max);
  g_pt.filter (*out_cloud_ptr);
  
  return;
}

    
////////////////////////////////////////////////////////////////////////////////
void
CW1::segPlane (PointCPtr &in_cloud_ptr)
{
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>), 
    cloud_f (new pcl::PointCloud<PointT>);

  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (seg_max_it);
  seg.setDistanceThreshold (seg_dist_thres);

  int nr_points = (int) in_cloud_ptr->size ();
  while (in_cloud_ptr->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (in_cloud_ptr);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (in_cloud_ptr);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    // std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *in_cloud_ptr = *cloud_f;
  }
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc)
{
  // Publish the data
  
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish (g_cloud_filtered_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::publishPose (geometry_msgs::PointStamped &cube_pt_msg)
{
  // Create and publish the centroid

  g_pub_pose.publish (cube_pt_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::pubFilteredRGB (ros::Publisher &pc_pub,
                               PointC &pc)
{
  // Publish the data
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  // g_cloud_filtered_msg.header.frame_id = g_input_pc_frame_id_;
  pc_pub.publish (g_cloud_filtered_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::euclideanCluster (PointCPtr &in_cloud_ptr)

{
  // pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  g_tree_cluster_ptr->setInputCloud (in_cloud_ptr);
  // pcl::PCDWriter writer;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (g_cluster_tolerance); // 2cm
  ec.setMinClusterSize (min_cluster_size);
  ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (g_tree_cluster_ptr);
  ec.setInputCloud (in_cloud_ptr);
  ec.extract (cluster_indices);
  
  
  // pcl::PointCloud<PointT>::Ptr cloud_cluster (new PointC);
  g_centroid_list.clear();
  int j = 1;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new PointC);
    for (const auto& idx : it->indices)
      cloud_cluster->push_back ((*in_cloud_ptr)[idx]); //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << g_required_colour << " cloud cluster " << j << "\n";
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points.\n" << std::endl;
    // std::stringstream ss;
    // ss << "cloud_cluster_" << j << ".pcd";
    // writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
    j++;

    geometry_msgs::PointStamped centroids;
    Eigen::Vector4f centroid_in;
    pcl::compute3DCentroid(*cloud_cluster, centroid_in);

    centroids.header.frame_id = g_input_pc_frame_id_;
    centroids.header.stamp = ros::Time (0);
    centroids.point.x = centroid_in[0];
    centroids.point.y = centroid_in[1];
    centroids.point.z = centroid_in[2];

    // Transform the point to new frame
      
    geometry_msgs::PointStamped cube_pt_msg_out;
    try
    {
      g_listener_.transformPoint (g_target_frame,  
                                  centroids,
                                  cube_pt_msg_out);
      //ROS_INFO ("trying transform...");
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
    }
 
    g_centroid_list.push_back(cube_pt_msg_out);
    publishPose (cube_pt_msg_out);


  } ; 

  std::cout << j-1 << " centroid(s) found! \n" ;

  return;

}

////////////////////////////////////////////////////////////////////////////////

void
CW1::addCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  /* add a collision object in RViz and the MoveIt planning scene */
  
  // input header information
  g_collision_object.id = object_name;
  g_collision_object.header.frame_id = base_frame_;

  // define the primitive and its dimensions
  g_collision_object.primitives.resize(1);
  g_collision_object.primitives[0].type = g_collision_object.primitives[0].BOX;
  g_collision_object.primitives[0].dimensions.resize(3);
  g_collision_object.primitives[0].dimensions[0] = dimensions.x;
  g_collision_object.primitives[0].dimensions[1] = dimensions.y;
  g_collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  g_collision_object.primitive_poses.resize(1);
  g_collision_object.primitive_poses[0].position.x = centre.x;
  g_collision_object.primitive_poses[0].position.y = centre.y;
  g_collision_object.primitive_poses[0].position.z = centre.z;
  g_collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object 
  // hint: what about collision_object.REMOVE?
  g_collision_object.operation = g_collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  g_object_vector.push_back(g_collision_object);
  planning_scene_interface_.applyCollisionObjects(g_object_vector);

  return;
}

void
CW1::filterPurpleRedCubes(PointCPtr &in_cloud_ptr)
{
  pcl::PackedRGBComparison<PointT>::Ptr
    red_purple_r_lb(new pcl::PackedRGBComparison<PointT>("r", pcl::ComparisonOps::GE, g_red_purple_cube_r-g_rbg_tolerance));
  pcl::PackedRGBComparison<PointT>::Ptr
    red_purple_r_ub(new pcl::PackedRGBComparison<PointT>("r", pcl::ComparisonOps::LE, g_red_purple_cube_r+g_rbg_tolerance));
  pcl::PackedRGBComparison<PointT>::Ptr
    red_purple_g_lb(new pcl::PackedRGBComparison<PointT>("g", pcl::ComparisonOps::GE, g_red_purple_cube_g-g_rbg_tolerance));
  pcl::PackedRGBComparison<PointT>::Ptr
    red_purple_g_ub(new pcl::PackedRGBComparison<PointT>("g", pcl::ComparisonOps::LE, g_red_purple_cube_g+g_rbg_tolerance));

  pcl::ConditionAnd<PointT>::Ptr color_cond_red_purple (new pcl::ConditionAnd<PointT> ());
  color_cond_red_purple->addComparison (red_purple_r_lb);
  color_cond_red_purple->addComparison (red_purple_r_ub);
  color_cond_red_purple->addComparison (red_purple_g_lb);
  color_cond_red_purple->addComparison (red_purple_g_ub);

  // Build the filter
  color_filter_red_purple.setInputCloud(in_cloud_ptr);
  color_filter_red_purple.setCondition (color_cond_red_purple);
  color_filter_red_purple.filter(*g_red_purple_cube_cloud);

  pubFilteredPCMsg (g_pub_purple_red, *g_red_purple_cube_cloud);

  return;
}

void
CW1::redPurpleCluster (PointCPtr &in_cloud_ptr)

{
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (in_cloud_ptr);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (g_cluster_tolerance); // 2cm
  ec.setMinClusterSize (min_cluster_size);
  ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (in_cloud_ptr);
  ec.extract (cluster_indices);
  
  g_centroid_list2.clear();
  int j = 1;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster2 (new PointC);
    for (const auto& idx : it->indices)
      cloud_cluster2->push_back ((*in_cloud_ptr)[idx]); //*
    cloud_cluster2->width = cloud_cluster2->size ();
    cloud_cluster2->height = 1;
    cloud_cluster2->is_dense = true;

    std::cout << "Red/Purple cluster " << j << "\n";
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster2->size () << " data points.\n" << std::endl;

    j++;

    geometry_msgs::PointStamped centroids_rpc;
    Eigen::Vector4f centroid_in;
    pcl::compute3DCentroid(*cloud_cluster2, centroid_in);

    // centroids_rpc.header.frame_id = g_input_pc_frame_id_;
    centroids_rpc.header.stamp = ros::Time (0);
    // centroids.point = j;
    centroids_rpc.point.x = centroid_in[0];
    centroids_rpc.point.y = centroid_in[1];
    centroids_rpc.point.z = centroid_in[2];

    geometry_msgs::PointStamped g_rpc_pt_msg;
    try
    {
      g_listener2_.transformPoint (base_frame_,  
                                  centroids_rpc,
                                  g_rpc_pt_msg);
      //ROS_INFO ("trying transform...");
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
    }
 
    g_centroid_list2.push_back(g_rpc_pt_msg);

  }; 

  for (auto i: g_centroid_list2)

    std::cout << 'Non target cubes centroid list: \n' << i ;

  return;

}

void 
CW1::addCollisionItems (geometry_msgs::Point &goal_location)

{

  g_floor_centre.x = g_floor_centre_x;
  g_floor_centre.y = g_floor_centre_y;
  g_floor_centre.z = g_floor_centre_z;
  g_floor_dimensions.x = g_floor_dimensions_x;
  g_floor_dimensions.y = g_floor_dimensions_x;
  g_floor_dimensions.z = g_floor_dimensions_z;

  g_box_centre.x = goal_location.x ;
  g_box_centre.y = goal_location.y ;
  g_box_centre.z = goal_location.z - box_centre_offset;
  g_box_dimensions.x = g_box_dimensions_x;
  g_box_dimensions.y = g_box_dimensions_y;
  g_box_dimensions.z = g_box_dimensions_z;
  g_box_orientation.x = g_box_orientation_x;
  g_box_orientation.y = g_box_orientation_y;
  g_box_orientation.z = g_box_orientation_z;
  g_box_orientation.w = g_box_orientation_w;

  redPurpleCluster (g_red_purple_cube_cloud);

  g_cube_dimensions.x = g_cube_dimensions_x;
  g_cube_dimensions.y = g_cube_dimensions_y;
  g_cube_dimensions.z = g_cube_dimensions_y;

  for (auto i: g_centroid_list2)
  {  
    g_box_centre.x = i.point.x ;
    g_cube_centre.y = i.point.y ;
    g_cube_centre.z = i.point.z ;
    
    addCollisionObject(g_cube, g_cube_centre, g_cube_dimensions,
    g_box_orientation);
    ROS_INFO("purple red cube collision added");
  }

  addCollisionObject(g_floor, g_floor_centre, g_floor_dimensions,
    g_box_orientation);

  addCollisionObject(g_box, g_box_centre, g_box_dimensions,
    g_box_orientation);


  return;
}