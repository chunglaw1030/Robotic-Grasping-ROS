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

typedef pcl::PointXYZRGBA PointD;
typedef pcl::PointCloud<PointD> PointE;
typedef PointE::Ptr PointEPtr;

CW1::CW1(ros::NodeHandle &nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_plane (new PointC), // plane point cloud
  g_cloud_cylinder (new PointC), // cylinder point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  g_cloud_normals (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_cloud_normals2 (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_inliers_plane (new pcl::PointIndices), // plane seg
  g_inliers_cylinder (new pcl::PointIndices), // cylidenr seg
  g_coeff_plane (new pcl::ModelCoefficients), // plane coeff
  g_coeff_cylinder (new pcl::ModelCoefficients), // cylinder coeff
  debug_ (false),

  // g_rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
  // g_cloud_filtered_rgb (new pcl::PointCloud<pcl::PointXYZRGB>)
  g_rgb_cloud (new PointE),
  g_cloud_filtered_rgb (new PointE),
  g_cloud_crop_hull (new PointE),
  g_red_purple_cube_cloud (new PointE),
  g_cloud_filtered_hsv (new pcl::PointCloud<pcl::PointXYZHSV>)
{
  /* Constructor function, this is run only when an object of the class is first
  created. The aim of this function is to initialise the class */

  nh_ = nh;

  // namespace for our ROS services, they will appear as "/namespace/srv_name"
  std::string service_ns = "/CW1";

  task1_srv_ = nh_.advertiseService("/task1_start",
    &CW1::task1Callback, this);  
  task2_srv_ = nh_.advertiseService("/task2_start",
    &CW1::task2Callback, this);  
  task3_srv_ = nh_.advertiseService("/task3_start",
    &CW1::task3Callback, this);  


  ROS_INFO("MoveIt! services initialisation finished, namespace: %s", 
    service_ns.c_str());

    // Define the publishers
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, true);
  g_pub_cloud_centroid = nh.advertise<sensor_msgs::PointCloud2> ("centroid_cloud", 1, true);
  g_pub_purple_red = nh.advertise<sensor_msgs::PointCloud2> ("purple_red_cubes", 1, true);
  g_pub_crop_hull = nh.advertise<sensor_msgs::PointCloud2> ("crop_hull", 1, true);
  g_pub_pose = nh.advertise<geometry_msgs::PointStamped> ("cyld_pt", 5, true);
  
  // Define public variables
  g_vg_leaf_sz = 0.01; // VoxelGrid leaf size: Better in a config file
  g_pt_thrs_min = 0.0; // PassThrough min thres: Better in a config file
  g_pt_thrs_max = 0.7; // PassThrough max thres: Better in a config file
  g_k_nn = 50; // Normals nn size: Better in a config file

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
  pick(object_location);

  geometry_msgs::Point goal_location;
  goal_location = request.goal_loc.point;
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
  g_r.data = request.r.data * 100;
  g_g.data = request.g.data * 100;
  g_b.data = request.b.data * 100;

  geometry_msgs::Pose task2_pose ;
  
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);
  task2_pose.position.x = 0.3;
  task2_pose.position.y = 0;
  task2_pose.position.z = 0.74;
  task2_pose.orientation = grasp_orientation;
  task2_pose.position.z += z_offset_;

  moveArm(task2_pose);

  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  // ros::Subscriber sub_rgb =
  // nh_.subscribe ("/r200/camera/depth_registered/points",
  //               1,
  //               &CW1::findCentroid,
  //               this);

  ROS_INFO("RGB values: %g %g %g", 
    g_r.data, g_g.data, g_b.data);

  ROS_INFO("Filtered RGB values: %g %g %g", 
  g_r.data, g_g.data, g_b.data);

  if (g_r.data ==10 && g_g.data == 10 && g_b.data == 80) {
  ROS_INFO("Colour: blue");
  g_required_colour = "blue";
  // g_hue = 42;
  // g_hue = 127;
  } 
  else if (g_r.data ==80 && g_g.data == 10 && g_b.data == 80) {
  ROS_INFO("Colour: Purple");
  g_required_colour = "purple";
  // g_hue = 84.5;
  // g_hue = 127;
  } 
  else if (g_r.data ==80 && g_g.data == 10 && g_b.data == 10) {
  ROS_INFO("Colour: red");
  g_required_colour = "red";
  // g_hue = -127;
  // g_hue = 50;
  } 

  // ros::Subscriber sub_rgb =
  // nh_.subscribe ("/r200/camera/depth_registered/points",
  //               1,
  //               findCentroid(sensor_msgs::PointCloud2ConstPtr &cloud_input_msg));

  // ros::Subscriber sub_rgb =
  // nh_.subscribe ("/r200/camera/depth_registered/points",
  //               1,
  //               findCentroid);
  // pubFilteredRGB (g_pub_cloud, *g_cloud_filtered_rgb);
  // ros::MultiThreadedSpinner spinner(1); // Use 4 threads
  // spinner.spin(); 
    // Creating the KdTree object for the search method of the extraction


  euclideanCluster (g_cloud_filtered_rgb);
  // start
  // pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  // tree->setInputCloud (g_cloud_filtered_rgb);
  // pcl::PCDWriter writer;
  // std::vector<pcl::PointIndices> cluster_indices;
  // pcl::EuclideanClusterExtraction<PointT> ec;
  // ec.setClusterTolerance (0.05); // 2cm
  // ec.setMinClusterSize (1);
  // ec.setMaxClusterSize (25000);
  // ec.setSearchMethod (tree);
  // ec.setInputCloud (g_cloud_filtered_rgb);
  // ec.extract (cluster_indices);
  
  // // for (auto i: cluster_indices)
  // //   std::cout << i << ' ';
  // // std::cout << "cluster_indices: " << cluster_indices; 
  
  // // pcl::PointCloud<PointT>::Ptr cloud_cluster (new PointE);
  // centroid_list.clear();
  // int j = 0;
  // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  // {
  //   pcl::PointCloud<PointT>::Ptr cloud_cluster (new PointE);
  //   for (const auto& idx : it->indices)
  //     cloud_cluster->push_back ((*g_cloud_filtered_rgb)[idx]); //*
  //   cloud_cluster->width = cloud_cluster->size ();
  //   cloud_cluster->height = 1;
  //   cloud_cluster->is_dense = true;

  //   std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
  //   std::stringstream ss;
  //   ss << "cloud_cluster_" << j << ".pcd";
  //   writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
  //   j++;

  //   geometry_msgs::PointStamped centroids;
  //   Eigen::Vector4f centroid_in;
  //   pcl::compute3DCentroid(*cloud_cluster, centroid_in);

  //   centroids.header.frame_id = g_input_pc_frame_id_;
  //   centroids.header.stamp = ros::Time (0);
  //   // centroids.point = j;
  //   centroids.point.x = centroid_in[0];
  //   centroids.point.y = centroid_in[1];
  //   centroids.point.z = centroid_in[2];
  //   // findCylPose ((*cloud_cluster)[j]);
  //   // pubFilteredPCMsg (g_pub_cloud, *cloud_cluster);
  //   // findCylPose (cloud_cluster);

  //   // std::list<geometry_msgs::PointStamped> centroid_list;
  //   // centroid_list.push_back(centroids);
  //     // Transform the point to new frame
  //   geometry_msgs::PointStamped g_cyl_pt_msg_out;
  //   try
  //   {
  //     g_listener_.transformPoint ("world",  // bad styling
  //                                 centroids,
  //                                 g_cyl_pt_msg_out);
  //     //ROS_INFO ("trying transform...");
  //   }
  //   catch (tf::TransformException& ex)
  //   {
  //     ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  //   }
 
  //   centroid_list.push_back(g_cyl_pt_msg_out);
  //   publishPose (g_cyl_pt_msg_out);
    
  // } ; 

  //stop

  // std::cout << "Centroid list: " <<  centroid_list; //centroid_list.size();
  
  // std::list<geometry_msgs::PointStamped>::iterator it;
  // for (it = centroid_list.begin(); it != centroid_list.end(); ++it){
  //     // std::cout << it->name;
  //     publishPose (it);
  // }

  // for (auto i: centroid_list)
  //   // auto it = yourList.begin();
  //   // std::advance(it, index);
  //   std::cout << 'centroid list: ' << i ;

  //   publishPose (centroid_list);
  //   // publishPose (centroid_list);

  pubFilteredPCMsg (g_pub_cloud_centroid, *g_cloud_filtered_rgb);
  
  // response = centroid_list;

  for (auto i: centroid_list)
  // auto it = yourList.begin();
  // std::advance(it, index);
    std::cout << g_required_colour <<' centroid list: \n' << i ;
      
  return free;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::task3Callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{

  // g_home_postion = arm_group_.getCurrentJointValues ();


  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  tf2::Quaternion q_x180deg2(-1, 0, 0, 0);
  tf2::Quaternion q_object2;
  q_object2.setRPY(angle_offset2_, angle_offset2_, angle_offset_);
  tf2::Quaternion q_result2 = q_x180deg2 * q_object2;
  geometry_msgs::Quaternion grasp_orientation2 = tf2::toMsg(q_result2);

  geometry_msgs::Point goal_location;
  goal_location = request.goal_loc.point;
  g_box_location = goal_location;
  addCollisionTask3(centroid_list2, goal_location);

  g_task3_pose1.position.x = 0.3;
  g_task3_pose1.position.y = 0;
  g_task3_pose1.position.z = 0.85;
  g_task3_pose1.orientation = grasp_orientation;


  g_task3_pose2.position.x = -0.3;
  g_task3_pose2.position.y = 0;
  g_task3_pose2.position.z = 0.85;
  g_task3_pose2.orientation = grasp_orientation;


  g_task3_pose_test.position.x = 0.05;
  g_task3_pose_test.position.y = 0;
  g_task3_pose_test.position.z = 0.65;
  g_task3_pose_test.orientation = grasp_orientation2;
  g_task3_pose_test.position.z += z_offset_;


  if (g_r.data ==10 && g_g.data == 10 && g_b.data == 80) {
  ROS_INFO("Colour: blue");
  g_required_colour = "blue";
  // g_hue = 42;
  // g_hue = 127;
  } 
  else if (g_r.data ==80 && g_g.data == 10 && g_b.data == 80) {
  ROS_INFO("Colour: Purple");
  g_required_colour = "purple";
  // g_hue = 84.5;
  // g_hue = 127;
  } 
  else if (g_r.data ==80 && g_g.data == 10 && g_b.data == 10) {
  ROS_INFO("Colour: red");
  g_required_colour = "red";
  // g_hue = -127;
  // g_hue = 50;
  } 

  g_r.data = request.r.data * 100;
  g_g.data = request.g.data * 100;
  g_b.data = request.b.data * 100;

  // moveArm(g_task3_pose_test);

  moveArm(g_task3_pose1);


  euclideanCluster (g_cloud_filtered_rgb);

  // pubFilteredPCMsg (g_pub_cloud_centroid, *g_cloud_filtered_rgb);

  while (! centroid_list.empty()){ 
    for (auto i: centroid_list){
    // auto it = yourList.begin();
    // std::advance(it, index);
      std::cout << 'centroid list: \n' << i ;

      geometry_msgs::Point object_location;
      object_location = i.point;
      pick(object_location);

      geometry_msgs::Point goal_location;
      goal_location = request.goal_loc.point;
      drop(goal_location);
      moveArm(g_task3_pose1);
      // euclideanCluster (g_cloud_filtered_rgb);
      }

    // applyCropHull (g_cloud_filtered_rgb,
    //   goal_location, g_cloud_crop_hull);
      // applyCropHull (g_cloud_filtered_rgb,
      //   goal_location, );

    // applyCropHull (g_cloud_filtered_rgb); // idk
    moveArm(g_task3_pose1);
    euclideanCluster (g_cloud_filtered_rgb);
  }

  // applyCropHull (g_cloud_filtered_rgb,
  //     goal_location, g_cloud_crop_hull);

  moveArm(g_task3_pose2);

  euclideanCluster (g_cloud_filtered_rgb);

  while (! centroid_list.empty()){ 
    for (auto i: centroid_list){
    // auto it = yourList.begin();
    // std::advance(it, index);
      std::cout << 'centroid list: \n' << i ;

      geometry_msgs::Point object_location;
      object_location = i.point;
      pick(object_location);

      drop(goal_location);
      moveArm(g_task3_pose2);
      // euclideanCluster (g_cloud_filtered_rgb);
      }
    euclideanCluster (g_cloud_filtered_rgb);
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
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = arm_group_.computeCartesianPath(waypoints, 
    eef_step, jump_threshold, trajectory);

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

///////////////////////////////////////////////////////////////////////////////

void
CW1::colourFilter(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{

  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;

  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_rgb_cloud);
  applyVX (g_rgb_cloud, g_rgb_cloud);
  segCube (g_rgb_cloud);
  
  filterPurpleRedCubes(g_rgb_cloud);
  // applyCropHull (g_rgb_cloud); // idk

  pcl::ConditionalRemoval<pcl::PointXYZRGBA> color_filter;

  pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr
      red_condition_lb(new pcl::PackedRGBComparison<pcl::PointXYZRGBA>("r", pcl::ComparisonOps::GE, g_r.data-10));
  pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr
      red_condition_ub(new pcl::PackedRGBComparison<pcl::PointXYZRGBA>("r", pcl::ComparisonOps::LE, g_r.data+10));

  pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr
      green_condition_lb(new pcl::PackedRGBComparison<pcl::PointXYZRGBA>("g", pcl::ComparisonOps::GE, g_g.data-10));
  pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr
      green_condition_ub(new pcl::PackedRGBComparison<pcl::PointXYZRGBA>("g", pcl::ComparisonOps::LE, g_g.data+10));

  pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr
      blue_condition_lb(new pcl::PackedRGBComparison<pcl::PointXYZRGBA>("b", pcl::ComparisonOps::GE, g_b.data-5));
  pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr
      blue_condition_ub(new pcl::PackedRGBComparison<pcl::PointXYZRGBA>("b", pcl::ComparisonOps::LE, g_b.data+5));

  // pcl::PackedHSIComparison<pcl::PointXYZRGBA>::Ptr
      // hue_condition_lb(new pcl::PackedHSIComparison<pcl::PointXYZRGBA>("h", pcl::ComparisonOps::GE, g_hue-10));

  // pcl::PackedHSIComparison<pcl::PointXYZRGBA>::Ptr
  //     hue_condition_ub(new pcl::PackedHSIComparison<pcl::PointXYZRGBA>("h", pcl::ComparisonOps::LE, g_hue));
    
  pcl::ConditionAnd<pcl::PointXYZRGBA>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGBA> ());
  color_cond->addComparison (red_condition_lb);
  color_cond->addComparison (red_condition_ub);
  color_cond->addComparison (green_condition_lb);
  color_cond->addComparison (green_condition_ub);
  color_cond->addComparison (blue_condition_lb);
  color_cond->addComparison (green_condition_ub);
  // color_cond->addComparison (hue_condition_lb);
  // color_cond->addComparison (hue_condition_ub);

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
// CW1::applyCropHull (PointCPtr &in_cloud_ptr,
//       geometry_msgs::Point &goal_location,
//       PointCPtr &out_cloud_ptr)
CW1::applyCropHull (PointCPtr &in_cloud_ptr)
{
  // pcl::CropHull<pcl::PointXYZ> cropHullFilter;
  // boost::shared_ptr<PointCloud> hullCloud(new PointCloud());
  // boost::shared_ptr<PointCloud> hullPoints(new PointCloud());
  // std::vector<Vertices> hullPolygons;

  // hullCloud->clear();

  // goal_location


  pcl::PointXYZRGBA p1;
  p1.x = -g_box_location.x + 0.1;
  p1.y = g_box_location.y - 0.1;
  p1.z = g_box_location.z ;
  // hullCloud->push_back(p1);  }

  pcl::PointXYZRGBA p2;
  p2.x = -g_box_location.x + 0.1;
  p2.y = g_box_location.y + 0.1;
  p2.z = g_box_location.z ;
  // hullCloud->push_back(p);

  pcl::PointXYZRGBA p3;
  p3.x = -g_box_location.x - 0.1;
  p3.y = g_box_location.y - 0.1;
  p3.z = g_box_location.z ;
  // hullCloud->push_back(p);

  pcl::PointXYZRGBA p4;
  p4.x = -g_box_location.x - 0.1;
  p4.y = g_box_location.y + 0.1;
  p4.z = g_box_location.z ;
  // hullCloud->push_back(p);

  pcl::PointXYZRGBA p5;
  p5.x = -g_box_location.x + 0.1;
  p5.y = g_box_location.y - 0.1;
  p5.z = g_box_location.z - 0.2;
  // hullCloud->push_back(p1);

  pcl::PointXYZRGBA p6;
  p6.x = -g_box_location.x + 0.1;
  p6.y = g_box_location.y + 0.1;
  p6.z = g_box_location.z - 0.2;
  // hullCloud->push_back(p);

  pcl::PointXYZRGBA p7;
  p7.x = -g_box_location.x - 0.1;
  p7.y = g_box_location.y - 0.1;
  p7.z = g_box_location.z - 0.2;
  // hullCloud->push_back(p);

  pcl::PointXYZRGBA p8;
  p8.x = -g_box_location.x - 0.1;
  p8.y = g_box_location.y + 0.1;
  p8.z = g_box_location.z - 0.2;
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
  hull_calculator.setAlpha (20);
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
  
  // g_crop_hull.setHullCloud (in_cloud_ptr);
  // g_crop_hull.setHullIndices (polygons);
  // g_crop_hull.setDim (dim);
  // g_crop_hull.setCropOutside (false);
  // g_crop_hull.filter (*out_cloud_ptr);



  // // setup hull filter
  // pcl::ConvexHull<pcl::PointXYZ> cHull;
  // cHull.setInputCloud(hullCloud);
  // cHull.reconstruct(*hullPoints, hullPolygons);

  // g_crop_hull.setHullIndices(hullPolygons);
  // g_crop_hull.setHullCloud(hullPoints);
  // //cropHullFilter.setDim(2); // if you uncomment this, it will work
  // g_crop_hull.setCropOutside(false); // this will remove points inside the hull




  // g_crop_hull.setHullCloud (in_cloud_ptr);
  // g_crop_hull.setHullIndices (polygons);
  // g_crop_hull.setCropOutside (false);
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::applyPT (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("x");
  g_pt.setFilterLimits (g_pt_thrs_min, g_pt_thrs_max);
  g_pt.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::findNormals (PointCPtr &in_cloud_ptr)
{
  // Estimate point normals
  g_ne.setInputCloud (in_cloud_ptr);
  g_ne.setSearchMethod (g_tree_ptr);
  g_ne.setKSearch (g_k_nn);
  g_ne.compute (*g_cloud_normals);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::segPlane (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for the planar model
  // and set all the params
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  g_seg.setNormalDistanceWeight (0.1); //bad style
  g_seg.setMethodType (pcl::SAC_RANSAC);
  g_seg.setMaxIterations (100); //bad style
  g_seg.setDistanceThreshold (0.001); //bad style 0.03
  g_seg.setInputCloud (in_cloud_ptr);
  g_seg.setInputNormals (g_cloud_normals);
  // Obtain the plane inliers and coefficients
  g_seg.segment (*g_inliers_plane, *g_coeff_plane);
  
  // Extract the planar inliers from the input cloud
  g_extract_pc.setInputCloud (in_cloud_ptr);
  g_extract_pc.setIndices (g_inliers_plane);
  g_extract_pc.setNegative (false);
  
  // Write the planar inliers to disk
  g_extract_pc.filter (*g_cloud_plane);
  
  // Remove the planar inliers, extract the rest
  g_extract_pc.setNegative (true);
  g_extract_pc.filter (*g_cloud_filtered2);
  g_extract_normals.setNegative (true);
  g_extract_normals.setInputCloud (g_cloud_normals);
  g_extract_normals.setIndices (g_inliers_plane);
  g_extract_normals.filter (*g_cloud_normals2);

  //ROS_INFO_STREAM ("Plane coefficients: " << *g_coeff_plane);
  ROS_INFO_STREAM ("PointCloud representing the planar component: "
                   << g_cloud_plane->size ()
                   << " data points.");
}
    
////////////////////////////////////////////////////////////////////////////////
void
CW1::segCylind (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for cylinder segmentation
  // and set all the parameters
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_CYLINDER);
  g_seg.setMethodType (pcl::SAC_RANSAC);
  g_seg.setNormalDistanceWeight (0.1); //bad style
  g_seg.setMaxIterations (10000); //bad style
  g_seg.setDistanceThreshold (0.05); //bad style
  g_seg.setRadiusLimits (0, 0.1); //bad style
  g_seg.setInputCloud (g_cloud_filtered2);
  g_seg.setInputNormals (g_cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  g_seg.segment (*g_inliers_cylinder, *g_coeff_cylinder);
  
  // Write the cylinder inliers to disk
  g_extract_pc.setInputCloud (g_cloud_filtered2);
  g_extract_pc.setIndices (g_inliers_cylinder);
  g_extract_pc.setNegative (false);
  g_extract_pc.filter (*g_cloud_cylinder);
  
  ROS_INFO_STREAM ("PointCloud representing the cylinder component: "
                   << g_cloud_cylinder->size ()
                   << " data points.");
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::segCube (PointCPtr &in_cloud_ptr)
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
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

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
CW1::findCylPose (PointCPtr &in_cloud_ptr)
{
  Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*in_cloud_ptr, centroid_in);
  
  g_cyl_pt_msg.header.frame_id = g_input_pc_frame_id_;
  g_cyl_pt_msg.header.stamp = ros::Time (0);
  g_cyl_pt_msg.point.x = centroid_in[0];
  g_cyl_pt_msg.point.y = centroid_in[1];
  g_cyl_pt_msg.point.z = centroid_in[2];
  
  // Transform the point to new frame
  geometry_msgs::PointStamped g_cyl_pt_msg_out;
  try
  {
    g_listener_.transformPoint ("panda_link0",  // bad styling
                                g_cyl_pt_msg,
                                g_cyl_pt_msg_out);
    //ROS_INFO ("trying transform...");
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }
  
  publishPose (g_cyl_pt_msg_out);
  
  return;
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
CW1::publishPose (geometry_msgs::PointStamped &cyl_pt_msg)
{
  // Create and publish the cylinder pose (ignore orientation)

  g_pub_pose.publish (cyl_pt_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::pubFilteredRGB (ros::Publisher &pc_pub,
                               PointE &pc)
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
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (in_cloud_ptr);
  pcl::PCDWriter writer;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.05); // 2cm
  ec.setMinClusterSize (1);
  ec.setMaxClusterSize (20);
  ec.setSearchMethod (tree);
  ec.setInputCloud (in_cloud_ptr);
  ec.extract (cluster_indices);
  
  // for (auto i: cluster_indices)
  //   std::cout << i << ' ';
  // std::cout << "cluster_indices: " << cluster_indices; 
  
  // pcl::PointCloud<PointT>::Ptr cloud_cluster (new PointE);
  centroid_list.clear();
  int j = 1;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new PointE);
    for (const auto& idx : it->indices)
      cloud_cluster->push_back ((*in_cloud_ptr)[idx]); //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << g_required_colour << " cloud cluster " << j << "\n";
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points.\n" << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
    j++;

    geometry_msgs::PointStamped centroids;
    Eigen::Vector4f centroid_in;
    pcl::compute3DCentroid(*cloud_cluster, centroid_in);

    centroids.header.frame_id = g_input_pc_frame_id_;
    centroids.header.stamp = ros::Time (0);
    // centroids.point = j;
    centroids.point.x = centroid_in[0];
    centroids.point.y = centroid_in[1];
    centroids.point.z = centroid_in[2];
    // findCylPose ((*cloud_cluster)[j]);
    // pubFilteredPCMsg (g_pub_cloud, *cloud_cluster);
    // findCylPose (cloud_cluster);

    // std::list<geometry_msgs::PointStamped> centroid_list;
    // centroid_list.push_back(centroids);
      // Transform the point to new frame
    geometry_msgs::PointStamped g_cyl_pt_msg_out;
    try
    {
      g_listener_.transformPoint ("world",  // bad styling
                                  centroids,
                                  g_cyl_pt_msg_out);
      //ROS_INFO ("trying transform...");
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
    }
 
    centroid_list.push_back(g_cyl_pt_msg_out);
    publishPose (g_cyl_pt_msg_out);


  } ; 

  // for (auto i: centroid_list)
  // // auto it = yourList.begin();
  // // std::advance(it, index);
  //   std::cout << 'centroid list: ' << i ;

  // if (centroid_list.empty())
  //     std::cout << "No" << g_required_colour << " centroid found\n";

  return;

}

////////////////////////////////////////////////////////////////////////////////

void
CW1::addCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  /* add a collision object in RViz and the MoveIt planning scene */

  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object 
  // hint: what about collision_object.REMOVE?
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

void
CW1::filterPurpleRedCubes(PointCPtr &red_purple_cloud)
{

  pcl::ConditionalRemoval<pcl::PointXYZRGBA> color_filter_red_purple;

  pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr
      red_purple_r_lb(new pcl::PackedRGBComparison<pcl::PointXYZRGBA>("r", pcl::ComparisonOps::GE, g_red_purple_cube_r-10));
  pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr
      red_purple_r_ub(new pcl::PackedRGBComparison<pcl::PointXYZRGBA>("r", pcl::ComparisonOps::LE, g_red_purple_cube_r+10));

  pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr
      red_purple_g_lb(new pcl::PackedRGBComparison<pcl::PointXYZRGBA>("g", pcl::ComparisonOps::GE, g_red_purple_cube_g-10));
  pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr
      red_purple_g_ub(new pcl::PackedRGBComparison<pcl::PointXYZRGBA>("g", pcl::ComparisonOps::LE, g_red_purple_cube_g+10));

  pcl::ConditionAnd<pcl::PointXYZRGBA>::Ptr color_cond_red_purple (new pcl::ConditionAnd<pcl::PointXYZRGBA> ());
  color_cond_red_purple->addComparison (red_purple_r_lb);
  color_cond_red_purple->addComparison (red_purple_r_ub);
  color_cond_red_purple->addComparison (red_purple_g_lb);
  color_cond_red_purple->addComparison (red_purple_g_ub);

  // Build the filter
  color_filter_red_purple.setInputCloud(red_purple_cloud);
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
  pcl::PCDWriter writer;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.05); // 2cm
  ec.setMinClusterSize (1);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (in_cloud_ptr);
  ec.extract (cluster_indices);
  
  centroid_list2.clear();
  int j = 1;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster2 (new PointE);
    for (const auto& idx : it->indices)
      cloud_cluster2->push_back ((*in_cloud_ptr)[idx]); //*
    cloud_cluster2->width = cloud_cluster2->size ();
    cloud_cluster2->height = 1;
    cloud_cluster2->is_dense = true;

    std::cout << "Red/Purple cluster " << j << "\n";
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster2->size () << " data points.\n" << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster2_" << j << ".pcd";
    writer.write<PointT> (ss.str (), *cloud_cluster2, false); //*
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
      g_listener2_.transformPoint ("world",  // bad styling
                                  centroids_rpc,
                                  g_rpc_pt_msg);
      //ROS_INFO ("trying transform...");
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
    }
 
    centroid_list2.push_back(g_rpc_pt_msg);

  } ; 

  for (auto i: centroid_list2)
  // auto it = yourList.begin();
  // std::advance(it, index);
    std::cout << 'Non target cubes centroid list: \n' << i ;

  // if (centroid_list2.empty())
  //     std::cout << "No Red/Purple centroid found\n";

  return;

}

void 
CW1::addCollisionTask3 (std::list<geometry_msgs::PointStamped> 
  &red_purple_cubes, geometry_msgs::Point &goal_location)

{


  geometry_msgs::Quaternion box_orientation;
  box_orientation.x = 0;
  box_orientation.y = 0;
  box_orientation.z = 0;
  box_orientation.w = -1;



  std::string floor = "floor";
  geometry_msgs::Point floor_centre;
  geometry_msgs::Vector3 floor_dimensions;
  floor_centre.x = 0;
  floor_centre.y = 0;
  floor_centre.z = 0;
  floor_dimensions.x = 5;
  floor_dimensions.y = 5;
  floor_dimensions.z = 0.055;

  std::string box = "box";
  geometry_msgs::Point box_centre;
  geometry_msgs::Vector3 box_dimensions;

  box_centre.x = goal_location.x ;
  box_centre.y = goal_location.y ;
  box_centre.z = goal_location.z -0.1;
  box_dimensions.x = 0.2;
  box_dimensions.y = 0.2;
  box_dimensions.z = 0.2;

  redPurpleCluster (g_red_purple_cube_cloud);

  std::string cube = "cube";
  geometry_msgs::Vector3 cube_dimensions;
  cube_dimensions.x = 0.04;
  cube_dimensions.y = 0.04;
  cube_dimensions.z = 0.04;

  for (auto i: centroid_list2){
    geometry_msgs::Point cube_centre;
    geometry_msgs::Point cube_location;
    // cube_centre.clear();
    cube_centre.x = i.point.x ;
    cube_centre.y = i.point.y ;
    cube_centre.z = i.point.z ;
    addCollisionObject(cube, cube_centre, cube_dimensions,
    box_orientation);
    ROS_INFO("purple red cube collision added");
    }

  addCollisionObject(floor, floor_centre, floor_dimensions,
    box_orientation);

  addCollisionObject(box, box_centre, box_dimensions,
    box_orientation);


  return;
}