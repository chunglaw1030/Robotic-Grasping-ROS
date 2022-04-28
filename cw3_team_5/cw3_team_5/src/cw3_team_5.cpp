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

#include <cw3_team_5/cw3_team_5.h>

///////////////////////////////////////////////////////////////////////////////

// typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

CW3::CW3(ros::NodeHandle &nh):
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_floor (new PointC),
  g_cloud_filtered_out_floor (new PointC),
  g_cloud_filtered_colour (new PointC),
  g_cloud_filtered_stack (new PointC),
  g_cloud_filtered_stack_rgb (new PointC),
  g_cloud_filtered_out_stack (new PointC),
  g_cloud_filtered_out_stack_rgb (new PointC),
  g_cloud_plane (new PointC), // plane point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  g_tree_cluster_ptr (new pcl::search::KdTree<PointT>), // Euclidean Cluster KdTree
  g_cloud_normal (new pcl::PointCloud<pcl::PointNormal>),
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
  std::string service_ns = "/CW3";

  task1_srv_ = g_nh.advertiseService("/task1_start",
    &CW3::task1Callback, this);  
  task2_srv_ = g_nh.advertiseService("/task2_start",
    &CW3::task2Callback, this);  
  task3_srv_ = g_nh.advertiseService("/task3_start",
    &CW3::task3Callback, this);  


  ROS_INFO("MoveIt! services initialisation finished, namespace: %s", 
    service_ns.c_str());

    // Define the publishers
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud_no_floor", 5, true);
  // g_pub_norm = nh.advertise<sensor_msgs::PointCloud2> ("Normals", 1, true);
  // g_pub_norm = nh.advertise<pcl::PointCloud<pcl::PointNormal>>("normal_pointcloud", 1);
  // g_pub_norm = nh.advertise<pcl::PointCloud<pcl::Normal>>("normal_pointcloud", 1);
  g_pub_pose = nh.advertise<geometry_msgs::PoseStamped> ("centroid_pose", 5, true);
  g_pub_pose_test = nh.advertise<geometry_msgs::PoseStamped> ("test_pose", 5, true);
  // g_pub_cloud_centroid = nh.advertise<sensor_msgs::PointCloud2> ("centroid_cloud", 1, true);
  g_pub_plane = nh.advertise<sensor_msgs::PointCloud2> ("Plane", 1, true);
  g_pub_cloud_filtered2 = nh.advertise<sensor_msgs::PointCloud2> ("cloud_filtered2", 5, true);
  // g_pub_purple_red = nh.advertise<sensor_msgs::PointCloud2> ("purple_red_cubes", 1, true);
  // g_pub_crop_hull = nh.advertise<sensor_msgs::PointCloud2> ("crop_hull", 1, true);
  g_pub_crop_stack = nh.advertise<sensor_msgs::PointCloud2> ("cropped_stack", 5, true);
  g_pub_crop_stack_rgb = nh.advertise<sensor_msgs::PointCloud2> ("cropped_stack_rgb", 5, true);
  g_pub_crop_out_stack_rgb = nh.advertise<sensor_msgs::PointCloud2> ("cropped_out_stack_rgb", 5, true);
  g_pub_point = nh.advertise<geometry_msgs::PointStamped> ("centroid", 5, true);
  g_pub_stack_point = nh.advertise<geometry_msgs::PointStamped> ("stack_point", 5, true);
  g_pub_colour_filtered = nh.advertise<sensor_msgs::PointCloud2> ("Colour_filtered", 5, true);
}

///////////////////////////////////////////////////////////////////////////////

bool
CW3::task1Callback(cw3_world_spawner::Task1Service::Request &request,
  cw3_world_spawner::Task1Service::Response &response)
{
  /* Task 1 callback, this fnction moves to a scan position and finds the 
      angle, position and colour of the stack of cube*/
  
  moveJointPosition(g_home_postion);
  
  bool scan_front = true;
  moveScanPosition(scan_front);

  std::this_thread::sleep_for(std::chrono::seconds(3));

  findNormals(g_cloud_filtered_out_floor);
  
  ros::Duration(0.5).sleep();
    
  segPlane(g_cloud_filtered_out_floor);
  
  ros::Duration(2).sleep();
  
  std::cout << " Number of Normals " << g_cloud_normals2->points.size() << "\n";

  ros::Duration(1).sleep();

  // Visualise one of the normals
  Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*g_cloud_filtered_out_floor, centroid_in);

  double angle_x = g_cloud_normals2->points[1].normal_x;
  double angle_y = g_cloud_normals2->points[1].normal_y;
  double angle_z = g_cloud_normals2->points[1].normal_z;

  findCubeAngle (angle_x, angle_y, angle_z);
  response.stack_rotation = (float) g_stack_rotation;

  tf2::Quaternion test;
  test.setRPY(angle_x, angle_y, angle_z);
  geometry_msgs::Quaternion testMsg;
  testMsg = tf2::toMsg(test);

  geometry_msgs::PoseStamped testPose;
  testPose.header.frame_id = g_input_pc_frame_id_;
  testPose.header.stamp = ros::Time (0);
  testPose.pose.position.x = centroid_in[0];
  testPose.pose.position.y = centroid_in[1];
  testPose.pose.position.z = centroid_in[2];
  testPose.pose.orientation = testMsg;
  geometry_msgs::PoseStamped testPoseOut;

  try
  {
    g_listener_.transformPose (g_target_frame,  
                                testPose,
                                testPoseOut);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }

  g_pub_pose_test.publish(testPoseOut);

  std::cout << "x: " << testPoseOut.pose.position.x  << std::endl;
  std::cout << "y: " << testPoseOut.pose.position.y  << std::endl;
  std::cout << "z: " << testPoseOut.pose.position.z  << std::endl;

  response.stack_point.x = testPoseOut.pose.position.x;
  response.stack_point.y = testPoseOut.pose.position.y;
  response.stack_point.z = stack_point_z;

  //////////////// Using Color filter ///////////////////////////


  std::multimap< double, std::string, std::less<double> > mmapOfColour;

  mmapOfColour = findColourInStack();

  for (std::multimap<double, std::string>::iterator it = mmapOfColour.begin();
          it != mmapOfColour.end(); it++)
  {
    std::cout << it->first << " :: " << it->second << std::endl;

    std_msgs::ColorRGBA colour_of_cubes;
    if (it->second == "blue")
    {
      colour_of_cubes.r = g_blue_r_value/g_rgb_convert;
      colour_of_cubes.g = g_blue_g_value/g_rgb_convert;
      colour_of_cubes.b = g_blue_b_value/g_rgb_convert;
      response.stack_colours.push_back(colour_of_cubes);
    }

    else if (it->second == "purple")
    {
      colour_of_cubes.r = g_purple_r_value/g_rgb_convert;
      colour_of_cubes.g = g_purple_g_value/g_rgb_convert;
      colour_of_cubes.b = g_purple_b_value/g_rgb_convert;
      response.stack_colours.push_back(colour_of_cubes);
    }

    else if (it->second == "red")
    {
      colour_of_cubes.r = g_red_r_value/g_rgb_convert;
      colour_of_cubes.g = g_red_g_value/g_rgb_convert;
      colour_of_cubes.b = g_red_b_value/g_rgb_convert;
      response.stack_colours.push_back(colour_of_cubes);
    }

    else if (it->second == "orange")
    {
      colour_of_cubes.r = g_orange_r_value/g_rgb_convert;
      colour_of_cubes.g = g_orange_g_value/g_rgb_convert;
      colour_of_cubes.b = g_orange_b_value/g_rgb_convert;
      response.stack_colours.push_back(colour_of_cubes);
    }
    
    else if (it->second == "yellow")
    {
      colour_of_cubes.r = g_yellow_r_value/g_rgb_convert;
      colour_of_cubes.g = g_yellow_g_value/g_rgb_convert;
      colour_of_cubes.b = g_yellow_b_value/g_rgb_convert;
      response.stack_colours.push_back(colour_of_cubes);
    }

    else if (it->second == "pink")
    {
      colour_of_cubes.r = g_pink_r_value/g_rgb_convert;
      colour_of_cubes.g = g_pink_g_value/g_rgb_convert;
      colour_of_cubes.b = g_pink_b_value/g_rgb_convert;
      response.stack_colours.push_back(colour_of_cubes);
    }
  }

  return free;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW3::task2Callback(cw3_world_spawner::Task2Service::Request &request,
  cw3_world_spawner::Task2Service::Response &response)

{ 
  /* Task 2 callback, this function scan the ground and finds cubes of required
  colour from the stack_colours list, which are then picked and placed at 
  required location at an angle requesed*/

  moveJointPosition(g_home_postion);
  g_colour_list_q2 =  request.stack_colours; //std::vector<std_msgs::ColorRGBA>
  g_stack_point_q2 =  request.stack_point; //geometry_msgs::Point
  g_stack_rotation_q2 = request.stack_rotation; // float

  g_cube_dimensions.x = g_cube_dimensions_x;
  g_cube_dimensions.y = g_cube_dimensions_y;
  g_cube_dimensions.z = g_cube_dimensions_y;

  int k = 0; 
  for (auto i: g_colour_list_q2)
  {
    g_r.data = i.r * g_rgb_convert;
    g_g.data = i.g * g_rgb_convert;
    g_b.data = i.b * g_rgb_convert;
    colourOfCloud(g_r.data, g_g.data, g_b.data);

    bool scan_front = true;
    moveScanPosition(scan_front);

    
    euclideanCluster (g_cloud_filtered_colour);

    // g_box_orientation.x = g_box_orientation_x;
    // g_box_orientation.y = g_box_orientation_y;
    // g_box_orientation.z = g_box_orientation_z;
    // g_box_orientation.w = g_box_orientation_w;
    


    // std::this_thread::sleep_for(std::chrono::seconds(1));

    geometry_msgs::Point object_location;
    object_location = g_centroid_list.front().point;

    double pick_orientation;
    pick_orientation = g_angle_list.front();

    pick(object_location, pick_orientation); //g_stack_rotation

    g_stack_point_q2.z = g_stack_point_q2.z + k*0.04;

    drop(g_stack_point_q2, (double) g_stack_rotation_q2);

    k += 1;

  } 

  moveJointPosition(g_home_postion);

  
  // for (auto i: g_centroid_list)
  //   response.centroids.push_back(i);
  
  // for (auto i: response.centroids)
  //   std::cout <<  "response centroid list: \n" << i ;

  // if (g_centroid_list.empty())
  // //   std::cout <<  "No "<< g_required_colour <<" cubes found! \n";

  return free;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW3::task3Callback(cw3_world_spawner::Task3Service::Request &request,
  cw3_world_spawner::Task3Service::Response &response)
{
  /* Task 3 callback, this funciton first scans the ground at 4 positions to 
      find the stack, which is done by finding the centroids of the cubes. The 
      centroid with highest z-position is the top cube of the stack. With the 
      location of the stack, we can then filter out everything else except area
      around the stack using FieldComparision. We then repeat procedures in 
      task 1 to find angle and colour of stack. Finally, to replicate the stack, 
      we scan around the ground to find cubes with colour required. Pick and
      place at location requested and at an angle same as stack*/
  moveJointPosition(g_home_postion);
  

  std::this_thread::sleep_for(std::chrono::seconds(3));

  // first find the tallest point 

  g_tallest_point.x = tallest_point_init;
  g_tallest_point.y = tallest_point_init;
  g_tallest_point.z = tallest_point_init;

  ROS_INFO("Finding stack");
  moveScanPositionQ3(1);
  findTallestStack();
  std::cout << "x" << g_tallest_point.x << std::endl;
  std::cout << "y" << g_tallest_point.y << std::endl;
  std::cout << "z" << g_tallest_point.z << std::endl;
  moveScanPositionQ3(3);
  findTallestStack();
  std::cout << "x" << g_tallest_point.x << std::endl;
  std::cout << "y" << g_tallest_point.y << std::endl;
  std::cout << "z" << g_tallest_point.z << std::endl;
  moveScanPositionQ3(4);
  findTallestStack();
  std::cout << "x" << g_tallest_point.x << std::endl;
  std::cout << "y" << g_tallest_point.y << std::endl;
  std::cout << "z" << g_tallest_point.z << std::endl;
  moveScanPositionQ3(2);
  findTallestStack();
  std::cout << "x" << g_tallest_point.x << std::endl;
  std::cout << "y" << g_tallest_point.y << std::endl;
  std::cout << "z" << g_tallest_point.z << std::endl;

  g_stack_point_q3 = g_tallest_point;



  geometry_msgs::PointStamped stackpoint;
  stackpoint.header.frame_id = g_target_frame; //g_input_pc_frame_id_
  stackpoint.header.stamp = ros::Time (0);
  stackpoint.point = g_tallest_point;

  g_pub_stack_point.publish(stackpoint);

  if (g_tallest_point.y<0 && g_tallest_point.x<0.08 && g_tallest_point.x>-0.08)
  moveScanPositionQ3(4);
  else if (g_tallest_point.y>0 && g_tallest_point.x<0.08 && g_tallest_point.x>-0.08)
  moveScanPositionQ3(3);
  else if (g_tallest_point.x>0.08)
  moveScanPositionQ3(1);
  else if (g_tallest_point.y<-0.08)
  moveScanPositionQ3(2);

  std::multimap< double, std::string, std::less<double> > mmapOfColourQ3;

  ROS_INFO("Finding colur of stack");
  mmapOfColourQ3 = findColourInStackQ3();

  for (std::multimap<double, std::string>::iterator it = mmapOfColourQ3.begin();
          it != mmapOfColourQ3.end(); it++)
  {
    std::cout << it->first << " :: " << it->second << std::endl;

  }

  ROS_INFO("Finding angle of stack");

  findNormals(g_cloud_filtered_stack);
  
  ros::Duration(0.5).sleep();
    
  segPlane(g_cloud_filtered_stack);
  
  ros::Duration(2).sleep();
  
  std::cout << " Number of Normals " << g_cloud_normals2->points.size() << "\n";

  ros::Duration(1).sleep();

  double angle_x = g_cloud_normals2->points[1].normal_x;
  double angle_y = g_cloud_normals2->points[1].normal_y;
  double angle_z = g_cloud_normals2->points[1].normal_z;

  findCubeAngle (angle_x, angle_y, angle_z);

  g_goal_stack_point_q3 =  request.stack_point;

  int k = 0;

  for (std::multimap<double, std::string>::iterator it = mmapOfColourQ3.begin();
          it != mmapOfColourQ3.end(); it++)
  {
    std::cout << it->first << " :: " << it->second << std::endl;

    if (it->second == "blue")
    {
      g_r.data = g_blue_r_value;
      g_g.data = g_blue_g_value;
      g_b.data = g_blue_b_value;
      colourOfCloud(g_r.data, g_g.data, g_b.data);

      moveScanPositionQ3(1);

      euclideanCluster (g_cloud_filtered_colour);
      ros::Duration(2).sleep();
      if (! g_centroid_list.empty())
      {
        geometry_msgs::Point object_location;
        object_location = g_centroid_list.front().point;
        double pick_orientation;
        pick_orientation = g_angle_list.front();
        pick(object_location, pick_orientation); 
        g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
        drop(g_goal_stack_point_q3, (double) g_stack_rotation);
      }
      else if (g_centroid_list.empty())
      {
        moveScanPositionQ3(2);
        euclideanCluster (g_cloud_filtered_colour);
        ros::Duration(2).sleep();
        if (! g_centroid_list.empty())
        {
          geometry_msgs::Point object_location;
          object_location = g_centroid_list.front().point;
          double pick_orientation;
          pick_orientation = g_angle_list.front();
          pick(object_location, pick_orientation); 
          g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
          drop(g_goal_stack_point_q3, (double) g_stack_rotation);
        }
        else if (g_centroid_list.empty())
        {
          moveScanPositionQ3(3);
          euclideanCluster (g_cloud_filtered_colour);
          ros::Duration(2).sleep();
          if (! g_centroid_list.empty())
          {
            geometry_msgs::Point object_location;
            object_location = g_centroid_list.front().point;
            double pick_orientation;
            pick_orientation = g_angle_list.front();
            pick(object_location, pick_orientation); 
            g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
            drop(g_goal_stack_point_q3, (double) g_stack_rotation);
          }
          else if (g_centroid_list.empty())
          {
            moveScanPositionQ3(4);
            euclideanCluster (g_cloud_filtered_colour);
            ros::Duration(2).sleep();
            if (! g_centroid_list.empty())
            {
              geometry_msgs::Point object_location;
              object_location = g_centroid_list.front().point;
              double pick_orientation;
              pick_orientation = g_angle_list.front();
              pick(object_location, pick_orientation); 
              g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
              drop(g_goal_stack_point_q3, (double) g_stack_rotation);
            }
          }
        }
      }

    }

    else if (it->second == "purple")
    {
      g_r.data = g_purple_r_value;
      g_g.data = g_purple_g_value;
      g_b.data = g_purple_b_value;
      colourOfCloud(g_r.data, g_g.data, g_b.data);

      moveScanPositionQ3(1);

      euclideanCluster (g_cloud_filtered_colour);
      ros::Duration(2).sleep();
      if (! g_centroid_list.empty())
      {
        geometry_msgs::Point object_location;
        object_location = g_centroid_list.front().point;
        double pick_orientation;
        pick_orientation = g_angle_list.front();
        pick(object_location, pick_orientation); 
        g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
        drop(g_goal_stack_point_q3, (double) g_stack_rotation);
      }
      else if (g_centroid_list.empty())
      {
        moveScanPositionQ3(2);
        euclideanCluster (g_cloud_filtered_colour);
        ros::Duration(2).sleep();
        if (! g_centroid_list.empty())
        {
          geometry_msgs::Point object_location;
          object_location = g_centroid_list.front().point;
          double pick_orientation;
          pick_orientation = g_angle_list.front();
          pick(object_location, pick_orientation); 
          g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
          drop(g_goal_stack_point_q3, (double) g_stack_rotation);
        }
        else if (g_centroid_list.empty())
        {
          moveScanPositionQ3(3);
          euclideanCluster (g_cloud_filtered_colour);
          ros::Duration(2).sleep();
          if (! g_centroid_list.empty())
          {
            geometry_msgs::Point object_location;
            object_location = g_centroid_list.front().point;
            double pick_orientation;
            pick_orientation = g_angle_list.front();
            pick(object_location, pick_orientation); 
            g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
            drop(g_goal_stack_point_q3, (double) g_stack_rotation);
          }
          else if (g_centroid_list.empty())
          {
            moveScanPositionQ3(4);
            euclideanCluster (g_cloud_filtered_colour);
            ros::Duration(2).sleep();
            if (! g_centroid_list.empty())
            {
              geometry_msgs::Point object_location;
              object_location = g_centroid_list.front().point;
              double pick_orientation;
              pick_orientation = g_angle_list.front();
              pick(object_location, pick_orientation); 
              g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
              drop(g_goal_stack_point_q3, (double) g_stack_rotation);
            }
          }
        }
      }
    }

    else if (it->second == "red")
    {
      g_r.data = g_red_r_value;
      g_g.data = g_red_g_value;
      g_b.data = g_red_b_value;
      colourOfCloud(g_r.data, g_g.data, g_b.data);

      moveScanPositionQ3(1);

      euclideanCluster (g_cloud_filtered_colour);
      ros::Duration(2).sleep();
      if (! g_centroid_list.empty())
      {
        geometry_msgs::Point object_location;
        object_location = g_centroid_list.front().point;
        double pick_orientation;
        pick_orientation = g_angle_list.front();
        pick(object_location, pick_orientation); 
        g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
        drop(g_goal_stack_point_q3, (double) g_stack_rotation);
      }
      else if (g_centroid_list.empty())
      {
        moveScanPositionQ3(2);
        euclideanCluster (g_cloud_filtered_colour);
        ros::Duration(2).sleep();
        if (! g_centroid_list.empty())
        {
          geometry_msgs::Point object_location;
          object_location = g_centroid_list.front().point;
          double pick_orientation;
          pick_orientation = g_angle_list.front();
          pick(object_location, pick_orientation); 
          g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
          drop(g_goal_stack_point_q3, (double) g_stack_rotation);
        }
        else if (g_centroid_list.empty())
        {
          moveScanPositionQ3(3);
          euclideanCluster (g_cloud_filtered_colour);
          ros::Duration(2).sleep();
          if (! g_centroid_list.empty())
          {
            geometry_msgs::Point object_location;
            object_location = g_centroid_list.front().point;
            double pick_orientation;
            pick_orientation = g_angle_list.front();
            pick(object_location, pick_orientation); 
            g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
            drop(g_goal_stack_point_q3, (double) g_stack_rotation);
          }
          else if (g_centroid_list.empty())
          {
            moveScanPositionQ3(4);
            euclideanCluster (g_cloud_filtered_colour);
            ros::Duration(2).sleep();
            if (! g_centroid_list.empty())
            {
              geometry_msgs::Point object_location;
              object_location = g_centroid_list.front().point;
              double pick_orientation;
              pick_orientation = g_angle_list.front();
              pick(object_location, pick_orientation); 
              g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
              drop(g_goal_stack_point_q3, (double) g_stack_rotation);
            }
          }
        }
      }
    }

    else if (it->second == "orange")
    {
      g_r.data = g_orange_r_value;
      g_g.data = g_orange_g_value;
      g_b.data = g_orange_b_value;
      colourOfCloud(g_r.data, g_g.data, g_b.data);

      moveScanPositionQ3(1);

      euclideanCluster (g_cloud_filtered_colour);
      ros::Duration(2).sleep();
      if (! g_centroid_list.empty())
      {
        geometry_msgs::Point object_location;
        object_location = g_centroid_list.front().point;
        double pick_orientation;
        pick_orientation = g_angle_list.front();
        pick(object_location, pick_orientation); 
        g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
        drop(g_goal_stack_point_q3, (double) g_stack_rotation);
      }
      else if (g_centroid_list.empty())
      {
        moveScanPositionQ3(2);
        euclideanCluster (g_cloud_filtered_colour);
        ros::Duration(2).sleep();
        if (! g_centroid_list.empty())
        {
          geometry_msgs::Point object_location;
          object_location = g_centroid_list.front().point;
          double pick_orientation;
          pick_orientation = g_angle_list.front();
          pick(object_location, pick_orientation); 
          g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
          drop(g_goal_stack_point_q3, (double) g_stack_rotation);
        }
        else if (g_centroid_list.empty())
        {
          moveScanPositionQ3(3);
          euclideanCluster (g_cloud_filtered_colour);
          ros::Duration(2).sleep();
          if (! g_centroid_list.empty())
          {
            geometry_msgs::Point object_location;
            object_location = g_centroid_list.front().point;
            double pick_orientation;
            pick_orientation = g_angle_list.front();
            pick(object_location, pick_orientation); 
            g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
            drop(g_goal_stack_point_q3, (double) g_stack_rotation);
          }
          else if (g_centroid_list.empty())
          {
            moveScanPositionQ3(4);
            euclideanCluster (g_cloud_filtered_colour);
            ros::Duration(2).sleep();
            if (! g_centroid_list.empty())
            {
              geometry_msgs::Point object_location;
              object_location = g_centroid_list.front().point;
              double pick_orientation;
              pick_orientation = g_angle_list.front();
              pick(object_location, pick_orientation); 
              g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
              drop(g_goal_stack_point_q3, (double) g_stack_rotation);
            }
          }
        }
      }
    }

    
    else if (it->second == "yellow")
    {
      g_r.data = g_yellow_r_value;
      g_g.data = g_yellow_g_value;
      g_b.data = g_yellow_b_value;
      colourOfCloud(g_r.data, g_g.data, g_b.data);

      moveScanPositionQ3(1);

      euclideanCluster (g_cloud_filtered_colour);
      ros::Duration(2).sleep();
      if (! g_centroid_list.empty())
      {
        geometry_msgs::Point object_location;
        object_location = g_centroid_list.front().point;
        double pick_orientation;
        pick_orientation = g_angle_list.front();
        pick(object_location, pick_orientation); 
        g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
        drop(g_goal_stack_point_q3, (double) g_stack_rotation);
      }
      else if (g_centroid_list.empty())
      {
        moveScanPositionQ3(2);
        euclideanCluster (g_cloud_filtered_colour);
        ros::Duration(2).sleep();
        if (! g_centroid_list.empty())
        {
          geometry_msgs::Point object_location;
          object_location = g_centroid_list.front().point;
          double pick_orientation;
          pick_orientation = g_angle_list.front();
          pick(object_location, pick_orientation); 
          g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
          drop(g_goal_stack_point_q3, (double) g_stack_rotation);
        }
        else if (g_centroid_list.empty())
        {
          moveScanPositionQ3(3);
          euclideanCluster (g_cloud_filtered_colour);
          ros::Duration(2).sleep();
          if (! g_centroid_list.empty())
          {
            geometry_msgs::Point object_location;
            object_location = g_centroid_list.front().point;
            double pick_orientation;
            pick_orientation = g_angle_list.front();
            pick(object_location, pick_orientation); 
            g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
            drop(g_goal_stack_point_q3, (double) g_stack_rotation);
          }
          else if (g_centroid_list.empty())
          {
            moveScanPositionQ3(4);
            euclideanCluster (g_cloud_filtered_colour);
            ros::Duration(2).sleep();
            if (! g_centroid_list.empty())
            {
              geometry_msgs::Point object_location;
              object_location = g_centroid_list.front().point;
              double pick_orientation;
              pick_orientation = g_angle_list.front();
              pick(object_location, pick_orientation); 
              g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
              drop(g_goal_stack_point_q3, (double) g_stack_rotation);
            }
          }
        }
      }
    }

    else if (it->second == "pink")
    {
      g_r.data = g_pink_r_value;
      g_g.data = g_pink_g_value;
      g_b.data = g_pink_b_value;
      colourOfCloud(g_r.data, g_g.data, g_b.data);

      moveScanPositionQ3(1);

      euclideanCluster (g_cloud_filtered_colour);
      ros::Duration(2).sleep();
      if (! g_centroid_list.empty())
      {
        geometry_msgs::Point object_location;
        object_location = g_centroid_list.front().point;
        double pick_orientation;
        pick_orientation = g_angle_list.front();
        pick(object_location, pick_orientation); 
        g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
        drop(g_goal_stack_point_q3, (double) g_stack_rotation);
      }
      else if (g_centroid_list.empty())
      {
        moveScanPositionQ3(2);
        euclideanCluster (g_cloud_filtered_colour);
        ros::Duration(2).sleep();
        if (! g_centroid_list.empty())
        {
          geometry_msgs::Point object_location;
          object_location = g_centroid_list.front().point;
          double pick_orientation;
          pick_orientation = g_angle_list.front();
          pick(object_location, pick_orientation); 
          g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
          drop(g_goal_stack_point_q3, (double) g_stack_rotation);
        }
        else if (g_centroid_list.empty())
        {
          moveScanPositionQ3(3);
          euclideanCluster (g_cloud_filtered_colour);
          ros::Duration(2).sleep();
          if (! g_centroid_list.empty())
          {
            geometry_msgs::Point object_location;
            object_location = g_centroid_list.front().point;
            double pick_orientation;
            pick_orientation = g_angle_list.front();
            pick(object_location, pick_orientation); 
            g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
            drop(g_goal_stack_point_q3, (double) g_stack_rotation);
          }
          else if (g_centroid_list.empty())
          {
            moveScanPositionQ3(4);
            euclideanCluster (g_cloud_filtered_colour);
            ros::Duration(2).sleep();
            if (! g_centroid_list.empty())
            {
              geometry_msgs::Point object_location;
              object_location = g_centroid_list.front().point;
              double pick_orientation;
              pick_orientation = g_angle_list.front();
              pick(object_location, pick_orientation); 
              g_goal_stack_point_q3.z = g_goal_stack_point_q3.z + k*0.04;
              drop(g_goal_stack_point_q3, (double) g_stack_rotation);
            }
          }
        }
      }
    }

    k += 1;
  }



  return free;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW3::moveJointPosition(const std::vector< double > &group_variable_values)
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
CW3::moveScanPosition(bool scan_front)
{
  /* This function moves the move_group to the scan position, bool true will
    scan the front of robot, bool false will can back of robot*/

  // setup the target pose
  q_x180deg =  tf2::Quaternion(q_x180deg_x, q_x180deg_y, q_x180deg_z, q_x180deg_w);
  tf2::Quaternion q_object;
  tf2::Quaternion q_object2;
  tf2::Quaternion q_result;
  tf2::Quaternion q_result2;
  // q_object.setRPY(angle_offset2_, angle_offset2_, angle_offset_);
  q_object.setRPY(angle_offset_scan_, -angle_offset_scan_, angle_offset_);
  q_object2.setRPY(-angle_offset_scan_, angle_offset_scan_, angle_offset_);
  q_result = q_x180deg * q_object;
  q_result2 = q_x180deg * q_object2;
  geometry_msgs::Quaternion scan_orientation;
  scan_orientation = tf2::toMsg(q_result);
  geometry_msgs::Quaternion scan_orientation2;
  scan_orientation2 = tf2::toMsg(q_result2);
  

  geometry_msgs::Pose scan_pose ;

  if (scan_front == true)
  {
    scan_pose.position.x = g_scan_pose_x;
    scan_pose.position.y = g_scan_pose_y;
    scan_pose.orientation = scan_orientation;
  }
  else if (scan_front == false)
  {
    scan_pose.position.x = g_scan_pose_x2;
    scan_pose.position.y = - g_scan_pose_y;
    scan_pose.orientation = scan_orientation2;
  }
  // scan_pose.position.y = g_scan_pose_y;
  scan_pose.position.z = g_scan_pose_z;
  // scan_pose.orientation = scan_orientation;

  moveArm(scan_pose);

  return true;
}

bool
CW3::moveScanPositionT3a(void)
{
  /* This function moves the move_group to the scan position */

  q_x180deg =  tf2::Quaternion(q_x180deg_x, q_x180deg_y, q_x180deg_z, q_x180deg_w);

  tf2::Quaternion q_object;
  tf2::Quaternion q_result;
  geometry_msgs::Quaternion task3_pose1_orientation;
  q_object.setRPY(angle_offset_scan_, angle_offset_scan_, angle_offset_);
  q_result = q_x180deg * q_object;
  task3_pose1_orientation = tf2::toMsg(q_result);

  geometry_msgs::Pose task3_pose1 ;
  task3_pose1.position.x = g_task3_pose1_x;
  task3_pose1.position.y = g_task3_pose1_y;
  task3_pose1.position.z = g_task3_pose1_z;
  task3_pose1.orientation = task3_pose1_orientation;

  // q_x180deg =  tf2::Quaternion(q_x180deg_x, q_x180deg_y, q_x180deg_z, q_x180deg_w);
  // tf2::Quaternion q_object;
  // q_object.setRPY(angle_offset_scan_, -angle_offset_scan_, angle_offset_);
  // q_result = q_x180deg * q_object;/* This function moves the move_group to the scan position */

  // geometry_msgs::Pose task3_pose1 ;

  // task3_pose1.position.x = g_scan_pose_x;
  // task3_pose1.position.y = g_scan_pose_y;
  // task3_pose1.position.z = g_scan_pose_z;
  // task3_pose1.orientation = scan_orientation;

  moveArm(task3_pose1);

  return true;
}

bool
CW3::moveScanPositionT3b(void)
{
  /* This function moves the move_group to the scan position */

  q_x180deg =  tf2::Quaternion(q_x180deg_x, q_x180deg_y, q_x180deg_z, q_x180deg_w);

  tf2::Quaternion q_object;
  tf2::Quaternion q_result;
  geometry_msgs::Quaternion task3_pose2_orientation;
  q_object.setRPY(angle_offset_scan_, angle_offset_scan_, angle_offset_);
  q_result = q_x180deg * q_object;
  task3_pose2_orientation = tf2::toMsg(q_result);

  geometry_msgs::Pose task3_pose2 ;
  task3_pose2.position.x = g_task3_pose2_x;
  task3_pose2.position.y = g_task3_pose1_y;
  task3_pose2.position.z = g_task3_pose1_z;
  task3_pose2.orientation = task3_pose2_orientation;

  // q_x180deg =  tf2::Quaternion(q_x180deg_x, q_x180deg_y, q_x180deg_z, q_x180deg_w);
  // tf2::Quaternion q_object;
  // q_object.setRPY(angle_offset_scan_, -angle_offset_scan_, angle_offset_);
  // q_result = q_x180deg * q_object;
  // geometry_msgs::Quaternion scan_orientation;
  // scan_orientation = tf2::toMsg(q_result);

  // geometry_msgs::Pose task3_pose2 ;

  // task3_pose2.position.x = g_scan_pose_x2;
  // task3_pose2.position.y = g_scan_pose_y;
  // task3_pose2.position.z = g_scan_pose_z;
  // task3_pose2.orientation = scan_orientation;


  moveArm(task3_pose2);

  return true;
}

bool
CW3::moveScanPositionQ3(int pose)
{
  /* This function moves the move_group to the scan position for task 3, there
      are 4 poses defined, to scan the north east south west of the robot. We 
      simply change the robot base joint for the orientations*/

  // setup the target pose
  q_x180deg =  tf2::Quaternion(q_x180deg_x, q_x180deg_y, q_x180deg_z, q_x180deg_w);

  // determine the grasping orientation
  // q_object.setRPY(3.14159 / 18.0, 3.14159 / 18.0, angle_offset_);
  tf2::Quaternion q_object;
  tf2::Quaternion q_object2;
  tf2::Quaternion q_object3;
  tf2::Quaternion q_object4;
  tf2::Quaternion q_result;
  tf2::Quaternion q_result2;
  tf2::Quaternion q_result3;
  tf2::Quaternion q_result4;
  // q_object.setRPY(angle_offset2_, angle_offset2_, angle_offset_);
  q_object.setRPY(angle_offset_scan_, angle_offset_scan_, angle_offset_);
  q_object2.setRPY(-angle_offset_scan_, -angle_offset_scan_, angle_offset_);
  q_object3.setRPY(angle_offset_scan_, angle_offset_scan_, -angle_offset_);
  q_object4.setRPY(-angle_offset_scan_, -angle_offset_scan_, -angle_offset_);
  q_result = q_x180deg * q_object;
  q_result2 = q_x180deg * q_object2;
  q_result3 = q_x180deg * q_object3;
  q_result4 = q_x180deg * q_object4;
  geometry_msgs::Quaternion scan_orientation;
  scan_orientation = tf2::toMsg(q_result);
  geometry_msgs::Quaternion scan_orientation2;
  scan_orientation2 = tf2::toMsg(q_result2);
  geometry_msgs::Quaternion scan_orientation3;
  scan_orientation3 = tf2::toMsg(q_result3);
  geometry_msgs::Quaternion scan_orientation4;
  scan_orientation4 = tf2::toMsg(q_result4);



  geometry_msgs::Pose scan_pose ;

  if (pose == 1)
  { 
    scan_pose.position.x = g_task3_pose1_y;
    scan_pose.position.y = g_task3_pose1_y;
    scan_pose.position.z = g_task3_pose1_z;
    scan_pose.orientation = scan_orientation;
    moveArm(scan_pose);
    ros::Duration(1).sleep();
    moveit::core::RobotStatePtr current_state = arm_group_.getCurrentState();
    current_state->copyJointGroupPositions("panda_arm", g_joint_group_positions_q3);
  }
  else if (pose == 2)
  {
    // scan_pose.orientation = scan_orientation2;
    std::vector<double> pose2;
    pose2 = g_joint_group_positions_q3;
    pose2[0] = pose2[0] + joint_offset2_;
    moveJointPosition(pose2);
  }
  else if (pose == 3)
  {
    // scan_pose.orientation = scan_orientation3;
    std::vector<double> pose3;
    pose3 = g_joint_group_positions_q3;
    pose3[0] = pose3[0] + joint_offset1_;
    moveJointPosition(pose3);
  }
  else if (pose == 4)
  {
    // scan_pose.orientation = scan_orientation4;
    std::vector<double> pose4;
    pose4 = g_joint_group_positions_q3;
    pose4[0] = pose4[0] - joint_offset1_;
    moveJointPosition(pose4);
  }

  // scan_pose.position.x = g_task3_pose1_y;
  // scan_pose.position.y = g_task3_pose1_y;
  // scan_pose.position.z = g_task3_pose1_z;
  // scan_pose.orientation = scan_orientation;

  // moveArm(scan_pose);

  return true;
}


bool
CW3::moveArm(geometry_msgs::Pose target_pose)

{
  /* This function moves the move_group to the target position */

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);
  // arm_group_.setPlanningTime(10);
  // arm_group_.setGoalTolerance(0.1);
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
CW3::moveCartesian(geometry_msgs::Pose current_pose,
      geometry_msgs::Pose target_pose)

{
  /* This function moves the move_group to the target position with a cartesian
      move instead of a join move. Used when pick and place*/
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
CW3::moveGripper(float width)
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
CW3::pick(geometry_msgs::Point position, double angle)
{
  /* This function picks up an object at a given point and angle */

  // define grasping as from above
  q_x180deg =  tf2::Quaternion(q_x180deg_x, q_x180deg_y, q_x180deg_z, q_x180deg_w);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  tf2::Quaternion q_result;
  q_object.setRPY(angle_offset1_, angle_offset1_, angle_offset_ + angle);
  // q_result = q_x180deg * q_object;

  // tf2::Quaternion pick_angle;
  // pick_angle.setRPY(angle_offset1_, angle_offset1_, angle);

  // q_result = q_x180deg * q_object * pick_angle;
  geometry_msgs::Quaternion pick_orientation;
  q_result = q_x180deg * q_object ;
  pick_orientation = tf2::toMsg(q_result);

  // set the desired grasping pose
  geometry_msgs::Pose grasp_pose;
  grasp_pose.position = position;
  grasp_pose.orientation = pick_orientation;
  grasp_pose.position.z += g_pick_offset_;

  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose;
  approach_pose = grasp_pose;
  approach_pose.position.z += approach_distance_;

  // remove collision object first 

  tf2::Quaternion object_orientation;
  object_orientation.setRPY(angle_offset1_, angle_offset1_, angle);
  geometry_msgs::Quaternion object_orientation_msg;
  object_orientation_msg = tf2::toMsg(object_orientation);

  addCollisionObject(g_cube, position, g_cube_dimensions,
    object_orientation_msg);

  ros::Duration(2).sleep();

  /* Now perform the pick */

  bool success = true;

  ROS_INFO("Begining pick operation");

  // move the arm above the object
  success *= moveArm(approach_pose);

  removeCollisionObject(g_cube, position, g_cube_dimensions,
  object_orientation_msg);

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

  ros::Duration(0.5).sleep();
  // grasp!
  success *= moveGripper(gripper_closed_); //gripper_closed_

  ros::Duration(0.5).sleep();

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
CW3::drop(geometry_msgs::Point position, double angle)
{
  /* This function drops the picked object to the input position at a required
      angle */

  // define grasping as from above
  q_x180deg =  tf2::Quaternion(q_x180deg_x, q_x180deg_y, q_x180deg_z, q_x180deg_w);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  tf2::Quaternion q_result;
  q_object.setRPY(angle_offset1_, angle_offset1_, angle_offset_ + angle);
  
  // tf2::Quaternion pick_angle;
  // pick_angle.setRPY(angle_offset1_, angle_offset1_, angle);

  // q_result = q_x180deg * q_object * pick_angle;
  geometry_msgs::Quaternion drop_orientation;
  q_result = q_x180deg * q_object;
  drop_orientation = tf2::toMsg(q_result);

  // set the desired pose to drop cube
  geometry_msgs::Pose drop_pose;
  drop_pose.position = position;
  drop_pose.orientation = drop_orientation;
  drop_pose.position.z += g_drop_distance_;

  // set the desired pose to approach top of box
  geometry_msgs::Pose approach_box;
  approach_box = drop_pose;
  approach_box.position.z += approach_box_;

  /* Now perform the pick */


  // remove collision object first 

  tf2::Quaternion object_orientation;
  object_orientation.setRPY(angle_offset1_, angle_offset1_, angle);
  geometry_msgs::Quaternion object_orientation_msg;
  object_orientation_msg = tf2::toMsg(object_orientation);

  addCollisionObject(g_cube, position, g_cube_dimensions,
    object_orientation_msg);

  ros::Duration(2).sleep();


  bool success = true;

  ROS_INFO("Begining pick operation");

  // move the arm above the box
  success *= moveArm(approach_box);

  removeCollisionObject(g_cube, position, g_cube_dimensions,
  object_orientation_msg);

  if (not success) 
  {
    ROS_ERROR("Moving arm to box failed");
    return false;
  }

  ros::Duration(0.5).sleep();

  // move the arm to drop pose
  success *= moveArm(drop_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to drop pose failed");
    return false;
  }

  ros::Duration(0.5).sleep();

  // open the gripper
  success *= moveGripper(gripper_open_);

  if (not success) 
  {
    ROS_ERROR("Opening gripper prior to pick failed");
    return false;
  }

  ros::Duration(0.5).sleep();

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
CW3::colourFilter(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  /* This is the main callback from the subscriber. It first applies voxel 
      filtering then passthrough filter to remove unwanted aread of ground. 
      We then segment the ground to remove the floor. We also apply rbg 
      filtering to filter out unwanted colour using the RGBFilter funtion.
      The showStack funtion removes cloud around the stack, which is used in
      task 3*/

  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;

  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_rgb_cloud);
  applyVX (g_rgb_cloud, g_rgb_cloud);
  applyPT (g_rgb_cloud, g_rgb_cloud);
  // applyPT (g_rgb_cloud, g_cloud_filtered_out_floor);

  segFloor(g_rgb_cloud, g_cloud_filtered_out_floor);

  RGBFilter(g_rgb_cloud, g_cloud_filtered_colour);
  // pcl::PackedRGBComparison<PointT>::Ptr
  //   red_condition_lb(new pcl::PackedRGBComparison<PointT>("r", pcl::ComparisonOps::GE, g_r.data-g_rbg_tolerance));
  // pcl::PackedRGBComparison<PointT>::Ptr
  //   red_condition_ub(new pcl::PackedRGBComparison<PointT>("r", pcl::ComparisonOps::LE, g_r.data+g_rbg_tolerance));
  // pcl::PackedRGBComparison<PointT>::Ptr
  //   green_condition_lb(new pcl::PackedRGBComparison<PointT>("g", pcl::ComparisonOps::GE, g_g.data-g_rbg_tolerance));
  // pcl::PackedRGBComparison<PointT>::Ptr
  //   green_condition_ub(new pcl::PackedRGBComparison<PointT>("g", pcl::ComparisonOps::LE, g_g.data+g_rbg_tolerance));
  // pcl::PackedRGBComparison<PointT>::Ptr
  //   blue_condition_lb(new pcl::PackedRGBComparison<PointT>("b", pcl::ComparisonOps::GE, g_b.data-g_rbg_tolerance));
  // pcl::PackedRGBComparison<PointT>::Ptr
  //   blue_condition_ub(new pcl::PackedRGBComparison<PointT>("b", pcl::ComparisonOps::LE, g_b.data+g_rbg_tolerance));
  // pcl::PackedHSIComparisong_rgb_cloud<PointT>::Ptr
  //   hue_condition_lb(new pcl::PackedHSIComparison<PointT>("h", pcl::ComparisonOps::GE, g_hue - g_hsi_tolerance));
  // pcl::PackedHSIComparison<PointT>::Ptr
  //   hue_condition_ub(new pcl::PackedHSIComparison<PointT>("h", pcl::ComparisonOps::LE, g_hue + g_hsi_tolerance));
  // pcl::PackedHSIComparison<PointT>::Ptr
  //   sat_condition_lb(new pcl::PackedHSIComparison<PointT>("s", pcl::ComparisonOps::GE, g_sat - g_hsi_tolerance));
  // pcl::PackedHSIComparison<PointT>::Ptr
  //   sat_condition_ub(new pcl::PackedHSIComparison<PointT>("s", pcl::ComparisonOps::LE, g_sat + g_hsi_tolerance));
  // pcl::PackedHSIComparison<PointT>::Ptr
  //   int_condition_lb(new pcl::PackedHSIComparison<PointT>("i", pcl::ComparisonOps::GE, g_int - g_hsi_tolerance));
  // pcl::PackedHSIComparison<PointT>::Ptr
  //   int_condition_ub(new pcl::PackedHSIComparison<PointT>("i", pcl::ComparisonOps::LE, g_int + g_hsi_tolerance));
  
  // pcl::ConditionAnd<PointT>::Ptr color_cond (new pcl::ConditionAnd<PointT> ());
  // color_cond->addComparison (red_condition_lb);
  // color_cond->addComparison (red_condition_ub);
  // color_cond->addComparison (green_condition_lb);
  // color_cond->addComparison (green_condition_ub);
  // color_cond->addComparison (blue_condition_lb);
  // color_cond->addComparison (blue_condition_ub);
  // color_cond->addComparison (hue_condition_lb);
  // color_cond->addComparison (hue_condition_ub);
  // // color_cond->addComparison (sat_condition_lb);
  // // color_cond->addComparison (sat_condition_ub);
  // // color_cond->addComparison (int_condition_lb);
  // // color_cond->addComparison (int_condition_ub);

  // // Build the filter
  // color_filter.setInputCloud(g_rgb_cloud);
  // color_filter.setCondition (color_cond);
  // color_filter.filter(*g_cloud_filtered_colour); 
  
  // findNormals (g_rgb_cloud);
  
  pubFilteredPCMsg (g_pub_cloud, *g_cloud_filtered_out_floor);

  // Q1 2 and 3
  pubFilteredPCMsg (g_pub_colour_filtered, *g_cloud_filtered_colour);

  // Q3
  showStack(g_cloud_filtered_out_floor, g_cloud_filtered_stack);
  RGBFilter(g_cloud_filtered_stack, g_cloud_filtered_stack_rgb);

  removeStack(g_rgb_cloud, g_cloud_filtered_out_stack);
  RGBFilter(g_cloud_filtered_out_stack, g_cloud_filtered_out_stack_rgb);

  pubFilteredPCMsg (g_pub_crop_stack, *g_cloud_filtered_stack);

  pubFilteredPCMsg (g_pub_crop_stack_rgb, *g_cloud_filtered_stack_rgb);
  
  pubFilteredPCMsg (g_pub_crop_stack_rgb, *g_cloud_filtered_out_stack_rgb);

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::applyVX (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  /* Apply voxel filtering to down sample the cloud*/
  g_vx.setInputCloud (in_cloud_ptr);
  g_vx.setLeafSize (g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::applyPT (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  /* Apply pass through filtering to down sample the cloud*/

  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("y"); // filter in y direction
  // g_pt.setFilterLimits (g_pt_thrs_min, g_pt_thrs_max);
  g_pt.setFilterLimits (g_pt_thrs_min, g_pt_thrs_max);
  // g_pt.setFilterFieldName ("z"); // filter in y direction
  // g_pt.setFilterLimits (0, 0.65);
  g_pt.filter (*out_cloud_ptr);
  
  return;
}

void
CW3::findNormals (PointCPtr &in_cloud_ptr)
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
CW3::RGBFilter (PointCPtr &in_cloud_ptr,
                PointCPtr &out_cloud_ptr)
{
  /* This funciton filters out the clouds of any unwanteed colour. The RGB
      values of the required cloud are defined as global variables which 
      change for different tasks. This also filters out colour based on given
      hue values to distinguish colour with similar rbg values. Threshold is 
      added to create lower and upper bound for filter */

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
    hue_condition_lb(new pcl::PackedHSIComparison<PointT>("h", pcl::ComparisonOps::GE, g_hue - g_hsi_tolerance));
  pcl::PackedHSIComparison<PointT>::Ptr
    hue_condition_ub(new pcl::PackedHSIComparison<PointT>("h", pcl::ComparisonOps::LE, g_hue + g_hsi_tolerance));
  pcl::PackedHSIComparison<PointT>::Ptr
    sat_condition_lb(new pcl::PackedHSIComparison<PointT>("s", pcl::ComparisonOps::GE, g_sat - g_hsi_tolerance));
  pcl::PackedHSIComparison<PointT>::Ptr
    sat_condition_ub(new pcl::PackedHSIComparison<PointT>("s", pcl::ComparisonOps::LE, g_sat + g_hsi_tolerance));
  pcl::PackedHSIComparison<PointT>::Ptr
    int_condition_lb(new pcl::PackedHSIComparison<PointT>("i", pcl::ComparisonOps::GE, g_int - g_hsi_tolerance));
  pcl::PackedHSIComparison<PointT>::Ptr
    int_condition_ub(new pcl::PackedHSIComparison<PointT>("i", pcl::ComparisonOps::LE, g_int + g_hsi_tolerance));
  
  pcl::ConditionAnd<PointT>::Ptr color_cond (new pcl::ConditionAnd<PointT> ());
  color_cond->addComparison (red_condition_lb);
  color_cond->addComparison (red_condition_ub);
  color_cond->addComparison (green_condition_lb);
  color_cond->addComparison (green_condition_ub);
  color_cond->addComparison (blue_condition_lb);
  color_cond->addComparison (blue_condition_ub);
  color_cond->addComparison (hue_condition_lb);
  color_cond->addComparison (hue_condition_ub);
  // color_cond->addComparison (sat_condition_lb);
  // color_cond->addComparison (sat_condition_ub);
  // color_cond->addComparison (int_condition_lb);
  // color_cond->addComparison (int_condition_ub);

  // Build the filter
  color_filter.setInputCloud(in_cloud_ptr);
  color_filter.setCondition (color_cond);
  color_filter.filter(*out_cloud_ptr); 

}
void
CW3::segFloor (PointCPtr &in_cloud_ptr,
                PointCPtr &out_cloud_ptr)

{
  /* This funciton performs segmentation to filter out the ground*/

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (seg_dist_thres_floor);
  seg.setInputCloud (in_cloud_ptr);
  seg.segment (*inliers, *coefficients);

  // Extract the planar inliers from the input cloud
  g_extract_pc.setInputCloud (in_cloud_ptr);
  g_extract_pc.setIndices (inliers);
  g_extract_pc.setNegative (false);

  g_extract_pc.filter (*g_cloud_floor);

  // Remove the planar inliers, extract the rest
  g_extract_pc.setNegative (true);
  g_extract_pc.filter (*out_cloud_ptr);


  return;

}

////////////////////////////////////////////////////////////////////////////////
void
CW3::showStack (PointCPtr &in_cloud_ptr,
                PointCPtr &out_cloud_ptr)
{
  /* This funciton takes the position of the stack in task 3 and apply field 
      comparision to filter out area around the stack. The point is transformed
      back to the camera frame befire filtering*/

  geometry_msgs::PointStamped world;

  world.header.frame_id = g_target_frame;
  world.header.stamp = ros::Time (0);
  world.point = g_stack_point_q3;

  geometry_msgs::PointStamped cam;

  try
  {
    g_listener_.transformPoint (g_input_pc_frame_id_,  
                                world,
                                cam);
    //ROS_INFO ("trying transform...");
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }

  double floor_x = cam.point.x;
  double floor_y = cam.point.y;
  double floor_z = cam.point.z;

  // double floor_x = g_stack_point_q3.x;
  // double floor_y = g_stack_point_q3.y;
  // double floor_z = g_stack_point_q3.z;

  pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());

  
  pcl::FieldComparison<PointT>::ConstPtr floor_x_lb (
        new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GE, floor_x-g_xyz_tolerance));
  pcl::FieldComparison<PointT>::ConstPtr floor_x_ub (
        new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LE, floor_x+g_xyz_tolerance));
  pcl::FieldComparison<PointT>::ConstPtr floor_y_lb (
        new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GE, floor_y-g_xyz_tolerance));
  pcl::FieldComparison<PointT>::ConstPtr floor_y_ub (
        new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LE, floor_y+g_xyz_tolerance));
  pcl::FieldComparison<PointT>::ConstPtr floor_z_lb (
        new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GE, floor_z-g_xyz_tolerance));
  pcl::FieldComparison<PointT>::ConstPtr floor_z_ub (
        new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LE, floor_z+g_xyz_tolerance));

  range_cond->addComparison(floor_x_lb);
  range_cond->addComparison(floor_x_ub);
  range_cond->addComparison(floor_y_lb);
  range_cond->addComparison(floor_y_ub);
  // range_cond->addComparison(floor_z_lb);
  // range_cond->addComparison(floor_z_ub);

  floor_remove.setCondition(range_cond);
  floor_remove.setInputCloud(in_cloud_ptr);
  floor_remove.filter(*out_cloud_ptr);

  return;

}

////////////////////////////////////////////////////////////////////////////////

void
CW3::removeStack (PointCPtr &in_cloud_ptr,
                PointCPtr &out_cloud_ptr)
{
  /* This funciton does the opposite of showStack, to filter out area of the 
      stack in task 3 */
  
  geometry_msgs::PointStamped world;

  world.header.frame_id = g_target_frame;
  world.header.stamp = ros::Time (0);
  world.point = g_stack_point_q3;

  geometry_msgs::PointStamped cam;

  try
  {
    g_listener_.transformPoint (g_input_pc_frame_id_,  
                                world,
                                cam);
    //ROS_INFO ("trying transform...");
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }

  double floor_x = cam.point.x;
  double floor_y = cam.point.y;
  double floor_z = cam.point.z;

  // double floor_x = g_stack_point_q3.x;
  // double floor_y = g_stack_point_q3.y;
  // double floor_z = g_stack_point_q3.z;

  pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());

  
  pcl::FieldComparison<PointT>::ConstPtr floor_x_lb (
        new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GE, floor_x+g_xyz_tolerance));
  pcl::FieldComparison<PointT>::ConstPtr floor_x_ub (
        new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LE, floor_x-g_xyz_tolerance));
  pcl::FieldComparison<PointT>::ConstPtr floor_y_lb (
        new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GE, floor_y+g_xyz_tolerance));
  pcl::FieldComparison<PointT>::ConstPtr floor_y_ub (
        new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LE, floor_y-g_xyz_tolerance));
  pcl::FieldComparison<PointT>::ConstPtr floor_z_lb (
        new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GE, floor_z+g_xyz_tolerance));
  pcl::FieldComparison<PointT>::ConstPtr floor_z_ub (
        new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LE, floor_z-g_xyz_tolerance));

  range_cond->addComparison(floor_x_lb);
  range_cond->addComparison(floor_x_ub);
  range_cond->addComparison(floor_y_lb);
  range_cond->addComparison(floor_y_ub);
  // range_cond->addComparison(floor_z_lb);
  // range_cond->addComparison(floor_z_ub);

  floor_remove.setCondition(range_cond);
  floor_remove.setInputCloud(in_cloud_ptr);
  floor_remove.filter(*out_cloud_ptr);

  return;

}

////////////////////////////////////////////////////////////////////////////////

void
CW3::segPlane (PointCPtr &in_cloud_ptr)
{ 
  /* This function fits a plane and extract normal, which is used when finding 
  the angle of the stack */
  
  // Create the segmentation object for the planar model
  // and set all the params
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  g_seg.setNormalDistanceWeight (seg_dist_weight); 
  g_seg.setMethodType (pcl::SAC_RANSAC);
  g_seg.setMaxIterations (seg_max_it); 
  g_seg.setDistanceThreshold (seg_dist_thres_plane); 
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

  ROS_INFO_STREAM ("in_cloud_ptr: "
                   << in_cloud_ptr->size ()
                   << " data points.");

  ROS_INFO_STREAM ("g_cloud_plane: "
                   << g_cloud_plane->size ()
                   << " data points.");

  ROS_INFO_STREAM ("g_cloud_filtered2: "
                   << g_cloud_filtered2->size ()
                   << " data points.");

  ROS_INFO_STREAM ("g_cloud_normals2: "
                  << g_cloud_normals2->size ()
                  << " data points.");

  ROS_INFO_STREAM ("g_cloud_normals: "
                  << g_cloud_normals->size ()
                  << " data points.");

  pubFilteredPCMsg (g_pub_plane, *g_cloud_plane);
  pubFilteredPCMsg (g_pub_cloud_filtered2, *g_cloud_filtered2);


}
    

////////////////////////////////////////////////////////////////////////////////
void
CW3::pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc)
{
  // Publish the data
  
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish (g_cloud_filtered_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::pubNormal (geometry_msgs::PoseStamped &normal_msg)
{
  // Publish the data
  
  g_pub_pose.publish (normal_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::publishPoint (geometry_msgs::PointStamped &cube_pt_msg)
{
  // Create and publish the centroid

  g_pub_point.publish (cube_pt_msg);
  
  return;
}

void
CW3::publishPose (geometry_msgs::PoseStamped &cube_pt_msg)
{
  // Create and publish the pose

  g_pub_pose.publish (cube_pt_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW3::pubFilteredRGB (ros::Publisher &pc_pub,
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
CW3::euclideanCluster (PointCPtr &in_cloud_ptr)

{
  /* This function performs Euclidean Clustering to cluster the clouds, it 
  returns the angle of each cluster (by finding normal of each cluster), and 
  the list of centroids as global variables */

  // pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  g_tree_cluster_ptr->setInputCloud (in_cloud_ptr);
  // pcl::PCDWriter writer;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (g_cluster_tolerance); // 4cm
  // ec.setMinClusterSize (min_cluster_size);
  // ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (g_tree_cluster_ptr);
  ec.setInputCloud (in_cloud_ptr);
  ec.extract (cluster_indices);
  
  // g_cube_dimensions.x = g_cube_dimensions_x;
  // g_cube_dimensions.y = g_cube_dimensions_y;
  // g_cube_dimensions.z = g_cube_dimensions_y;

  // pcl::PointCloud<PointT>::Ptr cloud_cluster (new PointC);
  g_centroid_list.clear();
  g_angle_list.clear();
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

    // find angle of cubes
    findNormals(cloud_cluster);
    ros::Duration(0.5).sleep();
    double angle_x = g_cloud_normals->points[0].normal_x;
    double angle_y = g_cloud_normals->points[0].normal_y;
    double angle_z = g_cloud_normals->points[0].normal_z;
    findCubeAngle (angle_x, angle_y, angle_z);
    g_angle_list.push_back(g_stack_rotation);

    // tf2::Quaternion orientation;
    // orientation.setRPY(angle_x, angle_y, angle_z);
    // geometry_msgs::Quaternion orientationmsg;
    // orientationmsg = tf2::toMsg(orientation);
  
    Eigen::Vector4f centroid_in;
    pcl::compute3DCentroid(*cloud_cluster, centroid_in);

    geometry_msgs::PointStamped centroids;
    // geometry_msgs::PoseStamped centroids;
    centroids.header.frame_id = g_input_pc_frame_id_;
    centroids.header.stamp = ros::Time (0);
    centroids.point.x = centroid_in[0];
    centroids.point.y = centroid_in[1];
    centroids.point.z = centroid_in[2];
    // centroids.pose.position.x = centroid_in[0];
    // centroids.pose.position.y = centroid_in[1];
    // centroids.pose.position.z = centroid_in[2];
    // centroids.pose.orientation = orientationmsg;

    // Transform the point to new frame
      
    geometry_msgs::PointStamped cube_pt_msg_out;
    // geometry_msgs::PoseStamped cube_pt_msg_out;
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
    
    // publishPose (cube_pt_msg_out);
    publishPoint (cube_pt_msg_out);

    // ROS_INFO_STREAM ("centroid_x: "
    //               << cube_pt_msg_out.point.x);
    
    // ROS_INFO_STREAM ("centroid_y: "
    //                 << cube_pt_msg_out.point.y);

    // ROS_INFO_STREAM ("centroid_z: "
    //               << cube_pt_msg_out.point.z);    



  } ; 

  std::cout << j-1 << " centroid(s) found! \n" ;

  return;

}

////////////////////////////////////////////////////////////////////////////////


void
CW3::addCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  /* add a collision object in RViz and the MoveIt planning scene */

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

////////////////////////////////////////////////////////////////////////////////

void
CW3::removeCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  /* Remove collision objects in RViz and the MoveIt planning scene */
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
  collision_object.operation = collision_object.REMOVE;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

////////////////////////////////////////////////////////////////////////////////

void 
CW3::addCollisionItems (geometry_msgs::Point &goal_location)

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

////////////////////////////////////////////////////////////////////////////////

void
CW3::findCubeAngle (double x, double y, double z)

{
  /* This function finds the angle along the x-axis given a 3d vector. Calculates
  with simple trigonometry. Angle is found by taking modulus of 90 degree, as 
  object is a cube*/
  g_stack_rotation = acos(x/sqrt(pow(x,2) + pow(y,2) + pow(z,2)));


  g_stack_rotation = std::fmod(g_stack_rotation,cube_angle);  

  std::cout << "Cube Angle: " << g_stack_rotation << std::endl;

  return;
}

void
CW3::colourOfCloud (double r, double g, double b)

{
  /* This function finds the colour from the given rgb values, and change the
  hue and saturation global vaiables to be used in the RGB filtering*/

  ROS_INFO("RGB values: %g %g %g", 
    r, g, b);
    
  if (r ==g_blue_r_value && g == g_blue_g_value
    && b == g_blue_b_value) 
  {
    ROS_INFO("Colour: blue");
    g_required_colour = "blue";
    g_hue = g_hue_blue;
    g_sat = g_sat_blue;
    g_int = g_int_blue;
  } 
  else if (r ==g_purple_r_value && g == g_purple_g_value
      && b == g_purple_b_value) 
  {
    ROS_INFO("Colour: Purple");
    g_required_colour = "purple";
    g_hue = g_hue_purple;
    g_sat = g_sat_purple;
    g_int = g_int_purple;
  }
  else if (r ==g_red_r_value && g == g_red_g_value
      && b == g_red_b_value) 
  {
    ROS_INFO("Colour: red");
    g_required_colour = "red";
    g_hue = g_hue_red;
    g_sat = g_sat_red;
    g_int = g_int_red;   
  } 
  else if (r ==g_orange_r_value && g == g_orange_g_value
      && b == g_orange_b_value) 
  {
    ROS_INFO("Colour: Orange");
    g_required_colour = "orange";
    g_hue = g_hue_orange;
    g_sat = g_sat_orange;
    g_int = g_int_orange;   
  }
  else if (r ==g_yellow_r_value && g == g_yellow_g_value
      && b == g_yellow_b_value) 
  {
    ROS_INFO("Colour: Yellow");
    g_required_colour = "yellow";
    g_hue = g_hue_yellow;
    g_sat = g_sat_yellow;
    g_int = g_int_yellow;   
  } 
  else if (r ==g_pink_r_value && g == g_pink_g_value
      && b == g_pink_b_value) 
  {
    ROS_INFO("Colour: Pink");
    g_required_colour = "pink";
    g_hue = g_hue_pink;
    g_sat = g_sat_pink;
    g_int = g_int_pink;   
  } 
}

std::multimap< double, std::string, std::less<double> >
CW3::findColourInStack (void)

{
  /* This function finds colour in the stack using rbg filtering and 
  euclidean clustering. Changes the rbg values one at a time and find
  cluster and centroid. The order of colour is sorted in a multimap
  based on the z-position of the centroids*/
  std::multimap< double, std::string, std::less<double> > mmapOfColour;

  //// start with blue
  g_r.data = g_blue_r_value;
  g_g.data = g_blue_g_value;
  g_b.data = g_blue_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_colour);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      // z_value = i.pose.position.z;
      // mmapOfColour.insert(std::pair<std::string, double>("blue", z_value));
      mmapOfColour.insert(std::pair<double, std::string>(z_value, g_required_colour));
    }
    // euclideanCluster (g_cloud_filtered_rgb);
    // ros::Duration(0.2).sleep();
  }

  g_r.data = g_purple_r_value;
  g_g.data = g_purple_g_value;
  g_b.data = g_purple_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_colour);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      // z_value = i.pose.position.z;
      mmapOfColour.insert(std::pair<double, std::string>(z_value, g_required_colour));
      // mmapOfColour.insert(std::pair<std::string, double>("purple", z_value));
      // mmapOfColour.insert(std::pair<double, std::string>(z_value, "purple"));
      // mmapOfColour.insert(std::pair<double, std::string, double,
      //   double, double>(z_value, g_required_colour, g_r.data/g_rgb_convert,
      //     g_g.data/g_rgb_convert, g_b.data/g_rgb_convert ));
    }
    // euclideanCluster (g_cloud_filtered_rgb);
    // ros::Duration(0.2).sleep();
  }

  g_r.data = g_red_r_value;
  g_g.data = g_red_g_value;
  g_b.data = g_red_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_colour);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      // z_value = i.pose.position.z;
      // mmapOfColour.insert(std::pair<std::string, double>("red", z_value));
      // mmapOfColour.insert(std::pair<double, std::string>(z_value, "red"));
      mmapOfColour.insert(std::pair<double, std::string>(z_value, g_required_colour));
    }
    // euclideanCluster (g_cloud_filtered_rgb);
    // ros::Duration(0.2).sleep();
  }

  g_r.data = g_orange_r_value;
  g_g.data = g_orange_g_value;
  g_b.data = g_orange_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_colour);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      // z_value = i.pose.position.z;
      // mmapOfColour.insert(std::pair<std::string, double>("orange", z_value));
      // mmapOfColour.insert(std::pair<double, std::string>(z_value, "orange"));
      mmapOfColour.insert(std::pair<double, std::string>(z_value, g_required_colour));
    }
    // euclideanCluster (g_cloud_filtered_rgb);
    // ros::Duration(0.2).sleep();
  }

  g_r.data = g_yellow_r_value;
  g_g.data = g_yellow_g_value;
  g_b.data = g_yellow_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_colour);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      // z_value = i.pose.position.z;
      // mmapOfColour.insert(std::pair<std::string, double>("yellow", z_value));
      // mmapOfColour.insert(std::pair<double, std::string>(z_value, "yellow"));
      mmapOfColour.insert(std::pair<double, std::string>(z_value, g_required_colour));
    }
    // euclideanCluster (g_cloud_filtered_rgb);
    // ros::Duration(0.2).sleep();
  }

  g_r.data = g_pink_r_value;
  g_g.data = g_pink_g_value;
  g_b.data = g_pink_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_colour);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      // z_value = i.pose.position.z;
      // mmapOfColour.insert(std::pair<std::string, double>("pink", z_value));
      // mmapOfColour.insert(std::pair<double, std::string>(z_value, "pink"));
      mmapOfColour.insert(std::pair<double, std::string>(z_value, g_required_colour));
    }
    // euclideanCluster (g_cloud_filtered_rgb);
    // ros::Duration(0.2).sleep();
  }

 
  return mmapOfColour; 

}


std::multimap< double, std::string, std::less<double> >
CW3::findColourInStackQ3 (void)

{
  /* This function finds colour in the stack using rbg filtering and 
  euclidean clustering. Changes the rbg values one at a time and find
  cluster and centroid. The order of colour is sorted in a multimap
  based on the z-position of the centroids*/
  std::multimap< double, std::string, std::less<double> > mmapOfColour;

  //// start with blue
  g_r.data = g_blue_r_value;
  g_g.data = g_blue_g_value;
  g_b.data = g_blue_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_stack_rgb);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      // z_value = i.pose.position.z;
      // mmapOfColour.insert(std::pair<std::string, double>("blue", z_value));
      mmapOfColour.insert(std::pair<double, std::string>(z_value, g_required_colour));
    }
    // euclideanCluster (g_cloud_filtered_rgb);
    // ros::Duration(0.2).sleep();
  }

  g_r.data = g_purple_r_value;
  g_g.data = g_purple_g_value;
  g_b.data = g_purple_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_stack_rgb);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      // z_value = i.pose.position.z;
      mmapOfColour.insert(std::pair<double, std::string>(z_value, g_required_colour));
      // mmapOfColour.insert(std::pair<std::string, double>("purple", z_value));
      // mmapOfColour.insert(std::pair<double, std::string>(z_value, "purple"));
      // mmapOfColour.insert(std::pair<double, std::string, double,
      //   double, double>(z_value, g_required_colour, g_r.data/g_rgb_convert,
      //     g_g.data/g_rgb_convert, g_b.data/g_rgb_convert ));
    }
    // euclideanCluster (g_cloud_filtered_rgb);
    // ros::Duration(0.2).sleep();
  }

  g_r.data = g_red_r_value;
  g_g.data = g_red_g_value;
  g_b.data = g_red_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_stack_rgb);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      // z_value = i.pose.position.z;
      // mmapOfColour.insert(std::pair<std::string, double>("red", z_value));
      // mmapOfColour.insert(std::pair<double, std::string>(z_value, "red"));
      mmapOfColour.insert(std::pair<double, std::string>(z_value, g_required_colour));
    }
    // euclideanCluster (g_cloud_filtered_rgb);
    // ros::Duration(0.2).sleep();
  }

  g_r.data = g_orange_r_value;
  g_g.data = g_orange_g_value;
  g_b.data = g_orange_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_stack_rgb);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      // z_value = i.pose.position.z;
      // mmapOfColour.insert(std::pair<std::string, double>("orange", z_value));
      // mmapOfColour.insert(std::pair<double, std::string>(z_value, "orange"));
      mmapOfColour.insert(std::pair<double, std::string>(z_value, g_required_colour));
    }
    // euclideanCluster (g_cloud_filtered_rgb);
    // ros::Duration(0.2).sleep();
  }

  g_r.data = g_yellow_r_value;
  g_g.data = g_yellow_g_value;
  g_b.data = g_yellow_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_stack_rgb);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      // z_value = i.pose.position.z;
      // mmapOfColour.insert(std::pair<std::string, double>("yellow", z_value));
      // mmapOfColour.insert(std::pair<double, std::string>(z_value, "yellow"));
      mmapOfColour.insert(std::pair<double, std::string>(z_value, g_required_colour));
    }
    // euclideanCluster (g_cloud_filtered_rgb);
    // ros::Duration(0.2).sleep();
  }

  g_r.data = g_pink_r_value;
  g_g.data = g_pink_g_value;
  g_b.data = g_pink_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_stack_rgb);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      // z_value = i.pose.position.z;
      // mmapOfColour.insert(std::pair<std::string, double>("pink", z_value));
      // mmapOfColour.insert(std::pair<double, std::string>(z_value, "pink"));
      mmapOfColour.insert(std::pair<double, std::string>(z_value, g_required_colour));
    }
    // euclideanCluster (g_cloud_filtered_rgb);
    // ros::Duration(0.2).sleep();
  }

 
  return mmapOfColour; 

}


void
CW3::findTallestStack(void)
{  
   /* This function finds the stack in task 3 by finding centroids of each
   cube. Similar approach to fiding colour in stack, but saves the point of 
   the centroid. Centroid with highest z-value will be the position of the 
   cube at the top of the stack.*/
   
    //// start with blue
  g_r.data = g_blue_r_value;
  g_g.data = g_blue_g_value;
  g_b.data = g_blue_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_colour);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      if (z_value > g_tallest_point.z)
      {
        g_tallest_point = i.point;
      }
    }
  }

  g_r.data = g_purple_r_value;
  g_g.data = g_purple_g_value;
  g_b.data = g_purple_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_colour);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      if (z_value > g_tallest_point.z)
      {
        g_tallest_point = i.point;
      }
    }
  }

  g_r.data = g_red_r_value;
  g_g.data = g_red_g_value;
  g_b.data = g_red_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_colour);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      if (z_value > g_tallest_point.z)
      {
        g_tallest_point = i.point;
      }
    }
  }

  g_r.data = g_orange_r_value;
  g_g.data = g_orange_g_value;
  g_b.data = g_orange_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_colour);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      if (z_value > g_tallest_point.z)
      {
        g_tallest_point = i.point;
      }
    }
  }

  g_r.data = g_yellow_r_value;
  g_g.data = g_yellow_g_value;
  g_b.data = g_yellow_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_colour);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      if (z_value > g_tallest_point.z)
      {
        g_tallest_point = i.point;
      }
    }
  }

  g_r.data = g_pink_r_value;
  g_g.data = g_pink_g_value;
  g_b.data = g_pink_b_value;
  colourOfCloud(g_r.data, g_g.data, g_b.data);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  euclideanCluster (g_cloud_filtered_colour);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if (! g_centroid_list.empty())
  { 
    for (auto i: g_centroid_list)
    {
      double z_value;
      z_value = i.point.z;
      if (z_value > g_tallest_point.z)
      {
        g_tallest_point = i.point;
      }
    }
  }

}