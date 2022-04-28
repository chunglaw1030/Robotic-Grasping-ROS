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

#pragma once

// ros includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include<cmath>
#include <chrono>
#include <thread>
#include <map>
#include <bits/stdc++.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/point_cloud.h>

#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/Vertices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// standard c++ library includes (std::string, std::vector)
#include <string>
#include <vector>
#include <std_msgs/Float32.h>
#include <map>

// headers generated by catkin for the custom services we have made
// #include <cw3_world_spawner.h>
#include <cw3_world_spawner/Task1Service.h>
#include <cw3_world_spawner/Task2Service.h>
#include <cw3_world_spawner/Task3Service.h>
#include <cw3_world_spawner/TaskSetup.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

typedef pcl::Vertices Vertices;

// typedef pcl::PointXYZRGBA PointD;
// typedef pcl::PointCloud<PointD> PointE;
// typedef PointE::Ptr PointEPtr;

class CW3 
{
  public:
    /** \brief Class constructor.
      *
      * \input[in] ROS node handle
      */
    CW3 (ros::NodeHandle &nh);

    /** \brief Service callback function for task1
      *        for pick and place.
      * \input[in] request service request message 
      * \input[in] move arm to object_loc to pick up object
      * \input[in] move arm to goal_loc to place object
      */
    bool
    task1Callback(cw3_world_spawner::Task1Service::Request &request,
      cw3_world_spawner::Task1Service::Response &response);

    /** \brief Service callback function for task2
      *        for object detection and localization
      * \input[in] request service request message containing
      * r g b values of colour of object that neesds to 
      * be localised
      */
    bool
    task2Callback(cw3_world_spawner::Task2Service::Request &request,
      cw3_world_spawner::Task2Service::Response &response);

    /** \brief Service callback function for task3 
      *        to pick up blue objects
      * \input[in] request service request message containing
      * r g b values of colour of object that needs to 
      * be picked and placed into the goal location
      */
    bool
    task3Callback(cw3_world_spawner::Task3Service::Request &request,
      cw3_world_spawner::Task3Service::Response &response);

    /** \brief Pick an object up with a given position.
      * 
      * \input[in] position the xyz coordinates where the gripper converges
      */
    bool
    pick(geometry_msgs::Point position, double angle);

    /** \brief MoveIt function for moving the move_group to the target position.
      *
      * \input[in] target_pose pose to move the arm to
      *
      * \return true if moved to target position 
      */
    bool 
    moveArm(geometry_msgs::Pose target_pose);

    /** \brief MoveIt function to move back to starting position.
      *
      * \input[in] target joint values to move the arm to
      *
      * \return true if moved to target position 
      */
    bool
    moveJointPosition(const std::vector< double > &group_variable_values);

    bool
    moveScanPosition(bool scan_front);

    bool
    moveScanPositionT3a(void);

    bool
    moveScanPositionT3b(void);

    bool
    moveScanPositionQ3(int pose);

    /** \brief MoveIt function to perform a cartesian movement when picking cubes.
      *
      * \input[in] target pose to move the arm to
      *
      * \return true if moved to target position 
      */
    bool
    moveCartesian(geometry_msgs::Pose current_pose,
      geometry_msgs::Pose target_pose);

    /** \brief MoveIt function for moving the gripper fingers to a new position. 
      *
      * \input[in] width desired gripper finger width
      *
      * \return true if gripper fingers are moved to the new position
      */
    bool 
    moveGripper(float width);

    /** \brief Drop a picked object at a given position.
      * 
      * \input[in] position, xyz coordinates of the box
      */
    bool
    drop(geometry_msgs::Point position, double angle);

    /** \brief Filter out point cloud based on RGB values
      * 
      * \input[in] position, xyz coordinates of the box
      */
    void
    colourFilter(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);


    /** \brief MoveIt function for adding a cuboid collision object in RViz
      * and the MoveIt planning scene.
      *
      * \input[in] object_name name for the new object to be added
      * \input[in] centre point at which to add the new object
      * \input[in] dimensions dimensions of the cuboid to add in x,y,z
      * \input[in] orientation rotation to apply to the cuboid before adding
      */
    void
    addCollisionObject(std::string object_name, geometry_msgs::Point centre, 
    geometry_msgs::Vector3 dimensions, geometry_msgs::Quaternion orientation);

    void
    removeCollisionObject(std::string object_name, geometry_msgs::Point centre, 
    geometry_msgs::Vector3 dimensions, geometry_msgs::Quaternion orientation);


    /** \brief Apply Voxel Grid filtering.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      */
    void
    applyVX (PointCPtr &in_cloud_ptr,
              PointCPtr &out_cloud_ptr);

    /** \brief Apply Pass Through filtering.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      */
    void
    applyPT (PointCPtr &in_cloud_ptr,
              PointCPtr &out_cloud_ptr);


    /** \brief Normal estimation.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    findNormals (PointCPtr &in_cloud_ptr);

    void
    RGBFilter (PointCPtr &in_cloud_ptr,
                PointCPtr &out_cloud_ptr);

    /** \brief Segment plane for cube tasks.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    segPlane (PointCPtr &in_cloud_ptr);

    void
    segFloor (PointCPtr &in_cloud_ptr,
                  PointCPtr &out_cloud_ptr);

    void
    removeStack (PointCPtr &in_cloud_ptr,
                  PointCPtr &out_cloud_ptr);

    void
    showStack (PointCPtr &in_cloud_ptr,
                  PointCPtr &out_cloud_ptr);


    /** \brief Point Cloud publisher.
      * 
      *  \input pc_pub ROS publisher
      *  \input pc point cloud to be published
      */
    void
    pubFilteredPCMsg (ros::Publisher &pc_pub, PointC &pc);

    /** \brief Point Cloud publisher.
      * 
      *  \input pc_pub ROS publisher
      *  \input pc point cloud to be published
      */
    void
    pubNormal (geometry_msgs::PoseStamped &normal_msg);

    /** \brief Publish the point.
      * 
      *  \input[in] cube_pt_msg cube's geometry point
      *  
      */
    void
    publishPoint (geometry_msgs::PointStamped &cube_pt_msg);

    void
    publishPose (geometry_msgs::PoseStamped &cube_pt_msg);

    /** \brief Publish the filtered point cloud.
    * 
    *  \input[in] publisher and filtered point cloud
    *  
    */
    void
    pubFilteredRGB (ros::Publisher &pc_pub, PointC &pc);

    /** \brief Perform Euclidean Clustering to find clusters of cubes
    * 
    *  \input[in] in_cloud_ptr the input PointCloud2 pointer
    *  
    */
    void
    euclideanCluster (PointCPtr &in_cloud_ptr);

    // void
    // conditionalEuclideanCluster (PointCPtr &in_cloud_ptr);

    // bool
    // enforceIntensitySimilarity (const PointT& point_a, 
    //   const PointT& point_b, float squared_distance );

 
    /** \brief Add golf tiles, basket (and red and purple cubes in task 3)
    * as collision items  
    * 
    *  \input[in] location of basket
    */
    void 
    addCollisionItems (geometry_msgs::Point &goal_location);
    

    void
    findCubeAngle (double x, double y, double z);

    void
    colourOfCloud (double r, double g, double b);

    std::multimap< double, std::string, std::less<double> >
    findColourInStack (void);

    std::multimap< double, std::string, std::less<double> >
    findColourInStackQ3 (void);

    void
    findTallestStack(void);

    /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
      * these are defined in urdf. */
    moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
    moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

    /** \brief MoveIt interface to interact with the moveit planning scene 
      * (eg collision objects). */
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    // moveit::planning_interface::MoveGroupInterface::Plan g_my_plan;

  public:

    /** \brief  service servers for advertising ROS services  */
    ros::ServiceServer task1_srv_, task2_srv_, task3_srv_;

    /** \brief Node handle. */
    ros::NodeHandle g_nh;
    
    /** \brief The input point cloud frame id. */
    std::string g_input_pc_frame_id_;

    /** \brief ROS publishers. */
    ros::Publisher g_pub_cloud;

    ros::Publisher g_pub_plane;

    ros::Publisher g_pub_colour_filtered;

    ros::Publisher g_pub_cloud_filtered2;

    /** \brief ROS publishers. */
    ros::Publisher g_pub_cloud_centroid;

    ros::Publisher g_pub_purple_red;

    ros::Publisher g_pub_crop_hull, g_pub_crop_stack, g_pub_crop_stack_rgb;

    ros::Publisher g_pub_norm, g_pub_crop_out_stack_rgb;

    /** \brief ROS geometry message point. */
    geometry_msgs::PointStamped g_rpc_pt_msg;
    
    /** \brief ROS pose publishers. */
    ros::Publisher g_pub_pose, g_pub_pose_test;
            
    /** \brief ROS pose publishers. */
    ros::Publisher g_pub_point, g_pub_stack_point;

    /** \brief Point Cloud (filtered) pointer. */
    PointCPtr g_cloud_filtered, g_cloud_filtered2;

    /** \brief Point Cloud (filtered) pointer. */
    PointCPtr g_cloud_filtered_out_floor, g_cloud_floor;
    
    /** \brief Point Cloud (filtered) pointer. */
    PointCPtr g_cloud_filtered_colour, g_cloud_filtered_stack;

    PointCPtr g_cloud_filtered_stack_rgb, g_cloud_filtered_out_stack;

    PointCPtr g_cloud_filtered_out_stack_rgb;

    /** \brief Point Cloud (filtered) sensros_msg for publ. */
    sensor_msgs::PointCloud2 g_cloud_filtered_msg, g_cloud_normal_msg;
    
    /** \brief Point Cloud (input). */
    pcl::PCLPointCloud2 g_pcl_pc;
    
    /** \brief Point Cloud (input). */
    pcl::PCLPointCloud2 g_pcl_pc_red_purple;

    /** \brief Voxel Grid filter. */
    pcl::VoxelGrid<PointT> g_vx;
    
    /** \brief Pass Through filter. */
    pcl::PassThrough<PointT> g_pt;
        
    /** \brief KDTree for nearest neighborhood search. */
    pcl::search::KdTree<PointT>::Ptr g_tree_ptr;

    /** \brief KDTree for nearest neighborhood search. */
    pcl::search::KdTree<PointT>::Ptr g_tree_cluster_ptr;
    
    /** \brief Normal estimation. */
    pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
    
    /** \brief Point Normal estimation. */
    pcl::NormalEstimation<PointT, pcl::PointNormal> g_pne;
    
    /** \brief Cloud of normals. */
    pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals, g_cloud_normals2;

    /** \brief Cloud of normals. */
    pcl::PointCloud<pcl::PointNormal>::Ptr g_cloud_normal;
    
    // /** \brief Nearest neighborhooh size for normal estimation. */
    // double g_k_nn;
    
    /** \brief SAC segmentation. */
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg; 
    
    /** \brief Extract point cloud indices. */
    pcl::ExtractIndices<PointT> g_extract_pc;
  
    /** \brief Extract point cloud normal indices. */
    pcl::ExtractIndices<pcl::Normal> g_extract_normals;
    
    /** \brief Point indices for plane. */
    pcl::PointIndices::Ptr g_inliers_plane;
          
    /** \brief Model coefficients for the plane segmentation. */
    pcl::ModelCoefficients::Ptr g_coeff_plane;
    
    
    /** \brief Point cloud to hold plane and cylinder points. */
    PointCPtr g_cloud_plane;
    
    /** \brief cw1Q1: TF listener definition. */
    tf::TransformListener g_listener_;
    
        /** \brief cw1Q1: TF listener definition. */
    tf::TransformListener g_listener2_;

    pcl::ConditionalRemoval<PointT> color_filter, color_filter_red_purple, floor_remove;


    /** \brief Point indices for rgb cloud. */
    PointCPtr g_rgb_cloud, g_cloud_filtered_rgb, 
      g_cloud_crop_hull, g_red_purple_cube_cloud;

    /** \brief Quaternion to define box and grasp orientation */
    tf2::Quaternion q_x180deg; // q_result, q_result2;
    
    geometry_msgs::Quaternion g_box_orientation, grasp_orientation;

    /** \brief Point and dimensions defined for collision items. */
    geometry_msgs::Point g_floor_centre, g_cube_centre, g_box_centre, g_box_location;

    geometry_msgs::Point g_stack_point_q2, g_stack_point_q3, g_tallest_point, g_goal_stack_point_q3;
    
    geometry_msgs::Vector3 g_floor_dimensions, g_cube_dimensions, g_box_dimensions;

    std::string g_floor = "floor", g_box = "box", g_cube = "cube", 
      g_target_frame = "world", g_panda_link0 = "panda_link0";
    
    std::string base_frame_ = "panda_link0";

    std::vector<double> g_joint_group_positions_q3;

    std::string g_required_colour ;

    pcl::CropHull<PointT> g_crop_hull;

    /** \brief joint positions of robot for home position */
    std::vector<double> g_home_postion{2.51666e-06, 0.00104981,
      -3.50591e-05, -1.57243, 3.87816e-05, 1.57093, 0.784965};

    std_msgs::Float32 g_r;
    std_msgs::Float32 g_g;
    std_msgs::Float32 g_b;

    /** \brief Centroid list for task 2 and 3 */
    std::list<geometry_msgs::PointStamped> g_centroid_list;

    std::list<double> g_angle_list;

    /** \brief Centroid list of red and purple cubes */
    std::list<geometry_msgs::PointStamped> g_centroid_list2;

    std::vector<std_msgs::ColorRGBA> g_colour_list_q2, g_colour_list_q3;

    /** \brief collision objects  */
    moveit_msgs::CollisionObject g_collision_object;

    std::vector<moveit_msgs::CollisionObject> g_object_vector;

    /** \brief define robot poses for task 2 and 3 */
    
    geometry_msgs::Pose g_task2_pose ;
    geometry_msgs::Pose g_task3_pose1 ;
    geometry_msgs::Pose g_task3_pose2 ;
    geometry_msgs::Pose g_task3_pose_test ;

    /* Variables */   
    const tf2Scalar q_x180deg_x = -1;
    const tf2Scalar q_x180deg_y = 0;
    const tf2Scalar q_x180deg_z = 0;
    const tf2Scalar q_x180deg_w = 0;
    
    /** \brief Define some useful constant values */
    double gripper_open_ = 80e-3;
    double gripper_closed_ = 0.0;

    /** \brief Parameters to define the pick and place operation */
    double g_pick_offset_ = 0.095; //0.095
    double pick_offset_task1 = 0.125; //0.125
    double pick_offset_task3 = 0.095; //0.125
    double angle_offset_ = 3.14159 / 4.0;
    double angle_offset1_ = 0;
    double angle_offset2_ = 3.14159 / 8.0;
    double angle_offset_scan_ = 3.14159 / 10.0;
    double angle_offset_scan2_ = 3.14159 / 18.0;; //3.14159 / 18.0;

    double joint_offset1_ = 3.14159 / 2;
    double joint_offset2_ = 3.14159 ;

    // double angle_offset3_ = 0.288;
    double approach_distance_ = 0.18;
    double g_drop_distance_ = 0.16;
    double approach_box_ = 0.15;    // pcl::PackedRGBComparison<PointT>::Ptr

    double home_pose_ = 0;
    double box_centre_offset = 0.1;

    double g_task1_angle;
    double g_scan_pose_x = 0.35;
    double g_scan_pose_x2 = -0.30;
    double g_scan_pose_y = -0.45;
    double g_scan_pose_z = 0.7;


    // double g_scan_pose_z = 0.8;

    double g_task1_pose1_x = 0.38;
    double g_task1_pose1_y = -0.09;
    double g_task1_pose1_z = 0.2;


    double g_task3_pose1_x = 0.1;
    double g_task3_pose1_y = 0;
    double g_task3_pose1_z = 0.85; 
    double g_task3_pose2_x = -0.1;

    const double stack_point_z = 0;
    double g_stack_rotation;
    double cube_angle = 3.14159 / 2.0;
    // std_msgs::Float32 g_stack_rotation_q1;

    const double tallest_point_init = 0;

    float g_stack_rotation_q2;


    /** \brief Parameters to define RGB and hue values and tolerances */

    // BOX_COLORS = {'purple': [0.8, 0.1, 0.8],      
    //           'red':    [0.8, 0.1, 0.1], 
    //           'blue':   [0.1, 0.1, 0.8],
    //           'yellow': [1.0, 1.0, 0.0],
    //           'orange': [0.9, 0.4, 0.1],
    //           'pink':   [0.9, 0.7, 0.7]}

 
    double g_rgb_convert = 100;

    double g_blue_r_value = 10;
    double g_blue_g_value = 10;
    double g_blue_b_value = 80;

    double g_purple_r_value = 80;
    double g_purple_g_value = 10;
    double g_purple_b_value = 80;

    double g_red_r_value = 80;
    double g_red_g_value = 10;
    double g_red_b_value = 10;

    double g_orange_r_value = 90;
    double g_orange_g_value = 40;
    double g_orange_b_value = 10;

    double g_yellow_r_value = 100;
    double g_yellow_g_value = 100;
    double g_yellow_b_value = 0;

    double g_pink_r_value = 90;
    double g_pink_g_value = 70;
    double g_pink_b_value = 70;

    double g_hue, g_sat, g_int;

    double g_hue_blue = -84.7;
    double g_hue_purple = -42.3;
    double g_hue_red = 0.97;
    double g_hue_orange = 21.2;
    double g_hue_yellow = 42.3;
    double g_hue_pink = 0; //-9.17

    double g_sat_blue = 168.3;
    double g_sat_purple = 204;
    double g_sat_red = 168;
    double g_sat_orange = 191;
    double g_sat_yellow = 247;
    double g_sat_pink = 21.6; 
    
    double g_int_blue = 35;
    double g_int_purple = 58;
    double g_int_red = 35;
    double g_int_orange = 48;
    double g_int_yellow = 67;
    double g_int_pink = 77; 

    const double g_rbg_tolerance = 10;
    const double g_hsi_tolerance = 20;
    const double g_xyz_tolerance = 0.06;

    double g_tile_r = 0;
    double g_tile_g = 60;
    double g_tile_b = 0;


    /** \brief Parameters to define voxel grid and pass through filters */
    double g_vg_leaf_sz = 0.01; // VoxelGrid leaf size 0.01
    double g_pt_thrs_min = -0.05; // PassThrough min thres -0.2
    double g_pt_thrs_max = 4.0; // PassThrough max thres 2.0
    double g_k_nn = 50; // Normals nn size
    double seg_max_it = 150;
    double seg_dist_thres = 0.08;
    double seg_dist_thres_floor = 0.03;
    double seg_dist_thres_plane = 0.005; //0.01
    double seg_dist_weight = 0.1;


    /** \brief Parameters to define cartesian move */
    const double g_jump_threshold = 0.0;
    const double g_eef_step = 0.01;
    double g_fraction;

    /** \brief Parameters to define collision object */
    double g_box_dimensions_x = 0.2;
    double g_box_dimensions_y = 0.2;
    double g_box_dimensions_z = 0.2;

    double g_box_orientation_x = 0;
    double g_box_orientation_y = 0;
    double g_box_orientation_z = 0;
    double g_box_orientation_w = -1;

    double g_cube_dimensions_x = 0.04;
    double g_cube_dimensions_y = 0.04;
    double g_cube_dimensions_z = 0.04;

    double g_floor_dimensions_x = 2;
    double g_floor_dimensions_y = 2;
    double g_floor_dimensions_z = 0.032;

    double g_floor_centre_x = 0;
    double g_floor_centre_y = 0;
    double g_floor_centre_z = 0;

    /** \brief Parameters to define Euclidean Clustering */
    double g_cluster_tolerance = 0.030; //0.01 or 0.02 o r0.035
    double min_cluster_size = 2;
    double max_cluster_size = 20;

    /** \brief Parameters to Crop Hull filter */
    double concave_hull_alpha = 20;

  protected:
    /** \brief Debug mode. */
    bool debug_;

};
    // pcl::PackedRGBComparison<PointT>::Ptr
    //   red_condition_lb, red_condition_ub;

    // pcl::PackedRGBComparison<PointT>::Ptr
    //   green_condition_lb, green_condition_ub;

    // pcl::PackedRGBComparison<PointT>::Ptr
    //   blue_condition_lb, blue_condition_ub;

    // pcl::PackedHSIComparison<PointT>::Ptr    // pcl::PackedRGBComparison<PointT>::Ptr
    //   red_condition_lb, red_condition_ub;

    // pcl::PackedRGBComparison<PointT>::Ptr
    //   green_condition_lb, green_condition_ub;

    // pcl::PackedRGBComparison<PointT>::Ptr
    //   blue_condition_lb, blue_condition_ub;

    // pcl::PackedHSIComparison<PointT>::Ptr
    //   hue_condition_lb, hue_condition_ub;

    // pcl::PackedRGBComparison<PointT>::Ptr
    //   red_purple_r_lb, red_purple_r_ub;

    // pcl::PackedRGBComparison<PointT>::Ptr
    //   red_purple_g_lb, red_purple_g_ub;

    // pcl::ConditionAnd<PointT>::Ptr g_color_cond, g_color_cond_red_purple;
    //   hue_condition_lb, hue_condition_ub;

    // pcl::PackedRGBComparison<PointT>::Ptr
    //   red_purple_r_lb, red_purple_r_ub;

    // pcl::PackedRGBComparison<PointT>::Ptr
    //   red_purple_g_lb, red_purple_g_ub;

    // pcl::ConditionAnd<PointT>::Ptr g_color_cond, g_color_cond_red_purple;