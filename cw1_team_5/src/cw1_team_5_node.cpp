#include <cw1_team_5/cw1_team_5.h>

int main(int argc, char** argv)
{
  // initialise ros and the node
  ros::init(argc, argv, "cw1_team_5_node");
  ros::NodeHandle nh("~");
  ROS_INFO("initialized node cw1_team_5_node");
  // MoveIt! requirement for non-blocking group.move()
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // create the service object to handle all callbacks
  CW1 CW1_object(nh);

  // ros::Subscriber sub_rgb =
  //   nh.subscribe ("/r200/camera/colour/points",
  //                 1,
  //                 &PCLTutorial::cloudCallBackOne,
  //                 &pcl_tutorial);
  
  ros::Subscriber sub_rgb =
  nh.subscribe ("/r200/camera/depth_registered/points",
                1,
                &CW1::findCentroid,
                &CW1_object);

  // loop rate in Hz
  ros::Rate rate(10);

  while (ros::ok()) {

    // spin and process all pending callbacks
    ros::spinOnce();

    // sleep to fulfill the loop rate
    rate.sleep();
  }
}

