#include <cw3_team_5/cw3_team_5.h>

int main(int argc, char** argv)
{
  // initialise ros and the node
  ros::init(argc, argv, "cw3_team_5_node");
  ros::NodeHandle nh("~");
  ROS_INFO("initialized node cw1_team_5_node");
  // MoveIt! requirement for non-blocking group.move()
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // create the service object to handle all callbacks
  CW3 CW3_object(nh);
  
  ros::Subscriber sub_rgb =
  nh.subscribe ("/r200/camera/depth_registered/points",
                1,
                &CW3::colourFilter,
                &CW3_object);


  // loop rate in Hz
  ros::Rate rate(10);

  while (ros::ok()) {

    // spin and process all pending callbacks
    ros::spinOnce();
    // ros::MultiThreadedSpinner spinner(1); // Use 4 threads
    // spinner.spin(); // spin() will not return until the node has been shutdown
    // sleep to fulfill the loop rate
    rate.sleep();
  }
}

