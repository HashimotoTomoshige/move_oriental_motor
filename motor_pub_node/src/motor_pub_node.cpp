#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_pub_node");
  ros::NodeHandle nh;
  ros::Publisher motor_position_pub = nh.advertise<std_msgs::Int32MultiArray>("motor_position", 1);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::Int32MultiArray motor_position;

    // cout << "何か値を入れてね > ";
    // cin >> motor_position.data;
    int var;  
    printf("何か4つ値をいれてね\n");
    motor_position.data.resize(4);
    
    
    for(int i=0; i<4; i++){
      scanf("%d", &var);
      motor_position.data[i] = var;
    }

    motor_position_pub.publish(motor_position);
    ROS_INFO("published");
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}