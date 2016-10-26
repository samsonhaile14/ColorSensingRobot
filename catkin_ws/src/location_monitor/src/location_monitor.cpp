#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
   double x = msg->pose.pose.position.x;
   double y = msg->pose.pose.position.y;
   ROS_INFO("x: %f, y: %f", x, y);
}

int main(int argc, char** argv) {

   geometry_msgs::Twist dir;
   dir.linear.x=dir.linear.y=dir.linear.z=0;
   dir.angular.x=dir.angular.y=dir.angular.z=0;

   dir.linear.x = 0.2;

   ros::init(argc,argv, "location_monitor");
   ros::NodeHandle nh;
   ros::Subscriber sub = nh.subscribe( "odom", 10, OdomCallback);
   ros::Publisher pub = nh.advertise<geometry_msgs::Twist>     
                                    ("cmd_vel_mux/input/teleop", 5 );
   ros::Rate loop_rate(50);


while(ros::ok())
{
    pub.publish(dir);
    loop_rate.sleep();
    ros::Duration(1).sleep();
    ros::spinOnce();
}

   pub.publish(dir);
   ros::Duration(1).sleep();

   pub.publish( dir );
   return 0;

}
