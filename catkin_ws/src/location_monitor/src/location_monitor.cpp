#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include <vector>

struct Vert{

double x, y;

};

class LandmarkRegion{

public:
//vertices
std::vector<Vert> points;

//lines
std::vector<double> cofA;
std::vector<double> cofB;
std::vector<double> cofC;

LandmarkRegion( std::vector<Vert> input );
void CalculateLines();
bool DeterminePoint( Vert pos );
bool FindIntersection( double rayCofA, double rayCofB, double rayCofC, 
                       double cofA, double cofB, double cofC, double leftX, 
                       double rightX, double topY, double bottomY, Vert pos );

};

LandmarkRegion::LandmarkRegion( std::vector<Vert> input )
{

   for( int a = 0; a < input.size(); a++ ){
      points.push_back( input[a] );
   }

   CalculateLines();

}

void LandmarkRegion::CalculateLines()
{

   if(points.size() < 2){
      return;
   }

   for( int a = 0; a < points.size() - 1; a++ ){
      cofA.push_back( points[a].y - points[a+1].y );
      cofB.push_back( points[a+1].x - points[a].x );
      cofC.push_back( (points[a].x * points[a+1].y) - 
                        (points[a+1].x * points[a].y) );
   }

   cofA.push_back( points[points.size()-1].y - points[0].y );
   cofB.push_back( points[0].x - points[points.size()-1].x );
   cofC.push_back( (points[points.size()-1].x * points[0].y) - 
                        (points[0].x * points[points.size()-1].y) );

}

bool LandmarkRegion::DeterminePoint( Vert pos )
{
   if(points.size() == 0 || cofA.size() < 3)
      return false;

   //cast ray
      double rayCofA, rayCofB, rayCofC;

      rayCofA = ( 0.0 );
      rayCofB = ( 1.0 );
      rayCofC = ( -1.0 * pos.y );

   //check for intersecting lines
      bool inside = false;
      for( int a = 0; a < cofA.size();a++){
         double leftX = points[a].x;
         double rightX = points[a+1].x;
         double bottomY = points[a].y;
         double topY = points[a+1].y; 

         if( leftX > rightX ){
            double temp = leftX;
            leftX = rightX;
            rightX = temp;
         }

         if( bottomY > topY ){
            double temp = bottomY;
            bottomY = topY;
            topY = temp;
         }


         if( FindIntersection( rayCofA, rayCofB, rayCofC,
                               cofA[a], cofB[a], cofC[a], leftX, rightX,
                               topY, bottomY, pos ) ){
            inside = !inside;
         }

      }

   if(inside){
      ROS_INFO("Entering colored region");
   }

   return inside; 

   
}


//ensure leftX is greater than rightX when using
bool LandmarkRegion::FindIntersection( double rayCofA, double rayCofB, 
                                       double rayCofC, double cofA, double cofB, 
                                       double cofC, double leftX, double rightX,
                                       double topY, double bottomY, Vert pos )
{

   double pA, pB, pC;

   pA = rayCofB * cofC - cofB * rayCofC;
   pB = cofA * rayCofC - rayCofA * cofC;
   pC = rayCofA * cofB - cofA * rayCofB;

   if(pC == 0){
      return false;
   }

   pA /= pC;   //place in homogeneous coordinates
   pB /= pC;

   if( pA >= pos.x ){
      return false;
   }

   return true; //return true for contact on tested side

}


//globals
Vert robPos;   //global variable to communicate pos between callback and main

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
   robPos.x = msg->pose.pose.position.x;
   robPos.y = msg->pose.pose.position.y;

   ROS_INFO("x: %f, y: %f", robPos.x, robPos.y);
}

int main(int argc, char** argv) {

   geometry_msgs::Twist dir;
   dir.linear.x=dir.linear.y=dir.linear.z=0;
   dir.angular.x=dir.angular.y=dir.angular.z=0;

   std::vector<Vert> data;
   Vert temp;
   temp.x = 5.0;
   temp.y = -1.0;
   data.push_back( temp );
   temp.x = 7.0;
   temp.y = -1.0;
   data.push_back( temp );
   temp.x = 7.0;
   temp.y = 1.0;
   data.push_back( temp );
   temp.x = 5.0;
   temp.y = 1.0;
   data.push_back( temp );

   LandmarkRegion colorSquare( data );

   dir.linear.x = 0.2;

   ros::init(argc,argv, "location_monitor");
   ros::NodeHandle nh;
   ros::Subscriber sub = nh.subscribe( "odom", 1, OdomCallback);
   ros::Publisher pub = nh.advertise<geometry_msgs::Twist>     
                                    ("cmd_vel_mux/input/teleop", 1 );
   ros::Rate loop_rate(50);

while(ros::ok())
{
    pub.publish(dir);
    ros::spinOnce();
    if(colorSquare.DeterminePoint( robPos )){
      dir.angular.x = 25;
    }

    else{
      dir.angular.x = 0;
    }
}

   pub.publish(dir);

   return 0;

}
