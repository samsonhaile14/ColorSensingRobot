#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include <vector>
#include <cmath>

#define PI 3.14159265

struct Vert{

double x, y;
double vX;
double oT;

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

   if( (pos.x > points[0].x && pos.y > points[0].y) &&
       (pos.x < points[1].x && pos.y > points[1].y) &&
       (pos.x < points[2].x && pos.y < points[2].y) &&
       (pos.x > points[3].x && pos.y < points[3].y) ){
      return true;
   }

   else{
      return false;
   }


/*
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

   return inside; 
*/
   
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

//function prototypes
   void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
   double radToDegree( double x, double rad);


int main(int argc, char** argv) {

   robPos.x = robPos.y = robPos.vX = robPos.oT = 0;

   geometry_msgs::Twist dir;
   dir.linear.x=dir.linear.y=dir.linear.z=0;
   dir.angular.x=dir.angular.y=dir.angular.z=0;

   //set list of travel points
      Vert temp;
      std::vector< Vert > dest;
      temp.x = 6.0;
      temp.y = 0.0;
      dest.push_back( temp );
      temp.x = 4.0;
      temp.y = 5.0;
      dest.push_back( temp );
      temp.x = -2.0;
      temp.y = 5.0;
      dest.push_back( temp );
      temp.x = 0.0;
      temp.y = 0.0;
      dest.push_back( temp );


   //create list of colored square coordinates
      std::vector<Vert> data;
      std::vector< LandmarkRegion > colorPath;
      double halfWidth = 1.0;
      temp.x = dest[dest.size()-1].x - halfWidth;
      temp.y = dest[dest.size()-1].y - halfWidth;
      data.push_back( temp );
      temp.x = dest[dest.size()-1].x + halfWidth;
      temp.y = dest[dest.size()-1].y - halfWidth;
      data.push_back( temp );
      temp.x = dest[dest.size()-1].x + halfWidth;
      temp.y = dest[dest.size()-1].y + halfWidth;
      data.push_back( temp );
      temp.x = dest[dest.size()-1].x - halfWidth;
      temp.y = dest[dest.size()-1].y + halfWidth;
      data.push_back( temp );
      LandmarkRegion colorSquare( data );       //Class object representing colored squares
      colorPath.push_back( colorSquare );

      for( int a = 0; a < (dest.size() - 1); a++ ){
         data.clear();
         temp.x = dest[a].x - halfWidth;
         temp.y = dest[a].y - halfWidth;
         data.push_back( temp );
         temp.x = dest[a].x + halfWidth;
         temp.y = dest[a].y - halfWidth;
         data.push_back( temp );
         temp.x = dest[a].x + halfWidth;
         temp.y = dest[a].y + halfWidth;
         data.push_back( temp );
         temp.x = dest[a].x - halfWidth;
         temp.y = dest[a].y + halfWidth;
         data.push_back( temp );
         LandmarkRegion colSquare( data );       //Class object representing colored squares
         colorPath.push_back( colSquare );
      }



   dir.linear.x = 0.2;                    //preset velocity
   dir.angular.z = 0.0;

   ros::init(argc,argv, "location_monitor");
   ros::NodeHandle nh;
   ros::Subscriber sub = nh.subscribe( "odom", 1, OdomCallback);//keeps track of robot position
   ros::Publisher pub = nh.advertise<geometry_msgs::Twist>      //updates robot movement 
                              ("cmd_vel_mux/input/teleop", 1 );
   ros::Rate loop_rate(50);

   Vert targetDest;
   targetDest.x = 0;
   targetDest.y = 0;

   double pr_max = 0.3;
   double lambda = 1.0;
   double turnStrength = 0.25;

   //main loop of operation
   while(ros::ok())
   {

       for( int a = 0; a < colorPath.size(); a++ ){
         if(colorPath[a].DeterminePoint( robPos ) ){
            ROS_INFO("Driving over colored square %d", a+1);
            targetDest.x = dest[a].x;
            targetDest.y = dest[a].y;
         }
       }

      //path plan based on robotPosition and targetDest
         double dirVecX = targetDest.x - robPos.x;
         double dirVecY = targetDest.y - robPos.y;

         double distQrt = std::sqrt(dirVecX * dirVecX + dirVecY * dirVecY);

         double rho = 90.0+radToDegree(dirVecX,atan( dirVecY/dirVecX ));
         
         double newVel = sqrt( lambda * lambda * distQrt * distQrt );

         newVel = std::min( newVel, pr_max );   //cap velocity of robot

         double newTheta = rho;

      //make changes through multiplexer
         dir.linear.x = newVel;

         if(robPos.oT > newTheta){

            double valA = 360.0 - robPos.oT + newTheta;
            double valB = robPos.oT - newTheta;

            if(valA > valB){
               dir.angular.z = -1.0 * turnStrength;
            }
            else{
               dir.angular.z = turnStrength;
            }

         }

         else if( robPos.oT < newTheta ){

            double valA = newTheta - robPos.oT;
            double valB = robPos.oT + (360.0-newTheta);

            if(valA > valB){
               dir.angular.z = -1.0 * turnStrength;
            }
            else{
               dir.angular.z = turnStrength;
            }
         }

      //publish info
       pub.publish(dir);
       ros::spinOnce();

   }

   return 0;

}

//function implementations
   void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
      Vert lastPos = robPos;

      robPos.x = msg->pose.pose.position.x;
      robPos.y = msg->pose.pose.position.y;
      robPos.vX = msg->twist.twist.linear.x;
      robPos.oT = 90.0+radToDegree((robPos.x - lastPos.x),atan( (robPos.y - lastPos.y)/ (robPos.x - lastPos.x) ));
         

      ROS_INFO("x: %f, y: %f, vX: %f, oT: %f", robPos.x, robPos.y, robPos.vX, robPos.oT);

   }

   double radToDegree( double x, double rad){

      if( x < 0 ){
         return ( rad / PI ) * 180.0;
      }

      else{
         return ( rad / PI ) * 180.0 + 180.0;
      }

   }


