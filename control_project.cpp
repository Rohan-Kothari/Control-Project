#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>
#include <math.h>


std_msgs::Float64MultiArray msg;
geometry_msgs::Twist m ; 
int p = 0;
double x,y,th,t,tini;
ros::Time T;
int count = 0 ;
double pi = 3.1415926535897;
void CallbackOdom (nav_msgs::Odometry msg)

{
  double a,b,c,d;
  x = msg.pose.pose.position.x;
  y = msg.pose.pose.position.y;
  a = msg.pose.pose.orientation.w;
  b = msg.pose.pose.orientation.x;
  c = msg.pose.pose.orientation.y;
  d = msg.pose.pose.orientation.z;
  th = atan2(2*(a*d+b*c),(1-2*(c*c+d*d)));

  //if (th < 0 )
  //{th =  th +2*pi;}
  //std::cout << "In calllback odom"<< std::endl;


  //std::cout << th<< std::endl;
  p =1;
 
}


void Callbacktime (rosgraph_msgs::Clock msg)

{ 
  // std::cout << msg<< std::endl;
 
  T = msg.clock;
  if ( count == 0 )
  {
     tini = T.now().toSec();  
     count ++;
  }  
  t = T.now().toSec() - tini;
  
 // std::cout << "In calllback time"<< std::endl;
  //std::cout << t<< std::endl;
  p= 2 ;
}




int main(int argc, char **argv)
{
 

   ros::init(argc, argv, "control_project");
   ros::NodeHandle n;
  double pi = 3.1415926535897;
  double xc,yc,r,wd,vd,w,v,b,zeta,xddot,yddot,xd,yd,thd,x_dddot,y_dddot,e1,e2,e3,k1,k2,k3,u1,u2,thc;
  xc = 0;
  yc = 0;
  thc = 0;
  r = 1;
  wd = 0.5;

  //Control Parameters  
  b = 1;
  zeta = 0.7;

   msg.data.resize(3);
  ros::Subscriber subodom = n.subscribe("/odom", 1000, CallbackOdom);
  ros::Subscriber subtime = n.subscribe("/clock", 1000, Callbacktime);
  ros::Publisher Mat_pub = n.advertise<std_msgs::Float64MultiArray>("/To_Matlab",1000);
  
 
  ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
  
  ros::Rate loop_rate(700); 
  
  while (ros::ok())
  {
  // std::cout << "In while"<< std::endl;
   
   ros::spinOnce();
   if (p == 2)
  {
  //  std::cout << "In code"<< std::endl;
    p =0;
     
    //std::cout << "r "<<r<< "wd " << wd <<"time "<<t<< std::endl;
   
    // circle
     xd = (xc+r*cos(wd*t+thc));
     yd = (yc+r*sin(wd*t+thc));
    


    // eight 
    //xd = (xc+r*sin(2*wd*t+thc));
    //yd = (yc+r*sin(wd*t+thc));
    



    // xd = t/5 ; 
    // yd = xd;

    // if (yd == 3)
    //   { yd = -0.5*xd;}
    
    //std::cout << "pos x d "<< std::endl;
    //std::cout << xd<< std::endl;

    //std::cout << "pose y d "<< std::endl;
    //std::cout << yd<< std::endl;

     //circle 

    xddot = -wd * r * sin(wd*t+thc);
    yddot = wd * r * cos(wd*t+thc);

    // eight 
   // xddot = 2*wd * r * cos(2*wd*t+thc);
    //yddot = wd * r * cos(wd*t+thc);

    // xddot = 1;
    // yddot = 1;

    
     //std::cout << "xd vel "<< std::endl;
    //std::cout << xddot<< std::endl;

    //std::cout << "yd vel "<< std::endl;
    //std::cout << yddot<< std::endl;

    thd = atan2(yddot,xddot);
     //if (thd < 0 )
     //{thd =  thd +2*pi;}
     //x_dddot = -pow(wd,2) * r * cos(wd*t);
    //y_dddot = -pow(wd,2) * r * sin(wd*t);
     
     vd = sqrt(pow(xddot,2) + pow(yddot,2));

     //wd = (y_dddot * x_ddot - x_dddot * y_ddot)/(x_ddot^2 + y_ddot^2);

      //std::cout << "angular d  "<< std::endl;
    //std::cout << thd<< std::endl;

   // std::cout << " vel d"<< std::endl;
    //std::cout << vd<< std::endl;

     e1 = cos(th)*(xd-x) + sin(th)*(yd-y);
     e2 = -sin(th)*(xd-x) + cos(th)*(yd-y);
     e3 = thd - th;


     msg.data[0] = e1;
     msg.data[1] = e2;
     msg.data[2] = e3;
 
    
     k1 = 2*zeta*sqrt(pow(wd,2) + b*pow(vd,2));
     k3 = k1;
     k2 = b;

     u1 = -k1 * e1;
     u2 = -k2 * vd * (sin(e3)/e3) * e2 - k3*e3;

   // k1 = 2*zeta*b;
    //k3 =k1 ; 
    //k2 = (1-pow(wd,2))/vd ;
    //u1 = -k1*e1;
    //u2 = -k2*e2 - k3*e3; 



    //std::cout << "u1 "<< std::endl;
    //std::cout << u1<< std::endl;

    //std::cout << "u2 "<< std::endl;
    //std::cout << u2<< std::endl;


     v = vd*cos(e3) - u1;
     w = wd - u2;


     //std::cout << "linear vel "<< std::endl;
    //std::cout << v<<"and"<< vd <<std::endl;

    //std::cout << "angular vel "<< std::endl;
    //std::cout << w<< "and"<< wd << std::endl;



   std::cout << "angle "<< std::endl;
    std::cout <<th<< "and" <<thd<< std::endl;

    //std::cout << "current position"<< std::endl;
    //std::cout <<x<< "and" <<y<< std::endl;
    m.linear.x =  v;
    m.angular.z = w;
   // std::cout << "Data Sent "<< std::endl;
    //std::cout << m<< std::endl;
    pub_vel.publish(m);
    Mat_pub.publish(msg); 
    

    ros::spinOnce();
  }
  ros::spinOnce();
     loop_rate.sleep();
}
 ros::spin();  

}
