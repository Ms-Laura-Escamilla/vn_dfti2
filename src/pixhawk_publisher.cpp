#include "ros/ros.h"
#include "dfti2/dftiData.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/VFR_HUD.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/RCOut.h"
#include <vector>

class pixhawk_publisher_node
{
  public:
    pixhawk_publisher_node();
  private:
    ros::NodeHandle n;
    ros::Subscriber odom_sub;
    ros::Subscriber airspeed_sub;
    ros::Subscriber rcin_sub;
    ros::Subscriber rcout_sub;
    ros::Publisher data_pub;
    std::vector<int> rcin_pins;
    std::vector<int> rcout_pins;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& in_msg);
    void airspeedCallback(const mavros_msgs::VFR_HUD::ConstPtr& in_msg);
    void rcinCallback(const mavros_msgs::RCIn::ConstPtr& in_msg);
    void rcoutCallback(const mavros_msgs::RCOut::ConstPtr& in_msg);
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pixhawk_publisher");
  pixhawk_publisher_node start;
  ros::spin();
  return 0;
}

pixhawk_publisher_node::pixhawk_publisher_node()
{
  data_pub = n.advertise<dfti2::dftiData>("dfti_data",1000);
  odom_sub = n.subscribe("/mavros/local_position/odom",1000,&pixhawk_publisher_node::odomCallback,this);
  airspeed_sub = n.subscribe("/mavros/vfr_hud",1000,&pixhawk_publisher_node::airspeedCallback,this);
  rcin_sub = n.subscribe("/mavros/rc/in",1000,&pixhawk_publisher_node::rcinCallback,this);
  rcout_sub = n.subscribe("/mavros/rc/out",1000,&pixhawk_publisher_node::rcoutCallback,this);

  n.getParam("rcin_pins", rcin_pins);
  n.getParam("rcout_pins", rcout_pins);
}

void pixhawk_publisher_node::odomCallback(const nav_msgs::Odometry::ConstPtr& in_msg)
{
  dfti2::dftiData out_msg;
  out_msg.header = in_msg->header;

  out_msg.type = "x";
  out_msg.data = in_msg->pose.pose.position.y;
  data_pub.publish(out_msg);
  out_msg.type = "y";
  out_msg.data = in_msg->pose.pose.position.x;
  data_pub.publish(out_msg);
  out_msg.type = "z";
  out_msg.data = -in_msg->pose.pose.position.z;
  data_pub.publish(out_msg);

  out_msg.type = "q0";
  out_msg.data = in_msg->pose.pose.orientation.w; // Quaternions in East-North-Up (ENU) frame
  data_pub.publish(out_msg);
  out_msg.type = "q1";
  out_msg.data = in_msg->pose.pose.orientation.x;
  data_pub.publish(out_msg);
  out_msg.type = "q2";
  out_msg.data = in_msg->pose.pose.orientation.y;
  data_pub.publish(out_msg);
  out_msg.type = "q3";
  out_msg.data = in_msg->pose.pose.orientation.z;
  data_pub.publish(out_msg);

  out_msg.type = "u";
  out_msg.data = in_msg->twist.twist.linear.x;
  data_pub.publish(out_msg);
  out_msg.type = "v";
  out_msg.data = -in_msg->twist.twist.linear.y;
  data_pub.publish(out_msg);
  out_msg.type = "w";
  out_msg.data = -in_msg->twist.twist.linear.z;
  data_pub.publish(out_msg);

  out_msg.type = "p";
  out_msg.data = in_msg->twist.twist.angular.x;
  data_pub.publish(out_msg);
  out_msg.type = "q";
  out_msg.data = -in_msg->twist.twist.angular.y;
  data_pub.publish(out_msg);
  out_msg.type = "r";
  out_msg.data = -in_msg->twist.twist.angular.z;
  data_pub.publish(out_msg);
}

void pixhawk_publisher_node::airspeedCallback(const mavros_msgs::VFR_HUD::ConstPtr& in_msg)
{
  dfti2::dftiData out_msg;
  out_msg.header = in_msg->header;
  out_msg.type = "V";
  out_msg.data = in_msg->airspeed;
  data_pub.publish(out_msg);
}

void pixhawk_publisher_node::rcinCallback(const mavros_msgs::RCIn::ConstPtr& in_msg)
{
  if(in_msg->channels.size())
  {
    dfti2::dftiData out_msg;
    out_msg.header = in_msg->header;
    for(int i = 0; i<rcin_pins.size(); i++)
    {
      out_msg.type = "I" + std::to_string(rcin_pins[i]);
      out_msg.data = in_msg->channels[rcin_pins[i]-1];
      data_pub.publish(out_msg);
    }
  }
}

void pixhawk_publisher_node::rcoutCallback(const mavros_msgs::RCOut::ConstPtr& in_msg)
{
  if(in_msg->channels.size())
  {
    dfti2::dftiData out_msg;
    out_msg.header = in_msg->header;
    for(int i = 0; i<rcout_pins.size(); i++)
    {
      out_msg.type = "O" + std::to_string(rcout_pins[i]);
      out_msg.data = in_msg->channels[rcout_pins[i]-1];
      data_pub.publish(out_msg);
    }
  }
}
