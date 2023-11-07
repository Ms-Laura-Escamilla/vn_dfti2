#include "ros/ros.h"
#include "dfti2/dftiData.h"
#include "nav_msgs/Odometry.h"
#include <vectornav/Ins.h>
#include <vector>

class vectornav_publisher_node
{
  public:
    vectornav_publisher_node();
  private:
    ros::NodeHandle n;
    ros::Subscriber ins_sub;
    ros::Publisher data_pub;

    void insCallback(const vectornav::Ins::ConstPtr& in_msg);
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vectornav_publisher");
  vectornav_publisher_node start;
  ros::spin();
  return 0;
}

vectornav_publisher_node::vectornav_publisher_node()
{
  data_pub = n.advertise<dfti2::dftiData>("dfti_data",1000);
  ins_sub = n.subscribe("/vectornav/INS",1000,&vectornav_publisher_node::insCallback,this);
}

void vectornav_publisher_node::insCallback(const vectornav::Ins::ConstPtr& in_msg)
{
  dfti2::dftiData out_msg;
  out_msg.header = in_msg->header;

  out_msg.type = "vn_psi";  // default to NED unless changed in the vectornav param file
  out_msg.data = in_msg->yaw;
  data_pub.publish(out_msg);
  out_msg.type = "vn_theta";
  out_msg.data = in_msg->pitch;
  data_pub.publish(out_msg);
  out_msg.type = "vn_phi";
  out_msg.data = -in_msg->roll;
  data_pub.publish(out_msg);
}
