#include <ros/ros.h>
#include <dfti2/dfti.h>
#include <string>
#include <vector>
#include <stdlib.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "dfti");
  dfti dfti_node;
  ros::spin();
  dfti_node.logFile_.close();
}


dfti::dfti()
{
  // Where to store files

  int run; nh_.getParam("run",run);
  std::string path = getenv("HOME");
  logName_ = path + "/DFTI_run_" + std::to_string(run) + ".csv";

  // Create file
  ROS_INFO("Creating DFTI log file: %s",logName_.c_str());
  logFile_.open(logName_);
  logFile_ << std::fixed;
  logFile_ << "ID,type,time,data\n";
  sub_ = nh_.subscribe("dfti_data", 1000, &dfti::dataCallback, this);
  ROS_INFO("DFTI READY: Now logging data.");
}

void dfti::dataCallback(const dfti2::dftiData::ConstPtr& msg)
{
  logFile_ << ID_ << "," << msg->type << "," << ((double)msg->header.stamp.sec+((double)msg->header.stamp.nsec)/1000000000.0) << "," << msg->data << "\n";
  ID_++;
}
