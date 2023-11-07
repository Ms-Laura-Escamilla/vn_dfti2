#ifndef DFTI_HEADER_FILE
#define DFTI_HEADER_FILE

#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include <dfti2/dftiData.h>

#include <cmath>
#include <stdlib.h>
#include <chrono>
#include <ctime>


class dfti
{
public:
  dfti();
  std::ofstream logFile_;
private:
  ros::NodeHandle nh_;
  std::string logName_;
  unsigned long long int ID_ = 1;
  void dataCallback(const dfti2::dftiData::ConstPtr& msg);
  ros::Subscriber sub_;
};

std::string format(std::string str);

#endif
