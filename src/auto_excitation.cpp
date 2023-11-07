/*
 * Copyright (c) 2019 Kameron Eves TAMU VSCL.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyrighhttps://github.com/tamu-vscl/dfti2.gitt holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software wit#include <ros/ros.h>hout specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
The purpose of this node is to inject some signal into the outputs of the
rosflight board. This is useful, among other things, for system identification
*/

#include <cmath>
#include <vector>

#include <ros/ros.h>

#include <dfti2/auto_excitation.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_excitation");
  AutoExcitation auto_excitation;
  ROS_INFO("Auto excitation is ready!");
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
}

AutoExcitation::AutoExcitation()
{
  offboard_override_pub_ = nh_.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",0);
  data_pub_ = nh_.advertise<dfti2::dftiData>("dfti_data",1000);
  rc_in_sub_ = nh_.subscribe("/mavros/rc/in", 1000, &AutoExcitation::receive_rc_in_callback, this);

  load_param();

  arm_disturb_input_svc_ = nh_.advertiseService("arm_auto_excitation", &AutoExcitation::arm_auto_excitation_callback, this);
  arm_calibration_svc_ = nh_.advertiseService("arm_control_surface_calibration", &AutoExcitation::arm_calibration_callback, this);
}

void AutoExcitation::receive_rc_in_callback(const mavros_msgs::RCIn &rc_in)
{
  rc_trigger_channel_value = rc_in.channels[trigger_channel_];
}

bool AutoExcitation::arm_auto_excitation_callback(std_srvs::Trigger::Request  &req,
                                                  std_srvs::Trigger::Response &res)
{
  res.success = false;

  load_param();
  boost::shared_ptr<mavros_msgs::RCIn const> rc_raw_msg;
  ROS_WARN("Auto excitation armed!");
  double arm_time = ros::Time::now().toSec();
  ros::Rate loop_rate(update_rate_);
  while (1)
  {
   
    if(rc_trigger_channel_value>1900)
    {
      ROS_WARN("Initiating auto excitation!");
      break;
    }

    if ((ros::Time::now().toSec()-arm_time)>=arm_time_out_time_)
    {
      ROS_INFO("The auto excitation arm has timed out.");
      return res.success;
    }

    loop_rate.sleep();
  }

  loop_rate = ros::Rate(update_rate_);
  signal_start_time_ = ros::Time::now().toSec();
  mavros_msgs::OverrideRCIn msg;
  
  mavros_msgs::SetMode flt_mode;
  flt_mode.request.custom_mode = "STABILIZE";

  if (set_mode_client.call(flt_mode) &&
      flt_mode.response.mode_sent)
  {
    ROS_INFO("STABILIZE mode enabled");
  }
  else
  {
    ROS_INFO("Failed to enable STABILIZE");
  }

  ros::Duration(1.0).sleep();

  flt_mode.request.custom_mode = "MANUAL";

  if (set_mode_client.call(flt_mode) &&
      flt_mode.response.mode_sent)
  {
    ROS_INFO("MANUAL mode enabled");
  }
  else
  {
    ROS_INFO("Failed to enable MANUAL");
  }

  while (!res.success)
  {
    
    update_signals();

    for (int i = 0;i<8;i++)
    {
      msg.channels[i] = signal_[i].value;
    }

    offboard_override_pub_.publish(msg);
    // TODO: add multi input

    res.success = true;
    for(int i = 0;i<8;i++){
      if(signal_[i].percent_of_period < signal_[i].length)
      {
        res.success = false;
        break;
      }
    }

    if(rc_trigger_channel_value<=1900)
    {
      ROS_ERROR("Pilot Overrided the auto excitation!");
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Auto excitation complete.");

  return res.success;
}

bool AutoExcitation::arm_calibration_callback(dfti2::ControlSurfaceCalibration::Request &req,
                                                  dfti2::ControlSurfaceCalibration::Response &res)
{
  res.success = false;
  
  if (req.controlSurface != "LA" || req.controlSurface != "RA" || req.controlSurface != "BL" || req.controlSurface != "BR" ) // Temp vars 
  {
    return res.success;
  }
  





  dfti2::dftiData out_msg;
  out_msg.
  data_pub_.publish(out_msg);
  // indicate beginning of calibration in the log file
  // publishing to dfit node
  // call auto excitation callback function with desired sine sweeps
  // indicate end of calibration
  
  ROS_WARN((static_cast<std::string>("Starting control surface calibration: ") + req.controlSurface).c_str());
  return res.success;
}

void AutoExcitation::update_signals()
{
  for(int i = 0;i<8;i++)
  {
    if(signal_[i].type != IGNORE_CHANNEL)
    {
      signal_[i].percent_of_period = (ros::Time::now().toSec() - (signal_start_time_ + signal_[i].delay))/signal_[i].period;
    }
    if(0<=signal_[i].percent_of_period && signal_[i].percent_of_period<=signal_[i].length){update_signal(i);}
    else{signal_[i].value = IGNORE_CHANNEL;}
  }
}

void AutoExcitation::update_signal(int i)
{
 // ROS_INFO("Signal Type: %i",signal_[i].type);
  switch(signal_[i].type)
  {
  case SIGNAL_TYPE_OFFSET:
    signal_[i].value = signal_[i].offset;
    break;
  case SIGNAL_TYPE_SQUARE: // Note that the square wave and ramp offset is bassed off of the zero point. i.e. an offset of 0 will cause the wave to oscilate between 0 and 1 (if the amplitude is 1).
    signal_[i].value = (signal_[i].percent_of_period-floor(signal_[i].percent_of_period))>=0.5 ? signal_[i].amplitude+signal_[i].offset : signal_[i].offset;
    break;
  case SIGNAL_TYPE_SAWTOOTH: // Note that the square wave and ramp offset is bassed off of the zero point. i.e. an offset of 0 will cause the wave to oscilate between 0 and 1 (if the amplitude is 1).
    signal_[i].value = signal_[i].amplitude*floor(signal_[i].percent_of_period) + signal_[i].offset;
    break;
  case SIGNAL_TYPE_SINE: // Note that the sin wave offset is bassed off of the middle point. i.e. an offset of 0 will cause the wave to oscilate between -1 and 1 (if the amplitude is 1).
    signal_[i].value = signal_[i].amplitude*sin(signal_[i].percent_of_period*2*PI) + signal_[i].offset;
    break;
  case SIGNAL_TYPE_SINE_SWEEP:
    signal_[i].value = signal_[i].amplitude*sin(2*PI*((signal_[i].f1-signal_[i].f0)/2*pow(signal_[i].percent_of_period,2)*signal_[i].period + signal_[i].f0*signal_[i].percent_of_period*signal_[i].period)) + signal_[i].offset;
    break;
  case SIGNAL_TYPE_THROTTLE_MAX_IDLE:
    if (signal_[i].percent_of_period<=0.10) {
       signal_[i].value = 1871;
    }
    else
       signal_[i].value = 1262;
    break;
  case SIGNAL_TYPE_DOUBLET:
   // ROS_INFO("Percent of Period: %f",signal_[i].percent_of_period);
    if (signal_[i].percent_of_period<=0.5) {
       signal_[i].value = signal_[i].offset + signal_[i].amplitude;
       //signal_[i].value = signal_[i].offset;
    }
    else if (signal_[i].percent_of_period>0.5) {
       signal_[i].value = signal_[i].offset - signal_[i].amplitude;
	//signal_[i].value = signal_[i].amplitude;	
    }
    else // This will set control surface at the offset/trim value as a fail-safe if other cases fail
      signal_[i].value = signal_[i].offset;
    break;
  case SIGNAL_TYPE_TRIPLET:
    if (signal_[i].percent_of_period<=1.0/3.0) {
       signal_[i].value = signal_[i].offset + signal_[i].amplitude;
    }
    else if (signal_[i].percent_of_period>1.0/3.0 && signal_[i].percent_of_period<=2.0/3.0) {
       signal_[i].value = signal_[i].offset - signal_[i].amplitude;	
    }
    else if (signal_[i].percent_of_period>2.0/3.0) {
       signal_[i].value = signal_[i].offset + signal_[i].amplitude;
    }       
    else // This will set control surface at the offset/trim value as a fail-safe if other cases fail
      signal_[i].value = signal_[i].offset;
    break;
  case SIGNAL_TYPE_T2O2:
    if (signal_[i].percent_of_period<=3.0/7.0) {
      signal_[i].value = signal_[i].offset + signal_[i].amplitude;
    }
    else if (signal_[i].percent_of_period>3.0/7.0 && signal_[i].percent_of_period<=5.0/7.0) {
      signal_[i].value = signal_[i].offset - signal_[i].amplitude;
    }
    else if (signal_[i].percent_of_period>5.0/7.0 && signal_[i].percent_of_period<=6.0/7.0 ) {
      signal_[i].value = signal_[i].offset + signal_[i].amplitude;
    }
    else if (signal_[i].percent_of_period>6.0/7.0) {
      signal_[i].value = signal_[i].offset - signal_[i].amplitude;
    }
    else // This will set control surface at the offset/trim value as a fail-safe if other cases fail
      signal_[i].value = signal_[i].offset;
    break;
  case SIGNAL_TYPE_M2_MAX:
   if (signal_[i].percent_of_period <= 2.0/signal_[i].period) {
     signal_[i].value = 1871;
   }
   else if (signal_[i].percent_of_period > 2.0/signal_[i].period) {
     signal_[i].value = 1262;
   }
   else // This will set motor to idle as a fail-safe
     signal_[i].value = 1262;
   break;

  case SIGNAL_TYPE_SCHROEDER3:
   signal_[i].value = signal_[i].offset + signal_[i].amplitude*sqrt(signal_[i].p1)*cos((PI*signal_[i].k1*signal_[i].percent_of_period*signal_[i].period)+signal_[i].phi1) + signal_[i].amplitude*sqrt(signal_[i].p2)*cos((PI*signal_[i].k2*signal_[i].percent_of_period*signal_[i].period)+signal_[i].phi2) + signal_[i].amplitude*sqrt(signal_[i].p3)*cos((PI*signal_[i].k3*signal_[i].percent_of_period*signal_[i].period)+signal_[i].phi3);
   break;
  case SIGNAL_TYPE_SCHROEDER4:
   signal_[i].value = signal_[i].offset + signal_[i].amplitude*sqrt(signal_[i].p1)*cos((PI*signal_[i].k1*signal_[i].percent_of_period*signal_[i].period)+signal_[i].phi1) + signal_[i].amplitude*sqrt(signal_[i].p2)*cos((PI*signal_[i].k2*signal_[i].percent_of_period*signal_[i].period)+signal_[i].phi2) + signal_[i].amplitude*sqrt(signal_[i].p3)*cos((PI*signal_[i].k3*signal_[i].percent_of_period*signal_[i].period)+signal_[i].phi3) + signal_[i].amplitude*sqrt(signal_[i].p4)*cos((PI*signal_[i].k4*signal_[i].percent_of_period*signal_[i].period)+signal_[i].phi4);
   break;
  case SIGNAL_TYPE_SCHROEDER5:
   signal_[i].value = signal_[i].offset +  signal_[i].amplitude*sqrt(signal_[i].p1)*cos((PI*signal_[i].k1*signal_[i].percent_of_period*signal_[i].period)+signal_[i].phi1) + signal_[i].amplitude*sqrt(signal_[i].p2)*cos((PI*signal_[i].k2*signal_[i].percent_of_period*signal_[i].period)+signal_[i].phi2) + signal_[i].amplitude*sqrt(signal_[i].p3)*cos((PI*signal_[i].k3*signal_[i].percent_of_period*signal_[i].period)+signal_[i].phi3) + signal_[i].amplitude*sqrt(signal_[i].p4)*cos((PI*signal_[i].k4*signal_[i].percent_of_period*signal_[i].period)+signal_[i].phi4) + signal_[i].amplitude*sqrt(signal_[i].p5)*cos((PI*signal_[i].k5*signal_[i].percent_of_period*signal_[i].period)+signal_[i].phi5);
   break;
  case SIGNAL_TYPE_A_DOUBLET1:
   // ROS_INFO("Percent of Period: %f",signal_[i].percent_of_period);
    if (signal_[i].percent_of_period<=0.5) {
       signal_[i].value = signal_[i].offset + signal_[i].amplitude;
       //signal_[i].value = signal_[i].offset;
    }
    else if (signal_[i].percent_of_period>0.5) {
       signal_[i].value = signal_[i].offset - 0.75*signal_[i].amplitude;
	//signal_[i].value = signal_[i].amplitude;	
    }
    else // This will set control surface at the offset/trim value as a fail-safe if other cases fail
      signal_[i].value = signal_[i].offset;
    break;

  case SIGNAL_TYPE_A_DOUBLET2:
   // ROS_INFO("Percent of Period: %f",signal_[i].percent_of_period);
    if (signal_[i].percent_of_period<=0.5) {
       signal_[i].value = signal_[i].offset + signal_[i].amplitude;
       //signal_[i].value = signal_[i].offset;
    }
    else if (signal_[i].percent_of_period>0.5) {
       signal_[i].value = signal_[i].offset - 0.5*signal_[i].amplitude;
	//signal_[i].value = signal_[i].amplitude;	
    }
    else // This will set control surface at the offset/trim value as a fail-safe if other cases fail
      signal_[i].value = signal_[i].offset;
    break;

  case SIGNAL_TYPE_A_DOUBLET3:
   // ROS_INFO("Percent of Period: %f",signal_[i].percent_of_period);
    if (signal_[i].percent_of_period<=0.5) {
       signal_[i].value = signal_[i].offset + signal_[i].amplitude;
       //signal_[i].value = signal_[i].offset;
    }
    else if (signal_[i].percent_of_period>0.5) {
       signal_[i].value = signal_[i].offset - 0.25*signal_[i].amplitude;
	//signal_[i].value = signal_[i].amplitude;	
    }
    else // This will set control surface at the offset/trim value as a fail-safe if other cases fail
      signal_[i].value = signal_[i].offset;
    break;
  }
}

void AutoExcitation::load_param()
{
  std::vector<int> type;
  std::vector<double> period;
  std::vector<double> amplitude;
  std::vector<double> offset;
  std::vector<double> f0;
  std::vector<double> f1;
  std::vector<double> k1;
  std::vector<double> k2;
  std::vector<double> k3;
  std::vector<double> k4;
  std::vector<double> k5;
  std::vector<double> p1;
  std::vector<double> p2;
  std::vector<double> p3;
  std::vector<double> p4;
  std::vector<double> p5;
  std::vector<double> M;
  std::vector<double> delay;
  std::vector<double> length;

  // load parameters
  nh_.getParam("type",type);
  nh_.getParam("period",period);
  nh_.getParam("amplitude",amplitude);
  nh_.getParam("offset",offset);
  nh_.getParam("f0",f1);
  nh_.getParam("f1",f0);
  nh_.getParam("k1",k1);
  nh_.getParam("k2",k2);
  nh_.getParam("k3",k3);
  nh_.getParam("k4",k4);
  nh_.getParam("k5",k5);
  nh_.getParam("p1",p1);
  nh_.getParam("p2",p2);
  nh_.getParam("p3",p3);
  nh_.getParam("p4",p4);
  nh_.getParam("p5",p5);
  nh_.getParam("M",M);
  nh_.getParam("delay",delay);
  nh_.getParam("length",length);

  for(int i = 0;i<8;i++)
  {
    signal_[i].type = type[i];
    signal_[i].period = period[i];
    signal_[i].amplitude = amplitude[i];
    signal_[i].offset = offset[i];
    signal_[i].f0 = f0[i];
    signal_[i].f1 = f1[i];
    signal_[i].k1 = k1[i];
    signal_[i].k2 = k2[i];
    signal_[i].k3 = k3[i];
    signal_[i].k4 = k4[i];
    signal_[i].k5 = k5[i];
    signal_[i].p1 = p1[i];
    signal_[i].p2 = p2[i];
    signal_[i].p3 = p3[i];
    signal_[i].p4 = p4[i];
    signal_[i].p5 = p5[i]; 
    signal_[i].M = M[i];
    signal_[i].phi1 = 0.0;
    signal_[i].phi2 = signal_[i].phi1 - (PI*pow(signal_[i].k2,2))/signal_[i].M;
    signal_[i].phi3 = signal_[i].phi2 - (PI*pow(signal_[i].k3,2))/signal_[i].M;
    signal_[i].phi4 = signal_[i].phi3 - (PI*pow(signal_[i].k4,2))/signal_[i].M;
    signal_[i].phi5 = signal_[i].phi4 - (PI*pow(signal_[i].k5,2))/signal_[i].M;
    signal_[i].delay = delay[i];
    signal_[i].percent_of_period = 0;
    signal_[i].value = IGNORE_CHANNEL;
    signal_[i].length = length[i];
  }

  nh_.getParam("arm_time_out_time",arm_time_out_time_);
  nh_.getParam("trigger_channel",trigger_channel_);
  nh_.getParam("update_rate",update_rate_);
}
