#ifndef AUTO_EXCITATION_H
#define AUTO_EXCITATION_H

#define SIGNAL_TYPE_IGNORE_CHANNEL 0
#define SIGNAL_TYPE_OFFSET 1
#define SIGNAL_TYPE_SQUARE 2
#define SIGNAL_TYPE_SAWTOOTH 3
#define SIGNAL_TYPE_SINE 4
#define SIGNAL_TYPE_SINE_SWEEP 5
#define SIGNAL_TYPE_THROTTLE_MAX_IDLE 6
#define SIGNAL_TYPE_DOUBLET 7
#define SIGNAL_TYPE_TRIPLET 8
#define SIGNAL_TYPE_T2O2 9
#define SIGNAL_TYPE_M2_MAX 10
#define SIGNAL_TYPE_SCHROEDER3 11
#define SIGNAL_TYPE_SCHROEDER4 12
#define SIGNAL_TYPE_SCHROEDER5 13
#define SIGNAL_TYPE_A_DOUBLET1 14
#define SIGNAL_TYPE_A_DOUBLET2 15
#define SIGNAL_TYPE_A_DOUBLET3 16

#define PI 3.14159
#define IGNORE_CHANNEL UINT16_MAX

#include <ros/ros.h>

#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include "dfti2/dftiData.h"
#include <std_srvs/Trigger.h>
#include "dfti2/ControlSurfaceCalibration.h"
#include <mavros_msgs/RCIn.h>

class AutoExcitation
{
public:

  struct Signal
  {
    // Signal parameters
    int type = SIGNAL_TYPE_IGNORE_CHANNEL; // Type of signal
    double period = 1; // Length of 1 period in seconds.
    double amplitude = 0; // Scaling factor for signal (exact implimentation varies between signal types) TODO: Units
    double offset = 1500; // Moves the signal up or down to center it around a value other then 0
    double f0 = 1.0; // Intial frequency for sine sweep
    double f1 = 10.0; // Final frequency for sine sweep
    double k1 = 1.0; // First sinusoidal component for the Schroeder Sweep 
    double k2 = 2.0; // Second sinusoidal component for the Schroeder Sweep 
    double k3 = 3.0; // Third sinusoidal component for the Schroeder Sweep 
    double k4 = 4.0; // Fourth sinusoidal component for the Schroeder Sweep 
    double k5 = 5.0; // Fifth sinusoidal component for the Schroeder Sweep 
    double p1 = 0.20; // Power fraction for first sinusoidal component for the Schroeder Sweep 
    double p2 = 0.20; // Power fraction for second sinusoidal component for the Schroeder Sweep 
    double p3 = 0.20; // Power fraction for third sinusoidal component for the Schroeder Sweep 
    double p4 = 0.20; // Power fraction for fourth sinusoidal component for the Schroeder Sweep 
    double p5 = 0.20; // power fraction for fifth sinusoidal component for the Schroeder Sweep
    double phi1 = 0.0; // Phase offset for first sinusoidal component for the Schroeder Sweep
    double phi2 = 0.0; // Phase offset for the second sinusoidal component for the Schroeder Sweep
    double phi3 = 0.0; // Phase offset for the third sinusoidal component for the Schroeder Sweep
    double phi4 = 0.0;// Phase offset for the fourth sinusoiudal component for the Schroeder Sweep
    double phi5 = 0.0; // Phase offset for the fifth sinusoidal component for the Schroeder Sweep 
    double M = 3.0; // Number of sinusoidal components for the Schroeder Sweep
    double delay = 0; // Seconds before this signal begins

    // Logistical parameters
    double percent_of_period = 0; // Tracks where in the period the signal is curently
    double value = IGNORE_CHANNEL; // Actual ouput of the signal generator
    double length = 1; // Number of periods to execute. Can be partial period(s)
  };

  AutoExcitation();
  inline const Signal& signal(int i) const { return signal_[i]; }

private:

  ros::NodeHandle nh_;
  ros::Publisher offboard_override_pub_;
  ros::Publisher data_pub_;
  ros::Subscriber rc_in_sub_;
  ros::ServiceServer arm_disturb_input_svc_;
  ros::ServiceServer arm_calibration_svc_;
  ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  bool arm_auto_excitation_callback(std_srvs::Trigger::Request  &req,
                                    std_srvs::Trigger::Response &res);

  bool arm_calibration_callback(dfti2::ControlSurfaceCalibration::Request &req,
                                    dfti2::ControlSurfaceCalibration::Response &res);

  void receive_rc_in_callback(const mavros_msgs::RCIn &rc_in);
  void update_signals();
  void update_signal(int i);
  void load_param();

  Signal signal_ [8];
  double arm_time_out_time_;
  int trigger_channel_;
  double update_rate_;
  double signal_start_time_;

  unsigned int rc_trigger_channel_value;
};

#endif // AUTO_EXCITATION_H
