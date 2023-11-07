# DFTI2

*Note that this documentation is very rough. It's late and I'm not going to proof read it*

DFTI was a code and hardware logging solution. The idea was to log many data points at once. DFTI2 is an new code base designed to modernize DFTI. The majority of the hardware has been eliminated. All remaining is the computer running ROS, Pixhawk, (both already necessary to fly the vehicle) and an Arduino (if doing analog inputs).

## System Overview

DFIT uses ROS to log info. If you are not familiar with ROS we recommend you review the ROS tutorials.

If the dfti node from this package is running. Anything published to the topic dfti_data is logged in a log file. This file is named DFTI_run_#.csv where the run number is specified by a ROS parameter. *BE CAREFUL NOT TO USE THE SAME RUN NUMBER TWICE WITHOUT MOVING THE OLD FILES. DOING SO WILL OVERWRITE OLD DATA* These log files are saved in the computers home directory.

The dfti_data topic has the message type dftiData. It includes a typical ROS header with a timestamp. This timestamp should be recorded as close as possible to the the time the measurment is taken. Another parameter is the type which is a string indicating what type of measurment this is. Finally there is the data field which is of type double with the actual data you want to log.

### pixhawk_publisher
This package has the capability to read data from a pixhawk and a analog siginals by default. The c++ python node pixhawk_publisher publishes the state of the vehicle, the RC transmitter inputs, and the output to the motors. If desired, one of the RC Inputs can be mapped to a switch on the transmitter that the pilot can flip right before hacks.

This ros node reformats data from Ardupilot into the format needed for data logging on DFTI 2.

More specifically it does rotations into Forward Right Down format and passes on one data point at a time.

By default MAVROS uses several coordinate frames:

NED - Literally north east down on the compass rose
ENU - Literally east north up on the compass rose
FLU - Baselink - Forward left
up with respect to the vehicle
FRD - Aircraft - Forward right down with respect to the vehicle

Each of these can be earth or body fixed coordinate frames.

Also there is some discussion of a ECEF (earth centered earth fixed) coordinate frame and UTM coordinates. I have not seen the code for these though.

All units are metric.

This program uses the /mavros/local_position/odom message. From this message we get:
- pose
  - ENU position
  - ENU orientation
- twist (velocities)
  - FLU linear
  - FLU angular
This program also uses the /mavros/rc/out message. From this message we get:
  - PWM channels (signal to the ESCs for the motors)

This is then converted to
- NED Position
- Phi, Theta, Psi using their normal aviation definition of a 3-2-1 rotation (roll - pitch - yaw)
- FRD linear velocities
- FRD Angular velocities
- PWM Channels

*IMPORTANT*
Solution to the ODOM not publishing:
Make sure the ardupilot parameter EK2_GPS_TYPE is set to 0

Solution to the data not logging fast enough:
There are many potential problems that could cause this, but one hard to identify one we recently discovered is that the parameter SCHED_LOOPRATE should be 400hz. 

### arduino_publisher.py
This package also has the ability built in to read analog signials. Analog signals are read through an Arduino. Plug the ground wire in the ground of the arduino and the signal wire of the analog signal into an analog pin on the arduino. Then look up the pin number. You will need this later. Note that the numbers write on the side of the arduino is not sufficent. Each analog pin is also assigned a number without a letter before it. These values can be found in the file pins_arduino.h. An example of what I'm talking about can be found here https://forum.arduino.cc/index.php?topic=648092.0 The data for boards other then the mega can be found on the internet. Working with the mega example, say you put an analog wire in pin A5. The number associated with A5 on the arduino mega found at the link previously is 59. Finally connect the arduino via usb to the computer running dfti.

Following this, launching the file arduino_publisher python node should be sufficent. The node will find the arduino on it's own. The pins that are connected to the analog signal are specified with a ROS parameter.

### data_importer.m
After the data has been logged then the user can use the matlab file in the tools folder to convert the data to a more traditional log by interpolating the make the data point syncronus. All the user has to do is put the DFTI files in the same folder and then open the matlab folder and indicate what variable type is used as the hack indicator. The output is described in the file

## Instalation

- Make a Catkin workspace http://wiki.ros.org/catkin/Tutorials/create_a_workspace
- Clone this directory into the src folder of the catkin workspace
- catkin_make in the base directory of the workspace to compile it.
- Source devel/setup.bash       <- This needs to be done everytime you want to run this package.
- Install MAVROS via the instructions here: https://ardupilot.org/dev/docs/ros-install.html#installing-mavros

## Usage

The launch file pixhawk_logger.launch takes in various parameters as arguments and then starts all of the above components. Arguments are given in the default ROS fashion: "argument:=value" The only required argument is the run number. However, by default only the state vector is logged. If you want servo or motor commands, rc inputs, or analog siginals you need to use the parameters to specify this. AGAIN, UNLESS YOU HAVE MOVED THE OLD DATA TO A NEW FOLDER, SETTING THE RUN NUMBER TO THE SAME AS A PREVIOUS TEST WILL OVERWRITE THE PREVIOUS TEST DATA.

The other arguments you can use are rcin_pins (a array indicating which rc input channels you want to log), rcout_pins (an array indicating which motor and servo COMMANDS you want to log), and arduino_pins (an array indicating which pins on the arduino have an analog wire connected to it. This is the number you found earlier and should not begin with "A")

Here are two examples of running the code with this syntax:

roslaunch dfti2 pixhawk_loger.launch run:=1

This will create a log file called DFTI_run_1.csv in the home directory and log the state data x,y,z,u,v,w,p,q,r,q0,q1,q2,q3. where q0-4 are components of quaternian representing orentation.

roslaunch dfti2 pixhawk_logger.launch run:=1 rcin_pins:=[1,2,3,4] rcout_pins:=[6,7,8,9] arduino_pins:=[59] arduino_names:=[A]

this will log the same as the previous commands as well as the rc transmitter signal on channels 1, 2, 3 and 4, the command going to servos/motors on the output channels 6, 7, 8, and 9, and finally the analog single connected to pin 59 (A5). Note that if you include arduino_pins you must also include arduino_names and they must be the same length. The names are the identifier strings that will be logged with each dfti data point.

The file Logging_checklist.pdf has more information.
