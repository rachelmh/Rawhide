# rawhide


## First Setup Only
The following is adapted from http://sdk.rethinkrobotics.com/intera/Workstation_Setup and http://wiki.ros.org/melodic/Installation/Ubuntu for Ubuntu 18.04 and ROS Melodic.

### Install ROS
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### Clone Development Workspace
If you change the directory, make sure you adjust all following commands.
```
$ cd
$ git clone https://github.com/rachelmh/rawhide.git
$ cd rawhide/rawwhide_ws
```
Delete the build and devel folders
```
$cd ..
```

### Install Intera
```
$ sudo apt-get update
$ sudo apt-get install git-core python-argparse python-vcstools python-rosdep ros-melodic-control-msgs ros-melodic-joystick-drivers ros-melodic-xacro ros-melodic-tf2-ros ros-melodic-rviz ros-melodic-cv-bridge ros-melodic-actionlib ros-melodic-actionlib-msgs ros-melodic-dynamic-reconfigure ros-melodic-trajectory-msgs ros-melodic-rospy-message-converter
$ cd ~/rawhide/rawhide_ws/src
$ wstool init .
$ git clone https://github.com/RethinkRobotics/sawyer_robot.git
$ wstool merge sawyer_robot/sawyer_robot.rosinstall
$ wstool update
$ source /opt/ros/melodic/setup.bash
$ cd ~/rawhide/rawhide_ws
$ catkin_make
```

### Set Up Networking
Connect to the robot via the ethernet port on the outside of the Controller.
![Source: http://sdk.rethinkrobotics.com/intera/Workstation_Setup](http://sdk.rethinkrobotics.com/intera/a/images/e/ec/Ethernet_Port.png)

Open Ubuntu's "Settings" tool, and navigate to the "Network" tab. Then, click the gear icon in the "Wired" section.
![Click the gear icon in Network Settings](/images/network_settings.png?raw=true)

Navigate to the IPv4 tab and select "Link-Local Only." Then, click "Apply."
![Enable Link-Local Only](/images/wired_settings.png?raw=true)

You will not be able to use that ethernet port to access the internet until you revert the Method to "Automatic (DHCP)." You may now close the Settings window.

### Set Up Intera
Copy the `intera.sh` file into your ros workspace
```
$ cp ~/rawhide/rawhide_ws/src/intera_sdk/intera.sh ~/rawhide/rawhide_ws
```

Now you'll need to edit three lines of `intera.sh` specific to your system.
```
$ gedit intera.sh
```

#### Edit `robot_hostname`
Edit the `robot_hostname` field on line 22 to be the name of your robot. For example:
```
robot_hostname="tom.local"
```
where `tom` is the name of your robot. This needs to match the name you gave it when you set it up.

#### Replace `your_ip` with `your_hostname`
Comment out the `your_ip` field on line 26 and edit the `your_hostname` field on line 27 as follows:
```
#your_ip="192.168.XXX.XXX"
your_hostname="$(uname -n).local"
```

#### Edit `ros_version`
Edit the `ros_version` field on line 30 to be the version of ROS you are running. Assuming you followed this README, that is:
```
ros_version="melodic"
```

Save and close `intera.sh` script.

### Verify Environment
```
$ cd ~/rawhide/rawhide_ws
$ ./intera.sh
```

You should now be in a new shell titled `intera - http://tom.local:11311` where `tom` is the name of your robot. A useful command for viewing and validating your ROS environment setup is:
```
$ env | grep ROS
```

The important fields at this point:
`ROS_MASTER_URI` - This should now contain your robot's hostname.
`ROS_HOSTNAME` - This should contain your PC's hostname.
Try to get rostopic list from robot by typing following command:
```
$ rostopic list
```

You should see a the rostopic list from the command line output similar as following:
```
/array_topic
/audio_duration
/calibration_command/cancel
/calibration_command/feedback
/calibration_command/goal
/calibration_command/result
/calibration_command/status
/camera_field_calibration/cancel
/camera_field_calibration/feedback
/camera_field_calibration/goal
/camera_field_calibration/result
/camera_field_calibration/status
/cmd2browser
/cmd2shell
/collision/right/collision_detection
/collision/right/debug
/dev_topic
/diagnostics
/engine/task_state
/errors
/head_navigator_button
/intera/endpoint_ids
/io/comms/command
/io/comms/config
/io/comms/io/command
/io/comms/io/config
/io/comms/io/state
/io/comms/state
/io/end_effector/command
/io/end_effector/config
/io/end_effector/state
/io/internal_camera/command
...
```
### Set up git repository
```
$ git config --global user.name rachelmh
$ git config --global user.email rachelmh@mit.edu
```


### Install PyKDL and Robotiq
```
cd ~/rawhide/rawhide_ws/src
git clone https://github.com/rupumped/sawyer_pykdl.git
cd ..
catkin_make
cd src
git clone https://github.com/ros-industrial/robotiq.git
 source ~/rawhide/rawhide_ws/devel/setup.bash
rosdep install robotiq_modbus_tcp
sudo apt-get install ros-melodic-soem
cd ..
rosdep install --from-paths src --ignore-src -r -y
sudo usermod -a -G dialout $USER
catkin_make


```
### Make sure Robotiq Grippers Connect
To make sure the grippers are up and running, you can simply do this:
First check to see which ports the grippers are connected to by using
```
dmesg | grep ttyUSB
```
For me, when I have nothing but one gripper plugged in I see:
```
[  307.969155] usb 1-4: FTDI USB Serial Device converter now attached to ttyUSB0
[  307.970078] usb 1-4: FTDI USB Serial Device converter now attached to ttyUSB1
```
Make sure you use the higher number usb port when commanding the gripper.

When I have both grippers and the arduino plugged in via USB I see:
```
 usb 1-4: FTDI USB Serial Device converter now attached to ttyUSB0
 usb 1-4: FTDI USB Serial Device converter now attached to ttyUSB1
usb 1-3: FTDI USB Serial Device converter now attached to ttyUSB2
usb 1-3: FTDI USB Serial Device converter now attached to ttyUSB3
usb 1-2: FTDI USB Serial Device converter now attached to ttyUSB4
```
meaning that gripper1 should be commanded via ttyUSB1 and gripper2 should be commanded via ttyUSB3 and the arduino should be commanded via ttyUSB4. This should be remembered for later!!

To test the connection:

open a new terminal
```
cd ~/rawhide/rawhide_ws
./intera_poirot.sh
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB1
```
The correct output should be
```
 pub = rospy.Publisher('Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input)
 ```
 
 You can also check the ros topics
 ```
 rostopic list
 ```
 In addition to seeing all of the sawyer topics (if you are running the grippers via ROS on the sawyer), you should be able to see
 ```
 /Robotiq2FGripperRobotInput
/Robotiq2FGripperRobotOutput
```

If you want to play around with the grippers, I recommend following this tutorial
http://wiki.ros.org/robotiq/Tutorials/Control%20of%20a%202-Finger%20Gripper%20using%20the%20Modbus%20RTU%20protocol%20%28ros%20kinetic%20and%20newer%20releases%29

IMPORTANT: When you are following the above tutorial, make sure you activate-reset-activate. This will allow the gripper to actually work.




### Install Arduino
Go to https://www.arduino.cc/en/Main/Software and download the file linux 64 bits
extract the files in the /home/rachel directory

```
cd ~/arduino-1.8.9-linux64/arduino--1.8.9    
sudo ./install.sh
sudo usermod -a -G dialout $USER
```
### Install Ros Serial for Arduino
go to http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
```
cd ~/rawhide/rawhide_ws/src
git clone https://github.com/ros-drivers/rosserial.git
cd ..
catkin_make
#  cd <sketchbook>/libraries <sketchbook> is the directory where the Linux Arduino environment saves your sketches. Typically in #home directory
cd ~/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
 

```
The settings for the Arduino I use are 
Board: Arduino Pro or Pro Mini
Processor: ATMega328P 5V 16MHz

And the code running should be located in
/home/rachel/rawhide/rmh_code/multi_button_rmh/multi_button_rmh.ino

This code basically publishes topics for each of the 4 buttons (2 per sawyer)

### Make sure Arduino ROS Nodes work
check to see which port the arduino is plugged in then open a terminal running ros (either start a new terminal with roscore or ./intera.sh into one of the sawyers). Change ttyUSB0 to whatever port is connected to the arduino
 ```
 rosrun rosserial_python serial_node.py /dev/ttyUSB0
 ```
 If running properly, the output should be:
 ```
 [INFO] [1555376406.861082]: ROS Serial Python Node
[INFO] [1555376406.883253]: Connecting to /dev/ttyUSB2 at 57600 baud
[INFO] [1555376408.993138]: Requesting topics...
[INFO] [1555376409.068245]: Note: publish buffer size is 280 bytes
[INFO] [1555376409.074238]: Setup publisher on pushed1 [std_msgs/Bool]
[INFO] [1555376409.085425]: Setup publisher on pushed2 [std_msgs/Bool]
[INFO] [1555376409.097274]: Setup publisher on pushed3 [std_msgs/Bool]
[INFO] [1555376409.108392]: Setup publisher on pushed4 [std_msgs/Bool]
```

### Install Robot Raconteur

Install dependencies
```
apt-get install default-jdk default-jdk-headless default-jre default-jre-headless python2.7-minimal libpython2.7 python2.7-dev libpython2.7-dev libssl1.0.0 zlib1g zlib1g-dev libssl-dev libusb-1.0-0 libusb-1.0-0-dev libdbus-1-3 libdbus-1-dev libbluetooth3 libbluetooth-dev zlib1g zlib1g-dev python-numpy python-setuptools python-wheel git cmake-qt-gui g++ make libboost-all-dev autoconf automake libtool bison libpcre3-dev
```

First go to http://www.robotraconteur.com/download/. You will have to create a username and password.
Once inside, scroll down to find RobotRaconteur-0.8.1-beta-Python.linux-x86_64-py2.7-2016-07-18.tar.gz  (or RobotRaconteur-0.8.1-beta-Python.linux-i686-py2.7-2016-07-18.tar.gz depending on your computer)
Download the file.

Then
```
cd /
sudo tar xvzf /home/rachel/Downloads/RobotRaconteur-0.8.1-beta-Python.linux-x86_64-py2.7-2016-07-18.tar.gz
```

Robot raconteur also requires pyserial and numpy.

```
sudo apt-get install python-serial python-numpy python-opencv python-pygame
```

## Every time you run the main program

### Getting ready to launch everything

I have made several launch files to help starting up the code simpler, but first we need to make sure all of the ports are set.


```
cd ~/rawhide/rawhide_ws
ls
```
You should see 
fullsystem_rmh.sh
poirot_launch.launch
captain_launch.launch

First, open fullsystem_rmh.sh with your favorite editor
```
gedit fullsystem_rmh.sh
```
Inside, you should see this line
```
gnome-terminal --tab --title="poirot1" --command="bash -c 'cd ~/rawhide/rawhide_ws; source intera_poirot.sh;'"
```

change the source to match the .sh file for your robots. For example, if you create a .sh file for your robot local host name 'robot1' you should have
```
gnome-terminal --tab --title="poirot1" --command="bash -c 'cd ~/rawhide/rawhide_ws; source robot1.sh;'"
```
Update the second line similarly for your second robot.

Next, using your favorite editor, open poirot_launch.launch

Make sure the rosserial arduino port matches what it actually is when plugged in.  Same for the gripper ttyUSB
```
<launch>
  <!--Run rosserial for the conveyor arduino-->
	 <node name='arduino' pkg='rosserial_python' type='serial_node.py' args='/dev/ttyUSB4' respawn="true"/>
 
    <node name='gripper_p' pkg='robotiq_2f_gripper_control' type='Robotiq2FGripperRtuNode.py' args='/dev/ttyUSB3' respawn="true"/>
  ```  
    
Once those are saved, we are ready to launch everything! You should make sure the launch files match with the ports every time you plugin/unplug the usbs for the grippers.

### LAUNCHING!

1. Open a new terminal

```
source ~/rawhide/rawhide_ws/devel/setup.bash
cd ~/rawhide/rawhide_ws
 ./fullsystem_rmh.sh 
 ```
 Two additional tabs should have opened, one .sh'd into Poirot, one .sh'd into Captain
 In the poirot terminal
 
```
source ~/rawhide/rawhide_ws/devel/setup.bash
roslaunch  poirot_launch.launch
```
 In the captain terminal
```
source ~/rawhide/rawhide_ws/devel/setup.bash
roslaunch  captain_launch.launch
```

In the original Terminal to run the code

```
 source ~/rawhide/rawhide_ws/devel/setup.bash
 cd ~rawhide/rmh_code
 python TBD_4_4.py
 
 ```






http://www.robotraconteur.com/download/

https://s3.amazonaws.com/robotraconteurpublicfiles/docs/IntroductionToRobotRaconteur-2017-03-18.pdf

https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html

