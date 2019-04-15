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
$ cd rawhide
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


### Install Other Stuff
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

### Install Arduino
Go to https://www.arduino.cc/en/Main/Software and download the file linux 64 bits
extract the files in the /opt directory
```
cd /opt
sudo ./install.sh
```

### Install Miscellaneous Packages
```
$ sudo apt install python-pip
$ pip install gTTS --upgrade
$ pip install gTTS-token --upgrade --user
$ pip install pexpect
$ pip install playsound
```

## To Launch
Navigate to the `teachbot/robot` directory. 

Initialize SDK environment
```
$ ./intera.sh
```

Launch the websocket and start Node.js
```
$ roslaunch ../browser/websocket.launch & node ../browser/www.js &
```

Run the TeachBot ROS Node
```
$ rosrun teachbot module1.py
```
replacing `module1.py` with the desired module.

## Load Modules in Firefox
Open Firefox and go to the url https://localhost:8000

### If Firefox alerts you that your connection is not secure
Click "Advanced"

![Click "Advanced"](/images/insecure_connection.png?raw=true)

Click "Add Exception"

![Click "Add Exception"](/images/add_exception.png?raw=true)

Click "Confirm Security Exception"

![Click "Confirm Security Exception"](/images/confirm_exception.png?raw=true)

At this point, the module should begin, but there should be a pop-up window that reads "Error connecting to websocket server" like this:

![Navigate to localhost:9090 and repeat the security exception process.](/images/connection_error.png?raw=true)

Then, go to the url https://localhost:9090 and repeat the security exception process. After, https://localhost:9090 should simply read `Can "Upgrade" only to "WebSocket".` Navigate back to https://localhost:8000.
### EndIf

You have successfully launched the teaching module!

## Repository Overview
```
.
|
+-- browser                             Files related to the HTML/CSS/JS running in-browser.
|   +-- public                          Front-End Browser Content.
|       +-- audio                       TeachBot speech audio files.
|           +-- module#/                Directories containing audio files of TeachBot speech indexed by module.
|           +-- make_speech.py          Script to generate speech audio files from text scripts.
|       +-- css/                        Style sheets for the browser HTML
|       +-- html/                       Web pages to display in browser.
|       +-- images/                     Images displayed in browser by TeachBot.
|       +-- js                          JavaScript module control scripts and utilities.
|           +-- utils/                  Utility functions for browser.
|           +-- #.js                    Module control scripts for browser indexed by module.
|       +-- text/                       What TeachBot says.
|       +-- videos/                     Videos played in browser by TeachBot.
|   +-- routes/                         Node.js routes used by app.js.
|   +-- views/                          App templates.
|   +-- app.js                          Main configuration file for TeachBot browser.
|   +-- CMakeLists.txt                  Node CMakeLists.
|   +-- package-lock.json               NPM install utility.
|   +-- package.json                    NPM install utility.
|   +-- package.xml                     Node package manifest.
|   +-- websocket.launch                ROS launch file for web module.
|   +-- www.js                          Starts Node.js server.
|
+-- images/                             Images for the README
|
+-- robot                               Python ROS library responsible for communicating with the robot.
|   +-- src                             Source code.
|       +-- arduino_files/              Scripts for running peripheral devices on Arduino.
|       +-- teachbot                    TeachBot ROS package.
|           +-- Learner_Responses/      Data collected from user subject tests.
|           +-- src/                    Python source code.
|               +-- module#.py          Module control scripts for robot indexed by module.
|               +-- <other>.py          Utility classes and functions used by module control scripts.
|           +-- CMakeLists.txt          TeachBot package CMakeLists.
|           +-- package.xml             TeachBot package manifest.
|           +-- setup.py                TeachBot package setup script.
|   +-- sslcert/                        SSL certificate files required for setting up HTTPS connection.
|   +-- logo.png                        TeachBot logo to be displayed on Sawyer head display.
|   +-- module#.launch                  Launch files for each TeachBot module.
|   +-- safety1.mp3                     Audio file to be played when TeachBot limb exits safety zone.
|   +-- safety2.mp3                     Audio file to be played when TeachBot limb resets to within the safety zone.
```
The project is organized in two segments:
1) JavaScript Node application, responsible for displaying content in the browser, and
2) Python ROS library, responsible for communicating with the robot.

### JavaScript Node Application
The JavaScript scripts controlling the browser are located [here](/browser/public/js) and are indexed by module. The crux of these files is the `init()` function that initiates ROS communication between the browser and the Python shell and subscribes to the `cmd2browser` topic to receive commands from the shell. These messages are of type `Int32` and consist of the index of the command to run. The subscriber function is essentially a massive switch statement commanding the browser to display graphics and play audio depending on the index received on the topic. After performing the prescribed sequence, the script increments the index and passes it to the Python shell by publishing it on  the `cmd2shell` topic.

### Python ROS Library
All Python scripts are located in [the teachbot source directory](/robot/src/teachbot/src).

The main files have names beginning in 'module' (e.g. `module1.py`). These files are run using the command described above:
```
$ rosrun teachbot module1.py
```
The scripts are structured according to `module_template.py`. Their main function is as a ROS listener awaiting instructions from the browser and monitoring the robot ROSTopics. Each file contains the function `rx_command(self, data)`, which receives instructions from the browser in the form of ROS messages on the `cmd2shell` topic. Complementing the JavaScript `init()` subscriber, `rx_command()` also acts as essentially a massive switch statement. Whenever an index is received from the browser on the `cmd2shell` topic, `rx_command` performs a sequence of tasks corresponding to the index received, then publishes the index to the `cmd2browser` topic to inform the browser the sequence is complete.

## Editing the Script
The TeachBot script for each module can be found in [the text directory](/browser/public/text). Each line is synthesized into its own audio file and played by a single call to the `play()` method in JavaScript. There can be no empty lines in the script file.

After editing the script, navigate to [the audio directory](/browser/public/audio) and run the following command, replacing both instances of `1` with the module number for which you wish to generate audio:
```
$ python3 make_speech.py ../text/1.txt module1/
```
The `make_speech.py` script uses Google Text-to-Speech to synthesize audio files from each line of the input txt file and saves them in the given directory. The corresponding JavaScript module script downloads and plays these files.

Additionally, you may wish to update the JavaScript file with the new text. To do so, navigate to [the js directory](/browser/public/js) and run the following command, replacing `1.js` with the name of the JS file you wish to update:
```
$ python3 js_update.py 1.js
```
The script will generate a new .js file, `updated_1.js`. Review this file and copy its contents into `1.js`.




http://www.robotraconteur.com/download/

https://s3.amazonaws.com/robotraconteurpublicfiles/docs/IntroductionToRobotRaconteur-2017-03-18.pdf

https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html

