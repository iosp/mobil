Mobil shared coordinates package

Prerequisites:

1) libv4l: sudo apt-get install libv4l-dev
2) ar_pose installed on UAV.(http://wiki.ros.org/ar_pose)
3) Multimaster package installed on both UAV and UGV (http://wiki.ros.org/multimaster_fkie)
4) This package installed on both UAV and UGV.
5) yaml file on UGV with the markers data, including UGV markers transform 
   to base of UGV (see marker_example.yaml).
6) ar_pose launch file on UAV.
7) hector_quadrotor package for the quadrotor:

    sudo apt-get install ros-kinetic-gazebo-ros*
    sudo apt-get install ros-kinetic-hector*

    In catkin_ws/src: 
    git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git
    
    And then run catkin_make in the catkin_ws directory.
8) Adding the gazebo models (in the /gazebo_models directory) to gazebo. There are two ways:
    1. Moving the folders of the models to ~/.gazebo/models
    2. Adding the /gazebo_models directory to the GAZEBO_MODEL_PATH environment variable
9) teleop_twist_keyboard package for teleoperating the quadrotor:
    sudo apt-get install ros-kinetic-teleop-twist-keyboard
    

This package contains three launch files:

1. UGV.launch - Runs all the code needed to run on the UGV
2. UAV.launch - Runs all the code needed to run on the UAV
3. mobil_sim.launch - Simulation environment for this package.

Running on UAV (launch file: UAV.launch):

1. Creates static tfs between the cameras and the UAV base

2. Runs marker_data_client ther recives UGV markers and free 
   marker data and runs ar_pose with the marker data.


Running on UGV (launch file: UGV.launch):

1) Runs marker_data_service that sends markers data and publish the static transform 
   from the UGV markers to UGV base.

2) Runs mobil_shared coordinates - the program that queries the location of the  
                                    goal marker relative to the UGV and print it.


Running the mobil_shared_coordinate package:

Run UAV.launch on the UAV and UGV.launch on the UGV. Once both are up, the UAV part launches the ar_pose module which gives the transform of markers relative to the UAV's cameras. Using the obtained transforms from the ar_pose module and shared knowledge of the static transforms of the UAV and UGV, the UGV calculates the transforms of nearby objects relative to its base. It is possible to run the UAV and UGV parts either on one ROS core or on multiple ROS cores (using the Multimaster package).


Simulation (launch file: mobil_sim_gazebo.launch):

Run mobil_sim.launch. This runs the UAV and UGV parts and in addition it runs Gazebo which simulates the UAV's cameras and an environment with markers. Using this environment Gazebo publishes image data to the right topics, replacing the real-time images obtained from the UAV. In addition, the simulation launch file runs rviz which visualizes the output of the functional part. The visualization shows the outputs of the UAV cameras and the complete transform tree composed by the UAV and the UGV.

Controlling the quadrotor:

mobil_sim_gazebo.launch automatically enables the quadrotor's motors by calling the /enable_motors service with "true" and runs a node of type "teleop_twist_keyboard" for teleoperating the quadrotor. The controls are:

Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

***********************************************************************************
Michele's Notes:
**************************************************************************************
Environment: Ubuntu 16.04 ROS Kinetic Gazebo 8.6.0

In order to make it work:
1. copy $mobil/gazebo_models/free_marker_cube and $mobil/gazebo_models/marker_cube and $mobil/gazebo_models/ar_tags/model/marker0 to ~/.gazebo/models
2. run the script: cd $mobil/gazebo_models/ar_tags/scripts; ./generate_markers_model.py -i ~/ws/src/mobil/gazebo_models/ar_tags/images/ -g /home/robil/.gazebo/models/
3. clone the https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git and build it before dealing with https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git
The both repositories are in branch kinetic-devel
4. several ros packages had to be installed: for example ros-kinetic-gazebo8-ros-control

************************************************************************
Actual STATUS:
***********************************************************************
Launching roslaunch mobil_shared_coordinates mobil_sim_gazebo.launch actually open gazebo with hector and markers and RVIZ.
When on the terminal that launched the simulation, you can play with Hector using the keys as mentionned above.
There are still major errors: 
- from the console:
[ WARN] [1537193299.285611515]: TF between marker center_marker and base is unavailable
[ WARN] [1537193299.285655638]: TF between marker left_marker and base is unavailable
[ WARN] [1537193299.285674664]: TF between marker right_marker and base is unavailable
[ WARN] [1537193299.285701456]: TF between marker front_marker and base is unavailable
                                                                                                                                                                                 [ WARN] [1537193299.285726719]: TF between marker rear_marker and base is unavailable

[ WARN] [1537193301.387369067, 566.715000000]: Desired controller update period (0.010000000 s) is slower than the gazebo simulation period (0.001000000 s)

[ERROR] [1537193304.023512481, 569.134000000]: TF between the free marker and camera is unavailable

Warning: TF_OLD_DATA ignoring data from the past for frame ${name_body} at time 0 according to authority unknown_publisher 
                                                                                                                                                                            Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained at line 277 in /tmp/binarydeb/ros-kinetic-tf2-0.5.18/src/buffer_core.cpp

==> impossible to understand the data displayed on the console because of the error and warning messages.

- from gazebo log (~/.gazebo/server-11345/default.log):
(1537194080 436669066) [hector_quadrotor_controller_gazebo] Using ground truth from Gazebo as state input for control
(1537194080 437090980) [hector_quadrotor_controller_gazebo] Using ground truth from Gazebo as imu input for control




