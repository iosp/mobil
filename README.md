Mobil shared coordinates package

Prerequisites:

1) ar_pose installed on UAV.(http://wiki.ros.org/ar_pose)
2) Multimaster package installed on both UAV and UGV (http://wiki.ros.org/multimaster_fkie)
2) This package installed on both UAV and UGV.
3) yaml file on UGV with the markers data, including UGV markers transform 
   to base of UGV (see marker_example.yaml).
4) ar_pose launch file on UAV.

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


Simulation (launch file: mobil_sim.launch):

Run mobil_sim.launch. This runs the UAV and UGV parts and in addition it runs Gazebo which simulates the UAV's cameras and an environment with markers. Using this environment Gazebo publishes image data to the right topics, replacing the real-time images obtained from the UAV. In addition, the simulation launch file runs rviz which visualizes the output of the functional part. The visualization shows the outputs of the UAV cameras and the complete transform tree composed by the UAV and the UGV.
