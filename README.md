Mobil shared coordinates package

Prerequisites:

1) ar_pose installed on UAV.(http://wiki.ros.org/ar_pose)
2) Multimaster package installed on both UAV and UGV (http://wiki.ros.org/multimaster_fkie)
2) This package installed on both UAV and UGV.
3) yaml file on UGV with the markers data, including UGV markers transform 
   to base of UGV (see marker_example.ymal).
4) ar_pose launch file on UAV.

This package contains three launch files:

1. Runs all the code needed to run on the UGV
2. Runs all the code needed to run on the UAV
3. Runs a simulation which shows the capabilities of this package.

Running on UAV (launch file: UAV.launch):

1. Create static tfs between the cameras and the UAV base

2. Runs marker_data_client ther recives UGV markers and free 
   marker data and runs ar_pose with the marker data.


Running on UGV (launch file: UGV.launch):

1) Runs marker_data_service that sends markers data and publish the static transform 
   from the UGV marker to UGV base.

2) Runs mobil_shared coordinates - the program that query the location of the  
                                    goal marker relative to the UGV and print it.
