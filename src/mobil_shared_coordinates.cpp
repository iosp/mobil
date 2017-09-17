/*
 * shared_coordinates.cpp
 *
 *  Created on: May 27, 2015
 *      Author: mobil
 */

#include <ros/ros.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <fstream>

using namespace std;

string camera_fn, robot_fn;
string center_marker_fn, left_marker_fn, right_marker_fn, front_marker_fn, rear_marker_fn;
string free_marker_fn;
string base_link_fn; // format: [marker]_base_link
ofstream csv_file;
tf::Vector3 UAV_location;
tf::Vector3 freeMarkerLocation;


bool useMarkerToFindLocation(const tf::TransformListener &transListener, const string &marker_fn, const string &robot_fn, const string &camera_fn)
{
	tf::StampedTransform markerTransform, baseLinkTransform;
	float pos_x, pos_y, pos_z;

	try
	{
		if (transListener.waitForTransform(marker_fn, camera_fn, ros::Time(0), ros::Duration(1.0)))
		{
			transListener.lookupTransform(marker_fn, camera_fn, ros::Time(0), markerTransform);
			pos_x = markerTransform.getOrigin().x();
			pos_y = markerTransform.getOrigin().y();
			pos_z = markerTransform.getOrigin().z();

			ROS_INFO("Using marker %s, the camera relative to marker is at: (%.3f, %.3f, %.3f)", marker_fn.c_str(), pos_x, pos_y, pos_z);

			//string base_link_fn = marker_fn + "_base_link";
			base_link_fn = marker_fn + "_" + robot_fn;

			transListener.lookupTransform(base_link_fn, camera_fn, ros::Time(0), baseLinkTransform);
			UAV_location.setX(baseLinkTransform.getOrigin().x());
			UAV_location.setY(baseLinkTransform.getOrigin().y());
			UAV_location.setZ(baseLinkTransform.getOrigin().z());

			//ROS_INFO("Using marker %s, the camera relative to base link is at: (%.3f, %.3f, %.3f)", marker_fn.c_str(), pos_x, pos_y, pos_z);
			ROS_INFO("UAV location: (%.3f, %.3f, %.3f), using marker %s", UAV_location.getX(), UAV_location.getY(), UAV_location.getZ(), marker_fn.c_str());

			if (csv_file) {
				csv_file << marker_fn << "," << UAV_location.getX() << "," << UAV_location.getY() << "," << UAV_location.getZ() << ",";
			}
			return true;
		}
		else
		{
			ROS_WARN("TF between marker %s and base is unavailable", marker_fn.c_str());
		}
	}
	catch (const tf::TransformException & ex)
	{
		ROS_ERROR("shared_coordinates: %s", ex.what());
	}
	return false;
}


void getFreeMarkerLocation()
{
	tf::TransformListener transListener;
	tf::StampedTransform freeMarkerTransform;
	static tf::TransformBroadcaster broadcaster;

	bool transformFound = false;

	try
	{
		if (transListener.waitForTransform(base_link_fn, free_marker_fn, ros::Time(0), ros::Duration(2.0)))
		{
			transformFound = true;
			transListener.lookupTransform(base_link_fn, free_marker_fn, ros::Time(0), freeMarkerTransform);
			freeMarkerLocation.setX(freeMarkerTransform.getOrigin().x());
			freeMarkerLocation.setY(freeMarkerTransform.getOrigin().y());
			freeMarkerLocation.setZ(freeMarkerTransform.getOrigin().z());
			//cout<<"##################################################################################################"<<endl;
			//ROS_INFO("Free marker location: (%.3f, %.3f, %.3f)", freeMarkerLocation.getX(), freeMarkerLocation.getY(), freeMarkerLocation.getZ());
			//cout<<"##################################################################################################"<<endl;

			// broadcast transform from UGV base_link to free_marker
			freeMarkerTransform.child_frame_id_ = "free_marker_from_" + base_link_fn;
			freeMarkerTransform.frame_id_ = robot_fn;
			broadcaster.sendTransform(freeMarkerTransform);

			if (csv_file) {
				csv_file << freeMarkerLocation.getX() << "," << freeMarkerLocation.getY() << "," << freeMarkerLocation.getZ();
			}
		}
		else
		{
			ROS_ERROR("TF between the free marker and camera is unavailable");
		}
	}   
	catch (const tf::TransformException & ex)
	{
		ROS_ERROR("TF between the free marker and camera is unavailable");
		ROS_ERROR("shared_coordinates: %s", ex.what());
	}
	if (csv_file) {
		if (!transformFound)
			csv_file << ",,";
		csv_file << endl;
	}
}

bool getLocations(const tf::TransformListener &transListener)
{
	if (useMarkerToFindLocation(transListener, center_marker_fn, robot_fn, camera_fn))
		getFreeMarkerLocation();

	if (useMarkerToFindLocation(transListener, left_marker_fn, robot_fn, camera_fn))
		getFreeMarkerLocation();

	if (useMarkerToFindLocation(transListener, right_marker_fn, robot_fn, camera_fn))
		getFreeMarkerLocation();

	if (useMarkerToFindLocation(transListener, front_marker_fn, robot_fn, camera_fn))
		getFreeMarkerLocation();

	if (useMarkerToFindLocation(transListener, rear_marker_fn, robot_fn, camera_fn))
		getFreeMarkerLocation();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "shared_coordinates");

	if (argc > 1) {
		// Create csv file name from bag file name
		string bag_file = argv[1];
		int last_index = bag_file.find_last_of(".");
		string raw_file_name = bag_file.substr(0, last_index);
		string csv_file_name = raw_file_name + ".csv";
		csv_file.open(csv_file_name.c_str());
	}

	ros::NodeHandle nh("~");

	//nh.getParam("world_fn", world_fn);
	nh.getParam("camera_fn", camera_fn);
	nh.getParam("robot_fn", robot_fn);
	nh.getParam("center_marker_fn", center_marker_fn);
	nh.getParam("left_marker_fn", left_marker_fn);
	nh.getParam("right_marker_fn", right_marker_fn);
	nh.getParam("front_marker_fn", front_marker_fn);
	nh.getParam("rear_marker_fn", rear_marker_fn);
	nh.getParam("free_marker_fn", free_marker_fn);

	ROS_INFO("camera_fn: %s", camera_fn.c_str());
	ROS_INFO("center_marker_fn: %s", center_marker_fn.c_str());
	ROS_INFO("left_marker_fn: %s", left_marker_fn.c_str());
	ROS_INFO("right_marker_fn: %s", right_marker_fn.c_str());
	ROS_INFO("front_marker_fn: %s", front_marker_fn.c_str());
	ROS_INFO("rear_marker_fn: %s", rear_marker_fn.c_str());
	ROS_INFO("free_marker_fn: %s", free_marker_fn.c_str());

	ros::Rate rate(10.0);

	ROS_INFO("Starting shared_coordinates...");

	tf::TransformListener transListener;
	sleep(1.0); // RY: Need to wait between creating TF listener and waitForTransform, see bug http://answers.ros.org/question/43111/tf-wait-for-transform-failure/

	while (nh.ok())
	{
		try
		{
			getLocations(transListener);
		}
		catch (const tf::TransformException & ex)
		{
			ROS_ERROR("shared_coordinates: %s", ex.what());
		}
		ros::spinOnce();
		rate.sleep();
	}

	csv_file.close();
	return 0;
}




