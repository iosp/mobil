/*
 * marker_data_service.cpp
 *
 *  Created on: Aug 24, 2017
 *      Author: mikab4
 */


#include "ros/ros.h"
#include "mobil_shared_coordinates/object_data.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "mobil_shared_coordinates/Markers_file_parser.h"
#include <boost/lexical_cast.hpp>
using namespace std;

std::vector<Marker*> objects_markers;


bool send_marker_data(mobil_shared_coordinates::object_data::Request  &req,
		mobil_shared_coordinates::object_data::Response &res)
{
	res.file_contant = boost::lexical_cast<std::string>(objects_markers.size());
	res.file_contant += "\n";
	for(std::vector<Marker*>::iterator it = objects_markers.begin(); it != objects_markers.end(); ++it){
		res.file_contant += (*it)->toObjectFileString();
	}
	return true;
}


void publishTransform(tf::Transform transform, string frame_name,
		string parent_fn){
	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
	geometry_msgs::TransformStamped transformStamped;
	ROS_INFO("BRODCASTING TRANSFORM FROM %s to %s", parent_fn.c_str(), frame_name.c_str());
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = frame_name;
	transformStamped.child_frame_id = parent_fn;
	tf::transformTFToMsg(transform,transformStamped.transform);
	static_broadcaster.sendTransform(transformStamped);
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "marker_data_service");
	ros::NodeHandle nh("~");
	ros::NodeHandle nh2;
	string yaml_file_path;
	if(nh.hasParam("yaml_file_paths")){
		nh.getParam("yaml_file_paths", yaml_file_path);
	}
	else{
		cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
		cout<< "Missing parameter: yaml_file_paths"<<endl;
		cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
		return 0;
	}
	string robot_name;
	if(nh.hasParam("robot_name")){
		nh.getParam("robot_name", robot_name);
	}
	else{
		cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
		cout<< "Missing parameter: robot_name"<<endl;
		cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
		return 0;
	}

	MarkersFileParser parser(yaml_file_path);
	objects_markers = parser.getMarkersList();
	int num_of_makrers = objects_markers.size();
	ros::ServiceServer service = nh2.advertiseService("object_data_"+robot_name, send_marker_data);
	ROS_INFO("Ready to send markers data.");
	ros::Rate loop_rate(10);
	for (int i = 0; i < num_of_makrers; i++){
		if (!objects_markers[i]->isGoal()){
			publishTransform(objects_markers[i]->getStaticTransform(),objects_markers[i]->getName(),
					objects_markers[i]->getParentFram());
		}
	}

	ros::spin();
	return 0;
}
