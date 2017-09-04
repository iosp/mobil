/*
 * Ymalparser.cpp
 *
 *  Created on: Aug 24, 2017
 *      Author: mikab4
 */

#include "mobil_shared_coordinates/Markers_file_parser.h"

#include <yaml-cpp/yaml.h>
#include <sstream>
#include <string>

using namespace std;

MarkersFileParser::MarkersFileParser(string file_path) {
	num_of_obj_marker = 0;

	parse(file_path);
}

void MarkersFileParser::parse(string file_path){
	string name;
	string parent_name;
	string path;
	double width;
	double centerX;
	double centerY;
	double transX;
	double transY;
	double transZ;
	double roll;
	double yaw;
	double pitch;
	YAML::Node markers_file = YAML::LoadFile(file_path);
	if (markers_file["obj_name"]) {
		object_name = markers_file["obj_name"].as<std::string>();
	}
	cout<<"objact_name: "<<object_name<<endl;
	cout<<"-------------------------------------------------------------------------------"<<endl;
	//	int i = 0;
	for (YAML::const_iterator it=markers_file.begin();it!=markers_file.end();++it) {
		if (it->first.as<std::string>() == "marker"){
			//			cout<<i<<endl;
			//			i++;
			num_of_obj_marker++;
			name = it->second["name"].as<string>();
			parent_name = it->second["parent_fram"].as<string>();
			path = it->second["path"].as<string>();
			width = it->second["width"].as<double>();
			centerX = it->second["center"]["x"].as<double>();
			centerY = it->second["center"]["y"].as<double>();
			transX = it->second["transform"]["x"].as<double>();
			transY = it->second["transform"]["y"].as<double>();
			transZ = it->second["transform"]["z"].as<double>();
			roll = it->second["transform"]["roll"].as<double>();
			pitch = it->second["transform"]["pitch"].as<double>();
			yaw = it->second["transform"]["yaw"].as<double>();
			markers_list.push_back(new Marker(name, path, parent_name, centerX,
					centerY, width, transX, transY, transZ, roll, yaw, pitch));


		}
		if(it->first.as<std::string>() == "goal_marker"){
			name = it->second["name"].as<string>();
			path = it->second["path"].as<string>();
			width = it->second["width"].as<double>();
			centerX = it->second["center"]["x"].as<double>();
			centerY = it->second["center"]["y"].as<double>();
			markers_list.push_back(new Marker(name, path, centerX,
					centerY, width));
		}
	}
	//	cout<<i<<endl;
//		cout<<"------"<<markers_list.size()<<endl;
//		for(std::vector<Marker*>::iterator it = markers_list.begin() ; it != markers_list.end(); ++it){
//			cout<<(*it)->toObjectFileString()<<endl;
//		}
}

MarkersFileParser::~MarkersFileParser() {
	// TODO Auto-generated destructor stub
}


Marker::Marker(std::string name, std::string path, std::string parent_fram,
		double centerX, double centerY, double width,
		double transX, double transY, double transZ,
		double roll, double yaw, double pitch, bool is_goal){

	this->name = name;
	this->parent_fram = parent_fram;
	this->patt_file_path = path;
	this->center.first = centerX;
	this->center.second = centerY;
	this->width = width;
	this->is_goal = is_goal;
	this->static_transform = tf::Transform(tf::createQuaternionFromRPY(roll,pitch,yaw),
			tf::Vector3(transX, transY, transZ));



}

Marker::Marker(const Marker& other) {
	this->name = other.name;
	this->parent_fram = other.parent_fram;
	this->patt_file_path = other.patt_file_path;
	this->center.first = other.center.first;
	this->center.second = other.center.second;
	this->width = other.width;
	this->static_transform = other.static_transform;
	this->is_goal = other.is_goal;
}

Marker::Marker(std::string name, std::string path, double centerX,
		double centerY, double width, bool is_goal) {
	this->name = name;
	this->patt_file_path = path;
	this->center.first = centerX;
	this->center.second = centerY;
	this->width = width;
	this->is_goal = is_goal;
}

//std::string Marker::toString() {
//  string retVal = n
//}

std::string Marker::toObjectFileString() {
	ostringstream ss;
	ss<<name<<"\n"<<patt_file_path<<"\n"<<width<<"\n"<<center.first<<" "
			<<center.second<<"\n\n";
	cout<<ss.str()<<endl;
	return ss.str();
}



//int main(){
//	MarkersFileParser test = MarkersFileParser("/home/mikab4/catkin_ws/src/mobil_shared_coordinates/example_test.yaml");
//	int num = 5;
//	int myarr[num];
//	string fff = "hey";
//
//}
