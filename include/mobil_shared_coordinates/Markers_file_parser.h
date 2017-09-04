/*
 * Ymalparser.h
 *
 *  Created on: Aug 24, 2017
 *      Author: mikab4
 */

#ifndef MOBIL_SHARED_COORDINATES_SRC_MARKERS_FILE_PARSER_H_
#define MOBIL_SHARED_COORDINATES_SRC_MARKERS_FILE_PARSER_H_

#include <tf/transform_datatypes.h>


class Marker{
private:
	std::string name;
	std::string parent_fram;
	tf::Transform static_transform;
	std::string patt_file_path;
	double width;
	bool is_goal;
	std::pair<double, double> center;
public:
	// defult constructor
//	Marker(){}
	//constructor
	Marker(std::string name, std::string path, std::string parent_fram,
			double centerX, double centerY, double width,
			double transX, double transY, double transZ,
			double roll, double yaw, double pitch, bool is_goal=false);
	//constructor for goal marker
	Marker(std::string name, std::string path, double centerX,
			double centerY, double width, bool is_goal=true);
	// copy constructor
	Marker(const Marker &other);
	// String to put in the object file for ar_pose
	std::string toObjectFileString();

	bool isGoal() const {
		return is_goal;
	}

	const std::string& getName() const {
		return name;
	}

	const std::string& getParentFram() const {
		return parent_fram;
	}

	const tf::Transform& getStaticTransform() const {
		return static_transform;
	}
};


class MarkersFileParser {
private:
	std::string object_name;
	std::vector<Marker*> markers_list;
	int num_of_obj_marker;
	void parse(std::string file_path);
public:
	MarkersFileParser(std::string file_path);

	virtual ~MarkersFileParser();

	const std::vector<Marker*>& getMarkersList() const {
		return markers_list;
	}

	int getNumOfObjMarker() const {
		return num_of_obj_marker;
	}
};

#endif /* MOBIL_SHARED_COORDINATES_SRC_MARKERS_FILE_PARSER_H_ */
