#!/usr/bin/env python

import rospy
import sys
from mobil_shared_coordinates.srv import object_data
import roslaunch

def get_robot_data(robot_name, is_first):
    service_name = 'object_data_%s' % robot_name
    rospy.wait_for_service(service_name)
    try:
        object_data_srv = rospy.ServiceProxy(service_name, object_data)
        rsp = object_data_srv()
        if is_first:
            prem = 'w+'
        else:
            prem = 'a+'

        with open(object_data_file, prem) as fd:
            fd.write(rsp.file_contant)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == "__main__":

    rospy.init_node('marker_data_client.py')

    if rospy.has_param('~object_data_file_path'):
        global object_data_file
        object_data_file = rospy.get_param("~object_data_file_path")
    else:
        raise Exception('No object_data_file_path param')

    if rospy.has_param('~ar_pose_launch_file'):
        ar_pose_launch_file = rospy.get_param("~ar_pose_launch_file")
    else:
        raise Exception('No ar_pose_launch_file param')

    if rospy.has_param('~team_members'):
        team_members = rospy.get_param("~team_members")
        team_members = [x.strip() for x in team_members.split(',')]
        print team_members
    else:
        raise Exception('No team_members param')


    # gets all the robots object_data file and unify them to one data file
    is_first = True
    for robot_name in team_members:
        get_robot_data(robot_name, is_first)
        is_first = False

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [ar_pose_launch_file])
    launch.start()
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

    launch.shutdown()



















