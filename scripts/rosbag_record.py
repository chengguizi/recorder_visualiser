#!/usr/bin/env python

# ROS only supports Python 2

import rospy
from subprocess import Popen, PIPE, STDOUT, check_output, call
# from std_msgs.msg import String
from std_srvs.srv import SetBool

from time import sleep

import os
import sys
import psutil

process = 0


def srvCallbackRecord(req):

    global process

    if req.data == False:
        rospy.loginfo("Ending Existing Recording")
    else:
        rospy.loginfo("Start New Recording")

    # check disk usage

    disk_obj = psutil.disk_usage('/')
    rospy.logwarn("Free Disk Space: %.2f GB" % (disk_obj.free / (1024**3)))
    # check if the process already exist
    # process_rosbag = Popen(['pgrep','rosbag'])
    # process_rosbag.wait()


    # pid_rosbag = ''
    # try:
    #     pid_rosbag = check_output(['pgrep','rosbag'])
    # except Exception as e:
    #     output = str(e.output)

    # if pid_rosbag:
    #     pid_rosbag_vec = pid_rosbag.split('\n')

    #     for pid in pid_rosbag_vec:
    #         if pid:
    #             call(['pkill',pid,'-SIGINT'])
    #             rospy.loginfo('killing pid: ' + pid )

    call(['rosnode', 'kill','/my_record_bag'])

    if (req.data): # start recording
        topic_sensor = '/reset /imu0 /mag0 /rosout_agg /stereo/camera_stats /stereo/left/camera_info /stereo/left/image_rect_raw /stereo/right/camera_info /stereo/right/image_rect_raw'
        topic_results = ''
        # topic_results = '/dead_reckoning/pose /ekf_fusion/pose /ekf_fusion/pose_corrected /ekf_fusion/state_out /right/debug_left /right/debug_right /rosout_agg /stereo_odometer/pose /stereo_odometer/velocity /stereo_odometer/odometry'
        node_name = '__name:=my_record_bag'
        # process = Popen(['rosbag', 'record'] + topic_sensor.split(' ') + topic_results.split(' '), stdout=PIPE, stderr=STDOUT, shell=True)
        process = Popen(['rosbag', 'record'] + topic_sensor.split(' ') + topic_results.split(' ') + [node_name])
        rospy.loginfo( 'Starts with PID ' + str(process.pid) )
        return [True, "Recording Starts"]
    else:
        return [True, "Recording Ends"]

def srvCallbackWifi(req):

    if req.data == False:
        ret = call(["sudo","ifdown","wlp4s0"])
        return [not ret, "Wifi Disabled"]
    else:
        ret = call(["sudo","ifup","wlp4s0"])
        return [not ret, "Wifi Enabled"]
    


rospy.init_node('recorder')

rospy.logwarn("ROS Recorder Node Starts!")

dir_path = os.getcwd()
rospy.logwarn("ROS Recorder Current Path" + dir_path)

rospy.Service('record', SetBool, srvCallbackRecord)
rospy.Service('wifi', SetBool, srvCallbackWifi)

# while not rospy.is_shutdown():
#     if process:
#         print("true")
#         stdout = process.stdout.readline()
#         if stdout:
#             rospy.loginfo(stdout)
#     rospy.sleep(1)

rospy.spin()
