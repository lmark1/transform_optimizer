#!/usr/bin/env python

import csv
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import EmptyResponse, Empty

class CameraCalibration:

  def __init__(self):
    self.odom_recieved = False
    self.odom_msg = Odometry()

    self.pose_recieved = False
    self.pose_msg = PoseStamped()

    rospy.Subscriber("odometry", Odometry, self.odom_cb)
    rospy.Subscriber("detected_pose", PoseStamped, self.detected_pose_cb)

    self.base_file = rospy.get_param("~base_file")
    self.poses_file = rospy.get_param("~poses_file")
    self.s = rospy.Service("record", Empty, self.record_cb)

  def odom_cb(self, msg):
    self.odom_recieved = True
    self.odom_msg = msg

  def detected_pose_cb(self, msg):
    self.pose_recieved = True
    self.pose_msg = msg

  def record_cb(self, msg):
    
    if not self.odom_recieved:
      print("Odometry not recieved")
      return EmptyResponse()

    if not self.pose_recieved:
      print("Pose not recieved")
      return EmptyResponse()

    with open(self.poses_file, "a") as f:
      f.write(
        str(self.pose_msg.pose.position.x) + ", " + 
        str(self.pose_msg.pose.position.y) + ", " + 
        str(self.pose_msg.pose.position.z) + "\n")

      with open(self.base_file, "a") as f:
        f.write("translation: \n")
        t = self.odom_msg.pose.pose.position
        f.write("x: "+ str(t.x) + "\n")
        f.write("y: "+ str(t.y)+ "\n")
        f.write("z: "+ str(t.z)+ "\n")
        f.write("rotation: \n")
        t = self.odom_msg.pose.pose.orientation
        f.write("x: "+ str(t.x)+ "\n")
        f.write("y: "+ str(t.y)+ "\n")
        f.write("z: "+ str(t.z)+ "\n")
        f.write("w: "+ str(t.w)+ "\n")
    return EmptyResponse()

if __name__ == '__main__':
  rospy.init_node("camera_calibration")
  cam = CameraCalibration()
  rospy.spin()
  