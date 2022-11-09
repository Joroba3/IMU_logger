#!/usr/bin/env python
import os
import rospy
from sensor_msgs.msg import Imu
from sbg_driver.msg import SbgGpsPos, SbgEkfEuler
from datetime import datetime

time_str = datetime.now().strftime("%Y%m%d_%H%M")
os.mkdir("/home/jacobo/" + time_str)

imu_data_file = open("/home/jacobo/{0}/{0}_imu_data.csv".format(time_str), "w")
imu_data_file.write("sec; acc_x; acc_y; acc_z; angvel_x; angvel_y; angvel_z; or_x; or_y; or_z; or_w; or_cov\n")

gps_pos_file = open("/home/jacobo/{0}/{0}_gps_pos.csv".format(time_str), "w")
gps_pos_file.write("sec; latitude; longitude; pos_acc_x; pos_acc_y\n")

ekf_euler_file = open("/home/jacobo/{0}/{0}_ekf_euler.csv".format(time_str), "w")
ekf_euler_file.write("sec; angle_x; angle_y; angle_z; acc_x; acc_y; acc_y\n")

def imu_callback(data):
    imu_data_file.write("{};{};{};{};{};{};{};{};{};{};{};{}\n".format(rospy.Time.now().to_sec(),
                                                                     data.linear_acceleration.x,
                                                                     data.linear_acceleration.y,
                                                                     data.linear_acceleration.z,
                                                                     data.angular_velocity.x,
                                                                     data.angular_velocity.y,
                                                                     data.angular_velocity.z,
                                                                     data.orientation.x,
                                                                     data.orientation.y,
                                                                     data.orientation.z,
                                                                     data.orientation.w,
                                                                     data.orientation_covariance))

def gps_callback(data):
    gps_pos_file.write("{};{};{};{};{}\n".format(rospy.Time.now().to_sec(),
                                              data.latitude,
                                              data.longitude,
                                              data.position_accuracy.x,
                                              data.position_accuracy.y))

def euler_callback(data):
    ekf_euler_file.write("{};{};{};{};{};{};{}\n".format(rospy.Time.now().to_sec(),
                                                         data.angle.x,
                                                         data.angle.y,
                                                         data.angle.z,
                                                         data.accuracy.x,
                                                         data.accuracy.y,
                                                         data.accuracy.z))

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/imu/data", Imu, imu_callback)
    rospy.Subscriber("/sbg/gps_pos", SbgGpsPos, gps_callback)
    rospy.Subscriber("/sbg/ekf_euler", SbgEkfEuler, euler_callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
