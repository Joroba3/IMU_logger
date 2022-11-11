#!/usr/bin/env python
import os
import subprocess
import rospy
import rosbag
from sbg_driver.msg import SbgGpsPos, SbgEkfEuler, SbgImuData, SbgGpsVel
from datetime import datetime

time_str = datetime.now().strftime("%Y%m%d_%H%M")
os.mkdir("/home/jacobo/" + time_str)
bash_command = "rosbag record --all -O /home/jacobo/{0}/rec.bag".format(time_str)
subprocess.Popen(bash_command.split())


imu_data_file = open("/home/jacobo/{0}/{0}_imu_data.csv".format(time_str), "w")
imu_data_file.write("time(s);acc_x(m/s2);acc_y(m/s2);acc_z(m/s2);deltav_x(m/s2);deltav_y(m/s2);deltav_z(m/s2);gyro_x(rad/s);gyro_y(rad/s);gyro_z(rad/s)\n")

gps_pos_file = open("/home/jacobo/{0}/{0}_gps_pos.csv".format(time_str), "w")
gps_pos_file.write("sec(time); latitude(deg); longitude(deg); pos_acc_x(m); pos_acc_y(m)\n")

gps_vel_file = open("/home/jacobo/{0}/{0}_gps_vel.csv".format(time_str), "w")
gps_vel_file.write("sec(time);vel_x(m/s);vel_y(m/s);vel_z(m/s); vacc_x(m/s); vacc_y(m/s); vacc_z(m/s)\n")

ekf_euler_file = open("/home/jacobo/{0}/{0}_ekf_euler.csv".format(time_str), "w")
ekf_euler_file.write("sec;angle_x(rad);angle_y(rad);angle_z(rad);acc_x(rad);acc_y(rad);acc_y(rad)\n")


def imu_callback(data):
    imu_data_file.write("{};{};{};{};{};{};{};{};{};{}\n".format(rospy.Time.now().to_sec(),
                                                        data.accel.x,
                                                        data.accel.y,
                                                        data.accel.z,
                                                        data.delta_vel.x,
                                                        data.delta_vel.y,
                                                        data.delta_vel.z,
                                                        data.gyro.x,
                                                        data.gyro.x,
                                                        data.gyro.y,
                                                        data.gyro.z))


def gps_callback(data):
    gps_pos_file.write("{};{};{};{};{}\n".format(rospy.Time.now().to_sec(),
                                                 data.latitude,
                                                 data.longitude,
                                                 data.position_accuracy.x,
                                                 data.position_accuracy.y))
def gps_vel_callback(data):
    gps_vel_file.write("{};{};{};{};{};{};{}\n".format(rospy.Time.now().to_sec(),
                                                       data.velocity.x,
                                                       data.velocity.y,
                                                       data.velocity.z,
                                                       data.velocity_accuracy.x,
                                                       data.velocity_accuracy.y,
                                                       data.velocity_accuracy.z))

def euler_callback(data):
    ekf_euler_file.write("{};{};{};{};{};{};{}\n".format(rospy.Time.now().to_sec(),
                                                         data.angle.x,
                                                         data.angle.y,
                                                         data.angle.z,
                                                         data.accuracy.x,
                                                         data.accuracy.y,
                                                         data.accuracy.z))
    print(data.status)

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sbg/imu_data", SbgImuData, imu_callback)
    rospy.Subscriber("/sbg/gps_pos", SbgGpsPos, gps_callback)
    rospy.Subscriber("/sbg/gps_vel", SbgGpsVel, gps_vel_callback)
    rospy.Subscriber("/sbg/ekf_euler", SbgEkfEuler, euler_callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
