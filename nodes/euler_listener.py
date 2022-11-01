#!/usr/bin/python3
import tf
import rospy
import math
from sensor_msgs.msg import Imu
from ros_genisys_imu_listener.ImuTopic import RosParam
from ros_genisys_imu_listener.Models import TopicImuTemp, EulerTimeStamp


class EulerListener:
    def __init__(self) -> None:
        rospy.init_node('euler_listener_node')
        self.init_time_stamp_ack : bool = True
        self.init_time_stamp : float = 0.0
        self.node_name : str = rospy.get_name()
        self.ros_param : RosParam = RosParam()
        self.topic_imutemp : TopicImuTemp = self.ros_param.get_topic_imu(node=self.node_name)
        self.imu_sub = rospy.Subscriber(self.topic_imutemp.imu_data, Imu, self.imu_sub_callback)
    
    def imu_sub_callback(self,data : Imu):
        euler_timestamp : EulerTimeStamp = EulerTimeStamp()
        orientation = data.orientation
        acceleration = data.linear_acceleration
        header = data.header
        now_timestamp : float = header.stamp.to_sec()
        if self.init_time_stamp_ack:
            self.init_time_stamp = now_timestamp
            self.init_time_stamp_ack = False
        rpy = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        euler_timestamp.rx = rpy[0]
        euler_timestamp.ry = rpy[1]
        euler_timestamp.rz = rpy[2]
        self.rad_to_deg(rad=euler_timestamp.ry)
        euler_timestamp.az = acceleration.z
        euler_timestamp.time = now_timestamp - self.init_time_stamp
        self.ros_param.set_param_imu_euler(euler_timestamp=euler_timestamp, node=self.node_name)

    def rad_to_deg(self,rad):
        deg : float = (rad * 180.0) / math.pi
        return (rad * 180.0) / math.pi

def main():
    EulerListener()
    rospy.spin()

if __name__ == "__main__":
    main()
    

    
    
    

   













