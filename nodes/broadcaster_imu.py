#!/usr/bin/python3
import tf
import math
from pyquaternion import Quaternion
import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu

class TfBroadcasterImu:
    def __init__(self) -> None:
        rospy.init_node('tf_broadcaster_imu')
        self.node_name : str = rospy.get_name()
        self.pipe_radius : float = self.get_pipe_radius()
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.handle_imu_pose)

    def get_pipe_radius(self) -> float:
        pipe_radius : float = 0.0
        pipe_radius_name : str = f"{self.node_name}/pipe_radius"
        if rospy.has_param(pipe_radius_name):
            pipe_radius = rospy.get_param(pipe_radius_name)
        return pipe_radius

    def handle_imu_pose(self, msg : Imu):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "plane"
        t.child_frame_id = "imu_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.pipe_radius
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w
        br.sendTransform(t)

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        print(self.rad_to_deg(rad=pitch_y))
        return roll_x, pitch_y, yaw_z # in radians

    def rad_to_deg(self,rad):
        return (rad * 180.0) / math.pi

def main() -> None:
    TfBroadcasterImu()
    rospy.spin()

if __name__ == '__main__':
      main()










































# import cv2
# import math
# import rospy
# import numpy as np
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from ros_genisys_imu_listener.ImuParam import ImuEulerParam
# from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
# from ros_genisys_imu_listener.Models import EulerTimeStamp, RobotPosition





# class YawVisualize:
#     def __init__(self) -> None:
#         rospy.init_node('yaw_visualize_node', anonymous=True)
#         self.rate = rospy.Rate(10)
#         self.node_name : str = rospy.get_name()
#         self.init_yaw : float = 0.0
#         self.is_init_yaw : bool = self.set_is_init_yaw()
#         self.yaw_offset : float = self.set_yaw_offset()
#         self.mem_init_yaw : bool = self.is_init_yaw
#         self.bridge = CvBridge()
#         self.font_offset : int = 20
#         self.default_radius : float = 0.50
#         self.background_size : int = 950
#         self.background_pipe_r : int = 250
#         self.background_color : int = 100
#         self.background = np.zeros((self.background_size,self.background_size,3),np.uint8)
#         self.background.fill(self.background_color)
#         self.pipe_radius : float = self.get_pipe_radius(node=self.node_name)
#         self.pub_yaw_visualize = rospy.Publisher(f"{self.node_name}/yaw_visualize/image", Image, queue_size=1)
#         self.service_reset_yaw = rospy.Service(f"{self.node_name}/yaw_visualize/reset_yaw", Trigger, self.reset_yaw_callback)

#     def reset_yaw_callback(self, req : TriggerRequest) -> TriggerResponse:
#         trigger_response : TriggerResponse = TriggerResponse()
#         self.is_init_yaw = True
#         trigger_response.success = True
#         trigger_response.message = "Yaw has been reseted."
#         return trigger_response

#     def set_is_init_yaw(self) -> bool:
#         is_init_yaw : bool = False
#         is_init_yaw_param : str = f"{self.node_name}/is_init_yaw"
#         if rospy.has_param(is_init_yaw_param):
#             is_init_yaw = rospy.get_param(is_init_yaw_param)
#         return is_init_yaw

#     def set_yaw_offset(self) -> float:
#         yaw_offset : float = 0.0
#         yaw_offset_param : str = f"{self.node_name}/yaw_offset"
#         if rospy.has_param(yaw_offset_param):
#             yaw_offset = rospy.get_param(yaw_offset_param)
#         return yaw_offset

#     def get_pipe_radius(self, node : str) -> str:
#         param_name : str = f"{node}/pipe_radius"
#         if rospy.has_param(param_name):
#             return rospy.get_param(param_name)
#         else:
#             return self.default_radius

#     def get_euler(self) -> EulerTimeStamp:
#         imuEulerParam : ImuEulerParam = ImuEulerParam()
#         eulerTimeStamp : EulerTimeStamp = EulerTimeStamp()
#         if rospy.has_param(f"euler_listener_node{imuEulerParam.time}"):
#             eulerTimeStamp.time = rospy.get_param(f"euler_listener_node{imuEulerParam.time}")
#             eulerTimeStamp.rx = rospy.get_param(f"euler_listener_node{imuEulerParam.rx}")
#             eulerTimeStamp.ry = rospy.get_param(f"euler_listener_node{imuEulerParam.ry}")
#             eulerTimeStamp.rz = rospy.get_param(f"euler_listener_node{imuEulerParam.rz}")
#             eulerTimeStamp.az = rospy.get_param(f"euler_listener_node{imuEulerParam.az}")
#             if self.is_init_yaw:
#                 self.init_yaw = eulerTimeStamp.rz
#                 self.is_init_yaw = False
#             return eulerTimeStamp
#         else:
#             eulerTimeStamp.time = 0.0
#             eulerTimeStamp.rx = 0.0
#             eulerTimeStamp.ry = 0.0
#             eulerTimeStamp.rz = 0.0
#             eulerTimeStamp.az = 10.0
#             return eulerTimeStamp

#     def cv_to_rosimage(self, cv_image) -> Image:
#         return self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

#     def pub_visualize(self) -> None:
#         self.draw_visualize()
#         r_p : RobotPosition = self.get_robot_position()
#         self.draw_visualize_position(center_coordinates=(int(r_p.x_vis),int(r_p.y_vis)))
#         self.draw_info(robot_position=r_p)
#         self.pub_yaw_visualize.publish(self.cv_to_rosimage(cv_image=self.background))
#         self.clear_image()

#     def get_robot_position(self) -> RobotPosition: 
#         euler_time_stamp : EulerTimeStamp = self.get_euler()  
#         ph_radius : float = self.pipe_radius
#         vis_radius : float = self.background_pipe_r
#         robot_position : RobotPosition = self.set_robot_position()
#         img_cx, img_cy = int(self.background_size / 2), int(self.background_size / 2) 

#         angle_now : float = euler_time_stamp.rz
#         angle_init : float = self.init_yaw
#         robot_position.beta = angle_now - angle_init

#         robot_position.x_ph, robot_position.y_ph = ph_radius * math.cos(robot_position.beta), ph_radius * math.sin(robot_position.beta)
#         robot_position.x_vis, robot_position.y_vis = vis_radius * math.cos(robot_position.beta) + img_cx, vis_radius * math.sin(robot_position.beta) + img_cy
#         robot_position.dis_ph = robot_position.beta * ph_radius       
#         return robot_position
   
#     def draw_visualize(self) -> None:
#         p_radius = self.background_pipe_r
#         img_cx, img_cy = int(self.background_size / 2), int(self.background_size / 2)  
#         self.draw_rectangle(center_coordinates=(img_cx, img_cy), radius=p_radius)
#         self.draw_line(start_point=(img_cx - p_radius, img_cy), end_point=(img_cx + p_radius, img_cy))
#         self.draw_line(start_point=(img_cx , img_cy - p_radius), end_point=(img_cx , img_cy + p_radius))

#     def draw_visualize_position(self, center_coordinates) -> None:
#         color = (0, 0, 255)
#         if self.mem_init_yaw:
#             color = (0, 255, 0)
#         thickness = 3
#         cv2.arrowedLine(self.background, (int(self.background_size / 2), int(self.background_size / 2)), center_coordinates,color, thickness)

#     def rad_to_deg(self,rad):
#         return (rad * 180.0) / math.pi

#     def draw_info(self, robot_position : RobotPosition) -> None:
#         font = cv2.FONT_HERSHEY_SIMPLEX
#         org = (int(robot_position.x_vis) + self.font_offset, int(robot_position.y_vis))
#         fontScale = 0.75
#         color = (255, 255, 255)
#         thickness = 2
#         beta : float = float("{:.2f}".format(self.rad_to_deg(robot_position.beta) % 360.0))
#         cv2.putText(self.background, f"Angle: {beta}", 
#         org, font, fontScale, color, thickness, cv2.LINE_AA)
     
#     def draw_rectangle(self, center_coordinates, radius) -> None:
#         color = (0, 0, 0)
#         thickness = 3
#         cv2.rectangle(self.background, (center_coordinates[0] - radius, 0), (center_coordinates[0] + radius, self.background_size), color, thickness)

#     def draw_line(self,start_point ,end_point) -> None:
#         color = (0, 0, 0)
#         thickness = 3
#         cv2.line(self.background, start_point, end_point, color, thickness)

#     def set_robot_position(self) -> RobotPosition:
#         robot_position : RobotPosition = RobotPosition() 
#         robot_position.beta = 0.0
#         robot_position.x_ph = 0.0
#         robot_position.y_ph = 0.0
#         robot_position.x_vis = 0.0
#         robot_position.y_vis = 0.0
#         robot_position.dis_ph = 0.0
#         return robot_position

#     def clear_image(self) -> None:
#         self.background = np.zeros((self.background_size,self.background_size,3),np.uint8)
#         self.background.fill(self.background_color)

# def main():
#     yaw_visualize : YawVisualize = YawVisualize()
#     while not rospy.is_shutdown():
#         yaw_visualize.pub_visualize()
#         yaw_visualize.rate.sleep()
    

# if __name__ == "__main__":
#     main()