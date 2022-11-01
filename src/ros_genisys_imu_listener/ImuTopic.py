import rospy
from .ImuParam import ImuEulerParam
from .Models import TopicImuTemp, EulerTimeStamp




class RosParam:
    def __init__(self) -> None:
        self.imu_data_default : str = "/imu/data"
        self.imu_mag_default : str = "/imu/mag"
        self.imu_euler_param : ImuEulerParam = ImuEulerParam()
        
    def get_topic_imu(self, node : str) -> TopicImuTemp: 
        topic_imu : TopicImuTemp = TopicImuTemp()  
        imu_data_param : str = f"{node}/topic_imu/imu_data"
        imu_mag_param : str = f"{node}/topic_imu/imu_mag"
        if rospy.has_param(imu_data_param):
            topic_imu.imu_data = rospy.get_param(imu_data_param)
        else:
            topic_imu.imu_data = self.imu_data_default
        if rospy.has_param(imu_mag_param):
            topic_imu.imu_mag = rospy.get_param(imu_mag_param)
        else:
            topic_imu.imu_mag = self.imu_mag_default
        return topic_imu

    def set_param_imu_euler(self, euler_timestamp : EulerTimeStamp, node : str) -> None:
        rospy.set_param(f"{node}{self.imu_euler_param.rx}", euler_timestamp.rx)
        rospy.set_param(f"{node}{self.imu_euler_param.ry}", euler_timestamp.ry)
        rospy.set_param(f"{node}{self.imu_euler_param.rz}", euler_timestamp.rz)
        rospy.set_param(f"{node}{self.imu_euler_param.az}", euler_timestamp.az)
        rospy.set_param(f"{node}{self.imu_euler_param.time}", euler_timestamp.time)

        
        
        

        


        
        




    












