#!/usr/bin/python3
import math
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from ros_genisys_imu_listener.Models import EulerTimeStamp
from ros_genisys_imu_listener.ImuParam import ImuEulerParam


_fig = plt.figure()
ax = _fig.add_subplot(1, 1, 1)
time = [] 
row = [] 
pitch = [] 
yaw = [] 

def get_euler() -> EulerTimeStamp:
    imuEulerParam : ImuEulerParam = ImuEulerParam()
    eulerTimeStamp : EulerTimeStamp = EulerTimeStamp()
    if rospy.has_param('euler_listener_node'):
        eulerTimeStamp.time = rospy.get_param(f"euler_listener_node{imuEulerParam.time}")
        eulerTimeStamp.rx = rospy.get_param(f"euler_listener_node{imuEulerParam.rx}")
        eulerTimeStamp.ry = rospy.get_param(f"euler_listener_node{imuEulerParam.ry}")
        eulerTimeStamp.rz = rospy.get_param(f"euler_listener_node{imuEulerParam.rz}")
    else:
        eulerTimeStamp.time = 0.0
        eulerTimeStamp.rx = 0.0
        eulerTimeStamp.ry = 0.0
        eulerTimeStamp.rz = 0.0
    return eulerTimeStamp

def rad_to_deg(rad):
    return (rad * 180.0) / math.pi

def animate(i, time, row):
    eulerTimeStamp : EulerTimeStamp = get_euler()
    i = eulerTimeStamp.time
    row_deg = rad_to_deg(eulerTimeStamp.rx)
    pitch_deg = rad_to_deg(eulerTimeStamp.ry)
    yaw_deg = rad_to_deg(eulerTimeStamp.rz) 
    time.append(i)
    row.append(row_deg)
    pitch.append(pitch_deg)
    yaw.append(yaw_deg)
    ax.clear()
    ax.plot(time, row, label="Row", color='r')
    ax.plot(time, pitch, label="Pitch", color='g')
    ax.plot(time, yaw, label="Yaw", color='b')
    plt.xticks(rotation=45, ha='right')
    plt.title('IMU Monitor')
    plt.ylabel('Orentation (deg)')
    plt.xlabel('time (sec)')
    plt.legend(loc='upper right')
    plt.axis([i - 10, i, -360, 360]) 
    



def main():
    rospy.init_node('euler_graph_node')
    fig = animation.FuncAnimation(_fig, animate, fargs=(time, row), interval=10)
    plt.show()

if __name__ == "__main__":
    main()