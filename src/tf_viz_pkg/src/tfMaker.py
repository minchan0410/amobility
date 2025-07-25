#!/usr/bin/env python3


# ============== Description / 정민찬 ============== #
# 각 미션에서 차량의 거동을 Rviz로 확인하기 위해 TF를 생성.

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler


yaw = 0
x = 0
y = 0

def yaw_Callback(msg):
    global yaw
    yaw = msg.data

def utm_Callback(msg):
    global x,y
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y


    # -- TF publish -- #

    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "base_link_gps"

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 1.7 # gps 높이

    q = quaternion_from_euler(0,0,yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    # mission viewer 

def path_Callback(msg):
    path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
    
    min_dist = 100000 # inf
    nearest_idx = -1
    nearest_pnt = None
    for i, (px, py) in enumerate(path):
        dist = ((px-x)**2 + (py-y)**2)**0.5
        if dist < min_dist:
            nearest_idx = i
            nearest_pnt = (px, py)
    
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "viewer"

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 1.7 # gps 높이

    q = quaternion_from_euler(0,0,0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    ## warning 발생할 수 있는데 이는 use sim time 파라미터 설정 때문임.
    ## 실차 운행에서는 해당 파라미터 false 하면 에러 없음


def main():
    rospy.init_node('tfmaker')

    rospy.Subscriber('/odom_gps', Odometry, utm_Callback)
    rospy.Subscriber('/vehicle_yaw', Float32, yaw_Callback)
    rospy.Subscriber('/global_path', Path, path_Callback)

    rospy.loginfo("Map --> Base_link TF Publish Started")
    rospy.spin()

if __name__ == '__main__':
    main()
