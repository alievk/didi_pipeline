#!/usr/bin/env python

import numpy as np

import rospy
import tf
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


last_cap_r = None
last_cap_f = None
last_cap_yaw = None


def rtk_position_to_numpy(msg):
    assert isinstance(msg, Odometry)
    p = msg.pose.pose.position
    return np.array([p.x, p.y, p.z])


def get_yaw(p1, p2):
    if abs(p1[0] - p2[0]) < 1e-2:
        return 0.
    return np.arctan2(p1[1] - p2[1], p1[0] - p2[0])


def handle_msg(msg, who):
    assert isinstance(msg, Odometry)

    global last_cap_r, last_cap_f, last_cap_yaw

    if who == 'cap_r':
        last_cap_r = rtk_position_to_numpy(msg)
    elif who == 'cap_f' and last_cap_r is not None:
        cap_f = rtk_position_to_numpy(msg)
        cap_r = last_cap_r

        last_cap_f = cap_f
        last_cap_yaw = get_yaw(cap_f, cap_r)
    elif who == 'obs_r' and last_cap_f is not None and last_cap_yaw is not None:
        cos = np.cos(-last_cap_yaw)
        sin = np.sin(-last_cap_yaw)
        rot_z = np.array([
            [cos, -sin, 0.],
            [sin, cos,  0.],
            [0,    0,   1.]
        ])

        mdr = {
            'obstacle_name': 'obs1',
            'object_type': 'Car',
            'gps_l': 2.032,
            'gps_w': 1.4478/2,
            'gps_h': 1.6256,
            'l': 4.2418,
            'w': 1.4478,
            'h': 1.5748,
        }

        lrg_to_gps = [mdr['gps_l'], -mdr['gps_w'], mdr['gps_h']]
        lrg_to_centroid = [mdr['l'] / 2., -mdr['w'] / 2., mdr['h'] / 2.]
        gps_to_centroid = np.subtract(lrg_to_centroid, lrg_to_gps)
        # print gps_to_centroid

        cap_f = last_cap_f
        obs_r = rtk_position_to_numpy(msg)

        cap_to_obs = obs_r - cap_f
        cap_to_obs = np.dot(rot_z, cap_to_obs)
        cap_to_obs_centroid = cap_to_obs + gps_to_centroid

        br = tf.TransformBroadcaster()
        br.sendTransform(tuple(cap_to_obs_centroid), (0,0,0,1), rospy.Time.now(), 'obs_centroid', 'gps_antenna_front')

        marker = Marker()
        marker.header.frame_id = "obs_centroid"
        marker.header.stamp = rospy.Time.now()

        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.scale.x = mdr['l']
        marker.scale.y = mdr['w']
        marker.scale.z = mdr['h']

        marker.color.r = 0.2
        marker.color.g = 0.5
        marker.color.b = 0.2
        marker.color.a = 0.5

        marker.lifetime = rospy.Duration()

        pub = rospy.Publisher("obs_bbox", Marker, queue_size=10)
        pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('base_link_to_obs1_tf_broadcaster')

    obj_topics = {
        'cap_r': '/objects/capture_vehicle/rear/gps/rtkfix',
        'cap_f': '/objects/capture_vehicle/front/gps/rtkfix',
        'obs_r': '/objects/obs1/rear/gps/rtkfix'
    }
    
    for obj in obj_topics:
        rospy.Subscriber(obj_topics[obj],
                         Odometry,
                         handle_msg,
                         obj)

    rospy.spin()

