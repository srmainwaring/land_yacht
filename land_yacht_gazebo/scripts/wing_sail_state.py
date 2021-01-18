#!/usr/bin/env python3

'''
Tools for analysing the state of the Gazebo wing_sail model


References
----------
http://gazebosim.org/tutorials/?tut=ros_comm
https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/include/gazebo_plugins/gazebo_ros_ft_sensor.h


Notes:

- The force and torque on the wingsail joint is obtained using the gazebo_ros ft_sensor plugin
  and published to /gazebo/ft_sensor/wing_sail_joint
- The joint force and torque is reported in the CHILD frame, wing_sail_link
- The transform to the world from is obtained using a rosservice call:

    $ rosservice call /gazebo/get_link_state '{link_name: wing_sail_link, reference_frame: world }'

  as the /tf is not available for this model (no joint state publisher or robot description).

- set model state

    $ rosservice call /gazebo/set_model_state '{model_state: { model_name: wing_sail, pose: { position: { x: 0, y: 0 ,z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 1 } }, twist: { linear: {x: 0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 1.0 } } , reference_frame: world } }'

- set link state

    $ rosservice call /gazebo/set_link_state '{link_state: { link_name: base_link, pose: { position: { x: 0, y: 0 ,z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 1 } }, twist: { linear: {x: 0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 1.0 } } } }'

'''

import rospy

from geometry_msgs.msg import WrenchStamped
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import GetLinkState, SetLinkState

import numpy as np
from scipy.spatial.transform import Rotation

def do_transform_vector(rot, trans, vector):
    # transform the origin
    origin = np.array([0, 0, 0])
    origin_t = rot.apply(origin) + trans

    # transform the vector end point
    vector_t = rot.apply(vector) + trans

    return vector_t - origin_t


class WingSailStateNode(object):

    def __init__(self):
        self._link_yaw = 0

        # subscriptions

        # ft_sensor messages are broadcast in the CHILD link frame
        self._wing_sail_joint_wrench = WrenchStamped()
        self._wing_sail_joint_sub = rospy.Subscriber(
            '/gazebo/ft_sensor/wing_sail_joint', WrenchStamped, self._wing_sail_joint_cb)

        # pose of the child link frame in the world
        rospy.wait_for_service('/gazebo/get_link_state')
        self._get_link_state_proxy = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        rospy.wait_for_service('/gazebo/set_link_state')
        self._set_link_state_proxy = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)

    def update(self, event):
        # get link state
        response = GetLinkState()
        try:
            response = self._get_link_state_proxy("wing_sail_link", "world")
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: {}".format(e))

        # create transform
        v = response.link_state.pose.position
        q = response.link_state.pose.orientation
        tf_pos = np.array([v.x, v.y, v.z])
        tf_rot = Rotation.from_quat(np.array([q.x, q.y, q.z, q.w]))

        # get force vector
        v = self._wing_sail_joint_wrench.wrench.force
        force_body = np.array([v.x, v.y, v.z])

        # transform from the joint's child link frame to world frame
        force_world = do_transform_vector(tf_rot, tf_pos, force_body)

        print("tf_pos [m]:           {}".format(np.round(tf_pos, 4)))
        print("tf_rot [deg]:         {}".format(np.round(np.degrees(tf_rot.as_euler("xyz")), 2)))
        print("force_body [N]:       {}".format(np.round(force_body, 4)))
        print("force_world [N]:      {}".format(np.round(force_world, 4)))
        print("force_world_norm [N]: {}".format(np.linalg.norm(force_world)))
        print()
        
        # rotate base link about z
        try:
            # increase yaw by 1 deg modulo 360 deg
            self._link_yaw = np.radians((np.degrees(self._link_yaw) + 1) % 360)
            rot = Rotation.from_euler("xyz", [0, 0, self._link_yaw])
            q = rot.as_quat()
            link_state = LinkState()
            link_state.link_name = "base_link"
            link_state.pose.position.x = 0
            link_state.pose.position.y = 0
            link_state.pose.position.z = 0
            link_state.pose.orientation.x = q[0]
            link_state.pose.orientation.y = q[1]
            link_state.pose.orientation.z = q[2]
            link_state.pose.orientation.w = q[3]
            link_state.twist.linear.x = 0
            link_state.twist.linear.y = 0
            link_state.twist.linear.z = 0
            link_state.twist.angular.x = 0
            link_state.twist.angular.y = 0
            link_state.twist.angular.z = 0
            link_state.reference_frame = "world"
            response = self._link_state = self._set_link_state_proxy(link_state)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: {}".format(e))

    def _wing_sail_joint_cb(self, msg):
        # capture the message
        self._wing_sail_joint_wrench = msg

if __name__ == "__main__":
    rospy.init_node("wing_sail_state")

    rospy.loginfo("Starting wing_sail_state node")

    # create node
    node = WingSailStateNode()

    # start the update loop (10 Hz)
    update_frequency = 10.0
    
    rospy.loginfo("Starting update loop at {} Hz".format(update_frequency))
    update_timer = rospy.Timer(
        rospy.Duration(1.0 / update_frequency),
        node.update)

    # wait for shutdown
    rospy.spin()






