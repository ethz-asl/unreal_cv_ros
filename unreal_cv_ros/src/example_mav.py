#!/usr/bin/env python

# ros
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
import tf

# Python
import math
import numpy as np


class PathPublisher:

    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.delay = rospy.get_param('~delay', 0.0)     # Waiting time before publishing the path
        self.speed = rospy.get_param('~mav_velocity', 0.3)     # Velocity of mav m/s

        # publisher
        self.traj_pub = rospy.Publisher("command/trajectory", MultiDOFJointTrajectory, queue_size=10)

        # Prepare the trajectory, linear part
        self.traj_msg = MultiDOFJointTrajectory()
        self.traj_msg.joint_names = ["base_link"]
        sampling_rate = 20.0
        length = 4.5
        height = 0.5
        self.traj_time = 0.0
        l_curr = 0.0
        while l_curr < length:
            self.traj_time = self.traj_time + 1.0 / sampling_rate
            l_curr = self.speed * self.traj_time
            transforms = Transform()
            transforms.translation.x = l_curr
            transforms.translation.y = 0.0
            transforms.translation.z = height * l_curr / length
            transforms.rotation.x = 0.0
            transforms.rotation.y = 0.0
            transforms.rotation.z = 0.0
            transforms.rotation.w = 1.0
            point = MultiDOFJointTrajectoryPoint()
            point.transforms = [transforms]
            point.time_from_start = rospy.Duration(self.traj_time)
            self.traj_msg.points.append(point)

        # circular part
        self.traj_msg2 = MultiDOFJointTrajectory()
        self.traj_msg2.joint_names = ["base_link"]
        r_max = 1.5
        r_min = 0.5
        n_rotation = 1.5
        phi_curr = 0.0
        self.traj_time2 = 0.0
        while phi_curr <= n_rotation * 2 * math.pi:
            self.traj_time2 = self.traj_time2 + 1.0 / sampling_rate
            r_curr = r_max - (r_max-r_min) * phi_curr / (n_rotation * 2 * math.pi)
            phi_curr = phi_curr + self.speed / r_curr / sampling_rate
            transforms = Transform()
            transforms.translation.x = r_curr * math.sin(phi_curr) + length
            transforms.translation.y = r_curr * math.cos(phi_curr) - r_max
            transforms.translation.z = height
            quaternion = tf.transformations.quaternion_from_euler(0, 0, -phi_curr)
            transforms.rotation.x = quaternion[0]
            transforms.rotation.y = quaternion[1]
            transforms.rotation.z = quaternion[2]
            transforms.rotation.w = quaternion[3]
            point = MultiDOFJointTrajectoryPoint()
            point.transforms = [transforms]
            point.time_from_start = rospy.Duration(self.traj_time2)
            self.traj_msg2.points.append(point)

        self.launch_simulation()

    def launch_simulation(self):
        # Wait for unreal simulation to setup
        rospy.loginfo("Example Path Publisher: waiting for unreal MAV simulationto setup...")
        rospy.wait_for_message("unreal_simulation_ready", String)
        rospy.loginfo("Example Path Publisher: Waiting for unreal MAV simulationto setup... done.")

        if self.delay > 0:
            rospy.loginfo("Example Path Publisher: Launch in %d seconds.", self.delay)
            rospy.sleep(self.delay)

        # Publish the trajectory
        rospy.loginfo("Example Path Publisher: Succesfully started the simulation!")
        self.traj_msg.header.stamp = rospy.Time.now()
        self.traj_pub.publish(self.traj_msg)
        rospy.loginfo("Example Path Publisher: Requested Trajectory part 1/2.")
        rospy.sleep(self.traj_time)
        self.traj_msg2.header.stamp = rospy.Time.now()
        self.traj_pub.publish(self.traj_msg2)
        rospy.loginfo("Example Path Publisher: Requested Trajectory part 2/2.")
        rospy.sleep(self.traj_time2)
        rospy.loginfo("Example Path Publisher: Finished trajectory execution!")


if __name__ == '__main__':
    rospy.init_node('path_publisher', anonymous=True)
    pp = PathPublisher()

