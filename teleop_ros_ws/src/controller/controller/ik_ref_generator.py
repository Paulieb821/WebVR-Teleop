import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray

import os
import numpy as np
import mujoco as mj
from controller.NRIK import NRIK as ik


class IK_Reference_Generator(Node):

    def __init__(self):
        super().__init__('ik_reference_generator')

        # URDF Info (should be entered by the user or a config file long term)
        urdf_path = 'robot_models/4dof_arm_v2/4dof_arm_v2.xml'
        site_name = 'endeff'

        # Model and Data
        self.model = mj.MjModel.from_xml_path(urdf_path)  # MuJoCo model
        self.data = mj.MjData(self.model)                 # MuJoCo data
        self.numJoints = self.model.nv
        self.site = self.data.site(site_name)

        # IK solver and robot model
        self.ik = ik(self.model, self.data, self.site)
        mj.mj_forward(self.ik.model, self.ik.data)

        # Tracking active state and subscriber
        self.tracking_active = False
        self.tracking_active_sub = self.create_subscription(Bool, '/tracking_active', self.tracking_active_callback, 10)
        self.tracking_active_sub  # prevent unused variable warning
        
        # Controller position subscriber
        self.controller_sub = self.create_subscription(Point, '/right_controller_pos', self.ik_callback, 10)
        self.controller_sub  # prevent unused variable warning

        # Joint reference publisher - sends out references at a specified interval
        self.joint_msg = Float64MultiArray()
        self.joint_msg.data = self.data.qpos.flatten().tolist()
        self.joint_ref_pub = self.create_publisher(Float64MultiArray, '/joint_ref', 10)
        self.joint_ref_time = self.create_timer(0.02, self.send_joint_ref_callback)


    def tracking_active_callback(self, track_act):
        self.tracking_active = track_act.data

    def ik_callback(self, controller_pos):
        if self.tracking_active:
            pos_vec = np.array([controller_pos.x, controller_pos.y, controller_pos.z])
            angle_vec = self.ik.solveIK_3dof(pos_vec)
            #print(pos_vec)
            self.joint_msg.data = angle_vec.flatten().tolist()

    def send_joint_ref_callback(self):
        self.joint_ref_pub.publish(self.joint_msg)

        


def main(args=None):
    rclpy.init(args=args)

    ik_reference_generator = IK_Reference_Generator()

    rclpy.spin(ik_reference_generator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ik_reference_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()