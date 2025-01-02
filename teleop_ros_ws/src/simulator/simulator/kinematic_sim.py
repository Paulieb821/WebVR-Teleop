import mujoco as mj
from mujoco.glfw import glfw
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np

# ROS 2 Node Setup
class Kinematic_Simulator(Node):
    def __init__(self):
        super().__init__('kinematic_simulator')

        # Subscriber for joint positions
        self.joint_positions = None
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/joint_ref',
            self.joint_state_callback,
            10
        )

        # Publisher for end effector positioh
        self.endeff_pos_pub = self.create_publisher(Point, '/endeff_pos', 10)

        # MuJoCo Setup
        urdf_path = 'robot_models/4dof_arm_v2/4dof_arm_v2.xml'
        site_name = "endeff"

        # Load the MuJoCo model and data
        self.model = mj.MjModel.from_xml_path(urdf_path)
        self.data = mj.MjData(self.model)
        self.site = self.data.site(site_name)
        self.cam = mj.MjvCamera()
        self.opt = mj.MjvOption()

        # Initialize GLFW and window
        glfw.init()
        self.window = glfw.create_window(1200, 900, "Mujoco Kinematics Visualizer", None, None)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        # Initialize visualization data structures
        mj.mjv_defaultCamera(self.cam)
        mj.mjv_defaultOption(self.opt)
        self.scene = mj.MjvScene(self.model, maxgeom=10000)
        self.ctx = mj.MjrContext(self.model, mj.mjtFontScale.mjFONTSCALE_150)

        # Run the main loop
        self.timer = self.create_timer(1/50, self.update_scene)  # 50Hz timer

    def joint_state_callback(self, msg):
        """Callback to update the joint positions from the ROS topic."""
        self.joint_positions = np.array(msg.data)

    def update_scene(self):
        """Update the scene with new joint positions."""
        if self.joint_positions is not None:
            self.data.qpos[:len(self.joint_positions)] = self.joint_positions

        # Step the simulation forward
        mj.mj_forward(self.model, self.data)

        # Set camera configurations
        self.cam.azimuth = 135
        self.cam.elevation = -15
        self.cam.distance = 1.5
        self.cam.lookat = np.array([0, 0, 0.35])

        # Publish endeff position
        msg = Point()
        msg.x = self.site.xpos[0]
        msg.y = self.site.xpos[1]
        msg.z = self.site.xpos[2]
        self.endeff_pos_pub.publish(msg)

        # Update the scene and render
        mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam, mj.mjtCatBit.mjCAT_ALL.value, self.scene)
        viewport_width, viewport_height = glfw.get_framebuffer_size(self.window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
        mj.mjr_render(viewport, self.scene, self.ctx)

        # Swap OpenGL buffers
        glfw.swap_buffers(self.window)

        # Process pending GUI events, call GLFW callbacks
        glfw.poll_events()


def main(args=None):
    rclpy.init(args=args)

    kinematic_sim = Kinematic_Simulator()

    rclpy.spin(kinematic_sim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kinematic_sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()