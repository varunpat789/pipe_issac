from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import omni
from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.extensions import enable_extension
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.types import ArticulationAction
import omni.graph.core as og
import omni.replicator.core as rep
from isaacsim.sensors.camera import Camera
import omni.syntheticdata._syntheticdata as sd


enable_extension("isaacsim.ros2.bridge")

simulation_app.update()

import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import isaacsim.core.utils.numpy.rotations as rot_utils


STORM_SIGHT_USD_PATH = "/home/biorobotics/Documents/varun/arpa/pipe_issac_ws/src/pipe_issac/pipe_issac_sim/stormRobot.usd"

class StormRunnerSim(Node):
    def __init__(self):
        super().__init__("storm_runner_sim")

        self.timeline = omni.timeline.get_timeline_interface()
        self.ros_world = World(stage_units_in_meters=1.0)
        self.ros_world.scene.add_default_ground_plane()

        self.storm_runner = self.ros_world.scene.add(
            WheeledRobot(
                prim_path="/World/Storm_Sight_Runner",
                name="Storm_Sight_Runner",
                wheel_dof_names=["front_left_joint","front_right_joint", "back_left_joint","back_right_joint"],
                create_robot=True,
                usd_path=STORM_SIGHT_USD_PATH,
                position=np.array([0, 0.0, 0.1]),
            )
        )

        camera = Camera(
            prim_path="/World/Storm_Sight_Runner/storm_base/storm_sensors/intel_realsense_camera/back_camera",
            # position=np.array([0,-0.18433,0.15774]),
            # frequency=20,
            resolution=(256,256),
            # orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
        )

        camera.initialize()
        simulation_app.update()
        camera.initialize()
       
        self.wheel_vel_sub = self.create_subscription(Float64MultiArray, 'wheel_velocities', self.wheel_velocity_callback, 10)

        camera_freq = 30
        namespace = "/back_camera"

        self.publish_camera_info(camera, namespace, camera_freq)
        self.publish_rgb(camera, namespace, camera_freq)
        self.publish_depth(camera, namespace, camera_freq)

        self.ros_world.reset()

    def wheel_velocity_callback(self, msg):
        if self.ros_world.is_playing():
            if len(msg.data) == 4:
                wheel_velocities = np.array(msg.data)
                action = ArticulationAction(joint_velocities = wheel_velocities)
                self.storm_runner.apply_wheel_actions(action)
                print(f"Received wheel velocities: {wheel_velocities}")
            else:
                print(f"Warning: Received {len(msg.data)} values, need 4")

    
    def publish_camera_info(self, camera: Camera, namespace, freq):
        from isaacsim.ros2.bridge import read_camera_info

        render_product = camera._render_product_path
        step_size = int(60/freq)
        topic_name = camera.name+"_camera_info"
        queue_size = 1
        node_namespace = namespace
        frame_id = camera.prim_path.split("/")[-1] 

        writer = rep.writers.get("ROS2PublishCameraInfo")
        camera_info = read_camera_info(render_product_path=render_product)
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name,
            width=camera_info["width"],
            height=camera_info["height"],
            projectionType=camera_info["projectionType"],
            k=camera_info["k"].reshape([1, 9]),
            r=camera_info["r"].reshape([1, 9]),
            p=camera_info["p"].reshape([1, 12]),
            physicalDistortionModel=camera_info["physicalDistortionModel"],
            physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
        )
        writer.attach([render_product])

        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            "PostProcessDispatch" + "IsaacSimulationGate", render_product
        )

        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
        return

    def publish_rgb(self, camera: Camera, namespace, freq):
        render_product = camera._render_product_path
        step_size = int(60/freq)
        topic_name = camera.name+"_rgb"
        queue_size = 1
        node_namespace = namespace
        frame_id = camera.prim_path.split("/")[-1]

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        writer = rep.writers.get(rv + "ROS2PublishImage")
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name
        )
        writer.attach([render_product])

        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            rv + "IsaacSimulationGate", render_product
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

        return

    def publish_depth(self, camera: Camera, namespace, freq):
        render_product = camera._render_product_path
        step_size = int(60/freq)
        topic_name = camera.name+"_depth"
        queue_size = 1
        node_namespace = namespace
        frame_id = camera.prim_path.split("/")[-1]

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                                sd.SensorType.DistanceToImagePlane.name
                            )
        writer = rep.writers.get(rv + "ROS2PublishImage")
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name
        )
        writer.attach([render_product])

        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            rv + "IsaacSimulationGate", render_product
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

        return


    def run_simulation(self):
        self.timeline.play()
        reset_needed = False
        while simulation_app.is_running():
            self.ros_world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.ros_world.is_stopped() and not reset_needed:
                reset_needed = True
            if self.ros_world.is_playing():
                if reset_needed:
                    self.ros_world.reset()
                    reset_needed = False

        # Cleanup
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()

if __name__ == "__main__":
    rclpy.init()
    storm_runner_sim_node = StormRunnerSim()
    storm_runner_sim_node.run_simulation()

