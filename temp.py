from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.asset.importer.urdf import _urdf

import omni.kit.app
kit = omni.kit.app.get_app()
kit.get_extension_manager().set_extension_enabled("omni.importer.urdf", True)
kit.get_extension_manager().set_extension_enabled("omni.usd", True)

# Wait for extension to load fully (optional but helpful)
kit.update()

# import omni.kit.commands
# import omni.usd

import sys
import carb
import numpy as np
import os
import xacro

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path



class IsaacSimSimulation:
    def __init__(self, simulation_app):
        """Initialize the IsaacSimSimulation class."""
        
        self.simulation_app = simulation_app
        self.my_world = None
        self.arm = None
        self.car = None
        self.assets_root_path = None

    def prepare_scene(self):
        """Prepare the simulation scene, including assets and camera view."""

        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            self.simulation_app.close()
            sys.exit()

        # Initialize the world
        self.my_world = World(stage_units_in_meters=1.0)
        self.my_world.scene.add_default_ground_plane()  # add ground plane

        # Set camera view
        set_camera_view(
            eye=[5.0, 0.0, 1.5], target=[0.00, 0.00, 1.00], camera_prim_path="/OmniverseKit_Persp"
        )

        # # Add Franka robot
        # franka_asset_path = self.assets_root_path + "/Isaac/Robots/Franka/franka.usd"
        # add_reference_to_stage(usd_path=franka_asset_path, prim_path="/World/Arm")
        # self.arm = Articulation(prim_paths_expr="/World/Arm", name="my_arm")

        # # Add Carter robot
        # car_asset_path = self.assets_root_path + "/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd"
        # add_reference_to_stage(usd_path=car_asset_path, prim_path="/World/Car")
        # self.car = Articulation(prim_paths_expr="/World/Car", name="my_car")

        # Set initial poses to prevent collision
        # self.arm.set_world_poses(positions=np.array([[0.0, 1.0, 0.0]]) / get_stage_units())
        # self.car.set_world_poses(positions=np.array([[0.0, -1.0, 0.0]]) / get_stage_units())

        # Reset the world
        self.my_world.reset()

    def run_simulation(self):
        """Run the simulation."""
        try:
            while self.simulation_app.is_running():
                # Move the arm and the car
                # self.arm.set_joint_positions([[-1.5, 0.0, 0.0, -1.5, 0.0, 1.5, 0.5, 0.04, 0.04]])
                # self.car.set_joint_velocities([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])

                # Step the simulation
                self.my_world.step(render=True)
        except KeyboardInterrupt:
            print("Simulation interrupted by user")
        finally:
            self.close_simulation()

    def close_simulation(self):
        """Close the simulation application."""
        print("Closing simulation")
        self.simulation_app.close()

    def import_robot_from_xacro(self, xacro_file_path, prim_path, robot_name, xacro_params=None):
        """
        Import a robot from a XACRO file without creating intermediate files.
        
        Args:
            xacro_file_path (str): Path to the XACRO file
            prim_path (str): Path where to create the robot prim
            robot_name (str): Name for the robot articulation
            xacro_params (dict, optional): Parameters to pass to xacro processor
            
        Returns:
            Articulation: The imported robot as an Articulation object
        """
        if xacro_params is None:
            xacro_params = {}
            
        try:
            # Process the XACRO file to get URDF content as string
            carb.log_info(f"Processing XACRO file: {xacro_file_path}")
            urdf_str = xacro.process_file(xacro_file_path, mappings=xacro_params).toxml()
            
            # Acquire the URDF extension interface
            urdf_interface = _urdf.acquire_urdf_interface()
            
            # Configure import settings
            import_config = _urdf.ImportConfig()
            import_config.convex_decomp = False
            import_config.fix_base = True
            import_config.make_default_prim = True
            import_config.self_collision = False
            import_config.distance_scale = 1.0
            import_config.density = 0.0
            
            # Parse the URDF string
            result, robot_model = omni.kit.commands.execute(
                "URDFParseString",
                urdf_string=urdf_str,
                import_config=import_config
            )
            
            if not result:
                carb.log_error(f"Failed to parse URDF from XACRO: {xacro_file_path}")
                return None
                
            # Update joint drives for better control
            for joint in robot_model.joints:
                robot_model.joints[joint].drive.strength = 1047.19751  # High stiffness
                robot_model.joints[joint].drive.damping = 52.35988     # Moderate damping
            
            # Import the robot onto the stage
            result, imported_prim_path = omni.kit.commands.execute(
                "URDFImportRobot",
                urdf_robot=robot_model,
                import_config=import_config,
                prim_path=prim_path
            )
            
            if not result:
                carb.log_error(f"Failed to import robot to stage: {prim_path}")
                return None
            
            # Create an articulation object from the imported robot
            robot = Articulation(prim_paths_expr=imported_prim_path, name=robot_name)
            
            carb.log_info(f"Successfully imported robot '{robot_name}' from XACRO to {imported_prim_path}")
            return robot
            
        except Exception as e:
            carb.log_error(f"Error importing robot from XACRO: {str(e)}")
            return None

      

if __name__ == "__main__":
    # Create a simulation object and pass the already initialized app
    simulation = IsaacSimSimulation(simulation_app)
    simulation.prepare_scene()

    xacro_file_path = "/home/biorobotics/Documents/varun/arpa/pipe_issac_ws/src/pipe_issac/pipe_description/urdf/pipesprite.xacro"  
        
    # Check if the file exists before attempting to import
    try:
        import os
        if os.path.exists(xacro_file_path):
            # Import the robot with custom parameters
            simulation.xacro_robot = simulation.import_robot_from_xacro(
                xacro_file_path,
                "/World/XacroRobot", 
                "custom_robot",
                {"use_gazebo": "true", "robot_name": "my_custom_robot"}
            )
            
            # Position the imported robot if successful
            if simulation.xacro_robot:
                simulation.xacro_robot.set_world_poses(positions=np.array([[2.0, 0.0, 0.0]]) / get_stage_units())
                carb.log_info("XACRO robot imported and positioned successfully")
            else:
                carb.log_error("Failed to import XACRO robot")
        else:
            carb.log_warning(f"XACRO file not found: {xacro_file_path}")
    except Exception as e:
        carb.log_error(f"Error setting up XACRO robot: {str(e)}")

    simulation.run_simulation()