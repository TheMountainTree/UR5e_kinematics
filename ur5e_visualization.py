import sapien.core as sapien
from sapien.utils import Viewer
import numpy as np
import time
from collections import deque
import os

class UR5eVisualizer:
    def __init__(self, scene=None, timestep=1/60.0):
        self.scene = scene
        self.viewer = None
        self.robot = None
        self.loader = None
        
        # Trail settings
        self.trail_actors = deque()
        self.max_trail_points = 300
        self.trail_enabled = False
        self.trail_material = None
        self.last_trail_pos = None
        self.trail_min_dist = 0.005
        
        if self.scene is None:
            self._init_scene(timestep)
        else:
            # If scene provided, assume lights are set or user handles it?
            # Or we can add lights if we want consistency. 
            # Let's verify if ground exists? Hard to check.
            pass

    def _init_scene(self, timestep):
        """Initialize SAPIEN scene with lights and ground."""
        self.scene = sapien.Scene()
        self.scene.set_timestep(timestep)
        self.scene.add_ground(0)
        
        # Lighting
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, 1, -1], [1, 1, 1])
        self.scene.add_directional_light([1, -1, -1], [1, 1, 1]) # Added secondary light

    def load_robot(self, urdf_path, fix_root_link=True):
        """Load the robot from URDF."""
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF not found: {urdf_path}")
            
        self.loader = self.scene.create_urdf_loader()
        self.loader.fix_root_link = fix_root_link
        
        print(f"Loading robot from: {urdf_path}")
        self.robot = self.loader.load(urdf_path)
        
        if not self.robot:
            raise RuntimeError("Failed to load robot.")
            
        return self.robot

    def setup_viewer(self, camera_xyz=(1.5, 0, 1.0), camera_rpy=(0, -0.5, 3.14)):
        """Initialize the SAPIEN viewer."""
        try:
            self.viewer = Viewer()
            self.viewer.set_scene(self.scene)
            self.viewer.set_camera_xyz(*camera_xyz)
            self.viewer.set_camera_rpy(*camera_rpy)
            try:
                self.viewer.toggle_axes(False)
            except AttributeError:
                pass # toggle_axes might not exist in some versions
            print("Viewer initialized.")
            
            # Init material for trails if renderer is available
            self._init_trail_material()
            
        except Exception as e:
            print(f"Warning: Viewer initialization failed: {e}")
            self.viewer = None

    def _init_trail_material(self):
        try:
            renderer = sapien.SapienRenderer()
            self.trail_material = renderer.create_material()
            self.trail_material.base_color = [1.0, 0.0, 0.0, 1.0]
            self.trail_material.roughness = 0.5
            self.trail_material.metallic = 0.0
        except Exception as e:
            print(f"Warning: Could not create custom material: {e}")
    
    def enable_trails(self, enabled=True, max_points=300):
        self.trail_enabled = enabled
        self.max_trail_points = max_points
        
    def add_trail_point(self, position):
        if not self.trail_enabled or self.scene is None:
            return

        if self.last_trail_pos is not None and np.linalg.norm(np.array(position) - self.last_trail_pos) < self.trail_min_dist:
            return
        self.last_trail_pos = np.array(position)

        pose = sapien.Pose(p=position)

        if len(self.trail_actors) >= self.max_trail_points:
            actor = self.trail_actors.popleft()
            actor.set_pose(pose)
            self.trail_actors.append(actor)
        else:
            builder = self.scene.create_actor_builder()
            if self.trail_material:
                builder.add_sphere_visual(radius=0.004, material=self.trail_material)
            else:
                # Fallback: add visual without color if material creation failed
                # Some SAPIEN versions do not support 'color' kwarg without material
                builder.add_sphere_visual(radius=0.004)

            actor = builder.build_static(name="trail_point")
            actor.set_pose(pose)
            self.trail_actors.append(actor)

    def render(self):
        """Update scene and render."""
        self.scene.update_render()
        if self.viewer and not self.viewer.closed:
            self.viewer.render()

    def run(self, step_callback=None):
        """
        Run the main visualization loop.
        
        Args:
            step_callback: Function to call every frame. Should accept (visualizer_instance, step_count)
        """
        if not self.viewer:
            print("Viewer not initialized. Call setup_viewer() first.")
            return

        print("Press 'q' to quit visualization, 'Space' to toggle simulation.")
        step = 0
        while not self.viewer.closed:
            self.scene.step()
            
            if step_callback:
                step_callback(self, step)
            
            self.render()
            step += 1

    def close(self):
        if self.viewer:
            self.viewer.close()
