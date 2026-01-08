import numpy as np
import time
from ur5e_robot import UR5eRobot
from ur5e_visualization import UR5eVisualizer

def main():
    # 1. Initialize Visualizer
    viz = UR5eVisualizer()
    viz.setup_viewer(camera_xyz=(1.0, 0, 0.5), camera_rpy=(0, -0.5, 3.14))

    # 2. Load Robot (with gripper)
    try:
        # Pass viz.scene so robot loads into the visualization scene
        robot = UR5eRobot(scene=viz.scene, with_gripper=True)
    except Exception as e:
        print(f"Error loading robot: {e}")
        return

    # 3. Move to Home Pose
    home_pose = [0, -1.57, 1.57, -1.57, -1.57, 0]
    robot.set_qpos(home_pose)

    # 4. Define Control Loop Callback
    def control_callback(visualizer, step):
        # Oscillate gripper
        t = time.time()
        # 0 to 0.8
        val = 0.4 + 0.4 * np.sin(t * 3)
        robot.set_gripper(val)

    print("Press 'Space' in viewer to toggle simulation if needed.")
    print("Opening and closing gripper...")
    
    # 5. Run Loop
    viz.run(step_callback=control_callback)

if __name__ == "__main__":
    main()
