import numpy as np
import os
import sapien.core as sapien
from pathlib import Path

class UR5eRobot:
    """
    Unified class for UR5e Robot logic: Kinematics, Joint management, and Gripper state.
    Serves as the Model in MVC.
    """
    def __init__(self, scene=None, urdf_path=None, with_gripper=True):
        """
        Initialize the robot.
        
        Args:
            scene: SAPIEN scene. If None, creates a headless scene for computation.
            urdf_path: Path to URDF. If None, defaults to internal paths.
            with_gripper: Whether to expect/handle a gripper.
        """
        self.with_gripper = with_gripper
        
        # 1. Setup Scene
        if scene is None:
            self.scene = sapien.Scene()
            self.scene.set_timestep(1/60.0)
            self.scene.add_ground(0)
            # Add basic lights for internal scene just in case
            self.scene.set_ambient_light([0.5, 0.5, 0.5])
            self.scene.add_directional_light([1, -1, -1], [1, 1, 1])
            self.owns_scene = True
        else:
            self.scene = scene
            self.owns_scene = False

        # 2. Determine URDF Path
        if urdf_path is None:
            workspace_dir = Path(__file__).parent
            if with_gripper:
                self.urdf_path = workspace_dir / "ur5e_robotiq.urdf"
            else:
                self.urdf_path = workspace_dir.parent / "ur5e" / "ur5e_fixed.urdf"
        else:
            self.urdf_path = Path(urdf_path)
            
        if not self.urdf_path.exists():
             # Fallback if with_gripper is true but combined URDF missing?
             # For now, just raise error. User must generate it.
             raise FileNotFoundError(f"URDF file not found: {self.urdf_path}")

        # 3. Load Robot
        loader = self.scene.create_urdf_loader()
        loader.fix_root_link = True
        print(f"Loading robot from: {self.urdf_path}")
        self.robot = loader.load(str(self.urdf_path))
        if not self.robot:
            raise RuntimeError("Failed to load robot.")

        # 4. Initialize Joints (Arm)
        self.arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        
        self.all_active_joints = self.robot.get_active_joints()
        self.arm_joints = []
        self.arm_indices = []
        
        # Map arm names to indices
        for name in self.arm_joint_names:
            found = False
            for i, joint in enumerate(self.all_active_joints):
                if joint.name == name:
                    self.arm_joints.append(joint)
                    self.arm_indices.append(i)
                    found = True
                    break
            if not found:
                print(f"Warning: Arm joint {name} not found in model.")

        self.dof = len(self.arm_joints) # Should be 6 for UR5e

        # 5. Initialize Gripper (if applicable)
        self.gripper_mimic_map = {}
        self.gripper_indices = {}
        self.gripper_val = 0.0
        
        if self.with_gripper:
            # Map for Robotiq 2F-85
            self.gripper_mimic_map = {
                "finger_joint": 1.0,
                "left_inner_knuckle_joint": 1.0,
                "left_inner_finger_joint": -1.0,
                "right_outer_knuckle_joint": 1.0,
                "right_inner_knuckle_joint": 1.0,
                "right_inner_finger_joint": -1.0
            }
            for name, mult in self.gripper_mimic_map.items():
                for i, joint in enumerate(self.all_active_joints):
                    if joint.name == name:
                        self.gripper_indices[i] = mult

        # 6. End Effector
        # If gripper, use 'tool0' or 'robotiq_arg2f_base_link'? 
        # Usually 'tool0' is the attachment point.
        self.ee_link = self.robot.get_links()[-1] # Default last link
        for link in self.robot.get_links():
            if link.name == "tool0":
                self.ee_link = link
                break
        print(f"End-effector link: {self.ee_link.name}")

    # --- Kinematics ---

    def forward_kinematics(self, joint_angles):
        """
        Compute FK. Updates robot state.
        Args:
            joint_angles: 6-element array for arm joints.
        Returns:
            (position, quaternion)
        """
        self.set_qpos(joint_angles)
        
        pose = self.ee_link.pose
        return np.array(pose.p), np.array(pose.q)

    def inverse_kinematics(self, target_position, initial_guess=None, max_iterations=100, tolerance=1e-4):
        """
        Compute IK using Damped Least Squares.
        """
        if initial_guess is None:
            q = np.zeros(self.dof)
        else:
            q = np.array(initial_guess)
            
        target_pos = np.array(target_position)
        error_history = []

        for i in range(max_iterations):
            curr_pos, _ = self.forward_kinematics(q)
            error = target_pos - curr_pos
            error_norm = np.linalg.norm(error)
            error_history.append(error_norm)
            
            if error_norm < tolerance:
                return q, True, error_history

            # Jacobian
            J = self._compute_jacobian(q)
            
            # DLS
            damping = np.clip(0.1 * error_norm, 1e-4, 0.1)
            JJt = J @ J.T
            damping_matrix = damping**2 * np.eye(3)
            dq = J.T @ np.linalg.solve(JJt + damping_matrix, error)
            
            # Limit step size
            dq_norm = np.linalg.norm(dq)
            if dq_norm > 0.2:
                dq = dq / dq_norm * 0.2
                
            q += dq
            
            # Limits
            lower, upper = self.get_arm_joint_limits()
            q = np.clip(q, lower, upper)
            
        return q, False, error_history
    
    def solve_ik_multiple(self, target_position, n=10, tolerance=1e-4):
        """Multi-start IK."""
        lower, upper = self.get_arm_joint_limits()
        best_q = None
        best_err = np.inf
        
        for _ in range(n):
            q_init = np.random.uniform(lower, upper)
            q, success, _ = self.inverse_kinematics(target_position, q_init, 100, tolerance)
            pos, _ = self.forward_kinematics(q)
            err = np.linalg.norm(pos - target_position)
            
            if err < best_err:
                best_err = err
                best_q = q
            if success:
                return best_q, True, best_err
                
        return best_q, False, best_err

    def _compute_jacobian(self, q, delta=1e-6):
        J = np.zeros((3, self.dof))
        pos_base, _ = self.forward_kinematics(q)
        
        for i in range(self.dof):
            q_p = q.copy()
            q_p[i] += delta
            pos_p, _ = self.forward_kinematics(q_p)
            J[:, i] = (pos_p - pos_base) / delta
        return J

    # --- Robot State & Control ---

    def set_qpos(self, arm_q):
        """Sets the robot's qpos, handling arm and gripper."""
        full_q = self.robot.get_qpos()
        
        # Set Arm
        if len(arm_q) != self.dof:
             # Should warn, but for now just assume it matches
             pass
             
        for i, idx in enumerate(self.arm_indices):
            if i < len(arm_q):
                full_q[idx] = arm_q[i]
                
        # Set Gripper (Maintain current value)
        if self.with_gripper:
            for idx, mult in self.gripper_indices.items():
                full_q[idx] = self.gripper_val * mult
        
        self.robot.set_qpos(full_q)

    def set_gripper(self, val):
        """
        Set gripper state.
        val: 0.0 (Open) to 0.8 (Closed)
        """
        self.gripper_val = val
        # Re-apply current qpos to update gripper
        # We need current arm q to preserve it.
        # But reading self.robot.get_qpos() gives us current state.
        
        full_q = self.robot.get_qpos()
        for idx, mult in self.gripper_indices.items():
            full_q[idx] = self.gripper_val * mult
        self.robot.set_qpos(full_q)

    def get_arm_joint_limits(self):
        lower = []
        upper = []
        for joint in self.arm_joints:
            limits = joint.get_limits()
            if limits is None:
                lower.append(-2*np.pi)
                upper.append(2*np.pi)
            else:
                lower.append(limits[0,0])
                upper.append(limits[0,1])
        return np.array(lower), np.array(upper)

    def get_pose_6d(self, q):
        pos, quat = self.forward_kinematics(q)
        # Convert quat to euler
        # ... reusing the logic from previous solver
        w, x, y, z = quat
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(np.clip(sinp, -1, 1))
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.array([pos[0], pos[1], pos[2], roll, pitch, yaw])

    def compute_workspace_samples(self, n=1000):
        points = []
        l, u = self.get_arm_joint_limits()
        for _ in range(n):
            q = np.random.uniform(l, u)
            pos, _ = self.forward_kinematics(q)
            points.append(pos)
        return np.array(points)
