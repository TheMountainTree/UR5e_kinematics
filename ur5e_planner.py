#!/usr/bin/env python3
"""
UR5e Advanced Trajectory Planner with Robotiq 2F-85 Gripper
-----------------------------------------------------------
"""

import numpy as np
import time
import sys

# Try imports
try:
    from ur5e_robot import UR5eRobot
    from ur5e_visualization import UR5eVisualizer
except ImportError as e:
    print(f"Error: Missing dependencies. {e}")
    sys.exit(1)

# ==========================================
# 1. Kinematics Controller (Hybrid IK)
# ==========================================
class RobotController:
    """
    Interface between Planner (Cartesian) and Robot (Joints).
    """
    def __init__(self, robot: UR5eRobot):
        self.robot = robot
        self.dof = robot.dof
        self.lower, self.upper = robot.get_arm_joint_limits()
        
        # Initial pose
        self.current_q = np.array([0, -1.57, 1.57, -1.57, -1.57, 0], dtype=np.float64)
        # Apply initial pose
        self.robot.forward_kinematics(self.current_q)

    def solve_ik_tracking(self, target_pos):
        q_sol, success, _ = self.robot.inverse_kinematics(
            target_position=target_pos,
            initial_guess=self.current_q,
            max_iterations=15,
            tolerance=1e-4
        )
        if success or q_sol is not None:
            return q_sol
        return self.current_q

    def move_to(self, q):
        self.current_q = q
        self.robot.set_qpos(q)
        
    def get_ee_pos(self):
        pos, _ = self.robot.forward_kinematics(self.current_q)
        return pos
    
    def set_gripper(self, val):
        self.robot.set_gripper(val)


# ==========================================
# 2. Finite State Machine
# ==========================================
class StateMachine:
    def __init__(self):
        self.states = {}
        self.current_state = None
        self.running = True

    def add_state(self, name, handler):
        self.states[name] = handler

    def set_state(self, name):
        if name in self.states:
            self.current_state = name
        else:
            print(f"[FSM] Error: State {name} unknown")

    def run(self):
        if self.current_state and self.running:
            self.states[self.current_state]()


# ==========================================
# 3. Trajectory Planner App
# ==========================================
class UR5ePlannerApp:
    def __init__(self):
        # 1. Initialize Visualizer first
        self.viz = UR5eVisualizer()
        self.viz.setup_viewer(camera_xyz=(1.5, 0, 1.0), camera_rpy=(0, -0.5, 3.14))
        self.viz.enable_trails(True)

        # 2. Initialize Robot in the visualizer's scene
        try:
            self.robot = UR5eRobot(scene=self.viz.scene, with_gripper=True)
        except Exception as e:
            print(f"Failed to load robot: {e}")
            sys.exit(1)
            
        self.controller = RobotController(self.robot)
        
        # 3. Define Waypoints
        self.waypoints = {
            'home': np.array([0.3, 0.0, 0.4]),
            'pos_a': np.array([0.4, -0.2, 0.3]),
            'pos_b': np.array([0.4, 0.2, 0.3]),
            'center_a': np.array([0.4, -0.1, 0.3]),
            'center_b': np.array([0.4, 0.1, 0.3])
        }
        self.radius = 0.1
        
        # 4. Setup FSM
        self.fsm = StateMachine()
        self._setup_fsm()
        
        self.target_fps = 60
        self.dt = 1.0 / self.target_fps

    def _setup_fsm(self):
        self.fsm.add_state("INIT", self.state_init)
        
        self.fsm.add_state("MOVE_HOME", lambda: self.exec_line(self.waypoints['home'], 4.0, "MOVE_A"))
        self.fsm.add_state("MOVE_A", lambda: self.exec_line(self.waypoints['pos_a'], 4.0, "CIRCLE_A"))
        self.fsm.add_state("CIRCLE_A", lambda: self.exec_circle(self.waypoints['center_a'], self.radius, 10.0, "MOVE_B"))
        self.fsm.add_state("MOVE_B", lambda: self.exec_line(self.waypoints['pos_b'], 4.0, "CIRCLE_B"))
        self.fsm.add_state("CIRCLE_B", lambda: self.exec_circle(self.waypoints['center_b'], self.radius, 10.0, "RETURN_HOME", start_angle=np.pi/2))
        self.fsm.add_state("RETURN_HOME", lambda: self.exec_line(self.waypoints['home'], 4.0, "GRIPPER_TEST"))
        
        # New State: Gripper Test
        self.fsm.add_state("GRIPPER_TEST", self.state_gripper_test)
        
        self.fsm.add_state("DONE", self.state_done)
        self.fsm.set_state("INIT")

    # --- Actions ---

    def state_init(self):
        print("Initializing...")
        time.sleep(1.0)
        self.fsm.set_state("MOVE_HOME")

    def state_gripper_test(self):
        print("Starting Gripper Test (10x Open/Close)...")
        
        count = 10
        duration_per_cycle = 1.0 # 0.5s open, 0.5s close
        
        steps = int(duration_per_cycle * self.target_fps)
        
        for c in range(count):
            print(f"Gripper Cycle {c+1}/{count}")
            
            # Close
            for i in range(steps // 2):
                val = (i / (steps//2)) * 0.8 # 0 to 0.8
                self.controller.set_gripper(val)
                self.viz.render()
                time.sleep(self.dt)
            
            # Open
            for i in range(steps // 2):
                val = 0.8 - (i / (steps//2)) * 0.8 # 0.8 to 0
                self.controller.set_gripper(val)
                self.viz.render()
                time.sleep(self.dt)
                
            if self.viz.viewer and self.viz.viewer.closed:
                self.fsm.running = False
                return
                
        self.fsm.set_state("DONE")

    def state_done(self):
        print("Trajectory completed. Close viewer to exit.")
        while self.viz.viewer and not self.viz.viewer.closed:
            self.viz.render()
            self.viz.scene.step()
            time.sleep(0.01)
        self.fsm.running = False

    def exec_line(self, target, duration, next_state):
        start_pos = self.controller.get_ee_pos()
        steps = int(duration * self.target_fps)
        
        print(f"Moving to {np.round(target, 2)} ({duration}s)")
        
        for i in range(steps):
            t0 = time.perf_counter()
            t = (i + 1) / steps
            alpha = t * t * t * (10 - 15 * t + 6 * t * t)
            des_pos = (1 - alpha) * start_pos + alpha * target
            
            self._update_robot(des_pos)
            
            elapsed = time.perf_counter() - t0
            time.sleep(max(0, self.dt - elapsed))
            
            if self.viz.viewer and self.viz.viewer.closed:
                self.fsm.running = False
                return

        self.fsm.set_state(next_state)

    def exec_circle(self, center, radius, duration, next_state, start_angle=None):
        print(f"Drawing Circle at {np.round(center, 2)} ({duration}s)")
        
        curr_pos = self.controller.get_ee_pos()
        rel = curr_pos - center
        
        if start_angle is None:
            start_angle = np.arctan2(rel[1], rel[0])
            
        steps = int(duration * self.target_fps)
        
        for i in range(steps + 1):
            t0 = time.perf_counter()
            alpha = i / steps
            angle = start_angle + 2 * np.pi * alpha
            
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = center[2] 
            
            des_pos = np.array([x, y, z])
            
            self._update_robot(des_pos)
            
            elapsed = time.perf_counter() - t0
            time.sleep(max(0, self.dt - elapsed))
            
            if self.viz.viewer and self.viz.viewer.closed:
                self.fsm.running = False
                return

        self.fsm.set_state(next_state)

    def _update_robot(self, desired_pos):
        q = self.controller.solve_ik_tracking(desired_pos)
        self.controller.move_to(q)
        self.viz.add_trail_point(self.controller.get_ee_pos())
        self.viz.render()

    def run(self):
        print("Starting UR5e Trajectory Planner with Robotiq Gripper...")
        while self.fsm.running:
            self.fsm.run()

if __name__ == "__main__":
    app = UR5ePlannerApp()
    app.run()
            
