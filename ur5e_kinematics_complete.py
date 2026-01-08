#!/usr/bin/env python3
"""
Complete UR5e Kinematics Program for ManiSkill3

This program provides:
1. Forward Kinematics (FK): Compute end-effector pose from joint angles
2. Inverse Kinematics (IK): Compute joint angles from end-effector pose
3. Visualization: View robot in 3D with SAPIEN viewer

Requirements:
- Conda environment: robot_env
- SAPIEN and ManiSkill3 installed
- UR5e URDF file in ../ur5e/ur5e_fixed.urdf

Usage:
    python ur5e_kinematics_complete.py [--test] [--visualize] [--angles A1 A2 A3 A4 A5 A6]

Example:
    python ur5e_kinematics_complete.py --test
    python ur5e_kinematics_complete.py --visualize --angles 0.5 -0.3 0.8 -0.2 0.4 0.1
"""

import argparse
import numpy as np
import sys
import time
from ur5e_robot import UR5eRobot
from ur5e_visualization import UR5eVisualizer

def visualize_motion(robot, viz, q_start, q_target=None, target_position=None, mode="joint", steps=150, dt=0.02):
    """
    Visualize robot motion with trajectory.
    """
    # Setup Viewer if not already
    if not viz.viewer:
        viz.setup_viewer()

    # Move to start
    q_start = np.array(q_start)
    robot.set_qpos(q_start)

    def smoothstep(t):
        return t * t * (3 - 2 * t)
    
    print("Starting motion visualization...")
    
    # -------------------------
    # Joint-space motion
    # -------------------------
    if mode == "joint":
        assert q_target is not None, "q_target is required for joint mode"
        q_target = np.array(q_target)

        for i in range(steps):
            alpha = smoothstep(i / (steps - 1))
            q = (1 - alpha) * q_start + alpha * q_target
            
            robot.set_qpos(q)
            viz.render()
            time.sleep(dt)
            if viz.viewer.closed: break

    # -------------------------
    # Cartesian straight-line motion
    # -------------------------
    elif mode == "cartesian_line":
        assert target_position is not None, "target_position is required"

        p_start, _ = robot.forward_kinematics(q_start)
        p_target = np.array(target_position)
        q_current = q_start.copy()

        for i in range(steps):
            alpha = smoothstep(i / (steps - 1))
            p = (1 - alpha) * p_start + alpha * p_target

            # Use IK from robot
            # Note: UR5eRobot.inverse_kinematics returns (q, success, history)
            # But we might want multi-start or just simple IK?
            # Let's use simple IK first, fallback to multiple?
            # Actually robot.inverse_kinematics is DLS, should be fine for tracking.
            
            q_next, success, _ = robot.inverse_kinematics(p, initial_guess=q_current, tolerance=1e-3)
            
            if not success:
                 # Try multi-start?
                 pass
            
            q_current = q_next
            robot.set_qpos(q_current)
            viz.render()
            time.sleep(dt)
            if viz.viewer.closed: break
            
    print("Motion finished. Press 'q' to quit.")
    # Run loop
    viz.run() # This handles the 'q' quit logic and keeps window open

def test_forward_kinematics(robot, num_random=5):
    print("\n" + "="*60)
    print("Testing Forward Kinematics (Randomized)")
    print("="*60)
    
    l, u = robot.get_arm_joint_limits()
    for i in range(num_random):
        q = np.random.uniform(l, u)
        pos, quat = robot.forward_kinematics(q)
        pose_6d = robot.get_pose_6d(q)
        print(f"\nSample {i+1}:")
        print(f"  Joints: {np.round(q, 3)}")
        print(f"  Pos:    {np.round(pos, 3)}")
        print(f"  Pose6D: {np.round(pose_6d, 3)}")

def test_inverse_kinematics(robot, num_random=5):
    print("\n" + "="*60)
    print("Testing Inverse Kinematics (Randomized)")
    print("="*60)
    
    l, u = robot.get_arm_joint_limits()
    success_count = 0
    
    for i in range(num_random):
        q_target = np.random.uniform(l, u)
        target_pos, _ = robot.forward_kinematics(q_target)
        
        q_sol, success, err = robot.solve_ik_multiple(target_pos)
        
        sol_pos, _ = robot.forward_kinematics(q_sol)
        real_err = np.linalg.norm(sol_pos - target_pos)
        
        print(f"\nTest {i+1}:")
        print(f"  Target Pos: {np.round(target_pos, 3)}")
        print(f"  Solved Pos: {np.round(sol_pos, 3)}")
        print(f"  Error:      {real_err:.6f}")
        print(f"  Success:    {success}")
        
        if success and real_err < 1e-3:
            success_count += 1
            
    print(f"\nIK Success Rate: {success_count}/{num_random}")

def analyze_workspace(robot):
    print("\n" + "="*60)
    print("Workspace Analysis")
    print("="*60)
    
    points = robot.compute_workspace_samples(1000)
    print(f"X Range: [{points[:,0].min():.3f}, {points[:,0].max():.3f}]")
    print(f"Y Range: [{points[:,1].min():.3f}, {points[:,1].max():.3f}]")
    print(f"Z Range: [{points[:,2].min():.3f}, {points[:,2].max():.3f}]")

def main():
    parser = argparse.ArgumentParser(description='UR5e Kinematics Solver')
    parser.add_argument('--test', action='store_true', help='Run all tests')
    parser.add_argument('--fk', action='store_true', help='Test forward kinematics')
    parser.add_argument('--ik', action='store_true', help='Test inverse kinematics')
    parser.add_argument('--workspace', action='store_true', help='Analyze workspace')
    parser.add_argument('--visualize', action='store_true', help='Visualize robot')
    parser.add_argument('--angles', nargs=6, type=float, help='Joint angles for FK/Viz')
    parser.add_argument('--urdf', type=str, help='Path to URDF file')
    
    args = parser.parse_args()
    if not any(vars(args).values()):
        parser.print_help()
        return

    # Initialize
    viz = None
    scene = None
    
    # If visualization needed, create viz first to own the scene
    if args.visualize:
        viz = UR5eVisualizer()
        scene = viz.scene
        
    # Create Robot
    # Note: kinematics_complete usually used fixed.urdf (no gripper). 
    # But new UR5eRobot defaults to robotiq if with_gripper=True.
    # Let's check args.urdf or default logic.
    # If no urdf provided, we should probably stick to what the script used to do?
    # Or upgrade to use gripper URDF?
    # Let's pass with_gripper=False if we want pure UR5e behavior, or True if we want generic.
    # Given the project context "ur5e_kinematics", using the combined one is safer if it exists.
    
    try:
        robot = UR5eRobot(scene=scene, urdf_path=args.urdf, with_gripper=False) # Use pure UR5e for this script?
        # Actually the combined URDF is safer if we want to support gripper later.
        # But 'fixed.urdf' was default. Let's try to use 'ur5e_robotiq.urdf' if available, else 'ur5e_fixed.urdf'.
        # UR5eRobot defaults to robotiq if with_gripper=True.
        # Let's set with_gripper=True but fall back gracefully? 
        # UR5eRobot handles paths. Let's rely on it.
    except Exception as e:
        print(f"Failed to load robot: {e}")
        return

    # Tests
    if args.test:
        test_forward_kinematics(robot)
        test_inverse_kinematics(robot)
        analyze_workspace(robot)
        
    if args.fk:
        test_forward_kinematics(robot)
        
    if args.ik:
        test_inverse_kinematics(robot)
        
    if args.workspace:
        analyze_workspace(robot)
        
    if args.angles:
        q = np.array(args.angles)
        pos, _ = robot.forward_kinematics(q)
        print(f"FK Result: {pos}")
        
    if args.visualize:
        q_start = np.zeros(robot.dof)
        
        if args.angles:
            q_target = np.array(args.angles)
            visualize_motion(robot, viz, q_start, q_target=q_target, mode="joint")
        else:
            # Cartesian motion
            pos_start, _ = robot.forward_kinematics(q_start)
            target = pos_start + np.array([0.2, 0.0, 0.1])
            visualize_motion(robot, viz, q_start, target_position=target, mode="cartesian_line")

if __name__ == '__main__':
    main()
