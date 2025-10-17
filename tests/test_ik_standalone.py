#!/usr/bin/env python3
"""
Standalone IK test script (no pytest dependencies).
Tests Mink IK solver accuracy for reaching target poses.
"""

import numpy as np
import mujoco
import mink
from scipy.spatial.transform import Rotation
import sys


class IKTester:
    """Standalone IK tester for Panda robot."""
    
    def __init__(self, model_path):
        """Initialize robot model and IK configuration."""
        self.m = mujoco.MjModel.from_xml_path(model_path)
        self.d = mujoco.MjData(self.m)
        
        # Load home keyframe
        home_key_id = self.m.key("home").id
        mujoco.mj_resetDataKeyframe(self.m, self.d, home_key_id)
        
        # Create configuration
        self.conf = mink.Configuration(self.m)
        self.conf.update(self.d.qpos)
        
        # Get end-effector site
        self.ee_site_id = self.m.site("attachment_site").id
        
        # Create tasks
        self.ee_task = mink.tasks.FrameTask(
            frame_name="attachment_site",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        self.posture_task = mink.tasks.PostureTask(model=self.m, cost=1e-2)
        self.posture_task.set_target_from_configuration(self.conf)
        
        self.tasks = [self.ee_task, self.posture_task]
        
    def reset_to_home(self):
        """Reset configuration to home position."""
        home_key_id = self.m.key("home").id
        mujoco.mj_resetDataKeyframe(self.m, self.d, home_key_id)
        self.conf.update(self.d.qpos)
        
    def get_ee_pose(self):
        """Get current end-effector pose (returns position and rotation matrix)."""
        self.d.qpos[:] = self.conf.q
        mujoco.mj_forward(self.m, self.d)
        
        pos = self.d.site_xpos[self.ee_site_id].copy()
        rot_mat = self.d.site_xmat[self.ee_site_id].copy().reshape(3, 3)
        
        return pos, rot_mat
    
    def set_target_pose(self, target_pos, target_rot_mat):
        """Set IK target pose (using rotation matrix directly)."""
        transform = mink.SE3.from_rotation_and_translation(
            mink.SO3.from_matrix(target_rot_mat), target_pos
        )
        self.ee_task.set_target(transform)
    
    def solve_ik(self, max_iterations=200, dt=0.01, pos_tol=1e-3, ori_tol=1e-2, verbose=False):
        """
        Solve IK iteratively until convergence or max iterations.
        
        Returns:
            dict: Results including convergence status, iterations, errors
        """
        for i in range(max_iterations):
            # Solve one IK step (using daqp solver - the only one available)
            vel = mink.solve_ik(self.conf, self.tasks, dt, "daqp", 1e-3)
            
            # Integrate
            self.conf.integrate_inplace(vel, dt)
            
            # Check convergence
            pos, rot_mat = self.get_ee_pose()
            target_transform = self.ee_task.transform_target_to_world
            
            # Position error
            pos_error = np.linalg.norm(pos - target_transform.translation())
            
            # Orientation error (using rotation matrix)
            target_rot_mat = target_transform.rotation().as_matrix()
            # Compute rotation error using Frobenius norm of difference
            rot_diff = target_rot_mat @ rot_mat.T - np.eye(3)
            ori_error = np.linalg.norm(rot_diff)
            
            if verbose and (i % 50 == 0 or pos_error < pos_tol):
                print(f"    Iter {i}: pos_err={pos_error*1000:.4f}mm, ori_err={np.rad2deg(ori_error):.3f}deg")
            
            # Check convergence
            if pos_error < pos_tol and ori_error < ori_tol:
                return {
                    'converged': True,
                    'iterations': i + 1,
                    'pos_error': pos_error,
                    'ori_error': ori_error,
                    'final_pos': pos,
                    'final_rot_mat': rot_mat
                }
        
        return {
            'converged': False,
            'iterations': max_iterations,
            'pos_error': pos_error,
            'ori_error': ori_error,
            'final_pos': pos,
            'final_rot_mat': rot_mat
        }


def run_tests():
    """Run all IK tests."""
    model_path = "/home/ahmad.gazar/devel/mujoco-ros2-control/models/franka_panda/mjx_scene.xml"
    
    print("=" * 70)
    print("MINK IK SOLVER UNIT TESTS")
    print("=" * 70)
    print(f"Model: {model_path}")
    print()
    
    tester = IKTester(model_path)
    initial_pos, initial_rot_mat = tester.get_ee_pose()
    
    print(f"Home position: [{initial_pos[0]:.4f}, {initial_pos[1]:.4f}, {initial_pos[2]:.4f}]")
    print()
    
    tests = []
    total_tests = 0
    passed_tests = 0
    
    # Test 1: Maintain home position
    print("[1/7] Test: Maintain home position")
    tester.reset_to_home()
    pos, rot_mat = tester.get_ee_pose()
    tester.set_target_pose(pos, rot_mat)
    result = tester.solve_ik(max_iterations=10, pos_tol=1e-4, ori_tol=1e-3)
    total_tests += 1
    if result['converged'] and result['pos_error'] < 1e-4:
        print(f"  ✓ PASS: Converged in {result['iterations']} iters, pos_err={result['pos_error']*1000:.4f}mm")
        passed_tests += 1
    else:
        print(f"  ✗ FAIL: pos_err={result['pos_error']*1000:.2f}mm")
    print()
    
    # Test 2: Small diagonal translation (7.1 cm)
    print("[2/7] Test: Small diagonal translation (4cm forward, 5cm right, 3cm up)")
    tester.reset_to_home()
    pos, rot_mat = tester.get_ee_pose()
    target_pos = pos + np.array([0.04, 0.05, 0.03])
    tester.set_target_pose(target_pos, rot_mat)
    result = tester.solve_ik(max_iterations=200, pos_tol=1e-3, ori_tol=1e-2)
    total_tests += 1
    if result['converged']:
        print(f"  ✓ PASS: Converged in {result['iterations']} iters, pos_err={result['pos_error']*1000:.4f}mm")
        passed_tests += 1
    else:
        print(f"  ✗ FAIL: Did not converge, pos_err={result['pos_error']*1000:.2f}mm")
    print()
    
    # Test 3: Forward reach (10 cm)
    print("[3/7] Test: Forward reach (10 cm)")
    tester.reset_to_home()
    pos, rot_mat = tester.get_ee_pose()
    target_pos = pos + np.array([0.10, 0.0, 0.0])
    tester.set_target_pose(target_pos, rot_mat)
    result = tester.solve_ik(max_iterations=300, pos_tol=2e-3, ori_tol=1e-2)
    total_tests += 1
    if result['converged']:
        print(f"  ✓ PASS: Converged in {result['iterations']} iters, pos_err={result['pos_error']*1000:.4f}mm")
        passed_tests += 1
    else:
        print(f"  ✗ FAIL: pos_err={result['pos_error']*1000:.2f}mm")
    print()
    
    # Test 4: Upward reach (8 cm)
    print("[4/7] Test: Upward reach (8 cm)")
    tester.reset_to_home()
    pos, rot_mat = tester.get_ee_pose()
    target_pos = pos + np.array([0.0, 0.0, 0.08])
    tester.set_target_pose(target_pos, rot_mat)
    result = tester.solve_ik(max_iterations=300, pos_tol=2e-3, ori_tol=1e-2)
    total_tests += 1
    if result['converged']:
        print(f"  ✓ PASS: Converged in {result['iterations']} iters, pos_err={result['pos_error']*1000:.4f}mm")
        passed_tests += 1
    else:
        print(f"  ✗ FAIL: pos_err={result['pos_error']*1000:.2f}mm")
    print()
    
    # Test 5: Rotation (30 degrees around Z)
    print("[5/7] Test: Rotation (30 deg around Z-axis)")
    tester.reset_to_home()
    pos, rot_mat = tester.get_ee_pose()
    current_rot = Rotation.from_matrix(rot_mat)
    rotation_delta = Rotation.from_euler('z', 30, degrees=True)
    target_rot = rotation_delta * current_rot
    target_rot_mat = target_rot.as_matrix()
    tester.set_target_pose(pos, target_rot_mat)
    result = tester.solve_ik(max_iterations=300, pos_tol=2e-3, ori_tol=2e-2)
    total_tests += 1
    if result['converged']:
        print(f"  ✓ PASS: Converged in {result['iterations']} iters, ori_err={np.rad2deg(result['ori_error']):.2f}deg")
        passed_tests += 1
    else:
        print(f"  ✗ FAIL: ori_err={np.rad2deg(result['ori_error']):.2f}deg")
    print()
    
    # Test 6: Workspace positions
    print("[6/7] Test: Various workspace positions")
    workspace_tests = [
        ("Right 10cm", np.array([0.0, 0.10, 0.0])),
        ("Left 10cm", np.array([0.0, -0.10, 0.0])),
        ("Down 5cm", np.array([0.0, 0.0, -0.05])),
    ]
    workspace_passed = 0
    for name, offset in workspace_tests:
        tester.reset_to_home()
        pos, rot_mat = tester.get_ee_pose()
        target_pos = pos + offset
        tester.set_target_pose(target_pos, rot_mat)
        result = tester.solve_ik(max_iterations=400, pos_tol=3e-3, ori_tol=2e-2)
        if result['converged']:
            print(f"  ✓ {name}: {result['iterations']} iters, {result['pos_error']*1000:.2f}mm")
            workspace_passed += 1
        else:
            print(f"  ✗ {name}: Failed ({result['pos_error']*1000:.2f}mm)")
    total_tests += 1
    if workspace_passed == len(workspace_tests):
        passed_tests += 1
    print()
    
    # Test 7: Convergence speed
    print("[7/7] Test: Convergence speed (should be < 200 iterations)")
    tester.reset_to_home()
    pos, rot_mat = tester.get_ee_pose()
    target_pos = pos + np.array([0.03, 0.03, 0.03])
    tester.set_target_pose(target_pos, rot_mat)
    result = tester.solve_ik(max_iterations=500, pos_tol=1e-3, ori_tol=1e-2)
    total_tests += 1
    if result['converged'] and result['iterations'] < 200:
        print(f"  ✓ PASS: Converged in {result['iterations']} iterations")
        passed_tests += 1
    else:
        print(f"  ✗ FAIL: Took {result['iterations']} iterations (target < 200)")
    print()
    
    # Summary
    print("=" * 70)
    print(f"TEST SUMMARY: {passed_tests}/{total_tests} tests passed")
    print("=" * 70)
    
    return passed_tests == total_tests


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)

