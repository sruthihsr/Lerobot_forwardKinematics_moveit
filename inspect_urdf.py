#!/usr/bin/env python3
"""
URDF Structure Inspector
=========================
Parses the actual URDF file to show the kinematic chain.
"""

import xml.etree.ElementTree as ET
import os
import sys


def find_urdf():
    """Find the URDF file"""
    possible_paths = [
        os.path.expanduser('~/so101_robotics_course/ros2_ws/install/lerobot_description/share/lerobot_description/urdf/so101.urdf'),
        os.path.expanduser('~/lerobot_ws/install/lerobot_description/share/lerobot_description/urdf/so101.urdf'),
        os.path.expanduser('~/so101_robotics_course/ros2_ws/src/lerobot_description/urdf/so101.urdf'),
        '/opt/ros/humble/share/lerobot_description/urdf/so101.urdf',
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            return path
    return None


def parse_urdf(urdf_path):
    """Parse URDF and extract kinematic information"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    print("="*70)
    print(f"URDF FILE: {urdf_path}")
    print("="*70)
    
    # Get all joints
    print("\n[JOINTS]")
    print("-"*70)
    joints = []
    for joint in root.findall('joint'):
        name = joint.get('name')
        jtype = joint.get('type')
        parent = joint.find('parent').get('link') if joint.find('parent') is not None else None
        child = joint.find('child').get('link') if joint.find('child') is not None else None
        
        origin = joint.find('origin')
        xyz = origin.get('xyz') if origin is not None else "0 0 0"
        rpy = origin.get('rpy') if origin is not None else "0 0 0"
        
        axis = joint.find('axis')
        axis_xyz = axis.get('xyz') if axis is not None else "N/A"
        
        joints.append({
            'name': name,
            'type': jtype,
            'parent': parent,
            'child': child,
            'xyz': xyz,
            'rpy': rpy,
            'axis': axis_xyz
        })
        
        print(f"\nJoint: {name} ({jtype})")
        print(f"  {parent} → {child}")
        print(f"  Origin: xyz={xyz}, rpy={rpy}")
        if axis_xyz != "N/A":
            print(f"  Axis: {axis_xyz}")
    
    # Get all links
    print("\n" + "="*70)
    print("[LINKS]")
    print("-"*70)
    for link in root.findall('link'):
        name = link.get('name')
        print(f"  - {name}")
    
    # Build kinematic chain
    print("\n" + "="*70)
    print("[KINEMATIC CHAIN]")
    print("-"*70)
    
    # Find base joint
    base_joints = [j for j in joints if j['parent'] == 'base' or j['parent'] == 'base_link']
    if base_joints:
        chain = []
        current = base_joints[0]
        chain.append(f"base → {current['child']} (via {current['name']})")
        
        # Follow the chain
        while True:
            next_joints = [j for j in joints if j['parent'] == current['child']]
            if not next_joints:
                break
            current = next_joints[0]
            chain.append(f"  → {current['child']} (via {current['name']})")
        
        for link in chain:
            print(link)
    
    # Check for 'gripper' link/frame
    print("\n" + "="*70)
    print("[GRIPPER FRAME]")
    print("-"*70)
    
    gripper_links = [link.get('name') for link in root.findall('link') if 'gripper' in link.get('name').lower()]
    gripper_joints = [j for j in joints if 'gripper' in j['name'].lower() or 'gripper' in j['child'].lower()]
    
    print(f"Links with 'gripper': {gripper_links}")
    print(f"Joints related to gripper:")
    for j in gripper_joints:
        print(f"  - {j['name']}: {j['parent']} → {j['child']}")
        print(f"    Origin: {j['xyz']}")


def main():
    urdf_path = find_urdf()
    
    if not urdf_path:
        print("ERROR: Could not find SO-101 URDF file!")
        print("\nSearched in:")
        print("  ~/so101_robotics_course/ros2_ws/...")
        print("  ~/lerobot_ws/...")
        print("  /opt/ros/humble/...")
        print("\nMake sure you've built the lerobot_ws workspace.")
        sys.exit(1)
    
    parse_urdf(urdf_path)
    
    print("\n" + "="*70)
    print("ANALYSIS COMPLETE")
    print("="*70)
    print("\nUse this information to:")
    print("  1. Verify joint names match your code")
    print("  2. Check which link is the 'gripper' frame")
    print("  3. Understand the kinematic chain")
    print("  4. Calculate correct offsets for FK")


if __name__ == '__main__':
    main()