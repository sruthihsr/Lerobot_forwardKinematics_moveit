#!/usr/bin/env python3
"""
Real-time Joint State & TF Diagnostic
======================================
Run this WHILE your launch file is running to see what's happening.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer, TransformException
import numpy as np
import time


class LiveDiagnostic(Node):
    def __init__(self):
        super().__init__('live_diagnostic')
        
        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to joint states with BOTH QoS policies
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.sub_reliable = self.create_subscription(
            JointState, 
            '/joint_states', 
            lambda msg: self.joint_callback(msg, "RELIABLE"),
            qos_reliable
        )
        
        self.sub_best_effort = self.create_subscription(
            JointState,
            '/joint_states',
            lambda msg: self.joint_callback(msg, "BEST_EFFORT"),
            qos_best_effort
        )
        
        self.last_joint_msg = None
        self.message_count = {' RELIABLE': 0, 'BEST_EFFORT': 0}
        
        # Timer for TF checks
        self.create_timer(2.0, self.check_tf)
        
        self.get_logger().info('Live diagnostic started!')
        
    def joint_callback(self, msg, qos_type):
        self.message_count[qos_type] += 1
        self.last_joint_msg = msg
        
        if self.message_count[qos_type] == 1:
            print(f"\n[{qos_type}] First joint_states message received!")
            print(f"  Joint names: {msg.name}")
            print(f"  Joint positions: {[f'{p:.3f}' for p in msg.position]}")
    
    def check_tf(self):
        print("\n" + "="*70)
        print(f"DIAGNOSTIC CHECK ({time.strftime('%H:%M:%S')})")
        print("="*70)
        
        # Joint state info
        print("\n[JOINT STATES]")
        print(f"  RELIABLE subscriber: {self.message_count['RELIABLE']} messages")
        print(f"  BEST_EFFORT subscriber: {self.message_count['BEST_EFFORT']} messages")
        
        if self.last_joint_msg:
            print(f"\n  Last message:")
            for name, pos in zip(self.last_joint_msg.name, self.last_joint_msg.position):
                print(f"    Joint '{name}': {np.rad2deg(pos):.2f}° ({pos:.4f} rad)")
        else:
            print("  NO joint states received yet!")
        
        # TF tree info
        print("\n[TF FRAMES]")
        all_frames = self.tf_buffer.all_frames_as_yaml()
        if all_frames and len(all_frames) > 50:  # Has content
            print(f"  TF tree populated! Checking key frames...")
            
            # Try to find frames
            test_frames = ['base', 'gripper', '5', 'link5']
            for frame in test_frames:
                try:
                    t = self.tf_buffer.lookup_transform(
                        'base',
                        frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )
                    pos = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                    print(f"    'base' → '{frame}': [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
                except:
                    print(f"    'base' → '{frame}': NOT FOUND")
        else:
            print("  TF tree is EMPTY!")
            print("  robot_state_publisher may not be running or receiving joint states")


def main():
    print("\n" + "="*70)
    print("LIVE DIAGNOSTIC TOOL")
    print("="*70)
    print("\nThis tool checks:")
    print("  1. Are joint_states being published?")
    print("  2. With which QoS policy?")
    print("  3. Is the TF tree being populated?")
    print("  4. What are the current frame positions?")
    print("\nMake sure your launch file is running:")
    print("  ros2 launch lerobot_description so101_display_no_jsp.launch.py")
    print("\nPress Ctrl+C to exit")
    print("="*70 + "\n")
    
    rclpy.init()
    node = LiveDiagnostic()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nDiagnostic complete!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()