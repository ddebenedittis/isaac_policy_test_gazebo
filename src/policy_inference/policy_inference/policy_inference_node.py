#!/usr/bin/env python3

import yaml
import os

import numpy as np
import onnxruntime
import quaternion
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
from velocity_command_msgs.msg import SimpleVelocityCommand


def quat_rot(q, v):
    """
    Rotate a three dimensional vector v with quaternion q.
    
    params
    ----
    q: quaternion
    v: vector to be rotated
    
    return
    ----
    q * [0, v] * q.conj
    """
    
    return quaternion.as_vector_part(
        q * quaternion.from_vector_part(v) * q.conj()
    )


class PolicyInferenceNode(Node):
    def __init__(self):
        super().__init__('policy_inference_node')
        
        self.package_share_directory = get_package_share_directory('policy_inference')
        
        self.started = False
        
        # ============================ Subscribers =========================== #
        
        self.joint_states_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_callback,
            1)
        
        self.link_states_subscription = self.create_subscription(
            LinkStates,
            "/gazebo/link_states",
            self.link_states_callback,
            1)
        
        self.imu_subscription = self.create_subscription(
            Imu,
            "/imu_sensor_broadcaster/imu",
            self.imu_callback,
            1)
        
        self.vel_cmd_subscription = self.create_subscription(
            SimpleVelocityCommand,
            "/motion_generator/simple_velocity_command",
            self.vel_cmd_callback,
            1)
        
        # ============================ Publishers ============================ #
        
        self.pos_ref_publisher = self.create_publisher(
            Float64MultiArray,
            "/joints_position_reference",
            1)
        
        self.vel_ref_publisher = self.create_publisher(
            Float64MultiArray,
            "/joints_velocity_reference",
            1)
        
        # ==================================================================== #
        
        self.joint_names = [
            'LF_HFE',
            'LH_HFE',
            'RF_HFE',
            'RH_HFE',
            'LF_KFE',
            'LH_KFE',
            'RF_KFE',
            'RH_KFE',
        ]
        
        self.alphabetical_joint_names = [
            'LF_HFE',
            'LF_KFE',
            'LH_HFE',
            'LH_KFE',
            'RF_HFE',
            'RF_KFE',
            'RH_HFE',
            'RH_KFE',
        ]
        
        self.q_j = np.zeros(8)             # joints position
        self.v_j = np.zeros(8)             # joints velocity
        
        self.q_b = np.zeros(7)
        self.v_b = np.zeros(6)
        self.a_b = np.zeros(3)
        self.w_b = np.zeros(3)
        
        self.vel_cmd = np.array([0.0, 0.])
        
        self.actions = np.zeros(8)
        
        # ==================================================================== #
        
        self.model = None
        
        config_path = os.path.join(self.package_share_directory, 'policies', 'Mulinex_1', 'config.yaml')
        with open(config_path, 'r') as f:
            params = yaml.safe_load(f)
            
        # self.rate = 1.0 / (params['task']['sim']['dt'])
        self.rate = 60.
        
        default_joint_angles = params['task']['env']['defaultJointAngles']
        self.default_joint_angles = np.array([
            default_joint_angles['LF_HFE'],
            default_joint_angles['LH_HFE'],
            default_joint_angles['RF_HFE'],
            default_joint_angles['RH_HFE'],
            default_joint_angles['LF_KFE'],
            default_joint_angles['LH_KFE'],
            default_joint_angles['RF_KFE'],
            default_joint_angles['RH_KFE'],
        ])
        
        self.pos_ref = self.default_joint_angles

        self.action_scale       = params['task']['env']['control']['actionScale']
        self.linearVelocityScale   = params['task']['env']['learn']['linearVelocityScale']
        self.angularVelocityScale   = params['task']['env']['learn']['angularVelocityScale']
        self.dofPositionScale   = params['task']['env']['learn']['dofPositionScale']
        self.dofVelocityScale   = params['task']['env']['learn']['dofVelocityScale']
        
        # ==================================================================== #
        
        self.timer = self.create_timer(1/self.rate, self.timer_callback)
        
        # ==================================================================== #
        
        self.load_policy()
        
    def joint_states_callback(self, msg: JointState):
        """
        Read, reorder and save the joint states.
        The joints are saved in the following order: LF_(HAA -> HFE -> KFE) -> LH -> RF -> RH
        """
                        
        for i in range(len(self.joint_names)):
            self.q_j[self.joint_names.index(msg.name[i])] = msg.position[i]
            self.v_j[self.joint_names.index(msg.name[i])] = msg.velocity[i]
            
    def link_states_callback(self, msg):
        # The index [1] is used because the first link ([0]) is the ground_plane. The second one (the index [1]) corresponds to anymal base.
        base_id = next((i for i, name in enumerate(msg.name) if name != "base"), None)

        # /gazebo/link_states returns the pose and the twist in the inertial or world frame.
        
        # Extract the base position and orientation (quaternion)
        pos = msg.pose[base_id].position
        orient = msg.pose[base_id].orientation

        # Extract the base linear and angular velocity
        lin = msg.twist[base_id].linear
        ang = msg.twist[base_id].angular

        # Save the base pose and twists as numpy arrays
        self.q_b = np.array([pos.x, pos.y, pos.z, orient.w, orient.x, orient.y, orient.z])
        self.v_b = np.array([lin.x, lin.y, lin.z, ang.x, ang.y, ang.z])
    
    def imu_callback(self, msg: Imu):
        acc = msg.linear_acceleration
        ang_vel = msg.angular_velocity

        self.a_b = np.array([acc.x, acc.y, acc.z])
        self.w_b = np.array([ang_vel.x, ang_vel.y, ang_vel.z])  
        
    def vel_cmd_callback(self, msg: SimpleVelocityCommand):
        self.started = True
        
        self.vel_cmd = np.array([msg.velocity_forward, msg.yaw_rate])
    
    def load_policy(self):
        policy_path = os.path.join(self.package_share_directory, 'policies', 'Mulinex_1',  'Mulinexexport.onnx')
        self.model = onnxruntime.InferenceSession(policy_path)

    def perform_inference(self):
        orientation = quaternion.from_float_array(self.q_b[3:7])
        
        observations = np.concatenate((
            quat_rot(orientation.conj(), self.v_b[0:3]) * self.linearVelocityScale,
            quat_rot(orientation.conj(), self.v_b[3:6]) * self.angularVelocityScale,
            # - self.a_b / np.linalg.norm(self.a_b),
            quat_rot(orientation, np.array([0.,0.,-1.])),
            self.vel_cmd * np.array([self.linearVelocityScale, self.angularVelocityScale]),
            (self.q_j - self.default_joint_angles) * self.dofPositionScale,
            self.v_j * self.dofVelocityScale,
            self.actions.flatten(),
        )).astype(np.float32).reshape(1, -1)
        
        input_name = self.model.get_inputs()[0].name
        self.actions = self.model.run(None, {input_name: observations})[0]
        
    def timer_callback(self):
        if self.started:
            self.perform_inference()
            
            self.pos_ref = self.default_joint_angles + self.actions.flatten()
            pos_ref_reordered = np.zeros(len(self.pos_ref))
                        
            for i, joint in enumerate(self.alphabetical_joint_names):
                index = self.joint_names.index(joint)
                pos_ref_reordered[i] = self.pos_ref[index]
            
            msg = Float64MultiArray()
            msg.data = pos_ref_reordered
            
            self.pos_ref_publisher.publish(msg)
        

def main(args=None):    
    rclpy.init(args=args)
    node = PolicyInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
