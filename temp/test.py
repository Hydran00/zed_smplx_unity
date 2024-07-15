#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import smplx
import torch
### example API
# model = smplx.create(model_folder, model_type=model_type,
#              gender=gender, use_face_contour=use_face_contour,
#              num_betas=num_betas,
#              num_expression_coeffs=num_expression_coeffs,
#              ext=ext)
# output = model(betas=betas, expression=expression,
#        return_verts=True)
import open3d as o3d
from joint_names import JOINT_NAMES

NUM_BETAS = 10;
NUM_EXPRESSIONS = 10;
NUM_JOINTS = 55;

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class SMPLXVisualizer(Node):
    def __init__(self):
        super().__init__('smplx_visualizer')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/smpl_params',
            self.listener_callback,
            10)
        self.subscription
        self.model = smplx.create('../models/SMPLX_MALE.npz', model_type='smplx',
            gender='male')
        
    def listener_callback(self, msg):
        parameters = np.array(msg.data)
        
        if parameters.size != 185:
            self.get_logger().error(f"Received unexpected number of parameters: {parameters.size}")
            return

        self.betas = torch.tensor([parameters[:NUM_BETAS]])  
        self.expressions = torch.tensor([parameters[NUM_BETAS:NUM_BETAS+NUM_EXPRESSIONS]])
        self.joint_rotations = []
        for i in range(21):
            self.joint_rotations.append([parameters[NUM_BETAS+NUM_EXPRESSIONS+3*i], parameters[NUM_BETAS+NUM_EXPRESSIONS+3*i+1], parameters[NUM_BETAS+NUM_EXPRESSIONS+3*i+2]])
        self.joint_rotations = torch.tensor(self.joint_rotations)
            
        print("betas:",self.betas.shape)
        print("expressions:",self.expressions.shape)
        print("joint_rotations:",self.joint_rotations.shape)

        self.visualize_model()

    def visualize_model(self):
        

        output = self.model(betas=self.betas, expression=self.expressions, body_pose=self.joint_rotations, return_verts=True) 
        vertices = output.vertices
        faces = output.faces
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices)
        mesh.triangles = o3d.utility.Vector3iVector(faces)
        o3d.visualization.draw_geometries([mesh])
        
def main(args=None):
    rclpy.init(args=args)
    node = SMPLXVisualizer()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()