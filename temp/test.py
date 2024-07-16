#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import torch
import sys
import numpy as np
import open3d as o3d
from joint_names import JOINT_NAMES
import smplx

NUM_BETAS = 10;
NUM_EXPRESSIONS = 10;
NUM_JOINTS = 24;
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


# VISUALIZE

# show wireframe

class SMPLXVisualizer(Node):
    def __init__(self):
        super().__init__('smplx_visualizer')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/smpl_params',
            self.listener_callback,
            10)
        self.subscription
        self.smplx_model = smplx.create('../models/smplx/SMPLX_MALE.npz', model_type='smplx',
                                  gender='male', ext='npz', num_betas=NUM_BETAS, num_expressions=NUM_EXPRESSIONS,
                                  use_face_contour=False, num_joints=NUM_JOINTS).to(DEVICE)
                                  
        self.betas = torch.tensor(np.zeros(NUM_BETAS)).to(DEVICE) 
        self.expression = torch.tensor(np.zeros(NUM_EXPRESSIONS)).to(DEVICE)
        self.pose = torch.tensor(np.zeros(3*NUM_JOINTS)).to(DEVICE)
        self.viz = o3d.visualization.Visualizer()
        self.viz.create_window()
        opt = self.viz.get_render_option()
        opt.show_coordinate_frame = True
        opt.background_color = np.asarray([0.5, 0.5, 0.5])
        opt.mesh_show_wireframe = True
        self.first = True
        
        print("Node Started")
        
    def listener_callback(self, msg):
        parameters = np.array(msg.data)

        self.betas = torch.tensor(parameters[:NUM_BETAS]).reshape(1, NUM_BETAS).to(DEVICE)
        self.expression = torch.tensor(parameters[NUM_BETAS:NUM_BETAS+NUM_EXPRESSIONS]).reshape(1, NUM_EXPRESSIONS).to(DEVICE)
        self.pose = torch.tensor(parameters[NUM_BETAS+NUM_EXPRESSIONS: NUM_BETAS+NUM_EXPRESSIONS+3*NUM_JOINTS]).reshape(1, 3*NUM_JOINTS).to(DEVICE)        
        
        self.visualize_model()

    def visualize_model(self):

        if self.first:
            self.mesh = o3d.geometry.TriangleMesh()
            
        output = self.smplx_model(betas=self.betas, expression=self.expression)
        vertices = output.vertices[0].detach().cpu().numpy() 
        faces = self.smplx_model.faces
        self.mesh.vertices = o3d.utility.Vector3dVector(vertices)
        self.mesh.triangles = o3d.utility.Vector3iVector(faces)
        
        if self.first:
            self.viz.add_geometry(self.mesh)
            self.first = False
        
        self.viz.update_geometry(self.mesh)
        self.viz.poll_events()
        self.viz.update_renderer()
        
def main(args=None):
    rclpy.init(args=args)
    node = SMPLXVisualizer()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()