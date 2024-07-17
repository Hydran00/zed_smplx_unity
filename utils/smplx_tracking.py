#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty  # Import the Empty service type
from std_msgs.msg import Float32MultiArray
import numpy as np
import torch
import sys
import open3d as o3d
import smplx
import pickle

NUM_BETAS = 10;
NUM_EXPRESSIONS = 10;

NUM_GLOBAL_ORIENT_JOINTS = 1;
NUM_BODY_JOINTS = 21;
NUM_FACE_JOINTS = 3;
NUM_HAND_JOINTS = 15;

NUM_PARAMS = NUM_BETAS + NUM_EXPRESSIONS + 3 * (NUM_GLOBAL_ORIENT_JOINTS + NUM_BODY_JOINTS + NUM_FACE_JOINTS + 2 * NUM_HAND_JOINTS); # 185

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
        self.smplx_model = smplx.create('../models/smplx/SMPLX_MALE.npz', model_type='smplx',
                                  gender='male', ext='npz', num_betas=NUM_BETAS, num_expressions=NUM_EXPRESSIONS, use_pca=False).to(DEVICE)
        
        self.viz = o3d.visualization.Visualizer()
        self.viz.create_window()
        opt = self.viz.get_render_option()
        opt.show_coordinate_frame = True
        opt.background_color = np.asarray([0.5, 0.5, 0.5])
        opt.mesh_show_wireframe = True
        self.first = True
        
        self.mesh = o3d.geometry.TriangleMesh()
        
        self.srv = self.create_service(Empty, 'dump_smplx_parameters', self.dump_params_callback)
        
        print("Node Started")
        
    def listener_callback(self, msg):
        if(len(msg.data) != NUM_PARAMS):
            print("Invalid number of parameters: {} but they should be {}".format(len(msg.data), NUM_PARAMS))
            return
        parameters = np.array(msg.data)
        
        # betas are the first 10 parameters
        self.betas = torch.tensor(parameters[:NUM_BETAS]).reshape(1, NUM_BETAS).to(DEVICE)
            
        # expression are the next 10 parameters
        self.expression = torch.tensor(parameters[NUM_BETAS:NUM_BETAS+NUM_EXPRESSIONS]).reshape(1, NUM_EXPRESSIONS).to(DEVICE)

        # translation are the next 3 parameters (PELVIS)
        global_orient_start = NUM_BETAS + NUM_EXPRESSIONS
        global_orient_end = global_orient_start + 3

        self.global_orient = torch.tensor(parameters[global_orient_start:global_orient_end]).reshape(1, 3).to(DEVICE)
        
        # body pose are the next parameters: (SKIP PELVIS)
        body_pose_start = global_orient_end
        body_pose_end = body_pose_start + NUM_BODY_JOINTS * 3
        self.body_pose = torch.tensor(parameters[body_pose_start:body_pose_end]).reshape(1, NUM_BODY_JOINTS * 3).to(DEVICE)
        
        # jaw, left_eye_smplhf, right_eye_smplhf
        face_pose_start = body_pose_end
        face_pose_end = face_pose_start + 9
        self.jaw_pose = torch.tensor(parameters[face_pose_start:face_pose_end - 6]).reshape(1, 3).to(DEVICE)
        self.left_eye_pose = torch.tensor(parameters[face_pose_start + 3:face_pose_end - 3]).reshape(1, 3).to(DEVICE)
        self.right_eye_pose = torch.tensor(parameters[face_pose_start + 6:face_pose_end]).reshape(1, 3).to(DEVICE)
        
        
        # right hand pose are the next parameters:
        right_hand_pose_start = face_pose_end
        right_hand_pose_end = right_hand_pose_start + NUM_HAND_JOINTS * 3
        self.right_hand_pose = torch.tensor(parameters[right_hand_pose_start:right_hand_pose_end]).reshape(1, NUM_HAND_JOINTS * 3).to(DEVICE)
        
        # left hand pose are the next parameters:
        left_hand_pose_start = right_hand_pose_end
        left_hand_pose_end = left_hand_pose_start + NUM_HAND_JOINTS * 3
        self.left_hand_pose = torch.tensor(parameters[left_hand_pose_start:left_hand_pose_end]).reshape(1, NUM_HAND_JOINTS * 3).to(DEVICE)
        
        self.visualize_model()

    def visualize_model(self):

        output = self.smplx_model(
            betas=self.betas,
            global_orient=self.global_orient,
            body_pose=self.body_pose,
            jaw_pose=self.jaw_pose,
            leye_pose=self.left_eye_pose,
            reye_pose=self.right_eye_pose,
            left_hand_pose=self.left_hand_pose,
            right_hand_pose=self.right_hand_pose,
            expression=self.expression,
            return_verts=True,
            return_full_pose=False
        )

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

    def dump_params_callback(self, request, response):
        with open('~/smplx_params.pkl', 'wb') as f:
            pickle.dump({
                'betas': self.betas,
                'global_orient': self.global_orient,
                'body_pose': self.body_pose,
                'jaw_pose': self.jaw_pose,
                'left_eye_pose': self.left_eye_pose,
                'right_eye_pose': self.right_eye_pose,
                'left_hand_pose': self.left_hand_pose,
                'right_hand_pose': self.right_hand_pose,
                'expression': self.expression
            }, f)
        print("Dumped SMPLX Parameters")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SMPLXVisualizer()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
