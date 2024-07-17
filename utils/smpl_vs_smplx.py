#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import torch
import sys
import numpy as np
import open3d as o3d
import smplx
from scipy.spatial.transform import Rotation as R

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
        self.smpl_model=smplx.create(model_path='../models/smpl/smpl_male.pkl', model_type='smpl',
                                     gender='female', num_betas=NUM_BETAS).to(DEVICE)

        self.viz = o3d.visualization.Visualizer()
        self.viz.create_window()
        opt = self.viz.get_render_option()
        opt.show_coordinate_frame = True
        opt.background_color = np.asarray([0.5, 0.5, 0.5])
        opt.mesh_show_wireframe = True
        self.first = True
        
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

        if self.first:
            self.mesh = o3d.geometry.TriangleMesh()
            self.mesh_smpl = o3d.geometry.TriangleMesh()
            
        output = self.smplx_model(
            betas=self.betas,
            global_orient=self.global_orient,
            body_pose=self.body_pose,
            left_hand_pose=self.left_hand_pose,
            right_hand_pose=self.right_hand_pose,
            # transl=transl,
            expression=self.expression,
            return_verts=True,
            return_full_pose=False
        )
        
        output_smpl = self.smpl_model(
            betas=self.betas,
            global_orient=self.global_orient,
            body_pose=torch.cat([self.body_pose, torch.tensor([[0, 0, 0]]).to(DEVICE), torch.tensor([[0, 0, 0]]).to(DEVICE)], dim=1)
        )

        vertices = output.vertices[0].detach().cpu().numpy() 
        faces = self.smplx_model.faces
        self.mesh.vertices = o3d.utility.Vector3dVector(vertices)
        self.mesh.triangles = o3d.utility.Vector3iVector(faces)
        self.mesh.compute_vertex_normals()
        self.mesh.paint_uniform_color([1, 0.706, 0])
        
        
        vertices_smpl = output_smpl.vertices[0].detach().cpu().numpy()
        faces_smpl = self.smpl_model.faces
        self.mesh_smpl.vertices = o3d.utility.Vector3dVector(vertices_smpl)
        self.mesh_smpl.triangles = o3d.utility.Vector3iVector(faces_smpl)
        self.mesh_smpl.compute_vertex_normals()
        
        # translate the mesh to the right
        translation = np.array([0, 0, -1])
        self.mesh.translate(translation)
        
        
        if self.first:
            self.viz.add_geometry(self.mesh)
            self.viz.add_geometry(self.mesh_smpl)
            self.first = False
        
        self.viz.update_geometry(self.mesh)
        self.viz.update_geometry(self.mesh_smpl)
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