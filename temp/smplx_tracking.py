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
NUM_JOINTS = 55;
NUM_BODY_JOINTS = 21;

HAND_JOINTS = 15;
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
                                  gender='male', ext='npz', num_betas=NUM_BETAS, num_expressions=NUM_EXPRESSIONS, use_pca=False).to(DEVICE)

                                  
        # self.betas = torch.tensor(np.zeros(NUM_BETAS)).to(DEVICE) 
        # self.expression = torch.tensor(np.zeros(NUM_EXPRESSIONS)).to(DEVICE)
        # self.body_pose = torch.zeros(3*NUM_JOINTS).to(DEVICE)
        # self.left_hand_pose = torch.zeros(3*HAND_JOINTS).to(DEVICE)
        # self.right_hand_pose = torch.zeros(3*HAND_JOINTS).to(DEVICE)
    
        
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
        
        # betas are the first 10 parameters
        self.betas = torch.tensor(parameters[:NUM_BETAS]).reshape(1, NUM_BETAS).to(DEVICE)
            
        # expression are the next 10 parameters
        self.expression = torch.tensor(parameters[NUM_BETAS:NUM_BETAS+NUM_EXPRESSIONS]).reshape(1, NUM_EXPRESSIONS).to(DEVICE)

        # translation are the next 3 parameters (PELVIS)
        global_orient_start = NUM_BETAS + NUM_EXPRESSIONS
        global_orient_end = global_orient_start + 3

        self.global_orient = torch.tensor(parameters[global_orient_start:global_orient_end]).reshape(1, 3).to(DEVICE)
        
        # body pose are the next parameters: (SKIP PELVIS)
        body_pose_start = NUM_BETAS + NUM_EXPRESSIONS  + 3
        body_pose_end = body_pose_start + NUM_BODY_JOINTS * 3
        self.body_pose = torch.tensor(parameters[body_pose_start:body_pose_end]).reshape(1, NUM_BODY_JOINTS * 3).to(DEVICE)
        
        # right hand pose are the next parameters:
        right_hand_pose_start = body_pose_end
        right_hand_pose_end = right_hand_pose_start + HAND_JOINTS * 3
        self.right_hand_pose = torch.tensor(parameters[right_hand_pose_start:right_hand_pose_end]).reshape(1, HAND_JOINTS * 3).to(DEVICE)
        
        # left hand pose are the next parameters:
        left_hand_pose_start = right_hand_pose_end
        left_hand_pose_end = left_hand_pose_start + HAND_JOINTS * 3
        self.left_hand_pose = torch.tensor(parameters[left_hand_pose_start:left_hand_pose_end]).reshape(1, HAND_JOINTS * 3).to(DEVICE)
        
        # print("Received Parameters with the following shapes:")
        # print("Betas: ", self.betas.shape)
        # print("Expression: ", self.expression.shape)
        print("Body Pose: ", self.body_pose.shape)
        # print("Right Hand Pose: ", self.right_hand_pose.shape)
        # print("Left Hand Pose: ", self.left_hand_pose.shape)
    
        self.visualize_model()

    def visualize_model(self):

        if self.first:
            self.mesh = o3d.geometry.TriangleMesh()
        
        # create a copy
        # old_bodypose = self.body_pose.clone()
        # self.body_pose[0,:] = 0.0
        
        
        # # start = 17 * 3
        # # end = 18 * 3
        
        # r = R.from_rotvec(self.body_pose[0,16*3:16*3+3].detach().cpu().numpy())
        # euler = r.as_euler('xyz')
        
        
        # self.body_pose[0,16*3] = old_bodypose[0,16*3]
        # self.body_pose[0,16*3 +1 ] = old_bodypose[0,16*3 +1]
        # self.body_pose[0,16*3 +2 ] = old_bodypose[0,16*3 +2]
        
        # print("Shoulder Euler: ", euler)
        # print("Shoulder: ", self.body_pose[0,16*3:16*3+3])
        
        
        # print(self.body_pose)
        output = self.smplx_model(
            betas=self.betas,
            # 180 deg on z axis
            global_orient=self.global_orient,
            body_pose=self.body_pose,
            left_hand_pose=self.left_hand_pose,
            right_hand_pose=self.right_hand_pose,
            # transl=transl,
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
        
def main(args=None):
    rclpy.init(args=args)
    node = SMPLXVisualizer()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()