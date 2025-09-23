import rospy
import rospkg
import numpy as np

import trimesh
from math import cos, sin


def rpy_to_matrix(rpy):
    """Convert roll-pitch-yaw to rotation matrix."""
    roll, pitch, yaw = rpy
    Rx = np.array([[1, 0, 0],
                   [0, cos(roll), -sin(roll)],
                   [0, sin(roll), cos(roll)]])
    Ry = np.array([[cos(pitch), 0, sin(pitch)],
                   [0, 1, 0],
                   [-sin(pitch), 0, cos(pitch)]])
    Rz = np.array([[cos(yaw), -sin(yaw), 0],
                   [sin(yaw), cos(yaw), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx

def transform_mesh(mesh, rpy, xyz):
    """Apply RPY + translation transform to mesh."""
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return mesh.copy().apply_transform(T)

def load_mesh_with_transform(path, rpy=(0,0,0), xyz=(0,0,0)):
    mesh = trimesh.load_mesh(path, force='mesh')
    return transform_mesh(mesh, rpy, xyz)

def getSE3(rpy, xyz):
    T = np.eye((4))
    T[:3, :3] = rpy_to_matrix(rpy)
    T[:3, -1] = np.array(xyz)
    
    return T

rospack = rospkg.RosPack()
package_path = rospack.get_path('robotiq_85_description')
path_to_stl = package_path + '/meshes/collision'
# path_to_stl = package_path + path_to_stl

components = [
    # base_link
    {
        "file": f"{path_to_stl}/robotiq_85_base_link.stl",
        "rpy": (3.1416, -1.5708, 0),
        "xyz": (0, 0, -0.006)
    },

    # knuckle 1
    {
        "file": f"{path_to_stl}/robotiq_85_knuckle_link.stl",
        "rpy": (np.pi, 0, 0),
        "xyz": (0.05490451627, 0.03060114443, 0.0)
    },

    # knuckle 2
    {
        "file": f"{path_to_stl}/robotiq_85_knuckle_link.stl",
        "rpy": (0, 0, 0),
        "xyz": (0.05490451627, -0.03060114443, 0.0)
    },

    # finger 1 : relative to knuckle 1
    {
        "file": f"{path_to_stl}/robotiq_85_finger_link.stl",
        "rpy": (0, 0, 0),
        "xyz": (-0.00408552455, -0.03148604435, 0.0)
    },

    # finger 2 : relative to knuckle 2
    {
        "file": f"{path_to_stl}/robotiq_85_finger_link.stl",
        "rpy": (0, 0, 0),
        "xyz": (-0.00408552455, -0.03148604435, 0.0)
    },
    # inner knuckle 1 : relative to base link
    {
        "file": f"{path_to_stl}/robotiq_85_inner_knuckle_link.stl",
        "rpy": (np.pi, 0, 0),
        "xyz": (0.06142, 0.0127, 0)
    },

    # inner knuckle 2 : relative to base link
    {
        "file": f"{path_to_stl}/robotiq_85_inner_knuckle_link.stl",
        "rpy": (0, 0, 0),
        "xyz": (0.06142, -0.0127, 0)
    },

    # finger tip 1 : relative to inner knuckle 1
    {
        "file": f"{path_to_stl}/robotiq_85_finger_tip_link.stl",
        "rpy": (0, 0, 0),
        "xyz": (0.04303959807, -0.03759940821, 0.0)
    },

    # finger tip 2 : relative to inner knuckle 2
    {
        "file": f"{path_to_stl}/robotiq_85_finger_tip_link.stl",
        "rpy": (0, 0, 0),
        "xyz": (0.04303959807, -0.03759940821, 0.0)
    },
]

# Relative Index
# 0 - 1,2, 5,6
# 1 - 3
# 2 - 4
# 5 - 7
# 6 - 8

T_wrist_23 = getSE3((1.5707963267948966, 0, 0), (0, 0, -0.0989))

mesh_ee = trimesh.load_mesh("/home/chan/catkin_ws/src/robot_description/ur_e_description/meshes/ur5e/collision/wrist3.stl")
mesh_ee.apply_transform(T_wrist_23)
mesh_ee.visual.face_colors = [255, 0, 0, 255] 
mesh_list =[mesh_ee]

mesh_camera = trimesh.load_mesh("/home/chan/catkin_ws/src/robot_description/summit_xl_description/meshes/structures/camera_bracket.STL")
T_camera = T_wrist_23 @ getSE3((-np.pi/2, 0, 0), (0, 0.1, 0)) @ getSE3((0, 0, 0), (0, 0, 0.007)) @ getSE3((1.5708, 0, 1.5708), (0.0, 0.0, 0.0))
mesh_camera.apply_transform(T_camera)
mesh_list.append(mesh_camera)

T_hand_base = T_wrist_23 @ getSE3((-np.pi/2, 0, 0), (0, 0.1, 0)) @ getSE3((0, 0, 0), (0, 0, 0.007))

rel_base = [1,2,5,6]

base_mesh = load_mesh_with_transform(components[0]['file'], components[0]['rpy'], components[0]['xyz'])
base_mesh.apply_transform(T_hand_base)

mesh_list.append(base_mesh)
T_rel_list = []

for i in rel_base:
    mesh = trimesh.load_mesh(components[i]['file'])
    T_rel = T_hand_base @ getSE3(components[0]['rpy'], components[0]['xyz']) @ getSE3(components[i]['rpy'], components[i]['xyz'])
    mesh.apply_transform(T_rel)
    mesh_list.append(mesh)

T_rel_13 = T_hand_base @ getSE3(components[0]['rpy'], components[0]['xyz']) @ getSE3(components[1]['rpy'], components[1]['xyz']) @ getSE3(components[3]['rpy'], components[3]['xyz'])
mesh3 = trimesh.load_mesh(components[3]['file'])
mesh3.apply_transform(T_rel_13)
mesh_list.append(mesh3)

T_rel_24 = T_hand_base @ getSE3(components[0]['rpy'], components[0]['xyz']) @ getSE3(components[2]['rpy'], components[2]['xyz']) @ getSE3(components[4]['rpy'], components[4]['xyz'])
mesh4 = trimesh.load_mesh(components[4]['file'])
mesh4.apply_transform(T_rel_24)
mesh_list.append(mesh4)

T_rel_57 = T_hand_base @ getSE3(components[0]['rpy'], components[0]['xyz']) @ getSE3(components[5]['rpy'], components[5]['xyz']) @ getSE3(components[7]['rpy'], components[7]['xyz'])
mesh7 = trimesh.load_mesh(components[7]['file'])
mesh7.apply_transform(T_rel_57)
mesh_list.append(mesh7) 

T_rel_68 = T_hand_base @ getSE3(components[0]['rpy'], components[0]['xyz']) @ getSE3(components[6]['rpy'], components[6]['xyz']) @ getSE3(components[8]['rpy'], components[8]['xyz'])
mesh8 = trimesh.load_mesh(components[8]['file'])
mesh8.apply_transform(T_rel_68)
mesh_list.append(mesh8) 




scene = trimesh.util.concatenate(mesh_list)

# 렌더링
scene.show()

txt_path = "/home/chan/catkin_ws/src/wrist3.txt"
txt_lines = []

for v in scene.vertices:
    txt_lines.append("\t".join(f"{coord:.14f}" for coord in v))

# Add face section
txt_lines.append("F")
for f in scene.faces:
    txt_lines.append("\t".join(str(idx) for idx in f))

# Save to new txt file
with open(txt_path, "w") as f:
    f.write("\n".join(txt_lines))