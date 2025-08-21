import trimesh
import numpy as np
import os

link_transforms = {
    "base": {
        "rpy": [0, 0, np.pi],
        "xyz": [0, 0, 0]
    },
    "shoulder": {
        "rpy": [0, 0, np.pi],
        "xyz": [0, 0, 0]
    },
    "upperarm": {
        "rpy": [np.pi/2, 0, -np.pi/2],
        "xyz": [0, 0, 0.138]
    },
    "forearm": {
        "rpy": [np.pi/2, 0, -np.pi/2],
        "xyz": [0, 0, 0.007]
    },
    "wrist1": {
        "rpy": [np.pi/2, 0, 0],
        "xyz": [0, 0, -0.127]
    },
    "wrist2": {
        "rpy": [0, 0, 0],
        "xyz": [0, 0, -0.0997]
    },
    "wrist3": {
        "rpy": [np.pi/2, 0, 0],
        "xyz": [0, 0, -0.0989]
    }
}

def rpy_to_matrix(roll, pitch, yaw):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx

def transform_vertices(vertices, rpy, xyz):
    R = rpy_to_matrix(*rpy)
    return vertices @ R.T + np.array(xyz)

def get_obj_files_by_prefix(directory: str, prefix: str):
    return sorted([
        os.path.join(directory, fname)
        for fname in os.listdir(directory)
        if fname.startswith(prefix) and fname.endswith('.dae')
    ])

def export_mesh_to_txt(mesh: trimesh.Trimesh, link_name: str, filename: str):
    with open(filename, 'w') as f:
        f.write(f"{link_name}\n")
        f.write("V\n")
        for vertex in mesh.vertices:
            f.write(f"{vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")
        f.write("F\n")
        for face in mesh.faces:
            f.write(f"{face[0]} {face[1]} {face[2]}\n")

path_to_folder = "/home/chan/catkin_ws/src/robot_description/ur_description/meshes/ur5e"
assets_folder = os.path.join(path_to_folder, "visual")
mesh_folder = os.path.join(path_to_folder, "sca_meshes")
linkMap = ["base", "shoulder", "upperarm", "forearm", "wrist1", "wrist2", "wrist3"]

all_meshes = []

for link in linkMap:
    obj_files = get_obj_files_by_prefix(assets_folder, link)
    meshes = [trimesh.load(f, force='mesh') for f in obj_files]
    combined = trimesh.util.concatenate(meshes)
    
    if link in link_transforms:
        rpy = link_transforms[link]["rpy"]
        xyz = link_transforms[link]["xyz"]
        combined.vertices = transform_vertices(combined.vertices, rpy, xyz)

    filename = os.path.join(mesh_folder, f"{link}.txt")
    export_mesh_to_txt(combined, link_name=link, filename=filename)


