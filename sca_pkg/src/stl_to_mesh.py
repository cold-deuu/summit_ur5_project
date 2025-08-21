import os
import trimesh
import numpy as np
from math import cos, sin
from pathlib import Path

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

# ======================================
# 🛠️ Main Assembly Script
# ======================================

# URDF 패키지 경로 예시 (여기에 맞게 수정하세요)
base_path = "/home/chan/catkin_ws/src/robot_description/summit_xl_description/meshes/bases/xls"
structure_path = "/home/chan/catkin_ws/src/robot_description/summit_xl_description/meshes/structures"
wheel_path = "/home/chan/catkin_ws/src/robot_description/summit_xl_description/meshes/wheels"
# 불러올 메쉬들 및 변환 정의 (예시는 일부만, 필요한 경우 모두 추가 가능)
components = [
    # base_link
    {
        "file": f"{base_path}/summit_xls_chassis.stl",
        "rpy": (0, 0, 0),
        "xyz": (0, 0, 0)
    },
    # chapa
    {
        "file": f"{base_path}/summit_xls_chapas_inox_tapas.stl",
        "rpy": (0, 0, 0),
        "xyz": (0.0, 0.0, 0.38062)
    },
    # # logo_left
    # {
    #     "file": f"{base_path}/robotnik_logo_chasis.stl",
    #     "rpy": (0, 0, 0),
    #     "xyz": (0.0, 0.28931, 0.29062)
    # },
    # # logo_right
    # {
    #     "file": f"{base_path}/robotnik_logo_chasis.stl",
    #     "rpy": (0, 0, 3.141592653589793),
    #     "xyz": (0.0, -0.28931, 0.29062)
    # },
    # # logo_front
    # {
    #     "file": f"{base_path}/robotnik_logo_chasis.stl",
    #     "rpy": (0, 0, -1.5707963267948966),
    #     "xyz": (0.345, 0.0, 0.0)
    # },
    # # logo_rear
    # {
    #     "file": f"{base_path}/robotnik_logo_chasis.stl",
    #     "rpy": (0, 0, 1.5707963267948966),
    #     "xyz": (-0.345, 0.0, 0.0)
    # },
    # top_structure
    {
        "file": f"{structure_path}/top_structure_roas.STL",
        "rpy": (0, 0, 0),
        "xyz": (0, 0, -0.127)  # base_link (0) + top offset (0.127) + collision offset (0.29) + z = 0.75
    },
    # camera
    # {
    #     "file": f"{structure_path}/d435.dae",
    #     "rpy": (1.5707963267948966, 0, 1.5707963267948966),
    #     "xyz": (0.4063, 0, 1.056)  # top_structure offset 포함
    # },
    # front right
    {
        "file": f"{wheel_path}/omni_wheel_2.dae",
        "rpy": (0,0,0),
        "xyz": (0.2225, 0.2045, 0.0)
    },
    # front left
    {
        "file": f"{wheel_path}/omni_wheel_1.dae",
        "rpy": (0,0,0),
        "xyz": (0.2225, -0.2045, 0.0)
    },  

    # rear right
    {
        "file": f"{wheel_path}/omni_wheel_2.dae",
        "rpy": (0,0,0),
        "xyz": (-0.2225, -0.2045, 0.0)
    },     
    # rear left
    {
        "file": f"{wheel_path}/omni_wheel_1.dae",
        "rpy": (0,0,0),
        "xyz": (-0.2225, 0.2045, 0.0)
    },      
    # lidar (os1)
#     {
#         "file": f"{structure_path}/os1.dae",
#         "rpy": (0, 0, 1.5707963267948966),
#         "xyz": (-0.2867, 0, 1.02618)
#     }
]

# 모든 메쉬 통합
all_meshes = []
for i, comp in enumerate(components):
    path = comp["file"]
    if not os.path.exists(path):
        print(f"[❌] File not found: {path}")
        continue

    print(f"[📦] Loading mesh {i}: {os.path.basename(path)}")
    print(f"     ↳ RPY: {comp['rpy']}, XYZ: {comp['xyz']}")
    mesh = load_mesh_with_transform(path, rpy=comp["rpy"], xyz=comp["xyz"])

    # 스케일 너무 작거나 너무 큰 경우 경고
    extents = mesh.extents
    if np.any(extents > 10):
        print(f"[⚠️] Mesh {path} may be too large (extents={extents})")
    if np.any(extents < 0.001):
        print(f"[⚠️] Mesh {path} may be too small (extents={extents})")

    all_meshes.append(mesh)
    # all_meshes[i].show()  # 첫 번째 메쉬부터 하나씩 띄워보며 위치 확인
# 하나의 씬으로 병합
print(all_meshes)
scene = trimesh.util.concatenate(all_meshes)

# 렌더링
scene.show()

# 저장 옵션
scene.export("summit_xl_assembled.obj")

# 🔽 OBJ 대신 TXT로 저장
txt_lines = []

# # Write vertices
# for v in scene.vertices:
#     txt_lines.append(f"v {v[0]} {v[1]} {v[2]}")

# # Write faces (OBJ indexing is 1-based)
# for f in scene.faces:
#     txt_lines.append(f"f {f[0]+1} {f[1]+1} {f[2]+1}")

# # 저장
# txt_path = "summit_xl_assembled.txt"
# with open(txt_path, "w") as f:
#     f.write("\n".join(txt_lines))

# print(f"[✅] 저장 완료: {txt_path}")

txt_path = "/home/chan/catkin_ws/src/summit_xl_assembled.txt"

# # Parse vertices and faces
# vertices = []
# faces = []

# with open(txt_path, "r") as f:
#     for line in f:
#         parts = line.strip().split()
#         if not parts:
#             continue
#         if parts[0] == 'v':
#             vertices.append([float(p) for p in parts[1:4]])
#         elif parts[0] == 'f':
#             # Convert from 1-based to 0-based indexing
#             faces.append([int(p) - 1 for p in parts[1:4]])

# # Create mesh
# mesh = trimesh.Trimesh(vertices=vertices, faces=faces)

# # Show mesh
# mesh.show()

# Write vertices with high precision and tab-separated values
for v in scene.vertices:
    txt_lines.append("\t".join(f"{coord:.14f}" for coord in v))

# Add face section
txt_lines.append("F")
for f in scene.faces:
    txt_lines.append("\t".join(str(idx) for idx in f))

# Save to new txt file
with open(txt_path, "w") as f:
    f.write("\n".join(txt_lines))