import trimesh
import numpy as np
def create_axis(length=0.1, radius=0.002):
    axes = []

    # X-axis (red)
    x_cyl = trimesh.creation.cylinder(radius=radius, height=length, sections=20)
    x_cyl.apply_translation([length / 2, 0, 0])
    x_cyl.visual.face_colors = [255, 0, 0, 255]
    axes.append(x_cyl)

    # Y-axis (green)
    y_cyl = trimesh.creation.cylinder(radius=radius, height=length, sections=20)
    y_cyl.apply_translation([0, length / 2, 0])
    y_cyl.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0,1,0]))
    y_cyl.visual.face_colors = [0, 255, 0, 255]
    axes.append(y_cyl)

    # Z-axis (blue)
    z_cyl = trimesh.creation.cylinder(radius=radius, height=length, sections=20)
    z_cyl.apply_translation([0, 0, length / 2])
    z_cyl.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1,0,0]))
    z_cyl.visual.face_colors = [0, 0, 255, 255]
    axes.append(z_cyl)

    return axes

def load_mesh_from_txt(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    vertices = []
    faces = []
    mode = None

    for line in lines:
        line = line.strip()
        if line == 'V':
            mode = 'vertex'
            continue
        elif line == 'F':
            mode = 'face'
            continue

        if mode == 'vertex':
            x, y, z = map(float, line.split())
            vertices.append([x, y, z])
        elif mode == 'face':
            i, j, k = map(int, line.split())
            faces.append([i, j, k])

    vertices = np.array(vertices)
    faces = np.array(faces)

    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    return mesh

# 파일 로드 및 시각화
mesh0 = load_mesh_from_txt('/home/chan/catkin_ws/src/sca_pkg/new_mesh/new_mesh0.txt')
mesh1 = load_mesh_from_txt('/home/chan/catkin_ws/src/sca_pkg/new_mesh/new_mesh1.txt')
mesh2 = load_mesh_from_txt('/home/chan/catkin_ws/src/sca_pkg/new_mesh/new_mesh2.txt')
mesh3 = load_mesh_from_txt('/home/chan/catkin_ws/src/sca_pkg/new_mesh/new_mesh3.txt')
mesh4 = load_mesh_from_txt('/home/chan/catkin_ws/src/sca_pkg/new_mesh/new_mesh4.txt')
mesh5 = load_mesh_from_txt('/home/chan/catkin_ws/src/sca_pkg/new_mesh/new_mesh5.txt')
mesh6 = load_mesh_from_txt('/home/chan/catkin_ws/src/sca_pkg/new_mesh/new_mesh6.txt')
mesh7 = load_mesh_from_txt('/home/chan/catkin_ws/src/sca_pkg/new_mesh/new_mesh7.txt')
scene = trimesh.Scene()
scene.add_geometry(mesh0)
scene.add_geometry(mesh1)
scene.add_geometry(mesh2)
scene.add_geometry(mesh3)
scene.add_geometry(mesh4)
scene.add_geometry(mesh5)
scene.add_geometry(mesh6)
scene.add_geometry(mesh7)
# for axis in create_axis():
#     scene.add_geometry(axis)

scene.show()