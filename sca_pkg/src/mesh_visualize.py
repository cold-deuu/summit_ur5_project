import open3d as o3d

src = "/home/chan/catkin_ws/src/robot_description/universal_robots_ur5e/assets/summit_xl_assembled.stl"

mesh = o3d.io.read_triangle_mesh(src)
mesh.compute_vertex_normals()

# 목표 삼각형 수: 180,000 (여유 있게 200k 미만)
target_faces = 180_000
mesh_decim = mesh.simplify_quadric_decimation(target_number_of_triangles=target_faces)

# OBJ로 저장(권장: 파서 안정적)
dst = src.replace(".stl", "_decim.obj")
o3d.io.write_triangle_mesh(dst, mesh_decim)
print("saved:", dst)