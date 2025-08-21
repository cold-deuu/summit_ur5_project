import trimesh

def load_custom_mesh(txt_path):
    with open(txt_path, 'r') as f:
        lines = f.readlines()

    vertices = []
    faces = []
    mode = None

    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line == 'V':
            mode = 'vertices'
            continue
        elif line == 'F':
            mode = 'faces'
            continue
        elif line.isdigit() or line.isalpha():
            # Ignore headers like "0" or "base_link"
            continue

        parts = line.split()
        if mode == 'vertices':
            vertices.append([float(p) for p in parts])
        elif mode == 'faces':
            faces.append([int(p) for p in parts])

    return trimesh.Trimesh(vertices=vertices, faces=faces)

# 예시 경로 (수정 가능)
txt_path = "/home/chan/catkin_ws/src/summit_xl_assembled.txt"

# 메쉬 로드 및 시각화
mesh = load_custom_mesh(txt_path)
mesh.show()
