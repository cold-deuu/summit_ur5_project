import numpy as np
import os
from glob import glob

def convert_bin_to_txt(bin_file, txt_file, shape):
    data = np.fromfile(bin_file, dtype=np.float32)
    data = data.reshape(shape)
    np.savetxt(txt_file, data, fmt="%.6f")
    print(f"[OK] {os.path.basename(bin_file)} → {os.path.basename(txt_file)} (shape={shape})")

# 절대 경로 설정
BIN_DIR = "/home/chan/catkin_ws/src/sca_pkg/result"
TXT_DIR = "/home/chan/catkin_ws/src/sca_pkg/result/result_txt"
# 결과 디렉토리가 없다면 생성
os.makedirs(TXT_DIR, exist_ok=True)

# 레이어별 shape 정의 (PyTorch의 [out_features, in_features])
layer_shapes = {
    "l1_weight": (128, 6),
    "l2_weight": (128, 128),
    "l3_weight": (64, 128),
    "l4_weight": (32, 64),
    "l5_weight": (2, 32),
    "l1_bias": (128,),
    "l2_bias": (128,),
    "l3_bias": (64,),
    "l4_bias": (32,),
    "l5_bias": (2,),
}

# 변환 실행
if __name__ == "__main__":
    for bin_path in glob(os.path.join(BIN_DIR, "*.bin")):
        base = os.path.splitext(os.path.basename(bin_path))[0]  # ex: "l1_weight"
        if base in layer_shapes:
            txt_path = os.path.join(TXT_DIR, base + ".txt")
            convert_bin_to_txt(bin_path, txt_path, shape=layer_shapes[base])
        else:
            print(f"[WARN] Unknown shape for: {base}")


# import torch
# import torch.nn as nn
# import torch.optim as optim
# import matplotlib.pyplot as plt

# # 1. 데이터 생성
# N = 1000
# x = torch.rand(N, 2) * 2 * torch.pi  # [0, 2π] 범위
# y = torch.stack([torch.sin(x[:, 0]), torch.cos(x[:, 1])], dim=1)  # [sin(x1), cos(x2)]

# # 2. MLP 모델 정의
# class MLP(nn.Module):
#     def __init__(self):
#         super().__init__()
#         self.net = nn.Sequential(
#             nn.Linear(2, 64),
#             nn.ReLU(),
#             nn.Linear(64, 64),
#             nn.ReLU(),
#             nn.Linear(64, 2)  # y1, y2 회귀 출력
#         )

#     def forward(self, x):
#         return self.net(x)

# # 3. 학습 준비
# model = MLP()
# criterion = nn.MSELoss()
# optimizer = optim.Adam(model.parameters(), lr=1e-3)

# # 4. 학습 루프
# epochs = 2000
# for epoch in range(epochs):
#     pred = model(x)
#     loss = criterion(pred, y)

#     optimizer.zero_grad()
#     loss.backward()
#     optimizer.step()

#     if epoch % 200 == 0:
#         print(f"Epoch {epoch}, Loss: {loss.item():.6f}")

# # 5. 평가 (시각화)
# x_test = torch.linspace(0, 2 * torch.pi, 100)
# x1_test = torch.stack([x_test, torch.zeros_like(x_test)], dim=1)
# x2_test = torch.stack([torch.zeros_like(x_test), x_test], dim=1)

# with torch.no_grad():
#     y1_pred = model(x1_test)[:, 0]
#     y2_pred = model(x2_test)[:, 1]

# plt.figure(figsize=(10, 4))

# plt.subplot(1, 2, 1)
# plt.plot(x_test.numpy(), torch.sin(x_test).numpy(), label='True sin(x1)')
# plt.plot(x_test.numpy(), y1_pred.numpy(), '--', label='Predicted y1')
# plt.title('y1 = sin(x1)')
# plt.legend()

# plt.subplot(1, 2, 2)
# plt.plot(x_test.numpy(), torch.cos(x_test).numpy(), label='True cos(x2)')
# plt.plot(x_test.numpy(), y2_pred.numpy(), '--', label='Predicted y2')
# plt.title('y2 = cos(x2)')
# plt.legend()

# plt.tight_layout()
# plt.show()
