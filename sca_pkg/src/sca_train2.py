# %% Load collision dataset
import time
import torch
import numpy as np
from sklearn.datasets import load_svmlight_file
from sklearn.model_selection import train_test_split
import rospkg
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from scipy.io import savemat

# CUDA
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

# ROS package path
rospack = rospkg.RosPack()
package_path = rospack.get_path('sca_pkg')
data_txt = "/src/data/data_10_svm.txt"
data_path = package_path + data_txt

# Load libsvm formatted dataset
data = load_svmlight_file(data_path)
x_all = torch.Tensor(data[0].toarray())           # shape: [N, 7]
y_raw = torch.Tensor(data[1])                     # shape: [N]

# Train/test split
X_train, X_test, y_train_raw, y_test_raw = train_test_split(x_all, y_raw, test_size=0.05555)

# One-hot 변환: 0 → [0, 1], 1 → [1, 0]
def to_onehot(y_tensor):
    return torch.zeros(y_tensor.size(0), 2).scatter_(1, (1 - y_tensor.long()).unsqueeze(1), 1)

y_train = to_onehot(y_train_raw)
y_test = to_onehot(y_test_raw)

# Tensor 준비
x_tens_train = torch.Tensor(X_train)
x_tens_test = torch.Tensor(X_test)

# %% Define Network
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.l1 = nn.Linear(7, 50)
        self.l2 = nn.Linear(50, 30)
        self.l3 = nn.Linear(30, 10)
        self.l4 = nn.Linear(10, 2)
        self.act = nn.Tanh()

    def forward(self, x):
        x = self.act(self.l1(x))
        x = self.act(self.l2(x))
        x = self.act(self.l3(x))
        x = torch.sigmoid(self.l4(x))  # 확률로 해석 가능
        return x

net = Net()
net.to(device)
print(net)

# %% Loss + Optimizer
criterion = nn.BCELoss()  # binary cross-entropy with one-hot
optimizer = optim.Rprop(net.parameters())

# GPU 할당
x_gpu = x_tens_train.to(device)
y_gpu = y_train.to(device)

# %% Training loop
start_time = time.time()
for epoch in range(1000):
    optimizer.zero_grad()
    outputs = net(x_gpu)
    loss = criterion(outputs, y_gpu)
    loss.backward()
    optimizer.step()

    print(f"Epoch {epoch+1}, Loss: {loss.item():.4f}")

print("Finished Training. Time elapsed: %.2fs" % (time.time() - start_time))

# %% Evaluation
net.to('cpu')
with torch.no_grad():
    preds = net(x_tens_test)
    pred_labels = torch.argmax(preds, dim=1)
    true_labels = torch.argmax(y_test, dim=1)
    acc = (pred_labels == true_labels).float().mean().item()
    print(f"Test Accuracy: {acc * 100:.2f}%")

# %% Save weights as .mat
savedict = {
    k.replace('.', '_'): v.cpu().numpy()
    for k, v in net.state_dict().items()
}
savemat(package_path + "/src/nn_py_bce.mat", savedict)
print("Model weights saved to nn_py_bce.mat")