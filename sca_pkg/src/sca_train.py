# %% Load collision dataset
import time
import torch
import numpy as np
from sklearn.datasets import load_svmlight_file
from sklearn.model_selection import train_test_split
from torch.utils.data import TensorDataset, DataLoader

# Chan
import rospkg
import json


def convert_to_libsvm(input_txt, output_txt):
    with open(input_txt, 'r') as fin, open(output_txt, 'w') as fout:
        for line in fin:
            parts = line.strip().split()
            label = parts[0]
            features = parts[1:]
            feature_str = ' '.join([f"{i+1}:{v}" for i, v in enumerate(features)])
            fout.write(f"{label} {feature_str}\n")

# CUDA
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


rospack = rospkg.RosPack()
package_path = rospack.get_path('sca_pkg')
package_to_data = "/src/data/data_500.txt"
package_to_data2 = "/src/data/data_500_svm2.txt"
convert_to_libsvm(package_path+package_to_data, package_path+package_to_data2)

data_train = load_svmlight_file(package_path+package_to_data2)
x_tens_train = torch.Tensor(data_train[0].toarray())
y_tens_train = torch.Tensor(data_train[1])

for cv in range(1): #increase the number of iterations to perform crossvalidation
    X_train, X_test, y_train, y_test = train_test_split(x_tens_train, y_tens_train, test_size=0.05555)
    x_tens_train = torch.Tensor(X_train)
    y_tens_train = torch.Tensor(y_train)
    x_tens_test = torch.Tensor(X_test)
    y_tens_test = torch.Tensor(y_test)


    train_dataset = TensorDataset(X_train, y_train.to(dtype=torch.long))
    train_loader = DataLoader(train_dataset, batch_size=1024, shuffle=True)
    # print("Train DataSet : ", train_dataset)
    # print(train_dataset.tensors[0].shape)  # X 데이터
    # print(train_dataset.tensors[1].shape)  # y 데이터

    # print(train_dataset.tensors[0][0])     # 첫 번째 샘플의 feature
    # print(train_dataset.tensors[1][0])     # 첫 번째 샘플의 label


    # print(train_loader)
    # x_batch, y_batch = next(iter(train_loader))
    # print(x_batch)
    # print(y_batch)

    # %% NN definition
    import torch.nn as nn
    import torch.nn.functional as F
    
    class Net(nn.Module):
        def __init__(self):
            super(Net, self).__init__()
            self.l1 = nn.Linear(6, 128)
            self.l2 = nn.Linear(128, 128)
            self.l3 = nn.Linear(128, 64)
            self.l4 = nn.Linear(64, 32)
            self.l5 = nn.Linear(32, 2)
            self.act = nn.Tanh()
            self.lsm = nn.LogSoftmax()
            self.sm = nn.Softmax(dim=-1)
        def forward(self, x):
            x = self.l1(x)
            x = self.act(x)
            x = self.l2(x)
            x = self.act(x)
            x = self.l3(x)
            x = self.act(x)
            x = self.l4(x)
            x = self.act(x)
            x = self.l5(x)
            # x = self.lsm(x)
            return x
    
    net = Net()
    
    print(net)
    params = list(net.parameters())
    print(len(params))
    print(params[0].size())
    from scipy.io import loadmat, savemat

    # %% Loss fcn
    import torch.optim as optim
    
    criterion = nn.CrossEntropyLoss()
    # criterion = nn.NLLLoss()
    
    optimizer = optim.Rprop(net.parameters())
    
    #optimizer = optim.Adadelta(net.parameters())
    #optimizer = optim.SGD(net.parameters(),lr = 0.1, momentum = 0.9)
    # inputs = x_tens_train[0]
    # labels = y_tens_train[0]
    # outputs = net(inputs)
    # loss = criterion(outputs, labels)
    # %% Training
    net.to(device)
    x_gpu = x_tens_train.to(device)
    y_gpu = y_tens_train.to(device = device, dtype=torch.int64)

    print(y_tens_train.min(), y_tens_train.max())    
    t = time.time()
    for epoch in range(1000):  # loop over the dataset multiple times, 5000 for the best accuracy
        running_loss = 0.0
    
        #print(labels)
        # zero the parameter gradients
        inputs, labels = x_gpu, y_gpu
        optimizer.zero_grad()

        # forward + backward + optimize
        outputs = net(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()
        # print statistics
        running_loss += loss.item()
        
        print('Epoch %d, Loss: %.3f' %
              (epoch + 1, running_loss))
        running_loss = 0.0
    


    print('Finished Training')
    elapsed = time.time() - t
    print('Time:%5.1f seconds' % elapsed)

    # t = time.time()
    # for epoch in range(5000):  # epoch 수는 상황에 맞게 조절
    #     running_loss = 0.0
    #     for i, (inputs, labels) in enumerate(train_loader):

    #         inputs = inputs.to(device)
    #         labels = labels.to(device).long()

    #         optimizer.zero_grad()
    #         outputs = net(inputs)
    #         loss = criterion(outputs, labels)
    #         loss.backward()
    #         optimizer.step()

    #         running_loss += loss.item()

    #     print('Epoch %d, Loss: %.3f' % (epoch + 1, running_loss))
    # print('Finished Training')
    # elapsed = time.time() - t
    # print('Time:%5.1f seconds' % elapsed)

    # %% save model
    #PATH = './cifar_net.pth'
    #torch.save(net.state_dict(), PATH)
    
    # %% verification
    postures = x_tens_test.to(device)
    labels = y_tens_test.to(device = device, dtype=torch.int64)
    net.to('cpu')
    postures = x_tens_test
    labels = y_tens_test.to(dtype=torch.int64)
    with torch.no_grad():
        outputs = net(postures)
        
        _, predicted = torch.max(outputs.data,1)

        total = labels.size(0)
        correct = (predicted == labels).sum().item()
    print('Accuracy of the network on the test dataset: %.5f %%' % (
        100 * correct / total))

    # %% num weights
    total_sz = 0
    
    savedict = dict()
    for key in net.state_dict().keys():
        total_sz += np.prod(net.state_dict()[key].size())
        savedict[key.replace(".", "_")]=net.state_dict()[key].cpu().numpy()
        
    savemat(package_path + "/src/nn_mm_100.mat", savedict)
    print("Model weights saved to nn_mm_100.mat")

    for key in net.state_dict().keys():
        weight = net.state_dict()[key].cpu().numpy()
        weight.astype(np.float32).tofile(f"{key.replace('.', '_')}.bin")
