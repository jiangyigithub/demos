import torch
import torch.nn as nn
import torch.nn.parallel
import torch.utils.data
from torch.autograd import Variable
import numpy as np
import torch.nn.functional as F
import torch.autograd as grad



class STN3d(nn.Module):
    def __init__(self):
        super(STN3d, self).__init__()
        self.conv1 = torch.nn.Conv1d(3, 64, 1)
        self.conv2 = torch.nn.Conv1d(64, 128, 1)
        self.conv3 = torch.nn.Conv1d(128, 1024, 1)
        self.fc1 = nn.Linear(1024, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, 9)
        self.relu = nn.ReLU()

        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(1024)
        self.bn4 = nn.BatchNorm1d(512)
        self.bn5 = nn.BatchNorm1d(256)


    def forward(self, x):
        batchsize = x.size()[0]
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 1024)

        x = F.relu(self.bn4(self.fc1(x)))
        x = F.relu(self.bn5(self.fc2(x)))
        x = self.fc3(x)

        iden = Variable(torch.from_numpy(np.array([1,0,0,0,1,0,0,0,1]).astype(np.float32))).view(1,9).repeat(batchsize,1)
        if x.is_cuda:
            iden = iden.cuda()
        x = x + iden
        x = x.view(-1, 3, 3)
        return x


class PointNetfeat(nn.Module):
    def __init__(self, global_feat = True):
        super(PointNetfeat, self).__init__()
        self.stn = STN3d()
        self.conv1 = torch.nn.Conv1d(3, 64, 1)
        self.conv2 = torch.nn.Conv1d(64, 128, 1)
        self.conv3 = torch.nn.Conv1d(128, 512, 1) #torch.nn.Conv1d(128, 1024, 1)
        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(512) #nn.BatchNorm1d(1024)
        self.global_feat = global_feat

    def forward(self, x):
        batchsize = x.size()[0]
        n_pts = x.size()[2]
        trans = self.stn(x)
        x = x.transpose(2,1)
        x = torch.bmm(x, trans)
        x = x.transpose(2,1)
        x = F.relu(self.bn1(self.conv1(x)))
        pointfeat = x
        x = F.relu(self.bn2(self.conv2(x)))
        x = self.bn3(self.conv3(x))
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 512) #x.view(-1, 1024)
        if self.global_feat:
            return x, trans
        else:
            x = x.view(-1, 512, 1).repeat(1, 1, n_pts) # x.view(-1, 1024, 1).repeat(1, 1, n_pts)
            return torch.cat([x, pointfeat], 1), trans


class PointNetClassifier(nn.Module):
    def __init__(self, k = 3):
        super(PointNetClassifier, self).__init__()

        self.num_classes = k

        self.feat = PointNetfeat(global_feat=True)
        self.fc1 = nn.Linear(512, 256) # nn.Linear(1024, 512)
        self.fc2 = nn.Linear(256, 128) # nn.Linear(512, 256)
        self.fc3 = nn.Linear(128, k) #nn.Linear(256, k)
        self.bn1 = nn.BatchNorm1d(256) #nn.BatchNorm1d(512)
        self.bn2 = nn.BatchNorm1d(128) #nn.BatchNorm1d(256)
        self.relu = nn.ReLU()

    def forward(self, x):
        x, trans = self.feat(x)
        x = F.relu(self.bn1(self.fc1(x)))
        x = F.relu(self.bn2(self.fc2(x)))
        x = self.fc3(x)
        #return F.log_softmax(x, dim=0), trans
        return x, trans

    def run_train(self, points, target, loss_fn, optimizer):
        self.train()

        points = grad.Variable(points).float().cuda()
        target = grad.Variable(target).cuda()

        # Zero out the gradients
        optimizer.zero_grad()

        # Forward pass
        outp, _ = self(points)

        # compute my error
        error = loss_fn(outp, target)

        # Backpropagate
        error.backward()
        # Update the weights
        optimizer.step()

        return error

    def run_eval(self, points, target, loss_fn):
        self.eval()

        points = grad.Variable(points).float().cuda()
        target = grad.Variable(target).cuda()

        # Forward pass
        outp, _ = self(points)

        # compute error and acc
        error = loss_fn(outp, target)
        _, preds = torch.max(F.softmax(outp, dim=1), 1)
        batch_acc = torch.sum(preds == target.data).item() / target.size()[0]

        conf_matrix = np.zeros((self.num_classes, self.num_classes), dtype=np.int)
        for i in range(self.num_classes):
            for j in range(self.num_classes):
                # predicted to be i
                pred_i = preds == i

                # is actually j
                gt_j = target.data == j

                for k in range(pred_i.size()[0]):
                    if pred_i[k].item() == 1 and gt_j[k].item() == 1:
                        conf_matrix[i, j] += 1

        return error, batch_acc, conf_matrix


class PointNetDenseCls(nn.Module):
    def __init__(self, k = 2):
        super(PointNetDenseCls, self).__init__()
        self.k = k
        self.feat = PointNetfeat(global_feat=False)
        self.conv1 = torch.nn.Conv1d(1088, 512, 1)
        self.conv2 = torch.nn.Conv1d(512, 256, 1)
        self.conv3 = torch.nn.Conv1d(256, 128, 1)
        self.conv4 = torch.nn.Conv1d(128, self.k, 1)
        self.bn1 = nn.BatchNorm1d(512)
        self.bn2 = nn.BatchNorm1d(256)
        self.bn3 = nn.BatchNorm1d(128)

    def forward(self, x):
        batchsize = x.size()[0]
        n_pts = x.size()[2]
        x, trans = self.feat(x)
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = self.conv4(x)
        x = x.transpose(2,1).contiguous()
        x = F.log_softmax(x.view(-1,self.k), dim=-1)
        x = x.view(batchsize, n_pts, self.k)
        return x, trans


    def run_train(self, points, target, loss_fn, optimizer):
        self.train()

        points = grad.Variable(points).float().cuda()
        target = grad.Variable(target).cuda()

        # Zero out the gradients
        optimizer.zero_grad()

        # Forward pass
        outp, _ = self(points)

        # compute my error
        error = loss_fn(outp, target)

        # Backpropagate
        error.backward()
        # Update the weights
        optimizer.step()

        return error

    def run_eval(self, points, target, loss_fn):
        self.eval()

        points = grad.Variable(points).float().cuda()
        target = grad.Variable(target).cuda()

        # Forward pass
        outp, _ = self(points)

        # compute error and acc
        error = loss_fn(outp, target)
        _, preds = torch.max(F.softmax(outp, dim=1), 1)
        batch_acc = torch.sum(preds == target.data).item() / target.size()[0]

        conf_matrix = np.zeros((self.num_classes, self.num_classes), dtype=np.int)
        for i in range(self.num_classes):
            for j in range(self.num_classes):
                # predicted to be i
                pred_i = preds == i

                # is actually j
                gt_j = target.data == j

                for k in range(pred_i.size()[0]):
                    if pred_i[k].item() == 1 and gt_j[k].item() == 1:
                        conf_matrix[i, j] += 1

        return error, batch_acc, conf_matrix



'''
if __name__ == '__main__':
    sim_data = Variable(torch.rand(32,3,2500))
    trans = STN3d()
    out = trans(sim_data)
    print('stn', out.size())

    pointfeat = PointNetfeat(global_feat=True)
    out, _ = pointfeat(sim_data)
    print('global feat', out.size())

    pointfeat = PointNetfeat(global_feat=False)
    out, _ = pointfeat(sim_data)
    print('point feat', out.size())

    cls = PointNetCls(k = 5)
    out, _ = cls(sim_data)
    print('class', out.size())

    seg = PointNetDenseCls(k = 3)
    out, _ = seg(sim_data)
    print('seg', out.size())
'''