from torch.nn.modules.loss import _Loss
import torch.nn.functional as F
import torch
import torch.autograd as grad


class Weighted_CrossEntropyLoss(_Loss):
    def forward(self, output, target):
        #pred_loss = F.cross_entropy(output, target, torch.FloatTensor([0.719, 1.641, 12.263, 0.452]).cuda()) #kitti only
        pred_loss = F.cross_entropy(output, target, torch.FloatTensor([0.748, 1.508, 5.6, 0.341]).cuda()) #kitti & stanford

        return pred_loss


class CrossEntropyLoss(_Loss):
    def forward(self, output, target):
        pred_loss = F.cross_entropy(output, target)

        return pred_loss


class Weighted_CrossEntropyLoss_with_Reg(_Loss):
    def forward(self, output, T2, phase, target):
        #pred_loss = F.cross_entropy(output, target, torch.FloatTensor([0.719, 1.641, 12.263, 0.452]).cuda()) #kitti only
        pred_loss = F.cross_entropy(output, target, torch.FloatTensor([0.748, 1.508, 5.6, 0.341]).cuda()) #kitti & stanford

        if phase == 'train':
            # Identity matrix for enforcing orthogonality of second transform
            identity = grad.Variable(torch.eye(64).float().cuda(), requires_grad=False)

            reg_loss = F.mse_loss(torch.bmm(T2, T2.permute(0, 2, 1)), identity.expand(T2.shape[0], -1, -1))
            return pred_loss + reg_loss * 0.001
        else:
            return pred_loss


class CrossEntropyLoss_with_Reg(_Loss):
    def forward(self, output, T2, phase, target):
        pred_loss = F.cross_entropy(output, target)

        if phase == 'train':
            # Identity matrix for enforcing orthogonality of second transform
            identity = grad.Variable(torch.eye(64).float().cuda(), requires_grad=False)

            reg_loss = F.mse_loss(torch.bmm(T2, T2.permute(0, 2, 1)), identity.expand(T2.shape[0], -1, -1))
            return pred_loss + reg_loss * 0.001
        else:
            return pred_loss
