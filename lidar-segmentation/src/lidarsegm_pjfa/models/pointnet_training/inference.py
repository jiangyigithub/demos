from models import pointnet_classifier_git0, pointnet_classifier_git1, pointnet_classifier_skinny, pointnet_classifier_extra_skinny
import torch
import os
import time
import numpy as np
import os.path as osp
import pypcd
from utils import get_cloud_size
import random
import torch.nn.functional as F

label_dict = {'car': 0, 'pedestrian': 1, 'cyclist': 2, 'other': 3}
rev_label_dict = {0:'car', 1:'pedestrian', 2:'cyclist', 3:'other'}

def vertex_parser(path_to_file):
    vertices = []

    data = pypcd.point_cloud_from_path(path_to_file).pc_data
    for item in data:
        vertices.append([item[0], item[1], item[2]])

    vertices = np.asarray(vertices)
    return vertices.transpose(1, 0)



# Random sampling
def pc_random_sample(pc, num_points, idx=None,):
    np.random.seed(idx)
    random_idxs = np.random.choice(pc.shape[1], num_points, replace=True)
    pc = pc[:, random_idxs]
    np.random.seed()

    return pc


# Normalize cloud
def pc_normalize(pc):
    # zero mean
    centroid = np.mean(pc, axis=1)
    pc[0] = pc[0] - centroid[0]
    pc[1] = pc[1] - centroid[1]
    pc[2] = pc[2] - centroid[2]

    # between -1 and 1
    x_dist = abs(np.max(pc[0]) - np.min(pc[0]))
    y_dist = abs(np.max(pc[1]) - np.min(pc[1]))
    z_dist = abs(np.max(pc[2]) - np.min(pc[2]))

    max_dist = np.max([x_dist, y_dist, z_dist])

    pc = pc / max_dist
    return pc


# Random shuffle
def pc_shuffle(pc, num_points, idx=None):
    shuffled_idxs = list(range(num_points))
    random.Random(idx).shuffle(shuffled_idxs)
    pc = pc[:, shuffled_idxs]

    return pc


# collect pcd paths
data_dir = '../pointnet_inference/to_check'
pcds = os.listdir(data_dir)
pcds.sort()
#print(pcds)


# load model
model = pointnet_classifier_skinny.PointNetClassifier(k=4)
model.load_state_dict(torch.load('logs/medium_augs_kittiford_celoss_skinny_pointnet_256pts_2018-11-14_11_46_28_836122/weights_107678.pkl'))
model.eval()
model.cuda()


label_cntr = [0,0,0,0]

# for each pcd
for idx, pcd_name in enumerate(pcds):
    # check cloud size
    cloud_size = get_cloud_size(osp.join(data_dir,pcd_name))
    if cloud_size >= 30:
        vertices = vertex_parser(osp.join(data_dir,pcd_name))
        cloud_shape = vertices.shape

        vertices = pc_random_sample(vertices, 256, idx)
        vertices = pc_normalize(vertices)
        vertices = pc_shuffle(vertices, 256, idx)

        #print(vertices)

        vertices = np.expand_dims(vertices, 0)
        vertices = torch.from_numpy(vertices)


        #print(vertices)
        vertices = vertices.cuda()

        outp, _ = model(vertices)

        _, pred = torch.max(F.softmax(outp, dim=1), 1)

        pred_label = pred.cpu().numpy()[0]
        text_label = rev_label_dict[pred_label]
        label_cntr[pred_label]+=1

        print('name: {}, size: {}, network_output: {}, pred: {}'
              .format(pcd_name, cloud_shape, outp.cpu().data.numpy(), text_label))
        break

print(label_cntr)


