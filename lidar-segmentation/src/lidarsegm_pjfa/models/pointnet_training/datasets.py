import torch.utils.data
import time
import numpy as np
import random
import math
import os
import os.path as osp
import torch
from torch.utils.data import DataLoader
from struct import *
import sys
import re
import pypcd
from utils import get_cloud_size


class PointCloudDataset(torch.utils.data.Dataset):

    def __init__(self, kitti_dataset_root_path=None, stanford_dataset_root_path=None,
                 carla_dataset_root_path=None, modelnet_dataset_root_path=None,
                 test=False, num_points=2000, rotate=False, stretch=(), slide=(), dropout=0, jitter=0):

        self.label_dict = {'car': 0, 'pedestrian': 1, 'cyclist': 2, 'other': 3}
        if kitti_dataset_root_path == "" or kitti_dataset_root_path == "None":
            kitti_dataset_root_path = None
        if stanford_dataset_root_path == "" or stanford_dataset_root_path == "None":
            stanford_dataset_root_path = None
        if carla_dataset_root_path == "" or carla_dataset_root_path == "None":
            carla_dataset_root_path = None
        if modelnet_dataset_root_path == "" or modelnet_dataset_root_path == "None":
            modelnet_dataset_root_path = None

        self.avg_cloud_size = 0

        self.test = test
        self.num_points = num_points

        self.rotate = rotate
        self.stretch = stretch
        self.slide = slide
        self.dropout = dropout
        self.jitter = jitter

        print(
            '\nnum_points={}, test={}, rotate={}, stretch={}, slide={}, dropout={}, jitter={}'.format(num_points, test,
                                                                                                      rotate, stretch,
                                                                                                      slide, dropout,
                                                                                                      jitter))
        # Build path list
        self.input_clouds, self.input_labels, self.input_origins = self.create_input_list(kitti_dataset_root_path,
                                                                                          stanford_dataset_root_path,
                                                                                          carla_dataset_root_path,
                                                                                          modelnet_dataset_root_path)

        # get the number of occurences
        self.num_classes = len(set(self.input_labels))
        self.occurences_per_class = {}
        for i in range(self.num_classes):
            self.occurences_per_class[i] = self.input_labels.count(i)

        print('data: {}. avg_pc_size: {:.2f}, occurences per class: {}\n'.format(len(self), self.avg_cloud_size,
                                                                                 self.occurences_per_class))

    def __len__(self):
        return len(self.input_clouds)

    def __getitem__(self, idx):
        # Select the path
        path, label, origin = self.input_clouds[idx], self.input_labels[idx], self.input_origins[idx]

        # Parse the vertices from the file
        vertices = self.vertex_parser(path)

        # Augmentations
        vertices = self.augment_data(vertices, idx, origin)

        # Convert numpy format to torch variable
        return [torch.from_numpy(vertices).float(), label]

    def vertex_parser(self, path_to_file):

        vertices = []

        data = pypcd.point_cloud_from_path(path_to_file).pc_data
        for item in data:
            vertices.append([item[0], item[1], item[2]])

        vertices = np.asarray(vertices)
        return vertices.transpose(1, 0)

    def create_input_list(self, kitti_dataset_root_path=None, stanford_dataset_root_path=None,
                          carla_dataset_root_path=None, modelnet_dataset_root_path=None):
        pcd_paths = []
        pcd_labels = []
        pcd_origins = []

        # PARSING MODELNET
        if modelnet_dataset_root_path is not None and not self.test:
            classes = [f for f in os.listdir(osp.join(modelnet_dataset_root_path))]
            for class_ in classes:
                pcds = [osp.join(modelnet_dataset_root_path, class_, f)
                        for f in os.listdir(osp.join(modelnet_dataset_root_path, class_))]

                for pcd in pcds:
                    cloud_size = get_cloud_size(pcd)
                    if cloud_size > 30:
                        self.avg_cloud_size += cloud_size

                        pcd_paths.append(pcd)
                        pcd_labels.append(self.label_dict[class_.lower()])
                        pcd_origins.append('modelnet')

        # PARSING CARLA
        if carla_dataset_root_path is not None and not self.test:
            classes = [f for f in os.listdir(osp.join(carla_dataset_root_path))]
            for class_ in classes:
                pcds = [osp.join(carla_dataset_root_path, class_, f)
                        for f in os.listdir(osp.join(carla_dataset_root_path, class_))]

                for pcd in pcds:
                    cloud_size = get_cloud_size(pcd)
                    if cloud_size > 30:
                        self.avg_cloud_size += cloud_size

                        pcd_paths.append(pcd)
                        pcd_labels.append(self.label_dict[class_.lower()])
                        pcd_origins.append('carla')

        # PARSING STANFORD
        if stanford_dataset_root_path is not None and not self.test:
            scenes = os.listdir(stanford_dataset_root_path)

            for scene in scenes:
                pcds = [osp.join(stanford_dataset_root_path, scene, f)
                        for f in os.listdir(osp.join(stanford_dataset_root_path, scene))]

                for pcd in pcds:
                    cloud_size = get_cloud_size(pcd)
                    if cloud_size > 30:
                        self.avg_cloud_size += cloud_size

                        label = pcd.strip().split('/')[-1].split('_')[0]

                        # if label != 'background':
                        pcd_paths.append(pcd)
                        if label == 'background':
                            label = 'other'
                        if label == 'bicyclist':
                            label = 'cyclist'
                        pcd_labels.append(self.label_dict[label])
                        pcd_origins.append('stanford')

        # PARSING KITTI
        if kitti_dataset_root_path is not None:
            scenes = os.listdir(kitti_dataset_root_path)

            if self.test:
                scenes = ['0000', '0001']

            else:
                scenes.remove('0000')
                scenes.remove('0001')

            for scene in scenes:
                classes = [f for f in os.listdir(osp.join(kitti_dataset_root_path, scene, 'out')) if
                           not f.endswith('.pcd')]  # class folders

                for class_ in classes:

                    pcds = [osp.join(kitti_dataset_root_path, scene, 'out', class_, f) for f in
                            os.listdir(osp.join(kitti_dataset_root_path, scene, 'out', class_))]

                    for pcd in pcds:

                        cloud_size = get_cloud_size(pcd)
                        if cloud_size > 30:
                            self.avg_cloud_size += cloud_size

                            pcd_paths.append(pcd)
                            pcd_labels.append(self.label_dict[class_.lower()])
                            pcd_origins.append('kitti')

        else:
            sys.exit('ERROR: kitti_dataset_path is \"None\"')

        self.avg_cloud_size /= len(pcd_labels)

        return pcd_paths, pcd_labels, pcd_origins

    # Augmentations
    def augment_data(self, vertices, idx, origin):
        vertices = self.pc_normalize(vertices)
        vertices = self.pc_random_sample(vertices, idx)
        vertices = self.pc_shuffle(vertices, idx)

        # other augmentations
        if self.jitter != 0:
            vertices = self.pc_jitter(vertices, origin)

        if self.dropout != 0:
            vertices = self.pc_dropout(vertices, origin)

        if self.stretch != ():
            vertices = self.pc_stretch(vertices, origin)

        if self.slide != ():
            vertices = self.pc_slide(vertices, origin)

        if self.rotate:
            vertices = self.pc_random_rotate(vertices, idx)

        return vertices

    # Random sampling
    def pc_random_sample(self, pc, idx=None):
        if pc.shape[1] < self.num_points:
            pad = np.zeros((pc.shape[0], self.num_points - pc.shape[1]))
            pc = np.concatenate((pc, pad), 1)

        elif pc.shape[1] > self.num_points:
            if not self.test:
                random_idxs = np.random.choice(pc.shape[1], self.num_points, replace=True)
                pc = pc[:, random_idxs]

            else:
                np.random.seed(idx)
                random_idxs = np.random.choice(pc.shape[1], self.num_points, replace=True)
                pc = pc[:, random_idxs]
                np.random.seed()

        return pc

    # Normalize cloud
    def pc_normalize(self, pc):
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
    def pc_shuffle(self, pc, idx=None):
        if not self.test:
            shuffled_idxs = list(range(self.num_points))
            random.shuffle(shuffled_idxs)
            pc = pc[:, shuffled_idxs]
        else:
            shuffled_idxs = list(range(self.num_points))
            random.Random(idx).shuffle(shuffled_idxs)
            pc = pc[:, shuffled_idxs]

        return pc

    # Random rotation about the Y-axis
    def pc_random_rotate(self, pc, idx=None):

        rotation_angle = np.random.uniform(0, 1) * 2 * np.pi
        # if not self.test:
        #    rotation_angle = np.random.uniform(0, 1) * 2 * np.pi
        # else:
        #    rotation_angle = random.Random(idx).uniform(0, 1) * 2 * np.pi

        cosval = np.cos(rotation_angle)
        sinval = np.sin(rotation_angle)
        # rotation_matrix = np.array([[cosval, 0, sinval], [0, 1, 0], [-sinval, 0, cosval]])
        rotation_matrix = np.array([[cosval, -sinval, 0], [sinval, cosval, 0], [0, 0, 1]])

        pc = rotation_matrix @ pc
        return pc

    # Add Gaussian noise with standard deviation of 0.01
    def pc_jitter(self, pc, origin):
        #if origin in ['carla', 'modelnet']:
        #    pc += np.random.normal(loc=0.0, scale=self.jitter*2.0, size=pc.shape)
        #else:
        pc += np.random.normal(loc=0.0, scale=self.jitter, size=pc.shape)
        return pc

    # Randomly drop points
    def pc_dropout(self, pc, origin):
        rnd = np.random.rand(pc.shape[1])

        #if origin in ['carla', 'modelnet']:
        #    drop = rnd > self.dropout*0.9
        #else:
        #    drop = rnd > self.dropout
        drop = rnd > self.dropout

        for i in range(len(drop)):
            if drop[i]:
                pc[:, i] = pc[:, 0]

        return pc

    # Randomly stretch the cloud a little bit
    def pc_stretch(self, pc, origin):
        for i in range(pc.shape[0]):
            stretch_ratio = random.uniform(self.stretch[0], self.stretch[1])
            #if origin in ['carla', 'modelnet']:
            #    stretch_ratio = stretch_ratio * 1.5
            pc[i] = pc[i] * stretch_ratio

        return pc

    # Random slide
    def pc_slide(self, pc, origin):
        for i in range(pc.shape[0]):
            slide = random.uniform(self.slide[0], self.slide[1])
            #if origin in ['carla', 'modelnet']:
            #    slide = slide * 1.5
            pc[i] = pc[i] + slide

        return pc

# # ALL DATA
# dataset_train = PointCloudDataset(kitti_dataset_path='/home/hsk2bp/datasets/lidar_data/kitti',
#                                    stanford_dataset_path='/home/hsk2bp/datasets/stanford_pcds',
#                                    carla_dataset_path = '/home/hsk2bp/datasets/carla_and_modelnet40_pcds/Carla',
#                                    modelnet_dataset_path = '/home/hsk2bp/datasets/carla_and_modelnet40_pcds/ModelNet',
#                                    test=False)
# dataset_test = PointCloudDataset(kitti_dataset_path='/home/hsk2bp/datasets/lidar_data/kitti',
#                                    stanford_dataset_path='/home/hsk2bp/datasets/stanford_pcds',
#                                    carla_dataset_path = '/home/hsk2bp/datasets/carla_and_modelnet40_pcds/Carla',
#                                    modelnet_dataset_path = '/home/hsk2bp/datasets/carla_and_modelnet40_pcds/ModelNet',
#                                    test=True)

# # SIMULATED ONLY
# dataset_train = PointCloudDataset(kitti_dataset_path=None,
#                                    stanford_dataset_path=None,
#                                    carla_dataset_path = '/home/hsk2bp/datasets/carla_and_modelnet40_pcds/Carla',
#                                    modelnet_dataset_path = '/home/hsk2bp/datasets/carla_and_modelnet40_pcds/ModelNet',
#                                    test=False)
#
#
# data_loader = DataLoader(dataset_test, batch_size=10, shuffle=True, num_workers=0)
# for sample in data_loader:
#     print(sample[0].size(), sample[1].size())
#     print(sample[0], sample[1])
#     break
