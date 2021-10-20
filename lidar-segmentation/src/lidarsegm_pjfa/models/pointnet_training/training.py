import argparse
import os
from neurAD.pytorch.examples.utils import save_params, get_timestamp
from NN import Optimus
import torch
from losses import loss
from datasets import PointCloudDataset
from torch.utils.data import DataLoader
from models import pointnet_classifier_git0, pointnet_classifier_git1, pointnet_classifier_skinny
from models import pointnet_classifier_extra_skinny, pointnet_classifier_extra_skinny_and_short
from models import pointnet_classifier_anorexic

def get_data(args):
    # Instantiate a dataset loader
    model_net_train = PointCloudDataset(args.kitti_data, args.stanford_data, args.carla_data, args.modelnet_data,
                                        test=False, num_points=args.num_points, jitter=args.jitter,
                                        stretch=args.stretch, slide=args.slide, dropout=args.dropout,
                                        rotate=args.train_rotate)

    model_net_val = PointCloudDataset(args.kitti_data, args.stanford_data, args.carla_data, args.modelnet_data,
                                      test=True, num_points=args.num_points, jitter=0, stretch=(), slide=(),
                                      dropout=0, rotate=args.test_rotate)

    # The neurAD always need both the test and validation datasets
    # To keep it simple the test and validation sets are the same
    loaders = {}
    loaders['train'] =  torch.utils.data.DataLoader(model_net_train, batch_size=args.train_batch_size,
                                                    shuffle=True, num_workers=args.num_workers)
    loaders['validation'] = torch.utils.data.DataLoader(model_net_val, batch_size=args.test_batch_size,
                                                        shuffle=True, num_workers=args.num_workers)
    loaders['test'] = loaders['validation']

    return loaders


def get_model(args):
    dims = 3
    num_classes = 4

    if args.model == "git0":
        print("git0 model")
        model = pointnet_classifier_git0.PointNetClassifier(num_points=args.num_points, K=dims, num_classes=num_classes).train().cuda()
    elif args.model == "git1":
        print("git1 model")
        model = pointnet_classifier_git1.PointNetClassifier(num_classes).train().cuda()
    elif args.model == "skinny":
        print("skinny model")
        model = pointnet_classifier_skinny.PointNetClassifier(num_classes).train().cuda()
    elif args.model == "extra_skinny":
        print("extra skinny model")
        model = pointnet_classifier_extra_skinny.PointNetClassifier(num_classes).train().cuda()
    elif args.model == "extra_skinny_and_short":
        print("extra skinny and short model")
        model = pointnet_classifier_extra_skinny_and_short.PointNetClassifier(num_classes).train().cuda()
    elif args.model == "anorexic":
        print("anorexic model")
        model = pointnet_classifier_anorexic.PointNetClassifier(num_classes).train().cuda()


    return model


def get_loss(args):
    if args.weighted_loss and args.reg_loss:
        loss_fn = loss.Weighted_CrossEntropyLoss_with_Reg()
    elif args.weighted_loss and not args.reg_loss:
        loss_fn = loss.Weighted_CrossEntropyLoss()
    elif not args.weighted_loss and args.reg_loss:
        loss_fn = loss.CrossEntropyLoss_with_Reg()
    elif not args.weighted_loss and not args.reg_loss:
        loss_fn = loss.CrossEntropyLoss()

    print(loss_fn)
    return loss_fn


def train_new(args=None):

    logdir = os.path.join(args.log_dir, args.run_name + get_timestamp())
    if not os.path.isdir(logdir):
        os.makedirs(logdir)
    save_params(logdir, args)

    # Create model based on
    model = get_model(args)

    # Get loaders
    loaders = get_data(args)

    # Loss
    loss_fn = get_loss(args)

    # Evaluation freqs
    batch_num = len(loaders["train"])
    evaluation_frequencies = {
        "validation": max(int(batch_num / args.valid_freq), 1),
        "save": max(int(batch_num / args.save_freq), 1),
        "log": max(int(batch_num / args.log_freq), 1)
    }

    # Init trainer
    deep = Optimus(model=model, data_loaders=loaders, log_dir=logdir, optimizer=args.optimizer,
                   learning_rate=args.learning_rate, loss_fn=loss_fn, eval_freq=evaluation_frequencies,
                   stopper_patience=args.stopper_patience, histogram_save_freq=None, multi_gpu=False, easy_mode=False)

    # Train NN
    deep.train(epoch=args.num_epochs)


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Train Pointnet')

    parser.add_argument("--run_name", default="medium_augs_alldata_celoss_anorexic_pointnet_256pts")

    # model
    parser.add_argument("--model", default="anorexic")

    # optimizer
    parser.add_argument("--optimizer", default='Adam')
    parser.add_argument("--learning_rate", type=float, default=0.001)

    # datasets and dataloader
    parser.add_argument('--kitti_data', default='/home/hsk2bp/datasets/lidar_data/kitti')
    parser.add_argument('--stanford_data', default='/home/hsk2bp/datasets/stanford_pcds')
    parser.add_argument('--carla_data', default='/home/hsk2bp/datasets/carla_and_modelnet40_pcds/Carla')
    parser.add_argument('--modelnet_data', default='/home/hsk2bp/datasets/carla_and_modelnet40_pcds/ModelNet')
    parser.add_argument("--num_workers", type=int, default=4)
    parser.add_argument('--train_batch_size', default=32, type=int)
    parser.add_argument("--test_batch_size", default=160, type=int)
    parser.add_argument('--num_epochs', default=60, type=int)
    parser.add_argument("--num_points", type=int, default=256)
    parser.add_argument("--jitter", type=float, default=0.01)
    parser.add_argument("--stretch", default=(0.85, 1.15))
    parser.add_argument("--slide", default=(-0.1, 0.1))
    parser.add_argument("--dropout", type=float, default=0.875)
    parser.add_argument("--train_rotate", default=True)
    parser.add_argument("--test_rotate", default=False)

    # loss function
    parser.add_argument("--reg_loss", default = False)
    parser.add_argument("--weighted_loss", default=False)

    parser.add_argument("--comment", default='')

    # logs and saving
    parser.add_argument("--log_dir", default='./new_logs')
    parser.add_argument("--stopper_patience", type=int, default=900)
    parser.add_argument("--log_freq", type=int, default=900)
    parser.add_argument("--valid_freq", type=int, default=20)
    parser.add_argument("--save_freq", type=int, default=20)

    args = parser.parse_args()
    train_new(args)