"""
Contains neural network training and inference classes for PyTorch
"""
import torch
from torch.jit import verify
import torch.nn as nn
from tensorboardX import SummaryWriter
from tqdm import trange
import os
from graphviz import Digraph
import torch.utils.data
import numpy as np
from neurAD.common.utils import early_stop
from neurAD.pytorch.pipeline import PreLoadDataSet, GenDataSet, SimpleDataLoader
import warnings
import tracemalloc
import gc

if torch.__version__.split('.')[1] != '4' and torch.__version__.split('.')[1] != '5':
    warnings.warn("Current neurAD is tested only with PyTorch 0.4 and 0.5, if problems occur, contact @vel2bp")


########################################################################################################################
# General torch NN
########################################################################################################################


class TorchNet(object):
    """
    PyTorch neural network base class
    """
    def __init__(self, gpu=True, half=False, async_=True):
        self.use_GPU = gpu
        self.tensor_core = half
        self.weights = None
        if half:
            self.dtype = np.float16
        else:
            self.dtype = np.float32
        self.async_ = async_

    def gpu(self, item):
        """
        Make model run on GPU

        :param item: pytorch object
        :return: the pinned item
        """
        if self.use_GPU:
            if self.tensor_core:
                return item.cuda().half()
            else:
                return item.cuda()
        else:
            return item

    def load_model(self, path):
        """
        Loads pytorch model from path

        :param path: path pf the model file
        :return:  the loaded weights
        """
        if os.path.isdir(path):
            weight_num_list = [int(s[8:-4]) for s in os.listdir(path) if s.startswith("weights_")]
            latest_weight = max(weight_num_list)
            path = os.path.join(path, "weights_" + str(latest_weight) + ".pkl")
        self.weights = torch.load(path)
        if self.model is not None:
            self.model.load_state_dict(self.weights)
            print("Succesfully restored model weights from ", path)
        else:
            print("Model is not defined, weights are not loaded.")
        self.gpu(self.model)
        return self.weights

    @staticmethod
    def datasets_from_generators(generator_dict, preload=True):
        """
        Create datasets from generator dictionary

        :param generator_dict: generator dictionary (function calls)
        :param preload: cache in advance to speed up operations
        :return: the pytorch datasets
        """
        torch_data = dict()

        if preload:
            DataSet = PreLoadDataSet
        else:
            DataSet = GenDataSet

        for dat in ["train", "test", "validation"]:
            torch_data[dat] = DataSet(generator_dict[dat + "_size"], generator_dict[dat])

        return torch_data

    @staticmethod
    def loaders_from_datasets(dataset_dict, batch_size, shuffle=False, pin=False, workers=1):
        """
        Creates pytorch data loaders from pytorch datasets

        :param dataset_dict: dataset dictionary, keys: train, test, validation
        :param batch_size: batch size
        :param shuffle: shuffle the items for batches
        :param pin: pin to gpu memory (may speed up computations)
        :param workers: number of workers for the data pipeline (stick to 0 if you are not explicitly threadsafe)
        :return: the loaders dictionary with the same keys
        """
        loaders = dict()

        for dat in ["train", "test", "validation"]:
            loaders[dat] = torch.utils.data.DataLoader(dataset=dataset_dict[dat],
                                                       batch_size=batch_size,
                                                       shuffle=shuffle,
                                                       pin_memory=pin,
                                                       num_workers=workers)
        return loaders

    @staticmethod
    def simple_loaders_from_generators(generator_dict, batch_size):
        """
        Create simple pytorch loaders from generator dictionary

        :param generator_dict: generator dictionary
        :param batch_size: number of items in one batch
        :return: the simple loader dictionary
        """
        loaders = dict()

        for dat in ["train", "test", "validation"]:
            loaders[dat] = SimpleDataLoader(generator_dict[dat], batch_size)
        return loaders

    @staticmethod
    def get_optimizer(opt_name):
        """
        Get optimizer from torch from a string input

        :param opt_name: name of the optimizer
        :return: the optimizer for the problem
        """
        return getattr(torch.optim, opt_name)

    @staticmethod
    def get_loss(loss_name):
        """
        Gets loss (criterion) for training the network

        :param loss_name: name of the loss in PyTorch
        :return: the loss function
        """
        if loss_name == "L2":
            return torch.nn.modules.loss.MSELoss()
        elif loss_name == "L1":
            return nn.L1Loss()
        elif loss_name == "crossentropy":
            return nn.CrossEntropyLoss()
        else:
            raise ModuleNotFoundError

    @staticmethod
    def copy_in_params(net, params):
        net_params = list(net.parameters())
        for i in range(len(params)):
            net_params[i].data.copy_(params[i].data)

    @staticmethod
    def set_grad(params, params_with_grad):

        for param, param_w_grad in zip(params, params_with_grad):
            if param.grad is None:
                param.grad = torch.nn.Parameter(param.data.new().resize_(*param.data.size()))
            if param_w_grad.grad is not None:
                param.grad.data.copy_(param_w_grad.grad.data)

    def plot_graph(self, dummy_input, params=None):
        """
        Plots the directed acyclic graph of the neural network

        :param dummy_input: dummy input for running a simple forward pass on the NN
        :param params: the parameters of the neural network to plot
        :return: nothing
        """
        var = self.model(dummy_input)

        if params is not None:
            assert isinstance(params.values()[0], torch.Tensor)
            param_map = {id(v): k for k, v in params.items()}

        node_attr = dict(style='filled',
                         shape='box',
                         align='left',
                         fontsize='12',
                         ranksep='0.1',
                         height='0.2')
        dot = Digraph(node_attr=node_attr, graph_attr=dict(size="12,12"))
        seen = set()

        def size_to_str(size):
            """
            convert variable size tto string
            """
            return '(' + (', ').join(['%d' % v for v in size]) + ')'

        def add_nodes(var):
            """
            Add nodes to the DAG to plot
            """
            if not isinstance(var, list) and var not in seen:
                if torch.is_tensor(var):
                    dot.node(str(id(var)), size_to_str(var.size()), fillcolor='orange')
                elif hasattr(var, 'variable'):
                    u = var.variable
                    name = "Weight"
                    if params is not None and id(u) in param_map:
                        name = param_map[id(u)] if params is not None else ''
                    node_name = '%s\n %s' % (name, size_to_str(u.size()))
                    dot.node(str(id(var)), node_name, fillcolor='lightblue')
                else:
                    dot.node(str(id(var)), str(type(var).__name__))
                seen.add(var)
                if hasattr(var, 'next_functions'):
                    for u in var.next_functions:
                        if u[0] is not None:
                            dot.edge(str(id(u[0])), str(id(var)))
                            add_nodes(u[0])
                if hasattr(var, 'saved_tensors'):
                    for t in var.saved_tensors:
                        dot.edge(str(id(t)), str(id(var)))
                        add_nodes(t)

        add_nodes(var.grad_fn)
        dot.format = 'pdf'
        return dot

    def ndarray_to_batched_tensor(self, ndarray):
        """
        Convert an ndarray to a batched tensor

        :param ndarray: ndarray
        :return: tensor
        """
        return self.gpu(torch.unsqueeze(torch.from_numpy(np.asarray(ndarray, dtype=self.dtype)), 0))

    def inference(self, tensor):
        """
        Forward pass the model without gradient calculation and mid-variable saving

        :param tensor: inference the network with given weights
        :return: the output of the network
        """
        with torch.no_grad():
            tensor = self.gpu(tensor)
            out = self.model.forward(tensor)
        return out

    @staticmethod
    def assert_no_nan(tensor):
        assert not torch.isnan(tensor).any()

    def list2tensor(self, inlist: list):
        return self.gpu(torch.Tensor(np.asarray(inlist, dtype=self.dtype)))


########################################################################################################################
# Train NN
########################################################################################################################


class Optimus(TorchNet):
    """
    PyTorch neural network training class
    """
    def __init__(self, model, data_loaders, log_dir, learning_rate, eval_freq, optimizer=None, loss_fn=None,
                 gpu=True, plot=False, valid_loss_avg=True, stopper_patience=100, tensorcore=False,
                 easy_mode=True, loss_scale=1, profile=True, grad_clip=None, histogram_save_freq=None,
                 compile_model=False, example_input=None, multi_gpu=True):
        """
        Init the Optimus trainer object

        :param model: neural network model (nn.Model)
        :param data_loaders: nn data loaders in a dict with keys train, test, validation
        :param log_dir: logging directory root
        :param optimizer: gradient based optimiser
        :param loss_fn: torch loss criterion or a list of loss functions to call
        :param eval_freq: dictionary of train, test, validation
        :param gpu: True, use GPU or not
        :param plot: whether to plot the graph in tensorboard or graphviz
        :param valid_loss_avg: average the validation loss over batches
        :param stopper_patience: how many steps to take after valid loss grows
        :param tensorcore: whether to use mixed precision training
        :param easy_mode: if set, neurAD will use its own train and evaluation functions to optimize
        :param compile_model: it is an early beta function. You must use @compile decorator for your model
                        and set this parameter to True
        :param example_input: if provided, graph is drawn
        :param multi_gpu: whether to use multiple GPUs
        """
        super(Optimus, self).__init__(gpu, half=tensorcore)

        self.data_loaders = data_loaders
        self.optimizer = optimizer
        self.multiloss = False
        # Checking if one or more losses are given
        if type(loss_fn) is list and not easy_mode:
            self.loss_fn = loss_fn
            self.multiloss = True
        elif type(loss_fn) is list and easy_mode:
            warnings.warn("Multiple loss functions are not supported in easy mode! First given loss will be used.")
            self.loss_fn = loss_fn[0]
        elif type(loss_fn) is not None:
            self.loss_fn = self.gpu(loss_fn)
        else:
            self.loss_fn = None
        self.log_dir = log_dir
        self.global_step = 0
        self.writer = SummaryWriter(log_dir=self.log_dir)
        self.eval_freq = eval_freq
        self.valid_loss_avg = valid_loss_avg
        self.easy_mode = easy_mode
        self.loss_scaler = loss_scale
        self.grad_clip = grad_clip
        self.hooks = {"train": [], "test": [], "validation": []}
        self.profile = profile
        self.histogram_save_freq = histogram_save_freq
        self.clip_total_norm = []
        self.model = model

        # The hook needs this actual_output, temporally solution before refactor
        if not hasattr(self.model, "actual_putput"):
            self.model.actual_output = None
        self.multi_gpu = multi_gpu

        if not self.easy_mode:
            warnings.warn("Not easy mode and parallel training is not compatible, parallel mode deactivated!")
            # TODO: Fix it
            self.multi_gpu = False

        if torch.cuda.device_count() > 1 and self.multi_gpu:
            print("Using multiple GPUs for parallel training: ", torch.cuda.device_count())
            self.model = nn.DataParallel(self.model)

        self.model = self.gpu(self.model)
        self.model.train(True)

        self.example_input = None
        if example_input is None:
            if hasattr(model, "dummy_input"):
                self.example_input = model.dummy_input
        else:
            self.example_input = example_input

        if compile_model:
            # TODO elimiinate dummy input and use first batch for this
            if self.example_input is None:
               warnings.warn("Cannot compile NN model without dummy input to trace.")
            else:
                out1 = self.model(self.example_input)
                out1.sum().backward()
                #verify(self.model, example_input)

        self.sentinel = early_stop(patience=stopper_patience)
        self.should_stop = False

        assert hasattr(model, "forward")
        if not self.easy_mode:
            assert hasattr(model, "run_train") and hasattr(model, "run_eval")
            
        if plot and hasattr(self.model, "dummy_input"):
            try:
                result = self.model(self.model.dummy_input)
                self.plot(self.model.dummy_input)
                self.writer.add_graph(self.model, result)
            except:
                print("Plotting the computational graph was unsuccessful!")
                    
        if self.tensor_core:
            param_copy = [param.clone().type(torch.cuda.FloatTensor).detach() for param in self.model.parameters()]
            for param in param_copy:
                param.requires_grad = True
            self.params_fp32 = param_copy
            
        if self.tensor_core:
            self.optimizer = TorchNet.get_optimizer(optimizer)(params=self.params_fp32, lr=learning_rate)
        else:
            self.optimizer = TorchNet.get_optimizer(optimizer)(params=self.model.parameters(), lr=learning_rate)

        self.save()

    def add_hook_fn(self, fn_ref, hook_on="train"):
        """
        Add hook function to either train, test, validation

        :param fn_ref: function reference, paramaters: input_t, output_t, target_t, step
        :param hook_on: either train, test, validation
        """
        self.hooks[hook_on].append(fn_ref)
        
    def run_trainstep(self, inputs, targets):
        """
        Run inference, loss calculation and backprop and weight update on NN module
        """
        self.optimizer.zero_grad()
        #print(inputs.size())
        self.model.actual_output = self.model.forward(inputs)
        #print(self.model.forward(inputs))

        loss = self.loss_fn(self.model.actual_output, targets) * self.loss_scaler
        self.assert_no_nan(loss)

        loss.backward()
        
        if self.tensor_core:
            self.set_grad(self.params_fp32, list(self.model.parameters()))
            
            if self.loss_scaler != 1:
                for param in self.params_fp32:
                        param.grad.data = param.grad.data / self.loss_scaler

            self.optimizer.step()

            self.copy_in_params(self.model, self.params_fp32)
        else:
            if self.loss_scaler != 1:
                for param in self.model.parameters():
                    if param.grad is not None:
                        param.grad.data = param.grad.data / self.loss_scaler
                        
            self.optimizer.step()
        loss = loss / self.loss_scaler
        return loss

    def run_evalstep(self, inputs, targets):
        """
        Run inference and loss calculation on NN module
        """
        outs = self.model.forward(inputs)

        loss = self.loss_fn(outs, targets)
        return loss

    def trainer_fn(self, input, target):
        input, target = self.gpu(input), self.gpu(target)
        if self.easy_mode:
            return self.run_trainstep(input, target)
        else:
            return self.model.run_train(input, target, self.loss_fn, self.optimizer)

    def train_one(self, input, target):
        """
        Train one batch

        :param input: input variables
        :param target: target variables for loss calculation
        :return: None
        """
        # optionally profile your training
        if self.profile and self.global_step == 10:  # 10 steps for warmup
            with torch.autograd.profiler.profile(use_cuda=self.use_GPU) as prof:
                loss = self.trainer_fn(input, target)
            prof.export_chrome_trace(self.log_dir + "/trainstep_profile.trace")
            with open(self.log_dir + "/trainstep_profile_function.log", "w") as profprint:
                print(prof, file=profprint)
        else:
            loss = self.trainer_fn(input, target)

        if self.histogram_save_freq is not None and self.global_step % self.histogram_save_freq == 1:
            for name, param in self.model.named_parameters():
                self.writer.add_histogram(name, param.clone().cpu().data.numpy(), global_step=self.global_step,
                                             bins="tensorflow")
                self.writer.add_histogram(name + "_grad", param.grad.clone().cpu().data.numpy(),
                                             global_step=self.global_step, bins="tensorflow")

        if self.grad_clip is not None:
            clip_total_norm = nn.utils.clip_grad_norm(self.model.parameters(), self.grad_clip)
            if self.histogram_save_freq is not None:
                self.clip_total_norm.append(clip_total_norm)
                if self.global_step % self.histogram_save_freq == 1:
                    self.writer.add_histogram("total_norm", np.array(self.clip_total_norm),
                                              global_step=self.global_step, bins="doane")
                    self.clip_total_norm = []

        if len(self.hooks["train"]) != 0:
            _ = [fn(input_t=input, output_t=self.model.actual_output, target_t=target, optimus=self)
                 for fn in self.hooks["train"]]

        if self.eval_freq["log"] != 0 and (self.global_step + 1) % self.eval_freq["log"] == 0:
            # If more losses are given, then each of them is written to log
            if self.multiloss:
                for a in range(len(loss)):
                    self.writer.add_scalar("loss_{}".format(a), loss[a].item(), global_step=self.global_step)
            else:
                self.writer.add_scalar("batch_loss", loss.item(), global_step=self.global_step)
        if self.eval_freq["save"] != 0 and (self.global_step + 1) % self.eval_freq["save"] == 0:
            self.save()
        if self.eval_freq["validation"] != 0 and (self.global_step + 1) % self.eval_freq["validation"] == 0:
            self.eval("validation")

    def train(self, epoch=1, steps=None):
        """
        Train the network for n epochs

        :param epoch: number of epoch to train for
        :param steps: number of steps, by default None, the dataset contains this info
        :return: None
        """
        if steps is None:
            steps = len(self.data_loaders["train"])
        try:
            for ep in range(epoch):
                if self.should_stop:
                    print("\nEarly stopper stopped training, validation loss diverged")
                    break
                data_iter = iter(self.data_loaders["train"])
                print("\nepoch ", ep, ":")
                for step in trange(steps, desc='train_batches'):
                    self.global_step += 1
                    try:
                        in_tensor, out_tensor = next(data_iter)
                    except StopIteration:
                        print("end of dataset...")
                        break
                    self.train_one(in_tensor, out_tensor)
            self.eval("test")
        except KeyboardInterrupt:
            print("Training stopped by user, testing...")
            self.eval("test")
        finally:
            self.save()

    def eval(self, dataset):
        """
        Run evaluation on the test/validaion dataset

        :param dataset: test or validation (string)
        :return: the summarized loss on the given dataset
        """
        if self.use_GPU:
            torch.cuda.empty_cache()
        try:
            steps = len(self.data_loaders[dataset])
            data_iter = iter(self.data_loaders[dataset])
        except KeyError:
            print("Dataset name is not correct, should be train, test, validation")
            return

        sum_loss = 0.0
        sum_acc = 0.0
        counter = 0.0
        conf_matrix = np.zeros((self.model.num_classes, self.model.num_classes), dtype=np.int)
        batch_conf_matrix = np.zeros((self.model.num_classes, self.model.num_classes), dtype=np.int)

        for steps in trange(steps, desc=str(dataset)):
            inputs, targets = next(data_iter)
            with torch.no_grad():
                inputs, targets = self.gpu(inputs), self.gpu(targets)
                if self.easy_mode:
                    loss = self.run_evalstep(inputs, targets)
                else:
                    loss, batch_acc, batch_conf_matrix = self.model.run_eval(inputs, targets, self.loss_fn)
                sum_loss += loss.item()
                sum_acc += batch_acc
                counter += 1.0
                conf_matrix += batch_conf_matrix

        if self.valid_loss_avg:
            sum_loss = sum_loss / counter
            sum_acc = sum_acc / counter

        print('\nEval: loss={:.4f} acc={:.4f}\n'.format(sum_loss, sum_acc))
        print('Conf matrix:\n', conf_matrix)


        self.sentinel.add(sum_loss)
        self.should_stop = self.sentinel.should_stop()

        self.writer.add_scalar(dataset + "_loss", sum_loss, global_step=self.global_step)
        self.writer.add_scalar(dataset + "_accuracy", sum_acc, global_step=self.global_step)

        return sum_loss

    def save(self, path=None):
        """
        Save the model with the trained weights

        :param path: path to save the model
        :return: None
        """
        if path is None:
            path = os.path.join(self.log_dir, "weights_" + str(self.global_step) + ".pkl")
        if self.tensor_core:
            # save fp32 master copy of weights instead of fp16
            torch.save(self.params_fp32, path)
        else:
            torch.save(self.model.state_dict(), path)

    def onnx_export(self, out_file, verbose=True):
        if self.example_input is not None:
            return torch.onnx.export(self.model, self.example_input, out_file, verbose=verbose)
        else:
            warnings.warn("Cannot compile NN model without dummy input to trace.")

    def plot(self, inputs):
        """
        Create a plot of the directed acyclic graph

        :param inputs: inputs to use for the forward pass
        :return: None
        """
        plt = self.plot_graph(inputs)
        plt.render(filename="graph", directory=self.log_dir)

    def collect_stats(self):
        """
        Collect memory usage statistic, and print the increase grouped by filename
        Example usage in train loop:
        tracemalloc.start(10)                                                       <- inserted line
        for ep in range(epoch):
            if self.should_stop:
                print("\nEarly stopper stopped training, validation loss diverged")
                break
            data_iter = iter(self.data_loaders["train"])
            print("\nepoch ", ep, ":")
            for step in trange(steps, desc='train_batches'):
                self.global_step += 1
                try:
                    in_tensor, out_tensor = next(data_iter)
                except StopIteration:
                    print("end of dataset...")
                    break
                self.train_one(in_tensor, out_tensor)
                if self.global_step % 10 == 0:                                      <- inserted line
                        self.collect_stats()                                        <- inserted line


        :return:
        """
        if not hasattr(self, 'snapshots'):
            self.snapshots = []
        gc.collect()
        snapshot = tracemalloc.take_snapshot()
        snapshot = snapshot.filter_traces((
                tracemalloc.Filter(False, "*tracemalloc*"),
                tracemalloc.Filter(False, "*linecache*"),
            ))
        self.snapshots.append(snapshot)
        if len(self.snapshots) > 1:
            stats = self.snapshots[-1].compare_to(self.snapshots[-2], key_type='filename', cumulative=True)

            for stat in stats[:10]:
                print("{} new KiB {} total KiB {} new {} total memory blocks: ".format(stat.size_diff/1024, stat.size / 1024, stat.count_diff ,stat.count))
                for line in stat.traceback.format():
                    print(line)

########################################################################################################################
# Inference trained model
########################################################################################################################


class Inference(TorchNet):
    """
    PyTorch loader and inference class
    """

    def __init__(self, model, path, gpu=True, half=False):
        """
        Init the Inference class

        :param model: neural network model (nn.Module)
        :param path: path of the saved weights
        :param gpu: whether to use GPU for running the network
        """
        super(Inference, self).__init__(gpu, half=half)
        self.model = model
        self.load_model(path=path)
        self.model.train(False)

    def feed_ndarray(self, ndarray, dtype=np.float32):
        """
        Feed an ndarray of given type to the network and forward pass it with a corresponding tensor

        :param ndarray: np array
        :param dtype: np.float___
        :return: the outputs of the network in numpy arrays
        """

        if ndarray.dtype != dtype:
            ndarray = np.asarray(ndarray, dtype=dtype)
        tensor = self.gpu(torch.from_numpy(ndarray))
        output = self.inference(torch.unsqueeze(tensor, 0))

        return torch.squeeze(output).cpu().numpy()
