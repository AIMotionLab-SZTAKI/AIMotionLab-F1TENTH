import torch
import gpytorch
from abc import ABC, abstractmethod

class GP(ABC):
    @abstractmethod
    def predict(self):
        pass
    
    @abstractmethod
    def train_model(self):
        pass

    @abstractmethod
    def eval_gp(self):
        pass

    @abstractmethod
    def add_to_batch(self):
        pass

    @abstractmethod
    def batch_update(self):
        pass

    @abstractmethod
    def configure_update_method(self):
        pass

    @abstractmethod
    def predict(self):
        pass


class ExactGPModel(gpytorch.models.ExactGP):
    def __init__(self, train_x, train_y, likelihood):
        """Exact GP model for regression"""
        super(ExactGPModel, self).__init__(train_x, train_y, likelihood)
        self.mean_module = gpytorch.means.ZeroMean()
        self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.RBFKernel(ard_num_dims=train_x.shape[1]))

    def forward(self, x: torch.Tensor):
        """Forward pass for the model
        
        :param x: input tensor
        :type x: torch.Tensor
        :return: MultivariateNormal distribution
        """
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)


class SparseGPModel(gpytorch.models.ExactGP):
    def __init__(self, train_x, train_y, initial_incuding_points, likelihood):
        """Sparse GP model for regression"""
        super(SparseGPModel, self).__init__(train_x, train_y, likelihood)
        self.mean_module = gpytorch.means.ZeroMean()
        self.base_covar_module=gpytorch.kernels.ScaleKernel(gpytorch.kernels.RBFKernel(ard_num_dims=train_x.shape[1]))
        self.covar_module = gpytorch.kernels.InducingPointKernel(self.base_covar_module, inducing_points=initial_incuding_points, likelihood=likelihood)

    def forward(self, x: torch.Tensor):
        """Forward pass for the model
        
        :param x: input tensor
        :type x: torch.Tensor
        :return: MultivariateNormal distribution
        """
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)
    