from .gp_base import SparseGPModel, GP
from .GPModel import GPModel
from .util import train_gp, ChannelScaler, STDScaler
import torch
import gpytorch
import math
import numpy as np
import pickle

class SGPModel(GP):
    def __init__(self, data_x: np.ndarray, data_y:np.ndarray, num_of_inducing=None, training_iter=100, lr: float = 0.1):
        """Standard Spare GP model for regression with VFE inducing point optimization
        
        :param data_x: training input data
        :type data_x: np.ndarray or torch.tensor
        :param data_y: training output data
        :type data_y: np.ndarray or torch.tensor
        :param num_of_inducing: number of inducing points, defaults to None which is 1/10 of the training data
        :type num_of_inducing: int, optional
        :param training_iter: number of training iterations, defaults to 100
        :type training_iter: int, optional
        :param lr: learning rate for the optimizer, defaults to 0.1
        :type lr: float, optional
        """

        self.final_trained_loss=None
        
        if num_of_inducing is None:
            self.num_of_inducing=math.floor(data_x.shape[0]/10)
        else:
            self.num_of_inducing=num_of_inducing

        
        self.data_x=torch.tensor(data_x, dtype=torch.float64).contiguous()
        
        # scale input data to [-1, 1]
        self.input_scaler=ChannelScaler(lb=-1.0, ub=1.0)
        self.data_x=self.input_scaler(self.data_x)

        # initialize inducing points randomly, DEV: consider clustering
        #inducing_points=data_x[np.random.choice(data_x.shape[0], self.num_of_inducing, replace=False), :]    
        inducing_indices = torch.randperm(self.data_x.shape[0])[:self.num_of_inducing]
        inducing_points = self.data_x[inducing_indices]
        
        # init output data and use STD scaler
        self.data_y=torch.from_numpy(data_y).double().contiguous()
        
        self.output_scaler = STDScaler()
        self.data_y=self.output_scaler(self.data_y)

        
        self.training_iter=training_iter

        self.likelihood = gpytorch.likelihoods.GaussianLikelihood()
        self.model=SparseGPModel(self.data_x, self.data_y, inducing_points, self.likelihood)
        self.loss = gpytorch.mlls.ExactMarginalLogLikelihood(self.likelihood, self.model)
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=lr)


        # TODO: cuda support


    def train_model(self, verbose=True):
        # train the model
        self.final_trained_loss = train_gp(self.model, self.likelihood, self.loss, self.data_x, self.data_y, self.training_iter, self.optimizer, verbose=verbose)

        # switch to eval mode
        # Get into evaluation (predictive posterior) mode
        self.model.eval()
        self.likelihood.eval()

        if verbose:
            print(f"[Gaussian Process]: Training finished, switch to evaluation mode!")

        return self.final_trained_loss

    
    def eval_gp(self, gp_input, scale_input: bool = True, descale_output: bool = True):
        """Evaluates the model at the given test points
        
        :param gp_input: test input data
        :type gp_input: np.ndarray or torch.tensor
        :param scale_input: scale the input data, defaults to True
        :type scale_input: bool, optional
        :param descale_output: descale the output data, defaults to True
        :type descale_output: bool, optional
        :return: mean and variance of the GP model
        :rtype: tuple
        """
        with torch.no_grad(): # no gradient calculation
            
            if type(gp_input) is np.ndarray:
                gp_input=torch.tensor(gp_input, dtype=torch.float64)
                r_type = "np"
            else:
                r_type = "torch"

            if scale_input:
                gp_input=self.input_scaler(gp_input)

            pred=self.likelihood(self.model(gp_input))

            if descale_output: # descale the output
                mean = self.output_scaler.descale_data(pred.mean)
            else:
                mean = pred.mean


            if r_type == "np":
                return mean.numpy(), pred.variance.numpy()
            else:
                return mean, pred.variance
            
    def predict(self, gp_input, scale_input: bool = True, descale_output: bool = True):
        """Predicts the output at the given input points
    
        
        :param gp_input: input data
        :type gp_input: np.ndarray or torch.tensor
        :param scale    _input: scale the input data, defaults to True
        :type scale_input: bool, optional
        :param descale_output: descale the output data, defaults to True
        :type descale_output: bool, optional
        :return: predicted output
        :rtype: torch.tensor
        """

        # wrapper for eval_gp for comaptibilty
        return self.eval_gp(gp_input=gp_input, scale_input=scale_input, descale_output=descale_output) 
    

    def add_to_batch(self, *args, **kwargs):
        raise NotImplementedError("SGP model does not support batch training")
    
    def batch_update(self, *args, **kwargs):
        raise NotImplementedError("SGP model does not support batch training")
    
    def configure_update_method(self, *args, **kwargs):
        raise NotImplementedError("SGP model does not support online update")
    

    def export_to_GP(self):
        # inducing points and outputs
        inducing_inputs = self.model.covar_module.inducing_points.detach().clone()
        inducing_outputs = self.eval_gp(inducing_inputs, scale_input=False, descale_output=False)[0]

        # create a new GP model
        gp=GPModel(inducing_inputs.numpy(),
                   inducing_outputs.numpy(),
                   input_scaler=pickle.loads(pickle.dumps(self.input_scaler)), 
                   output_scaler=pickle.loads(pickle.dumps(self.output_scaler)))
        sgp_params = self.model.state_dict()
        gp_params = {
            "likelihood.noise_covar.raw_noise": sgp_params["likelihood.noise_covar.raw_noise"],
            "covar_module.raw_outputscale": sgp_params["base_covar_module.raw_outputscale"],
            "covar_module.base_kernel.raw_lengthscale": sgp_params["base_covar_module.base_kernel.raw_lengthscale"],
        }
        gp.model.initialize(**gp_params)
        gp.model.eval()
        gp.likelihood.eval()

        return gp


    def __str__(self) -> str:
        return f"SGP: M={self.num_of_inducing}"