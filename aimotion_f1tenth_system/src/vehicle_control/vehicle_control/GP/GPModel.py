from .gp_base import SparseGPModel, GP, ExactGPModel
from .util import train_gp, ChannelScaler, STDScaler
import torch
import gpytorch
import math
import numpy as np


class GPModel(GP):
    def __init__(self, data_x: np.ndarray, data_y:np.ndarray, training_iter=100, lr: float = 0.1, input_scaler=None, output_scaler=None):
        """Standard Spare GP model for regression with VFE inducing point optimization
        
        :param data_x: training input data
        :type data_x: np.ndarray or torch.tensor
        :param data_y: training output data
        :type data_y: np.ndarray or torch.tensor
        :param training_iter: number of training iterations, defaults to 100
        :type training_iter: int, optional
        :param lr: learning rate for the optimizer, defaults to 0.1
        :type lr: float, optional
        :param input_scaler: input scaler, defaults to None. When provided the data_x data_y will not be scaled at initialization just at the evaluation
        :type input_scaler: ChannelScaler, optional
        :param output_scaler: output scaler, defaults to None. When provided the data_x data_y will not be scaled at initialization just at the evaluation
        :type output_scaler: STDScaler, optional
        """

        self.final_trained_loss=None

        
        self.data_x=torch.tensor(data_x, dtype=torch.float64)
        
        # scale input data to [-1, 1]
        if input_scaler is None:
            self.input_scaler=ChannelScaler(lb=-1.0, ub=1.0)
            self.data_x=self.input_scaler(self.data_x)
        
        else: 
            self.input_scaler = input_scaler
        # init output data and use STD scaler

        self.data_y=torch.from_numpy(data_y).double()
        
        if output_scaler is None: 
            self.output_scaler = STDScaler()
            self.data_y=self.output_scaler(self.data_y)
        else:
            self.output_scaler = output_scaler
        
        self.training_iter=training_iter

        self.likelihood = gpytorch.likelihoods.GaussianLikelihood()
        self.model=ExactGPModel(self.data_x, self.data_y, self.likelihood)
        self.loss = gpytorch.mlls.ExactMarginalLogLikelihood(self.likelihood, self.model)
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=lr)




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
        :param scale_input: scale the input data, defaults to True
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
    
    def extend_dataset(self, new_x, new_y):
        """Extends the dataset with new data
        
        :param new_x: new input data
        :type new_x: np.ndarray or torch.tensor
        :param new_y: new output data
        :type new_y: np.ndarray or torch.tensor
        """
        new_x=torch.tensor(new_x, dtype=torch.float64)
        new_y=torch.tensor(new_y, dtype=torch.float64)
        
        new_x=self.input_scaler(new_x)
        new_y=self.output_scaler(new_y)
       
        self.data_x=torch.cat((self.data_x, new_x), dim=0)
        self.data_y=torch.cat((self.data_y, new_y.unsqueeze(0)), dim=0)
        
        self.model.set_train_data(inputs=self.data_x, targets=self.data_y, strict=False)
        

    def __str__(self) -> str:
        return "GP"
    