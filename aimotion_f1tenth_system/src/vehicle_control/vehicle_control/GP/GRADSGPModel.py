from .gp_base import SparseGPModel, GP
from .GPModel import GPModel
from .util import train_gp, ChannelScaler, STDScaler
from .util import blockPrint, enablePrint
import torch
import gpytorch
import math
import numpy as np
import pickle

class GRADSGPModel(GP):
    def __init__(self, data_x: np.ndarray, data_y:np.ndarray, num_of_inducing: int = None, training_iter: int = 100, lr: float = 0.1):
        """Sparse GP model for regression with batch update capability
        
        :param data_x: training input data
        :type data_x: np.ndarray
        :param data_y: training output data
        :type data_y: np.ndarray
        :param num_of_inducing: number of inducing points, defaults to None which is 1/10 of the training data
        :type num_of_inducing: int, optional
        :param training_iter: number of training iterations, defaults to 100
        :type training_iter: int, optional
        :param lr: learning rate for the optimizer, defaults to 0.1
        :type lr: float, optional
        """
        
        # if no number of inducing points is given, use 1/10 of the data
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

        # numbre of iterations for offline training
        self.training_iter=training_iter

        # initlialize GP realted classed
        self.likelihood = gpytorch.likelihoods.GaussianLikelihood()
        self.model=SparseGPModel(self.data_x, self.data_y, inducing_points, self.likelihood)
        self.loss = gpytorch.mlls.ExactMarginalLogLikelihood(self.likelihood, self.model)
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=lr)


        # initialize the batch update variables and parameters as None
        self.retrain_iter = None
        self.batch_size = None
        self.batch_index = 0


    def configure_update_method(self, batch_size: int = None, retrain_iter: int = 0, lr: float = 0.1):
        """Configures the batch update method
        
        :param batch_size: number of data points to be used for the batch update, defaults to the number of inducing points
        :type batch_size: int, optional
        :param retrain_iter: number of iterations for the retraining, defaults to 0, i.e. no hyperparamter retraining
        :type retrain_iter: int, optional
        :param lr: learning rate for the optimizer, defaults to 0.1
        :type lr: float, optional
        """
        
        # batch upate parameters
        self.batch_size = batch_size if batch_size is not None else self.num_of_inducing
        self.retrain_iter = retrain_iter
        
        # allocate tensors for the new data
        self.batch_index = 0 # used for indexing
        self.batch_x=torch.zeros((self.batch_size, self.data_x.shape[1]), dtype=torch.float64)
        self.batch_y=torch.zeros((self.batch_size), dtype=torch.float64)
        

        # init step optimizer for single step updates
        self.step_optimizer = torch.optim.SGD(self.model.parameters(), lr=lr)


    def train_model(self, iters=None, train_x=None, train_y=None, optimizer=None, verbose=True):
        """Trains the hyperparameters of the model
        
        :param iters: number of iterations for the training, defaults to None which uses the offline training iterations
        :type iters: int, optional
        :param train_x: training input data, defaults to None which uses the initial training data
        :type train_x: torch.tensor, optional
        :param train_y: training output data, defaults to None which uses the initial training data
        :type train_y: torch.tensor, optional
        :param optimizer: optimizer for the training, defaults to None which uses the offline training optimizer (ADAM)
        """

        # Switch to training mode
        self.model.train()
        self.likelihood.train()

        # spacify optional parameters if not given
        if train_x is None:
            train_x=self.data_x
            train_y=self.data_y

        if iters is None:
            iters=self.training_iter

        if optimizer is None:
            optimizer=self.optimizer

        # train the model
        self.final_trained_loss = train_gp(self.model, self.likelihood, self.loss, train_x, train_y, iters, optimizer)

        # Get into evaluation (predictive posterior) mode
        self.model.eval()
        self.likelihood.eval()

        if verbose:
            print(f"Gaussian Process]: Training finished, switch to evaluation mode!")

        return self.final_trained_loss

    
    def eval_gp(self, gp_input, scale_input: bool = True, descale_output: bool = True):
        """Evaluates the model at the given test points 
        :param gp_input: test input data
        :type gp_input: np.ndarray or torch.tensor
        :param scale_input: if the input data should be scaled, defaults to True
        :type scale_input: bool, optional
        :param descale_output: if the output data should be descaled, defaults to True
        :type descale_output: bool, optional
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


            if r_type == "numpy":
                return mean.numpy(), pred.variance.numpy()
            else:
                return mean, pred.variance
            
            
    def add_to_batch(self, new_x, new_y):
        """Adds new data to the batch for the batch update
        
        :param new_x: new input data
        :type new_x: np.ndarray
        :param new_y: new output data
        :type new_y: np.ndarray
        """

        self.batch_x[self.batch_index, :] = self.input_scaler(torch.tensor(new_x, dtype=torch.float64))
        self.batch_y[self.batch_index] = self.output_scaler.scale_data(torch.tensor(new_y, dtype=torch.float64))

        # if the batch is full, update the model
        if self.batch_index == self.batch_size-1:
            self.batch_index = 0
            self.batch_update(self.batch_x, self.batch_y, self.retrain_iter)
        else:
            # else update the index
            self.batch_index+=1

        
    def batch_update(self, new_x, new_y, retrains: int = 0):
        """
        :param new_x: new input data
        :type new_x: torch.tensor
        :param new_y: new output data
        :type new_y: torch.tensor
        :param retrains: number of retraining iterations, defaults to 0
        :type retrains: int, optional
        
        """

        # get the inducing points and outputs
        inducing_inputs = self.model.covar_module.inducing_points.detach().clone()
        inducing_outputs = self.eval_gp(inducing_inputs, scale_input=False, descale_output=False)[0]

        update_x=torch.cat((inducing_inputs, new_x)) 
        update_y=torch.cat((inducing_outputs, new_y))

        # set new data
        self.model.set_train_data(update_x, update_y, strict=False)

        if retrains:
            self.train_model(retrains, update_x, update_y, optimizer=self.step_optimizer, verbose=False)


     
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
            "covar_module_raw_outputscale": sgp_params["base_covar_module.raw_outputscale"],
            "covar_module.base_kernel.raw_lengthscale": sgp_params["base_covar_module.base_kernel.raw_lengthscale"],
        }
        gp.model.initialize(**gp_params)
        gp.model.eval()
        gp.likelihood.eval()

        return gp


    def __str__(self) -> str:
        return f"GRADSGP: M={self.num_of_inducing}, i_max={self.retrain_iter}, N_u={self.batch_size}"