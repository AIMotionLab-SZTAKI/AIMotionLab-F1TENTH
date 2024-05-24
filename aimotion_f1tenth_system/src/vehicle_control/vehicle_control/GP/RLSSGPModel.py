from .gp_base import SparseGPModel, GP
from .GPModel import GPModel
from .util import train_gp, ChannelScaler, STDScaler
import torch
import gpytorch
import math
import numpy as np
from linear_operator.utils.cholesky import psd_safe_cholesky
import pickle

class RLSSGPModel(GP):
    def __init__(self, data_x: np.ndarray, data_y:np.ndarray, num_of_inducing: int = None, training_iter: int = 100, lr: float = 0.1):
        """Sparse GP model for regression with online update capability based on recursive least squares
        
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
        self.batch_size = None
        self.batch_index = 0


    def configure_update_method(self, batch_size: int = 10, confidence_level: float = 1, forgetting_fator: float = 0):
        """Configures the recursive update method
        
        :param batch_size: number of data points to be used for the batch update, defaults to 10
        :type batch_size: int, optional
        :param confidence_level: confidence level for the update, defaults to 1
        :type confidence_level: float, optional
        :param forgetting_fator: forgetting factor, defaults to 0
        :type forgetting_fator: float, optional
        """
        self.forgetting_factor = forgetting_fator
        self.batch_size = batch_size
        self.confidence_level = confidence_level

        # retrieve inducing inputs
        self.inducing_points=self.model.covar_module.inducing_points.detach()
        
        # obtain inducing outputs
        self.m, _ = self.eval_gp(self.inducing_points.numpy(), scale_input=False, descale_output=False)
        
        # for iterative update
        self.S = 1 / self.confidence_level * torch.eye(self.num_of_inducing).double()


        # precompute necessary and constant matrices used in the prediction
        self.K_MM = self.model.base_covar_module(self.inducing_points).evaluate().detach()
        self.L_MM = psd_safe_cholesky(self.K_MM, upper=True)  # m x m: L_m^T where L_m is lower triangular
        self.L_MM_inv = torch.linalg.solve_triangular(self.L_MM, torch.eye(self.num_of_inducing), upper=True)

        # allocate tensors for the new data
        self.batch_index = 0 # used for indexing
        self.batch_x=torch.zeros((self.batch_size, self.data_x.shape[1]), dtype=torch.float64)
        self.batch_y=torch.zeros((self.batch_size), dtype=torch.float64)

        self.Q_MM, self.R_MM = torch.linalg.qr(self.K_MM)
        R_inv = torch.linalg.solve_triangular(self.R_MM, torch.eye(self.num_of_inducing), upper=True)
        self.K_MM_inv = R_inv @ self.Q_MM.T



    def train_model(self, iters=None, train_x=None, train_y=None, optimizer=None, verbose=True):
        """Trains the hyperparameters of the model
        
        :param iters: number of iterations for the training, defaults to None which uses the offline training iterations
        :type iters: int, optional
        :param train_x: training input data, defaults to None which uses the initial training data
        :type train_x: torch.tensor, optional
        :param train_y: training output data, defaults to None which uses the initial training data
        :type train_y: torch.tensor, optional
        :param optimizer: optimizer for the training, defaults to None which uses the offline training optimizer (ADAM)
        :type optimizer: torch.optim.Optimizer, optional
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
        self.final_trained_loss = train_gp(self.model, self.likelihood, self.loss, train_x, train_y, iters, optimizer, verbose)

        # Get into evaluation (predictive posterior) mode
        self.model.eval()
        self.likelihood.eval()

        if verbose:
            print("")
            print(f"Gaussian Process]: Training finished, switch to evaluation mode!")

        return self.final_trained_loss

    
    def eval_gp(self, gp_input, scale_input: bool = True, descale_output: bool = True):
        """Evaluates the model at the given test points 

        :param gp_input: input data for the prediction
        :type gp_input: np.ndarray or torch.tensor
        :param scale_input: scale the input data, defaults to True
        :type scale_input: bool, optional
        :param descale_output: descale the output data, defaults to True
        :type descale_output: bool, optional
        :return: mean and variance of the GP model
        :rtype: tuple of torch.tensor or np.ndarray

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
            self.batch_update(self.batch_x, self.batch_y)
        else:
            # else update the index
            self.batch_index+=1

        
    def batch_update(self, new_x, new_y):
        """
        :param new_x: new input data
        :type new_x: torch.tensor
        :param new_y: new output data
        :type new_y: torch.tensor
        
        """

        # get the inducing points and outputs
        Phi = self.model.base_covar_module(new_x, self.inducing_points).evaluate().detach() @ self.K_MM_inv

        G = torch.eye(self.batch_size)*self.forgetting_factor + Phi @ self.S @ Phi.T

        Q_G, R_G = torch.linalg.qr(G)
        R_inv = torch.linalg.solve_triangular(R_G, torch.eye(self.batch_size), upper=True)
        G_inv = R_inv @ Q_G.T

        L = self.S @ Phi.T @ G_inv

        self.S = 1 / self.forgetting_factor * (self.S - L @ G @ L.T)

        r = new_y - Phi @ self.m

        self.m = self.m + L @ r




    def predict(self, x, scale_input: bool = True, descale_output: bool = True):
        """Predicts the mean and variance of the GP at the given input

        :param x: input data for the prediction
        :type x: np.ndarray
        :return: mean and variance of the GP
        :rtype: tuple of torch.tensor or np.ndarray

        """
        
        if self.batch_size is None:
            return self.eval_gp(x, scale_input=scale_input, descale_output=descale_output)

        if type(x) is np.ndarray:
            x=torch.tensor(x, dtype=torch.float64)
            r_type = "np"
        else:
            r_type = "torch"
    
        if scale_input:
            x = self.input_scaler(x)

        # compute the covariance matrices
        K_xM = self.model.base_covar_module(x, self.inducing_points).evaluate().detach()
        k_xx = self.model.base_covar_module(x).evaluate().detach()
        
        K_xML_MM_inv = K_xM @ self.L_MM_inv
        
        alpha = self.L_MM_inv.T @ self.m

        mean = torch.matmul(K_xML_MM_inv, alpha)

        # Compute variance
        var = k_xx - torch.matmul(K_xML_MM_inv, K_xML_MM_inv.T) + K_xM @ self.K_MM_inv @self.S @ self.K_MM_inv.T @ K_xM.T

        if descale_output:
            mean = self.output_scaler.descale_data(mean)


        if r_type == "np":
            return mean.numpy(), var.numpy()
        
        return mean, var
    
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
        return f"RLSSGP: M={self.num_of_inducing}, lambda={self.forgetting_factor}, beta={self.confidence_level}, N_u={self.batch_size}"