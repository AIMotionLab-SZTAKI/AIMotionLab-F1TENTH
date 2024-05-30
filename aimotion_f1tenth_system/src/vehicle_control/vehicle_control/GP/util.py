import torch
import sys
import os

def blockPrint():
    sys.stdout = open(os.devnull, 'w')


# Restore
def enablePrint():
    sys.stdout = sys.__stdout__

def train_gp(model, likelihood, loss, train_x, train_y, training_iter, optimizer, verbose=True):
    """Trains the hyperparameters of the GP model

    :param model: GP model
    :type model: gpytorch.models.GP
    :param likelihood: likelihood function
    :type likelihood: gpytorch.likelihoods.Likelihood
    :param loss: loss function
    :type loss: gpytorch.mlls.MarginalLogLikelihood
    :param train_x: training input data
    :type train_x: torch.tensor
    :param train_y: training output data
    :type train_y: torch.tensor
    :param training_iter: number of iterations for the training
    :type training_iter: int
    :param optimizer: optimizer for the training
    :type optimizer: torch.optim.Optimizer
    """

    # "Loss" for GPs - the marginal log likelihood
    for i in range(training_iter):
        # Zero gradients from previous iteration
        optimizer.zero_grad()
        # Output from model
        output = model(train_x)
        # Calc loss and backprop gradients
        l = -loss(output, train_y)
        l.backward()

        if verbose:
            print(f"[Gaussian Process trainging]: Iter: Iter {i+1} - Loss: {l.item():.23}", end='\r')
        optimizer.step()

        # potential cuda support
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        
    if verbose:
        print("")# to break the line

    return l.item() # return the final loss


class ChannelScaler:
    def __init__(self, lb=-1.0, ub=1.0):
        """Scales input data to the range [lb, ub] independently for each channel"""
        self.lb = lb
        self.ub = ub
        self.mins = None
        self.maxs = None
        self.trained = False

    def train(self, data):
        """Trains the scaler on the given data
        
        :param data: data to be trained on
        :type data: torch.tensor or np.ndarray
        """
        self.mins = data.min(dim=0).values
        self.maxs = data.max(dim=0).values
        self.trained = True

    def __call__(self, data: torch.tensor):
        if not self.trained:
            self.train(data)
        
        return self._scale_data(data)
    
    def _scale_data(self, data):
        """Scales the data into the range [lb, ub] independently for each channel
        
        :param data: data to be scaled
        :type data: torch.tensor or np.ndarray
        """

        if not self.trained:
            raise ValueError("Scaler has not been trained yet")
        
        # Compute channel-wise scaling factors
        channel_mins = self.mins.unsqueeze(0)  # shape: (1, num_channels)
        channel_maxs = self.maxs.unsqueeze(0)  # shape: (1, num_channels)
        channel_ranges = channel_maxs - channel_mins
        
        # Avoid division by zero (set range to 1 where zero)
        channel_ranges[channel_ranges == 0] = 1
        
        # Scale the data
        scaled_data = ((data - channel_mins) / channel_ranges) * (self.ub - self.lb) + self.lb
        
        return scaled_data
    

class STDScaler:
    def __init__(self) -> None:
        """Data scaler based on standard deviation and mean"""
        self.mean=None
        self.std=None
        self.trained = False

    def train(self, data):
        """Trains the scaler on the given data
        :param data: data to be trained on
        :type data: torch.tensor or np.ndarray
        """
        self.mean = data.mean()
        self.std = data.std()
        self.trained = True

    def __call__(self, data: torch.tensor):
        if not self.trained:
            self.train(data)
            return self.scale_data(data)
        else:
            return self.descale_data(data)
    
    def scale_data(self, data):
        """Scales the data by std deviation and mean
        :param data: data to be scaled
        :type data: torch.tensor or np.ndarray
        """

        if not self.trained:
            raise ValueError("Scaler has not been trained yet")
        
        data = (data - self.mean) / self.std

        return data
    
    def descale_data(self, data):
        """Scales the data into the range [lb, ub] independently for each channel
        :param data: scaled data to be descaled
        :type data: torch.tensor or np.ndarray

        """
        data = (data * self.std) + self.mean

        return data
    


if __name__ =="__main__":
    print("This is a utility file. It should not be run directly.")
    print("It contains helper functions for the SGP model.")


    data_x=torch.tensor([[1, 2, 3], [4, 5, 6], [7, 8, 9]], dtype=torch.float64)
    data_y=torch.tensor([-5, 5,1,4,2,5,2,1,1,5,1,3,5,-3, -4,-1,-5], dtype=torch.float64)

    input_scaler=ChannelScaler()
    data_x_scaled=input_scaler(data_x)
    print(f"x: {data_x}")
    print(f"scaled_x: {data_x_scaled}")

    output_scaler=STDScaler()
    data_y_scaled=output_scaler(data_y)
    data_y_descaled=output_scaler.descale_data(data_y_scaled)
    print(f"y: {data_y}")
    print(f"scaled_y: {data_y_scaled}")
    print(f"descaled_y: {data_y_descaled}")
