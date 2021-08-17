import cpeplat as cpe
import numpy as np
import time
import cpeplat.result as res
import xmlrpc.client


class BuckParams:
    """A class with the basic model params for the buck PLECS simulation.
    This is a list containing the minimal set of parameters required to run
    the simulation.

    """
    def __init__(self, model_params={}):
        params = {}

        # Input-side
        params['V_in'] = 16
        params['L_in'] = 10e-9
        params['R_in'] = 30e-3

        params['C_in'] = 2 * 560e-6
        params['R_in'] = 60e-3 / 2

        # Switching
        params['R_ds'] = 15e-3
        params['f_pwm'] = 200e3

        # Buck
        params['L'] = 47e-6
        params['Rl'] = 15e-3

        params['C_out'] = 560e-6
        params['R_Cout'] = 60e-3

        # Output
        params['R_out'] = 3e-3
        params['R_load'] = 1.1
        params['R_dist'] = 2.2

        # Others
        params['t_sim'] = 10e-3
        params['t_r_dist_on'] = 6e-3
        params['t_r_dist_off'] = 8e-3
        
        params['V_ref'] = 8
        
        # Goes through each item defined in model_params. If this parameter
        # is already defined in the params variable, then the params variable
        # is overwritten. Otherwise, a new parameter is created.
        for param in model_params:
                params[param] = model_params[param]

        self.params = params


class Buck:
    """A class to provide an interface to the buck PLECS simulation.
        
    """
    def __init__(self, file=None, path=None, model_params={}):

        self.sim_file = file
        self.sim_path = path
        self.model = BuckParams(model_params)


    def sim(self, file=None, path=None, model_params={}):
        """Sets observer mode.

        Parameters
        ------
        file : str
            PLECS file to run the simulation. If `None`, runs the file first
            initialized.

        path : str
            Path where the file is located.

        params : dict
            A dictionary containing the simulation parameters.

        Returns
        -------
        data : np.array
            A N x M numpy array, where N is the number of time samples and M is
            the number of signals. The first column contains the time instants.
            The remaining columns depends on how the simulation is set up.
        Raises
        ------
        ValueError
            Raises `ValueError` if file or path is `None` and no file/path
            was initialized.
        
        """
        if file is None and self.sim_file is None:
            raise ValueError('No simulation file was defined.')

        if path is None and self.path is None:
            raise ValueError('No simulation path was defined.')
        
        if file is None:
            file = self.sim_file

        if path is None:
            path = self.path
            
        if model_params is {}:
            model_params = self.model.params

        # Opens PLECS connection
        server = xmlrpc.client.Server("http://localhost:1080/RPC2")
        server.plecs.load(path + '/' + file)

        # Setup model parameters
        plecs_params = {'ModelVars': model_params}

        # Runs simulations
        sim_data = server.plecs.simulate(file, plecs_params)

        # Retrieves sim data
        N = len(sim_data['Time'])
        M = len(sim_data['Values']) + 1

        data = np.zeros((N, M))

        data[:, 0] = np.array(sim_data['Time'])
        data[:, 1:] = np.array(sim_data['Values']).T
        
        return data
