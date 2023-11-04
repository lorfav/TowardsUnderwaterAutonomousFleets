import numpy as numpy
import matplotlib.pyplot as plt
import sys
import pandas
from casadi import *
import do_mpc

from Modello import *
from Controllore import *

rov = MyROVModel()

mpc = MyController(rov)

estimator = do_mpc.estimator.StateFeedback(rov.model)

simulator = do_mpc.simulator.Simulator(rov.model)

tvp_template = simulator.get_tvp_template()

simulator.set_tvp_fun(tvp_template)

params_simulator = {
    'integration_tool': 'idas',
    'abstol': 1e-10,
    'reltol': 1e-10,
    't_step': 0.1,
}

simulator.set_param(**params_simulator)

simulator.setup()

x0 = np.array([1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])

mpc.x0 = x0

estimator.x0 = x0

simulator.x0 = x0

mpc.mpc.set_initial_guess()

mpc_graphics = do_mpc.graphics.Graphics(mpc.mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator.data)