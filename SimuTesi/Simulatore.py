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

fig, ax = plt.subplots(3, sharex=True)
fig.align_ylabels

for g in [sim_graphics, mpc_graphics]:
    # Plot the angle positions (phi_1, phi_2, phi_2) on the first axis:
    g.add_line(var_type='_x', var_name='x', axis=ax[0])
    g.add_line(var_type='_x', var_name='y', axis=ax[0])
    g.add_line(var_type='_x', var_name='z', axis=ax[0])
    g.add_line(var_type='_x', var_name='u', axis=ax[0])
    g.add_line(var_type='_x', var_name='v', axis=ax[0])
    g.add_line(var_type='_x', var_name='w', axis=ax[0])
    
    g.add_line(var_type='_x', var_name='phi', axis=ax[2])
    g.add_line(var_type='_x', var_name='theta', axis=ax[2])
    g.add_line(var_type='_x', var_name='psi', axis=ax[2])
    g.add_line(var_type='_x', var_name='p', axis=ax[2])
    g.add_line(var_type='_x', var_name='q', axis=ax[2])
    g.add_line(var_type='_x', var_name='r', axis=ax[2])

    g.add_line(var_type='_u', var_name='u_1', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_2', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_3', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_4', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_5', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_6', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_7', axis=ax[1])
    g.add_line(var_type='_u', var_name='u_8', axis=ax[1])

    # Plot the set motor positions (phi_m_1_set, phi_m_2_set) on the second axis:

ax[0].set_ylabel('Position [m], velocity [m/s]')
ax[1].set_ylabel('Input [N]')
ax[2].set_ylabel('Angle [rad]')

plot1 = []
plot2 = []

setpoint1 = [[],[],[]]

u0 = np.zeros((8,1))
j = 0

n_sim = 1300

function_line = True
firt_itr = True

rot_count = 0

for i in range(n_sim):

    for i in range(30):
        print("\t\t\t\t\t\t\t\t\t\t\t\t{}/{}".format(j,n_sim))
    
    j += 1

    u0 = mpc.mpc.make_step(x0)

    y_next = simulator.make_step(u0)

    x0 = estimator.make_step(y_next)

