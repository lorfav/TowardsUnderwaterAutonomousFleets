### Simulator file, imports controller and model ###
import numpy as np
import matplotlib.pyplot as plt
import sys
from casadi import *
import pandas as pd
from scipy.spatial.transform import Rotation

import do_mpc

from rovModel_Current import * #current in the model
#from rovModel import * # no current in the model

from rovController import *
import math


def sine_wave(t):
    """ Sine wave along the x axis, oscilllating in the z axis """
    z = 1*np.sin(np.pi*t/10)
    x = t/10
    y = 0
    return x,y,z 

def vector_between_rovs(x1,y1,z1,x2,y2,z2):
    """ Returns vector defined by two points """
    x = (x2-x1)
    y = (y2-y1)
    z = (z2-z1)
    return [x, y, z]

def x_directional_vector_from_quaternion(q0, e1, e2, e3):
    """ Returns the x-directional vector from quaternion orientation """
    x = 1-(2*e2**2+2*e3**2)
    y = 2*e1*e2+2*e3*q0
    z = 2*e1*e3-2*e2*q0
    return [x, y, z]

def quarternion_reference_other_ROV(x2,y2,z2,x1,y1,z1,q0,e1,e2,e3):
    """ Return """
        vector =  [float(x2-x1), float(y2-y1), float(z2-z1)]
        vector = vector / np.linalg.norm(vector)
        theta = np.arccos(np.dot([1, 0, 0], vector))
        axis = np.cross([1, 0, 0], vector)
        if(sum(axis) != 0):
            axis = axis / np.linalg.norm(axis)
            q_0_setp = np.cos(theta/2)
            e_1_setp = axis[0] * np.sin(theta/2)
            e_2_setp = axis[1] * np.sin(theta/2)
            e_3_setp = axis[2] * np.sin(theta/2)
            return [q_0_setp, e_1_setp, e_2_setp, e_3_setp]
        else:
            return [q0, e1, e2, e3]


def euler_from_quaternion(w, x, y, z):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def tvp_fun1(t_now):
    return(tvp_template1)
def tvp_fun2(t_now):
    return(tvp_template2)
#Creates parameters for cubic polynomial path
def trajectory_parameter_generation(x0,dx0,x1,dx1, t0, t1):
    
    A =np.array([[1,t0, t0**2, t0**3],
                [0, 1, 2*t0, 3*t0**2],
                [1, t1, t1**2, t1**3],
                [0, 1, 2*t1, 3*t1**2]])

    #b = np.array = ([a0,a1,a2,a3]).reshape(-1,1)
    y = np.array([x0,dx0,x1,dx1]).reshape(-1,1)
    b = np.linalg.inv(A)@y
    return b
#Takes in the parameters as a list, and time t, returns the setpoint at that given time.
def trajectory_generation(a,t):
    """ Returns the position of a cubic polynomial with parameters a at time t"""
    qt = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3
    return qt

def init_path():
    x=[]
    y=[]
    z=[]
    u=[]
    v=[]
    w=[]
    t=[]
    n_waypoints = int(input("Enter number of waypoints: "))
    for i in range(n_waypoints):
        inp = input("enter waypoint " +str(i+1)+": x y z u v w t: ")
        inp = list(map(int, inp.split()))

        x.append(inp[0])
        y.append(inp[1])
        z.append(inp[2])
        u.append(inp[3])
        v.append(inp[4])
        w.append(inp[5])
        t.append(inp[6])

    return x,y,z,u,v,w,t    
        
modelRov1 = MyROVModel()
modelRov2 = MyROVModel()

mpc1 = MyController(modelRov1,2,[1,0,0,0.5,0.5,0.5,0.5,0,0,0,0,0,0,0,0,0])
mpc2 = MyController(modelRov2,2,[2,0,0,0.5,0.5,0.5,0.5,0,0,0,0,0,0,0,0,0])
estimator1 = do_mpc.estimator.StateFeedback(modelRov1.model)
estimator2 = do_mpc.estimator.StateFeedback(modelRov1.model)


simulator1 = do_mpc.simulator.Simulator(modelRov1.model)
simulator2 = do_mpc.simulator.Simulator(modelRov2.model)

tvp_template1 = simulator1.get_tvp_template()
tvp_template2 = simulator2.get_tvp_template()

simulator1.set_tvp_fun(tvp_fun1)
simulator2.set_tvp_fun(tvp_fun2)

params_simulator = {
    # Note: cvode doesn't support DAE systems.
    'integration_tool': 'idas',
    'abstol': 1e-10,
    'reltol': 1e-10,
    't_step': 0.1,

}

simulator1.set_param(**params_simulator)
simulator2.set_param(**params_simulator)

simulator1.setup()
simulator2.setup()

#x0 = np.array([20, -11.4, -1.5, 10, 20, 20, -10, 1,1,2,3,4]).reshape(-1,1)
#               x,y,z,q_0,e_1,e_2,e_3,u,v,w,p,q,r
x0_1 = np.array([4, 2, 0, 1,0,0,0, 0,0,0,0,0,0]).reshape(-1,1)
x0_2 = np.array([1, 0, 0, 1,0,0,0, 0,0,0,0,0,0]).reshape(-1,1)

mpc1.x0 = x0_1
mpc2.x0 = x0_2

estimator1.x0 = x0_1
simulator1.x0 = x0_1

estimator2.x0 = x0_2
simulator2.x0 = x0_2

mpc1.mpc.set_initial_guess()
mpc2.mpc.set_initial_guess()

mpc_graphics = do_mpc.graphics.Graphics(mpc1.mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator1.data)

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
    
    g.add_line(var_type='_x', var_name='q_0', axis=ax[2])
    g.add_line(var_type='_x', var_name='e_1', axis=ax[2])
    g.add_line(var_type='_x', var_name='e_2', axis=ax[2])
    g.add_line(var_type='_x', var_name='e_3', axis=ax[2])
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

#u0 = mpc.make_step(x0)
#y_next = simulator.make_step(u0)

plot1 = []
plot2 = []

quart1 = []
quart2 = []

quart_sp1 = []
quart_sp2 = []

u0_1 = np.zeros((8,1))
u0_2 = np.zeros((8,1))

max_itr = 100
for i in range(max_itr):
    mpc1.u_c = 0
    mpc1.v_c = 0
    mpc1.w_c = 0
    print(x0_1)
    print(x0_2)
    u0_1 = mpc1.mpc.make_step(x0_1)
    u0_2 = mpc2.mpc.make_step(x0_2)

    y_next_1 = simulator1.make_step(u0_1)
    y_next_2 = simulator2.make_step(u0_2)
    
    x0_1 = estimator1.make_step(y_next_1)
    x0_2 = estimator2.make_step(y_next_2)

    quart1.append(x0_1[3:7])
    quart2.append(x0_2[3:7])

    mpc1.x_2 = x0_2[0]
    mpc1.y_2 = x0_2[1]
    mpc1.z_2 = x0_2[2]

    mpc2.x_2 = x0_1[0]
    mpc2.y_2 = x0_1[1]
    mpc2.z_2 = x0_1[2]

    quart_ref_1 = quarternion_reference_other_ROV(x0_2[0],x0_2[1],x0_2[2], x0_1[0], x0_1[1], x0_1[2], x0_1[3], x0_1[4], x0_1[5], x0_1[6])
    quart_ref_2 = quarternion_reference_other_ROV(x0_1[0],x0_1[1],x0_1[2], x0_2[0], x0_2[1], x0_2[2], x0_2[3], x0_2[4], x0_2[5], x0_2[6])
    #mpc2.q_0_setp = quart_ref_2[0]
    #mpc2.e_1_setp = quart_ref_2[1]
    #mpc2.e_2_setp = quart_ref_2[2]
    #mpc2.e_3_setp = quart_ref_2[3]
#
    #quart_sp1.append([mpc1.q_0_setp, mpc1.e_1_setp, mpc1.e_2_setp, mpc1.e_3_setp])
    #quart_sp2.append([mpc2.q_0_setp, mpc2.e_1_setp, mpc2.e_2_setp, mpc2.e_3_setp])



    if i == 100:
        mpc1.x_setp += 1
        mpc1.y_setp += 1
        mpc1.q_0_setp = 0
        mpc1.e_1_setp = 1
        mpc1.e_2_setp = 0
        mpc1.e_3_setp = 0
        mpc2.q_0_setp = 0
        mpc2.e_1_setp = 0
        mpc2.e_2_setp = 1
        mpc2.e_3_setp = 0
    if i == 200:
        mpc2.x_setp -= 7
        mpc2.y_setp += 7

    for k in range(35):
        print("\t\t\t\t\t\t\t\t\t\t\t\tIteration {}/{}:\t".format(i,max_itr))

    x0_1_euler = np.copy(x0_1)
    x0_1_euler[3] = euler_from_quaternion(x0_1[3], x0_1[4], x0_1[5], x0_1[6])[0]
    x0_1_euler[4] = euler_from_quaternion(x0_1[3], x0_1[4], x0_1[5], x0_1[6])[1]
    x0_1_euler[5] = euler_from_quaternion(x0_1[3], x0_1[4], x0_1[5], x0_1[6])[2]
    x0_1_euler = np.delete(x0_1_euler, 6)



    x0_2_euler = np.copy(x0_2)
    x0_2_euler[3] = euler_from_quaternion(x0_2[3], x0_2[4], x0_2[5], x0_2[6])[0]
    x0_2_euler[4] = euler_from_quaternion(x0_2[3], x0_2[4], x0_2[5], x0_2[6])[1]
    x0_2_euler[5] = euler_from_quaternion(x0_2[3], x0_2[4], x0_2[5], x0_2[6])[2]
    x0_2_euler = np.delete(x0_2_euler, 6)

    
    plot1.append(x0_1_euler)
    plot2.append(x0_2_euler)


######################### DETTE ER FOR PLOT ########################################
for i in range(len(plot1)):
    plot1[i] = [float(plot1[i][j]) for j in range(len(plot1[i]))]
data = [list(plot1[i]) for i in range(len(plot1))]
df = pd.DataFrame(data, columns=['x','y','z','phi','theta','psi','u','v','w','p','q','r'])
df.to_csv('data1.csv', index=False)
print(df)
#####################################################################################
for i in range(len(plot2)):
    plot2[i] = [float(plot2[i][j]) for j in range(len(plot2[i]))]
data = [list(plot2[i]) for i in range(len(plot2))]
df = pd.DataFrame(data, columns=['x','y','z','phi','theta','psi','u','v','w','p','q','r'])
df.to_csv('data2.csv', index=False)
print(df)
###################################################################################
for i in range(len(plot2)):
    plot2[i] = [float(plot2[i][j]) for j in range(len(plot2[i]))]
data = [list(plot2[i]) for i in range(len(plot2))]
df = pd.DataFrame(data, columns=['x','y','z','phi','theta','psi','u','v','w','p','q','r'])
df.to_csv('data2.csv', index=False)
print(df)
###################################################################################
for i in range(len(quart1)):
    quart1[i] = [float(quart1[i][j]) for j in range(len(quart1[i]))]
data = [list(quart1[i]) for i in range(len(quart1))]
df = pd.DataFrame(data, columns=['q0','q1','q2','q3'])
df.to_csv('quart1.csv', index=False)
###################################################################################
for i in range(len(quart2)):
    quart2[i] = [float(quart2[i][j]) for j in range(len(quart2[i]))]
data = [list(quart2[i]) for i in range(len(quart2))]
df = pd.DataFrame(data, columns=['q0','q1','q2','q3'])
df.to_csv('quart2.csv', index=False)
###################################################################################
for i in range(len(quart_sp1)):
    quart_sp1[i] = [float(quart_sp1[i][j]) for j in range(len(quart_sp1[i]))]
data = [list(quart_sp1[i]) for i in range(len(quart_sp1))]
df = pd.DataFrame(data, columns=['q0','q1','q2','q3'])
df.to_csv('quart_sp1.csv', index=False)
###################################################################################
for i in range(len(quart_sp2)):
    quart_sp2[i] = [float(quart_sp2[i][j]) for j in range(len(quart_sp2[i]))]
data = [list(quart_sp2[i]) for i in range(len(quart_sp2))]
df = pd.DataFrame(data, columns=['q0','q1','q2','q3'])
df.to_csv('quart_sp2.csv', index=False)



lines = (sim_graphics.result_lines['_x', 'x']+
        sim_graphics.result_lines['_x', 'y']+
        sim_graphics.result_lines['_x', 'z']+
        sim_graphics.result_lines['_x', 'u']+
        sim_graphics.result_lines['_x', 'v']+
        sim_graphics.result_lines['_x', 'w']
        )
ax[0].legend(lines,'xyzuvw',title='position')

lines = (sim_graphics.result_lines['_u', 'u_1']+
        sim_graphics.result_lines['_u', 'u_2']+
        sim_graphics.result_lines['_u', 'u_3']+
        sim_graphics.result_lines['_u', 'u_4']+
        sim_graphics.result_lines['_u', 'u_5']+
        sim_graphics.result_lines['_u', 'u_6']+
        sim_graphics.result_lines['_u', 'u_7']+
        sim_graphics.result_lines['_u', 'u_8']
        )

ax[1].legend(lines,'12345678',title='input')

lines = (sim_graphics.result_lines['_x', 'q_0']+
        sim_graphics.result_lines['_x', 'e_1']+
        sim_graphics.result_lines['_x', 'e_2']+
        sim_graphics.result_lines['_x', 'e_3'])
       # sim_graphics.result_lines['_x', 'p']+
       # sim_graphics.result_lines['_x', 'q']+
       # sim_graphics.result_lines['_x', 'r'])
ax[2].legend(lines,'0123pqr',title='quaternions')
sim_graphics.plot_results()

sim_graphics.reset_axes()

plt.show()
