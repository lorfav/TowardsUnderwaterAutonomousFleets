
"""_________________________________________________________________________________________________
THIS IS A SIMULATOR FOR 3D PLOTTING OF POSITIONAL DATA OF TWO ROV's
____________________________________________________________________________________________________
The simulator plots the position of the ROV in 3D space and the orientation of the ROV as a vector.
The simulator can plot two ROVs at the same time, and gets the data from two csv files generated
by a MPC controller.
The simulator can also save the animation as a GIF file.

Made by: Towards Autonomous Underwater Fleets BCs-group 2023
____________________________________________________________________________________________________
References: 
https://stackoverflow.com/questions/55169099/animating-a-3d-vector-with-matplotlib
https://matplotlib.org/stable/gallery/animation/random_walk.html
https://coderslegacy.com/python/save-animations-in-matplotlib/
_________________________________________________________________________________________________"""

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import pandas as pd

############# Settings ############################################
csv_file_1_name = "data1"           #Name of csv file 1 without .csv
csv_file_2_name = "data2"           #Name of csv file 2 without .csv
saving_option = False               #True for saving animation as GIF
save_name = "With_hardcon"     #Name of saved file without .gif
fps_value = 30                      #Frames per second of GIF

############# Import and format data from csv file ################
#First ROV
df_1 = pd.read_csv("{}.csv".format(csv_file_1_name))
x_1 = list(df_1[df_1.columns[0]])
y_1 = list(df_1[df_1.columns[1]])
z_1 = list(df_1[df_1.columns[2]])
t_1 = list(range(len(x_1)))
t_1 = [float(i) for i in t_1]
phi_1 = list(df_1[df_1.columns[5]]) #Yaw
theta_1 = list(df_1[df_1.columns[4]]) #Pitch
x_sp_1 = list(df_1[df_1.columns[9]])
y_sp_1 = list(df_1[df_1.columns[10]])
z_sp_1 = list(df_1[df_1.columns[11]])
df_1 = pd.DataFrame({"time": t_1 ,"x" : x_1, "y" : y_1, "z" : z_1, "phi" : phi_1, "theta" : theta_1, "x_sp" : x_sp_1, "y_sp" : y_sp_1, "z_sp" : z_sp_1})

#Second ROV
df_2 = pd.read_csv("{}.csv".format(csv_file_2_name))
x_2 = list(df_2[df_2.columns[0]])
y_2 = list(df_2[df_2.columns[1]])
z_2 = list(df_2[df_2.columns[2]])
t_2 = list(range(len(x_2)))
t_2 = [float(i) for i in t_2]
phi_2 = list(df_2[df_2.columns[5]]) #Yaw
theta_2 = list(df_2[df_2.columns[4]]) #Pitch
x_sp_2 = list(df_2[df_2.columns[9]])
y_sp_2 = list(df_2[df_2.columns[10]])
z_sp_2 = list(df_2[df_2.columns[11]])
df_2 = pd.DataFrame({"time": t_2 ,"x" : x_2, "y" : y_2, "z" : z_2, "phi" : phi_2, "theta" : theta_2, "x_sp" : x_sp_2, "y_sp" : y_sp_2, "z_sp" : z_sp_2})

############# Functions ###########################################
def get_vector(phi, theta):
    """Get directional vector from oriantation angles phi and theta"""
    u = np.cos(phi)*np.cos(theta)
    v = np.sin(phi)*np.cos(theta)
    w = np.sin(theta)
    return [u,v,w]

def plot_vector(x_1, y_1, z_1, x_2, y_2, z_2, phi_1, theta_1, phi_2, theta_2):
    """Function for plotting directional vector"""
    global vector_1
    global vector_2
    vector_1.remove()
    vector_2.remove()
    vector_1 = ax.quiver(x_1,y_1,z_1,*get_vector(phi_1, theta_1), length=3, normalize=True, color="blue")
    vector_2 = ax.quiver(x_2,y_2,z_2,*get_vector(phi_2, theta_2), length=3, normalize=True, color="green")

def get_position_from_dataframe(df, num):
    """Function for getting position from dataframe"""
    data = df[df['time']==num]
    x = data.x.values
    y = data.y.values
    z = data.z.values
    x_sp = data.x_sp.values
    y_sp = data.y_sp.values
    z_sp = data.z_sp.values
    theta = data.theta.values[0]
    phi = data.phi.values[0]
    return x, y, z, theta, phi, x_sp, y_sp, z_sp

def distance_rov_1_to_2(x_1, y_1, z_1, x_2, y_2, z_2):
    """Function for calculating distance between ROV 1 and ROV 2"""
    return np.sqrt((x_1-x_2)**2 + (y_1-y_2)**2 + (z_1-z_2)**2)

def distance_sp(x_sp, y_sp, z_sp, x, y, z):
    """Function for calculating distance between setpoints of ROV 1 and ROV 2"""
    return np.sqrt((x_sp-x)**2 + (y_sp-y)**2 + (z_sp-z)**2)

def update_graph(num):
    """Animation function for updating the figure. Runs in a loop"""
    ## Get states from dataframe
    x_1, y_1, z_1, theta_1, phi_1, x_sp_1, y_sp_1, z_sp_1 = get_position_from_dataframe(df_1, num)
    x_2, y_2, z_2, theta_2, phi_2, x_sp_2, y_sp_2, z_sp_2 = get_position_from_dataframe(df_2, num)
    ## Plotting directional vector
    plot_vector(x_1, y_1, z_1, x_2, y_2, z_2, phi_1, theta_1, phi_2, theta_2)
    ## Plotting position of ROV
    graph_1.set_data (x_1, y_1)
    graph_1.set_3d_properties(z_1)
    graph_2.set_data (x_2, y_2)
    graph_2.set_3d_properties(z_2)

    graph_1_sp.set_data (x_sp_1, y_sp_1)
    graph_1_sp.set_3d_properties(z_sp_1)
    #graph_2_sp.set_data (x_sp_2, y_sp_2)
    #graph_2_sp.set_3d_properties(z_sp_2)
    ## Plotting text
    plt.legend(loc="upper left")
    sample_count.set_text("Step nr.:{}".format(num))
    distance_rov = float(distance_rov_1_to_2(x_1, y_1, z_1, x_2, y_2, z_2))
    distance_sp_1 = float(distance_sp(x_sp_1, y_sp_1, z_sp_1, x_1, y_1, z_1))
    distance_sp_2 = float(distance_sp(x_sp_2, y_sp_2, z_sp_2, x_2, y_2, z_2))
    #print("SP: ({}, {}, {})\tDistance ROV 1 to ROV 2: {}\tDistance SP 1 to ROV 1: {}\tDistance SP 2 to ROV 2: {}".format(x_sp_1, y_sp_1, z_sp_1, f'{distance_rov:10.2f}',f'{distance_sp_1:10.2f}',f'{distance_sp_2:10.2f}'))

    title.set_text("MPC controlled ROV \nAgent1 pos: ({}, {}, {}) \nAgent2 pos: ({}, {}, {})\nDist ROV's: {}, Dist1 to SP: {}, Dist2 to SP: {}".format(f'{x_1[0]:10.2f}', f'{y_1[0]:10.2f}', f'{z_1[0]:10.2f}', f'{x_2[0]:10.2f}', f'{y_2[0]:10.2f}', f'{z_2[0]:10.2f}', f'{distance_rov:10.2f}', f'{distance_sp_1:10.2f}', f'{distance_sp_2:10.2f}'))
    return title, graph_1, graph_2, sample_count, graph_1_sp#, graph_2_sp

############# Setting up the figure ###############################
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
## Setting up limits and labels
ax.set(xlim3d=(-20, 20), xlabel='X')
ax.set(ylim3d=(-20, 20), ylabel='Y')
ax.set(zlim3d=(-20, 20), zlabel='Z')
## Setting up text
title = ax.set_title('MPC plot')
sample_count = ax.text(0, 0, 12, "", fontsize=10, color="Black")
## Setting up directional vector
vector_1 = ax.quiver(1,1,1,1,1,1, length=3, normalize=True, color="blue")
vector_2 = ax.quiver(1,1,1,1,1,1, length=3, normalize=True, color="green")
## Setting up position of ROV
data_1 = df_1[df_1['time']==0]
data_2 = df_2[df_2['time']==0]
graph_1, = ax.plot(data_1.x, data_1.y, data_1.z, linestyle="", marker="o", color="blue", label="Agent1", animated=True, markersize=9)
graph_2, = ax.plot(data_2.x, data_2.y, data_2.z, linestyle="", marker="o", color="green", label="Agent2", animated=True, markersize=9)  

graph_1_sp, = ax.plot(data_1.x_sp, data_1.y_sp, data_1.z_sp, linestyle="", marker="o", color="red", markersize=2, label="Setpoint")
#graph_2_sp, = ax.plot(data_2.x_sp, data_2.y_sp, data_2.z_sp, linestyle="", marker="o", color="green",markersize=2)  
## Setting up animation
ani = animation.FuncAnimation(fig, update_graph, len(t_1), 
                               interval=10, blit=True,)
############# Saving animation ####################################
## Saving animation as .GIF

if(saving_option):
    filnavn = save_name+".gif"
    ani.save(filnavn, writer='imagemagick', fps=fps_value)


plt.show()

