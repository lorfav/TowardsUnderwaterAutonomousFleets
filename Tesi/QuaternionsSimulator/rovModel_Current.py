### This file contains the ROV model, using the relative velocity vector to account for currents ###

import numpy as np
from casadi import *

import do_mpc

I3x3 = np.diag([1]*3) #3x3 Identity matrix

def skew(x):
    x = x.reshape(1,-1)[0]
    print(x)
    return np.array([[0, -x[2], x[1]],
                    [x[2], 0, -x[0]],
                    [-x[1], x[0], 0]])

def Combine4(M11, M12, M21, M22):
    return np.array(np.concatenate((np.concatenate((M11, M12), axis=1),
                                    np.concatenate((M21, M22), axis=1))))


def transp_rot_quat(vec):
    np.transpose(I3x3 + 2*vec[0,0]*skew(vec[1:]) + 2*skew(vec[1:])**2)

model_type = 'continuous'
class MyROVModel():
    def __init__(self):
        self.model = do_mpc.model.Model(model_type)

        ########## PARAMETERS #############      

        m = 11.5                    # Kg
        W = 112.8                   # m*g, Newton
        #W = 114.8                   # Newton neutral buoyancy
        B = 114.8                   # buoyancy, Newton
        r_b = np.array([[0],        # m centre of buoyancy 
                        [0],
                        [0]])
        r_g = np.array([[0],        # m centre of gravity
                        [0],
                        [0.02]])
        
        z_g = r_g[2]
        y_g = 0
        x_g = 0
        #inertia
        I_x = 0.16         *1         # Kg m**2
        I_y = 0.16         *1         # Kg m**2
        I_z = 0.16         *1         # Kg m**2
        
        # Linear damping
        X_u = -4.03       *1          # Ns/m
        Y_v = -6.22       *1          # Ns/m
        Z_w = -5.18       *1
        K_p = -0.07       *1          # Ns/m
        M_q = -0.07       *1         # Ns/m
        N_r = -0.07       *1          # Ns/m
        
        #Quadratic dampenining
        X_u_abs = -18.18   *1         # Ns**2/m**2
        Y_v_abs = -21.66   *1         # Ns**2/m**2
        Z_w_abs = -36.99   *1         # Ns**2/m**2
        K_p_abs = -1.55    *1         # Ns**2/m**2
        M_q_abs = -1.55    *1        # Ns**2/m**2
        N_r_abs = -1.55    *1 
        
        #added mass
        X_udot = -5.5      *1         # Kg
        Y_vdot = -12.7     *1         # Kg
        Z_wdot = -14.57    *1         # Kg
        K_pdot = -0.12     *1        # Kg m**2/rad
        M_qdot = -0.12     *1        # Kg m**2/rad
        N_rdot = -0.12     *1         # Kg m**2/rad
        
        
        
        #STATES

        #position
        x = self.model.set_variable('_x', 'x')
        y = self.model.set_variable('_x', 'y')
        z = self.model.set_variable('_x', 'z')

        #attitude quaternions
        q_0 = self.model.set_variable('_x','q_0')
        e_1 = self.model.set_variable('_x','e_1')
        e_2 = self.model.set_variable('_x','e_2')
        e_3 = self.model.set_variable('_x','e_3')

        #linear velocity
        u = self.model.set_variable('_x', 'u')
        v = self.model.set_variable('_x', 'v')
        w = self.model.set_variable('_x', 'w')
        
        #linear accelerations
        u_dot = self.model.set_variable('_z', 'u_dot')
        v_dot = self.model.set_variable('_z', 'v_dot')
        w_dot = self.model.set_variable('_z', 'w_dot')
    
        #angular velocity
        p = self.model.set_variable('_x', 'p')
        q = self.model.set_variable('_x', 'q')
        r = self.model.set_variable('_x', 'r')
        
        #angular acceleration
        p_dot = self.model.set_variable('_z', 'p_dot')
        q_dot = self.model.set_variable('_z', 'q_dot')
        r_dot = self.model.set_variable('_z', 'r_dot')

        #input to thrusters
        u_1 = self.model.set_variable('_u', 'u_1')
        u_2 = self.model.set_variable('_u', 'u_2')
        u_3 = self.model.set_variable('_u', 'u_3')
        u_4 = self.model.set_variable('_u', 'u_4')
        u_5 = self.model.set_variable('_u', 'u_5')
        u_6 = self.model.set_variable('_u', 'u_6')
        u_7 = self.model.set_variable('_u', 'u_7')
        u_8 = self.model.set_variable('_u', 'u_8')
       #u5 og u8 hadde ogsaa minusfortegn 
        

        #Time-varying parameters for setpoints
        x_sp = self.model.set_variable('_tvp', 'x_sp')
        y_sp = self.model.set_variable('_tvp', 'y_sp')
        z_sp = self.model.set_variable('_tvp', 'z_sp')
        q_0_sp = self.model.set_variable('_tvp', 'q_0_sp')
        e_1_sp = self.model.set_variable('_tvp', 'e_1_sp')
        e_2_sp = self.model.set_variable('_tvp', 'e_2_sp')        
        e_3_sp = self.model.set_variable('_tvp', 'e_3_sp')
        
        #States of other ROVs implemented as time-varying parameters
        x_2 = self.model.set_variable('_tvp', 'x_2') 
        y_2 = self.model.set_variable('_tvp', 'y_2') 
        z_2 = self.model.set_variable('_tvp', 'z_2')    
        x_3 = self.model.set_variable('_tvp', 'x_3') 
        y_3 = self.model.set_variable('_tvp', 'y_3') 
        z_3 = self.model.set_variable('_tvp', 'z_3')  
#        x_4 = self.model.set_variable('_tvp', 'x_4') 
#        y_4 = self.model.set_variable('_tvp', 'y_4') 
#        z_4 = self.model.set_variable('_tvp', 'z_4')    
#        x_5 = self.model.set_variable('_tvp', 'x_5') 
#        y_5 = self.model.set_variable('_tvp', 'y_5') 
#        z_5 = self.model.set_variable('_tvp', 'z_5')
#        x_6 = self.model.set_variable('_tvp', 'x_6') 
#        y_6 = self.model.set_variable('_tvp', 'y_6') 
#        z_6 = self.model.set_variable('_tvp', 'z_6')   
        
        # Ocean currents ENU
        #u_enu_c = self.model.set_variable('_tvp', 'u_enu_c')
        #v_enu_c = self.model.set_variable('_tvp', 'v_enu_c')
        #w_enu_c = self.model.set_variable('_tvp', 'w_enu_c')

        # Ocean current BODY
        u_c = self.model.set_variable('_tvp', 'u_c')
        v_c = self.model.set_variable('_tvp', 'v_c')
        w_c = self.model.set_variable('_tvp', 'w_c')
        #u_c = 0
        #v_c = 0
        #w_c = 0
        #u_c_dot = self.model.set_variable('_z', 'u_c_dot')
        #v_c_dot = self.model.set_variable('_z', 'v_c_dot')
        #w_c_dot = self.model.set_variable('_z', 'w_c_dot')
        #nu_c = model.set_variable(var_type='_x', var_name='dphi', shape=(3,1))
        #Quaternion rotation matrix translating change in body frame to NED
        
        #Quaternion rotation matrix translating change in body frame to NED
        self.model.set_rhs('x',(1 - 2*(e_2**2 + e_3**2))*u + 2*(e_1*e_2 - e_3*q_0)*v + 2*(e_1*e_3 + e_2*q_0)*w )
        self.model.set_rhs('y', 2*(e_1*e_2 + e_3*q_0)*u + (1 - 2*(e_1**2 + e_3**2))*v + 2*(e_2*e_3 - e_1*q_0)*w)
        self.model.set_rhs('z', 2*(e_1*e_3 - e_2*q_0)*u + 2*(e_2*e_3 + e_1*q_0)*v + (1 - 2*(e_1**2 + e_2**2))*w)
        
        # Translating change in attitude from body to NED
        self.model.set_rhs('q_0', 0.5*(-e_1*p - e_2*q - e_3*r))
        self.model.set_rhs('e_1', 0.5*(q_0*p - e_3*q + e_2*r))
        self.model.set_rhs('e_2', 0.5*(e_3*p + q_0*q - e_1*r))
        self.model.set_rhs('e_3', 0.5*(-e_2*p + e_1*q + q_0*r))

        # Change in rotational velocity
        self.model.set_rhs('p', p_dot)
        self.model.set_rhs('q', q_dot)
        self.model.set_rhs('r', r_dot)

        # Change in linear velocity
        self.model.set_rhs('u', u_dot)
        self.model.set_rhs('v', v_dot)
        self.model.set_rhs('w', w_dot)
        
        nu_dot_vec = np.array([u_dot,v_dot,w_dot,p_dot,q_dot,r_dot]).reshape(-1,1)
        nu_vec = np.array([u,v,w,p,q,r]).reshape(-1,1)
        q_vec = np.array([q_0,e_1,e_2,e_3]).reshape(-1,1)
        nu_c_vec = np.array([u_c,v_c,w_c,0,0,0]).reshape(-1,1) 
        #nu_enu_c_vec = np.array([u_enu_c,v_enu_c,w_enu_c]).reshape(-1,1) 
        #nu_c_dot_vec = np.array([u_c_dot,v_c_dot,w_c_dot,0,0,0]).reshape(-1,1) 

        rot_enu_ned = np.array([[0 , 1, 0], #rotation matrix from ENU frame to NED
                                [1, 0, 0],
                                [0 ,0 , -1]]) 



        I_b = np.diag((I_x, I_y, I_z)) #Inertia dyadic

        #System Inertia matrix

        M_rb_11 = np.array(m*I3x3)
        M_rb_12 = np.array(-m*skew(r_g))
        M_rb_21 = -np.array(-m*skew(r_g))
        M_rb_22 = I_b

        M_rb = Combine4(M_rb_11, M_rb_12, M_rb_21, M_rb_22)
        
       # print((M_rb@nu_dot_vec)[3,0])
                #, - m*skew(r_g)],
                #        m*skew(r_g), []])


        # C_rb Linear velocity inpedendent paramtrization
        C_rb_11 = m*skew(nu_vec[3:])
        C_rb_12 = -m*skew(nu_vec[3:])@skew(r_g)
        C_rb_21 = m*skew(r_g)@skew(nu_vec[3:])
        C_rb_22 = -skew(I_b@nu_vec[3:])
        
        C_rb = Combine4(C_rb_11, C_rb_12, C_rb_21, C_rb_22)

        #Added mass
        M_a = -np.diag((X_udot,Y_vdot,Z_wdot,K_pdot,M_qdot,N_rdot))


        #Added Coriolis
        A_11 = M_a[:3,:3]
        A_12 = M_a[:3,3:]
        A_21 = M_a[3:,:3]
        A_22 = M_a[3:,3:]

        C_a_11 = np.diag((0,0,0))
        C_a_12 = -skew(A_11@(nu_vec[:3] - nu_c_vec[:3]) + A_12@nu_vec[3:])
        C_a_21 = -skew(A_11@(nu_vec[:3] - nu_c_vec[:3]) + A_12@nu_vec[3:])
        C_a_22 = -skew(A_21@(nu_vec[:3] - nu_c_vec[:3]) + A_22@nu_vec[3:])
        
        C_a = Combine4(C_a_11, C_a_12, C_a_21, C_a_22)
        print(C_a)

        # Linear damping
        D_l = -np.diag((X_u,Y_v,Z_w,K_p,M_q,N_r))



        #rot_quat = I3x3 + 2*q_vec[0]*skew(q_vec[1:]) + 2*skew(q_vec[1:])**2 #quaternion rotation matrix
        #nu_c_vec = rot_enu_ned@np.transpose(rot_quat)@nu_enu_c_vec #translate current from enu to body
        #print(type(nu_c_vec))
        zerovec = np.array([[0],[0],[0]])
        #print(type(rot_quat))
        #nu_c_vec = np.concatenate((nu_c_vec,zerovec),axis=0)



        #print(nu_c_vec)
        #### MATRICES ############### (multiplied out)
        
        #hydrostatics
        #g_1 = (B-W)*(2*e_1*e_3 - 2*e_2*q_0)
        #g_2 = (B-W)*(2*e_2*e_3 - 2*e_1*q_0)
        #g_3 = (W-B)*(2*e_1**2 + 2*e_2**2 -1)
        #g_4 = z_g*W*(2*e_2*e_3 + 2*e_1*q_0)
        #g_5 = z_g*W*(2*e_1*e_3 - 2*e_2*q_0)
        #g_6 = 0
        g_1 = (B-W)*(2*e_1*e_3 - 2*e_2*q_0)
        g_2 = (B-W)*(2*e_2*e_3 + 2*e_1*q_0)
        g_3 = -(B-W)*(2*e_1**2 + 2*e_2**2 -1)
        g_4 = W*z_g*(2*e_2*e_3 + 2*e_1*q_0)
        g_5 = -W*z_g*(2*e_1*e_3 - 2*e_2*q_0)
        g_6 = 0
        
        #tau forces
        #tau_1 = (0.707*u_1 + 0.707*u_2-0.707*u_3-0.707*u_4)
        #tau_2 =(-0.707*u_1 + 0.707*u_2-0.707*u_3 + 0.707*u_4)
        #tau_3 =( u_5 + u_6 + u_7 + u_8)
        #tau_4 =(0.06*u_1 - 0.06*u_2 + 0.06*u_3 - 0.06*u_4 -0.218*u_5 - 0.218*u_6 + 0.218*u_7 + 0.218*u_8)
        #tau_5 =(0.06*u_1 + 0.06*u_2 - 0.06*u_3 - 0.06*u_4 + 0.120*u_5 - 0.120*u_6 + 0.120*u_7 - 0.120*u_8)
        #tau_6 =(-0.1888*u_1 + 0.1888*u_2 + 0.1888*u_3 - 0.1888*u_4)
        
        #Tau matrix, inputs
        tau_1 = (0.707*u_1 + 0.707*u_2-0.707*u_3-0.707*u_4) #x
        tau_2 =(-0.707*u_1 + 0.707*u_2-0.707*u_3 + 0.707*u_4) #y
        tau_3 =( -u_5 + u_6 + u_7 - u_8)                        #z
        tau_4 =(0.06*u_1 - 0.06*u_2 + 0.06*u_3 - 0.06*u_4 -0.218*u_5 - 0.218*u_6 + 0.218*u_7 + 0.218*u_8) #p
        tau_5 =(0.06*u_1 + 0.06*u_2 - 0.06*u_3 - 0.06*u_4 + 0.120*u_5 - 0.120*u_6 + 0.120*u_7 - 0.120*u_8) #q
        tau_6 =(-0.1888*u_1 + 0.1888*u_2 + 0.1888*u_3 - 0.1888*u_4) #r

        #M_rb, Rigid-body mass matrix 
        M_rb_1 = m*(u_dot + z_g*q_dot)
        M_rb_2 = m*(v_dot - z_g*p_dot)
        M_rb_3 = m*w_dot
        M_rb_4 = -m*z_g*v_dot + I_x*p_dot
        M_rb_5 = m*z_g*u_dot + I_y*q_dot
        M_rb_6 = I_z*r_dot
        
        #C_rb Rigid-body coriolis and centripital matrix
        #C_rb_1 = m*w*q
        #C_rb_2 =  -m*w*p
        #C_rb_3 =  m*(v*p - u*q)
        #C_rb_4 =  m*(w*v - v*w) +I_z*r*q - I_y*q*r
        #C_rb_5 = -m*(w*u - u*w) - I_z*r*p + I_x*p*r
        #C_rb_6 = m*(v*u - u*v) + I_y*q*p -I_x*p*q
        C_rb_1 = m*w*q + (m*p*z_g-m*v)*r
        C_rb_2 = -m*w*p + (m*u + m*q*z_g)*r
        C_rb_3 =  m*((v - p*z_g)*p + (-u - q*z_g)*q)
        C_rb_4 =  m*(w*v + (p*z_g - v)*w) +I_z*r*q  + (- I_y*q - m*u*z_g)*r
        C_rb_5 = m*(-w*u + (u + q*z_g)*w) - I_z*r*p + (I_x*p - m*v*z_g)*r
        C_rb_6 = m*((v - p*z_g)*u + (-u -q*z_g)*v) + (I_y*q + m*u*z_g)*p  + (m*v*z_g - I_x*p)*q
        
        
        # M_a Added mass matrix
        M_a_1 = -X_udot*u_dot
        M_a_2 = -Y_vdot*v_dot
        M_a_3 = -Z_wdot*w_dot
        M_a_4 = -K_pdot*p_dot
        M_a_5 = -M_qdot*q_dot
        M_a_6 = -N_rdot*r_dot
        
        #C_a Coriolis and centripital matrix
        #C_a_1 = Z_wdot*w*q
        #C_a_2 = -Z_wdot*w*p - X_udot*u*r
        #C_a_3 = -Y_vdot*v*p + X_udot*u*q
        #C_a_4 = -Z_wdot*w*v + Y_vdot*v*w - N_rdot*r*q + M_qdot*q*r
        #C_a_5 = Z_wdot*w*u - X_udot*u*w + N_rdot*r*p - K_pdot*p*r
        #C_a_6 = - Y_vdot*v*u + X_udot*u*v - M_qdot*q*p + K_pdot*p*q
        C_a_1 = -Z_wdot*w*q + Y_vdot*v*r
        C_a_2 = Z_wdot*w*p - X_udot*u*r
        C_a_3 = -Y_vdot*v*p + X_udot*u*q
        C_a_4 = -Z_wdot*w*v + Y_vdot*v*w - N_rdot*r*q + M_qdot*q*r
        C_a_5 = Z_wdot*w*u - X_udot*u*w + N_rdot*r*p - K_pdot*p*r
        C_a_6 = - Y_vdot*v*u + X_udot*u*v - M_qdot*q*p + K_pdot*p*q
        
        
        
        #D_l Linear damping matrix, skin friction
        D_l_1 = -X_u*u  
        D_l_2 = -Y_v*v 
        D_l_3 = -Z_w*w 
        D_l_4 = -K_p*p
        D_l_5 = -M_q*q
        D_l_6 = -N_r*r
        
        #D_nl non-linear damping matrix
        D_nl_1 = -(X_u_abs*fabs(u-u_c))*(u - u_c)
        D_nl_2 = -(Y_v_abs*fabs(v-v_c))*(v - v_c) 
        D_nl_3 = -(Z_w_abs*fabs(w-w_c))*(w - w_c)
        D_nl_4 = -(K_p_abs*fabs(p))*p
        D_nl_5 = -(M_q_abs*fabs(q))*q
        D_nl_6 = -(N_r_abs*fabs(r))*r
        
        
        ################# EQUATIONS OF MOTION ##################
        added_term = (M_rb+M_a)@np.concatenate((skew(nu_vec[3:])@nu_c_vec[:3],zerovec),axis=0)
        #print(added_term)
        E_Q_M = added_term + (M_rb + M_a)@nu_dot_vec + (C_rb + C_a)@(nu_vec-nu_c_vec) + (D_l)@(nu_vec - nu_c_vec)
        # Have to append some terms, as numpy does not always work perfectly with casadi symbols
        f_1 = E_Q_M[0,:] + D_nl_1 + g_1 - tau_1
        f_2 = E_Q_M[1,:] + D_nl_2 + g_2 - tau_2
        f_3 = E_Q_M[2,:] + D_nl_3 + g_3 - tau_3
        f_4 = E_Q_M[3,:] + D_nl_4 + g_4 - tau_4
        f_5 = E_Q_M[4,:] + D_nl_5 + g_5 - tau_5
        f_6 = E_Q_M[5,:] + D_nl_6 + g_6 - tau_6

        #print(added_term)
        #f_1 = added_term[0] + (M_rb@(nu_dot_vec))[0,0] + M_a_1 + (C_rb@(nu_vec - nu_c_vec))[0,0] + C_a_1 + D_l_1 + D_nl_1 + g_1 - tau_1
        #f_2 = added_term[1] + (M_rb@(nu_dot_vec))[1,0] + M_a_2 + (C_rb@(nu_vec - nu_c_vec))[1,0] + C_a_2 + D_l_2 + D_nl_2 + g_2 - tau_2
        #f_3 = added_term[2] + (M_rb@(nu_dot_vec))[2,0] + M_a_3 + (C_rb@(nu_vec - nu_c_vec))[2,0] + C_a_3 + D_l_3 + D_nl_3 + g_3 - tau_3
        #f_4 = (M_rb@(nu_dot_vec))[3,0] + M_a_4 + (C_rb@(nu_vec))[3,0] + C_a_4 + D_l_4 + D_nl_4 + g_4 - tau_4
        #f_5 = (M_rb@(nu_dot_vec))[4,0] + M_a_5 + (C_rb@(nu_vec))[4,0] + C_a_5 + D_l_5 + D_nl_5 + g_5 - tau_5
        #f_6 = (M_rb@(nu_dot_vec))[5,0] + M_a_6 + (C_rb@(nu_vec))[5,0] + C_a_6 + D_l_6 + D_nl_6 + g_6 - tau_6
        #f_7 = nu_c_vec[0]-nu_enu_c_vec[0]
        #f_8 = nu_c_vec[1]-nu_enu_c_vec[1]
        #f_9 = nu_c_vec[2]-nu_enu_c_vec[2]
        #f_10 = u_c_dot + (skew(nu_vec[3:])@nu_c_vec[:3])[0]
        #f_11 = v_c_dot + (skew(nu_vec[3:])@nu_c_vec[:3])[1]
        #f_12 = w_c_dot + (skew(nu_vec[3:])@nu_c_vec[:3])[2]

        dynamics = vertcat(f_1,f_2,f_3,f_4,f_5,f_6)
        
        self.model.set_alg('dynamics',dynamics)
        
        self.model.setup()

