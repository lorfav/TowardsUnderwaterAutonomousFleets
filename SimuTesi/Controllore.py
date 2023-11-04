import do_mpc
from casadi import *

class MyController():

    def __init__(rovModel, setPoints = [0,0,0,0,0,0,0,0,0,0,0,0]):
        x_setp = setPoints[0]
        y_setp = setPoints[1]
        z_setp = setPoints[2]
        phi_setp = setPoints[3]
        theta_setp = setPoints[4]
        psi_setp = setPoints[5]

        mpc = do_mpc.controller.MPC(rovModel.model)

        setup_mpc = {
            'n_horizon':20,
            't_step':0.1,
            'n_robust':2,
            'store_full_solution':True
        }

        mpc.set_param(**setup_mpc)

        
        _x_rov = rovModel.model.x
        _u_rov = rovModel.model.u
        _tvp_rov = rovModel.model.tvp

        mterm = (_x_rov['x']**2 + _x_rov['z']**2 + _x_rov['y']**2 + 
                 _x_rov['phi']**2 + (_x_rov['theta'])**2 + _x_rov['psi']**2)
        
        lterm = ((_tvp_rov['x_sp'] - _x_rov['x'])**2 + (_tvp_rov['y_sp'] - _x_rov['y'])**2 +
                (_tvp_rov['z_sp'] - _x_rov['z'])**2 + (_tvp_rov['phi_sp'] - _x_rov['phi'])**2 +
                (_tvp_rov['theta_sp'] - _x_rov['theta'])**2 + (_tvp_rov['psi_sp'] - _x_rov['psi'])**2
        )

        tvp_template = mpc.get_tvp_template()

        def tvp_fun(t_now):
            tvp_template['x_sp'] =  x_setp
            tvp_template['y_sp'] =  y_setp
            tvp_template['z_sp'] =  z_setp
            tvp_template['phi_sp'] = phi_setp
            tvp_template['theta_sp'] = theta_setp
            tvp_template['psi_sp'] = psi_setp
            return tvp_template
        
        mpc.set_tvp_fun(tvp_fun)

        mpc.set_rterm(
            u_1 = 0.1,
            u_2 = 0.1,
            u_3 = 0.1,
            u_4 = 0.1,
            u_5 = 0.1,
            u_6 = 0.1,
            u_7 = 0.1,
            u_8 = 0.1,
        )
    
        mpc.set_objective(mterm = mterm, lterm = lterm)



        mpc.bounds['lower', '_u', 'u_1'] = -6.2
        mpc.bounds['lower', '_u', 'u_2'] = -6.2
        mpc.bounds['lower', '_u', 'u_3'] = -6.2
        mpc.bounds['lower', '_u', 'u_4'] = -6.2
        mpc.bounds['lower', '_u', 'u_5'] = -6.2
        mpc.bounds['lower', '_u', 'u_6'] = -6.2
        mpc.bounds['lower', '_u', 'u_7'] = -6.2
        mpc.bounds['lower', '_u', 'u_8'] = -6.2
        
        
        mpc.bounds['upper', '_u', 'u_1'] =  6.2
        mpc.bounds['upper', '_u', 'u_2'] =  6.2
        mpc.bounds['upper', '_u', 'u_3'] =  6.2
        mpc.bounds['upper', '_u', 'u_4'] =  6.2
        mpc.bounds['upper', '_u', 'u_5'] =  6.2
        mpc.bounds['upper', '_u', 'u_6'] =  6.2
        mpc.bounds['upper', '_u', 'u_7'] =  6.2
        mpc.bounds['upper', '_u', 'u_8'] =  6.2


        mpc.setup()




