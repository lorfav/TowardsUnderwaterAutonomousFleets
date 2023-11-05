import do_mpc
from casadi import *

class MyController():

    def __init__(self, rovModel, setPoints = [0,0,0,0,0,0,0,0,0,0,0,0]):
        self.x_setp = setPoints[0]
        self.y_setp = setPoints[1]
        self.z_setp = setPoints[2]
        self.phi_setp = setPoints[3]
        self.theta_setp = setPoints[4]
        self.psi_setp = setPoints[5]

        self.mpc = do_mpc.controller.MPC(rovModel.model)

        setup_mpc = {
            'n_horizon':20,
            't_step':0.1,
            'n_robust':2,
            'store_full_solution':True
        }

        self.mpc.set_param(**setup_mpc)

        
        _x_rov = rovModel.model.x
        _u_rov = rovModel.model.u
        _tvp_rov = rovModel.model.tvp

        mterm = (_x_rov['x']**2 + _x_rov['z']**2 + _x_rov['y']**2 + 
                 _x_rov['phi']**2 + (_x_rov['theta'])**2 + _x_rov['psi']**2)
        
        lterm = ((_tvp_rov['x_sp'] - _x_rov['x'])**2 + (_tvp_rov['y_sp'] - _x_rov['y'])**2 +
                (_tvp_rov['z_sp'] - _x_rov['z'])**2 + (_tvp_rov['phi_sp'] - _x_rov['phi'])**2 +
                (_tvp_rov['theta_sp'] - _x_rov['theta'])**2 + (_tvp_rov['psi_sp'] - _x_rov['psi'])**2
        )

    
        
        
        self.mpc.set_tvp_fun(self.tvp_fun)

        self.mpc.set_rterm(
            u_1 = 0.1,
            u_2 = 0.1,
            u_3 = 0.1,
            u_4 = 0.1,
            u_5 = 0.1,
            u_6 = 0.1,
            u_7 = 0.1,
            u_8 = 0.1,
        )
    
        self.mpc.set_objective(mterm = mterm, lterm = lterm)



        self.mpc.bounds['lower', '_u', 'u_1'] = -6.2
        self.mpc.bounds['lower', '_u', 'u_2'] = -6.2
        self.mpc.bounds['lower', '_u', 'u_3'] = -6.2
        self.mpc.bounds['lower', '_u', 'u_4'] = -6.2
        self.mpc.bounds['lower', '_u', 'u_5'] = -6.2
        self.mpc.bounds['lower', '_u', 'u_6'] = -6.2
        self.mpc.bounds['lower', '_u', 'u_7'] = -6.2
        self.mpc.bounds['lower', '_u', 'u_8'] = -6.2
        
        
        self.mpc.bounds['upper', '_u', 'u_1'] =  6.2
        self.mpc.bounds['upper', '_u', 'u_2'] =  6.2
        self.mpc.bounds['upper', '_u', 'u_3'] =  6.2
        self.mpc.bounds['upper', '_u', 'u_4'] =  6.2
        self.mpc.bounds['upper', '_u', 'u_5'] =  6.2
        self.mpc.bounds['upper', '_u', 'u_6'] =  6.2
        self.mpc.bounds['upper', '_u', 'u_7'] =  6.2
        self.mpc.bounds['upper', '_u', 'u_8'] =  6.2


        self.mpc.setup()

    def tvp_fun(self, t_now):
        tvp_template = self.mpc.get_tvp_template()
        for k in range(21):
            tvp_template['_tvp', k, 'x_sp'] =  self.x_setp
            tvp_template['_tvp', k, 'y_sp'] =  self.y_setp
            tvp_template['_tvp', k, 'z_sp'] =  self.z_setp
            tvp_template['_tvp', k, 'phi_sp'] = self.phi_setp
            tvp_template['_tvp', k, 'theta_sp'] = self.theta_setp
            tvp_template['_tvp', k, 'psi_sp'] = self.psi_setp

        return tvp_template




