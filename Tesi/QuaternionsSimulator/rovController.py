import do_mpc
import numpy as np
class MyController():


    def __init__(self, rovModel1, trackMode,  setPoints = [0,0,0,1,0,0,0,0,0,0,0,0,0]):
        self.x_setp = setPoints[0]
        self.y_setp = setPoints[1]
        self.z_setp = setPoints[2]
        self.q_0_setp = 0
        self.e_1_setp = 0
        self.e_2_setp = 0
        self.e_3_setp = 1

        self.u_c = 0
        self.v_c = 0
        self.w_c = 0

        self.x_2 = 0 
        self.y_2 = 0
        self.z_2 = 0

        self.mpc = do_mpc.controller.MPC(rovModel1.model)
        
        setup_mpc = {
                'n_horizon':20,
                't_step':0.1,
                'n_robust':1,
                'store_full_solution':True,
                'nlpsol_opts': {'ipopt.max_iter': 25},
        
                }
        
        self.mpc.set_param(**setup_mpc)
        _x_rov1 = rovModel1.model.x
        _u_rov1  = rovModel1.model.u
        _tvp_rov1 = rovModel1.model.tvp

        match trackMode:
            case 0:
                mterm =   _x_rov1['z']**2 + _x_rov1['y']**2 +  _x_rov1['phi']**2 + (_x_rov1['theta'])**2 + _x_rov1['psi']**2 + _x_rov1['x']**2
                lterm =   ((_x_rov1['z']-_tvp_rov1['z_sp'])**2 + (_x_rov1['y']-_tvp_rov1['y_sp'])**2 +  (_x_rov1['phi'] - _tvp_rov1['phi_sp'])**2 + 
                            (_x_rov1['theta']- _tvp_rov1['theta_sp'] )**2 + (_x_rov1['psi'] - _tvp_rov1['psi_sp'])**2 + (_x_rov1['x']- _tvp_rov1['x_sp'])**2 + 
                            (_u_rov1['u_1']**2+_u_rov1['u_2']**2+_u_rov1['u_3']**2+_u_rov1['u_4']**2+_u_rov1['u_5']**2 + _u_rov1['u_6']**2+_u_rov1['u_7']**2+_u_rov1['u_8']**2)*0.01)

            case 1:
                mterm = (_x_rov1['x'] + 2 - _tvp_rov1['x_sp'])**2 + (_x_rov1['y'] + 0 - _tvp_rov1['y_sp'])**2 + (_x_rov1['z'] + 2 - _tvp_rov1['z_sp'])**2 + (_x_rov1['phi'] - _tvp_rov1['phi_sp'])**2 + (_x_rov1['theta'] - _tvp_rov1['theta_sp'])**2 +(_x_rov1['psi']  - _tvp_rov1['psi_sp'])**2  
                lterm = (_x_rov1['x'] + 2 - _tvp_rov1['x_sp'])**2 + (_x_rov1['y'] + 0 - _tvp_rov1['y_sp'])**2 + (_x_rov1['z'] + 2 - _tvp_rov1['z_sp'])**2 +(_x_rov1['phi'] - _tvp_rov1['phi_sp'])**2 + (_x_rov1['theta']  - _tvp_rov1['theta_sp'])**2 +(_x_rov1['psi']  - _tvp_rov1['psi_sp'])**2
            case 2:
                mterm = (3*(((1*(_tvp_rov1['x_sp']-_x_rov1['x'])**2+ 1*(_tvp_rov1['y_sp']-_x_rov1['y'])**2)-2**2)**2 +
                20*(_x_rov1['z']-_tvp_rov1['z_sp'])**2)
                + 400*((((_x_rov1['q_0']*_tvp_rov1['q_0_sp']+_x_rov1['e_1'] * _tvp_rov1['e_1_sp']+_x_rov1['e_2']* _tvp_rov1['e_2_sp']+_x_rov1['e_3']* _tvp_rov1['e_3_sp'])**2-1)**2 )
                +(-_tvp_rov1['e_1_sp']*_x_rov1['q_0']+_tvp_rov1['q_0_sp']*_x_rov1['e_1']-_tvp_rov1['e_3_sp']*_x_rov1['e_2']+_tvp_rov1['e_2_sp']*_x_rov1['e_3'])**2
                +(-_tvp_rov1['e_2_sp']*_x_rov1['q_0']+_tvp_rov1['e_3_sp']*_x_rov1['e_1']+_tvp_rov1['q_0_sp']*_x_rov1['e_2']-_tvp_rov1['e_1_sp']*_x_rov1['e_3'])**2
                +(-_tvp_rov1['e_3_sp']*_x_rov1['q_0']-_tvp_rov1['e_2_sp']*_x_rov1['e_1']+_tvp_rov1['e_1_sp']*_x_rov1['e_2']+_tvp_rov1['q_0_sp']*_x_rov1['e_3'])**2
                ))
                lterm = mterm + (_u_rov1['u_1']**2+_u_rov1['u_2']**2+_u_rov1['u_3']**2 +
                                 _u_rov1['u_4']**2+_u_rov1['u_5']**2 + _u_rov1['u_6']**2+
                                 _u_rov1['u_7']**2+_u_rov1['u_8']**2)*1

        #_x['phi']**2 + _x['theta']**2 + _x['psi']**2 +
        #_x['phi']**2 + _x['theta']**2 + _x['psi']**2 +
        tvp_template = self.mpc.get_tvp_template()
        
        self.mpc.set_tvp_fun(self.tvp_fun)
        self.mpc.set_rterm(
                u_1 = 1,
                u_2 = 1,
                u_3 = 1,
                u_4 = 1,
                u_5 = 1,
                u_6 = 1,
                u_7 = 1,
                u_8 = 1
                )
        
        #self.mpc.set_nl_cons("FOV",
        #        2*(np.cos((45/180)*3.14)*2-((1-(2*_x_rov1['e_2']**2+2*_x_rov1['e_3']**2))*(_tvp_rov1['x_2']-_x_rov1['x'])
        #        +(2*_x_rov1['e_1']*_x_rov1['e_2']+2*_x_rov1['e_3']*_x_rov1['q_0'])*(_tvp_rov1['y_2']-_x_rov1['y'])
        #        +(2*_x_rov1['e_1']*_x_rov1['e_3']-2*_x_rov1['e_2']*_x_rov1['q_0'])*(_tvp_rov1['z_2']-_x_rov1['z'])))
        #        , ub=0, soft_constraint=True, penalty_term_cons=10
        #        )
        
        self.mpc.set_objective(mterm=mterm,lterm=lterm)
        
        self.mpc.bounds['lower', '_u', 'u_1'] = - 10
        self.mpc.bounds['lower', '_u', 'u_2'] = - 10
        self.mpc.bounds['lower', '_u', 'u_3'] = - 10
        self.mpc.bounds['lower', '_u', 'u_4'] = - 10
        self.mpc.bounds['lower', '_u', 'u_5'] = - 10
        self.mpc.bounds['lower', '_u', 'u_6'] = - 10
        self.mpc.bounds['lower', '_u', 'u_7'] = - 10
        self.mpc.bounds['lower', '_u', 'u_8'] = - 10
        
        
        self.mpc.bounds['upper', '_u', 'u_1'] =  10
        self.mpc.bounds['upper', '_u', 'u_2'] =  10
        self.mpc.bounds['upper', '_u', 'u_3'] =  10
        self.mpc.bounds['upper', '_u', 'u_4'] =  10
        self.mpc.bounds['upper', '_u', 'u_5'] =  10
        self.mpc.bounds['upper', '_u', 'u_6'] =  10
        self.mpc.bounds['upper', '_u', 'u_7'] =  10
        self.mpc.bounds['upper', '_u', 'u_8'] =  10
        
        
        self.mpc.setup()

    def tvp_fun(self, t_now):
        tvp_template = self.mpc.get_tvp_template()
        for k in range(21):
            tvp_template['_tvp',k,'x_sp'] =  self.x_setp
            tvp_template['_tvp',k,'y_sp'] =  self.y_setp
            tvp_template['_tvp',k,'z_sp'] =  self.z_setp
            tvp_template['_tvp',k,'q_0_sp'] =  self.q_0_setp
            tvp_template['_tvp',k,'e_1_sp'] =  self.e_1_setp
            tvp_template['_tvp',k,'e_2_sp'] =  self.e_2_setp
            tvp_template['_tvp',k,'e_3_sp'] =  self.e_3_setp
            tvp_template['_tvp',k,'x_2'] =  self.x_2
            tvp_template['_tvp',k,'y_2'] =  self.y_2
            tvp_template['_tvp',k,'z_2'] =  self.z_2
            tvp_template['_tvp',k,'u_c'] =  self.u_c
            tvp_template['_tvp',k,'v_c'] =  self.v_c
            tvp_template['_tvp',k,'w_c'] =  self.w_c
    
        return tvp_template

