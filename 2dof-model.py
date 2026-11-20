import grama as gr
import pandas as pd
import numpy as np
from scipy.integrate import solve_ivp

def state_transition (init_cond: tuple[float,float,float,float,float,float], input_vec: pd.Series, t_span):


    for step in t_span:

        # numerically integrate to solve next state
        state = solve_ivp(
        slocum_eoms, 
        [t_span[0], t_span[-1]], 
        init_cond, 
        method='RK45', 
        dense_output=True,
    )
     
    return gr.df_make(
        
    )

def slocum_eoms(t_span, states):

    # grab previous states
    x,z,theta,x_dot,z_dot,theta_dot = states[-1]

    # grab constant control inputs
    m_b = input_vec[0]
    r_p = input_vec[1]

    # aerodynamic constants
    rho = 
    A =
    g =

    # vehicle constants
    m = 
    m_hull =
    m_bar =  
    r_b = 0
    m_f1 =
    m_f3 =

    # calculate mass of vehicle and net buoyancy
    m_v = m_hull + m_b + m_bar
    m0 = m_v - m

    # added mass moment terms from j.g. graver (2003)
    m_f1 = 1
    m_f3 = 2
    
    # velocity frame transformations
    v1 = -z_dot*np.sin(theta) + x_dot*np.cos(theta)
    v3 = x_dot*np.sin(theta) - z_dot*np.cos(theta)

    V = np.sqrt(v1**2 + v3**2)

    # calculate angle of attack and glide angle (epsilon)
    aoa = np.arctan2(v3,v1)

    epsilon = theta - aoa

    # fit aerodynamic coefficients from j.g. graver 2003
    c_d = 0.214 * aoa * 32.3 * aoa**2
    c_l = 11.76 * aoa + 4.6 * aoa * abs(aoa)
    c_m = 0.63 * aoa

    # calculate forces
    L = 0.5 * rho * c_l * A * V**2
    D = 0.5 * rho * c_d * A * V**2
    M_dl = 0.5 * rho * c_m * A * V**2

    # calculate accelerations
    e1_ddot = L * np.sin(aoa) - D*np.cos(theta) - m0*g*np.sin(theta)
    e3_ddot = L * np.cos(aoa) + D*np.sin(theta) -m0*g*np.cos(theta)
    theta_ddot = (m_f1 - m_f3)*v1*v3 -m_bar*g*(r_p*np.cos(theta)) - m_b*g*(r_b*np.cos(theta)) + M_dl

    x_ddot = e1_ddot * np.cos(theta) + e3_ddot * np.sin(theta)
    z_ddot = -e1_ddot * np.sin(theta) + e3_ddot * np.cos(theta)

    return [x_dot,z_dot,theta_dot,x_ddot,z_ddot,theta_ddot]
    

md_det_abs = (
        gr.Model()
        >> gr.cp_vec_function(
            fun=model_trajectory,
            var=["m_b", "r_p"],
            out=["glide_angle"],
        )
        >> gr.cp_bounds(
            #wide ranges selected to allow for system exploration
            m_b =(1e2, 1e4),
            r_p = (1e1, 1e2),
        )
    )