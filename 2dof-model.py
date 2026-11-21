import grama as gr
import pandas as pd
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.use('TkAgg') # Or 'QtAgg' if you installed PyQt
import matplotlib.pyplot as plt

def state_transition(init_cond: tuple[float,float,float,float,float,float], input_vec: pd.Series, t_span):


     # grab constant control inputs
    m_b = input_vec[0]
    r_p = input_vec[1]


    control_inputs = (m_b, r_p)

    
    state = solve_ivp(
        slocum_eoms, 
        [t_span[0], t_span[-1]], 
        init_cond, 
        method='RK45', 
        args=control_inputs,
        dense_output=True,
        )
     
    return state

def slocum_eoms(t_span, states, m_b, r_p):

    # grab previous states
    x,z,theta,x_dot,z_dot,theta_dot = states

    # aerodynamic constants
    rho = 1000 #kg/m**3
    A = 0.0355 #m**2
    g = 9.81 #m/s**2

    # vehicle constants
    m_hull = 50
    m_bar = 9.1
    m = m_hull + m_bar
    r_b = 0

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
    
v_b = 2.5e-4
m_b = v_b * 1000
r_p = 0.25

control_vec = pd.Series([m_b, r_p])

x = 0
z = 0
theta = -np.pi/2
x_dot = 1
z_dot = 1
theta_dot = 0

ics = (x,z,theta,x_dot,z_dot,theta_dot)

t_span = [0,1000]

glide_path = state_transition(ics,control_vec, t_span)

# print(glide_path.t)
print(glide_path.y)
# print(np.shape(glide_path.y))

t_res = glide_path.t
state_res = glide_path.y

plt.scatter(state_res[0,:],state_res[1,:],t_res)
plt.xlabel("X Position")
plt.ylabel("Z Position")
plt.title("Steady Control Input Glide Path")
plt.show()