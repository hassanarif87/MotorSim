'''
Hassan Arif
23-03-2019
 02-05-2018(Matlab) 
 Vc Control voltage
 u command
 Cvu Coefficient mapping u to Vc
 K_t Torque coefficient 
 http://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
'''

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
pi = np.pi

def MotorModel(omega, t):

    battery_voltage = 16.8  # V
    motor_kv = 2400.0  # V/rev
    J = 2814.588e-9  # kg m^2
    K_t = 60.0 / (2.0 * pi * motor_kv)
    A_t = 1.6211e-08  # Nmrad/s
    A_l = 1.2e-06  #
    Cvu = battery_voltage / 100.0
    R = 0.045  # motor resistance

    u = SignalGenerator(t)
    Vc = Cvu * u
    i = (Vc - omega * K_t) / R
    iSat = 80.0
    i = min(i, iSat)
    trqMot = K_t * i
    omegaDot = trqMot / J - A_t / J * omega * omega - 35.0 * omega
    return omegaDot

def SignalGenerator(t):
    u = 20
    if (t > 1.5):
        u = 50.0
    return u

u0 =0.0 # initial signal
f = 1000.0
dt = 1.0/f
#ODE45 solver
tend = 5.0
tspan = np.linspace(0,tend,(int)(tend*f))
omega0 = 650.0
A_l = 1.2e-06  #
omega = odeint(MotorModel, omega0, tspan)
thrust = omega*omega*A_l


plt.plot(tspan, omega,linewidth= 0.5)
plt.xlabel('Time(s)')
plt.ylabel('Omega (rad/s)')
plt.title('Motor response Rotational Velocity')
plt.show()

plt.figure(2)

plt.plot(tspan, thrust,linewidth= 0.5)
plt.xlabel('Time(s)')
plt.ylabel('Thrust (N)')
plt.title('Motor response Thrust')
plt.show()