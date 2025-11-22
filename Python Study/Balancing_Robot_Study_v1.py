# -*- coding: utf-8 -*-
"""
This program defines a self balancing robot and then study
its behaviour through State space modeling and transfer fuction modeling.

The robot is stabilized with both PID and LQR regulations designed using
poles placement methods.

@author: Mathieu Freuville
"""
from colorama import  Fore, Style
import control as ct
import math
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from numpy.linalg import eig
from scipy.linalg import solve


###############################################################################
# PID Parameters
###############################################################################


# PID paramters for torque model with PID in the feedback loop
Kp_TorqueModel= -6.5
Ki_TorqueModel = -2.5
Kd_TorqueModel = -2


# PID paramters for motor voltage model with PID in the feedback loop
Kp_VoltModel = -130
Ki_VoltModel = -30
Kd_VoltModel = -6
###############################################################################
# Misc
###############################################################################

s = ct.TransferFunction.s #Laplacian number
Fig_Count = 0  # Total number of figures. Simplify the futures numbering


###############################################################################
# System Constants
###############################################################################

g = 9.8067             # m/s^2 - Acceleration due to gravity.

# --- Parameters of Geared Motor  ---
Tstall = 0.5                           # N-m - stall torque. (GA37-520 DC motor datasheet)
RPMnoload = 360                         # No-load RPM at Vin=12V (GA37-520 DC motor datasheet)
wnl = (RPMnoload * 2 * math.pi)/60      # No load gearbox output shaft rotation velocity in rad/sec
Inl = 0.02                              # No load current into motor, SHALL BE MEASURED (GA37-520 DC motor datasheet)
Vin = 12                                # Volt. Input voltage to armature during test
Istall = 1.2                            # Amp, stall armature current (from JGB37-520 datasheet)
Ng = 30                                 # Gear ratio
Ra = Vin/Istall                         # Armature resistance
Kt = (Tstall/(Vin*Ng))*Ra               # Armature torque constant
Kb = (Vin-Ra*Inl)/(wnl*Ng)              # Armature back EMF constant
                                      
# --- Parameters of Wheel Assembly ---
R = 0.0325                              # m - Radius of wheel, 35 mm.
mw = 0.086                              # kg - Mass of wheel 
Jw = 0.5*mw*R*R                         # kgm2 - Moment inertia of wheel assy

# --- Parameters of Main Body ---
mb = 0.663                              # kg - Mass of the body and motor stators
lb = 0.0388                             # m - Distance of center of mass to wheel axle
Jb = 0.85*mb*lb*lb                      # kgm2 - Moment inertia of Main Body reference to the center of mass


# Constants for DC motor, gear box and wheel
Cm1 = Ng*Kt/Ra
Cm2 = Cm1*Kb
M = mb + 2*(mw+Jw/(R*R)) - ((mb*mb*lb*lb)/(mb*lb*lb + Jb))
J = (mb*lb*lb) + Jb - ((mb*mb*lb*lb)/(mb + 2*(mw+Jw/(R*R))))
C1 = (1/M)*((mb*mb*lb*lb*g)/(Jb + mb*lb*lb))
C2 = (2/M)*((1/R) + ((mb*lb)/(Jb + mb*lb*lb)))
C3 = (mb*lb*g)/J
C4 = (2/(J*R))*(R + ((mb*lb)/(mb + 2*(mw + Jw/(R*R)))))

###############################################################################
# Internal Functions
###############################################################################

def PrintTitle(Title_Text):
    '''
    This function prints a magenta title over de Python console.
    This facilitates future readings
    '''
    
    print(Fore.MAGENTA + "\n" + "="*80)
    print(Title_Text)
    print("="*80 + Style.RESET_ALL)
    return


def clean_transfer_function(tf, tol=1e-6):
    '''
    This function cleans a transfer function from its little coeficients. 
    These coeficiets are artefacts created by errors in the rounding during 
    transfer function calculations.
    '''
    # Get the numerator and denominator
    num = tf.num[0][0]  # SISO model
    den = tf.den[0][0]
    
    # Save the names of variables and TF
    input_name = tf.input_labels if hasattr(tf, 'input_labels') else None
    output_name = tf.output_labels if hasattr(tf, 'output_labels') else None
    tf_name = tf.name if hasattr(tf, 'name') else None
    
    # First cleaning
    num_clean = num.copy()
    den_clean = den.copy()
    num_clean[np.abs(num_clean) < tol] = 0
    den_clean[np.abs(den_clean) < tol] = 0
    
    # Create a TF then simplify it with minreal
    Cleaned_TF = ct.TransferFunction(num_clean, den_clean)
    Cleaned_TF = ct.minreal(Cleaned_TF, tol)
    
    # Second cleaning after the minreal (removes rounding errors)
    num_final = Cleaned_TF.num[0][0].copy()
    den_final = Cleaned_TF.den[0][0].copy()
    num_final[np.abs(num_final) < tol] = 0
    den_final[np.abs(den_final) < tol] = 0
    
    # Créer la TF finale
    Final_TF = ct.TransferFunction(num_final, den_final, 
                                     inputs=input_name, 
                                     outputs=output_name,
                                     name=tf_name)
    return Final_TF

def Generate_PZ_Map(TF, GraphTitle):
    
    global Fig_Count
    Fig_Count = Fig_Count + 1
    
    plt.figure(Fig_Count)
    ct.pzmap(TF)
    plt.suptitle('') #supress the automatic title from pzmap
    TitleText= GraphTitle
    plt.title(TitleText)
    plt.grid()
    p = ct.poles(TF)
    z = ct.zeros(TF)
    print(GraphTitle)
    print("-"*80)
    print("poles = ", p)
    print("zeros = ", z)
    print("\n")
    return

def Generate_Color_PZ_Map(TF, GraphTitle):
    
    global Fig_Count
    Fig_Count = Fig_Count + 1
    
    p = ct.poles(TF)
    z = ct.zeros(TF)
    
    fig, ax = plt.subplots(num=Fig_Count)
    TitleText = GraphTitle
    ax.set_title(TitleText)
    
    # Trace poles values
    ax.plot(np.real(p), np.imag(p), 'rx', markersize=8, markeredgewidth=1.5, label='Pôles')
    
    # Trace zeros values
    if len(z) > 0:
        ax.plot(np.real(z), np.imag(z), 'bo', markersize=8, 
                markerfacecolor='none', markeredgewidth=1.5, label='Zéros')
    
    ax.grid()
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax.axvline(x=0, color='k', linestyle='-', linewidth=0.5)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('Real')
    ax.set_ylabel('Imaginary')
    ax.legend()
    
    # Récupérer les ticks actuels
    xticks = list(ax.get_xticks())
    yticks = list(ax.get_yticks())
    
    # Print data over the console
    PrintTitle(GraphTitle)
    print("poles = ", p)
    print("zeros = ", z)
    print("\n")
    return

def Generate_Bode_Plot(TF, GraphTitle, w0, w1, dw):
    
    global Fig_Count
    Fig_Count = Fig_Count + 1
    
    nw = int((w1 - w0) / dw) + 1  # Number of points of freq
    w = np.linspace(w0, w1, nw)
    
    plt.figure(Fig_Count, figsize=(12, 9))
    ct.bode_plot(TF, w, dB=True, deg=True, display_margins=True)
    # Access sub graphs (magnitude et phase)
    axes = plt.gcf().get_axes()

    # Remove automatic title
    axes[0].set_title("")
    
    # 1st subplot : Magnitude
    axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
    
    # 2nd subplot : Phase
    axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
    
    # General Title
    plt.suptitle(GraphTitle, fontsize=16, fontweight='bold')
    
    # Calculating stability margins and crossover frequencies:
    (GM, PM, wg, wp) = ct.margin(TF)
    print(str(TF.output_labels) + " Vs " + str(TF.input_labels) + "\n")
    print ( '  GM [1 ( not dB )] = ' ,GM )
    print ( ' PM [ deg ] = ' ,PM )
    print ( ' wg [ rad / s ] = ' ,wg)
    print ( ' wp [ rad / s ] = ' , wp)
    print("\n")
    
    plt.tight_layout()
    plt.show()
    
def Sweep_PID_Values(Process_TF,Kp_range, Kd_range, Ki_range,  PID_in_Feedback = True,  PrintGraphs = True):
    
    '''
    This function performs a sweep in all possible Kp, Ki, Kd combinations
    and keeps only combinations with relevant impulse response
    
    Results are then saved in an Excel files
    '''
    global Fig_Count 

    Max_Allowable_Overshoot = 0.05
    Max_Allowable_SettlingTime = 5 
    SettlingTime_Tolerance = 1  
    Stable_Systems = []
    print("Start of the sweeps \n")
    Total_states = len(Kp_range) * len(Kd_range) * (len(Ki_range))
    Calculated_states = 0                                               

    for i in range(len(Kp_range)):
        for j in range(len(Kd_range)):
            for k in range(len(Ki_range)):
                
                Calculated_states = Calculated_states + 1
                PID_num = [Kd_range[j], Kp_range[i], Ki_range[k]]  
                PID_den = [1, 0] 
                PID_TF = ct.tf(PID_num, PID_den)
                
                if (PID_in_Feedback == True):
                    CL_System = ct.feedback(Process_TF, PID_TF)
                else:
                    # PID in main loop with a negative feedback
                    Forward_Path = PID_TF * Process_TF
                    CL_System = ct.feedback(Forward_Path, 1)  
                
                t, y = ct.impulse_response(CL_System, T=5)
                p = ct.poles(CL_System)
                
                if np.any(np.real(p) > 0):
                    User_message = "Calculated state = " + str(Calculated_states) + " / " + str(Total_states) + " Unstable \n"
                    #print(User_message)
                    
                else:
                    User_message = "Calculated state = " + str(Calculated_states) + " / " + str(Total_states) + " Stable Kp=" + str(Kp_range[i]) + " Kd=" + str(Kd_range[j]) + " Ki="  + str(Ki_range[k]) +"\n"
                    print(User_message)
                    
                    # maximal overshoot calculation
                    y_final = y[-1]  # final value (steady-state)
                    y_max = np.max(y)  # max value
                    overshoot_value = y_max   # absolute value of overshoot
                    
                    if (overshoot_value < Max_Allowable_Overshoot):
                        
                        # when did overshoot happened?
                        t_max = t[np.argmax(y)]
                        
                        # settling time 
                        # upper_bound = y_final * (1 + SettlingTime_Tolerance)
                        # lower_bound = y_final * (1 - SettlingTime_Tolerance)
                        upper_bound = SettlingTime_Tolerance
                        lower_bound = -SettlingTime_Tolerance 
                        
                        # Final moment when response falls outside the tolerance zone
                        settling_indices = np.where((y > upper_bound) | (y < lower_bound))[0]
                        if len(settling_indices) > 0:
                            settling_time = t[settling_indices[-1]]
                        else:
                            settling_time = 0  # Already inside band since the beginning
                        
                        if (settling_time < Max_Allowable_SettlingTime):
                            # Save stable system data
                            Stable_Systems.append({
                                'Kp': Kp_range[i],
                                'Kd': Kd_range[j],
                                'Ki': Ki_range[k],
                                'Overshoot': overshoot_value,
                                'Settling Time (s)': settling_time})
                                
                            
                            if (PrintGraphs == True):
                                TitleText= "Response of Pendulum Position to an Impulse Disturbance under PID Control \n Kp=" + str(Kp_range[i]) + " Ki="+ str(Ki_range[k]) +" Kd =" + str(Kd_range[j]) 
                                Fig_Count = Fig_Count + 1
                                plt.figure(Fig_Count)
                                plt.plot(t,y)
                                plt.title(TitleText)
                                #plt.axis(xmin=0, xmax=2.5)
                                
                                # Trace lines for visualizing overshoot
                                plt.axhline(y=y_final, color='r', linestyle='--', linewidth=1, label=f'Valeur finale = {y_final:.4f}')
                                plt.axhline(y=y_max, color='g', linestyle='--', linewidth=1, label=f'Max = {y_max:.4f}')
                                plt.plot(t_max, y_max, 'ro', markersize=2, label=f'Overshoot = {overshoot_value:.4f}')
                                if settling_time > 0:
                                    plt.axvline(x=settling_time, color='purple', linestyle='--', linewidth=1, label=f'Settling time = {settling_time:.2f}s')
                                plt.grid()
                                plt.legend()
                                plt.xlabel("Time (s)")
                                plt.ylabel("Pendulum Angle (rad)")

    # Afficher le tableau
    print("\n" + "="*80)
    print("STABLE SYSTEMS")
    print("="*80)
    print(f"{'Kp':>6} {'Kd':>8} {'Ki':>8} {'Overshoot':>15} {'Settling Time (s)':>18}")
    print("-"*80)
    for row in Stable_Systems:
        print(f"{row['Kp']:>6} {row['Kd']:>8.2f} {row['Ki']:>8.2f} {row['Overshoot']:>15.4f} {row['Settling Time (s)']:>18.4f}")
    
    # Create a DataFrame with all stable systems
    df_stable = pd.DataFrame(Stable_Systems)
    # Save in Excel
    df_stable.to_excel('stable_systems.xlsx', index=False, sheet_name='Stable')
    print("\n Table has been saved in 'stable_systems.xlsx'")
    
    return

def PrintPolynomial(Coefs):
    # Print a readable polynominal
    n = len(Coefs) - 1
    poly_str = ""
    for i, c in enumerate(Coefs):
        power = n - i
        if i > 0 and c >= 0:
            poly_str += " + "
        elif c < 0:
            poly_str += " - "
            c = abs(c)
        
        if power > 1:
            poly_str += f"{c:.4f}·s^{power}"
        elif power == 1:
            poly_str += f"{c:.4f}·s"
        else:
            poly_str += f"{c:.4f}"
    print(f"   {poly_str}")
    
###############################################################################
# SSM Definition
###############################################################################

'''
This section creates 2 SSM based on constants entered above.
'''

Mstates = ['x', 'x_dot', 'Theta', 'Theta_dot']
Minputs = ['Torque']
Moutputs = ['x', 'Theta']

A_Unmotorized = [[0, 1, 0, 0], [0, 0, -C1, 0], [0, 0, 0, 1], [0, 0, C3, 0] ]
B_Unmotorized = [[0], [C2], [0], [-C4]]
C_Unmotorized = [[1, 0, 0, 0], [0, 0, 1, 0]]
D_Unmotorized = 0

SSmodel_Unmotorized = ct.ss(A_Unmotorized,B_Unmotorized,C_Unmotorized,D_Unmotorized, inputs = Minputs, outputs = Moutputs, states = Mstates)

Mstates = ['x', 'x_dot', 'Theta', 'Theta_dot']
Minputs = ['Voltage']
Moutputs = ['x', 'Theta']

A_Motorized = [[0, 1, 0, 0], [0, -C2*Cm2/(R*Ng), -C1, C2*Cm2], [0, 0, 0, 1], [0, C4*Cm2/(R*Ng), C3, -C4*Cm2] ]
B_Motorized = [[0], [C2*Cm1], [0], [-C4*Cm1]]
C_Motorized = [[1, 0, 0, 0], [0, 0, 1, 0]]
D_Motorized = 0

SSmodel_Motorized = ct.ss(A_Motorized,B_Motorized,C_Motorized,D_Motorized, inputs = Minputs, outputs = Moutputs, states = Mstates)


PrintTitle('SS Model for torque based solution')
print(SSmodel_Unmotorized)

PrintTitle('SS Model for motor voltage based solution')
print(SSmodel_Motorized)


###############################################################################
# System Transfer Functions
###############################################################################

# Create 2 TF/system (Theta and x)
TF_Unmotorized_Full = ct.ss2tf(SSmodel_Unmotorized)
TF_Motorized_Full = ct.ss2tf(SSmodel_Motorized)

# Generate simple TF for unmotorized solution (Actuator = Torque)
TF_Unmotorized_X = TF_Unmotorized_Full[0,0]
TF_Unmotorized_Theta = TF_Unmotorized_Full[1,0]

# Generate simple TF for unmotorized solution (Actuator = Motor voltage)
TF_Motorized_X = TF_Motorized_Full[0,0]
TF_Motorized_Theta = TF_Motorized_Full[1,0]

# Print all TF for verification
PrintTitle('Transfer Functions  for unmotorized solution')
print(TF_Unmotorized_Full)
print(TF_Unmotorized_X)
print(TF_Unmotorized_Theta)

PrintTitle('Transfer Functions  for motorized solution')
print(TF_Motorized_Full)
print(TF_Motorized_X)
print(TF_Motorized_Theta)

###############################################################################
# Cleaning of the Transfer Functions
###############################################################################
PrintTitle('Cleaning of the Transfer functions')
TF_Unmotorized_X = clean_transfer_function(TF_Unmotorized_X, tol=1e-8)
TF_Unmotorized_Theta = clean_transfer_function(TF_Unmotorized_Theta, tol=1e-8)
TF_Motorized_X = clean_transfer_function(TF_Motorized_X, tol=1e-8)
TF_Motorized_Theta = clean_transfer_function(TF_Motorized_Theta, tol=1e-8)

print(TF_Unmotorized_X)
print(TF_Unmotorized_Theta)
print(TF_Motorized_X)
print(TF_Motorized_Theta)

###############################################################################
# Open Loop Impulse response
###############################################################################

if (True):
    PrintTitle('Impulse Response - Unmotorized model')
    t, y = ct.impulse_response(TF_Unmotorized_X, T=0.2)
    Fig_Count = Fig_Count + 1
    plt.figure(Fig_Count)
    TitleText= "Response of robot Position to an Impulse Disturbance \n Model based on torque" 
    plt.title(TitleText)
    plt.plot(t,y)
    plt.xlabel("Time (s)")
    plt.ylabel("Robot Position (m)")
    plt.grid()
    
    t, y = ct.impulse_response(TF_Unmotorized_Theta, T=0.2)
    Fig_Count = Fig_Count + 1
    plt.figure(Fig_Count)
    TitleText= "Response of robot tilt angle to an Impulse Disturbance \n Model based on torque" 
    plt.title(TitleText)
    plt.xlabel("Time (s)")
    plt.ylabel("Robot Angle (rad)")
    plt.plot(t,-y)
    plt.grid()
    #plt.axis(xmin=0, xmax=2)
    
    PrintTitle('Impulse Response - Motorized model')
    t, y = ct.impulse_response(TF_Unmotorized_X, T=0.2)
    Fig_Count = Fig_Count + 1
    plt.figure(Fig_Count) 
    TitleText= "Response of robot position to an Impulse Disturbance \n Model based on motor voltage" 
    plt.title(TitleText)
    plt.plot(t,y)
    plt.xlabel("Time (s)")
    plt.ylabel("Robot Position (m)")
    plt.grid()
    
    t, y = ct.impulse_response(TF_Unmotorized_Theta, T=0.2)
    Fig_Count = Fig_Count + 1
    plt.figure(Fig_Count)
    TitleText= "Response of robot tilt angle to an Impulse Disturbance \n Model based on motor voltage" 
    plt.title(TitleText)
    plt.xlabel("Time (s)")
    plt.ylabel("Robot Angle (rad)")
    plt.plot(t,-y)
    plt.grid()
    #plt.axis(xmin=0, xmax=2)

###############################################################################
# PZ Map - Open loop
###############################################################################

if (True):
    PrintTitle('Open loop Poles & Zeros map for both models')
    
    Generate_Color_PZ_Map(TF_Unmotorized_X, "PZ Map for robot x position \nModel based on torque ")
    Generate_Color_PZ_Map(TF_Unmotorized_Theta,"PZ Map for robot tilt angle\n Model based on torque " )
    Generate_Color_PZ_Map(TF_Motorized_X,"poles/Zeros for robot x Position\nModel based on motor voltage:")
    Generate_Color_PZ_Map(TF_Motorized_Theta, "PZ Map for robot tilt angle \nModel based on motor voltage " )

###############################################################################
# Find the open-loop poles of the system (eigenvalues  of A matrix)
###############################################################################

if (True):
    PrintTitle('Open loop Poles found by eigen values')
    #eigenvalues
    w,v=eig(A_Unmotorized)
    print('Eigenvalues : \n', w, "\n")
    print('Eigenvector : \n ', v, "\n")
    
    w,v=eig(A_Motorized)
    print('Eigenvalues : \n', w, "\n")
    print('Eigenvector : \n ', v, "\n")

###############################################################################
# Open loop - Frequency response
###############################################################################

if (False):
    PrintTitle('Open loop Frequency analysis - Bode plots ')
    
    Generate_Bode_Plot(TF_Unmotorized_X, "GraphTitle", 1, 1000, 0.1)
    
    Fig_Count = Fig_Count + 1
    plt.figure(Fig_Count)
    freqresp = ct.frequency_response(TF_Unmotorized_X)
    cplt = freqresp.plot()
    
    Generate_Bode_Plot(TF_Unmotorized_Theta, "GraphTitle", 1, 1000, 0.1)
    
    Fig_Count = Fig_Count + 1
    plt.figure(Fig_Count)
    freqresp = ct.frequency_response(TF_Unmotorized_Theta)
    cplt = freqresp.plot()
    
    Generate_Bode_Plot(TF_Motorized_X, "GraphTitle", 1, 1000, 0.1)
    
    Fig_Count = Fig_Count + 1
    plt.figure(Fig_Count)
    freqresp = ct.frequency_response(TF_Motorized_X)
    cplt = freqresp.plot()
    
    Generate_Bode_Plot(TF_Motorized_Theta, "GraphTitle", 1, 1000, 0.1)
    
    Fig_Count = Fig_Count + 1
    plt.figure(Fig_Count)
    freqresp = ct.frequency_response(TF_Motorized_Theta)
    cplt = freqresp.plot()

###############################################################################
# Parallel PID controller creation
###############################################################################

PrintTitle("Creation of the PID controlers")
PID_num = [Kd_TorqueModel, Kp_TorqueModel, Ki_TorqueModel]  
PID_den = [1, 0] 
PID_TF_TorqueModel = ct.tf(PID_num, PID_den)
print("PID transfer function : \n", PID_TF_TorqueModel, "\n")

PID_num = [Kd_VoltModel, Kp_VoltModel, Ki_VoltModel]  
PID_den = [1, 0] 
PID_TF_VoltModel = ct.tf(PID_num, PID_den)
print("PID transfer function : \n", PID_TF_VoltModel, "\n")

###############################################################################
# CLosed Loop system - PID in the feedback loop
###############################################################################
PrintTitle("Creation of the controlled system based on theta angle")
CL_Unmotorized_System = ct.feedback(TF_Unmotorized_Theta,PID_TF_TorqueModel,sign = -1)
CL_Motorized_System = ct.feedback(TF_Motorized_Theta,PID_TF_VoltModel,sign = -1)

CL_Unmotorized_System = clean_transfer_function(CL_Unmotorized_System, tol=1e-8)
CL_Motorized_System = clean_transfer_function(CL_Motorized_System, tol=1e-8)

print("Unmotorized model (torque based): \n",CL_Unmotorized_System)
print("Motorized model (Voltage based): \n",CL_Motorized_System)

###############################################################################
# Impulse response of the controlled systems - PID in the feedback loop
###############################################################################
if (False):
    PrintTitle("Impulse response of the controlled system with PID in feedback loop \n Model based on torque")
    t = np.linspace(0, 10, 1000)
    t, y = ct.impulse_response(CL_Unmotorized_System, T=t)
    Fig_Count = Fig_Count + 1
    plt.figure(Fig_Count)
    TitleText= "Impulse response of the controlled system \n Model based on torque - PID in feedback loop" 
    plt.title(TitleText)
    plt.xlabel("Time (s)")
    plt.ylabel("Robot Angle (rad)")
    plt.plot(t,y)
    plt.grid()
    #plt.axis(xmin=0, xmax=2)
    
    PrintTitle("Impulse response of the controlled system with PID in feedback loop \n Model based on motor voltage")
    t = np.linspace(0, 10, 1000)
    t, y = ct.impulse_response(CL_Motorized_System, T=t)
    Fig_Count = Fig_Count + 1
    plt.figure(Fig_Count)
    TitleText= "Impulse response of the controlled system \n Model based on motor voltage - PID in feedback loop" 
    plt.title(TitleText)
    plt.xlabel("Time (s)")
    plt.ylabel("Robot Angle (rad)")
    plt.plot(t,-y)
    plt.grid()
    #plt.axis(xmin=0, xmax=2)
###############################################################################
# Stability - PID Controlled - PZMap
###############################################################################
if (True):
    TitleText= "PZ Map of Pendulum Angle under PID Control \n For unmotorized system : \n Kp=" + str(-Kp_TorqueModel) + " Ki="+ str(-Ki_TorqueModel) +" Kd =" + str(-Kd_TorqueModel) 
    Generate_Color_PZ_Map(CL_Unmotorized_System, TitleText)
    
    TitleText= "PZ Map of Pendulum Angle under PID Control \n For motorized system : \n Kp=" + str(-Kp_VoltModel) + " Ki="+ str(-Ki_VoltModel) +" Kd =" + str(-Kd_VoltModel) 
    Generate_Color_PZ_Map(CL_Motorized_System, TitleText)
###############################################################################
# Test with initial conditions
###############################################################################
if (False):
    PrintTitle("Test aux conditions initiales")
    # Vérifier le nombre d'états (= nombre de CI nécessaires)
    TestSS = ct.tf2ss(CL_Unmotorized_System)
    print(f"Nombre d'états: {TestSS.nstates}")
    
    X0 = [0.1,0,0]
    # Réponse aux CI uniquement
    t = np.linspace(0, 10, 1000)
    t, y = ct.initial_response(CL_Motorized_System, t, X0=X0)
    
    plt.figure()
    plt.plot(t, -y)
    plt.xlabel('Temps (s)')
    plt.ylabel('Theta (rad)')
    plt.title(f'Réponse aux CI: X0 = {X0}')
    plt.grid()
    plt.show()


###############################################################################
# Simplified Method (dominant poles)

# TORQUE BASED MODEL
###############################################################################  

PrintTitle("Simplified method (dominant poles) - Torque based solution")

# Desired poles (must be entered manually)
Desired_poles = [-40, -15, -0.5]
Desired_poles = [-22000, -2.8, -1]
Desired_poles = [-10000, -10, -1]


# Show the initial transfer function
print("\nInitial TF of torque based solution:")
print("\n====================================\n")
print(TF_Unmotorized_Theta)
p = ct.poles(TF_Unmotorized_Theta)
print("Poles of the TF: \n",p)

# Calculate a simplified denominator model based on dominant poles method
# Check PZ map and remove poles which are not domiant
# Then create a factorized polynom by hand
# Note: In this specific case, we have not simplified the TF as denominater is order 2'
Den_Polynom = (s - 14.95482461) * (s + 14.95482461)
print("\nDenominator Polynom = \n")
print(Den_Polynom)

# Generate simplified TF based on initial TF dominant poles, values entered manually in source code 
# Numerator is the initial numerator
#Denominator is the simplified model
Simplified_Num = [-3297]
Simplified_Den = [1, 0, -223.6]  
Simplified_TF = ct.tf(Simplified_Num,Simplified_Den)
print("Simplified TF of torque based solution:")
print("\n=====================================\n")
print(Simplified_TF)
Generate_PZ_Map(Simplified_TF, "PZ Map for torque based simplified system")

# Create a polynom using the desired poles values
Desired_poly = np.poly(Desired_poles)
print("\n Desired polynom based on dominant poles:")
PrintPolynomial(Desired_poly)


print("\n" + "="*80)
print("PID Coeficients calculation")
print("="*80)
# Le polynôme caractéristique en boucle fermée est :
# s·den(s) + num·(Kd·s² + Kp·s + Ki) = polynôme désiré

# Construction du système d'équations linéaires
# s·den(s) donne les coefficients [an, an-1, ..., a0, 0]
s_den = np.append(Simplified_Den, 0)
# print(s_den)

# On doit résoudre : s·den(s) + K_num·[Kd, Kp, Ki] = desired_poly
# Réorganisé : K_num·[Kd, Kp, Ki] = desired_poly - s·den(s)
# Matrice du système (positions des coefficients Kd, Kp, Ki)
n = len(Desired_poly)
A = np.zeros((n, 3))

# Kd multiplie s² : position 2 à partir de la fin
# Kp multiplie s  : position 1 à partir de la fin
# Ki multiplie 1  : position 0 à partir de la fin
if n >= 3:
    A[n-3, 0] = Simplified_Num[0]  # Coefficient de Kd·s²
if n >= 2:
    A[n-2, 1] = Simplified_Num[0]  # Coefficient de Kp·s
A[n-1, 2] = Simplified_Num[0]      # Coefficient de Ki

# Vecteur second membre
b = Desired_poly - s_den

# Résolution du système (seulement les 3 dernières équations)
# car on a seulement 3 inconnues (Kd, Kp, Ki)
A_reduced = A[-3:, :]
b_reduced = b[-3:]

pid_coeffs = solve(A_reduced, b_reduced)

Kd = pid_coeffs[0]
Kp = pid_coeffs[1]
Ki = pid_coeffs[2]

print(f"   Kd = {-Kd:.2f}")
print(f"   Kp = {-Kp:.2f}")
print(f"   Ki = {-Ki:.2f}")

###############################################################################
# Generate PID based on simplified model
############################################################################### 

PrintTitle("Simplified method (dominant poles) - Unmotorized solution \n Parallel PID controller generation")
PID_num = [Kd, Kp, Ki]  
PID_den = [1, 0] 

PID_TF = ct.tf(PID_num, PID_den)

print(PID_TF)

###############################################################################
# CLosed Loop system - PID in Feedback
###############################################################################
PrintTitle("CLosed Loop system generation - Simplified model (PID in feedback)")

# For Theta 
Simplified_Angle_PID_Control_inFeedback = ct.feedback(Simplified_TF, PID_TF)
print(Simplified_Angle_PID_Control_inFeedback)


###############################################################################
# CLosed Loop system - PID in Feedback
###############################################################################
PrintTitle("CLosed Loop system generation - Full model (PID in feedback)")

# For Theta 
Angle_PID_Control_inFeedback = ct.feedback(TF_Unmotorized_Theta, PID_TF)
print(Angle_PID_Control_inFeedback)

###############################################################################
# Impulse response - PID Controlled (both models)
###############################################################################
PrintTitle("Impulse response - PID Controlled")

t = np.linspace(0, 5, 1000)

t, y1 = ct.impulse_response(Simplified_Angle_PID_Control_inFeedback, T=t)

Fig_Count = Fig_Count + 1
TitleText= "Response of Theta to an Impulse Disturbance under PID Control\n Simplified model \n Kp=" + f"{-Kp:.2f}" + " Ki= "+  f"{-Ki:.2f}"  +" Kd =" +  f"{-Kd:.2f}" 
plt.figure(Fig_Count)
plt.plot(t,-y1)
plt.title(TitleText)
plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Pendulum Angle (rad)")

t, y2 = ct.impulse_response(Angle_PID_Control_inFeedback, T=t)

Fig_Count = Fig_Count + 1
TitleText= "Response of Theta to an Impulse Disturbance under PID Control\n Full model \n Kp=" + f"{-Kp:.2f}" + " Ki= "+  f"{-Ki:.2f}"  +" Kd =" +  f"{-Kd:.2f}" 
plt.figure(Fig_Count)
plt.plot(t,-y2)
plt.title(TitleText)
plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Pendulum Angle (rad)")

Fig_Count = Fig_Count + 1
TitleText= "Response of Theta to an Impulse Disturbance under PID Control\n  Kp= " + f"{-Kp:.2f}" + " Ki= "+  f"{-Ki:.2f}"  +" Kd =" +  f"{-Kd:.2f}" 
plt.figure(Fig_Count)
plt.plot(t, -y2, label='Full model', linewidth=2)
plt.plot(t, -y1, label='Simplified model', linewidth=2, linestyle='--', color ='r')
plt.title(TitleText)
plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Pendulum Angle (rad)")
plt.legend()

###############################################################################
# Stability - PID Controlled - PZMap
###############################################################################

TitleText= "PZ Map of Pendulum Angle under PID Control \n For simplified torque controlled model : \n Kp=" + f"{-Kp:.2f}" + " Ki="+ f"{-Ki:.2f}" +" Kd =" + f"{-Kd:.2f}" 
Generate_Color_PZ_Map(Simplified_Angle_PID_Control_inFeedback, TitleText)

TitleText= "PZ Map of Pendulum Angle under PID Control \n For torque controlled model  : \n Kp=" + f"{-Kp:.2f}" + " Ki="+ f"{-Ki:.2f}" +" Kd =" + f"{-Kd:.2f}" 
Generate_Color_PZ_Map(Angle_PID_Control_inFeedback, TitleText)

###############################################################################
# Simplified Method (dominant poles)

# VOLTAGE BASED MODEL
###############################################################################  

PrintTitle("Simplified method (dominant poles) - Voltage based solution")

# Desired poles (must be entered manually)
Desired_poles = [-53.5, -28.7, -0.45]   
Desired_poles = [-40, -5, -1]
Desired_poles = [-40, -15, -0.5]
# Desired_poles = [-100, -6, -1]
# Show the initial transfer function
print("\nInitial TF of voltage based solution:")
print("\n====================================\n")
print(TF_Motorized_Theta)
p = ct.poles(TF_Motorized_Theta)
print("Poles of the TF: \n",p)

# Calculate a simplified denominator model based on dominant poles method
# Check PZ map and remove poles which are not domiant
# Then create a factorized polynom by hand
# Note: In this specific case, we have not simplified the TF as denominater is order 2'
Den_Polynom = (s - 14.8813061) * (s + 0.00297920184)

print("\nDenominator Polynom = \n")
print(Den_Polynom)

# Generate simplified TF based on initial TF dominant poles, values entered manually in source code 
# Numerator is the initial numerator
#Denominator is the simplified model
Simplified_Num = [-13.74]
Simplified_Den = [1, -14.88, -0.04433]  
Simplified_TF = ct.tf(Simplified_Num,Simplified_Den)
print("Simplified TF of voltage based solution:")
print("\n=====================================\n")
print(Simplified_TF)
Generate_PZ_Map(Simplified_TF, "PZ Map for torque based simplified system")

# Create a polynom using the desired poles values
Desired_poly = np.poly(Desired_poles)
print("\n Desired polynom based on dominant poles:")
PrintPolynomial(Desired_poly)


print("\n" + "="*80)
print("PID Coeficients calculation")
print("="*80)
# Le polynôme caractéristique en boucle fermée est :
# s·den(s) + num·(Kd·s² + Kp·s + Ki) = polynôme désiré

# Construction du système d'équations linéaires
# s·den(s) donne les coefficients [an, an-1, ..., a0, 0]
s_den = np.append(Simplified_Den, 0)

print("s_den =")
PrintPolynomial(s_den)
print("\n")

# On doit résoudre : s·den(s) + K_num·[Kd, Kp, Ki] = desired_poly
# Réorganisé : K_num·[Kd, Kp, Ki] = desired_poly - s·den(s)
# Matrice du système (positions des coefficients Kd, Kp, Ki)
n = len(Desired_poly)
A = np.zeros((n, 3))

print("n = ", n)
print("\n")
print("A = ", A)
print("\n")

# Kd multiplie s² : position 2 à partir de la fin
# Kp multiplie s  : position 1 à partir de la fin
# Ki multiplie 1  : position 0 à partir de la fin
if n >= 3:
    A[n-3, 0] = Simplified_Num[0]  # Coefficient de Kd·s²
if n >= 2:
    A[n-2, 1] = Simplified_Num[0]  # Coefficient de Kp·s
A[n-1, 2] = Simplified_Num[0]      # Coefficient de Ki

print("A = ", A)
print("\n")

# Vecteur second membre
b = Desired_poly - s_den

print("b =")
PrintPolynomial(b)
print("\n")

# Résolution du système (seulement les 3 dernières équations)
# car on a seulement 3 inconnues (Kd, Kp, Ki)
A_reduced = A[-3:, :]
b_reduced = b[-3:]
print("A_reduced = ", A_reduced)
print("\n")
print("b_reduced = ", b_reduced)
print("\n")

pid_coeffs = solve(A_reduced, b_reduced)

Kd = pid_coeffs[0]
Kp = pid_coeffs[1]
Ki = pid_coeffs[2]

print(f"   Kd = {-Kd:.2f}")
print(f"   Kp = {-Kp:.2f}")
print(f"   Ki = {-Ki:.2f}")

###############################################################################
# Generate PID based on simplified model
############################################################################### 

PrintTitle("Simplified method (dominant poles) - Unmotorized solution \n Parallel PID controller generation")
PID_num = [Kd, Kp, Ki]  
PID_den = [1, 0] 

PID_TF = ct.tf(PID_num, PID_den)

print(PID_TF)

###############################################################################
# CLosed Loop system - PID in Feedback
###############################################################################
PrintTitle("CLosed Loop system generation - Simplified model (PID in feedback)")

# For Theta 
Simplified_Angle_PID_Control_inFeedback = ct.feedback(Simplified_TF, PID_TF)
print(Simplified_Angle_PID_Control_inFeedback)


###############################################################################
# CLosed Loop system - PID in Feedback
###############################################################################
PrintTitle("CLosed Loop system generation - Full model (PID in feedback)")

# For Theta 
Angle_PID_Control_inFeedback = ct.feedback(TF_Unmotorized_Theta, PID_TF)
print(Angle_PID_Control_inFeedback)

###############################################################################
# Impulse response - PID Controlled (both models)
###############################################################################
PrintTitle("Impulse response - PID Controlled")

t = np.linspace(0, 5, 1000)

t, y1 = ct.impulse_response(Simplified_Angle_PID_Control_inFeedback, T=t)

Fig_Count = Fig_Count + 1
TitleText= "Response of Theta to an Impulse Disturbance under PID Control\n Simplified model \n Kp=" + f"{-Kp:.2f}" + " Ki= "+  f"{-Ki:.2f}"  +" Kd =" +  f"{-Kd:.2f}" 
plt.figure(Fig_Count)
plt.plot(t,-y1)
plt.title(TitleText)
plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Pendulum Angle (rad)")

t, y2 = ct.impulse_response(Angle_PID_Control_inFeedback, T=t)

Fig_Count = Fig_Count + 1
TitleText= "Response of Theta to an Impulse Disturbance under PID Control\n Full model \n Kp=" + f"{-Kp:.2f}" + " Ki= "+  f"{-Ki:.2f}"  +" Kd =" +  f"{-Kd:.2f}" 
plt.figure(Fig_Count)
plt.plot(t,-y2)
plt.title(TitleText)
plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Pendulum Angle (rad)")

Fig_Count = Fig_Count + 1
TitleText= "Response of Theta to an Impulse Disturbance under PID Control\n  Kp= " + f"{-Kp:.2f}" + " Ki= "+  f"{-Ki:.2f}"  +" Kd =" +  f"{-Kd:.2f}" 
plt.figure(Fig_Count)
plt.plot(t, -y2, label='Full model', linewidth=2)
plt.plot(t, -y1, label='Simplified model', linewidth=2, linestyle='--', color ='r')
plt.title(TitleText)
plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Pendulum Angle (rad)")
plt.legend()

###############################################################################
# Stability - PID Controlled - PZMap
###############################################################################

TitleText= "PZ Map of Pendulum Angle under PID Control \n For voltage controlled model : \n Kp=" + f"{-Kp:.2f}" + " Ki="+ f"{-Ki:.2f}" +" Kd =" + f"{-Kd:.2f}" 
Generate_Color_PZ_Map(Simplified_Angle_PID_Control_inFeedback, TitleText)

TitleText= "PZ Map of Pendulum Angle under PID Control \n For voltage controlled model  : \n Kp=" + f"{-Kp:.2f}" + " Ki="+ f"{-Ki:.2f}" +" Kd =" + f"{-Kd:.2f}" 
Generate_Color_PZ_Map(Angle_PID_Control_inFeedback, TitleText)

###############################################################################
# Actuator values verification
###############################################################################
t = np.linspace(0, 5, 500)
t, y2 = ct.impulse_response(Angle_PID_Control_inFeedback, T=t)
dt = t[1] - t[0]

P_signal = Kp * (y2)
I_signal = Ki * np.cumsum(y2) * dt
D_signal = Kd * np.gradient(y2, dt)
n = 10  # Number of samples to be ignored as we have a Dirac Impulse
D_signal[:n] = 0

# Signal de correction total
PID_total = P_signal + I_signal + D_signal

Fig_Count = Fig_Count + 1
fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

axes[0].plot(t, -y2, color='tab:purple')
axes[0].set_ylabel("Pendulum Angle (rad)")
axes[0].set_title("Impulse response")
axes[0].grid(True)

axes[1].plot(t, P_signal, color='tab:blue')
axes[1].set_ylabel('Proportional correction [V]')
axes[1].set_title("Proportional")
axes[1].grid(True)

axes[2].plot(t, I_signal, color='tab:orange')
axes[2].set_ylabel('Integral correction [V]')
axes[2].set_title("Integral")
axes[2].grid(True)

axes[3].plot(t, D_signal, color='tab:green')
axes[3].set_ylabel('Derivative correction [V]')
axes[3].set_title("Derivative")
axes[3].grid(True)
axes[3].set_xlabel('Time [s]')

plt.tight_layout()
plt.show()


###############################################################################
# Test with initial conditions
###############################################################################
if (True):
    Initial_Angle = 0.2
    Title = "Test at initial angle of " + str(Initial_Angle) + " radians"
    PrintTitle(Title)
    # Vérifier le nombre d'états (= nombre de CI nécessaires)
    TestSS = ct.tf2ss(Angle_PID_Control_inFeedback)
    print(f"Number of states: {TestSS.nstates}")
    

    X0 = [Initial_Angle,0,0]
    t = np.linspace(0, 5, 500)
    t, y2 = ct.initial_response(Angle_PID_Control_inFeedback, t, X0=X0)
    
    plt.figure()
    plt.plot(t, y2)
    plt.xlabel('Temps (s)')
    plt.ylabel('Theta (rad)')
    plt.title(f'Initial conditions: X0 = {X0}')
    plt.grid()
    plt.show()
    
    dt = t[1] - t[0]

    P_signal = Kp * (-y2)
    I_signal = Ki * np.cumsum(-y2) * dt
    D_signal = Kd * np.gradient(-y2, dt)
    n = 2  # Number of samples to be ignored 
    D_signal[:n] = 0

    # Signal de correction total
    PID_total = P_signal + I_signal + D_signal

    Fig_Count = Fig_Count + 1
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

    axes[0].plot(t, y2, color='tab:purple')
    axes[0].set_ylabel("Pendulum Angle (rad)")
    axes[0].set_title(Title)
    axes[0].grid(True)

    axes[1].plot(t, P_signal, color='tab:blue')
    axes[1].set_ylabel('Proportional correction [V]')
    axes[1].set_title("Proportional")
    axes[1].grid(True)

    axes[2].plot(t, I_signal, color='tab:orange')
    axes[2].set_ylabel('Integral correction [V]')
    axes[2].set_title("Integral")
    axes[2].grid(True)

    axes[3].plot(t, D_signal, color='tab:green')
    axes[3].set_ylabel('Derivative correction [V]')
    axes[3].set_title("Derivative")
    axes[3].grid(True)
    axes[3].set_xlabel('Time [s]')

    plt.tight_layout()
    plt.show()