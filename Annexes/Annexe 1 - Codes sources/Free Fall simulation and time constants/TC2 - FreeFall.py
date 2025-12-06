# -*- coding: utf-8 -*-
"""
Created on Wed Feb 26 11:46:37 2025

Solution codée en utilisant l'équation 5.1.32 '
comparée avec l'équation du pendule simple
Les valeurs numériques sont les valeurs du robot ELEGOO

'
@author: Catri
"""

import numpy as np
import math
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import matplotlib.transforms as tr
from matplotlib.animation import PillowWriter
from PIL import Image
import io

#----------------------------------------------------------------------------------
# Robot Physcal Parameters
#----------------------------------------------------------------------------------

#Hauteur des CDG individuels
RoueCouplage_roue = 0.0325                 #CDG Roue et rayon roue
Moteurs = 0.0325 + 0.007                    #CDG Moteur f(roue)
Base_alu = Moteurs + 0.0305         
Electronique = Base_alu + 0.0125
Piliers_M3X45 = Electronique + 0.01
Plastique_etage1 = Piliers_M3X45 + 0.0225
Piliers_M3X23 = Plastique_etage1 + 0.02375
Etage_sup = Piliers_M3X23 + 0.00625

#Masse des éléments individuels
Masse_RoueCouplage_roue = 0.086
Masse_Moteurs = 0.294
Masse_Base_alu = 0.104
Masse_Electronique = 0.050
Masse_Piliers_M3X45 = 0.020
Masse_Plastique_etage1 = 0.040
Masse_Piliers_M3X23 = 0.008
Masse_Etage_sup = 0.147

Masse_TOT = Masse_RoueCouplage_roue + Masse_Moteurs + Masse_Base_alu + Masse_Electronique + Masse_Piliers_M3X45 + Masse_Plastique_etage1 + Masse_Piliers_M3X23 + Masse_Etage_sup

g = 9.81        # Accélération due à la gravité (m/s^2)
Mb = Masse_Moteurs + Masse_Base_alu + Masse_Electronique + Masse_Piliers_M3X45 + Masse_Plastique_etage1 + Masse_Piliers_M3X23 + Masse_Etage_sup
Mw = Masse_RoueCouplage_roue
Rw = RoueCouplage_roue    #wheel radius

#Inertie du parallélipipède
Ip = (1/12) * Mb * ( 0.082**2 + (Etage_sup - Moteurs)**2 )

#Inertie du parallélipipède axe de rotation décalé sur axe roues
Ib = Ip + ( Mb * ((Ip - RoueCouplage_roue)**2) )

#Inertie des roues
Iw = Mw * (Rw**2)

#Centre de masse du pendule simple
COM_Pendulum = ((1/Masse_TOT) * ((RoueCouplage_roue*Masse_RoueCouplage_roue)+(Moteurs*Masse_Moteurs)+(Base_alu*Masse_Base_alu)+(Electronique*Masse_Electronique)+(Piliers_M3X45*Masse_Piliers_M3X45)+(Plastique_etage1*Masse_Plastique_etage1)+(Piliers_M3X23*Masse_Piliers_M3X23)+(Etage_sup*Masse_Etage_sup)) )

#Centre de masse du corps du robot
COM_Body = ( (1/Mb) * ((Moteurs*Masse_Moteurs)+(Base_alu*Masse_Base_alu)+(Electronique*Masse_Electronique)+(Piliers_M3X45*Masse_Piliers_M3X45)+(Plastique_etage1*Masse_Plastique_etage1)+(Piliers_M3X23*Masse_Piliers_M3X23)+(Etage_sup*Masse_Etage_sup)) )

# Distance centre de masses du corps du robot par rapport à l'axe des roues
l = COM_Body - (RoueCouplage_roue + 0.007)

# Size of the body for animation purposes
ANI_Body_Lenght = 0.08
ANI_Body_Height = Etage_sup
ANI_Rw = Rw


print('Masse Totale :', Masse_TOT)
print('Masse Corps :', Mb)
print('hauteur CDG Pendule :', COM_Pendulum)
print('hauteur CDG Corps seul :', COM_Body)
print('Distance CDG Corps - Axe roue :', l)
print('Inertie du parallélipipède :', Ip)
print('Inertie du parallélipipède - Axe décalé :', Ib)

#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------


#----------------------------------------------------------------------------------
#Mathematical model constants
#----------------------------------------------------------------------------------
M = Mb + 2 * ( Mw + (Iw/(Rw**2)) ) - ( (( Mb*l )**2) / (( Mb*(l**2) + Ib) ) )
J = Ib + (Mb * l**2) -  ( (Mb * l)**2/ (Mb + 2 * (Mw + (Iw / (Rw**2) ) ) ) )
C1 = (1/M) * (   ( g * (( Mb*l )**2)  )  /  ( Mb*(l**2) + Ib )    )
C2 = (2/M) * ( (1/Rw) + ( ( Mb*l ) / ( Mb*(l**2) + Ib) ) )
C3 = Mb * g * l / J
C4 = (2/(J*Rw))  *  ( Rw + ( ( Mb*l )/ (Mb+ (2 *( Mw + (Iw/(Rw**2)) )))) )
#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------


#----------------------------------------------------------------------------------
# Simulation Parameters
#----------------------------------------------------------------------------------
Samples = 100
SimulationTime = 0.3
LimitAngle = 10
Graph_MaxTime = 0.3

# Iniial conditions
theta0TWSBR = np.radians(1)      # Angle initial (en radians)
theta0Pendulum = np.radians(1)      # Angle initial (en radians)
omega0 = 0.0               # Vitesse angulaire initiale (rad/s)
y0TWSBR = [theta0TWSBR, omega0]
y0Pendulum = [theta0Pendulum, omega0]

# Discrete time vector for graph
t = np.linspace(0, SimulationTime, Samples)  # 0 à SimulationTime secondes, Samples points

#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------

# Animation du robot
def TWSBR_Frame(xpos=0, theta=np.pi, i=-1):
    
    # Clear the current figure.
    plt.clf()
    
    #Size of the image
    plt.figure(figsize=(10, 5))
    fig=plt.gcf()       #create reference to the current figure
    ax=fig.gca()        #get the reference to the current axises
    
    #limit of x & y axis
    plt.xlim(-0.25, 0.25)
    plt.ylim(0, 0.25)
    plt.title(i)



    # Rotate Body of the robot
    Body_Centre_of_Rot = (xpos, ANI_Rw)
    Rectangle_Transform = tr.Affine2D().rotate_deg_around(Body_Centre_of_Rot[0],Body_Centre_of_Rot[1],np.degrees(theta)) + ax.transData
    
    #create patches
    Robot_Body = plt.Rectangle(xy=(xpos-(ANI_Body_Lenght/2),ANI_Rw), width=ANI_Body_Lenght, height=ANI_Body_Height, fill=False, linewidth=1, transform=Rectangle_Transform)
    Robot_Wheels = plt.Circle(xy=(xpos,ANI_Rw), radius=ANI_Rw, color='silver',alpha=0.5, fill=True,edgecolor='b', linewidth=1)

    Wheels_Marker1 = plt.Circle(xy=(xpos+ANI_Rw/2*math.cos(theta),ANI_Rw+(ANI_Rw/2)*math.sin(theta)), radius=ANI_Rw/5,fill=True, linewidth=5)
    Wheels_Marker2 = plt.Circle(xy=(xpos-ANI_Rw/2*math.cos(theta),ANI_Rw-(ANI_Rw/2)*math.sin(theta)), radius=ANI_Rw/5,fill=True, linewidth=5)
    Wheels_Marker3 = plt.Circle(xy=(xpos-ANI_Rw/2*math.sin(theta),ANI_Rw+(ANI_Rw/2)*math.cos(theta)), radius=ANI_Rw/5,fill=True, linewidth=5)
    Wheels_Marker4 = plt.Circle(xy=(xpos+ANI_Rw/2*math.sin(theta),ANI_Rw-(ANI_Rw/2)*math.cos(theta)), radius=ANI_Rw/5,fill=True, linewidth=5)

    #Add patches to the figure
    ax.add_patch(Robot_Body)
    ax.add_patch(Robot_Wheels)
    ax.add_patch(Wheels_Marker1)
    ax.add_patch(Wheels_Marker2)
    ax.add_patch(Wheels_Marker3)
    ax.add_patch(Wheels_Marker4)
    

    if i > 0:
        plt.title("Time = " + str(  round(i*SimulationTime/Samples , 2)  ) +  " seconds" )


def calculateXPos(CurrentAngle):
    return (theta0Pendulum - CurrentAngle) * ANI_Rw

#----------------------------------------------------------------------------------
# Differential equations of TWSBR (Free fall model)
#----------------------------------------------------------------------------------
def TWSBR_derivs(y, t, g, l):
    theta, omega = y
    dydt = [omega,  C3 * theta ]
    # dydt = [omega,  C3 * math.sin(theta) ]
    print(dydt)
    return dydt
#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------

#----------------------------------------------------------------------------------
# Differential equations of simple pendulum (Free fall model)
#----------------------------------------------------------------------------------
def pendulum_derivs(y, t, g, l):
    theta, omega = y
    dydt = [omega,  (g/l) * math.sin(theta)]
    return dydt
#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------

#----------------------------------------------------------------------------------
# Solving of both differential equations 
#----------------------------------------------------------------------------------
solTWSBR = odeint(TWSBR_derivs, y0TWSBR, t, args=(g, l))
solPendulum = odeint(pendulum_derivs, y0Pendulum, t, args=(g, COM_Pendulum))

# Solutions extraction
thetaTWSBR = solTWSBR[:, 0]
omegaTWSBR = solTWSBR[:, 1]
thetaPendulum = solPendulum[:, 0]
omegaPendulum = solPendulum[:, 1]


#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------

#----------------------------------------------------------------------------------
# Results Plotting - Graph
#----------------------------------------------------------------------------------
plt.plot(t, np.degrees(thetaPendulum), label='Simple Pendulum model', color = 'red')
plt.plot(t, np.degrees(thetaTWSBR), label='Complex robot model')
# plt.plot(t, omega, label='Vitesse angulaire (rad/s)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (°)')
plt.xlim(left=0 , right=Graph_MaxTime)
plt.ylim(bottom=0 , top=LimitAngle+2)
plt.title("Chute du pendule")
plt.grid()

#Plot a cross at Limit Angle and extract value
for i in range(len(solTWSBR)):
    #print(np.degrees(solTWSBR[i]))
    
    if np.degrees(solTWSBR[i,0]) > LimitAngle:
        CurrentTime = i*SimulationTime/Samples
        print("For center of mass at height", str(l), " " , str(LimitAngle) , " degrees reached at time: ", round(CurrentTime,4), " seconds. Frequency=",1/CurrentTime )
        plt.axvline(x=CurrentTime, color="r",ls=':', ymax=1)
        plt.axhline(y=np.degrees(solTWSBR[i,0]),color="r",ls=':')
        
        MyLabel = "  " + str(round(np.degrees(solTWSBR[i,0]))) + "° à " + str(round(CurrentTime,2)) + " secondes"
        plt.annotate(text = MyLabel, xy=(CurrentTime,np.degrees(solTWSBR[i,0]) ) )
        
        break
    
plt.legend()      
plt.show()
#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------

#----------------------------------------------------------------------------------
# Creating GIF animation
#----------------------------------------------------------------------------------


frames = []

for i in range(len(solPendulum)):

    TWSBR_Frame(calculateXPos(solPendulum[i,0]), solPendulum[i,0],i)
    buf = io.BytesIO()
    plt.savefig(buf, format="png")
    buf.seek(0)
    frames.append(Image.open(buf))
    
# Create and save the animated GIF
frames[0].save(
    "Free Fall.gif",
    save_all=True,
    append_images=frames[1:],
    duration=SimulationTime,
    loop=0,
)


Time_List = []
SimplePend_List = []
TWSBR_List = []
LimitReached = False
frames2 = []

for i in range(len(solPendulum)):
    
    plt.figure(figsize=(10, 5))
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (°)')
    plt.xlim(left=0 , right=Graph_MaxTime)
    plt.ylim(bottom=0 , top=LimitAngle+5)
    plt.title("Chute du pendule")
    
    Time_List.append(t[i])
    SimplePend_List.append(np.degrees(thetaPendulum[i]))
    TWSBR_List.append(np.degrees(thetaTWSBR[i]))
    
    plt.plot(Time_List, SimplePend_List, label='Simple Pendulum model', color = 'red')
    plt.plot(Time_List, TWSBR_List, label='Complex robot model')
    plt.grid()

    '''    
    if TWSBR_List[i] > LimitAngle and LimitReached == False:
        LimitReached = True
        CurrentTime = Time_List[i]
        CurrentAngle = TWSBR_List[i]
        CurrentSample = i
        MyLabel = "  " + str(LimitAngle) + "° à " + str(round(CurrentTime,2)) + " secondes"
    if LimitReached == True:    
        plt.axvline(x=CurrentTime, color="r",ls=':', ymax=1)
        plt.axhline(y=LimitAngle,color="r",ls=':')
        plt.annotate(text = MyLabel, xy=(Time_List[CurrentSample-(Samples/10)],CurrentAngle ) )
        '''        
    buf2 = io.BytesIO()
    plt.savefig(buf2, format="png")
    buf2.seek(0)
    frames2.append(Image.open(buf2))
    
# Create and save the animated GIF 2
frames2[0].save(
    "Free Fall Graph.gif",
    save_all=True,
    append_images=frames2[1:],
    duration=SimulationTime,
    loop=0,)