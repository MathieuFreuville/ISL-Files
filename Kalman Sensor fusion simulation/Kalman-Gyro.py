"""
Filtre de Kalman pour fusion gyroscope/accéléromètre
Estimation de l'angle d'inclinaison d'un système

Le gyroscope mesure la vitesse angulaire mais dérive dans le temps (accumulation d'erreurs)
L'accéléromètre mesure l'angle d'inclinaison mais est très bruité
Le filtre de Kalman fusionne les deux pour obtenir une estimation optimale

Ce programme a été inspiré par l'exemple suivant:
https://lucidar.me/fr/kalman-filters/example-of-kalman-filter/'

@author: Mathieu Freuville
"""

import numpy as np
import matplotlib.pyplot as plt

# Paramètres de simulation
dt = 0.01  # Pas de temps [s] (100 Hz)
GyroStd = 0.5  # Écart-type du gyroscope [°/s]
GyroBias = 0.1  # Biais (dérive) du gyroscope [°/s]
AccelStd = 2.0  # Écart-type de l'accéléromètre [°]


###############################################################################
# Trajectory and sensor simulation
###############################################################################
# Time
t = np.arange(0, 20 + dt, dt)
t = np.arange(0, 100 + dt, dt)
n = len(t)

# Simulation of oscillating system (various possibilities)
angle_real = 30 * np.sin(0.5 * t) * np.exp(-0.05 * t)
angle_real = 5 * np.sin(0.5 * t) * np.exp(-0.05 * t)
angle_real = 5 * np.sin(0.5 * t) 

# Angular velocity (dtheta / dt)
omega_real = np.concatenate([[0], np.diff(angle_real) / dt])

###############################################################################
# Gyroscope with noise and bias
###############################################################################
omega_gyro = omega_real + GyroStd * np.random.randn(n) + GyroBias

# Gyro integration
angle_gyro = np.zeros(n)
for i in range(1, n):
    angle_gyro[i] = angle_gyro[i-1] + omega_gyro[i-1] * dt

###############################################################################
# Accelero with noise 
###############################################################################
angle_accel = angle_real + AccelStd * np.random.randn(n)

###############################################################################
# Shows data
###############################################################################

# Angle 
plt.figure(figsize=(12, 4))
plt.plot(t, angle_real, 'k', linewidth=2, label='Real Angle')
plt.grid(True)
plt.xlabel('Time [s]')
plt.ylabel('Angle [°]')
plt.title('Real Angle Theta=f(t)')
plt.legend()
plt.tight_layout()

# Angular speed from gyro
plt.figure(figsize=(12, 4))
plt.plot(t, omega_gyro, 'b', alpha=0.6, label='Gyroscope')
plt.plot(t, omega_real, 'k', linewidth=2, label='Real angular speed')
plt.plot(t, omega_real + 3 * GyroStd, 'k-.', linewidth=1, alpha=0.5)
plt.plot(t, omega_real - 3 * GyroStd, 'k-.', linewidth=1, alpha=0.5)
plt.grid(True)
plt.xlabel('Time [s]')
plt.ylabel('Angular speed [°/s]')
plt.title('Angular speed from gyroscope ω=f(t)')
plt.legend()
plt.tight_layout()

# Integrated gyro vs accelero
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(t, angle_real, 'k', linewidth=2, label='Real Angle')
plt.plot(t, angle_gyro, 'b', alpha=0.7, label='Integrated Gyro')
plt.grid(True)
plt.xlabel('Time [s]')
plt.ylabel('Angle [°]')
plt.title('Integrated Gyro')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(t, angle_real, 'k', linewidth=2, label='Real Angle')
plt.plot(t, angle_accel, 'r', alpha=0.5, label='Accelerometer')
plt.plot(t, angle_real + 3 * AccelStd, 'k-.', linewidth=1, alpha=0.5)
plt.plot(t, angle_real - 3 * AccelStd, 'k-.', linewidth=1, alpha=0.5)
plt.grid(True)
plt.xlabel('Time [s]')
plt.ylabel('Angle [°]')
plt.title('Accelerometer angle')
plt.legend()
plt.tight_layout()

###############################################################################
# Kalman Filter
###############################################################################



B = dt  # command matrix
F = 1   # state transition matrix
Q = (GyroStd * dt) ** 2  # Process noise Covariance  (error on integrated gyro)
H = 1   # Observation matrix
R = AccelStd ** 2  # Measure noise covariance (error on accelero)

# Initialisation
theta_hat = np.zeros(n)  # Estimated state (angle)
y_hat = np.zeros(n)      # Innovation 
P = np.zeros(n)          # State uncertainty
P[0] = 100               # Initial state uncertainty (High)

# Kalman filter loop
for i in range(1, n):
    
    # ========== Prédiction (based ongyroscope) ==========
    
    # State prediction
    theta_hat[i] = F * theta_hat[i-1] + B * omega_gyro[i-1]
    
    # Uncertainty prediction
    P[i] = F * P[i-1] * F + Q
    
    # ========== Update (with accelero) ==========
    
    # Innovation 
    y_hat[i] = angle_accel[i] - H * theta_hat[i]
    
    # Innovation covariance
    S = H * P[i] * H + R
    
    # Gain Kalman
    K = P[i] * H / S
    
    # State update
    theta_hat[i] = theta_hat[i] + K * y_hat[i]
    
    # Uncertainty update
    P[i] = (1 - K * H) * P[i]

# ========== Results ==========

plt.figure(figsize=(14, 8))


plt.subplot(2, 1, 1)
plt.plot(t, angle_real, 'k', linewidth=2.5, label='Real Angle')
plt.plot(t, angle_gyro, 'b--', alpha=0.6, linewidth=1.5, label='Integrated Gyro')
plt.plot(t, angle_accel, 'r.', alpha=0.3, markersize=2, label='Accelerometer')
plt.plot(t, theta_hat, 'g', linewidth=2, label='Kalman Filter')
plt.grid(True, alpha=0.3)
plt.xlabel('Time [s]')
plt.ylabel('Angle [°]')
plt.title('Comparaison of different angle estimation methods')
plt.legend(loc='upper right')


plt.subplot(2, 1, 2)
plt.plot(t, angle_real, 'k', linewidth=2, label='Real Angle')
plt.plot(t, theta_hat, 'g', linewidth=2, label='Kalman Filter')
plt.plot(t, theta_hat + np.sqrt(P), 'r-.', linewidth=1.5, label='Uncertainty (±σ)')
plt.plot(t, theta_hat - np.sqrt(P), 'r-.', linewidth=1.5)
plt.fill_between(t, theta_hat - np.sqrt(P), theta_hat + np.sqrt(P), alpha=0.2, color='red')
plt.grid(True, alpha=0.3)
plt.xlabel('Time [s]')
plt.ylabel('Angle [°]')
plt.title('Kalman estimated with uncertainty envelop')
plt.legend(loc='upper right')
plt.tight_layout()

# ========== Error Graph ==========
plt.figure(figsize=(12, 6))

error_gyro = angle_real - angle_gyro
error_accel = angle_real - angle_accel
error_kalman = angle_real - theta_hat

plt.subplot(3, 1, 1)
plt.plot(t, error_gyro, 'b', linewidth=1)
plt.grid(True, alpha=0.3)
plt.ylabel('Error [°]')
plt.title('Error on integrated gyro')
plt.axhline(y=0, color='k', linestyle='--', alpha=0.5)

plt.subplot(3, 1, 2)
plt.plot(t, error_accel, 'r', linewidth=1)
plt.grid(True, alpha=0.3)
plt.ylabel('Error [°]')
plt.title('Error on accelero (noise)')
plt.axhline(y=0, color='k', linestyle='--', alpha=0.5)

plt.subplot(3, 1, 3)
plt.plot(t, error_kalman, 'g', linewidth=1)
plt.grid(True, alpha=0.3)
plt.xlabel('Time [s]')
plt.ylabel('Error [°]')
plt.title('Error on Kalman filter')
plt.axhline(y=0, color='k', linestyle='--', alpha=0.5)
plt.tight_layout()

plt.show()

# ========== Statistiques ==========

# Calcul des erreurs RMS
rmse_gyro = np.sqrt(np.mean(error_gyro**2))
rmse_accel = np.sqrt(np.mean(error_accel**2))
rmse_kalman = np.sqrt(np.mean(error_kalman**2))

# Dérive finale du gyroscope
drift_gyro = abs(angle_gyro[-1] - angle_real[-1])

print("\n" + "="*70)
print("RESULTS DE LA FUSION GYROSCOPE/ACCÉLÉROMÈTRE")
print("="*70)
print("\nERREUR QUADRATIQUE MOYENNE (RMSE):")
print(f"  Gyroscope intégré     : {rmse_gyro:8.4f} °  (dérive dans le temps)")
print(f"  Accéléromètre         : {rmse_accel:8.4f} °  (bruité mais stable)")
print(f"  Filtre de Kalman      : {rmse_kalman:8.4f} °  (optimal)")
print(f"\nGAIN DU FILTRE DE KALMAN:")
print(f"  vs Gyroscope          : {(rmse_gyro/rmse_kalman):.2f}x meilleur")
print(f"  vs Accéléromètre      : {(rmse_accel/rmse_kalman):.2f}x meilleur")
print(f"\nDÉRIVE DU GYROSCOPE:")
print(f"  Dérive finale         : {drift_gyro:.2f} °")
print(f"  Dérive par seconde    : {drift_gyro/t[-1]:.2f} °/s")
print(f"\nINCERTITUDE FINALE:")
print(f"  σ (écart-type)        : {np.sqrt(P[-1]):.4f} °")
print("="*70)