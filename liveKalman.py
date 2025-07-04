import serial, time
import numpy as np
from numpy import sin, cos, asin, atan2

# Serial connection to BNO055
SERIAL_PORT = '/dev/cu.usbmodem1101'
BAUD        = 115200
ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
time.sleep(2)  # wait for serial to initialize

# Constants
g = 9.81              # gravitational acceleration
h = 0.05              # time step
R = 0.0001 * np.eye(4)  # measurement noise covariance
Q = 0.1    * np.eye(4)  # process noise covariance
H = np.eye(4)           # observation matrix (identity here)

# Convert Euler angles to quaternion
def euler2quat(phi, th, psi):
    s1,c1 = sin(phi/2), cos(phi/2)
    s2,c2 = sin(th /2), cos(th /2)
    s3,c3 = sin(psi/2), cos(psi/2)
    return np.array([
        s1*c2*c3 - c1*s2*s3,
        s1*c2*s3 + c1*s2*c3,
       -s1*s2*c3 + c1*c2*s3,
        s1*s2*s3 + c1*c2*c3])

# Convert quaternion to Euler angles
def quat2euler(q):
    q1,q2,q3,q4 = q
    a11 =  q1**2 - q2**2 - q3**2 + q4**2
    a12 = 2*(q1*q2 + q3*q4)
    a13 = 2*(q1*q3 - q2*q4)
    a23 = 2*(q2*q3 + q1*q4)
    a33 =  q3**2 - q1**2 - q2**2 + q4**2
    phi   = atan2(a23, a33)
    theta = atan2(-a13, np.sqrt(a11**2 + a12**2))
    psi   = atan2(a12, a11)
    return np.array([phi, theta, psi])

# Compute B matrix from angular rates
def getB(w1,w2,w3):
    return np.array([[ 0, 0.5*w3,-0.5*w2, 0.5*w1],
                     [-0.5*w3,0, 0.5*w1, 0.5*w2],
                     [ 0.5*w2,-0.5*w1,0, 0.5*w3],
                     [-0.5*w1,-0.5*w2,-0.5*w3,0]])

# Initialize state
quat_prev = euler2quat(0,0,0)  # initial level orientation
P_prev     = np.eye(4)         # initial error covariance

print("Reading BNO055…  Ctrl-C to stop")
try:
    while True:
        line = ser.readline().decode().strip()
        if not line:
            continue

        try:
            # Parse IMU data: accel (ax, ay, az), gyro (gx, gy, gz), mag (magX, magY)
            ax, ay, az, gx, gy, gz, magX, magY = map(float, line.split(','))
        except ValueError:
            print("Bad line:", line)
            continue

        # Compute Euler angles from accelerometer and magnetometer
        theta_meas = asin(ax / g)
        phi_meas   = atan2(ay, az)
        declination_deg = -7.7
        psi_meas = atan2(magY, magX) + np.radians(declination_deg)
        quat_meas  = euler2quat(phi_meas, theta_meas, psi_meas)

        # Angular rates in rad/s
        w1, w2, w3 = np.radians([gx, gy, gz])

        # State prediction (quaternion propagation)
        A_k = (h/2)*np.array([[2/h, w3,-w2, w1],
                              [-w3,2/h, w1, w2],
                              [ w2,-w1,2/h, w3],
                              [-w1,-w2,-w3,2/h]])
        quat_pred = A_k @ quat_prev
        B_k       = getB(w1,w2,w3)
        P_pred    = B_k @ P_prev @ B_k.T + Q

        # Correction step
        K         = P_pred @ np.linalg.inv(P_pred + R)  # Kalman gain
        quat_hat  = quat_pred + K @ (quat_meas - quat_pred)
        quat_hat /= np.linalg.norm(quat_hat)            # normalize
        P_prev    = (np.eye(4) - K) @ P_pred
        quat_prev = quat_hat

        # Output filtered orientation in degrees
        phi, theta, psi = np.degrees(quat2euler(quat_hat))
        print(f"Roll={phi:6.2f}°  Pitch={theta:6.2f}°  Yaw={psi:6.2f}°")

except KeyboardInterrupt:
    print("Stopping…")
finally:
    ser.close()
