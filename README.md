# Research-OSU-IMU-Dev
Intertial Measurment Unit Development using the Kalman Fillter

## Overview
- In drone systems, **Inertial Measurement Units (IMUs)** are critical for **position and attitude determination** (PAD), especially in GPS-denied environments.
- This project aims to design a **lightweight, low-cost IMU** capable of providing reliable orientation and position estimates for **indoor drones**.
- A **Kalman Filter** is used to estimate navigation and attitude parameters: **pitch**, **roll**, **yaw**, and **x, y, z** positions.
- The IMU is built around the **BNO055** sensor, which provides 9 readings: 3-axis linear acceleration, 3-axis angular velocity, and 3-axis magnetometer data.
- My role focuses on the **software development and filtering logic**. I modeled the Kalman Filter in **MATLAB and Simulink**, and am currently porting the system to **Python** for deployment on a custom PCB via USB data streaming.
- This project builds on my background in **calculus**, **object-oriented programming**, and experience with **embedded systems** like the Raspberry Pi and FPV drone hardware.

## A deeper dive into the Kalman filter
- The Kalman Filter is a recursive algorithm that blocks of noisy signal data and converges to the true value
- The math relies on the system's abiloty to measure and resuew the data
- In this case, we use the linear accelrations, angular velocities, and magnometer data to find accurate roll, pitch, and yaw angles of the BNO-055
  
1) Set theta, phi, psi = 0 as inital conditions and convert to into quarternton coordinates using the formula below. This is done ... (not sure why)
   ![Euler2Quat Conversion](quatFormula.png)
