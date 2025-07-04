# Research-OSU-IMU-Dev  
**Inertial Measurement Unit Development Using the Kalman Filter**

---

## 📊 Overview

- In drone systems, **Inertial Measurement Units (IMUs)** are critical for **position and attitude determination** (PAD), especially in GPS-denied environments.
- This project aims to design a **lightweight, low-cost IMU** capable of providing reliable orientation and position estimates for **indoor drones**.
- A **Kalman Filter** is used to estimate navigation and attitude parameters: **pitch**, **roll**, **yaw**, and **x, y, z** positions.
- The IMU is built around the **BNO055** sensor, which provides 9 readings: 3-axis linear acceleration, 3-axis angular velocity, and 3-axis magnetometer data.
- My role focuses on the **software development and filtering logic**. I modeled the Kalman Filter in **MATLAB and Simulink**, and am currently porting the system to **Python** for deployment on a custom PCB via USB data streaming.
- This project builds on my background in **calculus**, **object-oriented programming**, and experience with **embedded systems** like the Raspberry Pi and FPV drone hardware.

---

## 🔍 A Deeper Dive Into the Kalman Filter

The **Kalman Filter** is a recursive algorithm that estimates the true state of a system by combining **sensor data** and a **mathematical model** of the system. It filters out noise and converges to a more accurate estimate over time.

In our case, it’s used to compute precise **roll (𝜙), pitch (𝜃), and yaw (𝜓)** angles using data from the **BNO055**.

---

### 🧭 Sensor Inputs

We use the BNO055's:
- **Accelerometer**: 𝑎ₓ, 𝑎ᵧ, 𝑎_z
- **Gyroscope**: 𝜔ₓ, 𝜔ᵧ, 𝜔_z
- **Magnetometer**: magnetic heading

---

### 📐 Step-by-Step Implementation

1. **Initialize Orientation**
   - Set initial Euler angles: `𝜙 = 0, 𝜃 = 0, 𝜓 = 0`
   - Convert these to **quaternion coordinates** using the equation below:
     
     ![Euler2Quat Conversion](quatFormula.png)
   
     > Quaternions are used instead of Euler angles to avoid singularities (gimbal lock) and provide smooth interpolation of orientation.

---

2. **Propagate the Quaternion State (Prediction Step)**
   - Use the measured angular velocities (𝑝, 𝑞, 𝑟) to update the quaternion:
   ```math
   dq/dt = 0.5 * Ω(𝜔) * q
