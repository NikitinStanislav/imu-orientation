# IMU Real-Time Orientation Estimation (DCM + Arduino)

Real-time 3D orientation estimation using an **Arduino Nano R4** and **MPU6050** IMU sensor. Orientation is tracked using a **Direction Cosine Matrix (DCM)** updated at ~100 Hz with gyroscope integration and accelerometer correction. Roll, pitch, and yaw are streamed over serial and visualized live in the **Processing IDE** using a 3D airplane model.

---

## Overview

Most hobby IMU projects rely on libraries like Mahony or Madgwick, which are great but abstract away the math. This project implements orientation tracking from scratch using the DCM approach — a rotation matrix that continuously maps from the sensor's body frame to the world frame.

**Key ideas:**

- Gyroscope data is integrated to propagate the rotation matrix forward in time (first-order integration via the skew-symmetric angular velocity matrix)
- Accelerometer readings provide a gravity-direction reference used to correct gyroscope drift (complementary filter-style feedback via cross-product error)
- Gram-Schmidt orthonormalization is applied each step to keep the matrix a valid rotation matrix despite floating-point accumulation errors
- Euler angles (roll/pitch/yaw) are extracted from the final matrix for output

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Arduino Nano R4 |
| IMU | MPU6050 (I2C) |
| Interface | USB Serial @ 19200 baud |

### Wiring (MPU6050 → Arduino Nano R4)

| MPU6050 Pin | Arduino Pin |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |

---

## Algorithm

### Direction Cosine Matrix (DCM)

The rotation matrix `R` (3×3) encodes the orientation of the sensor body frame relative to the world frame. It is updated each timestep using:

```
R_{k+1} = R_k · (I + Ω · dt)
```

where `Ω` is the skew-symmetric matrix built from the corrected angular velocity vector `ω`.

### Accelerometer Correction

Raw gyroscope integration drifts over time. To counter this, the measured (normalized) acceleration vector is compared against the gravity direction predicted by the current rotation matrix. The cross product of these two vectors produces an error signal that nudges `ω` back toward the true orientation:

```
error = a_measured × g_predicted
ω_corrected = ω_gyro + Kacc · error
```

The correction gain `Kacc = 0.15` balances gyro responsiveness against accelerometer noise.

### Gram-Schmidt Orthonormalization

After each integration step, the rotation matrix is re-orthonormalized using Gram-Schmidt to prevent numerical drift from corrupting the matrix:

1. Normalize the X column vector
2. Remove the X component from Y (make them orthogonal), then normalize Y
3. Compute Z = X × Y

### Euler Angle Extraction (ZYX convention)

```
pitch = -asin(R[2][0])
roll  =  atan2(R[2][1], R[2][2])
yaw   =  atan2(R[1][0], R[0][0])
```

---

## MPU6050 Configuration

| Parameter | Setting |
|---|---|
| Accelerometer range | ±2g |
| Gyroscope range | ±250 °/s |
| DLPF bandwidth | 44 Hz |
| Loop rate | ~100 Hz (10 ms delay) |

---

## Serial Output Format

Data is streamed as plain ASCII over serial at **19200 baud**:

```
<roll>/<pitch>/<yaw>
```

Example:
```
-1.23/4.56/178.90
-1.21/4.60/178.95
```

All values are in **degrees**.

---

## Processing Visualization

The serial output is read by a **Processing IDE** sketch that renders a 3D airplane model rotated in real time according to the incoming roll/pitch/yaw values.

The 3D airplane model used in the visualization is courtesy of **toys_N_joys** on Free3D:

> *Toy Airplane (Cinema 4D / OBJ)* — [free3d.com](https://free3d.com/3d-model/toy-airplane-cinema-4d-obj-280982.html)

---

## Dependencies

### Arduino

Install via the Arduino Library Manager:

- [`Adafruit MPU6050`](https://github.com/adafruit/Adafruit_MPU6050)
- [`Adafruit Unified Sensor`](https://github.com/adafruit/Adafruit_Sensor)

### Processing

- [Processing IDE](https://processing.org/) with the `Serial` library (included by default)

---

## Getting Started

1. Wire the MPU6050 to the Arduino Nano R4 as shown above
2. Install the required Arduino libraries
3. Upload `arduino_code.cpp` to the board
4. Open the Processing sketch and set the correct serial port
5. Run the Processing sketch — the 3D model will track the sensor orientation live

---

## Limitations

- **No magnetometer / yaw drift:** Without a magnetometer, yaw is integrated from gyroscope data only and will drift over time. Roll and pitch remain stable due to accelerometer correction.
- **Linear acceleration sensitivity:** The accelerometer correction assumes the dominant acceleration is gravity. Fast movements will temporarily corrupt the roll/pitch correction signal.
- **First-order integration:** A higher-order integrator (e.g., Runge-Kutta) would improve accuracy at low sample rates, but is unnecessary at ~100 Hz.
