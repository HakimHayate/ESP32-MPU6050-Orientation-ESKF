# Real-Time IMU Orientation Estimation Using an Error-State Kalman Filter

### ESP32 + MPU6050 + Python + VPython

## Overview

This project implements and compares three methods of estimating 3D orientation using an MPU6050 inertial measurement unit (IMU):

1. **Gyroscope-only integration**
2. **Accelerometer-only tilt estimation**
3. **Error-State Extended Kalman Filter (ESKF)** combining both sensors

An ESP32 reads the MPU6050 measurements and transmits them to a Windows PC over USB. Python performs the orientation estimation and displays three virtual IMU boards side by side using VPython.

The main goal is to understand sensor fusion through both mathematical derivation and real hardware experiments.

## Experimental Visualization

The visualization shows three orientation estimates simultaneously:

| Method | Strength | Limitation |
|---|---|---|
| Gyroscope only | Smooth, tracks rapid rotations | Accumulates orientation drift |
| Accelerometer only | Provides an absolute gravity/tilt reference | No yaw information; disturbed by linear acceleration |
| ESKF | Uses gyro prediction and accelerometer correction | Requires calibration and covariance tuning; yaw remains unobservable |

**Experimental observation:** After rapid movements, the gyro-only estimate can remain tilted when the physical board returns to horizontal. The accelerometer-only estimate recovers its tilt reference, while the ESKF can correct tilt drift using gravity. Some trials still show residual ESKF tilt, which is being investigated.

*Add a screenshot of the three-model visualization here.*

## Hardware

- ESP32 development board
- MPU6050 / GY-521 accelerometer and gyroscope
- USB data cable
- Jumper wires
- Windows PC

### Wiring

For the standard ESP32 DevKit:

| MPU6050 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

The MPU6050 communicates through I²C using address `0x68`.

### Sensor configuration

| Setting | Value |
|---|---|
| Gyroscope full scale | ±1000°/s |
| Accelerometer full scale | ±2g |
| Sampling frequency | Approximately 100 Hz |
| Serial baud rate | 115200 |
| Accelerometer output | m/s² |
| Gyroscope output | rad/s |
| Timestamp | ESP32 microseconds |

The wider gyroscope range was introduced after observing orientation errors during fast rotations.

## System Architecture

```text
     MPU6050
   Gyro + Accel
         |
         | I2C
         v
       ESP32
         |
         | USB Serial
         | timestamp, ax, ay, az, gx, gy, gz
         v
       Python
         |
         +---- Gyro integration
         |
         +---- Accelerometer tilt
         |
         +---- Rotation-matrix ESKF
                      |
                      v
                3D Visualization
                    VPython
```

The ESP32 is responsible for data acquisition. All filtering and visualization take place on the computer.

## Mathematical Model

### 1. Rotation representation

The orientation is represented by a rotation matrix:

\[
R\in SO(3)
\]

We use the convention:

\[
v_W=Rv_B
\]

where \(R\) transforms body-frame coordinates into world-frame coordinates.

Unlike Euler angles, rotation matrices do not suffer from gimbal lock.

### 2. Gyroscope measurement model

The gyroscope measures:

\[
\omega_m=\omega_{\mathrm{true}}+b_g+n_g
\]

where:

- \(\omega_m\): measured angular velocity
- \(b_g\): gyroscope bias
- \(n_g\): measurement noise

The nominal angular velocity is:

\[
\hat\omega=\omega_m-\hat b_g
\]

### 3. Gyroscope orientation prediction

The rotation dynamics are:

\[
\dot R=R[\omega_B]_\times
\]

The discrete integration is:

\[
\boxed{
\hat R_{k+1}^-=
\hat R_k^+
\exp([(\omega_{m,k}-\hat b_k^+)\Delta t]_\times)
}
\]

The matrix exponential keeps the predicted orientation on \(SO(3)\), apart from numerical roundoff.

### 4. ESKF error state

The filter maintains the nominal orientation and gyroscope bias:

\[
\hat x=(\hat R,\hat b_g)
\]

The six-dimensional local error state is:

\[
\delta x=
\begin{bmatrix}
\delta\theta\\
\delta b_g
\end{bmatrix}
\]

The right-multiplicative orientation error is defined by:

\[
R_{\mathrm{true}}
=\hat R\exp([\delta\theta]_\times)
\]

The continuous-time linearized error dynamics are:

\[
\dot{\delta\theta}
=-[\hat\omega]_\times\delta\theta-\delta b_g-n_g
\]

\[
\dot{\delta b_g}=n_{bg}
\]

### 5. Prediction Jacobian

Using first-order discretization:

\[
\boxed{
F_k=
\begin{bmatrix}
I_3-[\hat\omega_k]_\times\Delta t&-I_3\Delta t\\
0&I_3
\end{bmatrix}
}
\]

The covariance prediction is:

\[
\boxed{P_{k+1}^-=F_kP_k^+F_k^T+Q_k}
\]

In the current simplified implementation, bias random-walk process noise is set to zero.

For per-sample gyro-rate noise covariance \(\Sigma_g\):

\[
Q_k\approx
\begin{bmatrix}
\Sigma_g\Delta t^2&0\\
0&0
\end{bmatrix}
\]

This assumes effectively independent per-sample gyro noise and constant gyro bias. It is a simplified discrete process-noise model.

### 6. Accelerometer measurement model

The accelerometer measures specific force:

\[
a_m=R^T(a_W-g_W)+b_a+n_a
\]

When linear acceleration is negligible:

\[
a_m\approx R^Tg_{\mathrm{up}}+n_a
\]

with:

\[
g_{\mathrm{up}}=
\begin{bmatrix}0\\0\\9.81\end{bmatrix}
\]

The predicted accelerometer measurement is:

\[
\boxed{\hat a=(\hat R^-)^Tg_{\mathrm{up}}}
\]

The innovation is:

\[
\boxed{r=a_m-\hat a}
\]

### 7. Accelerometer measurement Jacobian

Using:

\[
R_{\mathrm{true}}
=\hat R^-\exp([\delta\theta]_\times)
\]

and linearizing around zero orientation error:

\[
r\approx[\hat a]_\times\delta\theta+n_a
\]

Therefore:

\[
\boxed{
H=
\begin{bmatrix}
[\hat a]_\times&0_{3\times3}
\end{bmatrix}
}
\]

The accelerometer Jacobian has rank two for nonzero gravity.

This means that gravity can constrain tilt, but cannot determine absolute yaw.

### 8. Kalman correction

The innovation covariance is:

\[
S=HP^-H^T+V
\]

The Kalman gain is:

\[
K=P^-H^TS^{-1}
\]

The estimated correction is:

\[
\widehat{\delta x}=Kr
\]

Splitting the correction into rotation and bias components:

\[
\widehat{\delta x}=
\begin{bmatrix}
\widehat{\delta\theta}\\
\widehat{\delta b}
\end{bmatrix}
\]

we inject it into the nominal state:

\[
\boxed{
\hat R^+=\hat R^-\exp([\widehat{\delta\theta}]_\times)
}
\]

\[
\boxed{\hat b^+=\hat b^-+\widehat{\delta b}}
\]

The covariance is updated using the Joseph form, followed by the ESKF covariance reset.

## Sensor Calibration

Before filtering, the MPU6050 remains stationary while Python collects approximately 1,000 measurements.

### Initial gyroscope bias

Since the true angular velocity is zero when stationary:

\[
\boxed{
\hat b_{g,0}=\frac1N\sum_{i=1}^{N}\omega_{m,i}
}
\]

### Gyroscope noise covariance

\[
\boxed{
\Sigma_g=\operatorname{Cov}(\omega_{m,1},\ldots,\omega_{m,N})
}
\]

### Accelerometer noise covariance

\[
\boxed{
V_0=\operatorname{Cov}(a_{m,1},\ldots,a_{m,N})
}
\]

These covariances are estimated from the real sensor rather than selected arbitrarily.

The prototype also uses a one-position accelerometer offset estimate, which assumes the sensor is level with its positive Z-axis upward. This is not a substitute for full accelerometer calibration.

**Limitations:** Stationary calibration does not characterize all dynamic errors. The MPU6050's internal low-pass filter can correlate consecutive samples, and gyro bias can change with temperature.

## Gyroscope-Only Estimation

The gyroscope-only method integrates bias-corrected angular velocity:

\[
R_{k+1}=R_k\exp([(\omega_{m,k}-\hat b_{g,0})\Delta t]_\times)
\]

It tracks rotations smoothly but has no external reference to correct accumulated orientation errors.

## Accelerometer-Only Estimation

For the ZYX Euler convention:

\[
R=R_z(\psi)R_y(\theta)R_x(\phi)
\]

assuming negligible linear acceleration:

\[
a_m=g
\begin{bmatrix}
-\sin\theta\\
\cos\theta\sin\phi\\
\cos\theta\cos\phi
\end{bmatrix}
\]

Roll and pitch can be calculated as:

\[
\phi=\operatorname{atan2}(a_y,a_z)
\]

\[
\theta=\operatorname{atan2}
\left(-a_x,\sqrt{a_y^2+a_z^2}\right)
\]

Yaw is fixed at zero in the accelerometer-only visualization because the accelerometer cannot observe it.

The ESKF itself does not need Euler angles for prediction or correction.

## Why Sensor Fusion Works

**During motion:** The gyroscope continues predicting orientation. If substantial linear acceleration is present, accelerometer gravity measurements become less trustworthy.

**When stationary:** The accelerometer provides a reliable gravity reference. The filter compares measured gravity with predicted gravity and uses the innovation to correct tilt errors.

The gyroscope provides short-term orientation changes; the accelerometer provides a long-term tilt reference.

This does not eliminate absolute yaw drift.

## Known Limitations and Ongoing Work

- Absolute yaw is unobservable using only a gyroscope and accelerometer.
- Fast rotations can introduce saturation, timing, scale-factor or integration errors.
- Fast translations can be mistaken for changes in tilt.
- Stationary covariance estimates may underestimate disturbances during movement.
- Setting gyro bias process noise to zero assumes bias remains constant.
- Very small initial covariance values can make the ESKF overconfident.
- Some experiments show slow or incomplete tilt recovery after rapid movements.

Planned improvements include gravity-innovation diagnostics, adaptive accelerometer covariance, more complete bias characterization, calibration under multiple sensor orientations, and quantitative comparison of recovery time and residual tilt error.

## Running the Project

### Python dependencies

Install:

```bash
py -m pip install numpy scipy pyserial vpython
```

### Hardware setup

1. Connect the MPU6050 to the ESP32 using the wiring table above.
2. Upload the Arduino firmware that transmits IMU data over serial.
3. Connect the ESP32 to the PC.
4. Confirm that Windows recognizes its serial port.
5. Close Arduino Serial Monitor so Python can access the port.

### Run the visualization

The Python script reads data from COM3 at 115200 baud.

```bash
py compare_imu.py
```

Keep the MPU6050 flat and stationary during the initial calibration.

After calibration, the program displays the three orientation estimates simultaneously.

## Experimental Tests

| Experiment | Purpose |
|---|---|
| Stationary for several minutes | Measure gyro drift and filter stability |
| Slow 90° rotation | Check axis conventions and integration |
| Fast rotation followed by rest | Investigate gyro drift and ESKF recovery |
| Rotation about vertical Z | Demonstrate unobservable yaw |
| Rapid linear translation | Observe false accelerometer tilt |
| Return to a level surface | Compare residual tilt among the three methods |

## Conclusion

This project demonstrates how rotation-matrix kinematics, real sensor calibration, covariance propagation, and accelerometer measurements can be combined into an Error-State Kalman Filter.

It also illustrates why an IMU estimator must distinguish between gyroscope noise, bias, accelerometer noise and actual linear acceleration.

The system is an experimental implementation intended for learning, testing and further refinement.
