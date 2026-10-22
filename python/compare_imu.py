
import serial
import numpy as np
from scipy.linalg import expm
from vpython import (
    canvas, box, arrow, label,
    vector, color, rate
)

PORT = "COM3"
BAUD = 115200
G = 9.81
G_UP = np.array([0.0, 0.0, G])

# =========================================
# 1. ROTATION MATHEMATICS
# =========================================

def skew(w):
    x, y, z = w
    return np.array([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]
    ])


def rpy_to_matrix(roll, pitch, yaw=0.0):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp,   cp*sr,          cp*cr]
    ])


def gyro_integrate(R, gyro, bias, dt):
    omega = gyro - bias
    return R @ expm(skew(omega * dt))


def accel_orientation(a):
    # Gravity-only orientation.
    # No yaw information is available.
    roll = np.arctan2(a[1], a[2])
    pitch = np.arctan2(
        -a[0],
        np.hypot(a[1], a[2])
    )

    return rpy_to_matrix(roll, pitch, 0.0)


# =========================================
# 2. ESKF
# =========================================

def ESKF(R_prev, b_prev, P_prev,
         w_m, a, Covg, Cova, dt):

    I3 = np.eye(3)
    I6 = np.eye(6)

    # Nominal prediction
    w_hat = w_m - b_prev

    R_pred = R_prev @ expm(skew(w_hat * dt))
    b_pred = b_prev.copy()

    # Error-state Jacobian
    F = np.zeros((6, 6))
    F[:3, :3] = I3 - skew(w_hat) * dt
    F[:3, 3:] = -I3 * dt
    F[3:, 3:] = I3

    # Process noise: no bias random walk
    Q = np.zeros((6, 6))
    Q[:3, :3] = Covg * dt**2

    # Covariance prediction
    P_pred = F @ P_prev @ F.T + Q

    # Accelerometer prediction and innovation
    a_hat = R_pred.T @ G_UP
    r = a - a_hat

    # Accelerometer Jacobian
    H = np.zeros((3, 6))
    H[:, :3] = skew(a_hat)

    # Kalman gain
    S = H @ P_pred @ H.T + Cova
    K = np.linalg.solve(S, H @ P_pred).T

    # Estimated error correction
    dx = K @ r
    dtheta = dx[:3]
    db = dx[3:]

    # Inject correction
    R_post = R_pred @ expm(skew(dtheta))
    b_post = b_pred + db

    # Joseph covariance update
    A = I6 - K @ H

    P_corr = (
        A @ P_pred @ A.T
        + K @ Cova @ K.T
    )

    # Right-error covariance reset
    G_reset = I6.copy()
    G_reset[:3, :3] = (
        I3 - 0.5 * skew(dtheta)
    )

    P_post = G_reset @ P_corr @ G_reset.T
    P_post = 0.5 * (P_post + P_post.T)

    return R_post, b_post, P_post


# =========================================
# 3. READ IMU LINE
# =========================================

def parse_imu(line):
    parts = line.strip().split(",")

    if len(parts) != 8 or parts[0] != "IMU":
        return None

    try:
        t = int(parts[1])
        a = np.array(
            [float(v) for v in parts[2:5]]
        )
        w = np.array(
            [float(v) for v in parts[5:8]]
        )
    except ValueError:
        return None

    if not np.all(np.isfinite(a)):
        return None
    if not np.all(np.isfinite(w)):
        return None

    return t, a, w


# =========================================
# 4. SENSOR CALIBRATION
# =========================================

def calibrate_imu(esp, N=1000):
    print("Keep MPU6050 FLAT and STATIONARY")
    print(f"Collecting {N} samples...")

    acc_samples = []
    gyro_samples = []

    while len(gyro_samples) < N:
        line = esp.readline().decode(
            errors="ignore"
        )

        sample = parse_imu(line)

        if sample is None:
            continue

        _, a, w = sample

        acc_samples.append(a)
        gyro_samples.append(w)

    acc_samples = np.asarray(acc_samples)
    gyro_samples = np.asarray(gyro_samples)

    # Stationary gyro mean = initial bias
    b_gyro = np.mean(gyro_samples, axis=0)

    # One-position offset estimate:
    # ONLY valid if sensor is level, +Z up
    b_accel = (
        np.mean(acc_samples, axis=0) - G_UP
    )

    # Measured per-sample covariances
    Covg = np.cov(
        gyro_samples, rowvar=False
    )
    Cova = np.cov(
        acc_samples, rowvar=False
    )

    # Add quantization variance to prevent
    # a singular measurement covariance
    accel_lsb = 9.80665 / 16384.0
    Cova += np.eye(3) * accel_lsb**2 / 12.0

    # Bias mean covariance, assuming
    # independent calibration samples
    Cov_bias_initial = Covg / N

    # Initial tilt covariance from
    # uncertainty of averaged acceleration
    J = np.array([
        [0.0, 1.0/G, 0.0],
        [-1.0/G, 0.0, 0.0]
    ])

    Cov_tilt = J @ (Cova / N) @ J.T

    P0 = np.zeros((6, 6))

    # Yaw is defined as zero at startup
    # relative to our chosen reference frame.
    # It is not an absolute compass heading.

    print("\nGyro bias:")
    print(b_gyro)

    print("\nGyro covariance:")
    print(Covg)

    print("\nAccel covariance:")
    print(Cova)

    print("\nCalibration complete!")

    return b_gyro, b_accel, Covg, Cova, P0


# =========================================
# 5. VPYTHON VISUALIZATION
# =========================================

def np_vec(v):
    return vector(
        float(v[0]),
        float(v[1]),
        float(v[2])
    )


def make_viewer(name, x_position, board_color):
    center = vector(x_position, 0, 0)

    # Virtual MPU6050
    board = box(
        pos=center,
        size=vector(1.5, 0.9, 0.07),
        axis=vector(1, 0, 0),
        up=vector(0, 1, 0),
        color=board_color,
        opacity=0.8
    )

    # Label above each sensor
    title = label(
        pos=center + vector(0, 0, 1.6),
        text=name,
        height=17,
        box=False,
        opacity=0,
        color=color.white
    )

    # Fixed world axes for reference
    for axis, c in [
        (vector(0.8, 0, 0), color.red),
        (vector(0, 0.8, 0), color.green),
        (vector(0, 0, 0.8), color.blue)
    ]:
        arrow(
            pos=center,
            axis=axis,
            color=c,
            opacity=0.2,
            shaftwidth=0.025
        )

    # Moving body axes
    axes = [
        arrow(pos=center, color=color.red,
              shaftwidth=0.045),
        arrow(pos=center, color=color.green,
              shaftwidth=0.045),
        arrow(pos=center, color=color.blue,
              shaftwidth=0.045)
    ]

    return board, axes


def draw_orientation(viewer, R):
    board, axes = viewer

    x = np_vec(R[:, 0])
    y = np_vec(R[:, 1])
    z = np_vec(R[:, 2])

    board.axis = x
    board.up = y

    axes[0].axis = 1.2 * x
    axes[1].axis = 1.2 * y
    axes[2].axis = 1.2 * z


def create_visualization():
    scene = canvas(
        title="MPU6050: Gyro vs Accel vs ESKF",
        width=1200,
        height=650,
        background=vector(0.08, 0.08, 0.1)
    )

    scene.range = 6.0
    scene.center = vector(0, 0, 0)

    # World Z points upward
    scene.up = vector(0, 0, 1)

    # World Y points into the screen.
    # Slightly elevated view of flat boards.
    scene.forward = vector(0, 1, -0.65)

    gyro_view = make_viewer(
        "GYRO ONLY", -3.4, color.orange
    )

    accel_view = make_viewer(
        "ACCEL ONLY", 0, color.green
    )

    eskf_view = make_viewer(
        "ESKF", 3.4, color.cyan
    )

    return gyro_view, accel_view, eskf_view


# =========================================
# 6. MAIN PROGRAM
# =========================================

R_gyro = np.eye(3)
R_accel = np.eye(3)
R_eskf = np.eye(3)

prev_t = None

with serial.Serial(PORT, BAUD, timeout=1) as esp:

    # Calibrate from stationary measurements
    b0, b_accel, Covg, Cova, Pk = calibrate_imu(
        esp, N=1000
    )

    print(Covg, Cova)
    # ESKF maintains its own evolving bias
    bk = b0.copy()

    # Create three side-by-side viewers
    gyro_view, accel_view, eskf_view = (
        create_visualization()
    )

    draw_orientation(gyro_view, R_gyro)
    draw_orientation(accel_view, R_accel)
    draw_orientation(eskf_view, R_eskf)

    # Discard queued calibration-era samples
    esp.reset_input_buffer()
    esp.timeout = 0

    pending = b""

    print("\nReceiving IMU measurements...")
    print("Three orientation estimates running.")

    try:
        while True:
            rate(60)

            # Read all currently available USB data
            n = esp.in_waiting

            if n:
                pending += esp.read(n)

            # Process all complete measurements
            while b"\n" in pending:
                raw, pending = pending.split(
                    b"\n", 1
                )

                line = raw.decode(
                    errors="ignore"
                )

                sample = parse_imu(line)

                if sample is None:
                    continue

                t, a_raw, wg = sample

                if prev_t is None:
                    prev_t = t
                    continue

                # Correct microsecond timestamp wrap
                dt_us = (
                    t - prev_t
                ) & 0xFFFFFFFF

                prev_t = t
                dt = dt_us * 1e-6

                if not 0 < dt < 0.2:
                    continue

                # Calibrated accelerometer vector
                a = a_raw - b_accel

                # -----------------------------
                # A. GYROSCOPE ONLY
                # -----------------------------
                R_gyro = gyro_integrate(
                    R_gyro, wg, b0, dt
                )

                # -----------------------------
                # B. ACCELEROMETER ONLY
                # -----------------------------
                if np.linalg.norm(a) > 1e-6:
                    R_accel = accel_orientation(a)

                # -----------------------------
                # C. ESKF FUSION
                # -----------------------------
                d = abs(np.linalg.norm(a) - 9.81)

                Cova_adaptive = Cova + d**2 * np.eye(3)

                R_eskf, bk, Pk = ESKF(
                    R_eskf, bk, Pk,
                    wg, a, Covg, Cova_adaptive, dt
                )
            # Render together at up to 60 FPS
            draw_orientation(gyro_view, R_gyro)
            draw_orientation(accel_view, R_accel)
            draw_orientation(eskf_view, R_eskf)

    except KeyboardInterrupt:
        print("Stopped.")
