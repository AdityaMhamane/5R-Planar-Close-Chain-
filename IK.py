import mujoco
import mujoco.viewer
import math
import time

# Load the model from your XML file
model = mujoco.MjModel.from_xml_path('5RPlanar.xml')
data = mujoco.MjData(model)

def IK(x, y):
    l0 = 0.050  # 50 mm - Distance of actuated joint from origin
    l1 = 0.100  # Actuated link length
    l2 = 0.185  # Passive link length

    # In this model, the arm extends in the -Y direction.
    if y > 0:
        y = -y

    # Distances from left motor (-l0, 0) and right motor (+l0, 0) to end-effector (x, y)
    d1 = math.hypot(x + l0, y)
    d2 = math.hypot(x - l0, y)

    # Check if target is within reachable workspace
    if d1 > (l1 + l2) or d1 < abs(l1 - l2) or d2 > (l1 + l2) or d2 < abs(l1 - l2):
        print(f"Target ({x}, {y}) is outside reachable workspace!")
        return None, None

    # Law of Cosines for the two kinematic chains
    alpha1 = math.acos((l1**2 + d1**2 - l2**2) / (2 * l1 * d1))
    alpha2 = math.acos((l1**2 + d2**2 - l2**2) / (2 * l1 * d2))

    # Base-to-target orientation angles
    phi1 = math.atan2(y, x + l0)
    phi2 = math.atan2(y, x - l0)

    # Actuated joint angles (joint_actA and joint_actB)
    qA = math.pi - phi1 + alpha1
    qB = -phi2 - alpha2

    # Wrap angles to [-pi, pi]
    qA = (qA + math.pi) % (2 * math.pi) - math.pi
    qB = (qB + math.pi) % (2 * math.pi) - math.pi

    return qA, qB

def Trajectory(q0, q1, t, t0, t1):
    """
    Joint space trajectory using a Trapezoidal Velocity Profile:
    - Accelerates with 'acc' for t0 seconds
    - Cruises at 'maxVel' for t1 seconds
    - Retards with '-acc' for t0 seconds
    Total duration T = 2 * t0 + t1
    """
    T = 2 * t0 + t1
    if t <= 0:
        return q0
    if t >= T:
        return q1

    maxVel = (q1 - q0) / (t0 + t1)
    acc = maxVel / t0

    if t <= t0:
        return q0 + 0.5 * acc * t**2
    elif t <= t0 + t1:
        return q0 + 0.5 * acc * t0**2 + maxVel * (t - t0)
    else:
        return q1 - 0.5 * acc * (T - t)**2

# Desired target position of end-effector
target_x, target_y = 0.0, -0.250
q_target_A, q_target_B = IK(target_x, target_y)

# Initial joint angles
q0_A = data.qpos[0]
q0_B = data.qpos[2]

# Timing parameters for trapezoidal profile:
t0 = 1.0  # Acceleration and deceleration duration (s)
t1 = 1.0  # Constant velocity cruising duration (s)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # Update actuated joint controls along the trapezoidal profile
        if q_target_A is not None and q_target_B is not None:
            data.ctrl[0] = Trajectory(q0_A, q_target_A, data.time, t0, t1)
            data.ctrl[1] = Trajectory(q0_B, q_target_B, data.time, t0, t1)

        mujoco.mj_step(model, data)

        # Sync the viewer with the updated simulation data
        viewer.sync()

        # Keep simulation in real-time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
