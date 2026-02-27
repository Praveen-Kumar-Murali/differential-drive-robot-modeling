import numpy as np
import matplotlib.pyplot as plt
from DifferentialDriveSimulatedRobot import DifferentialDriveSimulatedRobot

# Initial robot state (6x1) [x, y, theta, vx, vy, omega]
xs0 = np.zeros((6, 1))

# Create robot
robot = DifferentialDriveSimulatedRobot(xs0)

# Simulation parameters
dt = 0.1
steps = 500
robot.dt = dt  # ensure robot uses this dt
robot.visualizationInterval = 10

# --------------- Circular trajectory ---------------
print("Simulating circular trajectory...")
u_circular = 0.5    # forward speed (m/s)
r_circular = 0.2    # angular speed (rad/s) -> controls radius

xsk_1 = xs0.copy()
robot.xTraj, robot.yTraj = [], []

for k in range(steps):
    usk = np.array([u_circular, r_circular])  # [forward, angular]
    xsk = robot.fs(xsk_1, usk)
    xsk_1 = xsk

plt.figure()
plt.title("Circular trajectory")
plt.plot(robot.xTraj, robot.yTraj, 'orange', label='Robot path')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.savefig('figures/circular_trajectory.png')
plt.close()


# --------------- Figure-eight trajectory (FINAL REVISION) ---------------


# Initial robot state (6x1)
xs0 = np.zeros((6, 1))
robot = DifferentialDriveSimulatedRobot(xs0)

# Simulation parameters
steps = 2000        # increase steps for higher resolution
A = 5.0             # amplitude of the figure-eight
dt = 0.05           # smaller dt for smoother path
robot.dt = dt

# Parametric single-8 trajectory
t = np.linspace(0, 2*np.pi, steps)
x_path = A * np.sin(t)
y_path = A * np.sin(t) * np.cos(t)

# Compute velocities
dx = np.gradient(x_path, dt)
dy = np.gradient(y_path, dt)
theta = np.arctan2(dy, dx)
dtheta = np.gradient(theta, dt)
u = np.sqrt(dx**2 + dy**2)       # linear velocity
r = dtheta                        # angular velocity

# Reset robot trajectory
xsk_1 = xs0.copy()
robot.xTraj, robot.yTraj = [], []

for k in range(steps):
    # For the last step, force the robot to exactly reach the last point
    if k == steps-1:
        final_dx = x_path[-1] - robot.xsk[0,0]
        final_dy = y_path[-1] - robot.xsk[1,0]
        final_theta = np.arctan2(final_dy, final_dx)
        final_u = np.sqrt(final_dx**2 + final_dy**2) / dt
        final_r = (final_theta - robot.xsk[2,0]) / dt
        usk = np.array([final_u, final_r])
    else:
        usk = np.array([u[k], r[k]])

    xsk = robot.fs(xsk_1, usk)
    xsk_1 = xsk

plt.figure()
plt.title("Single figure-eight trajectory (completed)")
plt.plot(robot.xTraj, robot.yTraj, 'blue', label='Robot path')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.savefig('figures/figure_eight_trajectory.png')
plt.close()
