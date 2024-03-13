import numpy as np
import matplotlib.pyplot as plt

rocket_landing_ref_data = np.genfromtxt("../build/rocket_landing_ref_data.csv", delimiter=",")
rocket_landing_traj_data = np.genfromtxt("../build/rocket_landing_traj_data.csv", delimiter=",")
rocket_landing_control_data = np.genfromtxt("../build/rocket_landing_control_data.csv", delimiter=",")

x_ref = rocket_landing_ref_data[:,0]
y_ref = rocket_landing_ref_data[:,1]
z_ref = rocket_landing_ref_data[:,2]

x_traj = rocket_landing_traj_data[:,0]
y_traj = rocket_landing_traj_data[:,1]
z_traj = rocket_landing_traj_data[:,2]

x_control = rocket_landing_control_data[:,0]
y_control = rocket_landing_control_data[:,1]
z_control = rocket_landing_control_data[:,2]

# fig = plt.figure(figsize=plt.figaspect(2.0))
fig = plt.figure(figsize=(6.0, 9.0))
fig.suptitle("rocket landing")

ax1 = fig.add_subplot(2, 1, 1, projection='3d')
ax1.plot(x_ref, y_ref, z_ref)
ax1.plot(x_traj, y_traj, z_traj)
ax1.set_title("xyz")
ax1.set_xlabel("x")
ax1.set_ylabel("y")
ax1.set_zlabel("z")

ax2 = fig.add_subplot(2, 1, 2)
ax2.plot(x_control, label="u_x")
ax2.plot(y_control, label="u_y")
ax2.plot(z_control, label="u_z")
ax2.legend()
ax2.grid()
ax2.set_ylim([-20,120])
ax2.set_title("controls")
ax2.set_xlabel("t")
ax2.set_ylabel("control")

plt.show()