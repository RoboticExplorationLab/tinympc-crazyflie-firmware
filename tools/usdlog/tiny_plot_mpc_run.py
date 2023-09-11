# -*- coding: utf-8 -*-
"""
example on how to plot decoded sensor data from crazyflie
@author: jsschell
"""
import cfusdlog
import matplotlib.pyplot as plt
import re
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("filename")
args = parser.parse_args()

# decode binary log data
logData_all = cfusdlog.decode(args.filename)

logDataFreq = logData_all['frequency']

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
    
# let's see which keys exist in current data set
keys = ""
for k, v in logData_all.items():
    keys += k

subplot_rows = 7
plotCurrent = 1

trajNames_x = ['h_x_0', 'h_x_1', 'h_x_2', 'h_x_3', 'h_x_4', 'h_x_5', 'h_x_6', 'h_x_7', 'h_x_8', 'h_x_9', 'h_x_10', 'h_x_11', 'h_x_12', 'h_x_13', 'h_x_14']
trajNames_y = ['h_y_0', 'h_y_1', 'h_y_2', 'h_y_3', 'h_y_4', 'h_y_5', 'h_y_6', 'h_y_7', 'h_y_8', 'h_y_9', 'h_y_10', 'h_y_11', 'h_y_12', 'h_y_13', 'h_y_14']
trajNames_z = ['h_z_0', 'h_z_1', 'h_z_2', 'h_z_3', 'h_z_4', 'h_z_5', 'h_z_6', 'h_z_7', 'h_z_8', 'h_z_9', 'h_z_10', 'h_z_11', 'h_z_12', 'h_z_13', 'h_z_14']

# new figure
plt.figure(args.filename)

plt.subplot(subplot_rows,1, plotCurrent)
plotCurrent += 1

for name in trajNames_x:
    plt.plot(logData_all['timestamp'], logData_all[name], '-')

plt.xlabel('timestamp [ms]')
plt.ylabel('X position [m]')
plt.legend()
plt.grid()




plt.subplot(subplot_rows,1, plotCurrent)
plotCurrent += 1

for name in trajNames_y:
    plt.plot(logData_all['timestamp'], logData_all[name], '-')

plt.xlabel('timestamp [ms]')
plt.ylabel('Y position [m]')
plt.legend()
plt.grid()




plt.subplot(subplot_rows,1, plotCurrent)
plotCurrent += 1

for name in trajNames_z:
    plt.plot(logData_all['timestamp'], logData_all[name], '-')

plt.xlabel('timestamp [ms]')
plt.ylabel('Z position [m]')
plt.legend()
plt.grid()



# plt.subplot(subplot_rows,1, plotCurrent)
# plotCurrent += 1

# plt.plot(logData_all['timestamp'], logData_all['stabilizer.x'], '-')
# plt.plot(logData_all['timestamp'], logData_all['stabilizer.y'], '-')
# plt.plot(logData_all['timestamp'], logData_all['stabilizer.z'], '-')
# plt.xlabel('timestamp [ms]')
# plt.ylabel('Cache level')
# plt.grid()



plt.subplot(subplot_rows,1, plotCurrent)
plotCurrent += 1

plt.scatter(logData_all['timestamp'], logData_all['mpcTime'], s=.8)
plt.xlabel('timestamp [ms]')
plt.ylabel('Solve time [us]')
plt.grid()




plt.subplot(subplot_rows,1, plotCurrent)
plotCurrent += 1

plt.scatter(logData_all['timestamp'], logData_all['iters'], s=.8)
plt.xlabel('timestamp [ms]')
plt.ylabel('ADMM iters')
plt.grid()



plt.subplot(subplot_rows,1, plotCurrent)
plotCurrent += 1

plt.plot(logData_all['timestamp'], logData_all['primal_residual'], label='primal residual')
plt.plot(logData_all['timestamp'], logData_all['dual_residual'], label='dual residual')
plt.xlabel('timestamp [ms]')
plt.ylabel('residual')
plt.grid()


# plt.subplot(subplot_rows,1, plotCurrent)
# plotCurrent += 1

# plt.plot(logData_all['timestamp'], logData_all['cache_level'], '-')
# plt.xlabel('timestamp [ms]')
# plt.ylabel('Cache level')
# plt.grid()



# plt.subplot(subplot_rows,1, plotCurrent)
# plotCurrent += 1

# plt.plot(logResiduals['timestamp'], logResiduals['prim_resid_state'], label='primal state residual')
# plt.plot(logResiduals['timestamp'], logResiduals['prim_resid_input'], label='primal input residual')
# plt.plot(logResiduals['timestamp'], logResiduals['dual_resid_state'], label='dual state residual')
# plt.plot(logResiduals['timestamp'], logResiduals['dual_resid_input'], label='dual input residual')
# plt.xlabel('timestamp [ms]')
# plt.ylabel('residual')
# plt.grid()


# plt.plot(logDataFixed['timestamp'], logDataFixed['tinympc.initial_velocity'], '-', label="initial z vel")
# # plt.axhline(0, linestyle='--', color='r')
# plt.xlabel('timestamp [ms]')
# plt.ylabel('Velocity [m/s]')
# plt.grid()


plt.legend(loc="upper right")

plt.show()
