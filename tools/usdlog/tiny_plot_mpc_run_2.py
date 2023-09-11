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

logHorizonData_x_1 = logData_all['horizon_x_part1']
logHorizonData_x_2 = logData_all['horizon_x_part2']
logHorizonData_x_3 = logData_all['horizon_x_part3']
# logHorizonData_x_4 = logData_all['horizon_x_part4']
logHorizonData_y_1 = logData_all['horizon_y_part1']
logHorizonData_y_2 = logData_all['horizon_y_part2']
logHorizonData_y_3 = logData_all['horizon_y_part3']
# logHorizonData_y_4 = logData_all['horizon_y_part4']
logHorizonData_z_1 = logData_all['horizon_z_part1']
logHorizonData_z_2 = logData_all['horizon_z_part2']
logHorizonData_z_3 = logData_all['horizon_z_part3']
# logHorizonData_z_4 = logData_all['horizon_z_part4']

logProblemData = logData_all['problem_data_event']
logResiduals = logData_all['problem_residuals_event']

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
    
# # let's see which keys exist in current data set
# keys = ""
# for k, v in logData.items():
#     keys += k


subplot_rows = 7
# current plot for simple subplot usage
plotCurrent = 1

trajNames_1 = ['h0', 'h1' ,'h2', 'h3', 'h4']
trajNames_2 = ['h5', 'h6', 'h7', 'h8', 'h9']
trajNames_3 = ['h10', 'h11', 'h12', 'h13', 'h14']
trajNames_4 = ['h15', 'h16', 'h17', 'h18', 'h19']

# new figure
plt.figure(args.filename)

plt.subplot(subplot_rows,1, plotCurrent)
plotCurrent += 1

for name in trajNames_1:
    plt.plot(logHorizonData_x_1['timestamp'], logHorizonData_x_1[name], '-')
for name in trajNames_2:
    plt.plot(logHorizonData_x_2['timestamp'], logHorizonData_x_2[name], '-')
for name in trajNames_3:
    plt.plot(logHorizonData_x_3['timestamp'], logHorizonData_x_3[name], '-')
# for name in trajNames_4:
#     plt.plot(logHorizonData_x_4['timestamp'], logHorizonData_x_4[name], '-')

plt.title('MPC Trajectory (x)')
plt.xlabel('timestamp [ms]')
plt.ylabel('Position [m]')
plt.legend()
plt.grid()

plt.subplot(subplot_rows,1, plotCurrent)
plotCurrent += 1

for name in trajNames_1:
    plt.plot(logHorizonData_y_1['timestamp'], logHorizonData_y_1[name], '-')
for name in trajNames_2:
    plt.plot(logHorizonData_y_2['timestamp'], logHorizonData_y_2[name], '-')
for name in trajNames_3:
    plt.plot(logHorizonData_y_3['timestamp'], logHorizonData_y_3[name], '-')
# for name in trajNames_4:
#     plt.plot(logHorizonData_y_4['timestamp'], logHorizonData_y_4[name], '-')

plt.title('MPC Trajectory (y)')
plt.xlabel('timestamp [ms]')
plt.ylabel('Position [m]')
plt.legend()
plt.grid()

plt.subplot(subplot_rows,1, plotCurrent)
plotCurrent += 1

for name in trajNames_1:
    plt.plot(logHorizonData_z_1['timestamp'], logHorizonData_z_1[name], '-')
for name in trajNames_2:
    plt.plot(logHorizonData_z_2['timestamp'], logHorizonData_z_2[name], '-')
for name in trajNames_3:
    plt.plot(logHorizonData_z_3['timestamp'], logHorizonData_z_3[name], '-')
# for name in trajNames_4:
#     plt.plot(logHorizonData_z_4['timestamp'], logHorizonData_z_4[name], '-')

plt.title('MPC Trajectory (z)')
plt.xlabel('timestamp [ms]')
plt.ylabel('Position [m]')
plt.legend()
plt.grid()


plt.subplot(subplot_rows,1, plotCurrent)
plotCurrent += 1

plt.scatter(logProblemData['timestamp'], logProblemData['solvetime_us'], s=.8)
plt.xlabel('timestamp [ms]')
plt.ylabel('Solve time [us]')
plt.grid()

plt.subplot(subplot_rows,1, plotCurrent)
plotCurrent += 1

plt.scatter(logProblemData['timestamp'], logProblemData['iters'], s=.8)
plt.xlabel('timestamp [ms]')
plt.ylabel('ADMM iters')
plt.grid()


plt.subplot(subplot_rows,1, plotCurrent)
plotCurrent += 1

plt.plot(logProblemData['timestamp'], logProblemData['cache_level'], '-')
plt.xlabel('timestamp [ms]')
plt.ylabel('Cache level')
plt.grid()


plt.subplot(subplot_rows,1, plotCurrent)
plotCurrent += 1

plt.plot(logResiduals['timestamp'], logResiduals['prim_resid_state'], label='primal state residual')
plt.plot(logResiduals['timestamp'], logResiduals['prim_resid_input'], label='primal input residual')
plt.plot(logResiduals['timestamp'], logResiduals['dual_resid_state'], label='dual state residual')
plt.plot(logResiduals['timestamp'], logResiduals['dual_resid_input'], label='dual input residual')
plt.xlabel('timestamp [ms]')
plt.ylabel('residual')
plt.grid()


# plt.plot(logDataFixed['timestamp'], logDataFixed['tinympc.initial_velocity'], '-', label="initial z vel")
# # plt.axhline(0, linestyle='--', color='r')
# plt.xlabel('timestamp [ms]')
# plt.ylabel('Velocity [m/s]')
# plt.grid()




# if re.search('stabilizer', keys):
#     plotCurrent += 1
#     plt.subplot(plotRows, plotCols, plotCurrent)
#     plt.plot(logData['timestamp'], logData['stabilizer.intToOut'], '-')
#     plt.xlabel('timestamp [ms]')
#     plt.ylabel('Time [us]')
 
# if re.search('stateEstimate', keys):
#     plotCurrent += 1
#     plt.subplot(plotRows, plotCols, plotCurrent)
#     plt.plot(logData['timestamp'], logData['stateEstimate.x'], '-', label='X')
#     plt.plot(logData['timestamp'], logData['stateEstimate.y'], '-', label='Y')
#     plt.plot(logData['timestamp'], logData['stateEstimate.z'], '-', label='Z')
#     plt.xlabel('timestamp [ms]')
#     plt.ylabel('Postion [m]')

# if re.search('stateEstimate', keys):
#     plotCurrent += 1
#     plt.subplot(plotRows, plotCols, plotCurrent)
#     plt.plot(logData['timestamp'], logData['stateEstimate.qx'], '-', label='X')
#     plt.plot(logData['timestamp'], logData['stateEstimate.qy'], '-', label='Y')
#     plt.plot(logData['timestamp'], logData['stateEstimate.qz'], '-', label='Z')
#     plt.plot(logData['timestamp'], logData['stateEstimate.qw'], '-', label='Z')
#     plt.xlabel('timestamp [ms]')
#     plt.ylabel('Quaternion')


plt.legend(loc="upper right")

plt.show()
