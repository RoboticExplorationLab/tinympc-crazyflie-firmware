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
logData = cfusdlog.decode(args.filename)

# let's see which keys exists in current data set
keys = ""
for k, v in logData.items():
    keys += k
    print(k)

#only focus on regular logging
logData_control = logData['control']
logData_traj_ref = logData['traj_ref']
logData_traj_pos = logData['traj_pos']
logData_solver_stats = logData['solver_stats']
logData_ff = logData['fixedFrequency']

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
    
# number of columns and rows for suplot
plotCols = 1
plotRows = 6

# let's see which keys exists in current data set
keys = ""
for k, v in logData_solver_stats.items():
    keys += k
    print(k)
    
# current plot for simple subplot usage
plotCurrent = 0

# new figure
plt.figure(args.filename)

# if re.search('stabilizer', keys):
#     plotCurrent += 1
#     plt.subplot(plotRows, plotCols, plotCurrent)
#     plt.plot(logData['timestamp'], logData['stabilizer.intToOut'], '-')
#     plt.xlabel('timestamp [ms]')
#     plt.ylabel('Time [us]')
 
# if re.search('stateEstimate', keys):
plotCurrent += 1
plt.subplot(plotRows, plotCols, plotCurrent)
# plt.plot(logData['timestamp'], logData['stateEstimate.x'], '-', label='X')
# plt.plot(logData['timestamp'], logData['stateEstimate.y'], '-', label='Y')
plt.plot(logData_ff['timestamp'], logData_ff['stateEstimate.z'], '-', label='Z')
plt.xlabel('timestamp [ms]')
plt.ylabel('postion [m]')
plt.ylim((0, 1.2))
plt.grid()

# if re.search('stateEstimate', keys):
#     plotCurrent += 1
#     plt.subplot(plotRows, plotCols, plotCurrent)
#     plt.plot(logData['timestamp'], logData['stateEstimate.qx'], '-', label='X')
#     plt.plot(logData['timestamp'], logData['stateEstimate.qy'], '-', label='Y')
#     plt.plot(logData['timestamp'], logData['stateEstimate.qz'], '-', label='Z')
#     plt.plot(logData['timestamp'], logData['stateEstimate.qw'], '-', label='Z')
#     plt.xlabel('timestamp [ms]')
#     plt.ylabel('Quaternion')

plotCurrent += 1
plt.subplot(plotRows, plotCols, plotCurrent)
plt.plot(logData_control['timestamp'], logData_control['u1'], '-', label='u1')
plt.plot(logData_control['timestamp'], logData_control['u2'], '-', label='u2')
plt.plot(logData_control['timestamp'], logData_control['u3'], '-', label='u3')
plt.plot(logData_control['timestamp'], logData_control['u4'], '-', label='u4')
plt.xlabel('timestamp [ms]')
plt.ylabel('control [m]')
plt.grid()

plotCurrent += 1
plt.subplot(plotRows, plotCols, plotCurrent)
plt.plot(logData_traj_ref['timestamp'], logData_traj_ref['x'], '-', label='u1')
plt.plot(logData_traj_ref['timestamp'], logData_traj_ref['y'], '-', label='u2')
plt.plot(logData_traj_ref['timestamp'], logData_traj_ref['z'], '-', label='u3')
plt.xlabel('timestamp [ms]')
plt.ylabel('reference [m]')
plt.grid()

plotCurrent += 1
plt.subplot(plotRows, plotCols, plotCurrent)
plt.plot(logData_traj_ref['timestamp'], logData_traj_pos['x'], '-', label='u1')
plt.plot(logData_traj_ref['timestamp'], logData_traj_pos['y'], '-', label='u2')
plt.plot(logData_traj_ref['timestamp'], logData_traj_pos['z'], '-', label='u3')
plt.xlabel('timestamp [ms]')
plt.ylabel('reference [m]')
plt.grid()

plotCurrent += 1
plt.subplot(plotRows, plotCols, plotCurrent)
plt.plot(logData_solver_stats['timestamp'], logData_solver_stats['iters'], '-', label='iters')
plt.xlabel('timestamp [ms]')
plt.ylabel('reference [m]')
plt.grid()

plt.show()
