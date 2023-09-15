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
# keys = ""
# for k, v in logData.items():
#     keys += k
#     print(k)

#only focus on regular logging
logData = logData['fixedFrequency']

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
    
# number of columns and rows for suplot
plotCols = 1
plotRows = 6

# let's see which keys exists in current data set
keys = ""
for k, v in logData.items():
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
 
if re.search('trajRef', keys):
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['timestamp'], logData['trajRef.x'], '-', label='X')
    plt.plot(logData['timestamp'], logData['trajRef.y'], '-', label='Y')
    plt.plot(logData['timestamp'], logData['trajRef.z'], '-', label='Z')
    plt.xlabel('timestamp [ms]')
    plt.ylabel('reference [m]')
    plt.ylim((-1.2, 1.2))
    # plt.grid()

if re.search('stateEstimate', keys):
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['timestamp'], logData['stateEstimate.vx'], '-', label='X')
    plt.plot(logData['timestamp'], logData['stateEstimate.vy'], '-', label='Y')
    plt.plot(logData['timestamp'], logData['stateEstimate.vz'], '-', label='Z')
    plt.xlabel('timestamp [ms]')
    plt.ylabel('position [m]')

    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['timestamp'], logData['stateEstimate.roll'], '-', label='X')
    plt.plot(logData['timestamp'], logData['stateEstimate.pitch'], '-', label='Y')
    plt.plot(logData['timestamp'], logData['stateEstimate.yaw'], '-', label='Z')
    plt.xlabel('timestamp [ms]')
    plt.ylabel('attitude')

if re.search('motorUncapped', keys):
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['timestamp'], logData['motorUncapped.m1'], '-', label='u1')
    plt.plot(logData['timestamp'], logData['motorUncapped.m2'], '-', label='u2')
    plt.plot(logData['timestamp'], logData['motorUncapped.m3'], '-', label='u3')
    plt.plot(logData['timestamp'], logData['motorUncapped.m4'], '-', label='u4')
    plt.axhline(65535, linestyle='--', color='r')
    plt.xlabel('timestamp [ms]')
    plt.ylabel('control [m]')
    plt.grid()

# if re.search('motorUncapped', keys):
#     plotCurrent += 1
#     plt.subplot(plotRows, plotCols, plotCurrent)
#     plt.plot(logData['timestamp'], logData['x'], '-', label='u1')
#     plt.plot(logData['timestamp'], logData['y'], '-', label='u2')
#     plt.plot(logData['timestamp'], logData['z'], '-', label='u3')
#     plt.xlabel('timestamp [ms]')
#     plt.ylabel('reference [m]')
#     plt.grid()

# plotCurrent += 1
# plt.subplot(plotRows, plotCols, plotCurrent)
# plt.plot(logData['timestamp'], logData_traj_pos['x'], '-', label='u1')
# plt.plot(logData['timestamp'], logData_traj_pos['y'], '-', label='u2')
# plt.plot(logData['timestamp'], logData_traj_pos['z'], '-', label='u3')
# plt.xlabel('timestamp [ms]')
# plt.ylabel('reference [m]')
# plt.grid()

# plotCurrent += 1
# plt.subplot(plotRows, plotCols, plotCurrent)
# plt.plot(logData_solver_stats['timestamp'], logData_solver_stats['iters'], '-', label='iters')
# plt.xlabel('timestamp [ms]')
# plt.ylabel('reference [m]')
# plt.grid()

plt.show()
