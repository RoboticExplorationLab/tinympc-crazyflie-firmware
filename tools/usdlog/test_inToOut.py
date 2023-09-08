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

# #only focus on regular logging
logDataFixed = logData_all['fixedFrequency']
logData1 = logData_all['horizon_part1']
logData2 = logData_all['horizon_part2']
logData3 = logData_all['horizon_part3']
# logDataXref = logData_all['xref_event']
logDataIters = logData_all['iters_event']
logDataCacheLevel = logData_all['cache_level_event']

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
    
# # let's see which keys exists in current data set
# keys = ""
# for k, v in logData.items():
#     keys += k

# current plot for simple subplot usage
plotCurrent = 0

# new figure
plt.figure(args.filename)

plt.subplot(3,1, 1)

plt.plot(logData1['timestamp'], logData1['h0'], '-', label="z_0")
# plt.plot(logData1['timestamp'], logData1['h1'], '--')#, label="z_1")
plt.plot(logData1['timestamp'], logData1['h2'], '-')#, label="z_2")
# plt.plot(logData1['timestamp'], logData1['h3'], '--')#, label="z_3")
plt.plot(logData1['timestamp'], logData1['h4'], '-')#, label="z_4")

# plt.plot(logData2['timestamp'], logData2['h5'], '--')#, label="z_5")
plt.plot(logData2['timestamp'], logData2['h6'], '-')#, label="z_6")
# plt.plot(logData2['timestamp'], logData2['h7'], '--')#, label="z_7")
plt.plot(logData2['timestamp'], logData2['h8'], '-')#, label="z_8")
# plt.plot(logData2['timestamp'], logData2['h9'], '--')#, label="z_9")

# plt.plot(logData3['timestamp'], logData3['h10'], '--')#, label="z_10")
plt.plot(logData3['timestamp'], logData3['h11'], '-')#, label="z_11")
# plt.plot(logData3['timestamp'], logData3['h12'], '--')#, label="z_12")
# plt.plot(logData3['timestamp'], logData3['h13'], '-')#, label="z_13")
plt.plot(logData3['timestamp'], logData3['h14'], '-', label="z_14")

# plt.plot(logDataXref['timestamp'], logDataXref['xref_z'], '-', label="xref_z")


plt.xlabel('timestamp [ms]')
plt.ylabel('Position [m]')
plt.legend()
plt.grid()


plt.subplot(3,1, 2)

plt.scatter(logDataIters['timestamp'], logDataIters['iters'], s=.5)
plt.xlabel('timestamp [ms]')
plt.ylabel('ADMM iters')
plt.grid()


plt.subplot(3,1, 3)

plt.plot(logDataCacheLevel['timestamp'], logDataCacheLevel['level'], label="cache level")
plt.xlabel('timestamp [ms]')
plt.ylabel('Cache level')
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
