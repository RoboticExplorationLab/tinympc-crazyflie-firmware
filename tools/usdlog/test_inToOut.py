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

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
    
# number of columns and rows for suplot
plotCols = 1
plotRows = 1

# # let's see which keys exists in current data set
# keys = ""
# for k, v in logData.items():
#     keys += k

# current plot for simple subplot usage
plotCurrent = 0

# new figure
plt.figure(args.filename)

plt.subplot(2,1, 1)

plt.plot(logData1['timestamp'], logData1['h0'], '-')#, label="z_0")
plt.plot(logData1['timestamp'], logData1['h1'], '-')#, label="z_1")
plt.plot(logData1['timestamp'], logData1['h2'], '-')#, label="z_2")
plt.plot(logData1['timestamp'], logData1['h3'], '-')#, label="z_3")
plt.plot(logData1['timestamp'], logData1['h4'], '-')#, label="z_4")

plt.plot(logData2['timestamp'], logData2['h5'], '-')#, label="z_5")
plt.plot(logData2['timestamp'], logData2['h6'], '-')#, label="z_6")
plt.plot(logData2['timestamp'], logData2['h7'], '-')#, label="z_7")
plt.plot(logData2['timestamp'], logData2['h8'], '-')#, label="z_8")
plt.plot(logData2['timestamp'], logData2['h9'], '-')#, label="z_9")

plt.plot(logData3['timestamp'], logData3['h10'], '-')#, label="z_10")
plt.plot(logData3['timestamp'], logData3['h11'], '-')#, label="z_11")
plt.plot(logData3['timestamp'], logData3['h12'], '-')#, label="z_12")
plt.plot(logData3['timestamp'], logData3['h13'], '-')#, label="z_13")
plt.plot(logData3['timestamp'], logData3['h14'], '-')#, label="z_14")
# plt.plot(logData['timestamp'], logData['tinympc.horizon_nh_z'], '-', label="Z_Nh")
plt.xlabel('timestamp [ms]')
plt.ylabel('Position [m]')



plt.subplot(2,1, 2)

plt.plot(logDataFixed['timestamp'], logDataFixed['tinympc.initial_velocity'], '-', label="initial z vel")

plt.xlabel('timestamp [ms]')
plt.ylabel('Velocity [m/s]')




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
