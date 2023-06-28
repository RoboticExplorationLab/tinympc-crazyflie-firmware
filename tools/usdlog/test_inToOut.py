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

#only focus on regular logging
logData = logData['fixedFrequency']

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
    
# number of columns and rows for suplot
plotCols = 1
plotRows = 3

# let's see which keys exists in current data set
keys = ""
for k, v in logData.items():
    keys += k
    
# current plot for simple subplot usage
plotCurrent = 0

# new figure
plt.figure(args.filename)

if re.search('stabilizer', keys):
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['timestamp'], logData['stabilizer.intToOut'], '-')
    plt.xlabel('timestamp [ms]')
    plt.ylabel('Time [us]')
 
if re.search('stateEstimate', keys):
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['timestamp'], logData['stateEstimate.x'], '-', label='X')
    plt.plot(logData['timestamp'], logData['stateEstimate.y'], '-', label='Y')
    plt.plot(logData['timestamp'], logData['stateEstimate.z'], '-', label='Z')
    plt.xlabel('timestamp [ms]')
    plt.ylabel('Postion [m]')

if re.search('stateEstimate', keys):
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['timestamp'], logData['stateEstimate.qx'], '-', label='X')
    plt.plot(logData['timestamp'], logData['stateEstimate.qy'], '-', label='Y')
    plt.plot(logData['timestamp'], logData['stateEstimate.qz'], '-', label='Z')
    plt.plot(logData['timestamp'], logData['stateEstimate.qw'], '-', label='Z')
    plt.xlabel('timestamp [ms]')
    plt.ylabel('Quaternion')

plt.show()
