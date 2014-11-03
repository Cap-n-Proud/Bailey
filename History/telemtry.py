#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      Paolo
#
# Created:     23/08/2014
# Copyright:   (c) Paolo 2014
# Licence:     <your licence>
#-------------------------------------------------------------------------------

import csv
import time
from threading import Timer

from datetime import datetime
from datetime import timedelta

start_time = datetime.now()


ArduRead = {"READ Read_APIDKp": "0",
            "READ Read_APIDKi": "0",
            "READ Read_APIDKd": "0",
            "READ Read_APIDAggKp": "0",
            "READ Read_APIDAggKi": "0",
            "READ Read_APIDAggKd": "0",
            "READ Read_SPIDKp": "0",
            "READ Read_SPIDKi": "0",
            "READ Read_SPIDKd": "0",
            "READ Read_Yaw": "0",
            "READ Read_Roll": "1.2",
            "READ Read_Pitch": "1",
            "READ Read_MotorsON": "False",
            "READ Read_LoopT": "1",
            "READ Read_SetsteerGain": "0",
            "READ Read_SetthrottleGain": "0",
            "READ Read_TriggerAngleAggressive": "0",
            "READ Read_Info": "",
            "READ Read_anglePIDOutput": "1.87"
            }

LogON = False
PathTelFile="/home/pi/Documents/Sketches/Selfbalancingrobot/Log/"
yprFile="/var/www/robot/ypr"
#PathTelFile=""
TelemetryExist = False

i = [start_time]

# returns the elapsed milliseconds since the start of the program
def millis():
   dt = datetime.now() - start_time
   ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
   return ms


def log():
        global TelemetryExist
        NameTelFile="Telemetry"
        NameTelFile = str(NameTelFile + ' ' + (start_time.strftime("%Y-%m-%d %H%M%S"))) + '.csv'
        Cols = ['Time', 'READ Read_anglePIDOutput', 'READ Read_Pitch', 'READ Read_Yaw', 'READ Read_Roll', 'READ Read_APIDKp', 'READ Read_APIDKi', 'READ Read_APIDKd', 'READ Read_APIDAggKp', 'READ Read_APIDAggKi', 'READ Read_APIDAggKd', 'READ Read_SPIDKp', 'READ Read_SPIDKi', 'READ Read_SPIDKd', 'READ Read_TriggerAngleAggressive', 'READ Read_LoopT']
        if (not TelemetryExist):
            TelemetryExist = True

            with open(PathTelFile + NameTelFile, 'a') as f:
                #for s in range(0, len(Cols)):
                f.write(str(Cols));
                f.write('\n');

        # This can be a timer
        # open a file for writing.
        csv_out = open(NameTelFile, 'a',  newline='')

        # create the csv writer object.
        mywriter = csv.writer(csv_out)
        i = [millis()]
        mywriter.writerow(\
        i\
        + [ArduRead['READ Read_anglePIDOutput']] \
        + [ArduRead['READ Read_Pitch']]\
        + [ArduRead['READ Read_Yaw']]\
        + [ArduRead['READ Read_Roll']]\
        + [ArduRead['READ Read_APIDKp']]\
        + [ArduRead['READ Read_APIDKi']]\
        + [ArduRead['READ Read_APIDKd']]\
        + [ArduRead['READ Read_APIDAggKp']]\
        + [ArduRead['READ Read_APIDAggKi']]\
        + [ArduRead['READ Read_APIDAggKd']]\
        + [ArduRead['READ Read_SPIDKp']]\
        + [ArduRead['READ Read_SPIDKi']]\
        + [ArduRead['READ Read_SPIDKd']]\
        + [ArduRead['READ Read_TriggerAngleAggressive']]\
        + [ArduRead['READ Read_LoopT']]\
        )

    # always make sure that you close the file.
    # otherwise you might find that it is empty.
        csv_out.close()

#-----------------------------------------
time.sleep(2)
log()

#for s in (fieldnames):
 #   print(ArduRead[s])