#! /usr/bin/env python
import sys
import asyncore
import socket
import select
import time
import serial
import os
import re
import threading
#import commands #used to read CPU temp MIGHT NOT BE NEEDED, chek system files

initHash = 0
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
            "READ Read_Roll": "0",
            "READ Read_Pitch": "0",
            "READ Read_MotorsON": "False",
            "READ Read_LoopT": "0",
            "READ Read_SetsteerGain": "0",
            "READ Read_SetthrottleGain": "0",
            "READ Read_TriggerAngleAggressive": "0",
            "READ Read_Info": "",
            "READ Read_anglePIDOutput": "0"
            }
connected = False
init = False
LogON = False
PathLogFile="/home/pi/Documents/Sketches/Selfbalancingrobot/Log/"
yprFile="/var/www/robot/ypr"
NameLogFile="Log_file"
# Check if Arduino is connected
# Find arduino path by unpluggin it and run "ls -la /dev/"
# then plug it and run the command again to see the
# additional port

def handle_data(data, LogON):
        global ArduRead
        with open(yprFile, 'w') as ypr:
                ypr.write(ArduRead['READ Read_Yaw']);
                ypr.write('\n');
                ypr.write(ArduRead['READ Read_Pitch']);
                ypr.write('\n');
                ypr.write(ArduRead['READ Read_Roll']);
                ypr.write('\n');
                ypr.close();
        if (data.count(' ')==2 and 'READ' in data):
                CMDRec = re.split('[ ]', data)
                ArduRead[CMDRec[0] + ' ' + CMDRec[1]] = (CMDRec[2].rstrip())
                #print ArduRead['READ Read_anglePIDOutput'];
                if (LogON == True):
                    logData();

def logData():
        global LogFile
        with open(PathLogFile + NameLogFile, 'a') as f:
                #[f.write('{0}\t{1}\t'.format(key, value) + str(time.time()) + '\n') for key, value in ArduRead.items()]
                #for key in ArduRead:

                #f.write(str(int(time.time())) + ",");
                f.write(str(ArduRead.values()));
                #f.write("," + str(ArduRead["READ Read_anglePIDOutput"]);
                f.write('\n');


def read_from_port(ser):
          global ArduRead
          global LogON
          while True:
                  reading = ser.readline()#.decode()
                  handle_data(reading, LogON)

def get_cpu_temp():
    tempFile = open( "/sys/class/thermal/thermal_zone0/temp" )
    cpu_temp = tempFile.read()
    tempFile.close()
    return float(cpu_temp)/1000
    # Uncomment the next line if you want the temp in Fahrenheit
    #return float(1.8*cpu_temp)+32


class Client(asyncore.dispatcher_with_send):
    def __init__(self, socket=None, pollster=None):
        asyncore.dispatcher_with_send.__init__(self, socket)
        self.data = ''
        if pollster:
            self.pollster = pollster
            pollster.register(self, select.EPOLLIN)

    def handle_close(self):
        if self.pollster:
            self.pollster.unregister(self)

    def handle_read(self):
        receivedData = self.recv(8192)
        if not receivedData:
            self.close()
            return
        receivedData = self.data + receivedData
        while '\n' in receivedData:
            line, receivedData = receivedData.split('\n',1)
            self.handle_command(line)
        self.data = receivedData


# Main block to handle communication
# It is essental to always send somenthing back to the remote

    def handle_command(self, line):
        global init
        global ArduRead
        global initHash
        global data
        global LogON

    	# The Raspberry Pi simply echoes the remote command to the Arduino

        if (line.startswith('SCMD')):
                ser.write(line  + '\n')
        #print (line)

        #print ser.readline()


		# Initialization of the key variables the "reads" values of the remote
		# are echoed to the Arduino that in turn sends back the value stored in the EEPROM
		# the loop iscompleted by sending back to the remote the value
        if (line == 'READ Read_APIDKp'):
            #ser.write(line  + '\n')
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
            #print(line  + ' ' + str(ArduRead[line]) + '\n')

            initHash += 1
        elif (line == 'READ Read_APIDKi'):
            self.send(line  + str(ArduRead[line]) + '\n')
            initHash += 1
        elif (line == 'READ Read_APIDKd'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
            initHash += 1
        elif (line == 'READ Read_APIDAggKp'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
            initHash += 1
        elif (line == 'READ Read_APIDAggKi'):
            self.send(line  + str(ArduRead[line]) + '\n')
            initHash += 1
        elif (line == 'READ Read_APIDAggKd'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
            initHash += 1

        elif (line == 'READ Read_SPIDKp'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
        elif (line == 'READ Read_SPIDKi'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
            initHash += 1
        elif (line == 'READ Read_SPIDKd'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
            initHash += 1
        elif (line == 'READ Read_Pitch'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')

        elif (line == 'READ Read_Roll'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')

        elif (line == 'READ Read_Yaw'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
        elif (line == 'READ motorsON'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
            initHash += 1
        elif (line == 'READ Read_CPUTemp'):
                self.send(line +  str(get_cpu_temp()) + '/n')
        elif (line == 'READ Read_CPUUtil'):
            self.send(line +  ' ' + str(psutil.cpu_percent(interval=1)) + '/n')
        elif (line == 'READ Read_LoopT'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
        elif (line == 'READ Read_Info'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
        elif (line == 'get status'):
            self.send('on\n')
            initHash += 1
        elif (line == 'READ Read_TriggerAngleAggressive'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
            initHash += 1
        elif (line == 'READ Read_SetthrottleGain'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
            initHash += 1
        elif (line == 'READ Read_SetsteerGain'):
            self.send(line  + ' ' + str(ArduRead[line]) + '\n')
            initHash += 1
        elif (line == 'SERCMD LogON'):
            self.send(line + '\n')
            LogON = True;
        elif (line == 'SERCMD LogOFF'):
            self.send(line + '\n')
            LogON = False;

            # This is how to read potentiometers and items that sends "command {value}"

        elif line.startswith("CH11"):
                self.send('CH11 change\n')
                #do_something(line[5:])  # this will crop the value out of the line variable



		# Good idea to allow closure of the socket to avoid waiting for a timeout
		# you should always place a button in the remote to send an EXIT command
        elif line == 'EXIT':
            print('Closing everything')

            #conn.shutdown()
            #handle_close(self)
            self.close()
            os.system("sudo shutdown -h now")
            quit()
		# It is essental to always send somenthing back to the remote
        elif line == 'REBOOT':
            print('Closing everything')

            #conn.shutdown()
            #handle_close(self)
            self.close()
            os.system("sudo reboot")
            quit()
        else:
            # It is essental to always send somenthing back to the remote        else:
            self.send('unknown command\n')



class Server(asyncore.dispatcher):
    def __init__(self, listen_to, pollster):
        asyncore.dispatcher.__init__(self)
        self.pollster = pollster
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.bind(listen_to)
        self.listen(5)

    def handle_accept(self):
        newSocket, address = self.accept()
        print "Connected from", address
        Client(newSocket,self.pollster)

def readwrite(obj, flags):
    try:
        if flags & select.EPOLLIN:
            obj.handle_read_event()
        if flags & select.EPOLLOUT:
            obj.handle_write_event()
        if flags & select.EPOLLPRI:
            obj.handle_expt_event()
        if flags & (select.EPOLLHUP | select.EPOLLERR | select.POLLNVAL):
            obj.handle_close()
    except socket.error, e:
        if e.args[0] not in asyncore._DISCONNECTED:
            obj.handle_error()
        else:
            obj.handle_close()
    except asyncore._reraised_exceptions:
        raise
    except:
        obj.handle_error()


class EPoll(object):
    def __init__(self):
        self.epoll = select.epoll()
        self.fdmap = {}
    def register(self, obj, flags):
        fd = obj.fileno()
        self.epoll.register(fd, flags)
        self.fdmap[fd] = obj
    def unregister(self, obj):
        fd = obj.fileno()
        del self.fdmap[fd]
        self.epoll.unregister(fd)
    def poll(self):
        evt = self.epoll.poll()
        for fd, flags in evt:
            yield self.fdmap[fd], flags



try:
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout=0.5)
    print ('Arduino connected to: ' + ser.name)
    thread = threading.Thread(target=read_from_port, args=(ser,))
    thread.start()
    NameLogFile = NameLogFile + str(int(time.time())) + '.csv'
    with open(PathLogFile + NameLogFile, 'a') as f:
            f.write(str(ArduRead.keys()));
            f.write('\n');


except serial.serialutil.SerialException:
        print ('Arduino not connected')
        quit()


if __name__ == "__main__":

        pollster = EPoll()
        pollster.register(Server(("",54321),pollster), select.EPOLLIN)


        while True:
                evt = pollster.poll()
                for obj, flags in evt:
                        readwrite(obj, flags)


