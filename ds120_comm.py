"""
Project Name    : Python root control program
File Name       : control.py
Purpose         : Controlling the Suruga-Seiki DS102 controller box for a 6-axis
                    motorized stage system.
Creation Date   : 2021-July-27
Authors         : D. Heydari and M. Catuneanu

NTT-Mabuchi Group
"""

from PyQt5.QtCore import QThread, pyqtSignal, QTimer,QObject

__all__ = ["Axis", "Units", "Attribute", "Seiki"]

from os import device_encoding, write
import serial
from enum import Enum
from enum import IntEnum
from math import hypot
from textwrap import wrap
import time
"""
Conventions:
    Z: optic axis (propagation direction)
    X: in plane of table
    Y: out of plane of table
"""

from aenum import AutoNumberEnum
class Axis(AutoNumberEnum):
    _init_ = 'value text'
    Y =  1, 'AXI1'
    X =  2, 'AXI2'


class Units(IntEnum):
    PULSE = 0
    UM = 1
    MM = 2
    DEG = 3
    MRAD = 4

class Command(Enum):
    IDN = '*IDN'
    SPD = 'SELSP'
    POS = 'POS'
    UNT = 'UNIT'
    GOA = 'GOABS'
    DRD = 'DRDIV'
    RES = 'RESOLUT'
    JOG = 'PULS'
    STOP = 'STOP 0'
    ESTOP = 'STOP 1'
    HOME = 'HOME'
    MOVING = 'MOTIONAll'

class Controller(QThread):
    MAXCHAR = 100
    pos_xy = pyqtSignal(list)
    def __init__(self, parent=None):
        super().__init__(parent)
        self.com_port = '/dev/ttyUSB0' 
        self.baud_rate = 38400
        self.ser = serial.Serial(self.com_port, self.baud_rate, timeout=0.1)

        self.stage_running = True
        self.stage_command = []

    def run(self):

        while self.stage_running:
            if len(self.stage_command) != 0:
                eval(self.stage_command[0])
                del self.stage_command[0]
            time.sleep(0.13)


    def _serial_write_read(self, write_data):
        self.ser.write(write_data)
        return self.ser.read_until()

    def _serial_write(self, write_data):
        self.ser.write(write_data)

    def _clean(self, output):
        return output.decode('utf-8').split()


    def query_xy(self, attributes):
        query = [(i, j) for i in ['AXIs2', 'AXIs1'] for j in attributes]
        write = ''.join([m[0] + ':' + str(m[1].value) + '?' + '\r' for m in query])
        self.outputs=self._clean(self._serial_write_read(write.encode('utf-8')))
        
        #while self.outputs == '' or len(list(map(int, self.outputs))) !=2: 
        #    time.sleep(0.13)
        #    self.outputs=self._clean(self._serial_write_read(write.encode('utf-8')))
        self.pos_xy.emit(self.outputs)
        


    def query(self, axes, attributes):
        query = [(i, j) for i in axes for j in attributes]
        write = ''.join([m[0] + ':' + str(m[1].value) + '?' + '\r' for m in query])
        if len(write) > Controller.MAXCHAR: return query, self._multi_write(write)
        return query, self._clean(self._serial_write_read(write.encode('utf-8')))

    def _multi_write(self, write):
        multiwrite = wrap(write, Controller.MAXCHAR, replace_whitespace=False)
        print('multi-write active')
        multiwrite = [m + '\r' for m in multiwrite]
        result = []
        for m in multiwrite:
            result.append(self._serial_write_read(( m.split('\n')[0]).encode('utf-8') ))
            time.sleep(0.2)
        return [item for sublist in [self._clean(r) for r in result] for item in sublist]

    def set(self, axis, attribute, value):  # one axis and attribute at a time
        write = axis.text + ':' + str(attribute.value) + ' ' + str(value) + '\r'
        self._serial_write(write.encode('utf-8'))

    def sent_command(self, command):  # one axis and attribute at a time
        write = command + '\r'
        print(self._serial_write_read(write.encode('utf-8')))

    def jog(self, axis, data, dir='CW'):
        writedata = (axis.text + ":PULS " + str(int(data)) + \
            ':GO ' + str(dir) + ':DW' + '\r').encode('utf-8')
        self._serial_write_read(writedata)

    def goabs_XY(self, data):
        cur_pos=list(map(int, self.outputs))
        writedata = ("AXIs2"+ ":UNIT 1:SELSP 6"
                            + ":GOABS " + str(data[0]) + ":DW"
                    +":AXIs1"+ ":UNIT 1:SELSP 6"
                            + ":GOABS " + str(data[1]) + ":DW"
                            + '\r').encode('utf-8')
        self._serial_write(writedata)
        traveltime=round(hypot(data[0]-cur_pos[0], data[1]-cur_pos[1])/5)
        self.timer1=QTimer()
        self.timer1.timeout.connect(lambda:self.query_xy([Command.POS]))
        self.timer1.setSingleShot(True)
        self.timer1.start(traveltime+200)
 

    def cont_jog(self, axis, dir, speed):

        writedata = (axis.text 
                           + ":Unit 0"
                           + ":LSpeed0 " + str(round(speed*100))
                           + ":Rate0 " + str(round(speed*10)) 
                           + ":FSpeed0 " + str(round(speed*100)) 
                           + ":GO " + str(dir)+ "J" + '\r').encode('utf-8') # Starts the movement of the axis.
        self._serial_write(writedata)

    def cont_jogXY(self, dir, speed, ini):
        #ini_time=time.time()
        if ini:
            writedata0 = ('STOP 1' + '\r').encode('utf-8')
            self._serial_write_read(writedata0)
        writedata = ("AXIs1"+ ":Unit 1:SELSP 0"
                           + ":LSpeed0 " + str(speed[0])
                           + ":Rate0 " + str(10) 
                           + ":FSpeed0 " + str(speed[0]) 
                           + ":GO " + str(dir[0])+ "J"
                      +":AXIs2"+ ":Unit 1:SELSP 1"
                           + ":LSpeed1 " + str(speed[1])
                           + ":Rate1 " + str(10) 
                           + ":FSpeed1 " + str(speed[1]) 
                           + ":GO " + str(dir[1])+ "J" 
                           + '\r').encode('utf-8') # Starts the movement of the axis.
        self._serial_write(writedata)
        #loop_time=time.time()-ini_time
        #print(loop_time) #loop_time == 0.11 if ini==true

    def emergency_stop(self):
        writedata = ('STOP 0' + '\r').encode('utf-8')
        self._serial_write(writedata)

    def slow_stop(self):
        writedata = ('STOP 1' + '\r').encode('utf-8')
        self._serial_write(writedata)

    # Parameter query
    def identify(self):
        writedata = ('*IDN?' + '\r').encode('utf-8')
        return self._serial_write_read(writedata)

    def _verify_home(self, axis):  # (0), 1: (un)detected
        writedata = axis.text + ':HOME?'
        w_data = (writedata + '\r').encode('utf-8')
        return self._serial_write_read(w_data)
    
    def _verify_all_moving(self):
        writedata = 'MOTIONAll?'
        w_data = (writedata + '\r').encode('utf-8')
        return self._serial_write_read(w_data)

    def stop(self):
        self.stage_running = False