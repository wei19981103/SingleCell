from PyQt5.QtGui import QPainter
from PyQt5.QtCore import QTimer, QPointF, QRectF, QLineF, Qt, QThread
from PyQt5.QtWidgets import QWidget, QMainWindow, QApplication, QGridLayout, QStyleFactory
from PyQt5 import QtCore
import sys
from enum import Enum
from ds120_comm import Command, Axis, Controller
import math 
import time


class Joystick(QWidget):
    #pos_xy = QtCore.pyqtSignal(list)

    stageThread = QThread()
    def __init__(self, parent=None):
        #super(Joystick, self).__init__(parent)
        super().__init__(parent)
        self.setMinimumSize(150, 150)
        self.setMaximumSize(150, 150)
        self.movingOffset = QPointF(0, 0)
        self.grabCenter = False
        self.__maxDistance = 50
        self.Stage = Controller()
        self.Stage.start()
        self.speed=[]
        self.direction = []
        self.jog_dir = 0
        self.switchdir=False
        self.jog_speed_mode=200
        #self.Stage.set(Axis['X'], Command.SPD, 2)
        #self.Stage.set(Axis['Y'], Command.SPD, 2)

    def paintEvent(self, event):
        painter = QPainter(self)
        self.bounds = QRectF(-self.__maxDistance, -self.__maxDistance, self.__maxDistance * 2, self.__maxDistance * 2).translated(self._center())
        painter.drawEllipse(self.bounds)
        painter.setBrush(Qt.black)
        painter.drawEllipse(self._centerEllipse())

    def _centerEllipse(self):
        if self.grabCenter:
            return QRectF(-20, -20, 40, 40).translated(self.movingOffset)
        return QRectF(-20, -20, 40, 40).translated(self._center())

    def _center(self):
        return QPointF(self.width()/2, self.height()/2)


    def _boundJoystick(self, point):
        limitLine = QLineF(self._center(), point)
        if (limitLine.length() > self.__maxDistance):
            limitLine.setLength(self.__maxDistance)
        return limitLine.p2()

    def joystickDirection(self):
        if not self.grabCenter:
            return 0
        normVector = QLineF(self._center(), self.movingOffset)
        currentDistance = normVector.length()
        angle = normVector.angle()

        distance = min(currentDistance / self.__maxDistance, 1.0)
        
        

        return (angle, distance)

      
    def start_cont_jog(self):
        self.timer=QTimer()

        self.timer.timeout.connect(self.sendjogcommand)
        self.update_pos_count=1
        self.count=0
        self.timer.start(65)


    def mousePressEvent(self, ev):
        self.grabCenter = self.bounds.contains(ev.pos())
        if self.grabCenter:
     
            self.movingOffset = self._boundJoystick(ev.pos())
            self.ismoving=True
            angle, distance=self.joystickDirection()
            self.speed_calculate(angle, distance)
            #self.Stage.cont_jogXY(self.direction,self.speed,ini=False)
            self.jog_dir_now=self.jog_dir
            self.start_cont_jog()        
            self.update()
        #self.grabCenter = self._centerEllipse().contains(ev.pos())
        return super().mousePressEvent(ev)

    def mouseReleaseEvent(self, event):
        self.grabCenter = False
        self.movingOffset = QPointF(0, 0) 
        self.ismoving=False
        self.count=0
        self.update()
        self.Stage.stage_command.append('self.slow_stop()')
        #self.Stage.slow_stop()
        self.timer.stop()
        #self.Stage.query_xy([Command.POS])
        self.Stage.stage_command.append('self.query_xy([Command.POS])')
  
        #self.pos_xy.emit(outputs)
        #self.lbl3.setText('Stage Position (X,Y): (' + ','.join(self.stage_pos)+')')
        #loop_time=time.time()-ini_time
        #print(loop_time) #loop_time==0.15 
        

    def mouseMoveEvent(self, event):
        
        if self.grabCenter:
            self.movingOffset = self._boundJoystick(event.pos())
            angle, distance=self.joystickDirection()
            self.speed_calculate(angle, distance)
            self.ismoving=True
            self.update()
            

    def sendjogcommand(self):
        
        if self.count==0:
            if self.ismoving:
                if self.jog_dir_now == self.jog_dir:
                    self.count=1
                    self.Stage.stage_command.append('self.cont_jogXY('+ str(self.direction) + ',' + str(self.speed) +',ini=False)')
                    #self.Stage.cont_jogXY(self.direction,self.speed,ini=False)
                    
                else:
                    self.count=1 
                    #self.Stage.cont_jogXY(self.direction,self.speed,ini=True)
                    self.Stage.stage_command.append('self.cont_jogXY('+ str(self.direction) +',' + str(self.speed) +',ini=True)')
                    self.jog_dir_now = self.jog_dir
                self.ismoving=False
            else:
                if self.update_pos_count % 5 == 0:
                    self.count=2 #wait 2*65=130msec
                    self.update_pos_count =0
                    self.Stage.stage_command.append('self.query_xy([Command.POS])')
            
        else:
            self.count-=1

        self.update_pos_count +=1

    def speed_calculate(self, angle, distance):
        if 0 <= angle < 90:
            self.direction=['CW','CCW']
            self.jog_dir = 1
        elif 90 <= angle < 180:
            self.direction=['CW','CW']
            self.jog_dir = 2
        elif 180 <= angle < 270:
            self.direction=['CCW','CW']
            self.jog_dir = 3
        elif angle >= 270:    
            self.direction=['CCW','CCW']
            self.jog_dir = 4
        self.speed=[round(self.jog_speed_mode*abs(distance*(math.sin(math.radians(angle))))),round(self.jog_speed_mode*abs(distance*(math.cos(math.radians(angle)))))]

        # Y+ = axi1_CW, X+ = axi2_CCW



if __name__ == '__main__':
    # Create main application window
    app = QApplication([])
    app.setStyle(QStyleFactory.create("Cleanlooks"))
    mw = QMainWindow()
    mw.setWindowTitle('Joystick example')

    # Create and set widget layout
    # Main widget container
    cw = QWidget()
    ml = QGridLayout()
    cw.setLayout(ml)
    mw.setCentralWidget(cw)

    # Create joystick 
    joystick = Joystick()


    # ml.addLayout(joystick.get_joystick_layout(),0,0)
    ml.addWidget(joystick,0,0)

    mw.show()

    ## Start Qt event loop unless running in interactive mode or using pyside.
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QApplication.instance().exec_()