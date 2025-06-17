from PyQt5.QtWidgets import (QApplication, QMainWindow, 
    QPushButton, QLabel, QVBoxLayout, QWidget, QAction, QDesktopWidget, QInputDialog, QLineEdit, QCheckBox)
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QBrush
from PyQt5.QtCore import QPropertyAnimation, QPoint, QEasingCurve, Qt, QSequentialAnimationGroup, QSize, QThread
import cv2
import sys, time, os
import numpy as np
import torch
import pandas as pd
from PyQt5 import QtCore, QtGui, QtWidgets
from Ui_main import Ui_MainWindow
import pypylon.pylon as py
from math import hypot
from ds120_comm import Command, Axis, Controller
from joystick import Joystick
from datetime import datetime

BINGING_FACTOR = 1 # 1,2,4
SCANLINE_HEIGHT = 2168//BINGING_FACTOR
SCANLINE_WIDTH = 4096//BINGING_FACTOR
HALF_H=SCANLINE_HEIGHT//2
HALF_W=SCANLINE_WIDTH//2
STEP_UM_FACTOR=1.9944
MAP_HEIGHT = 20000 #(20mm)
MAP_WIDTH = 10000 #(10mm)

#SCREEN_FACTOR_X=1920/1024 
#SCREEN_FACTOR_Y=1080/542 
#SCREEN_OFFSET_X=0
#SCREEN_OFFSET_Y=0
SCREEN_FACTOR_X=1920/1024 # Projected Correction Factor between Camera and DMD
SCREEN_FACTOR_Y=1010/542 
SCREEN_OFFSET_X=10
SCREEN_OFFSET_Y=15
SPOT_SIZE=50 # pix = 2um = 1step
SPEED=250

class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):    
      
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        
        self.setupUi(self)
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)
        self.viewData.setScaledContents(True)
        self.view_x = self.view.horizontalScrollBar()
        self.view_y = self.view.verticalScrollBar()
        self.viewData.setMouseTracking(True)
        self.view.installEventFilter(self)
        self.viewData.installEventFilter(self)
        self.last_move_x = 0
        self.last_move_y = 0
        self.frame_num = 0
        self.path_index=0
        self.dot_number=np.zeros(10000,dtype=int)
        self.pdot=[0,0]
        self.roi_rate = 0.25
        self.global_pos=np.array([0,0])
        self.mouse_wpos=np.array([0,0])
        self.stage_pos=np.array([0,0])
        self.mouse_global_pos=np.add(self.global_pos,[self.mouse_wpos[0]-HALF_W,HALF_H-self.mouse_wpos[1]])
        self.home_pos=self.stage_pos
        self.img_rgb=np.zeros((SCANLINE_HEIGHT,SCANLINE_WIDTH,3),dtype=np.uint8)
        self.img_circle=np.zeros((SCANLINE_HEIGHT,SCANLINE_WIDTH,3),dtype=np.uint8)
        self.img_rgb1=np.zeros((SCANLINE_HEIGHT,SCANLINE_WIDTH,3),dtype=np.uint8)
        self.path_map=np.zeros((MAP_HEIGHT,MAP_WIDTH,3),dtype=np.uint8)

        self.w = AnotherWindow()
        self.ProcessCam = Camera()
        self.cv_cal=CV_calculation()
        self.joystick=Joystick()
        self.midrightBox2.addWidget(self.joystick)
        
        self.joystick.Stage.pos_xy.connect(self.update_position)
        self.joystick.Stage.query_xy([Command.POS])
        self.home_pos=self.stage_pos
        self.update_position(list(map(str, self.stage_pos)))

        #---- YOLOv5 model ----
        self.yolov5_path = '/home/mengting/Wei/231211/yolov5'
        self.model_path = '/home/mengting/Wei/231211/yolov5/runs/train/exp/weights/best.pt/'
        self.model = self.load_model()
        self.cv_cal.get_model(self.model)
        
        if self.ProcessCam.connect:
            self.debugBar("Connected Device: "+ str(self.ProcessCam.cam.GetDeviceInfo().GetModelName())+ 
            " (HxV):"+ str(self.ProcessCam.cam.Width.Value) + "x"+ str(self.ProcessCam.cam.Height.Value))
            self.ProcessCam.rawdata.connect(self.getRaw)
        else:
            self.debugBar('Disconnection!!!')



        self.command_action.triggered.connect(self.set_stage)
        self.camBtn_open.clicked.connect(self.openCam)
        self.camBtn_stop.clicked.connect(self.stopCam)
        self.camBtn_record.clicked.connect(self.recordCam)
        self.camBtn_capture.clicked.connect(self.captureCam)
        self.Btn_w.clicked.connect(self.toggle_window)
        self.camBtn_stop.setEnabled(False)
        self.lbl2.setText('Droplet Selected: %0.0f' % (self.path_index))
        self.joyBtn_slow.clicked.connect(self.set_joyspeed)
        self.joyBtn_fast.clicked.connect(self.set_joyspeed)
        self.joyBtn_goto.clicked.connect(self.joy_goabs)
        self.joyBtn_go.clicked.connect(self.joy_go)
        self.joyBtn_sethome.clicked.connect(self.set_home)
        self.joy_LCDx.setText(str(self.global_pos[0]))
        self.joy_LCDy.setText(str(self.global_pos[1]))
        self.joyBtn_gohome.clicked.connect(self.go_home)
        self.Btn_endtracking.clicked.connect(self.end_tracking)
        self.Btn_detectandpredict.clicked.connect(self.start_tracking)




    def getRaw(self, data):
        self.showData(data)
        self.cv_cal.get_img(data)

    def getCircleImg(self, data):
        self.img_circle = data

    def openCam(self):
        if self.ProcessCam.connect:
            self.ProcessCam.open()
            self.ProcessCam.start()   

            self.camBtn_open.setEnabled(False)
            self.camBtn_stop.setEnabled(True)
            self.viewCbo_roi.setEnabled(True)
            self.camBtn_record.setEnabled(False)
    
    def recordCam(self):
        if self.ProcessCam.connect: 
            self.ProcessCam.open()
            self.ProcessCam.record()
            self.ProcessCam.start()

            self.camBtn_open.setEnabled(False)
            self.camBtn_stop.setEnabled(True)
            self.viewCbo_roi.setEnabled(True)
            self.camBtn_record.setEnabled(False)

    def captureCam(self):
        if self.ProcessCam.connect:
            frame = self.img_rgb

            dt = datetime.now()
            dt = dt.strftime("%d_%m_%Y_%H_%M_%S")
            filename = '/home/mengting/Wei/221205/%s.png' % dt
            cv2.imwrite(filename, frame)



    def stopCam(self):
        if self.ProcessCam.connect:
            self.ProcessCam.stop()
            self.camBtn_open.setEnabled(True)
            self.camBtn_record.setEnabled(True)
            self.camBtn_stop.setEnabled(False)
            self.viewCbo_roi.setEnabled(False)

    def showData(self, img):
        if self.frame_num == 0:
            self.time_start = time.time()
        self.Ny, self.Nx = SCANLINE_HEIGHT, SCANLINE_WIDTH
        if self.viewCbo_roi.currentIndex() == 0: self.roi_rate = 0.125
        elif self.viewCbo_roi.currentIndex() == 1: self.roi_rate = 0.25
        elif self.viewCbo_roi.currentIndex() == 2: self.roi_rate = 0.5
        elif self.viewCbo_roi.currentIndex() == 3: self.roi_rate = 0.75
        elif self.viewCbo_roi.currentIndex() == 4: self.roi_rate = 1.00
        else: pass
        self.viewForm.setMinimumSize(round(self.Nx*self.roi_rate), round(self.Ny*self.roi_rate))
        self.viewForm.setMaximumSize(round(self.Nx*self.roi_rate), round(self.Ny*self.roi_rate))
        self.viewData.setMinimumSize(round(self.Nx*self.roi_rate), round(self.Ny*self.roi_rate))
        self.viewData.setMaximumSize(round(self.Nx*self.roi_rate), round(self.Ny*self.roi_rate))

        # 顏色
        self.img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        self.processingtimeperframe_start = time.time()

        if self.cv_cal.tracking == True:
            self.lbl_cirlce_num.setText(' Droplet detected: %0.0f' % ((len(self.cv_cal.droplets))))
            self.cv_cal.track(img)
        else:
            self.lbl_cirlce_num.setText(' Droplet not detected')


        if self.chk_show_circle.isChecked():
            self.cv_cal.drawing = True
            self.cv_cal.circle_img.connect(self.getCircleImg)
            qimg = QtGui.QImage(self.img_circle, self.Nx, self.Ny, QtGui.QImage.Format_RGB888)
            self.viewData.setScaledContents(True)
            self.viewData.setPixmap(QtGui.QPixmap.fromImage(qimg))
        else:
            self.cv_cal.drawing = False
            qimg = QtGui.QImage(self.img_rgb, self.Nx, self.Ny, QtGui.QImage.Format_RGB888)
            self.viewData.setScaledContents(True)
            self.viewData.setPixmap(QtGui.QPixmap.fromImage(qimg))

        self.processingtimeperframe_total = time.time() - self.processingtimeperframe_start
        print("One frame taken time:" + str(self.processingtimeperframe_total) + 's') 
        

        if self.frame_num >= 0:
            self.frame_num += 1 
            if self.frame_num % 100 == 0:
                self.t_total = time.time() - self.time_start
                self.frame_rate = float(self.frame_num) / self.t_total
                self.debugBar('FPS: %0.1f frames/sec' % self.frame_rate)
                self.frame_num = 0

    def eventFilter(self, source, event):
        if source == self.viewData:
            if event.type() == QtCore.QEvent.MouseButtonPress:
                if self.joyBtn_sethome.isChecked():
                    mouse_global_pos=self.mousetoglobal([event.pos().x(),event.pos().y()])
                    new_home=self.globaltostage(mouse_global_pos)
                    self.home_pos=new_home
                    self.update_position(list(map(str, self.stage_pos)))

                    self.joyBtn_sethome.toggle()

            if event.type() == QtCore.QEvent.MouseMove and event.button() == QtCore.Qt.NoButton:
                    self.mouse_wpos=[round(event.pos().x()/self.roi_rate), round(event.pos().y()/self.roi_rate)]
                    self.mouse_global_pos=np.add(self.global_pos,[self.mouse_wpos[0]-HALF_W,HALF_H-self.mouse_wpos[1]])
                    self.lbl1.setText(str([event.pos().x(), event.pos().y()]))
                    self.lbl_m_local_xy.setText(' Mouse (Local):  ' + str(self.mouse_wpos))
                    self.lbl_m_global_xy.setText(' Mouse (Global):  ' + str(self.mouse_global_pos))
            

        if source == self.view:
            if self.viewCbo_roi.currentIndex() > 1:
                if event.type() == QtCore.QEvent.MouseMove:
                    if self.last_move_x == 0 or self.last_move_y == 0:
                        self.last_move_x = event.pos().x()
                        self.last_move_y = event.pos().y()
                    distance_x = self.last_move_x - event.pos().x()
                    distance_y = self.last_move_y - event.pos().y()
                    self.view_x.setValue(self.view_x.value() + distance_x)
                    self.view_y.setValue(self.view_y.value() + distance_y)
                    self.last_move_x = event.pos().x()
                    self.last_move_y = event.pos().y()
                    
                elif event.type() == QtCore.QEvent.MouseButtonRelease:
                    self.last_move_x = 0
                    self.last_move_y = 0          
     
                    
        return QtWidgets.QWidget.eventFilter(self, source, event)
    
    def closeEvent(self, event):
        if self.ProcessCam.running:
            self.ProcessCam.close()
            self.ProcessCam.terminate()
        QtWidgets.QApplication.closeAllWindows()
        
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Q:
            if self.ProcessCam.running:
                self.ProcessCam.close()
                time.sleep(1)
                self.ProcessCam.terminate()
            QtWidgets.QApplication.closeAllWindows()  

        if event.key() == QtCore.Qt.Key_Shift:
            if self.dot_number[self.path_index] != 0:
                self.path_index += 1
                self.lbl2.setText('Droplet Selected: %0.0f' % (self.path_index))
            #print(self.droplet_pos)
        
        if event.key() == QtCore.Qt.Key_H:
            for i in range(self.w.index_activated_anim_group):
                self.w.label[i].hide()

    def debugBar(self, msg):
        self.statusBar.showMessage(str(msg))
    
    def toggle_window(self, checked):
        if self.w.isVisible():
            
            self.w.hide()
        else:
            self.w.show()
            monitor = QDesktopWidget().screenGeometry(1)
            self.w.move(monitor.left(), monitor.top())
            self.w.showFullScreen()

    def update_position(self,data):
        self.stage_pos=list(map(int, data))
        self.global_pos=self.stagetoglobal(self.stage_pos)
        self.joy_LCDx.setText(str(self.global_pos[0]))
        self.joy_LCDy.setText(str(self.global_pos[1]))
        self.lbl3.setText('Stage Position (X,Y): (' + ','.join(data)+')')
        
        
    def stagetoglobal(self,data):
        result=[round((self.home_pos[0]-data[0])*STEP_UM_FACTOR),round((data[1]-self.home_pos[1])*STEP_UM_FACTOR)]
        return result    

    def globaltostage(self,data):
        result=[round(data[0]/-STEP_UM_FACTOR)+self.home_pos[0],round(data[1]/STEP_UM_FACTOR)+self.home_pos[1]]
        return result

    def mousetoglobal(self,data):
        result=[self.global_pos[0]+round(data[0]/self.roi_rate)-HALF_W , self.global_pos[1]+HALF_H-round(data[1]/self.roi_rate)]
        return result

    def set_joyspeed(self):
        if self.joyBtn_slow.isEnabled():
                self.joyBtn_slow.setEnabled(False)
                self.joyBtn_slow.setStyleSheet('''color: black;background-color: Gray;''')
                self.joyBtn_fast.setEnabled(True)
                self.joyBtn_fast.setStyleSheet('''color: black;background-color: lightGray;''')
                self.joystick.jog_speed_mode=50
        elif self.joyBtn_fast.isEnabled():
                self.joyBtn_fast.setEnabled(False)
                self.joyBtn_fast.setStyleSheet('''color: black;background-color: Gray;''')
                self.joyBtn_slow.setEnabled(True)
                self.joyBtn_slow.setStyleSheet('''color: black;background-color: lightGray;''')
                self.joystick.jog_speed_mode=2000

    def joy_goabs(self):
        if self.textx.text() != '' and self.texty.text() != '':
            data=self.globaltostage([int(self.textx.text()),int(self.texty.text())])  
            if  -25000 < data[0] < 25000 and -25000 < data[1] < 25000: 
                self.joystick.Stage.goabs_XY(data) 
        else:
            self.debugBar('Out of Range!!')

    def joy_go(self):
        data=self.globaltostage([int(self.textx.text())+int(self.joy_LCDx.text()),int(self.texty.text())+int(self.joy_LCDy.text())])
        #data=[int(self.joy_LCDx.text())+int(self.textx.text()),int(self.joy_LCDy.text())+int(self.texty.text())]
        if  -25000 < data[0] < 25000 and -25000 < data[1] < 25000: 
            self.joystick.Stage.goabs_XY(data) 
        else:
            self.debugBar('Out of Range!!')

    def set_stage(self):

        command, yes = QInputDialog.getText(self, "Command","input:", QLineEdit.Normal, "")
        if yes:
            self.joystick.Stage.sent_command(command)

    def set_home(self):
        self.debugBar('Please define a new home [0,0] by mouse clicking!')

    def go_home(self):
        self.joystick.Stage.goabs_XY(self.home_pos)

    def load_model(self):    
        # load model
        model = torch.hub.load(self.yolov5_path,'custom',
                            path = self.model_path,
                            source = 'local')
        # set confidence threshold
        model.conf = 0.65

        return model         
    def start_tracking(self):
        self.cv_cal.droplets.clear()
        self.cv_cal.start()
        _translate = QtCore.QCoreApplication.translate
        self.Btn_detectandpredict.setEnabled(False)
        self.chk_show_circle.setEnabled(True)
        self.Btn_detectandpredict.setText(_translate("MainWindow", "Tracking"))
        self.Btn_endtracking.setEnabled(True)

    def end_tracking(self):
        _translate = QtCore.QCoreApplication.translate
        self.cv_cal.stop()
        self.Btn_detectandpredict.setEnabled(True)
        self.chk_show_circle.setEnabled(False)
        self.chk_show_circle.setChecked(False)
        self.Btn_detectandpredict.setText(_translate("MainWindow", "Detect + Predict"))
        self.Btn_endtracking.setEnabled(False)


class AnotherWindow(QWidget):
    """
    This "window" is a QWidget. If it has no parent, it
    will appear as a free-floating window as we want.
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("DMD Window")
        self.setGeometry(0, 0, 1920, 1080)
        self.setStyleSheet("background-color: black;")
        self.label=[]
        self.anim_group=[]
        self.index_activated_anim_group=0
        self.image = QPixmap('/home/mengting/test/Black.jpg')
       
    def keyPressEvent(self, e):  

        if e.key() == QtCore.Qt.Key_F11:
            if self.isFullScreen():
                self.showMaximized
                self.showNormal()
            else:
                self.showFullScreen()
        #if e.key() == QtCore.Qt.Key_Space:
        #    self.move_droplets()
    
    def move_droplets(self,path):
           
        self.label.append(QtWidgets.QLabel(self))    
        self.label[self.index_activated_anim_group].setStyleSheet("background-color:white")#;border-radius:2px;"
        self.label[self.index_activated_anim_group].move(path[0][0]-SPOT_SIZE//2,path[0][1]+SPOT_SIZE//2)
        #label.setText("{}-{}".format(pos.x(), pos.y()))
        self.label[self.index_activated_anim_group].resize(SPOT_SIZE, SPOT_SIZE)
        self.label[self.index_activated_anim_group].show()
        #anim = QPropertyAnimation(label, b"pos")
        self.anim = []
        
        #self.anim_group = QSequentialAnimationGroup()
        self.anim_group.append(QSequentialAnimationGroup(self))
        for i in range(len(path)-1):
            start=path[i]
            end=path[i+1]
            self.anim.append(QPropertyAnimation(self))
            self.anim[i].setTargetObject(self.label[self.index_activated_anim_group])
            self.anim[i].setPropertyName(b'pos')
            self.anim[i].setEndValue(QPoint(end[0]-SPOT_SIZE//2,end[1]+SPOT_SIZE//2))
            self.anim[i].setEasingCurve(QEasingCurve.Linear) #InOutCubic or Linear
            self.dist = hypot(end[0] - start[0], end[1]- start[1])
            self.duration=round(1000*self.dist/SPEED) # speed=pix/sec
            self.anim[i].setDuration(self.duration)
            self.anim_group[self.index_activated_anim_group].addAnimation(self.anim[i])
        
        self.anim_group[self.index_activated_anim_group].start()
        self.index_activated_anim_group +=1

    def paintEvent(self, e):
        self.painter = QPainter(self)
            
        self.painter.drawPixmap(self.rect(), self.image)

        #self.painter.setPen(QPen(Qt.red, 5, Qt.SolidLine))
        #self.painter.setBrush(QBrush(Qt.white, Qt.SolidPattern))
        #self.painter.drawRect(100, 100, 300, 100)

        self.painter.end()
 
class Camera(QtCore.QThread):
    rawdata = QtCore.pyqtSignal(np.ndarray)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.tlf = py.TlFactory.GetInstance()
        self.cam = py.InstantCamera(self.tlf.CreateFirstDevice())
        self.cam.Open()
        # setup for Basler Aca4096-30um area scan camera
        self.cam.BinningHorizontal = BINGING_FACTOR
        self.cam.BinningVertical = BINGING_FACTOR
        self.cam.Height = SCANLINE_HEIGHT
        self.cam.Width = SCANLINE_WIDTH
        self.cam.CenterX = True
        self.cam.CenterY = True
        self.cam.PixelFormat = "Mono8" 
        self.cam.Gain = 5 # 1-36
        self.cam.ExposureTime = 5000 # Unit : us
        self.cam.BinningHorizontalMode = "Sum" # Sum or Average
        self.cam.BinningVerticalMode = "Sum"
        print("Using device:", self.cam.GetDeviceInfo().GetModelName(), 
            " Resulting framerate:", round(self.cam.ResultingFrameRate.Value, 1),
            " Scan Resolution (HxV):", self.cam.Width.Value, "x", self.cam.Height.Value)
        self.cam.StartGrabbing()
        if self.cam is None:
            self.connect = False
            self.running = False
            self.writing = False
        else:
            self.connect = True
            self.running = False
            self.writing = False

    def run(self):
        
        if self.writing:
            self.writer= cv2.VideoWriter('/home/mengting/Videos/basicvideo1.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (1024,542))
        init_time = time.time()
        now_sec = 0
        while self.running and self.connect:
            
            img = np.ones((SCANLINE_HEIGHT, SCANLINE_WIDTH), dtype=np.uint8)
            with self.cam.RetrieveResult(2000) as result:
                if result.GrabSucceeded():
                    with result.GetArrayZeroCopy() as out_array:
                        img = out_array
                        self.rawdata.emit(img)
                else:
                    print("Error: ", result.ErrorCode, result.ErrorDescription)
                    self.connect = False 

            if self.writing:    
                
                time_stamp = self.secs_to_minsec(now_sec)
                img_1=cv2.resize(img,(1024,542), interpolation=cv2.INTER_LINEAR)
                img_rgb = cv2.cvtColor(img_1, cv2.COLOR_GRAY2BGR)
                #cv2.line(img_rgb, (900,480), (950,480), (255, 255, 255), 5)
                cv2.rectangle(img_rgb, (900,475), (950,480), (255, 255, 255), -1)
                cv2.putText(img_rgb, '200um', (900,500), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(img_rgb, time_stamp, (890,530), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                self.writer.write(img_rgb)
                now_sec = round(time.time() - init_time, 3)



   
    def open(self):
        if self.connect:
            self.running = True

    def record(self):
            self.writing = True

    def stop(self):
        if self.connect:
            self.running = False
            if self.writing:
                self.writer.release()
                self.writing = False
            

    def close(self):
        self.stop()
        time.sleep(1)
        self.cam.StopGrabbing()
        self.cam.close()

    def secs_to_minsec(self, sec):
        mins = int(sec // 60)
        secs = round(sec % 60)
        msecs= round(1000*(sec % 1))
        minsec = f'{mins:02}:{secs:02}.{msecs:03}'
        return minsec
    
class CV_calculation(QtCore.QThread):
    circle_img = QtCore.pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()

        self.img = np.zeros((SCANLINE_HEIGHT,SCANLINE_WIDTH),dtype=np.uint8)
        self.img_circle = np.zeros((SCANLINE_HEIGHT,SCANLINE_WIDTH,3),dtype=np.uint8)

        #---- image processing variables ----
        self.thresh_par1 = 45 # pixel neighborhood size (45*45 sub-region in the image to compute our threshold value)
        self.thresh_par2 = 20 # grayscale value to add or subtract
        self.iterations = 2 # dilation

        #---- circle detection parameters ----
        self.maxRadius = 38 # droplet max radius
        self.minRadius = 13 # droplet min radius
        self.dp = 1.3 # inverse ratio of the accumulator resolution to the image resolution
        self.circle_par1 = 100 # canny edge detection low threshold
        self.circle_par2 = 0.7 # Perfectness of circle (1 = perfect circle)
        self.extension_pixel = 5 # extend pixels from edge of droplets

        #---- droplets information (x_center, y_center, radius, cell_num) ----
        self.droplets = []

        self.operation_mode=0
        self.tracking = False
        self.drawing = False
        self.crop_img_time_total = 0
        self.pre_process_time_total = 0
        self.detect_circle_time_total = 0
        self.save_data_time_total = 0
        self.draw_circle_time_total = 0



    def run(self):
        self.tracking = True
        self.img_blr = self.pre_process(self.img)
        self.detectandpredict(self.img_blr)


            
    def get_img(self,image):
        self.img = image

    def get_model(self,model):
        self.model = model

    def pre_process(self,image):
        # # grayscale
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # apply binary thresholding
        thresh = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, self.thresh_par1, self.thresh_par2)

        # # detect the contours on the binary image
        # contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_CCOMP, method=cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(image=thresh, contours=contours, contourIdx=-1, color=(0, 0, 0), thickness=1, lineType=cv2.LINE_AA)

        # dilation
        el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        dil = cv2.dilate(thresh, el, iterations=self.iterations)

        # apply a blur using the Gaussian filter
        blr = cv2.GaussianBlur(dil,(3,3), cv2.BORDER_DEFAULT)

        return blr

    def detectandpredict(self,img):

        circles = cv2.HoughCircles(image=img, method=cv2.HOUGH_GRADIENT_ALT, dp=self.dp, minDist=15, 
                            param1=self.circle_par1, param2=self.circle_par2, minRadius=self.minRadius, maxRadius=self.maxRadius)

        # determine ROI
        if np.any(circles):
            circles = (np.rint(circles)).astype(int)

            for i in circles[0,:]:
                # i[2] = radius, (i[0],i[1])=center
                circle = [i[0], i[1], i[2], 0]
                self.droplets.append(circle)
            
        # predict and save result
        model = self.model

        result = []
        for i in range(len(self.droplets)):
            # determine roi
            roi = img[max(0,self.droplets[i][1]-self.droplets[i][2]-1):
                        min(img.shape[0],self.droplets[i][1]+self.droplets[i][2]+1),
                        max(0,self.droplets[i][0]-self.droplets[i][2]-1):
                        min(img.shape[1],self.droplets[i][0]+self.droplets[i][2]+1)]
            result.append(model(roi))

        df = []
        # Count cells appearing in each image
        for i in range(len(result)):
            df.append(pd.DataFrame(result[i].pandas().xyxy[0]))
        # df:  
        # xmin      ymin      xmax       ymax        confidence    class  name
        # 3.568027  3.287125  57.74757  57.158997    0.364773      0      cell

        for i in range(len(df)):
            count = self.count_cell(df[i])

            if count == 1:
                self.droplets[i][3] = 1
            elif count == 2:
                self.droplets[i][3] = 2
            elif count >= 2:
                self.droplets[i][3] = 3

    # Function of counting cells in one image
    def count_cell(self,df):
        if int(df["name"].count()) == 1:
            return 1
        elif int(df["name"].count()) == 2:
            return 2
        elif int(df["name"].count()) == 3:
            return 3
        elif int(df["name"].count()) == 4:
            return 4
        elif int(df["name"].count()) == 5:
            return 5
        elif int(df["name"].count()) == 6:
            return 6
        elif int(df["name"].count()) == 7:
            return 7
        elif int(df["name"].count()) == 8:
            return 8
        else:
            return 0


    def track(self,img):
        if self.tracking == True:
            self.img_circle = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
            for i in range(len(self.droplets)):

                self.crop_img_time_start = time.time()
                roi = img[max(0,self.droplets[i][1]-self.droplets[i][2]-self.extension_pixel):
                            min(img.shape[0],self.droplets[i][1]+self.droplets[i][2]+self.extension_pixel),
                            max(0,self.droplets[i][0]-self.droplets[i][2]-self.extension_pixel):
                            min(img.shape[1],self.droplets[i][0]+self.droplets[i][2]+self.extension_pixel)]
                self.crop_img_time_total = time.time() - self.crop_img_time_start

                self.pre_process_time_start = time.time()
                #roi_blr = self.pre_process(roi)
                #roi_blr = cv2.adaptiveThreshold(roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, self.thresh_par1, self.thresh_par2)
                roi_blr = cv2.GaussianBlur(roi,(5,5), cv2.BORDER_DEFAULT)

                self.pre_process_time_total = time.time() - self.pre_process_time_start

                self.detect_circle_time_start = time.time()
                circles = cv2.HoughCircles(image=roi_blr, method=cv2.HOUGH_GRADIENT_ALT, dp=self.dp, minDist=15, 
                                    param1=self.circle_par1, param2=self.circle_par2-0.1, minRadius=self.minRadius, maxRadius=self.maxRadius)
                self.detect_circle_time_total = time.time() - self.detect_circle_time_start

                self.save_data_time_start = time.time()
                if np.any(circles):
                    circles = (np.rint(circles)).astype(int)
                    self.droplets[i][0]= self.droplets[i][0]-self.droplets[i][2]-self.extension_pixel+circles[0][0][0]
                    self.droplets[i][1]= self.droplets[i][1]-self.droplets[i][2]-self.extension_pixel+circles[0][0][1]
                    self.droplets[i][2]= circles[0][0][2]
                self.save_data_time_total = time.time() - self.save_data_time_start

                self.draw_circle_time_start = time.time()
                if self.drawing == True:
                    if self.droplets[i][3] == 0:
                        cv2.circle(self.img_circle, (self.droplets[i][0],self.droplets[i][1]), self.droplets[i][2],(255,0,0),3)
                    elif self.droplets[i][3] == 1:
                        cv2.circle(self.img_circle, (self.droplets[i][0],self.droplets[i][1]), self.droplets[i][2],(0,255,0),3)
                    elif self.droplets[i][3] == 2:
                        cv2.circle(self.img_circle, (self.droplets[i][0],self.droplets[i][1]), self.droplets[i][2],(0,0,255),3)
                    elif self.droplets[i][3] == 3:
                        cv2.circle(self.img_circle, (self.droplets[i][0],self.droplets[i][1]), self.droplets[i][2],(255,255,0),3)        
                self.draw_circle_time_total = time.time() - self.draw_circle_time_start

            self.circle_img.emit(self.img_circle)
            #print("crop_img:" + str(self.crop_img_time_total))
            #print("pre_process:" + str(self.pre_process_time_total))
            #print("detect_circle:" + str(self.detect_circle_time_total))
            #print("save_data:" + str(self.save_data_time_total))
            #print("draw_circle:" + str(self.draw_circle_time_total))

    def drawCircles(self):
        self.img_circle = self.img
        for i in range(len(self.droplets)):            
            if self.droplets[i][3] == 0:
                cv2.circle(self.img_circle, (self.droplets[i][0],self.droplets[i][1]), self.droplets[i][2],(255,0,0),3)
            elif self.droplets[i][3] == 1:
                cv2.circle(self.img_circle, (self.droplets[i][0],self.droplets[i][1]), self.droplets[i][2],(0,255,0),3)
            elif self.droplets[i][3] == 2:
                cv2.circle(self.img_circle, (self.droplets[i][0],self.droplets[i][1]), self.droplets[i][2],(0,0,255),3)
            elif self.droplets[i][3] == 3:
                cv2.circle(self.img_circle, (self.droplets[i][0],self.droplets[i][1]), self.droplets[i][2],(255,255,0),3)
        return self.img_circle             

    def stop(self):
        self.tracking = False

if __name__=='__main__':
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
