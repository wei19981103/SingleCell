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