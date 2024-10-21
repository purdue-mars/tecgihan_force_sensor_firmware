import os, shutil, sys, ctypes, win32api
import glob, csv, errno, psutil
import ctypes.wintypes
import concurrent.futures as cf
import string, time, wx, math
import winsound 
#client = Client()  # start local workers as processes
# or
#client = Client(processes=False)  # start local workers as threads
#import pathos.multiprocessing as mp
import QSTMGraphicalAnalysisQ1 as qstmGA
from time import localtime,strftime
from QwarePaths import Q_DataBase
from pyqtgraph import ptime as t

import serial.tools.list_ports
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import * 
from PyQt4.QtCore import * 
import pyqtgraph.console
from pyqtgraph.dockarea import *
import numpy as np
import pyqtgraph as pg
##ProgramGraphPath = "F:\QSTM New Visualization\QSTMGraphicalAnalysis.py"
##devChkCond = False
##eventFlag = 0
##noDeviceFlag = 0
##RT_PlotInterrupt =0
##rstCnt = 0;self.tableParams =[] 
##RT_time=[]
##startTime=0.0;rstStrtTime=0.0;rstStopTime=0.0
##saveTime = 0.0
##lstRT_PlotInterrupt = 0
##UpdateSBValue =0
##UpdateLevelVal = 0
##connected = []; tableData = []
##csvWriteRow = []
##FrcX = [];FrcY= []; FrcZ=[]; FrcRes=[];FrcResAvgArr=[]; TargetArr=[]; TargetSetArr=[];
##YawG = [];PitchG=[]; RollG=[];
##YawS = [];PitchS=[]; RollS=[];
##AccRMS = []; GyroRMS = []; csvRow = [];
##ptr =0;cptr=0;

#-------------------File Storage and Temporary Memory Path------------------------

##CSIDL_PERSONAL= 5
##SHGFP_TYPE_CURRENT= 0
##buf= ctypes.create_unicode_buffer(ctypes.wintypes.MAX_PATH)
##ctypes.windll.shell32.SHGetFolderPathW(0, CSIDL_PERSONAL, 0, SHGFP_TYPE_CURRENT, buf)
##DefaultDestinationPath=''
##DefaultSearchPath = str(buf.value) 
###tempCsvPath =
##QSTM_Path = DefaultSearchPath+"\\QSTM" 
###storePath = QSTM_Path+"\\QSTM Patients"
##tempStorePath = QSTM_Path+"\\QSTM_Temp"
##rstDataStorePath = tempStorePath+"\\Subsession_Data"
##processDataStorePath = tempStorePath+"\\Process_Data"
###PatientFolderStorePath = QSTM_Path+"\\QSTM Patients Folder"
##
##if not os.path.exists(QSTM_Path):    
##    os.makedirs(QSTM_Path)
##else:
##    QSTM_Folders=os.walk(QSTM_Path).next()[1]
##
##if not os.path.exists(tempStorePath):    
##    os.makedirs(tempStorePath)
##else:
##    TempList=os.walk(tempStorePath).next()[1]
##
##if not os.path.exists(rstDataStorePath):    
##    os.makedirs(rstDataStorePath)
##else:
##    TempList=os.walk(rstDataStorePath).next()[1]
##
##if not os.path.exists(processDataStorePath):    
##    os.makedirs(processDataStorePath)
##else:
##    TempList=os.walk(processDataStorePath).next()[1]


#-------------------Misc codes-------------------------------------

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

#-----------------------Q1 visualization Class---------------------

class Q1_Visual_GUI(object):
    
    def setupUI (self, Frame):

        Frame.setObjectName(_fromUtf8("Frame"))
        Frame.resize(1438, 860)
        #Frame.setFrameShape(QtGui.QFrame.StyledPanel)
        #Frame.setFrameShadow(QtGui.QFrame.Raised)
        self.gridLayout = QtGui.QGridLayout(Frame)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))


        
        #MainWindow.setObjectName(_fromUtf8("MainWindow"))
        #MainWindow.resize(1438, 860)
        #self.centralWidget = QtGui.QWidget(MainWindow)
        #self.centralWidget.setObjectName(_fromUtf8("centralwidget"))

        self.area = DockArea()
        #MainWindow.setCentralWidget(self.area)
        self.gridLayout.addWidget(self.area, 0, 0, 1, 1)
        #win.resize(1200,700)
        Frame.setWindowTitle('Force-Motion Visualization')


        ###--------------------------Docks for different visual Widgets---------------
        self.d1 = Dock("Dock1 RealForce", size=(100, 100))     ## give this dock the minimum possible size
        self.d2 = Dock("Dock2 - Console", size=(100,20), closable=True)
        self.d3 = Dock("Dock3 Force Chart", size=(700,400))
        self.d4 = Dock("Dock4 Skin Angle Data", size=(500,200))
        self.d5 = Dock("Dock5 Geo Angle Data", size=(500,200))
        self.d6 = Dock("Dock6 Table", size=(500,200))

        self.area.addDock(self.d2, 'right')     ## place d2 at right edge of dock area
        self.area.addDock(self.d4, 'left')     ## place d4 at right edge of dock area
        self.area.addDock(self.d3, 'right', self.d4)## place d3 at center
        self.area.addDock(self.d5, 'bottom', self.d4)  ## place d5 at left edge of d1
        self.area.addDock(self.d6, 'bottom', self.d2)   ## place d5 at top edge of d4
        self.area.addDock(self.d1, 'bottom', self.d6)      ## place d1 at left edge of dock area (it will fill the whole space since there are no other docks yet)


        self.area.moveDock(self.d4, 'left', self.d3)     ## move d4 to top edge of d2
        self.area.moveDock(self.d6, 'bottom', self.d2)   ## move d6 to stack on top of d4
        self.area.moveDock(self.d5, 'bottom', self.d4)     ## move d5 to top edge of d2
                        
        ###-----------------------Setup Plots in widgets-------------------------------------

        self.FrcPlt=pg.PlotWidget(title=" Force Data ", enableMouse=False, enableMenu=False)
        self.FrcPlt.setMouseEnabled(x=False, y=False)
        self.FrcPlt.setBackgroundBrush(QtGui.QColor(250,250,255))
        self.FrcPlt.showGrid(y=True)
        self.labelStyle = {'color': '#576574', 'font-size': '12pt', 'font-style': 'Times New Roman'}
        #FrcPlt.setLabel('left',text="<span style='color: #ff0000; font-weight: bold; font-size: 12pt'>Force</span> <i>Axis</i>")
        self.FrcPlt.setLabel('left',"Force (Newtons)",**self.labelStyle)
        self.FrcPlt.setLabel('bottom',"Time(5-seconds)",**self.labelStyle)
        self.FrcLegend = self.FrcPlt.addLegend(offset=(0,0)) #pen=pg.mkPen(color=(128, 0, 0), width=2), fillLevel=UpdateLevelVal, fillBrush=(128,0,0,30)
        #FrcLegend.paint(p.setPen = 250, 250, 255)
        #FrcPlt.setRange(rect=None, xRange=None, yRange=(-5.0,5.0), padding=None, update=True, disableAutoRange=False)
        self.d3.addWidget(self.FrcPlt)
        self.d3.hideTitleBar()
        ##win.nextColumn()
        self.SkinAPlt=pg.PlotWidget(title=" Skin Angle Data ", enableMouse=False, enableMenu=False)
        self.SkinAPlt.setMouseEnabled(x=False, y=False)
        self.SkinAPlt.setBackgroundBrush(QtGui.QColor(250,250,255))
        self.SkinAPlt.showGrid(y=True)
        self.d4.addWidget(self.SkinAPlt)
        self.d4.hideTitleBar()
        ##win.nextRow()
        self.GeoAPlt=pg.PlotWidget(title=" Geo Angle Data ", enableMouse=False, enableMenu=False)
        self.GeoAPlt.setMouseEnabled(x=False, y=False)
        self.GeoAPlt.setBackgroundBrush(QtGui.QColor(250,250,255))
        self.GeoAPlt.showGrid(y=True)
        self.GeoAPlt.addLegend(offset=(0,0))
        self.d5.addWidget(self.GeoAPlt)
        self.d5.hideTitleBar()
        #win.nextColumn()

        ###-----------------------Setup TABLE for SubSession Data----------------------------------------------
        self.table=QtGui.QTableWidget()
        # initiate table
        self.table.resize = (700,700)
        self.table.setRowCount(12)
        self.table.setColumnCount(2)
        #table.setBackgroundBrush(QtGui.QColor(250,250,255))
        # set label
        self.table.setHorizontalHeaderLabels(QString("VALUE;DESCRIPTION;").split(";"))
        self.table.setVerticalHeaderLabels(QString("STATUS;Avg Comp Force;Avg Rez Force;Max Peak;Avg Peak;No. of Burst;No. of Strokes;Strokes Frequency;Avg Pitch;Avg Roll;Avg Yaw;Time elapsed;").split(";"))      
        self.tableValList = ["WAIT","0","0","0","0","0","0","0","0","0","0","0"]
        self.tableDescList = ["Device State","Newtons","Newtons","Newtons","Newtons"," "," ","Hz","Degrees","Degrees","Degrees","Seconds"]
        ## set data
        for tableRow in range(0,12):
            self.table.setItem(tableRow,0, QTableWidgetItem(self.tableValList[tableRow]))  
            self.table.setItem(tableRow,1, QTableWidgetItem(self.tableDescList[tableRow]))
        self.d6.addWidget(self.table)
        self.d6.hideTitleBar()
        '''tableData = np.array([
            (1,'STATUS',   'WAIT' ,  'Device State'),
            (2,'Avg Comp Force',   0,   'Newtons'),
            (3,'Avg Rez Force',   0,   'Newtons'),
            (4,'Max Peak',   0,   'Newtons'),
            (5,'Avg Peak',   0,   'Newtons'),
            (6,'No. of Burst',   0,   ' '),
            (7,'No. of Strokes',   0,   ' '),
            (8,'Strokes Frequency',   0,  'Hz'),
            (9,'Avg Yaw',   0,   'Degrees'),
            (10,'Avg Pitch',   0,   'Degrees'),
            (11,'Avg Roll',   0,   'Degrees'),
            (12,'Time elapsed', 0, 'Seconds'),
            ], dtype=[('#', int),('NAME', object), ('VALUE', object), ('DESCRIPTION', object)])
        #w.setSortMode(column = 1, mode = 3)'''
            

        #####-----------------------Setup Text for realtime Force Monitoring-------------------------------------
        ##forceText=pg.GraphicsView()
        ##forceText.setBackgroundBrush(QtGui.QColor(0,0,0))
        ##vb = pg.ViewBox()
        ###vb.setRange(rect = QtCore.QRectF(0,0,100,50))
        ##forceText.setCentralWidget(vb)
        ##
        ##vb.setAspectLocked()
        ###vb.setRange(rect = QtCore.QRectF(10, 10, 100, 50))
        ##forceText.setCentralWidget(vb)
        ###forceText.setRange(rect=None, xRange=None, yRange=(-5.0,5.0), padding=None, update=True, disableAutoRange=False)
        ###text = pg.TextItem(html='<div style="text-align: center"><span style="color: #FFF;">This is the</span><br><span style="color: #FF0; font-size: 20pt;">PEAK</span></div>', border='w', fill=(100, 100, 255, 100))
        ##text = pg.TextItem(text = "Resultant Force", color=(200, 200, 200), anchor=(0,0))
        ###serifFont = QtGui.QFont("Times", 10, QFont.Bold)
        ##text.setFont(QtGui.QFont("Times", 20, QtGui.QFont.Bold))
        ###forceText.setCentralItem(text)
        ##vb.addItem(text)
        ##vb.setRange(rect = vb.itemBoundingRect(text))
        ##d1.addWidget(forceText)
        ##d1.hideTitleBar()


        ###-----------------------Setup Text for realtime Force Monitoring Tansformed-------------------------------------
        self.forceText=pg.GraphicsView()
        self.forceText.setBackgroundBrush(QtGui.QColor(255,255,255))
        self.forceText.resize(100, 100)
        self.gridLayout = QtGui.QGridLayout(self.forceText)

        spacerItem = QtGui.QSpacerItem(20, 50, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.gridLayout.addItem(spacerItem, 2, 1, 1, 1)
        spacerItem1 = QtGui.QSpacerItem(20, 50, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.gridLayout.addItem(spacerItem1, 3, 2, 1, 1)


        self.ForceLabel = QtGui.QLabel(self.forceText)
        self.ForceLabel.setFont(QtGui.QFont("Times", 20, QtGui.QFont.Bold))
        self.ForceLabel.setText("Resultant Force")
        #ForceLabel.setObjectName(_fromUtf8("ForceMonitor"))
        self.gridLayout.addWidget(self.ForceLabel, 2, 2, 1, 1)
        spacerItem2 = QtGui.QSpacerItem(60, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.gridLayout.addItem(spacerItem2, 2, 3, 1, 1)

        self.spinBox = QtGui.QSpinBox(self.forceText)
        #spinBox.setObjectName(_fromUtf8("spinBox"))
        self.gridLayout.addWidget(self.spinBox, 0, 1, 1, 1)
        #spinBox.valueChanged.connect(UpdateSpBoxValue)

        spacerItem3 = QtGui.QSpacerItem(60, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.gridLayout.addItem(spacerItem3, 1, 2, 1, 1)

        #setButton = QtGui.QPushButton(forceText)
        #setButton.setText("SET Target")
        #setButton.setObjectName(_fromUtf8("setButton"))
        #gridLayout.addWidget(setButton, 0, 2, 1, 1)

        self.targetLabel = QtGui.QLabel(self.forceText)
        self.targetLabel.setFont(QtGui.QFont("Times", 12, QtGui.QFont.Bold))
        self.targetLabel.setText("Target Force - ")
        #targetLabel.setObjectName(_fromUtf8("label"))
        self.gridLayout.addWidget(self.targetLabel, 0, 2, 1, 1)
        
        self.d1.addWidget(self.forceText)
        self.d1.hideTitleBar()

        ###-----------------------Setup Timer for Q1 Data Read and Write-----------------------

        self.Q1timer = QtCore.QTimer()
        self.Q1timer.timeout.connect(self.UpdateExec)


        ###-----------------------Setup Plot Configuration-------------------------------------
        pg.setConfigOptions(antialias=True)

        self.FrcTrgCurve=self.FrcPlt.plot(pen=pg.mkPen(color=(128, 0, 0), width=2), fillLevel=0, fillBrush=(128,0,0,30))
        #FrcTrgSetCurve=FrcPlt.plot(pen=(250,250,255), fillLevel=UpdateLevelVal, fillBrush=(128,0,0,20))

        #FrcTrgRange = FrcPlt.LinearRegionItem(values=(0, 1), orientation='horizontal', brush=(128,0,0,30), pen=(255,200,255))

        self.FrcXCurve=self.FrcPlt.plot(pen=(255,0,0), name="Force X")
        self.FrcYCurve=self.FrcPlt.plot(pen=(0,155,0), name="Force Y")
        self.FrcZCurve=self.FrcPlt.plot(pen=(0,0,255), name="Force Z")
        self.FrcResCurve=self.FrcPlt.plot(pen=(0,0,0), name="Rez Force")


        #FrcResCurve=SkinAPlt.plot(pen=(0,0,0))

        self.GeoYCurve=self.GeoAPlt.plot(pen=(255,0,0), name="Geo-Yaw")
        self.GeoPCurve=self.GeoAPlt.plot(pen=(0,155,0), name="Geo-Pitch")
        self.GeoRCurve=self.GeoAPlt.plot(pen=(0,0,255), name="Geo-Roll")


        self.SkinYCurve = self.SkinAPlt.plot(pen=(255,0,0))
        self.SkinPCurve = self.SkinAPlt.plot(pen=(0,155,0))
        self.SkinRCurve = self.SkinAPlt.plot(pen=(0,0,255))

        
    def SystemInitQ1(self, serialComm):
        #global startTime,ofile,rstTimeFile,mainCSVwriter,csvPath,storeRST_FilePath,rstFile,rstFileWriter
        #TempSubsessionClear()
        #SerialCommInit()
        #csvPath = self.storeCSVpath

        #self.Q1timer = QtCore.QTimer()
        #self.Q1timer.timeout.connect(self.UpdateExec)

        ProgramGraphPath = "F:\QSTM New Visualization\QSTMGraphicalAnalysis.py"
        devChkCond = False
        eventFlag = 0
        noDeviceFlag = 0
        self.RT_PlotInterrupt =0
        rstCnt = 0;self.tableParams =[] 
        self.RT_time=[]
        self.startTime=0.0;rstStrtTime=0.0;rstStopTime=0.0
        self.saveTime = 0.0
        self.lstRT_PlotInterrupt = 0
        self.UpdateSBValue =0
        self.UpdateLevelVal = 0
        connected = []; tableData = []
        csvWriteRow = []
        self.FrcX = [];self.FrcY= []; self.FrcZ=[]; self.FrcRes=[];self.FrcResAvgArr=[]; self.TargetArr=[]; self.TargetSetArr=[];
        self.YawG = [];self.PitchG=[]; self.RollG=[];
        self.YawS = [];self.PitchS=[]; self.RollS=[];
        self.AccRMS = []; self.GyroRMS = []; self.csvRow = [];
        self.ptr =0;self.cptr=0;
        self.serialComm = serialComm
        self.DevState = -1; self.lastDevState=-1; self.TreatSt = -1; self.fps=None;
        self.rstStopTime = 0.01000
        self.StopTimeArr=[]
        self.StartTimeArr=[]

        self.TreatButtonCnt=-1; self.ResetButtonCnt=-1; self.IntervalButtonCnt=-1;
        self.db = Q_DataBase()

        #self.Curves=[self.GeoYCurve, self.SkinYCurve, self.GeoPCurve, self.SkinPCurve, self.GeoRCurve, self.SkinRCurve, self.FrcXCurve, self.FrcYCurve, self.FrcZCurve, self.FrcResCurve, self.FrcTrgCurve]
        #self.CurveArr = [self.YawG, self.YawS, self.PitchG, self.PitchS, self.RollG, self.RollS, self.FrcX, self.FrcY, self.FrcZ,self.FrcRes, self.TargetArr]



        
        self.Local_time=strftime("%H-%M-%S_%m-%d-%Y",localtime())
        self.tempStorePath=self.db.tempStorePath

        self.rstDataStorePath = self.db.rstDataStorePath
        self.processDataStorePath = self.db.processDataStorePath
        
        self.storeCSVpath=self.tempStorePath+"\\"+"_Q1RawOutputChart_"+self.Local_time+".csv"
        self.resultCSVpath=self.tempStorePath+"\\"+"_Q1ResultChart_"+self.Local_time+".csv"
        self.processCSVpath = self.processDataStorePath+"\\"+"ProcessQ1data"+".csv"
        self.rstTimeCSVpath = self.rstDataStorePath+"\\"+"RstQ1TimeData"+".csv"
        self.lstRT_PlotInterrupt = 0
        while True:
            while (self.serialComm.inWaiting()==1):
                pass
            dataStream = self.serialComm.readline()
            self.dataString = str(dataStream)
            if (self.lstRT_PlotInterrupt == 0):
                self.serialComm.write(str("ACK"))
                self.lstRT_PlotInterrupt = 4
            
                   
                        
            #readVal = int(arduStrng)
            if self.dataString.startswith('Palpation Device is ready'):
                print ("Graph Data Started")
                time.sleep(1)
                self.RT_PlotInterrupt=1
                self.startTime=time.time()
                t_now= time.time()-self.startTime
                
                #storeRST_FilePath=rstDataStorePath+"\\"+"RstQ1Chart_0_"+str(t_now)+".csv"
                #rstFile = open(storeRST_FilePath, 'wb')
                #rstFileWriter = csv.writer(rstFile, delimiter=',')
                
                #labelRow = ['Time','ForceX','ForceY','ForceZ','ForceRez','YawG','PitchG','RollG','YawS','PitchS','RollS']
                labelRow = ['Time','ForceX','ForceY','ForceZ','ForceRez','YawG','PitchG','RollG','AccelX','AccelY','AccelZ','AccRMS','RST','GyroRMS','GyroX','GyroY', 'GyroZ','MagX','MagY','MagZ']            
                self.mainCSVFile  = open(self.storeCSVpath, 'wb')
                self.mainCSVwriter = csv.writer(self.mainCSVFile, delimiter=',')
                self.mainCSVwriter.writerow(labelRow)
                
                self.rstTimeFile  = open(self.rstTimeCSVpath, 'wb')
                self.rstTimeFile.close()
                processPid = os.getpid()
                processRow = [processPid,self.storeCSVpath,self.resultCSVpath]
                processFile = open(self.processCSVpath, 'wb')
                processFileWriter = csv.writer(processFile, delimiter=',')
                processFileWriter.writerow(processRow)
                processFile.close()
                print("System Initaialization Done")                
                #self.Q1timer.start(1)
            #elif self.dataString.startswith('T:Q1'): #Start Device
            elif self.dataString.startswith('Start Device'):
                self.lastNow=t.time()
                
                break
            print (self.dataString) 

    def ReadDataStream(self):
        #global dataString,serialElements,RT_PlotInterrupt;
        #global lstRT_PlotInterrupt,rstStrtTime,rstStopTime,qstmGA; 
        #global rstCnt,storeRST_FilePath,rstFile,rstFileWriter;
       
         
        #while (self.serialComm.inWaiting()==0):
         #   pass

        if (self.serialComm.inWaiting()==0) and (self.DevState<0):
            self.dataStream= ("N:Q1_0.000,No Data")
        else:
            while(self.serialComm.inWaiting()==2):
                self.dataStream=""
                #self.dataString = str(dataStream)
                return
                #continue
            self.dataStream = self.serialComm.readline()
        self.dataString = str(self.dataStream)
        self.serialElements = self.dataString.split(",")
        #serFirstElements = serialElements[0].split("_")
        
        modeElements = (self.serialElements[0]).split("_")
        #self.modeTimeStamp = modeElements[1]
        #self.modeState,self.DeviceID = (modeElements[0]).split(":")
        
        if (self.dataString.startswith('T:Q1')):#(self.modeState is "T"):
            self.RT_PlotInterrupt =1
            self.modeTimeStamp = modeElements[1]
            self.modeState,self.DeviceID = (modeElements[0]).split(":")
            self.TreatButtonCnt = ((int)(self.serialElements[12]))
        elif (self.dataString.startswith('R:Q1')):#(self.modeState is "R"):
            self.RT_PlotInterrupt =2
            self.modeTimeStamp = modeElements[1]
            self.modeState,self.DeviceID = (modeElements[0]).split(":")
            resetString = self.serialElements[1].split(" ")
            self.ResetButtonCnt = ((int)(resetString[len(resetString)-1]))
        elif (self.dataString.startswith('I:Q1')):#(self.modeState is "I"):
            self.RT_PlotInterrupt =3
            self.modeTimeStamp = modeElements[1]
            self.modeState,self.DeviceID = (modeElements[0]).split(":")
            resetString = self.serialElements[1].split(" ")
            self.IntervalButtonCnt = ((int)(resetString[len(resetString)-1]))

        if (self.IntervalButtonCnt>self.TreatButtonCnt):
            self.DevState = 1
        elif(self.IntervalButtonCnt<self.TreatButtonCnt):
            self.DevState = 0
            

            
        #if(self.RT_PlotInterrupt>self.lstRT_PlotInterrupt) and (self.IntervalButtonCnt>self.TreatButtonCnt):
        #if (self.IntervalButtonCnt>self.TreatButtonCnt):
        if (self.DevState>self.lastDevState) and (self.lastDevState >=0):
##            #rstStrtTime = (float)(modeElements[1])
##            print("Reset start timeStamp",(rstStrtTime))       
            
            
            print(" Wait")         
            
            ###----------- write reset row data and put update Table here-------------------
            if(self.TreatSt == 0):
                self.rstStrtTime = (float)(modeElements[1])
                self.rst = 1
                self.Ts = self.modeTimeStamp            
                self.csvRow = []
                self.csvRow.append((float)(self.Ts))
                self.csvRow.append(self.FrcX[499])
                self.csvRow.append(self.FrcY[499])
                self.csvRow.append(self.FrcZ[499])
                self.csvRow.append(self.FrcRes[499])
                self.csvRow.append(self.YawG[499])
                self.csvRow.append(self.PitchG[499])
                self.csvRow.append(self.RollG[499])
                self.csvRow.append(self.YawS[499])
                self.csvRow.append(self.PitchS[499])
                self.csvRow.append(self.RollS[499])
                self.csvRow.append(self.AccRMS[499])
                self.csvRow.append(self.rst)
                self.csvRow.append(self.GyroRMS[499])
                self.csvRow.append(self.gX)
                self.csvRow.append(self.gY)
                self.csvRow.append(self.gZ)
                self.csvRow.append(self.mX)
                self.csvRow.append(self.mY)
                self.csvRow.append(self.mZ)
                #self.mainCSVwriter.writerow(self.csvRow)
                if (self.mainCSVFile.closed == True):
                    self.mainCSVFile  = open(self.storeCSVpath, 'ab')
                    self.mainCSVwriter = csv.writer(self.mainCSVFile, delimiter=',')
                    self.mainCSVwriter.writerow(self.csvRow)
                    self.mainCSVFile.close()
                else:
                    self.mainCSVwriter.writerow(self.csvRow)
                    self.mainCSVFile.close()
                    print ("Written in File")
                
                self.tableParams = qstmGA.TableDataCalculation(self.storeCSVpath)
            elif (self.TreatSt > 0):
                try:
                    self.csvRow = [(float)(self.modeTimeStamp),self.Fx,self.Fy,self.Fz,self.Fres,self.Yg,self.Pg,self.Rg,self.Ys,self.Ps,self.Rs,self.aRms,self.rst,self.gRms,self.gX,self.gY,self.gZ,self.mX,self.mY,self.mZ]
                    self.mainCSVwriter.writerow(self.csvRow)
                    for tt in range (1,25):
                        self.Ts = (float)(self.modeTimeStamp)+(0.01*tt)
                        if tt<10:                            
                            self.Fx = self.Fx - ((tt*0.1)*self.Fx)
                            self.Fy = self.Fy - ((tt*0.1)*self.Fy)
                            self.Fz = self.Fz - ((tt*0.1)*self.Fz)
                            Fsum = pow(self.Fx,2) + pow(self.Fy,2) + pow(self.Fz,2)
                            self.Fres = math.sqrt(Fsum)
                        else:                            
                            self.Fx = self.Fx - (tt*0.001*self.Fx)
                            self.Fy = self.Fy - (tt*0.001*self.Fy)
                            self.Fz = self.Fz - (tt*0.001*self.Fz)
                            Fsum = pow(self.Fx,2) + pow(self.Fy,2) + pow(self.Fz,2)
                            self.Fres = math.sqrt(Fsum)
                        self.csvRow = [self.Ts,round(self.Fx,2),round(self.Fy,2),round(self.Fz,2),round(self.Fres,2),self.Yg,self.Pg,self.Rg,self.Ys,self.Ps,self.Rs,self.aRms,self.rst,self.gRms,self.gX,self.gY,self.gZ,self.mX,self.mY,self.mZ]
                        self.mainCSVwriter.writerow(self.csvRow)
                    self.csvRow = [self.Ts+0.01,round(self.Fx,2),round(self.Fy,2),round(self.Fz,2),round(self.Fres,2),self.Yg,self.Pg,self.Rg,self.Ys,self.Ps,self.Rs,self.aRms,1,self.gRms,self.gX,self.gY,self.gZ,self.mX,self.mY,self.mZ]
                    self.mainCSVwriter.writerow(self.csvRow)
                    self.mainCSVFile.close()                   
                    
                except ValueError or IOError :
                    self.mainCSVFile  = open(self.storeCSVpath, 'ab')
                    self.mainCSVwriter = csv.writer(self.mainCSVFile, delimiter=',')
                    
                    self.csvRow = [(float)(self.modeTimeStamp),self.Fx,self.Fy,self.Fz,self.Fres,self.Yg,self.Pg,self.Rg,self.Ys,self.Ps,self.Rs,self.aRms,self.rst,self.gRms,self.gX,self.gY,self.gZ,self.mX,self.mY,self.mZ]
                    self.mainCSVwriter.writerow(self.csvRow)
                    for tt in range (1,25):                        
                        self.Ts = (float)(self.modeTimeStamp)+(0.01*tt)
                        if tt<10:                            
                            self.Fx = self.Fx - ((tt*0.1)*self.Fx)
                            self.Fy = self.Fy - ((tt*0.1)*self.Fy)
                            self.Fz = self.Fz - ((tt*0.1)*self.Fz)
                            Fsum = pow(self.Fx,2) + pow(self.Fy,2) + pow(self.Fz,2)
                            self.Fres = math.sqrt(Fsum)
                        else:                            
                            self.Fx = self.Fx - (tt*0.001*self.Fx)
                            self.Fy = self.Fy - (tt*0.001*self.Fy)
                            self.Fz = self.Fz - (tt*0.001*self.Fz)
                            Fsum = pow(self.Fx,2) + pow(self.Fy,2) + pow(self.Fz,2)
                            self.Fres = math.sqrt(Fsum)
                        self.csvRow = [self.Ts,round(self.Fx,2),round(self.Fy,2),round(self.Fz,2),round(self.Fres,2),self.Yg,self.Pg,self.Rg,self.Ys,self.Ps,self.Rs,self.aRms,self.rst,self.gRms,self.gX,self.gY,self.gZ,self.mX,self.mY,self.mZ]
                        self.mainCSVwriter.writerow(self.csvRow)
                    self.csvRow = [self.Ts+0.01,round(self.Fx,2),round(self.Fy,2),round(self.Fz,2),round(self.Fres,2),self.Yg,self.Pg,self.Rg,self.Ys,self.Ps,self.Rs,self.aRms,1,self.gRms,self.gX,self.gY,self.gZ,self.mX,self.mY,self.mZ]
                    self.mainCSVwriter.writerow(self.csvRow)                    
                    self.mainCSVFile.close()
                    
                self.rstStrtTime = self.Ts+0.01
                self.tableParams = ["Wrong Press","N/A","N/A","N/A","N/A","N/A","N/A","N/A","N/A","N/A","N/A"]

                self.ErrorMsg = self.showMsgDialog()
           
            #self.tableParams =[]
            rstTimeRow = [round(self.rstStopTime,3),round(self.rstStrtTime,3),self.storeCSVpath]
            self.rstTimeFile  = open(self.rstTimeCSVpath, 'ab')
            rstTimeWriter = csv.writer(self.rstTimeFile, delimiter=',')
            rstTimeWriter.writerow(rstTimeRow)
            self.rstTimeFile.close()
            #self.mainCSVFile.close()

            self.StartTimeArr.append(round(self.rstStopTime,2))
            self.StopTimeArr.append(round(self.rstStrtTime,2))
                
            print(len(self.tableParams))
            if (len(self.tableParams))>3:
                self.tableParams.insert(0,"WAITING")                 
            else:
                self.tableParams = ["No Activity","0","0","0","0","0","0","0","0","0","0","0"]
            for Row in range(0,12):
                self.table.setItem(Row,0, QTableWidgetItem(str(self.tableParams[Row])))
            
            #import QSTMGraphicalAnalysis as qstmGA
            ###--------------end update table here----------------
            #storeRST_FilePath=rstDataStorePath+"\\RstQ1Chart_"+str(rstCnt)+"_"+str(round(rstStrtTime,2))+".csv"        
            #rstFile = open(storeRST_FilePath, 'wb')
            #rstFileWriter = csv.writer(rstFile, delimiter=',')
        #elif(self.RT_PlotInterrupt<self.lstRT_PlotInterrupt) and ((self.IntervalButtonCnt<self.TreatButtonCnt)):
        #elif (self.IntervalButtonCnt<self.TreatButtonCnt):
        elif (self.DevState<self.lastDevState):
            print("Q1 Go")
            

            #print ("TreatmentButtonCount",(self.TreatButtonCnt))
            #print ("ResetButtonCount",(self.ResetButtonCnt))
            #print ("IntervalButtonCount",(self.IntervalButtonCnt))
            
            #self.TreatButtonCnt=0; self.ResetButtonCnt=0; self.IntervalButtonCnt=0;

           # labelRow = [' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','1',' ']

            self.mainCSVFile  = open(self.storeCSVpath, 'ab')
            self.mainCSVwriter = csv.writer(self.mainCSVFile, delimiter=',')
            self.mainCSVwriter.writerow(self.csvRow)

            self.rstStopTime = (float)(modeElements[1])
            
            #print("Reset stop timeStamp",(self.rstStopTime))
            
            self.tableParams =[]
            self.tableValList[0]="Ready"
            for Row in range(0,12):
                self.table.setItem(Row,0, QTableWidgetItem(self.tableValList[Row]))
            
        
        self.lstRT_PlotInterrupt = self.RT_PlotInterrupt
        self.lastDevState = self.DevState
        #serialElements[0] = modeTimeStamp    
        #return serialElements
##
##
    def storeData(self):
        #global saveTime, csvRow, cptr, Ts, rst, TargetArr
        rst = 1
        self.ReadDataStream()

        if (len(self.dataStream)<=0):
            return
        
        #Ts = Elements[0]

        if (self.DevState<0):           
            self.Ts = (float)((-360000.0+self.cptr)/100.0)
            self.Fx=0.0; self.Fy=0.0; self.Fz=0.0; self.Fres=0.0;
            self.Yg = 0.0; self.Pg = 0.0;  self.Rg = 0.0;
            self.Ys = 0.0; self.Ps = 0.0;  self.Rs = 0.0;
            self.aRms = 0.0; self.rst = -1; self.gRms = 0.0;
            self.gX=0.0; self.gY=0.0; self.gZ=0.0;
            self.mX=0.0; self.mY=0.0; self.mZ=0.0;

            self.FrcX.append(self.Fx);self.FrcY.append(self.Fy);
            self.FrcZ.append(self.Fz);self.FrcRes.append(self.Fres);
            self.YawG.append(self.Yg);self.PitchG.append(self.Pg);self.RollG.append(self.Rg);
            self.YawS.append(self.Ys);self.PitchS.append(self.Ps);self.RollS.append(self.Rs);
            self.AccRMS.append(self.aRms); self.GyroRMS.append(self.gRms); self.RT_time.append(self.Ts)

            if (self.cptr==500):
                saveTime = float(str(self.Ts))
                print("Q1 Recess{}".format(saveTime))
                #print(ptr,cptr)
            if (self.cptr>500):
                self.FrcX.pop(0); self.FrcY.pop(0); self.FrcZ.pop(0); self.FrcRes.pop(0);
                self.YawG.pop(0); self.PitchG.pop(0); self.RollG.pop(0)
                self.YawS.pop(0); self.PitchS.pop(0); self.RollS.pop(0)
                self.AccRMS.pop(0);  self.GyroRMS.pop(0); self.RT_time.pop(0)
                #self.TargetArr = (np.ones(500))* self.UpdateSBValue

            if (self.RT_PlotInterrupt ==2):
                #print("Treatment Session On")
                self.DevState=0
                #self.cptr=0


            self.cptr+=1
       
        
        #print(Elements)
        #self.storeCSVpath=tempStorePath+"\\"+"_TempQ1Chart_"+self.Local_time+".csv"
        if (self.RT_PlotInterrupt==1) and (self.DevState==0):
            self.Ts = self.modeTimeStamp
            #t = time.time()-startTime
            self.Fx = (float)(self.serialElements[1])
            self.Fy = (float)(self.serialElements[2])
            self.Fz = (float)(self.serialElements[3])
            self.Fres = (float)(self.serialElements[4])
            self.Yg = (float)(self.serialElements[5])
            self.Pg = (float)(self.serialElements[6])
            self.Rg = (float)(self.serialElements[7])
            self.Ys = (float)(self.serialElements[8])
            self.Ps = (float)(self.serialElements[9])
            self.Rs = (float)(self.serialElements[10])
            self.aRms = (float)(self.serialElements[11])
            self.rst = ((int)(self.serialElements[12]))%2
            self.gRms = (float)(self.serialElements[13])
            self.gX = (float)(self.serialElements[14])
            self.gY = (float)(self.serialElements[15])
            self.gZ = (float)(self.serialElements[16])
            self.mX = (float)(self.serialElements[17])
            self.mY = (float)(self.serialElements[18])
            self.mZ = (float)(self.serialElements[19])
            #rst = ((int)(Elements[23]))%2
            self.FrcX.append(self.Fx);self.FrcY.append(self.Fy);
            self.FrcZ.append(self.Fz);self.FrcRes.append(self.Fres);
            self.YawG.append(self.Yg);self.PitchG.append(self.Pg);self.RollG.append(self.Rg);
            self.YawS.append(self.Ys);self.PitchS.append(self.Ps);self.RollS.append(self.Rs);
            self.AccRMS.append(self.aRms); self.GyroRMS.append(self.gRms); self.RT_time.append(self.Ts)
            #FrcResAvg = getAvgGauss(FrcRes,25);FrcResAvgArr.append(FrcResAvg)
            #print(cptr)
            self.csvRow = [self.Ts,self.Fx,self.Fy,self.Fz,self.Fres,self.Yg,self.Pg,self.Rg,self.Ys,self.Ps,self.Rs,self.aRms,self.rst,self.gRms,self.gX,self.gY,self.gZ,self.mX,self.mY,self.mZ]
            self.mainCSVwriter.writerow(self.csvRow) 
            #rstFileWriter.writerow(csvRow)
            if (self.cptr==500):
                saveTime = float(str(self.Ts))
                print("Q1 Treat{}".format(saveTime))
                #print(ptr,cptr)
            if (self.cptr>500):
                #csvRow = [round(RT_time[0],3),FrcX[0],FrcY[0],FrcZ[0],FrcRes[0],YawG[0],PitchG[0],RollG[0],YawS[0],PitchS[0],RollS[0],AccRMS[0]]
                ##ofile  = open(self.storeCSVpath, "a")
                ##writer = csv.writer(ofile, delimiter=',')
                ##writer.writerow(csvRow)
                ##ofile.close()
                if(self.Fres>0.8):
                    self.table.setItem(0,0, QTableWidgetItem("Working"))
                    self.TreatSt = 1
                else:
                    self.table.setItem(0,0, QTableWidgetItem("Ready"))
                    self.TreatSt = 0
                self.FrcX.pop(0)
                self.FrcY.pop(0)
                self.FrcZ.pop(0)
                self.FrcRes.pop(0)
                self.YawG.pop(0)
                self.PitchG.pop(0)
                self.RollG.pop(0)
                self.YawS.pop(0)
                self.PitchS.pop(0)
                self.RollS.pop(0)
                self.AccRMS.pop(0)
                self.GyroRMS.pop(0)
                self.RT_time.pop(0)
                self.TargetArr = (np.ones(500))* self.UpdateSBValue
                #TargetSetArr = (np.ones(500))*((UpdateSBValue)+(0.05*UpdateSBValue))
                #FrcResAvgArr.pop(0)
                #FrcRes.pop(0);
                #RT_time.pop(0)
    ##        if (len(RT_time)>600):
    ##            RT_time.pop(0)
            self.cptr+=1
        elif (self.RT_PlotInterrupt>=2) and (self.DevState==1):

            #self.rst = ((int)(self.serialElements[12]))%2
            self.rst = 1

            #print(" Device state change RST {}".format(self.rst))
            if (self.TreatSt == 0):                
            
                
                self.Ts = self.modeTimeStamp            
                self.csvRow = []
                self.csvRow.append((float)(self.Ts))
                self.csvRow.append(self.FrcX[499])
                self.csvRow.append(self.FrcY[499])
                self.csvRow.append(self.FrcZ[499])
                self.csvRow.append(self.FrcRes[499])
                self.csvRow.append(self.YawG[499])
                self.csvRow.append(self.PitchG[499])
                self.csvRow.append(self.RollG[499])
                self.csvRow.append(self.YawS[499])
                self.csvRow.append(self.PitchS[499])
                self.csvRow.append(self.RollS[499])
                self.csvRow.append(self.AccRMS[499])
                self.csvRow.append(self.rst)
                self.csvRow.append(self.GyroRMS[499])
                self.csvRow.append(self.gX)
                self.csvRow.append(self.gY)
                self.csvRow.append(self.gZ)
                self.csvRow.append(self.mX)
                self.csvRow.append(self.mY)
                self.csvRow.append(self.mZ)
            else:
                self.csvRow = [(float)(self.modeTimeStamp),0.01,0.01,0.01,0.01,self.Yg,self.Pg,self.Rg,self.Ys,self.Ps,self.Rs,self.aRms,self.rst,self.gRms,self.gX,self.gY,self.gZ,self.mX,self.mY,self.mZ]

            #print(self.csvRow)
            
            #print("FRC-Y")
            #print (self.FrcY[499])
            #print("FRC-Y")
            #print (self.FrcY[499])
            #self.csvRow = [(float)(Ts),self.FrcX[499],self.FrcY[499],self.FrcZ[499],self.FrcRes[499],self.YawG[499],self.PitchG[499],self.RollG[499],self.YawS[499],self.PitchS[499],self.RollS[499],self.AccRMS[499],rst,GyroRMS[499]]
            #nowTime=strftime("%H-%M-%S",localtime())
            #print(nowTime)

        self.CurveArr = [self.YawG, self.YawS, self.PitchG, self.PitchS, self.RollG, self.RollS, self.FrcX, self.FrcY, self.FrcZ,self.FrcRes, self.TargetArr,self.DevState]

        
##    
##

    def GetStarted(self, serialComm):
        self.startFlag=0
        while (True):
            while (self.serialComm.inWaiting()==1):
                pass
            dataStream = self.serialComm.readline()
            self.dataString = str(dataStream)
            if self.dataString.startswith('T:Q3'):
                self.startFlag=1
                self.Q1timer.start(1)
                break
            else:
                continue
            print (self.dataString)

    def PlotCurve(self, curveObj, curveArg):
        curveObj.setData(curveArg)

    def showMsgDialog(self):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)

        msg.setText("You Have Performed Wrong Practice")
        msg.setInformativeText(" Do You want to continue treatment session ? ")
        msg.setWindowTitle("Caution Message")
        msg.setDetailedText("The details are as follows:\r\n"+ "\r\n"+
                            "Wrong Device Orientation during Button Press" +
                            "Accidental button press during treatment")
        msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        #msg.buttonClicked.connect(msgbtn)
            
        retval = msg.exec_()
        print ("value of pressed message box button:", retval)
        return retval
                
    def multiTaskingPlot(self):
        
        P1 = mp.Process(target= self.PlotCurve, args=(self.FrcXCurve, self.FrcX))
        P2 = mp.Process(target= self.PlotCurve, args=(self.FrcYCurve, self.FrcY))
        P3 = mp.Process(target= self.PlotCurve, args=(self.FrcZCurve, self.FrcZ))
        P4 = mp.Process(target= self.PlotCurve, args=(self.FrcResCurve, self.FrcRes))

        P5 = mp.Process(target= self.PlotCurve, args=(self.GeoYCurve, self.YawG))
        P6 = mp.Process(target= self.PlotCurve, args=(self.SkinYCurve, self.YawS))

        P7 = mp.Process(target= self.PlotCurve, args=(self.GeoPCurve, self.PitchG))
        P8 = mp.Process(target= self.PlotCurve, args=(self.SkinPCurve, self.PitchS))

        P9 = mp.Process(target= self.PlotCurve, args=(self.GeoRCurve, self.RollG))
        P10 = mp.Process(target= self.PlotCurve, args=(self.SkinRCurve, self.RollS))

        P11 = mp.Process(target= self.PlotCurve, args=(self.FrcTrgCurve, self.TargetArr))

        self.processes = [P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11]
        
            
    def UpdateExec(self):
        #global GeoYCurve,GeoPCurve,GeoRCurve,FrcXCurve,FrcYCurve,FrcZCurve,FrcResCurve,t,ptr,UpdateSBValue
        #storeData()

        self.now = t.time()

        self.storeData()

        dt = self.ptr%5
        ct = self.ptr%10
        gt = self.ptr%20            
        
        if (max(self.FrcRes)<5.0):
            self.FrcPlt.setRange(rect=None, xRange=None, yRange=(-5.0,5.0), padding=None, update=True, disableAutoRange=True)
        else:
            self.FrcPlt.setRange(rect=None, xRange=None, yRange= (-(max(self.FrcRes)),max(self.FrcRes)), padding=None, update=True, disableAutoRange=False)

##      
        
        if dt == 0:
            self.GeoYCurve.setData(self.YawG)
            self.SkinYCurve.setData(self.YawS)           
        elif dt == 1:
            self.GeoPCurve.setData(self.PitchG)
            self.SkinPCurve.setData(self.PitchS)                  
        elif dt == 2:
            self.GeoRCurve.setData(self.RollG)        
            self.SkinRCurve.setData(self.RollS)
        elif dt == 3:
            self.FrcXCurve.setData(self.FrcX)
            self.FrcYCurve.setData(self.FrcY)
            #FrcTrgSetCurve.setData(TargetSetArr)
        elif dt == 4:
            self.FrcZCurve.setData(self.FrcZ)
            self.FrcResCurve.setData(self.FrcRes)
            self.FrcTrgCurve.setData(self.TargetArr)
            
        if (gt==10 and self.ptr>500 and self.RT_PlotInterrupt==1):
            self.ForceLabel.setText(str(self.Fres))
            self.ForceLabel.setFont(QtGui.QFont("Times", 80, QtGui.QFont.Bold))
            self.FrcTrgCurve.setFillLevel(0.95*self.UpdateSBValue)
            
            

        if(gt == 0 and self.ptr>500 and self.RT_PlotInterrupt==1):
            self.UpdateSBValue = self.spinBox.value()
            self.targetLabel.setText("Target Force - " + str(self.UpdateSBValue)+" Newtons")
            self.UpdateLevelVal = 0.1*self.UpdateSBValue
            #FrcTrgRange.setBounds(min = UpdateSBValue-UpdateLevelVal, max=UpdateSBValue-UpdateLevelVal)
            #FrcTrgCurve( fillLevel=UpdateLevelVal)
            #UpdateSpBoxValue()

        bt = self.now - self.lastNow
        self.lastNow = self.now

        if self.fps is None:
            self.fps = 1.0/bt
        else:
            s = np.clip(bt*3., 0, 1)
            self.fps = self.fps * (1-s) + (1.0/bt) * s
            #self.fps = 1.0/bt

        self.FrcPlt.setLabel('bottom','5 seconds Data @ %0.2f fps' % self.fps,**self.labelStyle)

            
            #vb.autoRange()
            
            
        #self.mainCSVwriter.writerow(self.csvRow) 
        
        self.ptr+=1





if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Frame = QtGui.QFrame()
    gui = Q1_Visual_GUI()
    gui.setupUI(Frame)

    serConnect= serial.Serial(port=str("COM20"))
    serConnect.baudrate = 115200
    gui.SystemInitQ1(serConnect)
    #gui.GetStarted(serConnect)
    gui.Q1timer.start(1)
    Frame.show()
    sys.exit(app.exec_())
