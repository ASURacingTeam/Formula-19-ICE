from PyQt5 import QtGui, QtChart, QtCore, QtWidgets
import os.path
# import serial
import time
# import sys
from serial import*
from PyQt5.QtWidgets import*
from PyQt5.QtGui import*
from PyQt5.QtCore import*
from PyQt5.QtChart import*
from startWidget import*
from displayWidget import*


# ser = serial.Serial("COM4", 9600) #readjust the port number and the baudrate, timeout = x //unit secnods might be added
# data = [Speed1,Speed2,Speed3,Speed4,AvgSpeed,RPM,Sus1,Sus2,Sus3,Sus4,Temp,x_acc,y_acc,z_acc,x_gyro,y_gyro,z_gyro]
# while 1:
#     if ser.inwaiting():
#         for i in range(17):
#             data[i] = ser.readline(ser.inwaiting()) #might be changed to read() instead of readline()

# print(data)

# def serial_ports():
#     """ Lists serial port names
#
#         :raises EnvironmentError:
#             On unsupported or unknown platforms
#         :returns:
#             A list of the serial ports available on the system
#     """
#     if sys.platform.startswith('win'):
#         ports = ['COM%s' % (i + 1) for i in range(256)]
#     elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
#         # this excludes your current terminal "/dev/tty"
#         ports = glob.glob('/dev/tty[A-Za-z]*')
#     elif sys.platform.startswith('darwin'):
#         ports = glob.glob('/dev/tty.*')
#     else:
#         raise EnvironmentError('Unsupported platform')
#
#     result = []
#     for port in ports:
#         try:
#             s = serial.Serial(port)
#             s.close()
#             result.append(port)
#         except (OSError, serial.SerialException):
#             pass
#     return result
#
# connectedPort = serial_ports()
# ser = serial.Serial('/dev/ttyACM0')
# ser.flushInput()
# while True:
#     try:
#         ser_bytes = ser.readline()
#         decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
#         print(decoded_bytes)
#         with open("vehicle_data.csv","a") as f:
#             writer = csv.writer(f,delimiter=",")
#             writer.writerow([time.time(),decoded_bytes])
#     except:
#         print("Keyboard Interrupt")
#         break


#Reading the text file
#f = open("data.txt", "r")
#f.close()
"""
filename = "bestand.py"
if not os.path.isfile(filename):
    print('File does not exist.')
else:
# Open the file as f.
# The function readlines() reads the file.
with open(filename) as f:
    content = f.read().splitlines()

# Show the file contents line by line.
# We added the comma to print single newlines and not double newlines.
# This is because the lines contain the newline character '\n'.
for line in content:
    print(line)
"""
class Start_Window(QMainWindow):

    def __init__(teleStart):
        super(Start_Window,teleStart).__init__()
        teleStart.left = 2 #position from left of screen
        teleStart.top = 40 #position from top of screen
        teleStart.title = 'ASU Racing Team telemetry'
        teleStart.width = 2000 #width of window
        teleStart.height = 1000 #height of window

        teleStart.InitWindow()

    def InitWindow(teleStart):
        teleStart.setWindowTitle(teleStart.title)
        #teleStart.setGeometry(teleStart.top, teleStart.left, teleStart.width, teleStart.height)
        teleStart.setWindowIcon(QIcon('Team Logo'))
        teleStart.setIconSize(QSize(10,10))
        #teleStart.showFullScreen()
        teleStart.setAutoFillBackground(True)
        p = teleStart.palette()
        p.setColor(teleStart.backgroundRole(), Qt.white) #OR teleStart.setStyleSheet("background-color: white;")
        teleStart.setPalette(p)



        firstWidget = startWidget()
        teleStart.setCentralWidget(firstWidget)
        firstWidget.button.clicked.connect(teleStart.start_onClick)
        # teleStart.show() 

    def editor(teleStart):
        teleStart.textEdit = QTextEdit()
        teleStart.setCentralWidget(teleStart.textEdit)


    @pyqtSlot()
    def start_onClick(teleStart):
        teleStart.statusBar().showMessage("Telemetry started")
        # name,_ = QtWidgets.QFileDialog.getOpenFileName(teleStart,'Open File')
        # file = open(name,'r')
        # teleStart.editor()
        #
        # with file:
        #     text = file.read()
        #     teleStart.textEdit.setText(text)
        secondWidget = DisplayWidget()
        teleStart.setCentralWidget(secondWidget)
        # teleStart.cams = Window()
        # teleStart.cams.show()
        # teleStart.close()
