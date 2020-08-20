from PyQt5 import QtWidgets, QtCore, QtGui, QtChart
from PyQt5.QtWidgets import*
from PyQt5.QtGui import*
from PyQt5.QtChart import*
from PyQt5.QtCore import*

class startWidget(QWidget):
    def __init__(self):
        super(startWidget,self).__init__()
        layout = QVBoxLayout()

        lbl1 = QLabel("ASU Racing Team Car no. 80", self)
        lbl1.move(380, 50)
        pixmap = QPixmap('Team Logo.jpg')
        lbl1.setPixmap(pixmap)
        font_lbl = QFont()
        font_lbl.setPixelSize(50)
        font_lbl.setBold(True)
        font_lbl.setFamily("Magma")
        lbl1.setAutoFillBackground(True)
        lbl1.setAlignment(Qt.AlignCenter)
        #w.setColor(lbl1.backgroundRole(),Qt.white) #for changing the text background color
        w = lbl1.palette()
        w.setColor(lbl1.foregroundRole(), Qt.white)
        lbl1.setPalette(w)
        lbl1.setFont(font_lbl)
        lbl1.adjustSize()


        self.button = QPushButton('Start 80', self)
        font_btn = QFont()
        font_btn.setPixelSize(20)
        fp = QPalette()
        fp.setColor(self.button.foregroundRole(), Qt.white)
        self.button.setFont(font_btn)
        self.button.setPalette(fp)
        self.button.resize(200,80)

        self.button.setStyleSheet("background-color: black;")
        self.button.move(550,600)

        layout.addWidget(lbl1)
        layout.addWidget(self.button)
        layout.setAlignment(Qt.AlignCenter)
        self.setLayout(layout)
