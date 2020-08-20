from PyQt5 import QtWidgets, QtCore, QtGui, QtChart
from PyQt5.QtWidgets import*
from PyQt5.QtGui import*
from PyQt5.QtChart import*
from PyQt5.QtCore import*

# Dummy values for variables
FRS = 1  	#Front Right Wheel speed - Speed1
RRS = 1.1 	#Rear Right Wheel - Speed2
FLS = 1.2 	#Front Left Wheel - Speed3
RLS = 1.1 	#Rear Left Wheel - Speed4
ratio = 180/140
example_speed = "60" # str(AvgSpeed)
s = int(example_speed)*ratio
FRD = 0.1 #Sus1
RRD = 0.2 #Sus2
FLD = 0.1 #Sus3
RLD = 0.2 #Sus4
battery_voltage = "12v"
gear = "2"
rpm = "12k" #RPM
steering_angle = "4°"
temperature = "50°C" #Temp
class DisplayWidget(QWidget):
    def __init__(self):
        super(DisplayWidget,self).__init__()
        layout = QHBoxLayout()
        mainGroupBox = QGroupBox()
        self.gridLayout = QGridLayout()
        self.gridLayout.setColumnStretch(2,3)
        self.gridLayout.setColumnStretch(1, 2)
        self.gridLayout.setColumnStretch(0,1)

        self.fillGridLayout()
        mainGroupBox.setLayout(self.gridLayout)
        layout.addWidget(mainGroupBox)
        self.setLayout(layout)

    def fillGridLayout(self):

        series_1= QBarSeries()
        chart_1 = QChart()

        set0 = QBarSet("Front Right Wheel")
        set1 = QBarSet("Rear Right Wheel")
        set2 = QBarSet("Front Left Wheel")
        set3 = QBarSet("Rear Left Wheel")

        set0 << FRS
        set1 << RRS
        set2 << FLS
        set3 << RLS

        series_1.append(set0)
        series_1.append(set1)
        series_1.append(set2)
        series_1.append(set3)

        chart_1.addSeries(series_1)
        chart_1.setTitle("Wheels Speed in Km/hr")
        chart_1.setAnimationOptions(QChart.SeriesAnimations)
        categories_1 = ["Wheels"]

        x1_axis = QBarCategoryAxis()
        x1_axis.append(categories_1)
        chart_1.createDefaultAxes()
        chart_1.setAxisX(x1_axis, series_1)
        chart_1.legend().setVisible(True)
        chart_1.legend().setAlignment(Qt.AlignBottom)

        chartView_1 = QChartView(chart_1)
        chartView_1.setRenderHint(QPainter.Antialiasing)
        slip_ratio = str(round((set1[0]-set0[0])/set1[0],5))
#-------------------------------------------------------------------------------
        speedLabel = QLabel(example_speed + "Km/hr")
        font_speed = QFont()
        font_speed.setPixelSize(27)
        font_speed.setBold(True)
        font_speed.setFamily("Magma")
        w = speedLabel.palette()
        w.setColor(speedLabel.foregroundRole(), Qt.green)
        speedLabel.setPalette(w)
        speedLabel.setFont(font_speed)
        speedLabel.setAlignment(Qt.AlignVCenter)

        mar = QHBoxLayout()
        mar.addWidget(speedLabel)
        mar.setGeometry(QRect(0, 0, 80, 100))
#-------------------------------------------------------------------------------
        lbl_SR = QLabel("Slip Ratio = " + slip_ratio)
        lbl_SR.move(500, 480)
        font_lbl_SR = QFont()
        font_lbl_SR.setPixelSize(20)
        font_lbl_SR.setBold(True)
        font_lbl_SR.setFamily("Magma")

        z = lbl_SR.palette()
        z.setColor(lbl_SR.foregroundRole(), Qt.darkCyan)
        lbl_SR.setPalette(z)
        lbl_SR.setFont(font_lbl_SR)
        lbl_SR.adjustSize()

#-------------------------------------------------------------------------------
        series_2= QBarSeries()
        chart_2 = QChart()

        set4 = QBarSet("Front Right Damper")
        set5 = QBarSet("Rear Right Damper")
        set6 = QBarSet("Front Left Damper")
        set7 = QBarSet("Rear Left Damper")

        set4 << FRD
        set5 << RRD
        set6 << FLD
        set7 << RLD

        series_2.append(set4)
        series_2.append(set5)
        series_2.append(set6)
        series_2.append(set7)

        chart_2.addSeries(series_2)
        chart_2.setTitle("Damper Displacement")
        chart_2.setAnimationOptions(QChart.SeriesAnimations)
        categories_2 = ["Potentiometer"]

        x2_axis = QBarCategoryAxis()
        x2_axis.append(categories_2)
        chart_2.createDefaultAxes()
        chart_2.setAxisX(x2_axis, series_2)
        chart_2.legend().setVisible(True)
        chart_2.legend().setAlignment(Qt.AlignBottom)

        chartView_2 = QChartView(chart_2)
        chartView_2.setRenderHint(QPainter.Antialiasing)
#-------------------------------------------------------------------------------
        # lbl_SA = QLabel("Steering angle = " + steering_angle)
        # #lbl_SA.move(500, 480)
        # font_lbl_SA = QFont()
        # font_lbl_SA.setPixelSize(20)
        # font_lbl_SA.setBold(True)
        # font_lbl_SA.setFamily("Magma")
        #
        # x = lbl_SA.palette()
        # x.setColor(lbl_SA.foregroundRole(), Qt.darkCyan)
        # lbl_SA.setPalette(x)
        # lbl_SA.setFont(font_lbl_SA)
        # lbl_SA.adjustSize()
#-------------------------------------------------------------------------------
        table = QTableWidget(5,2)
        table.setHorizontalHeaderLabels(['Parameter','Value'])
        table.setItem(0,0, QTableWidgetItem("Battery Voltage"))
        table.setItem(1,0, QTableWidgetItem("Gear"))
        table.setItem(2,0, QTableWidgetItem("RPM"))
        table.setItem(3,0, QTableWidgetItem("Steering Angle"))
        table.setItem(4,0, QTableWidgetItem("Temperature"))

        table.setItem(0,1, QTableWidgetItem(battery_voltage))
        table.setItem(1,1, QTableWidgetItem(gear))
        table.setItem(2,1, QTableWidgetItem(rpm))
        table.setItem(3,1, QTableWidgetItem(steering_angle))
        table.setItem(4,1, QTableWidgetItem(temperature))

        table.item(0,1).setTextAlignment(Qt.AlignCenter)
        table.item(1,1).setTextAlignment(Qt.AlignCenter)
        table.item(2,1).setTextAlignment(Qt.AlignCenter)
        table.item(3,1).setTextAlignment(Qt.AlignCenter)
        table.item(4,1).setTextAlignment(Qt.AlignCenter)

        Hheader = table.horizontalHeader()
        Hheader.setSectionResizeMode(QHeaderView.ResizeToContents)
        Hheader.setSectionResizeMode(1, QHeaderView.Stretch)
        Hheader.setDefaultAlignment(Qt.AlignCenter)

        Vheader = table.verticalHeader()
        Vheader.setSectionResizeMode(QHeaderView.ResizeToContents)
        Vheader.setSectionResizeMode(0, QHeaderView.Stretch)
        Vheader.setSectionResizeMode(1, QHeaderView.Stretch)
        Vheader.setSectionResizeMode(2, QHeaderView.Stretch)
        Vheader.setSectionResizeMode(3, QHeaderView.Stretch)
        Vheader.setSectionResizeMode(4, QHeaderView.Stretch)


        font_table = QFont()
        font_table.setPixelSize(15)
        font_table.setBold(True)
        font_table.setFamily("Magma")

        table.setFont(font_table)


#-------------------------------------------------------------------------------
        self.gridLayout.addLayout(mar,0,1)
        self.gridLayout.addWidget(chartView_1,0,2)
        self.gridLayout.addWidget(chartView_2,2,2)
        self.gridLayout.addWidget(lbl_SR,1,2)
        self.gridLayout.addWidget(table,2,0,1,2)


    def paintEvent(self,event):
        arc_g = QPainter()
        arc_o = QPainter()
        arc_r = QPainter()
        speed = QPainter()

        arc_g.begin(self)
        arc_o.begin(self)
        arc_r.begin(self)

        arc_g.setRenderHint(QPainter.Antialiasing)
        arc_g.setPen(QPen(Qt.green,  5, Qt.SolidLine))

        arc_o.setRenderHint(QPainter.Antialiasing)
        arc_o.setPen(QPen(Qt.yellow,  5, Qt.SolidLine))

        arc_r.setRenderHint(QPainter.Antialiasing)
        arc_r.setPen(QPen(Qt.red,  5, Qt.SolidLine))
        #speed.setPen(QtCore.Qt.green)
        arc_g.setBrush(QtCore.Qt.white)
        # speed.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        arc_g.drawArc(200, 100,300,300, 0*16, 180*16)
        arc_o.drawArc(200, 100,300,300, 0*16, 180*8)
        arc_r.drawArc(200, 100,300,300, 0*16, 180*3)

        speed.begin(self)
        speed.setRenderHint(QPainter.Antialiasing)
        speed.setPen(QPen(Qt.blue,  20, Qt.SolidLine))
        speed.setBrush(QtCore.Qt.white)
        speed.drawArc(214, 117,270,250, 180*16, -1*s*16) #s to be variated according to speed with max 180
