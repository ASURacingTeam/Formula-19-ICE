import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from startWindow import*

App = QApplication(sys.argv)
ex = Start_Window()
ex.show()
App.setStyleSheet("".join(open("styling.css").readlines()))
sys.exit(App.exec())
