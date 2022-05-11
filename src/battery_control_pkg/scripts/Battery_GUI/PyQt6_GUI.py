from StatsArray import StatsArray
import numpy as np
import math
import matplotlib.pyplot as plt
from PyQt6.QtWidgets import QApplication
from PyQt6.QtWidgets import QMainWindow
from PyQt6.QtWidgets import QWidget
from PyQt6.QtWidgets import QPushButton
from PyQt6.QtWidgets import QLabel
from PyQt6.QtWidgets import QLineEdit
from PyQt6.QtWidgets import QGridLayout
from PyQt6.QtWidgets import QVBoxLayout
from PyQt6.QtWidgets import QHBoxLayout
from PyQt6.QtWidgets import QComboBox
from PyQt6.QtWidgets import QCheckBox
from PyQt6.QtWidgets import QGroupBox
from PyQt6.QtWidgets import QRadioButton
import sys
import os

# create a class for the main window


class MainWindow(QWidget):
    # create the constructor
    def __init__(self):
        # call the constructor of the parent class
        super().__init__()

        self.initUI()  # call the initUI function to create the GUI elements and layout the widgets in the window

    # create the initUI function
    def initUI(self):
        # set the title of the window
        self.setWindowTitle("Battery Control GUI PYQT6")
        # set the size of the window
        self.setGeometry(200, 200, 800, 600) # x, y, width, height

        self.show()  # show the window on the screen and make it visible to the user (must be called after setGeometry)


if __name__ == '__main__':
    # create an instance of the QApplication class
    app = QApplication(sys.argv)
    # create an instance of the MainWindow class
    window = MainWindow()
    # execute the application
    # this will exit the application when the window is closed
    sys.exit(app.exec())
