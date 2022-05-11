from matplotlib import image
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
from PyQt6.QtGui import QPixmap, QIcon, QFont, QPalette
from PyQt6.QtCore import Qt
import sys
import os

# create a class for the main window


class MainWidgetWindow(QWidget):
    # create the constructor
    def __init__(self):
        # call the constructor of the parent class
        super().__init__()

        self.initUI()  # call the initUI function to create the GUI elements and layout the widgets in the window

    # create the initUI function
    def initUI(self):
        # set up application Gui
        # set the title of the window
        self.setWindowTitle("Battery Control GUI PYQT6")
        # set the size of the window
        self.setGeometry(200, 200, 800, 600)  # x, y, width, height

        # set up the main window
        self.setupMainWindow()

        self.show()  # show the window on the screen and make it visible to the user (must be called after setGeometry)

    # create the setupMainWindow function
    def setupMainWindow(self):
        # create a qlabel to be displayed in the main window
        self.headerlabel = QLabel(self)
        self.headerlabel.setText("Battery Control GUI")
        self.headerlabel.setFont(QFont("Arial", 20))
        # set the alignment of the label
        self.headerlabel.setAlignment(Qt.AlignmentFlag.AlignCenter) # center the text in the label 
        self.headerlabel.setStyleSheet("color: blue")
        self.headerlabel.move(20, 50)

        # find images in the current directory
        # get the path to the images directory
        image_path = os.path.join(os.path.dirname(__file__), 'images')
        image_files = []
        # loop through the files in the images directory
        for file in os.listdir(image_path):
            if file.endswith(".png"):  # if the file is a png file
                image_files.append(file)  # add the file to the list of images

        # create display for the image_files
        self.image_files_label = QLabel(self)
        for file in image_files:
            self.image_files_label.setText(
                self.image_files_label.text() + file + "\n")

        self.image_files_label.setFont(QFont("Arial", 12))
        self.image_files_label.setAlignment(Qt.AlignmentFlag.AlignRight) # set the text alignment to right (default is left) on the label
        self.image_files_label.setStyleSheet("color: Red")
        self.image_files_label.move(20, 100)


if __name__ == '__main__':
    # create an instance of the QApplication class
    app = QApplication(sys.argv)
    # create an instance of the MainWindow class
    window = MainWidgetWindow()
    # execute the application
    # this will exit the application when the window is closed
    sys.exit(app.exec())
