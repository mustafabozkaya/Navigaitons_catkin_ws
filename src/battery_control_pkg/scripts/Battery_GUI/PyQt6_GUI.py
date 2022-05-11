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
        # center the text in the label
        self.headerlabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
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
        # set the text alignment to right (default is left) on the label
        self.image_files_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.image_files_label.setStyleSheet("color: Red")
        self.image_files_label.move(20, 100)

        # create a qlabel to be displayed in the main window

        for i in range(len(image_files)):
            self.image_labes = QLabel(self)  # create a label for the image
            # create a pixmap from the first image in the list
            pixmap = QPixmap(os.path.join(image_path, image_files[i]))
            self.image_labes.setPixmap(pixmap)  # set the pixmap to the label
            # set the position of the label
            self.image_labes.move(20+i*10, 250)
            # set the size of the label to the size of the pixmap
            scale_factor = 0.5

            self.image_labes.resize(
                pixmap.width()*scale_factor, pixmap.height()*scale_factor)


if __name__ == '__main__':
    # create an instance of the QApplication class
    app = QApplication(sys.argv)
    # create an instance of the MainWindow class
    window = MainWidgetWindow()
    # execute the application
    # this will exit the application when the window is closed
    sys.exit(app.exec())
