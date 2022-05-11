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
        # allow the text to wrap around to the next line   if it is too long
        self.image_files_label.setWordWrap(False)
        self.image_files_label.move(20, 100)

        # call the createimgLabel function to display the images
        self.createimgLabel(image_files, image_path)

        # call the pushButton function to create the push button
        self.pushButton()

    # create image_files_label function
    def createimgLabel(self, image_files, image_path):
        # create a qlabel to be displayed in the main window
        x, y = 20, 250
        for image in image_files:

            try:
                with open(os.path.join(image_path, image), 'rb'):

                    # create a label for the image
                    self.image_labes = QLabel(self)
                    # create a pixmap from the first image in the list
                    pixmap = QPixmap(os.path.join(image_path, image))
                    # set the pixmap to the label
                    self.image_labes.setPixmap(pixmap)
                    # set the position of the label
                    self.image_labes.move(x, y)
                    # set the size of the label to the size of the pixmap
                    scale_factor = 0.5

                    self.image_labes.resize(
                        int(pixmap.width()*scale_factor), int(pixmap.height()*scale_factor))

            except FileNotFoundError as err:
                print(f"İmage file not found \n : {err}")

            x += 15
            y += 30
    # create the pushButton function    to create a push button with a label

    def pushButton(self):
        """create and arrange widgets in the main window"""
        # create a time press button
        self.time_press = 0
        self.button_label = QLabel("don't push me", self)
        self.button_label.setFont(QFont("Arial", 12))
        self.button_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.button_label.setStyleSheet("color: orange")
        self.button_label.move(200, 30)
        self.button_label.resize(200, 50)
        # create a push button
        self.button = QPushButton("Push Me", self)
        self.button.setFont(QFont("Arial", 12))
        self.button.setStyleSheet("color: red")
        self.button.move(20, 350)
        # connect the button to the button_clicked function when clicked
        self.button.clicked.connect(self.button_clicked)

    # create the button_clicked function
    def button_clicked(self):
        # increment the time_press variable
        self.time_press += 1
        # set the text of the button label to the value of the time_press variable
        if self.time_press == 1:
            self.button_label.setText("you pushed me")
        if self.time_press == 2:
            self.button_label.setText("you pushed me again")
            self.button.setText("Push Me Again")
            self.button.adjustSize()
            self.button.move(250, 40)

        if self.time_press == 3:
            print("you pushed me 3 times")
            print("Window will close")
            self.close()


if __name__ == '__main__':
    # create an instance of the QApplication class
    app = QApplication(sys.argv)
    # create an instance of the MainWindow class
    window = MainWidgetWindow()
    # execute the application
    # this will exit the application when the window is closed
    sys.exit(app.exec())
