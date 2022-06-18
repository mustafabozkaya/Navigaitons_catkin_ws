from matplotlib import image
from StatsArray import StatsArray
from registration import NewUserDialog
import numpy as np
import math
import matplotlib.pyplot as plt

"""
QMainWindow vs. QWidget
The QMainWindow class focuses on creating and managing the layout for the main
window of an application. It allows you to set up a window with a status bar, a toolbar,
dock widgets, or other menu features in predefined locations.
The QWidget class is the base class for all user interface objects in Qt, including
widgets. The widgets you have used, such as QPushButton and QTextEdit, inherit
QWidget, granting them access to a wide array of methods for interacting with an
interface or setting the parameters of a widget instance. It is important to note that the
QMainWindow and the QDialog classes also inherit QWidget and are special purpose
classes focusing on creating main windows and dialogs, respectively

The QMainWindow
class provides the functionalities for building a main window’s key features, such as
menu bars and toolbars. In PyQt6, the QAction class is now located in the QtGui module
"""

# Import the Qt Application class from the PyQt5.QtWidgets module  for the GUI application
from PyQt6.QtWidgets import QApplication, QStyleFactory
# QWidget is the base class for all user interface objects in QT. it provides a lot of functionality for the user interface.
from PyQt6.QtWidgets import QWidget
# QDialog is the base class for all dialogs in QT. it provides a lot of functionality for the dialogs.
from PyQt6.QtWidgets import QDialog
# QMainWindow  is the base class for all main windows in QT. it provides a lot of functionality for the main windows, such as menu bars, toolbars, and status bars, central widgets, and dock widgets.
from PyQt6.QtWidgets import QMainWindow
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
from PyQt6.QtWidgets import QMessageBox
from PyQt6.QtWidgets import QFileDialog
from PyQt6.QtWidgets import QDockWidget, QTabWidget, QTableWidget, QTableWidgetItem
# in PyQt6 QAction is a class that represents an action in a GUI application
from PyQt6.QtGui import QAction
from PyQt6.QtGui import QPixmap, QIcon, QFont, QPalette, QColor, QBrush, QPainter, QPen
from PyQt6.QtCore import Qt, QSize, QThread, QTimer, QObject, pyqtSignal, pyqtSlot
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

        # set the minimum size of the window
        self.minimumSize = QSize(500, 500)
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
        # set the alignment of the label center the text in the label
        self.headerlabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.headerlabel.setStyleSheet("color: blue")
        self.headerlabel.move(20, 50)

        # call the image_files function to get the image files names from the image_path and return a list of image files names and image_path
        image_files, image_path = self.image_files()

        # call the form user label function
        self.form_user_label()

        # call the createimgLabel function to display the images
        #self.createimgLabel(image_files, image_path)

        # call the pushButton function to create the push button
        # self.pushButton()

    # create image_files function to get the image files names from the image_path and return a list of image files names and image_path

    def image_files(self):
        # find images in the current directory
        # get the path to the images directory
        # if images directory does not exist, create it
        if not os.path.exists("images"):  # if the images directory does not exist
            os.mkdir("images")  # create the images directory

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

        return image_files, image_path

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
        self.button_label.setAlignment(Qt.AlignmentFlag.AlignVCenter)
        self.button_label.setStyleSheet("color: orange")
        # set the position of the label to the right of the image_files_label widget and below the image_files_label widget
        self.button_label.move(self.image_files_label.x(
        ) + self.image_files_label.width() + 40, self.image_files_label.y())
        self.button_label.resize(200, 50)
        # create a push button
        self.button = QPushButton("Push Me everywhere", self)
        self.button.setFont(QFont("Arial", 12))
        self.button.setStyleSheet("color: red")

        self.button.move(self.button_label.x(
        )+20, self.button_label.y() + self.button_label.height() + 10)  # locate the button below the button_label widget
        # connect the button to the button_clicked function when clicked
        self.button.clicked.connect(self.button_clicked)

    # create the button_clicked function
    def button_clicked(self):
        """Handle when the button is clicked.
            Demonstrates how to change text for widgets,
            update their sizes and locations, and how to
            close the window due to events."""

        # increment the time_press variable
        self.time_press += 1
        # set the text of the button label to the value of the time_press variable
        if self.time_press == 1:
            self.button_label.setText("you pushed me")
        if self.time_press == 2:
            self.button_label.setText("you pushed me again")
            self.button.setText("Push Me")
            self.button.adjustSize()  # adjust the size of the button to fit the text in the button
            # move the button down by 50 pixels from the previous button
            self.button.move(self.button.x(), self.button.y() + 50)

        if self.time_press == 3:
            print("you pushed me 3 times")
            print("Window will close")
            self.close()

    # create the QlineEdit function

    def form_user_label(self):
        """create and arrange widgets in the main window"""
        QLabel("Enter your name:", self).move(
            20, 200)  # create a label to display the text "Enter your name:"
        self.line_edit = QLineEdit(self)  # create a line edit widget
        # create a label to display the user's name and last name
        self.user_info = QLabel("", self)
        # adjust the size of the label to fit the text in the label
        self.user_info.setFont(QFont("Arial", 12))

        self.user_info.move(150, 200)  # set the position of the label
        # self.user_info.resize(200, 50)  # set the size of the label
        # create a label to display the text "your last name "
        QLabel("your last name ").move(20, 250)
        # create a line edit widget for the last name
        self.line_edit_lastname = QLineEdit(self)
        # set the position of the line edit widget
        self.line_edit.move(20, 220)
        # set the position of the line edit widget for the last name
        self.line_edit_lastname.move(20, 270)
        self.line_edit.resize(200, 30)  # set the size of the line edit widget
        # set the size of the line edit widget for the last name
        self.line_edit_lastname.resize(200, 30)
        # set the font of the line edit widget
        self.line_edit.setFont(QFont("Arial", 12))
        # set the font of the line edit widget for the last name
        self.line_edit_lastname.setFont(QFont("Arial", 12))
        # set the style of the line edit widget
        self.line_edit.setStyleSheet("color: blue")
        # set the style of the line edit widget for the last name
        self.line_edit_lastname.setStyleSheet("color: blue")
        # set the placeholder text of the line edit widget
        self.line_edit.setPlaceholderText("Enter your name")
        # set the placeholder text of the line edit widget for the last name
        self.line_edit_lastname.setPlaceholderText("Enter your last name")
        # set the echo mode of the line edit widget
        self.line_edit.setEchoMode(QLineEdit.EchoMode.Normal)
        # set the echo mode of the line edit widget for the last name
        self.line_edit_lastname.setEchoMode(QLineEdit.EchoMode.Normal)
        # set the maximum length of the line edit widget
        self.line_edit.setMaxLength(20)
        # set the maximum length of the line edit widget for the last name
        self.line_edit_lastname.setMaxLength(20)

        clear_button = QPushButton("Clear", self)  # create a push button
        clear_button.setEnabled(True)  # set the button to be enabled
        clear_button.move(self.line_edit_lastname.x(
        ), self.line_edit_lastname.y()+self.line_edit_lastname.height()+10)
        clear_button.clicked.connect(self.clear_button_clicked)
        # set the font of the push button
        clear_button.setFont(QFont("Arial", 10))
        # set the style of the push button
        clear_button.setStyleSheet("color: red")
        # create a push button to send the data
        send_button = QPushButton("Send", self)
        send_button.move(clear_button.x() +
                         clear_button.width()+10, clear_button.y())
        send_button.clicked.connect(self.send_button_clicked)

    def clear_button_clicked(self):
        """clear the line edit widget"""
        self.messageBox()  # call the message
        self.line_edit.clear()
        self.line_edit_lastname.clear()
        self.user_info.setText("")

    def send_button_clicked(self):
        """send the data to the console"""
        print(f"{self.line_edit.text()} {self.line_edit_lastname.text()}")
        self.user_info.setText(
            f"Mr. {self.line_edit.text()} {self.line_edit_lastname.text()}")
        self.user_info.setStyleSheet("background-color: yellow")
        self.user_info.adjustSize()

    def messageBox(self):
        """create a message boxsalert message FOR THE USER"""
        QMessageBox.about(
            self, "about", "clears froms data")  # create a message box with the title "Title" and the message "This is a message box"
        """create a message box to alert message FOR THE USER"""
        answer = QMessageBox.question(self, "User not found", """<h4>User not found</h4> <p>Do you register new user </p>""", QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                      QMessageBox.StandardButton.No)

        if answer == QMessageBox.StandardButton.No:
            print("application will close")
            self.close()
        elif answer == QMessageBox.StandardButton.Yes:
            print("register new user")
            self.register_user()

    def register_user(self):
        self.new_user = NewUserDialog()
        self.new_user.show()


if __name__ == '__main__':
    # create an instance of the QApplication class
    app = QApplication(sys.argv)
    app_style = QStyleFactory.keys()  # get the list of available styles
    print("app style", app_style)  # print the available styles
    app.setStyle(app_style[1])  # set the style of the application
    # create an instance of the MainWindow class
    window = MainWidgetWindow()
    # execute the application
    # this will exit the application when the window is closed
    sys.exit(app.exec())
