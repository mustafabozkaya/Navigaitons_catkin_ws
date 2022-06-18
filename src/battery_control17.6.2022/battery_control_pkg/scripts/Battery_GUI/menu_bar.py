
from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QIcon, QPixmap, QPainter, QColor, QFont, QAction
from PyQt6.QtCore import Qt, QSize
from PyQt6.QtWidgets import QWidget, QPushButton, QLabel, QLineEdit, QGridLayout, QVBoxLayout, QHBoxLayout, QComboBox, QCheckBox, QMessageBox, QFileDialog
from PyQt6.QtWidgets import QMainWindow
import sys
import os
import time
import subprocess


class Menubar(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Menubar")
        self.setGeometry(300, 300, 300, 300)
        self.setWindowIcon(QIcon("images/icon.png"))
        self.statusBar().showMessage("Ready")
        self.create_menu()

    def create_menu(self):
        self.menu = self.menuBar()
        self.file_menu = self.menu.addMenu("File")
        self.file_menu.addAction(QAction("New", self))
        self.file_menu.addAction(QAction("Open", self))
        self.file_menu.addAction(QAction("Save", self))
        self.file_menu.addAction(QAction("Save as", self))
        # Exit is a built-in function
        self.file_menu.addAction(QAction("Exit", self))
        # connect the menu to the function menu_choice
        self.file_menu.triggered[QAction].connect(self.menu_choice)

    def menu_choice(self, q):
        print(q.text() + " was triggered")

  

    def createactions(self):
        """Create the application's menu actions"""

        self.new_user_action = QAction(
            QIcon("new_user.png"), "&New User", self)
        self.new_user_action.setShortcut("Ctrl+N")
        # set the status tip for the action to be displayed in the status bar of the window when the mouse is over the action item in the menu bar
        self.new_user_action.setStatusTip("Create a new user")
        self.new_user_action.triggered.connect(self.register_user)

        self.exit_action = QAction(QIcon("exit.png"), "&Exit", self)
        self.exit_action.setShortcut("Ctrl+Q")
        # set the status tip for the exit action  (tool tip)
        self.exit_action.setStatusTip("Exit application")
        self.exit_action.triggered.connect(self.close)

        self.about_action = QAction(QIcon("about.png"), "&About", self)
        self.about_action.setShortcut("Ctrl+A")
        # set the status tip for the about action  (tool tip)
        self.about_action.setStatusTip("About application")
        # triggered function for the about action
        self.about_action.triggered.connect(self.about)

        self.help_action = QAction(QIcon("help.png"), "&Help", self)
        self.help_action.setShortcut("Ctrl+H")
        # set the status tip for the help action  (tool tip)
        self.help_action.setStatusTip("Help")
        # triggered function for the help action
        self.help_action.triggered.connect(self.help)

        self.open_action = QAction(QIcon("open.png"), "&Open", self)
        self.open_action.setShortcut("Ctrl+O")
        # set the status tip for the open action  (tool tip)
        self.open_action.setStatusTip("Open image")
        # triggered function for the open action
        self.open_action.triggered.connect(self.open_image)

    def open_image(self):
        # open the file dialog to select the image
        filename = QFileDialog.getOpenFileName(
            self, 'Open file', os.getenv('HOME'))
        # set the filename to the filename variable
        print("filename: ", filename)
        self.filename = filename[0]
        # call the image_files function to get the image files names from the image_path and return a list of image files names and image_path
        image_files, image_path = self.image_files()
        # call the createimgLabel function to display the images
        self.createimgLabel(image_files, image_path)

    def help(self):
        print("help")

    def about(self):
        print("about")

    # create menu function to create the menu bar

    def createMenu(self):
        # create menu bar
        menubar = self.menuBar()
        # menubar.setNativeMenuBar(False) # set the menu bar to be not native to the OS (i.e. not the OS menu bar)
        # create the file menu and add the actions to it
        file_menu = menubar.addMenu("&File")
        # add the actions to the file menu
        # add the open action to the file menu
        file_menu.addAction(self.open_action)
        file_menu.addSeparator()  # add a separator to the file menu
        # add the exit action to the file menu
        file_menu.addAction(self.exit_action)

        # create the user menu and add the actions to it
        user_menu = menubar.addMenu("&User")
        # add the actions to the user menu
        # add the new user action to the user menu
        user_menu.addAction(self.new_user_action)

        # create the about menu
        about_menu = menubar.addMenu("&About")
        # add the about action to the about menu
        about_menu.addAction(self.about_action)

        # create the help menu
        help_menu = menubar.addMenu("&Help")
        # add the help action to the help menu
        help_menu.addAction(self.help_action)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Menubar()
    window.show()
    sys.exit(app.exec())


