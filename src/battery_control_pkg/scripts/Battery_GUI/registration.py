
# Import necessary modules
import sys
import os
from PyQt6.QtWidgets import (QApplication, QDialog, QLabel,
                             QPushButton, QLineEdit, QMessageBox)
from PyQt6.QtGui import QFont, QPixmap


class NewUserDialog(QDialog):

    def __init__(self):
        super().__init__()
        self.setModal(True)
        self.initializeUI()

    def initializeUI(self):
        """Set up the application's GUI."""
        self.setFixedSize(360, 320)
        self.setWindowTitle("Registration GUI")
        self.setUpWindow()

    def setUpWindow(self):
        """Create and arrange widgets in the window for 
        collecting new account information."""
        login_label = QLabel("Create New Account", self)
        login_label.setFont(QFont("Arial", 20))
        login_label.move(90, 20)

        # if images directory does not exist, create it
        if not os.path.exists("images"):  # if the images directory does not exist
            os.mkdir("images")  # create the images directory

        image_path = os.path.join(os.path.dirname(__file__), 'images')
        image_files = []
        # loop through the files in the images directory
        for file in os.listdir(image_path):
            if file.endswith(".png"):  # if the file is a png file
                print(f"file type : {type(file)}\n file name : {file}")
                image_files.append(file)  # add the file to the list of images

        # if the file is a user image file (i.e. user1.png) then set the image to user_image
        user_image = ""
        for file in image_files:
            if file.lower().startswith("profile"):
                user_image = file

        try:
            with open(user_image):
                user_label = QLabel(self)
                pixmap = QPixmap(user_image)
                user_label.setPixmap(pixmap)
                user_label.move(150, 60)
        except FileNotFoundError as error:
            print(f"Image not found. Error: {error}")

        # Create name QLabel and QLineEdit widgets
        name_label = QLabel("Username:", self)
        name_label.move(20, 144)

        self.name_edit = QLineEdit(self)
        self.name_edit.resize(250, 24)
        self.name_edit.move(90, 140)

        full_name_label = QLabel("Full Name:", self)
        full_name_label.move(20, 174)

        full_name_edit = QLineEdit(self)
        full_name_edit.resize(250, 24)
        full_name_edit.move(90, 170)

        # Create password QLabel and QLineEdit widgets
        new_pswd_label = QLabel("Password:", self)
        new_pswd_label.move(20, 204)

        self.new_pswd_edit = QLineEdit(self)
        self.new_pswd_edit.setEchoMode(QLineEdit.EchoMode.Password)
        self.new_pswd_edit.resize(250, 24)
        self.new_pswd_edit.move(90, 200)

        confirm_label = QLabel("Confirm:", self)
        confirm_label.move(20, 234)

        self.confirm_edit = QLineEdit(self)
        self.confirm_edit.setEchoMode(QLineEdit.EchoMode.Password)
        self.confirm_edit.resize(250, 24)
        self.confirm_edit.move(90, 230)

        # Create sign up QPushButton
        sign_up_button = QPushButton("Sign Up", self)
        sign_up_button.resize(320, 32)
        sign_up_button.move(20, 270)
        sign_up_button.clicked.connect(self.confirmSignUp)

    def confirmSignUp(self):
        """Check if user information is entered and correct. 
        If so, append username and password text to file."""
        name_text = self.name_edit.text()
        pswd_text = self.new_pswd_edit.text()
        confirm_text = self.confirm_edit.text()

        if name_text == "" or pswd_text == "":
            # Display QMessageBox if passwords don't match
            QMessageBox.warning(self, "Error Message",
                                "Please enter username or password values.",
                                QMessageBox.StandardButton.Close,
                                QMessageBox.StandardButton.Close)
        elif pswd_text != confirm_text:
            # Display QMessageBox if passwords don't match
            QMessageBox.warning(self, "Error Message",
                                "The passwords you entered do not match.",
                                QMessageBox.StandardButton.Close,
                                QMessageBox.StandardButton.Close)
        else:
            # Return to login window if passwords match
            with open(os.path.join(os.path.dirname(__file__), 'files/users.txt'), 'a') as f:
                f.write("\n" + name_text + " ")
                f.write(pswd_text)
            self.close()
