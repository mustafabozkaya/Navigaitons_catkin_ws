"""Dialog-Style application."""

import sys

from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QDialog
from PyQt5.QtWidgets import QDialogButtonBox
from PyQt5.QtWidgets import QFormLayout
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout


class Dialog(QDialog): #
    """Dialog."""

    def __init__(self, parent=None):
        """Initializer."""
        super().__init__(parent)
        self.setWindowTitle('QDialog') # set the title of the window to QDialog (the title of the dialog)
        dlgLayout = QHBoxLayout() #create a horizontal layout for the dialog box to be displayed in the center of the screen

        formLayout = QFormLayout() # create a form layout to hold the line edits
        formLayout.addRow('Name:', QLineEdit()) # add a row to the form layout with the label 'Name:' and the line edit
        formLayout.addRow('Age:', QLineEdit()) 
        formLayout.addRow('Job:', QLineEdit())
        formLayout.addRow('Hobbies:', QLineEdit())
        dlgLayout.addLayout(formLayout) # add the form layout to the dialog layout
        btns = QDialogButtonBox() # create a dialog button box to hold the buttons for the dialog box
        btns.setStandardButtons(
            QDialogButtonBox.Cancel | QDialogButtonBox.Ok) # set the buttons to be cancel and ok
        dlgLayout.addWidget(btns) # add the button box to the dialog layout
        self.setLayout(dlgLayout) # set the layout of the dialog box to the dialog layout


if __name__ == '__main__':
    app = QApplication(sys.argv)
    dlg = Dialog()
    dlg.show()
    sys.exit(app.exec_())


    