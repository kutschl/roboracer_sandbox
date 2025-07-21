from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QTermWidget import QTermWidget


class ControlWidget(QWidget):

    def __init__(self, parent=None):
        super(ControlWidget, self).__init__(parent)
        self.setWindowTitle("Control Widget")
        self.setGeometry(100, 100, 800, 600)

        # Create a QVBoxLayout
        layout = QVBoxLayout()

        # Create a QTermWidget
        self.term_widget = QTermWidget(self)
        self.term_widget.setColorScheme("WhiteOnBlack")
        self.term_widget.setScrollBarPosition(QTermWidget.ScrollBarRight)
        layout.addWidget(self.term_widget)

        # Set the layout for the widget
        self.setLayout(layout)

        self.term_widget.startShellProgram()