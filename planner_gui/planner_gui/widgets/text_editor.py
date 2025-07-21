import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPlainTextEdit, QTextEdit,
    QPushButton, QWidget, QVBoxLayout, QHBoxLayout, QFileDialog
)
from PyQt5.QtGui import QFont, QPainter, QColor, QTextFormat
from PyQt5.QtCore import Qt, QSize

from styles import plain_text_edit_stylesheet, button_thin_stylesheet

class LineNumberArea(QWidget):
    def __init__(self, editor):
        super().__init__(editor)
        self.codeEditor = editor

    def sizeHint(self):
        return QSize(self.codeEditor.lineNumberAreaWidth(), 0)

    def paintEvent(self, event):
        self.codeEditor.lineNumberAreaPaintEvent(event)

class CodeEditor(QPlainTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.lineNumberArea = LineNumberArea(self)

        self.blockCountChanged.connect(self.updateLineNumberAreaWidth)
        self.updateRequest.connect(self.updateLineNumberArea)
        self.cursorPositionChanged.connect(self.highlightCurrentLine)

        self.updateLineNumberAreaWidth(0)
        self.highlightCurrentLine()

    def lineNumberAreaWidth(self):
        # Calculate the width needed for the line number area.
        digits = len(str(max(1, self.blockCount())))
        space = 3 + self.fontMetrics().width('9') * digits
        return space

    def updateLineNumberAreaWidth(self, _):
        self.setViewportMargins(self.lineNumberAreaWidth(), 0, 0, 0)

    def updateLineNumberArea(self, rect, dy):
        if dy:
            self.lineNumberArea.scroll(0, dy)
        else:
            self.lineNumberArea.update(0, rect.y(), self.lineNumberArea.width(), rect.height())

        if rect.contains(self.viewport().rect()):
            self.updateLineNumberAreaWidth(0)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        cr = self.contentsRect()
        self.lineNumberArea.setGeometry(cr.left(), cr.top(), self.lineNumberAreaWidth(), cr.height())

    def highlightCurrentLine(self):
        extraSelections = []

        if not self.isReadOnly():
            selection = QTextEdit.ExtraSelection()
            lineColor = QColor(60, 60, 60)
            selection.format.setBackground(lineColor)
            selection.format.setProperty(QTextFormat.FullWidthSelection, True)
            selection.cursor = self.textCursor()
            selection.cursor.clearSelection()
            extraSelections.append(selection)

        self.setExtraSelections(extraSelections)

    def lineNumberAreaPaintEvent(self, event):
        painter = QPainter(self.lineNumberArea)
        painter.fillRect(event.rect(), QColor(50, 50, 50))
        
        # Use a smaller NotoMono font for line numbers
        font = QFont("NotoMono", 8)
        painter.setFont(font)
        
        block = self.firstVisibleBlock()
        blockNumber = block.blockNumber()
        top = self.blockBoundingGeometry(block).translated(self.contentOffset()).top()
        bottom = top + self.blockBoundingRect(block).height()

        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                number = str(blockNumber + 1)
                # Set a grayish color with reduced opacity
                painter.setPen(QColor(150, 150, 150, 180))
                painter.drawText(
                    0, int(top),
                    self.lineNumberArea.width() - 3,
                    self.fontMetrics().height(),
                    Qt.AlignRight, number
                )
            block = block.next()
            top = bottom
            bottom = top + self.blockBoundingRect(block).height()
            blockNumber += 1

class TextEditorWidget(QWidget):
    def __init__(self, file_path=None, parent=None):
        super().__init__(parent)
        self.file_path = file_path

        # Create header with Refresh and Save buttons
        header_widget = QWidget()
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(5, 5, 0, 5)
        header_layout.setSpacing(5)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.setStyleSheet(button_thin_stylesheet)
        self.refresh_btn.clicked.connect(self.refresh_file)

        self.save_btn = QPushButton("Save")
        self.save_btn.setStyleSheet(button_thin_stylesheet)
        self.save_btn.clicked.connect(self.save_file)

        header_layout.addWidget(self.refresh_btn)
        header_layout.addWidget(self.save_btn)
        header_layout.addStretch()
        header_widget.setLayout(header_layout)

        # Create the CodeEditor widget (with line numbers) using the plain text edit stylesheet.
        self.editor = CodeEditor()
        self.editor.setStyleSheet(plain_text_edit_stylesheet)
        self.editor.setLineWrapMode(QPlainTextEdit.NoWrap)
        # Set a monospace font
        font = QFont("NotoMono", 10)
        self.editor.setFont(font)

        # Organize header and editor in a vertical layout
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(header_widget)
        layout.addWidget(self.editor)

        self.setLayout(layout)

        # If a file path is provided, load the file
        if self.file_path:
            self.load_file(self.file_path)

    def load_file(self, path: str):
        """Load file contents into the editor."""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                text = f.read()
            self.editor.setPlainText(text)
            self.file_path = path
        except Exception as e:
            self.editor.setPlainText(f"Error loading file: {e}")

    def refresh_file(self):
        """Refresh the editor content from file."""
        if self.file_path:
            self.load_file(self.file_path)

    def save_file(self):
        """Save the current editor content back to the file.
        Optionally, prompt for new filename if no file_path is set."""
        if not self.file_path:
            # Prompt the user to choose a file path if not already set
            path, _ = QFileDialog.getSaveFileName(self, "Save File")
            if path:
                self.file_path = path
            else:
                return
        try:
            with open(self.file_path, 'w', encoding='utf-8') as f:
                f.write(self.editor.toPlainText())
        except Exception as e:
            self.editor.setPlainText(f"Error saving file: {e}")