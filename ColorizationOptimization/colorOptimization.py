# -*- coding: utf-8 -*-
"""
Created on Mon Nov 27 23:29:45 2017

@author: David
"""
import cv2
import colorOptimizationHelper as coHelper
from PyQt5.QtCore import QDir, QPoint, QRect, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QImageWriter, QPainter, QPen, qRgb
from PyQt5.QtWidgets import (QAction, QApplication, QColorDialog, QFileDialog,
        QInputDialog, QMainWindow, QMenu, QMessageBox, QWidget, QPushButton)
from PyQt5.QtPrintSupport import QPrintDialog, QPrinter

#############################################################################
##
## Copyright (C) 2013 Riverbank Computing Limited.
## Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
## All rights reserved.
##
## This file is part of the examples of PyQt.
##
## $QT_BEGIN_LICENSE:BSD$
## You may use this file under the terms of the BSD license as follows:
##
## "Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are
## met:
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above copyright
##     notice, this list of conditions and the following disclaimer in
##     the documentation and/or other materials provided with the
##     distribution.
##   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
##     the names of its contributors may be used to endorse or promote
##     products derived from this software without specific prior written
##     permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
## A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
## OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
## SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
## LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
## DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
## THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
## $QT_END_LICENSE$
##
#############################################################################

class ScribbleArea(QWidget):
    def __init__(self, parent=None):
        super(ScribbleArea, self).__init__(parent)

        self.setAttribute(Qt.WA_StaticContents)
        
        self.modified = False
        self.scribbling = False
        self.myPenWidth = 5
        self.myPenColor = Qt.blue
        self.image = QImage()
        self.imageClear = 0
        self.lastPoint = QPoint()
    
    def openImage(self, fileName):
        loadedImage = QImage()
        if not loadedImage.load(fileName):
            return False

        newSize = loadedImage.size().expandedTo(self.size())
        self.resizeImage(loadedImage, newSize)
        self.image = loadedImage
        self.imageClear = fileName
        self.modified = False
        self.update()
        return True
    
    def openImageM(self, fileName):
        loadedImage = QImage()
        if not loadedImage.load(fileName):
            return False

        newSize = loadedImage.size().expandedTo(self.size())
        self.resizeImage(loadedImage, newSize)
        self.image = loadedImage
        self.modified = False
        self.update()
        return True

    def saveImage(self, fileName, fileFormat):
        visibleImage = self.image
        if visibleImage.save(fileName, fileFormat):
            self.modified = False
            return True
        else:
            return False

    def setPenColor(self, newColor):
        self.myPenColor = newColor

    def setPenWidth(self, newWidth):
        self.myPenWidth = newWidth

    def clearImage(self):
        self.image.load(self.imageClear)
        self.modified = False
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.lastPoint = event.pos()
            self.scribbling = True

    def mouseMoveEvent(self, event):
        if (event.buttons() & Qt.LeftButton) and self.scribbling:
            self.drawLineTo(event.pos())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.scribbling:
            self.drawLineTo(event.pos())
            self.scribbling = False

    def paintEvent(self, event):
        painter = QPainter(self)
        dirtyRect = event.rect()
        painter.drawImage(dirtyRect, self.image, dirtyRect)

    def drawLineTo(self, endPoint):
        painter = QPainter(self.image)
        painter.setPen(QPen(self.myPenColor, self.myPenWidth, Qt.SolidLine,
                Qt.RoundCap, Qt.RoundJoin))
        painter.drawLine(self.lastPoint, endPoint)
        self.modified = True

        rad = self.myPenWidth / 2 + 2
        self.update(QRect(self.lastPoint, endPoint).normalized().adjusted(-rad, -rad, +rad, +rad))
        self.lastPoint = QPoint(endPoint)

    def resizeImage(self, image, newSize):
        if image.size() == newSize:
            return

        newImage = QImage(newSize, QImage.Format_RGB32)
        newImage.fill(qRgb(255, 255, 255))
        painter = QPainter(newImage)
        painter.drawImage(QPoint(0, 0), image)
        self.image = newImage
    
    def print_(self):
        printer = QPrinter(QPrinter.HighResolution)

        printDialog = QPrintDialog(printer, self)
        if printDialog.exec_() == QPrintDialog.Accepted:
            painter = QPainter(printer)
            rect = painter.viewport()
            size = self.image.size()
            size.scale(rect.size(), Qt.KeepAspectRatio)
            painter.setViewport(rect.x(), rect.y(), size.width(), size.height())
            painter.setWindow(self.image.rect())
            painter.drawImage(0, 0, self.image)
            painter.end()

    def isModified(self):
        return self.modified

    def penColor(self):
        return self.myPenColor

    def penWidth(self):
        return self.myPenWidth

#------------------------------------------------------------------------------

class ColorThread(QThread):
    myResult = pyqtSignal(object)

    def __init__(self,oPath,mPath):
        QThread.__init__(self)
        self.fileOPath = oPath
        self.fileMPath = mPath

    def __del__(self):
        self.wait()

    def run(self):
        imageO = cv2.imread(self.fileOPath)
        imageM = cv2.imread(self.fileMPath)

        markTF = coHelper.markMapTF(imageO,imageM)

        result = coHelper.colorOp(imageO,imageM,markTF)

        cv2.imshow("Colorization",result)
        cv2.waitKey()

        self.myResult.emit(result)
       
#------------------------------------------------------------------------------

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.saveAsActs = []

        self.scribbleArea = ScribbleArea()
        self.setCentralWidget(self.scribbleArea)

        self.createActions()
        self.createMenus()

        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.gray)
        self.setPalette(p)

        self.fileOPath = 0
        self.fileMPath = 0

        self.setWindowTitle("Colorization Using Optimization")
        self.resize(400, 390)
        
        self.bwbutton = QPushButton('Import B/W Image', self)
        self.bwbutton.setToolTip('Import a B/W image file')
        self.bwbutton.move(self.width()-120,40)
        self.bwbutton.clicked.connect(self.openO)
        
        self.bcbutton = QPushButton('Brush Color', self)
        self.bcbutton.setToolTip('Choose a brush color')
        self.bcbutton.move(self.width()-120,90)
        self.bcbutton.clicked.connect(self.penColor)
        
        self.bsbutton = QPushButton('Brush Size', self)
        self.bsbutton.setToolTip('Choose a brush size')
        self.bsbutton.move(self.width()-120,140)
        self.bsbutton.clicked.connect(self.penWidth)
        
        self.cmbutton = QPushButton('Clear Color Marks', self)
        self.cmbutton.setToolTip('Clear color marks from image')
        self.cmbutton.move(self.width()-120,190)
        self.cmbutton.clicked.connect(self.scribbleArea.clearImage)
        
        self.smbutton = QPushButton('Save Color Marks', self)
        self.smbutton.setToolTip('Save color marked image')
        self.smbutton.move(self.width()-120,240)
        self.smbutton.clicked.connect(self.maybeSave)
        
        self.icbutton = QPushButton('Import Color Marks', self)
        self.icbutton.setToolTip('Import color marked image file')
        self.icbutton.move(self.width()-120,290)
        self.icbutton.clicked.connect(self.openM)
        
        self.vrbutton = QPushButton('View/Save Results', self)
        self.vrbutton.setToolTip('View and choose to save results')
        self.vrbutton.move(self.width()-120,340)
        self.vrbutton.clicked.connect(self.color)
        
    def color(self):
        if(self.fileOPath == 0):
            QMessageBox.warning(self, "Save", "Unmarked Black & White image must be imported first.")
        elif(self.fileMPath == 0):
            QMessageBox.warning(self, "Save", "Color marks must be saved or imported first.")
        else:
            self.myThread = ColorThread(self.fileOPath,self.fileMPath)
            QMessageBox.information(self, "Processing...", "May take several minutes.\n"
                                                    "Result will appear in new window.\n"
                                                    "Close result window for save options.\n"
                                                    "Press OK to begin.")
            self.myThread.myResult.connect(self.saveResult)
            self.myThread.start()
            self.vrbutton.setEnabled(False)
        
    def saveResult(self, result):
        ret = QMessageBox.warning(self, "Colorization",
                        "Do you want to save your results?\n"
                        "File will save to current directory.\n",
                        QMessageBox.Save | QMessageBox.Discard)
        if ret == QMessageBox.Save:
            result = result*255
            cv2.imwrite('ColorResults.bmp',result)
        self.vrbutton.setEnabled(True)
    
    def closeEvent(self, event):
        if self.maybeSave():
            event.accept()
        else:
            event.ignore()

    def openO(self):
        if self.maybeSave():
            fileName, _ = QFileDialog.getOpenFileName(self, "Open File",
                    QDir.currentPath())
            if fileName:
                self.fileOPath = fileName
                self.scribbleArea.openImage(fileName)
                self.resize(400, 390)
                self.update()
                if (self.width()-120) < self.scribbleArea.image.width() and self.height() < self.scribbleArea.image.height():
                    newWidth = self.scribbleArea.image.width()+140
                    newHeight = self.scribbleArea.image.height()+20
                    self.resize(newWidth, newHeight)
                    self.update()
                    self.bwbutton.move(self.width()-120,40)
                    self.bcbutton.move(self.width()-120,90)
                    self.bsbutton.move(self.width()-120,140)
                    self.cmbutton.move(self.width()-120,190)
                    self.smbutton.move(self.width()-120,240)
                    self.icbutton.move(self.width()-120,290)
                    self.vrbutton.move(self.width()-120,340)
                elif (self.width()-120) < self.scribbleArea.image.width() and self.height() > self.scribbleArea.image.height():
                    newWidth = self.scribbleArea.image.width()+140
                    self.resize(newWidth, 390)
                    self.update()
                    self.bwbutton.move(self.width()-120,40)
                    self.bcbutton.move(self.width()-120,90)
                    self.bsbutton.move(self.width()-120,140)
                    self.cmbutton.move(self.width()-120,190)
                    self.smbutton.move(self.width()-120,240)
                    self.icbutton.move(self.width()-120,290)
                    self.vrbutton.move(self.width()-120,340)
                elif (self.width()-120) > self.scribbleArea.image.width() and self.height() < self.scribbleArea.image.height():
                    newHeight = self.scribbleArea.image.height()+20
                    self.resize(400, newHeight)
                    self.update()
                    
    def openM(self):
        if self.maybeSave():
            fileName, _ = QFileDialog.getOpenFileName(self, "Open File",
                    QDir.currentPath())
            if fileName:
                self.fileMPath = fileName
                self.scribbleArea.openImageM(fileName)
                self.resize(400, 390)
                self.update()
                if (self.width()-120) < self.scribbleArea.image.width() and self.height() < self.scribbleArea.image.height():
                    newWidth = self.scribbleArea.image.width()+140
                    newHeight = self.scribbleArea.image.height()+20
                    self.resize(newWidth, newHeight)
                    self.update()
                    self.bwbutton.move(self.width()-120,40)
                    self.bcbutton.move(self.width()-120,90)
                    self.bsbutton.move(self.width()-120,140)
                    self.cmbutton.move(self.width()-120,190)
                    self.smbutton.move(self.width()-120,240)
                    self.icbutton.move(self.width()-120,290)
                    self.vrbutton.move(self.width()-120,340)
                elif (self.width()-120) < self.scribbleArea.image.width() and self.height() > self.scribbleArea.image.height():
                    newWidth = self.scribbleArea.image.width()+140
                    self.resize(newWidth, 390)
                    self.update()
                    self.bwbutton.move(self.width()-120,40)
                    self.bcbutton.move(self.width()-120,90)
                    self.bsbutton.move(self.width()-120,140)
                    self.cmbutton.move(self.width()-120,190)
                    self.smbutton.move(self.width()-120,240)
                    self.icbutton.move(self.width()-120,290)
                    self.vrbutton.move(self.width()-120,340)
                elif (self.width()-120) > self.scribbleArea.image.width() and self.height() < self.scribbleArea.image.height():
                    newHeight = self.scribbleArea.image.height()+20
                    self.resize(400, newHeight)
                    self.update()
                    
    def save(self):
        action = self.sender()
        fileFormat = action.data()
        self.saveFile(fileFormat)

    def penColor(self):
        newColor = QColorDialog.getColor(self.scribbleArea.penColor())
        if newColor.isValid():
            self.scribbleArea.setPenColor(newColor)

    def penWidth(self):
        newWidth, ok = QInputDialog.getInt(self, "Colorization",
                "Select pen width:", self.scribbleArea.penWidth(), 1, 50, 1)
        if ok:
            self.scribbleArea.setPenWidth(newWidth)

    def about(self):
        QMessageBox.about(self, "About Colorization using Optimization",
                "<p>This interface allows a user to import a black and white "
                "image, apply color marks, and through colorization using "
                "optimization produce a fully colored image. There are options "
                "to draw new color marks on an image or import an image "
                "pre-colored with marks.</p>"
                "<p>This process is accomplished with the method introduced "
                "in the paper <b>Colorization using Optimization</b> by "
                "Anat Levin, Dani Lischinski, and Yair Weiss. It is based on "
                "the premise that neighboring pixels in space-time that have "
                "similar intensities should have similar colors.</p>")

    def createActions(self):
        self.openAct = QAction("&Open...", self, shortcut="Ctrl+O",
                triggered=self.openO)

        for format in QImageWriter.supportedImageFormats():
            format = str(format)

            text = format.upper() + "..."

            action = QAction(text, self, triggered=self.save)
            action.setData(format)
            self.saveAsActs.append(action)

        self.printAct = QAction("&Print...", self,
                triggered=self.scribbleArea.print_)

        self.exitAct = QAction("E&xit", self, shortcut="Ctrl+Q",
                triggered=self.close)

        self.penColorAct = QAction("&Pen Color...", self,
                triggered=self.penColor)

        self.penWidthAct = QAction("Pen &Width...", self,
                triggered=self.penWidth)

        self.clearScreenAct = QAction("&Clear Screen", self, shortcut="Ctrl+L",
                triggered=self.scribbleArea.clearImage)

        self.aboutAct = QAction("&About", self, triggered=self.about)

        self.aboutQtAct = QAction("About &Qt", self,
                triggered=QApplication.instance().aboutQt)

    def createMenus(self):
        self.saveAsMenu = QMenu("&Save As", self)
        for action in self.saveAsActs:
            self.saveAsMenu.addAction(action)

        fileMenu = QMenu("&File", self)
        fileMenu.addAction(self.openAct)
        fileMenu.addMenu(self.saveAsMenu)
        fileMenu.addAction(self.printAct)
        fileMenu.addSeparator()
        fileMenu.addAction(self.exitAct)

        optionMenu = QMenu("&Options", self)
        optionMenu.addAction(self.penColorAct)
        optionMenu.addAction(self.penWidthAct)
        optionMenu.addSeparator()
        optionMenu.addAction(self.clearScreenAct)

        helpMenu = QMenu("&Help", self)
        helpMenu.addAction(self.aboutAct)
        helpMenu.addAction(self.aboutQtAct)

        self.menuBar().addMenu(fileMenu)
        self.menuBar().addMenu(optionMenu)
        self.menuBar().addMenu(helpMenu)

    def maybeSave(self):
        if self.scribbleArea.isModified():
            ret = QMessageBox.warning(self, "Colorization",
                        "The image has been modified.\n"
                        "Do you want to save your changes?",
                        QMessageBox.Save | QMessageBox.Discard |
                        QMessageBox.Cancel)
            if ret == QMessageBox.Save:
                return self.saveFile('bmp')
            elif ret == QMessageBox.Cancel:
                return False

        return True

    def saveFile(self, fileFormat):
        initialPath = QDir.currentPath() + '/untitled.' + fileFormat

        fileName, _ = QFileDialog.getSaveFileName(self, "Save As", initialPath,
                "%s Files (*.%s);;All Files (*)" % (fileFormat.upper(), fileFormat))
        self.fileMPath = fileName
        if fileName:
            self.fileMPath = fileName
            return self.scribbleArea.saveImage(fileName, fileFormat)

        return False
    
#------------------------------------------------------------------------------

if __name__ == '__main__':

    import sys

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
