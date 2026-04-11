import sys
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtWidgets

app = QtWidgets.QApplication(sys.argv)  # Create QApplication

x = np.arange(1000)
y = np.random.normal(size=(3, 1000))

plotWidget = pg.plot(title="Three plot curves")

for i in range(3):
    plotWidget.plot(x, y[i], pen=(i,3))  ## setting pen=(i,3) automaticaly creates three different-colored pens


sys.exit(app.exec_())  # Start the Qt event loop and keep the window open
