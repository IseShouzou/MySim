import sys
import time
import numpy as np

from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *


#from PyQt6.QtCore import *
#from PyQt6.QtWidgets import *
#from PyQt6.QtOpenGL import *
#from PyQt6.QtOpenGLWidgets import QOpenGLWidget

#from MyGLWidget import *
#from MyFunc import *

import pyqtgraph as pg


class GridLayoutWidget( QWidget ) :
    
    def __init__( self, parent = None ) :
        super().__init__( parent )
        self.layout = QGridLayout( self )

    def addWidget( self, widget, *args, **kwargs ) :
        self.layout.addWidget( widget, *args, **kwargs )
        return widget


class HBoxLayoutWidget( QWidget ) :
    
    def __init__( self, parent = None ) :
        super().__init__( parent )
        self.layout = QHBoxLayout( self )

    def addWidget( self, widget, *args, **kwargs ) :
        self.layout.addWidget( widget, *args, **kwargs )
        return widget

class VBoxLayoutWidget( QWidget ) :
    
    def __init__( self, parent = None ) :
        super().__init__( parent )
        self.layout = QVBoxLayout( self )

    def addWidget( self, widget, *args, **kwargs ) :
        self.layout.addWidget( widget, *args, **kwargs )
        return widget





class ManyTHGraphs( QWidget ) :

    def __init__( self, parent, iniTime = 0.0, dspTime = 5.0, graphData = None ) :

        super().__init__( parent )
        
        self.layout = QGridLayout( self )
        self.setLayout( self.layout )

        self.graphs = []
        self.nPlots = []
        for d in graphData :
            self.nPlots.append( len( d[2]) )
            self.graphs.append( THGraph( d[2], d[3], iniTime, dspTime, self ) )
            self.layout.addWidget( self.graphs[-1], d[0], d[1] )

    def clearData( self ) :
        for graph in self.graphs :
            graph.clearData()

    def addData( self, data ) :
        cnt = 1
        for graph, n in zip( self.graphs, self.nPlots ) :
            graph.addData( [ data[0], *data[ cnt : cnt + n ] ] )
            cnt += n

    def setData( self, data ) :
        cnt = 1
        for graph, n in zip( self.graphs, self.nPlots ) :
            graph.setData( np.vstack( [ data[ 0:1, : ], data[ cnt : cnt + n, : ] ] ) )
            cnt += n



class THGraph( pg.GraphicsLayoutWidget ) :

    default_Tittle   = ( 'AAA', 'BBB', 'CCC' )
    default_IniTime  = 0.0
    default_DspTime = 3.0
    default_Yrange   = ( -1.0, 1.0 )
    
    def __init__( self, tittle = None, yRange = None, iniTime = None, dspTime = None, parent = None ) :
        
        super().__init__( parent )

        tittle        = self.default_Tittle  if tittle   is None else tittle
        self.iniTime  = self.default_IniTime if iniTime  is None else iniTime
        self.dspTime  = self.default_DspTime if dspTime  is None else dspTime
        yRange        = self.default_Yrange  if yRange   is None else yRange
        
        if len( yRange ) == 2 :
            yRange1 = yRange
            yRange2 = yRange
        else :
            yRange1 = yRange[0:2]
            yRange2 = yRange[2:4]
        
        self.nPlot = len( tittle )
        
        self.graph = pg.PlotItem()
        self.graph.setRange( xRange = ( self.iniTime, self.iniTime + self.dspTime ), yRange = yRange1 )
        self.addItem( self.graph, 1, 0, 1, self.nPlot )
        
        for t in tittle :
            if t[-1] == '>' :
                p2 = pg.ViewBox()
                self.graph.showAxis('right')
                self.graph.scene().addItem( p2 )
                self.graph.getAxis('right').linkToView(p2)
                p2.setXLink( self.graph )
                p2.setRange( yRange = yRange2 )
                p2.sigRangeChanged.connect( lambda: p2.setGeometry(self.graph.vb.sceneBoundingRect() ) )
                break        
        
        self.plots = []
        self.plotData = [ [] ]
        for i in range( self.nPlot ) :
            c = [ 'r', 'g', 'b', 'y', 'm', 'c', 'w' ] [ i % 7 ]
            self.addLabel( f'-- {tittle[i]}', 0, i, 1, 1, color = c )
            self.plots.append( pg.PlotDataItem( pen = pg.mkPen( color = c ) ) )
            if tittle[i][-1] == '>' :
                p2.addItem( self.plots[i] )
            else :
                self.graph.addItem( self.plots[i] )
            self.plotData.append( [] )
            
    def setYRange( self, ymin, ymax ) :
        self.graph.setRange( yRange = ( ymin, ymax ) )

    def clearData( self ) :
        for pd in self.plotData :
            pd.clear()
        self.graph.setRange( xRange = ( self.iniTime, self.iniTime + self.dspTime ) )
        self.update()
        
    def update( self ) :
        for i in range( self.nPlot ) :
            self.plots[i].setData( self.plotData[0], self.plotData[i+1] )

    def addData( self, data ) :
        
        for pd, d in zip( self.plotData, data ) :
            pd.append( d )
        t0 = max( self.iniTime, self.plotData[0][-1] - self.dspTime )
        self.graph.setRange( xRange = ( t0, t0 + self.dspTime ) )       
        
        while self.plotData[0][0] < t0 :
            for pd in self.plotData :
                del pd[0]
            
        self.update()

    def setData( self, data ) :
        self.plotData = data
        self.update()




class MySlider( QWidget ) :
    
    valueChanged = pyqtSignal()
    
    def __init__(self, index, curVal, minVal, maxVal, nDiv = 200 ):
        
        super().__init__()
        
        self.index  = index
        self.curVal = curVal
        self.minVal = minVal
        self.maxVal = maxVal
        self.nDiv = nDiv

        d = abs( ( maxVal - minVal ) / nDiv )
        if d > 0.9 :
            self.decimal = None
        else :
            self.decimal = int( - np.log10( d ) ) + 2

        self.layout = QHBoxLayout()
        self.setLayout( self.layout )

        self.slider = QSlider( Qt.Orientation.Horizontal )
        self.slider.setMinimum( 0 )
        self.slider.setMaximum( nDiv )
        
        self.curValLabel = QLabel( self._t( self.curVal ) )
        self.minValLabel = QLineEdit( str( minVal ) ) 
        self.maxValLabel = QLineEdit( str( maxVal ) )

        self.curValLabel.setAlignment( Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter )
        self.minValLabel.setAlignment( Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter )
        self.maxValLabel.setAlignment( Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter )

        self.slider.valueChanged.connect( self.sliderValueChanged )
        self.minValLabel.returnPressed.connect( self.valLabelReturnPressed )
        self.maxValLabel.returnPressed.connect( self.valLabelReturnPressed )

        self.layout.addWidget( QLabel( index )  )
        self.layout.addWidget( self.curValLabel )
        self.layout.addWidget( self.minValLabel )
        self.layout.addWidget( self.slider      )
        self.layout.addWidget( self.maxValLabel )
        for k, v in enumerate( [ 1, 1, 1, 5, 1 ] ) :
            self.layout.setStretch( k, v )

        self.setSlider()

    def _t( self, val ) :
        if self.decimal is None :
            return str( round( val ) )
        else :
            f = '{:.' + str( self.decimal ) + 'f}'
            return f.format( val )

    def setSlider( self ) :
        #print( 'setSlider' )
        v0 = self.minVal
        dv = self.maxVal - self.minVal
        val = int( ( self.curVal - v0 ) / dv * self.nDiv )
        self.slider.setValue( val )
        
    def getValue( self ) :
        return self.curVal
            
    def setValue( self, value ) :
        self.curVal = value
        self.setSlider()
            
    def sliderValueChanged( self ) :
        #print( 'sliderValueChanged' )
        v0 = self.minVal
        dv = self.maxVal - self.minVal
        self.curVal = v0 + self.slider.value() / self.nDiv * dv
        self.curValLabel.setText( self._t( self.curVal ) )
        self.valueChanged.emit()
        
    def valLabelReturnPressed( self ) :
        print( 'minValLabelReturnPressed' )
        mn = float( self.minValLabel.text() )
        mx = float( self.maxValLabel.text() )
        if abs( mx - mn ) < 1.0e-6 :
            self.minValLabel.setText( str( self.minVal ) )
            self.maxValLabel.setText( str( self.maxVal ) )
        else :
            self.minVal = mn
            self.maxVal = mx
            self.setSlider()
        

class MySliders( QWidget ) :
    
    valueChanged = pyqtSignal( object )
    
    def __init__(self, *args ):
        
        super().__init__()
        
        self.sliders = [ MySlider( *arg ) for arg in args ]
        self.layout = QVBoxLayout()
        self.setLayout( self.layout )
        for s in self.sliders :
            self.layout.addWidget( s )
            s.valueChanged.connect( self.sliderValueChanged )

    def getValue( self ) :
        return [ s.getValue() for s in self.sliders ]
    
    def setValue( self, values ) :
        for s, v in zip( self.sliders, values ) : 
            s.setValue( v )
    
    def sliderValueChanged( self ) :
        #print( 'sliderValueChanged' )
        self.valueChanged.emit( self.getValue() )


class MySpinBox( QWidget ) :
    
    widthRate = [ 2, 2, 1 ]    
    valueChanged = pyqtSignal()

    def __init__(self, text, val, dVal ) :
        #print( 'MySpinBox init' )
    
        super().__init__()
    
        layout = QHBoxLayout()
        self.setLayout( layout )
                
        layout.setSpacing( 0 )        
        layout.setContentsMargins( 0, 0, 0, 0 )        
                
        self.spinbox = QDoubleSpinBox()
        self.lineEdit = QLineEdit( dVal )
        self.decimals = len( val.split('.')[1] ) if '.' in val else 0  
        
        self.spinbox.setAlignment( Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter )
        self.lineEdit.setAlignment( Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter )
        
        self.spinbox.setRange( -99999.9, 99999.9)
        self.spinbox.setDecimals( self.decimals )      
        self.spinbox.setValue( float( val ) )
        self.spinbox.setSingleStep( float( dVal ) )
        self.spinbox.valueChanged.connect( lambda : self.valueChanged.emit() )
                
        #self.lineEdit.editingFinished.connect( self.lineEditChanged )
        self.lineEdit.textChanged.connect( self.lineEditChanged )
        
        
        if  text is not None :   
            self.label = QLabel( text, self )
            self.label.setAlignment( Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter )
            widgets = [ self.label, self.spinbox, self.lineEdit ]
        else :
            widgets = [ self.spinbox, self.lineEdit ]
                
        for i, w in enumerate( widgets ) :
            layout.addWidget( w )
            layout.setStretch( i, MySpinBox.widthRate[i] )
            w.setSizePolicy(QSizePolicy.Policy.Expanding , QSizePolicy.Policy.Expanding )
            w.setMinimumSize( 1, 1 )
       
    def  lineEditChanged( self ) :
        #print( 'lineEditChanged' )
        self.spinbox.setSingleStep( float( self.lineEdit.text() ) )
        
    def  value( self ) :
        return self.spinbox.value()
            
    def  setValue( self, val ) :
        #print( 'setValue' )
        self.spinbox.setValue( round( val, self.decimals ) )








class MainWindow( QMainWindow ) :
    
    def __init__(self):
        
        super().__init__()
        
        self.setGeometry( 300, 100, 1000, 600)
        self.setWindowTitle( 'Test' )
        
        mainWidget = QWidget()
        self.setCentralWidget( mainWidget )
        mainVBoxLO = QVBoxLayout()
        mainWidget.setLayout( mainVBoxLO )
 
        btnLW = VBoxLayoutWidget()
 
        self.btnA = QPushButton( 'START', self )
        self.btnA.clicked.connect( self.buttonAClicked )
        btnLW.addWidget( self.btnA )
        
        mainVBoxLO.addWidget( btnLW )
        
        self.graph = ManyTHGraphs( self, 0.0, 5.0,
                                  [ [  0, 0, ( 'AAA', 'BBB', 'CCC' ), ( -10.0, 10.0 ) ],
                                    [  0, 1, ( 'XXX', 'YYY', 'ZZZ' ), ( -10.0, 10.0 ) ] ] )
        
        mainVBoxLO.addWidget( self.graph )
        
        self.curTime = 0.0
        
        
        
    def buttonAClicked( self ) :
        print( 'buttonAClicked' )

        if self.btnA.text() == 'STOP' :
            self.btnA.setText( 'START' )
            self.timer.stop()
            
        elif self.btnA.text() == 'START' :
            self.btnA.setText( 'STOP' )

            self.iniTime = time.time() - self.curTime
            self.curTime = time.time() - self.iniTime
        
            self.timer = QTimer()
            self.timer.setInterval( 100 )
            self.timer.timeout.connect( self.step )
            self.timer.start()
        
        
    def step( self ) :
        
        self.curTime = time.time() - self.iniTime

        print( self.curTime )
        self.graph.addData( [ self.curTime,
                              np.cos( 1.0 * self.curTime ),
                              np.cos( 2.0 * self.curTime ),
                              np.cos( 3.0 * self.curTime ),
                              np.cos( 4.0 * self.curTime ),
                              np.cos( 5.0 * self.curTime ),
                              np.cos( 6.0 * self.curTime ) ]  )
        

        
if __name__ == '__main__':
    app = QApplication( sys.argv )
    mainwindow = MainWindow()
    mainwindow.show()  
    app.exec()

    