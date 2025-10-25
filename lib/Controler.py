import sys
import time

import numpy as np

import pygame
from pygame.locals import *

from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *



class Joystick( QThread ) :
    
    buttonSignal = pyqtSignal( str )
    axisSignal = pyqtSignal( object )
    
    def __init__( self, parent = None) :
        print( '__init__ Joystick' )
        
        super().__init__()
        
        self.parent = parent
        pygame.init()
        
        self.joysticks = []
        self.initVals = {}
        self.initVals[ 'X' ] = 0.0
        self.initVals[ 'Y' ] = 0.0
            
        for k in range( pygame.joystick.get_count() ) :
            self.joysticks.append( pygame.joystick.Joystick( k ) )
            #print( pygame.joystick.Joystick( k ).get_name() )
        
        self.XYZB      = [ 0.0, 0.0, 0.0, 0.0 ]
        self.XYZT      = [ 0.0, 0.0, 0.0, 0.0 ]
        
        self.start()
        
              
    def reset( self ) :
        
        self.initVals[ 'X' ] = 0.0
        self.initVals[ 'Y' ] = 0.0
        
        event = pygame.event.Event( pygame.JOYAXISMOTION )
        for k, js in enumerate( self.joysticks ) :
            
            event.instance_id = k
            pygame.event.post( event )
            
            if js.get_name() == 'Simple HID Device Demo' :
                self.initVals[ 'X' ] = js.get_axis( 0 )
                self.initVals[ 'Y' ] = js.get_axis( 1 )
                
            
        
    def run( self ) :
        #print( 'run' )
        
        done = False
        
        while not done:
            pygame.time.wait( 10 )
            for event in pygame.event.get() :
                #if event.type == pygame.QUIT :
                #    done = True  # Flag that we are done so we exit this loop.
                CMD = ''
                if event.type == pygame.JOYBUTTONDOWN :
                    #print("Joystick button pressed.")
                    js     = self.joysticks[ event.instance_id ]
                    jsName = js.get_name()
                
                    btnId = event.button
                    
                    if jsName == 'PS4 Controller' :
                        if    btnId == 13 :  CMD = 'THR_UP'
                        elif  btnId == 12 :  CMD = 'THR_DN'
                        elif  btnId == 10 :  CMD = 'trmY_UP'
                        elif  btnId ==  9 :  CMD = 'trmY_DN'
                        #elif  btnId ==  0 :  CMD = 'BRAKE'
                        elif  btnId ==  1 :  CMD = 'SB'
                        elif  btnId ==  0 :  CMD = 'FLAP'
                        elif  btnId ==  2 :  CMD = 'GEAR'
                    
                    elif jsName == 'PLAYSTATION(R)3 Controller' :
                        if    btnId ==  7 :  CMD = 'THR_UP'
                        elif  btnId ==  6 :  CMD = 'THR_DN'
                        elif  btnId == 11 :  CMD = 'trmY_UP'
                        elif  btnId == 10 :  CMD = 'trmY_DN'
                        elif  btnId == 12 :  CMD = 'BRAKE'
                        elif  btnId == 13 :  CMD = 'SB'
                        elif  btnId == 14 :  CMD = 'FLAP'
                        elif  btnId == 15 :  CMD = 'GEAR'
                        
                    elif jsName == 'Simple HID Device Demo' :
                        if    btnId ==  1 :  CMD = 'FLAP_UP'
                        elif  btnId ==  2 :  CMD = 'SB_CLOSE'
                        elif  btnId ==  0 :  CMD = 'GEAR_UP'
                        
                    self.buttonSignal.emit( CMD )
                        
                elif event.type == pygame.JOYBUTTONUP :
                    #print("JOYBUTTONUP")
                    js     = self.joysticks[ event.instance_id ]
                    jsName = js.get_name()
                    btnId = event.button
                    
                    if jsName == 'Simple HID Device Demo' :
                        if    btnId ==  1 :  CMD = 'FLAP_DN'
                        elif  btnId ==  2 :  CMD = 'SB_OPEN'
                        elif  btnId ==  0 :  CMD = 'GEAR_DN'
                    self.buttonSignal.emit( CMD )

                elif event.type == pygame.JOYAXISMOTION :
                    #print("Joystick JOYAXISMOTION.")                    
                    js     = self.joysticks[ event.instance_id ]
                    jsName = js.get_name()
                    if jsName == 'PS4 Controller' :
                        #print( 'PS4 Controller' )
                        x =   js.get_axis( 2 )
                        if abs( x ) < 0.05 : x = 0.0
                        elif x > 0.0       : x = 1.0 * ( x - 0.05 )
                        else               : x = 1.0 * ( x + 0.05 )
                        y = - js.get_axis( 3 )
                        y *= 0.5
                        z =   js.get_axis( 0 )
                        b =   js.get_axis( 1 )
                        self.XYZTB = [ x, y, 0.0, t, b ]
                        
                        
                        
                    elif jsName == 'PLAYSTATION(R)3 Controller' :
                        #print( 'PLAYSTATION(R)3 Controller' )
                        x =   js.get_axis( 2 )
                        y = - js.get_axis( 3 )
                        z =   js.get_axis( 0 )
                        b =   js.get_axis( 1 )
                        self.XYZTB = [ x, y, z, t, b ]
                        
                    elif jsName == 'Simple HID Device Demo' :
                        #print( 'Simple HID Device Demo' )
                                                
                        x =   js.get_axis( 0 ) - self.initVals[ 'X' ]
                        if   x >  0.10 : x -= 0.10
                        elif x > -0.10 : x =  0.0
                        else           : x += 0.10
                        
                        y = - js.get_axis( 1 ) + self.initVals[ 'Y' ]
                        if   y >  0.10 : y -= 0.10
                        elif y > -0.10 : y =  0.0
                        else           : y += 0.10
                        
                        z = 0.0
                        t = ( js.get_axis( 2 ) + 1.0 ) * 0.5
                        b = ( js.get_axis( 3 ) + 1.0 ) * 0.5
                        
                        #print( x, y )
                        #self.XYZTB = [ x*0.8, y*0.5, z, t, b ]
                        self.XYZTB = [ x, y, z, t, b ]
                        
                    self.axisSignal.emit( self.XYZTB )


                elif event.type == pygame.JOYHATMOTION :
                    #print( 'JOYHATMOTION')   
                    js     = self.joysticks[ event.instance_id ]
                    jsName = js.get_name()
                    
                    if jsName == 'Simple HID Device Demo' :
                        #print( 'Simple HID Device Demo' )
                        val = event.value
                        txt = 'XXX'
                        CMD = ''
                        if val[0] == 0 :
                            if   val[1] ==  1 : CMD = 'trmY_UP' # txt = 'up'
                            elif val[1] == -1 : CMD = 'trmY_DN' # txt = 'down'
                        elif val[1] == 0 :
                            if   val[0] == -1 : CMD = 'trmX_RH' # txt = 'right'
                            elif val[0] ==  1 : CMD = 'trmX_LH' # txt = 'left'
                            
                        if CMD != '' :
                            self.buttonSignal.emit( CMD )   
                            


class ControlPanel( QWidget ) :
    
    signal = pyqtSignal()
    
    def __init__(self, parent = None ) :
    
        super().__init__()
            
        self.parent = parent

        self.aircraft = None

        self.resizeFlag = False
        self.mousePressed = False
        self.setMouseTracking( True )
        self.mousePos = QPointF( 0.0, 0.0 )
        self.state = 0
        
        self.brakeOn = False
        self.brakeVal = 0.8
        self.brakeMax = 0.9
        
        
#     def controlValue( self ) :
#         print( 'controlValue' )
#                 
#         s = self.stick.center() - self.stickRect.center()        
#         X =   s.x() / self.dX * 2.0
#         Y = - s.y() / self.dY * 2.0
#                
#         p = self.pedal.center() - self.pedalRect.center()
#         P =  p.x() / self.dZ * 2.0
#         
#         t = self.throtRect.bottom() - self.throt.bottom()
#         T =  t / ( self.throtRect.height() - self.throt.height() )
#         
#         b = self.brakeRect.bottom() - self.brake.bottom()
#         B =  b / ( self.brakeRect.height() - self.brake.height() )
#         
#         Xtrim =   ( self.trimX.x1() - self.stickRect.center().x() ) / self.dX * 2.0
#         Ytrim = - ( self.trimY.y1() - self.stickRect.center().y() ) / self.dY * 2.0
#         Ptrim =   ( self.trimZ.x1() - self.pedalRect.center().x() ) / self.dZ * 2.0
#         
#         return np.array( [ X, Y, P, T, B ] )
    

    @property
    def XYZpos( self ) :
        s = self.stick.center() - self.stickRect.center()        
        X =   s.x() / self.dX * 2.0
        Y = - s.y() / self.dY * 2.0  
        p = self.pedal.center() - self.pedalRect.center()
        P =  p.x() / self.dZ * 2.0
        return np.array( [ X, Y, P ] )
        #return ( X, Y, P )
        
    @XYZpos.setter
    def XYZpos( self, value ) :
        #print( 'XYZpos.setter', value )
        X, Y, P = value
        x =   max( -1.0, min( 1.0, X ) ) * 0.5
        y = - max( -1.0, min( 1.0, Y ) ) * 0.5
        
        x *= self.stickRect.width()  - self.stick.width()
        y *= self.stickRect.height() - self.stick.height()
        #self.stick = self.stickRect.center() + QPointF( x, y )
        self.stick.moveCenter( self.stickRect.center() + QPointF( x, y ) )
        p = max( -1.0, min( 1.0, P ) ) * 0.5 + 0.5
        p *= self.pedalRect.width() - self.pedal.width()         
        self.pedal.moveLeft( self.pedalRect.left() + p )
        #self.update()
        
    @property
    def XYZtrm( self ) :
        Xtrim =   ( self.trimX.x1() - self.stickRect.center().x() ) / self.dX * 2.0
        Ytrim = - ( self.trimY.y1() - self.stickRect.center().y() ) / self.dY * 2.0
        Ptrim =   ( self.trimZ.x1() - self.pedalRect.center().x() ) / self.dZ * 2.0
        return np.array( [ Xtrim, Ytrim, Ptrim ] )
        #return ( Xtrim, Ytrim, Ptrim )
        
    @XYZtrm.setter
    def XYZtrm( self, value ) :
        X, Y, P = value
        
        y1, y2 = self.trimX.y1(), self.trimX.y2()
        x = self.stickRect.center().x() + X / 2.0 * self.dX 
        self.trimX.setLine( x, y1, x, y2 )
                
        x1, x2 = self.trimY.x1(), self.trimY.x2()
        y = self.stickRect.center().y() - Y / 2.0 * self.dY 
        self.trimY.setLine( x1, y, x2, y )
        
        y1, y2 = self.trimZ.y1(), self.trimZ.y2()
        x = self.pedalRect.center().x() + P / 2.0 * self.dZ 
        self.trimZ.setLine( x, y1, x, y2 )
        
    @property
    def thrPos( self ) :
        t = self.throtRect.bottom() - self.throt.bottom()
        T =  t / ( self.throtRect.height() - self.throt.height() )
        return T
    
    @thrPos.setter
    def thrPos( self, value ) :
        #print( 'thrPos.setter' )
        t = max( 0.0, min( 1.0, value ) )          
        t *= self.throtRect.height() - self.throt.height()         
        self.throt.moveBottom( self.throtRect.bottom() - t )

    @property
    def brkPos( self ) :
        b = self.brakeRect.bottom() - self.brake.bottom()
        B =  b / ( self.brakeRect.height() - self.brake.height() )
        return B

    @brkPos.setter
    def brkPos( self, value ) :
        #print( 'brkPos.setter' )
        b0 = self.brkPos
        b = max( 0.0, min( 1.0, value ) )
        
#         if b0 < self.brakeMax and b > self.brakeMax :
#             self.brakeOn = not self.brakeOn
# 
#         if self.brakeOn :
#             b = max( self.brakeVal, b )
            
        b *= self.brakeRect.height() - self.brake.height()         
        self.brake.moveBottom( self.brakeRect.bottom() - b )
        

    def mouseReleaseEvent( self, event ) :
        #print( 'mouseReleaseEvent' )
        
        if event.button() == Qt.MouseButton.LeftButton :
            
            if self.state == 1 :       # Stick
                #x = self.trimX.x1()
                #y = self.trimY.y1()
                x = self.stickRect.center().x()
                y = self.stick.center().y()
                self.stick.moveCenter( QPointF( x, y ) )
                
            elif self.state == 2 :      # Pedal
                #x = self.trimZ.x1()
                #y = self.pedal.center().y()
                x = self.pedalRect.center().x()
                y = self.pedalRect.center().y()
                self.pedal.moveCenter( QPointF( x, y ) )
                
            elif self.state == 4 :      # Brake           
                Y09 = self.brakeMax * self.Vmin + ( 1.0 - self.brakeMax ) * self.Vmax
                Y08 = self.brakeVal * self.Vmin + ( 1.0 - self.brakeVal ) * self.Vmax
                if self.brake.top() < Y09 :
                    if self.brakeOn :
                        self.brakeMin = self.Vmax
                        self.brakeOn = False
                    else :
                        self.brakeMin = Y08
                        self.brakeOn = True
                self.brake.moveTop( self.brakeMin )
                        
            self.state = 0
            self.mousePressed = False
        
        elif event.button() == Qt.MouseButton.RightButton :
            pass
        
        self.update()
        
    def mousePressEvent( self, event ) :
        #print( 'mousePressEvent', event.buttons() )
        
        if event.button() == Qt.MouseButton.LeftButton :
            
            self.mousePos = QPointF( event.pos() )
            self.mousePressed = True
            
            self.state = 0
            if self.stick.contains( self.mousePos ) :
                self.state = 1
            elif self.pedal.contains( self.mousePos ) :
                self.state = 2
            elif self.throt.contains( self.mousePos ) :
                self.state = 3
            elif self.brake.contains( self.mousePos ) :
                self.state = 4
    
        elif event.button() == Qt.MouseButton.RightButton :
        
            s = self.stick.center()
            dx = s.x() - self.trimX.x1()
            dy = s.y() - self.trimY.y1()
            self.trimX = self.trimX.translated( dx , 0.0 )
            self.trimY = self.trimY.translated( 0.0, dy  )
                    
            p = self.pedal.center()
            dx = p.x() - self.trimZ.x1()
            self.trimZ = self.trimZ.translated( dx , 0.0 )
            
        self.signal.emit()
        self.update()
          
          
    def mouseMoveEvent( self, event ) :
        #print( 'mouseMoveEvent' )
                   
        if not self.mousePressed : return
                   
        dp = QPointF( event.pos() ) - self.mousePos
        self.mousePos = QPointF( event.pos() )

        if self.state == 1 :
            c = self.stick.center() + dp            
            x = max( self.Xmin, min( self.Xmax, c.x() ) )
            y = max( self.Ymin, min( self.Ymax, c.y() ) )
            self.stick.moveCenter( QPointF( x, y) )
            
        elif self.state == 2 :
            p = self.pedal.x() + dp.x()
            self.pedal.moveLeft( max( self.Zmin, min( self.Zmax, p ) ) )
            
        elif self.state == 3 :
            t = self.throt.y() + dp.y()
            self.throt.moveTop( max( self.Vmin, min( self.Vmax, t ) ) )

        elif self.state == 4 :
            b = self.brake.y() + dp.y()
            self.brake.moveTop( max( self.Vmin, min( self.Vmax, b ) ) )

        self.signal.emit()
        self.update()


    def resizeEvent( self, event ) :
        print( 'resizeEvent ControlPanel' )
        self.resizeFlag = True
        
        W, H = self.width(), self.height()
    
        w, h = 0.6, 0.9
        X0 = 0.0
        X1 = W * ( 1.0 - w ) / 2.0
        X2 = W - X1
        X3 = W
        Y0 = 0.0
        Y1 = h * H
        Y2 = H
        
        Ds = min( W, H ) * 0.10
        Dp = ( X2 - X1 ) * 0.06
        Dt = ( Y2 - Y0 ) * 0.06
               
        self.Xmin = X1 + Ds / 2.0
        self.Xmax = X2 - Ds / 2.0
        self.Ymin = Y0 + Ds / 2.0
        self.Ymax = Y1 - Ds / 2.0

        self.Zmin = X1
        self.Zmax = X2 - Dp

        self.Vmin = 0.0
        self.Vmax = Y2 - Dt
        self.brakeMin = self.Vmax
                    
        self.throtRect = QRectF( X0, Y0, X1-X0, Y2-Y0 )
        self.stickRect = QRectF( X1, Y0, X2-X1, Y1-Y0 )
        self.pedalRect = QRectF( X1, Y1, X2-X1, Y2-Y1 )
        self.brakeRect = QRectF( X2, Y0, X3-X2, Y2-Y0 )
           
        self.throt = QRectF( QPointF( X0, Y2 - Dt ), QSizeF( X1 - X0, Dt ) )
        self.stick = QRectF( QPointF( ( X1 + X2 - Ds ) / 2.0, ( Y0 + Y1 - Ds ) / 2.0 ), QSizeF( Ds, Ds ) )
        self.pedal = QRectF( QPointF( ( X1 + X2 - Dp ) / 2.0, Y1 ), QSizeF( Dp, Y2 - Y1 ) )
        self.brake = QRectF( QPointF( X2, Y2 - Dt )               , QSizeF( X3 - X2, Dt ) )
           
        self.trimX = QLineF( ( X1 + X2 ) / 2.0, Y0, ( X1 + X2 ) / 2.0, Y1 )
        self.trimY = QLineF( X1, ( Y0 + Y1 ) / 2.0, X2, ( Y0 + Y1 ) / 2.0 )
        self.trimZ = QLineF( ( X1 + X2 ) / 2.0, Y1, ( X1 + X2 ) / 2.0, Y2 )

        self.dX =  self.stickRect.width()  - self.stick.width() 
        self.dY =  self.stickRect.height() - self.stick.height() 
        self.dZ =  self.pedalRect.width()  - self.pedal.width() 

        #self.colorBackground = Qt.GlobalColor.green
        #self.colorBackground = QColor( 34, 139, 34 )
        self.colorBackground = QColor( '#008000' )


    def paintEvent( self, event ) :
        #print( 'paintEvent ControlPanel' )
        
        WW, HH = self.width(), self.height()
                
                
        painter = QPainter(self)
        
        painter.setBrush( self.colorBackground )            
        painter.drawRect( self.stickRect )
        painter.drawRect( self.pedalRect )
        painter.drawRect( self.throtRect )
        painter.drawRect( self.brakeRect )

        if self.aircraft is not None :
            th = self.aircraft.thst / self.aircraft.thst_ref
        else :
            th = 0.0
            
        painter.setBrush( QColor( '#006000' ) )
        X = self.throtRect.left()
        W = self.throtRect.width()
        H = th * self.throtRect.height()
        Y = self.throtRect.bottom() - H
        rect = QRectF( X, Y, W, H )
        painter.drawRect( rect )
        
        
        X1, X2 = self.stickRect.left(), self.stickRect.right()
        Y1, Y2 = self.stickRect.top(), self.stickRect.bottom()
        Y3, Y4 = self.pedalRect.top(), self.pedalRect.bottom()
        XX = ( X1 + X2 ) / 2.0
        YY = ( Y1 + Y2 ) / 2.0
        painter.drawLine( QPointF( X1, YY ), QPointF( X2, YY ) )
        painter.drawLine( QPointF( XX, Y1 ), QPointF( XX, Y2 ) ) 
                 
        painter.setPen( QPen( Qt.GlobalColor.yellow, 3 ) )
        painter.drawLine( self.trimX )
        painter.drawLine( self.trimY )
                 
        painter.setPen(Qt.PenStyle.NoPen)
        if self.state == 1 : painter.setBrush( Qt.GlobalColor.red )
        else               : painter.setBrush( Qt.GlobalColor.blue )
        r = self.stick.width() / 2.0
        painter.drawRoundedRect( self.stick, r, r )
        
        if self.state == 2 : painter.setBrush( Qt.GlobalColor.red )
        else               : painter.setBrush( Qt.GlobalColor.blue )
        painter.drawRect( self.pedal )
        
        if self.state == 3 : painter.setBrush( Qt.GlobalColor.red )
        else               : painter.setBrush( Qt.GlobalColor.blue )
        painter.drawRect( self.throt )

        if self.state == 4 : painter.setBrush( Qt.GlobalColor.red )
        else               : painter.setBrush( Qt.GlobalColor.blue )
        painter.drawRect( self.brake )

        painter.setPen( QPen( Qt.GlobalColor.yellow, 3 ) )
        painter.drawLine( self.trimZ )
        

class Controler( QWidget ) :

#     signal = pyqtSignal()
    panelOperation = pyqtSignal()
    deviceButtonPushed = pyqtSignal()

    def __init__( self ) :
        print( '__init__ Controler' )

        super().__init__()
                
        self.joystick = Joystick( self )
        self.joystick.buttonSignal.connect( self.joystickbuttonSignalRecv )
        self.joystick.axisSignal.connect( self.joystickAxisSignalRecv )
    
        self.controlPanel = ControlPanel()
        self.controlPanel.signal.connect( self.controlPanelSignalRecv )
        
        
        self.switchValue = [ 0, 0, 0 ]
        
        layout = QVBoxLayout( self )
        
        layout.addWidget( self.controlPanel )

        switches = QHBoxLayout()
        grpBoxes = [   QGroupBox( txt )    for txt in [ 'FLAP' , 'S.B.' , 'GEAR' ]   ]
        buttons  = [ [ QRadioButton( txt ) for txt in [ 'UP'   , 'MID'  , 'DN'   ] ],
                     [ QRadioButton( txt ) for txt in [ 'CLOSE', 'OPEN'          ] ],
                     [ QRadioButton( txt ) for txt in [ 'UP'   , 'DN'            ] ] ]
        self.btnGrps = [ QButtonGroup() for _ in grpBoxes ]
        
        for gbox, grp, btns in zip( grpBoxes, self.btnGrps, buttons ) :
            switches.addWidget( gbox )
            lo = QVBoxLayout()
            gbox.setLayout( lo )
            for k, btn in enumerate( btns ) :
                lo.addWidget( btn )
                grp.addButton( btn, k )
                btn.clicked.connect( self.switchClicked )
        for btns, sv in zip( buttons, self.switchValue ) :
            btns[ sv ].setChecked( True )

        layout.addLayout( switches )


        progBars = QGridLayout()
        
        progBars.addWidget( QLabel( 'FLAP' ), 0, 0 )
        progBars.addWidget( QLabel( 'S.B.' ), 1, 0 )
        progBars.addWidget( QLabel( 'GEAR' ), 2, 0 )
        
        self.qProgs = [ QProgressBar() for _ in range( 3 ) ]
        for k, p in enumerate( self.qProgs ) :
            progBars.addWidget( p, k, 1 )
            p.setSizePolicy( QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding )
            p.setValue( 50 )

        layout.addLayout( progBars )

        layout.setStretch( 0, 3 )
        layout.setStretch( 1, 1 )
        layout.setStretch( 2, 1 )

#         self.qtimer = QTimer()
#         self.qtimer.setInterval( 10 )
#         self.qtimer.timeout.connect( self.update )
#         self.qtimer.start()                

        print( '__init__ Controler ZZZ' )


    def reset( self ) :
        print( 'reset Controler' )
        self.joystick.reset()


    def update( self ) :
#         #print( 'update Controler' )
#     
#         if self.controlPanel.resizeFlag :
#             if not self.controlPanel.mousePressed :
#                 X, Y, Z, B = self.joystick.XYZB
#                 self.stickPos  = ( X, Y, Z )
#                 self.brkPos = B            
#             
        self.controlPanel.update()
            
  
    def joystickbuttonSignalRecv( self, CMD ) :
        #print( 'joystickbuttonSignalRecv Controler' )
        #print( CMD )

        if CMD.startswith( 'THR' )  :
            #thr, brk = self.thrbrkPos
            thr = self.thrPos
            if    CMD == 'THR_UP' : thr = min( 1.0, thr + 0.05 ) 
            elif  CMD == 'THR_DN' : thr = max( 0.0, thr - 0.05 )
            #self.thrbrkPos = ( thr, brk )
            self.thrPos = thr
            self.panelOperation.emit()
                
        elif CMD.startswith( 'trm' ) :
            X, Y, Z = self.trimPos
            if    CMD == 'trmY_UP' : Y = min(   1.0, Y + 0.02 ) 
            elif  CMD == 'trmY_DN' : Y = max( - 1.0, Y - 0.02 )
            elif  CMD == 'trmX_LH' : X = max( - 1.0, X + 0.01 )
            elif  CMD == 'trmX_RH' : X = max( - 1.0, X - 0.01 )
            self.trimPos = ( X, Y, Z )
            self.panelOperation.emit()
            
        elif CMD == 'FLAP' :
            chkId = self.btnGrps[0].checkedId()
            chkId = ( chkId + 1 ) % 3
            self.btnGrps[0].button( chkId ).setChecked( True )
            self.switchClicked()
      
        elif CMD == 'FLAP_UP' :
            #print( 'AAA FLAP_UP' )
            self.btnGrps[0].button( 0 ).setChecked( True )
            self.switchClicked()
      
        elif CMD == 'FLAP_DN' :
            #print( 'AAA FLAP_DN' )
            self.btnGrps[0].button( 2 ).setChecked( True )
            self.switchClicked()
      
        elif CMD == 'SB' :
            chkId = self.btnGrps[1].checkedId()
            chkId = ( chkId + 1 ) % 2
            self.btnGrps[1].button( chkId ).setChecked( True )
            self.switchClicked()
      
        elif CMD == 'SB_CLOSE' :
            self.btnGrps[1].button( 0 ).setChecked( True )
            self.switchClicked()
      
        elif CMD == 'SB_OPEN' :
            self.btnGrps[1].button( 1 ).setChecked( True )
            self.switchClicked()
      
        elif CMD == 'GEAR' :
            chkId = self.btnGrps[2].checkedId()
            chkId = ( chkId + 1 ) % 2
            self.btnGrps[2].button( chkId ).setChecked( True )
            self.switchClicked()
            
        elif CMD == 'GEAR_UP' :
            self.btnGrps[2].button( 0 ).setChecked( True )
            self.switchClicked()
      
        elif CMD == 'GEAR_DN' :
            self.btnGrps[2].button( 1 ).setChecked( True )
            self.switchClicked()
      
        self.update()

    def joystickAxisSignalRecv( self, XYZTB ) :
        #print( 'joystickAxisSignalRecv Controler' )
        X, Y, Z, T, B = XYZTB
        self.stickPos = ( X, Y, Z ) 
        self.thrPos   =  T
        self.brkPos   =  B
        #print( 'joystickAxisSignalRecv Controler', B )
        self.panelOperation.emit()
        self.update()
        
    def controlPanelSignalRecv( self ) :
        #print( 'controlPanelSignalRecv Controler' )
        self.panelOperation.emit()




#     @property
#     def value( self ) :
#         X, Y, Z, TH, BR = self.controlPanel.controlValue() 
#         FL, SB, GR = self.switchValue
#         FL *= 0.5
#         return ( X, Y, Z, TH, BR, FL, SB, GR )


    @property
    def stickPos( self ) :
        return self.controlPanel.XYZpos
        
    @stickPos.setter
    def stickPos( self, value ) :
        #print('stickPos.setter')
        self.controlPanel.XYZpos = value
        
    @property
    def trimPos( self ) :
        return self.controlPanel.XYZtrm
        
    @trimPos.setter
    def trimPos( self, value ) :
        self.controlPanel.XYZtrm = value

    @property
    def thrPos( self ) :
        return self.controlPanel.thrPos

    @thrPos.setter
    def thrPos( self, value ) :
        self.controlPanel.thrPos = value

    @property
    def brkPos( self ) :
        return self.controlPanel.brkPos

    @brkPos.setter
    def brkPos( self, value ) :
        self.controlPanel.brkPos = value


    @property
    def DEVcmd( self ) :
        FL, SB, GR = self.switchValue
        FL *= 0.5
        return ( FL, SB, GR )
    
    @DEVcmd.setter
    def DEVcmd( self, value ) :
        
        if   value[0] < 0.25 : self.btnGrps[0].button(0).setChecked( True )
        elif value[0] < 0.75 : self.btnGrps[0].button(1).setChecked( True )
        else                 : self.btnGrps[0].button(2).setChecked( True )
        
        if   value[1] < 0.50 : self.btnGrps[1].button(0).setChecked( True )
        else                 : self.btnGrps[1].button(1).setChecked( True )
        
        if   value[2] < 0.50 : self.btnGrps[2].button(0).setChecked( True )
        else                 : self.btnGrps[2].button(1).setChecked( True )
        
        self.switchClicked() 
    
    @property
    def DEVpos( self ) :
        vals = [ prg.value() / 100.0 for prg in self.qProgs ]
        return vals

    @DEVpos.setter
    def DEVpos( self, value ) :
        FL, SB, GR = value
        self.qProgs[0].setValue( round( FL * 100.0 ) )
        self.qProgs[1].setValue( round( SB * 100.0 ) )
        self.qProgs[2].setValue( round( GR * 100.0 ) )
   

    def setAircraft( self, aircraft ) :
        print( 'setAircraft Controler' )
        self.aircraft = aircraft
        self.controlPanel.aircraft = aircraft
        

    def switchClicked( self ) :
        print( 'switchClicked' )
        self.switchValue = [ grp.checkedId() for grp in self.btnGrps ]
        self.deviceButtonPushed.emit()
        
# # 
# # pygame.init()
# # joystick = pygame.joystick.Joystick(0) # create a joystick instance
# # joystick.init() # init instance
# # 
# # print('ジョイスティックの名前:', joystick.get_name())
# # print('ボタン数 :', joystick.get_numbuttons())
# # 
# # while True :
# #     pygame.time.wait( 10 )
# #     for event in pygame.event.get() :
# #         if event.type == pygame.JOYBUTTONDOWN :
# #             pass
# #             print( event )
# #             #print("\nJoystick button pressed.")
# #         elif event.type == pygame.JOYHATMOTION :
# #             #print("\nJoystick JOYHATMOTION.")
# #             print( event )
# #         print( event )
# #             #<Event(1538-JoyHatMotion {'joy': 0, 'instance_id': 0, 'hat': 0, 'value': (-1, 0)})>
# # #         elif event.type == KEYDOWN and event.key == K_ESCAPE:
# # #           print("\nEsc pressed.")
# # #         elif event.type == KEYDOWN and event.key == K_ESCAPE:
# # #           print("\nEsc pressed.")
# #                
 
            
class MainWindow( QMainWindow ) :

    def __init__(self, parent = None ) :
        
        super().__init__()
        
        self.setGeometry( 300, 100, 500, 500)
        self.setWindowTitle( 'Main Window' )
        
        self.controler = Controler()
        self.setCentralWidget( self.controler )
        
        self.joystick = Joystick()
#         
#         try:
#            # ジョイスティックインスタンスの生成
#            joystick = pygame.joystick.Joystick(0)
#            joystick.init()
#            print('ジョイスティックの名前:', joystick.get_name())
#            print('ボタン数 :', joystick.get_numbuttons())
#         except pygame.error:
#            print('ジョイスティックが接続されていません')
#                 
#             #PS4 Controller
#         
#         pygame.init()
#         
#         while True :
#             pygame.time.wait( 10 )
#             for event in pygame.event.get() :
#                 print( event.type )
#                        
#                            
        
#         self.controler = Controler()
#         self.setCentralWidget( self.controler )


if __name__ == '__main__' :    
    app = QApplication( sys.argv )
    mainwindow = MainWindow()
    mainwindow.show()
    app.exec()

# # # 
# # # import pygame
# # # 
# # # pygame.init()
# # # 
# # # 
# # # # This is a simple class that will help us print to the screen.
# # # # It has nothing to do with the joysticks, just outputting the
# # # # information.
# # # class TextPrint:
# # #     def __init__(self):
# # #         self.reset()
# # #         self.font = pygame.font.Font(None, 25)
# # # 
# # #     def tprint(self, screen, text):
# # #         text_bitmap = self.font.render(text, True, (0, 0, 0))
# # #         screen.blit(text_bitmap, (self.x, self.y))
# # #         self.y += self.line_height
# # # 
# # #     def reset(self):
# # #         self.x = 10
# # #         self.y = 10
# # #         self.line_height = 15
# # # 
# # #     def indent(self):
# # #         self.x += 10
# # # 
# # #     def unindent(self):
# # #         self.x -= 10
# # # 
# # # 
# # # def main():
# # #     # Set the width and height of the screen (width, height), and name the window.
# # #     screen = pygame.display.set_mode((500, 700))
# # #     pygame.display.set_caption("Joystick example")
# # # 
# # #     # Used to manage how fast the screen updates.
# # #     clock = pygame.time.Clock()
# # # 
# # #     # Get ready to print.
# # #     text_print = TextPrint()
# # # 
# # #     # This dict can be left as-is, since pygame will generate a
# # #     # pygame.JOYDEVICEADDED event for every joystick connected
# # #     # at the start of the program.
# # #     joysticks = {}
# # # 
# # #     done = False
# # #     while not done:
# # #         # Event processing step.
# # #         # Possible joystick events: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
# # #         # JOYBUTTONUP, JOYHATMOTION, JOYDEVICEADDED, JOYDEVICEREMOVED
# # #         for event in pygame.event.get():
# # #             if event.type == pygame.QUIT:
# # #                 done = True  # Flag that we are done so we exit this loop.
# # # 
# # #             if event.type == pygame.JOYBUTTONDOWN:
# # #                 print("Joystick button pressed.")
# # #                 if event.button == 0:
# # #                     joystick = joysticks[event.instance_id]
# # #                     if joystick.rumble(0, 0.7, 500):
# # #                         print(f"Rumble effect played on joystick {event.instance_id}")
# # # 
# # #             if event.type == pygame.JOYBUTTONUP:
# # #                 print("Joystick button released.")
# # # 
# # #             # Handle hotplugging
# # #             if event.type == pygame.JOYDEVICEADDED:
# # #                 # This event will be generated when the program starts for every
# # #                 # joystick, filling up the list without needing to create them manually.
# # #                 joy = pygame.joystick.Joystick(event.device_index)
# # #                 joysticks[joy.get_instance_id()] = joy
# # #                 print(f"Joystick {joy.get_instance_id()} connencted")
# # # 
# # #             if event.type == pygame.JOYDEVICEREMOVED:
# # #                 del joysticks[event.instance_id]
# # #                 print(f"Joystick {event.instance_id} disconnected")
# # # 
# # #         # Drawing step
# # #         # First, clear the screen to white. Don't put other drawing commands
# # #         # above this, or they will be erased with this command.
# # #         screen.fill((255, 255, 255))
# # #         text_print.reset()
# # # 
# # #         # Get count of joysticks.
# # #         joystick_count = pygame.joystick.get_count()
# # # 
# # #         text_print.tprint(screen, f"Number of joysticks: {joystick_count}")
# # #         text_print.indent()
# # # 
# # #         # For each joystick:
# # #         for joystick in joysticks.values():
# # #             jid = joystick.get_instance_id()
# # # 
# # #             text_print.tprint(screen, f"Joystick {jid}")
# # #             text_print.indent()
# # # 
# # #             # Get the name from the OS for the controller/joystick.
# # #             name = joystick.get_name()
# # #             text_print.tprint(screen, f"Joystick name: {name}")
# # # 
# # #             guid = joystick.get_guid()
# # #             text_print.tprint(screen, f"GUID: {guid}")
# # # 
# # #             power_level = joystick.get_power_level()
# # #             text_print.tprint(screen, f"Joystick's power level: {power_level}")
# # # 
# # #             # Usually axis run in pairs, up/down for one, and left/right for
# # #             # the other. Triggers count as axes.
# # #             axes = joystick.get_numaxes()
# # #             text_print.tprint(screen, f"Number of axes: {axes}")
# # #             text_print.indent()
# # # 
# # #             for i in range(axes):
# # #                 axis = joystick.get_axis(i)
# # #                 text_print.tprint(screen, f"Axis {i} value: {axis:>6.3f}")
# # #             text_print.unindent()
# # # 
# # #             buttons = joystick.get_numbuttons()
# # #             text_print.tprint(screen, f"Number of buttons: {buttons}")
# # #             text_print.indent()
# # # 
# # #             for i in range(buttons):
# # #                 button = joystick.get_button(i)
# # #                 text_print.tprint(screen, f"Button {i:>2} value: {button}")
# # #             text_print.unindent()
# # # 
# # #             hats = joystick.get_numhats()
# # #             text_print.tprint(screen, f"Number of hats: {hats}")
# # #             text_print.indent()
# # # 
# # #             # Hat position. All or nothing for direction, not a float like
# # #             # get_axis(). Position is a tuple of int values (x, y).
# # #             for i in range(hats):
# # #                 hat = joystick.get_hat(i)
# # #                 text_print.tprint(screen, f"Hat {i} value: {str(hat)}")
# # #             text_print.unindent()
# # # 
# # #             text_print.unindent()
# # # 
# # #         # Go ahead and update the screen with what we've drawn.
# # #         pygame.display.flip()
# # # 
# # #         # Limit to 30 frames per second.
# # #         clock.tick(30)
# # # 
# # # 
# # # if __name__ == "__main__":
# # #     main()
# # #     # If you forget this line, the program will 'hang'
# # #     # on exit if running from IDLE.
# # #     pygame.quit()
# # # 
