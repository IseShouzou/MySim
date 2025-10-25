
import numpy as np

from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *

#from PyQt6.QtOpenGL import *
#from PyQt6.QtOpenGLWidgets import QOpenGLWidget

 
class HUD( QWidget ) :
 
    keyPressed = pyqtSignal( QKeyEvent )
 
    def __init__( self, parent ) :
        print( '__init__ HUD' )
        super().__init__( parent )
        
        self.parent = parent
        
        self.setFocusPolicy( Qt.FocusPolicy.StrongFocus )
        
        self.aircraft = None
        
        self.lineColor = QColor( 0, 100, 0 )
            
        self.V_kt   = 300.0
        self.alt_ft = 12000.0
        self.psi = 0.0
        self.uvw = np.array( [ 1.0, 0.1, 0.02 ] )
        
        self.the = np.radians( 20.0 )
        self.phi = np.radians( 40.0 )
        
        self.magVariation_deg = 0.0
        
        
    def keyPressEvent( self, event ) :
        #print( 'keyPressEvent HUD' )
        self.keyPressed.emit( event )
        return super().keyPressEvent( event )
    
    def setAircraft( self, aircraft ) :
        print( 'setAircraft HUD' )
        self.aircraft = aircraft
        
    def setMagVariation_deg( self, magVariation_deg ) :
        print( 'setMagVariation_deg HUD' )
        self.magVariation_deg = magVariation_deg
        
    def resizeEvent( self, event ) :
        print( 'resizeEvent HUD' )
        
        W, H = self.width(), self.height()
        
        self.lineThickness1 = max( 0.5, ( W + H ) * 0.0015 )
        
        fr = 24.0 / QFontMetrics( QFont( 'Arial', 24 ) ).height()
        px = round( 0.038 * H * fr )
        #self.font1 = QFont( 'Arial', px, QFont.Weight.Bold, False )
        self.font1 = QFont( 'Arial', px, QFont.Weight.Normal, False )
        
        px = round( 0.03 * H * fr )
        self.font2 = QFont( 'Arial', px, QFont.Weight.Normal, False )
        
        
    def paintEvent( self, event ) :
        #print( 'paintEvent HUD' )
        
        V   = self.V_kt
        alt = self.alt_ft
        psi_deg = np.degrees( self.psi )
        uvw = self.uvw
        
        the = self.the
        phi = self.phi
        #print( np.degrees( the ), np.degrees( phi ) )
        
        mach = 0.0
        
        if self.aircraft is not None :
            V   = self.aircraft.Vc_kt
            alt = self.aircraft.alt_sl_ft
            psi_deg = np.degrees( self.aircraft.eul[2] ) - self.magVariation_deg
            uvw = self.aircraft.uvw
            
            the = self.aircraft.eul[1] 
            phi = self.aircraft.eul[0]
            
            alp_deg = np.degrees( self.aircraft.alp )
            mach    = self.aircraft.mach
            #Nz      = - self.aircraft.pilotNxyz[2]
            Nz      = self.aircraft.Nxyz[2]
            #print( self.aircraft.Nxyz )
            #pilotNxyz
            
        W, H = self.width(), self.height()
        
        fovy = self.parent.projFovy
        e = 0.5 * H / np.tan( np.radians( 0.5 * fovy ) )
        
        painter = QPainter( self )
        painter.setRenderHint( QPainter.RenderHint.Antialiasing )

        painter.setFont( self.font1 )        
        painter.setPen( QPen( self.lineColor, self.lineThickness1 ) )
               
        painter.save()
        painter.translate( W / 2.0, H / 2.0 )

        #
        # Whisky
        #
        painter.drawPolyline( [ QPointF( - 0.04 * H, 0.0      ),
                                QPointF( - 0.02 * H, 0.0      ),
                                QPointF( - 0.01 * H, 0.02 * H ),
                                QPointF(   0.0     , 0.0      ),
                                QPointF(   0.01 * H, 0.02 * H ),
                                QPointF(   0.02 * H, 0.0      ),
                                QPointF(   0.04 * H, 0.0      ),
                                ] )
        
        
        
        x1 = - 0.35 * H
        x2 = - 0.27 * H
        w =    0.08 * H
        h =    0.10 * H
        ys   = [ 0.16 * H, 0.20 * H, 0.24 * H ]
        txts = [ 'AOA'   , 'M'     , 'G'      ]
        vals = [ format( alp_deg, '.1f'),
                 format( mach   , '.1f'),
                 format( Nz     , '.1f'),  ]
        painter.setFont( self.font1 )
        for k in range( 3 ) :
            rect = QRectF( x1, ys[k], w, h )
            painter.drawText( rect, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop, txts[k] )
            rect = QRectF( x2, ys[k], w, h )
            painter.drawText( rect, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTop, vals[k] )
                    

        #
        # Velosity Vector
        #
        if abs( uvw[0] ) < 1.0e-6 : uvw[0] = 1.0
        Vx = e * uvw[1] / uvw[0]
        Vy = e * uvw[2] / uvw[0]
        painter.save()
        painter.translate( Vx, Vy )
        r1, r2 = round( 0.02 * H ), round( 0.04 * H )
        painter.drawEllipse( QPoint( 0, 0) , r1, r1 )
        painter.drawLine( QPoint(   0 , - r2 ) , QPoint(   0 , - r1 ) )
        painter.drawLine( QPoint( - r2,   0  ) , QPoint( - r1,   0  ) )
        painter.drawLine( QPoint(   r2,   0  ) , QPoint(   r1,   0  ) )
        painter.restore()        
        
        #
        # Pitch Ladder
        #
        cPhi, sPhi = np.cos( phi ), np.sin( phi )
        t = cPhi * Vx - sPhi * Vy 
        Qx, Qy = t * cPhi, - t * sPhi
        
        x = round( - 0.25 * H )
        y = round( - 0.25 * H )
        w = round( 0.50 * H )
        h = round( 0.50 * H )
        painter.setClipRegion( QRegion( x, y, w, h ) )
        
        painter.save()
        
        painter.translate( Qx, Qy )
        painter.rotate( round( np.degrees( - phi ) ) )
        
        x01 = round( 0.05  * H )
        x02 = round( 0.20  * H )
        x03 = round( 0.15  * H )
        x04 = round( 0.155 * H )
        x10 = round( 0.05  * H )
        x11 = round( ( 0.05 + 0.10 / 7.0 * 1.0 ) * H )
        x12 = round( ( 0.05 + 0.10 / 7.0 * 2.0 ) * H  )
        x13 = round( ( 0.05 + 0.10 / 7.0 * 3.0 ) * H )
        x14 = round( ( 0.05 + 0.10 / 7.0 * 4.0 ) * H  )
        x15 = round( ( 0.05 + 0.10 / 7.0 * 5.0 ) * H  )
        x16 = round( ( 0.05 + 0.10 / 7.0 * 6.0 ) * H  )
        x17 = round( ( 0.05 + 0.10             ) * H  )
        y00 = round( 0.02 * H )
        y01 = round( 0.01 * H )

        the_deg = np.degrees( the )
        k1 = int( ( the_deg - fovy * 0.5 ) / 5.0 ) - 2
        k2 = int( ( the_deg + fovy * 0.5 ) / 5.0 )
        painter.setFont( self.font2 )
        for k in range( k1, k2 ) :
            v = k * 5.0
            yy = round( - H / fovy *( v - the_deg ) )
            txt = str( round( abs( v ) ) )
            if k == 0 :
                painter.drawLine(   x01, yy,   x02, yy )
                painter.drawLine( - x01, yy, - x02, yy )
            elif k > 0 :
                painter.drawLine(   x03, yy,   x03, yy + y00 )
                painter.drawLine( - x03, yy, - x03, yy + y00 )
                painter.drawLine(   x01, yy,   x03, yy )
                painter.drawLine( - x01, yy, - x03, yy )
                rect = QRectF(  x04, yy, 100.0,  y00 )
                painter.drawText( rect, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, txt )
                rect = QRectF( - x04 - 100.0, yy, 100.0, y00 )
                painter.drawText( rect, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, txt )
            else :
                painter.drawLine(   x03, yy,   x03, yy - y00 )
                painter.drawLine( - x03, yy, - x03, yy - y00 )
                painter.drawLine(   x10, yy,   x11, yy )
                painter.drawLine(   x12, yy,   x13, yy )
                painter.drawLine(   x14, yy,   x15, yy )
                painter.drawLine(   x16, yy,   x17, yy )
                painter.drawLine( - x10, yy, - x11, yy )
                painter.drawLine( - x12, yy, - x13, yy )
                painter.drawLine( - x14, yy, - x15, yy )
                painter.drawLine( - x16, yy, - x17, yy )
                rect = QRectF(  x04, yy - y00, 100.0,  y00 )
                painter.drawText( rect, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, txt )
                rect = QRectF( - x04 - 100.0, yy - y00, 100.0, y00 )
                painter.drawText( rect, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, txt )
                
        painter.restore()        
        painter.setClipping( False )        
        
        painter.restore()        
        
        
        #
        # Speed Meter
        #
        x = round( 0.50 * W - 0.35 * H )
        y = round( 0.25 * H )
        w = round( 0.15 * H )
        h = round( 0.40 * H )
        
        painter.setClipRegion( QRegion( x, y, w, h ) )
        painter.save()
        painter.translate( x, y )
        
        yc = h / 2.0
        x1 = w - H * 0.02
        x2 = w - H * 0.03
        x3 = w - H * 0.04
        x4 = w - H * 0.05
        dy = H * 0.010
        
        painter.drawLine( QPointF( w, yc - dy ), QPointF( x1, yc ) )
        painter.drawLine( QPointF( w, yc + dy ), QPointF( x1, yc ) )
        painter.drawLine( QPointF( x1, 0), QPointF( x1, h ) )
        
        painter.setFont( self.font1 )
        k1 = int( ( V - 50.0 ) / 10.0 ) 
        k2 = int( ( V + 50.0 ) / 10.0 ) + 2
        for k in range( k1, k2 ) :
            v = k * 10.0 
            yy = yc * ( 1.0 + ( v - V ) / 50.0 )
            xx = x2
            if k % 2 == 0 and k >= 0 :
                xx = x3
                rect = QRectF( 0, yy - 100.0, x4, 200.0 )
                txt = str( round( v ) )
                painter.drawText( rect, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, txt )
            painter.drawLine( QPointF( x1, yy ), QPointF( xx, yy ) )
        
        painter.restore()        
        painter.setClipping( False )        
        

        #
        # Altitude Meter
        #
        x = round( 0.50 * W + 0.20 * H )
        y = round( 0.25 * H )
        w = round( 0.15 * H )
        h = round( 0.40 * H )
        
        painter.setClipRegion( QRegion( x, y, w, h ) )
        painter.save()
        painter.translate( x, y )
        
        yc = h / 2.0
        x1 = H * 0.02
        x2 = H * 0.03
        x3 = H * 0.04
        x4 = H * 0.05
        dy = H * 0.010
        
        painter.drawLine( QPointF( 0, yc - dy ), QPointF( x1, yc ) )
        painter.drawLine( QPointF( 0, yc + dy ), QPointF( x1, yc ) )
        painter.drawLine( QPointF( x1, 0), QPointF( x1, h ) )
        
        FL = alt / 100.0
        FL1 = int( FL - 7.5 )
        FL2 = int( FL + 7.5 ) + 2
        for f in range( FL1, FL2 ) :
            yy = yc * ( 1.0 - ( f - FL ) / 7.5 )
            xx = x2
            if f % 5 == 0 and f >= 0 :
                xx = x3
                rect = QRectF( 0, yy - 100.0, w, 200.0 )
                txt = str( f ) + '00'
                painter.drawText( rect, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, txt )
            painter.drawLine( QPointF( x1, yy ), QPointF( xx, yy ) )
        
        painter.restore()        
        painter.setClipping( False )        
        
        #
        # Haeding
        #
        x = round( 0.50 * W - 0.15 * H  )
        y = round( 0.25 * H )
        w = round( 0.30 * H )
        h = round( 0.085 * H )
        
        painter.setClipRegion( QRegion( x, y, w, h ) )
        painter.save()
        painter.translate( x, y )
                
        xc = w / 2.0
        y1 = h - H * 0.02
        y2 = h - H * 0.035
        y3 = h - H * 0.04
        y4 = h - H * 0.05
        dx = H * 0.010
        
        painter.drawLine( QPointF( xc - dx , h ), QPointF( xc, y1 ) )
        painter.drawLine( QPointF( xc + dx , h ), QPointF( xc, y1 ) )
        painter.drawLine( QPointF( 0, y1 ), QPointF( w, y1 ) )
        
        P = psi_deg
        P1 = int( ( P - 15.0 ) / 2.0 ) - 1
        P2 = int( ( P + 15.0 ) / 2.0 ) + 2
        for k in range( P1, P2 ) :
            p = k * 2.0
            xx = xc * ( 1.0 + ( p - P ) / 15.0 )
            yy = y2
            if p % 10 == 0 :
                yy = y3
                rect = QRectF( xx - 100, 0.0, 200.0, h )
                txt = str( round( p / 10 ) % 36 ).zfill(2)
                painter.drawText( rect, Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop, txt )
            painter.drawLine( QPointF( xx, y1 ), QPointF( xx, yy ) )
         
        painter.restore()        
        painter.setClipping( False )        
        
        