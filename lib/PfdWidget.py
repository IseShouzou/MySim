import sys
import numpy as np

from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *

#from MyFunc import *
#from MyPyqt import *
#from MyOpenGL import *
#from Aircraft import *

class PFDOBJECT :
    
#     class VerticalSpeedMeter( QWidget ) :
#         
#         def __init__( self, parent = None ) :
#             
#             super().__init__( parent )
#             self.value = 3.5
#             
#         def paintEvent( self, event ) :
#             #print('paintEvent VerticalSpeedMeter')
#             
#             WW, HH = self.width(), self.height()
#             W0, H0 = 100.0, 100.0
#             a = min( WW / W0, HH / H0 )
#             W, H = int( W0 * a ), int( H0 * a )
#             
#             R = min( W, H ) / 2.0 
#             fr = 24.0 / QFontMetrics( QFont( 'Arial', 24 ) ).height()
#             
#             painter = QPainter( self )
#             painter.setBrush( Qt.GlobalColor.black )
#             painter.drawRect( 0, 0, WW, HH )
#             
#             painter.save()
#             painter.translate( int( WW / 2.0 ), int( HH / 2.0 ) )
#             
#             painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )
#             painter.drawEllipse( QPointF( 0.0, 0.0 ), R, R )
#             
#             dp1, sz1, px1 = QPointF( 0.2*R, 0.2*R ), QSizeF( 0.4*R, 0.4*R ), int( fr*0.40*R*0.75 )
#             dp2, sz2, px2 = QPointF( 0.1*R, 0.1*R ), QSizeF( 0.2*R, 0.2*R ), int( fr*0.25*R )
# 
#             v = np.concatenate( [ np.arange( 0.0, 1.0, 0.1), np.arange( 1.0, 6.1, 0.5)] )
#             ang = np.radians( 220.0 * np.log10( 1.0 + 0.8 * v ) )
#             X, Y = - R * np.cos( ang ), - R * np.sin( ang )
#             
#             for i, ( x, y ) in enumerate( zip( X, Y ) ):
#                 p1, p2 = QPointF( x, y ), QPointF( x, -y )
#                 if   i == 0 :
#                     r = 0.80
#                     painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )
#                     painter.setFont( QFont( 'Arial', px1, QFont.Weight.Normal, False ) )
#                     painter.drawText( QRectF( 0.65*p1 - dp1, sz1 ), Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter, str( 0 ) )
#                 elif i == 5 :
#                     r = 0.85
#                     painter.setPen( QPen( Qt.GlobalColor.white, 1 ) )
#                     painter.setFont( QFont( 'Arial', px2, QFont.Weight.Normal, False ) )
#                     painter.drawText( QRectF( 0.7*p1-dp2, sz2 ), Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter, '.5' )
#                     painter.drawText( QRectF( 0.7*p2-dp2, sz2 ), Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter, '.5' )
#                 elif i <= 9 :
#                     r = 0.90
#                     painter.setPen( QPen( Qt.GlobalColor.white, 1 ) )
#                 elif i%2 == 0 :
#                     r = 0.80
#                     painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )
#                     if i in  [ 10, 12, 16 ] :
#                         t = str( int( ( i - 10 ) / 2.0 ) + 1 )
#                         painter.setFont( QFont( 'Arial', px1, QFont.Weight.Normal, False ) )
#                         painter.drawText( QRectF( 0.65*p1-dp1, sz1 ), Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter, str( t ) )
#                         painter.drawText( QRectF( 0.65*p2-dp1, sz1 ), Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter, str( t ) )
#                 else :
#                     r = 0.85
#                     painter.setPen( QPen( Qt.GlobalColor.white, 1 ) )
#                 painter.drawLine( p1, r * p1 )
#                 if i !=0 : painter.drawLine( p2, r * p2 )
#                 
#             painter.drawText( QRectF( 0.65*QPointF( R, 0 )-dp1, sz1 ), Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter, '6' )
#             
#             r, t = 0.85 * R,  ( 180 - int( np.degrees( ang[-1] ) )  - 2 ) * 16
#             painter.drawArc( QRectF( -r, -r, 2.0*r, 2.0*r ), t, -2*t )
#             
#             painter.setPen( QPen( Qt.GlobalColor.white, 5 ) )
#             v = abs( max( -6.0, min( 6.0, self.value ) ) )    
#             ang = np.sign( self.value ) * np.radians( 220.0 * np.log10( 1.0 + 0.8 * v ) )
#             p = QPointF( - R * np.cos( ang ), - R * np.sin( ang ) )
#             painter.drawLine( 0.8 * p, - 0.1 * p )
#             
#             painter.restore()



#     class VerticalSpeedMeter( QWidget ) :
#         
#         def __init__( self, parent = None ) :
#             
#             super().__init__( parent )
#             self.value = 3.5
#             
#             self.yd = [ -0.4, -0.3061, -0.1857, 0.0, 0.1857, 0.3061, 0.4  ]
#             self.wd = [ -6.0,   -2.0 ,    -1.0, 0.0,    1.0,    2.0, 6.0  ]
#             
#                         
#         def paintEvent( self, event ) :
#             #print('paintEvent VerticalSpeedMeter')
#             
# #             WW, HH = self.width(), self.height()
# #             W0, H0 = 100.0, 100.0
# #             a = min( WW / W0, HH / H0 )
# #             W, H = int( W0 * a ), int( H0 * a )
#             
#             W, H = self.width(), self.height()
#                         
#             painter = QPainter( self )
#             painter.setBrush( Qt.GlobalColor.black )
#             painter.drawRect( 0, 0, W, H )
#  
#             painter.translate( 0, int( H / 2.0 ) )
# 
#             painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )
#             px = int( H * 0.08 * 24.0 / QFontMetrics( QFont( 'Arial', 24 ) ).height() )
#             painter.setFont( QFont( 'Arial', px, QFont.Weight.Normal, False ) )
# 
#             for k in range( 7 ) :
#                 y = - self.yd[ k ] * H
#                 rect = QRectF( QPointF( 0.0, y - 20 ), QPointF( 0.28 * W, y + 20 ) )
#                 txt = str( round( self.wd[ k ] ) )
#                 painter.drawLine( QPointF( 0.3 * W, y ), QPointF( 0.5 * W, y ) )
#                 painter.drawText( rect, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, txt )
# 
#             for k in range( 6 ) :
#                 y = - ( self.yd[ k ] + self.yd[ k+1 ] ) / 2.0 * H
#                 painter.drawLine( QPointF( 0.4 * W, y ), QPointF( 0.5 * W, y ) )
# 
#             painter.setPen( QPen( Qt.GlobalColor.white, 3 ) )
#             y = - 0.4 * H * np.tanh( self.value / 2.0 ) / 0.999987711650796
#             painter.drawLine( QPointF( 0.4 * W, y ), QPointF( 1.5 * W, 0.0 ) )




    class VerticalSpeedMeter( QWidget ) :
        
        def __init__( self, parent = None ) :
            
            super().__init__( parent )
            self.value = 3.5
                        
#             self.yd = [ -0.42, -0.32, -0.18, 0.0, 0.18, 0.32, 0.42 ]
#             self.wd = [  -6.0,  -2.0,  -1.0, 0.0,  1.0,  2.0, 6.0  ]
                      
                      
        def resizeEvent( self, event ) :
            #print('resizeEvent VerticalSpeedMeter')
            
            W, H = self.width(), self.height()
            
            self.base = QPolygonF( [ QPointF( 0.0    , 0.0     ),
                                     QPointF( 0.5 * W, 0.0     ),
                                     QPointF( 1.0 * W, 0.2 * H ),
                                     QPointF( 1.0 * W, 0.8 * H ),
                                     QPointF( 0.5 * W, 1.0 * H ),
                                     QPointF( 0.0    , 1.0 * H ),
                                     QPointF( 0.0    , 0.0     ) ] )
            self.X1 = 0.3 * W          
            self.X2 = 0.4 * W
            self.X3 = 0.5 * W
            self.X4 = 1.5 * W
            
            self.yd = np.array( [ -0.42, -0.35, -0.22, 0.0,  0.22,  0.35,  0.42 ] ) * H
            self.wd = np.array( [  -6.0,  -2.0,  -1.0, 0.0,   1.0,   2.0,  6.0  ] )

            self.px = int( H * 0.10 * 24.0 / QFontMetrics( QFont( 'Arial', 24 ) ).height() )
            
            
        def paintEvent( self, event ) :
            #print('paintEvent VerticalSpeedMeter')
                        
            W, H = self.width(), self.height()
                        
            painter = QPainter( self )
            painter.setBrush( Qt.GlobalColor.black )
            painter.drawPolygon( self.base )
 
            painter.translate( 0, int( H / 2.0 ) )

            painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )
            painter.setFont( QFont( 'Arial', self.px, QFont.Weight.Normal, False ) )
            for y, w in zip( self.yd, self.wd ) :
                painter.drawLine( QPointF( self.X1, - y ), QPointF( self.X3, - y ) )
                txt = str( round( abs( w ) ) )
                rect = QRectF( QPointF( 0.0, y - 20 ), QPointF( self.X1, y + 20 ) )
                painter.drawText( rect, Qt.AlignmentFlag.AlignCenter, txt )

            for y1, y2 in zip( self.yd, self.yd[1:] ) :
                ym = - ( y1 + y2 ) / 2.0
                painter.drawLine( QPointF( self.X2, ym ), QPointF( self.X3, ym ) )
                
            y = np.interp( self.value, self.wd, self.yd )
            painter.drawLine( QPointF( self.X1, - y ), QPointF( self.X4, 0.0 ) )



    class EnginePowerIndicator( QWidget ) :
        
        def __init__( self, parent = None ) :
            
            super().__init__( parent )
        
            self.value = 80.0
            
        def paintEvent( self, event ) :
            #print('paintEvent EnginePowerIndicator')
            
            W, H = self.width(), self.height()
            V = self.value
        
            R  = min( W, H ) / 2.0 * 0.95
            fr = 24.0 / QFontMetrics( QFont( 'Arial', 24 ) ).height()
            px = int( 0.25 * R * fr )
            
            painter = QPainter( self )
            painter.setBrush( Qt.GlobalColor.black )
            painter.setFont( QFont( 'Arial', px, QFont.Weight.Normal, False ) )
            painter.drawRect( -1, -1, W+1, H+1 )
            
            pen1 = QPen( Qt.GlobalColor.white, 1 )
            pen2 = QPen( Qt.GlobalColor.white, 2 )
            pen3 = QPen( Qt.GlobalColor.white, 5 )
            
            painter.save()
            painter.translate( W / 2.0, H / 2.0 )
            
            painter.setPen( pen2 )
            painter.drawEllipse( QPointF( 0.0, 0.0 ), R, R )
            
            ang = np.radians( np.arange( 51 ) * 270.0 / 50.0 )
            X, Y = R * np.sin( ang ), - R * np.cos( ang )
            dp, s = QPointF( 0.2*R, 0.2*R ), QSizeF( 0.4*R, 0.4*R )
            for i, ( x, y ) in enumerate( zip( X, Y ) ):
                p = QPointF( x, y )
                if i%5 != 0 :
                    painter.setPen( pen1 )
                    painter.drawLine( p, 0.95 * p )
                else :
                    painter.setPen( pen2 )
                    painter.drawLine( p, 0.90 * p )
                    painter.drawText( QRectF( 0.75*p - dp, s ), Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter, str( 2*i ) )
            
            painter.setPen( pen3 )
            ang = np.radians( self.value * 2.7 )
            p = QPointF( R * np.sin( ang ), - R * np.cos( ang )  )
            painter.drawLine( 0.8 * p, - 0.1 * p )
            
            painter.restore()
            

    class SpeedMeter( QWidget ) :
        
        def __init__( self, parent = None ) :
            
            super().__init__( parent )
            self.value = 100.0
            
        def paintEvent( self, event ) :
            #print('paintEvent SpeedMeter')
            
            W, H = self.width(), self.height()
            V = self.value
            
            R, p1, p2 = 110, 10, 20
            n = int( R / p1 )
            w60, w75, w80, w90 = int( 0.60*W ), int( 0.75*W ), int( 0.80*W ), int( 0.90*W )
            h20, h40, h50      = int( 0.20*H ), int( 0.40*H ), int( 0.50*H )

            px0 = h20 * 24.0 / QFontMetrics( QFont( 'Arial', 24 ) ).height()
            px5, px6 = int( px0*0.5 ), int( px0*0.6 )

            painter = QPainter( self )
            painter.setPen( QPen( Qt.GlobalColor.white, 1 ) )
            painter.setBrush( Qt.GlobalColor.black )
            painter.setFont( QFont( 'Arial', px5, QFont.Weight.Normal, False ) )
            
            painter.drawRect( -1, -1, W+1, H+1 )
            v = round( ( V - R / 2.0 ) / p2 ) * p2
            for i in range( n + 1 ) :
                y = int( H * ( 0.5 + ( V - v ) / R ) )
                if i % 2 == 0 :
                    painter.drawLine( QPointF( w80, y ), QPointF( W, y ) )
                    painter.drawText( QRectF( 0, y-25, w75, 50 ), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, str( v ) )
                else :
                    painter.drawLine( QPointF( w90, y ), QPointF( W, y ) )
                v += p1
            painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )
            painter.drawRect( 0, h40, w75, h20 )
            painter.setFont( QFont( 'Arial', px6, QFont.Weight.Bold, False ) )
            painter.drawText( 0, h40, w60, h20, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, str( round(V) ) )
            painter.setPen( QPen( Qt.GlobalColor.white , 3 ) )
            painter.drawLine( w75, h50, W, h50 )


    class AltitudeMeter( QWidget ) :
        
        def __init__( self, parent = None ) :
            
            super().__init__( parent )
            self.value = 100.0
            
        def resizeEvent( self, event ) :
            #print('resizeEvent AltitudeMeter')
            
            W, H = self.width(), self.height()
            
            self.X1 = int( 0.10 * W )
            self.X2 = int( 0.20 * W )
            
            self.X3 = int( 0.35 * W )
            self.X4 = int( 0.65 * W )
            self.W1 = int( 0.30 * W )
            self.W2 = int( 0.30 * W )
            
            fr = 24.0 / QFontMetrics( QFont( 'Arial', 24, QFont.Weight.Normal ) ).horizontalAdvance( '00' )
            self.px1 = int( fr * self.W1 )
            fr = 24.0 / QFontMetrics( QFont( 'Arial', 24, QFont.Weight.Normal ) ).horizontalAdvance( '000' )
            self.px2 = int( fr * self.W2 )
            
            
            self.X5 = int( 0.25 * W )
            self.X6 = int( 0.60 * W )
            self.W3 = int( 0.35 * W )
            self.W4 = int( 0.30 * W )
            
            fr = 24.0 / QFontMetrics( QFont( 'Arial', 24, QFont.Weight.Bold ) ).horizontalAdvance( '00' )
            self.px3 = int( fr * self.W3 )
            fr = 24.0 / QFontMetrics( QFont( 'Arial', 24, QFont.Weight.Bold ) ).horizontalAdvance( '000' )
            self.px4 = int( fr * self.W4 )
            
            
            self.X7 = int( 0.20 * W )
            self.W5 = int( 0.75 * W )
            
            
            
            
        def paintEvent( self, event ) :
            #print('paintEvent AltitudeMeter')
                        
            W, H = self.width(), self.height()
        
            V = self.value
            
            R, p1, p2 = 1000.0, 100.0, 200.0
            n = int( R / p1 )
            
#             w10, w20, w25  = int( 0.10*W ), int( 0.20*W ), int( 0.25*W )
#             w30, w35, w45  = int( 0.30*W ), int( 0.35*W ), int( 0.45*W )
#             w55, w65, w90  = int( 0.55*W ), int( 0.65*W ), int( 0.90*W )
#             h20, h40, h50  = int( 0.20*H ), int( 0.40*H ), int( 0.50*H )

            #w25 = int( 0.25*W )
            #w30, w35, w45  = int( 0.30*W ), int( 0.35*W ), int( 0.45*W )
            #w55, w65, w90  = int( 0.55*W ), int( 0.65*W ), int( 0.90*W )
            h20, h40, h50  = int( 0.20*H ), int( 0.40*H ), int( 0.50*H )

            px0 = h20 * 24.0 / QFontMetrics( QFont( 'Arial', 24 ) ).height()
            px4, px5, px6 = int( px0*0.4 ), int( px0*0.5 ), int( px0*0.6 )

            
            #fr = 24.0 / QFontMetrics( QFont( 'Arial', 24 ) ).horizontalAdvance( '000' )
            #px4 = int( fr * self.W2 )

            painter = QPainter( self )
            painter.setPen( QPen( Qt.GlobalColor.white, 1 ) )
            painter.setBrush( Qt.GlobalColor.black )

            painter.drawRect( -1, -1, W+1, H+1 )
            v = round( ( V - R / 2.0 ) / p2 ) * p2
            for i in range( n + 1 ) :
                y = int( H * ( 0.5 + ( V - v ) / R ) )
                if i%2 == 0 :
                    painter.drawLine( 0, y, self.X2, y )
                    s = str( round( v ) )
                    s1, s2 = s[:-3], ( '000' + s[-3:] )[-3:]
                    painter.setFont( QFont( 'Arial', self.px1, QFont.Weight.Normal, False ) )
                    painter.drawText( self.X3, int( y-25 ), self.W1, 50, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, s1 )
                    painter.setFont( QFont( 'Arial', self.px2, QFont.Weight.Normal, False ) )
                    painter.drawText( self.X4, int( y-25 ), self.W2, 50, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, s2 )
                else :
                    painter.drawLine( 0, y, self.X1, y )
                v += p1
            painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )
            painter.drawRect( self.X7, h40, self.W5, h20 )
            painter.setPen( QPen( Qt.GlobalColor.white, 3 ) )
            s = str( round( V ) )
            s1, s2 = s[:-3], ( '000' + s[-3:] )[-3:]
            #painter.setFont( QFont( 'Arial', px6, QFont.Weight.Bold, False ) )
            painter.setFont( QFont( 'Arial', self.px3, QFont.Weight.Bold, False ) )
            painter.drawText( self.X5, h40, self.W3, h20, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, s1 )
            painter.setFont( QFont( 'Arial', self.px4, QFont.Weight.Bold, False ) )
            painter.drawText( self.X6, h40, self.W4, h20, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, s2 )
            painter.drawLine( 0, h50, self.X2, h50 )


    class AttitudeIndicator( QWidget ) :
        
        def __init__( self, parent = None ) :
            
            super().__init__( parent )
            self.phiDeg = 0.0
            self.theDeg = 0.0
            
        def paintEvent( self, event ) :
            #print('paintEvent AttitudeIndicator')
            
            WW, HH = self.width(), self.height()
            W0, H0 = 100.0, 100.0
            a = min( WW / W0, HH / H0 )
            W, H = W0 * a, H0 * a
            
            Tmax = 25.0 # deg
            dt   =  2.5 # deg
            L = int( 2.0 * max( W, H ) )
            R  = H / 2.0
            
            R05, R10, R15 = int( 0.05*R ), int( 0.10*R ), int( 0.15*R )
            R20, R30, R40 = int( 0.20*R ), int( 0.30*R ), int( 0.40*R )
            R60, R70, R80 = int( 0.60*R ), int( 0.70*R ), int( 0.80*R )
            
            K = R60 / Tmax        
            fr = 24.0 / QFontMetrics( QFont( 'Arial', 24 ) ).height()
            px = int( R / ( Tmax / dt / 4.0 + 1.0 ) * fr * 0.6 )
            
            painter = QPainter( self )
            #painter.setRenderHint( QPainter.RenderHint.Antialiasing )
            
            #---- SKY BLUE ------
            painter.setPen( Qt.PenStyle.NoPen )
            painter.setBrush( QColor( 30, 144, 255 ) )
            painter.drawRect( QRectF( QPointF( 0.0, 0.0 ), QSizeF( WW, HH ) ) )
                        
            painter.translate( int( WW / 2.0 ), int( HH / 2.0 ) )
            
            painter.save()
            #painter.rotate( int( - self.phiDeg ) )
            painter.rotate( - self.phiDeg )
            painter.translate( 0, int( K * self.theDeg ) )
            
            #---- GROUND BROWN ------
            painter.setBrush( QColor( 165, 42, 42 ) )
            painter.drawRect( QRectF( -L, 0, 2*L, 2*L ) )

            #---- PITCH TICK ------
            painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )
            painter.setFont( QFont( 'Arial', px, QFont.Weight.Normal, False ) )
            #painter.setFont( QFont( 'Arial', px, QFont.Normal, False ) )
            k1 = round( ( self.theDeg - Tmax ) / dt )
            k2 = round( ( self.theDeg + Tmax ) / dt )
            for k in range( k1, k2+1 ) :
                y = int( dt * k * K )
                if k % 4 == 0 :
                    painter.drawLine( QPoint( -R30, -y ), QPoint( R30, -y ) )
                    v = round( abs( dt * k ) )
                    xt1, xt2, yt, wt = int( R30+5 ), int(-R30-5-2*px ), int(-y-px), int(2*px)
                    painter.drawText( xt1, yt, wt, wt, Qt.AlignmentFlag.AlignLeft  | Qt.AlignmentFlag.AlignVCenter, str(v) )
                    painter.drawText( xt2, yt, wt, wt, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, str(v) )
                elif  k % 2 == 0 : 
                    painter.drawLine( QPoint( -R15, -y ), QPoint( R15, -y ) )
                else : 
                    painter.drawLine( QPoint( -R05, -y ), QPoint( R05, -y ) )
            
            #---- TRIANGLE ------
            painter.translate( 0, - int( K * self.theDeg ) )
            painter.setBrush( Qt.BrushStyle.NoBrush )
            painter.drawPolygon( [ QPoint(    0, -R80 ),
                                   QPoint(  R10, -R70 ),
                                   QPoint( -R10, -R70 ) ] )
            painter.restore()
            
            #---- ROLL TICK ------
            painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )
            ang = np.radians( np.arange( 36 ) * 10.0 )
            X, Y = R80 * np.cos( ang ), R80 * np.sin( ang )
            for i in range( len( ang ) ) :
                p = QPointF( X[i], Y[i] )
                painter.drawLine( p, ( 1.08 if i%3==0 else 1.05 ) * p )

            #---- AIRPLANE ------
            painter.setPen( QPen( Qt.GlobalColor.cyan , 3 ) )
            painter.drawPolyline( [ QPoint( -R40,   0 ),
                                    QPoint( -R20,   0 ),
                                    QPoint( -R10, R20 ),
                                    QPoint(    0,   0 ),
                                    QPoint(  R10, R20 ),
                                    QPoint(  R20,   0 ),
                                    QPoint(  R40,   0 ) ] )


    class HSI( QWidget ) :
        
        def __init__( self, parent = None ) :
            
            super().__init__( parent )
            
            #self.setMouseTracking( True )
            
            self.psiDeg = 0.0
            
            self.tacanDirect = 30.0
            self.tacanDist    = 12.345
            self.tacanCourse  = 123.0
            self.tacanHeading =  30.0
            self.magVariation_deg = 0.0
            self.TacanXY_m        = 0.0
            
            self.mousePressed = False
            self.mousePos = QPoint( 0, 0 )
            
            
        def setAircraft( self, aircraft ) :
            #print('setAircraft HSI')
            self.aircraft = aircraft
            
            
        def setAirport( self, airport ) :
            print( 'setAirport HSI' )
            self.magVariation_deg = airport[ 'MagVariation_deg' ]
            
            loc0 = np.radians( airport[ 'Location_RwyCntr' ] )
            loc1 = np.radians( airport[ 'Location_TACAN' ] )
            self.TacanXY_m = 6356766.0 * ( loc1 - loc0 )
            self.TacanXY_m[1] *= np.cos( loc0[0] )
                        
            #print( self.TacanXY_m )
            
        def resizeEvent( self, event ) :
            #print('resizeEvent HSI')
            
            W, H = self.width(), self.height()
            
            #fr = 24.0 / QFontMetrics( QFont( 'Arial', 24, QFont.Weight.Normal ) ).horizontalAdvance( '00' )
            #self.px1 = int( fr * 20 )
            
            px = int( 0.10 * H * 24.0 / QFontMetrics( QFont( 'Arial', 24, QFont.Weight.Normal ) ).horizontalAdvance( '00' ) )
            self.font0 = QFont( 'Arial', px, QFont.Weight.Normal, False )
            
            px = int( 0.06 * H * 24.0 / QFontMetrics( QFont( 'Arial', 24, QFont.Weight.Normal ) ).horizontalAdvance( '00' ) )
            self.font1 = QFont( 'Arial', px, QFont.Weight.Normal, False )
            
            px = int( 0.08 * H * 24.0 / QFontMetrics( QFont( 'Arial', 24, QFont.Weight.Normal ) ).horizontalAdvance( '00' ) )
            self.font2 = QFont( 'Arial', px, QFont.Weight.Normal, False )
            
#             fr = 24.0 / QFontMetrics( QFont( 'Arial', 24, QFont.Weight.Normal ) ).horizontalAdvance( '000' )
#             self.px2 = int( fr * self.W2 )

            xr, yr = W * 0.87, H * 0.25
            wr, hr = W * 0.10, H * 0.50
            self.tacanCourseDialRect = QRectF( xr, yr, wr, hr )
            self.tacanCourseDialVal = 0.0
            
            xr, yr = W * 0.03, H * 0.25
            wr, hr = W * 0.10, H * 0.50
            self.tacanHeadingDialRect = QRectF( xr, yr, wr, hr )
            self.tacanHeadingDialVal = 0.0
            
                        
            
        def mousePressEvent( self, event ) :
            #print( 'mousePressEvent HSI' )
            self.mousePressed = True
            self.mousePos = event.pos()

        def mouseReleaseEvent( self, event ) :
            #print( 'mouseReleaseEvent HSI' )
            self.mousePressed = False

        def mouseMoveEvent( self, event ) :
            #print( 'mouseMoveEvent HSI' )
            dXY = event.pos() - self.mousePos
            if self.tacanCourseDialRect.contains( QPointF( event.pos() ) ) :
                dY = dXY.y()
                self.tacanCourseDialVal += dY / self.tacanCourseDialRect.height()
                self.tacanCourseDialVal = self.tacanCourseDialVal % 1.0
                self.tacanCourse += dY
                
            elif self.tacanHeadingDialRect.contains( QPointF( event.pos() ) ) :
                dY = dXY.y()
                self.tacanHeadingDialVal += dY / self.tacanHeadingDialRect.height()
                self.tacanHeadingDialVal = self.tacanHeadingDialVal % 1.0
                self.tacanHeading += dY
                
            self.mousePos = event.pos()  
            self.update()               
                
        def wheelEvent( self, event ) :
            #print( 'wheelEvent HSI' )
            
            if self.tacanCourseDialRect.contains( QPointF( event.position() ) ) :
                dY = np.sign( event.angleDelta().y() ) * ( -1.0 )
                #print( dY )
                
                self.tacanCourseDialVal += dY / self.tacanCourseDialRect.height()
                self.tacanCourseDialVal = self.tacanCourseDialVal % 1.0
                self.tacanCourse += dY
                
                self.update()
            
            elif self.tacanHeadingDialRect.contains( QPointF( event.position() ) ) :
                dY = np.sign( event.angleDelta().y() ) * ( -1.0 )
                #print( dY )
                
                self.tacanHeadingDialVal += dY / self.tacanHeadingDialRect.height()
                self.tacanHeadingDialVal = self.tacanHeadingDialVal % 1.0
                self.tacanHeading += dY
                
                self.update()
            
            
        def paintEvent( self, event ) :
            #print('paintEvent HSI')

            if self.aircraft is not None :
                
                psiTrueDeg = self.aircraft.eul_deg[2]
            
                self.psiDeg = psiTrueDeg - self.magVariation_deg
                
                dXY = self.TacanXY_m - self.aircraft.XYZ[0:2]
                tacanTrueDirect  = np.degrees( np.arctan2( dXY[1], dXY[0] ) )
                self.tacanDirect = tacanTrueDirect - self.magVariation_deg
                self.tacanDist   = np.linalg.norm( dXY ) / 1852.0
                
                dCourse = self.tacanCourse - self.tacanDirect
                dCourse = ( ( dCourse + 180.0 ) % 360.0 ) - 180.0
                
            WW, HH = self.width(), self.height()
            
            R1 = HH * 0.39
            R2 = HH * 0.37
            R3 = HH * 0.36
            R4 = HH * 0.34
            
            painter = QPainter( self )
            #painter.setRenderHint( QPainter.RenderHint.Antialiasing )
            
            painter.setPen( Qt.PenStyle.NoPen )
            
            #
            #  BACKGROUND
            #
            painter.setBrush( QColor( 0, 0, 100 ) )
            painter.drawRect( QRectF( QPointF( 0.0, 0.0 ), QSizeF( WW, HH ) ) )


            #
            #  TACAN COURSE DIAL
            #
            painter.setBrush( QColor( 0, 0, 128 ) )
            painter.drawRect( self.tacanCourseDialRect )

            painter.save()
            painter.setClipRect( self.tacanCourseDialRect )

            xr, yr = self.tacanCourseDialRect.left() , self.tacanCourseDialRect.top()
            wr, hr = self.tacanCourseDialRect.width(), self.tacanCourseDialRect.height()
            
            x1 = xr + 0.1 * wr
            w1, h1 = 0.8 * wr, 0.05 * hr
            v1 = ( self.tacanCourseDialVal * 10 ) % 1.0
            painter.setBrush( QColor( 80, 80, 80 ) )
            for k in range( -1, 10 ) :
                y1 = yr + ( k + v1 ) * hr / 10
                painter.drawRect( QRectF( x1, y1, w1, h1 ) )
            
            painter.restore()


            #
            #  TACAN HEADING DIAL
            #
            painter.setBrush( QColor( 0, 0, 128 ) )
            painter.drawRect( self.tacanHeadingDialRect )

            painter.save()
            painter.setClipRect( self.tacanHeadingDialRect )

            xr, yr = self.tacanHeadingDialRect.left() , self.tacanHeadingDialRect.top()
            wr, hr = self.tacanHeadingDialRect.width(), self.tacanHeadingDialRect.height()
            
            x1 = xr + 0.1 * wr
            w1, h1 = 0.8 * wr, 0.05 * hr
            v1 = ( self.tacanHeadingDialVal * 10 ) % 1.0
            #painter.setBrush( QColor( 0, 100, 0 ) )
            painter.setBrush( QColor( 80, 80, 80 ) )
            for k in range( -1, 10 ) :
                y1 = yr + ( k + v1 ) * hr / 10
                painter.drawRect( QRectF( x1, y1, w1, h1 ) )
            
            painter.restore()


            painter.setBrush( Qt.BrushStyle.NoBrush )
            painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )

            painter.setFont( self.font0 )
            align = Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter

            #
            #  TACAN MILES
            #
            xr, yr = 0.0      , HH * 0.03
            wr, hr = HH * 0.25, HH * 0.12
            rect = QRectF( xr, yr, wr, hr )
            painter.drawRect( rect )

            painter.save()
            painter.setClipRect( rect )
            
            #print( self.tacanDist )
            
            h0 = HH * 0.09
            w0 = HH * 0.25 / 3
            y0 = yr + 0.5 * hr
            rect = QRectF( 1.5 * w0, y0 - 0.5 * h0, w0, h0 )
            painter.drawText( rect, align, '.' ) 
            xx = [ 2.0 * w0,  1.0 * w0,  0.2 * w0 ]
            dv, zz = np.modf( self.tacanDist * 10.0 )
            vals = [ 0, 0 ] + [ int( z ) for z in str( round( zz ) ) ]
            flag = True
            for k in range( 3 ) :    
                val = vals[ - 1 - k ]
                for i in range( -1, 2 ) :
                    s = val + i 
                    if flag : y = y0 - h0 * ( float( i ) - dv )
                    else    : y = y0 - h0 *   float( i ) 
                    rect = QRectF( xx[ k ], y - 0.5 * h0, w0, h0 )            
                    painter.drawText( rect, align, str( s % 10 ) ) 
                flag = flag and val == 9               
            painter.restore()


            #
            #  TACAN COURSE
            #
            xr, yr = WW - HH * 0.25, HH * 0.03
            wr, hr = HH * 0.25     , HH * 0.12
            rect = QRectF( xr, yr, wr, hr )
            painter.drawRect( rect )

            painter.save()
            painter.setClipRect( rect )
            
            h0 = HH * 0.09
            w0 = wr / 3
            y0 = yr + 0.5 * hr
            xx = [ xr + 2.0 * w0,  xr + 1.0 * w0,  xr ]
            tacanCourse = round( self.tacanCourse ) % 360
            vals = [ 0, 0 ] + [ int( z ) for z in str( tacanCourse ) ]
            for k in range( 3 ) :    
                val = vals[ - 1 - k ]
                rect = QRectF( xx[ k ], y0 - 0.5 * h0, w0, h0 )            
                painter.drawText( rect, align, str( val ) ) 
                
            painter.restore()


        
            painter.setFont( self.font1 )
            
            align = Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop
            rect = QRectF( 0.0, HH * 0.15, HH * 0.25, HH * 0.10 )
            painter.drawText( rect, align, 'MILES' )

            rect = QRectF( WW - HH * 0.25, HH * 0.15, HH * 0.25, HH * 0.10 )
            painter.drawText( rect, align, 'COURSE' )


            painter.translate( int( WW / 2.0 ), int( HH / 2.0 ) )
            
            painter.save()
            for k in range( 8 ) :
                painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )
                painter.drawLine( QPointF( 0.0, - 0.46 * HH ),
                                  QPointF( 0.0, - 0.43 * HH ) )
                painter.rotate( 45 )
            painter.restore()
            
            painter.save()
            #painter.rotate( round( - self.psiDeg ) )
            
            painter.rotate( - self.psiDeg )
            
            #
            #  TICKS FOR DIRECTION
            #
            painter.save()
            painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )
            painter.setFont( self.font2 )
            txts = [ 'N', '3', '6', 'E', '12', '15', 'S', '21', '24', 'W', '30', '33']
            for txt in txts :
                align = Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop
                painter.drawText( QRectF( -50, -R4, 100, 100 ), align, txt )
                for k in range( 6 ) :
                    if k % 2 == 0 :
                        painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )
                        painter.drawLine( QPointF( 0.0, -R1 ), QPointF( 0.0, - R3 ) )
                    else :
                        painter.setPen( QPen( Qt.GlobalColor.white, 1 ) )
                        painter.drawLine( QPointF( 0.0, -R1 ), QPointF( 0.0, - R2 ) )
                    painter.rotate( 5 )
            painter.restore()


            #
            #  TACAN DIRECTION
            #
            painter.save()
            painter.rotate( round( self.tacanDirect ) )

            painter.setPen( Qt.PenStyle.NoPen )
            painter.setBrush( QColor( 255, 255, 0 ) )
            
            arrowHead = QPolygonF( [ QPointF(  0.0      , -0.45 * HH ),
                                     QPointF(  0.03 * HH, -0.42 * HH ),
                                     QPointF(  0.03 * HH, -0.39 * HH ),
                                     QPointF( -0.03 * HH, -0.39 * HH ),
                                     QPointF( -0.03 * HH, -0.42 * HH ) ] )
            painter.drawPolygon( arrowHead )
            
            arrowTail = QPolygonF( [ QPointF(  0.03 * HH,  0.45 * HH ),
                                     QPointF(  0.03 * HH,  0.39 * HH ),
                                     QPointF( -0.03 * HH,  0.39 * HH ),
                                     QPointF( -0.03 * HH,  0.45 * HH ) ] )
            painter.drawPolygon( arrowTail )
            
            painter.setPen( QPen( QColor( 255, 0, 0 ), 2 ) )
            painter.drawLine( QPointF(  0.0      , -0.445 * HH ),
                              QPointF(  0.0      , -0.395 * HH ) )
            painter.drawLine( QPointF(  0.0      ,  0.395 * HH ),
                              QPointF(  0.0      ,  0.445 * HH ) )

            painter.restore()

            #
            #  TACAN COURSE
            #
            painter.save()
            painter.rotate( round( self.tacanCourse ) )

            painter.setPen( QPen( Qt.GlobalColor.white, 2 ) )
            
            painter.drawLine( QPointF( 0.0, -0.06 * HH ),
                              QPointF( 0.0,  0.09 * HH ) )
            
            painter.drawLine( QPointF(  0.06 * HH, 0.0 ),
                              QPointF( -0.06 * HH, 0.0 ) )
            
            painter.drawLine( QPointF(  0.02 * HH, 0.08 * HH ),
                              QPointF( -0.02 * HH, 0.08 * HH ) )
            
            painter.setPen( Qt.PenStyle.NoPen )
            painter.setBrush( Qt.GlobalColor.white )
            
            r = 0.012 * HH
            for x in [ 0.10 * HH, 0.20 * HH ] :
                painter.drawEllipse( QPointF(  x, 0.0 ), r, r )
                painter.drawEllipse( QPointF( -x, 0.0 ), r, r )
            
            arrowHead = QPolygonF( [ QPointF(  0.0       , -0.35 * HH ),
                                     QPointF(  0.015 * HH, -0.28 * HH ),
                                     QPointF(  0.015 * HH, -0.19 * HH ),
                                     QPointF( -0.015 * HH, -0.19 * HH ),
                                     QPointF( -0.015 * HH, -0.28 * HH ) ] )
            painter.drawPolygon( arrowHead )
            
            arrowTail = QPolygonF( [ QPointF(  0.0       ,  0.35 * HH ),
                                     QPointF(  0.015 * HH,  0.24 * HH ),
                                     QPointF( -0.015 * HH,  0.24 * HH ) ] )
            painter.drawPolygon( arrowTail )
            
            painter.setPen( QPen( QColor( 255, 0, 0 ), 1 ) )
            painter.drawLine( QPointF(  0.0      , -0.34  * HH ),
                              QPointF(  0.0      , -0.195 * HH ) )
            painter.drawLine( QPointF(  0.0      ,  0.33  * HH ),
                              QPointF(  0.0      ,  0.245 * HH ) )

            x = 0.20 * HH * np.clip( dCourse / 5.0, -1.0, 1.0 )
            painter.setPen( QPen( QColor( 255, 255, 255 ), 2 ) )
            painter.drawLine( QPointF( - x, -0.25 * HH ),
                              QPointF( - x,  0.25 * HH ) )


            painter.restore()

            #
            #  TACAN HEADING
            #
            painter.save()
            painter.rotate( round( self.tacanHeading ) )
            
            painter.setPen( Qt.PenStyle.NoPen )
            painter.setBrush( QColor( 255, 255, 255 ) )
            
            rectPolyg = QPolygonF( [ QPointF(  0.04 * HH, -0.42 * HH ),
                                     QPointF(  0.04 * HH, -0.39 * HH ),
                                     QPointF( -0.04 * HH, -0.39 * HH ),
                                     QPointF( -0.04 * HH, -0.42 * HH ) ] )
            painter.drawPolygon( rectPolyg )
            
            painter.setPen( QPen( QColor( 0, 128, 0 ), 2 ) )
            painter.drawLine( QPointF(  0.0      , -0.42 * HH ),
                              QPointF(  0.0      , -0.39 * HH ) )
            
            painter.restore()

            painter.restore()  # painter.rotate( round( - self.psiDeg ) )


# 
# class PfdWidget( QWidget, PFDOBJECT ) :
#     
#     def __init__( self, parent = None ) :
#         
#         super().__init__()
#         
#         self.attitudeIndicator    = self.AttitudeIndicator( self )
#         self.speedMeter           = self.SpeedMeter( self ) 
#         self.altitudeMeter        = self.AltitudeMeter( self )
#         self.verticalSpeedMeter   = self.VerticalSpeedMeter( self )
#         self.enginePowerIndicator = self.EnginePowerIndicator( self )
#         
#     def setValue( self, val ) :
#         self.speedMeter.value           = val[0]
#         self.attitudeIndicator.phiDeg   = val[1]
#         self.attitudeIndicator.theDeg   = val[2]
#         self.altitudeMeter.value        = val[3]
#         self.verticalSpeedMeter.value   = val[4]
#         self.enginePowerIndicator.value = val[5]
#         self.update()
#                 
#     def resizeEvent( self, event ) :
#         print('resizeEvent PfdWidget')
#         
#         WW, HH = self.width(), self.height()
#         
#         KA = min( WW / 300.0, HH / 100.0 )
#         WA, HA = KA * 300.0, KA * 100.0
#         XA, XB, YC = 0,  WA*0.6, HH*0.5   
#         W1, W2, W3, W4, W5 = WA*0.09 , WA*0.60, WA*0.09  , HA*0.50, HA*0.50
#         H1, H2, H3, H4, H5 = W1*2.5  , HA     , W3*2.5   , W4     , W5
#         X1, X2, X3, X4, X5 = 0.06*WA , -XA    , 0.45*WA  , 0      , W4
#    
#         self.speedMeter.setGeometry          ( int( XA + X1 ), int( YC-H1/2.0 ), int( W1 ), int( H1 ) )
#         self.attitudeIndicator.setGeometry   ( int( XA + X2 ), int( YC-H2/2.0 ), int( W2 ), int( H2 ) )
#         self.altitudeMeter.setGeometry       ( int( XA + X3 ), int( YC-H3/2.0 ), int( W3 ), int( H3 ) )
#         self.verticalSpeedMeter.setGeometry  ( int( XB + X4 ), int( YC-H4/2.0 ), int( W4 ), int( H4 ) )
#         self.enginePowerIndicator.setGeometry( int( XB + X5 ), int( YC-H5/2.0 ), int( W5 ), int( H5 ) )
# 

class PfdWidget( QWidget, PFDOBJECT ) :
    
    def __init__( self, parent = None ) :
        
        super().__init__()
        
        self.ownAircraft = None

        self.hsi                  = self.HSI( self )

        self.attitudeIndicator    = self.AttitudeIndicator( self )
        self.speedMeter           = self.SpeedMeter( self ) 
        self.altitudeMeter        = self.AltitudeMeter( self )
        self.verticalSpeedMeter   = self.VerticalSpeedMeter( self )
#         self.enginePowerIndicator = self.EnginePowerIndicator( self )
        
        
    def setAirport( self, airport ) :
        print( 'setAirport PfdWidget')
        
        self.magVariation_deg = airport[ 'MagVariation_deg' ]
        
        self.hsi.setAirport( airport )
        
        
#     'DEFAULT' : { 'MagVariation_deg'  : 0.0,
#                   'RwyTrueBRG_deg'    : 0.0,
#                   'RwyElevation_ft'   : 0.0,
#                   'TacanXY_m'         : ( 0.0, 0.0 ) } ,
        
        
    def setValue( self, val ) :
        self.speedMeter.value           = val[0]
        self.attitudeIndicator.phiDeg   = val[1]
        self.attitudeIndicator.theDeg   = val[2]
        self.altitudeMeter.value        = val[4]
        self.verticalSpeedMeter.value   = val[5]
#         self.enginePowerIndicator.value = val[5]

        self.hsi.psiDeg                 = val[3]
        self.hsi.tacanDist              = val[6]
        self.hsi.tacanDirect            = val[7]


        self.update()
                
                
    def setAircrafts( self, aircrafts ) :
        #print( 'setAircrafts PfdWidget' )
        for ac in aircrafts :
            if ac is None : continue
            if ac.category == 'own' :
                self.ownAircraft = ac
                
                self.hsi.setAircraft( self.ownAircraft )
        

    def update( self ) :
        #print( 'update PfdWidget' )
        if self.ownAircraft is not None :
            self.speedMeter.value          = self.ownAircraft.Vc_kt
            self.attitudeIndicator.phiDeg  = self.ownAircraft.eul_deg[0]
            self.attitudeIndicator.theDeg  = self.ownAircraft.eul_deg[1]
            self.altitudeMeter.value       = self.ownAircraft.alt_sl_ft
            self.verticalSpeedMeter.value  = self.ownAircraft.UVW_fps[2] * ( -60.0 / 1000.0 ) 
                
        self.hsi.update()
        self.speedMeter.update()
        self.attitudeIndicator.update()
        self.altitudeMeter.update()
        self.verticalSpeedMeter.update()



    def resizeEvent( self, event ) :
        #print('resizeEvent PfdWidget')
        
        WW, HH = self.width(), self.height()
        
        
        W0, H0 = HH * 1.3  , HH         # HSI    
        W1, H1 = HH / 2.8  , HH         # SpeedMeter    
        W2, H2 = HH        , HH         # A.I. 
        W3, H3 = HH / 2.8  , HH         # AltitudeMeter    
        W4, H4 = HH / 3.0  , HH * 0.70  # VerticalSpeedMeter    
        
        X0, Y0 = 0.0     , 0.0
        X1, Y1 = X0 + W0 , 0.0
        X2, Y2 = X1 + W1 , 0.0
        X3, Y3 = X2 + W2 , 0.0
        X4, Y4 = X3 + W3 , HH / 2.0 - H4 / 2.0
        
#         Y1 = 0.0
#         Y2 = 0.0
#         Y3 = 0.0
#         Y4 =  HH / 2.0 - H4 / 2.0

#         H1 = HH / 1.0
#         W1 = H1       
#         #X1 = WW / 2.0 - W1 / 2.0
#         X1 = H1
# 
#         H2 = H1
#         W2 = H2 / 2.8
#         X2 = X1 - W2
# 
#         H3 = H1
#         W3 = H3 / 2.8
#         X3 = X1 + W1
# 
#         H4 = H1 * 0.70
#         W4 = H4 / 3.0
#         X4 = X3 + W3
#         Y4 = H1 / 2.0 - H4 / 2.0


        self.hsi.setGeometry                 ( int( X0 ), int( 0.0 ), int( W0 ), int( H0 ) )
        self.speedMeter.setGeometry          ( int( X1 ), int( 0.0 ), int( W1 ), int( H1 ) )
        self.attitudeIndicator.setGeometry   ( int( X2 ), int( 0.0 ), int( W2 ), int( H2 ) )
        self.altitudeMeter.setGeometry       ( int( X3 ), int( 0.0 ), int( W3 ), int( H3 ) )
        self.verticalSpeedMeter.setGeometry  ( int( X4 ), int( Y4  ), int( W4 ), int( H4 ) )
#         self.enginePowerIndicator.setGeometry( int( XB + X5 ), int( YC-H5/2.0 ), int( W5 ), int( H5 ) )
        
        


class MainWindow( QMainWindow ) :

    def __init__(self, parent = None) :
        
        super().__init__()
        
        self.setGeometry( 300, 100, 1000, 700)
        self.setWindowTitle( 'Main Window' )
        
        centralWidget = QWidget()
        self.setCentralWidget( centralWidget )
        vLayout = QVBoxLayout( centralWidget )
        
        self.pfd = PfdWidget()
        vLayout.addWidget( self.pfd )
                
        self.sliders = MySliders( [ 'V'  ,  50.0,    0.0,   300.0 ],
                                  [ 'phi',   0.0, -180.0,   180.0 ],
                                  [ 'the',   0.0,  -90.0,    90.0 ],
                                  [ 'psi',   0.0,  -90.0,    90.0 ],
                                  [ 'h'  ,   0.0,    0.0, 30000.0 ],
                                  [ 'W'  ,   0.0,  -10.0,    10.0 ],
                                  [ 'Pwr',   0.0,   18.0,    25.0 ],
                                  [ 'Dir',   0.0,    0.0,   360.0 ],
                                  )
        self.sliders.valueChanged.connect( self.slidersValueChanged )
        
        vLayout.addWidget( self.sliders )
        
        vLayout.setStretch( 0, 2 )
        vLayout.setStretch( 1, 1 )
        
        self.update()
        
        
    def slidersValueChanged( self ) :
        #print( 'slidersValueChanged' )
        self.update()
        
    def update( self ) :
        #print( 'update' )
        self.pfd.setValue( self.sliders.getValue() )
        
        #self.pfd.setValue( self.sliders.getValue() )
        #self.speedMeter.value = self.sliders1.value()
        #self.engineMeter.value = self.sliders2.value()
        
        #self.speedMeter.update()
        #self.engineMeter.update()
                
    def buttonAClicked( self ) :
        print('buttonAClicked')

if __name__ == '__main__' :    
    app = QApplication( sys.argv )
    mainwindow = MainWindow()
    mainwindow.show()
    app.exec()


    