import os
import sys
import numpy as np
import copy
import time

# import cv2

from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *

from PyQt6.QtOpenGL import *
from PyQt6.QtOpenGLWidgets import QOpenGLWidget

sys.path.append( os.getcwd() + '\lib' )

from MyOpenGL import *
from MyPyqt import *
from MyFunc import *
from JSBAircraft import *
from DIYAircraft import *
from PfdWidget import *
from Hud import *
from Controler import *

# import tornado.httpserver
# import tornado.websocket
# import tornado.ioloop
# import tornado.web


# class FirstOrderLagFilter :
#     
#     def __init__( self, tc = 0.5 ) :
#         print( '__init__ FirstOrderLagFilter' )
#             
#         self.timeConst = tc
#         self.curTime = -1.0
#         self.curVal = 0.0
#         
#     def val( self, newTime, newVal ) :
#         
#         dt = newTime - self.curTime
#         
#         if dt < 0.0 or dt > 1.0 :
#             v = newVal
#         else :
#             v = self.curVal + ( newVal - self.curVal ) / self.timeConst * dt
#         
#         self.curTime = newTime
#         self.curVal = newVal
#         
#         return v
#         

# 
# 33.88306度 東経130.65306度
# 
#
# 
AIRPORTS = {
    'DEFAULT' : { 'MagVariation_deg'  : 0.0,
                  'Location_RwyCntr'  : ( 35.255489, 136.924382 ),
                  'RwyTrueBRG_deg'    : 0.0,
                  'RwyElevation_ft'   : 0.0,
                  'RwySize_m'         : ( 2700.0, 45.0 ),
                  'RwyDesigMarking'   : ( '36', '18' ),
                  'OverrunAreaLength' : 50.0,
                  'Location_TACAN'    : ( 35.265486, 136.914508 ),                 
                  } ,
    
    'NAGOYA'  : { 'MagVariation_deg'  : -8.0,
                  'Location_RwyCntr'  : ( 35.255489, 136.924382 ),
                  'RwyTrueBRG_deg'    : 332.42,
                  'RwyElevation_ft'   : 52.0,
                  'RwySize_m'         : ( 2740.0, 45.0 ),
                  'RwyDesigMarking'   : ( '34', '16' ),
                  'OverrunAreaLength' :  95.0,
                  'Location_TACAN'    : ( 35.265486, 136.914508 ),
                  },
    
    'ASHIYA'  : { 'MagVariation_deg'  : -8.0,
                  'Location_RwyCntr'  : ( 33.882602, 130.654307 ),
                  'RwyTrueBRG_deg'    : 115.78,
                  'RwyElevation_ft'   : 52.0,
                  'RwySize_m'         : ( 1640.0, 45.0 ),
                  'RwyDesigMarking'   : ( '12', '30' ),
                  'OverrunAreaLength' :  160.0,
                  'Location_TACAN'    : ( 33.887428,130.649867 ),
                  },
    
    }


# 
# 
# 
INITIAL_VALUE_SETTINGS = {
    'Takeoff' : [
        (    '0',   '10' ),  # Vc KT
        (   '15',   '10' ),  # ALT FT
        #(  '180',    '1' ),  # BRG DEG
        #(  '0.7',  '0.1' ),  # DIST NM
        ( '-0.7',  '0.1' ),  # X NM
        (  '0.0',  '0.1' ),  # Y NM
        (  '0.0',  '0.5' ),  # GAM DEG
        (  '0.0',  '0.5' ),  # PSI DEG
        (  2, 0, 1 )    ,  # FL SB GR 
    ],
    
    'Landing 150kt'   : [
        (  '150',   '10' ),  # Vc KT
        ( '1000',  '100' ),  # ALT FT
        #(  '180',    '1' ),  # BRG DEG
        #(  '4.0',  '0.1' ),  # DIST NM
        ( '-8.0',  '1.0' ),  # X NM
        (  '0.0',  '0.1' ),  # Y NM
        ( '-3.0',  '0.5' ),  # GAM DEG
        (  '0.0',  '0.5' ),  # PSI DEG
        (  1, 0, 1 )    ,  # FL SB GR 
    ],
    
    'Cruise 200kt 1000ft' : [
        (  '200',   '10' ),  # Vc KT
        ( '1000',  '100' ),  # ALT FT
        #(  '360',    '1' ),  # BRG DEG
        #(  '0.0',  '0.1' ),  # DIST NM
        (  '0.0',  '0.1' ),  # X NM
        (  '0.0',  '0.1' ),  # Y NM
        (  '0.0',  '0.5' ),  # GAM DEG
        (  '0.0',  '0.5' ),  # PSI DEG
        (  0, 0, 0 )    ,  # FL SB GR 
    ],
    
    'Cruise 300kt 1000ft' : [
        (  '300',   '10' ),  # Vc KT
        ( '1000',  '100' ),  # ALT FT
        #(  '360',    '1' ),  # BRG DEG
        #(  '0.0',  '0.1' ),  # DIST NM
        (  '0.0',  '0.1' ),  # X NM
        (  '0.0',  '0.1' ),  # Y NM
        (  '0.0',  '0.5' ),  # GAM DEG
        (  '0.0',  '0.5' ),  # PSI DEG
        (  0, 0, 0 )    ,  # FL SB GR 
    ],
    
}
 
 

 
class MyGLWidget( GLWidget ) :

#     objesCollection1 = ObjeCollection()

    airport = None

    def __init__( self, parent, setupFunc = None ) :
        print( '__init__ MyGLWidget' )
        
        super().__init__( parent, setupFunc )

        self.beforePaintGL = None
        self.afterPaintGL = None

        self.x_TDPoint = 920.0


    def numPoints( self, txt ) :
        #print( 'numPoints' )
        
        if txt == '' :
            pts = []

        elif txt == '1' :
            pts = np.array( [ [ 17.4, 1.6, 0.0 ], [ 18.0, 2.2, 0.0 ], [ 14.4, 1.6, 0.0],
                              [ 18.0, 2.2, 0.0 ], [ 14.4, 1.6, 0.0 ], [ 15.0, 2.2, 0.0],
                              [ 18.0, 2.2, 0.0 ], [ 18.0, 3.8, 0.0 ], [ 0.0, 2.2, 0.0],
                              [ 18.0, 3.8, 0.0 ], [ 0.0,  3.8, 0.0 ], [ 0.0, 2.2, 0.0 ] ] )
        
        elif txt == '2' :
            pts = np.array( [ [13.2,0.0,0.0],[18.0,0.0,0.0],[13.2,1.6,0.0],
                              [13.2,1.6,0.0],[18.0,0.0,0.0],[18.0,1.6,0.0],
                              [18.0,1.6,0.0],[18.0,6.0,0.0],[15.0,1.6,0.0],
                              [15.0,1.6,0.0],[18.0,6.0,0.0],[15.0,6.0,0.0],
                              [15.0,4.4,0.0],[15.0,6.0,0.0],[12.2,4.4,0.0],
                              [12.2,4.4,0.0],[15.0,6.0,0.0],[12.2,6.0,0.0],
                              [12.2,4.4,0.0],[12.2,6.0,0.0],[ 5.2,0.0,0.0],
                              [ 5.2,0.0,0.0],[12.2,6.0,0.0],[ 5.2,1.6,0.0],
                              [ 5.2,0.0,0.0],[ 5.2,1.6,0.0],[ 0.0,0.0,0.0],
                              [ 0.0,0.0,0.0],[ 5.2,1.6,0.0],[ 0.0,1.6,0.0],
                              [ 3.0,1.6,0.0],[ 3.0,6.0,0.0],[ 0.0,1.6,0.0],
                              [ 0.0,1.6,0.0],[ 3.0,6.0,0.0],[ 0.0,6.0,0.0],
                            ] )

        elif txt == '3' :
            pts = np.array( [ [ 18.0, 0.0, 0.0 ], [ 18.0, 6.0, 0.0 ], [ 15.0, 6.0, 0.0 ],
                              [ 18.0, 0.0, 0.0 ], [ 15.0, 6.0, 0.0 ], [ 15.0, 0.0, 0.0 ],
                              [ 15.0, 4.4, 0.0 ], [ 15.0, 6.0, 0.0 ], [ 13.6, 4.4, 0.0 ],
                              [ 15.0, 6.0, 0.0 ], [ 13.0, 6.0, 0.0 ], [ 13.6, 4.4, 0.0 ],
                              [ 10.8, 2.2, 0.0 ], [ 13.6, 4.4, 0.0 ], [ 13.0, 6.0, 0.0 ],
                              [ 10.8, 2.2, 0.0 ], [ 13.0, 6.0, 0.0 ], [ 10.8, 4.4, 0.0 ],
                              [ 10.8, 2.2, 0.0 ], [ 10.8, 4.4, 0.0 ], [  8.6, 6.0, 0.0 ],
                              [ 10.8, 2.2, 0.0 ], [  8.6, 6.0, 0.0 ], [  8.0, 4.4, 0.0 ],
                              [  8.0, 4.4, 0.0 ], [  8.6, 6.0, 0.0 ], [  3.0, 6.0, 0.0 ],
                              [  8.0, 4.4, 0.0 ], [  3.0, 6.0, 0.0 ], [  3.0, 4.4, 0.0 ],
                              [  3.0, 0.0, 0.0 ], [  3.0, 6.0, 0.0 ], [  0.0, 6.0, 0.0 ],
                              [  3.0, 0.0, 0.0 ], [  0.0, 6.0, 0.0 ], [  0.0, 0.0, 0.0 ] ] )
            
        elif txt == '4' :
            pts = np.array( [ [ 18.0,  2.1, 0.0 ], [ 18.0, 3.9, 0.0 ], [ 3.8, -0.9, 0.0 ],
                              [ 18.0,  3.9, 0.0 ], [  3.8, 0.9, 0.0 ], [ 3.8, -0.9, 0.0 ],
                              [  6.8,  1.0, 0.0 ], [  6.8, 6.9, 0.0 ], [ 3.8, -0.9, 0.0 ],
                              [  3.8, -0.9, 0.0 ], [  6.8, 6.9, 0.0 ], [ 3.8,  6.9, 0.0 ],
                              [ 12.6,  3.9, 0.0 ], [ 12.6, 5.5, 0.0 ], [ 0.0,  3.9, 0.0 ],
                              [ 12.6,  5.5, 0.0 ], [  0.0, 5.5, 0.0 ], [ 0.0,  3.9, 0.0 ] ] )
         
        elif txt == '5' :
            pts = np.array( [ [18.0,0.0,0.0],[18.0,6.0,0.0],[15.0,0.0,0.0],
                              [15.0,0.0,0.0],[18.0,6.0,0.0],[15.0,6.0,0.0],
                              [15.0,0.0,0.0],[15.0,1.6,0.0],[12.6,0.0,0.0],
                              [12.6,0.0,0.0],[15.0,1.6,0.0],[12.6,1.6,0.0],
                              [12.6,0.0,0.0],[12.6,6.0,0.0],[ 9.6,0.0,0.0],
                              [ 9.6,0.0,0.0],[12.6,6.0,0.0],[ 9.6,6.0,0.0],
                              [ 9.6,4.4,0.0],[ 9.6,6.0,0.0],[ 3.0,4.4,0.0],
                              [ 3.0,4.4,0.0],[ 9.6,6.0,0.0],[ 3.0,6.0,0.0],
                              [ 3.0,0.0,0.0],[ 3.0,6.0,0.0],[ 0.0,0.0,0.0],
                              [ 0.0,0.0,0.0],[ 3.0,6.0,0.0],[ 0.0,6.0,0.0] ] )
                                     
        elif txt == '6' :
            pts = np.array( [ [15.0,0.0,0.0],[19.0,4.5,0.0],[14.2,1.6,0.0],
                              [14.2,1.6,0.0],[19.0,4.5,0.0],[16.7,4.5,0.0],
                              [15.0,0.0,0.0],[14.2,1.6,0.0],[11.0,0.0,0.0],
                              [11.0,0.0,0.0],[14.2,1.6,0.0],[11.0,1.6,0.0],
                              [11.0,0.0,0.0],[11.0,6.0,0.0],[ 8.0,0.0,0.0],
                              [ 8.0,0.0,0.0],[11.0,6.0,0.0],[ 8.0,6.0,0.0],
                              [ 8.0,0.0,0.0],[ 8.0,1.6,0.0],[ 3.0,0.0,0.0],
                              [ 3.0,0.0,0.0],[ 8.0,1.6,0.0],[ 3.0,1.6,0.0],
                              [ 8.0,4.4,0.0],[ 8.0,6.0,0.0],[ 3.0,4.4,0.0],
                              [ 3.0,4.4,0.0],[ 8.0,6.0,0.0],[ 3.0,6.0,0.0],
                              [ 3.0,0.0,0.0],[ 3.0,6.0,0.0],[ 0.0,0.0,0.0],
                              [ 0.0,0.0,0.0],[ 3.0,6.0,0.0],[ 0.0,6.0,0.0] ] )
                                     
        elif txt == '7' :
            pts = np.array( [ [18.0,-0.5,0.0],[18.0,6.5,0.0],[16.4,-0.5,0.0],
                              [16.4,-0.5,0.0],[18.0,6.5,0.0],[16.4, 6.0,0.0],
                              [18.0, 4.9,0.0],[18.0,6.5,0.0],[ 0.0, 0.1,0.0],
                              [ 0.0, 0.1,0.0],[18.0,6.5,0.0],[ 0.0, 1.7,0.0] ] )
                                     
        elif txt == '8' :
            pts = np.array( [ [18.0,0.0,0.0],[18.0,6.0,0.0],[15.0,0.0,0.0],
                              [15.0,0.0,0.0],[18.0,6.0,0.0],[15.0,6.0,0.0],
                              [15.0,0.0,0.0],[15.0,1.6,0.0],[11.3,0.0,0.0],
                              [11.3,0.0,0.0],[15.0,1.6,0.0],[ 9.7,1.6,0.0],
                              [15.0,4.4,0.0],[15.0,6.0,0.0],[ 9.7,4.4,0.0],
                              [ 9.7,4.4,0.0],[15.0,6.0,0.0],[11.3,6.0,0.0],
                              [11.3,1.6,0.0],[11.3,4.4,0.0],[ 9.1,1.6,0.0],
                              [ 9.1,1.6,0.0],[11.3,4.4,0.0],[ 9.1,4.4,0.0],
                              [ 9.1,0.0,0.0],[10.7,1.6,0.0],[ 3.0,0.0,0.0],
                              [ 3.0,0.0,0.0],[10.7,1.6,0.0],[ 3.0,1.6,0.0],
                              [10.7,4.4,0.0],[ 9.1,6.0,0.0],[ 3.0,4.4,0.0],
                              [ 3.0,4.4,0.0],[ 9.1,6.0,0.0],[ 3.0,6.0,0.0],
                              [ 3.0,0.0,0.0],[ 3.0,6.0,0.0],[ 0.0,0.0,0.0],
                              [ 0.0,0.0,0.0],[ 3.0,6.0,0.0],[ 0.0,6.0,0.0] ] )
                          
        elif txt == '9' :
            pts = np.array( [ [18.0,0.0,0.0],[18.0,6.0,0.0],[15.0,0.0,0.0],
                              [15.0,0.0,0.0],[18.0,6.0,0.0],[15.0,6.0,0.0],
                              [15.0,0.0,0.0],[15.0,1.6,0.0],[10.0,0.0,0.0],
                              [10.0,0.0,0.0],[15.0,1.6,0.0],[10.0,1.6,0.0],
                              [15.0,4.4,0.0],[15.0,6.0,0.0],[10.0,4.4,0.0],
                              [10.0,4.4,0.0],[15.0,6.0,0.0],[10.0,6.0,0.0],
                              [10.0,0.0,0.0],[10.0,6.0,0.0],[ 7.0,0.0,0.0],
                              [ 7.0,0.0,0.0],[10.0,6.0,0.0],[ 7.0,6.0,0.0],
                              [ 7.0,4.4,0.0],[ 7.0,6.0,0.0],[ 3.8,4.4,0.0],
                              [ 3.8,4.4,0.0],[ 7.0,6.0,0.0],[ 3.0,6.0,0.0],
                              [ 3.8,4.4,0.0],[ 3.0,6.0,0.0],[ 2.2,1.6,0.0],
                              [ 2.2,1.6,0.0],[ 3.0,6.0,0.0],[-1.0,1.6,0.0] ] )
                          
        elif txt == '0' :
            pts = np.array( [ [18.0,0.0,0.0],[18.0,6.0,0.0],[15.0,0.0,0.0],
                              [15.0,0.0,0.0],[18.0,6.0,0.0],[15.0,6.0,0.0],
                              [15.0,0.0,0.0],[15.0,1.6,0.0],[ 3.0,0.0,0.0],
                              [ 3.0,0.0,0.0],[15.0,1.6,0.0],[ 3.0,1.6,0.0],
                              [15.0,4.4,0.0],[15.0,6.0,0.0],[ 3.0,4.4,0.0],
                              [ 3.0,4.4,0.0],[15.0,6.0,0.0],[ 3.0,6.0,0.0],
                              [ 3.0,0.0,0.0],[ 3.0,6.0,0.0],[ 0.0,0.0,0.0],
                              [ 0.0,0.0,0.0],[ 3.0,6.0,0.0],[ 0.0,6.0,0.0],
                             ] )
                      
        elif txt == 'L' :
            pts = np.array( [ [18.0,0.0,0.0],[18.0,1.6,0.0],[3.0,0.0,0.0],
                              [ 3.0,0.0,0.0],[18.0,1.6,0.0],[3.0,1.6,0.0],
                              [ 3.0,0.0,0.0],[ 3.0,6.0,0.0],[0.0,0.0,0.0],
                              [ 0.0,0.0,0.0],[ 3.0,6.0,0.0],[0.0,6.0,0.0] ] )
                      
        elif txt == 'R' :
            pts = np.array( [ [18.0,0.0,0.0],[18.0,1.6,0.0],[ 0.0,0.0,0.0],
                              [ 0.0,0.0,0.0],[18.0,1.6,0.0],[ 0.0,1.6,0.0],
                              [18.0,1.6,0.0],[18.0,4.4,0.0],[15.0,0.0,0.0],
                              [15.0,0.0,0.0],[18.0,4.4,0.0],[15.0,4.4,0.0],
                              [18.0,4.4,0.0],[18.0,6.0,0.0],[ 7.4,4.4,0.0],
                              [ 7.4,4.4,0.0],[18.0,6.0,0.0],[ 7.4,6.0,0.0],
                              [10.4,1.6,0.0],[10.4,4.4,0.0],[ 7.4,0.0,0.0],
                              [ 7.4,0.0,0.0],[10.4,4.4,0.0],[ 7.4,4.4,0.0],
                              [ 7.4,2.6,0.0],[ 7.4,4.2,0.0],[ 0.0,4.4,0.0],
                              [ 0.0,4.4,0.0],[ 7.4,4.2,0.0],[ 0.0,6.0,0.0] ] )
                      
        elif txt == 'C' :
            pts = np.array( [ [18.0,0.0,0.0],[18.0,1.6,0.0],[ 0.0,0.0,0.0],
                              [ 0.0,0.0,0.0],[18.0,1.6,0.0],[ 0.0,1.6,0.0],
                              [18.0,1.6,0.0],[18.0,4.4,0.0],[15.0,0.0,0.0],
                              [15.0,0.0,0.0],[18.0,4.4,0.0],[15.0,4.4,0.0],
                              [18.0,4.4,0.0],[18.0,6.0,0.0],[13.8,4.4,0.0],
                              [13.8,4.4,0.0],[18.0,6.0,0.0],[13.8,6.0,0.0],
                              [ 3.0,1.6,0.0],[ 3.0,4.4,0.0],[ 0.0,0.0,0.0],
                              [ 0.0,0.0,0.0],[ 3.0,4.4,0.0],[ 0.0,4.4,0.0],
                              [ 4.2,4.4,0.0],[ 4.2,6.0,0.0],[ 0.0,4.4,0.0],
                              [ 0.0,4.4,0.0],[ 4.2,6.0,0.0],[ 0.0,6.0,0.0] ] )
                      
                      
        #print( pts )
        
        return pts
        
        
    def mkFeildObject( self ) :
        print( 'mkFeildObject' )
        
        if self.airport is None : return
            
        if 'field' in self.comObjes :
            self.delComObje( 'field' )
            #return
            
        psi =   self.airport[ 'RwyTrueBRG_deg' ]
        Z0  = - self.airport[ 'RwyElevation_ft' ] * 0.3048
        print( Z0 )
        #
        #  FIELD
        #
        fieldObjeCollection = ObjeCollection()
        
        dcm = euler2dcm( [ 0.0, 0.0, psi ], degree = True )
        mat = np.identity( 4 )
        mat[ 0:3, 0:3 ] = dcm.T
        mat[ 2  , 3   ] = Z0
       
        fieldObjeCollection.posMat = mat
        
        #
        #  field  100 X 100 km  ( 2km pitch )
        #
        W = 100000.0
        D = 2000.0
        fieldBase = Rectangle( W, W )
        #fieldBase.color = [ 0.2, 0.2, 0.2, 1.0 ]
        fieldBase.color = [ 0.0, 0.5, 0.0, 1.0 ]
        fieldObjeCollection.add( fieldBase )
        
        A = W / 2.0  
        pp = []
        for d in np.arange( - A, A, D )  :
            pp.append( [ d, -A, 0.0,  d, A, 0.0 ] )
            pp.append( [ -A, d, 0.0,  A, d, 0.0 ] )
            
        fieldLines = Lines( pp )
        fieldLines.color = [ 1.0, 1.0, 1.0, 1.0 ]
        fieldLines.depthTestFlag = False
        fieldObjeCollection.add( fieldLines )
        
        #
        #  AIRPORT
        #
        airport = Rectangle( 3200.0, 200.0 ) 
        airport.color = [ 0.5, 0.5, 0.0, 1.0 ]
        airport.depthTestFlag = False
        fieldObjeCollection.add( airport )
                
        # #######################################################
        #rwy_Length     = 2700.0
        #rwy_Width      =   45.0
        
        rwy_Length = self.airport[ 'RwySize_m' ][0]
        rwy_Width  = self.airport[ 'RwySize_m' ][1]
       
        rwy_DesigMarking   = self.airport[ 'RwyDesigMarking' ]
        
        if 'OverrunAreaLength' in self.airport :
            OverrunAreaLength  = self.airport[ 'OverrunAreaLength' ]
        else :
            OverrunAreaLength  = 50.0
                    
        colorRwy  = [ 0.5, 0.5, 0.5, 1.0 ]
        colorLine = [ 1.0, 1.0, 1.0, 1.0 ]
        
        #
        #  RUNWAY
        #
        runwayBase = Rectangle( rwy_Length, rwy_Width )
        runwayBase.color = colorRwy
        runwayBase.depthTestFlag = False
        fieldObjeCollection.add( runwayBase )
               
        runwayWhiteLines = ManyRectanglesWithLine()
        runwayWhiteLines.color = colorLine
        runwayWhiteLines.depthTestFlag = False


        # Center Line
        #
        n = round( ( rwy_Length / 2.0 - 78.0 ) / 55.0 )
        L = ( rwy_Length / 2.0 - 78.0 ) / n
        x0 = L - 30.0 / 2.0
        #x0 = L / 2.0
        for k in range( n ) :
            x = L * k + x0
            runwayWhiteLines.addRect( 30.0, 1.0, 0, 1, [   x, 0.0, 0.0 ] )
            runwayWhiteLines.addRect( 30.0, 1.0, 0, 1, [ - x, 0.0, 0.0 ] )

        # Side Line
        y = ( rwy_Width - 1.0 ) / 2.0
        runwayWhiteLines.addRect( rwy_Length, 1.0, 0, 1, [ 0.0,   y, 0.0 ] )
        runwayWhiteLines.addRect( rwy_Length, 1.0, 0, 1, [ 0.0, - y, 0.0 ] )

        # Runway Center Sign
        for k in range( -1, 2) :
            runwayWhiteLines.addRect( 1.8, rwy_Width, 0, 1, [  3.6 * k, 0.0, 0.0 ] )
                  
        # Runway End Identifier
        x = rwy_Length / 2.0 - 21.0
        d = ( rwy_Width - 2.0 - 1.8 * 13.0 ) / 13.0
        for k in range( 6 ) :
            y = ( k + 1 ) * ( d + 1.8 )
            runwayWhiteLines.addRect( 30.0, 1.8, 0, 1, [  x,  y, 0.0 ] )
            runwayWhiteLines.addRect( 30.0, 1.8, 0, 1, [  x, -y, 0.0 ] )
            runwayWhiteLines.addRect( 30.0, 1.8, 0, 1, [ -x,  y, 0.0 ] )
            runwayWhiteLines.addRect( 30.0, 1.8, 0, 1, [ -x, -y, 0.0 ] )
          
        # Runway Marker
        for i in [ -1.0, 1.0 ] :
            for j in [ -1.0, 1.0 ] :
                
                # Aiming Point Marker
                x = rwy_Length / 2.0 - 430.0
                y = 14.0
                runwayWhiteLines.addRect( 60.0, 10.0, 0, 1, [ i * x, j * y, 0.0 ] )  
                
                # Touch Down Zone Marker
                x1 = rwy_Length / 2.0 - 150.0 - 22.5 / 2.0
                x2 = rwy_Length / 2.0 - 300.0 - 22.5 / 2.0
                for k in range( 3 ) :
                    y = 9.9 + k * 3.4
                    runwayWhiteLines.addRect( 22.5, 1.8, 0, 1, [ i * x1, j * y, 0.0 ] )
                    runwayWhiteLines.addRect( 22.5, 1.8, 0, 1, [ i * x2, j * y, 0.0 ] )
          
                x = rwy_Length / 2.0 - 600.0 - 22.5 / 2.0
                for k in range( 2 ) :
                    y = 9.9 + k * 3.4
                    runwayWhiteLines.addRect( 22.5, 1.8, 0, 1, [ i * x, j * y, 0.0 ] )
          
                x1 = rwy_Length / 2.0 - 750.0 - 22.5 / 2.0
                x2 = rwy_Length / 2.0 - 900.0 - 22.5 / 2.0
                y = 9.9
                runwayWhiteLines.addRect( 22.5, 3.0, 0, 1, [ i * x1, j * y, 0.0 ] )
                runwayWhiteLines.addRect( 22.5, 3.0, 0, 1, [ i * x2, j * y, 0.0 ] )
                    
        fieldObjeCollection.add( runwayWhiteLines.mkObject( depthTestFlag = False ) )
                

        
        #txts = rwy_DesigMarking
        
        x = - rwy_Length / 2.0 + 48
        y = [ -8.0, 2.0 ]            
        ix = [ 1.0, -1.0 ]
        for i in range( 2 ) :
            for j in range( 2 ) :
                pts = self.numPoints( rwy_DesigMarking[i][j] )
                brgText = Triangles( ix[i] *( pts + [ x, y[j], 0.0 ] ) )
                brgText.color = [ 1.0, 1.0, 1.0, 1.0 ]
                brgText.depthTestFlag = False
                fieldObjeCollection.add( brgText )
                
        
        #
        #  OVERRUN AREA 
        #
        ovrRun_L = OverrunAreaLength
        ovrRunBase_Color = [ 0.2, 0.2, 0.2, 1.0 ]
        ovrRunLine_Color = [ 1.0, 0.5, 0.0, 1.0 ]
        
        A, B = ovrRun_L, rwy_Width
        DX   = ( rwy_Length + ovrRun_L ) / 2.0
        for i in [ -1.0, 1.0 ] :
            overRunAreaBase = Rectangle( A, B, offset = [ i * DX, 0.0, 0.0 ] )
            overRunAreaBase.color = ovrRunBase_Color
            overRunAreaBase.depthTestFlag = False
            fieldObjeCollection.add( overRunAreaBase )
        
        L = 0.5 * ( rwy_Width - 15.0 )
        n = int( ( ovrRun_L - 6.0 ) / L - 0.5 )
        Lh = rwy_Length / 2.0
        w = 0.9 / np.sqrt( 2.0 )       
        W = 0.9 * np.sqrt( 2.0 )
        
        pts0 = np.array( [ [ 0.0        , 0.5 * L     , 0.0 ],
                           [ 0.5 * L    , L           , 0.0 ],
                           [ 0.5 * L + w, L - w       , 0.0 ],
                           [ 0.0        , 0.5 * L     , 0.0 ],
                           [ 0.5 * L + w, L - w       , 0.0 ],
                           [ 0.0        , 0.5 * L - W, 0.0 ] ] )
        
        pts1 = np.array( [ [ 0.0  , 0.0  , 0.0 ],
                           [ L    , L    , 0.0 ],
                           [ L + w, L - w, 0.0 ],
                           [ 0.0  , 0.0  , 0.0 ],
                           [ L + w, L - w, 0.0 ],
                           [ W    , 0.0  , 0.0 ] ] )
        
        LX = ovrRun_L - 6.0 - ( n + 0.5 ) * L
        pts2 = np.array( [ [ 0.0   , 0.0   , 0.0 ],
                           [ LX    , LX    , 0.0 ],
                           [ LX    , LX - W, 0.0 ],
                           [ 0.0   , 0.0   , 0.0 ],
                           [ LX    , LX - W, 0.0 ],
                           [ W     , 0.0   , 0.0 ] ] )
        
        pts = []
        for i in [ -1.0, 1.0 ] :
            for j in [ -1.0, 1.0 ] :
                pts.extend( ( pts0 + [ rwy_Length / 2.0, 0.0, 0.0 ]  ) *  [ i * 1.0, j * 1.0, 1.0 ] )
                for k in range( n ) :
                    x = rwy_Length / 2.0 + ( k + 0.5 ) * L
                    pts.extend( ( pts1 + [ x, 0.0, 0.0 ]  ) *  [ i * 1.0, j * 1.0, 1.0 ] ) 
                x = rwy_Length / 2.0 + ( n + 0.5 ) * L
                pts.extend( ( pts2 + [ x, 0.0, 0.0 ]  ) *  [ i * 1.0, j * 1.0, 1.0 ] )
    
        orangeLine = Triangles( pts )
        orangeLine.color = ovrRunLine_Color
        orangeLine.depthTestFlag = False
        fieldObjeCollection.add( orangeLine )
        
    
               
        #
        #  PAPI
        #
        papiObjeCol = ObjeCollection()
        #x       = - self.x_TDPoint 
        x       = - 920.0
        y, z    =  -60.0, -5.0
        w, h, r =   40.0,  10.0,  3.0
        y1, y2 = w / 8.0, w / 8.0 * 3.0
        papiBase = papiObjeCol.add( Rectangle( w, h, ax = 0, offset = [ x, y, z ] ) )
        papiBase.color = [ 0.2, 0.2, 0.2, 1.0, 1.0, 0.0, 0.0, 1.0 ]
        papiBase.depthTestFlag = False
        MyGLWidget.papi1 = [ papiObjeCol.add( Circle( r, ax = 0, offset = [ x, y - y2, z ] ) ),
                             papiObjeCol.add( Circle( r, ax = 0, offset = [ x, y - y1, z ] ) ),
                             papiObjeCol.add( Circle( r, ax = 0, offset = [ x, y + y1, z ] ) ),
                             papiObjeCol.add( Circle( r, ax = 0, offset = [ x, y + y2, z ] ) ) ]
        
        for p in MyGLWidget.papi1 :
            p.color = [ 2.0, 2.0, 2.0, 1.0 ]
            p.depthTestFlag = False
            p.shadowFactor = 1.0
                        
        fieldObjeCollection.add( papiObjeCol )
               
        
        self.addComObje( 'field', fieldObjeCollection, shadow = False )
            


    def createComObje( self ) :
        print( 'createComObje' )
 
        if hasattr( GLWidget, 'done_createComObje' ) :
            if GLWidget.done_createComObje :
                return
            
        GLWidget.done_createComObje = True
        
        
#         #
#         #  SPHERE
#         #
#         sphere = Sphere( 30.0 )
#         sphere.color = [ 0.0, 1.0, 0.0, 0.5 ]
#         self.addComObje( 'sphere', sphere, shadow = False )

        #
        #  SKY
        #
        sky = SkyObject( 30.0, img = 'sky.jpg' )
        #sky.skyFlag = False
        #sky.color = [ 1.0, 1.0, 1.0, 1.0 ]
        #sky.depthTestFlag = False
        self.addComObje( 'sky', sky, shadow = False )

    
        #
        #  FeildObject
        #
        self.mkFeildObject()
    
    
#         #
#         #  PAPI
#         #
#         papiObjeCol = ObjeCollection()
#         x       = - self.x_TDPoint 
#         y, z    =  -60.0, -5.0
#         w, h, r =   40.0,  10.0,  3.0
#         y1, y2 = w / 8.0, w / 8.0 * 3.0
#         papiBase = papiObjeCol.add( Rectangle( w, h, ax = 0, offset = [ x, y, z ] ) )
#         papiBase.color = [ 0.2, 0.2, 0.2, 1.0, 1.0, 0.0, 0.0, 1.0 ]
#         papiBase.depthTestFlag = False
#         MyGLWidget.papi1 = [ papiObjeCol.add( Circle( r, ax = 0, offset = [ x, y - y2, z ] ) ),
#                              papiObjeCol.add( Circle( r, ax = 0, offset = [ x, y - y1, z ] ) ),
#                              papiObjeCol.add( Circle( r, ax = 0, offset = [ x, y + y1, z ] ) ),
#                              papiObjeCol.add( Circle( r, ax = 0, offset = [ x, y + y2, z ] ) ) ]
#         
#         for p in MyGLWidget.papi1 :
#             p.color = [ 2.0, 2.0, 2.0, 1.0 ]
#             p.depthTestFlag = False
#             p.shadowFactor = 1.0
#             
#         self.addComObje( 'papi', papiObjeCol, shadow = False )
#         
        
        
        #
        #  AIRPLANE
        #
        vehicle = SimpleAirplane()
        vehicle.posMat[ 2, 3 ] = -5.0
        vehicle.color = [ 1.0, 0.2, 0.2, 1.0 ]
        self.addComObje( 'vehicle', vehicle )
        
        GLWidget.aircraftObje = vehicle



        print( 'createComObje ZZZ' )
      


 
class PilotViewWidget( MyGLWidget ) :

    def __init__( self, parent = None ) :
        print( '__init__ PilotViewWidget' )
        
        super().__init__( parent, self.setupFunc )
 
        self.parent = parent
        #self.setFocusPolicy( Qt.StrongFocus )

        self.ownAircraft = None
 
#         self.lookAtMat = calcLookAtMat_XYZEuler( [ -1000.0, -1000.0, -100.0 ],
#                                                  [     0.0,     0.0,   45.0 ] )
        
        self.lookAtMat      = np.identity( 4 )
        self.pilotLookAtMat = np.identity( 4 )
        
        self.hud = HUD( self )
        
        layout = QHBoxLayout()
        layout.setContentsMargins( 0, 0, 0, 0 )
        layout.addWidget( self.hud  )
        self.setLayout( layout )
    
        self.settingWindow = QMainWindow()
        self.settingWindow.setWindowTitle( 'Pilot View Settings' )
        self.settingWindow.resize( 500, 300 )
        self.settingWindowSliders = MySliders( [ 'X'  ,   0.0, -10.0,  10.0 ],
                                               [ 'Y'  ,   0.0, -10.0,  10.0 ],
                                               [ 'Z'  ,  -0.0, -10.0,  10.0 ],
                                               [ 'PHI',   0.0, -90.0,  90.0 ],
                                               [ 'THE',   0.0, -90.0,  90.0 ],
                                               [ 'PSI',   0.0, -180.0,  180.0 ],
                                               [ 'FOV',   0.1,   0.1,  90.0 ],
                                                    )
        self.settingWindowSliders.valueChanged.connect( self.settingWindowSliderChanged )
        self.settingWindow.setCentralWidget( self.settingWindowSliders )
    
        self.hud.keyPressed.connect( self.keyPressEvent )

        self.projFovy = 30.0
        self.update()

                
    def settingWindowSliderChanged( self ) :
        #print( 'settingWindowSliderChanged' )
        
        vals = self.settingWindowSliders.getValue()
        XYZ = vals[0:3]
        eul = np.radians( vals[3:6] )
        
        mat = np.identity( 4 )  
        mat[ 0:3, 0:3 ] = euler2dcm( eul )
        mat[ 0:3, 3   ] = - np.dot( mat[ 0:3, 0:3 ], XYZ )
        self.pilotLookAtMat = mat
        
        self.projFovy = vals[6]
        
        self.update()
        

    def keyPressEvent( self, event ) :
        print( 'keyPressEvent' )
        if event.key() == Qt.Key.Key_F1 :
            dcm = self.pilotLookAtMat[ 0:3, 0:3 ]
            xyz = self.pilotLookAtMat[ 0:3, 3   ]
            eul = np.array( dcm2euler( dcm ) )
            XYZ = - np.dot( dcm.T, xyz )
            values = [ *XYZ, *eul, self.projFovy ]
            self.settingWindowSliders.setValue( values )
            self.settingWindow.show()

    def setAircrafts( self, aircrafts ) :
        print( 'setAircrafts PilotViewWidget' )
        for ac in aircrafts :
            if ac is None : continue
            if ac.category == 'own' :
                self.ownAircraft = ac
                self.hud.setAircraft( ac )

    def setAirport( self, airport ) :
        print( 'setAirport PilotViewWidget' )
        self.hud.setMagVariation_deg(airport[ 'MagVariation_deg' ] )


    def update( self ) :
        #print( 'update PilotViewWidget' )

        if self.ownAircraft is not None :
#             XYZ = self.ownAircraft.XYZ       
#             eul_deg = np.degrees( self.ownAircraft.eul )
#             self.lookAtMat = np.dot( self.pilotLookAtMat,
#                                      calcLookAtMat_XYZEuler( XYZ, eul_deg ) )
            
            m = np.array( [ [  0.0,  1.0,  0.0,  0.0 ],
                            [  0.0,  0.0, -1.0,  0.0 ],
                            [ -1.0,  0.0,  0.0,  0.0 ],
                            [  0.0,  0.0,  0.0,  1.0 ] ] )
                    
            XYZ = self.ownAircraft.XYZ            
            xyz = self.ownAircraft.xyz  
            dcm = self.ownAircraft.dcm  
            dxyzPilot = self.ownAircraft.xyzPilot - self.ownAircraft.xyzCG
        
            vehicleLkAtMat = np.identity( 4 )  
            vehicleLkAtMat[ 0:3, 0:3 ] = dcm
            vehicleLkAtMat[ 0:3, 3   ] = - ( xyz + dxyzPilot )
            
            self.lookAtMat = np.dot( m, np.dot( self.pilotLookAtMat, vehicleLkAtMat ) )
                                      
            if hasattr( self, 'papi1') :
                
                posMatAirport = GLWidget.comObjes[ 'field' ].posMat
                XYZ = np.dot( posMatAirport[ 0:3, 0:3 ].T, ( XYZ - posMatAirport[ 0:3, 3 ]  ) )
                
                if XYZ[0] < 0.0 :
                    tanAng = - XYZ[2] / ( - self.x_TDPoint - XYZ[0] )
                    for p in self.papi1 :
                        p.color = [ 2.0, 0.0, 0.0, 1.0 ]
                    if tanAng > 0.043661 : self.papi1[0].color = [ 2.0, 2.0, 2.0, 1.0 ]
                    if tanAng > 0.049486 : self.papi1[1].color = [ 2.0, 2.0, 2.0, 1.0 ]
                    if tanAng > 0.055313 : self.papi1[2].color = [ 2.0, 2.0, 2.0, 1.0 ]
                    if tanAng > 0.061162 : self.papi1[3].color = [ 2.0, 2.0, 2.0, 1.0 ]
                    # tan 3.500 = 0.061162   
                    # tan 3.166 = 0.055313  
                    # tan 2.833 = 0.049486   
                    # tan 2.500 = 0.043661

        self.hud.update()

        super().update()
        

    def setupFunc( self ) :
        print( 'setupFunc PilotViewWidget' )
        
        
        
        self.addDrawingComObje( 'sky' )
        
        #if 'field' in GLWidget.comObjes :
        self.addDrawingComObje( 'field' )
#         self.addDrawingComObje( 'papi' )
        self.addDrawingComObje( 'vehicle' )
          
 
class SubWindow( QMainWindow ) :
 
    def __init__( self, parent = None ) :
        print( '__init__ SubWindow' )
        
        super().__init__( parent )
 
        self.parent = parent
 
        self.setFocusPolicy( Qt.FocusPolicy.StrongFocus )
 
        self.setGeometry( 900, 50, 500, 500 )
        self.setWindowTitle("Sub Window")

        self.glWidget = MyGLWidget( self, self.setupFunc )
        self.setCentralWidget( self.glWidget )
         
        self.glWidget.preProcess = self.preProcess
        
        eye    = [ -100.0, -100.0, - 100.0 ]
        target = [    0.0,    0.0,     0.0 ]    
        upper  = [    0.0,    0.0,    -1.0 ]
        self.glWidget.lookAtMat = calcLookAtMat( eye, target, upper )
    
        
        self.settingWindow = QMainWindow()
        self.settingWindow.setWindowTitle( 'View Settings' )
        self.settingWindow.resize( 500, 200 )
        self.settingWindowSliders = MySliders( [ 'DIST FT' ,  500.0,    0.0,  1000.0 ],
                                               [ 'DIR  DEG', -150.0,  -200.0,  200.0 ],
                                               [ 'ALT  FT' ,  500.0, -1000.0, 1000.0 ],
                                               [ 'FOV  DEG',    0.1,     0.1,   90.0 ],)
        self.settingWindowDXYZ = [ -100.0, -100.0, -100.0 ]
        
        self.settingWindowSliders.valueChanged.connect( self.settingWindowSliderChanged )
        self.settingWindow.setCentralWidget( self.settingWindowSliders )

        self.glWidget.projFovy = 45.0
        self.glWidget.update()

        #self.update()
                
    def settingWindowSliderChanged( self ) :
        print( 'settingWindowSliderChanged' )
        vals = self.settingWindowSliders.getValue()
        RR   =   vals[0] * 0.3048
        ang  = np.radians( vals[1] )
        
        dX, dY = RR * np.cos( ang ), RR * np.sin( ang )
        dZ     = - vals[2] * 0.3048
        
        self.settingWindowDXYZ = [ dX, dY, dZ ]
        self.glWidget.projFovy = vals[3]
        
        self.update()



    def keyPressEvent( self, event ) :
        print( 'keyPressEvent SubWindow' )
        if event.key() == Qt.Key.Key_F1 :
            
            target = GLWidget.comObjes[ 'vehicle' ].posMat[ 0:3, 3 ]
            
            dcm = self.glWidget.lookAtMat[ 0:3, 0:3 ]
            xyz = self.glWidget.lookAtMat[ 0:3, 3   ]
            eul = np.array( dcm2euler( dcm ) )
            XYZ = - np.dot( dcm.T, xyz )
            
            dXYZ = ( XYZ - target ) / 0.3048
            RR = np.sqrt( dXYZ[0] * dXYZ[0] + dXYZ[1] * dXYZ[1] )
            ang = np.degrees( np.arctan2( dXYZ[1], dXYZ[0] ) )
            
            fovy = self.glWidget.projFovy
            
            self.settingWindowSliders.setValue( [ RR, ang, - dXYZ[2], fovy ] )
            self.settingWindow.show()


    def setupFunc( self ) :
        print( 'setupFunc SubWindow' )
        self.glWidget.addDrawingComObje( 'sky' )
        #if 'field' in GLWidget.comObjes :
        self.glWidget.addDrawingComObje( 'field' )
#         self.glWidget.addDrawingComObje( 'papi' )
        self.glWidget.addDrawingComObje( 'vehicle' )
  
    def preProcess( self ) :
        #print( 'preProcess SubWindow' )
        pass
    
    def update( self ) :
        #print( 'update SubWindow' )
        target = GLWidget.comObjes[ 'vehicle' ].posMat[ 0:3, 3 ]
        eye    = target + self.settingWindowDXYZ
        upper  = [ 0.0, 0.0, -1.0 ]
        self.glWidget.lookAtMat = calcLookAtMat( eye, target, upper )
        
        self.glWidget.update()
    
#         buf = cv2.flip( self.glWidget.getPixel(), 0 )
#         #print( len( buf ) )
#         cv2.imwrite( 'aaa.png', buf )
 
 
class MainWindow( QMainWindow ) :

    def __init__( self, parent = None ) :
        
        super().__init__( parent )
        print( 'MainWindow init' )
    
        self.myId = 'MainWindow'
        
        self.setGeometry( 50, 50, 1400, 700 )
        self.setWindowTitle("Main Window")

        self.curTime = 0.0
        self.goFlag = False

        MyGLWidget.airport = AIRPORTS[ 'DEFAULT' ]
        #MyGLWidget.airport = AIRPORTS[ 'NAGOYA' ]


        JSBAircraft.set_LAT0_deg( 35.0 + 15.0 / 60.0 + 18.0 / 3600.0 )
        JSBAircraft.set_LON0_deg( 136.0 + 55.0 / 60.0 + 28.0 / 3600.0 )
        JSBAircraft.set_ALT0_m  ( 14.0 )
         
        filePath = os.path.dirname( __file__ )
        print( 'MainWindow init filePath : ', filePath )
        
        JSBAircraft.set_RootDir     ( filePath       )
        JSBAircraft.set_AircraftPath( r'..\aircraft' )
        JSBAircraft.set_EnginePath  ( r'..\engine'   )
        JSBAircraft.set_SystemsPath ( r'..\systems'  )

        self.filter0 = FirstOrderLagFilter()
        self.filter1 = FirstOrderLagFilter()
        self.filter2 = FirstOrderLagFilter()

        self.ownAircraft = None
        self.aircrafts = [ self.ownAircraft ] 
   
        self.pfd = PfdWidget()
        self.pfd.setAircrafts( self.aircrafts )

        self.controler = Controler()  
        self.controler.panelOperation.connect( self.controlerPanelOperation )        
        self.controler.deviceButtonPushed.connect( self.controlerDeviceButtonPushed )        

        self.pilotViewWidget = PilotViewWidget( self )        
        self.pilotViewWidget.setAircrafts( self.aircrafts )
 
        self.subWindow = SubWindow( self )
        #self.SubWindow.show()

        mainTabWidget = QTabWidget()
        
        self.setCentralWidget( mainTabWidget )


        #
        #   Simulation Widget
        #
        simulationWidget = QWidget()
        
        mainTabWidget.addTab( simulationWidget, 'simulation' )
        
        simMainLO = QHBoxLayout()
        simulationWidget.setLayout( simMainLO )
        simulationWidget.layout().setContentsMargins( 0, 0, 0, 0 )

        leftLayout   = QVBoxLayout()
        centerLayout = QGridLayout()
        rightLayout  = QVBoxLayout()

        simMainLO.addLayout( leftLayout )
        simMainLO.addLayout( centerLayout )
        simMainLO.addLayout( rightLayout )
        
        #
        #   LEFT LAYOUT
        #
        leftLW = VBoxLayoutWidget()
        leftLayout.addWidget( leftLW )
        
        btnsLW = leftLW.addWidget( GridLayoutWidget() )
        btnsLW.setSizePolicy( QSizePolicy.Policy.Preferred,
                              QSizePolicy.Policy.Maximum   )
         
        self.startBtn = btnsLW.addWidget( QPushButton( 'START' ), 0, 0 )
        self.resetBtn = btnsLW.addWidget( QPushButton( 'RESET' ), 0, 1 )
        self.startBtn.clicked.connect( self.startBtnClicked )
        self.resetBtn.clicked.connect( self.resetBtnClicked )     
        
        self.selectAircraftCmbbx = leftLW.addWidget( QComboBox() )
        self.selectAircraftCmbbx.currentIndexChanged.connect( self.setAircraftCmbbx )
        
#         self.selectAircraftCmbbx.addItem( 'DIYAircraft_F15'      )
#         self.selectAircraftCmbbx.addItem( 'DIYAircraft_B747'     )
        self.selectAircraftCmbbx.addItem( 'JSBAircraft_B747'     )
        self.selectAircraftCmbbx.addItem( 'JSBAircraft_F15'      )
        self.selectAircraftCmbbx.addItem( 'JSBAircraft_F16'      )
        self.selectAircraftCmbbx.addItem( 'JSBAircraft_F22'      )
        self.selectAircraftCmbbx.addItem( 'JSBAircraft_F35'      )
        self.selectAircraftCmbbx.addItem( 'JSBAircraft_t6texan2' )
        self.selectAircraftCmbbx.addItem( 'JSBAircraft_c172p' )
   
        
        self.selectAirportCmbbx = leftLW.addWidget( QComboBox() )
        for airport in AIRPORTS :
            self.selectAirportCmbbx.addItem( airport ) 
        self.selectAirportCmbbx.currentIndexChanged.connect( self.setAirportCmbbx )
 
 
        self.selectSettingCmbbx = leftLW.addWidget( QComboBox() )
        for k in INITIAL_VALUE_SETTINGS :
            self.selectSettingCmbbx.addItem( k ) 
        self.selectSettingCmbbx.currentIndexChanged.connect( self.setSettingCmbbx )
 

 
        settigsLW = leftLW.addWidget( VBoxLayoutWidget() )
        self.settingSpins = [ MySpinBox( 'Vc KT'  ,  '200',  '10' ),
                              MySpinBox( 'ALT FT' , '1000',  '10' ),
                              #MySpinBox( 'BRG DEG',    '0',   '1' ),
                              #MySpinBox( 'DIST NM',  '0.0', '0.1' ),
                              MySpinBox( 'X NM',     '0.000', '0.1' ),
                              MySpinBox( 'Y NM',     '0.000', '0.1' ),
                              MySpinBox( 'GAM DEG', '-3.0', '0.5' ),
                              MySpinBox( 'PSI DEG',    '0',   '5' ) ]
        for spin in self.settingSpins :
            settigsLW.addWidget( spin )

        devBtns = leftLW.addWidget( HBoxLayoutWidget() )        
        txts = [ ( 'FLAP', [ 'UP'   , 'MID'  , 'DN' ] ),
                 ( 'S.B.', [ 'CLOSE', 'OPEN'        ] ),
                 ( 'GEAR', [ 'UP'   , 'DN'          ] ) ]
        self.devSettingBtnGrps = []
        for txt in txts :
            grpBox = devBtns.addWidget( QGroupBox( txt[0] ) )
            lo = QVBoxLayout()
            grpBox.setLayout( lo )
            self.devSettingBtnGrps.append( QButtonGroup() )
            for k, t in enumerate( txt[1] ) :
                btn = QRadioButton( t )
                self.devSettingBtnGrps[-1].addButton( btn, k )
                lo.addWidget( btn )
                
        self.setSettingCmbbx()

        self.statusLbl = leftLW.addWidget( QLabel( '' ) )

        leftLW.addWidget( self.subWindow )
  
       
        #
        #   CENTER LAYOUT
        #
        centerLayout.addWidget( self.pilotViewWidget, 0, 0 )                
        self.pilotViewWidget.setSizePolicy( QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding )
        centerLayout.addWidget( self.pfd, 1, 0 )
        centerLayout.layout().setContentsMargins( 0, 0, 0, 0 )
        
        centerLayout.setRowStretch( 0, 2 )
        centerLayout.setRowStretch( 1, 1 )
        
        #
        #   RIGHT LAYOUT
        #
        rightLayout.addWidget( self.controler )
        
        simMainLO.setStretch( 0, 1 )
        simMainLO.setStretch( 1, 3 )
        simMainLO.setStretch( 2, 1 )
        
        
        graphsWidget = VBoxLayoutWidget()
        mainTabWidget.addTab( graphsWidget, 'graph')

        self.graph = graphsWidget.addWidget( ManyTHGraphs( self, 0.0, 5.0,
                      [ [  0, 0, ( 'ALT_ft', 'VC_kt>' ), ( -10.0, 50.0 , 0.0, 500.0 ) ],
                        [  1, 0, ( 'U'  , 'V'  , 'W'   ), ( 0.0, 100.0 ) ],
                        [  2, 0, ( 'p'  , 'q'  , 'r'   ), ( 0.0, 100.0 ) ],
                        
                        [  0, 1, ( 'XXX', 'YYY', 'ZZZ' ), ( -1.0, 1.0 ) ],
                        [  1, 1, ( 'XXX', 'YYY', 'ZZZ' ), ( -1.0, 1.0 ) ],
                        [  2, 1, ( 'XXX', 'YYY', 'ZZZ' ), ( -10.0, 10.0 ) ],
                        
                        ] ) )

        self.waitUntilAllDone( self.allDone ) 
        
        print( 'MainWindow init ZZZ' )
  

    def waitUntilAllDone( self, callback ) :
        print( 'waitUntilAllDone' )
        
        def func() :
            #print( 'func' )
            if GLWidget._COUNT1 == GLWidget._COUNT2 :
                self.wTimer.stop()
                callback()
                
        self.wTimer = QTimer()
        self.wTimer.setInterval( 100 )
        self.wTimer.timeout.connect( func )
        self.wTimer.start() 
        
        
    def allDone( self ) :
        print( 'allDone' )
        
        self.setAirportCmbbx()
        
        #self.controler.update()
        self.reset()


    def ownAircraftSignalEmitted( self, msg ) :
        print( 'ownAircraftSignalEmitted', msg )
        self.statusLbl.setText( msg )



    def setAircraft( self, aircraft ) :
        print( 'setAircraft' )
 
        self.ownAircraft = eval( aircraft + '()' ) 
     
        self.ownAircraft.name = aircraft
        self.ownAircraft.category = 'own'
        self.ownAircraft.signal.connect( self.ownAircraftSignalEmitted )

        self.aircrafts = [ self.ownAircraft ]
        
        self.pfd.setAircrafts( self.aircrafts )
        self.pilotViewWidget.setAircrafts( self.aircrafts )
 
        self.controler.setAircraft( self.ownAircraft ) 


    def setAircraftCmbbx( self ) :
        print( 'setAircraftCmbbx' )
        txt = self.selectAircraftCmbbx.currentText()
        self.setAircraft( txt )



    def setAirport( self, airport ) :
        print( 'setAirport' )
        
        self.airport = AIRPORTS[ airport ]
        
        self.pilotViewWidget.setAirport( self.airport )
        
        MyGLWidget.airport = self.airport
        self.pilotViewWidget.mkFeildObject()
        self.pilotViewWidget.setupFunc()
        self.subWindow.setupFunc()
 
        self.pfd.setAirport( self.airport )
        self.setSettingCmbbx()
        
        #self.update()
        self.reset()
 
        
    def setAirportCmbbx( self ) :
        #print( 'setAirportCmbbx' )
        txt = self.selectAirportCmbbx.currentText()
        self.setAirport( txt )
        self.update()

    def setSettingCmbbx( self ) :
        #print( 'setSettingCmbbx' )
        curTxt = self.selectSettingCmbbx.currentText()
        settingVal = INITIAL_VALUE_SETTINGS[ curTxt ]
        
        if curTxt == 'Takeoff' :
            if hasattr( self, 'airport' ) :
                #print( 'Takeoff' )
                X = - self.airport[ 'RwySize_m' ][0] / 2.0 / 1852.0
                settingVal[ 2 ] = (  "{:.3f}".format( X ),  '0.001' )  # X NM
                #print( settingVal[ 2 ] )
                #print( X )
                
        for spn, val in zip( self.settingSpins, settingVal ) :
            spn.setValue( float( val[0] ) )
            spn.lineEdit.setText( val[1] )
        for grp, idx in zip( self.devSettingBtnGrps, settingVal[6] ) :
            grp.button( idx ).setChecked( True )
        
        
    def closeEvent( self, event ) :
        print( 'closeEvent' )
        self.subWindow.close()
        self.pilotViewWidget.settingWindow.close()
        self.subWindow.settingWindow.close()

    def controlerDeviceButtonPushed( self ) :
        print( 'controlerDeviceButtonPushed' )
        self.ownAircraft.DEVcmd = self.controler.DEVcmd

    def controlerPanelOperation( self ) :
        #print( 'controlerPanelOperation' )
 
        self.ownAircraft.stickPos = self.controler.stickPos
        self.ownAircraft.trimPos  = self.controler.trimPos

        T, B = self.controler.thrPos, self.controler.brkPos
        self.ownAircraft.thrbrkPos  = ( T, B )


    def update( self ) :
        #print( 'update' )
        
        XYZ = self.ownAircraft.XYZ
        dcm = euler2dcm( self.ownAircraft.eul )
        GLWidget.aircraftObje.posMat[ 0:3, 0:3 ] = dcm.T
        GLWidget.aircraftObje.posMat[ 0:3, 3   ] = XYZ

        self.pilotViewWidget.update()
        self.pfd.update()
        self.controler.update()
        self.subWindow.update()

       

    def reset( self ) :
        print( 'reset' )
        
        self.curTime = 0.0
        self.controler.reset()

        RwyTrueBRG_deg  = self.airport[ 'RwyTrueBRG_deg' ]
        RwyElevation_ft = self.airport[ 'RwyElevation_ft' ]


        Vc_kt   = self.settingSpins[0].value()
        alt_ft  = self.settingSpins[1].value() + RwyElevation_ft
        
        print( 'RwyElevation_ft : ', RwyElevation_ft, alt_ft )
        
               
        ang = np.radians( RwyTrueBRG_deg )
        c, s = np.cos( ang ), np.sin( ang )
        X = self.settingSpins[2].value() 
        Y = self.settingSpins[3].value()
        X_NM    = c * X - s * Y
        Y_NM    = s * X + c * Y
              
        gam_deg = self.settingSpins[4].value()
        psi_deg = self.settingSpins[5].value() + RwyTrueBRG_deg

        devCmd = [ grp.checkedId() for grp in self.devSettingBtnGrps ]
        devCmd[0] *= 0.5
        
        self.controler.DEVcmd = devCmd
        self.ownAircraft.DEVcmd = self.controler.DEVcmd

        self.ownAircraft.setGroundAlt_FT( self.airport[ 'RwyElevation_ft' ]  )
        self.ownAircraft.setIC( X_NM, Y_NM, alt_ft, psi_deg, Vc_kt, gam_deg )
        self.ownAircraft.reset()

        self.controler.stickPos = self.ownAircraft.stickPos
        self.controler.trimPos = self.ownAircraft.trimPos
        
        self.controler.thrbrkPos = self.ownAircraft.thrbrkPos

        T, B = self.ownAircraft.thrbrkPos
        self.controler.thrPos = T
        self.controler.brkPos = B
        
        self.controler.DEVpos = self.ownAircraft.DEVpos
        
        self.graph.clearData()

        self.update()
        print( 'resetBtnClicked ZZZ' )
        

    def startBtnClicked( self ) :
        print( 'startBtnClicked' )

        if self.startBtn.text() == 'START' :
            self.startBtn.setText( 'STOP' )

            self.iniTime = time.time() - self.curTime
            self.qtimer = QTimer()
            self.qtimer.setInterval( 10 )
            self.qtimer.timeout.connect( self.step )
            self.qtimer.start()                

        elif self.startBtn.text() == 'STOP' :
            self.startBtn.setText( 'START' )
            self.qtimer.stop()   


    def step( self ) :

        self.curTime = time.time() - self.iniTime
        #print( 'curTime  : ', self.curTime )
         
        
        xyz = self.controler.stickPos
#         self.ownAircraft.stickPos = np.array( [
#             self.filter0.val( self.curTime, xyz[0] * xyz[0] * xyz[0] * 0.5, ),
#             self.filter1.val( self.curTime, xyz[1] * 0.5 ),
#             self.filter2.val( self.curTime, xyz[2] )  ] )
#         
        #self.ownAircraft.stickPos = self.controler.stickPos
#         self.ownAircraft.stickPos = np.array( [
#             xyz[0] * abs( xyz[0] ) * 1.5,
#             #xyz[0] * xyz[0] * xyz[0] * 1.5,
#             xyz[1] * 0.5,
#             xyz[2]   ] )
#         

        self.ownAircraft.trimPos = self.controler.trimPos

        
        T, B = self.controler.thrPos, self.controler.brkPos
        self.ownAircraft.thrbrkPos = ( T, B )

        self.controler.DEVpos = self.ownAircraft.DEVpos
                
        self.ownAircraft.step( self.curTime )
        
        if self.ownAircraft.name == 'JSBAircraft_F22' :  
            self.graph.addData( [ self.curTime,
                                  self.ownAircraft.alt_sl_ft,
                                  self.ownAircraft.Vc_kt,
                                  
                                  self.ownAircraft.UVW[0],
                                  self.ownAircraft.UVW[1],
                                  self.ownAircraft.UVW[2],
                                  
                                  self.ownAircraft.pqr[0],
                                  self.ownAircraft.pqr[1],
                                  self.ownAircraft.pqr[2],
                                  
                                  self.ownAircraft.getPropVal( 'fcs/aileron-cmd-norm'  ),
                                  self.ownAircraft.getPropVal( 'fcs/elevator-cmd-norm' ),
                                  self.ownAircraft.getPropVal( 'fcs/rudder-cmd-norm'   ),
                                  
                                  self.ownAircraft.getPropVal( 'fcs/left-aileron-pos-rad'  ),
                                  self.ownAircraft.getPropVal( 'fcs/elevator-pos-rad' ),
                                  self.ownAircraft.getPropVal( 'fcs/rudder-pos-rad'   ),
                                  
                                  #self.ownAircraft.getPropVal( 'fcs/el-pitch-cmd'  ),
                                  #self.ownAircraft.getPropVal( 'fcs/stick-filter'  ),
                                  #self.ownAircraft.getPropVal( 'fcs/pitch-cmd-summer'  ),
                                  self.ownAircraft.getPropVal( 'fcs/pitch-rate-error'  ),
                                  0.0,
                                  0.0,
                                  
                                  ]  )
        
        self.update()


    def resetBtnClicked( self ) :
        print( 'resetBtnClicked' )
        self.reset()


    def btnAClicked( self ) :
        print( 'btnAClicked' )
        print( self.controler.value )

 
if __name__ == "__main__" :
    QApplication.setAttribute( Qt.ApplicationAttribute.AA_ShareOpenGLContexts )
    app = QApplication( sys.argv )
    mainwindow = MainWindow()
    mainwindow.show()
    app.exec()

    
