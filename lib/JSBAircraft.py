import os
import sys
import numpy as np

import jsbsim

from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
from PyQt6.QtOpenGL import *
from PyQt6.QtOpenGLWidgets import QOpenGLWidget

#from MyPyqt import *
from lib.MyFunc import *
from lib.DIYAircraft import *


class FirstOrderLagFilter :
    
    def __init__( self, tc = 0.5 ) :
        print( '__init__ FirstOrderLagFilter' )
            
        self.timeConst = tc
        self.curTime = -1.0
        self.curVal = 0.0
        
    def val( self, newTime, newVal ) :
        
        dt = newTime - self.curTime
        
        if dt < 0.0 or dt > 1.0 :
            v = newVal
        else :
            v = self.curVal + ( newVal - self.curVal ) / self.timeConst * dt
        
        self.curTime = newTime
        self.curVal = v
        
        return v
        



def calcRhoPress( h_m, dT = 0.0 ) :
    T0     =    288.15
    P0     = 101325.0
    R      =    287.05287
    grv    =      9.80665
    bet    =    - 0.0065
    Hptrop =  11000.0
    m = - grv / bet / R
    if h_m < Hptrop :
        T = T0 + dT + bet * h_m   
        P = P0 * ( ( T - dT ) / T0  ) ** m
    else :
        T = T0 + dT + bet * Hptrop
        Ptrop = P0 * ( ( T - dT ) / T0  ) ** m
        P = Ptrop * np.exp( - grv / R / ( T - dT ) * ( h_m - Hptrop ) )
    rho = P / R / T
    return rho, P

def calcVtas( Vcas, h_m, dT = 0.0 ) :
    rho0, P0 = 1.225, 101325.0
    rho , P  = calcRhoPress( h_m, dT )
    A0 = 7.0 * P0 / rho0 
    A1 = 7.0 * P  / rho
    PP = P0 * ( ( 1.0 + Vcas * Vcas / A0 ) ** 3.5 - 1.0 )
    Vtas = ( A1 * ( ( 1.0 + PP / P ) ** ( 1.0 / 3.5 ) - 1.0 ) ) ** 0.5
    return Vtas

def calcVcas( Vtas, h_m, dT = 0.0 ) :
    rho0, P0 = 1.225, 101325.0
    rho , P  = calcRhoPress( h_m, dT )
    A0 = 7.0 * P0 / rho0 
    A1 = 7.0 * P  / rho
    PP = P * ( ( 1.0 + Vtas * Vtas / A1 ) ** 3.5 - 1.0 ) 
    Vcas = ( A0 * ( ( 1.0 + PP / P0 ) ** ( 1.0 / 3.5 ) - 1.0 ) ) ** 0.5
    return Vcas


class PIDControler() :
    
    def __init__( self, Kp, Ki, Kd, t = 0.0, x = 0.0, v = 0.0 ) :
        self.Kp = Kp
        self.Ki = Ki if abs( Ki ) > 1.0e-8 else 1.0e-8 
        self.Kd = Kd
        self.reset( t, x, v )
        
    def reset( self, t = 0.0, x = 0.0, v = 0.0 ) :
        self.curTime = t
        self.x = x
        self.integX = v / self.Ki
        
    def calc( self, t, x ) :
        delTime = t - self.curTime
        self.integX += 0.5 * ( x + self.x ) * delTime
        vp = self.Kp * x
        vi = self.Ki * self.integX
        vd = self.Kd * ( x - self.x ) / delTime
        self.curTime = t
        self.x = x
        
        if self.name == 'pid_thr' :
            print( vp, vi, vd  )
        
        return vp + vi + vd
        
class I_PDControler() :
    
    def __init__( self, Kp, Ki, Kd ) :
        self.Kp = Kp
        self.Ki = Ki if abs( Ki ) > 1.0e-8 else 1.0e-8 
        self.Kd = Kd
        #self.reset( t, x, v )
        
    def reset( self, t = 0.0, r = 0.0, y = 0.0, v = 0.0 ) :
        self.curTime = t
        self.integX = ( v + self.Kp * y ) / self.Ki
        self.e = r - y
        self.y = y
        
    def calc( self, t, r, y ) :
        delTime = t - self.curTime
        e = r - y
        self.integX += 0.5 * ( e + self.e ) * delTime
        vi = self.Ki * self.integX
        vp = self.Kp * y
        vd = self.Kd * ( y - self.y ) / delTime
        self.curTime = t
        self.y = y
        self.e = e
#         if self.name == 'pid_thr' :
#             print( vp, vi, vd  )
        return vi - vp - vd
        

class PointMassAircraft() :

    def __init__( self ) :
        print( '__init__ PointMasaAircraft' )

        self.param = {}

        self.grav   = 9.80665
        self.Rearth = 6356766.0
        self.rho0   = 1.225
        
        self.mass = 10000.0
        self.Sref  =   300.0 * 0.3048 * 0.3048  # ft2 => m2

        self.CL0   = 0.1
        self.CLa   = np.degrees( 0.05 )
        self.CLmax = 2.0
        self.CD0   = 0.02
        self.CDK   = 0.2

        self.minAlp = ( - self.CLmax - self.CL0 ) / self.CLa 
        self.maxAlp = (   self.CLmax - self.CL0 ) / self.CLa 



        self.minThrust  =       0.0 * 4.44822   # lbs => N
        self.maxThrust  =   17800.0 * 4.44822   # lbs => N
        
        self.Kalp  = 1.0
        self.Kthr  = 1.0
        self.KPHI  = 1.0
        
        
        #self.pid_thr = PIDControler( 0.01, 0.001, 0.0 )
        self.pid_thr = I_PDControler( 0.1, 0.01, 0.0 )
        
        #self.pid_alp = PIDControler( 1.0, 0.001, 0.0 )
        self.pid_alp = I_PDControler( 2.0, 0.5, 0.0 )
        
        self.pid_thr.name = 'pid_thr'
        self.pid_alp.name = 'pid_alp'
        
        self.initXYZ = np.array( [ 0.0, 0.0, - 1000.0 * 0.3048 ] )
        self.initVt  = 300.0 * 0.514444
        self.initGam = np.radians(  0.0 )
        self.initPSI = np.radians(  0.0 )

        #self.alp = 0.0

        for k in range( 20 ) :
#             h = k * 1000.0
#             r, p = calcRhoPress( h )
#             print( h, r, p )

            h = k * 1000.0
            vt = calcVtas( 400 * 0.5144, h * 0.3048  ) / 0.5144
            print( h, vt )



    
    def calcRho( self, h ) :
        H = 6356.766 * h / ( self.Rearth + h ) ## km
        T = 15 - 6.5 * H
        p = np.power( 288.15 / ( T + 273.15 ), -5.256 )
        d = p * 288.15 /  ( T + 273.15 )
        return self.rho0 * d

    def calcWieght( self ) :
        #print( 'calcWieght PointMasaAircraft' )
        self.wieght = self.mass * self.grav
        self.minTbyW = self.minThrust / self.wieght
        self.maxTbyW = self.maxThrust / self.wieght
        

    def calcCommand( self ) :
        print( 'calcCommamd PointMasaAircraft' )

        self.Vt_target  = 400.0 * 0.514444
        self.ALT_target_ft = 2000.0
        
    
        delAlt_ft = self.ALT_target_ft - self.param[ 'ALT_ft'  ]
    
    
        self.gam_target = np.radians( min( delAlt_ft / 100.0, 10.0 ) )
        
        
        #self.gam_target = np.radians( 5.0 )

        
        #TbyWCom = self.pid_thr.calc( self.curTime, self.Vt_target - self.Vt )
        TbyWCom = self.pid_thr.calc( self.curTime, self.Vt_target, self.Vt )
        self.TbyWCom = max( min( TbyWCom, self.maxTbyW ), self.minTbyW )

        #alpCom  = self.pid_alp.calc( self.curTime, self.gam_target - self.gam )
        alpCom  = self.pid_alp.calc( self.curTime, self.gam_target, self.gam )
        self.alpCom = max( min( alpCom, self.maxAlp ), self.minAlp )

        print( 'alpCom  : ', np.degrees( self.alpCom ) )
        print( 'TbyWCom : ', self.TbyWCom )

#         self.minAlp = ( - self.CLmax - self.CL0 ) / self.CLa 
#         self.maxAlp = (   self.CLmax - self.CL0 ) / self.CLa 


        #self.alpCom  = self.alp
        #self.alpCom  = np.radians( 10.0 )
        #self.TbyWCom = self.TbyW
        self.PHICom  = self.PHI
        #self.PHICom  = np.radians( 30.0 )




    def reset( self, curTime = 0.0 ) :
        print( 'reset PointMasaAircraft' )

        self.curTime = curTime

        self.calcWieght()
        
        rho = self.calcRho( - self.initXYZ[2] )
        QS = 0.5 * rho * self.initVt * self.initVt * self.Sref

        cGam, sGam = np.cos( self.initGam ), np.sin( self.initGam )
        cPSI, sPSI = np.cos( self.initPSI ), np.sin( self.initPSI )

        alp = 0.0
        TbyW = sGam + QS * self.CD0 / self.wieght

        cnt = 0
        while cnt < 10 :
            cnt += 1
            
            CL = self.CL0 + self.CLa * alp
            CD = self.CD0 + self.CDK * CL *CL
            LbyW = QS * CL / self.wieght
            DbyW = QS * CD / self.wieght
            cAlp, sAlp = np.cos( alp ), np.sin( alp )

            X0 = - DbyW - sGam + TbyW * cAlp
            Y0 =   LbyW - cGam + TbyW * sAlp
            if abs( X0 ) < 1.0e-6 :
                if abs( Y0 ) < 1.0e-6 :
                    cnt = 0
                    break
            
            XA = - QS * 2.0 * CL * self.CLa / self.wieght - TbyW * sAlp
            YA =   QS * self.CLa / self.wieght - TbyW * cAlp
            XT = cAlp
            YT = sAlp

            D = XA * YT - XT * YA
            alp  -= (   YT * X0 - XT * Y0 ) / D
            TbyW -= ( - YA * X0 - XA * Y0 ) / D

#             print()
#             print( 'CL  ', CL )
#             print( 'CD  ', CD )
#             print( 'LbyW  ', LbyW )
#             print( 'DbyW  ', DbyW )
#             print()
#             print( alp, TbyW )
#             print( X0 , Y0 )
#             print( XA , XT )
#             print( YA , YT )


        if cnt > 0 :
            print( 'Not converged !!!' )

        self.XYZ  = self.initXYZ
        self.Vt   = self.initVt
        self.gam  = self.initGam
        self.PHI  = 0.0
        self.PSI  = self.initPSI

        self.alp  = alp
        self.TbyW = TbyW
        self.CL   = CL
        self.CD   = CD

        #self.pid_thr.reset( curTime, 0.0, self.TbyW  )
        self.pid_thr.reset( curTime, self.Vt, self.Vt, self.TbyW  )
        #self.pid_alp.reset( curTime, 0.0, self.alp )
        self.pid_alp.reset( curTime, self.gam, self.gam, self.alp )

        self.calc()


    def calc( self ) :
        #print( 'calc' )

        self.param[ 'XY_Nm'   ] = np.array( self.XYZ[ 0:2 ] ) / 1852.0 
        self.param[ 'ALT_ft'  ] = - self.XYZ[ 2 ] / 0.3048
        self.param[ 'Vt_kt'   ] = self.Vt / 0.514444

        self.param[ 'ALP_deg' ] = np.degrees( self.alp )
        self.param[ 'BET_deg' ] = 0.0

        self.param[ 'gam_deg' ] = np.degrees( self.gam )


        cGam, sGam = np.cos( self.gam ), np.sin( self.gam )
        cPSI, sPSI = np.cos( self.PSI ), np.sin( self.PSI )

        self.UVW = np.array( [   self.Vt * cGam * cPSI,
                                 self.Vt * cGam * sPSI,
                               - self.Vt * sGam        ]  )

        dcmW = euler2dcm( [ self.PHI, self.gam, self.PSI ] )

        ca, sa = np.cos( self.alp ), np.sin( self.alp )
        dcm = np.array( [ [ ca * dcmW[0][0] - sa * dcmW[2][0], dcmW[0][1], ca * dcmW[0][2] - sa * dcmW[2][2] ],
                          [      dcmW[1][0]                  , dcmW[1][1],      dcmW[1][2]                   ],
                          [ sa * dcmW[2][0] + ca * dcmW[2][0], dcmW[2][1], sa * dcmW[2][2] + ca * dcmW[2][2] ] ] )

        self.eul = np.array( dcm2euler( dcm ) )

        self.param[ 'eul_deg' ] = np.degrees( self.eul )

#         print( 'XY_Nm'  , self.param[ 'XY_Nm'   ]  )
#         print( 'ALT_ft' , self.param[ 'ALT_ft'  ]  )
#         print( 'Vt_kt'  , self.param[ 'Vt_kt'   ]  )
#         print( 'ALP_deg', self.param[ 'ALP_deg' ]  )


    def step( self, curTime ) :
        #print( 'step' )

        self.preTime = self.curTime
        self.curTime = curTime
        
        dTime = self.curTime - self.preTime
        
        self.calcWieght()
        self.calcCommand()
                

        CL = self.CL0 + self.CLa * self.alp
        self.CL = max( - self.CLmax, min( self.CLmax, CL ) )
        self.CD = self.CD0 + self.CDK * self.CL * self.CL
        
        rho = self.calcRho( - self.XYZ[2] )
        QS = 0.5 * rho * self.Vt * self.Vt * self.Sref
        self.LbyW = QS * self.CL / self.wieght
        self.DbyW = QS * self.CD / self.wieght

        cPHI, sPHI = np.cos( self.PHI ), np.sin( self.PHI )
        cGam, sGam = np.cos( self.gam ), np.sin( self.gam )
        cAlp, sAlp = np.cos( self.alp ), np.sin( self.alp )

        VtDot  = ( - self.DbyW        - sGam + self.TbyW * cAlp        ) * self.grav
        gamDot = (   self.LbyW * cPHI - cGam + self.TbyW * sAlp * cPHI ) * self.grav / self.Vt
        PSIDot = (   self.LbyW * sPHI        + self.TbyW * sAlp * sPHI ) * self.grav / self.Vt / cGam

        alpDot  = self.Kalp * ( self.alpCom  - self.alp )
        TbyWDot = self.Kthr * ( self.TbyWCom - self.TbyW )
        PHIDot  = self.KPHI * ( self.PHICom  - self.PHI )

        self.alp  += alpDot   * dTime
        self.TbyW += TbyWDot  * dTime
        self.PHI  += PHIDot   * dTime
        
        self.XYZ  += self.UVW * dTime    
        self.Vt   += VtDot    * dTime
        self.gam  += gamDot   * dTime
        self.PSI  += PSIDot   * dTime

        self.calc()



class JSBAircraft( AIRCRAFT ) :

#     signal = pyqtSignal( str )

    Rearth   = 6378136.59
    LAT0_deg =  35.0 + 15.0 / 60.0 + 18.0 / 3600.0
    LON0_deg = 136.0 + 55.0 / 60.0 + 28.0 / 3600.0
    ALT0_m   = 14.0

    RootDir      = None
    AircraftPath = None
    EnginePath   = None
    SystemsPath  = None

    def __init__( self, script = None ) :
        print( '__init__ JSBAircraft' )

        super().__init__()

        self.fdm = jsbsim.FGFDMExec( None )
        self.fdm.set_debug_level( 0 )
        
        self.loadScript( script )
        
#         self.fdm.set_root_dir     ( JSBAircraft.RootDir      )
#         self.fdm.set_aircraft_path( JSBAircraft.AircraftPath )
#         self.fdm.set_engine_path  ( JSBAircraft.EnginePath   )
#         self.fdm.set_systems_path ( JSBAircraft.SystemsPath  )
#                 
#         self.fdm.load_script( script )
        
#         for x in self.fdm.get_property_catalog() :
#             print( x )
        
#         self.fdm.set_aircraft_path( os.path.join( filePath, r'..\aircraft' ) )
#         self.fdm.set_engine_path  ( os.path.join( filePath, r'..\engine' ) )
#         #self.fdm.set_systems_path ( os.path.join( filePath, r'..\systems' ) )
#         self.fdm.load_script( 'myScript_F15.xml'  )
        
        
#         self.fdm.set_aircraft_path( os.path.join( filePath, r'..\aircraft' ) )
#         self.fdm.set_engine_path  ( os.path.join( filePath, r'..\engine' ) )
#         self.fdm.set_systems_path ( os.path.join( filePath, r'..\aircraft\f16\systems' ) )
#         self.fdm.load_script( 'myScript.xml'  )
        
#         self.lat0_rad  = np.radians( JSBAircraft.LAT0_deg )
#         self.lon0_rad  = np.radians( JSBAircraft.LON0_deg )
#         self.alt0_ft   = JSBAircraft.ALT0_m / 0.3048
#         
#         self.cosLat0 = np.cos( self.lat0_rad )
#         self.setPropVal( 'position/terrain-elevation-asl-ft', self.alt0_ft )
        
        self.numEngines = 1
        self.thst_ref = 0.001
        
        self.propCat_ThrustLbs = [ 'propulsion/engine/thrust-lbs' ]
        self.propCat_ThrotCmdNorm = [ 'fcs/throttle-cmd-norm' ]

        #         X_nm, Y_nm, Alt_ft, HDG_deg, Vc_kt, Gam_deg
        self.setIC( 0.0, 0.0, 10.0, 0.0, 0.0, 0.0 )
        self.fdm.run_ic()   
        self.calc()
        
        self.setPropVal( 'propulsion/set-running', -1 )
        
        print( '__init__ JSBAircraft ZZZ' )
        
        

    @classmethod
    def set_LAT0_deg( cls, LAT0_deg ) :
        JSBAircraft.LAT0_deg = LAT0_deg
        
    @classmethod
    def set_LON0_deg( cls, LON0_deg ) :
        JSBAircraft.LON0_deg = LON0_deg
        
    @classmethod
    def set_ALT0_m( cls, ALT0_m ) :
        JSBAircraft.ALT0_m = ALT0_m
        
    @classmethod
    def set_RootDir( cls, path ) :
        JSBAircraft.RootDir = path
        
    @classmethod
    def set_AircraftPath( cls, path ) :
        JSBAircraft.AircraftPath = path
        
    @classmethod
    def set_EnginePath( cls, path ) :
        JSBAircraft.EnginePath = path
        
    @classmethod
    def set_SystemsPath( cls, path ) :
        JSBAircraft.SystemsPath = path
         
         
    @property
    def numEngines( self ) :
        return self._numEngines
        
    @numEngines.setter
    def numEngines( self, value ) :
        #print( 'numEngines setter' )
        self._numEngines = value
        self.propCat_ThrustLbs = []
        self.propCat_ThrotCmdNorm = []
        for k in range( value ) :
            if k == 0 :
                self.propCat_ThrustLbs   .append( 'propulsion/engine/thrust-lbs' )
                self.propCat_ThrotCmdNorm.append( 'fcs/throttle-cmd-norm' )
            else :
                num = '[' + str( k ) + ']'
                self.propCat_ThrustLbs   .append( 'propulsion/engine' + num + '/thrust-lbs' )
                self.propCat_ThrotCmdNorm.append( 'fcs/throttle-cmd-norm' + num  )
        #print( 'numEngines setter ZZZ' )
         
        print( self.propCat_ThrustLbs )
        print( self.propCat_ThrotCmdNorm )

    @property
    def stickPos( self ) :
        X = self.getPropVal( 'fcs/aileron-cmd-norm'  )
        Y = self.getPropVal( 'fcs/elevator-cmd-norm' )
        Z = self.getPropVal( 'fcs/rudder-cmd-norm'   )
        return np.array( [ X, Y, Z ] )
        
    @stickPos.setter
    def stickPos( self, value ) :
        X, Y, Z = value
        self.setPropVal( 'fcs/aileron-cmd-norm' , X )
        self.setPropVal( 'fcs/elevator-cmd-norm', Y )
        self.setPropVal( 'fcs/rudder-cmd-norm'  , Z )
        
    @property
    def trimPos( self ) :
        X = self.getPropVal( 'fcs/roll-trim-cmd-norm'  )
        Y = self.getPropVal( 'fcs/pitch-trim-cmd-norm' )
        Z = self.getPropVal( 'fcs/yaw-trim-cmd-norm'   )
        return np.array( [ X, Y, Z ] )
        
    @trimPos.setter
    def trimPos( self, value ) :
        X, Y, Z = value
        self.setPropVal( 'fcs/roll-trim-cmd-norm' , X )
        self.setPropVal( 'fcs/pitch-trim-cmd-norm', Y )
        self.setPropVal( 'fcs/yaw-trim-cmd-norm'  , Z )
        
    @property
    def thrbrkPos( self ) :
        
        THS = [ self.getPropVal( cat )
                for cat in self.propCat_ThrotCmdNorm ]
        
        #print( THS )
        
        TH = np.average( THS )
    
        B1 = self.getPropVal( 'fcs/left-brake-cmd-norm' )
        B2 = self.getPropVal( 'fcs/right-brake-cmd-norm' )
        B3 = self.getPropVal( 'fcs/center-brake-cmd-norm' )
        
        return np.array( [ TH, ( B1 + B2 + B3 ) / 3.0 ] )

        
    @thrbrkPos.setter
    def thrbrkPos( self, value ) :
        TH, BB = value
        #print( '@thrbrkPos.setter', TH, BB )
        for cat in self.propCat_ThrotCmdNorm :
            self.setPropVal( cat, TH )
            
        self.setPropVal( 'fcs/left-brake-cmd-norm'  , BB)
        self.setPropVal( 'fcs/right-brake-cmd-norm' , BB)
        self.setPropVal( 'fcs/center-brake-cmd-norm', BB)
        
    @property
    def DEVcmd( self ) :
        FL = self.getPropVal( 'fcs/flap-cmd-norm' )
        SB = self.getPropVal( 'fcs/speedbrake-cmd-norm' )
        GR = self.getPropVal( 'gear/gear-cmd-norm' )
        return np.array( [ FL, SB, GR ] )
        
    @DEVcmd.setter
    def DEVcmd( self, value ) :
        print( 'DEVcmd setter MyAircraft' )
        FL, SB, GR = value
        print( FL, SB, GR )
        self.setPropVal( 'fcs/flap-cmd-norm', FL       )
        self.setPropVal( 'fcs/speedbrake-cmd-norm', SB )
        self.setPropVal( 'gear/gear-cmd-norm', GR      )
        
    @property
    def DEVpos( self ) :
        FL = self.getPropVal( 'fcs/flap-pos-norm' )
        SB = self.getPropVal( 'fcs/speedbrake-pos-norm' )
        GR = self.getPropVal( 'gear/gear-pos-norm' )
        return np.array( [ FL, SB, GR ] )

        
    def setGroundAlt_FT( self, alt ) :
        print( 'setIC setGroundAlt_FT' )
        #self.setPropVal( 'position/terrain-elevation-asl-ft', alt )
        JSBAircraft.ALT0_m = alt * 0.3048
            
    
        
    def getPropVal( self, props ) :
        if type( props ) == list :
            #ret = []
            #for p in props :
            #    ret.append( self.fdm.get_property_value( p ) )
            #   
            ret = np.array( [ self.fdm.get_property_value( p ) for p in props ] )    
            return ret
        else :
            return self.fdm.get_property_value( props )
    
    def setPropVal( self, props, vals ) :
        if type( props ) == list :
            for p, v in zip( props, vals )  :
                self.fdm.set_property_value( p, v )
        else :
            self.fdm.set_property_value( props, vals )


    def calc( self ) :
        #print( 'calc MyAircraft' )
        
        Rearth = JSBAircraft.Rearth 
        ALT0   = JSBAircraft.ALT0_m
        lat0   = np.radians( JSBAircraft.LAT0_deg )
        lon0   = np.radians( JSBAircraft.LON0_deg )       
        cosLat0 = np.cos( lat0 )
        
        self.lat  = self.getPropVal( 'position/lat-gc-rad'  )
        self.lon  = self.getPropVal( 'position/long-gc-rad' )
        
        self.alt_sl    = self.getPropVal( 'position/h-sl-meters' )
        self.alt_sl_ft = self.getPropVal( 'position/h-sl-ft' )
        
        self.XYZ = np.array( [   ( self.lat    - lat0 ) * Rearth,
                                 ( self.lon    - lon0 ) * Rearth * cosLat0,
                               -   self.alt_sl        ] )
                               #- ( self.alt_sl - ALT0 )   ] )
        
        self.eul     = self.getPropVal( [ 'attitude/phi-rad'      , 'attitude/theta-rad'    , 'attitude/psi-rad'      ] )
        self.eul_deg = self.getPropVal( [ 'attitude/phi-deg'      , 'attitude/theta-deg'    , 'attitude/psi-deg'      ] )
        self.uvw_fps = self.getPropVal( [ 'velocities/u-fps'       , 'velocities/v-fps'      , 'velocities/w-fps'      ] )
        self.UVW_fps = self.getPropVal( [ 'velocities/v-north-fps' , 'velocities/v-east-fps' , 'velocities/v-down-fps' ] )
        self.pqr     = self.getPropVal( [ 'velocities/p-rad_sec'   , 'velocities/q-rad_sec'  , 'velocities/r-rad_sec'  ] )
        self.abg     = self.getPropVal( [ 'aero/alpha-rad'         , 'aero/beta-rad'         , 'flight-path/gamma-rad' ] )
        self.abg_deg = self.getPropVal( [ 'aero/alpha-deg'         , 'aero/beta-deg'         , 'flight-path/gamma-deg' ] )
        
        self.alp    = self.getPropVal( 'aero/alpha-rad'        )
        self.bet    = self.getPropVal( 'aero/beta-rad'         )  
        self.gam    = self.getPropVal( 'flight-path/gamma-rad' )
        self.alpDot = self.getPropVal( 'velocities/vtrue-kts'  )
        
        self.Vt_kt   = self.getPropVal(   'velocities/vtrue-kts' )
        self.Vc_kt   = self.getPropVal(   'velocities/vc-kts'    )
        self.mach    = self.getPropVal(   'velocities/mach'      )

        self.Vt  = self.Vt_kt   * 0.5144 
        self.uvw = self.uvw_fps * 0.3048
        self.UVW = self.UVW_fps * 0.3048
   
        self.dcm = euler2dcm( self.eul )
        self.dcmt = np.transpose( self.dcm )
 
        self.xyz = np.dot( self.dcm, self.XYZ )
   
        self.xyzCG    = self.getPropVal( [ 'inertia/cg-x-in'       , 'inertia/cg-y-in'       , 'inertia/cg-z-in'       ] ) * ( - 0.0254 )
        self.xyzPilot = self.getPropVal( [ 'metrics/eyepoint-x-in' , 'metrics/eyepoint-y-in' , 'metrics/eyepoint-z-in' ] ) * ( - 0.0254 )
   
        self.thst_lbs = 0.0
        for cat in self.propCat_ThrustLbs :
            self.thst_lbs += self.getPropVal( cat )
            
        self.thst = self.thst_lbs * 0.4536 * 9.80665    
            
        self.uvwDot_fps2  = self.getPropVal( [ 'accelerations/udot-ft_sec2', 'accelerations/vdot-ft_sec2', 'accelerations/wdot-ft_sec2' ] ) 
        #self.uvwDot_fps2  = self.getPropVal( [ 'accelerations/uidot-ft_sec2', 'accelerations/vidot-ft_sec2', 'accelerations/widot-ft_sec2' ] ) 
        self.Nxyz         = self.getPropVal( [ 'accelerations/Nx', 'accelerations/Ny', 'accelerations/Nz' ] ) 
            
        #self.uvwDot_fps2  = self.getPropVal( [ 'accelerations/a-pilot-x-ft_sec2', 'accelerations/a-pilot-y-ft_sec2', 'accelerations/a-pilot-z-ft_sec2' ] ) 
        self.pilotNxyz    = self.getPropVal( [ 'accelerations/n-pilot-x-norm', 'accelerations/n-pilot-y-norm', 'accelerations/n-pilot-z-norm' ] ) 
            
            
        #print( 'fcs/elevator-pos-deg  : ',  self.getPropVal( 'fcs/elevator-pos-deg') )            
        #print( self.Nxyz )            
        #print( self.uvwDot_fps2 / self.Nxyz )            
            


    def setIC( self, X_nm, Y_nm, Alt_ft, HDG_deg, Vc_kt, Gam_deg ) :
        print( 'setIC MyAircraft' )
        
        lat0  = np.radians( JSBAircraft.LAT0_deg )
        lon0  = np.radians( JSBAircraft.LON0_deg )     
        cosLat0 = np.cos( lat0 )
        
        self.iniLat_rad = lat0 + X_nm * 1852.0 / JSBAircraft.Rearth
        self.iniLon_rad = lon0 + Y_nm * 1852.0 / JSBAircraft.Rearth / cosLat0
        
        #JSBAircraft.ALT0_m
        #self.iniAlt_ft  = JSBAircraft.ALT0_m / 0.3048 + Alt_ft
        self.iniAlt_ft  = Alt_ft
                
        self.setPropVal( 'ic/lat-gc-rad'  , self.iniLat_rad )
        self.setPropVal( 'ic/long-gc-rad' , self.iniLon_rad )
        self.setPropVal( 'ic/h-sl-ft'     , self.iniAlt_ft  )
        self.setPropVal( 'ic/phi-deg'     , 0.0             )
        self.setPropVal( 'ic/psi-true-deg', HDG_deg         )
        self.setPropVal( 'ic/vc-kts'      , Vc_kt           )
        self.setPropVal( 'ic/v-fps'       , 0.0             )
        self.setPropVal( 'ic/gamma-deg'   , Gam_deg         )
        self.setPropVal( 'ic/p-rad_sec'   , 0.0             )
        self.setPropVal( 'ic/q-rad_sec'   , 0.0             )
        self.setPropVal( 'ic/r-rad_sec'   , 0.0             )


    def reset( self ) :
        print( 'reset JSBAircraft' )
        
        self.fdm.set_sim_time( 0.0 )
        self.simTime = self.getPropVal(   'simulation/sim-time-sec'  )
        self.setPropVal( 'position/terrain-elevation-asl-ft', JSBAircraft.ALT0_m / 0.3048 )
        
        self.fdm.run_ic()
#         try :
#             self.fdm.do_trim( 1 )
#             print( 'trim O.K. !!' )
#             self.signal.emit( 'trim O.K. !!' )
#         except :
#             print( 'trim failed !!' )
#             self.signal.emit( 'trim failed !!' )
        
        self.calc()
        
        
# attitude/phi-rad (R)
# attitude/phi-deg (R)
# attitude/roll-rad (R)
# 
# attitude/theta-rad (R)
# attitude/theta-deg (R)
# attitude/pitch-rad (R)
# 
# attitude/psi-rad (R)
# attitude/psi-deg (R)
# attitude/heading-true-rad (R)
        
        
        
#         print()
#         print( 'attitude/psi-deg           : ',             self.getPropVal( 'attitude/psi-deg'   ) )
#         print( 'attitude/heading-true-rad  : ',             self.getPropVal( 'attitude/heading-true-rad'   ) )
#         print( '                           : ', np.degrees( self.getPropVal( 'attitude/heading-true-rad'   ) ) )
        
#         print()
#         print( 'eul_deg  : ', self.eul_deg )
#         print( 'Vc_kt    : ', self.Vc_kt )
#         print( 'abg_deg  : ', self.abg_deg )
#         print()
#         print( 'fcs/elevator-cmd-norm   : ', self.getPropVal( 'fcs/elevator-cmd-norm'   ) )
#         print( 'fcs/elevator-pos-norm   : ', self.getPropVal( 'fcs/elevator-pos-norm'   ) )
#         print( 'fcs/elevator-pos-deg    : ', self.getPropVal( 'fcs/elevator-pos-deg'    ) )
#         print( 'fcs/pitch-trim-cmd-norm : ', self.getPropVal( 'fcs/pitch-trim-cmd-norm' ) )
#         print()
#         print( 'fcs/flap-pos-norm : ', self.getPropVal( 'fcs/flap-pos-norm' ) )
#         print( 'fcs/speedbrake-pos-norm : ', self.getPropVal( 'fcs/speedbrake-pos-norm' ) )
#         print( 'gear/gear-pos-norm : ', self.getPropVal( 'gear/gear-pos-norm' ) )
#         print()
#         print( 'inertia/weight-lbs        : ', self.getPropVal( 'inertia/weight-lbs'       ) )
#         print( 'inertia/cg-x-in           : ', self.getPropVal( 'inertia/cg-x-in'          ) )
#         print( 'inertia/cg-y-in           : ', self.getPropVal( 'inertia/cg-y-in'          ) )
#         print( 'inertia/cg-z-in           : ', self.getPropVal( 'inertia/cg-z-in'          ) )
#         print( 'velocities/vtrue-kts      : ', self.getPropVal( 'velocities/vtrue-kts'     ) )
#         print( 'velocities/mach           : ', self.getPropVal( 'velocities/mach'          ) )
#         print( 'atmosphere/rho-slugs_ft3  : ', self.getPropVal( 'atmosphere/rho-slugs_ft3' ) )
#         print()
#         
#  
#         print( 'aero/qbar-psf             : ', self.getPropVal( 'aero/qbar-psf' ) )
#         
#         print( 'forces/fbx-total-lbs      : ', self.getPropVal( [ 'forces/fbx-total-lbs',
#                                                                   'forces/fby-total-lbs',
#                                                                   'forces/fbz-total-lbs' ]) )
#                 
#         print( 'forces/fbx-aero-lbs       : ', self.getPropVal( [ 'forces/fbx-aero-lbs',
#                                                                   'forces/fby-aero-lbs',
#                                                                   'forces/fbz-aero-lbs' ]) )
#         
#         print( 'forces/fbx-weight-lbs     : ', self.getPropVal( [ 'forces/fbx-weight-lbs',
#                                                                   'forces/fby-weight-lbs',
#                                                                   'forces/fbz-weight-lbs' ]) )
#         
#         print( 'forces/fbx-gear-lbs       : ', self.getPropVal( [ 'forces/fbx-gear-lbs',
#                                                                   'forces/fby-gear-lbs',
#                                                                   'forces/fbz-gear-lbs' ]) )
#         
#         print( 'forces/fbx-prop-lbs       : ', self.getPropVal( [ 'forces/fbx-prop-lbs',
#                                                                   'forces/fby-prop-lbs',
#                                                                   'forces/fbz-prop-lbs' ]) )
# 
#         
#         print( 'moments/l-total-lbsft     : ', self.getPropVal( [ 'moments/l-total-lbsft',
#                                                                   'moments/m-total-lbsft',
#                                                                   'moments/n-total-lbsft' ]) )
#         
#         print( 'moments/l-aero-lbsft      : ', self.getPropVal( [ 'moments/l-aero-lbsft',
#                                                                   'moments/m-aero-lbsft',
#                                                                   'moments/n-aero-lbsft' ]) )
#         
#         print( 'moments/l-gear-lbsft      : ', self.getPropVal( [ 'moments/l-gear-lbsft',
#                                                                   'moments/m-gear-lbsft',
#                                                                   'moments/n-gear-lbsft' ]) )
#         
#         print( 'moments/l-prop-lbsft      : ', self.getPropVal( [ 'moments/l-prop-lbsft',
#                                                                   'moments/m-prop-lbsft',
#                                                                   'moments/n-prop-lbsft' ]) )
#         

    def step( self, curTime ) :
        #print( 'step' ) 
        while self.getPropVal( 'simulation/sim-time-sec' ) < curTime :
            self.fdm.run()
        self.simTime = self.getPropVal(   'simulation/sim-time-sec'  )
        self.calc() 
                                  


 
class JSBAircraft_F15( JSBAircraft ) :
    
    def __init__( self ) :
        
        super().__init__( 'myScript_F15.xml' )
        
        self.setPropVal( 'gear/gear-cmd-norm'       , 0.0 )
        self.setPropVal( 'fcs/left-brake-cmd-norm'  , 0.0 )
        self.setPropVal( 'fcs/right-brake-cmd-norm' , 0.0 )
        self.setPropVal( 'fcs/center-brake-cmd-norm', 0.0 )

        self.numEngines = 2
        self.thst_ref = self.numEngines * 29000.0 * 0.4536 * 9.80665
        #
        #  F100-PW-229
        #

#         for x in self.fdm.get_property_catalog() :
#             print( x )

    def loadScript( self, script ) :
        #print( 'loadScript JSBAircraft_F15', script )
        filePath = os.path.dirname( __file__ )
        self.fdm.set_root_dir     ( filePath + '\JsbsimModel' )
        self.fdm.set_aircraft_path( 'F15' )
        self.fdm.set_engine_path  ( 'F15' )
        self.fdm.set_systems_path ( 'F15' )    
        self.fdm.load_script( script ) 
 
 
class JSBAircraft_F16( JSBAircraft ) :
    
    def __init__( self ) :
        
        super().__init__( 'myScript_F16.xml' )
        
        self.setPropVal( 'gear/gear-cmd-norm'       , 0.0 )
        self.setPropVal( 'fcs/left-brake-cmd-norm'  , 0.0 )
        self.setPropVal( 'fcs/right-brake-cmd-norm' , 0.0 )
        self.setPropVal( 'fcs/center-brake-cmd-norm', 0.0 )

        self.setPropVal( 'fcs/fbw-override', 0 )

        self.numEngines = 1
        self.thst_ref = self.numEngines * 29000.0 * 0.4536 * 9.80665

#         for x in self.fdm.get_property_catalog() :
#             print( x )

    def loadScript( self, script ) :
        #print( 'loadScript JSBAircraft_F15', script )
        filePath = os.path.dirname( __file__ )
        self.fdm.set_root_dir     ( filePath + '\JsbsimModel' )
        self.fdm.set_aircraft_path( 'F16' )
        self.fdm.set_engine_path  ( 'F16' )
        self.fdm.set_systems_path ( 'F16' )    
        self.fdm.load_script( script ) 



class JSBAircraft_F22( JSBAircraft ) :
    
    def __init__( self ) :
        
        super().__init__( 'myScript_F22.xml' )
        
        self.setPropVal( 'gear/gear-cmd-norm'       , 0.0 )
        self.setPropVal( 'fcs/left-brake-cmd-norm'  , 0.0 )
        self.setPropVal( 'fcs/right-brake-cmd-norm' , 0.0 )
        self.setPropVal( 'fcs/center-brake-cmd-norm', 0.0 )

        self.numEngines = 2
        self.thst_ref = self.numEngines * 26950.0 * 0.4536 * 9.80665

#         for x in self.fdm.get_property_catalog() :
#             print( x )

    def loadScript( self, script ) :
        #print( 'loadScript JSBAircraft_F15', script )
        filePath = os.path.dirname( __file__ )
        self.fdm.set_root_dir     ( filePath + '\JsbsimModel' )
        self.fdm.set_aircraft_path( 'F22' )
        self.fdm.set_engine_path  ( 'F22' )
        self.fdm.set_systems_path ( 'F22' )    
        self.fdm.load_script( script ) 
 
 
class JSBAircraft_F35( JSBAircraft ) :
    
    def __init__( self ) :
        
        super().__init__( 'myScript_F35.xml' )
        
        self.setPropVal( 'gear/gear-cmd-norm'       , 0.0 )
        self.setPropVal( 'fcs/left-brake-cmd-norm'  , 0.0 )
        self.setPropVal( 'fcs/right-brake-cmd-norm' , 0.0 )
        self.setPropVal( 'fcs/center-brake-cmd-norm', 0.0 )

        self.numEngines = 1
        self.thst_ref = self.numEngines * 28000.0 * 0.4536 * 9.80665

        for x in self.fdm.get_property_catalog() :
            print( x )

    def loadScript( self, script ) :
        #print( 'loadScript JSBAircraft_F15', script )
        filePath = os.path.dirname( __file__ )
        self.fdm.set_root_dir     ( filePath + '\JsbsimModel' )
        self.fdm.set_aircraft_path( 'F35' )
        self.fdm.set_engine_path  ( 'F35' )
        self.fdm.set_systems_path ( 'F35' )    
        self.fdm.load_script( script ) 
 
 
class JSBAircraft_t6texan2( JSBAircraft ) :
    
    def __init__( self ) :
        
        super().__init__( 'myScript_t6texan2.xml' )
        
        self.setPropVal( 'gear/gear-cmd-norm'       , 0.0 )
        self.setPropVal( 'fcs/left-brake-cmd-norm'  , 0.0 )
        self.setPropVal( 'fcs/right-brake-cmd-norm' , 0.0 )
        self.setPropVal( 'fcs/center-brake-cmd-norm', 0.0 )

        self.numEngines = 1
        self.thst_ref = self.numEngines * 2464.0 * 0.4536 * 9.80665

        self.xFilter = FirstOrderLagFilter( 0.1 )


    def loadScript( self, script ) :
        #print( 'loadScript JSBAircraft_F15', script )
        filePath = os.path.dirname( __file__ )
        self.fdm.set_root_dir     ( filePath + '\JsbsimModel' )
        self.fdm.set_aircraft_path( 't6texan2' )
        self.fdm.set_engine_path  ( 't6texan2' )
        self.fdm.set_systems_path ( 't6texan2' )    
        self.fdm.load_script( script ) 
 
    @property
    def stickPos( self ) :
        X = self.getPropVal( 'fcs/aileron-cmd-norm'  )
        Y = self.getPropVal( 'fcs/elevator-cmd-norm' )
        Z = self.getPropVal( 'fcs/rudder-cmd-norm'   )
        return np.array( [ X, Y, Z ] )
        
    @stickPos.setter
    def stickPos( self, value ) :
        XX, Y, Z = value
        
        X = XX
        #X = XX * ( 0.9 * XX * XX + 0.1 )
        #X = self.xFilter.val( self.simTime, XX )
        print( X )

        self.setPropVal( 'fcs/aileron-cmd-norm' , X )
        self.setPropVal( 'fcs/elevator-cmd-norm', Y * 0.5 )
        self.setPropVal( 'fcs/rudder-cmd-norm'  , Z )
 
 
class JSBAircraft_B747( JSBAircraft ) :
    
    def __init__( self ) :
        print( '__init__ JSBAircraft_B747 ' )
        
        super().__init__( 'myScript_B747.xml' )
        
        self.setPropVal( 'gear/gear-cmd-norm'       , 0.0 )
        self.setPropVal( 'fcs/left-brake-cmd-norm'  , 0.0 )
        self.setPropVal( 'fcs/right-brake-cmd-norm' , 0.0 )
        self.setPropVal( 'fcs/center-brake-cmd-norm', 0.0 )

        self.numEngines = 4
        self.thst_ref = self.numEngines * 58000.0 * 0.4536 * 9.80665

    def loadScript( self, script ) :
        #print( 'loadScript JSBAircraft_F15', script )
        filePath = os.path.dirname( __file__ )
        self.fdm.set_root_dir     ( filePath + '\JsbsimModel' )
        self.fdm.set_aircraft_path( 'B747' )
        self.fdm.set_engine_path  ( 'B747' )
        self.fdm.set_systems_path ( 'B747' )    
        self.fdm.load_script( script ) 

    @property
    def stickPos( self ) :
        X = self.getPropVal( 'fcs/aileron-cmd-norm'  )
        Y = self.getPropVal( 'fcs/elevator-cmd-norm' )
        Z = self.getPropVal( 'fcs/rudder-cmd-norm'   )
        return np.array( [ X, Y, Z ] )
        
    @stickPos.setter
    def stickPos( self, value ) :
        X, Y, Z = value
        
        #X = self.xFilter.val( self.simTime, XX )
         
        self.setPropVal( 'fcs/aileron-cmd-norm' , X )
        self.setPropVal( 'fcs/elevator-cmd-norm', Y )
        self.setPropVal( 'fcs/rudder-cmd-norm'  , Z )
 
 
 
class JSBAircraft_c172p( JSBAircraft ) :
    
    def __init__( self ) :
        print( '__init__ JSBAircraft_c172p ' )
        
        super().__init__( 'myScript_c172p.xml' )
        
        self.setPropVal( 'gear/gear-cmd-norm'       , 0.0 )
        self.setPropVal( 'fcs/left-brake-cmd-norm'  , 0.0 )
        self.setPropVal( 'fcs/right-brake-cmd-norm' , 0.0 )
        self.setPropVal( 'fcs/center-brake-cmd-norm', 0.0 )

        self.setPropVal( 'fcs/mixture-cmd-norm'     , 0.8 )

        self.numEngines = 1
        self.thst_ref = self.numEngines * 58000.0 * 0.4536 * 9.80665

        for x in self.fdm.get_property_catalog() :
            print( x )

    def loadScript( self, script ) :
        #print( 'loadScript JSBAircraft_F15', script )
        filePath = os.path.dirname( __file__ )
        self.fdm.set_root_dir     ( filePath + '\JsbsimModel' )
        self.fdm.set_aircraft_path( 'c172p' )
        self.fdm.set_engine_path  ( 'c172p' )
        self.fdm.set_systems_path ( 'c172p' )    
        self.fdm.load_script( script ) 






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
        
        self.graph = ManyTHGraphs( self, 0.0, 20.0,
                   [ [  0, 0, ( 'X', 'Y' )                       , ( -10.0, 10.0 )                ],
                     [  1, 0, ( 'Vt_kt', 'ALT_ft>' )             , (   0.0, 500.0, 0.0, 3000.0 ) ],
                     [  2, 0, ( 'CL', 'CD', 'TbyW' )             , (  -1.5, 1.5)                  ],
                     #[  0, 1, ( 'gam_deg', 'alp_deg', 'the_deg' ), ( -10.0, 10.0 )                ],
                     [  0, 1, ( 'gam_deg', 'gam_tar_deg', 'alp_deg', 'alpCom_deg' ), ( -10.0, 10.0 )                ],
                     [  1, 1, ( 'phi_deg', 'PHI_deg>' )          , ( -50.0, 50.0 )                ],
                     [  2, 1, ( 'psi_deg', 'PSI_deg>' )          , ( -50.0, 50.0 )                ],
                   ] )
        
        
        
        
        
        mainVBoxLO.addWidget( self.graph )
        
        self.curTime = 0.0
        
        self.ptmassAircraft = PointMassAircraft()
        self.ptmassAircraft.reset( self.curTime )
        
        
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

        self.ptmassAircraft.step( self.curTime )

        print( self.curTime )
        self.graph.addData( [ self.curTime,
                              
                              self.ptmassAircraft.param[ 'XY_Nm'  ][0] ,
                              self.ptmassAircraft.param[ 'XY_Nm'  ][1] ,
                              
                              self.ptmassAircraft.param[ 'Vt_kt'  ]    ,
                              self.ptmassAircraft.param[ 'ALT_ft' ]    ,
                              
                              self.ptmassAircraft.CL                   ,
                              self.ptmassAircraft.CD                   ,
                              self.ptmassAircraft.TbyW                 ,
                              
                              self.ptmassAircraft.param[ 'gam_deg' ]   ,
                              np.degrees( self.ptmassAircraft.gam_target )   ,
                              self.ptmassAircraft.param[ 'ALP_deg' ]   ,
                              np.degrees( self.ptmassAircraft.alpCom ) ,
                              #self.ptmassAircraft.param[ 'eul_deg' ][1],
                              
                              np.degrees( self.ptmassAircraft.PHI )    ,
                              self.ptmassAircraft.param[ 'eul_deg' ][0],
                              
                              np.degrees( self.ptmassAircraft.PSI )    ,
                              self.ptmassAircraft.param[ 'eul_deg' ][2],
                              
                              ]  )
 
 
 
        #self.param[ 'eul_deg' ] = np.degrees( self.eul )
#         self.param[ 'ALP_deg' ] = np.degrees( self.alp )
#         self.param[ 'BET_deg' ] = 0.0
# 
#         self.param[ 'gam_deg' ] = np.degrees( self.gam )
 
 
#         self.graph.addData( [ self.curTime,
#                               np.cos( 1.0 * self.curTime ),
#                               np.cos( 2.0 * self.curTime ),
#                               np.cos( 3.0 * self.curTime ),
#                               np.cos( 4.0 * self.curTime ),
#                               np.cos( 5.0 * self.curTime ),
#                               np.cos( 6.0 * self.curTime ) ]  )
        

        
if __name__ == '__main__':
    app = QApplication( sys.argv )
    mainwindow = MainWindow()
    mainwindow.show()  
    app.exec()







'''


     JSBSim Flight Dynamics Model v1.1.13 [GitHub build 986/commit a09715f01b9e568ce75ca2635ba0a78ce57f7cdd] Dec  3 2022 12:36:17
            [JSBSim-ML v2.0]

JSBSim startup beginning ...


#         for x in dir(self.fdm):
#             print( x )        


__class__
__delattr__
__delitem__
__dir__
__doc__
__eq__
__format__
__ge__
__getattribute__
__getitem__
__getstate__
__gt__
__hash__
__init__
__init_subclass__
__le__
__lt__
__ne__
__new__
__reduce__
__reduce_ex__
__repr__
__setattr__
__setitem__
__setstate__
__sizeof__
__str__
__subclasshook__
check_incremental_hold
debug_lvl
disable_highlighting
disable_output
do_trim
enable_increment_then_hold
enable_output
get_aerodynamics
get_aircraft
get_aircraft_path
get_atmosphere
get_auxiliary
get_debug_level
get_delta_t
get_engine_path
get_full_aircraft_path
get_ground_reactions
get_mass_balance
get_model_name
get_output_filename
get_output_path
get_propagate
get_property_catalog
get_property_manager
get_property_value
get_propulsion
get_propulsion_tank_report
get_root_dir
get_sim_time
get_systems_path
get_trim_status
get_version
hold
holding
incr_time
integration_suspended
load_ic
load_model
load_model_with_paths
load_script
print_property_catalog
print_simulation_configuration
query_property_catalog
reset_to_initial_conditions
resume
resume_integration
run
run_ic
set_aircraft_path
set_debug_level
set_dt
set_engine_path
set_logging_rate
set_output_directive
set_output_filename
set_output_path
set_property_value
set_root_dir
set_sim_time
set_systems_path
set_trim_status
suspend_integration



#         
#         for x in self.fdm.get_property_catalog() :
#             print( x )


inertial/sea-level-radius_ft (R)
simulation/gravity-model (RW)
simulation/integrator/rate/rotational (RW)
simulation/integrator/rate/translational (RW)
simulation/integrator/position/rotational (RW)
simulation/integrator/position/translational (RW)
simulation/write-state-file (W)
simulation/channel-dt (R)
simulation/gravitational-torque (RW)
simulation/force-output (W)
simulation/do_simple_trim (W)
simulation/reset (W)
simulation/disperse (R)
simulation/randomseed (RW)
simulation/terminate (RW)
simulation/pause (RW)
simulation/sim-time-sec (R)
simulation/dt (R)
simulation/jsbsim-debug (RW)
simulation/frame (RW)
simulation/trim-completed (RW)
velocities/h-dot-fps (R)
velocities/v-north-fps (R)
velocities/v-east-fps (R)
velocities/v-down-fps (R)
velocities/u-fps (R)
velocities/v-fps (R)
velocities/w-fps (R)
velocities/p-rad_sec (R)
velocities/q-rad_sec (R)
velocities/r-rad_sec (R)
velocities/pi-rad_sec (R)
velocities/qi-rad_sec (R)
velocities/ri-rad_sec (R)
velocities/eci-x-fps (R)
velocities/eci-y-fps (R)
velocities/eci-z-fps (R)
velocities/eci-velocity-mag-fps (R)
velocities/ned-velocity-mag-fps (R)
velocities/vc-fps (R)
velocities/vc-kts (R)
velocities/ve-fps (R)
velocities/ve-kts (R)
velocities/vtrue-fps (R)
velocities/vtrue-kts (R)
velocities/machU (R)
velocities/p-aero-rad_sec (R)
velocities/q-aero-rad_sec (R)
velocities/r-aero-rad_sec (R)
velocities/phidot-rad_sec (R)
velocities/thetadot-rad_sec (R)
velocities/psidot-rad_sec (R)
velocities/u-aero-fps (R)
velocities/v-aero-fps (R)
velocities/w-aero-fps (R)
velocities/vt-fps (R)
velocities/mach (R)
velocities/vg-fps (R)
position/h-sl-ft (RW)
position/h-sl-meters (RW)
position/lat-gc-rad (RW)
position/long-gc-rad (RW)
position/lat-gc-deg (RW)
position/long-gc-deg (RW)
position/lat-geod-rad (R)
position/lat-geod-deg (R)
position/geod-alt-ft (R)
position/h-agl-ft (RW)
position/geod-alt-km (R)
position/h-agl-km (RW)
position/radius-to-vehicle-ft (R)
position/terrain-elevation-asl-ft (RW)
position/eci-x-ft (R)
position/eci-y-ft (R)
position/eci-z-ft (R)
position/ecef-x-ft (R)
position/ecef-y-ft (R)
position/ecef-z-ft (R)
position/epa-rad (R)
position/distance-from-start-lon-mt (R)
position/distance-from-start-lat-mt (R)
position/distance-from-start-mag-mt (R)
position/vrp-gc-latitude_deg (R)
position/vrp-longitude_deg (R)
position/vrp-radius-ft (R)
metrics/terrain-radius (R)
metrics/Sw-sqft (RW)
metrics/bw-ft (R)
metrics/cbarw-ft (R)
metrics/iw-rad (R)
metrics/iw-deg (R)
metrics/Sh-sqft (R)
metrics/lh-ft (R)
metrics/Sv-sqft (R)
metrics/lv-ft (R)
metrics/lh-norm (R)
metrics/lv-norm (R)
metrics/vbarh-norm (R)
metrics/vbarv-norm (R)
metrics/aero-rp-x-in (RW)
metrics/aero-rp-y-in (RW)
metrics/aero-rp-z-in (RW)
metrics/eyepoint-x-in (R)
metrics/eyepoint-y-in (R)
metrics/eyepoint-z-in (R)
metrics/visualrefpoint-x-in (R)
metrics/visualrefpoint-y-in (R)
metrics/visualrefpoint-z-in (R)
attitude/phi-rad (R)
attitude/theta-rad (R)
attitude/psi-rad (R)
attitude/phi-deg (R)
attitude/theta-deg (R)
attitude/psi-deg (R)
attitude/roll-rad (R)
attitude/pitch-rad (R)
attitude/heading-true-rad (R)
atmosphere/T-R (R)
atmosphere/rho-slugs_ft3 (R)
atmosphere/P-psf (R)
atmosphere/a-fps (R)
atmosphere/T-sl-R (R)
atmosphere/rho-sl-slugs_ft3 (R)
atmosphere/a-sl-fps (R)
atmosphere/theta (R)
atmosphere/sigma (R)
atmosphere/delta (R)
atmosphere/a-ratio (R)
atmosphere/density-altitude (R)
atmosphere/pressure-altitude (R)
atmosphere/delta-T (RW)
atmosphere/SL-graded-delta-T (RW)
atmosphere/P-sl-psf (RW)
atmosphere/dew-point-R (RW)
atmosphere/vapor-pressure-psf (RW)
atmosphere/saturated-vapor-pressure-psf (R)
atmosphere/RH (RW)
atmosphere/vapor-fraction-ppm (RW)
atmosphere/psiw-rad (RW)
atmosphere/wind-north-fps (RW)
atmosphere/wind-east-fps (RW)
atmosphere/wind-down-fps (RW)
atmosphere/wind-mag-fps (RW)
atmosphere/gust-north-fps (RW)
atmosphere/gust-east-fps (RW)
atmosphere/gust-down-fps (RW)
atmosphere/cosine-gust/startup-duration-sec (W)
atmosphere/cosine-gust/steady-duration-sec (W)
atmosphere/cosine-gust/end-duration-sec (W)
atmosphere/cosine-gust/magnitude-ft_sec (W)
atmosphere/cosine-gust/frame (W)
atmosphere/cosine-gust/X-velocity-ft_sec (W)
atmosphere/cosine-gust/Y-velocity-ft_sec (W)
atmosphere/cosine-gust/Z-velocity-ft_sec (W)
atmosphere/cosine-gust/start (W)
atmosphere/updownburst/number-of-cells (W)
atmosphere/turb-north-fps (RW)
atmosphere/turb-east-fps (RW)
atmosphere/turb-down-fps (RW)
atmosphere/p-turb-rad_sec (R)
atmosphere/q-turb-rad_sec (R)
atmosphere/r-turb-rad_sec (R)
atmosphere/turb-type (RW)
atmosphere/turb-rate (RW)
atmosphere/turb-gain (RW)
atmosphere/turb-rhythmicity (RW)
atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps (RW)
atmosphere/turbulence/milspec/severity (RW)
atmosphere/total-wind-north-fps (R)
atmosphere/total-wind-east-fps (R)
atmosphere/total-wind-down-fps (R)
fcs/aileron-cmd-norm (RW)
fcs/elevator-cmd-norm (RW)
fcs/rudder-cmd-norm (RW)
fcs/flap-cmd-norm (RW)
fcs/speedbrake-cmd-norm (RW)
fcs/spoiler-cmd-norm (RW)
fcs/pitch-trim-cmd-norm (RW)
fcs/roll-trim-cmd-norm (RW)
fcs/yaw-trim-cmd-norm (RW)
fcs/left-aileron-pos-rad (RW)
fcs/left-aileron-pos-deg (RW)
fcs/left-aileron-pos-norm (RW)
fcs/mag-left-aileron-pos-rad (R)
fcs/right-aileron-pos-rad (RW)
fcs/right-aileron-pos-deg (RW)
fcs/right-aileron-pos-norm (RW)
fcs/mag-right-aileron-pos-rad (R)
fcs/elevator-pos-rad (RW)
fcs/elevator-pos-deg (RW)
fcs/elevator-pos-norm (RW)
fcs/mag-elevator-pos-rad (R)
fcs/rudder-pos-rad (RW)
fcs/rudder-pos-deg (RW)
fcs/rudder-pos-norm (RW)
fcs/mag-rudder-pos-rad (R)
fcs/flap-pos-rad (RW)
fcs/flap-pos-deg (RW)
fcs/flap-pos-norm (RW)
fcs/speedbrake-pos-rad (RW)
fcs/speedbrake-pos-deg (RW)
fcs/speedbrake-pos-norm (RW)
fcs/mag-speedbrake-pos-rad (R)
fcs/spoiler-pos-rad (RW)
fcs/spoiler-pos-deg (RW)
fcs/spoiler-pos-norm (RW)
fcs/mag-spoiler-pos-rad (R)
fcs/left-brake-cmd-norm (RW)
fcs/right-brake-cmd-norm (RW)
fcs/center-brake-cmd-norm (RW)
fcs/wing-fold-pos-norm (RW)
fcs/steer-cmd-norm (RW)
fcs/steer-pos-deg (RW)
fcs/throttle-cmd-norm (RW)
fcs/throttle-pos-norm (RW)
fcs/mixture-cmd-norm (RW)
fcs/mixture-pos-norm (RW)
fcs/advance-cmd-norm (RW)
fcs/advance-pos-norm (RW)
fcs/feather-cmd-norm (RW)
fcs/feather-pos-norm (RW)
fcs/throttle-cmd-norm[1] (RW)
fcs/throttle-pos-norm[1] (RW)
fcs/mixture-cmd-norm[1] (RW)
fcs/mixture-pos-norm[1] (RW)
fcs/advance-cmd-norm[1] (RW)
fcs/advance-pos-norm[1] (RW)
fcs/feather-cmd-norm[1] (RW)
fcs/feather-pos-norm[1] (RW)
fcs/pitch-trim-sum (RW)
fcs/elevator-position (RW)
fcs/roll-trim-sum (RW)
fcs/aileron-pos-norm (RW)
fcs/left-aileron-control (RW)
fcs/right-aileron-control (RW)
fcs/yaw-damper-gain (RW)
fcs/yaw-trim-sum (RW)
fcs/rudder-position (RW)
fcs/rudder-control (RW)
fcs/gear-control (RW)
fcs/speedbrake-control (RW)
gear/gear-pos-norm (RW)
gear/gear-cmd-norm (RW)
gear/tailhook-pos-norm (RW)
gear/num-units (R)
gear/wow (R)
gear/unit/solid (RW)
gear/unit/bumpiness (RW)
gear/unit/maximum-force-lbs (RW)
gear/unit/rolling_friction-factor (RW)
gear/unit/static-friction-factor (RW)
gear/unit/WOW (RW)
gear/unit/x-position (RW)
gear/unit/y-position (RW)
gear/unit/z-position (RW)
gear/unit/compression-ft (RW)
gear/unit/compression-velocity-fps (RW)
gear/unit/static_friction_coeff (RW)
gear/unit/dynamic_friction_coeff (RW)
gear/unit/slip-angle-deg (RW)
gear/unit/wheel-speed-fps (R)
gear/unit/side_friction_coeff (RW)
gear/unit/rolling_friction_coeff (RW)
gear/unit/pos-norm (RW)
gear/unit[1]/solid (RW)
gear/unit[1]/bumpiness (RW)
gear/unit[1]/maximum-force-lbs (RW)
gear/unit[1]/rolling_friction-factor (RW)
gear/unit[1]/static-friction-factor (RW)
gear/unit[1]/WOW (RW)
gear/unit[1]/x-position (RW)
gear/unit[1]/y-position (RW)
gear/unit[1]/z-position (RW)
gear/unit[1]/compression-ft (RW)
gear/unit[1]/compression-velocity-fps (RW)
gear/unit[1]/static_friction_coeff (RW)
gear/unit[1]/dynamic_friction_coeff (RW)
gear/unit[1]/slip-angle-deg (RW)
gear/unit[1]/wheel-speed-fps (R)
gear/unit[1]/side_friction_coeff (RW)
gear/unit[1]/rolling_friction_coeff (RW)
gear/unit[1]/pos-norm (RW)
gear/unit[2]/solid (RW)
gear/unit[2]/bumpiness (RW)
gear/unit[2]/maximum-force-lbs (RW)
gear/unit[2]/rolling_friction-factor (RW)
gear/unit[2]/static-friction-factor (RW)
gear/unit[2]/WOW (RW)
gear/unit[2]/x-position (RW)
gear/unit[2]/y-position (RW)
gear/unit[2]/z-position (RW)
gear/unit[2]/compression-ft (RW)
gear/unit[2]/compression-velocity-fps (RW)
gear/unit[2]/static_friction_coeff (RW)
gear/unit[2]/dynamic_friction_coeff (RW)
gear/unit[2]/slip-angle-deg (RW)
gear/unit[2]/wheel-speed-fps (R)
gear/unit[2]/side_friction_coeff (RW)
gear/unit[2]/rolling_friction_coeff (RW)
gear/unit[2]/pos-norm (RW)
gear/unit[3]/solid (RW)
gear/unit[3]/bumpiness (RW)
gear/unit[3]/maximum-force-lbs (RW)
gear/unit[3]/rolling_friction-factor (RW)
gear/unit[3]/static-friction-factor (RW)
gear/unit[3]/WOW (RW)
gear/unit[3]/x-position (RW)
gear/unit[3]/y-position (RW)
gear/unit[3]/z-position (RW)
gear/unit[3]/compression-ft (RW)
gear/unit[3]/compression-velocity-fps (RW)
gear/unit[3]/static_friction_coeff (RW)
gear/unit[3]/dynamic_friction_coeff (RW)
gear/unit[3]/slip-angle-deg (RW)
gear/unit[3]/wheel-speed-fps (R)
gear/unit[3]/side_friction_coeff (RW)
gear/unit[3]/rolling_friction_coeff (RW)
gear/unit[4]/solid (RW)
gear/unit[4]/bumpiness (RW)
gear/unit[4]/maximum-force-lbs (RW)
gear/unit[4]/rolling_friction-factor (RW)
gear/unit[4]/static-friction-factor (RW)
gear/unit[4]/WOW (RW)
gear/unit[4]/x-position (RW)
gear/unit[4]/y-position (RW)
gear/unit[4]/z-position (RW)
gear/unit[4]/compression-ft (RW)
gear/unit[4]/compression-velocity-fps (RW)
gear/unit[4]/static_friction_coeff (RW)
gear/unit[4]/dynamic_friction_coeff (RW)
gear/unit[4]/slip-angle-deg (RW)
gear/unit[4]/wheel-speed-fps (R)
gear/unit[4]/side_friction_coeff (RW)
gear/unit[4]/rolling_friction_coeff (RW)
gear/unit[5]/solid (RW)
gear/unit[5]/bumpiness (RW)
gear/unit[5]/maximum-force-lbs (RW)
gear/unit[5]/rolling_friction-factor (RW)
gear/unit[5]/static-friction-factor (RW)
gear/unit[5]/WOW (RW)
gear/unit[5]/x-position (RW)
gear/unit[5]/y-position (RW)
gear/unit[5]/z-position (RW)
gear/unit[5]/compression-ft (RW)
gear/unit[5]/compression-velocity-fps (RW)
gear/unit[5]/static_friction_coeff (RW)
gear/unit[5]/dynamic_friction_coeff (RW)
gear/unit[5]/slip-angle-deg (RW)
gear/unit[5]/wheel-speed-fps (R)
gear/unit[5]/side_friction_coeff (RW)
gear/unit[5]/rolling_friction_coeff (RW)
gear/unit[6]/solid (RW)
gear/unit[6]/bumpiness (RW)
gear/unit[6]/maximum-force-lbs (RW)
gear/unit[6]/rolling_friction-factor (RW)
gear/unit[6]/static-friction-factor (RW)
gear/unit[6]/WOW (RW)
gear/unit[6]/x-position (RW)
gear/unit[6]/y-position (RW)
gear/unit[6]/z-position (RW)
gear/unit[6]/compression-ft (RW)
gear/unit[6]/compression-velocity-fps (RW)
gear/unit[6]/static_friction_coeff (RW)
gear/unit[6]/dynamic_friction_coeff (RW)
gear/unit[6]/slip-angle-deg (RW)
gear/unit[6]/wheel-speed-fps (R)
gear/unit[6]/side_friction_coeff (RW)
gear/unit[6]/rolling_friction_coeff (RW)
gear/unit[7]/solid (RW)
gear/unit[7]/bumpiness (RW)
gear/unit[7]/maximum-force-lbs (RW)
gear/unit[7]/rolling_friction-factor (RW)
gear/unit[7]/static-friction-factor (RW)
gear/unit[7]/WOW (RW)
gear/unit[7]/x-position (RW)
gear/unit[7]/y-position (RW)
gear/unit[7]/z-position (RW)
gear/unit[7]/compression-ft (RW)
gear/unit[7]/compression-velocity-fps (RW)
gear/unit[7]/static_friction_coeff (RW)
gear/unit[7]/dynamic_friction_coeff (RW)
gear/unit[7]/slip-angle-deg (RW)
gear/unit[7]/wheel-speed-fps (R)
gear/unit[7]/side_friction_coeff (RW)
gear/unit[7]/rolling_friction_coeff (RW)
gear/unit[8]/solid (RW)
gear/unit[8]/bumpiness (RW)
gear/unit[8]/maximum-force-lbs (RW)
gear/unit[8]/rolling_friction-factor (RW)
gear/unit[8]/static-friction-factor (RW)
gear/unit[8]/WOW (RW)
gear/unit[8]/x-position (RW)
gear/unit[8]/y-position (RW)
gear/unit[8]/z-position (RW)
gear/unit[8]/compression-ft (RW)
gear/unit[8]/compression-velocity-fps (RW)
gear/unit[8]/static_friction_coeff (RW)
gear/unit[8]/dynamic_friction_coeff (RW)
gear/unit[8]/slip-angle-deg (RW)
gear/unit[8]/wheel-speed-fps (R)
gear/unit[8]/side_friction_coeff (RW)
gear/unit[8]/rolling_friction_coeff (RW)
inertia/mass-slugs (R)
inertia/weight-lbs (R)
inertia/empty-weight-lbs (R)
inertia/cg-x-in (R)
inertia/cg-y-in (R)
inertia/cg-z-in (R)
inertia/ixx-slugs_ft2 (R)
inertia/iyy-slugs_ft2 (R)
inertia/izz-slugs_ft2 (R)
inertia/ixy-slugs_ft2 (R)
inertia/ixz-slugs_ft2 (R)
inertia/iyz-slugs_ft2 (R)
inertia/print-mass-properties (W)
inertia/pointmass-weight-lbs (RW)
inertia/pointmass-location-X-inches (RW)
inertia/pointmass-location-Y-inches (RW)
inertia/pointmass-location-Z-inches (RW)
propulsion/tat-r (R)
propulsion/tat-c (R)
propulsion/pt-lbs_sqft (R)
propulsion/tank/contents-lbs (RW)
propulsion/tank/unusable-volume-gal (RW)
propulsion/tank/pct-full (R)
propulsion/tank/density-lbs_per_gal (R)
propulsion/tank/priority (RW)
propulsion/tank/external-flow-rate-pps (RW)
propulsion/tank/local-ixx-slug_ft2 (R)
propulsion/tank/local-iyy-slug_ft2 (R)
propulsion/tank/local-izz-slug_ft2 (R)
propulsion/tank/x-position (RW)
propulsion/tank/y-position (RW)
propulsion/tank/z-position (RW)
propulsion/tank[1]/contents-lbs (RW)
propulsion/tank[1]/unusable-volume-gal (RW)
propulsion/tank[1]/pct-full (R)
propulsion/tank[1]/density-lbs_per_gal (R)
propulsion/tank[1]/priority (RW)
propulsion/tank[1]/external-flow-rate-pps (RW)
propulsion/tank[1]/local-ixx-slug_ft2 (R)
propulsion/tank[1]/local-iyy-slug_ft2 (R)
propulsion/tank[1]/local-izz-slug_ft2 (R)
propulsion/tank[1]/x-position (RW)
propulsion/tank[1]/y-position (RW)
propulsion/tank[1]/z-position (RW)
propulsion/set-running (W)
propulsion/starter_cmd (RW)
propulsion/cutoff_cmd (RW)
propulsion/active_engine (RW)
propulsion/total-fuel-lbs (RW)
propulsion/total-oxidizer-lbs (RW)
propulsion/refuel (RW)
propulsion/fuel_dump (RW)
propulsion/fuel_freeze (W)
propulsion/engine/IdleThrust (R)
propulsion/engine/MilThrust (R)
propulsion/engine/AugThrust (R)
propulsion/engine/pitch-angle-rad (RW)
propulsion/engine/yaw-angle-rad (RW)
propulsion/engine/reverser-angle-rad (RW)
propulsion/engine/set-running (RW)
propulsion/engine/thrust-lbs (R)
propulsion/engine/fuel-flow-rate-pps (R)
propulsion/engine/fuel-flow-rate-gph (R)
propulsion/engine/fuel-used-lbs (R)
propulsion/engine/n1 (RW)
propulsion/engine/n2 (RW)
propulsion/engine/injection_cmd (RW)
propulsion/engine/seized (RW)
propulsion/engine/stalled (RW)
propulsion/engine/bleed-factor (RW)
propulsion/engine/MaxN1 (RW)
propulsion/engine/MaxN2 (RW)
propulsion/engine/InjectionTimer (RW)
propulsion/engine/InjWaterNorm (RW)
propulsion/engine/InjN1increment (RW)
propulsion/engine/InjN2increment (RW)
propulsion/engine[1]/IdleThrust (R)
propulsion/engine[1]/MilThrust (R)
propulsion/engine[1]/AugThrust (R)
propulsion/engine[1]/pitch-angle-rad (RW)
propulsion/engine[1]/yaw-angle-rad (RW)
propulsion/engine[1]/reverser-angle-rad (RW)
propulsion/engine[1]/set-running (RW)
propulsion/engine[1]/thrust-lbs (R)
propulsion/engine[1]/fuel-flow-rate-pps (R)
propulsion/engine[1]/fuel-flow-rate-gph (R)
propulsion/engine[1]/fuel-used-lbs (R)
propulsion/engine[1]/n1 (RW)
propulsion/engine[1]/n2 (RW)
propulsion/engine[1]/injection_cmd (RW)
propulsion/engine[1]/seized (RW)
propulsion/engine[1]/stalled (RW)
propulsion/engine[1]/bleed-factor (RW)
propulsion/engine[1]/MaxN1 (RW)
propulsion/engine[1]/MaxN2 (RW)
propulsion/engine[1]/InjectionTimer (RW)
propulsion/engine[1]/InjWaterNorm (RW)
propulsion/engine[1]/InjN1increment (RW)
propulsion/engine[1]/InjN2increment (RW)
accelerations/a-pilot-x-ft_sec2 (R)
accelerations/a-pilot-y-ft_sec2 (R)
accelerations/a-pilot-z-ft_sec2 (R)
accelerations/n-pilot-x-norm (R)
accelerations/n-pilot-y-norm (R)
accelerations/n-pilot-z-norm (R)
accelerations/Nx (R)
accelerations/Ny (R)
accelerations/Nz (R)
accelerations/pdot-rad_sec2 (R)
accelerations/qdot-rad_sec2 (R)
accelerations/rdot-rad_sec2 (R)
accelerations/pidot-rad_sec2 (R)
accelerations/qidot-rad_sec2 (R)
accelerations/ridot-rad_sec2 (R)
accelerations/udot-ft_sec2 (R)
accelerations/vdot-ft_sec2 (R)
accelerations/wdot-ft_sec2 (R)
accelerations/uidot-ft_sec2 (R)
accelerations/vidot-ft_sec2 (R)
accelerations/widot-ft_sec2 (R)
accelerations/gravity-ft_sec2 (R)
forces/load-factor (R)
forces/fbx-aero-lbs (R)
forces/fby-aero-lbs (R)
forces/fbz-aero-lbs (R)
forces/fwx-aero-lbs (R)
forces/fwy-aero-lbs (R)
forces/fwz-aero-lbs (R)
forces/fsx-aero-lbs (R)
forces/fsy-aero-lbs (R)
forces/fsz-aero-lbs (R)
forces/lod-norm (R)
forces/fbx-weight-lbs (R)
forces/fby-weight-lbs (R)
forces/fbz-weight-lbs (R)
forces/fbx-total-lbs (R)
forces/fby-total-lbs (R)
forces/fbz-total-lbs (R)
forces/fbx-gear-lbs (R)
forces/fby-gear-lbs (R)
forces/fbz-gear-lbs (R)
forces/hold-down (RW)
forces/fbx-prop-lbs (R)
forces/fby-prop-lbs (R)
forces/fbz-prop-lbs (R)
aero/alpha-rad (R)
aero/beta-rad (R)
aero/mag-beta-rad (R)
aero/alpha-deg (R)
aero/beta-deg (R)
aero/mag-beta-deg (R)
aero/Re (R)
aero/qbar-psf (R)
aero/qbarUW-psf (R)
aero/qbarUV-psf (R)
aero/alphadot-rad_sec (R)
aero/betadot-rad_sec (R)
aero/alphadot-deg_sec (R)
aero/betadot-deg_sec (R)
aero/h_b-cg-ft (R)
aero/h_b-mac-ft (R)
aero/cl-squared (R)
aero/qbar-area (RW)
aero/alpha-max-rad (RW)
aero/alpha-min-rad (RW)
aero/bi2vel (R)
aero/ci2vel (R)
aero/alpha-wing-rad (R)
aero/stall-hyst-norm (R)
aero/function/kCLge (R)
aero/function/kCDge (R)
aero/function/kCmge (R)
aero/coefficient/CDalpha (R)
aero/coefficient/CDi (R)
aero/coefficient/CDbeta (R)
aero/coefficient/CDDe (R)
aero/coefficient/CYb (R)
aero/coefficient/CYda (R)
aero/coefficient/CYdr (R)
aero/coefficient/CLalpha (R)
aero/coefficient/CLDe (R)
aero/coefficient/CLadot (R)
aero/coefficient/CLq (R)
aero/coefficient/Clb (R)
aero/coefficient/Clp (R)
aero/coefficient/Clr (R)
aero/coefficient/ClDa (R)
aero/coefficient/ClDr (R)
aero/coefficient/Cmalpha (R)
aero/coefficient/Cmadot (R)
aero/coefficient/CmM (R)
aero/coefficient/Cmq (R)
aero/coefficient/Cmde (R)
aero/coefficient/Cnb (R)
aero/coefficient/Cnp (R)
aero/coefficient/Cnr (R)
aero/coefficient/Cnda (R)
aero/coefficient/Cndr (R)
flight-path/gamma-rad (R)
flight-path/gamma-deg (R)
flight-path/psi-gt-rad (R)
moments/l-aero-lbsft (R)
moments/m-aero-lbsft (R)
moments/n-aero-lbsft (R)
moments/roll-stab-aero-lbsft (R)
moments/pitch-stab-aero-lbsft (R)
moments/yaw-stab-aero-lbsft (R)
moments/roll-wind-aero-lbsft (R)
moments/pitch-wind-aero-lbsft (R)
moments/yaw-wind-aero-lbsft (R)
moments/l-total-lbsft (R)
moments/m-total-lbsft (R)
moments/n-total-lbsft (R)
moments/l-gear-lbsft (R)
moments/m-gear-lbsft (R)
moments/n-gear-lbsft (R)
moments/l-prop-lbsft (R)
moments/m-prop-lbsft (R)
moments/n-prop-lbsft (R)
systems/stall-warn-norm (R)
ground/solid (RW)
ground/bumpiness (RW)
ground/maximum-force-lbs (RW)
ground/rolling_friction-factor (RW)
ground/static-friction-factor (RW)
ic/vc-kts (RW)
ic/ve-kts (RW)
ic/vg-kts (RW)
ic/vt-kts (RW)
ic/mach (RW)
ic/roc-fpm (RW)
ic/gamma-deg (RW)
ic/alpha-deg (RW)
ic/beta-deg (RW)
ic/theta-deg (RW)
ic/phi-deg (RW)
ic/psi-true-deg (RW)
ic/lat-gc-deg (RW)
ic/long-gc-deg (RW)
ic/h-sl-ft (RW)
ic/h-agl-ft (RW)
ic/terrain-elevation-ft (RW)
ic/vg-fps (RW)
ic/vt-fps (RW)
ic/vw-bx-fps (R)
ic/vw-by-fps (R)
ic/vw-bz-fps (R)
ic/vw-north-fps (R)
ic/vw-east-fps (R)
ic/vw-down-fps (R)
ic/vw-mag-fps (R)
ic/vw-dir-deg (RW)
ic/roc-fps (RW)
ic/u-fps (RW)
ic/v-fps (RW)
ic/w-fps (RW)
ic/vn-fps (RW)
ic/ve-fps (RW)
ic/vd-fps (RW)
ic/gamma-rad (RW)
ic/alpha-rad (RW)
ic/theta-rad (RW)
ic/beta-rad (RW)
ic/phi-rad (RW)
ic/psi-true-rad (RW)
ic/lat-gc-rad (RW)
ic/long-gc-rad (RW)
ic/p-rad_sec (RW)
ic/q-rad_sec (RW)
ic/r-rad_sec (RW)
ic/lat-geod-rad (RW)
ic/lat-geod-deg (RW)
ic/geod-alt-ft (R)
ic/targetNlf (RW)



------------------------ 
  F-15
------------------------ 

     JSBSim Flight Dynamics Model v1.1.13 [GitHub build 986/commit a09715f01b9e568ce75ca2635ba0a78ce57f7cdd] Dec  3 2022 12:36:17
            [JSBSim-ML v2.0]

JSBSim startup beginning ...

inertial/sea-level-radius_ft (R)
simulation/gravity-model (RW)
simulation/integrator/rate/rotational (RW)
simulation/integrator/rate/translational (RW)
simulation/integrator/position/rotational (RW)
simulation/integrator/position/translational (RW)
simulation/write-state-file (W)
simulation/channel-dt (R)
simulation/gravitational-torque (RW)
simulation/force-output (W)
simulation/do_simple_trim (W)
simulation/reset (W)
simulation/disperse (R)
simulation/randomseed (RW)
simulation/terminate (RW)
simulation/pause (RW)
simulation/sim-time-sec (R)
simulation/dt (R)
simulation/jsbsim-debug (RW)
simulation/frame (RW)
simulation/trim-completed (RW)
velocities/h-dot-fps (R)
velocities/v-north-fps (R)
velocities/v-east-fps (R)
velocities/v-down-fps (R)
velocities/u-fps (R)
velocities/v-fps (R)
velocities/w-fps (R)
velocities/p-rad_sec (R)
velocities/q-rad_sec (R)
velocities/r-rad_sec (R)
velocities/pi-rad_sec (R)
velocities/qi-rad_sec (R)
velocities/ri-rad_sec (R)
velocities/eci-x-fps (R)
velocities/eci-y-fps (R)
velocities/eci-z-fps (R)
velocities/eci-velocity-mag-fps (R)
velocities/ned-velocity-mag-fps (R)
velocities/vc-fps (R)
velocities/vc-kts (R)
velocities/ve-fps (R)
velocities/ve-kts (R)
velocities/vtrue-fps (R)
velocities/vtrue-kts (R)
velocities/machU (R)
velocities/p-aero-rad_sec (R)
velocities/q-aero-rad_sec (R)
velocities/r-aero-rad_sec (R)
velocities/phidot-rad_sec (R)
velocities/thetadot-rad_sec (R)
velocities/psidot-rad_sec (R)
velocities/u-aero-fps (R)
velocities/v-aero-fps (R)
velocities/w-aero-fps (R)
velocities/vt-fps (R)
velocities/mach (R)
velocities/vg-fps (R)
position/h-sl-ft (RW)
position/h-sl-meters (RW)
position/lat-gc-rad (RW)
position/long-gc-rad (RW)
position/lat-gc-deg (RW)
position/long-gc-deg (RW)
position/lat-geod-rad (R)
position/lat-geod-deg (R)
position/geod-alt-ft (R)
position/h-agl-ft (RW)
position/geod-alt-km (R)
position/h-agl-km (RW)
position/radius-to-vehicle-ft (R)
position/terrain-elevation-asl-ft (RW)
position/eci-x-ft (R)
position/eci-y-ft (R)
position/eci-z-ft (R)
position/ecef-x-ft (R)
position/ecef-y-ft (R)
position/ecef-z-ft (R)
position/epa-rad (R)
position/distance-from-start-lon-mt (R)
position/distance-from-start-lat-mt (R)
position/distance-from-start-mag-mt (R)
position/vrp-gc-latitude_deg (R)
position/vrp-longitude_deg (R)
position/vrp-radius-ft (R)
metrics/terrain-radius (R)
metrics/Sw-sqft (RW)
metrics/bw-ft (R)
metrics/cbarw-ft (R)
metrics/iw-rad (R)
metrics/iw-deg (R)
metrics/Sh-sqft (R)
metrics/lh-ft (R)
metrics/Sv-sqft (R)
metrics/lv-ft (R)
metrics/lh-norm (R)
metrics/lv-norm (R)
metrics/vbarh-norm (R)
metrics/vbarv-norm (R)
metrics/aero-rp-x-in (RW)
metrics/aero-rp-y-in (RW)
metrics/aero-rp-z-in (RW)
metrics/eyepoint-x-in (R)
metrics/eyepoint-y-in (R)
metrics/eyepoint-z-in (R)
metrics/visualrefpoint-x-in (R)
metrics/visualrefpoint-y-in (R)
metrics/visualrefpoint-z-in (R)
attitude/phi-rad (R)
attitude/theta-rad (R)
attitude/psi-rad (R)
attitude/phi-deg (R)
attitude/theta-deg (R)
attitude/psi-deg (R)
attitude/roll-rad (R)
attitude/pitch-rad (R)
attitude/heading-true-rad (R)
atmosphere/T-R (R)
atmosphere/rho-slugs_ft3 (R)
atmosphere/P-psf (R)
atmosphere/a-fps (R)
atmosphere/T-sl-R (R)
atmosphere/rho-sl-slugs_ft3 (R)
atmosphere/a-sl-fps (R)
atmosphere/theta (R)
atmosphere/sigma (R)
atmosphere/delta (R)
atmosphere/a-ratio (R)
atmosphere/density-altitude (R)
atmosphere/pressure-altitude (R)
atmosphere/delta-T (RW)
atmosphere/SL-graded-delta-T (RW)
atmosphere/P-sl-psf (RW)
atmosphere/dew-point-R (RW)
atmosphere/vapor-pressure-psf (RW)
atmosphere/saturated-vapor-pressure-psf (R)
atmosphere/RH (RW)
atmosphere/vapor-fraction-ppm (RW)
atmosphere/psiw-rad (RW)
atmosphere/wind-north-fps (RW)
atmosphere/wind-east-fps (RW)
atmosphere/wind-down-fps (RW)
atmosphere/wind-mag-fps (RW)
atmosphere/gust-north-fps (RW)
atmosphere/gust-east-fps (RW)
atmosphere/gust-down-fps (RW)
atmosphere/cosine-gust/startup-duration-sec (W)
atmosphere/cosine-gust/steady-duration-sec (W)
atmosphere/cosine-gust/end-duration-sec (W)
atmosphere/cosine-gust/magnitude-ft_sec (W)
atmosphere/cosine-gust/frame (W)
atmosphere/cosine-gust/X-velocity-ft_sec (W)
atmosphere/cosine-gust/Y-velocity-ft_sec (W)
atmosphere/cosine-gust/Z-velocity-ft_sec (W)
atmosphere/cosine-gust/start (W)
atmosphere/updownburst/number-of-cells (W)
atmosphere/turb-north-fps (RW)
atmosphere/turb-east-fps (RW)
atmosphere/turb-down-fps (RW)
atmosphere/p-turb-rad_sec (R)
atmosphere/q-turb-rad_sec (R)
atmosphere/r-turb-rad_sec (R)
atmosphere/turb-type (RW)
atmosphere/turb-rate (RW)
atmosphere/turb-gain (RW)
atmosphere/turb-rhythmicity (RW)
atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps (RW)
atmosphere/turbulence/milspec/severity (RW)
atmosphere/total-wind-north-fps (R)
atmosphere/total-wind-east-fps (R)
atmosphere/total-wind-down-fps (R)
fcs/aileron-cmd-norm (RW)
fcs/elevator-cmd-norm (RW)
fcs/rudder-cmd-norm (RW)
fcs/flap-cmd-norm (RW)
fcs/speedbrake-cmd-norm (RW)
fcs/spoiler-cmd-norm (RW)
fcs/pitch-trim-cmd-norm (RW)
fcs/roll-trim-cmd-norm (RW)
fcs/yaw-trim-cmd-norm (RW)
fcs/left-aileron-pos-rad (RW)
fcs/left-aileron-pos-deg (RW)
fcs/left-aileron-pos-norm (RW)
fcs/mag-left-aileron-pos-rad (R)
fcs/right-aileron-pos-rad (RW)
fcs/right-aileron-pos-deg (RW)
fcs/right-aileron-pos-norm (RW)
fcs/mag-right-aileron-pos-rad (R)
fcs/elevator-pos-rad (RW)
fcs/elevator-pos-deg (RW)
fcs/elevator-pos-norm (RW)
fcs/mag-elevator-pos-rad (R)
fcs/rudder-pos-rad (RW)
fcs/rudder-pos-deg (RW)
fcs/rudder-pos-norm (RW)
fcs/mag-rudder-pos-rad (R)
fcs/flap-pos-rad (RW)
fcs/flap-pos-deg (RW)
fcs/flap-pos-norm (RW)
fcs/speedbrake-pos-rad (RW)
fcs/speedbrake-pos-deg (RW)
fcs/speedbrake-pos-norm (RW)
fcs/mag-speedbrake-pos-rad (R)
fcs/spoiler-pos-rad (RW)
fcs/spoiler-pos-deg (RW)
fcs/spoiler-pos-norm (RW)
fcs/mag-spoiler-pos-rad (R)
fcs/left-brake-cmd-norm (RW)
fcs/right-brake-cmd-norm (RW)
fcs/center-brake-cmd-norm (RW)
fcs/wing-fold-pos-norm (RW)
fcs/steer-cmd-norm (RW)
fcs/steer-pos-deg (RW)
fcs/throttle-cmd-norm (RW)
fcs/throttle-pos-norm (RW)
fcs/mixture-cmd-norm (RW)
fcs/mixture-pos-norm (RW)
fcs/advance-cmd-norm (RW)
fcs/advance-pos-norm (RW)
fcs/feather-cmd-norm (RW)
fcs/feather-pos-norm (RW)
fcs/throttle-cmd-norm[1] (RW)
fcs/throttle-pos-norm[1] (RW)
fcs/mixture-cmd-norm[1] (RW)
fcs/mixture-pos-norm[1] (RW)
fcs/advance-cmd-norm[1] (RW)
fcs/advance-pos-norm[1] (RW)
fcs/feather-cmd-norm[1] (RW)
fcs/feather-pos-norm[1] (RW)
fcs/pitch-trim-sum (RW)
fcs/elevator-position (RW)
fcs/roll-trim-sum (RW)
fcs/aileron-pos-norm (RW)
fcs/left-aileron-control (RW)
fcs/right-aileron-control (RW)
fcs/yaw-damper-gain (RW)
fcs/yaw-trim-sum (RW)
fcs/rudder-position (RW)
fcs/rudder-control (RW)
fcs/gear-control (RW)
fcs/speedbrake-control (RW)
gear/gear-pos-norm (RW)
gear/gear-cmd-norm (RW)
gear/tailhook-pos-norm (RW)
gear/num-units (R)
gear/wow (R)
gear/unit/solid (RW)
gear/unit/bumpiness (RW)
gear/unit/maximum-force-lbs (RW)
gear/unit/rolling_friction-factor (RW)
gear/unit/static-friction-factor (RW)
gear/unit/WOW (RW)
gear/unit/x-position (RW)
gear/unit/y-position (RW)
gear/unit/z-position (RW)
gear/unit/compression-ft (RW)
gear/unit/compression-velocity-fps (RW)
gear/unit/static_friction_coeff (RW)
gear/unit/dynamic_friction_coeff (RW)
gear/unit/slip-angle-deg (RW)
gear/unit/wheel-speed-fps (R)
gear/unit/side_friction_coeff (RW)
gear/unit/rolling_friction_coeff (RW)
gear/unit/pos-norm (RW)
gear/unit[1]/solid (RW)
gear/unit[1]/bumpiness (RW)
gear/unit[1]/maximum-force-lbs (RW)
gear/unit[1]/rolling_friction-factor (RW)
gear/unit[1]/static-friction-factor (RW)
gear/unit[1]/WOW (RW)
gear/unit[1]/x-position (RW)
gear/unit[1]/y-position (RW)
gear/unit[1]/z-position (RW)
gear/unit[1]/compression-ft (RW)
gear/unit[1]/compression-velocity-fps (RW)
gear/unit[1]/static_friction_coeff (RW)
gear/unit[1]/dynamic_friction_coeff (RW)
gear/unit[1]/slip-angle-deg (RW)
gear/unit[1]/wheel-speed-fps (R)
gear/unit[1]/side_friction_coeff (RW)
gear/unit[1]/rolling_friction_coeff (RW)
gear/unit[1]/pos-norm (RW)
gear/unit[2]/solid (RW)
gear/unit[2]/bumpiness (RW)
gear/unit[2]/maximum-force-lbs (RW)
gear/unit[2]/rolling_friction-factor (RW)
gear/unit[2]/static-friction-factor (RW)
gear/unit[2]/WOW (RW)
gear/unit[2]/x-position (RW)
gear/unit[2]/y-position (RW)
gear/unit[2]/z-position (RW)
gear/unit[2]/compression-ft (RW)
gear/unit[2]/compression-velocity-fps (RW)
gear/unit[2]/static_friction_coeff (RW)
gear/unit[2]/dynamic_friction_coeff (RW)
gear/unit[2]/slip-angle-deg (RW)
gear/unit[2]/wheel-speed-fps (R)
gear/unit[2]/side_friction_coeff (RW)
gear/unit[2]/rolling_friction_coeff (RW)
gear/unit[2]/pos-norm (RW)
gear/unit[3]/solid (RW)
gear/unit[3]/bumpiness (RW)
gear/unit[3]/maximum-force-lbs (RW)
gear/unit[3]/rolling_friction-factor (RW)
gear/unit[3]/static-friction-factor (RW)
gear/unit[3]/WOW (RW)
gear/unit[3]/x-position (RW)
gear/unit[3]/y-position (RW)
gear/unit[3]/z-position (RW)
gear/unit[3]/compression-ft (RW)
gear/unit[3]/compression-velocity-fps (RW)
gear/unit[3]/static_friction_coeff (RW)
gear/unit[3]/dynamic_friction_coeff (RW)
gear/unit[3]/slip-angle-deg (RW)
gear/unit[3]/wheel-speed-fps (R)
gear/unit[3]/side_friction_coeff (RW)
gear/unit[3]/rolling_friction_coeff (RW)
gear/unit[4]/solid (RW)
gear/unit[4]/bumpiness (RW)
gear/unit[4]/maximum-force-lbs (RW)
gear/unit[4]/rolling_friction-factor (RW)
gear/unit[4]/static-friction-factor (RW)
gear/unit[4]/WOW (RW)
gear/unit[4]/x-position (RW)
gear/unit[4]/y-position (RW)
gear/unit[4]/z-position (RW)
gear/unit[4]/compression-ft (RW)
gear/unit[4]/compression-velocity-fps (RW)
gear/unit[4]/static_friction_coeff (RW)
gear/unit[4]/dynamic_friction_coeff (RW)
gear/unit[4]/slip-angle-deg (RW)
gear/unit[4]/wheel-speed-fps (R)
gear/unit[4]/side_friction_coeff (RW)
gear/unit[4]/rolling_friction_coeff (RW)
gear/unit[5]/solid (RW)
gear/unit[5]/bumpiness (RW)
gear/unit[5]/maximum-force-lbs (RW)
gear/unit[5]/rolling_friction-factor (RW)
gear/unit[5]/static-friction-factor (RW)
gear/unit[5]/WOW (RW)
gear/unit[5]/x-position (RW)
gear/unit[5]/y-position (RW)
gear/unit[5]/z-position (RW)
gear/unit[5]/compression-ft (RW)
gear/unit[5]/compression-velocity-fps (RW)
gear/unit[5]/static_friction_coeff (RW)
gear/unit[5]/dynamic_friction_coeff (RW)
gear/unit[5]/slip-angle-deg (RW)
gear/unit[5]/wheel-speed-fps (R)
gear/unit[5]/side_friction_coeff (RW)
gear/unit[5]/rolling_friction_coeff (RW)
gear/unit[6]/solid (RW)
gear/unit[6]/bumpiness (RW)
gear/unit[6]/maximum-force-lbs (RW)
gear/unit[6]/rolling_friction-factor (RW)
gear/unit[6]/static-friction-factor (RW)
gear/unit[6]/WOW (RW)
gear/unit[6]/x-position (RW)
gear/unit[6]/y-position (RW)
gear/unit[6]/z-position (RW)
gear/unit[6]/compression-ft (RW)
gear/unit[6]/compression-velocity-fps (RW)
gear/unit[6]/static_friction_coeff (RW)
gear/unit[6]/dynamic_friction_coeff (RW)
gear/unit[6]/slip-angle-deg (RW)
gear/unit[6]/wheel-speed-fps (R)
gear/unit[6]/side_friction_coeff (RW)
gear/unit[6]/rolling_friction_coeff (RW)
gear/unit[7]/solid (RW)
gear/unit[7]/bumpiness (RW)
gear/unit[7]/maximum-force-lbs (RW)
gear/unit[7]/rolling_friction-factor (RW)
gear/unit[7]/static-friction-factor (RW)
gear/unit[7]/WOW (RW)
gear/unit[7]/x-position (RW)
gear/unit[7]/y-position (RW)
gear/unit[7]/z-position (RW)
gear/unit[7]/compression-ft (RW)
gear/unit[7]/compression-velocity-fps (RW)
gear/unit[7]/static_friction_coeff (RW)
gear/unit[7]/dynamic_friction_coeff (RW)
gear/unit[7]/slip-angle-deg (RW)
gear/unit[7]/wheel-speed-fps (R)
gear/unit[7]/side_friction_coeff (RW)
gear/unit[7]/rolling_friction_coeff (RW)
gear/unit[8]/solid (RW)
gear/unit[8]/bumpiness (RW)
gear/unit[8]/maximum-force-lbs (RW)
gear/unit[8]/rolling_friction-factor (RW)
gear/unit[8]/static-friction-factor (RW)
gear/unit[8]/WOW (RW)
gear/unit[8]/x-position (RW)
gear/unit[8]/y-position (RW)
gear/unit[8]/z-position (RW)
gear/unit[8]/compression-ft (RW)
gear/unit[8]/compression-velocity-fps (RW)
gear/unit[8]/static_friction_coeff (RW)
gear/unit[8]/dynamic_friction_coeff (RW)
gear/unit[8]/slip-angle-deg (RW)
gear/unit[8]/wheel-speed-fps (R)
gear/unit[8]/side_friction_coeff (RW)
gear/unit[8]/rolling_friction_coeff (RW)
inertia/mass-slugs (R)
inertia/weight-lbs (R)
inertia/empty-weight-lbs (R)
inertia/cg-x-in (R)
inertia/cg-y-in (R)
inertia/cg-z-in (R)
inertia/ixx-slugs_ft2 (R)
inertia/iyy-slugs_ft2 (R)
inertia/izz-slugs_ft2 (R)
inertia/ixy-slugs_ft2 (R)
inertia/ixz-slugs_ft2 (R)
inertia/iyz-slugs_ft2 (R)
inertia/print-mass-properties (W)
inertia/pointmass-weight-lbs (RW)
inertia/pointmass-location-X-inches (RW)
inertia/pointmass-location-Y-inches (RW)
inertia/pointmass-location-Z-inches (RW)
propulsion/tat-r (R)
propulsion/tat-c (R)
propulsion/pt-lbs_sqft (R)
propulsion/tank/contents-lbs (RW)
propulsion/tank/unusable-volume-gal (RW)
propulsion/tank/pct-full (R)
propulsion/tank/density-lbs_per_gal (R)
propulsion/tank/priority (RW)
propulsion/tank/external-flow-rate-pps (RW)
propulsion/tank/local-ixx-slug_ft2 (R)
propulsion/tank/local-iyy-slug_ft2 (R)
propulsion/tank/local-izz-slug_ft2 (R)
propulsion/tank/x-position (RW)
propulsion/tank/y-position (RW)
propulsion/tank/z-position (RW)
propulsion/tank[1]/contents-lbs (RW)
propulsion/tank[1]/unusable-volume-gal (RW)
propulsion/tank[1]/pct-full (R)
propulsion/tank[1]/density-lbs_per_gal (R)
propulsion/tank[1]/priority (RW)
propulsion/tank[1]/external-flow-rate-pps (RW)
propulsion/tank[1]/local-ixx-slug_ft2 (R)
propulsion/tank[1]/local-iyy-slug_ft2 (R)
propulsion/tank[1]/local-izz-slug_ft2 (R)
propulsion/tank[1]/x-position (RW)
propulsion/tank[1]/y-position (RW)
propulsion/tank[1]/z-position (RW)
propulsion/set-running (W)
propulsion/starter_cmd (RW)
propulsion/cutoff_cmd (RW)
propulsion/active_engine (RW)
propulsion/total-fuel-lbs (RW)
propulsion/total-oxidizer-lbs (RW)
propulsion/refuel (RW)
propulsion/fuel_dump (RW)
propulsion/fuel_freeze (W)
propulsion/engine/IdleThrust (R)
propulsion/engine/MilThrust (R)
propulsion/engine/AugThrust (R)
propulsion/engine/pitch-angle-rad (RW)
propulsion/engine/yaw-angle-rad (RW)
propulsion/engine/reverser-angle-rad (RW)
propulsion/engine/set-running (RW)
propulsion/engine/thrust-lbs (R)
propulsion/engine/fuel-flow-rate-pps (R)
propulsion/engine/fuel-flow-rate-gph (R)
propulsion/engine/fuel-used-lbs (R)
propulsion/engine/n1 (RW)
propulsion/engine/n2 (RW)
propulsion/engine/injection_cmd (RW)
propulsion/engine/seized (RW)
propulsion/engine/stalled (RW)
propulsion/engine/bleed-factor (RW)
propulsion/engine/MaxN1 (RW)
propulsion/engine/MaxN2 (RW)
propulsion/engine/InjectionTimer (RW)
propulsion/engine/InjWaterNorm (RW)
propulsion/engine/InjN1increment (RW)
propulsion/engine/InjN2increment (RW)
propulsion/engine[1]/IdleThrust (R)
propulsion/engine[1]/MilThrust (R)
propulsion/engine[1]/AugThrust (R)
propulsion/engine[1]/pitch-angle-rad (RW)
propulsion/engine[1]/yaw-angle-rad (RW)
propulsion/engine[1]/reverser-angle-rad (RW)
propulsion/engine[1]/set-running (RW)
propulsion/engine[1]/thrust-lbs (R)
propulsion/engine[1]/fuel-flow-rate-pps (R)
propulsion/engine[1]/fuel-flow-rate-gph (R)
propulsion/engine[1]/fuel-used-lbs (R)
propulsion/engine[1]/n1 (RW)
propulsion/engine[1]/n2 (RW)
propulsion/engine[1]/injection_cmd (RW)
propulsion/engine[1]/seized (RW)
propulsion/engine[1]/stalled (RW)
propulsion/engine[1]/bleed-factor (RW)
propulsion/engine[1]/MaxN1 (RW)
propulsion/engine[1]/MaxN2 (RW)
propulsion/engine[1]/InjectionTimer (RW)
propulsion/engine[1]/InjWaterNorm (RW)
propulsion/engine[1]/InjN1increment (RW)
propulsion/engine[1]/InjN2increment (RW)
accelerations/a-pilot-x-ft_sec2 (R)
accelerations/a-pilot-y-ft_sec2 (R)
accelerations/a-pilot-z-ft_sec2 (R)
accelerations/n-pilot-x-norm (R)
accelerations/n-pilot-y-norm (R)
accelerations/n-pilot-z-norm (R)
accelerations/Nx (R)
accelerations/Ny (R)
accelerations/Nz (R)
accelerations/pdot-rad_sec2 (R)
accelerations/qdot-rad_sec2 (R)
accelerations/rdot-rad_sec2 (R)
accelerations/pidot-rad_sec2 (R)
accelerations/qidot-rad_sec2 (R)
accelerations/ridot-rad_sec2 (R)
accelerations/udot-ft_sec2 (R)
accelerations/vdot-ft_sec2 (R)
accelerations/wdot-ft_sec2 (R)
accelerations/uidot-ft_sec2 (R)
accelerations/vidot-ft_sec2 (R)
accelerations/widot-ft_sec2 (R)
accelerations/gravity-ft_sec2 (R)
forces/load-factor (R)
forces/fbx-aero-lbs (R)
forces/fby-aero-lbs (R)
forces/fbz-aero-lbs (R)
forces/fwx-aero-lbs (R)
forces/fwy-aero-lbs (R)
forces/fwz-aero-lbs (R)
forces/fsx-aero-lbs (R)
forces/fsy-aero-lbs (R)
forces/fsz-aero-lbs (R)
forces/lod-norm (R)
forces/fbx-weight-lbs (R)
forces/fby-weight-lbs (R)
forces/fbz-weight-lbs (R)
forces/fbx-total-lbs (R)
forces/fby-total-lbs (R)
forces/fbz-total-lbs (R)
forces/fbx-gear-lbs (R)
forces/fby-gear-lbs (R)
forces/fbz-gear-lbs (R)
forces/hold-down (RW)
forces/fbx-prop-lbs (R)
forces/fby-prop-lbs (R)
forces/fbz-prop-lbs (R)
aero/alpha-rad (R)
aero/beta-rad (R)
aero/mag-beta-rad (R)
aero/alpha-deg (R)
aero/beta-deg (R)
aero/mag-beta-deg (R)
aero/Re (R)
aero/qbar-psf (R)
aero/qbarUW-psf (R)
aero/qbarUV-psf (R)
aero/alphadot-rad_sec (R)
aero/betadot-rad_sec (R)
aero/alphadot-deg_sec (R)
aero/betadot-deg_sec (R)
aero/h_b-cg-ft (R)
aero/h_b-mac-ft (R)
aero/cl-squared (R)
aero/qbar-area (RW)
aero/alpha-max-rad (RW)
aero/alpha-min-rad (RW)
aero/bi2vel (R)
aero/ci2vel (R)
aero/alpha-wing-rad (R)
aero/stall-hyst-norm (R)
aero/function/kCLge (R)
aero/function/kCDge (R)
aero/function/kCmge (R)
aero/coefficient/CDalpha (R)
aero/coefficient/CDi (R)
aero/coefficient/CDbeta (R)
aero/coefficient/CDDe (R)
aero/coefficient/CYb (R)
aero/coefficient/CYda (R)
aero/coefficient/CYdr (R)
aero/coefficient/CLalpha (R)
aero/coefficient/CLDe (R)
aero/coefficient/CLadot (R)
aero/coefficient/CLq (R)
aero/coefficient/Clb (R)
aero/coefficient/Clp (R)
aero/coefficient/Clr (R)
aero/coefficient/ClDa (R)
aero/coefficient/ClDr (R)
aero/coefficient/Cmalpha (R)
aero/coefficient/Cmadot (R)
aero/coefficient/CmM (R)
aero/coefficient/Cmq (R)
aero/coefficient/Cmde (R)
aero/coefficient/Cnb (R)
aero/coefficient/Cnp (R)
aero/coefficient/Cnr (R)
aero/coefficient/Cnda (R)
aero/coefficient/Cndr (R)
flight-path/gamma-rad (R)
flight-path/gamma-deg (R)
flight-path/psi-gt-rad (R)
moments/l-aero-lbsft (R)
moments/m-aero-lbsft (R)
moments/n-aero-lbsft (R)
moments/roll-stab-aero-lbsft (R)
moments/pitch-stab-aero-lbsft (R)
moments/yaw-stab-aero-lbsft (R)
moments/roll-wind-aero-lbsft (R)
moments/pitch-wind-aero-lbsft (R)
moments/yaw-wind-aero-lbsft (R)
moments/l-total-lbsft (R)
moments/m-total-lbsft (R)
moments/n-total-lbsft (R)
moments/l-gear-lbsft (R)
moments/m-gear-lbsft (R)
moments/n-gear-lbsft (R)
moments/l-prop-lbsft (R)
moments/m-prop-lbsft (R)
moments/n-prop-lbsft (R)
systems/stall-warn-norm (R)
ground/solid (RW)
ground/bumpiness (RW)
ground/maximum-force-lbs (RW)
ground/rolling_friction-factor (RW)
ground/static-friction-factor (RW)
ic/vc-kts (RW)
ic/ve-kts (RW)
ic/vg-kts (RW)
ic/vt-kts (RW)
ic/mach (RW)
ic/roc-fpm (RW)
ic/gamma-deg (RW)
ic/alpha-deg (RW)
ic/beta-deg (RW)
ic/theta-deg (RW)
ic/phi-deg (RW)
ic/psi-true-deg (RW)
ic/lat-gc-deg (RW)
ic/long-gc-deg (RW)
ic/h-sl-ft (RW)
ic/h-agl-ft (RW)
ic/terrain-elevation-ft (RW)
ic/vg-fps (RW)
ic/vt-fps (RW)
ic/vw-bx-fps (R)
ic/vw-by-fps (R)
ic/vw-bz-fps (R)
ic/vw-north-fps (R)
ic/vw-east-fps (R)
ic/vw-down-fps (R)
ic/vw-mag-fps (R)
ic/vw-dir-deg (RW)
ic/roc-fps (RW)
ic/u-fps (RW)
ic/v-fps (RW)
ic/w-fps (RW)
ic/vn-fps (RW)
ic/ve-fps (RW)
ic/vd-fps (RW)
ic/gamma-rad (RW)
ic/alpha-rad (RW)
ic/theta-rad (RW)
ic/beta-rad (RW)
ic/phi-rad (RW)
ic/psi-true-rad (RW)
ic/lat-gc-rad (RW)
ic/long-gc-rad (RW)
ic/p-rad_sec (RW)
ic/q-rad_sec (RW)
ic/r-rad_sec (RW)
ic/lat-geod-rad (RW)
ic/lat-geod-deg (RW)
ic/geod-alt-ft (R)
ic/targetNlf (RW)



------------------------ 
  F-15 END
------------------------ 



----------------
B747
----------------


     JSBSim Flight Dynamics Model v1.1.13 [GitHub build 986/commit a09715f01b9e568ce75ca2635ba0a78ce57f7cdd] Dec  3 2022 12:36:17
            [JSBSim-ML v2.0]

JSBSim startup beginning ...

setIC MyAircraft
inertial/sea-level-radius_ft (R)
simulation/gravity-model (RW)
simulation/integrator/rate/rotational (RW)
simulation/integrator/rate/translational (RW)
simulation/integrator/position/rotational (RW)
simulation/integrator/position/translational (RW)
simulation/write-state-file (W)
simulation/channel-dt (R)
simulation/gravitational-torque (RW)
simulation/force-output (W)
simulation/do_simple_trim (W)
simulation/reset (W)
simulation/disperse (R)
simulation/randomseed (RW)
simulation/terminate (RW)
simulation/pause (RW)
simulation/sim-time-sec (R)
simulation/dt (R)
simulation/jsbsim-debug (RW)
simulation/frame (RW)
simulation/trim-completed (RW)
velocities/h-dot-fps (R)
velocities/v-north-fps (R)
velocities/v-east-fps (R)
velocities/v-down-fps (R)
velocities/u-fps (R)
velocities/v-fps (R)
velocities/w-fps (R)
velocities/p-rad_sec (R)
velocities/q-rad_sec (R)
velocities/r-rad_sec (R)
velocities/pi-rad_sec (R)
velocities/qi-rad_sec (R)
velocities/ri-rad_sec (R)
velocities/eci-x-fps (R)
velocities/eci-y-fps (R)
velocities/eci-z-fps (R)
velocities/eci-velocity-mag-fps (R)
velocities/ned-velocity-mag-fps (R)
velocities/vc-fps (R)
velocities/vc-kts (R)
velocities/ve-fps (R)
velocities/ve-kts (R)
velocities/vtrue-fps (R)
velocities/vtrue-kts (R)
velocities/machU (R)
velocities/p-aero-rad_sec (R)
velocities/q-aero-rad_sec (R)
velocities/r-aero-rad_sec (R)
velocities/phidot-rad_sec (R)
velocities/thetadot-rad_sec (R)
velocities/psidot-rad_sec (R)
velocities/u-aero-fps (R)
velocities/v-aero-fps (R)
velocities/w-aero-fps (R)
velocities/vt-fps (R)
velocities/mach (R)
velocities/vg-fps (R)
position/h-sl-ft (RW)
position/h-sl-meters (RW)
position/lat-gc-rad (RW)
position/long-gc-rad (RW)
position/lat-gc-deg (RW)
position/long-gc-deg (RW)
position/lat-geod-rad (R)
position/lat-geod-deg (R)
position/geod-alt-ft (R)
position/h-agl-ft (RW)
position/geod-alt-km (R)
position/h-agl-km (RW)
position/radius-to-vehicle-ft (R)
position/terrain-elevation-asl-ft (RW)
position/eci-x-ft (R)
position/eci-y-ft (R)
position/eci-z-ft (R)
position/ecef-x-ft (R)
position/ecef-y-ft (R)
position/ecef-z-ft (R)
position/epa-rad (R)
position/distance-from-start-lon-mt (R)
position/distance-from-start-lat-mt (R)
position/distance-from-start-mag-mt (R)
position/vrp-gc-latitude_deg (R)
position/vrp-longitude_deg (R)
position/vrp-radius-ft (R)
metrics/terrain-radius (R)
metrics/Sw-sqft (RW)
metrics/bw-ft (R)
metrics/cbarw-ft (R)
metrics/iw-rad (R)
metrics/iw-deg (R)
metrics/Sh-sqft (R)
metrics/lh-ft (R)
metrics/Sv-sqft (R)
metrics/lv-ft (R)
metrics/lh-norm (R)
metrics/lv-norm (R)
metrics/vbarh-norm (R)
metrics/vbarv-norm (R)
metrics/aero-rp-x-in (RW)
metrics/aero-rp-y-in (RW)
metrics/aero-rp-z-in (RW)
metrics/eyepoint-x-in (R)
metrics/eyepoint-y-in (R)
metrics/eyepoint-z-in (R)
metrics/visualrefpoint-x-in (R)
metrics/visualrefpoint-y-in (R)
metrics/visualrefpoint-z-in (R)
attitude/phi-rad (R)
attitude/theta-rad (R)
attitude/psi-rad (R)
attitude/phi-deg (R)
attitude/theta-deg (R)
attitude/psi-deg (R)
attitude/roll-rad (R)
attitude/pitch-rad (R)
attitude/heading-true-rad (R)
atmosphere/T-R (R)
atmosphere/rho-slugs_ft3 (R)
atmosphere/P-psf (R)
atmosphere/a-fps (R)
atmosphere/T-sl-R (R)
atmosphere/rho-sl-slugs_ft3 (R)
atmosphere/a-sl-fps (R)
atmosphere/theta (R)
atmosphere/sigma (R)
atmosphere/delta (R)
atmosphere/a-ratio (R)
atmosphere/density-altitude (R)
atmosphere/pressure-altitude (R)
atmosphere/delta-T (RW)
atmosphere/SL-graded-delta-T (RW)
atmosphere/P-sl-psf (RW)
atmosphere/dew-point-R (RW)
atmosphere/vapor-pressure-psf (RW)
atmosphere/saturated-vapor-pressure-psf (R)
atmosphere/RH (RW)
atmosphere/vapor-fraction-ppm (RW)
atmosphere/psiw-rad (RW)
atmosphere/wind-north-fps (RW)
atmosphere/wind-east-fps (RW)
atmosphere/wind-down-fps (RW)
atmosphere/wind-mag-fps (RW)
atmosphere/gust-north-fps (RW)
atmosphere/gust-east-fps (RW)
atmosphere/gust-down-fps (RW)
atmosphere/cosine-gust/startup-duration-sec (W)
atmosphere/cosine-gust/steady-duration-sec (W)
atmosphere/cosine-gust/end-duration-sec (W)
atmosphere/cosine-gust/magnitude-ft_sec (W)
atmosphere/cosine-gust/frame (W)
atmosphere/cosine-gust/X-velocity-ft_sec (W)
atmosphere/cosine-gust/Y-velocity-ft_sec (W)
atmosphere/cosine-gust/Z-velocity-ft_sec (W)
atmosphere/cosine-gust/start (W)
atmosphere/updownburst/number-of-cells (W)
atmosphere/turb-north-fps (RW)
atmosphere/turb-east-fps (RW)
atmosphere/turb-down-fps (RW)
atmosphere/p-turb-rad_sec (R)
atmosphere/q-turb-rad_sec (R)
atmosphere/r-turb-rad_sec (R)
atmosphere/turb-type (RW)
atmosphere/turb-rate (RW)
atmosphere/turb-gain (RW)
atmosphere/turb-rhythmicity (RW)
atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps (RW)
atmosphere/turbulence/milspec/severity (RW)
atmosphere/total-wind-north-fps (R)
atmosphere/total-wind-east-fps (R)
atmosphere/total-wind-down-fps (R)
fcs/aileron-cmd-norm (RW)
fcs/elevator-cmd-norm (RW)
fcs/rudder-cmd-norm (RW)
fcs/flap-cmd-norm (RW)
fcs/speedbrake-cmd-norm (RW)
fcs/spoiler-cmd-norm (RW)
fcs/pitch-trim-cmd-norm (RW)
fcs/roll-trim-cmd-norm (RW)
fcs/yaw-trim-cmd-norm (RW)
fcs/left-aileron-pos-rad (RW)
fcs/left-aileron-pos-deg (RW)
fcs/left-aileron-pos-norm (RW)
fcs/mag-left-aileron-pos-rad (R)
fcs/right-aileron-pos-rad (RW)
fcs/right-aileron-pos-deg (RW)
fcs/right-aileron-pos-norm (RW)
fcs/mag-right-aileron-pos-rad (R)
fcs/elevator-pos-rad (RW)
fcs/elevator-pos-deg (RW)
fcs/elevator-pos-norm (RW)
fcs/mag-elevator-pos-rad (R)
fcs/rudder-pos-rad (RW)
fcs/rudder-pos-deg (RW)
fcs/rudder-pos-norm (RW)
fcs/mag-rudder-pos-rad (R)
fcs/flap-pos-rad (RW)
fcs/flap-pos-deg (RW)
fcs/flap-pos-norm (RW)
fcs/speedbrake-pos-rad (RW)
fcs/speedbrake-pos-deg (RW)
fcs/speedbrake-pos-norm (RW)
fcs/mag-speedbrake-pos-rad (R)
fcs/spoiler-pos-rad (RW)
fcs/spoiler-pos-deg (RW)
fcs/spoiler-pos-norm (RW)
fcs/mag-spoiler-pos-rad (R)
fcs/left-brake-cmd-norm (RW)
fcs/right-brake-cmd-norm (RW)
fcs/center-brake-cmd-norm (RW)
fcs/wing-fold-pos-norm (RW)
fcs/steer-cmd-norm (RW)
fcs/steer-pos-deg (RW)
fcs/throttle-cmd-norm (RW)
fcs/throttle-pos-norm (RW)
fcs/mixture-cmd-norm (RW)
fcs/mixture-pos-norm (RW)
fcs/advance-cmd-norm (RW)
fcs/advance-pos-norm (RW)
fcs/feather-cmd-norm (RW)
fcs/feather-pos-norm (RW)
fcs/throttle-cmd-norm[1] (RW)
fcs/throttle-pos-norm[1] (RW)
fcs/mixture-cmd-norm[1] (RW)
fcs/mixture-pos-norm[1] (RW)
fcs/advance-cmd-norm[1] (RW)
fcs/advance-pos-norm[1] (RW)
fcs/feather-cmd-norm[1] (RW)
fcs/feather-pos-norm[1] (RW)
fcs/throttle-cmd-norm[2] (RW)
fcs/throttle-pos-norm[2] (RW)
fcs/mixture-cmd-norm[2] (RW)
fcs/mixture-pos-norm[2] (RW)
fcs/advance-cmd-norm[2] (RW)
fcs/advance-pos-norm[2] (RW)
fcs/feather-cmd-norm[2] (RW)
fcs/feather-pos-norm[2] (RW)
fcs/throttle-cmd-norm[3] (RW)
fcs/throttle-pos-norm[3] (RW)
fcs/mixture-cmd-norm[3] (RW)
fcs/mixture-pos-norm[3] (RW)
fcs/advance-cmd-norm[3] (RW)
fcs/advance-pos-norm[3] (RW)
fcs/feather-cmd-norm[3] (RW)
fcs/feather-pos-norm[3] (RW)
fcs/pitch-trim-sum (RW)
fcs/elevator-control (RW)
fcs/roll-trim-sum (RW)
fcs/left-aileron-control (RW)
fcs/right-aileron-control (RW)
fcs/rudder-command-sum (RW)
fcs/yaw-damper-rate (RW)
fcs/yaw-damper-beta (RW)
fcs/yaw-damper-sum (RW)
fcs/yaw-damper-final (RW)
fcs/rudder-sum (RW)
fcs/rudder-control (RW)
fcs/flaps-control (RW)
fcs/gear-control (RW)
fcs/speedbrake-control (RW)
gear/gear-pos-norm (RW)
gear/gear-cmd-norm (RW)
gear/tailhook-pos-norm (RW)
gear/num-units (R)
gear/wow (R)
gear/unit/solid (RW)
gear/unit/bumpiness (RW)
gear/unit/maximum-force-lbs (RW)
gear/unit/rolling_friction-factor (RW)
gear/unit/static-friction-factor (RW)
gear/unit/WOW (RW)
gear/unit/x-position (RW)
gear/unit/y-position (RW)
gear/unit/z-position (RW)
gear/unit/compression-ft (RW)
gear/unit/compression-velocity-fps (RW)
gear/unit/static_friction_coeff (RW)
gear/unit/dynamic_friction_coeff (RW)
gear/unit/slip-angle-deg (RW)
gear/unit/wheel-speed-fps (R)
gear/unit/side_friction_coeff (RW)
gear/unit/rolling_friction_coeff (RW)
gear/unit/pos-norm (RW)
gear/unit[1]/solid (RW)
gear/unit[1]/bumpiness (RW)
gear/unit[1]/maximum-force-lbs (RW)
gear/unit[1]/rolling_friction-factor (RW)
gear/unit[1]/static-friction-factor (RW)
gear/unit[1]/WOW (RW)
gear/unit[1]/x-position (RW)
gear/unit[1]/y-position (RW)
gear/unit[1]/z-position (RW)
gear/unit[1]/compression-ft (RW)
gear/unit[1]/compression-velocity-fps (RW)
gear/unit[1]/static_friction_coeff (RW)
gear/unit[1]/dynamic_friction_coeff (RW)
gear/unit[1]/slip-angle-deg (RW)
gear/unit[1]/wheel-speed-fps (R)
gear/unit[1]/side_friction_coeff (RW)
gear/unit[1]/rolling_friction_coeff (RW)
gear/unit[1]/pos-norm (RW)
gear/unit[2]/solid (RW)
gear/unit[2]/bumpiness (RW)
gear/unit[2]/maximum-force-lbs (RW)
gear/unit[2]/rolling_friction-factor (RW)
gear/unit[2]/static-friction-factor (RW)
gear/unit[2]/WOW (RW)
gear/unit[2]/x-position (RW)
gear/unit[2]/y-position (RW)
gear/unit[2]/z-position (RW)
gear/unit[2]/compression-ft (RW)
gear/unit[2]/compression-velocity-fps (RW)
gear/unit[2]/static_friction_coeff (RW)
gear/unit[2]/dynamic_friction_coeff (RW)
gear/unit[2]/slip-angle-deg (RW)
gear/unit[2]/wheel-speed-fps (R)
gear/unit[2]/side_friction_coeff (RW)
gear/unit[2]/rolling_friction_coeff (RW)
gear/unit[2]/pos-norm (RW)
inertia/mass-slugs (R)
inertia/weight-lbs (R)
inertia/empty-weight-lbs (R)
inertia/cg-x-in (R)
inertia/cg-y-in (R)
inertia/cg-z-in (R)
inertia/ixx-slugs_ft2 (R)
inertia/iyy-slugs_ft2 (R)
inertia/izz-slugs_ft2 (R)
inertia/ixy-slugs_ft2 (R)
inertia/ixz-slugs_ft2 (R)
inertia/iyz-slugs_ft2 (R)
inertia/print-mass-properties (W)
propulsion/tat-r (R)
propulsion/tat-c (R)
propulsion/pt-lbs_sqft (R)
propulsion/tank/contents-lbs (RW)
propulsion/tank/unusable-volume-gal (RW)
propulsion/tank/pct-full (R)
propulsion/tank/density-lbs_per_gal (R)
propulsion/tank/priority (RW)
propulsion/tank/external-flow-rate-pps (RW)
propulsion/tank/local-ixx-slug_ft2 (R)
propulsion/tank/local-iyy-slug_ft2 (R)
propulsion/tank/local-izz-slug_ft2 (R)
propulsion/tank/x-position (RW)
propulsion/tank/y-position (RW)
propulsion/tank/z-position (RW)
propulsion/tank[1]/contents-lbs (RW)
propulsion/tank[1]/unusable-volume-gal (RW)
propulsion/tank[1]/pct-full (R)
propulsion/tank[1]/density-lbs_per_gal (R)
propulsion/tank[1]/priority (RW)
propulsion/tank[1]/external-flow-rate-pps (RW)
propulsion/tank[1]/local-ixx-slug_ft2 (R)
propulsion/tank[1]/local-iyy-slug_ft2 (R)
propulsion/tank[1]/local-izz-slug_ft2 (R)
propulsion/tank[1]/x-position (RW)
propulsion/tank[1]/y-position (RW)
propulsion/tank[1]/z-position (RW)
propulsion/tank[2]/contents-lbs (RW)
propulsion/tank[2]/unusable-volume-gal (RW)
propulsion/tank[2]/pct-full (R)
propulsion/tank[2]/density-lbs_per_gal (R)
propulsion/tank[2]/priority (RW)
propulsion/tank[2]/external-flow-rate-pps (RW)
propulsion/tank[2]/local-ixx-slug_ft2 (R)
propulsion/tank[2]/local-iyy-slug_ft2 (R)
propulsion/tank[2]/local-izz-slug_ft2 (R)
propulsion/tank[2]/x-position (RW)
propulsion/tank[2]/y-position (RW)
propulsion/tank[2]/z-position (RW)
propulsion/tank[3]/contents-lbs (RW)
propulsion/tank[3]/unusable-volume-gal (RW)
propulsion/tank[3]/pct-full (R)
propulsion/tank[3]/density-lbs_per_gal (R)
propulsion/tank[3]/priority (RW)
propulsion/tank[3]/external-flow-rate-pps (RW)
propulsion/tank[3]/local-ixx-slug_ft2 (R)
propulsion/tank[3]/local-iyy-slug_ft2 (R)
propulsion/tank[3]/local-izz-slug_ft2 (R)
propulsion/tank[3]/x-position (RW)
propulsion/tank[3]/y-position (RW)
propulsion/tank[3]/z-position (RW)
propulsion/tank[4]/contents-lbs (RW)
propulsion/tank[4]/unusable-volume-gal (RW)
propulsion/tank[4]/pct-full (R)
propulsion/tank[4]/density-lbs_per_gal (R)
propulsion/tank[4]/priority (RW)
propulsion/tank[4]/external-flow-rate-pps (RW)
propulsion/tank[4]/local-ixx-slug_ft2 (R)
propulsion/tank[4]/local-iyy-slug_ft2 (R)
propulsion/tank[4]/local-izz-slug_ft2 (R)
propulsion/tank[4]/x-position (RW)
propulsion/tank[4]/y-position (RW)
propulsion/tank[4]/z-position (RW)
propulsion/set-running (W)
propulsion/starter_cmd (RW)
propulsion/cutoff_cmd (RW)
propulsion/active_engine (RW)
propulsion/total-fuel-lbs (RW)
propulsion/total-oxidizer-lbs (RW)
propulsion/refuel (RW)
propulsion/fuel_dump (RW)
propulsion/fuel_freeze (W)
propulsion/engine/IdleThrust (R)
propulsion/engine/MilThrust (R)
propulsion/engine/pitch-angle-rad (RW)
propulsion/engine/yaw-angle-rad (RW)
propulsion/engine/reverser-angle-rad (RW)
propulsion/engine/set-running (RW)
propulsion/engine/thrust-lbs (R)
propulsion/engine/fuel-flow-rate-pps (R)
propulsion/engine/fuel-flow-rate-gph (R)
propulsion/engine/fuel-used-lbs (R)
propulsion/engine/n1 (RW)
propulsion/engine/n2 (RW)
propulsion/engine/injection_cmd (RW)
propulsion/engine/seized (RW)
propulsion/engine/stalled (RW)
propulsion/engine/bleed-factor (RW)
propulsion/engine/MaxN1 (RW)
propulsion/engine/MaxN2 (RW)
propulsion/engine/InjectionTimer (RW)
propulsion/engine/InjWaterNorm (RW)
propulsion/engine/InjN1increment (RW)
propulsion/engine/InjN2increment (RW)
propulsion/engine[1]/IdleThrust (R)
propulsion/engine[1]/MilThrust (R)
propulsion/engine[1]/pitch-angle-rad (RW)
propulsion/engine[1]/yaw-angle-rad (RW)
propulsion/engine[1]/reverser-angle-rad (RW)
propulsion/engine[1]/set-running (RW)
propulsion/engine[1]/thrust-lbs (R)
propulsion/engine[1]/fuel-flow-rate-pps (R)
propulsion/engine[1]/fuel-flow-rate-gph (R)
propulsion/engine[1]/fuel-used-lbs (R)
propulsion/engine[1]/n1 (RW)
propulsion/engine[1]/n2 (RW)
propulsion/engine[1]/injection_cmd (RW)
propulsion/engine[1]/seized (RW)
propulsion/engine[1]/stalled (RW)
propulsion/engine[1]/bleed-factor (RW)
propulsion/engine[1]/MaxN1 (RW)
propulsion/engine[1]/MaxN2 (RW)
propulsion/engine[1]/InjectionTimer (RW)
propulsion/engine[1]/InjWaterNorm (RW)
propulsion/engine[1]/InjN1increment (RW)
propulsion/engine[1]/InjN2increment (RW)
propulsion/engine[2]/IdleThrust (R)
propulsion/engine[2]/MilThrust (R)
propulsion/engine[2]/pitch-angle-rad (RW)
propulsion/engine[2]/yaw-angle-rad (RW)
propulsion/engine[2]/reverser-angle-rad (RW)
propulsion/engine[2]/set-running (RW)
propulsion/engine[2]/thrust-lbs (R)
propulsion/engine[2]/fuel-flow-rate-pps (R)
propulsion/engine[2]/fuel-flow-rate-gph (R)
propulsion/engine[2]/fuel-used-lbs (R)
propulsion/engine[2]/n1 (RW)
propulsion/engine[2]/n2 (RW)
propulsion/engine[2]/injection_cmd (RW)
propulsion/engine[2]/seized (RW)
propulsion/engine[2]/stalled (RW)
propulsion/engine[2]/bleed-factor (RW)
propulsion/engine[2]/MaxN1 (RW)
propulsion/engine[2]/MaxN2 (RW)
propulsion/engine[2]/InjectionTimer (RW)
propulsion/engine[2]/InjWaterNorm (RW)
propulsion/engine[2]/InjN1increment (RW)
propulsion/engine[2]/InjN2increment (RW)
propulsion/engine[3]/IdleThrust (R)
propulsion/engine[3]/MilThrust (R)
propulsion/engine[3]/pitch-angle-rad (RW)
propulsion/engine[3]/yaw-angle-rad (RW)
propulsion/engine[3]/reverser-angle-rad (RW)
propulsion/engine[3]/set-running (RW)
propulsion/engine[3]/thrust-lbs (R)
propulsion/engine[3]/fuel-flow-rate-pps (R)
propulsion/engine[3]/fuel-flow-rate-gph (R)
propulsion/engine[3]/fuel-used-lbs (R)
propulsion/engine[3]/n1 (RW)
propulsion/engine[3]/n2 (RW)
propulsion/engine[3]/injection_cmd (RW)
propulsion/engine[3]/seized (RW)
propulsion/engine[3]/stalled (RW)
propulsion/engine[3]/bleed-factor (RW)
propulsion/engine[3]/MaxN1 (RW)
propulsion/engine[3]/MaxN2 (RW)
propulsion/engine[3]/InjectionTimer (RW)
propulsion/engine[3]/InjWaterNorm (RW)
propulsion/engine[3]/InjN1increment (RW)
propulsion/engine[3]/InjN2increment (RW)
accelerations/a-pilot-x-ft_sec2 (R)
accelerations/a-pilot-y-ft_sec2 (R)
accelerations/a-pilot-z-ft_sec2 (R)
accelerations/n-pilot-x-norm (R)
accelerations/n-pilot-y-norm (R)
accelerations/n-pilot-z-norm (R)
accelerations/Nx (R)
accelerations/Ny (R)
accelerations/Nz (R)
accelerations/pdot-rad_sec2 (R)
accelerations/qdot-rad_sec2 (R)
accelerations/rdot-rad_sec2 (R)
accelerations/pidot-rad_sec2 (R)
accelerations/qidot-rad_sec2 (R)
accelerations/ridot-rad_sec2 (R)
accelerations/udot-ft_sec2 (R)
accelerations/vdot-ft_sec2 (R)
accelerations/wdot-ft_sec2 (R)
accelerations/uidot-ft_sec2 (R)
accelerations/vidot-ft_sec2 (R)
accelerations/widot-ft_sec2 (R)
accelerations/gravity-ft_sec2 (R)
forces/load-factor (R)
forces/fbx-aero-lbs (R)
forces/fby-aero-lbs (R)
forces/fbz-aero-lbs (R)
forces/fwx-aero-lbs (R)
forces/fwy-aero-lbs (R)
forces/fwz-aero-lbs (R)
forces/fsx-aero-lbs (R)
forces/fsy-aero-lbs (R)
forces/fsz-aero-lbs (R)
forces/lod-norm (R)
forces/fbx-weight-lbs (R)
forces/fby-weight-lbs (R)
forces/fbz-weight-lbs (R)
forces/fbx-total-lbs (R)
forces/fby-total-lbs (R)
forces/fbz-total-lbs (R)
forces/fbx-gear-lbs (R)
forces/fby-gear-lbs (R)
forces/fbz-gear-lbs (R)
forces/hold-down (RW)
forces/fbx-prop-lbs (R)
forces/fby-prop-lbs (R)
forces/fbz-prop-lbs (R)
aero/alpha-rad (R)
aero/beta-rad (R)
aero/mag-beta-rad (R)
aero/alpha-deg (R)
aero/beta-deg (R)
aero/mag-beta-deg (R)
aero/Re (R)
aero/qbar-psf (R)
aero/qbarUW-psf (R)
aero/qbarUV-psf (R)
aero/alphadot-rad_sec (R)
aero/betadot-rad_sec (R)
aero/alphadot-deg_sec (R)
aero/betadot-deg_sec (R)
aero/h_b-cg-ft (R)
aero/h_b-mac-ft (R)
aero/cl-squared (R)
aero/qbar-area (RW)
aero/alpha-max-rad (RW)
aero/alpha-min-rad (RW)
aero/bi2vel (R)
aero/ci2vel (R)
aero/alpha-wing-rad (R)
aero/stall-hyst-norm (R)
aero/coefficient/CD0 (R)
aero/coefficient/CDi (R)
aero/coefficient/CDmach (R)
aero/coefficient/CDflap (R)
aero/coefficient/CDgear (R)
aero/coefficient/CDsb (R)
aero/coefficient/CDbeta (R)
aero/coefficient/CDde (R)
aero/coefficient/CYb (R)
aero/coefficient/CLalpha (R)
aero/coefficient/dCLflap (R)
aero/coefficient/dCLsb (R)
aero/coefficient/CLde (R)
aero/coefficient/Clb (R)
aero/coefficient/Clp (R)
aero/coefficient/Clr (R)
aero/coefficient/Clda (R)
aero/coefficient/Cldr (R)
aero/coefficient/Cmalpha (R)
aero/coefficient/Cmde (R)
aero/coefficient/Cmq (R)
aero/coefficient/Cmadot (R)
aero/coefficient/Cnb (R)
aero/coefficient/Cnr (R)
aero/coefficient/Cndr (R)
aero/coefficient/Cnda (R)
flight-path/gamma-rad (R)
flight-path/gamma-deg (R)
flight-path/psi-gt-rad (R)
moments/l-aero-lbsft (R)
moments/m-aero-lbsft (R)
moments/n-aero-lbsft (R)
moments/roll-stab-aero-lbsft (R)
moments/pitch-stab-aero-lbsft (R)
moments/yaw-stab-aero-lbsft (R)
moments/roll-wind-aero-lbsft (R)
moments/pitch-wind-aero-lbsft (R)
moments/yaw-wind-aero-lbsft (R)
moments/l-total-lbsft (R)
moments/m-total-lbsft (R)
moments/n-total-lbsft (R)
moments/l-gear-lbsft (R)
moments/m-gear-lbsft (R)
moments/n-gear-lbsft (R)
moments/l-prop-lbsft (R)
moments/m-prop-lbsft (R)
moments/n-prop-lbsft (R)
systems/stall-warn-norm (R)
ground/solid (RW)
ground/bumpiness (RW)
ground/maximum-force-lbs (RW)
ground/rolling_friction-factor (RW)
ground/static-friction-factor (RW)
ic/vc-kts (RW)
ic/ve-kts (RW)
ic/vg-kts (RW)
ic/vt-kts (RW)
ic/mach (RW)
ic/roc-fpm (RW)
ic/gamma-deg (RW)
ic/alpha-deg (RW)
ic/beta-deg (RW)
ic/theta-deg (RW)
ic/phi-deg (RW)
ic/psi-true-deg (RW)
ic/lat-gc-deg (RW)
ic/long-gc-deg (RW)
ic/h-sl-ft (RW)
ic/h-agl-ft (RW)
ic/terrain-elevation-ft (RW)
ic/vg-fps (RW)
ic/vt-fps (RW)
ic/vw-bx-fps (R)
ic/vw-by-fps (R)
ic/vw-bz-fps (R)
ic/vw-north-fps (R)
ic/vw-east-fps (R)
ic/vw-down-fps (R)
ic/vw-mag-fps (R)
ic/vw-dir-deg (RW)
ic/roc-fps (RW)
ic/u-fps (RW)
ic/v-fps (RW)
ic/w-fps (RW)
ic/vn-fps (RW)
ic/ve-fps (RW)
ic/vd-fps (RW)
ic/gamma-rad (RW)
ic/alpha-rad (RW)
ic/theta-rad (RW)
ic/beta-rad (RW)
ic/phi-rad (RW)
ic/psi-true-rad (RW)
ic/lat-gc-rad (RW)
ic/long-gc-rad (RW)
ic/p-rad_sec (RW)
ic/q-rad_sec (RW)
ic/r-rad_sec (RW)
ic/lat-geod-rad (RW)
ic/lat-geod-deg (RW)
ic/geod-alt-ft (R)
ic/targetNlf (RW)


----------------
  B747  END
----------------

----------------
  F22
----------------

     JSBSim Flight Dynamics Model v1.1.13 [GitHub build 986/commit a09715f01b9e568ce75ca2635ba0a78ce57f7cdd] Dec  3 2022 12:36:17
            [JSBSim-ML v2.0]

JSBSim startup beginning ...

setIC MyAircraft
__init__ JSBAircraft ZZZ
inertial/sea-level-radius_ft (R)
simulation/gravity-model (RW)
simulation/integrator/rate/rotational (RW)
simulation/integrator/rate/translational (RW)
simulation/integrator/position/rotational (RW)
simulation/integrator/position/translational (RW)
simulation/write-state-file (W)
simulation/channel-dt (R)
simulation/gravitational-torque (RW)
simulation/force-output (W)
simulation/do_simple_trim (W)
simulation/reset (W)
simulation/disperse (R)
simulation/randomseed (RW)
simulation/terminate (RW)
simulation/pause (RW)
simulation/sim-time-sec (R)
simulation/dt (R)
simulation/jsbsim-debug (RW)
simulation/frame (RW)
simulation/trim-completed (RW)
velocities/h-dot-fps (R)
velocities/v-north-fps (R)
velocities/v-east-fps (R)
velocities/v-down-fps (R)
velocities/u-fps (R)
velocities/v-fps (R)
velocities/w-fps (R)
velocities/p-rad_sec (R)
velocities/q-rad_sec (R)
velocities/r-rad_sec (R)
velocities/pi-rad_sec (R)
velocities/qi-rad_sec (R)
velocities/ri-rad_sec (R)
velocities/eci-x-fps (R)
velocities/eci-y-fps (R)
velocities/eci-z-fps (R)
velocities/eci-velocity-mag-fps (R)
velocities/ned-velocity-mag-fps (R)
velocities/vc-fps (R)
velocities/vc-kts (R)
velocities/ve-fps (R)
velocities/ve-kts (R)
velocities/vtrue-fps (R)
velocities/vtrue-kts (R)
velocities/machU (R)
velocities/p-aero-rad_sec (R)
velocities/q-aero-rad_sec (R)
velocities/r-aero-rad_sec (R)
velocities/phidot-rad_sec (R)
velocities/thetadot-rad_sec (R)
velocities/psidot-rad_sec (R)
velocities/u-aero-fps (R)
velocities/v-aero-fps (R)
velocities/w-aero-fps (R)
velocities/vt-fps (R)
velocities/mach (R)
velocities/vg-fps (R)
position/h-sl-ft (RW)
position/h-sl-meters (RW)
position/lat-gc-rad (RW)
position/long-gc-rad (RW)
position/lat-gc-deg (RW)
position/long-gc-deg (RW)
position/lat-geod-rad (R)
position/lat-geod-deg (R)
position/geod-alt-ft (R)
position/h-agl-ft (RW)
position/geod-alt-km (R)
position/h-agl-km (RW)
position/radius-to-vehicle-ft (R)
position/terrain-elevation-asl-ft (RW)
position/eci-x-ft (R)
position/eci-y-ft (R)
position/eci-z-ft (R)
position/ecef-x-ft (R)
position/ecef-y-ft (R)
position/ecef-z-ft (R)
position/epa-rad (R)
position/distance-from-start-lon-mt (R)
position/distance-from-start-lat-mt (R)
position/distance-from-start-mag-mt (R)
position/vrp-gc-latitude_deg (R)
position/vrp-longitude_deg (R)
position/vrp-radius-ft (R)
metrics/terrain-radius (R)
metrics/Sw-sqft (RW)
metrics/bw-ft (R)
metrics/cbarw-ft (R)
metrics/iw-rad (R)
metrics/iw-deg (R)
metrics/Sh-sqft (R)
metrics/lh-ft (R)
metrics/Sv-sqft (R)
metrics/lv-ft (R)
metrics/lh-norm (R)
metrics/lv-norm (R)
metrics/vbarh-norm (R)
metrics/vbarv-norm (R)
metrics/aero-rp-x-in (RW)
metrics/aero-rp-y-in (RW)
metrics/aero-rp-z-in (RW)
metrics/eyepoint-x-in (R)
metrics/eyepoint-y-in (R)
metrics/eyepoint-z-in (R)
metrics/visualrefpoint-x-in (R)
metrics/visualrefpoint-y-in (R)
metrics/visualrefpoint-z-in (R)
attitude/phi-rad (R)
attitude/theta-rad (R)
attitude/psi-rad (R)
attitude/phi-deg (R)
attitude/theta-deg (R)
attitude/psi-deg (R)
attitude/roll-rad (R)
attitude/pitch-rad (R)
attitude/heading-true-rad (R)
atmosphere/T-R (R)
atmosphere/rho-slugs_ft3 (R)
atmosphere/P-psf (R)
atmosphere/a-fps (R)
atmosphere/T-sl-R (R)
atmosphere/rho-sl-slugs_ft3 (R)
atmosphere/a-sl-fps (R)
atmosphere/theta (R)
atmosphere/sigma (R)
atmosphere/delta (R)
atmosphere/a-ratio (R)
atmosphere/density-altitude (R)
atmosphere/pressure-altitude (R)
atmosphere/delta-T (RW)
atmosphere/SL-graded-delta-T (RW)
atmosphere/P-sl-psf (RW)
atmosphere/dew-point-R (RW)
atmosphere/vapor-pressure-psf (RW)
atmosphere/saturated-vapor-pressure-psf (R)
atmosphere/RH (RW)
atmosphere/vapor-fraction-ppm (RW)
atmosphere/psiw-rad (RW)
atmosphere/wind-north-fps (RW)
atmosphere/wind-east-fps (RW)
atmosphere/wind-down-fps (RW)
atmosphere/wind-mag-fps (RW)
atmosphere/gust-north-fps (RW)
atmosphere/gust-east-fps (RW)
atmosphere/gust-down-fps (RW)
atmosphere/cosine-gust/startup-duration-sec (W)
atmosphere/cosine-gust/steady-duration-sec (W)
atmosphere/cosine-gust/end-duration-sec (W)
atmosphere/cosine-gust/magnitude-ft_sec (W)
atmosphere/cosine-gust/frame (W)
atmosphere/cosine-gust/X-velocity-ft_sec (W)
atmosphere/cosine-gust/Y-velocity-ft_sec (W)
atmosphere/cosine-gust/Z-velocity-ft_sec (W)
atmosphere/cosine-gust/start (W)
atmosphere/updownburst/number-of-cells (W)
atmosphere/turb-north-fps (RW)
atmosphere/turb-east-fps (RW)
atmosphere/turb-down-fps (RW)
atmosphere/p-turb-rad_sec (R)
atmosphere/q-turb-rad_sec (R)
atmosphere/r-turb-rad_sec (R)
atmosphere/turb-type (RW)
atmosphere/turb-rate (RW)
atmosphere/turb-gain (RW)
atmosphere/turb-rhythmicity (RW)
atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps (RW)
atmosphere/turbulence/milspec/severity (RW)
atmosphere/total-wind-north-fps (R)
atmosphere/total-wind-east-fps (R)
atmosphere/total-wind-down-fps (R)
fcs/aileron-cmd-norm (RW)
fcs/elevator-cmd-norm (RW)
fcs/rudder-cmd-norm (RW)
fcs/flap-cmd-norm (RW)
fcs/speedbrake-cmd-norm (RW)
fcs/spoiler-cmd-norm (RW)
fcs/pitch-trim-cmd-norm (RW)
fcs/roll-trim-cmd-norm (RW)
fcs/yaw-trim-cmd-norm (RW)
fcs/left-aileron-pos-rad (RW)
fcs/left-aileron-pos-deg (RW)
fcs/left-aileron-pos-norm (RW)
fcs/mag-left-aileron-pos-rad (R)
fcs/right-aileron-pos-rad (RW)
fcs/right-aileron-pos-deg (RW)
fcs/right-aileron-pos-norm (RW)
fcs/mag-right-aileron-pos-rad (R)
fcs/elevator-pos-rad (RW)
fcs/elevator-pos-deg (RW)
fcs/elevator-pos-norm (RW)
fcs/mag-elevator-pos-rad (R)
fcs/rudder-pos-rad (RW)
fcs/rudder-pos-deg (RW)
fcs/rudder-pos-norm (RW)
fcs/mag-rudder-pos-rad (R)
fcs/flap-pos-rad (RW)
fcs/flap-pos-deg (RW)
fcs/flap-pos-norm (RW)
fcs/speedbrake-pos-rad (RW)
fcs/speedbrake-pos-deg (RW)
fcs/speedbrake-pos-norm (RW)
fcs/mag-speedbrake-pos-rad (R)
fcs/spoiler-pos-rad (RW)
fcs/spoiler-pos-deg (RW)
fcs/spoiler-pos-norm (RW)
fcs/mag-spoiler-pos-rad (R)
fcs/left-brake-cmd-norm (RW)
fcs/right-brake-cmd-norm (RW)
fcs/center-brake-cmd-norm (RW)
fcs/wing-fold-pos-norm (RW)
fcs/steer-cmd-norm (RW)
fcs/steer-pos-deg (RW)
fcs/throttle-cmd-norm (RW)
fcs/throttle-pos-norm (RW)
fcs/mixture-cmd-norm (RW)
fcs/mixture-pos-norm (RW)
fcs/advance-cmd-norm (RW)
fcs/advance-pos-norm (RW)
fcs/feather-cmd-norm (RW)
fcs/feather-pos-norm (RW)
fcs/throttle-cmd-norm[1] (RW)
fcs/throttle-pos-norm[1] (RW)
fcs/mixture-cmd-norm[1] (RW)
fcs/mixture-pos-norm[1] (RW)
fcs/advance-cmd-norm[1] (RW)
fcs/advance-pos-norm[1] (RW)
fcs/feather-cmd-norm[1] (RW)
fcs/feather-pos-norm[1] (RW)
fcs/u-velocity-norm (RW)
fcs/aileron-cmd-limiter (RW)
fcs/roll-cmd-filter (RW)
fcs/roll-cmd-limit (RW)
fcs/roll-rate-cmd (RW)
fcs/roll-windup-trigger (RW)
fcs/roll-rate-error (RW)
fcs/roll-rate-integrator/initial-integrator-value (W)
fcs/v-velocity-norm (RW)
fcs/rudder-cmd-limiter (RW)
fcs/rudder-cmd-filter (RW)
fcs/yaw-rate-limit (RW)
fcs/yaw-rate-cmd (RW)
fcs/yaw-rate-filter (RW)
fcs/yaw-windup-trigger (RW)
fcs/yaw-rate-error (RW)
fcs/yaw-rate-integrator/initial-integrator-value (W)
fcs/yaw-rate-override (RW)
fcs/yaw-cmd-summer (RW)
fcs/roll-cmd (RW)
fcs/roll-reg-scale (RW)
fcs/aileron-act/malfunction/fail_zero (RW)
fcs/aileron-act/malfunction/fail_hardover (RW)
fcs/aileron-act/malfunction/fail_stuck (RW)
fcs/aileron-act/saturated (R)
fcs/left-aileron-out (RW)
fcs/left-aileron-control (RW)
fcs/left-aileron-neg (RW)
fcs/right-aileron-out (RW)
fcs/right-aileron-control (RW)
fcs/yaw-cmd (RW)
fcs/yaw-reg-scale (RW)
fcs/rudder-act/malfunction/fail_zero (RW)
fcs/rudder-act/malfunction/fail_hardover (RW)
fcs/rudder-act/malfunction/fail_stuck (RW)
fcs/rudder-act/saturated (R)
fcs/rudder-pos-out (RW)
fcs/rudder-position (RW)
fcs/n-pilot-z-correction (RW)
fcs/g-load-corrected (RW)
fcs/g-load-norm (RW)
fcs/stick-filter (RW)
fcs/tvc-inhibit (RW)
fcs/elevator-cmd-limiter (RW)
fcs/pitch-cmd-limiter (RW)
fcs/pitch-rate-cmd (RW)
fcs/pitch-cmd-g-limiter (RW)
fcs/g-limiter (RW)
fcs/gear-down-q-limit (RW)
fcs/q-gear-down-limiter (RW)
fcs/pitch-windup-trigger (RW)
fcs/pitch-rate-error (RW)
fcs/pitch-rate-integrator/initial-integrator-value (W)
fcs/pitch-cmd-summer (RW)
fcs/w-velocity-norm (RW)
fcs/el-pitch-cmd (RW)
fcs/el-reg-scale (RW)
fcs/elevator-act/malfunction/fail_zero (RW)
fcs/elevator-act/malfunction/fail_hardover (RW)
fcs/elevator-act/malfunction/fail_stuck (RW)
fcs/elevator-act/saturated (R)
fcs/elevator-position (RW)
fcs/dht-left-pos-rad (RW)
fcs/dht-right-pos-rad (RW)
fcs/tvc-gain-select (RW)
fcs/tvc-pitch-cmd (RW)
fcs/tvc-gain (RW)
fcs/tvc-gain-hi-lo (RW)
fcs/ss-trim (RW)
fcs/ss-trim-override (RW)
fcs/ss-trim-gain (RW)
fcs/tvc-summer (RW)
fcs/tvc-reg-scale (RW)
fcs/tvc-pos-norm (RW)
fcs/tvc-act/malfunction/fail_zero (RW)
fcs/tvc-act/malfunction/fail_hardover (RW)
fcs/tvc-act/malfunction/fail_stuck (RW)
fcs/tvc-act/saturated (R)
fcs/tvc-pos-rad (RW)
fcs/tvc-position (RW)
fcs/lef-alpha-gain (RW)
fcs/lef-alpha-filter (RW)
fcs/lef-transfer-function (RW)
fcs/lef-deg-to-rad (RW)
fcs/lef-pos (RW)
fcs/lef-norm (RW)
fcs/lef-pos-norm (RW)
fcs/lef-control (RW)
fcs/lef-pos-deg (RW)
fcs/lef-deg (RW)
fcs/lef-pos-rad (RW)
fcs/throttle-adjust (RW)
fcs/throttle-override (RW)
fcs/throttle-gain (RW)
fcs/throttle-filter (RW)
fcs/t1-summer (RW)
fcs/throttle1 (RW)
fcs/throttle1-filter (RW)
fcs/t2-summer (RW)
fcs/throttle2 (RW)
fcs/thrust-norm (RW)
fcs/thrust-normalized (RW)
fcs/tef-pos (RW)
fcs/tef-norm (RW)
fcs/flap-controller (RW)
fcs/flap-pos-out (RW)
fcs/tef-pos-deg (RW)
fcs/speedbrake-alpha-limiter (RW)
fcs/speedbrake-initiate (RW)
fcs/speedbrake-control (RW)
fcs/speedbrake-position-normalizer (RW)
fcs/speedbrake-aileron (RW)
fcs/speedbrake-aileron-right (RW)
fcs/speedbrake-ail-right (RW)
fcs/speedbrake-rudder (RW)
fcs/speedbrake-flap (RW)
fcs/speedbrake-flaps (RW)
fcs/gear-control (RW)
fcs/scheduled-steer-pos-deg (RW)
gear/gear-pos-norm (RW)
gear/gear-cmd-norm (RW)
gear/tailhook-pos-norm (RW)
gear/num-units (R)
gear/wow (R)
gear/unit/solid (RW)
gear/unit/bumpiness (RW)
gear/unit/maximum-force-lbs (RW)
gear/unit/rolling_friction-factor (RW)
gear/unit/static-friction-factor (RW)
gear/unit/WOW (RW)
gear/unit/x-position (RW)
gear/unit/y-position (RW)
gear/unit/z-position (RW)
gear/unit/compression-ft (RW)
gear/unit/compression-velocity-fps (RW)
gear/unit/static_friction_coeff (RW)
gear/unit/dynamic_friction_coeff (RW)
gear/unit/slip-angle-deg (RW)
gear/unit/wheel-speed-fps (R)
gear/unit/side_friction_coeff (RW)
gear/unit/rolling_friction_coeff (RW)
gear/unit/pos-norm (RW)
gear/unit[1]/solid (RW)
gear/unit[1]/bumpiness (RW)
gear/unit[1]/maximum-force-lbs (RW)
gear/unit[1]/rolling_friction-factor (RW)
gear/unit[1]/static-friction-factor (RW)
gear/unit[1]/WOW (RW)
gear/unit[1]/x-position (RW)
gear/unit[1]/y-position (RW)
gear/unit[1]/z-position (RW)
gear/unit[1]/compression-ft (RW)
gear/unit[1]/compression-velocity-fps (RW)
gear/unit[1]/static_friction_coeff (RW)
gear/unit[1]/dynamic_friction_coeff (RW)
gear/unit[1]/slip-angle-deg (RW)
gear/unit[1]/wheel-speed-fps (R)
gear/unit[1]/side_friction_coeff (RW)
gear/unit[1]/rolling_friction_coeff (RW)
gear/unit[1]/pos-norm (RW)
gear/unit[2]/solid (RW)
gear/unit[2]/bumpiness (RW)
gear/unit[2]/maximum-force-lbs (RW)
gear/unit[2]/rolling_friction-factor (RW)
gear/unit[2]/static-friction-factor (RW)
gear/unit[2]/WOW (RW)
gear/unit[2]/x-position (RW)
gear/unit[2]/y-position (RW)
gear/unit[2]/z-position (RW)
gear/unit[2]/compression-ft (RW)
gear/unit[2]/compression-velocity-fps (RW)
gear/unit[2]/static_friction_coeff (RW)
gear/unit[2]/dynamic_friction_coeff (RW)
gear/unit[2]/slip-angle-deg (RW)
gear/unit[2]/wheel-speed-fps (R)
gear/unit[2]/side_friction_coeff (RW)
gear/unit[2]/rolling_friction_coeff (RW)
gear/unit[2]/pos-norm (RW)
inertia/mass-slugs (R)
inertia/weight-lbs (R)
inertia/empty-weight-lbs (R)
inertia/cg-x-in (R)
inertia/cg-y-in (R)
inertia/cg-z-in (R)
inertia/ixx-slugs_ft2 (R)
inertia/iyy-slugs_ft2 (R)
inertia/izz-slugs_ft2 (R)
inertia/ixy-slugs_ft2 (R)
inertia/ixz-slugs_ft2 (R)
inertia/iyz-slugs_ft2 (R)
inertia/print-mass-properties (W)
inertia/pointmass-weight-lbs (RW)
inertia/pointmass-location-X-inches (RW)
inertia/pointmass-location-Y-inches (RW)
inertia/pointmass-location-Z-inches (RW)
propulsion/tat-r (R)
propulsion/tat-c (R)
propulsion/pt-lbs_sqft (R)
propulsion/tank/contents-lbs (RW)
propulsion/tank/unusable-volume-gal (RW)
propulsion/tank/pct-full (R)
propulsion/tank/density-lbs_per_gal (R)
propulsion/tank/priority (RW)
propulsion/tank/external-flow-rate-pps (RW)
propulsion/tank/local-ixx-slug_ft2 (R)
propulsion/tank/local-iyy-slug_ft2 (R)
propulsion/tank/local-izz-slug_ft2 (R)
propulsion/tank/x-position (RW)
propulsion/tank/y-position (RW)
propulsion/tank/z-position (RW)
propulsion/tank[1]/contents-lbs (RW)
propulsion/tank[1]/unusable-volume-gal (RW)
propulsion/tank[1]/pct-full (R)
propulsion/tank[1]/density-lbs_per_gal (R)
propulsion/tank[1]/priority (RW)
propulsion/tank[1]/external-flow-rate-pps (RW)
propulsion/tank[1]/local-ixx-slug_ft2 (R)
propulsion/tank[1]/local-iyy-slug_ft2 (R)
propulsion/tank[1]/local-izz-slug_ft2 (R)
propulsion/tank[1]/x-position (RW)
propulsion/tank[1]/y-position (RW)
propulsion/tank[1]/z-position (RW)
propulsion/set-running (W)
propulsion/starter_cmd (RW)
propulsion/cutoff_cmd (RW)
propulsion/active_engine (RW)
propulsion/total-fuel-lbs (RW)
propulsion/total-oxidizer-lbs (RW)
propulsion/refuel (RW)
propulsion/fuel_dump (RW)
propulsion/fuel_freeze (W)
propulsion/engine/IdleThrust (R)
propulsion/engine/MilThrust (R)
propulsion/engine/AugThrust (R)
propulsion/engine/pitch-angle-rad (RW)
propulsion/engine/yaw-angle-rad (RW)
propulsion/engine/reverser-angle-rad (RW)
propulsion/engine/set-running (RW)
propulsion/engine/thrust-lbs (R)
propulsion/engine/fuel-flow-rate-pps (R)
propulsion/engine/fuel-flow-rate-gph (R)
propulsion/engine/fuel-used-lbs (R)
propulsion/engine/n1 (RW)
propulsion/engine/n2 (RW)
propulsion/engine/injection_cmd (RW)
propulsion/engine/seized (RW)
propulsion/engine/stalled (RW)
propulsion/engine/bleed-factor (RW)
propulsion/engine/MaxN1 (RW)
propulsion/engine/MaxN2 (RW)
propulsion/engine/InjectionTimer (RW)
propulsion/engine/InjWaterNorm (RW)
propulsion/engine/InjN1increment (RW)
propulsion/engine/InjN2increment (RW)
propulsion/engine[1]/IdleThrust (R)
propulsion/engine[1]/MilThrust (R)
propulsion/engine[1]/AugThrust (R)
propulsion/engine[1]/pitch-angle-rad (RW)
propulsion/engine[1]/yaw-angle-rad (RW)
propulsion/engine[1]/reverser-angle-rad (RW)
propulsion/engine[1]/set-running (RW)
propulsion/engine[1]/thrust-lbs (R)
propulsion/engine[1]/fuel-flow-rate-pps (R)
propulsion/engine[1]/fuel-flow-rate-gph (R)
propulsion/engine[1]/fuel-used-lbs (R)
propulsion/engine[1]/n1 (RW)
propulsion/engine[1]/n2 (RW)
propulsion/engine[1]/injection_cmd (RW)
propulsion/engine[1]/seized (RW)
propulsion/engine[1]/stalled (RW)
propulsion/engine[1]/bleed-factor (RW)
propulsion/engine[1]/MaxN1 (RW)
propulsion/engine[1]/MaxN2 (RW)
propulsion/engine[1]/InjectionTimer (RW)
propulsion/engine[1]/InjWaterNorm (RW)
propulsion/engine[1]/InjN1increment (RW)
propulsion/engine[1]/InjN2increment (RW)
accelerations/a-pilot-x-ft_sec2 (R)
accelerations/a-pilot-y-ft_sec2 (R)
accelerations/a-pilot-z-ft_sec2 (R)
accelerations/n-pilot-x-norm (R)
accelerations/n-pilot-y-norm (R)
accelerations/n-pilot-z-norm (R)
accelerations/Nx (R)
accelerations/Ny (R)
accelerations/Nz (R)
accelerations/pdot-rad_sec2 (R)
accelerations/qdot-rad_sec2 (R)
accelerations/rdot-rad_sec2 (R)
accelerations/pidot-rad_sec2 (R)
accelerations/qidot-rad_sec2 (R)
accelerations/ridot-rad_sec2 (R)
accelerations/udot-ft_sec2 (R)
accelerations/vdot-ft_sec2 (R)
accelerations/wdot-ft_sec2 (R)
accelerations/uidot-ft_sec2 (R)
accelerations/vidot-ft_sec2 (R)
accelerations/widot-ft_sec2 (R)
accelerations/gravity-ft_sec2 (R)
forces/load-factor (R)
forces/fbx-aero-lbs (R)
forces/fby-aero-lbs (R)
forces/fbz-aero-lbs (R)
forces/fwx-aero-lbs (R)
forces/fwy-aero-lbs (R)
forces/fwz-aero-lbs (R)
forces/fsx-aero-lbs (R)
forces/fsy-aero-lbs (R)
forces/fsz-aero-lbs (R)
forces/lod-norm (R)
forces/fbx-weight-lbs (R)
forces/fby-weight-lbs (R)
forces/fbz-weight-lbs (R)
forces/fbx-total-lbs (R)
forces/fby-total-lbs (R)
forces/fbz-total-lbs (R)
forces/fbx-gear-lbs (R)
forces/fby-gear-lbs (R)
forces/fbz-gear-lbs (R)
forces/hold-down (RW)
forces/fbx-prop-lbs (R)
forces/fby-prop-lbs (R)
forces/fbz-prop-lbs (R)
aero/alpha-rad (R)
aero/beta-rad (R)
aero/mag-beta-rad (R)
aero/alpha-deg (R)
aero/beta-deg (R)
aero/mag-beta-deg (R)
aero/Re (R)
aero/qbar-psf (R)
aero/qbarUW-psf (R)
aero/qbarUV-psf (R)
aero/alphadot-rad_sec (R)
aero/betadot-rad_sec (R)
aero/alphadot-deg_sec (R)
aero/betadot-deg_sec (R)
aero/h_b-cg-ft (R)
aero/h_b-mac-ft (R)
aero/cl-squared (R)
aero/qbar-area (RW)
aero/alpha-max-rad (RW)
aero/alpha-min-rad (RW)
aero/bi2vel (R)
aero/ci2vel (R)
aero/alpha-wing-rad (R)
aero/stall-hyst-norm (R)
aero/function/kCLge (R)
aero/function/kCDge (R)
aero/coefficient/CD0 (R)
aero/coefficient/CDi (R)
aero/coefficient/CDmach (R)
aero/coefficient/CDflap (R)
aero/coefficient/CDgear (R)
aero/coefficient/CDsb (R)
aero/coefficient/CDbeta (R)
aero/coefficient/CDde (R)
aero/coefficient/CDDlef (R)
aero/coefficient/CDq (R)
aero/coefficient/CDq_Dlef (R)
aero/coefficient/CYb (R)
aero/coefficient/CYb_M (R)
aero/coefficient/CYbdot (R)
aero/coefficient/CYDa (R)
aero/coefficient/CYDr (R)
aero/coefficient/CYp (R)
aero/coefficient/CYr (R)
aero/coefficient/CLalpha (R)
aero/coefficient/CLDlef (R)
aero/coefficient/CLadot (R)
aero/coefficient/CLq (R)
aero/coefficient/dCLflap (R)
aero/coefficient/dCLsb (R)
aero/coefficient/CLde (R)
aero/coefficient/Clb (R)
aero/coefficient/Clda_M (R)
aero/coefficient/Cldr_M (R)
aero/coefficient/Clbdot (R)
aero/coefficient/Clp (R)
aero/coefficient/Clr (R)
aero/coefficient/Clda (R)
aero/coefficient/ClDr (R)
aero/coefficient/CmDh (R)
aero/coefficient/CmDtv (R)
aero/coefficient/Cma_M (R)
aero/coefficient/Cmq (R)
aero/coefficient/Cmadot (R)
aero/coefficient/Cnb (R)
aero/coefficient/Cnb_M (R)
aero/coefficient/Cnbdot (R)
aero/coefficient/Cnp (R)
aero/coefficient/Cnr (R)
aero/coefficient/CnDr (R)
aero/coefficient/Cndr_M (R)
aero/coefficient/Cnda (R)
flight-path/gamma-rad (R)
flight-path/gamma-deg (R)
flight-path/psi-gt-rad (R)
moments/l-aero-lbsft (R)
moments/m-aero-lbsft (R)
moments/n-aero-lbsft (R)
moments/roll-stab-aero-lbsft (R)
moments/pitch-stab-aero-lbsft (R)
moments/yaw-stab-aero-lbsft (R)
moments/roll-wind-aero-lbsft (R)
moments/pitch-wind-aero-lbsft (R)
moments/yaw-wind-aero-lbsft (R)
moments/l-total-lbsft (R)
moments/m-total-lbsft (R)
moments/n-total-lbsft (R)
moments/l-gear-lbsft (R)
moments/m-gear-lbsft (R)
moments/n-gear-lbsft (R)
moments/l-prop-lbsft (R)
moments/m-prop-lbsft (R)
moments/n-prop-lbsft (R)
systems/stall-warn-norm (R)
ground/solid (RW)
ground/bumpiness (RW)
ground/maximum-force-lbs (RW)
ground/rolling_friction-factor (RW)
ground/static-friction-factor (RW)
ic/vc-kts (RW)
ic/ve-kts (RW)
ic/vg-kts (RW)
ic/vt-kts (RW)
ic/mach (RW)
ic/roc-fpm (RW)
ic/gamma-deg (RW)
ic/alpha-deg (RW)
ic/beta-deg (RW)
ic/theta-deg (RW)
ic/phi-deg (RW)
ic/psi-true-deg (RW)
ic/lat-gc-deg (RW)
ic/long-gc-deg (RW)
ic/h-sl-ft (RW)
ic/h-agl-ft (RW)
ic/terrain-elevation-ft (RW)
ic/vg-fps (RW)
ic/vt-fps (RW)
ic/vw-bx-fps (R)
ic/vw-by-fps (R)
ic/vw-bz-fps (R)
ic/vw-north-fps (R)
ic/vw-east-fps (R)
ic/vw-down-fps (R)
ic/vw-mag-fps (R)
ic/vw-dir-deg (RW)
ic/roc-fps (RW)
ic/u-fps (RW)
ic/v-fps (RW)
ic/w-fps (RW)
ic/vn-fps (RW)
ic/ve-fps (RW)
ic/vd-fps (RW)
ic/gamma-rad (RW)
ic/alpha-rad (RW)
ic/theta-rad (RW)
ic/beta-rad (RW)
ic/phi-rad (RW)
ic/psi-true-rad (RW)
ic/lat-gc-rad (RW)
ic/long-gc-rad (RW)
ic/p-rad_sec (RW)
ic/q-rad_sec (RW)
ic/r-rad_sec (RW)
ic/lat-geod-rad (RW)
ic/lat-geod-deg (RW)
ic/geod-alt-ft (R)
ic/targetNlf (RW)
aero_ref_pt_shift_x (R)




----------------
  F22  END
----------------





'''
   