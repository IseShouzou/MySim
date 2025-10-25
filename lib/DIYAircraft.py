import os
import sys
import copy
import time

import numpy as np

from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *

from PyQt6.QtOpenGL import *
from PyQt6.QtOpenGLWidgets import QOpenGLWidget

# from MyGLWidget import *
from lib.MyPyqt import *
from lib.MyFunc import *

 
  

class AIRCRAFT( QObject ) :
    
    signal = pyqtSignal( str )
    
    def __init__( self, parent = None) :
        print( '__init__ AIRCRAFT' )
        
        super().__init__()

        self.parent = parent
 
 
 
class Atomsphere() :
    
    def __init__( self ) :
        
        self.T0   =   288.15
        self.P0   = 101325.0
        self.rho0 =    1.225
        self.a0   =  340.294
        
        self.kappa = 1.4
        self.R     = 287.05287
        self.g     = 9.80665
        self.beta  = -0.0065

        self.Hp_trop = 11000.0
        
        self.m         = - self.g / self.beta / self.R
        self.T_ISAtrop = self.T0 + self.beta * self.Hp_trop
        self.P_trop    = self.P0 * pow( self.T_ISAtrop / self.T0, self.m )
        
        
    def calcAtomsphere( self, Hp_m, dP = 0.0, dT = 0.0 ) :
        
        P_MSL = self.P0 + dP
        Hp_MSL = self.T0 / self.beta * ( pow( P_MSL / self.P0 , 1.0 / self.m ) - 1.0 )
        T_MSL = self.T0 + dT + self.beta * Hp_MSL
        #print( Hp_m, self.Hp_trop )
        if Hp_m < self.Hp_trop :
            T = self.T0 + dT + self.beta * Hp_m
            P = self.P0 * pow( ( T - dT ) / self.T0, self.m )
            #print( 'AAA', T, P )
        else :
            #T_ISAtrop = self.T0 + self.beta * self.Hp_trop
            T_trop = self.T_ISAtrop + dT
            #P_trop = self.P0 * pow( T_ISAtrop / self.T0, self.m )
            T = T_trop
            P = self.P_trop * np.exp( - self.g / self.R / self.T_ISAtrop * ( Hp_m - self.Hp_trop ) );
            #print( 'BBB', T, P )
        rho = P / self.R / T
        ss = np.sqrt( self.kappa * self.R * T )
        #print( rho )        
        return rho, P, ss
           
          
    def calcVtas( self, vc, Hp_m, dP = 0.0, dT = 0.0 ) :
        rho, P, ss = self.calcAtomsphere( Hp_m, dP, dT)
        AA = 7.0 * P / rho
        A0 = 7.0 * self.P0 / self.rho0
        qc = self.P0 * ( pow( 1.0 + vc * vc / A0, 3.5 ) - 1.0 )
        vt = np.sqrt( AA * ( pow( 1.0 + qc / P, 1.0 / 3.5 ) - 1.0 ) )
        return vt
        
        
    def calcVcas( self, vt, Hp_m, dP = 0.0, dT = 0.0 ) :
        rho, P, ss = self.calcAtomsphere( Hp_m, dP, dT)
        #print( rho, P, ss )
        AA = 7.0 * P / rho
        A0 = 7.0 * self.P0 / self.rho0
        qc = P * ( pow( 1.0 + vt * vt / AA, 3.5 ) - 1.0 )
        vc = np.sqrt( A0 * ( pow( 1.0 + qc / self.P0, 1.0 / 3.5 ) - 1.0 ) )
        return vc
        
                
            

class DIYAircraft( AIRCRAFT ) :
    
#     signal = pyqtSignal( str )
    
    def __init__( self ) :
        print( '__init__ DIYAircraft' )
        
        super().__init__()

        self.atoms = Atomsphere() 
        self.grav = 9.80665
        self.deltaTime = 0.01
          
        self.Zground = 0.0
          
        self.setParam()
          
#         self.initX   =  0.0
#         self.initY   =  0.0
#         self.initAlt =  3.0
#         
#         self.initVt  =  0.0 * 0.5144
#         self.initGam =  np.radians( 0.0 )
#         self.initPsi =  np.radians( 0.0 )
                
        self.xyz    = np.array( [  0.0,  0.0, -3.0 ] )
        self.qua    = np.array( [  0.0,  0.0,  0.0,  1.0 ] )
        self.uvw    = np.array( [  0.0,  0.0,  0.0 ] )
        self.pqr    = np.array( [  0.0,  0.0,  0.0 ] )
        self.uvwDot = np.array( [  0.0,  0.0,  0.0 ] )
        self.thst   = 0.0

        self.calc()


    def setParam( self ) :
          
        #
        self.Sref = 511.0
        self.Cbar =  8.32
        self.Bbar = 59.64

        self.Mass  =    255000.0
        Ixx   =   1898000.0 * self.grav
        Iyy   =   4214300.0 * self.grav
        Izz   =   5859200.0 * self.grav
        Izx   =    114100.0 * self.grav
        Ixy   =        0.0
        Iyz   =        0.0

 
        #self.r_pilot = np.array( [ 3.0, 0.0, 0.0 ] )
        
        self.gearPos = np.array( [ [  25.0,  0.0, 3.0 ],
                                   [  -2.0, -2.0, 3.0 ],
                                   [  -2.0,  2.0, 3.0 ], ] )
        
        self.gearDir = np.array( [ [  0.0, 0.0, -1.0 ],
                                   [  0.0, 0.0, -1.0 ],
                                   [  0.0, 0.0, -1.0 ], ] )
        
        self.gearKC  = np.array( [ [  300000.0, 100000.0 ],
                                   [ 2000000.0, 200000.0 ],
                                   [ 2000000.0, 200000.0 ], ] )
        
        
        self.thst_ref = 4.0 * 58000.0 * 0.4536 * 9.80665
 
        self.aeroCoeff = np.array( [ [  0.5430,  5.700 ,  0.338,   5.4  ,  -6.700 ],   # CL0, CLα, CLδe, CLq, CLαdot
                                     [  0.0200,  0.078 ,  0.000,   0.0  ,   0.0   ],   # CD0, CDK
                                     [  0.0100, -1.260 , -1.34 , -20.8  ,  -3.2   ],   # Cm0, Cmα, CLδe, Cmq, Cmαdot
                                     [ -0.96  ,  0.0   ,  0.175,   0.0  ,   0.0   ],   # Cyβ, Cyδa, Cyδr, Cyp, Cyr
                                     [ -0.221 ,  0.0461,  0.000,  -0.45 ,   0.101 ],   # Clβ, Clδa, Clδr, Clp, Clr
                                     [  0.15  ,  0.0064, -0.109,  -0.121,  -0.300 ],   # Cnβ, Cnδa, Cnδr, Cnp, Cnr
                                   ] )
        

        self.alp_CLmax = np.radians(  30.0 )
        self.deRange  = ( np.radians( -30.0 ), np.radians( 30.0 ) )
        
        
        self.deMax = np.radians( 30.0 )
        self.daMax = np.radians( 30.0 )
        self.drMax = np.radians( 30.0 )
        
        self.dFlapMax_deg = 30.0
        
        
        self.muMax = 1.0
        
        self.Iner = np.array( [ [  Ixx, -Ixy, -Izx ],
                                [ -Ixy,  Iyy, -Iyz ],
                                [ -Izx, -Iyz,  Izz ] ] )

        self.Iinv = np.linalg.inv( self.Iner )

        for k, gDir in enumerate( self.gearDir ) :
            self.gearDir[k] = gDir / np.linalg.norm( gDir )
            

    @property
    def stickPos( self ) :
        X = self.da_cmd_norm
        Y = self.de_cmd_norm
        Z = self.dr_cmd_norm
        return np.array( [ X, Y, Z ] )

    @stickPos.setter
    def stickPos( self, value ) :
        X, Y, Z = value
        self.da_cmd_norm = X
        self.de_cmd_norm = Y
        self.dr_cmd_norm = Z
        
    @property
    def trimPos( self ) :
        X = self.roll_trim_cmd_norm
        Y = self.pitch_trim_cmd_norm
        Z = self.yaw_trim_cmd_norm
        return np.array( [ X, Y, Z ] )
        
    @trimPos.setter
    def trimPos( self, value ) :
        X, Y, Z = value
        self.roll_trim_cmd_norm  = X
        self.pitch_trim_cmd_norm = Y
        self.yaw_trim_cmd_norm   = Z
        
    @property
    def thrbrkPos( self ) :
        TH = self.throttle_cmd_norm
        BB = self.brake_cmd_norm
        return np.array( [ TH, BB ] )
        
    @thrbrkPos.setter
    def thrbrkPos( self, value ) :
        TH, BB = value
        self.throttle_cmd_norm = TH
        self.brake_cmd_norm    = BB
        
    @property
    def DEVcmd( self ) :
        FL = self.flap_cmd_norm
        SB = self.speedbrake_cmd_norm
        GR = self.gear_cmd_norm
        return np.array( [ FL, SB, GR ] )
        
    @DEVcmd.setter
    def DEVcmd( self, value ) :
        FL, SB, GR = value
        self.flap_cmd_norm       = FL
        self.speedbrake_cmd_norm = SB
        self.gear_cmd_norm       = GR
        
    @property
    def DEVpos( self ) :
        FL = self.flap_pos_norm
        SB = self.speedbrake_pos_norm
        GR = self.gear_pos_norm
        return np.array( [ FL, SB, GR ] )

#     @DEVpos.setter
#     def DEVpos( self, value ) :
#         FL, SB, GR = value
#         self.flap_pos_norm       = FL
#         self.speedbrake_pos_norm = SB
#         self.gear_pos_norm       = GR
        

    def setGroundAlt_FT( self, alt ) :
        print( 'setIC setGroundAlt_FT' )
        self.Zground = - alt * 0.3048 

        
    def setIC( self, X_nm, Y_nm, Alt_ft, HDG_deg, Vc_kt, Gam_deg ) :
        print( 'setIC DIYAircraft' )
        
        self.initX   = X_nm * 1852.0
        self.initY   = Y_nm * 1852.0
        self.initAlt = Alt_ft * 0.3048
        self.initPsi =  np.radians( HDG_deg )
        self.initVt  = self.atoms.calcVtas( Vc_kt * 0.5144, self.initAlt )
        self.initGam = np.radians( Gam_deg )
        
        print( 'initAlt  : ', self.initAlt )
        print( self.atoms.calcAtomsphere( self.initAlt ) )
        print( 'Vc_kt  : ', Vc_kt               , Vc_kt * 0.5144 )
        print( 'initVt : ', self.initVt / 0.5144 , self.initVt)
        

    def reset( self ) :
        print( 'reset DIYAircraft' )
 
        #self.Zground = 0.0
        self.simTime = 0.0
        
        self.XYZ = np.array( [ self.initX, self.initY, - self.initAlt ] )
        self.Vt  = self.initVt
        self.gam = self.initGam
        
        self.alp     = 0.0
        self.bet     = 0.0 
        self.alpDot  = 0.0
        self.pqr     = np.array( [ 0.0, 0.0, 0.0 ] )
        self.uvwDot  = np.array( [ 0.0, 0.0, 0.0 ] )
        
        rho, P, ss = self.atoms.calcAtomsphere( - self.XYZ[2] )
        self.rho   = rho
        self.mach  = self.Vt / ss

        self.da_cmd_norm         = 0.0
        self.de_cmd_norm         = 0.0
        self.dr_cmd_norm         = 0.0
        
        self.roll_trim_cmd_norm  = 0.0
        self.pitch_trim_cmd_norm = 0.0
        self.yaw_trim_cmd_norm   = 0.0
        
        self.throttle_cmd_norm   = 0.0
        self.brake_cmd_norm      = 0.0
        
        self.flap_pos_norm       = self.flap_cmd_norm 
        self.speedbrake_pos_norm = self.speedbrake_cmd_norm
        self.gear_pos_norm       = self.gear_cmd_norm

        self.da     = self.daMax * ( self.da_cmd_norm + self.roll_trim_cmd_norm  )
        self.de     = self.deMax * ( self.de_cmd_norm + self.pitch_trim_cmd_norm )
        self.dr     = self.drMax * ( self.dr_cmd_norm + self.yaw_trim_cmd_norm   )
        self.thst   = self.thst_ref * self.throttle_cmd_norm
        
        self.gearMu = self.muMax * self.brake_cmd_norm * np.array( [ 1.0, 1.0, 1.0 ] )
                
        self.gearDef    = np.array( [ 0.0, 0.0, 0.0 ] )
        self.gearDefDot = np.array( [ 0.0, 0.0, 0.0 ] )
        self.gearLoad   = np.array( [ 0.0, 0.0, 0.0 ] )
        #self.gearF      = np.zeros( ( 3, 3 ) )
        #self.gearG      = np.zeros( ( 3, 3 ) )
        
        ###self.calcTrim()
        self.calcTrimAAA()
        
        self.pitch_trim_cmd_norm = self.de   / self.deMax
        self.throttle_cmd_norm   = self.thst / self.thst_ref
        
        cAlp, sAlp = np.cos( self.alp ), np.sin( self.alp )
        self.uvw = np.array( [ self.Vt * cAlp, 0.0, self.Vt * sAlp ] )

        self.eul = np.array( [ 0.0, self.alp + self.gam, self.initPsi ] )
        self.dcm = euler2dcm( self.eul )
        self.dcmt = np.transpose( self.dcm )
        self.xyz = np.dot( self.dcm, self.XYZ )
        self.qua = dcm2qua( self.dcm )
        
        self.alt_sl    = - self.XYZ[2]
        self.alt_sl_ft = self.alt_sl / 0.3048
        self.eul_deg   = np.degrees( self.eul )
        self.uvw_fps   = self.uvw / 0.3048
        self.UVW_fps   = self.UVW / 0.3048
        self.Vt_kt     = self.Vt / 0.5144
        self.Vc_kt     = self.atoms.calcVcas( self.Vt, self.alt_sl ) / 0.5144
        
#         self.gearDef    = np.array( [ 0.0, 0.0, 0.0 ] )
#         self.gearDefDot = np.array( [ 0.0, 0.0, 0.0 ] )
#         self.gearLoad   = np.array( [ 0.0, 0.0, 0.0 ] )
#         self.gearF      = np.zeros( ( 3, 3 ) )
#         self.gearG      = np.zeros( ( 3, 3 ) )
# 
        #self.calc()
        #print( 'self.totalFG  : ', self.totalFG )

        print( 'eul_deg : ', self.eul_deg )
        print( 'de_deg  : ', np.degrees( self.de ) )
        print( 'thst    : ', self.thst  )

        #self.calcTrimAAA()


    def calc( self ) :
        #print( 'calc DIYAircraft' )
        
        self.dcm = qua2dcm( self.qua )
        self.dcmt = np.transpose( self.dcm )
 
        self.XYZ = np.dot( self.dcmt, self.xyz )
        self.UVW = np.dot( self.dcmt, self.uvw )
        
        self.alt_sl    = - self.XYZ[2]
        self.alt_sl_ft = self.alt_sl / 0.3048
        self.eul       = dcm2euler( self.dcm )
        self.eul_deg   = np.degrees( self.eul )
        self.uvw_fps   = self.uvw / 0.3048
        self.UVW_fps   = self.UVW / 0.3048
        self.Vt        = np.linalg.norm( self.uvw )
        self.Vt_kt     = self.Vt / 0.5144
        self.Vc_kt     = self.atoms.calcVcas( self.Vt, self.alt_sl ) / 0.5144
        
        uw = self.uvw[0] * self.uvw[0] + self.uvw[2] * self.uvw[2]
        UV = np.sqrt( self.UVW[0] * self.UVW[0] + self.UVW[1] * self.UVW[1] )
        self.alp    = 0.0 if abs( self.uvw[0] ) < 0.001 else np.arctan2( self.uvw[2], self.uvw[0] )
        self.bet    = 0.0 if self.Vt            < 0.001 else np.arcsin( self.uvw[1] / self.Vt )
        self.gam    = 0.0 if UV < 0.001 else np.arctan2( - self.UVW[2], UV )
        self.alpDot = 0.0 if uw < 0.001 else ( self.uvwDot[2] * self.uvw[0] - self.uvwDot[0] * self.uvw[2] ) / uw
        
        
        

    def calcAeroCoeff( self ) :
        #print( 'calcAeroCoeff DIYAircraft' )
        
        pAst = 0.0 if self.Vt < 0.01 else 0.5 * self.Bbar / self.Vt * self.pqr[0]
        qAst = 0.0 if self.Vt < 0.01 else 0.5 * self.Cbar / self.Vt * self.pqr[1]
        rAst = 0.0 if self.Vt < 0.01 else 0.5 * self.Bbar / self.Vt * self.pqr[2]
        aAst = 0.0 if self.Vt < 0.01 else 0.5 * self.Cbar / self.Vt * self.alpDot
        
        alp = self.alp
        bet = self.bet
        
        da  = self.da
        de  = self.de
        dr  = self.dr
        
        self.CL = self.aeroCoeff[0][0]        + self.aeroCoeff[0][1] * alp  + self.aeroCoeff[0][2] * de \
           + self.aeroCoeff[0][3] * qAst + self.aeroCoeff[0][4] * aAst
        
        self.CL = max( min( self.CL, 2.0 ), -2.0 )
        
        self.CD = self.aeroCoeff[1][0]        + self.aeroCoeff[1][1] * self.CL * self.CL
        self.Cm = self.aeroCoeff[2][0]        + self.aeroCoeff[2][1] * alp  + self.aeroCoeff[2][2] * de \
           + self.aeroCoeff[2][3] * qAst + self.aeroCoeff[2][4] * aAst
        self.Cm = max( min( self.Cm, 2.0 ), -2.0 )
        
        self.Cy = self.aeroCoeff[3][0] * bet  + self.aeroCoeff[3][1] * da   + self.aeroCoeff[3][2] * dr \
           + self.aeroCoeff[3][3] * pAst + self.aeroCoeff[3][4] * rAst
        self.Cl = self.aeroCoeff[4][0] * bet  + self.aeroCoeff[4][1] * da   + self.aeroCoeff[4][2] * dr \
           + self.aeroCoeff[4][3] * pAst + self.aeroCoeff[4][4] * rAst
        self.Cn = self.aeroCoeff[5][0] * bet  + self.aeroCoeff[5][1] * da   + self.aeroCoeff[5][2] * dr \
           + self.aeroCoeff[5][3] * pAst + self.aeroCoeff[5][4] * rAst

        ca, sa = np.cos( self.alp ), np.sin( self.alp )
        self.Cx =   sa * self.CL - ca * self.CD
        self.Cz = - ca * self.CL - sa * self.CD
        
        
        

    def step( self, curTime ) :
        #print( 'step DIYAircraft' )
        while self.simTime < curTime :
            self.oneStep()
        return self.simTime


    def calcTotalFG( self ) :
        #print( 'calcForce DIYAircraft' )
        
        #
        #  aero. force
        #
        QS     = 0.5 * self.rho * self.Vt * self.Vt * self.Sref
        QSCbar = QS * self.Cbar
        QSBbar = QS * self.Bbar
        
        dxyzAero = self.xyzAeroRef - self.xyzCG
        
        self.calcAeroCoeff()
        airF = np.array( [ self.Cx * QS     , self.Cy * QS    , self.Cz * QS     ] )
        airG = np.cross( dxyzAero, airF )   \
             + np.array( [ self.Cl * QSBbar , self.Cm * QSCbar, self.Cn * QSBbar ] )
        self.airFG = np.array( [ *airF, *airG ] )

        #
        #  gravity
        #
        mg = self.Mass * self.grav
        cPhi, sPhi = np.cos( self.eul[0] ), np.sin( self.eul[0] )
        cThe, sThe = np.cos( self.eul[1] ), np.sin( self.eul[1] )
        self.grvFG = np.array( [ - mg * sThe, mg * cThe * sPhi, mg * cThe * cPhi,
                                  0.0       , 0.0             , 0.0               ] )

        #
        #  thrust
        #
        dxyzThr = self.xyzThrust - self.xyzCG
        thrVec = np.array( [ *self.thstDir, *np.cross( dxyzThr, self.thstDir ) ] )
        self.thrFG = self.thst * thrVec

        #
        #  gear force
        #
        dGearPos = self.gearPos - self.xyzCG
        CTZ      = self.dcm[:,2]
        delZ     = self.Zground - self.XYZ[2]
        Z0       = self.XYZ[2] - self.Zground
        Z0dot    = self.UVW[2]

        self.gearFG     = np.array( [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ] )
        self.gearDef    = np.array( [ 0.0, 0.0, 0.0 ] )
        self.gearDefDot = np.array( [ 0.0, 0.0, 0.0 ] )
        
        if Z0 > -50.0 :
            for k in range( 3 ) :
                CTZDir = np.dot( CTZ, self.gearDir[k] )
                if abs( CTZDir ) < 1.0e-6 :
                    self.gearLoad[k] = 0.0
                    gearZ = 0.0
                else :
                    self.gearDef[k] = - ( Z0 + np.dot( CTZ, dGearPos[k] ) ) / CTZDir
                    if self.gearDef[k] < 0.0 :
                        self.gearLoad[k] = 0.0
                    else :
                        omgRg = np.cross( self.pqr, dGearPos[k] )
                        omgUg = np.cross( self.pqr, self.gearDir[k] )
                        self.gearDefDot[k] = - ( Z0dot + np.dot( CTZ, omgRg + omgUg * self.gearDef[k] ) ) / CTZDir
                        self.gearLoad[k] = self.gearKC[k][0] * self.gearDef[k] + self.gearKC[k][1] * self.gearDefDot[k]
                    gearZ = self.gearLoad[k] / CTZDir

                uvwg = self.uvw + np.cross( self.pqr, dGearPos[k] )
                UVWg = np.dot( self.dcmt, uvwg )
                UVg = np.sqrt( UVWg[0] * UVWg[0] + UVWg[1] * UVWg[1] )
                
                fric = abs( self.gearMu[k] * gearZ )
                if UVg > 0.00001 :
                    gearX = - fric * UVWg[0] / UVg
                    gearY = - fric * UVWg[1] / UVg                
                    if UVg < 0.1 :
                        gearX *= UVg / 0.1
                        gearY *= UVg / 0.1               
                else :
                    gearX = 0.0
                    gearY = 0.0
                        
                pos = dGearPos[k] + self.gearDef[k] * self.gearDir[k]
                #self.gearF[k] = np.dot( self.dcm, [ gearX, gearY, gearZ ] )
                #self.gearG[k] = np.cross( pos, self.gearF[k] )
                
                #self.gearFG[0:3] += self.gearF[k]
                #self.gearFG[3:6] += self.gearG[k] 

                gearF = np.dot( self.dcm, [ gearX, gearY, gearZ ] )
                gearG = np.cross( pos, gearF )
                self.gearFG[0:3] += gearF
                self.gearFG[3:6] += gearG 


        self.totalFG = self.airFG + self.thrFG + self.grvFG + self.gearFG
        
        self.Nxyz   = ( self.airFG + self.thrFG + self.gearFG ) / mg
        
        
#         print( 'airF         : ', airFG[0:3] / 9.80665 / 0.45359 )
#         print( 'thrF         : ', thrFG[0:3] / 9.80665 / 0.45359 )
#         print( 'grvF         : ', grvFG[0:3] / 9.80665 / 0.45359 )
#         print()
#         print( 'airG         : ', airFG[3:6] / 9.80665 / 0.45359 / 0.3048 )
#         print( 'thrG         : ', thrFG[3:6] / 9.80665 / 0.45359 / 0.3048 )
#         print( 'grvG         : ', grvFG[3:6] / 9.80665 / 0.45359 / 0.3048 )
#         print()
#         print( 'airFG         : ', airFG )
#         print( 'thrFG         : ', thrFG )
#         print( 'grvFG         : ', grvFG )
#         print( 'self.totalFG  : ', self.totalFG )
#  


    def oneStep( self ) :
        #print( 'oneStep DIYAircraft' )

        rho, P, ss = self.atoms.calcAtomsphere( - self.XYZ[2] )
        self.rho   = rho
        self.mach  = self.Vt / ss

        dFlap = self.flap_cmd_norm       - self.flap_pos_norm
        dSB   = self.speedbrake_cmd_norm - self.speedbrake_pos_norm
        dGear = self.gear_cmd_norm       - self.gear_pos_norm

        self.flap_pos_norm       += dFlap / self.timeConst_flap * self.deltaTime
        self.speedbrake_pos_norm += dSB   / self.timeConst_sb   * self.deltaTime
        self.gear_pos_norm       += dGear / self.timeConst_gear * self.deltaTime


        self.da     = self.daMax * ( self.da_cmd_norm + self.roll_trim_cmd_norm  )
        self.de     = self.deMax * ( self.de_cmd_norm + self.pitch_trim_cmd_norm )
        self.dr     = self.drMax * ( self.dr_cmd_norm + self.yaw_trim_cmd_norm   )
        self.gearMu = self.muMax * self.brake_cmd_norm * np.array( [ 1.0, 1.0, 1.0 ] )
        

        targetThst = self.thst_ref * self.throttle_cmd_norm

        self.thst += ( targetThst - self.thst ) / self.timeConst_thst * self.deltaTime

        self.calcTotalFG()
        
        totalFG = self.totalFG
        
        
        OMEGA = np.array( [ [   0.0       ,    self.pqr[2], - self.pqr[1], self.pqr[0] ],
                            [ - self.pqr[2],   0.0        ,   self.pqr[0], self.pqr[1] ],
                            [   self.pqr[1], - self.pqr[0],   0.0        , self.pqr[2] ],
                            [ - self.pqr[0], - self.pqr[1], - self.pqr[2], 0.0         ] ] )
        
        self.force   = totalFG[ 0 : 3 ]
        self.moment  = totalFG[ 3 : 6 ]
        
        self.xyzDot = self.uvw          - np.cross( self.pqr, self.xyz )
        self.uvwDot = self.force / self.Mass - np.cross( self.pqr, self.uvw )
        self.quaDot = 0.5 * np.dot( OMEGA, self.qua )
        self.pqrDot = np.dot( self.Iinv, self.moment - np.cross( self.pqr, np.dot( self.Iner, self.pqr ) ) )
        
        self.xyz += self.deltaTime * self.xyzDot
        self.uvw += self.deltaTime * self.uvwDot
        self.qua += self.deltaTime * self.quaDot
        self.pqr += self.deltaTime * self.pqrDot
                
        self.qua /=  np.linalg.norm( self.qua )

        self.simTime += self.deltaTime

        self.calc()
        

 
 
    def calcFXFZMY( self, X ) :
        #print( 'calcFXFZMY DIYAircraft' )
        self.alp  = X[0]
        self.de   = X[1]
        self.thst = X[2]
        
        self.eul[1] = self.gam + self.alp 
        
        self.calcTotalFG()
        return np.array( [ self.totalFG[0], self.totalFG[2], self.totalFG[4] ] )
        
    
    def calcTrimAAA( self ) :
        print( 'calcTrimAAA DIYAircraft' )
        
        print( 'self.rho     : ', self.rho )
        print( 'self.XYZ     : ', self.XYZ )
        print( 'self.Vt      : ', self.Vt )
        print( 'self.mach    : ', self.mach )
        print( 'self.gam     : ', self.gam )
        print( 'self.eul_deg : ', self.eul_deg )

        print( 'self.alp   : ', np.degrees( self.alp ) )
        print( 'self.de    : ', np.degrees( self.de ) )
        print( 'self.thst  : ', self.thst )

        noError = True
        txt = ''

        alpCurr  = self.alp 
        deCurr   = self.de 
        thstCurr = self.thst 

        rng = [ [ - self.alp_CLmax, self.alp_CLmax ],
                [ - self.deMax    , self.deMax     ],
                [   0.0           , self.thst_ref  ] ]
        www = [ r[1] - r[0] for r in rng  ]
        
        eps = 0.001
        X = np.array( [ 0.0, 0.0, 0.0 ] )

        for iIter in range( 5 ) :
            print( 'iIter  :', iIter )
            
            print( 'X   : ', X )

            for x, r in zip( X, rng ) :
                if x < r[0] : noError = False
                if x > r[1] : noError = False
            if not noError :
                txt = 'out of range !!'
                break

            YY = self.calcFXFZMY( X )
            print( 'YY   : ', YY )


            if np.linalg.norm( YY ) < 1.0e-6 : break

            dX0 = eps * www[0]
            dX1 = eps * www[1]
            dX2 = eps * www[2]
            if X[0] + dX0 > rng[0][1] : dX0 = - dX0
            if X[1] + dX1 > rng[1][1] : dX1 = - dX1
            if X[2] + dX2 > rng[2][1] : dX2 = - dX2
         
            Y0 = self.calcFXFZMY( [ X[0] + dX0, X[1]     , X[2]      ] )    
            Y1 = self.calcFXFZMY( [ X[0]      , X[1]+ dX1, X[2]      ] )    
            Y2 = self.calcFXFZMY( [ X[0]      , X[1]     , X[2]+ dX2] )    

            dY0 = ( Y0 - YY ) / dX0
            dY1 = ( Y1 - YY ) / dX1
            dY2 = ( Y2 - YY ) / dX2

            print( 'dY0   : ', dY0 )
            print( 'dY1   : ', dY1 )
            print( 'dY2   : ', dY2 )

            try :
                mat = np.linalg.inv( np.array( [ dY0, dY1, dY2 ] ) )
                
                X += - np.dot( YY, mat )
                self.alp  = X[0]
                self.de   = X[1]
                self.thst = X[2]
                    
            except np.linalg.LinAlgError :
                noError = False
                txt = 'Singularity error !!'
                break
            
        if noError :
            txt = 'Trim OK !!'
            self.signal.emit( txt )
        else :
            self.alp = alpCurr
            self.de  = deCurr
            self.thst  = thstCurr
            self.signal.emit( txt )
            
        print( 'self.alp   : ', np.degrees( self.alp ) )
        print( 'self.de    : ', np.degrees( self.de ) )
        print( 'self.thst  : ', self.thst )
            
        print( 'calcTrimAAA DIYAircraft ZZZ' )
 
 

#----------------------------------------------------

class DIYAircraft_B747( DIYAircraft ) :
    
    def __init__( self ) :
        print( '__init__ DIYAircraft_B747' )
        
        super().__init__()


    def setParam( self ) :
          
        #
        self.Sref = 524.716   #   5648.0 * 0.3048 * 03048 
        self.Cbar =   8.324   #    27.31 * 0.3048
        self.Bbar =  64.465   #    211.5 * 0.3048
        
        self.xyzCG      = np.array( [ -1327.0, 0.0,   24.0 ] ) * 0.0254
        self.xyzAeroRef = np.array( [ -1377.0, 0.0,   24.0 ] ) * 0.0254
        self.xyzThrust  = np.array( [ -1327.0, 0.0,  109.0 ] ) * 0.0254

        self.xyzPilot   = np.array( [  -308.0, 0.0, -138.0 ] ) * 0.0254

      
        #
        #  1 slug = 1 lb s^2 / ft =  0.45359 / 0.3048 kgf s^2 / m
        #                         =  0.45359 / 0.3048 * 9.80665 kg
        #                         =  14.5939 kg
        #  1 slug ft^2            = 14.5939 * 0.3048^2 kg m^2
        #                         =  1.3558 kg
        #
        #self.Mass  =    255000.0
        self.Mass  =    551098.0 * 0.45359
        
        Ixx   =   1.82e+07 * 1.3558 #  slug ft^2 -> kg m^2
        Iyy   =   3.31e+07 * 1.3558 
        Izz   =   4.97e+07 * 1.3558  
        Izx   =  -970000.0 * 1.3558
        Ixy   =        0.0
        Iyz   =        0.0
        
        self.Iner = np.array( [ [  Ixx, -Ixy, -Izx ],
                                [ -Ixy,  Iyy, -Iyz ],
                                [ -Izx, -Iyz,  Izz ] ] )

        self.Iinv = np.linalg.inv( self.Iner )

        
        #
        #  Gear
        #
#         xNLG = ( 1327.0 -  396.0 ) * 25.4 / 1000.0
#         zNLG = (  206.0 -   24.0 ) * 25.4 / 1000.0
#         
#         xMLG = ( 1327.0 - 1554.0 ) * 25.4 / 1000.0
#         yMLG = (  216.5          ) * 25.4 / 1000.0
#         zMLG = (  216.0 -   24.0 ) * 25.4 / 1000.0
#        
       
        xNLG =  - 396.0 * 25.4 / 1000.0
        zNLG =    206.0 * 25.4 / 1000.0
        
        xMLG = - 1554.0 * 25.4 / 1000.0
        yMLG =    216.5 * 25.4 / 1000.0
        zMLG =    216.0 * 25.4 / 1000.0
       
        self.gearPos = np.array( [ [  xNLG,  0.0,  zNLG ],
                                   [  xMLG, -yMLG, zMLG ],
                                   [  xMLG,  yMLG, zMLG ] ] )
        
        #
        ang  = np.radians( 10.0 )
        c, s = np.cos( ang ), np.sin( ang )
        self.gearDir = np.array( [ [  -s , 0.0,  -c  ],
                                   [  0.0, 0.0, -1.0 ],
                                   [  0.0, 0.0, -1.0 ] ] )
        
        for k, gDir in enumerate( self.gearDir ) :
            self.gearDir[k] = gDir / np.linalg.norm( gDir )
            
            
        
        #
        #  1 lbf = 0.45359 kgf = 0.45359 * 9.80665 N
        #        = 4.4482 N
        #
        K_NLG =  22000.0 * 4.4482 / 0.3048 #  lbf / ft -> N / m
        K_MLG = 150000.0 * 4.4482 / 0.3048 
        
        C_NLG =  87302.6 * 4.4482 / 0.3048 #  lbf / ft / s -> N / m / s
        C_MLG = 174605.0 * 4.4482 / 0.3048 #  lbf / ft / s -> N / m / s
        
        self.gearKC  = np.array( [ [ K_NLG, C_NLG ],
                                   [ K_MLG, C_MLG ],
                                   [ K_MLG, C_MLG ] ] )
        
        self.muMax = 1.0
        
        
        #
        #  Engine
        #
        
        self.thst_ref = 4.0 * 58000.0 * 0.4536 * 9.80665
 
        ang = np.radians( 0.0 )
        self.thstDir = np.array( [ np.cos( ang ), 0.0, - np.sin( ang ) ]  ) 
 
 
#         self.deMax = np.radians( 30.0 )
#         self.daMax = np.radians( 30.0 )
#         self.drMax = np.radians( 30.0 )
        
        self.daRange = [ -0.350, 0.350 ]
        self.deRange = [ -0.350, 0.175 ]
        self.drRange = [ -0.350, 0.350 ]
        
        self.deMax = 0.350
        self.daMax = 0.350
        self.drMax = 0.350

        self.dFlapMax_deg = 30.0
        
        self.alp_CLmax = 0.23      
    
    
        self.timeConst_thst = 1.0
        self.timeConst_flap = 1.0
        self.timeConst_sb   = 1.0
        self.timeConst_gear = 1.0
    
    
#         self.rangeDe  = ( -0.350, 0.175 )
#         self.rangeAlp = ( -0.23 , 0.23 )
#         self.rangeThr = (  0.0  , self.thst_ref )
        
        
        #  timeFlap = 3.0
        #  timeGear = 5.0
        #  timeSB   = 1.0
        

    def calcAeroCoeff( self ) :
        #print( 'calcAeroCoeff DIYAircraft' )
        #
        #
        #
        #  self.alp, self.bet, self.mach self.da , self.de , self.dr
        #  pqr, alpDot
        #
        pAst = 0.0 if self.Vt < 0.01 else 0.5 * self.Bbar / self.Vt * self.pqr[0]
        qAst = 0.0 if self.Vt < 0.01 else 0.5 * self.Cbar / self.Vt * self.pqr[1]
        rAst = 0.0 if self.Vt < 0.01 else 0.5 * self.Bbar / self.Vt * self.pqr[2]
        aAst = 0.0 if self.Vt < 0.01 else 0.5 * self.Cbar / self.Vt * self.alpDot
        
        alp, bet, mach = self.alp, self.bet, self.mach
        da , de , dr   = self.da , self.de , self.dr
        
        flap_pos_deg = self.dFlapMax_deg * self.flap_pos_norm
        
        #   CL
        CL_alpha = np.interp( alp, [ -0.20, 0.00, 0.23, 0.60 ], [ -0.68, 0.20, 1.20, 0.60 ] )
        #CL_alpha = np.interp( alp, [ -0.20, 0.00, 0.23, 0.60 ], [ -0.48, 0.00, 1.00, 0.40 ] )
        CL_flap  =   0.05   * flap_pos_deg
        #CL_flap  =   0.03   * flap_pos_deg
        CL_sb    = - 0.0800 * self.speedbrake_pos_norm
        CL_de    =   0.2000 * self.de
        self.CL = CL_alpha + CL_flap + CL_sb + CL_de   
        
        #   CD
        CD0     = np.interp( alp,  [ -1.57, -0.26 , 0.0  ,  0.26, 1.57 ],
                                   [  1.5 ,  0.034, 0.017, 0.034, 1.5  ] )
        CDi     = 0.0420 * self.CL * self.CL
        CD_mach = np.interp( mach, [ 0.0, 0.79, 1.1  ,  1.8  ],
                                   [ 0.0, 0.0 , 0.023, 0.015 ] )
        CD_flap = 0.001833 * flap_pos_deg
        CD_gear = 0.0110   * self.gear_pos_norm
        CD_sb   = 0.0170   * self.speedbrake_pos_norm
        CD_beta = np.interp( bet,  [ -1.57, -0.26, 0.0, 0.26, 1.57 ],
                                   [  1.23,  0.05, 0.0, 0.05, 1.23 ] )
        CD_de   = abs( 0.0550 * self.de )
        self.CD = CD0 + CDi + CD_mach + CD_flap + CD_gear + CD_sb + CD_beta + CD_de
        
        #   Cm
        Cm_alpha    = -0.7000 * alp
        Cm_de       = de * np.interp( mach, [  0.0,  2.0   ],
                                            [ -1.3, -0.325 ] )
        Cm_q        = -21.0000 * qAst
        Cm_alphaDot = -0.4000  * aAst
        self.Cm = Cm_alpha + Cm_de + Cm_q + Cm_alphaDot 
        
        #   Cy
        #self.Cy = 1.0000 * bet
        self.Cy = -1.0000 * bet
        
        #   Cl
        Cl_beta = - 0.1000 * bet
        Cl_p    = - 0.4000 * pAst
        Cl_r    =   0.1500 * rAst
        Cl_da   =   da * np.interp( mach, [ 0.0, 2.0   ],
                                          [ 0.1, 0.033 ] )
        Cl_dr   =   0.0100 * dr
        self.Cl = Cl_beta + Cl_p + Cl_r + Cl_da + Cl_dr        
        
        #   Cn
        Cn_beta =   0.1200 * bet
        Cn_r    = - 0.1500 * rAst
        Cn_dr   = - 0.1000 * dr
        Cn_da   =   0.0000 * da
        self.Cn = Cn_beta + Cn_r + Cn_dr + Cn_da
        
        #   Cx, Cz
        ca, sa = np.cos( self.alp ), np.sin( self.alp )
        self.Cx =   sa * self.CL - ca * self.CD
        self.Cz = - ca * self.CL - sa * self.CD
 



#----------------------------------------------------

class DIYAircraft_F15( DIYAircraft ) :
    
    def __init__( self ) :
        print( '__init__ DIYAircraft_B747' )
        
        super().__init__()


    def setParam( self ) :
          
        #
        self.Sref = 608.0  * 0.3048 * 0.3048 
        self.Cbar =  15.95 * 0.3048
        self.Bbar =  42.83 * 0.3048
        
        self.xyzCG      = np.array( [   236.39, 0.0, -4.5  ] ) * 0.0254
        self.xyzAeroRef = np.array( [   234.15, 0.0,  0.0  ] ) * 0.0254
        self.xyzThrust  = np.array( [   -50.0 , 0.0,  0.0  ] ) * 0.0254
        self.xyzPilot   = np.array( [   398.45, 0.0, -4.58 ] ) * 0.0254

      
        #
        #  1 slug = 1 lb s^2 / ft =  0.45359 / 0.3048 kgf s^2 / m
        #                         =  0.45359 / 0.3048 * 9.80665 kg
        #                         =  14.5939 kg
        #  1 slug ft^2            = 14.5939 * 0.3048^2 kg m^2
        #                         =  1.3558 kg
        #
        self.Mass  =    28000.0 * 0.45359
        
        Ixx   =    28700.0 * 1.3558 #  slug ft^2 -> kg m^2
        Iyy   =   165100.0 * 1.3558 
        Izz   =   187900.0 * 1.3558  
        Izx   =      520.0 * 1.3558
        Ixy   =        0.0
        Iyz   =        0.0
        
        self.Iner = np.array( [ [  Ixx, -Ixy, -Izx ],
                                [ -Ixy,  Iyy, -Iyz ],
                                [ -Izx, -Iyz,  Izz ] ] )

        self.Iinv = np.linalg.inv( self.Iner )

        
        #
        #  Gear
        #
       
        xNLG =    393.2 * 25.4 / 1000.0
        zNLG =     89.2 * 25.4 / 1000.0
        
        xMLG =   187.0 * 25.4 / 1000.0
        yMLG =    54.0 * 25.4 / 1000.0
        zMLG =    85.4 * 25.4 / 1000.0
       
        self.gearPos = np.array( [ [  xNLG,  0.0,  zNLG ],
                                   [  xMLG, -yMLG, zMLG ],
                                   [  xMLG,  yMLG, zMLG ] ] )
        
        #
        ang  = np.radians( 0.0 )
        c, s = np.cos( ang ), np.sin( ang )
        self.gearDir = np.array( [ [  -s , 0.0,  -c  ],
                                   [  0.0, 0.0, -1.0 ],
                                   [  0.0, 0.0, -1.0 ] ] )
        
        for k, gDir in enumerate( self.gearDir ) :
            self.gearDir[k] = gDir / np.linalg.norm( gDir )
            
            
        #
        #  1 lbf = 0.45359 kgf = 0.45359 * 9.80665 N
        #        = 4.4482 N
        #
        K_NLG =  12200.0 * 4.4482 / 0.3048 #  lbf / ft -> N / m
        K_MLG =   5000.0 * 4.4482 / 0.3048 
        
        C_NLG =  22704.0 * 4.4482 / 0.3048 #  lbf / ft / s -> N / m / s
        C_MLG =   9000.0 * 4.4482 / 0.3048 #  lbf / ft / s -> N / m / s
        
        self.gearKC  = np.array( [ [ K_NLG, C_NLG ],
                                   [ K_MLG, C_MLG ],
                                   [ K_MLG, C_MLG ] ] )
        
        self.muMax = 1.0
        
        
        #
        #  Engine
        #
        
        self.thst_ref = 2.0 * 17800.0 * 0.4536 * 9.80665
 
        ang = np.radians( 0.0 )
        self.thstDir = np.array( [ np.cos( ang ), 0.0, - np.sin( ang ) ]  ) 
 

#         self.daRange = [ -0.350, 0.350 ]
#         self.deRange = [ -0.350, 0.175 ]
#         self.drRange = [ -0.350, 0.350 ]
        
        self.deMax = 0.350
        self.daMax = 0.350
        self.drMax = 0.350

        self.dFlapMax_deg = 30.0
        
        self.alp_CLmax = 0.23      
    
    
        self.timeConst_thst = 1.0
        self.timeConst_flap = 1.0
        self.timeConst_sb   = 1.0
        self.timeConst_gear = 1.0
    
    

    def calcAeroCoeff( self ) :
        #print( 'calcAeroCoeff DIYAircraft' )
        #
        #
        #
        #  self.alp, self.bet, self.mach self.da , self.de , self.dr
        #  pqr, alpDot
        #
        pAst = 0.0 if self.Vt < 0.01 else 0.5 * self.Bbar / self.Vt * self.pqr[0]
        qAst = 0.0 if self.Vt < 0.01 else 0.5 * self.Cbar / self.Vt * self.pqr[1]
        rAst = 0.0 if self.Vt < 0.01 else 0.5 * self.Bbar / self.Vt * self.pqr[2]
        aAst = 0.0 if self.Vt < 0.01 else 0.5 * self.Cbar / self.Vt * self.alpDot
        
        alp, bet, mach = self.alp, self.bet, self.mach
        da , de , dr   = self.da , self.de , self.dr
        
        flap_pos_deg = self.dFlapMax_deg * self.flap_pos_norm
        
        hb = self.alt_sl_ft / self.Bbar
        kCLge = np.interp( hb,
                [ 0.0  , 0.1  , 0.15 , 0.2  , 0.3  , 0.4  , 0.5  , 0.6  , 0.7  , 0.8, 0.9, 1.0, 1.1 ],
                [ 1.061, 1.033, 1.031, 1.033, 1.028, 1.011, 1.009, 1.005, 1.002, 1.0, 1.0, 1.0, 1.0 ] )
        
        kCDge = np.interp( hb,
                [ 0.0 , 0.1 , 0.15, 0.2 , 0.3 , 0.4 , 0.5 , 0.6 , 0.7 , 0.8 , 0.9, 1.0, 1.1 ],
                [ 0.81, 0.99, 0.95, 0.98, 0.99, 1.00, 1.04, 1.01, 0.99, 1.01, 1.0, 1.0, 1.0 ] )
        
        kCmge = np.interp( hb,
                [ 0.0  , 0.1  , 0.15 , 0.2  , 0.3  , 0.4  , 0.5  , 0.6  , 0.7  , 0.8  , 0.9, 1.0, 1.1 ],
                [ 0.992, 0.996, 0.999, 0.997, 1.000, 1.000, 1.002, 1.001, 1.998, 0.999, 1.0, 1.0, 1.0 ] )
        
        # LIFT 
        #      CLalpha    = qbar-psf * Sw-sqft * kCLge func( alpha_rad )
        #      CLDe         = qbar-psf * Sw-sqft * kCLge ( 0.5730 ) * de_rad
        #      CLadot      = qbar-psf * Sw-sqft * kCLge (  17.2232 ) * ci2vel  * alphadot-rad_sec
        #      CLq            = qbar-psf * Sw-sqft * kCLge ( -17.2232 ) * ci2vel  * q-aero-rad_sec
        #
        CLaList = [ [ -0.349, -0.262, -0.175, -0.087, 0.000, 0.07 , 0.14 , 0.209,
                       0.279,  0.349,  0.419,  0.489, 0.559, 0.698, 0.785, 0.873  ],
                    [ -0.93 , -0.86 , -0.58 , -0.26 , 0.06 , 0.3  , 0.58 , 0.78 ,
                       0.88 ,  1.01 ,  1.08 ,  1.16 , 1.19 , 1.17 , 1.09 , 1.05 ],
                    [ -0.25 , -0.23 , -0.16 , -0.07 , 0.02 , 0.08 , 0.16 , 0.21,
                       0.24 ,  0.27 ,  0.29 ,  0.31 , 0.32 , 0.31 , 0.29 , 0.28  ] ]
    
        if mach < 0.5 :
            CLalpha = np.interp( alp, CLaList[0], CLaList[1] )
        elif mach < 1.4 :
            CLa1 = np.interp( alp, CLaList[0], CLaList[1] )
            CLa2 = np.interp( alp, CLaList[0], CLaList[2] )
            CLalpha = ( ( 1.4 - mach ) * CLa1 + ( mach - 1.5 ) * CLa2 ) / 0.9 
        else :
            CLalpha = np.interp( alp, CLaList[0], CLaList[2] )
            
        CLDe   =   0.5730 * de
        CLadot =  17.2232 * aAst
        CLq    = -17.2232 * qAst

        self.CL = kCLge * ( CLalpha + CLDe + CLadot + CLq )  
            

        # DRAG 
        #      CDalpha = qbar-psf * Sw-sqft * kCDge  func( alpha_rad, mach )
        #      CDi          = qbar-psf * Sw-sqft * kCDge  func( alpha_rad )
        #      CDbeta   = qbar-psf * Sw-sqft * kCDge  func( alpha_rad ) |  beta_rad  |
        # 
        CDaList = [ [ -0.419, -0.279,  -0.14, -0.07,  0.0   , 0.07, 0.14 , 0.279, 0.419, 0.559, 0.698 ],
                    [  0.38 ,  0.22 ,  0.07 ,  0.04,  0.0147, 0.04, 0.07 , 0.22 , 0.38 , 0.8  , 1.01  ],
                    [ 0.097 ,  0.025,  0.018,  0.01,  0.0074, 0.01, 0.018, 0.025, 0.097, 0.203, 0.257 ] ]

        if mach < 0.5 :
            CDalpha = np.interp( alp, CDaList[0], CDaList[1] )
        elif mach < 1.4 :
            CLa1 = np.interp( alp, CDaList[0], CDaList[1] )
            CLa2 = np.interp( alp, CDaList[0], CDaList[2] )
            CDalpha = ( ( 1.4 - mach ) * CLa1 + ( mach - 1.5 ) * CLa2 ) / 0.9 
        else :
            CDalpha = np.interp( alp, CDaList[0], CDaList[2] )
    
        CDi = np.interp( alp,
              [ -0.349, -0.262, -0.175, -0.087, 0.0  , 0.07 , 0.14 , 0.209,
                 0.279,  0.349,  0.419,  0.489, 0.559, 0.698, 0.785, 0.873 ],
              [  0.227,  0.067,  0.048,  0.006, 0.006, 0.013, 0.02 , 0.047,
                 0.067,  0.212,  0.227,  0.331, 0.474, 0.212, 0.913, 0.721 ] )
    
        CDbeta = abs( bet ) * np.interp( alp,
              [ -0.349 , -0.262, -0.175 , -0.087, 0.0   ,  0.07 ,   0.14 ,  0.209,
                 0.279 ,  0.349,  0.419 ,  0.489, 0.559 ,  0.698,   0.785,  0.873  ],
              [  2.5954, 3.2686,  3.3563, 3.0533, 3.8228,  4.345,  4.1909,  3.7331,
                 3.1268, 2.8246,  2.4657, 2.2127, 1.9317, 1.2283,  0.9522,  0.7015 ] )
    
        self.CD = kCDge * ( CDalpha + CDi + CDbeta )  

        # PITCH 
        #     Cmalpha  = qbar-psf * Sw-sqft * cbarw-ft * kCmge * func( alpha_rad ) * alpha_rad
        #     Cmadot    = qbar-psf * Sw-sqft * cbarw-ft * ( -11.8870 ) * ci2vel* alphadot-rad_sec
        #     CmM         = qbar-psf * Sw-sqft * cbarw-ft * ( -0.0000 ) * mach
        #     Cmq          = qbar-psf * Sw-sqft * cbarw-ft * ( -4.7000 ) * ci2vel * q-aero-rad_sec
        #     Cmde        = qbar-psf * Sw-sqft * cbarw-ft * ( -0.4580 ) * elevator-pos-rad
        Cmalpha = alp * np.interp( alp,
              [ -0.349 , -0.262, -0.131 ,  0.0   ,  0.279 ,  0.419 ,  0.559 ,  0.873  ],
              [ -1.1459, -0.573, -0.0573, -0.2292, -0.5157, -0.8594, -0.4584, -0.5157 ] )

        Cmadot = -11.8870 * aAst
        CmM    =  -0.0000 * mach
        Cmq    =  -4.7000 * qAst
        Cmde   =  -0.4580 * de
        self.Cm = kCmge * Cmalpha + Cmadot + CmM + Cmq + Cmde


        # SIDE 
        #      CYb    = qbar-psf * Sw-sqft * func( alpha_rad ) * beta_rad 
        #      CYda  = qbar-psf * Sw-sqft * (  -0.0012   ) * da_rad
        #      CYdr   = qbar-psf * Sw-sqft * (  0.3724    ) * dr_rad
        CYb  = bet * np.interp( alp,
              [ -0.349 ,  0.07  ,  0.559 ,  0.873  ],
              [ -0.8594, -0.0974, -0.0115, -0.3552 ] )
        CYda  =  -0.0012  * da
        CYdr   =  0.3724  * dr
        self.Cy =  CYb + CYda + CYdr

        # ROLL 
        #      Clb    = qbar-psf * Sw-sqft * bw-ft   func( alpha_rad ) * beta_rad 
        #      Clp    = qbar-psf * Sw-sqft * bw-ft  func( alpha_rad )  * bi2vel * p-aero-rad_sec
        #      Clr     = qbar-psf * Sw-sqft * bw-ft  ( 0.0001 )               * bi2vel * r-aero-rad_sec
        #      ClDa  = qbar-psf * Sw-sqft * bw-ft  func( alpha_rad ) * bi2vel * left-aileron-pos-rad
        #      ClDr   = qbar-psf * Sw-sqft * bw-ft  func( 0.0115 ) * rudder-pos-rad
        Clb = bet * np.interp( alp,
              [ -0.349, -0.175 ,  0.0   ,  0.209 ,  0.314 ,  0.419 ,  0.698 ,  0.785 , 0.873  ],
              [ 0.0286, -0.2292, -0.0573, -0.1719, -0.1146, -0.1719, -0.0745, -0.1146, -0.1203] )
        Clp   = pAst * np.interp( alp,
              [ 0.0000 ,  0.3490 ],
              [ -0.4000, -0.3100 ] )
        Clr   =  0.0001 * rAst
        ClDa  = da * np.interp( alp,
              [ -0.349 , -0.087 , 0.14  , 0.559 , 0.873  ],
              [  0.0172,  0.0372, 0.0401, 0.0057, 0.0057 ] )
        ClDr  = 0.0115  * dr
        self.Cl =  Clb + Clp + Clr + ClDa + ClDr

        # YAW 
        #     Cnb    = qbar-psf * Sw-sqft * bw-ft * func( alpha_rad ) * beta-rad
        #     Cnp    = qbar-psf * Sw-sqft * bw-ft * func( alpha_rad ) * bi2vel * p-aero-rad_sec
        #     Cnr     = qbar-psf * Sw-sqft * bw-ft * ( -0.4447 )             * bi2vel * r-aero-rad_sec
        #     Cnda  = qbar-psf * Sw-sqft * bw-ft * ( 0.0022 )              * left-aileron-pos-rad
        #     Cndr  = qbar-psf * Sw-sqft * bw-ft * func( alpha_rad ) * rudder-pos-rad
        Cnb = bet * np.interp( alp,
              [ -0.349, 0.0   , 0.349 ,  0.524 ,  0.873  ],
              [ 0.0745, 0.1776, 0.0057, -0.0974, -0.0286 ]  )
        Cnp = pAst * np.interp( alp,
              [ -0.349, -0.087,  0.0, 0.279, 0.419, 0.873  ],
              [  0.11 , -0.14 , -0.1, 0.01 , -0.13, -0.186 ]  )
        Cnr =   -0.4447 * rAst
        Cnda  = 0.0022   * da    
        Cndr  = dr * np.interp( alp,
              [ -0.349 ,  0.873  ],
              [ -0.0688, -0.0115 ] )
        self.Cn =  Cnb + Cnp + Cnr + Cnda + Cndr
    
        #   Cx, Cz
        ca, sa = np.cos( self.alp ), np.sin( self.alp )
        self.Cx =   sa * self.CL - ca * self.CD
        self.Cz = - ca * self.CL - sa * self.CD
 




 
class MainWindow( QMainWindow ) :

    def __init__( self, parent = None ) :
        
        super().__init__( parent )
        print( 'MainWindow init' )
    
        self.myId = 'MainWindow'
        
        self.setGeometry( 50, 50, 1200, 700 )
        self.setWindowTitle("Main Window")

        self.curTime = 0.0
        self.goFlag = False

        self.aircraft = DIYAircraft()

        mainLW = VBoxLayoutWidget()
        self.setCentralWidget( mainLW )

        btnsLW = mainLW.addWidget( HBoxLayoutWidget() )
        self.startBtn = btnsLW.addWidget( QPushButton( 'START' ) )
        self.startBtn.clicked.connect( self.startBtnClicked )
        
        self.resetBtn = btnsLW.addWidget( QPushButton( 'RESET' ) )
        self.resetBtn.clicked.connect( self.resetBtnClicked )
                
        self.btn1 = btnsLW.addWidget( QPushButton( 'thrust-' ) )
        self.btn2 = btnsLW.addWidget( QPushButton( 'thrust+' ) )
        self.btn3 = btnsLW.addWidget( QPushButton( 'brake-' ) )
        self.btn4 = btnsLW.addWidget( QPushButton( 'brake+' ) )
        
        self.btn1.clicked.connect( self.btnClicked )
        self.btn2.clicked.connect( self.btnClicked )
        self.btn3.clicked.connect( self.btnClicked )
        self.btn4.clicked.connect( self.btnClicked )


        self.graph = mainLW.addWidget( ManyTHGraphs( self, 0.0, 100.0,
                      [ [  0, 0, ( 'alt_ft', 'Vt>' ), ( 0.0, 30.0, 0.0, 100.0 ) ],
                        [  1, 0, ( 'phi', 'the', 'psi' ), ( -10.0, 10.0 ) ], 
                        [  2, 0, ( 'p', 'q', 'r' ), ( -2.0, 2.0 ) ], 
                        [  3, 0, ( 'de', 'da', 'dr' ), ( -10.0, 10.0 ) ], 
                        [  0, 1, ( 'alp', 'bet', 'gam' ), ( -10.0, 10.0 ) ], 
                        [  1, 1, ( 'def1', 'def2', 'def3' ), ( -10.0, 10.0 ) ], 
                        [  2, 1, ( 'XXX', 'YYY', 'ZZZ' ), ( -1.0e8, 1.0e8 ) ],
                        [  3, 1, ( 'XXX', 'YYY', 'ZZZ' ), ( -1.0, 1.0 ) ],
                        [  0, 2, ( 'airX', 'airY', 'airZ' ), ( -1.0e6, 1.0e6 ) ], 
                        [  1, 2, ( 'thrX', 'thrY', 'thrZ' ), ( -1.0e6, 1.0e6 ) ],
                        [  2, 2, ( 'grvX', 'grvY', 'grvZ' ), ( -1.0e6, 1.0e6 ) ],
                        
                        ] ) )


    def closeEvent( self, event ) :
        print( 'closeEvent' )
       
        
    def btnClicked( self ) :
        print( 'btnClicked', self.sender().text() )
        
        txt = self.sender().text()
        
        if txt == 'thrust-' :
            self.aircraft.th_norm -= 0.05
            if self.aircraft.th_norm < 0.0 :
                self.aircraft.th_norm = 0.0
            self.aircraft.thst = self.aircraft.thst_ref * self.aircraft.th_norm
            
        elif txt == 'thrust+' :
            self.aircraft.th_norm += 0.05
            if self.aircraft.th_norm > 1.0 :
                self.aircraft.th_norm = 1.0
            self.aircraft.thst = self.aircraft.thst_ref * self.aircraft.th_norm
            
        if txt == 'brake-' :
            mu = self.aircraft.gearMu[0]
            mu -= 0.1
            if mu < 0.0 : mu = 0.0
            self.aircraft.gearMu = np.array( [ mu, mu, mu ] )
            
        if txt == 'brake+' :
            mu = self.aircraft.gearMu[0]
            mu += 0.1
            if mu > 0.5 : mu = 0.5
            self.aircraft.gearMu = np.array( [ mu, mu, mu ] )
                        

        print( self.aircraft.th_norm, self.aircraft.thst )             
        

        
    def resetBtnClicked( self ) :
        print( 'resetBtnClicked' )

        self.curTime = 0.0
        self.aircraft.reset()
        self.graph.clearData()


    def startBtnClicked( self ) :
        print( 'startBtnClicked' )

        if self.startBtn.text() == 'START' :
            self.startBtn.setText( 'STOP' )

            self.iniTime = time.time() - self.curTime
            self.qtimer = QTimer()
            self.qtimer.setInterval( 100 )
            self.qtimer.timeout.connect( self.step )
            self.qtimer.start()                

        elif self.startBtn.text() == 'STOP' :
            self.startBtn.setText( 'START' )
            self.qtimer.stop()   


    def step( self ) :

        self.curTime = time.time() - self.iniTime
        print( 'curTime  : ', self.curTime )
        
        self.aircraft.step( self.curTime )
        

        self.graph.addData( [ self.curTime,
                              
                              self.aircraft.alt_ft,
                              self.aircraft.Vt,
                              
                              np.degrees( self.aircraft.eul[0] ),
                              np.degrees( self.aircraft.eul[1] ),
                              np.degrees( self.aircraft.eul[2] ),
                              
                              np.degrees( self.aircraft.pqr[0] ),
                              np.degrees( self.aircraft.pqr[1] ),
                              np.degrees( self.aircraft.pqr[2] ),
                              
                              np.degrees( self.aircraft.de ),
                              np.degrees( self.aircraft.da ),
                              np.degrees( self.aircraft.dr ),
                              
                              np.degrees( self.aircraft.alp ),
                              np.degrees( self.aircraft.bet ),
                              np.degrees( self.aircraft.gam ),
                              
                              0.0,
                              0.0,
                              0.0,
                              
                              #self.aircraft.gearDef[0],
                              #self.aircraft.gearDef[1],
                              #self.aircraft.gearDef[2],
                              
                              #self.aircraft.alp,
                              #self.aircraft.YYY,
                              #self.aircraft.ZZZ,
                              
                              self.aircraft.moment[0],
                              self.aircraft.moment[1],
                              self.aircraft.moment[2],
                              
                              self.aircraft.pqr[0],
                              self.aircraft.pqr[1],
                              self.aircraft.pqr[2],
                              
                              #self.aircraft.gearLoad[0],
                              #self.aircraft.gearLoad[1] ,
                              #self.aircraft.gearLoad[2] ,
                              
                              self.aircraft.airFG[0],
                              self.aircraft.airFG[1],
                              self.aircraft.airFG[2],
                              
                              self.aircraft.thrFG[0],
                              self.aircraft.thrFG[1],
                              self.aircraft.thrFG[2],
                              
                              self.aircraft.grvFG[0],
                              self.aircraft.grvFG[1],
                              self.aircraft.grvFG[2],
                              
                              ]  )
        
        




if __name__ == "__main__" :
    QApplication.setAttribute( Qt.ApplicationAttribute.AA_ShareOpenGLContexts )
    app = QApplication( sys.argv )
    mainwindow = MainWindow()
    mainwindow.show()
    app.exec()

    

    