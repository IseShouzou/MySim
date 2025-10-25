import numpy as np
import math


def qua2dcm( q ) :
    q11, q12, q13, q14 = q[0]*q[0], 2.0*q[0]*q[1], 2.0*q[0]*q[2], 2.0*q[0]*q[3]
    q22, q23, q24      = q[1]*q[1], 2.0*q[1]*q[2], 2.0*q[1]*q[3]
    q33, q34           = q[2]*q[2], 2.0*q[2]*q[3]
    q44                = q[3]*q[3]
    return np.array( [ [ q11-q22-q33+q44,     q12+q34    ,      q13-q24     ], 
                       [     q12-q34    , q22-q33-q11+q44,      q23+q14     ],
                       [     q13+q24    ,     q23-q14    ,  q33-q11-q22+q44 ] ] )

def dcm2euler( dcm ) :
    return [ math.atan2(   dcm[1][2], dcm[2][2] ), 
             math.atan2( - dcm[0][2], math.sqrt( dcm[1][2] * dcm[1][2] + dcm[2][2] * dcm[2][2] ) ), 
             math.atan2(   dcm[0][1], dcm[0][0] ) ]
    
def dcm2qua( dcm ) :
    q = np.array( [ np.sqrt( abs( 1.0 + dcm[0][0] - dcm[1][1] - dcm[2][2] ) ) / 2.0,
                    np.sqrt( abs( 1.0 - dcm[0][0] + dcm[1][1] - dcm[2][2] ) ) / 2.0,
                    np.sqrt( abs( 1.0 - dcm[0][0] - dcm[1][1] + dcm[2][2] ) ) / 2.0,
                    np.sqrt( abs( 1.0 + dcm[0][0] + dcm[1][1] + dcm[2][2] ) ) / 2.0 ] )
    ix = np.argmax( q )
    if ix == 0 :
        q[1] = 0.25 / q[0] * ( dcm[0][1] + dcm[1][0] )
        q[2] = 0.25 / q[0] * ( dcm[0][2] + dcm[2][0] )
        q[3] = 0.25 / q[0] * ( dcm[1][2] - dcm[2][1] )
    elif ix == 1 :
        q[0] = 0.25 / q[1] * ( dcm[0][1] + dcm[1][0] )
        q[2] = 0.25 / q[1] * ( dcm[2][1] + dcm[1][2] )
        q[3] = 0.25 / q[1] * ( dcm[2][0] - dcm[0][2] )
    elif ix == 2 :
        q[0] = 0.25 / q[2] * ( dcm[2][0] + dcm[0][2] )
        q[1] = 0.25 / q[2] * ( dcm[2][1] + dcm[1][2] )
        q[3] = 0.25 / q[2] * ( dcm[0][1] - dcm[1][0] )
    elif ix == 3 :
        q[0] = 0.25 / q[3] * ( dcm[1][2] - dcm[2][1] )
        q[1] = 0.25 / q[3] * ( dcm[2][0] - dcm[0][2] )
        q[2] = 0.25 / q[3] * ( dcm[0][1] - dcm[1][0] )
    return q

def euler2dcm( euler, degree = False ) :
    if degree :
        euler = np.radians( euler )
    c1, s1 = np.cos( euler[0] ), np.sin( euler[0] )
    c2, s2 = np.cos( euler[1] ), np.sin( euler[1] )
    c3, s3 = np.cos( euler[2] ), np.sin( euler[2] )
    return np.array( [ [ c2*c3           , c2*s3           , -s2    ], 
                       [ s1*s2*c3 - c1*s3, s1*s2*s3 + c1*c3,  s1*c2 ],
                       [ c1*s2*c3 + s1*s3, c1*s2*s3 - s1*c3,  c1*c2 ] ] )

def qua2euler( qua ) :
    return dcm2euler( qua2dcm( qua ) )

def euler2qua( euler ) :
    return dcm2qua( euler2dcm( euler ) )



class ZeroSeeker :
    
    def __init__( self, xa, xb ) :
        self.cnt  = 0
        self.xc   = xa
        self.xb   = xb
        self.flag = False
        
    def nextValue( self, y ) :
        self.cnt += 1
        if self.cnt == 1 :
            self.xa, self.ya = self.xc, y
            self.xc = self.xb
        elif not self.flag :
            if ( self.ya >= 0.0 ) == ( y >= 0.0 ) :
                if abs( y ) > abs( self.ya ) :
                    self.xc = 2.0 * self.xa - self.xc
                else :
                    self.xc = 2.0 * self.xc - self.xa
            else :
                self.flag = True
                self.xb, self.yb = self.xc, y
                self.xc = ( self.yb * self.xa - self.ya * self.xb ) / ( self.yb - self.ya )
        else :
            if ( self.ya >= 0.0 ) == ( y >= 0.0 ) :
                self.xa, self.ya = self.xc, y
            else :
                self.xb, self.yb = self.xc, y
            if self.cnt%2 == 0 :
                self.xc = ( self.yb * self.xa - self.ya * self.xb ) / ( self.yb - self.ya )
            else :
                self.xc = ( self.xa + self.xb ) / 2.0
        return self.xc, y
            

class MinSeeker :
    
    def __init__( self, xa, xb ) :
            
        self.A1 = 0.381966011250105
        self.A2 = 0.618033988749895
        
        self.cnt = 0
        self.id  = 0
        self.x   = [ xa,
                     self.A2 * xa + self.A1 * xb,
                     self.A1 * xa + self.A2 * xb,
                     xb ]
        self.y    = [ 0.0, 0.0, 0.0, 0.0 ]
         
    def nextValue( self, y ) :
        self.cnt += 1
        self.y[ self.id ] = y
        if self.cnt <= 2 :
            self.id += 1
        else :
            if self.y[1] < self.y[2] :
                self.x[3], self.y[3] = self.x[2], self.y[2]
                self.x[2], self.y[2] = self.x[1], self.y[1]
                self.x[1] = self.A1 * self.x[0] + self.A2 * self.x[2]
                self.id = 1
            else :
                self.x[0], self.y[0] = self.x[1], self.y[1]
                self.x[1], self.y[1] = self.x[2], self.y[2]
                self.x[2] = self.A2 * self.x[2] + self.A1 * self.x[3]
                self.id = 2
        return self.x[ self.id ], abs( self.x[0] - self.x[3] )
                

    