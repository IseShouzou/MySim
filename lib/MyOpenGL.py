import sys
import pickle

import numpy as np

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
from PyQt6.QtOpenGL import *
from PyQt6.QtOpenGLWidgets import QOpenGLWidget

from PIL import Image, ImageOps
from PIL.ImageQt import ImageQt

from lib.MyFunc import *


def calcLookAtMat( e, t, u ) :
    z = np.array( e ) - np.array( t )
    z /= np.linalg.norm( z )
    x = np.cross( np.array( u ), z )
    x /= np.linalg.norm( x )
    y = np.cross( z, x )
    mat = np.identity(4)
    mat[ 0, 0:3 ] = x
    mat[ 1, 0:3 ] = y
    mat[ 2, 0:3 ] = z
    mat[ 0:3, 3 ] = -np.dot( mat[ 0:3, 0:3 ], e )
    return mat


def calcLookAtMat_XYZEuler( xyz, euler, degree = True ) :
    
#     m = np.array( [ [  0.0,  1.0,  0.0,  0.0 ],
#                     [  0.0,  0.0, -1.0,  0.0 ],
#                     [ -1.0,  0.0,  0.0,  0.0 ],
#                     [  0.0,  0.0,  0.0,  1.0 ] ] )
            
    mat = np.identity( 4 )  
    mat[ 0:3, 0:3 ] = euler2dcm( euler, degree )
    mat[ 0:3, 3   ] = - np.dot( mat[ 0:3, 0:3 ], xyz )
        
    return mat
    #return np.dot( m, mat )




def perspectiveMatrix( fovy, asp, near, far ) :
    cot = 1.0 / np.tan( np.radians( fovy / 2.0 ) )
    dz = far - near
    mat = np.zeros((4,4))
    
    mat[0,0] =  cot / asp
    mat[1,1] =  cot
    mat[2,2] = - (far + near)   / dz
    mat[3,2] = - 1.0
    mat[2,3] = - 2.0 * far * near / dz
    
#     mat[0,1] =    cot / asp
#     mat[1,2] =  - cot
#     mat[2,0] =    (far + near)   / dz
#     mat[3,0] =    1.0
#     mat[2,3] =    - 2.0 * far * near / dz
    
    return mat


def calcLightLookAt( z, mm ) :
    z /= - np.linalg.norm( z )
    u = np.zeros( 3 )
    u[ np.abs ( z ).argmin() ] = 1.0
    x = np.cross( u, z )
    x /= np.linalg.norm( x )
    y = np.cross( z, x )
    mat = np.identity(4)
    mat[ 0, 0:3 ] = x
    mat[ 1, 0:3 ] = y
    mat[ 2, 0:3 ] = z
    MM = np.array( [ [ mm[0,0], mm[0,0], mm[0,0], mm[0,0], mm[0,1], mm[0,1], mm[0,1], mm[0,1] ],
                     [ mm[1,0], mm[1,0], mm[1,1], mm[1,1], mm[1,0], mm[1,0], mm[1,1], mm[1,1] ],
                     [ mm[2,0], mm[2,1], mm[2,0], mm[2,1], mm[2,0], mm[2,1], mm[2,0], mm[2,1] ],
                     [ 1.0    , 1.0    , 1.0    , 1.0    , 1.0    , 1.0    , 1.0    , 1.0     ] ] )
    MX = np.dot( mat, MM )
    for i in range( 3 ) :
        Min = np.min( MX[i,:] )
        Max = np.max( MX[i,:] )
        Del = max( 0.001, Max - Min )
        Min -= Del * 0.1
        Max += Del * 0.1
        mat[ i, 0:3 ] *= 2.0 / ( Max - Min )
        mat[ i, 3   ] = - ( Max + Min ) / ( Max - Min )
    return mat

def corners( mm ) :
    return np.array( [ [ mm[0,0], mm[0,0], mm[0,0], mm[0,0], mm[0,1], mm[0,1], mm[0,1], mm[0,1] ],
                       [ mm[1,0], mm[1,0], mm[1,1], mm[1,1], mm[1,0], mm[1,0], mm[1,1], mm[1,1] ],
                       [ mm[2,0], mm[2,1], mm[2,0], mm[2,1], mm[2,0], mm[2,1], mm[2,0], mm[2,1] ],
                       [ 1.0    , 1.0    , 1.0    , 1.0    , 1.0    , 1.0    , 1.0    , 1.0     ] ] )

def calcMinMax( cc ) :
    return np.array( [ [ np.min( cc[i,:] ), np.max( cc[i,:] ) ] for i in range( 3 ) ] )

def dotMinMax( mat, mm ) :
    return calcMinMax( np.dot( mat, corners( mm ) ) )

def addMinMax( mm1, mm2 ) :
    return np.array( [ [ min( mm1[i,0], mm2[i,0] ), max( mm1[i,1], mm2[i,1] ) ] for i in range( 3 ) ] )



class OBJ :
    
    def __init__( self ) :
        
        self.posMat = np.identity( 4 )
        self.color  = None
        self.skyFlag  = False

        self.name = ''

class Obje( OBJ ) :
    '''
        GL_POINTS
        GL_LINE_STRIP
        GL_LINE_LOOP
        GL_LINES
        GL_LINE_STRIP_ADJACENCY
        GL_LINES_ADJACENCY
        GL_TRIANGLE_STRIP
        GL_TRIANGLE_FAN
        GL_TRIANGLES
        GL_TRIANGLE_STRIP_ADJACENCY
        GL_TRIANGLES_ADJACENCY
        GL_PATCHES
    '''
#     def __init__(self, typ = None, positions = None, normals = None, indices = None, uvs = None, bufObjName = None ) :
    def __init__(self, typ = None, positions = None, normals = None, indices = None, uvs = None ) :
        
        super().__init__()        
        
        self.color = None
        self.shadowFlag     = True
        self.depthTestFlag  = True
        self.resetDepthFlag = False
        self.shadowFactor   = 0.2
        
        self.texFlag = False
        self.texture = None
        #self.bufObjName = None
        
        if typ is not None :
            self.setGeometry( typ, positions, normals, indices, uvs )


    def deleteBuffer( self ) :
        
        #print( 'deleteBuffer Obje' )
        #print( self.vbos )
        #print( self.ibo )
        
        for vbo in self.vbos :
            #print( vbo )
            #glBindBuffer(GL_ARRAY_BUFFER, 0)
            #glUnmapBuffer( GL_ARRAY_BUFFER )
            #glBindBuffer(GL_ARRAY_BUFFER, 0)
            glDeleteBuffers( 1, [ vbo[0] ] )
            
        #glUnmapBuffer( GL_ELEMENT_ARRAY_BUFFER )
        #glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)
        glDeleteBuffers( 1, [ self.ibo ] )    
        
        return 
        
 

    def setGeometry( self, typ, positions, normals, indices, uvs = None ) :
        #print('setGeometry Obje')
    
        positions = np.array( positions ).flatten()
        normals   = np.array( normals ).flatten()
        indices   = np.array( indices ).flatten()
        if not uvs is None :
            uvs = np.array( uvs ).flatten()

        pp = np.array( positions ).reshape( ( -1, 3 ) )
        self.MinMax = np.array( [ [ np.min( pp[:,i] ), np.max( pp[:,i] ) ] for i in range( 3 ) ]  )

        nVertex = 3
        nNormal = 3

        self.type    = typ
        self.numElm = len( indices )
        self.vbos   = []
    
        vao = glGenVertexArrays(1)      
        glBindVertexArray(vao)
        vbo = glGenBuffers(1)
        n = len( positions )
        bytelength = n * 4
        data = np.array( positions, dtype = np.float32 )
        glBindBuffer(GL_ARRAY_BUFFER, vbo) 
        glBufferData(GL_ARRAY_BUFFER, bytelength, data, GL_STATIC_DRAW)
        #glEnableVertexAttribArray(0)
        #glVertexAttribPointer(0, nVertex, GL_FLOAT, GL_FALSE, 0, None)
        self.vbos.append( [ vbo, 0, nVertex ] )
         
        vbo = glGenBuffers(1)
        n = len( normals )
        bytelength = n * 4
        data = np.array( normals, dtype = np.float32 )
        glBindBuffer(GL_ARRAY_BUFFER, vbo) 
        glBufferData(GL_ARRAY_BUFFER, bytelength, data, GL_STATIC_DRAW)
        #glEnableVertexAttribArray(1)
        #glVertexAttribPointer(1, nNormal, GL_FLOAT, GL_FALSE, 0, None)
        self.vbos.append( [ vbo, 1, nNormal ] )

        if not uvs is None :
            vbo = glGenBuffers(1)
            n = len( uvs )
            bytelength = n * 4
            data = np.array( uvs, dtype = np.float32 )
            glBindBuffer(GL_ARRAY_BUFFER, vbo) 
            glBufferData(GL_ARRAY_BUFFER, bytelength, data, GL_STATIC_DRAW)
            #glEnableVertexAttribArray(2)
            #glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, None)
            self.vbos.append( [ vbo, 2, 2 ] )
        
        ibo = glGenBuffers(1)
        n = len( indices )
        bytelength = n * 4
        data = np.array( indices, dtype = np.int32 )
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, bytelength, data, GL_STATIC_DRAW)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)
        self.ibo = ibo

        glBindVertexArray(0)
                        
                          

    def getMinMax( self ) :
        #return dotMinMax( self.posMat, self.bufObj.MinMax )
        return dotMinMax( self.posMat, self.MinMax )


    def setTexture( self, imgData ) :
        #print('setTexture')
        self.texFlag = True
        w, h = imgData.size
        img  = imgData.tobytes()
        self.tex = glGenTextures( 1 )
        glBindTexture( GL_TEXTURE_2D, self.tex )
        gluBuild2DMipmaps( GL_TEXTURE_2D, GL_RGB, w, h, GL_RGB, GL_UNSIGNED_BYTE, imgData.tobytes() )
        #glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
        #glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
        #glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE)
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL) 
        glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE)
        glBindTexture( GL_TEXTURE_2D, 0)


    def draw( self, gl, posMat = np.identity(4), color = None ) :
        #print('draw Obje')
        
        
#         if self.name == 'Terrain' :
#             print( 'draw Terrain' )

#         if self.skyFlag :
#             #print('draw Obje skyFlag')
#             lookAtMat = gl.lookAtMat
#             e = - np.dot( lookAtMat[ 0:3, 0:3 ].T, lookAtMat[ 0:3, 3 ] )
#             self.posMat[ 0:3, 3 ] = e
        
        posMat = np.dot( posMat, self.posMat)
        if color is None :
            if self.color is not None :
                color = self.color
            else :
                color = [ 1.0, 0.0, 0.0, 1.0 ]
                
        color = np.array( color )          

        if gl.currentProgram == 0 :
            if not self.shadowFlag : return
            glUniformMatrix4fv( gl.posiMatLoc0, 1, GL_TRUE, posMat )
        
        elif gl.currentProgram == 1 :
            glUniformMatrix4fv( gl.posiMatLoc1, 1, GL_TRUE, posMat )
            glUniform1f ( gl.shadowFactorLoc1, self.shadowFactor )
            if len( color == 4 ) :
                glUniform4fv( gl.mtlAmbiLoc1, 1, color * 1.0 )
                glUniform4fv( gl.mtlDiffLoc1, 1, color * 0.7 )
                glUniform4fv( gl.mtlSpecLoc1, 1, color * 1.0 )
                glUniform1f ( gl.mtlShinLoc1, 5.0 )
            elif len( color == 8 ) :
                c = color[0:4]
                glUniform4fv( gl.mtlAmbiLoc1, 1, c * color[4] )
                glUniform4fv( gl.mtlDiffLoc1, 1, c * color[5] )
                glUniform4fv( gl.mtlSpecLoc1, 1, c * color[6] )
                glUniform1f ( gl.mtlShinLoc1, color[7] )
        
#             if self.skyFlag :
#                 glUniform1i( gl.skyFlagLoc1, 1 )
#             else :
#                 glUniform1i( gl.skyFlagLoc1, 0 )

            if self.texFlag :
                glUniform1i( gl.texFlagLoc1, 1 )
                glUniform1i( gl.texLoc1 , 1 )
                glActiveTexture( GL_TEXTURE1 )
                glBindTexture(GL_TEXTURE_2D, self.tex )
            else :
                glUniform1i( gl.texFlagLoc1, 0 )
               
        if self.depthTestFlag :
            glEnable( GL_DEPTH_TEST )
            glDepthFunc( GL_LESS )
        else :
            #glDisable( GL_DEPTH_TEST )
            glEnable( GL_DEPTH_TEST )
            glDepthFunc( GL_ALWAYS )

        #    
        #   draw object  
        #    
        for vbo in self.vbos :
            glBindBuffer( GL_ARRAY_BUFFER, vbo[0] )
            glEnableVertexAttribArray( vbo[1] )
            glVertexAttribPointer( vbo[1], vbo[2], GL_FLOAT, GL_FALSE, 0, None )
        glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, self.ibo )
        glDrawElements( self.type, self.numElm, GL_UNSIGNED_INT, None )
        
#         self.bufObj.draw()
        
#         glBindVertexArray( self.vao )
#         glDrawElements( self.type, self.numElm, GL_UNSIGNED_INT, None )
#         glBindVertexArray( 0 )       
                
        
        if self.resetDepthFlag :
            glClear( GL_DEPTH_BUFFER_BIT )

        if gl.currentProgram == 1 and self.texFlag :
            glBindTexture( GL_TEXTURE_2D, 0 )
            





class ObjeCollection( OBJ ) :

    def __init__( self ) :
        
        super().__init__()
        
        self.objeList = []
        self.shadowFlag    = True
        self.depthTestFlag = True
            
    def getMinMax( self ) :
        mm = self.objeList[0].getMinMax()
        for obj in self.objeList[1:] :
            mm = addMinMax( mm, obj.getMinMax() )
        return dotMinMax( self.posMat, mm )
        
        
    def add( self, obj ) :
        self.objeList.append( obj )
        return obj
        
        
    def deleteBuffer( self ) :
        #print( 'deleteBuffer ObjeCollection', len( self.objeList ) )
        for obj in self.objeList :
            obj.deleteBuffer()
            del obj
        
        
    def draw( self, gl, posMat = None, color = None ) :
              
        if posMat is None :
            posMat = self.posMat
        else :
            posMat = np.dot( posMat, self.posMat )
            
        if color is None :
            color = self.color
        
        for obj in self.objeList :
            #obj.depthTestFlag = self.depthTestFlag
            obj.draw( gl, posMat, color )


class Lines( Obje ) :
    
    def __init__( self, pnts, nor = [ 0.0, 0.0, 1.0 ], typ = GL_LINES ) :
        #
        # GL_LINE_STRIP : zigzag Lines
        # GL_LINE_LOOP
        # GL_LINES      : separated Lines
        #
        super().__init__()
        points = np.array( pnts ).flatten()
        nPoints = int( len( points ) / 3 )
        normals = np.array( [ nor ] * nPoints )
        indices = np.arange( nPoints )
        self.setGeometry( typ, points, normals, indices )
        

class TrianglesA( Obje ) :

    def __init__( self, PNTS, IDX, img = None ) :
        super().__init__()
        print( 'TrianglesA __init__' )
        
        #
        #  ID  X  Y  Z   Nx Ny Nz  U V
        #
        #
        pntData = np.array( PNTS )
        IDs = pntData[:,0]
        print( 'TrianglesA', IDs )
        points = []
        normals = []
        uvs = []
        for idx in np.array( IDX ).reshape( [ -1, 3 ] ) :
            print( 'TrianglesA', idx )
            flag = False
            for i in idx :
                n = np.where( IDs == i )[0][0]
                points .append( pntData[n][1:4] )
                normals.append( pntData[n][4:7] )
                uvs    .append( pntData[n][7:9] )
                if not flag :
                    print( max( normals[-1] ) )
                    if max( normals[-1] ) > 1.0 :
                        flag = True
                        
            print( flag )
                        
            if flag :
                
                p0 = points[-1]
                p1 = points[-2]
                p2 = points[-3]
                v = np.cross( p1 - p0, p2 - p0 )
                print( v )
                
                for k in range( 3 ) :
                    if max( normals[-1-k] ) :
                        normals[-1-k] = v
                    
        #print( 'points' )
        #print( points )
                    
        #print( 'normals' )
        #print( normals )
                    
        #print( 'indices' )
        #print( indices )
                    
        #print( 'uvs' )
        #print( uvs )
                    
        indices = np.arange( len( points ) )        
        self.setGeometry( GL_TRIANGLES, points, normals, indices, uvs )

        if img is not None :
            imgData = Image.open( img )
            self.setTexture( imgData )
            
                                  
 
class Triangles( Obje ) :

    def __init__( self, pts, idxs = None, uv = None, img = None ) :
        super().__init__()
        #print( 'Triangles __init__' )
        #print( pts )
        points = []
        normals = []
        pnts = np.array( pts ).reshape( [ -1, 3 ] )
        uvs = None
        if uv is not None :
            uvx = np.array( uv ).reshape( [ -1, 2 ] )
            uvs = []

        if idxs is None :
            idxs = np.arange( len( pnts ) )
        for idx in np.array( idxs ).reshape( [ -1, 3 ] ) :
            #print( 'Triangles AAA', idx )
            p0 = pnts[ idx[0] ]
            p1 = pnts[ idx[1] ]
            p2 = pnts[ idx[2] ]
            v = np.cross( p1 - p0, p2 - p0 )
            points.extend( [ p0, p1, p2 ] )
            normals.extend( [ v, v, v ] )
            if uv is not None :
                uvs.extend( [ uvx[ idx[0] ], uvx[ idx[1] ], uvx[ idx[2] ] ] )
        indices = np.arange( len( points ) )        
        self.setGeometry( GL_TRIANGLES, points, normals, indices, uvs )

        if img is not None :
            #print( img )
            imgData = Image.open( img )
            self.setTexture( imgData )


class Rectangle( Obje ) :
    
    #def __init__( self, a, b, ax1 = 0 , ax2 = 1 , offset = [ 0.0, 0.0, 0.0 ] ) :
    def __init__( self, a, b, ax = 2 , offset = [ 0.0, 0.0, 0.0 ] ) :
        
        super().__init__()

        points = np.array( [ offset ]* 4  )
        points[ :, ( ax + 1 ) % 3 ] += [ -a/2.0, -a/2.0, +a/2.0, +a/2.0 ]
        points[ :, ( ax + 2 ) % 3 ] += [ -b/2.0, +b/2.0, -b/2.0, +b/2.0 ]
        
        n = [ ( 1.0 if i == ax else 0.0 ) for i in range( 3 ) ]
        #n = np.cross( [ ( 1.0 if i == ax1 else 0.0 ) for i in range( 3 ) ] ,
        #              [ ( 1.0 if i == ax2 else 0.0 ) for i in range( 3 ) ] )
        normals = np.array( [ n ] * 4  )
        indices = np.arange( 4 )
        
        self.setGeometry( GL_TRIANGLE_STRIP, points, normals, indices )


class ManyRectangles( Obje ) :
    
    def __init__( self ) :
        super().__init__()
        self.points, self.normals = [], []

    def addRect( self, a, b, ax1 = 0 , ax2 = 1 , offset = [ 0.0, 0.0, 0.0 ]  ) :
        pnt = np.array( [ offset ]* 4  )
        pnt[ :, ax1 ] += [ -a/2.0, -a/2.0, +a/2.0, +a/2.0 ]
        pnt[ :, ax2 ] += [ -b/2.0, +b/2.0, -b/2.0, +b/2.0 ]
        n = np.cross( [ ( 1.0 if i == ax1 else 0.0 ) for i in range( 3 ) ] ,
                      [ ( 1.0 if i == ax2 else 0.0 ) for i in range( 3 ) ] )
        nor = np.array( [ n ] * 4  )
        self.points.extend( pnt )
        self.normals.extend( nor )
    
    def mkObject( self ) :
        indices = [ [ k, k+1, k+3, k, k+3, k+2 ] for k in range( 0, len( self.points ), 4 ) ]
        self.setGeometry( GL_TRIANGLES, self.points, self.normals, indices )
        return self
        
        
class ManyRectanglesWithLine( ObjeCollection ) :
    
    def __init__( self ) :
        
        super().__init__()
        self.points, self.normals = [], []
        
    def addRect( self, a, b, ax1 = 0 , ax2 = 1 , offset = [ 0.0, 0.0, 0.0 ]  ) :
        pnt = np.array( [ offset ]* 4  )
        pnt[ :, ax1 ] += [ -a/2.0, -a/2.0, +a/2.0, +a/2.0 ]
        pnt[ :, ax2 ] += [ -b/2.0, +b/2.0, -b/2.0, +b/2.0 ]
        n = np.cross( [ ( 1.0 if i == ax1 else 0.0 ) for i in range( 3 ) ] ,
                      [ ( 1.0 if i == ax2 else 0.0 ) for i in range( 3 ) ] )
        nor = np.array( [ n ] * 4  )
        self.points.extend( pnt )
        self.normals.extend( nor )
        
    def mkObject( self, depthTestFlag = True ) :
        #print( 'mkObject' )
        rects = Obje()
        indices = [ [ k, k+1, k+3, k, k+3, k+2 ] for k in range( 0, len( self.points ), 4 ) ]        
        rects.setGeometry( GL_TRIANGLES, self.points, self.normals, indices )
        rects.depthTestFlag = depthTestFlag
        self.add( rects )
        
        pnt, nor = [], []
        for k in range( 0, len( self.points ), 4 ) :
            pnt.append( self.points[k] )
            pnt.append( self.points[k+3] )
            nor.append( self.normals[k] )
            nor.append( self.normals[k+3] )
        idx = range( len( pnt ) )
        
        lines = Obje()
        lines.setGeometry( GL_LINES, pnt, nor, idx )
        rects.depthTestFlag = depthTestFlag
        self.add( lines )
        
        return self



class Circle( Obje ) :
    
    def __init__( self, R, n = 20, ax = 2, offset = [ 0.0, 0.0, 0.0 ] ) :
        
        super().__init__()
        
        ang = np.linspace( 0.0, np.pi * 2.0, n )
        xx, yy = R * np.cos( ang ), R * np.sin( ang )
        pp = np.array( offset )
        nn = np.array( [ 1.0 if i==ax else 0.0 for i in range( 3 )  ] )
        pnt, nor = [ *pp ], [ *nn ]
        idx = np.arange( n + 1 )
        for x, y in zip( xx, yy ) :
            pp = np.array( offset )
            pp[ ( ax + 1 ) % 3 ] += x
            pp[ ( ax + 2 ) % 3 ] += y
            pnt.extend( pp )
            nor.extend( nn )
            
        self.setGeometry( GL_TRIANGLE_FAN, pnt, nor, idx )
      


class Sphere( Obje ) :
    
    def __init__( self, R, m = 15, n = 12, img = None ) :
        #print( '__init__ Sphere' )
        super().__init__()
        the = np.linspace(   0.0        , np.pi * 2.0, m )
        phi = np.linspace( - np.pi / 2.0, np.pi / 2.0, n )
        ct, st = np.cos( the ), np.sin( the )
        cp, sp = np.cos( phi ), np.sin( phi )
        pnt, nor, idx = [], [], []
        for i in range( m ) :
            for j in range( n ) :
                x, y, z = cp[j] * ct[i], cp[j] * st[i], - sp[j]
                pnt.extend( [ R * x, R * y, R * z ] )
                nor.extend( [ x, y, z ] )
        for i in range( m - 1 ) :
            for j in range( n - 1 ) :
                k = n * i + j
                idx.extend( [ k, k + n    , k + n + 1 ] )
                idx.extend( [ k, k + n + 1, k + 1     ] )
        self.setGeometry( GL_TRIANGLES, pnt, nor, idx )
        


class SkyObject( Obje ) :
    
    def __init__( self, R, m = 15, n = 6, img = None ) :
        #print( 'SkyObject' )
        super().__init__()
        
        self.depthTestFlag  = False
        self.resetDepthFlag = True
        self.shadowFactor   = 1.0
        self.color          = [ 1.0, 1.0, 1.0, 1.0 ]
        
        the = np.linspace( 0.0, np.pi * 2.0, m )
        phi = np.linspace( 0.0, np.pi / 2.0, n )
        ct, st = np.cos( the ), np.sin( the )
        cp, sp = np.cos( phi ), np.sin( phi )
        pnt, nor, idx, uvs = [], [], [], []
        for i in range( m ) :
            for j in range( n ) :
                x, y, z = cp[j] * ct[i], cp[j] * st[i], - sp[j]
                r = 1.0 - 2.0 / np.pi * phi[j]
                u = 0.49 * ( 1.0 + r * ct[i] )
                v = 0.49 * ( 1.0 + r * st[i] )
                pnt.extend( [ R * x, R * y, R * z ] )
                nor.extend( [ x, y, z ] )
                uvs.extend( [ u, v ] )
        for i in range( m - 1 ) :
            for j in range( n - 1 ) :
                k = n * i + j
                idx.extend( [ k, k + n    , k + n + 1 ] )
                idx.extend( [ k, k + n + 1, k + 1     ] )
        self.setGeometry( GL_TRIANGLES, pnt, nor, idx, uvs )
        
        if img is not None :
            #print( img )
            imgData = Image.open( img )
            self.setTexture( imgData )


    def draw( self, gl, posMat = np.identity(4), color = None ) :
        #print( 'SkyObject' )

        lookAtMat = gl.lookAtMat
        e = - np.dot( lookAtMat[ 0:3, 0:3 ].T, lookAtMat[ 0:3, 3 ] )
        self.posMat[ 0:3, 3 ] = e

        glUniform1i( gl.skyFlagLoc1, 1 )
        super().draw( gl )
        glUniform1i( gl.skyFlagLoc1, 0 )


class SimpleAirplane( Obje ) :
    
    def __init__( self, L = 5.0, W = 5.0, H = 3.0  ) :     

        super().__init__()

        #print( 'SimpleAirplane' )

        points  = [  L,  0.0, 0.0,
                    -L,   W , 0.0,
                    -L,  -W , 0.0,
                    0.0, 0.0, 0.0,
                    -L , 0.0, 0.0,
                    -L , 0.0, -H   ]
        
        normals = [ 0.0, 0.0, 1.0,
                    0.0, 0.0, 1.0,
                    0.0, 0.0, 1.0,
                    0.0, 1.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 1.0, 0.0  ]
        
        indices = np.arange( 6 )
        
        self.setGeometry( GL_TRIANGLES, points, normals, indices )



class Cone( Obje ) :
    
    def __init__( self, R, H, n = 20 ) :
        
        super().__init__()

        L = math.sqrt( R * R + H * H )
        r, h = R / L, H / L
        ang = np.linspace(0, 2 * math.pi, n )
        cc, ss = np.cos( ang ), np.sin( ang )
        points  = [ 0.0, 0.0, H ]
        normals = [ 0.0, 0.0, 0.01 ]
        indices = np.arange( n + 1 )
        for c, s in zip( cc, ss ) :
            points.extend ( [ R * c, R * s, 0.0 ] )
            normals.extend( [ h * c, h * s, r   ] )
        
        self.setGeometry( GL_TRIANGLE_FAN, points, normals, indices )
        
        
class TruncatedCone( Obje ) :
    
    def __init__( self, R1, R2, H, n = 20 ) :
        
        super().__init__()

        dR = R2 - R1
        L = math.sqrt( dR*dR + H*H )
        r, h = dR/L, H/L
        ang = np.linspace(0, 2 * math.pi, n )
        cc, ss = np.cos( ang ), np.sin( ang )
        points  = []
        normals = []
        indices = np.arange( 2*n )
        for c, s in zip( cc, ss ) :
            points.extend ( [ R1*c, R1*s, H ,
                              R2*c, R2*s, 0.0 ] )
            normals.extend( [ h *c, h *s, r  ,
                              h *c, h *s, r   ] )
        self.setGeometry( GL_TRIANGLE_STRIP, points, normals, indices )

  

class Cylinder( Obje ) :
    
    def __init__( self, R, H, n = 20 ) :
        
        super().__init__()

        ang = np.linspace(0, 2 * math.pi, n )
        cc, ss = np.cos( ang ), np.sin( ang )
        points  = [ [ R*c, R*s, 0.0,  R*c, R*s, H   ] for c, s in zip( cc, ss ) ]
        normals = [ [ c  , s  , 0.0,  c  , s  , 0.0 ] for c, s in zip( cc, ss ) ]
        indices = np.arange( n * 2 )
        
        self.setGeometry( GL_TRIANGLE_STRIP, points, normals, indices )
        
        
       
class Arrow( ObjeCollection ) :
    
    def __init__(self) :
        
        super().__init__()
       
        axis = self.add( Cylinder( 0.01, 0.9 ) )
        
        cone = self.add( Cone( 0.1, 0.3 ) )
        cone.posMat[2,3] = 0.7
        
        plate = self.add( Circle( 0.1 ) )
        plate.posMat[2,3] = 0.7
        plate.color = [ 0.5, 0.5, 0.5, 1.0]


class RectTest( Obje ) :
    
    def __init__( self ) :
        print( '__init__ RectTest' )
        super().__init__()
        
        points = []
        normals = []
        
        n = 3
        for i in range( n ) :
            for j in range( n ) :
                points  += [ 100.0*i, 100.0*j, 0.0 ]
                normals += [ 0.0  , 0.0  , 1.0 ]

        indices = []
        for i in range( n - 1 ) :
            for j in range( n ) :
                print( i,j,  [ n*i + j, n*(i+1) + j ] )
                indices += [ n*i + j, n*(i+1) + j ]
#             if i != n-2 : indices += [ -1 ]
            indices += [ -1 ]
        print( indices )
        self.setGeometry( GL_TRIANGLE_STRIP, points, normals, indices )


#         self.setTexture( '_10_903_406.jpg' )
#         uvs = []
#         for i in range( n ) :
#             for j in range( n ) :
#                 #uvs += [ ( 256 - i ) / 256.0, j / 256.0 ]
#                 uvs += [ float( i ) / (n-1), float( j )  / (n-1) ]
#         self.setGeom( GL_TRIANGLE_STRIP, points, normals, indices, uvs )
# 

    
def LMN2LatLon( L, M, N ) :
    #
    #  Calclate lat. & lon.
    #  of the bottom-left corner of the panel
    #
    nDiv = 2 ** L
    e = np.exp( np.pi * ( 1.0 - 2.0 / nDiv * ( N + 1 ) ) )
    lat = 2.0 * np.arctan( ( e - 1.0 ) / ( e + 1.0 ) )
    #lon = M / nDiv * 2.0 * np.pi  - np.pi
    lon = np.pi * ( 2.0 * M / nDiv - 1.0 )
    return lat, lon
 

        
class TerrainA( ObjeCollection ) :
    
    Rearth = 6378136.6
    LAT0   = np.radians(  34.750504 )
    LON0   = np.radians( 137.703405 )
    
    def __init__( self ) :
        print( '__init__ TerrainA' )
        super().__init__()
        
        
        Rearth  = TerrainA.Rearth
        LAT0    = TerrainA.LAT0
        LON0    = TerrainA.LON0
        COSLAT0 = np.cos( LAT0 )
            
        #dataPath = os.path.dirname(__file__) + '/TerrainData3/_14/'
        #dataPath = os.path.dirname(__file__) + '/TerrainData3/_13A/'
        dataPath = os.path.dirname(__file__) + '/TerrainData3/test0_10_903_406/temp/'
        
        #listDir = os.listdir( os.path.dirname(__file__) + '/TerrainData3'  )
        listDir = os.listdir( dataPath  )
        print( listDir )
        LMNList = []
        for d in listDir :
            base, ext = os.path.splitext( d )
            if ext == '.jpg' :
                LMNtxt = base.split( '_' )
                LMNList.append( [ int( LMNtxt[1] ), int( LMNtxt[2] ), int( LMNtxt[3] ) ] )
        LMNList = sorted( LMNList )
        print( 'LMNList  : ', LMNList )



        firstFlag = True

        for lmn in LMNList :
            print( lmn )
            txt = ''.join( [ '_' + str( x ) for x in lmn ] )
            
            if firstFlag :
                firstFlag = False
                
                nDiv = 2 ** lmn[0]
                e = np.exp( 2.0 * np.pi * ( 0.5 - lmn[2] / nDiv ) )
                lat = 2.0 * np.arctan( ( e - 1.0 ) / ( e + 1.0 ) )
                lon = 2.0 * np.pi * lmn[1] / nDiv - np.pi
                W = 2.0 * np.pi * Rearth / ( nDiv ) * np.cos( lat ) #* 1.010
                
                M0 = lmn[1]
                N0 = lmn[2]
                
                #
                #  lat, lon : top-left corner
                #            
                
                D  = W / 256.0
                dX0 = Rearth * ( lat - LAT0 )
                dY0 = Rearth * ( lon - LON0 ) * COSLAT0
            
            X0 = dX0 - W * ( lmn[2] - N0 )
            Y0 = dY0 + W * ( lmn[1] - M0 )
             
            print( 'X0, Y0   : ', X0, Y0  )
            print( 'W, D   : ', W, D  )
            
            print( dataPath + txt + '_alt.png' )
            imgAlt = Image.open( dataPath + txt + '_alt.png' )
            imgNor = Image.open( dataPath + txt + '_nor.png' )

            datAlt = np.array( imgAlt, dtype='uint8' )
            datNor = np.array( imgNor, dtype='uint8' )
            
            print( datAlt.shape )
            print( datNor.shape )
            
            pnts, nors, indices, uvs  = [], [], [], []
            for i in range( 257 ) :   
                for j in range( 257 ) :
                    
                    X = X0 - i * D
                    Y = Y0 + j * D
                    
                    c = datAlt[i][j]                        
                    h = 65536 * int( c[0] ) + 256 * int( c[1] ) + int( c[2] )
                    if   h == 8388608 : h = 0           #  2**23 =  8388608
                    elif h >  8388608 : h -= 16777216   #  2**24 = 16777216
                    Z =  - 0.01 * h
                    
                    pnts += [ X, Y, Z ]            
                
                    for c in datNor[i][j] :
                        c = int( c )
                        nors += [ float( c if c < 128 else ( c - 256 ) ) / 127.0 ]
                        
                        
            for i in range( 256 ) :
                for j in range( 257 ) :
                    indices += [ 257 * i + j, 257 *( i + 1 ) + j ]
                    #uvs += [ j / 256, ( 256 - i ) / 256 ]
                indices += [ -1 ]            
        
        
            
            try :
                img = Image.open( dataPath + txt + '.jpg' ) 
            except :
                print('error !!')
                continue
            
            imgSize = 512
            for i in range( 257 ) :
                for j in range( 257 ) :
                    uvs += [ j / 256,  ( 256 - i ) / 256 ]
        
#             for i in range( 257 ) :
#                 for j in range( 257 ) :
#                     uvs += [ j / 256, ( 256 - i ) / 256 ]
        
            #obj = Obje( GL_TRIANGLE_STRIP, pnts, nors, indices )
            obj = Obje( GL_TRIANGLE_STRIP, pnts, nors, indices, uvs )
            obj.setTexture( img )
         
         
            self.add( obj )                        
            
#          
         
# 
#     def draw( self, gl, posMat = np.identity(4), color = None ) :
#         #print('draw Obje')
#         pass



class Terrain( ObjeCollection ) :
    
    Rearth = 6378136.6
    LAT0   = np.radians(  34.750504 )
    LON0   = np.radians( 137.703405 )
    
    def __init__( self ) :
        print( '__init__ Terrain' )
        super().__init__()
        
        TerrainObj.Rearth = Terrain.Rearth
        TerrainObj.LAT0   = Terrain.LAT0
        TerrainObj.LON0   = Terrain.LON0
        
        self.drawList = []
        
        #print( __file__ )
        #print( os.listdir( os.path.dirname(__file__) + '/TerrainData/' ) )
        #file =  os.path.dirname(__file__) + '/TerrainData/' + txt
        
        listDir = os.listdir( os.path.dirname(__file__) + '/TerrainData2/HAMAMATSU' )
        LMNList = []
        for d in listDir :
            base, ext = d.split( '.' )
            if ext == 'pkl' :
                #print( base.split( '_' ) )
                LMNtxt = base.split( '_' )
                LMNList.append( [ int( LMNtxt[1] ), int( LMNtxt[2] ), int( LMNtxt[3] ) ] )
                
        for x in LMNList :
            print( x )
        LMNList = sorted( LMNList )
        print( '' )
        for x in LMNList :
            print( x )
        
        for x in LMNList :
            self.add( TerrainObj( x ) )
        
        
        
#         self.add( TerrainObj(     ( 10, 903, 406 ) ) )
#         self.add( TerrainObj( ( 15, 28916, 13006 ) ) )
#         self.add( TerrainObj( ( 15, 28916, 13007 ) ) )
#         self.add( TerrainObj( ( 15, 28917, 13006 ) ) )
#         self.add( TerrainObj( ( 15, 28917, 13007 ) ) )
#         self.add( TerrainObj( ( 15, 28918, 13006 ) ) )
#         self.add( TerrainObj( ( 15, 28918, 13007 ) ) )
#         self.add( TerrainObj( ( 15, 28919, 13006 ) ) )
#         self.add( TerrainObj( ( 15, 28919, 13007 ) ) )
        
        dummyRect = Rectangle( 100000.0, 100000.0 )
        dummyRect.color = [ 0.5, 0.5, 0.5, 0.0 ]
        self.add( dummyRect )
        
       
#         
#     def draw( self, gl, posMat = None, color = None ) :
#               
#         if posMat is None :
#             posMat = self.posMat
#         else :
#             posMat = np.dot( posMat, self.posMat )
#             
#         if color is None :
#             color = self.color
#         
#         for obj in self.objeList :
#             #obj.depthTestFlag = self.depthTestFlag
#             obj.draw( gl, posMat, color )

        
    def draw( self, gl, posMat = np.identity(4), color = None ) :
        print('draw Obje')
        super().draw( gl )
        
        Rearth  = self.Rearth
        LAT0    = self.LAT0
        LON0    = self.LON0
        COSLAT0 = np.cos( LAT0 )
        
        
        lookAtMat = gl.lookAtMat
        dcm  = lookAtMat[ 0:3, 0:3 ]
        dcmt = dcm.T
        e = - np.dot( dcmt, lookAtMat[ 0:3, 3 ] )
        
        tanY = np.tan( np.radians( gl.projFovy / 2.0 ) )
        tanX = tanY * gl.width / gl.height
    
        FF   = 256.0 * 2.0 * tanY / gl.height
        
        V0 = np.array( [ [  tanX,  tanY, -1.0 ],
                         [  tanX, -tanY, -1.0 ],
                         [ -tanX, -tanY, -1.0 ],
                         [ -tanX,  tanY, -1.0 ] ] )
        
        V = [ np.dot( dcmt, V0[k] )         for k in range( 4 )  ]
        t = [ ( 0.0 - e[2] ) / V[k][2]      for k in range( 4 )  ]
        d = [ np.linalg.norm( t[k] * V[k] ) if t[k] > 0 else np.float64( 100000.0 ) for k in range( 4 )  ]
        P = [ e + t[k] * V[k]               for k in range( 4 )  ]

        tmin = 1000000.0 
        kA = -1
        for k in range( 4 ) :
            if t[k] > 0.0 and t[k] < tmin :
                tmin  = t[k] 
                kA = k
                
        #print( ' d :', d )     
        #print( ' FF :', FF )     
            
        if kA != -1 :
            kB = ( kA + 1 ) % 4
            kC = ( kA + 2 ) % 4
            kD = ( kA + 3 ) % 4
#             print( ' dA, PA :', d[ kA ], P[ kA ] )     
#             print( ' dB, PB :', d[ kB ], P[ kB ] )     
#             print( ' dC, PC :', d[ kC ], P[ kC ] )     
#             print( ' dD, PD :', d[ kD ], P[ kD ] )     
            
            u10 = P[ kB ] - P[ kA ]
            if t[kB] < 0.0 :
                u10 = - u10
            else :
                u11 = P[ kC ] - P[ kB ]
                if t[kC]< 0.0 :
                    u11 = - u11
                u11 = u11 / np.linalg.norm( u11 )    
            u10 = u10 / np.linalg.norm( u10 )    
            
            u20 = P[ kD ] - P[ kA ]
            if t[kD] < 0.0 :
                u20 = - u20
            else :
                u21 = P[ kC ] - P[ kD ]
                if t[kC]< 0.0 :
                    u21 = - u21
                u21 = u21 / np.linalg.norm( u21 )    
            u20 = u20 / np.linalg.norm( u20 )    
            
            WW = FF * d[ kA ]
            LL = int( np.log2( ( 2.0 * np.pi * Rearth * COSLAT0 ) / WW ) )
            
            #LL = int( np.log2( ( 2.0 * np.pi * Rearth * COSLAT0 ) / d[ kA ] ) )
            
            #print( LL, WW  )
            
            
            curList = [ P[ kA ] ]
            
            flagB = 0
            flagC = 0
            
            count1 = 0
            while True :
                #if count1 == 1 : break 
                #count1 += 1
            
                newList = []
                aa = 2**( LL - 1 ) / np.pi
                WW = Rearth * COSLAT0 / aa
                DD = WW / FF
                #print( LL, WW, DD )
                
                if DD > d[ kC ] : break

                count2 = 0
                while True :
                    if count2 == 1 : break 
                    count2 += 1
                
                    #if WW < d[kB] :
                    if DD < d[kB] :
                        k, u = kA, u10
                    else :
                        k, u = kB, u11 
                        if flagB == 0 :
                            flagB = 1
                            newList.append( P[ kB ] )
 
                    ePu = np.dot( e - P[ k ], u )
                    alp = np.sqrt( ePu * ePu + DD * DD - d[ k ] * d[ k ] ) - ePu
                    P1 = P[ k ] + alp * u
                    newList.append( P1 )
                    
                    if DD < d[kD] :
                        k, u = kA, u20
                    else :
                        k, u = kD, u21
                        if flagC == 0 :
                            flagC = 1
                            newList.append( P[ kD ] )
                    ePu = np.dot( e - P[ k ], u )
                    alp = np.sqrt( ePu * ePu + DD * DD - d[ k ] * d[ k ] ) - ePu
                    P2 = P[ k ] + alp * u
                        
                    newList.append( P2 )
                    
#                 print( 'LL, WW : ' , LL, WW )
#                 print( flagB, flagC )
#                 print( 'curList : ' , curList )
#                 print( 'newList : ' , newList )
                
                curList = [ P1, P2 ]
                #print( LL, WW )
                #
                for k, p in enumerate( curList + newList ) :
                    lat = LAT0 + p[0] / Rearth
                    lon = LON0 + p[1] / Rearth / COSLAT0                 
                    tt = np.tan( lat / 2.0 )
                    ff = np.log( ( 1.0 + tt ) / ( 1.0 - tt ) )
                    MM = int( aa * ( lon + np.pi ) )
                    NN = int( aa * ( np.pi - ff ) )
                    if k == 0 :
                        M0, M1 = MM, MM
                        N0, N1 = NN, NN
                    else :
                        M0, M1 = min( MM, M0 ), max( MM, M1 )
                        N0, N1 = min( NN, N0 ), max( NN, N1 )
                    
                print( 'LMN : ', LL, M0, M1, N0, N1 )                    
                    
                #
                #
                
                LL -= 1
              
                  
              
#             print( 'WW : ' , WW )
#             print( ' P1 :', P1 )
#             print( ' P2 :', P2 )
            
#         print( t )
#         print( d )
#         print( P )
# 

                    
        
        
class TerrainObj( Obje ) :
    
    Rearth = 6378136.6
    LAT0   = np.radians(  34.750504 )
    LON0   = np.radians( 137.703405 )
        
    #def __init__( self, LMN, offset = ( 0.0, 0.0 ) ) :
    def __init__( self, LMN ) :
        print( '__init__ TerrainObj' )
        super().__init__()
        
        #self.depthTestFlag = False
        self.resetDepthFlag = True

        
        Rearth  = TerrainObj.Rearth
        LAT0    = TerrainObj.LAT0
        LON0    = TerrainObj.LON0
        COSLAT0 = np.cos( LAT0 )
        
        self.name = 'Terrain'
        self.LMN = LMN

#         LMN = ( 10, 903, 406 )
#         LMN = ( 15, 28916, 13006 )

        lat, lon = LMN2LatLon( *LMN )
        offset = (  Rearth * ( lat - LAT0 ),
                    Rearth * ( lon - LON0 ) * COSLAT0 )
        
        
        #dX = self.Rearth * ( lat - self.LAT0 )
        #dY = self.Rearth * ( lon - self.LON0 ) * self.COSLAT0

        txt = ''.join( [ '_' + str( LMN[k] ) for  k in range(3) ] )
        
        file =  os.path.dirname(__file__) + '/TerrainData2/HAMAMATSU/' + txt
        with open( file + '.pkl', 'rb') as f :
            data = pickle.load( f )
            points  = data[ 'points'  ]
            normals = data[ 'normals' ]
            
        points = points.reshape( [ -1, 3 ] )
        points[ :, 0:2 ] += offset

        n = 257
        indices = []
        uvs     = []
        for i in range( n - 1 ) :
            for j in range( n ) :
                indices += [ n*i + j, n*(i+1) + j ]
                #uvs += [ j / ( n-1 ), ( n-1 - i ) / ( n-1 ) ]
            indices += [ -1 ]
            
        for i in range( 257 ) :
            for j in range( 257 ) :
                uvs += [ j / ( n-1 ), ( n-1 - i ) / ( n-1 ) ]
            
            
        imgData = Image.open( file + '.jpg' )
        self.setTexture( imgData )
        self.setGeometry( GL_TRIANGLE_STRIP, points, normals, indices, uvs )        
        #self.setGeometry( GL_TRIANGLE_STRIP, points, normals, indices )
                   
                
        
class FighterF15( ObjeCollection ) :
    
    def __init__( self ) :
        
        super().__init__()
        
        X0 = 10.0
        X1 =  5.0
        X2 =  0.0
        R1 =  1.0
        R2 =  1.5
        
        mat1 = np.array( [
            [ 0.0,  0.0, 1.0, 0.0 ],
            [ 0.0, -1.0, 0.0, 0.0 ],
            [ 1.0,  0.0, 0.0, 0.0 ],
            [ 0.0,  0.0, 0.0, 1.0 ] ] )
        
        
        cone1 = self.add( Cone( R1, X0 - X1 ) )
        cone1.posMat = mat1.copy()
        cone1.posMat[ 0:3, 3 ] = np.array( [ X1, 0.0, 0.0 ])
        
        cone2 = self.add( TruncatedCone( R1, R2, X1 - X2 ) )
 
        cone2.posMat = mat1.copy()
        cone2.posMat[ 0:3, 3 ] = np.array( [ X2, 0.0, 0.0 ])
 
        
        
#         cone1.posMat = np.array( [
#             [ 0.0,  0.0, 1.0, 20.0 ],
#             [ 0.0, -1.0, 0.0, 0.0 ],
#             [ 1.0,  0.0, 0.0, 0.0 ],
#             [ 0.0,  0.0, 0.0, 1.0 ] ] )
#  
 


class GLWidget( QOpenGLWidget ) :
    
    comObjes = {}
    comBuffs = {}
    #flagSetupFinished = False
    #firstSetupFinished = False
    
    _COUNT1 = 0
    _COUNT2 = 0
    allDoneSignal = pyqtSignal()
    
    def __init__(self, parent, setupFunc = None ) :
        
        super().__init__( parent )
                
        GLWidget._COUNT1 += 1
        print( '__init__ GLWidget', GLWidget._COUNT1, GLWidget._COUNT2 )
                
        self.parent = parent
        self.setupFunc = setupFunc
        self.objes = []
        self.namesOfDrawingComObje = []
        self.currentProgram = -1
    
        self.parentDevicePixelRatio = parent.devicePixelRatio()
    
        self.lookAtMat = calcLookAtMat( [  10.0,  10.0,  10.0 ],
                                        [   0.0,   0.0,   0.0 ],
                                        [   0.0,   0.0,   1.0 ] )

        self.projFovy    =       1.0
        self.projNear    =       1.0
        self.projFar     = 1000000.0

        self.lightDirect = [ 0.2, 0.0, -1.0 ]
        self.lightAmbi   = [ 0.5, 0.5, 0.5, 1.0 ]
        self.lightDiff   = [ 1.0, 1.0, 1.0, 1.0 ]
        self.lightSpec   = [ 0.2, 0.2, 0.2, 1.0 ]

        self.txLightMat = np.array( [ [ 0.5, 0.0, 0.0, 0.5 ],
                                      [ 0.0, 0.5, 0.0, 0.5 ],
                                      [ 0.0, 0.0, 0.5, 0.5 ],
                                      [ 0.0, 0.0, 0.0, 1.0 ] ] )
        self.texWidth  = 4096
        self.texHeight = 4096
        self.selPoint = None


    def initializeGL(self) :
        print('initializeGL')
                
        glClearColor( 0.0, 0.0, 0.5, 1.0 )
        #glClearColor( 1.0, 1.0, 1.0, 1.0 )
        #glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE )
        #glEnable( GL_DEPTH_TEST )
        glEnable( GL_ALPHA_TEST )

        glEnable ( GL_PRIMITIVE_RESTART )
        glPrimitiveRestartIndex (-1)


        self.tex = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.tex)

        glTexImage2D( GL_TEXTURE_2D,
                      0,
                      GL_DEPTH_COMPONENT24,
                      self.texWidth,
                      self.texHeight,
                      0,
                      GL_DEPTH_COMPONENT,
                      GL_FLOAT,
                      None )
        
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        #glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        #glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL) 
        glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE)
              
        self.fb = glGenFramebuffers(1)
        glBindFramebuffer(GL_FRAMEBUFFER, self.fb)
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
        GL_TEXTURE_2D, self.tex, 0)




        glDrawBuffer(GL_NONE)
        glReadBuffer(GL_NONE)

        glBindFramebuffer(GL_FRAMEBUFFER, 0)
  
        self.prog0 = self.createProgram(
            '''
            layout(location = 0) in vec3 position;
            uniform mat4 posiMat;
            uniform mat4 LtProjMat;
            void main(void){
              gl_Position = LtProjMat * posiMat * vec4( position, 1.0 );
              gl_Position.z = gl_Position.z + 0.0005;
            }        
            ''',
            '''
            void main (void){
              gl_FragColor = vec4( 1.0, 0.0, 0.0, 1.0 );
            }
            ''')
        self.posiMatLoc0   = glGetUniformLocation( self.prog0, 'posiMat'   )
        self.LtProjMatLoc0 = glGetUniformLocation( self.prog0, 'LtProjMat' )


        self.prog1 = self.createProgram(
            '''
            layout(location = 0) in vec3 position;
            layout(location = 1) in vec3 normal;
            layout(location = 2) in vec2 uv;
            uniform mat4 posiMat;
            uniform mat4 lookAtMat;
            uniform mat4 projMat;
            uniform mat4 tLtProjMat;
            varying vec4 texCoord;
            varying vec4 P;
            varying vec3 N;
            varying vec2 UV;
            void main(void){
              vec4 pos = posiMat * vec4( position, 1.0 );
              P = lookAtMat * pos ;              
              N = normalize( vec3( lookAtMat * posiMat * vec4( normal, 0.0 ) ) ) ;
              texCoord = tLtProjMat * pos ;
              gl_Position = projMat * P;
              UV.x = uv.x;
              UV.y = 1.0 - uv.y;
            }
            '''            
            ,
            '''
            uniform mat4 lookAtMat;
            //uniform vec4 lightDir;
            uniform mat4 tLtProjMat;
                        
            uniform vec4 mtlAmbi;
            uniform vec4 mtlDiff;
            uniform vec4 mtlSpec;
            uniform float mtlShin;
            uniform float shadowFactor;
            
            uniform vec4 lightAmbi;
            uniform vec4 lightDiff;
            uniform vec4 lightSpec;
            uniform int skyFlag;
            
            uniform sampler2DShadow texture;
            uniform sampler2D tex;
            uniform int texFlag;
            
            varying vec4 texCoord;
            varying vec4 P;
            varying vec3 N;
            varying vec2 UV;
            void main(void){
              vec3 L = normalize( vec3( lookAtMat * tLtProjMat[2] ) ) ;
              //vec3 L = normalize( vec3( lookAtMat * lightDir ) ) ;
              vec3 H = normalize( L - normalize( P.xyz ) ) ;
              float diffuse = abs( dot(L, N) );
              float specular = pow( abs( dot( N, H ) ), mtlShin ) ;
              vec4 fragColor;
              vec4 shadow;
              if( texFlag == 1 ){
                vec4 color = texture2D( tex, UV );
                fragColor = color * ( lightAmbi + lightDiff * diffuse + lightSpec * specular );
                if( skyFlag == 1 ) fragColor = color;
                //fragColor = color;
                //shadow = color * ( lightAmbi + lightDiff * diffuse * shadowFactor );
                shadow = color * shadowFactor;
              }else{
                fragColor =  mtlAmbi * lightAmbi
                          +  mtlDiff * lightDiff * diffuse
                          +  mtlSpec * lightSpec * specular;
                shadow =  mtlAmbi * lightAmbi + mtlDiff * lightDiff * diffuse * shadowFactor;
              }     
              gl_FragColor = shadow
                           + ( fragColor - shadow ) * shadow2DProj( texture, texCoord );
            
              //gl_FragColor =  fragColor;
              
            }
            ''' )
  
        self.posiMatLoc1      = glGetUniformLocation( self.prog1, 'posiMat'    )
        self.lookAtMatLoc1    = glGetUniformLocation( self.prog1, 'lookAtMat'  )
        self.projMatLoc1      = glGetUniformLocation( self.prog1, 'projMat'    )
        self.lightDirLoc1     = glGetUniformLocation( self.prog1, 'lightDir'   )
        self.tLtProjMatLoc1   = glGetUniformLocation( self.prog1, 'tLtProjMat' )
        
        self.mtlAmbiLoc1      = glGetUniformLocation( self.prog1, 'mtlAmbi'    )
        self.mtlDiffLoc1      = glGetUniformLocation( self.prog1, 'mtlDiff'    )
        self.mtlSpecLoc1      = glGetUniformLocation( self.prog1, 'mtlSpec'    )
        self.mtlShinLoc1      = glGetUniformLocation( self.prog1, 'mtlShin'    )
          
        self.lightAmbiLoc1    = glGetUniformLocation( self.prog1, 'lightAmbi'  )
        self.lightDiffLoc1    = glGetUniformLocation( self.prog1, 'lightDiff'  )
        self.lightSpecLoc1    = glGetUniformLocation( self.prog1, 'lightSpec'  )
        self.skyFlagLoc1      = glGetUniformLocation( self.prog1, 'skyFlag'    )

        self.textureLoc1      = glGetUniformLocation( self.prog1, 'texture'    )
        self.texLoc1          = glGetUniformLocation( self.prog1, 'tex'        )
        self.texFlagLoc1      = glGetUniformLocation( self.prog1, 'texFlag'    )
        self.shadowFactorLoc1 = glGetUniformLocation( self.prog1, 'shadowFactor' )

        self.createComObje()
                
        if self.setupFunc is not None :
            self.setupFunc()

        GLWidget._COUNT2 += 1

#         if GLWidget._COUNT1 == GLWidget._COUNT2 :
#             GLWidget.allDoneSignal.emit()

        print( 'initializeGL ZZZ', GLWidget._COUNT1, GLWidget._COUNT2 )


    def createComObje( self ) :
        pass


    def resizeGL( self, w, h ) :
        #print('resizeGL GLWidget')
        self.width  = w
        self.height = h
        dpr = self.parentDevicePixelRatio
        self.viewport =[ 0, 0, int( self.width * dpr ) + 1, int( self.height * dpr ) + 1]  
        self.update()
        

    def paintGL( self ) :
        #print('paintGL GLWidget')
        #print( len( self.objes ) )
       
        drawingObjs = []
        for obj in self.objes :
            drawingObjs.append( obj )
        for name in self.namesOfDrawingComObje :
            if name in GLWidget.comObjes :
                drawingObjs.append( GLWidget.comObjes[ name ] )
            
        shadowMinMax = np.array( [ [-1.0, 1.0] ] * 3 )
        firstFlag = True
        #for obj in self.objes :
        for obj in drawingObjs :
            if obj.shadowFlag :
                if firstFlag :
                    shadowMinMax = obj.getMinMax()
                    firstFlag = False
                else :
                    shadowMinMax = addMinMax( shadowMinMax, obj.getMinMax() )
                
        ltProjMat  = calcLightLookAt( self.lightDirect, shadowMinMax )          
        tLtProjMat = np.dot( self.txLightMat, ltProjMat)

        #        
        #-----------------------------------------------        
        #        1st step  :  create shadow map    
        #-----------------------------------------------        
        #
        glUseProgram(self.prog0)
        self.currentProgram = 0
        
        glBindFramebuffer(GL_FRAMEBUFFER, self.fb)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glViewport( 0, 0, self.texWidth, self.texHeight )  
        glUniformMatrix4fv( self.LtProjMatLoc0, 1, GL_TRUE, ltProjMat )
        #for obj in self.objes :
        for obj in drawingObjs :
            if obj.shadowFlag :
                obj.draw( self )
        glBindFramebuffer( GL_FRAMEBUFFER, 0 )
                
        #        
        #-----------------------------------------------        
        #        2nd step      
        #-----------------------------------------------        
        #
        self.makeCurrent()
        
        glUseProgram( self.prog1 )
        self.currentProgram = 1
        
        glActiveTexture( GL_TEXTURE0 )
        glBindTexture(GL_TEXTURE_2D, self.tex)
         
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT )
        glViewport( *self.viewport )

        glEnable( GL_BLEND )
        glBlendFuncSeparate( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_SRC_ALPHA,GL_ONE )
        #glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA)

        self.projMat = perspectiveMatrix( self.projFovy,
                                          self.width / self.height,
                                          self.projNear,
                                          self.projFar )
                  
        glUniformMatrix4fv( self.lookAtMatLoc1 , 1, GL_TRUE, self.lookAtMat    )     
        glUniformMatrix4fv( self.projMatLoc1   , 1, GL_TRUE, self.projMat      )         
        glUniform4fv      ( self.lightDirLoc1  , 1, [ *self.lightDirect, 0.0 ] )
        glUniformMatrix4fv( self.tLtProjMatLoc1, 1, GL_TRUE, tLtProjMat        )
        glUniform4fv      ( self.lightAmbiLoc1 , 1, self.lightAmbi             )
        glUniform4fv      ( self.lightDiffLoc1 , 1, self.lightDiff             )
        glUniform4fv      ( self.lightSpecLoc1 , 1, self.lightSpec             )
        glUniform1i       ( self.textureLoc1   , 0                             )
       
        #for obj in self.objes :
        for obj in drawingObjs :
            #print( obj.name )
            obj.draw( self ) 

        glUseProgram(0)
        self.currentProgram = -1

        #print('paintGL GLWidget ZZZ')


    def createProgram( self, vss, fss ) :

        vertex_shader = glCreateShader(GL_VERTEX_SHADER)
        glShaderSource(vertex_shader, vss)
        glCompileShader(vertex_shader)

        fragment_shader = glCreateShader(GL_FRAGMENT_SHADER)
        glShaderSource(fragment_shader, fss)
        glCompileShader(fragment_shader)

        program = glCreateProgram()
        glAttachShader(program, vertex_shader)
        glDeleteShader(vertex_shader)
        glAttachShader(program, fragment_shader)
        glDeleteShader(fragment_shader)

        glLinkProgram(program)

        return program


    def addObje( self, obj, shadow = True ) :
        self.objes.append( obj )
        obj.shadowFlag = shadow
        return obj


    def addDrawingComObje( self, name ) :
        #print( 'addDrawingComObje', name )
        if not name in self.namesOfDrawingComObje :
            self.namesOfDrawingComObje.append( name )
        #self.objes.append( GLWidget.comObjes[ name ] )

    def addComObje( self, name, obj, shadow = True ) :
        #print( 'addComObje' )
        GLWidget.comObjes[ name ] = obj
        obj.shadowFlag = shadow
        return obj


    def delComObje( self, name ) :
        #print( 'delComObje' )
        if name in GLWidget.comObjes :
            #print( 'delComObje' )
            GLWidget.comObjes[ name ].deleteBuffer()
            del( GLWidget.comObjes[ name ] )
            #print( GLWidget.comObjes )
            #GLWidget.comObjes.pop( name )

    def addComBuff( self, name, obj ) :
        GLWidget.comBuffs[ name ] = obj


    def getPixel( self ) :
        self.makeCurrent()
        X, Y, W, H = self.viewport
        buf = glReadPixelsf( X, Y, W, H, GL_BGR, GL_UNSIGNED_BYTE )
        return np.frombuffer( buf, dtype = 'uint8').reshape( [ H, W, 3 ] )


    def mousePressEvent( self, event ) :
        #print('mousePressEvent')
        self.mousePos = event.pos()
        if event.button() == Qt.MouseButton.LeftButton :
            self.mouseBtn = 1
        elif event.button() == Qt.MouseButton.RightButton :
            self.mouseBtn = 2
            
    def getPT( self, event ) :
        #print('getPT')
        self.makeCurrent()
        X0, Y0, W0, H0 = self.viewport
        x = event.pos().x() / self.width * W0
        y = ( self.height - event.pos().y() ) / self.height * H0
        z = glReadPixelsf( x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT )[ 0, 0 ]
        self.infFlag = True if z == 1.0 else False
        #print( z, self.infFlag  )
        p = np.array([ 2.0 * ( ( x - X0 ) / W0 ) - 1.0,
                       2.0 * ( ( y - Y0 ) / H0 ) - 1.0,
                       2.0 * z - 1.0,
                       1.0 ])
        q = np.dot( np.linalg.inv( self.projMat ), p )
        return np.array([ q[0]/q[3], q[1]/q[3], q[2]/q[3] ])
    
    def mouseDoubleClickEvent( self, event ) :
        #print('mouseDoubleClickEvent')
        self.selPoint = self.getPT( event )

    def mouseReleaseEvent( self, event ) :
        #print('mouseReleaseEvent')
        self.mouseBtn = 0
  
    def mouseMoveEvent( self, event ) :
        #print('mouseMoveEvent')
        P0 = glGetIntegerv(GL_VIEWPORT)
        z = self.getPT( event )[2]
        #print( z )
        if self.infFlag : return
        dx = event.pos().x() - self.mousePos.x()
        dy = event.pos().y() - self.mousePos.y()
        self.mousePos = event.pos()
        if self.mouseBtn == 1 :
            self.lookAtMat[0,3] -= 2 * dx * z / self.width  / self.projMat[0,0] 
            self.lookAtMat[1,3] += 2 * dy * z / self.height / self.projMat[1,1]
        elif self.mouseBtn == 2 :
            if self.selPoint is None : return
            L = 0.8 * min( self.width, self.height )
            ang1 , ang2 = dy / L , dx / L
            c1 , c2 = np.cos( ang1 ), np.cos( ang2 )
            s1 , s2 = np.sin( ang1 ), np.sin( ang2 )
            if abs( ( event.pos().x() - P0[0] )/ self.width - 0.5 ) < 0.4 :
                rotMat = np.array( [ [  c2, s2*s1, s2*c1 ],
                                     [ 0.0,   c1 ,  -s1  ],
                                     [ -s2, c2*s1, c2*c1 ] ])
                self.lookAtMat[0:3,0:3] = np.dot( rotMat, self.lookAtMat[0:3,0:3] )
                self.lookAtMat[0:3,3]   = np.dot( rotMat, self.lookAtMat[0:3,3] - self.selPoint ) \
                                        + self.selPoint
            else :
                if ( event.pos().x() - P0[0] ) / self.width > 0.5 : s1 = -s1
                rotMat = np.array( [ [  c1, -s1, 0.0 ],
                                     [  s1,  c1, 0.0 ],
                                     [ 0.0, 0.0, 1.0 ] ])
            self.lookAtMat[0:3,0:3] = np.dot( rotMat, self.lookAtMat[0:3,0:3] )
            self.lookAtMat[0:3,3]   = np.dot( rotMat, self.lookAtMat[0:3,3] - self.selPoint ) \
                                    + self.selPoint
        self.update()

    def wheelEvent( self, event ) :
        #print('wheelEvent')
        modifiers = QApplication.keyboardModifiers()
        if modifiers == Qt.KeyboardModifier.ShiftModifier :
            if event.angleDelta().y() < 0 :
                #self.viewFovy /= 1.2
                self.projFovy /= 1.2
            else :
                #self.viewFovy = self.viewFovy * 0.8 + 180.0 * 0.2
                self.projFovy = self.projFovy * 0.8 + 180.0 * 0.2
        else :
            if self.selPoint is None : return
            d = abs( self.selPoint[2] ) / 10.0
            if event.angleDelta().y() < 0 :
                self.lookAtMat[2,3] += d
            else :
                self.lookAtMat[2,3] -= d
        self.update()
