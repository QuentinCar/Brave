#!/usr/bin/env python2
#available at https://www.ensta-bretagne.fr/jaulin/roblib.py 
# For help : https://www.ensta-bretagne.fr/jaulin/python.html  
# used in KalMOOC :  https://www.ensta-bretagne.fr/jaulin/kalmooc.html
# used in RobMOOC :  https://www.ensta-bretagne.fr/jaulin/robmooc.html
# used in KalMOOC :  https://www.ensta-bretagne.fr/jaulin/inmooc.html


import numpy as np
import matplotlib.pyplot as plt
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,\
                    sign,sum,meshgrid,cross,linspace,append,round, matmul
from matplotlib.pyplot import *
from numpy.random import randn,rand
from numpy.linalg import inv, det, norm, eig
from scipy.linalg import sqrtm,expm,norm,block_diag

from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from math import factorial, atan2
from matplotlib.patches import Ellipse,Rectangle,Circle, Wedge, Polygon, Arc
from matplotlib.collections import PatchCollection


def init_figure(xmin,xmax,ymin,ymax): 
    fig = plt.figure(0)
    ax = fig.add_subplot(111, aspect='equal')   
    ax.xmin=xmin
    ax.xmax=xmax
    ax.ymin=ymin
    ax.ymax=ymax
    clear(ax)
    return ax

def clear(ax):
    plt.pause(0.001)
    plt.cla()
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)

def draw_sailboat(x,delta_s=0,delta_r=0,psi=0,awind=0):
    x=x.flatten()
    theta=x[2]
    hull=array([[-0.25,1.25,1.75,1.75,1.25,-0.25,-0.25,-0.25],[-0.5,-0.5,-0.25,0.25,0.5,0.5,-0.5,-0.5],[1,1,1,1,1,1,1,1]])
    sail=np.array([[-1.75,0],[0,0],[1,1]])
    rudder=np.array([[-0.25,0.25],[0,0],[1,1]])
    R=np.array([[cos(theta),-sin(theta),x[0]],[sin(theta),cos(theta),x[1]],[0,0,1]])
    Rs=np.array([[cos(delta_s),-sin(delta_s),0.25*3],[sin(delta_s),cos(delta_s),0],[0,0,1]])
    Rr=np.array([[cos(delta_r),-sin(delta_r),0.25*(-1)],[sin(delta_r),cos(delta_r),0],[0,0,1]])
    R1 = np.matmul(R, hull)
    R2 = np.matmul(np.matmul(R, Rs), sail)
    R3 = np.matmul(np.matmul(R, Rr), rudder)
    plot2D(R1,'black')
    plot2D(R2,'red',2)
    plot2D(R3,'red',2)
    if psi!=0 or awind!=0:
        draw_arrow(x[0]+5,x[1],psi,5*awind,'blue')

def plot2D(M,col='black',w=1):
    plt.plot(M[0, :], M[1, :], col, linewidth = w)

def draw_arrow(x,y,theta,L,col):
    e=0.2
    M1=L*np.array([[0,1,1-e,1,1-e],[0,0,-e,0,e]])
    M=np.append(M1,[[1,1,1,1,1]],axis=0)
    R=np.array([[cos(theta),-sin(theta),x],[sin(theta),cos(theta),y],[0,0,1]])
    plot2D(np.matmul(R, M),col)
    
    
def draw_ellipse(c,Gamma,Eta,ax,col): # Gaussian confidence ellipse with artist
    #draw_ellipse(array([[1],[2]]),eye(2),0.9,ax,[1,0.8-0.3*i,0.8-0.3*i])
    if (norm(Gamma)==0):
        Gamma=Gamma+0.001*eye(len(Gamma[1,:]))
    A=sqrtm(-2*log(1-Eta)*Gamma)    
    w, v = eig(A)    
    v1=array([[v[0,0]],[v[1,0]]])
    v2=array([[v[0,1]],[v[1,1]]])        
    f1=matmul(A, v1)
    f2=matmul(A, v2)      
    Phi=(arctan2(v1 [1,0],v1[0,0]))
    Alpha=Phi*180/3.14
    e = Ellipse(xy=c, width=2*norm(f1), height=2*norm(f2), angle=Alpha)   
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(0.7)
    e.set_facecolor(col)
    
    
def draw_disk(c,r,ax,col): 
    #draw_disk(array([[1],[2]]),0.5,ax,"blue")
    e = Ellipse(xy=c, width=2*r, height=2*r, angle=0)   
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(0.7)
    e.set_facecolor(col)

def sawtooth(x):
    return (x+pi)%(2*pi)-pi   # or equivalently   2*arctan(tan(x/2))

def tondarray(M):
    if type(M)==float:
        return array([[M]])
    elif type(M)==int:
        return array([[M]])        
    else:
        return M  
    
def mvnrnd2(x,G): 
    n=len(x)
    x1=x.reshape(n)
    y = np.random.multivariate_normal(x1,G).reshape(n,1)
    return(y)    

def mvnrnd1(G):
    G=tondarray(G)
    n=len(G)
    x=array([[0]] * n)
    return(mvnrnd2(x,G))  
