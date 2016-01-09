import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.colors import colorConverter
from matplotlib.collections import PolyCollection
import matplotlib as mpl
import itertools
import math

font = {'family' : 'Bitstream Vera Sans',
#        'weight' : 'bold',
        'size'   : 20}

x_lim,y_lim = (-4, 4),(-1.5, 5.5)

mycolors,mylabels,myf = [],[],-1

def savefig(fig,ax,name='img.png'):
    plt.show()
    extent = ax.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
    fig.savefig(name, dpi=300, format='png',bbox_inches=extent)

def makeFigure(axes = []):
    global x_lim,y_lim
    if(len(axes)>0): 
        x_lim,y_lim=(axes[0],axes[1]),(axes[2],axes[3])

    plt.close()
    fig = plt.figure()
    ax = plt.axes(xlim=x_lim,ylim=y_lim)
    ax.set_aspect('equal')
    return fig,ax

def dist(p,q):
    return math.hypot(q[0] - p[0], q[1] - p[1])

def closestPointTo(p,listOfPoints):  # structure of listOfPoints [[x0,x1,...],[y0,y1,...]] matlab like
    if(len(listOfPoints) != 2): print "wrong list structure [[x0,x1,...],[y0,y1,...]]"
    if(len(listOfPoints[0]) != len(listOfPoints[1])): print "xs and ys list must be same size" 
    imin,mindist = -1,9999999
    for i in range(len(listOfPoints[0])):
        d = dist(p, [listOfPoints[0][i],listOfPoints[1][i]] )
        if(d < mindist): imin, mindist = i,d
    return imin, mindist

def vrand():
    v = np.random.rand(2)-np.array([0.5,0.5])
    return v / np.linalg.norm(v)

def vnorm(u):
    return np.linalg.norm(u)

def vangle(u,v):
    c = np.dot(u,v)/np.linalg.norm(u)/np.linalg.norm(v)     # -> cosine of the angle
    angle = np.arccos(np.clip(c, -1, 1))
    return angle

def vangleSign(u,v):
    return np.arctan2(v[1],v[0]) - np.arctan2(u[1],u[0])

def vrotate(v, angle, anchor=[0,0]):
    """Rotate a vector `v` by the given angle, relative to the anchor point."""
    x, y = v
    x = x - anchor[0]
    y = y - anchor[1]
    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)
    nx = x*cos_theta - y*sin_theta
    ny = x*sin_theta + y*cos_theta
    nx = nx + anchor[0]
    ny = ny + anchor[1]
    return [nx, ny]

def computePointsAngle(pos,angle):
    vdir = vrotate([.4,0],angle)
    #p1 = map(sum,zip(pos,vdir))
    p1 = [pos[0]+vdir[0],pos[1]+vdir[1]]
    p2 = [pos[0]-vdir[0],pos[1]-vdir[1]]
    vdir = vrotate([.4,0],angle+np.pi/2)
    p3 = [pos[0]+vdir[0],pos[1]+vdir[1]]
    return p1,p2,p3

def drawCircle(ax,position,r, alpha = 0.4, color = 'b', fill=True, linestyle='solid'):
    lw = 1
    if(linestyle=='dashed'): lw = 3
    c = plt.Circle(position, radius=r, alpha=alpha, fill=fill, color=color, edgecolor=color,ls=linestyle,linewidth=lw)
    ax.add_patch( c )
    return c

def drawBox(ax, vertices, alpha = 0.5, color = 'b', fill=True, linestyle='solid'):
    lw = 1
    if(linestyle=='dashed'): lw = 3
    poly = plt.Polygon(vertices, alpha=alpha, fill=fill, color=color, edgecolor=color, ls=linestyle,linewidth=lw)
    ax.add_patch( poly )

def decorate(xlabel = None, ylabel = None, title = None, xticks = None, mainfont=20, legendfont=12, bLegend = False):
    global font
    font['size']=mainfont
    plt.rc('font', **font)
    plt.rc('legend',**{'fontsize':legendfont})
    if(xlabel != None): plt.xlabel(xlabel)
    if(ylabel != None): plt.ylabel(ylabel)
    if(title != None): plt.title(title)
    if(bLegend): plt.legend()

def drawBarPlot(values, xlabel = None, ylabel = None, title = None, xticks = None, color = 'b'):
    plt.rc('font', **font)
    N = len(values)
    maxv = max(values)
    index = np.arange(N)  # the x locations for the groups
    barWidth = 0.35       # the width of the bars
    fig, ax = makeFigure(xlim=(-barWidth, N - barWidth/2), ylim=(0, maxv + maxv/5.0))
    ax.bar(index, values, barWidth, color=color, error_kw=dict(elinewidth=2,ecolor='red'))
    if(xticks != None): plt.xticks(index + barWidth/2.0, xticks)
    decorate(xlabel,ylabel,title,xticks)

def drawPlotY(y, xlim = [0,100], label = None, color = None, linewidth=2, colors = [], labels = [], line='-'):
    global mycolors,mylabels,myf
    if(color != None or label != None): myf = -1
    if(len(colors) > 0): mycolors, myf = colors, 0
    if(len(labels) > 0): mylabels, myf = labels, 0

    x = np.linspace(xlim[0], xlim[1], len(y))

    if(myf < 0):
        if(color != None): p, = plt.plot(x,y,color=color,label=label)
        else: p, = plt.plot(x,y)
    else:
        p, = plt.plot(x,y,line,color=mycolors[myf],label=mylabels[myf])
        myf += 1

    plt.setp(p, linewidth=linewidth)

    return p


def error_monitor(errors,mean_error,mean_var):
    mean_error.append( np.mean(errors) )
    mean_var.append( np.var(errors) )

def runningMeanFast(x, N):
    return np.convolve(x, np.ones((N,))/N)[(N-1):]

def drawPlotXY(x, y, yerror = None, xlabel = None, ylabel = None, title = None, xticks = None, color = None, myplt = None):
    if(myplt == None): myplt = plt     
    if(color != None): p, = myplt.plot(x,y,color=color)
    else: p, = myplt.plot(x,y)
    #if(yerror != None): plt.fill_between(x, y-yerror, y+yerror, alpha=0.5, edgecolor='#CC4F1B', facecolor='#FF9848')
    if(yerror != None): 
        yminus = np.array(y)-np.array(yerror)
        yplus = np.array(y)+np.array(yerror)
        myplt.fill_between(x, yminus, yplus, alpha=0.5, linewidth=0, color=color)
    decorate(xlabel,ylabel,title,xticks)
    return p


def drawMaps(ax,a=[],b=[],extent=None,alpha1=0.3,alpha2=0.3):
    cmap1 = mpl.colors.LinearSegmentedColormap.from_list('my_cmap',['white','green'],256)  # make the colormaps    
    cmap2 = mpl.colors.LinearSegmentedColormap.from_list('my_cmap2',['white','green'],256)

    cmap1._init(), cmap2._init()  # create the _lut array, with rgba values
     
    alphas1 = np.linspace(0, alpha1, cmap1.N+3)     # create your alpha array and fill the colormap with them.
    alphas2 = np.linspace(0, alpha2, cmap2.N+3)     # here it is progressive, but you can create whathever you want

    cmap1._lut[:,-1] = alphas1
    cmap2._lut[:,-1] = alphas2

    if(a != []):
        if(extent != None):  ax.imshow(a, extent=extent, interpolation='nearest', cmap=cmap1, origin='lower')
        else:  ax.imshow(a, interpolation='nearest', cmap=cmap1, origin='lower')

    if(b != []):
        if(extent != None):  ax.imshow(b, extent=extent, interpolation='nearest', cmap=cmap2, origin='lower')
        else: ax.imshow(b, interpolation='nearest', cmap=cmap2, origin='lower')

