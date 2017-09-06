import numpy as np
from numpy import linalg as npla
from scipy.ndimage.filters import gaussian_filter
import math

class Matrix():

    @staticmethod
    def makeGaussian(size, fwhm = 5, center=None):
        """ Make a square gaussian kernel: size is the length of a side of the square
        fwhm is full-width-half-maximum, which can be thought of as an effective radius."""
        x = np.arange(0, size, 1, float)
        y = x[:,np.newaxis]
        if center is None: x0 = y0 = size // 2
        else:
            x0 = center[0]
            y0 = center[1]
        return np.exp(-4*np.log(2) * ((x-x0)**2 + (y-y0)**2) / fwhm**2)

    gaussian_kernel = makeGaussian.__func__(32)


    def __init__(self, xlim=[], ylim=[], width_height=[640,480],size=32,fwhm=12):
        self.w,self.h = width_height 
        self.xlim, self.ylim = xlim,ylim
        self.m = np.zeros((self.w,self.h))
        self.gaussian_kernel = self.makeGaussian(size,fwhm=fwhm)

    def getM(self):
        return np.transpose(self.m)

    def add(self, x, y, iMaxVal=0):
        xlim = self.xlim
        ylim = self.ylim
        i0 = (x - xlim[0]) / float(xlim[1]-xlim[0])
        j0 = (y - ylim[0]) / float(ylim[1]-ylim[0])
        n = len(self.gaussian_kernel[0])
        i = i0*self.w - n/2
        j = j0*self.h - n/2
        i = int(i)
        j = int(j)

        for row1, row2 in zip(self.m[i:], self.gaussian_kernel):
            row = row1[j:n + j]
            row1[j:n + j] = map(sum, zip(row2, row))
            if(iMaxVal>0):
                indices = row1 > iMaxVal
                row1[indices] = iMaxVal


    def sampleMat(self):
        maux = np.copy(self.m)
        maux = maux / np.max(maux)
        w,h = maux.shape
        th = 0.01
        niter, bFound = 0, False
        while not bFound:
            i,j = np.random.randint(w), np.random.randint(h)
            bFound = (maux[i,j] > th)
            niter += 1
            if(niter%100==0): th -= 0.01
        return (i/float(w),1-j/float(h))