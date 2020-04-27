# -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 01:58:26 2020

@author: David
"""
import cv2
import numpy as np
from scipy.sparse import csr_matrix as cm
from scipy.sparse.linalg import spsolve as sps

def neighborIntLoc(image,location):
    nList = []
    row = location[0]
    col = location[1]
    if((row == 0) & (col != 0) & (col != image.shape[1]-1)):                    # First row, no corners.
        nList.append([image[row,col-1], row, col-1])
        nList.append([image[row,col+1], row, col+1])
        nList.append([image[row+1,col-1], row+1, col-1])
        nList.append([image[row+1,col], row+1, col])
        nList.append([image[row+1,col+1], row+1, col+1])
    elif((col == 0) & (row != 0) & (row != image.shape[0]-1)):                  # First col, no corners.
        nList.append([image[row-1,col], row-1, col])
        nList.append([image[row-1,col+1], row-1, col+1])
        nList.append([image[row,col+1], row, col+1])
        nList.append([image[row+1,col], row+1,col])
        nList.append([image[row+1,col+1], row+1, col+1])
    elif((row == image.shape[0]-1) & (col != 0) & (col != image.shape[1]-1)):   # Last row, no corners.
        nList.append([image[row-1,col-1], row-1, col-1])
        nList.append([image[row-1,col], row-1, col])
        nList.append([image[row-1,col+1], row-1, col+1])
        nList.append([image[row,col-1], row, col-1])
        nList.append([image[row,col+1], row, col+1])
    elif((col == image.shape[1]-1) & (row != 0) & (row != image.shape[0]-1)):   # Last col, no corners.
        nList.append([image[row-1,col-1], row-1, col-1])
        nList.append([image[row-1,col], row-1, col])
        nList.append([image[row,col-1], row, col-1])
        nList.append([image[row+1,col-1], row+1, col-1])
        nList.append([image[row+1,col], row+1, col])
    elif((row == 0) & (col == 0)):                                              # Top left corner.
        nList.append([image[row,col+1], row, col+1])
        nList.append([image[row+1,col], row+1, col])
        nList.append([image[row+1,col+1], row+1, col+1])
    elif((row == 0) & (col == image.shape[1]-1)):                               # Top right corner.
        nList.append([image[row,col-1], row, col-1])
        nList.append([image[row+1,col-1], row+1, col-1])
        nList.append([image[row+1,col], row+1, col])
    elif((row == image.shape[0]-1) & (col == 0)):                               # Bottom left corner.
        nList.append([image[row-1,col], row-1, col])
        nList.append([image[row-1,col+1], row-1, col+1])
        nList.append([image[row,col+1], row, col+1])
    elif((row == image.shape[0]-1) & (col == image.shape[1]-1)):                # Bottom right corner.
        nList.append([image[row-1,col-1], row-1, col-1])
        nList.append([image[row-1,col], row-1, col])
        nList.append([image[row,col-1], row, col-1])
    else:                                                                       # Middle.
        nList.append([image[row-1,col-1], row-1, col-1])
        nList.append([image[row-1,col], row-1, col])
        nList.append([image[row-1,col+1], row-1, col+1])
        nList.append([image[row,col-1], row, col-1])
        nList.append([image[row,col+1], row, col+1])
        nList.append([image[row+1,col-1], row+1, col-1])
        nList.append([image[row+1,col], row+1, col])
        nList.append([image[row+1,col+1], row+1, col+1])
    
    # Convert to arrays.
    ints = np.array(nList)[:,0]
    locs = np.array(nList, dtype=np.integer)[:,1:3]
    
    return (ints, locs)
    
#------------------------------------------------------------------------------
    
def markMapTF(bwImage,mImage):
    # Subtract mImage from bwImage.
    abSub = abs(bwImage - mImage)
    # Sum all 3 channels.
    chSum = sum(np.rollaxis(abSub,2,0))
    # Keep only values greater than 0.01.
    result = (chSum > 0.01)
    
    return result
    
#------------------------------------------------------------------------------
        
def weightLocationData(markInt,rows,cols,markArea):
    sIdataList = []
    sIlocList = []
    # Build the weight matrix for each window.
    for row in range(rows):
        for col in range(cols):
            # Find neighbor intensity values and locations.
            nInt,nLoc = neighborIntLoc(markInt, (row,col))
            
            # Find variance of neighbor intensity values.
            window = np.append(nInt,markInt[row,col])
            var = np.var(window)
            if(var == 0):
                var = 0.000001
                
            # Create list of neighbor weights and locations.
            wlist = []
            seqC = np.ravel_multi_index(([row],[col]), (rows,cols))[0]
            if(markArea[row,col] == False):
                for i in xrange(nInt.shape[0]): # For each neighbor.
                    weight = np.exp((-(markInt[row,col] - nInt[i])**2) / (2.0*var))
                    wlist.append(weight)
                    seqN = np.ravel_multi_index(([nLoc[i][0]],[nLoc[i][1]]), (rows,cols))[0]
                    sIlocList.append(np.array([seqC,seqN]))
                    
            # Divide so sum of weights equal 1, then append weights.
            wlist = np.array(wlist)/sum(wlist)
            for i in xrange(len(wlist)):
                sIdataList.append(-wlist[i])
                
            # Append center.
            sIdataList.append(1.0)
            sIlocList.append(np.array([seqC,seqC]))
            
    # Convert lists to arrays.
    sIdata = np.array(sIdataList)
    sIloc = np.array(sIlocList)
    
    return (sIdata, sIloc)
    
#------------------------------------------------------------------------------
    
def vectorCrCb(image,size,markArea):
    # Create vectors for Cr and Cb channels.
    vCr = np.zeros(size)
    vCb = np.zeros(size)
    mIndex = np.nonzero(markArea.reshape(size))
    vCr[mIndex] = image[:,:,1].reshape(size)[mIndex]
    vCb[mIndex] = image[:,:,2].reshape(size)[mIndex]
    
    return (vCr, vCb)
    
#------------------------------------------------------------------------------
        
def colorOp(bwImage,mImage,markArea):
    # Dimensions.
    iRows = bwImage.shape[0]
    iCols = bwImage.shape[1]
    iSize = bwImage.shape[0] * bwImage.shape[1]
    
    # Convert.
    imageO_YCRCB = cv2.cvtColor(bwImage, cv2.COLOR_BGR2YCR_CB).astype(np.float64) / 255
    imageM_YCRCB = cv2.cvtColor(mImage, cv2.COLOR_BGR2YCR_CB).astype(np.float64) / 255
    
    # Separate channels.
    yO = imageO_YCRCB[:,:,0]
    yM = imageM_YCRCB[:,:,0]
    crM = imageM_YCRCB[:,:,1]
    cbM = imageM_YCRCB[:,:,2]
    
    # Combine imageO Y channel and imageM CR,CB channels.
    yCrCb = np.dstack((yO,crM,cbM))
    
    # Find neighbor weight and location.
    sIdata,sIloc = weightLocationData(yM,iRows,iCols,markArea)
    
    # Create sparse matrix.
    sMatrix = cm((sIdata, (sIloc[:,0], sIloc[:,1])), shape=(iSize,iSize))
    
    # Create vectors for Cr and Cb channels.  
    vCr,vCb = vectorCrCb(yCrCb,iSize,markArea)
    
    # Solve for Cr and Cb.
    resultY = yCrCb[:,:,0]
    resultCr = sps(sMatrix, vCr).reshape(iRows,iCols)
    resultCb = sps(sMatrix, vCb).reshape(iRows,iCols)
    
    # Combine channels and convert to BGR.
    result = np.dstack((resultY, resultCr, resultCb)).astype(np.float32)
    result = cv2.cvtColor(result, cv2.COLOR_YCR_CB2BGR)
    
    return result